#include "FtpTransaction.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <utility>

namespace mavmoos::ftp {
namespace {
constexpr std::uint8_t ack = 128;
constexpr std::uint8_t nak = 129;
constexpr std::uint8_t fail = 1;
constexpr std::uint8_t fail_errno = 2;
constexpr std::uint8_t end_of_file = 6;
constexpr std::uint8_t max_attempts = 3;
constexpr double local_retry_delay = .12;
constexpr double response_timeout = .5;
constexpr double handoff_lifetime = .25;
constexpr double max_reply_age = .25;
constexpr double request_interval = .04;
constexpr double unconfirmed_session_quiet = 3.1;

bool alphanumeric(char c)
{
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
}

std::uint16_t read16(const Payload& payload)
{
  return static_cast<std::uint16_t>(payload[0] | (static_cast<unsigned>(payload[1]) << 8));
}

std::uint32_t read32(const Payload& payload, std::size_t position)
{
  std::uint32_t value = 0;
  for (std::size_t i = 0; i < 4; ++i)
    value |= static_cast<std::uint32_t>(payload[position + i]) << (8 * i);
  return value;
}

bool sameIntent(const Intent& a, const Intent& b)
{
  return a.token == b.token && a.opcode == b.opcode && a.sequence == b.sequence &&
    a.session == b.session && a.offset == b.offset && a.size == b.size && a.path == b.path &&
    a.expires == b.expires && a.operation_expires == b.operation_expires &&
    a.cleanup_expires == b.cleanup_expires;
}

bool sessionMayExist(RemoteSession session)
{
  return session == RemoteSession::Possible || session == RemoteSession::Open;
}
}  // namespace

bool safeBasename(std::string_view name)
{
  return !name.empty() && name.size() <= max_basename &&
    alphanumeric(name.front()) &&
    std::all_of(name.begin(), name.end(), [](char c) {
      return alphanumeric(c) || c == '.' || c == '_' || c == '-';
    });
}

bool active(Phase phase)
{
  switch (phase) {
  case Phase::Listing:
  case Phase::Opening:
  case Phase::Reading:
  case Phase::VerifyingEof:
  case Phase::Closing:
    return true;
  default:
    return false;
  }
}

std::optional<Payload> encodePayload(const Intent& intent)
{
  if (intent.session != client_session || (intent.sequence & 1) != 0)
    return std::nullopt;
  switch (intent.opcode) {
  case Opcode::ListDirectory:
    if (intent.path != directory || intent.size != directory.size() ||
        intent.offset > max_list_offset)
      return std::nullopt;
    break;
  case Opcode::OpenFileRO:
    if (intent.offset != 0 || intent.size != intent.path.size() ||
        intent.path.size() <= directory.size() || intent.path.compare(0, directory.size(), directory) != 0 ||
        intent.path[directory.size()] != '/' ||
        !safeBasename(std::string_view(intent.path).substr(directory.size() + 1)))
      return std::nullopt;
    break;
  case Opcode::ReadFile:
    if (!intent.path.empty() || intent.size == 0 || intent.size > data_size ||
        intent.offset > max_file_size ||
        (intent.offset == max_file_size && intent.size != 1) ||
        (intent.offset < max_file_size && intent.size > max_file_size - intent.offset))
      return std::nullopt;
    break;
  case Opcode::TerminateSession:
    if (!intent.path.empty() || intent.size != 0 || intent.offset != 0)
      return std::nullopt;
    break;
  default:
    return std::nullopt;
  }

  Payload payload{};
  payload[0] = static_cast<std::uint8_t>(intent.sequence);
  payload[1] = static_cast<std::uint8_t>(intent.sequence >> 8);
  payload[2] = intent.session;
  payload[3] = static_cast<std::uint8_t>(intent.opcode);
  payload[4] = intent.size;
  for (std::size_t i = 0; i < 4; ++i)
    payload[8 + i] = static_cast<std::uint8_t>(intent.offset >> (8 * i));
  std::copy(intent.path.begin(), intent.path.end(), payload.begin() + header_size);
  return payload;
}

Transaction::Transaction(std::uint16_t initial_sequence)
  : m_next_sequence(static_cast<std::uint16_t>(initial_sequence & 0xfffe))
{
}

bool Transaction::clock(double now)
{
  if (!std::isfinite(now) || now < 0 || (m_high_water && now < *m_high_water)) {
    if (active(m_status.phase) || m_status.phase == Phase::Listed || m_status.phase == Phase::Complete)
      finish(Phase::Failed, "clock_discontinuity");
    m_clock_fault = true;
    return false;
  }
  m_high_water = now;
  return !m_clock_fault;
}

bool Transaction::startAllowed(double now, double deadline) const
{
  return !m_clock_fault && std::isfinite(deadline) && deadline > now &&
    deadline - now <= max_operation_seconds && now >= m_recover_at;
}

bool Transaction::beginList(double now, double deadline)
{
  return beginList(directory, now, deadline);
}

bool Transaction::beginList(std::string_view relative_directory, double now, double deadline)
{
  advance(now);
  if (relative_directory != directory || active(m_status.phase) || !startAllowed(now, deadline))
    return false;

  const bool session_uncertain = m_status.remote_session == RemoteSession::Uncertain;
  m_status = {};
  if (session_uncertain)
    m_status.remote_session = RemoteSession::Uncertain;
  m_status.phase = Phase::Listing;
  m_status.reason = "listing";
  m_status.operation_expires = deadline;
  m_listing.clear();
  m_listing_stage.clear();
  m_file.reset();
  m_list_offset = 0;
  queue(Opcode::ListDirectory, 0, static_cast<std::uint8_t>(directory.size()), directory, now);
  return true;
}

bool Transaction::beginGet(std::string_view listed_leaf, double now, double deadline)
{
  advance(now);
  if (m_status.phase != Phase::Listed || !startAllowed(now, deadline) || !safeBasename(listed_leaf))
    return false;

  const auto found = std::find_if(m_listing.begin(), m_listing.end(), [&](const DirectoryEntry& entry) {
    return entry.name == listed_leaf && entry.kind == EntryKind::File && entry.size <= max_file_size;
  });
  if (found == m_listing.end())
    return false;

  m_status.phase = Phase::Opening;
  m_status.reason = "opening";
  m_status.operation_expires = deadline;
  m_name = found->name;
  m_listed_size = found->size;
  const std::string path = std::string(directory) + "/" + m_name;
  queue(Opcode::OpenFileRO, 0, static_cast<std::uint8_t>(path.size()), path, now);
  return true;
}

void Transaction::queue(Opcode opcode, std::uint32_t offset, std::uint8_t size,
                        std::string_view path, double now)
{
  m_pending = Pending{opcode, m_next_sequence, client_session, offset, size, std::string(path),
    std::max(now, m_next_send_at), 0, std::nullopt, std::nullopt};
  // AP forms the reply sequence as request + 1 and compares using integer promotion.
  // Even request sequences avoid a request of 0xffff and keep retries replayable at wrap.
  m_next_sequence = static_cast<std::uint16_t>(m_next_sequence + 2u);
}

std::optional<Intent> Transaction::take(double now)
{
  advance(now);
  if (!m_pending || m_pending->issued || now < m_pending->ready_at)
    return std::nullopt;

  const double cutoff = m_status.phase == Phase::Closing ? m_status.cleanup_expires
                                                       : m_status.operation_expires;
  if (now >= cutoff)
    return std::nullopt;
  Pending& pending = *m_pending;
  ++pending.attempts;
  pending.issued = Intent{pending.opcode, pending.sequence, pending.session, pending.offset,
    pending.size, pending.path, ++m_next_token, std::min(cutoff, now + handoff_lifetime),
    m_status.operation_expires, m_status.cleanup_expires};
  return pending.issued;
}

bool Transaction::eligible(const Intent& intent, double now) const
{
  return !m_clock_fault && std::isfinite(now) && m_high_water && now >= *m_high_water &&
    now < intent.expires && m_pending && m_pending->issued &&
    sameIntent(intent, *m_pending->issued);
}

bool Transaction::sent(const Intent& intent, double now, bool local_accepted)
{
  if (!clock(now) || !eligible(intent, now))
    return false;
  Pending& pending = *m_pending;
  pending.issued.reset();
  if (!local_accepted) {
    pending.ready_at = now + local_retry_delay;
    return true;
  }

  if (!pending.first_sent_at)
    pending.first_sent_at = now;
  if (pending.opcode == Opcode::OpenFileRO)
    m_status.remote_session = RemoteSession::Possible;
  pending.ready_at = now + response_timeout;
  m_next_send_at = now + request_interval;
  return true;
}

bool Transaction::receive(const Reply& reply, double now)
{
  advance(now);
  if (!m_pending || !m_pending->first_sent_at || reply.source_system != fc_system ||
      reply.source_component != fc_component || reply.target_network != 0 ||
      reply.target_system != bridge_system || reply.target_component != bridge_component ||
      !std::isfinite(reply.effective_at) || !std::isfinite(reply.read_at) ||
      reply.effective_at < 0 || reply.read_at <= *m_pending->first_sent_at ||
      reply.effective_at > reply.read_at || reply.read_at > now || now - reply.effective_at > max_reply_age)
    return false;

  const Payload& p = reply.payload;
  const Pending& pending = *m_pending;
  if (read16(p) != static_cast<std::uint16_t>(pending.sequence + 1u) || p[2] != pending.session ||
      (p[3] != ack && p[3] != nak) || p[5] != static_cast<std::uint8_t>(pending.opcode) ||
      p[6] != 0 || p[7] != 0)
    return false;
  const auto offset = read32(p, 8);
  // The pinned AP ReadFile NAK leaves offset at zero. Its ACK always echoes it.
  if (offset != pending.offset && !(pending.opcode == Opcode::ReadFile && p[3] == nak && offset == 0))
    return false;

  if (p[4] > data_size || (p[3] == nak && (p[4] == 0 || p[header_size] == 0 ||
      p[4] != (p[header_size] == fail_errno ? 2 : 1)))) {
    abandon(Phase::Failed, "malformed_reply", now);
  } else if (p[3] == ack) {
    acknowledge(reply, now);
  } else {
    reject(reply, now);
  }
  return true;
}

bool Transaction::parseListPage(const Payload& payload, std::uint8_t size)
{
  const auto begin = payload.begin() + header_size;
  const auto end = begin + size;
  std::uint32_t next_offset = m_list_offset;
  for (auto start = begin; start != end;) {
    if (*start == 0) {
      // AP leaves an extra NUL after each record. Only entries count toward the offset.
      ++start;
      continue;
    }
    const auto stop = std::find(start, end, 0);
    if (stop == end || next_offset >= max_list_offset)
      return false;
    ++next_offset;
    const std::string_view record(reinterpret_cast<const char*>(&*start),
                                  static_cast<std::size_t>(stop - start));
    start = stop + 1;
    if (record == "S")
      continue;

    auto name = record.substr(1);
    EntryKind kind = EntryKind::Directory;
    std::uint32_t file_size = 0;
    if (record.front() == 'F') {
      const auto tab = name.rfind('\t');
      if (tab == std::string_view::npos)
        return false;
      const auto first = name.data() + tab + 1;
      const auto last = name.data() + name.size();
      const auto parsed = std::from_chars(first, last, file_size);
      if (parsed.ec != std::errc{} || parsed.ptr != last)
        return false;
      name = name.substr(0, tab);
      kind = EntryKind::File;
    } else if (record.front() != 'D') {
      return false;
    }

    if (name.empty())
      return false;
    if (!safeBasename(name))
      continue;
    if (std::any_of(m_listing_stage.begin(), m_listing_stage.end(), [&](const DirectoryEntry& entry) {
          return entry.name == name;
        }))
      return false;
    m_listing_stage.push_back({std::string(name), kind, file_size});
  }
  if (next_offset == m_list_offset)
    return false;
  m_list_offset = next_offset;
  return true;
}

void Transaction::queueRead(double now)
{
  const auto offset = static_cast<std::uint32_t>(m_file_stage.size());
  const auto remaining = *m_status.announced_bytes - offset;
  if (remaining == 0) {
    m_status.phase = Phase::VerifyingEof;
    m_status.reason = "verifying_eof";
  } else {
    m_status.phase = Phase::Reading;
    m_status.reason = "reading";
  }
  const auto count = remaining == 0 ? 1 : std::min<std::size_t>(data_size, remaining);
  queue(Opcode::ReadFile, offset, static_cast<std::uint8_t>(count), {}, now);
}

void Transaction::acknowledge(const Reply& reply, double now)
{
  const Payload& p = reply.payload;
  const std::uint8_t size = p[4];
  switch (m_status.phase) {
  case Phase::Listing:
    if (!parseListPage(p, size)) {
      abandon(Phase::Failed, "malformed_or_oversized_listing", now);
      return;
    }
    queue(Opcode::ListDirectory, m_list_offset, static_cast<std::uint8_t>(directory.size()), directory, now);
    return;
  case Phase::Opening:
    m_status.remote_session = RemoteSession::Open;
    if (size != sizeof(std::uint32_t)) {
      abandon(Phase::Failed, "malformed_open_ack", now);
      return;
    }
    m_status.announced_bytes = read32(p, header_size);
    if (*m_status.announced_bytes != m_listed_size || *m_status.announced_bytes > max_file_size) {
      abandon(Phase::Failed, "listed_file_size_changed", now);
      return;
    }
    m_file_stage.reserve(*m_status.announced_bytes);
    queueRead(now);
    return;
  case Phase::Reading:
    if (size == 0 || size > m_pending->size) {
      abandon(Phase::Failed, "invalid_read_size", now);
      return;
    }
    m_file_stage.insert(m_file_stage.end(), p.begin() + header_size, p.begin() + header_size + size);
    m_status.received_bytes = m_file_stage.size();
    queueRead(now);
    return;
  case Phase::VerifyingEof:
    abandon(Phase::Failed, "expected_eof", now);
    return;
  case Phase::Closing:
    if (size != 0) {
      abandon(Phase::Failed, "malformed_close_ack", now);
      return;
    }
    m_status.remote_session = RemoteSession::ConfirmedClosed;
    finish(m_after_close, m_after_close_reason);
    return;
  default:
    return;
  }
}

void Transaction::reject(const Reply& reply, double now)
{
  const Payload& p = reply.payload;
  const auto error = p[header_size];
  if (error == end_of_file && m_status.phase == Phase::Listing) {
    m_listing = std::move(m_listing_stage);
    m_status.listed_entries = m_listing.size();
    finish(Phase::Listed, "listing_complete");
    return;
  }
  if (error == end_of_file && m_status.phase == Phase::VerifyingEof) {
    close(Phase::Complete, "file_complete", now);
    return;
  }
  if (!m_status.remote_error) {
    m_status.remote_error = error;
    if (error == fail_errno)
      m_status.remote_errno = p[header_size + 1];
  }
  // The pinned AP returns generic Fail when the descriptor is already open.
  // An earlier unconfirmed close can leave our session in that state through re-listing.
  if (m_status.phase == Phase::Opening && error != fail)
    m_status.remote_session = RemoteSession::None;
  abandon(Phase::Failed, error == end_of_file ? "early_eof" : "remote_error", now);
}

void Transaction::advance(double now)
{
  if (!clock(now) || !m_pending)
    return;

  if (m_status.phase == Phase::Closing) {
    if (m_after_close == Phase::Complete && now >= m_status.operation_expires) {
      m_after_close = Phase::Expired;
      m_after_close_reason = "operation_expired";
      m_status.reason = m_after_close_reason;
      m_file_stage.clear();
    }
    if (now >= m_status.cleanup_expires) {
      abandon(Phase::Failed, "close_unconfirmed", now);
      return;
    }
  } else if (now >= m_status.operation_expires) {
    abandon(Phase::Expired, "operation_expired", now);
    return;
  }

  Pending& pending = *m_pending;
  if (pending.issued && now >= pending.issued->expires) {
    if (pending.opcode == Opcode::OpenFileRO)
      m_status.remote_session = RemoteSession::Possible;
    pending.issued.reset();
    pending.ready_at = now + local_retry_delay;
  }
  if (!pending.issued && pending.attempts >= max_attempts && now >= pending.ready_at)
    abandon(Phase::Failed, m_status.phase == Phase::Closing ? "close_unconfirmed" : "request_attempts_exhausted", now);
}

bool Transaction::cancel(double now)
{
  advance(now);
  if (!active(m_status.phase))
    return false;
  if (m_status.phase == Phase::Closing) {
    if (m_after_close != Phase::Complete)
      return false;
    m_after_close = Phase::Cancelled;
    m_after_close_reason = "cancelled";
    m_file_stage.clear();
    return true;
  }
  abandon(Phase::Cancelled, "cancelled", now);
  return true;
}

void Transaction::abandon(Phase phase, std::string_view reason, double now)
{
  if (m_status.phase == Phase::Closing) {
    if (m_after_close == Phase::Complete)
      finish(phase, reason);
    else
      finish(m_after_close, m_after_close_reason);
    return;
  }
  retirePending();
  if (sessionMayExist(m_status.remote_session)) {
    close(phase, reason, now);
  } else {
    finish(phase, reason);
  }
}

void Transaction::close(Phase final_phase, std::string_view reason, double now)
{
  m_after_close = final_phase;
  m_after_close_reason = reason;
  m_status.phase = Phase::Closing;
  m_status.reason = final_phase == Phase::Complete ? "closing" : reason;
  m_status.cleanup_expires = std::min(now, m_status.operation_expires) + cleanup_seconds;
  if (final_phase != Phase::Complete)
    m_file_stage.clear();
  if (now >= m_status.cleanup_expires) {
    finish(final_phase, reason);
    return;
  }
  queue(Opcode::TerminateSession, 0, 0, {}, now);
}

void Transaction::retirePending()
{
  if (m_pending && m_pending->opcode == Opcode::OpenFileRO && m_pending->issued)
    m_status.remote_session = RemoteSession::Possible;
  m_pending.reset();
}

void Transaction::finish(Phase phase, std::string_view reason)
{
  retirePending();
  if (sessionMayExist(m_status.remote_session)) {
    m_status.remote_session = RemoteSession::Uncertain;
    m_recover_at = *m_high_water + unconfirmed_session_quiet;
  }
  m_status.phase = phase;
  m_status.reason = reason;
  if (phase == Phase::Complete)
    m_file = File{std::move(m_name), std::move(m_file_stage)};
  else
    m_file.reset();
  m_file_stage.clear();
  m_listing_stage.clear();
}

void Transaction::reset(double now)
{
  const bool valid_now = std::isfinite(now) && now >= 0 && (!m_high_water || now >= *m_high_water);
  if (valid_now)
    m_high_water = now;
  m_clock_fault = !valid_now;
  finish(Phase::Idle, "reset");
  const bool uncertain = m_status.remote_session == RemoteSession::Uncertain;
  m_status = {};
  m_status.reason = "reset";
  if (uncertain)
    m_status.remote_session = RemoteSession::Uncertain;
  m_listing.clear();
}

std::optional<File> Transaction::takeFile()
{
  return std::exchange(m_file, std::nullopt);
}

}  // namespace mavmoos::ftp
