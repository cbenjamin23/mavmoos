#include "FtpTransaction.h"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <string_view>
#include <utility>

namespace {
using namespace mavmoos::ftp;

void check(bool condition, const char* expression, int line)
{
  if (!condition) {
    std::cerr << "line " << line << " failed: " << expression << '\n';
    std::exit(1);
  }
}

#define CHECK(expression) check(static_cast<bool>(expression), #expression, __LINE__)

std::vector<std::uint8_t> bytes(std::initializer_list<std::uint8_t> data)
{
  return {data};
}

std::vector<std::uint8_t> page(std::initializer_list<std::string_view> records)
{
  std::vector<std::uint8_t> data;
  for (const auto& record : records) {
    data.insert(data.end(), record.begin(), record.end());
    data.push_back(0);
    data.push_back(0);
  }
  return data;
}

std::vector<std::uint8_t> little32(std::uint32_t value)
{
  return bytes({static_cast<std::uint8_t>(value), static_cast<std::uint8_t>(value >> 8),
    static_cast<std::uint8_t>(value >> 16), static_cast<std::uint8_t>(value >> 24)});
}

Reply makeReply(const Intent& request, std::uint8_t opcode,
                const std::vector<std::uint8_t>& data, std::uint32_t offset, double dispatch)
{
  CHECK(data.size() <= data_size);
  Reply response{1, 1, 0, 245, 191, {}, dispatch - .015, dispatch + .01};
  const auto sequence = static_cast<std::uint16_t>(request.sequence + 1);
  response.payload[0] = static_cast<std::uint8_t>(sequence);
  response.payload[1] = static_cast<std::uint8_t>(sequence >> 8);
  response.payload[2] = request.session;
  response.payload[3] = opcode;
  response.payload[4] = static_cast<std::uint8_t>(data.size());
  response.payload[5] = static_cast<std::uint8_t>(request.opcode);
  const auto raw_offset = little32(offset);
  std::copy(raw_offset.begin(), raw_offset.end(), response.payload.begin() + 8);
  std::copy(data.begin(), data.end(), response.payload.begin() + 12);
  return response;
}

class Rig {
public:
  explicit Rig(std::uint16_t seed = 0) : transaction(seed) {}

  Intent issue(Opcode opcode, double wait = .05)
  {
    now += wait;
    const auto intent = transaction.take(now);
    CHECK(intent);
    CHECK(intent->opcode == opcode);
    CHECK(transaction.eligible(*intent, now));
    CHECK(encodePayload(*intent));
    return *intent;
  }

  Intent dispatch(Opcode opcode, double wait = .05)
  {
    const auto intent = issue(opcode, wait);
    CHECK(transaction.sent(intent, now, true));
    CHECK(!transaction.eligible(intent, now));
    return intent;
  }

  bool deliver(const Reply& response)
  {
    now = std::max(now, response.read_at) + .001;
    return transaction.receive(response, now);
  }

  void ack(const Intent& request, const std::vector<std::uint8_t>& data = {})
  {
    CHECK(deliver(makeReply(request, 128, data, request.offset, now)));
  }

  void nak(const Intent& request, std::uint8_t error = 6)
  {
    CHECK(deliver(makeReply(request, 129, bytes({error}),
      request.opcode == Opcode::ReadFile ? 0 : request.offset, now)));
  }

  void listFile(std::uint32_t size)
  {
    CHECK(transaction.beginList(now, now + 10));
    const auto first = dispatch(Opcode::ListDirectory);
    const auto entry = "Frecord.bin\t" + std::to_string(size);
    ack(first, page({entry}));
    const auto last = dispatch(Opcode::ListDirectory);
    CHECK(last.offset == 1);
    nak(last);
    CHECK(transaction.status().phase == Phase::Listed);
    CHECK(transaction.listing().size() == 1);
  }

  Intent startGet(double duration = 10)
  {
    CHECK(transaction.beginGet("record.bin", now, now + duration));
    return dispatch(Opcode::OpenFileRO);
  }

  Transaction transaction;
  double now = 10;
};

struct CachedApPeer {
  explicit CachedApPeer(std::vector<std::uint8_t> file) : contents(std::move(file)) {}

  Reply accept(const Intent& request, double now)
  {
    const auto raw = encodePayload(request);
    CHECK(raw);
    if (last_request && last_reply && (*last_request)[0] == (*raw)[0] &&
        (*last_request)[1] == (*raw)[1] && (*last_request)[2] == (*raw)[2]) {
      CHECK(*raw == *last_request);
      ++replayed;
      auto reply = *last_reply;
      reply.effective_at = now - .01;
      reply.read_at = now + .01;
      return reply;
    }

    std::vector<std::uint8_t> data;
    std::uint32_t offset = request.offset;
    std::uint8_t opcode = 128;
    switch (request.opcode) {
    case Opcode::ListDirectory:
      CHECK(!opened);
      if (offset == 0) {
        const auto file = "Frecord.bin\t" + std::to_string(contents.size());
        data = page({"D.", "D..", "Dnest", "S", file});
      } else if (offset == 5) {
        data = page({"Fhuge.bin\t65537", "F../hide\t12"});
      } else {
        CHECK(offset == 7);
        opcode = 129;
        data = bytes({6});
      }
      break;
    case Opcode::OpenFileRO:
      CHECK(!opened);
      CHECK(request.path == "mavmoos-ftp/record.bin");
      ++opens;
      opened = true;
      data = little32(static_cast<std::uint32_t>(contents.size()));
      break;
    case Opcode::ReadFile:
      CHECK(opened);
      CHECK(offset == next_offset);
      if (offset == contents.size()) {
        CHECK(request.size == 1);
        opcode = 129;
        offset = 0;
        data = bytes({6});
      } else {
        const auto count = std::min<unsigned>(request.size, offset == 0 ? 17 : 239);
        CHECK(count <= contents.size() - offset);
        data.assign(contents.begin() + offset, contents.begin() + offset + count);
        next_offset += count;
      }
      break;
    case Opcode::TerminateSession:
      CHECK(opened);
      ++closes;
      opened = false;
      break;
    }
    last_request = raw;
    last_reply = makeReply(request, opcode, data, offset, now);
    return *last_reply;
  }

  std::vector<std::uint8_t> contents;
  std::optional<Payload> last_request;
  std::optional<Reply> last_reply;
  std::uint32_t next_offset = 0;
  unsigned replayed = 0;
  unsigned opens = 0;
  unsigned closes = 0;
  bool opened = false;
};

void codecAndNames()
{
  CHECK(safeBasename("A-_0.name"));
  CHECK(!safeBasename(""));
  CHECK(!safeBasename(".."));
  CHECK(!safeBasename(".hidden"));
  CHECK(!safeBasename("a/b"));
  CHECK(!safeBasename("a\\b"));
  CHECK(!safeBasename("a b"));
  CHECK(!safeBasename("a\n"));
  CHECK(!safeBasename(std::string_view("a\0b", 3)));
  CHECK(safeBasename(std::string(max_basename, 'a')));
  CHECK(!safeBasename(std::string(max_basename + 1, 'a')));

  Rig rig(0xffff);
  CHECK(!rig.transaction.beginList("@SYS", rig.now, rig.now + 1));
  CHECK(!rig.transaction.beginList("/mavmoos-ftp", rig.now, rig.now + 1));
  CHECK(!rig.transaction.beginList(rig.now, rig.now + 60.01));
  CHECK(!rig.transaction.beginList(rig.now, std::numeric_limits<double>::infinity()));
  CHECK(rig.transaction.beginList(rig.now, rig.now + 60));
  auto request = rig.dispatch(Opcode::ListDirectory);
  CHECK(request.sequence == 0xfffe);
  auto payload = *encodePayload(request);
  CHECK(payload[0] == 0xfe && payload[1] == 0xff && payload[2] == 1);
  CHECK(payload[3] == 3 && payload[4] == directory.size());
  CHECK(std::equal(directory.begin(), directory.end(), payload.begin() + 12));
  CHECK(std::all_of(payload.begin() + 12 + directory.size(), payload.end(), [](auto byte) { return byte == 0; }));
  request.sequence = 0xffff;
  CHECK(!encodePayload(request));
  request.sequence = 0;
  request.offset = max_list_offset + 1;
  CHECK(!encodePayload(request));
  request.offset = 0;
  request.session = 0;
  CHECK(!encodePayload(request));
  request.session = 1;
  request.opcode = static_cast<Opcode>(2);
  CHECK(!encodePayload(request));
  request.opcode = static_cast<Opcode>(7);
  CHECK(!encodePayload(request));
  request.opcode = static_cast<Opcode>(15);
  CHECK(!encodePayload(request));
  request.opcode = Opcode::OpenFileRO;
  request.path = "mavmoos-ftp/../escape";
  request.size = static_cast<std::uint8_t>(request.path.size());
  CHECK(!encodePayload(request));
  request.path = "mavmoos-ftp/" + std::string(max_basename, 'a');
  request.size = static_cast<std::uint8_t>(request.path.size());
  payload = *encodePayload(request);
  CHECK(payload.back() == 0);
  request.opcode = Opcode::ReadFile;
  request.path.clear();
  request.offset = static_cast<std::uint32_t>(max_file_size);
  request.size = 1;
  payload = *encodePayload(request);
  CHECK(payload[8] == 0 && payload[9] == 0 && payload[10] == 1 && payload[11] == 0);
  request.size = 2;
  CHECK(!encodePayload(request));
  request.size = 1;
  ++request.offset;
  CHECK(!encodePayload(request));
  request.opcode = Opcode::TerminateSession;
  request.offset = 0;
  CHECK(!encodePayload(request));
}

void completeWithCachedRetries()
{
  std::vector<std::uint8_t> contents(512);
  for (std::size_t i = 0; i < contents.size() - 2; ++i)
    contents[i] = static_cast<std::uint8_t>(i);
  Rig rig(0xfffe);
  CachedApPeer peer(contents);
  CHECK(!rig.transaction.beginGet("record.bin", rig.now, rig.now + 1));
  CHECK(rig.transaction.beginList(rig.now, rig.now + 8));
  auto first = rig.dispatch(Opcode::ListDirectory);
  CHECK(first.sequence == 0xfffe);
  CHECK(rig.deliver(peer.accept(first, rig.now)));
  CHECK(rig.transaction.listing().empty());
  auto second = rig.dispatch(Opcode::ListDirectory);
  CHECK(second.sequence == 0 && second.offset == 5);
  CHECK(rig.deliver(peer.accept(second, rig.now)));
  auto last = rig.dispatch(Opcode::ListDirectory);
  CHECK(last.offset == 7);
  CHECK(rig.deliver(peer.accept(last, rig.now)));
  const auto& listing = rig.transaction.listing();
  CHECK(listing.size() == 3 && listing[0].kind == EntryKind::Directory && listing[1].size == 512);
  CHECK(listing[2].name == "huge.bin" && listing[2].size == 65537);
  CHECK(!rig.transaction.beginGet("nest", rig.now, rig.now + 1));
  CHECK(!rig.transaction.beginGet("huge.bin", rig.now, rig.now + 1));
  CHECK(!rig.transaction.beginGet("../hide", rig.now, rig.now + 1));
  CHECK(!rig.transaction.beginGet("Record.bin", rig.now, rig.now + 1));

  const auto open = rig.startGet();
  CHECK(open.sequence == 4 && open.session == last.session);
  const auto lost_open = peer.accept(open, rig.now);
  const auto retry_open = rig.dispatch(Opcode::OpenFileRO, .51);
  CHECK(retry_open.token != open.token && encodePayload(retry_open) == encodePayload(open));
  CHECK(rig.deliver(peer.accept(retry_open, rig.now)));
  CHECK(peer.opens == 1 && peer.replayed == 1);
  CHECK(rig.transaction.status().remote_session == RemoteSession::Open);
  CHECK(rig.transaction.status().announced_bytes == contents.size());

  const auto first_read = rig.dispatch(Opcode::ReadFile);
  CHECK(first_read.size == 239 && first_read.offset == 0);
  const auto delayed_read = peer.accept(first_read, rig.now);
  auto pending_retry = rig.issue(Opcode::ReadFile, .51);
  CHECK(pending_retry.token != first_read.token && encodePayload(pending_retry) == encodePayload(first_read));
  auto delayed_fresh = delayed_read;
  delayed_fresh.effective_at = rig.now - .01;
  delayed_fresh.read_at = rig.now + .01;
  CHECK(rig.deliver(delayed_fresh));
  CHECK(!rig.transaction.eligible(pending_retry, rig.now));
  CHECK(!rig.transaction.sent(pending_retry, rig.now, true));
  CHECK(rig.transaction.status().received_bytes == 17);

  for (const auto expected_offset : {17u, 256u, 495u}) {
    const auto read = rig.dispatch(Opcode::ReadFile);
    CHECK(read.offset == expected_offset);
    auto old = delayed_read;
    old.effective_at = rig.now - .01;
    old.read_at = rig.now + .01;
    CHECK(!rig.deliver(old));
    CHECK(rig.deliver(peer.accept(read, rig.now)));
    CHECK(!rig.transaction.takeFile());
  }
  CHECK(rig.transaction.status().phase == Phase::VerifyingEof);
  auto eof = rig.dispatch(Opcode::ReadFile);
  CHECK(eof.offset == 512 && eof.size == 1);
  CHECK(rig.deliver(peer.accept(eof, rig.now)));
  CHECK(rig.transaction.status().phase == Phase::Closing);
  CHECK(!rig.transaction.takeFile());
  auto close = rig.dispatch(Opcode::TerminateSession);
  auto lost_close = peer.accept(close, rig.now);
  auto retry_close = rig.dispatch(Opcode::TerminateSession, .51);
  CHECK(encodePayload(retry_close) == encodePayload(close));
  CHECK(rig.deliver(peer.accept(retry_close, rig.now)));
  CHECK(peer.closes == 1 && peer.replayed == 2 && !peer.opened);
  CHECK(rig.transaction.status().phase == Phase::Complete);
  CHECK(rig.transaction.status().remote_session == RemoteSession::ConfirmedClosed);
  const auto file = rig.transaction.takeFile();
  CHECK(file && file->name == "record.bin" && file->bytes == contents);
  CHECK(!rig.transaction.takeFile());
  CHECK(!rig.transaction.beginGet("record.bin", rig.now, rig.now + 1));
  CHECK(!rig.deliver(lost_open));
  CHECK(!rig.deliver(lost_close));
  CHECK(rig.transaction.beginList(rig.now, rig.now + 3));
  auto repeated_list = rig.dispatch(Opcode::ListDirectory);
  CHECK(repeated_list.sequence == retry_close.sequence + 2 && repeated_list.session == client_session);
}

void rejectMismatchedReplies()
{
  Rig rig;
  CHECK(rig.transaction.beginList(rig.now, rig.now + 5));
  const auto local_drop = rig.issue(Opcode::ListDirectory);
  const auto unsent_reply = makeReply(local_drop, 129, bytes({6}), 0, rig.now);
  CHECK(!rig.deliver(unsent_reply));
  auto wrong_intent = local_drop;
  ++wrong_intent.offset;
  CHECK(!rig.transaction.eligible(wrong_intent, rig.now));
  CHECK(!rig.transaction.sent(wrong_intent, rig.now, true));
  CHECK(rig.transaction.sent(local_drop, rig.now, false));
  CHECK(!rig.deliver(unsent_reply));
  const auto request = rig.dispatch(Opcode::ListDirectory, .13);
  CHECK(encodePayload(local_drop) == encodePayload(request));
  const auto valid = makeReply(request, 129, bytes({6}), 0, rig.now);
  for (auto member : {&Reply::source_system, &Reply::source_component,
       &Reply::target_network, &Reply::target_system, &Reply::target_component}) {
    auto wrong = valid;
    ++(wrong.*member);
    CHECK(!rig.deliver(wrong));
  }
  for (auto index : {0u, 2u, 3u, 5u, 6u, 7u, 8u}) {
    auto wrong = valid;
    wrong.payload[index] ^= 1;
    if (index == 3)
      wrong.payload[index] = 1;
    CHECK(!rig.deliver(wrong));
  }
  auto wrong = valid;
  wrong.effective_at -= .4;
  CHECK(!rig.deliver(wrong));
  wrong = valid;
  wrong.read_at = valid.effective_at;
  CHECK(!rig.deliver(wrong));
  wrong = valid;
  wrong.effective_at = wrong.read_at + .001;
  CHECK(!rig.deliver(wrong));
  wrong = valid;
  wrong.effective_at = std::numeric_limits<double>::quiet_NaN();
  CHECK(!rig.deliver(wrong));
  wrong = valid;
  wrong.read_at = rig.now + .1;
  CHECK(!rig.transaction.receive(wrong, rig.now));
  // The queue-empty floor can predate send. An actual post-send read is mandatory.
  CHECK(valid.effective_at < valid.read_at);
  CHECK(rig.deliver(valid));
  CHECK(rig.transaction.status().phase == Phase::Listed && rig.transaction.listing().empty());
}

void listingFailures()
{
  for (const auto& malformed : {std::vector<std::uint8_t>{}, bytes({'D','x'}),
       page({"Smore"}), page({"D"}), page({"Fname"}), page({"Fname\t-1"}),
       page({"Fname\t4294967296"}), page({"Fname\t1", "Dname"}), page({"Xunknown"})}) {
    Rig rig;
    CHECK(rig.transaction.beginList(rig.now, rig.now + 5));
    const auto request = rig.dispatch(Opcode::ListDirectory);
    rig.ack(request, malformed);
    CHECK(rig.transaction.status().phase == Phase::Failed);
    CHECK(rig.transaction.listing().empty());
    CHECK(!rig.transaction.take(rig.now));
  }
  for (const bool eof : {false, true}) {
    Rig rig;
    CHECK(rig.transaction.beginList(rig.now, rig.now + 5));
    std::vector<std::uint8_t> skip(238);
    for (std::size_t i = 0; i < skip.size(); i += 2)
      skip[i] = 'S';
    auto request = rig.dispatch(Opcode::ListDirectory);
    rig.ack(request, skip);
    CHECK(rig.transaction.status().phase == Phase::Listing);
    request = rig.dispatch(Opcode::ListDirectory);
    CHECK(request.offset == 119);
    rig.ack(request, page({"S", "S", "S", "S", "S", "S", "S", "S"}));
    request = rig.dispatch(Opcode::ListDirectory);
    CHECK(request.offset == max_list_offset);
    if (eof)
      rig.nak(request);
    else
      rig.ack(request, page({"Fone_more\t4"}));
    CHECK(rig.transaction.status().phase == (eof ? Phase::Listed : Phase::Failed));
    CHECK(rig.transaction.listing().empty());
  }
}

void cancellationAndHardExpiry()
{
  {
    Rig rig;
    CHECK(rig.transaction.beginList(rig.now, rig.now + 1));
    const auto list = rig.issue(Opcode::ListDirectory);
    CHECK(rig.transaction.cancel(rig.now));
    CHECK(rig.transaction.status().phase == Phase::Cancelled);
    CHECK(!rig.transaction.eligible(list, rig.now));
    CHECK(!rig.transaction.take(rig.now + .1));
  }
  {
    Rig rig;
    rig.listFile(1);
    CHECK(rig.transaction.beginGet("record.bin", rig.now, rig.now + 5));
    const auto open = rig.issue(Opcode::OpenFileRO);
    CHECK(rig.transaction.sent(open, rig.now, false));
    CHECK(rig.transaction.cancel(rig.now));
    CHECK(rig.transaction.status().phase == Phase::Cancelled);
    CHECK(rig.transaction.status().remote_session == RemoteSession::None);
    CHECK(!rig.transaction.take(rig.now));
  }
  for (const bool accepted : {false, true}) {
    Rig rig;
    rig.listFile(1);
    CHECK(rig.transaction.beginGet("record.bin", rig.now, rig.now + 5));
    const auto open = rig.issue(Opcode::OpenFileRO);
    const auto late_open = makeReply(open, 128, little32(1), 0, rig.now);
    if (accepted)
      CHECK(rig.transaction.sent(open, rig.now, true));
    CHECK(rig.transaction.cancel(rig.now));
    CHECK(rig.transaction.status().phase == Phase::Closing);
    CHECK(!rig.transaction.eligible(open, rig.now));
    CHECK(!rig.deliver(late_open));
    const auto close = rig.dispatch(Opcode::TerminateSession);
    CHECK(close.sequence == open.sequence + 2 && close.session == open.session);
    rig.ack(close);
    CHECK(rig.transaction.status().phase == Phase::Cancelled);
    CHECK(rig.transaction.status().remote_session == RemoteSession::ConfirmedClosed);
    CHECK(!rig.transaction.takeFile());
  }
  {
    Rig rig;
    rig.listFile(1);
    const auto open = rig.startGet(.2);
    rig.now = open.operation_expires;
    rig.transaction.advance(rig.now);
    CHECK(rig.transaction.status().phase == Phase::Closing);
    CHECK(rig.transaction.status().reason == "operation_expired");
    const auto close = rig.dispatch(Opcode::TerminateSession);
    CHECK(close.operation_expires == open.operation_expires);
    CHECK(close.cleanup_expires == open.operation_expires + cleanup_seconds);
    CHECK(close.expires > close.operation_expires);
    rig.ack(close);
    CHECK(rig.transaction.status().phase == Phase::Expired);
    CHECK(rig.transaction.status().remote_session == RemoteSession::ConfirmedClosed);
  }
  {
    Rig rig;
    rig.listFile(1);
    const auto open = rig.startGet(.2);
    rig.now = open.operation_expires + cleanup_seconds;
    CHECK(!rig.transaction.take(rig.now));
    CHECK(rig.transaction.status().phase == Phase::Expired);
    CHECK(rig.transaction.status().remote_session == RemoteSession::Uncertain);
    CHECK(!rig.transaction.beginList(rig.now + 2, rig.now + 8));
    rig.now += 3.11;
    // AP does not close a retained same-session descriptor when processing LIST.
    rig.listFile(1);
    CHECK(rig.transaction.status().remote_session == RemoteSession::Uncertain);
    const auto reopening = rig.startGet();
    rig.nak(reopening, 1);
    CHECK(rig.transaction.status().phase == Phase::Closing);
    const auto recovery_close = rig.dispatch(Opcode::TerminateSession);
    CHECK(recovery_close.session == open.session);
    rig.ack(recovery_close);
    CHECK(rig.transaction.status().phase == Phase::Failed);
    CHECK(rig.transaction.status().remote_session == RemoteSession::ConfirmedClosed);
    CHECK(rig.transaction.status().remote_error == 1);
    CHECK(!rig.transaction.takeFile());
  }
}

void finalFilePublicationGuards()
{
  for (const auto failure : {"changed", "zero", "oversize", "early_eof", "extra", "expired", "cancel", "close_nak",
       "close_shape", "close_lost"}) {
    Rig rig;
    rig.listFile(1);
    const std::string_view fault = failure;
    const auto open = rig.startGet(fault == "expired" ? .3 : 10);
    rig.ack(open, little32(fault == "changed" ? 2 : 1));
    if (fault != "changed") {
      const auto read = rig.dispatch(Opcode::ReadFile);
      if (fault == "zero")
        rig.ack(read);
      else if (fault == "oversize")
        rig.ack(read, bytes({7, 8}));
      else if (fault == "early_eof")
        rig.nak(read);
      else {
        rig.ack(read, bytes({0}));
        const auto eof = rig.dispatch(Opcode::ReadFile);
        if (fault == "extra")
          rig.ack(eof, bytes({9}));
        else
          rig.nak(eof);
      }
    }
    CHECK(rig.transaction.status().phase == Phase::Closing);
    CHECK(!rig.transaction.takeFile());
    const auto close = rig.dispatch(Opcode::TerminateSession);
    if (fault == "expired") {
      // A close sent near EOF whose only ACK arrives after the GET cutoff must withhold bytes.
      rig.now = open.operation_expires;
      auto late_close = makeReply(close, 128, {}, 0, rig.now);
      CHECK(rig.deliver(late_close));
      CHECK(rig.transaction.status().phase == Phase::Expired);
      CHECK(rig.transaction.status().remote_session == RemoteSession::ConfirmedClosed);
    } else if (fault == "cancel") {
      CHECK(rig.transaction.cancel(rig.now));
      rig.ack(close);
      CHECK(rig.transaction.status().phase == Phase::Cancelled);
    } else if (fault == "close_nak") {
      rig.nak(close, 4);
      CHECK(rig.transaction.status().phase == Phase::Failed);
      CHECK(rig.transaction.status().remote_session == RemoteSession::Uncertain);
    } else if (fault == "close_shape") {
      rig.ack(close, bytes({1}));
      CHECK(rig.transaction.status().phase == Phase::Failed);
      CHECK(rig.transaction.status().remote_session == RemoteSession::Uncertain);
    } else if (fault == "close_lost") {
      for (int i = 0; i < 2; ++i) {
        const auto retry = rig.dispatch(Opcode::TerminateSession, .51);
        CHECK(retry.token != close.token && encodePayload(retry) == encodePayload(close));
      }
      CHECK(!rig.transaction.take(rig.now + .51));
      CHECK(rig.transaction.status().phase == Phase::Failed);
      CHECK(rig.transaction.status().reason == "close_unconfirmed");
    } else {
      rig.ack(close);
      CHECK(rig.transaction.status().phase == Phase::Failed);
      CHECK(rig.transaction.status().remote_session == RemoteSession::ConfirmedClosed);
    }
    CHECK(!rig.transaction.takeFile());
  }
}

void boundedWholeFiles()
{
  for (const auto count : {std::size_t{0}, max_file_size}) {
    Rig rig;
    std::vector<std::uint8_t> file(count);
    for (std::size_t i = 0; i < count; ++i)
      file[i] = static_cast<std::uint8_t>(i ^ (i >> 8));
    rig.listFile(static_cast<std::uint32_t>(count));
    const auto open = rig.startGet(60);
    rig.ack(open, little32(static_cast<std::uint32_t>(count)));
    std::size_t offset = 0;
    while (offset < count) {
      const auto read = rig.dispatch(Opcode::ReadFile);
      CHECK(read.offset == offset);
      CHECK(read.size == std::min(data_size, count - offset));
      const auto begin = file.begin() + static_cast<std::ptrdiff_t>(offset);
      rig.ack(read, {begin, begin + read.size});
      offset += read.size;
    }
    const auto eof = rig.dispatch(Opcode::ReadFile);
    CHECK(eof.offset == count && eof.size == 1);
    rig.nak(eof);
    const auto close = rig.dispatch(Opcode::TerminateSession);
    rig.ack(close);
    CHECK(rig.transaction.status().received_bytes == count);
    CHECK(rig.transaction.status().phase == Phase::Complete);
    CHECK(rig.now < open.operation_expires);
    CHECK(rig.transaction.takeFile()->bytes == file);
  }
}

void errorAndReset()
{
  {
    Rig rig;
    rig.listFile(1);
    const auto open = rig.startGet();
    CHECK(rig.deliver(makeReply(open, 129, bytes({2, 13}), 0, rig.now)));
    CHECK(rig.transaction.status().phase == Phase::Failed);
    CHECK(rig.transaction.status().remote_error == 2);
    CHECK(rig.transaction.status().remote_errno == 13);
    CHECK(rig.transaction.status().remote_session == RemoteSession::None);
    CHECK(!rig.transaction.take(rig.now));
  }
  {
    Rig rig;
    rig.listFile(1);
    const auto open = rig.startGet();
    rig.ack(open, little32(1));
    const auto issued = rig.issue(Opcode::ReadFile);
    rig.transaction.advance(rig.now - .01);
    CHECK(rig.transaction.status().phase == Phase::Failed);
    CHECK(rig.transaction.status().reason == "clock_discontinuity");
    CHECK(rig.transaction.status().remote_session == RemoteSession::Uncertain);
    CHECK(!rig.transaction.eligible(issued, rig.now));
    CHECK(!rig.transaction.beginList(rig.now + 4, rig.now + 10));
    rig.now += 5;
    rig.transaction.reset(rig.now);
    CHECK(rig.transaction.status().phase == Phase::Idle);
    CHECK(rig.transaction.status().remote_session == RemoteSession::Uncertain);
    CHECK(rig.transaction.beginList(rig.now, rig.now + 5));
    const auto next = rig.dispatch(Opcode::ListDirectory);
    CHECK(next.sequence == issued.sequence + 2);
    CHECK(!rig.transaction.sent(issued, rig.now, true));
    CHECK(!rig.transaction.takeFile());
  }
  {
    Rig rig;
    CHECK(rig.transaction.beginList(rig.now, rig.now + 5));
    auto first = rig.issue(Opcode::ListDirectory);
    CHECK(rig.transaction.sent(first, rig.now, false));
    auto second = rig.issue(Opcode::ListDirectory, .13);
    CHECK(rig.transaction.sent(second, rig.now, false));
    auto third = rig.issue(Opcode::ListDirectory, .13);
    CHECK(rig.transaction.sent(third, rig.now, false));
    CHECK(encodePayload(first) == encodePayload(second));
    CHECK(encodePayload(second) == encodePayload(third));
    CHECK(!rig.transaction.take(rig.now + .13));
    CHECK(rig.transaction.status().phase == Phase::Failed);
  }
}
}  // namespace

int main()
{
  codecAndNames();
  completeWithCachedRetries();
  rejectMismatchedReplies();
  listingFailures();
  cancellationAndHardExpiry();
  finalFilePublicationGuards();
  boundedWholeFiles();
  errorAndReset();
  std::cout << "FTP transaction state, AP cache, bounded file, expiry and correlation checks passed\n";
}
