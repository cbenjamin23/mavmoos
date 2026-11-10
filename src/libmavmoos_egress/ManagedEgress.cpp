/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ManagedEgress.cpp                               */
/*    DATE: October 29, 2026                                */
/************************************************************/
#include "ManagedEgress.h"
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace mavmoos::egress {
const char* name(Reason v)
{
  switch(v) {
#define REASON(x) case Reason::x: return #x
  REASON(Ok); REASON(Missing); REASON(InvalidValue); REASON(InvalidTiming); REASON(Expired);
  REASON(Guard); REASON(HomeMismatch); REASON(ScopeMismatch); REASON(Permission);
  REASON(DuplicateId); REASON(ResultPressure); REASON(QueueFull); REASON(ResponseUsed);
  REASON(Closed); REASON(ClockFault); REASON(RequiresFence); REASON(Reconciliation);
#undef REASON
  }
  return "Unknown";
}
const char* name(Outcome v)
{
  switch(v) {
#define OUTCOME(x) case Outcome::x: return #x
  OUTCOME(Replaced); OUTCOME(Cancelled); OUTCOME(CancelledOverload); OUTCOME(Expired);
  OUTCOME(GateRejected); OUTCOME(Accepted); OUTCOME(WouldBlock); OUTCOME(IoError);
  OUTCOME(ShortDatagram); OUTCOME(Closed); OUTCOME(ClockFault);
#undef OUTCOME
  }
  return "Unknown";
}
namespace {
template<class T> bool progresses(const Sample<T>& old, const Sample<T>& next)
{
  if(!next.value && next.window.receipt == 0) return true; // explicit unavailable state
  if(next.window.receipt < old.window.receipt) return false;
  if(next.window.receipt == old.window.receipt)
    return next.window == old.window && next.value == old.value;
  return next.window.acquired >= old.window.acquired;
}
template<class T> void retainUnavailableWindow(Sample<T>& next, const Sample<T>& old)
{
  if(!next.value && next.window.receipt == 0) next.window = old.window;
}
}
ManagedEgress::Call::Call(ManagedEgress& owner) : engine(owner)
{
  engine.checkAccess();
  engine.m_busy = true;
}
ManagedEgress::Call::~Call() { engine.m_busy = false; }
void ManagedEgress::checkAccess() const
{
  // Check immutable owner first: a foreign thread never reads the mutable busy flag.
  if(std::this_thread::get_id() != m_owner)
    throw std::logic_error("ManagedEgress requires its construction thread");
  if(m_busy) throw std::logic_error("ManagedEgress forbids reentrant calls");
}
ManagedEgress::ManagedEgress(Binding binding, Envelope envelope, Clock& clock,
                             std::unique_ptr<DatagramIo> io)
  : m_binding(binding), m_envelope(envelope), m_clock(clock), m_io(std::move(io)),
    m_owner(std::this_thread::get_id())
{
  if(!m_io || !validEnvelope(envelope) || !binding.connection || !binding.vehicle ||
     !binding.source_system || !binding.source_component ||
     !binding.target_system || !binding.target_component)
    throw std::invalid_argument("explicit binding, envelope and I/O required");
  m_scope.connection = binding.connection; m_scope.vehicle = binding.vehicle;
  m_started = m_last_now = m_raw_now = m_clock.now();
  if(m_started < 0 || m_started > std::numeric_limits<Tick>::max() - 10 * second)
    throw std::invalid_argument("invalid monotonic clock range");
}
Tick ManagedEgress::now()
{
  const Tick at = m_clock.now();
  m_raw_now = at;
  if(at < m_last_now || at > std::numeric_limits<Tick>::max() - 10 * second) {
    m_clock_fault = true;
    return m_last_now;
  }
  m_last_now = at;
  return at;
}
std::size_t ManagedEgress::pendingCount() const
{
  return m_probe_count + (m_navigation ? 1 : 0) + (m_hold ? 1 : 0) + (m_active ? 1 : 0);
}
bool ManagedEgress::updateVehicle(const VehicleState& state)
{
  Call call(*this);
  now();
  if(m_closed || m_clock_fault || state.revision <= m_vehicle.revision ||
     !progresses(m_vehicle.flight, state.flight) || !progresses(m_vehicle.pose, state.pose) ||
     !progresses(m_vehicle.landed, state.landed) || !progresses(m_vehicle.global, state.global) ||
     !progresses(m_vehicle.estimator_eligible, state.estimator_eligible))
    return false;
  auto next = state;
  // Availability loss must not erase the receipt high-water mark and permit cached renewal.
  retainUnavailableWindow(next.flight, m_vehicle.flight);
  retainUnavailableWindow(next.landed, m_vehicle.landed);
  retainUnavailableWindow(next.pose, m_vehicle.pose);
  retainUnavailableWindow(next.global, m_vehicle.global);
  retainUnavailableWindow(next.estimator_eligible, m_vehicle.estimator_eligible);
  m_vehicle = next;
  return true;
}
Reason ManagedEgress::enable(const Enable& request)
{
  Call call(*this);
  const Tick at = now();
  if(m_closed || !m_io->isOpen()) return Reason::Closed;
  if(m_clock_fault) return Reason::ClockFault;
  if(m_scope.generation && (!m_revoked || !m_fence.is_applied || m_fence.scope != m_scope))
    return Reason::RequiresFence;
  if(m_scope.generation && request.reconciled_fence != m_fence.id)
    return Reason::Reconciliation;
  if(request.scope.connection != m_binding.connection || request.scope.vehicle != m_binding.vehicle ||
     request.scope.generation <= m_scope.generation)
    return Reason::ScopeMismatch;
  if(request.nav_token <= m_last_token || request.probe_token <= request.nav_token)
    return Reason::Permission;
  if(request.expires <= at) return Reason::Expired;
  const auto eligible = guard(m_vehicle, m_envelope, request.scope.home, at);
  if(eligible.reason != Reason::Ok) return eligible.reason;
  if(m_hold) {
    finish(*m_hold, Outcome::Cancelled, Reason::ScopeMismatch, at);
    m_hold.reset();
  }
  for(std::size_t i = 0; i < m_probe_count; ++i)
    finish(m_probes[i], Outcome::Cancelled, Reason::ScopeMismatch, at);
  m_probe_count = 0;
  m_scope = request.scope; m_enable = request; m_boundary = at;
  m_last_token = request.probe_token;
  m_response_granted = m_response_attempted = false;
  m_revoked = false; m_all_revoked = false;
  m_scope_accepted = 0; m_last_accepted = 0; m_scope_residual = false;
  m_nav_streak = 0;
  return Reason::Ok;
}
Reason ManagedEgress::grantResponse(const ResponseGrant& grant)
{
  Call call(*this);
  const Tick at = now();
  if(m_closed || !m_io->isOpen()) return Reason::Closed;
  if(m_clock_fault) return Reason::ClockFault;
  if(!m_scope.generation || grant.scope != m_scope) return Reason::ScopeMismatch;
  if(!m_revoked || !m_fence.is_applied || m_fence.scope != m_scope) return Reason::RequiresFence;
  if(m_all_revoked) return Reason::Permission;
  if(m_response_granted || grant.token <= m_last_token) return Reason::ResponseUsed;
  if(grant.triggered < m_boundary || grant.triggered > m_fence.requested || grant.triggered > at ||
     grant.triggered > std::numeric_limits<Tick>::max() - response_limit)
    return Reason::InvalidTiming;
  if(grant.expires <= at || at >= grant.triggered + response_limit) return Reason::Expired;
  const auto eligible = guard(m_vehicle, m_envelope, grant.scope.home, at, true);
  if(eligible.reason != Reason::Ok) return eligible.reason;
  m_response = grant;
  m_response.expires = std::min(grant.expires, grant.triggered + response_limit);
  m_last_token = grant.token; m_response_granted = true;
  return Reason::Ok;
}
Reason ManagedEgress::requestCheck(uint64_t id, Scope scope, uint64_t token, Kind kind, Tick at)
{
  if(id == 0 || id <= m_last_id) return Reason::DuplicateId;
  m_last_id = id; // all new requests, including rejected ones, consume their ID
  if(m_closed || !m_io->isOpen()) return Reason::Closed;
  if(m_clock_fault) return Reason::ClockFault;
  if(!m_scope.generation || scope != m_scope) return Reason::ScopeMismatch;
  if(kind == Kind::Response) {
    if(m_all_revoked || !m_response_granted || token != m_response.token) return Reason::Permission;
    if(m_response_attempted) return Reason::ResponseUsed;
    if(at >= m_response.expires) return Reason::Expired;
  } else {
    if((kind == Kind::Navigation ? m_revoked : m_all_revoked) ||
       token != (kind == Kind::Navigation ? m_enable.nav_token : m_enable.probe_token))
      return Reason::Permission;
    if(at >= m_enable.expires) return Reason::Expired;
  }
  return Reason::Ok;
}
Admission ManagedEgress::admit(uint64_t id, Scope scope, uint64_t token, Build build,
                              uint64_t revision, Tick at, const NavInput* input)
{
  Admission result{id, requestCheck(id, scope, token, build.intent.kind, at), at, 0, false};
  if(result.reason == Reason::Ok) result.reason = build.evaluation.reason;
  if(result.reason != Reason::Ok) { ++m_stats.rejected; return result; }
  if(m_result_size + pendingCount() >= result_capacity) {
    // Do not dispatch an older NAV merely because the latest admission could not reserve a result.
    if(build.intent.kind == Kind::Navigation && m_navigation) {
      finish(*m_navigation, Outcome::CancelledOverload, Reason::ResultPressure, at, id);
      m_navigation.reset();
    }
    ++m_stats.rejected; result.reason = Reason::ResultPressure; return result;
  }
  if((build.intent.kind == Kind::Probe && m_probe_count == probe_capacity) ||
     (build.intent.kind == Kind::Response && m_hold)) {
    ++m_stats.rejected; result.reason = Reason::QueueFull; return result;
  }
  Work work;
  work.intent = build.intent; work.token = token;
  auto& r = work.result;
  r.id = id; r.scope = scope; r.kind = build.intent.kind; r.admitted = at;
  r.permission_expires = r.kind == Kind::Response ? m_response.expires : m_enable.expires;
  r.original_deadline = std::min(build.evaluation.deadline, r.permission_expires);
  r.input_revision = revision; r.observation_revision = m_vehicle.revision;
  r.observation_windows = {m_vehicle.flight.window, m_vehicle.landed.window, m_vehicle.pose.window,
                           m_vehicle.global.window, m_vehicle.estimator_eligible.window};
  if(input) r.input_windows = {input->course_deg.window, input->speed_mps.window, input->relative_altitude_m.window};
  if(r.kind == Kind::Navigation) {
    if(m_navigation) finish(*m_navigation, Outcome::Replaced, Reason::Ok, at, id);
    m_navigation = work;
  } else if(r.kind == Kind::Response) m_hold = work;
  else m_probes[m_probe_count++] = work;
  ++m_stats.admitted;
  result.accepted = true; result.deadline = r.original_deadline;
  return result;
}
Admission ManagedEgress::submitNavigation(uint64_t id, Scope scope, uint64_t token, const NavInput& input)
{
  Call call(*this);
  const Tick at = now();
  return admit(id, scope, token, navigation(input, m_vehicle, m_envelope, scope.home, at, m_boundary),
               input.revision, at, &input);
}
Admission ManagedEgress::submitResponse(uint64_t id, Scope scope, uint64_t token)
{
  Call call(*this);
  const Tick at = now();
  return admit(id, scope, token, hold(m_vehicle, m_envelope, scope.home, at), 0, at);
}
Admission ManagedEgress::submitProbe(uint64_t id, Scope scope, uint64_t token, uint32_t sequence, Tick expires)
{
  Call call(*this);
  const Tick at = now();
  Build build;
  build.intent.kind = Kind::Probe; build.intent.probe_sequence = sequence;
  build.evaluation = guard(m_vehicle, m_envelope, scope.home, at, false, true);
  build.evaluation.deadline = std::min(build.evaluation.deadline, expires);
  if(expires <= at) build.evaluation.reason = Reason::Expired;
  return admit(id, scope, token, build, 0, at);
}
void ManagedEgress::finish(Work work, Outcome outcome, Reason reason, Tick at, uint64_t related)
{
  if(m_result_size >= result_capacity) throw std::logic_error("result reservation invariant");
  auto& r = work.result;
  r.outcome = outcome; r.reason = reason; r.related_id = related;
  if(!r.attempted) { r.returned = at; r.return_clock_sample = m_raw_now; }
  if(reason == Reason::ClockFault) r.clock_fault = true;
  m_results[(m_result_head + m_result_size++) % result_capacity] = r;
  if(outcome == Outcome::Replaced) ++m_stats.replaced;
  if(outcome == Outcome::Expired) ++m_stats.expired;
  if(outcome == Outcome::Cancelled || outcome == Outcome::CancelledOverload || outcome == Outcome::Closed)
    ++m_stats.cancelled;
}
bool ManagedEgress::cancel(uint64_t id)
{
  Call call(*this);
  const Tick at = now();
  for(auto* slot : {&m_navigation, &m_hold}) {
    if(*slot && (*slot)->result.id == id) {
      finish(**slot, Outcome::Cancelled, Reason::Ok, at);
      slot->reset();
      return true;
    }
  }
  for(std::size_t i = 0; i < m_probe_count; ++i) {
    if(m_probes[i].result.id != id) continue;
    finish(m_probes[i], Outcome::Cancelled, Reason::Ok, at);
    for(std::size_t j = i + 1; j < m_probe_count; ++j) m_probes[j - 1] = m_probes[j];
    --m_probe_count;
    return true;
  }
  return false;
}
uint64_t ManagedEgress::requestFenceInternal(Tick at, FenceClasses classes)
{
  if(m_revoked && m_fence.id && m_fence.scope == m_scope &&
     (m_fence.all_classes || classes == FenceClasses::Routine)) return m_fence.id;
  m_revoked = true; m_fence_pending = true;
  if(classes == FenceClasses::All) m_all_revoked = true;
  m_fence = {};
  m_fence.id = ++m_fence_counter; m_fence.scope = m_scope; m_fence.requested = at;
  m_fence.all_classes = classes == FenceClasses::All;
  m_fence.accepted_before = m_scope_accepted; m_fence.last_accepted_id = m_last_accepted;
  m_fence.residual_possible = m_scope_residual;
  return m_fence.id;
}
uint64_t ManagedEgress::requestFence(Scope scope, FenceClasses classes)
{
  Call call(*this);
  const Tick at = now();
  if(scope != m_scope) return scope == m_fence.scope ? m_fence.id : 0;
  return requestFenceInternal(at, classes);
}
void ManagedEgress::applyFence(Tick at)
{
  if(m_fence_pending) {
    const auto outcome = m_fence.reason == Reason::ClockFault ? Outcome::ClockFault : Outcome::Cancelled;
    if(m_navigation) {
      m_navigation->result.clock_fault = m_fence.reason == Reason::ClockFault;
      finish(*m_navigation, outcome, m_fence.reason, at); m_navigation.reset();
    }
    if(m_fence.all_classes) {
      for(std::size_t i = 0; i < m_probe_count; ++i)
        finish(m_probes[i], outcome, m_fence.reason, at);
      m_probe_count = 0;
      if(m_hold) { finish(*m_hold, outcome, m_fence.reason, at); m_hold.reset(); }
    }
    m_fence_pending = false;
    m_fence.is_applied = m_fence.is_quiescent = true;
    m_fence.applied = m_fence.quiescent = at;
    m_fence.accepted_before = m_scope_accepted; m_fence.last_accepted_id = m_last_accepted;
    m_fence.residual_possible = m_scope_residual;
  }
}
void ManagedEgress::expireWaiting(Tick at)
{
  for(auto* slot : {&m_navigation, &m_hold}) {
    if(*slot && at >= (*slot)->result.original_deadline) {
      finish(**slot, Outcome::Expired, Reason::Expired, at);
      slot->reset();
    }
  }
  for(std::size_t i = 0; i < m_probe_count;) {
    if(at < m_probes[i].result.original_deadline) { ++i; continue; }
    finish(m_probes[i], Outcome::Expired, Reason::Expired, at);
    for(std::size_t j = i + 1; j < m_probe_count; ++j) m_probes[j - 1] = m_probes[j];
    --m_probe_count;
  }
}
Evaluation ManagedEgress::gate(const Work& work, Tick at) const
{
  if(m_closed || !m_io->isOpen()) return {Reason::Closed, 0};
  if(m_clock_fault) return {Reason::ClockFault, 0};
  const auto& r = work.result;
  if(r.scope != m_scope) return {Reason::ScopeMismatch, 0};
  Tick permission = 0;
  if(r.kind == Kind::Response) {
    if(m_all_revoked || !m_response_granted || m_response_attempted || work.token != m_response.token)
      return {Reason::Permission, 0};
    permission = m_response.expires;
  } else {
    if((r.kind == Kind::Navigation ? m_revoked : m_all_revoked) ||
       work.token != (r.kind == Kind::Navigation ? m_enable.nav_token : m_enable.probe_token))
      return {Reason::Permission, 0};
    permission = m_enable.expires;
  }
  auto result = guard(m_vehicle, m_envelope, r.scope.home, at,
                      r.kind == Kind::Response, r.kind == Kind::Probe);
  result.deadline = std::min({result.deadline, r.original_deadline, r.selected + selected_limit, permission});
  if(result.reason == Reason::Ok && at >= result.deadline) result.reason = Reason::Expired;
  return result;
}
void ManagedEgress::pump()
{
  Call call(*this);
  const Tick selected = now();
  if(m_clock_fault) {
    requestFenceInternal(selected, FenceClasses::All); m_fence.reason = Reason::ClockFault;
    applyFence(selected); m_io->close(); m_closed = true;
    return;
  }
  const bool control_turn = m_fence_pending;
  applyFence(selected);
  if(control_turn) return; // expose the applied boundary before normal work can run
  expireWaiting(selected);
  if(m_closed) return;
  std::optional<Work> chosen;
  if(m_hold) { chosen = *m_hold; m_hold.reset(); }
  else if(m_navigation && (m_probe_count == 0 || m_nav_streak < 3)) {
    chosen = *m_navigation; m_navigation.reset(); m_nav_streak = std::min(3u, m_nav_streak + 1);
  } else if(m_probe_count) {
    chosen = m_probes[0];
    for(std::size_t i = 1; i < m_probe_count; ++i) m_probes[i - 1] = m_probes[i];
    --m_probe_count; m_nav_streak = 0;
  }
  if(!chosen) return;
  m_active = true;
  Work work = *chosen;
  auto& r = work.result;
  r.selected = selected;
  const auto elapsed_ms = static_cast<uint64_t>((selected - m_started) / 1000000);
  const Frame frame = encode(work.intent, m_binding, static_cast<uint32_t>(elapsed_ms), m_codec);
  r.frame_bytes = frame.size;
  const Tick at = now(); // the final clock/check is AFTER complete codec preparation
  const auto eligible = gate(work, at);
  r.final_deadline = eligible.deadline; r.gate_revision = m_vehicle.revision;
  if(eligible.reason != Reason::Ok) {
    m_active = false;
    finish(work, eligible.reason == Reason::Expired ? Outcome::Expired :
           (eligible.reason == Reason::ClockFault ? Outcome::ClockFault : Outcome::GateRejected),
           eligible.reason, at);
    return;
  }
  r.attempt_started = at; r.attempted = true;
  ++m_stats.attempted;
  if(r.kind == Kind::Response) m_response_attempted = true;
  const WriteResult io = m_io->tryWrite(frame); // one nonblocking datagram attempt, NEVER retry
  r.returned = now(); r.return_clock_sample = m_raw_now;
  r.accepted_bytes = io.bytes; r.error = io.error;
  r.clock_fault = m_clock_fault;
  r.late = m_clock_fault || r.returned >= r.final_deadline;
  r.residual_possible = io.bytes > 0;
  Outcome outcome = Outcome::IoError;
  if(io.code == WriteCode::Returned && io.bytes == frame.size) {
    outcome = Outcome::Accepted; r.locally_accepted = true;
    ++m_stats.accepted; ++m_scope_accepted; m_last_accepted = r.id;
  } else if(io.code == WriteCode::Returned) {
    outcome = Outcome::ShortDatagram; ++m_stats.short_datagrams;
  } else if(io.code == WriteCode::WouldBlock) {
    outcome = Outcome::WouldBlock; ++m_stats.would_block;
  } else ++m_stats.io_errors;
  m_scope_residual = m_scope_residual || r.residual_possible;
  if(r.late) ++m_stats.overruns;
  m_active = false;
  finish(work, outcome, m_clock_fault ? Reason::ClockFault : Reason::Ok, r.returned);
  if(r.late || outcome == Outcome::ShortDatagram) {
    requestFenceInternal(r.returned);
    m_fence.reason = m_clock_fault ? Reason::ClockFault : (r.late ? Reason::Expired : Reason::InvalidValue);
  }
}
Fence ManagedEgress::close()
{
  Call call(*this);
  if(m_closed) return m_fence;
  const Tick at = now();
  requestFenceInternal(at, FenceClasses::All); m_fence.reason = Reason::Closed; applyFence(at);
  m_io->close(); m_closed = true;
  return m_fence;
}
ReceiveBatch ManagedEgress::receiveBatch()
{
  Call call(*this);
  ReceiveBatch batch;
  for(unsigned i = 0; i < batch.datagrams.size(); ++i) {
    auto received = m_io->tryRead();
    if(received.code == ReadCode::WouldBlock || received.code == ReadCode::Closed) break;
    if(received.code == ReadCode::WrongPeer) { ++m_stats.rx_wrong_peer; continue; }
    if(received.code == ReadCode::Truncated) { ++m_stats.rx_truncated; continue; }
    if(received.code == ReadCode::Error) { ++m_stats.rx_errors; break; }
    received.acquired = now(); ++m_stats.rx_datagrams;
    batch.datagrams[batch.size++] = received;
  }
  return batch;
}
std::optional<Result> ManagedEgress::popResult()
{
  Call call(*this);
  if(!m_result_size) return {};
  Result result = m_results[m_result_head];
  m_result_head = (m_result_head + 1) % result_capacity; --m_result_size;
  return result;
}
Stats ManagedEgress::stats() const { checkAccess(); return m_stats; }
Fence ManagedEgress::fence() const { checkAccess(); return m_fence; }
std::size_t ManagedEgress::outstanding() const { checkAccess(); return m_result_size + pendingCount(); }
uint16_t ManagedEgress::localPort() const { checkAccess(); return m_io->localPort(); }
} // namespace mavmoos::egress
