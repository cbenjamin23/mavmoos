#include "EgressTestSupport.h"
#include <future>
#include <set>
#include <type_traits>

using namespace p2test;
namespace {
template<class T, class = void> struct PublicWrite : std::false_type {};
template<class T> struct PublicWrite<T, std::void_t<decltype(std::declval<T&>().tryWrite(
  std::declval<const Frame&>()))>> : std::true_type {};
static_assert(!PublicWrite<UdpLoopbackIo>::value, "real socket write must remain private");
static_assert(!PublicWrite<DatagramIo>::value, "raw transport bypass must remain private");

void mapping(Trace& trace)
{
  trace.begin("A1_mapping_masks_and_msl_hold");
  Rig rig(trace);
  const auto pose = state(rig.clock.at);
  for(const auto angle : {0.0, 90.0, 180.0, 270.0}) {
    auto built = navigation(input(rig.clock.at, angle), pose, envelope(), 1, rig.clock.at, 100 * second);
    check(built.evaluation.reason == Reason::Ok, "cardinal mapping eligible");
    const double expected_north = angle == 0 ? 2 : (angle == 180 ? -2 : 0);
    const double expected_east = angle == 90 ? 2 : (angle == 270 ? -2 : 0);
    near(built.intent.north, expected_north, "north axis");
    near(built.intent.east, expected_east, "east axis");
    near(built.intent.down, -1, "up is negative down");
    check(built.intent.mask == 2503, "moving velocity+yaw mask");
    mavlink_status_t codec{};
    const auto wire = encode(built.intent, binding(), 10, codec);
    const auto target = local(wire);
    check(target.time_boot_ms == 10, "declared lab wire timestamp");
    check(target.coordinate_frame == MAV_FRAME_LOCAL_NED, "local NED frame");
    check(target.target_system == 1 && target.target_component == 1, "fixed target IDs");
    near(target.yaw, angle * 3.14159265358979323846 / 180, "yaw radians");
  }
  for(double speed : {0.0, 0.05, 0.050001}) {
    auto built = navigation(input(rig.clock.at, 90, speed, 8), pose, envelope(), 1, rig.clock.at, 100 * second);
    check(built.intent.mask == (speed > .05 ? 2503 : 3527), "strict near-zero boundary");
    near(built.intent.down, 1, "descent clipped");
  }
  auto soft = navigation(input(rig.clock.at, 0, 0, 10.2), pose, envelope(), 1, rig.clock.at, 100 * second);
  near(soft.intent.down, -.1, "vertical feedback gain");
  auto capture = hold(pose, envelope(), 1, rig.clock.at);
  check(capture.evaluation.reason == Reason::Ok, "zero coordinates are present");
  mavlink_status_t codec{};
  auto message = decode(encode(capture.intent, binding(), 10, codec));
  mavlink_set_position_target_global_int_t target{};
  mavlink_msg_set_position_target_global_int_decode(&message, &target);
  check(target.coordinate_frame == MAV_FRAME_GLOBAL_INT && target.type_mask == 3576, "MSL position-only frame/mask");
  check(target.lat_int == 0 && target.lon_int == 0, "zero coordinates encoded exactly");
  near(target.alt, 123.45, "observed MSL, not desired home-relative altitude", 1e-5);
  trace.pass();
}
void timeBoundaries(Trace& trace)
{
  trace.begin("A4_selected_age_and_permission_expiry");
  {
    Rig r(trace);
    auto a = submit(r, trace, 1, input(r.clock.at));
    check(a.accepted, "selected budget work");
    const Tick selected = r.clock.at;
    r.clock.reads({selected, selected + selected_limit});
    r.engine->pump();
    auto result = drain(*r.engine, trace).at(0);
    check(result.outcome == Outcome::Expired && !result.attempted &&
          result.final_deadline == selected + selected_limit &&
          result.original_deadline == a.deadline, "selected limit is additional, never a renewal");
  }
  {
    Rig r(trace);
    const auto ticket = r.engine->requestFence(scope(), FenceClasses::All);
    r.engine->pump();
    const Tick expiry = r.clock.at + second / 25;
    check(r.engine->enable({scope(2), 4, 5, expiry, ticket}) == Reason::Ok, "short independent permit");
    r.clock.at += second / 100;
    auto a = r.engine->submitNavigation(1, scope(2), 4, input(r.clock.at));
    trace.admission(a);
    check(a.accepted && a.deadline == expiry, "permission dominates original deadline");
    r.clock.at = expiry; r.engine->pump();
    check(drain(*r.engine, trace).at(0).outcome == Outcome::Expired && r.io->calls == 0, "permit expires at equality");
  }
  trace.pass();
}
void invalids(Trace& trace)
{
  trace.begin("A2_missing_invalid_and_observation_guards");
  Rig r(trace);
  const auto pose = state(r.clock.at);
  auto baseline = input(r.clock.at);
  for(double value : {-1.0, 5.001, std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::infinity()}) {
    auto in = baseline; in.speed_mps.value = value;
    check(navigation(in, pose, envelope(), 1, r.clock.at, 100 * second).evaluation.reason == Reason::InvalidValue,
          "negative/out-of-envelope/nonfinite speed");
  }
  for(double angle : {-1.0, 360.0, std::numeric_limits<double>::quiet_NaN()}) {
    auto in = baseline; in.course_deg.value = angle;
    check(navigation(in, pose, envelope(), 1, r.clock.at, 100 * second).evaluation.reason == Reason::InvalidValue, "course range");
  }
  for(double altitude : {0.5, 50.01, std::numeric_limits<double>::infinity()}) {
    auto in = baseline; in.relative_altitude_m.value = altitude;
    check(navigation(in, pose, envelope(), 1, r.clock.at, 100 * second).evaluation.reason == Reason::InvalidValue, "height envelope");
  }
  auto missing = baseline; missing.speed_mps.value.reset();
  check(navigation(missing, pose, envelope(), 1, r.clock.at, 100 * second).evaluation.reason == Reason::Missing, "missing value has no zero default");
  auto future = baseline; future.course_deg.window.acquired = r.clock.at + 1;
  check(navigation(future, pose, envelope(), 1, r.clock.at, 100 * second).evaluation.reason == Reason::InvalidTiming, "future acquisition");
  auto old = baseline; old.course_deg.window.acquired = 99 * second;
  check(navigation(old, pose, envelope(), 1, r.clock.at, 100 * second).evaluation.reason == Reason::InvalidTiming, "pre-generation inputs");
  for(auto landed : {Landed::Undefined, Landed::OnGround, Landed::Takeoff, Landed::Landing}) {
    auto bad = pose; bad.landed.value = landed;
    check(hold(bad, envelope(), 1, r.clock.at).evaluation.reason == Reason::Guard, "only IN_AIR permits hold");
  }
  auto bad = pose; bad.flight.value = Flight{false, true};
  check(hold(bad, envelope(), 1, r.clock.at).evaluation.reason == Reason::Guard, "no mode recapture");
  bad = pose; bad.flight.value = Flight{true, false};
  check(hold(bad, envelope(), 1, r.clock.at).evaluation.reason == Reason::Guard, "disarmed");
  bad = pose; bad.estimator_eligible.value = false;
  check(hold(bad, envelope(), 1, r.clock.at).evaluation.reason == Reason::Guard, "estimator guard");
  bad = pose; bad.home_known = false;
  check(hold(bad, envelope(), 1, r.clock.at).evaluation.reason == Reason::HomeMismatch, "unknown reference");
  bad = pose; bad.global.value.reset();
  check(hold(bad, envelope(), 1, r.clock.at).evaluation.reason == Reason::Missing, "unknown global hold pose");
  bad = pose; bad.global.value->latitude_deg = 91;
  check(hold(bad, envelope(), 1, r.clock.at).evaluation.reason == Reason::InvalidValue, "latitude range");
  bad = pose; bad.pose.window.acquired -= second / 1000; bad.pose.window.expires = r.clock.at;
  check(hold(bad, envelope(), 1, r.clock.at).evaluation.reason == Reason::Expired, "expiry is exclusive");
  check(!validEnvelope({0, 1, 50}) && !validEnvelope({5, 50, 1}), "explicit valid envelope required");
  trace.pass();
}
void deadlines(Trace& trace)
{
  trace.begin("A3_original_deadlines_and_preparation_expiry");
  {
    Rig r(trace);
    auto in = input(r.clock.at); in.course_deg.window.expires = r.clock.at + second / 25;
    auto admitted = submit(r, trace, 1, in);
    check(admitted.accepted, "initial admission");
    r.clock.reads({r.clock.at, admitted.deadline}); // expires during preparation, before syscall
    r.engine->pump();
    auto results = drain(*r.engine, trace);
    check(results.size() == 1 && results[0].outcome == Outcome::Expired && !results[0].attempted, "final gate expires after preparation");
    check(r.io->calls == 0 && results[0].original_deadline == admitted.deadline, "no deadline renewal");
  }
  {
    Rig r(trace);
    auto in = input(r.clock.at);
    auto admitted = submit(r, trace, 1, in);
    r.clock.at += second / 5;
    check(r.engine->updateVehicle(state(r.clock.at, 2)), "new observations");
    r.clock.at = admitted.deadline;
    r.engine->pump();
    auto results = drain(*r.engine, trace);
    check(results[0].outcome == Outcome::Expired && r.io->calls == 0, "new observation cannot renew original dependency");
  }
  {
    Rig r(trace);
    auto lease = r.engine->enable({scope(2), 4, 5, r.clock.at + 10 * second, 0});
    check(lease == Reason::RequiresFence, "cannot extend permission by enabling again");
    auto in = input(r.clock.at); in.course_deg.window.expires = r.clock.at + 2 * second;
    auto a = submit(r, trace, 1, in); check(a.accepted, "copied input admitted");
    in.course_deg.value = 180; in.course_deg.window.expires += 10 * second;
    r.engine->pump();
    auto results = drain(*r.engine, trace);
    near(local(r.io->frames[0]).vy, 2, "caller mutation cannot change admitted target");
    check(results[0].original_deadline == a.deadline, "copy keeps original deadline");
  }
  trace.pass();
}
void replacementAndBounds(Trace& trace)
{
  trace.begin("B1_latest_nav_result_reservation_and_fence_priority");
  Rig r(trace);
  for(uint64_t id = 1; id <= ManagedEgress::result_capacity; ++id)
    check(submit(r, trace, id, input(r.clock.at)).accepted, "bounded replacement admitted");
  check(r.engine->outstanding() == 16, "16 reserved pending+result records");
  auto refused = submit(r, trace, 17, input(r.clock.at));
  check(!refused.accepted && refused.reason == Reason::ResultPressure, "result backpressure explicit");
  const auto ticket = r.engine->requestFence(scope(), FenceClasses::All);
  trace.fence(r.engine->fence());
  check(ticket && !r.engine->fence().is_applied, "request is distinct from application");
  r.engine->pump();
  trace.fence(r.engine->fence());
  check(r.io->calls == 0 && r.engine->fence().is_quiescent, "full result buffer cannot starve fence");
  auto results = drain(*r.engine, trace);
  std::set<uint64_t> ids;
  for(const auto& result : results) check(ids.insert(result.id).second, "one terminal per admitted ID");
  check(results.size() == 16 && results.back().outcome == Outcome::CancelledOverload, "older NAV removed on overload");
  check(r.engine->outstanding() == 0, "all results retained and drained");
  trace.pass();
  trace.begin("B2_newest_target_and_probe_fairness");
  Rig f(trace);
  for(uint64_t id = 1; id <= 3; ++id)
    check(submit(f, trace, id, input(f.clock.at, 0, double(id))).accepted, "replace NAV");
  f.engine->pump();
  near(local(f.io->frames[0]).vx, 3, "only newest target sent");
  check(local(f.io->frames[0]).time_boot_ms == 10, "elapsed instance time is not source freshness");
  auto initial = drain(*f.engine, trace);
  check(initial[0].related_id == 2 && initial[1].related_id == 3, "replacement lineage");
  check(f.engine->submitProbe(4, scope(), 2, 42, f.clock.at + second).accepted, "background probe");
  // One NAV already dispatched above; at most three consecutive NAV selections.
  for(uint64_t id = 5; id <= 7; ++id) {
    check(submit(f, trace, id, input(f.clock.at)).accepted, "fresh host NAV work");
    f.engine->pump();
  }
  check(decode(f.io->frames.back()).msgid == MAVLINK_MSG_ID_PING, "bounded probe service");
  f.engine->pump();
  drain(*f.engine, trace);
  trace.pass();
}
void errorsAndLate(Trace& trace)
{
  trace.begin("B3_would_block_error_short_and_no_retry");
  for(auto mode : {0, 1, 2, 3}) {
    Rig r(trace);
    if(mode < 2) {
      r.io->code = mode == 0 ? WriteCode::WouldBlock : WriteCode::Error;
      r.io->error = mode == 0 ? EAGAIN : EINTR; r.io->count = 0;
    } else r.io->count = mode == 2 ? 3 : 0;
    check(submit(r, trace, 1, input(r.clock.at)).accepted, "fault work admitted");
    r.engine->pump();
    auto result = drain(*r.engine, trace).at(0);
    const auto expected = mode == 0 ? Outcome::WouldBlock : mode == 1 ? Outcome::IoError : Outcome::ShortDatagram;
    check(result.outcome == expected && !result.locally_accepted, "no false handoff");
    check(result.residual_possible == (mode == 2), "short/error residual accounting");
    r.engine->pump(); r.engine->pump();
    check(r.io->calls == 1, "no retry or suffix datagram");
  }
  trace.pass();
  trace.begin("B4_late_acceptance_fence_and_delayed_result");
  Rig r(trace);
  r.io->delay = second / 5;
  check(submit(r, trace, 1, input(r.clock.at)).accepted, "late work admitted");
  r.engine->pump();
  check(!submit(r, trace, 2, input(r.clock.at)).accepted, "late return closes routine admission");
  check(!r.engine->fence().is_applied, "auto-fence requested, not yet applied");
  r.engine->pump(); trace.fence(r.engine->fence());
  const auto fence = r.engine->fence();
  check(fence.is_quiescent && fence.residual_possible && fence.accepted_before == 1, "accepted residual survives quiescence");
  check(r.engine->enable({scope(2), 4, 5, r.clock.at + second, 0}) == Reason::Reconciliation, "explicit reconciliation required");
  check(r.engine->enable({scope(2), 4, 5, r.clock.at + second, fence.id}) == Reason::Ok, "new explicit generation");
  auto old = r.engine->submitNavigation(3, scope(), 1, input(r.clock.at));
  trace.admission(old);
  check(old.reason == Reason::ScopeMismatch, "delayed old producer rejected");
  auto delayed = r.engine->popResult();
  check(delayed && delayed->id == 1 && delayed->outcome == Outcome::Accepted &&
        delayed->late && delayed->locally_accepted && delayed->returned > delayed->final_deadline,
        "late local acceptance retained, not relabelled as dropped");
  trace.result(*delayed);
  check(r.engine->requestFence(scope()) == fence.id, "old fence idempotent without revoking new generation");
  r.io->delay = 0;
  r.clock.at += second / 100;
  auto fresh = r.engine->submitNavigation(4, scope(2), 4, input(r.clock.at));
  trace.admission(fresh); check(fresh.accepted, "fresh new-generation decision");
  r.engine->pump();
  check(r.io->calls == 2, "only explicitly new work writes again");
  drain(*r.engine, trace); trace.pass();
}
void responses(Trace& trace)
{
  trace.begin("B5_response_priority_one_attempt_and_original_trigger");
  Rig r(trace);
  check(r.engine->submitProbe(1, scope(), 2, 42, r.clock.at + second).accepted, "queued probe");
  const Tick trigger = r.clock.at;
  r.engine->requestFence(scope()); r.engine->pump();
  trace.fence(r.engine->fence());
  check(r.engine->grantResponse({scope(), 3, trigger, trigger + second}) == Reason::Ok, "separate response permit");
  auto hold = r.engine->submitResponse(2, scope(), 3);
  trace.admission(hold); check(hold.accepted, "response work");
  check(r.engine->submitResponse(3, scope(), 3).reason == Reason::QueueFull, "one waiting response slot");
  r.io->code = WriteCode::WouldBlock; r.io->count = 0; r.io->error = EAGAIN;
  r.engine->pump();
  check(decode(r.io->frames[0]).msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT, "response precedes waiting probe");
  check(r.engine->submitResponse(4, scope(), 3).reason == Reason::ResponseUsed, "one OS attempt per permit");
  check(r.engine->grantResponse({scope(), 4, trigger, trigger + second}) == Reason::ResponseUsed, "cannot renew response permit");
  r.io->code = WriteCode::Returned; r.io->count = std::numeric_limits<std::size_t>::max();
  r.engine->pump();
  check(decode(r.io->frames[1]).msgid == MAVLINK_MSG_ID_PING, "background work survives routine-only fence");
  drain(*r.engine, trace); trace.pass();
  trace.begin("B6_late_response_unavailable_not_renewed_by_fence");
  Rig late(trace);
  const Tick old_trigger = late.clock.at;
  late.clock.at += second / 3;
  late.engine->requestFence(scope()); late.engine->pump();
  check(late.engine->updateVehicle(state(late.clock.at, 2)), "fresh independent pose");
  check(late.engine->grantResponse({scope(), 3, old_trigger, old_trigger + 10 * second}) == Reason::Ok, "earlier trigger retained");
  auto a = late.engine->submitResponse(1, scope(), 3);
  trace.admission(a);
  check(a.accepted && a.deadline == old_trigger + response_limit, "response capped at original trigger plus 0.5");
  late.clock.at = a.deadline;
  late.engine->pump();
  check(late.io->calls == 0 && drain(*late.engine, trace)[0].outcome == Outcome::Expired, "no late response write");
  trace.pass();
}
void fencesAndLifetime(Trace& trace)
{
  trace.begin("C1_cancel_fence_new_generation_and_original_inputs");
  Rig r(trace);
  auto old_input = input(r.clock.at);
  check(submit(r, trace, 1, old_input).accepted, "waiting nav");
  check(r.engine->cancel(1) && !r.engine->cancel(1), "cancel is waiting-only/idempotent outcome");
  check(r.engine->submitProbe(2, scope(), 2, 1, r.clock.at + second).accepted, "waiting probe");
  const auto ticket = r.engine->requestFence(scope(), FenceClasses::All);
  check(r.engine->requestFence(scope(), FenceClasses::All) == ticket, "fence request idempotent");
  check(r.engine->enable({scope(2), 4, 5, r.clock.at + second, ticket}) == Reason::RequiresFence, "request alone is not application");
  r.engine->pump(); trace.fence(r.engine->fence());
  check(r.io->calls == 0, "priority fence before write");
  r.clock.at += second / 100;
  check(r.engine->enable({scope(2), 4, 5, r.clock.at + second, ticket}) == Reason::Ok, "fenced scope explicitly replaced");
  auto retag = r.engine->submitNavigation(3, scope(2), 4, old_input);
  trace.admission(retag);
  check(retag.reason == Reason::InvalidTiming, "old decisions cannot be retagged into new grant");
  check(r.engine->submitProbe(4, scope(), 2, 2, r.clock.at + second).reason == Reason::ScopeMismatch, "old probe also bound");
  check(r.engine->submitNavigation(5, scope(2, 99), 4, input(r.clock.at)).reason == Reason::ScopeMismatch, "old/wrong connection epoch");
  drain(*r.engine, trace); trace.pass();
  trace.begin("C2_current_guard_changes_and_no_cached_renewal");
  Rig changed(trace);
  check(submit(changed, trace, 1, input(changed.clock.at)).accepted, "guarded target");
  auto next = state(100 * second);
  next.revision = 2; next.pose.window.expires += second;
  check(!changed.engine->updateVehicle(next), "same receipt cannot renew expiry");
  next = state(100 * second); next.revision = 2; next.flight = {};
  check(changed.engine->updateVehicle(next), "unavailable report");
  auto cached = state(100 * second); cached.revision = 3;
  cached.flight.window.acquired += second / 100; cached.flight.window.expires += second / 100;
  check(!changed.engine->updateVehicle(cached), "unavailable gap cannot erase receipt high-water mark");
  next = state(changed.clock.at, 2); next.flight.value->guided = false;
  next.revision = 3;
  check(changed.engine->updateVehicle(next), "observed override");
  changed.engine->pump();
  auto result = drain(*changed.engine, trace).at(0);
  check(!result.attempted && result.reason == Reason::Guard && changed.io->calls == 0, "final current mode guard");
  check(changed.engine->submitResponse(2, scope(), 3).reason == Reason::Permission, "no unsolicited response");
  trace.pass();
  trace.begin("C3_close_and_clock_fault_settle_reserved_work");
  Rig close(trace);
  check(submit(close, trace, 1, input(close.clock.at)).accepted, "close waiting nav");
  check(close.engine->submitProbe(2, scope(), 2, 1, close.clock.at + second).accepted, "close waiting probe");
  trace.fence(close.engine->close());
  check(close.engine->fence().is_quiescent && close.io->calls == 0 && !close.io->opened, "close settled, no callback buffers");
  check(close.engine->submitProbe(3, scope(), 2, 1, close.clock.at + second).reason == Reason::Closed, "closed admission");
  check(drain(*close.engine, trace).size() == 2, "one close outcome per waiting work");
  Rig fault(trace);
  check(submit(fault, trace, 1, input(fault.clock.at)).accepted, "clock work");
  fault.clock.at -= second;
  fault.engine->pump();
  check(fault.io->calls == 0 && !fault.io->opened && fault.engine->fence().is_quiescent, "clock regression latches and closes");
  drain(*fault.engine, trace); trace.pass();
  trace.begin("C5_clock_regression_after_local_acceptance");
  Rig returned_fault(trace);
  returned_fault.io->delay = -second;
  check(submit(returned_fault, trace, 1, input(returned_fault.clock.at)).accepted, "clock fault after attempt");
  returned_fault.engine->pump();
  auto uncertain = drain(*returned_fault.engine, trace).at(0);
  check(uncertain.locally_accepted && uncertain.clock_fault && uncertain.residual_possible &&
        uncertain.return_clock_sample < uncertain.attempt_started, "invalid raw clock sample retained without hiding acceptance");
  returned_fault.engine->pump(); trace.fence(returned_fault.engine->fence());
  check(returned_fault.engine->fence().all_classes &&
        returned_fault.engine->fence().reason == Reason::ClockFault, "clock fault blocks all further effects");
  trace.pass();
}
void moreBounds(Trace& trace)
{
  trace.begin("B7_discrete_capacity_fifo_and_waiting_cancel");
  Rig r(trace);
  for(uint64_t id = 1; id <= 4; ++id) {
    auto a = r.engine->submitProbe(id, scope(), 2, uint32_t(id), r.clock.at + second);
    trace.admission(a); check(a.accepted, "four probe slots");
  }
  check(r.engine->submitProbe(5, scope(), 2, 5, r.clock.at + second).reason == Reason::QueueFull, "fifth probe unavailable");
  check(r.engine->cancel(2), "cancel exact waiting discrete item");
  for(unsigned i = 0; i < 4; ++i) r.engine->pump();
  check(r.io->calls == 3, "no cancelled discrete write");
  for(unsigned i = 0; i < 3; ++i) {
    auto message = decode(r.io->frames[i]);
    mavlink_ping_t ping{}; mavlink_msg_ping_decode(&message, &ping);
    check(ping.seq == (i == 0 ? 1 : i + 2), "remaining FIFO order");
  }
  check(drain(*r.engine, trace).size() == 4, "all admitted discrete work settled");
  trace.pass();
  trace.begin("B8_response_result_backpressure_is_explicit");
  Rig pressure(trace);
  for(uint64_t id = 1; id <= 16; ++id)
    check(submit(pressure, trace, id, input(pressure.clock.at)).accepted, "fill reserved results");
  const Tick trigger = pressure.clock.at;
  pressure.engine->requestFence(scope()); pressure.engine->pump();
  check(pressure.engine->grantResponse({scope(), 3, trigger, trigger + response_limit}) == Reason::Ok, "control permit independent of result capacity");
  auto unavailable = pressure.engine->submitResponse(17, scope(), 3); trace.admission(unavailable);
  check(unavailable.reason == Reason::ResultPressure && !unavailable.accepted && pressure.io->calls == 0,
        "no hidden unbounded emergency queue");
  drain(*pressure.engine, trace); trace.pass();
  trace.begin("C6_all_class_upgrade_cancels_response_and_probe");
  Rig upgrade(trace);
  upgrade.engine->requestFence(scope()); upgrade.engine->pump();
  const auto routine = upgrade.engine->fence();
  check(upgrade.engine->grantResponse({scope(), 3, routine.requested, routine.requested + response_limit}) == Reason::Ok, "response before full revoke");
  check(upgrade.engine->submitResponse(1, scope(), 3).accepted, "waiting hold");
  check(upgrade.engine->submitProbe(2, scope(), 2, 1, upgrade.clock.at + second).accepted, "waiting probe");
  const auto all = upgrade.engine->requestFence(scope(), FenceClasses::All);
  check(all != routine.id, "stronger fence has distinct acknowledgement identity");
  upgrade.engine->pump(); trace.fence(upgrade.engine->fence());
  check(upgrade.io->calls == 0 && drain(*upgrade.engine, trace).size() == 2, "full revoke cancels every class");
  check(upgrade.engine->submitResponse(3, scope(), 3).reason == Reason::Permission,
        "all-class fence also closes old response admission");
  trace.pass();
}
void ownership(Trace& trace)
{
  trace.begin("C4_foreign_thread_and_reentrant_calls_rejected");
  Rig r(trace);
  check(submit(r, trace, 1, input(r.clock.at)).accepted, "ownership work");
  std::promise<void> entered, done;
  auto entered_future = entered.get_future(), done_future = done.get_future();
  unsigned foreign_rejections = 0, reentrant_rejections = 0;
  std::thread foreign([&] {
    entered_future.wait();
    for(auto action : {0, 1, 2, 3}) {
      try {
        if(action == 0) r.engine->requestFence(scope());
        if(action == 1) r.engine->pump();
        if(action == 2) r.engine->close();
        if(action == 3) r.engine->stats();
      } catch(const std::logic_error&) { ++foreign_rejections; }
    }
    done.set_value();
  });
  r.io->on_write = [&] {
    entered.set_value(); done_future.wait();
    try { r.engine->requestFence(scope()); } catch(const std::logic_error&) { ++reentrant_rejections; }
    try { r.engine->cancel(1); } catch(const std::logic_error&) { ++reentrant_rejections; }
  };
  r.engine->pump(); foreign.join();
  check(foreign_rejections == 4 && reentrant_rejections == 2 && !r.io->callback_failed, "owner-only API enforced during call");
  const auto accepted = drain(*r.engine, trace).at(0);
  check(accepted.locally_accepted, "already started call completed before admitted owner fence");
  r.engine->requestFence(scope(), FenceClasses::All); r.engine->pump();
  trace.fence(r.engine->fence());
  check(r.engine->fence().residual_possible, "no false recall after owner boundary");
  trace.pass();
}
}
int main(int argc, char** argv)
{
  try {
    check(argc == 2, "trace path argument");
    Trace trace(argv[1]);
    trace.out << "{\"event\":\"resource_limits\",\"nav_slots\":1,\"response_slots\":1,"
      "\"probe_slots\":4,\"result_slots\":16,\"writes_per_pump\":1,\"reads_per_batch\":8,"
      "\"receive_datagram_bytes\":4096,\"frame_capacity_bytes\":" << MAVLINK_MAX_PACKET_LEN
      << ",\"engine_bytes\":" << sizeof(ManagedEgress) << ",\"result_bytes\":" << sizeof(Result)
      << ",\"receive_batch_bytes\":" << sizeof(ReceiveBatch) << "}\n";
    mapping(trace); invalids(trace); deadlines(trace); timeBoundaries(trace); replacementAndBounds(trace);
    errorsAndLate(trace); responses(trace); moreBounds(trace); fencesAndLifetime(trace); ownership(trace);
    trace.summary("fake_clock_and_io");
    return 0;
  } catch(const std::exception& error) { std::cerr << "FAIL " << error.what() << '\n'; return 1; }
}
