/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: CopterModel.cpp                                 */
/*    DATE: October 29, 2026                                */
/************************************************************/
#include "CopterModel.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace mavmoos::egress {
namespace {
constexpr double pi = 3.14159265358979323846;
bool finiteFloat(double value)
{
  return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
}
template<class T> Evaluation lifetime(const Sample<T>& sample, Tick age, Tick now)
{
  if(!sample.value) return {Reason::Missing, 0};
  const auto& w = sample.window;
  if(w.receipt == 0 || w.acquired < 0 || w.acquired > now || w.expires <= w.acquired ||
     w.acquired > std::numeric_limits<Tick>::max() - age)
    return {Reason::InvalidTiming, 0};
  const Tick deadline = std::min(w.expires, w.acquired + age);
  return {now < deadline ? Reason::Ok : Reason::Expired, deadline};
}
void combine(Evaluation& result, Evaluation next)
{
  if(result.reason == Reason::Ok && next.reason != Reason::Ok) result.reason = next.reason;
  if(next.deadline > 0) result.deadline = std::min(result.deadline, next.deadline);
}
}
bool validEnvelope(const Envelope& e)
{
  return finiteFloat(e.maximum_speed_mps) && e.maximum_speed_mps > 0 &&
         finiteFloat(e.minimum_relative_m) && finiteFloat(e.maximum_relative_m) &&
         e.minimum_relative_m >= 0 && e.minimum_relative_m < e.maximum_relative_m;
}
Evaluation guard(const VehicleState& s, const Envelope& e, uint64_t home, Tick now,
                 bool need_global, bool probe_only)
{
  Evaluation result{Reason::Ok, std::numeric_limits<Tick>::max()};
  if(s.revision == 0) return {Reason::Missing, 0};
  combine(result, lifetime(s.flight, 2 * second, now));
  if(probe_only) return result;
  combine(result, lifetime(s.landed, 2 * second, now));
  combine(result, lifetime(s.pose, second / 2, now));
  combine(result, lifetime(s.estimator_eligible, 3 * second, now));
  if(result.reason != Reason::Ok) return result;
  if(!s.home_known || home == 0 || s.home_revision != home)
    return {Reason::HomeMismatch, result.deadline};
  if(!s.flight.value->guided || !s.flight.value->armed || *s.landed.value != Landed::InAir ||
     !*s.estimator_eligible.value)
    return {Reason::Guard, result.deadline};
  const auto& pose = *s.pose.value;
  if(!finiteFloat(pose.relative_home_m) || !finiteFloat(pose.north_mps) ||
     !finiteFloat(pose.east_mps) || !finiteFloat(pose.down_mps) ||
     pose.relative_home_m < e.minimum_relative_m || pose.relative_home_m > e.maximum_relative_m)
    return {Reason::InvalidValue, result.deadline};
  if(need_global) {
    combine(result, lifetime(s.global, second / 2, now));
    if(result.reason != Reason::Ok) return result;
    const auto& position = *s.global.value;
    if(!std::isfinite(position.latitude_deg) || std::abs(position.latitude_deg) > 90 ||
       !std::isfinite(position.longitude_deg) || std::abs(position.longitude_deg) > 180 ||
       !finiteFloat(position.altitude_msl_m))
      return {Reason::InvalidValue, result.deadline};
  }
  return result;
}
Build navigation(const NavInput& in, const VehicleState& state, const Envelope& envelope,
                 uint64_t home, Tick now, Tick boundary)
{
  Build out;
  out.intent.kind = Kind::Navigation;
  out.evaluation = guard(state, envelope, home, now);
  if(out.evaluation.reason != Reason::Ok) return out;
  for(const auto* sample : {&in.course_deg, &in.speed_mps, &in.relative_altitude_m}) {
    combine(out.evaluation, lifetime(*sample, 2 * second, now));
    if(out.evaluation.reason == Reason::Ok && sample->window.acquired <= boundary)
      out.evaluation.reason = Reason::InvalidTiming;
  }
  if(in.revision == 0) out.evaluation.reason = Reason::Missing;
  else if(in.home_revision != home) out.evaluation.reason = Reason::HomeMismatch;
  if(out.evaluation.reason != Reason::Ok) return out;
  const double course = *in.course_deg.value, speed = *in.speed_mps.value;
  const double altitude = *in.relative_altitude_m.value;
  if(!std::isfinite(course) || course < 0 || course >= 360 ||
     !finiteFloat(speed) || speed < 0 || speed > envelope.maximum_speed_mps ||
     !finiteFloat(altitude) || altitude < envelope.minimum_relative_m ||
     altitude > envelope.maximum_relative_m) {
    out.evaluation.reason = Reason::InvalidValue;
    return out;
  }
  out.intent.north = speed * std::cos(course * pi / 180);
  out.intent.east = speed * std::sin(course * pi / 180);
  out.intent.down = std::clamp(-0.5 * (altitude - state.pose.value->relative_home_m), -1.0, 1.0);
  // Canonical zero makes an idle target byte-stable across signs of trig/feedback products.
  if(out.intent.north == 0) out.intent.north = 0;
  if(out.intent.east == 0) out.intent.east = 0;
  if(out.intent.down == 0) out.intent.down = 0;
  out.intent.mask = speed > 0.05 ? 2503 : 3527;
  out.intent.yaw = speed > 0.05 ? course * pi / 180 : 0;
  return out;
}
Build hold(const VehicleState& state, const Envelope& envelope, uint64_t home, Tick now)
{
  Build out;
  out.intent.kind = Kind::Response;
  out.evaluation = guard(state, envelope, home, now, true);
  if(out.evaluation.reason != Reason::Ok) return out;
  out.intent.latitude = static_cast<int32_t>(std::llround(state.global.value->latitude_deg * 1e7));
  out.intent.longitude = static_cast<int32_t>(std::llround(state.global.value->longitude_deg * 1e7));
  out.intent.altitude_msl = state.global.value->altitude_msl_m;
  out.intent.mask = 3576;
  return out;
}
Frame encode(const Intent& intent, const Binding& b, uint32_t boot_ms, mavlink_status_t& status)
{
  mavlink_message_t message{};
  if(intent.kind == Kind::Navigation) {
    mavlink_set_position_target_local_ned_t target{};
    target.time_boot_ms = boot_ms;
    target.target_system = b.target_system; target.target_component = b.target_component;
    target.coordinate_frame = MAV_FRAME_LOCAL_NED; target.type_mask = intent.mask;
    target.vx = intent.north; target.vy = intent.east; target.vz = intent.down; target.yaw = intent.yaw;
    mavlink_msg_set_position_target_local_ned_encode_status(b.source_system, b.source_component,
                                                         &status, &message, &target);
  } else if(intent.kind == Kind::Response) {
    mavlink_set_position_target_global_int_t target{};
    target.time_boot_ms = boot_ms;
    target.target_system = b.target_system; target.target_component = b.target_component;
    target.coordinate_frame = MAV_FRAME_GLOBAL_INT; target.type_mask = intent.mask;
    target.lat_int = intent.latitude; target.lon_int = intent.longitude; target.alt = intent.altitude_msl;
    mavlink_msg_set_position_target_global_int_encode_status(b.source_system, b.source_component,
                                                          &status, &message, &target);
  } else {
    mavlink_ping_t ping{};
    ping.time_usec = uint64_t(boot_ms) * 1000; ping.seq = intent.probe_sequence;
    ping.target_system = b.target_system; ping.target_component = b.target_component;
    mavlink_msg_ping_encode_status(b.source_system, b.source_component, &status, &message, &ping);
  }
  Frame frame;
  frame.size = mavlink_msg_to_send_buffer(frame.bytes.data(), &message);
  return frame;
}
} // namespace mavmoos::egress
