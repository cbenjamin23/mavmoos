/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: Types.h                                        */
/*    DATE: October 29, 2026                                */
/************************************************************/
#pragma once
#include <ardupilotmega/mavlink.h>
#include <array>
#include <chrono>
#include <cstdint>
#include <optional>

namespace mavmoos::egress {
using Tick = int64_t; // steady-clock nanoseconds, never MOOS/simulator/wall time
constexpr Tick second = 1000000000;
constexpr Tick selected_limit = second / 10;
constexpr Tick response_limit = second / 2;

class Clock {
public:
  virtual ~Clock() = default;
  virtual Tick now() const noexcept = 0;
};
class SteadyClock final : public Clock {
public:
  Tick now() const noexcept override
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
};

enum class Reason {
  Ok, Missing, InvalidValue, InvalidTiming, Expired, Guard, HomeMismatch,
  ScopeMismatch, Permission, DuplicateId, ResultPressure, QueueFull,
  ResponseUsed, Closed, ClockFault, RequiresFence, Reconciliation
};
enum class Kind { Navigation, Response, Probe };
enum class FenceClasses { Routine, All };
enum class Outcome {
  Replaced, Cancelled, CancelledOverload, Expired, GateRejected, Accepted,
  WouldBlock, IoError, ShortDatagram, Closed, ClockFault
};
const char* name(Reason value);
const char* name(Outcome value);

struct Binding {
  uint64_t connection = 0, vehicle = 0;
  uint8_t source_system = 0, source_component = 0;
  uint8_t target_system = 0, target_component = 0;
};
struct Scope {
  uint64_t connection = 0, vehicle = 0, generation = 0, home = 0;
};
inline bool operator==(const Scope& a, const Scope& b)
{
  return a.connection == b.connection && a.vehicle == b.vehicle &&
         a.generation == b.generation && a.home == b.home;
}
inline bool operator!=(const Scope& a, const Scope& b) { return !(a == b); }
struct Window {
  uint64_t receipt = 0;
  Tick acquired = 0;
  Tick expires = 0; // original upstream bound, capped again by read-age policy
};
inline bool operator==(const Window& a, const Window& b)
{
  return a.receipt == b.receipt && a.acquired == b.acquired && a.expires == b.expires;
}
template<class T> struct Sample {
  std::optional<T> value;
  Window window;
};
struct Flight {
  bool guided = false, armed = false;
};
inline bool operator==(const Flight& a, const Flight& b)
{ return a.guided == b.guided && a.armed == b.armed; }
enum class Landed { Undefined, OnGround, InAir, Takeoff, Landing };
struct Pose {
  double relative_home_m = 0, north_mps = 0, east_mps = 0, down_mps = 0;
};
inline bool operator==(const Pose& a, const Pose& b)
{
  return a.relative_home_m == b.relative_home_m && a.north_mps == b.north_mps &&
         a.east_mps == b.east_mps && a.down_mps == b.down_mps;
}
struct GlobalPosition {
  double latitude_deg = 0, longitude_deg = 0, altitude_msl_m = 0;
};
inline bool operator==(const GlobalPosition& a, const GlobalPosition& b)
{
  return a.latitude_deg == b.latitude_deg && a.longitude_deg == b.longitude_deg &&
         a.altitude_msl_m == b.altitude_msl_m;
}
struct VehicleState {
  uint64_t revision = 0, home_revision = 0;
  bool home_known = false;
  Sample<Flight> flight;
  Sample<Landed> landed;
  Sample<Pose> pose;
  Sample<GlobalPosition> global;
  Sample<bool> estimator_eligible;
};
struct NavInput {
  uint64_t revision = 0, home_revision = 0;
  Sample<double> course_deg, speed_mps, relative_altitude_m;
};
struct Envelope {
  double maximum_speed_mps = 0, minimum_relative_m = 0, maximum_relative_m = 0;
};
struct Enable {
  Scope scope;
  uint64_t nav_token = 0, probe_token = 0;
  Tick expires = 0;
  uint64_t reconciled_fence = 0; // explicit host attestation, not FC evidence
};
struct ResponseGrant {
  Scope scope;
  uint64_t token = 0;
  Tick triggered = 0, expires = 0;
};
struct Intent {
  Kind kind = Kind::Probe;
  double north = 0, east = 0, down = 0, yaw = 0, altitude_msl = 0;
  int32_t latitude = 0, longitude = 0;
  uint16_t mask = 0;
  uint32_t probe_sequence = 0;
};
struct Frame {
  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> bytes{};
  std::size_t size = 0;
};
struct Evaluation {
  Reason reason = Reason::Ok;
  Tick deadline = 0;
};
struct Build {
  Evaluation evaluation;
  Intent intent;
};
struct Admission {
  uint64_t id = 0;
  Reason reason = Reason::Ok;
  Tick admitted = 0, deadline = 0;
  bool accepted = false;
};
struct Result {
  uint64_t id = 0, related_id = 0;
  Scope scope;
  Kind kind = Kind::Probe;
  Outcome outcome = Outcome::GateRejected;
  Reason reason = Reason::Ok;
  Tick admitted = 0, original_deadline = 0, selected = 0, final_deadline = 0;
  Tick attempt_started = 0, returned = 0, return_clock_sample = 0;
  uint64_t input_revision = 0, observation_revision = 0, gate_revision = 0;
  std::array<Window, 3> input_windows{};
  std::array<Window, 5> observation_windows{};
  Tick permission_expires = 0;
  std::size_t frame_bytes = 0, accepted_bytes = 0;
  int error = 0;
  bool attempted = false, locally_accepted = false, late = false, residual_possible = false;
  bool clock_fault = false;
};
struct Fence {
  uint64_t id = 0;
  Scope scope;
  Tick requested = 0, applied = 0, quiescent = 0;
  bool is_applied = false, is_quiescent = false, residual_possible = false;
  bool all_classes = false;
  Reason reason = Reason::Permission;
  uint64_t accepted_before = 0, last_accepted_id = 0;
};
struct Stats {
  uint64_t admitted = 0, rejected = 0, attempted = 0, accepted = 0;
  uint64_t replaced = 0, expired = 0, cancelled = 0, would_block = 0;
  uint64_t io_errors = 0, short_datagrams = 0, overruns = 0;
  uint64_t rx_datagrams = 0, rx_wrong_peer = 0, rx_truncated = 0, rx_errors = 0;
};
} // namespace mavmoos::egress
