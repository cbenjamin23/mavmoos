// Test-only clocks, fault injection, fixture state and JSONL evidence.
#pragma once
#include "ManagedEgress.h"
#include "UdpLoopbackIo.h"
#include <cerrno>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <vector>
#include <stdexcept>

namespace p2test {
using namespace mavmoos::egress;
using Bytes = std::vector<uint8_t>;
inline void check(bool condition, const std::string& why)
{
  if(!condition) throw std::runtime_error(why);
}

struct FakeClock final : Clock {
  mutable Tick at = 100 * second;
  mutable std::vector<Tick> script;
  mutable std::size_t next = 0;
  Tick now() const noexcept override
  {
    if(next < script.size()) at = script[next++];
    return at;
  }
  void reads(std::initializer_list<Tick> values) { script = values; next = 0; }
};
struct FakeIo final : DatagramIo {
  explicit FakeIo(FakeClock& time) : clock(time) {}
  FakeClock& clock;
  WriteCode code = WriteCode::Returned;
  std::size_t count = std::numeric_limits<std::size_t>::max();
  int error = 0;
  Tick delay = 0;
  bool opened = true, callback_failed = false;
  unsigned calls = 0, closes = 0;
  std::function<void()> on_write;
  std::vector<Frame> frames;
private:
  WriteResult tryWrite(const Frame& frame) noexcept override
  {
    ++calls; frames.push_back(frame);
    try { if(on_write) on_write(); } catch(...) { callback_failed = true; }
    clock.at += delay;
    return {code, count == std::numeric_limits<std::size_t>::max() ? frame.size : count, error};
  }
  ReadResult tryRead() noexcept override { return {}; }
  bool isOpen() const noexcept override { return opened; }
  uint16_t localPort() const noexcept override { return 0; }
  void close() noexcept override { opened = false; ++closes; }
};
inline Window window(Tick acquired, Tick expires, uint64_t receipt = 1)
{ return {receipt, acquired, expires}; }
template<class T> Sample<T> sample(T value, Tick at, Tick age, uint64_t receipt = 1)
{ return {value, window(at, at + age, receipt)}; }
inline Binding binding(uint64_t connection = 1)
{ return {connection, 1, 200, 191, 1, 1}; }
inline Scope scope(uint64_t generation = 1, uint64_t connection = 1, uint64_t home = 1)
{ return {connection, 1, generation, home}; }
inline Envelope envelope() { return {5, 1, 50}; } // laboratory numbers, not vehicle limits
inline VehicleState state(Tick at, uint64_t revision = 1)
{
  VehicleState state;
  state.revision = revision; state.home_revision = 1; state.home_known = true;
  state.flight = sample(Flight{true, true}, at, 2 * second, revision);
  state.landed = sample(Landed::InAir, at, 2 * second, revision);
  state.pose = sample(Pose{10, 0, 0, 0}, at, second / 2, revision);
  state.global = sample(GlobalPosition{0, 0, 123.45}, at, second / 2, revision);
  state.estimator_eligible = sample(true, at, 3 * second, revision);
  return state;
}
inline NavInput input(Tick at, double course = 90, double speed = 2, double altitude = 12)
{
  NavInput in;
  in.revision = in.home_revision = 1;
  in.course_deg = sample(course, at, 2 * second);
  in.speed_mps = sample(speed, at, 2 * second);
  in.relative_altitude_m = sample(altitude, at, 2 * second);
  return in;
}
inline mavlink_message_t decode(const Frame& frame)
{
  mavlink_message_t buffer{}, message{};
  mavlink_status_t status{}, result{};
  unsigned complete = 0;
  for(std::size_t i = 0; i < frame.size; ++i) {
    const auto code = mavlink_frame_char_buffer(&buffer, &status, frame.bytes[i], &message, &result);
    check(code == MAVLINK_FRAMING_INCOMPLETE || code == MAVLINK_FRAMING_OK, "captured CRC");
    if(code == MAVLINK_FRAMING_OK) ++complete;
  }
  check(complete == 1, "one complete MAVLink frame per datagram");
  return message;
}
inline mavlink_set_position_target_local_ned_t local(const Frame& frame)
{
  auto message = decode(frame);
  check(message.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED, "local target ID");
  mavlink_set_position_target_local_ned_t target{};
  mavlink_msg_set_position_target_local_ned_decode(&message, &target);
  return target;
}
inline void near(double actual, double expected, const char* why, double tolerance = 1e-6)
{ check(std::abs(actual - expected) <= tolerance, why); }

class Trace {
public:
  explicit Trace(const std::string& path)
  {
    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    out.open(path);
    check(bool(out), "trace file");
    out << std::boolalpha;
  }
  void begin(const std::string& name) { current = name; }
  void newInstance()
  {
    instance = ++instances;
    out << "{\"event\":\"fixture_instance\",\"case\":\"" << current
        << "\",\"fixture_instance\":" << instance << "}\n";
  }
  void pass()
  {
    ++cases;
    out << "{\"event\":\"case\",\"case\":\"" << current << "\",\"result\":\"PASS\"}\n";
    std::cout << "PASS " << current << '\n';
  }
  void result(const Result& r)
  {
    out << "{\"event\":\"result\",\"case\":\"" << current
        << "\",\"fixture_instance\":" << instance << ",\"id\":" << r.id
        << ",\"connection\":" << r.scope.connection << ",\"vehicle\":" << r.scope.vehicle
        << ",\"generation\":" << r.scope.generation << ",\"home\":" << r.scope.home
        << ",\"kind\":" << int(r.kind) << ",\"outcome\":\"" << name(r.outcome)
        << "\",\"reason\":\"" << name(r.reason) << "\",\"related_id\":" << r.related_id
        << ",\"admitted_ns\":" << r.admitted << ",\"original_deadline_ns\":" << r.original_deadline
        << ",\"selected_ns\":" << r.selected << ",\"final_deadline_ns\":" << r.final_deadline
        << ",\"attempt_started_ns\":" << r.attempt_started << ",\"returned_ns\":" << r.returned
        << ",\"raw_return_clock_ns\":" << r.return_clock_sample
        << ",\"permission_expires_ns\":" << r.permission_expires
        << ",\"input_revision\":" << r.input_revision << ",\"observation_revision\":" << r.observation_revision
        << ",\"gate_revision\":" << r.gate_revision << ",\"frame_bytes\":" << r.frame_bytes
        << ",\"accepted_bytes\":" << r.accepted_bytes << ",\"errno\":" << r.error
        << ",\"attempted\":" << r.attempted << ",\"locally_accepted\":" << r.locally_accepted
        << ",\"late\":" << r.late << ",\"clock_fault\":" << r.clock_fault
        << ",\"residual_possible\":" << r.residual_possible << ",\"input_windows\":[";
    windows(r.input_windows);
    out << "],\"observation_windows\":[";
    windows(r.observation_windows); out << "]}\n";
  }
  void admission(const Admission& a)
  {
    out << "{\"event\":\"admission\",\"case\":\"" << current
        << "\",\"fixture_instance\":" << instance << ",\"id\":" << a.id
        << ",\"accepted\":" << a.accepted << ",\"reason\":\"" << name(a.reason)
        << "\",\"admitted_ns\":" << a.admitted << ",\"deadline_ns\":" << a.deadline << "}\n";
  }
  void fence(const Fence& f)
  {
    out << "{\"event\":\"fence\",\"case\":\"" << current
        << "\",\"fixture_instance\":" << instance << ",\"id\":" << f.id
        << ",\"generation\":" << f.scope.generation << ",\"all_classes\":" << f.all_classes
        << ",\"reason\":\"" << name(f.reason) << "\""
        << ",\"requested_ns\":" << f.requested << ",\"applied_ns\":" << f.applied
        << ",\"quiescent_ns\":" << f.quiescent << ",\"applied\":" << f.is_applied
        << ",\"quiescent\":" << f.is_quiescent << ",\"accepted_before\":" << f.accepted_before
        << ",\"last_accepted_id\":" << f.last_accepted_id
        << ",\"residual_possible\":" << f.residual_possible << "}\n";
  }
  void packet(const Frame& frame, uint16_t from, uint16_t to)
  {
    static const char hex[] = "0123456789abcdef";
    out << "{\"event\":\"packet\",\"case\":\"" << current
        << "\",\"fixture_instance\":" << instance << ",\"source_port\":" << from
        << ",\"destination_port\":" << to << ",\"hex\":\"";
    for(std::size_t i = 0; i < frame.size; ++i)
      out << hex[frame.bytes[i] >> 4] << hex[frame.bytes[i] & 15];
    out << "\"}\n";
  }
  void summary(const char* layer)
  { out << "{\"event\":\"summary\",\"layer\":\"" << layer << "\",\"passed_cases\":" << cases << "}\n"; }
  std::ofstream out;
private:
  template<std::size_t N> void windows(const std::array<Window, N>& values)
  {
    for(std::size_t i = 0; i < N; ++i) {
      if(i) out << ',';
      out << "{\"receipt\":" << values[i].receipt << ",\"acquired_ns\":" << values[i].acquired
          << ",\"expires_ns\":" << values[i].expires << '}';
    }
  }
  std::string current;
  unsigned cases = 0;
  uint64_t instance = 0, instances = 0;
};
struct Rig {
  FakeClock clock;
  FakeIo* io = nullptr;
  std::unique_ptr<ManagedEgress> engine;
  explicit Rig(Trace& trace)
  {
    trace.newInstance();
    auto fake = std::make_unique<FakeIo>(clock);
    io = fake.get();
    engine = std::make_unique<ManagedEgress>(binding(), envelope(), clock, std::move(fake));
    check(engine->updateVehicle(state(clock.at)), "initial fixture observations");
    check(engine->enable({scope(), 1, 2, clock.at + 5 * second, 0}) == Reason::Ok, "fixture enable");
    clock.at += second / 100;
  }
};
inline Admission submit(Rig& r, Trace& trace, uint64_t id, NavInput in)
{
  auto a = r.engine->submitNavigation(id, scope(), 1, in);
  trace.admission(a);
  return a;
}
inline std::vector<Result> drain(ManagedEgress& engine, Trace& trace)
{
  std::vector<Result> results;
  while(auto result = engine.popResult()) { trace.result(*result); results.push_back(*result); }
  return results;
}
} // namespace p2test
