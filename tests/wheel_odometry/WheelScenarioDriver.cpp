#include "Plugin.h"
#include "WheelFrames.h"
#include <dlfcn.h>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

using namespace mavmoos::runtime;
namespace wheel = mavmoos::test::wheels;

namespace {
void require(bool valid, const char* why)
{
  if (!valid) throw std::runtime_error(why);
}

struct Host final : Services {
  double time = 100;
  Vehicle state;
  std::vector<std::pair<std::string, std::string>> posts;

  explicit Host(unsigned autopilot)
  {
    state.type = MAV_TYPE_GROUND_ROVER;
    state.autopilot = autopilot;
  }
  double now() const override { return time; }
  const Vehicle& vehicle() const override { return state; }
  void publish(const std::string& key, const std::string& value) override { posts.emplace_back(key, value); }
  void publish(const std::string&, double) override { effect(); }
  bool sendCommand(const CommandRequest&) override { effect(); }
  bool acquireNavigation(const std::string&) override { effect(); }
  void revokeNavigation(const std::string&) override { effect(); }
  bool navigation(const NavigationTarget&) override { effect(); }
  bool suspendNavigation(double) override { effect(); }
  bool resumeNavigation(const std::string&) override { effect(); }
  bool parameter(const ParameterRequest&) override { effect(); }
  bool mission(const MissionRequest&) override { effect(); }
  void missionFinished(const std::string&) override { effect(); }
  bool flightLog(const FlightLogRequest&) override { effect(); }
  void flightLogFinished(const std::string&) override { effect(); }

  [[noreturn]] static void effect() { throw std::runtime_error("wheel observation attempted a control effect"); }
};

struct Rig {
  Host host;
  std::unique_ptr<Plugin, DestroyPlugin> module;
  mavlink_status_t codec{};

  Rig(CreatePlugin create, DestroyPlugin destroy, unsigned autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA)
    : host(autopilot), module(create(plugin_abi, &host), destroy)
  {
    require(bool(module), "current ABI failed to load odometry");
  }

  void receive(const mavlink_message_t& message, double effective, double read)
  {
    host.time = read;
    module->onMessageWithMetadata(message, {effective, read,
      effective == read ? "READ_ACQUISITION" : "QUEUE_EMPTY_FLOOR"});
  }
  void receive(const mavlink_message_t& message, double at) { receive(message, at, at); }

  void heartbeat(double at)
  {
    host.state.heartbeat_valid = true;
    host.state.heartbeat_at = at;
    mavlink_message_t frame{};
    mavlink_msg_heartbeat_pack_status(1, 1, &codec, &frame, host.state.type, host.state.autopilot, 0, 0, MAV_STATE_STANDBY);
    receive(frame, at);
  }

  void localPosition(double at)
  {
    mavlink_message_t frame{};
    mavlink_msg_local_position_ned_pack_status(1, 1, &codec, &frame, 1, 2, -3, 0, 0, 0, 0);
    receive(frame, at);
  }

  void capture(const char* phase, double at)
  {
    host.time = at;
    host.posts.clear();
    module->iterate();
    require(host.posts.size() == 4, "unexpected observation roster or cadence");
    std::cout << "{\"phase\":\"" << phase << "\",\"posts\":{";
    bool first = true;
    for (const auto& [key, value] : host.posts) {
      if (!first) std::cout << ',';
      first = false;
      std::cout << '"' << key << "\":" << value;
    }
    std::cout << "}}\n";
  }
};

void exercise(Rig& rig)
{
  const auto descriptor = rig.module->descriptor();
  require(rig.module->subscriptions().empty() && !rig.module->controlBackend() &&
          descriptor.request_owners.empty() && descriptor.capabilities.size() == 4 &&
          descriptor.capabilities.back().name == "odometry.wheel_distance" &&
          descriptor.capabilities.back().access == "observation" && descriptor.observation_streams.empty(),
          "wheel observation introduced control or requested output");
  require(rig.module->configureModule({{"sample_timeout", ".6"}, {"publish_interval", ".1"}}).status ==
          ModuleConfigStatus::Accepted, "valid odometry timing rejected");
  rig.module->onMail({"MAVMOOS_WHEEL_TARGET", "speed=2", "fixture"});
  rig.receive(wheel::wheelDistance(rig.codec), 100.05);
  rig.capture("unbound", 100.12);
  rig.heartbeat(100.2);
  rig.receive(wheel::wheelDistance(rig.codec), 100.24, 100.28);
  rig.capture("fresh", 100.31);
  rig.localPosition(100.34);
  rig.receive(wheel::wheelDistance(rig.codec, 2, 1), 100.35);
  rig.receive(wheel::wheelDistance(rig.codec, 1, 2), 100.36);
  rig.receive(wheel::wheelDistance(rig.codec), 100.23, 100.37);

  auto value = wheel::payload();
  value.count = 17;
  rig.receive(wheel::encode(rig.codec, value), 100.38);
  value = wheel::payload();
  value.distance[0] = std::numeric_limits<double>::infinity();
  rig.receive(wheel::encode(rig.codec, value), 100.39);
  value.distance[0] *= -1;
  rig.receive(wheel::encode(rig.codec, value), 100.4);
  auto message = wheel::wheelDistance(rig.codec);
  message.magic = MAVLINK_STX_MAVLINK1;
  rig.receive(message, 100.41);
  message = wheel::wheelDistance(rig.codec);
  message.len = MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN + 1;
  rig.receive(message, 100.42);
  rig.capture("rejected", 100.46);

  value = wheel::payload();
  value.distance[15] = std::numeric_limits<double>::infinity();
  rig.receive(wheel::encode(rig.codec, value), 100.57);
  rig.capture("unused_tail", 100.58);
  value = {};
  value.time_usec = 42;
  value.count = MAVLINK_MSG_WHEEL_DISTANCE_FIELD_DISTANCE_LEN;
  for (unsigned i = 0; i < value.count; ++i) value.distance[i] = i % 2 ? -double(i) : double(i);
  rig.receive(wheel::encode(rig.codec, value), 100.68);
  rig.capture("full", 100.7);
  value = {};
  value.time_usec = 1;
  message = wheel::encode(rig.codec, value);
  require(message.len == 1, "zero-count example must exercise MAVLink2 trimming");
  rig.receive(message, 100.81);
  rig.capture("zero_count", 100.83);
  rig.heartbeat(101);
  rig.capture("stale", 101.43);
  rig.capture("lost", 103.1);
  rig.receive(wheel::wheelDistance(rig.codec), 103.2);
  rig.capture("data_cannot_rebind", 103.23);
  rig.heartbeat(103.3);
  rig.receive(wheel::wheelDistance(rig.codec), 103.37);
  rig.capture("recovered", 103.39);
  rig.host.time = 103.55;
  rig.module->onReset("fixture_reset");
  rig.receive(wheel::wheelDistance(rig.codec), 103.5, 103.56);
  rig.capture("reset", 103.61);
  rig.heartbeat(103.7);
  rig.receive(wheel::wheelDistance(rig.codec), 103.73);
  rig.capture("after_reset", 103.78);
  rig.capture("clock_rollback", 103.6);
}
} // namespace

int main(int argc, char** argv)
{
  try {
    require(argc == 2, "usage: wheel_scenario_driver MODULE");
    void* handle = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL | RTLD_NODELETE);
    require(handle, "cannot load odometry module");
    const auto create = reinterpret_cast<CreatePlugin>(dlsym(handle, "mavmoos_create_plugin"));
    const auto destroy = reinterpret_cast<DestroyPlugin>(dlsym(handle, "mavmoos_destroy_plugin"));
    require(create && destroy, "odometry factory missing");
    {
      Rig rig(create, destroy);
      exercise(rig);
      Rig px4(create, destroy, MAV_AUTOPILOT_PX4);
      px4.heartbeat(100.2);
      px4.receive(wheel::wheelDistance(px4.codec), 100.24);
      px4.capture("px4_shaped", 100.3);
    }
    dlclose(handle);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
