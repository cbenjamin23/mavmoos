#include "ObstacleFrames.h"
#include "Plugin.h"
#include <dlfcn.h>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

using namespace mavmoos::runtime;
namespace fixture = mavmoos::test::obstacles;

namespace {
void require(bool valid, const char* why)
{
  if (!valid) throw std::runtime_error(why);
}

struct Host final : Services {
  double time = 100;
  Vehicle state;
  std::vector<std::pair<std::string, std::string>> posts;

  Host()
  {
    state.type = MAV_TYPE_QUADROTOR;
    state.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
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

  [[noreturn]] static void effect() { throw std::runtime_error("obstacle provider attempted a control effect"); }
};

struct Rig {
  Host host;
  std::unique_ptr<Plugin, DestroyPlugin> module;
  mavlink_status_t codec{};

  Rig(CreatePlugin create, DestroyPlugin destroy) : module(create(plugin_abi, &host), destroy)
  { require(bool(module), "current ABI failed to load distance sensor"); }

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

  void scalar(double at)
  {
    mavlink_message_t frame{};
    const float quaternion[4]{};
    mavlink_msg_distance_sensor_pack_status(1, 1, &codec, &frame, 25, 10, 600, 120,
      MAV_DISTANCE_SENSOR_LASER, 7, MAV_SENSOR_ROTATION_NONE, 255, 0, 0, quaternion, 0);
    receive(frame, at);
  }

  void capture(const char* phase, double at)
  {
    host.time = at;
    host.posts.clear();
    module->iterate();
    require(host.posts.size() == 3, "unexpected publication roster or cadence");
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
  rig.module->configure("distance_sensor_timeout", ".6");
  rig.module->configure("distance_sensor_publish_interval", ".1");
  const auto descriptor = rig.module->descriptor();
  require(rig.module->subscriptions().empty() && !rig.module->controlBackend() &&
          descriptor.request_owners.empty() && descriptor.capabilities.size() == 2 &&
          descriptor.observation_streams.size() == 1 && descriptor.observation_streams[0].message_id == 132,
          "obstacle observation introduced a control or new wire request");
  rig.module->onMail({"MAVMOOS_OBSTACLE_INJECT", "ranges=2", "test"});
  rig.receive(fixture::obstacle(rig.codec), 100.05);
  rig.capture("unbound", 100.1);
  rig.heartbeat(100.15);
  rig.receive(fixture::obstacle(rig.codec), 100.2, 100.24);
  rig.capture("body", 100.26);

  rig.scalar(100.29);
  rig.receive(fixture::obstacle(rig.codec, 2, 1), 100.30);
  rig.receive(fixture::obstacle(rig.codec, 1, 13), 100.31);
  rig.receive(fixture::obstacle(rig.codec), 100.17, 100.32);
  rig.receive(fixture::obstacle(rig.codec), 100.14, 100.33);
  auto p = fixture::payload();
  p.min_distance = 601;
  rig.receive(fixture::encode(rig.codec, p), 100.34);
  p = fixture::payload();
  p.increment_f = 0;
  p.increment = 0;
  rig.receive(fixture::encode(rig.codec, p), 100.35);
  p.increment_f = std::numeric_limits<float>::quiet_NaN();
  rig.receive(fixture::encode(rig.codec, p), 100.36);
  p.increment_f = 361;
  rig.receive(fixture::encode(rig.codec, p), 100.37);
  p.increment_f = -2.5f;
  p.angle_offset = std::numeric_limits<float>::infinity();
  rig.receive(fixture::encode(rig.codec, p), 100.38);
  auto frame = fixture::obstacle(rig.codec);
  frame.magic = MAVLINK_STX_MAVLINK1;
  frame.len = MAVLINK_MSG_ID_OBSTACLE_DISTANCE_MIN_LEN;
  rig.receive(frame, 100.39);
  frame = fixture::obstacle(rig.codec);
  frame.len = MAVLINK_MSG_ID_OBSTACLE_DISTANCE_LEN + 1;
  rig.receive(frame, 100.40);
  rig.capture("rejected", 100.42);

  p = fixture::payload();
  p.frame = MAV_FRAME_GLOBAL;
  p.angle_offset = p.increment_f = 0;
  rig.receive(fixture::encode(rig.codec, p), 100.53);
  rig.capture("default_frame", 100.56);
  p.frame = MAV_FRAME_LOCAL_NED;
  p.sensor_type = 255;
  p.max_distance = UINT16_MAX - 1;
  p.distances[6] = UINT16_MAX - 1;
  rig.receive(fixture::encode(rig.codec, p), 100.68);
  rig.capture("uninterpreted_frame", 100.69);
  rig.heartbeat(101);
  rig.capture("stale", 101.3);
  rig.capture("expired_session", 103.15);
  rig.receive(fixture::obstacle(rig.codec), 103.2);
  rig.capture("data_cannot_rebind", 103.27);
  rig.heartbeat(103.31);
  rig.receive(fixture::obstacle(rig.codec), 103.35);
  rig.capture("new_session", 103.42);
  rig.host.time = 103.6;
  rig.module->onReset("fixture_reset");
  rig.receive(fixture::obstacle(rig.codec), 103.55, 103.65);
  rig.capture("reset", 103.67);
  rig.heartbeat(103.8);
  rig.receive(fixture::obstacle(rig.codec), 103.85);
  rig.capture("after_reset", 103.9);
  rig.capture("clock_rollback", 103.79);
}
} // namespace

int main(int argc, char** argv)
{
  try {
    require(argc == 2, "usage: obstacle_scenario_driver MODULE");
    void* handle = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL | RTLD_NODELETE);
    require(handle, "cannot load distance sensor module");
    const auto create = reinterpret_cast<CreatePlugin>(dlsym(handle, "mavmoos_create_plugin"));
    const auto destroy = reinterpret_cast<DestroyPlugin>(dlsym(handle, "mavmoos_destroy_plugin"));
    require(create && destroy, "distance sensor factory missing");
    {
      Rig rig(create, destroy);
      exercise(rig);
    }
    dlclose(handle);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
