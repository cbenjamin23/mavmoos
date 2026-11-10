#include "Plugin.h"
#include "TerrainFrames.h"
#include <dlfcn.h>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

using namespace mavmoos::runtime;
namespace wire = mavmoos::test::terrain;

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
    state.autopilot = autopilot;
    state.type = MAV_TYPE_QUADROTOR;
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
  void commandFinished(const std::string&) override { effect(); }
  void cancelCommand(const std::string&) override { effect(); }
  bool flightLog(const FlightLogRequest&) override { effect(); }
  void flightLogFinished(const std::string&) override { effect(); }

  [[noreturn]] static void effect() { throw std::runtime_error("terrain attempted an effectful service"); }
};

struct Rig {
  Host host;
  std::unique_ptr<Plugin, DestroyPlugin> module;
  mavlink_status_t codec{};

  Rig(CreatePlugin create, DestroyPlugin destroy, unsigned autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA)
    : host(autopilot), module(create(plugin_abi, &host), destroy)
  {
    require(bool(module), "current ABI failed to load");
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

  void capture(const char* phase, double at)
  {
    host.time = at;
    host.posts.clear();
    module->iterate();
    require(host.posts.size() == 2, "unexpected set of terrain publications");
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

void observe(Rig& rig)
{
  const auto descriptor = rig.module->descriptor();
  require(rig.module->name() == "terrain" && rig.module->subscriptions().empty() &&
          !rig.module->controlBackend() && descriptor.request_owners.empty() && descriptor.observation_streams.empty() &&
          descriptor.streamed_parameter_bench_compatible && descriptor.capabilities.size() == 2,
          "terrain provider exposes effects or incomplete observations");
  for (const auto& cap : descriptor.capabilities)
    require(cap.access == "observation" && cap.request_variable.empty(), "nonpassive terrain capability");
  require(rig.module->configureModule({{"sample_timeout", ".6"}, {"publish_interval", ".1"}}).status ==
          ModuleConfigStatus::Accepted, "valid terrain timing rejected");
  require(rig.module->configureModule({{"terrain_check", "true"}}).status == ModuleConfigStatus::UnknownKey,
          "unexpected terrain setting accepted");
  require(rig.module->configureModule({{"sample_timeout", ".25"}, {"publish_interval", ".4"}}).status ==
          ModuleConfigStatus::Invalid, "inverted timing bounds accepted");

  rig.capture("boot", 100);
  rig.receive(wire::report(rig.codec), 100.05);
  rig.capture("pre_heartbeat", 100.12);
  rig.heartbeat(100.2);
  rig.receive(wire::report(rig.codec), 100.28, 100.31);
  rig.receive(wire::request(rig.codec), 100.35, 100.36);
  rig.capture("fresh", 100.38);

  const auto inf = std::numeric_limits<float>::infinity();
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  rig.receive(wire::report(rig.codec, 100, 99, 5, 2, 1), 100.4);
  rig.receive(wire::request(rig.codec, 5, 100, 1, 2), 100.41);
  rig.receive(wire::report(rig.codec, 100, inf), 100.42);
  rig.receive(wire::report(rig.codec, 100, 12, nan), 100.43);
  rig.receive(wire::report(rig.codec, 100, 10, 5, 1, 1, 900000001), 100.44);
  rig.receive(wire::request(rig.codec, uint64_t{1} << 56), 100.45);
  rig.receive(wire::request(rig.codec, wire::mask, 0), 100.46);
  rig.receive(wire::request(rig.codec, wire::mask, 10, 1, 1, 0, -1800000001), 100.47);
  auto malformed = wire::report(rig.codec);
  malformed.len = 0;
  rig.receive(malformed, 100.48);
  malformed = wire::request(rig.codec);
  malformed.len = MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN + 1;
  rig.receive(malformed, 100.49);
  rig.receive(wire::report(rig.codec, 1, 777), 100.25, 100.5);
  rig.capture("rejected", 100.52);

  rig.receive(wire::report(rig.codec, 0, 123.5f, 0), 100.6);
  rig.capture("unavailable", 100.65);
  rig.heartbeat(101);
  rig.capture("stale", 101.21);
  rig.capture("session_expired", 103.1);
  rig.receive(wire::report(rig.codec), 103.2);
  rig.capture("data_cannot_rebind", 103.23);
  rig.heartbeat(103.3);

  rig.codec.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  rig.receive(wire::report(rig.codec, 30, 0, 0, 1, 1, 0, 0), 103.35);
  rig.receive(wire::request(rig.codec, 0), 103.36);
  rig.capture("v1_zero_at_equator", 103.42);
  rig.host.time = 104;
  rig.module->onReset("fixture_reset");
  rig.receive(wire::report(rig.codec), 103.95, 104.03);
  rig.capture("explicit_reset", 104.04);

  rig.heartbeat(104.15);
  rig.receive(wire::report(rig.codec), 104.21);
  rig.capture("ready_after_reset", 104.25);
  rig.capture("clock_regression", 104.01);
}
} // namespace

int main(int argc, char** argv)
{
  try {
    require(argc == 2, "usage: terrain_scenario_driver MODULE");
    void* handle = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL | RTLD_NODELETE);
    require(handle, "cannot load terrain module");
    const auto create = reinterpret_cast<CreatePlugin>(dlsym(handle, "mavmoos_create_plugin"));
    const auto destroy = reinterpret_cast<DestroyPlugin>(dlsym(handle, "mavmoos_destroy_plugin"));
    require(create && destroy, "terrain factory missing");
    {
      Host host(MAV_AUTOPILOT_ARDUPILOTMEGA);
      require(!create(plugin_abi - 1, &host) && !create(plugin_abi, nullptr), "incompatible factory input admitted");
      Rig rig(create, destroy);
      observe(rig);
      Rig px4(create, destroy, MAV_AUTOPILOT_PX4);
      px4.heartbeat(100.2);
      px4.receive(wire::report(px4.codec), 100.3);
      px4.capture("px4_shaped_common_report", 100.31);
    }
    dlclose(handle);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
