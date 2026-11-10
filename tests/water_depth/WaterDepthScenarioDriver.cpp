#include "Plugin.h"
#include "WaterDepthFrames.h"

#include <dlfcn.h>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

using namespace mavmoos::runtime;
namespace wire = mavmoos::test::water_depth;

namespace {
void require(bool valid, const char* reason)
{
  if (!valid)
    throw std::runtime_error(reason);
}

struct Host final : Services {
  double time = 100;
  Vehicle state;
  std::vector<std::pair<std::string, std::string>> posts;

  Host(uint8_t type, uint8_t autopilot)
  {
    state.type = type;
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
  void commandFinished(const std::string&) override { effect(); }
  void cancelCommand(const std::string&) override { effect(); }
  bool flightLog(const FlightLogRequest&) override { effect(); }
  void flightLogFinished(const std::string&) override { effect(); }

  [[noreturn]] static void effect() { throw std::runtime_error("water_depth attempted an effectful service"); }
};

struct Rig {
  Host host;
  std::unique_ptr<Plugin, DestroyPlugin> module;
  mavlink_status_t codec{};

  Rig(CreatePlugin create, DestroyPlugin destroy, uint8_t type = MAV_TYPE_SURFACE_BOAT,
      uint8_t autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA)
    : host(type, autopilot), module(create(plugin_abi, &host), destroy)
  {
    require(bool(module), "current ABI failed to load");
  }

  void receive(const mavlink_message_t& message, double effective, double read)
  {
    host.time = read;
    module->onMessageWithMetadata(message,
      {effective, read, effective == read ? "READ_ACQUISITION" : "QUEUE_EMPTY_FLOOR"});
  }

  void receive(const mavlink_message_t& message, double at) { receive(message, at, at); }

  void heartbeat(double at)
  {
    host.state.heartbeat_valid = true;
    host.state.heartbeat_at = at;
    receive(wire::heartbeat(codec, host.state.type, host.state.autopilot), at);
  }

  void capture(const char* phase, double at)
  {
    host.time = at;
    host.posts.clear();
    module->iterate();
    require(host.posts.size() == 1 && host.posts.front().first == "MAVMOOS_WATER_DEPTH",
            "water_depth published outside its sole observation variable");
    std::cout << "{\"phase\":\"" << phase << "\",\"sample\":" << host.posts.front().second << "}\n";
  }
};

void supportedBoat(Rig& rig)
{
  const auto descriptor = rig.module->descriptor();
  require(rig.module->name() == "water_depth" && rig.module->subscriptions().empty() &&
          !rig.module->controlBackend() && descriptor.request_owners.empty() &&
          descriptor.observation_streams.empty() && descriptor.streamed_parameter_bench_compatible &&
          descriptor.profile == "ardupilot_surface_water_depth" && descriptor.capabilities.size() == 1 &&
          descriptor.capabilities.front().name == "water_depth.water_depth" &&
          descriptor.capabilities.front().access == "observation" &&
          descriptor.capabilities.front().request_variable.empty(),
          "water_depth advertises effects or an incorrect passive descriptor");
  require(rig.module->configureModule({{"sample_timeout", ".6"}, {"publish_interval", ".1"}}).status ==
          ModuleConfigStatus::Accepted, "valid observation timing rejected");
  require(rig.module->configureModule({{"use_for_nav", "true"}}).status == ModuleConfigStatus::UnknownKey,
          "unexpected effects setting accepted");
  require(rig.module->configureModule({{"sample_timeout", ".25"}, {"publish_interval", ".4"}}).status ==
          ModuleConfigStatus::Invalid, "inverted observation timing admitted");

  const auto initial = wire::generatedFrames(rig.codec);
  rig.capture("boot", 100);
  rig.receive(initial[0], 100.05);
  rig.capture("pre_heartbeat", 100.11);
  rig.heartbeat(100.20);
  rig.receive(initial[0], 100.27, 100.30);
  rig.receive(initial[1], 100.33);
  rig.capture("fresh", 100.35);

  rig.receive(wire::encode(rig.codec, wire::reading(), 2, 1), 100.40);
  rig.receive(wire::encode(rig.codec, wire::reading(), 1, 2), 100.41);
  auto malformed = initial[0];
  malformed.len = MAVLINK_MSG_ID_WATER_DEPTH_LEN + 1;
  rig.receive(malformed, 100.42);
  malformed.len = 0;
  rig.receive(malformed, 100.43);
  malformed = initial[0];
  malformed.magic = MAVLINK_STX_MAVLINK1;
  rig.receive(malformed, 100.44);
  malformed = initial[0];
  malformed.incompat_flags = 0x80;
  rig.receive(malformed, 100.45);
  rig.receive(wire::encode(rig.codec, wire::reading(10)), 100.46);
  rig.receive(wire::encode(rig.codec, wire::reading(255)), 100.47);
  auto old = wire::reading();
  old.distance = 99;
  rig.receive(wire::encode(rig.codec, old), 100.26, 100.48);
  rig.capture("rejected", 100.51);

  auto unknown = wire::reading(9);
  unknown.healthy = 7;
  unknown.lat = -900000001;
  unknown.lng = 1800000000;
  unknown.alt = std::numeric_limits<float>::infinity();
  unknown.roll = std::numeric_limits<float>::quiet_NaN();
  unknown.yaw = -std::numeric_limits<float>::infinity();
  unknown.distance = std::numeric_limits<float>::quiet_NaN();
  unknown.temperature = std::numeric_limits<float>::infinity();
  rig.receive(wire::encode(rig.codec, unknown), 100.57);
  rig.capture("unknown", 100.64);

  auto negative = wire::reading();
  negative.lat = negative.lng = 0;
  negative.distance = -3.25f;
  rig.receive(wire::encode(rig.codec, negative), 100.66);
  rig.capture("negative", 100.77);
  auto zero = wire::reading(5);
  zero.lat = 0;
  zero.distance = zero.temperature = 0;
  rig.receive(wire::encode(rig.codec, zero), 100.82);
  rig.capture("zero", 100.89);

  rig.heartbeat(101.05);
  rig.capture("independent_staleness", 101.30);
  rig.capture("session_expired", 103.20);
  rig.receive(initial[0], 103.23);
  rig.capture("data_cannot_rebind", 103.33);
  rig.heartbeat(103.40);
  const auto empty_wire = wire::encode(rig.codec, mavlink_water_depth_t{});
  require(empty_wire.len == 1, "generated all-zero MAVLink2 frame was not tail-trimmed");
  rig.receive(empty_wire, 103.45);
  rig.capture("trimmed_zero", 103.51);

  rig.host.time = 103.58;
  rig.module->onReset("fixture_reset");
  rig.receive(initial[0], 103.52, 103.61);
  rig.capture("explicit_reset", 103.65);
  rig.heartbeat(103.70);
  rig.receive(initial[0], 103.76);
  rig.capture("ready_after_reset", 103.80);
  rig.host.state.type = MAV_TYPE_GROUND_ROVER;
  rig.heartbeat(103.90);
  rig.receive(initial[0], 103.94);
  rig.capture("vehicle_changed", 104.00);
}

void unsupported(Rig& rig, const char* phase)
{
  rig.heartbeat(100.20);
  rig.receive(wire::encode(rig.codec, wire::reading()), 100.30);
  rig.capture(phase, 100.40);
}
} // namespace

int main(int argc, char** argv)
{
  try {
    require(argc == 2, "usage: water_depth_scenario_driver MODULE");
    void* handle = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL | RTLD_NODELETE);
    require(handle, "cannot load water_depth module");
    const auto create = reinterpret_cast<CreatePlugin>(dlsym(handle, "mavmoos_create_plugin"));
    const auto destroy = reinterpret_cast<DestroyPlugin>(dlsym(handle, "mavmoos_destroy_plugin"));
    require(create && destroy, "water_depth factory missing");
    {
      Host host(MAV_TYPE_SURFACE_BOAT, MAV_AUTOPILOT_ARDUPILOTMEGA);
      require(!create(plugin_abi - 1, &host) && !create(plugin_abi, nullptr), "incompatible factory input admitted");
      Rig boat(create, destroy);
      supportedBoat(boat);
      Rig ground(create, destroy, MAV_TYPE_GROUND_ROVER);
      unsupported(ground, "ground_rover");
      Rig sub(create, destroy, MAV_TYPE_SUBMARINE);
      unsupported(sub, "ardusub");
      Rig px4(create, destroy, MAV_TYPE_SURFACE_BOAT, MAV_AUTOPILOT_PX4);
      unsupported(px4, "px4_boat");
    }
    dlclose(handle);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
