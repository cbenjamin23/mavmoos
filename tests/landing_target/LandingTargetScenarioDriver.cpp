#include "Plugin.h"
#include <dlfcn.h>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace mavmoos::runtime;

namespace {

void require(bool value, const char* reason)
{
  if (!value) throw std::runtime_error(reason);
}

struct Host final : Services {
  double time = 100;
  Vehicle state;
  std::string report;

  double now() const override { return time; }
  const Vehicle& vehicle() const override { return state; }
  void publish(const std::string& key, const std::string& value) override
  {
    require(key == "MAVMOOS_LANDING_TARGET", "unexpected publication");
    report = value;
  }
  void publish(const std::string&, double) override { unexpected(); }
  bool sendCommand(const CommandRequest&) override { unexpected(); }
  bool acquireNavigation(const std::string&) override { unexpected(); }
  void revokeNavigation(const std::string&) override { unexpected(); }
  bool navigation(const NavigationTarget&) override { unexpected(); }
  bool parameter(const ParameterRequest&) override { unexpected(); }
  bool mission(const MissionRequest&) override { unexpected(); }
  void missionFinished(const std::string&) override { unexpected(); }
  void commandFinished(const std::string&) override { unexpected(); }
  void cancelCommand(const std::string&) override { unexpected(); }
  bool recordStoredMissionProof(const std::string&, const std::vector<MissionItem>&) override { unexpected(); }
  void invalidateStoredMissionProof(const std::string&) override { unexpected(); }
  bool beginMissionExecution(const std::string&, uint64_t, const std::string&, double) override { unexpected(); }
  bool beginMissionExecutionAbort(const std::string&, const std::string&, double) override { unexpected(); }
  bool finishMissionExecution(const std::string&, const std::string&) override { unexpected(); }
  bool suspendNavigation(double) override { unexpected(); }
  bool resumeNavigation(const std::string&) override { unexpected(); }

  [[noreturn]] static void unexpected() { throw std::runtime_error("unexpected effectful service"); }
};

struct Rig {
  Host host;
  Plugin* plugin;
  DestroyPlugin destroy;
  mavlink_status_t codec{};

  Rig(CreatePlugin create, DestroyPlugin destroy)
    : plugin(create(plugin_abi, &host)), destroy(destroy)
  {
    require(plugin, "factory rejected current ABI");
  }
  ~Rig() { destroy(plugin); }

  void receive(const mavlink_message_t& message, double effective_at, double read_at)
  {
    host.time = read_at;
    plugin->onMessageWithMetadata(message, {effective_at, read_at,
      effective_at == read_at ? "READ_ACQUISITION" : "QUEUE_EMPTY_FLOOR"});
  }
  void receive(const mavlink_message_t& message, double time) { receive(message, time, time); }

  void heartbeat(double at, uint8_t component = 1, uint8_t type = MAV_TYPE_QUADROTOR)
  {
    host.state.heartbeat_valid = true;
    host.state.heartbeat_at = at;
    host.state.type = type;
    host.state.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    mavlink_heartbeat_t value{};
    value.type = type;
    value.autopilot = host.state.autopilot;
    mavlink_message_t message{};
    mavlink_msg_heartbeat_encode_status(1, component, &codec, &message, &value);
    receive(message, at);
  }

  mavlink_message_t frame(const mavlink_landing_target_t& value, uint8_t system = 1,
                          uint8_t component = 158, bool v1 = false)
  {
    mavlink_message_t message{};
    codec.flags = v1 ? MAVLINK_STATUS_FLAG_OUT_MAVLINK1 : 0;
    mavlink_msg_landing_target_encode_status(system, component, &codec, &message, &value);
    codec.flags = 0;
    return message;
  }

  void capture(const char* phase, double at)
  {
    host.time = at;
    host.report.clear();
    plugin->iterate();
    require(!host.report.empty(), "expected observer publication");
    std::cout << "{\"phase\":\"" << phase << "\",\"report\":" << host.report << "}\n";
  }
};

void exercise(Rig& rig)
{
  require(rig.plugin->name() == "landing_target", "wrong module name");
  const auto descriptor = rig.plugin->descriptor();
  require(descriptor.capabilities.size() == 1 &&
          descriptor.capabilities.front().name == "observations.landing_target" &&
          descriptor.request_owners.empty() && descriptor.observation_streams.empty(),
          "landing_target descriptor must be passive");
  require(rig.plugin->configureModule({}).status == ModuleConfigStatus::Invalid,
          "missing configured component accepted");
  for (const auto* bad : {"0", "256", "12x", "-1", ""}) {
    bool invalid = false;
    try { rig.plugin->configure("landing_target_component", bad); }
    catch (const std::invalid_argument&) { invalid = true; }
    require(invalid, "invalid source component accepted");
  }
  rig.plugin->configure("landing_target_component", "158");
  require(rig.plugin->configureModule({}).status == ModuleConfigStatus::Accepted, "source config rejected");

  mavlink_landing_target_t angular{};
  angular.time_usec = std::numeric_limits<uint64_t>::max();
  angular.target_num = 7;
  angular.frame = MAV_FRAME_BODY_FRD;
  angular.distance = 2.5;
  angular.angle_x = .125;
  angular.angle_y = -.25;

  rig.capture("startup", 100);
  rig.receive(rig.frame(angular), 100.01);
  rig.heartbeat(100.02, 158);
  rig.receive(rig.frame(angular), 100.05);
  rig.capture("sensor_cannot_bind", 100.11);
  rig.heartbeat(100.12, 1, MAV_TYPE_FIXED_WING);
  rig.receive(rig.frame(angular), 100.15);
  rig.capture("plane_cannot_bind", 100.23);
  rig.heartbeat(100.24);
  rig.receive(rig.frame(angular, 2), 100.25);
  rig.receive(rig.frame(angular, 1, 1), 100.26);
  rig.receive(rig.frame(angular, 1, 158, true), 100.27, 100.30);
  rig.capture("angular_v1", 100.35);

  auto pose = angular;
  pose.target_num = 8;
  pose.position_valid = 1;
  pose.x = .5;
  pose.y = -.125;
  pose.z = 2;
  pose.angle_x = std::numeric_limits<float>::quiet_NaN();
  rig.receive(rig.frame(pose), 100.41, 100.45);
  rig.capture("pose_v2", 100.47);
  auto malformed = pose;
  malformed.position_valid = 2;
  rig.receive(rig.frame(malformed), 100.49);
  malformed = pose;
  malformed.z = std::numeric_limits<float>::infinity();
  rig.receive(rig.frame(malformed), 100.50);
  auto bad_wire = rig.frame(pose);
  bad_wire.len = MAVLINK_MSG_ID_LANDING_TARGET_LEN + 1;
  rig.receive(bad_wire, 100.51);
  rig.receive(rig.frame(angular), 100.37, 100.52);
  rig.capture("preserved_after_bad_data", 100.59);
  rig.capture("stale", 101.0);

  rig.host.time = 101.11;
  rig.host.state.heartbeat_valid = false;
  rig.plugin->onInputBoundary();
  rig.receive(rig.frame(pose), 101.12);
  rig.capture("after_heartbeat_loss", 101.14);
  rig.heartbeat(101.27);
  const auto trimmed = rig.frame({});
  require(trimmed.len == 1, "expected a valid trimmed v2 frame");
  rig.receive(trimmed, 101.29);
  rig.capture("trimmed_v2", 101.36);
  rig.host.time = 101.46;
  rig.plugin->onReset("test_reset");
  rig.receive(rig.frame(pose), 101.35, 101.48);
  rig.receive(rig.frame(pose), 101.49);
  rig.capture("after_reset", 101.53);
}

} // namespace

int main(int argc, char** argv)
{
  try {
    require(argc == 2, "usage: landing_target_scenario_driver <module>");
    void* module = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
    require(module, "could not open landing_target module");
    const auto create = reinterpret_cast<CreatePlugin>(dlsym(module, "mavmoos_create_plugin"));
    const auto destroy = reinterpret_cast<DestroyPlugin>(dlsym(module, "mavmoos_destroy_plugin"));
    require(create && destroy, "missing plugin entry points");
    {
      Rig rig(create, destroy);
      require(!create(plugin_abi + 1, &rig.host) && !create(plugin_abi, nullptr), "invalid factory arguments accepted");
      exercise(rig);
    }
    {
      Rig rig(create, destroy);
      rig.plugin->configure("landing_target_component", "1");
      require(rig.plugin->configureModule({}).status == ModuleConfigStatus::Accepted, "FC component rejected");
      rig.heartbeat(100.1);
      mavlink_landing_target_t angular{};
      angular.angle_x = .125;
      angular.distance = -1;
      rig.receive(rig.frame(angular, 1, 1), 100.2);
      rig.capture("fc_component", 100.25);
    }
    dlclose(module);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
