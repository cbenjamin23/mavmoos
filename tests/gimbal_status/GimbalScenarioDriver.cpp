#include "Plugin.h"

#include <cmath>
#include <cstring>
#include <dlfcn.h>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>

using namespace mavmoos::runtime;

namespace {
constexpr unsigned gimbal = 154;
constexpr const char* info_key = "MAVMOOS_GIMBAL_DEVICE_INFORMATION";
constexpr const char* attitude_key = "MAVMOOS_GIMBAL_DEVICE_ATTITUDE_STATUS";

void require(bool value, const char* reason)
{
  if(!value) throw std::runtime_error(reason);
}

struct Host final : Services {
  double time = 100;
  Vehicle state;
  std::map<std::string,std::string> reports;

  double now() const override { return time; }
  const Vehicle& vehicle() const override { return state; }
  void publish(const std::string& key, const std::string& value) override
  {
    require(key == info_key || key == attitude_key, "unexpected provider publication");
    reports[key] = value;
  }
  void publish(const std::string&, double) override { effect(); }
  bool sendCommand(const CommandRequest&) override { effect(); }
  bool acquireNavigation(const std::string&) override { effect(); }
  void revokeNavigation(const std::string&) override { effect(); }
  bool navigation(const NavigationTarget&) override { effect(); }
  bool parameter(const ParameterRequest&) override { effect(); }
  bool mission(const MissionRequest&) override { effect(); }
  void missionFinished(const std::string&) override { effect(); }
  void commandFinished(const std::string&) override { effect(); }
  void cancelCommand(const std::string&) override { effect(); }
  bool suspendNavigation(double) override { effect(); }
  bool resumeNavigation(const std::string&) override { effect(); }

  [[noreturn]] static void effect() { throw std::runtime_error("observer attempted an effect"); }
};

template<typename Report, typename Encode>
mavlink_message_t frame(mavlink_status_t& codec, const Report& report, Encode encode,
                        uint8_t system = 1, uint8_t component = gimbal)
{
  mavlink_message_t result{};
  encode(system, component, &codec, &result, &report);
  return result;
}

mavlink_gimbal_device_information_t information(uint8_t id)
{
  mavlink_gimbal_device_information_t report{};
  report.uid = 18446744073709551000ULL;
  report.time_boot_ms = 910;
  report.firmware_version = 0x04030201;
  report.hardware_version = 0x01010309;
  report.roll_min = -1;
  report.roll_max = 1;
  report.pitch_min = NAN;
  report.pitch_max = 2;
  report.yaw_min = 1;
  report.yaw_max = -1;
  report.cap_flags = GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS | GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK;
  report.cap_flags2 = report.cap_flags | GIMBAL_DEVICE_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL | 0x80000000u;
  report.custom_cap_flags = 123;
  std::memcpy(report.vendor_name, "Gimbal\"Co", 9);
  std::memset(report.model_name, 'X', sizeof(report.model_name));
  report.custom_name[0] = 'C';
  report.custom_name[1] = static_cast<char>(0xff);
  report.gimbal_device_id = id;
  return report;
}

mavlink_gimbal_device_attitude_status_t attitude(uint8_t id)
{
  mavlink_gimbal_device_attitude_status_t report{};
  report.time_boot_ms = 920;
  report.q[0] = .5;
  report.q[1] = -.5;
  report.q[2] = .5;
  report.q[3] = -.5;
  report.angular_velocity_x = -.1f;
  report.angular_velocity_y = NAN;
  report.angular_velocity_z = .3f;
  report.flags = GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME | GIMBAL_DEVICE_FLAGS_ROLL_LOCK |
    GIMBAL_DEVICE_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME | 0x8000;
  report.failure_flags = GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT | GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING |
    0x80000000u;
  report.delta_yaw = -.25f;
  report.delta_yaw_velocity = NAN;
  report.gimbal_device_id = id;
  return report;
}

mavlink_message_t heartbeat(mavlink_status_t& codec, bool device)
{
  mavlink_message_t result{};
  mavlink_msg_heartbeat_pack_status(1, device ? gimbal : 1, &codec, &result,
    device ? MAV_TYPE_GIMBAL : MAV_TYPE_QUADROTOR,
    device ? MAV_AUTOPILOT_INVALID : MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, MAV_STATE_STANDBY);
  return result;
}

struct Rig {
  Host host;
  Plugin* plugin;
  DestroyPlugin destroy;
  mavlink_status_t codec{};

  Rig(CreatePlugin create, DestroyPlugin deleter) : plugin(create(plugin_abi, &host)), destroy(deleter)
  {
    require(plugin, "gimbal factory rejected current ABI");
    host.state.type = MAV_TYPE_QUADROTOR;
    host.state.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
  }
  ~Rig() { destroy(plugin); }

  void receive(const mavlink_message_t& message, double at, double read = -1)
  {
    host.time = read < 0 ? at : read;
    plugin->onMessageWithMetadata(message, {at, host.time, read < 0 ? "READ_ACQUISITION" : "QUEUE_EMPTY_FLOOR"});
  }
  void fcHeartbeat(double at)
  {
    host.state.heartbeat_valid = true;
    host.state.heartbeat_at = at;
    receive(heartbeat(codec, false), at);
  }
  void snap(const char* name, double now)
  {
    host.time = now;
    host.reports.clear();
    plugin->iterate();
    require(host.reports.size() == 2, "gimbal records missing");
    for(const auto& [key, value] : host.reports)
      std::cout << "{\"case\":\"" << name << "\",\"key\":\"" << key << "\",\"value\":" << value << "}\n";
  }
};

void observe(CreatePlugin create, DestroyPlugin destroy)
{
  Rig rig(create, destroy);
  const auto descriptor = rig.plugin->descriptor();
  require(rig.plugin->name() == "gimbal_status" && descriptor.capabilities.size() == 2 &&
          descriptor.request_owners.empty() && descriptor.observation_streams.empty() &&
          rig.plugin->subscriptions().empty() && rig.plugin->controlBackend() == nullptr, "observer descriptor");
  require(rig.plugin->configureModule({}).status == ModuleConfigStatus::Invalid, "missing component accepted");
  for(const auto* value : {"0", "1", "256", "-1", "154.0", " 154"}) {
    bool failed = false;
    try { rig.plugin->configure("gimbal_component", value); }
    catch(const std::invalid_argument&) { failed = true; }
    require(failed, "bad component accepted");
  }
  rig.plugin->configure("gimbal_component", "154");
  require(rig.plugin->configureModule({{"sample_timeout", ".5"}}).status == ModuleConfigStatus::Accepted,
          "bounded sample timeout rejected");
  bool changed = false;
  try { rig.plugin->configure("gimbal_component", "155"); }
  catch(const std::invalid_argument&) { changed = true; }
  require(changed, "fixed component changed");

  auto info = information(0);
  auto orientation = attitude(0);
  rig.receive(heartbeat(rig.codec, true), 100.1);
  rig.receive(frame(rig.codec, info, mavlink_msg_gimbal_device_information_encode_status), 100.15);
  rig.snap("component_only", 100.2);
  rig.fcHeartbeat(100.3);
  rig.receive(frame(rig.codec, info, mavlink_msg_gimbal_device_information_encode_status), 100.25, 100.35);
  rig.receive(frame(rig.codec, info, mavlink_msg_gimbal_device_information_encode_status, 1, 155), 100.36);
  rig.receive(frame(rig.codec, info, mavlink_msg_gimbal_device_information_encode_status, 2), 100.37);
  rig.snap("foreign_or_old", 100.4);
  rig.receive(frame(rig.codec, info, mavlink_msg_gimbal_device_information_encode_status), 100.5);
  rig.receive(frame(rig.codec, orientation, mavlink_msg_gimbal_device_attitude_status_encode_status), 100.55, 100.6);
  rig.snap("bound", 100.62);
  rig.snap("republish", 100.75);

  info.gimbal_device_id = 2;
  info.uid = 0;
  info.cap_flags2 = GIMBAL_DEVICE_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL;
  rig.receive(frame(rig.codec, info, mavlink_msg_gimbal_device_information_encode_status), 100.81);
  orientation.gimbal_device_id = 2;
  orientation.flags = GIMBAL_DEVICE_FLAGS_YAW_LOCK;
  orientation.delta_yaw = 1;
  orientation.delta_yaw_velocity = 0;
  rig.receive(frame(rig.codec, orientation, mavlink_msg_gimbal_device_attitude_status_encode_status), 100.82);
  rig.snap("two_devices", 100.9);

  orientation.flags = GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME | GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME;
  orientation.q[2] = INFINITY;
  rig.receive(frame(rig.codec, orientation, mavlink_msg_gimbal_device_attitude_status_encode_status), 100.95);
  rig.snap("conflicting", 101.02);

  orientation.gimbal_device_id = 0;
  orientation.flags = GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME;
  orientation.q[2] = .5;
  orientation.delta_yaw = 0;
  orientation.delta_yaw_velocity = 0;
  const auto missing_extensions = frame(rig.codec, orientation, mavlink_msg_gimbal_device_attitude_status_encode_status);
  rig.receive(missing_extensions, 101.10);
  auto invalid_wire = missing_extensions;
  invalid_wire.len = 0;
  rig.receive(invalid_wire, 101.12);
  orientation.gimbal_device_id = 7;
  rig.receive(frame(rig.codec, orientation, mavlink_msg_gimbal_device_attitude_status_encode_status), 101.14);
  rig.receive(missing_extensions, 101.05, 101.15);
  rig.snap("expiry_and_malformed", 101.25);

  rig.receive(heartbeat(rig.codec, true), 102.5);
  rig.receive(missing_extensions, 102.52);
  rig.snap("cleared", 102.6);
  rig.fcHeartbeat(102.7);
  rig.receive(missing_extensions, 102.75);
  rig.snap("rebound", 102.8);
  rig.host.time = 103;
  rig.plugin->onReset("MOOS disconnect");
  rig.snap("reset", 103);
}

void printFrame(const char* label, const mavlink_message_t& message)
{
  uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};
  const auto size = mavlink_msg_to_send_buffer(bytes, &message);
  std::cout << label << ' ' << std::hex << std::setfill('0');
  for(unsigned i = 0; i < size; ++i) std::cout << std::setw(2) << unsigned(bytes[i]);
  std::cout << '\n';
}

void frames()
{
  mavlink_status_t codec{};
  for(unsigned count = 0; count < 100; ++count) {
    auto info = information(0);
    auto pose = attitude(0);
    info.time_boot_ms += count;
    pose.time_boot_ms += count;
    printFrame("heartbeat", heartbeat(codec, false));
    printFrame("gimbal_heartbeat", heartbeat(codec, true));
    printFrame("information", frame(codec, info, mavlink_msg_gimbal_device_information_encode_status));
    printFrame("attitude", frame(codec, pose, mavlink_msg_gimbal_device_attitude_status_encode_status));
    info.uid = 999;
    printFrame("wrong_component", frame(codec, info, mavlink_msg_gimbal_device_information_encode_status, 1, 155));
    printFrame("wrong_system", frame(codec, info, mavlink_msg_gimbal_device_information_encode_status, 2));
    mavlink_message_t wrong_family{};
    const char unrelated[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = "unrelated component message";
    mavlink_msg_statustext_pack_status(1, gimbal, &codec, &wrong_family, MAV_SEVERITY_INFO,
                                      unrelated, 0, 0);
    printFrame("wrong_family", wrong_family);
  }
}
}

int main(int argc, char** argv)
{
  try {
    require(argc == 2, "usage: gimbal_status_driver MODULE|--frames");
    if(std::string(argv[1]) == "--frames") {
      frames();
      return 0;
    }
    void* library = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL | RTLD_NODELETE);
    require(library, "gimbal module load failed");
    const auto create = reinterpret_cast<CreatePlugin>(dlsym(library, "mavmoos_create_plugin"));
    const auto destroy = reinterpret_cast<DestroyPlugin>(dlsym(library, "mavmoos_destroy_plugin"));
    require(create && destroy, "missing gimbal module ABI exports");
    Host host;
    require(!create(plugin_abi - 1, &host) && !create(plugin_abi + 1, &host) && !create(plugin_abi, nullptr),
            "wrong ABI or null host accepted");
    observe(create, destroy);
    dlclose(library);
    return 0;
  } catch(const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
