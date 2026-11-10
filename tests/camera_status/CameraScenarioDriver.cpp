#include "CameraFrames.h"
#include "Plugin.h"
#include <dlfcn.h>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

using namespace mavmoos::runtime;
namespace fixture = mavmoos::test::camera;

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

  [[noreturn]] static void effect() { throw std::runtime_error("camera observation attempted a control effect"); }
};

struct Rig {
  Host host;
  std::unique_ptr<Plugin, DestroyPlugin> module;
  mavlink_status_t codec{};

  Rig(CreatePlugin create, DestroyPlugin destroy) : module(create(plugin_abi, &host), destroy)
  { require(bool(module), "current ABI failed to load camera status"); }

  void receive(const mavlink_message_t& message, double effective, double read)
  {
    host.time = read;
    module->onMessageWithMetadata(message, {effective, read,
      effective == read ? "READ_ACQUISITION" : "QUEUE_EMPTY_FLOOR"});
  }
  void receive(const mavlink_message_t& message, double at) { receive(message, at, at); }

  void heartbeat(double at, uint8_t component = 1)
  {
    const bool fc = component == 1;
    if (fc) {
      host.state.heartbeat_valid = true;
      host.state.heartbeat_at = at;
    }
    const unsigned type = fc ? host.state.type : unsigned(MAV_TYPE_CAMERA);
    const unsigned autopilot = fc ? host.state.autopilot : unsigned(MAV_AUTOPILOT_INVALID);
    mavlink_message_t frame{};
    mavlink_msg_heartbeat_pack_status(1, component, &codec, &frame, type, autopilot, 0, 0, MAV_STATE_STANDBY);
    receive(frame, at);
  }

  void capture(const char* phase, double at)
  {
    host.time = at;
    host.posts.clear();
    module->iterate();
    require(host.posts.size() == 4, "unexpected camera status roster or cadence");
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
  require(rig.module->name() == "camera_status" && rig.module->subscriptions().empty() &&
          !rig.module->controlBackend() && descriptor.request_owners.empty() && descriptor.observation_streams.empty() &&
          descriptor.capabilities.size() == 4 && descriptor.streamed_parameter_bench_compatible,
          "camera status introduced control or a requested stream");
  for (const auto& cap : descriptor.capabilities)
    require(cap.access == "observation" && cap.request_variable.empty(), "nonpassive camera capability");
  require(rig.module->configureModule({}).status == ModuleConfigStatus::Invalid,
          "camera source nomination was optional");
  for (const auto* value : {"0", "1", "256", " 100", "100x"}) {
    bool rejected = false;
    try { rig.module->configure("camera_component", value); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "invalid camera component admitted");
  }
  rig.module->configure("camera_component", "100");
  bool fixed = false;
  try { rig.module->configure("camera_component", "101"); }
  catch (const std::invalid_argument&) { fixed = true; }
  require(fixed, "nominated camera source could change");
  require(rig.module->configureModule({{"sample_timeout", ".6"}, {"publish_interval", ".1"}}).status ==
          ModuleConfigStatus::Accepted, "valid camera timing rejected");
  rig.module->onMail({"MAVMOOS_CAMERA_REQUEST", "action=capture", "fixture"});
  rig.receive(fixture::information(rig.codec), 100.05);
  rig.heartbeat(100.08, fixture::component);
  rig.capture("no_fc_heartbeat", 100.11);
  rig.heartbeat(100.2);
  rig.receive(fixture::information(rig.codec), 100.24, 100.25);
  rig.receive(fixture::settings(rig.codec), 100.26);
  rig.receive(fixture::capture(rig.codec), 100.27);
  rig.receive(fixture::storage(rig.codec), 100.28);
  rig.capture("fresh", 100.31);

  rig.receive(fixture::information(rig.codec, 2), 100.33);
  rig.receive(fixture::settings(rig.codec, 1, 101), 100.34);
  rig.receive(fixture::capture(rig.codec, 1, 1), 100.35);
  rig.receive(fixture::information(rig.codec), 100.23, 100.36);
  rig.receive(fixture::excludedImage(rig.codec), 100.37);
  auto info = fixture::informationPayload();
  info.camera_device_id = 1;
  rig.receive(fixture::encode(rig.codec, info, mavlink_msg_camera_information_encode_status), 100.38);
  auto settings = fixture::settingsPayload();
  settings.zoomLevel = 101;
  rig.receive(fixture::encode(rig.codec, settings, mavlink_msg_camera_settings_encode_status), 100.39);
  auto capture = fixture::capturePayload();
  capture.available_capacity = -1;
  rig.receive(fixture::encode(rig.codec, capture, mavlink_msg_camera_capture_status_encode_status), 100.40);
  rig.receive(fixture::encode(rig.codec, fixture::storagePayload(3, 2), mavlink_msg_storage_information_encode_status),
              100.41);
  auto frame = fixture::settings(rig.codec);
  frame.magic = MAVLINK_STX_MAVLINK1;
  rig.receive(frame, 100.42);
  frame = fixture::information(rig.codec);
  frame.len = MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN + 1;
  rig.receive(frame, 100.43);
  rig.heartbeat(100.44, fixture::component);
  rig.capture("rejected", 100.48);

  settings = {};
  settings.time_boot_ms = 201;
  settings.mode_id = CAMERA_MODE_IMAGE_SURVEY;
  frame = fixture::encode(rig.codec, settings, mavlink_msg_camera_settings_encode_status);
  require(frame.len == MAVLINK_MSG_ID_CAMERA_SETTINGS_MIN_LEN, "old settings did not trim extensions");
  rig.receive(frame, 100.59);
  capture = fixture::capturePayload();
  capture.recording_time_ms = 0;
  capture.image_count = -1;
  rig.receive(fixture::encode(rig.codec, capture, mavlink_msg_camera_capture_status_encode_status), 100.6);
  auto storage = fixture::storagePayload(2);
  storage.status = STORAGE_STATUS_NOT_SUPPORTED;
  storage.total_capacity = std::numeric_limits<float>::infinity();
  storage.used_capacity = -1;
  rig.receive(fixture::encode(rig.codec, storage, mavlink_msg_storage_information_encode_status), 100.62);
  rig.capture("unavailable_fields", 100.65);
  rig.capture("independent_storage_ages", 101.01);

  for (unsigned id = 3; id <= 9; ++id)
    rig.receive(fixture::encode(rig.codec, fixture::storagePayload(id, 12), mavlink_msg_storage_information_encode_status),
                101.08 + id * .01);
  storage = fixture::storagePayload(1, 12);
  storage.time_boot_ms = 999;
  rig.receive(fixture::encode(rig.codec, storage, mavlink_msg_storage_information_encode_status), 101.18);
  rig.capture("bounded_storage", 101.19);

  rig.capture("fc_lost", 102.4);
  rig.heartbeat(102.5, fixture::component);
  rig.receive(fixture::information(rig.codec), 102.52);
  rig.capture("camera_cannot_rebind", 102.55);
  rig.heartbeat(102.61);
  rig.receive(fixture::information(rig.codec), 102.65);
  rig.capture("fc_rebound", 102.71);
  rig.host.time = 102.82;
  rig.module->onReset("fixture_reset");
  rig.receive(fixture::storage(rig.codec), 102.8, 102.86);
  rig.capture("reset", 102.88);
  rig.capture("clock_rollback", 102.73);
}
} // namespace

int main(int argc, char** argv)
{
  try {
    require(argc == 2, "usage: camera_scenario_driver MODULE");
    void* handle = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL | RTLD_NODELETE);
    require(handle, "cannot load camera status module");
    const auto create = reinterpret_cast<CreatePlugin>(dlsym(handle, "mavmoos_create_plugin"));
    const auto destroy = reinterpret_cast<DestroyPlugin>(dlsym(handle, "mavmoos_destroy_plugin"));
    require(create && destroy, "camera status factory missing");
    {
      Host host;
      require(!create(plugin_abi - 1, &host) && !create(plugin_abi, nullptr), "invalid factory arguments admitted");
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
