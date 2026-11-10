#include "EscFrames.h"
#include "Plugin.h"
#include <dlfcn.h>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>

namespace {
using namespace mavmoos::runtime;
namespace frames = mavmoos::test::esc;

void require(bool condition, const char* explanation)
{
  if (!condition)
    throw std::runtime_error(explanation);
}

struct Host final : Services {
  double time = 100;
  Vehicle state;
  std::map<std::string, std::string> published;

  Host()
  {
    state.type = MAV_TYPE_QUADROTOR;
    state.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
  }

  double now() const override { return time; }
  const Vehicle& vehicle() const override { return state; }
  void publish(const std::string& key, const std::string& value) override
  {
    require(published.emplace(key, value).second, "duplicate publication in one iterate");
  }
  void publish(const std::string&, double) override { unexpectedEffect(); }
  bool sendCommand(const CommandRequest&) override { unexpectedEffect(); }
  bool acquireNavigation(const std::string&) override { unexpectedEffect(); }
  void revokeNavigation(const std::string&) override { unexpectedEffect(); }
  bool navigation(const NavigationTarget&) override { unexpectedEffect(); }
  bool parameter(const ParameterRequest&) override { unexpectedEffect(); }
  bool mission(const MissionRequest&) override { unexpectedEffect(); }

  [[noreturn]] static void unexpectedEffect()
  {
    throw std::runtime_error("ESC module invoked an effectful service");
  }
};

struct Rig {
  Host host;
  std::unique_ptr<Plugin, DestroyPlugin> plugin;
  mavlink_status_t codec{};

  Rig(CreatePlugin create, DestroyPlugin destroy) : plugin(create(plugin_abi, &host), destroy)
  {
    require(bool(plugin), "ESC factory rejected the matching ABI");
  }

  void frame(const mavlink_message_t& message, double time)
  {
    frame(message, time, time, "READ_ACQUISITION");
  }

  void frame(const mavlink_message_t& message, double effective, double read, const char* basis)
  {
    host.time = read;
    plugin->onMessageWithMetadata(message, {effective, read, basis});
  }

  void heartbeat(double time)
  {
    host.state.heartbeat_at = time;
    host.state.heartbeat_valid = true;
    mavlink_heartbeat_t heartbeat{};
    heartbeat.type = host.state.type;
    heartbeat.autopilot = host.state.autopilot;
    mavlink_message_t message{};
    mavlink_msg_heartbeat_encode_status(1, 1, &codec, &message, &heartbeat);
    frame(message, time);
  }

  void capture(const char* name, double time)
  {
    host.time = time;
    host.published.clear();
    plugin->iterate();
    require(host.published.size() == 3, "expected one publication for each ESC wire family");
    for (const auto& entry : host.published) {
      std::cout << "{\"case\":\"" << name << "\",\"key\":\"" << entry.first
                << "\",\"value\":" << entry.second << "}\n";
    }
  }
};

void checkDescriptor(const Plugin& plugin)
{
  const auto descriptor = plugin.descriptor();
  require(plugin.name() == "esc_status" && plugin.subscriptions().empty() && !plugin.controlBackend(),
          "ESC module exposed an unexpected app surface");
  require(descriptor.request_owners.empty() && descriptor.observation_streams.empty(),
          "ESC descriptor exposed request ownership or stream demands");
  require(descriptor.exclusive_resources == std::vector<std::string>{"observation.esc"} &&
          descriptor.streamed_parameter_bench_compatible &&
          descriptor.capabilities.size() == 3, "ESC descriptor mismatch");
  for (const auto& capability : descriptor.capabilities)
    require(capability.access == "observation" && capability.request_variable.empty() &&
            capability.request_value.empty(), "ESC capability exposed a request");
}

void checkGroups(CreatePlugin create, DestroyPlugin destroy)
{
  Rig rig(create, destroy);
  checkDescriptor(*rig.plugin);
  rig.capture("startup", 100.0);
  rig.frame(frames::info(rig.codec), 100.1);
  rig.capture("before_heartbeat", 100.2);
  rig.heartbeat(100.3);
  rig.frame(frames::info(rig.codec), 100.4, 100.5, "QUEUE_EMPTY_FLOOR");
  rig.frame(frames::status(rig.codec), 100.6);
  rig.frame(frames::telemetry(rig.codec), 100.7);
  rig.capture("first_group", 100.75);
  rig.frame(frames::status(rig.codec, 60), 100.9);
  rig.capture("sparse", 101.0);
  rig.capture("independent_ages", 101.75);
  rig.frame(frames::telemetry(rig.codec, 1, 1, 0), 101.9);
  rig.capture("counter_wrap", 102.0);

  rig.host.time = 102.4;
  rig.plugin->onInputBoundary();
  rig.capture("heartbeat_expired", 102.4);
  rig.heartbeat(102.5);
  rig.capture("new_heartbeat", 102.6);
  rig.frame(frames::telemetry(rig.codec), 102.65);
  rig.capture("new_sample", 102.8);

  rig.host.time = 103.0;
  rig.plugin->onReset("test reset");
  rig.heartbeat(103.1);
  rig.frame(frames::telemetry(rig.codec), 102.9, 103.2, "QUEUE_EMPTY_FLOOR");
  rig.capture("pre_reset_sample", 103.3);
  rig.frame(frames::info(rig.codec), 103.4);
  rig.capture("after_reset", 103.5);

  rig.host.time = 103.45;
  rig.plugin->onInputBoundary();
  rig.heartbeat(103.6);
  rig.frame(frames::info(rig.codec), 103.7);
  rig.capture("clock_fault", 103.8);
  rig.host.time = 103.9;
  rig.plugin->onReset("test clock recovery");
  rig.heartbeat(104.0);
  rig.frame(frames::info(rig.codec), 104.1);
  rig.capture("clock_recovered", 104.2);
}

void checkLimits(CreatePlugin create, DestroyPlugin destroy)
{
  Rig rig(create, destroy);
  rig.heartbeat(100.1);
  for (uint8_t index = 0; index <= 60; index += 4) {
    rig.frame(frames::info(rig.codec, index, 64), 100.2);
    rig.frame(frames::status(rig.codec, index), 100.2);
  }
  rig.capture("full_common_groups", 100.5);
  rig.frame(frames::info(rig.codec, 2, 64), 100.6);
  rig.frame(frames::info(rig.codec, 4, 4), 100.7);
  rig.frame(frames::info(rig.codec, 0, 65), 100.8);
  rig.frame(frames::status(rig.codec, 64), 100.9);
  rig.capture("invalid_group_shapes", 101.0);

  mavlink_esc_info_t empty{};
  mavlink_message_t message{};
  mavlink_msg_esc_info_encode_status(1, 1, &rig.codec, &message, &empty);
  require(message.len == 1, "fixture must exercise a real MAVLink 2 zero-tail encoding");
  rig.frame(message, 101.2);
  rig.capture("count_zero", 101.3);
}

void checkInvalid(CreatePlugin create, DestroyPlugin destroy)
{
  Rig rig(create, destroy);
  rig.heartbeat(100.1);
  rig.frame(frames::status(rig.codec), 100.4, 100.5, "QUEUE_EMPTY_FLOOR");
  rig.capture("before_rejections", 100.55);
  rig.frame(frames::status(rig.codec, 0, 2), 100.6);
  rig.frame(frames::status(rig.codec, 0, 1, 2), 100.7);
  rig.frame(frames::status(rig.codec), 100.3, 100.8, "QUEUE_EMPTY_FLOOR");
  rig.frame(frames::status(rig.codec), 100.4, 100.9, "READ_ACQUISITION");
  auto message = frames::status(rig.codec);
  message.magic = MAVLINK_STX_MAVLINK1;
  rig.frame(message, 101.0);
  message.magic = MAVLINK_STX;
  message.len = MAVLINK_MSG_ID_ESC_STATUS_LEN + 1;
  rig.frame(message, 101.1);
  message.len = 0;
  rig.frame(message, 101.2);
  message = frames::status(rig.codec);
  message.incompat_flags = 0x80;
  rig.frame(message, 101.3);
  rig.capture("rejections", 101.4);
  rig.frame(frames::status(rig.codec), 101.5);
  rig.capture("after_rejections", 101.6);

  Rig px4(create, destroy);
  px4.host.state.autopilot = MAV_AUTOPILOT_PX4;
  px4.heartbeat(100.1);
  px4.frame(frames::status(px4.codec), 100.2);
  px4.capture("unsupported_autopilot", 100.3);
}

void checkSettings(CreatePlugin create, DestroyPlugin destroy)
{
  Rig rig(create, destroy);
  auto status = [&](const std::vector<ModuleSetting>& settings) {
    return rig.plugin->configureModule(settings).status;
  };
  require(status({{"sample_timeout", "0.4"}, {"publish_interval", "0.05"}}) ==
            ModuleConfigStatus::Accepted, "valid ESC settings rejected");
  require(status({{"bogus", "0.4"}}) == ModuleConfigStatus::UnknownKey, "unknown ESC setting accepted");
  require(status({{"sample_timeout", "0.5"}, {"sample_timeout", "0.6"}}) ==
            ModuleConfigStatus::Invalid, "duplicate ESC setting accepted");
  require(status({{"sample_timeout", "nan"}}) == ModuleConfigStatus::Invalid,
          "nonfinite ESC setting accepted");
  require(status({{"sample_timeout", "0.4"}, {"publish_interval", "0.6"}}) ==
            ModuleConfigStatus::Invalid, "publication slower than expiry accepted");
  rig.heartbeat(100.1);
  rig.frame(frames::telemetry(rig.codec), 100.2);
  rig.capture("configured_fresh", 100.3);
  rig.capture("configured_stale", 100.7);
}

} // namespace

int main(int argc, char** argv)
{
  try {
    require(argc == 2, "usage: esc_status_scenario_driver MODULE");
    std::unique_ptr<void, decltype(&dlclose)> library(dlopen(argv[1], RTLD_NOW | RTLD_LOCAL), dlclose);
    if (!library)
      throw std::runtime_error(dlerror());
    auto create = reinterpret_cast<CreatePlugin>(dlsym(library.get(), "mavmoos_create_plugin"));
    auto destroy = reinterpret_cast<DestroyPlugin>(dlsym(library.get(), "mavmoos_destroy_plugin"));
    require(create && destroy, "ESC factory symbols missing");
    Host host;
    require(!create(plugin_abi - 1, &host) && !create(plugin_abi, nullptr),
            "ESC factory accepted wrong ABI or null Services");
    checkGroups(create, destroy);
    checkLimits(create, destroy);
    checkInvalid(create, destroy);
    checkSettings(create, destroy);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
