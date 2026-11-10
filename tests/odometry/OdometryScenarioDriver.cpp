#include "OdometryFixture.h"
#include "Plugin.h"
#include <algorithm>
#include <dlfcn.h>
#include <iostream>
#include <iterator>
#include <memory>
#include <stdexcept>

using namespace mavmoos::runtime;
using namespace mavmoos::test::odometry;

namespace {
void require(bool condition, const char* message)
{
  if (!condition) throw std::runtime_error(message);
}

struct Host final : Services {
  double clock = 100;
  Vehicle state;
  std::vector<std::pair<std::string, std::string>> publications;

  double now() const override { return clock; }
  const Vehicle& vehicle() const override { return state; }
  void publish(const std::string& key, const std::string& value) override
  { publications.emplace_back(key, value); }
  void publish(const std::string&, double) override
  { throw std::runtime_error("odometry published a scalar"); }
  bool sendCommand(const CommandRequest&) override
  { throw std::runtime_error("odometry sent a command"); }
  bool acquireNavigation(const std::string&) override
  { throw std::runtime_error("odometry acquired navigation"); }
  void revokeNavigation(const std::string&) override
  { throw std::runtime_error("odometry revoked navigation"); }
  bool navigation(const NavigationTarget&) override
  { throw std::runtime_error("odometry sent navigation"); }
  bool parameter(const ParameterRequest&) override
  { throw std::runtime_error("odometry requested a parameter"); }
  bool mission(const MissionRequest&) override
  { throw std::runtime_error("odometry requested a mission"); }
};

struct Factory {
  CreatePlugin create;
  DestroyPlugin destroy;
};

struct Rig {
  Host host;
  std::unique_ptr<Plugin, DestroyPlugin> plugin;
  mavlink_status_t codec{};

  explicit Rig(const Factory& factory, unsigned autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA)
    : plugin(factory.create(plugin_abi, &host), factory.destroy)
  {
    require(bool(plugin), "odometry factory rejected current ABI");
    host.state.type = MAV_TYPE_QUADROTOR;
    host.state.autopilot = autopilot;
  }

  void receive(const mavlink_message_t& message, double effective_at, double read_at)
  {
    host.clock = std::max(host.clock, read_at);
    plugin->onMessageWithMetadata(message, {effective_at, read_at,
      effective_at == read_at ? "READ_ACQUISITION" : "QUEUE_EMPTY_FLOOR"});
  }

  void receive(const mavlink_message_t& message, double at) { receive(message, at, at); }

  void heartbeat(double at, uint8_t system = 1, uint8_t component = 1)
  {
    mavlink_heartbeat_t payload{};
    payload.type = host.state.type;
    payload.autopilot = host.state.autopilot;
    mavlink_message_t message{};
    mavlink_msg_heartbeat_encode_status(system, component, &codec, &message, &payload);
    if (system == 1 && component == 1) {
      host.state.heartbeat_valid = true;
      host.state.heartbeat_at = at;
    }
    receive(message, at);
  }

  void capture(const char* label, double at)
  {
    host.clock = at;
    host.publications.clear();
    plugin->iterate();
    for (const auto& [key, value] : host.publications)
      std::cout << "{\"case\":\"" << label << "\",\"key\":\"" << key << "\",\"value\":" << value << "}\n";
  }
};

void checkDescriptor(const Rig& rig)
{
  const auto descriptor = rig.plugin->descriptor();
  require(rig.plugin->name() == "odometry" && rig.plugin->subscriptions().empty() &&
          !rig.plugin->controlBackend() && descriptor.request_owners.empty() &&
          descriptor.observation_streams.empty(),
          "unexpected control interface");
  require(descriptor.exclusive_resources == std::vector<std::string>{"observation.odometry"} &&
          descriptor.capabilities.size() == 4 &&
          descriptor.capabilities[2].name == "odometry.local_origin" &&
          descriptor.capabilities.back().name == "odometry.wheel_distance", "missing observation descriptor");
  for (const auto& capability : descriptor.capabilities)
    require(capability.access == "observation" && capability.request_variable.empty() &&
            capability.request_value.empty(), "effectful odometry capability");
}

void families(const Factory& factory)
{
  Rig rig(factory);
  checkDescriptor(rig);
  const auto local = encode(rig.codec, localPayload());
  require(local.magic == MAVLINK_STX && local.len < MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN,
          "local fixture must exercise real MAVLink 2 trailing zero truncation");
  rig.capture("absent", 100);
  rig.heartbeat(100.04, 2);
  rig.receive(local, 100.07);
  rig.capture("before_bound_source", 100.11);

  rig.heartbeat(100.12);
  rig.receive(local, 100.14, 100.30);
  rig.capture("local", 100.31);
  rig.receive(encode(rig.codec, rawPayload()), 100.72);
  rig.capture("both", 100.75);
  rig.capture("local_stale", 101.20);
  rig.receive(encode(rig.codec, localPayload(), 2), 101.25);
  rig.receive(encode(rig.codec, localPayload(), 1, 200), 101.30);
  rig.receive(local, 100.13, 101.32);
  rig.capture("rejected_sources_and_order", 101.34);
}

void shapesAndValues(const Factory& factory)
{
  Rig rig(factory);
  rig.heartbeat(100.1);
  rig.codec.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  const auto local_v1 = encode(rig.codec, localPayload());
  rig.codec.flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  rig.receive(local_v1, 100.2);
  const auto raw = encode(rig.codec, rawPayload());
  rig.receive(raw, 100.2);
  rig.capture("valid_before_bad_frames", 100.24);

  auto bad = local_v1;
  --bad.len;
  rig.receive(bad, 100.25);
  bad = raw;
  bad.magic = MAVLINK_STX_MAVLINK1;
  bad.len = MAVLINK_MSG_ID_ODOMETRY_MIN_LEN;
  rig.receive(bad, 100.26);
  bad = raw;
  bad.len = 0;
  rig.receive(bad, 100.27);
  bad.len = 255;
  rig.receive(bad, 100.28);
  bad = raw;
  bad.incompat_flags = 0x80;
  rig.receive(bad, 100.29);
  bad = raw;
  bad.len = 8;
  rig.receive(bad, 100.30);

  const auto infinity = std::numeric_limits<float>::infinity();
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  auto local = localPayload();
  local.x = infinity;
  rig.receive(encode(rig.codec, local), 100.31);
  auto value = rawPayload();
  value.rollspeed = nan;
  rig.receive(encode(rig.codec, value), 100.32);
  value = rawPayload();
  std::fill(std::begin(value.q), std::end(value.q), 0);
  rig.receive(encode(rig.codec, value), 100.33);
  value = rawPayload();
  value.velocity_covariance[4] = infinity;
  rig.receive(encode(rig.codec, value), 100.34);
  value = rawPayload();
  value.pose_covariance[0] = infinity;
  rig.receive(encode(rig.codec, value), 100.35);
  value = rawPayload();
  value.quality = 101;
  rig.receive(encode(rig.codec, value), 100.36);
  value.quality = -2;
  rig.receive(encode(rig.codec, value), 100.37);
  rig.capture("after_bad_frames", 100.5);

  value = rawPayload();
  value.quality = -1;
  value.time_usec = 100;
  value.reset_counter = 8;
  value.pose_covariance[0] = 0.5;
  rig.receive(encode(rig.codec, value), 100.7);
  rig.capture("explicit_failed_estimate", 100.75);
  value.quality = 0;
  value.reset_counter = 0;
  value.estimator_type = 0;
  value.velocity_covariance[0] = nan;
  const auto without_extensions = encode(rig.codec, value);
  require(without_extensions.len == MAVLINK_MSG_ID_ODOMETRY_MIN_LEN, "expected truncated optional fields");
  rig.receive(without_extensions, 100.9);
  rig.capture("absent_extensions", 100.93);
}

void lifecycle(const Factory& factory)
{
  Rig rig(factory);
  const auto valid = rig.plugin->configureModule({{"sample_timeout", "0.25"}, {"publish_interval", "0.1"}});
  require(valid.status == ModuleConfigStatus::Accepted, "valid module settings rejected");
  require(rig.plugin->configureModule({{"unexpected", "1"}}).status == ModuleConfigStatus::UnknownKey,
          "unknown module setting accepted");
  require(rig.plugin->configureModule({{"sample_timeout", "nan"}}).status == ModuleConfigStatus::Invalid,
          "nonfinite module setting accepted");
  const auto local = encode(rig.codec, localPayload());
  rig.heartbeat(100.1);
  rig.receive(local, 100.20);
  rig.capture("short_fresh", 100.33);
  rig.capture("interval_suppressed", 100.35);
  rig.capture("short_stale", 100.46);

  rig.host.clock = 102.2;
  rig.plugin->onInputBoundary();
  rig.capture("heartbeat_expired", 102.21);
  rig.heartbeat(102.40);
  rig.receive(local, 102.1, 102.41);
  rig.capture("old_floor_after_rebind", 102.42);
  rig.receive(local, 102.55);
  rig.capture("new_after_rebind", 102.56);

  rig.host.clock = 102.7;
  rig.plugin->onReset("odometry_test");
  rig.capture("explicit_reset", 102.71);
  rig.heartbeat(102.90);
  rig.receive(encode(rig.codec, rawPayload()), 102.95);
  rig.capture("new_raw_session", 103.02);
}

void originRecords(const Factory& factory)
{
  Rig rig(factory);
  rig.heartbeat(100.1);
  const auto origin = encode(rig.codec, originPayload());
  require(origin.len == MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN - 1,
          "origin fixture should trim the high zero timestamp byte");
  rig.receive(origin, 100.13, 100.15);
  rig.capture("origin_v2", 100.19);

  auto invalid = originPayload();
  invalid.latitude = std::numeric_limits<int32_t>::min();
  rig.receive(encode(rig.codec, invalid), 100.20);
  invalid = originPayload();
  invalid.longitude = 1800000001;
  rig.receive(encode(rig.codec, invalid), 100.21);
  rig.receive(encode(rig.codec, originPayload(), 2), 100.22);
  rig.receive(encode(rig.codec, originPayload(), 1, 200), 100.23);
  rig.receive(origin, 100.12, 100.24);

  rig.codec.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  auto origin_v1 = originPayload();
  origin_v1.latitude = -900000000;
  origin_v1.longitude = 1800000000;
  origin_v1.altitude = -3456;
  const auto legacy = encode(rig.codec, origin_v1);
  rig.codec.flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  require(legacy.magic == MAVLINK_STX_MAVLINK1 && legacy.len == MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN,
          "legacy origin fixture should omit the timestamp extension");
  auto malformed = legacy;
  --malformed.len;
  rig.receive(malformed, 100.25);
  malformed = origin;
  malformed.len = MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN + 1;
  rig.receive(malformed, 100.26);
  rig.capture("origin_rejected", 100.32);

  rig.receive(encode(rig.codec, localPayload()), 100.90);
  rig.capture("origin_stale", 101.16);
  rig.receive(legacy, 101.30);
  rig.capture("origin_v1", 101.36);
  const auto zero = encode(rig.codec, mavlink_gps_global_origin_t{});
  require(zero.magic == MAVLINK_STX && zero.len == 1, "zero origin fixture must exercise valid trimming");
  rig.receive(zero, 101.50);
  rig.capture("origin_zero", 101.55);

  rig.host.clock = 102.2;
  rig.plugin->onInputBoundary();
  rig.capture("origin_lost", 102.22);
  rig.heartbeat(102.40);
  rig.receive(origin, 102.50);
  rig.capture("origin_rebound", 102.51);
  rig.host.clock = 102.7;
  rig.plugin->onReset("origin_test");
  rig.capture("origin_reset", 102.71);
}

void sources(const Factory& factory)
{
  for (const auto autopilot : {MAV_AUTOPILOT_PX4, MAV_AUTOPILOT_GENERIC}) {
    Rig rig(factory, autopilot);
    rig.heartbeat(100.1);
    rig.receive(encode(rig.codec, localPayload()), 100.2);
    rig.receive(encode(rig.codec, px4RawPayload()), 100.3);
    rig.receive(encode(rig.codec, originPayload()), 100.31);
    rig.capture(autopilot == MAV_AUTOPILOT_PX4 ? "px4_fixture" : "unsupported_autopilot", 100.4);
  }
}
} // namespace

int main(int argc, char** argv)
{
  try {
    require(argc == 2, "usage: odometry_scenario_driver PLUGIN");
    void* library = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
    if (!library) throw std::runtime_error(dlerror());
    const Factory factory{
      reinterpret_cast<CreatePlugin>(dlsym(library, "mavmoos_create_plugin")),
      reinterpret_cast<DestroyPlugin>(dlsym(library, "mavmoos_destroy_plugin"))};
    require(factory.create && factory.destroy, "missing odometry factory");
    Host host;
    require(!factory.create(plugin_abi - 1, &host) && !factory.create(plugin_abi, nullptr),
            "odometry factory accepted an invalid ABI or host");
    families(factory);
    shapesAndValues(factory);
    lifecycle(factory);
    originRecords(factory);
    sources(factory);
    require(dlclose(library) == 0, "odometry fixture could not unload module");
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
