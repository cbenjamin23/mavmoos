#include "ClockStatusFrames.h"
#include "Plugin.h"

#include <dlfcn.h>

#include <iostream>
#include <map>
#include <stdexcept>

using namespace mavmoos::runtime;
namespace fixture = mavmoos::test::clock_status_fixture;

namespace {

void require(bool condition, const char* reason)
{
  if (!condition) throw std::runtime_error(reason);
}

struct Host final : Services {
  double time = 100;
  Vehicle state;
  std::map<std::string, std::string> posts;

  double now() const override { return time; }
  const Vehicle& vehicle() const override { return state; }
  void publish(const std::string& key, const std::string& value) override
  {
    require(key == "MAVMOOS_SYSTEM_TIME" || key == "MAVMOOS_TIMESYNC_REQUEST" ||
            key == "MAVMOOS_TIMESYNC_REPLY", "unexpected publication");
    require(posts.emplace(key, value).second, "duplicate clock publication");
  }
  void publish(const std::string&, double) override { unexpected(); }
  bool sendCommand(const CommandRequest&) override { unexpected(); }
  bool acquireNavigation(const std::string&) override { unexpected(); }
  void revokeNavigation(const std::string&) override { unexpected(); }
  bool navigation(const NavigationTarget&) override { unexpected(); }
  bool parameter(const ParameterRequest&) override { unexpected(); }
  bool mission(const MissionRequest&) override { unexpected(); }
  void commandFinished(const std::string&) override { unexpected(); }
  void cancelCommand(const std::string&) override { unexpected(); }
  void missionFinished(const std::string&) override { unexpected(); }
  bool suspendNavigation(double) override { unexpected(); }
  bool resumeNavigation(const std::string&) override { unexpected(); }
  bool flightLog(const FlightLogRequest&) override { unexpected(); }

  [[noreturn]] static void unexpected() { throw std::runtime_error("clock observer attempted an effect"); }
};

struct Rig {
  Host host;
  DestroyPlugin destroy;
  Plugin* plugin;
  mavlink_status_t codec{};

  Rig(CreatePlugin create, DestroyPlugin release) : destroy(release), plugin(create(plugin_abi, &host))
  {
    require(plugin, "factory rejected matching ABI");
  }
  ~Rig() { destroy(plugin); }

  void receive(const mavlink_message_t& message, double effective, double read)
  {
    host.time = read;
    plugin->onMessageWithMetadata(message, {effective, read,
      effective == read ? "READ_ACQUISITION" : "QUEUE_EMPTY_FLOOR"});
  }
  void receive(const mavlink_message_t& message, double at) { receive(message, at, at); }

  void heartbeat(double at, uint8_t autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA)
  {
    host.state.heartbeat_valid = true;
    host.state.heartbeat_at = at;
    host.state.type = MAV_TYPE_QUADROTOR;
    host.state.autopilot = autopilot;
    mavlink_heartbeat_t value{};
    value.type = host.state.type;
    value.autopilot = autopilot;
    mavlink_message_t message{};
    mavlink_msg_heartbeat_encode_status(1, 1, &codec, &message, &value);
    receive(message, at);
  }

  void capture(const char* phase, double at, bool publishes = true)
  {
    host.time = at;
    host.posts.clear();
    plugin->iterate();
    require(host.posts.size() == (publishes ? 3u : 0u), "unexpected observer cadence");
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

void lifecycle(Rig& rig)
{
  rig.capture("absent", 100);
  rig.receive(fixture::systemTime(rig.codec), 100.04);
  rig.receive(fixture::request(rig.codec), 100.05);
  rig.capture("prebind", 100.12);
  rig.heartbeat(100.20);
  rig.receive(fixture::systemTime(rig.codec), 100.22, 100.26);
  // An unsolicited reply is observed independently from either request.
  rig.receive(fixture::reply(rig.codec), 100.27);
  rig.capture("reply_first", 100.30);
  rig.receive(fixture::request(rig.codec), 100.34);
  rig.capture("fresh", 100.41);
  rig.capture("interval_suppressed", 100.42, false);

  rig.receive(fixture::systemTime(rig.codec, 2), 100.45);
  rig.receive(fixture::request(rig.codec, 1, MAV_COMP_ID_TELEMETRY_RADIO), 100.46);
  rig.receive(fixture::systemTime(rig.codec), 100.21, 100.47);
  rig.receive(fixture::reply(rig.codec), 100.23, 100.48);
  rig.capture("identity_and_order", 100.53);

  auto invalid_system = fixture::systemTime(rig.codec);
  invalid_system.len = 0;
  rig.receive(invalid_system, 100.55);
  invalid_system.len = MAVLINK_MSG_ID_SYSTEM_TIME_LEN + 1;
  rig.receive(invalid_system, 100.56);
  auto invalid_timesync = fixture::request(rig.codec);
  invalid_timesync.len = MAVLINK_MSG_ID_TIMESYNC_LEN + 1;
  rig.receive(invalid_timesync, 100.57);
  invalid_timesync.len = MAVLINK_MSG_ID_TIMESYNC_MIN_LEN + 1;
  invalid_timesync.magic = MAVLINK_STX_MAVLINK1;
  rig.receive(invalid_timesync, 100.58);
  invalid_timesync.magic = MAVLINK_STX;
  invalid_timesync.incompat_flags = 0x80;
  rig.receive(invalid_timesync, 100.59);
  rig.capture("malformed", 100.66);

  rig.heartbeat(101.20);
  rig.receive(fixture::request(rig.codec), 101.25);
  rig.capture("independent_staleness", 101.42);
  rig.receive(fixture::encode(rig.codec, mavlink_system_time_t{0, 7}), 101.47);
  rig.capture("boot_without_utc", 101.54);

  rig.capture("heartbeat_loss", 103.30);
  rig.receive(fixture::reply(rig.codec), 103.34);
  rig.receive(fixture::systemTime(rig.codec), 103.35);
  rig.capture("rebind_required", 103.45);
  rig.heartbeat(103.51, MAV_AUTOPILOT_PX4);
  rig.receive(fixture::reply(rig.codec), 103.54);
  rig.capture("px4_rebind", 103.62);
  rig.capture("clock_fault", 103.58);

  rig.host.time = 104;
  rig.plugin->onReset("test_reset");
  rig.receive(fixture::request(rig.codec), 104, 104.03);
  rig.capture("reset_floor", 104.06);
  rig.heartbeat(104.11, MAV_AUTOPILOT_GENERIC);
  rig.receive(fixture::systemTime(rig.codec), 104.12);
  rig.capture("unknown_autopilot", 104.20);
}

void wireCases(Rig& rig)
{
  rig.heartbeat(100.10);
  rig.codec.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  rig.receive(fixture::systemTime(rig.codec), 100.19);
  rig.receive(fixture::request(rig.codec), 100.20);
  auto negative_reply = fixture::replyPayload();
  negative_reply.tc1 = -1;
  rig.receive(fixture::encode(rig.codec, negative_reply), 100.21);
  rig.codec.flags = 0;
  rig.capture("v1", 100.25);

  const auto partial_system = fixture::encode(rig.codec, mavlink_system_time_t{1, 0});
  require(partial_system.len == 1, "fixture did not trim a system clock base field");
  rig.receive(partial_system, 100.30);
  const auto partial_request = fixture::encode(rig.codec, mavlink_timesync_t{});
  require(partial_request.len == 1, "fixture did not trim a timesync base field");
  rig.receive(partial_request, 100.31);
  auto component_only = fixture::replyPayload();
  component_only.target_system = 0;
  rig.receive(fixture::encode(rig.codec, component_only), 100.32);
  rig.capture("partial_v2", 100.40);
}

void verifyProvider(CreatePlugin create, Rig& rig)
{
  require(!create(plugin_abi + 1, &rig.host) && !create(plugin_abi, nullptr), "wrong factory boundary");
  auto* plugin = rig.plugin;
  const auto descriptor = plugin->descriptor();
  require(plugin->name() == "clock_status" && descriptor.profile == "mavlink_clock_observations",
          "wrong provider identity");
  require(descriptor.exclusive_resources == std::vector<std::string>{"observation.clock_status"} &&
          descriptor.capabilities.size() == 3 && descriptor.streamed_parameter_bench_compatible,
          "wrong observation descriptor");
  for (const auto& capability : descriptor.capabilities)
    require(capability.access == "observation" && capability.request_variable.empty(), "effectful capability");
  require(plugin->subscriptions().empty() && !plugin->controlBackend() && descriptor.request_owners.empty() &&
          descriptor.observation_streams.empty(), "clock observer tried to own an effect");
  require(plugin->configureModule({{"offset", "1"}}).status == ModuleConfigStatus::UnknownKey,
          "clock offset setting accepted");
  require(plugin->configureModule({{"sample_timeout", "NaN"}}).status == ModuleConfigStatus::Invalid,
          "nonfinite timeout accepted");
  require(plugin->configureModule({{"sample_timeout", ".5"}, {"publish_interval", ".6"}}).status ==
          ModuleConfigStatus::Invalid, "interval longer than timeout accepted");
}

} // namespace

int main(int argc, char** argv)
{
  try {
    require(argc == 2, "usage: clock_status_scenario_driver MODULE");
    void* library = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
    if (!library) throw std::runtime_error(dlerror());
    const auto create = reinterpret_cast<CreatePlugin>(dlsym(library, "mavmoos_create_plugin"));
    const auto destroy = reinterpret_cast<DestroyPlugin>(dlsym(library, "mavmoos_destroy_plugin"));
    require(create && destroy, "missing ABI exports");
    { Rig rig(create, destroy); verifyProvider(create, rig); lifecycle(rig); }
    { Rig rig(create, destroy); wireCases(rig); }
    dlclose(library);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
