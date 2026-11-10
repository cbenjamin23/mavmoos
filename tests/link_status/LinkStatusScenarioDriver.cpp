#include "LinkStatusFrames.h"
#include "Plugin.h"

#include <dlfcn.h>

#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>

using namespace mavmoos::runtime;
namespace fixture = mavmoos::test::link_status_fixture;

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
    require(key == "MAVMOOS_RADIO_STATUS" || key == "MAVMOOS_CELLULAR_STATUS", "unexpected publication");
    require(posts.emplace(key, value).second, "duplicate status publication");
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

  [[noreturn]] static void unexpected() { throw std::runtime_error("link observer attempted an effect"); }
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
    require(host.posts.size() == (publishes ? 2u : 0u), "unexpected observer cadence");
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
  rig.receive(fixture::radio(rig.codec), 100.04);
  rig.receive(fixture::cellular(rig.codec), 100.06);
  rig.capture("prebind", 100.12);
  rig.heartbeat(100.20);
  rig.receive(fixture::radio(rig.codec), 100.22, 100.26);
  rig.receive(fixture::cellularBase(rig.codec), 100.27);
  rig.receive(fixture::cellular(rig.codec), 100.28);
  rig.capture("fresh", 100.34);
  rig.capture("interval_suppressed", 100.35, false);

  rig.receive(fixture::radio(rig.codec, '3', 'D'), 100.45);
  rig.receive(fixture::cellular(rig.codec, 1, MAV_COMP_ID_TELEMETRY_RADIO), 100.46);
  rig.receive(fixture::radio(rig.codec), 100.21, 100.47);
  rig.receive(fixture::cellular(rig.codec), 100.22, 100.48);
  mavlink_radio_t legacy{};
  legacy.rssi = 98;
  mavlink_message_t legacy_message{};
  mavlink_msg_radio_encode_status(1, 1, &rig.codec, &legacy_message, &legacy);
  rig.receive(legacy_message, 100.49);
  rig.capture("identity_and_order", 100.52);

  auto invalid_radio = fixture::radio(rig.codec);
  invalid_radio.len = 0;
  rig.receive(invalid_radio, 100.55);
  invalid_radio.len = MAVLINK_MSG_ID_RADIO_STATUS_LEN + 1;
  rig.receive(invalid_radio, 100.56);
  invalid_radio.len = MAVLINK_MSG_ID_RADIO_STATUS_LEN;
  invalid_radio.incompat_flags = 0x80;
  rig.receive(invalid_radio, 100.57);
  auto invalid_cellular = fixture::cellular(rig.codec);
  invalid_cellular.magic = MAVLINK_STX_MAVLINK1;
  invalid_cellular.len = MAVLINK_MSG_ID_CELLULAR_STATUS_MIN_LEN;
  rig.receive(invalid_cellular, 100.58);
  auto bad_extension = fixture::cellularPayload();
  bad_extension.band_frequency = std::numeric_limits<float>::infinity();
  rig.receive(fixture::encode(rig.codec, bad_extension), 100.59);
  rig.capture("malformed", 100.66);

  rig.heartbeat(101.20);
  auto cellular = fixture::cellularPayload();
  cellular.status = CELLULAR_STATUS_FLAG_CONNECTED;
  cellular.failure_reason = 254;
  cellular.quality = 191;
  rig.receive(fixture::encode(rig.codec, cellular), 101.25);
  rig.capture("independent_staleness", 101.42);

  rig.capture("heartbeat_loss", 103.30);
  rig.receive(fixture::radio(rig.codec), 103.34);
  rig.receive(fixture::cellular(rig.codec), 103.35);
  rig.capture("rebind_required", 103.45);
  rig.heartbeat(103.51, MAV_AUTOPILOT_PX4);
  rig.receive(fixture::encode(rig.codec, fixture::radioWithoutSignal()), 103.54);
  rig.capture("px4_rebind", 103.62);
  rig.capture("clock_fault", 103.58);

  rig.host.time = 104;
  rig.plugin->onReset("test_reset");
  rig.receive(fixture::radio(rig.codec), 104, 104.03);
  rig.capture("reset_floor", 104.06);
  rig.heartbeat(104.11, MAV_AUTOPILOT_GENERIC);
  rig.receive(fixture::cellular(rig.codec), 104.12);
  rig.capture("unknown_autopilot", 104.20);
}

void trimming(Rig& rig)
{
  rig.heartbeat(100.10);
  rig.codec.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  rig.receive(fixture::radio(rig.codec), 100.20);
  rig.codec.flags = 0;
  rig.capture("v1", 100.25);
  const auto trimmed_radio = fixture::encode(rig.codec, mavlink_radio_status_t{});
  require(trimmed_radio.len == 1, "fixture did not trim the radio base payload");
  rig.receive(trimmed_radio, 100.30);
  mavlink_cellular_status_t partial{};
  partial.id = 5;
  partial.link_tx_rate = 7;
  const auto trimmed_cell = fixture::encode(rig.codec, partial);
  require(trimmed_cell.len == 12, "fixture did not trim a cellular extension mid-field");
  rig.receive(trimmed_cell, 100.31);
  rig.capture("partial_v2", 100.40);

  auto malformed_tower = fixture::cellularPayload();
  malformed_tower.cell_tower_id[0] = '\x01';
  rig.receive(fixture::encode(rig.codec, malformed_tower), 100.48);
  auto invalid_percentage = fixture::radioPayload();
  invalid_percentage.txbuf = 200;
  rig.receive(fixture::encode(rig.codec, invalid_percentage), 100.49);
  rig.capture("field_availability", 100.54);
}

void capacity(Rig& rig)
{
  rig.heartbeat(100.10);
  double at = 100.20;
  for (uint8_t id : {0, 1, 2, 3, 4, 5, 6, 255}) {
    rig.receive(fixture::encode(rig.codec, fixture::cellularPayload(id)), at);
    at += .01;
  }
  rig.receive(fixture::encode(rig.codec, fixture::cellularPayload(254)), at);
  rig.receive(fixture::radio(rig.codec), at + .01);
  auto update = fixture::cellularPayload(255);
  update.quality = 100;
  rig.receive(fixture::encode(rig.codec, update), at + .02);
  rig.capture("capacity", 100.40);

  rig.host.time = 100.50;
  rig.plugin->onReset("vehicle_connection_lost");
  rig.heartbeat(100.60);
  rig.receive(fixture::encode(rig.codec, fixture::cellularPayload(254)), 100.65);
  rig.capture("capacity_reset", 100.70);
}

void verifyProvider(CreatePlugin create, Rig& rig)
{
  require(!create(plugin_abi + 1, &rig.host) && !create(plugin_abi, nullptr), "wrong factory boundary");
  auto* plugin = rig.plugin;
  const auto descriptor = plugin->descriptor();
  require(plugin->name() == "link_status" && descriptor.profile == "mavlink_link_status_observations",
          "wrong provider identity");
  require(descriptor.exclusive_resources == std::vector<std::string>{"observation.link_status"} &&
          descriptor.capabilities.size() == 2 && descriptor.streamed_parameter_bench_compatible,
          "wrong observation descriptor");
  for (const auto& capability : descriptor.capabilities)
    require(capability.access == "observation" && capability.request_variable.empty(), "effectful capability");
  require(plugin->subscriptions().empty() && !plugin->controlBackend() && descriptor.request_owners.empty() &&
          descriptor.observation_streams.empty(), "link observer tried to own an effect");
  require(plugin->configureModule({{"unknown", "1"}}).status == ModuleConfigStatus::UnknownKey,
          "unknown setting accepted");
  require(plugin->configureModule({{"sample_timeout", "NaN"}}).status == ModuleConfigStatus::Invalid,
          "nonfinite timeout accepted");
  require(plugin->configureModule({{"sample_timeout", ".5"}, {"publish_interval", ".6"}}).status ==
          ModuleConfigStatus::Invalid, "interval longer than timeout accepted");
}

} // namespace

int main(int argc, char** argv)
{
  try {
    require(argc == 2, "usage: link_status_scenario_driver MODULE");
    void* library = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
    if (!library) throw std::runtime_error(dlerror());
    const auto create = reinterpret_cast<CreatePlugin>(dlsym(library, "mavmoos_create_plugin"));
    const auto destroy = reinterpret_cast<DestroyPlugin>(dlsym(library, "mavmoos_destroy_plugin"));
    require(create && destroy, "missing ABI exports");
    { Rig rig(create, destroy); verifyProvider(create, rig); lifecycle(rig); }
    { Rig rig(create, destroy); trimming(rig); }
    { Rig rig(create, destroy); capacity(rig); }
    dlclose(library);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
