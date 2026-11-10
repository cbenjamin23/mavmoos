#include "OpticalFlowFrames.h"
#include "Plugin.h"

#include <dlfcn.h>

#include <iostream>
#include <map>
#include <stdexcept>

using namespace mavmoos::runtime;
namespace fixture = mavmoos::test::optical_flow_fixture;

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
    require(key == "MAVMOOS_OPTICAL_FLOW" || key == "MAVMOOS_OPTICAL_FLOW_RAD", "unexpected publication");
    require(posts.emplace(key, value).second, "duplicate family publication");
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

  [[noreturn]] static void unexpected() { throw std::runtime_error("observer attempted an effect"); }
};

struct Rig {
  Host host;
  DestroyPlugin destroy;
  Plugin* plugin;
  mavlink_status_t codec{};

  Rig(CreatePlugin create, DestroyPlugin release) : destroy(release), plugin(create(plugin_abi, &host))
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
  void receive(const mavlink_message_t& message, double at) { receive(message, at, at); }

  void heartbeat(double at, uint8_t autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA, uint8_t component = 1)
  {
    host.state.heartbeat_valid = true;
    host.state.heartbeat_at = at;
    host.state.type = MAV_TYPE_QUADROTOR;
    host.state.autopilot = autopilot;
    mavlink_heartbeat_t value{};
    value.type = host.state.type;
    value.autopilot = autopilot;
    mavlink_message_t message{};
    mavlink_msg_heartbeat_encode_status(1, component, &codec, &message, &value);
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
  rig.receive(fixture::flow(rig.codec), 100.05);
  rig.capture("prebind", 100.12);
  rig.heartbeat(100.20);
  rig.receive(fixture::flow(rig.codec), 100.21, 100.25);
  rig.receive(fixture::rad(rig.codec), 100.26);
  rig.receive(fixture::trimmedFlow(rig.codec), 100.27);
  rig.capture("fresh", 100.34);
  rig.capture("interval_suppressed", 100.35, false);

  rig.receive(fixture::flow(rig.codec, 2), 100.45);
  rig.receive(fixture::rad(rig.codec, 1, 158), 100.46);
  rig.receive(fixture::flow(rig.codec), 100.205, 100.47);
  auto ignored = fixture::flow(rig.codec);
  ignored.msgid = MAVLINK_MSG_ID_HIL_OPTICAL_FLOW;
  rig.receive(ignored, 100.48);
  rig.capture("identity_and_order", 100.51);

  auto bad_shape = fixture::flow(rig.codec);
  bad_shape.len = 0;
  rig.receive(bad_shape, 100.54);
  bad_shape.len = MAVLINK_MSG_ID_OPTICAL_FLOW_LEN + 1;
  rig.receive(bad_shape, 100.55);
  bad_shape.len = MAVLINK_MSG_ID_OPTICAL_FLOW_LEN;
  bad_shape.incompat_flags = 0x80;
  rig.receive(bad_shape, 100.56);
  rig.codec.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  auto malformed_v1 = fixture::flow(rig.codec);
  rig.codec.flags = 0;
  --malformed_v1.len;
  rig.receive(malformed_v1, 100.57);
  auto invalid_flow = fixture::flowPayload();
  invalid_flow.flow_comp_m_y = std::numeric_limits<float>::quiet_NaN();
  rig.receive(fixture::encode(rig.codec, invalid_flow), 100.58);
  auto invalid_rad = fixture::radPayload();
  invalid_rad.integrated_ygyro = std::numeric_limits<float>::infinity();
  rig.receive(fixture::encode(rig.codec, invalid_rad), 100.59);
  invalid_rad = fixture::radPayload();
  invalid_rad.integrated_y = std::numeric_limits<float>::quiet_NaN();
  rig.receive(fixture::encode(rig.codec, invalid_rad), 100.60);
  rig.capture("malformed", 100.66);

  rig.heartbeat(101.20);
  rig.receive(fixture::trimmedFlow(rig.codec), 101.25);
  auto rad = fixture::radPayload();
  rad.distance = -2;
  rig.receive(fixture::encode(rig.codec, rad), 101.26);
  rig.capture("independent_staleness", 101.40);

  rig.capture("heartbeat_loss", 103.30);
  rig.receive(fixture::rad(rig.codec), 103.35);
  rig.capture("rebind_required", 103.43);
  rig.heartbeat(103.50, MAV_AUTOPILOT_PX4);
  rig.receive(fixture::rad(rig.codec), 103.54);
  rig.capture("px4_rebind", 103.60);
  rig.capture("clock_fault", 103.56);

  rig.host.time = 104;
  rig.plugin->onReset("test_reset");
  rig.receive(fixture::flow(rig.codec), 104, 104.03);
  rig.capture("reset_floor", 104.05);
  rig.heartbeat(104.11, MAV_AUTOPILOT_GENERIC);
  rig.receive(fixture::flow(rig.codec), 104.12);
  rig.capture("unknown_autopilot", 104.18);
  rig.heartbeat(104.21, MAV_AUTOPILOT_ARDUPILOTMEGA, 158);
  rig.receive(fixture::rad(rig.codec), 104.22);
  rig.capture("wrong_heartbeat_component", 104.30);
}

void trimming(Rig& rig)
{
  rig.heartbeat(100.1);
  rig.codec.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  rig.receive(fixture::flow(rig.codec), 100.2);
  rig.receive(fixture::rad(rig.codec), 100.21);
  rig.codec.flags = 0;
  rig.capture("v1", 100.25);

  auto partial = fixture::flowPayload(8);
  partial.flow_rate_x = std::numeric_limits<float>::denorm_min();
  partial.flow_rate_y = 0;
  const auto frame = fixture::encode(rig.codec, partial);
  require(frame.len == 27, "fixture did not trim within a float");
  rig.receive(frame, 100.31);
  mavlink_optical_flow_rad_t minimal{};
  minimal.time_usec = 1;
  const auto minimal_rad = fixture::encode(rig.codec, minimal);
  require(minimal_rad.len == 1, "fixture did not trim into the RAD base fields");
  rig.receive(minimal_rad, 100.32);
  rig.capture("partial_v2", 100.40);
}

void capacity(Rig& rig)
{
  rig.heartbeat(100.1);
  double at = 100.2;
  for (uint8_t id : {255, 1, 22, 8, 9, 63, 109, 200}) {
    rig.receive(fixture::encode(rig.codec, fixture::flowPayload(id)), at);
    at += .01;
  }
  rig.receive(fixture::encode(rig.codec, fixture::flowPayload(99)), at);
  rig.receive(fixture::rad(rig.codec), at + .01);
  auto update = fixture::flowPayload(255);
  update.quality = 255;
  rig.receive(fixture::encode(rig.codec, update), at + .02);
  rig.capture("capacity", 100.4);

  rig.host.time = 100.5;
  rig.plugin->onReset("vehicle_connection_lost");
  rig.heartbeat(100.6);
  rig.receive(fixture::encode(rig.codec, fixture::flowPayload(99)), 100.65);
  rig.capture("capacity_reset", 100.7);
}

void verifyProvider(CreatePlugin create, const Rig& rig)
{
  require(!create(plugin_abi + 1, nullptr), "accepted mismatched ABI or null host");
  require(!create(plugin_abi, nullptr), "accepted a null host");
  auto* plugin = rig.plugin;
  const auto descriptor = plugin->descriptor();
  require(plugin->name() == "optical_flow" && descriptor.profile == "mavlink_optical_flow_observations",
          "wrong provider identity");
  require(descriptor.exclusive_resources == std::vector<std::string>{"observation.optical_flow"} &&
          descriptor.capabilities.size() == 2 && descriptor.streamed_parameter_bench_compatible,
          "wrong descriptor resources or capabilities");
  for (const auto& capability : descriptor.capabilities)
    require(capability.access == "observation" && capability.request_variable.empty(), "effectful capability");
  require(plugin->subscriptions().empty() && !plugin->controlBackend() && descriptor.request_owners.empty() &&
          descriptor.observation_streams.empty(), "provider tried to own an effect or rate request");
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
    require(argc == 2, "usage: optical_flow_scenario_driver MODULE");
    void* library = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
    if (!library) throw std::runtime_error(dlerror());
    const auto create = reinterpret_cast<CreatePlugin>(dlsym(library, "mavmoos_create_plugin"));
    const auto destroy = reinterpret_cast<DestroyPlugin>(dlsym(library, "mavmoos_destroy_plugin"));
    require(create && destroy, "missing ABI exports");
    {
      Rig rig(create, destroy);
      require(!create(plugin_abi + 1, &rig.host), "accepted mismatched ABI with valid host");
      verifyProvider(create, rig);
      lifecycle(rig);
    }
    { Rig rig(create, destroy); trimming(rig); }
    { Rig rig(create, destroy); capacity(rig); }
    dlclose(library);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
