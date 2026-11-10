#include "EventFrames.h"
#include "Plugin.h"

#include <dlfcn.h>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>

using namespace mavmoos::runtime;
namespace fixture = mavmoos::test::event_timeline_fixture;

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
    require(key == "MAVMOOS_EVENT_TIMELINE" || key == "MAVMOOS_EVENT_SEQUENCE_STATUS" ||
            key == "MAVMOOS_EVENT_REPLAY_ERROR", "unexpected event publication");
    require(posts.emplace(key, value).second, "duplicate event publication");
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
    require(plugin->configureModule({{"sample_timeout", ".65"}}).status == ModuleConfigStatus::Accepted,
            "event observer timeout rejected");
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
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack_status(1, 1, &codec, &message, host.state.type, autopilot, 0, 0, MAV_STATE_STANDBY);
    receive(message, at);
  }

  void batch(unsigned index, double effective, double read)
  {
    for (const auto& message : fixture::frames(codec, index)) receive(message, effective, read);
  }

  void capture(const char* phase, double at, bool publishes = true)
  {
    host.time = at;
    host.posts.clear();
    plugin->iterate();
    require(host.posts.size() == (publishes ? 3u : 0u), "unexpected event publication cadence");
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
  rig.capture("initial", 100);
  rig.batch(15, 100.05, 100.06);
  rig.capture("prebind", 100.12);
  rig.heartbeat(100.20);
  rig.receive(fixture::encode(rig.codec, fixture::current(65530)), 100.21);
  rig.batch(15, 100.22, 100.25);
  rig.capture("before_wrap", 100.29);
  rig.capture("cadence", 100.30, false);
  rig.batch(16, 100.34, 100.36);
  rig.capture("after_wrap", 100.41);

  rig.receive(fixture::encode(rig.codec, fixture::event(3)), 100.44);
  rig.receive(fixture::encode(rig.codec, fixture::event(2)), 100.45);
  rig.receive(fixture::encode(rig.codec, fixture::event(32771)), 100.46);
  rig.receive(fixture::encode(rig.codec, fixture::event(4), 2, 1), 100.47);
  rig.receive(fixture::encode(rig.codec, fixture::current(10), 1, 2), 100.48);
  rig.receive(fixture::encode(rig.codec, fixture::event(4)), 100.33, 100.49);
  auto ignored = fixture::encode(rig.codec, fixture::event(4));
  ignored.msgid = MAVLINK_MSG_ID_REQUEST_EVENT;
  rig.receive(ignored, 100.50);
  auto bad = fixture::encode(rig.codec, fixture::event(4));
  bad.magic = MAVLINK_STX_MAVLINK1;
  rig.receive(bad, 100.51);
  bad = fixture::encode(rig.codec, fixture::error(4, 5));
  bad.len = MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN + 1;
  rig.receive(bad, 100.52);
  rig.receive(fixture::encode(rig.codec, fixture::current(7)), 100.55);
  rig.receive(fixture::encode(rig.codec, fixture::error(5, 30000, 42)), 100.56);
  rig.capture("rejected_and_status", 100.59);

  rig.receive(fixture::encode(rig.codec, fixture::current(9, 0x81)), 100.65);
  rig.capture("reset", 100.70);
  rig.receive(fixture::encode(rig.codec, fixture::event(10)), 100.74);
  rig.receive(fixture::encode(rig.codec, fixture::current(9, 0x81)), 100.75);
  rig.receive(fixture::encode(rig.codec, fixture::event(11)), 100.64, 100.76);
  rig.receive(fixture::encode(rig.codec, fixture::error(11, 12)), 100.64, 100.77);
  rig.capture("same_reset_and_old_mail", 100.82);

  rig.heartbeat(101.0);
  rig.receive(fixture::encode(rig.codec, fixture::event(11)), 101.05);
  rig.receive(fixture::encode(rig.codec, fixture::current(12)), 101.06);
  rig.capture("independent_ages", 101.50);
  rig.capture("heartbeat_loss", 103.10);
  rig.batch(17, 103.15, 103.16);
  rig.capture("unbound_after_loss", 103.22);
  rig.heartbeat(103.3, MAV_AUTOPILOT_PX4);
  rig.batch(18, 103.32, 103.35);
  rig.capture("px4_rebind", 103.40);
  rig.capture("clock_fault", 103.38);
}

void capacityAndTrimming(Rig& rig)
{
  rig.heartbeat(100.1);
  rig.receive(fixture::encode(rig.codec, fixture::current(0, 0)), 100.16);
  mavlink_event_t zero{};
  rig.receive(fixture::encode(rig.codec, zero), 100.17);
  rig.capture("trimmed", 100.18);
  rig.receive(fixture::encode(rig.codec, fixture::current(1)), 100.20);
  rig.capture("advertised_ahead", 100.29);
  rig.receive(fixture::encode(rig.codec, fixture::event(1)), 100.31);
  rig.capture("advertised_caught_up", 100.41);
  for (unsigned sequence = 2; sequence <= 22; ++sequence)
    rig.receive(fixture::encode(rig.codec, fixture::event(sequence)), 100.42 + sequence * .01);
  rig.capture("capacity", 100.68);
  rig.host.time = 100.74;
  rig.plugin->onReset("test_link_boundary");
  rig.capture("explicit_source_reset", 100.80);
}

void verifyDescriptor(CreatePlugin create, Rig& rig)
{
  require(!create(plugin_abi + 1, &rig.host) && !create(plugin_abi, nullptr), "unsafe event factory");
  const auto descriptor = rig.plugin->descriptor();
  require(rig.plugin->name() == "event_timeline" && descriptor.profile == "mavlink_event_timeline_observations" &&
          descriptor.exclusive_resources == std::vector<std::string>{"observation.event_timeline"} &&
          descriptor.capabilities.size() == 3 && descriptor.streamed_parameter_bench_compatible,
          "wrong event provider descriptor");
  for (const auto& capability : descriptor.capabilities)
    require(capability.access == "observation" && capability.request_variable.empty(), "effectful event capability");
  require(rig.plugin->subscriptions().empty() && !rig.plugin->controlBackend() && descriptor.request_owners.empty() &&
          descriptor.observation_streams.empty(), "event provider attempted effect ownership or rate negotiation");
  require(rig.plugin->configureModule({{"unknown", "1"}}).status == ModuleConfigStatus::UnknownKey &&
          rig.plugin->configureModule({{"sample_timeout", "NaN"}}).status == ModuleConfigStatus::Invalid,
          "event observer configuration bounds differ");
}

void printFrames()
{
  mavlink_status_t codec{};
  for (const unsigned batch : {15, 16}) {
    for (const auto& message : fixture::frames(codec, batch)) {
      uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};
      const auto length = mavlink_msg_to_send_buffer(bytes, &message);
      for (unsigned index = 0; index < length; ++index)
        std::cout << std::hex << std::setfill('0') << std::setw(2) << unsigned(bytes[index]);
      std::cout << '\n';
    }
  }
}

} // namespace

int main(int argc, char** argv)
{
  try {
    require(argc == 2, "usage: event_timeline_scenario_driver MODULE|--frames");
    if (std::string_view(argv[1]) == "--frames") {
      printFrames();
      return 0;
    }
    void* library = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
    if (!library) throw std::runtime_error(dlerror());
    const auto create = reinterpret_cast<CreatePlugin>(dlsym(library, "mavmoos_create_plugin"));
    const auto destroy = reinterpret_cast<DestroyPlugin>(dlsym(library, "mavmoos_destroy_plugin"));
    require(create && destroy, "missing event provider ABI exports");
    {
      Rig rig(create, destroy);
      verifyDescriptor(create, rig);
      lifecycle(rig);
    }
    {
      Rig rig(create, destroy);
      capacityAndTrimming(rig);
    }
    dlclose(library);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
