#include "DebugFrames.h"
#include "Plugin.h"

#include <dlfcn.h>

#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>

using namespace mavmoos::runtime;
namespace fixture = mavmoos::test::debug_fixture;

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
    require(key == "MAVMOOS_DEBUG" || key == "MAVMOOS_DEBUG_VECT" ||
            key == "MAVMOOS_DEBUG_FLOAT_ARRAY" ||
            key == "MAVMOOS_NAMED_VALUE_FLOAT" || key == "MAVMOOS_NAMED_VALUE_INT",
            "unexpected debug publication");
    require(posts.emplace(key, value).second, "duplicate debug publication");
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

  void receive(const mavlink_message_t& message, double effective, double read)
  {
    host.time = read;
    plugin->onMessageWithMetadata(message, {effective, read,
      effective == read ? "READ_ACQUISITION" : "QUEUE_EMPTY_FLOOR"});
  }
  void receive(const mavlink_message_t& message, double at) { receive(message, at, at); }

  void heartbeat(double at, uint8_t autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA, uint8_t component = 1)
  {
    host.state.heartbeat_valid = true;
    host.state.heartbeat_at = at;
    host.state.type = MAV_TYPE_QUADROTOR;
    host.state.autopilot = autopilot;
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack_status(1, component, &codec, &message, host.state.type, autopilot,
                                     0, 0, MAV_STATE_STANDBY);
    receive(message, at);
  }

  void batch(double effective, double read)
  {
    for (const auto& message : fixture::frames(codec)) receive(message, effective, read);
  }

  void capture(const char* phase, double at, bool publishes = true)
  {
    host.time = at;
    host.posts.clear();
    plugin->iterate();
    require(host.posts.size() == (publishes ? 5u : 0u), "unexpected publication cadence");
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
  rig.batch(100.05, 100.06);
  rig.capture("prebind", 100.12);
  rig.heartbeat(100.20);
  rig.batch(100.22, 100.26);
  rig.capture("fresh", 100.32);
  rig.capture("cadence", 100.34, false);

  rig.receive(fixture::encode(rig.codec, fixture::debug(7, 20), 2, 1), 100.45);
  rig.receive(fixture::encode(rig.codec, fixture::namedFloat("shared", 20), 1, 2), 100.46);
  rig.receive(fixture::encode(rig.codec, fixture::debug(7, 20)), 100.21, 100.47);
  auto unrelated = fixture::encode(rig.codec, fixture::vector());
  unrelated.msgid = MAVLINK_MSG_ID_STATUSTEXT;
  rig.receive(unrelated, 100.48);
  rig.capture("rejected_source_and_order", 100.50);

  auto bad = fixture::encode(rig.codec, fixture::debug(7, 20));
  bad.len = 0;
  rig.receive(bad, 100.53);
  bad.len = MAVLINK_MSG_ID_DEBUG_LEN + 1;
  rig.receive(bad, 100.54);
  bad = fixture::encode(rig.codec, fixture::vector());
  bad.incompat_flags = 0x80;
  rig.receive(bad, 100.55);
  rig.codec.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  bad = fixture::encode(rig.codec, fixture::namedInt());
  rig.codec.flags = 0;
  --bad.len;
  rig.receive(bad, 100.56);
  bad = fixture::encode(rig.codec, fixture::floatArray());
  bad.magic = MAVLINK_STX_MAVLINK1;
  bad.len = MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_MIN_LEN;
  rig.receive(bad, 100.57);
  rig.capture("malformed", 100.64);

  rig.heartbeat(101.22);
  auto new_value = fixture::debug(0, std::numeric_limits<float>::quiet_NaN());
  new_value.time_boot_ms = 0;
  rig.receive(fixture::encode(rig.codec, new_value), 101.25);
  auto renamed = fixture::floatArray(17, "renamed");
  renamed.time_usec = 1;
  std::fill(std::begin(renamed.data), std::end(renamed.data), 0);
  rig.receive(fixture::encode(rig.codec, renamed), 101.26);
  rig.capture("independent_ages", 101.40);
  rig.capture("heartbeat_loss", 103.30);
  rig.batch(103.35, 103.37);
  rig.capture("unbound_after_loss", 103.43);

  rig.heartbeat(103.50, MAV_AUTOPILOT_PX4);
  rig.batch(103.53, 103.55);
  rig.capture("px4_rebind", 103.60);
  rig.capture("clock_fault", 103.56);
  rig.host.time = 104;
  rig.plugin->onReset("test_reset");
  rig.receive(fixture::encode(rig.codec, fixture::debug()), 104, 104.03);
  rig.capture("reset_floor", 104.05);
  rig.heartbeat(104.11, MAV_AUTOPILOT_GENERIC);
  rig.batch(104.12, 104.13);
  rig.capture("unsupported_autopilot", 104.18);
  rig.heartbeat(104.21, MAV_AUTOPILOT_ARDUPILOTMEGA, 158);
  rig.batch(104.22, 104.25);
  rig.capture("wrong_heartbeat_component", 104.30);
}

void trimmingAndIdentity(Rig& rig)
{
  rig.heartbeat(100.1);
  rig.codec.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  for (const auto& message : fixture::baseFrames(rig.codec)) rig.receive(message, 100.2, 100.21);
  rig.codec.flags = 0;
  rig.capture("v1", 100.25);
  auto changed = fixture::namedFloat("shared", 1.5f);
  changed.name[7] = 'x';
  changed.name[8] = 'y';
  changed.time_boot_ms = 0;
  rig.receive(fixture::encode(rig.codec, changed), 100.32);
  rig.capture("name_padding_and_boot_wrap", 100.40);
}

void capacity(Rig& rig)
{
  rig.heartbeat(100.1);
  for (unsigned index = 1; index <= 9; ++index) {
    const auto name = "key" + std::to_string(index);
    const auto at = 100.2 + index * .01;
    rig.receive(fixture::encode(rig.codec, fixture::debug(index)), at);
    rig.receive(fixture::encode(rig.codec, fixture::vector(name)), at);
    rig.receive(fixture::encode(rig.codec, fixture::floatArray(index, name)), at);
    rig.receive(fixture::encode(rig.codec, fixture::namedFloat(name)), at);
    rig.receive(fixture::encode(rig.codec, fixture::namedInt(name)), at);
  }
  rig.receive(fixture::encode(rig.codec, fixture::debug(8, 16)), 100.30);
  rig.receive(fixture::encode(rig.codec, fixture::namedFloat("key8", 16)), 100.31);
  rig.capture("capacity", 100.4);

  rig.host.time = 100.5;
  rig.plugin->onReset("link_loss");
  rig.heartbeat(100.6);
  rig.receive(fixture::encode(rig.codec, fixture::debug(9)), 100.65);
  rig.receive(fixture::encode(rig.codec, fixture::namedInt("key9")), 100.66);
  rig.receive(fixture::encode(rig.codec, fixture::floatArray(9)), 100.67);
  rig.capture("capacity_reset", 100.70);
}

void verifyDescriptor(CreatePlugin create, Rig& rig)
{
  require(!create(plugin_abi + 1, &rig.host) && !create(plugin_abi, nullptr), "unsafe factory accepted");
  const auto descriptor = rig.plugin->descriptor();
  require(rig.plugin->name() == "debug_values" && descriptor.profile == "mavlink_debug_value_observations" &&
          descriptor.exclusive_resources == std::vector<std::string>{"observation.debug_values"} &&
          descriptor.capabilities.size() == 5 && descriptor.streamed_parameter_bench_compatible,
          "wrong provider descriptor");
  for (const auto& capability : descriptor.capabilities)
    require(capability.access == "observation" && capability.request_variable.empty(), "effectful debug capability");
  require(rig.plugin->subscriptions().empty() && !rig.plugin->controlBackend() && descriptor.request_owners.empty() &&
          descriptor.observation_streams.empty(), "debug provider attempted effect ownership or rate negotiation");
  require(rig.plugin->configureModule({{"unknown", "1"}}).status == ModuleConfigStatus::UnknownKey &&
          rig.plugin->configureModule({{"sample_timeout", "NaN"}}).status == ModuleConfigStatus::Invalid &&
          rig.plugin->configureModule({{"sample_timeout", ".5"}, {"publish_interval", ".6"}}).status ==
            ModuleConfigStatus::Invalid &&
          rig.plugin->configureModule({{"sample_timeout", "1"}, {"publish_interval", ".1"}}).status ==
            ModuleConfigStatus::Accepted,
          "debug module configuration bounds differ");
}

void printFrames()
{
  mavlink_status_t codec{};
  for (const auto& message : fixture::frames(codec)) {
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};
    const auto length = mavlink_msg_to_send_buffer(bytes, &message);
    for (unsigned index = 0; index < length; ++index)
      std::cout << std::hex << std::setfill('0') << std::setw(2) << unsigned(bytes[index]);
    std::cout << '\n';
  }
}

} // namespace

int main(int argc, char** argv)
{
  try {
    require(argc == 2, "usage: debug_values_scenario_driver MODULE|--frames");
    if (std::string_view(argv[1]) == "--frames") {
      printFrames();
      return 0;
    }
    void* library = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
    if (!library) throw std::runtime_error(dlerror());
    const auto create = reinterpret_cast<CreatePlugin>(dlsym(library, "mavmoos_create_plugin"));
    const auto destroy = reinterpret_cast<DestroyPlugin>(dlsym(library, "mavmoos_destroy_plugin"));
    require(create && destroy, "missing debug provider ABI exports");
    {
      Rig rig(create, destroy);
      verifyDescriptor(create, rig);
      lifecycle(rig);
    }
    {
      Rig rig(create, destroy);
      trimmingAndIdentity(rig);
    }
    {
      Rig rig(create, destroy);
      capacity(rig);
    }
    dlclose(library);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
