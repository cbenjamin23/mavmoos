#include "Plugin.h"
#include "TrafficFrames.h"

#include <dlfcn.h>
#include <iostream>
#include <memory>
#include <stdexcept>

using namespace mavmoos::runtime;
using namespace mavmoos::test::traffic;

namespace {
void require(bool value, const char* reason)
{
  if (!value)
    throw std::runtime_error(reason);
}

struct Host final : Services {
  double clock = 100;
  Vehicle state;
  std::string report;

  double now() const override { return clock; }
  const Vehicle& vehicle() const override { return state; }
  void publish(const std::string& key, const std::string& value) override
  {
    require(key == "MAVMOOS_TRAFFIC_CONTACTS", "unexpected traffic publication");
    report = value;
  }
  void publish(const std::string&, double) override { unexpected(); }
  bool sendCommand(const CommandRequest&) override { unexpected(); }
  bool acquireNavigation(const std::string&) override { unexpected(); }
  void revokeNavigation(const std::string&) override { unexpected(); }
  bool navigation(const NavigationTarget&) override { unexpected(); }

  [[noreturn]] static void unexpected() { throw std::runtime_error("traffic tried to use an effectful service"); }
};

struct Rig {
  Host host;
  std::unique_ptr<Plugin, DestroyPlugin> plugin;
  mavlink_status_t codec{};

  Rig(CreatePlugin create, DestroyPlugin destroy) : plugin(create(plugin_abi, &host), destroy)
  {
    require(bool(plugin), "factory rejected current ABI");
  }

  void receive(const mavlink_message_t& message, double effective_at, double read_at)
  {
    host.clock = read_at;
    plugin->onMessageWithMetadata(message, {effective_at, read_at,
      effective_at == read_at ? "READ_ACQUISITION" : "QUEUE_EMPTY_FLOOR"});
  }

  void receive(const mavlink_message_t& message, double at) { receive(message, at, at); }

  void heartbeat(double at, uint8_t autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA)
  {
    host.state.heartbeat_valid = true;
    host.state.heartbeat_at = at;
    host.state.type = MAV_TYPE_QUADROTOR;
    host.state.autopilot = autopilot;
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack_status(1, 1, &codec, &message, MAV_TYPE_QUADROTOR,
                                     autopilot, 0, 0, MAV_STATE_STANDBY);
    receive(message, at);
  }

  void capture(const char* phase, double at)
  {
    host.clock = at;
    host.report.clear();
    plugin->iterate();
    require(!host.report.empty(), "expected traffic snapshot");
    std::cout << "{\"phase\":\"" << phase << "\",\"report\":" << host.report << "}\n";
  }
};

void exercise(Rig& rig)
{
  const auto descriptor = rig.plugin->descriptor();
  require(rig.plugin->name() == "traffic" && descriptor.capabilities.size() == 1 &&
          descriptor.capabilities.front().name == "observations.adsb_traffic" &&
          descriptor.request_owners.empty() && descriptor.observation_streams.empty() &&
          rig.plugin->subscriptions().empty() && !rig.plugin->controlBackend(),
          "traffic descriptor or ownership differs");
  require(rig.plugin->configureModule({{"sample_timeout", "0.2"}}).status == ModuleConfigStatus::Invalid &&
          rig.plugin->configureModule({{"sample_timeout", "0.8"}, {"publish_interval", "0.9"}}).status ==
            ModuleConfigStatus::Invalid &&
          rig.plugin->configureModule({{"not_a_setting", "1"}}).status == ModuleConfigStatus::UnknownKey &&
          rig.plugin->configureModule({{"sample_timeout", "0.8"}}).status == ModuleConfigStatus::Accepted,
          "traffic timeout settings differ");

  const auto a = encode(rig.codec, complete());
  rig.capture("startup", 100);
  rig.receive(a, 100.01);
  rig.heartbeat(100.04, MAV_AUTOPILOT_PX4);
  rig.receive(a, 100.05);
  rig.capture("unbound", 100.11);
  rig.heartbeat(100.12);
  rig.receive(encode(rig.codec, complete(foreign_icao), 2), 100.13);
  rig.receive(encode(rig.codec, complete(foreign_icao), 1, MAV_COMP_ID_ADSB), 100.14);
  auto bad_wire = a;
  bad_wire.len = MAVLINK_MSG_ID_ADSB_VEHICLE_LEN + 1;
  rig.receive(bad_wire, 100.15);
  rig.receive(encode(rig.codec, complete(invalid_icao)), 100.16);
  rig.receive(encode(rig.codec, {}), 100.165);

  rig.codec.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  const auto legacy = encode(rig.codec, complete());
  require(legacy.magic == MAVLINK_STX_MAVLINK1, "expected MAVLink 1 fixture");
  rig.codec.flags = 0;
  rig.receive(legacy, 100.17, 100.18);
  rig.receive(encode(rig.codec, partial()), 100.19);
  mavlink_adsb_vehicle_t identity_only{};
  identity_only.ICAO_address = 42;
  const auto trimmed = encode(rig.codec, identity_only);
  require(trimmed.len == 1, "expected trimmed MAVLink 2 fixture");
  rig.receive(trimmed, 100.20);
  rig.capture("three_contacts", 100.22);

  rig.receive(a, 100.26);
  auto rollback = complete();
  rollback.heading = 100;
  rig.receive(encode(rig.codec, rollback), 100.16, 100.29);
  rig.capture("duplicate_and_out_of_order", 100.34);
  rig.receive(encode(rig.codec, complete(43)), 100.20, 101.01);
  rig.capture("two_expired", 101.02);
  for (uint32_t address = 1000; address <= 1031; ++address)
    rig.receive(encode(rig.codec, complete(address)), 101.03);
  auto renewed = complete();
  renewed.heading = 9000;
  rig.receive(encode(rig.codec, renewed), 101.04);
  rig.capture("at_capacity", 101.17);

  rig.heartbeat(101.40);
  rig.receive(encode(rig.codec, renewed), 101.44);
  rig.capture("one_left", 102.03);
  rig.receive(encode(rig.codec, complete(1031)), 102.07);
  rig.capture("capacity_available", 102.16);

  rig.host.clock = 102.18;
  rig.host.state.heartbeat_valid = false;
  rig.plugin->onInputBoundary();
  rig.receive(a, 102.17, 102.19);
  rig.receive(a, 102.20);
  rig.capture("heartbeat_lost", 102.22);
  rig.heartbeat(102.30);
  rig.receive(a, 102.31);
  rig.capture("reconnected", 102.34);
  rig.capture("clock_regression", 102.20);
}
}

int main(int argc, char** argv)
{
  try {
    require(argc == 2, "usage: traffic_scenario_driver MODULE");
    void* module = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
    require(module, "traffic module could not be opened");
    const auto create = reinterpret_cast<CreatePlugin>(dlsym(module, "mavmoos_create_plugin"));
    const auto destroy = reinterpret_cast<DestroyPlugin>(dlsym(module, "mavmoos_destroy_plugin"));
    require(create && destroy, "traffic module entry points missing");
    {
      Rig rig(create, destroy);
      require(!create(plugin_abi + 1, &rig.host) && !create(plugin_abi, nullptr), "invalid traffic factory arguments accepted");
      exercise(rig);
    }
    dlclose(module);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
