#include "AirDataFrames.h"
#include "Plugin.h"
#include <dlfcn.h>
#include <iostream>
#include <stdexcept>

using namespace mavmoos::runtime;
namespace frames = mavmoos::test::air_data;

namespace {

void require(bool condition, const char* reason)
{
  if(!condition) throw std::runtime_error(reason);
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
  void publish(const std::string& key, const std::string& value) override
  {
    posts.emplace_back(key, value);
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

  [[noreturn]] static void effect()
  {
    throw std::runtime_error("air_data attempted an effectful service or scalar publication");
  }
};

struct Rig {
  Host host;
  Plugin* plugin;
  DestroyPlugin destroy;
  mavlink_status_t codec{};

  Rig(CreatePlugin create, DestroyPlugin destroy)
    : plugin(create(plugin_abi, &host)), destroy(destroy)
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
  void receive(const mavlink_message_t& message, double time) { receive(message, time, time); }

  void heartbeat(double time)
  {
    host.state.heartbeat_valid = true;
    host.state.heartbeat_at = time;
    mavlink_heartbeat_t value{};
    value.type = host.state.type;
    value.autopilot = host.state.autopilot;
    mavlink_message_t message{};
    mavlink_msg_heartbeat_encode_status(1, 1, &codec, &message, &value);
    receive(message, time);
  }

  void capture(const char* name, double time)
  {
    host.time = time;
    host.posts.clear();
    plugin->iterate();
    std::cout << "{\"phase\":\"" << name << "\",\"posts\":{";
    bool first = true;
    for(const auto& [key, value] : host.posts) {
      if(!first) std::cout << ',';
      first = false;
      std::cout << '"' << key << "\":" << value;
    }
    std::cout << "}}\n";
  }
};

void exerciseObservations(Rig& rig)
{
  rig.capture("absent", 100);
  rig.receive(frames::vfrHud(rig.codec), 100.1);
  rig.capture("no_heartbeat", 100.15);
  rig.heartbeat(100.25);
  rig.receive(frames::vfrHud(rig.codec), 100.4, 100.5);
  rig.receive(frames::wind(rig.codec), 100.8);
  rig.receive(frames::windCov(rig.codec), 101);
  rig.capture("fresh", 101.1);
  rig.receive(frames::vfrHud(rig.codec, 1, 2), 101.25);
  rig.capture("wrong_source", 101.26);
  rig.receive(frames::vfrHud(rig.codec), 100.35, 101.3);
  rig.capture("independent", 101.6);
  rig.capture("heartbeat_expired", 102.26);
  rig.heartbeat(102.4);
  rig.receive(frames::wind(rig.codec), 102.5);
  rig.capture("reacquired", 102.65);
  rig.host.time = 102.75;
  rig.plugin->onReset("test_reset");
  rig.receive(frames::wind(rig.codec), 102.7, 102.8);
  rig.capture("reset", 102.9);
}

void exerciseMalformed(Rig& rig)
{
  rig.heartbeat(100.1);
  rig.receive(frames::vfrHud(rig.codec), 100.2);
  rig.receive(frames::wind(rig.codec), 100.25);
  rig.receive(frames::windCov(rig.codec), 100.3);
  rig.capture("before_invalid", 100.31);

  auto message = frames::vfrHud(rig.codec);
  message.len = 0;
  rig.receive(message, 100.4);
  message = frames::vfrHud(rig.codec);
  message.len = MAVLINK_MSG_ID_VFR_HUD_LEN + 1;
  rig.receive(message, 100.41);
  message = frames::vfrHud(rig.codec);
  message.incompat_flags = 128;
  rig.receive(message, 100.42);

  mavlink_vfr_hud_t hud{};
  hud.airspeed = std::numeric_limits<float>::quiet_NaN();
  mavlink_msg_vfr_hud_encode_status(1, 1, &rig.codec, &message, &hud);
  rig.receive(message, 100.5);
  hud.airspeed = 0;
  hud.heading = 361;
  mavlink_msg_vfr_hud_encode_status(1, 1, &rig.codec, &message, &hud);
  rig.receive(message, 100.51);
  hud.heading = 1;
  hud.throttle = 101;
  mavlink_msg_vfr_hud_encode_status(1, 1, &rig.codec, &message, &hud);
  rig.receive(message, 100.52);

  mavlink_wind_t wind{};
  wind.speed = -1;
  mavlink_msg_wind_encode_status(1, 1, &rig.codec, &message, &wind);
  rig.receive(message, 100.6);
  wind.speed = 0;
  wind.direction = -181;
  mavlink_msg_wind_encode_status(1, 1, &rig.codec, &message, &wind);
  rig.receive(message, 100.61);

  mavlink_wind_cov_t cov{};
  cov.var_vert = -1;
  mavlink_msg_wind_cov_encode_status(1, 1, &rig.codec, &message, &cov);
  rig.receive(message, 100.7);
  cov.var_vert = 0;
  cov.wind_y = std::numeric_limits<float>::infinity();
  mavlink_msg_wind_cov_encode_status(1, 1, &rig.codec, &message, &cov);
  rig.receive(message, 100.71);
  cov.wind_y = 0;
  cov.horiz_accuracy = std::numeric_limits<float>::quiet_NaN();
  mavlink_msg_wind_cov_encode_status(1, 1, &rig.codec, &message, &cov);
  rig.receive(message, 100.72);
  rig.capture("invalid_preserves", 100.8);
}

void exerciseCodec(Rig& rig)
{
  rig.heartbeat(100.1);
  rig.codec.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  auto message = frames::vfrHud(rig.codec);
  require(message.magic == MAVLINK_STX_MAVLINK1, "fixture did not encode MAVLink 1");
  rig.receive(message, 100.2);
  auto bad_v1 = frames::wind(rig.codec);
  --bad_v1.len;
  rig.receive(bad_v1, 100.21);
  rig.codec.flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  mavlink_wind_cov_t zero_tail{};
  zero_tail.time_usec = 1;
  mavlink_msg_wind_cov_encode_status(1, 1, &rig.codec, &message, &zero_tail);
  require(message.magic == MAVLINK_STX && message.len == 1, "fixture did not trim MAVLink 2");
  rig.receive(message, 100.3);
  rig.capture("codec", 100.4);
}

void exercisePressure(Rig& rig)
{
  rig.heartbeat(100.1);
  const auto first = frames::pressure(rig.codec);
  rig.receive(first, 100.2, 100.25);

  mavlink_scaled_pressure2_t second{};
  second.time_boot_ms = std::numeric_limits<uint32_t>::max();
  second.press_diff = -4.5f;
  second.temperature = 2134;
  second.temperature_press_diff = 1;
  mavlink_message_t second_message{};
  mavlink_msg_scaled_pressure2_encode_status(1, 1, &rig.codec, &second_message, &second);
  require(second_message.magic == MAVLINK_STX && second_message.len == MAVLINK_MSG_ID_SCALED_PRESSURE2_LEN - 1,
          "fixture did not trim the extension's zero high byte");
  rig.receive(second_message, 100.5);

  mavlink_scaled_pressure3_t third{};
  third.time_boot_ms = 123;
  third.press_abs = 912.5f;
  third.temperature_press_diff = -125;
  mavlink_message_t third_message{};
  rig.codec.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  mavlink_msg_scaled_pressure3_encode_status(1, 1, &rig.codec, &third_message, &third);
  require(third_message.magic == MAVLINK_STX_MAVLINK1 &&
          third_message.len == MAVLINK_MSG_ID_SCALED_PRESSURE3_MIN_LEN, "fixture did not encode pressure v1");
  rig.codec.flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  rig.receive(third_message, 100.8);
  rig.capture("pressure_fresh", 100.9);

  auto invalid = first;
  invalid.len = MAVLINK_MSG_ID_SCALED_PRESSURE_LEN + 1;
  rig.receive(invalid, 101);
  mavlink_scaled_pressure_t malformed{};
  malformed.press_abs = -1;
  mavlink_msg_scaled_pressure_encode_status(1, 1, &rig.codec, &invalid, &malformed);
  rig.receive(invalid, 101.01);
  malformed.press_abs = std::numeric_limits<float>::quiet_NaN();
  mavlink_msg_scaled_pressure_encode_status(1, 1, &rig.codec, &invalid, &malformed);
  rig.receive(invalid, 101.02);
  malformed.press_abs = 1000;
  malformed.press_diff = std::numeric_limits<float>::infinity();
  mavlink_msg_scaled_pressure_encode_status(1, 1, &rig.codec, &invalid, &malformed);
  rig.receive(invalid, 101.03);
  --third_message.len;
  rig.receive(third_message, 101.04);
  rig.receive(frames::pressure(rig.codec, 1, 2), 101.05);
  rig.receive(second_message, 100.4, 101.06);
  rig.capture("pressure_independent", 101.35);

  third = {};
  third.time_boot_ms = 1;
  mavlink_msg_scaled_pressure3_encode_status(1, 1, &rig.codec, &third_message, &third);
  require(third_message.magic == MAVLINK_STX && third_message.len == 1,
          "fixture did not trim all-zero pressure fields");
  rig.receive(third_message, 101.4);
  rig.capture("pressure_zero_tail", 101.55);
  rig.capture("pressure_heartbeat_expired", 102.2);
}

void exerciseConfiguration(Rig& rig)
{
  const auto accepted = ModuleConfigStatus::Accepted;
  const auto invalid = ModuleConfigStatus::Invalid;
  require(rig.plugin->configureModule({{"sample_timeout", ".25"}, {"publish_interval", ".2"}}).status ==
          accepted, "valid settings rejected");
  require(rig.plugin->configureModule({{"wrong_key", "1"}}).status == ModuleConfigStatus::UnknownKey,
          "unknown module key accepted");
  require(rig.plugin->configureModule({{"sample_timeout", "1"}, {"sample_timeout", "2"}}).status ==
          invalid, "duplicate module key accepted");
  require(rig.plugin->configureModule({{"sample_timeout", ".25"}, {"publish_interval", ".3"}}).status ==
          invalid, "publication interval longer than timeout accepted");
  for(const auto* value : {"nan", "inf", "1 ", "0", "10.1", "1x"}) {
    require(rig.plugin->configureModule({{"sample_timeout", value}}).status == invalid,
            "invalid sample timeout accepted");
  }
  rig.heartbeat(100.1);
  rig.receive(frames::vfrHud(rig.codec), 100.2);
  rig.capture("configuration", 100.21);
  rig.capture("interval_suppressed", 100.3);
  rig.capture("configured_stale", 100.46);
}

void exerciseFixedWing(Rig& rig)
{
  rig.host.state.type = MAV_TYPE_FIXED_WING;
  rig.receive(frames::airspeed(rig.codec), 100.05);
  rig.receive(frames::highLatency(rig.codec), 100.06);
  rig.capture("fixed_wing_before_bind", 100.08);
  rig.heartbeat(100.1);
  rig.receive(frames::airspeed(rig.codec), 100.2, 100.25);
  const auto second_sensor = frames::airspeed(rig.codec, 1);
  require(second_sensor.magic == MAVLINK_STX && second_sensor.len == MAVLINK_MSG_ID_AIRSPEED_LEN - 1,
          "expected MAVLink 2 AIRSPEED with trimmed flags");
  rig.receive(second_sensor, 100.3);
  const auto high_latency = frames::highLatency(rig.codec);
  require(high_latency.magic == MAVLINK_STX && high_latency.len < MAVLINK_MSG_ID_HIGH_LATENCY2_LEN,
          "expected MAVLink 2 HIGH_LATENCY2 with trimmed tail");
  rig.receive(high_latency, 100.35);
  rig.receive(frames::vfrHud(rig.codec), 100.37);
  rig.capture("fixed_wing_fresh", 100.5);

  rig.receive(frames::airspeed(rig.codec, 0, 2, 1), 100.6);
  rig.receive(frames::airspeed(rig.codec, 1, 1, 2), 100.61);
  rig.receive(frames::highLatency(rig.codec, 2, 1), 100.62);
  rig.receive(frames::highLatency(rig.codec, 1, 2), 100.63);
  rig.receive(frames::highLatency(rig.codec, 1, 1, MAV_TYPE_QUADROTOR), 100.64);
  rig.receive(frames::highLatency(rig.codec, 1, 1, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_PX4), 100.65);
  rig.receive(frames::airspeed(rig.codec, 2), 100.66);
  rig.receive(frames::airspeed(rig.codec, 255), 100.67);

  mavlink_airspeed_t sensor{};
  mavlink_message_t message{};
  sensor.airspeed = -1;
  mavlink_msg_airspeed_encode_status(1, 1, &rig.codec, &message, &sensor);
  rig.receive(message, 100.68);
  sensor.airspeed = std::numeric_limits<float>::quiet_NaN();
  mavlink_msg_airspeed_encode_status(1, 1, &rig.codec, &message, &sensor);
  rig.receive(message, 100.69);
  sensor.airspeed = 0;
  sensor.raw_press = std::numeric_limits<float>::infinity();
  mavlink_msg_airspeed_encode_status(1, 1, &rig.codec, &message, &sensor);
  rig.receive(message, 100.7);
  message = frames::airspeed(rig.codec);
  message.len = 0;
  rig.receive(message, 100.71);
  message.len = MAVLINK_MSG_ID_AIRSPEED_LEN;
  message.magic = MAVLINK_STX_MAVLINK1;
  rig.receive(message, 100.72);
  message.magic = MAVLINK_STX;
  message.len = MAVLINK_MSG_ID_AIRSPEED_LEN + 1;
  rig.receive(message, 100.73);
  message.len = MAVLINK_MSG_ID_AIRSPEED_LEN;
  message.incompat_flags = 128;
  rig.receive(message, 100.74);

  rig.receive(frames::airspeed(rig.codec), 100.15, 100.75);
  rig.receive(frames::highLatency(rig.codec), 100.34, 100.76);
  rig.capture("fixed_wing_rejected", 100.8);
  rig.receive(frames::airspeed(rig.codec, 1), 101.15);
  rig.capture("fixed_wing_independent", 101.6);
  rig.capture("fixed_wing_lost", 102.15);
  rig.receive(frames::airspeed(rig.codec), 102.2);
  rig.receive(frames::highLatency(rig.codec), 102.21);
  rig.capture("fixed_wing_payload_without_heartbeat", 102.27);

  rig.heartbeat(102.4);
  sensor = {};
  mavlink_msg_airspeed_encode_status(1, 1, &rig.codec, &message, &sensor);
  require(message.magic == MAVLINK_STX && message.len == 1, "expected one-byte zero AIRSPEED fixture");
  rig.receive(message, 102.45);
  mavlink_high_latency2_t edge{};
  edge.type = MAV_TYPE_FIXED_WING;
  edge.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
  edge.airspeed = std::numeric_limits<uint8_t>::max();
  rig.codec.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  mavlink_msg_high_latency2_encode_status(1, 1, &rig.codec, &message, &edge);
  require(message.magic == MAVLINK_STX_MAVLINK1 && message.len == MAVLINK_MSG_ID_HIGH_LATENCY2_MIN_LEN,
          "expected MAVLink 1 HIGH_LATENCY2 fixture");
  rig.codec.flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  rig.receive(message, 102.46);
  rig.capture("fixed_wing_rebound", 102.53);
  rig.host.time = 102.7;
  rig.plugin->onReset("fixed_wing_reset");
  rig.receive(frames::airspeed(rig.codec, 1), 102.65, 102.8);
  rig.receive(frames::highLatency(rig.codec), 102.66, 102.81);
  rig.capture("fixed_wing_reset", 102.9);
}

void exerciseWrongVehicleType(Rig& rig)
{
  rig.heartbeat(100.1);
  rig.receive(frames::airspeed(rig.codec), 100.2);
  rig.receive(frames::highLatency(rig.codec), 100.21);
  rig.receive(frames::vfrHud(rig.codec), 100.22);
  rig.capture("wrong_vehicle_type", 100.35);
}

} // namespace

int main(int argc, char** argv)
{
  try {
    require(argc == 2, "usage: air_data_scenario_driver MODULE");
    void* library = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
    if(!library) throw std::runtime_error(dlerror());
    const auto create = reinterpret_cast<CreatePlugin>(dlsym(library, "mavmoos_create_plugin"));
    const auto destroy = reinterpret_cast<DestroyPlugin>(dlsym(library, "mavmoos_destroy_plugin"));
    require(create && destroy, "missing factory symbols");
    Host host;
    require(!create(plugin_abi - 1, &host) && !create(plugin_abi, nullptr),
            "factory accepted incompatible ABI or null Services");
    {
      Rig rig(create, destroy);
      const auto descriptor = rig.plugin->descriptor();
      require(rig.plugin->name() == "air_data" && rig.plugin->subscriptions().empty() &&
              !rig.plugin->controlBackend() && descriptor.request_owners.empty() &&
              descriptor.observation_streams.empty(), "unexpected effect surface");
      require(descriptor.profile == "ardupilot_air_data_observations" &&
              descriptor.exclusive_resources == std::vector<std::string>{"observation.air_data"} &&
              descriptor.capabilities.size() == 6 && descriptor.streamed_parameter_bench_compatible,
              "unexpected capability descriptor");
      for(const auto& capability : descriptor.capabilities) {
        require(capability.access == "observation" && capability.request_variable.empty() &&
                capability.request_value.empty(), "effectful descriptor");
      }
      exerciseObservations(rig);
    }
    {
      Rig rig(create, destroy);
      exerciseMalformed(rig);
    }
    {
      Rig rig(create, destroy);
      exerciseCodec(rig);
    }
    {
      Rig rig(create, destroy);
      exercisePressure(rig);
    }
    {
      Rig rig(create, destroy);
      exerciseConfiguration(rig);
    }
    {
      Rig rig(create, destroy);
      exerciseFixedWing(rig);
    }
    {
      Rig rig(create, destroy);
      exerciseWrongVehicleType(rig);
    }
    {
      Rig rig(create, destroy);
      rig.host.state.type = MAV_TYPE_FIXED_WING;
      rig.host.state.autopilot = MAV_AUTOPILOT_PX4;
      rig.heartbeat(100.1);
      rig.receive(frames::vfrHud(rig.codec), 100.2);
      rig.receive(frames::airspeed(rig.codec), 100.21);
      rig.receive(frames::highLatency(rig.codec), 100.22);
      rig.capture("unsupported_autopilot", 100.3);
    }
    require(dlclose(library) == 0, "could not close test module");
  } catch(const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
