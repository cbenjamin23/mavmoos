// Actual loader and module, fake Services/clock only. No MOOS, socket or simulator.
#include "PluginHost.h"
#include <dlfcn.h>
#include <iostream>
#include <functional>
#include <sstream>
#include <vector>
#include <stdexcept>

using namespace mavmoos::runtime;
void require(bool value, const std::string& why) { if (!value) throw std::runtime_error(why); }
class FakeServices final : public Services {
public:
  double clock = 100;
  unsigned effects = 0, publications = 0;
  mutable unsigned service_calls = 0;
  bool configuring = false;
  std::string last;
  Vehicle state;
  void touched(const char* method) const {
    ++service_calls;
    require(!configuring, std::string("configuration attempted Services::") + method);
  }
  double now() const override { touched("now"); return clock; }
  const Vehicle& vehicle() const override { touched("vehicle"); return state; }
  void publish(const std::string& key, const std::string& value) override {
    touched("publish_text");
    require(key == "EXAMPLE_HEARTBEAT_OBSERVATION", "unexpected publication"); last = value; ++publications;
  }
  void publish(const std::string&, double) override { touched("publish_numeric"); throw std::runtime_error("unexpected numeric publication"); }
  bool effect(const char* method) {
    touched(method); ++effects;
    throw std::runtime_error(std::string("read-only example attempted effect: ") + method);
  }
  bool sendCommand(const CommandRequest&) override { return effect("sendCommand"); }
  bool acquireNavigation(const std::string&) override { return effect("acquireNavigation"); }
  void revokeNavigation(const std::string&) override { effect("revokeNavigation"); }
  bool navigation(const NavigationTarget&) override { return effect("navigation"); }
  bool parameter(const ParameterRequest&) override { return effect("parameter"); }
  bool mission(const MissionRequest&) override { return effect("mission"); }
  void missionFinished(const std::string&) override { effect("missionFinished"); }
  bool recordStoredMissionProof(const std::string&, const std::vector<MissionItem>&) override { return effect("recordStoredMissionProof"); }
  void invalidateStoredMissionProof(const std::string&) override { effect("invalidateStoredMissionProof"); }
  bool beginMissionExecution(const std::string&, uint64_t, const std::string&, double) override { return effect("beginMissionExecution"); }
  bool beginMissionExecutionAbort(const std::string&, const std::string&, double) override { return effect("beginMissionExecutionAbort"); }
  bool finishMissionExecution(const std::string&, const std::string&) override { return effect("finishMissionExecution"); }
  bool suspendNavigation(double) override { return effect("suspendNavigation"); }
  bool resumeNavigation(const std::string&) override { return effect("resumeNavigation"); }
  void commandFinished(const std::string&) override { effect("commandFinished"); }
  void cancelCommand(const std::string&) override { effect("cancelCommand"); }
  bool flightLog(const FlightLogRequest&) override { return effect("flightLog"); }
  void flightLogFinished(const std::string&) override { effect("flightLogFinished"); }
  StoredMissionProof storedMissionProof() const override { touched("storedMissionProof"); return {}; }
  bool missionExecutionGranted(const std::string&) const override { touched("missionExecutionGranted"); return false; }
  bool navigationGranted() const override { touched("navigationGranted"); return false; }
  bool controlBusy() const override { touched("controlBusy"); return false; }
  bool projectGlobal(double, double, double&, double&) const override { touched("projectGlobal"); return false; }
};

std::string jsonText(const std::string& value) {
  constexpr char hex[] = "0123456789abcdef";
  std::string result = "\"";
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') { result += '\\'; result += static_cast<char>(c); }
    else if (c < 32) { result += "\\u00"; result += hex[c >> 4]; result += hex[c & 15]; }
    else result += static_cast<char>(c);
  }
  return result + '"';
}

void emit(PluginHost& host, FakeServices& services, const std::string& phase, double now) {
  services.clock = now; host.inputBoundary(); host.iterate();
  require(!services.last.empty(), "no observation");
  std::cout << "{\"phase\":" << jsonText(phase) << ",\"effects\":" << services.effects
            << ",\"publications\":" << services.publications << ",\"observation\":" << services.last << "}\n";
}

ModuleConfigResult configure(PluginHost& host, FakeServices& services,
                             const std::vector<ModuleSetting>& settings) {
  const auto before = services.service_calls;
  services.configuring = true;
  ModuleConfigResult result;
  try { result = host.plugins().front()->configureModule(settings); }
  catch (...) { services.configuring = false; throw; }
  services.configuring = false;
  require(services.service_calls == before, "configuration called Services");
  require(!services.effects && !services.publications, "configuration produced output/effects");
  return result;
}

void configureStartup(PluginHost& host, FakeServices& services,
                      const std::vector<ModuleSetting>& settings) {
  const auto before = services.service_calls;
  services.configuring = true;
  const bool accepted = host.configureModules({{"example_heartbeat_observer", settings}});
  services.configuring = false;
  require(accepted, host.lastError());
  require(services.service_calls == before && !services.effects && !services.publications,
          "checked host startup used Services");
}

void checkedHostFailure(const char* path) {
  FakeServices services; PluginHost host; require(host.load(path, services), host.lastError());
  const auto before = services.service_calls;
  services.configuring = true;
  const bool accepted = host.configureModules({{"example_heartbeat_observer", {{"sample_timeout", "0.25"}, {"publish_interval", "0.5"}}}});
  services.configuring = false;
  const auto error = host.lastError();
  require(!accepted && error.find("example_heartbeat_observer") != std::string::npos &&
          error.find("publish_interval") != std::string::npos && error.find("must not exceed") != std::string::npos,
          "checked host failed batch lacked useful diagnostic");
  require(services.service_calls == before && !services.effects && !services.publications,
          "failed checked host startup used Services");
  host.clear(); // A failed startup is never followed by operational dispatch.
  std::cout << "{\"phase\":\"host_configuration_rejected\",\"effects\":0,\"accepted\":false,\"configuration_service_calls\":0,\"publications\":0,\"error\":" << jsonText(error) << "}\n";
}

void configurationCases(const char* path) {
  const auto run = [&](const std::string& phase, const std::vector<ModuleSetting>& batch,
                       ModuleConfigStatus expected, const std::string& key = "", bool previous = false) {
    FakeServices services; PluginHost host;
    require(host.load(path, services), host.lastError());
    if (previous) require(configure(host, services, {{"sample_timeout", "1"}, {"publish_interval", "0.1"}}).status == ModuleConfigStatus::Accepted, "baseline configuration rejected");
    const auto result = configure(host, services, batch);
    require(result.status == expected && (expected == ModuleConfigStatus::Accepted || (result.key == key && !result.reason.empty())), "configuration classification/key/diagnostic mismatch: " + phase);
    std::cout << "{\"phase\":" << jsonText("config." + phase + ".result") << ",\"effects\":0,\"status\":"
              << jsonText(result.status == ModuleConfigStatus::Accepted ? "Accepted" : result.status == ModuleConfigStatus::UnknownKey ? "UnknownKey" : "Invalid")
              << ",\"key\":" << jsonText(result.key) << ",\"reason\":" << jsonText(result.reason) << ",\"configuration_service_calls\":0}\n";
    emit(host, services, "config." + phase + ".observation", 100);
  };
  run("defaults", {}, ModuleConfigStatus::Accepted);
  run("timeout_only", {{"sample_timeout", "4"}}, ModuleConfigStatus::Accepted);
  run("interval_only", {{"publish_interval", "0.125"}}, ModuleConfigStatus::Accepted);
  run("minimum", {{"sample_timeout", "0.25"}, {"publish_interval", "0.02"}}, ModuleConfigStatus::Accepted);
  run("maximum", {{"sample_timeout", "10"}, {"publish_interval", "2"}}, ModuleConfigStatus::Accepted);
  run("equal", {{"sample_timeout", "0.5"}, {"publish_interval", "0.5"}}, ModuleConfigStatus::Accepted);
  run("scientific", {{"sample_timeout", "1e0"}, {"publish_interval", "1e-1"}}, ModuleConfigStatus::Accepted);
  run("empty_replaces_previous", {}, ModuleConfigStatus::Accepted, "", true);
  run("unknown", {{"publish_interva1", "0.1"}}, ModuleConfigStatus::UnknownKey, "publish_interva1", true);
  run("valid_then_unknown", {{"sample_timeout", "5"}, {"publish_interva1", "0.1"}}, ModuleConfigStatus::UnknownKey, "publish_interva1", true);
  run("duplicate_timeout", {{"sample_timeout", "5"}, {"sample_timeout", "1"}}, ModuleConfigStatus::Invalid, "sample_timeout", true);
  run("duplicate_interval", {{"publish_interval", "0.25"}, {"publish_interval", "0.1"}}, ModuleConfigStatus::Invalid, "publish_interval", true);
  run("cross_field", {{"sample_timeout", "0.25"}, {"publish_interval", "0.5"}}, ModuleConfigStatus::Invalid, "publish_interval", true);
  run("cross_field_reversed", {{"publish_interval", "0.5"}, {"sample_timeout", "0.25"}}, ModuleConfigStatus::Invalid, "publish_interval", true);
  const std::vector<std::string> invalid = {"", "nan", "NaN", "inf", "-inf", "1e9999", "1e-9999", "1x", " 1", "1 ", "1\n", "0x1p0", "+1", "1,0", "-1", "0", "0.249", "10.01"};
  for (size_t i = 0; i < invalid.size(); ++i)
    run("invalid_timeout_" + std::to_string(i), {{"publish_interval", "0.2"}, {"sample_timeout", invalid[i]}}, ModuleConfigStatus::Invalid, "sample_timeout", true);
  for (size_t i = 0; i < 16; ++i)
    run("invalid_interval_" + std::to_string(i), {{"sample_timeout", "5"}, {"publish_interval", invalid[i]}}, ModuleConfigStatus::Invalid, "publish_interval", true);
  for (const auto& value : std::vector<std::pair<std::string, std::string>>{{"below", "0.019"}, {"above", "2.001"}})
    run("interval_" + value.first, {{"sample_timeout", "5"}, {"publish_interval", value.second}}, ModuleConfigStatus::Invalid, "publish_interval", true);
}

void guardSelfTest() {
  FakeServices guard; Services& api = guard;
  const std::vector<std::pair<std::string, std::function<void()>>> calls = {
    {"sendCommand", [&] { api.sendCommand({}); }},
    {"acquireNavigation", [&] { api.acquireNavigation("test"); }},
    {"revokeNavigation", [&] { api.revokeNavigation("test"); }},
    {"navigation", [&] { api.navigation({}); }},
    {"parameter", [&] { api.parameter({}); }},
    {"mission", [&] { api.mission({}); }},
    {"missionFinished", [&] { api.missionFinished("test"); }},
    {"recordStoredMissionProof", [&] { api.recordStoredMissionProof("test", {}); }},
    {"invalidateStoredMissionProof", [&] { api.invalidateStoredMissionProof("test"); }},
    {"beginMissionExecution", [&] { api.beginMissionExecution("test", 1, "content", 100); }},
    {"beginMissionExecutionAbort", [&] { api.beginMissionExecutionAbort("abort", "test", 100); }},
    {"finishMissionExecution", [&] { api.finishMissionExecution("test", "RELEASED"); }},
    {"suspendNavigation", [&] { api.suspendNavigation(100); }},
    {"resumeNavigation", [&] { api.resumeNavigation("test"); }},
    {"commandFinished", [&] { api.commandFinished("test"); }},
    {"cancelCommand", [&] { api.cancelCommand("test"); }},
    {"flightLog", [&] { api.flightLog({}); }},
    {"flightLogFinished", [&] { api.flightLogFinished("test"); }}
  };
  std::cout << "{\"phase\":\"effect_guards\",\"effects\":0,\"detected\":[";
  bool first = true;
  for (const auto& call : calls) {
    const auto before = guard.effects; bool rejected = false;
    try { call.second(); }
    catch (const std::runtime_error& error) { rejected = error.what() == "read-only example attempted effect: " + call.first; }
    require(rejected && guard.effects == before + 1, "effect guard failed: " + call.first);
    if (!first) std::cout << ','; first = false; std::cout << jsonText(call.first);
  }
  std::cout << "],\"intentional_guard_attempts\":" << guard.effects << "}\n";
}

void configuredLifecycle(const char* path) {
  FakeServices services; PluginHost host; require(host.load(path, services), host.lastError());
  configureStartup(host, services, {{"sample_timeout", "0.5"}, {"publish_interval", "0.125"}});
  emit(host, services, "configured.startup", 100);
  emit(host, services, "configured.before_due", 100.124);
  mavlink_message_t message{}; mavlink_msg_heartbeat_pack(1, 1, &message, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 4, MAV_STATE_STANDBY);
  services.clock = 100.125; host.dispatchMessage(message, {100.0625, 100.0625, "READ_ACQUISITION"});
  emit(host, services, "configured.fresh", 100.125);
  emit(host, services, "configured.not_early", 100.249);
  emit(host, services, "configured.cached", 100.25);
  emit(host, services, "configured.exact_expiry", 100.5625);
  emit(host, services, "configured.stale", 100.6875);
  services.clock = 100.75; host.reset("configured_reset"); emit(host, services, "configured.reset", 100.75);
  services.clock = 100.875; host.dispatchMessage(message, {100.8125, 100.8125, "READ_ACQUISITION"});
  emit(host, services, "configured.new_session", 100.875);
  require(!services.effects, "configured lifecycle effects");
}

int main(int argc, char** argv) {
  try {
    require(argc == 3, "module and wrong-ABI module required");
    guardSelfTest();
    configurationCases(argv[1]);
    checkedHostFailure(argv[1]);
    configuredLifecycle(argv[1]);
    FakeServices services;
    PluginHost rejected;
    require(!rejected.load(argv[2], services) && rejected.plugins().empty(), "wrong ABI accepted");
    require(rejected.lastError().find("ABI") != std::string::npos, "missing ABI diagnostic");
    PluginHost host;
    require(host.load(argv[1], services), host.lastError());
    require(host.plugins().size() == 1 && !host.controlBackend() && host.observationStreams().empty() &&
            host.subscriptions().empty() && host.hasCapability("example.heartbeat.observation"), "wrong composition");
    require(!host.load(argv[1], services) && host.plugins().size() == 1, "duplicate module accepted");
    void* image = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL | RTLD_NODELETE);
    require(image, "fixture cannot reopen module");
    auto create = reinterpret_cast<CreatePlugin>(dlsym(image, "mavmoos_create_plugin"));
    require(create && !create(plugin_abi + 1, &services) && !create(plugin_abi, nullptr), "factory must reject context/ABI");
    dlclose(image);
    configureStartup(host, services, {});
    auto emit = [&](const char* phase, double now) {
      services.clock = now; host.inputBoundary(); host.iterate(); require(!services.last.empty(), "no observation");
      std::cout << "{\"phase\":\"" << phase << "\",\"effects\":" << services.effects
                << ",\"publications\":" << services.publications << ",\"observation\":" << services.last << "}\n";
    };
    auto heartbeat = [](unsigned system) {
      mavlink_message_t message{};
      mavlink_msg_heartbeat_pack(system, 1, &message, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA,
                                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, MAV_STATE_STANDBY);
      return message;
    };
    auto receive = [&](double now, const MessageTime& time, unsigned system = 1) {
      services.clock = now; host.dispatchMessage(heartbeat(system), time);
    };
    emit("startup", 100);
    emit("default_not_early", 100.249);
    receive(100.25, {100.1, 100.2, "QUEUE_EMPTY_FLOOR"}); emit("fresh", 100.25);
    emit("cached", 101);
    receive(101.25, {101.3, 101.3, "READ_ACQUISITION"}); emit("future_rejected", 101.25);
    receive(101.5, {101.3, 101.4, "UNKNOWN"}); emit("basis_rejected", 101.5);
    receive(101.75, {101.6, 101.7, "QUEUE_EMPTY_FLOOR"}, 2); emit("source_rejected", 101.75);
    receive(102, {100.05, 100.06, "QUEUE_EMPTY_FLOOR"}); emit("order_rejected", 102);
    emit("stale", 102.5);
    services.clock = 102.75; host.reset("example_restart"); emit("reset", 102.75);
    receive(103, {102.6, 102.8, "QUEUE_EMPTY_FLOOR"}); emit("pre_reset_rejected", 103);
    receive(103.2, {103, 103.1, "QUEUE_EMPTY_FLOOR"}); emit("new_session", 103.25);
    emit("clock_fault", 102);
    receive(104, {103.8, 103.9, "QUEUE_EMPTY_FLOOR"}); emit("clock_latched", 104);
    services.clock = 104.25; host.reset("explicit_clock_recovery"); emit("clock_reset", 104.25);
    receive(104.5, {104.4, 104.4, "READ_ACQUISITION"}); emit("read_acquisition", 104.5);
    auto malformed = heartbeat(1); malformed.len = 0; services.clock = 104.75;
    host.dispatchMessage(malformed, {104.6, 104.7, "QUEUE_EMPTY_FLOOR"}); emit("wire_rejected", 104.75);
    host.clear(); require(services.effects == 0 && host.plugins().empty(), "effect or retained object after clear");
    std::cout << "{\"phase\":\"complete\",\"effects\":0,\"plugin_abi\":" << plugin_abi
              << ",\"wrong_abi_rejected\":true,\"null_context_rejected\":true,\"duplicate_rejected\":true}\n";
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
