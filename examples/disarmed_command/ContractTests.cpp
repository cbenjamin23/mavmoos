// Load the actual standalone module with a deterministic fake owner clock.
#include "PluginHost.h"
#include <dlfcn.h>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace mavmoos::runtime;
namespace {
void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
bool contains(const std::string& text, const std::string& substring) {
  return text.find(substring) != std::string::npos;
}
class FakeServices final : public Services {
public:
  double clock = 100;
  Vehicle cache;
  std::vector<CommandRequest> commands;
  std::vector<std::string> cancelled, finished, reports;
  bool allow = true, busy = false, navigation_granted = false, configuring = false;

  void accessed() const { require(!configuring, "module configuration accessed Services"); }
  double now() const override { accessed(); return clock; }
  const Vehicle& vehicle() const override { accessed(); return cache; }
  void publish(const std::string& key, const std::string& value) override {
    accessed();
    require(key == "EXAMPLE_DISARM_RESULT", "unexpected result publication");
    reports.push_back(value);
  }
  void publish(const std::string&, double) override { forbidden("numeric publication"); }
  bool sendCommand(const CommandRequest& request) override {
    accessed();
    require(request.command == MAV_CMD_COMPONENT_ARM_DISARM &&
            request.params == std::array<float, 7>{} && request.execution_id.empty() &&
            request.abort_id.empty() && request.id.rfind("example.disarm.", 0) == 0 &&
            clock < request.expires && request.expires <= request.operation_expires,
            "example attempted a different command, parameters or expiry");
    commands.push_back(request);
    return allow;
  }
  void cancelCommand(const std::string& id) override { accessed(); cancelled.push_back(id); }
  void commandFinished(const std::string& id) override { accessed(); finished.push_back(id); }
  bool controlBusy() const override { accessed(); return busy; }
  bool navigationGranted() const override { accessed(); return navigation_granted; }
  bool acquireNavigation(const std::string&) override { forbidden("acquireNavigation"); }
  void revokeNavigation(const std::string&) override { forbidden("revokeNavigation"); }
  bool navigation(const NavigationTarget&) override { forbidden("navigation"); }
  bool suspendNavigation(double) override { forbidden("suspendNavigation"); }
  bool resumeNavigation(const std::string&) override { forbidden("resumeNavigation"); }
  bool parameter(const ParameterRequest&) override { forbidden("parameter"); }
  bool mission(const MissionRequest&) override { forbidden("mission"); }
  void missionFinished(const std::string&) override { forbidden("missionFinished"); }
  bool recordStoredMissionProof(const std::string&, const std::vector<MissionItem>&) override {
    forbidden("recordStoredMissionProof");
  }
  void invalidateStoredMissionProof(const std::string&) override { forbidden("invalidateStoredMissionProof"); }
  bool beginMissionExecution(const std::string&, uint64_t, const std::string&, double) override {
    forbidden("beginMissionExecution");
  }
  bool beginMissionExecutionAbort(const std::string&, const std::string&, double) override {
    forbidden("beginMissionExecutionAbort");
  }
  bool finishMissionExecution(const std::string&, const std::string&) override { forbidden("finishMissionExecution"); }
private:
  [[noreturn]] static void forbidden(const char* name) {
    throw std::runtime_error(std::string("unexpected Services effect ") + name);
  }
};
struct Rig {
  FakeServices services;
  PluginHost host;
  explicit Rig(const char* path, std::string source = {}) {
    require(host.load(path, services, "example_disarmed_command"), host.lastError());
    std::map<std::string, std::vector<ModuleSetting>> settings;
    if (!source.empty()) settings["example_disarmed_command"] = {{"request_source", source}};
    services.configuring = true;
    const auto accepted = host.configureModules(settings);
    services.configuring = false;
    require(accepted, host.lastError());
    require(host.subscriptions() == std::vector<std::string>{"EXAMPLE_DISARM_REQUEST"} &&
            !host.controlBackend(), "the example claimed a second backend or extra mail");
  }
  void ground(double stamp) {
    services.clock = stamp;
    auto& v = services.cache;
    v.heartbeat_valid = v.landed_valid = true;
    v.heartbeat_at = v.landed_at = stamp;
    v.landed_state = MAV_LANDED_STATE_ON_GROUND;
    v.type = MAV_TYPE_QUADROTOR;
    v.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    v.armed = v.in_air = false;
  }
  void mail(const std::string& value, double published = 0, const std::string& source = "uPokeDB") {
    const auto now = services.clock;
    host.dispatchMail({"EXAMPLE_DISARM_REQUEST", value, source, 0, published ? published : now, now, false});
  }
  void request(const std::string& id = "one", double published = 0, const std::string& source = "uPokeDB") {
    mail("schema=1,id=" + id + ",action=disarm,timeout=3", published, source);
  }
  void cancel(const std::string& target = "one") {
    mail("schema=1,id=stop,action=cancel,timeout=1,target=" + target);
  }
  void tick() { host.inputBoundary(); host.iterate(); }
  void sent(bool accepted = true) {
    require(!services.commands.empty(), "missing submitted command");
    host.commandSent(services.commands.back(), services.clock, accepted);
  }
  void ack(double effective, double read, unsigned source = 1, unsigned component = 1,
           unsigned target = 245, unsigned result = MAV_RESULT_ACCEPTED,
           unsigned command = MAV_CMD_COMPONENT_ARM_DISARM, unsigned target_component = 191) {
    mavlink_message_t message{};
    mavlink_status_t encoder{};
    mavlink_msg_command_ack_pack_status(source, component, &encoder, &message,
      command, result, 0, 0, target, target_component);
    host.dispatchMessage(message, {effective, read, effective == read ? "READ_ACQUISITION" : "QUEUE_EMPTY_FLOOR"});
  }
  bool result(const std::string& id, const std::string& status, const std::string& reason = {}) const {
    for (const auto& row : services.reports) {
      if (contains(row, "\"id\":\"" + id + "\"") && contains(row, "\"status\":\"" + status + "\"") &&
          (reason.empty() || contains(row, "\"reason\":\"" + reason + "\""))) return true;
    }
    return false;
  }
};

void loaderControls(char** paths) {
  FakeServices services;
  PluginHost host;
  require(!host.load(paths[2], services, "example_disarmed_command") &&
          contains(host.lastError(), "ABI"), "wrong ABI loaded");
  require(!host.load(paths[1], services, "other_owner") &&
          contains(host.lastError(), "selected module ID"), "wrong selected owner loaded");
  require(host.load(paths[1], services, "example_disarmed_command"), host.lastError());
  require(!host.load(paths[3], services, "example_disarm_conflicting_owner") &&
          contains(host.lastError(), "duplicate request owner"), "competing request owner loaded");
  require(host.plugins().size() == 1 && services.commands.empty(), "failed composition caused an effect");
  void* library = dlopen(paths[1], RTLD_NOW | RTLD_LOCAL);
  require(library != nullptr, "could not reopen module for context guard");
  const auto create = reinterpret_cast<CreatePlugin>(dlsym(library, "mavmoos_create_plugin"));
  const bool rejects_null = create && !create(plugin_abi, nullptr);
  dlclose(library);
  require(rejects_null, "factory accepted null context");
  const auto run = [&](const std::vector<ModuleSetting>& settings, const std::string& diagnostic) {
    FakeServices rejected_services;
    PluginHost rejected;
    require(rejected.load(paths[1], rejected_services), rejected.lastError());
    rejected_services.configuring = true;
    const auto accepted = rejected.configureModules({{"example_disarmed_command", settings}});
    rejected_services.configuring = false;
    require(!accepted && contains(rejected.lastError(), diagnostic) &&
            rejected_services.reports.empty() && rejected_services.commands.empty(),
            "invalid module startup did not fail cleanly");
  };
  run({{"command", "400"}}, "unknown disarm setting");
  run({{"request_source", "two words"}}, "case-sensitive MOOS source token");
  run({{"observation_timeout", "1.501"}}, "[0.5,1.5]");
  run({{"observation_timeout", "nan"}}, "[0.5,1.5]");
  run({{"request_source", "one"}, {"request_source", "two"}}, "duplicate");
}
}

int main(int argc, char** argv) {
  try {
    require(argc == 4, "usage: example_disarm_contract MODULE WRONG_ABI CONFLICTING_OWNER");
    loaderControls(argv);
    unsigned passed = 0;
    const auto test = [&](const char* name, const auto& run) {
      Rig rig(argv[1]);
      rig.ground(101);
      run(rig);
      std::cout << "PASS " << name << '\n';
      ++passed;
    };
    test("request then bounded submission, real callback, ACK and two later observations", [](Rig& r) {
      r.tick();
      require(r.services.commands.empty(), "automatic command");
      r.request("one", 100.7);
      require(r.result("one", "REQUESTED") && r.services.commands.empty(), "mail sent directly");
      r.tick();
      require(r.services.commands.size() == 1 && r.result("one", "SUBMITTED") &&
              !r.result("one", "SENT"), "submission treated as handoff");
      const auto command = r.services.commands.front();
      require(command.operation_expires == 103.7 && command.expires == 102.5, "original deadline mismatch");
      r.sent();
      require(r.result("one", "SENT") && !r.result("one", "COMPLETED"), "handoff treated as completion");
      r.services.clock = 101.2;
      r.ack(100.9, 101.2);
      require(r.result("one", "ACKNOWLEDGED") && !r.result("one", "COMPLETED"), "ACK treated as completion");
      r.services.cache.heartbeat_at = 101.3;
      r.services.clock = 101.3;
      r.ack(101.3, 101.3);
      require(!r.result("one", "COMPLETED"), "only one post-send observation completed");
      r.ground(101.4);
      r.ack(101.4, 101.4);
      require(r.result("one", "COMPLETED") && r.services.finished == std::vector<std::string>{command.id},
              "observed completion did not retire the lease");
      r.request();
      r.tick();
      require(r.services.commands.size() == 1 && r.result("one", "REJECTED"), "duplicate executed");
    });
    test("unrelated malformed stale and unowned requests are inert", [](Rig& r) {
      const std::vector<std::string> requests = {
        "schema=1,id=one,action=disarm,timeout=1,force=true",
        "schema=1,id=one,action=arm,timeout=3",
        "schema=1,id=one,action=disarm,timeout=1,",
        "schema=1,id=one,id=two,action=disarm,timeout=1",
        "schema=1,id=one,action=disarm,timeout=nan",
        "schema=1,id=one,action=disarm,timeout=5.01",
        "schema=1,id=one,action=disarm,timeout=2s",
        "schema=1,id=one,action=cancel,timeout=1,target=one"
      };
      for (const auto& request : requests) r.mail(request);
      r.request("one", 101, "OtherSource");
      r.request("one", 102);
      r.request("one", 100);
      r.host.dispatchMail({"MAVMOOS_COMMAND", "action=disarm", "uPokeDB", 0, 101, 101, false});
      r.host.dispatchMail({"EXAMPLE_DISARM_REQUEST", {}, "uPokeDB", 400, 101, 101, true});
      r.tick();
      require(r.services.commands.empty() && r.result("", "REJECTED", "invalid_request") &&
              r.result("", "REJECTED", "wrong_request_source"), "invalid or foreign request queued");
    });
    test("armed airborne stale and competing control are independently denied", [](Rig& r) {
      r.services.cache.armed = true;
      r.request("armed");
      r.services.cache.armed = false;
      r.services.cache.landed_state = MAV_LANDED_STATE_IN_AIR;
      r.request("airborne");
      r.ground(101.1);
      r.services.clock = 102.6;
      r.request("stale");
      r.ground(102.7);
      r.services.busy = true;
      r.request("busy");
      r.services.busy = false;
      r.services.navigation_granted = true;
      r.request("nav");
      r.tick();
      for (auto id : {"armed", "airborne", "stale", "busy", "nav"})
        require(r.result(id, "REJECTED"), std::string("admitted ") + id);
      require(r.services.commands.empty(), "guard rejection had a command");
    });
    test("fresh original publication and packet expiry are strict", [](Rig& r) {
      r.services.clock = 101.25;
      r.mail("schema=1,id=old,action=disarm,timeout=1", 100.25);
      require(r.result("old", "REJECTED"), "request at original expiry accepted");
      r.request("local", 100.5);
      r.services.clock = 102.5;
      r.ground(102.5);
      r.tick();
      require(r.result("local", "TIMED_OUT", "packet_expiry_reached_before_submission") &&
              r.services.commands.empty(), "new observations renewed the original packet cutoff");
    });
    test("pre-handoff cancellation, invalid cancel target and replay", [](Rig& r) {
      r.request();
      r.mail("schema=1,id=bad,action=cancel,timeout=1,target=wrong");
      r.cancel();
      r.tick();
      require(r.result("one", "CANCELLED") && r.result("stop", "CANCEL_ACCEPTED") &&
              r.result("bad", "REJECTED") && r.services.commands.empty() &&
              r.services.cancelled.empty(), "local cancellation changed host or sent");
    });
    test("queued cancellation retains uncertainty and ignores late accepted callback", [](Rig& r) {
      r.request();
      r.tick();
      r.cancel();
      r.sent();
      r.ground(101.1);
      r.ack(101.1, 101.1);
      r.request("two");
      require(r.result("one", "LATE_HANDOFF") && !r.result("one", "COMPLETED") &&
              r.result("two", "REJECTED", "control_busy") && r.services.finished.empty() &&
              r.services.cancelled == std::vector<std::string>{"example.disarm.one"},
              "late handoff or ACK revived cancelled work or released lease");
      r.ground(104);
      r.request("three");
      r.tick();
      require(r.services.commands.size() == 2, "original bound did not release local quarantine");
    });
    test("post-handoff loss and reset preserve uncertainty and cut the prior session", [](Rig& r) {
      r.request();
      r.tick();
      r.sent();
      r.services.clock = 101.1;
      r.host.reset("connection_lost");
      r.ground(101.2);
      r.ack(101.2, 101.2);
      r.request("old", 101.05);
      r.request("too_soon");
      require(r.result("one", "LOST") && !r.result("one", "ACKNOWLEDGED") &&
              r.result("old", "REJECTED", "request_time_invalid_or_expired") &&
              r.result("too_soon", "REJECTED", "control_busy") && r.services.finished.empty(),
              "session reset revived old work");
    });
    test("only fresh matching post-handoff ACK from bound target counts", [](Rig& r) {
      r.request();
      r.tick();
      r.sent();
      r.services.clock = 101.3;
      r.ack(100.99, 101);
      r.ack(101.1, 101.2, 2);
      r.ack(101.1, 101.2, 1, 2);
      r.ack(101.1, 101.2, 1, 1, 246);
      r.ack(101.1, 101.2, 1, 1, 245, MAV_RESULT_ACCEPTED, MAV_CMD_NAV_TAKEOFF);
      r.ack(101.1, 101.2, 1, 1, 245, MAV_RESULT_ACCEPTED, MAV_CMD_COMPONENT_ARM_DISARM, 190);
      r.ack(101.1, 101.2, 1, 1, 245, MAV_RESULT_IN_PROGRESS);
      require(!r.result("one", "ACKNOWLEDGED"), "unmatched ACK counted");
      r.ack(101.1, 101.2, 1, 1, 245, MAV_RESULT_DENIED);
      r.ground(101.4);
      r.ack(101.4, 101.4);
      require(r.result("one", "FAILED", "fc_rejected_disarm") && !r.result("one", "COMPLETED") &&
              r.services.finished.empty(), "rejected result released an uncertain accepted handoff");
    });
    test("ACK and disarmed ground at original expiry cannot complete", [](Rig& r) {
      r.mail("schema=1,id=one,action=disarm,timeout=1");
      r.tick();
      r.sent();
      r.ground(102);
      r.ack(102, 102);
      require(r.result("one", "TIMED_OUT") && !r.result("one", "ACKNOWLEDGED") &&
              !r.result("one", "COMPLETED"), "original expiry refreshed");
    });
    test("host reject and unaccepted OS callback terminate without uncertainty", [](Rig& r) {
      r.services.allow = false;
      r.request("blocked");
      r.tick();
      require(r.result("blocked", "REJECTED", "host_admission_denied") &&
              r.services.finished.empty(), "host refusal touched a lease");
      r.services.allow = true;
      r.request();
      r.tick();
      r.sent(false);
      require(r.result("one", "FAILED", "local_handoff_not_accepted") &&
              r.services.finished == std::vector<std::string>{"example.disarm.one"} &&
              !r.result("one", "SENT"), "failed handoff looked accepted");
    });
    test("invalid clock cancels and requires valid reset", [](Rig& r) {
      r.request();
      r.tick();
      r.services.clock = std::numeric_limits<double>::quiet_NaN();
      r.tick();
      r.ground(101.2);
      r.request("clock");
      r.tick();
      require(r.result("one", "LOST", "owner_clock_invalid") &&
              r.result("", "REJECTED", "owner_clock_invalid") && r.services.commands.size() == 1,
              "invalid clock silently recovered");
    });
    test("bounded history rejects new IDs without replay eviction", [](Rig& r) {
      r.services.cache.armed = true;
      for (unsigned i = 0; i < 64; ++i) r.request("id" + std::to_string(i));
      r.ground(101.1);
      r.request("id64");
      r.request("id0");
      r.tick();
      require(r.result("id64", "REJECTED", "duplicate_or_exhausted_request_history") &&
              r.result("id0", "REJECTED", "duplicate_or_exhausted_request_history") &&
              r.services.commands.empty(), "bounded history evicted a prior ID");
    });
    test("history exhaustion still permits exact active cancellation", [](Rig& r) {
      r.services.cache.armed = true;
      for (unsigned i = 0; i < 63; ++i) r.request("id" + std::to_string(i));
      r.ground(101.1);
      r.request();
      r.cancel("foreign");
      r.mail("schema=1,id=real_cancel,action=cancel,timeout=1,target=one");
      r.tick();
      require(r.result("one", "CANCELLED") && r.result("real_cancel", "CANCEL_ACCEPTED") &&
              r.services.commands.empty(), "history exhaustion blocked cancellation");
    });
    test("configured shorter observation bound persists after invalid batch", [](Rig& r) {
      auto* plugin = r.host.plugins().front();
      r.services.configuring = true;
      const auto ok = plugin->configureModule({{"observation_timeout", "0.75"}});
      const auto bad = plugin->configureModule({{"observation_timeout", "1.4"}, {"request_source", "two words"}});
      r.services.configuring = false;
      require(ok.status == ModuleConfigStatus::Accepted && bad.status == ModuleConfigStatus::Invalid,
              "invalid candidate was not rejected");
      r.services.clock = 101.75;
      r.request();
      r.tick();
      require(r.result("one", "REJECTED", "fresh_disarmed_ground_required") && r.services.commands.empty(),
              "rejected batch renewed observation age");
    });
    Rig configured(argv[1], "explicit_source");
    configured.ground(101);
    configured.request("default");
    configured.request("configured", 101, "explicit_source");
    configured.tick();
    require(configured.result("", "REJECTED", "wrong_request_source") &&
            configured.services.commands.size() == 1 && configured.result("configured", "SUBMITTED"),
            "checked source setting not used");
    std::cout << "PASS " << passed + 2 << " groups including actual ABI loader, owner and checked configuration\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
