// Exact Runtime and loaded DSOs. The final I/O adapter is a deterministic fake.
#include "Runtime.h"
#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace mavmoos::runtime;
namespace {
void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
bool contains(const std::string& text, const std::string& substring) {
  return text.find(substring) != std::string::npos;
}
struct Frame {
  mavlink_message_t message{};
  std::string hex;
  double expires = 0;
  bool accepted = false;
};
struct Rig {
  double time = 100;
  bool io_success = true;
  std::function<void(double)> at_disarm_gate;
  std::vector<Frame> frames;
  std::vector<std::pair<std::string, std::string>> reports;
  Runtime host;
  mavlink_status_t fc{};
  Rig(char** paths, bool simulation = true, bool shadow = false, bool backend = true)
    : host([this](const auto& name, const auto& text, double, bool numeric) {
          if (!numeric) reports.emplace_back(name, text);
        }, [](const uint8_t*, std::size_t) { throw std::runtime_error("unbounded fallback sender"); return false; },
        simulation, shadow, [this] { return time; }, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, true) {
    host.setFirmwareRequestsEnabled(false);
    if (backend) require(host.load(paths[1], "arducopter_control"), host.lastConfigurationError());
    require(host.load(paths[2], "example_disarmed_command"), host.lastConfigurationError());
    host.configure("control_native_actions", "false");
    host.configure("control_helm", "false");
    host.configure("control_mission_execution", "false");
    host.configureModules({{"example_disarmed_command", {{"request_source", "disarm_contract"}}}});
    host.finishStartup();
    require(host.controlProfile() == (backend ? "arducopter" : "") &&
            host.subscriptions() == std::vector<std::string>{"EXAMPLE_DISARM_REQUEST"},
            "unexpected composition, ordinary request facets or backend");
    host.setBoundedSend([this](const uint8_t* data, std::size_t size, double expiry, const auto& eligible) {
      Frame frame;
      mavlink_message_t receive{};
      mavlink_status_t parse{}, output{};
      unsigned complete = 0;
      std::ostringstream hex;
      hex << std::hex << std::setfill('0');
      for (std::size_t i = 0; i < size; ++i) {
        hex << std::setw(2) << unsigned(data[i]);
        if (mavlink_frame_char_buffer(&receive, &parse, data[i], &frame.message, &output) == MAVLINK_FRAMING_OK)
          ++complete;
      }
      require(complete == 1, "Runtime handed off a non-frame or more than one frame");
      frame.hex = hex.str();
      frame.expires = expiry;
      if (frame.message.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        mavlink_command_long_t command{};
        mavlink_msg_command_long_decode(&frame.message, &command);
        if (command.command == MAV_CMD_COMPONENT_ARM_DISARM && at_disarm_gate) at_disarm_gate(expiry);
      }
      frame.accepted = eligible(time) && io_success;
      frames.push_back(frame);
      return frame.accepted;
    });
  }
  void heartbeat(double at, bool armed = false) {
    time = at;
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack_status(1, 1, &fc, &message, MAV_TYPE_QUADROTOR,
      MAV_AUTOPILOT_ARDUPILOTMEGA, armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0, 0, MAV_STATE_STANDBY);
    host.message(message, at);
  }
  void landed(double at, unsigned state = MAV_LANDED_STATE_ON_GROUND) {
    time = at;
    mavlink_message_t message{};
    mavlink_msg_extended_sys_state_pack_status(1, 1, &fc, &message, MAV_VTOL_STATE_UNDEFINED, state);
    host.message(message, at);
  }
  void ground(double at) { heartbeat(at); landed(at + .001); }
  void ack(double effective, double read) {
    time = read;
    mavlink_message_t message{};
    mavlink_msg_command_ack_pack_status(1, 1, &fc, &message, MAV_CMD_COMPONENT_ARM_DISARM,
      MAV_RESULT_ACCEPTED, 0, 0, 245, 191);
    host.message(message, effective, read, "QUEUE_EMPTY_FLOOR");
  }
  void mail(const std::string& key, const std::string& text, double published,
            const std::string& source = "disarm_contract") {
    host.mail({key, text, source, 0, published, time, false});
  }
  void request(const std::string& id = "one", double published = 100.9, double timeout = 3) {
    mail("EXAMPLE_DISARM_REQUEST", "schema=1,id=" + id + ",action=disarm,timeout=" +
         std::to_string(timeout), published);
  }
  void cancel() {
    mail("EXAMPLE_DISARM_REQUEST", "schema=1,id=stop,action=cancel,target=one,timeout=1", time);
  }
  void tick(double at) { time = at; host.tick(at); }
  bool result(const std::string& id, const std::string& status, const std::string& detail = {}) const {
    for (const auto& entry : reports) {
      if (entry.first == "EXAMPLE_DISARM_RESULT" && contains(entry.second, "\"id\":\"" + id + "\"") &&
          contains(entry.second, "\"status\":\"" + status + "\"") &&
          (detail.empty() || contains(entry.second, detail))) return true;
    }
    return false;
  }
  std::vector<Frame> disarms() const {
    std::vector<Frame> result;
    for (const auto& frame : frames) {
      if (frame.message.msgid == MAVLINK_MSG_ID_HEARTBEAT) continue;
      require(frame.message.msgid == MAVLINK_MSG_ID_COMMAND_LONG, "unexpected egress message");
      mavlink_command_long_t command{};
      mavlink_msg_command_long_decode(&frame.message, &command);
      if (command.command == MAV_CMD_SET_MESSAGE_INTERVAL) continue;
      require(command.command == MAV_CMD_COMPONENT_ARM_DISARM, "unexpected command family");
      require(frame.message.sysid == 245 && frame.message.compid == 191 &&
              command.target_system == 1 && command.target_component == 1 &&
              command.confirmation == 0 && command.param1 == 0 && command.param2 == 0 &&
              command.param3 == 0 && command.param4 == 0 && command.param5 == 0 &&
              command.param6 == 0 && command.param7 == 0, "unexpected disarm wire fields");
      result.push_back(frame);
    }
    return result;
  }
};
}

int main(int argc, char** argv) {
  try {
    require(argc == 3, "usage: external_disarm_runtime COPTER_BACKEND EXAMPLE");
    unsigned passed = 0;
    const auto test = [&](const char* name, const auto& run) {
      Rig rig(argv);
      rig.ground(101);
      run(rig);
      std::cout << "PASS " << name << '\n';
      ++passed;
    };
    test("real Runtime fixed send, post-send read ACK and observed ground", [](Rig& r) {
      r.tick(101.02);
      r.mail("MAVMOOS_COMMAND", "schema=1,id=alternate,action=disarm,timeout=3", 101.02);
      r.tick(101.04);
      require(r.disarms().empty(), "ordinary disabled owner or startup sent DISARM");
      r.request();
      r.tick(101.05);
      const auto frames = r.disarms();
      require(frames.size() == 1 && frames[0].accepted && frames[0].expires == 102.5 &&
              r.result("one", "SENT") && r.host.controlBusy() && !r.host.navigationGranted(),
              "accepted command or original packet fence missing");
      r.ack(101.04, 101.05);
      require(!r.result("one", "ACKNOWLEDGED"), "read at handoff accepted");
      r.ack(101.04, 101.06);
      require(r.result("one", "ACKNOWLEDGED") && !r.result("one", "COMPLETED") && r.host.controlBusy(),
              "fast ACK rejected or treated as result");
      r.landed(101.2);
      require(!r.result("one", "COMPLETED"), "missing post-handoff heartbeat ignored");
      r.heartbeat(101.3);
      require(r.result("one", "COMPLETED") && !r.host.controlBusy() && !r.host.navigationGranted(),
              "post-handoff ground did not complete");
      r.tick(101.4);
      require(r.disarms().size() == 1, "operation retried automatically");
      std::cout << "{\"scope\":\"real Runtime bytes at bounded fake I/O\",\"command\":400,"
                << "\"wire_hex\":\"" << frames[0].hex << "\",\"packet_expires\":" << frames[0].expires << "}\n";
    });
    test("original request already expired and local cancel never reach socket adapter", [](Rig& r) {
      r.request("expired", 100.001, 1);
      r.request();
      r.cancel();
      r.tick(101.02);
      require(r.result("expired", "REJECTED") && r.result("one", "CANCELLED") &&
              r.result("stop", "CANCEL_ACCEPTED") && r.disarms().empty() && !r.host.controlBusy(),
              "expired or local-cancelled work queued");
    });
    test("cancel during final handoff check stops wire and preserves original lease", [](Rig& r) {
      r.request();
      r.at_disarm_gate = [&](double) { r.cancel(); };
      r.tick(101.05);
      const auto frames = r.disarms();
      require(frames.size() == 1 && !frames[0].accepted && r.result("one", "CANCELLED") &&
              !r.result("one", "SENT") && r.host.controlBusy(), "cancelled wire or premature lease retirement");
      r.ground(103.8);
      r.request("busy", 103.8);
      require(r.result("busy", "REJECTED"), "cancel extended/released authority unexpectedly");
      r.ground(103.9);
      require(!r.host.controlBusy() && r.disarms().size() == 1, "lease did not expire without retry");
    });
    test("armed observation injected before final handoff cancels before the backend gate", [](Rig& r) {
      r.request();
      r.at_disarm_gate = [&](double) { r.heartbeat(r.time + .001, true); };
      r.tick(101.05);
      const auto frames = r.disarms();
      require(frames.size() == 1 && !frames[0].accepted && r.result("one", "LOST") &&
              !r.result("one", "SENT"), "disarm crossed newly observed armed state");
    });
    test("final sender at original packet deadline rejects even when Runtime prepares wire", [](Rig& r) {
      r.request();
      r.at_disarm_gate = [&](double deadline) { r.time = deadline; };
      r.tick(101.05);
      const auto frames = r.disarms();
      require(frames.size() == 1 && !frames[0].accepted && r.result("one", "FAILED") &&
              !r.host.controlBusy(), "packet deadline crossed or unsent lease retained");
    });
    test("post-handoff cancel cannot be undone by ACK/fresh state or queue a retry", [](Rig& r) {
      r.request();
      r.tick(101.05);
      r.time = 101.06;
      r.cancel();
      r.ack(101.05, 101.07);
      r.ground(101.1);
      r.request("second", 101.1);
      r.tick(101.12);
      require(r.result("one", "CANCELLED", "\"remote_effect_uncertain\":true") &&
              r.result("second", "REJECTED") && !r.result("one", "COMPLETED") &&
              r.disarms().size() == 1 && r.host.controlBusy(), "sent cancel lost bounded uncertainty");
    });
    test("local OS refusal is terminal and sends no second command", [](Rig& r) {
      r.io_success = false;
      r.request();
      r.tick(101.05);
      r.io_success = true;
      r.tick(101.1);
      const auto frames = r.disarms();
      require(frames.size() == 1 && !frames[0].accepted && r.result("one", "FAILED") &&
              !r.result("one", "SENT") && !r.host.controlBusy(), "failed I/O retried or remained in flight");
    });
    test("original operation expiry excludes ACK and later state", [](Rig& r) {
      r.request("one", 100.9, 1);
      r.tick(101.05);
      r.ack(101.1, 101.9);
      r.ground(102);
      require(r.result("one", "TIMED_OUT") && !r.result("one", "ACKNOWLEDGED") &&
              !r.result("one", "COMPLETED") && !r.host.controlBusy() && r.disarms().size() == 1,
              "original expiry did not terminate the actual host lease");
    });
    for (const auto mode : {0, 1, 2}) {
      Rig r(argv, mode != 0, mode == 1, mode != 2);
      r.ground(101);
      r.request();
      r.tick(101.05);
      require(r.disarms().empty() && r.result("one", "REJECTED") && !r.host.controlBusy(),
              "RX/shadow/missing backend admitted command");
      if (mode != 2) require(r.frames.empty(), "RX or shadow emitted any MAVLink bytes");
      ++passed;
    }
    std::cout << "PASS " << passed << " actual Runtime groups\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
