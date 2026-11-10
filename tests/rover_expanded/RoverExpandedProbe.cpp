// Drive an owned simulation through MOOS. Independent FC readback lives in run_native.py.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#ifdef __APPLE__
#include <mach/mach_time.h>
#endif

namespace {
using Fields = std::map<std::string, std::string>;
constexpr double expiryWitnessSeconds = 4.2;

double monotonic()
{
#ifdef __APPLE__
  // Python time.monotonic() uses this timebase. libc++ steady_clock on Darwin
  // can also include system sleep and would misalign the two processes.
  static const auto timebase = [] {
    mach_timebase_info_data_t value{};
    mach_timebase_info(&value);
    return value;
  }();
  return static_cast<double>(mach_absolute_time()) * timebase.numer / timebase.denom / 1e9;
#else
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
#endif
}

Fields fields(const std::string& text)
{
  Fields result;
  std::istringstream input(text);
  std::string item;
  while(std::getline(input, item, ',')) {
    const auto eq = item.find('=');
    if(eq != std::string::npos)
      result.emplace(item.substr(0, eq), item.substr(eq + 1));
  }
  return result;
}

std::string field(const Fields& data, const std::string& key)
{
  const auto item = data.find(key);
  return item == data.end() ? "" : item->second;
}

double number(const Fields& data, const std::string& key)
{
  try { return std::stod(field(data, key)); }
  catch(const std::exception&) { return -1; }
}

bool on(const Fields& data, const std::string& key)
{
  const auto value = field(data, key);
  return value == "true" || value == "1";
}

std::string jsonString(const std::string& text)
{
  std::ostringstream out;
  out << '"';
  for(unsigned char c : text) {
    if(c == '"' || c == '\\') out << '\\' << static_cast<char>(c);
    else if(c >= 32) out << static_cast<char>(c);
    else out << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(c);
  }
  out << '"';
  return out.str();
}

void require(bool value, const std::string& reason)
{
  if(!value) throw std::runtime_error(reason);
}

struct Sample {
  double value;
  double received;
};

class Workflow {
public:
  Workflow(int port, const char* trace, const char* markers, const std::string& kind)
    : output(trace), control(markers), profile(kind)
  {
    require(bool(output), "workflow trace open failed");
    require(std::filesystem::is_directory(control), "owned control directory missing");
    require(port > 0 && port <= 65535 && (profile == "ground" || profile == "boat"),
            "invalid MOOS port or profile");
    client.SetQuiet(true);
    require(client.Run("127.0.0.1", port, "mavmoos_rover_expanded", 30), "MOOS connection failed");
  }

  ~Workflow() { client.Close(); }

  void run()
  {
    mark("ready");
    wait("ready", 55, [&] { return ready() && freshPosition(); });
    require(!on(runtime, "armed"), "fixture must start disarmed");
    notify("MAVMOOS_COMMAND", "id=flight,action=takeoff");
    result("flight", "REJECTED", 5);

    if(profile == "boat") {
      // The pinned hull model coasts for many seconds after the motors go
      // neutral. Measure unpowered current before any drive command.
      wait("initial native Hold", 10, [&] { return mode(4) && ready(); });
      command("arm", "arm");
      wait("armed initial Hold", 5, [&] { return on(runtime, "armed") && mode(4); });
      remainReleased("initial neutral boat Hold", 2, 4);
      mark("boat_passive_baseline_begin");
      remainReleased("current-only drift before any motor target", 8, 4);
      mark("boat_passive_baseline_done");
      command("guided", "guided");
    } else {
      command("guided", "guided");
      command("arm", "arm");
    }
    wait("armed Guided", 5, [&] { return on(runtime, "armed") && mode(15); });

    const double north = value("NAV_Y");
    const double east = value("NAV_X");
    mark("stock_waypoint_begin");
    notify("MOOS_MANUAL_OVERRIDE", "false");
    notify("DEPLOY", "true");
    acquire();
    wait("stock waypoint in two axes", 40, [&] {
      return active() && freshPosition() && value("NAV_Y") - north >= 4 &&
        value("NAV_X") - east >= 3 && helmScalars.size() == 2;
    });
    mark("stock_waypoint_done");

    mark("allstop_begin");
    notify("DEPLOY", "false");
    wait("all-stop revocation", 4, [&] { return latched() && !permission(); });
    require(field(helm, "allstop") != "clear", "stock all-stop was not observed");
    wait("all-stop settling", 6, [&] {
      return freshPosition() && value("NAV_SPEED") <= .4;
    });
    mark("allstop_stopped");
    notify("DEPLOY", "true");
    remainReleased("restored stock mail cannot regain expired permission", expiryWitnessSeconds);
    requireStopped("all-stop renewal witness");
    resumeMotion("allstop_reacquired");

    mark("pilot_takeover_begin");
    marker("takeover.request");
    wait("pilot Manual and helm latch", 8, [&] { return mode(0) && !permission() && latched(); });
    remainReleased("pilot Manual must persist", 1.4, 0);
    mark("pilot_takeover_done");
    require(std::filesystem::remove(control / "takeover.request"), "pilot Hold handback request missing");
    wait("pilot selected Hold", 5, [&] { return mode(4) && !permission(); });
    command("guided-after-manual", "guided");
    remainReleased("Guided after pilot must not grant helm", .35, 15);
    resumeMotion("manual_reacquired");

    mark("link_loss_begin");
    marker("pause.request");
    wait("heartbeat loss and helm revocation", 6,
         [&] { return !on(runtime, "heartbeat_valid") && !permission() && latched(); });
    mark("link_loss_detected");
    wait("selected Rover link returned", 10, [&] { return ready() && freshPosition(); });
    mark("link_fresh_observations");
    // The pinned boat may reverse briefly while internal Guided Loiter
    // catches its stopping point after the old attitude target expires.
    const double heldTransmissions = number(runtime, "nav_tx_accepted");
    remainReleased("expired pre-loss Guided target and settled recovery", expiryWitnessSeconds, 15, true);
    mark("link_settlement_requested");
    marker("link_settled.request");
    wait("independent link settling witness", 6, [&] {
      require(!permission() && mode(15) && number(runtime, "nav_tx_accepted") == heldTransmissions,
              "link recovery attempted to resume before independent settlement");
      return std::filesystem::is_regular_file(control / "link_settled.verified") &&
             freshPosition() && value("NAV_SPEED") <= .4;
    });
    mark("link_settlement_acknowledged");
    mark("link_recovered");
    resumeMotion("link_reacquired");

    mark("native_hold_begin");
    command("native-hold", "hold", profile == "boat" ? 27 : 15);
    wait("Hold authority", 4, [&] { return mode(4) && !permission(); });
    remainReleased("native Hold must block stock helm", 2, 4);
    mark("native_hold_drift_begin");
    remainReleased("native Hold persists after drive settles", profile == "boat" ? 8 : 1, 4);
    mark("native_hold_done");
    notify("DEPLOY", "false");
    if(profile == "boat") {
      mark("native_loiter_begin");
      operation("boat-loiter", "loiter", "native", "");
      wait("boat native Loiter authority", 4, [&] { return mode(5) && !permission(); });
      mark("native_loiter_settled");
      remainReleased("native Loiter must retain FC authority", 25, 5);
      mark("native_loiter_done");
      notify("DEPLOY", "true");
      resumeMotion("loiter_reacquired");
      notify("DEPLOY", "false");
      notify("MAVMOOS_NAV_CONTROL", "release");
      wait("release after Loiter recovery", 7, [&] {
        return !permission() && freshPosition() && value("NAV_SPEED") <= .4;
      });
    } else {
      notify("MAVMOOS_CONTROL_REQUEST", "schema=1,id=ground-loiter,action=loiter,strategy=native");
      result("ground-loiter", "REJECTED", 5, "MAVMOOS_CONTROL_RESULT");
    }

    command("direct-guided", "guided");
    remainReleased("direct Guided must not grant helm", .35, 15);
    mark("guided_goto_begin");
    operation("goto", "goto", "guided", ",lat=42.358501,lon=-71.087589,speed=1.2,timeout=50");
    mark("guided_goto_done");
    mark("guided_hold_begin");
    operation("guided-hold", "hold", "guided", "");
    remainReleased("guided hold is independent of stock helm", 1.5, 15);
    mark("guided_hold_done");

    mark("guided_cancel_begin");
    request("cancel-target", "goto", "guided", ",lat=42.358560,lon=-71.087450,speed=1.2,timeout=30");
    const double before = value("NAV_Y");
    wait("cancellable operation must first move", 15,
         [&] { return freshPosition() && value("NAV_Y") - before >= 1.3; });
    notify("MAVMOOS_CONTROL_REQUEST", "schema=1,id=cancel,action=cancel,target=cancel-target");
    result("cancel", "COMPLETED", 5, "MAVMOOS_CONTROL_RESULT");
    result("cancel-target", "CANCELLED", 5, "MAVMOOS_CONTROL_RESULT");
    wait("cancel settles motion", 8, [&] {
      return freshPosition() && value("NAV_SPEED") <= .4;
    });
    mark("guided_cancel_stopped");
    remainReleased("cancel cannot renew guidance", expiryWitnessSeconds, 15);
    requireStopped("cancel renewal witness");
    mark("guided_cancel_done");

    wait("fresh home for guided and native return", 6, [&] {
      return ready() && on(runtime, "home_valid") && freshPosition();
    });
    mark("guided_return_begin");
    operation("guided-return", "return", "guided", ",speed=1.2,timeout=35");
    mark("guided_return_done");
    operation("rtl-departure", "goto", "guided",
              ",lat=42.358528,lon=-71.087589,speed=1.2,timeout=30");
    mark("native_rtl_begin");
    operation("native-rtl", "return", "native", ",timeout=45");
    wait("native RTL owns mode", 4, [&] { return mode(11) && !permission(); });
    mark("native_rtl_done");
    command("final-hold", "hold", profile == "boat" ? 27 : 15);
    command("disarm", "disarm");
    wait("final disarmed stop", 5, [&] {
      return !on(runtime, "armed") && !permission() && mode(4) && freshPosition() && value("NAV_SPEED") <= .4;
    });
    remainReleased("disarmed final hold", 1.5, 4);
    log("result", "PASS", "stock 2D helm, expiry, pilot, link recovery, Guided actions and native actions");
  }

  void passive(const std::string& expected)
  {
    passiveMode = expected;
    wait("mismatched sibling MAV_TYPE", 10, [&] {
      return field(app, "state") == expected && field(app, "identity") == "MISMATCH" &&
             number(runtime, "nav_tx_accepted") == 0;
    });
    remainReleased("mismatched profile stays ineligible", .7);
    require(!freshPosition() && !on(runtime, "heartbeat_valid") && !on(runtime, "position_valid"),
            "mismatched sibling profile admitted position or heartbeat");
    mark("sibling_type_rejected");
    marker("expected.request");
    wait("expected MAV_TYPE accepts position", 10, [&] {
      return field(app, "identity") == "EXPECTED" && ready() && freshPosition();
    });
    mark("selected_type_observed");
    notify("MAVMOOS_COMMAND", "id=passive-arm,action=arm");
    notify("MAVMOOS_CONTROL_REQUEST", "schema=1,id=passive-hold,action=hold,strategy=guided");
    notify("MOOS_MANUAL_OVERRIDE", "false");
    notify("DEPLOY", "true");
    acquire();
    if(expected == "SHADOW") {
      wait("stock helm shadow target", 7, [&] {
        return field(helm, "state") == "ACTIVE" && std::isfinite(value("MAVMOOS_SHADOW_COURSE")) &&
          value("MAVMOOS_SHADOW_SPEED") > 0 && helmScalars.size() == 2;
      });
      mark("shadow_target_computed");
    } else {
      remainReleased("receive-only requests cannot acquire", 1.3);
      require(!std::isfinite(value("MAVMOOS_SHADOW_COURSE")), "receive-only app computed a shadow target");
    }
    mark("passive_no_effects");
    log("result", "PASS", "sibling profile rejected, selected type observed, no egress");
  }

  void fail(const std::string& message) { log("result", "FAIL", message); }

private:
  void log(const std::string& kind, const std::string& key, const std::string& text)
  {
    output << std::fixed << std::setprecision(6)
           << "{\"mono\":" << monotonic() << ",\"type\":" << jsonString(kind)
           << ",\"key\":" << jsonString(key) << ",\"value\":" << jsonString(text) << "}\n";
    output.flush();
  }

  void mark(const std::string& name) { log("phase", name, "observed"); }

  void notify(const std::string& key, const std::string& text)
  {
    require(client.Notify(key, text), "MOOS Notify failed for " + key);
    log("request", key, text);
  }

  void marker(const std::string& name)
  {
    std::ofstream signal(control / name);
    require(bool(signal), "owned fixture marker failed");
    signal << "owned simulation event\n";
  }

  void fetch()
  {
    if(client.IsConnected() && !registered) {
      for(const auto* key : {"MAVMOOS_CAPABILITIES", "MAVMOOS_STATUS", "MAVMOOS_RUNTIME_STATUS",
          "MAVMOOS_COMMAND_RESULT", "MAVMOOS_CONTROL_RESULT", "MAVMOOS_CONTROL_PROGRESS",
          "MAVMOOS_TX_RESULT", "MAVMOOS_HELM_STATUS", "NAV_X", "NAV_Y", "NAV_SPEED",
          "MAVMOOS_SHADOW_COURSE", "MAVMOOS_SHADOW_SPEED",
          "DESIRED_HEADING", "DESIRED_SPEED", "DESIRED_ALTITUDE", "IVPHELM_ALLSTOP", "IVPHELM_STATE"})
        client.Register(key, 0);
      registered = true;
    }
    MOOSMSG_LIST mail;
    client.Fetch(mail);
    for(const auto& message : mail) {
      const auto key = message.GetKey();
      const auto source = message.GetSource();
      if(source == "iMAVLink" && message.IsString()) {
        const auto text = message.GetString();
        if(key == "MAVMOOS_RUNTIME_STATUS") runtime = fields(text);
        if(key == "MAVMOOS_STATUS") app = fields(text);
        if(!passiveMode.empty() && key == "MAVMOOS_STATUS") {
          require(field(app, "state") == passiveMode && field(app, "tx_enabled") == "false" &&
                  number(app, "mavlink_tx_attempts") == 0 && number(app, "mavlink_tx_accepted") == 0,
                  "passive app left zero-egress mode");
        }
        if(!passiveMode.empty() && key == "MAVMOOS_RUNTIME_STATUS")
          require(!permission() && number(runtime, "tx_attempts") == 0 && number(runtime, "tx_accepted") == 0,
                  "passive runtime admitted control or attempted egress");
        if(key == "MAVMOOS_HELM_STATUS") helm = fields(text);
        if(key == "MAVMOOS_COMMAND_RESULT" || key == "MAVMOOS_CONTROL_RESULT") {
          const auto parsed = fields(text);
          results[field(parsed, "id")][key] = parsed;
        }
        log("observation", key, text);
      }
      if(source == "iMAVLink" && message.IsDouble()) {
        if(key.rfind("NAV_", 0) == 0) nav[key] = {message.GetDouble(), monotonic()};
        if(key.rfind("MAVMOOS_SHADOW_", 0) == 0) {
          nav[key] = {message.GetDouble(), monotonic()};
          log("observation", key, std::to_string(message.GetDouble()));
        }
      }
      if(source == "pHelmIvP" && key.rfind("DESIRED_", 0) == 0) {
        require(key != "DESIRED_ALTITUDE", "2D stock helm emitted altitude");
        if(message.IsDouble()) {
          helmScalars.insert(key);
          log("stock_helm", key, std::to_string(message.GetDouble()));
        }
      }
      if(source == "pHelmIvP" && message.IsString() && key.rfind("IVPHELM_", 0) == 0)
        log("stock_helm", key, message.GetString());
    }
  }

  template<typename Predicate>
  void wait(const std::string& label, double timeout, Predicate predicate)
  {
    const double end = monotonic() + timeout;
    while(monotonic() < end) {
      fetch();
      if(predicate()) return;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error(label + " timed out; helm=" + field(helm, "state") + ":" +
      field(helm, "reason") + ", FC=" + field(runtime, "fc_mode") + ":" + field(runtime, "reason"));
  }

  void result(const std::string& id, const std::string& expected, double timeout,
              const std::string& key = "MAVMOOS_COMMAND_RESULT")
  {
    wait(id + " " + expected, timeout, [&] {
      const auto& item = results[id][key];
      const auto status = field(item, "status");
      if(status == expected) return true;
      if(status == "REJECTED" || status == "FAILED" || status == "TIMED_OUT" ||
         status == "CANCELLED" || status == "EXPIRED" || status == "COMPLETED")
        throw std::runtime_error(id + " unexpected " + status + ":" + field(item, "reason"));
      return false;
    });
  }

  void command(const std::string& id, const std::string& action, double resultTimeout = 15)
  {
    notify("MAVMOOS_COMMAND", "id=" + id + ",action=" + action);
    result(id, "COMPLETED", resultTimeout);
  }

  void request(const std::string& id, const std::string& action, const std::string& strategy,
               const std::string& fields)
  {
    notify("MAVMOOS_CONTROL_REQUEST", "schema=1,id=" + id + ",action=" + action +
         ",strategy=" + strategy + fields);
  }

  void operation(const std::string& id, const std::string& action, const std::string& strategy,
                 const std::string& fields)
  {
    request(id, action, strategy, fields);
    result(id, "COMPLETED", 52, "MAVMOOS_CONTROL_RESULT");
  }

  bool mode(int expected) const { return field(runtime, "fc_mode") == std::to_string(expected); }
  bool permission() const { return on(runtime, "permission"); }
  bool latched() const { return field(helm, "state") == "LATCHED"; }
  bool active() const { return field(helm, "state") == "ACTIVE" && permission() && mode(15); }

  bool ready() const
  {
    const double flags = number(runtime, "estimator_flags");
    const double age = number(runtime, "estimator_age_s");
    if(flags < 0 || flags > 65535 || !std::isfinite(flags)) return false;
    const auto valid = static_cast<unsigned>(flags);
    constexpr unsigned horizontalReady = 19;
    constexpr unsigned uninitialized = 1024;
    return on(runtime, "heartbeat_valid") && on(runtime, "position_valid") &&
      on(runtime, "estimator_horizontal_valid") && age >= 0 && age <= 3 &&
      (valid & horizontalReady) == horizontalReady && (valid & uninitialized) == 0;
  }

  double value(const std::string& key) const
  {
    const auto item = nav.find(key);
    return item == nav.end() ? NAN : item->second.value;
  }

  bool freshPosition() const
  {
    for(const auto* key : {"NAV_X", "NAV_Y", "NAV_SPEED"}) {
      const auto item = nav.find(key);
      if(item == nav.end() || monotonic() - item->second.received > .5 ||
         !std::isfinite(item->second.value)) return false;
    }
    return true;
  }

  void acquire() { notify("MAVMOOS_NAV_CONTROL", "acquire"); }

  void remainReleased(const std::string& label, double duration, int expectedMode = -1, bool settle = false)
  {
    const double start = monotonic();
    const double transmissions = number(runtime, "nav_tx_accepted");
    double stoppedSince = -1;
    wait(label, duration + (settle ? 5 : 1), [&] {
      require(!permission(), label);
      require(expectedMode < 0 || mode(expectedMode), label + ": mode changed");
      require(number(runtime, "nav_tx_accepted") == transmissions, label + ": unexpected target egress");
      const double now = monotonic();
      if(settle) {
        if(!freshPosition() || value("NAV_SPEED") > .4) stoppedSince = -1;
        else if(stoppedSince < 0) stoppedSince = now;
        if(stoppedSince < 0 || now - stoppedSince < 1) return false;
      }
      return now - start >= duration;
    });
  }

  void requireStopped(const std::string& label) const
  {
    require(freshPosition() && value("NAV_SPEED") <= .4, label + ": fresh stopped observation unavailable");
  }

  void resumeMotion(const std::string& name)
  {
    const double transmissions = number(runtime, "nav_tx_accepted");
    const double x = value("NAV_X");
    const double y = value("NAV_Y");
    mark(name + "_begin");
    acquire();
    wait(name, 14, [&] {
      return active() && number(runtime, "nav_tx_accepted") > transmissions && freshPosition() &&
             std::hypot(value("NAV_X") - x, value("NAV_Y") - y) >= 1.2;
    });
    mark(name + "_done");
  }

  CMOOSCommClient client;
  std::ofstream output;
  std::filesystem::path control;
  std::string profile;
  bool registered = false;
  std::string passiveMode;
  Fields app;
  Fields runtime;
  Fields helm;
  std::map<std::string, std::map<std::string, Fields>> results;
  std::map<std::string, Sample> nav;
  std::set<std::string> helmScalars;
};
}

int main(int argc, char** argv)
{
  try {
    if(argc == 2 && std::string(argv[1]) == "--clock") {
      std::cout << std::setprecision(17) << monotonic() << '\n';
      return 0;
    }
    require(argc == 5 || argc == 6,
            "usage: rover_expanded_probe MOOSDB_PORT TRACE CONTROL_DIRECTORY ground|boat [RX_ONLY|SHADOW]");
    Workflow workflow(std::stoi(argv[1]), argv[2], argv[3], argv[4]);
    try {
      if(argc == 5) workflow.run();
      else {
        const std::string mode = argv[5];
        require(mode == "RX_ONLY" || mode == "SHADOW", "unknown passive mode");
        workflow.passive(mode);
      }
    } catch(const std::exception& error) {
      workflow.fail(error.what());
      throw;
    }
    return 0;
  } catch(const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
