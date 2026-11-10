// One bounded MOOS client test. The probe has no MAVLink socket.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <vector>

namespace {
constexpr char source[] = "mavmoos_disarm_probe";
constexpr char foreign_source[] = "mavmoos_disarm_foreign";
constexpr char request_key[] = "EXAMPLE_DISARM_REQUEST";
constexpr char result_key[] = "EXAMPLE_DISARM_RESULT";
double now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::string quote(const std::string& text) {
  constexpr char hex[] = "0123456789abcdef";
  std::string out = "\"";
  for (unsigned char c : text) {
    if (c == '"' || c == '\\') { out += '\\'; out += static_cast<char>(c); }
    else if (c < 32) { out += "\\u00"; out += hex[c >> 4]; out += hex[c & 15]; }
    else out += static_cast<char>(c);
  }
  return out + '"';
}
bool contains(const std::string& text, const std::string& field) {
  return text.find(field) != std::string::npos;
}
bool field(const std::string& text, const std::string& key, const std::string& value) {
  return contains("," + text + ",", "," + key + "=" + value + ",");
}
void require(bool condition, const std::string& why) {
  if (!condition) throw std::runtime_error(why);
}

class Probe {
public:
  void run(int port) {
    client_.SetQuiet(true);
    foreign_.SetQuiet(true);
    require(client_.Run("127.0.0.1", port, source, 50) &&
            foreign_.Run("127.0.0.1", port, foreign_source, 50), "start MOOS clients");
    wait(10, [&] {
      return registered_ && foreign_.IsConnected() &&
        field(status_, "mode", "SITL_CONTROL") && field(status_, "control_profile", "arducopter") &&
        field(status_, "permission", "false") && field(status_, "armed", "0") &&
        field(status_, "in_air", "0") && field(status_, "heartbeat_valid", "1") &&
        field(status_, "landed_valid", "1") && field(status_, "command_leases", "0") &&
        field(status_, "nav_tx_accepted", "0") && !catalog_.empty();
    }, "initial disarmed ground observations");
    sendAndWait(foreign_, foreign_source, "foreign", "schema=1,id=foreign,action=disarm,timeout=5",
                "", "REJECTED", "wrong_request_source");
    sendAndWait(client_, source, "invalid", "schema=1,id=invalid,action=disarm,timeout=5,force=true",
                "", "REJECTED", "invalid_request");
    sendAndWait(client_, source, "expired", "schema=1,id=expired,action=disarm,timeout=1",
                "expired", "REJECTED", "request_time_invalid_or_expired", 4);
    sendAndWait(client_, source, "missing_cancel", "schema=1,id=missing,action=cancel,target=absent,timeout=1",
                "missing", "REJECTED", "cancel_target_not_active");
    sendAndWait(client_, source, "positive", "schema=1,id=positive,action=disarm,timeout=5",
                "positive", "COMPLETED", "fresh_disarmed_ground_observed");
    sendAndWait(client_, source, "duplicate", "schema=1,id=positive,action=disarm,timeout=5",
                "positive", "REJECTED", "duplicate_or_exhausted_request_history");
    const auto until = now() + 1.2;
    while (now() < until) pump();
    require(field(status_, "heartbeat_valid", "1") && field(status_, "landed_valid", "1") &&
            field(status_, "armed", "0") && field(status_, "in_air", "0") &&
            field(status_, "command_leases", "0") && field(status_, "permission", "false") &&
            field(status_, "nav_tx_accepted", "0"), "final disarmed ground and empty lease");
    std::cout << "{\"event\":\"done\",\"elapsed\":" << now() - started_ << "}\n";
  }
private:
  void pump() {
    if (registered_) require(client_.IsConnected() && foreign_.IsConnected(), "MOOS client disconnected");
    if (!registered_ && client_.IsConnected() && foreign_.IsConnected()) {
      for (const auto* key : {result_key, "MAVMOOS_TX_RESULT", "MAVMOOS_RUNTIME_STATUS", "MAVMOOS_CAPABILITIES"})
        require(client_.Register(key, 0), "MOOS subscription failed");
      registered_ = true;
    }
    MOOSMSG_LIST mail;
    client_.Fetch(mail);
    for (const auto& message : mail) {
      if (message.GetSource() != "iMAVLink" || !message.IsString()) continue;
      require(++records_ < 10000, "MOOS record bound reached");
      const auto key = message.GetKey();
      const auto text = message.GetString();
      if (key == result_key) results_.push_back(text);
      else if (key == "MAVMOOS_RUNTIME_STATUS") status_ = text;
      else if (key == "MAVMOOS_CAPABILITIES") catalog_ = text;
      else if (key != "MAVMOOS_TX_RESULT") continue;
      std::cout << "{\"event\":\"mail\",\"elapsed\":" << now() - started_
                << ",\"key\":" << quote(key) << ",\"source\":" << quote(message.GetSource())
                << ",\"value\":" << quote(text) << "}\n";
    }
    std::cout.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  template<typename Predicate>
  void wait(double seconds, const Predicate& ready, const std::string& phase) {
    const auto deadline = now() + seconds;
    while (now() < deadline) {
      pump();
      if (now() < deadline && ready()) return;
    }
    throw std::runtime_error("probe deadline reached waiting for " + phase);
  }
  void sendAndWait(CMOOSCommClient& sender, const char* publisher, const char* phase,
                   const char* request, const char* id, const char* status, const char* reason,
                   double age = 0) {
    const auto start = results_.size();
    const auto wall = MOOSTime();
    require(std::isfinite(wall) && wall > age, "invalid MOOS time");
    std::cout << "{\"event\":\"request\",\"elapsed\":" << now() - started_
              << ",\"phase\":" << quote(phase) << ",\"source\":" << quote(publisher)
              << ",\"key\":" << quote(request_key) << ",\"value\":" << quote(request)
              << ",\"moos_sample_at\":" << wall << ",\"moos_published_at\":" << wall - age << "}\n"
              << std::flush;
    require(sender.Notify(request_key, request, wall - age), "MOOS Notify failed");
    wait(6, [&] {
      for (auto i = start; i < results_.size(); ++i) {
        const auto& text = results_[i];
        if (contains(text, "\"id\":" + quote(id)) && contains(text, "\"status\":" + quote(status)) &&
            contains(text, "\"reason\":" + quote(reason))) return true;
        if (std::string_view(phase) == "positive" && contains(text, "\"id\":\"positive\"") &&
            (contains(text, "\"status\":\"REJECTED\"") || contains(text, "\"status\":\"FAILED\"") ||
             contains(text, "\"status\":\"LOST\"") || contains(text, "\"status\":\"TIMED_OUT\"")))
          throw std::runtime_error("positive request failed: " + text);
      }
      return false;
    }, phase);
  }

  CMOOSCommClient client_, foreign_;
  const double started_ = now();
  bool registered_ = false;
  unsigned records_ = 0;
  std::string status_, catalog_;
  std::vector<std::string> results_;
};
}

int main(int argc, char** argv) {
  try {
    require(argc == 2, "usage: disarm_sitl_probe DB_PORT");
    std::size_t used = 0;
    const auto port = std::stoi(argv[1], &used);
    require(used == std::string(argv[1]).size() && port >= 1 && port <= 65535, "invalid MOOS port");
    std::cout << std::setprecision(17);
    Probe{}.run(port);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Disarm probe: " << error.what() << '\n';
    return 1;
  }
}
