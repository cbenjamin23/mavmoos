// Bounded operator for the owned native Copter bench.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
constexpr auto request_key = "MAVMOOS_MISSION_REQUEST";
constexpr auto result_key = "MAVMOOS_MISSION_INSPECTION_RESULT";
constexpr auto snapshot_key = "MAVMOOS_MISSION_INSPECTION_FENCE_SNAPSHOT";
constexpr auto runtime_key = "MAVMOOS_RUNTIME_STATUS";

double now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::map<std::string, std::string> fields(const std::string& text)
{
  std::istringstream input(text);
  std::map<std::string, std::string> result;
  std::string field;
  while (std::getline(input, field, ',')) {
    const auto at = field.find('=');
    if (at != std::string::npos)
      result.emplace(field.substr(0, at), field.substr(at + 1));
  }
  return result;
}

bool terminal(const std::string& status)
{
  return status == "COMPLETED" || status == "FAILED" || status == "TIMED_OUT" ||
    status == "UNSUPPORTED" || status == "REJECTED" || status == "CANCELLED" ||
    status == "DUPLICATE" || status == "INCOMPLETE";
}

int run(unsigned port, const std::filesystem::path& out, unsigned steps)
{
  CMOOSCommClient client;
  const auto started = now();
  const auto log = [&](const auto& key, const auto& value) {
    std::cout << "{\"elapsed\":" << std::fixed << std::setprecision(6) << now() - started
      << ",\"key\":" << std::quoted(key) << ",\"value\":" << std::quoted(value) << "}" << std::endl;
  };
  client.SetQuiet(true);
  if (!client.Run("127.0.0.1", port, "mavmoos_fence_operator", 50))
    throw std::runtime_error("MOOS connection failed");
  bool registered = false, capability = false, snapshot = false, completed = false;
  bool interrupt = false, cancel_sent = false;
  std::string pending;
  unsigned step = 0;
  double requested_at = 0, runtime_at = 0;
  while (now() - started < 55) {
    if (client.IsConnected() && !registered) {
      for (const auto* key : {runtime_key, result_key, snapshot_key, "MAVMOOS_CAPABILITIES",
                             "MAVMOOS_MISSION_TX_RESULT", "MAVMOOS_STORED_MISSION_PROOF",
                             "IVPHELM_STATE", "MAVMOOS_HELM_STATUS"})
        client.Register(key, 0);
      registered = true;
    }
    MOOSMSG_LIST mail;
    client.Fetch(mail);
    for (const auto& message : mail) {
      const auto& key = message.GetKey();
      if (key == "IVPHELM_STATE" || key == "MAVMOOS_HELM_STATUS")
        throw std::runtime_error("unexpected helm traffic in storage-only bench");
      if (message.GetSource() != "iMAVLink" || !message.IsString())
        continue;
      const auto& value = message.GetString();
      log(key, value);
      if (key == "MAVMOOS_CAPABILITIES")
        capability = value.find("missions.fence.replace.simulator") != std::string::npos;
      if (key == runtime_key) {
        auto runtime = fields(value);
        if (runtime["armed"] != "0" || runtime["permission"] != "false" ||
            runtime["nav_tx_accepted"] != "0" || runtime["control_profile"] != "none")
          throw std::runtime_error("unexpected motion authority in storage-only bench");
        runtime_at = runtime["heartbeat_valid"] == "1" ? now() : 0;
      }
      if (pending.empty())
        continue;
      if (interrupt && !cancel_sent && key == "MAVMOOS_MISSION_TX_RESULT") {
        const auto sent = fields(value);
        if (sent.at("operation_id") == pending && sent.at("msgid") == "73" &&
            sent.at("status") == "LOCAL_ACCEPTED") {
          const auto cancel = "schema=1,id=" + pending + ",action=cancel,type=fence";
          if (!client.Notify(request_key, cancel))
            throw std::runtime_error("failed to request cancellation");
          log(request_key, cancel);
          cancel_sent = true;
        }
      }
      if (key == snapshot_key && value.find("\"status\":\"complete\"") != std::string::npos &&
          value.find("\"id\":\"" + pending + "\"") != std::string::npos)
        snapshot = true;
      if (key == result_key) {
        auto result = fields(value);
        if (result["id"] != pending || !terminal(result["status"]))
          continue;
        const auto expected = interrupt ? "CANCELLED" : "COMPLETED";
        if (result["status"] != expected || result["type"] != "fence" || (interrupt && !cancel_sent))
          throw std::runtime_error("unexpected fence outcome " + value);
        completed = true;
      }
    }
    if (!pending.empty() && completed && (snapshot || interrupt)) {
      log("complete", pending);
      pending.clear();
      if (++step == steps) {
        client.Close();
        return 0;
      }
    }
    if (!pending.empty() && now() - requested_at > 10)
      throw std::runtime_error("fence result deadline for " + pending);
    const auto gate = out / ("step-" + std::to_string(step) + ".request");
    if (pending.empty() && capability && now() - runtime_at < .4 &&
        std::filesystem::is_regular_file(gate)) {
      std::string request;
      std::ifstream input(gate);
      if (!std::getline(input, request))
        throw std::runtime_error("operator request is missing");
      auto data = fields(request);
      pending = data["id"];
      if (pending.empty() || data["schema"] != "1" || data["type"] != "fence" ||
          !client.Notify(request_key, request))
        throw std::runtime_error("invalid or unsent operator request");
      log(request_key, request);
      requested_at = now();
      snapshot = completed = cancel_sent = false;
      interrupt = pending == "replace-interrupted";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  throw std::runtime_error("bounded fence operator did not complete");
}
}

int main(int argc, char** argv)
{
  try {
    if (argc != 4)
      throw std::runtime_error("usage: fence_moos_probe DB_PORT OUTPUT_DIR STEPS");
    return run(std::stoul(argv[1]), argv[2], std::stoul(argv[3]));
  } catch (const std::exception& error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
