// Independent MOOS operator for the owned, disarmed FTP fixture.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <thread>

namespace {
constexpr auto request_key = "MAVMOOS_FILE_REQUEST";
constexpr auto result_key = "MAVMOOS_FILE_RESULT";
constexpr auto catalog_key = "MAVMOOS_FILE_CATALOG";
constexpr auto tx_key = "MAVMOOS_FILE_TX_RESULT";
constexpr auto rx_key = "MAVMOOS_FILE_RX_RESULT";
constexpr auto runtime_key = "MAVMOOS_RUNTIME_STATUS";
constexpr auto capabilities_key = "MAVMOOS_CAPABILITIES";
constexpr auto app_key = "MAVMOOS_STATUS";

double now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void require(bool condition, const char* reason)
{
  if(!condition) throw std::runtime_error(reason);
}

std::string quote(const std::string& text)
{
  constexpr auto hex = "0123456789abcdef";
  std::string out = "\"";
  for(const unsigned char c : text) {
    if(c == '"' || c == '\\') {
      out += '\\';
      out += static_cast<char>(c);
    } else if(c < 32) {
      out += "\\u00";
      out += hex[c >> 4];
      out += hex[c & 15];
    } else {
      out += static_cast<char>(c);
    }
  }
  return out + '"';
}

std::map<std::string, std::string> fields(const std::string& text)
{
  std::map<std::string, std::string> out;
  std::size_t start = 0;
  while(start < text.size()) {
    const auto end = text.find(',', start);
    const auto part = text.substr(start, end == std::string::npos ? end : end - start);
    const auto equal = part.find('=');
    if(equal != std::string::npos) out.emplace(part.substr(0, equal), part.substr(equal + 1));
    if(end == std::string::npos) break;
    start = end + 1;
  }
  return out;
}

bool jsonValue(const std::string& row, const std::string& key, const std::string& value)
{
  return row.find(quote(key) + ":" + quote(value)) != std::string::npos;
}

void log(const char* kind, const std::string& key, const std::string& value, double current)
{
  std::cout << "{\"kind\":" << quote(kind) << ",\"key\":" << quote(key)
            << ",\"mono\":" << std::setprecision(17) << current
            << ",\"value\":" << quote(value) << "}\n";
}

enum class Stage { Startup, ListBinary, Binary, ListEmpty, Empty, ListCancel, BeginCancel, EndCancel,
                   ListAfter, After, Done };
}

int main(int argc, char** argv)
{
  CMOOSCommClient client;
  try {
    require(argc == 2, "usage: mavlink_files_moos_probe MOOS_PORT");
    std::size_t used = 0;
    const std::string input = argv[1];
    const auto port = std::stoi(input, &used);
    require(used == input.size() && port > 0 && port <= 65535, "invalid MOOS port");
    client.SetQuiet(true);
    require(client.Run("127.0.0.1", port, "mavmoos_files_probe", 50), "MOOS probe startup failed");
    const auto deadline = now() + 50;
    auto stage = Stage::Startup;
    double finish = 0;
    bool registered = false, ground = false, composition = false, first_read = false;
    std::set<std::string> catalogs;
    std::map<std::string, std::string> outcomes;
    auto notify = [&](const std::string& text, Stage next) {
      const auto request = "schema=1," + text;
      require(client.Notify(request_key, request), "MOOS operator request handoff failed");
      log("request", request_key, request, now());
      stage = next;
    };

    while(now() < deadline) {
      if(client.IsConnected() && !registered) {
        for(const auto key : {result_key, catalog_key, tx_key, rx_key, runtime_key, capabilities_key, app_key})
          require(client.Register(key, 0), "MOOS probe registration failed");
        registered = true;
      }
      MOOSMSG_LIST mail;
      client.Fetch(mail);
      for(const auto& message : mail) {
        if(message.GetSource() != "iMAVLink" || !message.IsString()) continue;
        const auto key = message.GetKey(), value = message.GetString();
        log("app", key, value, now());
        if(key == runtime_key) {
          auto state = fields(value);
          ground = state["mode"] == "SITL_CONTROL" && state["heartbeat_valid"] == "1" &&
            state["landed_valid"] == "1" && state["armed"] == "0" && state["in_air"] == "0" &&
            state["permission"] == "false" && state["control_profile"] == "none";
        } else if(key == capabilities_key) {
          composition = jsonValue(value, "execution_mode", "SITL_CONTROL") &&
            jsonValue(value, "name", "mavlink_files");
        } else if(key == catalog_key) {
          for(const auto id : {"list1", "list2", "list3", "list4"})
            if(jsonValue(value, "id", id)) catalogs.insert(id);
        } else if(key == tx_key) {
          auto fields_of_reply = fields(value);
          first_read |= fields_of_reply["operation_id"] == "getstop" && fields_of_reply["opcode"] == "5" &&
            fields_of_reply["status"] == "LOCAL_ACCEPTED";
        } else if(key == result_key) {
          for(const auto id : {"list1", "list2", "list3", "list4", "get1", "getempty", "getstop", "stop1", "get2"}) {
            if(!jsonValue(value, "id", id)) continue;
            for(const auto status : {"LISTED", "COMPLETE", "CANCELLED", "FAILED", "REJECTED", "EXPIRED"}) {
              if(jsonValue(value, "status", status)) outcomes[id] = status;
            }
            if(outcomes[id] == "FAILED" || outcomes[id] == "REJECTED" || outcomes[id] == "EXPIRED")
              throw std::runtime_error("operator request failed: " + value);
          }
        }
      }
      if(stage == Stage::Startup && composition && ground)
        notify("id=list1,action=list", Stage::ListBinary);
      else if(stage == Stage::ListBinary && catalogs.count("list1") && outcomes["list1"] == "LISTED")
        notify("id=get1,action=get,catalog=list1,name=sample.bin", Stage::Binary);
      else if(stage == Stage::Binary && outcomes["get1"] == "COMPLETE")
        notify("id=list2,action=list", Stage::ListEmpty);
      else if(stage == Stage::ListEmpty && catalogs.count("list2") && outcomes["list2"] == "LISTED")
        notify("id=getempty,action=get,catalog=list2,name=empty.bin", Stage::Empty);
      else if(stage == Stage::Empty && outcomes["getempty"] == "COMPLETE")
        notify("id=list3,action=list", Stage::ListCancel);
      else if(stage == Stage::ListCancel && catalogs.count("list3") && outcomes["list3"] == "LISTED")
        notify("id=getstop,action=get,catalog=list3,name=cancel.bin", Stage::BeginCancel);
      else if(stage == Stage::BeginCancel && first_read)
        notify("id=stop1,action=cancel,operation_id=getstop", Stage::EndCancel);
      else if(stage == Stage::EndCancel && outcomes["getstop"] == "CANCELLED")
        notify("id=list4,action=list", Stage::ListAfter);
      else if(stage == Stage::ListAfter && catalogs.count("list4") && outcomes["list4"] == "LISTED")
        notify("id=get2,action=get,catalog=list4,name=sample.bin", Stage::After);
      else if(stage == Stage::After && outcomes["get2"] == "COMPLETE") {
        finish = now() + .3;
        stage = Stage::Done;
      }
      std::cout.flush();
      if(stage == Stage::Done && now() >= finish) {
        client.Close();
        return 0;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    throw std::runtime_error("FTP operator workflow did not complete before the probe deadline");
  } catch(const std::exception& error) {
    client.Close();
    std::cerr << "mavlink_files probe: " << error.what() << '\n';
    return 1;
  }
}
