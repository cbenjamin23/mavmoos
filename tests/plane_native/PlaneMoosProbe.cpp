// Owned stock-MOOS fixture probe. It never publishes a helm output variable.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"

#include <chrono>
#include <cmath>
#include <cerrno>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <unistd.h>

namespace {
double clockSeconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string quoted(const std::string& value)
{
  static constexpr char hex[] = "0123456789abcdef";
  std::string result = "\"";
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') {
      result += '\\';
      result += static_cast<char>(c);
    } else if (c < 32 || c >= 127) {
      result += "\\u00";
      result += hex[c >> 4];
      result += hex[c & 15];
    } else result += static_cast<char>(c);
  }
  return result + '"';
}

void require(bool condition, const char* detail)
{
  if (!condition) throw std::runtime_error(detail);
}

bool allow(const std::string& key, const std::string& value)
{
  if (key == "AUTONOMY_GRADE_READY") return value == "true";
  if (key == "AUTONOMY_ROUTE" || key == "AUTONOMY_DETOUR")
    return value.size() > 7 && value.size() < 120 && value.compare(0, 7, "points=") == 0 &&
      value.find_first_not_of("0123456789.,:-", 7) == std::string::npos;
  if (key == "DEPLOY") return value == "true" || value == "false";
  if (key == "MAVMOOS_NAV_CONTROL") return value == "acquire" || value == "release";
  if (key == "MAVMOOS_AUTO_RECOVERY_REQUEST") {
    const std::string prefix = "schema=1,id=fixture-approach-handback,action=handback,session=";
    return value.size() > prefix.size() && value.size() <= 160 && value.compare(0, prefix.size(), prefix) == 0 &&
      value.find_first_not_of("0123456789abcdef", prefix.size()) == std::string::npos;
  }
  if (key == "MAVMOOS_COMMAND")
    return value == "id=fixture-loiter,action=loiter" || value == "id=fixture-return,action=rtl" ||
      value == "id=fixture-manual,action=manual" || value == "id=fixture-arm,action=arm" ||
      value == "id=fixture-air-disarm,action=disarm" ||
      value == "id=fixture-disarm,action=disarm" || value == "id=fixture-speed,action=change_speed,delta=-4" ||
      value == "id=fixture-speed-rejected,action=change_speed,delta=-8" ||
      value == "id=fixture-speed-minimum,action=reset_speed_min";
  if (key == "MAVMOOS_MISSION_REQUEST" || key == "MAVMOOS_MISSION_CONTROL" || key == "MAVMOOS_CONTROL_REQUEST") {
    const std::string prefix=key=="MAVMOOS_MISSION_REQUEST"?"id=fixture-":"schema=1,id=fixture-";
    return value.size()<=1024&&value.compare(0,prefix.size(),prefix)==0;
  }
  if (key == "PLANE_COURSE_UPDATE" || key == "PLANE_SPEED_UPDATE" || key == "PLANE_HEIGHT_UPDATE") {
    const auto prefix = key == "PLANE_COURSE_UPDATE" ? "heading=" : key == "PLANE_SPEED_UPDATE" ? "speed=" : "altitude=";
    const std::string_view text(value);
    const std::string_view name(prefix);
    if (text.substr(0, name.size()) != name) return false;
    const auto number = value.substr(name.size());
    std::size_t used = 0;
    const auto parsed = std::stod(number, &used);
    return used == number.size() && std::isfinite(parsed) && parsed >= 0 && parsed <= 359;
  }
  return false;
}

void reportMail(const CMOOSMsg& mail)
{
  std::cout << "{\"kind\":\"mail\",\"mono\":" << clockSeconds() << ",\"source\":" << quoted(mail.GetSource())
            << ",\"key\":" << quoted(mail.GetKey()) << ",\"published\":" << mail.GetTime() << ",\"value\":";
  if (mail.IsString()) std::cout << quoted(mail.GetString());
  else if (mail.IsDouble() && std::isfinite(mail.GetDouble())) std::cout << mail.GetDouble();
  else std::cout << "null";
  std::cout << "}\n";
}
} // namespace

int main(int argc, char** argv)
{
  CMOOSCommClient client;
  try {
    require(argc == 2, "usage: plane_native_probe DB_PORT");
    const int port = std::stoi(argv[1]);
    require(port > 0 && port <= 65535, "invalid private DB port");
    require(fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK) == 0, "probe stdin setup failed");
    client.SetQuiet(true);
    require(client.Run("127.0.0.1", port, "plane_native_probe", 50), "private MOOS connection startup failed");
    const auto deadline = clockSeconds() + 400;
    bool registered = false;
    unsigned records = 0;
    std::string pending;
    std::cout << std::setprecision(17);
    while (clockSeconds() < deadline) {
      if (registered) require(client.IsConnected(), "private MOOS connection lost");
      if (!registered && client.IsConnected()) {
        for (const auto* key : {"DESIRED_HEADING", "DESIRED_SPEED", "DESIRED_ALTITUDE", "IVPHELM_STATE", "IVPHELM_ALLSTOP",
                                "NAV_X", "NAV_Y", "NAV_ALTITUDE", "MAVMOOS_CAPABILITIES", "MAVMOOS_HELM_STATUS",
                                "MAVMOOS_COMMAND_RESULT", "MAVMOOS_RUNTIME_STATUS", "MAVMOOS_STATUS", "MAVMOOS_PLANE_STATUS",
                                "MAVMOOS_NAV_STATUS", "MAVMOOS_TX_RESULT", "MAVMOOS_AIRSPEED", "MAVMOOS_CONTROL_RESULT",
                                "MAVMOOS_MISSION_RESULT", "MAVMOOS_STORED_MISSION_PROOF", "MAVMOOS_MISSION_EXECUTION_RESULT",
                                "MAVMOOS_MISSION_EXECUTION_LEASE", "MAVMOOS_AUTO_RECOVERY_STATUS",
                                "MAVMOOS_AUTO_RECOVERY_RESULT", "VIEW_MARKER", "MAVMOOS_HELM_OPERATION",
                                "MAVMOOS_OPERATION_RETURN_UPDATE", "MAVMOOS_PLANE_HELM_TASK_STATUS", "MAVMOOS_HELM_TASK_READY",
                                "AUTONOMY_STAGE"})
          require(client.Register(key, 0), "private probe subscription failed");
        registered = true;
        std::cout << "{\"kind\":\"connected\",\"mono\":" << clockSeconds() << "}\n";
      }
      if (registered) {
        char input[512];
        const auto size = read(STDIN_FILENO, input, sizeof(input));
        if (size == 0) {
          client.Close();
          return 0;
        }
        if (size < 0) require(errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR, "private probe control pipe failed");
        else pending.append(input, static_cast<std::size_t>(size));
        std::size_t end = 0;
        while ((end = pending.find('\n')) != std::string::npos) {
          const auto request = pending.substr(0, end);
          pending.erase(0, end + 1);
          const auto tab = request.find('\t');
          require(tab != std::string::npos, "private probe control missing separator");
          const auto key = request.substr(0, tab);
          const auto value = request.substr(tab + 1);
          require(allow(key, value), "private probe command outside fixture policy");
          require(client.Notify(key, value), "private probe command publication failed");
          std::cout << "{\"kind\":\"request\",\"mono\":" << clockSeconds() << ",\"key\":" << quoted(key)
                    << ",\"value\":" << quoted(value) << "}\n";
        }
        require(pending.size() <= 1152, "private probe control line exceeds bound");
        MOOSMSG_LIST mail;
        client.Fetch(mail);
        for (const auto& message : mail) {
          require(++records <= 80000, "private probe record bound exceeded");
          reportMail(message);
        }
      }
      std::cout.flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    throw std::runtime_error("private probe exceeded its total deadline");
  } catch (const std::exception& error) {
    client.Close();
    std::cerr << error.what() << '\n';
    return 1;
  }
}
