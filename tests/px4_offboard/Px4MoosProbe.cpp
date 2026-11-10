// Stock MOOS operator and receive-only observer of the separate PX4 GCS link.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include <ardupilotmega/mavlink.h>
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace {
constexpr uint16_t observer_port = 14550;
constexpr uint16_t gcs_source_port = 18570;

double now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void require(bool ok, const char* why)
{
  if (!ok)
    throw std::runtime_error(why);
}

std::string quoted(const std::string& text)
{
  static constexpr char hex[] = "0123456789abcdef";
  std::string result = "\"";
  for (const unsigned char c : text) {
    if (c == '"' || c == '\\')
      result += '\\';
    if (c < 32 || c >= 127) {
      result += "\\u00";
      result += hex[c >> 4];
      result += hex[c & 15];
    } else {
      result += static_cast<char>(c);
    }
  }
  return result + '"';
}

std::ostringstream traceRow(const char* kind)
{
  std::ostringstream row;
  row << std::setprecision(17) << "{\"kind\":" << quoted(kind) << ",\"mono\":" << now();
  return row;
}

void emit(std::ostringstream& row)
{
  // The MOOS connection thread can print diagnostics concurrently. A single
  // write with separators keeps each structured record intact and distinct.
  const auto line = '\n' + row.str() + "}\n";
  ssize_t size;
  do { size = write(STDOUT_FILENO, line.data(), line.size()); } while (size < 0 && errno == EINTR);
  require(size == static_cast<ssize_t>(line.size()), "structured trace write failed");
}

struct GcsObserver {
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  mavlink_message_t parser{};
  mavlink_status_t status{};
  std::ofstream capture;
  unsigned datagrams = 0;
  const bool fixed_wing;
  const bool rover;
  const bool aircraft;

  GcsObserver(const char* path, bool fixed_wing_fixture, bool rover_fixture, bool aircraft_fixture)
    : capture(path, std::ios::binary), fixed_wing(fixed_wing_fixture || aircraft_fixture),
      rover(rover_fixture), aircraft(aircraft_fixture)
  {
    require(fd >= 0 && capture.is_open(), "GCS observer socket/capture unavailable");
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(observer_port);
    local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    require(bind(fd, reinterpret_cast<const sockaddr*>(&local), sizeof(local)) == 0,
            "independent GCS loopback bind failed");
  }

  ~GcsObserver() { if (fd >= 0) close(fd); }

  void log(const mavlink_message_t& message) const
  {
    if (message.sysid != 1 || message.compid != 1 || message.magic != MAVLINK_STX)
      return;
    auto row = traceRow("wire");
    const auto prefix = [&] (const char* kind) { row << ",\"message\":" << quoted(kind); };
    if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
      mavlink_heartbeat_t value{};
      mavlink_msg_heartbeat_decode(&message, &value);
      prefix("HEARTBEAT");
      row << ",\"mav_type\":" << unsigned(value.type) << ",\"autopilot\":" << unsigned(value.autopilot)
        << ",\"mode\":" << value.custom_mode << ",\"armed\":"
        << ((value.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? "true" : "false");
    } else if (message.msgid == MAVLINK_MSG_ID_CURRENT_MODE) {
      mavlink_current_mode_t value{};
      mavlink_msg_current_mode_decode(&message, &value);
      prefix("CURRENT_MODE");
      row << ",\"mode\":" << value.custom_mode << ",\"intended_mode\":" << value.intended_custom_mode;
    } else if (message.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
      mavlink_local_position_ned_t value{};
      mavlink_msg_local_position_ned_decode(&message, &value);
      if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z) ||
          !std::isfinite(value.vx) || !std::isfinite(value.vy) || !std::isfinite(value.vz))
        return;
      prefix("LOCAL_POSITION_NED");
      row << ",\"north\":" << value.x << ",\"east\":" << value.y << ",\"down\":" << value.z
        << ",\"vx\":" << value.vx << ",\"vy\":" << value.vy << ",\"vz\":" << value.vz;
      if (rover)
        row << ",\"boot_ms\":" << value.time_boot_ms;
    } else if (message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
      mavlink_global_position_int_t value{};
      mavlink_msg_global_position_int_decode(&message, &value);
      prefix("GLOBAL_POSITION_INT");
      row << ",\"latitude_e7\":" << value.lat << ",\"longitude_e7\":" << value.lon
        << ",\"relative_m\":" << value.relative_alt / 1000.;
      if (rover)
        row << ",\"heading_deg\":" << value.hdg / 100.
          << ",\"vx\":" << value.vx / 100. << ",\"vy\":" << value.vy / 100.
          << ",\"boot_ms\":" << value.time_boot_ms;
    } else if (message.msgid == MAVLINK_MSG_ID_EXTENDED_SYS_STATE) {
      mavlink_extended_sys_state_t value{};
      mavlink_msg_extended_sys_state_decode(&message, &value);
      prefix("EXTENDED_SYS_STATE");
      row << ",\"landed_state\":" << unsigned(value.landed_state) << ",\"vtol_state\":" << unsigned(value.vtol_state);
    } else if (fixed_wing && message.msgid == MAVLINK_MSG_ID_VFR_HUD) {
      mavlink_vfr_hud_t value{};
      mavlink_msg_vfr_hud_decode(&message, &value);
      prefix("VFR_HUD");
      row << ",\"airspeed\":";
      if (std::isfinite(value.airspeed)) row << value.airspeed;
      else row << "null";
    } else if (rover && message.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
      mavlink_gps_raw_int_t value{};
      mavlink_msg_gps_raw_int_decode(&message, &value);
      prefix("GPS_RAW_INT");
      row << ",\"sensor_usec\":" << value.time_usec << ",\"speed_cmps\":" << value.vel
        << ",\"velocity_accuracy_mmps\":" << value.vel_acc << ",\"horizontal_accuracy_mm\":" << value.h_acc
        << ",\"fix_type\":" << unsigned(value.fix_type) << ",\"satellites\":" << unsigned(value.satellites_visible)
        << ",\"latitude_e7\":" << value.lat << ",\"longitude_e7\":" << value.lon;
    } else if ((rover || aircraft) && message.msgid == MAVLINK_MSG_ID_HOME_POSITION) {
      mavlink_home_position_t value{};
      mavlink_msg_home_position_decode(&message, &value);
      prefix("HOME_POSITION");
      row << ",\"latitude_e7\":" << value.latitude << ",\"longitude_e7\":" << value.longitude
        << ",\"altitude_msl_m\":" << value.altitude / 1000.;
    } else if (rover && message.msgid == MAVLINK_MSG_ID_MISSION_CURRENT) {
      mavlink_mission_current_t value{};
      mavlink_msg_mission_current_decode(&message, &value);
      prefix("MISSION_CURRENT");
      row << ",\"sequence\":" << value.seq << ",\"total\":" << value.total
        << ",\"state\":" << unsigned(value.mission_state) << ",\"mission_id\":" << value.mission_id;
    } else if (rover && message.msgid == MAVLINK_MSG_ID_MISSION_ITEM_REACHED) {
      mavlink_mission_item_reached_t value{};
      mavlink_msg_mission_item_reached_decode(&message, &value);
      prefix("MISSION_ITEM_REACHED");
      row << ",\"sequence\":" << value.seq;
    } else if (rover && message.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
      mavlink_sys_status_t value{};
      mavlink_msg_sys_status_decode(&message, &value);
      prefix("SYS_STATUS");
      row << ",\"prearm_healthy\":"
        << ((value.onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK) ? "true" : "false");
    } else {
      return;
    }
    emit(row);
  }

  void drain()
  {
    for (unsigned i = 0; i < 128; ++i) {
      unsigned char data[4096];
      sockaddr_in from{};
      socklen_t address_length = sizeof(from);
      const auto size = recvfrom(fd, data, sizeof(data), MSG_DONTWAIT,
                                reinterpret_cast<sockaddr*>(&from), &address_length);
      if (size < 0) {
        require(errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR, "GCS receive failed");
        return;
      }
      require(from.sin_addr.s_addr == htonl(INADDR_LOOPBACK) && ntohs(from.sin_port) == gcs_source_port,
              "GCS observer received an unexpected endpoint");
      require(++datagrams <= 60000, "GCS capture exceeded the fixture bound");
      capture.write(reinterpret_cast<const char*>(data), size);
      require(capture.good(), "GCS trace write failed");
      for (ssize_t index = 0; index < size; ++index) {
        mavlink_message_t received{};
        mavlink_status_t received_status{};
        if (mavlink_frame_char_buffer(&parser, &status, data[index], &received, &received_status) == MAVLINK_FRAMING_OK)
          log(received);
      }
    }
  }
};

void reportMail(const CMOOSMsg& mail)
{
  auto row = traceRow("mail");
  row << ",\"source\":" << quoted(mail.GetSource())
    << ",\"key\":" << quoted(mail.GetKey()) << ",\"published\":" << mail.GetTime() << ",\"value\":";
  if (mail.IsString()) row << quoted(mail.GetString());
  else if (mail.IsDouble() && std::isfinite(mail.GetDouble())) row << mail.GetDouble();
  else row << "null";
  emit(row);
}

bool allowed(const std::string& key, const std::string& value, bool rover, bool aircraft)
{
  return (key == "DEPLOY" && (value == "true" || value == "false")) ||
    (key == "MAVMOOS_NAV_CONTROL" && (value == "acquire" || value == "release")) ||
    (aircraft && key == "AIRCRAFT_LEG" && (value == "hover" || value == "forward" || value == "final")) ||
    ((aircraft || rover) && key == "STRESS_ROUTE_UPDATE" &&
     value.size() <= 128 && value.find("points=") == 0) ||
    (aircraft && key == "MAVMOOS_COMMAND" && value.size() <= 180 && value.find("id=") == 0) ||
    (rover && key == "ROVER_LEG" && (value == "straight" || value == "turn")) ||
    (rover && value.size() <= 1024 &&
     ((key == "MAVMOOS_MISSION_REQUEST" && value.find("id=") == 0) ||
      ((key == "MAVMOOS_MISSION_CONTROL" || key == "MAVMOOS_AUTO_RECOVERY_REQUEST") &&
       value.find("schema=1,") == 0)));
}
}

int main(int argc, char** argv)
{
  CMOOSCommClient client;
  try {
    const bool fixed_wing = argc == 4 && std::string(argv[3]) == "--fixed-wing";
    const bool rover = argc == 4 && std::string(argv[3]) == "--rover";
    const bool aircraft = argc == 4 && std::string(argv[3]) == "--aircraft";
    require(argc == 3 || fixed_wing || rover || aircraft,
            "usage: px4_offboard_moos_probe DB_PORT RAW_GCS_CAPTURE [--fixed-wing|--rover|--aircraft]");
    const int port = std::stoi(argv[1]);
    require(port > 0 && port <= 65535, "invalid private MOOS port");
    require(fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK) == 0, "operator pipe setup failed");
    GcsObserver observer(argv[2], fixed_wing, rover, aircraft);
    auto listening = traceRow("listening");
    emit(listening);
    client.SetQuiet(true);
    require(client.Run("127.0.0.1", port, "px4_sitl_operator", 50), "MOOS connection startup failed");
    bool registered = false;
    std::string pending;
    unsigned records = 0;
    const double deadline = now() + (aircraft ? 280 : rover ? 230 : fixed_wing ? 205 : 115);
    while (now() < deadline) {
      observer.drain();
      if (registered)
        require(client.IsConnected(), "private MOOS connection lost");
      if (!registered && client.IsConnected()) {
        for (const auto* key : {"DESIRED_HEADING", "DESIRED_SPEED", "DESIRED_ALTITUDE", "IVPHELM_STATE",
                                "IVPHELM_ALLSTOP", "NAV_X", "NAV_Y", "NAV_ALTITUDE", "MAVMOOS_CAPABILITIES",
                                "MAVMOOS_HELM_STATUS", "MAVMOOS_RUNTIME_STATUS",
                                "MAVMOOS_TX_RESULT"})
          require(client.Register(key, 0), "MOOS subscription failed");
        if (aircraft)
          for (const auto* key : {"MAVMOOS_COMMAND_RESULT", "MAVMOOS_PX4_VTOL_PHASE", "AUTONOMY_INDEX"})
            require(client.Register(key, 0), "aircraft MOOS subscription failed");
        if (rover)
          for (const auto* key : {"MAVMOOS_PX4_ROVER_STATUS", "MAVMOOS_MISSION_RESULT", "ROVER_LEG", "AUTONOMY_INDEX",
                                  "MAVMOOS_MISSION_SNAPSHOT", "MAVMOOS_STORED_MISSION_PROOF",
                                  "MAVMOOS_MISSION_EXECUTION_LEASE", "MAVMOOS_MISSION_EXECUTION_RESULT",
                                  "MAVMOOS_MISSION_EXECUTION_STATUS", "MAVMOOS_MISSION_EXECUTION_PROGRESS",
                                  "MAVMOOS_AUTO_RECOVERY_STATUS", "MAVMOOS_AUTO_RECOVERY_RESULT"})
            require(client.Register(key, 0), "Rover MOOS subscription failed");
        registered = true;
        auto connected = traceRow("connected");
        emit(connected);
      }
      if (registered) {
        char data[128];
        const auto size = read(STDIN_FILENO, data, sizeof(data));
        if (size == 0) {
          client.Close();
          return 0;
        }
        if (size < 0) require(errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR, "operator pipe read failed");
        else pending.append(data, static_cast<std::size_t>(size));
        for (auto end = pending.find('\n'); end != std::string::npos; end = pending.find('\n')) {
          const auto input = pending.substr(0, end);
          pending.erase(0, end + 1);
          const auto tab = input.find('\t');
          require(tab != std::string::npos, "invalid operator command");
          const auto key = input.substr(0, tab);
          const auto value = input.substr(tab + 1);
          require(allowed(key, value, rover, aircraft) && client.Notify(key, value), "operator command denied");
          auto request = traceRow("request");
          request << ",\"key\":" << quoted(key) << ",\"value\":" << quoted(value);
          emit(request);
        }
        require(pending.size() <= (rover ? 1100 : aircraft ? 220 : 128), "operator command length exceeded");
        MOOSMSG_LIST mail;
        client.Fetch(mail);
        for (const auto& message : mail) {
          require(++records <= (aircraft ? 98000 : rover ? 85000 : 40000), "MOOS fixture exceeded its record bound");
          reportMail(message);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    throw std::runtime_error("bounded PX4 observer expired");
  } catch (const std::exception& error) {
    client.Close();
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
