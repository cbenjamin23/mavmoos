// Operator relay for the exact owned, disarmed camera and gimbal fixture.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"

#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

namespace {
double now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void need(bool condition, const char* reason)
{
  if (!condition) throw std::runtime_error(reason);
}

void record(const char* kind, const std::string& key, const std::string& value)
{
  need(value.find_first_of("\t\r\n") == std::string::npos, "payload probe record is not one field");
  std::cout << kind << '\t' << key << '\t' << now() << '\t' << value << '\n';
}
}

int main(int argc, char** argv)
{
  CMOOSCommClient client;
  try {
    need(argc == 2, "usage: payload_operations_moos_probe DB_PORT");
    std::size_t used = 0;
    const std::string input = argv[1];
    const auto port = std::stoi(input, &used);
    need(used == input.size() && port > 0 && port <= 65535, "invalid private DB port");
    need(fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK) == 0, "probe stdin setup failed");
    client.SetQuiet(true);
    need(client.Run("127.0.0.1", port, "payload_operations_probe", 50), "private MOOS connection failed");
    std::cout << std::setprecision(17);
    const auto deadline = now() + 60;
    bool registered = false;
    std::string pending;
    while (now() < deadline) {
      if (registered) need(client.IsConnected(), "private MOOS connection lost");
      if (!registered && client.IsConnected()) {
        for (const auto* key : {"MAVMOOS_CAPABILITIES", "MAVMOOS_RUNTIME_STATUS", "MAVMOOS_STATUS",
                               "MAVMOOS_CAMERA_RESULT", "MAVMOOS_CAMERA_CAPTURE_STATUS", "MAVMOOS_CAMERA_INFORMATION",
                               "MAVMOOS_GIMBAL_RESULT", "MAVMOOS_GIMBAL_MANAGER_STATUS",
                               "MAVMOOS_GIMBAL_DEVICE_ATTITUDE_STATUS"})
          need(client.Register(key, 0), "payload probe registration failed");
        registered = true;
      }
      if (registered) {
        char input_buffer[512];
        const auto size = read(STDIN_FILENO, input_buffer, sizeof(input_buffer));
        if (size == 0) { client.Close(); return 0; }
        if (size < 0) need(errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR, "payload probe pipe failed");
        else pending.append(input_buffer, static_cast<std::size_t>(size));
        for (auto end = pending.find('\n'); end != std::string::npos; end = pending.find('\n')) {
          const auto request = pending.substr(0, end);
          pending.erase(0, end + 1);
          const auto tab = request.find('\t');
          need(tab != std::string::npos && end <= 256, "invalid payload probe request");
          const auto key = request.substr(0, tab), value = request.substr(tab + 1);
          need((key == "MAVMOOS_CAMERA_REQUEST" || key == "MAVMOOS_GIMBAL_REQUEST") &&
               value.compare(0, 9, "schema=1,") == 0, "payload probe request outside fixture policy");
          need(client.Notify(key, value), "payload probe publication failed");
          record("request", key, value);
        }
        need(pending.size() <= 256, "payload probe request exceeds bound");
        MOOSMSG_LIST mail;
        client.Fetch(mail);
        for (const auto& message : mail)
          if (message.GetSource() == "iMAVLink" && message.IsString())
            record("app", message.GetKey(), message.GetString());
      }
      std::cout.flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    throw std::runtime_error("payload probe deadline elapsed");
  } catch (const std::exception& error) {
    client.Close();
    std::cerr << error.what() << '\n';
    return 1;
  }
}
