// Subscribe to the actual app's observations without issuing MOOS requests.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

int main(int argc, char** argv)
{
  CMOOSCommClient client;
  try {
    if (argc != 3)
      throw std::invalid_argument("usage: gimbal_status_moos_probe DB_PORT SECONDS");
    const int port = std::stoi(argv[1]);
    const int duration = std::stoi(argv[2]);
    if (port < 1 || port > 65535 || duration < 1 || duration > 30)
      throw std::invalid_argument("probe port/duration outside fixture bounds");
    client.SetQuiet(true);
    if (!client.Run("127.0.0.1", port, "gimbal_status_moos_probe", 50))
      throw std::runtime_error("MOOS startup failed");

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(duration);
    bool registered = false;
    unsigned records = 0;
    while (std::chrono::steady_clock::now() < deadline) {
      if (!client.IsConnected()) registered = false;
      else if (!registered) {
        for (const auto* key : {"MAVMOOS_CAPABILITIES", "MAVMOOS_STATUS", "MAVMOOS_RUNTIME_STATUS",
                                "MAVMOOS_GIMBAL_DEVICE_INFORMATION", "MAVMOOS_GIMBAL_DEVICE_ATTITUDE_STATUS",
                                "MAVMOOS_CAMERA_INFORMATION", "MAVMOOS_CAMERA_SETTINGS",
                                "MAVMOOS_CAMERA_CAPTURE_STATUS", "MAVMOOS_CAMERA_STORAGE"})
          client.Register(key, 0);
        registered = true;
      }
      MOOSMSG_LIST mail;
      client.Fetch(mail);
      for (const auto& message : mail) {
        if (!message.IsString() || message.GetSource() != "iMAVLink") continue;
        if (++records > 10000) throw std::runtime_error("probe record bound exceeded");
        std::cout << "CAPTURE\t" << message.GetKey() << '\t' << message.GetString() << '\n';
      }
      std::cout.flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    client.Close();
    return records ? 0 : 1;
  } catch (const std::exception& error) {
    client.Close();
    std::cerr << error.what() << '\n';
    return 1;
  }
}
