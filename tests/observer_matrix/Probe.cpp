// Subscribe to the actual app's observations without issuing MOOS requests.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"

#include <chrono>
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

int main(int argc, char** argv)
{
  CMOOSCommClient client;
  try {
    bool landing_target = false, traffic = false, diagnostics = false;
    std::vector<std::string> watched;
    for (int i = 3; i < argc; ++i) {
      const std::string option = argv[i];
      if (option == "--landing-target") landing_target = true;
      else if (option == "--traffic") traffic = true;
      else if (option == "--status-diagnostics") diagnostics = true;
      else if (option.rfind("--watch=", 0) == 0) {
        const auto key = option.substr(8);
        if (key.size() <= 8 || key.size() > 80 || key.rfind("MAVMOOS_", 0) != 0 || watched.size() >= 32 ||
            !std::all_of(key.begin(),key.end(),[](char c){return (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_';}) ||
            std::find(watched.begin(),watched.end(),key) != watched.end())
          throw std::invalid_argument("invalid or repeated observation key");
        watched.push_back(key);
      }
      else throw std::invalid_argument("unknown observer probe option " + option);
    }
    if (argc < 3)
      throw std::invalid_argument("usage: observer_matrix_probe DB_PORT SECONDS [--landing-target] [--traffic] [--status-diagnostics] [--watch=MAVMOOS_KEY]");
    const int port = std::stoi(argv[1]);
    const int duration = std::stoi(argv[2]);
    if (port < 1 || port > 65535 || duration < 1 || duration > 30)
      throw std::invalid_argument("probe port/duration outside fixture bounds");
    client.SetQuiet(true);
    if (!client.Run("127.0.0.1", port, "observer_matrix_probe", 50))
      throw std::runtime_error("MOOS startup failed");

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(duration);
    bool registered = false;
    unsigned records = 0;
    while (std::chrono::steady_clock::now() < deadline) {
      if (!client.IsConnected()) registered = false;
      else if (!registered) {
        for (const auto* key : {"MAVMOOS_CAPABILITIES", "MAVMOOS_RUNTIME_STATUS", "MAVMOOS_STATUS",
                                "MAVMOOS_LOCAL_KINEMATICS", "MAVMOOS_ODOMETRY", "MAVMOOS_VFR_HUD",
                                "MAVMOOS_WIND", "MAVMOOS_WIND_COV", "MAVMOOS_ESC_INFO",
                                "MAVMOOS_ESC_STATUS", "MAVMOOS_ESC_TELEMETRY", "MAVMOOS_LOCAL_ORIGIN",
                                "MAVMOOS_SCALED_PRESSURE", "MAVMOOS_IMU_VIBRATION"})
          client.Register(key, 0);
        if (landing_target) client.Register("MAVMOOS_LANDING_TARGET", 0);
        if (traffic) client.Register("MAVMOOS_TRAFFIC_CONTACTS", 0);
        for (const auto& key : watched) client.Register(key, 0);
        if (diagnostics) {
          for (const auto* key : {"MAVMOOS_POWER_STATUS", "MAVMOOS_MCU_STATUS",
                                  "MAVMOOS_EKF_STATUS_REPORT", "MAVMOOS_ESTIMATOR_STATUS",
                                  "MAVMOOS_SYSTEM_STATUS", "MAVMOOS_SYS_STATUS"})
            client.Register(key, 0);
        }
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
