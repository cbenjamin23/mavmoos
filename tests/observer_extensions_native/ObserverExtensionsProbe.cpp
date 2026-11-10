// Read-only MOOS probe for grouped, disarmed Copter observer evidence.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

int main(int argc, char** argv)
{
  CMOOSCommClient client;
  try {
    if(argc != 3)
      throw std::invalid_argument("usage: observer_extensions_native_probe DB_PORT SECONDS");
    const int port = std::stoi(argv[1]);
    const int duration = std::stoi(argv[2]);
    if(port < 1 || port > 65535 || duration < 1 || duration > 30)
      throw std::invalid_argument("probe port or duration outside fixture bounds");
    client.SetQuiet(true);
    if(!client.Run("127.0.0.1", port, "observer_extensions_native", 50))
      throw std::runtime_error("MOOS connection failed");

    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(duration);
    bool registered = false;
    unsigned captured = 0;
    while(std::chrono::steady_clock::now() < end) {
      if(!client.IsConnected()) registered = false;
      else if(!registered) {
        for(const auto* key : {"MAVMOOS_CAPABILITIES", "MAVMOOS_STATUS", "MAVMOOS_RUNTIME_STATUS",
            "MAVMOOS_OPTICAL_FLOW", "MAVMOOS_OPTICAL_FLOW_RAD", "MAVMOOS_TERRAIN_REPORT",
            "MAVMOOS_TERRAIN_REQUEST_OBSERVED", "MAVMOOS_NAV_CONTROLLER_OUTPUT",
            "MAVMOOS_POSITION_TARGET_LOCAL_NED", "MAVMOOS_POSITION_TARGET_GLOBAL_INT"})
          client.Register(key, 0);
        registered = true;
      }
      MOOSMSG_LIST mail;
      client.Fetch(mail);
      for(const auto& message : mail) {
        if(message.GetSource() != "iMAVLink" || !message.IsString()) continue;
        if(++captured > 10000) throw std::runtime_error("MOOS capture exceeded fixture bound");
        std::cout << "CAPTURE\t" << message.GetKey() << '\t' << message.GetString() << '\n';
      }
      std::cout.flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    client.Close();
    return captured ? 0 : 1;
  } catch(const std::exception& error) {
    client.Close();
    std::cerr << error.what() << '\n';
    return 1;
  }
}
