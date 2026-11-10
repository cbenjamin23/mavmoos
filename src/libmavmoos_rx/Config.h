/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: Config.h                                        */
/*    DATE: October 29, 2026                                */
/************************************************************/
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace mavmoos {

struct RxConfig {
  std::string bind_address;
  std::string peer_address;
  uint16_t bind_port = 0;
  uint16_t peer_port = 0;
  uint8_t target_system = 0;
  uint8_t target_component = 0;
  // Explicitly configured identity; zero is not used as a wildcard.
  int expected_type = -1;
  int expected_autopilot = -1;
  unsigned max_datagrams_per_iter = 32;
  double max_queue_age = 0.25;
  double heartbeat_timeout = 2.0;
  double position_timeout = 0.5;
  double landed_timeout = 2.0;
  double estimator_timeout = 3.0;
  double publish_interval = 0.2;
  std::string simulator_profile="arducopter";
  uint8_t landing_target_component = 0;
  uint8_t camera_component = 0;
  uint8_t gimbal_component = 0;
  uint8_t remote_id_component = 0;
  uint8_t component_metadata_component = 0;
};

// Helpers validate whole tokens; invalid updates never change the destination.
bool parseUnsigned(const std::string& text, unsigned low, unsigned high, unsigned& out);
bool parseSeconds(const std::string& text, double low, double high, double& out);
bool isIpv4(const std::string& text);
std::vector<std::string> validateConfig(const RxConfig& config);
double monotonicSeconds();
std::string newSessionId();

} // namespace mavmoos
