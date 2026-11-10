/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: Config.cpp                                      */
/*    DATE: October 29, 2026                                */
/************************************************************/
#include "Config.h"
#include <ardupilotmega/mavlink.h>
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <locale>
#include <random>
#include <sstream>

namespace mavmoos {

bool parseUnsigned(const std::string& text, unsigned low, unsigned high, unsigned& out)
{
  if(text.empty())
    return false;
  unsigned value = 0;
  for(char c : text) {
    if(c < '0' || c > '9' || value > high / 10)
      return false;
    value = value * 10 + static_cast<unsigned>(c - '0');
    if(value > high)
      return false;
  }
  if(value < low)
    return false;
  out = value;
  return true;
}

bool parseSeconds(const std::string& text, double low, double high, double& out)
{
  std::istringstream input(text);
  input.imbue(std::locale::classic());
  double value = 0;
  input >> std::noskipws >> value;
  if(!input || !input.eof() || !std::isfinite(value) || value < low || value > high)
    return false;
  out = value;
  return true;
}

bool isIpv4(const std::string& text)
{
  in_addr address{};
  return inet_pton(AF_INET, text.c_str(), &address) == 1;
}

std::vector<std::string> validateConfig(const RxConfig& c)
{
  std::vector<std::string> errors;
  if(c.simulator_profile!="arducopter" && c.simulator_profile!="ardurover" && c.simulator_profile!="arduplane" && c.simulator_profile!="px4" &&
     c.simulator_profile!="ardusub")
    errors.emplace_back("unsupported_simulator_profile");
  if(!isIpv4(c.bind_address)) errors.emplace_back("bind_address_required_ipv4");
  if(!isIpv4(c.peer_address) || c.peer_address == "0.0.0.0")
    errors.emplace_back("peer_address_required_ipv4_unicast");
  else {
    in_addr address{};
    inet_pton(AF_INET, c.peer_address.c_str(), &address);
    const auto ip = ntohl(address.s_addr);
    if((ip >> 24) == 0 || (ip >> 24) >= 224)
      errors.emplace_back("peer_address_required_ipv4_unicast");
  }
  if(c.bind_port == 0) errors.emplace_back("bind_port_required");
  if(c.peer_port == 0) errors.emplace_back("peer_port_required");
  if(c.bind_address == c.peer_address && c.bind_port == c.peer_port)
    errors.emplace_back("bind_and_peer_must_differ");
  if(c.target_system == 0) errors.emplace_back("target_system_required");
  if(c.target_component == 0) errors.emplace_back("target_component_required");
  const bool loopback_1_1 = c.bind_address == "127.0.0.1" && c.peer_address == "127.0.0.1" &&
    c.target_system == 1 && c.target_component == 1;
  if(c.landing_target_component != 0 &&
     (!loopback_1_1 || c.expected_type != MAV_TYPE_QUADROTOR || c.expected_autopilot != MAV_AUTOPILOT_ARDUPILOTMEGA))
    errors.emplace_back("landing_target_requires_loopback_arducopter_1_1");
  if(c.camera_component || c.gimbal_component) {
    if(!loopback_1_1 || (c.expected_autopilot != MAV_AUTOPILOT_ARDUPILOTMEGA && c.expected_autopilot != MAV_AUTOPILOT_PX4))
      errors.emplace_back("payload_components_require_loopback_ardupilot_or_px4_1_1");
    if(c.camera_component == 1 || c.gimbal_component == 1)
      errors.emplace_back("payload_component_ids_must_be_2_to_255");
    if((c.camera_component && (c.camera_component == c.gimbal_component || c.camera_component == c.landing_target_component)) ||
       (c.gimbal_component && c.gimbal_component == c.landing_target_component))
      errors.emplace_back("payload_component_ids_must_be_distinct");
  }
  if((c.remote_id_component || c.component_metadata_component) &&
     (!loopback_1_1 || (c.expected_autopilot!=MAV_AUTOPILOT_ARDUPILOTMEGA && c.expected_autopilot!=MAV_AUTOPILOT_PX4)))
    errors.emplace_back("selected_observer_components_require_loopback_ardupilot_or_px4_1_1");
  if(c.expected_type < 0 || c.expected_type > 255)
    errors.emplace_back("expected_type_required");
  if(c.expected_autopilot < 0 || c.expected_autopilot > 255)
    errors.emplace_back("expected_autopilot_required");
  if(c.max_datagrams_per_iter < 1 || c.max_datagrams_per_iter > 128)
    errors.emplace_back("invalid_receive_budget");
  if(!std::isfinite(c.max_queue_age) || c.max_queue_age < 0.01 || c.max_queue_age > 2)
    errors.emplace_back("invalid_max_queue_age");
  for(double t : {c.heartbeat_timeout, c.position_timeout, c.landed_timeout,
                  c.estimator_timeout, c.publish_interval}) {
    if(!std::isfinite(t) || t < 0.01 || t > 60) {
      errors.emplace_back("invalid_time_budget");
      break;
    }
  }
  return errors;
}

double monotonicSeconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string newSessionId()
{
  std::random_device random;
  std::ostringstream out;
  out << std::hex << random() << random();
  return out.str();
}
} // namespace mavmoos
