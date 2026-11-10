// Test-only generated encoder and loopback sender. Never linked into iMAVLink.
#pragma once
#include "Config.h"
#include <ardupilotmega/mavlink.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <array>
#include <cerrno>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace fixture {
using Bytes = std::vector<uint8_t>;
inline void check(bool condition, const std::string& reason)
{
  if(!condition) throw std::runtime_error(reason);
}
inline std::map<std::string, std::string> fields(const std::string& record)
{
  std::map<std::string, std::string> result;
  std::size_t start = 0;
  while(start < record.size()) {
    const auto end = record.find(',', start);
    const auto part = record.substr(start, end == std::string::npos ? end : end - start);
    const auto equal = part.find('=');
    if(equal != std::string::npos) result[part.substr(0, equal)] = part.substr(equal + 1);
    if(end == std::string::npos) break;
    start = end + 1;
  }
  return result;
}
class Encoder {
public:
  uint8_t system = 1;
  uint8_t component = 1;
  mavlink_status_t status{};
  mavlink_signing_t signing{};
  void useSigning()
  {
    signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
    signing.timestamp = 100000;
    signing.link_id = 7;
    signing.secret_key[0] = 42;
    status.signing = &signing;
  }
  template<class T>
  Bytes encode(const T& value, uint16_t (*encode_fn)(uint32_t, uint8_t, mavlink_status_t*,
                                                   mavlink_message_t*, const T*))
  {
    mavlink_message_t message{};
    encode_fn(system, component, &status, &message, &value);
    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const auto size = mavlink_msg_to_send_buffer(buffer.data(), &message);
    return Bytes(buffer.begin(), buffer.begin() + size);
  }
  Bytes heartbeat(uint8_t type = MAV_TYPE_QUADROTOR)
  {
    mavlink_heartbeat_t value{};
    value.type = type;
    value.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    value.base_mode = MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    value.custom_mode = 4;
    value.system_status = MAV_STATE_ACTIVE;
    return encode(value, mavlink_msg_heartbeat_encode_status);
  }
  Bytes position(int32_t latitude = 0, uint16_t heading = UINT16_MAX)
  {
    mavlink_global_position_int_t value{};
    value.time_boot_ms = 123456;
    value.lat = latitude;
    value.lon = 0;
    value.alt = 123450;
    value.relative_alt = 23450;
    value.vx = 123; value.vy = -456; value.vz = -75;
    value.hdg = heading;
    return encode(value, mavlink_msg_global_position_int_encode_status);
  }
  Bytes landed(uint8_t state = MAV_LANDED_STATE_LANDING)
  {
    mavlink_extended_sys_state_t value{};
    value.landed_state = state;
    return encode(value, mavlink_msg_extended_sys_state_encode_status);
  }
  Bytes home()
  {
    mavlink_home_position_t value{};
    value.latitude = 123456789;
    value.longitude = -1234567890;
    value.altitude = 100000;
    return encode(value, mavlink_msg_home_position_encode_status);
  }
  Bytes estimator(float velocity_variance = 0.1f, float airspeed_variance = 0)
  {
    mavlink_ekf_status_report_t value{};
    value.flags = 31;
    value.velocity_variance = velocity_variance;
    value.pos_horiz_variance = 0.2f; value.pos_vert_variance = 0.3f;
    value.compass_variance = 0.4f; value.terrain_alt_variance = 0.5f;
    value.airspeed_variance = airspeed_variance;
    return encode(value, mavlink_msg_ekf_status_report_encode_status);
  }
  Bytes shortV1Position()
  {
    mavlink_status_t v1{};
    v1.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    mavlink_message_t message{};
    message.msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
    mavlink_finalize_message_buffer(&message, system, component, &v1, 1, 1,
                                    MAVLINK_MSG_ID_GLOBAL_POSITION_INT_CRC);
    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const auto size = mavlink_msg_to_send_buffer(buffer.data(), &message);
    return Bytes(buffer.begin(), buffer.begin() + size);
  }
};
inline mavmoos::RxConfig config()
{
  mavmoos::RxConfig config;
  config.bind_address = config.peer_address = "127.0.0.1";
  config.bind_port = 14600; config.peer_port = 14601;
  config.target_system = config.target_component = 1;
  config.expected_type = MAV_TYPE_QUADROTOR;
  config.expected_autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
  return config;
}
class Socket {
public:
  explicit Socket(uint16_t port = 0)
  {
    m_fd = socket(AF_INET, SOCK_DGRAM, 0);
    check(m_fd >= 0, "fixture socket");
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(port);
    if(bind(m_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
      ::close(m_fd); m_fd = -1; throw std::runtime_error("fixture bind");
    }
    socklen_t size = sizeof(address);
    check(getsockname(m_fd, reinterpret_cast<sockaddr*>(&address), &size) == 0, "getsockname");
    m_port = ntohs(address.sin_port);
    check(fcntl(m_fd, F_SETFL, O_NONBLOCK) == 0, "nonblocking fixture");
  }
  ~Socket() { if(m_fd >= 0) ::close(m_fd); }
  Socket(const Socket&) = delete;
  Socket& operator=(const Socket&) = delete;
  uint16_t port() const { return m_port; }
  void send(const Bytes& bytes, uint16_t target)
  {
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(target);
    check(sendto(m_fd, bytes.data(), bytes.size(), 0,
                 reinterpret_cast<sockaddr*>(&address), sizeof(address)) ==
          static_cast<ssize_t>(bytes.size()), "fixture sendto");
  }
  unsigned drain()
  {
    unsigned count = 0;
    std::array<uint8_t, 65536> bytes{};
    while(recv(m_fd, bytes.data(), bytes.size(), 0) >= 0)
      ++count;
    check(errno == EAGAIN || errno == EWOULDBLOCK, "fixture receive error");
    return count;
  }
private:
  int m_fd = -1;
  uint16_t m_port = 0;
};
inline uint16_t unusedUdpPort()
{
  Socket reservation;
  return reservation.port();
}
} // namespace fixture
