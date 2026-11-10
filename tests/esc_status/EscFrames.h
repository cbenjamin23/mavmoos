#pragma once
#include <ardupilotmega/mavlink.h>
#include <limits>

namespace mavmoos::test::esc {

inline mavlink_message_t info(mavlink_status_t& codec, uint8_t first_index = 4,
                              uint8_t count = 6, uint8_t system = 1, uint8_t component = 1)
{
  mavlink_esc_info_t report{};
  report.index = first_index;
  report.count = count;
  report.time_usec = 123456789;
  report.counter = 21;
  report.connection_type = 1;
  report.info = 0x9b;
  report.error_count[0] = 11;
  report.error_count[1] = 12;
  report.error_count[2] = 13;
  report.error_count[3] = 14;
  report.failure_flags[1] = 0x4000;
  report.temperature[0] = 3125;
  report.temperature[1] = std::numeric_limits<int16_t>::max();
  report.temperature[2] = -425;
  mavlink_message_t message{};
  mavlink_msg_esc_info_encode_status(system, component, &codec, &message, &report);
  return message;
}

inline mavlink_message_t status(mavlink_status_t& codec, uint8_t first_index = 0,
                                uint8_t system = 1, uint8_t component = 1)
{
  mavlink_esc_status_t report{};
  report.index = first_index;
  report.time_usec = 123456790;
  report.rpm[0] = 1700;
  report.rpm[1] = -1200;
  report.rpm[3] = 450;
  report.voltage[0] = 16.75f;
  report.voltage[1] = 15.5f;
  report.voltage[2] = std::numeric_limits<float>::quiet_NaN();
  report.current[0] = .75f;
  report.current[1] = -.25f;
  report.current[2] = std::numeric_limits<float>::infinity();
  mavlink_message_t message{};
  mavlink_msg_esc_status_encode_status(system, component, &codec, &message, &report);
  return message;
}

inline mavlink_message_t telemetry(mavlink_status_t& codec, uint8_t system = 1,
                                   uint8_t component = 1, uint16_t packet_count = 65535)
{
  mavlink_esc_telemetry_1_to_4_t report{};
  report.voltage[0] = 1680;
  report.current[0] = 80;
  report.totalcurrent[0] = 1;
  report.rpm[0] = 350;
  report.count[0] = packet_count;
  report.temperature[0] = 32;
  mavlink_message_t message{};
  mavlink_msg_esc_telemetry_1_to_4_encode_status(system, component, &codec, &message, &report);
  return message;
}

} // namespace mavmoos::test::esc
