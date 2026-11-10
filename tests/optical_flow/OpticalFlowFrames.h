#pragma once

#include <ardupilotmega/mavlink.h>

#include <limits>

namespace mavmoos::test::optical_flow_fixture {

inline mavlink_optical_flow_t flowPayload(uint8_t sensor = 7)
{
  mavlink_optical_flow_t value{};
  value.time_usec = 9007199254741001ULL;
  value.flow_x = 21;
  value.flow_y = -17;
  value.flow_comp_m_x = .75f;
  value.flow_comp_m_y = -1.25f;
  value.ground_distance = -1;
  value.sensor_id = sensor;
  value.quality = 127;
  value.flow_rate_x = .125f;
  value.flow_rate_y = -.25f;
  return value;
}

inline mavlink_optical_flow_rad_t radPayload(uint8_t sensor = 7)
{
  mavlink_optical_flow_rad_t value{};
  value.time_usec = 9007199254741003ULL;
  value.integration_time_us = 30000;
  value.integrated_x = .25f;
  value.integrated_y = -.125f;
  value.integrated_xgyro = .03125f;
  value.integrated_ygyro = std::numeric_limits<float>::quiet_NaN();
  value.integrated_zgyro = -.0625f;
  value.time_delta_distance_us = 2000;
  value.distance = 0;
  value.temperature = -180;
  value.sensor_id = sensor;
  value.quality = 240;
  return value;
}

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_optical_flow_t& value,
                                uint8_t system = 1, uint8_t component = 1)
{
  mavlink_message_t message{};
  mavlink_msg_optical_flow_encode_status(system, component, &codec, &message, &value);
  return message;
}

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_optical_flow_rad_t& value,
                                uint8_t system = 1, uint8_t component = 1)
{
  mavlink_message_t message{};
  mavlink_msg_optical_flow_rad_encode_status(system, component, &codec, &message, &value);
  return message;
}

inline mavlink_message_t flow(mavlink_status_t& codec, uint8_t system = 1, uint8_t component = 1)
{
  return encode(codec, flowPayload(), system, component);
}

inline mavlink_message_t trimmedFlow(mavlink_status_t& codec)
{
  auto value = flowPayload(201);
  value.ground_distance = 0;
  value.quality = 0;
  value.flow_rate_y = 0;
  return encode(codec, value);
}

inline mavlink_message_t rad(mavlink_status_t& codec, uint8_t system = 1, uint8_t component = 1)
{
  return encode(codec, radPayload(), system, component);
}

} // namespace mavmoos::test::optical_flow_fixture
