#pragma once
#include <ardupilotmega/mavlink.h>
#include <limits>

namespace mavmoos::test::air_data {

inline mavlink_message_t vfrHud(mavlink_status_t& codec, uint8_t system = 1, uint8_t component = 1)
{
  mavlink_vfr_hud_t value{};
  value.airspeed = 5.5f;
  value.groundspeed = 7.25f;
  value.alt = 132.5f;
  value.climb = -1.25f;
  value.heading = 278;
  value.throttle = 42;
  mavlink_message_t message{};
  mavlink_msg_vfr_hud_encode_status(system, component, &codec, &message, &value);
  return message;
}

inline mavlink_message_t wind(mavlink_status_t& codec, uint8_t system = 1, uint8_t component = 1)
{
  mavlink_wind_t value{};
  value.direction = -135.5f;
  value.speed = 8.25f;
  value.speed_z = -2.5f;
  mavlink_message_t message{};
  mavlink_msg_wind_encode_status(system, component, &codec, &message, &value);
  return message;
}

inline mavlink_message_t windCov(mavlink_status_t& codec, uint8_t system = 1, uint8_t component = 1)
{
  const auto unknown = std::numeric_limits<float>::quiet_NaN();
  mavlink_wind_cov_t value{};
  value.time_usec = std::numeric_limits<uint64_t>::max();
  value.wind_x = 3.25f;
  value.wind_y = unknown;
  value.wind_z = -1.5f;
  value.var_horiz = 1.5f;
  value.var_vert = unknown;
  value.wind_alt = 140.5f;
  value.vert_accuracy = .75f;
  mavlink_message_t message{};
  mavlink_msg_wind_cov_encode_status(system, component, &codec, &message, &value);
  return message;
}

inline mavlink_message_t pressure(mavlink_status_t& codec, uint8_t system = 1, uint8_t component = 1)
{
  mavlink_scaled_pressure_t value{};
  value.time_boot_ms = 424242;
  value.press_abs = 1007.25f;
  value.press_diff = -1.75f;
  value.temperature = 2345;
  value.temperature_press_diff = -125;
  mavlink_message_t message{};
  mavlink_msg_scaled_pressure_encode_status(system, component, &codec, &message, &value);
  return message;
}

inline mavlink_message_t airspeed(mavlink_status_t& codec, uint8_t sensor = 0,
                                  uint8_t system = 1, uint8_t component = 1)
{
  mavlink_airspeed_t packet{};
  packet.id = sensor;
  if(sensor == 0) {
    packet.airspeed = 19.75f;
    packet.raw_press = -175;
    packet.temperature = -125;
    packet.flags = AIRSPEED_SENSOR_UNHEALTHY | AIRSPEED_SENSOR_USING | 0x80;
  } else {
    packet.airspeed = 22.25f;
    packet.raw_press = std::numeric_limits<float>::quiet_NaN();
    packet.temperature = std::numeric_limits<int16_t>::max();
  }
  mavlink_message_t message{};
  mavlink_msg_airspeed_encode_status(system, component, &codec, &message, &packet);
  return message;
}

inline mavlink_message_t highLatency(mavlink_status_t& codec, uint8_t system = 1, uint8_t component = 1,
                                     uint8_t type = MAV_TYPE_FIXED_WING,
                                     uint8_t autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA)
{
  mavlink_high_latency2_t packet{};
  packet.type = type;
  packet.autopilot = autopilot;
  packet.timestamp = std::numeric_limits<uint32_t>::max();
  packet.airspeed = 101;
  packet.airspeed_sp = 112;
  mavlink_message_t message{};
  mavlink_msg_high_latency2_encode_status(system, component, &codec, &message, &packet);
  return message;
}

} // namespace mavmoos::test::air_data
