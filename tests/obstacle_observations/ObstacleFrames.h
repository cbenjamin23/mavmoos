#pragma once

#include <ardupilotmega/mavlink.h>
#include <cstddef>
#include <cstdint>

namespace mavmoos::test::obstacles {
inline mavlink_obstacle_distance_t payload()
{
  mavlink_obstacle_distance_t value{};
  value.time_usec = 9007199254740997ULL;
  value.min_distance = 20;
  value.max_distance = 600;
  value.sensor_type = MAV_DISTANCE_SENSOR_LASER;
  value.increment = 5;
  value.increment_f = -2.5f;
  value.angle_offset = 12.25f;
  value.frame = MAV_FRAME_BODY_FRD;
  for (std::size_t i = 0; i < MAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_DISTANCES_LEN; ++i)
    value.distances[i] = UINT16_MAX;
  value.distances[0] = 0;
  value.distances[1] = 25;
  value.distances[2] = 601;
  value.distances[4] = 10;
  value.distances[5] = 602;
  value.distances[6] = 600;
  value.distances[71] = 250;
  return value;
}

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_obstacle_distance_t& value,
                                uint8_t system = 1, uint8_t component = 1)
{
  mavlink_message_t frame{};
  mavlink_msg_obstacle_distance_encode_status(system, component, &codec, &frame, &value);
  return frame;
}

inline mavlink_message_t obstacle(mavlink_status_t& codec, uint8_t system = 1, uint8_t component = 1)
{ return encode(codec, payload(), system, component); }
} // namespace mavmoos::test::obstacles
