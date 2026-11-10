#pragma once

#include <ardupilotmega/mavlink.h>
#include <cstdint>
#include <limits>

namespace mavmoos::test::wheels {
inline mavlink_wheel_distance_t payload()
{
  mavlink_wheel_distance_t value{};
  value.time_usec = 9007199254740999ULL;
  value.count = 4;
  value.distance[0] = 12.125;
  value.distance[1] = -3.75;
  value.distance[2] = std::numeric_limits<double>::quiet_NaN();
  value.distance[3] = -0.0;
  return value;
}

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_wheel_distance_t& value,
                                uint8_t system = 1, uint8_t component = 1)
{
  mavlink_message_t frame{};
  mavlink_msg_wheel_distance_encode_status(system, component, &codec, &frame, &value);
  return frame;
}

inline mavlink_message_t wheelDistance(mavlink_status_t& codec, uint8_t system = 1, uint8_t component = 1)
{ return encode(codec, payload(), system, component); }
} // namespace mavmoos::test::wheels
