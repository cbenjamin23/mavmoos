#pragma once

#include <ardupilotmega/mavlink.h>
#include <limits>

namespace mavmoos::test::imu {

inline mavlink_message_t vibration(mavlink_status_t& codec)
{
  mavlink_vibration_t packet{};
  packet.time_usec = std::numeric_limits<uint64_t>::max();
  packet.vibration_x = 1.25f;
  packet.vibration_y = 2.5f;
  packet.vibration_z = 3.75f;
  packet.clipping_0 = 7;
  packet.clipping_1 = std::numeric_limits<uint32_t>::max();
  packet.clipping_2 = 0;
  mavlink_message_t message{};
  mavlink_msg_vibration_encode_status(1, 1, &codec, &message, &packet);
  return message;
}

} // namespace mavmoos::test::imu
