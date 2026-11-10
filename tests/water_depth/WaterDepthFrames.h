#pragma once

#include <ardupilotmega/mavlink.h>

#include <array>
#include <limits>

namespace mavmoos::test::water_depth {

inline mavlink_water_depth_t reading(uint8_t id = 0)
{
  mavlink_water_depth_t result{};
  result.time_boot_ms = std::numeric_limits<uint32_t>::max();
  result.lat = 423584567;
  result.lng = -710875891;
  result.alt = -1.25f;
  result.roll = -.25f;
  result.pitch = .125f;
  result.yaw = 1.75f;
  result.distance = 4.75f;
  result.temperature = -2.5f;
  result.id = id;
  result.healthy = 1;
  return result;
}

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_water_depth_t& value,
                                uint8_t system = 1, uint8_t component = 1)
{
  mavlink_message_t frame{};
  mavlink_msg_water_depth_encode_status(system, component, &codec, &frame, &value);
  return frame;
}

inline mavlink_message_t heartbeat(mavlink_status_t& codec, uint8_t type = MAV_TYPE_SURFACE_BOAT,
                                   uint8_t autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA)
{
  mavlink_message_t frame{};
  mavlink_msg_heartbeat_pack_status(1, 1, &codec, &frame, type, autopilot, 0, 0, MAV_STATE_STANDBY);
  return frame;
}

inline std::array<mavlink_message_t, 2> generatedFrames(mavlink_status_t& codec)
{
  auto secondary = reading(5);
  secondary.time_boot_ms = 2;
  secondary.distance = 31.25f;
  secondary.temperature = 0;
  secondary.healthy = 0;
  return {encode(codec, reading()), encode(codec, secondary)};
}

inline std::array<mavlink_message_t, 2> generatedWrongSources(mavlink_status_t& codec)
{
  auto rogue = reading(9);
  rogue.distance = 444;
  return {encode(codec, rogue, 2, 1), encode(codec, rogue, 1, 2)};
}

} // namespace mavmoos::test::water_depth
