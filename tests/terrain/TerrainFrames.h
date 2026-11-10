#pragma once

#include <ardupilotmega/mavlink.h>
#include <cstdint>

namespace mavmoos::test::terrain {
constexpr uint64_t mask = 0x80000000000001ULL;

inline mavlink_message_t report(mavlink_status_t& codec, uint16_t spacing = 100, float terrain = -14.75f,
                                float current = 5.5f, uint8_t system = 1, uint8_t component = 1,
                                int32_t latitude = 423584567, int32_t longitude = -710875891)
{
  mavlink_message_t frame{};
  mavlink_msg_terrain_report_pack_status(system, component, &codec, &frame, latitude, longitude,
                                         spacing, terrain, current, 12, 4);
  return frame;
}

inline mavlink_message_t request(mavlink_status_t& codec, uint64_t bits = mask, uint16_t spacing = 100,
                                 uint8_t system = 1, uint8_t component = 1, int32_t latitude = 423580000,
                                 int32_t longitude = -710880000)
{
  mavlink_message_t frame{};
  mavlink_msg_terrain_request_pack_status(system, component, &codec, &frame, latitude, longitude,
                                          spacing, bits);
  return frame;
}
} // namespace mavmoos::test::terrain
