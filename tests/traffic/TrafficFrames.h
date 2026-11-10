#pragma once

#include <ardupilotmega/mavlink.h>
#include <algorithm>

namespace mavmoos::test::traffic {

constexpr uint32_t complete_icao = 0x5f0021;
constexpr uint32_t partial_icao = 0x05a4c0;
constexpr uint32_t foreign_icao = 0x05a4c1;
constexpr uint32_t invalid_icao = 0x0105a4c2;

inline mavlink_adsb_vehicle_t complete(uint32_t address = complete_icao)
{
  mavlink_adsb_vehicle_t value{};
  value.ICAO_address = address;
  value.lat = 423584560;
  value.lon = -710875890;
  value.altitude = 123450;
  value.altitude_type = ADSB_ALTITUDE_TYPE_GEOMETRIC;
  value.heading = 27125;
  value.hor_velocity = 2575;
  value.ver_velocity = -125;
  constexpr char callsign[] = "SIM\"\\3";
  std::copy(std::begin(callsign), std::end(callsign), value.callsign);
  value.emitter_type = ADSB_EMITTER_TYPE_LIGHT;
  value.tslc = 7;
  value.squawk = 7700;
  value.flags = ADSB_FLAGS_VALID_COORDS | ADSB_FLAGS_VALID_ALTITUDE | ADSB_FLAGS_VALID_HEADING |
                ADSB_FLAGS_VALID_VELOCITY | ADSB_FLAGS_VALID_CALLSIGN | ADSB_FLAGS_VALID_SQUAWK |
                ADSB_FLAGS_SIMULATED | ADSB_FLAGS_VERTICAL_VELOCITY_VALID;
  return value;
}

inline mavlink_adsb_vehicle_t partial()
{
  auto value = complete(partial_icao);
  value.lat = INT32_MAX;
  value.altitude = INT32_MAX;
  value.heading = UINT16_MAX;
  value.hor_velocity = UINT16_MAX;
  value.ver_velocity = INT16_MAX;
  value.squawk = 8888;
  std::fill(std::begin(value.callsign), std::end(value.callsign), 'X');
  value.tslc = 255;
  return value;
}

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_adsb_vehicle_t& value,
                                uint8_t system = 1, uint8_t component = 1)
{
  mavlink_message_t message{};
  mavlink_msg_adsb_vehicle_encode_status(system, component, &codec, &message, &value);
  return message;
}

} // namespace mavmoos::test::traffic
