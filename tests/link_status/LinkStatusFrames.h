#pragma once

#include <ardupilotmega/mavlink.h>

#include <cstring>

namespace mavmoos::test::link_status_fixture {

inline mavlink_radio_status_t radioPayload()
{
  mavlink_radio_status_t value{};
  value.rssi = 201;
  value.remrssi = UINT8_MAX;
  value.noise = 0;
  value.remnoise = 47;
  value.txbuf = 37;
  value.rxerrors = UINT16_MAX;
  value.fixed = 4000;
  return value;
}

inline mavlink_cellular_status_t cellularPayload(uint8_t modem = 3)
{
  mavlink_cellular_status_t value{};
  value.status = CELLULAR_STATUS_FLAG_FAILED;
  value.failure_reason = CELLULAR_NETWORK_FAILED_REASON_SIM_ERROR;
  value.type = CELLULAR_NETWORK_RADIO_TYPE_LTE;
  value.quality = 72;
  value.mcc = 302;
  value.mnc = UINT16_MAX;
  value.id = modem;
  value.link_tx_rate = 123;
  value.link_rx_rate = 456;
  constexpr char tower[] = "C\"42\\AB99";
  static_assert(sizeof(tower) - 1 == sizeof(value.cell_tower_id));
  std::memcpy(value.cell_tower_id, tower, sizeof(value.cell_tower_id));
  value.band_number = 3;
  value.band_frequency = 1800.25f;
  value.channel_number = 1822;
  value.rx_level = -85.5f;
  value.tx_level = 13.25f;
  value.rx_quality = -12.5f;
  value.sinr = 8.75f;
  return value;
}

inline mavlink_radio_status_t radioWithoutSignal()
{
  mavlink_radio_status_t value{};
  value.rssi = value.remrssi = value.noise = value.remnoise = UINT8_MAX;
  value.txbuf = 100;
  return value;
}

inline mavlink_cellular_status_t cellularBasePayload()
{
  mavlink_cellular_status_t value{};
  value.status = 200;
  value.failure_reason = 254;
  value.type = 253;
  value.quality = UINT8_MAX;
  value.mcc = UINT16_MAX;
  return value;
}

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_radio_status_t& value,
                                uint8_t system = 1, uint8_t component = 1)
{
  mavlink_message_t message{};
  mavlink_msg_radio_status_encode_status(system, component, &codec, &message, &value);
  return message;
}

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_cellular_status_t& value,
                                uint8_t system = 1, uint8_t component = 1)
{
  mavlink_message_t message{};
  mavlink_msg_cellular_status_encode_status(system, component, &codec, &message, &value);
  return message;
}

inline mavlink_message_t radio(mavlink_status_t& codec, uint8_t system = 1, uint8_t component = 1)
{
  return encode(codec, radioPayload(), system, component);
}

inline mavlink_message_t cellular(mavlink_status_t& codec, uint8_t system = 1, uint8_t component = 1)
{
  return encode(codec, cellularPayload(), system, component);
}

inline mavlink_message_t cellularBase(mavlink_status_t& codec)
{
  return encode(codec, cellularBasePayload());
}

} // namespace mavmoos::test::link_status_fixture
