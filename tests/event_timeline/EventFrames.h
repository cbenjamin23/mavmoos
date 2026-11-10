#pragma once

#include <ardupilotmega/mavlink.h>
#include <array>

namespace mavmoos::test::event_timeline_fixture {

inline mavlink_event_t event(uint16_t sequence, bool directed = false)
{
  mavlink_event_t packet{};
  packet.sequence = sequence;
  packet.id = directed ? 0x01020304u : 0xab010203u;
  packet.event_time_boot_ms = directed ? 12u : 0xfffffffeu;
  packet.destination_system = directed ? 245 : 0;
  packet.destination_component = directed ? 191 : 0;
  packet.log_levels = directed ? 0xc6 : 0x84;
  packet.arguments[0] = 0x12;
  packet.arguments[2] = 0xff;
  if (!directed) packet.arguments[39] = 0x5a;
  return packet;
}

inline mavlink_current_event_sequence_t current(uint16_t sequence, uint8_t flags = 0x80)
{
  return {sequence, flags};
}

inline mavlink_response_event_error_t error(uint16_t sequence, uint16_t oldest, uint8_t reason = 0)
{
  return {sequence, oldest, 247, 190, reason};
}

template<typename Payload, typename Encoder>
inline mavlink_message_t encodeFrame(mavlink_status_t& codec, const Payload& packet, Encoder encode,
                                     uint8_t system, uint8_t component)
{
  mavlink_message_t message{};
  encode(system, component, &codec, &message, &packet);
  return message;
}

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_event_t& packet,
                                uint8_t system = 1, uint8_t component = 1)
{ return encodeFrame(codec, packet, mavlink_msg_event_encode_status, system, component); }

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_current_event_sequence_t& packet,
                                uint8_t system = 1, uint8_t component = 1)
{ return encodeFrame(codec, packet, mavlink_msg_current_event_sequence_encode_status, system, component); }

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_response_event_error_t& packet,
                                uint8_t system = 1, uint8_t component = 1)
{ return encodeFrame(codec, packet, mavlink_msg_response_event_error_encode_status, system, component); }

inline std::array<mavlink_message_t, 5> frames(mavlink_status_t& codec, unsigned batch)
{
  const auto first = static_cast<uint16_t>(65456u + batch * 5u);
  auto other = event(static_cast<uint16_t>(first + 1), true);
  other.destination_system = 44;
  other.destination_component = 19;
  other.id = 0xfeffffff;
  const auto missing = static_cast<uint16_t>(first + 2);
  const auto local = static_cast<uint16_t>(first + 3);
  return {encode(codec, event(first)), encode(codec, other), encode(codec, event(local, true)),
          encode(codec, current(static_cast<uint16_t>(first + 4))), encode(codec, error(missing, local))};
}

} // namespace mavmoos::test::event_timeline_fixture
