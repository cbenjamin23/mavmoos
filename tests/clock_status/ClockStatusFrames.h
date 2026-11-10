#pragma once

#include <ardupilotmega/mavlink.h>

#include <limits>

namespace mavmoos::test::clock_status_fixture {

inline mavlink_system_time_t systemTimePayload()
{
  return {std::numeric_limits<uint64_t>::max(), std::numeric_limits<uint32_t>::max()};
}

inline mavlink_timesync_t requestPayload()
{
  mavlink_timesync_t value{};
  value.ts1 = std::numeric_limits<int64_t>::min();
  value.target_system = 42;
  return value;
}

inline mavlink_timesync_t replyPayload()
{
  mavlink_timesync_t value{};
  value.tc1 = std::numeric_limits<int64_t>::max();
  value.ts1 = -9007199254741011LL;
  value.target_system = 245;
  value.target_component = 191;
  return value;
}

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_system_time_t& value,
                                uint8_t system = 1, uint8_t component = 1)
{
  mavlink_message_t message{};
  mavlink_msg_system_time_encode_status(system, component, &codec, &message, &value);
  return message;
}

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_timesync_t& value,
                                uint8_t system = 1, uint8_t component = 1)
{
  mavlink_message_t message{};
  mavlink_msg_timesync_encode_status(system, component, &codec, &message, &value);
  return message;
}

inline mavlink_message_t systemTime(mavlink_status_t& codec, uint8_t system = 1, uint8_t component = 1)
{
  return encode(codec, systemTimePayload(), system, component);
}

inline mavlink_message_t request(mavlink_status_t& codec, uint8_t system = 1, uint8_t component = 1)
{
  return encode(codec, requestPayload(), system, component);
}

inline mavlink_message_t reply(mavlink_status_t& codec, uint8_t system = 1, uint8_t component = 1)
{
  return encode(codec, replyPayload(), system, component);
}

} // namespace mavmoos::test::clock_status_fixture
