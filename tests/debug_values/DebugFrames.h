#pragma once

#include <ardupilotmega/mavlink.h>
#include <algorithm>
#include <array>
#include <iterator>
#include <limits>
#include <string_view>

namespace mavmoos::test::debug_fixture {

inline mavlink_debug_t debug(uint8_t index = 7, float value = -2.5f)
{
  mavlink_debug_t result{};
  result.ind = index;
  result.value = value;
  result.time_boot_ms = std::numeric_limits<uint32_t>::max();
  return result;
}

inline mavlink_debug_vect_t vector(std::string_view name = "x\"\\v")
{
  mavlink_debug_vect_t result{};
  std::copy_n(name.begin(), std::min(name.size(), sizeof(result.name)), result.name);
  result.time_usec = 9007199254741001ULL;
  result.x = .125f;
  result.y = std::numeric_limits<float>::quiet_NaN();
  result.z = std::numeric_limits<float>::infinity();
  return result;
}

inline mavlink_named_value_float_t namedFloat(std::string_view name = "shared", float value = -0.0f)
{
  mavlink_named_value_float_t result{};
  std::copy_n(name.begin(), std::min(name.size(), sizeof(result.name)), result.name);
  result.time_boot_ms = std::numeric_limits<uint32_t>::max();
  result.value = value;
  return result;
}

inline mavlink_named_value_int_t namedInt(std::string_view name = "shared",
                                         int32_t value = std::numeric_limits<int32_t>::min())
{
  mavlink_named_value_int_t result{};
  std::copy_n(name.begin(), std::min(name.size(), sizeof(result.name)), result.name);
  result.value = value;
  return result;
}

inline mavlink_debug_float_array_t floatArray(uint16_t id = 17, std::string_view name = "ARRAY01234")
{
  mavlink_debug_float_array_t result{};
  std::copy_n(name.begin(), std::min(name.size(), sizeof(result.name)), result.name);
  result.array_id = id;
  result.time_usec = 9007199254741009ULL;
  result.data[0] = -0.0f;
  result.data[1] = std::numeric_limits<float>::quiet_NaN();
  result.data[2] = std::numeric_limits<float>::infinity();
  result.data[3] = -std::numeric_limits<float>::infinity();
  result.data[4] = 1.5f;
  result.data[57] = 4.5f;
  return result;
}

template<typename Payload, typename Encoder>
inline mavlink_message_t encodeFrame(mavlink_status_t& codec, const Payload& value,
                                     Encoder encoder, uint8_t system, uint8_t component)
{
  mavlink_message_t frame{};
  encoder(system, component, &codec, &frame, &value);
  return frame;
}

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_debug_t& value,
                                uint8_t system = 1, uint8_t component = 1)
{ return encodeFrame(codec, value, mavlink_msg_debug_encode_status, system, component); }

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_debug_vect_t& value,
                                uint8_t system = 1, uint8_t component = 1)
{ return encodeFrame(codec, value, mavlink_msg_debug_vect_encode_status, system, component); }

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_named_value_float_t& value,
                                uint8_t system = 1, uint8_t component = 1)
{ return encodeFrame(codec, value, mavlink_msg_named_value_float_encode_status, system, component); }

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_named_value_int_t& value,
                                uint8_t system = 1, uint8_t component = 1)
{ return encodeFrame(codec, value, mavlink_msg_named_value_int_encode_status, system, component); }

inline mavlink_message_t encode(mavlink_status_t& codec, const mavlink_debug_float_array_t& value,
                                uint8_t system = 1, uint8_t component = 1)
{ return encodeFrame(codec, value, mavlink_msg_debug_float_array_encode_status, system, component); }

inline std::array<mavlink_message_t, 8> baseFrames(mavlink_status_t& codec)
{
  auto zero_index = debug(0, 0);
  zero_index.time_boot_ms = 1;
  auto full_name = vector("VECTOR0123");
  full_name.time_usec = 18446744073709551000ULL;
  full_name.x = -0.0f;
  full_name.y = -std::numeric_limits<float>::infinity();
  full_name.z = 2.25f;
  auto empty_name = namedInt("", std::numeric_limits<int32_t>::max());
  empty_name.time_boot_ms = 0x80000000;
  const char raw_name[] = {'x', static_cast<char>(0xff)};
  return {encode(codec, debug()), encode(codec, zero_index),
          encode(codec, vector()), encode(codec, full_name),
          encode(codec, namedFloat()),
          encode(codec, namedFloat(std::string_view(raw_name, 2), std::numeric_limits<float>::quiet_NaN())),
          encode(codec, namedInt()), encode(codec, empty_name)};
}

inline std::array<mavlink_message_t, 10> frames(mavlink_status_t& codec)
{
  std::array<mavlink_message_t, 10> result{};
  const auto base = baseFrames(codec);
  std::copy(base.begin(), base.end(), result.begin());
  auto trimmed = floatArray(std::numeric_limits<uint16_t>::max());
  trimmed.time_usec = 18446744073709551000ULL;
  std::fill(std::begin(trimmed.data), std::end(trimmed.data), 0);
  // Binary32 denorm_min occupies one nonzero little-endian wire byte.
  trimmed.data[0] = std::numeric_limits<float>::denorm_min();
  result[8] = encode(codec, floatArray());
  result[9] = encode(codec, trimmed);
  return result;
}

} // namespace mavmoos::test::debug_fixture
