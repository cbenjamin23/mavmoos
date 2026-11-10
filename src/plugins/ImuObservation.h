#pragma once
#include <ardupilotmega/mavlink.h>
#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace mavmoos::runtime::imu {
// Scalar copies avoid referring to potentially unaligned packed wire members.
// The decoder does not bind a source, infer health or own an observation clock.
struct Observation {
  uint32_t message_id=0;
  std::string family;
  unsigned sensor_instance=0, payload_length=0;
  bool mavlink2=false, raw_id_extension=false;
  uint64_t remote_stamp=0;
  std::array<int16_t,3> acceleration{}, angular_velocity{}, magnetic_field{};
  int16_t temperature=0;
};
struct VibrationReport {
  uint64_t time_usec=0;
  std::array<float,3> levels{};
  std::array<uint32_t,3> clipping{};
  unsigned payload_length=0;
  bool mavlink2=false;
};
bool shape(const mavlink_message_t&);
std::optional<Observation> decode(const mavlink_message_t&);
std::string data(const Observation&,bool remote_timestamp_regressed);
std::optional<VibrationReport> decodeVibration(const mavlink_message_t&);
std::string data(const VibrationReport&,bool remote_timestamp_regressed);
} // namespace mavmoos::runtime::imu
