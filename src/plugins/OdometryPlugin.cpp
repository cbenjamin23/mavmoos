/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: OdometryPlugin.cpp                              */
/*    DATE: November 5, 2026                                */
/************************************************************/
#include "PassiveObserver.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iterator>
#include <locale>
#include <optional>
#include <sstream>
#include <utility>

namespace mavmoos::runtime {
namespace {

template<class Range>
bool allFinite(const Range& values)
{
  return std::all_of(std::begin(values), std::end(values),
                     [](float value) { return std::isfinite(value); });
}

template<class Range>
void writeArray(std::ostream& out, const Range& values)
{
  out << '[';
  bool first = true;
  for (float value : values) {
    if (!first) out << ',';
    first = false;
    if (std::isnan(value)) out << "null";
    else out << value;
  }
  out << ']';
}

template<std::size_t N>
bool validCovariance(const float (&values)[N])
{
  return std::isnan(values[0]) || std::none_of(std::begin(values), std::end(values),
                                             [](float value) { return std::isinf(value); });
}

template<std::size_t N>
void writeCovariance(std::ostream& out, const float (&values)[N])
{
  if (std::isnan(values[0])) out << "null";
  else writeArray(out, values);
}

std::ostringstream beginData(const mavlink_message_t& message)
{
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(17) << "{\"message_id\":" << message.msgid
      << ",\"wire_version\":" << (message.magic == MAVLINK_STX ? 2 : 1)
      << ",\"payload_length\":" << unsigned(message.len);
  return out;
}

std::optional<std::string> localPosition(const mavlink_message_t& message)
{
  mavlink_local_position_ned_t value{};
  mavlink_msg_local_position_ned_decode(&message, &value);
  const std::array<float, 3> position{value.x, value.y, value.z};
  const std::array<float, 3> velocity{value.vx, value.vy, value.vz};
  if (!allFinite(position) || !allFinite(velocity)) return std::nullopt;

  auto out = beginData(message);
  out << ",\"frame\":\"LOCAL_NED\",\"origin\":\"FC_LOCAL_ORIGIN\""
      << ",\"time_boot_ms\":" << value.time_boot_ms << ",\"position_m\":";
  writeArray(out, position);
  out << ",\"velocity_m_s\":";
  writeArray(out, velocity);
  out << '}';
  return out.str();
}

std::optional<std::string> rawOdometry(const mavlink_message_t& message)
{
  mavlink_odometry_t value{};
  mavlink_msg_odometry_decode(&message, &value);
  const std::array<float, 3> position{value.x, value.y, value.z};
  const std::array<float, 3> velocity{value.vx, value.vy, value.vz};
  const std::array<float, 3> angular{value.rollspeed, value.pitchspeed, value.yawspeed};
  const bool zero_quaternion = std::all_of(std::begin(value.q), std::end(value.q),
                                          [](float component) { return component == 0; });
  if (!allFinite(position) || !allFinite(velocity) || !allFinite(angular) ||
      !allFinite(value.q) || zero_quaternion || value.quality < -1 || value.quality > 100 ||
      !validCovariance(value.pose_covariance) || !validCovariance(value.velocity_covariance))
    return std::nullopt;

  auto out = beginData(message);
  out << ",\"time_usec_raw\":\"" << value.time_usec << "\",\"frame_id\":" << unsigned(value.frame_id)
      << ",\"child_frame_id\":" << unsigned(value.child_frame_id) << ",\"position_m\":";
  writeArray(out, position);
  out << ",\"quaternion_wxyz\":";
  writeArray(out, value.q);
  out << ",\"velocity_m_s\":";
  writeArray(out, velocity);
  out << ",\"angular_velocity_rad_s\":";
  writeArray(out, angular);
  out << ",\"pose_covariance\":";
  writeCovariance(out, value.pose_covariance);
  out << ",\"velocity_covariance\":";
  writeCovariance(out, value.velocity_covariance);
  out << ",\"reset_counter\":" << unsigned(value.reset_counter)
      << ",\"estimator_type\":" << unsigned(value.estimator_type) << ",\"quality\":";
  if (value.quality == 0) out << "null"; else out << int(value.quality);
  out << '}';
  return out.str();
}

std::optional<std::string> globalOrigin(const mavlink_message_t& message)
{
  mavlink_gps_global_origin_t value{};
  mavlink_msg_gps_global_origin_decode(&message, &value);
  if (value.latitude < -900000000 || value.latitude > 900000000 ||
      value.longitude < -1800000000 || value.longitude > 1800000000)
    return std::nullopt;

  auto out = beginData(message);
  out << ",\"latitude_deg_e7\":" << value.latitude
      << ",\"longitude_deg_e7\":" << value.longitude
      << ",\"altitude_msl_mm\":" << value.altitude
      << ",\"latitude_deg\":" << value.latitude / 1e7
      << ",\"longitude_deg\":" << value.longitude / 1e7
      << ",\"altitude_msl_m\":" << value.altitude / 1000.0
      << ",\"time_usec_raw\":";
  if (message.len > MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN && value.time_usec)
    out << '\"' << value.time_usec << '\"';
  else
    out << "null";
  out << '}';
  return out.str();
}

std::optional<std::string> wheelDistance(const mavlink_message_t& message)
{
  mavlink_wheel_distance_t value{};
  mavlink_msg_wheel_distance_decode(&message, &value);
  constexpr unsigned capacity = MAVLINK_MSG_WHEEL_DISTANCE_FIELD_DISTANCE_LEN;
  const unsigned count = value.count;
  if (count > capacity) return std::nullopt;

  std::array<unsigned, capacity> unknown_indices{};
  unsigned unknown_count = 0;
  auto out = beginData(message);
  out << ",\"time_usec_raw\":\"" << value.time_usec
      << "\",\"remote_clock\":\"unix_or_boot_unmapped_uint64\""
      << ",\"wheel_count\":" << count << ",\"wheel_capacity\":" << capacity
      << ",\"wheel_order\":\"wire_index\",\"distance_m\":[";
  for (unsigned index = 0; index < count; ++index) {
    const double distance = value.distance[index];
    if (std::isinf(distance)) return std::nullopt;
    if (index) out << ',';
    if (std::isnan(distance)) {
      out << "null";
      unknown_indices[unknown_count++] = index;
    } else if (distance == 0 && std::signbit(distance)) {
      out << "-0.0";
    } else {
      out << distance;
    }
  }
  out << "],\"unknown_wheel_indices\":[";
  for (unsigned index = 0; index < unknown_count; ++index) {
    if (index) out << ',';
    out << unknown_indices[index];
  }
  out << "]}";
  return out.str();
}

class Odometry final : public PassiveObserver {
public:
  explicit Odometry(Services& services) : PassiveObserver(services, true) {}

  std::string name() const override { return "odometry"; }
  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.profile = "common_local_ned_and_raw_odometry";
    result.capabilities = {{"odometry.local_kinematics", "", "", "observation"},
                           {"odometry.raw", "", "", "observation"},
                           {"odometry.local_origin", "", "", "observation"},
                           {"odometry.wheel_distance", "", "", "observation"}};
    result.exclusive_resources = {"observation.odometry"};
    result.streamed_parameter_bench_compatible = true;
    return result;
  }

private:
  struct Sample {
    Stamp stamp;
    std::string data = "null";
  };

  bool observes(uint32_t message_id) const override
  {
    return message_id == MAVLINK_MSG_ID_LOCAL_POSITION_NED || message_id == MAVLINK_MSG_ID_ODOMETRY ||
           message_id == MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN || message_id == MAVLINK_MSG_ID_WHEEL_DISTANCE;
  }

  void observe(const mavlink_message_t& message, const MessageTime& time) override
  {
    switch (message.msgid) {
      case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        observeSample(message, time, m_local, localPosition,
                      MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);
        break;
      case MAVLINK_MSG_ID_ODOMETRY:
        observeSample(message, time, m_raw, rawOdometry, 0, MAVLINK_MSG_ID_ODOMETRY_LEN);
        break;
      case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
        observeSample(message, time, m_origin, globalOrigin,
                      MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_MIN_LEN, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN);
        break;
      case MAVLINK_MSG_ID_WHEEL_DISTANCE:
        observeSample(message, time, m_wheels, wheelDistance, 0, MAVLINK_MSG_ID_WHEEL_DISTANCE_LEN);
        break;
    }
  }

  void observeSample(const mavlink_message_t& message, const MessageTime& time, Sample& sample,
                     std::optional<std::string> (*decode)(const mavlink_message_t&),
                     unsigned v1_length, unsigned max_length)
  {
    if (!wireShape(message, v1_length, max_length)) {
      rejectWire();
      return;
    }
    auto data = decode(message);
    if (!data) {
      rejectWire();
      return;
    }
    if (update(sample.stamp, time)) sample.data = std::move(*data);
  }

  void clearSamples() override
  {
    m_local = {};
    m_raw = {};
    m_origin = {};
    m_wheels = {};
  }

  void publishSamples() override
  {
    publishSample("MAVMOOS_LOCAL_KINEMATICS", m_local);
    publishSample("MAVMOOS_ODOMETRY", m_raw);
    publishSample("MAVMOOS_LOCAL_ORIGIN", m_origin);
    publishSample("MAVMOOS_WHEEL_DISTANCE", m_wheels);
  }

  void publishSample(const char* key, const Sample& sample)
  {
    auto record = metadata(sample.stamp);
    record.pop_back();
    record += ",\"data\":" + sample.data + '}';
    services().publish(key, record);
  }

  Sample m_local, m_raw, m_origin, m_wheels;
};

} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi, mavmoos::runtime::Services* services)
{
  return abi == mavmoos::runtime::plugin_abi && services ? new mavmoos::runtime::Odometry(*services) : nullptr;
}

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin)
{
  delete plugin;
}
