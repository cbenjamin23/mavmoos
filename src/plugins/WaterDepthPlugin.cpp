#include "PassiveObserver.h"

#include <array>
#include <limits>

namespace mavmoos::runtime {
namespace {

// The pinned Rover SITL build supports ten onboard rangefinder instances.
constexpr std::size_t sensor_limit = 10;

void writeNullable(std::ostream& out, float value)
{
  if (std::isfinite(value))
    out << value;
  else
    out << "null";
}

const char* healthName(uint8_t health)
{
  if (health == 1)
    return "healthy";
  if (health == 0)
    return "unhealthy";
  return "unknown";
}

const char* distanceStatus(const mavlink_water_depth_t& depth)
{
  if (!std::isfinite(depth.distance))
    return "unavailable_nonfinite";
  if (depth.distance < 0)
    return "invalid_negative";
  if (depth.healthy != 1)
    return "unavailable_health";
  return "reported_healthy";
}

void writeDepth(std::ostream& out, const mavlink_water_depth_t& depth)
{
  const bool coordinates_in_bounds = depth.lat >= -900000000 && depth.lat <= 900000000 &&
    depth.lng >= -1800000000 && depth.lng <= 1800000000;
  const bool zero_position = depth.lat == 0 && depth.lng == 0;
  const bool position_reported = coordinates_in_bounds && !zero_position;
  const bool distance_reported = depth.healthy == 1 && std::isfinite(depth.distance) && depth.distance >= 0;
  const bool temperature_reported = std::isfinite(depth.temperature) && depth.temperature != 0;

  out << "{\"time_boot_ms_raw\":" << depth.time_boot_ms
      << ",\"health_raw\":" << unsigned(depth.healthy) << ",\"health\":\"" << healthName(depth.healthy)
      << "\",\"distance_m_raw\":";
  writeNullable(out, depth.distance);
  out << ",\"bottom_distance_uncorrected_m\":";
  if (distance_reported)
    out << depth.distance;
  else
    out << "null";
  out << ",\"distance_status\":\"" << distanceStatus(depth)
      << "\",\"latitude_e7_raw\":" << depth.lat << ",\"longitude_e7_raw\":" << depth.lng
      << ",\"position_status\":\""
      << (position_reported ? "reported_unconfirmed" : zero_position ? "zero_ambiguous" : "out_of_bounds")
      << "\",\"latitude_deg\":";
  if (position_reported)
    out << depth.lat / 1e7;
  else
    out << "null";
  out << ",\"longitude_deg\":";
  if (position_reported)
    out << depth.lng / 1e7;
  else
    out << "null";
  out << ",\"vehicle_altitude_msl_m_reported\":";
  writeNullable(out, depth.alt);
  out << ",\"vehicle_roll_rad_reported\":";
  writeNullable(out, depth.roll);
  out << ",\"vehicle_pitch_rad_reported\":";
  writeNullable(out, depth.pitch);
  out << ",\"vehicle_yaw_rad_reported\":";
  writeNullable(out, depth.yaw);
  out << ",\"temperature_c_raw\":";
  writeNullable(out, depth.temperature);
  out << ",\"temperature_c\":";
  if (temperature_reported)
    out << depth.temperature;
  else
    out << "null";
  out << ",\"temperature_status\":\""
      << (temperature_reported ? "reported_nonzero" :
          depth.temperature == 0 ? "zero_ambiguous" : "unavailable_nonfinite") << "\"}";
}

class WaterDepth final : public PassiveObserver {
public:
  explicit WaterDepth(Services& services)
    : PassiveObserver(services, false, 1, MAV_TYPE_SURFACE_BOAT) {}

  std::string name() const override { return "water_depth"; }

  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.profile = "ardupilot_surface_water_depth";
    result.capabilities = {{"water_depth.water_depth", "", "", "observation"}};
    result.exclusive_resources = {"observation.water_depth"};
    result.streamed_parameter_bench_compatible = true;
    return result;
  }

private:
  struct Sample {
    Stamp stamp;
    mavlink_water_depth_t depth{};
    uint8_t payload_length = 0;
  };

  bool observes(uint32_t id) const override { return id == MAVLINK_MSG_ID_WATER_DEPTH; }

  void observe(const mavlink_message_t& message, const MessageTime& time) override
  {
    mavlink_water_depth_t depth{};
    const bool shape = wireShape(message, MAVLINK_MSG_ID_WATER_DEPTH_MIN_LEN, MAVLINK_MSG_ID_WATER_DEPTH_LEN);
    if (shape)
      mavlink_msg_water_depth_decode(&message, &depth);
    if (!shape || depth.id >= m_samples.size()) {
      rejectWire();
      ++m_invalid;
      return;
    }

    auto& sample = m_samples[depth.id];
    if (update(sample.stamp, time)) {
      sample.depth = depth;
      sample.payload_length = message.len;
    }
  }

  void clearSamples() override { m_samples = {}; }

  void publishSamples() override
  {
    bool present = false;
    for (const auto& sample : m_samples)
      present = present || sample.stamp.id != 0;

    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::boolalpha << std::setprecision(std::numeric_limits<double>::max_digits10)
        << "{\"schema\":1,\"family\":\"WATER_DEPTH\",\"message_id\":" << MAVLINK_MSG_ID_WATER_DEPTH
        << ",\"present\":" << present << ",\"read_only\":true,\"authoritative_for_control\":false"
        << ",\"atomic\":false,\"sensor_limit\":" << m_samples.size()
        << ",\"invalid_samples\":" << m_invalid << ",\"samples\":[";
    bool first = true;
    for (const auto& sample : m_samples) {
      if (!sample.stamp.id)
        continue;
      if (!first)
        out << ',';
      first = false;
      out << "{\"sensor_id\":" << unsigned(sample.depth.id) << ",\"wire_version\":2,\"payload_length\":"
          << unsigned(sample.payload_length) << ",\"metadata\":" << metadata(sample.stamp) << ",\"data\":";
      writeDepth(out, sample.depth);
      out << '}';
    }
    out << "]}";
    services().publish("MAVMOOS_WATER_DEPTH", out.str());
  }

  std::array<Sample, sensor_limit> m_samples{};
  uint64_t m_invalid = 0;
};

} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi, mavmoos::runtime::Services* services)
{
  using namespace mavmoos::runtime;
  return abi == plugin_abi && services ? new WaterDepth(*services) : nullptr;
}

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* module) { delete module; }
