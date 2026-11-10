/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: OpticalFlowPlugin.cpp                           */
/*    DATE: November 6, 2026                                */
/************************************************************/
#include "PassiveObserver.h"

#include <cstddef>
#include <limits>
#include <map>

namespace mavmoos::runtime {
namespace {

constexpr std::size_t sensor_limit = 8;

struct Wire {
  unsigned version;
  unsigned length;
};

void writeNullable(std::ostream& out, float value, bool reported)
{
  if (reported)
    out << value;
  else
    out << "null";
}

bool validPayload(const mavlink_optical_flow_t& value)
{
  return std::isfinite(value.flow_comp_m_x) && std::isfinite(value.flow_comp_m_y) &&
    std::isfinite(value.ground_distance) && std::isfinite(value.flow_rate_x) &&
    std::isfinite(value.flow_rate_y);
}

bool validPayload(const mavlink_optical_flow_rad_t& value)
{
  // PX4 forwards NaN for unavailable integrated gyro axes.
  return std::isfinite(value.integrated_x) && std::isfinite(value.integrated_y) &&
    !std::isinf(value.integrated_xgyro) && !std::isinf(value.integrated_ygyro) &&
    !std::isinf(value.integrated_zgyro) && std::isfinite(value.distance);
}

void writeData(std::ostream& out, const mavlink_optical_flow_t& value, const Wire& wire)
{
  out << "{\"time_usec_raw\":\"" << value.time_usec << "\""
      << ",\"flow_x_raw\":" << value.flow_x << ",\"flow_y_raw\":" << value.flow_y
      << ",\"flow_comp_m_x_raw\":" << value.flow_comp_m_x
      << ",\"flow_comp_m_y_raw\":" << value.flow_comp_m_y
      << ",\"ground_distance_m_raw\":" << value.ground_distance << ",\"ground_distance_m\":";
  writeNullable(out, value.ground_distance, value.ground_distance > 0);
  // MAVLink 2 can trim a field to a single byte. A whole absent extension
  // cannot distinguish a transmitted zero from an unsupported field.
  out << ",\"flow_rate_x_rad_s\":";
  writeNullable(out, value.flow_rate_x, wire.length > offsetof(mavlink_optical_flow_t, flow_rate_x));
  out << ",\"flow_rate_y_rad_s\":";
  writeNullable(out, value.flow_rate_y, wire.length > offsetof(mavlink_optical_flow_t, flow_rate_y));
  out << ",\"quality_raw\":" << unsigned(value.quality) << '}';
}

void writeData(std::ostream& out, const mavlink_optical_flow_rad_t& value, const Wire&)
{
  out << "{\"time_usec_raw\":\"" << value.time_usec << "\""
      << ",\"integration_time_us\":" << value.integration_time_us
      << ",\"integrated_x_rad\":" << value.integrated_x
      << ",\"integrated_y_rad\":" << value.integrated_y
      << ",\"integrated_xgyro_rad\":";
  writeNullable(out, value.integrated_xgyro, std::isfinite(value.integrated_xgyro));
  out << ",\"integrated_ygyro_rad\":";
  writeNullable(out, value.integrated_ygyro, std::isfinite(value.integrated_ygyro));
  out << ",\"integrated_zgyro_rad\":";
  writeNullable(out, value.integrated_zgyro, std::isfinite(value.integrated_zgyro));
  out << ",\"distance_m_raw\":" << value.distance << ",\"distance_m\":";
  writeNullable(out, value.distance, value.distance >= 0);
  out << ",\"time_delta_distance_us_raw\":" << value.time_delta_distance_us
      << ",\"temperature_cdeg_raw\":" << value.temperature
      << ",\"quality_raw\":" << unsigned(value.quality) << '}';
}

class OpticalFlow final : public PassiveObserver {
public:
  explicit OpticalFlow(Services& services) : PassiveObserver(services, true) {}

  std::string name() const override { return "optical_flow"; }

  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.profile = "mavlink_optical_flow_observations";
    result.capabilities = {{"optical_flow.optical_flow", "", "", "observation"},
                           {"optical_flow.optical_flow_rad", "", "", "observation"}};
    result.exclusive_resources = {"observation.optical_flow"};
    result.streamed_parameter_bench_compatible = true;
    return result;
  }

private:
  template<typename Payload>
  struct Sample {
    Stamp stamp;
    Wire wire{};
    Payload payload{};
  };

  template<typename Payload>
  struct Family {
    std::map<unsigned, Sample<Payload>> samples;
    uint64_t invalid = 0;
    uint64_t capacity_drops = 0;
  };

  bool observes(uint32_t id) const override
  {
    return id == MAVLINK_MSG_ID_OPTICAL_FLOW || id == MAVLINK_MSG_ID_OPTICAL_FLOW_RAD;
  }

  void observe(const mavlink_message_t& message, const MessageTime& time) override
  {
    if (message.msgid == MAVLINK_MSG_ID_OPTICAL_FLOW)
      accept(m_flow, message, time, MAVLINK_MSG_ID_OPTICAL_FLOW_MIN_LEN,
             MAVLINK_MSG_ID_OPTICAL_FLOW_LEN, mavlink_msg_optical_flow_decode);
    else
      accept(m_rad, message, time, MAVLINK_MSG_ID_OPTICAL_FLOW_RAD_MIN_LEN,
             MAVLINK_MSG_ID_OPTICAL_FLOW_RAD_LEN, mavlink_msg_optical_flow_rad_decode);
  }

  template<typename Payload>
  void accept(Family<Payload>& family, const mavlink_message_t& message, const MessageTime& time,
              unsigned min_length, unsigned max_length,
              void (*decode)(const mavlink_message_t*, Payload*))
  {
    Payload payload{};
    const bool shape = wireShape(message, min_length, max_length);
    if (shape)
      decode(&message, &payload);
    if (!shape || !validPayload(payload)) {
      rejectWire();
      ++family.invalid;
      return;
    }

    auto sample = family.samples.find(payload.sensor_id);
    if (sample == family.samples.end()) {
      if (family.samples.size() >= sensor_limit) {
        ++family.capacity_drops;
        return;
      }
      sample = family.samples.try_emplace(payload.sensor_id).first;
    }
    if (update(sample->second.stamp, time)) {
      sample->second.wire = {message.magic == MAVLINK_STX ? 2u : 1u, message.len};
      sample->second.payload = payload;
    }
  }

  void clearSamples() override
  {
    m_flow.samples.clear();
    m_rad.samples.clear();
  }

  void publishSamples() override
  {
    publish(m_flow, "OPTICAL_FLOW", MAVLINK_MSG_ID_OPTICAL_FLOW);
    publish(m_rad, "OPTICAL_FLOW_RAD", MAVLINK_MSG_ID_OPTICAL_FLOW_RAD);
  }

  template<typename Payload>
  void publish(const Family<Payload>& family, const char* name, unsigned id)
  {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(std::numeric_limits<float>::max_digits10) << std::boolalpha
        << "{\"schema\":1,\"family\":\"" << name << "\",\"message_id\":" << id
        << ",\"present\":" << !family.samples.empty()
        << ",\"read_only\":true,\"authoritative_for_control\":false,\"atomic\":false"
        << ",\"axis_basis\":\"sensor_wire\",\"sensor_limit\":" << sensor_limit
        << ",\"invalid_samples\":" << family.invalid
        << ",\"capacity_drops\":" << family.capacity_drops << ",\"samples\":[";
    bool first = true;
    for (const auto& [sensor, sample] : family.samples) {
      if (!first) out << ',';
      first = false;
      out << "{\"sensor_id\":" << sensor << ",\"wire_version\":" << sample.wire.version
          << ",\"payload_length\":" << sample.wire.length
          << ",\"metadata\":" << metadata(sample.stamp) << ",\"data\":";
      writeData(out, sample.payload, sample.wire);
      out << '}';
    }
    out << "]}";
    services().publish(std::string("MAVMOOS_") + name, out.str());
  }

  Family<mavlink_optical_flow_t> m_flow;
  Family<mavlink_optical_flow_rad_t> m_rad;
};

} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi, mavmoos::runtime::Services* services)
{
  return abi == mavmoos::runtime::plugin_abi && services ? new mavmoos::runtime::OpticalFlow(*services) : nullptr;
}

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin)
{
  delete plugin;
}
