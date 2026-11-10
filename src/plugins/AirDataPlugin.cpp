/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: AirDataPlugin.cpp                               */
/*    DATE: November 5, 2026                                */
/************************************************************/
#include "PassiveObserver.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <utility>

namespace mavmoos::runtime {
namespace {

std::ostringstream jsonNumbers()
{
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<float>::max_digits10);
  return out;
}

std::string decodeHud(const mavlink_message_t& message)
{
  mavlink_vfr_hud_t hud{};
  mavlink_msg_vfr_hud_decode(&message, &hud);
  if(!std::isfinite(hud.airspeed) || hud.airspeed < 0 ||
     !std::isfinite(hud.groundspeed) || hud.groundspeed < 0 ||
     !std::isfinite(hud.alt) || !std::isfinite(hud.climb) ||
     hud.heading < 0 || hud.heading > 360 || hud.throttle > 100)
    return {};
  auto out = jsonNumbers();
  out << "{\"airspeed_reported_m_s\":" << hud.airspeed
      << ",\"airspeed_basis\":\"fc_reported_unspecified\""
      << ",\"groundspeed_m_s\":" << hud.groundspeed
      << ",\"heading_deg\":" << hud.heading
      << ",\"throttle_pct\":" << hud.throttle
      << ",\"alt_msl_m\":" << hud.alt
      << ",\"climb_m_s\":" << hud.climb << '}';
  return out.str();
}

std::string decodeWind(const mavlink_message_t& message)
{
  mavlink_wind_t wind{};
  mavlink_msg_wind_decode(&message, &wind);
  // Copter uses a signed atan2 bearing and reports the full vector length.
  if(!std::isfinite(wind.direction) || wind.direction < -180 || wind.direction > 360 ||
     !std::isfinite(wind.speed) || wind.speed < 0 || !std::isfinite(wind.speed_z))
    return {};
  auto out = jsonNumbers();
  out << "{\"direction_from_deg\":" << wind.direction
      << ",\"speed_reported_m_s\":" << wind.speed
      << ",\"speed_z_reported_m_s\":" << wind.speed_z << '}';
  return out.str();
}

void nullable(std::ostream& out, float value, bool zero_is_unknown = false)
{
  if(std::isnan(value) || (zero_is_unknown && value == 0))
    out << "null";
  else
    out << value;
}

std::string decodeWindCov(const mavlink_message_t& message)
{
  mavlink_wind_cov_t wind{};
  mavlink_msg_wind_cov_decode(&message, &wind);
  // MAVLink reserves NaN for the vector, variability and altitude unknowns.
  // A zero accuracy field is unknown, while zero wind is a valid estimate.
  if(std::isinf(wind.wind_x) || std::isinf(wind.wind_y) || std::isinf(wind.wind_z) ||
     std::isinf(wind.var_horiz) || wind.var_horiz < 0 ||
     std::isinf(wind.var_vert) || wind.var_vert < 0 || std::isinf(wind.wind_alt) ||
     !std::isfinite(wind.horiz_accuracy) || wind.horiz_accuracy < 0 ||
     !std::isfinite(wind.vert_accuracy) || wind.vert_accuracy < 0)
    return {};
  auto out = jsonNumbers();
  out << "{\"frame\":\"NED\",\"time_usec_raw\":\"" << wind.time_usec << "\",\"north_m_s\":";
  nullable(out, wind.wind_x);
  out << ",\"east_m_s\":";
  nullable(out, wind.wind_y);
  out << ",\"down_m_s\":";
  nullable(out, wind.wind_z);
  out << ",\"horizontal_variability_std_m_s\":";
  nullable(out, wind.var_horiz);
  out << ",\"vertical_variability_std_m_s\":";
  nullable(out, wind.var_vert);
  out << ",\"alt_msl_m\":";
  nullable(out, wind.wind_alt);
  out << ",\"horizontal_accuracy_std_m_s\":";
  nullable(out, wind.horiz_accuracy, true);
  out << ",\"vertical_accuracy_std_m_s\":";
  nullable(out, wind.vert_accuracy, true);
  out << '}';
  return out.str();
}

void writeAirspeed(std::ostream& out, const mavlink_airspeed_t& packet)
{
  constexpr uint8_t known_flags = AIRSPEED_SENSOR_UNHEALTHY | AIRSPEED_SENSOR_USING;
  out << "{\"airspeed_reported_cas_m_s\":" << packet.airspeed << ",\"temperature_cdeg\":";
  if(packet.temperature == std::numeric_limits<int16_t>::max())
    out << "null";
  else
    out << packet.temperature;
  out << ",\"raw_pressure_wire\":";
  nullable(out, packet.raw_press);
  // The 4.6.3 ArduPilot sender puts pascals directly on a wire field declared hPa.
  out << ",\"raw_pressure_mavlink_unit\":\"hPa\",\"differential_pressure_hpa_if_pinned_ardupilot_pa\":";
  if(std::isnan(packet.raw_press))
    out << "null";
  else
    out << static_cast<double>(packet.raw_press) / 100;
  out << ",\"flags_raw\":" << unsigned(packet.flags)
      << ",\"sensor_unhealthy_flag\":" << bool(packet.flags & AIRSPEED_SENSOR_UNHEALTHY)
      << ",\"sensor_used_flag\":" << bool(packet.flags & AIRSPEED_SENSOR_USING)
      << ",\"unknown_flags_raw\":" << unsigned(packet.flags & ~known_flags) << '}';
}

std::string decodeHighLatency(const mavlink_message_t& message)
{
  mavlink_high_latency2_t packet{};
  mavlink_msg_high_latency2_decode(&message, &packet);
  if(packet.type != MAV_TYPE_FIXED_WING || packet.autopilot != MAV_AUTOPILOT_ARDUPILOTMEGA)
    return {};
  auto out = jsonNumbers();
  out << "{\"timestamp_ms_raw\":" << packet.timestamp
      << ",\"type_raw\":" << unsigned(packet.type)
      << ",\"autopilot_raw\":" << unsigned(packet.autopilot)
      << ",\"airspeed_raw\":" << unsigned(packet.airspeed)
      << ",\"airspeed_setpoint_raw\":" << unsigned(packet.airspeed_sp)
      << ",\"airspeed_reported_m_s\":" << packet.airspeed / 5.0
      << ",\"airspeed_setpoint_reported_m_s\":" << packet.airspeed_sp / 5.0
      << ",\"airspeed_basis\":\"fc_reported_unspecified\""
      << ",\"airspeed_availability\":\"unknown\",\"airspeed_setpoint_availability\":\"unknown\"}";
  return out.str();
}

template<typename Pressure, void (*decode)(const mavlink_message_t*, Pressure*)>
std::string decodeScaledPressure(const mavlink_message_t& message)
{
  Pressure pressure{};
  decode(&message, &pressure);
  if(!std::isfinite(pressure.press_abs) || pressure.press_abs < 0 ||
     !std::isfinite(pressure.press_diff))
    return {};

  // ArduPilot supplies zero fields for a missing half of an instance. A zero
  // differential pressure can also be a measurement. Wire value 1 for its
  // temperature represents either zero degrees or one centidegree.
  const bool diff_temperature_reported = message.magic == MAVLINK_STX &&
    message.len > MAVLINK_MSG_ID_SCALED_PRESSURE_MIN_LEN && pressure.temperature_press_diff != 0;
  auto out = jsonNumbers();
  out << "{\"time_boot_ms\":" << pressure.time_boot_ms << ",\"press_abs_hpa\":";
  nullable(out, pressure.press_abs, true);
  out << ",\"press_diff_hpa\":" << pressure.press_diff
      << ",\"press_diff_availability\":\""
      << (pressure.press_diff == 0 ? "zero_ambiguous" : "nonzero_reported") << '"'
      << ",\"temperature_cdeg\":";
  if(pressure.press_abs == 0 && pressure.temperature == 0)
    out << "null";
  else
    out << pressure.temperature;
  out << ",\"temperature_press_diff_cdeg_raw\":";
  if(diff_temperature_reported)
    out << pressure.temperature_press_diff;
  else
    out << "null";
  out << ",\"temperature_press_diff_encoding\":\""
      << (!diff_temperature_reported ? "unknown" :
          pressure.temperature_press_diff == 1 ? "zero_or_one_cdeg" : "centidegrees")
      << "\"}";
  return out.str();
}

struct Family {
  uint32_t id;
  uint8_t min_payload_length;
  uint8_t max_payload_length;
  const char* name;
  const char* variable;
  std::string (*decode)(const mavlink_message_t&);
};

constexpr std::array<Family, 7> families{{
  {MAVLINK_MSG_ID_VFR_HUD, MAVLINK_MSG_ID_VFR_HUD_MIN_LEN, MAVLINK_MSG_ID_VFR_HUD_LEN,
   "VFR_HUD", "MAVMOOS_VFR_HUD", decodeHud},
  {MAVLINK_MSG_ID_WIND, MAVLINK_MSG_ID_WIND_MIN_LEN, MAVLINK_MSG_ID_WIND_LEN,
   "WIND", "MAVMOOS_WIND", decodeWind},
  {MAVLINK_MSG_ID_WIND_COV, MAVLINK_MSG_ID_WIND_COV_MIN_LEN, MAVLINK_MSG_ID_WIND_COV_LEN,
   "WIND_COV", "MAVMOOS_WIND_COV", decodeWindCov},
  {MAVLINK_MSG_ID_SCALED_PRESSURE, MAVLINK_MSG_ID_SCALED_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE_LEN,
   "SCALED_PRESSURE", "MAVMOOS_SCALED_PRESSURE", decodeScaledPressure<mavlink_scaled_pressure_t, mavlink_msg_scaled_pressure_decode>},
  {MAVLINK_MSG_ID_SCALED_PRESSURE2, MAVLINK_MSG_ID_SCALED_PRESSURE2_MIN_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE2_LEN,
   "SCALED_PRESSURE2", "MAVMOOS_SCALED_PRESSURE2", decodeScaledPressure<mavlink_scaled_pressure2_t, mavlink_msg_scaled_pressure2_decode>},
  {MAVLINK_MSG_ID_SCALED_PRESSURE3, MAVLINK_MSG_ID_SCALED_PRESSURE3_MIN_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN,
   "SCALED_PRESSURE3", "MAVMOOS_SCALED_PRESSURE3", decodeScaledPressure<mavlink_scaled_pressure3_t, mavlink_msg_scaled_pressure3_decode>},
  {MAVLINK_MSG_ID_HIGH_LATENCY2, MAVLINK_MSG_ID_HIGH_LATENCY2_MIN_LEN, MAVLINK_MSG_ID_HIGH_LATENCY2_LEN,
   "HIGH_LATENCY2", "MAVMOOS_HIGH_LATENCY_AIR_DATA", decodeHighLatency},
}};

constexpr std::size_t airspeed_sensor_limit = 2;

std::size_t familyIndex(uint32_t id)
{
  for(std::size_t i = 0; i < families.size(); ++i)
    if(families[i].id == id) return i;
  return families.size();
}

class AirData final : public PassiveObserver {
public:
  explicit AirData(Services& host) : PassiveObserver(host) {}

  std::string name() const override { return "air_data"; }

  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.profile = "ardupilot_air_data_observations";
    result.capabilities = {{"air_data.vfr_hud", "", "", "observation"},
                           {"air_data.wind", "", "", "observation"},
                           {"air_data.wind_cov", "", "", "observation"},
                           {"air_data.scaled_pressure", "", "", "observation"},
                           {"air_data.airspeed", "", "", "observation"},
                           {"air_data.high_latency2", "", "", "observation"}};
    result.exclusive_resources = {"observation.air_data"};
    result.streamed_parameter_bench_compatible = true;
    return result;
  }

protected:
  bool observes(uint32_t id) const override
  {
    return id == MAVLINK_MSG_ID_AIRSPEED || familyIndex(id) < families.size();
  }

  void observe(const mavlink_message_t& message, const MessageTime& time) override
  {
    if(message.msgid == MAVLINK_MSG_ID_AIRSPEED || message.msgid == MAVLINK_MSG_ID_HIGH_LATENCY2) {
      if(services().vehicle().type != MAV_TYPE_FIXED_WING) return;
      if(message.msgid == MAVLINK_MSG_ID_AIRSPEED) {
        observeAirspeed(message, time);
        return;
      }
    }
    const auto i = familyIndex(message.msgid);
    if(i == families.size()) return;
    const auto& family = families[i];
    auto data = wireShape(message, family.min_payload_length, family.max_payload_length)
      ? family.decode(message) : std::string{};
    if(data.empty()) {
      rejectWire();
      ++m_rejected[i];
      return;
    }
    auto& sample = m_samples[i];
    if(update(sample.stamp, time)) sample.data = std::move(data);
  }

  void clearSamples() override
  {
    m_samples = {};
    m_airspeed = {};
  }

  void publishSamples() override
  {
    for(std::size_t i = 0; i < families.size(); ++i) {
      const auto& family = families[i];
      const auto& sample = m_samples[i];
      auto result = metadata(sample.stamp);
      result.pop_back();
      result += ",\"message_id\":" + std::to_string(family.id) +
                ",\"family\":\"" + family.name + "\",\"invalid_samples\":" + std::to_string(m_rejected[i]) +
                ",\"data\":" + (sample.data.empty() ? "null" : sample.data) + '}';
      services().publish(family.variable, result);
    }
    publishAirspeed();
  }

private:
  struct Sample {
    Stamp stamp;
    std::string data;
  };

  struct AirspeedSample {
    Stamp stamp;
    mavlink_airspeed_t packet{};
    uint8_t payload_length = 0;
  };

  void observeAirspeed(const mavlink_message_t& message, const MessageTime& time)
  {
    if(wireShape(message, MAVLINK_MSG_ID_AIRSPEED_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_LEN)) {
      mavlink_airspeed_t packet{};
      mavlink_msg_airspeed_decode(&message, &packet);
      if(packet.id < m_airspeed.size() && std::isfinite(packet.airspeed) && packet.airspeed >= 0 &&
         !std::isinf(packet.raw_press)) {
        auto& sample = m_airspeed[packet.id];
        if(update(sample.stamp, time)) {
          sample.packet = packet;
          sample.payload_length = message.len;
        }
        return;
      }
    }
    rejectWire();
    ++m_airspeed_rejected;
  }

  void publishAirspeed()
  {
    const bool present = std::any_of(m_airspeed.begin(), m_airspeed.end(),
                                     [](const auto& sample) { return sample.stamp.id != 0; });
    auto out = jsonNumbers();
    out << std::boolalpha << "{\"schema\":1,\"family\":\"AIRSPEED\",\"message_id\":" << MAVLINK_MSG_ID_AIRSPEED
        << ",\"present\":" << present << ",\"read_only\":true,\"authoritative_for_control\":false,\"atomic\":false"
        << ",\"sensor_limit\":" << m_airspeed.size() << ",\"invalid_samples\":" << m_airspeed_rejected
        << ",\"samples\":[";
    bool first = true;
    for(const auto& sample : m_airspeed) {
      if(!sample.stamp.id) continue;
      if(!first) out << ',';
      first = false;
      out << "{\"sensor_id\":" << unsigned(sample.packet.id) << ",\"wire_version\":2,\"payload_length\":"
          << unsigned(sample.payload_length) << ",\"metadata\":" << metadata(sample.stamp) << ",\"data\":";
      writeAirspeed(out, sample.packet);
      out << '}';
    }
    out << "]}";
    services().publish("MAVMOOS_AIRSPEED", out.str());
  }

  std::array<Sample, families.size()> m_samples{};
  std::array<uint64_t, families.size()> m_rejected{};
  std::array<AirspeedSample, airspeed_sensor_limit> m_airspeed{};
  uint64_t m_airspeed_rejected = 0;
};

} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi, mavmoos::runtime::Services* host)
{
  return abi == mavmoos::runtime::plugin_abi && host ? new mavmoos::runtime::AirData(*host) : nullptr;
}

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin)
{
  delete plugin;
}
