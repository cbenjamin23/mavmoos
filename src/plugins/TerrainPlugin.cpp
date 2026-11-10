/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: TerrainPlugin.cpp                               */
/*    DATE: November 6, 2026                                 */
/************************************************************/
#include "PassiveObserver.h"
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <utility>

namespace mavmoos::runtime {
namespace {
constexpr uint64_t grid_mask = (uint64_t{1} << 56) - 1;

bool coordinatesValid(int32_t latitude, int32_t longitude)
{
  return latitude >= -900000000 && latitude <= 900000000 &&
         longitude >= -1800000000 && longitude <= 1800000000;
}

void appendCoordinates(std::ostream& out, int32_t latitude, int32_t longitude)
{
  out << std::fixed << std::setprecision(7)
      << "\"latitude_deg\":" << latitude / 1e7 << ",\"longitude_deg\":" << longitude / 1e7
      << std::defaultfloat << std::setprecision(std::numeric_limits<float>::max_digits10)
      << ",\"horizontal_datum\":\"WGS84\"";
}

std::string decodeReport(const mavlink_message_t& message)
{
  mavlink_terrain_report_t report{};
  mavlink_msg_terrain_report_decode(&message, &report);
  if (!coordinatesValid(report.lat, report.lon) ||
      !std::isfinite(report.terrain_height) || !std::isfinite(report.current_height))
    return {};

  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << '{';
  appendCoordinates(out, report.lat, report.lon);
  out << ",\"terrain_vertical_datum\":\"MSL\",\"spacing_m\":" << report.spacing
      << ",\"terrain_available\":" << std::boolalpha << (report.spacing != 0)
      << ",\"terrain_height_msl_m\":";
  if (report.spacing) out << report.terrain_height;
  else out << "null";
  // Pinned Copter may supply an extrapolated height with zero spacing and can
  // compute current_height at the vehicle even for a different reported point.
  out << ",\"terrain_height_reported_msl_m\":" << report.terrain_height
      << ",\"current_height_reported_m\":" << report.current_height
      << ",\"current_height_validity\":\"unspecified\""
      << ",\"pending_blocks\":" << report.pending << ",\"loaded_blocks\":" << report.loaded << '}';
  return out.str();
}

std::string decodeRequest(const mavlink_message_t& message)
{
  mavlink_terrain_request_t request{};
  mavlink_msg_terrain_request_decode(&message, &request);
  if (!coordinatesValid(request.lat, request.lon) || request.grid_spacing == 0 || (request.mask & ~grid_mask))
    return {};
  unsigned count = 0;
  for (uint64_t remaining = request.mask; remaining; remaining &= remaining - 1) ++count;

  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << '{';
  appendCoordinates(out, request.lat, request.lon);
  out << ",\"coordinate_role\":\"southwest_grid_origin\",\"spacing_m\":" << request.grid_spacing
      << ",\"requested_blocks\":" << count << ",\"grid_mask_hex\":\"0x"
      << std::hex << std::setfill('0') << std::setw(14) << request.mask << "\"}";
  return out.str();
}

struct Family {
  uint32_t id;
  unsigned length;
  const char* message_name;
  const char* variable;
  std::string (*decode)(const mavlink_message_t&);
};
constexpr std::array<Family, 2> families{{
  {MAVLINK_MSG_ID_TERRAIN_REPORT, MAVLINK_MSG_ID_TERRAIN_REPORT_LEN,
   "TERRAIN_REPORT", "MAVMOOS_TERRAIN_REPORT", decodeReport},
  {MAVLINK_MSG_ID_TERRAIN_REQUEST, MAVLINK_MSG_ID_TERRAIN_REQUEST_LEN,
   "TERRAIN_REQUEST", "MAVMOOS_TERRAIN_REQUEST_OBSERVED", decodeRequest},
}};

class Terrain final : public PassiveObserver {
public:
  explicit Terrain(Services& host) : PassiveObserver(host, true, 3) {}
  std::string name() const override { return "terrain"; }
  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.profile = "mavlink_terrain_observations";
    result.capabilities = {{"terrain.report", "", "", "observation"},
                           {"terrain.request_observed", "", "", "observation"}};
    result.exclusive_resources = {"observation.terrain"};
    result.streamed_parameter_bench_compatible = true;
    return result;
  }

protected:
  bool observes(uint32_t id) const override
  { return id == MAVLINK_MSG_ID_TERRAIN_REPORT || id == MAVLINK_MSG_ID_TERRAIN_REQUEST; }

  void observe(const mavlink_message_t& message, const MessageTime& time) override
  {
    const std::size_t index = message.msgid == MAVLINK_MSG_ID_TERRAIN_REPORT ? 0 : 1;
    const auto& family = families[index];
    auto data = wireShape(message, family.length, family.length) ? family.decode(message) : std::string{};
    if (data.empty()) {
      rejectWire();
      ++m_rejected[index];
      return;
    }
    auto& sample = m_samples[index];
    if (update(sample.stamp, time)) sample.data = std::move(data);
  }

  void clearSamples() override { m_samples = {}; }
  void publishSamples() override
  {
    for (std::size_t index = 0; index < families.size(); ++index) {
      const auto& family = families[index];
      const auto& sample = m_samples[index];
      auto result = metadata(sample.stamp);
      result.pop_back();
      result += ",\"message_id\":" + std::to_string(family.id) +
                ",\"family\":\"" + family.message_name + "\",\"invalid_samples\":" +
                std::to_string(m_rejected[index]) + ",\"data\":" +
                (sample.data.empty() ? "null" : sample.data) + '}';
      services().publish(family.variable, result);
    }
  }

private:
  struct Sample {
    Stamp stamp;
    std::string data;
  };
  std::array<Sample, families.size()> m_samples{};
  std::array<uint64_t, families.size()> m_rejected{};
};
} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi, mavmoos::runtime::Services* host)
{
  return abi == mavmoos::runtime::plugin_abi && host ? new mavmoos::runtime::Terrain(*host) : nullptr;
}

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin)
{
  delete plugin;
}
