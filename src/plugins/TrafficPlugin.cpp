/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: TrafficPlugin.cpp                               */
/*    DATE: November 6, 2026                                */
/************************************************************/
#include "PassiveObserver.h"

#include <algorithm>
#include <map>
#include <string_view>

namespace mavmoos::runtime {
namespace {

constexpr std::size_t contact_capacity = 32;
constexpr uint32_t max_icao = 0x00ffffff;

void writeNumber(std::ostream& out, double value, bool valid)
{
  if (valid)
    out << value;
  else
    out << "null";
}

void writeCallsign(std::ostream& out, const mavlink_adsb_vehicle_t& report)
{
  const std::string_view bytes(report.callsign, sizeof(report.callsign));
  const auto end = bytes.find('\0');
  const auto name = bytes.substr(0, end);
  const bool printable = std::all_of(name.begin(), name.end(), [](unsigned char c) {
    return c >= 32 && c <= 126;
  });
  if (!(report.flags & ADSB_FLAGS_VALID_CALLSIGN) || end == bytes.npos ||
      name.find_first_not_of(' ') == name.npos || !printable) {
    out << "null";
    return;
  }
  out << '"';
  for (char c : name) {
    if (c == '"' || c == '\\')
      out << '\\';
    out << c;
  }
  out << '"';
}

bool validSquawk(unsigned code)
{
  return code / 1000 <= 7 && code / 100 % 10 <= 7 &&
         code / 10 % 10 <= 7 && code % 10 <= 7;
}

void writeReport(std::ostream& out, const mavlink_adsb_vehicle_t& report)
{
  const auto flags = report.flags;
  const bool coords = (flags & ADSB_FLAGS_VALID_COORDS) &&
                      report.lat >= -900000000 && report.lat <= 900000000 &&
                      report.lon >= -1800000000 && report.lon <= 1800000000;
  const auto altitude_type = report.altitude_type;
  const bool altitude = (flags & ADSB_FLAGS_VALID_ALTITUDE) && report.altitude != INT32_MAX &&
                        (altitude_type == ADSB_ALTITUDE_TYPE_PRESSURE_QNH ||
                         altitude_type == ADSB_ALTITUDE_TYPE_GEOMETRIC);
  out << ",\"flags_raw\":" << flags << ",\"simulated\":" << bool(flags & ADSB_FLAGS_SIMULATED)
      << ",\"reported_tslc_s\":" << unsigned(report.tslc)
      << ",\"emitter_type_raw\":" << unsigned(report.emitter_type)
      << ",\"latitude_deg\":";
  writeNumber(out, report.lat / 1e7, coords);
  out << ",\"longitude_deg\":";
  writeNumber(out, report.lon / 1e7, coords);
  out << ",\"altitude_type_raw\":" << unsigned(altitude_type)
      << ",\"altitude_m_asl\":";
  writeNumber(out, report.altitude / 1e3, altitude);
  out << ",\"course_deg\":";
  writeNumber(out, report.heading / 1e2,
              (flags & ADSB_FLAGS_VALID_HEADING) && report.heading < 36000);
  out << ",\"horizontal_velocity_m_s\":";
  writeNumber(out, report.hor_velocity / 1e2,
              (flags & ADSB_FLAGS_VALID_VELOCITY) && report.hor_velocity != UINT16_MAX);
  out << ",\"vertical_velocity_up_m_s\":";
  writeNumber(out, report.ver_velocity / 1e2,
              (flags & ADSB_FLAGS_VERTICAL_VELOCITY_VALID) && report.ver_velocity != INT16_MAX);
  out << ",\"callsign\":";
  writeCallsign(out, report);
  out << ",\"squawk_decimal\":";
  if ((flags & ADSB_FLAGS_VALID_SQUAWK) && validSquawk(report.squawk))
    out << report.squawk;
  else
    out << "null";
}

class Traffic final : public PassiveObserver {
public:
  explicit Traffic(Services& services) : PassiveObserver(services, false, 3) {}

  std::string name() const override { return "traffic"; }

  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.profile = "ardupilot_adsb_observations";
    result.capabilities = {{"observations.adsb_traffic", "", "", "observation"}};
    result.exclusive_resources = {"observation.traffic"};
    result.streamed_parameter_bench_compatible = true;
    return result;
  }

private:
  struct Contact {
    Stamp stamp;
    mavlink_adsb_vehicle_t report{};
  };

  bool observes(uint32_t id) const override { return id == MAVLINK_MSG_ID_ADSB_VEHICLE; }

  void observe(const mavlink_message_t& message, const MessageTime& time) override
  {
    if (!wireShape(message, MAVLINK_MSG_ID_ADSB_VEHICLE_MIN_LEN, MAVLINK_MSG_ID_ADSB_VEHICLE_LEN)) {
      rejectWire();
      return;
    }
    mavlink_adsb_vehicle_t report{};
    mavlink_msg_adsb_vehicle_decode(&message, &report);
    const uint32_t address = report.ICAO_address;
    if (address == 0 || address > max_icao) {
      rejectWire();
      return;
    }
    const auto now = services().now();
    expire(now);
    if (now - time.effective_at > sampleTimeout()) {
      ++m_stale_rejected;
      return;
    }
    auto contact = m_contacts.find(address);
    if (contact == m_contacts.end()) {
      if (m_contacts.size() == contact_capacity) {
        ++m_capacity_rejected;
        return;
      }
      contact = m_contacts.try_emplace(address).first;
    }
    if (update(contact->second.stamp, time))
      contact->second.report = report;
  }

  void expire(double now)
  {
    for (auto it = m_contacts.begin(); it != m_contacts.end();) {
      if (now - it->second.stamp.time.effective_at > sampleTimeout()) {
        it = m_contacts.erase(it);
        ++m_expired;
      } else {
        ++it;
      }
    }
  }

  void clearSamples() override { m_contacts.clear(); }

  void publishSamples() override
  {
    expire(services().now());
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(17) << std::boolalpha
        << "{\"schema\":1,\"present\":" << !m_contacts.empty()
        << ",\"read_only\":true,\"authoritative_for_control\":false,\"atomic\":false"
        << ",\"coverage\":\"unknown\",\"track_age\":\"unverified\""
        << ",\"contact_capacity\":" << contact_capacity << ",\"capacity_rejected\":" << m_capacity_rejected
        << ",\"expired\":" << m_expired << ",\"stale_ingress_rejected\":" << m_stale_rejected
        << ",\"contacts\":[";
    Stamp latest;
    for (const auto& [address, contact] : m_contacts) {
      if (latest.id)
        out << ',';
      if (contact.stamp.id > latest.id)
        latest = contact.stamp;
      out << "{\"icao_address\":" << address << ",\"metadata\":" << metadata(contact.stamp);
      writeReport(out, contact.report);
      out << '}';
    }
    out << "],\"latest_report\":" << metadata(latest) << '}';
    services().publish("MAVMOOS_TRAFFIC_CONTACTS", out.str());
  }

  std::map<uint32_t, Contact> m_contacts;
  uint64_t m_capacity_rejected = 0;
  uint64_t m_expired = 0;
  uint64_t m_stale_rejected = 0;
};

} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi,
                                                         mavmoos::runtime::Services* services)
{
  if (abi != mavmoos::runtime::plugin_abi || !services)
    return nullptr;
  return new mavmoos::runtime::Traffic(*services);
}

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin)
{
  delete plugin;
}
