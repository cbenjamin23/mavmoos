/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: EscStatusPlugin.cpp                             */
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

namespace mavmoos::runtime {
namespace {

constexpr unsigned group_width = 4;
constexpr unsigned common_groups = 16;

bool validIndex(unsigned index)
{
  return index < common_groups * group_width && index % group_width == 0;
}

void writeFinite(std::ostream& out, float value)
{
  if (std::isfinite(value))
    out << value;
  else
    out << "null";
}

void writeInfo(std::ostream& out, const mavlink_esc_info_t& report)
{
  out << ",\"time_usec_raw\":\"" << report.time_usec << '"'
      << ",\"counter\":" << report.counter
      << ",\"count\":" << unsigned(report.count)
      << ",\"connection_type\":" << unsigned(report.connection_type)
      << ",\"info_raw\":" << unsigned(report.info)
      << ",\"entries\":[";
  const auto entries = std::min(group_width, unsigned(report.count) - report.index);
  for (unsigned i = 0; i < entries; ++i) {
    if (i != 0)
      out << ',';
    out << "{\"index\":" << report.index + i
        << ",\"error_count\":" << report.error_count[i]
        << ",\"failure_flags\":" << report.failure_flags[i]
        << ",\"temperature_cdeg\":";
    if (report.temperature[i] == std::numeric_limits<int16_t>::max())
      out << "null";
    else
      out << report.temperature[i];
    out << '}';
  }
  out << ']';
}

void writeStatus(std::ostream& out, const mavlink_esc_status_t& report)
{
  out << ",\"time_usec_raw\":\"" << report.time_usec << "\",\"entries\":[";
  for (unsigned i = 0; i < group_width; ++i) {
    if (i != 0)
      out << ',';
    out << "{\"index\":" << report.index + i << ",\"rpm\":" << report.rpm[i]
        << ",\"voltage_v\":";
    writeFinite(out, report.voltage[i]);
    out << ",\"current_a\":";
    writeFinite(out, report.current[i]);
    out << '}';
  }
  out << ']';
}

void writeTelemetry(std::ostream& out, const mavlink_esc_telemetry_1_to_4_t& report)
{
  out << ",\"entries\":[";
  for (unsigned i = 0; i < group_width; ++i) {
    if (i != 0)
      out << ',';
    out << "{\"index\":" << i << ",\"voltage_cv\":" << report.voltage[i]
        << ",\"current_ca\":" << report.current[i]
        << ",\"total_current_mah\":" << report.totalcurrent[i]
        << ",\"rpm_raw\":" << report.rpm[i]
        << ",\"packet_count\":" << report.count[i]
        << ",\"temperature_c\":" << unsigned(report.temperature[i]) << '}';
  }
  out << ']';
}

class EscStatus final : public PassiveObserver {
public:
  explicit EscStatus(Services& services) : PassiveObserver(services) {}

  std::string name() const override { return "esc_status"; }

  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.profile = "ardupilot_esc_observations";
    result.capabilities = {
      {"observations.esc_info", "", "", "observation"},
      {"observations.esc_status", "", "", "observation"},
      {"observations.esc_telemetry", "", "", "observation"},
    };
    result.exclusive_resources = {"observation.esc"};
    result.streamed_parameter_bench_compatible = true;
    return result;
  }

private:
  template<typename Payload>
  struct Sample {
    Stamp stamp;
    Payload report{};
  };

  bool observes(uint32_t id) const override
  {
    return id == MAVLINK_MSG_ID_ESC_INFO || id == MAVLINK_MSG_ID_ESC_STATUS ||
           id == MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4;
  }

  void observe(const mavlink_message_t& message, const MessageTime& time) override
  {
    // All three IDs require MAVLink 2. The generated decoders zero-fill trimmed tails.
    switch (message.msgid) {
      case MAVLINK_MSG_ID_ESC_INFO: {
        if (!wireShape(message, 0, MAVLINK_MSG_ID_ESC_INFO_LEN)) {
          rejectWire();
          return;
        }
        mavlink_esc_info_t report{};
        mavlink_msg_esc_info_decode(&message, &report);
        // A zero count can explicitly describe an empty source at index zero.
        if (!validIndex(report.index) || report.count > common_groups * group_width ||
            (report.count == 0 ? report.index != 0 : report.index >= report.count)) {
          rejectWire();
          return;
        }
        save(m_info[report.index / group_width], report, time);
        break;
      }
      case MAVLINK_MSG_ID_ESC_STATUS: {
        if (!wireShape(message, 0, MAVLINK_MSG_ID_ESC_STATUS_LEN)) {
          rejectWire();
          return;
        }
        mavlink_esc_status_t report{};
        mavlink_msg_esc_status_decode(&message, &report);
        if (!validIndex(report.index)) {
          rejectWire();
          return;
        }
        save(m_status[report.index / group_width], report, time);
        break;
      }
      case MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4: {
        if (!wireShape(message, 0, MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_LEN)) {
          rejectWire();
          return;
        }
        mavlink_esc_telemetry_1_to_4_t report{};
        mavlink_msg_esc_telemetry_1_to_4_decode(&message, &report);
        save(m_telemetry.front(), report, time);
        break;
      }
    }
  }

  template<typename Payload>
  void save(Sample<Payload>& sample, const Payload& report, const MessageTime& time)
  {
    if (update(sample.stamp, time))
      sample.report = report;
  }

  void clearSamples() override
  {
    m_info = {};
    m_status = {};
    m_telemetry = {};
  }

  void publishSamples() override
  {
    publishGroups("MAVMOOS_ESC_INFO", MAVLINK_MSG_ID_ESC_INFO, m_info, writeInfo);
    publishGroups("MAVMOOS_ESC_STATUS", MAVLINK_MSG_ID_ESC_STATUS, m_status, writeStatus);
    publishGroups("MAVMOOS_ESC_TELEMETRY", MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4,
                  m_telemetry, writeTelemetry);
  }

  template<typename Payload, std::size_t Size>
  void publishGroups(const std::string& key, unsigned message_id,
                     const std::array<Sample<Payload>, Size>& cache,
                     void (*write)(std::ostream&, const Payload&))
  {
    const auto present = std::any_of(cache.begin(), cache.end(), [](const auto& group) {
      return group.stamp.id != 0;
    });
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(std::numeric_limits<float>::max_digits10) << std::boolalpha
        << "{\"schema\":1,\"present\":" << present
        << ",\"read_only\":true,\"authoritative_for_control\":false,\"atomic\":false"
        << ",\"device_presence\":\"unknown\",\"health\":\"unassessed\",\"groups\":[";
    bool first = true;
    for (std::size_t group = 0; group < cache.size(); ++group) {
      const auto& sample = cache[group];
      if (sample.stamp.id == 0)
        continue;
      if (!first)
        out << ',';
      first = false;
      out << "{\"first_index\":" << group * group_width << ",\"message_id\":" << message_id
          << ",\"metadata\":" << metadata(sample.stamp);
      write(out, sample.report);
      out << '}';
    }
    out << "]}";
    services().publish(key, out.str());
  }

  std::array<Sample<mavlink_esc_info_t>, common_groups> m_info{};
  std::array<Sample<mavlink_esc_status_t>, common_groups> m_status{};
  std::array<Sample<mavlink_esc_telemetry_1_to_4_t>, 1> m_telemetry{};
};

} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi,
                                                         mavmoos::runtime::Services* services)
{
  if (abi != mavmoos::runtime::plugin_abi || !services)
    return nullptr;
  return new mavmoos::runtime::EscStatus(*services);
}

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin)
{
  delete plugin;
}
