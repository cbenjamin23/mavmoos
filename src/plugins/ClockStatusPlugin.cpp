/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ClockStatusPlugin.cpp                           */
/*    DATE: November 6, 2026                                */
/************************************************************/
#include "PassiveObserver.h"

#include <array>
#include <cstddef>

namespace mavmoos::runtime {
namespace {

struct Family {
  unsigned id;
  const char* name;
  const char* variable;
  const char* capability;
};

enum SampleIndex : std::size_t { SystemTime, TimesyncRequest, TimesyncReply };
constexpr std::array<Family, 3> families{{
  {MAVLINK_MSG_ID_SYSTEM_TIME, "SYSTEM_TIME", "MAVMOOS_SYSTEM_TIME", "clock_status.system_time"},
  {MAVLINK_MSG_ID_TIMESYNC, "TIMESYNC", "MAVMOOS_TIMESYNC_REQUEST", "clock_status.timesync_request"},
  {MAVLINK_MSG_ID_TIMESYNC, "TIMESYNC", "MAVMOOS_TIMESYNC_REPLY", "clock_status.timesync_reply"},
}};

std::string systemTimeData(const mavlink_system_time_t& value)
{
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::boolalpha << "{\"time_boot_ms\":" << value.time_boot_ms
      << ",\"time_unix_usec_raw\":\"" << value.time_unix_usec << "\""
      << ",\"utc_reported\":" << (value.time_unix_usec != 0) << ",\"time_unix_usec\":";
  if (value.time_unix_usec)
    out << '"' << value.time_unix_usec << '"';
  else
    out << "null";
  out << '}';
  return out.str();
}

const char* targetScope(const mavlink_timesync_t& value)
{
  if (value.target_system)
    return value.target_component ? "system_component" : "system_all_components";
  return value.target_component ? "component_without_system" : "broadcast_or_unspecified";
}

std::string timesyncData(const mavlink_timesync_t& value, unsigned length)
{
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << "{\"packet_kind\":\"" << (value.tc1 == 0 ? "request" : "reply")
      << "\",\"tc1_ns_raw\":\"" << value.tc1 << "\",\"ts1_ns_raw\":\"" << value.ts1
      << "\",\"timestamp_domains\":\"unqualified\",\"target_scope\":\"" << targetScope(value)
      << "\",\"target_system_raw\":";
  if (length > offsetof(mavlink_timesync_t, target_system))
    out << unsigned(value.target_system);
  else
    out << "null";
  out << ",\"target_component_raw\":";
  if (length > offsetof(mavlink_timesync_t, target_component))
    out << unsigned(value.target_component);
  else
    out << "null";
  out << '}';
  return out.str();
}

class ClockStatus final : public PassiveObserver {
public:
  explicit ClockStatus(Services& services) : PassiveObserver(services, true) {}

  std::string name() const override { return "clock_status"; }

  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.profile = "mavlink_clock_observations";
    for (const auto& family : families)
      result.capabilities.push_back({family.capability, "", "", "observation"});
    result.exclusive_resources = {"observation.clock_status"};
    result.streamed_parameter_bench_compatible = true;
    return result;
  }

private:
  struct Sample {
    Stamp stamp;
    unsigned wire_version = 0;
    unsigned length = 0;
    std::string data;
  };

  bool observes(uint32_t id) const override
  {
    return id == MAVLINK_MSG_ID_SYSTEM_TIME || id == MAVLINK_MSG_ID_TIMESYNC;
  }

  void observe(const mavlink_message_t& message, const MessageTime& time) override
  {
    if (message.msgid == MAVLINK_MSG_ID_SYSTEM_TIME) {
      if (!wireShape(message, MAVLINK_MSG_ID_SYSTEM_TIME_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_TIME_LEN)) {
        rejectWire();
        ++m_system_time_invalid;
        return;
      }
      mavlink_system_time_t value{};
      mavlink_msg_system_time_decode(&message, &value);
      auto& sample = m_samples[SystemTime];
      if (update(sample.stamp, time)) {
        sample.wire_version = message.magic == MAVLINK_STX ? 2u : 1u;
        sample.length = message.len;
        sample.data = systemTimeData(value);
      }
      return;
    }
    if (!wireShape(message, MAVLINK_MSG_ID_TIMESYNC_MIN_LEN, MAVLINK_MSG_ID_TIMESYNC_LEN)) {
      rejectWire();
      ++m_timesync_invalid;
      return;
    }
    mavlink_timesync_t value{};
    mavlink_msg_timesync_decode(&message, &value);
    auto& sample = m_samples[value.tc1 == 0 ? TimesyncRequest : TimesyncReply];
    if (update(sample.stamp, time)) {
      sample.wire_version = message.magic == MAVLINK_STX ? 2u : 1u;
      sample.length = message.len;
      sample.data = timesyncData(value, message.len);
    }
  }

  void clearSamples() override { m_samples = {}; }

  void publishSamples() override
  {
    for (std::size_t i = 0; i < families.size(); ++i) {
      const auto& family = families[i];
      const auto& sample = m_samples[i];
      auto record = metadata(sample.stamp);
      record.pop_back();
      record += ",\"family\":\"" + std::string(family.name) + "\",\"message_id\":" + std::to_string(family.id) +
        ",\"invalid_family_frames\":" + std::to_string(i == SystemTime ? m_system_time_invalid : m_timesync_invalid) +
        ",\"wire_version\":" + (sample.stamp.id ? std::to_string(sample.wire_version) : "null") +
        ",\"payload_length\":" + (sample.stamp.id ? std::to_string(sample.length) : "null") +
        ",\"data\":" + (sample.stamp.id ? sample.data : "null") + '}';
      services().publish(family.variable, record);
    }
  }

  std::array<Sample, families.size()> m_samples{};
  uint64_t m_system_time_invalid = 0;
  uint64_t m_timesync_invalid = 0;
};

} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi, mavmoos::runtime::Services* services)
{
  return abi == mavmoos::runtime::plugin_abi && services ? new mavmoos::runtime::ClockStatus(*services) : nullptr;
}

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin)
{
  delete plugin;
}
