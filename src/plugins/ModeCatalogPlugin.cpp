/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ModeCatalogPlugin.cpp                           */
/*    DATE: November 6, 2026                                */
/************************************************************/
#include "PassiveObserver.h"

#include <algorithm>
#include <string_view>

namespace mavmoos::runtime {
namespace {

constexpr unsigned mode_limit = 64;
constexpr double mode_timeout = 5;

struct Entry {
  uint32_t custom, properties;
  uint8_t standard;
  std::string name;

  bool matches(const Entry& other) const
  {
    return custom == other.custom && properties == other.properties &&
      standard == other.standard && name == other.name;
  }
};

class ModeCatalog final : public PassiveObserver {
public:
  explicit ModeCatalog(Services& services) : PassiveObserver(services, true, mode_timeout) {}

  std::string name() const override { return "mode_catalog"; }

  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.profile = "mavlink_passive_standard_modes";
    result.capabilities = {{"mode_catalog.available", "", "", "observation"},
                           {"mode_catalog.current", "", "", "observation"}};
    result.exclusive_resources = {"observation.mode_catalog"};
    result.streamed_parameter_bench_compatible = true;
    return result;
  }

private:
  struct Collection {
    Stamp first;
    uint8_t count = 0, sequence = 0;
    unsigned received = 0;
    bool conflict = false;
    std::array<std::optional<Entry>, mode_limit> entries;
  };

  bool observes(uint32_t id) const override
  {
    return id == MAVLINK_MSG_ID_AVAILABLE_MODES || id == MAVLINK_MSG_ID_CURRENT_MODE ||
      id == MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR;
  }

  void observeHeartbeat(const mavlink_heartbeat_t& heartbeat, const MessageTime& time) override
  {
    if (!update(m_heartbeat_stamp, time)) return;
    m_heartbeat_mode = heartbeat.custom_mode;
    m_heartbeat_custom = (heartbeat.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) != 0;
  }

  static bool precedes(const MessageTime& time, const Stamp& boundary)
  {
    return boundary.id && (time.effective_at < boundary.time.effective_at ||
                           time.read_at < boundary.time.read_at);
  }

  bool fresh(const Stamp& stamp)
  {
    return stamp.id && services().now() - stamp.time.effective_at <= sampleTimeout();
  }

  void expireCollection()
  {
    if (!fresh(m_collection.first)) m_collection = {};
  }

  bool acceptSequence(uint8_t sequence, const MessageTime& time)
  {
    if (precedes(time, m_revision_stamp)) return false;
    if (m_revision) {
      if (sequence == *m_revision) return true;
      const uint8_t distance = sequence - *m_revision;
      if (distance >= 128) return false;
    }
    if (m_collection.count && (m_revision || sequence != m_collection.sequence)) m_collection = {};
    m_revision = sequence;
    update(m_revision_stamp, time);
    return true;
  }

  void observe(const mavlink_message_t& message, const MessageTime& time) override
  {
    switch (message.msgid) {
    case MAVLINK_MSG_ID_CURRENT_MODE:
      if (!wireShape(message, MAVLINK_MSG_ID_CURRENT_MODE_MIN_LEN, MAVLINK_MSG_ID_CURRENT_MODE_LEN)) {
        rejectWire();
        return;
      }
      if (update(m_current_stamp, time))
        mavlink_msg_current_mode_decode(&message, &m_current);
      return;
    case MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR: {
      if (!wireShape(message, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_MIN_LEN,
                     MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR_LEN)) {
        rejectWire();
        return;
      }
      const uint8_t sequence = mavlink_msg_available_modes_monitor_get_seq(&message);
      if (precedes(time, m_monitor_stamp) || !acceptSequence(sequence, time)) {
        ++m_catalog_rejected;
        return;
      }
      update(m_monitor_stamp, time);
      m_monitor_sequence = sequence;
      return;
    }
    default:
      observeEntry(message, time);
    }
  }

  void observeEntry(const mavlink_message_t& message, const MessageTime& time)
  {
    if (!wireShape(message, MAVLINK_MSG_ID_AVAILABLE_MODES_MIN_LEN, MAVLINK_MSG_ID_AVAILABLE_MODES_LEN)) {
      rejectWire();
      return;
    }
    mavlink_available_modes_t value{};
    mavlink_msg_available_modes_decode(&message, &value);
    const std::string_view field(value.mode_name, sizeof(value.mode_name));
    const auto end = field.find('\0');
    const auto label = field.substr(0, end);
    if (!value.number_modes || value.number_modes > mode_limit || !value.mode_index ||
        value.mode_index > value.number_modes || end == std::string_view::npos ||
        !std::all_of(label.begin(), label.end(), [](unsigned char ch) { return ch >= 32 && ch <= 126; })) {
      rejectWire();
      return;
    }

    expireCollection();
    if (services().now() - time.effective_at > sampleTimeout() || precedes(time, m_catalog_order) ||
        precedes(time, m_revision_stamp) || (value.seq && !acceptSequence(value.seq, time)) ||
        (!value.seq && m_collection.sequence)) {
      ++m_catalog_rejected;
      return;
    }
    update(m_catalog_order, time);
    if (value.seq && m_collection.count && !m_collection.sequence) m_collection = {};
    if (!m_collection.count) {
      m_collection.first = m_catalog_order;
      m_collection.count = value.number_modes;
      m_collection.sequence = value.seq;
    }

    if (m_collection.conflict) return;
    auto& slot = m_collection.entries[value.mode_index - 1];
    Entry entry{value.custom_mode, value.properties, value.standard_mode, std::string(label)};
    if (value.number_modes != m_collection.count || (slot && !slot->matches(entry))) {
      m_collection.entries = {};
      m_collection.received = 0;
      m_collection.conflict = true;
    } else if (!slot) {
      slot = std::move(entry);
      ++m_collection.received;
    }
  }

  void clearSamples() override
  {
    m_collection = {};
    m_current_stamp = m_heartbeat_stamp = m_monitor_stamp = m_revision_stamp = m_catalog_order = {};
    m_current = {};
    m_heartbeat_mode = m_monitor_sequence = 0;
    m_heartbeat_custom = false;
    m_revision.reset();
  }

  bool complete() const
  {
    return m_collection.count && m_collection.received == m_collection.count && !m_collection.conflict;
  }

  void writeLabel(std::ostream& out, uint32_t custom, uint8_t standard) const
  {
    const Entry* match = nullptr;
    if (complete()) {
      for (const auto& item : m_collection.entries) {
        if (!item) continue;
        const bool same = standard ? item->standard == standard : item->custom == custom;
        if (!same) continue;
        if (match) { match = nullptr; break; }
        match = &*item;
      }
    }
    if (match && !match->name.empty())
      out << std::quoted(match->name);
    else
      out << std::quoted((standard && custom == 0 ? "standard " + std::to_string(standard)
                                                 : std::to_string(custom)));
  }

  void publishActive()
  {
    const bool fallback = fresh(m_heartbeat_stamp) && m_heartbeat_custom;
    const bool disagrees = fallback && precedes(m_current_stamp.time, m_heartbeat_stamp) &&
      m_current.custom_mode != m_heartbeat_mode;
    const bool use_current = fresh(m_current_stamp) && !disagrees;
    const auto& stamp = use_current ? m_current_stamp : (fallback ? m_heartbeat_stamp : m_empty);
    std::ostringstream out;
    out << "{\"metadata\":" << metadata(stamp) << ",\"data\":";
    if (!stamp.id) {
      out << "null";
    } else {
      const uint32_t active = use_current ? m_current.custom_mode : m_heartbeat_mode;
      const uint8_t standard = use_current ? m_current.standard_mode : 0;
      out << "{\"source\":\"" << (use_current ? "CURRENT_MODE" : "HEARTBEAT")
          << "\",\"active_custom_mode\":" << active << ",\"standard_mode\":";
      if (standard) out << unsigned(standard); else out << "null";
      out << ",\"active_label\":";
      writeLabel(out, active, standard);
      out << ",\"intended_custom_mode\":";
      const uint32_t intended = use_current ? m_current.intended_custom_mode : 0;
      if (intended) out << intended; else out << "null";
      out << ",\"intended_label\":";
      if (intended) writeLabel(out, intended, 0); else out << "null";
      out << '}';
    }
    out << '}';
    services().publish("MAVMOOS_CURRENT_MODE", out.str());
  }

  void publishSamples() override
  {
    expireCollection();
    std::ostringstream out;
    out << std::boolalpha << "{\"metadata\":" << metadata(m_collection.first)
        << ",\"monitor_metadata\":" << metadata(m_monitor_stamp) << ",\"monitor_sequence\":";
    if (m_monitor_stamp.id) out << m_monitor_sequence; else out << "null";
    out << ",\"revision\":";
    if (m_revision) out << unsigned(*m_revision); else out << "null";
    out << ",\"sequence_bound\":" << (m_collection.sequence != 0)
        << ",\"complete\":" << complete() << ",\"conflict\":" << m_collection.conflict
        << ",\"expected_count\":";
    if (m_collection.count) out << unsigned(m_collection.count); else out << "null";
    out << ",\"received_count\":" << m_collection.received
        << ",\"catalog_rejected\":" << m_catalog_rejected << ",\"entries\":[";
    bool first = true;
    for (unsigned i = 0; i < m_collection.count; ++i) {
      const auto& entry = m_collection.entries[i];
      if (!entry) continue;
      if (!first) out << ',';
      first = false;
      out << "{\"index\":" << i + 1 << ",\"custom_mode\":" << entry->custom
          << ",\"standard_mode\":" << unsigned(entry->standard) << ",\"properties\":" << entry->properties
          << ",\"name\":";
      if (entry->name.empty()) out << "null"; else out << std::quoted(entry->name);
      out << '}';
    }
    out << "]}";
    services().publish("MAVMOOS_MODE_CATALOG", out.str());
    publishActive();
  }

  Collection m_collection;
  Stamp m_current_stamp, m_heartbeat_stamp, m_monitor_stamp, m_revision_stamp, m_catalog_order;
  const Stamp m_empty{};
  mavlink_current_mode_t m_current{};
  uint32_t m_heartbeat_mode = 0;
  unsigned m_monitor_sequence = 0;
  bool m_heartbeat_custom = false;
  std::optional<uint8_t> m_revision;
  uint64_t m_catalog_rejected = 0;
};

} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi, mavmoos::runtime::Services* services)
{
  if (abi != mavmoos::runtime::plugin_abi || !services) return nullptr;
  return new mavmoos::runtime::ModeCatalog(*services);
}

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin)
{
  delete plugin;
}
