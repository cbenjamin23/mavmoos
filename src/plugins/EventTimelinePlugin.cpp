/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: EventTimelinePlugin.cpp                         */
/*    DATE: November 6, 2026                                */
/************************************************************/
#include "PassiveObserver.h"

#include <array>
#include <cstdint>
#include <deque>

namespace mavmoos::runtime {
namespace {

constexpr std::size_t timeline_limit = 16;
constexpr uint16_t half_range = 0x8000;
constexpr uint8_t bridge_system = 245;
constexpr uint8_t bridge_component = 191;

uint16_t forwardDistance(uint16_t from, uint16_t to)
{
  return static_cast<uint16_t>(to - from);
}

bool precedes(const MessageTime& candidate, const MessageTime& existing)
{
  return candidate.effective_at < existing.effective_at || candidate.read_at < existing.read_at;
}

bool forBridge(const mavlink_event_t& event)
{
  return (event.destination_system == 0 || event.destination_system == bridge_system) &&
    (event.destination_component == 0 || event.destination_component == bridge_component);
}

void writeSequence(std::ostream& out, std::optional<uint16_t> value)
{
  if (value) out << *value;
  else out << "null";
}

void writeLevel(std::ostream& out, unsigned level)
{
  constexpr std::array names{"emergency", "alert", "critical", "error", "warning",
                             "notice", "info", "debug", "protocol", "disabled"};
  out << "{\"raw\":" << level << ",\"name\":";
  if (level < names.size()) out << '"' << names[level] << '"';
  else out << "null";
  out << '}';
}

struct SequenceReport {
  mavlink_current_event_sequence_t packet{};
  bool reset_applied = false;
};

void writeData(std::ostream& out, const SequenceReport& report)
{
  constexpr unsigned reset = MAV_EVENT_CURRENT_SEQUENCE_FLAGS_RESET;
  const unsigned flags = report.packet.flags;
  out << "{\"sequence\":" << report.packet.sequence << ",\"flags_raw\":" << flags
      << ",\"unknown_flags\":" << (flags & ~reset)
      << ",\"reset_announced\":" << bool(flags & reset)
      << ",\"reset_applied\":" << report.reset_applied << '}';
}

void writeData(std::ostream& out, const mavlink_response_event_error_t& error)
{
  out << "{\"sequence\":" << error.sequence
      << ",\"sequence_oldest_available\":" << error.sequence_oldest_available
      << ",\"target_system\":" << unsigned(error.target_system)
      << ",\"target_component\":" << unsigned(error.target_component)
      << ",\"reason_raw\":" << unsigned(error.reason) << ",\"reason\":";
  if (error.reason == MAV_EVENT_ERROR_REASON_UNAVAILABLE) out << "\"unavailable\"";
  else out << "null";
  out << ",\"request_correlation\":\"none\"}";
}

class EventTimeline final : public PassiveObserver {
public:
  explicit EventTimeline(Services& services) : PassiveObserver(services, true, 5) {}

  std::string name() const override { return "event_timeline"; }

  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.profile = "mavlink_event_timeline_observations";
    result.capabilities = {{"event_timeline.events", "", "", "observation"},
                           {"event_timeline.sequence_status", "", "", "observation"},
                           {"event_timeline.replay_error", "", "", "observation"}};
    result.exclusive_resources = {"observation.event_timeline"};
    result.streamed_parameter_bench_compatible = true;
    return result;
  }

private:
  template<typename Payload>
  struct Observation {
    Stamp stamp;
    unsigned length = 0;
    Payload payload{};
  };

  struct Event {
    Observation<mavlink_event_t> observation;
    unsigned missing_before = 0;
    bool wrap = false;
  };

  struct Counters {
    uint64_t holes = 0;
    uint64_t wraps = 0;
    uint64_t duplicates = 0;
    uint64_t old_sequences = 0;
    uint64_t ambiguous = 0;
    uint64_t other_destinations = 0;
    uint64_t evicted = 0;
    uint64_t order_rejected = 0;
  };

  bool observes(uint32_t id) const override
  {
    return id == MAVLINK_MSG_ID_EVENT || id == MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE ||
           id == MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR;
  }

  bool validPacket(const mavlink_message_t& message, unsigned length)
  {
    if (wireShape(message, length, length)) return true;
    rejectWire();
    return false;
  }

  void observe(const mavlink_message_t& message, const MessageTime& time) override
  {
    if (m_reset_floor && precedes(time, *m_reset_floor)) {
      ++m_counters.order_rejected;
      return;
    }
    switch (message.msgid) {
    case MAVLINK_MSG_ID_EVENT:
      observeEvent(message, time);
      break;
    case MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE:
      observeSequence(message, time);
      break;
    case MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR:
      if (validPacket(message, MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR_LEN) && update(m_error.stamp, time)) {
        m_error.length = message.len;
        mavlink_msg_response_event_error_decode(&message, &m_error.payload);
      }
      break;
    }
  }

  std::optional<uint16_t> reference() const
  {
    return m_latest ? m_latest : m_anchor;
  }

  void observeEvent(const mavlink_message_t& message, const MessageTime& time)
  {
    if (!validPacket(message, MAVLINK_MSG_ID_EVENT_LEN)) return;
    mavlink_event_t packet{};
    mavlink_msg_event_decode(&message, &packet);
    const auto prior = reference();
    const auto distance = prior ? forwardDistance(*prior, packet.sequence) : 0;
    if (m_latest && distance == 0) {
      ++m_counters.duplicates;
      return;
    }
    if (distance == half_range) {
      ++m_counters.ambiguous;
      return;
    }
    if (distance > half_range) {
      ++m_counters.old_sequences;
      return;
    }
    if (!update(m_latest_stamp, time)) return;
    m_latest = packet.sequence;
    const unsigned missing = distance > 1 ? distance - 1 : 0;
    const bool wrap = prior && distance && packet.sequence < *prior;
    m_counters.holes += missing;
    m_counters.wraps += wrap;
    // EVENT's destination fields do not prevent broadcast routing. Every
    // valid sequence contributes to continuity, including another recipient.
    if (!forBridge(packet)) {
      ++m_counters.other_destinations;
      return;
    }
    if (m_timeline.size() == timeline_limit) {
      m_timeline.pop_front();
      ++m_counters.evicted;
    }
    m_timeline.push_back({{m_latest_stamp, message.len, packet}, missing, wrap});
  }

  void observeSequence(const mavlink_message_t& message, const MessageTime& time)
  {
    if (!validPacket(message, MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN)) return;
    mavlink_current_event_sequence_t packet{};
    mavlink_msg_current_event_sequence_decode(&message, &packet);
    const bool flagged = packet.flags & MAV_EVENT_CURRENT_SEQUENCE_FLAGS_RESET;
    const bool apply_reset = flagged && m_reset_marker != packet.sequence;
    if (apply_reset && ((m_latest_stamp.id && precedes(time, m_latest_stamp.time)) ||
                        (m_error.stamp.id && precedes(time, m_error.stamp.time)))) {
      ++m_counters.order_rejected;
      return;
    }
    if (!update(m_current.stamp, time)) return;
    if (apply_reset) {
      clearSequence();
      m_reset_floor = time;
      ++m_sequence_epoch;
      ++m_resets;
    }
    m_reset_marker = flagged ? std::optional<uint16_t>(packet.sequence) : std::nullopt;
    if (!reference()) m_anchor = packet.sequence;
    m_current.length = message.len;
    m_current.payload = {packet, apply_reset};
  }

  void clearSequence()
  {
    m_timeline.clear();
    m_error = {};
    m_latest.reset();
    m_latest_stamp = {};
    m_anchor.reset();
    m_counters = {};
  }

  void clearSamples() override
  {
    clearSequence();
    m_current = {};
    m_reset_marker.reset();
    m_reset_floor.reset();
    m_sequence_epoch = 1;
    m_resets = 0;
  }

  void publishSamples() override
  {
    publishTimeline();
    publishReport("MAVMOOS_EVENT_SEQUENCE_STATUS", "CURRENT_EVENT_SEQUENCE",
                  MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE, m_current);
    publishReport("MAVMOOS_EVENT_REPLAY_ERROR", "RESPONSE_EVENT_ERROR",
                  MAVLINK_MSG_ID_RESPONSE_EVENT_ERROR, m_error);
  }

  void writeAdvertisement(std::ostream& out) const
  {
    const auto prior = reference();
    const char* relation = "unseen";
    std::optional<uint16_t> unseen;
    if (m_current.stamp.id && prior) {
      const auto sequence = m_current.payload.packet.sequence;
      const auto distance = forwardDistance(*prior, sequence);
      if (distance == half_range) relation = "ambiguous";
      else if (distance > half_range) relation = "behind";
      else {
        unseen = distance;
        relation = distance == 0 ? "same" : sequence < *prior ? "ahead_across_wrap" : "ahead";
      }
    }
    out << ",\"advertised_sequence\":";
    writeSequence(out, m_current.stamp.id ? std::optional<uint16_t>(m_current.payload.packet.sequence) : std::nullopt);
    out << ",\"advertised_relation\":\"" << relation << "\",\"advertised_unseen\":";
    writeSequence(out, unseen);
  }

  void writeEvent(std::ostream& out, const Event& event) const
  {
    constexpr char hex[] = "0123456789abcdef";
    constexpr unsigned arguments_offset = 13;
    const auto& sample = event.observation;
    const auto& packet = sample.payload;
    const auto argument_bytes = sample.length > arguments_offset ? sample.length - arguments_offset : 0;
    out << "{\"wire_version\":2,\"payload_length\":" << sample.length
        << ",\"metadata\":" << metadata(sample.stamp) << ",\"data\":{\"event_id_raw\":" << packet.id
        << ",\"event_namespace_raw\":" << (packet.id >> 24)
        << ",\"event_time_boot_ms_raw\":" << packet.event_time_boot_ms << ",\"sequence\":" << packet.sequence
        << ",\"destination_system\":" << unsigned(packet.destination_system)
        << ",\"destination_component\":" << unsigned(packet.destination_component)
        << ",\"missing_before\":" << event.missing_before << ",\"sequence_wrap\":" << event.wrap
        << ",\"log_levels_raw\":" << unsigned(packet.log_levels) << ",\"internal_log_level\":";
    writeLevel(out, packet.log_levels >> 4);
    out << ",\"external_log_level\":";
    writeLevel(out, packet.log_levels & 15);
    out << ",\"argument_wire_bytes\":" << argument_bytes << ",\"arguments_raw_hex\":\"";
    for (const auto byte : packet.arguments) out << hex[byte >> 4] << hex[byte & 15];
    out << "\"}}";
  }

  void publishTimeline()
  {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::boolalpha << "{\"schema\":1,\"family\":\"EVENT\",\"message_id\":410"
        << ",\"present\":" << !m_timeline.empty()
        << ",\"read_only\":true,\"authoritative_for_control\":false,\"atomic\":false"
        << ",\"timeline_limit\":" << timeline_limit << ",\"sequence_epoch\":" << m_sequence_epoch
        << ",\"sequence_resets\":" << m_resets << ",\"sequence_anchor\":";
    writeSequence(out, m_anchor);
    out << ",\"latest_observed_sequence\":";
    writeSequence(out, m_latest);
    writeAdvertisement(out);
    out << ",\"holes_observed\":" << m_counters.holes << ",\"sequence_wraps\":" << m_counters.wraps
        << ",\"duplicates\":" << m_counters.duplicates << ",\"older_or_replayed\":" << m_counters.old_sequences
        << ",\"ambiguous_sequence\":" << m_counters.ambiguous
        << ",\"other_destinations\":" << m_counters.other_destinations << ",\"evicted\":" << m_counters.evicted
        << ",\"sequence_order_rejected\":" << m_counters.order_rejected << ",\"samples\":[";
    bool first = true;
    for (const auto& event : m_timeline) {
      if (!first) out << ',';
      first = false;
      writeEvent(out, event);
    }
    out << "]}";
    services().publish("MAVMOOS_EVENT_TIMELINE", out.str());
  }

  template<typename Payload>
  void publishReport(const char* key, const char* family, unsigned id, const Observation<Payload>& sample)
  {
    auto envelope = metadata(sample.stamp);
    envelope.pop_back();
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::boolalpha << envelope << ",\"family\":\"" << family << "\",\"message_id\":" << id
        << ",\"sequence_epoch\":" << m_sequence_epoch << ",\"wire_version\":"
        << (sample.stamp.id ? "2" : "null") << ",\"payload_length\":";
    if (sample.stamp.id) {
      out << sample.length << ",\"data\":";
      writeData(out, sample.payload);
    } else out << "null,\"data\":null";
    out << '}';
    services().publish(key, out.str());
  }

  std::deque<Event> m_timeline;
  Observation<SequenceReport> m_current;
  Observation<mavlink_response_event_error_t> m_error;
  std::optional<uint16_t> m_anchor, m_latest, m_reset_marker;
  std::optional<MessageTime> m_reset_floor;
  Stamp m_latest_stamp;
  Counters m_counters;
  uint64_t m_sequence_epoch = 1, m_resets = 0;
};

} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi, mavmoos::runtime::Services* services)
{
  return abi == mavmoos::runtime::plugin_abi && services ? new mavmoos::runtime::EventTimeline(*services) : nullptr;
}

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin)
{
  delete plugin;
}
