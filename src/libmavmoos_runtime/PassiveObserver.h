#pragma once

#include "Plugin.h"

#include <charconv>
#include <cmath>
#include <iomanip>
#include <locale>
#include <optional>
#include <sstream>

namespace mavmoos::runtime {

// Common source, session and time rules for passive wire observations. Derived
// modules decide which message families and payloads they can interpret.
class PassiveObserver : public Plugin {
public:
  explicit PassiveObserver(Services& services, bool accept_px4 = false, double timeout = 1,
                           std::optional<unsigned> expected_type = std::nullopt)
    : m_services(services), m_accept_px4(accept_px4), m_expected_type(expected_type),
      m_default_timeout(timeout), m_timeout(timeout)
  {
    const double now = m_services.now();
    m_clock_fault = !std::isfinite(now) || now < 0;
    if (!m_clock_fault) m_floor = m_last_now = now;
  }

  ModuleConfigResult configureModule(const std::vector<ModuleSetting>& settings) override
  {
    double timeout = m_default_timeout, interval = .1;
    bool saw_timeout = false, saw_interval = false;
    for (const auto& setting : settings) {
      const bool is_timeout = setting.key == "sample_timeout";
      if (!is_timeout && setting.key != "publish_interval")
        return {ModuleConfigStatus::UnknownKey, setting.key, "unknown observer setting"};
      bool& seen = is_timeout ? saw_timeout : saw_interval;
      if (seen) return {ModuleConfigStatus::Invalid, setting.key, "duplicate observer setting"};
      seen = true;
      const char* begin = setting.value.data();
      const char* end = begin + setting.value.size();
      double value = 0;
      const auto parsed = std::from_chars(begin, end, value);
      const double minimum = is_timeout ? .25 : .02;
      const double maximum = is_timeout ? 10 : 2;
      if (parsed.ec != std::errc{} || parsed.ptr != end || !std::isfinite(value) ||
          value < minimum || value > maximum)
        return {ModuleConfigStatus::Invalid, setting.key, "finite decimal seconds outside allowed range"};
      (is_timeout ? timeout : interval) = value;
    }
    if (interval > timeout)
      return {ModuleConfigStatus::Invalid, "publish_interval", "must not exceed sample_timeout"};
    m_timeout = timeout;
    m_interval = interval;
    return {};
  }

  void onInputBoundary() final { checkBoundary(); }

  void onReset(const std::string&) final
  {
    const double now = m_services.now();
    m_clock_fault = !std::isfinite(now) || now < m_last_now;
    if (!m_clock_fault) m_floor = m_last_now = now;
    clear();
  }

  void onMessageWithMetadata(const mavlink_message_t& message, const MessageTime& time) final
  {
    const bool heartbeat = message.msgid == MAVLINK_MSG_ID_HEARTBEAT;
    if (!heartbeat && !observes(message.msgid)) return;
    if (!checkBoundary() || !validTime(time)) {
      ++m_metadata_rejected;
      return;
    }
    const unsigned component = heartbeat ? 1 : m_observation_component;
    if (message.sysid != 1 || !component || message.compid != component) {
      ++m_source_rejected;
      return;
    }
    if (heartbeat) {
      bindHeartbeat(message, time);
      return;
    }
    if (!m_bound || older(time, m_heartbeat)) {
      ++m_source_rejected;
      return;
    }
    observe(message, time);
  }

  void iterate() final
  {
    checkBoundary();
    const double now = m_services.now();
    if (!std::isfinite(now) || now < 0 || now < m_next_publish) return;
    m_next_publish = now + m_interval;
    publishSamples();
  }

protected:
  struct Stamp { MessageTime time; uint64_t id = 0; };
  virtual void observeHeartbeat(const mavlink_heartbeat_t&, const MessageTime&) {}
  virtual bool observes(uint32_t id) const = 0;
  virtual void observe(const mavlink_message_t&, const MessageTime&) = 0;
  virtual void clearSamples() = 0;
  virtual void publishSamples() = 0;

  Services& services() { return m_services; }
  double sampleTimeout() const { return m_timeout; }
  unsigned autopilot() const { return m_autopilot; }
  unsigned observationComponent() const { return m_observation_component; }
  void setObservationComponent(unsigned component) { m_observation_component = component; }
  void rejectWire() { ++m_wire_rejected; }

  static bool wireShape(const mavlink_message_t& message, unsigned v1_length, unsigned maximum)
  {
    if (message.magic == MAVLINK_STX)
      return message.len > 0 && message.len <= maximum && !(message.incompat_flags & ~MAVLINK_IFLAG_SIGNED);
    return message.magic == MAVLINK_STX_MAVLINK1 && message.msgid < 256 && message.len == v1_length;
  }

  bool update(Stamp& stamp, const MessageTime& time)
  {
    if (stamp.id && older(time, stamp.time)) {
      ++m_order_rejected;
      return false;
    }
    stamp = {time, ++m_observation};
    return true;
  }

  std::string metadata(const Stamp& stamp) const
  {
    const bool present = m_bound && stamp.id;
    const double age = present ? m_last_now - stamp.time.effective_at : 0;
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::boolalpha << std::setprecision(17)
        << "{\"schema\":1,\"session_epoch\":" << m_session
        << ",\"read_only\":true,\"authoritative_for_control\":false"
        << ",\"present\":" << present << ",\"fresh\":" << (present && age <= m_timeout)
        << ",\"clock_fault\":" << m_clock_fault
        << ",\"observation_id\":" << stamp.id
        << ",\"sample_timeout_s\":" << m_timeout << ",\"publish_interval_s\":" << m_interval
        << ",\"source_age\":\"unknown\",\"source_quality\":\"unassessed\"";
    if (present) {
      out << ",\"effective_at\":" << stamp.time.effective_at << ",\"read_at\":" << stamp.time.read_at
          << ",\"age_s\":" << age << ",\"age_basis\":\"" << stamp.time.age_basis << '"'
          << ",\"age_is_upper_bound\":" << (stamp.time.age_basis == "QUEUE_EMPTY_FLOOR");
    } else {
      out << ",\"effective_at\":null,\"read_at\":null,\"age_s\":null,\"age_basis\":\"UNKNOWN\""
          << ",\"age_is_upper_bound\":false";
    }
    out << ",\"system_id\":" << (m_bound ? "1" : "null")
        << ",\"component_id\":" << (m_bound ? std::to_string(m_observation_component) : "null")
        << ",\"autopilot\":" << (m_bound ? std::to_string(m_autopilot) : "null")
        << ",\"metadata_rejected\":" << m_metadata_rejected << ",\"source_rejected\":" << m_source_rejected
        << ",\"wire_rejected\":" << m_wire_rejected << ",\"order_rejected\":" << m_order_rejected << '}';
    return out.str();
  }

private:
  static bool older(const MessageTime& time, const MessageTime& previous)
  {
    return time.effective_at < previous.effective_at || time.read_at < previous.read_at;
  }

  bool validTime(const MessageTime& time) const
  {
    return std::isfinite(time.effective_at) && std::isfinite(time.read_at) &&
           time.effective_at > m_floor && time.effective_at <= time.read_at && time.read_at <= m_last_now &&
           (time.age_basis == "QUEUE_EMPTY_FLOOR" ||
            (time.age_basis == "READ_ACQUISITION" && time.effective_at == time.read_at));
  }

  bool freshHeartbeat(double time) const
  {
    return std::isfinite(time) && time > 0 && time <= m_last_now && m_last_now - time <= 2;
  }

  void bindHeartbeat(const mavlink_message_t& message, const MessageTime& time)
  {
    if (!wireShape(message, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN) ||
        message.len != MAVLINK_MSG_ID_HEARTBEAT_LEN) {
      rejectWire();
      return;
    }
    mavlink_heartbeat_t packet{};
    mavlink_msg_heartbeat_decode(&message, &packet);
    const auto& vehicle = m_services.vehicle();
    const bool supported = packet.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA ||
                           (m_accept_px4 && packet.autopilot == MAV_AUTOPILOT_PX4);
    if (!supported || (m_expected_type && packet.type != *m_expected_type) ||
        !vehicle.heartbeat_valid || !freshHeartbeat(vehicle.heartbeat_at) ||
        !freshHeartbeat(time.effective_at) || packet.type != vehicle.type || packet.autopilot != vehicle.autopilot) {
      ++m_source_rejected;
      return;
    }
    if (m_bound && older(time, m_heartbeat)) {
      ++m_order_rejected;
      return;
    }
    m_bound = true;
    m_type = packet.type;
    m_autopilot = packet.autopilot;
    m_heartbeat = time;
    observeHeartbeat(packet, time);
  }

  bool checkBoundary()
  {
    if (m_clock_fault) return false;
    const double now = m_services.now();
    if (!std::isfinite(now) || now < m_last_now) {
      m_clock_fault = true;
      clear();
      return false;
    }
    m_last_now = now;
    const auto& vehicle = m_services.vehicle();
    if (m_bound && (!vehicle.heartbeat_valid || !freshHeartbeat(vehicle.heartbeat_at) ||
                    !freshHeartbeat(m_heartbeat.effective_at) ||
                    vehicle.type != m_type || vehicle.autopilot != m_autopilot)) {
      m_floor = now;
      clear();
    }
    return true;
  }

  void clear()
  {
    clearSamples();
    m_bound = false;
    m_type = m_autopilot = 0;
    m_heartbeat = {};
    m_next_publish = -1;
    ++m_session;
  }

  Services& m_services;
  const bool m_accept_px4;
  const std::optional<unsigned> m_expected_type;
  const double m_default_timeout;
  bool m_bound = false, m_clock_fault = false;
  unsigned m_type = 0, m_autopilot = 0, m_observation_component = 1;
  double m_floor = 0, m_last_now = 0, m_next_publish = -1;
  double m_timeout = 1, m_interval = .1;
  MessageTime m_heartbeat;
  uint64_t m_session = 1, m_observation = 0;
  uint64_t m_metadata_rejected = 0, m_source_rejected = 0, m_wire_rejected = 0, m_order_rejected = 0;
};

} // namespace mavmoos::runtime
