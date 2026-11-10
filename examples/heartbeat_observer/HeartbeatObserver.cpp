// A startup-loaded external observer. No socket, stream request or control effect.
#include "Plugin.h"
#include <charconv>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <set>
#include <sstream>

namespace mavmoos::example {
using namespace runtime;
struct ObserverSettings {
  double sample_timeout = 2, publish_interval = .25;
};

class HeartbeatObserver final : public Plugin {
public:
  explicit HeartbeatObserver(Services& services) : services_(services) {
    const auto now = services_.now();
    clock_fault_ = !std::isfinite(now) || now < 0;
    if (!clock_fault_) boundary_ = last_now_ = now;
  }
  std::string name() const override { return "example_heartbeat_observer"; }
  ModuleConfigResult configureModule(const std::vector<ModuleSetting>& settings) override {
    // The host supplies one complete startup batch. An omitted value starts
    // from its default, and neither a valid nor an invalid batch uses Services.
    ObserverSettings candidate;
    std::set<std::string> seen;
    for (const auto& setting : settings) {
      if (setting.key != "sample_timeout" && setting.key != "publish_interval")
        return {ModuleConfigStatus::UnknownKey, setting.key, "unknown observer setting"};
      if (!seen.insert(setting.key).second)
        return {ModuleConfigStatus::Invalid, setting.key, "duplicate observer setting"};
      double value = 0;
      const auto* begin = setting.value.data();
      const auto parsed = std::from_chars(begin, begin + setting.value.size(), value);
      if (setting.value.empty() || parsed.ec != std::errc{} ||
          parsed.ptr != begin + setting.value.size() || !std::isfinite(value))
        return {ModuleConfigStatus::Invalid, setting.key,
                "expected finite decimal seconds, with no whitespace or suffix"};
      if (setting.key == "sample_timeout") {
        if (value < .25 || value > 10)
          return {ModuleConfigStatus::Invalid, setting.key, "seconds must be in [0.25,10]"};
        candidate.sample_timeout = value;
      } else {
        if (value < .02 || value > 2)
          return {ModuleConfigStatus::Invalid, setting.key, "seconds must be in [0.02,2]"};
        candidate.publish_interval = value;
      }
    }
    if (candidate.publish_interval > candidate.sample_timeout)
      return {ModuleConfigStatus::Invalid, "publish_interval", "must not exceed sample_timeout"};
    settings_ = candidate;
    return {};
  }
  PluginDescriptor descriptor() const override {
    PluginDescriptor value;
    value.profile = "example_read_only_heartbeat";
    value.capabilities = {{"example.heartbeat.observation", "", "", "observation"}};
    value.exclusive_resources = {"observation.example_heartbeat"};
    value.streamed_parameter_bench_compatible = true;
    return value;
  }
  void onInputBoundary() override { checkClock(); }
  void onReset(const std::string&) override {
    const auto now = services_.now();
    clock_fault_ = !std::isfinite(now) || now < 0 || now < last_now_;
    if (!clock_fault_) boundary_ = last_now_ = now;
    clear();
  }
  void onMessageWithMetadata(const mavlink_message_t& message, const MessageTime& time) override {
    if (message.msgid != MAVLINK_MSG_ID_HEARTBEAT) return;
    if (!checkClock() || !std::isfinite(time.effective_at) || !std::isfinite(time.read_at) ||
        time.effective_at <= boundary_ || time.read_at < time.effective_at || time.read_at > last_now_ ||
        (time.age_basis != "QUEUE_EMPTY_FLOOR" &&
         !(time.age_basis == "READ_ACQUISITION" && time.read_at == time.effective_at))) {
      ++metadata_rejected_; return;
    }
    const bool wire = message.sysid && message.compid &&
      ((message.magic == MAVLINK_STX && message.len > 0 && message.len <= MAVLINK_MSG_ID_HEARTBEAT_LEN &&
        !(message.incompat_flags & ~MAVLINK_IFLAG_SIGNED)) ||
       (message.magic == MAVLINK_STX_MAVLINK1 && message.len == MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN));
    if (!wire) { ++wire_rejected_; return; }
    if (present_ && (message.sysid != system_ || message.compid != component_)) {
      ++source_rejected_; return;
    }
    if (present_ && (time.effective_at < time_.effective_at || time.read_at < time_.read_at)) {
      ++order_rejected_; return;
    }
    mavlink_heartbeat_t payload{};
    mavlink_msg_heartbeat_decode(&message, &payload);
    present_ = true; system_ = message.sysid; component_ = message.compid;
    custom_mode_ = payload.custom_mode; time_ = time; ++observation_id_;
  }
  void iterate() override {
    checkClock();
    const auto now = services_.now();
    if (!std::isfinite(now) || now < 0 || (next_publish_ >= 0 && now < next_publish_)) return;
    next_publish_ = now + settings_.publish_interval;
    const bool present = present_ && !clock_fault_;
    const auto age = present ? now - time_.effective_at : 0;
    std::ostringstream out;
    out << std::setprecision(17) << std::boolalpha
        << "{\"schema\":1,\"example\":true,\"read_only\":true,\"authoritative_for_control\":false"
        << ",\"session_epoch\":" << session_ << ",\"clock_fault\":" << clock_fault_
        << ",\"present\":" << present << ",\"fresh\":" << (present && age <= settings_.sample_timeout)
        << ",\"sample_timeout_s\":" << settings_.sample_timeout
        << ",\"publish_interval_s\":" << settings_.publish_interval
        << ",\"observation_id\":" << observation_id_
        << ",\"effective_at\":" << (present ? number(time_.effective_at) : "null")
        << ",\"read_at\":" << (present ? number(time_.read_at) : "null")
        << ",\"age_s\":" << (present ? number(age) : "null")
        << ",\"age_basis\":\"" << (present ? time_.age_basis : "UNKNOWN") << "\""
        << ",\"source_age\":\"unknown\",\"source_quality\":\"unassessed\""
        << ",\"system_id\":" << (present ? std::to_string(system_) : "null")
        << ",\"component_id\":" << (present ? std::to_string(component_) : "null")
        << ",\"custom_mode_raw\":" << (present ? std::to_string(custom_mode_) : "null")
        << ",\"metadata_rejected\":" << metadata_rejected_ << ",\"source_rejected\":" << source_rejected_
        << ",\"wire_rejected\":" << wire_rejected_ << ",\"order_rejected\":" << order_rejected_ << '}';
    services_.publish("EXAMPLE_HEARTBEAT_OBSERVATION", out.str());
  }
private:
  static std::string number(double value) {
    std::ostringstream out; out << std::setprecision(17) << value; return out.str();
  }
  void clear() { present_ = false; next_publish_ = -1; ++session_; }
  bool checkClock() {
    const auto now = services_.now();
    if (!std::isfinite(now) || now < 0 || now < last_now_) {
      if (!clock_fault_) clear();
      clock_fault_ = true;
    }
    if (clock_fault_) return false;
    last_now_ = now; return true;
  }
  Services& services_;
  ObserverSettings settings_;
  MessageTime time_;
  double boundary_ = 0, last_now_ = 0, next_publish_ = -1;
  bool present_ = false, clock_fault_ = false;
  unsigned system_ = 0, component_ = 0;
  uint32_t custom_mode_ = 0;
  uint64_t session_ = 1, observation_id_ = 0;
  uint64_t metadata_rejected_ = 0, source_rejected_ = 0, wire_rejected_ = 0, order_rejected_ = 0;
};
}

#ifndef MAVMOOS_EXAMPLE_ABI_DELTA
#define MAVMOOS_EXAMPLE_ABI_DELTA 0
#endif
extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi, mavmoos::runtime::Services* services) {
  return abi == mavmoos::runtime::plugin_abi + MAVMOOS_EXAMPLE_ABI_DELTA && services
    ? new mavmoos::example::HeartbeatObserver(*services) : nullptr;
}
extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin) { delete plugin; }
