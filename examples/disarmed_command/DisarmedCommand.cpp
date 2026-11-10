// A trusted startup module for one explicitly requested, non-forced DISARM.
#include "Plugin.h"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string_view>

namespace mavmoos::example {
using namespace runtime;
namespace {
constexpr char module_id[] = "example_disarmed_command";
constexpr char request_key[] = "EXAMPLE_DISARM_REQUEST";
constexpr char result_key[] = "EXAMPLE_DISARM_RESULT";
constexpr double maximum_observation_age = 1.5;
constexpr std::size_t request_limit = 64;

bool token(std::string_view value, std::size_t maximum) {
  return !value.empty() && value.size() <= maximum &&
    std::all_of(value.begin(), value.end(), [](unsigned char c) {
      return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
             (c >= '0' && c <= '9') || c == '_' || c == '-';
    });
}
bool boundedSeconds(std::string_view value, double minimum, double maximum, double& seconds) {
  const auto* end = value.data() + value.size();
  const auto parsed = std::from_chars(value.data(), end, seconds);
  return parsed.ec == std::errc{} && parsed.ptr == end && std::isfinite(seconds) &&
         seconds >= minimum && seconds <= maximum;
}
struct Request {
  std::string id, target;
  double timeout = 0;
};
std::optional<Request> parse(std::string_view text) {
  if (text.empty() || text.size() > 192 || text.back() == ',') return {};
  std::map<std::string_view, std::string_view> fields;
  while (!text.empty()) {
    const auto comma = text.find(',');
    const auto item = text.substr(0, comma);
    const auto equal = item.find('=');
    if (equal == std::string_view::npos ||
        !fields.emplace(item.substr(0, equal), item.substr(equal + 1)).second) return {};
    if (comma == std::string_view::npos) break;
    text.remove_prefix(comma + 1);
  }
  if (fields.size() < 4 || fields.size() > 5 || fields.count("schema") == 0 ||
      fields.count("id") == 0 || fields.count("action") == 0 ||
      fields.count("timeout") == 0 || fields.at("schema") != "1" ||
      !token(fields.at("id"), 32)) return {};
  Request request{std::string(fields.at("id")), {}, 0};
  if (!boundedSeconds(fields.at("timeout"), 1, 5, request.timeout)) return {};
  if (fields.at("action") == "disarm" && fields.size() == 4) return request;
  if (fields.at("action") != "cancel" || fields.count("target") == 0 ||
      !token(fields.at("target"), 32) || fields.at("target") == request.id) return {};
  request.target = fields.at("target");
  return request;
}
std::string number(double value) {
  if (!std::isfinite(value)) return "null";
  std::ostringstream out;
  out << std::setprecision(17) << value;
  return out.str();
}
std::string frameHex(const mavlink_message_t& message) {
  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> data{};
  const auto count = mavlink_msg_to_send_buffer(data.data(), &message);
  constexpr char hex[] = "0123456789abcdef";
  std::string out;
  for (std::size_t i = 0; i < count; ++i) {
    out += hex[data[i] >> 4];
    out += hex[data[i] & 15];
  }
  return out;
}
bool validAckFrame(const mavlink_message_t& message) {
  return (message.magic == MAVLINK_STX && message.len >= 2 &&
          message.len <= MAVLINK_MSG_ID_COMMAND_ACK_LEN &&
          !(message.incompat_flags & ~MAVLINK_IFLAG_SIGNED)) ||
         (message.magic == MAVLINK_STX_MAVLINK1 && message.len == MAVLINK_MSG_ID_COMMAND_ACK_MIN_LEN);
}
}

class DisarmedCommand final : public Plugin {
public:
  explicit DisarmedCommand(Services& services) : services_(services) { onReset("startup"); }
  std::string name() const override {
#ifdef MAVMOOS_EXAMPLE_CONFLICTING_OWNER
    return "example_disarm_conflicting_owner";
#else
    return module_id;
#endif
  }
  PluginDescriptor descriptor() const override {
    PluginDescriptor value;
    value.profile = module_id;
#ifndef MAVMOOS_EXAMPLE_CONFLICTING_OWNER
    value.capabilities = {{"example.disarmed_command.disarm", request_key,
      "schema=1,id=<id>,action=disarm,timeout=<seconds>", "request"}};
    value.exclusive_resources = {"command.example_disarm"};
#endif
    value.request_owners = {request_key};
    return value;
  }
  std::vector<std::string> subscriptions() const override { return {request_key}; }
  ModuleConfigResult configureModule(const std::vector<ModuleSetting>& settings) override {
    std::string source = "uPokeDB";
    double observation_age = maximum_observation_age;
    std::set<std::string> seen;
    for (const auto& setting : settings) {
      if (!seen.insert(setting.key).second)
        return {ModuleConfigStatus::Invalid, setting.key, "duplicate disarm setting"};
      if (setting.key == "request_source") {
        if (!token(setting.value, 48))
          return {ModuleConfigStatus::Invalid, setting.key, "expected a case-sensitive MOOS source token"};
        source = setting.value;
      } else if (setting.key == "observation_timeout") {
        if (!boundedSeconds(setting.value, .5, maximum_observation_age, observation_age))
          return {ModuleConfigStatus::Invalid, setting.key, "expected finite seconds in [0.5,1.5]"};
      } else {
        return {ModuleConfigStatus::UnknownKey, setting.key, "unknown disarm setting"};
      }
    }
    request_source_ = source;
    observation_age_ = observation_age;
    return {};
  }

  void onMail(const Mail& mail) override {
    if (mail.key != request_key) return;
    if (!advance()) {
      publish({}, "REJECTED", "owner_clock_invalid", false);
      return;
    }
    if (mail.source != request_source_) {
      publish({}, "REJECTED", "wrong_request_source", false);
      return;
    }
    const auto request = mail.numeric ? std::nullopt : parse(mail.text);
    if (!request) {
      publish({}, "REJECTED", "invalid_request", false);
      return;
    }
    const double expires = mail.published_at + request->timeout;
    if (!std::isfinite(mail.published_at) || !std::isfinite(mail.received_at) ||
        mail.published_at <= boundary_ || mail.published_at > mail.received_at ||
        mail.received_at > last_now_ || last_now_ - mail.published_at > 2 ||
        !std::isfinite(expires) || last_now_ >= expires) {
      publish(request->id, "REJECTED", "request_time_invalid_or_expired", false);
      return;
    }
    // Keep capacity for cancelling the last admitted operation.
    const bool active_cancel = !request->target.empty() && active_ &&
      active_->phase != Phase::Retired && active_->id == request->target;
    if (seen_.count(request->id) || (seen_.size() >= request_limit && !active_cancel)) {
      publish(request->id, "REJECTED", "duplicate_or_exhausted_request_history", false);
      return;
    }
    seen_.insert(request->id);
    if (!request->target.empty()) {
      cancel(*request);
      return;
    }
    if (active_ || services_.controlBusy() || services_.navigationGranted()) {
      publish(request->id, "REJECTED", "control_busy", false);
      return;
    }
    if (!freshGround()) {
      publish(request->id, "REJECTED", "fresh_disarmed_ground_required", false);
      return;
    }
    CommandRequest wire;
    wire.id = "example.disarm." + request->id;
    wire.command = MAV_CMD_COMPONENT_ARM_DISARM;
    wire.operation_expires = expires;
    // The packet cannot outlive the observations that admitted the request.
    const auto& vehicle = services_.vehicle();
    wire.expires = std::min({expires, vehicle.heartbeat_at + observation_age_,
                            vehicle.landed_at + observation_age_});
    active_.emplace(request->id, wire, mail.published_at);
    publishActive("REQUESTED", "explicit_disarmed_ground_request", false);
  }

  void onInputBoundary() override { advance(); }

  void iterate() override {
    if (!advance() || !active_ || active_->phase != Phase::Local) return;
    if (services_.controlBusy() || services_.navigationGranted()) {
      retire("REJECTED", "control_busy");
      return;
    }
    active_->phase = Phase::Submitted;
    active_->submitted_at = last_now_;
    if (!services_.sendCommand(active_->wire)) {
      publishActive("REJECTED", "host_admission_denied", false);
      active_.reset();
      return;
    }
    publishActive("SUBMITTED", "queued_with_original_expiry", false);
  }

  void onCommandSent(const CommandRequest& request, double at, bool accepted) override {
    if (!active_ || request.id != active_->wire.id || request.command != active_->wire.command ||
        request.params != active_->wire.params || request.expires != active_->wire.expires ||
        request.operation_expires != active_->wire.operation_expires ||
        !request.execution_id.empty() || !request.abort_id.empty()) return;
    if (active_->phase == Phase::Retired) {
      if (accepted) {
        active_->handoff_at = at;
        publishActive("LATE_HANDOFF", "accepted_after_local_retirement", true);
      }
      return;
    }
    if (active_->phase != Phase::Submitted) return;
    if (!accepted) {
      publishActive("FAILED", "local_handoff_not_accepted", false);
      services_.commandFinished(active_->wire.id);
      active_.reset();
      return;
    }
    active_->phase = Phase::Sent;
    active_->handoff_at = at;
    if (!advance()) return;
    if (!active_ || active_->phase != Phase::Sent) return;
    if (!std::isfinite(at) || at < active_->submitted_at || at > last_now_ ||
        at >= active_->wire.expires) {
      retire("FAILED", "accepted_handoff_time_invalid");
      return;
    }
    publishActive("SENT", "local_handoff_accepted", true);
  }

  void onMessageWithMetadata(const mavlink_message_t& message, const MessageTime& time) override {
    if (!advance() || !active_ || active_->phase != Phase::Sent) return;
    if (message.msgid == MAVLINK_MSG_ID_COMMAND_ACK && message.sysid == 1 && message.compid == 1 &&
        validAckFrame(message) && ackTime(time) && !active_->acknowledged) {
      mavlink_command_ack_t ack{};
      mavlink_msg_command_ack_decode(&message, &ack);
      if (ack.command == active_->wire.command && (!ack.target_system || ack.target_system == 245) &&
          (!ack.target_component || ack.target_component == 191)) {
        if (ack.result != MAV_RESULT_ACCEPTED && ack.result != MAV_RESULT_IN_PROGRESS) {
          retire("FAILED", "fc_rejected_disarm");
          return;
        }
        if (ack.result == MAV_RESULT_ACCEPTED) {
          active_->acknowledged = true;
          active_->ack_time = time;
          active_->ack_frame = frameHex(message);
          publishActive("ACKNOWLEDGED", "matching_accepted_protocol_ack", true);
        }
      }
    }
    const auto& vehicle = services_.vehicle();
    if (active_->acknowledged && vehicle.heartbeat_at > active_->handoff_at &&
        vehicle.landed_at > active_->handoff_at) {
      publishActive("COMPLETED", "fresh_disarmed_ground_observed", false);
      services_.commandFinished(active_->wire.id);
      active_.reset();
    }
  }

  void onReset(const std::string&) override {
    const auto now = services_.now();
    clock_fault_ = !std::isfinite(now) || now < 0 || now < last_now_;
    if (!clock_fault_) boundary_ = last_now_ = now;
    retire("LOST", "session_reset");
  }

private:
  enum class Phase { Local, Submitted, Sent, Retired };
  struct Operation {
    Operation(const std::string& request_id, const CommandRequest& request, double published)
      : id(request_id), wire(request), published_at(published) {}
    std::string id;
    CommandRequest wire;
    double published_at;
    double submitted_at = 0;
    Phase phase = Phase::Local;
    double handoff_at = 0;
    bool acknowledged = false;
    MessageTime ack_time{};
    std::string ack_frame;
  };

  bool freshGround() const {
    const auto& vehicle = services_.vehicle();
    const auto fresh = [this](double stamp) {
      return std::isfinite(stamp) && stamp > boundary_ && stamp <= last_now_ &&
             last_now_ < stamp + observation_age_;
    };
    return vehicle.heartbeat_valid && vehicle.landed_valid &&
      vehicle.type == MAV_TYPE_QUADROTOR && vehicle.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA &&
      !vehicle.armed && !vehicle.in_air && vehicle.landed_state == MAV_LANDED_STATE_ON_GROUND &&
      fresh(vehicle.heartbeat_at) && fresh(vehicle.landed_at);
  }
  bool ackTime(const MessageTime& time) const {
    return std::isfinite(time.effective_at) && std::isfinite(time.read_at) &&
      time.effective_at > boundary_ && time.effective_at <= time.read_at &&
      time.read_at > active_->handoff_at && time.read_at <= last_now_ &&
      last_now_ < time.effective_at + observation_age_ &&
      (time.age_basis == "QUEUE_EMPTY_FLOOR" ||
       (time.age_basis == "READ_ACQUISITION" && time.effective_at == time.read_at));
  }
  bool advance() {
    const auto now = services_.now();
    if (!std::isfinite(now) || now < 0 || now < last_now_) {
      clock_fault_ = true;
      retire("LOST", "owner_clock_invalid");
    }
    if (clock_fault_) return false;
    last_now_ = now;
    if (!active_) return true;
    if (now >= active_->wire.operation_expires) {
      retire("TIMED_OUT", "original_expiry_reached");
      active_.reset();
    } else if (active_->phase != Phase::Retired && !freshGround()) {
      retire("LOST", "disarmed_ground_evidence_lost");
    } else if (active_->phase == Phase::Local && now >= active_->wire.expires) {
      retire("TIMED_OUT", "packet_expiry_reached_before_submission");
    }
    return true;
  }
  void cancel(const Request& request) {
    if (!active_ || active_->phase == Phase::Retired || active_->id != request.target) {
      publish(request.id, "REJECTED", "cancel_target_not_active", false, request.target);
      return;
    }
    const bool uncertain = active_->phase != Phase::Local;
    retire("CANCELLED", "explicit_cancel");
    publish(request.id, "CANCEL_ACCEPTED", "target_retired_locally", uncertain, request.target);
  }
  void retire(const char* status, const char* reason) {
    if (!active_ || active_->phase == Phase::Retired) return;
    const bool uncertain = active_->phase != Phase::Local;
    publishActive(status, reason, uncertain);
    if (uncertain) {
      services_.cancelCommand(active_->wire.id);
      // The host's command lease and this local slot keep their original bound.
      active_->phase = Phase::Retired;
    } else {
      active_.reset();
    }
  }
  void publish(const std::string& id, const char* status, const char* reason, bool uncertain,
               const std::string& target = {}, const Operation* operation = nullptr) {
    std::ostringstream out;
    out << std::boolalpha << "{\"schema\":1,\"id\":\"" << id << "\",\"status\":\"" << status
        << "\",\"reason\":\"" << reason << "\",\"observed_at\":" << number(last_now_)
        << ",\"remote_effect_uncertain\":" << uncertain;
    if (!target.empty()) out << ",\"target\":\"" << target << "\"";
    if (operation) {
      const auto& vehicle = services_.vehicle();
      out << ",\"wire_id\":\"" << operation->wire.id
          << "\",\"published_at\":" << number(operation->published_at)
          << ",\"operation_expires\":" << number(operation->wire.operation_expires)
          << ",\"packet_expires\":" << number(operation->wire.expires)
          << ",\"handoff_at\":" << (operation->handoff_at > 0 ? number(operation->handoff_at) : "null")
          << ",\"heartbeat_at\":" << number(vehicle.heartbeat_at)
          << ",\"landed_at\":" << number(vehicle.landed_at)
          << ",\"disarmed_ground_fresh\":" << (!clock_fault_ && freshGround());
      if (operation->acknowledged) {
        out << ",\"ack_effective_at\":" << number(operation->ack_time.effective_at)
            << ",\"ack_read_at\":" << number(operation->ack_time.read_at)
            << ",\"ack_age_basis\":\"" << operation->ack_time.age_basis
            << "\",\"ack_frame_hex\":\"" << operation->ack_frame << "\"";
      }
    }
    out << '}';
    services_.publish(result_key, out.str());
  }
  void publishActive(const char* status, const char* reason, bool uncertain) {
    publish(active_->id, status, reason, uncertain, {}, &*active_);
  }

  Services& services_;
  std::string request_source_ = "uPokeDB";
  std::set<std::string> seen_;
  std::optional<Operation> active_;
  double observation_age_ = maximum_observation_age;
  double boundary_ = 0, last_now_ = 0;
  bool clock_fault_ = false;
};
}

#ifndef MAVMOOS_EXAMPLE_ABI_DELTA
#define MAVMOOS_EXAMPLE_ABI_DELTA 0
#endif
extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi, mavmoos::runtime::Services* services) {
  return abi == mavmoos::runtime::plugin_abi + MAVMOOS_EXAMPLE_ABI_DELTA && services
    ? new mavmoos::example::DisarmedCommand(*services) : nullptr;
}
extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin) { delete plugin; }
