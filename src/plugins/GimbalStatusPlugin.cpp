#include "PassiveObserver.h"
#include "PayloadService.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <initializer_list>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>

namespace mavmoos::runtime {
namespace {

constexpr unsigned device_count = 7;
constexpr uint8_t controller_system = 245, controller_component = 191;
constexpr double mail_lifetime = 2, operation_lifetime = 8, status_lifetime = 1;
constexpr std::size_t request_limit = 128;
constexpr char request_key[] = "MAVMOOS_GIMBAL_REQUEST";
constexpr char result_key[] = "MAVMOOS_GIMBAL_RESULT";
constexpr float pitch_min_deg = -75, pitch_max_deg = 20;
constexpr float yaw_min_deg = -90, yaw_max_deg = 90;

bool identifier(const std::string& value)
{
  return !value.empty() && value.size() <= 40 &&
    std::all_of(value.begin(), value.end(), [](unsigned char c) {
      return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
             (c >= '0' && c <= '9') || c == '_' || c == '.' || c == '-';
    });
}

std::map<std::string, std::string> parse(const std::string& input)
{
  if(input.empty() || input.size() > 180 || input.back() == ',') return {};
  std::map<std::string, std::string> result;
  std::istringstream fields(input);
  for(std::string field; std::getline(fields, field, ',');) {
    const auto equals = field.find('=');
    if(equals == std::string::npos || equals == 0 || equals + 1 == field.size() ||
       !result.emplace(field.substr(0, equals), field.substr(equals + 1)).second) return {};
  }
  return result;
}

bool angle(const std::string& input, float minimum, float maximum, float& value)
{
  const auto* end = input.data() + input.size();
  const auto parsed = std::from_chars(input.data(), end, value);
  return parsed.ec == std::errc{} && parsed.ptr == end && std::isfinite(value) &&
         value >= minimum && value <= maximum;
}

// Half-range ordering accepts the ordinary 32-bit millisecond wrap and rejects
// repeated or older reports without converting a remote clock to local time.
bool advances(uint32_t next, uint32_t previous)
{
  const uint32_t delta = next - previous;
  return delta && delta < uint32_t{0x80000000};
}

bool primaryFree(const mavlink_gimbal_manager_status_t& value)
{
  return value.primary_control_sysid == 0 && value.primary_control_compid == 0;
}

bool primaryOurs(const mavlink_gimbal_manager_status_t& value)
{
  return value.primary_control_sysid == controller_system && value.primary_control_compid == controller_component;
}

bool secondaryFree(const mavlink_gimbal_manager_status_t& value)
{
  return value.secondary_control_sysid == 0 && value.secondary_control_compid == 0;
}

bool managerConflict(const mavlink_gimbal_manager_status_t& value)
{
  return !secondaryFree(value) ||
         (value.flags & (GIMBAL_MANAGER_FLAGS_RC_EXCLUSIVE | GIMBAL_MANAGER_FLAGS_RC_MIXED));
}

std::optional<std::pair<double, double>> vehicleAngles(const mavlink_gimbal_device_attitude_status_t& value)
{
  constexpr unsigned prohibited = GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME | GIMBAL_DEVICE_FLAGS_YAW_LOCK |
    GIMBAL_DEVICE_FLAGS_PITCH_LOCK | GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE | GIMBAL_DEVICE_FLAGS_RC_MIXED |
    GIMBAL_DEVICE_FLAGS_RETRACT | GIMBAL_DEVICE_FLAGS_NEUTRAL;
  if(!(value.flags & GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME) || (value.flags & prohibited) || value.failure_flags)
    return {};
  double norm_squared = 0;
  for(float item : value.q) norm_squared += double(item) * item;
  const double norm = std::sqrt(norm_squared);
  if(!std::isfinite(norm) || std::abs(norm - 1) > .02) return {};
  const double w = value.q[0] / norm, x = value.q[1] / norm, y = value.q[2] / norm, z = value.q[3] / norm;
  constexpr double degrees = 57.29577951308232;
  const double pitch = std::asin(std::clamp(2 * (w * y - z * x), -1., 1.));
  const double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return std::pair{pitch * degrees, yaw * degrees};
}

std::string text(const char* data, std::size_t length)
{
  constexpr char hex[] = "0123456789abcdef";
  std::string display = "\"";
  std::string bytes;
  for(std::size_t i = 0; i < length && data[i] != 0; ++i) {
    const auto byte = static_cast<unsigned char>(data[i]);
    bytes += hex[byte >> 4];
    bytes += hex[byte & 15];
    if(byte == '"' || byte == '\\') {
      display += '\\';
      display += static_cast<char>(byte);
    } else if(byte >= 32 && byte <= 126) {
      display += static_cast<char>(byte);
    } else {
      display += "\\u00";
      display += hex[byte >> 4];
      display += hex[byte & 15];
    }
  }
  return "{\"text\":" + display + "\",\"raw_hex\":\"" + bytes + "\"}";
}

void number(std::ostream& out, double value)
{
  if(std::isfinite(value)) out << value;
  else out << "null";
}

const char* numericState(float value)
{
  if(std::isnan(value)) return "wire_nan";
  if(!std::isfinite(value)) return value > 0 ? "invalid_positive_infinity" : "invalid_negative_infinity";
  return "reported";
}

void reported(std::ostream& out, float value)
{
  out << "{\"value\":";
  number(out, value);
  out << ",\"state\":\"" << numericState(value) << "\"}";
}

void version(std::ostream& out, uint32_t raw)
{
  out << "{\"raw\":" << raw << ",\"major\":" << (raw & 255) << ",\"minor\":" << ((raw >> 8) & 255)
      << ",\"patch\":" << ((raw >> 16) & 255) << ",\"dev\":" << (raw >> 24) << '}';
}

using Flag = std::pair<uint32_t, const char*>;

void flags(std::ostream& out, uint32_t raw, std::initializer_list<Flag> names)
{
  uint32_t unknown = raw;
  bool first = true;
  out << "{\"raw\":" << raw << ",\"asserted\":[";
  for(const auto& [bit, name] : names) {
    unknown &= ~bit;
    if(!(raw & bit)) continue;
    if(!first) out << ',';
    first = false;
    out << '"' << name << '"';
  }
  out << "],\"unknown_mask\":" << unknown << '}';
}

void limits(std::ostream& out, float minimum, float maximum)
{
  out << "{\"min\":";
  reported(out, minimum);
  out << ",\"max\":";
  reported(out, maximum);
  out << ",\"ordered\":";
  if(std::isfinite(minimum) && std::isfinite(maximum)) out << (minimum <= maximum);
  else out << "null";
  out << '}';
}

void header(std::ostream& out, uint32_t boot, uint8_t length, uint8_t device_id, bool id_present)
{
  out << std::boolalpha << std::setprecision(17)
      << "{\"time_boot_ms_raw\":" << boot << ",\"remote_clock\":\"unmapped_boot_ms\",\"payload_length\":"
      << unsigned(length) << ",\"gimbal_device_id_raw\":";
  if(id_present) out << unsigned(device_id);
  else out << "null";
}

std::string information(const mavlink_message_t& message, const mavlink_gimbal_device_information_t& value)
{
  std::ostringstream out;
  out.imbue(std::locale::classic());
  header(out, value.time_boot_ms, message.len, value.gimbal_device_id,
         message.len > offsetof(mavlink_gimbal_device_information_t, gimbal_device_id));
  const bool extended = message.len > offsetof(mavlink_gimbal_device_information_t, cap_flags2);
  const bool prefer_extended = extended && value.cap_flags2 != 0;
  out << ",\"uid_raw\":\"" << value.uid << "\",\"uid_known\":" << (value.uid != 0)
      << ",\"vendor\":" << text(value.vendor_name, sizeof(value.vendor_name))
      << ",\"model\":" << text(value.model_name, sizeof(value.model_name))
      << ",\"custom_name\":" << text(value.custom_name, sizeof(value.custom_name))
      << ",\"firmware_version\":";
  version(out, value.firmware_version);
  out << ",\"hardware_version\":";
  version(out, value.hardware_version);
  out << ",\"cap_flags_raw\":" << value.cap_flags << ",\"cap_flags2_raw\":";
  if(extended) out << value.cap_flags2;
  else out << "null";
  out << ",\"capability_basis\":\"" << (prefer_extended ? "cap_flags2" : "cap_flags") << "\""
      << ",\"cap_flags_lower_bits_match\":";
  if(prefer_extended) out << ((value.cap_flags2 & 65535u) == value.cap_flags);
  else out << "null";
  out << ",\"custom_cap_flags_raw\":" << value.custom_cap_flags << ",\"capabilities\":";
  flags(out, prefer_extended ? value.cap_flags2 : value.cap_flags, {
    {GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT, "retract"},
    {GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL, "neutral"},
    {GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS, "roll_axis"},
    {GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW, "roll_follow"},
    {GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK, "roll_lock"},
    {GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS, "pitch_axis"},
    {GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW, "pitch_follow"},
    {GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK, "pitch_lock"},
    {GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS, "yaw_axis"},
    {GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW, "yaw_follow"},
    {GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK, "yaw_lock"},
    {GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW, "infinite_yaw"},
    {GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME, "earth_frame_yaw"},
    {GIMBAL_DEVICE_CAP_FLAGS_HAS_RC_INPUTS, "rc_inputs"},
    {GIMBAL_DEVICE_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL, "point_local"},
    {GIMBAL_DEVICE_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL, "point_global"},
  });
  out << ",\"angular_limits_rad\":{\"roll\":";
  limits(out, value.roll_min, value.roll_max);
  out << ",\"pitch\":";
  limits(out, value.pitch_min, value.pitch_max);
  out << ",\"yaw\":";
  limits(out, value.yaw_min, value.yaw_max);
  out << "}}";
  return out.str();
}

void optionalDelta(std::ostream& out, float value, bool on_wire, bool usable)
{
  out << "{\"raw\":";
  if(on_wire) number(out, value);
  else out << "null";
  out << ",\"value\":";
  if(on_wire && usable) number(out, value);
  else out << "null";
  out << ",\"state\":\"" << (!on_wire ? "extension_not_on_wire" :
                               !usable ? "explicit_yaw_frame_required" : numericState(value))
      << "\",\"raw_state\":\"" << (on_wire ? numericState(value) : "extension_not_on_wire") << "\"}";
}

std::string attitude(const mavlink_message_t& message, const mavlink_gimbal_device_attitude_status_t& value)
{
  std::ostringstream out;
  out.imbue(std::locale::classic());
  header(out, value.time_boot_ms, message.len, value.gimbal_device_id,
         message.len > offsetof(mavlink_gimbal_device_attitude_status_t, gimbal_device_id));
  const bool vehicle = value.flags & GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME;
  const bool earth = value.flags & GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME;
  const bool locked = value.flags & GIMBAL_DEVICE_FLAGS_YAW_LOCK;
  const bool conflict = vehicle && earth;
  const bool explicit_frame = vehicle != earth;
  const char* yaw_frame = "vehicle_heading";
  if(conflict) yaw_frame = "conflicting";
  else if(earth || (!vehicle && locked)) yaw_frame = "earth_north";
  out << ",\"target_system_raw\":" << unsigned(value.target_system)
      << ",\"target_component_raw\":" << unsigned(value.target_component)
      << ",\"yaw_frame\":\"" << yaw_frame
      << "\",\"yaw_frame_basis\":\"" << (conflict ? "conflicting_flags" : explicit_frame ? "explicit_flag" : "legacy_yaw_lock")
      << "\",\"quaternion_wxyz\":[";
  double norm_squared = 0;
  for(unsigned axis = 0; axis < 4; ++axis) {
    if(axis) out << ',';
    number(out, value.q[axis]);
    norm_squared += double(value.q[axis]) * value.q[axis];
  }
  out << "],\"quaternion_component_states\":[";
  for(unsigned axis = 0; axis < 4; ++axis) {
    if(axis) out << ',';
    out << '"' << numericState(value.q[axis]) << '"';
  }
  out << "],\"quaternion_unit_within_tolerance\":";
  if(std::isfinite(norm_squared)) out << (std::abs(std::sqrt(norm_squared) - 1.) <= .02);
  else out << "null";
  out << ",\"angular_velocity_rad_s\":{\"roll_x\":";
  reported(out, value.angular_velocity_x);
  out << ",\"pitch_y\":";
  reported(out, value.angular_velocity_y);
  out << ",\"yaw_z\":";
  reported(out, value.angular_velocity_z);
  out << "},\"flags\":";
  flags(out, value.flags, {
    {GIMBAL_DEVICE_FLAGS_RETRACT, "retract"}, {GIMBAL_DEVICE_FLAGS_NEUTRAL, "neutral"},
    {GIMBAL_DEVICE_FLAGS_ROLL_LOCK, "roll_lock"}, {GIMBAL_DEVICE_FLAGS_PITCH_LOCK, "pitch_lock"},
    {GIMBAL_DEVICE_FLAGS_YAW_LOCK, "yaw_lock"}, {GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME, "yaw_vehicle"},
    {GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME, "yaw_earth"},
    {GIMBAL_DEVICE_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME, "accepts_yaw_earth"},
    {GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE, "rc_exclusive"}, {GIMBAL_DEVICE_FLAGS_RC_MIXED, "rc_mixed"},
  });
  out << ",\"failure_flags\":";
  flags(out, value.failure_flags, {
    {GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT, "roll_limit"},
    {GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT, "pitch_limit"},
    {GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT, "yaw_limit"},
    {GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR, "encoder_error"},
    {GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR, "power_error"},
    {GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR, "motor_error"},
    {GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR, "software_error"},
    {GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR, "communications_error"},
    {GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING, "calibrating"},
    {GIMBAL_DEVICE_ERROR_FLAGS_NO_MANAGER, "no_manager"},
  });
  out << ",\"delta_yaw_rad\":";
  optionalDelta(out, value.delta_yaw, message.len > offsetof(mavlink_gimbal_device_attitude_status_t, delta_yaw),
                explicit_frame);
  out << ",\"delta_yaw_velocity_rad_s\":";
  optionalDelta(out, value.delta_yaw_velocity,
                message.len > offsetof(mavlink_gimbal_device_attitude_status_t, delta_yaw_velocity), explicit_frame);
  out << '}';
  return out.str();
}

class GimbalStatus final : public PassiveObserver, public PayloadListener {
public:
  explicit GimbalStatus(Services& services)
    : PassiveObserver(services, true), m_payload(dynamic_cast<PayloadService*>(&services)), m_boundary(services.now())
  {
    setObservationComponent(0);
  }

  std::string name() const override { return "gimbal_status"; }

  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.profile = "gimbal_device_observations";
    result.capabilities = {{"gimbal.device_information", "", "", "observation"},
                           {"gimbal.attitude_status", "", "", "observation"}};
    result.exclusive_resources = {"observation.gimbal_device"};
    result.streamed_parameter_bench_compatible = !m_point_enabled;
    if(m_point_enabled) {
      result.capabilities.push_back({"gimbal.manager_status", "", "", "observation"});
      result.capabilities.push_back({"gimbal.control.point", request_key, "action=point"});
      result.capabilities.push_back({"gimbal.control.cancel", request_key, "action=cancel"});
      result.exclusive_resources.push_back("protocol.gimbal_control");
      result.request_owners = {request_key};
      result.observation_streams = {{MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 250000.f}};
    }
    return result;
  }

  void configure(const std::string& key, const std::string& value) override
  {
    if(key != "gimbal_component") return;
    unsigned component = 0;
    const auto* end = value.data() + value.size();
    const auto parsed = std::from_chars(value.data(), end, component);
    if(parsed.ec != std::errc{} || parsed.ptr != end || component < 2 || component > 255)
      throw std::invalid_argument("gimbal_component must name a distinct component in [2,255]");
    if(observationComponent() && observationComponent() != component)
      throw std::invalid_argument("gimbal_component is fixed for the lifetime of the provider");
    setObservationComponent(component);
  }

  ModuleConfigResult configureModule(const std::vector<ModuleSetting>& settings) override
  {
    if(!observationComponent())
      return {ModuleConfigStatus::Invalid, "gimbal_component", "required for gimbal_status"};
    std::vector<ModuleSetting> observer;
    std::optional<bool> enabled;
    for(const auto& setting : settings) {
      if(setting.key != "point_enabled") {
        observer.push_back(setting);
        continue;
      }
      if(enabled || (setting.value != "true" && setting.value != "false"))
        return {ModuleConfigStatus::Invalid, setting.key, "expected a single true or false"};
      enabled = setting.value == "true";
    }
    if(enabled.value_or(false) && (!m_payload || observationComponent() < 7))
      return {ModuleConfigStatus::Invalid, "point_enabled", "requires payload service and a component-native gimbal"};
    auto result = PassiveObserver::configureModule(observer);
    if(result.status == ModuleConfigStatus::Accepted) m_point_enabled = enabled.value_or(false);
    return result;
  }

  std::vector<std::string> subscriptions() const override
  {
    return m_point_enabled ? std::vector<std::string>{request_key} : std::vector<std::string>{};
  }

  void onMail(const Mail& mail) override
  {
    if(!m_point_enabled || mail.key != request_key) return;
    const auto fields = mail.numeric ? std::map<std::string, std::string>{} : parse(mail.text);
    const auto read = [&](const char* key) {
      const auto it = fields.find(key);
      return it == fields.end() ? std::string{} : it->second;
    };
    const auto id = read("id"), action = read("action"), operation_id = read("operation_id");
    float pitch = 0, yaw = 0;
    const bool pointing = action == "point" && fields.size() == 5 &&
      angle(read("pitch_deg"), pitch_min_deg, pitch_max_deg, pitch) &&
      angle(read("yaw_deg"), yaw_min_deg, yaw_max_deg, yaw);
    const bool cancelling = action == "cancel" && fields.size() == 4 && identifier(operation_id);
    if(read("schema") != "1" || !identifier(id) || (!pointing && !cancelling)) {
      report(identifier(id) ? id : "invalid", "REJECTED", "invalid_request");
      return;
    }
    const double now = services().now();
    const double age = mail.received_at - mail.published_at;
    if(!std::isfinite(now) || !std::isfinite(age) || age < -.05 || age > mail_lifetime ||
       mail.published_at <= m_boundary || mail.received_at > now || now - mail.received_at > mail_lifetime) {
      report(id, "REJECTED", "stale_future_or_pre_session_request");
      return;
    }
    if(m_seen.count(id) || m_seen.size() >= request_limit + (cancelling ? 1 : 0)) {
      report(id, "REJECTED", "duplicate_or_history_full");
      return;
    }
    advanceOperation();
    if(cancelling) {
      if(!m_operation || m_operation->id != operation_id || m_operation->cancelled) {
        report(id, "REJECTED", "no_matching_active_operation");
        return;
      }
      m_seen.insert(id);
      report(id, "CANCEL_ACCEPTED", "operator_cancel");
      m_operation->cancelled = true;
      if(m_operation->phase == Phase::Release) return;
      if(!m_operation->claim_sent) {
        finish("CANCELLED", "claim_retired", "unmodified");
      } else {
        if(m_operation->phase == Phase::Point) release("operator_cancel");
        else {
          m_payload->cancelPayload(hostId());
          advanceOperation();
        }
      }
      return;
    }
    if(m_operation) {
      report(id, "REJECTED", "busy");
      return;
    }
    if(!fresh(m_manager_stamp) || !primaryFree(m_manager) || managerConflict(m_manager) || !freshPose() ||
       !m_payload->payloadReady(PayloadAction::GimbalClaim)) {
      report(id, "REJECTED", "fresh_free_manager_and_vehicle_pose_required");
      return;
    }
    m_seen.insert(id);
    m_operation.emplace();
    m_operation->id = id;
    m_operation->pitch = pitch;
    m_operation->yaw = yaw;
    m_operation->deadline = mail.published_at + operation_lifetime;
    if(queue(Phase::Claim)) report(id, "QUEUED", "original_deadline_bound");
    else finish("REJECTED", "claim_not_admitted", "unmodified");
  }

  void onPayloadSent(const PayloadRequest& request, double sent_at, bool accepted) override
  {
    if(!m_operation || request.operation_id != hostId() ||
       request.id != packetId(m_operation->phase) || request.action != action(m_operation->phase) ||
       request.operation_expires != m_operation->deadline || m_operation->sent || !std::isfinite(sent_at) ||
       sent_at < m_operation->phase_start || sent_at > services().now()) return;
    const auto phase = m_operation->phase;
    if(!accepted) {
      if(phase == Phase::Claim) finish("FAILED", "claim_not_sent", "unmodified");
      else if(phase == Phase::Point) release("point_not_sent");
      else finish("UNCERTAIN", "release_not_sent", "uncertain");
      return;
    }
    m_operation->sent = sent_at;
    if(phase == Phase::Claim) m_operation->claim_sent = true;
    if(phase == Phase::Point) m_operation->point_sent = true;
    report(m_operation->id, "SENT", phaseName(phase));
  }

private:
  enum class Phase { Claim, Point, Release };
  struct Operation {
    std::string id, terminal_reason;
    float pitch = 0, yaw = 0;
    double deadline = 0, phase_start = 0, phase_end = 0;
    std::optional<double> sent;
    Phase phase = Phase::Claim;
    bool claim_sent = false, point_sent = false, claim_observed = false, pose_observed = false, cancelled = false;
  };

  static const char* phaseName(Phase phase)
  {
    switch(phase) {
    case Phase::Claim: return "claim";
    case Phase::Point: return "point";
    case Phase::Release: return "release";
    }
    return "invalid";
  }

  static PayloadAction action(Phase phase)
  {
    switch(phase) {
    case Phase::Claim: return PayloadAction::GimbalClaim;
    case Phase::Point: return PayloadAction::GimbalPoint;
    case Phase::Release: return PayloadAction::GimbalRelease;
    }
    return PayloadAction::GimbalRelease;
  }

  std::string hostId() const { return "gimbal/" + m_operation->id; }
  std::string packetId(Phase phase) const { return hostId() + "-" + phaseName(phase); }

  bool fresh(const Stamp& stamp)
  {
    const auto now = services().now();
    return stamp.id && now >= stamp.time.effective_at && now - stamp.time.effective_at <= status_lifetime;
  }

  bool freshPose() { return m_pose && fresh(m_pose_stamp); }

  bool afterSend(const Stamp& stamp)
  {
    return m_operation->sent && fresh(stamp) && stamp.time.effective_at > *m_operation->sent;
  }

  bool queue(Phase phase)
  {
    const auto now = services().now();
    auto& op = *m_operation;
    op.phase = phase;
    op.phase_start = now;
    op.phase_end = std::min(op.deadline, now + (phase == Phase::Point ? 3. : 2.));
    op.sent.reset();
    PayloadRequest request;
    request.id = packetId(phase);
    request.operation_id = hostId();
    request.action = action(phase);
    request.expires = std::min(op.phase_end, now + .75);
    request.operation_expires = op.deadline;
    if(phase == Phase::Point) {
      request.pitch_deg = op.pitch;
      request.yaw_deg = op.yaw;
    }
    return now < request.expires && m_payload->payload(request);
  }

  void report(const std::string& id, const char* status, const char* reason, const char* ownership = "unassessed")
  {
    std::string row = "schema=1,id=" + id + ",status=" + status + ",reason=" + reason +
      ",remote_ownership=" + ownership;
    if(m_operation && id == m_operation->id) {
      row += std::string(",claim_observed=") + (m_operation->claim_observed ? "true" : "false") +
        ",point_sent=" + (m_operation->point_sent ? "true" : "false") +
        ",pose_observed=" + (m_operation->pose_observed ? "true" : "false");
    }
    services().publish(result_key, row);
  }

  void finish(const char* status, const char* reason, const char* ownership)
  {
    m_payload->finishPayload(hostId());
    report(m_operation->id, status, reason, ownership);
    m_operation.reset();
  }

  void release(const char* reason)
  {
    auto& op = *m_operation;
    op.terminal_reason = reason;
    m_payload->cancelPayload(hostId());
    if(!fresh(m_manager_stamp) || !primaryOurs(m_manager) || !secondaryFree(m_manager) ||
       !queue(Phase::Release))
      finish("UNCERTAIN", "release_not_admitted", "uncertain");
  }

  void advanceOperation()
  {
    if(!m_operation) return;
    auto& op = *m_operation;
    const bool manager_fresh = fresh(m_manager_stamp);
    const double now = services().now();
    if(!std::isfinite(now) || now >= op.deadline) {
      finish(op.claim_sent ? "UNCERTAIN" : "FAILED", "original_deadline_elapsed",
             op.claim_sent ? "uncertain" : "unmodified");
      return;
    }
    if(manager_fresh && !primaryFree(m_manager) && !primaryOurs(m_manager)) {
      finish(op.cancelled ? "CANCELLED" : "FAILED", "manager_taken_over", "foreign_observed");
      return;
    }
    if(manager_fresh && primaryFree(m_manager) && !secondaryFree(m_manager)) {
      finish(op.cancelled ? "CANCELLED" : "FAILED", "manager_secondary_controller", "conflict_observed");
      return;
    }
    if(op.phase == Phase::Claim) {
      if(afterSend(m_manager_stamp) && primaryOurs(m_manager)) {
        op.claim_observed = true;
        if(op.cancelled || managerConflict(m_manager) || !freshPose()) {
          release(op.cancelled ? "operator_cancel" : "pose_or_ownership_unavailable");
        } else if(!queue(Phase::Point)) {
          release("point_not_admitted");
        }
      } else if(now >= op.phase_end) {
        finish(op.claim_sent ? "UNCERTAIN" : "FAILED", "claim_unconfirmed",
               op.claim_sent ? "uncertain" : "unmodified");
      }
      return;
    }
    if(op.phase == Phase::Point) {
      if(manager_fresh && primaryFree(m_manager)) {
        finish("FAILED", "manager_relinquished", "free_observed");
      } else if(!manager_fresh) {
        finish("UNCERTAIN", "manager_status_stale", "uncertain");
      } else if(managerConflict(m_manager)) {
        release("manager_conflict");
      } else if(!op.sent && !freshPose()) {
        release("pose_lost_before_point");
      } else if(afterSend(m_pose_stamp) && m_pose &&
                std::abs(m_pose->first - op.pitch) <= 2 && std::abs(m_pose->second - op.yaw) <= 2) {
        op.pose_observed = true;
        report(op.id, "POINTED", "matching_vehicle_frame_pose");
        release("point_released");
      } else if(now >= op.phase_end) {
        release("pose_unconfirmed");
      }
      return;
    }
    if(afterSend(m_manager_stamp) && primaryFree(m_manager)) {
      const auto reason = op.terminal_reason;
      finish(op.cancelled ? "CANCELLED" : op.pose_observed ? "CONFIRMED" : "FAILED",
             reason.c_str(), "free_observed");
    } else if(manager_fresh && primaryFree(m_manager) && !op.sent) {
      finish(op.cancelled ? "CANCELLED" : "FAILED", "manager_relinquished", "free_observed");
    } else if(now >= op.phase_end) {
      finish("UNCERTAIN", "release_unconfirmed", "uncertain");
    }
  }

  struct Sample {
    Stamp stamp;
    std::string data;
  };
  using Devices = std::array<Sample, device_count>;

  bool observes(uint32_t id) const override
  {
    return id == MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION || id == MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS ||
      (m_point_enabled && id == MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS);
  }

  template<typename Report, typename Decode, typename Data>
  void capture(Devices& devices, const mavlink_message_t& message, const MessageTime& time,
               unsigned maximum, Decode decode, Data data)
  {
    if(!wireShape(message, 0, maximum)) {
      rejectWire();
      return;
    }
    Report value{};
    decode(&message, &value);
    if(value.gimbal_device_id >= devices.size()) {
      rejectWire();
      return;
    }
    auto& sample = devices[value.gimbal_device_id];
    if(update(sample.stamp, time)) sample.data = data(message, value);
  }

  void observe(const mavlink_message_t& message, const MessageTime& time) override
  {
    if(message.msgid == MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION)
      capture<mavlink_gimbal_device_information_t>(m_information, message, time,
        MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN, mavlink_msg_gimbal_device_information_decode, information);
    else if(message.msgid == MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS)
      observeManager(message, time);
    else {
      capture<mavlink_gimbal_device_attitude_status_t>(m_attitude, message, time,
        MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS_LEN, mavlink_msg_gimbal_device_attitude_status_decode,
        [this, &time](const auto& msg, const auto& value) {
          if(m_point_enabled && value.gimbal_device_id == 0) observePose(value, time);
          return attitude(msg, value);
        });
      if(m_point_enabled) advanceOperation();
    }
  }

  void observeManager(const mavlink_message_t& message, const MessageTime& time)
  {
    mavlink_gimbal_manager_status_t value{};
    mavlink_msg_gimbal_manager_status_decode(&message, &value);
    if(!wireShape(message, 0, MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS_LEN) ||
       value.gimbal_device_id != observationComponent()) {
      rejectWire();
      return;
    }
    if(m_manager_stamp.id && !advances(value.time_boot_ms, m_manager.time_boot_ms)) return;
    if(update(m_manager_stamp, time)) {
      m_manager = value;
      advanceOperation();
    }
  }

  void observePose(const mavlink_gimbal_device_attitude_status_t& value, const MessageTime& time)
  {
    const bool addressed = value.target_system == controller_system && value.target_component == controller_component;
    if(!addressed && (value.target_system || value.target_component)) return;
    if(m_pose_stamp.id && !advances(value.time_boot_ms, m_pose_boot)) return;
    if(update(m_pose_stamp, time)) {
      m_pose_boot = value.time_boot_ms;
      m_pose = vehicleAngles(value);
    }
  }

  void clearSamples() override
  {
    m_information = {};
    m_attitude = {};
    m_manager = {};
    m_manager_stamp = {};
    m_pose.reset();
    m_pose_stamp = {};
    const auto now = services().now();
    if(std::isfinite(now) && now >= m_boundary) m_boundary = now;
    if(m_operation)
      finish(m_operation->claim_sent ? "UNCERTAIN" : "FAILED", "session_reset",
             m_operation->claim_sent ? "uncertain" : "unmodified");
  }

  void publish(const char* key, const char* family, const Devices& devices)
  {
    std::string list;
    for(unsigned id = 0; id < devices.size(); ++id) {
      const auto& sample = devices[id];
      if(!sample.stamp.id) continue;
      if(!list.empty()) list += ',';
      list += "{\"device_id\":" + std::to_string(id) + ",\"metadata\":" + metadata(sample.stamp) +
        ",\"data\":" + sample.data + '}';
    }
    const std::string present = list.empty() ? "false" : "true";
    services().publish(key, std::string("{\"schema\":1,\"family\":\"") + family +
      "\",\"read_only\":true,\"authoritative_for_control\":false,\"atomic\":false,\"present\":" + present +
      ",\"device_capacity\":" + std::to_string(device_count) + ",\"status\":" + metadata({}) +
      ",\"devices\":[" + list + "]}");
  }

  void publishSamples() override
  {
    publish("MAVMOOS_GIMBAL_DEVICE_INFORMATION", "GIMBAL_DEVICE_INFORMATION", m_information);
    publish("MAVMOOS_GIMBAL_DEVICE_ATTITUDE_STATUS", "GIMBAL_DEVICE_ATTITUDE_STATUS", m_attitude);
    if(!m_point_enabled) return;
    std::ostringstream out;
    out << "{\"schema\":1,\"metadata\":" << metadata(m_manager_stamp) << ",\"manager\":";
    if(!m_manager_stamp.id) out << "null";
    else out << "{\"device_id\":" << unsigned(m_manager.gimbal_device_id)
             << ",\"time_boot_ms_raw\":" << m_manager.time_boot_ms << ",\"flags_raw\":" << m_manager.flags
             << ",\"primary_system\":" << unsigned(m_manager.primary_control_sysid)
             << ",\"primary_component\":" << unsigned(m_manager.primary_control_compid)
             << ",\"secondary_system\":" << unsigned(m_manager.secondary_control_sysid)
             << ",\"secondary_component\":" << unsigned(m_manager.secondary_control_compid) << '}';
    out << '}';
    services().publish("MAVMOOS_GIMBAL_MANAGER_STATUS", out.str());
    advanceOperation();
  }

  Devices m_information;
  Devices m_attitude;
  PayloadService* const m_payload;
  double m_boundary;
  bool m_point_enabled = false;
  Stamp m_manager_stamp, m_pose_stamp;
  mavlink_gimbal_manager_status_t m_manager{};
  uint32_t m_pose_boot = 0;
  std::optional<std::pair<double, double>> m_pose;
  std::optional<Operation> m_operation;
  std::set<std::string> m_seen;
};

} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi, mavmoos::runtime::Services* services)
{
  return abi == mavmoos::runtime::plugin_abi && services ? new mavmoos::runtime::GimbalStatus(*services) : nullptr;
}

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin)
{
  delete plugin;
}
