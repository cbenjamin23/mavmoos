/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: CameraStatusPlugin.cpp                          */
/*    DATE: November 6, 2026                                */
/************************************************************/
#include "PassiveObserver.h"
#include "PayloadService.h"
#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <optional>
#include <set>
#include <string_view>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace mavmoos::runtime {
namespace {
constexpr std::size_t storage_capacity = 8;

bool nonnegative(float value)
{ return std::isnan(value) || (std::isfinite(value) && value >= 0); }

bool percentage(float value)
{ return nonnegative(value) && (std::isnan(value) || value <= 100); }

void optionalFloat(std::ostream& out, float value, bool reported = true)
{
  if (reported && std::isfinite(value)) out << value;
  else out << "null";
}

void optionalInteger(std::ostream& out, unsigned value, bool reported = true)
{
  if (reported) out << value;
  else out << "null";
}

template<typename Char, std::size_t Size>
void boundedText(std::ostream& out, const Char (&text)[Size])
{
  if (text[0] == 0) { out << "null";return; }
  constexpr char hex[] = "0123456789abcdef";
  out << '"';
  for (const unsigned char value : text) {
    if (!value) break;
    if (value == '"' || value == '\\') out << '\\' << char(value);
    else if (value < 32 || value >= 127) out << "\\u00" << hex[value >> 4] << hex[value & 15];
    else out << char(value);
  }
  out << '"';
}

template<std::size_t Size>
const char* label(unsigned value, const std::array<const char*, Size>& labels)
{ return value < labels.size() ? labels[value] : "UNRECOGNIZED"; }

std::ostringstream beginData(const mavlink_message_t& message, uint32_t boot_ms)
{
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<float>::max_digits10) << "{\"message_id\":" << message.msgid
      << ",\"payload_length\":" << unsigned(message.len) << ",\"time_boot_ms\":" << boot_ms
      << ",\"remote_clock\":\"camera_boot_ms_unmapped_uint32\"";
  return out;
}

std::string information(const mavlink_message_t& message)
{
  mavlink_camera_information_t value{};
  mavlink_msg_camera_information_decode(&message, &value);
  if (value.camera_device_id != 0 || !nonnegative(value.focal_length) ||
      !nonnegative(value.sensor_size_h) || !nonnegative(value.sensor_size_v))
    return {};

  auto out = beginData(message, value.time_boot_ms);
  out << ",\"vendor\":";
  boundedText(out, value.vendor_name);
  out << ",\"model\":";
  boundedText(out, value.model_name);
  const uint32_t firmware = value.firmware_version;
  out << ",\"firmware_version_raw\":" << firmware << ",\"firmware_version\":";
  if (firmware) {
    out << "{\"major\":" << (firmware & 255) << ",\"minor\":" << ((firmware >> 8) & 255)
        << ",\"patch\":" << ((firmware >> 16) & 255) << ",\"development\":" << (firmware >> 24) << '}';
  } else out << "null";
  out << ",\"capability_flags_raw\":" << value.flags << ",\"focal_length_mm\":";
  optionalFloat(out, value.focal_length);
  out << ",\"sensor_width_mm\":";
  optionalFloat(out, value.sensor_size_h);
  out << ",\"sensor_height_mm\":";
  optionalFloat(out, value.sensor_size_v);
  out << ",\"resolution_width_px\":";
  optionalInteger(out, value.resolution_h, value.resolution_h != 0);
  out << ",\"resolution_height_px\":";
  optionalInteger(out, value.resolution_v, value.resolution_v != 0);
  out << ",\"lens_id\":";
  optionalInteger(out, value.lens_id, value.lens_id != 0);
  out << ",\"gimbal_device_id\":";
  optionalInteger(out, value.gimbal_device_id, value.gimbal_device_id != 0);
  out << ",\"definition_version\":";
  optionalInteger(out, value.cam_definition_version, value.cam_definition_version != 0);
  out << ",\"definition_uri_present\":" << std::boolalpha << (value.cam_definition_uri[0] != 0) << '}';
  return out.str();
}

std::string settings(const mavlink_message_t& message)
{
  mavlink_camera_settings_t value{};
  mavlink_msg_camera_settings_decode(&message, &value);
  if (value.camera_device_id != 0 || !percentage(value.zoomLevel) || !percentage(value.focusLevel))
    return {};

  constexpr std::array<const char*, 3> modes{"IMAGE", "VIDEO", "IMAGE_SURVEY"};
  auto out = beginData(message, value.time_boot_ms);
  out << ",\"mode_raw\":" << unsigned(value.mode_id) << ",\"mode\":\"" << label(value.mode_id, modes) << "\",\"zoom_pct\":";
  optionalFloat(out, value.zoomLevel, message.len > MAVLINK_MSG_ID_CAMERA_SETTINGS_MIN_LEN);
  out << ",\"focus_pct\":";
  optionalFloat(out, value.focusLevel, message.len > MAVLINK_MSG_ID_CAMERA_SETTINGS_MIN_LEN + sizeof(float));
  out << '}';
  return out.str();
}

std::string captureStatus(const mavlink_message_t& message)
{
  mavlink_camera_capture_status_t value{};
  mavlink_msg_camera_capture_status_decode(&message, &value);
  if (value.camera_device_id != 0 || !nonnegative(value.image_interval) || !nonnegative(value.available_capacity))
    return {};

  constexpr std::array<const char*, 4> image{"IDLE", "CAPTURING", "INTERVAL_IDLE", "INTERVAL_CAPTURING"};
  constexpr std::array<const char*, 2> video{"IDLE", "RECORDING"};
  auto out = beginData(message, value.time_boot_ms);
  out << ",\"image_status_raw\":" << unsigned(value.image_status) << ",\"image_status\":\""
      << label(value.image_status, image) << "\",\"video_status_raw\":" << unsigned(value.video_status)
      << ",\"video_status\":\"" << label(value.video_status, video) << "\",\"image_interval_s\":";
  optionalFloat(out, value.image_interval);
  out << ",\"recording_time_ms\":";
  optionalInteger(out, value.recording_time_ms, value.recording_time_ms != 0);
  out << ",\"available_capacity_mib\":";
  optionalFloat(out, value.available_capacity);
  out << ",\"image_count_raw\":";
  const bool count_reported = message.len > MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN;
  if (count_reported) out << value.image_count;
  else out << "null";
  out << ",\"image_count\":";
  optionalInteger(out, static_cast<unsigned>(value.image_count), count_reported && value.image_count >= 0);
  out << '}';
  return out.str();
}

std::string storageData(const mavlink_message_t& message, const mavlink_storage_information_t& value)
{
  const bool ready = value.status == STORAGE_STATUS_READY;
  if (!value.storage_id || value.storage_id > value.storage_count ||
      !nonnegative(value.read_speed) || !nonnegative(value.write_speed) ||
      (ready && (!nonnegative(value.total_capacity) || !nonnegative(value.used_capacity) ||
                 !nonnegative(value.available_capacity))))
    return {};

  constexpr std::array<const char*, 4> statuses{"EMPTY", "UNFORMATTED", "READY", "NOT_SUPPORTED"};
  constexpr std::array<const char*, 8> types{"UNKNOWN", "USB_STICK", "SD", "MICROSD", "CF", "CFE", "XQD", "HD"};
  auto out = beginData(message, value.time_boot_ms);
  out << ",\"storage_id\":" << unsigned(value.storage_id) << ",\"storage_count\":" << unsigned(value.storage_count)
      << ",\"status_raw\":" << unsigned(value.status) << ",\"status\":\"" << label(value.status, statuses)
      << "\",\"type_raw\":";
  optionalInteger(out, value.type, message.len > MAVLINK_MSG_ID_STORAGE_INFORMATION_MIN_LEN);
  out << ",\"type\":\"" << (value.type == STORAGE_TYPE_OTHER ? "OTHER" : label(value.type, types))
      << "\",\"storage_usage_raw\":";
  optionalInteger(out, value.storage_usage, message.len == MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN);
  out << ",\"name\":";
  boundedText(out, value.name);
  out << ",\"total_capacity_mib\":";
  optionalFloat(out, value.total_capacity, ready);
  out << ",\"used_capacity_mib\":";
  optionalFloat(out, value.used_capacity, ready);
  out << ",\"available_capacity_mib\":";
  optionalFloat(out, value.available_capacity, ready);
  out << ",\"read_speed_mib_s\":";
  optionalFloat(out, value.read_speed);
  out << ",\"write_speed_mib_s\":";
  optionalFloat(out, value.write_speed);
  out << '}';
  return out.str();
}

struct Family {
  uint32_t id;
  unsigned length;
  const char* variable;
  std::string (*decode)(const mavlink_message_t&);
};
constexpr std::array<Family, 3> families{{
  {MAVLINK_MSG_ID_CAMERA_INFORMATION, MAVLINK_MSG_ID_CAMERA_INFORMATION_LEN, "MAVMOOS_CAMERA_INFORMATION", information},
  {MAVLINK_MSG_ID_CAMERA_SETTINGS, MAVLINK_MSG_ID_CAMERA_SETTINGS_LEN, "MAVMOOS_CAMERA_SETTINGS", settings},
  {MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_LEN, "MAVMOOS_CAMERA_CAPTURE_STATUS", captureStatus},
}};

std::size_t familyIndex(uint32_t id)
{
  for (std::size_t index = 0; index < families.size(); ++index)
    if (families[index].id == id) return index;
  return families.size();
}

constexpr char capture_request[] = "MAVMOOS_CAMERA_REQUEST";
constexpr char capture_result[] = "MAVMOOS_CAMERA_RESULT";
constexpr double request_lifetime = 2;
constexpr double capture_lifetime = 6;
constexpr double camera_ready_age = 2;
constexpr uint32_t last_capture_sequence = (1u << 24) - 1;
constexpr std::size_t request_history_limit = 256;

struct CaptureMail {
  std::string id;
  bool cancel;
};

bool validId(std::string_view value)
{
  if (value.empty() || value.size() > 48) return false;
  for (const char ch : value) {
    if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
        (ch >= '0' && ch <= '9') || ch == '_' || ch == '-' || ch == '.') continue;
    return false;
  }
  return true;
}

std::optional<CaptureMail> parseCaptureMail(const Mail& mail)
{
  if (mail.numeric || mail.text.size() > 120) return {};
  std::string_view input(mail.text), id, action, schema;
  for (unsigned field = 0; field < 3; ++field) {
    const auto comma = input.find(',');
    if ((field < 2) != (comma != std::string_view::npos)) return {};
    const auto pair = input.substr(0, comma);
    const auto equals = pair.find('=');
    if (equals == std::string_view::npos || equals + 1 == pair.size()) return {};
    const auto key = pair.substr(0, equals);
    const auto value = pair.substr(equals + 1);
    if (key == "id" && id.empty()) id = value;
    else if (key == "schema" && schema.empty()) schema = value;
    else if (key == "action" && action.empty()) action = value;
    else return {};
    if (field < 2) input.remove_prefix(comma + 1);
  }
  if (schema != "1" || !validId(id) || (action != "capture" && action != "cancel")) return {};
  return CaptureMail{std::string(id), action == "cancel"};
}

class CameraStatus final : public PassiveObserver, public PayloadListener {
public:
  explicit CameraStatus(Services& host)
    : PassiveObserver(host, true, 3), m_payload(dynamic_cast<PayloadService*>(&host)), m_boundary(host.now())
  { setObservationComponent(0); }
  std::string name() const override { return "camera_status"; }

  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.profile = "mavlink_camera_status_observations";
    result.capabilities = {{"camera.information", "", "", "observation"}, {"camera.settings", "", "", "observation"},
                           {"camera.capture_status", "", "", "observation"}, {"camera.storage", "", "", "observation"}};
    result.exclusive_resources = {"observation.camera_status"};
    result.streamed_parameter_bench_compatible = !m_capture_enabled;
    if (m_capture_enabled) {
      result.capabilities.push_back({"camera.capture.single", capture_request, "action=capture"});
      result.request_owners = {capture_request};
    }
    return result;
  }

  void configure(const std::string& key, const std::string& value) override
  {
    if (key != "camera_component") return;
    unsigned component = 0;
    const auto* end = value.data() + value.size();
    const auto parsed = std::from_chars(value.data(), end, component);
    if (parsed.ec != std::errc{} || parsed.ptr != end || component < 2 || component > 255 ||
        (observationComponent() && observationComponent() != component))
      throw std::invalid_argument("camera_component must name one fixed component in [2,255]");
    setObservationComponent(component);
  }

  ModuleConfigResult configureModule(const std::vector<ModuleSetting>& settings) override
  {
    if (!observationComponent())
      return {ModuleConfigStatus::Invalid, "camera_component", "required for camera_status"};
    std::optional<bool> capture;
    std::vector<ModuleSetting> observer_settings;
    for (const auto& setting : settings) {
      if (setting.key != "capture_enabled") {
        observer_settings.push_back(setting);
        continue;
      }
      if (capture || (setting.value != "true" && setting.value != "false"))
        return {ModuleConfigStatus::Invalid, setting.key, "expected a single true or false"};
      capture = setting.value == "true";
    }
    if (capture.value_or(false) && !m_payload)
      return {ModuleConfigStatus::Invalid, "capture_enabled", "host payload service required"};
    const auto result = PassiveObserver::configureModule(observer_settings);
    if (result.status == ModuleConfigStatus::Accepted) m_capture_enabled = capture.value_or(false);
    return result;
  }

  std::vector<std::string> subscriptions() const override
  { return m_capture_enabled ? std::vector<std::string>{capture_request} : std::vector<std::string>{}; }

  void onMail(const Mail& mail) override
  {
    if (!m_capture_enabled || mail.key != capture_request) return;
    const auto request = parseCaptureMail(mail);
    if (!request) { report("invalid", "REJECTED", "invalid_request"); return; }

    checkCaptureBoundary();
    const double now = services().now();
    const double age = mail.received_at - mail.published_at;
    if (!std::isfinite(now) || now < 0 || !std::isfinite(age) || age < 0 || age > request_lifetime ||
        mail.published_at <= m_boundary || mail.received_at > now || now - mail.received_at > request_lifetime) {
      report(request->id, "REJECTED", "stale_future_or_pre_session_request");
      return;
    }
    if (request->cancel) {
      if (m_operation && m_operation->id == request->id) finish("CANCELLED", "operator_cancel");
      else report(request->id, "REJECTED", "no_matching_capture");
      return;
    }
    if (m_seen.count(request->id) || m_seen.size() >= request_history_limit) {
      report(request->id, "REJECTED", "duplicate_or_history_full");
      return;
    }
    m_seen.insert(request->id);
    if (m_operation) { report(request->id, "REJECTED", "camera_capture_busy"); return; }
    if (!m_payload->payloadReady(PayloadAction::CameraCaptureOne)) {
      report(request->id, "REJECTED", "owned_disarmed_payload_bench_required");
      return;
    }
    if (!cameraReady(now)) { report(request->id, "REJECTED", "fresh_idle_camera_with_new_count_required"); return; }
    if (m_sequence >= last_capture_sequence) { report(request->id, "REJECTED", "capture_sequence_exhausted"); return; }

    const double expires = mail.published_at + capture_lifetime;
    const double handoff_by = mail.published_at + request_lifetime;
    if (now >= handoff_by) { report(request->id, "EXPIRED", "original_send_deadline"); return; }
    m_operation = CaptureOperation{request->id, ++m_sequence, m_camera_status.image_count,
                                  m_camera_status.time_boot_ms, handoff_by, expires, {}, {}, {}};
    PayloadRequest payload;
    payload.id = request->id;
    payload.operation_id = "camera/" + request->id;
    payload.action = PayloadAction::CameraCaptureOne;
    payload.expires = handoff_by;
    payload.operation_expires = expires;
    payload.capture_sequence = m_sequence;
    if (!m_payload->payload(payload)) {
      finish("REJECTED", "host_admission_rejected");
      return;
    }
    report(request->id, "QUEUED", "awaiting_local_handoff", &*m_operation);
  }

  void onPayloadSent(const PayloadRequest& request, double at, bool accepted) override
  {
    if (!m_operation || m_operation->sent_at || request.action != PayloadAction::CameraCaptureOne ||
        request.id != m_operation->id || request.operation_id != "camera/" + m_operation->id ||
        request.capture_sequence != m_operation->sequence) return;
    if (!accepted) { finish("REJECTED", "local_send_not_accepted"); return; }

    m_used_count = m_operation->expected_index;
    m_operation->sent_at = at;
    if (!std::isfinite(at) || at > services().now() || at >= m_operation->send_by) {
      finish("UNCERTAIN", "accepted_without_usable_send_receipt");
      return;
    }
    report(m_operation->id, "SENT", "local_send_accepted", &*m_operation);
  }

private:
  struct Sample {
    Stamp stamp;
    std::string data;
  };
  struct CaptureOperation {
    std::string id;
    uint32_t sequence;
    int32_t expected_index;
    uint32_t baseline_boot;
    double send_by, expires;
    std::optional<double> sent_at;
    std::optional<uint8_t> ack_result;
    std::optional<int32_t> observed_index;
  };

  bool observes(uint32_t id) const override
  {
    return id == MAVLINK_MSG_ID_STORAGE_INFORMATION || familyIndex(id) != families.size() ||
      (m_capture_enabled && (id == MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED || id == MAVLINK_MSG_ID_COMMAND_ACK));
  }

  void observe(const mavlink_message_t& message, const MessageTime& time) override
  {
    checkCaptureBoundary();
    if (message.msgid == MAVLINK_MSG_ID_STORAGE_INFORMATION) {
      observeStorage(message, time);
      return;
    }
    if (message.msgid == MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED) {
      observeImage(message, time);
      return;
    }
    if (message.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
      observeAck(message, time);
      return;
    }
    const auto index = familyIndex(message.msgid);
    if (index == families.size()) return;
    const auto& family = families[index];
    auto data = wireShape(message, 0, family.length) ? family.decode(message) : std::string{};
    if (data.empty()) { rejectWire(); return; }
    auto& sample = m_samples[index];
    if (!update(sample.stamp, time)) return;
    sample.data = std::move(data);
    if (!m_capture_enabled) return;
    if (message.msgid == MAVLINK_MSG_ID_CAMERA_INFORMATION) {
      mavlink_camera_information_t information{};
      mavlink_msg_camera_information_decode(&message, &information);
      m_image_capable = (information.flags & CAMERA_CAP_FLAGS_CAPTURE_IMAGE) != 0;
    } else if (message.msgid == MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS) {
      mavlink_camera_capture_status_t status{};
      mavlink_msg_camera_capture_status_decode(&message, &status);
      if (message.len <= MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN) status.image_count = -1;
      if (m_capture_status_at && status.time_boot_ms <= m_camera_status.time_boot_ms) {
        if (m_operation && status.time_boot_ms < m_camera_status.time_boot_ms) {
          if (m_operation->sent_at) finish("UNCERTAIN", "camera_boot_clock_regressed_after_handoff");
          else finish("CANCELLED", "camera_boot_clock_regressed_before_handoff");
        }
        return;
      }
      m_capture_status_at = time.effective_at;
      m_camera_status = status;
      if (m_operation && m_operation->sent_at &&
          (status.image_count < m_operation->expected_index ||
           status.image_count > m_operation->expected_index + 1))
        finish("UNCERTAIN", "camera_counter_changed_after_handoff");
    }
    checkQueuedReadiness();
  }

  bool cameraReady(double now) const
  {
    const auto fresh = [this, now](double at) {
      return at > 0 && now >= at && now - at <= std::min(sampleTimeout(), camera_ready_age);
    };
    return m_image_capable && fresh(m_samples[familyIndex(MAVLINK_MSG_ID_CAMERA_INFORMATION)].stamp.time.effective_at) &&
      fresh(m_capture_status_at) &&
      m_camera_status.image_status == 0 && m_camera_status.video_status == 0 &&
      m_camera_status.image_count > m_used_count &&
      m_camera_status.image_count < std::numeric_limits<int32_t>::max();
  }

  void checkQueuedReadiness()
  {
    if (!m_operation || m_operation->sent_at) return;
    if (!cameraReady(services().now()) || m_camera_status.image_count != m_operation->expected_index)
      finish("CANCELLED", "camera_state_changed_before_handoff");
  }

  bool replyEligible(const MessageTime& time)
  {
    return m_operation && m_operation->sent_at && time.read_at > *m_operation->sent_at &&
      services().now() - time.effective_at <= camera_ready_age;
  }

  void observeAck(const mavlink_message_t& message, const MessageTime& time)
  {
    if (!replyEligible(time) || !wireShape(message, MAVLINK_MSG_ID_COMMAND_ACK_MIN_LEN, MAVLINK_MSG_ID_COMMAND_ACK_LEN)) return;
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&message, &ack);
    if (ack.command != MAV_CMD_IMAGE_START_CAPTURE || ack.target_system != 245 || ack.target_component != 191 ||
        ack.result == MAV_RESULT_IN_PROGRESS || m_operation->ack_result) return;
    m_operation->ack_result = ack.result;
    // Capture sequence is absent from the ACK. The bound image index is the effect evidence.
    report(m_operation->id, "ACK_OBSERVED", "uncorrelated_camera_ack", &*m_operation);
  }

  void observeImage(const mavlink_message_t& message, const MessageTime& time)
  {
    if (!replyEligible(time) || !wireShape(message, 0, MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN)) return;
    mavlink_camera_image_captured_t image{};
    mavlink_msg_camera_image_captured_decode(&message, &image);
    if (image.camera_id != 0 || image.capture_result != MAV_BOOL_TRUE ||
        image.image_index != m_operation->expected_index || image.time_boot_ms <= m_operation->baseline_boot) return;
    m_operation->observed_index = image.image_index;
    finish("CONFIRMED", "new_camera_image_observed");
  }

  void checkCaptureBoundary()
  {
    if (!m_operation) return;
    const auto now = services().now();
    if (m_operation->sent_at && now >= m_operation->expires)
      finish("UNCERTAIN", "capture_evidence_deadline");
    else if (!m_operation->sent_at && now >= m_operation->send_by)
      finish("EXPIRED", "capture_deadline_before_handoff");
    else if (!m_payload->payloadReady(PayloadAction::CameraCaptureOne)) {
      if (m_operation->sent_at) finish("UNCERTAIN", "payload_bench_lost_after_handoff");
      else finish("CANCELLED", "payload_bench_lost_before_handoff");
    }
  }

  void report(const std::string& id, const char* status, const char* reason,
              const CaptureOperation* operation = nullptr)
  {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << "{\"schema\":1,\"id\":\"" << id << "\",\"status\":\"" << status
        << "\",\"reason\":\"" << reason << "\",\"system_id\":1,\"component_id\":" << observationComponent()
        << ",\"capture_sequence\":" << (operation ? std::to_string(operation->sequence) : "null")
        << ",\"expected_image_index\":" << (operation ? std::to_string(operation->expected_index) : "null")
        << ",\"observed_image_index\":" << (operation && operation->observed_index ?
                                               std::to_string(*operation->observed_index) : "null")
        << ",\"ack_result\":" << (operation && operation->ack_result ? std::to_string(*operation->ack_result) : "null")
        << ",\"may_have_captured\":" << std::boolalpha << (operation && operation->sent_at.has_value()) << '}';
    services().publish(capture_result, out.str());
  }

  void finish(const char* status, const char* reason)
  {
    auto operation = std::move(*m_operation);
    m_operation.reset();
    const auto host_operation = "camera/" + operation.id;
    m_payload->cancelPayload(host_operation);
    m_payload->finishPayload(host_operation);
    report(operation.id, status, reason, &operation);
  }

  void observeStorage(const mavlink_message_t& message, const MessageTime& time)
  {
    if (!wireShape(message, 0, MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN)) { rejectWire();return; }
    mavlink_storage_information_t value{};
    mavlink_msg_storage_information_decode(&message, &value);
    auto data = storageData(message, value);
    if (data.empty()) { rejectWire();return; }
    const unsigned id = value.storage_id;
    auto found = m_storage.find(id);
    if (found == m_storage.end()) {
      if (m_storage.size() >= storage_capacity) { ++m_capacity_rejected;return; }
      found = m_storage.emplace(id, Sample{}).first;
    }
    auto& sample = found->second;
    if (!update(sample.stamp, time)) return;
    sample.data = std::move(data);
    if (!m_latest_storage.id ||
        (time.effective_at >= m_latest_storage.time.effective_at && time.read_at >= m_latest_storage.time.read_at))
      m_latest_storage = sample.stamp;
  }

  void clearSamples() override
  {
    if (m_operation) {
      if (m_operation->sent_at) finish("UNCERTAIN", "camera_session_lost_after_handoff");
      else finish("CANCELLED", "camera_session_lost_before_handoff");
    }
    const double now = services().now();
    if (std::isfinite(now) && (!std::isfinite(m_boundary) || now > m_boundary)) m_boundary = now;
    m_image_capable = false;
    m_capture_status_at = 0;
    m_camera_status = {};
    m_camera_status.image_count = -1;
    m_samples = {};
    m_storage.clear();
    m_latest_storage = {};
  }

  std::string record(const Sample& sample) const
  {
    auto result = metadata(sample.stamp);
    result.pop_back();
    return result + ",\"data\":" + (sample.data.empty() ? "null" : sample.data) + '}';
  }

  void publishSamples() override
  {
    checkCaptureBoundary();
    checkQueuedReadiness();
    for (std::size_t index = 0; index < families.size(); ++index)
      services().publish(families[index].variable, record(m_samples[index]));
    auto result = metadata(m_latest_storage);
    result.pop_back();
    result += ",\"atomic\":false,\"cache_capacity\":" + std::to_string(storage_capacity) +
              ",\"capacity_rejected\":" + std::to_string(m_capacity_rejected) + ",\"data\":";
    if (m_storage.empty()) result += "null";
    else {
      result += '[';
      bool first = true;
      for (const auto& [id, sample] : m_storage) {
        if (!first) result += ',';
        first = false;
        result += record(sample);
      }
      result += ']';
    }
    services().publish("MAVMOOS_CAMERA_STORAGE", result + '}');
  }

  PayloadService* m_payload;
  double m_boundary;
  bool m_capture_enabled = false, m_image_capable = false;
  double m_capture_status_at = 0;
  uint32_t m_sequence = 0;
  int32_t m_used_count = -1;
  mavlink_camera_capture_status_t m_camera_status{};
  std::set<std::string> m_seen;
  std::optional<CaptureOperation> m_operation;
  std::array<Sample, families.size()> m_samples{};
  std::map<unsigned, Sample> m_storage;
  Stamp m_latest_storage;
  uint64_t m_capacity_rejected = 0;
};
} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi, mavmoos::runtime::Services* host)
{
  return abi == mavmoos::runtime::plugin_abi && host ? new mavmoos::runtime::CameraStatus(*host) : nullptr;
}

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin)
{
  delete plugin;
}
