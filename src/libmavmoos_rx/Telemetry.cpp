/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: Telemetry.cpp                                   */
/*    DATE: October 29, 2026                                */
/************************************************************/
#include "Telemetry.h"
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>

namespace mavmoos {
namespace {
std::ostringstream stream()
{
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::fixed << std::setprecision(7);
  return out;
}
bool coordinates(int32_t latitude, int32_t longitude)
{
  return latitude >= -900000000 && latitude <= 900000000 &&
         longitude >= -1800000000 && longitude <= 1800000000;
}
const char* landedName(uint8_t value)
{
  switch(value) {
  case MAV_LANDED_STATE_UNDEFINED: return "undefined";
  case MAV_LANDED_STATE_ON_GROUND: return "on_ground";
  case MAV_LANDED_STATE_IN_AIR: return "in_air";
  case MAV_LANDED_STATE_TAKEOFF: return "takeoff";
  case MAV_LANDED_STATE_LANDING: return "landing";
  default: return "unknown";
  }
}

enum class Route { Primary, Camera, Gimbal, RemoteId, ComponentMetadata };

Route observationRoute(const RxConfig& config,const mavlink_message_t& message,bool camera_responses,bool gimbal_responses)
{
  switch(message.msgid) {
  case MAVLINK_MSG_ID_COMMAND_ACK: {
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&message,&ack);
    if(camera_responses && message.compid==config.camera_component &&
       ack.command==MAV_CMD_IMAGE_START_CAPTURE) return Route::Camera;
    if(gimbal_responses && message.compid==config.gimbal_component &&
       (ack.command==MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE ||
        ack.command==MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW)) return Route::Gimbal;
    return Route::Primary;
  }
  case MAVLINK_MSG_ID_CAMERA_INFORMATION:
  case MAVLINK_MSG_ID_CAMERA_SETTINGS:
  case MAVLINK_MSG_ID_STORAGE_INFORMATION:
  case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS:
    return config.camera_component ? Route::Camera : Route::Primary;
  case MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED:
    return camera_responses ? Route::Camera : Route::Primary;
  case MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION:
  case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:
    return config.gimbal_component ? Route::Gimbal : Route::Primary;
  case MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS:
    return gimbal_responses ? Route::Gimbal : Route::Primary;
  case MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID:
  case MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION:
  case MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS:
    return config.remote_id_component ? Route::RemoteId : Route::Primary;
  case MAVLINK_MSG_ID_COMPONENT_METADATA:
    return config.component_metadata_component ? Route::ComponentMetadata : Route::Primary;
  default:
    return Route::Primary;
  }
}
}

Telemetry::Telemetry(const RxConfig& config, std::string session)
  : m_config(config), m_session(std::move(session)) {}

std::vector<Family> Telemetry::families()
{
  return {Family::Heartbeat, Family::Position, Family::Landed, Family::Home, Family::Estimator};
}

const char* Telemetry::variable(Family family)
{
  switch(family) {
  case Family::Heartbeat: return "MAVMOOS_HEARTBEAT";
  case Family::Position: return "MAVMOOS_GLOBAL_POSITION";
  case Family::Landed: return "MAVMOOS_LANDED_STATE";
  case Family::Home: return "MAVMOOS_HOME_POSITION";
  case Family::Estimator: return "MAVMOOS_ESTIMATOR";
  default: return "MAVMOOS_UNKNOWN";
  }
}

const Observation& Telemetry::observation(Family family) const
{
  return m_reports.at(static_cast<std::size_t>(family));
}

std::string Telemetry::identityState() const
{
  return !m_identity_seen ? "UNSEEN" : (m_identity_matches ? "EXPECTED" : "MISMATCH");
}

bool Telemetry::heartbeatFresh(double now) const
{
  const auto& report = observation(Family::Heartbeat);
  return m_identity_matches && report.present && report.encoding_valid &&
         now >= report.received_at && now - report.received_at <= m_config.heartbeat_timeout;
}

bool Telemetry::duplicate(std::deque<Seen>& cache, const mavlink_message_t& message, double received_at)
{
  // Bounded exact-frame suppression, not authenticated replay detection or source ordering.
  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> bytes{};
  const auto size = mavlink_msg_to_send_buffer(bytes.data(), &message);
  std::string key(reinterpret_cast<const char*>(bytes.data()), size);
  while(!cache.empty() && received_at - cache.front().received_at > 2.0)
    cache.pop_front();
  for(const auto& seen : cache) {
    if(seen.bytes == key)
      return true;
  }
  if(cache.size() >= 128)
    cache.pop_front();
  cache.push_back({std::move(key), received_at});
  return false;
}

void Telemetry::ingest(const uint8_t* bytes, std::size_t size, double received_at,
                       uint64_t datagram_id)
{
  ingest(bytes,size,received_at,datagram_id,received_at,"READ_ACQUISITION");
}

void Telemetry::ingest(const uint8_t* bytes, std::size_t size, double received_at,
                       uint64_t datagram_id, double read_at, const std::string& age_basis)
{
  if(!std::isfinite(received_at) || !std::isfinite(read_at) || received_at<0 || read_at<received_at ||
     (age_basis!="READ_ACQUISITION" && age_basis!="QUEUE_EMPTY_FLOOR") ||
     (age_basis=="READ_ACQUISITION" && received_at!=read_at)) {
    ++m_stats.invalid_timestamps; return;
  }
  // Deliberately require complete frames within a UDP datagram. Never stitch across reads.
  mavlink_message_t buffer{}, message{};
  mavlink_status_t state{}, result{};
  bool any_frame = false;
  for(std::size_t i = 0; i < size; ++i) {
    const auto framing = mavlink_frame_char_buffer(&buffer, &state, bytes[i], &message, &result);
    if(framing == MAVLINK_FRAMING_INCOMPLETE)
      continue;
    any_frame = true;
    if(message.incompat_flags != 0) {
      ++m_stats.unsupported_flags;
      continue;
    }
    if(framing != MAVLINK_FRAMING_OK) {
      ++m_stats.bad_frames;
      continue;
    }
    const auto* entry = mavlink_get_msg_entry(message.msgid);
    // MAVLink 2 may truncate trailing zero bytes; MAVLink 1 has a fixed base length.
    if(!entry || message.len > entry->max_msg_len ||
       (entry->min_msg_len > 0 && message.len == 0) ||
       (message.magic == MAVLINK_STX_MAVLINK1 && message.len != entry->min_msg_len)) {
      ++m_stats.bad_frames;
      continue;
    }
    ++m_stats.frames;
    const auto route = observationRoute(m_config,message,m_camera_responses,m_gimbal_responses);
    auto expected_component = m_config.target_component;
    auto* duplicate_cache = &m_seen;
    if(route == Route::Camera) {
      expected_component = m_config.camera_component;
      duplicate_cache = &m_camera_seen;
    } else if(route == Route::Gimbal) {
      expected_component = m_config.gimbal_component;
      duplicate_cache = &m_gimbal_seen;
    } else if(route == Route::RemoteId) {
      expected_component = m_config.remote_id_component;
      duplicate_cache = &m_remote_id_seen;
    } else if(route == Route::ComponentMetadata) {
      expected_component = m_config.component_metadata_component;
      duplicate_cache = &m_component_metadata_seen;
    }
    const bool selected_landing_target = m_config.landing_target_component != 0 &&
      message.msgid == MAVLINK_MSG_ID_LANDING_TARGET && message.compid == m_config.landing_target_component;
    if(message.sysid != m_config.target_system ||
       (message.compid != expected_component && !selected_landing_target)) {
      ++m_stats.wrong_target;
      continue;
    }
    // Payload frames use a fresh FC identity, but cannot bind or refresh it.
    if(route != Route::Primary &&
       (!heartbeatFresh(read_at) || received_at < observation(Family::Heartbeat).received_at)) {
      ++m_stats.unbound_reports;
      continue;
    }
    if(duplicate(*duplicate_cache, message, received_at)) {
      ++m_stats.duplicates;
      continue;
    }
    if(route == Route::Primary)
      accept(message, received_at, datagram_id,read_at,age_basis);
    if(m_handler && (m_identity_matches || message.msgid == MAVLINK_MSG_ID_HEARTBEAT))
      m_handler(message, received_at,read_at);
  }
  if(state.parse_state != MAVLINK_PARSE_STATE_IDLE &&
     state.parse_state != MAVLINK_PARSE_STATE_UNINIT) {
    ++m_stats.incomplete_datagrams;
  } else if(!any_frame) {
    ++m_stats.empty_or_noise;
  }
}

void Telemetry::accept(const mavlink_message_t& msg, double received_at, uint64_t datagram_id,
                       double read_at, const std::string& age_basis)
{
  Family family;
  switch(msg.msgid) {
  case MAVLINK_MSG_ID_HEARTBEAT: family = Family::Heartbeat; break;
  case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: family = Family::Position; break;
  case MAVLINK_MSG_ID_EXTENDED_SYS_STATE: family = Family::Landed; break;
  case MAVLINK_MSG_ID_HOME_POSITION: family = Family::Home; break;
  case MAVLINK_MSG_ID_ESTIMATOR_STATUS:
    if(m_config.expected_autopilot!=MAV_AUTOPILOT_PX4) { ++m_stats.unsupported_messages;return; }
    family = Family::Estimator; break;
  case MAVLINK_MSG_ID_EKF_STATUS_REPORT:
    if(m_config.expected_autopilot==MAV_AUTOPILOT_PX4) { ++m_stats.unsupported_messages;return; }
    family = Family::Estimator; break;
  default: ++m_stats.unsupported_messages; return;
  }
  if(family != Family::Heartbeat && !m_identity_matches) {
    ++m_stats.unbound_reports;
    return;
  }
  Observation report;
  report.present = true;
  report.encoding_valid = true;
  report.reason = "ok";
  report.received_at = received_at;
  report.read_at = read_at;
  report.age_basis = age_basis;
  report.datagram_id = datagram_id;
  report.wire_sequence = msg.seq;
  auto fields = stream();
  if(family == Family::Heartbeat) {
    mavlink_heartbeat_t value{};
    mavlink_msg_heartbeat_decode(&msg, &value);
    m_identity_seen = true;
    m_identity_matches = value.type == m_config.expected_type &&
                         value.autopilot == m_config.expected_autopilot;
    report.encoding_valid = m_identity_matches;
    if(!m_identity_matches) {
      report.reason = "identity_mismatch";
      for(auto other : families())
        m_reports[static_cast<std::size_t>(other)] = {};
    }
    fields << ",source_time=not_supplied"
           << ",vehicle_type=" << unsigned(value.type)
           << ",autopilot=" << unsigned(value.autopilot)
           << ",base_mode=" << unsigned(value.base_mode)
           << ",custom_mode=" << value.custom_mode
           << ",custom_mode_enabled=" << ((value.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) ? "true" : "false")
           << ",armed=" << ((value.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? "true" : "false")
           << ",system_status=" << unsigned(value.system_status);
  } else if(family == Family::Position) {
    mavlink_global_position_int_t value{};
    mavlink_msg_global_position_int_decode(&msg, &value);
    report.encoding_valid = coordinates(value.lat, value.lon) &&
                           (value.hdg <= 35999 || value.hdg == UINT16_MAX);
    if(!report.encoding_valid) report.reason = "position_field_out_of_range";
    fields << ",source_time_domain=boot_ms,source_time=" << value.time_boot_ms
           << ",lat_deg=" << value.lat / 1e7 << ",lon_deg=" << value.lon / 1e7
           << ",alt_msl_m=" << value.alt / 1000.0
           << ",alt_relative_home_m=" << value.relative_alt / 1000.0
           << ",vn_mps=" << value.vx / 100.0 << ",ve_mps=" << value.vy / 100.0
           << ",vd_mps=" << value.vz / 100.0 << ",heading_deg=";
    if(value.hdg == UINT16_MAX) fields << "unknown";
    else fields << value.hdg / 100.0;
  } else if(family == Family::Landed) {
    mavlink_extended_sys_state_t value{};
    mavlink_msg_extended_sys_state_decode(&msg, &value);
    report.encoding_valid = value.landed_state >= MAV_LANDED_STATE_ON_GROUND &&
                           value.landed_state <= MAV_LANDED_STATE_LANDING;
    if(!report.encoding_valid)
      report.reason = value.landed_state == MAV_LANDED_STATE_UNDEFINED ?
        "landed_state_undefined" : "unknown_landed_state";
    fields << ",source_time=not_supplied,landed_state=" << unsigned(value.landed_state)
           << ",landed_name=" << landedName(value.landed_state)
           << ",vtol_state=" << unsigned(value.vtol_state);
  } else if(family == Family::Home) {
    mavlink_home_position_t value{};
    mavlink_msg_home_position_decode(&msg, &value);
    report.encoding_valid = coordinates(value.latitude, value.longitude);
    if(!report.encoding_valid) report.reason = "home_coordinate_out_of_range";
    fields << ",projection=global_only,source_time_domain=remote_usec_unmapped,source_time=";
    if(value.time_usec == 0) fields << "unknown";
    else fields << value.time_usec;
    fields << ",lat_deg=" << value.latitude / 1e7
           << ",lon_deg=" << value.longitude / 1e7
           << ",alt_msl_m=" << value.altitude / 1000.0;
  } else if(family == Family::Estimator && msg.msgid==MAVLINK_MSG_ID_ESTIMATOR_STATUS) {
    mavlink_estimator_status_t value{};
    mavlink_msg_estimator_status_decode(&msg,&value);
    report.encoding_valid=value.time_usec>0 && std::isfinite(value.pos_horiz_accuracy) &&
      value.pos_horiz_accuracy>=0 && std::isfinite(value.pos_vert_accuracy) && value.pos_vert_accuracy>=0;
    if(!report.encoding_valid)report.reason="invalid_estimator_accuracy";
    fields << ",protocol=ESTIMATOR_STATUS,source_time_domain=remote_usec_unmapped,source_time="
           << value.time_usec << ",flags=" << value.flags << ",pos_horiz_accuracy_m="
           << value.pos_horiz_accuracy << ",pos_vert_accuracy_m=" << value.pos_vert_accuracy;
  } else if(family == Family::Estimator) {
    mavlink_ekf_status_report_t value{};
    mavlink_msg_ekf_status_report_decode(&msg, &value);
    for(float variance : {value.velocity_variance, value.pos_horiz_variance,
                          value.pos_vert_variance, value.compass_variance, value.terrain_alt_variance}) {
      if(!std::isfinite(variance) || variance < 0)
        report.encoding_valid = false;
    }
    fields << ",source_time=not_supplied,flags=" << value.flags
           << ",velocity_variance=" << value.velocity_variance
           << ",pos_horiz_variance=" << value.pos_horiz_variance
           << ",pos_vert_variance=" << value.pos_vert_variance
           << ",compass_variance=" << value.compass_variance
           << ",terrain_alt_variance=" << value.terrain_alt_variance
           << ",airspeed_variance=";
    // Do not present an unobserved optional extension as measured zero.
    if(msg.len <= 22) {
      fields << "not_observed";
    } else {
      fields << value.airspeed_variance;
      if(!std::isfinite(value.airspeed_variance) || value.airspeed_variance < 0)
        report.encoding_valid = false;
    }
    if(!report.encoding_valid) report.reason = "invalid_estimator_variance";
  }
  report.fields = fields.str();
  report.receipt_id = ++m_stats.observations;
  if(!report.encoding_valid) ++m_stats.invalid_observations;
  m_reports[static_cast<std::size_t>(family)] = std::move(report);
}

std::string Telemetry::record(Family family, double now) const
{
  const auto& report = observation(family);
  auto out = stream();
  out << "schema=2,session=" << m_session << ",present=" << (report.present ? "true" : "false")
      << ",source_quality=unassessed,source_age=unknown,identity=" << identityState();
  if(!report.present) {
    out << ",encoding_valid=unknown,freshness=ABSENT,reason=absent,age_s=unknown";
    return out.str();
  }
  const double age = now - report.received_at;
  const std::array<double, 5> timeouts{m_config.heartbeat_timeout, m_config.position_timeout,
                                    m_config.landed_timeout, 0, m_config.estimator_timeout};
  std::string freshness = "INVALID";
  if(report.encoding_valid) {
    if(age < 0) freshness = "CLOCK_ERROR";
    else if(family == Family::Home) freshness = "REFERENCE_KNOWN";
    else freshness = age <= timeouts.at(static_cast<std::size_t>(family)) ? "FRESH" : "STALE";
  }
  out << ",encoding_valid=" << (report.encoding_valid ? "true" : "false")
      << ",freshness=" << freshness << ",reason=" << report.reason
      << ",age_s=" << age << ",rx_mono_s=" << report.received_at
      << ",age_basis=" << report.age_basis
      << ",age_is_upper_bound=" << (report.age_basis=="QUEUE_EMPTY_FLOOR"?"true":"false")
      << ",read_mono_s=" << report.read_at
      << ",queue_age_upper_s=" << (report.read_at-report.received_at)
      << ",receipt_id=" << report.receipt_id << ",datagram_id=" << report.datagram_id
      << ",sysid=" << unsigned(m_config.target_system)
      << ",compid=" << unsigned(m_config.target_component)
      << ",wire_seq=" << unsigned(report.wire_sequence) << report.fields;
  return out.str();
}
} // namespace mavmoos
