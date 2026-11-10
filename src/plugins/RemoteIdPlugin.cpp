/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: RemoteIdPlugin.cpp                              */
/*    DATE: November 6, 2026                                */
/************************************************************/
#include "PassiveObserver.h"

#include <algorithm>
#include <stdexcept>

namespace mavmoos::runtime {
namespace {

constexpr double remote_id_timeout = 3.5;

template<std::size_t Size>
bool emptyIdentifier(const uint8_t (&bytes)[Size])
{
  return std::all_of(std::begin(bytes), std::end(bytes), [](uint8_t byte) { return byte == 0; });
}

template<typename Value>
void writeKnown(std::ostream& out, Value value, bool known)
{
  if (known) out << +value;
  else out << "null";
}

class RemoteId final : public PassiveObserver {
public:
  explicit RemoteId(Services& services) : PassiveObserver(services, true, remote_id_timeout)
  {
    setObservationComponent(0);
  }

  std::string name() const override { return "remote_id"; }

  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.profile = "mavlink_open_drone_id_observations";
    result.capabilities = {{"remote_id.identity", "", "", "observation"},
                           {"remote_id.location", "", "", "observation"},
                           {"remote_id.health", "", "", "observation"}};
    result.exclusive_resources = {"observation.remote_id"};
    result.streamed_parameter_bench_compatible = true;
    return result;
  }

  void configure(const std::string& key, const std::string& value) override
  {
    if (key != "remote_id_component") return;
    unsigned component = 0;
    const auto parsed = std::from_chars(value.data(), value.data() + value.size(), component);
    if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() ||
        component < 1 || component > 255)
      throw std::invalid_argument("remote_id_component must be an integer in 1..255");
    setObservationComponent(component);
  }

private:
  bool observes(uint32_t id) const override
  {
    return id == MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID || id == MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION ||
      id == MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS;
  }

  void observe(const mavlink_message_t& message, const MessageTime& time) override
  {
    switch (message.msgid) {
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID:
      observeIdentity(message, time);
      break;
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION:
      observeLocation(message, time);
      break;
    default:
      observeHealth(message, time);
    }
  }

  template<std::size_t Size>
  bool ownAircraft(uint8_t target_system, uint8_t target_component, const uint8_t (&relay_id)[Size]) const
  {
    return (target_system == 0 || target_system == 1) &&
      (target_component == 0 || target_component == 1 || target_component == observationComponent()) &&
      emptyIdentifier(relay_id);
  }

  void observeIdentity(const mavlink_message_t& message, const MessageTime& time)
  {
    if (!wireShape(message, MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID_MIN_LEN,
                   MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID_LEN)) {
      rejectWire();
      return;
    }
    mavlink_open_drone_id_basic_id_t value{};
    mavlink_msg_open_drone_id_basic_id_decode(&message, &value);
    const bool present = !emptyIdentifier(value.uas_id);
    if (!ownAircraft(value.target_system, value.target_component, value.id_or_mac) ||
        value.id_type >= MAV_ODID_ID_TYPE_ENUM_END || value.ua_type >= MAV_ODID_UA_TYPE_ENUM_END ||
        present != (value.id_type != MAV_ODID_ID_TYPE_NONE)) {
      rejectWire();
      return;
    }
    if (!update(m_basic_stamp, time)) return;

    // Retain only enough to detect an identity change within one FC session.
    // The identifier bytes are never included in a MOOS publication.
    if (present && (!m_basic_present || m_id_type != value.id_type ||
                    !std::equal(m_identifier.begin(), m_identifier.end(), value.uas_id)))
      ++m_identity_generation;
    std::copy(std::begin(value.uas_id), std::end(value.uas_id), m_identifier.begin());
    m_basic_present = present;
    m_id_type = value.id_type;
    m_ua_type = value.ua_type;
  }

  void observeLocation(const mavlink_message_t& message, const MessageTime& time)
  {
    if (!wireShape(message, MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION_MIN_LEN,
                   MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION_LEN)) {
      rejectWire();
      return;
    }
    mavlink_open_drone_id_location_t value{};
    mavlink_msg_open_drone_id_location_decode(&message, &value);
    if (!ownAircraft(value.target_system, value.target_component, value.id_or_mac) ||
        value.status >= MAV_ODID_STATUS_ENUM_END || value.height_reference >= MAV_ODID_HEIGHT_REF_ENUM_END ||
        value.latitude < -900000000 || value.latitude > 900000000 ||
        value.longitude < -1800000000 || value.longitude > 1800000000 ||
        !std::isfinite(value.altitude_barometric) || !std::isfinite(value.altitude_geodetic) ||
        !std::isfinite(value.height) || !std::isfinite(value.timestamp) ||
        (value.timestamp != 65535 && (value.timestamp < 0 || value.timestamp >= 3600)) ||
        (value.direction != 36100 && value.direction > 35999) ||
        (value.speed_horizontal != 25500 && value.speed_horizontal > 25425) ||
        (value.speed_vertical != 6300 && (value.speed_vertical < -6200 || value.speed_vertical > 6200)) ||
        value.horizontal_accuracy >= MAV_ODID_HOR_ACC_ENUM_END ||
        value.vertical_accuracy >= MAV_ODID_VER_ACC_ENUM_END ||
        value.barometer_accuracy >= MAV_ODID_VER_ACC_ENUM_END ||
        value.speed_accuracy >= MAV_ODID_SPEED_ACC_ENUM_END ||
        value.timestamp_accuracy >= MAV_ODID_TIME_ACC_ENUM_END) {
      rejectWire();
      return;
    }
    if (update(m_location_stamp, time)) m_location = value;
  }

  void observeHealth(const mavlink_message_t& message, const MessageTime& time)
  {
    if (!wireShape(message, MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_MIN_LEN,
                   MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS_LEN)) {
      rejectWire();
      return;
    }
    mavlink_open_drone_id_arm_status_t value{};
    mavlink_msg_open_drone_id_arm_status_decode(&message, &value);
    if (value.status >= MAV_ODID_ARM_STATUS_ENUM_END) {
      rejectWire();
      return;
    }
    if (update(m_health_stamp, time)) {
      m_health = value.status;
      m_error_present = value.error[0] != '\0';
    }
  }

  bool current(const Stamp& stamp)
  {
    return stamp.id && services().now() - stamp.time.effective_at <= sampleTimeout();
  }

  void publishSamples() override
  {
    std::ostringstream identity;
    identity << std::boolalpha << "{\"metadata\":" << metadata(m_basic_stamp) << ",\"data\":";
    if (current(m_basic_stamp)) {
      identity << "{\"id_type\":" << unsigned(m_id_type) << ",\"ua_type\":" << unsigned(m_ua_type)
               << ",\"id_present\":" << m_basic_present << ",\"identity_generation\":";
      writeKnown(identity, m_identity_generation, m_basic_present);
      identity << '}';
    } else {
      identity << "null";
    }
    identity << '}';
    services().publish("MAVMOOS_REMOTE_ID_IDENTITY", identity.str());

    std::ostringstream location;
    location.imbue(std::locale::classic());
    location << std::setprecision(12) << "{\"metadata\":" << metadata(m_location_stamp) << ",\"data\":";
    if (current(m_location_stamp)) writeLocation(location);
    else location << "null";
    location << '}';
    services().publish("MAVMOOS_REMOTE_ID_LOCATION", location.str());

    std::ostringstream health;
    health << std::boolalpha << "{\"metadata\":" << metadata(m_health_stamp) << ",\"data\":";
    if (current(m_health_stamp)) {
      health << "{\"status\":" << unsigned(m_health) << ",\"reported_ready\":"
             << (m_health == MAV_ODID_ARM_STATUS_GOOD_TO_ARM)
             << ",\"error_present\":" << m_error_present << '}';
    } else {
      health << "null";
    }
    health << '}';
    services().publish("MAVMOOS_REMOTE_ID_HEALTH", health.str());
  }

  void writeLocation(std::ostream& out) const
  {
    const auto& value = m_location;
    const bool position = value.latitude != 0 || value.longitude != 0;
    out << "{\"status\":" << unsigned(value.status) << ",\"latitude_deg\":";
    writeKnown(out, value.latitude / 1e7, position);
    out << ",\"longitude_deg\":";
    writeKnown(out, value.longitude / 1e7, position);
    out << ",\"barometric_altitude_m\":";
    writeKnown(out, value.altitude_barometric, value.altitude_barometric != -1000);
    out << ",\"wgs84_altitude_m\":";
    writeKnown(out, value.altitude_geodetic, value.altitude_geodetic != -1000);
    out << ",\"height_m\":";
    writeKnown(out, value.height, value.height != -1000);
    out << ",\"height_reference\":";
    writeKnown(out, value.height_reference, value.height != -1000);
    out << ",\"course_deg\":";
    writeKnown(out, value.direction / 100., value.direction != 36100);
    out << ",\"ground_speed_m_s\":";
    writeKnown(out, value.speed_horizontal / 100., value.speed_horizontal != 25500);
    out << ",\"vertical_speed_up_m_s\":";
    writeKnown(out, value.speed_vertical / 100., value.speed_vertical != 6300);
    out << ",\"utc_second_of_hour\":";
    writeKnown(out, value.timestamp, value.timestamp != 65535);
    out << ",\"horizontal_accuracy\":" << unsigned(value.horizontal_accuracy)
        << ",\"vertical_accuracy\":" << unsigned(value.vertical_accuracy)
        << ",\"barometer_accuracy\":" << unsigned(value.barometer_accuracy)
        << ",\"speed_accuracy\":" << unsigned(value.speed_accuracy)
        << ",\"timestamp_accuracy\":" << unsigned(value.timestamp_accuracy) << '}';
  }

  void clearSamples() override
  {
    m_basic_stamp = m_location_stamp = m_health_stamp = {};
    m_identifier.fill(0);
    m_basic_present = m_error_present = false;
    m_identity_generation = m_id_type = m_ua_type = m_health = 0;
    m_location = {};
  }

  Stamp m_basic_stamp, m_location_stamp, m_health_stamp;
  std::array<uint8_t, MAVLINK_MSG_OPEN_DRONE_ID_BASIC_ID_FIELD_UAS_ID_LEN> m_identifier{};
  bool m_basic_present = false, m_error_present = false;
  uint64_t m_identity_generation = 0;
  uint8_t m_id_type = 0, m_ua_type = 0, m_health = 0;
  mavlink_open_drone_id_location_t m_location{};
};

} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi, mavmoos::runtime::Services* services)
{
  if (abi != mavmoos::runtime::plugin_abi || !services) return nullptr;
  return new mavmoos::runtime::RemoteId(*services);
}

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin)
{
  delete plugin;
}
