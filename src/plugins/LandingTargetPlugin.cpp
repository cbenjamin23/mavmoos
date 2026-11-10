/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: LandingTargetPlugin.cpp                         */
/*    DATE: November 5, 2026                                */
/************************************************************/
#include "PassiveObserver.h"
#include <cstddef>
#include <stdexcept>

namespace mavmoos::runtime {
namespace {

std::optional<std::string> landingTarget(const mavlink_message_t& message)
{
  mavlink_landing_target_t value{};
  mavlink_msg_landing_target_decode(&message, &value);
  const bool position = value.position_valid == 1;
  const bool distance_known = std::isfinite(value.distance) && value.distance >= 0;
  if (value.position_valid > 1 ||
      (!position && (!std::isfinite(value.angle_x) || !std::isfinite(value.angle_y))) ||
      (position && (!distance_known || !std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z))))
    return std::nullopt;

  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::boolalpha << std::setprecision(17)
      << "{\"message_id\":" << message.msgid
      << ",\"wire_version\":" << (message.magic == MAVLINK_STX ? 2 : 1)
      << ",\"payload_length\":" << unsigned(message.len)
      << ",\"time_usec_raw\":\"" << value.time_usec << '\"'
      << ",\"target_num\":" << unsigned(value.target_num)
      << ",\"frame\":" << unsigned(value.frame)
      << ",\"distance_m\":";
  if (distance_known) out << value.distance; else out << "null";
  out << ",\"position_valid\":" << position
      << ",\"target_type_raw\":";
  if (message.len > offsetof(mavlink_landing_target_t, type)) out << unsigned(value.type);
  else out << "null";
  out << ",\"angle_x_rad\":";
  if (std::isfinite(value.angle_x)) out << value.angle_x; else out << "null";
  out << ",\"angle_y_rad\":";
  if (std::isfinite(value.angle_y)) out << value.angle_y; else out << "null";
  out << ",\"position_m\":";
  if (position) out << '[' << value.x << ',' << value.y << ',' << value.z << ']';
  else out << "null";
  out << '}';
  return out.str();
}

class LandingTarget final : public PassiveObserver {
public:
  explicit LandingTarget(Services& services)
    : PassiveObserver(services, false, .5, MAV_TYPE_QUADROTOR)
  {
    setObservationComponent(0);
  }

  std::string name() const override { return "landing_target"; }

  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.profile = "arducopter_landing_target_observation";
    result.capabilities = {{"observations.landing_target", "", "", "observation"}};
    result.exclusive_resources = {"observation.landing_target"};
    result.streamed_parameter_bench_compatible = true;
    return result;
  }

  void configure(const std::string& key, const std::string& value) override
  {
    if (key != "landing_target_component") return;
    unsigned component = 0;
    const auto* end = value.data() + value.size();
    const auto parsed = std::from_chars(value.data(), end, component);
    if (parsed.ec != std::errc{} || parsed.ptr != end || component < 1 || component > 255)
      throw std::invalid_argument("landing_target_component must be an integer in [1,255]");
    setObservationComponent(component);
  }

  ModuleConfigResult configureModule(const std::vector<ModuleSetting>& settings) override
  {
    if (!observationComponent())
      return {ModuleConfigStatus::Invalid, "landing_target_component", "required for landing_target"};
    return PassiveObserver::configureModule(settings);
  }

private:
  bool observes(uint32_t id) const override { return id == MAVLINK_MSG_ID_LANDING_TARGET; }

  void observe(const mavlink_message_t& message, const MessageTime& time) override
  {
    if (!wireShape(message, MAVLINK_MSG_ID_LANDING_TARGET_MIN_LEN, MAVLINK_MSG_ID_LANDING_TARGET_LEN)) {
      rejectWire();
      return;
    }
    const auto data = landingTarget(message);
    if (!data) {
      rejectWire();
      return;
    }
    if (update(m_stamp, time)) m_data = *data;
  }

  void clearSamples() override
  {
    m_stamp = {};
    m_data = "null";
  }

  void publishSamples() override
  {
    auto record = metadata(m_stamp);
    record.pop_back();
    record += ",\"data\":" + m_data + '}';
    services().publish("MAVMOOS_LANDING_TARGET", record);
  }

  Stamp m_stamp;
  std::string m_data = "null";
};

} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi, mavmoos::runtime::Services* services)
{
  return abi == mavmoos::runtime::plugin_abi && services ? new mavmoos::runtime::LandingTarget(*services) : nullptr;
}

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin)
{
  delete plugin;
}
