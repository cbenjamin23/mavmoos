// Passive FC-reported navigation diagnostics, independent of helm and outbound requests.
#include "PassiveObserver.h"
#include <array>
#include <cmath>
#include <utility>

namespace mavmoos::runtime {
namespace {
using json = std::ostringstream;

json numbers()
{
  json out;
  out.imbue(std::locale::classic());
  out << std::boolalpha << std::setprecision(17);
  return out;
}

struct Target {
  uint32_t boot;
  uint16_t mask;
  uint8_t frame;
  std::array<float, 3> velocity, acceleration;
  float yaw, yaw_rate;
};

bool active(uint16_t mask, unsigned axis) { return (mask & (1u << axis)) == 0; }

void optionalValue(std::ostream& out, bool present, double value)
{
  if (present) out << value;
  else out << "null";
}

bool validTarget(const Target& t)
{
  for (unsigned i = 0; i < 3; ++i) {
    if (active(t.mask, i + 3) && !std::isfinite(t.velocity[i])) return false;
    if (active(t.mask, i + 6) && !std::isfinite(t.acceleration[i])) return false;
  }
  return (!active(t.mask, 10) || std::isfinite(t.yaw)) &&
         (!active(t.mask, 11) || std::isfinite(t.yaw_rate));
}

template <typename Packet>
Target shared(const Packet& p)
{
  return {p.time_boot_ms, p.type_mask, p.coordinate_frame, {p.vx,p.vy,p.vz},
          {p.afx,p.afy,p.afz}, p.yaw, p.yaw_rate};
}

void writeTarget(std::ostream& out, const Target& t, const char* frame)
{
  out << "{\"time_boot_ms_raw\":" << t.boot << ",\"coordinate_frame_raw\":" << unsigned(t.frame)
      << ",\"coordinate_frame\":";
  if (frame) out << '"' << frame << '"';
  else out << "null";
  out << ",\"type_mask_raw\":" << t.mask << ",\"mask_unassigned_bits\":" << (t.mask & 0xF000u)
      << ",\"velocity_xyz_m_s\":[";
  for (unsigned i = 0; i < 3; ++i) {
    if (i) out << ',';
    optionalValue(out, active(t.mask, i + 3), t.velocity[i]);
  }
  out << "],\"acceleration_or_force\":\""
      << ((t.mask & POSITION_TARGET_TYPEMASK_FORCE_SET) ? "force_n" : "acceleration_m_s2")
      << "\",\"acceleration_or_force_xyz\":[";
  for (unsigned i = 0; i < 3; ++i) {
    if (i) out << ',';
    optionalValue(out, active(t.mask, i + 6), t.acceleration[i]);
  }
  out << "],\"yaw_rad\":";
  optionalValue(out, active(t.mask, 10), t.yaw);
  out << ",\"yaw_rate_rad_s\":";
  optionalValue(out, active(t.mask, 11), t.yaw_rate);
}

const char* localFrame(unsigned id)
{
  switch (id) {
  case MAV_FRAME_LOCAL_NED: return "local_ned";
  case MAV_FRAME_LOCAL_OFFSET_NED: return "local_offset_ned";
  case MAV_FRAME_BODY_NED: return "body_ned_wire";
  case MAV_FRAME_BODY_OFFSET_NED: return "body_offset_ned_wire";
  default: return nullptr;
  }
}

const char* globalFrame(unsigned id)
{
  switch (id) {
  case MAV_FRAME_GLOBAL: case MAV_FRAME_GLOBAL_INT: return "global_msl";
  case MAV_FRAME_GLOBAL_RELATIVE_ALT: case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT: return "global_relative_home";
  case MAV_FRAME_GLOBAL_TERRAIN_ALT: case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT: return "global_terrain_relative";
  default: return nullptr;
  }
}

std::string localTarget(const mavlink_message_t& msg)
{
  mavlink_position_target_local_ned_t p{};
  mavlink_msg_position_target_local_ned_decode(&msg, &p);
  const Target target = shared(p);
  const std::array<float,3> position{p.x,p.y,p.z};
  if (!validTarget(target)) return {};
  for (unsigned i = 0; i < 3; ++i)
    if (active(target.mask, i) && !std::isfinite(position[i])) return {};
  auto out = numbers();
  writeTarget(out, target, localFrame(target.frame));
  out << ",\"position_xyz_m\":[";
  for (unsigned i = 0; i < 3; ++i) {
    if (i) out << ',';
    optionalValue(out, active(target.mask, i), position[i]);
  }
  out << "]}";
  return out.str();
}

std::string globalTarget(const mavlink_message_t& msg)
{
  mavlink_position_target_global_int_t p{};
  mavlink_msg_position_target_global_int_decode(&msg, &p);
  const Target target = shared(p);
  const auto* frame = globalFrame(target.frame);
  if (!validTarget(target) || (active(target.mask, 2) && !std::isfinite(p.alt)) ||
      (frame && active(target.mask, 0) && std::abs(double(p.lat_int)) > 900000000) ||
      (frame && active(target.mask, 1) && std::abs(double(p.lon_int)) > 1800000000)) return {};
  auto out = numbers();
  writeTarget(out, target, frame);
  out << ",\"latitude_e7_raw\":";
  optionalValue(out, active(target.mask, 0), p.lat_int);
  out << ",\"longitude_e7_raw\":";
  optionalValue(out, active(target.mask, 1), p.lon_int);
  out << ",\"latitude_deg\":";
  optionalValue(out, frame && active(target.mask, 0), double(p.lat_int) / 1e7);
  out << ",\"longitude_deg\":";
  optionalValue(out, frame && active(target.mask, 1), double(p.lon_int) / 1e7);
  out << ",\"altitude_reported_m\":";
  optionalValue(out, active(target.mask, 2), p.alt);
  out << '}';
  return out.str();
}

std::string controller(const mavlink_message_t& msg)
{
  mavlink_nav_controller_output_t p{};
  mavlink_msg_nav_controller_output_decode(&msg, &p);
  if (!std::isfinite(p.nav_roll) || !std::isfinite(p.nav_pitch) ||
      !std::isfinite(p.alt_error) || !std::isfinite(p.aspd_error) ||
      !std::isfinite(p.xtrack_error)) return {};
  auto out = numbers();
  out << "{\"nav_roll_reported_deg\":" << p.nav_roll
      << ",\"nav_pitch_reported_deg\":" << p.nav_pitch
      << ",\"nav_bearing_reported_deg\":" << p.nav_bearing
      << ",\"target_bearing_reported_deg\":" << p.target_bearing
      << ",\"waypoint_distance_reported_m\":" << p.wp_dist
      << ",\"altitude_error_reported_m\":" << p.alt_error
      << ",\"airspeed_error_reported_raw\":" << p.aspd_error
      << ",\"cross_track_error_reported_m\":" << p.xtrack_error << '}';
  return out.str();
}

struct Family {
  uint32_t id;
  uint8_t min_length, length;
  const char* name;
  const char* variable;
  const char* capability;
  std::string (*decode)(const mavlink_message_t&);
};
constexpr std::array<Family,3> families{{
  {MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_MIN_LEN,
   MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN, "NAV_CONTROLLER_OUTPUT", "MAVMOOS_NAV_CONTROLLER_OUTPUT",
   "navigation_monitor.controller_output", controller},
  {MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED_MIN_LEN,
   MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED_LEN, "POSITION_TARGET_LOCAL_NED", "MAVMOOS_POSITION_TARGET_LOCAL_NED",
   "navigation_monitor.target_local", localTarget},
  {MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT, MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT_MIN_LEN,
   MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT_LEN, "POSITION_TARGET_GLOBAL_INT", "MAVMOOS_POSITION_TARGET_GLOBAL_INT",
   "navigation_monitor.target_global", globalTarget}
}};

std::size_t familyIndex(uint32_t id)
{
  for (std::size_t i = 0; i < families.size(); ++i)
    if (id == families[i].id) return i;
  return families.size();
}

class NavigationMonitor final : public PassiveObserver {
public:
  explicit NavigationMonitor(Services& host) : PassiveObserver(host, true, 2.) {}
  std::string name() const override { return "navigation_monitor"; }

  PluginDescriptor descriptor() const override
  {
    PluginDescriptor d;
    d.profile = "mavlink_navigation_observations";
    for (const auto& f : families) d.capabilities.push_back({f.capability, "", "", "observation"});
    d.exclusive_resources = {"observation.navigation_monitor"};
    d.streamed_parameter_bench_compatible = true;
    return d;
  }

protected:
  bool observes(uint32_t id) const override { return familyIndex(id) < families.size(); }
  void observe(const mavlink_message_t& msg, const MessageTime& time) override
  {
    const auto i = familyIndex(msg.msgid);
    if (i == families.size()) return;
    const auto& family = families[i];
    auto data = wireShape(msg, family.min_length, family.length) ? family.decode(msg) : std::string{};
    if (data.empty()) { rejectWire(); ++m_invalid[i]; return; }
    if (update(m_samples[i].stamp, time)) m_samples[i].data = std::move(data);
  }
  void clearSamples() override { m_samples = {}; }
  void publishSamples() override
  {
    for (std::size_t i = 0; i < families.size(); ++i) {
      const auto& family = families[i];
      const auto& sample = m_samples[i];
      auto out = metadata(sample.stamp);
      out.pop_back();
      out += ",\"message_id\":" + std::to_string(family.id) + ",\"family\":\"" + family.name +
             "\",\"invalid_samples\":" + std::to_string(m_invalid[i]) + ",\"data\":" +
             (sample.data.empty() ? "null" : sample.data) + '}';
      services().publish(family.variable, out);
    }
  }

private:
  struct Sample { Stamp stamp; std::string data; };
  std::array<Sample, families.size()> m_samples{};
  std::array<uint64_t, families.size()> m_invalid{};
};
} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi, mavmoos::runtime::Services* services)
{
  return abi == mavmoos::runtime::plugin_abi && services ? new mavmoos::runtime::NavigationMonitor(*services) : nullptr;
}

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin) { delete plugin; }
