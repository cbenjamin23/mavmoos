#pragma once
#include <ardupilotmega/mavlink.h>
#include <limits>

namespace mavmoos::test::navigation_frames {

inline mavlink_message_t controller(mavlink_status_t& codec, uint8_t system = 1, uint8_t component = 1)
{
  mavlink_nav_controller_output_t p{};
  p.nav_roll = 4.25f;
  p.nav_pitch = -8.5f;
  p.nav_bearing = -115;
  p.target_bearing = 295;
  p.wp_dist = 513;
  p.alt_error = -4.25f;
  p.aspd_error = 2.5f;
  p.xtrack_error = -2.75f;
  mavlink_message_t result{};
  mavlink_msg_nav_controller_output_encode_status(system, component, &codec, &result, &p);
  return result;
}

inline mavlink_message_t local(mavlink_status_t& codec, bool bad = false, uint8_t frame = MAV_FRAME_LOCAL_NED)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  mavlink_position_target_local_ned_t p{};
  p.time_boot_ms = 4294967295u;
  p.coordinate_frame = frame;
  p.type_mask = POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
    POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE | POSITION_TARGET_TYPEMASK_FORCE_SET;
  p.x = bad ? nan : 1.25f;
  p.y = nan;
  p.z = -3.5f;
  p.vx = 5;
  p.vy = -1.25f;
  p.vz = nan;
  p.afx = -.5f;
  p.afy = nan;
  p.yaw = .25f;
  p.yaw_rate = nan;
  mavlink_message_t result{};
  mavlink_msg_position_target_local_ned_encode_status(1, 1, &codec, &result, &p);
  return result;
}

inline mavlink_message_t global(mavlink_status_t& codec, bool bad = false, uint8_t frame = MAV_FRAME_GLOBAL)
{
  mavlink_position_target_global_int_t p{};
  p.time_boot_ms = 121;
  p.coordinate_frame = frame;
  p.lat_int = bad ? 900000001 : 423584560;
  p.lon_int = -710875890;
  p.alt = 65.5f;
  p.type_mask = 0xF000 | POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE |
    POSITION_TARGET_TYPEMASK_VZ_IGNORE | POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE |
    POSITION_TARGET_TYPEMASK_AZ_IGNORE | POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
  mavlink_message_t result{};
  mavlink_msg_position_target_global_int_encode_status(1, 1, &codec, &result, &p);
  return result;
}
}
