#pragma once
#include "ControlBackend.h"
#include <cmath>
#include <string_view>

namespace mavmoos::runtime::px4 {
constexpr char multicopter_profile[]="px4_multicopter", fixed_wing_profile[]="px4_fixed_wing",
  standard_vtol_profile[]="px4_standard_vtol", rover_profile[]="px4_rover_ackermann";
inline bool isProfile(std::string_view profile)
{ return profile==multicopter_profile || profile==fixed_wing_profile ||
  profile==standard_vtol_profile || profile==rover_profile; }

constexpr unsigned manual=1u<<16, altitude=2u<<16, position=3u<<16, stabilized=7u<<16, offboard=6u<<16;
constexpr unsigned takeoff=(4u<<16)|(2u<<24), hold=(4u<<16)|(3u<<24), mission=(4u<<16)|(4u<<24),
  rtl=(4u<<16)|(5u<<24), land=(4u<<16)|(6u<<24);
constexpr double setpoint_period=.09, max_stream_gap=.35, prestream_duration=1.25;
constexpr double acquisition_lifetime=5, local_age=.35, position_age=.5, estimator_age=1.6;
constexpr double rover_heartbeat_age=2;
constexpr double fixed_wing_airspeed_age=.65, fixed_wing_lookahead_s=5;
constexpr double fixed_wing_max_ground_speed=35, fixed_wing_control_radius_m=500;
constexpr double radians=3.14159265358979323846/180., metres_per_degree=6371000.*radians;
constexpr unsigned healthy_flags=ESTIMATOR_ATTITUDE|ESTIMATOR_VELOCITY_HORIZ|ESTIMATOR_VELOCITY_VERT|
  ESTIMATOR_POS_HORIZ_REL|ESTIMATOR_POS_HORIZ_ABS|ESTIMATOR_POS_VERT_ABS|
  ESTIMATOR_PRED_POS_HORIZ_REL|ESTIMATOR_PRED_POS_HORIZ_ABS;
// In the pinned PX4 EKF the legacy CONST_POS_MODE bit also means vehicle_at_rest.
constexpr unsigned unhealthy_flags=ESTIMATOR_GPS_GLITCH|ESTIMATOR_ACCEL_ERROR;

inline bool fresh(double at,double now,double age)
{ return at>=0 && now>=at && now-at<=age; }

inline bool healthy(const mavlink_estimator_status_t& status)
{
  return (status.flags&healthy_flags)==healthy_flags && !(status.flags&unhealthy_flags) &&
    status.time_usec>0 && std::isfinite(status.pos_horiz_accuracy) && status.pos_horiz_accuracy>=0 &&
    std::isfinite(status.pos_vert_accuracy) && status.pos_vert_accuracy>=0;
}

struct LocalPoint { float north=0,east=0,down=0; };

inline ControlFrame encode(mavlink_set_position_target_local_ned_t value,double now,mavlink_status_t& codec)
{
  value.time_boot_ms=static_cast<uint32_t>(std::fmod(now*1000.,4294967296.));
  value.coordinate_frame=MAV_FRAME_LOCAL_NED;
  value.target_system=1; value.target_component=1;
  mavlink_message_t message{};
  mavlink_msg_set_position_target_local_ned_encode_status(245,191,&codec,&message,&value);
  ControlFrame result;
  result.size=mavlink_msg_to_send_buffer(result.bytes.data(),&message);
  return result;
}
constexpr uint16_t ignore_acceleration=POSITION_TARGET_TYPEMASK_AX_IGNORE|POSITION_TARGET_TYPEMASK_AY_IGNORE|
  POSITION_TARGET_TYPEMASK_AZ_IGNORE;
constexpr uint16_t position_mask=ignore_acceleration|POSITION_TARGET_TYPEMASK_VX_IGNORE|
  POSITION_TARGET_TYPEMASK_VY_IGNORE|POSITION_TARGET_TYPEMASK_VZ_IGNORE|
  POSITION_TARGET_TYPEMASK_YAW_IGNORE|POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
inline ControlFrame holdFrame(const LocalPoint& p,double now,mavlink_status_t& codec)
{
  mavlink_set_position_target_local_ned_t value{};
  value.type_mask=position_mask;
  value.x=p.north;value.y=p.east;value.z=p.down;
  return encode(value,now,codec);
}

inline ControlFrame velocityFrame(float north,float east,double now,mavlink_status_t& codec)
{
  mavlink_set_position_target_local_ned_t value{};
  value.type_mask=ignore_acceleration|POSITION_TARGET_TYPEMASK_X_IGNORE|POSITION_TARGET_TYPEMASK_Y_IGNORE|
    POSITION_TARGET_TYPEMASK_Z_IGNORE|POSITION_TARGET_TYPEMASK_YAW_IGNORE|POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
  value.vx=north;
  value.vy=east;
  return encode(value,now,codec);
}

inline bool insideFixedWingArea(const Vehicle& v,double north=0,double east=0)
{
  if(std::abs(v.latitude)>=89 || std::abs(v.home_latitude)>=89 ||
     std::abs(v.longitude)>180 || std::abs(v.home_longitude)>180) return false;
  const double from_home_north=(v.latitude-v.home_latitude)*metres_per_degree;
  const double from_home_east=std::remainder(v.longitude-v.home_longitude,360.)*metres_per_degree*
    std::cos((v.latitude+v.home_latitude)*.5*radians);
  return std::hypot(from_home_north+north,from_home_east+east)<=fixed_wing_control_radius_m;
}
} // namespace mavmoos::runtime::px4
