#pragma once
#include "PlaneComponents.h"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace plane_fixture {
using namespace mavmoos::runtime;

inline void require(bool condition,const char* description)
{
  if(!condition)throw std::runtime_error(description);
}

inline Vehicle airborne(double at,unsigned mode)
{
  Vehicle vehicle;
  vehicle.heartbeat_valid=vehicle.position_valid=vehicle.home_valid=vehicle.landed_valid=true;
  vehicle.estimator_valid=vehicle.estimator_horizontal_valid=vehicle.armed=vehicle.in_air=true;
  vehicle.type=MAV_TYPE_FIXED_WING;
  vehicle.autopilot=MAV_AUTOPILOT_ARDUPILOTMEGA;
  vehicle.mode=mode;
  vehicle.landed_state=MAV_LANDED_STATE_IN_AIR;
  vehicle.estimator_flags=EKF_ATTITUDE|EKF_VELOCITY_HORIZ|EKF_VELOCITY_VERT|EKF_POS_HORIZ_ABS|EKF_POS_VERT_ABS;
  vehicle.heartbeat_at=vehicle.position_at=vehicle.home_at=vehicle.landed_at=vehicle.estimator_at=at;
  vehicle.latitude=vehicle.home_latitude=43.;
  vehicle.longitude=vehicle.home_longitude=-71.;
  vehicle.home_msl=200.;
  vehicle.msl_height=275.;
  vehicle.relative_height=75.;
  vehicle.north_velocity=22.;
  return vehicle;
}

inline mavlink_message_t sensor(float speed=22.f,uint8_t flags=AIRSPEED_SENSOR_USING,
                                uint8_t id=0,uint8_t system=1,uint8_t component=1)
{
  mavlink_message_t message{};
  mavlink_msg_airspeed_pack(system,component,&message,id,speed,1500,10.f,flags);
  return message;
}

inline mavlink_message_t state(uint8_t vtol=MAV_VTOL_STATE_UNDEFINED,uint8_t landed=MAV_LANDED_STATE_IN_AIR)
{
  mavlink_message_t message{};
  mavlink_msg_extended_sys_state_pack(1,1,&message,vtol,landed);
  return message;
}

inline NavigationTarget intent(double expires,double course=0.)
{
  NavigationTarget target;
  target.course=course;
  target.speed=plane::cruise_ground_intent;
  target.height=75.;
  target.expires=expires;
  return target;
}
} // namespace plane_fixture
