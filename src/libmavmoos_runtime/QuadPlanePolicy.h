#pragma once

#include "Plugin.h"
#include <algorithm>
#include <cmath>

namespace mavmoos::runtime::quadplane {
inline constexpr char profile[]="arduplane_quadplane";
inline constexpr unsigned auto_mode=10, qloiter=19, qland=20, qrtl=21;
inline constexpr unsigned takeoff_sequence=1, forward_sequence=2, land_sequence=3;
inline constexpr std::size_t plan_items=4;
inline constexpr char hover_capability[]="control.navigation.hover";
inline constexpr double hover_min_height=25.,hover_max_height=90.,hover_max_intent=3.;
inline constexpr double hover_offer_age=.35,hover_max_lease=.75;
inline constexpr unsigned hover_marker=0x3148514d; // Wire bytes MQH1
inline constexpr unsigned hover_hold_offer=3,hover_guided_offer=1,hover_target=2;
inline constexpr unsigned hover_offer_bytes=10,hover_target_bytes=24;

inline bool fresh(double at,double now,double limit)
{ return std::isfinite(at)&&std::isfinite(now)&&at>=0&&now>=at&&now-at<=limit; }

inline bool identity(const Vehicle& vehicle)
{ return vehicle.type==MAV_TYPE_VTOL_FIXEDROTOR && vehicle.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA; }

inline bool observed(const Vehicle& vehicle,double now)
{
  return identity(vehicle)&&vehicle.heartbeat_valid&&fresh(vehicle.heartbeat_at,now,2)&&
    vehicle.position_valid&&fresh(vehicle.position_at,now,.75)&&
    vehicle.home_valid&&fresh(vehicle.home_at,now,5)&&std::isfinite(vehicle.relative_height)&&
    vehicle.landed_valid&&fresh(vehicle.landed_at,now,2);
}

inline bool stoppedOnGround(const Vehicle& vehicle,double now)
{
  return observed(vehicle,now)&&vehicle.landed_state==MAV_LANDED_STATE_ON_GROUND&&!vehicle.in_air&&
    std::abs(vehicle.relative_height)<=4&&std::abs(vehicle.down_velocity)<=1&&
    std::hypot(vehicle.north_velocity,vehicle.east_velocity)<=1;
}

inline double separation(double lat,double lon,int32_t to_lat,int32_t to_lon)
{
  constexpr double rad=3.14159265358979323846/180.;
  return 6371000.*rad*std::hypot(to_lat*1e-7-lat,
    std::remainder(to_lon*1e-7-lon,360.)*std::cos(lat*rad));
}

inline bool hoverField(unsigned index,double value)
{
  if(!std::isfinite(value))return false;
  if(index==0)return value>=0&&value<=360;
  if(index==1)return value>=0&&value<=hover_max_intent;
  return index==2&&value>=hover_min_height&&value<=hover_max_height;
}

inline bool hoverTarget(const NavigationTarget& n)
{
  return n.kind==NavigationKind::CourseSpeed&&n.dimensions==NavigationDimensions::Spatial&&
    hoverField(0,n.course)&&hoverField(1,n.speed)&&hoverField(2,n.height);
}

inline bool hoverReady(const Vehicle& v,double now,double position_timeout)
{
  constexpr unsigned required=EKF_ATTITUDE|EKF_VELOCITY_HORIZ|EKF_VELOCITY_VERT|
    EKF_POS_HORIZ_ABS|EKF_POS_VERT_ABS;
  constexpr unsigned invalid=EKF_CONST_POS_MODE|EKF_UNINITIALIZED|EKF_GPS_GLITCHING;
  return observed(v,now)&&v.armed&&v.in_air&&v.landed_state==MAV_LANDED_STATE_IN_AIR&&
    v.estimator_valid&&v.estimator_horizontal_valid&&fresh(v.estimator_at,now,3)&&
    (v.estimator_flags&required)==required&&!(v.estimator_flags&invalid)&&
    fresh(v.position_at,now,std::min(position_timeout,.5))&&fresh(v.home_at,now,3)&&
    std::isfinite(v.latitude)&&std::abs(v.latitude)<89.9&&std::isfinite(v.longitude)&&std::abs(v.longitude)<=180&&
    std::isfinite(v.home_latitude)&&std::abs(v.home_latitude)<89.9&&std::isfinite(v.home_longitude)&&
    std::abs(v.home_longitude)<=180&&std::isfinite(v.home_msl)&&std::isfinite(v.msl_height)&&
    std::abs(v.msl_height-v.home_msl-v.relative_height)<=2&&
    hoverField(2,v.relative_height)&&std::hypot(v.north_velocity,v.east_velocity)<=5&&
    std::abs(v.down_velocity)<=2&&separation(v.home_latitude,v.home_longitude,
      static_cast<int32_t>(std::round(v.latitude*1e7)),static_cast<int32_t>(std::round(v.longitude*1e7)))<=1100;
}

inline uint32_t hoverUint(const uint8_t* bytes,unsigned width)
{
  uint32_t result=0;
  for(unsigned i=0;i<width;++i)result|=uint32_t(bytes[i])<<(8*i);
  return result;
}

inline void hoverBytes(uint8_t* bytes,uint32_t value,unsigned width)
{
  for(unsigned i=0;i<width;++i)bytes[i]=static_cast<uint8_t>(value>>(8*i));
}

inline bool validItem(const MissionItem& item)
{
  constexpr uint16_t commands[]={MAV_CMD_NAV_WAYPOINT,MAV_CMD_NAV_VTOL_TAKEOFF,
    MAV_CMD_NAV_WAYPOINT,MAV_CMD_NAV_VTOL_LAND};
  if(item.seq>=plan_items||item.command!=commands[item.seq]||item.autocontinue!=1||item.current>1||
     !std::isfinite(item.z)||std::abs(double(item.x))>=9e8||std::abs(double(item.y))>18e8||
     !std::all_of(item.params.begin(),item.params.end(),[](float p){return p==0;}))return false;
  if(item.seq==0)return item.frame==MAV_FRAME_GLOBAL&&item.z>=-1000&&item.z<=20000;
  if(item.frame!=MAV_FRAME_GLOBAL_RELATIVE_ALT||(item.x==0&&item.y==0))return false;
  if(item.seq==land_sequence)return item.z==0;
  return item.z>=25&&item.z<=55;
}

inline bool validPlan(const std::vector<MissionItem>& items,const Vehicle& vehicle)
{
  if(items.size()!=plan_items||!vehicle.home_valid||!std::isfinite(vehicle.home_msl))return false;
  for(std::size_t i=0;i<items.size();++i)
    if(items[i].seq!=i||!validItem(items[i]))return false;
  const auto range=[&](const MissionItem& point) {
    return separation(vehicle.home_latitude,vehicle.home_longitude,point.x,point.y);
  };
  return range(items[0])<=8&&std::abs(items[0].z-vehicle.home_msl)<=4&&
    range(items[takeoff_sequence])<=8&&range(items[forward_sequence])>=250&&
    range(items[forward_sequence])<=500&&range(items[land_sequence])<=40;
}

inline bool modeCommand(const CommandRequest& r,unsigned mode)
{
  return r.command==MAV_CMD_DO_SET_MODE&&r.params[0]==MAV_MODE_FLAG_CUSTOM_MODE_ENABLED&&r.params[1]==mode&&
    std::all_of(r.params.begin()+2,r.params.end(),[](float value){return value==0;});
}

inline bool armCommand(const CommandRequest& r,bool arm)
{
  return r.command==MAV_CMD_COMPONENT_ARM_DISARM&&r.params[0]==(arm?1:0)&&
    std::all_of(r.params.begin()+1,r.params.end(),[](float value){return value==0;});
}

// Internal RTTI interface. The shared plugin and service layouts stay ABI 10.
class PhasePolicy {
public:
  virtual ~PhasePolicy()=default;
  virtual unsigned phase(double now) const=0;
};
} // namespace mavmoos::runtime::quadplane
