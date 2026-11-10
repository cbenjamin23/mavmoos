#pragma once
#include "Plugin.h"
#include <algorithm>
#include <cmath>

namespace mavmoos::runtime::plane {
inline constexpr double operator_min_airspeed=15.,operator_max_airspeed=28.;
inline constexpr unsigned manual_mode=0,auto_mode=10,autoland_mode=26;
inline constexpr int native_approach_distance_m=485,native_approach_height_m=45;

inline bool freshState(double at,double now,double age)
{ return std::isfinite(at)&&std::isfinite(now)&&at>0&&now>=at&&now-at<=age; }

inline bool groundState(const Vehicle& v,double now)
{
  return v.type==MAV_TYPE_FIXED_WING&&v.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA&&
    v.heartbeat_valid&&freshState(v.heartbeat_at,now,2)&&v.landed_valid&&!v.in_air&&
    v.landed_state==MAV_LANDED_STATE_ON_GROUND&&freshState(v.landed_at,now,2)&&
    v.position_valid&&freshState(v.position_at,now,.5)&&
    std::abs(v.relative_height)<=4&&std::hypot(v.north_velocity,v.east_velocity)<=1.5&&
    std::abs(v.down_velocity)<=1.5;
}

inline bool armCommand(const CommandRequest& r)
{
  return r.command==MAV_CMD_COMPONENT_ARM_DISARM&&(r.params[0]==0||r.params[0]==1)&&
    std::all_of(r.params.begin()+1,r.params.end(),[](float value){return value==0;});
}

inline bool manualCommand(const CommandRequest& r)
{
  return r.command==MAV_CMD_DO_SET_MODE&&r.params[0]==MAV_MODE_FLAG_CUSTOM_MODE_ENABLED&&
    std::all_of(r.params.begin()+1,r.params.end(),[](float value){return value==0;});
}

inline bool speedCommand(const CommandRequest& r)
{
  return r.command==MAV_CMD_GUIDED_CHANGE_SPEED&&r.params[0]==0&&
    r.params[1]>=operator_min_airspeed&&r.params[1]<=operator_max_airspeed&&r.params[2]==2&&
    std::all_of(r.params.begin()+3,r.params.end(),[](float value){return value==0;});
}

inline constexpr unsigned lifecycle_items=6,takeoff_item=1,takeoff_hold_item=2,
  approach_entry_item=3,final_approach_item=4,landing_item=5;

inline bool lifecycleItem(const MissionItem& item)
{
  constexpr uint16_t commands[]={MAV_CMD_NAV_WAYPOINT,MAV_CMD_NAV_TAKEOFF,MAV_CMD_NAV_LOITER_UNLIM,
    MAV_CMD_NAV_WAYPOINT,MAV_CMD_NAV_WAYPOINT,MAV_CMD_NAV_LAND};
  if(item.seq>=lifecycle_items||item.command!=commands[item.seq]||item.autocontinue!=1||item.current>1||
     std::abs(double(item.x))>=9e8||std::abs(double(item.y))>18e8||!std::isfinite(item.z)||
     item.params[1]!=0||item.params[2]!=0)return false;
  if(item.seq==0)return item.frame==MAV_FRAME_GLOBAL&&item.z>=-1000&&item.z<=20000&&
    item.params[0]==0&&item.params[3]==0;
  if(item.frame!=MAV_FRAME_GLOBAL_RELATIVE_ALT||(item.x==0&&item.y==0))return false;
  if(item.seq==takeoff_item)return item.z>=55&&item.z<=75&&
    item.params[0]>=10&&item.params[0]<=20&&std::trunc(item.params[0])==item.params[0]&&item.params[3]==0;
  if(item.seq==landing_item)return item.z==0&&item.params[0]==0&&item.params[3]==1;
  if(item.params[0]!=0||item.params[3]!=0)return false;
  return item.seq==takeoff_hold_item?(item.z>=55&&item.z<=85):(item.z>=30&&item.z<=65);
}

inline double distance(double latitude,double longitude,int32_t x,int32_t y)
{
  constexpr double radians=3.14159265358979323846/180.,metres=6378137.*radians;
  return std::hypot((x*1e-7-latitude)*metres,
    std::remainder(y*1e-7-longitude,360.)*metres*std::cos(latitude*radians));
}

inline bool lifecyclePlan(const std::vector<MissionItem>& items,const Vehicle& vehicle)
{
  if(items.size()!=lifecycle_items||!vehicle.home_valid||!std::isfinite(vehicle.home_msl))return false;
  for(unsigned i=0;i<items.size();++i) {
    if(items[i].seq!=i||!lifecycleItem(items[i])||
       !(distance(vehicle.home_latitude,vehicle.home_longitude,items[i].x,items[i].y)<=850))return false;
  }
  const auto& entry=items[approach_entry_item];
  const auto& approach=items[final_approach_item];
  const auto& touchdown=items[landing_item];
  const double final_leg=distance(approach.x*1e-7,approach.y*1e-7,touchdown.x,touchdown.y);
  const double align_leg=distance(entry.x*1e-7,entry.y*1e-7,approach.x,approach.y);
  const double route_length=distance(entry.x*1e-7,entry.y*1e-7,touchdown.x,touchdown.y);
  const double glide=final_leg/approach.z;
  return distance(vehicle.home_latitude,vehicle.home_longitude,items[0].x,items[0].y)<=8&&
    std::abs(items[0].z-vehicle.home_msl)<=4&&
    distance(vehicle.home_latitude,vehicle.home_longitude,touchdown.x,touchdown.y)<=100&&
    align_leg>=160&&align_leg<=360&&std::abs(route_length-final_leg-align_leg)<=2&&
    std::abs(entry.z-approach.z)<=.02&&glide>=6&&glide<=15;
}

inline bool nativeLandingPlan(const std::vector<MissionItem>& items,const Vehicle& vehicle,double launch_heading)
{
  if(items.size()!=lifecycle_items||!std::isfinite(launch_heading)||launch_heading<0||launch_heading>=360||
     std::abs(vehicle.home_latitude)>=80)return false;
  constexpr double radians=3.14159265358979323846/180.,metres=6378137.*radians;
  const double heading=launch_heading*radians;
  const double latitude=vehicle.home_latitude-native_approach_distance_m*std::cos(heading)/metres;
  const double longitude=vehicle.home_longitude-native_approach_distance_m*std::sin(heading)/
    (metres*std::cos(vehicle.home_latitude*radians));
  const auto& approach=items[final_approach_item];
  const auto& touchdown=items[landing_item];
  return distance(latitude,longitude,approach.x,approach.y)<=25&&
    std::abs(approach.z-native_approach_height_m)<=.02&&
    distance(vehicle.home_latitude,vehicle.home_longitude,touchdown.x,touchdown.y)<=8;
}
} // namespace mavmoos::runtime::plane
