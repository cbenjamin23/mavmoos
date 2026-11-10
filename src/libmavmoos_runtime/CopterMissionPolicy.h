#pragma once

#include "Plugin.h"
#include <algorithm>
#include <cmath>

namespace mavmoos::runtime::copter_mission {
inline bool takeoffPlan(const std::vector<MissionItem>& items)
{ return items.size()>1&&items[1].command==MAV_CMD_NAV_TAKEOFF; }

inline bool landingPlan(const std::vector<MissionItem>& items)
{ return items.size()>1&&items.back().command==MAV_CMD_NAV_LAND; }

inline double distance(double latitude,double longitude,const MissionItem& point)
{
  constexpr double rad=3.14159265358979323846/180.;
  return 6371000.*rad*std::hypot(point.x*1e-7-latitude,
    std::remainder(point.y*1e-7-longitude,360.)*std::cos(latitude*rad));
}

inline bool ground(const Vehicle& vehicle)
{
  return vehicle.landed_state==MAV_LANDED_STATE_ON_GROUND&&
    std::abs(vehicle.relative_height)<=1&&std::abs(vehicle.down_velocity)<=.35&&
    std::hypot(vehicle.north_velocity,vehicle.east_velocity)<=.5;
}

inline bool airbornePhase(const Vehicle& vehicle)
{
  return vehicle.in_air||vehicle.landed_state==MAV_LANDED_STATE_TAKEOFF||
    vehicle.landed_state==MAV_LANDED_STATE_LANDING;
}

inline bool entryState(const Vehicle& vehicle,const std::vector<MissionItem>& items)
{
  return vehicle.armed&&vehicle.mode==4&&
    (takeoffPlan(items)?ground(vehicle)&&distance(vehicle.latitude,vehicle.longitude,items[1])<=2:vehicle.in_air);
}

inline bool groundedAtLandingPoint(const Vehicle& vehicle,const std::vector<MissionItem>& items)
{
  return landingPlan(items)&&ground(vehicle)&&distance(vehicle.latitude,vehicle.longitude,items.back())<=3;
}

inline bool landingNearGround(const Vehicle& vehicle,const std::vector<MissionItem>& items)
{
  return landingPlan(items)&&
    (vehicle.landed_state==MAV_LANDED_STATE_LANDING||vehicle.landed_state==MAV_LANDED_STATE_ON_GROUND)&&
    std::abs(vehicle.relative_height)<=1&&std::hypot(vehicle.north_velocity,vehicle.east_velocity)<=.5&&
    distance(vehicle.latitude,vehicle.longitude,items.back())<=3;
}

inline bool validPlan(const std::vector<MissionItem>& items,const Vehicle& vehicle)
{
  if(items.size()<2||items.size()>8||!vehicle.home_valid||!std::isfinite(vehicle.home_latitude)||
     !std::isfinite(vehicle.home_longitude)||!std::isfinite(vehicle.home_msl))return false;
  unsigned waypoints=0;
  for(std::size_t n=0;n<items.size();++n) {
    const auto& item=items[n];
    if(item.seq!=n||item.autocontinue!=1||item.current>1||!std::isfinite(item.z)||
       std::abs(double(item.x))>9e8||std::abs(double(item.y))>18e8||
       !std::all_of(item.params.begin(),item.params.end(),[](float p){return std::isfinite(p);})||
       item.params[1]!=0||item.params[2]!=0)return false;
    const auto range=distance(vehicle.home_latitude,vehicle.home_longitude,item);
    if(n==0) {
      if(item.command!=MAV_CMD_NAV_WAYPOINT||item.frame!=MAV_FRAME_GLOBAL||range>1.5||
         std::abs(item.z-vehicle.home_msl)>.5||item.params[0]!=0||item.params[3]!=0)return false;
      continue;
    }
    if(item.frame!=MAV_FRAME_GLOBAL_RELATIVE_ALT||(item.x==0&&item.y==0)||range>100)return false;
    if(item.command==MAV_CMD_NAV_LAND) {
      // ArduPilot returns +1 for its default stored LAND direction.
      if(n+1!=items.size()||item.z!=0||item.params[0]!=0||
         (item.params[3]!=0&&item.params[3]!=1))return false;
      continue;
    }
    if(item.z<1||item.z>20||item.params[3]!=0)return false;
    if(item.command==MAV_CMD_NAV_TAKEOFF) {
      if(n!=1||item.z<3||range>1.5||item.params[0]!=0)return false;
    } else if(item.command==MAV_CMD_NAV_WAYPOINT) {
      if(item.params[0]<0||item.params[0]>30||std::trunc(item.params[0])!=item.params[0])return false;
      ++waypoints;
    } else return false;
  }
  return waypoints>0;
}
} // namespace mavmoos::runtime::copter_mission
