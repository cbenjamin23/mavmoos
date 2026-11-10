#pragma once

#include "Plugin.h"
#include <algorithm>
#include <cmath>

namespace mavmoos::runtime {
namespace ardu_rover_mission {

constexpr unsigned auto_mode=10, hold_mode=4, loiter_mode=5;
constexpr double cursor_age=2.5;
constexpr double pi=3.14159265358979323846;

inline bool fresh(double stamp,double now,double max_age)
{
  return std::isfinite(stamp) && std::isfinite(now) && stamp>0 && now>=stamp && now-stamp<=max_age;
}

inline double separation(double latitude,double longitude,double target_lat,double target_lon)
{
  constexpr double metres_per_degree=6371000.*pi/180.;
  const auto north=(target_lat-latitude)*metres_per_degree;
  const auto east=std::remainder(target_lon-longitude,360.)*metres_per_degree*std::cos(latitude*pi/180.);
  return std::hypot(north,east);
}

inline unsigned handbackMode(bool boat)
{ return boat?loiter_mode:hold_mode; }

inline bool handbackModeAllowed(bool boat,unsigned mode)
{ return mode==hold_mode || (boat && mode==loiter_mode); }

inline bool itemValid(const MissionItem& item)
{
  return item.command==MAV_CMD_NAV_WAYPOINT && item.current<=1 && item.autocontinue==1 &&
    std::isfinite(item.z) && std::abs(double(item.x))<850000000 && std::abs(double(item.y))<=1800000000 &&
    (item.seq==0 ? item.frame==MAV_FRAME_GLOBAL : item.frame==MAV_FRAME_GLOBAL_RELATIVE_ALT && item.z==0) &&
    std::all_of(item.params.begin(),item.params.end(),[](float value){return value==0;});
}

inline bool boundPoint(const MissionItem& item,const Vehicle& vehicle)
{
  if(!itemValid(item) || !vehicle.home_valid ||
     !std::isfinite(vehicle.home_latitude) || std::abs(vehicle.home_latitude)>=85 ||
     !std::isfinite(vehicle.home_longitude) || std::abs(vehicle.home_longitude)>180 ||
     !std::isfinite(vehicle.home_msl)) return false;
  const double distance=separation(vehicle.home_latitude,vehicle.home_longitude,item.x*1e-7,item.y*1e-7);
  return item.seq==0 ? distance<=1.5 && std::abs(item.z-vehicle.home_msl)<=.75 : distance<=60;
}

inline bool planValid(const std::vector<MissionItem>& items,const Vehicle& vehicle)
{
  if(items.size()<3 || items.size()>8) return false;
  for(std::size_t i=0;i<items.size();++i)
    if(items[i].seq!=i || !boundPoint(items[i],vehicle)) return false;
  return true;
}

inline bool selectedCursor(const mavlink_mission_current_t& cursor,std::size_t count)
{ return count>=3 && cursor.total==count-1 && cursor.seq>=1 && cursor.seq<count; }

inline bool inactiveCursor(const mavlink_mission_current_t& cursor)
{ return cursor.mission_state==MISSION_STATE_NOT_STARTED && cursor.mission_mode==0; }

inline bool activeCursor(const mavlink_mission_current_t& cursor)
{ return cursor.mission_state==MISSION_STATE_ACTIVE && cursor.mission_mode==1; }

inline bool wireMode(const CommandRequest& request,unsigned mode)
{
  return request.command==MAV_CMD_DO_SET_MODE &&
    request.params[0]==MAV_MODE_FLAG_CUSTOM_MODE_ENABLED && request.params[1]==mode &&
    std::all_of(request.params.begin()+2,request.params.end(),[](float value){return value==0;});
}

} // namespace ardu_rover_mission

// Optional private profile policy. It does not change the host's provider ABI.
class ArduRoverMissionPolicy {
public:
  virtual ~ArduRoverMissionPolicy()=default;
  virtual bool ready(const Vehicle&,double now) const=0;
  virtual bool settled(const Vehicle&,double now,double after) const=0;
  virtual void observe(const mavlink_message_t&,const MessageTime&,const Vehicle&,double now)=0;
  virtual void readConfiguration(Services&)=0;
  virtual void parameterSent(const ParameterRequest&,double,bool)=0;
  virtual void reset(double)=0;
};

} // namespace mavmoos::runtime
