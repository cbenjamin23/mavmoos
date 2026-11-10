#pragma once
#include "ControlBackend.h"
#include <algorithm>
#include <cmath>

namespace mavmoos::runtime {
// Optional policy for the selected Ackermann backend. The host retains mode
// evidence, authority and the final handoff. Existing backend ABI is unchanged.
class Px4RoverPolicy {
public:
  virtual ~Px4RoverPolicy()=default;
  virtual bool verifiedConfig(double now) const=0;
  virtual bool stopped(double now,double after) const=0;
  virtual bool missionReady(const Vehicle&,double now) const=0;
  virtual bool targetAllowed(const NavigationTarget&,const Vehicle&,double now) const=0;
};

class Px4RoverMissionBinding {
public:
  virtual ~Px4RoverMissionBinding()=default;
  virtual bool recordPx4MissionProof(const std::string&,const std::vector<MissionItem>&,uint32_t)=0;
  virtual uint32_t verifiedPx4MissionId() const=0;
};

namespace px4_rover {
inline bool relativeFrame(unsigned frame)
{
  return frame==MAV_FRAME_GLOBAL_RELATIVE_ALT || frame==MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
}

inline bool validWaypoint(const MissionItem& item)
{
  return item.command==MAV_CMD_NAV_WAYPOINT && relativeFrame(item.frame) &&
    item.current<=1 && item.autocontinue==1 && item.z==0 &&
    std::abs(double(item.x))<850000000 && std::abs(double(item.y))<=1800000000 &&
    std::all_of(item.params.begin(),item.params.end(),[](float value){return value==0;});
}

inline bool validMission(const std::vector<MissionItem>& items)
{
  if(items.size()<2 || items.size()>8) return false;
  for(std::size_t i=0;i<items.size();++i)
    if(items[i].seq!=i || !validWaypoint(items[i])) return false;
  return true;
}
}
} // namespace mavmoos::runtime
