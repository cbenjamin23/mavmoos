#pragma once
#include <ardupilotmega/mavlink.h>
#include <string_view>

namespace mavmoos::runtime {
inline bool supportedCopterType(unsigned type)
{
  return type==MAV_TYPE_QUADROTOR || type==MAV_TYPE_HEXAROTOR || type==MAV_TYPE_OCTOROTOR;
}

inline unsigned copterFrameType(std::string_view frame)
{
  if(frame=="quad") return MAV_TYPE_QUADROTOR;
  if(frame=="hexa") return MAV_TYPE_HEXAROTOR;
  if(frame=="octa") return MAV_TYPE_OCTOROTOR;
  return MAV_TYPE_GENERIC;
}
} // namespace mavmoos::runtime
