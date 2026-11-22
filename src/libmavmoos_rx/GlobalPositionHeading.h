#pragma once
#include <ardupilotmega/mavlink.h>
#include <cstdint>
#include <optional>

namespace mavmoos {
inline std::optional<double> globalPositionHeadingDegrees(uint16_t centidegrees,int autopilot)
{
  if(centidegrees<36000) return centidegrees/100.;
  // PX4 can report north as exactly one turn in GLOBAL_POSITION_INT.
  if(centidegrees==36000 && autopilot==MAV_AUTOPILOT_PX4) return 0.;
  return std::nullopt;
}
}
