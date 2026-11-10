#pragma once
#include "Plugin.h"

namespace mavmoos::runtime {
// Initial pinned ArduPilot observation profile, usable without motion control.
// Keep the existing stream requests for parameter-only/telemetry configurations.
inline std::vector<ObservationStream> arduPilotObservationStreams()
{
  return {{33,50000.f},{245,250000.f},{193,500000.f},{242,500000.f}};
}
} // namespace mavmoos::runtime
