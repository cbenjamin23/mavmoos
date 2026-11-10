#pragma once
#include "Plugin.h"
#include <memory>

namespace mavmoos::runtime {
class Px4RoverBackend;

// Private part of px4_rover_control. Runtime owns the durable authority lease
// and final send. This coordinator settles the operator's bounded requests.
std::unique_ptr<Plugin> makePx4RoverMission(Services&, const Px4RoverBackend&);
}
