/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: RoverComponents.h                              */
/*    DATE: November 4, 2026                                */
/************************************************************/
#pragma once
#include "ControlBackend.h"
#include <memory>

namespace mavmoos::runtime {
namespace rover {
constexpr unsigned manual_mode=0,hold_mode=4,loiter_mode=5,rtl_mode=11,guided_mode=15;
constexpr double maximum_speed=3.0;
constexpr double boat_maximum_speed=2.0;
constexpr double speedLimit(bool boat) { return boat?boat_maximum_speed:maximum_speed; }
}
std::unique_ptr<VehicleControlBackend> makeArduRoverBackend(bool boat=false);
std::unique_ptr<Plugin> makeRoverCommands(Services&,const VehicleControlBackend&,Plugin* helm=nullptr);
std::unique_ptr<Plugin> makeArduRoverMission(Services&,const VehicleControlBackend&);
} // namespace mavmoos::runtime
