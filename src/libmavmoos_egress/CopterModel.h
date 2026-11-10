/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: CopterModel.h                                   */
/*    DATE: October 29, 2026                                */
/************************************************************/
#pragma once
#include "Types.h"

namespace mavmoos::egress {
bool validEnvelope(const Envelope& envelope);
Evaluation guard(const VehicleState& state, const Envelope& envelope, uint64_t home,
                 Tick now, bool need_global = false, bool probe_only = false);
Build navigation(const NavInput& input, const VehicleState& state, const Envelope& envelope,
                 uint64_t home, Tick now, Tick generation_boundary);
Build hold(const VehicleState& state, const Envelope& envelope, uint64_t home, Tick now);
Frame encode(const Intent& intent, const Binding& binding, uint32_t boot_ms,
             mavlink_status_t& status);
} // namespace mavmoos::egress
