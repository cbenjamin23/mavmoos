#pragma once
#include "ControlComponents.h"

namespace mavmoos::runtime {
// Private component of the ArduCopter control provider, not a separate plugin.
// Default disabled. The host owns stored-mission proof and persistent authority;
// this component owns the bounded simple-AUTO execution state machine.
// Requests: MAVMOOS_MISSION_CONTROL, schema=1, id, action=start|pause|abort|release.
// start adds verification_id, content_id, proof_revision, timeout(1..120s).
// Other actions add target=<start id>, timeout(1..15s).
// Results/status/progress: MAVMOOS_MISSION_EXECUTION_{RESULT,STATUS,PROGRESS}.
// The provider forwards metadata-bearing messages and actual command receipts.
std::unique_ptr<Plugin> makeCopterMissionExecution(Services&);
} // namespace mavmoos::runtime
