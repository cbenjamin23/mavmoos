/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: SimulationSendGate.cpp                          */
/*    DATE: November 4, 2026                                 */
/************************************************************/
#include "SimulationSendGate.h"
#include <cmath>

namespace mavmoos::detail {
SimulationSendOutcome attemptSimulationSend(
  double expires,
  const std::function<bool()>& prove_endpoint,
  const std::function<double()>& clock,
  const std::function<bool(double)>& eligible,
  const std::function<bool()>& write) noexcept
{
  SimulationSendOutcome outcome;
  if(!std::isfinite(expires) || expires<=0 || !prove_endpoint || !clock || !eligible || !write)
    return outcome;

  try {
    const double before=clock();
    if(!std::isfinite(before) || before<0) {
      outcome.reason=SimulationSendReason::ClockInvalid; return outcome;
    }
    outcome.checked_at=before;
    if(before>=expires) { outcome.reason=SimulationSendReason::Expired; return outcome; }
    if(!prove_endpoint()) { outcome.reason=SimulationSendReason::EndpointRejected; return outcome; }

    const double after_proof=clock();
    if(!std::isfinite(after_proof) || after_proof<before) {
      outcome.reason=SimulationSendReason::ClockInvalid; return outcome;
    }
    outcome.checked_at=after_proof;
    if(after_proof>=expires) { outcome.reason=SimulationSendReason::Expired; return outcome; }
    if(!eligible(after_proof)) { outcome.reason=SimulationSendReason::Ineligible; return outcome; }

    // Endpoint proof or eligibility may have taken time. Never grant either a
    // new lifetime; the original absolute expiry remains the final upper bound.
    const double handoff=clock();
    if(!std::isfinite(handoff) || handoff<after_proof) {
      outcome.reason=SimulationSendReason::ClockInvalid; return outcome;
    }
    outcome.checked_at=handoff;
    if(handoff>=expires) { outcome.reason=SimulationSendReason::Expired; return outcome; }
  } catch(...) {
    outcome.reason=SimulationSendReason::PreflightException; return outcome;
  }

  outcome.write_attempted=true;
  try {
    outcome.reason=write()?SimulationSendReason::Accepted:SimulationSendReason::WriteFailed;
  } catch(...) {
    // A writer exception cannot prove that no bytes escaped. The real writer
    // is a single nonblocking syscall and does not throw; this preserves the
    // distinction for injected or future backends rather than claiming recall.
    outcome.reason=SimulationSendReason::WriteException;
  }
  return outcome;
}
} // namespace mavmoos::detail
