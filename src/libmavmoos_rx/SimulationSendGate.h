/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: SimulationSendGate.h                            */
/*    DATE: November 4, 2026                                 */
/************************************************************/
#pragma once

#include <functional>

namespace mavmoos::detail {
enum class SimulationSendReason {
  Accepted,
  InvalidPermit,
  ClockInvalid,
  Expired,
  EndpointRejected,
  Ineligible,
  PreflightException,
  WriteFailed,
  WriteException
};

struct SimulationSendOutcome {
  SimulationSendReason reason=SimulationSendReason::InvalidPermit;
  bool write_attempted=false;
  double checked_at=-1;
  bool accepted() const { return reason==SimulationSendReason::Accepted; }
};

// Internal last-handoff policy, not a feature-plugin transport service. Tests
// inject proof, clock and I/O; the real receiver retains its private descriptor.
// eligible must be a cheap owner-thread recheck. Expiry is sampled again after
// it, but scheduling between the final userspace check and sendto is not atomic.
SimulationSendOutcome attemptSimulationSend(
  double expires,
  const std::function<bool()>& prove_endpoint,
  const std::function<double()>& clock,
  const std::function<bool(double)>& eligible,
  const std::function<bool()>& write) noexcept;
} // namespace mavmoos::detail
