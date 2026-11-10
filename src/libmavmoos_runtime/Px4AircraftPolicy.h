#pragma once

namespace mavmoos::runtime {
// Optional PX4 aircraft policy implemented by the selected control backend.
// A standard VTOL's prestream uses the continuation for its current phase.
class Px4AircraftPolicy {
public:
  virtual ~Px4AircraftPolicy()=default;
  virtual bool forwardFlight(double now) const=0;
  virtual bool nativeActionsEnabled() const=0;
};
} // namespace mavmoos::runtime
