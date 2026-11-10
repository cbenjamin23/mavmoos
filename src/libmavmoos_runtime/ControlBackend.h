#pragma once
#include "Plugin.h"
#include <cstddef>
#include <optional>

namespace mavmoos::runtime {
// Pure, startup-owned policy/encoding. It has no socket, MOOS or send service.
// The host retains authority, original deadlines, queueing and the final gate.
struct ControlFrame {
  std::array<uint8_t,MAVLINK_MAX_PACKET_LEN> bytes{};
  std::size_t size=0;
};
struct HelmProfile {
  NavigationDimensions dimensions=NavigationDimensions::Spatial;
  bool requires_height=true,requires_home=true;
};
class VehicleControlBackend {
public:
  virtual ~VehicleControlBackend()=default;
  virtual std::string profile() const=0;
  virtual HelmProfile helmProfile() const { return {}; }
  virtual bool identityCompatible(unsigned type,unsigned autopilot) const=0;
  virtual bool flightReady(const Vehicle&,double now,bool motion) const=0;
  virtual bool acquisitionAllowed(const Vehicle&,double now) const=0;
  virtual bool activeModeAllowed(const Vehicle&,bool acquisition_grace) const=0;
  virtual std::optional<CommandRequest> acquisitionCommand(const Vehicle&,const std::string& id,double now) const=0;
  virtual bool commandAllowed(const CommandRequest&,const Vehicle&,double now) const=0;
  // Vehicle semantics only; the host independently validates the execution or
  // stop-only abort binding before admission and at the actual socket handoff.
  virtual bool missionExecutionCommandAllowed(const CommandRequest&,const Vehicle&,double) const { return false; }
  virtual bool navigationInputAllowed(const NavigationTarget&) const=0;
  virtual bool helmFieldAllowed(unsigned index,double value) const=0;
  virtual bool helmVehicleReady(const Vehicle&,double now,double position_timeout,bool shadow,bool allow_acquisition) const=0;
  virtual std::optional<ControlFrame> navigationFrame(const NavigationTarget&,const Vehicle&,double now,uint64_t revision,mavlink_status_t&) const=0;
  virtual std::optional<ControlFrame> stopFrame(const Vehicle&,double now,uint64_t revision,mavlink_status_t&) const=0;
};
} // namespace mavmoos::runtime
