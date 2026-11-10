#pragma once

#include "ControlBackend.h"
#include "Px4RoverPolicy.h"
#include <array>
#include <deque>

namespace mavmoos::runtime {
class Px4RoverBackend final : public VehicleControlBackend, public Px4RoverPolicy {
public:
  std::string profile() const override;
  HelmProfile helmProfile() const override;
  bool identityCompatible(unsigned type,unsigned autopilot) const override;
  bool flightReady(const Vehicle&,double now,bool motion) const override;
  bool acquisitionAllowed(const Vehicle&,double now) const override;
  bool activeModeAllowed(const Vehicle&,bool acquisition_grace) const override;
  std::optional<CommandRequest> acquisitionCommand(const Vehicle&,const std::string& id,double now) const override;
  bool commandAllowed(const CommandRequest&,const Vehicle&,double now) const override;
  bool missionExecutionCommandAllowed(const CommandRequest&,const Vehicle&,double now) const override;
  bool navigationInputAllowed(const NavigationTarget&) const override;
  bool helmFieldAllowed(unsigned index,double value) const override;
  bool helmVehicleReady(const Vehicle&,double now,double position_timeout,bool shadow,bool allow_acquisition) const override;
  std::optional<ControlFrame> navigationFrame(const NavigationTarget&,const Vehicle&,double now,uint64_t revision,mavlink_status_t&) const override;
  std::optional<ControlFrame> stopFrame(const Vehicle&,double now,uint64_t revision,mavlink_status_t&) const override;

  bool verifiedConfig(double now) const override;
  bool stopped(double now,double after) const override;
  bool missionReady(const Vehicle&,double now) const override;
  bool targetAllowed(const NavigationTarget&,const Vehicle&,double now) const override;
  bool holdObserved(double now,double after) const;
  bool missionObserved(double now,double after) const;
  void observe(const mavlink_message_t&,const MessageTime&,const Vehicle&,double now);
  void reset(double now);
  void readConfiguration(Services&);
  void parameterSent(const ParameterRequest&,double sent_at,bool accepted);

private:
  struct PositionSample {
    double at;
    uint64_t firmware_usec;
    int32_t lat,lon;
  };
  struct ConfigurationRead {
    ParameterRequest request;
    std::size_t index;
    double sent_at=-1;
  };

  bool connected(const Vehicle&,double now) const;
  bool gpsFresh(double now) const;
  bool groundStateReady(const Vehicle&,double now) const;
  void observeGps(const mavlink_message_t&,double acquired);
  void observeParameter(const mavlink_message_t&,const MessageTime&);

  double m_observation_floor=0;
  double m_gps_at=-1;
  uint64_t m_gps_usec=0;
  bool m_gps_valid=false;
  std::deque<PositionSample> m_slow_samples;
  double m_mode_at=-1;
  unsigned m_active_mode=0,m_intended_mode=0;
  std::array<double,4> m_config_at{{-1,-1,-1,-1}};
  std::array<unsigned,4> m_attempts{};
  double m_next_read=0;
  uint64_t m_read_sequence=0;
  bool m_conflict=false;
  std::optional<ConfigurationRead> m_read;
};
} // namespace mavmoos::runtime
