/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: PlaneComponents.h                               */
/*    DATE: November 6, 2026                                */
/************************************************************/
#pragma once
#include "ControlBackend.h"
#include "PlaneOperationPolicy.h"

namespace mavmoos::runtime::plane {
inline constexpr unsigned rtl_mode=11,loiter_mode=12,guided_mode=15;
inline constexpr double cruise_ground_intent=22.0,lookahead_m=300.0;
inline constexpr double home_radius_m=1200.0,turn_reserve_m=120.0;
inline constexpr double minimum_target_height_m=45.0,maximum_target_height_m=120.0;
inline constexpr double airspeed_age_limit=1.5;

CommandRequest modeRequest(const std::string& id,unsigned mode,double expires);

class Backend final:public VehicleControlBackend {
public:
  explicit Backend(double now,bool quadplane=false):m_quadplane(quadplane),m_observation_floor(now) {}
  std::string profile() const override { return "arduplane"; }
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
  std::optional<ControlFrame> navigationFrame(const NavigationTarget&,const Vehicle&,double now,
                                               uint64_t revision,mavlink_status_t&) const override;
  std::optional<ControlFrame> stopFrame(const Vehicle&,double,uint64_t,mavlink_status_t&) const override
  { return std::nullopt; }

  void observe(const mavlink_message_t&,const MessageTime&,const Vehicle&,double now);
  void reset(double now);
  const char* readiness(const Vehicle&,double now,double position_timeout=0.5) const;
  bool groundReady(const Vehicle&,double now) const;
  bool prearmReady(double now) const;
  bool directTargetAllowed(const NavigationTarget&,const Vehicle&) const;
  std::optional<NavigationTarget> directCourseTarget(const Vehicle&,double course) const;
  std::optional<double> airspeed() const { return m_airspeed; }
  double airspeedAt() const { return m_airspeed_time.effective_at; }
private:
  bool m_quadplane;
  double m_observation_floor;
  std::optional<double> m_airspeed;
  MessageTime m_airspeed_time,m_airframe_time;
  double m_prearm_at=0;
  bool m_prearm_ok=false;
  bool m_forward_airborne=false,m_conventional_ground=false;
};
} // namespace mavmoos::runtime::plane
