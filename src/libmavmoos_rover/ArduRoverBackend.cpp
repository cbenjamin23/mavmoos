/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ArduRoverBackend.cpp                            */
/*    DATE: November 4, 2026                                */
/************************************************************/
#include "RoverComponents.h"
#include "RoverMissionEvidence.h"
#include <cmath>

namespace mavmoos::runtime {
namespace {
namespace mission=ardu_rover_mission;
constexpr double pi=3.14159265358979323846;
constexpr unsigned horizontal_estimator=EKF_ATTITUDE|EKF_VELOCITY_HORIZ|EKF_POS_HORIZ_ABS;
bool fresh(double stamp,double now,double limit)
{
  return std::isfinite(stamp)&&std::isfinite(now)&&stamp>0&&now>=stamp&&now-stamp<=limit;
}
double degrees(double angle)
{
  return std::fmod(std::fmod(angle,360.)+360.,360.);
}
bool position(const Vehicle& v)
{
  return std::isfinite(v.latitude)&&std::isfinite(v.longitude)&&std::abs(v.latitude)<=90&&std::abs(v.longitude)<=180;
}
ControlFrame frame(const mavlink_message_t& message)
{
  ControlFrame result;
  result.size=mavlink_msg_to_send_buffer(result.bytes.data(),&message);
  return result;
}
class ArduRoverBackend final:public VehicleControlBackend,public ArduRoverMissionPolicy {
public:
  explicit ArduRoverBackend(bool boat):m_boat(boat),m_mission(boat) {}
  std::string profile() const override { return m_boat?"ardurover_boat":"ardurover"; }
  HelmProfile helmProfile() const override
  { return {NavigationDimensions::Planar,false,false}; }
  bool identityCompatible(unsigned type,unsigned autopilot) const override
  {
    const unsigned expected=m_boat?MAV_TYPE_SURFACE_BOAT:MAV_TYPE_GROUND_ROVER;
    return type==expected&&autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA;
  }
  bool flightReady(const Vehicle& v,double now,bool motion) const override
  { return horizontalReady(v,now,.5)&&(!motion||(v.armed&&v.mode==rover::guided_mode)); }
  bool acquisitionAllowed(const Vehicle& v,double now) const override
  { return flightReady(v,now,false)&&v.armed&&activeModeAllowed(v,true); }
  bool activeModeAllowed(const Vehicle& v,bool acquisition_grace) const override
  { return v.mode==rover::guided_mode||(acquisition_grace&&preGuidedMode(v.mode)); }
  std::optional<CommandRequest> acquisitionCommand(const Vehicle& v,const std::string& id,double now) const override
  {
    if(!preGuidedMode(v.mode)||!acquisitionAllowed(v,now))return std::nullopt;
    CommandRequest request;request.id="acquire-"+id;request.command=MAV_CMD_DO_SET_MODE;
    request.params[0]=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;request.params[1]=rover::guided_mode;
    request.expires=now+5;return request;
  }
  bool commandAllowed(const CommandRequest& r,const Vehicle& v,double now) const override
  {
    if(!identityCompatible(v.type,v.autopilot)||v.mode==rover::manual_mode)return false;
    if(r.command==MAV_CMD_COMPONENT_ARM_DISARM) {
      if(r.params[1]!=0||(r.params[0]!=0&&r.params[0]!=1))return false;
      if(r.params[0]==1)
        return !v.armed&&flightReady(v,now,false)&&(v.mode==rover::guided_mode||v.mode==rover::hold_mode);
      return true;
    }
    if(r.command==MAV_CMD_DO_SET_MODE)
      return r.params[0]==MAV_MODE_FLAG_CUSTOM_MODE_ENABLED&&
        (r.params[1]==rover::guided_mode||r.params[1]==rover::hold_mode||
         (m_boat&&r.params[1]==rover::loiter_mode&&v.armed&&flightReady(v,now,false)));
    if(r.command==MAV_CMD_NAV_RETURN_TO_LAUNCH)
      return flightReady(v,now,false)&&v.armed&&v.home_valid&&
        std::isfinite(v.home_latitude)&&std::isfinite(v.home_longitude)&&
        std::abs(v.home_latitude)<=90&&std::abs(v.home_longitude)<=180;
    return false;
  }
  bool missionExecutionCommandAllowed(const CommandRequest& request,const Vehicle& v,double now) const override
  {
    if(!identityCompatible(v.type,v.autopilot) || !v.heartbeat_valid || !fresh(v.heartbeat_at,now,2))
      return false;
    if(mission::wireMode(request,mission::handbackMode(m_boat)))
      return ((v.armed && v.mode==mission::auto_mode) ||
              (!request.abort_id.empty() && mission::handbackModeAllowed(m_boat,v.mode))) &&
        (!m_boat || horizontalReady(v,now,.5));
    if(!request.abort_id.empty() || !ready(v,now) || !mission::handbackModeAllowed(m_boat,v.mode)) return false;
    if(mission::wireMode(request,mission::auto_mode)) return true;
    if(request.command==MAV_CMD_MISSION_START)
      return std::all_of(request.params.begin(),request.params.end(),[](float value){return value==0;});
    return request.command==MAV_CMD_DO_SET_MISSION_CURRENT && request.params[0]==1 &&
      std::all_of(request.params.begin()+1,request.params.end(),[](float value){return value==0;});
  }
  bool ready(const Vehicle& v,double now) const override
  {
    return horizontalReady(v,now,.5) && v.armed && v.home_valid && fresh(v.home_at,now,10) &&
      m_mission.configured(now) && ardu_rover_mission::separation(v.home_latitude,v.home_longitude,
        v.latitude,v.longitude)<=65;
  }
  bool settled(const Vehicle& v,double now,double after) const override
  {
    return horizontalReady(v,now,.5) && m_mission.settled(now,after);
  }
  void observe(const mavlink_message_t& message,const MessageTime& time,const Vehicle& v,double now) override
  {
    if(v.heartbeat_valid && fresh(v.heartbeat_at,now,2) && identityCompatible(v.type,v.autopilot))
      m_mission.observe(message,time,now);
  }
  void readConfiguration(Services& services) override { m_mission.readConfiguration(services); }
  void parameterSent(const ParameterRequest& request,double at,bool accepted) override
  { m_mission.parameterSent(request,at,accepted); }
  void reset(double now) override { m_mission.reset(now); }
  bool navigationInputAllowed(const NavigationTarget& target) const override
  {
    // These Rover profiles couple steering and course.
    if(target.dimensions!=NavigationDimensions::Planar||target.facing!=FacingPolicy::Course)return false;
    if(target.kind==NavigationKind::CourseSpeed)
      return std::isfinite(target.course)&&std::isfinite(target.speed)&&
        target.speed>=0&&target.speed<=rover::speedLimit(m_boat);
    return false;
  }
  bool helmFieldAllowed(unsigned index,double value) const override
  {
    return std::isfinite(value)&&(index==0||(index==1&&value>=0&&value<=rover::speedLimit(m_boat)));
  }
  bool helmVehicleReady(const Vehicle& v,double now,double timeout,bool shadow,bool acquisition) const override
  {
    return horizontalReady(v,now,timeout)&&v.armed&&(shadow||activeModeAllowed(v,acquisition));
  }
  std::optional<ControlFrame> navigationFrame(const NavigationTarget& target,const Vehicle& v,
      double now,uint64_t,mavlink_status_t& codec) const override
  {
    if(!navigationInputAllowed(target)||!std::isfinite(target.expires)||target.expires<=now||
       !flightReady(v,now,true)||!std::isfinite(now)||now<0)return std::nullopt;
    const double angle=degrees(target.course)*pi/180.;
    const float north=static_cast<float>(target.speed*std::cos(angle));
    const float east=static_cast<float>(target.speed*std::sin(angle));
    mavlink_message_t message{};
    // At true zero speed, preserve current steering direction with an explicit
    // yaw target instead of atan2(0,0)'s arbitrary north. No epsilon motion.
    const bool stopped=target.speed==0;
    const uint16_t mask=stopped?0x09E7:0x0DE7;
    mavlink_msg_set_position_target_global_int_pack_status(245,191,&codec,&message,
      static_cast<uint32_t>(std::fmod(now*1000.,4294967296.)),1,1,MAV_FRAME_GLOBAL,mask,
      0,0,0,north,east,0,0,0,0,stopped?static_cast<float>(v.heading*pi/180.):0.f,0);
    return frame(message);
  }
  std::optional<ControlFrame> stopFrame(const Vehicle& v,double now,uint64_t revision,mavlink_status_t& codec) const override
  {
    NavigationTarget target;target.dimensions=NavigationDimensions::Planar;
    target.speed=0;target.expires=now+.5;
    return navigationFrame(target,v,now,revision,codec);
  }
private:
  bool preGuidedMode(unsigned mode) const
  { return mode==rover::hold_mode||(m_boat&&mode==rover::loiter_mode); }
  bool horizontalReady(const Vehicle& v,double now,double timeout) const
  {
    // Runtime keeps raw flags even when its Copter-oriented vertical validity is
    // false. Rover requires attitude, horizontal velocity and absolute position.
    return identityCompatible(v.type,v.autopilot)&&v.heartbeat_valid&&
      fresh(v.heartbeat_at,now,2)&&v.position_valid&&v.estimator_horizontal_valid&&
      fresh(v.position_at,now,timeout)&&position(v)&&fresh(v.estimator_at,now,3)&&
      (v.estimator_flags&horizontal_estimator)==horizontal_estimator&&
      !(v.estimator_flags&EKF_UNINITIALIZED)&&std::isfinite(v.heading)&&v.heading>=0&&v.heading<360;
  }
  const bool m_boat;
  RoverMissionEvidence m_mission;
};
} // namespace
std::unique_ptr<VehicleControlBackend> makeArduRoverBackend(bool boat)
{ return std::make_unique<ArduRoverBackend>(boat); }
} // namespace mavmoos::runtime
