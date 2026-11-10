/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ArduPlaneBackend.cpp                             */
/*    DATE: November 6, 2026                                */
/************************************************************/
#include "PlaneComponents.h"
#include <algorithm>
#include <cmath>

namespace mavmoos::runtime::plane {
namespace {
constexpr double pi=3.14159265358979323846,radians=pi/180.,earth_radius_m=6371000.;
constexpr double minimum_height_m=35.,maximum_height_m=160.;
constexpr double minimum_airspeed_mps=10.,maximum_airspeed_mps=35.;
constexpr unsigned required_ekf=EKF_ATTITUDE|EKF_VELOCITY_HORIZ|EKF_VELOCITY_VERT|
  EKF_POS_HORIZ_ABS|EKF_POS_VERT_ABS;
constexpr unsigned invalid_ekf=EKF_CONST_POS_MODE|EKF_UNINITIALIZED|EKF_GPS_GLITCHING;

struct Point { double lat,lon; };

bool fresh(double at,double now,double limit)
{
  return std::isfinite(at)&&std::isfinite(now)&&at>0&&now>=at&&now-at<=limit;
}

bool valid(Point p)
{
  return std::isfinite(p.lat)&&std::isfinite(p.lon)&&std::abs(p.lat)<90&&std::abs(p.lon)<=180;
}

double separation(Point from,Point to)
{
  const double lat1=from.lat*radians,lat2=to.lat*radians;
  const double dlon=std::remainder(to.lon-from.lon,360.)*radians;
  const double sin_lat=std::sin((lat2-lat1)/2),sin_lon=std::sin(dlon/2);
  const double haversine=sin_lat*sin_lat+std::cos(lat1)*std::cos(lat2)*sin_lon*sin_lon;
  return 2*earth_radius_m*std::asin(std::sqrt(std::clamp(haversine,0.,1.)));
}

Point lookahead(Point from,double course)
{
  const double angle=lookahead_m/earth_radius_m;
  const double lat1=from.lat*radians,bearing=course*radians;
  const double lat2=std::asin(std::clamp(std::sin(lat1)*std::cos(angle)+
    std::cos(lat1)*std::sin(angle)*std::cos(bearing),-1.,1.));
  const double delta_lon=std::atan2(std::sin(bearing)*std::sin(angle)*std::cos(lat1),
    std::cos(angle)-std::sin(lat1)*std::sin(lat2));
  return {lat2/radians,std::remainder(from.lon+delta_lon/radians,360.)};
}

bool atOrAfter(const MessageTime& candidate,const MessageTime& previous)
{
  // Frames in one datagram can share both acquisition stamps. Process their
  // decoder order without letting any of them renew the acquisition age.
  return candidate.effective_at>=previous.effective_at&&candidate.read_at>=previous.read_at;
}
} // namespace

CommandRequest modeRequest(const std::string& id,unsigned mode,double expires)
{
  CommandRequest request;
  request.id=id;request.command=MAV_CMD_DO_SET_MODE;request.expires=expires;
  request.params[0]=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  request.params[1]=static_cast<float>(mode);
  return request;
}

bool Backend::identityCompatible(unsigned type,unsigned autopilot) const
{
  return type==(m_quadplane?MAV_TYPE_VTOL_FIXEDROTOR:MAV_TYPE_FIXED_WING)&&
    autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA;
}

void Backend::observe(const mavlink_message_t& message,const MessageTime& time,const Vehicle& v,double now)
{
  if(message.msgid!=MAVLINK_MSG_ID_AIRSPEED&&message.msgid!=MAVLINK_MSG_ID_EXTENDED_SYS_STATE&&
     message.msgid!=MAVLINK_MSG_ID_SYS_STATUS)return;
  if(message.sysid!=1||message.compid!=1||!identityCompatible(v.type,v.autopilot)||
     !v.heartbeat_valid||!fresh(v.heartbeat_at,now,2)||
     !std::isfinite(time.effective_at)||!std::isfinite(time.read_at)||
     time.effective_at<=m_observation_floor||time.effective_at<v.heartbeat_at||
     time.effective_at>time.read_at||time.read_at>now||
     (time.age_basis!="QUEUE_EMPTY_FLOOR"&&
      (time.age_basis!="READ_ACQUISITION"||time.effective_at!=time.read_at)))return;

  if(message.msgid==MAVLINK_MSG_ID_SYS_STATUS) {
    if(time.effective_at<m_prearm_at)return;
    mavlink_sys_status_t status{};
    mavlink_msg_sys_status_decode(&message,&status);
    const auto enabled=status.onboard_control_sensors_present&status.onboard_control_sensors_enabled;
    m_prearm_ok=(enabled&status.onboard_control_sensors_health&MAV_SYS_STATUS_PREARM_CHECK)!=0;
    m_prearm_at=time.effective_at;
    return;
  }

  if(message.msgid==MAVLINK_MSG_ID_EXTENDED_SYS_STATE) {
    if(!atOrAfter(time,m_airframe_time))return;
    m_airframe_time=time;
    mavlink_extended_sys_state_t state{};
    mavlink_msg_extended_sys_state_decode(&message,&state);
    const bool shaped=(message.magic==MAVLINK_STX||message.magic==MAVLINK_STX_MAVLINK1)&&
      message.len==MAVLINK_MSG_ID_EXTENDED_SYS_STATE_LEN;
    const auto forward=m_quadplane?MAV_VTOL_STATE_FW:MAV_VTOL_STATE_UNDEFINED;
    m_forward_airborne=shaped&&state.vtol_state==forward&&state.landed_state==MAV_LANDED_STATE_IN_AIR;
    m_conventional_ground=shaped&&!m_quadplane&&state.vtol_state==MAV_VTOL_STATE_UNDEFINED&&
      state.landed_state==MAV_LANDED_STATE_ON_GROUND;
    return;
  }

  if(message.magic!=MAVLINK_STX||message.len==0||message.len>MAVLINK_MSG_ID_AIRSPEED_LEN)return;
  mavlink_airspeed_t airspeed{};
  mavlink_msg_airspeed_decode(&message,&airspeed);
  if(airspeed.id!=0||!atOrAfter(time,m_airspeed_time))return;
  m_airspeed_time=time;
  m_airspeed.reset();
  // USING is the last byte. Zero-truncated or otherwise unqualified sensor
  // reports can retire readiness, but cannot prove it.
  const double minimum=m_quadplane?18.:minimum_airspeed_mps;
  if(message.len==MAVLINK_MSG_ID_AIRSPEED_LEN&&airspeed.flags==AIRSPEED_SENSOR_USING&&
     std::isfinite(airspeed.airspeed)&&airspeed.airspeed>=minimum&&
     airspeed.airspeed<=maximum_airspeed_mps)
    m_airspeed=airspeed.airspeed;
}

void Backend::reset(double now)
{
  m_observation_floor=std::max(m_observation_floor,now);
  m_airspeed.reset();m_airspeed_time={};
  m_prearm_ok=false;m_prearm_at=0;
  m_forward_airborne=false;m_conventional_ground=false;m_airframe_time={};
}

bool Backend::groundReady(const Vehicle& v,double now) const
{ return m_conventional_ground&&m_airframe_time.effective_at==v.landed_at&&groundState(v,now); }

bool Backend::prearmReady(double now) const
{ return m_prearm_ok&&fresh(m_prearm_at,now,2); }

const char* Backend::readiness(const Vehicle& v,double now,double position_timeout) const
{
  if(!identityCompatible(v.type,v.autopilot)||!v.heartbeat_valid||!fresh(v.heartbeat_at,now,2))
    return "fixed_wing_heartbeat_required";
  if(!v.armed)return "armed_required";
  if(!v.landed_valid||!v.in_air||v.landed_state!=MAV_LANDED_STATE_IN_AIR||!fresh(v.landed_at,now,2)||
     !m_forward_airborne||m_airframe_time.effective_at!=v.landed_at)
    return m_quadplane?"quadplane_forward_state_required":"conventional_airborne_state_required";
  if(!v.estimator_valid||!v.estimator_horizontal_valid||!fresh(v.estimator_at,now,3)||
     (v.estimator_flags&required_ekf)!=required_ekf||(v.estimator_flags&invalid_ekf))
    return "horizontal_and_vertical_estimator_required";
  const Point current{v.latitude,v.longitude},home{v.home_latitude,v.home_longitude};
  if(!v.position_valid||!fresh(v.position_at,now,std::min(position_timeout,.5))||
     !valid(current)||!std::isfinite(v.relative_height)||!std::isfinite(v.msl_height))
    return "fresh_global_position_required";
  if(!v.home_valid||!fresh(v.home_at,now,3)||!valid(home)||!std::isfinite(v.home_msl))
    return "fresh_home_required";
  if(v.relative_height<minimum_height_m||v.relative_height>maximum_height_m||
     std::abs(v.msl_height-v.home_msl-v.relative_height)>5)
    return "actual_height_outside_envelope";
  if(separation(home,current)>home_radius_m-turn_reserve_m)
    return "actual_position_outside_envelope";
  if(!m_airspeed||!fresh(m_airspeed_time.effective_at,now,airspeed_age_limit))
    return "active_airspeed_required";
  return nullptr;
}

bool Backend::flightReady(const Vehicle& v,double now,bool motion) const
{
  return (!motion||v.mode==guided_mode)&&!readiness(v,now);
}

bool Backend::acquisitionAllowed(const Vehicle& v,double now) const
{
  return activeModeAllowed(v,true)&&flightReady(v,now,false);
}

bool Backend::activeModeAllowed(const Vehicle& v,bool acquisition_grace) const
{
  return v.mode==guided_mode||(acquisition_grace&&v.mode==loiter_mode);
}

std::optional<CommandRequest> Backend::acquisitionCommand(const Vehicle& v,const std::string& id,double now) const
{
  if(v.mode!=loiter_mode||!acquisitionAllowed(v,now))return std::nullopt;
  return modeRequest("acquire-"+id,guided_mode,now+5);
}

bool Backend::commandAllowed(const CommandRequest& r,const Vehicle& v,double now) const
{
  if(!r.execution_id.empty()||!r.abort_id.empty())return false;
  if(armCommand(r))return v.mode==manual_mode&&groundReady(v,now)&&v.armed!=(r.params[0]==1)&&
    (r.params[0]==0||prearmReady(now));
  if(manualCommand(r))return !v.armed&&groundReady(v,now)&&(v.mode==rtl_mode||v.mode==loiter_mode);
  if(speedCommand(r))return flightReady(v,now,true);
  if(r.command!=MAV_CMD_DO_SET_MODE||r.params[0]!=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED||
     std::any_of(r.params.begin()+2,r.params.end(),[](float param){return param!=0;}))return false;
  // A host-bound Loiter handback may relinquish Guided after motion evidence
  // expires. Ordinary operator mode requests still require full readiness.
  if(r.params[1]==loiter_mode&&v.mode==guided_mode)
    return v.armed&&identityCompatible(v.type,v.autopilot)&&
      v.heartbeat_valid&&fresh(v.heartbeat_at,now,2);
  return flightReady(v,now,false)&&
    ((r.params[1]==guided_mode&&v.mode==loiter_mode)||
     (r.params[1]==rtl_mode&&(v.mode==guided_mode||v.mode==loiter_mode))||
     (r.params[1]==loiter_mode&&v.mode==rtl_mode));
}

bool Backend::missionExecutionCommandAllowed(const CommandRequest& r,const Vehicle& v,double now) const
{
  if(r.execution_id.empty()||!r.abort_id.empty())return false;
  if(r.command==MAV_CMD_DO_SET_MISSION_CURRENT||r.command==MAV_CMD_MISSION_START)
    return v.mode==manual_mode?(v.armed&&groundReady(v,now)):
      ((v.mode==loiter_mode||v.mode==guided_mode)&&flightReady(v,now,false));
  if(r.command==MAV_CMD_DO_SET_MODE&&r.params[1]==autoland_mode)
    return flightReady(v,now,false)&&(v.mode==loiter_mode||v.mode==guided_mode);
  if(v.mode!=auto_mode&&v.mode!=autoland_mode)return false;
  if(armCommand(r))return v.armed&&r.params[0]==0&&groundReady(v,now);
  return r.command==MAV_CMD_DO_SET_MODE&&
    ((r.params[1]==loiter_mode&&flightReady(v,now,false))||
     (r.params[1]==manual_mode&&!v.armed&&groundReady(v,now)));
}

bool Backend::navigationInputAllowed(const NavigationTarget& target) const
{
  if(target.dimensions!=NavigationDimensions::Spatial||target.facing!=FacingPolicy::Course||
     !helmFieldAllowed(2,target.height))return false;
  if(target.kind==NavigationKind::GlobalPosition)
    return (target.latitude!=0||target.longitude!=0)&&valid({target.latitude,target.longitude});
  return target.kind==NavigationKind::CourseSpeed&&helmFieldAllowed(0,target.course)&&helmFieldAllowed(1,target.speed);
}

bool Backend::helmFieldAllowed(unsigned index,double value) const
{
  if(!std::isfinite(value))return false;
  if(index==0)return value>=0&&value<=360;
  if(index==1)return std::abs(value-cruise_ground_intent)<=.05;
  return index==2&&value>=minimum_target_height_m&&value<=maximum_target_height_m;
}

bool Backend::helmVehicleReady(const Vehicle& v,double now,double timeout,bool shadow,bool acquisition) const
{
  return (shadow||activeModeAllowed(v,acquisition))&&!readiness(v,now,timeout);
}

bool Backend::directTargetAllowed(const NavigationTarget& target,const Vehicle& v) const
{
  return target.kind==NavigationKind::GlobalPosition&&navigationInputAllowed(target)&&
    (target.latitude!=0||target.longitude!=0)&&v.home_valid&&
    separation({v.home_latitude,v.home_longitude},{target.latitude,target.longitude})<=home_radius_m-turn_reserve_m;
}

std::optional<NavigationTarget> Backend::directCourseTarget(const Vehicle& v,double course) const
{
  if(!std::isfinite(course)||course<0||course>=360)return std::nullopt;
  const Point point=lookahead({v.latitude,v.longitude},course);
  NavigationTarget target;
  target.kind=NavigationKind::GlobalPosition;
  target.latitude=point.lat;target.longitude=point.lon;target.height=v.relative_height;
  if(!directTargetAllowed(target,v))return std::nullopt;
  return target;
}

std::optional<ControlFrame> Backend::navigationFrame(const NavigationTarget& target,const Vehicle& v,
  double now,uint64_t,mavlink_status_t& codec) const
{
  if(!navigationInputAllowed(target)||!std::isfinite(target.expires)||target.expires<=now||
     !flightReady(v,now,true))return std::nullopt;
  if(target.kind==NavigationKind::GlobalPosition&&!directTargetAllowed(target,v))return std::nullopt;
  const Point point=target.kind==NavigationKind::GlobalPosition?Point{target.latitude,target.longitude}:
    lookahead({v.latitude,v.longitude},target.course);
  if(!valid(point))return std::nullopt;

  mavlink_command_int_t command{};
  command.target_system=1;command.target_component=1;command.command=MAV_CMD_DO_REPOSITION;
  command.frame=MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
  command.param1=-1; // Pinned Plane uses its own airspeed policy.
  command.x=static_cast<int32_t>(std::llround(point.lat*1e7));
  command.y=static_cast<int32_t>(std::llround(point.lon*1e7));
  command.z=static_cast<float>(target.height);
  // Plane treats zero latitude/longitude as an unspecified current location.
  if((command.x==0&&command.y==0)||separation({v.home_latitude,v.home_longitude},
     {command.x/1e7,command.y/1e7})>home_radius_m-turn_reserve_m)return std::nullopt;
  mavlink_message_t message{};
  mavlink_msg_command_int_encode_status(245,191,&codec,&message,&command);
  ControlFrame result;
  result.size=mavlink_msg_to_send_buffer(result.bytes.data(),&message);
  return result;
}
} // namespace mavmoos::runtime::plane
