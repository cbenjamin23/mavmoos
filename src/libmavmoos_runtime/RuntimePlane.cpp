#include "Runtime.h"
#include "PlaneOperationPolicy.h"
#include "QuadPlanePolicy.h"
#include <algorithm>
#include <cmath>
#include <ostream>
#include <string_view>

namespace mavmoos::runtime {
namespace {
constexpr unsigned loiter=12, guided=15, rtl=11;
constexpr double handback_window=3., target_readback_window=2., point_period=1.;
struct ProfileParameter { const char* name; uint8_t type; float value; };
constexpr ProfileParameter plane_parameters[]={
  {"Q_ENABLE",MAV_PARAM_TYPE_INT8,0},
  {"SYSID_MYGCS",MAV_PARAM_TYPE_INT16,245},
  {"FS_GCS_ENABL",MAV_PARAM_TYPE_INT8,1},
  {"FS_LONG_ACTN",MAV_PARAM_TYPE_INT8,1},
  {"FS_LONG_TIMEOUT",MAV_PARAM_TYPE_REAL32,5},
  {"RTL_AUTOLAND",MAV_PARAM_TYPE_INT8,0},
  {"RTL_ALTITUDE",MAV_PARAM_TYPE_REAL32,65},
  {"RTL_RADIUS",MAV_PARAM_TYPE_INT16,0},
  {"WP_LOITER_RAD",MAV_PARAM_TYPE_INT16,80},
  {"AIRSPEED_CRUISE",MAV_PARAM_TYPE_REAL32,22},
  {"AIRSPEED_MIN",MAV_PARAM_TYPE_INT16,10},
  {"AIRSPEED_MAX",MAV_PARAM_TYPE_INT16,30},
  {"ARSPD_USE",MAV_PARAM_TYPE_INT8,1},
  {"ARSPD_TYPE",MAV_PARAM_TYPE_INT8,100},
  {"ARSPD_PRIMARY",MAV_PARAM_TYPE_INT8,0},
  {"ARSPD2_TYPE",MAV_PARAM_TYPE_INT8,0},
  {"MIS_RESTART",MAV_PARAM_TYPE_INT8,0}
};
constexpr std::size_t legacy_parameter_count=sizeof(plane_parameters)/sizeof(plane_parameters[0]);
constexpr ProfileParameter modern_landing_parameters[]={
  {"AUTOLAND_WP_ALT",MAV_PARAM_TYPE_INT16,plane::native_approach_height_m},
  {"AUTOLAND_WP_DIST",MAV_PARAM_TYPE_INT16,plane::native_approach_distance_m},
  {"AUTOLAND_DIR_OFF",MAV_PARAM_TYPE_INT16,0},
  {"AUTOLAND_OPTIONS",MAV_PARAM_TYPE_INT8,1},
  {"AUTOLAND_CLIMB",MAV_PARAM_TYPE_INT16,0},
  {"MAV_GCS_SYSID_HI",MAV_PARAM_TYPE_INT16,0},
  {"ARMING_SKIPCHK",MAV_PARAM_TYPE_INT32,0}
};
constexpr ProfileParameter quadplane_parameters[]={
  {"Q_ENABLE",MAV_PARAM_TYPE_INT8,1},
  {"Q_MAV_TYPE",MAV_PARAM_TYPE_INT8,MAV_TYPE_VTOL_FIXEDROTOR},
  {"Q_GUIDED_MODE",MAV_PARAM_TYPE_INT8,0},
  {"Q_FRAME_CLASS",MAV_PARAM_TYPE_INT8,1},
  {"Q_FRAME_TYPE",MAV_PARAM_TYPE_INT8,1},
  {"Q_TILT_ENABLE",MAV_PARAM_TYPE_INT8,0},
  {"Q_TAILSIT_ENABLE",MAV_PARAM_TYPE_INT8,0},
  {"Q_RTL_MODE",MAV_PARAM_TYPE_INT8,0},
  {"Q_ASSIST_SPEED",MAV_PARAM_TYPE_REAL32,18},
  {"SYSID_MYGCS",MAV_PARAM_TYPE_INT16,245},
  {"FS_GCS_ENABL",MAV_PARAM_TYPE_INT8,1},
  {"FS_LONG_ACTN",MAV_PARAM_TYPE_INT8,1},
  {"FS_LONG_TIMEOUT",MAV_PARAM_TYPE_REAL32,5},
  {"RTL_AUTOLAND",MAV_PARAM_TYPE_INT8,0},
  {"RTL_ALTITUDE",MAV_PARAM_TYPE_REAL32,65},
  {"RTL_RADIUS",MAV_PARAM_TYPE_INT16,0},
  {"WP_LOITER_RAD",MAV_PARAM_TYPE_INT16,80},
  {"AIRSPEED_CRUISE",MAV_PARAM_TYPE_REAL32,22},
  {"AIRSPEED_MIN",MAV_PARAM_TYPE_INT16,13},
  {"AIRSPEED_MAX",MAV_PARAM_TYPE_INT16,35},
  {"ARSPD_USE",MAV_PARAM_TYPE_INT8,1},
  {"ARSPD_TYPE",MAV_PARAM_TYPE_INT8,100},
  {"ARSPD_PRIMARY",MAV_PARAM_TYPE_INT8,0},
  {"ARSPD2_TYPE",MAV_PARAM_TYPE_INT8,0}
};
constexpr ProfileParameter quadplane_hover_parameters[]={
  {"SCR_ENABLE",MAV_PARAM_TYPE_INT8,1},
  {"FENCE_ENABLE",MAV_PARAM_TYPE_INT8,0},
  {"FENCE_AUTOENABLE",MAV_PARAM_TYPE_INT8,0},
};

ProfileParameter parameterAt(std::size_t index,bool modern,bool quadplane,bool hover)
{
  if(quadplane) {
    if(index>=std::size(quadplane_parameters))return quadplane_hover_parameters[index-std::size(quadplane_parameters)];
    auto parameter=quadplane_parameters[index];
    if(hover&&std::string_view(parameter.name)=="Q_GUIDED_MODE")parameter.value=1;
    if(hover&&std::string_view(parameter.name)=="Q_RTL_MODE")parameter.value=3;
    return parameter;
  }
  if(modern&&index==1)return {"MAV_GCS_SYSID",MAV_PARAM_TYPE_INT16,245};
  if(index>=legacy_parameter_count)return modern_landing_parameters[index-legacy_parameter_count];
  return plane_parameters[index];
}

bool modeCommand(const CommandRequest& r,unsigned destination)
{
  return r.command==MAV_CMD_DO_SET_MODE && r.params[0]==MAV_MODE_FLAG_CUSTOM_MODE_ENABLED &&
    r.params[1]==destination && std::all_of(r.params.begin()+2,r.params.end(),[](float p){return p==0;});
}

std::optional<mavlink_command_int_t> reposition(const ControlFrame& frame)
{
  mavlink_message_t buffer{},message{};
  mavlink_status_t status{},result{};
  if(frame.size==0 || frame.size>frame.bytes.size()) return std::nullopt;
  for(std::size_t i=0;i<frame.size;++i) {
    if(mavlink_frame_char_buffer(&buffer,&status,frame.bytes[i],&message,&result)!=MAVLINK_FRAMING_OK) continue;
    if(i+1!=frame.size || message.sysid!=245 || message.compid!=191 ||
       message.msgid!=MAVLINK_MSG_ID_COMMAND_INT) return std::nullopt;
    mavlink_command_int_t p{};
    mavlink_msg_command_int_decode(&message,&p);
    if(p.command==MAV_CMD_DO_REPOSITION && p.target_system==1 && p.target_component==1 &&
       p.frame==MAV_FRAME_GLOBAL_RELATIVE_ALT_INT && !p.current && !p.autocontinue &&
       p.param1==-1 && p.param2==0 && p.param3==0 && p.param4==0 && std::isfinite(p.z))
      return p;
    return std::nullopt;
  }
  return std::nullopt;
}

double pointSeparation(int32_t lat,int32_t lon,int32_t old_lat,int32_t old_lon)
{
  constexpr double radians=3.14159265358979323846/180.;
  constexpr double metres_per_degree=6378137.*radians;
  const double north=(double(lat)-old_lat)*1e-7;
  const double east=std::remainder((double(lon)-old_lon)*1e-7,360.)*std::cos(lat*1e-7*radians);
  return std::hypot(north,east)*metres_per_degree;
}
} // namespace

bool Runtime::planeProfile() const
{
  return m_expected_type==MAV_TYPE_FIXED_WING && m_backend && m_backend->profile()=="arduplane";
}

bool Runtime::quadPlaneHelmProfile() const
{ return quadPlaneProfile()&&m_host.hasCapability("control.navigation.helm"); }

bool Runtime::quadPlaneHoverProfile() const
{ return quadPlaneHelmProfile()&&m_host.hasCapability(quadplane::hover_capability); }

unsigned Runtime::planeHoldMode() const
{ return quadPlaneHoverProfile()?quadplane::qloiter:loiter; }

unsigned Runtime::quadPlaneHelmPhase() const
{ return quadPlaneHoverProfile()?MAV_VTOL_STATE_MC:MAV_VTOL_STATE_FW; }

bool Runtime::quadPlaneHoverSettling() const
{
  if(!quadPlaneHoverProfile()||!m_nav_permission||m_now>=m_acquire_until||m_plane.acquire_sent_at<0||
     autoRecoveryBlocksEffects()||(m_vehicle.mode!=guided&&m_vehicle.mode!=quadplane::qloiter))return false;
  const auto phase=quadPlanePhase();
  return (phase==MAV_VTOL_STATE_MC||phase==MAV_VTOL_STATE_TRANSITION_TO_MC||
          phase==MAV_VTOL_STATE_TRANSITION_TO_FW)&&
    quadplane::hoverReady(m_vehicle,m_now,.5);
}

bool Runtime::planeGuidanceProfile() const
{ return planeProfile()||quadPlaneHelmProfile(); }

bool Runtime::planeNativeAutoland() const
{ return planeProfile()&&m_host.hasCapability("control.land.native_autoland"); }

bool Runtime::planeProfileReady() const
{
  const bool needs_version=planeNativeAutoland()||quadPlaneHelmProfile();
  return !m_plane.parameter_mismatch&&(!needs_version||m_plane.version_verified)&&
    m_plane.parameter_index==planeParameterCount();
}

std::size_t Runtime::planeParameterCount() const
{
  if(quadPlaneHelmProfile())
    return std::size(quadplane_parameters)+(quadPlaneHoverProfile()?std::size(quadplane_hover_parameters):0);
  if(planeNativeAutoland())return legacy_parameter_count+std::size(modern_landing_parameters);
  return legacy_parameter_count-(m_host.hasCapability("control.mission.start")?0:1);
}

bool Runtime::planeHeartbeatEligible() const
{
  if(!planeProfileReady() || !m_vehicle.heartbeat_valid || !fresh(m_vehicle.heartbeat_at,2)) return false;
  // A new process/connection cannot sustain a retained Guided target. Establish
  // the watchdog from a verified native holding mode before accepting a grant.
  const bool quadplane=quadPlaneHelmProfile();
  const bool hover=quadPlaneHoverProfile();
  if(!m_plane.heartbeat_started&&quadplane)
    return m_vehicle.mode==planeHoldMode()&&flightReady(false)&&(!hover||quadPlaneHoverAdapterReady(false));
  if(!m_plane.heartbeat_started)
    return (plane::groundState(m_vehicle,m_now)&&(m_vehicle.mode==plane::manual_mode||
      (!m_vehicle.armed&&(m_vehicle.mode==loiter||m_vehicle.mode==rtl)))) ||
      ((m_vehicle.mode==loiter || m_vehicle.mode==rtl) && flightReady(false));
  if(m_execution&&!quadplane)return !m_execution->quarantined&&m_now<m_execution->expires;
  // The pinned FC watchdog remains enabled in external pilot modes.
  if(m_vehicle.mode!=guided) return true;
  if(m_plane.transition && m_now<m_plane.transition->until)
    return !quadplane||quadPlanePhase()==quadPlaneHelmPhase();
  if(hover&&quadPlaneHoverSettling())return true;
  if(!flightReady(true)) return false;
  if(m_nav_permission)
    return m_now<m_plane.intent_until && (!m_plane.awaiting || m_now<m_plane.awaiting->verify_until);
  return m_resume_until>m_now && planeResumeAllowed();
}

bool Runtime::planeCanReadProfile() const
{
  return (!quadPlaneHelmProfile()||!autoRecoveryBlocksEffects()) &&
    m_plane.parameter_index<planeParameterCount() && !m_plane.parameter_mismatch &&
    m_plane.parameter_attempts<3 && m_now>=m_plane.parameter_retry_at &&
    m_vehicle.heartbeat_valid && fresh(m_vehicle.heartbeat_at,2);
}

void Runtime::flushPlaneProfileRead()
{
  const auto index=m_plane.parameter_index;
  const auto connection=m_connection_revision;
  mavlink_message_t msg{};
  const bool quadplane=quadPlaneHelmProfile();
  if((planeNativeAutoland()||quadplane)&&!m_plane.version_verified) {
    mavlink_msg_command_long_pack_status(245,191,&m_codec,&msg,1,1,MAV_CMD_REQUEST_MESSAGE,0,
      MAVLINK_MSG_ID_AUTOPILOT_VERSION,0,0,0,0,0,0);
  } else {
    mavlink_param_request_read_t request{};
    request.target_system=1; request.target_component=1; request.param_index=-1;
    const std::string_view name=parameterAt(index,planeNativeAutoland(),quadplane,quadPlaneHoverProfile()).name;
    std::copy(name.begin(),name.end(),request.param_id);
    mavlink_msg_param_request_read_encode_status(245,191,&m_codec,&msg,&request);
  }
  const bool accepted=send(msg,m_now+.25,[this,index]{
    return m_plane.parameter_index==index && planeCanReadProfile();
  });
  if(connection!=m_connection_revision) return;
  ++m_plane.parameter_attempts;
  if(accepted) m_plane.parameter_sent_at=m_now;
  m_plane.parameter_retry_at=m_now+.8;
}

void Runtime::observePlaneMessage(const mavlink_message_t& msg,double at,double read_at)
{
  if(!planeGuidanceProfile() || m_shadow || !m_simulation || msg.sysid!=1 || msg.compid!=1) return;
  const bool modern=planeNativeAutoland();
  const bool quadplane=quadPlaneHelmProfile();
  const bool hover=quadPlaneHoverProfile();
  const bool needs_version=modern||quadplane;
  if(msg.msgid==MAVLINK_MSG_ID_AUTOPILOT_VERSION&&needs_version&&!m_plane.version_verified&&
     m_plane.parameter_sent_at>=0&&read_at>m_plane.parameter_sent_at&&fresh(at,1.5)) {
    mavlink_autopilot_version_t version{};
    mavlink_msg_autopilot_version_decode(&msg,&version);
    const std::string_view pin=quadplane?"3fc7011a":"fb773567";
    m_plane.version_verified=version.flight_sw_version==(quadplane?0x040603ffu:0x040702ffu)&&
      std::equal(pin.begin(),pin.end(),version.flight_custom_version);
    m_plane.parameter_mismatch=!m_plane.version_verified;
    m_plane.parameter_attempts=0;
    m_plane.parameter_sent_at=-1;
    m_plane.parameter_retry_at=0;
  } else if(msg.msgid==MAVLINK_MSG_ID_PARAM_VALUE && (!needs_version||m_plane.version_verified) && fresh(at,1.5)) {
    mavlink_param_value_t p{};
    mavlink_msg_param_value_decode(&msg,&p);
    const auto name=std::string_view(p.param_id,std::find(p.param_id,p.param_id+16,'\0')-p.param_id);
    for(std::size_t i=0;i<planeParameterCount();++i) {
      const auto expected=parameterAt(i,modern,quadplane,hover);
      if(name!=expected.name) continue;
      const bool matches=p.param_type==expected.type && std::isfinite(p.param_value) &&
        std::abs(p.param_value-expected.value)<=.001f;
      if(i<m_plane.parameter_index && !matches) m_plane.parameter_mismatch=true;
      if(i==m_plane.parameter_index && m_plane.parameter_sent_at>=0 && read_at>m_plane.parameter_sent_at &&
         p.param_index==UINT16_MAX && !m_plane.parameter_mismatch) {
        if(matches) {
          ++m_plane.parameter_index;
          m_plane.parameter_attempts=0;
          m_plane.parameter_sent_at=-1;
          m_plane.parameter_retry_at=0;
        } else m_plane.parameter_mismatch=true;
      }
      break;
    }
    refreshPlaneAuthority();
  } else if(hover&&msg.msgid==MAVLINK_MSG_ID_TUNNEL) {
    observeQuadPlaneHoverAdapter(msg,at);
  } else if(!quadplane&&msg.msgid==MAVLINK_MSG_ID_MISSION_CURRENT&&at>m_plane.mission_cursor_at&&fresh(at,2)) {
    mavlink_mission_current_t cursor{};
    mavlink_msg_mission_current_decode(&msg,&cursor);
    m_plane.mission_seq=cursor.seq;
    m_plane.mission_cursor_at=at;
  } else if(msg.msgid==MAVLINK_MSG_ID_HEARTBEAT) {
    if(modern) {
      const bool manual_ground=m_vehicle.mode==plane::manual_mode&&plane::groundState(m_vehicle,m_now);
      if(!m_vehicle.armed) {
        m_plane.launch_heading.reset();
        m_plane.ground_disarmed_seen=manual_ground;
      } else if(m_plane.ground_disarmed_seen) {
        m_plane.ground_disarmed_seen=false;
        if(manual_ground&&std::isfinite(m_vehicle.heading))m_plane.launch_heading=m_vehicle.heading;
      }
    }
    refreshPlaneAuthority();
  } else if(msg.msgid==MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT && fresh(at,.75) && m_vehicle.mode==guided) {
    mavlink_position_target_global_int_t p{};
    mavlink_msg_position_target_global_int_decode(&msg,&p);
    if(p.coordinate_frame!=MAV_FRAME_GLOBAL || p.type_mask!=0xFDF8 || !std::isfinite(p.alt)) return;
    const auto matches=[&p](const PlaneState::Target& point) {
      return p.lat_int==point.lat && p.lon_int==point.lon && std::abs(p.alt-point.msl_height)<=.3;
    };
    if(m_plane.awaiting && m_nav_permission && at>m_plane.awaiting->sent_at &&
       m_now<m_plane.awaiting->verify_until && matches(*m_plane.awaiting)) {
      m_plane.verified=m_plane.awaiting;
      m_plane.verified->verified_at=at;
      m_plane.awaiting.reset();
      ++m_plane.verified_count;
    } else if(!m_plane.awaiting && m_plane.verified && at>m_plane.verified->verified_at) {
      if(matches(*m_plane.verified)) m_plane.verified->verified_at=at;
      else revokeNavigation("plane_target_changed_externally");
    }
  }
}

bool Runtime::planeExplicitModeAllowed(const CommandRequest& r) const
{
  if(!m_plane.heartbeat_started||!planeProfileReady()||m_plane.transition||!flightReady(false))return false;
  if(m_vehicle.mode==guided)
    return m_plane.guided_confirmed&&(m_nav_permission||m_resume_until>m_now)&&
      (modeCommand(r,loiter)||modeCommand(r,rtl));
  return !m_nav_permission&&m_resume_until<=m_now&&
    ((m_vehicle.mode==loiter&&modeCommand(r,rtl))||(m_vehicle.mode==rtl&&modeCommand(r,loiter)));
}

bool Runtime::planeArmAllowed(const CommandRequest& r) const
{
  if(!planeProfileReady()||!m_plane.heartbeat_started||m_plane.transition||m_nav_permission||
     m_resume_until>m_now||!plane::groundState(m_vehicle,m_now))return false;
  if(plane::manualCommand(r))
    return !m_vehicle.armed&&(m_vehicle.mode==loiter||m_vehicle.mode==rtl);
  return plane::armCommand(r)&&m_vehicle.mode==plane::manual_mode;
}

bool Runtime::planeSpeedAllowed(const CommandRequest& r) const
{
  return plane::speedCommand(r)&&planeProfileReady()&&m_plane.heartbeat_started&&m_plane.guided_confirmed&&
    !m_plane.transition&&m_nav_permission&&m_now<m_plane.intent_until&&flightReady(true);
}

bool Runtime::planeMissionEntry(const std::vector<MissionItem>& items) const
{
  if(!m_auto_recovery||!planeProfileReady()||!m_plane.heartbeat_started||m_plane.transition||
     !m_vehicle.armed||!fresh(m_vehicle.home_at,3)||!plane::lifecyclePlan(items,m_vehicle))return false;
  if(planeNativeAutoland()&&(!m_plane.launch_heading||
     !plane::nativeLandingPlan(items,m_vehicle,*m_plane.launch_heading)))return false;
  if(m_vehicle.mode==plane::manual_mode)
    return !m_nav_permission&&m_resume_until<=m_now&&plane::groundState(m_vehicle,m_now);
  return flightReady(false)&&
    (m_vehicle.mode==loiter||(m_vehicle.mode==guided&&m_nav_permission&&m_plane.guided_confirmed));
}

bool Runtime::planeMissionCommandEligible(const CommandRequest& r) const
{
  if(!m_execution||!r.abort_id.empty()||!planeProfileReady()||!m_plane.heartbeat_started)return false;
  const auto& v=m_vehicle;
  const bool ground=plane::groundState(v,m_now);
  const bool landing=m_execution->home.in_air;
  const bool waiting=m_plane.mission_started_at<0&&v.mode==m_execution->home.mode;
  const bool preflight=landing?flightReady(false):ground;
  const bool autoland=landing&&planeNativeAutoland();
  const auto start_item=landing?plane::approach_entry_item:plane::takeoff_item;
  const auto tailZero=[&r](std::size_t from) {
    return std::all_of(r.params.begin()+from,r.params.end(),[](float value){return value==0;});
  };
  if(r.command==MAV_CMD_DO_SET_MISSION_CURRENT)
    return !autoland&&waiting&&preflight&&v.armed&&m_plane.mission_selected_at<0&&
      r.params[0]==start_item&&tailZero(1);
  if(r.command==MAV_CMD_MISSION_START)
    return !autoland&&waiting&&preflight&&v.armed&&tailZero(0)&&m_plane.mission_selected_at>=0&&
      m_plane.mission_cursor_at>m_plane.mission_selected_at&&fresh(m_plane.mission_cursor_at,2)&&
      m_plane.mission_seq==start_item;
  if(modeCommand(r,plane::autoland_mode))return autoland&&waiting&&flightReady(false);
  const auto execution_mode=autoland?plane::autoland_mode:plane::auto_mode;
  if(m_plane.mission_started_at<0||!m_execution->auto_seen||v.mode!=execution_mode||
     m_plane.mission_return_sent_at>=0)return false;
  if(!landing)
    return modeCommand(r,loiter)&&flightReady(false)&&fresh(m_plane.mission_cursor_at,2)&&
      m_plane.mission_cursor_at>m_plane.mission_started_at&&m_plane.mission_seq==plane::takeoff_hold_item&&
      v.relative_height>=m_stored_mission.items[plane::takeoff_item].z-8;
  const bool landing_observed=ground&&v.landed_at>m_plane.mission_started_at&&
    (autoland||(m_plane.mission_seq==plane::landing_item&&fresh(m_plane.mission_cursor_at,2)));
  if(plane::armCommand(r))return r.params[0]==0&&v.armed&&landing_observed;
  return modeCommand(r,plane::manual_mode)&&!v.armed&&landing_observed;
}

bool Runtime::planeMissionCompleted() const
{
  if(!m_execution||m_execution->quarantined||m_plane.mission_return_sent_at<0||
     m_now>=m_execution->expires||m_vehicle.heartbeat_at<=m_plane.mission_return_sent_at)return false;
  return m_execution->home.in_air?
    m_vehicle.mode==plane::manual_mode&&!m_vehicle.armed&&plane::groundState(m_vehicle,m_now)&&
      m_vehicle.landed_at>m_plane.mission_return_sent_at:
    m_vehicle.mode==loiter&&flightReady(false);
}

void Runtime::refreshPlaneMissionExecution()
{
  auto& lease=*m_execution;
  if(lease.quarantined)return;
  if(!m_configuration_valid||lease.session_epoch!=m_connection_revision||!m_stored_mission.valid||
     m_stored_mission.revision!=lease.proof_revision||!planeProfileReady()) {
    quarantineMissionExecution("plane_session_or_plan_lost");return;
  }
  if(m_now>=lease.expires||!m_vehicle.heartbeat_valid||!fresh(m_vehicle.heartbeat_at,2)||
     !m_vehicle.position_valid||!fresh(m_vehicle.position_at,1)) {
    quarantineMissionExecution("plane_deadline_or_observation_lost");return;
  }
  const auto mode=m_vehicle.mode;
  const auto execution_mode=lease.home.in_air&&planeNativeAutoland()?plane::autoland_mode:plane::auto_mode;
  if(mode==execution_mode) {
    if(!lease.start_handoff)quarantineMissionExecution("plane_unowned_auto");
    else lease.auto_seen=true;
    if(execution_mode==plane::auto_mode&&m_plane.mission_started_at>=0&&m_plane.mission_cursor_at>m_plane.mission_started_at) {
      const auto first=lease.home.in_air?plane::approach_entry_item:plane::takeoff_item;
      const auto last=lease.home.in_air?plane::landing_item:plane::takeoff_hold_item;
      if(m_plane.mission_seq<first||m_plane.mission_seq>last)quarantineMissionExecution("plane_cursor_outside_verified_phase");
    }
  } else if(!(mode==lease.home.mode&&!lease.auto_seen) &&
            !(m_plane.mission_return_sent_at>=0&&mode==(lease.home.in_air?plane::manual_mode:loiter))) {
    quarantineMissionExecution("plane_external_mode_change");
  }
  if(!lease.home.in_air&&!m_vehicle.armed)quarantineMissionExecution("plane_takeoff_disarmed");
}

void Runtime::trackPlaneMissionHandoff(const CommandRequest& r,bool accepted)
{
  if(!m_execution||r.execution_id!=m_execution->id)return;
  if(!accepted) { quarantineMissionExecution("plane_local_handoff_failed");return; }
  if(r.command==MAV_CMD_DO_SET_MISSION_CURRENT)m_plane.mission_selected_at=m_now;
  else if(r.command==MAV_CMD_MISSION_START||modeCommand(r,plane::autoland_mode))
    m_plane.mission_started_at=m_now;
  else if(r.command==MAV_CMD_DO_SET_MODE)m_plane.mission_return_sent_at=m_now;
}

bool Runtime::planeModeHandoffEligible(const CommandRequest& r,bool acquisition) const
{
  const bool quadplane=quadPlaneHelmProfile();
  if(quadplane&&!acquisition&&(!r.execution_id.empty()||r.command!=MAV_CMD_DO_SET_MODE))
    return true; // QuadPlane's existing native workflow has its own gate.
  if(!m_plane.heartbeat_started) return false;
  if(acquisition) return planeProfileReady() && m_vehicle.mode==planeHoldMode() && modeCommand(r,guided) &&
    (!quadPlaneHoverProfile()||quadPlaneHoverAdapterReady(false));
  if(!r.execution_id.empty())return planeMissionCommandEligible(r);
  if(plane::armCommand(r)||plane::manualCommand(r))return planeArmAllowed(r);
  if(plane::speedCommand(r))return planeSpeedAllowed(r);
  const auto& transition=m_plane.transition;
  return transition && transition->id==r.id && transition->queued && !transition->attempted &&
    m_now<transition->until && m_vehicle.mode==transition->origin && m_vehicle.armed &&
    (!quadplane||quadPlanePhase()==quadPlaneHelmPhase()) &&
    modeCommand(r,transition->destination) && (transition->internal || (planeProfileReady() && flightReady(false)));
}

bool Runtime::planeResumeAllowed() const
{
  return planeProfileReady() && m_plane.heartbeat_started && m_plane.guided_confirmed && m_plane.verified &&
    fresh(m_plane.verified->verified_at,.75) && !m_plane.awaiting && !m_plane.transition &&
    m_vehicle.mode==guided && flightReady(true) &&
    (!quadPlaneHoverProfile()||(quadPlaneHoverAdapterReady(true)&&m_now<m_plane.hover_lease_until));
}

void Runtime::planPlaneHandback(const std::string& reason,bool owned)
{
  m_plane.intent_until=0;
  if(reason=="normal_handoff") return;
  const bool guided_owned=m_plane.guided_confirmed || m_plane.acquire_sent_at>=0;
  m_plane.guided_confirmed=false;
  m_plane.acquire_sent_at=-1;
  m_plane.awaiting.reset();
  m_plane.verified.reset();
  m_plane.hover_hold.reset();
  m_plane.hover_lease_until=0;
  if(!owned || !guided_owned || m_plane.transition || reason=="connection_lost" ||
     reason=="external_command_admitted" || reason=="external_mode_change" ||
     reason=="mission_execution_acquired" ||
     !m_plane.heartbeat_started || !m_vehicle.armed ||
     (quadPlaneHelmProfile()&&quadPlanePhase()!=quadPlaneHelmPhase()) ||
     m_vehicle.mode!=guided || !m_vehicle.heartbeat_valid || !fresh(m_vehicle.heartbeat_at,2)) return;
  m_plane.transition=PlaneState::Transition{"plane-handback-"+std::to_string(m_authority_revision),
    guided,planeHoldMode(),m_now+handback_window,-1,true,false,false};
  m_plane.handback="AWAIT_MODE";
}

void Runtime::queuePlaneHandback()
{
  auto& pending=m_plane.transition;
  if(!pending || !pending->internal || pending->queued || pending->attempted) return;
  CommandRequest r;
  r.id=pending->id;
  r.command=MAV_CMD_DO_SET_MODE;
  r.params[0]=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  r.params[1]=static_cast<float>(pending->destination);
  r.expires=std::min(m_now+.5,pending->until);
  r.operation_expires=pending->until;
  pending->queued=queueCommand(r,false);
  if(!pending->queued) pending->attempted=true;
}

void Runtime::trackPlaneMode(const CommandRequest& r,bool acquisition,bool accepted)
{
  if(acquisition) {
    if(m_nav_permission && m_acquire_until) {
      if(accepted) m_plane.acquire_sent_at=m_now;
      else revokeNavigation("plane_guided_socket_failed");
    }
    return;
  }
  auto& pending=m_plane.transition;
  if(!pending || pending->id!=r.id) return;
  pending->attempted=true;
  if(accepted) pending->sent_at=m_now;
  else m_plane.handback="HANDOFF_FAILED";
}

void Runtime::refreshPlaneAuthority()
{
  if(!planeGuidanceProfile() || m_shadow || !m_simulation) return;
  const bool quadplane=quadPlaneHelmProfile();
  const bool hover=quadPlaneHoverProfile();
  // The pinned FC can briefly leave MC while entering Guided from QLoiter.
  // That interval grants no target egress and retains the original acquisition deadline.
  if(quadPlaneHoverSettling()&&m_vehicle.mode==guided&&m_vehicle.heartbeat_at>m_plane.acquire_sent_at&&
     m_vehicle.landed_at>m_plane.acquire_sent_at&&flightReady(true)&&
     m_plane.hover_offer_at>m_plane.acquire_sent_at&&quadPlaneHoverAdapterReady(true)) {
    m_plane.guided_confirmed=true;
    retireAcquisition();
  }
  if(quadplane && (m_nav_permission || m_resume_until>0) && !flightReady(false)&&!quadPlaneHoverSettling())
    revokeNavigation(hover?"quadplane_hover_evidence_lost":"quadplane_forward_evidence_lost");
  else if((m_nav_permission || m_resume_until>0) && !planeProfileReady())
    revokeNavigation("plane_profile_lost");
  else if(hover&&m_plane.guided_confirmed&&(m_nav_permission||m_resume_until>0)&&!quadPlaneHoverAdapterReady(true))
    revokeNavigation("quadplane_hover_adapter_lost");
  else if(m_nav_permission && m_now>=m_plane.intent_until)
    revokeNavigation("plane_producer_or_acquisition_expired");
  else if(m_plane.awaiting && m_now>=m_plane.awaiting->verify_until)
    revokeNavigation("plane_target_unconfirmed");
  else if(m_resume_until>0 && (m_now>=m_resume_until || !planeResumeAllowed()))
    revokeNavigation("handoff_window_or_target_expired");
  if(m_plane.transition) {
    const auto pending=*m_plane.transition;
    if(quadplane&&quadPlanePhase()!=quadPlaneHelmPhase()) {
      m_plane.handback="UNCONFIRMED";
      commandFinished(pending.id);
      m_plane.transition.reset();
      return;
    }
    if(pending.sent_at>=0 && m_vehicle.mode==pending.destination && m_vehicle.armed &&
       m_vehicle.heartbeat_at<=pending.sent_at && m_now<pending.until && (pending.internal || planeProfileReady())) return;
    if(m_vehicle.mode!=pending.origin || m_now>=pending.until || !m_vehicle.armed || (!pending.internal && !planeProfileReady())) {
      const bool confirmed=pending.sent_at>=0 && m_vehicle.heartbeat_at>pending.sent_at &&
        m_vehicle.mode==pending.destination && m_now<pending.until;
      m_plane.handback=confirmed?"CONFIRMED":m_vehicle.mode!=pending.origin?"EXTERNAL_MODE":"UNCONFIRMED";
      commandFinished(pending.id);
      m_plane.transition.reset();
    }
  }
}

void Runtime::flushPlanePoint(const NavigationTarget& n,uint64_t authority)
{
  if(quadPlaneHoverProfile()) {flushQuadPlaneHoverPoint(n,authority);return;}
  const auto live=[this,n,authority] {
    return authority==m_authority_revision && m_nav_permission && m_plane.guided_confirmed && !m_plane.transition &&
      m_plane.heartbeat_started && planeProfileReady() && m_now<n.expires && flightReady(true);
  };
  if(!live() || m_plane.awaiting || m_now-m_plane.last_target_at<point_period) return;
  // Insignificant candidates and a waiting readback leave the one newest input
  // slot replaceable. Only an independently confirmed point can support a pause.
  const auto frame=m_backend->navigationFrame(n,m_vehicle,m_now,m_revision,m_codec);
  const auto candidate=frame?reposition(*frame):std::nullopt;
  if(!candidate) { revokeNavigation("plane_target_outside_envelope"); return; }
  if(m_plane.verified &&
     pointSeparation(candidate->x,candidate->y,m_plane.verified->lat,m_plane.verified->lon)<20 &&
     std::abs(candidate->z-m_plane.verified->relative_height)<2) return;
  const double altitude=m_vehicle.home_msl+candidate->z;
  if(!sampleClock() || !live()) return;
  if(!sendControlFrame(*frame,n.expires,live)) {
    if(authority==m_authority_revision) revokeNavigation("plane_target_socket_failed");
    return;
  }
  ++m_nav_accepted;
  m_plane.last_target_at=m_now;
  m_plane.awaiting=PlaneState::Target{candidate->x,candidate->y,candidate->z,m_now,
    m_now+target_readback_window,altitude,0};
}

void Runtime::appendPlaneStatus(std::ostream& s) const
{
  if(!planeGuidanceProfile()) return;
  const auto param_state=m_plane.parameter_mismatch?"MISMATCH":planeProfileReady()?"VERIFIED":
    m_plane.parameter_attempts>=3 && m_now>=m_plane.parameter_retry_at?"UNCONFIRMED":"PENDING";
  s<<",plane_profile="<<param_state<<",plane_parameters_verified="<<m_plane.parameter_index
    <<",plane_native_autoland_verified="<<(planeNativeAutoland()&&planeProfileReady())
    <<",plane_watchdog="<<(planeHeartbeatEligible()?"ENABLED":"WITHHELD")
    <<",plane_watchdog_started="<<m_plane.heartbeat_started
    <<",plane_guided_confirmed="<<m_plane.guided_confirmed
    <<",plane_target="<<(m_plane.awaiting?"AWAIT_READBACK":m_plane.verified?"VERIFIED":"NONE")
    <<",plane_targets_verified="<<m_plane.verified_count<<",plane_handback="<<m_plane.handback;
  if(quadPlaneHelmProfile())
    s<<",quadplane_phase="<<quadPlanePhase()<<",quadplane_forward_helm="<<(!quadPlaneHoverProfile()&&planeProfileReady());
  if(quadPlaneHoverProfile())
    s<<",quadplane_hover_adapter="<<(quadPlaneHoverAdapterReady(m_vehicle.mode==guided)?"READY":"WAITING");
}

} // namespace mavmoos::runtime
