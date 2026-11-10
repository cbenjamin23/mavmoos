#include "Runtime.h"
#include <algorithm>
#include <cmath>

namespace mavmoos::runtime {
namespace {
namespace rover=ardu_rover_mission;

bool zeroArguments(const CommandRequest& request,std::size_t first=0)
{
  return std::all_of(request.params.begin()+first,request.params.end(),[](float value){return value==0;});
}

bool homeBound(const Vehicle& current,const Vehicle& bound)
{
  return std::abs(current.home_latitude-bound.home_latitude)<=1e-7 &&
    std::abs(current.home_longitude-bound.home_longitude)<=1e-7 &&
    std::abs(current.home_msl-bound.home_msl)<=.05;
}
} // namespace

const ArduRoverMissionPolicy* Runtime::arduRoverMissionPolicy() const
{
  if(!m_backend || m_expected_autopilot!=MAV_AUTOPILOT_ARDUPILOTMEGA ||
     (m_expected_type!=MAV_TYPE_GROUND_ROVER && m_expected_type!=MAV_TYPE_SURFACE_BOAT)) return nullptr;
  const auto profile=m_backend->profile();
  if(((profile=="ardurover" && m_expected_type==MAV_TYPE_GROUND_ROVER) ||
      (profile=="ardurover_boat" && m_expected_type==MAV_TYPE_SURFACE_BOAT)) &&
     m_host.hasCapability("control.mission.start"))
    return dynamic_cast<const ArduRoverMissionPolicy*>(m_backend);
  return nullptr;
}

bool Runtime::arduRoverHandbackMode() const
{
  return m_vehicle.heartbeat_valid && fresh(m_vehicle.heartbeat_at,2) &&
    rover::handbackModeAllowed(m_expected_type==MAV_TYPE_SURFACE_BOAT,m_vehicle.mode);
}

bool Runtime::arduRoverMissionHandback(double after) const
{
  const auto* policy=arduRoverMissionPolicy();
  return policy && arduRoverHandbackMode() && m_vehicle.heartbeat_at>after &&
    policy->settled(m_vehicle,m_now,after);
}

bool Runtime::arduRoverStopAllowed() const
{
  if(arduRoverHandbackMode()) return true;
  return m_execution && m_vehicle.mode==rover::auto_mode && m_vehicle.heartbeat_valid &&
    fresh(m_vehicle.heartbeat_at,2) && m_execution->session_epoch==m_connection_revision &&
    m_execution->ardurover_auto_authority==ExecutionLease::ArduRoverAutoAuthority::Owned;
}

void Runtime::observeArduRoverAutoAuthority()
{
  auto& lease=*m_execution;
  using Authority=ExecutionLease::ArduRoverAutoAuthority;
  using Phase=ExecutionLease::RoverPhase;
  auto& authority=lease.ardurover_auto_authority;
  if(lease.session_epoch!=m_connection_revision) authority=Authority::Lost;
  if(!m_vehicle.heartbeat_valid || !fresh(m_vehicle.heartbeat_at,2)) return;

  const auto mode=m_vehicle.mode;
  if(!lease.abort_id.empty() && mode!=lease.ardurover_abort_from)
    lease.ardurover_abort_mode_changed=true;

  const bool in_auto=mode==rover::auto_mode;
  const auto pause_mode=rover::handbackMode(m_expected_type==MAV_TYPE_SURFACE_BOAT);
  const bool post_transition=m_vehicle.heartbeat_at>lease.rover_transition_at;
  switch(authority) {
    case Authority::BeforeStart:
      if(in_auto) {
        if(!lease.rover_start_sent) authority=Authority::Lost;
        else if(post_transition) authority=Authority::Owned;
      } else if(!arduRoverHandbackMode()) authority=Authority::Lost;
      break;
    case Authority::Owned:
      if(!in_auto)
        authority=mode==pause_mode && lease.rover_phase==Phase::PauseSent?Authority::Paused:Authority::Lost;
      break;
    case Authority::Paused:
      if(in_auto) {
        if(lease.rover_phase!=Phase::ResumeSent) authority=Authority::Lost;
        else if(post_transition) authority=Authority::Owned;
      } else if(mode!=pause_mode) authority=Authority::Lost;
      break;
    case Authority::Lost:
      break;
  }
}

void Runtime::observeArduRoverMissionCurrent(const mavlink_message_t& msg,double at)
{
  if(msg.sysid!=1 || msg.compid!=1 || msg.magic!=MAVLINK_STX || msg.len<5 ||
     !m_vehicle.heartbeat_valid || at<=m_ardu_rover_cursor.at ||
     !rover::fresh(at,m_now,rover::cursor_age)) return;
  mavlink_msg_mission_current_decode(&msg,&m_ardu_rover_cursor.value);
  m_ardu_rover_cursor.at=at;
  if(m_stored_mission.valid && at>m_stored_mission.verified_at &&
     m_ardu_rover_cursor.value.total!=m_stored_mission.items.size()-1)
    invalidateStoredMissionProof("ardurover_mission_count_changed");
}

bool Runtime::arduRoverCursorFresh(double after) const
{
  return m_ardu_rover_cursor.at>after && rover::fresh(m_ardu_rover_cursor.at,m_now,rover::cursor_age) &&
    m_stored_mission.items.size()>=3 && m_ardu_rover_cursor.value.total==m_stored_mission.items.size()-1 &&
    m_ardu_rover_cursor.value.seq<m_stored_mission.items.size();
}

bool Runtime::arduRoverMissionEntryReady(const std::vector<MissionItem>& items) const
{
  const auto* policy=arduRoverMissionPolicy();
  return policy && m_auto_recovery && !m_nav_permission && m_resume_until<=m_now &&
    rover::planValid(items,m_vehicle) && policy->ready(m_vehicle,m_now) &&
    arduRoverMissionHandback(m_stored_mission.verified_at) && arduRoverCursorFresh(m_stored_mission.verified_at) &&
    m_ardu_rover_cursor.value.mission_mode==0 &&
    (m_ardu_rover_cursor.value.mission_state==MISSION_STATE_NOT_STARTED ||
     m_ardu_rover_cursor.value.mission_state==MISSION_STATE_COMPLETE);
}

void Runtime::refreshArduRoverMissionExecution()
{
  auto& lease=*m_execution;
  // Keep observing the stop origin while a lost/completed execution retains
  // its marker. A quarantined lease cannot adopt a later pilot AUTO entry.
  observeArduRoverAutoAuthority();
  if(lease.quarantined) return;
  const auto* policy=arduRoverMissionPolicy();
  if(!m_configuration_valid || !policy || !m_stored_mission.valid ||
     lease.proof_revision!=m_stored_mission.revision || lease.session_epoch!=m_connection_revision) {
    quarantineMissionExecution("ardurover_session_or_plan_lost");
    return;
  }
  if(lease.ardurover_auto_authority==ExecutionLease::ArduRoverAutoAuthority::Lost) {
    quarantineMissionExecution("ardurover_auto_authority_lost");
    return;
  }
  if(m_now>=lease.expires) { quarantineMissionExecution("original_execution_deadline"); return; }
  if(!policy->ready(m_vehicle,m_now) || !homeBound(m_vehicle,lease.home) ||
     !arduRoverCursorFresh(m_stored_mission.verified_at)) {
    quarantineMissionExecution("ardurover_mission_observation_continuity_lost");
    return;
  }

  const auto& cursor=m_ardu_rover_cursor.value;
  const bool in_auto=m_vehicle.mode==rover::auto_mode;
  const bool handback=arduRoverHandbackMode();
  const bool active=rover::activeCursor(cursor) &&
    rover::selectedCursor(cursor,m_stored_mission.items.size()) &&
    arduRoverCursorFresh(lease.rover_transition_at);
  using Phase=ExecutionLease::RoverPhase;
  switch(lease.rover_phase) {
    case Phase::Starting:
      if(in_auto) {
        if(!lease.rover_start_sent) { quarantineMissionExecution("unrequested_ardurover_auto"); return; }
        if(active && cursor.seq==1 && m_vehicle.heartbeat_at>lease.rover_transition_at) {
          lease.auto_seen=true;
          lease.rover_phase=Phase::Driving;
        }
      } else if(!handback) quarantineMissionExecution("external_mode_change");
      break;
    case Phase::Driving:
      if(!in_auto) quarantineMissionExecution("ardurover_left_auto");
      break;
    case Phase::PauseSent:
      if(handback && arduRoverMissionHandback(lease.rover_transition_at) &&
         arduRoverCursorFresh(lease.rover_transition_at) && rover::inactiveCursor(cursor) &&
         rover::selectedCursor(cursor,m_stored_mission.items.size())) {
        lease.ardurover_paused_seq=cursor.seq;
        lease.rover_phase=Phase::Paused;
      } else if(!in_auto && !handback) quarantineMissionExecution("external_mode_change");
      break;
    case Phase::Paused:
      if(!handback || !rover::inactiveCursor(cursor) || cursor.seq!=lease.ardurover_paused_seq)
        quarantineMissionExecution("ardurover_paused_cursor_or_mode_changed");
      break;
    case Phase::ResumeSent:
      if(in_auto && active && m_vehicle.heartbeat_at>lease.rover_transition_at) {
        if(cursor.seq!=lease.ardurover_paused_seq) quarantineMissionExecution("ardurover_resume_cursor_changed");
        else lease.rover_phase=Phase::Driving;
      } else if(!in_auto && !handback) quarantineMissionExecution("external_mode_change");
      break;
  }
}

bool Runtime::arduRoverMissionCommandEligible(const CommandRequest& request) const
{
  const auto& lease=*m_execution;
  const auto stop_mode=rover::handbackMode(m_expected_type==MAV_TYPE_SURFACE_BOAT);
  const bool in_auto=m_vehicle.mode==rover::auto_mode;
  if(!request.abort_id.empty())
    return !lease.rover_abort_attempted && !lease.ardurover_abort_mode_changed &&
      m_vehicle.mode==lease.ardurover_abort_from && rover::wireMode(request,stop_mode) &&
      arduRoverStopAllowed();

  const auto* policy=arduRoverMissionPolicy();
  if(!policy || !policy->ready(m_vehicle,m_now) || !homeBound(m_vehicle,lease.home) ||
     !arduRoverCursorFresh(m_stored_mission.verified_at)) return false;
  const auto& cursor=m_ardu_rover_cursor.value;
  using Phase=ExecutionLease::RoverPhase;
  if(request.command==MAV_CMD_DO_SET_MISSION_CURRENT)
    return lease.rover_phase==Phase::Starting && lease.ardurover_select_at<0 &&
      request.params[0]==1 && zeroArguments(request,1) &&
      arduRoverMissionHandback(m_stored_mission.verified_at);
  if(request.command==MAV_CMD_MISSION_START)
    return lease.rover_phase==Phase::Starting && lease.ardurover_select_at>=0 &&
      !lease.rover_start_sent && zeroArguments(request) &&
      arduRoverMissionHandback(m_stored_mission.verified_at) &&
      arduRoverCursorFresh(lease.ardurover_select_at) && cursor.seq==1 && rover::inactiveCursor(cursor);
  if(rover::wireMode(request,stop_mode)) return in_auto && lease.rover_phase==Phase::Driving;
  return rover::wireMode(request,rover::auto_mode) && lease.rover_phase==Phase::Paused &&
    arduRoverMissionHandback(lease.rover_transition_at) && rover::inactiveCursor(cursor) &&
    cursor.seq==lease.ardurover_paused_seq;
}

void Runtime::trackArduRoverMissionHandoff(const CommandRequest& request,bool accepted)
{
  if(!m_execution || m_execution->id!=request.execution_id) return;
  auto& lease=*m_execution;
  if(!request.abort_id.empty()) {
    if(request.abort_id==lease.abort_id) lease.rover_abort_attempted=true;
    return;
  }
  if(!accepted) { quarantineMissionExecution("ardurover_command_handoff_uncertain"); return; }
  using Phase=ExecutionLease::RoverPhase;
  if(request.command==MAV_CMD_DO_SET_MISSION_CURRENT) {
    lease.ardurover_select_at=m_now;
  } else if(request.command==MAV_CMD_MISSION_START) {
    lease.rover_start_sent=true;
    lease.rover_transition_at=m_now;
  } else if(request.command==MAV_CMD_DO_SET_MODE) {
    lease.rover_phase=request.params[1]==rover::auto_mode?Phase::ResumeSent:Phase::PauseSent;
    lease.rover_transition_at=m_now;
  }
}
} // namespace mavmoos::runtime
