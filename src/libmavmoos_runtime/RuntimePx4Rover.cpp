#include "Runtime.h"
#include <algorithm>
#include <cmath>

namespace mavmoos::runtime {
uint32_t Runtime::verifiedPx4MissionId() const
{
  return px4RoverProfile() && storedMissionProof().valid?m_px4_mission_id:0;
}

void Runtime::observePx4RoverMissionCurrent(const mavlink_message_t& msg,double at)
{
  if(msg.sysid!=1 || msg.compid!=1 || msg.magic!=MAVLINK_STX ||
     !m_vehicle.heartbeat_valid || at<m_vehicle.heartbeat_at || at<m_px4_mission_cursor.at ||
     !px4::fresh(at,m_now,2.5)) return;
  mavlink_mission_current_t cursor{};
  mavlink_msg_mission_current_decode(&msg,&cursor);
  m_px4_mission_cursor={cursor.mission_id,cursor.seq,cursor.total,at};
  if(m_stored_mission.valid && cursor.mission_id && cursor.mission_id!=m_px4_mission_id)
    invalidateStoredMissionProof("px4_mission_id_changed");
}

bool Runtime::px4MissionIdentityCurrent() const
{
  return m_px4_mission_id && m_px4_mission_cursor.id==m_px4_mission_id &&
    m_px4_mission_cursor.total==m_stored_mission.items.size() &&
    m_px4_mission_cursor.seq<m_px4_mission_cursor.total &&
    m_px4_mission_cursor.at>m_stored_mission.verified_at &&
    px4::fresh(m_px4_mission_cursor.at,m_now,2.5);
}

void Runtime::refreshPx4RoverExecution()
{
  auto& lease=*m_execution;
  if(lease.quarantined) return;
  if(!m_configuration_valid || lease.session_epoch!=m_connection_revision ||
     !m_stored_mission.valid || m_stored_mission.revision!=lease.proof_revision ||
     !px4MissionIdentityCurrent()) {
    quarantineMissionExecution("session_or_px4_plan_binding_lost");
    return;
  }
  if(m_now>=lease.expires) { quarantineMissionExecution("original_execution_deadline"); return; }
  const auto* policy=px4RoverPolicy();
  if(!policy || !policy->missionReady(m_vehicle,m_now) || !px4LocalReady()) {
    quarantineMissionExecution("rover_observation_continuity_lost");
    return;
  }
  if(std::abs(m_vehicle.home_latitude-lease.home.home_latitude)>1e-7 ||
     std::abs(m_vehicle.home_longitude-lease.home.home_longitude)>1e-7) {
    quarantineMissionExecution("execution_home_changed");
    return;
  }
  using Phase=ExecutionLease::RoverPhase;
  switch(lease.rover_phase) {
  case Phase::Starting:
    if(m_vehicle.mode==px4::mission) {
      if(!lease.start_handoff) { quarantineMissionExecution("mission_observed_before_start_handoff"); return; }
      if(px4ObservedMode(px4::mission,lease.last_execution_handoff)) {
        lease.rover_phase=Phase::Driving;
        lease.auto_seen=true;
      }
    } else if(m_vehicle.mode!=px4::hold) quarantineMissionExecution("external_mode_change");
    break;
  case Phase::Driving:
    if(m_vehicle.mode!=px4::mission) quarantineMissionExecution("rover_left_mission");
    break;
  case Phase::PauseSent:
    if(m_vehicle.mode==px4::hold) {
      if(roverHoldStopped(lease.rover_transition_at)) lease.rover_phase=Phase::Paused;
    } else if(m_vehicle.mode!=px4::mission) quarantineMissionExecution("external_mode_change");
    break;
  case Phase::Paused:
    if(m_vehicle.mode!=px4::hold) quarantineMissionExecution("external_mode_change");
    break;
  case Phase::ResumeSent:
    if(m_vehicle.mode==px4::mission) {
      if(px4ObservedMode(px4::mission,lease.rover_transition_at)) lease.rover_phase=Phase::Driving;
    } else if(m_vehicle.mode!=px4::hold) quarantineMissionExecution("external_mode_change");
    break;
  }
}

bool Runtime::px4RoverMissionCommandEligible(const CommandRequest& request) const
{
  const auto& lease=*m_execution;
  using Phase=ExecutionLease::RoverPhase;
  const bool mode=request.command==MAV_CMD_DO_SET_MODE &&
    request.params[0]==MAV_MODE_FLAG_CUSTOM_MODE_ENABLED && request.params[1]==4 &&
    std::all_of(request.params.begin()+3,request.params.end(),[](float p){return p==0;});
  if(!request.abort_id.empty())
    return mode && request.params[2]==3 && px4ObservedMode(px4::mission,-1) &&
      !lease.rover_abort_attempted;
  const auto* policy=px4RoverPolicy();
  if(!policy || !policy->missionReady(m_vehicle,m_now) || !px4LocalReady() ||
     !px4MissionIdentityCurrent()) return false;
  if(request.command==MAV_CMD_MISSION_START)
    return lease.rover_phase==Phase::Starting && !lease.rover_start_sent && roverHoldStopped(-1) && request.params[0]==0 &&
      request.params[1]==float(m_stored_mission.items.size()-1) &&
      std::all_of(request.params.begin()+2,request.params.end(),[](float p){return p==0;});
  if(!mode) return false;
  if(request.params[2]==3) return lease.rover_phase==Phase::Driving && px4ObservedMode(px4::mission,-1);
  return request.params[2]==4 && lease.rover_phase==Phase::Paused && roverHoldStopped(lease.rover_transition_at);
}

void Runtime::trackPx4RoverMissionHandoff(const CommandRequest& request,bool accepted)
{
  if(!m_execution || m_execution->id!=request.execution_id) return;
  if(!request.abort_id.empty()) {
    if(request.abort_id==m_execution->abort_id) m_execution->rover_abort_attempted=true;
    return;
  }
  if(!accepted) { quarantineMissionExecution("rover_command_handoff_uncertain"); return; }
  if(request.command==MAV_CMD_MISSION_START) m_execution->rover_start_sent=true;
  if(request.command==MAV_CMD_DO_SET_MODE && !m_execution->quarantined) {
    m_execution->rover_transition_at=m_now;
    m_execution->rover_phase=request.params[2]==3 ? ExecutionLease::RoverPhase::PauseSent :
      ExecutionLease::RoverPhase::ResumeSent;
  }
}
} // namespace mavmoos::runtime
