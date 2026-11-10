/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: RuntimeMissionExecution.cpp                     */
/*    DATE: November 4, 2026                                */
/************************************************************/
// Host-held readback identity and persistent AUTO ownership. No wire encoder.
#include "Runtime.h"
#include "CopterFrame.h"
#include "CopterMissionPolicy.h"
#include "QuadPlanePolicy.h"
#include "PlaneOperationPolicy.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace mavmoos::runtime {
namespace {
bool identity(const std::string& value)
{
  return !value.empty() && value.size()<=96 &&
    std::all_of(value.begin(),value.end(),[](unsigned char c) {
      return (c>='a'&&c<='z') || (c>='A'&&c<='Z') || (c>='0'&&c<='9') || c=='_' || c=='-' || c=='.' || c=='/';
    });
}
bool itemsValid(const std::vector<MissionItem>& items)
{
  if(items.size()<2 || items.size()>64) return false;
  for(unsigned i=0;i<items.size();++i) {
    const auto& item=items[i];
    if(item.seq!=i || item.command!=MAV_CMD_NAV_WAYPOINT || item.autocontinue!=1 ||
       !std::isfinite(item.z) || std::abs(double(item.x))>9e8 || std::abs(double(item.y))>18e8) return false;
    for(float p:item.params) if(!std::isfinite(p)) return false;
    if(i==0) { if(item.frame!=MAV_FRAME_GLOBAL) return false; continue; }
    if(item.frame!=MAV_FRAME_GLOBAL_RELATIVE_ALT || item.z<1 || item.z>50 ||
       item.params[0]<0 || item.params[0]>60 || std::trunc(item.params[0])!=item.params[0] ||
       item.params[1]!=0 || item.params[2]!=0 || item.params[3]!=0) return false;
  }
  return true;
}
}

StoredMissionProof Runtime::storedMissionProof() const
{
  auto proof=m_stored_mission;
  proof.valid=proof.valid && m_configuration_valid && !autoRecoveryBlocksEffects() && proof.session_epoch==m_connection_revision &&
    std::isfinite(proof.verified_at) && proof.verified_at<=m_now;
  return proof;
}

bool Runtime::recordStoredMissionProof(const std::string& id,const std::vector<MissionItem>& items)
{
  return !px4RoverProfile() && recordMissionProof(id,items,0);
}

bool Runtime::recordPx4MissionProof(const std::string& id,const std::vector<MissionItem>& items,uint32_t remote_id)
{
  return px4RoverProfile() && remote_id!=0 && recordMissionProof(id,items,remote_id);
}

bool Runtime::recordMissionProof(const std::string& id,const std::vector<MissionItem>& items,uint32_t remote_id)
{
  if(configurationEffectAttempt()) return false;
  // Only the trusted transfer provider's accepted final ACK callback can issue
  // a proof. A MOOS snapshot/result publication never enters this path.
  if(autoRecoveryBlocksEffects() || !m_configuration_valid || !m_simulation || m_shadow || m_execution || !identity(id) ||
     !m_mission_receipt || !m_mission_receipt_accepted ||
     m_mission_receipt->operation_id!=id || m_mission_receipt->action!=MissionAction::Ack ||
     m_now>=m_mission_receipt->expires ||
     m_mission_receipt->mission_type!=MAV_MISSION_TYPE_MISSION ||
     m_mission_receipt->result!=MAV_MISSION_ACCEPTED || !m_mission_operations.count(id) ||
     !m_mission_mutations.count(id) || m_now>=m_mission_operations.at(id).expires ||
     !m_vehicle.heartbeat_valid || !fresh(m_vehicle.heartbeat_at,2) ||
     !(px4RoverProfile()?px4_rover::validMission(items):
       quadPlaneProfile()?quadplane::validPlan(items,m_vehicle):
       planeProfile()?plane::lifecyclePlan(items,m_vehicle):
       arduRoverMissionPolicy()?ardu_rover_mission::planValid(items,m_vehicle):
       itemsValid(items)||(supportedCopterType(m_expected_type)&&m_expected_autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA&&
         copter_mission::validPlan(items,m_vehicle))) ||
     m_stored_mission_revision==std::numeric_limits<uint64_t>::max()) return false;
  StoredMissionProof proof;
  proof.valid=true;proof.session_epoch=m_connection_revision;proof.revision=++m_stored_mission_revision;
  proof.verification_id=id;proof.verified_at=m_now;proof.items=items;
  // This is a host-local object identifier, not an FC mission ID or digest.
  proof.content_id="mission/"+(m_auto_recovery?m_auto_recovery->session+"/":"")+
    std::to_string(proof.session_epoch)+"/"+std::to_string(proof.revision);
  m_stored_mission=std::move(proof);
  m_px4_mission_id=remote_id;
  publishStoredMissionProof("semantic_upload_readback");
  return true;
}

void Runtime::invalidateStoredMissionProof(const std::string& reason)
{
  if(configurationEffectAttempt()) return;
  const bool was_valid=m_stored_mission.valid;
  m_stored_mission.valid=false;
  quarantineMissionExecution(reason);
  if(was_valid)publishStoredMissionProof(reason);
}

void Runtime::publishStoredMissionProof(const std::string& reason)
{
  const auto& p=m_stored_mission;std::ostringstream out;out.precision(17);
  out<<"schema=1,valid="<<(storedMissionProof().valid?"true":"false")<<",verification_id="<<p.verification_id
     <<",content_id="<<p.content_id<<",revision="<<p.revision<<",session_epoch="<<p.session_epoch
     <<",count="<<p.items.size()<<",verified_at="<<p.verified_at<<",reason="<<reason
     <<",source_age=unknown,atomic=false,external_writer_excluded_by_profile=true";
  if(px4RoverProfile()) out<<",remote_id="<<verifiedPx4MissionId();
  publish("MAVMOOS_STORED_MISSION_PROOF",out.str());
}

bool Runtime::beginMissionExecution(const std::string& id,uint64_t revision,const std::string& content,double expires)
{
  if(configurationEffectAttempt()) return false;
  expireCommandLeases();expireMissionOperations();
  const auto proof=storedMissionProof();
  if(!m_configuration_valid || !m_simulation || m_shadow || !m_backend ||
     m_execution || !m_command_leases.empty() ||
     m_flight_log || !m_mission_mutations.empty() || !identity(id) || !std::isfinite(expires) ||
     expires<=m_now || expires-m_now>120 || !proof.valid || proof.revision!=revision ||
     proof.content_id!=content || !missionEntryReady(proof.items)) return false;
  if(!armAutoRecoveryMarker(expires)) return false;
  ExecutionLease lease;lease.id=id;lease.content_id=content;lease.proof_revision=revision;
  lease.session_epoch=m_connection_revision;lease.started=m_now;lease.expires=expires;lease.home=m_vehicle;
  if(planeProfile()) {
    m_plane.mission_selected_at=m_plane.mission_started_at=m_plane.mission_return_sent_at=-1;
  }
  revokeNavigation("mission_execution_acquired");m_hold.reset();
  m_execution=std::move(lease);publishMissionExecutionLease();return true;
}

bool Runtime::missionEntryReady(const std::vector<MissionItem>& items) const
{
  if(quadPlaneProfile())return quadPlaneMissionEntry(items);
  if(planeProfile())return planeMissionEntry(items);
  if(px4RoverProfile()) {
    const auto* policy=px4RoverPolicy();
    if(!m_auto_recovery || !policy || m_nav_permission || m_resume_until>0 ||
       !px4LocalReady() || !policy->missionReady(m_vehicle,m_now) ||
       !roverHoldStopped(-1) || !px4MissionIdentityCurrent() || !px4_rover::validMission(items)) return false;
    for(const auto& item:items) {
      const double north=(item.x*1e-7-m_vehicle.home_latitude)*px4::metres_per_degree;
      const double east=std::remainder(item.y*1e-7-m_vehicle.home_longitude,360.)*px4::metres_per_degree*
        std::cos(m_vehicle.home_latitude*px4::radians);
      if(!(std::hypot(north,east)<=60)) return false;
    }
    return true;
  }
  if(arduRoverMissionPolicy()) return arduRoverMissionEntryReady(items);
  return m_backend && m_backend->profile()=="arducopter" && supportedCopterType(m_expected_type) &&
    copter_mission::validPlan(items,m_vehicle) && flightReady(false) && copter_mission::entryState(m_vehicle,items);
}

unsigned Runtime::executionMode() const
{
  if(px4RoverProfile())return px4::mission;
  if(quadPlaneProfile())return quadplane::auto_mode;
  if(planeProfile())return plane::auto_mode;
  if(arduRoverMissionPolicy())return ardu_rover_mission::auto_mode;
  return 3;
}

bool Runtime::observedExecutionMode() const
{ return m_vehicle.mode==executionMode() || (planeProfile()&&m_vehicle.mode==plane::autoland_mode); }

bool Runtime::missionExecutionGranted(const std::string& id) const
{
  return !autoRecoveryBlocksEffects() && m_execution && m_execution->id==id && !m_execution->quarantined &&
    m_execution->session_epoch==m_connection_revision && m_now<m_execution->expires &&
    m_stored_mission.valid && m_stored_mission.revision==m_execution->proof_revision;
}

void Runtime::cancelExecutionCommands(bool include_abort)
{
  if(!m_execution) return;
  const auto id=m_execution->id;
  const auto matches=[&](const QueuedCommand& q) {
    return q.request.execution_id==id && (include_abort || q.request.abort_id.empty());
  };
  if(m_command_handoff && matches(*m_command_handoff)) m_command_handoff.reset();
  m_commands.erase(std::remove_if(m_commands.begin(),m_commands.end(),matches),m_commands.end());
  for(const auto& key:m_execution->command_ids) m_command_leases.erase(key);
}

void Runtime::quarantineMissionExecution(const std::string& reason)
{
  if(!m_execution || m_execution->quarantined) return;
  m_execution->quarantined=true;m_execution->reason=reason;
  cancelExecutionCommands(false);publishMissionExecutionLease();
}

void Runtime::refreshMissionExecution()
{
  if(!m_execution) return;
  if(px4RoverProfile()) { refreshPx4RoverExecution(); return; }
  if(quadPlaneProfile()) { refreshQuadPlaneExecution(); return; }
  if(arduRoverMissionPolicy()) { refreshArduRoverMissionExecution(); return; }
  if(planeProfile()) { refreshPlaneMissionExecution(); return; }
  auto& lease=*m_execution;
  if(m_vehicle.heartbeat_valid && fresh(m_vehicle.heartbeat_at,2)) {
    if(m_vehicle.mode==3) {
      lease.auto_seen=true;
      if(!lease.start_handoff) {quarantineMissionExecution("auto_observed_before_start_handoff");return;}
    } else if(lease.auto_seen) lease.copter_auto_departed=true;
  }
  if(lease.quarantined) return;
  if(!m_configuration_valid || lease.session_epoch!=m_connection_revision ||
     !m_stored_mission.valid || m_stored_mission.revision!=lease.proof_revision) {
    quarantineMissionExecution("session_or_mission_binding_lost");return;
  }
  if(m_now>=lease.expires) { quarantineMissionExecution("original_execution_deadline");return; }
  const auto& items=m_stored_mission.items;
  const bool auto_flight=m_vehicle.mode==3&&((m_vehicle.armed&&
    (copter_mission::airbornePhase(m_vehicle)||(copter_mission::takeoffPlan(items)&&
     copter_mission::ground(m_vehicle))))||copter_mission::groundedAtLandingPoint(m_vehicle,items));
  if(!flightReady(false) || (!auto_flight&&!copter_mission::entryState(m_vehicle,items))) {
    quarantineMissionExecution("flight_observation_continuity_lost");return;
  }
  const bool landing_ground=lease.auto_seen&&m_vehicle.mode==3&&
    copter_mission::groundedAtLandingPoint(m_vehicle,items)&&std::abs(m_vehicle.msl_height-lease.home.home_msl)<=1.5;
  if(!landing_ground&&(std::abs(m_vehicle.home_latitude-lease.home.home_latitude)>1e-7 ||
     std::abs(m_vehicle.home_longitude-lease.home.home_longitude)>1e-7 ||
     std::abs(m_vehicle.home_msl-lease.home.home_msl)>.05)) {
    quarantineMissionExecution("execution_home_changed");return;
  }
  if(m_vehicle.mode==3) {
    lease.auto_seen=true;
  } else if(m_vehicle.mode!=4 || lease.auto_seen) quarantineMissionExecution("external_mode_change");
}

bool Runtime::beginMissionExecutionAbort(const std::string& abort_id,const std::string& target,double expires)
{
  if(configurationEffectAttempt()) return false;
  if(autoRecoveryBlocksEffects() || !m_configuration_valid || !m_simulation || m_shadow || !m_backend || !m_execution ||
     m_execution->id!=target || !identity(abort_id) || !std::isfinite(expires) ||
     expires<=m_now || expires-m_now>15 || !m_vehicle.heartbeat_valid || !fresh(m_vehicle.heartbeat_at,2)) return false;
  if(px4RoverProfile() && !px4ObservedMode(px4::mission,-1) && !px4ObservedMode(px4::hold,-1)) return false;
  const bool ardurover=arduRoverMissionPolicy()!=nullptr;
  if(ardurover && !arduRoverStopAllowed()) return false;
  auto& lease=*m_execution;
  if(lease.abort_ids.count(abort_id) || lease.abort_ids.size()>=64) return false;
  if(!lease.abort_id.empty() && m_now<lease.abort_expires) return false;
  quarantineMissionExecution("explicit_abort_or_release");cancelExecutionCommands(true);
  lease.abort_id=abort_id;lease.abort_expires=expires;lease.abort_epoch=m_connection_revision;
  lease.abort_ids.insert(abort_id);
  lease.abort_started=m_now;lease.abort_sent_at=-1;lease.rover_abort_attempted=false;
  lease.quadplane_abort_attempted=false;
  if(ardurover) {
    lease.ardurover_abort_from=m_vehicle.mode;
    lease.ardurover_abort_mode_changed=false;
  }
  publishMissionExecutionLease();return true;
}

bool Runtime::finishMissionExecution(const std::string& id,const std::string& disposition)
{
  if(configurationEffectAttempt()) return false;
  if(!m_execution || m_execution->id!=id) return false;
  const auto& lease=*m_execution;
  const bool rejected=disposition=="REJECTED" && !lease.start_handoff && !lease.auto_seen && lease.last_execution_handoff<0 &&
    m_vehicle.heartbeat_valid && fresh(m_vehicle.heartbeat_at,2) && !observedExecutionMode();
  const auto boundary=std::max({lease.started,lease.last_execution_handoff,lease.abort_sent_at});
  const bool released=disposition=="RELEASED" && !lease.abort_id.empty() &&
    lease.abort_epoch==m_connection_revision && m_now<lease.abort_expires &&
    (px4RoverProfile()?roverHoldStopped(std::max(boundary,lease.abort_started)):
      quadPlaneProfile()?quadPlaneGroundHandback(std::max(boundary,lease.abort_started)):
      arduRoverMissionPolicy()?(lease.abort_sent_at>=lease.abort_started &&
        arduRoverMissionHandback(std::max(boundary,lease.abort_started))):
      m_vehicle.heartbeat_valid && fresh(m_vehicle.heartbeat_at,2) && !observedExecutionMode() &&
      m_vehicle.heartbeat_at>boundary);
  const bool plane_complete=disposition=="COMPLETED"&&planeProfile()&&planeMissionCompleted();
  if(rejected || released || plane_complete) {
    if(!clearAutoRecoveryMarker()) return false;
    cancelExecutionCommands(true);m_execution.reset();publishMissionExecutionLease();return true;
  }
  quarantineMissionExecution("terminal_"+disposition+"_remote_execution_may_continue");
  return disposition!="RELEASED" && disposition!="REJECTED" && !(planeProfile()&&disposition=="COMPLETED");
}

bool Runtime::executionCommandEligible(const CommandRequest& request) const
{
  if(!m_execution || request.execution_id!=m_execution->id || !m_backend ||
     request.operation_expires<=m_now || request.operation_expires<request.expires ||
     !m_backend->missionExecutionCommandAllowed(request,m_vehicle,m_now)) return false;
  if(px4RoverProfile() && !px4RoverMissionCommandEligible(request)) return false;
  if(quadPlaneProfile() && !quadPlaneCommandEligible(request)) return false;
  if(arduRoverMissionPolicy() && !arduRoverMissionCommandEligible(request)) return false;
  if(planeProfile() && !planeMissionCommandEligible(request)) return false;
  if(m_backend->profile()=="arducopter" &&
     (request.command==MAV_CMD_DO_SET_MISSION_CURRENT||request.command==MAV_CMD_MISSION_START) &&
     !copter_mission::entryState(m_vehicle,m_stored_mission.items)) return false;
  const auto& lease=*m_execution;
  if(!request.abort_id.empty()) {
    // A no-send release may use the retained lease after pilot takeover. A
    // wire abort belongs only to this session's continuously observed AUTO.
    if(m_backend->profile()=="arducopter" && (!lease.auto_seen || lease.copter_auto_departed ||
       lease.session_epoch!=m_connection_revision || lease.copter_start_accepted_at<0 ||
       m_vehicle.heartbeat_at<=lease.copter_start_accepted_at)) return false;
    return request.abort_id==lease.abort_id && lease.abort_epoch==m_connection_revision &&
      m_now<lease.abort_expires && request.expires<=lease.abort_expires && request.operation_expires<=lease.abort_expires;
  }
  return missionExecutionGranted(lease.id) && request.expires<=lease.expires &&
    request.operation_expires<=lease.expires;
}

void Runtime::publishMissionExecutionLease()
{
  std::ostringstream out;
  out<<"schema=1,held="<<(m_execution?"true":"false")<<",id="<<(m_execution?m_execution->id:"none")
     <<",quarantined="<<(m_execution&&m_execution->quarantined?"true":"false")
     <<",reason="<<(m_execution?m_execution->reason:"released")
     <<",expires="<<(m_execution?m_execution->expires:0)
     <<",remote_execution_may_continue="<<(m_execution?"true":"false");
  publish("MAVMOOS_MISSION_EXECUTION_LEASE",out.str());
}
} // namespace mavmoos::runtime
