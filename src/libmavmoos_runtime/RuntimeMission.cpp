/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: RuntimeMission.cpp                              */
/*    DATE: November 4, 2026                                */
/************************************************************/
#include "Runtime.h"
#include "CopterFrame.h"
#include "QuadPlanePolicy.h"
#include "FencePlan.h"
#include "PlaneOperationPolicy.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace mavmoos::runtime {
namespace {
std::string clockStamp(double value)
{
  std::ostringstream out;out<<std::setprecision(17)<<value;return out.str();
}
bool identifier(const std::string& value)
{
  return !value.empty() && value.size()<=96 &&
    std::all_of(value.begin(),value.end(),[](unsigned char c) {
      return (c>='a'&&c<='z') || (c>='A'&&c<='Z') ||
        (c>='0'&&c<='9') || c=='_' || c=='-' || c=='.' || c=='/';
    });
}
bool mutation(MissionAction action)
{
  return action==MissionAction::Count || action==MissionAction::Item || action==MissionAction::Clear;
}
bool itemShape(const MissionItem& item)
{
  if(item.seq>=4096 || item.current>1 || item.autocontinue>1 || !std::isfinite(item.z)) return false;
  for(float value:item.params) if(!std::isfinite(value)) return false;
  if(item.frame==MAV_FRAME_MISSION) return true;
  if(item.frame!=MAV_FRAME_GLOBAL && item.frame!=MAV_FRAME_GLOBAL_RELATIVE_ALT &&
     item.frame!=MAV_FRAME_GLOBAL_TERRAIN_ALT && item.frame!=MAV_FRAME_GLOBAL_INT &&
     item.frame!=MAV_FRAME_GLOBAL_RELATIVE_ALT_INT && item.frame!=MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) return false;
  return std::abs(double(item.x))<=9e8 && std::abs(double(item.y))<=18e8;
}
}

bool Runtime::missionEligible(const MissionRequest& request) const
{
  if(!m_configuration_valid || !m_simulation || m_shadow || m_files_bench ||
     !identifier(request.id) || !identifier(request.operation_id) ||
     !std::isfinite(request.expires) || !std::isfinite(request.operation_expires) ||
     request.expires<=m_now || request.expires-m_now>2 ||
     request.operation_expires<request.expires || request.operation_expires<=m_now ||
     request.operation_expires-m_now>120 ||
     !m_vehicle.heartbeat_valid || !fresh(m_vehicle.heartbeat_at,2)) return false;
  const bool typed=request.mission_type!=MAV_MISSION_TYPE_MISSION;
  const bool changes_plan=mutation(request.action);
  if(typed && request.mission_type!=MAV_MISSION_TYPE_FENCE && request.mission_type!=MAV_MISSION_TYPE_RALLY)
    return false;
  const bool fence_write=changes_plan && request.mission_type==MAV_MISSION_TYPE_FENCE &&
    m_vehicle.type==MAV_TYPE_QUADROTOR && m_vehicle.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA &&
    m_host.hasCapability("missions.fence.replace.simulator");
  if(typed && changes_plan && !fence_write) return false;
  const bool starts_fence_write=fence_write &&
    (request.action==MissionAction::Count || request.action==MissionAction::Clear);
  const auto operation=m_mission_operations.find(request.operation_id);
  if(operation!=m_mission_operations.end()) {
    if(operation->second.expires!=request.operation_expires || operation->second.type!=request.mission_type ||
       (changes_plan && !m_mission_mutations.count(request.operation_id) && !starts_fence_write)) return false;
  } else if(typed && request.action!=MissionAction::RequestList && !starts_fence_write) return false;
  switch(request.action) {
  case MissionAction::RequestList: break;
  case MissionAction::RequestItem: if(request.seq>=4096) return false; break;
  case MissionAction::Count:
    if(request.count>4096 || (fence_write && (request.count<3 || request.count>maximum_fence_vertices))) return false;
    break;
  case MissionAction::Item:
    if(fence_write ? (request.seq!=request.item.seq || !fenceVertex(request.item,request.count)) :
       !itemShape(request.item)) return false;
    break;
  case MissionAction::Clear: break;
  case MissionAction::Ack:
    if(request.result!=MAV_MISSION_ACCEPTED && request.result!=MAV_MISSION_ERROR &&
       request.result!=MAV_MISSION_OPERATION_CANCELLED) return false;
    if(typed && request.result!=MAV_MISSION_ACCEPTED) return false;
    break;
  default: return false;
  }
  if(autoRecoveryBlocksEffects() && request.action==MissionAction::Ack &&
     m_mission_mutations.count(request.operation_id)) return false;
  if(!changes_plan) return true;
  if(m_flight_log || m_vehicle.armed || m_nav_permission || m_shadow_grant || m_resume_until>m_now || controlBusy()) return false;
  if(quadPlaneProfile() && (m_vehicle.mode!=quadplane::qloiter || quadPlanePhase()!=MAV_VTOL_STATE_MC ||
     !quadplane::stoppedOnGround(m_vehicle,m_now) ||
     (request.action==MissionAction::Count && request.count!=quadplane::plan_items) ||
     (request.action==MissionAction::Item && !quadplane::validItem(request.item)))) return false;
  if(planeProfile())
    return m_auto_recovery&&!autoRecoveryBlocksEffects()&&planeProfileReady()&&m_plane.heartbeat_started&&
      m_vehicle.mode==plane::manual_mode&&plane::groundState(m_vehicle,m_now)&&
      (request.action!=MissionAction::Count||request.count==plane::lifecycle_items)&&
      (request.action!=MissionAction::Item||plane::lifecycleItem(request.item));
  if(px4RoverProfile()) {
    const auto* policy=px4RoverPolicy();
    if(!policy || !policy->verifiedConfig(m_now) || !roverHoldStopped(-1) ||
       (request.action==MissionAction::Count && (request.count<2 || request.count>8)) ||
       (request.action==MissionAction::Item && !px4_rover::validWaypoint(request.item))) return false;
  }
  if(arduRoverMissionPolicy() && request.mission_type==MAV_MISSION_TYPE_MISSION &&
     (!arduRoverMissionHandback(-1) || !fresh(m_vehicle.home_at,10) ||
      (request.action==MissionAction::Count && (request.count<3 || request.count>8)) ||
      (request.action==MissionAction::Item && !ardu_rover_mission::boundPoint(request.item,m_vehicle)))) return false;
  // Copter mutation additionally needs actual ground evidence. A Rover is not
  // required to invent an airborne/landed state to edit its disarmed plan.
  if(supportedCopterType(m_vehicle.type) && (!m_vehicle.landed_valid ||
     !fresh(m_vehicle.landed_at,2) || m_vehicle.landed_state!=MAV_LANDED_STATE_ON_GROUND)) return false;
  return true;
}

bool Runtime::mission(const MissionRequest& request)
{
  if(configurationEffectAttempt()) return false;
  expireMissionOperations();
  if(m_missions.size()>=4 || !missionEligible(request)) return false;
  // Mission replies contain no caller transaction ID; serialize this protocol.
  if(!m_mission_operations.count(request.operation_id) && !m_mission_operations.empty()) return false;
  for(const auto& queued:m_missions) if(queued.id==request.id) return false;
  m_mission_operations.emplace(request.operation_id,MissionOperation{request.operation_expires,request.mission_type});
  if(mutation(request.action)) {
    if(request.mission_type==MAV_MISSION_TYPE_MISSION)
      invalidateStoredMissionProof("mission_mutation_admitted");
    m_mission_mutations.emplace(request.operation_id,request.operation_expires);
  }
  m_missions.push_back(request);
  return true;
}

void Runtime::missionFinished(const std::string& operation_id)
{
  if(configurationEffectAttempt()) return;
  if(m_mission_handoff && m_mission_handoff->operation_id==operation_id) m_mission_handoff.reset();
  m_mission_operations.erase(operation_id); m_mission_mutations.erase(operation_id);
  m_missions.erase(std::remove_if(m_missions.begin(),m_missions.end(),[&](const auto& request) {
    return request.operation_id==operation_id;
  }),m_missions.end());
}

void Runtime::expireMissionOperations()
{
  for(auto it=m_mission_operations.begin();it!=m_mission_operations.end();)
    if(m_now>=it->second.expires) { const auto id=it->first; ++it; missionFinished(id); }
    else ++it;
}

bool Runtime::flushMission()
{
  const auto request=m_missions.front(); m_missions.pop_front();
  m_mission_handoff=request;
  mavlink_message_t message{};
  switch(request.action) {
  case MissionAction::RequestList: {
    mavlink_mission_request_list_t value{}; value.target_system=1; value.target_component=1;
    value.mission_type=request.mission_type;
    mavlink_msg_mission_request_list_encode_status(245,191,&m_codec,&message,&value); break;
  }
  case MissionAction::RequestItem: {
    mavlink_mission_request_int_t value{}; value.target_system=1; value.target_component=1;
    value.seq=request.seq; value.mission_type=request.mission_type;
    mavlink_msg_mission_request_int_encode_status(245,191,&m_codec,&message,&value); break;
  }
  case MissionAction::Count: {
    mavlink_mission_count_t value{}; value.target_system=1; value.target_component=1;
    value.count=request.count; value.mission_type=request.mission_type;
    mavlink_msg_mission_count_encode_status(245,191,&m_codec,&message,&value); break;
  }
  case MissionAction::Item: {
    mavlink_mission_item_int_t value{}; value.target_system=1; value.target_component=1;
    value.seq=request.item.seq; value.command=request.item.command; value.frame=request.item.frame;
    value.current=request.item.current; value.autocontinue=request.item.autocontinue;
    value.param1=request.item.params[0]; value.param2=request.item.params[1];
    value.param3=request.item.params[2]; value.param4=request.item.params[3];
    value.x=request.item.x; value.y=request.item.y; value.z=request.item.z;
    value.mission_type=request.mission_type;
    mavlink_msg_mission_item_int_encode_status(245,191,&m_codec,&message,&value); break;
  }
  case MissionAction::Clear: {
    mavlink_mission_clear_all_t value{}; value.target_system=1; value.target_component=1;
    value.mission_type=request.mission_type;
    mavlink_msg_mission_clear_all_encode_status(245,191,&m_codec,&message,&value); break;
  }
  case MissionAction::Ack: {
    mavlink_mission_ack_t value{}; value.target_system=1; value.target_component=1;
    value.type=request.result; value.mission_type=request.mission_type;
    mavlink_msg_mission_ack_encode_status(245,191,&m_codec,&message,&value); break;
  }
  }
  if(!sampleClock()) return false;
  const auto live=[this,request] {return m_mission_handoff && m_mission_handoff->id==request.id &&
    m_mission_handoff->operation_id==request.operation_id && m_mission_operations.count(request.operation_id) && missionEligible(request);};
  const bool accepted=live() && send(message,request.expires,live);
  m_mission_handoff.reset();
  const bool clock_ok=sampleClock();
  publish("MAVMOOS_MISSION_TX_RESULT","id="+request.id+",operation_id="+request.operation_id+
    ",msgid="+std::to_string(message.msgid)+
    ",at="+clockStamp(m_now)+
    ",status="+(accepted?"LOCAL_ACCEPTED":"NOT_ACCEPTED")+
    ",late="+(m_now>=request.expires?"true":"false")+",clock_valid="+(clock_ok?"true":"false")+
    ",mission_type="+std::to_string(request.mission_type));
  m_mission_receipt=request;m_mission_receipt_accepted=accepted&&clock_ok;
  try { m_host.missionSent(request,m_now,accepted); }
  catch(...) {m_mission_receipt.reset();m_mission_receipt_accepted=false;throw;}
  m_mission_receipt.reset();m_mission_receipt_accepted=false;
  return clock_ok;
}
} // namespace mavmoos::runtime
