#include "Runtime.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string_view>

namespace mavmoos::runtime {
namespace {
constexpr unsigned camera_slot=0, gimbal_slot=1, invalid_slot=2;
constexpr uint8_t own_system=245, own_component=191;
constexpr uint32_t max_capture_sequence=1u<<24;

unsigned slotFor(PayloadAction action)
{
  switch(action) {
  case PayloadAction::CameraCaptureOne: return camera_slot;
  case PayloadAction::GimbalClaim:
  case PayloadAction::GimbalPoint:
  case PayloadAction::GimbalRelease: return gimbal_slot;
  }
  return invalid_slot;
}

bool serialNewer(uint32_t next,uint32_t previous)
{
  const uint32_t delta=next-previous;
  return delta!=0 && delta<0x80000000u;
}

bool token(std::string_view text)
{
  return !text.empty() && text.size()<=96 &&
    std::all_of(text.begin(),text.end(),[](unsigned char c) {
      return (c>='a'&&c<='z') || (c>='A'&&c<='Z') || (c>='0'&&c<='9') ||
        c=='_' || c=='-' || c=='.' || c=='/';
    });
}
}

void Runtime::enablePayloadBench(uint8_t camera,uint8_t gimbal)
{
  const auto names=moduleNames();
  if(!m_startup_blocked || !m_module_configuration_complete || m_payload_bench ||
     !m_configuration_valid || !m_simulation || m_shadow || m_backend ||
     m_expected_type!=MAV_TYPE_QUADROTOR || m_expected_autopilot!=MAV_AUTOPILOT_ARDUPILOTMEGA ||
     (!camera&&!gimbal) || camera==1 || (gimbal&&gimbal<7) || (camera&&camera==gimbal) ||
     (camera && !m_host.hasCapability("camera.capture.single")) ||
     (gimbal && !m_host.hasCapability("gimbal.control.point")) ||
     std::find(names.begin(),names.end(),"telemetry")==names.end() ||
     std::any_of(names.begin(),names.end(),[](const std::string& name) {
       return name!="telemetry" && name!="camera_status" && name!="gimbal_status";
     })) throw std::invalid_argument("active payload commands require a dedicated disarmed Copter SITL bench");
  m_payload_slots[camera_slot].component=camera;
  m_payload_slots[gimbal_slot].component=gimbal;
  m_payload_bench=true;
}

bool Runtime::payloadGroundSafe() const
{
  return m_payload_bench && !m_backend && m_simulation && !m_shadow &&
    !m_startup_blocked && !m_configuration_callbacks && m_configuration_valid &&
    m_vehicle.type==MAV_TYPE_QUADROTOR && m_vehicle.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA &&
    m_vehicle.heartbeat_valid && fresh(m_vehicle.heartbeat_at,2) && !m_vehicle.armed &&
    m_vehicle.landed_valid && fresh(m_vehicle.landed_at,2) &&
    m_vehicle.landed_state==MAV_LANDED_STATE_ON_GROUND;
}

bool Runtime::payloadReady(PayloadAction action) const
{
  const auto slot=slotFor(action);
  return slot<invalid_slot && m_payload_slots[slot].component && payloadGroundSafe();
}

bool Runtime::payloadEligible(const PayloadRequest& request) const
{
  const unsigned index=slotFor(request.action);
  if(!payloadReady(request.action) || !token(request.id) || !token(request.operation_id) ||
     !std::isfinite(request.expires) || !std::isfinite(request.operation_expires) ||
     request.expires<=m_now || request.expires-m_now>2 || request.operation_expires<request.expires ||
     request.operation_expires-m_now>20 || !std::isfinite(request.pitch_deg) ||
     !std::isfinite(request.yaw_deg)) return false;
  const std::string_view prefix=index==camera_slot?"camera/":"gimbal/";
  if(request.operation_id.size()<=prefix.size() ||
     request.operation_id.compare(0,prefix.size(),prefix)!=0) return false;
  const auto& lease=m_payload_slots[index].lease;
  if(lease && (lease->operation_id!=request.operation_id || lease->until!=request.operation_expires ||
               (lease->cancelled && request.action!=PayloadAction::GimbalRelease))) return false;
  if(index==camera_slot)
    return (!lease || !lease->started) && request.pitch_deg==0 && request.yaw_deg==0 &&
      request.capture_sequence>m_payload_camera_sequence && request.capture_sequence<=max_capture_sequence &&
      m_payload_camera.capture_supported && fresh(m_payload_camera.information_at,5) &&
      m_payload_camera.idle && fresh(m_payload_camera.status_at,2);

  if(request.capture_sequence || !m_payload_gimbal.valid || !fresh(m_payload_gimbal.at,1.5) ||
     m_payload_gimbal.secondary_system || m_payload_gimbal.secondary_component) return false;
  if(request.action!=PayloadAction::GimbalRelease &&
     (m_payload_gimbal.flags&(GIMBAL_MANAGER_FLAGS_RC_EXCLUSIVE|GIMBAL_MANAGER_FLAGS_RC_MIXED)))
    return false;
  if(request.action==PayloadAction::GimbalClaim)
    return (!lease || !lease->started) && request.pitch_deg==0 && request.yaw_deg==0 &&
      m_payload_gimbal.primary_system==0 && m_payload_gimbal.primary_component==0;
  if(!lease || lease->claim_sent_at<0 || m_payload_gimbal.at<=lease->claim_sent_at ||
     m_payload_gimbal.primary_system!=own_system || m_payload_gimbal.primary_component!=own_component ||
     lease->release_attempted) return false;
  if(request.action==PayloadAction::GimbalRelease)
    return request.pitch_deg==0 && request.yaw_deg==0;
  return !lease->point_attempted && request.pitch_deg>=-75 && request.pitch_deg<=20 &&
    request.yaw_deg>=-90 && request.yaw_deg<=90;
}

bool Runtime::payload(const PayloadRequest& request)
{
  if(configurationEffectAttempt() || !payloadEligible(request)) return false;
  auto& slot=m_payload_slots[slotFor(request.action)];
  if(slot.pending || (m_payload_handoff && slotFor(m_payload_handoff->action)==slotFor(request.action)))
    return false;
  if(!slot.lease) slot.lease=PayloadLease{request.operation_id,request.operation_expires};
  slot.pending=request;
  return true;
}

void Runtime::cancelPayload(const std::string& id)
{
  if(configurationEffectAttempt()) return;
  for(auto& slot:m_payload_slots) {
    if(!slot.lease || slot.lease->operation_id!=id) continue;
    slot.lease->cancelled=true;
    slot.pending.reset();
  }
  if(m_payload_handoff && m_payload_handoff->operation_id==id) m_payload_handoff.reset();
}

void Runtime::finishPayload(const std::string& id)
{
  if(configurationEffectAttempt()) return;
  cancelPayload(id);
  for(auto& slot:m_payload_slots)
    if(slot.lease && slot.lease->operation_id==id) slot.lease.reset();
}

void Runtime::observePayload(const mavlink_message_t& message,double at)
{
  if(!m_payload_bench || message.sysid!=1 || !m_vehicle.heartbeat_valid ||
     at<m_vehicle.heartbeat_at || !fresh(at,2) || message.magic!=MAVLINK_STX) return;
  if(m_payload_slots[camera_slot].component && message.compid==m_payload_slots[camera_slot].component) {
    if(message.msgid==MAVLINK_MSG_ID_CAMERA_INFORMATION && at>=m_payload_camera.information_at) {
      mavlink_camera_information_t info{};
      mavlink_msg_camera_information_decode(&message,&info);
      m_payload_camera.information_at=at;
      m_payload_camera.capture_supported=(info.flags&CAMERA_CAP_FLAGS_CAPTURE_IMAGE)!=0;
    } else if(message.msgid==MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS && at>=m_payload_camera.status_at) {
      mavlink_camera_capture_status_t status{};
      mavlink_msg_camera_capture_status_decode(&message,&status);
      if(m_payload_camera.seen_status && !serialNewer(status.time_boot_ms,m_payload_camera.boot_ms)) return;
      m_payload_camera.status_at=at;
      m_payload_camera.boot_ms=status.time_boot_ms;
      m_payload_camera.seen_status=true;
      m_payload_camera.idle=status.image_status==0 && status.video_status==0 && !status.camera_device_id &&
        message.len>MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS_MIN_LEN && status.image_count>=0;
    }
  } else if(m_payload_slots[gimbal_slot].component &&
            message.compid==m_payload_slots[gimbal_slot].component &&
            message.msgid==MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS && at>=m_payload_gimbal.at) {
    mavlink_gimbal_manager_status_t status{};
    mavlink_msg_gimbal_manager_status_decode(&message,&status);
    if(status.gimbal_device_id!=message.compid ||
       (m_payload_gimbal.seen && !serialNewer(status.time_boot_ms,m_payload_gimbal.boot_ms))) return;
    m_payload_gimbal={at,status.time_boot_ms,status.flags,status.primary_control_sysid,status.primary_control_compid,
      status.secondary_control_sysid,status.secondary_control_compid,true,true};
  }
}

void Runtime::resetPayload()
{
  for(auto& slot:m_payload_slots) { slot.lease.reset();slot.pending.reset(); }
  m_payload_handoff.reset();
  m_payload_camera={};
  m_payload_gimbal={};
}

void Runtime::refreshPayload()
{
  if(!m_payload_bench) return;
  for(auto& slot:m_payload_slots) {
    if(slot.lease && (!payloadGroundSafe() || m_now>=slot.lease->until)) {
      slot.lease.reset();
      slot.pending.reset();
    }
  }
}

bool Runtime::payloadWaiting() const
{
  return m_payload_slots[camera_slot].pending || m_payload_slots[gimbal_slot].pending;
}

void Runtime::flushPayload()
{
  const unsigned index=m_payload_slots[m_payload_next].pending?m_payload_next:1-m_payload_next;
  auto& slot=m_payload_slots[index];
  if(!slot.pending) return;
  m_payload_next=1-index;
  const auto request=*slot.pending;
  slot.pending.reset();
  m_payload_handoff=request;
  const auto connection=m_connection_revision;
  const auto live=[this,&request] {
    return m_payload_handoff && m_payload_handoff->id==request.id &&
      m_payload_handoff->operation_id==request.operation_id && payloadEligible(request);
  };
  bool accepted=false;
  if(sampleClock() && live()) {
    uint16_t command=0;
    std::array<float,7> params{};
    switch(request.action) {
    case PayloadAction::CameraCaptureOne:
      command=MAV_CMD_IMAGE_START_CAPTURE;
      params[2]=1;params[3]=static_cast<float>(request.capture_sequence);
      params[6]=std::numeric_limits<float>::quiet_NaN();
      break;
    case PayloadAction::GimbalClaim:
    case PayloadAction::GimbalRelease:
      command=MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
      params[0]=params[1]=request.action==PayloadAction::GimbalClaim?-2.f:-3.f;
      params[2]=params[3]=-1;
      params[6]=slot.component;
      break;
    case PayloadAction::GimbalPoint:
      command=MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW;
      params[0]=request.pitch_deg;params[1]=request.yaw_deg;
      params[2]=params[3]=std::numeric_limits<float>::quiet_NaN();
      params[4]=GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME;
      params[6]=slot.component;
      break;
    }
    mavlink_message_t msg{};
    mavlink_msg_command_long_pack_status(own_system,own_component,&m_codec,&msg,1,slot.component,
      command,0,params[0],params[1],params[2],params[3],params[4],params[5],params[6]);
    accepted=send(msg,request.expires,live);
  }
  m_payload_handoff.reset();
  if(connection!=m_connection_revision) return;
  if(slot.lease && slot.lease->operation_id==request.operation_id) {
    auto& lease=*slot.lease;
    switch(request.action) {
    case PayloadAction::CameraCaptureOne:
      m_payload_camera_sequence=std::max(m_payload_camera_sequence,request.capture_sequence);
      lease.started=true;
      break;
    case PayloadAction::GimbalClaim:
      lease.started=true;
      if(accepted) lease.claim_sent_at=m_now;
      break;
    case PayloadAction::GimbalPoint: lease.point_attempted=true; break;
    case PayloadAction::GimbalRelease: lease.release_attempted=true; break;
    }
  }
  m_host.payloadSent(request,m_now,accepted);
}

} // namespace mavmoos::runtime
