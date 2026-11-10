#include "Runtime.h"
#include "Px4AircraftPolicy.h"
#include <algorithm>
#include <cmath>
#include <ostream>

namespace mavmoos::runtime {
bool Runtime::px4Profile() const
{
  return m_expected_autopilot==MAV_AUTOPILOT_PX4 && m_backend &&
    px4::isProfile(m_backend->profile());
}

bool Runtime::px4FixedWingProfile() const
{
  return m_expected_autopilot==MAV_AUTOPILOT_PX4 && m_backend &&
    m_backend->profile()==px4::fixed_wing_profile;
}

bool Runtime::px4VtolProfile() const
{
  return px4Profile() && m_backend->profile()==px4::standard_vtol_profile;
}

bool Runtime::px4ForwardFlight() const
{
  const auto* aircraft=px4VtolProfile()?dynamic_cast<const Px4AircraftPolicy*>(m_backend):nullptr;
  return px4FixedWingProfile() || (aircraft && aircraft->forwardFlight(m_now));
}

bool Runtime::px4AircraftNativeActions() const
{
  const auto* aircraft=px4Profile()?dynamic_cast<const Px4AircraftPolicy*>(m_backend):nullptr;
  return aircraft && aircraft->nativeActionsEnabled();
}

bool Runtime::px4OffboardIntentRequired() const
{
  return px4RoverProfile() || px4VtolProfile() || px4AircraftNativeActions();
}

bool Runtime::px4RoverProfile() const
{
  return m_expected_autopilot==MAV_AUTOPILOT_PX4 && m_expected_type==MAV_TYPE_GROUND_ROVER &&
    m_backend && m_backend->profile()==px4::rover_profile;
}

const Px4RoverPolicy* Runtime::px4RoverPolicy() const
{
  return px4RoverProfile()?dynamic_cast<const Px4RoverPolicy*>(m_backend):nullptr;
}

bool Runtime::px4ObservedMode(unsigned mode,double after) const
{
  return m_vehicle.heartbeat_valid && m_vehicle.mode==mode && m_vehicle.heartbeat_at>after &&
    px4::fresh(m_vehicle.heartbeat_at,m_now,2) && m_px4_mode.active==mode &&
    m_px4_mode.intended==mode && m_px4_mode.at>after && px4::fresh(m_px4_mode.at,m_now,1.6);
}

bool Runtime::roverHoldStopped(double after) const
{
  const auto* policy=px4RoverPolicy();
  return policy && px4ObservedMode(px4::hold,after) && policy->stopped(m_now,after);
}

bool Runtime::px4LocalReady() const
{
  return m_px4_local && px4::fresh(m_px4_local->at,m_now,px4::local_age);
}

bool Runtime::px4NativeHoldReady() const
{
  // HEARTBEAT alone shows Hold during Offboard loss even while PX4 would
  // automatically re-enter Offboard if it received a stream again.
  return m_vehicle.heartbeat_valid && m_vehicle.mode==px4::hold &&
    m_px4_mode.active==px4::hold && m_px4_mode.intended==px4::hold &&
    px4::fresh(m_px4_mode.at,m_now,1.6);
}

void Runtime::observePx4ModeIntent(const mavlink_message_t& msg,double at)
{
  if(msg.sysid!=1 || msg.compid!=1 || msg.magic!=MAVLINK_STX ||
     !m_vehicle.heartbeat_valid || at<m_vehicle.heartbeat_at ||
     at<m_px4_mode.at || !px4::fresh(at,m_now,1.6)) return;
  mavlink_current_mode_t p{};
  mavlink_msg_current_mode_decode(&msg,&p);
  m_px4_mode={p.custom_mode,p.intended_custom_mode,at};
}

void Runtime::observePx4Local(const mavlink_message_t& msg,double at)
{
  if(msg.sysid!=1 || msg.compid!=1 || msg.magic!=MAVLINK_STX ||
     !m_vehicle.heartbeat_valid || at<m_vehicle.heartbeat_at ||
     !px4::fresh(at,m_now,px4::local_age)) return;
  mavlink_local_position_ned_t p{};
  mavlink_msg_local_position_ned_decode(&msg,&p);
  if(m_px4_local && (at<m_px4_local->at || p.time_boot_ms==m_px4_local->boot)) return;
  const bool finite=std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z) &&
    std::isfinite(p.vx)&&std::isfinite(p.vy)&&std::isfinite(p.vz);
  if(!finite || std::abs(p.x)>10000 || std::abs(p.y)>10000 || std::abs(p.z)>1000) {
    m_px4_local.reset();
    return;
  }
  if(m_px4_local) {
    const uint32_t delta=p.time_boot_ms-m_px4_local->boot;
    const auto& before=m_px4_local->point;
    const double distance=std::hypot(std::hypot(p.x-before.north,p.y-before.east),p.z-before.down);
    // Reject a discontinuous local reference (including a reset of the remote
    // boot clock) before the new sample can keep an old anchor alive.
    const double speed_limit=(px4FixedWingProfile() || px4VtolProfile())?px4::fixed_wing_max_ground_speed+5:12.;
    if(delta>=0x80000000u || distance>1.+speed_limit*(at-m_px4_local->at)) {
      m_px4_local.reset();
      if(m_nav_permission || m_resume_until>0) revokeNavigation("px4_local_reference_changed");
      return;
    }
  }
  m_px4_local=Px4LocalSample{{p.x,p.y,p.z},at,p.time_boot_ms};
}

bool Runtime::acceptPx4OrderedPosition(const mavlink_message_t& msg,double at,uint32_t boot)
{
  if(msg.sysid!=1 || msg.compid!=1 || msg.magic!=MAVLINK_STX ||
     !m_vehicle.heartbeat_valid || at<m_vehicle.heartbeat_at || at<m_vehicle.position_at ||
     !px4::fresh(at,m_now,px4::position_age)) return false;
  if(m_px4_global_boot) {
    const uint32_t delta=boot-*m_px4_global_boot;
    if(delta==0) return false;
    if(delta>=0x80000000u) {
      m_vehicle.position_valid=false;
      m_px4_global_boot.reset();
      if(m_nav_permission || m_resume_until>0) revokeNavigation("px4_global_position_clock_regressed");
      return false;
    }
  }
  m_px4_global_boot=boot;
  return true;
}

bool Runtime::beginPx4Acquisition(const std::string& id)
{
  if(!px4LocalReady() || !px4NativeHoldReady() || m_nav_permission || m_resume_until>0 ||
     !m_backend->acquisitionAllowed(m_vehicle,m_now)) return false;
  ++m_authority_revision;
  m_navigation.reset();
  m_hold.reset();
  m_px4={};
  m_px4.id=id;
  m_nav_permission=true;
  m_acquire_from=px4::hold;
  m_acquire_until=m_now+px4::acquisition_lifetime;
  m_reason="px4_waiting_for_post_acquisition_inputs";
  return true;
}

void Runtime::observePx4Mode(double at)
{
  if(!m_acquire_until || m_vehicle.mode!=px4::offboard) return;
  if(m_px4.mode_sent_at<0) {
    revokeNavigation("px4_offboard_not_requested_by_bridge");
    return;
  }
  if(at<=m_px4.mode_sent_at) return;
  if(px4OffboardIntentRequired() && !px4ObservedMode(px4::offboard,m_px4.mode_sent_at)) return;
  m_px4.confirmed=true;
  retireAcquisition();
  m_reason="px4_waiting_for_post_confirmation_inputs";
}

void Runtime::refreshPx4()
{
  if(!px4Profile() || m_shadow || (!m_nav_permission && m_resume_until<=0)) return;
  if(m_nav_permission && !m_px4.confirmed && m_now>=m_acquire_until) {
    revokeNavigation("px4_offboard_acquisition_expired");return;
  }
  if(m_nav_permission && !m_px4.confirmed && m_px4.mode_sent_at<0 && !px4NativeHoldReady()) {
    revokeNavigation("px4_native_hold_evidence_lost");return;
  }
  if(px4OffboardIntentRequired() && m_px4.confirmed && !px4ObservedMode(px4::offboard,m_px4.mode_sent_at)) {
    revokeNavigation(px4RoverProfile()?"rover_offboard_intent_evidence_lost":"px4_offboard_intent_evidence_lost");
    return;
  }
  if((m_nav_permission && !navigationModeAllowed()) ||
     (m_resume_until>0 && m_vehicle.mode!=px4::offboard)) {
    revokeNavigation("external_mode_change");return;
  }
  if(!flightReady(false)) {
    revokeNavigation(!m_vehicle.estimator_valid || !px4::fresh(m_vehicle.estimator_at,m_now,px4::estimator_age)?
      "px4_estimator_stale_or_invalid":!px4LocalReady()?"px4_local_position_stale_or_invalid":
      "px4_observation_continuity_lost");
    return;
  }
  if(m_resume_until>0 && m_now>=m_resume_until) {
    revokeNavigation("handoff_window_expired");return;
  }
  if(m_nav_permission && m_px4.intent_until>0 && m_now>=m_px4.intent_until) {
    revokeNavigation("px4_helm_intent_expired");return;
  }
  if(m_px4.last_setpoint>=0 && m_now-m_px4.last_setpoint>px4::max_stream_gap)
    revokeNavigation("px4_setpoint_continuity_lost");
}

bool Runtime::suspendPx4(double until)
{
  if(!m_px4.confirmed || !m_nav_permission || !m_px4.motion_started || !flightReady(true))
    return false;
  ++m_authority_revision;
  if(m_navigation) { ++m_cancelled; m_navigation.reset(); }
  m_nav_permission=false;
  m_px4.anchor=m_px4_local->point;
  m_px4.intent_until=0;
  m_resume_until=until;
  m_reason="normal_handoff";
  return true;
}

bool Runtime::resumePx4()
{
  if(!m_px4.confirmed || m_nav_permission || m_now>=m_resume_until || !flightReady(true))
    return false;
  ++m_authority_revision;
  m_resume_until=0;
  m_nav_permission=true;
  // The adapter submits a new post-pause tuple on the same owner iteration.
  m_reason="resumed_after_normal_handoff";
  return true;
}

bool Runtime::px4ModeSendEligible(const CommandRequest& r) const
{
  return m_nav_permission && px4NativeHoldReady() && !m_px4.confirmed &&
    m_px4.mode_sent_at<0 && m_px4.first_setpoint>=0 && m_px4.last_setpoint>=0 &&
    m_px4.prestream_count>=8 &&
    m_px4.last_setpoint-m_px4.first_setpoint>=px4::prestream_duration &&
    m_now-m_px4.last_setpoint<=px4::max_stream_gap && m_now<m_px4.intent_until &&
    m_now<m_acquire_until && px4LocalReady() && r.id=="acquire-"+m_px4.id &&
    r.command==MAV_CMD_DO_SET_MODE;
}

void Runtime::trackPx4Mode(bool accepted)
{
  if(!m_nav_permission) return;
  if(accepted) {
    m_px4.mode_sent_at=m_now;
    m_reason="px4_waiting_for_offboard_confirmation";
  } else revokeNavigation("px4_offboard_mode_handoff_failed");
}

void Runtime::flushPx4Hold()
{
  const bool paused=m_resume_until>m_now;
  const double until=paused?m_resume_until:m_px4.intent_until;
  if((!paused && (!m_nav_permission || m_px4.motion_started || !m_px4.anchor)) ||
     (!paused && m_px4.confirmed && m_navigation && m_now-m_last_nav>=.1) ||
     until<=m_now || !m_px4.anchor ||
     (m_px4.last_setpoint>=0 && m_now-m_px4.last_setpoint<px4::setpoint_period)) return;
  const auto authority=m_authority_revision;
  const auto check=[this,authority,paused,until] {
    return m_authority_revision==authority && m_now<until && px4LocalReady() &&
      (m_px4.last_setpoint<0 || m_now-m_px4.last_setpoint<=px4::max_stream_gap) &&
      (!m_px4.confirmed || !px4OffboardIntentRequired() || px4ObservedMode(px4::offboard,m_px4.mode_sent_at)) &&
      (paused?(m_resume_until==until && m_px4.confirmed && flightReady(true)):
        (m_nav_permission && m_px4.intent_until==until && !m_px4.motion_started &&
         navigationModeAllowed() && flightReady(false) &&
         (m_px4.confirmed || m_px4.mode_sent_at>=0 || px4NativeHoldReady())));
  };
  if(!check()) return;
  auto point=*m_px4.anchor;
  if(px4ForwardFlight()) {
    // Use a forward position at the captured altitude. PX4 1.17's
    // MAVLink receiver does not interpret the legacy loiter mask.
    const double north=m_vehicle.north_velocity*px4::fixed_wing_lookahead_s;
    const double east=m_vehicle.east_velocity*px4::fixed_wing_lookahead_s;
    if(!px4::insideFixedWingArea(m_vehicle,north,east)) {
      revokeNavigation("px4_fixed_wing_continuation_out_of_envelope");
      return;
    }
    point.north=static_cast<float>(m_px4_local->point.north+north);
    point.east=static_cast<float>(m_px4_local->point.east+east);
  }
  const auto frame=px4RoverProfile()?px4::velocityFrame(0,0,m_now,m_codec):px4::holdFrame(point,m_now,m_codec);
  if(!sendControlFrame(frame,std::min(until,m_now+.2),check)) {
    if(m_authority_revision==authority) revokeNavigation("px4_hold_handoff_failed");
    return;
  }
  if(m_authority_revision!=authority)return;
  m_px4.last_setpoint=m_now;
  ++m_px4_hold_accepted;
  if(!paused && !m_px4.confirmed && m_px4.mode_sent_at<0) {
    if(m_px4.first_setpoint<0) m_px4.first_setpoint=m_now;
    ++m_px4.prestream_count;
    if(m_px4.last_setpoint-m_px4.first_setpoint>=px4::prestream_duration && m_px4.prestream_count>=8) {
      auto setup=m_backend->acquisitionCommand(m_vehicle,m_px4.id,m_now);
      if(!setup) {revokeNavigation("px4_offboard_mode_admission_failed");return;}
      setup->expires=std::min({setup->expires,until,m_acquire_until});
      if(!queueCommand(*setup,true)) revokeNavigation("px4_offboard_mode_admission_failed");
    }
  }
}

void Runtime::flushPx4Motion(const NavigationTarget& target,uint64_t authority)
{
  const auto check=[this,&target,authority] {
    const auto* rover=px4RoverPolicy();
    return m_authority_revision==authority && m_nav_permission && m_px4.confirmed &&
      m_px4.last_setpoint>=0 && m_now-m_px4.last_setpoint<=px4::max_stream_gap &&
      m_now<target.expires && flightReady(true) &&
      (!px4OffboardIntentRequired() || px4ObservedMode(px4::offboard,m_px4.mode_sent_at)) &&
      (!px4RoverProfile() || (rover && rover->targetAllowed(target,m_vehicle,m_now)));
  };
  if(!check()) {
    ++m_expired;
    if(px4RoverProfile() && m_authority_revision==authority && m_nav_permission)
      revokeNavigation("rover_motion_target_no_longer_eligible");
    return;
  }
  const auto frame=m_backend->navigationFrame(target,m_vehicle,m_now,m_revision,m_codec);
  if(frame && sendControlFrame(*frame,target.expires,check)) {
    if(m_authority_revision!=authority)return;
    m_px4.last_setpoint=m_now;
    m_px4.motion_started=true;
    m_reason="px4_offboard_motion";
    ++m_nav_accepted;
  } else if(m_authority_revision==authority) revokeNavigation("px4_motion_handoff_failed");
}

void Runtime::appendPx4Status(std::ostream& s) const
{
  if(!px4Profile())return;
  s<<",px4_local_ready="<<px4LocalReady()<<",px4_offboard_confirmed="<<m_px4.confirmed
   <<",px4_native_hold_ready="<<px4NativeHoldReady()<<",px4_reported_mode="<<m_px4_mode.active
   <<",px4_intended_mode="<<m_px4_mode.intended<<",px4_prestream_count="<<m_px4.prestream_count<<",px4_hold_tx_accepted="<<m_px4_hold_accepted;
}
} // namespace mavmoos::runtime
