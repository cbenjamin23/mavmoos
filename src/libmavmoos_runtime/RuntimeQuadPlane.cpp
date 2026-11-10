#include "Runtime.h"
#include "QuadPlanePolicy.h"

namespace mavmoos::runtime {
bool Runtime::quadPlaneProfile() const
{
  return m_expected_type==MAV_TYPE_VTOL_FIXEDROTOR&&m_expected_autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA&&
    m_backend&&m_backend->profile()==quadplane::profile;
}

unsigned Runtime::quadPlanePhase() const
{
  const auto* policy=dynamic_cast<const quadplane::PhasePolicy*>(m_backend);
  return quadPlaneProfile()&&policy?policy->phase(m_now):unsigned(MAV_VTOL_STATE_UNDEFINED);
}

bool Runtime::quadPlaneHoverAdapterReady(bool guided) const
{
  return m_plane.hover_offer_kind==(guided?quadplane::hover_guided_offer:quadplane::hover_hold_offer)&&
    quadplane::fresh(m_plane.hover_offer_at,m_now,quadplane::hover_offer_age);
}

void Runtime::observeQuadPlaneHoverAdapter(const mavlink_message_t& message,double at)
{
  if(message.magic!=MAVLINK_STX||message.len!=5+quadplane::hover_offer_bytes||
     !quadplane::fresh(at,m_now,quadplane::hover_offer_age)||at<=m_plane.hover_offer_at)return;
  mavlink_tunnel_t packet{};
  mavlink_msg_tunnel_decode(&message,&packet);
  if(packet.target_system!=245||packet.target_component!=191||packet.payload_type!=0||
     packet.payload_length!=quadplane::hover_offer_bytes||quadplane::hoverUint(packet.payload,4)!=quadplane::hover_marker||
     (packet.payload[8]!=quadplane::hover_guided_offer&&packet.payload[8]!=quadplane::hover_hold_offer)||
     packet.payload[9]!=1)return;
  const auto token=quadplane::hoverUint(packet.payload+4,4);
  if(token<=m_plane.hover_offer)return;
  m_plane.hover_offer=token;
  // Retire even a well-formed token seen outside stable MC. A delayed replay
  // cannot supply the missing stable-phase authorization after the transition.
  m_plane.hover_offer_kind=quadPlanePhase()==MAV_VTOL_STATE_MC?packet.payload[8]:0;
  m_plane.hover_offer_at=at;
}

void Runtime::flushQuadPlaneHoverPoint(const NavigationTarget& n,uint64_t authority)
{
  const auto token=m_plane.hover_offer;
  const auto live=[this,authority,token] {
    return authority==m_authority_revision&&m_nav_permission&&m_plane.guided_confirmed&&!m_plane.transition&&
      !m_plane.awaiting&&m_plane.heartbeat_started&&planeProfileReady()&&flightReady(true)&&
      quadPlaneHoverAdapterReady(true)&&token==m_plane.hover_offer&&token!=m_plane.hover_attempt&&
      m_plane.hover_offer_at>m_plane.acquire_sent_at;
  };
  if(!live()||!quadplane::hoverTarget(n)||!sampleClock()||!live()||m_now>=n.expires)return;

  // A stock helm decision supplies course, speed and height. Translate it to
  // short position steps because the FC's hover controller owns velocity.
  constexpr double radians=3.14159265358979323846/180.;
  constexpr double metres_per_degree=6371000.*radians;
  const double distance=n.speed*1.7;
  const double course=n.course*radians;
  const auto lat=static_cast<int32_t>(std::lround(1e7*(m_vehicle.latitude+distance*std::cos(course)/metres_per_degree)));
  const auto lon=static_cast<int32_t>(std::lround(1e7*std::remainder(m_vehicle.longitude+
    distance*std::sin(course)/(metres_per_degree*std::cos(m_vehicle.latitude*radians)),360.)));
  const auto point=n.speed==0&&m_plane.hover_hold?*m_plane.hover_hold:std::pair{lat,lon};
  const double relative=std::clamp(n.height,m_vehicle.relative_height-1.25,m_vehicle.relative_height+1.25);
  const auto altitude_cm=static_cast<int32_t>(std::lround((m_vehicle.home_msl+relative)*100));

  // The receiver starts this budget at its challenge generation, earlier
  // than our receipt. Delays cannot renew the original helm decision.
  const auto ttl=static_cast<uint16_t>(std::floor(1000*std::min(quadplane::hover_max_lease,n.expires-m_now)));
  const double cutoff=std::min(n.expires,m_plane.hover_offer_at+ttl*.001-.05);
  if(m_now>=cutoff)return;
  mavlink_tunnel_t target{};
  target.target_system=1;target.target_component=1;
  target.payload_length=quadplane::hover_target_bytes;
  quadplane::hoverBytes(target.payload,quadplane::hover_marker,4);
  quadplane::hoverBytes(target.payload+4,token,4);
  target.payload[8]=quadplane::hover_target;
  quadplane::hoverBytes(target.payload+9,ttl,2);
  quadplane::hoverBytes(target.payload+11,static_cast<uint32_t>(point.first),4);
  quadplane::hoverBytes(target.payload+15,static_cast<uint32_t>(point.second),4);
  quadplane::hoverBytes(target.payload+19,static_cast<uint32_t>(altitude_cm),4);
  target.payload[23]=1;
  mavlink_message_t msg{};
  mavlink_msg_tunnel_encode_status(245,191,&m_codec,&msg,&target);
  if(!send(msg,cutoff,[this,live] {return !autoRecoveryBlocksEffects()&&live();})) {
    if(authority==m_authority_revision)revokeNavigation("quadplane_hover_socket_failed");
    return;
  }
  if(authority!=m_authority_revision)return;
  m_plane.hover_attempt=token;
  m_plane.hover_lease_until=cutoff;
  if(n.speed==0)m_plane.hover_hold=point;
  else m_plane.hover_hold.reset();
  ++m_nav_accepted;
  m_plane.awaiting=PlaneState::Target{point.first,point.second,static_cast<float>(relative),m_now,
    cutoff,altitude_cm*.01,0};
}

bool Runtime::quadPlaneMissionEntry(const std::vector<MissionItem>& items) const
{
  return m_auto_recovery&&m_vehicle.mode==quadplane::qloiter&&m_vehicle.armed&&
    quadPlanePhase()==MAV_VTOL_STATE_MC&&quadplane::stoppedOnGround(m_vehicle,m_now)&&
    quadplane::validPlan(items,m_vehicle);
}

void Runtime::observeQuadPlaneCursor(const mavlink_message_t& message,double at)
{
  if(message.sysid!=1||message.compid!=1||message.magic!=MAVLINK_STX||message.len<3||at<m_quadplane_cursor_at||
     !quadplane::fresh(at,m_now,2))return;
  mavlink_mission_current_t cursor{};
  mavlink_msg_mission_current_decode(&message,&cursor);
  if(cursor.total!=quadplane::plan_items-1) {
    m_quadplane_cursor=0;m_quadplane_cursor_at=at;
    if(m_stored_mission.valid)invalidateStoredMissionProof("quadplane_mission_count_changed");
    return;
  }
  if(message.len>=5) {m_quadplane_cursor=cursor.seq;m_quadplane_cursor_at=at;}
}

bool Runtime::quadPlaneGroundHandback(double after) const
{
  return !m_vehicle.armed&&m_vehicle.mode==quadplane::qloiter&&quadPlanePhase()==MAV_VTOL_STATE_MC&&
    m_vehicle.heartbeat_at>after&&m_vehicle.landed_at>after&&quadplane::stoppedOnGround(m_vehicle,m_now);
}

bool Runtime::quadPlaneHomeBound() const
{
  const auto& home=m_execution->home;
  return std::abs(m_vehicle.home_latitude-home.home_latitude)<=1e-7&&
    std::abs(m_vehicle.home_longitude-home.home_longitude)<=1e-7&&
    std::abs(m_vehicle.home_msl-home.home_msl)<=.05;
}

bool Runtime::quadPlaneAtLandingPoint() const
{
  const auto& items=m_stored_mission.items;
  const auto& lease=*m_execution;
  // Native disarm or exit from AUTO can reset the cursor. Preserve the bound
  // landing only after an accepted ground-only handoff, with live geometry.
  const bool landing_bound=lease.quadplane_disarm_sent||lease.quadplane_handback_at>=0||
    (m_quadplane_cursor==quadplane::land_sequence&&m_quadplane_cursor_at>lease.started&&
     quadplane::fresh(m_quadplane_cursor_at,m_now,2));
  if(items.size()!=quadplane::plan_items||quadPlanePhase()!=MAV_VTOL_STATE_MC||
     !quadplane::stoppedOnGround(m_vehicle,m_now)||!landing_bound||
     std::abs(m_vehicle.msl_height-lease.home.home_msl)>4)return false;
  const auto& landing=items.back();
  return quadplane::separation(m_vehicle.latitude,m_vehicle.longitude,landing.x,landing.y)<=20;
}

bool Runtime::quadPlaneCommandEligible(const CommandRequest& r) const
{
  const auto& lease=*m_execution;
  if(!r.abort_id.empty())return !lease.quadplane_abort_attempted&&!lease.quadplane_left_auto&&
    lease.session_epoch==m_connection_revision&&lease.auto_seen&&m_vehicle.mode==quadplane::auto_mode&&quadPlaneHomeBound();
  if(r.command==MAV_CMD_DO_SET_MISSION_CURRENT)return !lease.start_handoff&&lease.quadplane_selected_at<0;
  if(r.command==MAV_CMD_MISSION_START)return !lease.quadplane_start_sent&&lease.quadplane_selected_at>=0&&
    m_quadplane_cursor==quadplane::takeoff_sequence&&m_quadplane_cursor_at>lease.quadplane_selected_at&&
    quadplane::fresh(m_quadplane_cursor_at,m_now,2);
  return lease.auto_seen&&lease.quadplane_handback_at<0&&
    (r.command!=MAV_CMD_COMPONENT_ARM_DISARM||!lease.quadplane_disarm_sent)&&quadPlaneAtLandingPoint();
}

void Runtime::trackQuadPlaneHandoff(const CommandRequest& r,bool accepted)
{
  if(!m_execution||r.execution_id!=m_execution->id)return;
  if(!r.abort_id.empty()) {
    if(r.abort_id==m_execution->abort_id)m_execution->quadplane_abort_attempted=true;
    return;
  }
  if(!accepted) {quarantineMissionExecution("quadplane_handoff_failed");return;}
  if(r.command==MAV_CMD_DO_SET_MISSION_CURRENT)m_execution->quadplane_selected_at=m_now;
  if(r.command==MAV_CMD_MISSION_START)m_execution->quadplane_start_sent=true;
  if(r.command==MAV_CMD_COMPONENT_ARM_DISARM)m_execution->quadplane_disarm_sent=true;
  if(quadplane::modeCommand(r,quadplane::qloiter))m_execution->quadplane_handback_at=m_now;
}

void Runtime::refreshQuadPlaneExecution()
{
  auto& lease=*m_execution;
  if(lease.auto_seen&&m_vehicle.heartbeat_valid&&quadplane::fresh(m_vehicle.heartbeat_at,m_now,2)&&
     m_vehicle.mode!=quadplane::auto_mode)lease.quadplane_left_auto=true;
  if(lease.quarantined)return;
  if(!m_configuration_valid||lease.session_epoch!=m_connection_revision||m_now>=lease.expires||
     !m_stored_mission.valid||m_stored_mission.revision!=lease.proof_revision||
     !quadplane::observed(m_vehicle,m_now)||quadPlanePhase()==MAV_VTOL_STATE_UNDEFINED) {
    quarantineMissionExecution("quadplane_observation_or_plan_lost");return;
  }
  // Plane can resurvey HOME as it touches down or disarms. Retain only the
  // ground handback at the original bound landing point and MSL envelope.
  if(!quadPlaneHomeBound()&&!(lease.auto_seen&&quadPlaneAtLandingPoint())) {
    quarantineMissionExecution("quadplane_home_changed");return;
  }
  if(m_vehicle.mode==quadplane::auto_mode) {
    if(!lease.start_handoff)quarantineMissionExecution("quadplane_unowned_auto");
    else lease.auto_seen=true;
  } else if(!(m_vehicle.mode==quadplane::qloiter&&!lease.auto_seen&&m_vehicle.armed)&&
            !(lease.quadplane_handback_at>=0&&m_vehicle.mode==quadplane::qloiter&&!m_vehicle.armed)) {
    quarantineMissionExecution("quadplane_external_mode_change");
  }
}
} // namespace mavmoos::runtime
