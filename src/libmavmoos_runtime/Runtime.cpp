#include "Runtime.h"
#include "PlaneOperationPolicy.h"
#include "ControlReplayJournal.h"
#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace mavmoos::runtime {
Runtime::Runtime(Publish p,Send s,bool simulation,bool shadow,std::function<double()> clock,
                 unsigned expected_type,unsigned expected_autopilot,bool defer_startup)
  :m_publish(std::move(p)),m_send(std::move(s)),m_clock(std::move(clock)),m_simulation(simulation),m_shadow(shadow),
   m_startup_blocked(defer_startup),
   m_expected_type(expected_type),m_expected_autopilot(expected_autopilot),
   m_firmware(expected_type,expected_autopilot)
{
  if(!m_clock) { m_firmware.reset(m_now,"host_started"); return; }
  const double initial=m_clock();
  if(!std::isfinite(initial)||initial<0) {
    // An unqualified initial domain cannot establish a trustworthy session.
    // Keep Services time representable and require a fresh Runtime instance.
    m_configuration_valid=false;m_reason="invalid_initial_clock";
    m_stream_requests.reset(std::numeric_limits<double>::quiet_NaN(),m_reason);
    m_firmware.reset(std::numeric_limits<double>::quiet_NaN(),m_reason);
    return;
  }
  m_now=initial;
  m_firmware.reset(m_now,"host_started");
}
Runtime::~Runtime()
{
  // Destroy components while their Services state remains alive.
  // Detach before reverse teardown: later observer destructors may use Services.
  m_backend=nullptr; m_configuration_valid=false; m_simulation=false;m_configuration_callbacks=true;
  m_host.clear();
}
void Runtime::setBoundedSend(BoundedSend send) { m_bounded_send=std::move(send); }
bool Runtime::load(const std::string& path,const std::string& expected_name)
{
  if(!m_configuration_valid || m_configuration_callbacks) return false;
  // Checked app startup seals the composition. Legacy direct Runtime callers
  // still establish their clock or observations before loading providers.
  if(m_module_configuration_complete) {
    m_configuration_error="plugin composition is startup-only";return false;
  }
  if(!m_startup_blocked) m_configuration_effect_attempted=false;
  m_configuration_callbacks=true;
  bool loaded=false;
  try {
    loaded=m_host.load(path,*this,expected_name);
    if(loaded) m_backend=m_host.controlBackend();
  } catch(const std::exception& error) {
    m_configuration_error=error.what();invalidateConfiguration();return false;
  } catch(...) {
    m_configuration_error="module loading threw an unknown exception";invalidateConfiguration();return false;
  }
  m_configuration_callbacks=false;
  if(m_configuration_effect_attempted) {
    m_configuration_error="module attempted an effect during construction or rejected-load teardown";
    invalidateConfiguration();return false;
  }
  if(!loaded) return false;
  m_configuration_error.clear();
  m_stream_requests.reset(m_now,"module_loaded"); return true;
}
bool Runtime::configurationEffectAttempt()
{
  if(!m_configuration_callbacks&&!m_startup_blocked) return false;
  m_configuration_effect_attempted=true;return true;
}
void Runtime::invalidateConfiguration()
{
  m_configuration_callbacks=false;m_startup_blocked=false;
  m_backend=nullptr;m_configuration_valid=false;++m_connection_revision;
  m_nav_permission=false;m_shadow_grant=false;m_resume_until=0;
  m_navigation.reset();m_hold.reset();m_commands.clear();m_command_leases.clear();m_parameters.clear();
  m_command_handoff.reset();m_mission_handoff.reset();++m_authority_revision;
  m_missions.clear();m_mission_operations.clear();m_mission_mutations.clear();
  resetFlightLog();
  resetFileTransfer();
  resetPayload();
  resetExternalNavigation();
  invalidateStoredMissionProof("configuration_failed");
  m_mission_receipt.reset();m_mission_receipt_accepted=false;
  m_stream_requests.reset(m_now,"configuration_failed");
  // Destructors can publish diagnostics but cannot mutate host state or queue work.
  m_configuration_callbacks=true;m_host.clear();m_configuration_callbacks=false;
  m_reason="plugin_configuration_failed";
}
void Runtime::configure(const std::string& key,const std::string& value)
{
  if(!m_configuration_valid || m_configuration_callbacks)
    throw std::runtime_error("plugin configuration is invalid or reentrant; restart required");
  if((key=="vehicle_profile" || key=="copter_frame") && (m_operational_started||m_module_configuration_complete))
    throw std::runtime_error("vehicle profile and frame are startup-only; restart required");
  try {
    if(!m_startup_blocked) m_configuration_effect_attempted=false;
    m_configuration_callbacks=true;
    m_host.configure(key,value);m_backend=m_host.controlBackend();m_configuration_callbacks=false;
    if(m_configuration_effect_attempted) throw std::invalid_argument("module attempted an effect during configuration");
    m_stream_requests.reset(m_now,"configuration_changed");
  } catch(const std::exception& error) {
    m_configuration_error=error.what();invalidateConfiguration();throw;
  } catch(...) {
    m_configuration_error="plugin configuration threw an unknown exception";invalidateConfiguration();throw;
  }
}
void Runtime::configureModules(const std::map<std::string,std::vector<ModuleSetting>>& batches)
{
  if(!m_configuration_valid || m_configuration_callbacks || m_operational_started || m_module_configuration_complete)
    throw std::runtime_error("checked module configuration is startup-only; restart required");
  try {
    if(!m_startup_blocked) m_configuration_effect_attempted=false;
    m_configuration_callbacks=true;
    if(!m_host.configureModules(batches)) throw std::invalid_argument(m_host.lastError());
    m_backend=m_host.controlBackend();m_configuration_callbacks=false;
    if(m_configuration_effect_attempted) throw std::invalid_argument("module attempted an effect during checked configuration");
    m_module_configuration_complete=true;
    m_stream_requests.reset(m_now,"module_configuration_complete");
  } catch(const std::exception& error) {
    m_configuration_error=error.what();invalidateConfiguration();throw;
  } catch(...) {
    m_configuration_error="module configuration threw an unknown exception";invalidateConfiguration();throw;
  }
}
void Runtime::finishStartup()
{
  if(!m_startup_blocked||!m_configuration_valid||m_configuration_callbacks||!m_module_configuration_complete)
    throw std::runtime_error("startup configuration has not completed");
  if(m_configuration_effect_attempted) {
    m_configuration_error="module attempted an effect during startup validation";
    invalidateConfiguration();throw std::invalid_argument(m_configuration_error);
  }
  m_startup_blocked=false;
}
std::vector<std::string> Runtime::subscriptions() const
{
  auto result=m_host.subscriptions();
  if(m_auto_recovery) result.push_back("MAVMOOS_AUTO_RECOVERY_REQUEST");
  return result;
}
void Runtime::publish(const std::string& k,const std::string& v) { m_publish(k,v,0,false); }
void Runtime::publish(const std::string& k,double v) { m_publish(k,"",v,true); }
void Runtime::mail(const Mail& mail,double original_publication)
{
  if(!checkpoint(mail.received_at)) return;
  if(m_auto_recovery) {
    if(mail.key=="MAVMOOS_AUTO_RECOVERY_REQUEST") { handleAutoRecoveryRequest(mail); return; }
    if(!std::isfinite(mail.published_at) || mail.published_at<=m_auto_recovery->mail_after) return;
  }
  if(m_control_replay_enabled && (mail.key=="DEAD_MAN_POST_INTERRUPT" || mail.key=="MAVMOOS_SUB_HANDBACK") &&
     std::isfinite(original_publication) && original_publication>0) {
    std::ostringstream event;
    event<<':'<<mail.key<<':'<<mail.source<<':'<<std::hexfloat<<original_publication;
    m_replay_event=event.str();
  }
  m_host.dispatchMail(mail);
  m_replay_event.clear();
}
bool Runtime::fresh(double at,double limit) const { return at>0 && m_now>=at && m_now-at<=limit; }
bool Runtime::sampleClock()
{
  if(!m_clock) return true;
  const double sampled=m_clock();
  if(!std::isfinite(sampled) || sampled<m_now) { disconnect(-1,"clock_fault"); return false; }
  m_now=sampled; return true;
}
void Runtime::refreshValidity()
{
  m_vehicle.heartbeat_valid=m_vehicle.heartbeat_valid && fresh(m_vehicle.heartbeat_at,2);
  m_vehicle.position_valid=m_vehicle.position_valid && fresh(m_vehicle.position_at,0.5);
  m_vehicle.landed_valid=m_vehicle.landed_valid && fresh(m_vehicle.landed_at,2);
  m_vehicle.estimator_valid=m_vehicle.estimator_valid && fresh(m_vehicle.estimator_at,3);
  m_vehicle.estimator_horizontal_valid=m_vehicle.estimator_horizontal_valid && fresh(m_vehicle.estimator_at,3);
}
void Runtime::message(const mavlink_message_t& msg,double at)
{
  message(msg,at,at,"READ_ACQUISITION");
}
void Runtime::message(const mavlink_message_t& msg,double at,double read_at,const std::string& age_basis)
{
  if(!std::isfinite(at) || !std::isfinite(read_at) || at<0 || at>read_at || !checkpoint(read_at)) return;
  if((px4Profile() || arduRoverMissionPolicy() || quadPlaneHelmProfile()) && !m_shadow && m_simulation &&
     (msg.sysid!=1 || msg.compid!=1)) return;
  switch(msg.msgid) {
  case MAVLINK_MSG_ID_HEARTBEAT: {
    const bool fixed_source=m_auto_recovery || planeProfile() || subProfile() ||
      (m_simulation && !m_shadow && guidedHandoffRequired());
    if(fixed_source && (msg.sysid!=1 || msg.compid!=1)) return;
    mavlink_heartbeat_t p{}; mavlink_msg_heartbeat_decode(&msg,&p);
    if(p.type!=m_expected_type || p.autopilot!=m_expected_autopilot ||
       (m_backend&&!m_backend->identityCompatible(p.type,p.autopilot)) ||
       ((px4Profile() || subProfile() || quadPlaneHelmProfile()) && !(p.base_mode&MAV_MODE_FLAG_CUSTOM_MODE_ENABLED))) {
      disconnect(); break;
    }
    if(m_had_connection && m_now-m_vehicle.heartbeat_at>2) disconnect();
    m_had_connection=true; m_vehicle.heartbeat_valid=true; m_vehicle.heartbeat_at=at;
    m_vehicle.type=p.type; m_vehicle.autopilot=p.autopilot;
    m_vehicle.mode=p.custom_mode; m_vehicle.armed=(p.base_mode&MAV_MODE_FLAG_SAFETY_ARMED)!=0;
    observeAutoRecoveryHeartbeat(msg);
    if(m_nav_permission) {
      if(!navigationModeAllowed()) revokeNavigation("external_mode_change");
      else if(m_acquire_until && m_backend->activeModeAllowed(m_vehicle,false)) {
        if(px4Profile()) observePx4Mode(at);
        else if(guidedHandoffRequired()) {
          if(m_guided_acquire_sent_at<0)
            revokeNavigation(subProfile()?"unrequested_sub_guided":"unrequested_copter_guided");
          else if(at>m_guided_acquire_sent_at && fresh(at,2)) retireAcquisition();
        }
        else if(!planeGuidanceProfile()) retireAcquisition();
        else if(m_plane.acquire_sent_at<0) revokeNavigation("external_mode_change");
        else if(at>m_plane.acquire_sent_at && !quadPlaneHoverProfile()) {
          m_plane.guided_confirmed=true;
          retireAcquisition();
        }
      }
    }
    if(m_hold&&!flightReady(true)) m_hold.reset();
    break;
  }
  case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
    mavlink_global_position_int_t p{}; mavlink_msg_global_position_int_decode(&msg,&p);
    if((px4FixedWingProfile() || px4VtolProfile() || px4RoverProfile()) &&
       !acceptPx4OrderedPosition(msg,at,p.time_boot_ms)) break;
    m_vehicle.position_at=at;
    m_vehicle.position_read_at=read_at; m_vehicle.position_time_basis=age_basis;
    m_vehicle.position_valid=std::abs(double(p.lat))<=9e8 && std::abs(double(p.lon))<=18e8;
    m_vehicle.latitude=p.lat/1e7; m_vehicle.longitude=p.lon/1e7;
    m_vehicle.msl_height=p.alt/1000.; m_vehicle.relative_height=p.relative_alt/1000.;
    m_vehicle.north_velocity=p.vx/100.; m_vehicle.east_velocity=p.vy/100.; m_vehicle.down_velocity=p.vz/100.;
    m_vehicle.heading=p.hdg>35999?NAN:p.hdg/100.; break;
  }
  case MAVLINK_MSG_ID_EXTENDED_SYS_STATE: {
    mavlink_extended_sys_state_t p{}; mavlink_msg_extended_sys_state_decode(&msg,&p);
    m_vehicle.landed_at=at; m_vehicle.landed_state=p.landed_state;
    m_vehicle.landed_valid=p.landed_state>=1 && p.landed_state<=4;
    m_vehicle.in_air=p.landed_state==MAV_LANDED_STATE_IN_AIR; break;
  }
  case MAVLINK_MSG_ID_HOME_POSITION: {
    mavlink_home_position_t p{}; mavlink_msg_home_position_decode(&msg,&p);
    const double lat=p.latitude/1e7,lon=p.longitude/1e7,h=p.altitude/1000.;
    if((!m_backend || m_backend->helmProfile().requires_home) && m_vehicle.home_valid && (std::abs(lat-m_vehicle.home_latitude)>1e-7 ||
        std::abs(lon-m_vehicle.home_longitude)>1e-7 ||
        (!px4RoverProfile() && std::abs(h-m_vehicle.home_msl)>0.05)))
      revokeNavigation("home_changed");
    m_vehicle.home_valid=std::abs(lat)<=90 && std::abs(lon)<=180;
    m_vehicle.home_at=at; m_vehicle.home_latitude=lat; m_vehicle.home_longitude=lon; m_vehicle.home_msl=h; break;
  }
  case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
    if(px4Profile())observePx4Local(msg,at);
    break;
  case MAVLINK_MSG_ID_CURRENT_MODE:
    if(px4Profile())observePx4ModeIntent(msg,at);
    break;
  case MAVLINK_MSG_ID_MISSION_CURRENT:
    if(px4RoverProfile()) observePx4RoverMissionCurrent(msg,at);
    if(quadPlaneProfile()) observeQuadPlaneCursor(msg,at);
    if(arduRoverMissionPolicy()) observeArduRoverMissionCurrent(msg,at);
    break;
  case MAVLINK_MSG_ID_ESTIMATOR_STATUS: {
    if(m_expected_autopilot!=MAV_AUTOPILOT_PX4)break;
    mavlink_estimator_status_t p{}; mavlink_msg_estimator_status_decode(&msg,&p);
    if(p.time_usec==m_px4_estimator_usec || at<m_vehicle.estimator_at)break;
    if(m_px4_estimator_usec && p.time_usec<m_px4_estimator_usec) {
      m_vehicle.estimator_valid=m_vehicle.estimator_horizontal_valid=false;
      if(px4Profile() && (m_nav_permission||m_resume_until>0))
        revokeNavigation("px4_estimator_clock_regressed");
    } else {
      m_vehicle.estimator_flags=p.flags;
      m_vehicle.estimator_at=at;
      m_vehicle.estimator_valid=px4::healthy(p);
      m_vehicle.estimator_horizontal_valid=m_vehicle.estimator_valid;
    }
    m_px4_estimator_usec=p.time_usec;
    break;
  }
  case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
    if(m_expected_autopilot==MAV_AUTOPILOT_PX4)break;
    mavlink_ekf_status_report_t p{}; mavlink_msg_ekf_status_report_decode(&msg,&p);
    m_vehicle.estimator_flags=p.flags; m_vehicle.estimator_at=at;
    m_vehicle.estimator_horizontal_valid=std::isfinite(p.pos_horiz_variance) && p.pos_horiz_variance>=0;
    m_vehicle.estimator_valid=(p.flags & 51)==51 && m_vehicle.estimator_horizontal_valid &&
      std::isfinite(p.pos_vert_variance) && p.pos_vert_variance>=0;
    break;
  }
  default: break;
  }
  // A normal handoff retains continuity, not permission to reacquire after
  // takeover or an invalid motion observation that later becomes fresh again.
  if(!m_shadow && !planeGuidanceProfile() && !px4FixedWingProfile() && m_resume_until>m_now && !flightReady(true))
    revokeNavigation("normal_handoff_prerequisite_lost");
  ++m_revision;
  if(msg.msgid==MAVLINK_MSG_ID_COMMAND_ACK) {
    mavlink_command_ack_t ack{}; mavlink_msg_command_ack_decode(&msg,&ack);
    if((ack.target_system!=0 && ack.target_system!=245) ||
       (ack.target_component!=0 && ack.target_component!=191)) return;
  }
  refreshMissionExecution();
  refreshFlightLog();
  refreshFileTransfer();
  // Protocol receipt is the actual read, distinct from the conservative observation floor.
  m_firmware.onMessage(msg,MessageTime{at,read_at,age_basis},m_now);
  observePlaneMessage(msg,at,read_at);
  observePayload(msg,at);
  refreshExternalNavigation();
  refreshPx4();
  m_host.dispatchMessage(msg,MessageTime{at,read_at,age_basis});
  // Fixed-wing airspeed and conventional airframe evidence are provider-owned.
  if((px4FixedWingProfile() || px4VtolProfile()) && (msg.msgid==MAVLINK_MSG_ID_VFR_HUD ||
     msg.msgid==MAVLINK_MSG_ID_EXTENDED_SYS_STATE)) refreshPx4();
  if(px4RoverProfile() && (msg.msgid==MAVLINK_MSG_ID_GPS_RAW_INT ||
     msg.msgid==MAVLINK_MSG_ID_PARAM_VALUE)) {
    refreshPx4();
    refreshMissionExecution();
  }
  if(msg.msgid==MAVLINK_MSG_ID_CURRENT_MODE && px4OffboardIntentRequired()) {
    observePx4Mode(m_vehicle.heartbeat_at);
    refreshPx4();
  }
  // Plane joins the provider's airframe observation after dispatch.
  if(quadPlaneHelmProfile()) refreshPlaneAuthority();
  if(!m_shadow && planeGuidanceProfile() && m_resume_until>m_now && !flightReady(true))
    revokeNavigation("normal_handoff_prerequisite_lost");
}
bool Runtime::subProfile() const
{
  return m_expected_type==MAV_TYPE_SUBMARINE && m_expected_autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA &&
    m_backend && m_backend->profile()=="ardusub";
}

bool Runtime::guidedHandoffRequired() const
{
  return subProfile() || (m_backend && m_backend->profile()=="arducopter");
}

bool Runtime::flightReady(bool motion) const
{
  return !autoRecoveryBlocksEffects() && m_backend && (!px4Profile() || px4LocalReady()) &&
    (!motion || !m_acquire_until || !guidedHandoffRequired()) &&
    m_backend->flightReady(m_vehicle,m_now,motion);
}
bool Runtime::handoff(const uint8_t* bytes,std::size_t size,double expires,const std::function<bool()>& check)
{
  const auto connection=m_connection_revision;
  const auto eligible=[this,connection,expires,check](double at) {
    if(!std::isfinite(at) || at<m_now) { disconnect(-1,"clock_fault");return false; }
    m_now=at;refreshValidity();refreshMissionExecution();refreshFlightLog();refreshFileTransfer();
    return !m_startup_blocked && !m_configuration_callbacks && connection==m_connection_revision && m_configuration_valid && m_simulation && !m_shadow &&
      std::isfinite(expires) && at<expires && (!check || check());
  };
  if(!eligible(m_now))return false;
  ++m_attempts;
  const bool accepted=m_bounded_send?m_bounded_send(bytes,size,expires,eligible):m_send(bytes,size);
  if(accepted)++m_accepted;
  return accepted;
}
bool Runtime::send(const mavlink_message_t& msg,double expires,std::function<bool()> eligible)
{
  if(!m_configuration_valid || !m_simulation || m_shadow) return false;
  uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};
  const auto count=mavlink_msg_to_send_buffer(bytes,&msg);
  // Housekeeping is a fresh local intent; queued operation callers supply
  // their existing absolute phase/target bound instead of receiving a new one.
  return handoff(bytes,count,expires==0?m_now+.25:expires,eligible);
}
bool Runtime::sendControlFrame(const ControlFrame& frame,double expires,std::function<bool()> eligible)
{
  if(autoRecoveryBlocksEffects() || !m_configuration_valid || !m_simulation || m_shadow ||
     frame.size==0 || frame.size>frame.bytes.size()) return false;
  return handoff(frame.bytes.data(),frame.size,expires,[this,eligible] {
    return !autoRecoveryBlocksEffects() && (!eligible || eligible());
  });
}
bool Runtime::sendCommand(const CommandRequest& r)
{
  if(configurationEffectAttempt()) return false;
  if(px4Profile()) {
    if(px4RoverProfile()) {
      if(r.execution_id.empty()) return false;
    } else if(!px4AircraftNativeActions() || controlBusy() || !r.execution_id.empty() || !r.abort_id.empty()) {
      return false;
    }
  }
  if(quadPlaneHelmProfile() && r.execution_id.empty() && r.command==MAV_CMD_DO_SET_MODE) return false;
  if(!planeProfile()) return queueCommand(r,false);
  if(!r.execution_id.empty())return queueCommand(r,false);
  if(plane::armCommand(r)||plane::manualCommand(r))return planeArmAllowed(r)&&queueCommand(r,false);
  if(plane::speedCommand(r))return planeSpeedAllowed(r)&&queueCommand(r,false);
  if(!planeExplicitModeAllowed(r) || !queueCommand(r,false)) return false;
  const double operation_end=r.operation_expires==0?r.expires:r.operation_expires;
  m_plane.transition=PlaneState::Transition{r.id,m_vehicle.mode,static_cast<unsigned>(r.params[1]),
    std::min(m_now+3.,operation_end),-1,false,true,false};
  m_plane.handback="AWAIT_MODE";
  return true;
}
bool Runtime::queueCommand(const CommandRequest& r,bool navigation_acquisition)
{
  expireCommandLeases(); expireMissionOperations();
  if(m_commands.size()>=4 || !commandEligible(r)) return false;
  for(const auto& x:m_commands) if(x.request.id==r.id) return false;
  const double until=r.operation_expires==0?r.expires:r.operation_expires;
  if(!std::isfinite(r.operation_expires) || !std::isfinite(until) || until<r.expires || until<=m_now || until-m_now>120) return false;
  if(m_execution && r.execution_id==m_execution->id &&
     (m_execution->command_ids.count(r.id) || m_execution->command_ids.size()>=64)) return false;
  if(!navigation_acquisition) {
    expireCommandLeases();
    if(m_command_leases.size()>=4 || m_command_leases.count(r.id)) return false;
    // A Plane airspeed adjustment belongs to the live Guided grant. Every
    // other explicit command starts a new authority boundary.
    if(!(planeProfile()&&planeSpeedAllowed(r))) {
      revokeNavigation("external_command_admitted"); m_hold.reset();
    }
    m_command_leases.emplace(r.id,until);
    if(m_execution && r.execution_id==m_execution->id) {
      m_execution->command_ids.insert(r.id);
    }
  }
  m_commands.push_back({r,navigation_acquisition}); return true;
}
void Runtime::commandFinished(const std::string& id)
{
  if(configurationEffectAttempt()) return;
  if(m_command_handoff && !m_command_handoff->navigation_acquisition && m_command_handoff->request.id==id)
    m_command_handoff.reset();
  m_command_leases.erase(id);
  m_commands.erase(std::remove_if(m_commands.begin(),m_commands.end(),[&id](const auto& queued) {
    return !queued.navigation_acquisition && queued.request.id==id;
  }),m_commands.end());
}
void Runtime::cancelCommand(const std::string& id)
{
  if(configurationEffectAttempt()) return;
  if(m_command_handoff && !m_command_handoff->navigation_acquisition && m_command_handoff->request.id==id)
    m_command_handoff.reset();
  m_commands.erase(std::remove_if(m_commands.begin(),m_commands.end(),[&](const auto& queued) {
    return !queued.navigation_acquisition && queued.request.id==id;
  }),m_commands.end());
}
void Runtime::expireCommandLeases()
{
  for(auto it=m_command_leases.begin();it!=m_command_leases.end();)
    if(m_now>=it->second) { const auto id=it->first; ++it; commandFinished(id); } else ++it;
}
bool Runtime::commandEligible(const CommandRequest& r) const
{
  if(autoRecoveryBlocksEffects() || !m_configuration_valid || !m_backend || m_flight_log || !m_mission_mutations.empty() || !m_simulation || m_shadow || !std::isfinite(r.expires) || r.expires<=m_now || r.id.empty() ||
     r.expires-m_now>120 || m_commands.size()>=4 || !m_vehicle.heartbeat_valid || !fresh(m_vehicle.heartbeat_at,2)) return false;
  for(float p:r.params) if(!std::isfinite(p)) return false;
  if(!r.execution_id.empty() || !r.abort_id.empty()) return executionCommandEligible(r);
  if(m_execution) return false;
  return m_backend->commandAllowed(r,m_vehicle,m_now);
}
bool Runtime::acquireNavigation(const std::string& id)
{
  if(configurationEffectAttempt()) return false;
  if(planeGuidanceProfile() && !m_shadow && (m_nav_permission || m_resume_until>0 || m_plane.transition ||
     !m_plane.heartbeat_started || !planeProfileReady() || m_vehicle.mode!=planeHoldMode() ||
     (quadPlaneHoverProfile()&&!quadPlaneHoverAdapterReady(false)))) return false;
  if((px4Profile() || subProfile()) && !m_shadow && (m_nav_permission || m_resume_until>0)) return false;
  m_resume_until=0;
  return acquireNavigationImpl(id);
}
bool Runtime::suspendNavigation(double until)
{
  if(configurationEffectAttempt()) return false;
  expireCommandLeases(); expireMissionOperations();
  if(!m_configuration_valid || !m_backend || !m_mission_mutations.empty() || !std::isfinite(until) || until<=m_now || until-m_now>2 ||
     controlBusy() || (!m_nav_permission && !(m_shadow && m_shadow_grant)) ||
     (planeGuidanceProfile() && !m_shadow && !planeResumeAllowed()) ||
     (!m_shadow && !flightReady(true))) return false;
  if(px4Profile() && !m_shadow) return suspendPx4(until);
  revokeNavigation("normal_handoff");
  m_resume_until=until;
  return true;
}
bool Runtime::resumeNavigation(const std::string& id)
{
  if(configurationEffectAttempt()) return false;
  if(m_resume_until<=m_now) {
    if((planeGuidanceProfile() || px4Profile()) && m_resume_until>0) revokeNavigation("handoff_window_expired");
    else m_resume_until=0;
    return false;
  }
  if(px4Profile() && !m_shadow) return resumePx4();
  if(planeGuidanceProfile() && !m_shadow && !planeResumeAllowed()) return false;
  if(!m_shadow && !flightReady(true)) {
    revokeNavigation("normal_handoff_prerequisite_lost"); return false;
  }
  if(subProfile() && !m_shadow) {
    if(!m_configuration_valid || controlBusy() || m_flight_log || !m_mission_mutations.empty()) return false;
    ++m_authority_revision;
    m_navigation.reset();
    m_hold.reset();
    m_resume_until=0;
    m_nav_permission=true;
    m_reason="resumed_after_normal_handoff";
    return true;
  }
  return acquireNavigationImpl(id);
}
bool Runtime::acquireNavigationImpl(const std::string& id)
{
  expireCommandLeases(); expireMissionOperations();
  if(!m_configuration_valid || !m_backend || controlBusy() || m_flight_log || !m_mission_mutations.empty()) return false;
  if(m_shadow) { ++m_authority_revision; m_navigation.reset(); m_shadow_grant=true; m_resume_until=0; m_hold.reset(); return true; }
  if(!m_simulation || !m_backend->acquisitionAllowed(m_vehicle,m_now)) return false;
  if(px4Profile()) return beginPx4Acquisition(id);
  const auto setup=m_backend->acquisitionCommand(m_vehicle,id,m_now);
  // Replacement acquisition must never inherit an older queued mode request.
  if(m_acquire_until) revokeNavigation("acquisition_replaced");
  if(setup&&!queueCommand(*setup,true)) return false;
  ++m_authority_revision; m_navigation.reset();
  m_hold.reset();
  if(planeGuidanceProfile()) {
    m_plane.intent_until=setup?setup->expires:m_resume_until;
    if(setup) { m_plane.guided_confirmed=false; m_plane.acquire_sent_at=-1; }
  }
  m_resume_until=0;
  m_nav_permission=true;
  m_acquire_from=m_vehicle.mode;
  m_acquire_until=setup?setup->expires:0;
  m_reason="acquired"; return true;
}
bool Runtime::navigationModeAllowed() const
{
  if(m_acquire_until && m_now>=m_acquire_until && guidedHandoffRequired()) return false;
  const bool rover_confirming=px4RoverProfile() && !m_px4.confirmed && m_px4.mode_sent_at>=0 &&
    m_now<m_acquire_until && m_vehicle.mode==px4::offboard;
  return m_backend&&m_vehicle.armed&&(m_backend->activeModeAllowed(m_vehicle,false)||
    rover_confirming ||
    (m_now<m_acquire_until&&m_vehicle.mode==m_acquire_from&&m_backend->activeModeAllowed(m_vehicle,true)));
}
void Runtime::retireAcquisition()
{
  m_acquire_until=0;
  m_guided_acquire_sent_at=-1;
  if(m_command_handoff && m_command_handoff->navigation_acquisition) m_command_handoff.reset();
  m_commands.erase(std::remove_if(m_commands.begin(),m_commands.end(),[](const auto& item) {
    return item.navigation_acquisition;
  }),m_commands.end());
}
bool Runtime::parameterEligible(const ParameterRequest& r) const
{
  if(!m_configuration_valid || !m_simulation || m_shadow || m_files_bench || r.id.empty() || r.id.size()>96 ||
     !std::isfinite(r.expires) || r.expires<=m_now || r.expires-m_now>60 ||
     !m_vehicle.heartbeat_valid || !fresh(m_vehicle.heartbeat_at,2)) return false;
  for(unsigned char c:r.id) if(!std::isalnum(c) && c!='_' && c!='-' && c!='.' && c!='/') return false;
  if(r.action!=ParameterAction::Read && r.action!=ParameterAction::List && r.action!=ParameterAction::Write) return false;
  if(r.action==ParameterAction::List) {
    // Full-list streaming changes this FC's telemetry scheduling. Its explicit
    // legacy fixture has no command/navigation feature that could arm mid-list.
    if(!r.name.empty() || r.index!=-1 || m_vehicle.armed || !m_vehicle.landed_valid ||
       !fresh(m_vehicle.landed_at,2) || m_vehicle.landed_state!=MAV_LANDED_STATE_ON_GROUND ||
       m_nav_permission || m_resume_until>m_now || controlBusy() || m_flight_log) return false;
    return m_host.streamedParameterBenchCompatible();
  }
  if(r.action==ParameterAction::Read && r.name.empty()) return r.index>=0;
  if(r.name.empty() || r.name.size()>16 || r.index!=-1) return false;
  for(unsigned char c:r.name) if(!(c>='A'&&c<='Z') && !(c>='0'&&c<='9') && c!='_') return false;
  if(r.action==ParameterAction::Read) return !planeProfile() || planeProfileReady();
  return !planeProfile() && !controlBusy() && !m_flight_log && m_mission_mutations.empty() && r.name=="SIM_WIND_SPD" && r.type==MAV_PARAM_TYPE_REAL32 && std::isfinite(r.value) &&
    r.value>=0 && r.value<=20 && !m_vehicle.armed && m_vehicle.landed_valid &&
    fresh(m_vehicle.landed_at,2) && m_vehicle.landed_state==MAV_LANDED_STATE_ON_GROUND;
}
bool Runtime::parameter(const ParameterRequest& request)
{
  if(configurationEffectAttempt()) return false;
  if(m_parameters.size()>=4 || !parameterEligible(request)) return false;
  for(const auto& queued:m_parameters) if(queued.id==request.id) return false;
  m_parameters.push_back(request); return true;
}
void Runtime::revokeNavigation(const std::string& reason)
{
  if(configurationEffectAttempt()) return;
  ++m_authority_revision;
  const bool unconfirmed_guided=m_acquire_until>0 && guidedHandoffRequired();
  // A mode request must not survive the permission that created it.
  retireAcquisition();
  const bool held=m_nav_permission;
  const bool plane_owned=held || m_resume_until>0;
  // An extra operator release cannot erase a pending stop or renew its expiry.
  if(held||reason!="operator_release") m_hold.reset();
  m_resume_until=0; m_shadow_grant=false;
  m_nav_permission=false; m_reason=reason;
  if(m_navigation) { ++m_cancelled; m_navigation.reset(); }
  if(px4Profile()) { m_px4={}; return; }
  if(planeGuidanceProfile() && !m_shadow) {
    planPlaneHandback(reason,plane_owned);
    return;
  }
  // Only capture current position while still Guided, using independent fresh state.
  if(held && !unconfirmed_guided && flightReady(true)) {
    m_hold=m_backend->stopFrame(m_vehicle,m_now,m_revision,m_codec);
    if(m_hold) {
      m_hold_until=std::min(m_now+.5,m_vehicle.position_at+.5);
    }
  }
}
bool Runtime::navigation(const NavigationTarget& n)
{
  if(configurationEffectAttempt()) return false;
  if(autoRecoveryBlocksEffects() || !m_configuration_valid || !m_backend || !m_backend->navigationInputAllowed(n) ||
     !std::isfinite(n.expires)||n.expires<=m_now||n.expires>m_now+2.01) return false;
  if(px4Profile() && !m_shadow) {
    if(!m_nav_permission || !flightReady(false) || !navigationModeAllowed()) return false;
    m_px4.intent_until=n.expires;
    if(!m_px4.confirmed) {
      if(!m_px4.anchor)m_px4.anchor=m_px4_local->point;
      return m_acquire_until>m_now;
    }
  }
  if(!m_shadow && m_acquire_until && guidedHandoffRequired())
    return m_nav_permission && m_now<m_acquire_until && flightReady(false) && navigationModeAllowed();
  if(!m_shadow && (!m_nav_permission || !flightReady(true))) return false;
  if(planeGuidanceProfile() && !m_shadow) {
    if(!planeProfileReady() || !m_plane.heartbeat_started || m_plane.transition) return false;
    if(!m_plane.guided_confirmed) return m_acquire_until>m_now;
    m_plane.intent_until=n.expires;
  }
  m_navigation=n; return true;
}
void Runtime::disconnect(double boundary,const std::string& reason)
{
  ++m_connection_revision;
  invalidateStoredMissionProof(reason);
  m_mission_receipt.reset();m_mission_receipt_accepted=false;
  if(m_execution) {m_execution->abort_id.clear();m_execution->abort_expires=0;}
  if(std::isfinite(boundary) && boundary>m_now) m_now=boundary;
  revokeNavigation("connection_lost"); m_hold.reset(); m_commands.clear();
  m_command_handoff.reset(); m_mission_handoff.reset();
  m_parameters.clear(); m_missions.clear(); m_mission_operations.clear(); m_mission_mutations.clear();
  m_command_leases.clear();
  resetFlightLog();
  resetFileTransfer();
  resetPayload();
  resetExternalNavigation();
  m_vehicle={};
  m_plane={};
  m_px4={};m_px4_local.reset();m_px4_global_boot.reset();m_px4_mode={};m_px4_estimator_usec=0;
  m_px4_mission_cursor={};
  m_ardu_rover_cursor={};
  if(m_auto_recovery) {
    m_auto_recovery->observation_after=std::max(m_auto_recovery->observation_after,m_now);
    m_auto_recovery->observed_at=-1;
  }
  // Stream setup keeps its numeric fault latch until a valid reset. Firmware
  // identity instead clears at retained reliable time; a later valid owner
  // clock and post-floor heartbeat may start a new read-only discovery epoch.
  m_stream_requests.reset(reason=="clock_fault"?std::numeric_limits<double>::quiet_NaN():m_now,reason);
  m_firmware.reset(m_now,reason);
  m_had_connection=false;
  publishStreamStatus();
  publishFirmwareStatus(true);
  m_host.reset(reason);
}
bool Runtime::checkpoint(double at)
{
  if(m_startup_blocked||m_configuration_callbacks||!m_configuration_valid) return false;
  m_operational_started=true;
  if(!std::isfinite(at) || at<m_now) { disconnect(-1,"clock_fault"); return false; }
  m_now=at; refreshValidity();
  expireCommandLeases(); expireMissionOperations();
  refreshMissionExecution();
  refreshFlightLog();
  refreshFileTransfer();
  // Assess the previous cache before a queued/new packet can hide a local observation gap.
  if(m_had_connection && !m_vehicle.heartbeat_valid) disconnect();
  else if(!px4Profile() && m_nav_permission && !flightReady(false) && !quadPlaneHoverSettling())
    revokeNavigation("observation_continuity_lost");
  else if(!px4Profile() && m_nav_permission && !navigationModeAllowed()) revokeNavigation("external_mode_change");
  else if(!px4Profile() && !m_shadow && m_resume_until>m_now && !flightReady(true))
    revokeNavigation("normal_handoff_prerequisite_lost");
  if(m_hold&&!flightReady(true)) m_hold.reset();
  refreshPlaneAuthority();
  refreshPx4();
  refreshPayload();
  refreshExternalNavigation();
  m_host.inputBoundary();
  m_firmware.inputBoundary(m_now);
  return true;
}
void Runtime::tick(double at)
{
  if(m_startup_blocked||m_configuration_callbacks) return;
  if(!m_configuration_valid) { publish("MAVMOOS_RUNTIME_STATUS",status()); publishFirmwareStatus(); return; }
  if(!checkpoint(at)) return;
  m_host.iterate();
  if(!sampleClock()) return;
  refreshValidity();
  refreshPlaneAuthority();
  refreshPx4();
  publish("MAVMOOS_RUNTIME_STATUS",status());
  publishAutoRecoveryStatus();
  m_stream_requests.advance(m_now);
  publishStreamStatus();
  const bool firmware_ready=m_firmware.ready(m_now,firmwareEffectsAllowed());
  publishFirmwareStatus();
  if(!m_simulation || m_shadow) { m_navigation.reset(); return; }
  // Heartbeat and telemetry requests exist only in explicit isolated simulation mode.
  const bool plane=planeProfile();
  const bool plane_guidance=planeGuidanceProfile();
  if(m_now-m_last_heartbeat>=1 && (!plane_guidance || planeHeartbeatEligible())) {
    mavlink_message_t msg{};
    mavlink_msg_heartbeat_pack_status(245,191,&m_codec,&msg,MAV_TYPE_ONBOARD_CONTROLLER,
      MAV_AUTOPILOT_INVALID,0,0,MAV_STATE_ACTIVE);
    const bool accepted=plane_guidance?send(msg,0,[this]{return planeHeartbeatEligible();}):send(msg);
    if(plane_guidance && accepted) m_plane.heartbeat_started=true;
    m_last_heartbeat=m_now;
  }
  const bool stream_requests_enabled=!autoRecoveryBlocksEffects()||px4RoverProfile()||arduRoverMissionPolicy()||quadPlaneProfile();
  if(stream_requests_enabled && m_vehicle.heartbeat_valid &&
     fresh(m_vehicle.heartbeat_at,2) && !m_stream_requests.started()) {
    m_stream_requests.start(m_host.observationStreams(),m_now);
    publishStreamStatus();
  }
  if(m_hold) {
    const auto hold=*m_hold; const auto until=m_hold_until; const auto authority=m_authority_revision;
    m_hold.reset();
    if(!sampleClock()) return;
    if(authority==m_authority_revision && m_now<until && flightReady(true))
      sendControlFrame(hold,until,[this,authority]{return authority==m_authority_revision&&flightReady(true);});
    else ++m_expired;
  }
  if(plane_guidance) queuePlaneHandback();
  if(px4Profile()) flushPx4Hold();
  if(!m_commands.empty()) {
    const auto queued=m_commands.front(); const auto r=queued.request; m_commands.pop_front();
    const auto command_authority=m_authority_revision;
    m_command_handoff=queued;
    const auto live=[this,r,acquisition=queued.navigation_acquisition] {
      return m_command_handoff && m_command_handoff->request.id==r.id &&
      (!acquisition || (m_nav_permission&&m_now<m_acquire_until&&
        m_vehicle.mode==m_acquire_from&&m_backend&&m_backend->acquisitionAllowed(m_vehicle,m_now))) &&
      commandEligible(r) && (!planeGuidanceProfile() || planeModeHandoffEligible(r,acquisition)) &&
      (!px4Profile() || (acquisition && px4ModeSendEligible(r)) ||
       (px4RoverProfile() && !r.execution_id.empty()) ||
       (!acquisition && px4AircraftNativeActions() && m_command_leases.count(r.id)));};
    const bool eligible=live();
    bool accepted=false;
    if(eligible) {
      mavlink_message_t msg{}; const auto& p=r.params;
      if(plane&&plane::speedCommand(r))
        mavlink_msg_command_int_pack_status(245,191,&m_codec,&msg,1,1,MAV_FRAME_GLOBAL,r.command,0,0,
          p[0],p[1],p[2],p[3],0,0,0);
      else
        mavlink_msg_command_long_pack_status(245,191,&m_codec,&msg,1,1,r.command,0,p[0],p[1],p[2],p[3],p[4],p[5],p[6]);
      if(!sampleClock()) return;
      if(live()) {
        if(m_execution && r.execution_id==m_execution->id && r.abort_id.empty()) {
          m_execution->last_execution_handoff=m_now;
          if(r.command==MAV_CMD_MISSION_START||(plane&&r.command==MAV_CMD_DO_SET_MODE&&p[1]==plane::autoland_mode))
            m_execution->start_handoff=true;
        }
        accepted=send(msg,r.expires,live);
      }
      publish("MAVMOOS_TX_RESULT","id="+r.id+",status="+(accepted?"LOCAL_ACCEPTED":"IO_FAILED_OR_EXPIRED"));
    } else { ++m_expired; publish("MAVMOOS_TX_RESULT","id="+r.id+",status=EXPIRED_OR_REVOKED"); }
    m_command_handoff.reset();
    const bool command_clock_ok=sampleClock();
    if(plane_guidance && command_clock_ok) trackPlaneMode(r,queued.navigation_acquisition,accepted);
    if(plane && command_clock_ok && !r.execution_id.empty())trackPlaneMissionHandoff(r,accepted);
    if(px4Profile() && command_clock_ok && queued.navigation_acquisition) trackPx4Mode(accepted);
    if(px4RoverProfile() && command_clock_ok && !r.execution_id.empty())
      trackPx4RoverMissionHandoff(r,accepted);
    if(quadPlaneProfile() && command_clock_ok && !r.execution_id.empty())
      trackQuadPlaneHandoff(r,accepted);
    if(arduRoverMissionPolicy() && command_clock_ok && !r.execution_id.empty())
      trackArduRoverMissionHandoff(r,accepted);
    if(accepted && command_clock_ok && m_execution && r.execution_id==m_execution->id &&
       m_backend && m_backend->profile()=="arducopter" && r.command==MAV_CMD_MISSION_START)
      m_execution->copter_start_accepted_at=m_now;
    if(command_clock_ok && queued.navigation_acquisition && command_authority==m_authority_revision &&
       m_nav_permission && m_acquire_until && guidedHandoffRequired()) {
      if(accepted) m_guided_acquire_sent_at=m_now;
      else revokeNavigation(subProfile()?"sub_guided_handoff_failed":"copter_guided_handoff_failed");
    }
    if(m_execution && r.execution_id==m_execution->id && r.abort_id==m_execution->abort_id &&
       !r.abort_id.empty() && accepted && command_clock_ok) m_execution->abort_sent_at=m_now;
    m_host.commandSent(r,m_now,accepted);
    if(!command_clock_ok) return;
  }
  if(m_navigation && m_now-m_last_nav>=.1) {
    auto n=*m_navigation; const auto authority=m_authority_revision; m_navigation.reset();
    if(plane_guidance) flushPlanePoint(n,authority);
    else if(px4Profile()) flushPx4Motion(n,authority);
    else if(m_nav_permission && m_now<n.expires && flightReady(true)) {
      const auto frame=m_backend->navigationFrame(n,m_vehicle,m_now,m_revision,m_codec);
      if(frame) {
        if(!sampleClock()) return;
        if(authority==m_authority_revision && m_nav_permission && m_now<n.expires && flightReady(true)) {
          if(sendControlFrame(*frame,n.expires,[this,n,authority]{return authority==m_authority_revision &&
             m_nav_permission && m_now<n.expires && flightReady(true);})) ++m_nav_accepted;
        } else ++m_expired;
      } else ++m_expired;
    } else ++m_expired;
    m_last_nav=m_now;
  }
  // Protocol work shares one slot per tick, including addressed payload commands.
  // Control work keeps priority; adding a feature does not double this budget.
  const unsigned lanes=(m_payload_bench || m_external_navigation_bench)?6:5;
  for(unsigned offset=0;offset<lanes;++offset) {
    const unsigned lane=(m_next_protocol+offset)%lanes;
    const bool stream_ready=lane==2 && stream_requests_enabled &&
      m_stream_requests.ready(m_now);
    const bool plane_profile_read=plane_guidance && lane==1 && planeCanReadProfile();
    const bool ready=lane==0?!m_missions.empty():lane==1?plane_profile_read||!m_parameters.empty():
      lane==2?stream_ready||firmware_ready:lane==3?m_flight_log_pending.has_value():
      lane==4?m_file_pending.has_value():m_payload_bench?payloadWaiting():externalNavigationWaiting();
    if(!ready) continue;
    m_next_protocol=(lane+1)%lanes;
    if(lane==0) flushMission();
    else if(lane==1) { if(plane_profile_read) flushPlaneProfileRead(); else flushParameter(); }
    else if(lane==3) flushFlightLog();
    else if(lane==4) flushFileTransfer();
    else if(lane==5) { if(m_payload_bench) flushPayload(); else flushExternalNavigation(); }
    else {
      // Version discovery and stream setup share this one low-priority lane.
      // Alternation prevents either read/setup family from doubling the budget.
      const bool version=firmware_ready&&(!stream_ready||m_firmware_turn);
      m_firmware_turn=!version;
      if(version) flushFirmwareRequest(); else flushObservationRequest();
    }
    break;
  }
}
bool Runtime::flushParameter()
{
    const auto request=m_parameters.front();m_parameters.pop_front();
    mavlink_message_t msg{};
    if(request.action==ParameterAction::List) {
      mavlink_param_request_list_t p{};p.target_system=1;p.target_component=1;
      mavlink_msg_param_request_list_encode_status(245,191,&m_codec,&msg,&p);
    } else if(request.action==ParameterAction::Read) {
      mavlink_param_request_read_t p{};p.target_system=1;p.target_component=1;p.param_index=request.index;
      std::copy(request.name.begin(),request.name.end(),p.param_id);
      mavlink_msg_param_request_read_encode_status(245,191,&m_codec,&msg,&p);
    } else {
      mavlink_param_set_t p{};p.target_system=1;p.target_component=1;p.param_value=request.value;p.param_type=request.type;
      std::copy(request.name.begin(),request.name.end(),p.param_id);
      mavlink_msg_param_set_encode_status(245,191,&m_codec,&msg,&p);
    }
    if(!sampleClock()) return false;
    const bool accepted=parameterEligible(request)&&send(msg,request.expires,[this,request]{return parameterEligible(request);});
    const bool clock_ok=sampleClock();
    publish("MAVMOOS_PARAM_TX_RESULT","id="+request.id+",status="+(accepted?"LOCAL_ACCEPTED":"NOT_ACCEPTED")+
      ",late="+(m_now>=request.expires?"true":"false")+",clock_valid="+(clock_ok?"true":"false"));
    m_host.parameterSent(request,m_now,accepted);
  return true;
}

std::string Runtime::status() const
{
  std::ostringstream s;
  s<<"mode="<<(m_simulation&&!m_shadow?"SITL_CONTROL":m_shadow?"SHADOW":"RX_ONLY")
   <<",permission="<<(m_nav_permission?"true":"false")<<",reason="<<m_reason
   <<",fc_mode="<<m_vehicle.mode<<",armed="<<m_vehicle.armed<<",in_air="<<m_vehicle.in_air
   <<",heartbeat_valid="<<m_vehicle.heartbeat_valid<<",position_valid="<<m_vehicle.position_valid
   <<",home_valid="<<m_vehicle.home_valid<<",estimator_valid="<<m_vehicle.estimator_valid
   <<",estimator_horizontal_valid="<<m_vehicle.estimator_horizontal_valid
   <<",estimator_flags="<<m_vehicle.estimator_flags
   <<",estimator_age_s="<<(m_vehicle.estimator_at>0?m_now-m_vehicle.estimator_at:-1)
   <<",landed_valid="<<m_vehicle.landed_valid<<",tx_attempts="<<m_attempts<<",tx_accepted="<<m_accepted
   <<",expired="<<m_expired<<",cancelled="<<m_cancelled;
  s<<",parameter_queue="<<m_parameters.size()<<",mission_queue="<<m_missions.size()
   <<",mission_operations="<<m_mission_operations.size()<<",mission_mutations="<<m_mission_mutations.size()
   <<",flight_log_lease="<<bool(m_flight_log)<<",file_transfer_lease="<<bool(m_file_transfer);
  s<<",command_leases="<<m_command_leases.size()<<",resume_permit="<<(m_resume_until>m_now?"true":"false");
  s<<",mission_execution_held="<<(m_execution?"true":"false")
   <<",mission_execution_quarantined="<<(m_execution&&m_execution->quarantined?"true":"false");
  s<<",nav_tx_accepted="<<m_nav_accepted;
  s<<",control_profile="<<(m_backend?m_backend->profile():"none");
  s<<",configuration_valid="<<(m_configuration_valid?"true":"false");
  appendPlaneStatus(s);
  appendPx4Status(s);
  if(m_external_navigation_bench)
    s<<",external_nav_run="<<(m_external_lease?m_external_lease->run_id:0)
     <<",external_nav_tx_accepted="<<m_external_accepted<<",external_nav_replaced="<<m_external_replaced;
  if(m_auto_recovery) s<<",auto_recovery_state="<<autoRecoveryState()
    <<",auto_recovery_blocked="<<(autoRecoveryBlocksEffects()?"true":"false");
  return s.str();
}
std::string Runtime::capabilities() const
{
  const std::string mode=m_simulation&&!m_shadow?"SITL_CONTROL":m_shadow?"SHADOW":"RX_ONLY";
  return "{\"schema\":1,\"execution_mode\":\""+mode+"\",\"composition\":"+m_host.catalog()+"}";
}
std::vector<std::string> Runtime::moduleNames() const
{
  std::vector<std::string> names;
  for(const auto* plugin:m_host.plugins()) names.push_back(plugin->name());
  return names;
}
}

namespace mavmoos::runtime {
bool Runtime::projectGlobal(double latitude,double longitude,double& x,double& y) const
{
  if(!m_configuration_valid || !std::isfinite(latitude) || !std::isfinite(longitude) ||
     latitude < -80 || latitude > 84 || std::abs(longitude)>180) return false;
  double east=0,north=0;
  if(!m_host.projectGlobal(latitude,longitude,east,north) || !std::isfinite(east) || !std::isfinite(north)) return false;
  x=east; y=north; return true;
}
}
