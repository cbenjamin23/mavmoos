/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: HelmAdapter.cpp                                 */
/*    DATE: November 3, 2026                                 */
/************************************************************/
#include "ControlComponents.h"
#include "Px4Offboard.h"
#include "QuadPlanePolicy.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace mavmoos::runtime {
namespace {
class HelmAdapter final : public Plugin {
public:
  HelmAdapter(Services& host,const VehicleControlBackend& backend)
    :m_host(host),m_backend(backend),m_started(host.now()) {}
  std::string name() const override { return "helm"; }
  std::vector<std::string> subscriptions() const override {
    std::vector<std::string> result={"DESIRED_HEADING","DESIRED_SPEED",
      "IVPHELM_STATE","IVPHELM_ALLSTOP","MAVMOOS_NAV_CONTROL"};
    if(m_backend.helmProfile().requires_height) result.push_back(verticalKey());
    return result;
  }
  void configure(const std::string& key,const std::string& value) override {
    if(key=="helm_source") { if(value.empty()) throw std::invalid_argument("empty helm source"); m_source=value; }
    else if(key=="shadow_mode") {
      if(value!="true" && value!="false") throw std::invalid_argument("shadow_mode must be true/false");
      m_shadow=value=="true";
    } else if(key=="facing_policy") {
      if(value=="course") m_facing=FacingPolicy::Course;
      else if(value=="keep") m_facing=FacingPolicy::Keep;
      else if(value=="heading") m_facing=FacingPolicy::Heading;
      else throw std::invalid_argument("unsupported facing policy");
    } else if(key=="facing_heading") m_facing_heading=std::fmod(number(value,0,360),360.);
    else if(key=="freshness_timeout") m_timeout=number(value,0.1,10);
    else if(key=="handoff_window") m_window=number(value,0.05,2);
    else if(key=="field_spread") m_spread=number(value,0.01,2);
    else if(key=="nav_position_timeout") m_position_timeout=number(value,0.05,5);
  }
  void onReset(const std::string& reason) override {
    const bool local_handoff=reason=="rover_direct_operation";
    // This local handoff follows admission of a replacement host grant.
    if(!local_handoff) m_host.revokeNavigation("reset:"+reason);
    m_phase=Phase::Latched; m_reason="reset:"+reason;
    m_started=m_host.now(); m_acquired=0; m_paused=0; m_last_send=-1;
    m_state_at=m_started; m_helm_state.clear();
    // FC loss and a local handoff invalidate control/scalars but leave the producer's stop state.
    // A retained clear is never permission: explicit acquisition and new DRIVE/scalars still follow.
    if(reason!="vehicle_connection_lost"&&!local_handoff) {
      m_stop_at=m_started; m_stop="unknown"; m_producer_resync_after_reset=true;
    }
    m_home={}; m_guided_mode_sent=-1; m_guided_boundary_required=false;
    m_post_mode_boundary=false; clearFields(m_started);
  }
  void onMail(const Mail& mail) override {
    const double now=m_host.now();
    if(mail.key=="MAVMOOS_NAV_CONTROL") {
      if(mail.numeric || !validTime(mail,now) || mail.published_at<m_started ||
         mail.published_at<=m_control_at || (isQuadPlane()&&mail.published_at>now)) return;
      if(mail.text!="release" && mail.text!="acquire") return;
      m_control_at=mail.published_at;
      if(mail.text=="release") {
        m_host.revokeNavigation("operator_release");
        m_phase=Phase::Disabled; m_reason="operator_release"; clearFields(now);
      } else if(mail.text=="acquire") {
        if((m_backend.profile()=="arduplane" || isPx4() || guidedBoundaryRequired()) &&
           (m_phase==Phase::Acquiring || m_phase==Phase::Active || m_phase==Phase::Paused)) return;
        // The host owns mode acquisition. The plugin owns producer readiness.
        const bool needs_guided_boundary=guidedBoundaryRequired()||
          (isCopter()&&!m_backend.activeModeAllowed(m_host.vehicle(),false));
        if(m_host.acquireNavigation("helm_acquire_"+std::to_string(++m_request))) {
          m_phase=Phase::Acquiring; m_reason="waiting_for_post_acquisition_inputs";
          m_guided_boundary_required=needs_guided_boundary;
          clearFields(now); m_acquired=now; m_home=m_host.vehicle();m_post_mode_boundary=false;m_guided_mode_sent=-1;
        } else latch("acquisition_rejected");
      }
      return;
    }
    if(mail.source!=m_source) return;
    if(mail.key=="IVPHELM_STATE") {
      if(!mail.numeric && validTime(mail,now) && mail.published_at>=m_state_at) {
        m_helm_state=mail.text; m_state_at=mail.published_at;
        if(m_phase==Phase::Active && mail.text!="DRIVE") latch("helm_not_driving");
      }
      return;
    }
    if(mail.key=="IVPHELM_ALLSTOP") {
      // clear is a retained state, not a heartbeat or scalar freshness renewal.
      if(!mail.numeric && validMetadata(mail,now) && mail.published_at>=m_stop_at) {
        m_stop=mail.text; m_stop_at=mail.published_at; m_producer_resync_after_reset=false;
        if(m_stop!="clear" && m_phase==Phase::Active) {
          if(m_stop=="NothingToDo" || m_stop=="NoGoalBehavior") {
            if(!m_host.suspendNavigation(now+m_window)) { latch("normal_handoff_rejected"); return; }
            m_phase=Phase::Paused; m_paused=now; m_reason="normal_handoff"; clearFields(now);
          } else latch("allstop:"+m_stop);
        }
      }
      return;
    }
    const int index=mail.key=="DESIRED_HEADING" ? 0 : mail.key=="DESIRED_SPEED" ? 1 :
                    mail.key==verticalKey() ? 2 : -1;
    if(index<0 || (index==2 && !m_backend.helmProfile().requires_height) || m_phase==Phase::Disabled || m_phase==Phase::Latched || m_stop!="clear") return;
    auto& field=m_fields[static_cast<std::size_t>(index)];
    if(!validTime(mail,now) || mail.published_at<=m_boundary || mail.published_at<=field.published) return;
    if(!mail.numeric || !std::isfinite(mail.value)) { latch("invalid_helm_input"); return; }
    if(!m_backend.helmFieldAllowed(static_cast<unsigned>(index),mail.value)) { latch("input_out_of_envelope"); return; }
    field={true,mail.value,mail.published_at,mail.published_at+m_timeout};
  }
  void onCommandSent(const CommandRequest& request,double at,bool accepted) override {
    if(m_guided_boundary_required && m_phase==Phase::Acquiring && accepted && request.command==MAV_CMD_DO_SET_MODE &&
       request.id=="acquire-helm_acquire_"+std::to_string(m_request))m_guided_mode_sent=at;
  }
  void onInputBoundary() override {
    // Observe expiry before new ingress can replace the evidence from a stalled owner.
    const double now=m_host.now();
    checkContinuity(now);
    m_last_now=now;
  }
  void iterate() override {
    const double now=m_host.now();
    checkContinuity(now);
    if(m_phase==Phase::Acquiring || m_phase==Phase::Active || m_phase==Phase::Paused) {
      if(m_phase==Phase::Paused && vehicleReady(now) && inputsReady(now)) {
        if(m_host.resumeNavigation("helm_resume_"+std::to_string(++m_request))) {
          m_phase=Phase::Active; m_reason="resumed_after_normal_handoff"; m_last_send=-1;
        } else latch("handoff_resume_rejected");
      }
      if(m_phase==Phase::Acquiring && isPx4() && !m_shadow) {
        if(m_backend.activeModeAllowed(m_host.vehicle(),false)) {
          if(!m_post_mode_boundary) {
            clearFields(now);
            m_post_mode_boundary=true;
            m_reason="waiting_for_post_offboard_inputs";
            m_last_send=-1;
          }
        } else if(inputsReady(now) && m_backend.helmVehicleReady(m_host.vehicle(),now,m_position_timeout,false,true) &&
                  (m_last_send<0 || now-m_last_send>=0.095)) {
          if(!m_host.navigation(target())) latch("px4_prestream_rejected");
          m_last_send=now;
        }
      }
      if(m_phase==Phase::Acquiring && m_guided_boundary_required && !m_shadow && !m_post_mode_boundary &&
         m_guided_mode_sent>=0 && m_host.vehicle().heartbeat_at>m_guided_mode_sent &&
         m_backend.activeModeAllowed(m_host.vehicle(),false)) {
        clearFields(now);
        m_post_mode_boundary=true;
        m_reason="waiting_for_post_guided_inputs";
      }
      if(m_phase==Phase::Acquiring && vehicleReady(now) && inputsReady(now) &&
         (!m_guided_boundary_required || m_shadow || m_post_mode_boundary)) {
        m_phase=Phase::Active; m_reason=m_shadow?"shadow_eligible":"eligible"; m_last_send=-1;
      }
      if(m_phase==Phase::Active && (m_last_send<0 || now-m_last_send>=0.095)) {
        const auto target=this->target();
        m_host.publish("MAVMOOS_SHADOW_COURSE",target.course);
        m_host.publish("MAVMOOS_SHADOW_SPEED",target.speed);
        if(m_backend.helmProfile().requires_height) m_host.publish(isSub()?"MAVMOOS_SHADOW_DEPTH":"MAVMOOS_SHADOW_ALTITUDE",target.height);
        if(!m_host.navigation(target) && !m_shadow) latch("navigation_rejected");
        m_last_send=now;
      }
    }
    m_last_now=now;
    std::ostringstream status;
    status << "state=" << phaseName() << ",reason=" << m_reason
           << ",coherence=ARRIVAL_GROUPED,source=" << m_source
           << ",shadow=" << (m_shadow?"true":"false")
           << ",allstop=" << m_stop
           << ",producer_resync_required=" << (m_stop=="unknown"?"true":"false");
    appendReadiness(status,now);
    m_host.publish("MAVMOOS_HELM_STATUS",status.str());
  }
private:
  bool isPx4() const { return px4::isProfile(m_backend.profile()); }
  bool isSub() const { return m_backend.profile()=="ardusub"; }
  bool isQuadPlane() const { return m_backend.profile()==quadplane::profile; }
  bool isCopter() const { return m_backend.profile()=="arducopter"; }
  bool guidedBoundaryRequired() const { return isSub() || isQuadPlane(); }
  const char* verticalKey() const { return isSub()?"DESIRED_DEPTH":"DESIRED_ALTITUDE"; }
  NavigationTarget target() const {
    NavigationTarget n;
    n.course=std::fmod(std::fmod(m_fields[0].value,360.0)+360.0,360.0);
    n.speed=m_fields[1].value;
    n.dimensions=m_backend.helmProfile().dimensions;
    n.height=m_backend.helmProfile().requires_height?m_fields[2].value:0;
    n.facing=m_facing; n.facing_heading=m_facing_heading;
    n.expires=inputExpiry();
    return n;
  }
  bool m_guided_boundary_required=false,m_post_mode_boundary=false;
  double m_guided_mode_sent=-1;
  enum class Phase { Disabled, Acquiring, Active, Paused, Latched };
  struct Field { bool present=false; double value=0,published=-1,expires=0; };
  void checkContinuity(double now) {
    if(m_phase!=Phase::Acquiring && m_phase!=Phase::Active && m_phase!=Phase::Paused) return;
    if(!std::isfinite(now) || now<m_last_now) latch("monotonic_clock_fault");
    const auto& vehicle=m_host.vehicle();
    if((isPx4() || guidedBoundaryRequired() || (m_phase==Phase::Acquiring && m_guided_boundary_required)) &&
       !m_shadow && m_phase!=Phase::Paused && !m_host.navigationGranted()) {
      // Preserve the host's specific cause (estimator expiry, local reference
      // reset, mode takeover, or failed transport). It has already fenced work.
      m_phase=Phase::Latched;
      if(isCopter())m_reason="copter_host_authority_withdrawn";
      else m_reason=isSub()?"sub_host_authority_withdrawn":isQuadPlane()?"quadplane_host_authority_withdrawn":"px4_host_authority_withdrawn";
      clearFields(now);return;
    }
    if(!m_shadow && m_phase!=Phase::Acquiring && !m_backend.activeModeAllowed(vehicle,false)) latch("external_mode_change");
    if(!m_shadow && m_phase==Phase::Acquiring && vehicle.heartbeat_valid &&
       now-vehicle.heartbeat_at<=2 && !m_backend.activeModeAllowed(vehicle,true))
      latch("external_mode_change_during_acquisition");
    if(m_phase!=Phase::Latched && !sameHome(vehicle)) latch("home_reference_changed");
    if((m_phase==Phase::Active || m_phase==Phase::Paused) && !vehicleReady(now)) latch("vehicle_observation_stale_or_invalid");
    if(m_phase==Phase::Paused && now-m_paused>=m_window) latch("handoff_window_expired");
    if(m_phase==Phase::Acquiring && now-m_acquired>=5) latch("acquisition_timeout");
    if(m_phase==Phase::Active && !inputsReady(now)) latch("helm_input_stale_or_incomplete");
  }
  static double number(const std::string& text,double lo,double hi) {
    std::size_t used=0; double value=std::stod(text,&used);
    if(used!=text.size() || !std::isfinite(value) || value<lo || value>hi)
      throw std::invalid_argument("invalid helm timing configuration");
    return value;
  }
  bool validTime(const Mail& mail,double now) const {
    return validMetadata(mail,now) && now-mail.published_at<=m_timeout;
  }
  bool validMetadata(const Mail& mail,double now) const {
    return std::isfinite(mail.published_at) && std::isfinite(mail.received_at) &&
           mail.received_at<=now+0.1 && mail.published_at<=mail.received_at+0.1 &&
           mail.published_at>=0;
  }
  bool vehicleReady(double now) const {
    return m_backend.helmVehicleReady(m_host.vehicle(),now,m_position_timeout,m_shadow,false);
  }
  bool sameHome(const Vehicle& v) const {
    return !m_backend.helmProfile().requires_home || !m_home.home_valid || (v.home_valid && v.home_latitude==m_home.home_latitude &&
      v.home_longitude==m_home.home_longitude &&
      (m_backend.profile()==px4::rover_profile || v.home_msl==m_home.home_msl));
  }
  bool inputsReady(double now) const {
    if(m_stop!="clear" || m_helm_state!="DRIVE" || now-m_state_at>m_timeout) return false;
    for(std::size_t i=0;i<fieldCount();++i) if(!m_fields[i].present || now>=m_fields[i].expires) return false;
    return publicationSpread()<=m_spread;
  }
  static double age(double now,double published,bool known) {
    return known && std::isfinite(now) && std::isfinite(published) && now>=published ? now-published : -1;
  }
  static std::string joined(const std::vector<std::string>& values) {
    std::string result;
    for(const auto& value:values) { if(!result.empty()) result+='|'; result+=value; }
    return result.empty()?"none":result;
  }
  std::vector<std::string> preparationBlocks(double now,bool allow_loiter) const {
    std::vector<std::string> blocks;
    const auto& v=m_host.vehicle();
    auto observation=[&](bool valid,double at,double limit,const std::string& name) {
      if(at>0 && (!std::isfinite(now) || !std::isfinite(at) || now<at || now-at>limit)) blocks.push_back(name+"_stale_or_future");
      else if(!valid) blocks.push_back(name+"_missing_or_invalid");
    };
    if(!std::isfinite(now)) blocks.push_back("clock_invalid");
    observation(v.heartbeat_valid,v.heartbeat_at,2,"heartbeat");
    observation(v.position_valid,v.position_at,m_position_timeout,"position");
    if(!isSub() && m_backend.helmProfile().dimensions==NavigationDimensions::Spatial)
      observation(v.landed_valid,v.landed_at,2,"landed");
    if(!v.armed) blocks.push_back("not_armed");
    if(!isSub() && m_backend.helmProfile().dimensions==NavigationDimensions::Spatial && !v.in_air) blocks.push_back("not_airborne");
    if(!vehiclePrepared(now) && blocks.empty()) blocks.push_back("vehicle_profile_prerequisites");
    if(!m_shadow) {
      const bool mode_allowed=isSub()&&allow_loiter?m_backend.acquisitionAllowed(v,now):
        m_backend.activeModeAllowed(v,allow_loiter);
      if(!mode_allowed)
        blocks.push_back(isSub()&&allow_loiter?"position_hold_acquisition_required":
          isPx4()?(allow_loiter?"mode_not_hold_or_offboard":"mode_not_offboard"):
          (allow_loiter?"mode_not_guided_or_loiter":"mode_not_guided"));
    }
    if(m_stop=="unknown") blocks.push_back("allstop_unknown");
    else if(m_stop!="clear") blocks.push_back("allstop_not_clear");
    if(m_helm_state.empty()) blocks.push_back("helm_state_missing");
    else if(m_helm_state!="DRIVE") blocks.push_back("helm_not_driving");
    else if(!std::isfinite(now) || !std::isfinite(m_state_at) || now-m_state_at>m_timeout) blocks.push_back("helm_state_stale");
    return blocks;
  }
  void appendReadiness(std::ostringstream& status,double now) const {
    const bool inactive=m_phase==Phase::Disabled || m_phase==Phase::Latched;
    const auto preparation=preparationBlocks(now,true);
    auto blocks=preparationBlocks(now,false);
    if(inactive) blocks.push_back("explicit_acquire_required");
    else {
      const char* names[]={"heading","speed",isSub()?"depth":"altitude"};
      bool complete=true;
      for(std::size_t i=0;i<fieldCount();++i) {
        const auto& field=m_fields[i];
        if(!field.present) { blocks.push_back(std::string(names[i])+"_missing"); complete=false; }
        else if(!std::isfinite(now) || now>=field.expires) blocks.push_back(std::string(names[i])+"_expired");
      }
      if(complete) {
        if(publicationSpread()>m_spread) blocks.push_back("field_spread_exceeded");
      }
      if(!sameHome(m_host.vehicle())) blocks.push_back("home_reference_changed");
    }
    const bool ready=m_phase==Phase::Active && vehicleReady(now) && inputsReady(now) && sameHome(m_host.vehicle());
    std::string next;
    if(ready) next="NONE";
    else if(!std::isfinite(now) || m_reason=="monotonic_clock_fault") next="REVIEW_CLOCK_FAULT";
    else if(m_stop=="unknown") next=m_producer_resync_after_reset?"RESYNCHRONIZE_HELM":"WAIT_HELM_STATE";
    else if(m_phase==Phase::Paused) next="WAIT_BOUNDED_HANDOFF";
    else if(!vehiclePrepared(now)) next="WAIT_VEHICLE_READY";
    else if(!m_shadow && !(isSub()&&inactive?m_backend.acquisitionAllowed(m_host.vehicle(),now):
      m_backend.activeModeAllowed(m_host.vehicle(),true))) {
      const auto profile=m_backend.profile();
      next=profile=="ardurover"?"OPERATOR_SELECT_GUIDED_OR_HOLD":
        profile=="ardurover_boat"?"OPERATOR_SELECT_GUIDED_HOLD_OR_LOITER":
        profile=="arduplane"?"OPERATOR_SELECT_LOITER":isPx4()?"OPERATOR_SELECT_HOLD":
        isSub()?"OPERATOR_SELECT_POSITION_HOLD":"OPERATOR_SELECT_GUIDED_OR_LOITER";
    }
    else if(m_stop!="clear") next="REVIEW_HELM_ALLSTOP";
    else if(m_helm_state!="DRIVE" || now-m_state_at>m_timeout) next="WAIT_FRESH_HELM_STATE";
    else if(inactive) next="EXPLICIT_ACQUIRE_WITH_NEW_INPUTS";
    else if(!m_shadow && !m_backend.activeModeAllowed(m_host.vehicle(),false)) next=isPx4()?"WAIT_OFFBOARD_CONFIRMATION":"WAIT_GUIDED_CONFIRMATION";
    else next="WAIT_FRESH_INPUTS";
    status << ",readiness=" << (ready?"READY":inactive?"INACTIVE":"BLOCKED")
           << ",blocked_by=" << joined(blocks)
           << ",acquire_preparation=" << (preparation.empty()?"READY":"BLOCKED")
           << ",acquire_blocked_by=" << joined(preparation)
           << ",next_action=" << next << ",acquire_required=" << (inactive?"true":"false")
           << ",readiness_scope=MODULE_LOCAL,host_gates=INDEPENDENT"
           << ",drive_age_s=" << age(now,m_state_at,!m_helm_state.empty());
    const char* names[]={"heading","speed",isSub()?"depth":"altitude"};
    for(std::size_t i=0;i<fieldCount();++i)
      status << ',' << names[i] << "_age_s=" << age(now,m_fields[i].published,m_fields[i].present);
    double remaining=-1;
    bool complete=true;
    for(std::size_t i=0;i<fieldCount();++i) complete=complete && m_fields[i].present;
    if(complete && std::isfinite(now)) remaining=std::max(0.0,inputExpiry()-now);
    status << ",dimensions=" << (m_backend.helmProfile().dimensions==NavigationDimensions::Planar?"PLANAR":"SPATIAL")
           << ",height_required=" << (m_backend.helmProfile().requires_height&&!isSub()?"true":"false");
    if(isSub())status << ",depth_required=true,vertical_reference=FC_HOME_SURFACE_MSL";
    status
           << ",input_remaining_s=" << remaining
           << ",handoff_remaining_s=" << (m_phase==Phase::Paused && std::isfinite(now)?std::max(0.0,m_window-(now-m_paused)):-1);
  }
  std::size_t fieldCount() const { return m_backend.helmProfile().requires_height?3:2; }
  double publicationSpread() const {
    double lo=m_fields[0].published,hi=lo;
    for(std::size_t i=1;i<fieldCount();++i) { lo=std::min(lo,m_fields[i].published); hi=std::max(hi,m_fields[i].published); }
    return hi-lo;
  }
  double inputExpiry() const {
    double result=m_fields[0].expires;
    for(std::size_t i=1;i<fieldCount();++i) result=std::min(result,m_fields[i].expires);
    return result;
  }
  bool vehiclePrepared(double now) const {
    // Preparation advice retains its legacy mode-independent observation check.
    return m_backend.helmVehicleReady(m_host.vehicle(),now,m_position_timeout,true,false);
  }
  void clearFields(double boundary) { m_fields={}; m_boundary=boundary; }
  void latch(const std::string& reason) {
    if(m_phase!=Phase::Latched) m_host.revokeNavigation(reason);
    m_phase=Phase::Latched; m_reason=reason; clearFields(m_host.now());
  }
  const char* phaseName() const {
    switch(m_phase) {
      case Phase::Disabled:return "DISABLED"; case Phase::Acquiring:return "ACQUIRING";
      case Phase::Active:return "ACTIVE"; case Phase::Paused:return "PAUSED"; case Phase::Latched:return "LATCHED";
    }
    return "LATCHED";
  }
  Services& m_host;
  const VehicleControlBackend& m_backend;
  Phase m_phase=Phase::Disabled;
  std::array<Field,3> m_fields{};
  Vehicle m_home;
  std::string m_source="pHelmIvP",m_helm_state,m_stop="unknown",m_reason="not_acquired";
  double m_started=0,m_timeout=2,m_window=0.5,m_spread=0.5,m_position_timeout=0.5,m_boundary=0,m_acquired=0,m_paused=0;
  double m_state_at=-1,m_stop_at=-1,m_control_at=-1,m_last_send=-1,m_last_now=0;
  FacingPolicy m_facing=FacingPolicy::Course;
  double m_facing_heading=0;
  unsigned long m_request=0;
  bool m_shadow=false;
  bool m_producer_resync_after_reset=false;
};
} // namespace
std::unique_ptr<Plugin> makeHelmAdapter(Services& services,const VehicleControlBackend& backend)
{ return std::make_unique<HelmAdapter>(services,backend); }
} // namespace mavmoos::runtime
