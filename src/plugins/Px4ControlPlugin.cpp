#include "ControlReplayService.h"
#include "ControlComponents.h"
#include "Px4AircraftPolicy.h"
#include "Px4Offboard.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace mavmoos::runtime {
namespace {
enum class Aircraft { Quad, Plane, StandardVtol };
enum class Action { Arm, Disarm, Takeoff, Hold, Return, Land, Forward, Hover };
constexpr std::pair<std::string_view,Action> actions[]={
  {"arm",Action::Arm},{"disarm",Action::Disarm},{"takeoff",Action::Takeoff},{"hold",Action::Hold},
  {"rtl",Action::Return},{"land",Action::Land},{"transition_fw",Action::Forward},{"transition_mc",Action::Hover}
};

bool transition(Action action) { return action==Action::Forward || action==Action::Hover; }
unsigned targetPhase(Action action)
{ return action==Action::Forward?MAV_VTOL_STATE_FW:MAV_VTOL_STATE_MC; }

class Px4Backend final : public VehicleControlBackend, public Px4AircraftPolicy {
public:
  void setProfile(std::string_view profile)
  {
    if(profile==px4::fixed_wing_profile) m_aircraft=Aircraft::Plane;
    else if(profile==px4::standard_vtol_profile) m_aircraft=Aircraft::StandardVtol;
    else if(profile==px4::multicopter_profile) m_aircraft=Aircraft::Quad;
    else throw std::invalid_argument("invalid PX4 aircraft profile");
  }
  bool nativeActionsEnabled() const override { return m_native_enabled; }
  void setNativeActions(bool enabled) { m_native_enabled=enabled; }
  bool fixedWing() const { return m_aircraft==Aircraft::Plane; }
  bool vtol() const { return m_aircraft==Aircraft::StandardVtol; }
  bool forwardFlight(double now) const override
  { return fixedWing() || (vtol() && phase(now)==MAV_VTOL_STATE_FW); }
  std::string profile() const override
  { return fixedWing()?px4::fixed_wing_profile:vtol()?px4::standard_vtol_profile:px4::multicopter_profile; }
  bool identityCompatible(unsigned type,unsigned autopilot) const override
  {
    return type==(fixedWing()?MAV_TYPE_FIXED_WING:vtol()?MAV_TYPE_VTOL_FIXEDROTOR:MAV_TYPE_QUADROTOR) &&
      autopilot==MAV_AUTOPILOT_PX4;
  }
  bool observe(const mavlink_message_t& msg,const MessageTime& time,const Vehicle& v,double now)
  {
    if(msg.sysid!=1 || msg.compid!=1 || msg.magic!=MAVLINK_STX ||
       !v.heartbeat_valid || !identityCompatible(v.type,v.autopilot) ||
       time.effective_at<v.heartbeat_at) return false;
    if(msg.msgid==MAVLINK_MSG_ID_CURRENT_MODE && time.effective_at>=m_mode_at &&
       px4::fresh(time.effective_at,now,1.6)) {
      mavlink_current_mode_t state{};
      mavlink_msg_current_mode_decode(&msg,&state);
      m_mode=state.custom_mode;
      m_intended=state.intended_custom_mode;
      m_mode_at=time.effective_at;
    } else if((fixedWing() || vtol()) && msg.msgid==MAVLINK_MSG_ID_EXTENDED_SYS_STATE &&
              time.effective_at>=m_airframe_at && px4::fresh(time.effective_at,now,1.5)) {
      mavlink_extended_sys_state_t state{};
      mavlink_msg_extended_sys_state_decode(&msg,&state);
      const bool changed=vtol() && m_airframe_at>=0 && m_phase!=state.vtol_state;
      m_phase=state.vtol_state;
      m_conventional_airborne=m_phase==MAV_VTOL_STATE_UNDEFINED &&
        state.landed_state==MAV_LANDED_STATE_IN_AIR;
      m_airframe_at=time.effective_at;
      return changed;
    } else if((fixedWing() || vtol()) && msg.msgid==MAVLINK_MSG_ID_VFR_HUD &&
              time.effective_at>=m_airspeed_at && px4::fresh(time.effective_at,now,px4::fixed_wing_airspeed_age)) {
      mavlink_vfr_hud_t hud{};
      mavlink_msg_vfr_hud_decode(&msg,&hud);
      m_airspeed_valid=std::isfinite(hud.airspeed) && hud.airspeed>=(vtol()?6:10) &&
        hud.airspeed<=(vtol()?20:35);
      m_airspeed_at=time.effective_at;
    }
    return false;
  }
  void reset(double now)
  {
    m_conventional_airborne=false;
    m_airspeed_valid=false;
    m_phase=MAV_VTOL_STATE_UNDEFINED;
    m_mode=m_intended=0;
    m_airframe_at=std::max(m_airframe_at,now);
    m_airspeed_at=std::max(m_airspeed_at,now);
    m_mode_at=std::max(m_mode_at,now);
    m_native.reset();
  }
  unsigned phase(double now,double after=-1) const
  { return m_airframe_at>after && px4::fresh(m_airframe_at,now,1.5)?m_phase:MAV_VTOL_STATE_UNDEFINED; }
  bool observedMode(const Vehicle& v,double now,unsigned mode,double after=-1) const
  {
    return heartbeat(v,now,after) && v.mode==mode && m_mode==mode && m_intended==mode &&
      m_mode_at>after && px4::fresh(m_mode_at,now,1.6);
  }
  bool heartbeat(const Vehicle& v,double now,double after=-1) const
  {
    return v.heartbeat_valid && identityCompatible(v.type,v.autopilot) &&
      v.heartbeat_at>after && px4::fresh(v.heartbeat_at,now,2);
  }
  bool flightReady(const Vehicle& v,double now,bool motion) const override
  {
    const bool forward=forwardFlight(now);
    return heartbeat(v,now) && v.armed && v.in_air && v.landed_valid && px4::fresh(v.landed_at,now,1.5) &&
      positionReady(v,now) && (!vtol() || (px4::insideFixedWingArea(v) &&
        (forward || phase(now)==MAV_VTOL_STATE_MC))) &&
      (forward?fixedWingReady(v,now):
        std::isfinite(v.relative_height) && v.relative_height>=1 && v.relative_height<=(vtol()?110:60)) &&
      activeModeAllowed(v,!motion);
  }
  bool acquisitionAllowed(const Vehicle& v,double now) const override
  { return v.mode==px4::hold && flightReady(v,now,false); }
  bool activeModeAllowed(const Vehicle& v,bool grace) const override
  { return v.mode==px4::offboard || (grace && v.mode==px4::hold); }
  std::optional<CommandRequest> acquisitionCommand(const Vehicle& v,const std::string& id,double now) const override
  {
    if(!acquisitionAllowed(v,now))return std::nullopt;
    CommandRequest r;
    r.id="acquire-"+id;r.command=MAV_CMD_DO_SET_MODE;r.params[0]=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    r.params[1]=6;r.expires=now+.3;return r;
  }
  bool commandAllowed(const CommandRequest& r,const Vehicle& v,double now) const override
  {
    if(r.command==MAV_CMD_DO_SET_MODE && r.params[1]==6) {
      return acquisitionAllowed(v,now) && r.params[0]==MAV_MODE_FLAG_CUSTOM_MODE_ENABLED &&
        std::all_of(r.params.begin()+2,r.params.end(),[](float p){return p==0;});
    }
    return m_native && r.id==m_native->request.id && r.command==m_native->request.command &&
      r.params==m_native->request.params && r.expires==m_native->request.expires &&
      r.operation_expires==m_native->request.operation_expires && r.execution_id.empty() && r.abort_id.empty() &&
      v.armed==m_native->armed && v.mode==m_native->mode && m_intended==m_native->intended &&
      (!vtol() || phase(now)==m_native->phase) &&
      nativeReady(m_native->action,v,now) && (m_native->action!=Action::Takeoff ||
        (v.home_latitude==m_native->home_latitude && v.home_longitude==m_native->home_longitude &&
         std::abs(v.home_msl-m_native->home_msl)<=.05));
  }
  bool nativeReady(Action action,const Vehicle& v,double now) const
  {
    if(!heartbeat(v,now) || !v.landed_valid || !px4::fresh(v.landed_at,now,1.5) ||
       m_mode!=v.mode || !px4::fresh(m_mode_at,now,1.6)) return false;
    const bool lost_offboard_hold=action==Action::Hold && v.mode==px4::hold && m_intended==px4::offboard;
    if(m_intended!=m_mode && !lost_offboard_hold) return false;
    const unsigned vtol_phase=phase(now);
    if(vtol() && vtol_phase!=MAV_VTOL_STATE_MC && vtol_phase!=MAV_VTOL_STATE_FW) return false;
    const bool grounded=v.landed_state==MAV_LANDED_STATE_ON_GROUND;
    const bool parked_mode=v.mode==px4::manual || v.mode==px4::altitude || v.mode==px4::position ||
      v.mode==px4::stabilized || v.mode==px4::hold;
    if(action==Action::Arm) return grounded && parked_mode && (!vtol() || vtol_phase==MAV_VTOL_STATE_MC);
    if(action==Action::Disarm) return grounded && v.mode!=px4::offboard;
    if(action==Action::Takeoff)
      return grounded && v.armed && parked_mode && positionReady(v,now) &&
        std::abs(v.relative_height)<=5 && px4::insideFixedWingArea(v) &&
        (!vtol() || vtol_phase==MAV_VTOL_STATE_MC);
    if(!v.armed || !v.in_air || !positionReady(v,now)) return false;
    if(transition(action))
      return vtol() && v.mode==px4::hold && vtol_phase!=targetPhase(action) &&
        v.relative_height>=22 && v.relative_height<=100 && px4::insideFixedWingArea(v);
    if(action==Action::Land && vtol() && vtol_phase!=MAV_VTOL_STATE_MC) return false;
    return v.mode==px4::hold || v.mode==px4::offboard || v.mode==px4::rtl;
  }
  bool noEffectNeeded(Action action,const Vehicle& v,double now) const
  {
    return (action==Action::Arm && v.armed) || (action==Action::Disarm && !v.armed) ||
      (action==Action::Hold && observedMode(v,now,px4::hold)) ||
      (action==Action::Return && observedMode(v,now,px4::rtl));
  }
  bool takeoffHeight(double height) const
  { return std::isfinite(height) && height>=(fixedWing()?35:vtol()?25:3) && height<=(fixedWing()?100:vtol()?65:30); }
  bool bindNative(Action action,CommandRequest& request,double height,const Vehicle& v,double now)
  {
    if(!m_native_enabled || m_native || !nativeReady(action,v,now)) return false;
    switch(action) {
    case Action::Arm:
    case Action::Disarm:
      request.command=MAV_CMD_COMPONENT_ARM_DISARM;
      request.params[0]=action==Action::Arm?1:0;
      break;
    case Action::Takeoff: {
      if(!takeoffHeight(height) || !std::isfinite(v.heading) || v.heading<0 || v.heading>=360) return false;
      const double bearing=v.heading*px4::radians;
      const double north=fixedWing()?160*std::cos(bearing):0;
      const double east=fixedWing()?160*std::sin(bearing):0;
      if(!px4::insideFixedWingArea(v,north,east)) return false;
      request.command=MAV_CMD_NAV_TAKEOFF;
      request.params[4]=static_cast<float>(v.latitude+north/px4::metres_per_degree);
      request.params[5]=static_cast<float>(std::remainder(
        v.longitude+east/(px4::metres_per_degree*std::cos(v.latitude*px4::radians)),360.));
      request.params[6]=static_cast<float>(v.home_msl+height);
      break;
    }
    case Action::Hold:
      request.command=MAV_CMD_DO_SET_MODE;
      request.params[0]=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
      request.params[1]=4;
      request.params[2]=3;
      break;
    case Action::Return: request.command=MAV_CMD_NAV_RETURN_TO_LAUNCH; break;
    case Action::Land: request.command=MAV_CMD_NAV_LAND; break;
    case Action::Forward:
    case Action::Hover:
      request.command=MAV_CMD_DO_VTOL_TRANSITION;
      request.params[0]=static_cast<float>(targetPhase(action));
      break;
    }
    m_native=NativeBinding{request,action,v.armed,v.mode,m_intended,phase(now),
                           v.home_latitude,v.home_longitude,v.home_msl};
    return true;
  }
  void clearNative() { m_native.reset(); }
  bool navigationInputAllowed(const NavigationTarget& t) const override
  {
    if(t.kind!=NavigationKind::CourseSpeed || t.dimensions!=NavigationDimensions::Spatial ||
       !helmFieldAllowed(0,t.course) || !helmFieldAllowed(1,t.speed) || !helmFieldAllowed(2,t.height)) return false;
    if(fixedWing() || vtol()) return t.facing==FacingPolicy::Course;
    return t.facing==FacingPolicy::Course || t.facing==FacingPolicy::Keep ||
      (t.facing==FacingPolicy::Heading && std::isfinite(t.facing_heading) &&
       t.facing_heading>=0 && t.facing_heading<360);
  }
  bool helmFieldAllowed(unsigned index,double value) const override
  {
    if(!std::isfinite(value)) return false;
    if(index==0) return std::abs(value)<=360;
    if(index==1) return fixedWing()?(value>=12 && value<=22):
      (vtol() && m_phase==MAV_VTOL_STATE_FW)?(value>=7 && value<=16):(value>=0 && value<=3);
    return index==2 && (fixedWing()?(value>=35 && value<=100):
      vtol()?(value>=(m_phase==MAV_VTOL_STATE_FW?25:2) && value<=100):(value>=2 && value<=30));
  }
  bool helmVehicleReady(const Vehicle& v,double now,double timeout,bool shadow,bool acquisition) const override
  {
    auto state=v;
    if(shadow)state.mode=px4::hold;
    return flightReady(state,now,!shadow&&!acquisition) && px4::fresh(v.position_at,now,timeout);
  }
  std::optional<ControlFrame> navigationFrame(const NavigationTarget& t,const Vehicle& v,double now,
                                             uint64_t,mavlink_status_t& codec) const override
  {
    if(!std::isfinite(now) || now<0 || now>=t.expires || !flightReady(v,now,true) || !navigationInputAllowed(t))
      return std::nullopt;
    if(forwardFlight(now)) return fixedWingTarget(t,v,now,codec);
    mavlink_set_position_target_local_ned_t value{};
    value.type_mask=px4::ignore_acceleration|POSITION_TARGET_TYPEMASK_X_IGNORE|POSITION_TARGET_TYPEMASK_Y_IGNORE|
      POSITION_TARGET_TYPEMASK_Z_IGNORE|POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    value.vx=static_cast<float>(t.speed*std::cos(t.course*px4::radians));
    value.vy=static_cast<float>(t.speed*std::sin(t.course*px4::radians));
    value.vz=static_cast<float>(std::clamp((v.relative_height-t.height)*.5,-1.,1.));
    if(t.facing==FacingPolicy::Heading || (t.facing==FacingPolicy::Course && t.speed>=.2))
      value.yaw=static_cast<float>(std::remainder(t.facing==FacingPolicy::Heading?t.facing_heading:t.course,360.)*px4::radians);
    else value.type_mask|=POSITION_TARGET_TYPEMASK_YAW_IGNORE;
    return px4::encode(value,now,codec);
  }
  std::optional<ControlFrame> stopFrame(const Vehicle&,double,uint64_t,mavlink_status_t&) const override
  { return std::nullopt; }
private:
  bool positionReady(const Vehicle& v,double now) const
  {
    return v.position_valid && px4::fresh(v.position_at,now,px4::position_age) &&
      v.estimator_valid && px4::fresh(v.estimator_at,now,px4::estimator_age) &&
      v.home_valid && px4::fresh(v.home_at,now,4) && std::isfinite(v.home_msl);
  }
  bool fixedWingReady(const Vehicle& v,double now) const
  {
    const double ground_speed=std::hypot(v.north_velocity,v.east_velocity);
    return (vtol() || (m_conventional_airborne && px4::fresh(m_airframe_at,now,1.5))) &&
      m_airspeed_valid && px4::fresh(m_airspeed_at,now,px4::fixed_wing_airspeed_age) &&
      v.relative_height>=20 && v.relative_height<=130 &&
      std::abs(v.msl_height-v.home_msl-v.relative_height)<=5 &&
      ground_speed>=(vtol()?5:8) && ground_speed<=px4::fixed_wing_max_ground_speed && px4::insideFixedWingArea(v);
  }
  std::optional<ControlFrame> fixedWingTarget(const NavigationTarget& t,const Vehicle& v,double now,
                                            mavlink_status_t& codec) const
  {
    const double bearing=t.course*px4::radians;
    const double north=t.speed*px4::fixed_wing_lookahead_s*std::cos(bearing);
    const double east=t.speed*px4::fixed_wing_lookahead_s*std::sin(bearing);
    if(!px4::insideFixedWingArea(v,north,east)) return std::nullopt;
    const double lat=v.latitude+north/px4::metres_per_degree;
    const double lon=std::remainder(v.longitude+east/(px4::metres_per_degree*std::cos(v.latitude*px4::radians)),360.);
    mavlink_set_position_target_global_int_t value{};
    value.time_boot_ms=static_cast<uint32_t>(std::fmod(now*1000.,4294967296.));
    value.target_system=1;
    value.target_component=1;
    // PX4 v1.17.0 expects the _INT coordinate-frame enum for MSL.
    value.coordinate_frame=MAV_FRAME_GLOBAL_INT;
    value.type_mask=px4::position_mask;
    value.lat_int=static_cast<int32_t>(std::llround(lat*1e7));
    value.lon_int=static_cast<int32_t>(std::llround(lon*1e7));
    value.alt=static_cast<float>(v.home_msl+t.height);
    mavlink_message_t message{};
    mavlink_msg_set_position_target_global_int_encode_status(245,191,&codec,&message,&value);
    ControlFrame frame;
    frame.size=mavlink_msg_to_send_buffer(frame.bytes.data(),&message);
    return frame;
  }

  struct NativeBinding {
    CommandRequest request;
    Action action;
    bool armed;
    unsigned mode,intended,phase;
    double home_latitude,home_longitude,home_msl;
  };
  std::optional<NativeBinding> m_native;
  Aircraft m_aircraft=Aircraft::Quad;
  bool m_conventional_airborne=false,m_airspeed_valid=false,m_native_enabled=false;
  unsigned m_phase=MAV_VTOL_STATE_UNDEFINED,m_mode=0,m_intended=0;
  double m_airframe_at=-1,m_airspeed_at=-1,m_mode_at=-1;
};

struct Operation {
  CommandRequest request;
  std::string action_name;
  Action action=Action::Arm;
  double height=0, received=0, sent=-1;
  unsigned initial_mode=0;
  bool ack_observed=false, confirmed=false;
};

class Px4Control final : public Plugin {
public:
  explicit Px4Control(Services& services):m_host(services),m_helm(makeHelmAdapter(services,m_backend)),
    m_boundary(services.now()) {}
  std::string name() const override { return "px4_control"; }
  const VehicleControlBackend* controlBackend() const override { return &m_backend; }
  PluginDescriptor descriptor() const override
  {
    PluginDescriptor d;
    d.profile=m_backend.profile();d.exclusive_resources={"vehicle.control"};
    d.request_owners={"MAVMOOS_NAV_CONTROL","MAVMOOS_COMMAND"};
    d.observation_streams={{MAVLINK_MSG_ID_ESTIMATOR_STATUS,200000.f},{MAVLINK_MSG_ID_CURRENT_MODE,500000.f}};
    if(m_backend.fixedWing() || m_backend.vtol()) d.observation_streams.push_back({MAVLINK_MSG_ID_VFR_HUD,200000.f});
    if(m_helm_enabled) d.capabilities={{"control.navigation.helm","MAVMOOS_NAV_CONTROL","acquire|release"},
      {"control.hold.offboard_prestream","","","internal"}};
    if(m_backend.vtol()) d.capabilities.push_back({"control.px4.vtol.phase","MAVMOOS_PX4_VTOL_PHASE","","observation"});
    if(m_backend.nativeActionsEnabled()) for(const auto& [name,action]:actions)
      if(!transition(action) || m_backend.vtol())
        d.capabilities.push_back({"control.px4.native."+std::string(name),"MAVMOOS_COMMAND","action="+std::string(name)});
    return d;
  }
  std::vector<std::string> subscriptions() const override
  {
    auto result=m_helm_enabled?m_helm->subscriptions():std::vector<std::string>{};
    if(m_backend.nativeActionsEnabled()) result.push_back("MAVMOOS_COMMAND");
    return result;
  }
  void configure(const std::string& key,const std::string& value) override
  {
    if(key=="vehicle_profile") m_backend.setProfile(value);
    if((m_backend.fixedWing() || m_backend.vtol()) && key=="facing_policy" && value!="course")
      throw std::invalid_argument("PX4 winged aircraft use course guidance");
    if(key=="control_native_actions" || key=="control_helm") {
      if(value!="true"&&value!="false")throw std::invalid_argument("invalid "+key);
      if(key=="control_helm") m_helm_enabled=value=="true";
      else m_backend.setNativeActions(value=="true");
    }
    m_helm->configure(key,value);
  }
  void onMail(const Mail& mail) override
  {
    if(mail.key=="MAVMOOS_COMMAND") {
      if(m_backend.nativeActionsEnabled()) queueNative(mail);
    } else if(m_helm_enabled) m_helm->onMail(mail);
  }
  void onCommandSent(const CommandRequest& request,double at,bool accepted) override
  {
    if(!m_operation || request.id!=m_operation->request.id || request.command!=m_operation->request.command ||
       m_operation->sent>=0) return;
    if(!accepted) {finish("FAILED","local_send_not_accepted");return;}
    if(!std::isfinite(at) || at<m_operation->received || at>=request.expires) {
      finish("FAILED","unusable_send_timestamp",true);return;
    }
    m_operation->sent=at;
    m_backend.clearNative();
  }
  void onMessageWithMetadata(const mavlink_message_t& message,const MessageTime& time) override
  {
    const auto now=m_host.now();
    if(m_backend.observe(message,time,m_host.vehicle(),now))
      m_host.revokeNavigation("px4_vtol_phase_changed");
    expire(now);
    // ACKs use the actual read time. The queue-empty floor may predate a fast reply.
    if(!m_operation || m_operation->sent<0 || m_operation->ack_observed || message.msgid!=MAVLINK_MSG_ID_COMMAND_ACK ||
       message.magic!=MAVLINK_STX || message.sysid!=1 || message.compid!=1 ||
       !std::isfinite(time.read_at) || time.read_at<=m_operation->sent || time.read_at>now) return;
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&message,&ack);
    if(ack.command!=m_operation->request.command || ack.target_system!=245 || ack.target_component!=191) return;
    m_operation->ack_observed=true;
    report(*m_operation,"ACK_OBSERVED","uncorrelated_ack_result_"+std::to_string(ack.result));
  }
  void onInputBoundary() override
  {
    expire(m_host.now());
    if(m_helm_enabled)m_helm->onInputBoundary();
  }
  void iterate() override
  {
    if(m_queued) submit();
    expire(m_host.now());
    if(m_operation && m_operation->sent>=0) observeEffect();
    if(m_helm_enabled)m_helm->iterate();
    if(m_backend.vtol()) m_host.publish("MAVMOOS_PX4_VTOL_PHASE",double(m_backend.phase(m_host.now())));
  }
  void onReset(const std::string& reason) override
  {
    if(m_operation) finish("CANCELLED","connection_reset",m_operation->sent>=0);
    if(m_queued) report(*m_queued,"CANCELLED","connection_reset");
    m_queued.reset();
    m_boundary=m_host.now();
    m_backend.reset(m_boundary);
    if(m_helm_enabled)m_helm->onReset(reason);
  }
private:
  void queueNative(const Mail& mail)
  {
    Operation op;
    op.request.id="invalid";
    op.action_name="invalid";
    const auto reject=[&] {report(op,"REJECTED","invalid_request");};
    if(mail.numeric || mail.text.empty() || mail.text.size()>180 || mail.text.back()==',') {reject();return;}
    std::map<std::string,std::string> fields;
    std::istringstream input(mail.text);
    for(std::string item;std::getline(input,item,',');) {
      const auto split=item.find('=');
      if(split==std::string::npos || !fields.emplace(item.substr(0,split),item.substr(split+1)).second) {
        reject();return;
      }
    }
    const auto id=fields.find("id"),action=fields.find("action"),height=fields.find("height");
    if(id==fields.end() || action==fields.end() || id->second.empty() || id->second.size()>48 ||
       !std::all_of(id->second.begin(),id->second.end(),[](unsigned char ch) {
         return (ch>='a' && ch<='z') || (ch>='A' && ch<='Z') || (ch>='0' && ch<='9') ||
           ch=='_' || ch=='-' || ch=='.';
       })) {reject();return;}
    op.request.id=id->second;
    const auto entry=std::find_if(std::begin(actions),std::end(actions),
      [&](const auto& candidate){return candidate.first==action->second;});
    if(entry==std::end(actions) || (transition(entry->second) && !m_backend.vtol())) {reject();return;}
    op.action_name=action->second;
    op.action=entry->second;
    const bool takeoff=op.action==Action::Takeoff;
    if(fields.size()!=(takeoff?3u:2u) || takeoff!=(height!=fields.end())) {reject();return;}
    if(takeoff) {
      try {
        std::size_t used=0;
        op.height=std::stod(height->second,&used);
        if(used!=height->second.size() || !m_backend.takeoffHeight(op.height)) {reject();return;}
      } catch(const std::exception&) {reject();return;}
    }
    const double now=m_host.now();
    if(!std::isfinite(mail.published_at) || !std::isfinite(mail.received_at) || mail.published_at<=m_boundary ||
       mail.published_at>mail.received_at || !px4::fresh(mail.received_at,now,.5) ||
       mail.received_at-mail.published_at>.5) {
      report(op,"REJECTED","stale_or_future_request");return;
    }
    if(m_seen.count(op.request.id)) {report(op,"DUPLICATE","id_already_seen");return;}
    if(m_seen.size()>=128) {report(op,"REJECTED","request_history_full");return;}
    const auto replay=reserveControlRequest(m_host,op.request.id);
    if(replay!=ReplayResult::Reserved) {report(op,replayRejection(replay),replayReason(replay));return;}
    m_seen.insert(op.request.id);
    if(m_operation || m_queued || m_host.controlBusy()) {report(op,"REJECTED","busy");return;}
    op.received=mail.received_at;
    op.request.expires=mail.published_at+.65;
    const double duration=takeoff?65:op.action==Action::Land?118:transition(op.action)?35:8;
    op.request.operation_expires=mail.published_at+duration;
    m_queued=std::move(op);
  }
  void submit()
  {
    auto op=std::move(*m_queued);
    m_queued.reset();
    const auto& v=m_host.vehicle();
    const auto now=m_host.now();
    if(now>=op.request.expires || m_host.controlBusy() || !m_backend.nativeReady(op.action,v,now)) {
      report(op,"REJECTED","native_state_or_deadline");return;
    }
    if(m_backend.noEffectNeeded(op.action,v,now)) {
      if(op.action==Action::Hold) m_host.revokeNavigation("px4_native_hold_observed");
      report(op,"NO_OP","already_observed");return;
    }
    op.initial_mode=v.mode;
    if(!m_backend.bindNative(op.action,op.request,op.height,v,now) || !m_host.sendCommand(op.request)) {
      m_backend.clearNative();
      report(op,"REJECTED","send_admission_denied");return;
    }
    m_operation=std::move(op);
    report(*m_operation,"ADMITTED","awaiting_local_send");
  }
  void expire(double now)
  {
    if(!m_operation) return;
    const auto& op=*m_operation;
    if(now>=op.request.operation_expires || (op.sent<0 && now>=op.request.expires)) {
      finish("TIMED_OUT","native_command_deadline",op.sent>=0);
    }
  }
  void observeEffect()
  {
    const auto& op=*m_operation;
    const auto& v=m_host.vehicle();
    const double now=m_host.now(), sent=op.sent;
    const bool beat=m_backend.heartbeat(v,now,sent);
    const bool grounded=v.landed_valid && px4::fresh(v.landed_at,now,1.5) && v.landed_at>sent &&
      v.landed_state==MAV_LANDED_STATE_ON_GROUND;
    const bool airborne=v.landed_valid && px4::fresh(v.landed_at,now,1.5) && v.landed_at>sent && v.in_air;
    const auto mode=[&](unsigned expected){return m_backend.observedMode(v,now,expected,sent);};
    bool active=false,done=false,interrupted=false;
    switch(op.action) {
    case Action::Arm:
    case Action::Disarm:
      active=done=grounded && beat && (v.armed==(op.action==Action::Arm));
      break;
    case Action::Takeoff:
      active=mode(px4::takeoff);
      interrupted=beat && (!v.armed || (mode(v.mode) && v.mode!=px4::takeoff &&
        v.mode!=px4::hold && v.mode!=op.initial_mode));
      done=op.confirmed && mode(px4::hold) && v.armed && airborne && v.position_valid &&
        px4::fresh(v.position_at,now,px4::position_age) && v.position_at>sent &&
        std::abs(v.relative_height-op.height)<=(m_backend.fixedWing()?6:1.2);
      break;
    case Action::Hold:
    case Action::Return:
      active=done=mode(op.action==Action::Hold?px4::hold:px4::rtl) && v.armed && airborne;
      break;
    case Action::Land:
      active=mode(px4::land);
      done=op.confirmed && beat && grounded;
      interrupted=beat && op.confirmed && !grounded && mode(v.mode) && v.mode!=px4::land;
      break;
    case Action::Forward:
    case Action::Hover:
      active=done=mode(px4::hold) && v.armed && airborne &&
        m_backend.phase(now,sent)==targetPhase(op.action);
      interrupted=beat && (!v.armed || (mode(v.mode) && v.mode!=px4::hold));
      break;
    }
    if(interrupted) {finish("CANCELLED","native_effect_interrupted",true);return;}
    if(active && !op.confirmed) {
      m_operation->confirmed=true;
      report(op,"CONFIRMED","fresh_post_send_observation");
    }
    if(done) finish("COMPLETED","observed_native_effect");
  }
  void report(const Operation& op,const std::string& status,const std::string& detail)
  {
    m_host.publish("MAVMOOS_COMMAND_RESULT","id="+op.request.id+",action="+op.action_name+
      ",status="+status+",reason="+detail);
  }
  void finish(const std::string& status,const std::string& detail,bool uncertain=false)
  {
    const auto op=std::move(*m_operation);
    m_operation.reset();
    m_backend.clearNative();
    if(uncertain) m_host.cancelCommand(op.request.id);
    else m_host.commandFinished(op.request.id);
    report(op,status,detail);
  }

  Services& m_host;
  Px4Backend m_backend;
  std::unique_ptr<Plugin> m_helm;
  double m_boundary;
  bool m_helm_enabled=true;
  std::set<std::string> m_seen;
  std::optional<Operation> m_queued,m_operation;
};
} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi,mavmoos::runtime::Services* services)
{ return abi==mavmoos::runtime::plugin_abi && services?new mavmoos::runtime::Px4Control(*services):nullptr; }

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin) { delete plugin; }
