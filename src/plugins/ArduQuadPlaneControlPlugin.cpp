/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ArduQuadPlaneControlPlugin.cpp                   */
/*    DATE: November 6, 2026                                */
/************************************************************/
#include "ControlReplayService.h"
#include "ControlComponents.h"
#include "PlaneComponents.h"
#include "QuadPlanePolicy.h"
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace mavmoos::runtime {
namespace {
constexpr char request_key[]="MAVMOOS_QUADPLANE_REQUEST", result_key[]="MAVMOOS_QUADPLANE_RESULT";

bool token(const std::string& text)
{
  return !text.empty()&&text.size()<=80&&std::all_of(text.begin(),text.end(),[](unsigned char c) {
    return (c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='-'||c=='_'||c=='.';
  });
}

std::map<std::string,std::string> fields(const std::string& text)
{
  if(text.empty()||text.size()>768||text.back()==',')return {};
  std::map<std::string,std::string> values;
  std::istringstream stream(text);
  for(std::string pair;std::getline(stream,pair,',');) {
    const auto at=pair.find('=');
    if(at==0||at==std::string::npos||at+1==pair.size()||
       !values.emplace(pair.substr(0,at),pair.substr(at+1)).second)return {};
  }
  return values;
}

class Backend final : public VehicleControlBackend, public quadplane::PhasePolicy {
public:
  explicit Backend(double now):m_forward(now,true) {}
  std::string profile() const override { return quadplane::profile; }
  bool identityCompatible(unsigned type,unsigned autopilot) const override
  { return type==MAV_TYPE_VTOL_FIXEDROTOR&&autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA; }
  unsigned phase(double now) const override
  { return quadplane::fresh(m_phase_at,now,1.5)?m_phase:unsigned(MAV_VTOL_STATE_UNDEFINED); }
  void observe(const mavlink_message_t& msg,double at)
  {
    if(at<m_phase_at)return;
    mavlink_extended_sys_state_t state{};
    mavlink_msg_extended_sys_state_decode(&msg,&state);
    m_phase=msg.len==MAVLINK_MSG_ID_EXTENDED_SYS_STATE_LEN&&
      state.vtol_state>=MAV_VTOL_STATE_TRANSITION_TO_FW&&state.vtol_state<=MAV_VTOL_STATE_FW?
      state.vtol_state:unsigned(MAV_VTOL_STATE_UNDEFINED);
    m_phase_at=at;
  }
  void enableHelm(bool enabled) { m_helm_enabled=enabled; }
  void enableHover(bool enabled) { m_hover_enabled=enabled; }
  void observeForward(const mavlink_message_t& msg,const MessageTime& time,const Vehicle& v,double now)
  {
    if(m_helm_enabled)m_forward.observe(msg,time,v,now);
  }
  void reset(double now) { m_phase_at=-1;m_forward.reset(now); }

  bool commandAllowed(const CommandRequest& r,const Vehicle& v,double now) const override
  {
    if(m_helm_enabled) {
      if(quadplane::modeCommand(r,plane::guided_mode))return acquisitionAllowed(v,now);
      if(quadplane::modeCommand(r,holdMode()))return v.mode==plane::guided_mode&&v.armed&&
        quadplane::identity(v)&&v.heartbeat_valid&&quadplane::fresh(v.heartbeat_at,now,2)&&
        phase(now)==helmPhase();
    }
    return v.mode==quadplane::qloiter&&phase(now)==MAV_VTOL_STATE_MC&&
      quadplane::stoppedOnGround(v,now)&&quadplane::armCommand(r,!v.armed);
  }
  bool missionExecutionCommandAllowed(const CommandRequest& r,const Vehicle& v,double now) const override
  {
    if(!quadplane::observed(v,now)||phase(now)==MAV_VTOL_STATE_UNDEFINED)return false;
    if(!r.abort_id.empty())return v.armed&&v.in_air&&v.mode==quadplane::auto_mode&&quadplane::modeCommand(r,quadplane::qrtl);
    if(v.mode==quadplane::qloiter&&v.armed&&quadplane::stoppedOnGround(v,now)&&phase(now)==MAV_VTOL_STATE_MC) {
      if(r.command==MAV_CMD_DO_SET_MISSION_CURRENT)return r.params[0]==quadplane::takeoff_sequence&&
        r.params[1]==1&&std::all_of(r.params.begin()+2,r.params.end(),[](float x){return x==0;});
      return r.command==MAV_CMD_MISSION_START&&
        std::all_of(r.params.begin(),r.params.end(),[](float x){return x==0;});
    }
    return v.mode==quadplane::auto_mode&&phase(now)==MAV_VTOL_STATE_MC&&quadplane::stoppedOnGround(v,now)&&
      (v.armed?quadplane::armCommand(r,false):quadplane::modeCommand(r,quadplane::qloiter));
  }

  bool flightReady(const Vehicle& v,double now,bool motion) const override
  {
    return m_helm_enabled&&phase(now)==helmPhase()&&(m_hover_enabled?
      (!motion||v.mode==plane::guided_mode)&&quadplane::hoverReady(v,now,.5):
      m_forward.flightReady(v,now,motion));
  }
  bool acquisitionAllowed(const Vehicle& v,double now) const override
  { return activeModeAllowed(v,true)&&flightReady(v,now,false); }
  bool activeModeAllowed(const Vehicle& v,bool grace) const override
  { return m_helm_enabled&&(v.mode==plane::guided_mode||(grace&&v.mode==holdMode())); }
  std::optional<CommandRequest> acquisitionCommand(const Vehicle& v,const std::string& id,double now) const override
  {
    return v.mode==holdMode()&&acquisitionAllowed(v,now)?
      std::make_optional(plane::modeRequest("acquire-"+id,plane::guided_mode,now+5)):std::nullopt;
  }
  bool navigationInputAllowed(const NavigationTarget& target) const override
  {
    return m_helm_enabled&&target.kind==NavigationKind::CourseSpeed&&
      (m_hover_enabled?quadplane::hoverTarget(target):m_forward.navigationInputAllowed(target));
  }
  bool helmFieldAllowed(unsigned index,double value) const override
  { return m_helm_enabled&&(m_hover_enabled?quadplane::hoverField(index,value):m_forward.helmFieldAllowed(index,value)); }
  bool helmVehicleReady(const Vehicle& v,double now,double timeout,bool shadow,bool acquisition) const override
  {
    return m_helm_enabled&&phase(now)==helmPhase()&&(m_hover_enabled?
      (shadow||activeModeAllowed(v,acquisition))&&quadplane::hoverReady(v,now,timeout):
      m_forward.helmVehicleReady(v,now,timeout,shadow,acquisition));
  }
  std::optional<ControlFrame> navigationFrame(const NavigationTarget& target,const Vehicle& v,double now,
    uint64_t revision,mavlink_status_t& codec) const override
  {
    if(m_hover_enabled||!navigationInputAllowed(target)||!flightReady(v,now,true))return std::nullopt;
    return m_forward.navigationFrame(target,v,now,revision,codec);
  }
  std::optional<ControlFrame> stopFrame(const Vehicle&,double,uint64_t,mavlink_status_t&) const override { return {}; }

private:
  unsigned holdMode() const { return m_hover_enabled?quadplane::qloiter:plane::loiter_mode; }
  unsigned helmPhase() const { return m_hover_enabled?MAV_VTOL_STATE_MC:MAV_VTOL_STATE_FW; }
  plane::Backend m_forward;
  bool m_helm_enabled=false,m_hover_enabled=false;
  double m_phase_at=-1;
  unsigned m_phase=MAV_VTOL_STATE_UNDEFINED;
};

enum class Step { Arm, Disarm, Select, Start, Flight, LandDisarm, LandHold, Abort, Release };
struct Work {
  std::string id,action,wire;
  Step step;
  double deadline=0,sent=-1;
  unsigned flight_phase=0;
  bool awaiting=false;
};

class Control final : public Plugin {
public:
  explicit Control(Services& host)
    :m_host(host),m_backend(host.now()),m_helm(makeHelmAdapter(host,m_backend)),m_floor(host.now()) {}
  std::string name() const override { return "arduquadplane_control"; }
  const VehicleControlBackend* controlBackend() const override { return &m_backend; }
  PluginDescriptor descriptor() const override
  {
    PluginDescriptor d;
    d.profile=quadplane::profile;d.exclusive_resources={"vehicle.control"};
    if(m_enabled||m_helm_enabled)
      d.observation_streams={{MAVLINK_MSG_ID_GLOBAL_POSITION_INT,100000},
        {MAVLINK_MSG_ID_HOME_POSITION,1000000},{MAVLINK_MSG_ID_EXTENDED_SYS_STATE,500000}};
    if(m_enabled) {
      d.request_owners={request_key};
      d.capabilities={{"control.arm",request_key,"arm"},{"control.disarm",request_key,"disarm"},
        {"control.mission.start",request_key,"run"},{"control.mission.abort",request_key,"abort"},
        {"control.mission.release",request_key,"release"}};
      d.observation_streams.push_back({MAVLINK_MSG_ID_MISSION_CURRENT,500000});
    }
    if(m_helm_enabled) {
      d.request_owners.push_back("MAVMOOS_NAV_CONTROL");
      d.capabilities.push_back({"control.navigation.helm","MAVMOOS_NAV_CONTROL","acquire|release"});
      d.observation_streams.insert(d.observation_streams.end(),{
        {MAVLINK_MSG_ID_EKF_STATUS_REPORT,250000},
        {MAVLINK_MSG_ID_AIRSPEED,250000},
        {MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT,m_hover_enabled?200000.f:500000.f}});
      if(m_hover_enabled)
        d.capabilities.push_back({quadplane::hover_capability,"MAVMOOS_NAV_CONTROL","acquire|release"});
    }
    return d;
  }
  std::vector<std::string> subscriptions() const override
  {
    auto keys=m_helm_enabled?m_helm->subscriptions():std::vector<std::string>{};
    if(m_enabled)keys.push_back(request_key);
    return keys;
  }
  void configure(const std::string& key,const std::string& value) override
  {
    if(key=="vehicle_profile"&&value!=quadplane::profile)throw std::invalid_argument("invalid QuadPlane profile");
    if(key=="control_mission_execution") {
      if(value!="true"&&value!="false")throw std::invalid_argument("invalid QuadPlane mission enablement");
      m_enabled=value=="true";
    } else if(key=="control_helm") {
      if(value!="true"&&value!="false")throw std::invalid_argument("invalid QuadPlane helm enablement");
      m_helm_enabled=value=="true";
      m_backend.enableHelm(m_helm_enabled);
    }
    m_helm->configure(key,value);
  }

  ModuleConfigResult configureModule(const std::vector<ModuleSetting>& settings) override
  {
    if(settings.empty())return {};
    for(const auto& item:settings)
      if(item.key!="hover_adapter")return {ModuleConfigStatus::UnknownKey,item.key,"use hover_adapter"};
    if(settings.size()!=1||(settings.front().value!="true"&&settings.front().value!="false")||
       (settings.front().value=="true"&&!m_helm_enabled))
      return {ModuleConfigStatus::Invalid,"hover_adapter","one boolean setting requiring control_helm=true"};
    m_hover_enabled=settings.front().value=="true";
    m_backend.enableHover(m_hover_enabled);
    return {};
  }

  void onMail(const Mail& mail) override
  {
    if(m_helm_enabled)m_helm->onMail(mail);
    if(!m_enabled||mail.key!=request_key||mail.numeric)return;
    const auto now=m_host.now();
    auto values=fields(mail.text);
    const auto id=values["id"],action=values["action"];
    if(values["schema"]!="1"||!token(id)||m_seen.count(id)||m_seen.size()>=64||
       !quadplane::fresh(mail.published_at,now,1)||!quadplane::fresh(mail.received_at,now,1)||
       mail.published_at<=m_floor||mail.published_at>mail.received_at+.1) {
      result(token(id)?id:"invalid","REJECTED","invalid_stale_or_duplicate");return;
    }
    if(action=="abort"||action=="release") {
      if(values.size()!=4||m_retained.empty()||values["target"]!=m_retained) {
        result(id,"REJECTED","unknown_execution");return;
      }
      handback(id,action,mail.published_at);return;
    }
    if(m_work||!m_retained.empty()) { result(id,"REJECTED","control_busy");return; }
    const auto& v=m_host.vehicle();
    if(action=="arm"||action=="disarm") {
      if(values.size()!=3||!quadplane::stoppedOnGround(v,now)||v.mode!=quadplane::qloiter||
         v.armed==(action=="arm")) { result(id,"REJECTED","fresh_stopped_qloiter_required");return; }
      const auto replay=reserveControlRequest(m_host,id);
      if(replay!=ReplayResult::Reserved) {result(id,replayRejection(replay),replayReason(replay));return;}
      m_work=Work{id,action,{},action=="arm"?Step::Arm:Step::Disarm,mail.published_at+8};
    } else if(action=="run") {
      const auto proof=m_host.storedMissionProof();
      const double expires=mail.published_at+120.;
      if(values.size()!=6||!proof.valid||values["verification_id"]!=proof.verification_id||
         values["revision"]!=std::to_string(proof.revision)||values["content_id"]!=proof.content_id||
         m_host.now()>=expires) {
        result(id,"REJECTED","verified_plan_and_armed_qloiter_required");return;
      }
      const auto replay=reserveControlRequest(m_host,id);
      if(replay!=ReplayResult::Reserved) {result(id,replayRejection(replay),replayReason(replay));return;}
      if(!m_host.beginMissionExecution(id,proof.revision,proof.content_id,expires)) {
        result(id,"REJECTED","verified_plan_and_armed_qloiter_required");return;
      }
      m_retained=id;m_work=Work{id,action,{},Step::Select,expires};
    } else { result(id,"REJECTED","unsupported_action");return; }
    m_seen.insert(id);result(id,"QUEUED","admitted");
  }

  void onMessageWithMetadata(const mavlink_message_t& message,const MessageTime& time) override
  {
    if(message.sysid!=1||message.compid!=1||time.effective_at<=m_floor||
       !quadplane::fresh(time.effective_at,m_host.now(),2))return;
    m_backend.observeForward(message,time,m_host.vehicle(),m_host.now());
    if(message.msgid==MAVLINK_MSG_ID_EXTENDED_SYS_STATE)m_backend.observe(message,time.effective_at);
    if(message.msgid==MAVLINK_MSG_ID_MISSION_CURRENT&&message.magic==MAVLINK_STX&&message.len>=3&&
       time.effective_at>=m_cursor_at) {
      mavlink_mission_current_t cursor{};mavlink_msg_mission_current_decode(&message,&cursor);
      if(cursor.total!=quadplane::plan_items-1) {m_cursor=0;m_cursor_at=time.effective_at;return;}
      if(message.len>=5) {m_cursor=cursor.seq;m_cursor_at=time.effective_at;}
    }
  }
  void onCommandSent(const CommandRequest& request,double at,bool accepted) override
  {
    if(m_helm_enabled)m_helm->onCommandSent(request,at,accepted);
    if(!m_work||m_work->wire!=request.id)return;
    if(!accepted) { end("FAILED","local_handoff_failed");return; }
    m_work->sent=at;result(m_work->id,"SENT","independent_observation_required");
  }
  void onReset(const std::string& reason) override
  {
    if(m_work)end("UNCONFIRMED","connection_reset");
    m_backend.reset(m_host.now());m_cursor_at=-1;m_floor=m_host.now();
    if(m_helm_enabled)m_helm->onReset(reason);
  }
  void onInputBoundary() override
  {
    if(m_helm_enabled)m_helm->onInputBoundary();
    if(m_work&&m_host.now()>=m_work->deadline)end("TIMED_OUT","original_deadline");
  }
  void iterate() override
  {
    if(m_helm_enabled)m_helm->iterate();
    if(!m_work)return;
    const auto now=m_host.now();
    if(now>=m_work->deadline) { end("TIMED_OUT","original_deadline");return; }
    auto& work=*m_work;
    const auto& v=m_host.vehicle();
    if(work.step==Step::Release) {
      if(v.heartbeat_at>work.sent&&m_host.finishMissionExecution(m_retained,"RELEASED")) {
        m_retained.clear();end("COMPLETED","observed_disarmed_handback");
      }
      return;
    }
    if(work.action=="run"&&!m_host.missionExecutionGranted(work.id)) { end("UNCONFIRMED","execution_fenced");return; }
    if(work.step==Step::Flight) {
      advanceFlight(work,v,now);return;
    }
    if(!work.awaiting) { queue(work,now);return; }
    if(work.sent<0)return;
    if(work.step==Step::Select) {
      if(m_cursor_at>work.sent&&m_cursor==quadplane::takeoff_sequence)next(Step::Start);
      else if(now-work.sent>=5)end("UNCONFIRMED","cursor_selection_missing");
      return;
    }
    if(v.heartbeat_at>work.sent&&quadplane::fresh(v.heartbeat_at,now,2)) {
      if(work.step==Step::Start&&v.mode==quadplane::auto_mode&&v.armed) {
        next(Step::Flight);result(work.id,"CONFIRMED","native_auto_observed");return;
      }
      if(work.step==Step::Abort&&v.mode==quadplane::qrtl) { end("CONFIRMED","native_qrtl_observed_lease_retained");return; }
      if(work.step==Step::LandDisarm&&!v.armed) { next(Step::LandHold);return; }
      if(work.step==Step::LandHold&&!v.armed&&v.mode==quadplane::qloiter&&quadplane::stoppedOnGround(v,now)) {
        end("COMPLETED","hover_forward_hover_touchdown_disarm_observed");return;
      }
      if((work.step==Step::Arm||work.step==Step::Disarm)&&v.armed==(work.step==Step::Arm)&&
         v.mode==quadplane::qloiter&&quadplane::stoppedOnGround(v,now)) {
        end("COMPLETED","ground_arming_state_observed");return;
      }
    }
    if(now-work.sent>=5)end("UNCONFIRMED","mode_or_arming_observation_missing");
  }

private:
  void handback(const std::string& id,const std::string& action,double stamp)
  {
    const auto now=m_host.now();
    const auto& v=m_host.vehicle();
    if(action=="release"&&(m_work||v.armed||v.mode!=quadplane::qloiter||!quadplane::stoppedOnGround(v,now))) {
      result(id,"REJECTED","fresh_disarmed_qloiter_required");return;
    }
    const auto replay=reserveControlRequest(m_host,id);
    if(replay!=ReplayResult::Reserved) {result(id,replayRejection(replay),replayReason(replay));return;}
    if(!m_host.beginMissionExecutionAbort(id,m_retained,stamp+10)) {
      result(id,"REJECTED","host_handback_denied");return;
    }
    if(m_work)end("CANCELLED","explicit_stop_superseded_execution");
    m_seen.insert(id);
    m_work=Work{id,action,{},action=="abort"?Step::Abort:Step::Release,stamp+10};
    m_work->sent=now;result(id,"QUEUED","retained_execution_bound");
  }

  void advanceFlight(Work& work,const Vehicle& v,double now)
  {
    if(!quadplane::observed(v,now)||v.mode!=quadplane::auto_mode)return;
    const auto phase=m_backend.phase(now);
    if(work.flight_phase==0&&phase==MAV_VTOL_STATE_MC&&v.in_air&&v.relative_height>=8) {
      work.flight_phase=1;result(work.id,"ACTIVE","observed_airborne_hover");
    } else if(work.flight_phase==1&&phase==MAV_VTOL_STATE_FW&&v.in_air&&
              std::hypot(v.north_velocity,v.east_velocity)>=10) {
      work.flight_phase=2;result(work.id,"ACTIVE","observed_forward_flight");
    } else if(work.flight_phase==2&&phase==MAV_VTOL_STATE_MC&&v.in_air&&
              m_cursor==quadplane::land_sequence&&quadplane::fresh(m_cursor_at,now,2)) {
      work.flight_phase=3;result(work.id,"ACTIVE","observed_hover_on_landing_approach");
    } else if(work.flight_phase==3&&m_cursor==quadplane::land_sequence&&
              quadplane::fresh(m_cursor_at,now,2)&&quadplane::stoppedOnGround(v,now)) {
      next(v.armed?Step::LandDisarm:Step::LandHold);
    }
  }

  void queue(Work& work,double now)
  {
    CommandRequest request;
    request.id="quad/"+work.id+"/"+std::to_string(static_cast<unsigned>(work.step));
    request.expires=std::min(now+.4,work.deadline);request.operation_expires=work.deadline;
    if(work.action=="run"||work.step==Step::Abort)request.execution_id=m_retained;
    if(work.step==Step::Abort)request.abort_id=work.id;
    if(work.step==Step::Arm||work.step==Step::Disarm||work.step==Step::LandDisarm) {
      request.command=MAV_CMD_COMPONENT_ARM_DISARM;request.params[0]=work.step==Step::Arm?1:0;
    } else if(work.step==Step::Select) {
      request.command=MAV_CMD_DO_SET_MISSION_CURRENT;request.params[0]=quadplane::takeoff_sequence;request.params[1]=1;
    } else if(work.step==Step::Start)request.command=MAV_CMD_MISSION_START;
    else {
      request.command=MAV_CMD_DO_SET_MODE;request.params[0]=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
      request.params[1]=work.step==Step::Abort?quadplane::qrtl:quadplane::qloiter;
    }
    if(!m_host.sendCommand(request)) { end("REJECTED","final_gate_denied");return; }
    work.wire=request.id;work.awaiting=true;
  }

  void next(Step step)
  {
    if(!m_work->wire.empty())m_host.commandFinished(m_work->wire);
    m_work->step=step;m_work->wire.clear();m_work->awaiting=false;m_work->sent=-1;
  }
  void end(const char* status,const char* reason)
  {
    const auto work=std::move(*m_work);m_work.reset();
    if(!work.wire.empty())m_host.commandFinished(work.wire);
    if(work.action=="run") {
      const std::string disposition=status;
      if(m_host.finishMissionExecution(work.id,disposition)&&disposition=="REJECTED")m_retained.clear();
    }
    result(work.id,status,reason);
  }
  void result(const std::string& id,const char* status,const char* reason)
  { m_host.publish(result_key,"schema=1,id="+id+",status="+status+",reason="+reason); }

  Services& m_host;
  Backend m_backend;
  std::unique_ptr<Plugin> m_helm;
  std::optional<Work> m_work;
  std::set<std::string> m_seen;
  std::string m_retained;
  double m_floor,m_cursor_at=-1;
  unsigned m_cursor=0;
  bool m_enabled=false,m_helm_enabled=false,m_hover_enabled=false;
};
}
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi,mavmoos::runtime::Services* services)
{ return abi==mavmoos::runtime::plugin_abi&&services?new mavmoos::runtime::Control(*services):nullptr; }
extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin) { delete plugin; }
