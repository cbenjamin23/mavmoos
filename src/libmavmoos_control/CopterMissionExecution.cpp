#include "ControlReplayService.h"
#include "CopterMissionExecution.h"
#include "CopterMissionPolicy.h"
#include "CopterFrame.h"
#include <algorithm>
#include <cmath>
#include <cctype>
#include <deque>
#include <iomanip>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>

namespace mavmoos::runtime {
namespace {
bool finite(double x){return std::isfinite(x);}
bool id(const std::string& x){return !x.empty()&&x.size()<=40&&std::all_of(x.begin(),x.end(),[](unsigned char c){return std::isalnum(c)||c=='_'||c=='-'||c=='.';});}
bool token(const std::string& x){return !x.empty()&&x.size()<=128&&std::all_of(x.begin(),x.end(),[](unsigned char c){return c>32&&c<127&&c!=','&&c!='=';});}
bool numeric(const std::string& x,double& out){try{std::size_t n=0;out=std::stod(x,&n);return n==x.size()&&finite(out);}catch(...){return false;}}
double separation(double a,double b,double x,double y){constexpr double r=3.14159265358979323846/180.;return 6371000.*std::hypot((x-a)*r,(y-b)*r*std::cos(a*r));}
bool fresh(bool valid,double at,double now,double bound){return valid&&finite(at)&&at>0&&at<=now&&now-at<=bound;}
struct Request {std::string id,action,target,verification,content;uint64_t revision=0;double published=0,expires=0;};
enum class Stage {Select,Start,Pause,Resume,Abort};
struct Phase {Request owner;CommandRequest wire;Stage stage;bool sent=false,ack_observed=false;double sent_at=0;unsigned cursor=0;};
struct Execution {
  Request request;
  StoredMissionProof proof;
  std::string state="SELECTING",terminal={};
  bool handed=false,active_seen=false,seen_first=false,seen_last=false,last_reached=false,released=false;
  bool airborne_seen=false,landing_seen=false;
  double start_sent=-1,last_handed=-1,settled=-1,landing_reset=-1;
  bool abort_auto_seen=false,abort_auto_departed=false;
};
class Coordinator final:public Plugin {
public:
  explicit Coordinator(Services& services):m_services(services),m_boundary(services.now()),m_last_now(services.now()){}
  std::string name()const override{return "copter_mission_execution";}
  PluginDescriptor descriptor()const override {
    PluginDescriptor d;d.profile="arducopter_simple_auto_v1";
    if(m_enabled)for(const auto& action:{"start","pause","resume","abort","release"})d.capabilities.push_back({std::string("control.mission.")+action,"MAVMOOS_MISSION_CONTROL",std::string("schema=1,action=")+action,"request"});
    if(m_enabled)d.capabilities.push_back({"control.mission.progress","MAVMOOS_MISSION_EXECUTION_PROGRESS","","observation"});
    if(m_enabled)d.observation_streams={{MAVLINK_MSG_ID_MISSION_CURRENT,500000}};
    return d;
  }
  std::vector<std::string> subscriptions()const override{return m_enabled?std::vector<std::string>{"MAVMOOS_MISSION_CONTROL"}:std::vector<std::string>{};}
  void configure(const std::string& key,const std::string& value)override {
    if(key=="copter_frame") {
      const auto type=copterFrameType(value);
      if(!supportedCopterType(type))throw std::invalid_argument("unsupported Copter frame");
      m_frame_type=type;
    }
    if(key=="control_mission_execution") {if(value!="true"&&value!="false")throw std::invalid_argument("control_mission_execution must be true/false");m_enabled=value=="true";}
  }
  void onMail(const Mail& mail)override {
    if(!m_enabled||mail.key!="MAVMOOS_MISSION_CONTROL")return;
    Request r;std::string why;
    if(!parse(mail,r,why)){report(r,"REJECTED",why);return;}
    if(m_seen.count(r.id)){report(r,"DUPLICATE","id_already_seen");return;}
    if(m_seen.size()>=128||m_queue.size()>=4){report(r,"REJECTED","request_bound");return;}
    const auto replay=reserveControlRequest(m_services,r.id);
    if(replay!=ReplayResult::Reserved) {report(r,replayRejection(replay),replayReason(replay));return;}
    m_seen.insert(r.id);m_queue.push_back(r);
  }
  void onMessage(const mavlink_message_t& message,double at)override {onMessageWithMetadata(message,{at,at,"READ_ACQUISITION"});}
  void onMessageWithMetadata(const mavlink_message_t& message,const MessageTime& time)override {
    if(!m_enabled||!m_execution||message.sysid!=1||message.compid!=1||!metadata(time))return;
    // Protocol correlation uses the actual read. A legitimate fast reply can
    // retain a queue-empty observation floor from before this send. The ACK
    // still cannot replace strictly post-send effective state/progress evidence.
    if(message.msgid==MAVLINK_MSG_ID_COMMAND_ACK&&m_phase&&m_phase->sent&&!m_phase->ack_observed&&
       time.read_at>m_phase->sent_at&&m_services.now()<m_phase->wire.expires) {
      mavlink_command_ack_t ack{};mavlink_msg_command_ack_decode(&message,&ack);
      if(ack.command!=m_phase->wire.command||(ack.target_system!=0&&ack.target_system!=245)||(ack.target_component!=0&&ack.target_component!=191))return;
      m_phase->ack_observed=true;
      report(m_phase->owner,"ACK_OBSERVED","uncorrelated_ack_result_"+std::to_string(ack.result));
    } else if(message.msgid==MAVLINK_MSG_ID_MISSION_CURRENT&&message.magic==MAVLINK_STX&&message.len>=5) {
      if(m_current_valid&&time.effective_at<=m_current_time.effective_at)return;
      mavlink_mission_current_t current{};mavlink_msg_mission_current_decode(&message,&current);
      const auto& v=m_services.vehicle();
      const bool finishing_land=m_execution->landing_seen&&m_execution->terminal.empty()&&v.mode==3;
      // After the pinned firmware disarms it can report cursor zero briefly,
      // then its first item again while AUTO remains selected on the ground.
      const bool landing_reset=finishing_land&&current.seq<=1&&
        flightObservations(v,m_services.now())&&copter_mission::landingNearGround(v,m_execution->proof.items);
      const bool wrong_land_cursor=finishing_land&&!landing_reset&&current.seq!=m_execution->proof.items.size()-1;
      if(current.total!=m_execution->proof.items.size()-1||(!landing_reset&&current.seq<1)||
         current.seq>=m_execution->proof.items.size()||wrong_land_cursor){
        if(m_execution->handed&&time.effective_at>m_execution->last_handed)finish("LOST","observed_mission_shape_contradicts_bound_content");
        return;
      }
      if(landing_reset&&m_execution->landing_reset<0)m_execution->landing_reset=time.effective_at;
      m_current=current;m_current_time=time;m_current_valid=true;
      if(m_execution->start_sent>0&&time.effective_at>m_execution->start_sent&&current.mission_state==MISSION_STATE_ACTIVE&&m_services.vehicle().mode==3) {
        m_execution->active_seen=true;m_execution->seen_first|=current.seq==1;m_execution->seen_last|=current.seq==m_execution->proof.items.size()-1;
        if(m_execution->airborne_seen&&current.seq==m_execution->proof.items.size()-1&&
           copter_mission::landingPlan(m_execution->proof.items)&&copter_mission::airbornePhase(m_services.vehicle()))
          m_execution->landing_seen=true;
      }
      publishProgress();
    } else if(message.msgid==MAVLINK_MSG_ID_MISSION_ITEM_REACHED&&m_execution->start_sent>0&&time.effective_at>m_execution->start_sent) {
      mavlink_mission_item_reached_t reached{};mavlink_msg_mission_item_reached_decode(&message,&reached);
      if(reached.seq==m_execution->proof.items.size()-1)m_execution->last_reached=true;
    }
  }
  void onCommandSent(const CommandRequest& request,double at,bool accepted)override {
    if(!m_phase||m_phase->wire.id!=request.id) {
      if(accepted&&m_retired.count(request.id)&&m_execution&&m_retired[request.id]==m_execution->request.id){
        m_execution->handed=true;m_execution->last_handed=at;m_execution->released=false;
        report(m_execution->request,"LATE_HANDOFF","retired_command_accepted_remote_effect_uncertain");
        m_services.finishMissionExecution(m_execution->request.id,"LOST");
        finish("LOST","late_retired_command_receipt",false);
      }
      return;
    }
    if(accepted){m_execution->handed=true;m_execution->last_handed=at;}
    if(!accepted){phaseFailure("FAILED","local_send_rejected");return;}
    if(!finite(at)||at<m_boundary||at>m_services.now()||at>=m_phase->wire.expires||m_services.now()>=m_phase->wire.expires){phaseFailure("TIMED_OUT","send_receipt_after_original_deadline");return;}
    m_phase->sent=true;m_phase->sent_at=at;
    if(m_phase->stage==Stage::Start)m_execution->start_sent=at;
    report(m_phase->owner,"SENT","local_os_acceptance_not_remote_execution");
  }
  void onReset(const std::string&)override {
    for(const auto& r:m_queue)report(r,"CANCELLED","session_reset");m_queue.clear();
    if(m_phase&&m_phase->stage==Stage::Abort)report(m_phase->owner,"CANCELLED","session_reset_remote_stop_unconfirmed");
    retire(false);
    if(m_execution)m_execution->abort_auto_departed=true;
    finish("LOST","session_reset");m_current_valid=false;m_boundary=m_services.now();m_last_now=m_boundary;
  }
  void onInputBoundary()override {check();}
  void iterate()override {
    if(!m_enabled)return;
    check();
    if(m_clock_fault){status();return;}
    if(!m_queue.empty()){const auto request=m_queue.front();m_queue.pop_front();admit(request);}
    if(m_phase&&m_phase->sent)advance();
    if(m_execution&&!m_execution->released&&m_execution->terminal.empty()&&m_execution->state=="RUNNING")complete();
    status();
  }
private:
  bool heartbeat(const Vehicle& v,double now)const {
    return v.type==m_frame_type && v.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA &&
      fresh(v.heartbeat_valid,v.heartbeat_at,now,1.5);
  }
  bool flightObservations(const Vehicle& v,double now)const {
    return heartbeat(v,now) && v.home_valid && fresh(v.position_valid,v.position_at,now,.5) &&
      fresh(v.landed_valid,v.landed_at,now,1.5) && fresh(v.estimator_valid,v.estimator_at,now,1) &&
      v.estimator_horizontal_valid && finite(v.latitude) && std::abs(v.latitude)<=90 &&
      finite(v.longitude) && std::abs(v.longitude)<=180 && finite(v.relative_height) &&
      finite(v.home_latitude) && std::abs(v.home_latitude)<=90 && finite(v.home_longitude) &&
      std::abs(v.home_longitude)<=180 && finite(v.home_msl) && finite(v.north_velocity) && finite(v.east_velocity);
  }
  bool metadata(const MessageTime& t)const{return finite(t.effective_at)&&finite(t.read_at)&&t.effective_at>m_boundary&&t.effective_at<=t.read_at&&t.read_at<=m_services.now()&&m_services.now()-t.effective_at<=2.5&&(t.age_basis=="READ_ACQUISITION"||t.age_basis=="QUEUE_EMPTY_FLOOR");}
  bool parse(const Mail& mail,Request& r,std::string& why)const {
    why="invalid_request";if(mail.numeric||mail.text.empty()||mail.text.size()>768)return false;
    std::map<std::string,std::string> f;std::istringstream in(mail.text);std::string part;
    while(std::getline(in,part,',')){auto at=part.find('=');if(at==std::string::npos||!f.emplace(part.substr(0,at),part.substr(at+1)).second)return false;}
    r.id=f["id"];r.action=f["action"];if(!id(r.id)||f["schema"]!="1"||mail.text.back()==',')return false;
    if(r.action!="start"&&r.action!="pause"&&r.action!="resume"&&r.action!="abort"&&r.action!="release"){why="unsupported_action";return false;}
    const double now=m_services.now(),age=mail.received_at-mail.published_at;
    if(m_clock_fault||!finite(now)||!finite(mail.published_at)||!finite(mail.received_at)||mail.published_at<=m_boundary||age<-.1||age>2||mail.received_at>now+.1||now-mail.received_at>2){why="stale_future_or_pre_session_request";return false;}
    double timeout=r.action=="start"?90:8;if(f.count("timeout")&&!numeric(f["timeout"],timeout))return false;
    if(timeout<1||timeout>(r.action=="start"?120:15))return false;
    r.published=mail.published_at;r.expires=r.published+timeout;if(r.expires<=now){why="original_deadline";return false;}
    std::set<std::string> allowed={"schema","id","action","timeout"};
    if(r.action=="start") {
      allowed.insert({"verification_id","content_id","proof_revision"});r.verification=f["verification_id"];r.content=f["content_id"];
      const auto value=f["proof_revision"];if(!id(r.verification)||!token(r.content)||value.empty()||!std::all_of(value.begin(),value.end(),[](unsigned char c){return c>='0'&&c<='9';}))return false;
      try{r.revision=std::stoull(value);}catch(...){return false;}if(!r.revision)return false;
    } else {allowed.insert("target");r.target=f["target"];if(!id(r.target))return false;}
    for(const auto& field:f)if(!allowed.count(field.first))return false;
    return true;
  }
  bool validProof(const StoredMissionProof& p)const {
    return p.valid&&p.session_epoch&&p.revision&&finite(p.verified_at)&&p.verified_at>0&&
      p.verified_at<=m_services.now()&&p.items.size()>=3&&copter_mission::validPlan(p.items,m_services.vehicle());
  }
  bool currentAfter(double floor,unsigned state)const{return m_current_valid&&m_current_time.effective_at>floor&&m_services.now()-m_current_time.effective_at<=2.5&&m_current.mission_state==state;}
  void admit(const Request& r) {
    if(m_services.now()>=r.expires){report(r,"REJECTED","original_deadline");return;}
    if(r.action=="start") {
      if((m_execution&&!m_execution->released)||m_phase){report(r,"REJECTED","execution_or_handoff_busy");return;}
      const auto proof=m_services.storedMissionProof();const auto& v=m_services.vehicle();
      if(!flightObservations(v,m_services.now())||!validProof(proof)||!copter_mission::entryState(v,proof.items)||
         proof.revision!=r.revision||proof.content_id!=r.content||proof.verification_id!=r.verification){report(r,"REJECTED","current_verified_guided_entry_required");return;}
      if(!m_services.beginMissionExecution(r.id,proof.revision,proof.content_id,r.expires)){report(r,"REJECTED","host_execution_lease_denied");return;}
      m_execution=Execution{r,proof};m_current_valid=false;report(r,"ADMITTED","typed_stored_mission_bound");issue(Stage::Select,r);return;
    }
    if(!m_execution||m_execution->released||r.target!=m_execution->request.id){report(r,"REJECTED","target_not_retained");return;}
    if(r.action=="pause") {
      if(m_phase||!m_execution->terminal.empty()||m_execution->state!="RUNNING"||!m_services.vehicle().in_air||
         !m_services.missionExecutionGranted(r.target)){report(r,"REJECTED","airborne_running_execution_required");return;}
      issue(Stage::Pause,r);return;
    }
    if(r.action=="resume") {
      if(m_phase||!m_execution->terminal.empty()||m_execution->state!="PAUSED"||!m_services.vehicle().in_air||
         !currentAfter(m_execution->start_sent,MISSION_STATE_PAUSED)||!m_services.missionExecutionGranted(r.target)){
        report(r,"REJECTED","fresh_paused_execution_required");return;}
      issue(Stage::Resume,r);return;
    }
    if(r.action=="release") {
      const auto& v=m_services.vehicle();if(m_phase||!fresh(v.heartbeat_valid,v.heartbeat_at,m_services.now(),1.5)||v.mode==3||v.heartbeat_at<=m_execution->last_handed||!m_services.beginMissionExecutionAbort(r.id,r.target,r.expires)){report(r,"REJECTED","fresh_non_auto_explicit_release_required");return;}
      if(!m_services.finishMissionExecution(r.target,"RELEASED")){report(r,"FAILED","host_release_not_confirmed");finish("LOST","host_release_not_confirmed");return;}
      if(m_execution->terminal.empty())finish("CANCELLED","explicit_non_auto_release",false);
      m_execution->released=true;m_execution->state="RELEASED";report(r,"COMPLETED","observed_non_auto_no_automatic_helm_acquire");return;
    }
    if(m_phase&&m_phase->stage==Stage::Abort){report(r,"REJECTED","abort_pending");return;}
    const auto& v=m_services.vehicle();
    if(!heartbeat(v,m_services.now())||v.mode!=3||!m_execution->abort_auto_seen||m_execution->abort_auto_departed) {
      report(r,"REJECTED","fresh_owned_auto_required_use_explicit_non_auto_release");return;
    }
    if(!m_services.beginMissionExecutionAbort(r.id,r.target,r.expires)){report(r,"REJECTED","host_stop_only_lease_denied");return;}
    retire(false);m_execution->state="ABORTING";report(r,"ADMITTED","stop_only_authority_no_old_deadline_renewal");issue(Stage::Abort,r);
  }
  void issue(Stage stage,const Request& owner) {
    Phase p;p.owner=owner;p.stage=stage;p.wire.id="mx."+std::to_string(++m_wire_counter);p.wire.execution_id=m_execution->request.id;
    p.wire.operation_expires=stage==Stage::Abort?owner.expires:m_execution->request.expires;p.wire.expires=std::min({m_services.now()+5,owner.expires,p.wire.operation_expires});
    if(stage==Stage::Select){p.wire.command=MAV_CMD_DO_SET_MISSION_CURRENT;p.wire.params[0]=1;}
    else if(stage==Stage::Start)p.wire.command=MAV_CMD_MISSION_START;
    else if(stage==Stage::Pause||stage==Stage::Resume) {
      p.wire.command=MAV_CMD_DO_PAUSE_CONTINUE;p.wire.params[0]=stage==Stage::Resume?1:0;
      p.cursor=m_current.seq;
    }
    else{p.wire.command=MAV_CMD_DO_SET_MODE;p.wire.params[0]=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;p.wire.params[1]=4;p.wire.abort_id=owner.id;}
    const auto request=p.wire;m_phase=p;
    if(!m_services.sendCommand(request)&&m_phase&&m_phase->wire.id==request.id)phaseFailure("FAILED","host_queue_admission_denied");
    else if(m_phase&&m_phase->wire.id==request.id)report(owner,"SUBMITTED","bounded_host_queue_not_wire_receipt");
  }
  void retire(bool resolved) {
    if(!m_phase)return;const auto wire=m_phase->wire;
    if(m_retired.size()<256)m_retired[wire.id]=wire.execution_id;
    if(resolved)m_services.commandFinished(wire.id);else m_services.cancelCommand(wire.id);
    m_phase.reset();
  }
  void phaseFailure(const std::string& state,const std::string& why) {
    if(!m_phase)return;const auto owner=m_phase->owner;const auto stage=m_phase->stage;
    retire(false);if(stage==Stage::Pause||stage==Stage::Resume||stage==Stage::Abort)report(owner,state,why);
    if(stage==Stage::Select&&!m_execution->handed) {
      const bool released=m_services.finishMissionExecution(m_execution->request.id,"REJECTED");
      finish(state=="TIMED_OUT"?"TIMED_OUT":"REJECTED",why,false);m_execution->released=released;return;
    }
    finish(state=="TIMED_OUT"?"TIMED_OUT":"FAILED",why);
  }
  void advance() {
    if(!m_phase)return;const auto p=*m_phase;const auto& v=m_services.vehicle();
    if(p.stage==Stage::Select&&v.mode==4&&m_current.seq==1&&(currentAfter(p.sent_at,MISSION_STATE_NOT_STARTED)||currentAfter(p.sent_at,MISSION_STATE_PAUSED))) {
      retire(true);m_execution->state="STARTING";issue(Stage::Start,m_execution->request);
    } else if(p.stage==Stage::Start&&v.mode==3&&v.heartbeat_at>p.sent_at&&currentAfter(p.sent_at,MISSION_STATE_ACTIVE)) {
      retire(true);m_execution->state="RUNNING";report(m_execution->request,"ACTIVE","post_start_auto_and_mission_progress");
    } else if(p.stage==Stage::Pause&&v.mode==3&&currentAfter(p.sent_at,MISSION_STATE_PAUSED)) {
      retire(true);m_execution->state="PAUSED";report(p.owner,"COMPLETED","observed_auto_pause_not_mode_exit");report(m_execution->request,"PAUSED","explicit_pause_no_automatic_resume");
    } else if(p.stage==Stage::Resume&&v.mode==3&&currentAfter(p.sent_at,MISSION_STATE_ACTIVE)&&m_current.seq==p.cursor) {
      retire(true);m_execution->state="RUNNING";report(p.owner,"COMPLETED","observed_active_same_cursor");
      report(m_execution->request,"ACTIVE","explicit_same_cursor_resume");
    } else if(p.stage==Stage::Abort&&v.mode==4&&v.heartbeat_at>p.sent_at&&fresh(v.heartbeat_valid,v.heartbeat_at,m_services.now(),1.5)) {
      retire(true);
      if(!m_services.finishMissionExecution(m_execution->request.id,"RELEASED")){report(p.owner,"FAILED","host_release_not_confirmed");finish("LOST","host_release_not_confirmed");return;}
      if(m_execution->terminal.empty())finish("CANCELLED","observed_guided_handback",false);
      m_execution->released=true;m_execution->state="RELEASED";report(p.owner,"COMPLETED","observed_guided_no_automatic_helm_acquire");
    }
  }
  void complete() {
    auto& e=*m_execution;const auto& v=m_services.vehicle();const auto& last=e.proof.items.back();
    if(copter_mission::landingPlan(e.proof.items)) {
      // The pinned firmware can disarm during NAV_LAND before emitting a
      // final reached/COMPLETE frame. Bind the active final cursor, then the effect.
      const bool landing_cursor=m_current.seq==last.seq||(e.landing_reset>=0&&m_current.seq<=1);
      const bool landed=e.active_seen&&e.seen_first&&e.landing_seen&&landing_cursor&&v.mode==3&&!v.armed&&
        copter_mission::groundedAtLandingPoint(v,e.proof.items);
      if(!landed){e.settled=-1;return;}if(e.settled<0)e.settled=m_services.now();
      if(m_services.now()-e.settled>=1)finish("COMPLETED","bound_landing_and_disarmed_dwell_observed_lease_retained");
      return;
    }
    const bool arrived=e.active_seen&&e.seen_first&&e.seen_last&&e.last_reached&&currentAfter(e.start_sent,MISSION_STATE_COMPLETE)&&m_current.seq==last.seq&&v.mode==3&&separation(v.latitude,v.longitude,last.x*1e-7,last.y*1e-7)<=2&&std::abs(v.relative_height-last.z)<=.5&&std::hypot(v.north_velocity,v.east_velocity)<=.5;
    if(!arrived){e.settled=-1;return;}if(e.settled<0)e.settled=m_services.now();
    if(m_services.now()-e.settled>=1)finish("COMPLETED","fresh_complete_final_reached_and_settled_auto_lease_retained");
  }
  void finish(const std::string& terminal,const std::string& why,bool notify_host=true) {
    if(!m_execution||m_execution->released)return;
    if(m_phase&&m_phase->stage!=Stage::Abort)retire(false);
    auto& e=*m_execution;if(e.terminal.empty()){e.terminal=terminal;e.state=terminal;report(e.request,terminal,why);if(notify_host)m_services.finishMissionExecution(e.request.id,terminal);}
  }
  void check() {
    const double now=m_services.now();if(!finite(now)||now<m_last_now){
      m_clock_fault=true;
      for(const auto& r:m_queue)report(r,"CANCELLED","owner_clock_fault");m_queue.clear();
      if(m_phase&&m_phase->stage==Stage::Abort)report(m_phase->owner,"FAILED","owner_clock_fault_remote_stop_unconfirmed");
      retire(false);finish("LOST","owner_clock_fault");return;
    }m_last_now=now;
    if(!m_execution||m_execution->released)return;
    const auto& v=m_services.vehicle();
    if(m_execution->start_sent>0&&heartbeat(v,now)&&v.heartbeat_at>m_execution->start_sent) {
      if(v.mode==3)m_execution->abort_auto_seen=true;
      else if(m_execution->abort_auto_seen)m_execution->abort_auto_departed=true;
    }
    if(m_phase&&now>=m_phase->wire.expires)phaseFailure("TIMED_OUT","original_phase_deadline");
    if(m_execution->terminal.empty()&&now>=m_execution->request.expires)finish("TIMED_OUT","original_execution_deadline_auto_may_continue");
    if(!m_execution->terminal.empty()&&!m_phase)return;
    if(m_phase&&m_phase->stage==Stage::Abort){if(!heartbeat(v,now))phaseFailure("FAILED","abort_heartbeat_lost");return;}
    if(m_execution->terminal.empty()) {
      const auto proof=m_services.storedMissionProof();
      if(!m_services.missionExecutionGranted(m_execution->request.id)||!proof.valid||proof.session_epoch!=m_execution->proof.session_epoch||proof.revision!=m_execution->proof.revision||proof.content_id!=m_execution->proof.content_id){finish("LOST","session_proof_or_authority_lost_auto_may_continue");return;}
      const auto& items=m_execution->proof.items;
      const bool native_flight=v.mode==3&&((v.armed&&
        (copter_mission::airbornePhase(v)||(copter_mission::takeoffPlan(items)&&!m_execution->airborne_seen&&copter_mission::ground(v))))||
        (m_execution->landing_seen&&copter_mission::groundedAtLandingPoint(v,items)));
      if(!flightObservations(v,now)||(!native_flight&&!copter_mission::entryState(v,items))) {
        finish("LOST","vehicle_freshness_or_mode_takeover");return;}
      if(m_execution->landing_reset>=0&&now-m_execution->landing_reset>2&&
         (v.armed||!copter_mission::groundedAtLandingPoint(v,items))) {
        finish("LOST","landing_cursor_reset_without_disarmed_ground");return;
      }
      if(v.mode==3&&v.armed&&v.in_air)m_execution->airborne_seen=true;
      if((m_execution->state=="RUNNING"||m_execution->state=="PAUSED")&&(v.mode!=3||!m_current_valid||now-m_current_time.effective_at>2.5))finish("LOST","auto_mode_or_progress_lost");
    }
  }
  void report(const Request& r,const std::string& state,const std::string& reason) {
    std::ostringstream out;out<<std::fixed<<std::setprecision(9)<<"schema=1,id="<<r.id<<",action="<<r.action<<",target="<<(r.target.empty()?"none":r.target)<<",status="<<state<<",reason="<<reason<<",expires="<<r.expires<<",at="<<m_services.now()<<",remote_execution_may_continue="<<(m_execution&&m_execution->handed&&!m_execution->released?"true":"false")<<",wire_operation_id=false,content_binding=host_local_verified_items";
    if(m_execution&&r.id==m_execution->request.id)out<<",verification_id="<<m_execution->proof.verification_id<<",content_id="<<m_execution->proof.content_id<<",proof_revision="<<m_execution->proof.revision<<",session_epoch="<<m_execution->proof.session_epoch;
    m_services.publish("MAVMOOS_MISSION_EXECUTION_RESULT",out.str());
  }
  void publishProgress(){std::ostringstream out;out<<std::setprecision(16)<<"schema=1,id="<<m_execution->request.id<<",seq="<<m_current.seq<<",total="<<m_current.total<<",mission_state="<<unsigned(m_current.mission_state)<<",effective_at="<<m_current_time.effective_at<<",read_at="<<m_current_time.read_at<<",source_age=unknown,wire_operation_id=false";m_services.publish("MAVMOOS_MISSION_EXECUTION_PROGRESS",out.str());}
  void status(){std::ostringstream out;out<<"schema=1,enabled="<<(m_enabled?"true":"false")<<",id="<<(m_execution?m_execution->request.id:"none")<<",phase="<<(m_execution?m_execution->state:"IDLE")<<",lease_retained="<<(m_execution&&!m_execution->released?"true":"false")<<",resume_supported=true,source_age=unknown";m_services.publish("MAVMOOS_MISSION_EXECUTION_STATUS",out.str());}
  Services& m_services;bool m_enabled=false,m_clock_fault=false;double m_boundary,m_last_now;uint64_t m_wire_counter=0;
  unsigned m_frame_type=MAV_TYPE_QUADROTOR;
  std::deque<Request> m_queue;std::set<std::string> m_seen;std::optional<Execution> m_execution;std::optional<Phase> m_phase;
  std::map<std::string,std::string> m_retired;bool m_current_valid=false;mavlink_mission_current_t m_current{};MessageTime m_current_time{};
};
}
std::unique_ptr<Plugin> makeCopterMissionExecution(Services& services){return std::make_unique<Coordinator>(services);}
} // namespace mavmoos::runtime
