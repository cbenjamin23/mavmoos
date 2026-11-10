#include "ControlReplayService.h"
#include "RoverComponents.h"
#include "ArduRoverMissionPolicy.h"
#include <deque>
#include <iomanip>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace mavmoos::runtime {
namespace {
namespace mission=ardu_rover_mission;
constexpr char request_key[]="MAVMOOS_MISSION_CONTROL";
constexpr char result_key[]="MAVMOOS_MISSION_EXECUTION_RESULT";
constexpr char status_key[]="MAVMOOS_MISSION_EXECUTION_STATUS";
constexpr char progress_key[]="MAVMOOS_MISSION_EXECUTION_PROGRESS";

enum class Stage { Select, Start, Pause, Resume, Stop };
enum class State { Selecting, Starting, Running, Pausing, Paused, Resuming, Stopping, Completed, Uncertain };

const char* stateName(State state)
{
  switch(state) {
    case State::Selecting: return "SELECTING";
    case State::Starting: return "STARTING";
    case State::Running: return "RUNNING";
    case State::Pausing: return "PAUSING";
    case State::Paused: return "PAUSED";
    case State::Resuming: return "RESUMING";
    case State::Stopping: return "STOPPING";
    case State::Completed: return "COMPLETED";
    case State::Uncertain: return "UNCERTAIN";
  }
  return "UNKNOWN";
}

bool id(std::string_view value)
{
  return !value.empty() && value.size()<=40 &&
    std::all_of(value.begin(),value.end(),[](unsigned char c) {
      return (c>='a'&&c<='z') || (c>='A'&&c<='Z') || (c>='0'&&c<='9') || c=='_' || c=='-' || c=='.';
    });
}

struct Request {
  std::string id,action,target,verification,content;
  uint64_t revision=0;
  double expires=0;
};

bool parse(const Mail& mail,double now,double boundary,Request& request,std::string& why)
{
  why="invalid_request";
  if(mail.numeric || mail.text.empty() || mail.text.size()>768 || mail.text.back()==',') return false;
  std::map<std::string,std::string> fields;
  std::istringstream input(mail.text);
  for(std::string field;std::getline(input,field,',');) {
    const auto at=field.find('=');
    if(at==std::string::npos || !fields.emplace(field.substr(0,at),field.substr(at+1)).second) return false;
  }
  request.id=fields["id"];request.action=fields["action"];
  if(!id(request.id) || fields["schema"]!="1") return false;
  const bool start=request.action=="start";
  if(!start && request.action!="pause" && request.action!="resume" &&
     request.action!="abort" && request.action!="release") { why="unsupported_action"; return false; }
  const auto age=mail.received_at-mail.published_at;
  if(!std::isfinite(now) || !std::isfinite(mail.published_at) || !std::isfinite(mail.received_at) ||
     mail.published_at<=boundary || age<-.1 || age>2 ||
     mail.received_at>now+.1 || now-mail.received_at>2) {
    why="stale_future_or_pre_session_request";return false;
  }
  double timeout=start?100:12;
  if(const auto it=fields.find("timeout");it!=fields.end()) {
    try {
      std::size_t parsed=0;
      timeout=std::stod(it->second,&parsed);
      if(parsed!=it->second.size()) return false;
    } catch(const std::exception&) { return false; }
  }
  if(!std::isfinite(timeout) || timeout<1 || timeout>(start?120:15)) return false;
  request.expires=mail.published_at+timeout;
  if(now>=request.expires) { why="original_deadline";return false; }
  std::set<std::string> allowed={"schema","id","action","timeout"};
  if(start) {
    allowed.insert({"verification_id","content_id","proof_revision"});
    request.verification=fields["verification_id"];request.content=fields["content_id"];
    const auto& revision=fields["proof_revision"];
    if(!id(request.verification) || request.content.empty() || request.content.size()>128 ||
       !std::all_of(request.content.begin(),request.content.end(),[](unsigned char c){return c>32&&c<127&&c!='=';}) ||
       revision.empty() || !std::all_of(revision.begin(),revision.end(),[](unsigned char c){return c>='0'&&c<='9';}))
      return false;
    try { request.revision=std::stoull(revision); } catch(const std::exception&) { return false; }
    if(!request.revision) return false;
  } else {
    allowed.insert("target");request.target=fields["target"];
    if(!id(request.target)) return false;
  }
  return std::all_of(fields.begin(),fields.end(),[&](const auto& field){return allowed.count(field.first);});
}

struct Phase {
  Request request;
  CommandRequest command;
  Stage stage=Stage::Select;
  double sent_at=-1;
  bool ack_observed=false;
};
struct Execution {
  Request request;
  StoredMissionProof proof;
  State state=State::Selecting;
  double handoff=-1,running_after=-1,final_reached=-1;
  unsigned paused_seq=0,active_seq=0;
};
struct Cursor {
  mavlink_mission_current_t packet{};
  double at=-1;
};

class ArduRoverMission final : public Plugin {
public:
  ArduRoverMission(Services& host,const VehicleControlBackend& backend)
    :m_host(host),m_policy(dynamic_cast<const ArduRoverMissionPolicy&>(backend)),
     m_boat(backend.profile()=="ardurover_boat"),m_boundary(host.now()),m_last_now(host.now()) {}
  std::string name() const override {return "ardurover_mission";}
  PluginDescriptor descriptor() const override
  {
    PluginDescriptor d;
    if(!m_enabled) return d;
    d.request_owners={request_key};
    for(const auto* action:{"start","pause","resume","abort","release"})
      d.capabilities.push_back({std::string("control.mission.")+action,request_key,std::string("schema=1,action=")+action});
    d.capabilities.push_back({"control.mission.progress",progress_key,"","observation"});
    d.observation_streams={{MAVLINK_MSG_ID_MISSION_CURRENT,500000.f}};
    return d;
  }
  std::vector<std::string> subscriptions() const override
  { return m_enabled?std::vector<std::string>{request_key}:std::vector<std::string>{}; }
  void configure(const std::string& key,const std::string& value) override
  {
    if(key!="control_mission_execution") return;
    if(value!="true" && value!="false") throw std::invalid_argument(key+" must be true/false");
    m_enabled=value=="true";
  }
  void onMail(const Mail& mail) override
  {
    if(!m_enabled || mail.key!=request_key) return;
    Request request;
    std::string why;
    if(m_clock_fault || !parse(mail,m_host.now(),m_boundary,request,why)) {
      report(request,"REJECTED",m_clock_fault?"owner_clock_fault":why);
      return;
    }
    if(m_seen.count(request.id)) { report(request,"DUPLICATE","id_already_seen");return; }
    if(m_seen.size()>=128 || m_queue.size()>=4) { report(request,"REJECTED","request_bound");return; }
    const auto replay=reserveControlRequest(m_host,request.id);
    if(replay!=ReplayResult::Reserved) {report(request,replayRejection(replay),replayReason(replay));return;}
    m_seen.insert(request.id);
    m_queue.push_back(std::move(request));
  }
  void onMessageWithMetadata(const mavlink_message_t& message,const MessageTime& time) override
  {
    if(!m_enabled || message.sysid!=1 || message.compid!=1 || message.magic!=MAVLINK_STX ||
       !heartbeat() || time.effective_at<=m_boundary || !mission::fresh(time.effective_at,m_host.now(),2.5) ||
       !std::isfinite(time.read_at) || time.read_at<time.effective_at || time.read_at>m_host.now() ||
       (time.age_basis!="READ_ACQUISITION" && time.age_basis!="QUEUE_EMPTY_FLOOR")) return;
    if(message.msgid==MAVLINK_MSG_ID_COMMAND_ACK) observeAck(message,time.read_at);
    if(message.msgid==MAVLINK_MSG_ID_MISSION_CURRENT && message.len>=5 && time.effective_at>m_cursor.at) {
      mavlink_msg_mission_current_decode(&message,&m_cursor.packet);
      m_cursor.at=time.effective_at;
      observeProgress();
    }
    if(message.msgid==MAVLINK_MSG_ID_MISSION_ITEM_REACHED && m_execution &&
       m_execution->state==State::Running && time.effective_at>m_execution->running_after) {
      mavlink_mission_item_reached_t reached{};
      mavlink_msg_mission_item_reached_decode(&message,&reached);
      if(reached.seq==m_execution->proof.items.back().seq && m_execution->final_reached<0)
        m_execution->final_reached=time.effective_at;
    }
  }
  void onCommandSent(const CommandRequest& request,double at,bool accepted) override
  {
    if(!m_execution || request.execution_id!=m_execution->request.id || request.id.rfind("armx.",0)!=0) return;
    if(accepted) m_execution->handoff=std::max(m_execution->handoff,m_host.now());
    if(!m_phase || m_phase->command.id!=request.id) {
      if(accepted) uncertain("late_retired_handoff");
      return;
    }
    if(!accepted) { fail("FAILED","local_send_rejected");return; }
    if(!std::isfinite(at) || at<=m_boundary || at>m_host.now() || at>=request.expires || m_host.now()>=request.expires) {
      fail("TIMED_OUT","handoff_outside_original_window");return;
    }
    m_phase->sent_at=at;
    report(m_phase->request,"SENT","awaiting_independent_vehicle_evidence");
  }
  void onReset(const std::string&) override
  {
    for(const auto& request:m_queue) report(request,"CANCELLED","session_reset");
    m_queue.clear();
    uncertain("session_reset");
    m_boundary=m_last_now=m_host.now();
    m_cursor={};
  }
  void onInputBoundary() override { if(m_enabled) check(); }
  void iterate() override
  {
    if(!m_enabled) return;
    check();
    if(!m_clock_fault) {
      if(!m_queue.empty()) {
        const auto request=m_queue.front();
        m_queue.pop_front();
        admit(request);
      }
      if(m_phase && m_phase->sent_at>=0) advance();
      if(m_execution && m_execution->state==State::Running) complete();
    }
    std::ostringstream out;
    out<<"schema=1,enabled=true,id="<<(m_execution?m_execution->request.id:"none")
       <<",phase="<<(m_execution?stateName(m_execution->state):"IDLE")
       <<",lease_retained="<<(m_execution?"true":"false")<<",resume_supported=true"
       <<",upload_ready="<<(!m_clock_fault && !m_execution && !m_host.vehicle().armed &&
          !m_host.controlBusy() && !m_host.navigationGranted() && settled(-1) &&
          m_host.vehicle().home_valid && mission::fresh(m_host.vehicle().home_at,m_host.now(),10)?"true":"false")
       <<",vehicle_ready="<<(m_policy.ready(m_host.vehicle(),m_host.now())?"true":"false")
       <<",start_ready="
       <<(!m_clock_fault && !m_execution && startReady(m_host.storedMissionProof())?"true":"false");
    m_host.publish(status_key,out.str());
  }

private:
  bool heartbeat() const
  {
    const auto& v=m_host.vehicle();
    return v.heartbeat_valid && mission::fresh(v.heartbeat_at,m_host.now(),2) &&
      v.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA &&
      v.type==(m_boat?MAV_TYPE_SURFACE_BOAT:MAV_TYPE_GROUND_ROVER);
  }
  bool inMode(unsigned mode,double after=-1) const
  { return heartbeat() && m_host.vehicle().mode==mode && m_host.vehicle().heartbeat_at>after; }
  bool safeMode() const
  { return heartbeat() && mission::handbackModeAllowed(m_boat,m_host.vehicle().mode); }
  bool settled(double after) const
  {
    return safeMode() && m_host.vehicle().heartbeat_at>after &&
      m_policy.settled(m_host.vehicle(),m_host.now(),after);
  }
  bool cursorFor(const StoredMissionProof& proof,double after) const
  {
    return proof.items.size()>=3 && m_cursor.at>after && mission::fresh(m_cursor.at,m_host.now(),mission::cursor_age) &&
      m_cursor.packet.total==proof.items.size()-1 && m_cursor.packet.seq<proof.items.size();
  }
  bool startReady(const StoredMissionProof& proof) const
  {
    return proof.valid && proof.session_epoch && proof.revision && !m_host.controlBusy() && !m_host.navigationGranted() &&
      mission::planValid(proof.items,m_host.vehicle()) && m_policy.ready(m_host.vehicle(),m_host.now()) &&
      settled(proof.verified_at) && cursorFor(proof,proof.verified_at) && m_cursor.packet.mission_mode==0 &&
      (m_cursor.packet.mission_state==MISSION_STATE_NOT_STARTED || m_cursor.packet.mission_state==MISSION_STATE_COMPLETE);
  }
  void admit(const Request& request)
  {
    if(m_host.now()>=request.expires) {report(request,"REJECTED","original_deadline");return;}
    if(request.action=="start") {
      const auto proof=m_host.storedMissionProof();
      if(m_execution || !startReady(proof) || request.revision!=proof.revision ||
         request.verification!=proof.verification_id || request.content!=proof.content_id ||
         !m_host.beginMissionExecution(request.id,proof.revision,proof.content_id,request.expires)) {
        report(request,"REJECTED","idle_armed_stopped_verified_plan_required");return;
      }
      m_execution=Execution{request,proof};
      report(request,"ADMITTED","verified_plan_bound_to_retained_lease");
      issue(Stage::Select,request);
      return;
    }
    if(!m_execution || request.target!=m_execution->request.id) {
      report(request,"REJECTED","target_not_retained");return;
    }
    if(request.action=="abort" || request.action=="release") {
      if((m_phase && m_phase->stage==Stage::Stop) || (!safeMode() && !inMode(mission::auto_mode)) ||
         (request.action=="release" && !safeMode() && m_execution->state!=State::Completed) ||
         !m_host.beginMissionExecutionAbort(request.id,request.target,request.expires)) {
        report(request,"REJECTED","explicit_stop_or_supported_handback_required");return;
      }
      if(m_phase) {
        report(m_phase->request,"CANCELLED","superseded_by_stop_only");
        retire(false);
      }
      m_execution->state=State::Stopping;
      issue(Stage::Stop,request);
      return;
    }
    if(m_phase || !m_host.missionExecutionGranted(request.target) ||
       !cursorFor(m_execution->proof,m_execution->handoff)) {
      report(request,"REJECTED","live_mission_and_fresh_cursor_required");return;
    }
    if(request.action=="pause" && m_execution->state==State::Running && inMode(mission::auto_mode)) {
      m_execution->state=State::Pausing;
      issue(Stage::Pause,request);
    } else if(request.action=="resume" && m_execution->state==State::Paused && settled(m_execution->handoff) &&
              mission::inactiveCursor(m_cursor.packet) && m_cursor.packet.seq==m_execution->paused_seq) {
      m_execution->state=State::Resuming;
      issue(Stage::Resume,request);
    } else report(request,"REJECTED","requested_transition_not_ready");
  }
  void issue(Stage stage,const Request& request)
  {
    Phase phase;
    phase.request=request;phase.stage=stage;
    auto& command=phase.command;
    command.id="armx."+std::to_string(++m_wire);
    command.execution_id=m_execution->request.id;
    command.operation_expires=stage==Stage::Stop?request.expires:m_execution->request.expires;
    command.expires=std::min({command.operation_expires,request.expires,m_host.now()+8});
    if(stage==Stage::Select) {
      command.command=MAV_CMD_DO_SET_MISSION_CURRENT;
      command.params[0]=1;
    } else if(stage==Stage::Start) command.command=MAV_CMD_MISSION_START;
    else {
      command.command=MAV_CMD_DO_SET_MODE;
      command.params[0]=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
      command.params[1]=stage==Stage::Resume?mission::auto_mode:mission::handbackMode(m_boat);
      if(stage==Stage::Stop) command.abort_id=request.id;
    }
    m_phase=phase;
    if(!m_host.sendCommand(command) && m_phase && m_phase->command.id==command.id)
      fail("FAILED","host_queue_admission_denied");
    else if(m_phase && m_phase->command.id==command.id)
      report(request,"SUBMITTED","bounded_queue");
  }
  void observeAck(const mavlink_message_t& message,double at)
  {
    if(!m_phase || m_phase->sent_at<0 || at<=m_phase->sent_at ||
       m_host.now()>=m_phase->command.expires || m_phase->ack_observed) return;
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&message,&ack);
    if(ack.command!=m_phase->command.command || ack.target_system!=245 || ack.target_component!=191) return;
    // The wire command number does not identify the phase or its parameters.
    m_phase->ack_observed=true;
    report(m_phase->request,"ACK_OBSERVED","uncorrelated_ack_result_"+std::to_string(ack.result));
  }
  void observeProgress()
  {
    if(!m_execution || !cursorFor(m_execution->proof,m_execution->proof.verified_at)) return;
    if(m_execution->state==State::Running && m_cursor.at>m_execution->running_after &&
       mission::activeCursor(m_cursor.packet)) {
      if(!mission::selectedCursor(m_cursor.packet,m_execution->proof.items.size()) ||
         m_cursor.packet.seq<m_execution->active_seq) {
        uncertain("active_cursor_regressed");return;
      }
      m_execution->active_seq=m_cursor.packet.seq;
    }
    std::ostringstream out;
    out<<std::setprecision(16)<<"schema=1,id="<<m_execution->request.id<<",seq="<<m_cursor.packet.seq
       <<",total="<<m_cursor.packet.total<<",mission_state="<<unsigned(m_cursor.packet.mission_state)
       <<",effective_at="<<m_cursor.at<<",wire_operation_id=false";
    m_host.publish(progress_key,out.str());
  }
  void advance()
  {
    const auto phase=*m_phase;
    auto& execution=*m_execution;
    const auto at=phase.sent_at;
    const bool current=cursorFor(execution.proof,at);
    if(phase.stage==Stage::Select && current && m_cursor.packet.seq==1 && mission::inactiveCursor(m_cursor.packet) && settled(at)) {
      retire(true);execution.state=State::Starting;issue(Stage::Start,execution.request);
    } else if((phase.stage==Stage::Start || phase.stage==Stage::Resume) && current &&
              inMode(mission::auto_mode,at) && mission::activeCursor(m_cursor.packet) &&
              m_cursor.packet.seq==(phase.stage==Stage::Start?1:execution.paused_seq)) {
      retire(true);execution.state=State::Running;
      execution.active_seq=m_cursor.packet.seq;execution.running_after=at;execution.final_reached=-1;
      if(phase.stage==Stage::Resume) report(phase.request,"COMPLETED","same_cursor_resumed_in_auto");
      report(execution.request,"ACTIVE","observed_auto_and_active_cursor");
    } else if(phase.stage==Stage::Pause && current && mission::inactiveCursor(m_cursor.packet) &&
              mission::selectedCursor(m_cursor.packet,execution.proof.items.size()) &&
              inMode(mission::handbackMode(m_boat),at) && settled(at)) {
      retire(true);execution.state=State::Paused;execution.paused_seq=m_cursor.packet.seq;
      report(phase.request,"COMPLETED","settled_and_cursor_retained");
      report(execution.request,"PAUSED","explicit_resume_required");
    } else if(phase.stage==Stage::Stop && inMode(mission::handbackMode(m_boat),at) && settled(at)) {
      retire(true);
      if(!m_host.finishMissionExecution(execution.request.id,"RELEASED")) {
        report(phase.request,"FAILED","host_handback_unconfirmed");
        uncertain("host_handback_unconfirmed");
        return;
      }
      const auto start=execution.request;
      m_execution.reset();
      report(start,"RELEASED","explicit_handback");
      report(phase.request,"COMPLETED","settled_handback_no_helm_grant");
    }
  }
  void complete()
  {
    auto& execution=*m_execution;
    const auto& final=execution.proof.items.back();
    const auto& v=m_host.vehicle();
    if(execution.final_reached<=execution.running_after || !inMode(mission::auto_mode,execution.running_after) ||
       !cursorFor(execution.proof,execution.running_after) || m_cursor.packet.seq!=final.seq ||
       m_cursor.packet.mission_state!=MISSION_STATE_COMPLETE ||
       mission::separation(v.latitude,v.longitude,final.x*1e-7,final.y*1e-7)>3 ||
       !m_policy.settled(v,m_host.now(),execution.final_reached)) return;
    execution.state=State::Completed;
    m_host.finishMissionExecution(execution.request.id,"COMPLETED");
    report(execution.request,"COMPLETED","final_cursor_arrival_and_settled_lease_retained");
  }
  void retire(bool resolved)
  {
    if(!m_phase) return;
    const auto wire=m_phase->command.id;
    m_phase.reset();
    if(resolved) m_host.commandFinished(wire);
    else m_host.cancelCommand(wire);
  }
  void fail(const char* status,const char* why)
  {
    if(!m_phase || !m_execution) return;
    const auto phase=*m_phase;
    retire(false);
    if(phase.stage==Stage::Select && m_execution->handoff<0 &&
       m_host.finishMissionExecution(m_execution->request.id,"REJECTED")) {
      m_execution.reset();report(phase.request,"REJECTED",why);return;
    }
    report(phase.request,status,why);
    uncertain(why);
  }
  void uncertain(const char* why)
  {
    if(!m_execution || m_execution->state==State::Uncertain) return;
    if(m_phase) report(m_phase->request,"CANCELLED",why);
    retire(false);
    m_execution->state=State::Uncertain;
    m_host.finishMissionExecution(m_execution->request.id,"LOST");
    report(m_execution->request,"LOST",why);
  }
  void check()
  {
    const auto now=m_host.now();
    if(!std::isfinite(now) || now<m_last_now) {
      m_clock_fault=true;
      for(const auto& request:m_queue) report(request,"CANCELLED","owner_clock_fault");
      m_queue.clear();uncertain("owner_clock_fault");return;
    }
    m_last_now=now;
    if(!m_execution || m_execution->state==State::Uncertain || m_execution->state==State::Completed) return;
    if(m_phase && now>=m_phase->command.expires) { fail("TIMED_OUT","original_phase_deadline");return; }
    if(m_execution->state==State::Stopping) {
      if(!safeMode() && !inMode(mission::auto_mode)) uncertain("stop_mode_or_link_lost");
      return;
    }
    if(now>=m_execution->request.expires || !m_host.missionExecutionGranted(m_execution->request.id) ||
       !m_policy.ready(m_host.vehicle(),now) || !cursorFor(m_execution->proof,m_execution->proof.verified_at))
      uncertain("plan_authority_or_observations_lost");
  }
  void report(const Request& request,const char* state,std::string_view why)
  {
    std::ostringstream out;
    out<<std::fixed<<std::setprecision(9)<<"schema=1,id="<<request.id<<",action="<<request.action
       <<",target="<<(request.target.empty()?"none":request.target)<<",status="<<state<<",reason="<<why
       <<",at="<<m_host.now()<<",expires="<<request.expires<<",lease_retained="<<(m_execution?"true":"false")
       <<",wire_operation_id=false";
    m_host.publish(result_key,out.str());
  }

  Services& m_host;
  const ArduRoverMissionPolicy& m_policy;
  const bool m_boat;
  double m_boundary,m_last_now;
  bool m_enabled=false,m_clock_fault=false;
  uint64_t m_wire=0;
  std::deque<Request> m_queue;
  std::set<std::string> m_seen;
  std::optional<Execution> m_execution;
  std::optional<Phase> m_phase;
  Cursor m_cursor;
};

} // namespace

std::unique_ptr<Plugin> makeArduRoverMission(Services& host,const VehicleControlBackend& backend)
{ return std::make_unique<ArduRoverMission>(host,backend); }

} // namespace mavmoos::runtime
