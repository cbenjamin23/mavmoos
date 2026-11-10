#include "ControlReplayService.h"
#include "Px4RoverMission.h"
#include "Px4RoverBackend.h"
#include "Px4Offboard.h"
#include "Px4RoverPolicy.h"
#include <algorithm>
#include <cmath>
#include <deque>
#include <iomanip>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace mavmoos::runtime {
namespace {
constexpr unsigned in_mission=1, suspended=2;
constexpr double progress_age=2.5, command_window=5, final_hold_window=8;
constexpr char request_variable[]="MAVMOOS_MISSION_CONTROL";
constexpr char result_variable[]="MAVMOOS_MISSION_EXECUTION_RESULT";
constexpr char status_variable[]="MAVMOOS_MISSION_EXECUTION_STATUS";
constexpr char progress_variable[]="MAVMOOS_MISSION_EXECUTION_PROGRESS";

enum class Action { Start, Pause, Resume, Abort, Release };
enum class State { Starting, Running, Pausing, Paused, Resuming, Settling, Stopping, Completed, Uncertain };

const char* actionName(Action action)
{
  switch(action) {
    case Action::Start: return "start";
    case Action::Pause: return "pause";
    case Action::Resume: return "resume";
    case Action::Abort: return "abort";
    case Action::Release: return "release";
  }
  return "unknown";
}
const char* stateName(State state)
{
  switch(state) {
    case State::Starting: return "STARTING";
    case State::Running: return "RUNNING";
    case State::Pausing: return "PAUSING";
    case State::Paused: return "PAUSED";
    case State::Resuming: return "RESUMING";
    case State::Settling: return "SETTLING";
    case State::Stopping: return "STOPPING";
    case State::Completed: return "COMPLETED";
    case State::Uncertain: return "UNCERTAIN";
  }
  return "UNKNOWN";
}
bool stopOnly(Action action) { return action==Action::Abort || action==Action::Release; }
bool safeId(std::string_view value)
{
  return !value.empty() && value.size()<=40 &&
    std::all_of(value.begin(),value.end(),[](unsigned char c) {
      return (c>='a'&&c<='z') || (c>='A'&&c<='Z') || (c>='0'&&c<='9') || c=='_' || c=='-' || c=='.';
    });
}
bool safeContent(std::string_view value)
{
  return !value.empty() && value.size()<=128 &&
    std::all_of(value.begin(),value.end(),[](unsigned char c) {
      return c>32 && c<127 && c!=',' && c!='=';
    });
}
double separation(double a,double b,double latitude,double longitude)
{
  const auto north=(latitude-a)*px4::metres_per_degree;
  const auto east=std::remainder(longitude-b,360.)*px4::metres_per_degree*std::cos(a*px4::radians);
  return std::hypot(north,east);
}
bool heartbeat(const Vehicle& v,double now)
{
  return v.type==MAV_TYPE_GROUND_ROVER && v.autopilot==MAV_AUTOPILOT_PX4 &&
    v.heartbeat_valid && px4::fresh(v.heartbeat_at,now,px4::rover_heartbeat_age);
}
bool validProof(const StoredMissionProof& proof,const Vehicle& v,double now)
{
  if(!proof.valid || !proof.session_epoch || !proof.revision || !std::isfinite(proof.verified_at) ||
     proof.verified_at<=0 || proof.verified_at>now || !px4_rover::validMission(proof.items) ||
     !v.home_valid || std::abs(v.home_latitude)>=89 || std::abs(v.home_longitude)>180) return false;
  return std::all_of(proof.items.begin(),proof.items.end(),[&](const auto& item) {
    return separation(v.home_latitude,v.home_longitude,item.x*1e-7,item.y*1e-7)<=60;
  });
}

struct Request {
  std::string id,target,verification,content;
  Action action=Action::Start;
  uint64_t revision=0;
  double expires=0;
};
struct Cursor {
  mavlink_mission_current_t value{};
  MessageTime time;
};
struct Phase {
  Request request;
  CommandRequest wire;
  double deadline=0, observed_after=0;
  bool sent=false, ack_observed=false;
};
struct Execution {
  Request start;
  StoredMissionProof proof;
  uint32_t mission_id=0;
  State state=State::Starting;
  std::string terminal;
  double last_handoff=-1, running_after=-1, final_reached=-1, hold_seen=-1;
  bool first_reached=false;
  unsigned last_active_seq=0, pause_seq=0;
};

class Coordinator final : public Plugin {
public:
  Coordinator(Services& host,const Px4RoverBackend& backend)
    :m_host(host),m_backend(backend),m_binding(dynamic_cast<const Px4RoverMissionBinding*>(&host)),
     m_boundary(host.now()),m_last_now(host.now()) {}

  std::string name() const override { return "px4_rover_mission_execution"; }
  PluginDescriptor descriptor() const override
  {
    PluginDescriptor d;
    d.profile="px4_rover_ackermann";
    if(!m_enabled) return d;
    d.request_owners={request_variable};
    for(auto action:{Action::Start,Action::Pause,Action::Resume,Action::Abort,Action::Release})
      d.capabilities.push_back({std::string("control.mission.")+actionName(action),
        request_variable,std::string("schema=1,action=")+actionName(action),"request"});
    d.capabilities.push_back({"control.mission.progress",progress_variable,"","observation"});
    return d;
  }
  std::vector<std::string> subscriptions() const override
  { return m_enabled?std::vector<std::string>{request_variable}:std::vector<std::string>{}; }
  void configure(const std::string& key,const std::string& value) override
  {
    if(key!="control_mission_execution") return;
    if(value!="true" && value!="false")
      throw std::invalid_argument("control_mission_execution must be true/false");
    m_enabled=value=="true";
  }
  void onMail(const Mail& mail) override
  {
    if(!m_enabled || mail.key!=request_variable) return;
    Request request;
    std::string reason;
    if(!parse(mail,request,reason)) { report(request,"REJECTED",reason); return; }
    if(m_seen.count(request.id)) { report(request,"DUPLICATE","id_already_seen"); return; }
    if(m_seen.size()>=128 || m_queue.size()>=4) { report(request,"REJECTED","request_bound"); return; }
    const auto replay=reserveControlRequest(m_host,request.id);
    if(replay!=ReplayResult::Reserved) {report(request,replayRejection(replay),replayReason(replay));return;}
    m_seen.insert(request.id);
    m_queue.push_back(std::move(request));
  }
  void onMessage(const mavlink_message_t& message,double at) override
  { onMessageWithMetadata(message,{at,at,"READ_ACQUISITION"}); }
  void onMessageWithMetadata(const mavlink_message_t& message,const MessageTime& time) override
  {
    if(!m_enabled || message.magic!=MAVLINK_STX || message.sysid!=1 || message.compid!=1 ||
       !heartbeat(m_host.vehicle(),m_host.now()) || !validMetadata(time)) return;
    switch(message.msgid) {
      case MAVLINK_MSG_ID_COMMAND_ACK: observeAck(message,time.read_at); break;
      case MAVLINK_MSG_ID_MISSION_CURRENT: observeCursor(message,time); break;
      case MAVLINK_MSG_ID_MISSION_ITEM_REACHED: observeReached(message,time.effective_at); break;
      default: break;
    }
  }
  void onCommandSent(const CommandRequest& request,double at,bool accepted) override
  {
    if(!m_execution || request.execution_id!=m_execution->start.id ||
       request.id.rfind("px4-rm.",0)!=0) return;
    if(accepted) m_execution->last_handoff=std::max(m_execution->last_handoff,
      std::isfinite(at) && at<=m_host.now()?at:m_host.now());
    if(!m_phase || m_phase->wire.id!=request.id) {
      if(accepted) {
        uncertain("late_retired_handoff_remote_effect_uncertain");
      }
      return;
    }
    if(!accepted) { phaseFailed("FAILED","local_send_rejected"); return; }
    if(!std::isfinite(at) || at<m_phase->observed_after || at>m_host.now() || at>=m_phase->wire.expires ||
       m_host.now()>=m_phase->wire.expires) {
      phaseFailed("TIMED_OUT","send_receipt_outside_original_window");
      return;
    }
    m_phase->sent=true;
    m_phase->observed_after=at;
    report(m_phase->request,"SENT","local_handoff_awaiting_vehicle_evidence");
  }
  void onInputBoundary() override { if(m_enabled) check(); }
  void onReset(const std::string&) override
  {
    for(const auto& request:m_queue) report(request,"CANCELLED","session_reset");
    m_queue.clear();
    uncertain("session_reset_remote_effect_uncertain");
    m_cursor.reset();
    m_boundary=m_host.now();
    m_last_now=m_boundary;
  }
  void iterate() override
  {
    if(!m_enabled) return;
    check();
    if(!m_clock_fault) {
      if(!m_queue.empty()) {
        const auto request=std::move(m_queue.front());
        m_queue.pop_front();
        admit(request);
      }
      if(m_phase) settlePhase();
      if(m_execution && (m_execution->state==State::Running || m_execution->state==State::Settling))
        settleCompletion();
    }
    publishStatus();
  }

private:
  bool validMetadata(const MessageTime& time) const
  {
    return std::isfinite(time.effective_at) && std::isfinite(time.read_at) &&
      time.effective_at>m_boundary && time.effective_at<=time.read_at &&
      time.read_at<=m_host.now() && m_host.now()-time.effective_at<=progress_age &&
      (time.age_basis=="READ_ACQUISITION" || time.age_basis=="QUEUE_EMPTY_FLOOR");
  }
  bool parse(const Mail& mail,Request& request,std::string& reason) const
  {
    reason="invalid_request";
    if(mail.numeric || mail.text.empty() || mail.text.size()>768 || mail.text.back()==',') return false;
    std::map<std::string,std::string> fields;
    std::istringstream input(mail.text);
    for(std::string part;std::getline(input,part,',');) {
      const auto eq=part.find('=');
      if(eq==std::string::npos || !fields.emplace(part.substr(0,eq),part.substr(eq+1)).second) return false;
    }
    const auto& supplied_id=fields["id"];
    if(!safeId(supplied_id) || fields["schema"]!="1") return false;
    request.id=supplied_id;
    const auto& action=fields["action"];
    if(action=="start") request.action=Action::Start;
    else if(action=="pause") request.action=Action::Pause;
    else if(action=="resume") request.action=Action::Resume;
    else if(action=="abort") request.action=Action::Abort;
    else if(action=="release") request.action=Action::Release;
    else { reason="unsupported_action"; return false; }
    const auto now=m_host.now();
    const auto age=mail.received_at-mail.published_at;
    if(m_clock_fault || !std::isfinite(now) || !std::isfinite(mail.published_at) ||
       !std::isfinite(mail.received_at) || mail.published_at<=m_boundary || age<-.1 || age>2 ||
       mail.received_at>now+.1 || now-mail.received_at>2) {
      reason="stale_future_or_pre_session_request";
      return false;
    }
    const bool start=request.action==Action::Start;
    double timeout=start?90:10;
    if(const auto it=fields.find("timeout");it!=fields.end()) {
      try {
        std::size_t used=0;
        timeout=std::stod(it->second,&used);
        if(used!=it->second.size()) return false;
      } catch(const std::exception&) { return false; }
    }
    if(!std::isfinite(timeout) || timeout<1 || timeout>(start?120:15)) return false;
    request.expires=mail.published_at+timeout;
    if(request.expires<=now) { reason="original_deadline"; return false; }
    std::set<std::string> allowed={"schema","id","action","timeout"};
    if(start) {
      allowed.insert({"verification_id","content_id","proof_revision"});
      request.verification=fields["verification_id"];
      request.content=fields["content_id"];
      const auto& revision=fields["proof_revision"];
      if(!safeId(request.verification) || !safeContent(request.content) || revision.empty() ||
         !std::all_of(revision.begin(),revision.end(),[](char c){return c>='0'&&c<='9';})) return false;
      try { request.revision=std::stoull(revision); } catch(const std::exception&) { return false; }
      if(request.revision==0) return false;
    } else {
      allowed.insert("target");
      if(!safeId(fields["target"])) return false;
      request.target=fields["target"];
    }
    return std::all_of(fields.begin(),fields.end(),[&](const auto& field){return allowed.count(field.first);});
  }
  bool freshCursor(double after) const
  {
    return m_cursor && m_cursor->time.effective_at>after &&
      px4::fresh(m_cursor->time.effective_at,m_host.now(),progress_age);
  }
  bool cursorMatches(const StoredMissionProof& proof) const
  {
    return m_cursor && m_cursor->value.total==proof.items.size() &&
      m_cursor->value.seq<proof.items.size() && m_cursor->value.mission_id!=0 &&
      m_binding && m_cursor->value.mission_id==m_binding->verifiedPx4MissionId();
  }
  bool startReady(const StoredMissionProof& proof) const
  {
    const auto now=m_host.now();
    const auto& v=m_host.vehicle();
    return validProof(proof,v,now) && !m_host.controlBusy() && !m_host.navigationGranted() &&
      m_backend.missionReady(v,now) && heldAfter(proof.verified_at) &&
      freshCursor(proof.verified_at) && cursorMatches(proof) &&
      m_cursor->value.mission_mode==suspended && m_cursor->value.mission_state!=MISSION_STATE_NO_MISSION &&
      m_cursor->value.mission_state!=MISSION_STATE_UNKNOWN;
  }
  bool sameProof() const
  {
    const auto p=m_host.storedMissionProof();
    return p.valid && p.session_epoch==m_execution->proof.session_epoch &&
      p.revision==m_execution->proof.revision && p.content_id==m_execution->proof.content_id &&
      m_binding && m_binding->verifiedPx4MissionId()==m_execution->mission_id;
  }
  bool observedMode(unsigned mode,double after) const
  {
    const auto& v=m_host.vehicle();
    const auto now=m_host.now();
    return heartbeat(v,now) && v.heartbeat_at>after && v.mode==mode &&
      (mode==px4::hold?m_backend.holdObserved(now,after):m_backend.missionObserved(now,after));
  }
  bool heldAfter(double after) const
  { return observedMode(px4::hold,after) && m_backend.stopped(m_host.now(),after); }

  void admit(const Request& request)
  {
    const auto now=m_host.now();
    if(now>=request.expires) { report(request,"REJECTED","original_deadline"); return; }
    if(request.action==Action::Start) {
      const auto proof=m_host.storedMissionProof();
      if(m_execution || m_phase || !startReady(proof) || proof.verification_id!=request.verification ||
         proof.content_id!=request.content || proof.revision!=request.revision) {
        report(request,"REJECTED","idle_verified_plan_and_stopped_armed_hold_required");
        return;
      }
      if(!m_host.beginMissionExecution(request.id,proof.revision,proof.content_id,request.expires)) {
        report(request,"REJECTED","host_execution_lease_denied");
        return;
      }
      m_execution=Execution{request,proof,m_cursor->value.mission_id,State::Starting,{}};
      report(request,"ADMITTED","verified_plan_and_cursor_bound");
      issue(request);
      return;
    }
    if(!m_execution || request.target!=m_execution->start.id) {
      report(request,"REJECTED","target_not_retained");
      return;
    }
    if(stopOnly(request.action)) { admitStop(request); return; }
    if(m_phase || !m_host.missionExecutionGranted(request.target) || !sameProof() ||
       !m_backend.missionReady(m_host.vehicle(),now) || !freshCursor(m_execution->last_handoff)) {
      report(request,"REJECTED","live_mission_lease_and_fresh_progress_required");
      return;
    }
    if(request.action==Action::Pause) {
      if(m_execution->state!=State::Running || !observedMode(px4::mission,m_execution->last_handoff)) {
        report(request,"REJECTED","observed_running_mission_required");
        return;
      }
      m_execution->state=State::Pausing;
    } else {
      if(m_execution->state!=State::Paused || !heldAfter(m_execution->last_handoff) ||
         m_cursor->value.mission_mode!=suspended || m_cursor->value.seq!=m_execution->pause_seq ||
         m_cursor->value.mission_state==MISSION_STATE_COMPLETE) {
        report(request,"REJECTED","confirmed_paused_cursor_and_stopped_hold_required");
        return;
      }
      m_execution->state=State::Resuming;
    }
    issue(request);
  }
  void admitStop(const Request& request)
  {
    const auto now=m_host.now();
    const bool hold=observedMode(px4::hold,-1);
    if((m_phase && stopOnly(m_phase->request.action)) ||
       (!hold && !observedMode(px4::mission,-1)) ||
       !m_host.beginMissionExecutionAbort(request.id,request.target,request.expires)) {
      report(request,"REJECTED","stop_only_lease_or_vehicle_mode_denied");
      return;
    }
    if(m_phase) {
      report(m_phase->request,"CANCELLED","explicit_stop_superseded_pending_phase");
      retire(false);
    }
    m_execution->state=State::Stopping;
    report(request,"ADMITTED","bounded_stop_only_handback");
    if(hold) {
      Phase phase;
      phase.request=request;
      phase.deadline=request.expires;
      phase.observed_after=std::max(now,m_execution->last_handoff);
      m_phase=std::move(phase);
    } else {
      issue(request);
    }
  }
  void issue(const Request& request)
  {
    Phase phase;
    phase.request=request;
    phase.observed_after=m_host.now();
    auto& wire=phase.wire;
    wire.id="px4-rm."+std::to_string(++m_wire_id);
    wire.execution_id=m_execution->start.id;
    const bool stopping=stopOnly(request.action);
    wire.operation_expires=stopping?request.expires:m_execution->start.expires;
    phase.deadline=std::min(request.expires,wire.operation_expires);
    wire.expires=std::min(m_host.now()+command_window,phase.deadline);
    if(stopping) wire.abort_id=request.id;
    if(request.action==Action::Start) {
      wire.command=MAV_CMD_MISSION_START;
      wire.params[1]=static_cast<float>(m_execution->proof.items.size()-1);
    } else {
      wire.command=MAV_CMD_DO_SET_MODE;
      wire.params[0]=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
      wire.params[1]=4;
      wire.params[2]=request.action==Action::Resume?4.f:3.f;
    }
    const auto command=wire;
    m_phase=std::move(phase);
    if(!m_host.sendCommand(command) && m_phase && m_phase->wire.id==command.id)
      phaseFailed("FAILED","host_queue_admission_denied");
    else if(m_phase && m_phase->wire.id==command.id)
      report(request,"SUBMITTED","host_queue_accepted_awaiting_handoff");
  }
  void observeAck(const mavlink_message_t& message,double read_at)
  {
    if(!m_phase || !m_phase->sent || m_phase->ack_observed || read_at<=m_phase->observed_after ||
       m_host.now()>=m_phase->wire.expires) return;
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&message,&ack);
    if(ack.command!=m_phase->wire.command || ack.target_system!=245 || ack.target_component!=191) return;
    // Pause and resume share a command number. An ACK cannot identify its phase.
    m_phase->ack_observed=true;
    report(m_phase->request,"ACK_OBSERVED","uncorrelated_ack_result_"+std::to_string(ack.result));
  }
  void observeCursor(const mavlink_message_t& message,const MessageTime& time)
  {
    if(m_cursor && time.effective_at<=m_cursor->time.effective_at) return;
    Cursor observed;
    mavlink_msg_mission_current_decode(&message,&observed.value);
    observed.time=time;
    m_cursor=observed;
    if(!m_execution || m_execution->state==State::Stopping || m_execution->state==State::Uncertain) return;
    if(!cursorMatches(m_execution->proof) || observed.value.mission_id!=m_execution->mission_id) {
      uncertain("observed_mission_identity_or_count_changed");
      return;
    }
    if(m_execution->state==State::Running && time.effective_at>m_execution->running_after &&
       observed.value.mission_mode==in_mission && observed.value.mission_state==MISSION_STATE_ACTIVE) {
      if(observed.value.seq<m_execution->last_active_seq) {
        uncertain("observed_mission_cursor_regressed");
        return;
      }
      m_execution->last_active_seq=observed.value.seq;
    }
    std::ostringstream out;
    out<<std::setprecision(17)<<"schema=1,id="<<m_execution->start.id<<",seq="<<observed.value.seq
      <<",total="<<observed.value.total<<",mission_state="<<unsigned(observed.value.mission_state)
      <<",mission_id="<<observed.value.mission_id<<",effective_at="<<time.effective_at<<",read_at="<<time.read_at;
    m_host.publish(progress_variable,out.str());
  }
  void observeReached(const mavlink_message_t& message,double at)
  {
    if(!m_execution) return;
    mavlink_mission_item_reached_t reached{};
    mavlink_msg_mission_item_reached_decode(&message,&reached);
    auto& execution=*m_execution;
    if(execution.state==State::Starting && m_phase && m_phase->sent &&
       at>m_phase->observed_after && reached.seq==0) execution.first_reached=true;
    // PX4 re-emits the final item event. Retain its first observation so
    // retransmissions cannot move the floor for the stopped dwell.
    if((execution.state==State::Running || execution.state==State::Settling) &&
       at>execution.running_after && reached.seq==execution.proof.items.back().seq &&
       execution.final_reached<0) execution.final_reached=at;
  }
  void settlePhase()
  {
    const auto& p=*m_phase;
    if(!p.wire.id.empty() && !p.sent) return;
    const auto action=p.request.action;
    const auto after=p.observed_after;
    if(action==Action::Start || action==Action::Resume) {
      if(!observedMode(px4::mission,after) || !freshCursor(after) ||
         m_cursor->value.mission_state!=MISSION_STATE_ACTIVE || m_cursor->value.mission_mode!=in_mission ||
         (action==Action::Start?m_cursor->value.seq!=0 && !m_execution->first_reached:
          m_cursor->value.seq<m_execution->pause_seq)) return;
      const auto owner=p.request;
      retire(true);
      m_execution->state=State::Running;
      m_execution->running_after=after;
      m_execution->last_active_seq=m_cursor->value.seq;
      m_execution->final_reached=-1;
      m_execution->hold_seen=-1;
      if(action==Action::Resume) report(owner,"COMPLETED","observed_resumed_mission_and_cursor");
      report(m_execution->start,"ACTIVE","post_handoff_mission_and_fresh_bound_cursor");
      return;
    }
    if(!heldAfter(after)) return;
    if(action==Action::Pause) {
      if(!freshCursor(after) || m_cursor->value.mission_mode!=suspended ||
         m_cursor->value.seq<m_execution->last_active_seq) return;
      if(m_cursor->value.mission_state==MISSION_STATE_COMPLETE) {
        phaseFailed("FAILED","mission_finished_during_pause");
        return;
      }
      const auto owner=p.request;
      retire(true);
      m_execution->state=State::Paused;
      m_execution->pause_seq=m_cursor->value.seq;
      report(owner,"COMPLETED","observed_hold_and_stopped_dwell");
      report(m_execution->start,"PAUSED","lease_retained_explicit_resume_required");
      return;
    }
    const auto owner=p.request;
    retire(true);
    if(!m_host.finishMissionExecution(m_execution->start.id,"RELEASED")) {
      report(owner,"FAILED","host_stopped_handback_not_confirmed");
      uncertain("host_stopped_handback_not_confirmed");
      return;
    }
    const auto start=m_execution->start;
    const auto active=m_execution->terminal.empty();
    m_execution.reset();
    if(active) report(start,"CANCELLED","explicit_observed_stopped_handback");
    report(owner,"COMPLETED","fresh_hold_stopped_and_lease_released");
  }
  void settleCompletion()
  {
    auto& e=*m_execution;
    const auto& v=m_host.vehicle();
    const auto& goal=e.proof.items.back();
    if(e.final_reached<=e.running_after ||
       !freshCursor(e.running_after) || m_cursor->value.seq!=goal.seq ||
       m_cursor->value.mission_state!=MISSION_STATE_COMPLETE ||
       !v.position_valid || !px4::fresh(v.position_at,m_host.now(),.5) || v.position_at<=e.running_after ||
       !(separation(v.latitude,v.longitude,goal.x*1e-7,goal.y*1e-7)<=3)) return;
    const auto after=std::max({e.running_after,e.final_reached,e.hold_seen});
    if(!(heldAfter(after) || (observedMode(px4::mission,after) && m_backend.stopped(m_host.now(),after)))) return;
    e.state=State::Completed;
    e.terminal="COMPLETED";
    m_host.finishMissionExecution(e.start.id,e.terminal);
    report(e.start,e.terminal,"final_cursor_goal_and_stopped_dwell_lease_retained");
  }
  void retire(bool observed)
  {
    if(!m_phase) return;
    const auto wire=m_phase->wire.id;
    m_phase.reset();
    if(wire.empty()) return;
    if(observed) m_host.commandFinished(wire);
    else m_host.cancelCommand(wire);
  }
  void phaseFailed(const std::string& status,const std::string& reason)
  {
    const auto owner=m_phase->request;
    retire(false);
    if(owner.action==Action::Start && m_execution->last_handoff<0 &&
       m_host.finishMissionExecution(m_execution->start.id,"REJECTED")) {
      m_execution.reset();
      report(owner,status,reason);
      return;
    }
    if(owner.action!=Action::Start) report(owner,status,reason);
    uncertain(reason,owner.action==Action::Start?status:"LOST");
  }
  void uncertain(const std::string& reason,const std::string& status="LOST")
  {
    if(!m_execution) return;
    if(m_phase) {
      if(m_phase->request.action!=Action::Start) report(m_phase->request,status,reason);
      retire(false);
    }
    if(m_execution->state==State::Uncertain) return;
    m_execution->state=State::Uncertain;
    m_host.finishMissionExecution(m_execution->start.id,status);
    if(m_execution->terminal.empty()) {
      m_execution->terminal=status;
      report(m_execution->start,status,reason);
    }
  }
  void check()
  {
    const auto now=m_host.now();
    if(!std::isfinite(now) || now<m_last_now) {
      m_clock_fault=true;
      for(const auto& request:m_queue) report(request,"CANCELLED","owner_clock_fault");
      m_queue.clear();
      uncertain("owner_clock_fault_remote_effect_uncertain");
      return;
    }
    m_last_now=now;
    if(!m_execution) return;
    if(m_phase && (now>=m_phase->deadline ||
       (!m_phase->wire.id.empty() && !m_phase->sent && now>=m_phase->wire.expires))) {
      phaseFailed("TIMED_OUT","original_phase_deadline_remote_effect_unconfirmed");
      return;
    }
    auto& e=*m_execution;
    if(e.state==State::Completed || e.state==State::Uncertain) return;
    const auto& v=m_host.vehicle();
    if(!heartbeat(v,now) || (v.mode!=px4::mission && v.mode!=px4::hold)) {
      uncertain("vehicle_observation_lost_or_external_mode");
      return;
    }
    if(e.state==State::Stopping) return;
    if(now>=e.start.expires) {
      uncertain("original_execution_deadline_remote_execution_may_continue","TIMED_OUT");
      return;
    }
    if(!sameProof() || !m_backend.missionReady(v,now)) {
      uncertain("bound_mission_or_vehicle_prerequisites_lost");
      return;
    }
    // PX4 automatically enters Hold on the final waypoint. The subsequent
    // COMPLETE cursor and stopped dwell can arrive after the Hold heartbeat.
    if(e.state==State::Running && v.mode==px4::hold) {
      e.state=State::Settling;
      e.hold_seen=now;
    }
    if(e.state==State::Settling) {
      if(v.mode==px4::hold && now-e.hold_seen<final_hold_window) return;
      uncertain("hold_without_independently_confirmed_completion");
      return;
    }
    if(!m_host.missionExecutionGranted(e.start.id) ||
       (e.state==State::Running && (v.mode!=px4::mission || !freshCursor(e.running_after))) ||
       (e.state==State::Paused && (v.mode!=px4::hold || !freshCursor(e.last_handoff)))) {
      uncertain("mission_lease_progress_or_expected_mode_lost");
    }
  }
  void report(const Request& request,const std::string& status,const std::string& reason)
  {
    std::ostringstream out;
    out<<std::fixed<<std::setprecision(9)<<"schema=1,id="<<request.id<<",action="<<actionName(request.action)
      <<",target="<<(request.target.empty()?"none":request.target)<<",status="<<status<<",reason="<<reason
      <<",at="<<m_host.now()<<",expires="<<request.expires
      <<",lease_retained="<<(m_execution?"true":"false")
      <<",remote_execution_may_continue="<<(m_execution && m_execution->last_handoff>=0 &&
        m_execution->state!=State::Paused && m_execution->state!=State::Completed?"true":"false")
      <<",wire_operation_id=false";
    m_host.publish(result_variable,out.str());
  }
  void publishStatus()
  {
    const auto proof=m_host.storedMissionProof();
    std::ostringstream out;
    out<<"schema=1,enabled="<<(m_enabled?"true":"false")<<",id="<<(m_execution?m_execution->start.id:"none")
      <<",phase="<<(m_execution?stateName(m_execution->state):"IDLE")
      <<",lease_retained="<<(m_execution?"true":"false")<<",resume_supported=true"
      <<",start_ready="<<(!m_clock_fault && !m_execution && startReady(proof)?"true":"false");
    m_host.publish(status_variable,out.str());
  }

  Services& m_host;
  const Px4RoverBackend& m_backend;
  const Px4RoverMissionBinding* m_binding;
  bool m_enabled=false,m_clock_fault=false;
  double m_boundary,m_last_now;
  uint64_t m_wire_id=0;
  std::deque<Request> m_queue;
  std::set<std::string> m_seen;
  std::optional<Cursor> m_cursor;
  std::optional<Phase> m_phase;
  std::optional<Execution> m_execution;
};
}

std::unique_ptr<Plugin> makePx4RoverMission(Services& host,const Px4RoverBackend& backend)
{ return std::make_unique<Coordinator>(host,backend); }
} // namespace mavmoos::runtime
