#include "CopterMissionExecution.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

using namespace mavmoos::runtime;
namespace {
void require(bool x,const std::string& why){if(!x)throw std::runtime_error(why);}
struct Rig:Services {
  double clock=10,lease_expiry=0,last_sent=-1;Vehicle v;StoredMissionProof proof;
  std::unique_ptr<Plugin> component;
  bool enabled=true,deny_begin=false,deny_abort=false,deny_release=false,deny_queue=false,blocked=false,active=false,handed=false;
  std::string lease,abort_id;unsigned nav_acquires=0;
  std::vector<CommandRequest> commands;std::vector<std::pair<std::string,std::string>> publications,dispositions;
  std::vector<std::string> cancelled,finished;
  mavlink_status_t codec{};
  explicit Rig(bool on=true):enabled(on){
    v.type=MAV_TYPE_QUADROTOR;v.autopilot=MAV_AUTOPILOT_ARDUPILOTMEGA;v.mode=4;v.armed=v.in_air=v.home_valid=v.estimator_horizontal_valid=true;
    v.latitude=v.home_latitude=42;v.longitude=v.home_longitude=-71;v.home_msl=10;v.relative_height=5;
    proof.valid=true;proof.session_epoch=1;proof.revision=7;proof.verification_id="upload";proof.content_id="host.1.7";proof.verified_at=9;
    for(unsigned n=0;n<3;++n){MissionItem i;i.seq=n;i.frame=n?MAV_FRAME_GLOBAL_RELATIVE_ALT:MAV_FRAME_GLOBAL;i.x=420000000+(n?1000:0);i.y=-710000000+(n==2?1000:0);i.z=n?5:10;proof.items.push_back(i);}
    refresh();component=makeCopterMissionExecution(*this);if(on)component->configure("control_mission_execution","true");
  }
  double now()const override{return clock;}
  const Vehicle& vehicle()const override{return v;}
  void refresh(){v.heartbeat_valid=v.position_valid=v.landed_valid=v.estimator_valid=true;v.heartbeat_at=v.position_at=v.landed_at=v.estimator_at=clock;}
  void publish(const std::string& k,const std::string& x)override{publications.emplace_back(k,x);}
  void publish(const std::string&,double)override{}
  bool sendCommand(const CommandRequest& r)override{commands.push_back(r);return !deny_queue;}
  bool acquireNavigation(const std::string&)override{++nav_acquires;return false;}
  void revokeNavigation(const std::string&)override{}
  bool navigation(const NavigationTarget&)override{throw std::runtime_error("AUTO coordinator streamed a Guided target");}
  StoredMissionProof storedMissionProof()const override{return proof;}
  bool beginMissionExecution(const std::string& id,uint64_t revision,const std::string& content,double expires)override{
    if(deny_begin||blocked||!proof.valid||revision!=proof.revision||content!=proof.content_id)return false;
    lease=id;lease_expiry=expires;blocked=active=true;handed=false;return true;
  }
  bool missionExecutionGranted(const std::string& id)const override{return active&&lease==id&&clock<lease_expiry;}
  bool beginMissionExecutionAbort(const std::string& id,const std::string& target,double expires)override{
    if(deny_abort||!blocked||lease!=target||clock>=expires)return false;abort_id=id;active=false;return true;
  }
  bool finishMissionExecution(const std::string& id,const std::string& state)override{
    dispositions.emplace_back(id,state);
    if(state=="RELEASED"){if(deny_release||v.mode==3)return false;blocked=active=false;return true;}
    if(state=="REJECTED"){if(handed)return false;blocked=active=false;return true;}
    active=false;blocked=true;return true;
  }
  void cancelCommand(const std::string& id)override{cancelled.push_back(id);}
  void commandFinished(const std::string& id)override{finished.push_back(id);}
  bool result(const std::string& id,const std::string& state)const{for(const auto& p:publications)if(p.first=="MAVMOOS_MISSION_EXECUTION_RESULT"&&p.second.find("id="+id+",")!=std::string::npos&&p.second.find("status="+state+",")!=std::string::npos)return true;return false;}
  bool text(const std::string& needle)const{for(const auto& p:publications)if(p.second.find(needle)!=std::string::npos)return true;return false;}
  void tick(double delta=.01){clock+=delta;refresh();component->onInputBoundary();component->iterate();}
  void request(const std::string& body){clock+=.01;refresh();Mail m;m.key="MAVMOOS_MISSION_CONTROL";m.text="schema=1,"+body;m.published_at=m.received_at=clock;component->onMail(m);component->iterate();}
  void start(const std::string& tail=""){request("id=run,action=start,verification_id=upload,content_id=host.1.7,proof_revision=7"+tail);}
  void sent(bool accepted=true){clock+=.01;refresh();handed|=accepted;last_sent=clock;const auto r=commands.back();component->onCommandSent(r,clock,accepted);}
  void deliver(mavlink_message_t& m,double effective=-1){clock+=.01;refresh();component->onMessageWithMetadata(m,{effective<0?clock:effective,clock,"READ_ACQUISITION"});}
  void ack(unsigned result=MAV_RESULT_ACCEPTED,unsigned sys=1){mavlink_command_ack_t a{};a.command=commands.back().command;a.result=result;a.target_system=245;a.target_component=191;mavlink_message_t m{};mavlink_msg_command_ack_encode_status(sys,1,&codec,&m,&a);deliver(m);}
  void ackTimed(double effective,double read,const std::string& basis="QUEUE_EMPTY_FLOOR",unsigned type=0,unsigned target_system=245,unsigned target_component=191){clock=std::max(clock+.01,read);refresh();mavlink_command_ack_t a{};a.command=type?type:commands.back().command;a.result=MAV_RESULT_ACCEPTED;a.target_system=target_system;a.target_component=target_component;mavlink_message_t m{};mavlink_msg_command_ack_encode_status(1,1,&codec,&m,&a);component->onMessageWithMetadata(m,{effective,read,basis});}
  void current(unsigned seq,unsigned state,double effective=-1){mavlink_mission_current_t c{};c.seq=seq;c.total=proof.items.size()-1;c.mission_state=state;c.mission_mode=v.mode==3?1:0;mavlink_message_t m{};mavlink_msg_mission_current_encode_status(1,1,&codec,&m,&c);deliver(m,effective);}
  void reached(unsigned seq){mavlink_mission_item_reached_t r{};r.seq=seq;mavlink_message_t m{};mavlink_msg_mission_item_reached_encode_status(1,1,&codec,&m,&r);deliver(m);}
  void running(){start();require(commands.size()==1,"selection not queued");sent();ack();current(1,MISSION_STATE_NOT_STARTED);tick();require(commands.back().command==MAV_CMD_MISSION_START,"start not queued after cursor proof");sent();ack();v.mode=3;current(1,MISSION_STATE_ACTIVE);tick();require(result("run","ACTIVE"),"AUTO not active");}
  void completed(){running();current(2,MISSION_STATE_ACTIVE);reached(2);current(2,MISSION_STATE_COMPLETE);v.latitude=proof.items[2].x*1e-7;v.longitude=proof.items[2].y*1e-7;tick();tick(1.01);require(result("run","COMPLETED"),"actual completion not observed");}
};
void disabled(){Rig r(false);r.start();require(r.commands.empty()&&r.component->descriptor().capabilities.empty(),"disabled component exposed control");}
void default_host_denial(){Rig r;r.deny_begin=true;r.start();require(r.result("run","REJECTED")&&r.commands.empty(),"host denial bypassed");}
void unarmed_or_ground_rejected(){for(unsigned k=0;k<2;++k){Rig r;if(k==0)r.v.armed=false;else r.v.in_air=false;r.start();require(r.result("run","REJECTED")&&r.commands.empty(),"implicit ARM/takeoff admitted");}}
void invalid_proof(){for(unsigned k=0;k<5;++k){Rig r;if(k==0)r.proof.valid=false;if(k==1)r.proof.revision++;if(k==2)r.proof.items[1].command=MAV_CMD_NAV_LAND;if(k==3)r.proof.items[2].z=21;if(k==4)r.proof.items[1].params[0]=31;r.start();require(r.commands.empty()&&r.result("run","REJECTED"),"unsupported/unbound proof admitted");}}
void no_reply_before_send(){Rig r;r.start();r.ack();r.current(1,MISSION_STATE_NOT_STARTED);r.tick();require(r.commands.size()==1,"queue admission opened response phase");}
void foreign_ack(){Rig r;r.start();r.sent();r.ack(MAV_RESULT_ACCEPTED,2);r.tick();require(r.commands.size()==1&&!r.result("run","ACK_OBSERVED"),"foreign ACK unlocked start");}
void fixed_selection_and_start(){Rig r;r.running();require(r.commands.size()==2&&r.commands[0].command==224&&r.commands[0].params[0]==1&&r.commands[0].params[1]==0&&r.commands[1].command==300,"unexpected mission wire effects");for(const auto& c:r.commands)require(c.execution_id=="run"&&c.operation_expires==r.lease_expiry&&c.expires<=c.operation_expires,"lost lease/original expiry");require(r.nav_acquires==0,"AUTO acquired helm");}
void ack_not_completion(){Rig r;r.running();require(!r.result("run","COMPLETED")&&r.blocked,"ACK qualified completed mission");}
void actual_completion_retains_lease(){Rig r;r.completed();require(r.blocked&&!r.active&&r.nav_acquires==0,"completion freed AUTO/helm authority");}
void missing_final_reached(){Rig r;r.running();r.current(2,MISSION_STATE_ACTIVE);r.current(2,MISSION_STATE_COMPLETE);r.v.latitude=r.proof.items[2].x*1e-7;r.v.longitude=r.proof.items[2].y*1e-7;r.tick();r.tick(1.01);require(!r.result("run","COMPLETED"),"completion omitted reached evidence");}
void wrong_final_geometry(){Rig r;r.running();r.current(2,MISSION_STATE_ACTIVE);r.reached(2);r.current(2,MISSION_STATE_COMPLETE);r.tick();r.tick(1.01);require(!r.result("run","COMPLETED"),"far position qualified completion");}
void pause_is_observed(){Rig r;r.running();r.request("id=pause,action=pause,target=run");r.sent();r.ack();r.tick();require(!r.result("pause","COMPLETED"),"pause ACK qualified holding");r.current(1,MISSION_STATE_PAUSED);r.tick();require(r.result("pause","COMPLETED")&&r.result("run","PAUSED")&&r.blocked&&r.v.mode==3,"pause state/lease incorrect");}
void delayed_pause_denial_does_not_fail_resume(){
  Rig r;r.running();r.request("id=pause,action=pause,target=run");r.sent();
  r.current(1,MISSION_STATE_PAUSED);r.tick();
  require(r.result("pause","COMPLETED")&&r.active,"pause needed an ACK");
  r.request("id=resume,action=resume,target=run");r.sent();r.ack(MAV_RESULT_DENIED);r.tick();
  require(r.result("resume","ACK_OBSERVED")&&!r.result("resume","FAILED")&&
          !r.result("resume","COMPLETED")&&r.active,"old denial settled resume or released its lease");
  r.current(1,MISSION_STATE_ACTIVE);r.tick();
  require(r.result("resume","COMPLETED")&&r.active&&r.commands.size()==4,
          "fresh resume evidence did not settle the single send");
}
void resume_requires_paused_same_cursor(){
  Rig r;r.running();const auto count=r.commands.size();
  r.request("id=early,action=resume,target=run");
  require(r.commands.size()==count&&r.result("early","REJECTED"),"running mission admitted resume");
  r.request("id=pause,action=pause,target=run");r.sent();r.ack();r.current(1,MISSION_STATE_PAUSED);r.tick();
  r.request("id=resume,action=resume,target=run");
  require(r.commands.back().command==MAV_CMD_DO_PAUSE_CONTINUE&&r.commands.back().params[0]==1&&
    r.commands.back().operation_expires==r.lease_expiry,"resume lost original bound or wrong command");
  r.sent();r.ack();r.current(2,MISSION_STATE_ACTIVE);r.tick();
  require(!r.result("resume","COMPLETED"),"resume ACK and wrong cursor confirmed");
  r.current(1,MISSION_STATE_ACTIVE);r.tick();
  require(r.result("resume","COMPLETED")&&r.active&&r.text("explicit_same_cursor_resume"),"observed same-cursor resume failed");
}
void stored_takeoff_and_land_require_effects(){
  struct Outcome {unsigned cursor;bool ground,at_target,complete;};
  for(const Outcome outcome:std::vector<Outcome>{{0,true,true,true},{1,true,true,true},
                                                 {2,true,true,false},{1,false,true,false},{0,true,false,false}}) {
    Rig r;r.v.in_air=false;r.v.relative_height=0;r.v.landed_state=MAV_LANDED_STATE_ON_GROUND;
    auto& takeoff=r.proof.items[1];takeoff.command=MAV_CMD_NAV_TAKEOFF;takeoff.x=r.proof.items[0].x;
    r.proof.items[2].params[0]=3;
    auto landing=r.proof.items[2];landing.seq=3;landing.command=MAV_CMD_NAV_LAND;landing.params={0,0,0,1};landing.z=0;
    r.proof.items.push_back(landing);
    r.running();
    r.v.in_air=true;r.v.landed_state=MAV_LANDED_STATE_IN_AIR;r.v.relative_height=5;
    r.current(2,MISSION_STATE_ACTIVE);r.tick();
    r.current(3,MISSION_STATE_ACTIVE);r.tick();
    r.v.landed_state=MAV_LANDED_STATE_LANDING;r.v.in_air=false;r.tick();
    require(!r.result("run","COMPLETED")&&r.active,"landing phase alone qualified touchdown");
    if(outcome.ground){r.v.landed_state=MAV_LANDED_STATE_ON_GROUND;r.v.armed=false;r.v.relative_height=0;}
    if(outcome.at_target){r.v.latitude=landing.x*1e-7;r.v.longitude=landing.y*1e-7;}
    r.current(outcome.cursor,MISSION_STATE_ACTIVE);r.tick();r.tick(1.01);
    require(r.result("run","COMPLETED")==outcome.complete&&r.blocked&&!r.active&&r.nav_acquires==0,
      "stored takeoff/landing did not require bound physical ground and disarm");
  }
}
void abort_needs_mode_evidence(){Rig r;r.running();r.request("id=abort,action=abort,target=run");require(r.commands.back().command==176&&r.commands.back().abort_id=="abort","abort not stop-only");r.sent();r.ack();r.tick();require(!r.result("abort","COMPLETED")&&r.blocked,"abort ACK claimed stopped");r.v.mode=4;r.tick();require(r.result("abort","COMPLETED")&&r.result("run","CANCELLED")&&!r.blocked&&r.nav_acquires==0,"observed handback failed");}
void false_host_release_not_completed(){Rig r;r.running();r.deny_release=true;r.request("id=abort,action=abort,target=run");r.sent();r.ack();r.v.mode=4;r.tick();require(r.result("abort","FAILED")&&!r.result("abort","COMPLETED")&&r.blocked,"host release rejection ignored");}
void explicit_release_host_denial(){Rig r;r.completed();r.v.mode=5;r.deny_release=true;r.request("id=release,action=release,target=run");require(r.result("release","FAILED")&&!r.result("release","COMPLETED")&&r.blocked,"release assumed host success");}
void explicit_release_does_not_acquire_helm(){Rig r;r.completed();r.v.mode=5;r.request("id=release,action=release,target=run");require(r.result("release","COMPLETED")&&!r.blocked&&r.nav_acquires==0,"release acquired helm or retained latch");}
void expiry_not_remote_stop(){Rig r;r.running();const auto n=r.commands.size();r.clock=r.lease_expiry+.01;r.refresh();r.component->onInputBoundary();r.component->iterate();require(r.result("run","TIMED_OUT")&&r.blocked&&r.commands.size()==n&&r.text("remote_execution_may_continue=true"),"expiry claimed remote stop or sent new command");}
void abort_after_expiry_fresh_deadline(){Rig r;r.running();r.clock=r.lease_expiry+.01;r.refresh();r.component->onInputBoundary();r.request("id=abort,action=abort,target=run,timeout=8");require(r.commands.back().abort_id=="abort"&&r.commands.back().operation_expires>r.lease_expiry,"abort renewed old deadline or was blocked");r.sent();r.ack();r.v.mode=4;r.tick();require(r.result("abort","COMPLETED")&&r.result("run","TIMED_OUT")&&!r.result("run","CANCELLED"),"fresh abort rewrote old terminal");}
void pilot_takeover_latches(){Rig r;r.running();const auto n=r.commands.size();r.v.mode=5;r.tick();require(r.result("run","LOST")&&r.blocked&&r.commands.size()==n,"pilot takeover resumed or freed authority");}
void progress_silence_latches(){Rig r;r.running();r.tick(2.6);require(r.result("run","LOST")&&r.blocked,"progress silence accepted");}
void proof_change_latches(){Rig r;r.running();r.proof.revision++;r.tick();require(r.result("run","LOST")&&r.blocked,"changed stored mission kept execution grant");}
void reset_does_not_resume(){Rig r;r.running();const auto n=r.commands.size();r.component->onReset("test");r.tick();r.current(2,MISSION_STATE_ACTIVE);r.tick();require(r.result("run","LOST")&&r.blocked&&r.commands.size()==n,"reset automatically resumed");r.start();require(r.result("run","DUPLICATE"),"reset erased request history");}
void stale_progress_cannot_open_phase(){Rig r;r.start();r.sent();r.ack();r.current(1,MISSION_STATE_NOT_STARTED,9);r.tick();require(r.commands.size()==1,"pre-session cached progress accepted");}
void queue_rejection_before_handoff_can_retire(){Rig r;r.deny_queue=true;r.start();require(r.result("run","REJECTED")&&!r.blocked,"unsent rejected selection retained false authority");}
void clock_regression_latches(){Rig r;r.running();r.clock-=1;r.component->onInputBoundary();require(r.result("run","LOST")&&r.blocked,"regressed owner clock accepted");}
void nonfinite_vehicle_rejected(){for(unsigned n=0;n<3;++n){Rig r;const double bad=std::numeric_limits<double>::quiet_NaN();if(n==0)r.v.home_latitude=bad;if(n==1)r.v.home_msl=bad;if(n==2)r.v.north_velocity=bad;r.start();require(r.commands.empty()&&r.result("run","REJECTED"),"nonfinite flight state admitted");}}
void pre_send_read_ack_rejected(){Rig r;r.start();r.sent();const auto sent=r.clock;r.ackTimed(sent-.001,sent-.001,"READ_ACQUISITION");r.tick();require(r.commands.size()==1&&!r.result("run","ACK_OBSERVED"),"pre-send actual read was rebound to new command");}
void unchanged_original_phase_deadline(){Rig r;r.start();const auto expiry=r.commands.back().expires;r.sent();for(unsigned n=0;n<4;++n){r.ack(MAV_RESULT_IN_PROGRESS);r.tick(.5);}require(r.commands.front().expires==expiry,"IN_PROGRESS renewed phase expiry");r.clock=expiry+.001;r.refresh();r.component->onInputBoundary();r.component->iterate();require(r.result("run","TIMED_OUT")&&r.blocked&&r.commands.size()==1,"original sent phase timeout did not quarantine");}
void unsent_phase_timeout_is_timeout(){Rig r;r.start();r.clock=r.commands.back().expires+.001;r.refresh();r.component->onInputBoundary();r.component->iterate();require(r.result("run","TIMED_OUT")&&!r.blocked,"known unsent timeout lost outcome or retained false lease");}
void late_sent_receipt_quarantines(){Rig r;r.start();const auto expiry=r.commands.back().expires;r.clock=expiry+.001;r.sent();require(r.result("run","TIMED_OUT")&&r.blocked&&!r.result("run","ACTIVE"),"expired actual handoff escaped quarantine");}
void contradictory_progress_latches(){Rig r;r.running();mavlink_mission_current_t c{};c.seq=1;c.total=9;c.mission_state=MISSION_STATE_ACTIVE;mavlink_message_t m{};mavlink_msg_mission_current_encode_status(1,1,&r.codec,&m,&c);r.deliver(m);r.tick();require(r.result("run","LOST")&&r.blocked,"contradictory stored mission count was silently retained");}
void reset_cancels_abort(){Rig r;r.running();r.request("id=abort,action=abort,target=run");const auto wire=r.commands.back();r.sent();r.component->onReset("test");r.v.mode=4;r.ack();r.tick();require(r.result("abort","CANCELLED")&&!r.result("abort","COMPLETED")&&r.blocked&&std::find(r.cancelled.begin(),r.cancelled.end(),wire.id)!=r.cancelled.end(),"reset retained independent abort phase");}
void clock_fault_cancels_abort_and_queued_actions(){Rig r;r.running();r.request("id=abort,action=abort,target=run");r.sent();r.clock-=1;r.component->onInputBoundary();r.v.mode=4;r.ack();r.component->iterate();require(r.result("abort","FAILED")&&!r.result("abort","COMPLETED")&&r.blocked,"clock fault let pending abort publish success");const auto count=r.commands.size();r.request("id=again,action=abort,target=run");require(r.commands.size()==count&&r.result("again","REJECTED"),"clock fault admitted fresh work");}
void pilot_takeover_allows_no_send_release(){
  Rig r;r.running();const auto count=r.commands.size();
  r.v.mode=5;r.tick();r.request("id=abort,action=abort,target=run");
  require(r.result("run","LOST")&&r.result("abort","REJECTED")&&r.blocked&&r.commands.size()==count,
          "pilot-selected mode admitted a Copter wire abort");
  r.v.mode=3;r.tick();r.request("id=reentry,action=abort,target=run");
  require(r.result("reentry","REJECTED")&&r.blocked&&r.commands.size()==count,
          "pilot re-entry into AUTO revived an old abort grant");
  r.v.mode=5;r.tick();r.request("id=release,action=release,target=run");
  require(r.result("release","COMPLETED")&&r.result("run","LOST")&&!r.blocked&&
          r.commands.size()==count&&r.nav_acquires==0,"explicit handback changed pilot mode, terminal result or helm");
}
void first_and_active_progress_required(){Rig r;r.start();r.sent();r.ack();r.current(1,MISSION_STATE_NOT_STARTED);r.tick();r.sent();r.ack();r.v.mode=3;r.current(2,MISSION_STATE_COMPLETE);r.reached(2);r.v.latitude=r.proof.items[2].x*1e-7;r.v.longitude=r.proof.items[2].y*1e-7;r.tick();r.tick(1.01);require(!r.result("run","ACTIVE")&&!r.result("run","COMPLETED"),"completion without post-start ACTIVE/first sequence accepted");}
void result_binds_exact_host_proof(){Rig r;r.running();require(r.text("verification_id=upload,content_id=host.1.7,proof_revision=7,session_epoch=1"),"results omitted typed proof identity");}
void host_denied_release_never_allows_restart(){Rig r;r.running();r.deny_release=true;r.request("id=abort,action=abort,target=run");r.sent();r.ack();r.v.mode=4;r.tick();const auto count=r.commands.size();r.request("id=another,action=start,verification_id=upload,content_id=host.1.7,proof_revision=7");require(r.result("run","LOST")&&r.result("another","REJECTED")&&r.commands.size()==count&&r.blocked,"host-denied handback permitted new execution");}
void retired_handoff_retains_uncertainty(){Rig r;r.start();const auto old=r.commands.back();r.clock=old.expires+.001;r.refresh();r.component->onInputBoundary();require(!r.blocked&&r.result("run","TIMED_OUT"),"unsent timeout precondition");r.component->onCommandSent(old,r.clock,true);r.tick();require(r.blocked&&r.result("run","LATE_HANDOFF")&&r.text("remote_execution_may_continue=true"),"late retired handoff hidden by previous local terminal");r.request("id=another,action=start,verification_id=upload,content_id=host.1.7,proof_revision=7");require(r.result("another","REJECTED")&&r.commands.size()==1,"late handoff permitted new execution");}
void equal_timestamp_ack_rejected(){Rig r;r.start();r.sent();const auto sent=r.last_sent;r.ackTimed(sent,sent,"READ_ACQUISITION");r.tick();require(r.commands.size()==1&&!r.result("run","ACK_OBSERVED"),"equal actual-read timestamp ACK opened phase");}
void equal_timestamp_cursor_rejected(){Rig r;r.start();r.sent();const auto sent=r.last_sent;r.ack();r.current(1,MISSION_STATE_NOT_STARTED,sent);r.tick();require(r.commands.size()==1,"equal-time selected cursor opened start");}
void equal_timestamp_active_rejected(){Rig r;r.start();r.sent();r.ack();r.current(1,MISSION_STATE_NOT_STARTED);r.tick();r.sent();const auto sent=r.last_sent;r.ack();r.v.mode=3;r.current(1,MISSION_STATE_ACTIVE,sent);r.tick();require(!r.result("run","ACTIVE"),"equal-time ACTIVE bound to new start");r.current(1,MISSION_STATE_ACTIVE);r.tick();require(r.result("run","ACTIVE"),"strictly later ACTIVE rejected");}
void equal_timestamp_start_heartbeat_rejected(){Rig r;r.start();r.sent();r.ack();r.current(1,MISSION_STATE_NOT_STARTED);r.tick();r.sent();const auto sent=r.last_sent;r.ack();r.v.mode=3;r.current(1,MISSION_STATE_ACTIVE);r.v.heartbeat_at=sent;r.component->iterate();require(!r.result("run","ACTIVE"),"equal-time AUTO heartbeat bound to new start");r.tick();require(r.result("run","ACTIVE"),"strictly later AUTO heartbeat rejected");}
void equal_timestamp_pause_rejected(){Rig r;r.running();r.request("id=pause,action=pause,target=run");r.sent();const auto sent=r.last_sent;r.ack();r.current(1,MISSION_STATE_PAUSED,sent);r.tick();require(!r.result("pause","COMPLETED"),"equal-time PAUSED qualified new pause");r.current(1,MISSION_STATE_PAUSED);r.tick();require(r.result("pause","COMPLETED"),"strictly later PAUSED rejected");}
void equal_timestamp_abort_heartbeat_rejected(){Rig r;r.running();r.request("id=abort,action=abort,target=run");r.sent();const auto sent=r.last_sent;r.ack();r.v.mode=4;r.v.heartbeat_at=sent;r.component->iterate();require(!r.result("abort","COMPLETED")&&r.blocked,"equal-time Guided heartbeat released lease");r.tick();require(r.result("abort","COMPLETED")&&!r.blocked,"strictly later Guided heartbeat rejected");}
void equal_timestamp_release_heartbeat_rejected(){Rig r;r.completed();r.clock+=.01;r.refresh();r.v.mode=5;r.v.heartbeat_at=r.last_sent;Mail m;m.key="MAVMOOS_MISSION_CONTROL";m.text="schema=1,id=release,action=release,target=run";m.published_at=m.received_at=r.clock;r.component->onMail(m);r.component->iterate();require(r.result("release","REJECTED")&&r.blocked,"equal-time non-AUTO heartbeat qualified release");}
void equal_timestamp_reached_rejected(){Rig r;r.running();r.current(2,MISSION_STATE_ACTIVE);mavlink_mission_item_reached_t reached{};reached.seq=2;mavlink_message_t m{};mavlink_msg_mission_item_reached_encode_status(1,1,&r.codec,&m,&reached);r.deliver(m,r.last_sent);r.current(2,MISSION_STATE_COMPLETE);r.v.latitude=r.proof.items[2].x*1e-7;r.v.longitude=r.proof.items[2].y*1e-7;r.tick();r.tick(1.01);require(!r.result("run","COMPLETED"),"equal-time final reached qualified mission completion");}
void delayed_ack_does_not_refresh_cursor(){Rig r;r.start();r.sent();r.current(1,MISSION_STATE_NOT_STARTED);r.tick(2.6);r.ack();r.tick();require(r.commands.size()==1,"delayed ACK renewed stale selected cursor");r.current(1,MISSION_STATE_NOT_STARTED);r.tick();require(r.commands.size()==2&&r.commands.back().command==300,"fresh selected cursor did not permit bounded start");}
void queue_floor_before_send_preserves_fast_ack(){Rig r;r.start();r.sent();const auto sent=r.last_sent,floor=sent-.001,read=sent+.01;r.ackTimed(floor,read);r.current(1,MISSION_STATE_NOT_STARTED);r.tick();std::cout<<"{\"case\":\"fast_ACK_queue_floor\",\"effective_at\":"<<floor<<",\"sent_at\":"<<sent<<",\"read_at\":"<<read<<",\"queued_commands\":"<<r.commands.size()<<"}\n";require(r.commands.size()==2&&r.commands.back().command==300,"legitimate fast ACK rejected because queue-empty floor preceded send");require(!r.result("run","ACTIVE")&&!r.result("run","COMPLETED"),"ACK promoted execution outcome");}
void fast_ack_does_not_refresh_pre_send_state(){Rig r;r.start();r.sent();const auto sent=r.last_sent;r.current(1,MISSION_STATE_NOT_STARTED,sent);r.ackTimed(sent-.001,r.clock+.01);r.tick();require(r.result("run","ACK_OBSERVED")&&r.commands.size()==1,"ACK replaced required post-handoff effective cursor");r.current(1,MISSION_STATE_NOT_STARTED);r.tick();require(r.commands.size()==2,"fresh cursor rejected after valid read-correlated ACK");}
void wrong_ack_command_and_target_rejected(){for(unsigned n=0;n<3;++n){Rig r;r.start();r.sent();const auto sent=r.last_sent;r.ackTimed(sent-.001,sent+.01,"QUEUE_EMPTY_FLOOR",n==0?300:224,n==1?244:245,n==2?190:191);r.tick();require(r.commands.size()==1&&!r.result("run","ACK_OBSERVED"),"foreign command/ACK target opened phase");}}
void expired_phase_rejects_freshly_read_ack(){Rig r;r.start();r.sent();r.clock=r.commands.back().expires+.001;r.refresh();r.ackTimed(r.clock,r.clock,"READ_ACQUISITION");r.current(1,MISSION_STATE_NOT_STARTED);r.tick();require(r.commands.size()==1&&!r.result("run","ACK_OBSERVED")&&r.result("run","TIMED_OUT"),"fresh ACK renewed an expired original phase");}
void ack_metadata_validity_is_preserved(){for(unsigned n=0;n<4;++n){Rig r;r.start();r.sent();const auto sent=r.last_sent;if(n==0)r.ackTimed(sent+.02,sent+.01);if(n==1)r.ackTimed(sent-.001,sent+.01,"UNKNOWN");if(n==2)r.ackTimed(9,sent+.01);if(n==3)r.ackTimed(sent-.001,sent+2.6);r.tick();require(r.commands.size()==1&&!r.result("run","ACK_OBSERVED"),"invalid metadata became read-correlated protocol evidence");}}
void progress_demand_is_enabled_only(){Rig disabled(false),enabled;require(disabled.component->descriptor().observation_streams.empty(),"disabled feature requested progress traffic");const auto streams=enabled.component->descriptor().observation_streams;require(streams.size()==1&&streams[0].message_id==42&&streams[0].interval_us==500000,"enabled execution lacks bounded primary progress demand");}
}
int main(){unsigned done=0;try{for(const auto test:{disabled,default_host_denial,unarmed_or_ground_rejected,invalid_proof,no_reply_before_send,foreign_ack,fixed_selection_and_start,ack_not_completion,actual_completion_retains_lease,missing_final_reached,wrong_final_geometry,pause_is_observed,delayed_pause_denial_does_not_fail_resume,resume_requires_paused_same_cursor,stored_takeoff_and_land_require_effects,abort_needs_mode_evidence,false_host_release_not_completed,explicit_release_host_denial,explicit_release_does_not_acquire_helm,expiry_not_remote_stop,abort_after_expiry_fresh_deadline,pilot_takeover_latches,progress_silence_latches,proof_change_latches,reset_does_not_resume,stale_progress_cannot_open_phase,queue_rejection_before_handoff_can_retire,clock_regression_latches,nonfinite_vehicle_rejected,pre_send_read_ack_rejected,unchanged_original_phase_deadline,unsent_phase_timeout_is_timeout,late_sent_receipt_quarantines,contradictory_progress_latches,reset_cancels_abort,clock_fault_cancels_abort_and_queued_actions,pilot_takeover_allows_no_send_release,first_and_active_progress_required,result_binds_exact_host_proof,host_denied_release_never_allows_restart,retired_handoff_retains_uncertainty,equal_timestamp_ack_rejected,equal_timestamp_cursor_rejected,equal_timestamp_active_rejected,equal_timestamp_start_heartbeat_rejected,equal_timestamp_pause_rejected,equal_timestamp_abort_heartbeat_rejected,equal_timestamp_release_heartbeat_rejected,equal_timestamp_reached_rejected,delayed_ack_does_not_refresh_cursor,queue_floor_before_send_preserves_fast_ack,fast_ack_does_not_refresh_pre_send_state,wrong_ack_command_and_target_rejected,expired_phase_rejects_freshly_read_ack,ack_metadata_validity_is_preserved,progress_demand_is_enabled_only}){test();++done;}std::cout<<"{\"result\":\"PASS\",\"controls\":"<<done<<",\"scope\":\"pure fake host/command/progress; no socket/native/vehicle\"}\n";return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';std::cout<<"{\"result\":\"FAIL\",\"completed\":"<<done<<"}\n";return 1;}}
