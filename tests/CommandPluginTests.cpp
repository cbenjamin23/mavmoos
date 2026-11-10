#include "ControlComponents.h"
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>

using namespace mavmoos::runtime;
namespace {
void check(bool passed,const std::string& reason) { if(!passed) throw std::runtime_error(reason); }
struct Fake final:Services {
  double clock=100;
  Vehicle state;
  bool allowed=true;
  std::vector<CommandRequest> attempts;
  std::vector<std::string> publications,events,finished;
  Fake() { refresh(); state.mode=4; state.home_valid=true; state.landed_state=MAV_LANDED_STATE_ON_GROUND; }
  void refresh() {
    state.heartbeat_valid=state.position_valid=state.landed_valid=state.estimator_valid=true;
    state.heartbeat_at=state.position_at=state.landed_at=state.estimator_at=clock;
  }
  double now() const override { return clock; }
  const Vehicle& vehicle() const override { return state; }
  void publish(const std::string& key,const std::string& value) override { publications.push_back(key+":"+value); }
  void publish(const std::string&,double) override {}
  bool sendCommand(const CommandRequest& request) override { events.push_back("send"); if(allowed) attempts.push_back(request); return allowed; }
  void commandFinished(const std::string& id) override { finished.push_back(id); }
  bool acquireNavigation(const std::string&) override { return false; }
  void revokeNavigation(const std::string& reason) override { events.push_back(reason); }
  bool navigation(const NavigationTarget&) override { return false; }
  bool contains(const std::string& status) const {
    for(const auto& record:publications) if(record.find(status)!=std::string::npos) return true;
    return false;
  }
};
struct Rig {
  Fake host;
  std::unique_ptr<Plugin> plugin=makeCopterCommands(host);
  void command(const std::string& request,double age=0) {
    host.clock+=.01;
    const auto attempts=host.attempts.size();
    Mail mail; mail.key="MAVMOOS_COMMAND"; mail.text=request; mail.received_at=host.clock; mail.published_at=host.clock-age;
    plugin->onMail(mail); plugin->iterate();
    if(host.attempts.size()>attempts) plugin->onCommandSent(host.attempts.back(),host.clock,true);
  }
  void ack(uint16_t command,uint8_t result) {
    host.clock+=.0001;
    mavlink_message_t message{};
    mavlink_msg_command_ack_pack(1,1,&message,command,result,0,0,245,191);
    plugin->onMessage(message,host.clock);
  }
  void advance(double elapsed) { host.clock+=elapsed; host.refresh(); plugin->iterate(); }
};
void admission()
{
  Rig r;
  check(r.plugin->name()=="commands","module identity");
  r.command("id=bad,action=takeoff,height=nan");
  r.command("id=bad,action=arm,extra=1");
  r.command("id=bad,id=again,action=arm");
  r.command("id=bad,action=unknown");
  r.command("id=stale,action=arm",2.01);
  r.command("id=future,action=arm",-.11);
  check(r.host.attempts.empty(),"malformed and stale requests never send");
  r.command("id=arm1,action=arm");
  check(r.host.attempts.size()==1,"arm submits once");
  check(r.host.attempts[0].command==MAV_CMD_COMPONENT_ARM_DISARM&&r.host.attempts[0].params[0]==1&&r.host.attempts[0].params[1]==0,"ordinary arm without force");
  check(r.host.contains("status=ADMITTED")&&r.host.contains("status=SUBMITTED"),"admission and local send visible");
  r.command("id=arm1,action=arm");
  r.command("id=arm2,action=arm");
  check(r.host.attempts.size()==1&&r.host.contains("status=DUPLICATE")&&r.host.contains("reason=busy"),"duplicate/busy cannot send");
}
void acknowledgements()
{
  Rig r; r.command("id=a,action=arm");
  r.ack(MAV_CMD_NAV_LAND,MAV_RESULT_DENIED);
  check(!r.host.contains("status=FAILED"),"unrelated ACK ignored");
  r.ack(MAV_CMD_COMPONENT_ARM_DISARM,MAV_RESULT_ACCEPTED);
  check(r.host.contains("status=ACK_OBSERVED")&&!r.host.contains("status=COMPLETED"),"ACK is not armed confirmation");
  r.host.state.armed=true; r.plugin->iterate();
  check(!r.host.contains("status=COMPLETED"),"old heartbeat cannot prove new state");
  r.advance(.1);
  check(r.host.contains("status=CONFIRMED")&&r.host.contains("status=COMPLETED"),"new armed heartbeat completes");
  r.command("id=a,action=arm");
  check(r.host.attempts.size()==1,"completed ID not repeated");
  Rig denied; denied.command("id=b,action=arm"); denied.ack(MAV_CMD_COMPONENT_ARM_DISARM,MAV_RESULT_DENIED);
  check(denied.host.contains("status=ACK_OBSERVED,reason=uncorrelated_ack_result_2")&&
        denied.host.finished.empty(),"unbound denial released the lease");
}
void takeoffLifecycle()
{
  Rig r; r.host.state.armed=true;
  r.command("id=badheight,action=takeoff,height=21");
  check(r.host.attempts.empty(),"height envelope rejected");
  r.command("id=up,action=takeoff,height=5");
  check(r.host.attempts.size()==1&&r.host.attempts[0].command==MAV_CMD_NAV_TAKEOFF&&r.host.attempts[0].params[6]==5,"takeoff target encoded");
  r.ack(MAV_CMD_NAV_TAKEOFF,MAV_RESULT_ACCEPTED);
  r.host.state.in_air=true; r.host.state.landed_state=MAV_LANDED_STATE_IN_AIR; r.host.state.relative_height=1;
  r.advance(.2);
  check(r.host.contains("status=CONFIRMED")&&!r.host.contains("status=COMPLETED"),"airborne distinct from target height");
  r.host.state.relative_height=4.6; r.advance(.2);
  check(r.host.contains("status=COMPLETED"),"height reached completes takeoff");
  check(r.host.attempts.size()==1,"no repeating command timer");
  Rig interrupted; interrupted.host.state.armed=true; interrupted.command("id=t,action=takeoff");
  interrupted.host.state.mode=9; interrupted.advance(.2);
  check(interrupted.host.contains("reason=takeoff_interrupted"),"external landing interrupts takeoff");
}
void landingAndHold()
{
  Rig r; r.host.state.armed=true; r.host.state.in_air=true; r.host.state.landed_state=MAV_LANDED_STATE_IN_AIR;
  r.command("id=down,action=land");
  check(r.host.events.size()==1&&r.host.events[0]=="send","module duplicated host-owned authority boundary");
  r.ack(MAV_CMD_NAV_LAND,MAV_RESULT_ACCEPTED); r.host.state.mode=9; r.advance(.1);
  check(r.host.contains("status=CONFIRMED")&&!r.host.contains("status=COMPLETED"),"LAND mode is not touchdown");
  r.host.state.in_air=false; r.host.state.landed_state=MAV_LANDED_STATE_ON_GROUND; r.advance(.1);
  check(r.host.contains("status=COMPLETED"),"fresh on-ground completes land");
  r.command("id=off,action=disarm");
  check(r.host.attempts.back().params[0]==0&&r.host.attempts.back().params[1]==0,"ground disarm never forced");
  Rig hold; hold.host.state.armed=true; hold.host.state.in_air=true;
  hold.command("id=hold,action=hold");
  check(hold.host.attempts[0].command==MAV_CMD_DO_SET_MODE&&hold.host.attempts[0].params[1]==5,"native Loiter explicit hold");
  hold.host.state.mode=5; hold.advance(.1);
  check(hold.host.contains("status=COMPLETED"),"observed Loiter confirms hold request");
  for(unsigned mode:{0u,6u,9u}) {
    Rig outside;outside.host.state.armed=outside.host.state.in_air=true;outside.host.state.mode=mode;
    outside.command("id=land,action=land");
    check(outside.host.attempts.empty()&&outside.host.contains("reason=routine_land_mode_inhibited"),
      "routine LAND overrode pilot, RTL, or an already active LAND");
  }
}
void guardsAndTimeouts()
{
  Rig air; air.host.state.armed=true; air.host.state.in_air=true; air.host.state.landed_state=MAV_LANDED_STATE_IN_AIR;
  air.command("id=unsafe,action=disarm"); check(air.host.attempts.empty(),"airborne disarm inhibited");
  Rig stale; stale.host.state.heartbeat_at=90; stale.command("id=a,action=arm"); check(stale.host.attempts.empty(),"stale heartbeat inhibited");
  Rig mode; mode.host.state.armed=true; mode.host.state.mode=0; mode.command("id=t,action=takeoff"); check(mode.host.attempts.empty(),"takeoff requires observed Guided");
  Rig inhibited; inhibited.host.allowed=false; inhibited.command("id=a,action=arm"); check(inhibited.host.contains("reason=host_send_inhibited"),"host gate authoritative");
  Rig timeout; timeout.command("id=a,action=arm"); timeout.advance(8);
  check(timeout.host.contains("status=TIMED_OUT")&&timeout.host.attempts.size()==1,"missing ACK/state timeout without retry");
  Rig expired; Mail mail; mail.key="MAVMOOS_COMMAND"; mail.text="id=a,action=arm"; mail.published_at=mail.received_at=100.01;
  expired.host.clock=100.01;
  expired.plugin->onMail(mail); expired.host.clock=103; expired.plugin->iterate();
  check(expired.host.attempts.empty()&&expired.host.contains("queued_request_expired"),"queued mail cannot renew expiry");
  Rig rollback; rollback.command("id=a,action=arm"); rollback.host.clock=99; rollback.plugin->iterate();
  check(rollback.host.contains("clock_discontinuity"),"rollback ends pending transaction");
  Rig reset; reset.command("id=reset,action=arm"); reset.plugin->onReset("connection_lost");
  reset.host.state.armed=true; reset.advance(.1);
  check(reset.host.contains("status=CANCELLED,reason=connection_reset")&&!reset.host.contains("status=COMPLETED"),"reset cannot complete old operation with new observations");
  reset.command("id=reset,action=arm");
  check(reset.host.attempts.size()==1&&reset.host.contains("status=DUPLICATE"),"reset retains admitted request history");
  Rig retained; Mail old; old.key="MAVMOOS_COMMAND"; old.text="id=old,action=arm"; old.received_at=old.published_at=100;
  retained.plugin->onMail(old); retained.plugin->iterate();
  check(retained.host.attempts.empty()&&retained.host.contains("pre_session_request"),"retained pre-start request rejected");
}
void deadlineOrdering()
{
  for(double elapsed:{8.0,8.01}) {
    Rig r;r.host.state.mode=5;r.command("id=hard,action=guided");
    r.ack(MAV_CMD_DO_SET_MODE,MAV_RESULT_ACCEPTED);
    r.host.clock=100.01+elapsed;r.host.refresh();r.host.state.mode=4;r.plugin->iterate();
    check(r.host.contains("status=TIMED_OUT,reason=operation_deadline")&&!r.host.contains("status=COMPLETED"),
      "new state at/after hard deadline completed expired request");
  }
  for(double elapsed:{4.0,4.01}) {
    Rig ack;ack.host.state.mode=5;ack.command("id=lateack,action=guided");
    ack.host.clock=100.01+elapsed;ack.ack(MAV_CMD_DO_SET_MODE,MAV_RESULT_ACCEPTED);
    ack.host.refresh();ack.host.state.mode=4;ack.plugin->iterate();
    check(ack.host.contains("status=COMPLETED")&&!ack.host.contains("status=ACK_OBSERVED"),
      "late ACK changed the observation window or emitted a diagnostic after its cutoff");
    Rig state;state.host.state.mode=5;state.command("id=latestate,action=guided");
    state.host.clock=100.01+elapsed;state.host.refresh();state.host.state.mode=4;state.plugin->iterate();
    check(state.host.contains("status=COMPLETED"),"handoff window shortened the observed-state deadline");
  }
  Rig before;before.host.state.mode=5;before.command("id=early,action=guided");
  before.host.state.mode=4;before.advance(3.99);
  check(before.host.contains("status=COMPLETED"),"timely observation could not complete without ACK");
  Rig acked;acked.host.state.armed=true;acked.command("id=acked,action=takeoff");
  acked.ack(MAV_CMD_NAV_TAKEOFF,MAV_RESULT_ACCEPTED);
  acked.host.state.in_air=true;acked.host.state.landed_state=MAV_LANDED_STATE_IN_AIR;acked.host.state.relative_height=5;acked.advance(6);
  check(acked.host.contains("status=COMPLETED"),"timely ACK did not preserve longer takeoff deadline");
  Rig confirmed;confirmed.host.state.armed=true;confirmed.command("id=progress,action=takeoff");
  confirmed.host.state.in_air=true;confirmed.host.state.landed_state=MAV_LANDED_STATE_IN_AIR;confirmed.host.state.relative_height=1;confirmed.advance(.1);
  check(confirmed.host.contains("status=CONFIRMED")&&!confirmed.host.contains("status=COMPLETED"),"progress fixture");
  confirmed.host.state.relative_height=5;confirmed.advance(6);
  check(confirmed.host.contains("status=COMPLETED"),"previously admitted progress did not preserve longer deadline");
  Rig boundary;boundary.host.state.mode=5;boundary.command("id=boundary,action=guided");
  boundary.host.clock=108.01;boundary.plugin->onInputBoundary();
  check(boundary.host.contains("status=TIMED_OUT")&&boundary.host.attempts.size()==1,"boundary hook did not retire without sending");
  boundary.host.refresh();boundary.host.state.mode=4;boundary.plugin->iterate();
  check(!boundary.host.contains("status=COMPLETED"),"retired boundary transaction revived");
}
void operationLeaseBounds()
{
  Rig arm;arm.command("id=arm-lease,action=arm");
  check(arm.host.attempts[0].expires==104.01&&arm.host.attempts[0].operation_expires==108.01,
    "ordinary command did not preserve separate send and operation deadlines");
  Rig takeoff;takeoff.host.state.armed=true;takeoff.command("id=up-lease,action=takeoff");
  check(takeoff.host.attempts[0].operation_expires==145.01,"takeoff lease missed original operation bound");
  Rig land;land.host.state.armed=true;land.command("id=land-lease,action=land");
  check(land.host.attempts[0].operation_expires==160.01,"landing lease missed original operation bound");
  Rig configured;configured.plugin->configure("command_ack_timeout","0.5");
  configured.plugin->configure("takeoff_timeout","7");configured.host.state.armed=true;
  configured.command("id=config-lease,action=takeoff");
  check(configured.host.attempts[0].expires==100.51&&configured.host.attempts[0].operation_expires==107.01,
    "configured operation deadline conflated with ACK/send deadline");
}
void terminalLeaseRelease()
{
  auto once=[](const Rig& r,const std::string& id) {
    check(r.host.finished.size()==1&&r.host.finished[0]==id,"terminal callback did not release exactly matching admitted ID");
  };
  Rig complete;complete.command("id=complete,action=arm");complete.host.state.armed=true;complete.advance(.1);
  complete.plugin->iterate();complete.ack(MAV_CMD_COMPONENT_ARM_DISARM,MAV_RESULT_ACCEPTED);once(complete,"complete");
  Rig denied;denied.command("id=denied,action=arm");denied.ack(MAV_CMD_COMPONENT_ARM_DISARM,MAV_RESULT_DENIED);
  check(denied.host.finished.empty(),"unbound denial retired an admitted ID");
  denied.advance(8);once(denied,"denied");
  Rig timeout;timeout.command("id=timeout,action=arm");timeout.advance(8);timeout.plugin->onInputBoundary();once(timeout,"timeout");
  Rig hard;hard.host.state.mode=5;hard.command("id=hard-lease,action=guided");hard.ack(MAV_CMD_DO_SET_MODE,MAV_RESULT_ACCEPTED);hard.advance(8);once(hard,"hard-lease");
  Rig reset;reset.command("id=reset-lease,action=arm");reset.plugin->onReset("connection_lost");reset.plugin->onReset("again");once(reset,"reset-lease");
  Rig clock;clock.command("id=clock-lease,action=arm");clock.host.clock=99;clock.plugin->onInputBoundary();once(clock,"clock-lease");
  Rig interrupted;interrupted.host.state.armed=true;interrupted.command("id=interrupted-lease,action=takeoff");interrupted.host.state.mode=9;interrupted.advance(.1);once(interrupted,"interrupted-lease");
  Rig inhibited;inhibited.host.allowed=false;inhibited.command("id=not-admitted,action=arm");check(inhibited.host.finished.empty(),"host rejection released an unowned lease");
  Rig queued;queued.host.clock=100.01;Mail m;m.key="MAVMOOS_COMMAND";m.text="id=never-admitted,action=arm";m.received_at=m.published_at=queued.host.clock;
  queued.plugin->onMail(m);queued.plugin->onReset("before_submission");
  check(queued.host.attempts.empty()&&queued.host.finished.empty(),"pre-admission reset released an unowned lease");
}
}
int main()
{
  try {
    admission(); acknowledgements(); takeoffLifecycle(); landingAndHold(); guardsAndTimeouts(); deadlineOrdering();operationLeaseBounds();terminalLeaseRelease();
    std::cout<<"CommandPlugin: 8 groups passed (admission, ACK, takeoff, landing/hold, guards/timeouts, deadline ordering, operation lease bounds, terminal release)\n";
    return 0;
  } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
