// Real Rover command component with fake host services and deterministic observations.
#include "RoverComponents.h"
#include "RoverNativeReplay.h"
#include "NativeReplayDirectory.h"
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <csignal>
#include <fstream>
#include <sys/wait.h>

using namespace mavmoos::runtime;
namespace {
void require(bool value,const std::string& why) { if(!value)throw std::runtime_error(why); }

struct FakeServices final:Services {
  double time=100,lease_until=0;
  Vehicle state;
  bool grant=false,allow_acquire=true,allow_command=true;
  unsigned zero_stops=0,command_cancels=0;
  std::vector<NavigationTarget> targets;
  std::vector<CommandRequest> commands;
  std::vector<std::pair<std::string,std::string>> publications;
  explicit FakeServices(bool boat) {
    state.type=boat?MAV_TYPE_SURFACE_BOAT:MAV_TYPE_GROUND_ROVER;
    state.autopilot=MAV_AUTOPILOT_ARDUPILOTMEGA;
    state.mode=rover::guided_mode;
    state.armed=true;
    state.latitude=42.35;
    state.longitude=-71.08;
    state.heading=90;
    refresh(100);
  }
  void refresh(double at) {
    time=at;
    state.heartbeat_valid=state.position_valid=state.estimator_horizontal_valid=true;
    state.heartbeat_at=state.position_at=state.estimator_at=at;
    state.estimator_flags=EKF_ATTITUDE|EKF_VELOCITY_HORIZ|EKF_POS_HORIZ_ABS;
  }
  double now() const override { return time; }
  const Vehicle& vehicle() const override { return state; }
  void publish(const std::string& k,const std::string& v) override { publications.emplace_back(k,v); }
  void publish(const std::string&,double) override {}
  bool sendCommand(const CommandRequest& request) override {
    if(controlBusy()||!allow_command)return false;
    commands.push_back(request);
    lease_until=request.operation_expires;
    grant=false;
    return true;
  }
  bool acquireNavigation(const std::string&) override {
    if(!allow_acquire||controlBusy())return false;
    return grant=true;
  }
  void revokeNavigation(const std::string&) override {
    if(grant&&state.mode==rover::guided_mode&&state.armed&&time>=state.position_at&&time-state.position_at<=.5)
      ++zero_stops;
    grant=false;
  }
  bool navigation(const NavigationTarget& n) override {
    if(!grant||time>=n.expires)return false;
    targets.push_back(n);
    return true;
  }
  bool navigationGranted() const override { return grant; }
  bool controlBusy() const override { return time<lease_until; }
  void commandFinished(const std::string&) override { lease_until=0; }
  void cancelCommand(const std::string&) override { ++command_cancels; }
};
struct FakeHelm final:Plugin {
  FakeServices& host;
  unsigned acquires=0,releases=0,resets=0;
  explicit FakeHelm(FakeServices& services):host(services) {}
  std::string name() const override { return "fake_helm"; }
  void onMail(const Mail& mail) override {
    if(mail.text=="acquire") { ++acquires;host.acquireNavigation("helm"); }
    if(mail.text=="release") { ++releases;host.revokeNavigation("helm_release"); }
  }
  void onReset(const std::string& reason) override {
    require(reason=="rover_direct_operation","unexpected helm reset reason");
    ++resets;
  }
  void iterate() override {}
};
struct Rig {
  NativeReplayDirectory replay;
  FakeServices host;
  FakeHelm helm{host};
  std::unique_ptr<VehicleControlBackend> backend;
  std::unique_ptr<Plugin> commands;
  explicit Rig(bool boat=false):host(boat),backend(makeArduRoverBackend(boat)),
    commands(makeRoverCommands(host,*backend,&helm)) {
    commands->configure("native_replay_directory",replay.path.string());
  }
  void post(const std::string& value,double received,double published=0,
            const std::string& key="MAVMOOS_CONTROL_REQUEST") {
    host.time=received;
    Mail mail;mail.key=key;mail.text=value;mail.source="operator";
    mail.published_at=published==0?received:published;mail.received_at=received;
    commands->onMail(mail);
  }
  void tick(double at,bool refresh=true) {
    if(refresh)host.refresh(at);else host.time=at;
    commands->iterate();
  }
  void gotoEast(const std::string& id,double at) {
    post("schema=1,id="+id+",action=goto,strategy=guided,lat=42.35,lon=-71.0798",at);
    tick(at);
  }
  void sent(double at,bool accepted=true) {
    host.time=at;
    commands->onCommandSent(host.commands.back(),at,accepted);
  }
  bool row(const std::string& id,const std::string& field,const std::string& key="MAVMOOS_CONTROL_RESULT") const {
    for(const auto& p:host.publications)
      if(p.first==key&&p.second.find(",id="+id+",")!=std::string::npos&&p.second.find(field)!=std::string::npos)return true;
    return false;
  }
  unsigned progress(const std::string& id) const {
    unsigned result=0;
    for(const auto& p:host.publications)
      if(p.first=="MAVMOOS_CONTROL_PROGRESS"&&p.second.find(",id="+id+",")!=std::string::npos)++result;
    return result;
  }
};
} // namespace
int main()
{
  try {
    unsigned count=0;
    const auto test=[&](const char* label,const std::function<void()>& run) { run();++count;std::cout<<"PASS "<<label<<'\n'; };
    test("native replay survives a process boundary and an interface change",[]{
      Rig r;
      r.post("id=prior,action=hold",100.1,0,"MAVMOOS_COMMAND");
      r.tick(100.1);
      require(r.host.commands.size()==1,"original did not reach the host");
      r.commands.reset();r.host.lease_until=0;
      r.commands=makeRoverCommands(r.host,*r.backend,&r.helm);
      r.commands->configure("native_replay_directory",r.replay.path.string());
      r.post("schema=1,id=prior,action=hold,strategy=native",100.2);
      r.tick(100.2);
      require(r.host.commands.size()==1&&r.row("prior","reason=native_id_already_recorded"),
              "replay through the alternate interface reached the host");
    });
    test("current native mode uses fresh observations without sending",[]{
      Rig r;
      r.host.state.mode=rover::hold_mode;
      r.post("schema=1,id=already,action=hold,strategy=native",100.1);
      r.tick(100.1);
      require(r.host.commands.empty()&&r.row("already","status=OBSERVING")&&!r.row("already","status=COMPLETED"),
              "cached Hold was credited or sent again");
      r.tick(100.2);
      require(r.host.commands.empty()&&r.row("already","status=COMPLETED"),"new Hold evidence was not observed");
    });
    test("a transient observed Manual takeover retires queued mail",[]{
      Rig r;
      r.post("schema=1,id=queued,action=hold,strategy=native",100.1);
      r.host.refresh(100.15);r.host.state.mode=rover::manual_mode;
      mavlink_message_t heartbeat{};
      heartbeat.msgid=MAVLINK_MSG_ID_HEARTBEAT;heartbeat.sysid=heartbeat.compid=1;
      r.commands->onMessage(heartbeat,100.15);
      r.host.state.mode=rover::hold_mode;r.tick(100.2);
      require(r.host.commands.empty()&&r.row("queued","reason=pilot_manual_takeover"),
              "leaving Manual revived queued mail");
      r.host.state.mode=rover::manual_mode;
      r.post("schema=1,id=fresh,action=guided,strategy=native",100.3);r.tick(100.3);
      require(r.host.commands.empty()&&r.row("fresh","reason=pilot_manual"),"new ID displaced Manual");
      CommandRequest request;request.command=MAV_CMD_DO_SET_MODE;
      request.params[0]=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;request.params[1]=rover::hold_mode;
      require(!r.backend->commandAllowed(request,r.host.state,100.3),"final-send policy allowed Manual takeover");
    });
    test("an ID denied in Manual cannot revive in Hold after restart",[]{
      Rig r;
      r.host.state.mode=rover::manual_mode;
      r.post("schema=1,id=manual-denied,action=guided,strategy=native",100.1);r.tick(100.1);
      require(r.host.commands.empty()&&r.row("manual-denied","reason=pilot_manual"),"Manual denial missing");
      r.commands.reset();
      r.commands=makeRoverCommands(r.host,*r.backend,&r.helm);
      r.commands->configure("native_replay_directory",r.replay.path.string());
      r.host.state.mode=rover::hold_mode;
      r.post("schema=1,id=manual-denied,action=guided,strategy=native",100.2);r.tick(100.2);
      require(r.host.commands.empty()&&r.row("manual-denied","reason=native_id_already_recorded"),
              "denied ID revived after pilot Hold and restart");
      r.post("schema=1,id=fresh-after-hold,action=guided,strategy=native",100.3);r.tick(100.3);
      require(r.host.commands.size()==1,"distinct fresh Guided request could not follow Hold");
    });
    test("unconfigured native effects fail closed",[]{
      Rig r;
      r.commands=makeRoverCommands(r.host,*r.backend,&r.helm);
      r.post("schema=1,id=no-store,action=hold,strategy=native",100.1);
      r.tick(100.1);
      require(r.host.commands.empty()&&r.row("no-store","reason=native_replay_directory_required"),
              "untracked native request was admitted");
    });
    test("a rejected host handoff cannot resurrect a reserved ID after restart",[]{
      Rig r;
      r.host.allow_command=false;
      r.post("schema=1,id=uncertain,action=hold,strategy=native",100.1);r.tick(100.1);
      require(r.host.commands.empty()&&r.row("uncertain","reason=host_send_inhibited"),
              "fixture did not exercise rejection after recording");
      r.commands.reset();r.host.allow_command=true;
      r.commands=makeRoverCommands(r.host,*r.backend,&r.helm);
      r.commands->configure("native_replay_directory",r.replay.path.string());
      r.post("schema=1,id=uncertain,action=guided,strategy=native",100.2);r.tick(100.2);
      require(r.host.commands.empty()&&r.row("uncertain","reason=native_id_already_recorded"),
              "changing the action revived the reserved ID");
    });
    test("an unsafe replay record prevents startup",[]{
      NativeReplayDirectory directory;
      std::filesystem::create_symlink("/dev/null",directory.path/"61");
      bool rejected=false;
      try {rover::NativeReplay replay(directory.path.string());}
      catch(const std::invalid_argument&) {rejected=true;}
      require(rejected,"a symlink was accepted as a durable ID record");
    });
    test("private replay storage rejects concurrent use, aliases and replacement",[]{
      NativeReplayDirectory directory;
      using Replay=rover::NativeReplay;
      {
        Replay store(directory.path.string());
        bool locked=false;
        try {Replay second(directory.path.string());}
        catch(const std::invalid_argument&) {locked=true;}
        require(locked,"two bridges shared one replay directory");
        require(store.reserve("Case")==Replay::Result::Reserved&&store.reserve("case")==Replay::Result::Reserved,
                "case-distinct IDs collided on a case-insensitive filesystem");
        require(store.reserve("..")==Replay::Result::Reserved,"dot ID did not map to a safe file");
      }
      Replay restarted(directory.path.string());
      require(restarted.seen("Case")&&restarted.seen("case")&&restarted.seen(".."),
              "recorded IDs were not loaded");
      NativeReplayDirectory replacement;
      const auto retired=replacement.path/"retired";
      std::filesystem::rename(directory.path,retired);
      std::filesystem::create_directory(directory.path);
      require(restarted.reserve("new")==Replay::Result::Unavailable,"replaced path allowed a native request");
    });
    test("durable reservation survives process death on either side of handoff",[]{
      using Replay=rover::NativeReplay;
      NativeReplayDirectory directory;
      const auto path=directory.path.string();
      // Import a file written by the former Rover-owned journal.
      { Replay legacy(path);require(legacy.reserve("legacy")==Replay::Result::Reserved,"legacy reservation"); }
      for(const bool send:{false,true}) {
        const std::string id=send?"after":"before";
        int handoff[2];require(pipe(handoff)==0,"cannot create handoff fixture");
        const auto child=fork();
        require(child>=0,"cannot fork replay fixture");
        if(child==0) {
          close(handoff[0]);
          Replay journal(path,"test-route");
          if(journal.reserve(id)!=Replay::Result::Reserved || write(handoff[1],send?"S":"R",1)!=1) _exit(2);
          pause();_exit(3);
        }
        close(handoff[1]);
        char boundary=0;
        const auto count=read(handoff[0],&boundary,1);
        close(handoff[0]);
        kill(child,SIGKILL);
        int status=0;
        require(waitpid(child,&status,0)==child && WIFSIGNALED(status) && count==1 && boundary==(send?'S':'R'),
                "fixture did not die at its selected boundary");
        Replay restarted(path,"test-route");
        require(restarted.seen("legacy")&&restarted.reserve(id)==Replay::Result::Seen,
                "restart lost a consumed request");
      }
      bool wrong_route=false;
      try { Replay changed(path,"different-route"); }
      catch(const std::invalid_argument&) {wrong_route=true;}
      require(wrong_route,"wrong route reused the journal");
    });
    test("full durable journal retains known IDs and denies new reservations",[]{
      using Replay=rover::NativeReplay;
      NativeReplayDirectory directory;
      for(std::size_t i=0;i<Replay::capacity;++i) {
        std::string name;
        for(unsigned char c:std::to_string(i)) {
          constexpr char hex[]="0123456789abcdef";
          name+=hex[c>>4];name+=hex[c&15];
        }
        std::ofstream(directory.path/name).close();
        std::filesystem::permissions(directory.path/name,std::filesystem::perms::owner_read|
          std::filesystem::perms::owner_write);
      }
      Replay full(directory.path.string(),"full-test");
      require(full.size()==Replay::capacity && full.reserve("0")==Replay::Result::Seen &&
              full.reserve("new")==Replay::Result::Full,"full journal discarded old IDs or admitted new ones");
    });
    test("strict strategies and profile-specific bounds",[]{
      Rig ground,boat(true);
      ground.post("schema=1,id=loiter,action=loiter,strategy=native",100.1);
      ground.post("schema=1,id=alt,action=goto,strategy=guided,lat=42.35,lon=-71.08,alt=5",100.11);
      ground.post("schema=1,id=raw,action=hold,strategy=guided,velocity=1",100.12);
      ground.post("schema=2,id=version,action=hold,strategy=guided",100.13);
      ground.post("schema=1,id=dupe,action=hold,action=hold,strategy=guided",100.14);
      ground.post("schema=1,id=far,action=goto,strategy=guided,lat=42.40,lon=-71.08",100.15);
      ground.tick(100.15);
      for(const auto* id:{"loiter","alt","raw","version","dupe","far"})require(ground.row(id,"status=REJECTED"),id);
      require(ground.host.targets.empty()&&ground.host.commands.empty(),"invalid input caused an effect");
      boat.post("schema=1,id=fast,action=goto,strategy=guided,lat=42.35,lon=-71.0798,speed=2.01",100.1);
      boat.tick(100.1);
      require(boat.row("fast","status=REJECTED")&&boat.host.targets.empty(),"boat speed cap bypassed");
      ground.post("schema=1,id=ok,action=goto,strategy=guided,lat=42.35,lon=-71.0798,speed=2.5",100.2);
      ground.tick(100.2);
      require(ground.host.targets.size()==1&&ground.host.targets.back().speed==2.5,"ground cap regressed");
    });
    test("same planar, expiring velocity path for both profiles",[]{
      for(bool boat:{false,true}) {
        Rig r(boat);
        r.post("schema=1,id=east,action=goto,strategy=guided,lat=42.35,lon=-71.0798,speed=1.2,timeout=1",100.15,100.05);
        r.tick(100.16);
        require(r.host.targets.size()==1&&r.row("east","expires=101.050000000"),"missing immutable deadline");
        for(double time:{100.28,100.4,100.52,100.64,100.76,100.88,101.0,101.06,101.2})r.tick(time);
        require(r.host.targets.size()==8&&r.row("east","status=TIMED_OUT,reason=operation_deadline"),"bounded stream did not expire");
        require(r.host.commands.empty()&&!r.host.grant&&r.host.zero_stops==1,"deadline caused a native effect or retained authority");
        for(const auto& n:r.host.targets)
          require(n.kind==NavigationKind::CourseSpeed&&n.dimensions==NavigationDimensions::Planar&&
                  std::abs(n.course-90)<.01&&n.speed==1.2&&n.expires<=101.05000001,"wrong velocity target");
      }
    });
    test("Guided hold is true zero and requires advancing post-target pose",[]{
      Rig r(true);
      r.post("schema=1,id=stop,action=hold,strategy=guided",100.1);
      r.tick(100.1);
      require(r.host.targets.size()==1&&r.host.targets.back().speed==0,"hold target was not zero");
      r.tick(100.2,false);
      r.tick(100.4,false);
      require(r.progress("stop")==0&&!r.row("stop","status=CONFIRMED"),"cached pre-target pose confirmed hold");
      r.host.state.east_velocity=.15;
      for(double time:{100.45,100.85,101.24})r.tick(time);
      require(!r.row("stop","status=COMPLETED"),"settled dwell cut short");
      r.tick(101.46);
      require(r.row("stop","status=COMPLETED")&&r.progress("stop")==4&&!r.row("stop","status=SENT"),"hold evidence or send semantics wrong");
      require(r.host.commands.empty()&&!r.host.grant&&r.host.zero_stops==1,"hold retained an owner or changed mode");
    });
    test("goto arrival requires proximity as well as sustained low speed",[]{
      Rig r;
      r.gotoEast("arrival",100.1);
      for(double time:{100.21,100.61,101.01,101.31})r.tick(time);
      require(!r.row("arrival","status=COMPLETED"),"stationary far from goal completed goto");
      r.host.state.longitude=-71.0798;
      r.host.state.east_velocity=.8;
      r.tick(101.45);
      require(r.host.targets.back().speed==0&&!r.row("arrival","status=COMPLETED"),"arrival did not first slow down");
      r.host.state.east_velocity=.2;
      for(double time:{101.55,101.95,102.35,102.57})r.tick(time);
      require(r.row("arrival","status=COMPLETED")&&!r.host.grant,"near settled outcome not observed");
    });
    test("a destination across the antimeridian uses the short course",[]{
      Rig r;
      r.host.state.latitude=0;r.host.state.longitude=179.99995;
      r.post("schema=1,id=wrap,action=goto,strategy=guided,lat=0,lon=-179.99995",100.1);
      r.tick(100.1);
      require(r.host.targets.size()==1&&std::abs(r.host.targets.front().course-90)<.001,"longitude wrap produced wrong distance/course");
    });
    test("Guided return binds the horizontal home reference",[]{
      Rig r;
      r.post("schema=1,id=nohome,action=return,strategy=guided",100.1);
      r.tick(100.1);
      require(r.row("nohome","reason=armed_home_required"),"return without home accepted");
      r.host.state.home_valid=true;
      r.host.state.home_latitude=42.35;r.host.state.home_longitude=-71.0802;
      r.post("schema=1,id=home,action=return,strategy=guided",100.2);
      r.tick(100.2);
      require(r.host.targets.size()==1&&std::abs(r.host.targets.back().course-270)<.01,"return did not use bound home");
      r.host.state.home_longitude=-71.081;
      r.tick(100.32);
      require(r.row("home","status=CANCELLED,reason=home_reference_changed")&&r.host.targets.size()==1,"return silently retargeted");
    });
    test("old, invalid and replayed helm requests cannot supersede direct control",[]{
      Rig r;
      r.post("acquire",100.1,0,"MAVMOOS_NAV_CONTROL");
      r.gotoEast("direct",100.2);
      require(r.helm.resets==1&&r.helm.acquires==1&&r.host.grant,"direct did not replace helm locally");
      r.post("release",100.21,100.1,"MAVMOOS_NAV_CONTROL");
      r.post("acquire",100.22,100.19,"MAVMOOS_NAV_CONTROL");
      r.post("release",100.23,97,"MAVMOOS_NAV_CONTROL");
      r.post("unknown",100.24,0,"MAVMOOS_NAV_CONTROL");
      r.post("id=native,action=rtl",100.25,97,"MAVMOOS_COMMAND");
      r.tick(100.32);
      require(r.host.grant&&r.host.targets.size()==2&&!r.row("direct","status=CANCELLED")&&r.helm.releases==0,
              "old or malformed mail took control");
      r.post("release",100.33,0,"MAVMOOS_NAV_CONTROL");
      require(!r.host.grant&&r.host.zero_stops==1&&r.helm.releases==0&&r.row("direct","reason=operator_release"),
              "explicit release did not preserve one conditional stop");
    });
    test("accepted helm takeover allows one host stop cycle and is cancelable",[]{
      Rig r;
      r.gotoEast("move",100.1);
      r.post("acquire",100.2,0,"MAVMOOS_NAV_CONTROL");
      require(r.row("move","reason=helm_takeover")&&!r.host.grant&&r.host.zero_stops==1&&r.helm.acquires==0,
              "acquire cleared the stop before a host cycle");
      r.tick(100.21);
      require(r.helm.acquires==0,"old target was replaced before a host cycle");
      r.tick(100.31);
      require(r.helm.acquires==1&&r.host.grant&&r.host.targets.size()==1,"fresh helm request did not take over");
      r.gotoEast("move2",100.4);
      r.post("acquire",100.5,0,"MAVMOOS_NAV_CONTROL");
      r.post("release",100.51,0,"MAVMOOS_NAV_CONTROL");
      r.tick(100.52);r.tick(100.62);
      require(r.helm.acquires==1&&!r.host.grant,"release did not cancel deferred helm acquisition");
    });
    test("a deferred helm handoff cannot recapture a pilot mode change",[]{
      Rig r(true);
      r.gotoEast("move",100.1);
      r.post("acquire",100.2,0,"MAVMOOS_NAV_CONTROL");
      r.host.state.mode=rover::loiter_mode;
      r.host.refresh(100.21);
      mavlink_message_t heartbeat{};
      heartbeat.msgid=MAVLINK_MSG_ID_HEARTBEAT;heartbeat.sysid=heartbeat.compid=1;
      r.commands->onMessage(heartbeat,100.21);
      r.host.state.mode=rover::guided_mode;
      r.tick(100.22);r.tick(100.32);
      require(r.helm.acquires==0&&!r.host.grant,"old handoff crossed a newer pilot mode");
      r.post("acquire",100.33,0,"MAVMOOS_NAV_CONTROL");
      require(r.helm.acquires==1&&r.host.grant,"fresh acquisition after mode change was denied");
    });
    test("rejected direct admission does not clear the old helm",[]{
      Rig r;
      r.post("acquire",100.1,0,"MAVMOOS_NAV_CONTROL");
      r.host.allow_acquire=false;
      r.gotoEast("denied",100.2);
      require(r.row("denied","reason=host_navigation_inhibited")&&r.helm.resets==0&&r.host.grant&&r.host.targets.empty(),
              "rejected request reset or took the old grant");
    });
    test("queued cancel and later duplicate cannot cause a send",[]{
      Rig r;
      const std::string move="schema=1,id=move,action=goto,strategy=guided,lat=42.35,lon=-71.0798";
      r.post(move,100.1);
      r.post("schema=1,id=old,action=cancel,target=move",100.11,100.09);
      r.post("schema=1,id=cancel,action=cancel,target=move",100.12);
      r.tick(100.2);
      r.post(move,100.21);
      r.post("id=cancel,action=hold",100.22,0,"MAVMOOS_COMMAND");
      require(r.row("old","reason=pre_operation_cancel")&&r.row("move","status=CANCELLED")&&
              r.row("cancel","status=COMPLETED")&&r.row("move","status=DUPLICATE")&&
              r.row("cancel","status=DUPLICATE","MAVMOOS_COMMAND_RESULT"),"cancel/ID result mismatch");
      require(r.host.targets.empty()&&r.host.commands.empty()&&r.host.zero_stops==0,"waiting work survived cancel");
    });
    test("active cancel retires velocity and preserves native effect uncertainty",[]{
      Rig r;
      r.gotoEast("direct",100.1);
      r.post("schema=1,id=c1,action=cancel,target=direct",100.15);
      r.tick(100.2);
      require(r.row("direct","reason=operator_cancel")&&r.row("c1","status=COMPLETED")&&
              r.host.targets.size()==1&&r.host.zero_stops==1,"active stream survived cancel");
      r.post("id=hold,action=hold",100.25,0,"MAVMOOS_COMMAND");
      r.tick(100.25);
      require(r.host.commands.size()==1,"legacy native queue missing");
      r.sent(100.26);
      r.post("schema=1,id=c2,action=cancel,target=hold",100.27);
      r.post("acquire",100.28,0,"MAVMOOS_NAV_CONTROL");
      r.post("schema=1,id=busy,action=hold,strategy=guided",100.29);
      require(r.row("hold","reason=operator_cancel_effect_may_persist","MAVMOOS_COMMAND_RESULT")&&
              r.row("c2","status=COMPLETED")&&r.host.command_cancels==1&&r.host.controlBusy()&&r.helm.acquires==0&&
              r.row("busy","reason=busy"),"sent native uncertainty did not retain lease");
      r.post("acquire",108.26,0,"MAVMOOS_NAV_CONTROL");
      require(r.helm.acquires==1&&!r.host.controlBusy(),"uncertainty lease exceeded original deadline");
    });
    test("fresh pilot takeover retires a waiting incompatible native command",[]{
      Rig r;
      r.post("schema=1,id=stop,action=hold,strategy=native",100.1);
      r.tick(100.1);
      require(r.host.controlBusy()&&r.host.commands.size()==1,"native queue missing");
      r.host.state.mode=0;
      r.tick(100.2);
      require(r.row("stop","status=CANCELLED,reason=mode_changed_before_handoff")&&!r.host.controlBusy(),
              "queued command retained authority after a newer Manual observation");
      r.sent(100.21);
      require(!r.row("stop","status=SENT"),"late receipt reopened cancelled command");
    });
    test("pilot mode and telemetry discontinuity do not silently reacquire",[]{
      Rig r;
      r.gotoEast("manual",100.1);
      r.host.state.mode=0;
      r.tick(100.2);
      r.host.state.mode=rover::guided_mode;
      r.tick(100.3);
      require(r.row("manual","status=CANCELLED,reason=external_mode_change")&&r.host.targets.size()==1&&
              r.host.zero_stops==0&&r.host.commands.empty(),"pilot takeover caused a mode or motion effect");
      r.gotoEast("loss",100.4);
      r.host.time=100.92;
      r.commands->onInputBoundary();
      r.tick(100.92);
      require(r.row("loss","reason=armed_guided_horizontal_state_lost")&&r.host.targets.size()==2&&!r.host.grant,
              "fresh ingress concealed the prior continuity loss");
    });
    test("boat Loiter uses native mode and independent post-mode stationkeeping evidence",[]{
      Rig r(true);
      r.post("schema=1,id=loiter,action=loiter,strategy=native",100.1);
      r.tick(100.1);
      require(r.host.commands.size()==1&&r.host.commands.back().params[1]==5,"boat Loiter did not request mode 5");
      r.sent(100.11);
      r.host.state.mode=rover::loiter_mode;
      r.host.state.heartbeat_at=100.12;
      r.tick(100.12,false);
      require(r.row("loiter","status=CONFIRMED")&&!r.row("loiter","status=COMPLETED")&&r.progress("loiter")==0,
              "mode observation completed Loiter without independent pose");
      r.host.state.east_velocity=.15;
      r.tick(100.2);
      r.host.state.longitude+=.0001;
      for(double time:{100.6,101.0,101.39})r.tick(time);
      require(!r.row("loiter","status=COMPLETED"),"stationkeeping dwell ignored displacement");
      r.tick(101.61);
      require(r.row("loiter","status=COMPLETED")&&!r.host.controlBusy(),"settled native Loiter did not complete");
      r.post("id=hold,action=hold",101.7,0,"MAVMOOS_COMMAND");
      r.tick(101.7);
      require(r.host.commands.back().params[1]==4,"boat native Hold silently became Loiter");
    });
    test("native boat Hold allows coasting time without extending its handoff or an explicit deadline",[]{
      for(bool boat:{false,true}) for(bool versioned:{false,true}) {
        Rig r(boat);
        if(versioned)r.post("schema=1,id=hold,action=hold,strategy=native",100.1);
        else r.post("id=hold,action=hold",100.1,0,"MAVMOOS_COMMAND");
        r.tick(100.1);
        require(r.host.commands.size()==1&&r.host.commands.back().params[1]==4,"Hold was not admitted");
        const auto& command=r.host.commands.back();
        require(std::abs(command.expires-104.1)<1e-6&&
                std::abs(command.operation_expires-(boat?125.1:108.1))<1e-6,
                "Hold duration changed the bounded send or used the wrong physical deadline");
      }
      Rig boat(true);
      boat.post("schema=1,id=hold,action=hold,strategy=native,timeout=10",100.2);
      boat.tick(100.2);
      require(boat.host.commands.size()==1&&std::abs(boat.host.commands.back().operation_expires-110.2)<1e-6,
              "boat Hold lengthened the operator's original deadline");
    });
    test("versioned native return keeps separate handoff and observed outcome",[]{
      Rig r;
      r.host.state.home_valid=true;
      r.host.state.home_latitude=r.host.state.latitude;r.host.state.home_longitude=r.host.state.longitude;
      r.post("schema=1,id=rtl,action=return,strategy=native,timeout=45",100.1);
      r.tick(100.1);
      require(r.host.commands.size()==1&&r.host.commands.back().command==MAV_CMD_NAV_RETURN_TO_LAUNCH&&
              r.row("rtl","expires=145.100000000")&&!r.row("rtl","status=SENT"),"native return admission semantics wrong");
      r.sent(100.11);
      r.host.state.mode=rover::rtl_mode;
      r.tick(100.2);
      require(r.row("rtl","status=SENT")&&r.row("rtl","status=COMPLETED")&&r.host.targets.empty(),
              "native return did not use independent native completion path");
    });
    std::cout<<"PASS "<<count<<" Rover operation groups\n";
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<"FAIL "<<e.what()<<'\n';return 1;
  }
}
