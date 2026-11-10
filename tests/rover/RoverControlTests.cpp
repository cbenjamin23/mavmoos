// Real combined Rover DSO with deterministic observations and captured sends.
#include "PluginHost.h"
#include "NativeReplayDirectory.h"
#include "RoverComponents.h"
#include "Runtime.h"
#include <algorithm>
#include <cmath>
#include <dlfcn.h>
#include <functional>
#include <iostream>
#include <set>
#include <stdexcept>
#include <vector>
using namespace mavmoos::runtime;
namespace {
void require(bool value,const std::string& why){if(!value)throw std::runtime_error(why);}
struct Fake final:Services {
  double time=100,permit=0;Vehicle state;bool granted=false,busy=false;
  unsigned acquires=0,resumes=0;
  std::vector<CommandRequest> commands;std::vector<NavigationTarget> targets;
  std::vector<std::pair<std::string,std::string>> publications;
  Fake(){fresh();}
  void fresh(){state.type=10;state.autopilot=3;state.mode=15;state.armed=true;state.in_air=false;
    state.heartbeat_valid=state.position_valid=state.estimator_horizontal_valid=true;
    state.heartbeat_at=state.position_at=state.estimator_at=time;
    state.estimator_flags=EKF_ATTITUDE|EKF_VELOCITY_HORIZ|EKF_POS_HORIZ_ABS;
    state.latitude=42.358456;state.longitude=-71.087589;state.heading=90;
    state.home_valid=state.landed_valid=state.estimator_valid=false;
    state.relative_height=state.msl_height=NAN;}
  double now() const override{return time;}const Vehicle& vehicle() const override{return state;}
  void publish(const std::string& k,const std::string& v) override{publications.emplace_back(k,v);}
  void publish(const std::string&,double) override{}
  bool sendCommand(const CommandRequest& request) override{commands.push_back(request);busy=true;granted=false;permit=0;return true;}
  bool acquireNavigation(const std::string&) override{if(busy)return false;++acquires;granted=true;permit=0;return true;}
  bool navigationGranted()const override{return granted;}
  bool controlBusy()const override{return busy;}
  void revokeNavigation(const std::string&) override{granted=false;permit=0;}
  bool navigation(const NavigationTarget& n) override{if(!granted||time>=n.expires||state.mode!=15)return false;targets.push_back(n);return true;}
  bool suspendNavigation(double until)override{if(!granted||busy)return false;granted=false;permit=until;return true;}
  bool resumeNavigation(const std::string&)override{if(busy||time>=permit||state.mode!=15)return false;granted=true;permit=0;++resumes;return true;}
  void commandFinished(const std::string&)override{busy=false;}
  bool has(const std::string& key,const std::string& value)const{
    for(const auto& p:publications)if(p.first==key&&p.second.find(value)!=std::string::npos)return true;return false;
  }
};
struct Rig {
  NativeReplayDirectory replay;
  Fake services;PluginHost host;
  explicit Rig(const std::string& path) {
    require(host.load(path,services),"load Rover: "+host.lastError());
    host.configure("shadow_mode","false");
    require(host.configureModules({{"ardurover_control",{{"native_replay_directory",replay.path.string()}}}}),
            "configure Rover replay directory");
  }
  Plugin& plugin(){require(host.plugins().size()==1,"one real provider required");return *host.plugins()[0];}
  void text(const std::string& key,const std::string& value,double at,const std::string& source="operator"){
    services.time=at;Mail m;m.key=key;m.text=value;m.source=source;m.received_at=m.published_at=at;host.dispatchMail(m);
  }
  void number(const std::string& key,double value,double at,const std::string& source="pHelmIvP"){
    services.time=at;Mail m;m.key=key;m.value=value;m.numeric=true;m.source=source;m.received_at=m.published_at=at;host.dispatchMail(m);
  }
  void fields(double at,const std::string& source="pHelmIvP"){
    services.time=at;services.fresh();text("IVPHELM_STATE","DRIVE",at,source);
    number("DESIRED_HEADING",90,at,source);number("DESIRED_SPEED",1,at,source);
  }
  void acquire(double at=100.1){
    services.time=at;services.fresh();text("IVPHELM_ALLSTOP","clear",at,"pHelmIvP");
    text("IVPHELM_STATE","DRIVE",at+.01,"pHelmIvP");text("MAVMOOS_NAV_CONTROL","acquire",at+.02);
    fields(at+.03);host.iterate();
  }
  void ack(double effective,double read,uint8_t system=1,uint8_t component=1,
           uint8_t target_system=245,uint8_t target_component=191)
  {
    services.time=read;
    mavlink_command_ack_t ack{};ack.command=MAV_CMD_DO_SET_MODE;ack.result=MAV_RESULT_DENIED;
    ack.target_system=target_system;ack.target_component=target_component;
    mavlink_message_t message{};
    mavlink_msg_command_ack_encode(system,component,&message,&ack);
    host.dispatchMessage(message,{effective,read,"QUEUE_EMPTY_FLOOR"});
  }
};
std::set<std::string> capabilities(const PluginDescriptor& d){std::set<std::string>s;for(const auto& c:d.capabilities)require(s.insert(c.name).second,"duplicatecapability");return s;}

struct RoverRuntimeRig {
  NativeReplayDirectory replay;
  double clock=100;
  const uint8_t type;
  int32_t latitude=423584560,home_latitude=423584560;
  int16_t north_velocity=0;
  std::vector<mavlink_command_long_t> commands;
  std::vector<mavlink_set_position_target_global_int_t> targets;
  std::vector<std::pair<std::string,std::string>> publications;
  std::function<void(const mavlink_message_t&)> before_send;
  Runtime runtime;

  RoverRuntimeRig(const std::string& path,bool boat)
    :type(boat?MAV_TYPE_SURFACE_BOAT:MAV_TYPE_GROUND_ROVER),
     runtime([this](const std::string& key,const std::string& value,double,bool numeric) {
               if(!numeric) publications.emplace_back(key,value);
             },[](const uint8_t*,std::size_t){return false;},true,false,{},type,MAV_AUTOPILOT_ARDUPILOTMEGA)
  {
    require(runtime.load(path,"ardurover_control"),"Runtime could not load the real Rover provider");
    runtime.configure("vehicle_profile",boat?"ardurover_boat":"ardurover");
    runtime.configureModules({{"ardurover_control",{{"native_replay_directory",replay.path.string()}}}});
    runtime.setBoundedSend([this](const uint8_t* bytes,std::size_t size,double,const Runtime::SendEligibility& eligible) {
      mavlink_message_t buffer{},packet{};
      mavlink_status_t parser{},result{};
      bool decoded=false;
      for(std::size_t i=0;i<size;++i)
        decoded=mavlink_frame_char_buffer(&buffer,&parser,bytes[i],&packet,&result)==MAVLINK_FRAMING_OK;
      require(decoded && packet.sysid==245 && packet.compid==191,"invalid Rover wire frame");
      if(before_send) before_send(packet);
      if(!eligible(clock)) return false;
      if(packet.msgid==MAVLINK_MSG_ID_COMMAND_LONG) {
        mavlink_command_long_t command{};
        mavlink_msg_command_long_decode(&packet,&command);
        if(command.command==MAV_CMD_DO_SET_MODE || command.command==MAV_CMD_COMPONENT_ARM_DISARM ||
           command.command==MAV_CMD_NAV_RETURN_TO_LAUNCH) commands.push_back(command);
      } else if(packet.msgid==MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT) {
        mavlink_set_position_target_global_int_t target{};
        mavlink_msg_set_position_target_global_int_decode(&packet,&target);
        targets.push_back(target);
      }
      return true;
    });
  }

  void heartbeat(unsigned mode,bool armed=true)
  {
    mavlink_message_t packet{};
    mavlink_msg_heartbeat_pack(1,1,&packet,type,MAV_AUTOPILOT_ARDUPILOTMEGA,
      MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|(armed?MAV_MODE_FLAG_SAFETY_ARMED:0),mode,MAV_STATE_ACTIVE);
    runtime.message(packet,clock);
  }

  void observe(unsigned mode,bool armed=true,double interval=.02)
  {
    clock+=interval;
    heartbeat(mode,armed);
    mavlink_message_t packet{};
    mavlink_msg_global_position_int_pack(1,1,&packet,0,latitude,-710875890,0,0,north_velocity,0,0,9000);
    runtime.message(packet,clock);
    mavlink_msg_ekf_status_report_pack(1,1,&packet,EKF_ATTITUDE|EKF_VELOCITY_HORIZ|EKF_POS_HORIZ_ABS,
      .01f,.01f,.01f,.01f,.01f,0);
    runtime.message(packet,clock);
    const float q[]{1,0,0,0};
    mavlink_msg_home_position_pack(1,1,&packet,home_latitude,-710875890,0,0,0,0,q,0,0,0,0);
    runtime.message(packet,clock);
  }

  void text(const std::string& key,const std::string& value,const std::string& source="operator")
  {
    runtime.mail({key,value,source,0,clock,clock,false});
  }

  void helm(double course=90)
  {
    const std::string source="pHelmIvP";
    text("IVPHELM_STATE","DRIVE",source);
    text("IVPHELM_ALLSTOP","clear",source);
    runtime.mail({"DESIRED_HEADING","",source,course,clock,clock,true});
    runtime.mail({"DESIRED_SPEED","",source,1,clock,clock,true});
  }

  void tick() { runtime.tick(clock); }

  bool result(bool versioned,const std::string& id,const std::string& field) const
  {
    const auto* key=versioned?"MAVMOOS_CONTROL_RESULT":"MAVMOOS_COMMAND_RESULT";
    return std::any_of(publications.begin(),publications.end(),[&](const auto& row) {
      return row.first==key && row.second.find(",id="+id+",")!=std::string::npos &&
        row.second.find(","+field+",")!=std::string::npos;
    });
  }

  void accept(uint16_t command)
  {
    clock+=.01;
    mavlink_message_t packet{};
    mavlink_msg_command_ack_pack(1,1,&packet,command,MAV_RESULT_ACCEPTED,0,0,245,191);
    runtime.message(packet,clock);
  }
};

void checkNativeVariants(const std::string& path)
{
  struct Action {
    const char* legacy;
    const char* versioned;
    uint16_t command;
    float param1,param2;
    unsigned from,to;
    bool armed_before,armed_after,boat_only;
  };
  constexpr Action actions[]{
    {"arm","arm",MAV_CMD_COMPONENT_ARM_DISARM,1,0,4,4,false,true,false},
    {"disarm","disarm",MAV_CMD_COMPONENT_ARM_DISARM,0,0,4,4,true,false,false},
    {"guided","guided",MAV_CMD_DO_SET_MODE,1,15,4,15,true,true,false},
    {"hold","hold",MAV_CMD_DO_SET_MODE,1,4,15,4,true,true,false},
    {"rtl","return",MAV_CMD_NAV_RETURN_TO_LAUNCH,0,0,15,11,true,true,false},
    {"loiter","loiter",MAV_CMD_DO_SET_MODE,1,5,15,5,true,true,true}
  };
  for(bool boat:{false,true}) for(bool versioned:{false,true}) for(const auto& action:actions) {
    RoverRuntimeRig rig(path,boat);
    const std::string name=versioned?action.versioned:action.legacy;
    const std::string label=(boat?"boat ":"ground ")+name+(versioned?" versioned":" legacy");
    rig.observe(action.from,action.armed_before);
    if(versioned)
      rig.text("MAVMOOS_CONTROL_REQUEST","schema=1,id=native,action="+name+",strategy=native");
    else
      rig.text("MAVMOOS_COMMAND","id=native,action="+name);
    rig.tick();
    if(action.boat_only && !boat) {
      require(rig.commands.empty() && rig.result(versioned,"native","status=REJECTED"),
              label+" escaped the ground Rover profile");
      continue;
    }
    require(rig.commands.size()==1 && rig.targets.empty() && rig.result(versioned,"native","status=SENT"),
            label+" did not use one native command handoff");
    const auto& command=rig.commands.front();
    require(command.target_system==1 && command.target_component==1 && command.command==action.command &&
            command.param1==action.param1 && command.param2==action.param2 && command.confirmation==0,
            label+" sent the wrong target, command or variant");
    rig.accept(action.command);
    require(rig.result(versioned,"native","status=ACK_OBSERVED") &&
            !rig.result(versioned,"native","status=COMPLETED") && rig.runtime.controlBusy(),
            label+" completed on the protocol reply without independent observation");
    rig.observe(action.to,action.armed_after);
    rig.tick();
    if(action.boat_only) {
      require(!rig.result(versioned,"native","status=COMPLETED"),label+" skipped its stationary dwell");
      for(int i=0;i<3;++i) {
        rig.observe(action.to,action.armed_after,.4);
        rig.tick();
      }
    }
    require(rig.result(versioned,"native","status=COMPLETED") && !rig.runtime.controlBusy() &&
            !rig.runtime.navigationGranted() && rig.commands.size()==1 && rig.targets.empty(),
            label+" failed to complete once from FC state or changed navigation ownership");
  }
}

void checkOwnersAndFinalGate(const std::string& path)
{
  for(bool boat:{false,true}) {
    RoverRuntimeRig rig(path,boat);
    const auto resting=boat?rover::loiter_mode:rover::hold_mode;
    rig.observe(resting);
    rig.helm();
    rig.clock+=.01;
    rig.text("MAVMOOS_NAV_CONTROL","acquire");
    rig.tick();
    require(rig.commands.size()==1 && rig.commands.front().command==MAV_CMD_DO_SET_MODE &&
            rig.commands.front().param2==rover::guided_mode && rig.targets.empty(),
            "Rover acquisition did not request Guided without moving in the native mode");
    rig.observe(rover::guided_mode);
    rig.tick();
    require(rig.targets.empty(),"pre-acquisition stock scalars escaped after Guided entry");
    rig.observe(rover::guided_mode,true,.12);
    rig.helm();
    rig.tick();
    require(rig.targets.size()==1 && rig.targets.back().type_mask==0x0DE7 &&
            std::abs(rig.targets.back().vy-1)<1e-5,"fresh helm did not send planar eastward motion");

    rig.observe(rover::guided_mode,true,.12);
    rig.helm();
    rig.text("MAVMOOS_CONTROL_REQUEST","schema=1,id=north,action=goto,strategy=guided,lat=42.3586,lon=-71.087589");
    rig.tick();
    require(rig.targets.size()==2 && rig.targets.back().vx>.9 && std::abs(rig.targets.back().vy)<1e-5,
            "direct route and cached helm did not resolve to the new northward owner");
    rig.observe(rover::guided_mode,true,.12);
    rig.helm();
    rig.text("MAVMOOS_COMMAND","id=busy,action=hold");
    rig.tick();
    require(rig.result(false,"busy","reason=busy") && rig.commands.size()==1 &&
            rig.targets.size()==3 && rig.targets.back().vx>.9,"native request or stale helm displaced the direct route");

    rig.observe(rover::guided_mode);
    rig.text("MAVMOOS_CONTROL_REQUEST","schema=1,id=cancel,action=cancel,target=north");
    rig.tick();
    require(rig.result(true,"north","status=CANCELLED") && rig.result(true,"cancel","status=COMPLETED") &&
            rig.targets.size()==4 && rig.targets.back().type_mask==0x09E7 &&
            rig.targets.back().vx==0 && rig.targets.back().vy==0 && !rig.runtime.navigationGranted(),
            "direct cancellation failed to fence motion and preserve one conditional stop");
    rig.observe(rover::guided_mode,true,.12);
    rig.helm();
    rig.tick();
    require(rig.targets.size()==4,"fresh producer mail silently regained helm authority");
    rig.clock+=.01;
    rig.text("MAVMOOS_NAV_CONTROL","acquire");
    rig.observe(rover::guided_mode,true,.12);
    rig.helm();
    bool serialized=false;
    rig.before_send=[&](const mavlink_message_t& packet) {
      if(packet.msgid!=MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT) return;
      serialized=true;
      rig.clock+=.001;
      rig.heartbeat(0); // The pilot selected Manual before the socket accepted the encoded target.
    };
    rig.tick();
    require(serialized && rig.targets.size()==4 && !rig.runtime.navigationGranted(),
            "a target escaped after Manual arrived at the final send check");
    rig.observe(rover::guided_mode,true,.12);
    rig.helm();
    rig.tick();
    require(rig.targets.size()==4 && rig.commands.size()==1 && !rig.runtime.navigationGranted(),
            "fresh Guided or stock helm mail revived a pilot-retired target");
  }
}

void checkDirectSettling(const std::string& path)
{
  for(bool boat:{false,true}) for(bool returning:{false,true}) {
    RoverRuntimeRig rig(path,boat);
    const std::string action=returning?"return":"hold";
    rig.home_latitude-=1600;
    rig.observe(rover::guided_mode);
    rig.text("MAVMOOS_CONTROL_REQUEST","schema=1,id=direct,action="+action+",strategy=guided");
    rig.tick();
    require(rig.targets.size()==1 && rig.targets.back().coordinate_frame==MAV_FRAME_GLOBAL &&
            rig.targets.back().target_system==1 && rig.targets.back().target_component==1 &&
            rig.targets.back().type_mask==(returning?0x0DE7:0x09E7) &&
            std::abs(rig.targets.back().vx-(returning?-1.f:0.f))<1e-5 &&
            std::abs(rig.targets.back().vy)<1e-5,
            "Guided return did not head to bound home or Guided hold did not request true zero");
    require(!rig.result(true,"direct","status=CONFIRMED"),"cached pose confirmed Guided work");
    if(returning) rig.latitude=rig.home_latitude;
    rig.north_velocity=70;
    rig.observe(rover::guided_mode,true,.12);
    rig.tick();
    require(rig.result(true,"direct","status=CONFIRMED") && !rig.result(true,"direct","status=COMPLETED") &&
            rig.targets.back().type_mask==0x09E7 && rig.targets.back().vx==0 && rig.targets.back().vy==0,
            "moving pose finished a Guided stop or continued driving after reaching the goal");
    rig.north_velocity=0;
    for(int i=0;i<3;++i) {
      rig.observe(rover::guided_mode,true,.35);
      rig.tick();
    }
    require(!rig.result(true,"direct","status=COMPLETED"),"short stopped dwell finished Guided work");
    rig.observe(rover::guided_mode,true,.35);
    rig.tick();
    require(rig.result(true,"direct","status=COMPLETED") && !rig.runtime.navigationGranted() &&
            !rig.runtime.controlBusy() && rig.commands.empty(),
            "settled Guided work failed to close or changed the FC mode");
  }
}
}
int main(int argc,char** argv)
{
  try {
    require(argc==2,"usage: rover_control_tests ROVER_MODULE");const std::string path=argv[1];unsigned count=0;
    const auto test=[&](const char* label,const std::function<void()>& run){run();++count;std::cout<<"PASS "<<label<<'\n';};
    test("factory enforces the current ABI and produces no effect",[&]{
      void* h=dlopen(path.c_str(),RTLD_NOW|RTLD_LOCAL|RTLD_NODELETE);require(h,"dlopen Rover");
      auto create=reinterpret_cast<CreatePlugin>(dlsym(h,"mavmoos_create_plugin"));auto destroy=reinterpret_cast<DestroyPlugin>(dlsym(h,"mavmoos_destroy_plugin"));require(create&&destroy,"exports missing");
      Fake f;require(!create(6,&f)&&!create(plugin_abi+1,&f)&&!create(plugin_abi,nullptr),"badABI/context accepted");auto* p=create(plugin_abi,&f);require(p&&p->name()=="ardurover_control","wrongidentity");destroy(p);dlclose(h);
      require(f.commands.empty()&&f.targets.empty()&&f.acquires==0,"factory caused effect");
    });
    test("one coherent provider advertises only supported Rover paths",[&]{
      Rig r(path);const auto d=r.plugin().descriptor();const auto c=capabilities(d);
      require(d.profile=="ardurover"&&d.exclusive_resources==std::vector<std::string>{"vehicle.control"},"wrong profile/role");
      require(c==std::set<std::string>{"control.arm","control.disarm","control.guided","control.hold.native_stop","control.return.native_rtl","control.navigation.helm","control.hold.guided_zero_velocity","control.operations.native","control.operations.guided","control.operations.cancel"},"unsupported paths advertised");
      require(r.plugin().controlBackend()&&r.plugin().controlBackend()->helmProfile().dimensions==NavigationDimensions::Planar,"planarbackend missing");
      require(!r.host.streamedParameterBenchCompatible(),"control admitted to streamedbench");
      require(!r.host.load(path,r.services)&&r.host.plugins().size()==1,"duplicate provider accepted");
    });
    test("planar subscription set has no desired altitude",[&]{
      Rig r(path);const auto s=r.plugin().subscriptions();require(std::find(s.begin(),s.end(),"DESIRED_ALTITUDE")==s.end(),"Rover subscribes to altitude");
      for(const auto& key:{"DESIRED_HEADING","DESIRED_SPEED","IVPHELM_STATE","IVPHELM_ALLSTOP","MAVMOOS_NAV_CONTROL","MAVMOOS_COMMAND","MAVMOOS_CONTROL_REQUEST"})require(std::find(s.begin(),s.end(),key)!=s.end(),"Rover input missing");
    });
    test("boat profile shares the planar helm and advertises its own native Loiter",[&]{
      Rig r(path);r.host.configure("vehicle_profile","ardurover_boat");
      const auto d=r.plugin().descriptor();
      require(d.profile=="ardurover_boat"&&capabilities(d).count("control.hold.native_loiter")==1,"boat catalog");
      const auto* backend=r.plugin().controlBackend();
      require(backend&&backend->identityCompatible(MAV_TYPE_SURFACE_BOAT,MAV_AUTOPILOT_ARDUPILOTMEGA)&&
        !backend->identityCompatible(MAV_TYPE_GROUND_ROVER,MAV_AUTOPILOT_ARDUPILOTMEGA)&&
        backend->helmProfile().dimensions==NavigationDimensions::Planar,"boat backend not bound");
    });
    test("two stock scalars acquire without home vertical or flight state",[&]{
      Rig r(path);r.acquire();require(r.services.acquires==1&&r.services.targets.size()==1,"2D helm notactive");const auto n=r.services.targets.back();
      require(n.dimensions==NavigationDimensions::Planar&&n.kind==NavigationKind::CourseSpeed&&n.course==90&&n.speed==1,"wrong 2D intent");
      require(r.services.has("MAVMOOS_HELM_STATUS","state=ACTIVE"),"noactive status");
    });
    test("unrequested altitude cannot inhibit or renew planar intent",[&]{
      Rig r(path);r.acquire();const auto expires=r.services.targets.back().expires;
      r.number("DESIRED_ALTITUDE",NAN,100.2);r.services.time=100.21;r.host.iterate();
      require(!r.services.has("MAVMOOS_HELM_STATUS","invalid_helm_input"),"altitude inhibited Rover");
      if(r.services.targets.size()>1)require(r.services.targets.back().expires==expires,"altitude renewedintent");
    });
    test("wrong producer and pre-acquisition cache cannot satisfy intent",[&]{
      Rig r(path);r.fields(100.01);r.text("IVPHELM_ALLSTOP","clear",100.02,"pHelmIvP");r.text("MAVMOOS_NAV_CONTROL","acquire",100.03);
      r.fields(100.04,"other");r.host.iterate();require(r.services.targets.empty(),"old/wrongproducer fields activated");
      r.fields(100.05);r.host.iterate();require(r.services.targets.size()==1,"fresh qualifiedfields denied");
    });
    test("pose expiry latches and fresh fields do not auto restore",[&]{
      Rig r(path);r.acquire();const auto count=r.services.targets.size();r.services.time=100.8;r.host.inputBoundary();
      r.fields(100.81);r.host.iterate();require(r.services.targets.size()==count&&r.services.has("MAVMOOS_HELM_STATUS","state=LATCHED"),"pose lapse auto restored");
      r.text("MAVMOOS_NAV_CONTROL","acquire",100.82);r.fields(100.83);r.host.iterate();require(r.services.targets.size()==count+1,"fresh explicitrecovery denied");
    });
    test("pilot manual takeover latches without Guided recapture",[&]{
      Rig r(path);r.acquire();const auto count=r.services.targets.size();r.services.time=100.2;r.services.state.mode=0;r.host.inputBoundary();r.host.iterate();
      r.fields(100.3);r.host.iterate();require(r.services.targets.size()==count&&r.services.acquires==1,"pilot takeover auto recaptured");
    });
    test("bounded normal pause resumes only fresh post-pause scalars",[&]{
      Rig r(path);r.acquire();r.text("IVPHELM_ALLSTOP","NothingToDo",100.2,"pHelmIvP");r.text("IVPHELM_ALLSTOP","clear",100.21,"pHelmIvP");
      r.fields(100.22);r.host.iterate();require(r.services.resumes==1&&r.services.targets.size()==2,"normal 2D handoff did not resume");
    });
    test("native Hold wins against pending normal resume",[&]{
      Rig r(path);r.acquire();r.text("IVPHELM_ALLSTOP","NothingToDo",100.2,"pHelmIvP");
      r.text("MAVMOOS_COMMAND","id=hold,action=hold",100.21);r.text("IVPHELM_ALLSTOP","clear",100.22,"pHelmIvP");r.fields(100.23);r.host.iterate();
      require(r.services.commands.size()==1&&r.services.commands[0].params[1]==4&&r.services.resumes==0,"native Hold lost to resume");
    });
    test("reset cancels old native operation and latches helm",[&]{
      Rig r(path);r.acquire();r.text("MAVMOOS_COMMAND","id=pending,action=hold",100.2);r.host.iterate();r.services.time=100.21;r.host.reset("vehicle_connection_lost");
      r.fields(100.3);r.host.iterate();require(r.services.has("MAVMOOS_COMMAND_RESULT","status=CANCELLED")&&r.services.has("MAVMOOS_HELM_STATUS","state=LATCHED"),"reset not composed");
    });
    test("Rover confirmation requires exact accepted handoff and fresh mode then pose",[&]{
      Rig r(path);const std::string key="MAVMOOS_COMMAND_RESULT";
      r.text("MAVMOOS_COMMAND","id=local,action=hold",100.1);r.host.iterate();
      require(r.services.commands.size()==1,"Hold did not enter host queue");
      const auto expected=r.services.commands.front();
      auto mismatched=expected;mismatched.params[1]=15;
      r.services.time=100.2;r.host.commandSent(mismatched,100.2,true);
      r.services.state.mode=4;r.services.state.heartbeat_at=r.services.state.position_at=100.2;
      r.host.iterate();
      require(!r.services.has(key,"status=SENT")&&!r.services.has(key,"status=CONFIRMED")&&
              !r.services.has(key,"status=COMPLETED"),"unhanded Hold completed");
      r.services.time=100.21;r.host.commandSent(expected,100.21,true);
      r.host.iterate();
      require(r.services.has(key,"status=SENT")&&!r.services.has(key,"status=CONFIRMED"),
              "pre-handoff state was accepted after the receipt");
      r.services.time=100.24;r.services.state.heartbeat_at=100.24;r.services.state.position_at=100.23;
      r.host.iterate();
      require(r.services.has(key,"status=CONFIRMED")&&!r.services.has(key,"status=COMPLETED"),
              "pose preceding mode confirmation completed Hold");
      r.services.time=100.25;r.services.state.position_at=100.25;r.host.iterate();
      require(r.services.has(key,"status=COMPLETED")&&!r.services.busy,"stopped Hold failed to close");
    });
    test("Rover filters ACK diagnostics by identity and local read window",[&]{
      Rig r(path);const std::string key="MAVMOOS_COMMAND_RESULT";
      r.text("MAVMOOS_COMMAND","id=ack,action=hold",100.1);r.host.iterate();
      r.ack(100.05,100.11);
      require(!r.services.has(key,"status=ACK_OBSERVED"),"pre-handoff ACK was reported");
      r.services.time=100.12;r.host.commandSent(r.services.commands.front(),100.12,true);
      r.ack(100.05,100.12);
      r.ack(100.05,100.13,2);
      r.ack(100.05,100.14,1,2);
      r.ack(100.05,100.15,1,1,246);
      r.ack(100.05,100.16,1,1,245,192);
      require(!r.services.has(key,"status=ACK_OBSERVED"),"foreign or equal-time ACK was reported");
      r.ack(100.05,100.17);
      require(r.services.has(key,"status=ACK_OBSERVED,reason=uncorrelated_ack_result_2")&&r.services.busy&&
              !r.services.has(key,"status=FAILED"),"addressed ACK changed the operation or its diagnostic was lost");
    });
    test("Rover failed local send retires the lease and cannot complete later",[&]{
      Rig r(path);const std::string key="MAVMOOS_COMMAND_RESULT";
      r.text("MAVMOOS_COMMAND","id=failed,action=hold",100.1);r.host.iterate();
      r.services.time=100.11;r.host.commandSent(r.services.commands.front(),100.11,false);
      require(r.services.has(key,"status=FAILED,reason=local_send_rejected_or_failed")&&!r.services.busy,
              "local failure did not retire the command");
      r.services.time=100.2;r.services.state.mode=4;r.services.state.heartbeat_at=r.services.state.position_at=100.2;
      r.host.iterate();
      require(!r.services.has(key,"status=COMPLETED"),"failed local send later claimed an outcome");
    });
    test("Manual observed at native final send retires the packet even after pilot Hold",[&]{
      RoverRuntimeRig rig(path,false);
      rig.observe(rover::guided_mode);
      rig.text("MAVMOOS_COMMAND","id=pre-manual,action=hold");
      bool serialized=false;
      rig.before_send=[&](const mavlink_message_t& packet) {
        if(packet.msgid!=MAVLINK_MSG_ID_COMMAND_LONG)return;
        mavlink_command_long_t command{};
        mavlink_msg_command_long_decode(&packet,&command);
        if(command.command!=MAV_CMD_DO_SET_MODE)return;
        serialized=true;
        rig.clock+=.001;
        rig.heartbeat(rover::manual_mode);
        rig.clock+=.001;
        rig.heartbeat(rover::hold_mode);
      };
      rig.tick();
      require(serialized&&rig.commands.empty()&&rig.result(false,"pre-manual","reason=pilot_manual_takeover"),
              "a queued native mode packet survived pilot Manual at the final gate");
      rig.before_send={};
      rig.observe(rover::hold_mode);
      rig.text("MAVMOOS_COMMAND","id=pre-manual,action=guided");rig.tick();
      require(rig.commands.empty()&&rig.result(false,"pre-manual","status=DUPLICATE"),
              "pilot Hold revived a retired ID");
    });
    test("Runtime routes the encoded Hold send result back to the Rover provider",[&]{
      for(bool accepted:{false,true}) {
        double now=100;
        std::vector<std::string> results;
        unsigned hold_attempts=0;
        Runtime runtime([&](const std::string& key,const std::string& value,double,bool){
          if(key=="MAVMOOS_COMMAND_RESULT")results.push_back(value);
        },[&](const uint8_t* bytes,std::size_t count){
          mavlink_message_t parser_message{},decoded{};
          mavlink_status_t parser_status{},decoded_status{};
          for(std::size_t i=0;i<count;++i)
            if(mavlink_frame_char_buffer(&parser_message,&parser_status,bytes[i],&decoded,&decoded_status)==MAVLINK_FRAMING_OK&&
               decoded.msgid==MAVLINK_MSG_ID_COMMAND_LONG) {
              mavlink_command_long_t command{};mavlink_msg_command_long_decode(&decoded,&command);
              if(command.command==MAV_CMD_DO_SET_MODE&&command.param2==4&&command.target_system==1&&command.target_component==1) {
                ++hold_attempts;return accepted;
              }
            }
          return true;
        },true,false,[&]{return now;},MAV_TYPE_GROUND_ROVER,MAV_AUTOPILOT_ARDUPILOTMEGA);
        require(runtime.load(path),"Runtime could not load Rover provider");
        NativeReplayDirectory replay;
        runtime.configureModules({{"ardurover_control",{{"native_replay_directory",replay.path.string()}}}});
        auto heartbeat=[&](unsigned mode){
          mavlink_message_t message{};
          mavlink_msg_heartbeat_pack(1,1,&message,MAV_TYPE_GROUND_ROVER,MAV_AUTOPILOT_ARDUPILOTMEGA,
                                     MAV_MODE_FLAG_SAFETY_ARMED,mode,MAV_STATE_ACTIVE);
          runtime.message(message,now);
        };
        now=100.01;heartbeat(15);
        now=100.02;
        Mail mail;mail.key="MAVMOOS_COMMAND";mail.text="id=runtime,action=hold";
        mail.received_at=mail.published_at=now;runtime.mail(mail);runtime.tick(now);
        const auto has=[&](const std::string& pattern){
          return std::any_of(results.begin(),results.end(),[&](const auto& row){return row.find(pattern)!=std::string::npos;});
        };
        require(hold_attempts==1&&has(accepted?"status=SENT":"status=FAILED,reason=local_send_rejected_or_failed"),
                "Runtime's local send receipt did not reach Rover");
        now=100.03;heartbeat(4);
        mavlink_message_t position{};
        mavlink_msg_global_position_int_pack(1,1,&position,1000,423584560,-710875890,0,0,0,0,0,9000);
        now=100.04;runtime.message(position,now);runtime.tick(now);
        require(has("status=COMPLETED")==accepted&&!runtime.controlBusy(),
                "Runtime lease or terminal Rover outcome diverged from actual handoff");
      }
    });
    test("boat setup grace stays bound to the original mode and ends on first Guided heartbeat",[&]{
      for(unsigned event:{0u,1u,2u,3u,4u,5u}) {
        double now=100;
        unsigned setup_sends=0;
        Runtime runtime([](const std::string&,const std::string&,double,bool){},
          [&](const uint8_t* bytes,std::size_t count){
            mavlink_message_t parser{},message{};
            mavlink_status_t parser_status{},status{};
            for(std::size_t i=0;i<count;++i)
              if(mavlink_frame_char_buffer(&parser,&parser_status,bytes[i],&message,&status)==MAVLINK_FRAMING_OK&&
                 message.msgid==MAVLINK_MSG_ID_COMMAND_LONG) {
                mavlink_command_long_t command{};mavlink_msg_command_long_decode(&message,&command);
                if(command.command==MAV_CMD_DO_SET_MODE&&command.param2==15) ++setup_sends;
              }
            return true;
          },true,false,[&]{return now;},MAV_TYPE_SURFACE_BOAT,MAV_AUTOPILOT_ARDUPILOTMEGA);
        require(runtime.load(path),"could not load boat provider");
        runtime.configure("vehicle_profile","ardurover_boat");
        auto heartbeat=[&](unsigned mode,bool armed=true){
          now+=.01;
          mavlink_message_t message{};
          mavlink_msg_heartbeat_pack(1,1,&message,MAV_TYPE_SURFACE_BOAT,MAV_AUTOPILOT_ARDUPILOTMEGA,
                                     armed?MAV_MODE_FLAG_SAFETY_ARMED:0,mode,MAV_STATE_ACTIVE);
          runtime.message(message,now);
        };
        heartbeat(event==0?15:event==1?4:5);
        mavlink_message_t message{};
        mavlink_msg_global_position_int_pack(1,1,&message,1000,423584560,-710875890,0,0,0,0,0,9000);
        now+=.01;runtime.message(message,now);
        mavlink_msg_ekf_status_report_pack(1,1,&message,EKF_ATTITUDE|EKF_VELOCITY_HORIZ|EKF_POS_HORIZ_ABS,
                                          .01f,.01f,.01f,.01f,.01f,0);
        now+=.01;runtime.message(message,now);
        require(runtime.acquireNavigation("operator"),"fresh armed boat did not acquire");
        if(event==0) heartbeat(5);                   // Already Guided; there was never setup grace.
        if(event==1) heartbeat(5);                   // A different pre-Guided mode took over.
        if(event==2) {heartbeat(15);heartbeat(5);}   // External mode change before queued setup sent.
        if(event==3) heartbeat(5,false);             // Disarm before setup handoff.
        if(event>=4) {
          if(event==5) {
            require(runtime.acquireNavigation("replacement"),"replacement acquire failed");
            runtime.tick(now);now+=.01;
          }
          runtime.tick(now);heartbeat(15);require(runtime.navigationGranted(),"Guided entry lost its grant");heartbeat(5);
        }
        heartbeat(15);
        now+=.01;runtime.tick(now);
        require(!runtime.navigationGranted()&&setup_sends==(event>=4?1:0),
                "external mode change retained a grant or emitted an old Guided setup");
        bool rejected=false;
        try {runtime.configure("vehicle_profile","ardurover");} catch(const std::runtime_error&) {rejected=true;}
        require(rejected,"runtime allowed a profile change after input");
      }
    });
    test("facets hide unsupported consumers while ownership stays reserved",[&]{
      Rig r(path);r.host.configure("control_native_actions","false");const auto s=r.plugin().subscriptions();require(std::find(s.begin(),s.end(),"MAVMOOS_COMMAND")==s.end(),"disablednative consumer");
      r.host.configure("control_helm","false");require(r.plugin().subscriptions().empty()&&r.plugin().descriptor().capabilities.empty(),"disabledpublic surfaces remain");
      require(r.plugin().descriptor().request_owners==std::vector<std::string>{"MAVMOOS_COMMAND","MAVMOOS_NAV_CONTROL","MAVMOOS_CONTROL_REQUEST"}&&r.plugin().controlBackend(),"disabled facets lost owner/backend");
    });
    test("non-Course Rover facing configuration rejects",[&]{
      Rig r(path);for(const auto& value:{"keep","heading","nonsense"}){bool rejected=false;try{r.host.configure("facing_policy",value);}catch(const std::invalid_argument&){rejected=true;}require(rejected,"unsupportedfacing silentlyaccepted");}
    });
    test("legacy and versioned native variants send once and require observed outcome",[&]{checkNativeVariants(path);});
    test("stock helm, direct operations and pilot takeover share one final send gate",[&]{checkOwnersAndFinalGate(path);});
    test("both profiles send Guided hold and return and require advancing settled pose",[&]{checkDirectSettling(path);});
    std::cout<<"PASS "<<count<<" real Rover provider groups; deterministic software evidence\n";return 0;
  }catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
