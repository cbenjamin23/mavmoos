/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: CopterOperationTests.cpp                         */
/*    DATE: November 4, 2026                                */
/************************************************************/
#include "CopterOperations.h"
#include <algorithm>
#include <cmath>
#include <dlfcn.h>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>
using namespace mavmoos::runtime;
namespace {
void require(bool value,const std::string& why) {if(!value)throw std::runtime_error(why);}
bool near(double a,double b,double tolerance=1e-5) {return std::abs(a-b)<=tolerance;}
struct Host final:Services {
  double time=100,resume_until=0;Vehicle state;bool grant=false,allow_command=true,allow_navigation=true,projection=true;
  unsigned acquisitions=0;std::vector<CommandRequest> commands,waiting;
  std::vector<NavigationTarget> targets;std::vector<std::pair<std::string,std::string>> publications;
  std::vector<std::string> cancellations,finished,revocations;std::map<std::string,double> leases;
  Host() {fresh();}
  void fresh(bool airborne=true) {
    state.heartbeat_valid=state.position_valid=state.home_valid=state.landed_valid=state.estimator_valid=true;
    state.heartbeat_at=state.position_at=state.landed_at=state.estimator_at=time;
    state.mode=4;state.type=2;state.autopilot=3;state.armed=state.in_air=airborne;
    state.landed_state=airborne?2:1;state.latitude=42;state.longitude=-71;
    state.relative_height=5;state.msl_height=15;state.home_latitude=42;state.home_longitude=-71;state.home_msl=10;
  }
  double now() const override {return time;}
  const Vehicle& vehicle() const override {return state;}
  void publish(const std::string& key,const std::string& value) override {publications.emplace_back(key,value);}
  void publish(const std::string&,double) override {}
  bool sendCommand(const CommandRequest& request) override {
    if(!allow_command||controlBusy())return false;
    commands.push_back(request);waiting.push_back(request);leases[request.id]=request.operation_expires;grant=false;return true;
  }
  bool acquireNavigation(const std::string&) override {++acquisitions;if(controlBusy()||!allow_navigation)return false;grant=true;return true;}
  void revokeNavigation(const std::string& reason) override {grant=false;revocations.push_back(reason);}
  bool navigation(const NavigationTarget& target) override {if(!grant||!allow_navigation)return false;targets.push_back(target);return true;}
  bool navigationGranted() const override {return grant;}
  bool suspendNavigation(double until) override {if(!grant||controlBusy())return false;grant=false;resume_until=until;return true;}
  bool resumeNavigation(const std::string&) override {if(time>=resume_until||controlBusy())return false;resume_until=0;grant=true;return true;}
  bool controlBusy() const override {return std::any_of(leases.begin(),leases.end(),[&](const auto& item){return item.second>time;});}
  void commandFinished(const std::string& id) override {finished.push_back(id);leases.erase(id);}
  void cancelCommand(const std::string& id) override {
    cancellations.push_back(id);waiting.erase(std::remove_if(waiting.begin(),waiting.end(),[&](const auto& r){return r.id==id;}),waiting.end());
  }
  bool projectGlobal(double latitude,double longitude,double& x,double& y) const override {
    if(!projection)return false;x=(longitude+71)*100000;y=(latitude-42)*100000;return true;
  }
  bool found(const std::string& key,const std::string& text) const {
    return std::any_of(publications.begin(),publications.end(),[&](const auto& value){return value.first==key&&value.second.find(text)!=std::string::npos;});
  }
  bool result(const std::string& id,const std::string& status) const {
    return std::any_of(publications.begin(),publications.end(),[&](const auto& value){return value.first=="MAVMOOS_CONTROL_RESULT"&&value.second.find("id="+id+",")!=std::string::npos&&value.second.find("status="+status+",")!=std::string::npos;});
  }
};
struct Rig {
  Host host;void* image=nullptr;DestroyPlugin destroy=nullptr;Plugin* plugin=nullptr;
  explicit Rig(const std::string& path) {
    image=dlopen(path.c_str(),RTLD_NOW|RTLD_LOCAL|RTLD_NODELETE);require(image,"load control image");
    const auto create=reinterpret_cast<CreatePlugin>(dlsym(image,"mavmoos_create_plugin"));
    destroy=reinterpret_cast<DestroyPlugin>(dlsym(image,"mavmoos_destroy_plugin"));require(create&&destroy,"missing factory");
    plugin=create(plugin_abi,&host);require(plugin,"create matching ABI provider");
  }
  ~Rig() {if(plugin)destroy(plugin);if(image)dlclose(image);}
  void step(double at) {host.time=at;plugin->onInputBoundary();plugin->iterate();}
  void handoff(double at) {
    require(!host.waiting.empty(),"no queued command to hand off");
    const auto request=host.waiting.front();host.waiting.erase(host.waiting.begin());
    host.time=at;plugin->onCommandSent(request,at,true);
  }
  void mail(const std::string& text,double at=100.1,double published=-1,const std::string& key="MAVMOOS_CONTROL_REQUEST") {
    host.time=at;Mail message;message.key=key;message.text=text;message.source="operator";message.received_at=at;message.published_at=published<0?at:published;plugin->onMail(message);
  }
  void request(const std::string& text,double at=100.1) {mail(text,at);host.fresh();step(at);}
  void cancel(const std::string& target,double at=100.3,const std::string& id="cancel") {request("schema=1,id="+id+",action=cancel,target="+target,at);}
  void helmText(const std::string& key,const std::string& text,double at) {
    host.time=at;Mail mail;mail.key=key;mail.text=text;mail.source="pHelmIvP";mail.received_at=mail.published_at=at;plugin->onMail(mail);
  }
  void helmFields(double at) {
    host.time=at;host.fresh();helmText("IVPHELM_ALLSTOP","clear",at);helmText("IVPHELM_STATE","DRIVE",at);
    for(const auto& p:std::vector<std::pair<std::string,double>>{{"DESIRED_HEADING",90},{"DESIRED_SPEED",1},{"DESIRED_ALTITUDE",5}}) {
      Mail mail;mail.key=p.first;mail.value=p.second;mail.numeric=true;mail.source="pHelmIvP";mail.received_at=mail.published_at=at;plugin->onMail(mail);
    }
    step(at);
  }
  void ack(uint16_t command,uint8_t result,double at) {
    host.time=at;mavlink_message_t message{};mavlink_status_t codec{};
    mavlink_msg_command_ack_pack_status(1,1,&codec,&message,command,result,0,0,245,191);plugin->onMessage(message,at);
  }
};
mavlink_message_t decode(const ControlFrame& frame) {
  mavlink_message_t output{},buffer{};mavlink_status_t status{},decoded{};unsigned complete=0;
  for(std::size_t i=0;i<frame.size;++i)if(mavlink_frame_char_buffer(&buffer,&status,frame.bytes[i],&output,&decoded)==MAVLINK_FRAMING_OK)++complete;
  require(complete==1,"exactly one valid MAVLink frame");return output;
}
}
int main(int argc,char** argv) {
 try {
  require(argc==2,"usage: copter_operation_tests CONTROL_DSO");const std::string path=argv[1];unsigned passed=0;
  auto test=[&](const std::string& name,const std::function<void()>& run){run();++passed;std::cout<<"PASS "<<name<<'\n';};
  test("new capability surfaces are coherent and helm strategies opt in",[&]{
    Rig r(path);auto d=r.plugin->descriptor();
    require(std::find(d.request_owners.begin(),d.request_owners.end(),"MAVMOOS_CONTROL_REQUEST")!=d.request_owners.end(),"new owner missing");
    auto has=[&](const std::string& n){const auto value=r.plugin->descriptor();return std::any_of(value.capabilities.begin(),value.capabilities.end(),[&](const auto& c){return c.name==n;});};
    require(has("control.operations.native")&&has("control.operations.guided")&&!has("control.operations.helm"),"default strategies");
    r.plugin->configure("operation_helm_enabled","true");require(has("control.operations.helm"),"configured helm absent");
    r.plugin->configure("control_native_actions","false");require(!has("control.operations.native")&&!has("control.operations.guided")&&has("control.operations.helm"),"facets not reflected");
  });
  test("strict schema strategy fields and malformed requests cause no effects",[&]{
    for(const auto& text:std::vector<std::string>{"id=a,action=hold","schema=2,id=a,action=hold","schema=1,id=a,id=b,action=hold","schema=1,id=a,action=goto,strategy=native","schema=1,id=a,action=return,strategy=guided","schema=1,id=a,action=hold,extra=1","schema=1,id=a,action=goto,lat=42,lon=-71,height=nan","schema=1,id=a,action=hold,heading=90","schema=1,id=a,action=goto,lat=42,lon=-71,height=5,speed=2","schema=1,id=a,action=hold,"}) {
      Rig r(path);r.request(text);require(r.host.commands.empty()&&r.host.targets.empty()&&r.host.acquisitions==0,"malformed operation affected control: "+text);require(r.host.found("MAVMOOS_CONTROL_RESULT","status=REJECTED"),"missing rejection");
    }
    Rig oversized(path);oversized.request("schema=1,id=large,action=hold,strategy=guided,"+std::string(512,'x'));
    require(oversized.host.result("invalid","REJECTED")&&oversized.host.acquisitions==0,"oversized request entered field parsing");
  });
  test("retained stale future and numeric requests cannot acquire control",[&]{
    for(const auto& times:std::vector<std::pair<double,double>>{{100.1,100},{103,100.5},{100.1,100.3}}) {
      Rig r(path);r.mail("schema=1,id=bad,action=hold,strategy=guided",times.first,times.second);r.step(times.first);require(r.host.acquisitions==0,"bad publication acquired");
    }
    Rig r(path);Mail mail;mail.key="MAVMOOS_CONTROL_REQUEST";mail.numeric=true;mail.text="schema=1,id=n,action=hold,strategy=guided";mail.published_at=mail.received_at=100.1;r.host.time=100.1;r.plugin->onMail(mail);r.step(100.1);require(r.host.acquisitions==0,"numeric request acquired");
  });
  test("Guided goto emits standard spatial target and preserves original deadline",[&]{
    Rig r(path);r.mail("schema=1,id=goto,action=goto,lat=42.0001,lon=-71,height=6,timeout=1",100.2,100.1);r.host.fresh();r.step(100.3);
    require(r.host.targets.size()==1&&r.host.targets[0].kind==NavigationKind::GlobalPosition,"missing global target");
    require(r.host.targets[0].dimensions==NavigationDimensions::Spatial&&near(r.host.targets[0].latitude,42.0001)&&near(r.host.targets[0].height,6),"target semantics");
    require(r.host.targets[0].expires<=100.8&&r.host.targets[0].expires<=101.1,"local target lifetime");
    r.host.time=101.1;r.host.fresh();r.step(101.1);require(r.host.result("goto","TIMED_OUT")&&!r.host.grant,"original publication deadline renewed");
  });
  test("Guided hold captures once and explicit cancel retires local navigation",[&]{
    Rig r(path);r.request("schema=1,id=hold,action=hold,strategy=guided");
    require(r.host.targets.size()==1&&near(r.host.targets[0].latitude,42)&&near(r.host.targets[0].height,5),"capture current pose");
    r.host.time=100.25;r.host.fresh();r.host.state.latitude=42.0002;r.step(100.25);require(near(r.host.targets.back().latitude,42),"hold follows moving cache");
    r.cancel("hold");const auto n=r.host.targets.size();r.host.time=100.5;r.host.fresh();r.step(100.5);
    require(r.host.result("hold","CANCELLED")&&r.host.result("cancel","COMPLETED")&&!r.host.grant&&r.host.targets.size()==n,"cancel semantics");
  });
  test("vertical adjustment captures current XY and height and observes arrival",[&]{
    Rig r(path);
    r.mail("schema=1,id=vertical,action=adjust_altitude,delta=2,timeout=3",100.2,100.1);
    auto observe=[&](double at,double height) {
      r.host.time=at;r.host.fresh();
      r.host.state.latitude=42.0001;r.host.state.longitude=-71.0002;r.host.state.relative_height=height;
      r.step(at);
    };
    observe(100.3,6.25);
    require(r.host.targets.size()==1,"relative height operation did not submit");
    const auto& target=r.host.targets.front();
    require(target.kind==NavigationKind::GlobalPosition&&target.facing==FacingPolicy::Keep&&
      near(target.latitude,42.0001)&&near(target.longitude,-71.0002)&&near(target.height,8.25)&&
      target.expires<=100.8,"vertical adjustment did not capture current pose without yaw");
    mavlink_status_t codec{};
    const auto frame=r.plugin->controlBackend()->navigationFrame(target,r.host.state,r.host.time,0,codec);
    require(bool(frame),"vertical adjustment failed to encode");
    const auto message=decode(*frame);
    mavlink_set_position_target_global_int_t wire{};
    mavlink_msg_set_position_target_global_int_decode(&message,&wire);
    require(message.msgid==MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT&&
      wire.coordinate_frame==MAV_FRAME_GLOBAL_RELATIVE_ALT&&wire.lat_int==420001000&&
      wire.lon_int==-710002000&&near(wire.alt,8.25)&&(wire.type_mask&1024u),
      "vertical target lost its wire altitude, XY or yaw semantics");
    observe(100.5,7);
    require(near(r.host.targets.back().height,8.25)&&!r.host.result("vertical","COMPLETED"),"moving sample changed the captured target");
    for(double at:{100.7,101.,101.3,101.6})observe(at,8.25);
    require(!r.host.result("vertical","COMPLETED"),"vertical arrival dwell was shortened");
    observe(101.75,8.25);
    require(r.host.result("vertical","COMPLETED")&&!r.host.grant,"observed height and captured XY did not complete");
    require(std::count(r.host.revocations.begin(),r.host.revocations.end(),"operator_release")==2&&
      std::find(r.host.revocations.begin(),r.host.revocations.end(),"operation_COMPLETED")==r.host.revocations.end(),
      "vertical completion revoked twice and could discard the conditional stop");
  });
  test("vertical requests reject invalid or stale input and retain the publication deadline",[&]{
    for(const auto& request:std::vector<std::string>{
        "action=adjust_altitude", "action=adjust_altitude,delta=nan", "action=adjust_altitude,delta=.05",
        "action=adjust_altitude,delta=46", "action=adjust_altitude,delta=-4.1",
        "action=adjust_altitude,delta=1,strategy=helm", "action=adjust_altitude,delta=1,height=6",
        "action=hold,strategy=guided,delta=1"}) {
      Rig r(path);r.request("schema=1,id=bad,"+request);
      require(r.host.result("bad","REJECTED")&&r.host.targets.empty()&&r.host.acquisitions==0,"invalid vertical request affected control: "+request);
    }
    Rig stale(path);stale.mail("schema=1,id=stale,action=adjust_altitude,delta=1",100.7);stale.step(100.7);
    require(stale.host.result("stale","REJECTED")&&stale.host.acquisitions==0,"stale height captured");
    Rig r(path);r.mail("schema=1,id=vertical,action=adjust_altitude,delta=-2,timeout=1",100.2,100.1);
    r.host.time=100.9;r.host.fresh();r.step(100.9);
    require(r.host.targets.size()==1&&near(r.host.targets[0].height,3)&&near(r.host.targets[0].expires,101.1),"negative adjustment or original deadline");
    r.host.time=101.1;r.host.fresh();r.step(101.1);
    require(r.host.result("vertical","TIMED_OUT")&&!r.host.grant&&r.host.targets.size()==1,"vertical operation renewed at expiry");
  });
  test("home visualization uses projection without taking navigation ownership",[&]{
    Rig r(path);r.request("schema=1,id=held,action=hold,strategy=guided");
    const auto acquisitions=r.host.acquisitions;
    const auto revocations=r.host.revocations.size();
    r.request("schema=1,id=mark,action=visualize_home",100.12);
    require(r.host.found("VIEW_MARKER","x=0,y=0,label=MAVMOOS_HOME")&&r.host.result("mark","COMPLETED")&&
      !r.host.result("held","CANCELLED")&&r.host.acquisitions==acquisitions&&
      r.host.revocations.size()==revocations&&r.host.grant&&r.host.commands.empty(),"marker took flight authority");
    r.host.projection=false;r.request("schema=1,id=no-grid,action=visualize_home",100.14);
    require(r.host.result("no-grid","REJECTED")&&r.host.grant,"missing grid affected the active hold");
    Rig unavailable(path);unavailable.mail("schema=1,id=no-home,action=visualize_home");
    unavailable.host.state.home_valid=false;unavailable.step(100.1);
    require(unavailable.host.result("no-home","REJECTED")&&!unavailable.host.found("VIEW_MARKER","label="),"unbound home drawn");
  });
  test("cancel before first iteration prevents operation admission and effects",[&]{
    Rig r(path);r.mail("schema=1,id=queued,action=hold,strategy=guided");r.mail("schema=1,id=cancel,action=cancel,target=queued",100.11);r.step(100.12);
    require(r.host.acquisitions==0&&r.host.commands.empty()&&r.host.targets.empty()&&r.host.result("queued","CANCELLED")&&r.host.result("cancel","COMPLETED"),"queued cancellation sent work");
  });
  test("same goal needs post-start valid pose and one second of observed settling",[&]{
    Rig r(path);r.request("schema=1,id=goal,action=goto,lat=42,lon=-71,height=5");require(!r.host.result("goal","COMPLETED"),"cached pose completed");
    for(double at:{100.3,100.7,101.0}) {r.host.time=at;r.host.fresh();r.step(at);}
    require(!r.host.result("goal","COMPLETED"),"settled too early");
    r.host.time=101.31;r.host.fresh();r.step(101.31);require(r.host.result("goal","COMPLETED")&&!r.host.grant,"post-start goal dwell not complete");
  });
  test("leaving goal region restarts observed completion dwell",[&]{
    Rig r(path);r.request("schema=1,id=goal,action=goto,lat=42,lon=-71,height=5");
    r.host.time=100.3;r.host.fresh();r.step(100.3);r.host.time=100.9;r.host.fresh();r.host.state.latitude=42.001;r.step(100.9);
    r.host.time=101.1;r.host.fresh();r.step(101.1);r.host.time=101.8;r.host.fresh();r.step(101.8);require(!r.host.result("goal","COMPLETED"),"old dwell survived departure");
  });
  test("takeover stale pose home change and revoked permission latch operations",[&]{
    for(unsigned fault=0;fault<4;++fault) {
      Rig r(path);r.request("schema=1,id=active,action=hold,strategy=guided");r.host.time=100.3;r.host.fresh();
      if(fault==0)r.host.state.mode=6;if(fault==1)r.host.state.position_at=99;if(fault==2)r.host.state.home_msl=11;if(fault==3)r.host.grant=false;
      r.step(100.3);const auto n=r.host.targets.size();r.host.time=100.5;r.host.fresh();r.step(100.5);
      require(r.host.result("active","CANCELLED")&&!r.host.grant&&r.host.targets.size()==n,"fault auto-revived operation");
    }
  });
  test("reset cancels active queued requests and retains duplicate history",[&]{
    Rig r(path);r.request("schema=1,id=old,action=hold,strategy=guided");r.host.time=100.3;r.plugin->onReset("vehicle_connection_lost");r.host.fresh();r.step(100.4);require(r.host.result("old","CANCELLED"),"reset outcome");
    r.request("schema=1,id=old,action=hold,strategy=guided",100.5);require(r.host.result("old","DUPLICATE"),"reset forgot ID");
  });
  test("clock rollback latches until reset rather than silently restarting",[&]{
    Rig r(path);r.request("schema=1,id=old,action=hold,strategy=guided");r.host.time=100.2;r.host.fresh();r.step(100.2);r.step(100.15);require(r.host.result("old","CANCELLED"),"clock rollback did not cancel");
    const auto n=r.host.targets.size();r.host.time=100.4;r.host.fresh();r.step(100.4);require(r.host.targets.size()==n&&!r.host.grant,"clock return resumed target");
  });
  test("legacy explicit acquire is forwarded after operation cancellation",[&]{
    Rig r(path);r.request("schema=1,id=old,action=hold,strategy=guided");const auto before=r.host.acquisitions;
    r.mail("acquire",100.3,-1,"MAVMOOS_NAV_CONTROL");require(r.host.result("old","CANCELLED")&&r.host.acquisitions==before+1&&r.host.grant,"internal release hid legacy acquire");
  });
  test("native operation uses legacy command engine and observed outcome",[&]{
    Rig r(path);r.host.fresh(false);r.mail("schema=1,id=arm,action=arm",100.1);r.host.fresh(false);r.step(100.1);
    require(r.host.commands.size()==1&&r.host.commands[0].command==MAV_CMD_COMPONENT_ARM_DISARM,"native arm routing");
    r.handoff(100.11);
    r.ack(MAV_CMD_COMPONENT_ARM_DISARM,MAV_RESULT_DENIED,100.2);
    require(r.host.result("arm","ACK_OBSERVED")&&!r.host.result("arm","FAILED")&&r.host.controlBusy(),
      "uncorrelated denial retired the active arm request");
    r.host.time=100.3;r.host.fresh(false);r.host.state.armed=true;r.step(100.3);require(r.host.result("arm","COMPLETED")&&!r.host.controlBusy(),"fresh observed arm outcome");
  });
  test("native return maps to RTL and requires fresh landed completion",[&]{
    Rig r(path);r.request("schema=1,id=return,action=return");require(r.host.commands.size()==1&&r.host.commands[0].command==MAV_CMD_NAV_RETURN_TO_LAUNCH,"return is not RTL");
    r.handoff(100.11);
    r.host.time=100.3;r.host.fresh();r.host.state.mode=6;r.step(100.3);require(r.host.result("return","CONFIRMED")&&!r.host.result("return","COMPLETED"),"RTL mode is not touchdown");
    r.host.time=100.5;r.host.fresh(false);r.host.state.mode=6;r.step(100.5);require(r.host.result("return","COMPLETED"),"fresh touchdown not complete");
  });
  test("native delegated send and lease cannot outlive caller original deadline",[&]{
    Rig r(path);r.mail("schema=1,id=native,action=hold,timeout=1",100.2,100.1);r.host.fresh();r.step(100.4);
    require(r.host.commands.size()==1&&r.host.commands[0].expires<=101.1&&r.host.commands[0].operation_expires<=101.1,"native deadline renewed");
  });
  test("rejected native policy does not revoke an existing legacy navigation grant",[&]{
    Rig r(path);r.host.grant=true;r.request("schema=1,id=bad-takeoff,action=takeoff,height=5");
    require(r.host.result("bad-takeoff","REJECTED")&&r.host.grant&&r.host.commands.empty(),"rejected native policy preempted valid navigation");
  });
  test("native cancel removes waiting work while preserving uncertain lease",[&]{
    Rig r(path);r.request("schema=1,id=native,action=return,timeout=30");require(r.host.controlBusy(),"missing native lease");r.cancel("native");
    require(r.host.waiting.empty()&&r.host.controlBusy()&&r.host.result("native","CANCELLED"),"cancel prematurely retired lease");
    r.request("schema=1,id=next,action=hold,strategy=guided",100.5);require(r.host.result("next","REJECTED")&&!r.host.grant,"uncertain native effect overlapped navigation");
  });
  test("missing native handoff keeps lease quarantine to original operation deadline",[&]{
    Rig r(path);r.request("schema=1,id=native,action=return,timeout=30");r.host.time=104.2;r.host.fresh();r.step(104.2);
    require(r.host.result("native","TIMED_OUT")&&r.host.controlBusy()&&r.host.finished.empty(),"early helper timeout retired uncertain lease");
    r.mail("acquire",104.3,-1,"MAVMOOS_NAV_CONTROL");require(!r.host.grant,"legacy helm bypassed quarantine");
    r.host.time=130.2;r.host.fresh();r.step(130.2);require(!r.host.controlBusy()&&!r.host.finished.empty(),"lease never retired at original bound");
  });
  test("shorter native lifetime terminates a still-unobserved public operation",[&]{
    Rig r(path);r.request("schema=1,id=native,action=hold,timeout=30");
    r.handoff(100.11);
    r.ack(MAV_CMD_DO_SET_MODE,MAV_RESULT_ACCEPTED,100.2);
    r.host.time=108.11;r.host.fresh();r.step(108.11);
    require(r.host.result("native","TIMED_OUT")&&!r.host.controlBusy(),"native deadline retired its correlation ID before terminal outcome");
    r.request("schema=1,id=next,action=hold,strategy=guided",108.3);
    require(r.host.grant,"expired native operation remained busy until the longer public upper bound");
  });
  test("legacy command admission cancels Guided operation without a second owner",[&]{
    Rig r(path);r.request("schema=1,id=old,action=hold,strategy=guided");r.mail("id=legacy,action=hold",100.3,-1,"MAVMOOS_COMMAND");r.host.fresh();r.step(100.3);
    require(r.host.result("old","CANCELLED")&&r.host.commands.size()==1&&!r.host.grant,"legacy native admission did not supersede guidance");
  });
  test("helm strategy requires explicit binding and configured projection",[&]{
    Rig r(path);r.request("schema=1,id=no,action=goto,strategy=helm,lat=42.0001,lon=-71,height=5");require(r.host.result("no","REJECTED")&&r.host.acquisitions==0,"unconfigured helm acquired");
    r.plugin->configure("operation_helm_enabled","true");r.host.projection=false;r.request("schema=1,id=projection,action=goto,strategy=helm,lat=42.0001,lon=-71,height=5",100.3);require(r.host.result("projection","REJECTED")&&r.host.acquisitions==0,"missing projection acquired");
  });
  test("helm goto emits ordinary behavior updates and accepts post-acquisition stock inputs",[&]{
    Rig r(path);r.plugin->configure("operation_helm_enabled","true");r.request("schema=1,id=helm,action=goto,strategy=helm,lat=42.0001,lon=-71,height=5,speed=1");
    require(r.host.grant&&r.host.found("MAVMOOS_OPERATION_HOLD_UPDATE","station_pt=x=")&&r.host.found("MAVMOOS_HELM_OPERATION","route"),"ordinary helm binding not emitted");
    require(r.host.targets.empty(),"cached helm fields leaked");r.helmFields(100.2);require(r.host.targets.size()==1&&r.host.targets[0].kind==NavigationKind::CourseSpeed,"stock helm target not used");
    r.step(100.21);require(r.host.result("helm","CONFIRMED"),"current eligible helm not confirmed");
  });
  test("operator adjustments update stock behavior without replacing helm authority",[&]{
    Rig r(path);r.plugin->configure("operation_helm_enabled","true");
    r.plugin->configure("operation_helm_adjustments","true");r.plugin->configure("command_speed_min","0.4");
    r.mail("acquire",100.1,-1,"MAVMOOS_NAV_CONTROL");r.helmFields(100.2);
    r.request("schema=1,id=adjust-speed,action=adjust_speed,strategy=helm,delta=.25",100.3);
    require(r.host.result("adjust-speed","ACCEPTED")&&r.host.found("MAVMOOS_HELM_SPEED_UPDATE","speed=1.250000"),"stock speed update missing");
    r.request("schema=1,id=adjust-height,action=adjust_altitude,strategy=helm,delta=1",100.4);
    require(r.host.result("adjust-height","ACCEPTED")&&r.host.found("MAVMOOS_OPERATION_ALTITUDE_UPDATE","altitude=6.000000"),"stock altitude update missing");
    r.request("schema=1,id=minimum,action=reset_speed_min,strategy=helm",100.5);
    require(r.host.result("minimum","ACCEPTED")&&r.host.found("MAVMOOS_HELM_SPEED_UPDATE","speed=0.400000")&&
      r.host.grant&&r.host.acquisitions==1&&r.host.commands.empty(),"stock adjustment changed ownership or skipped configured minimum");
    r.request("schema=1,id=guided,action=adjust_altitude,strategy=guided,delta=1",100.55);
    require(r.host.result("guided","REJECTED")&&r.host.grant&&r.host.acquisitions==1,"guided altitude change preempted helm");
    r.request("schema=1,id=stale,action=adjust_altitude,strategy=helm,delta=1",100.8);
    require(r.host.result("stale","REJECTED"),"stale demand was used to make a stock behavior update");
  });
  test("survey and Voronoi require startup opt-in, explicit strategy and a prior helm grant",[&]{
    Rig r(path);r.plugin->configure("operation_helm_enabled","true");
    for(const auto& value:{"", "survey,survey", "survey,", "survey,goto"}) {
      bool rejected=false;
      try {r.plugin->configure("operation_helm_tasks",value);} catch(const std::invalid_argument&) {rejected=true;}
      require(rejected,"invalid task list accepted");
    }
    r.plugin->configure("operation_helm_tasks","survey");
    const auto d=r.plugin->descriptor();
    auto has=[&](const std::string& name){return std::any_of(d.capabilities.begin(),d.capabilities.end(),[&](const auto& cap){return cap.name==name;});};
    require(has("control.operations.helm.survey")&&!has("control.operations.helm.voronoi"),"configured task selection missing from catalog");
    r.request("schema=1,id=none,action=survey,strategy=helm");
    r.request("schema=1,id=disabled,action=voronoi,strategy=helm",100.2);
    r.request("schema=1,id=implicit,action=survey",100.3);
    r.request("schema=1,id=extra,action=survey,strategy=helm,height=6",100.4);
    require(r.host.acquisitions==0&&r.host.targets.empty()&&r.host.result("none","REJECTED")&&
      r.host.result("disabled","REJECTED")&&r.host.result("implicit","REJECTED")&&r.host.result("extra","REJECTED"),"task activated without opt-in or consent");
  });
  test("task switch clears the old helm output and clamps new targets to its original expiry",[&]{
    Rig r(path);r.plugin->configure("operation_helm_enabled","true");r.plugin->configure("operation_helm_tasks","survey, voronoi");
    r.mail("acquire",100.1,-1,"MAVMOOS_NAV_CONTROL");r.helmFields(100.2);
    require(r.host.targets.size()==1,"initial helm target unavailable");
    r.mail("schema=1,id=survey,action=survey,strategy=helm,timeout=2",100.3,100.25);
    r.host.fresh();r.step(100.3);
    require(r.host.found("MAVMOOS_HELM_OPERATION","survey")&&r.host.result("survey","ADMITTED")&&r.host.targets.size()==1&&
      !r.host.result("survey","CONFIRMED"),"old helm inputs crossed the task boundary");
    for(double at:{100.4,100.8,101.2,101.6,102.0})r.helmFields(at);
    require(r.host.result("survey","CONFIRMED")&&!r.host.result("survey","COMPLETED")&&r.host.targets.size()>1&&
      near(r.host.targets.back().expires,102.25),"selected task failed fresh input, result or deadline semantics");
    const auto before=r.host.targets.size();
    r.host.time=102.25;r.host.fresh();r.step(102.25);
    require(r.host.result("survey","TIMED_OUT")&&!r.host.grant&&r.host.targets.size()==before&&
      r.host.found("MAVMOOS_HELM_OPERATION","none"),"expired task left a target or selector live");
  });
  test("task requires fresh consent boundary and cancels after a pilot mode change",[&]{
    Rig r(path);r.plugin->configure("operation_helm_enabled","true");r.plugin->configure("operation_helm_tasks","voronoi");
    r.mail("acquire",100.1,-1,"MAVMOOS_NAV_CONTROL");r.host.fresh();r.step(100.1);
    r.request("schema=1,id=old,action=voronoi,strategy=helm",100.1);
    require(r.host.result("old","REJECTED")&&r.host.grant&&!r.host.found("MAVMOOS_HELM_OPERATION","voronoi"),"same-boundary request reused consent");
    r.request("schema=1,id=task,action=voronoi,strategy=helm",100.2);
    require(r.host.result("task","ADMITTED")&&r.host.grant,"configured idle helm did not select task");
    r.host.time=100.3;r.host.fresh();r.host.state.mode=5;r.step(100.3);
    require(r.host.result("task","CANCELLED")&&!r.host.grant&&r.host.commands.empty()&&
      r.host.found("MAVMOOS_HELM_OPERATION","none"),"task fought mode takeover");
    r.helmFields(100.4);require(r.host.targets.empty()&&!r.host.grant,"fresh fields silently reactivated task");
  });
  test("explicit cancellation clears the selected task and its helm permission",[&]{
    Rig r(path);r.plugin->configure("operation_helm_enabled","true");r.plugin->configure("operation_helm_tasks","survey");
    r.mail("acquire",100.1,-1,"MAVMOOS_NAV_CONTROL");r.helmFields(100.2);
    r.request("schema=1,id=task,action=survey,strategy=helm",100.3);r.helmFields(100.4);
    const auto count=r.host.targets.size();r.cancel("task",100.5);r.helmFields(100.6);
    require(r.host.result("task","CANCELLED")&&r.host.result("cancel","COMPLETED")&&!r.host.grant&&
      r.host.targets.size()==count&&r.host.found("MAVMOOS_HELM_OPERATION","none"),"cancelled task retained authority");
  });
  test("helm hold and return share configured operation provider and use distinct goals",[&]{
    Rig r(path);r.plugin->configure("operation_helm_enabled","true");r.request("schema=1,id=hold,action=hold,strategy=helm");require(r.host.found("MAVMOOS_OPERATION_HOLD_UPDATE","station_pt=x="),"hold update missing");r.cancel("hold");
    r.host.time=100.5;r.host.fresh();r.host.state.latitude=42.0001;r.mail("schema=1,id=home,action=return,strategy=helm,height=5",100.5);r.step(100.5);
    require(r.host.found("MAVMOOS_OPERATION_HOLD_UPDATE","station_pt=x=0,y=0,"),"return does not use bound home");
  });
  test("normal handoff observes current adapter phase before judging revoked permission",[&]{
    Rig r(path);r.plugin->configure("operation_helm_enabled","true");r.request("schema=1,id=helm,action=goto,strategy=helm,lat=42.0001,lon=-71,height=5");r.helmFields(100.2);
    require(r.host.result("helm","CONFIRMED")&&r.host.grant,"helm did not become active");
    r.host.time=100.3;r.host.fresh();r.helmText("IVPHELM_ALLSTOP","NothingToDo",100.3);
    require(!r.host.grant,"normal handoff did not suspend host permission");r.step(100.31);
    require(!r.host.result("helm","CANCELLED")&&r.host.found("MAVMOOS_HELM_STATUS","state=PAUSED"),"cached ACTIVE misclassified current suspension");
    r.helmFields(100.4);require(r.host.grant&&!r.host.result("helm","CANCELLED"),"normal bounded handoff did not resume same operation");
  });
  test("expired handoff and missing helm decision variables still latch the operation",[&]{
    for(const auto& allstop:std::vector<std::string>{"NothingToDo","MissingDecVars:speed,course"}) {
      Rig r(path);r.plugin->configure("operation_helm_enabled","true");r.request("schema=1,id=helm,action=goto,strategy=helm,lat=42.0001,lon=-71,height=5");r.helmFields(100.2);
      r.host.time=100.3;r.host.fresh();r.helmText("IVPHELM_ALLSTOP",allstop,100.3);r.step(100.31);
      if(allstop=="NothingToDo") {r.host.time=100.9;r.host.fresh();r.step(100.9);}
      require(r.host.result("helm","CANCELLED")&&!r.host.grant,"unsafe or expired all-stop did not latch");
      require(r.host.found("MAVMOOS_CONTROL_RESULT","reason=helm_latched"),"comma-bearing internal diagnostics hid LATCHED state");
    }
  });
  test("helm route uses fixed StationKeep transit and preserves speed radius and observed dwell",[&]{
    Rig r(path);r.plugin->configure("operation_helm_enabled","true");r.request("schema=1,id=helm,action=goto,strategy=helm,lat=42.0001,lon=-71,height=5");
    require(r.host.found("MAVMOOS_OPERATION_HOLD_UPDATE","#inner_radius=0.5#outer_radius=2#outer_speed=1#transit_speed=1.5"),"point-goal profile missing speed/radius semantics");
    r.helmFields(100.2);r.host.time=100.3;r.host.fresh();r.host.state.latitude=42.0001;r.step(100.3);
    require(r.host.found("MAVMOOS_HELM_OPERATION","route")&&!r.host.result("helm","COMPLETED"),"point-goal operation bypassed observed dwell");
  });
  test("disabled public facets stay inert and reserve new request ownership",[&]{
    Rig r(path);r.plugin->configure("control_native_actions","false");r.plugin->configure("control_helm","false");r.host.publications.clear();r.request("schema=1,id=none,action=hold,strategy=guided");
    require(r.host.publications.empty()&&r.host.targets.empty()&&r.host.commands.empty()&&r.plugin->subscriptions().empty(),"disabled operation facet ran");
    require(r.plugin->descriptor().request_owners.size()==4,"disabled owner reservation lost");
  });
  test("host busy gate rejects independent operations before any strategy activation",[&]{
    Rig r(path);r.host.leases["known-native-transition"]=105;r.request("schema=1,id=busy,action=hold,strategy=guided");
    require(r.host.result("busy","REJECTED")&&r.host.acquisitions==0&&r.host.targets.empty(),"busy host was bypassed");
  });
  test("history remains bounded without disabling cancellation of the last active operation",[&]{
    Rig r(path);
    for(unsigned i=0;i<127;++i)r.request("schema=1,id=rejected"+std::to_string(i)+",action=arm",100.1+.02*i);
    r.request("schema=1,id=last,action=hold,strategy=guided",103);
    require(r.host.grant,"last bounded request did not start");r.cancel("last",103.2,"cancel-last");
    require(r.host.result("last","CANCELLED")&&r.host.result("cancel-last","COMPLETED")&&!r.host.grant,"full history blocked cancellation");
    r.request("schema=1,id=over-capacity,action=hold,strategy=guided",103.4);require(r.host.result("over-capacity","REJECTED"),"history silently discarded old IDs");
  });
  test("held position is active until explicit cancel or its original deadline",[&]{
    Rig r(path);r.request("schema=1,id=held,action=hold,strategy=guided,timeout=3");
    for(double at:{100.3,100.7,101.0,101.4,101.8,102.2,102.6}) {r.host.time=at;r.host.fresh();r.step(at);}
    require(!r.host.result("held","COMPLETED")&&r.host.grant,"hold was treated as a completed go-to");
    r.host.time=103.11;r.host.fresh();r.step(103.11);require(r.host.result("held","TIMED_OUT")&&!r.host.grant,"hold renewed its operation lifetime");
  });
  test("backend emits standard global position targets and rejects planar Copter goals",[&]{
    Host h;auto backend=makeArduCopterBackend();NavigationTarget t;t.kind=NavigationKind::GlobalPosition;t.latitude=42.0001;t.longitude=-71.0002;t.height=7;t.expires=100.4;t.facing=FacingPolicy::Keep;
    mavlink_status_t codec{};auto frame=backend->navigationFrame(t,h.state,100,0,codec);require(bool(frame),"position encode failed");auto message=decode(*frame);require(message.msgid==MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT,"custom/non-global wire packet");
    mavlink_set_position_target_global_int_t p{};mavlink_msg_set_position_target_global_int_decode(&message,&p);require(p.coordinate_frame==MAV_FRAME_GLOBAL_RELATIVE_ALT&&p.lat_int==420001000&&p.lon_int==-710002000&&near(p.alt,7)&&p.type_mask==3576,"global position units/reference/mask");
    t.dimensions=NavigationDimensions::Planar;require(!backend->navigationInputAllowed(t)&&!backend->navigationFrame(t,h.state,100,0,codec),"planar target accepted by Copter");
  });
  test("position facing choices preserve FC yaw policy or send explicit bounded heading",[&]{
    Host h;auto backend=makeArduCopterBackend();NavigationTarget t;t.kind=NavigationKind::GlobalPosition;t.latitude=42;t.longitude=-70.9999;t.height=5;t.expires=100.4;mavlink_status_t codec{};
    for(const auto policy:{FacingPolicy::Course,FacingPolicy::Keep,FacingPolicy::Heading}) {
      t.facing=policy;t.facing_heading=450;auto frame=backend->navigationFrame(t,h.state,100,0,codec);require(bool(frame),"facing encode");auto message=decode(*frame);mavlink_set_position_target_global_int_t p{};mavlink_msg_set_position_target_global_int_decode(&message,&p);
      if(policy==FacingPolicy::Keep)require((p.type_mask&1024)!=0,"keep commands yaw");else require((p.type_mask&1024)==0&&near(p.yaw,3.14159265358979323846/2),"course/explicit yaw mapping");
    }
    t.facing=FacingPolicy::Heading;t.facing_heading=std::numeric_limits<double>::quiet_NaN();require(!backend->navigationInputAllowed(t),"nonfinite facing accepted");
  });
  test("position bearing uses short dateline arc and same-point Course omits yaw",[&]{
    Host h;auto backend=makeArduCopterBackend();h.state.latitude=0;h.state.longitude=179.9999;
    NavigationTarget t;t.kind=NavigationKind::GlobalPosition;t.latitude=0;t.longitude=-179.9999;t.height=5;t.expires=100.4;mavlink_status_t codec{};
    auto frame=backend->navigationFrame(t,h.state,100,0,codec);require(bool(frame),"dateline goal rejected");auto message=decode(*frame);mavlink_set_position_target_global_int_t p{};mavlink_msg_set_position_target_global_int_decode(&message,&p);
    require(!(p.type_mask&1024)&&near(p.yaw,3.14159265358979323846/2),"course faced long dateline arc");
    t.longitude=h.state.longitude;frame=backend->navigationFrame(t,h.state,100,0,codec);require(bool(frame),"same-point target rejected");message=decode(*frame);mavlink_msg_set_position_target_global_int_decode(&message,&p);require(p.type_mask&1024,"same-point Course imposed yaw");
  });
  test("position builder rejects expired invalid enum nonfinite and out-of-range goals",[&]{
    Host h;auto backend=makeArduCopterBackend();NavigationTarget base;base.kind=NavigationKind::GlobalPosition;base.latitude=42;base.longitude=-71;base.height=5;base.expires=100.4;mavlink_status_t codec{};
    for(unsigned fault=0;fault<7;++fault) {
      auto t=base;if(fault==0)t.expires=100;if(fault==1)t.latitude=91;if(fault==2)t.longitude=181;if(fault==3)t.height=51;if(fault==4)t.kind=static_cast<NavigationKind>(99);if(fault==5)t.facing=static_cast<FacingPolicy>(99);if(fault==6)t.latitude=std::numeric_limits<double>::infinity();
      require(!backend->navigationFrame(t,h.state,100,0,codec),"invalid position goal emitted");
    }
  });
  test("course-speed Keep and Heading alter yaw only while preserving motion conversion",[&]{
    Host h;auto backend=makeArduCopterBackend();NavigationTarget t;t.course=90;t.speed=2;t.height=6;t.expires=100.4;mavlink_status_t codec{};
    for(const auto policy:{FacingPolicy::Course,FacingPolicy::Keep,FacingPolicy::Heading}) {
      t.facing=policy;t.facing_heading=180;auto frame=backend->navigationFrame(t,h.state,100,0,codec);require(bool(frame),"velocity encode");auto message=decode(*frame);mavlink_set_position_target_local_ned_t p{};mavlink_msg_set_position_target_local_ned_decode(&message,&p);
      require(near(p.vx,0)&&near(p.vy,2)&&near(p.vz,-.5),"yaw choice changed velocity");if(policy==FacingPolicy::Keep)require(p.type_mask&1024,"keep yaw active");if(policy==FacingPolicy::Heading)require(!(p.type_mask&1024)&&near(p.yaw,3.14159265358979323846),"explicit velocity yaw");
    }
  });
  std::cout<<"PASS "<<passed<<" Copter operation/backend groups; fake host and real DSO only\n";return 0;
 }catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
