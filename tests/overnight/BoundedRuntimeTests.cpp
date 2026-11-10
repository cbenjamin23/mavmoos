#include "Runtime.h"
#include <functional>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace mavmoos::runtime;
namespace {
void require(bool b,const char* why){if(!b)throw std::runtime_error(why);}
struct Rig {
 double clock=100;std::vector<mavlink_message_t> sent;Runtime host;
 explicit Rig(const std::string& path):host([](const auto&,const auto&,double,bool){},[&](const uint8_t* p,std::size_t n){capture(p,n);return true;},true,false,[&]{return clock;}){
  require(host.load(path),"load control provider");host.configure("control_native_actions","false");host.configure("control_helm","false");fresh();host.tick(clock);sent.clear();clock=100.1;fresh();
 }
 void capture(const uint8_t* p,std::size_t n){mavlink_message_t b{},m{};mavlink_status_t s{},r{};for(std::size_t i=0;i<n;++i)if(mavlink_frame_char_buffer(&b,&s,p[i],&m,&r)==MAVLINK_FRAMING_OK)sent.push_back(m);}
 void fresh(unsigned mode=4,bool airborne=true){mavlink_message_t m{};mavlink_status_t c{};
  mavlink_msg_heartbeat_pack_status(1,1,&c,&m,2,3,airborne?128:0,mode,4);host.message(m,clock);
  mavlink_msg_global_position_int_pack_status(1,1,&c,&m,0,423584560,-710875890,15000,5000,0,0,0,9000);host.message(m,clock);
  mavlink_msg_extended_sys_state_pack_status(1,1,&c,&m,0,airborne?2:1);host.message(m,clock);
  mavlink_home_position_t home{};home.latitude=423584560;home.longitude=-710875890;home.altitude=10000;mavlink_msg_home_position_encode_status(1,1,&c,&m,&home);host.message(m,clock);
  mavlink_ekf_status_report_t e{};e.flags=51;mavlink_msg_ekf_status_report_encode_status(1,1,&c,&m,&e);host.message(m,clock);
 }
 void bounded(std::function<void()> proof){host.setBoundedSend([&,proof](const uint8_t* p,std::size_t n,double expires,const Runtime::SendEligibility& eligible){proof();require(expires>0,"missing original deadline");if(clock>=expires||!eligible(clock))return false;capture(p,n);return true;});}
 unsigned count(unsigned id)const{unsigned n=0;for(auto& m:sent)if(m.msgid==id)++n;return n;}
 unsigned command(unsigned id)const{unsigned n=0;for(const auto& m:sent)if(m.msgid==76){mavlink_command_long_t c{};mavlink_msg_command_long_decode(&m,&c);if(c.command==id)++n;}return n;}
};
MissionRequest mission(){MissionRequest r;r.id="phase";r.operation_id="op";r.action=MissionAction::Clear;r.expires=101;r.operation_expires=110;return r;}
}
int main(int argc,char** argv){try{require(argc==2,"usage: bounded_runtime_tests CONTROL_MODULE");unsigned groups=0;auto test=[&](const char* name,const auto& run){run();++groups;std::cout<<"PASS "<<name<<'\n';};const std::string control=argv[1];
 test("navigation expires during endpoint proof",[&]{Rig r(control);require(r.host.acquireNavigation("nav"),"grant");require(r.host.navigation({90,1,5,100.5}),"target");r.bounded([&]{r.clock=100.6;});r.host.tick(100.1);require(r.count(84)==0,"expired socket-boundary NAV sent");});
 test("fresh target still requires fresh vehicle state after proof",[&]{Rig r(control);require(r.host.acquireNavigation("nav"),"grant");require(r.host.navigation({90,1,5,101}),"target");r.bounded([&]{r.clock=100.7;});r.host.tick(100.1);require(r.count(84)==0,"stale pose socket-boundary NAV sent");});
 test("new grant cannot validate old frame during proof",[&]{Rig r(control);require(r.host.acquireNavigation("old"),"grant");require(r.host.navigation({90,1,5,101}),"target");r.bounded([&]{r.host.revokeNavigation("proof");require(r.host.acquireNavigation("new"),"new grant");});r.host.tick(100.1);require(r.count(84)==0&&r.host.navigationGranted(),"old frame crossed fresh grant");});
 test("dequeued native cancellation is rechecked after proof",[&]{Rig r(control);r.fresh(4,false);CommandRequest c;c.id="arm";c.command=400;c.params[0]=1;c.expires=101;c.operation_expires=105;require(r.host.sendCommand(c),"arm admission");r.bounded([&]{r.host.cancelCommand("arm");});r.host.tick(100.1);require(r.command(400)==0&&r.host.controlBusy(),"cancelled ARM sent or lease erased");});
 test("acquisition permission is rechecked after proof",[&]{Rig r(control);r.fresh(5);require(r.host.acquireNavigation("setup"),"acquire");r.bounded([&]{r.host.revokeNavigation("proof");});r.host.tick(100.1);require(r.command(176)==0,"revoked acquisition sent");});
 test("retired mission mutation is rechecked after proof",[&]{Rig r(control);r.fresh(4,false);require(r.host.mission(mission()),"clear admission");r.bounded([&]{r.host.missionFinished("op");});r.host.tick(100.1);require(r.count(45)==0,"retired CLEAR sent");});
 test("reset plus fresh telemetry cannot revive old parameter work",[&]{for(auto action:{ParameterAction::Read,ParameterAction::Write}){Rig r(control);r.fresh(4,false);ParameterRequest p;p.id="param";p.name="SIM_WIND_SPD";p.action=action;p.value=.25;p.expires=101;require(r.host.parameter(p),"parameter admission");r.bounded([&]{r.clock=100.2;r.host.disconnect(r.clock);r.fresh(4,false);});r.host.tick(100.1);require(r.count(20)==0&&r.count(23)==0,"old parameter crossed new connection");}});
 test("independent read survives ordinary navigation revocation",[&]{Rig r(control);require(r.host.acquireNavigation("nav"),"grant");ParameterRequest p;p.id="read";p.name="SIM_WIND_SPD";p.expires=101;require(r.host.parameter(p),"read");r.bounded([&]{r.host.revokeNavigation("operator");});r.host.tick(100.1);require(r.count(20)==1&&!r.host.navigationGranted(),"independent parameter read incorrectly fenced");});
 test("fresh standard frame remains accepted",[&]{Rig r(control);require(r.host.acquireNavigation("nav"),"grant");require(r.host.navigation({90,1,5,101}),"target");r.bounded([]{});r.host.tick(100.1);require(r.count(84)==1,"fresh frame failed final boundary");});
 std::cout<<"PASS "<<groups<<" bounded host handoff groups; real backend, fake proof/I/O\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
