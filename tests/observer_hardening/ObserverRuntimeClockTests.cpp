/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ObserverRuntimeClockTests.cpp                    */
/*    DATE: November 4, 2026                                */
/************************************************************/
// Actual Runtime and loaded observers, deterministic fake clocks/I/O only.
#include "Runtime.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
using namespace mavmoos::runtime;
namespace {
void require(bool condition,const std::string& reason){if(!condition)throw std::runtime_error(reason);}
std::string quote(const std::string& value){std::string out="\"";for(char c:value){if(c=='"'||c=='\\')out+='\\';out+=c;}return out+'"';}
const std::vector<std::string> status_keys={"MAVMOOS_ATTITUDE_STATUS","MAVMOOS_SYSTEM_STATUS","MAVMOOS_RC_IO_STATUS"};
const std::vector<std::string> sample_keys={"MAVMOOS_ATTITUDE_EULER","MAVMOOS_SYS_STATUS","MAVMOOS_RC_INPUT"};
uint64_t epoch(const std::string& text){const auto pos=text.find("\"session_epoch\":");require(pos!=std::string::npos,"missing session epoch");return std::stoull(text.substr(pos+16));}
bool field(const std::string& value,const std::string& key,const std::string& expected){return value.find('"'+key+"\":"+expected)!=std::string::npos;}
struct Rig {
  double raw=100;unsigned writes=0;std::map<std::string,std::string> latest;
  std::vector<std::string> modules;Runtime runtime;mavlink_status_t codec{};
  Rig(std::vector<std::string> paths,bool simulation=false):modules(std::move(paths)),runtime(
    [this](const auto& key,const auto& text,double,bool numeric){require(!numeric,"observer published numeric/NAV output");latest[key]=text;},
    [this](const uint8_t*,std::size_t){++writes;return true;},simulation,false,[this]{return raw;})
  {for(const auto& path:modules)require(runtime.load(path),"load actual observer MODULE: "+path);}
  void heartbeat(double at){raw=at;mavlink_message_t m{};mavlink_msg_heartbeat_pack_status(1,1,&codec,&m,2,3,0,4,4);runtime.message(m,at);}
  void samples(double at){raw=at;mavlink_message_t m{};mavlink_attitude_t a{};a.yaw=.4;mavlink_msg_attitude_encode_status(1,1,&codec,&m,&a);runtime.message(m,at);
    mavlink_sys_status_t s{};s.voltage_battery=12300;s.current_battery=-1;s.battery_remaining=-1;mavlink_msg_sys_status_encode_status(1,1,&codec,&m,&s);runtime.message(m,at);
    mavlink_rc_channels_t c{};c.chancount=1;c.chan1_raw=1500;c.rssi=255;mavlink_msg_rc_channels_encode_status(1,1,&codec,&m,&c);runtime.message(m,at);}
  void seed(){heartbeat(100.1);samples(101);runtime.tick(101);for(const auto& key:sample_keys)require(field(latest.at(key),"present","true")&&field(latest.at(key),"fresh","true"),"seed did not establish actual observation: "+key);}
  std::vector<uint64_t> epochs()const{std::vector<uint64_t> out;for(const auto& key:status_keys)out.push_back(epoch(latest.at(key)));return out;}
  void cleared(const std::vector<uint64_t>& before,const std::string& reason){
    require(!runtime.vehicle().heartbeat_valid&&!runtime.navigationGranted(),"host fault retained shared session/authority");
    for(std::size_t i=0;i<sample_keys.size();++i){require(field(latest.at(sample_keys[i]),"present","false")&&field(latest.at(sample_keys[i]),"fresh","false"),"reset retained sample: "+sample_keys[i]);
      const auto& state=latest.at(status_keys[i]);require(epoch(state)>before[i],"reset did not advance observer epoch");require(field(state,"state","\"WAITING_HEARTBEAT\""),"raw host fault became an unrecoverable numeric provider fault");require(field(state,"reset_reason",quote(reason)),"reset reason lost");}
    require(writes==0,"clock/reset fixture emitted MAVLink");}
  void snapshot(const std::string& label)const{std::cout<<"{\"case\":"<<quote(label)<<",\"runtime_now\":"<<runtime.now()<<",\"runtime_status\":"<<quote(runtime.status())<<",\"writes\":"<<writes<<",\"observers\":{";bool comma=false;for(const auto& key:status_keys){if(comma)std::cout<<',';comma=true;std::cout<<quote(key)<<':'<<latest.at(key);}std::cout<<"}}\n";}
};
}
int main(int argc,char** argv){try{
  require(argc==4,"usage: observer_runtime_clock_tests ATTITUDE STATUS RC_IO");const std::vector<std::string> modules={argv[1],argv[2],argv[3]};unsigned groups=0;
  auto test=[&](const char* name,const auto& run){run();++groups;std::cout<<"PASS "<<name<<'\n';};
  test("rejected checkpoint cannot rewind owner time or retain observations",[&]{for(double bad:{100.9,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}){Rig r(modules);r.seed();const auto before=r.epochs();require(!r.runtime.checkpoint(bad)&&r.runtime.now()==101,"bad checkpoint rewound/poisoned Runtime.now");r.raw=101.1;r.runtime.tick(r.raw);r.cleared(before,"clock_fault");r.snapshot("checkpoint_rejected");}});
  test("rejected sampled clock clears caches at retained good time",[&]{for(double bad:{100.9,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}){Rig r(modules);r.seed();const auto before=r.epochs();r.raw=bad;r.runtime.tick(101);require(r.runtime.now()==101,"sampleClock rewound/poisoned owner time");r.raw=101.1;r.runtime.tick(r.raw);r.cleared(before,"clock_fault");r.snapshot("sample_rejected");}});
  test("final handoff predicate cannot pass a regressing or nonfinite clock",[&]{for(double bad:{100.9,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}){Rig r(modules,true);r.heartbeat(100.1);r.samples(101);unsigned proofs=0;std::vector<uint64_t> before;r.runtime.setBoundedSend([&](const uint8_t*,std::size_t,double,const Runtime::SendEligibility& eligible){before=r.epochs();++proofs;require(!eligible(bad),"bad final clock eligible");return false;});r.runtime.tick(101);require(proofs==1&&r.runtime.now()==101,"clock-fault final predicate not exercised exactly once");r.raw=101.1;r.runtime.tick(r.raw);r.cleared(before,"clock_fault");r.snapshot("final_predicate_rejected");}});
  test("explicit lower disconnect boundary retains owner high-water",[&]{Rig r(modules);r.seed();const auto before=r.epochs();r.runtime.disconnect(99,"explicit_test_reset");require(r.runtime.now()==101,"disconnect rewound owner clock");r.raw=101.1;r.runtime.tick(r.raw);r.cleared(before,"explicit_test_reset");r.snapshot("disconnect_retained");});
  test("host fault reason does not prevent valid new heartbeat recovery",[&]{Rig r(modules);r.seed();const auto before=r.epochs();require(!r.runtime.checkpoint(100.9),"fixture bad checkpoint accepted");r.raw=101.1;r.runtime.tick(r.raw);r.cleared(before,"clock_fault");
    r.heartbeat(101.2);r.samples(101.3);r.raw=101.7;r.runtime.tick(r.raw);for(const auto& key:sample_keys)require(field(r.latest.at(key),"fresh","true"),"qualified new observations did not recover");for(const auto& key:status_keys)require(field(r.latest.at(key),"reset_reason","\"clock_fault\"")&&field(r.latest.at(key),"state","\"BOUND\""),"reason/numeric-state distinction lost");require(!r.runtime.navigationGranted()&&r.writes==0,"observation recovery restored control/effects");r.snapshot("new_session_observed");});
  test("retained reset floor rejects old acquisitions despite later read",[&]{Rig r(modules);r.seed();r.runtime.disconnect(101.1,"reset_floor");r.raw=101.2;mavlink_message_t m{};mavlink_msg_heartbeat_pack_status(1,1,&r.codec,&m,2,3,0,4,4);r.runtime.message(m,101.05,101.2);r.samples(101.3);r.runtime.tick(101.3);for(const auto& key:sample_keys)require(field(r.latest.at(key),"present","false"),"old acquisition rebound observer");for(const auto& key:status_keys)require(field(r.latest.at(key),"state","\"WAITING_HEARTBEAT\""),"old floor bound source");require(r.writes==0&&!r.runtime.navigationGranted(),"old floor gained effects/authority");r.snapshot("old_floor_rejected");});
  std::cout<<"PASS "<<groups<<" actual Runtime observer-clock groups; fake I/O only\n";return 0;
}catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}}
