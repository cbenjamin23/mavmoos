// Real MODULE/factory, fake Services; no sockets, MOOS or native vehicle.
#include "Plugin.h"
#include <cfloat>
#include <cmath>
#include <dlfcn.h>
#include <iostream>
#include <stdexcept>
using namespace mavmoos::runtime;
namespace {
constexpr double pi=3.14159265358979323846;
void require(bool value,const char* text){if(!value)throw std::runtime_error(text);}
struct Host final:Services {
 double time=100;Vehicle state;unsigned effects=0;std::vector<std::pair<std::string,std::string>> output;
 Host(){state.type=2;state.autopilot=3;state.heartbeat_valid=true;state.heartbeat_at=time;}
 double now()const override{return time;}const Vehicle& vehicle()const override{return state;}
 void publish(const std::string& key,const std::string& value)override{require(key.rfind("MAVMOOS_ATTITUDE_",0)==0,"observer wrote another capability variable");output.emplace_back(key,value);}
 void publish(const std::string&,double)override{throw std::runtime_error("unexpected numeric/NAV projection");}
 bool sendCommand(const CommandRequest&)override{++effects;return false;}
 bool acquireNavigation(const std::string&)override{++effects;return false;}
 void revokeNavigation(const std::string&)override{++effects;}
 bool navigation(const NavigationTarget&)override{++effects;return false;}
 bool parameter(const ParameterRequest&)override{++effects;return false;}
 bool mission(const MissionRequest&)override{++effects;return false;}
 void missionFinished(const std::string&)override{++effects;}
 bool suspendNavigation(double)override{++effects;return false;}
 bool resumeNavigation(const std::string&)override{++effects;return false;}
 void commandFinished(const std::string&)override{++effects;}
 void cancelCommand(const std::string&)override{++effects;}
};
struct Rig {
 Host host;void* handle=nullptr;Plugin* plugin=nullptr;DestroyPlugin destroy=nullptr;mavlink_status_t codec{};
 explicit Rig(const char* path){handle=dlopen(path,RTLD_NOW|RTLD_LOCAL|RTLD_NODELETE);require(handle,"dlopen");auto create=reinterpret_cast<CreatePlugin>(dlsym(handle,"mavmoos_create_plugin"));destroy=reinterpret_cast<DestroyPlugin>(dlsym(handle,"mavmoos_destroy_plugin"));require(create&&destroy,"exports");require(!create(plugin_abi-1,&host)&&!create(plugin_abi+1,&host)&&!create(plugin_abi,nullptr),"ABI/context");plugin=create(plugin_abi,&host);require(plugin&&plugin->name()=="attitude","factory identity");require(!host.effects&&host.output.empty(),"factory effect");plugin->configure("attitude_publish_interval",".02");}
 ~Rig(){if(plugin)destroy(plugin);if(handle)dlclose(handle);if(host.effects){std::cerr<<"observer effect\n";std::terminate();}}
 void receive(const mavlink_message_t& m,double effective,double read,const std::string& basis="QUEUE_EMPTY_FLOOR"){host.time=read;plugin->onMessageWithMetadata(m,{effective,read,basis});}
 void heartbeat(double effective,double read,unsigned system=1,unsigned component=1,unsigned type=2,unsigned autopilot=3){mavlink_message_t m{};host.state.heartbeat_valid=true;host.state.heartbeat_at=effective;mavlink_msg_heartbeat_pack_status(system,component,&codec,&m,type,autopilot,0,4,MAV_STATE_ACTIVE);receive(m,effective,read);}
 void bind(){heartbeat(100.1,100.1);}
 void euler(const mavlink_attitude_t& p,double effective,double read){mavlink_message_t m{};mavlink_msg_attitude_encode_status(1,1,&codec,&m,&p);receive(m,effective,read);}
 void quaternion(const mavlink_attitude_quaternion_t& p,double effective,double read){mavlink_message_t m{};mavlink_msg_attitude_quaternion_encode_status(1,1,&codec,&m,&p);receive(m,effective,read);}
 void capture(const char* label){plugin->iterate();for(const auto& item:host.output)std::cout<<"{\"case\":\""<<label<<"\",\"key\":\""<<item.first<<"\",\"value\":"<<item.second<<"}\n";host.output.clear();require(!host.effects,"observer effect");}
};
mavlink_attitude_t euler(double roll=.2,double pitch=-.3,double yaw=-pi/2){mavlink_attitude_t p{};p.time_boot_ms=1234;p.roll=roll;p.pitch=pitch;p.yaw=yaw;p.rollspeed=1.25;p.pitchspeed=-2.5;p.yawspeed=.125;return p;}
mavlink_attitude_quaternion_t quaternion(double roll=0,double pitch=0,double yaw=0){mavlink_attitude_quaternion_t p{};p.time_boot_ms=1235;const double cr=cos(roll/2),sr=sin(roll/2),cp=cos(pitch/2),sp=sin(pitch/2),cy=cos(yaw/2),sy=sin(yaw/2);p.q1=cr*cp*cy+sr*sp*sy;p.q2=sr*cp*cy-cr*sp*sy;p.q3=cr*sp*cy+sr*cp*sy;p.q4=cr*cp*sy-sr*sp*cy;p.rollspeed=1.25;p.pitchspeed=-2.5;p.yawspeed=.125;return p;}
}
int main(int argc,char** argv){try{
 require(argc==2,"usage: attitude_scenario_driver ATTITUDE_MODULE");const char* path=argv[1];
 {Rig r(path);auto d=r.plugin->descriptor();require(d.profile=="ardupilot_attitude_observations"&&d.capabilities.size()==2&&d.request_owners.empty()&&d.exclusive_resources==std::vector<std::string>{"observation.attitude"}&&d.streamed_parameter_bench_compatible&&r.plugin->subscriptions().empty()&&!r.plugin->controlBackend(),"descriptor/effects");require(d.observation_streams.size()==2&&d.observation_streams[0].message_id==30&&d.observation_streams[1].message_id==31&&d.observation_streams[0].interval_us==200000&&d.observation_streams[1].interval_us==200000,"stream declaration");r.capture("absent");}
 {Rig r(path);r.euler(euler(),100.1,100.1);r.capture("unbound");r.bind();r.euler(euler(),100.12,100.3);r.capture("euler");r.host.time=100.9;r.capture("cached");r.heartbeat(101,101);r.host.time=101.2;r.capture("stale_euler");}
 {Rig r(path);r.bind();r.euler(euler(0,0,3*pi/2),100.2,100.2);r.capture("ardupilot_yaw");r.euler(euler(0,0,0),100.3,100.3);r.capture("zero_yaw");r.euler(euler(0,0,pi),100.4,100.4);r.capture("pi_yaw");r.euler(euler(0,0,2*pi),100.5,100.5);r.capture("twopi_yaw");}
 {Rig r(path);r.bind();r.euler(euler(0,0,-1e-17),100.2,100.2);r.capture("tiny_negative_euler");r.quaternion(quaternion(0,0,-1e-17),100.3,100.3);r.capture("tiny_negative_quaternion");r.euler(euler(0,0,1e-17),100.4,100.4);r.capture("tiny_positive_euler");r.quaternion(quaternion(0,0,1e-17),100.5,100.5);r.capture("tiny_positive_quaternion");r.euler(euler(0,0,-0.0),100.6,100.6);r.capture("negative_zero_heading");}
 {Rig r(path);r.bind();r.quaternion(quaternion(),100.2,100.2);r.capture("q_identity");r.quaternion(quaternion(0,0,pi/2),100.3,100.3);r.capture("q_east");r.quaternion(quaternion(0,0,-pi/2),100.4,100.4);r.capture("q_west");r.quaternion(quaternion(pi/2,0,0),100.5,100.5);r.capture("q_roll");r.quaternion(quaternion(.2,-.3,2.2),100.6,100.6);r.capture("q_mixed");}
 {Rig r(path);r.bind();auto p=quaternion(.2,-.3,2.2);p.q1=-p.q1;p.q2=-p.q2;p.q3=-p.q3;p.q4=-p.q4;r.quaternion(p,100.2,100.2);r.capture("q_negative");p=quaternion();p.q1=.9995;r.quaternion(p,100.3,100.3);r.capture("q_near_unit");p.q1=2;r.quaternion(p,100.4,100.4);r.capture("q_nonunit");p.q1=0;r.quaternion(p,100.5,100.5);r.capture("q_zero");p.q1=FLT_MAX;p.q2=FLT_MAX;r.quaternion(p,100.6,100.6);r.capture("q_huge");}
 {Rig r(path);r.bind();r.quaternion(quaternion(0,pi/2,0),100.2,100.2);r.capture("q_singular");r.euler(euler(0,pi/2,pi/3),100.3,100.3);r.capture("euler_singular");}
 {Rig r(path);r.bind();r.euler(euler(),100.2,100.2);r.quaternion(quaternion(),100.3,100.3);r.capture("both_families");r.heartbeat(101.1,101.1);r.quaternion(quaternion(0,0,pi/2),101.3,101.3);r.capture("independent_ages");}
 {Rig r(path);r.bind();auto p=euler();p.roll=NAN;r.euler(p,100.2,100.2);r.capture("euler_nan");p=euler();p.pitch=INFINITY;r.euler(p,100.3,100.3);r.capture("euler_inf");p=euler(4,0,0);r.euler(p,100.4,100.4);r.capture("euler_range");p=euler(0,0,1000);r.euler(p,100.5,100.5);r.capture("yaw_range");p=euler();p.pitchspeed=NAN;r.euler(p,100.6,100.6);r.capture("euler_bad_rate");}
 {Rig r(path);r.bind();auto p=quaternion();p.q2=NAN;r.quaternion(p,100.2,100.2);r.capture("q_nan");p=quaternion();p.q4=-INFINITY;r.quaternion(p,100.3,100.3);r.capture("q_inf");p=quaternion();p.yawspeed=INFINITY;r.quaternion(p,100.4,100.4);r.capture("q_bad_rate");}
 {Rig r(path);r.bind();auto p=quaternion();r.quaternion(p,100.2,100.2);r.capture("offset_unsupported");p.repr_offset_q[0]=1;r.quaternion(p,100.3,100.3);r.capture("offset_identity");p.repr_offset_q[0]=std::sqrt(.5);p.repr_offset_q[1]=std::sqrt(.5);r.quaternion(p,100.4,100.4);r.capture("offset_unapplied");p.repr_offset_q[0]=2;p.repr_offset_q[1]=0;r.quaternion(p,100.5,100.5);r.capture("offset_invalid");p.repr_offset_q[0]=NAN;r.quaternion(p,100.6,100.6);r.capture("offset_nan");}
 {Rig r(path);r.bind();auto p=euler();p.time_boot_ms=UINT32_MAX;r.euler(p,100.2,100.2);r.capture("boot_max");p.time_boot_ms=0;r.euler(p,100.3,100.3);r.capture("boot_wrap");r.euler(p,100.4,100.4);r.capture("same_remote_new_local");}
 {Rig r(path);r.bind();r.euler(euler(),100.2,100.3);r.capture("order_before");mavlink_message_t m{};auto p=euler(0,0,0);mavlink_msg_attitude_encode_status(1,1,&r.codec,&m,&p);r.host.time=100.8;r.plugin->onMessageWithMetadata(m,{100.15,100.7,"QUEUE_EMPTY_FLOOR"});r.plugin->onMessageWithMetadata(m,{100.2,100.29,"QUEUE_EMPTY_FLOOR"});r.capture("order_after");}
 {Rig r(path);r.bind();r.euler(euler(0,0,0),100.2,100.3);r.capture("coalesced_before");r.euler(euler(0,0,1),100.2,100.3);r.host.time=100.8;r.capture("coalesced_after");}
 {Rig r(path);r.bind();auto p=euler();mavlink_message_t m{};mavlink_msg_attitude_encode_status(1,1,&r.codec,&m,&p);for(const auto& t:std::vector<MessageTime>{{NAN,100.2,"QUEUE_EMPTY_FLOOR"},{100.2,INFINITY,"QUEUE_EMPTY_FLOOR"},{100.3,100.2,"QUEUE_EMPTY_FLOOR"},{100.2,101,"QUEUE_EMPTY_FLOOR"},{100.1,100.2,"READ_ACQUISITION"},{100.2,100.2,"UNKNOWN"},{99,100.2,"QUEUE_EMPTY_FLOOR"},{100,100.2,"QUEUE_EMPTY_FLOOR"}}){r.host.time=100.2;r.plugin->onMessageWithMetadata(m,t);}r.capture("metadata_invalid");r.receive(m,100.4,100.4,"READ_ACQUISITION");r.capture("read_acquisition");}
 {Rig r(path);r.heartbeat(99.9,100.1);r.euler(euler(),100.11,100.11);r.capture("preconstruction_floor");r.heartbeat(100.2,100.2);r.euler(euler(),100.3,100.3);r.capture("after_boundary");}
 {Rig r(path);r.bind();r.euler(euler(),100.2,100.2);r.capture("source_before");auto p=euler(0,0,0);mavlink_message_t m{};mavlink_msg_attitude_encode_status(2,1,&r.codec,&m,&p);r.receive(m,100.3,100.3);m.sysid=1;m.compid=200;r.receive(m,100.4,100.4);r.host.time=100.8;r.capture("foreign_source");r.heartbeat(100.9,100.9,2);r.heartbeat(101,101,1,1,10);r.heartbeat(101.1,101.1,1,1,2,12);r.capture("foreign_heartbeats");r.host.time=102.2;r.plugin->onInputBoundary();r.capture("heartbeat_lapse");r.heartbeat(102.3,102.3);r.capture("heartbeat_only");}
 {Rig r(path);r.bind();r.euler(euler(),100.2,100.2);r.quaternion(quaternion(),100.3,100.3);r.host.time=101;r.plugin->onReset("quotes\" newline\n reset");r.capture("reset");r.heartbeat(100.9,101.1);r.euler(euler(),100.95,101.2);r.capture("old_floor_after_reset");r.heartbeat(101.3,101.3);r.capture("rebound_empty");r.euler(euler(),101.4,101.4);r.capture("new_after_reset");}
 {Rig r(path);r.bind();r.euler(euler(),100.2,100.2);r.host.time=99;r.capture("clock_fault");r.heartbeat(101,101);r.capture("clock_still_latched");r.host.time=101.1;r.plugin->onReset("clock_recovered");r.heartbeat(101.2,101.2);r.euler(euler(),101.3,101.3);r.capture("clock_recovery");}
 {Rig r(path);r.bind();r.host.time=NAN;r.plugin->onInputBoundary();r.host.time=101;r.capture("clock_nan_latched");r.host.time=NAN;r.plugin->onReset("bad_clock_reset");r.host.time=101.1;r.capture("clock_bad_reset_latched");}
 {Rig r(path);r.bind();r.euler(euler(),100.2,100.2);r.host.time=100.3;r.host.state.heartbeat_valid=false;r.plugin->onInputBoundary();r.capture("host_heartbeat_invalid");}
 {Rig r(path);r.bind();auto p=euler();mavlink_message_t m{};mavlink_msg_attitude_encode_status(1,1,&r.codec,&m,&p);for(unsigned n=0;n<5;++n){auto bad=m;switch(n){case 0:bad.magic=0;break;case 1:bad.len=255;break;case 2:bad.len=0;break;case 3:bad.incompat_flags=128;break;case 4:bad.magic=MAVLINK_STX_MAVLINK1;bad.len=27;break;}r.receive(bad,100.2,100.2);}auto unrelated=m;unrelated.msgid=MAVLINK_MSG_ID_SYS_STATUS;r.plugin->onMessageWithMetadata(unrelated,{NAN,NAN,"UNKNOWN"});r.capture("wire_rejects");m.len=1;r.receive(m,100.3,100.3);r.host.time=100.8;r.capture("v2_zero_tail");m.magic=MAVLINK_STX_MAVLINK1;m.len=MAVLINK_MSG_ID_ATTITUDE_MIN_LEN;r.receive(m,100.9,100.9);r.host.time=101.1;r.capture("v1_euler");}
 {Rig r(path);r.bind();mavlink_message_t m{};auto p=quaternion();mavlink_msg_attitude_quaternion_encode_status(1,1,&r.codec,&m,&p);m.magic=MAVLINK_STX_MAVLINK1;m.len=MAVLINK_MSG_ID_ATTITUDE_QUATERNION_MIN_LEN;r.receive(m,100.2,100.2);r.capture("v1_quaternion");m.magic=MAVLINK_STX;m.len=1;r.receive(m,100.3,100.3);r.capture("q_v2_zero_tail");}
 {Rig r(path);r.bind();r.plugin->configure("attitude_publish_interval",".5");r.euler(euler(),100.2,100.2);r.capture("interval_first");r.euler(euler(0,0,0),100.3,100.3);r.capture("interval_suppressed");r.host.time=100.8;r.capture("interval_later");}
 {Rig r(path);r.bind();for(const auto& key:{"attitude_timeout","attitude_publish_interval"})for(const auto& value:{"nan","inf"," 1","0","100","1x"}){bool rejected=false;try{r.plugin->configure(key,value);}catch(...){rejected=true;}require(rejected,"invalid config accepted");}r.plugin->configure("attitude_timeout",".25");r.euler(euler(),100.2,100.2);r.host.time=100.46;r.capture("short_timeout");}
 {Rig r(path);r.bind();uint32_t random=0x428397u;for(unsigned n=0;n<2048;++n){auto next=[&](){random=random*1664525u+1013904223u;return random;};mavlink_message_t m{};m.magic=n%11==0?MAVLINK_STX_MAVLINK1:MAVLINK_STX;m.msgid=n%2?30:31;m.len=next()%64;m.sysid=1;m.compid=1;m.incompat_flags=n%13==0?128:0;auto payload=reinterpret_cast<unsigned char*>(m.payload64);for(std::size_t i=0;i<sizeof(m.payload64);++i)payload[i]=next()>>24;const double at=100.2+n*.0001;r.receive(m,at,at);if(n%127==126)r.capture("bounded_random_frames");}r.host.time=101.1;r.capture("bounded_random_frames");}
 // Direct Services counterexample: a reset must retain the reliable101 s high-water.
 {Rig r(path);r.bind();r.heartbeat(100.8,100.8);r.euler(euler(),101,101);r.capture("hardening_reliable");
  r.host.time=100.9;r.plugin->onInputBoundary();r.plugin->onReset("behind_highwater");r.capture("hardening_early_reset");
  r.heartbeat(100.95,100.95);r.euler(euler(),100.96,100.96);r.host.time=101.1;r.capture("hardening_input_before_highwater");
  r.heartbeat(101.9,101.9);r.euler(euler(),101.95,101.95);r.host.time=102;r.capture("hardening_catchup_without_reset");
  r.host.time=102.1;r.plugin->onReset("explicit_numeric_recovery");r.capture("hardening_valid_reset");
  r.heartbeat(102,102.2);r.euler(euler(),102.05,102.25);r.host.time=102.3;r.capture("hardening_old_floor");
  r.heartbeat(102.4,102.4);r.euler(euler(),102.45,102.45);r.host.time=102.5;r.capture("hardening_recovered");}
 {unsigned index=0;for(double bad:{-1.0,double(NAN),double(INFINITY),100.9}){Rig r(path);r.bind();r.heartbeat(100.8,100.8);r.euler(euler(),101,101);r.plugin->onInputBoundary();
  r.host.time=bad;r.plugin->onReset("invalid_numeric_reset");r.host.time=101.1;
  const auto label="hardening_invalid_reset_"+std::to_string(index++);r.capture(label.c_str());}}
 {Rig r(path);r.bind();r.heartbeat(100.8,100.8);r.euler(euler(),101,101);r.plugin->onInputBoundary();
  r.host.time=101;r.plugin->onReset("equal_highwater");r.heartbeat(101.1,101.1);r.euler(euler(),101.2,101.2);r.capture("hardening_equal_reset");}
 std::cout<<"{\"case\":\"driver_complete\",\"key\":\"summary\",\"value\":{\"effects\":0,\"abi\":"<<plugin_abi<<"}}\n";
 return 0;
}catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}}
