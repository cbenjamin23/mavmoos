// Real MODULE/factory with fake Services; no sockets, MOOS or vehicle effects.
#include "Plugin.h"
#include <cmath>
#include <dlfcn.h>
#include <iostream>
#include <stdexcept>
using namespace mavmoos::runtime;
namespace {
void require(bool value,const char* text){if(!value)throw std::runtime_error(text);}
struct Host final:Services {
 double time=100;Vehicle state;unsigned effects=0;std::vector<std::pair<std::string,std::string>> publications;
 Host(){state.type=2;state.autopilot=3;state.heartbeat_valid=true;state.heartbeat_at=time;}
 double now()const override{return time;}const Vehicle& vehicle()const override{return state;}
 void publish(const std::string& key,const std::string& value)override{require(key=="MAVMOOS_RC_INPUT"||key=="MAVMOOS_SERVO_OUTPUT"||key=="MAVMOOS_RC_IO_STATUS","unexpected publication");require(value.size()<100000,"bounded publication");publications.emplace_back(key,value);}
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
 bool navigationGranted()const override{throw std::runtime_error("authority query");}
 bool controlBusy()const override{throw std::runtime_error("control query");}
 bool projectGlobal(double,double,double&,double&)const override{throw std::runtime_error("projection query");}
};
struct Rig {
 Host host;void* handle=nullptr;Plugin* plugin=nullptr;DestroyPlugin destroy=nullptr;mavlink_status_t codec{};
 explicit Rig(const char* path){handle=dlopen(path,RTLD_NOW|RTLD_LOCAL|RTLD_NODELETE);require(handle,"dlopen");auto create=reinterpret_cast<CreatePlugin>(dlsym(handle,"mavmoos_create_plugin"));destroy=reinterpret_cast<DestroyPlugin>(dlsym(handle,"mavmoos_destroy_plugin"));require(create&&destroy,"exports");require(!create(plugin_abi-1,&host)&&!create(plugin_abi+1,&host)&&!create(plugin_abi,nullptr),"ABI/context");plugin=create(plugin_abi,&host);require(plugin&&plugin->name()=="rc_io","factory identity");require(!host.effects&&host.publications.empty(),"factory effect");plugin->configure("rc_io_publish_interval",".02");}
 ~Rig(){if(plugin)destroy(plugin);if(handle)dlclose(handle);if(host.effects){std::cerr<<"observer effect\n";std::terminate();}}
 void receive(const mavlink_message_t& m,double effective,double read,const std::string& basis="QUEUE_EMPTY_FLOOR"){host.time=read;plugin->onMessageWithMetadata(m,{effective,read,basis});}
 void heartbeat(double effective,double read,unsigned system=1,unsigned component=1,unsigned type=2,unsigned autopilot=3){mavlink_message_t m{};host.state.heartbeat_valid=true;host.state.heartbeat_at=effective;mavlink_msg_heartbeat_pack_status(system,component,&codec,&m,type,autopilot,0,4,MAV_STATE_ACTIVE);receive(m,effective,read);}
 void bind(){heartbeat(100.1,100.1);}
 mavlink_message_t inputFrame(const mavlink_rc_channels_t& p){mavlink_message_t m{};mavlink_msg_rc_channels_encode_status(1,1,&codec,&m,&p);return m;}
 mavlink_message_t outputFrame(const mavlink_servo_output_raw_t& p){mavlink_message_t m{};mavlink_msg_servo_output_raw_encode_status(1,1,&codec,&m,&p);return m;}
 void input(const mavlink_rc_channels_t& p,double effective,double read){receive(inputFrame(p),effective,read);}
 void output(const mavlink_servo_output_raw_t& p,double effective,double read){receive(outputFrame(p),effective,read);}
 void capture(const char* label){plugin->iterate();for(const auto& item:host.publications)std::cout<<"{\"case\":\""<<label<<"\",\"key\":\""<<item.first<<"\",\"value\":"<<item.second<<"}\n";host.publications.clear();require(!host.effects,"observer effect");}
};
mavlink_rc_channels_t input(unsigned count=18,unsigned rssi=254){mavlink_rc_channels_t p{};p.time_boot_ms=1234;p.chancount=count;p.rssi=rssi;p.chan1_raw=1001;p.chan2_raw=1102;p.chan3_raw=1203;p.chan4_raw=1304;p.chan5_raw=1405;p.chan6_raw=1506;p.chan7_raw=1607;p.chan8_raw=1708;p.chan9_raw=1809;p.chan10_raw=1910;p.chan11_raw=2011;p.chan12_raw=2112;p.chan13_raw=2213;p.chan14_raw=2314;p.chan15_raw=2415;p.chan16_raw=2516;p.chan17_raw=2617;p.chan18_raw=2718;return p;}
mavlink_servo_output_raw_t output(unsigned port=0){mavlink_servo_output_raw_t p{};p.time_usec=4321000;p.port=port;p.servo1_raw=1001;p.servo2_raw=1102;p.servo3_raw=1203;p.servo4_raw=1304;p.servo5_raw=1405;p.servo6_raw=1506;p.servo7_raw=1607;p.servo8_raw=1708;p.servo9_raw=1809;p.servo10_raw=1910;p.servo11_raw=2011;p.servo12_raw=2112;p.servo13_raw=2213;p.servo14_raw=2314;p.servo15_raw=2415;p.servo16_raw=2516;return p;}
}
int main(int argc,char** argv){try{
 require(argc==2,"usage: rc_io_scenario_driver RC_IO_MODULE");const char* path=argv[1];
 {Rig r(path);auto d=r.plugin->descriptor();require(d.profile=="ardupilot_rc_observations"&&d.capabilities.size()==2&&d.capabilities[0].name=="rc.input"&&d.capabilities[1].name=="rc.servo_output"&&d.capabilities[0].access=="observation"&&d.capabilities[1].access=="observation"&&d.request_owners.empty()&&d.exclusive_resources==std::vector<std::string>{"observation.rc_io"}&&d.streamed_parameter_bench_compatible&&r.plugin->subscriptions().empty()&&!r.plugin->controlBackend(),"descriptor/effects");require(d.observation_streams.size()==2&&d.observation_streams[0].message_id==65&&d.observation_streams[1].message_id==36&&d.observation_streams[0].interval_us==100000&&d.observation_streams[1].interval_us==200000,"stream declaration");r.plugin->onMail({"MAVMOOS_RC_OVERRIDE","anything","test"});r.capture("absent");}
 {Rig r(path);r.input(input(),100.02,100.02);r.output(output(),100.03,100.03);r.capture("unbound");r.bind();r.input(input(),100.12,100.3);r.output(output(),100.15,100.31);r.capture("both");r.host.time=100.9;r.capture("cached");r.heartbeat(101,101);r.output(output(1),101.2,101.2);r.capture("independent_ages");}
 {Rig r(path);r.bind();unsigned n=0;for(unsigned count:{0u,1u,8u,18u,19u,255u}){const double at=100.2+n++*.1;r.input(input(count),at,at);const auto label="count_"+std::to_string(count);r.capture(label.c_str());}}
 {Rig r(path);r.bind();auto p=input(2,255);p.chan1_raw=0;p.chan2_raw=65535;r.input(p,100.2,100.2);r.capture("input_extremes");r.input(input(18,0),100.3,100.3);r.capture("rssi_zero");}
 {Rig r(path);r.bind();r.output(output(1),100.2,100.2);r.output(output(0),100.3,100.3);r.output(output(255),100.4,100.4);r.capture("ports_sorted");r.output(output(1),100.5,100.5);r.capture("port_update");}
 {Rig r(path);r.bind();for(unsigned n=0;n<8;++n)r.output(output(n),100.2+n*.001,100.2+n*.001);r.capture("ports_full");r.output(output(255),100.3,100.3);r.output(output(255),100.4,100.4);r.capture("port_capacity");auto p=output(3);p.servo1_raw=1999;r.output(p,100.5,100.5);r.capture("retained_port_update");r.host.time=100.6;r.plugin->onReset("capacity_reset");r.heartbeat(100.7,100.7);r.output(output(255),100.8,100.8);r.capture("port_reset_capacity");}
 {Rig r(path);r.bind();auto p=output();p.servo1_raw=0;p.servo2_raw=65535;p.servo9_raw=0;p.servo16_raw=0;r.output(p,100.2,100.2);r.capture("output_extremes");p={};p.port=1;r.output(p,100.3,100.3);r.capture("output_zero");}
 {Rig r(path);r.bind();auto m=r.outputFrame(output());m.magic=MAVLINK_STX_MAVLINK1;m.len=MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_MIN_LEN;r.receive(m,100.2,100.2);r.capture("servo_v1");m=r.inputFrame(input());m.magic=MAVLINK_STX_MAVLINK1;m.len=MAVLINK_MSG_ID_RC_CHANNELS_MIN_LEN;r.receive(m,100.3,100.3);r.capture("input_v1");}
 {Rig r(path);r.bind();auto p=output();p.servo9_raw=7;auto m=r.outputFrame(p);m.len=22;r.receive(m,100.2,100.2);r.capture("partial_extension");m.len=21;r.receive(m,100.3,100.3);r.capture("omitted_extension");m.len=1;r.receive(m,100.4,100.4);r.capture("v2_zero_tail");m=r.inputFrame(input());m.len=1;r.receive(m,100.5,100.5);r.capture("input_zero_tail");}
 {Rig r(path);r.bind();auto p=input();p.time_boot_ms=UINT32_MAX;r.input(p,100.2,100.2);auto q=output();q.time_usec=UINT32_MAX;r.output(q,100.21,100.21);r.capture("remote_max");p.time_boot_ms=0;q.time_usec=0;r.input(p,100.3,100.3);r.output(q,100.31,100.31);r.capture("remote_wrap");r.input(p,100.4,100.4);r.output(q,100.41,100.41);r.capture("remote_repeat");}
 {Rig r(path);r.bind();r.input(input(),100.2,100.3);r.output(output(),100.2,100.3);r.capture("order_before");auto p=input();p.chan1_raw=1999;auto q=output();q.servo1_raw=1999;r.host.time=100.8;r.plugin->onMessageWithMetadata(r.inputFrame(p),{100.15,100.7,"QUEUE_EMPTY_FLOOR"});r.plugin->onMessageWithMetadata(r.outputFrame(q),{100.2,100.29,"QUEUE_EMPTY_FLOOR"});r.capture("order_after");r.host.time=100.9;r.plugin->onMessageWithMetadata(r.inputFrame(p),{100.2,100.3,"QUEUE_EMPTY_FLOOR"});r.plugin->onMessageWithMetadata(r.outputFrame(q),{100.2,100.3,"QUEUE_EMPTY_FLOOR"});r.capture("coalesced_frames");}
 {Rig r(path);r.bind();const auto m=r.inputFrame(input());for(const auto& t:std::vector<MessageTime>{{NAN,100.2,"QUEUE_EMPTY_FLOOR"},{100.2,INFINITY,"QUEUE_EMPTY_FLOOR"},{100.3,100.2,"QUEUE_EMPTY_FLOOR"},{100.2,101,"QUEUE_EMPTY_FLOOR"},{100.1,100.2,"READ_ACQUISITION"},{100.2,100.2,"UNKNOWN"},{99,100.2,"QUEUE_EMPTY_FLOOR"},{100,100.2,"QUEUE_EMPTY_FLOOR"}}){r.host.time=100.2;r.plugin->onMessageWithMetadata(m,t);}r.capture("metadata_invalid");r.receive(m,100.4,100.4,"READ_ACQUISITION");r.capture("read_acquisition");}
 {Rig r(path);r.heartbeat(99.9,100.1);r.input(input(),100.11,100.11);r.capture("preconstruction_floor");r.heartbeat(100.2,100.2);r.input(input(),100.3,100.3);r.capture("after_boundary");}
 {Rig r(path);r.bind();r.input(input(),100.2,100.2);r.output(output(),100.21,100.21);r.capture("source_before");auto m=r.inputFrame(input());m.sysid=2;r.receive(m,100.3,100.3);m.sysid=1;m.compid=200;r.receive(m,100.4,100.4);m=r.outputFrame(output(1));m.sysid=2;r.receive(m,100.5,100.5);r.heartbeat(100.6,100.6,2);r.heartbeat(100.7,100.7,1,1,10);r.heartbeat(100.8,100.8,1,1,2,12);r.capture("foreign_sources");r.host.time=102.2;r.plugin->onInputBoundary();r.capture("heartbeat_lapse");r.heartbeat(102.3,102.3);r.capture("heartbeat_only");}
 {Rig r(path);r.bind();r.input(input(),100.2,100.2);r.output(output(),100.3,100.3);r.host.time=101;r.plugin->onReset("quotes\" newline\n reset");r.capture("reset");r.heartbeat(100.9,101.1);r.input(input(),100.95,101.2);r.capture("old_floor_after_reset");r.heartbeat(101.3,101.3);r.capture("rebound_empty");r.input(input(),101.4,101.4);r.output(output(255),101.5,101.5);r.capture("new_after_reset");}
 {Rig r(path);r.bind();r.input(input(),100.2,100.2);r.host.time=99;r.capture("clock_fault");r.heartbeat(101,101);r.capture("clock_still_latched");r.host.time=101.1;r.plugin->onReset("clock_recovered");r.heartbeat(101.2,101.2);r.input(input(),101.3,101.3);r.capture("clock_recovery");}
 {Rig r(path);r.bind();r.input(input(),100.2,100.2);r.host.time=101;r.capture("highwater_before");r.host.time=100.9;r.capture("highwater_rollback");r.plugin->onReset("reset_behind_highwater");r.capture("highwater_bad_reset");r.heartbeat(100.95,100.95);r.input(input(),100.96,100.96);r.output(output(),100.97,100.97);r.host.time=101.1;r.capture("highwater_catchup_still_latched");r.host.time=101.2;r.plugin->onReset("highwater_caught_up");r.heartbeat(101.3,101.3);r.input(input(),101.4,101.4);r.output(output(),101.5,101.5);r.capture("highwater_recovery");}
 {Rig r(path);r.bind();r.host.time=NAN;r.plugin->onInputBoundary();r.host.time=101;r.capture("clock_nan_latched");r.host.time=NAN;r.plugin->onReset("bad_clock_reset");r.host.time=101.1;r.capture("clock_bad_reset_latched");}
 {Rig r(path);r.bind();r.input(input(),100.2,100.2);r.output(output(),100.21,100.21);r.host.time=100.3;r.host.state.heartbeat_valid=false;r.plugin->onInputBoundary();r.capture("host_heartbeat_invalid");}
 {Rig r(path);r.bind();auto m=r.inputFrame(input());for(unsigned n=0;n<5;++n){auto bad=m;switch(n){case 0:bad.magic=0;break;case 1:bad.len=255;break;case 2:bad.len=0;break;case 3:bad.incompat_flags=128;break;case 4:bad.magic=MAVLINK_STX_MAVLINK1;bad.len=41;break;}r.receive(bad,100.2,100.2);}auto unrelated=m;unrelated.msgid=MAVLINK_MSG_ID_RC_CHANNELS_RAW;r.plugin->onMessageWithMetadata(unrelated,{NAN,NAN,"UNKNOWN"});r.capture("wire_rejects");}
 {Rig r(path);r.bind();r.plugin->configure("rc_io_publish_interval",".5");r.input(input(),100.2,100.2);r.capture("interval_first");r.input(input(1),100.3,100.3);r.capture("interval_suppressed");r.host.time=100.8;r.capture("interval_later");}
 {Rig r(path);r.bind();for(const auto& key:{"rc_io_timeout","rc_io_publish_interval"})for(const auto& value:{"nan","inf"," 1","0","100","1x"}){bool rejected=false;try{r.plugin->configure(key,value);}catch(...){rejected=true;}require(rejected,"invalid config accepted");}r.plugin->configure("rc_io_timeout",".25");r.input(input(),100.2,100.2);r.output(output(),100.21,100.21);r.host.time=100.47;r.capture("short_timeout");}
 {Rig r(path);r.bind();uint32_t random=0x428397u;for(unsigned n=0;n<2048;++n){auto next=[&](){random=random*1664525u+1013904223u;return random;};mavlink_message_t m{};m.magic=n%11==0?MAVLINK_STX_MAVLINK1:MAVLINK_STX;m.msgid=n%2?65:36;m.len=next()%48;m.sysid=1;m.compid=1;m.incompat_flags=n%13==0?128:0;auto payload=reinterpret_cast<unsigned char*>(m.payload64);for(std::size_t i=0;i<sizeof(m.payload64);++i)payload[i]=next()>>24;const double at=100.2+n*.0001;r.receive(m,at,at);if(n%127==126)r.capture("bounded_random_frames");}r.host.time=101.1;r.capture("bounded_random_frames");}
 std::cout<<"{\"case\":\"driver_complete\",\"key\":\"summary\",\"value\":{\"effects\":0,\"abi\":"<<plugin_abi<<",\"random_frames\":2048}}\n";
 return 0;
}catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}}
