// Real compiled plugin, fake Services, no sockets/MOOS/vehicle.
#include "Plugin.h"
#include "DiagnosticFrames.h"
#include "PowerFrames.h"
#include "../gnss_observations/GnssCases.h"
#include <dlfcn.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>
using namespace mavmoos::runtime;
namespace diagnostics=mavmoos::test::diagnostics;
namespace power=mavmoos::test::power_fixture;
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
struct Host final:Services {
 double time=100;Vehicle state;unsigned effects=0;std::vector<std::pair<std::string,std::string>> output;
 Host(){state.type=2;state.autopilot=3;state.heartbeat_valid=true;state.heartbeat_at=time;}
 double now()const override{return time;}const Vehicle& vehicle()const override{return state;}
 void publish(const std::string& key,const std::string& value)override{output.emplace_back(key,value);}
 void publish(const std::string&,double)override{throw std::runtime_error("unexpected numeric status projection");}
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
 explicit Rig(const char* path){handle=dlopen(path,RTLD_NOW|RTLD_LOCAL|RTLD_NODELETE);require(handle,"dlopen failed");auto create=reinterpret_cast<CreatePlugin>(dlsym(handle,"mavmoos_create_plugin"));destroy=reinterpret_cast<DestroyPlugin>(dlsym(handle,"mavmoos_destroy_plugin"));require(create&&destroy,"missing exports");require(!create(plugin_abi-1,&host)&&!create(plugin_abi+1,&host)&&!create(plugin_abi,nullptr),"ABI/context failure");plugin=create(plugin_abi,&host);require(plugin&&plugin->name()=="system_status","factory identity");require(host.effects==0&&host.output.empty(),"factory effect");plugin->configure("status_publish_interval",".05");}
 ~Rig(){if(plugin)destroy(plugin);if(handle)dlclose(handle);if(host.effects){std::cerr<<"status provider caused effect\n";std::terminate();}}
 void receive(const mavlink_message_t& message,double effective,double read,const std::string& basis="QUEUE_EMPTY_FLOOR"){host.time=read;plugin->onMessageWithMetadata(message,{effective,read,basis});}
 void heartbeat(double effective,double read,unsigned system=1,unsigned component=1){mavlink_message_t msg{};host.state.heartbeat_valid=true;host.state.heartbeat_at=effective;mavlink_msg_heartbeat_pack_status(system,component,&codec,&msg,2,3,0,4,MAV_STATE_ACTIVE);receive(msg,effective,read);}
 void bind(){heartbeat(100.1,100.1);}
 void system(const mavlink_sys_status_t& p,double effective,double read){mavlink_message_t msg{};mavlink_msg_sys_status_encode_status(1,1,&codec,&msg,&p);receive(msg,effective,read);}
 void battery(const mavlink_battery_status_t& p,double effective,double read){mavlink_message_t msg{};mavlink_msg_battery_status_encode_status(1,1,&codec,&msg,&p);receive(msg,effective,read);}
 void gps(const mavlink_gps_raw_int_t& p,double effective,double read){mavlink_message_t msg{};mavlink_msg_gps_raw_int_encode_status(1,1,&codec,&msg,&p);receive(msg,effective,read);}
 void text(unsigned id,unsigned sequence,const std::string& bytes,double at,unsigned severity=4){mavlink_statustext_t p{};p.id=id;p.chunk_seq=sequence;p.severity=severity;std::memcpy(p.text,bytes.data(),std::min(bytes.size(),sizeof(p.text)));mavlink_message_t msg{};mavlink_msg_statustext_encode_status(1,1,&codec,&msg,&p);receive(msg,at,at);}
 void capture(const char* label){plugin->iterate();for(const auto& item:host.output)std::cout<<"{\"case\":\""<<label<<"\",\"key\":\""<<item.first<<"\",\"value\":"<<item.second<<"}\n";host.output.clear();require(host.effects==0,"observer caused authority/IO effect");}
};
mavlink_sys_status_t system(){mavlink_sys_status_t p{};p.onboard_control_sensors_present=15;p.onboard_control_sensors_enabled=7;p.onboard_control_sensors_health=5;p.load=125;p.voltage_battery=12345;p.current_battery=250;p.battery_remaining=75;p.drop_rate_comm=123;p.errors_comm=4;p.errors_count1=5;return p;}
mavlink_battery_status_t battery(unsigned id=0){mavlink_battery_status_t p{};p.id=id;p.temperature=2500;p.current_battery=-100;p.current_consumed=1234;p.energy_consumed=720;p.battery_remaining=80;p.time_remaining=600;p.charge_state=1;for(unsigned i=0;i<10;++i)p.voltages[i]=UINT16_MAX;p.voltages[0]=3700;p.voltages[1]=3800;p.voltages[2]=3900;return p;}
mavlink_gps_raw_int_t gps(){mavlink_gps_raw_int_t p{};p.time_usec=18446744073709551000ULL;p.fix_type=3;p.lat=423584560;p.lon=-710875890;p.alt=10083;p.eph=123;p.epv=234;p.vel=345;p.cog=9000;p.satellites_visible=12;p.alt_ellipsoid=35000;p.h_acc=1200;p.v_acc=2300;p.vel_acc=123;p.hdg_acc=345000;p.yaw=36000;return p;}
}
int main(int argc,char** argv){try{
 require(argc==2,"usage: status_scenario_driver SYSTEM_STATUS_MODULE");const char* path=argv[1];
 {Rig r(path);auto d=r.plugin->descriptor();require(d.profile=="ardupilot_system_observations"&&d.capabilities.size()==12&&d.request_owners.empty()&&d.exclusive_resources==std::vector<std::string>{"observation.system_status"}&&d.streamed_parameter_bench_compatible&&!r.plugin->controlBackend()&&r.plugin->subscriptions().empty(),"descriptor/effects");require(d.observation_streams.size()==3,"stream profile");r.capture("absent");}
 {
  Rig r(path);r.bind();
  const auto power=diagnostics::encode(r.codec,diagnostics::power(),mavlink_msg_power_status_encode_status);
  const auto common=diagnostics::encode(r.codec,diagnostics::estimator(),mavlink_msg_estimator_status_encode_status);
  const auto mcu=diagnostics::encode(r.codec,diagnostics::mcu(2),mavlink_msg_mcu_status_encode_status);
  r.receive(power,100.08,100.12); // The queued frame predates the bound heartbeat.
  r.receive(power,100.2,100.25);
  r.receive(common,100.26,100.26);
  r.receive(mcu,100.27,100.27);
  for(unsigned id:{6,10,20,30,40,50,60,70}) {
    auto frame=diagnostics::encode(r.codec,diagnostics::mcu(id),mavlink_msg_mcu_status_encode_status);
    r.receive(frame,100.28,100.28);
  }
  auto invalid_v1=mcu;
  invalid_v1.magic=MAVLINK_STX_MAVLINK1;
  r.receive(invalid_v1,100.29,100.29);
  auto foreign=diagnostics::encode(r.codec,diagnostics::power(),mavlink_msg_power_status_encode_status,1,2);
  r.receive(foreign,100.29,100.29);
  r.codec.flags=MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  r.receive(diagnostics::encode(r.codec,diagnostics::ekf(),mavlink_msg_ekf_status_report_encode_status),100.3,100.3);
  r.codec.flags=0;
  r.capture("diagnostics");
  auto ekf=diagnostics::ekf();
  ekf.compass_variance=std::numeric_limits<float>::infinity();
  ekf.pos_horiz_variance=-1;
  r.receive(diagnostics::encode(r.codec,ekf,mavlink_msg_ekf_status_report_encode_status),100.4,100.4);
  auto outside=diagnostics::mcu(2);outside.MCU_voltage_max=3250;
  r.receive(diagnostics::encode(r.codec,outside,mavlink_msg_mcu_status_encode_status),100.41,100.41);
  r.capture("diagnostics_field_fault");
  r.heartbeat(101.1,101.1);
  r.plugin->configure("status_timeout",".8");
  r.receive(power,101.3,101.3);r.capture("diagnostics_age_split");
  r.host.time=103.2;r.capture("diagnostics_cleared");
  r.receive(common,103.3,103.3);r.capture("diagnostics_data_only");
  r.heartbeat(103.4,103.4);
  r.receive(power,103.5,103.5);r.capture("diagnostics_rebound");
 }
 {
  Rig r(path);
  r.receive(power::batteryFrame(r.codec),100.02,100.03);r.capture("power_metadata_unbound");
  r.bind();
  r.receive(power::batteryFrame(r.codec),100.15,100.16);
  r.receive(power::batteryFrame(r.codec,power::unknown()),100.19,100.20);
  for(unsigned id: {0,1,2,3,5,6,7})
    r.receive(power::batteryFrame(r.codec,power::battery(id)),100.21,100.21);
  r.receive(power::generatorFrame(r.codec),100.23,100.25);
  r.battery(battery(4),100.26,100.26);r.capture("power_metadata");
  auto invalid=power::generator();invalid.bus_voltage=-1;
  r.receive(power::generatorFrame(r.codec,invalid,2,1),100.28,100.28);
  auto wrong_shape=power::generatorFrame(r.codec,invalid);wrong_shape.len=0;
  r.receive(wrong_shape,100.29,100.29);
  auto older=power::battery();older.design_capacity=99;
  r.receive(power::batteryFrame(r.codec,older),100.14,100.30);
  r.host.time=100.32;r.capture("power_metadata_rejected");
  r.heartbeat(101.05,101.05);r.plugin->configure("status_timeout",".5");
  r.receive(power::batteryFrame(r.codec,power::battery(9)),101.10,101.11);
  invalid.power_generated=std::numeric_limits<float>::infinity();
  invalid.generator_temperature=0;
  r.receive(power::generatorFrame(r.codec,invalid),101.12,101.13);r.capture("power_metadata_age_split");
  r.host.time=103.2;r.capture("power_metadata_reset");
 }
 {Rig r(path);r.system(system(),100.1,100.1);r.capture("unbound");r.bind();r.system(system(),100.12,100.30);r.capture("system");r.host.time=100.9;r.capture("same_sample");r.plugin->configure("status_timeout",".5");r.heartbeat(101,101);r.capture("stale_family");}
 {Rig r(path);r.bind();auto p=system();p.voltage_battery=UINT16_MAX;p.current_battery=-1;p.battery_remaining=-1;p.load=1001;p.drop_rate_comm=10001;r.system(p,100.2,100.2);r.capture("system_unknown");}
 {
  Rig r(path);r.bind();auto p=system();
  p.onboard_control_sensors_present|=MAV_SYS_STATUS_PREARM_CHECK;
  p.onboard_control_sensors_enabled|=MAV_SYS_STATUS_PREARM_CHECK;
  r.system(p,100.2,100.2);r.capture("prearm_unhealthy");
  p.onboard_control_sensors_health|=MAV_SYS_STATUS_PREARM_CHECK;
  r.system(p,100.3,100.3);r.capture("prearm_healthy");
  p.onboard_control_sensors_enabled&=~MAV_SYS_STATUS_PREARM_CHECK;
  r.system(p,100.4,100.4);r.capture("prearm_disabled");
 }
 {Rig r(path);r.bind();r.system(system(),100.2,100.2);r.battery(battery(0),100.21,100.21);r.battery(battery(4),100.22,100.22);r.capture("multi_battery");}
 {Rig r(path);r.bind();auto p=battery();p.temperature=INT16_MAX;p.current_battery=-1;p.current_consumed=p.energy_consumed=-1;p.battery_remaining=-1;p.time_remaining=0;for(unsigned i=0;i<10;++i)p.voltages[i]=UINT16_MAX;r.battery(p,100.2,100.2);r.capture("battery_unknown");p.battery_remaining=101;p.current_consumed=-2;p.energy_consumed=-2;p.time_remaining=-1;p.voltages[0]=4000;p.voltages[2]=4000;r.battery(p,100.3,100.3);r.host.time=100.8;r.capture("battery_invalid");}
 {Rig r(path);r.bind();auto p=battery();for(unsigned i=0;i<10;++i)p.voltages[i]=3300;p.voltages_ext[0]=1;p.voltages_ext[1]=0;p.fault_bitmask=3;p.charge_state=MAV_BATTERY_CHARGE_STATE_FAILED;r.battery(p,100.2,100.2);r.capture("battery_extended");}
 {Rig r(path);r.bind();auto p=battery();for(unsigned i=0;i<4;++i)p.voltages_ext[i]=UINT16_MAX;r.battery(p,100.2,100.2);r.capture("battery_ardupilot_unused_extension");p.voltages[0]=65534;p.voltages[1]=12345;p.voltages[2]=UINT16_MAX;r.battery(p,100.3,100.3);r.host.time=100.8;r.capture("battery_split_pack");}
 {Rig r(path);r.bind();for(unsigned id=0;id<9;++id)r.battery(battery(id),100.2+id*.01,100.2+id*.01);r.capture("battery_capacity");r.host.time=101;r.plugin->onReset("test_reset");r.capture("battery_reset");r.heartbeat(101.1,101.1);r.battery(battery(9),101.2,101.2);r.capture("battery_after_reset");}
 {Rig r(path);r.bind();r.gps(gps(),100.2,100.2);r.capture("gps");auto p=gps();p.fix_type=1;p.eph=p.epv=p.vel=p.cog=UINT16_MAX;p.satellites_visible=UINT8_MAX;p.yaw=UINT16_MAX;p.alt_ellipsoid=p.h_acc=p.v_acc=p.vel_acc=p.hdg_acc=0;r.gps(p,100.3,100.3);r.host.time=100.8;r.capture("gps_unknown");p.fix_type=3;p.lat=0;p.lon=0;p.yaw=0;p.vel=0;p.cog=0;r.gps(p,100.9,100.9);r.host.time=101.4;r.capture("gps_zero");}
 {Rig r(path);r.bind();auto p=gps();p.lat=INT32_MAX;p.cog=40000;p.yaw=40000;r.gps(p,100.2,100.2);r.capture("gps_invalid");}
 {Rig r(path);r.bind();r.system(system(),100.2,100.2);r.capture("order_before");auto p=system();p.load=999;r.host.time=100.8;r.plugin->onMessageWithMetadata([&]{mavlink_message_t m{};mavlink_msg_sys_status_encode_status(1,1,&r.codec,&m,&p);return m;}(),{100.15,100.7,"QUEUE_EMPTY_FLOOR"});r.capture("order_after");}
 {
  Rig r(path);r.heartbeat(100.5,100.5);r.system(system(),100.7,100.7);
  mavlink_message_t older{};
  mavlink_msg_heartbeat_pack_status(1,1,&r.codec,&older,2,3,0,4,MAV_STATE_ACTIVE);
  r.receive(older,100.3,101);
  r.host.time=102.35;r.capture("older_heartbeat");
 }
 {Rig r(path);r.bind();auto p=system();mavlink_message_t m{};mavlink_msg_sys_status_encode_status(1,1,&r.codec,&m,&p);for(const auto& t:std::vector<MessageTime>{{NAN,100.2,"QUEUE_EMPTY_FLOOR"},{100.2,NAN,"QUEUE_EMPTY_FLOOR"},{100.3,100.2,"QUEUE_EMPTY_FLOOR"},{100.2,101,"QUEUE_EMPTY_FLOOR"},{100.1,100.2,"READ_ACQUISITION"},{100.2,100.2,"UNKNOWN"},{99,100.2,"QUEUE_EMPTY_FLOOR"}}){r.host.time=100.2;r.plugin->onMessageWithMetadata(m,t);}r.capture("invalid_metadata");}
 {Rig r(path);r.bind();auto p=system();mavlink_message_t base{};mavlink_msg_sys_status_encode_status(1,1,&r.codec,&base,&p);for(unsigned n=0;n<5;++n){auto m=base;switch(n){case 0:m.magic=0;break;case 1:m.len=255;break;case 2:m.len=0;break;case 3:m.incompat_flags=128;break;case 4:m.magic=MAVLINK_STX_MAVLINK1;m.len=MAVLINK_MSG_ID_SYS_STATUS_MIN_LEN-1;break;}r.receive(m,100.2,100.2);}auto unrelated=base;unrelated.msgid=MAVLINK_MSG_ID_ATTITUDE;r.plugin->onMessageWithMetadata(unrelated,{NAN,NAN,"UNKNOWN"});r.capture("wire_shapes");auto truncated=base;truncated.len=1;r.receive(truncated,100.3,100.3);r.host.time=100.8;r.capture("v2_zero_truncation");auto v1=base;v1.magic=MAVLINK_STX_MAVLINK1;v1.len=MAVLINK_MSG_ID_SYS_STATUS_MIN_LEN;r.receive(v1,100.9,100.9);r.host.time=101.4;r.capture("v1_status");}
 {Rig r(path);r.heartbeat(99.9,100.1);r.system(system(),100.11,100.11);r.capture("preconstruction_floor");r.heartbeat(100.2,100.2);r.system(system(),100.3,100.3);r.capture("post_boundary");}
 {Rig r(path);r.bind();r.system(system(),100.2,100.2);r.capture("source_before");mavlink_message_t p{};auto data=system();mavlink_msg_sys_status_encode_status(2,1,&r.codec,&p,&data);r.receive(p,100.3,100.3);r.host.time=100.8;r.capture("foreign_source");r.host.time=102.2;r.plugin->onInputBoundary();r.capture("heartbeat_lapse");r.heartbeat(102.3,102.3);r.capture("heartbeat_only");}
 {Rig r(path);r.bind();r.system(system(),100.2,100.2);r.host.time=99;r.capture("clock_fault");r.host.time=101;r.plugin->onReset("recovered_clock");r.heartbeat(101.1,101.1);r.system(system(),101.2,101.2);r.capture("clock_recovery");}
 {Rig r(path);r.bind();r.text(0,0,"quotes \"slash\\ newline\n\t",100.2);r.capture("text_ascii");r.text(0,0,std::string("caf\xc3\xa9"),100.3);r.capture("text_utf8");r.text(0,0,std::string("bad\xff",4),100.4);r.capture("text_bad_utf8");}
 {Rig r(path);r.bind();std::string controls;for(unsigned c=1;c<32;++c)controls+=char(c);controls+="\"\\";r.text(0,0,controls,100.2);r.capture("text_controls");r.text(0,0,std::string("\xc0\x80\xed\xa0\x80\xf4\x90\x80\x80",9),100.3);r.capture("text_invalid_sequences");r.text(0,0,std::string(50,'z'),100.4,255);r.capture("text_full_single");r.text(66,0,std::string(50,'x'),100.5);r.text(66,1,"",100.6);r.capture("text_empty_terminator");}
 {Rig r(path);r.bind();r.text(42,0,std::string(49,'a')+char(0xc3),100.2);r.text(42,1,std::string(1,char(0xa9))+"done",100.3);r.capture("text_assembled_utf8");r.host.time=100.9;r.capture("text_not_republished");}
 {Rig r(path);r.bind();r.text(7,0,std::string(50,'a'),100.2);r.text(7,0,std::string(50,'a'),100.4);r.text(7,1,"end",100.5);r.capture("text_duplicate");r.text(8,0,std::string(50,'a'),100.6);r.text(8,2,"gap",100.7);r.capture("text_gap");r.text(9,1,"missing",100.8);r.capture("text_missing");}
 {Rig r(path);r.bind();r.text(10,0,std::string(50,'a'),100.2);r.text(10,0,std::string(50,'b'),100.3);r.capture("text_conflicting");r.text(11,0,std::string(50,'a'),100.4);r.text(11,1,"changed",100.5,3);r.capture("text_severity");}
 {Rig r(path);r.bind();r.plugin->configure("status_text_timeout",".5");r.text(12,0,std::string(50,'a'),100.2);r.text(12,0,std::string(50,'a'),100.6);r.host.time=100.71;r.capture("text_expired");r.text(12,1,"late",100.8);r.capture("text_late_continuation");}
 {Rig r(path);r.bind();for(unsigned id=1;id<=5;++id)r.text(id,0,std::string(50,'a'),100.2+id*.01);r.capture("text_capacity");r.host.time=100.8;r.plugin->onReset("partial_text_reset");r.capture("text_reset");}
 {Rig r(path);r.bind();for(unsigned sequence=0;sequence<16;++sequence)r.text(50,sequence,std::string(50,'x'),100.2+sequence*.01);r.capture("text_chunk_bound");}
 {Rig r(path);r.bind();for(unsigned n=0;n<25;++n)r.text(0,0,"event"+std::to_string(n),100.2+n*.01);r.capture("event_capacity");}
 {Rig r(path);r.bind();r.plugin->configure("status_publish_interval",".5");r.system(system(),100.2,100.2);r.capture("interval_initial");r.system(system(),100.3,100.3);r.capture("interval_suppressed");r.host.time=100.8;r.capture("interval_later");}
 {Rig r(path);r.bind();r.plugin->configure("status_text_timeout",".25");mavlink_statustext_t p{};p.id=78;std::memset(p.text,'a',50);mavlink_message_t m{};mavlink_msg_statustext_encode_status(1,1,&r.codec,&m,&p);r.receive(m,100.2,100.8);r.capture("text_acquisition_expired");}
 {Rig r(path);r.bind();uint32_t random=0x428397u;const uint32_t ids[]={MAVLINK_MSG_ID_SYS_STATUS,MAVLINK_MSG_ID_BATTERY_STATUS,MAVLINK_MSG_ID_GPS_RAW_INT,MAVLINK_MSG_ID_STATUSTEXT};for(unsigned n=0;n<2048;++n){auto next=[&](){random=random*1664525u+1013904223u;return random;};mavlink_message_t m{};m.magic=(n%11==0)?MAVLINK_STX_MAVLINK1:MAVLINK_STX;m.msgid=ids[n%4];m.len=next()%256;m.sysid=1;m.compid=1;m.incompat_flags=(n%13==0)?128:0;auto payload=reinterpret_cast<unsigned char*>(m.payload64);for(std::size_t i=0;i<sizeof(m.payload64);++i)payload[i]=next()>>24;const double at=100.2+n*.0001;r.receive(m,at,at);if(n%127==126){r.capture("bounded_random_frames");}}r.host.time=101.1;r.capture("bounded_random_frames");}
 {Rig r(path);r.bind();for(const auto& key:{"status_timeout","status_publish_interval","status_text_timeout"})for(const auto& value:{"nan","inf"," 1","0","100","1x"}){bool rejected=false;try{r.plugin->configure(key,value);}catch(...){rejected=true;}require(rejected,"invalid config accepted");}r.capture("config_rejections");}
 // Direct Services counterexample: a reset must retain the reliable101 s high-water.
 {Rig r(path);r.bind();r.heartbeat(100.8,100.8);r.system(system(),101,101);r.capture("hardening_reliable");
  r.host.time=100.9;r.plugin->onInputBoundary();r.plugin->onReset("behind_highwater");r.capture("hardening_early_reset");
  r.heartbeat(100.95,100.95);r.system(system(),100.96,100.96);r.host.time=101.1;r.capture("hardening_input_before_highwater");
  r.heartbeat(101.9,101.9);r.system(system(),101.95,101.95);r.host.time=102;r.capture("hardening_catchup_without_reset");
  r.host.time=102.1;r.plugin->onReset("explicit_numeric_recovery");r.capture("hardening_valid_reset");
  r.heartbeat(102,102.2);r.system(system(),102.05,102.25);r.host.time=102.3;r.capture("hardening_old_floor");
  r.heartbeat(102.4,102.4);r.system(system(),102.45,102.45);r.host.time=102.5;r.capture("hardening_recovered");}
 {unsigned index=0;for(double bad:{-1.0,double(NAN),double(INFINITY),100.9}){Rig r(path);r.bind();r.heartbeat(100.8,100.8);r.system(system(),101,101);r.plugin->onInputBoundary();
  r.host.time=bad;r.plugin->onReset("invalid_numeric_reset");r.host.time=101.1;
  const auto label="hardening_invalid_reset_"+std::to_string(index++);r.capture(label.c_str());}}
 {Rig r(path);r.bind();r.heartbeat(100.8,100.8);r.system(system(),101,101);r.plugin->onInputBoundary();
  r.host.time=101;r.plugin->onReset("equal_highwater");r.heartbeat(101.1,101.1);r.system(system(),101.2,101.2);r.capture("hardening_equal_reset");}
 mavmoos::test::gnss::run<Rig>(path);
 std::cout<<"{\"case\":\"driver_complete\",\"key\":\"summary\",\"value\":{\"effects\":0,\"abi\":"<<plugin_abi<<"}}\n";
 return 0;
}catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}}
