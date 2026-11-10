#include "Plugin.h"
#include "VibrationFrames.h"
#include <algorithm>
#include <cmath>
#include <dlfcn.h>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

using namespace mavmoos::runtime;
using mavmoos::test::imu::vibration;
namespace {
void require(bool value,const std::string& why){if(!value)throw std::runtime_error(why);}
struct Host final:Services {
  double time=100;Vehicle state;unsigned effects=0;
  std::vector<std::pair<std::string,std::string>> published;
  Host(){state.type=MAV_TYPE_QUADROTOR;state.autopilot=MAV_AUTOPILOT_ARDUPILOTMEGA;}
  double now()const override{return time;}
  const Vehicle& vehicle()const override{return state;}
  void publish(const std::string& key,const std::string& value)override{published.emplace_back(key,value);}
  void publish(const std::string&,double)override{throw std::runtime_error("IMU wrote a scalar navigation value");}
  bool sendCommand(const CommandRequest&)override{++effects;throw std::runtime_error("IMU sent command");}
  bool acquireNavigation(const std::string&)override{++effects;throw std::runtime_error("IMU acquired navigation");}
  void revokeNavigation(const std::string&)override{++effects;throw std::runtime_error("IMU revoked navigation");}
  bool navigation(const NavigationTarget&)override{++effects;throw std::runtime_error("IMU sent navigation");}
  bool parameter(const ParameterRequest&)override{++effects;throw std::runtime_error("IMU requested parameter");}
  bool mission(const MissionRequest&)override{++effects;throw std::runtime_error("IMU requested mission");}
};
CreatePlugin create=nullptr;DestroyPlugin destroy=nullptr;
struct Rig {
  Host host;Plugin* plugin=nullptr;mavlink_status_t codec{};
  explicit Rig(double initial=100){host.time=initial;plugin=create(plugin_abi,&host);require(plugin,"factory rejected current ABI");}
  ~Rig(){destroy(plugin);}
  void receive(const mavlink_message_t& m,double effective,double read,const std::string& basis="READ_ACQUISITION")
  {host.time=read;plugin->onMessageWithMetadata(m,{effective,read,basis});}
  void heartbeat(double at=100.1,unsigned system=1,unsigned component=1,unsigned type=MAV_TYPE_QUADROTOR,unsigned autopilot=MAV_AUTOPILOT_ARDUPILOTMEGA,double read=-1)
  {
    if(read<0)read=at;
    if(system==1&&component==1&&type==host.state.type&&autopilot==host.state.autopilot){host.state.heartbeat_valid=true;host.state.heartbeat_at=at;}
    mavlink_heartbeat_t p{};p.type=type;p.autopilot=autopilot;mavlink_message_t m{};
    mavlink_msg_heartbeat_encode_status(system,component,&codec,&m,&p);receive(m,at,read,read==at?"READ_ACQUISITION":"QUEUE_EMPTY_FLOOR");
  }
  void capture(const std::string& label)
  {
    host.published.clear();plugin->iterate();
    for(const auto& p:host.published)std::cout<<"{\"case\":\""<<label<<"\",\"key\":\""<<p.first<<"\",\"value\":"<<p.second<<"}\n";
    require(host.effects==0,"effectful Services used");
  }
};
mavlink_message_t sample(Rig& r,unsigned family=MAVLINK_MSG_ID_RAW_IMU,unsigned id=0,uint64_t stamp=123456789,int16_t temperature=2350)
{
  mavlink_message_t m{};
  if(family==MAVLINK_MSG_ID_RAW_IMU){mavlink_raw_imu_t p{};p.time_usec=stamp;p.id=id;p.temperature=temperature;
    p.xacc=1000;p.yacc=-2000;p.zacc=0;p.xgyro=1000;p.ygyro=-2000;p.zgyro=125;p.xmag=1000;p.ymag=-250;p.zmag=0;
    mavlink_msg_raw_imu_encode_status(1,1,&r.codec,&m,&p);
  }else if(family==MAVLINK_MSG_ID_SCALED_IMU){mavlink_scaled_imu_t p{};p.time_boot_ms=stamp;p.temperature=temperature;
    p.xacc=1000;p.yacc=-2000;p.zacc=0;p.xgyro=1000;p.ygyro=-2000;p.zgyro=125;p.xmag=1000;p.ymag=-250;p.zmag=0;
    mavlink_msg_scaled_imu_encode_status(1,1,&r.codec,&m,&p);
  }else if(family==MAVLINK_MSG_ID_SCALED_IMU2){mavlink_scaled_imu2_t p{};p.time_boot_ms=stamp;p.temperature=temperature;
    p.xacc=1000;p.yacc=-2000;p.zacc=0;p.xgyro=1000;p.ygyro=-2000;p.zgyro=125;p.xmag=1000;p.ymag=-250;p.zmag=0;
    mavlink_msg_scaled_imu2_encode_status(1,1,&r.codec,&m,&p);
  }else{mavlink_scaled_imu3_t p{};p.time_boot_ms=stamp;p.temperature=temperature;
    p.xacc=1000;p.yacc=-2000;p.zacc=0;p.xgyro=1000;p.ygyro=-2000;p.zgyro=125;p.xmag=1000;p.ymag=-250;p.zmag=0;
    mavlink_msg_scaled_imu3_encode_status(1,1,&r.codec,&m,&p);
  }return m;
}

mavlink_message_t vibrationValue(Rig& rig,float level)
{
  mavlink_vibration_t packet{};
  packet.time_usec=1;
  packet.vibration_x=level;
  mavlink_message_t message{};
  mavlink_msg_vibration_encode_status(1,1,&rig.codec,&message,&packet);
  return message;
}
}
int main(int argc,char** argv){try{
  require(argc==2,"usage: imu_scenario_driver PLUGIN");
  void* library=dlopen(argv[1],RTLD_NOW|RTLD_LOCAL);if(!library)throw std::runtime_error(dlerror());
  create=reinterpret_cast<CreatePlugin>(dlsym(library,"mavmoos_create_plugin"));destroy=reinterpret_cast<DestroyPlugin>(dlsym(library,"mavmoos_destroy_plugin"));
  require(create&&destroy,"plugin symbols missing");Host abi_host;require(!create(plugin_abi-1,&abi_host)&&!create(plugin_abi,nullptr),"factory accepted wrong ABI/null Services");
  {Rig r;require(r.plugin->name()=="imu"&&r.plugin->subscriptions().empty()&&!r.plugin->controlBackend(),"unexpected public control interface");
   const auto d=r.plugin->descriptor();require(d.profile=="ardupilot_raw_scaled_imu_observations"&&d.exclusive_resources==std::vector<std::string>{"observation.imu"}&&d.streamed_parameter_bench_compatible,"descriptor mismatch");
   require(d.capabilities.size()==5&&d.capabilities.back().name=="imu.vibration"&&d.observation_streams.size()==4,"descriptor size");const std::vector<unsigned> ids={27,26,116,129};
   for(unsigned n=0;n<4;++n)require(d.observation_streams[n].message_id==ids[n]&&d.observation_streams[n].interval_us==200000,"stream demand");
   for(const auto& cap:d.capabilities)require(cap.access=="observation"&&cap.request_variable.empty(),"effectful capability");
   r.capture("absent");auto m=sample(r);r.receive(m,100.05,100.05);r.host.time=100.1;r.capture("unbound");r.heartbeat();r.receive(m,100.12,100.3,"QUEUE_EMPTY_FLOOR");r.capture("raw");r.host.time=100.9;r.capture("cached");r.host.time=101.31;r.capture("stale");}
  {Rig r;r.heartbeat();r.receive(sample(r),100.2,100.2);r.capture("raw_first");r.receive(sample(r,26),100.3,100.3);r.receive(sample(r,116),100.4,100.4);r.receive(sample(r,129),100.5,100.5);r.capture("four_families");r.host.time=101.31;r.capture("independent_ages");}
  {Rig r;r.heartbeat();r.receive(sample(r,27,0),100.2,100.2);r.receive(sample(r,27,3),100.3,100.3);r.capture("raw_identities");}
  for(unsigned family:{27u,26u,116u,129u})for(int temperature:{0,1,-4000,32767}){
    Rig r;r.heartbeat();r.receive(sample(r,family,0,1234,temperature),100.2,100.2);r.capture("temperature_"+std::to_string(family)+"_"+std::to_string(temperature));
  }
  for(unsigned family:{27u,26u,116u,129u}){Rig r;r.heartbeat();auto m=sample(r,family,7,1234,2350);m.magic=MAVLINK_STX_MAVLINK1;m.len=family==27?MAVLINK_MSG_ID_RAW_IMU_MIN_LEN:MAVLINK_MSG_ID_SCALED_IMU_MIN_LEN;r.receive(m,100.2,100.2);r.capture("v1_"+std::to_string(family));}
  {Rig r;r.heartbeat();auto m=sample(r);m.len=1;r.receive(m,100.2,100.2);r.capture("v2_zero_tail");}
  {Rig r;r.heartbeat();auto m=sample(r,27,0,std::numeric_limits<uint64_t>::max());r.receive(m,100.2,100.2);r.capture("max_remote_stamp");r.receive(sample(r,27,0,1),100.4,100.4);r.capture("remote_clock_regression");}
  {Rig r;r.heartbeat();r.receive(sample(r,26,0,std::numeric_limits<uint32_t>::max()),100.2,100.2);r.receive(sample(r,26,0,0),100.3,100.3);r.capture("remote_ms_wrap");}
  {Rig r;r.heartbeat();auto m=sample(r);r.receive(m,100.3,100.5,"QUEUE_EMPTY_FLOOR");r.capture("before_order_reject");r.receive(sample(r,27,0,999),100.2,100.6,"QUEUE_EMPTY_FLOOR");r.host.time=100.7;r.plugin->onMessageWithMetadata(sample(r,27,0,888),{100.4,100.45,"QUEUE_EMPTY_FLOOR"});r.host.time=100.8;r.capture("order_reject");}
  {Rig r;r.heartbeat();auto m=sample(r);for(const auto& t:std::vector<MessageTime>{{NAN,100.2,"READ_ACQUISITION"},{100.2,NAN,"QUEUE_EMPTY_FLOOR"},{100.3,100.2,"QUEUE_EMPTY_FLOOR"},{100.1,100.2,"READ_ACQUISITION"},{100.2,100.2,"UNKNOWN"},{100,100.2,"QUEUE_EMPTY_FLOOR"},{100.4,100.4,"READ_ACQUISITION"}}){r.host.time=100.3;r.plugin->onMessageWithMetadata(m,t);}r.host.time=100.4;r.capture("metadata_reject");}
  {Rig r;r.heartbeat();auto m=sample(r);for(unsigned n=0;n<5;++n){auto bad=m;if(n==0)bad.magic=0;if(n==1)bad.len=0;if(n==2)bad.len=255;if(n==3)bad.incompat_flags=128;if(n==4){bad.magic=MAVLINK_STX_MAVLINK1;bad.len=25;}r.receive(bad,100.2,100.2);}r.capture("wire_reject");}
  {Rig r;r.heartbeat();auto m=sample(r);r.receive(m,100.2,100.2);m.sysid=2;r.receive(m,100.3,100.3);m.sysid=1;m.compid=200;r.receive(m,100.4,100.4);r.heartbeat(100.5,2);r.heartbeat(100.6,1,200);r.heartbeat(100.7,1,1,MAV_TYPE_GROUND_ROVER);r.heartbeat(100.8,1,1,MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_PX4);r.capture("foreign_sources");}
  {Rig r;r.heartbeat(100.1,2);r.receive(sample(r),100.2,100.2);r.capture("foreign_first_heartbeat");}
  {Rig r;r.heartbeat(99.9,1,1,MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_ARDUPILOTMEGA,100.1);r.receive(sample(r),100.2,100.2);r.capture("preconstruction_heartbeat");r.heartbeat(100.3);r.receive(sample(r),100.4,100.4);r.capture("postfloor_bind");}
  {Rig r;r.heartbeat();r.receive(sample(r),100.2,100.2);r.host.time=100.8;r.plugin->onReset("quotes\" newline\n reset");r.capture("reset");r.heartbeat(100.7,1,1,MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_ARDUPILOTMEGA,100.9);r.receive(sample(r),100.75,101,"QUEUE_EMPTY_FLOOR");r.capture("old_floor_after_reset");r.heartbeat(101.1);r.capture("rebound_empty");r.receive(sample(r),101.3,101.3);r.capture("new_after_reset");}
  {Rig r;r.heartbeat();r.receive(sample(r),100.2,100.2);r.host.time=102.2;r.plugin->onInputBoundary();r.capture("heartbeat_lapse");r.heartbeat(102.3);r.capture("lapse_rebound");}
  {Rig r;r.heartbeat();r.receive(sample(r),100.2,100.2);r.host.time=100.3;r.host.state.heartbeat_valid=false;r.plugin->onInputBoundary();r.capture("host_heartbeat_invalid");}
  {Rig r;r.heartbeat();r.heartbeat(100.8);r.receive(sample(r),101,101);r.host.time=100.9;r.plugin->onInputBoundary();r.plugin->onReset("behind_highwater");r.host.time=101.2;r.capture("clock_latched");r.heartbeat(101.3);r.receive(sample(r),101.4,101.4);r.capture("clock_not_auto_recovered");r.host.time=101.5;r.plugin->onReset("numeric_recovery");r.heartbeat(101.6);r.receive(sample(r),101.7,101.7);r.capture("clock_explicit_recovery");}
  for(double bad:{-1.,double(NAN),double(INFINITY)}){Rig r;r.heartbeat();r.receive(sample(r),101,101);r.host.time=bad;r.plugin->onReset("invalid_reset");r.host.time=101.2;r.capture("invalid_reset");}
  for(double bad:{-1.,double(NAN),double(INFINITY)}){Rig r(bad);r.host.time=100;r.capture("invalid_initial_clock");r.plugin->onReset("valid_initial_recovery");r.heartbeat();r.receive(sample(r),100.2,100.2);r.capture("initial_clock_explicit_recovery");}
  for(double bad:{double(NAN),double(INFINITY)}){Rig r;r.heartbeat();r.receive(sample(r),100.2,100.2);r.host.time=bad;r.plugin->iterate();r.host.time=100.3;r.capture("nonfinite_owner_clock");}
  {Rig r;r.heartbeat();for(unsigned n=0;n<8;++n)r.receive(sample(r,27,n),100.2+n*.01,100.2+n*.01);r.capture("capacity_eight");r.receive(sample(r,27,8),100.4,100.4);r.receive(sample(r,26),100.5,100.5);r.capture("capacity_reject");r.receive(sample(r,27,0,999),100.7,100.7);r.capture("capacity_existing_update");}
  {Rig r;r.heartbeat();for(const auto& key:{"imu_timeout","imu_publish_interval"})for(const auto& value:{"nan","inf"," 1","0","100","1x"}){bool rejected=false;try{r.plugin->configure(key,value);}catch(...){rejected=true;}require(rejected,"invalid configuration accepted");}r.plugin->configure("imu_timeout",".25");r.receive(sample(r),100.2,100.2);r.host.time=100.451;r.capture("short_timeout");}
  {Rig r;r.heartbeat();r.plugin->configure("imu_publish_interval",".5");r.receive(sample(r),100.2,100.2);r.capture("interval_first");r.receive(sample(r,26),100.3,100.3);r.capture("interval_suppressed");r.host.time=100.71;r.capture("interval_later");}
  {Rig r;r.heartbeat();r.receive(sample(r),100.2,100.2);r.host.time=101.2;r.capture("age_exact_limit");}
  {Rig r;r.heartbeat();r.receive(sample(r),100.2,100.2);r.host.time=101.2001;r.capture("age_over_limit");}
  {Rig r;r.host.state.type=MAV_TYPE_GROUND_ROVER;r.heartbeat(100.1,1,1,MAV_TYPE_GROUND_ROVER);r.receive(sample(r),100.2,100.2);r.capture("rover_binding");}
  {Rig r;r.heartbeat();auto m=sample(r);m.msgid=MAVLINK_MSG_ID_HIGHRES_IMU;r.plugin->onMessageWithMetadata(m,{NAN,NAN,"UNKNOWN"});r.capture("deferred_highres_ignored");}
  {
    Rig r;
    r.receive(vibration(r.codec),100.05,100.05);
    r.capture("vibration_unbound");
    r.heartbeat(100.1);
    r.receive(sample(r),100.15,100.15);
    r.receive(vibration(r.codec),100.2,100.35,"QUEUE_EMPTY_FLOOR");
    r.capture("vibration_valid");
    r.host.time=100.9;
    r.capture("vibration_cached");
    r.receive(sample(r),101.3,101.3);
    r.capture("vibration_stale");
    r.receive(vibrationValue(r,1.f),101.5,101.5);
    r.capture("vibration_remote_regression");
  }
  {
    Rig r;
    r.heartbeat();
    r.receive(vibration(r.codec),100.2,100.3,"QUEUE_EMPTY_FLOOR");
    r.capture("vibration_before_rejected");
    auto foreign=vibrationValue(r,9.f);
    foreign.sysid=2;
    r.receive(foreign,100.4,100.4);
    foreign.sysid=1;foreign.compid=2;
    r.receive(foreign,100.5,100.5);
    r.receive(vibrationValue(r,9.f),100.15,100.6,"QUEUE_EMPTY_FLOOR");
    r.host.time=100.7;
    r.plugin->onMessageWithMetadata(vibrationValue(r,9.f),{100.25,100.26,"QUEUE_EMPTY_FLOOR"});
    r.heartbeat(100.8);
    r.receive(vibrationValue(r,9.f),100.75,100.9,"QUEUE_EMPTY_FLOOR");
    for(float level:{NAN,INFINITY,-1.f})r.receive(vibrationValue(r,level),101.,101.);
    auto malformed=vibration(r.codec);
    malformed.len=0;r.receive(malformed,101.1,101.1);
    malformed.len=MAVLINK_MSG_ID_VIBRATION_LEN+1;r.receive(malformed,101.2,101.2);
    malformed.len=MAVLINK_MSG_ID_VIBRATION_LEN;malformed.incompat_flags=128;r.receive(malformed,101.3,101.3);
    malformed.magic=MAVLINK_STX_MAVLINK1;malformed.len=MAVLINK_MSG_ID_VIBRATION_MIN_LEN-1;
    r.receive(malformed,101.4,101.4);
    r.capture("vibration_after_rejected");
  }
  {
    Rig r;
    r.heartbeat();
    r.codec.flags|=MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    r.receive(vibration(r.codec),100.2,100.2);
    r.capture("vibration_v1");
    r.codec.flags&=~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    r.receive(vibrationValue(r,0),100.4,100.4);
    r.capture("vibration_trimmed_v2");
    r.host.time=102.2;
    r.plugin->onInputBoundary();
    r.capture("vibration_heartbeat_lapse");
    r.heartbeat(102.3);
    r.receive(vibration(r.codec),102.4,102.4);
    r.capture("vibration_rebound");
    r.host.time=102.6;
    r.plugin->onReset("explicit observer reset");
    r.capture("vibration_reset");
    r.receive(vibration(r.codec),102.7,102.7);
    r.host.time=102.8;
    r.capture("vibration_rebind_required");
    r.heartbeat(102.9);
    r.receive(vibration(r.codec),103.0,103.0);
    r.capture("vibration_recovered");
  }
  {Rig r;r.heartbeat();uint32_t random=0x9876u;for(unsigned n=0;n<2048;++n){auto next=[&](){random=random*1664525u+1013904223u;return random;};auto m=sample(r,n%4==0?27:n%4==1?26:n%4==2?116:129);m.magic=n%11==0?MAVLINK_STX_MAVLINK1:MAVLINK_STX;m.len=next()%40;m.incompat_flags=n%13==0?128:0;auto bytes=reinterpret_cast<unsigned char*>(m.payload64);for(std::size_t k=0;k<sizeof(m.payload64);++k)bytes[k]=next()>>24;const double at=100.2+n*.0001;r.receive(m,at,at);}r.host.time=100.9;r.capture("bounded_random_frames");}
  require(dlclose(library)==0,"dlclose failed");
  std::cout<<"{\"case\":\"driver_complete\",\"key\":\"summary\",\"value\":{\"effects\":0,\"abi\":"<<plugin_abi<<",\"scope\":\"actual DSO and fake Services; no sockets or native\"}}\n";
  return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
