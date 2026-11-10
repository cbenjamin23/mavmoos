/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: DistanceSensorScenarioDriver.cpp                */
/*    DATE: November 4, 2026                                */
/************************************************************/
// Actual MODULE/factory, decoded/generated MAVLink and fake Services; no sockets.
#include "Plugin.h"
#include <cmath>
#include <cstring>
#include <dlfcn.h>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace mavmoos::runtime;
namespace {
void require(bool value,const char* text){if(!value)throw std::runtime_error(text);}
struct Host final:Services {
 double time=100;Vehicle state;unsigned effects=0;std::vector<std::pair<std::string,std::string>> publications;
 Host(){state.type=2;state.autopilot=3;state.heartbeat_valid=true;state.heartbeat_at=time;}
 double now()const override{return time;}const Vehicle& vehicle()const override{return state;}
 void publish(const std::string& key,const std::string& value)override{require(key=="MAVMOOS_DISTANCE_SENSORS"||key=="MAVMOOS_DISTANCE_STATUS"||key=="MAVMOOS_OBSTACLE_DISTANCE","unexpected publication");require(value.size()<100000,"publication bound");publications.emplace_back(key,value);}
 void publish(const std::string&,double)override{throw std::runtime_error("numeric/NAV effect");}
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
 bool controlBusy()const override{throw std::runtime_error("busy query");}
 bool projectGlobal(double,double,double&,double&)const override{throw std::runtime_error("projection query");}
};
struct Rig {
 Host host;void* handle=nullptr;Plugin* plugin=nullptr;DestroyPlugin destroy=nullptr;mavlink_status_t codec{};
 explicit Rig(const char* path,double initial=100){host.time=initial;handle=dlopen(path,RTLD_NOW|RTLD_LOCAL|RTLD_NODELETE);require(handle,"dlopen");auto create=reinterpret_cast<CreatePlugin>(dlsym(handle,"mavmoos_create_plugin"));destroy=reinterpret_cast<DestroyPlugin>(dlsym(handle,"mavmoos_destroy_plugin"));require(create&&destroy,"exports");require(!create(plugin_abi-1,&host)&&!create(plugin_abi+1,&host)&&!create(plugin_abi,nullptr),"ABI/context");plugin=create(plugin_abi,&host);require(plugin&&plugin->name()=="distance_sensor","factory");require(!host.effects&&host.publications.empty(),"factory effects");plugin->configure("distance_sensor_publish_interval",".02");}
 ~Rig(){if(plugin)destroy(plugin);if(handle)dlclose(handle);if(host.effects)std::terminate();}
 void metadata(const mavlink_message_t& m,const MessageTime& time){plugin->onMessageWithMetadata(m,time);}
 void receive(const mavlink_message_t& m,double effective,double read,const std::string& basis="QUEUE_EMPTY_FLOOR"){host.time=read;metadata(m,{effective,read,basis});}
 void heartbeat(double effective,double read,unsigned system=1,unsigned component=1,unsigned type=2,unsigned autopilot=3){mavlink_message_t m{};host.state.heartbeat_valid=true;host.state.heartbeat_at=effective;mavlink_msg_heartbeat_pack_status(system,component,&codec,&m,type,autopilot,0,4,MAV_STATE_ACTIVE);receive(m,effective,read);}
 void bind(){heartbeat(100.1,100.1);}
 mavlink_message_t frame(const mavlink_distance_sensor_t& p){mavlink_message_t m{};mavlink_msg_distance_sensor_encode_status(1,1,&codec,&m,&p);return m;}
 void sensor(const mavlink_distance_sensor_t& p,double effective,double read){receive(frame(p),effective,read);}
 void capture(const char* label){plugin->iterate();for(const auto& item:host.publications)std::cout<<"{\"case\":\""<<label<<"\",\"key\":\""<<item.first<<"\",\"value\":"<<item.second<<"}\n";host.publications.clear();require(!host.effects,"observer effect");}
};
mavlink_distance_sensor_t sensor(unsigned id=0){mavlink_distance_sensor_t p{};p.time_boot_ms=1234;p.min_distance=20;p.max_distance=1000;p.current_distance=432;p.type=0;p.id=id;p.orientation=25;p.covariance=25;p.horizontal_fov=.5f;p.vertical_fov=.4f;p.quaternion[0]=1;p.signal_quality=75;return p;}
}
int main(int argc,char** argv){try{
 require(argc==2,"usage: distance_sensor_scenario_driver DISTANCE_MODULE");const char* path=argv[1];
 {Rig r(path);const auto d=r.plugin->descriptor();require(d.profile=="ardupilot_distance_observations"&&d.capabilities.size()==2&&d.capabilities[0].name=="distance.observation"&&d.capabilities[0].access=="observation"&&d.capabilities[1].name=="distance.obstacle_sectors"&&d.capabilities[1].access=="observation"&&d.exclusive_resources==std::vector<std::string>{"observation.distance_sensor"}&&d.request_owners.empty()&&r.plugin->subscriptions().empty()&&!r.plugin->controlBackend()&&d.streamed_parameter_bench_compatible,"descriptor");require(d.observation_streams.size()==1&&d.observation_streams[0].message_id==132&&d.observation_streams[0].interval_us==100000,"streams");r.plugin->onMail({"MAVMOOS_RANGE_INJECT","anything","test"});r.capture("absent");}
 {Rig r(path);r.sensor(sensor(),100.02,100.02);r.capture("unbound");r.bind();r.sensor(sensor(),100.12,100.3);r.capture("normal");r.host.time=100.9;r.capture("cached");r.heartbeat(101,101);r.sensor(sensor(1),101.2,101.2);r.capture("independent_ages");}
 {Rig r(path);r.bind();unsigned n=0;for(unsigned id:{255u,0u,7u}){r.sensor(sensor(id),100.2+n*.03,100.2+n*.03);++n;}r.capture("ids_sorted");auto p=sensor(0);p.current_distance=555;r.sensor(p,100.5,100.5);r.capture("id_update");}
 {Rig r(path);r.bind();for(unsigned i=0;i<8;++i)r.sensor(sensor(i),100.2+i*.001,100.2+i*.001);r.capture("full");r.sensor(sensor(255),100.3,100.3);r.sensor(sensor(254),100.4,100.4);r.capture("capacity");auto p=sensor(3);p.current_distance=999;r.sensor(p,100.5,100.5);r.capture("retained_update");r.host.time=100.6;r.plugin->onReset("capacity_reset");r.heartbeat(100.7,100.7);r.sensor(sensor(255),100.8,100.8);r.capture("capacity_reset");}
 {Rig r(path);r.bind();auto p=sensor();p.min_distance=100;p.max_distance=50;p.current_distance=75;r.sensor(p,100.2,100.2);r.capture("reversed_bounds");p=sensor();p.current_distance=10;r.sensor(p,100.3,100.3);r.capture("below_min");p.current_distance=2000;r.sensor(p,100.4,100.4);r.capture("above_max");p.min_distance=p.max_distance=p.current_distance=0;r.sensor(p,100.5,100.5);r.capture("zero_distance");p.max_distance=p.current_distance=65535;r.sensor(p,100.6,100.6);r.capture("max_distance");}
 {Rig r(path);r.bind();unsigned i=0;for(unsigned value:{0u,1u,25u,254u,255u}){auto p=sensor();p.covariance=value;const double at=100.2+i++*.03;r.sensor(p,at,at);const auto name="covariance_"+std::to_string(value);r.capture(name.c_str());}}
 {Rig r(path);r.bind();unsigned i=0;for(unsigned value:{0u,1u,2u,100u,101u,255u}){auto p=sensor();p.signal_quality=value;const double t=100.2+i++*.03;r.sensor(p,t,t);const auto name="quality_"+std::to_string(value);r.capture(name.c_str());}}
 {Rig r(path);r.bind();auto p=sensor();p.type=4;p.orientation=100;r.sensor(p,100.2,100.2);r.capture("custom_unit");p.quaternion[0]=0;r.sensor(p,100.3,100.3);r.capture("custom_zero");p.quaternion[0]=2;r.sensor(p,100.4,100.4);r.capture("custom_nonunit");p.quaternion[0]=1.0005f;r.sensor(p,100.5,100.5);r.capture("custom_nearunit");p.quaternion[0]=NAN;r.sensor(p,100.6,100.6);r.capture("custom_nan");p.type=255;p.orientation=255;r.sensor(p,100.7,100.7);r.capture("unknown_enums");}
 {Rig r(path);r.bind();unsigned i=0;for(unsigned orientation:{0u,24u,25u,40u,41u,100u}){auto p=sensor();p.orientation=orientation;const double t=100.2+i++*.03;r.sensor(p,t,t);const auto name="orientation_"+std::to_string(orientation);r.capture(name.c_str());}}
 {Rig r(path);r.bind();auto p=sensor();p.horizontal_fov=0;p.vertical_fov=-1;r.sensor(p,100.2,100.2);r.capture("fov_zero_negative");p.horizontal_fov=NAN;p.vertical_fov=INFINITY;r.sensor(p,100.3,100.3);r.capture("fov_nonfinite");p.horizontal_fov=7;p.vertical_fov=6.283185f;r.sensor(p,100.4,100.4);r.capture("fov_geometric");}
 {Rig r(path);r.bind();auto p=sensor();p.orientation=100;auto m=r.frame(p);m.magic=MAVLINK_STX_MAVLINK1;m.len=MAVLINK_MSG_ID_DISTANCE_SENSOR_MIN_LEN;r.receive(m,100.2,100.2);r.capture("v1_dirty_extensions");m=r.frame(p);m.len=MAVLINK_MSG_ID_DISTANCE_SENSOR_MIN_LEN;r.receive(m,100.3,100.3);r.capture("v2_omitted_extensions");m={};m.magic=MAVLINK_STX;m.sysid=m.compid=1;m.msgid=132;m.len=15;auto raw=reinterpret_cast<unsigned char*>(m.payload64);raw[11]=1;raw[14]=7;r.receive(m,100.4,100.4);r.capture("partial_extension");m.len=1;raw[0]=9;r.receive(m,100.5,100.5);r.capture("v2_zero_trim");}
 {Rig r(path);r.bind();auto p=sensor();p.time_boot_ms=UINT32_MAX;r.sensor(p,100.2,100.2);r.capture("remote_max");p.time_boot_ms=0;r.sensor(p,100.3,100.3);r.capture("remote_wrap");r.sensor(p,100.4,100.4);r.capture("remote_repeat");}
 {Rig r(path);r.bind();r.sensor(sensor(),100.2,100.3);r.capture("order_before");auto p=sensor();p.current_distance=600;r.sensor(p,100.19,100.4);r.metadata(r.frame(p),{100.21,100.29,"QUEUE_EMPTY_FLOOR"});r.host.time=100.5;r.capture("order_after");r.host.time=100.6;r.metadata(r.frame(p),{100.2,100.3,"QUEUE_EMPTY_FLOOR"});r.capture("coalesced");}
 {Rig r(path);r.bind();auto m=r.frame(sensor());r.host.time=100.5;for(auto t:std::vector<MessageTime>{{NAN,100.3,"QUEUE_EMPTY_FLOOR"},{100.2,NAN,"QUEUE_EMPTY_FLOOR"},{100.4,100.3,"QUEUE_EMPTY_FLOOR"},{100,100.3,"QUEUE_EMPTY_FLOOR"},{-1,100.3,"QUEUE_EMPTY_FLOOR"},{100.2,100.6,"QUEUE_EMPTY_FLOOR"},{100.2,100.3,"UNKNOWN"},{100.2,100.3,"READ_ACQUISITION"}})r.metadata(m,t);r.capture("bad_metadata");r.receive(m,100.6,100.6,"READ_ACQUISITION");r.capture("read_acquisition");}
 {Rig r(path);r.bind();r.sensor(sensor(),100.2,100.2);r.capture("source_before");auto m=r.frame(sensor());m.sysid=2;r.receive(m,100.3,100.3);m.sysid=1;m.compid=2;r.receive(m,100.4,100.4);r.heartbeat(100.5,100.5,2);r.heartbeat(100.6,100.6,1,2);r.heartbeat(100.7,100.7,1,1,10);r.heartbeat(100.8,100.8,1,1,2,12);r.capture("foreign_sources");}
 {Rig r(path);r.bind();r.sensor(sensor(),100.2,100.2);r.host.time=102.2;r.capture("heartbeat_lapse");r.heartbeat(102.3,102.3);r.capture("heartbeat_only");r.sensor(sensor(),102.4,102.4);r.host.state.heartbeat_valid=false;r.host.time=102.5;r.capture("host_heartbeat_invalid");}
 {Rig r(path);r.bind();r.sensor(sensor(),100.2,100.2);r.host.time=100.4;r.plugin->onReset("quotes\" newline\n reset");r.capture("reset");r.heartbeat(100.3,100.5);r.sensor(sensor(),100.39,100.6);r.capture("old_floor");r.heartbeat(100.7,100.7);r.capture("rebound_empty");r.sensor(sensor(255),100.8,100.8);r.capture("after_reset");}
 {Rig r(path);r.bind();r.sensor(sensor(),100.2,100.2);r.host.time=101;r.plugin->onInputBoundary();r.capture("highwater_before");r.host.time=100.9;r.plugin->onInputBoundary();r.capture("rollback");r.plugin->onReset("premature");r.host.time=100.95;r.heartbeat(100.94,100.95);r.sensor(sensor(),100.96,100.96);r.capture("premature_reset");r.host.time=101.1;r.capture("caught_up_latched");r.host.time=101.2;r.plugin->onReset("qualified");r.heartbeat(101.3,101.3);r.sensor(sensor(),101.4,101.4);r.capture("clock_recovered");}
 {Rig r(path);r.bind();double good=101;for(double invalid:{double(NAN),double(INFINITY),-1.0}){r.host.time=invalid;r.plugin->onReset("bad");r.host.time=good;good+=.03;r.capture("invalid_reset");}r.host.time=101.2;r.plugin->onReset("qualified");r.heartbeat(101.3,101.3);r.sensor(sensor(),101.4,101.4);r.capture("invalid_reset_recovered");}
 {Rig r(path,NAN);r.host.time=100;r.capture("invalid_start");r.heartbeat(100.1,100.1);r.sensor(sensor(),100.2,100.2);r.capture("invalid_start_latched");r.host.time=100.3;r.plugin->onReset("qualified");r.heartbeat(100.4,100.4);r.sensor(sensor(),100.5,100.5);r.capture("invalid_start_recovered");}
 {Rig r(path);r.bind();auto m=r.frame(sensor());for(unsigned i=0;i<5;++i){auto bad=m;if(i==0)bad.magic=0;if(i==1){bad.magic=MAVLINK_STX_MAVLINK1;bad.len=13;}if(i==2){bad.magic=MAVLINK_STX_MAVLINK1;bad.len=39;}if(i==3)bad.len=40;if(i==4)bad.incompat_flags=128;r.receive(bad,100.2+i*.03,100.2+i*.03);}m.msgid=173;r.receive(m,100.4,100.4);r.capture("bad_wire");}
 {Rig r(path);r.bind();r.plugin->configure("distance_sensor_publish_interval",".2");r.sensor(sensor(),100.2,100.2);r.capture("pacing_first");r.host.time=100.3;r.capture("pacing_suppressed");r.sensor(sensor(1),100.41,100.41);r.capture("pacing_later");for(const auto& key:{"distance_sensor_timeout","distance_sensor_publish_interval"})for(const auto& value:{"nan","inf"," 1","0","100","1x"}){bool rejected=false;try{r.plugin->configure(key,value);}catch(...){rejected=true;}require(rejected,"invalid config");}r.plugin->configure("distance_sensor_timeout",".25");r.host.time=100.7;r.capture("short_timeout");}
 {Rig r(path);r.bind();uint32_t random=0x9843u;for(unsigned i=0;i<2048;++i){auto next=[&](){random=random*1664525u+1013904223u;return random;};mavlink_message_t m{};m.magic=i%11==0?MAVLINK_STX_MAVLINK1:MAVLINK_STX;m.msgid=132;m.len=next()%44;m.sysid=m.compid=1;m.incompat_flags=i%13==0?128:0;auto bytes=reinterpret_cast<unsigned char*>(m.payload64);for(std::size_t n=0;n<sizeof(m.payload64);++n)bytes[n]=next()>>24;const double at=100.2+i*.0001;r.receive(m,at,at);if(i%127==126)r.capture("random_frames");}r.host.time=101;r.capture("random_frames");}
 std::cout<<"{\"case\":\"driver_complete\",\"key\":\"summary\",\"value\":{\"effects\":0,\"abi\":"<<plugin_abi<<",\"random_frames\":2048}}\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
