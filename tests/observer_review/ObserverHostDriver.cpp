/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ObserverHostDriver.cpp                          */
/*    DATE: November 4, 2026                                */
/************************************************************/
// Real Runtime + startup MODULEs; synchronous fake I/O never opens a socket.
#include "Runtime.h"
#include "Telemetry.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
using namespace mavmoos::runtime;
namespace {
std::string telemetry_module,status_module,attitude_module,control_module,parameter_module,interval_fixture;
void require(bool value,const std::string& why){if(!value)throw std::runtime_error(why);}
std::string jsonString(const std::string& text)
{
  static constexpr char hex[]="0123456789abcdef";std::string result="\"";
  for(unsigned char c:text){if(c=='"'||c=='\\'){result+='\\';result+=char(c);}else if(c<32){result+="\\u00";result+=hex[c>>4];result+=hex[c&15];}else result+=char(c);}
  return result+'"';
}
struct Frame{mavlink_message_t message{};double at=0;};
struct Rig {
  double time=100;
  std::map<std::string,std::string> strings;
  std::map<std::string,double> numbers;
  std::map<std::string,double> publication_times;
  std::map<std::string,unsigned> numeric_counts;
  std::vector<std::pair<std::string,std::string>> publications;
  std::vector<Frame> frames;
  Runtime runtime;
  Rig(bool simulation=false,bool observers=true,bool control=false,bool parameters=false)
    :runtime([this](const auto& key,const auto& text,double value,bool numeric){
      publication_times[key]=time;
      if(numeric){numbers[key]=value;++numeric_counts[key];}else{strings[key]=text;publications.emplace_back(key,text);}
    },[this](const uint8_t* bytes,std::size_t size){
      mavlink_message_t message{},buffer{};mavlink_status_t parse{},status{};
      for(std::size_t i=0;i<size;++i)if(mavlink_frame_char_buffer(&buffer,&parse,bytes[i],&message,&status)==MAVLINK_FRAMING_OK)frames.push_back({message,time});
      return true;
    },simulation,false,[this]{return time;})
  {
    require(runtime.load(telemetry_module),"load telemetry MODULE");
    if(observers){
      require(runtime.load(status_module),"load status MODULE");runtime.configure("status_publish_interval","0.05");
      if(attitude_module!="-"){require(runtime.load(attitude_module),"load attitude MODULE");runtime.configure("attitude_publish_interval","0.02");}
    }
    if(control){require(runtime.load(control_module),"load control MODULE");runtime.configure("control_native_actions","false");runtime.configure("control_helm","false");}
    if(parameters)require(runtime.load(parameter_module),"load parameter MODULE");
    runtime.configure("lat_origin","42.358456");runtime.configure("long_origin","-71.087589");
  }
  void input(const mavlink_message_t& message,double effective=-1,const std::string& basis="QUEUE_EMPTY_FLOOR")
  {runtime.message(message,effective<0?time-.005:effective,time,basis);}
  void heartbeat(double effective=-1)
  {mavlink_message_t m{};mavlink_msg_heartbeat_pack(1,1,&m,2,3,128,4,4);input(m,effective);}
  void ready(bool position=true)
  {
    heartbeat();mavlink_message_t m{};
    if(position){mavlink_msg_global_position_int_pack(1,1,&m,1000,423584560,-710875890,15000,5000,0,100,0,27000);input(m);}
    mavlink_msg_extended_sys_state_pack(1,1,&m,0,2);input(m);
    mavlink_home_position_t home{};home.latitude=423584560;home.longitude=-710875890;home.altitude=10000;mavlink_msg_home_position_encode(1,1,&m,&home);input(m);
    mavlink_ekf_status_report_t ekf{};ekf.flags=51;mavlink_msg_ekf_status_report_encode(1,1,&m,&ekf);input(m);
  }
  void system(double effective=-1,unsigned source=1,const std::string& basis="QUEUE_EMPTY_FLOOR",bool invalid=false)
  {
    mavlink_sys_status_t value{};value.onboard_control_sensors_present=3;value.onboard_control_sensors_enabled=3;
    value.onboard_control_sensors_health=invalid?0:3;value.voltage_battery=12600;value.current_battery=0;value.battery_remaining=invalid?-2:100;
    mavlink_message_t m{};mavlink_msg_sys_status_encode(source,1,&m,&value);input(m,effective,basis);
  }
  void gps(double effective=-1)
  {
    mavlink_gps_raw_int_t value{};value.time_usec=1234567890123456ULL;value.fix_type=3;value.lat=423584560;value.lon=-710875890;
    value.alt=15000;value.vel=100;value.cog=9000;value.satellites_visible=12;
    mavlink_message_t m{};mavlink_msg_gps_raw_int_encode(1,1,&m,&value);input(m,effective);
  }
  void euler(double effective=-1,unsigned source=1,const std::string& basis="QUEUE_EMPTY_FLOOR",bool invalid=false)
  {
    mavlink_attitude_t value{};value.time_boot_ms=4000000000U;value.roll=.1f;value.pitch=-.2f;value.yaw=invalid?std::numeric_limits<float>::quiet_NaN():1.57079632679f;
    value.rollspeed=.3f;value.pitchspeed=-.4f;value.yawspeed=.5f;
    mavlink_message_t m{};mavlink_msg_attitude_encode(source,1,&m,&value);input(m,effective,basis);
  }
  void quaternion(double effective=-1,unsigned source=1,const std::string& basis="QUEUE_EMPTY_FLOOR",bool invalid=false)
  {
    mavlink_attitude_quaternion_t value{};value.time_boot_ms=4000000001U;value.q1=invalid?0.f:.70710678118f;value.q4=invalid?0.f:.70710678118f;
    value.rollspeed=.3f;value.pitchspeed=-.4f;value.yawspeed=.5f;
    mavlink_message_t m{};mavlink_msg_attitude_quaternion_encode(source,1,&m,&value);input(m,effective,basis);
  }
  void observations(double effective=-1,unsigned source=1,const std::string& basis="QUEUE_EMPTY_FLOOR",bool invalid=false)
  {system(effective,source,basis,invalid);euler(effective,source,basis,invalid);quaternion(effective,source,basis,invalid);}
  void tick(){runtime.tick(time);}
  void request(const std::string& id)
  {Mail m;m.key="MAVMOOS_PARAM_REQUEST";m.text="id="+id+",action=read,name=FRAME_CLASS";m.received_at=m.published_at=time;runtime.mail(m);}
  void parameterValue(double floor)
  {
    mavlink_param_value_t value{};const std::string name="FRAME_CLASS";std::copy(name.begin(),name.end(),value.param_id);
    value.param_value=1;value.param_type=MAV_PARAM_TYPE_INT8;value.param_index=UINT16_MAX;value.param_count=1364;
    mavlink_message_t m{};mavlink_msg_param_value_encode(1,1,&m,&value);input(m,floor);
  }
  bool result(const std::string& id,const std::string& status)const
  {for(const auto& p:publications)if(p.first=="MAVMOOS_PARAM_RESULT"&&p.second.find("id="+id+",")!=std::string::npos&&p.second.find("status="+status+",")!=std::string::npos)return true;return false;}
  unsigned count(unsigned id)const
  {return std::count_if(frames.begin(),frames.end(),[id](const auto& frame){return frame.message.msgid==id;});}
  void target(){require(runtime.navigation({90,1,5,time+.2}),"current navigation target denied");tick();}
  void snapshot(const std::string& name,const std::string& stage)const
  {
    std::cout<<std::setprecision(17)<<"{\"type\":\"snapshot\",\"case\":"<<jsonString(name)<<",\"stage\":"<<jsonString(stage)<<",\"time\":"<<time<<",\"runtime\":"<<jsonString(runtime.status())<<",\"capabilities\":"<<jsonString(runtime.capabilities())<<",\"strings\":{";
    bool comma=false;for(const auto& item:strings){if(comma)std::cout<<',';comma=true;std::cout<<jsonString(item.first)<<':'<<jsonString(item.second);}std::cout<<"},\"numbers\":{";
    comma=false;for(const auto& item:numbers){if(comma)std::cout<<',';comma=true;std::cout<<jsonString(item.first)<<':';if(std::isfinite(item.second))std::cout<<item.second;else std::cout<<"null";}
    std::cout<<"},\"numeric_counts\":{";comma=false;for(const auto& item:numeric_counts){if(comma)std::cout<<',';comma=true;std::cout<<jsonString(item.first)<<':'<<item.second;}
    std::cout<<"},\"publication_times\":{";comma=false;for(const auto& item:publication_times){if(comma)std::cout<<',';comma=true;std::cout<<jsonString(item.first)<<':'<<item.second;}
    std::cout<<"},\"frame_count\":"<<frames.size()<<",\"navigation_frames\":"<<count(84)<<"}\n";
  }
};
void seed(Rig& rig){rig.time=100.1;rig.ready();rig.time=100.2;rig.observations();rig.gps();rig.tick();}
std::vector<std::vector<double>> navigationProfile(bool observers,bool invalid)
{
  Rig rig(true,observers,true,true);rig.time=100.1;rig.ready();require(rig.runtime.acquireNavigation("explicit"),"explicit test grant denied");
  for(unsigned n=0;n<20;++n){rig.time=100.2+n*.1;rig.ready();if(observers)for(unsigned i=0;i<128;++i)rig.observations(-1,1,"QUEUE_EMPTY_FLOOR",invalid);rig.target();require(rig.runtime.navigationGranted(),"observer absence/data revoked host authority");}
  std::vector<std::vector<double>> targets;
  for(const auto& frame:rig.frames)if(frame.message.msgid==84){mavlink_set_position_target_local_ned_t p{};mavlink_msg_set_position_target_local_ned_decode(&frame.message,&p);targets.push_back({frame.at,double(p.coordinate_frame),double(p.type_mask),p.vx,p.vy,p.vz,p.yaw,p.yaw_rate});}
  require(!targets.empty(),"no control targets in comparison profile");rig.snapshot(invalid?"invalid_observer_authority":"healthy_observer_authority",observers?"with_observers":"baseline");return targets;
}
}
int main(int argc,char** argv)
{
  try {
    require(argc==7,"usage: observer_host_driver TELEMETRY STATUS ATTITUDE_OR_DASH CONTROL PARAMETERS INTERVAL_FIXTURE");telemetry_module=argv[1];status_module=argv[2];attitude_module=argv[3];control_module=argv[4];parameter_module=argv[5];interval_fixture=argv[6];
    unsigned groups=0;auto test=[&](const char* name,const std::function<void()>& fn){fn();++groups;std::cout<<"{\"type\":\"pass\",\"case\":"<<jsonString(name)<<"}\n";};
    test("coalesced_frames_and_actual_ingress_filtering",[]{
      Rig r; mavmoos::RxConfig config;config.target_system=1;config.target_component=1;config.expected_type=2;config.expected_autopilot=3;
      mavmoos::Telemetry ingress(config,"combined-observer-test");unsigned callbacks=0;uint64_t datagram=0;
      ingress.setTimedMessageHandler([&](const mavlink_message_t& m,double effective,double read){++callbacks;r.runtime.message(m,effective,read);});
      mavlink_status_t codec{};mavlink_message_t m{};std::vector<uint8_t> bytes;
      const auto append=[&]{uint8_t frame[MAVLINK_MAX_PACKET_LEN]{};const auto size=mavlink_msg_to_send_buffer(frame,&m);bytes.insert(bytes.end(),frame,frame+size);};
      const auto ingest=[&](double effective){ingress.ingest(bytes.data(),bytes.size(),effective,++datagram,r.time,"QUEUE_EMPTY_FLOOR");};
      r.time=100.1;mavlink_msg_heartbeat_pack_status(1,1,&codec,&m,2,3,128,4,4);append();ingest(100.095);bytes.clear();
      r.time=100.2;mavlink_msg_attitude_pack_status(1,1,&codec,&m,100,.1f,-.2f,.25f,.3f,-.4f,.5f);append();
      mavlink_msg_attitude_pack_status(1,1,&codec,&m,101,.1f,-.2f,1.5f,.3f,-.4f,.5f);append();ingest(100.15);r.tick();
      require(callbacks==3,"one complete datagram frame was not dispatched");r.snapshot("coalesced_frames_and_actual_ingress_filtering","last_frame");
      r.time=100.3;ingest(100.25);r.tick();require(callbacks==3&&ingress.stats().duplicates==2,"exact-byte duplicate suppression changed");r.snapshot("coalesced_frames_and_actual_ingress_filtering","duplicates");bytes.clear();
      r.time=100.4;mavlink_msg_attitude_pack_status(2,1,&codec,&m,102,0,0,2,0,0,0);append();ingest(100.35);bytes.clear();
      mavlink_msg_attitude_pack_status(1,1,&codec,&m,103,0,0,2.5f,0,0,0);append();bytes.back()^=1;ingest(100.35);r.tick();
      require(callbacks==3&&ingress.stats().wrong_target==1&&ingress.stats().bad_frames==1,"actual ingress filtering passed foreign/bad-CRC attitude");
      require(r.frames.empty()&&!r.runtime.navigationGranted(),"byte ingress granted effects");r.snapshot("coalesced_frames_and_actual_ingress_filtering","filtered");
      std::cout<<"{\"type\":\"ingress_metric\",\"callbacks\":"<<callbacks<<",\"duplicates\":"<<ingress.stats().duplicates<<",\"wrong_target\":"<<ingress.stats().wrong_target<<",\"bad_frames\":"<<ingress.stats().bad_frames<<",\"socket_io\":false}\n";
    });
    test("payload_routes_preserve_fc_authority_and_observation_age",[]{
      Rig rig(true,true,true);
      mavmoos::RxConfig config;
      config.bind_address=config.peer_address="127.0.0.1";
      config.bind_port=14600;config.peer_port=14601;
      config.target_system=config.target_component=1;
      config.expected_type=MAV_TYPE_QUADROTOR;config.expected_autopilot=MAV_AUTOPILOT_ARDUPILOTMEGA;
      config.camera_component=100;config.gimbal_component=154;
      require(mavmoos::validateConfig(config).empty(),"test payload route failed validation");
      mavmoos::Telemetry ingress(config,"payload-authority-test");
      unsigned callbacks=0;
      ingress.setTimedMessageHandler([&](const mavlink_message_t& message,double effective,double read) {
        ++callbacks;rig.runtime.message(message,effective,read);
      });
      mavlink_message_t message{};
      mavlink_status_t codec{};
      const auto ingest=[&] {
        uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};
        const auto size=mavlink_msg_to_send_buffer(bytes,&message);
        ingress.ingest(bytes,size,rig.time,1);
      };
      rig.time=100.1;rig.ready();
      mavlink_msg_heartbeat_pack_status(1,1,&codec,&message,MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_ARDUPILOTMEGA,
        MAV_MODE_FLAG_SAFETY_ARMED|MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,4,MAV_STATE_ACTIVE);
      ingest();
      require(rig.runtime.acquireNavigation("payload-route-test"),"initial explicit authority");
      const auto before=rig.runtime.vehicle();
      rig.time=100.2;
      mavlink_camera_settings_t camera{};
      mavlink_msg_camera_settings_encode_status(1,config.camera_component,&codec,&message,&camera);
      ingest();
      mavlink_gimbal_device_attitude_status_t gimbal{};
      mavlink_msg_gimbal_device_attitude_status_encode_status(1,config.gimbal_component,&codec,&message,&gimbal);
      ingest();
      mavlink_msg_heartbeat_pack_status(1,config.camera_component,&codec,&message,MAV_TYPE_CAMERA,
        MAV_AUTOPILOT_INVALID,0,0,MAV_STATE_STANDBY);
      ingest();
      mavlink_global_position_int_t pose{};
      mavlink_msg_global_position_int_encode_status(1,config.gimbal_component,&codec,&message,&pose);
      ingest();
      require(callbacks==3&&ingress.stats().wrong_target==2&&ingress.identityState()=="EXPECTED",
              "payload route forwarded unauthorized FC observations");
      const auto& after=rig.runtime.vehicle();
      require(after.type==before.type&&after.autopilot==before.autopilot&&after.mode==before.mode&&
              after.armed==before.armed&&after.heartbeat_at==before.heartbeat_at&&after.position_at==before.position_at&&
              rig.runtime.navigationGranted(), "payload route changed FC identity, observation age or active grant");
      rig.target();
      require(rig.count(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED)==1,"payload route blocked a valid grant");
      rig.time=100.75;
      mavlink_msg_camera_settings_encode_status(1,config.camera_component,&codec,&message,&camera);
      ingest();
      require(callbacks==4&&!rig.runtime.navigationGranted()&&!rig.runtime.vehicle().position_valid,
              "payload observations hid the original FC pose deadline");
      rig.tick();
      require(rig.count(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED)==1,"expired grant sent guidance");
      rig.time=100.8;rig.ready();
      mavlink_msg_gimbal_device_attitude_status_encode_status(1,config.gimbal_component,&codec,&message,&gimbal);
      ingest();
      require(!rig.runtime.navigationGranted(),"payload observations implicitly reacquired authority");
    });
    test("host_interval_union_has_no_control_authority",[]{
      Rig r(true,true,false,true);require(r.runtime.load(interval_fixture),"load test-only interval fixture");seed(r);
      std::map<unsigned,double> expected={{33,50000},{245,250000},{193,500000},{242,500000},{1,1000000},{147,1000000},{24,250000}};
      if(attitude_module!="-"){expected[30]=200000;expected[31]=200000;}
      // Setup is paced in the shared bulk slot, not completed in one burst.
      for(unsigned step=0;step<expected.size()+1;++step){r.time+=.11;r.ready();r.tick();}
      std::map<unsigned,double> observed;
      for(const auto& frame:r.frames){
        require(frame.message.sysid==245&&frame.message.compid==191,"interval source identity changed");
        if(frame.message.msgid==0)continue;
        require(frame.message.msgid==76,"observer-only host emitted a navigation/protocol control packet");
        mavlink_command_long_t p{};mavlink_msg_command_long_decode(&frame.message,&p);
        require(p.command==MAV_CMD_SET_MESSAGE_INTERVAL&&p.target_system==1&&p.target_component==1&&p.confirmation==0,"unexpected observer command or target");
        const auto id=static_cast<unsigned>(p.param1);require(p.param1==id&&expected.count(id)&&!observed.count(id),"duplicate/unknown interval request");
        require(p.param3==0&&p.param4==0&&p.param5==0&&p.param6==0&&p.param7==0,"unexpected interval control parameters");observed[id]=p.param2;
      }
      require(observed==expected,"descriptor union/minimum interval differs from constructed host requests");
      const auto count=r.frames.size();r.time+=.01;r.tick();require(r.frames.size()==count&&!r.runtime.navigationGranted(),"interval setup repeated or acquired control");
      r.snapshot("host_interval_union_has_no_control_authority","configured");
      std::cout<<"{\"type\":\"metric\",\"case\":\"host_interval_union_has_no_control_authority\",\"intervals_us\":{";bool comma=false;
      for(const auto& item:observed){if(comma)std::cout<<',';comma=true;std::cout<<jsonString(std::to_string(item.first))<<':'<<item.second;}
      std::cout<<"},\"native_wire_claim\":false}\n";
    });
    test("receive_only_projection_and_bounded_republication",[]{
      Rig r;seed(r);r.snapshot("receive_only_projection_and_bounded_republication","before");const auto nav=r.numeric_counts["NAV_HEADING"];
      r.time=100.3;for(unsigned i=0;i<2048;++i)r.observations();r.tick();
      require(r.frames.empty()&&!r.runtime.navigationGranted(),"observer-only RX host produced effects or acquired authority");
      require(r.numbers.at("NAV_HEADING")==270&&r.numeric_counts["NAV_HEADING"]==nav,"attitude replaced or renewed NAV heading");r.snapshot("receive_only_projection_and_bounded_republication","after");
    });
    test("original_ages_and_family_expiry",[]{
      Rig r;seed(r);r.snapshot("original_ages_and_family_expiry","initial");r.time=100.9;r.heartbeat();r.tick();r.snapshot("original_ages_and_family_expiry","republished");
      r.time=101.3;r.heartbeat();r.tick();r.snapshot("original_ages_and_family_expiry","attitude_expired");require(r.frames.empty(),"expiry produced output");
    });
    test("independent_family_updates",[]{
      Rig r;r.runtime.configure("status_timeout","0.25");if(attitude_module!="-")r.runtime.configure("attitude_timeout","0.25");seed(r);r.snapshot("independent_family_updates","initial");
      r.time=100.7;r.heartbeat();r.system(100.65);r.euler(100.65);r.tick();r.snapshot("independent_family_updates","one_each_updated");
    });
    test("foreign_old_and_invalid_metadata_do_not_renew",[]{
      Rig r;seed(r);r.snapshot("foreign_old_and_invalid_metadata_do_not_renew","initial");r.time=100.6;
      r.observations(100.59,2);r.observations(100.05);r.observations(100.59,1,"UNKNOWN");
      mavlink_message_t bad{};mavlink_msg_attitude_pack(1,1,&bad,1,0,0,0,0,0,0);r.runtime.message(bad,101,100.6);
      r.tick();r.snapshot("foreign_old_and_invalid_metadata_do_not_renew","rejected");require(r.frames.empty(),"invalid observations caused output");
    });
    test("post_reset_floor_must_follow_boundary",[]{
      Rig r;seed(r);r.snapshot("post_reset_floor_must_follow_boundary","initial");r.time=101;r.runtime.disconnect(101,"observer_test");r.tick();r.snapshot("post_reset_floor_must_follow_boundary","reset");
      r.time=101.2;r.heartbeat(100.99);r.observations(101.1);r.tick();r.snapshot("post_reset_floor_must_follow_boundary","preboundary_heartbeat");
      r.time=101.4;r.heartbeat(101.3);r.observations(101.31);r.tick();r.snapshot("post_reset_floor_must_follow_boundary","new_session");require(r.frames.empty(),"reset caused RX output");
    });
    test("new_heartbeat_cannot_hide_continuity_gap",[]{
      Rig r(true,true,true);seed(r);require(r.runtime.acquireNavigation("old"),"old grant");require(r.runtime.navigation({90,1,5,100.4}),"old target admission");
      r.time=102.3;r.heartbeat(102.25);r.observations(102.26);r.tick();require(!r.runtime.navigationGranted()&&r.count(84)==0,"fresh observers/heartbeat revived old navigation");r.snapshot("new_heartbeat_cannot_hide_continuity_gap","gap");
      r.time=102.5;r.ready();r.observations();r.tick();require(!r.runtime.navigationGranted(),"new observation session acquired navigation");r.snapshot("new_heartbeat_cannot_hide_continuity_gap","fresh_no_grant");
    });
    test("protocol_read_keeps_actual_receipt_during_observer_flood",[]{
      Rig r(true,true,true,true);seed(r);require(r.runtime.acquireNavigation("read"),"read grant");r.time=100.25;r.request("combined-read");r.tick();require(r.count(20)==1,"read not sent");
      r.time=100.35;for(unsigned i=0;i<1024;++i)r.observations(100.1);r.parameterValue(100.1);r.tick();
      require(r.result("combined-read","COMPLETED")&&r.runtime.navigationGranted(),"observation floor changed protocol correlation or authority");r.snapshot("protocol_read_keeps_actual_receipt_during_observer_flood","complete");
    });
    test("observers_do_not_change_target_schedule_or_geometry",[]{
      const auto baseline=navigationProfile(false,false);require(navigationProfile(true,false)==baseline,"healthy observers changed target schedule/geometry");require(navigationProfile(true,true)==baseline,"invalid observers changed target schedule/geometry");
    });
    test("healthy_observers_cannot_mask_pose_loss",[]{
      Rig r(true,true,true,true);seed(r);require(r.runtime.acquireNavigation("pose"),"pose grant");r.time=100.25;r.request("pose-read");r.tick();
      r.time=100.8;r.ready(false);r.observations();r.parameterValue(100.79);r.tick();
      require(!r.runtime.navigationGranted()&&r.result("pose-read","COMPLETED"),"healthy observers replaced pose or cancelled independent read");r.snapshot("healthy_observers_cannot_mask_pose_loss","pose_lost");
      r.time=100.9;r.ready();r.observations();r.tick();require(!r.runtime.navigationGranted(),"fresh pose/observer implicitly reacquired");
    });
    test("observer_updates_cannot_renew_resume_permit",[]{
      Rig r(true,true,true);seed(r);require(r.runtime.acquireNavigation("handoff"),"handoff grant");require(r.runtime.suspendNavigation(100.55),"suspend navigation");
      for(unsigned n=0;n<4;++n){r.time=100.3+n*.07;r.ready();r.observations();r.tick();require(!r.runtime.navigationGranted(),"observer update resumed navigation");}
      r.time=100.56;r.ready();r.observations();require(!r.runtime.resumeNavigation("late"),"observers renewed original resume lifetime");r.snapshot("observer_updates_cannot_renew_resume_permit","expired");
    });
    test("heartbeat_loss_cancels_old_protocol_despite_observers",[]{
      Rig r(true,true,true,true);seed(r);r.time=100.25;r.request("old-read");r.tick();r.time=102.4;r.observations();r.tick();
      require(r.result("old-read","CANCELLED"),"healthy observation families prolonged heartbeat continuity");r.time=102.6;r.ready();r.observations();r.parameterValue(102.59);r.tick();
      require(!r.result("old-read","COMPLETED")&&!r.runtime.navigationGranted(),"new session revived old read or authority");r.snapshot("heartbeat_loss_cancels_old_protocol_despite_observers","cancelled");
    });
    std::cout<<"{\"type\":\"complete\",\"groups\":"<<groups<<",\"attitude_loaded\":"<<(attitude_module=="-"?"false":"true")<<",\"io\":\"synchronous_fake_only\"}\n";return 0;
  }catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
