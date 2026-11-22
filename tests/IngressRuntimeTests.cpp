#include "Runtime.h"
#include "Telemetry.h"
#include "Fixtures.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>
using namespace mavmoos::runtime;
namespace {
void check(bool ok,const char* why) { if(!ok) throw std::runtime_error(why); }
std::string control_module;
struct Rig {
  double time=100;
  unsigned nav_frames=0;
  std::map<std::string,std::string> records;
  Runtime runtime;
  Rig():runtime([this](const auto& key,const auto& text,double,bool numeric) {if(!numeric)records[key]=text;},
    [this](const uint8_t* bytes,std::size_t size) {
      mavlink_message_t frame{},buffer{};mavlink_status_t parse{},status{};
      for(std::size_t i=0;i<size;++i)
        if(mavlink_frame_char_buffer(&buffer,&parse,bytes[i],&frame,&status)==MAVLINK_FRAMING_OK && frame.msgid==84)++nav_frames;
      return true;
    },true,false,[this]{return time;}) {
    check(runtime.load(control_module),"load Copter control provider");
    runtime.configure("control_native_actions","false");runtime.configure("control_helm","false");
    fresh();
  }
  void fresh(double floor=-1) {
    if(floor<0)floor=time;
    mavlink_message_t m{};
    mavlink_msg_heartbeat_pack(1,1,&m,2,3,128,4,4);runtime.message(m,floor,time);
    mavlink_msg_global_position_int_pack(1,1,&m,1000,423584560,-710875890,15000,5000,0,0,0,9000);runtime.message(m,floor,time);
    mavlink_msg_extended_sys_state_pack(1,1,&m,0,2);runtime.message(m,floor,time);
    mavlink_home_position_t home{};home.latitude=423584560;home.longitude=-710875890;home.altitude=10000;
    mavlink_msg_home_position_encode(1,1,&m,&home);runtime.message(m,floor,time);
    mavlink_ekf_status_report_t ekf{};ekf.flags=51;
    mavlink_msg_ekf_status_report_encode(1,1,&m,&ekf);runtime.message(m,floor,time);
  }
  void request(const std::string& id) {
    Mail m;m.key="MAVMOOS_PARAM_REQUEST";m.text="id="+id+",action=read,name=FRAME_CLASS";
    m.received_at=m.published_at=time;runtime.mail(m);runtime.tick(time);
  }
  void value(double floor) {
    mavlink_param_value_t v{};const std::string name="FRAME_CLASS";
    std::copy(name.begin(),name.end(),v.param_id);v.param_value=1;v.param_type=MAV_PARAM_TYPE_INT8;
    v.param_index=UINT16_MAX;v.param_count=1364;
    mavlink_message_t m{};mavlink_msg_param_value_encode(1,1,&m,&v);runtime.message(m,floor,time);
  }
};

void positionHeadingBoundaries(const char* telemetry_module,uint8_t autopilot)
{
  std::map<std::string,double> publications;
  Runtime runtime([&](const auto& key,const auto&,double value,bool numeric) {
    if(numeric) publications[key]=value;
  },[](const uint8_t*,std::size_t) -> bool { throw std::runtime_error("receive-only runtime transmitted"); },
    false,false,{},MAV_TYPE_GROUND_ROVER,autopilot);
  check(runtime.load(telemetry_module),"load telemetry for position headings");
  mavmoos::RxConfig config;
  config.target_system=1; config.target_component=1;
  config.expected_type=MAV_TYPE_GROUND_ROVER; config.expected_autopilot=autopilot;
  mavmoos::Telemetry rx(config,"position-heading");
  rx.setTimedMessageHandler([&](const auto& m,double at,double read_at){runtime.message(m,at,read_at);});

  fixture::Encoder fc;
  uint64_t datagram=0;
  auto deliver=[&](const fixture::Bytes& bytes) {
    const double at=100.+(++datagram)*.01;
    publications.clear();
    rx.ingest(bytes.data(),bytes.size(),at,datagram);
    runtime.tick(at);
  };
  const bool px4=autopilot==MAV_AUTOPILOT_PX4;
  mavlink_heartbeat_t heartbeat{};
  heartbeat.type=MAV_TYPE_GROUND_ROVER; heartbeat.base_mode=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  heartbeat.system_status=MAV_STATE_ACTIVE;
  heartbeat.autopilot=px4?MAV_AUTOPILOT_ARDUPILOTMEGA:MAV_AUTOPILOT_PX4;
  deliver(fc.encode(heartbeat,mavlink_msg_heartbeat_encode_status));
  deliver(fc.position(0,36000));
  check(!rx.observation(mavmoos::Family::Position).present && !runtime.vehicle().position_valid &&
        !publications.count("NAV_HEADING"),"mismatched controller supplied a heading");
  heartbeat.autopilot=autopilot;
  deliver(fc.encode(heartbeat,mavlink_msg_heartbeat_encode_status));

  auto sample=[&](uint16_t raw,bool valid_encoding,double expected) {
    deliver(fc.position(0,raw));
    const auto& report=rx.observation(mavmoos::Family::Position);
    fixture::check(report.present && report.encoding_valid==valid_encoding,
      "GLOBAL_POSITION_INT.hdg="+std::to_string(raw)+" autopilot="+std::to_string(autopilot)+
      " raw encoding_valid="+std::to_string(report.encoding_valid)+" Runtime heading="+
      std::to_string(runtime.vehicle().heading)+" NAV_HEADING_VALID="+std::to_string(publications.at("NAV_HEADING_VALID")));
    const auto raw_heading=fixture::fields(report.fields).at("heading_deg");
    if(raw==UINT16_MAX) check(raw_heading=="unknown","unknown heading lost its sentinel");
    else check(std::stod(raw_heading)==(std::isfinite(expected)?expected:raw/100.),"raw position heading changed");
    check(runtime.vehicle().position_valid && publications.at("NAV_VALID")==1,"heading invalidated location");
    if(std::isfinite(expected)) {
      check(runtime.vehicle().heading==expected && publications.at("NAV_HEADING_VALID")==1 &&
            publications.at("NAV_HEADING")==expected,"usable heading missing from Runtime or telemetry");
    } else {
      check(std::isnan(runtime.vehicle().heading) && publications.at("NAV_HEADING_VALID")==0 &&
            !publications.count("NAV_HEADING"),"unusable heading published as a bearing");
    }
  };
  sample(0,true,0.);
  sample(35999,true,359.99);
  sample(36000,px4,px4?0.:NAN);
  sample(36001,false,NAN);
  sample(65534,false,NAN);
  sample(UINT16_MAX,true,NAN);
}
}
int main(int argc,char** argv) {
  try {
    check(argc==4,"usage: ingress_runtime_tests PARAMETERS_MODULE CONTROL_MODULE TELEMETRY_MODULE");control_module=argv[2];
    positionHeadingBoundaries(argv[3],MAV_AUTOPILOT_PX4);
    positionHeadingBoundaries(argv[3],MAV_AUTOPILOT_ARDUPILOTMEGA);
    { Rig r;check(r.runtime.acquireNavigation("initial"),"initial NAV acquire");
      check(r.runtime.navigation({90,1,5,101}),"initial target");
      r.time=100.7;r.runtime.checkpoint(r.time);r.fresh(100.65);
      check(!r.runtime.navigation({90,1,5,101}),"fresh ingress concealed old pose expiry");
      r.runtime.tick(r.time);check(r.nav_frames==0,"old pending target sent after observation gap");
      check(r.runtime.acquireNavigation("new"),"explicit new acquisition denied after fresh state");
    }
    { Rig r;r.time=100.3;r.fresh(100.0);
      check(r.runtime.vehicle().position_at==100.0 && r.runtime.vehicle().position_read_at==100.3,"effective/read timestamps conflated");
      check(r.runtime.acquireNavigation("floor"),"valid conservative floor denied");
      r.time=100.6;r.runtime.tick(r.time);
      check(!r.runtime.navigation({90,1,5,101}),"read time extended position freshness");
    }
    { Rig r;check(r.runtime.load(argv[1]),"load parameter module");r.time=100.02;r.request("floor-before-send");
      r.time=100.03;r.value(100.0);
      check(r.records["MAVMOOS_PARAM_RESULT"].find("status=COMPLETED")!=std::string::npos,"conservative floor broke post-send protocol receipt");
    }
    { Rig r;check(r.runtime.load(argv[1]),"load parameter module");r.time=100.02;r.request("pose-gap");
      r.time=100.8;r.runtime.checkpoint(r.time);r.value(100.7);
      check(r.records["MAVMOOS_PARAM_RESULT"].find("status=COMPLETED")!=std::string::npos,"navigation pose gap cancelled independent parameter read");
    }
    { Rig r;check(r.runtime.load(argv[1]),"load parameter module");r.time=100.02;r.request("heartbeat-gap");
      r.time=102.1;r.runtime.checkpoint(r.time);r.fresh(102.05);r.value(102.05);
      check(r.records["MAVMOOS_PARAM_RESULT"].find("status=CANCELLED")!=std::string::npos,"heartbeat session gap revived pending protocol operation");
    }
    std::cout<<"PASS 7 ingress/runtime groups: firmware heading boundaries, observation continuity, conservative floor, protocol receipt and scoped cancellation\n";
    return 0;
  } catch(const std::exception& e) {std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
