#include "Runtime.h"
#include <algorithm>
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
}
int main(int argc,char** argv) {
  try {
    check(argc==3,"usage: ingress_runtime_tests PARAMETERS_MODULE CONTROL_MODULE");control_module=argv[2];
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
    std::cout<<"PASS 5 ingress/runtime groups: observation continuity, conservative floor, protocol receipt and scoped cancellation\n";
    return 0;
  } catch(const std::exception& e) {std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
