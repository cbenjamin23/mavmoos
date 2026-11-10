#include "Runtime.h"
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
using namespace mavmoos::runtime;
namespace {
void check(bool ok,const char* why) { if(!ok) throw std::runtime_error(why); }
struct Rig {
  double time=100;
  std::map<std::string,std::string> records;
  std::vector<mavlink_message_t> frames;
  Runtime runtime;
  Rig():runtime([this](const auto& key,const auto& text,double,bool numeric) {if(!numeric)records[key]=text;},
    [this](const uint8_t* data,std::size_t size) {
      mavlink_message_t message{},buffer{};mavlink_status_t parser{},status{};
      for(std::size_t i=0;i<size;++i)
        if(mavlink_frame_char_buffer(&buffer,&parser,data[i],&message,&status)==MAVLINK_FRAMING_OK)frames.push_back(message);
      return true;
    },true,false,[this]{return time;}) { fresh(); }
  void fresh(unsigned mode=4) {
    mavlink_message_t m{};mavlink_msg_heartbeat_pack(1,1,&m,2,3,128,mode,4);runtime.message(m,time);
    mavlink_msg_global_position_int_pack(1,1,&m,1000,423584560,-710875890,15000,5000,0,0,0,9000);runtime.message(m,time);
    mavlink_msg_extended_sys_state_pack(1,1,&m,0,2);runtime.message(m,time);
    mavlink_home_position_t home{};home.latitude=423584560;home.longitude=-710875890;home.altitude=10000;
    mavlink_msg_home_position_encode(1,1,&m,&home);runtime.message(m,time);
    mavlink_ekf_status_report_t ekf{};ekf.flags=51;mavlink_msg_ekf_status_report_encode(1,1,&m,&ekf);runtime.message(m,time);
  }
  void text(const std::string& key,const std::string& value,double at,const std::string& source="pHelmIvP") {
    time=at;Mail m;m.key=key;m.text=value;m.source=source;m.published_at=m.received_at=time;runtime.mail(m);
  }
  void fields(double at) {
    time=at;
    for(const auto& p:std::vector<std::pair<std::string,double>>{{"DESIRED_HEADING",90},{"DESIRED_SPEED",1},{"DESIRED_ALTITUDE",5}}) {
      Mail m;m.key=p.first;m.value=p.second;m.numeric=true;m.source="pHelmIvP";m.published_at=m.received_at=time;runtime.mail(m);
    }
  }
  void load(const std::string& control) {
    check(runtime.load(control),"load coherent control provider");
  }
  void start(bool active=true) {
    text("IVPHELM_ALLSTOP","clear",100.01);text("IVPHELM_STATE","DRIVE",100.02);
    text("MAVMOOS_NAV_CONTROL","acquire",100.03,"operator");fields(100.04);
    if(active) { runtime.tick(time);check(records["MAVMOOS_HELM_STATUS"].find("state=ACTIVE")!=std::string::npos,"initial ACTIVE missing"); }
  }
};
struct Counts { unsigned nav=0,hold=0,land=0,rtl=0,guided=0,position_hold=0; };
Counts count(const Rig& r) {
  Counts c;
  for(const auto& frame:r.frames) {
    if(frame.msgid==MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED)++c.nav;
    if(frame.msgid==MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT)++c.position_hold;
    if(frame.msgid!=MAVLINK_MSG_ID_COMMAND_LONG)continue;
    mavlink_command_long_t p{};mavlink_msg_command_long_decode(&frame,&p);
    if(p.command==MAV_CMD_NAV_LAND)++c.land;
    if(p.command==MAV_CMD_NAV_RETURN_TO_LAUNCH)++c.rtl;
    if(p.command==MAV_CMD_DO_SET_MODE&&p.param2==4)++c.guided;
    if(p.command==MAV_CMD_DO_SET_MODE&&p.param2==5)++c.hold;
  }
  return c;
}
}
int main(int argc,char** argv) {
  try {
    check(argc==2||argc==3,"args: control module[, --expect-conflict]");
    const bool expect_conflict=argc==3&&std::string(argv[2])=="--expect-conflict";
    unsigned conflicts=0,hold_leaks=0;
    for(const std::string action:{"hold","land","rtl"}) {
      Rig r;r.load(argv[1]);r.start();
      r.text("IVPHELM_ALLSTOP","NothingToDo",100.10);
      r.text("IVPHELM_ALLSTOP","clear",100.14);r.fields(100.15);
      r.text("MAVMOOS_COMMAND","id=explicit-"+action+",action="+action,100.16,"operator");
      r.frames.clear();r.runtime.tick(r.time);auto c=count(r);
      const bool permission=r.runtime.status().find("permission=true")!=std::string::npos;
      const bool conflict=c.nav>0||permission;conflicts+=conflict;
      std::cout<<"{\"case\":\"paused_resume_vs_"<<action<<"\",\"conflict\":"<<(conflict?"true":"false")
        <<",\"nav_frames\":"<<c.nav<<",\"permission\":"<<(permission?"true":"false")
        <<",\"hold\":"<<c.hold<<",\"land\":"<<c.land<<",\"rtl\":"<<c.rtl<<",\"guided\":"<<c.guided<<",\"wire_ids\":[";
      for(std::size_t i=0;i<r.frames.size();++i)std::cout<<(i?",":"")<<r.frames[i].msgid;
      std::cout<<"]}\n";
      check(c.hold+c.land+c.rtl==1,"explicit command missing");
    }
    for(bool automatic:{true,false}) {
      Rig r;r.load(argv[1]);r.start();
      if(automatic) {
        r.text("IVPHELM_ALLSTOP","NothingToDo",100.10);
        r.text("IVPHELM_ALLSTOP","clear",100.14);r.fields(100.15);
      } else {
        r.text("MAVMOOS_NAV_CONTROL","release",100.10,"operator");
        r.text("MAVMOOS_NAV_CONTROL","acquire",100.14,"operator");r.fields(100.15);
      }
      r.frames.clear();r.time=100.16;r.runtime.tick(r.time);auto c=count(r);
      hold_leaks+=c.position_hold>0;
      std::cout<<"{\"case\":\"obsolete_position_hold_after_"<<(automatic?"resume":"explicit_reacquire")
        <<"\",\"position_hold_frames\":"<<c.position_hold<<",\"nav_frames\":"<<c.nav<<",\"wire_ids\":[";
      for(std::size_t i=0;i<r.frames.size();++i)std::cout<<(i?",":"")<<r.frames[i].msgid;
      std::cout<<"]}\n";
      check(c.nav==1,"fresh navigation did not resume");
    }
    for(const std::string action:{"hold","land","rtl"}) {
      Rig r;r.fresh(5);r.load(argv[1]);r.start(false);
      r.text("MAVMOOS_COMMAND","id=cancel-guided-"+action+",action="+action,100.06,"operator");
      r.frames.clear();r.runtime.tick(r.time);auto c=count(r);
      std::cout<<"{\"case\":\"queued_acquire_cancelled_by_"<<action<<"\",\"guided\":"<<c.guided<<",\"nav_frames\":"<<c.nav<<"}\n";
      check(c.guided==0&&c.nav==0,"queued Guided acquisition survived explicit interruption");
    }
    check(expect_conflict?conflicts==3:conflicts==0,"authority conflict expectation not met");
    check(expect_conflict?hold_leaks==2:hold_leaks==0,"obsolete conditional hold expectation not met");
    std::cout<<"PASS "<<(expect_conflict?"baseline reproduced three conflicts":"explicit commands retain navigation inhibition")<<"; queued Guided cancellation preserved\n";
    return 0;
  } catch(const std::exception& e) {std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
