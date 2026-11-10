// Real Runtime + coherent Copter provider, with optional active helm strategy; fake I/O only.
#include "Runtime.h"
#include <algorithm>
#include <iostream>
#include <map>
#include <stdexcept>
#include <vector>
using namespace mavmoos::runtime;
namespace {
void check(bool ok,const char* why) { if(!ok)throw std::runtime_error(why); }
std::string control_module;
struct Rig {
  double time=100;
  std::vector<mavlink_message_t> wire;
  std::map<std::string,std::string> latest;
  Runtime runtime;
  explicit Rig(bool helm=false):runtime(
    [this](const auto& key,const auto& text,double,bool numeric){if(!numeric)latest[key]=text;},
    [this](const uint8_t* bytes,std::size_t size) {
      mavlink_message_t message{},buffer{};mavlink_status_t parser{},status{};
      for(std::size_t i=0;i<size;++i)
        if(mavlink_frame_char_buffer(&buffer,&parser,bytes[i],&message,&status)==MAVLINK_FRAMING_OK)wire.push_back(message);
      return true;
    },true,false,[this]{return time;}) {
    check(runtime.load(control_module),"real Copter provider failed to load");
    runtime.configure("control_native_actions","false");runtime.configure("control_helm",helm?"true":"false");
    fresh();
  }
  void heartbeat(unsigned mode=4) {
    mavlink_message_t message{};mavlink_msg_heartbeat_pack(1,1,&message,2,3,128,mode,4);runtime.message(message,time);
  }
  void fresh(unsigned mode=4) {
    heartbeat(mode);mavlink_message_t message{};
    mavlink_msg_global_position_int_pack(1,1,&message,1000,423584560,-710875890,15000,5000,0,0,0,9000);runtime.message(message,time);
    mavlink_msg_extended_sys_state_pack(1,1,&message,0,MAV_LANDED_STATE_IN_AIR);runtime.message(message,time);
    mavlink_home_position_t home{};home.latitude=423584560;home.longitude=-710875890;home.altitude=10000;
    mavlink_msg_home_position_encode(1,1,&message,&home);runtime.message(message,time);
    mavlink_ekf_status_report_t ekf{};ekf.flags=51;mavlink_msg_ekf_status_report_encode(1,1,&message,&ekf);runtime.message(message,time);
  }
  void text(const std::string& key,const std::string& value) {
    Mail mail;mail.key=key;mail.text=value;mail.source=key=="MAVMOOS_NAV_CONTROL"?"operator":"pHelmIvP";
    mail.received_at=mail.published_at=time;runtime.mail(mail);
  }
  void fields() {
    text("IVPHELM_STATE","DRIVE");
    for(const auto& field:std::vector<std::pair<std::string,double>>{{"DESIRED_HEADING",90},{"DESIRED_SPEED",1},{"DESIRED_ALTITUDE",5}}) {
      Mail mail;mail.key=field.first;mail.value=field.second;mail.numeric=true;mail.source="pHelmIvP";
      mail.received_at=mail.published_at=time;runtime.mail(mail);
    }
  }
  void active() {
    time=100.01;text("IVPHELM_ALLSTOP","clear");time=100.02;text("IVPHELM_STATE","DRIVE");
    time=100.03;text("MAVMOOS_NAV_CONTROL","acquire");time=100.04;fresh();fields();runtime.tick(time);
    check(latest["MAVMOOS_HELM_STATUS"].find("state=ACTIVE")!=std::string::npos,"real Helm did not activate");
  }
  unsigned nav()const {return std::count_if(wire.begin(),wire.end(),[](const auto& message){return message.msgid==84;});}
};
void hostCase(unsigned scenario,bool expect_gap) {
  Rig rig;check(rig.runtime.acquireNavigation("start"),"host acquire denied");
  rig.time=scenario==2?100.4:100;rig.runtime.checkpoint(rig.time);
  const double until=rig.time+.5;check(rig.runtime.suspendNavigation(until),"host pause denied");
  if(scenario==0) {rig.time=100.1;rig.heartbeat(5);}
  else if(scenario==1) {rig.time=100.1;rig.heartbeat(6);rig.time=100.2;rig.heartbeat(4);}
  else {rig.time=100.6;rig.runtime.checkpoint(rig.time);rig.time=100.61;rig.fresh();}
  const std::string before=rig.runtime.status();
  const bool resumed=rig.runtime.resumeNavigation("normal-resume");
  if(resumed)rig.runtime.navigation({90,1,5,rig.time+.2});
  rig.runtime.tick(rig.time);
  unsigned guided=0;
  for(const auto& message:rig.wire)if(message.msgid==76) {
    mavlink_command_long_t command{};mavlink_msg_command_long_decode(&message,&command);
    if(command.command==MAV_CMD_DO_SET_MODE&&command.param2==4)++guided;
  }
  std::cout<<"{\"layer\":\"bare_host\",\"case\":"<<scenario<<",\"handoff_window\":0.5,\"resumed\":"<<(resumed?"true":"false")
    <<",\"guided_requests\":"<<guided<<",\"nav_frames\":"<<rig.nav()<<",\"permit_before_resume\":"
    <<(before.find("resume_permit=true")!=std::string::npos?"true":"false")<<"}\n";
  check(resumed==expect_gap,"host resume-permit expectation failed");
  if(!expect_gap)check(guided==0&&rig.nav()==0,"invalidated handoff emitted fresh control work");
}
void moduleControl(unsigned scenario) {
  Rig rig(true);rig.active();const auto before=rig.nav();
  rig.time=scenario==2?100.4:100.1;rig.text("IVPHELM_ALLSTOP","NothingToDo");
  if(scenario==0) {rig.time=100.2;rig.heartbeat(5);rig.time=100.3;rig.fresh();}
  else if(scenario==1) {rig.time=100.2;rig.heartbeat(6);rig.time=100.3;rig.fresh();}
  else {rig.time=100.6;rig.runtime.checkpoint(rig.time);rig.time=100.61;rig.fresh();}
  rig.time+=.01;rig.text("IVPHELM_ALLSTOP","clear");rig.fields();rig.runtime.tick(rig.time);
  const bool latched=rig.latest["MAVMOOS_HELM_STATUS"].find("state=LATCHED")!=std::string::npos;
  std::cout<<"{\"layer\":\"real_helm_control\",\"case\":"<<scenario<<",\"latched\":"<<(latched?"true":"false")
    <<",\"new_nav_frames\":"<<rig.nav()-before<<"}\n";
  check(latched&&rig.nav()==before,"real Helm failed to inhibit invalid handoff");
}
}
int main(int argc,char** argv) {
  try {
    check(argc==2||(argc==3&&std::string(argv[2])=="--expect-gap"),"usage: resume_permit_diagnostic CONTROL_MODULE [--expect-gap]");
    const bool expect_gap=argc==3;control_module=argv[1];
    for(unsigned scenario=0;scenario<3;++scenario)hostCase(scenario,expect_gap);
    for(unsigned scenario=0;scenario<3;++scenario)moduleControl(scenario);
    std::cout<<"PASS 3 host "<<(expect_gap?"baseline gaps":"invalidations")<<" + 3 real-Helm controls; fake I/O only\n";return 0;
  }catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
