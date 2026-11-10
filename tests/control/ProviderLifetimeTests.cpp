#include "Runtime.h"
#include <functional>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace mavmoos::runtime;
namespace {
void require(bool ok,const char* why) {if(!ok)throw std::runtime_error(why);}
struct Effects {
  unsigned sends=0,observers_destroyed=0;
  std::vector<mavlink_message_t> frames;
  Runtime::Publish publication()
  {
    return [this](const auto& key,const auto& value,double,bool numeric) {
      if(!numeric&&key=="MAVMOOS_TEST_PROVIDER_LIFETIME"&&value=="observer_destroyed")++observers_destroyed;
    };
  }
  Runtime::Send sender()
  {
    return [this](const uint8_t* bytes,std::size_t size) {
      ++sends;mavlink_message_t message{},buffer{};mavlink_status_t parser{},status{};
      for(std::size_t n=0;n<size;++n)
        if(mavlink_frame_char_buffer(&buffer,&parser,bytes[n],&message,&status)==MAVLINK_FRAMING_OK)frames.push_back(message);
      return true;
    };
  }
  unsigned count(unsigned type) const
  {unsigned result=0;for(const auto& frame:frames)if(frame.msgid==type)++result;return result;}
};
void fresh(Runtime& runtime,double now=100,unsigned mode=4)
{
  mavlink_message_t message{};
  mavlink_msg_heartbeat_pack(1,1,&message,2,3,128,mode,4);runtime.message(message,now);
  mavlink_msg_global_position_int_pack(1,1,&message,1000,423584560,-710875890,15000,5000,0,0,0,9000);runtime.message(message,now);
  mavlink_msg_extended_sys_state_pack(1,1,&message,0,MAV_LANDED_STATE_IN_AIR);runtime.message(message,now);
  mavlink_home_position_t home{};home.latitude=423584560;home.longitude=-710875890;home.altitude=10000;
  mavlink_msg_home_position_encode(1,1,&message,&home);runtime.message(message,now);
  mavlink_ekf_status_report_t estimator{};estimator.flags=51;
  mavlink_msg_ekf_status_report_encode(1,1,&message,&estimator);runtime.message(message,now);
}
void replacement(const std::string& provider)
{
  Effects effects;
  {
    Runtime runtime(effects.publication(),effects.sender(),false,false,[]{return 100.;});
    require(runtime.load(provider),"provider fixture not loaded");
    runtime.configure("replace_backend","true");
    require(runtime.status().find("control_profile=arducopter")!=std::string::npos,"successful configure retained a dead or missing provider");
    require(runtime.moduleNames()==std::vector<std::string>{"replaceable_provider"},"valid provider replacement changed composition");
  }
  require(effects.sends==0,"startup provider configuration attempted I/O");
}
void teardown(const std::string& provider,const std::string& observer)
{
  Effects effects;
  {
    Runtime runtime(effects.publication(),effects.sender(),true,false,[]{return 100.;});
    require(runtime.load(observer)&&runtime.load(provider),"teardown fixture composition failed");
    fresh(runtime);require(runtime.acquireNavigation("active"),"teardown fixture grant failed");
    require(runtime.navigation({90,1,5,101}),"teardown fixture NAV not waiting");
    // Reverse destruction retires the provider before the observer destructor
    // calls Services. Code residency cannot keep the deleted backend alive.
  }
  require(effects.observers_destroyed==1,"observer Services callback did not execute exactly once during teardown");
  require(effects.sends==0,"teardown attempted I/O");
}
void rejectedConfiguration(const std::string& provider,const std::string& observer)
{
  Effects effects;
  {
    Runtime runtime(effects.publication(),effects.sender(),true,false,[]{return 100.;});
    require(runtime.load(observer)&&runtime.load(provider),"failed-config fixture composition failed");
    fresh(runtime);require(runtime.acquireNavigation("active"),"failed-config fixture grant failed");
    require(runtime.navigation({90,1,5,101}),"failed-config fixture NAV not queued");
    ParameterRequest read;read.id="queued-before-config";read.name="FRAME_CLASS";read.expires=101;
    require(runtime.parameter(read),"failed-config fixture parameter not queued");
    bool rejected=false;
    try {runtime.configure("reject_reconfigure","true");}
    catch(const std::exception& error) {
      rejected=std::string(error.what()).find("duplicate capability")!=std::string::npos;
    }
    require(rejected,"metadata collision not rejected");
    require(effects.observers_destroyed==1,"failed configure did not release the rejected composition");
    const auto status=runtime.status();
    require(status.find("configuration_valid=false")!=std::string::npos&&status.find("control_profile=none")!=std::string::npos&&
      status.find("permission=false")!=std::string::npos&&status.find("parameter_queue=0")!=std::string::npos,"failed configure left stale state live");
    require(runtime.moduleNames().empty()&&runtime.subscriptions().empty(),"failed composition remained dispatchable");
    // Fresh observations and caller continuation must not revive effects after
    // a partially applied configuration replaced then invalidated its provider.
    fresh(runtime);
    require(!runtime.acquireNavigation("after-invalid-composition")&&!runtime.resumeNavigation("old")&&
      !runtime.navigation({90,1,5,101}),"failed configure restored navigation");
    CommandRequest command;command.id="after-failed-config";command.command=MAV_CMD_NAV_LAND;command.expires=101;
    require(!runtime.sendCommand(command),"failed configure admitted a command");
    read.id="after-failed-config";require(!runtime.parameter(read),"failed configure admitted protocol work");
    require(!runtime.load(provider),"invalid configuration was silently reusable");
    bool restart_required=false;
    try {runtime.configure("replace_backend","true");}catch(const std::exception&){restart_required=true;}
    require(restart_required,"invalid configuration accepted further settings");
    runtime.tick(100);
  }
  require(effects.observers_destroyed==1,"composition destroyed twice");
  require(effects.sends==0,"caller continuation after rejected configure attempted I/O");
}
void syntheticDelegation(const std::string& provider)
{
  Effects effects;double now=100;
  Runtime runtime(effects.publication(),effects.sender(),true,false,[&]{return now;});
  require(runtime.load(provider),"synthetic provider fixture not loaded");
  runtime.configure("synthetic_mode_77","true");fresh(runtime,now,5);
  require(runtime.status().find("control_profile=test_mode_77")!=std::string::npos,"synthetic provider not selected");
  require(!runtime.navigation({90,1,5,101}),"provider policy bypassed host permission");
  require(runtime.acquireNavigation("synthetic"),"host hard-coded Copter acquisition semantics");
  require(!runtime.navigation({90,1,5,101}),"setup grace was treated as eligible motion");
  runtime.tick(now);
  unsigned mode77=0;
  for(const auto& frame:effects.frames)if(frame.msgid==MAVLINK_MSG_ID_COMMAND_LONG) {
    mavlink_command_long_t command{};mavlink_msg_command_long_decode(&frame,&command);
    if(command.command==MAV_CMD_DO_SET_MODE) {
      require(command.param2==77,"host rewrote provider's synthetic acquisition mode");++mode77;
    }
  }
  require(mode77==1,"provider acquisition command was not transmitted exactly once");
  now=100.1;fresh(runtime,now,77);
  require(!runtime.navigation({90,1,5,100.05}),"provider bypassed host expiry admission");
  require(runtime.navigation({90,1,5,100.25}),"host rejected provider-specific active mode");runtime.tick(now);
  require(effects.count(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED)==1,"synthetic active provider did not produce NAV");
  runtime.revokeNavigation("synthetic_release");
  require(!runtime.navigation({90,1,5,100.25}),"provider bypassed host revocation");
  now=100.2;fresh(runtime,now,77);runtime.tick(now);
  require(runtime.acquireNavigation("new-synthetic"),"synthetic explicit reacquisition denied");
  require(runtime.navigation({90,1,5,100.25}),"synthetic waiting target denied");
  now=100.3;fresh(runtime,now,77);runtime.tick(now);
  require(effects.count(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED)==1,"provider bypassed final-send expiry gate");
}
}
int main(int argc,char** argv)
{
  try {
    require(argc==3,"usage: provider_lifetime_tests REPLACEABLE_PROVIDER_MODULE OBSERVER_MODULE");
    replacement(argv[1]);std::cout<<"PASS successful startup configuration refreshes provider\n";
    teardown(argv[1],argv[2]);std::cout<<"PASS provider detaches before reverse observer teardown\n";
    rejectedConfiguration(argv[1],argv[2]);std::cout<<"PASS rejected provider configuration closes effects and survives caller continuation\n";
    syntheticDelegation(argv[1]);std::cout<<"PASS synthetic mode77 policy delegates while host permission and expiry remain mandatory\n";
    std::cout<<"PASS 4 provider lifetime/delegation groups; fake callbacks, no sockets or vehicle I/O\n";return 0;
  }catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
