#include "Runtime.h"
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace mavmoos::runtime;
namespace {
constexpr unsigned camera=100,gimbal=154;
void require(bool valid,const char* why)
{ if(!valid)throw std::runtime_error(why); }

class Rig {
public:
  Rig(char** args)
    :host([](const auto&,const auto&,double,bool){},[](const auto*,std::size_t){return false;},true,false,
      [this]{return at;},MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_ARDUPILOTMEGA,true)
  {
    require(host.load(args[1],"telemetry") && host.load(args[2],"camera_status") &&
      host.load(args[3],"gimbal_status"),"could not load actual payload modules");
    host.configure("camera_component","100");host.configure("gimbal_component","154");
    host.configureModules({{"camera_status",{{"capture_enabled","true"}}},
      {"gimbal_status",{{"point_enabled","true"}}}});
    host.setFirmwareRequestsEnabled(false);host.enablePayloadBench(camera,gimbal);
    host.finishStartup();
    host.setBoundedSend([this](const uint8_t* data,std::size_t size,double expires,const Runtime::SendEligibility& eligible) {
      mavlink_message_t rx{},m{};
      mavlink_status_t decoder{},res{};
      bool ok=false;
      for(std::size_t i=0;i<size;++i)
        ok=mavlink_frame_char_buffer(&rx,&decoder,data[i],&m,&res)==MAVLINK_FRAMING_OK;
      require(ok && m.sysid==245 && m.compid==191,"bad frame");
      mavlink_command_long_t command{};
      if(m.msgid==MAVLINK_MSG_ID_COMMAND_LONG)mavlink_msg_command_long_decode(&m,&command);
      const bool effect=command.command==MAV_CMD_IMAGE_START_CAPTURE ||
        command.command==MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE ||
        command.command==MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW;
      if(effect) {
        if(final_expiry) {final_expiry=false;at=expires;}
        if(final_cancel) {final_cancel=false;host.cancelPayload("camera/cancel");}
        if(final_arm) {final_arm=false;ground(true);}
      }
      const bool sent=eligible(at);
      if(effect && sent)commands.push_back(command);
      return sent;
    });
    ground(false);
  }

  void message(const mavlink_message_t& m) {at+=.001;host.message(m,at);}
  void ground(bool armed)
  {
    mavlink_message_t m{};
    mavlink_msg_heartbeat_pack_status(1,1,&codec,&m,MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_ARDUPILOTMEGA,
      armed?MAV_MODE_FLAG_SAFETY_ARMED:0,0,MAV_STATE_ACTIVE);
    message(m);
    mavlink_msg_extended_sys_state_pack_status(1,1,&codec,&m,0,
      armed?MAV_LANDED_STATE_IN_AIR:MAV_LANDED_STATE_ON_GROUND);
    message(m);
  }
  void cameraState(uint8_t source=camera,bool idle=true)
  {
    mavlink_message_t m{};
    mavlink_camera_information_t info{};
    info.flags=CAMERA_CAP_FLAGS_CAPTURE_IMAGE;
    mavlink_msg_camera_information_encode_status(1,source,&codec,&m,&info);
    message(m);
    mavlink_camera_capture_status_t status{};
    status.time_boot_ms=++camera_boot;status.image_count=1;status.image_status=idle?0:1;
    mavlink_msg_camera_capture_status_encode_status(1,source,&codec,&m,&status);
    message(m);
  }
  void manager(uint8_t system,uint8_t component,uint32_t flags=0,uint32_t boot=0)
  {
    mavlink_message_t m{};
    mavlink_msg_gimbal_manager_status_pack_status(1,gimbal,&codec,&m,boot?boot:++gimbal_boot,
      flags,gimbal,system,component,0,0);
    message(m);
  }
  PayloadRequest shot(const std::string& id,uint32_t seq=1) const
  {return {id,"camera/"+id,PayloadAction::CameraCaptureOne,at+.2,at+1,seq,0,0};}
  PayloadRequest rotate(PayloadAction action,const std::string& id,double until,float pitch=0,float yaw=0) const
  {return {id,"gimbal/one",action,at+.2,until,0,pitch,yaw};}
  void pump()
  { for(int i=0;i<8;++i) {at+=.01;host.tick(at);} }

  double at=100;
  Runtime host;
  mavlink_status_t codec{};
  uint32_t camera_boot=100,gimbal_boot=100;
  bool final_expiry=false,final_cancel=false,final_arm=false;
  std::vector<mavlink_command_long_t> commands;
};

void cameraChecks(char** args)
{
  Rig r(args);
  r.cameraState(camera+1);
  require(!r.host.payload(r.shot("foreign")),"foreign component qualified camera effect");
  r.cameraState(camera,false);
  require(!r.host.payload(r.shot("busy")),"busy camera admitted capture");
  r.cameraState();
  require(r.host.payload(r.shot("cancel")),"selected camera denied");
  r.final_cancel=true;r.pump();
  require(r.commands.empty(),"cancel at final gate sent camera effect");
  r.host.finishPayload("camera/cancel");
  require(r.host.payload(r.shot("expired",2)),"deadline test admission failed");
  r.final_expiry=true;r.pump();
  require(r.commands.empty(),"expired shot left fake syscall");
  r.host.finishPayload("camera/expired");
  require(r.host.payload(r.shot("armed",3)),"arm test admission failed");
  r.final_arm=true;r.pump();
  require(r.commands.empty(),"arming during final handoff allowed a shot");
  r.ground(false);r.cameraState();
  require(r.host.payload(r.shot("one",4)),"ground image could not queue");
  r.pump();
  require(r.commands.size()==1 && r.commands[0].command==MAV_CMD_IMAGE_START_CAPTURE &&
    r.commands[0].target_system==1 && r.commands[0].target_component==camera && r.commands[0].confirmation==0 &&
    r.commands[0].param1==0 && r.commands[0].param2==0 && r.commands[0].param3==1 &&
    r.commands[0].param4==4 && r.commands[0].param5==0 && r.commands[0].param6==0 &&
    std::isnan(r.commands[0].param7),"camera wire changed from single addressed shot");
  r.host.finishPayload("camera/one");
  require(!r.host.payload(r.shot("replay",4)),"a new ID replayed an attempted capture sequence");
}

void gimbalChecks(char** args)
{
  Rig r(args);
  const double until=r.at+4;
  r.manager(0,0,GIMBAL_MANAGER_FLAGS_RC_EXCLUSIVE);
  require(!r.host.payload(r.rotate(PayloadAction::GimbalClaim,"claim",until)),"host ignored RC exclusive claim");
  r.manager(0,0);
  require(r.host.payload(r.rotate(PayloadAction::GimbalClaim,"claim",until)),"unowned manager denied claim");
  r.pump();
  require(r.commands.size()==1 && r.commands.back().target_component==gimbal &&
    r.commands.back().param1==-2 && r.commands.back().param2==-2 &&
    r.commands.back().param3==-1 && r.commands.back().param4==-1 &&
    r.commands.back().param7==gimbal,"wrong ownership request");
  require(!r.host.payload(r.rotate(PayloadAction::GimbalPoint,"point",until,-30,40)),"point without observed ownership");
  r.manager(245,191);
  require(r.host.payload(r.rotate(PayloadAction::GimbalPoint,"point",until,-30,40)),"owned manager denied point");
  r.manager(42,190);r.pump();
  require(r.commands.size()==1,"takeover after admission did not fence motion");
  r.host.cancelPayload("gimbal/one");
  require(!r.host.payload(r.rotate(PayloadAction::GimbalRelease,"release",until)),
    "cancel released independent controller");
}
}

int main(int argc,char** argv)
{
  try {
    require(argc==4,"pass telemetry, camera and gimbal modules");
    cameraChecks(argv);gimbalChecks(argv);
    std::cout<<"Dedicated payload host gate passed addressed, expiry, cancellation, arming and takeover checks\n";
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<e.what()<<'\n';return 1;
  }
}
