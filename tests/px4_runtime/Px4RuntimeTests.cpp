#include "Runtime.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>

using namespace mavmoos::runtime;
namespace {
void require(bool value,const std::string& why)
{ if(!value)throw std::runtime_error(why); }

bool isTarget(const mavlink_message_t& m)
{ return m.msgid==MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED; }
bool isGlobalTarget(const mavlink_message_t& m)
{ return m.msgid==MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT; }
bool isMode(const mavlink_message_t& m)
{
  if(m.msgid!=MAVLINK_MSG_ID_COMMAND_LONG)return false;
  mavlink_command_long_t p{};mavlink_msg_command_long_decode(&m,&p);
  if(p.command==MAV_CMD_DO_SET_MODE)return true;
  require(p.command==MAV_CMD_SET_MESSAGE_INTERVAL &&
    ((p.param1==MAVLINK_MSG_ID_ESTIMATOR_STATUS && p.param2==200000) ||
     (p.param1==MAVLINK_MSG_ID_CURRENT_MODE && p.param2==500000) ||
     (p.param1==MAVLINK_MSG_ID_VFR_HUD && p.param2==200000)) &&
    p.target_system==1 && p.target_component==1 && p.confirmation==0,
    "unexpected PX4 command or observation demand");
  return false;
}

struct Rig {
  enum class Airframe { Quad, FixedWing, Vtol };
  double now=10, cancel_at=-1;
  unsigned mode=px4::hold,intended=0;
  uint8_t type=MAV_TYPE_QUADROTOR,autopilot=MAV_AUTOPILOT_PX4;
  uint8_t vtol_state=MAV_VTOL_STATE_UNDEFINED;
  bool healthy=true,armed=true,grounded=false,feed_local=true,expire_mode_in_sender=false;
  bool feed_current_mode=true,feed_airspeed=false,feed_global=true;
  int32_t home_msl=489400;
  int32_t relative_mm=8000,latitude_e7=473977430,longitude_e7=85455940;
  int16_t north_speed_cm_s=0,east_speed_cm_s=0;
  float north=2,east=3,down=-8,airspeed=16;
  double course=90,speed=2,height=9;
  std::vector<std::string> results;
  Runtime host;
  std::vector<std::pair<double,mavlink_message_t>> sent;
  std::function<void(const mavlink_message_t&)> before_handoff;

  Rig(const char* provider,bool simulation=true,bool shadow=false)
    :Rig(provider,Airframe::Quad,simulation,shadow) {}
  Rig(const char* provider,Airframe airframe,bool simulation=true,bool shadow=false)
    :type(airframe==Airframe::Quad?MAV_TYPE_QUADROTOR:
       airframe==Airframe::FixedWing?MAV_TYPE_FIXED_WING:MAV_TYPE_VTOL_FIXEDROTOR),
     host([this](const auto& key,const auto& value,double,bool) {
       if(key=="MAVMOOS_COMMAND_RESULT") results.push_back(value);
     },[](const auto*,std::size_t){return false;},
          simulation,shadow,[this]{return now;},type,MAV_AUTOPILOT_PX4)
  {
    const bool fixed_wing=airframe==Airframe::FixedWing;
    if(fixed_wing) {
      relative_mm=55000;down=-55;
      east_speed_cm_s=1600;speed=16;height=60;
      feed_airspeed=true;
    }
    if(airframe==Airframe::Vtol) {
      relative_mm=30000;down=-30;height=30;
      vtol_state=MAV_VTOL_STATE_MC;feed_airspeed=true;
    }
    require(host.load(provider,"px4_control"),host.lastConfigurationError());
    host.configure("vehicle_profile",fixed_wing?"px4_fixed_wing":
      airframe==Airframe::Vtol?"px4_standard_vtol":"px4_multicopter");
    host.configure("control_native_actions","false");
    host.configure("shadow_mode",shadow?"true":"false");
    host.setFirmwareRequestsEnabled(false);
    host.setBoundedSend([this](const uint8_t* bytes,std::size_t size,double expires,const Runtime::SendEligibility& gate) {
      mavlink_message_t message{},rx{};
      mavlink_status_t codec{},status{};
      bool accepted=false;
      for(std::size_t i=0;i<size;++i)
        accepted=mavlink_frame_char_buffer(&rx,&codec,bytes[i],&message,&status)==MAVLINK_FRAMING_OK;
      require(accepted && message.sysid==245 && message.compid==191,"bad runtime wire frame");
      if(expire_mode_in_sender && isMode(message)) now=expires;
      if((isTarget(message)||isGlobalTarget(message)) && cancel_at>0 && now>=cancel_at) {
        cancel_at=-1;host.revokeNavigation("test_last_handoff_cancel");
      }
      if(before_handoff)before_handoff(message);
      if(!gate(now))return false;
      sent.emplace_back(now,message);
      return true;
    });
  }

  void heartbeat(double effective=-1,uint8_t system=1,uint8_t component=1)
  {
    mavlink_message_t m{};
    mavlink_msg_heartbeat_pack(system,component,&m,type,autopilot,
      MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|(armed?MAV_MODE_FLAG_SAFETY_ARMED:0),mode,MAV_STATE_ACTIVE);
    host.message(m,effective<0?now:effective,now);
  }
  void currentMode(unsigned active,unsigned requested,double effective=-1,
                   uint8_t system=1,uint8_t component=1)
  {
    mavlink_message_t m{};
    mavlink_msg_current_mode_pack(system,component,&m,0,active,requested);
    host.message(m,effective<0?now:effective,now);
  }
  void home()
  {
    mavlink_message_t m{};
    const float q[]{1,0,0,0};
    mavlink_msg_home_position_pack(1,1,&m,473977430,85455940,home_msl,0,0,0,q,0,0,0,1);
    host.message(m,now);
  }
  void localPosition(uint32_t boot,uint8_t system=1,uint8_t component=1)
  {
    mavlink_message_t m{};
    mavlink_msg_local_position_ned_pack(system,component,&m,boot,north,east,down,
      north_speed_cm_s/100.f,east_speed_cm_s/100.f,0);
    host.message(m,now);
  }
  void airspeedReport(double effective=-1,uint8_t system=1,uint8_t component=1)
  {
    mavlink_message_t m{};
    mavlink_msg_vfr_hud_pack(system,component,&m,airspeed,16,90,50,
      static_cast<float>((489400+relative_mm)/1000.),0);
    host.message(m,effective<0?now:effective,now);
  }
  void globalPosition(uint32_t boot)
  {
    mavlink_message_t m{};
    mavlink_msg_global_position_int_pack(1,1,&m,boot,latitude_e7,longitude_e7,489400+relative_mm,
      relative_mm,north_speed_cm_s,east_speed_cm_s,0,0);
    host.message(m,now);
  }
  void samples()
  {
    mavlink_message_t m{};
    const auto boot=static_cast<uint32_t>(now*1000);
    heartbeat();
    if(feed_current_mode)currentMode(mode,intended?intended:mode);
    if(feed_global)globalPosition(boot);
    home();
    mavlink_msg_extended_sys_state_pack(1,1,&m,vtol_state,
      armed&&!grounded?MAV_LANDED_STATE_IN_AIR:MAV_LANDED_STATE_ON_GROUND);
    host.message(m,now);
    mavlink_msg_estimator_status_pack(1,1,&m,static_cast<uint64_t>(now*1e6),healthy?1023:229,
      0,0,0,0,0,0,.2,.3);
    host.message(m,now);
    if(feed_local)localPosition(boot);
    if(feed_airspeed)airspeedReport();
  }
  void mail(const char* key,const char* value,const char* source="pHelmIvP")
  { host.mail({key,value,source,0,now,now,false}); }
  void field(const char* key,double value,const char* source)
  { host.mail({key,"",source,value,now,now,true}); }
  void prepare()
  {
    samples();mail("IVPHELM_STATE","DRIVE");mail("IVPHELM_ALLSTOP","clear");
  }
  void acquire()
  { mail("MAVMOOS_NAV_CONTROL","acquire","operator"); }
  void step(bool helm=true,const char* source="pHelmIvP",double dt=.052)
  {
    now+=dt;samples();
    if(helm) {
      mail("IVPHELM_STATE","DRIVE",source);
      field("DESIRED_HEADING",course,source);field("DESIRED_SPEED",speed,source);
      field("DESIRED_ALTITUDE",height,source);
    }
    host.tick(now);
  }
  void ack(uint16_t command,uint8_t system=1,uint8_t component=1,uint8_t target=245,uint8_t target_component=191,
           double queue_age=0,uint8_t result=MAV_RESULT_ACCEPTED)
  {
    now+=.001;
    mavlink_message_t message{};
    mavlink_msg_command_ack_pack(system,component,&message,command,result,0,0,target,target_component);
    host.message(message,now-queue_age,now);
  }
  std::vector<mavlink_command_long_t> nativeCommands() const
  {
    std::vector<mavlink_command_long_t> commands;
    for(const auto& frame:sent) if(frame.second.msgid==MAVLINK_MSG_ID_COMMAND_LONG) {
      mavlink_command_long_t command{};mavlink_msg_command_long_decode(&frame.second,&command);
      if(command.command!=MAV_CMD_SET_MESSAGE_INTERVAL) commands.push_back(command);
    }
    return commands;
  }
  bool result(const std::string& id,const std::string& status) const
  {
    return std::any_of(results.begin(),results.end(),[&](const auto& event) {
      return event.rfind("id="+id+",",0)==0 && event.find(",status="+status+",")!=std::string::npos;
    });
  }
  int modes() const
  { return std::count_if(sent.begin(),sent.end(),[](const auto& p){return isMode(p.second);}); }
  std::vector<uint32_t> observationDemands() const
  {
    std::vector<uint32_t> ids;
    for(const auto& frame:sent) if(frame.second.msgid==MAVLINK_MSG_ID_COMMAND_LONG && !isMode(frame.second)) {
      mavlink_command_long_t command{};
      mavlink_msg_command_long_decode(&frame.second,&command);
      ids.push_back(static_cast<uint32_t>(command.param1));
    }
    std::sort(ids.begin(),ids.end());
    return ids;
  }
  std::vector<mavlink_set_position_target_local_ned_t> targets() const
  {
    std::vector<mavlink_set_position_target_local_ned_t> t;
    for(const auto& p:sent) if(isTarget(p.second)) {
      mavlink_set_position_target_local_ned_t n{};
      mavlink_msg_set_position_target_local_ned_decode(&p.second,&n);
      require(n.target_system==1 && n.target_component==1 && n.coordinate_frame==MAV_FRAME_LOCAL_NED,
              "wrong PX4 target route or coordinate frame");
      t.push_back(n);
    }
    return t;
  }
  std::vector<mavlink_set_position_target_global_int_t> globalTargets() const
  {
    std::vector<mavlink_set_position_target_global_int_t> targets;
    for(const auto& frame:sent) if(isGlobalTarget(frame.second)) {
      mavlink_set_position_target_global_int_t target{};
      mavlink_msg_set_position_target_global_int_decode(&frame.second,&target);
      require(target.target_system==1 && target.target_component==1 &&
              target.coordinate_frame==MAV_FRAME_GLOBAL_INT,"wrong fixed-wing target route or altitude frame");
      targets.push_back(target);
    }
    return targets;
  }
  bool state(std::string_view needle) const { return host.status().find(needle)!=std::string::npos; }

  void startOffboard()
  {
    prepare();acquire();
    for(int i=0;i<40 && modes()==0;++i)step();
    require(modes()==1,"fixture never sent a qualified Offboard request");
    mode=px4::offboard;
    for(int i=0;i<6;++i)step();
    require(host.navigationGranted() && !targets().empty() && (targets().back().type_mask&7)==7,
            "fixture never sent motion from observed Offboard");
  }
};

void runSuccess(const char* module)
{
  Rig a(module);a.prepare();a.acquire();
  for(int i=0;i<5;++i)a.step(true,"not-helm");
  require(a.targets().empty(),"wrong MOOS source started prestream");
  for(int i=0;i<40 && a.modes()==0;++i)a.step();
  require(a.modes()==1,"expected one mode send: "+a.host.status());
  require(a.observationDemands()==std::vector<uint32_t>{MAVLINK_MSG_ID_ESTIMATOR_STATUS,MAVLINK_MSG_ID_CURRENT_MODE},
          "quadrotor observation demands were absent, repeated or altered");
  const auto command=std::find_if(a.sent.begin(),a.sent.end(),[](const auto& p){return isMode(p.second);});
  const auto first=std::find_if(a.sent.begin(),a.sent.end(),[](const auto& p){return isTarget(p.second);});
  require(first!=a.sent.end() && command->first-first->first>=1.25,"short Offboard prestream");
  mavlink_command_long_t p{};
  mavlink_msg_command_long_decode(&command->second,&p);
  require(p.command==MAV_CMD_DO_SET_MODE && p.param1==1 && p.param2==6 && p.param3==0 &&
    p.target_system==1 && p.target_component==1 && p.confirmation==0,"wrong PX4 mode transaction");
  auto holds=a.targets();
  require(holds.size()>=8,"insufficient prestream setpoints");
  for(const auto& n:holds) require(n.x==2&&n.y==3&&n.z==-8 && (n.type_mask&7)==0 &&
    (n.type_mask&56)==56,"prestream moved or altered the captured hold");
  a.mode=px4::offboard;
  a.now+=.01;a.heartbeat(command->first-.001);
  require(a.state("px4_offboard_confirmed=0"),"pre-send heartbeat confirmed entry");
  a.step();
  require(a.state("px4_offboard_confirmed=1") && a.state("nav_tx_accepted=0"),
          "fresh FC mode or post-confirmation stock input boundary missing");
  for(int i=0;i<5;++i)a.step();
  auto movement=a.targets().back();
  require(a.modes()==1 && a.state("px4_offboard_motion") && (movement.type_mask&7)==7 &&
    (movement.type_mask&56)==0 && std::abs(movement.vx)<.01 && movement.vy==2 &&
    movement.vz==-.5 && std::abs(movement.yaw-1.5707963)<.01,"incorrect east/up/yaw translation");

  a.now+=.015;a.mail("IVPHELM_ALLSTOP","NothingToDo");
  for(int i=0;i<3;++i)a.step(false);
  require(a.state("resume_permit=true") && (a.targets().back().type_mask&7)==0,
          "normal pause did not hold");
  a.now+=.01;a.mail("IVPHELM_ALLSTOP","clear");
  for(int i=0;i<5;++i)a.step();
  require(a.modes()==1 && (a.targets().back().type_mask&7)==7,"normal pause failed to resume without mode change");

  a.now+=.01;a.mail("MAVMOOS_NAV_CONTROL","release","operator");
  const auto stopped=a.targets().size();
  for(int i=0;i<8;++i)a.step();
  a.acquire();
  for(int i=0;i<8;++i)a.step();
  require(a.targets().size()==stopped && a.modes()==1 && a.state("permission=false"),
          "release or retained Offboard allowed motion or refreshed the stream");
  a.mode=px4::hold;a.intended=px4::offboard;
  a.step();a.acquire();
  for(int i=0;i<8;++i)a.step();
  require(a.targets().size()==stopped && a.state("px4_native_hold_ready=0"),
    "failsafe Hold with retained Offboard intention restarted the stream");
  a.intended=px4::hold;a.step();a.acquire();
  for(int i=0;i<40 && a.modes()<2;++i)a.step();
  require(a.modes()==2,"new explicit acquisition from Hold did not start a second prestream");
  a.mode=px4::offboard;
  for(int i=0;i<6;++i)a.step();
  require((a.targets().back().type_mask&7)==7,"explicit acquisition did not restore fresh control");
  a.mode=4u<<16; // independent native mission/other AUTO mode
  const auto takeover=a.targets().size();
  for(int i=0;i<6;++i)a.step();
  require(a.targets().size()==takeover && a.modes()==2 && a.state("permission=false") &&
    a.state("reason=external_mode_change"),"active pilot-mode takeover did not retire navigation");
}

void modeEvidence(const char* module)
{
  Rig missing(module);missing.feed_current_mode=false;missing.prepare();
  missing.acquire();missing.step();
  require(missing.targets().empty() && missing.modes()==0 && !missing.host.navigationGranted(),
          "Hold heartbeat alone admitted prestream");
  missing.now+=.01;missing.currentMode(px4::hold,px4::hold,-1,1,42);
  missing.currentMode(px4::hold,px4::hold,missing.now-.2);
  require(missing.state("px4_native_hold_ready=0"),
          "foreign or buffered CURRENT_MODE admitted native Hold readiness");
  missing.currentMode(px4::hold,px4::hold);
  require(missing.state("px4_native_hold_ready=1"),
          "fresh bound CURRENT_MODE did not confirm native Hold");
  for(int i=0;i<34;++i)missing.step();
  missing.acquire();missing.step();
  require(missing.state("px4_native_hold_ready=0") && missing.targets().empty() &&
          missing.modes()==0 && !missing.host.navigationGranted(),
          "expired CURRENT_MODE authorized prestream with fresh heartbeat and position");
  missing.feed_current_mode=true;missing.step();
  require(missing.state("px4_native_hold_ready=1") && missing.targets().empty(),
          "fresh Hold evidence revived a denied acquisition");
  missing.acquire();
  for(int i=0;i<8;++i)missing.step();
  require(!missing.targets().empty() && missing.modes()==0 && missing.host.navigationGranted(),
          "explicit acquisition did not start prestream with renewed Hold evidence");

  Rig changed(module);changed.prepare();changed.acquire();
  bool reached_mode_handoff=false;
  changed.before_handoff=[&](const auto& message) {
    if(!isMode(message))return;
    reached_mode_handoff=true;
    changed.intended=px4::offboard;
    changed.currentMode(px4::hold,px4::offboard);
  };
  for(int i=0;i<40 && !reached_mode_handoff;++i)changed.step();
  const auto count=changed.targets().size();
  for(int i=0;i<8;++i)changed.step();
  require(reached_mode_handoff && count>=8 && changed.modes()==0 &&
          changed.targets().size()==count && !changed.host.navigationGranted(),
          "retained Offboard intent at socket handoff left a mode request or renewed the stream");
}

void identityAndFacing(const char* module)
{
  for(const auto type:{MAV_TYPE_FIXED_WING,MAV_TYPE_VTOL_TILTROTOR,MAV_TYPE_GROUND_ROVER,MAV_TYPE_HEXAROTOR}) {
    Rig foreign(module);foreign.type=type;foreign.prepare();foreign.acquire();foreign.step();
    require(!foreign.host.vehicle().heartbeat_valid && !foreign.host.navigationGranted() &&
            foreign.targets().empty() && foreign.modes()==0,
            "non-quad PX4 heartbeat admitted Offboard");
  }
  Rig different_firmware(module);different_firmware.autopilot=MAV_AUTOPILOT_ARDUPILOTMEGA;
  different_firmware.prepare();different_firmware.acquire();different_firmware.step();
  require(!different_firmware.host.vehicle().heartbeat_valid &&
          !different_firmware.host.navigationGranted() && different_firmware.targets().empty() &&
          different_firmware.modes()==0,"ArduPilot quad heartbeat admitted PX4 Offboard");
  Rig passive(module,false);passive.prepare();passive.acquire();passive.step();
  require(passive.sent.empty() && !passive.host.navigationGranted(),
          "PX4 outside simulation admitted effects");

  struct FacingCase {
    const char* policy;
    double speed,height;
    float yaw,vertical_speed;
    bool yaw_ignored;
  };
  const FacingCase variants[]{
    {"keep",2,9,0,-.5f,true},
    {"heading",0,30,-1.5707963f,-1,false},
    {"course",.1,2,0,1,true}
  };
  for(const auto& variant:variants) {
    Rig rig(module);rig.speed=variant.speed;rig.height=variant.height;
    rig.host.configure("facing_policy",variant.policy);
    rig.host.configure("facing_heading","270");
    rig.startOffboard();
    const auto target=rig.targets().back();
    require(std::abs(target.vx)<.001 && std::abs(target.vy-variant.speed)<.001 &&
            std::abs(target.vz-variant.vertical_speed)<.001 &&
            bool(target.type_mask&POSITION_TARGET_TYPEMASK_YAW_IGNORE)==variant.yaw_ignored &&
            (variant.yaw_ignored || std::abs(target.yaw-variant.yaw)<.001),
            std::string("incorrect PX4 target or yaw mask for ")+variant.policy);
  }
}

void retiredStream(const char* module)
{
  Rig pending(module);pending.prepare();pending.acquire();
  for(int i=0;i<7;++i)pending.step();
  const auto holds=pending.targets().size();
  require(holds>0 && pending.modes()==0,"release fixture did not have a partial prestream");
  pending.now+=.01;pending.mail("MAVMOOS_NAV_CONTROL","release","operator");
  pending.mode=px4::offboard;
  for(int i=0;i<8;++i)pending.step();
  require(!pending.host.navigationGranted() && pending.modes()==0 && pending.targets().size()==holds,
          "release during prestream let an external Offboard mode inherit control");

  Rig home_changed(module);home_changed.startOffboard();
  const auto before=home_changed.targets().size();
  bool reached_motion_handoff=false;
  home_changed.before_handoff=[&](const auto& message) {
    if(reached_motion_handoff || !isTarget(message))return;
    mavlink_set_position_target_local_ned_t target{};
    mavlink_msg_set_position_target_local_ned_decode(&message,&target);
    require((target.type_mask&7)==7,"HOME-change fixture intercepted a non-motion target");
    reached_motion_handoff=true;
    home_changed.home_msl+=84;
    home_changed.home();
  };
  for(int i=0;i<8 && !reached_motion_handoff;++i)home_changed.step();
  require(reached_motion_handoff && home_changed.targets().size()==before &&
          !home_changed.host.navigationGranted() && home_changed.state("reason=home_changed"),
          "HOME change at final handoff failed to retire the encoded motion target");
  home_changed.mode=px4::hold;
  for(int i=0;i<8;++i)home_changed.step();
  require(home_changed.state("px4_native_hold_ready=1") && home_changed.targets().size()==before &&
          home_changed.modes()==1 && !home_changed.host.navigationGranted(),
          "fresh HOME and Hold revived authority without an explicit acquire");
  home_changed.acquire();
  for(int i=0;i<40 && home_changed.modes()<2;++i)home_changed.step();
  require(home_changed.modes()==2 && home_changed.targets().size()>before,
          "fresh explicit acquire could not establish a new grant after a HOME change");

  Rig expired(module);expired.host.configure("handoff_window","0.2");expired.startOffboard();
  expired.now+=.01;expired.mail("IVPHELM_ALLSTOP","NoGoalBehavior");
  expired.step(false);expired.step(false);
  require(expired.state("resume_permit=true") && (expired.targets().back().type_mask&7)==0,
          "NoGoalBehavior did not suspend into the captured hold");
  for(int i=0;i<5;++i)expired.step(false);
  const auto expired_count=expired.targets().size();
  expired.now+=.01;expired.mail("IVPHELM_ALLSTOP","clear");
  for(int i=0;i<7;++i)expired.step();
  require(!expired.host.navigationGranted() && expired.state("resume_permit=false") &&
          expired.modes()==1 && expired.targets().size()==expired_count,
          "late clear restarted a stream after the bounded hold expired");
}

void runFaults(const char* module)
{
  Rig foreign(module);foreign.prepare();foreign.acquire();
  for(int i=0;i<7;++i)foreign.step();
  const auto partial=foreign.targets().size();
  foreign.mode=px4::offboard;
  for(int i=0;i<7;++i)foreign.step();
  require(partial>0 && foreign.targets().size()==partial && foreign.modes()==0,
          "unrequested Offboard inherited the prestream");

  Rig late(module);late.prepare();late.acquire();late.expire_mode_in_sender=true;
  for(int i=0;i<33;++i)late.step();
  require(late.modes()==0 && late.state("permission=false"),"expired Offboard request left the socket or retained permission");

  Rig bad(module);bad.prepare();bad.acquire();
  for(int i=0;i<4;++i)bad.step();
  const auto before=bad.targets().size();
  bad.healthy=false;
  for(int i=0;i<4;++i)bad.step();
  require(before>0 && bad.targets().size()==before && bad.state("permission=false"),
          "estimator invalidation did not withdraw prestream");
  bad.healthy=true;bad.step();bad.acquire();bad.step();bad.north=7;
  const auto unchanged=bad.targets().size();bad.step();
  require(bad.targets().size()==unchanged && bad.state("permission=false"),
          "local reference jump moved the anchor or preserved old permission");

  Rig cancelled(module);cancelled.prepare();cancelled.acquire();
  cancelled.cancel_at=cancelled.now+.01;
  for(int i=0;i<8;++i)cancelled.step();
  require(cancelled.targets().empty() && cancelled.modes()==0,"last handoff cancellation sent setpoint");

  Rig shadow(module,true,true);shadow.prepare();shadow.acquire();
  for(int i=0;i<35;++i)shadow.step();
  require(shadow.sent.empty(),"PX4 shadow emitted MAVLink");
  Rig disarmed(module);disarmed.armed=false;disarmed.prepare();disarmed.acquire();
  for(int i=0;i<7;++i)disarmed.step();
  require(disarmed.targets().empty() && disarmed.modes()==0,"disarmed PX4 admitted Offboard");
}

constexpr uint16_t ignored_position_target_fields=POSITION_TARGET_TYPEMASK_VX_IGNORE|
  POSITION_TARGET_TYPEMASK_VY_IGNORE|POSITION_TARGET_TYPEMASK_VZ_IGNORE|
  POSITION_TARGET_TYPEMASK_AX_IGNORE|POSITION_TARGET_TYPEMASK_AY_IGNORE|POSITION_TARGET_TYPEMASK_AZ_IGNORE|
  POSITION_TARGET_TYPEMASK_YAW_IGNORE|POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
void sendFixedWingEntry(Rig& rig,int expected_mode_count)
{
  for(int i=0;i<40 && rig.modes()<expected_mode_count;++i)rig.step();
  require(rig.modes()==expected_mode_count,"fixture never sent fixed-wing Offboard entry: "+rig.host.status());
}

void confirmFixedWingMotion(Rig& rig)
{
  const auto before=rig.globalTargets().size();
  rig.mode=px4::offboard;
  rig.step();
  require(rig.state("px4_offboard_confirmed=1") && rig.globalTargets().size()==before,
          "fixed wing moved without new post-confirmation helm input");
  for(int i=0;i<5;++i)rig.step();
  require(rig.host.navigationGranted() && rig.globalTargets().size()>before,
          "fixed-wing Offboard never issued a position lookahead: "+rig.host.status());
}

void fixedWingWorkflow(const char* module)
{
  Rig rig(module,Rig::Airframe::FixedWing);
  rig.prepare();
  for(int i=0;i<7;++i)rig.step();
  require(rig.targets().empty() && rig.globalTargets().empty() && rig.modes()==0,
          "fixed-wing helm inputs started a stream without a grant");
  rig.acquire();
  for(int i=0;i<5;++i)rig.step(true,"not-helm");
  require(rig.targets().empty(),"wrong MOOS source started fixed-wing prestream");
  for(int i=0;i<6;++i)rig.step();
  require(!rig.targets().empty() && rig.modes()==0 &&
          std::abs(rig.targets().front().y-83)<.2,"prestream did not lead the observed ground track");
  rig.east+=1;
  sendFixedWingEntry(rig,1);
  require(rig.observationDemands()==std::vector<uint32_t>{MAVLINK_MSG_ID_VFR_HUD,MAVLINK_MSG_ID_ESTIMATOR_STATUS,
          MAVLINK_MSG_ID_CURRENT_MODE},"fixed-wing observation demands were absent, repeated or altered");
  const auto holds=rig.targets();
  const auto entry=std::find_if(rig.sent.begin(),rig.sent.end(),[](const auto& frame){return isMode(frame.second);});
  const auto first_hold=std::find_if(rig.sent.begin(),rig.sent.end(),[](const auto& frame){return isTarget(frame.second);});
  require(holds.size()>=8 && first_hold!=rig.sent.end() && entry!=rig.sent.end() &&
          entry->first-first_hold->first>=1.25 &&
          rig.globalTargets().empty(),"fixed-wing prestream lacked duration or started motion before Offboard");
  require(std::all_of(holds.begin(),holds.end(),[](const auto& n) {
            return n.type_mask==ignored_position_target_fields && n.x==2 && n.z==-55;
          }) && std::abs(holds.back().y-84)<.2,
          "fixed-wing prestream did not move a position-only lookahead with the observed aircraft");
  mavlink_command_long_t mode{};
  mavlink_msg_command_long_decode(&entry->second,&mode);
  require(mode.param1==MAV_MODE_FLAG_CUSTOM_MODE_ENABLED && mode.param2==6 && mode.param3==0 &&
          mode.confirmation==0 && mode.target_system==1 && mode.target_component==1,
          "fixed-wing mode request did not address the nominated PX4 Offboard mode");

  rig.now+=.01;
  mavlink_message_t ack{};
  mavlink_msg_command_ack_pack(1,1,&ack,MAV_CMD_DO_SET_MODE,MAV_RESULT_ACCEPTED,0,0,245,191);
  rig.host.message(ack,rig.now);
  rig.mode=px4::offboard;
  rig.heartbeat(-1,1,42);
  rig.heartbeat(entry->first-.001);
  require(rig.state("px4_offboard_confirmed=0") && rig.globalTargets().empty(),
          "ACK, wrong component or pre-send mode evidence confirmed fixed-wing Offboard");
  confirmFixedWingMotion(rig);

  constexpr double metres_per_degree=111195;
  const auto east=rig.globalTargets().back();
  const double east_north=(east.lat_int/1e7-47.397743)*metres_per_degree;
  const double east_east=(east.lon_int/1e7-8.545594)*metres_per_degree*std::cos(47.397743*std::acos(-1)/180);
  require(east.type_mask==ignored_position_target_fields && std::abs(east.alt-549.4)<.01 &&
          east.vx==0 && east.vy==0 && east.vz==0 && east.yaw==0 && east.yaw_rate==0 &&
          std::abs(east_north)<1 && std::abs(east_east-80)<2,
          "eastbound lookahead was not position-only and home-relative in the global MSL frame");
  rig.course=0;rig.speed=20;
  for(int i=0;i<5;++i)rig.step();
  const auto north=rig.globalTargets().back();
  require(std::abs((north.lat_int/1e7-47.397743)*metres_per_degree-100)<2 &&
          std::abs(north.lon_int/1e7-8.545594)*metres_per_degree<1,
          "northbound course and nominal speed did not replace the lookahead");

  rig.east_speed_cm_s=2000;
  rig.east+=4;
  rig.step(true,"pHelmIvP",.2);
  require(rig.host.navigationGranted(),"valid fixed-wing movement at reduced observation cadence caused a false reset");
  rig.north=3;
  rig.step();
  rig.now+=.015;rig.mail("IVPHELM_ALLSTOP","NothingToDo");
  const auto motion_before_pause=rig.globalTargets().size();
  const auto hold_before_pause=rig.targets().size();
  for(int i=0;i<3;++i)rig.step(false);
  const auto pause=rig.targets().back();
  require(rig.state("resume_permit=true") && rig.globalTargets().size()==motion_before_pause &&
          rig.targets().size()>hold_before_pause && pause.type_mask==ignored_position_target_fields &&
          pause.x==3 && std::abs(pause.y-108)<.2 && pause.z==-55,
          "normal handoff did not continue on the observed track with a position-only target");
  rig.now+=.01;rig.mail("IVPHELM_ALLSTOP","clear");
  for(int i=0;i<5;++i)rig.step();
  require(rig.modes()==1 && rig.globalTargets().size()>motion_before_pause,
          "normal handoff failed to resume position lookahead in the same Offboard grant");

  rig.now+=.01;rig.mail("MAVMOOS_NAV_CONTROL","release","operator");
  const auto stopped_motion=rig.globalTargets().size();
  const auto stopped_hold=rig.targets().size();
  for(int i=0;i<8;++i)rig.step();
  rig.acquire();
  rig.step();
  rig.mode=px4::hold;rig.intended=px4::offboard;
  rig.step();rig.acquire();
  for(int i=0;i<8;++i)rig.step();
  require(rig.globalTargets().size()==stopped_motion && rig.targets().size()==stopped_hold &&
          rig.modes()==1 && !rig.host.navigationGranted() && rig.state("px4_native_hold_ready=0"),
          "release or retained Offboard intent allowed fixed-wing reacquisition");
  rig.intended=px4::hold;
  for(int i=0;i<5;++i)rig.step();
  require(rig.targets().size()==stopped_hold && !rig.host.navigationGranted(),
          "native Hold and fresh inputs automatically revived an expired grant");
  rig.acquire();
  sendFixedWingEntry(rig,2);
  rig.intended=0;
  confirmFixedWingMotion(rig);

  const auto takeover_motion=rig.globalTargets().size();
  const auto takeover_hold=rig.targets().size();
  bool intercepted=false;
  rig.before_handoff=[&](const auto& message) {
    if(!isGlobalTarget(message))return;
    intercepted=true;
    rig.mode=1u<<16; // Pilot-selected Manual.
    rig.now+=.001;
    rig.heartbeat();
  };
  for(int i=0;i<8 && !intercepted;++i)rig.step();
  for(int i=0;i<8;++i)rig.step();
  require(intercepted && rig.globalTargets().size()==takeover_motion && rig.targets().size()==takeover_hold &&
          rig.modes()==2 && !rig.host.navigationGranted() && rig.state("reason=external_mode_change"),
          "pilot takeover between encoding and socket handoff sent the old position target");
}

void fixedWingFaults(const char* module)
{
  using Airframe=Rig::Airframe;
  for(const auto type:{MAV_TYPE_QUADROTOR,MAV_TYPE_VTOL_TILTROTOR,MAV_TYPE_GROUND_ROVER}) {
    Rig wrong(module,Airframe::FixedWing);wrong.type=type;
    wrong.prepare();wrong.acquire();wrong.step();
    require(!wrong.host.vehicle().heartbeat_valid && !wrong.host.navigationGranted() &&
            wrong.targets().empty() && wrong.globalTargets().empty() && wrong.modes()==0,
            "a different PX4 vehicle type acquired fixed-wing Offboard");
  }
  Rig wrong_fw(module,Airframe::FixedWing);wrong_fw.autopilot=MAV_AUTOPILOT_ARDUPILOTMEGA;
  wrong_fw.prepare();wrong_fw.acquire();wrong_fw.step();
  require(!wrong_fw.host.vehicle().heartbeat_valid && !wrong_fw.host.navigationGranted() &&
          wrong_fw.targets().empty(),"ArduPilot Plane identity opened PX4 fixed-wing effects");
  Rig vtol(module,Airframe::FixedWing);vtol.vtol_state=MAV_VTOL_STATE_FW;
  vtol.prepare();vtol.acquire();vtol.step();
  require(!vtol.host.navigationGranted() && vtol.targets().empty(),
          "VTOL state under fixed-wing heartbeat opened conventional Offboard");
  Rig invalid_configuration(module,Airframe::FixedWing);
  bool rejected=false;
  try { invalid_configuration.host.configure("facing_policy","heading"); }
  catch(const std::invalid_argument&) { rejected=true; }
  require(rejected,"fixed-wing profile accepted independent yaw configuration");
  Rig shadow(module,Airframe::FixedWing,true,true);shadow.prepare();shadow.acquire();
  for(int i=0;i<35;++i)shadow.step();
  require(shadow.sent.empty(),"fixed-wing shadow emitted MAVLink");
  Rig passive(module,Airframe::FixedWing,false);passive.prepare();passive.acquire();passive.step();
  require(passive.sent.empty() && !passive.host.navigationGranted(),
          "fixed-wing startup outside simulation admitted effects");

  Rig delayed(module,Airframe::FixedWing);delayed.prepare();delayed.acquire();delayed.expire_mode_in_sender=true;
  for(int i=0;i<38;++i)delayed.step();
  require(delayed.targets().size()>=8 && delayed.modes()==0 && !delayed.host.navigationGranted(),
          "expired fixed-wing mode request was sent or retained the grant");

  Rig malformed(module,Airframe::FixedWing);malformed.feed_local=false;
  malformed.prepare();malformed.localPosition(10000,1,42);malformed.acquire();malformed.step();
  require(!malformed.host.navigationGranted() && malformed.targets().empty(),
          "foreign local position opened fixed-wing prestream");
  malformed.feed_local=true;malformed.north=std::numeric_limits<float>::quiet_NaN();
  malformed.step();malformed.acquire();malformed.step();
  require(!malformed.host.navigationGranted() && malformed.targets().empty(),
          "nonfinite local position opened fixed-wing prestream");

  for(double speed:{std::numeric_limits<double>::quiet_NaN(),11.9,22.1}) {
    Rig invalid_helm(module,Airframe::FixedWing);invalid_helm.prepare();invalid_helm.acquire();
    invalid_helm.speed=speed;
    for(int i=0;i<6;++i)invalid_helm.step();
    require(!invalid_helm.host.navigationGranted() && invalid_helm.targets().empty() && invalid_helm.modes()==0,
            "invalid helm speed left fixed-wing prestream eligible");
  }

  Rig unobserved(module,Airframe::FixedWing);unobserved.feed_airspeed=false;
  unobserved.prepare();
  unobserved.airspeedReport(-1,42,1);unobserved.airspeedReport(-1,1,42);
  unobserved.airspeedReport(unobserved.now-1);
  unobserved.acquire();unobserved.step();
  require(!unobserved.host.navigationGranted() && unobserved.targets().empty(),
          "foreign or stale airspeed evidence granted fixed-wing authority");
  for(float airspeed:{std::numeric_limits<float>::quiet_NaN(),9.9f,35.1f}) {
    Rig outside(module,Airframe::FixedWing);outside.airspeed=airspeed;
    outside.prepare();outside.acquire();outside.step();
    require(!outside.host.navigationGranted() && outside.targets().empty(),
            "nonfinite or out-of-envelope observed airspeed admitted prestream");
  }

  Rig stale_air(module,Airframe::FixedWing);
  stale_air.prepare();stale_air.acquire();sendFixedWingEntry(stale_air,1);confirmFixedWingMotion(stale_air);
  stale_air.feed_airspeed=false;
  for(int i=0;i<16;++i)stale_air.step();
  const auto stale_motion=stale_air.globalTargets().size(),stale_hold=stale_air.targets().size();
  require(!stale_air.host.navigationGranted(),"expired airspeed did not revoke fixed-wing Offboard");
  stale_air.feed_airspeed=true;
  for(int i=0;i<8;++i)stale_air.step();
  require(!stale_air.host.navigationGranted() && stale_air.globalTargets().size()==stale_motion &&
          stale_air.targets().size()==stale_hold,"renewed airspeed restored expired fixed-wing authority");

  Rig transient_air(module,Airframe::FixedWing);
  transient_air.prepare();transient_air.acquire();sendFixedWingEntry(transient_air,1);
  confirmFixedWingMotion(transient_air);
  const auto before_airspeed_loss=transient_air.globalTargets().size();
  bool intercepted=false;
  transient_air.before_handoff=[&](const auto& message) {
    if(!isGlobalTarget(message))return;
    intercepted=true;
    transient_air.now+=.001;
    transient_air.airspeed=std::numeric_limits<float>::quiet_NaN();
    transient_air.airspeedReport();
    transient_air.now+=.001;
    transient_air.airspeed=16;
    transient_air.airspeedReport();
  };
  for(int i=0;i<8 && !intercepted;++i)transient_air.step();
  require(intercepted && !transient_air.host.navigationGranted() &&
          transient_air.globalTargets().size()==before_airspeed_loss,
          "airspeed invalidation at handoff did not fence the target after sensor recovery");

  Rig edge(module,Airframe::FixedWing);edge.latitude_e7+=39000;edge.north=435;
  edge.prepare();edge.acquire();sendFixedWingEntry(edge,1);
  edge.mode=px4::offboard;edge.course=0;
  for(int i=0;i<8;++i)edge.step();
  require(!edge.host.navigationGranted() && edge.globalTargets().empty(),
          "outbound lookahead crossed the fixed-wing control radius");
  Rig continuation_edge(module,Airframe::FixedWing);
  continuation_edge.latitude_e7+=39000;continuation_edge.north=435;
  continuation_edge.north_speed_cm_s=1600;continuation_edge.east_speed_cm_s=0;
  continuation_edge.prepare();continuation_edge.acquire();
  for(int i=0;i<7;++i)continuation_edge.step();
  require(!continuation_edge.host.navigationGranted() && continuation_edge.targets().empty() &&
          continuation_edge.modes()==0,"prestream continuation crossed the fixed-wing control radius");

  Rig loss(module,Airframe::FixedWing);loss.host.configure("handoff_window","0.2");
  loss.prepare();loss.acquire();sendFixedWingEntry(loss,1);confirmFixedWingMotion(loss);
  loss.now+=.01;loss.mail("IVPHELM_ALLSTOP","NoGoalBehavior");
  loss.step(false);loss.step(false);
  require(loss.state("resume_permit=true") && loss.targets().back().type_mask==ignored_position_target_fields,
          "NoGoalBehavior failed to enter bounded forward continuation");
  for(int i=0;i<6;++i)loss.step(false);
  const auto holds=loss.targets().size(),motion=loss.globalTargets().size();
  loss.now+=.01;loss.mail("IVPHELM_ALLSTOP","clear");
  for(int i=0;i<8;++i)loss.step();
  require(!loss.host.navigationGranted() && loss.state("resume_permit=false") &&
          loss.targets().size()==holds && loss.globalTargets().size()==motion,
          "late clear or fresh inputs restarted the expired fixed-wing pause");

  Rig jump(module,Airframe::FixedWing);
  jump.prepare();jump.acquire();sendFixedWingEntry(jump,1);confirmFixedWingMotion(jump);
  const auto prior_motion=jump.globalTargets().size();
  jump.north+=100;
  jump.step();
  for(int i=0;i<6;++i)jump.step();
  require(!jump.host.navigationGranted() && jump.globalTargets().size()==prior_motion &&
          jump.state("reason=px4_local_reference_changed"),
          "local reference discontinuity carried old fixed-wing authority across the reset");
}

void fixedWingGlobalPositionEvidence(const char* module)
{
  using Airframe=Rig::Airframe;
  Rig duplicate(module,Airframe::FixedWing);
  duplicate.prepare();duplicate.acquire();sendFixedWingEntry(duplicate,1);
  confirmFixedWingMotion(duplicate);
  const double original_at=duplicate.host.vehicle().position_at;
  const double original_latitude=duplicate.host.vehicle().latitude;
  const auto repeated_boot=static_cast<uint32_t>(original_at*1000);
  duplicate.feed_global=false;
  duplicate.latitude_e7+=1000;
  for(int i=0;i<12;++i) {
    duplicate.step();
    duplicate.globalPosition(repeated_boot);
  }
  require(duplicate.host.vehicle().position_at==original_at &&
          duplicate.host.vehicle().latitude==original_latitude && !duplicate.host.vehicle().position_valid &&
          !duplicate.host.navigationGranted(),"repeated global boot timestamp changed position or renewed expired authority");

  Rig regressed(module,Airframe::FixedWing);
  regressed.prepare();regressed.acquire();sendFixedWingEntry(regressed,1);
  confirmFixedWingMotion(regressed);
  const auto older_boot=static_cast<uint32_t>(regressed.host.vehicle().position_at*1000)-75;
  regressed.feed_global=false;
  regressed.step();
  const auto motion_before=regressed.globalTargets().size();
  require(regressed.state("px4_local_ready=1") && regressed.host.navigationGranted(),
          "older global position fixture did not preserve fresh local readiness");
  regressed.globalPosition(older_boot);
  require(!regressed.host.vehicle().position_valid && !regressed.host.navigationGranted(),
          "backwards global boot timestamp did not withdraw the fixed-wing grant");
  regressed.feed_global=true;
  regressed.mode=px4::hold;
  for(int i=0;i<6;++i)regressed.step();
  require(regressed.host.vehicle().position_valid && !regressed.host.navigationGranted() &&
          regressed.globalTargets().size()==motion_before && regressed.modes()==1,
          "fresh position and Hold revived authority without a new acquire");
  regressed.acquire();
  sendFixedWingEntry(regressed,2);
  confirmFixedWingMotion(regressed);
}
void nativeLifecycle(const char* module,Rig::Airframe airframe)
{
  const bool plane=airframe==Rig::Airframe::FixedWing,vtol=airframe==Rig::Airframe::Vtol;
  Rig rig(module,airframe);
  rig.host.configure("control_native_actions","true");
  rig.mode=px4::position;rig.armed=false;rig.grounded=true;
  rig.relative_mm=0;rig.down=0;
  rig.prepare();
  const auto request=[&](const char* command) {
    rig.now+=.001;rig.mail("MAVMOOS_COMMAND",command,"operator");rig.step(false);
  };
  request("id=early,action=takeoff,height=25");
  require(rig.result("early","REJECTED") && rig.nativeCommands().empty(),
          "unarmed PX4 takeoff reached the wire");
  request("id=invalid,action=arm,height=10");
  require(rig.result("invalid","REJECTED") && rig.nativeCommands().empty(),"arm accepted extraneous height");
  request("id=arm,action=arm");
  const auto armed_request=rig.nativeCommands();
  require(armed_request.size()==1 && armed_request[0].command==MAV_CMD_COMPONENT_ARM_DISARM &&
          armed_request[0].param1==1 && armed_request[0].param2==0,"unbounded PX4 arm wire request");
  rig.ack(MAV_CMD_COMPONENT_ARM_DISARM,1,1,245,191,.02);
  rig.step(false);
  require(rig.result("arm","ACK_OBSERVED") && !rig.result("arm","COMPLETED"),"arm completed from ACK alone");
  request("id=duplicate,action=disarm");
  require(rig.result("duplicate","REJECTED") && rig.nativeCommands().size()==1,"busy native command changed the wire");
  rig.armed=true;rig.step(false);
  require(rig.result("arm","COMPLETED"),"fresh armed evidence did not complete arm");
  request("id=arm,action=arm");
  require(rig.result("arm","DUPLICATE") && rig.nativeCommands().size()==1,"replayed id sent a second arm");

  request("id=ground_disarm,action=disarm");
  rig.ack(MAV_CMD_COMPONENT_ARM_DISARM,42,1,245,191);
  rig.ack(MAV_CMD_COMPONENT_ARM_DISARM,1,1,42,191);
  require(!rig.result("ground_disarm","ACK_OBSERVED") && !rig.result("ground_disarm","COMPLETED"),
          "foreign ACK correlated with disarm");
  rig.ack(MAV_CMD_COMPONENT_ARM_DISARM,1,1,245,191,0,MAV_RESULT_DENIED);
  require(rig.result("ground_disarm","ACK_OBSERVED") && !rig.result("ground_disarm","FAILED"),
          "delayed arm denial failed a disarm request");
  rig.armed=false;rig.step(false);
  require(rig.result("ground_disarm","COMPLETED"),"fresh grounded disarm did not complete");

  request("id=rearm,action=arm");
  rig.ack(MAV_CMD_COMPONENT_ARM_DISARM);
  rig.armed=true;rig.step(false);
  const double height=plane?45:vtol?30:8;
  const auto takeoff="id=takeoff,action=takeoff,height="+std::to_string(height);
  request(takeoff.c_str());
  const auto climb=rig.nativeCommands().back();
  require(climb.command==MAV_CMD_NAV_TAKEOFF && std::abs(climb.param7-rig.home_msl/1000.-height)<.01 &&
          std::abs(climb.param5-47.397743)<.01 && std::abs(climb.param6-8.545594)<.01,
          "takeoff target did not bind finite position and MSL height");
  rig.step(false);
  require(!rig.result("takeoff","COMPLETED"),"takeoff completed while grounded");
  for(unsigned i=0;i<32;++i) {rig.now+=.1;rig.step(false);}
  require(!rig.result("takeoff","TIMED_OUT"),"ACK silence shortened the original deadline");
  rig.mode=px4::takeoff;rig.grounded=false;rig.step(false);
  require(rig.result("takeoff","CONFIRMED") && !rig.result("takeoff","COMPLETED"),"native takeoff activation not distinct");
  rig.mode=px4::hold;rig.relative_mm=static_cast<int32_t>(height*1000);rig.down=-height;rig.step(false);
  require(rig.result("takeoff","COMPLETED"),"bounded airborne Hold did not complete takeoff");
  request("id=air_disarm,action=disarm");
  require(rig.result("air_disarm","REJECTED") && rig.nativeCommands().back().command==MAV_CMD_NAV_TAKEOFF,
          "airborne disarm escaped native gate");

  rig.intended=px4::offboard;rig.step(false);
  request("id=return_retained,action=rtl");
  require(rig.result("return_retained","REJECTED"),"RTL admitted from retained Offboard intent");
  request("id=hold,action=hold");
  require(rig.nativeCommands().back().command==MAV_CMD_DO_SET_MODE &&
          rig.nativeCommands().back().param2==4 && rig.nativeCommands().back().param3==3,
          "explicit native Hold was not sent from retained intent");
  rig.ack(MAV_CMD_DO_SET_MODE);
  rig.step(false);
  require(!rig.result("hold","COMPLETED"),"Hold completed with retained Offboard intent");
  rig.intended=0;rig.step(false);
  require(rig.result("hold","COMPLETED"),"fresh deliberate native Hold was not confirmed");
  request("id=return,action=rtl");
  rig.ack(MAV_CMD_NAV_RETURN_TO_LAUNCH);
  rig.mode=px4::rtl;rig.step(false);
  require(rig.result("return","COMPLETED"),"fresh native Return mode did not complete command");
  request("id=land,action=land");
  rig.ack(MAV_CMD_NAV_LAND);
  rig.mode=px4::land;rig.step(false);
  require(rig.result("land","CONFIRMED") && !rig.result("land","COMPLETED"),"Land completed before touchdown");
  rig.grounded=true;rig.step(false);
  require(rig.result("land","COMPLETED"),"observed touchdown did not complete Land");
}

void nativeFinalGate(const char* module)
{
  Rig rig(module);
  rig.host.configure("control_native_actions","true");
  rig.armed=false;rig.grounded=true;rig.mode=px4::position;rig.prepare();
  rig.now+=.001;
  rig.mail("MAVMOOS_COMMAND","id=arm,action=arm","operator");
  bool intercepted=false;
  rig.before_handoff=[&](const auto& message) {
    if(message.msgid!=MAVLINK_MSG_ID_COMMAND_LONG) return;
    mavlink_command_long_t cmd{};mavlink_msg_command_long_decode(&message,&cmd);
    if(cmd.command!=MAV_CMD_COMPONENT_ARM_DISARM) return;
    intercepted=true;
    rig.mode=px4::manual;rig.now+=.001;rig.samples();
  };
  rig.step(false);
  require(intercepted && rig.nativeCommands().empty() && rig.result("arm","FAILED"),
          "changed mode at endpoint proof allowed queued native work");

  Rig disabled(module);
  disabled.armed=false;disabled.grounded=true;disabled.prepare();
  disabled.now+=.001;
  disabled.mail("MAVMOOS_COMMAND","id=arm,action=arm","operator");
  disabled.step(false);
  require(disabled.nativeCommands().empty(),"default profile sent a native command");

  Rig delayed(module);
  delayed.host.configure("control_native_actions","true");
  delayed.armed=false;delayed.grounded=true;delayed.mode=px4::position;
  delayed.now+=1;delayed.prepare();
  delayed.host.mail({"MAVMOOS_COMMAND","id=delayed,action=arm","operator",0,delayed.now-.4,delayed.now,false});
  delayed.step(false,"pHelmIvP",.26);
  require(delayed.nativeCommands().empty()&&delayed.result("delayed","REJECTED"),
          "late local receipt extended the original native command expiry");

  Rig missing_ack(module);
  missing_ack.host.configure("control_native_actions","true");
  missing_ack.armed=false;missing_ack.grounded=true;missing_ack.prepare();
  missing_ack.now+=.001;
  missing_ack.mail("MAVMOOS_COMMAND","id=arm,action=arm","operator");
  missing_ack.step(false);
  missing_ack.armed=true;
  for(int i=0;i<64;++i) missing_ack.step(false);
  require(missing_ack.result("arm","COMPLETED") && !missing_ack.result("arm","ACK_OBSERVED"),
          "observed arm did not complete without ACK");
  missing_ack.mail("MAVMOOS_COMMAND","id=disarm,action=disarm","operator");
  missing_ack.step(false);
  for(int i=0;i<64;++i) missing_ack.step(false);
  require(!missing_ack.result("disarm","TIMED_OUT") && missing_ack.state("command_leases=1"),
          "unobserved effect lost its original lease early");
  for(int i=0;i<100;++i) missing_ack.step(false);
  require(missing_ack.result("disarm","TIMED_OUT") && !missing_ack.result("disarm","COMPLETED") &&
          missing_ack.nativeCommands().size()==2,"deadline claimed an effect or retried a command");
}

void vtolPhaseAuthority(const char* module)
{
  Rig rig(module,Rig::Airframe::Vtol);
  rig.host.configure("control_native_actions","true");
  rig.prepare();rig.acquire();
  for(int i=0;i<40 && rig.nativeCommands().empty();++i) rig.step();
  require(rig.nativeCommands().size()==1 && rig.nativeCommands().back().param2==6,"VTOL hover never prestreamed");
  rig.mode=px4::offboard;
  for(int i=0;i<8;++i) rig.step();
  require(rig.host.navigationGranted() && !rig.targets().empty() && (rig.targets().back().type_mask&7)==7 &&
          rig.globalTargets().empty(),"VTOL hover used the wrong guidance strategy");
  const auto count=rig.targets().size();
  bool intercepted=false;
  rig.before_handoff=[&](const auto& message) {
    if(!isTarget(message))return;
    intercepted=true;
    rig.now+=.001;
    rig.vtol_state=MAV_VTOL_STATE_TRANSITION_TO_FW;
    rig.samples();
  };
  for(int i=0;i<8 && !intercepted;++i) rig.step();
  require(intercepted && rig.targets().size()==count && !rig.host.navigationGranted(),
          "a hover target survived a VTOL phase transition at final send");
  rig.before_handoff={};
  rig.mode=px4::hold;rig.vtol_state=MAV_VTOL_STATE_FW;rig.speed=10;rig.airspeed=10;rig.east_speed_cm_s=1000;
  for(int i=0;i<8;++i) rig.step();
  require(rig.globalTargets().empty() && !rig.host.navigationGranted(),"stable new phase restored old authority");
  rig.acquire();
  for(int i=0;i<40 && rig.nativeCommands().size()==1;++i) rig.step();
  require(rig.nativeCommands().size()==2,"VTOL forward phase did not prestream");
  rig.mode=px4::offboard;
  for(int i=0;i<8;++i) rig.step();
  require(rig.host.navigationGranted() && !rig.globalTargets().empty(),"forward phase did not send a position target");
  rig.mode=px4::hold;rig.step(false);
  rig.now+=.001;rig.mail("MAVMOOS_COMMAND","id=back,action=transition_mc","operator");rig.step(false);
  require(rig.nativeCommands().back().command==MAV_CMD_DO_VTOL_TRANSITION &&
          rig.nativeCommands().back().param1==MAV_VTOL_STATE_MC,"native VTOL back-transition encoding differs");
  rig.ack(MAV_CMD_DO_VTOL_TRANSITION);
  rig.vtol_state=MAV_VTOL_STATE_TRANSITION_TO_MC;rig.step(false);
  require(!rig.result("back","COMPLETED"),"transition completed in a transient phase");
  rig.acquire();rig.step(false);
  require(!rig.host.navigationGranted(),"incomplete transition admitted helm authority");
  rig.vtol_state=MAV_VTOL_STATE_MC;rig.step(false);
  require(rig.result("back","COMPLETED") && !rig.host.navigationGranted(),"VTOL phase was not independently observed");
}

void nativeModeAgreement(const char* module)
{
  Rig rig(module);
  rig.host.configure("control_native_actions","true");
  rig.prepare();rig.acquire();
  for(int i=0;i<40 && !rig.modes();++i) rig.step();
  require(rig.modes()==1,"native-facet prestream did not request Offboard");

  rig.mode=px4::offboard;
  rig.intended=px4::hold;
  for(int i=0;i<8;++i) rig.step();
  const auto held=rig.targets();
  require(rig.state("px4_offboard_confirmed=0") && !held.empty() &&
          std::all_of(held.begin(),held.end(),[](const auto& target) {
            return target.type_mask==px4::position_mask;
          }),"Offboard heartbeat alone admitted motion in the native profile");

  rig.intended=px4::offboard;
  for(int i=0;i<8;++i) rig.step();
  require(rig.state("px4_offboard_confirmed=1") && (rig.targets().back().type_mask&7)==7,
          "fresh active and intended Offboard did not admit new stock helm motion");

  const auto before=rig.targets().size();
  rig.intended=px4::hold;
  rig.step();
  require(!rig.host.navigationGranted() && rig.targets().size()==before,
          "changed Offboard intention failed to fence motion");

  rig.intended=px4::offboard;
  for(int i=0;i<8;++i) rig.step();
  require(!rig.host.navigationGranted() && rig.targets().size()==before,
          "returning Offboard intent restored a retired helm grant");
}
} // namespace

int main(int argc,char** argv)
{
  try {
    require(argc==2 || (argc==3 && std::string_view(argv[2])=="--fixed-wing"),
            "pass the loaded PX4 provider and optional --fixed-wing");
    if(argc==3) {
      fixedWingWorkflow(argv[1]);fixedWingFaults(argv[1]);fixedWingGlobalPositionEvidence(argv[1]);
      nativeLifecycle(argv[1],Rig::Airframe::FixedWing);
      std::cout<<"PX4 fixed-wing lookahead, forward handoff, mode evidence and final-send tests passed\n";
    } else {
      runSuccess(argv[1]);modeEvidence(argv[1]);identityAndFacing(argv[1]);
      retiredStream(argv[1]);runFaults(argv[1]);
      nativeLifecycle(argv[1],Rig::Airframe::Quad);nativeLifecycle(argv[1],Rig::Airframe::Vtol);
      nativeFinalGate(argv[1]);nativeModeAgreement(argv[1]);vtolPhaseAuthority(argv[1]);
      std::cout<<"PX4 runtime, real module, stock-mail contract and final-gate fault paths passed\n";
    }
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<e.what()<<'\n';return 1;
  }
}
