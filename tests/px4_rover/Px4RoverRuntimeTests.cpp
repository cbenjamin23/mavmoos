#include "../protocol_faults/ProtocolSupport.h"
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
#include <iostream>
#include <unistd.h>

using namespace protocol_faults;
namespace {
constexpr char rover_status[]="MAVMOOS_PX4_ROVER_STATUS";
constexpr char recovery_status[]="MAVMOOS_AUTO_RECOVERY_STATUS";
constexpr char mission_result[]="MAVMOOS_MISSION_EXECUTION_RESULT";
constexpr unsigned manual=1u<<16;
constexpr int32_t latitude=473977430,longitude=85455940;
constexpr uint32_t mission_id=0x4321ABCD;

struct Slot {
  std::filesystem::path path;
  Slot() {
    auto name=(std::filesystem::temp_directory_path()/"mavmoos-px4-rover-XXXXXX").string();
    require(mkdtemp(name.data())!=nullptr,"could not create private recovery slot");
    path=name;
  }
  ~Slot() { std::error_code ignored;std::filesystem::remove_all(path,ignored); }
  bool marked() const { return std::filesystem::exists(path/"auto-may-continue"); }
};

bool motion(const mavlink_message_t& message)
{
  if(message.msgid!=MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED)return false;
  mavlink_set_position_target_local_ned_t target{};
  mavlink_msg_set_position_target_local_ned_decode(&message,&target);
  require(target.target_system==1 && target.target_component==1 &&
    target.coordinate_frame==MAV_FRAME_LOCAL_NED && target.type_mask==0x0DC7 && target.vz==0,
    "incorrect Ackermann target encoding");
  return std::hypot(target.vx,target.vy)>.01;
}

struct Rig {
  double now=100;
  unsigned mode=px4::hold,intended=0;
  bool armed=true;
  uint8_t current_system=1,current_component=1;
  unsigned gps_speed=5;
  std::string bad_setting,session;
  uint8_t plan_state=MISSION_STATE_NOT_STARTED,plan_mode=2;
  bool loaded_plan=false;
  std::vector<Frame> frames;
  std::vector<Publication> publications;
  std::function<void(const mavlink_message_t&,double)> before_handoff;
  std::size_t answered=0;
  Runtime host;

  Rig(const char* provider,const char* missions,const Slot& slot,std::string process="process-one")
    :session(std::move(process)),
     host([this](const auto& key,const auto& value,double,bool numeric) {
       if(!numeric)publications.push_back({key,value,now});
     },[](const uint8_t*,std::size_t) { throw std::runtime_error("unbounded sender used");return false; },
     true,false,[this]{return now;},MAV_TYPE_GROUND_ROVER,MAV_AUTOPILOT_PX4,true)
  {
    require(host.load(provider,"px4_rover_control"),"could not load real Rover provider");
    require(host.load(missions,"missions"),"could not load real missions provider");
    host.configure("vehicle_profile",px4::rover_profile);
    host.configure("control_native_actions","true");
    host.configure("control_mission_execution","true");
    host.configure("shadow_mode","false");
    host.configureModules({});
    host.setFirmwareRequestsEnabled(false);
    host.configureAutoRecovery(slot.path.string(),session);
    host.setBoundedSend([this](const uint8_t* bytes,std::size_t size,double expires,const Runtime::SendEligibility& gate) {
      const auto packet=decode(bytes,size);
      require(packet.sysid==245 && packet.compid==191,"wrong companion source");
      if(before_handoff)before_handoff(packet,expires);
      const bool accepted=gate(now);
      frames.push_back({packet,now,expires,accepted});
      return accepted;
    });
    host.finishStartup();
  }
  void heartbeat()
  {
    mavlink_message_t m{};
    mavlink_msg_heartbeat_pack(1,1,&m,MAV_TYPE_GROUND_ROVER,MAV_AUTOPILOT_PX4,
      MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|(armed?MAV_MODE_FLAG_SAFETY_ARMED:0),mode,MAV_STATE_ACTIVE);
    host.message(m,now);
  }
  void currentMode()
  {
    if(!current_system)return;
    mavlink_message_t m{};
    mavlink_msg_current_mode_pack(current_system,current_component,&m,0,mode,intended?intended:mode);
    host.message(m,now);
  }
  void samples()
  {
    const auto boot=static_cast<uint32_t>(std::lround(now*1e3));
    const auto usec=static_cast<uint64_t>(std::llround(now*1e6));
    mavlink_message_t m{};
    heartbeat();currentMode();
    mavlink_msg_global_position_int_pack(1,1,&m,boot,latitude,longitude,489400,0,0,0,0,0);
    host.message(m,now);
    mavlink_msg_local_position_ned_pack(1,1,&m,boot,0,0,0,0,0,0);
    host.message(m,now);
    const float q[]{1,0,0,0};
    mavlink_msg_home_position_pack(1,1,&m,latitude,longitude,489400,0,0,0,q,0,0,0,usec);
    host.message(m,now);
    mavlink_msg_estimator_status_pack(1,1,&m,usec,px4::healthy_flags,0,0,0,0,0,0,.2,.3);
    host.message(m,now);
    mavlink_msg_gps_raw_int_pack(1,1,&m,usec,GPS_FIX_TYPE_3D_FIX,latitude,longitude,489400,
      90,90,gps_speed,0,10,489400,900,1500,400,0,0);
    host.message(m,now);
    if(loaded_plan) {
      mavlink_msg_mission_current_pack(1,1,&m,0,2,plan_state,plan_mode,mission_id,0,0);
      host.message(m,now);
    }
  }
  void setting(const std::string& name,int32_t value)
  {
    mavlink_message_t m{};
    float wire=.6f;
    if(name!="COM_OF_LOSS_T")std::memcpy(&wire,&value,sizeof(value));
    mavlink_msg_param_value_pack(1,1,&m,name.c_str(),wire,
      name=="COM_OF_LOSS_T"?MAV_PARAM_TYPE_REAL32:MAV_PARAM_TYPE_INT32,4,0);
    host.message(m,now);
  }
  static MissionItem waypoint(unsigned seq)
  {
    MissionItem result;
    result.seq=seq;result.current=seq==0;result.frame=MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    result.x=latitude+1000*(1+seq);result.y=longitude;result.z=0;
    return result;
  }
  void answer()
  {
    while(answered<frames.size()) {
      const auto frame=frames[answered++];
      if(!frame.accepted)continue;
      const auto& out=frame.message;
      mavlink_message_t m{};
      switch(out.msgid) {
      case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
        mavlink_param_request_read_t p{};mavlink_msg_param_request_read_decode(&out,&p);
        const std::string name(p.param_id,std::find(p.param_id,p.param_id+16,'\0'));
        const int32_t value=name=="SYS_AUTOSTART"?10045:name=="COM_OBL_RC_ACT"?5:1;
        require(p.target_system==1 && p.target_component==1 && p.param_index==-1 &&
          (name=="SYS_AUTOSTART" || name=="COM_OBL_RC_ACT" || name=="MAV_FWDEXTSP" || name=="COM_OF_LOSS_T"),
          "unexpected Rover configuration request");
        now+=.001;setting(name,name==bad_setting?0:value);continue;
      }
      case MAVLINK_MSG_ID_MISSION_COUNT:
      case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
        unsigned seq=0;
        if(out.msgid==MAVLINK_MSG_ID_MISSION_ITEM_INT) {
          mavlink_mission_item_int_t item{};mavlink_msg_mission_item_int_decode(&out,&item);seq=item.seq+1;
        }
        if(seq<2)mavlink_msg_mission_request_int_pack(1,1,&m,245,191,seq,MAV_MISSION_TYPE_MISSION);
        else {
          loaded_plan=true;
          mavlink_msg_mission_ack_pack(1,1,&m,245,191,MAV_MISSION_ACCEPTED,MAV_MISSION_TYPE_MISSION,mission_id);
        }
        break;
      }
      case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        mavlink_msg_mission_count_pack(1,1,&m,245,191,2,MAV_MISSION_TYPE_MISSION,mission_id);
        break;
      case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
        mavlink_mission_request_int_t request{};mavlink_msg_mission_request_int_decode(&out,&request);
        const auto p=waypoint(request.seq);
        mavlink_msg_mission_item_int_pack(1,1,&m,245,191,p.seq,p.frame,p.command,p.current,p.autocontinue,
          0,0,0,0,p.x,p.y,p.z,MAV_MISSION_TYPE_MISSION);
        break;
      }
      default: continue;
      }
      now+=.001;host.message(m,now);
    }
  }
  void mail(const std::string& key,const std::string& text,const char* source="operator")
  { host.mail({key,text,source,0,now,now,false}); }
  void helm()
  {
    mail("IVPHELM_STATE","DRIVE","pHelmIvP");
    host.mail({"DESIRED_HEADING","","pHelmIvP",0,now,now,true});
    host.mail({"DESIRED_SPEED","","pHelmIvP",1,now,now,true});
  }
  void step(unsigned times=1,bool input=false)
  {
    while(times--) {now+=.08;samples();if(input)helm();host.tick(now);answer();}
  }
  std::map<std::string,std::string> last(const std::string& key) const
  {for(auto it=publications.rbegin();it!=publications.rend();++it)if(it->key==key)return fields(it->text);return {};}
  unsigned movements() const
  {return std::count_if(frames.begin(),frames.end(),[](const auto& f){return f.accepted&&motion(f.message);});}
  bool sent(unsigned id) const
  {return std::any_of(frames.begin(),frames.end(),[id](const auto& f){return f.accepted&&command(f.message)==id;});}
  void acquire()
  {mail("IVPHELM_ALLSTOP","clear","pHelmIvP");mail("MAVMOOS_NAV_CONTROL","acquire");}
  void startOffboard()
  {
    step(28);require(last(rover_status)["hold_ready"]=="true","fixture never reached stopped qualified Hold");
    acquire();
    for(unsigned i=0;i<40 && !sent(MAV_CMD_DO_SET_MODE);++i)step(1,true);
    require(sent(MAV_CMD_DO_SET_MODE),"fixture did not send Offboard: "+host.status());
    mode=px4::offboard;current_system=0;step(2,true);
    require(host.navigationGranted() && !movements(),"heartbeat alone either moved or retired the pending Offboard entry");
    current_system=1;step(7,true);
    require(host.navigationGranted() && movements(),"fixture did not send positive Offboard: "+host.status());
  }
  void handback()
  {
    now+=.001;
    mail("MAVMOOS_AUTO_RECOVERY_REQUEST","schema=1,id=return,action=handback,session="+session);
  }
  void startMission()
  {
    armed=false;step(28);
    const auto first=waypoint(0),second=waypoint(1);
    mail("MAVMOOS_MISSION_REQUEST","id=verified,action=upload,items=0:16:6:1:1:0:0:0:0:"+
      std::to_string(first.x)+":"+std::to_string(first.y)+":0|1:16:6:0:1:0:0:0:0:"+
      std::to_string(second.x)+":"+std::to_string(second.y)+":0");
    for(unsigned i=0;i<60 && !host.storedMissionProof().valid;++i)step();
    const auto proof=host.storedMissionProof();
    require(proof.valid && host.verifiedPx4MissionId()==mission_id,"real Rover plan did not verify");
    armed=true;step(25);
    mail("MAVMOOS_MISSION_CONTROL","schema=1,id=run,action=start,verification_id=verified,content_id="+
      proof.content_id+",proof_revision="+std::to_string(proof.revision)+",timeout=40");
    step();
    require(sent(MAV_CMD_MISSION_START),"native start missing at wire: "+last(mission_result)["reason"]);
    now+=.001;
    mavlink_message_t ack{};
    mavlink_msg_command_ack_pack(1,1,&ack,MAV_CMD_MISSION_START,MAV_RESULT_ACCEPTED,0,0,245,191);
    host.message(ack,now);
    mode=px4::mission;plan_state=MISSION_STATE_ACTIVE;plan_mode=1;step(3);
    require(host.missionExecutionGranted("run") && last(mission_result)["status"]=="ACTIVE","native mission fixture not active");
  }
};
}

int main(int argc,char** argv)
{
  if(argc!=3)return 2;
  unsigned failed=0;
  auto test=[&](const char* name,const auto& body) {
    try {body();std::cout<<"PASS "<<name<<'\n';}
    catch(const std::exception& error){++failed;std::cerr<<"FAIL "<<name<<": "<<error.what()<<'\n';}
  };
  test("verified profile and selected Hold intent required", [&] {
    for(const auto* setting:{"SYS_AUTOSTART","COM_OBL_RC_ACT"}) {
      Slot slot;Rig r(argv[1],argv[2],slot);r.bad_setting=setting;r.step(28);r.acquire();r.step(10,true);
      require(r.last(rover_status)["config_verified"]=="false" && !r.host.navigationGranted() &&
        !r.sent(MAV_CMD_DO_SET_MODE) && !r.movements(),"bad readback allowed effects");
    }
    Slot slot;Rig r(argv[1],argv[2],slot);r.current_system=2;r.step(28);r.acquire();r.step(10,true);
    require(r.last(rover_status)["config_verified"]=="true" && r.last(rover_status)["hold_ready"]=="false" &&
      !r.host.navigationGranted() && !r.sent(MAV_CMD_DO_SET_MODE),"foreign CURRENT_MODE permitted acquisition");
    r.current_system=1;r.intended=px4::offboard;r.step(2);r.acquire();r.step(10,true);
    require(r.last(rover_status)["stop_confirmed"]=="true" && !r.host.navigationGranted() &&
      !r.sent(MAV_CMD_DO_SET_MODE),"retained Offboard intent permitted acquisition");
  });
  test("Manual takeover at the final positive handoff", [&] {
    Slot slot;Rig r(argv[1],argv[2],slot);r.startOffboard();const auto before=r.movements();
    bool injected=false;
    r.before_handoff=[&](const auto& packet,double) {
      if(!injected && motion(packet)){injected=true;r.mode=manual;r.heartbeat();r.currentMode();}
    };
    r.step(5,true);
    require(injected && r.movements()==before && !r.host.navigationGranted(),"Manual takeover crossed final motion gate");
  });
  test("selected contradictory intent at the final positive handoff", [&] {
    Slot slot;Rig r(argv[1],argv[2],slot);r.startOffboard();const auto before=r.movements();
    bool injected=false;
    r.before_handoff=[&](const auto& packet,double) {
      if(!injected && motion(packet)){injected=true;r.intended=manual;r.currentMode();}
    };
    r.step(5,true);
    require(injected && r.movements()==before,"contradictory intent crossed final motion gate");
  });
  test("foreign reports cannot renew live selected mode evidence", [&] {
    Slot slot;Rig r(argv[1],argv[2],slot);r.startOffboard();
    r.current_component=2;r.step(22,true);const auto before=r.movements();r.step(4,true);
    require(r.movements()==before && !r.host.navigationGranted(),"motion continued after selected CURRENT_MODE aged out");
  });
  test("unowned Mission marker survives restart and requires a stopped Hold handback", [&] {
    Slot slot;
    {Rig first(argv[1],argv[2],slot);first.mode=px4::mission;first.step(2);
      require(slot.marked() && first.last(recovery_status)["state"]=="UNRESOLVED","unowned Mission did not persist uncertainty");}
    Rig restart(argv[1],argv[2],slot,"process-two");restart.intended=px4::mission;restart.step(28);
    restart.handback();
    require(slot.marked() && restart.last(recovery_status)["state"]=="UNRESOLVED" &&
      !restart.host.navigationGranted(),"retained Mission intent allowed handback");
    restart.intended=0;restart.gps_speed=70;restart.step(5);restart.handback();
    require(slot.marked(),"moving Hold allowed handback");
    restart.gps_speed=5;restart.step(25);restart.handback();
    require(!slot.marked() && restart.last(recovery_status)["state"]=="READY" &&
      !restart.host.navigationGranted() && !restart.sent(MAV_CMD_DO_SET_MODE),"stopped Hold handback granted effects");
  });
  test("stop-only request cannot overrule Manual at final mission handoff", [&] {
    Slot slot;Rig r(argv[1],argv[2],slot);r.startMission();
    bool attempted=false;
    r.before_handoff=[&](const auto& packet,double) {
      if(command(packet)==MAV_CMD_DO_SET_MODE) {
        attempted=true;require(slot.marked(),"stop-only handoff lost the durable marker");
        r.mode=manual;r.heartbeat();r.currentMode();
      }
    };
    r.mail("MAVMOOS_MISSION_CONTROL","schema=1,id=stop,action=abort,target=run,timeout=8");
    r.step(3);
    require(attempted && !r.sent(MAV_CMD_DO_SET_MODE) && slot.marked() &&
      !r.host.missionExecutionGranted("run"),"stop-only Hold overrode Manual or retired uncertainty");
  });
  return failed?1:0;
}
