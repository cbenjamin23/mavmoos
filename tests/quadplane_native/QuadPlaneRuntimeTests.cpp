// Loaded provider and mission DSOs through the production final handoff gate.
#include "../protocol_faults/ProtocolSupport.h"
#include "QuadPlanePolicy.h"
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
#include <iostream>
#include <unistd.h>

using namespace protocol_faults;
namespace {
constexpr char request[]="MAVMOOS_QUADPLANE_REQUEST",result[]="MAVMOOS_QUADPLANE_RESULT";
constexpr unsigned hover=MAV_VTOL_STATE_MC,forward=MAV_VTOL_STATE_FW,transition=MAV_VTOL_STATE_TRANSITION_TO_FW;
constexpr unsigned loiter=12,guided=15;
constexpr int32_t lat=423584560,lon=-710875890;
constexpr char nav[]="MAVMOOS_NAV_CONTROL";

struct Slot {
  std::filesystem::path directory;
  Slot() {
    auto name=(std::filesystem::temp_directory_path()/"mavmoos-quadplane-XXXXXX").string();
    require(mkdtemp(name.data())!=nullptr,"private recovery directory");
    directory=name;
  }
  ~Slot() {std::error_code error;std::filesystem::remove_all(directory,error);}
  bool marked() const {return std::filesystem::exists(directory/"auto-may-continue");}
};

struct Rig {
  double now=100;
  unsigned mode=quadplane::qloiter,phase=hover,cursor=0;
  uint8_t cursor_system=1;
  uint8_t cursor_state=MISSION_STATE_NOT_STARTED;
  unsigned cursor_total=3;
  bool armed=false,airborne=false;
  bool helm_enabled=false,hover_enabled=false;
  float height=0,speed=0;
  int32_t home_lat=lat;
  std::vector<Frame> frames;
  std::vector<Publication> publications;
  std::function<void(const mavlink_message_t&)> before_send;
  Runtime host;

  Rig(const char* control,const char* missions,const Slot& slot,const char* session="owned-one",
      bool enable_helm=false,bool enable_hover=false)
    :helm_enabled(enable_helm),hover_enabled(enable_hover),host([this](const auto& key,const auto& value,double,bool numeric) {
       if(!numeric)publications.push_back({key,value,now});
     },[](const uint8_t*,std::size_t) {
       throw std::runtime_error("unbounded transport used");
       return false;
     },true,false,[this]{return now;},MAV_TYPE_VTOL_FIXEDROTOR,MAV_AUTOPILOT_ARDUPILOTMEGA,true)
  {
    require(host.load(control,"arduquadplane_control")&&host.load(missions,"missions"),"load actual DSOs");
    host.configure("vehicle_profile",quadplane::profile);
    host.configure("control_native_actions","true");
    host.configure("control_mission_execution","true");
    host.configure("control_helm",helm_enabled?"true":"false");
    if(hover_enabled)host.configureModules({{"arduquadplane_control",{{"hover_adapter","true"}}}});
    else host.configureModules({});
    host.setFirmwareRequestsEnabled(false);
    host.configureAutoRecovery(slot.directory.string(),session);
    host.setBoundedSend([this](const uint8_t* bytes,std::size_t size,double expires,const Runtime::SendEligibility& gate) {
      const auto packet=decode(bytes,size);
      if(before_send)before_send(packet);
      const auto accepted=gate(now);
      frames.push_back({packet,now,expires,accepted});
      return accepted;
    });
    host.finishStartup();
    require(host.hasCapability("control.navigation.helm")==helm_enabled,"QuadPlane helm facet was not opt-in");
    require(host.hasCapability(quadplane::hover_capability)==hover_enabled,"QuadPlane hover was not opt-in");
  }

  void samples(bool phase_evidence=true)
  {
    mavlink_message_t m{};
    mavlink_msg_heartbeat_pack(1,1,&m,MAV_TYPE_VTOL_FIXEDROTOR,MAV_AUTOPILOT_ARDUPILOTMEGA,
      MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|(armed?MAV_MODE_FLAG_SAFETY_ARMED:0),mode,MAV_STATE_ACTIVE);
    host.message(m,now);
    if(!phase_evidence)return;
    mavlink_msg_global_position_int_pack(1,1,&m,static_cast<uint32_t>(now*1000),lat,lon,
      static_cast<int32_t>(10000+height*1000),static_cast<int32_t>(height*1000),static_cast<int16_t>(speed*100),0,0,0);
    host.message(m,now);
    const float orientation[]{1,0,0,0};
    mavlink_msg_home_position_pack(1,1,&m,home_lat,lon,10000,0,0,0,orientation,0,0,0,static_cast<uint64_t>(now*1e6));
    host.message(m,now);
    mavlink_msg_extended_sys_state_pack(1,1,&m,phase,airborne?MAV_LANDED_STATE_IN_AIR:MAV_LANDED_STATE_ON_GROUND);
    host.message(m,now);
    if(helm_enabled) {
      mavlink_msg_ekf_status_report_pack(1,1,&m,55,.1,.1,.1,.1,0,0);
      host.message(m,now);
      mavlink_msg_airspeed_pack(1,1,&m,0,speed,1500,10.f,AIRSPEED_SENSOR_USING);
      host.message(m,now);
    }
    mavlink_msg_mission_current_pack(cursor_system,1,&m,cursor,cursor_total,cursor_state,0,0,0,0);
    if(cursor_state==MISSION_STATE_UNKNOWN)
      require(m.len==3,"fixture count must arrive without the state extension");
    host.message(m,now);
  }
  void tick()
  {
    now+=.02;samples();host.tick(now);
  }
  void post(const std::string& key,const std::string& value,const std::string& source="fixture")
  {
    now+=.01;
    Mail mail{};mail.key=key;mail.text=value;mail.source=source;mail.received_at=mail.published_at=now;
    host.mail(mail);
  }
  void helm(double course=90)
  {
    post("IVPHELM_STATE","DRIVE","pHelmIvP");
    post("IVPHELM_ALLSTOP","clear","pHelmIvP");
    host.mail({"DESIRED_HEADING","","pHelmIvP",course,now,now,true});
    host.mail({"DESIRED_SPEED","","pHelmIvP",hover_enabled?2.:22.,now,now,true});
    host.mail({"DESIRED_ALTITUDE","","pHelmIvP",65,now,now,true});
  }
  void flying(unsigned selected=loiter)
  {
    armed=airborne=true;phase=forward;mode=selected;height=60;speed=22;
    tick();
  }
  bool status(const std::string& expected) const
  { return host.status().find(expected)!=std::string::npos; }
  std::size_t framesOf(unsigned id) const
  {
    return std::count_if(frames.begin(),frames.end(),[id](const auto& frame) {
      return frame.accepted&&frame.message.msgid==id;
    });
  }
  void version(bool valid=true)
  {
    mavlink_autopilot_version_t sample{};
    sample.flight_sw_version=valid?0x040603ffu:0x040604ffu;
    std::copy_n("3fc7011a",8,sample.flight_custom_version);
    mavlink_message_t message{};
    mavlink_msg_autopilot_version_encode(1,1,&message,&sample);
    now+=.001;host.message(message,now);
  }
  void parameter(const std::string& name,uint8_t type,float value,uint8_t system=1)
  {
    char key[16]{};
    std::copy_n(name.data(),name.size(),key);
    mavlink_message_t message{};
    mavlink_msg_param_value_pack(system,1,&message,key,value,type,100,UINT16_MAX);
    now+=.001;host.message(message,now);
  }
  void qualify(bool hover_guidance=false,bool wrong_version=false)
  {
    const std::map<std::string,std::pair<uint8_t,float>> proof={
      {"Q_ENABLE",{2,1}},{"Q_MAV_TYPE",{2,22}},{"Q_GUIDED_MODE",{2,(hover_guidance||hover_enabled)?1.f:0.f}},
      {"Q_FRAME_CLASS",{2,1}},{"Q_FRAME_TYPE",{2,1}},{"Q_TILT_ENABLE",{2,0}},
      {"Q_TAILSIT_ENABLE",{2,0}},{"Q_RTL_MODE",{2,hover_enabled?3.f:0.f}},{"Q_ASSIST_SPEED",{9,18}},
      {"SYSID_MYGCS",{4,245}},{"FS_GCS_ENABL",{2,1}},{"FS_LONG_ACTN",{2,1}},
      {"FS_LONG_TIMEOUT",{9,5}},{"RTL_AUTOLAND",{2,0}},{"RTL_ALTITUDE",{9,65}},
      {"RTL_RADIUS",{4,0}},{"WP_LOITER_RAD",{4,80}},{"AIRSPEED_CRUISE",{9,22}},
      {"AIRSPEED_MIN",{4,13}},{"AIRSPEED_MAX",{4,35}},{"ARSPD_USE",{2,1}},
      {"ARSPD_TYPE",{2,100}},{"ARSPD_PRIMARY",{2,0}},{"ARSPD2_TYPE",{2,0}},
      {"SCR_ENABLE",{2,1}},{"FENCE_ENABLE",{2,0}},{"FENCE_AUTOENABLE",{2,0}}};
    std::size_t consumed=0;
    for(unsigned i=0;i<150&&!status("plane_profile=VERIFIED")&&!status("plane_profile=MISMATCH");++i) {
      tick();
      while(consumed<frames.size()) {
        const auto outbound=frames[consumed++];
        if(!outbound.accepted)continue;
        if(command(outbound.message)==MAV_CMD_REQUEST_MESSAGE) {
          version(!wrong_version);continue;
        }
        if(outbound.message.msgid!=MAVLINK_MSG_ID_PARAM_REQUEST_READ)continue;
        mavlink_param_request_read_t read{};
        mavlink_msg_param_request_read_decode(&outbound.message,&read);
        const std::string name(read.param_id,std::find(read.param_id,read.param_id+16,'\0'));
        require(read.param_index==-1&&proof.count(name),"unexpected QuadPlane pin request "+name);
        const auto [type,value]=proof.at(name);
        parameter(name,type,value);
      }
    }
    require(status((hover_guidance||wrong_version)?"plane_profile=MISMATCH":"plane_profile=VERIFIED"),
      "QuadPlane proof did not reach expected state "+host.status());
    // The watchdog heartbeat is sent on its own bounded cadence.
    for(unsigned i=0;i<55;++i)tick();
  }
  mavlink_command_int_t point() const
  {
    const auto last=std::find_if(frames.rbegin(),frames.rend(),[](const auto& frame) {
      return frame.accepted&&frame.message.msgid==MAVLINK_MSG_ID_COMMAND_INT;
    });
    require(last!=frames.rend(),"no accepted QuadPlane point");
    mavlink_command_int_t target{};
    mavlink_msg_command_int_decode(&last->message,&target);
    return target;
  }
  void readback(const mavlink_command_int_t& target,uint8_t source=1)
  {
    mavlink_message_t message{};
    mavlink_msg_position_target_global_int_pack(source,1,&message,1,MAV_FRAME_GLOBAL,0xFDF8,
      target.x,target.y,10+target.z,0,0,0,0,0,0,0,0);
    now+=.01;host.message(message,now);
  }
  void offer(uint32_t token,uint8_t kind,uint8_t system=1)
  {
    mavlink_tunnel_t sample{};
    sample.target_system=245;sample.target_component=191;
    sample.payload_length=quadplane::hover_offer_bytes;
    quadplane::hoverBytes(sample.payload,quadplane::hover_marker,4);
    quadplane::hoverBytes(sample.payload+4,token,4);
    sample.payload[8]=kind;sample.payload[9]=1;
    mavlink_message_t message{};
    mavlink_msg_tunnel_encode(system,1,&message,&sample);
    now+=.01;host.message(message,now);
  }
  void act(const std::string& body)
  {
    post(request,"schema=1,"+body);tick();
  }
  bool seen(const char* key,const std::string& id,const std::string& status,const std::string& reason="") const
  {
    return std::any_of(publications.begin(),publications.end(),[&](const auto& row) {
      if(row.key!=key)return false;
      const auto f=fields(row.text);
      return f.at("id")==id&&f.at("status")==status&&(reason.empty()||f.at("reason")==reason);
    });
  }
  std::size_t sent(unsigned command_id,float param1=0,float param2=0) const
  {
    return std::count_if(frames.begin(),frames.end(),[&](const auto& frame) {
      if(!frame.accepted||command(frame.message)!=command_id)return false;
      mavlink_command_long_t c{};mavlink_msg_command_long_decode(&frame.message,&c);
      require(frame.message.sysid==245&&frame.message.compid==191&&c.target_system==1&&c.target_component==1&&
        c.confirmation==0&&c.param3==0&&c.param4==0&&c.param5==0&&c.param6==0&&c.param7==0,
        "incorrect command identity or payload");
      return c.param1==param1&&c.param2==param2;
    });
  }

  std::string planRows(int32_t forward_lat=lat+34000) const
  {
    return "0:16:0:1:1:0:0:0:0:423584560:-710875890:10|"
      "1:84:3:0:1:0:0:0:0:423584560:-710875890:30|"
      "2:16:3:0:1:0:0:0:0:"+std::to_string(forward_lat)+":-710875890:35|"
      "3:85:3:0:1:0:0:0:0:423584560:-710875890:0";
  }
  StoredMissionProof upload()
  {
    std::vector<mavlink_mission_item_int_t> items;
    std::size_t next=frames.size();
    post("MAVMOOS_MISSION_REQUEST","id=plan,action=upload,items="+planRows());
    for(unsigned step=0;step<60&&!host.storedMissionProof().valid;++step) {
      tick();
      while(next<frames.size()) {
        const auto out=frames[next++];
        if(!out.accepted)continue;
        mavlink_message_t reply{};
        switch(out.message.msgid) {
        case MAVLINK_MSG_ID_MISSION_COUNT:
          mavlink_msg_mission_request_int_pack(1,1,&reply,245,191,0,MAV_MISSION_TYPE_MISSION);break;
        case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
          mavlink_mission_item_int_t item{};
          mavlink_msg_mission_item_int_decode(&out.message,&item);
          require(item.seq==items.size()&&items.size()<4,"invalid typed upload sequence");
          item.target_system=245;item.target_component=191;items.push_back(item);
          if(items.size()==4)mavlink_msg_mission_ack_pack(1,1,&reply,245,191,MAV_MISSION_ACCEPTED,MAV_MISSION_TYPE_MISSION,0);
          else mavlink_msg_mission_request_int_pack(1,1,&reply,245,191,item.seq+1,MAV_MISSION_TYPE_MISSION);
          break;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
          require(items.size()==4,"incorrect typed upload size");
          mavlink_msg_mission_count_pack(1,1,&reply,245,191,4,MAV_MISSION_TYPE_MISSION,0);break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
          mavlink_mission_request_int_t q{};
          mavlink_msg_mission_request_int_decode(&out.message,&q);
          require(q.seq<items.size(),"invalid typed readback sequence");
          mavlink_msg_mission_item_int_encode(1,1,&reply,&items[q.seq]);break;
        }
        default:continue;
        }
        now+=.001;host.message(reply,now);
      }
    }
    const auto proof=host.storedMissionProof();
    require(proof.valid&&proof.items.size()==4,"exact typed readback did not produce proof");
    return proof;
  }
  void select(const Slot& slot)
  {
    tick();
    const auto proof=upload();
    act("id=arm,action=arm");
    require(sent(MAV_CMD_COMPONENT_ARM_DISARM,1)==1&&!seen(result,"arm","COMPLETED"),"arming requires fresh effect");
    armed=true;tick();
    require(seen(result,"arm","COMPLETED"),"new observed arming not confirmed");
    cursor=1;tick();
    const std::string binding=",verification_id="+proof.verification_id+",revision="+
      std::to_string(proof.revision)+",content_id="+proof.content_id;
    act("id=wrong,action=run"+binding+"-wrong");
    require(seen(result,"wrong","REJECTED")&&!slot.marked(),"mismatched proof gained ownership");
    act("id=run,action=run"+binding);
    require(sent(MAV_CMD_DO_SET_MISSION_CURRENT,1,1)==1&&slot.marked(),"cursor not selected under durable lease");
  }
  void launch(const Slot& slot)
  {
    select(slot);
    cursor_system=9;tick();
    require(sent(MAV_CMD_MISSION_START)==0,"foreign cursor confirmed selection");
    cursor_system=1;tick();tick();
    require(sent(MAV_CMD_MISSION_START)==1&&!seen(result,"run","CONFIRMED"),"missing one start or acceptance mistaken for effect");
    mode=quadplane::auto_mode;airborne=true;height=10;tick();tick();
    require(seen(result,"run","CONFIRMED")&&seen(result,"run","ACTIVE","observed_airborne_hover"),
      "native AUTO and measured hover were not observed");
    act("id=early,action=release,target=run");
    require(seen(result,"early","REJECTED")&&slot.marked(),"airborne release cleared durable lease");
  }
};

void complete(const char* control,const char* missions)
{
  Slot slot;
  Rig r(control,missions,slot);
  r.tick();
  const auto before=r.frames.size();
  r.post("MAVMOOS_MISSION_REQUEST","id=bad,action=upload,items="+r.planRows(lat+100));
  r.tick();
  require(r.frames.size()==before&&r.seen("MAVMOOS_MISSION_RESULT","bad","REJECTED"),
    "out-of-envelope plan emitted a mutation");
  r.launch(slot);
  r.phase=transition;r.speed=15;r.cursor=2;r.tick();
  require(!r.seen(result,"run","ACTIVE","observed_forward_flight"),"transition was mistaken for forward flight");
  r.phase=forward;r.tick();
  require(r.seen(result,"run","ACTIVE","observed_forward_flight"),"forward effect was not observed");
  r.phase=hover;r.cursor=3;r.speed=0;r.tick();
  require(r.seen(result,"run","ACTIVE","observed_hover_on_landing_approach"),"landing phase was not observed");
  r.airborne=false;r.height=0;r.tick();r.tick();
  require(r.sent(MAV_CMD_COMPONENT_ARM_DISARM)==1&&r.sent(MAV_CMD_DO_SET_MODE,1,quadplane::qloiter)==0,
    "grounded handback lacked one ordinary disarm");
  r.armed=false;r.home_lat+=5;r.tick();r.tick();
  require(r.sent(MAV_CMD_DO_SET_MODE,1,quadplane::qloiter)==1&&slot.marked(),"native ground handback not sent under lease");
  r.mode=quadplane::qloiter;r.cursor=0;r.tick();
  require(r.seen(result,"run","COMPLETED")&&slot.marked(),"end-state was not observed or lease released implicitly");
  r.act("id=release,action=release,target=run");
  require(r.seen(result,"release","COMPLETED")&&!slot.marked(),"explicit fresh grounded release failed");
}

void cancelled_and_restarted(const char* control,const char* missions)
{
  Slot slot;
  {
    Rig r(control,missions,slot);
    r.launch(slot);
    r.before_send=[&](const auto& packet) {
      if(command(packet)!=MAV_CMD_DO_SET_MODE)return;
      r.now+=.001;r.mode=5;r.samples();
    };
    r.act("id=stop,action=abort,target=run");
    require(r.sent(MAV_CMD_DO_SET_MODE,1,quadplane::qrtl)==0&&
      r.seen(result,"stop","FAILED")&&slot.marked(),"pilot takeover at handoff transmitted or removed marker");
    r.before_send={};r.now+=11;r.mode=quadplane::auto_mode;r.tick();
    r.act("id=stop-again,action=abort,target=run");
    require(r.sent(MAV_CMD_DO_SET_MODE,1,quadplane::qrtl)==0&&r.seen(result,"stop-again","REJECTED")&&
      slot.marked(),"pilot re-entry into AUTO revived the old stop authority");
  }
  Rig r(control,missions,slot,"owned-two");
  r.mode=quadplane::auto_mode;r.armed=r.airborne=true;r.height=20;r.tick();
  std::set<unsigned> observation_requests;
  for(unsigned i=0;i<45;++i) {r.now+=.1;r.samples(false);r.host.tick(r.now);}
  for(const auto& frame:r.frames)if(frame.accepted&&frame.message.msgid==MAVLINK_MSG_ID_COMMAND_LONG) {
    mavlink_command_long_t sent{};mavlink_msg_command_long_decode(&frame.message,&sent);
    require(sent.command==MAV_CMD_SET_MESSAGE_INTERVAL,"restart fence allowed a non-observation command");
    observation_requests.insert(static_cast<unsigned>(sent.param1));
  }
  require(observation_requests==std::set<unsigned>{MAVLINK_MSG_ID_GLOBAL_POSITION_INT,MAVLINK_MSG_ID_HOME_POSITION,
    MAVLINK_MSG_ID_EXTENDED_SYS_STATE},"restart fence did not request exactly the needed handback evidence");
  const std::string release="schema=1,id=handback,action=handback,session=owned-two";
  r.post("MAVMOOS_AUTO_RECOVERY_REQUEST",release);r.tick();
  require(slot.marked()&&!r.host.storedMissionProof().valid,"restart cleared live native ownership");
  r.mode=quadplane::qloiter;r.tick();
  r.post("MAVMOOS_AUTO_RECOVERY_REQUEST",release);r.tick();
  require(slot.marked(),"airborne QLoiter cleared restart marker");
  r.armed=r.airborne=false;r.height=0;r.tick();
  r.post("MAVMOOS_AUTO_RECOVERY_REQUEST",release);r.tick();
  require(!slot.marked()&&r.seen("MAVMOOS_AUTO_RECOVERY_RESULT","handback","RELEASED"),
    "fresh disarmed ground recovery did not retire marker");
}

void changed_plan_at_handoff(const char* control,const char* missions,uint8_t cursor_state)
{
  Slot slot;
  Rig r(control,missions,slot);
  r.select(slot);
  bool intercepted=false;
  r.before_send=[&](const auto& packet) {
    if(command(packet)!=MAV_CMD_MISSION_START)return;
    intercepted=true;
    r.now+=.001;r.cursor_total=4;r.cursor_state=cursor_state;r.samples();
  };
  r.tick();r.tick();
  require(intercepted&&r.sent(MAV_CMD_MISSION_START)==0&&!r.host.storedMissionProof().valid&&slot.marked(),
    "fresh contradictory mission count allowed start or retired the uncertainty marker");
}

void acquire_forward(Rig& r,std::size_t total)
{
  const auto previous_points=r.framesOf(MAVLINK_MSG_ID_COMMAND_INT);
  r.post(nav,"acquire");r.tick();
  require(r.sent(MAV_CMD_DO_SET_MODE,1,guided)==total&&r.status("plane_guided_confirmed=0"),
    "acquisition did not emit one unconfirmed Guided request");
  r.helm();r.tick();
  r.mode=guided;r.tick();
  require(r.status("plane_guided_confirmed=1")&&r.framesOf(MAVLINK_MSG_ID_COMMAND_INT)==previous_points,
    "pre-Guided helm values crossed the confirmed mode boundary");
}

void forward_helm(const char* control,const char* missions)
{
  Slot slot;
  Rig r(control,missions,slot,"helm-one",true);
  r.flying();
  r.post(nav,"acquire");r.tick();
  require(!r.host.navigationGranted()&&r.sent(MAV_CMD_DO_SET_MODE,1,guided)==0,
    "unverified forward profile acquired helm");
  r.qualify();
  require(r.status("plane_watchdog_started=1"),"qualified forward Loiter did not start watchdog");
  for(const auto phase:{hover,transition}) {
    r.phase=phase;r.tick();
    r.post(nav,"acquire");r.tick();
    require(!r.host.navigationGranted()&&r.sent(MAV_CMD_DO_SET_MODE,1,guided)==0,
      "hover or transition acquired helm");
  }
  r.phase=forward;r.tick();
  CommandRequest bypass{};
  bypass.id="unowned-mode";bypass.command=MAV_CMD_DO_SET_MODE;bypass.params[0]=1;
  bypass.params[1]=guided;bypass.expires=r.now+.3;
  require(!r.host.sendCommand(bypass),"unowned direct Guided command bypassed helm acquisition");
  acquire_forward(r,1);
  r.helm();r.tick();
  const auto target=r.point();
  require(target.target_system==1&&target.target_component==1&&target.command==MAV_CMD_DO_REPOSITION&&
    target.frame==MAV_FRAME_GLOBAL_RELATIVE_ALT_INT&&target.param1==-1&&target.param2==0&&
    target.param3==0&&target.param4==0&&target.y>lon&&target.z==65,
    "forward target differs from addressed course lookahead");
  r.readback(target,9);
  require(r.status("plane_target=AWAIT_READBACK"),"foreign readback confirmed a target");
  r.readback(target);
  require(r.status("plane_target=VERIFIED"),"selected fresh point was not confirmed");
  r.post("IVPHELM_ALLSTOP","NothingToDo","pHelmIvP");r.tick();
  require(!r.host.navigationGranted()&&r.sent(MAV_CMD_DO_SET_MODE,1,loiter)==0,
    "normal pause emitted a competing fallback or retained the grant");
  r.helm();r.tick();
  require(r.host.navigationGranted()&&r.sent(MAV_CMD_DO_SET_MODE,1,guided)==1,
    "bounded pause did not resume in the same phase");
  r.readback(target);
  r.post("IVPHELM_ALLSTOP","NothingToDo","pHelmIvP");r.tick();
  for(unsigned i=0;i<30;++i)r.tick();
  require(!r.host.navigationGranted()&&r.sent(MAV_CMD_DO_SET_MODE,1,loiter)==1,
    "expired handoff did not request exactly one safe native Loiter");
  r.mode=loiter;r.tick();
  require(r.status("plane_handback=CONFIRMED"),"Loiter handback lacked fresh selected state");
  r.helm();r.tick();
  require(!r.host.navigationGranted(),"ordinary helm mail reacquired an expired grant");
  acquire_forward(r,2);
  r.mode=5;r.tick();
  const auto loiters=r.sent(MAV_CMD_DO_SET_MODE,1,loiter);
  r.mode=loiter;r.helm();r.tick();
  require(!r.host.navigationGranted()&&r.sent(MAV_CMD_DO_SET_MODE,1,loiter)==loiters,
    "old permission survived a pilot mode excursion");
}

void forward_fences(const char* control,const char* missions)
{
  for(const bool bad_version:{false,true}) {
    Slot slot;
    Rig r(control,missions,slot,"profile-mismatch",true);
    r.flying();r.qualify(!bad_version,bad_version);
    r.post(nav,"acquire");r.tick();
    require(!r.host.navigationGranted()&&r.sent(MAV_CMD_DO_SET_MODE,1,guided)==0&&
      r.framesOf(MAVLINK_MSG_ID_HEARTBEAT)==0,"mismatched profile acquired or sustained the watchdog");
  }
  Slot slot;
  Rig r(control,missions,slot,"phase-at-send",true);
  r.flying();r.qualify();
  bool acquisition_intercepted=false;
  r.before_send=[&](const auto& message) {
    if(command(message)!=MAV_CMD_DO_SET_MODE)return;
    acquisition_intercepted=true;r.now+=.001;r.phase=transition;r.samples();
    r.now+=.001;r.phase=forward;r.samples();
  };
  r.post(nav,"acquire");r.tick();
  require(acquisition_intercepted&&!r.host.navigationGranted()&&r.sent(MAV_CMD_DO_SET_MODE,1,guided)==0,
    "a phase excursion permitted a queued Guided request");
  r.before_send={};
  acquire_forward(r,1);
  bool intercepted=false;
  r.before_send=[&](const auto& message) {
    if(message.msgid!=MAVLINK_MSG_ID_COMMAND_INT)return;
    intercepted=true;r.now+=.001;
    // MAVLink 2 trims the missing landed-state byte. A phase-only observation
    // cannot keep a grant or make a Loiter handback eligible.
    mavlink_message_t invalid{};
    mavlink_msg_extended_sys_state_pack(1,1,&invalid,forward,MAV_LANDED_STATE_UNDEFINED);
    require(invalid.len==1,"fixture must omit the landed-state byte");
    r.host.message(invalid,r.now);
    r.now+=.001;r.phase=forward;r.samples();
  };
  r.helm();r.tick();
  require(intercepted&&!r.host.navigationGranted()&&r.framesOf(MAVLINK_MSG_ID_COMMAND_INT)==0&&
    r.sent(MAV_CMD_DO_SET_MODE,1,loiter)==0,"a phase excursion permitted an encoded old target or Loiter");
  r.before_send={};
  const auto heartbeats=r.framesOf(MAVLINK_MSG_ID_HEARTBEAT);
  for(unsigned i=0;i<60;++i)r.tick();
  require(!r.host.navigationGranted()&&r.framesOf(MAVLINK_MSG_ID_HEARTBEAT)==heartbeats,
    "unowned Guided phase recovery renewed watchdog");
}

void helm_and_retained_auto(const char* control,const char* missions)
{
  Slot slot;
  {
    Rig r(control,missions,slot,"mission-owner",true);
    r.qualify();r.launch(slot);
    r.flying();
    r.post(nav,"acquire");r.helm();r.tick();
    require(!r.host.navigationGranted()&&slot.marked()&&r.sent(MAV_CMD_DO_SET_MODE,1,guided)==0,
      "native AUTO retention did not fence a fresh forward Loiter acquire");
  }
  Rig restarted(control,missions,slot,"restarted",true);
  restarted.flying();
  for(unsigned i=0;i<60;++i)restarted.tick();
  require(restarted.framesOf(MAVLINK_MSG_ID_PARAM_REQUEST_READ)==0&&
    restarted.sent(MAV_CMD_REQUEST_MESSAGE,MAVLINK_MSG_ID_AUTOPILOT_VERSION)==0&&
    restarted.framesOf(MAVLINK_MSG_ID_HEARTBEAT)==0,"restart fence opened forward-profile proof or watchdog");
}

void scripted_hover(const char* control,const char* missions)
{
  Slot slot;
  Rig r(control,missions,slot,"hover",true,true);
  r.tick();
  r.armed=r.airborne=true;
  r.height=60;
  r.qualify();
  r.offer(1,quadplane::hover_hold_offer,9);
  r.post(nav,"acquire");r.tick();
  require(!r.host.navigationGranted()&&r.framesOf(MAVLINK_MSG_ID_HEARTBEAT)==0,
    "missing selected adapter evidence allowed hover acquisition");
  r.offer(2,quadplane::hover_hold_offer);
  r.tick();
  r.post(nav,"acquire");r.tick();
  require(r.sent(MAV_CMD_DO_SET_MODE,1,guided)==1&&!r.status("plane_guided_confirmed=1"),
    "fresh native hover did not request exactly one Guided acquisition");
  r.mode=guided;
  uint32_t transition_token=2;
  for(const auto phase:{MAV_VTOL_STATE_TRANSITION_TO_FW,MAV_VTOL_STATE_TRANSITION_TO_MC}) {
    r.phase=phase;r.tick();
    r.offer(++transition_token,quadplane::hover_guided_offer);
    r.helm();r.tick();
    require(r.host.navigationGranted()&&!r.status("plane_guided_confirmed=1")&&
      r.framesOf(MAVLINK_MSG_ID_TUNNEL)==0,"hover acquisition changed in a transitional phase "+r.host.status());
  }
  r.phase=hover;r.tick();
  r.offer(transition_token+1,quadplane::hover_guided_offer);
  r.helm();
  for(unsigned i=0;i<10&&!r.framesOf(MAVLINK_MSG_ID_TUNNEL);++i)r.tick();
  require(r.framesOf(MAVLINK_MSG_ID_TUNNEL)==1&&r.status("plane_target=AWAIT_READBACK"),
    "fresh Guided hover did not emit exactly one target awaiting readback");
  const auto sent=std::find_if(r.frames.rbegin(),r.frames.rend(),[](const auto& frame) {
    return frame.accepted&&frame.message.msgid==MAVLINK_MSG_ID_TUNNEL;
  });
  mavlink_tunnel_t packet{};
  mavlink_msg_tunnel_decode(&sent->message,&packet);
  const auto decode=[&packet](unsigned index) {
    return static_cast<int32_t>(quadplane::hoverUint(packet.payload+index,4));
  };
  require(sent->message.sysid==245&&sent->message.compid==191&&packet.target_system==1&&
    packet.target_component==1&&packet.payload_type==0&&packet.payload_length==quadplane::hover_target_bytes&&
    packet.payload[8]==quadplane::hover_target&&quadplane::hoverUint(packet.payload+9,2)<=750&&
    decode(11)==lat&&decode(15)>lon&&decode(19)==7125,"hover intent wire envelope differs");
  mavlink_command_int_t applied{};
  applied.x=decode(11);applied.y=decode(15);applied.z=61.25;
  r.readback(applied,9);
  require(r.status("plane_target=AWAIT_READBACK"),"foreign hover readback counted as FC effect");
  r.readback(applied);
  require(r.status("plane_target=VERIFIED"),"independent FC target did not confirm hover");
  const auto used=quadplane::hoverUint(packet.payload+4,4);
  r.offer(used,quadplane::hover_guided_offer);
  for(unsigned i=0;i<20;++i) {r.helm();r.tick();}
  require(!r.host.navigationGranted()&&r.framesOf(MAVLINK_MSG_ID_TUNNEL)==1&&
    r.sent(MAV_CMD_DO_SET_MODE,1,quadplane::qloiter)==1,
    "duplicate adapter evidence changed authority or repeated native hold: "+r.host.status()+
    ", mode handoffs="+std::to_string(r.sent(MAV_CMD_DO_SET_MODE,1,quadplane::qloiter)));
  r.mode=quadplane::qloiter;r.tick();
  r.offer(transition_token+2,quadplane::hover_hold_offer);
  r.helm();r.tick();
  require(!r.host.navigationGranted(),"fresh stock input implicitly re-acquired hover");
  r.post(nav,"acquire");r.tick();
  r.mode=guided;r.tick();
  r.offer(transition_token+3,quadplane::hover_guided_offer);
  r.tick();r.helm();
  bool intercepted=false;
  r.before_send=[&](const auto& message) {
    if(message.msgid!=MAVLINK_MSG_ID_TUNNEL)return;
    intercepted=true;
    r.now+=.001;r.mode=5;r.phase=transition;r.samples();
  };
  for(unsigned i=0;i<10&&!intercepted;++i)r.tick();
  require(intercepted&&!r.host.navigationGranted()&&r.framesOf(MAVLINK_MSG_ID_TUNNEL)==1&&
    r.sent(MAV_CMD_DO_SET_MODE,1,quadplane::qloiter)==1,
    "pilot takeover at final send admitted an encoded target or competing mode");
}
} // namespace

int main(int argc,char** argv)
{
  try {
    require(argc==3,"expected control and mission DSO paths");
    complete(argv[1],argv[2]);
    cancelled_and_restarted(argv[1],argv[2]);
    changed_plan_at_handoff(argv[1],argv[2],MISSION_STATE_NOT_STARTED);
    changed_plan_at_handoff(argv[1],argv[2],MISSION_STATE_UNKNOWN);
    forward_helm(argv[1],argv[2]);
    forward_fences(argv[1],argv[2]);
    helm_and_retained_auto(argv[1],argv[2]);
    scripted_hover(argv[1],argv[2]);
    std::cout<<"PASS QuadPlane native mission and gated stock forward and hover helm\n";
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<error.what()<<'\n';return 1;
  }
}
