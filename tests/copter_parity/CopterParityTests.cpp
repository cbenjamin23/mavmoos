// Real Runtime and startup-loaded control provider with a deterministic fake peer.
#include "../protocol_faults/ProtocolSupport.h"
#include "../rover/NativeReplayDirectory.h"
#include "CopterFrame.h"
#include <array>
#include <functional>
#include <iostream>

namespace {
using namespace protocol_faults;
constexpr unsigned guided = 4;
constexpr unsigned loiter = 5;
constexpr unsigned rtl = 6;
constexpr unsigned land = 9;
constexpr unsigned pilot = 0;

class Rig {
public:
  explicit Rig(const std::string& module, bool enabled = true, bool shadow = false, bool helm = false,
               const char* frame = "quad",const std::string& replay_directory={})
    : type(copterFrameType(frame)),runtime([this](const auto& key, const auto& text, double, bool numeric) {
        if(!numeric) publications.push_back({key,text,now});
      }, [](const uint8_t*,std::size_t) { return false; }, true, shadow, [this] { return now; },type,
      MAV_AUTOPILOT_ARDUPILOTMEGA,!replay_directory.empty())
  {
    require(runtime.load(module,"arducopter_control"),"cannot load Copter control provider");
    runtime.configure("copter_frame",frame);
    runtime.configure("control_helm",helm?"true":"false");
    if(enabled) {
      runtime.configure("control_copter_adjustments","true");
      runtime.configure("command_speed_min","0.75");
      runtime.configure("control_precision_loiter","true");
      runtime.configure("dead_man_source","deadman_fixture");
    }
    runtime.setBoundedSend([this](const uint8_t* bytes,std::size_t size,double expires,
                                  const Runtime::SendEligibility& eligible) {
      const auto frame = decode(bytes,size);
      require(frame.sysid==245&&frame.compid==191,"unexpected bridge wire identity");
      if(before_handoff) before_handoff(frame);
      const bool accepted = eligible(now) && (!accept_handoff || accept_handoff(frame));
      frames.push_back({frame,now,expires,accepted});
      return accepted;
    });
    if(!replay_directory.empty()) {
      runtime.configureModules({});
      runtime.configureControlReplay(replay_directory,"copter-parity");
      runtime.finishStartup();
    }
    advance();
    fresh();
  }

  void advance(double seconds = .01) { now += seconds; }
  void tick() { runtime.tick(now); }
  void pump() { for(int i=0;i<3;++i) { advance(); tick(); } }
  void heartbeat(unsigned mode,bool armed=true,double effective=-1,uint8_t system=1)
  {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(system,1,&message,type,MAV_AUTOPILOT_ARDUPILOTMEGA,
      armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0,mode,MAV_STATE_ACTIVE);
    if(effective<0)runtime.message(message,now);
    else runtime.message(message,effective,now);
  }
  void position(unsigned heading = 359, int16_t north = 120, int16_t east = 160, int32_t height_mm = 5000)
  {
    mavlink_message_t message{};
    mavlink_msg_global_position_int_pack(1,1,&message,1000,423584560,-710875890,
      10000+height_mm,height_mm,north,east,0,heading*100);
    runtime.message(message,now);
  }
  void landed(uint8_t state)
  {
    mavlink_message_t message{};
    mavlink_msg_extended_sys_state_pack(1,1,&message,MAV_VTOL_STATE_UNDEFINED,state);
    runtime.message(message,now);
  }
  void fresh(unsigned mode = guided, bool armed = true, uint8_t state = MAV_LANDED_STATE_IN_AIR)
  {
    heartbeat(mode,armed);
    position();
    landed(state);
    mavlink_message_t message{};
    mavlink_home_position_t home{};
    home.latitude=423584560;
    home.longitude=-710875890;
    home.altitude=10000;
    mavlink_msg_home_position_encode(1,1,&message,&home);
    runtime.message(message,now);
    mavlink_ekf_status_report_t ekf{};
    ekf.flags=51;
    mavlink_msg_ekf_status_report_encode(1,1,&message,&ekf);
    runtime.message(message,now);
  }
  void mail(const std::string& key, const std::string& text,
            const std::string& source = "operator", double published = -1)
  {
    Mail mail;
    mail.key=key;
    mail.text=text;
    mail.source=source;
    mail.received_at=now;
    mail.published_at=published<0 ? now : published;
    runtime.mail(mail);
  }
  void request(const std::string& text, const std::string& key = "MAVMOOS_COMMAND")
  {
    advance();
    mail(key,text);
    tick();
  }
  void helmDemand()
  {
    mail("IVPHELM_ALLSTOP","clear","pHelmIvP");
    mail("IVPHELM_STATE","DRIVE","pHelmIvP");
    for(const auto& [key,value]:std::array<std::pair<const char*,double>,3>{
          {{"DESIRED_HEADING",90},{"DESIRED_SPEED",1},{"DESIRED_ALTITUDE",5}}}) {
      Mail input;
      input.key=key;
      input.value=value;
      input.numeric=true;
      input.source="pHelmIvP";
      input.received_at=input.published_at=now;
      runtime.mail(input);
    }
  }
  void ack(uint16_t command_id, uint8_t result = MAV_RESULT_ACCEPTED, uint8_t system = 1,
           uint8_t component = 1, uint8_t target_system = 245, uint8_t target_component = 191,
           double effective = -1)
  {
    mavlink_message_t message{};
    mavlink_msg_command_ack_pack(system,component,&message,command_id,result,0,0,
                                 target_system,target_component);
    if(effective<0) runtime.message(message,now);
    else runtime.message(message,effective,now);
  }
  void value(const std::string& name, float value, uint16_t index = UINT16_MAX,
             uint8_t type = MAV_PARAM_TYPE_INT8, uint8_t component = 1)
  {
    mavlink_param_value_t parameter{};
    std::copy(name.begin(),name.end(),parameter.param_id);
    parameter.param_value=value;
    parameter.param_count=1300;
    parameter.param_index=index;
    parameter.param_type=type;
    mavlink_message_t message{};
    mavlink_msg_param_value_encode(1,component,&message,&parameter);
    runtime.message(message,now);
  }
  bool result(const std::string& id,const std::string& status,const std::string& reason = "",
              const std::string& key = "MAVMOOS_COMMAND_RESULT") const
  {
    for(const auto& publication:publications) {
      if(publication.key!=key) continue;
      const auto report=fields(publication.text);
      if(report.at("id")==id&&report.at("status")==status&&
         (reason.empty()||report.at("reason")==reason)) return true;
    }
    return false;
  }
  std::size_t navigationFrames() const
  {
    return std::count_if(frames.begin(),frames.end(),[](const auto& frame) {
      return frame.accepted && (frame.message.msgid==MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED ||
                                frame.message.msgid==MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT);
    });
  }
  std::vector<mavlink_command_long_t> commandPackets(uint16_t id) const
  {
    std::vector<mavlink_command_long_t> result;
    for(const auto& frame:frames) {
      if(!frame.accepted||command(frame.message)!=id) continue;
      mavlink_command_long_t command{};
      mavlink_msg_command_long_decode(&frame.message,&command);
      require(command.target_system==1&&command.target_component==1&&command.confirmation==0,
              "command target or retry flag changed");
      result.push_back(command);
    }
    return result;
  }
  std::vector<std::string> namedReads() const
  {
    std::vector<std::string> result;
    for(const auto& frame:frames) {
      if(!frame.accepted||frame.message.msgid!=MAVLINK_MSG_ID_PARAM_REQUEST_READ) continue;
      mavlink_param_request_read_t read{};
      mavlink_msg_param_request_read_decode(&frame.message,&read);
      require(read.target_system==1&&read.target_component==1&&read.param_index==-1,
              "precision proof did not send an addressed named parameter read");
      result.emplace_back(read.param_id,std::find(read.param_id,read.param_id+16,'\0'));
    }
    return result;
  }

  double now=100;
  const unsigned type;
  std::vector<Frame> frames;
  std::vector<Publication> publications;
  std::function<void(const mavlink_message_t&)> before_handoff;
  std::function<bool(const mavlink_message_t&)> accept_handoff;
  Runtime runtime;
};

void speedPacket(const mavlink_command_long_t& packet, float limit)
{
  require(packet.param1==1&&packet.param2==limit&&packet.param3==-1&&packet.param4==0&&
          packet.param5==0&&packet.param6==0&&packet.param7==0,"wrong groundspeed limit wire shape");
}
void auxPacket(const mavlink_command_long_t& packet,float switch_position)
{
  require(packet.param1==39&&packet.param2==switch_position&&packet.param3==0&&packet.param4==0&&
          packet.param5==0&&packet.param6==0&&packet.param7==0,"wrong precision Loiter AUX wire shape");
}

void nativeLifecycle(const std::string& module,bool versioned,bool explicit_height,const char* frame)
{
  Rig rig(module,true,false,false,frame);
  rig.runtime.configure("takeoff_height","6");
  const std::string input=versioned?"MAVMOOS_CONTROL_REQUEST":"MAVMOOS_COMMAND";
  const std::string output=versioned?"MAVMOOS_CONTROL_RESULT":"MAVMOOS_COMMAND_RESULT";
  const auto report=[&](const std::string& id,const std::string& status) {
    return rig.result(id,status,"",output);
  };
  const auto submit=[&](const std::string& id,const std::string& fields,uint16_t wire,
                        const std::array<float,7>& expected) {
    const auto count=rig.commandPackets(wire).size();
    const std::string prefix=versioned?"schema=1,strategy=native,":"";
    rig.request(prefix+"id="+id+",action="+fields,input);
    const auto packets=rig.commandPackets(wire);
    require(packets.size()==count+1&&rig.runtime.controlBusy(),id+" did not take one native lease and send once");
    const auto& packet=packets.back();
    require(expected==std::array<float,7>{packet.param1,packet.param2,packet.param3,packet.param4,
                                        packet.param5,packet.param6,packet.param7},
            id+" changed its native wire parameters");
    rig.advance();
    rig.ack(wire);
    rig.tick();
    require(report(id,"ACK_OBSERVED")&&!report(id,"COMPLETED")&&rig.runtime.controlBusy(),
            id+" used its ACK as the observed outcome");
  };
  const auto completed=[&](const std::string& id) {
    rig.tick();
    require(report(id,"COMPLETED")&&!rig.runtime.controlBusy(),id+" did not close on independent state");
  };
  rig.advance();
  rig.fresh(loiter,false,MAV_LANDED_STATE_ON_GROUND);
  submit("guided","guided",MAV_CMD_DO_SET_MODE,{1,4});
  rig.advance();
  rig.heartbeat(guided,false);
  completed("guided");

  submit("arm","arm",MAV_CMD_COMPONENT_ARM_DISARM,{1});
  rig.advance();
  rig.heartbeat(guided,true);
  completed("arm");

  // The versioned interface documents a five-metre default. Legacy takeoff
  // reads its startup setting. An explicit height overrides either default.
  const float height=explicit_height?7.f:versioned?5.f:6.f;
  submit("takeoff",explicit_height?"takeoff,height=7":"takeoff",MAV_CMD_NAV_TAKEOFF,{0,0,0,0,0,0,height});
  rig.advance();
  rig.heartbeat(guided);
  rig.landed(MAV_LANDED_STATE_IN_AIR);
  rig.position(359,0,0,1000);
  rig.tick();
  require(report("takeoff","CONFIRMED")&&!report("takeoff","COMPLETED"),"airborne alone completed takeoff");
  rig.advance();
  rig.position(359,0,0,static_cast<int32_t>(height*1000));
  completed("takeoff");

  submit("hold","hold",MAV_CMD_DO_SET_MODE,{1,5});
  rig.advance();
  rig.heartbeat(loiter);
  completed("hold");

  // The two height cases also cover both terminal native modes on each
  // operator interface.
  const auto terminal_mode=explicit_height?rtl:land;
  submit("end-flight",explicit_height?(versioned?"return":"rtl"):"land",
         explicit_height?MAV_CMD_NAV_RETURN_TO_LAUNCH:MAV_CMD_NAV_LAND,{});
  rig.advance();
  rig.heartbeat(terminal_mode);
  rig.tick();
  require(report("end-flight","CONFIRMED")&&!report("end-flight","COMPLETED"),
          "landing mode alone completed the terminal flight command");
  rig.advance();
  rig.landed(MAV_LANDED_STATE_ON_GROUND);
  completed("end-flight");

  submit("disarm","disarm",MAV_CMD_COMPONENT_ARM_DISARM,{});
  rig.advance();
  rig.heartbeat(terminal_mode,false);
  completed("disarm");
  require(rig.navigationFrames()==0,"native-only lifecycle sent a navigation target");
}
}

int main(int argc,char** argv)
{
  try {
    require(argc==2,"usage: copter_parity_tests ARDUCOPTER_CONTROL_MODULE");
    const std::string module=argv[1];
    unsigned count=0;
    const auto test=[&](const char* name,const auto& run) {
      run();
      ++count;
      std::cout<<"PASS "<<name<<'\n';
    };
    test("Copter command wrapper uses the host journal across public request interfaces",[&] {
      NativeReplayDirectory journal;
      {
        Rig first(module,true,false,false,"quad",journal.path.string());
        first.advance();first.fresh(guided,true,MAV_LANDED_STATE_ON_GROUND);
        first.request("schema=1,id=takeoff-replay,action=takeoff,strategy=native,height=5","MAVMOOS_CONTROL_REQUEST");
        first.pump();
        require(first.commandPackets(MAV_CMD_NAV_TAKEOFF).size()==1 &&
                first.runtime.controlRequestSeen("takeoff-replay"),"delegated command did not reserve its public ID");
      }
      Rig restarted(module,true,false,false,"quad",journal.path.string());
      restarted.advance();restarted.fresh(guided,true,MAV_LANDED_STATE_ON_GROUND);
      restarted.request("id=takeoff-replay,action=takeoff,height=5");
      require(restarted.commandPackets(MAV_CMD_NAV_TAKEOFF).empty()&&
              restarted.result("takeoff-replay","DUPLICATE","native_id_already_recorded"),
              "the other public interface replayed the consumed ID after restart");
    });
    test("configured Copter frames bind exact identity and the same opt-in capabilities",[&] {
      for(const auto* frame:{"hexa","octa"}) {
        double now=10;
        const unsigned type=copterFrameType(frame);
        Runtime host([](const auto&,const auto&,double,bool) {},[](const uint8_t*,std::size_t) {return false;},
                     true,false,[&]{return now;},type,MAV_AUTOPILOT_ARDUPILOTMEGA);
        require(host.load(module,"arducopter_control"),"cannot load Copter provider");
        host.configure("copter_frame",frame);
        require(host.hasCapability("control.arm") && host.hasCapability("control.return.native_rtl") &&
                host.hasCapability("control.navigation.helm") && host.hasCapability("control.operations.native") &&
                !host.hasCapability("control.mission.start") && !host.hasCapability("control.precision_loiter.enable"),
                "Copter frame capability composition lost a baseline path or enabled an opt-in facet");
        host.configure("control_precision_loiter","true");
        host.configure("control_copter_adjustments","true");
        host.configure("control_mission_execution","true");
        require(host.hasCapability("control.mission.start") && host.hasCapability("control.mission.resume") &&
                host.hasCapability("control.guided.speed_adjustment") &&
                host.hasCapability("control.precision_loiter.enable"),"Copter frame did not expose opted-in capabilities");
        const auto channels=host.subscriptions();
        require(std::find(channels.begin(),channels.end(),"MAVMOOS_COMMAND")!=channels.end() &&
                std::find(channels.begin(),channels.end(),"MAVMOOS_CONTROL_REQUEST")!=channels.end() &&
                std::find(channels.begin(),channels.end(),"MAVMOOS_MISSION_CONTROL")!=channels.end(),
                "qualified Copter frame lost an opted-in subscription");
        const auto heartbeat=[&](unsigned received_type,unsigned autopilot) {
          mavlink_message_t message{};
          mavlink_msg_heartbeat_pack(1,1,&message,received_type,autopilot,0,guided,MAV_STATE_ACTIVE);
          now+=.01;
          host.message(message,now);
        };
        heartbeat(MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_ARDUPILOTMEGA);
        require(!host.vehicle().heartbeat_valid,"selected frame accepted a quad heartbeat");
        heartbeat(type,MAV_AUTOPILOT_PX4);
        require(!host.vehicle().heartbeat_valid,"selected frame accepted foreign firmware");
        heartbeat(type,MAV_AUTOPILOT_ARDUPILOTMEGA);
        require(host.vehicle().heartbeat_valid && host.vehicle().type==type,"selected frame was not accepted");
        bool refused=false;
        try {host.configure("copter_frame","quad");} catch(const std::runtime_error&) {refused=true;}
        require(refused,"operational frame identity could be changed");
      }
    });
    test("legacy and versioned Copter lifecycle sends exact commands and needs independent state",[&] {
      for(const auto* frame:{"quad","hexa","octa"})
        for(bool versioned:{false,true})
          for(bool explicit_height:{false,true}) nativeLifecycle(module,versioned,explicit_height,frame);
    });
    test("Copter receive-only and shadow preserve the configured heartbeat source",[&] {
      for(bool shadow:{false,true}) {
        double now=10;
        Runtime host([](const auto&,const auto&,double,bool){},[](const uint8_t*,std::size_t){return false;},
          shadow,shadow,[&]{return now;},MAV_TYPE_HEXAROTOR);
        require(host.load(module,"arducopter_control"),"cannot load Copter provider");
        host.configure("copter_frame","hexa");
        mavlink_message_t message{};
        mavlink_msg_heartbeat_pack(9,2,&message,MAV_TYPE_HEXAROTOR,MAV_AUTOPILOT_ARDUPILOTMEGA,0,
          loiter,MAV_STATE_ACTIVE);
        now+=.01;host.message(message,now);
        require(host.vehicle().heartbeat_valid&&host.vehicle().mode==loiter,
          "non-effectful Copter rejected the configured non-1/1 heartbeat");
      }
    });
    test("Copter Loiter setup fences pilot Guided and failed local handoff",[&] {
      for(const auto* frame:{"hexa","octa"}) {
        Rig pilot_entry(module,true,false,true,frame);
        pilot_entry.fresh(loiter);
        bool intercepted=false;
        pilot_entry.before_handoff=[&](const auto& packet) {
          if(command(packet)!=MAV_CMD_DO_SET_MODE)return;
          intercepted=true;pilot_entry.advance();pilot_entry.heartbeat(guided);
        };
        pilot_entry.request("acquire","MAVMOOS_NAV_CONTROL");
        pilot_entry.advance(.15);pilot_entry.helmDemand();pilot_entry.tick();
        require(intercepted&&pilot_entry.commandPackets(MAV_CMD_DO_SET_MODE).empty()&&
          !pilot_entry.runtime.navigationGranted()&&pilot_entry.navigationFrames()==0,
          "pilot-selected Guided before the final gate retained old permission or a stop target");

        Rig failed(module,true,false,true,frame);
        failed.fresh(loiter);
        failed.accept_handoff=[](const auto& packet){return command(packet)!=MAV_CMD_DO_SET_MODE;};
        failed.request("acquire","MAVMOOS_NAV_CONTROL");
        require(!failed.runtime.navigationGranted()&&failed.commandPackets(MAV_CMD_DO_SET_MODE).empty(),
          "rejected local mode handoff preserved the pending grant");
        failed.advance(.15);failed.fresh(guided);failed.helmDemand();failed.tick();
        require(!failed.runtime.navigationGranted()&&failed.navigationFrames()==0,
          "later Guided revived a failed acquisition");
        failed.request("acquire","MAVMOOS_NAV_CONTROL");
        failed.advance();failed.helmDemand();failed.tick();
        require(failed.runtime.navigationGranted()&&failed.navigationFrames()==1&&
          failed.commandPackets(MAV_CMD_DO_SET_MODE).empty(),
          "fresh explicit already-Guided acquire required a redundant mode command");
      }
    });
    test("Copter Loiter acquisition needs newer selected Guided and fresh helm fields",[&] {
      for(const auto* frame:{"hexa","octa"}) {
        Rig rig(module,true,false,true,frame);
        rig.fresh(loiter);rig.request("acquire","MAVMOOS_NAV_CONTROL");
        const auto sent=rig.now;
        require(rig.commandPackets(MAV_CMD_DO_SET_MODE).size()==1,"Loiter setup did not send one mode request");
        rig.advance();rig.heartbeat(guided,true,-1,9);
        require(rig.runtime.vehicle().mode==loiter,"foreign heartbeat changed the selected mode");
        rig.advance();rig.heartbeat(guided,true,sent);rig.helmDemand();rig.tick();
        require(rig.runtime.navigationGranted()&&rig.navigationFrames()==0&&
          !rig.runtime.suspendNavigation(rig.now+.2),"same-time Guided authorized navigation or a pause hold");
        rig.advance();rig.fresh(guided);rig.tick();
        require(rig.navigationFrames()==0,"pre-confirmation helm fields were reused");
        rig.advance(.12);rig.helmDemand();rig.tick();
        require(rig.runtime.navigationGranted()&&rig.navigationFrames()==1,
          "new selected Guided and new helm fields did not authorize motion");

        rig.advance();rig.mail("IVPHELM_ALLSTOP","NothingToDo","pHelmIvP");rig.tick();
        const auto after_pause=rig.navigationFrames();
        require(!rig.runtime.navigationGranted(),"normal pause did not suspend permission");
        rig.advance(.12);rig.helmDemand();rig.tick();
        require(rig.runtime.navigationGranted()&&rig.navigationFrames()==after_pause+1&&
          rig.commandPackets(MAV_CMD_DO_SET_MODE).size()==1,
          "normal handoff required another mode change or failed to resume");
      }
    });
    test("direct Copter position operation waits for its own Guided observation",[&] {
      for(const auto* frame:{"hexa","octa"}) {
        Rig rig(module,true,false,true,frame);
        rig.fresh(loiter);
        rig.request("schema=1,id=position,action=hold,strategy=guided","MAVMOOS_CONTROL_REQUEST");
        const auto sent=rig.now;
        require(rig.commandPackets(MAV_CMD_DO_SET_MODE).size()==1,"direct hold did not request Guided");
        rig.advance();rig.heartbeat(guided,true,sent);rig.tick();
        require(rig.navigationFrames()==0&&
          !rig.result("position","CONFIRMED","","MAVMOOS_CONTROL_RESULT"),
          "stale Guided confirmed or transmitted direct position work");
        rig.advance(.12);rig.fresh(guided);rig.tick();
        require(rig.navigationFrames()==1&&rig.result("position","CONFIRMED","","MAVMOOS_CONTROL_RESULT"),
          "fresh Guided did not confirm the admitted position operation");
      }
    });
    test("versioned Loiter retires active helm and requires a fresh Guided reacquire",[&] {
      Rig rig(module,true,false,true);
      rig.request("acquire","MAVMOOS_NAV_CONTROL");
      rig.advance();
      rig.helmDemand();
      rig.tick();
      const auto moving=rig.navigationFrames();
      require(moving==1&&rig.runtime.navigationGranted(),"fresh helm did not send one Guided target");

      rig.request("schema=1,id=loiter,action=hold,strategy=native","MAVMOOS_CONTROL_REQUEST");
      const auto mode_packets=rig.commandPackets(MAV_CMD_DO_SET_MODE);
      require(mode_packets.size()==1&&mode_packets.back().param2==5&&
              !rig.runtime.navigationGranted()&&rig.navigationFrames()==moving,
              "native Loiter did not cancel helm at admission");
      rig.advance();
      rig.ack(MAV_CMD_DO_SET_MODE);
      rig.tick();
      require(rig.result("loiter","ACK_OBSERVED","","MAVMOOS_CONTROL_RESULT")&&
              !rig.result("loiter","COMPLETED","","MAVMOOS_CONTROL_RESULT"),
              "Loiter ACK was reported as mode confirmation");
      rig.request("acquire","MAVMOOS_NAV_CONTROL");
      require(rig.result("loiter","CANCELLED","legacy_navigation_request","MAVMOOS_CONTROL_RESULT")&&
              rig.runtime.controlBusy()&&!rig.runtime.navigationGranted()&&
              rig.commandPackets(MAV_CMD_DO_SET_MODE).size()==1,
              "explicit helm acquire failed to cancel the public request or bypassed its native lease");

      rig.advance();
      rig.fresh(loiter);
      rig.helmDemand();
      rig.tick();
      require(rig.result("op.loiter","COMPLETED","observed_outcome")&&
              !rig.result("loiter","COMPLETED","","MAVMOOS_CONTROL_RESULT")&&
              !rig.runtime.controlBusy()&&!rig.runtime.navigationGranted()&&rig.navigationFrames()==moving,
              "Loiter confirmation failed to retire only the sent native effect");
      rig.request("acquire","MAVMOOS_NAV_CONTROL");
      const auto reacquire=rig.commandPackets(MAV_CMD_DO_SET_MODE);
      require(reacquire.size()==2&&reacquire.back().param2==4&&rig.navigationFrames()==moving,
              "explicit Loiter reacquire did not request Guided exactly once");
      rig.advance(.12);
      rig.fresh(guided);
      rig.tick();
      require(rig.navigationFrames()==moving,"Guided observation reused pre-acquisition demands");
      rig.advance();
      rig.helmDemand();
      rig.tick();
      require(rig.navigationFrames()==moving+1&&rig.runtime.navigationGranted(),
              "Guided confirmation plus new helm inputs did not resume motion");
    });
    test("new command paths are opt-in and shadow remains silent",[&] {
      Rig off(module,false);
      off.request("id=default,action=reset_speed_min");
      require(off.result("default","REJECTED","guided_adjustments_disabled")&&
              off.commandPackets(MAV_CMD_DO_CHANGE_SPEED).empty(),"adjustments enabled by default");
      Rig shadow(module,true,true);
      shadow.request("id=shadow,action=reset_speed_min");
      require(shadow.result("shadow","REJECTED","host_send_inhibited")&&shadow.frames.empty(),
              "shadow attempted MAVLink egress");
    });
    test("legacy adjustments require helm release and use fresh speed after a mode change",[&] {
      Rig rig(module,true,false,true);
      rig.request("acquire","MAVMOOS_NAV_CONTROL");
      rig.advance();
      rig.helmDemand();
      rig.tick();
      require(rig.runtime.navigationGranted()&&rig.navigationFrames()==1,"test did not start active helm motion");
      for(const std::string request:{"change_speed,delta=0.5","reset_speed_min","change_course,delta=15"}) {
        const auto id="busy-"+request.substr(0,request.find(','));
        rig.request("id="+id+",action="+request);
        require(rig.result(id,"REJECTED","release_helm_and_control_first"),
                "native adjustment interfered with a held helm grant");
      }
      require(rig.runtime.navigationGranted()&&rig.commandPackets(MAV_CMD_DO_CHANGE_SPEED).empty()&&
              rig.commandPackets(MAV_CMD_CONDITION_YAW).empty(),"rejected adjustment changed control ownership");
      rig.request("release","MAVMOOS_NAV_CONTROL");
      require(!rig.runtime.navigationGranted(),"explicit release did not retire helm");
      rig.request("id=reset,action=reset_speed_min");
      auto packets=rig.commandPackets(MAV_CMD_DO_CHANGE_SPEED);
      require(packets.size()==1,"explicitly released helm still inhibited a valid reset");
      speedPacket(packets.back(),.75f);
      rig.advance();
      rig.ack(MAV_CMD_DO_CHANGE_SPEED);
      require(rig.result("reset","ACK_OBSERVED")&&!rig.result("reset","ACCEPTED"),
              "a speed ACK claimed a confirmed cap");
      rig.advance();
      rig.heartbeat(loiter);
      rig.advance();
      rig.fresh(guided);
      rig.request("id=after-mode,action=change_speed,delta=.25");
      packets=rig.commandPackets(MAV_CMD_DO_CHANGE_SPEED);
      require(packets.size()==2&&!rig.runtime.navigationGranted(),"mode return altered ownership");
      speedPacket(packets.back(),2.25f);
    });
    test("groundspeed reset and relative adjustment reject unbound speed-cap claims",[&] {
      Rig rig(module);
      rig.runtime.configure("command_ack_timeout","0.5");
      const double floor=rig.now;
      rig.before_handoff=[&](const mavlink_message_t& frame) {
        if(command(frame)==MAV_CMD_DO_CHANGE_SPEED) rig.ack(MAV_CMD_DO_CHANGE_SPEED,MAV_RESULT_DENIED);
      };
      rig.request("id=reset,action=reset_speed_min");
      rig.before_handoff={};
      auto packets=rig.commandPackets(MAV_CMD_DO_CHANGE_SPEED);
      require(packets.size()==1&&rig.runtime.controlBusy(),"pre-handoff ACK closed reset");
      speedPacket(packets.back(),.75f);
      rig.ack(MAV_CMD_DO_CHANGE_SPEED,MAV_RESULT_DENIED);
      rig.advance();rig.ack(MAV_CMD_DO_CHANGE_SPEED,MAV_RESULT_DENIED,2);
      rig.advance();rig.ack(MAV_CMD_DO_CHANGE_SPEED,MAV_RESULT_DENIED,1,2);
      rig.advance();rig.ack(MAV_CMD_DO_CHANGE_SPEED,MAV_RESULT_DENIED,1,1,246);
      rig.advance();rig.ack(MAV_CMD_DO_CHANGE_SPEED,MAV_RESULT_DENIED,1,1,245,192);
      rig.advance();rig.ack(MAV_CMD_CONDITION_YAW,MAV_RESULT_DENIED);
      require(rig.runtime.controlBusy()&&!rig.result("reset","ACK_OBSERVED"),"foreign or equal-time ACK was reported");
      rig.advance();rig.ack(MAV_CMD_DO_CHANGE_SPEED,MAV_RESULT_ACCEPTED,1,1,245,191,floor);
      require(rig.result("reset","ACK_OBSERVED","uncorrelated_ack_result_0")&&
              !rig.result("reset","ACCEPTED")&&rig.runtime.controlBusy(),
              "an ACK claimed a confirmed speed cap or released the lease");
      rig.advance(.5);rig.tick();rig.fresh();
      require(rig.result("reset","UNCONFIRMED","guided_speed_cap_not_observable")&&!rig.runtime.controlBusy(),
              "speed observation deadline did not close");
      rig.request("id=change,action=change_speed,delta=0.5");
      packets=rig.commandPackets(MAV_CMD_DO_CHANGE_SPEED);
      require(packets.size()==2,"relative speed request did not send");
      speedPacket(packets.back(),2.5f);
      rig.advance();rig.ack(MAV_CMD_DO_CHANGE_SPEED);
      rig.advance(.5);rig.tick();rig.fresh();
      rig.request("id=outside,action=change_speed,delta=10");
      require(rig.commandPackets(MAV_CMD_DO_CHANGE_SPEED).size()==2&&
              rig.result("outside","REJECTED","speed_outside_profile"),"out-of-range speed reached wire");
    });
    test("speed failure and late ACK retire the lease without automatic replay",[&] {
      Rig rig(module);
      rig.runtime.configure("command_ack_timeout","0.5");
      rig.request("id=late,action=change_speed,delta=0.5");
      auto packets=rig.commandPackets(MAV_CMD_DO_CHANGE_SPEED);
      require(packets.size()==1,"speed request without previous cap did not send");
      speedPacket(packets.back(),2.5f);
      rig.advance(.5);rig.ack(MAV_CMD_DO_CHANGE_SPEED);rig.tick();
      require(rig.result("late","UNCONFIRMED")&&!rig.result("late","ACCEPTED")&&
              !rig.runtime.controlBusy(),"late ACK revived expired speed operation");
      rig.fresh();
      rig.request("id=new,action=reset_speed_min");
      rig.advance();rig.ack(MAV_CMD_DO_CHANGE_SPEED); // A positive reply for the older request.
      require(rig.result("new","ACK_OBSERVED")&&!rig.result("new","ACCEPTED"),
              "older positive ACK falsely established the reset cap");
      rig.advance(.5);rig.tick();rig.fresh();
      rig.request("id=relative,action=change_speed,delta=0.5");
      packets=rig.commandPackets(MAV_CMD_DO_CHANGE_SPEED);
      require(packets.size()==3,"relative speed request after delayed ACK missing");
      speedPacket(packets.back(),2.5f);
      rig.advance(.5);rig.tick();
      rig.fresh();
      rig.accept_handoff=[](const mavlink_message_t& frame) { return command(frame)!=MAV_CMD_DO_CHANGE_SPEED; };
      rig.request("id=failed,action=reset_speed_min");
      rig.advance();rig.ack(MAV_CMD_DO_CHANGE_SPEED);
      require(rig.result("failed","FAILED","local_send_not_accepted")&&
              !rig.result("failed","ACCEPTED")&&!rig.runtime.controlBusy()&&
              rig.commandPackets(MAV_CMD_DO_CHANGE_SPEED).size()==3,"failed local speed send was accepted or retried");
    });
    test("absolute yaw wraps and completes from a separate fresh position",[&] {
      Rig rig(module);
      rig.request("id=yaw,action=change_course,delta=15");
      const auto packets=rig.commandPackets(MAV_CMD_CONDITION_YAW);
      require(packets.size()==1,"yaw command missing");
      const auto& yaw=packets.front();
      require(yaw.param1==14&&yaw.param2==30&&yaw.param3==0&&yaw.param4==0&&
              yaw.param5==0&&yaw.param6==0&&yaw.param7==0,"wrong absolute shortest-turn yaw packet");
      rig.advance();rig.ack(MAV_CMD_CONDITION_YAW);rig.tick();
      require(rig.result("yaw","ACK_OBSERVED")&&!rig.result("yaw","COMPLETED"),
              "yaw ACK or cached pose completed command");
      rig.advance();rig.position(355);rig.tick();
      require(!rig.result("yaw","COMPLETED"),"out-of-tolerance yaw completed");
      rig.advance();rig.position(14);rig.tick();
      require(rig.result("yaw","COMPLETED","observed_outcome")&&!rig.runtime.controlBusy(),"fresh yaw outcome missed");
    });
    test("precision Loiter requires two locally sent named proofs",[&] {
      Rig rig(module);
      rig.runtime.configure("command_ack_timeout","0.5");
      rig.advance();rig.fresh(loiter);
      rig.before_handoff=[&](const mavlink_message_t& frame) {
        if(frame.msgid==MAVLINK_MSG_ID_PARAM_REQUEST_READ) rig.value("PLND_ENABLED",1);
      };
      rig.request("id=precision,action=precision_loiter_on");
      rig.before_handoff={};
      require(rig.namedReads()==std::vector<std::string>{"PLND_ENABLED"},"first precision proof is missing or out of order");
      rig.advance();rig.value("PLND_TYPE",1);
      rig.advance();rig.value("PLND_ENABLED",1,14);
      rig.advance();rig.value("PLND_ENABLED",1,UINT16_MAX,MAV_PARAM_TYPE_INT8,3);
      rig.pump();
      require(rig.namedReads()==std::vector<std::string>{"PLND_ENABLED"}&&
              rig.commandPackets(MAV_CMD_DO_AUX_FUNCTION).empty(),
              "pre-handoff, indexed, foreign or wrong-name value advanced precision proof");
      rig.advance();rig.value("PLND_ENABLED",1);rig.pump();
      require(rig.namedReads()==std::vector<std::string>{"PLND_ENABLED","PLND_TYPE"}&&
              rig.commandPackets(MAV_CMD_DO_AUX_FUNCTION).empty(),"precision sent without both proofs");
      rig.advance();rig.value("PLND_TYPE",4);rig.tick();
      auto packets=rig.commandPackets(MAV_CMD_DO_AUX_FUNCTION);
      require(packets.size()==1,"precision AUX enable missing");
      auxPacket(packets.back(),2.f);
      rig.advance();rig.ack(MAV_CMD_DO_AUX_FUNCTION);
      require(rig.result("precision","ACK_OBSERVED","uncorrelated_ack_result_0")&&rig.runtime.controlBusy(),
              "precision AUX acceptance confused with target tracking");
      rig.advance(.5);rig.tick();rig.fresh(loiter);
      require(rig.result("precision","UNCONFIRMED","precision_aux_state_not_observable"),
              "precision enable did not retire without a state claim");
      rig.request("id=off,action=precision_loiter_off");
      packets=rig.commandPackets(MAV_CMD_DO_AUX_FUNCTION);
      require(packets.size()==2&&rig.namedReads().size()==2,"precision disable did not send its own AUX without proof reads");
      auxPacket(packets.back(),0.f);
      rig.advance();rig.ack(MAV_CMD_DO_AUX_FUNCTION); // Delayed acceptance of the older enable.
      require(rig.result("off","ACK_OBSERVED")&&!rig.result("off","ACCEPTED")&&rig.runtime.controlBusy(),
              "delayed enable ACK confirmed precision disable");
      rig.advance(.5);rig.tick();
      require(rig.result("off","UNCONFIRMED","precision_aux_state_not_observable")&&!rig.runtime.controlBusy()&&
              rig.commandPackets(MAV_CMD_DO_AUX_FUNCTION).size()==2,"precision ACK extended the original window or caused replay");
    });
    test("precision proof cannot survive invalid configuration or a pilot mode transition",[&] {
      Rig invalid(module);
      invalid.advance();invalid.fresh(loiter);
      invalid.request("id=disabled,action=precision_loiter_on");
      invalid.advance();invalid.value("PLND_ENABLED",0);invalid.pump();
      require(invalid.result("disabled","REJECTED","precision_landing_not_configured")&&
              invalid.namedReads().size()==1&&invalid.commandPackets(MAV_CMD_DO_AUX_FUNCTION).empty(),
              "disabled FC precision configuration allowed AUX");

      Rig unsupported(module);
      unsupported.advance();unsupported.fresh(loiter);
      unsupported.request("id=unsupported,action=precision_loiter_on");
      unsupported.advance();unsupported.value("PLND_ENABLED",1);unsupported.pump();
      unsupported.advance();unsupported.value("PLND_TYPE",5);unsupported.tick();
      require(unsupported.result("unsupported","REJECTED","precision_landing_not_configured")&&
              unsupported.commandPackets(MAV_CMD_DO_AUX_FUNCTION).empty(),"unknown FC precision type allowed AUX");

      Rig takeover(module);
      takeover.advance();takeover.fresh(loiter);
      takeover.request("id=interrupted,action=precision_loiter_on");
      takeover.advance();takeover.value("PLND_ENABLED",1);takeover.pump();
      takeover.advance();takeover.value("PLND_TYPE",1);
      takeover.advance();takeover.heartbeat(pilot);
      takeover.advance();takeover.heartbeat(loiter);
      takeover.tick();
      require(takeover.result("interrupted","CANCELLED","flight_authority_changed")&&
              takeover.commandPackets(MAV_CMD_DO_AUX_FUNCTION).empty(),"out-and-back pilot mode revived precision proof");
    });
    test("dead-man RTL requires a fresh event from the configured producer",[&] {
      Rig rig(module);
      rig.advance();
      rig.mail("DEAD_MAN_POST_INTERRUPT","true","other");
      rig.mail("DEAD_MAN_POST_INTERRUPT","false","deadman_fixture");
      rig.mail("DEAD_MAN_POST_INTERRUPT","true","deadman_fixture",rig.now-3);
      rig.tick();
      require(rig.commandPackets(MAV_CMD_NAV_RETURN_TO_LAUNCH).empty(),"untrusted event produced RTL");
      require(rig.runtime.acquireNavigation("owned_helm"),"fixture could not grant Guided navigation");
      rig.advance();const auto event=rig.now;
      rig.mail("DEAD_MAN_POST_INTERRUPT","true","deadman_fixture");rig.tick();
      const auto packets=rig.commandPackets(MAV_CMD_NAV_RETURN_TO_LAUNCH);
      require(packets.size()==1&&rig.result("deadman.1","SUBMITTED")&&!rig.runtime.navigationGranted(),
              "trusted dead-man event did not revoke helm and submit RTL");
      const auto& packet=packets.front();
      require(packet.param1==0&&packet.param2==0&&packet.param3==0&&packet.param4==0&&
              packet.param5==0&&packet.param6==0&&packet.param7==0,"dead-man RTL parameters changed");
      rig.advance();rig.ack(MAV_CMD_NAV_RETURN_TO_LAUNCH);rig.tick();
      require(!rig.result("deadman.1","COMPLETED"),"RTL ACK completed return");
      rig.advance();rig.heartbeat(rtl);rig.tick();
      require(rig.result("deadman.1","CONFIRMED")&&!rig.result("deadman.1","COMPLETED"),
              "RTL mode was missed or completed return");
      rig.advance();rig.landed(MAV_LANDED_STATE_ON_GROUND);rig.tick();
      require(rig.result("deadman.1","COMPLETED")&&!rig.runtime.controlBusy(),"observed return did not close lease");
      rig.advance();rig.fresh();
      rig.advance();rig.mail("DEAD_MAN_POST_INTERRUPT","true","deadman_fixture",event);rig.tick();
      require(rig.commandPackets(MAV_CMD_NAV_RETURN_TO_LAUNCH).size()==1,"retained event replayed RTL");
    });
    test("dead-man does not replace LAND or act in pilot mode",[&] {
      Rig active_land(module);
      active_land.advance();active_land.fresh(land);
      active_land.mail("DEAD_MAN_POST_INTERRUPT","true","deadman_fixture");active_land.tick();
      require(active_land.result("deadman.1","NO_OP","land_in_progress")&&
              active_land.commandPackets(MAV_CMD_NAV_RETURN_TO_LAUNCH).empty(),"dead-man interrupted observed LAND");

      Rig queued_land(module);
      queued_land.advance();queued_land.mail("MAVMOOS_COMMAND","id=land,action=land");
      queued_land.mail("DEAD_MAN_POST_INTERRUPT","true","deadman_fixture");queued_land.tick();
      require(queued_land.result("deadman.1","NO_OP","land_in_progress")&&
              queued_land.commandPackets(MAV_CMD_NAV_LAND).size()==1&&
              queued_land.commandPackets(MAV_CMD_NAV_RETURN_TO_LAUNCH).empty(),"dead-man replaced queued LAND");

      Rig manual(module);
      manual.advance();manual.fresh(pilot);
      manual.mail("DEAD_MAN_POST_INTERRUPT","true","deadman_fixture");manual.tick();
      require(manual.result("deadman.1","REJECTED","guided_or_loiter_required")&&
              manual.commandPackets(MAV_CMD_NAV_RETURN_TO_LAUNCH).empty(),"dead-man sent RTL during pilot control");
    });
    test("dead-man final send checks authority after admission",[&] {
      Rig rig(module);
      bool reached_handoff=false;
      rig.before_handoff=[&](const mavlink_message_t& frame) {
        if(command(frame)!=MAV_CMD_NAV_RETURN_TO_LAUNCH) return;
        reached_handoff=true;
        rig.advance();rig.heartbeat(pilot);
        rig.advance();rig.heartbeat(guided);
      };
      rig.advance();
      rig.mail("DEAD_MAN_POST_INTERRUPT","true","deadman_fixture");rig.tick();
      require(reached_handoff&&rig.result("deadman.1","CANCELLED","flight_authority_changed")&&
              rig.commandPackets(MAV_CMD_NAV_RETURN_TO_LAUNCH).empty()&&!rig.runtime.controlBusy(),
              "queued dead-man RTL survived pilot mode before actual handoff");
    });
    std::cout<<"PASS "<<count<<" Copter parity groups through real Runtime and fake I/O\n";
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<"FAIL "<<error.what()<<'\n';
    return 1;
  }
}
