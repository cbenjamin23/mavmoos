#include "../protocol_faults/ProtocolSupport.h"
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <iostream>
#include <unistd.h>

using namespace protocol_faults;
namespace {
namespace rover=ardu_rover_mission;
constexpr const char* request_key="MAVMOOS_MISSION_CONTROL";
constexpr const char* result_key="MAVMOOS_MISSION_EXECUTION_RESULT";
constexpr const char* recovery_key="MAVMOOS_AUTO_RECOVERY_REQUEST";
constexpr int32_t latitude=423584560,longitude=-710875890;

struct Directory {
  std::filesystem::path path;
  Directory()
  {
    auto pattern=(std::filesystem::temp_directory_path()/"mavmoos-ardu-rover-auto-XXXXXX").string();
    require(mkdtemp(pattern.data())!=nullptr,"private recovery fixture could not be created");
    path=pattern;
  }
  ~Directory() { std::error_code ignored;std::filesystem::remove_all(path,ignored); }
  bool marked() const {return std::filesystem::exists(path/"auto-may-continue");}
};

struct Rig {
  double now=100;
  unsigned mode=rover::hold_mode;
  uint8_t type,mission_state=MISSION_STATE_NOT_STARTED,mission_mode=0;
  uint16_t seq=0,total=2;
  bool armed=false,have_plan=false,send_progress=true,send_pose=true;
  int32_t lat=latitude;
  int16_t speed=0;
  std::string process;
  std::vector<Frame> frames;
  std::vector<Publication> publications;
  std::function<void(const mavlink_message_t&)> before_handoff;
  std::size_t answered=0;
  Runtime host;

  Rig(const char* control,const char* missions,const Directory& directory,bool boat=false,std::string session="one")
    :type(boat?MAV_TYPE_SURFACE_BOAT:MAV_TYPE_GROUND_ROVER),process(std::move(session)),
     host([this](const auto& key,const auto& value,double,bool numeric) {
       if(!numeric) publications.push_back({key,value,now});
     },[](const uint8_t*,std::size_t) {throw std::runtime_error("unbounded sender used");return false;},
     true,false,[this]{return now;},type,MAV_AUTOPILOT_ARDUPILOTMEGA,true)
  {
    require(host.load(control,"ardurover_control"),"real Rover DSO could not be loaded");
    require(host.load(missions,"missions"),"real missions DSO could not be loaded");
    host.configure("vehicle_profile",boat?"ardurover_boat":"ardurover");
    host.configure("control_native_actions","true");
    host.configure("control_mission_execution","true");
    host.configureModules({});
    host.setFirmwareRequestsEnabled(false);
    host.configureAutoRecovery(directory.path.string(),process);
    host.setBoundedSend([this](const uint8_t* data,std::size_t size,double expires,const Runtime::SendEligibility& live) {
      const auto packet=decode(data,size);
      if(before_handoff) before_handoff(packet);
      const bool accepted=live(now);
      frames.push_back({packet,now,expires,accepted});
      return accepted;
    });
    host.finishStartup();
  }

  static MissionItem point(unsigned seq)
  {
    MissionItem item;
    item.seq=seq;item.frame=seq?MAV_FRAME_GLOBAL_RELATIVE_ALT:MAV_FRAME_GLOBAL;
    item.x=latitude+int32_t(seq*1000);item.y=longitude;item.z=seq?0:10;
    return item;
  }
  void heartbeat()
  {
    mavlink_message_t m{};
    mavlink_msg_heartbeat_pack(1,1,&m,type,MAV_AUTOPILOT_ARDUPILOTMEGA,
      MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|(armed?MAV_MODE_FLAG_SAFETY_ARMED:0),mode,MAV_STATE_ACTIVE);
    host.message(m,now);
  }
  void progress(uint8_t sys=1,uint8_t comp=1,double age=0)
  {
    mavlink_message_t m{};
    mavlink_msg_mission_current_pack(sys,comp,&m,seq,total,mission_state,mission_mode,0,0,0);
    host.message(m,now-age,now,"QUEUE_EMPTY_FLOOR");
  }
  void pose(uint8_t sys=1)
  {
    mavlink_message_t m{};
    mavlink_msg_global_position_int_pack(sys,1,&m,uint32_t(std::lround(now*1e3)),lat,longitude,10000,0,speed,0,0,0);
    host.message(m,now);
  }
  void samples()
  {
    heartbeat();
    mavlink_message_t m{};
    const float q[]{1,0,0,0};
    mavlink_msg_home_position_pack(1,1,&m,latitude,longitude,10000,0,0,0,q,0,0,0,0);
    host.message(m,now);
    mavlink_msg_ekf_status_report_pack(1,1,&m,EKF_ATTITUDE|EKF_VELOCITY_HORIZ|EKF_POS_HORIZ_ABS,0,0,0,0,0,0);
    host.message(m,now);
    if(send_pose)pose();
    if(have_plan&&send_progress)progress();
  }
  void answer()
  {
    while(answered<frames.size()) {
      const auto frame=frames[answered++];
      if(!frame.accepted) continue;
      const auto& out=frame.message;
      mavlink_message_t m{};
      if(out.msgid==MAVLINK_MSG_ID_PARAM_REQUEST_READ) {
        mavlink_param_request_read_t r{};
        mavlink_msg_param_request_read_decode(&out,&r);
        const std::string name(r.param_id,std::find(r.param_id,r.param_id+16,'\0'));
        const auto float_type=name=="WP_SPEED"||name=="WP_RADIUS"||name=="AUTO_KICKSTART";
        const auto value=name=="AUTO_TRIGGER_PIN"?-1.f:name=="WP_SPEED"?1.6f:
          name=="WP_RADIUS" || (name=="MIS_DONE_BEHAVE" && type==MAV_TYPE_SURFACE_BOAT)?1.f:0.f;
        mavlink_msg_param_value_pack(1,1,&m,r.param_id,value,
          float_type?MAV_PARAM_TYPE_REAL32:MAV_PARAM_TYPE_INT8,6,UINT16_MAX);
      } else if(out.msgid==MAVLINK_MSG_ID_MISSION_COUNT||out.msgid==MAVLINK_MSG_ID_MISSION_ITEM_INT) {
        unsigned next=0;
        if(out.msgid==MAVLINK_MSG_ID_MISSION_ITEM_INT) {
          mavlink_mission_item_int_t item{};
          mavlink_msg_mission_item_int_decode(&out,&item);
          next=item.seq+1;
        }
        if(next<3) mavlink_msg_mission_request_int_pack(1,1,&m,245,191,next,MAV_MISSION_TYPE_MISSION);
        else {
          have_plan=true;
          mavlink_msg_mission_ack_pack(1,1,&m,245,191,MAV_MISSION_ACCEPTED,MAV_MISSION_TYPE_MISSION,0);
        }
      } else if(out.msgid==MAVLINK_MSG_ID_MISSION_REQUEST_LIST) {
        mavlink_msg_mission_count_pack(1,1,&m,245,191,3,MAV_MISSION_TYPE_MISSION,0);
      } else if(out.msgid==MAVLINK_MSG_ID_MISSION_REQUEST_INT) {
        mavlink_mission_request_int_t r{};
        mavlink_msg_mission_request_int_decode(&out,&r);
        const auto p=point(r.seq);
        mavlink_msg_mission_item_int_pack(1,1,&m,245,191,p.seq,p.frame,p.command,p.current,p.autocontinue,
          0,0,0,0,p.x,p.y,p.z,MAV_MISSION_TYPE_MISSION);
      } else continue;
      now+=.001;host.message(m,now);
    }
  }
  void step(unsigned n=1)
  { while(n--) { now+=.08;samples();host.tick(now);answer(); } }
  void post(const std::string& key,const std::string& text,double age=0)
  { host.mail({key,text,"operator",0,now-age,now,false}); }
  void ack(uint16_t id,uint8_t sys=1,uint8_t component=1,uint8_t target=245,uint8_t result=MAV_RESULT_ACCEPTED)
  {
    now+=.001;
    mavlink_message_t m{};
    mavlink_msg_command_ack_pack(sys,component,&m,id,result,0,0,target,191);
    host.message(m,now);
  }
  unsigned sent(unsigned id,bool accepted=true) const
  {
    return std::count_if(frames.begin(),frames.end(),[=](const auto& frame) {
      return frame.accepted==accepted && command(frame.message)==id;
    });
  }
  std::string lastStatus(const std::string& id,const std::string& variable=result_key) const
  {
    for(auto it=publications.rbegin();it!=publications.rend();++it) {
      if(it->key!=variable) continue;
      auto row=fields(it->text);
      if(row["id"]==id) return row["status"];
    }
    return {};
  }
  void ready()
  {
    step(40);
    std::string upload="id=plan,action=upload,timeout=20,items=";
    for(unsigned i=0;i<3;++i) {
      const auto p=point(i);
      if(i) upload+='|';
      upload+=std::to_string(i)+":16:"+std::to_string(p.frame)+":0:1:0:0:0:0:"+
        std::to_string(p.x)+":"+std::to_string(p.y)+":"+std::to_string(p.z);
    }
    auto invalid=upload;
    invalid.replace(invalid.find("id=plan"),7,"id=outside");
    const auto terminal_lat=std::to_string(point(2).x);
    invalid.replace(invalid.rfind(terminal_lat),terminal_lat.size(),std::to_string(point(2).x+100000));
    post("MAVMOOS_MISSION_REQUEST",invalid);
    step();
    require(lastStatus("outside","MAVMOOS_MISSION_RESULT")=="REJECTED" &&
            std::none_of(frames.begin(),frames.end(),[](const auto& frame) {
              return frame.message.msgid==MAVLINK_MSG_ID_MISSION_COUNT;
            }),"out-of-bounds plan emitted a mission mutation");
    post("MAVMOOS_MISSION_REQUEST",upload);
    for(unsigned i=0;i<80&&!host.storedMissionProof().valid;++i) step();
    require(host.storedMissionProof().valid,"native upload/readback proof was not produced");
    armed=true;step(20);
  }
  void requestStart(const std::string& id,double age=0)
  {
    const auto proof=host.storedMissionProof();
    post(request_key,"schema=1,id="+id+",action=start,timeout=90,verification_id=plan,content_id="+
      proof.content_id+",proof_revision="+std::to_string(proof.revision),age);
    step();
  }
  void start()
  {
    requestStart("run");
    require(sent(MAV_CMD_DO_SET_MISSION_CURRENT)==1,"start did not select first waypoint");
    seq=1;ack(MAV_CMD_DO_SET_MISSION_CURRENT);step(18);
    require(sent(MAV_CMD_MISSION_START)==1,"verified selected cursor did not start");
    ack(MAV_CMD_MISSION_START);
    mode=rover::auto_mode;mission_mode=1;mission_state=MISSION_STATE_ACTIVE;step(3);
    require(lastStatus("run")=="ACTIVE"&&host.missionExecutionGranted("run"),"no observed native activation");
  }
  void transition(const std::string& id,const std::string& action)
  {post(request_key,"schema=1,id="+id+",action="+action+",target=run,timeout=14");step();}
  void handback(const std::string& id,const std::string& session)
  {
    now+=.01;
    post(recovery_key,"schema=1,id="+id+",action=handback,session="+session);
  }
  unsigned stopMode() const
  {return rover::handbackMode(type==MAV_TYPE_SURFACE_BOAT);}
};

void fullWorkflow(const char* provider,const char* missions,bool boat)
{
  Directory dir;
  Rig r(provider,missions,dir,boat);
  r.ready();
  r.requestStart("stale",3);
  require(r.lastStatus("stale")=="REJECTED" && !dir.marked(),"stale request created ownership");
  r.start();
  require(dir.marked(),"accepted start lacks durable marker");
  r.transition("pause","pause");
  r.mode=r.stopMode();r.mission_mode=0;r.mission_state=MISSION_STATE_NOT_STARTED;r.speed=70;r.step(17);
  require(r.lastStatus("pause")!="COMPLETED" && dir.marked(),"mode/cursor falsely confirmed a moving pause");
  r.speed=0;r.step(16);
  require(r.lastStatus("pause")=="COMPLETED" && r.host.missionExecutionGranted("run"),
          "observed stopped pause lost retained lease");
  r.post("MAVMOOS_NAV_CONTROL","acquire");r.step();
  require(!r.host.navigationGranted(),"paused mission admitted helm");
  r.transition("resume","resume");
  // A delayed denial for the pause has the same command number as resume.
  r.ack(MAV_CMD_DO_SET_MODE,1,1,245,MAV_RESULT_DENIED);
  r.step(2);
  require(r.lastStatus("resume")=="ACK_OBSERVED"&&dir.marked(),
          "delayed denial decided a newer resume");
  r.mode=rover::auto_mode;r.mission_mode=1;r.mission_state=MISSION_STATE_ACTIVE;r.step(4);
  require(r.lastStatus("resume")=="COMPLETED" && r.host.missionExecutionGranted("run"),"same cursor did not resume");
  r.seq=2;r.lat=Rig::point(2).x;r.step(2);
  mavlink_message_t reached{};
  mavlink_msg_mission_item_reached_pack(1,1,&reached,2);
  r.host.message(reached,r.now);r.mission_state=MISSION_STATE_COMPLETE;r.step(20);
  require(r.lastStatus("run")=="COMPLETED" && dir.marked(),"completed plan released the uncertainty marker");
  r.transition("release","release");
  r.ack(MAV_CMD_DO_SET_MODE);
  r.mode=r.stopMode();r.mission_mode=0;r.step(18);
  require(r.lastStatus("release")=="COMPLETED" && !dir.marked() && !r.host.navigationGranted(),
          "explicit observed handback did not clear marker");
}

void adversarial(const char* provider,const char* missions)
{
  Directory dir;
  Rig r(provider,missions,dir);
  r.ready();
  r.requestStart("run");
  require(r.sent(MAV_CMD_DO_SET_MISSION_CURRENT)==1,"select phase missing");
  r.seq=1;r.send_progress=false;
  r.ack(MAV_CMD_DO_SET_MISSION_CURRENT,1,2);
  r.ack(MAV_CMD_DO_SET_MISSION_CURRENT,1,1,254);
  r.ack(MAV_CMD_DO_SET_MISSION_CURRENT);
  r.progress(2,1);r.progress(1,1,3);
  r.step(20);
  require(r.sent(MAV_CMD_MISSION_START)==0,"foreign or stale progress authorized AUTO");
  r.send_progress=true;
  r.before_handoff=[&](const auto& packet) {
    if(command(packet)==MAV_CMD_MISSION_START) {r.mode=0;r.now+=.001;r.heartbeat();}
  };
  r.step(2);
  require(r.sent(MAV_CMD_MISSION_START)==0 && r.sent(MAV_CMD_MISSION_START,false)==1 && dir.marked(),
          "fresh Manual at final handoff did not fence start");
  r.transition("manual-abort","abort");
  require(r.lastStatus("manual-abort")=="REJECTED","abort moved an operator's Manual Rover");
  r.before_handoff={};
  r.mode=rover::auto_mode;r.mission_mode=1;r.mission_state=MISSION_STATE_ACTIVE;r.step(3);
  r.transition("unowned-auto-abort","abort");
  require(r.lastStatus("unowned-auto-abort")=="REJECTED" && r.sent(MAV_CMD_DO_SET_MODE)==0 && dir.marked(),
          "pilot AUTO acquired stop authority after the original start never sent");
  r.mission_mode=0;r.mission_state=MISSION_STATE_NOT_STARTED;
  r.mode=rover::hold_mode;r.step(17);
  r.transition("release","release");
  r.ack(MAV_CMD_DO_SET_MODE);
  r.step(20);
  require(r.lastStatus("release")=="COMPLETED" && !dir.marked(),"explicit handback did not resolve retained stop");
}

void restarted(const char* provider,const char* missions)
{
  Directory dir;
  {
    Rig first(provider,missions,dir);
    first.ready();first.start();
    require(dir.marked(),"restart fixture did not create marker");
  }
  Rig next(provider,missions,dir,false,"two");
  next.armed=true;next.mode=rover::auto_mode;next.step(23);
  next.handback("old-session","one");
  require(next.lastStatus("old-session","MAVMOOS_AUTO_RECOVERY_RESULT")=="REJECTED",
          "old process session was allowed to clear marker");
  next.mode=0;next.step(17);next.handback("manual","two");
  require(next.lastStatus("manual","MAVMOOS_AUTO_RECOVERY_RESULT")=="REJECTED"&&dir.marked(),
          "Manual alone released remote uncertainty");
  next.mode=rover::hold_mode;next.step(18);next.handback("hold","two");
  require(next.lastStatus("hold","MAVMOOS_AUTO_RECOVERY_RESULT")=="RELEASED" && !dir.marked() &&
          !next.host.navigationGranted() && !next.host.storedMissionProof().valid,
          "fresh new-session handback retained authority or proof");
}

void stoppedHandback(Rig& r,const Directory& dir)
{
  r.mode=rover::hold_mode;
  r.mission_mode=0;r.mission_state=MISSION_STATE_NOT_STARTED;
  r.step(18);
  r.transition("native-release","release");
  r.ack(MAV_CMD_DO_SET_MODE);
  r.mode=r.stopMode();r.step(18);
  require(r.lastStatus("native-release")=="COMPLETED" && !dir.marked() &&
          !r.host.navigationGranted(),"fresh explicit native handback failed after old AUTO was fenced");
}

void pilotReenteredAuto(const char* provider,const char* missions,bool boat)
{
  Directory dir;
  Rig r(provider,missions,dir,boat);
  r.ready();r.start();
  r.mode=0;r.step();
  require(r.lastStatus("run")=="LOST" && dir.marked(),"pilot Manual did not fence old execution");
  r.mode=rover::auto_mode;r.step(3);
  const auto before=r.sent(MAV_CMD_DO_SET_MODE);
  r.transition("pilot-reentry-abort","abort");
  require(r.lastStatus("pilot-reentry-abort")=="REJECTED" &&
          r.sent(MAV_CMD_DO_SET_MODE)==before && dir.marked(),
          "pilot AUTO reentry revived the retained execution's wire stop");
  stoppedHandback(r,dir);
}

void queuedStopCrossedPilotExit(const char* provider,const char* missions,bool boat)
{
  Directory dir;
  Rig r(provider,missions,dir,boat);
  r.ready();r.start();
  bool intercepted=false;
  r.before_handoff=[&](const mavlink_message_t& packet) {
    if(command(packet)!=MAV_CMD_DO_SET_MODE)return;
    intercepted=true;
    r.now+=.001;r.mode=r.stopMode();r.heartbeat();
    r.now+=.001;r.mode=rover::auto_mode;r.heartbeat();
  };
  r.transition("queued-abort","abort");
  require(intercepted && r.sent(MAV_CMD_DO_SET_MODE)==0 && r.sent(MAV_CMD_DO_SET_MODE,false)==1 &&
          dir.marked(),"queued stop survived a pilot mode round trip at final handoff");
  r.before_handoff={};
  r.step(180);
  r.transition("later-abort","abort");
  require(r.lastStatus("later-abort")=="REJECTED" && r.sent(MAV_CMD_DO_SET_MODE)==0 && dir.marked(),
          "new deadline revived old wire stop after its queued predecessor was fenced");
  stoppedHandback(r,dir);
}

void pauseWithoutOwnedResume(const char* provider,const char* missions,bool boat)
{
  Directory dir;
  Rig r(provider,missions,dir,boat);
  r.ready();r.start();
  r.transition("pause","pause");
  r.ack(MAV_CMD_DO_SET_MODE);
  r.mode=r.stopMode();r.mission_mode=0;r.mission_state=MISSION_STATE_NOT_STARTED;
  r.speed=70;r.step();
  require(r.lastStatus("pause")!="COMPLETED","pause became settled without stopped dwell");
  r.mode=rover::auto_mode;r.mission_mode=1;r.mission_state=MISSION_STATE_ACTIVE;
  r.speed=0;r.step();
  const auto before=r.sent(MAV_CMD_DO_SET_MODE);
  r.transition("unrequested-resume-abort","abort");
  require(r.lastStatus("unrequested-resume-abort")=="REJECTED" &&
          r.sent(MAV_CMD_DO_SET_MODE)==before && dir.marked(),
          "AUTO reentry without an accepted resume inherited the old owner's stop");
}

void disconnectedAutoReentry(const char* provider,const char* missions,bool boat)
{
  Directory dir;
  Rig r(provider,missions,dir,boat);
  r.ready();r.start();
  r.now+=2.1;r.host.tick(r.now);
  require(!r.host.storedMissionProof().valid && dir.marked(),"link loss failed to fence old session");
  r.step(3);
  r.transition("reconnected-abort","abort");
  require(r.lastStatus("reconnected-abort")=="REJECTED" &&
          r.sent(MAV_CMD_DO_SET_MODE)==0 && dir.marked(),
          "fresh AUTO in a new connection revived old-session wire stop");
  stoppedHandback(r,dir);
}

void queuedBoatHandbackCrossedPilotExit(const char* provider,const char* missions)
{
  Directory dir;
  Rig r(provider,missions,dir,true);
  r.ready();r.start();
  r.mode=rover::hold_mode;r.mission_mode=0;r.mission_state=MISSION_STATE_NOT_STARTED;r.step(18);
  bool intercepted=false;
  r.before_handoff=[&](const mavlink_message_t& packet) {
    if(command(packet)!=MAV_CMD_DO_SET_MODE)return;
    intercepted=true;
    r.now+=.001;r.mode=rover::loiter_mode;r.heartbeat();
    r.now+=.001;r.mode=rover::hold_mode;r.heartbeat();
  };
  r.transition("queued-native-release","release");
  require(intercepted && r.sent(MAV_CMD_DO_SET_MODE)==0 && r.sent(MAV_CMD_DO_SET_MODE,false)==1 &&
          dir.marked(),"queued native boat handback survived a pilot mode round trip");
  r.before_handoff={};
  r.step(180);stoppedHandback(r,dir);
}
}

int main(int argc,char** argv)
{
  try {
    require(argc==3,"Rover and missions module paths required");
    fullWorkflow(argv[1],argv[2],false);
    fullWorkflow(argv[1],argv[2],true);
    adversarial(argv[1],argv[2]);
    restarted(argv[1],argv[2]);
    for(bool boat:{false,true}) {
      pilotReenteredAuto(argv[1],argv[2],boat);
      queuedStopCrossedPilotExit(argv[1],argv[2],boat);
      pauseWithoutOwnedResume(argv[1],argv[2],boat);
      disconnectedAutoReentry(argv[1],argv[2],boat);
    }
    queuedBoatHandbackCrossedPilotExit(argv[1],argv[2]);
    std::cout<<"Rover native mission loaded Runtime 13/13 passed\n";
  } catch(const std::exception& error) {
    std::cerr<<error.what()<<'\n';return 1;
  }
}
