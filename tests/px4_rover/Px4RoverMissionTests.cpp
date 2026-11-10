#include "Px4RoverMission.h"
#include "Px4RoverBackend.h"
#include "Px4Offboard.h"
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace mavmoos::runtime;

namespace {
constexpr char control[]="MAVMOOS_MISSION_CONTROL";
constexpr char result[]="MAVMOOS_MISSION_EXECUTION_RESULT";
constexpr uint32_t plan_id=1962107623;
constexpr int32_t home_lat=473977420, home_lon=85455940;

void require(bool check,const std::string& reason)
{
  if(!check) throw std::runtime_error(reason);
}

struct Host final : Services, Px4RoverMissionBinding {
  double clock=10;
  Vehicle state;
  StoredMissionProof proof;
  std::string lease,stop;
  bool granted=false, handed=false;
  std::vector<CommandRequest> commands;
  std::vector<ParameterRequest> parameters;
  std::vector<std::string> results;

  double now() const override { return clock; }
  const Vehicle& vehicle() const override { return state; }
  void publish(const std::string& key,const std::string& value) override
  { if(key==result) results.push_back(value); }
  void publish(const std::string&,double) override {}
  bool sendCommand(const CommandRequest& command) override
  { commands.push_back(command); return true; }
  bool acquireNavigation(const std::string&) override { return false; }
  void revokeNavigation(const std::string&) override {}
  bool navigation(const NavigationTarget&) override { return false; }
  bool parameter(const ParameterRequest& request) override
  { parameters.push_back(request); return true; }
  StoredMissionProof storedMissionProof() const override { return proof; }
  bool recordPx4MissionProof(const std::string&,const std::vector<MissionItem>&,uint32_t) override
  { return false; }
  uint32_t verifiedPx4MissionId() const override { return proof.valid?plan_id:0; }
  bool beginMissionExecution(const std::string& id,uint64_t,const std::string&,double) override
  {
    if(!lease.empty()) return false;
    lease=id;
    granted=true;
    return true;
  }
  bool missionExecutionGranted(const std::string& id) const override
  { return granted && lease==id; }
  bool beginMissionExecutionAbort(const std::string& id,const std::string& target,double) override
  {
    if(lease!=target) return false;
    granted=false;
    stop=id;
    return true;
  }
  bool finishMissionExecution(const std::string& id,const std::string& disposition) override
  {
    if(lease!=id) return false;
    if(disposition=="RELEASED" || disposition=="REJECTED") {
      if((disposition=="RELEASED" && stop.empty()) || (disposition=="REJECTED" && handed)) return false;
      lease.clear();
      stop.clear();
      handed=false;
    }
    granted=false;
    return true;
  }
  bool hasResult(const std::string& id,const std::string& status) const
  {
    for(const auto& item:results)
      if(item.find(",id="+id+",")!=std::string::npos &&
         item.find(",status="+status+",")!=std::string::npos) return true;
    return false;
  }
};

struct Fixture {
  Host host;
  Px4RoverBackend backend;
  std::unique_ptr<Plugin> mission=makePx4RoverMission(host,backend);
  mavlink_status_t codec{};

  Fixture()
  {
    auto& v=host.state;
    v.heartbeat_valid=v.position_valid=v.estimator_horizontal_valid=v.home_valid=true;
    v.armed=true;
    v.type=MAV_TYPE_GROUND_ROVER;
    v.autopilot=MAV_AUTOPILOT_PX4;
    v.home_latitude=v.latitude=home_lat*1e-7;
    v.home_longitude=v.longitude=home_lon*1e-7;
    v.heading=0;
    tick(0,px4::hold,px4::hold,0);
    host.proof.valid=true;
    host.proof.revision=3;
    host.proof.session_epoch=7;
    host.proof.verification_id="upload1";
    host.proof.content_id="mission/7/3";
    host.proof.verified_at=host.clock;
    for(unsigned seq=0;seq<2;++seq) {
      MissionItem item;
      item.seq=seq;
      item.frame=MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
      item.x=home_lat+(seq?2880:1350);
      item.y=home_lon;
      host.proof.items.push_back(item);
    }
    mission->configure("control_mission_execution","true");
    config();
    for(unsigned n=0;n<9;++n) tick(.25,px4::hold,px4::hold,0);
    cursor(0,MISSION_STATE_NOT_STARTED,2);
  }

  void receive(const mavlink_message_t& message)
  {
    MessageTime time{host.clock,host.clock,"READ_ACQUISITION"};
    backend.observe(message,time,host.state,host.clock);
    mission->onMessageWithMetadata(message,time);
  }
  void tick(double delta,unsigned mode,unsigned intended,uint16_t speed,bool receive_heartbeat=true)
  {
    host.clock+=delta;
    auto& v=host.state;
    v.position_at=v.home_at=v.estimator_at=host.clock;
    if(receive_heartbeat) {
      v.heartbeat_at=host.clock;
      v.mode=mode;
    }
    mavlink_current_mode_t active{};
    active.custom_mode=mode;
    active.intended_custom_mode=intended;
    mavlink_message_t message{};
    mavlink_msg_current_mode_encode_status(1,1,&codec,&message,&active);
    receive(message);
    mavlink_gps_raw_int_t raw{};
    raw.time_usec=static_cast<uint64_t>(std::llround(host.clock*1e6));
    raw.fix_type=GPS_FIX_TYPE_3D_FIX;
    raw.satellites_visible=12;
    raw.lat=static_cast<int32_t>(std::llround(v.latitude*1e7));
    raw.lon=static_cast<int32_t>(std::llround(v.longitude*1e7));
    raw.vel=speed;
    raw.h_acc=500;
    raw.vel_acc=150;
    mavlink_msg_gps_raw_int_encode_status(1,1,&codec,&message,&raw);
    receive(message);
  }
  void config()
  {
    const std::array<int32_t,3> expected={10045,1,5};
    for(unsigned i=0;i<4;++i) {
      backend.readConfiguration(host);
      require(host.parameters.size()==i+1,"parameter read did not queue");
      const auto request=host.parameters.back();
      backend.parameterSent(request,host.clock,true);
      tick(.25,px4::hold,px4::hold,0);
      mavlink_param_value_t reply{};
      std::memcpy(reply.param_id,request.name.data(),request.name.size());
      reply.param_count=20;
      reply.param_index=i;
      if(i<3) {
        reply.param_type=MAV_PARAM_TYPE_INT32;
        std::memcpy(&reply.param_value,&expected[i],sizeof(float));
      } else {
        reply.param_type=MAV_PARAM_TYPE_REAL32;
        reply.param_value=.7f;
      }
      mavlink_message_t message{};
      mavlink_msg_param_value_encode_status(1,1,&codec,&message,&reply);
      receive(message);
    }
    require(backend.verifiedConfig(host.clock),"typed fallback configuration did not verify");
  }
  void cursor(unsigned seq,unsigned state,unsigned mode,uint32_t id=plan_id,uint8_t sysid=1,
              double effective_at=-1)
  {
    mavlink_mission_current_t value{};
    value.seq=seq;
    value.total=2;
    value.mission_state=state;
    value.mission_mode=mode;
    value.mission_id=id;
    mavlink_message_t message{};
    mavlink_msg_mission_current_encode_status(sysid,1,&codec,&message,&value);
    if(effective_at<0) receive(message);
    else mission->onMessageWithMetadata(message,{effective_at,host.clock,"QUEUE_EMPTY_FLOOR"});
  }
  void reached(unsigned seq)
  {
    mavlink_mission_item_reached_t value{};
    value.seq=seq;
    mavlink_message_t message{};
    mavlink_msg_mission_item_reached_encode_status(1,1,&codec,&message,&value);
    receive(message);
  }
  void request(const std::string& fields)
  {
    Mail mail{control,"schema=1,"+fields,"fixture",0,host.clock-.01,host.clock,false};
    mission->onMail(mail);
    mission->iterate();
  }
  void start(const std::string& id="start")
  {
    request("id="+id+",action=start,verification_id=upload1,content_id=mission/7/3,proof_revision=3");
  }
  void handoff()
  {
    host.handed=true;
    mission->onCommandSent(host.commands.back(),host.clock,true);
  }
  void ack(uint8_t system=245,uint8_t component=191,uint8_t result=MAV_RESULT_ACCEPTED)
  {
    mavlink_command_ack_t response{};
    response.command=host.commands.back().command;
    response.result=result;
    response.target_system=system;
    response.target_component=component;
    mavlink_message_t message{};
    mavlink_msg_command_ack_encode_status(1,1,&codec,&message,&response);
    receive(message);
  }
};

void progressAndStop()
{
  Fixture f;
  f.start();
  require(f.host.commands.size()==1 && f.host.commands.back().command==MAV_CMD_MISSION_START &&
    f.host.commands.back().params[0]==0 && f.host.commands.back().params[1]==1,
    "start did not address the exact bounded plan");
  const auto handoff_at=f.host.clock;
  f.handoff();
  f.tick(.25,px4::hold,px4::hold,0);
  f.ack();
  f.mission->iterate();
  require(!f.host.hasResult("start","ACTIVE"),"ACK alone reported mission active");
  f.tick(.25,px4::mission,px4::mission,70);
  f.cursor(0,MISSION_STATE_ACTIVE,1,plan_id,2);
  f.cursor(0,MISSION_STATE_ACTIVE,1,plan_id,1,handoff_at);
  f.mission->iterate();
  require(!f.host.hasResult("start","ACTIVE"),"foreign or pre-handoff cursor settled the start");
  f.tick(.25,px4::mission,px4::mission,70);
  f.cursor(0,MISSION_STATE_ACTIVE,1);
  f.mission->iterate();
  require(f.host.hasResult("start","ACTIVE"),"selected new mode and cursor did not settle start");

  f.request("id=pause,action=pause,target=start");
  require(f.host.commands.size()==2 && f.host.commands.back().command==MAV_CMD_DO_SET_MODE &&
    f.host.commands.back().params[2]==3,"pause did not request native Hold");
  f.handoff();
  f.tick(.25,px4::hold,px4::hold,70);
  f.ack();
  f.cursor(0,MISSION_STATE_NOT_STARTED,2);
  f.mission->iterate();
  require(!f.host.hasResult("pause","COMPLETED"),"Hold ACK and mode bypassed stopped evidence");
  for(unsigned n=0;n<8;++n) {
    f.tick(.25,px4::hold,px4::hold,0);
    f.cursor(0,MISSION_STATE_NOT_STARTED,2);
    f.mission->iterate();
  }
  require(f.host.hasResult("pause","COMPLETED"),"verified early pause rejected the native NOT_STARTED cursor");
  f.request("id=resume,action=resume,target=start");
  require(f.host.commands.size()==3 && f.host.commands.back().command==MAV_CMD_DO_SET_MODE &&
    f.host.commands.back().params[2]==4,"resume restarted the plan instead of restoring Mission mode");
  f.handoff();
  f.tick(.25,px4::hold,px4::hold,0);
  f.ack(245,191,MAV_RESULT_DENIED); // A late denial for the earlier Hold request.
  f.mission->iterate();
  require(f.host.hasResult("resume","ACK_OBSERVED") && !f.host.hasResult("resume","FAILED") &&
          !f.host.hasResult("resume","COMPLETED") && f.host.lease=="start",
          "delayed Hold denial decided a different request");
  f.tick(.25,px4::mission,px4::mission,70);
  f.cursor(0,MISSION_STATE_ACTIVE,1);
  f.mission->iterate();
  require(f.host.hasResult("resume","COMPLETED"),"resume lacked an observed active cursor");
  f.tick(.25,px4::mission,px4::mission,50);
  f.cursor(1,MISSION_STATE_ACTIVE,1);
  f.mission->iterate();
  f.host.state.latitude=f.host.proof.items.back().x*1e-7;
  f.host.granted=false; // Host fences command authority as soon as native Mission exits.
  f.tick(.25,px4::hold,px4::hold,0);
  f.reached(1);
  f.mission->iterate();
  require(!f.host.hasResult("start","COMPLETED"),"final item alone reported completion");
  for(unsigned n=0;n<9;++n) {
    f.tick(.25,px4::hold,px4::hold,0);
    f.cursor(1,MISSION_STATE_COMPLETE,2);
    f.reached(1);
    f.mission->iterate();
  }
  require(f.host.hasResult("start","COMPLETED") && f.host.lease=="start",
    "native Hold before COMPLETE or repeated final events prevented bounded settlement");
  f.request("id=release,action=release,target=start");
  require(f.host.commands.size()==3 && f.host.lease=="start",
    "release from already intended Hold sent a command or used pre-request dwell");
  for(unsigned n=0;n<8;++n) {
    f.tick(.25,px4::hold,px4::hold,0);
    f.mission->iterate();
  }
  require(f.host.hasResult("release","COMPLETED") && f.host.lease.empty(),
    "fresh stopped Hold did not release the lease");
}

void uncertainStart()
{
  Fixture f;
  f.request("id=start,action=start,verification_id=upload1,content_id=mission/7/3,proof_revision=3,timeout=6");
  f.handoff();
  for(unsigned n=0;n<24;++n) {
    f.tick(.25,px4::hold,px4::hold,0);
    f.cursor(0,MISSION_STATE_NOT_STARTED,2);
    f.ack(255,190);
    f.mission->iterate();
    if(n<23) require(!f.host.hasResult("start","TIMED_OUT"),"handoff window shortened the original observation deadline");
  }
  require(f.host.hasResult("start","TIMED_OUT") && f.host.lease=="start" && !f.host.granted &&
    !f.host.hasResult("start","ACK_OBSERVED") && f.host.commands.size()==1,
    "missing vehicle evidence retried start or cleared authority");
  f.tick(.25,px4::mission,px4::mission,50);
  f.ack();
  f.cursor(0,MISSION_STATE_ACTIVE,1);
  f.mission->iterate();
  require(!f.host.hasResult("start","ACTIVE"),"late ACK and mode renewed a retired start");
  f.start("again");
  require(f.host.hasResult("again","REJECTED") && f.host.commands.size()==1,
    "a new start bypassed unresolved ownership");
  constexpr auto manual=1u<<16;
  f.tick(.25,manual,manual,0);
  f.request("id=unsafe-stop,action=abort,target=start");
  require(f.host.hasResult("unsafe-stop","REJECTED") && f.host.commands.size()==1,
    "stop-only authority overrode external Manual mode");
  f.tick(.25,px4::mission,px4::mission,50);
  f.request("id=abort,action=abort,target=start");
  require(f.host.commands.size()==2 && f.host.commands.back().abort_id=="abort" &&
    f.host.commands.back().params[2]==3,"uncertain execution could not request bounded Hold");
  f.handoff();
  f.tick(.25,px4::mission,px4::mission,50);
  f.ack();
  f.mission->iterate();
  require(f.host.lease=="start" && !f.host.hasResult("abort","COMPLETED"),
    "stop ACK cleared retained authority without stopped Hold");
  for(unsigned n=0;n<8;++n) {
    f.tick(.25,px4::hold,px4::hold,0);
    f.mission->iterate();
  }
  require(f.host.hasResult("abort","COMPLETED") && f.host.lease.empty(),
    "explicit stopped handback did not resolve uncertain authority");
}

void opaqueMismatch()
{
  Fixture f;
  f.tick(.25,px4::hold,px4::hold,0);
  f.cursor(0,MISSION_STATE_NOT_STARTED,2,plan_id+1);
  f.start();
  require(f.host.hasResult("start","REJECTED") && f.host.commands.empty() && f.host.lease.empty(),
    "equal item count with a different FC plan ID admitted start");
}

void sharedHeartbeatHorizon()
{
  Fixture f;
  f.start();
  f.handoff();
  f.tick(.25,px4::mission,px4::mission,70);
  f.ack();
  f.cursor(0,MISSION_STATE_ACTIVE,1);
  f.mission->iterate();
  require(f.host.hasResult("start","ACTIVE"),"heartbeat test did not establish active mission");

  // CURRENT_MODE and MISSION_CURRENT continue arriving between heartbeats.
  for(double delta:{.25,.25,.25,.25,.25,.25,.20}) {
    f.tick(delta,px4::mission,px4::mission,70,false);
    f.cursor(0,MISSION_STATE_ACTIVE,1);
    f.mission->iterate();
  }
  require(f.host.granted && !f.host.hasResult("start","LOST"),
    "a 1.7 s heartbeat gap quarantined the mission with fresh mode and cursor");

  f.tick(.40,px4::mission,px4::mission,70,false);
  f.cursor(0,MISSION_STATE_ACTIVE,1);
  f.mission->iterate();
  require(!f.host.granted && f.host.hasResult("start","LOST") &&
    f.host.lease=="start" && f.host.commands.size()==1,
    "a 2.1 s heartbeat gap did not fence the mission and retain the lease");
}
} // namespace

int main()
{
  try {
    progressAndStop();
    uncertainStart();
    opaqueMismatch();
    sharedHeartbeatHorizon();
    std::cout<<"PX4 Ackermann mission evidence checks passed\n";
  } catch(const std::exception& e) {
    std::cerr<<e.what()<<'\n';
    return 1;
  }
}
