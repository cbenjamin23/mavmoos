// Real Runtime and standard module DSOs. The clock and MAVLink peer are inert.
#include "../protocol_faults/ProtocolSupport.h"
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <sys/stat.h>
#include <unistd.h>

using namespace mavmoos::runtime;
using namespace protocol_faults;
namespace {
constexpr char status_key[]="MAVMOOS_AUTO_RECOVERY_STATUS";
constexpr char result_key[]="MAVMOOS_AUTO_RECOVERY_RESULT";
constexpr char request_key[]="MAVMOOS_AUTO_RECOVERY_REQUEST";
struct Slot {
  std::filesystem::path root,directory;
  Slot()
  {
    auto name=(std::filesystem::temp_directory_path()/"mavmoos-recovery-XXXXXX").string();
    require(mkdtemp(name.data())!=nullptr,"temporary recovery slot");
    root=name;directory=root/"state";
    require(mkdir(directory.c_str(),0700)==0,"private recovery directory");
  }
  ~Slot() {std::error_code error;std::filesystem::remove_all(root,error);}
  std::filesystem::path marker()const{return directory/"auto-may-continue";}
  void mark()const{std::ofstream file(marker());require(bool(file),"seed test marker");require(chmod(marker().c_str(),0600)==0,"private test marker");}
};

struct Rig {
  double time=100,advance_on_marker=-1;
  unsigned mode=4;
  bool armed=false,airborne=false;
  std::string session;
  const Slot& slot;
  std::vector<Publication> publications;
  std::vector<Frame> frames;
  mavlink_status_t codec{};
  Runtime host;
  std::vector<MissionItem> plan={home(),waypoint()};
  std::size_t processed=0;
  bool marker_seen_on_start=false;
  Rig(const std::vector<std::string>& modules,const Slot& state,std::string id="process-one",bool configure=true)
    :session(std::move(id)),slot(state),host(
      [this](const auto& key,const auto& text,double,bool numeric) {
        if(!numeric)publications.push_back({key,text,time});
      },[](const uint8_t*,std::size_t) {throw std::runtime_error("unbounded sender called");return false;},
      true,false,[this] {
        if(advance_on_marker>=0 && std::filesystem::exists(slot.marker())) {
          time=advance_on_marker;advance_on_marker=-1;
        }
        return time;
      },MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_ARDUPILOTMEGA,true)
  {
    for(const auto& path:modules)require(host.load(path),host.lastConfigurationError());
    host.configure("control_native_actions","true");
    host.configure("control_mission_execution","true");
    host.configureModules({});
    host.setBoundedSend([this](const uint8_t* bytes,std::size_t size,double expiry,const Runtime::SendEligibility& live) {
      const auto message=decode(bytes,size);
      if(command(message)==MAV_CMD_MISSION_START)marker_seen_on_start=std::filesystem::exists(slot.marker());
      const bool accepted=live(time);frames.push_back({message,time,expiry,accepted});return accepted;
    });
    if(configure)finish();
  }
  void finish()
  {
    host.configureAutoRecovery(slot.directory.string(),session);
    host.finishStartup();
  }
  void fresh(uint8_t source=1)
  {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack_status(source,1,&codec,&message,2,3,armed?128:0,mode,4);
    host.message(message,time);
    mavlink_msg_global_position_int_pack_status(1,1,&codec,&message,1000,423584560,-710875890,
      airborne?15000:10000,airborne?5000:0,0,0,0,9000);host.message(message,time);
    mavlink_msg_extended_sys_state_pack_status(1,1,&codec,&message,0,airborne?2:1);host.message(message,time);
    mavlink_home_position_t h{};h.latitude=423584560;h.longitude=-710875890;h.altitude=10000;
    mavlink_msg_home_position_encode_status(1,1,&codec,&message,&h);host.message(message,time);
    mavlink_ekf_status_report_t e{};e.flags=51;mavlink_msg_ekf_status_report_encode_status(1,1,&codec,&message,&e);host.message(message,time);
  }
  std::map<std::string,std::string> last(const std::string& key)const
  {for(auto p=publications.rbegin();p!=publications.rend();++p)if(p->key==key)return fields(p->text);return {};}
  void mail(const std::string& key,const std::string& text,double published)
  {Mail m;m.key=key;m.text=text;m.source="operator_fixture";m.published_at=published;m.received_at=time;host.mail(m);}
  void handback(const std::string& process,double published)
  {mail(request_key,"schema=1,id=handback,action=handback,session="+process,published);}
  void step(bool update=true)
  {time+=.02;if(update)fresh();host.tick(time);answerMissions();}
  void answerMissions()
  {
    while(processed<frames.size()) {
      const auto frame=frames[processed++];
      if(!frame.accepted)continue;
      const auto& out=frame.message;
      mavlink_message_t response{};
      if(out.msgid==MAVLINK_MSG_ID_MISSION_COUNT || out.msgid==MAVLINK_MSG_ID_MISSION_ITEM_INT) {
        unsigned next=0;
        if(out.msgid==MAVLINK_MSG_ID_MISSION_ITEM_INT) {
          mavlink_mission_item_int_t item{};mavlink_msg_mission_item_int_decode(&out,&item);next=item.seq+1;
        }
        if(next<plan.size()) {
          mavlink_mission_request_int_t r{};r.target_system=245;r.target_component=191;r.seq=next;
          mavlink_msg_mission_request_int_encode_status(1,1,&codec,&response,&r);
        } else {
          mavlink_mission_ack_t r{};r.target_system=245;r.target_component=191;r.type=MAV_MISSION_ACCEPTED;
          mavlink_msg_mission_ack_encode_status(1,1,&codec,&response,&r);
        }
      } else if(out.msgid==MAVLINK_MSG_ID_MISSION_REQUEST_LIST) {
        mavlink_mission_count_t r{};r.target_system=245;r.target_component=191;r.count=plan.size();
        mavlink_msg_mission_count_encode_status(1,1,&codec,&response,&r);
      } else if(out.msgid==MAVLINK_MSG_ID_MISSION_REQUEST_INT) {
        mavlink_mission_request_int_t request{};mavlink_msg_mission_request_int_decode(&out,&request);
        const auto& p=plan.at(request.seq);
        mavlink_mission_item_int_t r{};r.target_system=245;r.target_component=191;r.seq=p.seq;
        r.command=p.command;r.frame=p.frame;r.autocontinue=p.autocontinue;r.x=p.x;r.y=p.y;r.z=p.z;
        mavlink_msg_mission_item_int_encode_status(1,1,&codec,&response,&r);
      } else continue;
      time+=.001;host.message(response,time);
    }
  }
  StoredMissionProof verified()
  {
    step();time+=.001;
    mail("MAVMOOS_MISSION_REQUEST","id=verified,action=upload,items="+rows(),time);
    for(unsigned i=0;i<150 && !host.storedMissionProof().valid;++i)step();
    auto proof=host.storedMissionProof();require(proof.valid,"actual mission readback did not issue proof");return proof;
  }
  bool begin(const StoredMissionProof& proof,double expires)
  {return host.beginMissionExecution("execution",proof.revision,proof.content_id,expires);}
};

void denyEffects(Rig& rig)
{
  require(rig.host.controlBusy()&&!rig.host.navigationGranted(),"uncertainty did not own the control gate");
  require(!rig.host.acquireNavigation("competing"),"competing navigation acquisition accepted");
  require(!rig.host.navigation({90,1,5,rig.time+1}),"competing navigation accepted");
  require(!rig.host.resumeNavigation("competing"),"old navigation resume accepted");
  CommandRequest command;command.id="competing";command.command=MAV_CMD_DO_SET_MODE;
  command.params[0]=1;command.params[1]=4;command.expires=rig.time+1;
  require(!rig.host.sendCommand(command),"competing mode command accepted");
  command.command=MAV_CMD_NAV_LAND;
  require(!rig.host.sendCommand(command),"normal LAND bypassed recovery");
  MissionRequest mission;mission.id="clear";mission.operation_id="clear";mission.action=MissionAction::Clear;
  mission.expires=rig.time+1;mission.operation_expires=rig.time+3;
  require(!rig.host.mission(mission),"competing mission clear accepted");
  ParameterRequest parameter;parameter.id="wind";parameter.name="SIM_WIND_SPD";parameter.action=ParameterAction::Write;
  parameter.value=1;parameter.expires=rig.time+1;
  require(!rig.host.parameter(parameter),"competing simulation parameter write accepted");
}
}

int main(int argc,char** argv)
{
  unsigned count=0;
  try {
    require(argc==4,"expected actual control, mission, parameter DSOs");
    const std::vector<std::string> modules={argv[1],argv[2],argv[3]};
    auto test=[&](const char* name,const std::function<void()>& body){body();++count;std::cout<<"PASS "<<name<<'\n';};
    test("clean slot waits for strict target observation", [&]{
      Slot slot;Rig rig(modules,slot);
      rig.fresh();rig.host.tick(rig.time);
      require(rig.last(status_key)["state"]=="WAIT_HEARTBEAT","equal startup observation opened gate");
      rig.time+=.1;rig.fresh(2);rig.host.tick(rig.time);
      require(rig.last(status_key)["state"]=="WAIT_HEARTBEAT","foreign heartbeat opened gate");
      rig.step();require(rig.last(status_key)["state"]=="READY"&&!rig.host.controlBusy(),"fresh target non-AUTO did not open clean boot");
    });
    test("one owner and no public or symlink directory", [&]{
      Slot slot;Rig first(modules,slot);bool rejected=false;
      try {Rig second(modules,slot,"second");} catch(const std::invalid_argument&) {rejected=true;}
      require(rejected,"concurrent bridge acquired same directory");
      Slot public_slot;require(chmod(public_slot.directory.c_str(),0755)==0,"public test directory");
      rejected=false;try {Rig unsafe(modules,public_slot);} catch(const std::invalid_argument&) {rejected=true;}
      require(rejected,"public recovery directory accepted");
      Slot linked;std::filesystem::remove(linked.directory);std::filesystem::create_directory_symlink(slot.directory,linked.directory);
      rejected=false;try {Rig unsafe(modules,linked);} catch(const std::invalid_argument&) {rejected=true;}
      require(rejected,"symlink recovery directory accepted");
    });
    test("first observed unowned AUTO persists uncertainty", [&]{
      Slot slot;
      {Rig first(modules,slot);first.mode=3;first.step();
       require(first.last(status_key)["state"]=="UNRESOLVED"&&std::filesystem::exists(slot.marker()),"unowned AUTO not persisted");}
      Rig restarted(modules,slot,"second");restarted.step();
      require(restarted.last(status_key)["state"]=="UNRESOLVED","fresh Guided erased durable uncertainty");
      denyEffects(restarted);
    });
    test("restart accepts read-only service and rejects early or stale handback", [&]{
      Slot slot;slot.mark();Rig rig(modules,slot,"new-session");
      rig.mode=3;rig.step();denyEffects(rig);
      rig.time+=.001;rig.handback("new-session",rig.time);
      require(rig.last(result_key)["reason"]=="fresh_prior_non_auto_observation_required","AUTO handback accepted");
      ParameterRequest read;read.id="read";read.name="FRAME_CLASS";read.expires=rig.time+1;
      require(rig.host.parameter(read),"read-only parameter request blocked");
      MissionRequest mission;mission.id="read-list";mission.operation_id="read";mission.expires=rig.time+1;mission.operation_expires=rig.time+4;
      require(rig.host.mission(mission),"read-only mission request blocked");
      for(int n=0;n<12;++n)rig.step();
      bool param=false,plan=false;
      for(const auto& frame:rig.frames)if(frame.accepted) {
        param|=frame.message.msgid==MAVLINK_MSG_ID_PARAM_REQUEST_READ;
        plan|=frame.message.msgid==MAVLINK_MSG_ID_MISSION_REQUEST_LIST;
        require(frame.message.msgid==MAVLINK_MSG_ID_HEARTBEAT || frame.message.msgid==MAVLINK_MSG_ID_PARAM_REQUEST_READ ||
          frame.message.msgid==MAVLINK_MSG_ID_MISSION_REQUEST_LIST,"effect/stream setup crossed unresolved recovery");
      }
      require(param&&plan,"read-only wire evidence absent");
      rig.mode=4;rig.step();const double observed=rig.time;rig.time+=.001;
      rig.handback("old-session",rig.time);require(rig.last(result_key)["reason"]=="wrong_process_session","old selector accepted");
      rig.handback("new-session",rig.time+1);require(rig.last(result_key)["reason"]=="stale_or_future_request","future publication accepted");
      rig.handback("new-session",observed);require(rig.last(result_key)["reason"]=="fresh_prior_non_auto_observation_required","equal evidence/request time accepted");
      rig.time+=2.1;rig.handback("new-session",rig.time);require(rig.last(result_key)["status"]=="REJECTED","stale non-AUTO released");
      require(std::filesystem::exists(slot.marker()),"rejected handback removed marker");
    });
    test("explicit fresh handback clears marker and requires fresh acquisition", [&]{
      Slot slot;slot.mark();Rig rig(modules,slot,"new-session");
      rig.mode=4;rig.armed=rig.airborne=true;rig.step();denyEffects(rig);
      rig.time+=.001;rig.handback(rig.session,rig.time);
      require(rig.last(result_key)["status"]=="RELEASED"&&!std::filesystem::exists(slot.marker()),"explicit handback not durable");
      require(!rig.host.navigationGranted()&&!rig.host.storedMissionProof().valid&&!rig.host.vehicle().heartbeat_valid,"handback revived old grant/proof/observations");
      require(!rig.host.acquireNavigation("without-new-evidence"),"old evidence acquired after handback");
      rig.step();require(rig.host.acquireNavigation("new-explicit"),"fresh deliberate acquisition denied");
      rig.mode=3;rig.step();require(rig.last(status_key)["state"]=="UNRESOLVED"&&std::filesystem::exists(slot.marker()),"later unowned AUTO did not refence");
      denyEffects(rig);
    });
    test("durable marker precedes AUTO and only safe in-process release clears", [&]{
      Slot slot;Rig rig(modules,slot);
      const auto proof=rig.verified();rig.armed=rig.airborne=true;rig.step();
      require(rig.begin(proof,rig.time+10)&&std::filesystem::exists(slot.marker()),"lease admitted without marker");
      require(rig.host.finishMissionExecution("execution","REJECTED")&&!std::filesystem::exists(slot.marker()),"provably unsent safe release did not clear");
      require(rig.begin(proof,rig.time+10),"second execution admission");
      CommandRequest start;start.id="first-auto";start.command=MAV_CMD_MISSION_START;
      start.execution_id="execution";start.expires=rig.time+1;start.operation_expires=rig.time+9;
      require(rig.host.sendCommand(start),"start command not queued");rig.step();
      require(rig.marker_seen_on_start,"marker missing at final AUTO wire handoff");
      require(!rig.host.finishMissionExecution("execution","REJECTED")&&std::filesystem::exists(slot.marker()),"sent AUTO lost uncertainty");
    });
    test("Copter pilot takeover retains the marker until an explicit no-send release", [&]{
      Slot slot;Rig rig(modules,slot);
      const auto proof=rig.verified();rig.armed=rig.airborne=true;rig.step();
      require(rig.begin(proof,rig.time+10),"execution was not admitted");
      CommandRequest start;
      start.id="owned-start";start.command=MAV_CMD_MISSION_START;start.execution_id="execution";
      start.expires=rig.time+1;start.operation_expires=rig.time+8;
      require(rig.host.sendCommand(start),"AUTO start was not queued");rig.step();
      rig.mode=3;rig.step();
      require(rig.host.missionExecutionGranted("execution")&&std::filesystem::exists(slot.marker()),
              "fresh owned AUTO did not retain its marker");
      require(rig.host.beginMissionExecutionAbort("stop","execution",rig.time+3),"owned AUTO abort was not admitted");
      CommandRequest stop;
      stop.id="queued-stop";stop.command=MAV_CMD_DO_SET_MODE;stop.execution_id="execution";stop.abort_id="stop";
      stop.params[0]=1;stop.params[1]=4;stop.expires=rig.time+1;stop.operation_expires=rig.time+3;
      require(rig.host.sendCommand(stop),"owned AUTO abort was not queued");
      rig.mode=0;rig.step();
      rig.mode=3;rig.step();stop.id="unowned-reentry";
      require(!rig.host.sendCommand(stop)&&std::filesystem::exists(slot.marker()),
              "unowned AUTO re-entry revived a stop or removed the marker");
      rig.time+=3;rig.mode=0;rig.step();
      require(rig.host.beginMissionExecutionAbort("release","execution",rig.time+3)&&
              rig.host.finishMissionExecution("execution","RELEASED")&&
              !std::filesystem::exists(slot.marker())&&!rig.host.navigationGranted()&&
              rig.host.vehicle().mode==0&&std::none_of(rig.frames.begin(),rig.frames.end(),[](const auto& frame) {
                return frame.accepted&&command(frame.message)==MAV_CMD_DO_SET_MODE;
              }),"no-send release changed pilot mode, sent a mode command or retained the marker");
    });
    test("crash window before first send and after original expiry is fenced", [&]{
      Slot slot;
      {Rig rig(modules,slot);const auto proof=rig.verified();rig.armed=rig.airborne=true;rig.step();
       require(rig.begin(proof,rig.time+1),"pending lease fixture rejected");}
      Rig restarted(modules,slot,"new-session");restarted.time+=15;restarted.step();
      require(restarted.last(status_key)["state"]=="UNRESOLVED"&&!restarted.host.storedMissionProof().valid,"pending/expired lease replayed or released");
      denyEffects(restarted);
    });
    test("marker storage cannot extend the original deadline", [&]{
      Slot slot;Rig rig(modules,slot);
      const auto proof=rig.verified();rig.armed=rig.airborne=true;rig.step();
      rig.advance_on_marker=rig.time+.2;
      require(!rig.begin(proof,rig.time+.1)&&std::filesystem::exists(slot.marker()),"late marker work admitted or discarded uncertainty");
      require(!rig.host.missionExecutionGranted("execution")&&rig.host.controlBusy(),"expired admission restored effects");
    });
    test("replaced directory fails closed during admission", [&]{
      Slot slot;Rig rig(modules,slot);
      const auto proof=rig.verified();rig.armed=rig.airborne=true;rig.step();
      std::filesystem::rename(slot.directory,slot.root/"moved");
      require(mkdir(slot.directory.c_str(),0700)==0,"replacement test directory");
      require(!rig.begin(proof,rig.time+1)&&rig.last(status_key)["state"]=="STORAGE_FAULT","replaced directory accepted");
      denyEffects(rig);
    });
    std::cout<<"PASS "<<count<<" AUTO recovery groups with real Runtime and fake peer\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<"FAIL after "<<count<<" groups: "<<error.what()<<'\n';return 1;}
}
