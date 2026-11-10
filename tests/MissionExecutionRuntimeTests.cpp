// Actual Runtime + control/mission DSOs, deterministic fake MAVLink I/O only.
#include "Runtime.h"
#include "protocol_faults/ProtocolSupport.h"
#include <algorithm>
#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
using namespace mavmoos::runtime;
namespace {
void require(bool ok,const char* why) {if(!ok)throw std::runtime_error(why);}
std::string control_path,mission_path;
struct Rig {
  double time=100;
  std::vector<mavlink_message_t> wire;
  std::vector<MissionItem> plan;
  std::vector<protocol_faults::Publication> publications;
  Runtime runtime;
  explicit Rig(bool control_enabled=false):runtime([this,control_enabled](const auto& key,const auto& value,double,bool numeric) {
    if(control_enabled&&!numeric) publications.push_back({key,value,time});
  },[this](const uint8_t* p,std::size_t n){
    mavlink_message_t message{},buffer{};mavlink_status_t state{},result{};
    for(std::size_t i=0;i<n;++i)if(mavlink_frame_char_buffer(&buffer,&state,p[i],&message,&result)==MAVLINK_FRAMING_OK)wire.push_back(message);
    return true;
  },true,false,[this]{return time;}) {
    require(runtime.load(control_path),"control DSO");require(runtime.load(mission_path),"mission DSO");
    runtime.configure("control_native_actions",control_enabled?"true":"false");
    runtime.configure("control_helm",control_enabled?"true":"false");
    if(control_enabled) {
      runtime.configure("control_mission_execution","true");
      runtime.configure("dead_man_source","deadman_fixture");
    }
    for(unsigned i=0;i<3;++i) {MissionItem item;item.seq=uint16_t(i);item.frame=i?MAV_FRAME_GLOBAL_RELATIVE_ALT:MAV_FRAME_GLOBAL;
      item.x=423584560+int32_t(i)*100;item.y=-710875890;item.z=i?5:10;plan.push_back(item);}
    refresh(false);
  }
  void refresh(bool airborne=true,unsigned mode=4) {
    mavlink_message_t m{};mavlink_heartbeat_t hb{};hb.type=2;hb.autopilot=3;hb.base_mode=airborne?128:0;hb.custom_mode=mode;
    mavlink_msg_heartbeat_encode(1,1,&m,&hb);runtime.message(m,time);
    mavlink_global_position_int_t pos{};pos.lat=423584560;pos.lon=-710875890;pos.alt=15000;pos.relative_alt=5000;
    mavlink_msg_global_position_int_encode(1,1,&m,&pos);runtime.message(m,time);
    mavlink_extended_sys_state_t landed{};landed.landed_state=airborne?2:1;
    mavlink_msg_extended_sys_state_encode(1,1,&m,&landed);runtime.message(m,time);
    mavlink_home_position_t home{};home.latitude=423584560;home.longitude=-710875890;home.altitude=10000;
    mavlink_msg_home_position_encode(1,1,&m,&home);runtime.message(m,time);
    mavlink_ekf_status_report_t ekf{};ekf.flags=51;
    mavlink_msg_ekf_status_report_encode(1,1,&m,&ekf);runtime.message(m,time);
  }
  void mail(const std::string& value) {
    time+=.01;refresh(false);Mail m;m.key="MAVMOOS_MISSION_REQUEST";m.text=value;m.source="operator";
    m.received_at=m.published_at=time;runtime.mail(m);
  }
  void post(const std::string& key,const std::string& value,const std::string& source="operator") {
    Mail message;
    message.key=key;
    message.text=value;
    message.source=source;
    message.published_at=message.received_at=time;
    runtime.mail(message);
  }
  void helmDemand() {
    post("IVPHELM_ALLSTOP","clear","pHelmIvP");
    post("IVPHELM_STATE","DRIVE","pHelmIvP");
    for(const auto& [key,value]:std::vector<std::pair<std::string,double>>{
          {"DESIRED_HEADING",90},{"DESIRED_SPEED",1},{"DESIRED_ALTITUDE",5}}) {
      Mail message;
      message.key=key;
      message.numeric=true;
      message.value=value;
      message.source="pHelmIvP";
      message.published_at=message.received_at=time;
      runtime.mail(message);
    }
  }
  void ack(uint16_t command) {
    mavlink_message_t message{};
    mavlink_msg_command_ack_pack(1,1,&message,command,MAV_RESULT_ACCEPTED,0,0,245,191);
    runtime.message(message,time);
  }
  void progress(uint8_t state) {
    mavlink_message_t message{};
    mavlink_mission_current_t current{};
    current.seq=1;
    current.total=2;
    current.mission_state=state;
    mavlink_msg_mission_current_encode(1,1,&message,&current);
    runtime.message(message,time);
  }
  bool result(const std::string& id,const std::string& status) const {
    return std::any_of(publications.begin(),publications.end(),[&](const auto& record) {
      if(record.key!="MAVMOOS_MISSION_EXECUTION_RESULT") return false;
      const auto values=protocol_faults::fields(record.text);
      return values.at("id")==id && values.at("status")==status;
    });
  }
  std::size_t navigationFrames() const {
    return std::count_if(wire.begin(),wire.end(),[](const auto& message) {
      return message.msgid==MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED ||
             message.msgid==MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT;
    });
  }
  void checkCommand(uint16_t id,float first=0,float second=0) const {
    require(sent(id)==1,"expected exactly one AUTO command handoff");
    const auto frame=std::find_if(wire.begin(),wire.end(),[&](const auto& message) {
      return protocol_faults::command(message)==id;
    });
    mavlink_command_long_t command{};
    mavlink_msg_command_long_decode(&*frame,&command);
    require(frame->sysid==245&&frame->compid==191&&command.target_system==1&&command.target_component==1&&
            command.confirmation==0&&command.param1==first&&command.param2==second&&command.param3==0&&
            command.param4==0&&command.param5==0&&command.param6==0&&command.param7==0,
            "wrong AUTO sender, target, arguments or retry flag");
  }
  void respond(const mavlink_message_t& out,const std::vector<MissionItem>& contents) {
    mavlink_message_t reply{};bool ready=true;time+=.001;
    switch(out.msgid) {
    case MAVLINK_MSG_ID_MISSION_COUNT: {
      mavlink_mission_request_int_t r{};r.target_system=245;r.target_component=191;
      mavlink_msg_mission_request_int_encode(1,1,&reply,&r);break;
    }
    case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
      mavlink_mission_item_int_t item{};mavlink_msg_mission_item_int_decode(&out,&item);
      if(item.seq+1<contents.size()) {mavlink_mission_request_int_t r{};r.target_system=245;r.target_component=191;r.seq=item.seq+1;
        mavlink_msg_mission_request_int_encode(1,1,&reply,&r);}
      else {mavlink_mission_ack_t ack{};ack.target_system=245;ack.target_component=191;ack.type=MAV_MISSION_ACCEPTED;
        mavlink_msg_mission_ack_encode(1,1,&reply,&ack);}break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
      mavlink_mission_request_list_t request{};mavlink_msg_mission_request_list_decode(&out,&request);
      mavlink_mission_count_t count{};count.target_system=245;count.target_component=191;count.count=uint16_t(contents.size());
      count.mission_type=request.mission_type;
      mavlink_msg_mission_count_encode(1,1,&reply,&count);break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
      mavlink_mission_request_int_t r{};mavlink_msg_mission_request_int_decode(&out,&r);require(r.seq<contents.size(),"item request outside fixture");
      const auto& x=contents[r.seq];mavlink_mission_item_int_t item{};item.target_system=245;item.target_component=191;
      item.seq=x.seq;item.command=x.command;item.frame=x.frame;item.current=x.current;item.autocontinue=x.autocontinue;item.mission_type=r.mission_type;
      item.x=x.x;item.y=x.y;item.z=x.z;item.param1=x.params[0];item.param2=x.params[1];item.param3=x.params[2];item.param4=x.params[3];
      mavlink_msg_mission_item_int_encode(1,1,&reply,&item);break;
    }
    default:ready=false;
    }
    if(ready)runtime.message(reply,time);
  }
  void transfer(const std::string& request,const std::vector<MissionItem>& contents,bool airborne=false) {
    const std::size_t begin=wire.size();std::size_t cursor=begin;
    if(!airborne)mail(request);
    else {time+=.01;refresh();Mail m;m.key="MAVMOOS_MISSION_REQUEST";m.text=request;m.source="operator";m.published_at=m.received_at=time;runtime.mail(m);}
    bool finished=false;
    for(unsigned i=0;i<80&&!finished;++i) {
      time+=.01;refresh(airborne);runtime.tick(time);
      while(cursor<wire.size()) {const auto out=wire[cursor++];respond(out,contents);if(out.msgid==MAVLINK_MSG_ID_MISSION_ACK)finished=true;}
    }
    require(finished,"bounded fake transfer did not reach final ACK");
  }
  StoredMissionProof upload() {
    std::ostringstream text;text<<"id=verified,action=upload,items=";
    for(unsigned i=0;i<plan.size();++i) {const auto& x=plan[i];if(i)text<<'|';text<<i<<":16:"<<unsigned(x.frame)<<":0:1:0:0:0:0:"<<x.x<<':'<<x.y<<':'<<x.z;}
    transfer(text.str(),plan);const auto proof=runtime.storedMissionProof();require(proof.valid,"real upload readback did not certify");return proof;
  }
  void acquire(double life=10) {const auto proof=upload();time+=.01;refresh();require(runtime.beginMissionExecution("run",proof.revision,proof.content_id,time+life),"execution acquisition");}
  CommandRequest command(uint16_t type,const std::string& id="phase",const std::string& abort="") {
    CommandRequest r;r.id=id;r.command=type;r.execution_id="run";r.abort_id=abort;r.expires=time+1;r.operation_expires=time+5;
    if(type==MAV_CMD_DO_SET_MISSION_CURRENT)r.params[0]=1;
    if(type==MAV_CMD_DO_SET_MODE){r.params[0]=1;r.params[1]=4;}
    return r;
  }
  unsigned sent(uint16_t type) const {
    unsigned result=0;for(const auto& m:wire)if(m.msgid==MAVLINK_MSG_ID_COMMAND_LONG) {
      mavlink_command_long_t c{};mavlink_msg_command_long_decode(&m,&c);if(c.command==type)++result;
    }return result;
  }
};

void operatorAutoHandback(bool abort)
{
  Rig r(true);
  const auto proof=r.upload();
  r.time+=.02;
  r.refresh();
  r.post("MAVMOOS_NAV_CONTROL","acquire");
  r.time+=.02;
  r.helmDemand();
  r.runtime.tick(r.time);
  const auto moving=r.navigationFrames();
  require(moving==1&&r.runtime.navigationGranted(),"stock helm fixture did not start");

  const auto request=[&](const std::string& text) {
    r.time+=.02;
    r.post("MAVMOOS_MISSION_CONTROL","schema=1,"+text);
    r.runtime.tick(r.time);
  };
  request("id=run,action=start,verification_id="+proof.verification_id+
          ",content_id="+proof.content_id+",proof_revision="+std::to_string(proof.revision));
  r.checkCommand(MAV_CMD_DO_SET_MISSION_CURRENT,1);
  require(r.result("run","ADMITTED")&&!r.runtime.navigationGranted()&&r.navigationFrames()==moving,
          "verified AUTO start did not retire the previous helm grant");
  r.time+=.02;
  r.ack(MAV_CMD_DO_SET_MISSION_CURRENT);
  r.runtime.tick(r.time);
  require(r.sent(MAV_CMD_MISSION_START)==0,"cursor ACK replaced fresh cursor observation");
  r.time+=.02;
  r.progress(MISSION_STATE_NOT_STARTED);
  r.runtime.tick(r.time);
  r.checkCommand(MAV_CMD_MISSION_START);
  r.time+=.02;
  r.ack(MAV_CMD_MISSION_START);
  r.refresh(true,3);
  r.runtime.tick(r.time);
  require(!r.result("run","ACTIVE")&&r.runtime.controlBusy(),"ACK and AUTO heartbeat replaced active progress");
  r.time+=.02;
  r.progress(MISSION_STATE_ACTIVE);
  r.runtime.tick(r.time);
  require(r.result("run","ACTIVE")&&r.runtime.missionExecutionGranted("run"),
          "verified start and active progress did not establish AUTO");

  r.time+=.02;
  r.post("MAVMOOS_NAV_CONTROL","acquire");
  r.helmDemand();
  r.post("MAVMOOS_COMMAND","id=legacy-land,action=land");
  r.post("MAVMOOS_CONTROL_REQUEST","schema=1,id=native-guided,action=guided,strategy=native");
  r.post("MAVMOOS_CONTROL_REQUEST","schema=1,id=guided-hold,action=hold,strategy=guided");
  r.post("DEAD_MAN_POST_INTERRUPT","true","deadman_fixture");
  r.runtime.tick(r.time);
  require(r.sent(MAV_CMD_NAV_LAND)==0&&r.sent(MAV_CMD_NAV_RETURN_TO_LAUNCH)==0&&r.sent(MAV_CMD_DO_SET_MODE)==0&&
          r.navigationFrames()==moving&&!r.runtime.navigationGranted()&&r.runtime.missionExecutionGranted("run"),
          "competing operator path sent a packet or changed the AUTO grant");

  request("id=pause,action=pause,target=run");
  r.checkCommand(MAV_CMD_DO_PAUSE_CONTINUE);
  r.time+=.02;
  r.ack(MAV_CMD_DO_PAUSE_CONTINUE);
  r.runtime.tick(r.time);
  require(!r.result("pause","COMPLETED"),"pause ACK was mistaken for observed pause");
  r.time+=.02;
  r.progress(MISSION_STATE_PAUSED);
  r.runtime.tick(r.time);
  require(r.result("pause","COMPLETED")&&r.result("run","PAUSED")&&r.runtime.controlBusy(),
          "observed pause did not retain AUTO ownership");
  request("id=early-release,action=release,target=run");
  require(r.result("early-release","REJECTED")&&r.runtime.controlBusy()&&r.sent(MAV_CMD_DO_SET_MODE)==0,
          "pause allowed release while FC still reported AUTO");

  if(abort) {
    request("id=abort,action=abort,target=run");
    r.checkCommand(MAV_CMD_DO_SET_MODE,1,4);
    r.time+=.02;
    r.ack(MAV_CMD_DO_SET_MODE);
    r.runtime.tick(r.time);
    require(!r.result("abort","COMPLETED")&&r.runtime.controlBusy(),"abort ACK freed AUTO ownership");
    r.time+=.02;
    r.refresh();
    r.runtime.tick(r.time);
    require(r.result("abort","COMPLETED"),"abort did not observe fresh Guided handback");
  } else {
    r.time+=.02;
    r.refresh();
    r.runtime.tick(r.time);
    require(r.runtime.controlBusy(),"external Guided transition alone released AUTO");
    request("id=release,action=release,target=run");
    require(r.result("release","COMPLETED")&&r.sent(MAV_CMD_DO_SET_MODE)==0,
            "explicit release after fresh non-AUTO state did not retire the lease locally");
  }

  r.time+=.12;
  r.refresh();
  r.helmDemand();
  r.runtime.tick(r.time);
  require(!r.runtime.controlBusy()&&!r.runtime.navigationGranted()&&r.navigationFrames()==moving,
          "AUTO handback implicitly restored helm motion");
  r.time+=.02;
  r.post("MAVMOOS_NAV_CONTROL","acquire");
  r.runtime.tick(r.time);
  require(r.navigationFrames()==moving,"new grant reused the old helm tuple");
  r.time+=.12;
  r.refresh();
  r.helmDemand();
  r.runtime.tick(r.time);
  require(r.runtime.navigationGranted()&&r.navigationFrames()==moving+1,
          "fresh explicit grant and helm demand did not restore exactly one motion target");
}
}
int main(int argc,char** argv) {
  try {
    require(argc==3,"usage: mission_execution_runtime_tests CONTROL_DSO MISSION_DSO");control_path=argv[1];mission_path=argv[2];unsigned count=0;
    auto test=[&](const char* name,const std::function<void()>& fn){fn();++count;std::cout<<"PASS "<<name<<'\n';};
    test("MOOS AUTO start and pause block competing controls until an observed stop-only abort",[]{operatorAutoHandback(true);});
    test("MOOS AUTO release needs fresh non-AUTO state and separate explicit helm acquisition",[]{operatorAutoHandback(false);});
    test("typed proof cannot be issued by arbitrary API or MOOS publication",[]{Rig r;require(!r.runtime.recordStoredMissionProof("forged",r.plan),"forged proof");
      Mail m;m.key="MAVMOOS_STORED_MISSION_PROOF";m.text="valid=true,content_id=forged";m.received_at=m.published_at=r.time;r.runtime.mail(m);
      require(!r.runtime.storedMissionProof().valid&&!r.runtime.beginMissionExecution("x",1,"forged",r.time+5),"publication granted authority");});
    test("actual upload and accepted readback produce exact original proof",[]{Rig r;const auto p=r.upload();require(p.session_epoch>0&&p.items.size()==3&&p.verification_id=="verified"&&p.verified_at<=r.time&&!p.content_id.empty(),"proof metadata");
      require(!r.runtime.beginMissionExecution("ground",p.revision,p.content_id,r.time+5),"ground execution");r.time+=.01;r.refresh();
      require(!r.runtime.beginMissionExecution("bad",p.revision+1,p.content_id,r.time+5)&&!r.runtime.beginMissionExecution("bad",p.revision,"wrong",r.time+5),"wrong binding accepted");});
    test("ordinary native commands cannot impersonate execution effects",[]{Rig r;r.acquire();for(auto type:{MAV_CMD_MISSION_START,MAV_CMD_DO_PAUSE_CONTINUE,MAV_CMD_DO_SET_MISSION_CURRENT}) {
      auto c=r.command(type);c.execution_id.clear();require(!r.runtime.sendCommand(c),"unbound effect");}
      auto c=r.command(MAV_CMD_COMPONENT_ARM_DISARM);require(!r.runtime.sendCommand(c),"lease broadened command family");});
    test("execution persists after a phase finishes and blocks competing owners",[]{Rig r;r.acquire();auto c=r.command(MAV_CMD_MISSION_START);require(r.runtime.sendCommand(c),"start admission");r.runtime.tick(r.time);
      require(r.sent(MAV_CMD_MISSION_START)==1,"actual start send");r.runtime.commandFinished(c.id);require(r.runtime.controlBusy()&&!r.runtime.acquireNavigation("helm"),"ACK/phase completion released ownership");
      auto native=r.command(MAV_CMD_DO_SET_MODE,"competitor");native.execution_id.clear();require(!r.runtime.sendCommand(native),"native competitor admitted");
      require(r.runtime.finishMissionExecution("run","COMPLETED")&&r.runtime.controlBusy(),"local completion hid AUTO ownership");});
    test("queued preparation is cancelled before any AUTO effect",[]{Rig r;r.acquire();auto c=r.command(MAV_CMD_DO_SET_MISSION_CURRENT);require(r.runtime.sendCommand(c),"cursor admission");
      require(r.runtime.finishMissionExecution("run","REJECTED"),"pre-handoff retirement denied");r.runtime.tick(r.time);
      require(!r.runtime.controlBusy()&&r.sent(MAV_CMD_DO_SET_MISSION_CURRENT)==0,"cancelled cursor was transmitted");});
    test("original deadline quarantines without reviving on fresh data",[]{Rig r;r.acquire(.1);r.time+=.2;r.refresh();
      require(!r.runtime.missionExecutionGranted("run")&&r.runtime.controlBusy(),"expiry freed uncertain ownership");
      require(!r.runtime.beginMissionExecution("again",1,"wrong",r.time+5)&&!r.runtime.sendCommand(r.command(MAV_CMD_MISSION_START)),"expiry revived");});
    test("mode takeover remains latched after AUTO and Guided return",[]{Rig r;r.acquire();auto c=r.command(MAV_CMD_MISSION_START);require(r.runtime.sendCommand(c),"start");r.runtime.tick(r.time);r.runtime.commandFinished(c.id);
      r.time+=.01;r.refresh(true,3);require(r.runtime.missionExecutionGranted("run"),"post-start AUTO rejected");
      r.time+=.01;r.refresh(true,5);r.time+=.01;r.refresh(true,4);require(!r.runtime.missionExecutionGranted("run")&&r.runtime.controlBusy(),"takeover auto-resumed");});
    test("Copter abort yields to pilot mode at the final socket check",[]{
      Rig r;
      r.acquire();
      auto start=r.command(MAV_CMD_MISSION_START,"start");
      require(r.runtime.sendCommand(start),"start admission");
      r.runtime.tick(r.time);
      r.runtime.commandFinished(start.id);
      r.time+=.01;
      r.refresh(true,3);
      require(r.runtime.missionExecutionGranted("run") &&
              r.runtime.beginMissionExecutionAbort("stop","run",r.time+5),"owned AUTO abort admission");
      auto stop=r.command(MAV_CMD_DO_SET_MODE,"stop-wire","stop");
      require(r.runtime.sendCommand(stop),"stop queue admission");
      bool switched=false;
      r.runtime.setBoundedSend([&](const uint8_t* data,std::size_t size,double,const Runtime::SendEligibility& live) {
        const auto packet=protocol_faults::decode(data,size);
        if(protocol_faults::command(packet)==MAV_CMD_DO_SET_MODE) {
          switched=true;
          r.time+=.01;
          r.refresh(true,0);
        }
        const bool accepted=live(r.time);
        if(accepted)r.wire.push_back(packet);
        return accepted;
      });
      r.runtime.tick(r.time);
      require(switched && r.sent(MAV_CMD_DO_SET_MODE)==0 && r.runtime.vehicle().mode==0 &&
              r.runtime.controlBusy(),"Copter abort transmitted after final-handoff pilot takeover");
    });
    test("fresh explicit abort survives old expiry and needs observed handback",[]{Rig r;r.acquire(.2);
      auto start=r.command(MAV_CMD_MISSION_START,"original-start");start.expires=start.operation_expires=r.time+.15;
      require(r.runtime.sendCommand(start),"original start denied");r.runtime.tick(r.time);r.runtime.commandFinished(start.id);
      r.time+=.01;r.refresh(true,3);r.time+=.3;r.refresh(true,3);
      require(r.runtime.beginMissionExecutionAbort("stop","run",r.time+5),"fresh owned AUTO abort denied");
      auto stop=r.command(MAV_CMD_DO_SET_MODE,"stop-wire","stop");require(r.runtime.sendCommand(stop),"bound stop denied");r.runtime.tick(r.time);r.runtime.commandFinished(stop.id);
      require(r.sent(MAV_CMD_DO_SET_MODE)==1,"stop not sent");r.time+=.01;r.refresh();require(r.runtime.finishMissionExecution("run","RELEASED"),"observed handback denied");
      require(!r.runtime.controlBusy()&&r.runtime.acquireNavigation("fresh-explicit-helm"),"explicit new owner denied");});
    test("release cannot reuse pre-handoff Guided evidence",[]{Rig r;r.acquire();r.time+=.01;auto start=r.command(MAV_CMD_MISSION_START);require(r.runtime.sendCommand(start),"start");r.runtime.tick(r.time);r.runtime.commandFinished(start.id);
      require(r.runtime.beginMissionExecutionAbort("release","run",r.time+3),"release context");require(!r.runtime.finishMissionExecution("run","RELEASED")&&r.runtime.controlBusy(),"old Guided sample released pending AUTO");});
    test("reset invalidates proof but preserves unresolved remote ownership",[]{Rig r;r.acquire();const auto old=r.command(MAV_CMD_MISSION_START);r.runtime.disconnect();r.time+=.01;r.refresh();
      require(!r.runtime.storedMissionProof().valid&&r.runtime.controlBusy()&&!r.runtime.sendCommand(old),"reset erased uncertainty or reused proof");
      require(r.runtime.beginMissionExecutionAbort("new-session-release","run",r.time+2),"fresh non-AUTO release denied");
      require(r.runtime.finishMissionExecution("run","RELEASED")&&r.sent(MAV_CMD_DO_SET_MODE)==0,"release did not clear without a mode command");});
    test("mutation and restricted writes remain blocked after disarm",[]{Rig r;r.acquire();r.time+=.01;r.refresh(false);MissionRequest m;m.id="clear";m.operation_id="clear";m.action=MissionAction::Clear;m.expires=r.time+1;m.operation_expires=r.time+5;
      require(!r.runtime.mission(m),"mutation bypassed persistent owner");ParameterRequest p;p.id="wind";p.action=ParameterAction::Write;p.name="SIM_WIND_SPD";p.expires=r.time+1;
      require(!r.runtime.parameter(p),"write bypassed owner");p.action=ParameterAction::Read;require(r.runtime.parameter(p),"safe read unnecessarily blocked");});
    test("changed executable readback invalidates retained proof and authority",[]{Rig r;r.acquire();auto changed=r.plan;changed[1].x+=1000;
      r.transfer("id=observe,action=download",changed,true);require(!r.runtime.storedMissionProof().valid&&!r.runtime.missionExecutionGranted("run")&&r.runtime.controlBusy(),"changed plan did not latch");});
    test("versioned inspection preserves matching proof and invalidates a contrary flight plan only",[]{
      Rig r;
      r.transfer("schema=1,id=inspect-before-upload,action=inspect,type=flight",r.plan);
      require(!r.runtime.storedMissionProof().valid,"flight inspection minted proof");
      r.acquire();const auto prior=r.runtime.storedMissionProof();
      r.transfer("schema=1,id=inspect-flight,action=inspect,type=flight",r.plan,true);
      MissionItem fence;fence.command=MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;fence.frame=MAV_FRAME_GLOBAL;
      fence.x=423584560;fence.y=-710875890;fence.params[0]=50;fence.autocontinue=0;
      r.transfer("schema=1,id=inspect-fence,action=inspect,type=fence",{fence},true);
      r.transfer("schema=1,id=inspect-rally,action=inspect,type=rally",{},true);
      const auto proof=r.runtime.storedMissionProof();
      require(proof.valid&&proof.revision==prior.revision&&proof.content_id==prior.content_id&&proof.verified_at==prior.verified_at&&
              r.runtime.missionExecutionGranted("run"),"matching flight or unrelated plan inspection affected proof");
      auto changed=r.plan;changed[1].x+=1000;
      r.transfer("schema=1,id=inspect-changed-flight,action=inspect,type=flight",changed,true);
      require(!r.runtime.storedMissionProof().valid&&!r.runtime.missionExecutionGranted("run")&&r.runtime.controlBusy(),
              "contradictory flight inspection preserved stale proof");
    });
    test("late local ACK acceptance cannot certify storage readback",[]{
      Rig r;
      r.runtime.setBoundedSend([&](const uint8_t* data,std::size_t size,double expires,const Runtime::SendEligibility& eligible) {
        if(!eligible(r.time))return false;
        mavlink_message_t message{},buffer{};mavlink_status_t state{},result{};
        for(std::size_t i=0;i<size;++i)
          if(mavlink_frame_char_buffer(&buffer,&state,data[i],&message,&result)==MAVLINK_FRAMING_OK)r.wire.push_back(message);
        if(message.msgid==MAVLINK_MSG_ID_MISSION_ACK)r.time=expires;
        return true;
      });
      r.transfer("id=late-readback,action=upload,items=0:16:0:0:1:0:0:0:0:423584560:-710875890:10"
                 "|1:16:3:0:1:0:0:0:0:423584660:-710875890:5"
                 "|2:16:3:0:1:0:0:0:0:423584760:-710875890:5",r.plan);
      require(!r.runtime.storedMissionProof().valid,"late local ACK certified readback");
    });
    test("final socket gate rejects revocation after packing",[]{Rig r;r.acquire();auto c=r.command(MAV_CMD_MISSION_START);require(r.runtime.sendCommand(c),"start admission");bool checked=false;
      r.runtime.setBoundedSend([&](const uint8_t* data,std::size_t n,double,const Runtime::SendEligibility& eligible){
        mavlink_message_t msg{},buffer{};mavlink_status_t state{},status{};
        for(std::size_t i=0;i<n;++i)mavlink_frame_char_buffer(&buffer,&state,data[i],&msg,&status);
        if(msg.msgid==MAVLINK_MSG_ID_COMMAND_LONG){mavlink_command_long_t command{};mavlink_msg_command_long_decode(&msg,&command);
          if(command.command==MAV_CMD_MISSION_START){checked=true;r.runtime.invalidateStoredMissionProof("revoked_at_socket");}}
        const bool ok=eligible(r.time);if(ok)r.wire.push_back(msg);return ok;
      });r.runtime.tick(r.time);require(checked&&r.sent(MAV_CMD_MISSION_START)==0&&!r.runtime.missionExecutionGranted("run")&&r.runtime.controlBusy(),"final execution gate bypassed");});
    test("completed command IDs and expired abort IDs cannot renew",[]{Rig r;r.acquire();auto c=r.command(MAV_CMD_DO_SET_MISSION_CURRENT);require(r.runtime.sendCommand(c),"cursor");r.runtime.tick(r.time);r.runtime.commandFinished(c.id);
      c.expires+=.1;require(!r.runtime.sendCommand(c),"command ID reused");require(r.runtime.beginMissionExecutionAbort("once","run",r.time+.1),"abort once");
      r.time+=.2;r.refresh();require(!r.runtime.beginMissionExecutionAbort("once","run",r.time+3),"same abort renewed original expiry");require(r.runtime.beginMissionExecutionAbort("new-explicit","run",r.time+3),"new explicit abort denied");});
    test("abort binding cannot authorize continuation or foreign ownership",[]{Rig r;r.acquire();require(r.runtime.beginMissionExecutionAbort("stop","run",r.time+5),"abort context");
      for(auto type:{MAV_CMD_MISSION_START,MAV_CMD_DO_SET_MISSION_CURRENT,MAV_CMD_DO_PAUSE_CONTINUE})require(!r.runtime.sendCommand(r.command(type,"wrong-kind","stop")),"abort granted a motion/start effect");
      auto stop=r.command(MAV_CMD_DO_SET_MODE,"foreign-stop","stop");stop.execution_id="other";require(!r.runtime.sendCommand(stop),"foreign execution binding accepted");});
    test("pending observation transfer does not acquire vehicle control",[]{Rig r;const auto p=r.upload();r.time+=.01;r.refresh();MissionRequest read;read.id="read-list";read.operation_id="read";read.expires=r.time+1;read.operation_expires=r.time+5;
      require(r.runtime.mission(read),"read admission");require(r.runtime.beginMissionExecution("run",p.revision,p.content_id,r.time+10),"read-only transfer blocked explicit execution");
      require(r.runtime.controlBusy()&&r.runtime.missionExecutionGranted("run"),"read took execution authority");});
    test("equal-time pre-send Guided heartbeat cannot release pending AUTO",[]{Rig r;r.acquire();auto start=r.command(MAV_CMD_MISSION_START);require(r.runtime.sendCommand(start),"start");r.runtime.tick(r.time);r.runtime.commandFinished(start.id);
      require(r.runtime.beginMissionExecutionAbort("same-time-release","run",r.time+3),"release context");require(!r.runtime.finishMissionExecution("run","RELEASED")&&r.runtime.controlBusy(),"equal-time cached Guided heartbeat released execution");});
    test("unexpected AUTO before start cannot be retired as a rejection",[]{Rig r;r.acquire();r.time+=.01;r.refresh(true,3);
      require(!r.runtime.finishMissionExecution("run","REJECTED")&&r.runtime.controlBusy(),"unexpected AUTO freed ownership");});
    test("AUTO observed after expiry also retains unresolved ownership",[]{Rig r;r.acquire(.1);r.time+=.2;r.refresh();r.time+=.01;r.refresh(true,3);
      require(!r.runtime.finishMissionExecution("run","REJECTED")&&r.runtime.controlBusy(),"post-expiry AUTO escaped latch");});
    test("accepted cursor handoff cannot retire as an unsent rejection",[]{Rig r;r.acquire();auto cursor=r.command(MAV_CMD_DO_SET_MISSION_CURRENT);require(r.runtime.sendCommand(cursor),"cursor");r.runtime.tick(r.time);r.runtime.commandFinished(cursor.id);
      require(r.sent(MAV_CMD_DO_SET_MISSION_CURRENT)==1&&!r.runtime.finishMissionExecution("run","REJECTED")&&r.runtime.controlBusy(),"sent cursor retired as no-handoff rejection");});
    std::cout<<"PASS "<<count<<" mission execution host groups; actual DSOs, fake I/O only\n";return 0;
  }catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
