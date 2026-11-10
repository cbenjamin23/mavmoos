#include "Runtime.h"
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace mavmoos::runtime;
namespace {
void require(bool ok,const char* reason)
{
  if(!ok) throw std::runtime_error(reason);
}

mavlink_message_t decode(const uint8_t* data,std::size_t size)
{
  mavlink_message_t message{},buffer{};
  mavlink_status_t status{},result{};
  for(std::size_t i=0;i<size;++i)
    if(mavlink_frame_char_buffer(&buffer,&status,data[i],&message,&result)==MAVLINK_FRAMING_OK)
      return message;
  throw std::runtime_error("invalid encoded packet");
}

bool logRequest(uint32_t id)
{
  return id==MAVLINK_MSG_ID_LOG_REQUEST_LIST || id==MAVLINK_MSG_ID_LOG_REQUEST_DATA ||
    id==MAVLINK_MSG_ID_LOG_REQUEST_END || id==MAVLINK_MSG_ID_LOG_ERASE;
}

struct Rig {
  double at=100;
  std::vector<mavlink_message_t> sent;
  Runtime host;
  explicit Rig(bool simulation=true,bool shadow=false,unsigned type=MAV_TYPE_QUADROTOR)
    :host([](const auto&,const auto&,double,bool){},
          [this](const uint8_t* data,std::size_t size) {
            sent.push_back(decode(data,size));return true;
          },simulation,shadow,[this]{return at;},type)
  { heartbeat(false,type); }
  void heartbeat(bool armed=false,unsigned type=MAV_TYPE_QUADROTOR)
  {
    mavlink_message_t msg{};
    mavlink_msg_heartbeat_pack(1,1,&msg,type,MAV_AUTOPILOT_ARDUPILOTMEGA,
      armed?MAV_MODE_FLAG_SAFETY_ARMED:0,0,MAV_STATE_STANDBY);
    host.message(msg,at);
  }
  void ground(unsigned landed=MAV_LANDED_STATE_ON_GROUND)
  {
    mavlink_message_t msg{};
    mavlink_msg_extended_sys_state_pack(1,1,&msg,0,landed);host.message(msg,at);
  }
  void tick() { host.tick(at); }
  std::vector<unsigned> logs() const
  {
    std::vector<unsigned> ids;
    for(const auto& msg:sent) if(logRequest(msg.msgid)) ids.push_back(msg.msgid);
    return ids;
  }
};

FlightLogRequest request(FlightLogAction action=FlightLogAction::List,double expires=101,double operation=102)
{
  FlightLogRequest r;
  r.id="phase-"+std::to_string(unsigned(action));r.operation_id="transfer";
  r.action=action;r.expires=expires;r.operation_expires=operation;
  if(action==FlightLogAction::Data) { r.log_id=7;r.offset=90;r.count=180; }
  return r;
}
}

int main(int argc,char** argv)
{
  try {
    require(argc==2,"expected Copter provider image");
    { Rig r;require(!r.host.flightLog(request()),"missing landed evidence allowed a transfer");
      r.ground();r.heartbeat(true);require(!r.host.flightLog(request()),"armed transfer admitted");
      r.heartbeat(false);r.ground(MAV_LANDED_STATE_IN_AIR);
      require(!r.host.flightLog(request()),"in-air transfer admitted"); }
    { Rig rx(false),shadow(true,true),rover(true,false,MAV_TYPE_GROUND_ROVER);
      for(auto* r:{&rx,&shadow,&rover}) {
        r->ground();require(!r->host.flightLog(request()),"ineligible profile admitted");
        r->tick();require(r->logs().empty(),"ineligible profile emitted");
      }
      require(rx.sent.empty()&&shadow.sent.empty(),"RX/shadow lost zero egress"); }
    { Rig r;r.ground();auto data=request(FlightLogAction::Data);
      data.count=3601;require(!r.host.flightLog(data),"oversize data admitted");
      data.count=100;data.offset=UINT32_MAX;require(!r.host.flightLog(data),"range overflow admitted");
      auto list=request();list.first=10;list.last=9;
      require(!r.host.flightLog(list),"reversed catalog range admitted");
      list.last=1000;require(!r.host.flightLog(list),"unbounded partial catalog range admitted");
      auto invalid=request(static_cast<FlightLogAction>(3));
      require(!r.host.flightLog(invalid),"unsupported log operation admitted");
      require(!r.host.flightLog(request(FlightLogAction::End)),"END without a prior handoff admitted");
      require(r.host.flightLog(request()),"valid list denied");
      require(!r.host.flightLog(request(FlightLogAction::Data)),"waiting queue exceeds one intent");
      r.host.flightLogFinished("transfer");r.tick();
      require(r.logs().empty(),"unsent cancel emitted protocol traffic");
      require(!r.host.flightLog(request(FlightLogAction::End)),"unsent cancel manufactured cleanup"); }
    { Rig r;r.ground();require(r.host.load(argv[1]),"load Copter control provider");
      mavlink_message_t estimator{};mavlink_ekf_status_report_t health{};health.flags=51;
      mavlink_msg_ekf_status_report_encode(1,1,&estimator,&health);r.host.message(estimator,r.at);
      require(r.host.flightLog(request()),"valid transfer denied");
      r.tick();r.tick(); // The initial stream request shares this protocol slot.
      const auto& list=r.sent.back();mavlink_log_request_list_t payload{};
      mavlink_msg_log_request_list_decode(&list,&payload);
      require(list.msgid==117&&list.sysid==245&&list.compid==191&&payload.target_system==1&&
        payload.target_component==1&&payload.start==0&&payload.end==0xffff,"list wire scope");
      auto data=request(FlightLogAction::Data);
      data.operation_id="other";require(!r.host.flightLog(data),"competing log operation admitted");
      data.operation_id="transfer";data.operation_expires=103;
      require(!r.host.flightLog(data),"original operation deadline changed");
      data.operation_expires=102;require(r.host.flightLog(data),"valid data denied");
      CommandRequest arm{"arm",MAV_CMD_COMPONENT_ARM_DISARM,{},101,102,{},{}};
      arm.params[0]=1;require(!r.host.sendCommand(arm),"arming overlapped active transfer");
      MissionRequest clear;clear.id="clear";clear.operation_id="mission";clear.action=MissionAction::Clear;
      clear.expires=101;clear.operation_expires=102;
      require(!r.host.mission(clear),"mutation overlapped active transfer");
      r.tick();
      const auto& block=r.sent.back();mavlink_log_request_data_t range{};
      mavlink_msg_log_request_data_decode(&block,&range);
      require(block.msgid==119&&range.target_system==1&&range.target_component==1&&range.id==7&&
        range.ofs==90&&range.count==180,"data wire scope");
      require(r.host.flightLog(data),"waiting replacement denied");
      require(r.host.flightLog(request(FlightLogAction::End)),"authorized END denied");
      require(!r.host.flightLog(data),"data survived END-only state");r.tick();
      require(r.logs()==std::vector<unsigned>({117,119,122}),"END did not supersede waiting data");
      mavlink_log_request_end_t end{};mavlink_msg_log_request_end_decode(&r.sent.back(),&end);
      require(end.target_system==1&&end.target_component==1,"END wire scope");
      require(!r.host.flightLog(request(FlightLogAction::End)),"successful END left a live transfer");
      require(r.host.sendCommand(arm),"successful END failed to release control fence"); }
    { Rig r;r.ground();require(r.host.flightLog(request()),"first list denied");r.tick();
      r.at=101.5;r.heartbeat();r.ground();
      r.at=102.1;r.heartbeat();r.ground();r.tick();
      require(!r.host.flightLog(request(FlightLogAction::Data,103,102)),"expired transfer resumed");
      require(!r.host.flightLog(request(FlightLogAction::End,104.1,102)),"cleanup deadline extended");
      require(r.host.flightLog(request(FlightLogAction::End,103,102)),"bounded post-expiry cleanup denied");
      r.tick();require(r.logs()==std::vector<unsigned>({117,122}),"expiry/cleanup wire sequence"); }
    { Rig r;r.ground();require(r.host.flightLog(request()),"expiry fixture admission");
      bool reached=false;
      r.host.setBoundedSend([&](const uint8_t* data,std::size_t size,double expires,const auto& eligible) {
        if(decode(data,size).msgid!=117) return true;
        reached=true;r.at=expires;
        require(!eligible(r.at),"exact deadline transmitted");return false;
      });
      r.tick();require(reached&&!r.host.flightLog(request(FlightLogAction::End,101.9,102)),
        "unaccepted handoff allowed END"); }
    { Rig r;r.ground();require(r.host.flightLog(request()),"cancel fixture admission");
      bool reached=false;
      r.host.setBoundedSend([&](const uint8_t* data,std::size_t size,double,const auto& eligible) {
        if(decode(data,size).msgid!=117) return true;
        reached=true;r.host.flightLogFinished("transfer");
        require(!eligible(r.at),"cancelled handoff transmitted");return false;
      });
      r.tick();require(reached,"did not reach the final send check"); }
    { Rig r;r.ground();require(r.host.flightLog(request()),"connection fixture admission");r.tick();
      require(r.host.flightLog(request(FlightLogAction::Data)),"queue under live connection");
      r.host.disconnect();r.at=100.1;r.heartbeat();r.ground();
      require(!r.host.flightLog(request(FlightLogAction::End)),"reset revived old cleanup scope");
      r.tick();require(r.logs()==std::vector<unsigned>({117}),"old-session data survived reset"); }
    { Rig r;r.ground();require(r.host.flightLog(request()),"arm fixture admission");r.tick();
      require(r.host.flightLog(request(FlightLogAction::Data)),"queue under disarmed identity");
      r.heartbeat(true);r.heartbeat(false);r.ground();r.tick();
      require(r.logs()==std::vector<unsigned>({117})&&!r.host.flightLog(request(FlightLogAction::End)),
        "transient external arm left old operation eligible"); }
    std::cout<<"Flight log host wire, expiry, exclusivity, cleanup and reset checks passed\n";
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<"FAIL flight log host: "<<error.what()<<'\n';return 1;
  }
}
