/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                  */
/*    FILE: CopterCommands.cpp                               */
/*    DATE: November 3, 2026                                 */
/************************************************************/
#include "ControlReplayService.h"
#include "ControlComponents.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>

namespace mavmoos::runtime {
namespace {
std::string trim(const std::string& value)
{
  const auto first=value.find_first_not_of(" \t\r\n");
  if(first==std::string::npos) return {};
  return value.substr(first,value.find_last_not_of(" \t\r\n")-first+1);
}
bool number(const std::string& value,double& result)
{
  try { std::size_t used=0; result=std::stod(value,&used); return used==value.size()&&std::isfinite(result); }
  catch(...) { return false; }
}
bool identifier(const std::string& value)
{
  return !value.empty()&&value.size()<=48&&std::all_of(value.begin(),value.end(),[](unsigned char c){
    return (c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_'||c=='-'||c=='.';
  });
}
struct Operation {
  std::string id,action,replay_id;
  double height=5,delta=0,started=0,expires=0,handoff_expires=0;
  uint16_t command=0;
  std::array<float,7> params{};
  std::optional<double> sent_at;
  std::optional<ParameterRequest> parameter;
  std::optional<double> parameter_sent_at;
  unsigned precision_parameter=0;
  bool confirmed=false,ack_observed=false;
};
bool speedAdjustment(const Operation& op)
{ return op.action=="change_speed"||op.action=="reset_speed_min"; }
bool precisionToggle(const Operation& op)
{ return op.action=="precision_loiter_on"||op.action=="precision_loiter_off"; }
bool returnToLaunch(const Operation& op)
{ return op.action=="rtl"||op.action=="dead_man_rtl"; }
constexpr float precision_loiter_function=39;
class Commands final: public Plugin {
public:
  explicit Commands(Services& services):m_services(services),m_boundary(services.now()) {}
  std::string name() const override { return "commands"; }
  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    if(m_adjustments_enabled) {
      result.capabilities.push_back({"control.guided.speed_adjustment","MAVMOOS_COMMAND","action=change_speed","request"});
      result.capabilities.push_back({"control.guided.speed_reset","MAVMOOS_COMMAND","action=reset_speed_min","request"});
      result.capabilities.push_back({"control.guided.course_adjustment","MAVMOOS_COMMAND","action=change_course","request"});
    }
    if(m_precision_enabled) {
      result.capabilities.push_back({"control.precision_loiter.enable","MAVMOOS_COMMAND","action=precision_loiter_on","request"});
      result.capabilities.push_back({"control.precision_loiter.disable","MAVMOOS_COMMAND","action=precision_loiter_off","request"});
    }
    if(!m_dead_man_source.empty()) {
      result.capabilities.push_back({"control.return.dead_man_rtl","DEAD_MAN_POST_INTERRUPT","true","request"});
      result.request_owners.push_back("DEAD_MAN_POST_INTERRUPT");
    }
    return result;
  }
  std::vector<std::string> subscriptions() const override
  {
    if(m_dead_man_source.empty()) return {"MAVMOOS_COMMAND"};
    return {"MAVMOOS_COMMAND","DEAD_MAN_POST_INTERRUPT"};
  }
  void configure(const std::string& key,const std::string& value) override
  {
    if(key=="control_precision_loiter"||key=="control_copter_adjustments") {
      if(value!="true"&&value!="false") throw std::invalid_argument("invalid "+key);
      (key=="control_precision_loiter"?m_precision_enabled:m_adjustments_enabled)=value=="true";
    }
    if(key=="dead_man_source") {
      if(!identifier(value)) throw std::invalid_argument("invalid dead_man_source");
      m_dead_man_source=value;
    }
    double parsed=0;
    if(key=="command_speed_min") {
      if(!number(value,parsed)||parsed<.1||parsed>5) throw std::invalid_argument("command_speed_min outside .1..5m/s");
      m_min_speed=parsed;
    }
    if(key=="command_ack_timeout"||key=="takeoff_timeout"||key=="land_timeout"||key=="takeoff_height") {
      if(!number(value,parsed)||parsed<=0||parsed>120) throw std::invalid_argument("invalid "+key);
      if(key=="command_ack_timeout") m_command_window=parsed;
      if(key=="takeoff_timeout") m_takeoff_timeout=parsed;
      if(key=="land_timeout") m_land_timeout=parsed;
      if(key=="takeoff_height") {
        if(parsed<1||parsed>20) throw std::invalid_argument("takeoff_height outside 1..20m");
        m_default_height=parsed;
      }
    }
  }
  void onMail(const Mail& mail) override { receive(mail,{}); }
  void receive(const Mail& mail,const std::string& public_id)
  {
    if(mail.key=="DEAD_MAN_POST_INTERRUPT") { queueDeadMan(mail); return; }
    if(mail.key!="MAVMOOS_COMMAND") return;
    Operation op; op.height=m_default_height;
    if(mail.numeric||mail.text.empty()||mail.text.size()>256) {
      op.id=op.action="invalid";report(op,"REJECTED","invalid_request");return;
    }
    std::map<std::string,std::string> fields;
    std::istringstream stream(mail.text); std::string item;
    bool valid=true;
    while(valid&&std::getline(stream,item,',')) {
      const auto split=item.find('=');
      if(split==std::string::npos) { valid=false; break; }
      const auto key=trim(item.substr(0,split)),value=trim(item.substr(split+1));
      if((key!="id"&&key!="action"&&key!="height"&&key!="delta")||!fields.emplace(key,value).second) { valid=false; break; }
    }
    op.id=fields.count("id")?fields["id"]:"invalid";
    op.action=fields.count("action")?fields["action"]:"invalid";
    if(!identifier(op.id)||!fields.count("id")||op.id.rfind("deadman.",0)==0) { op.id="invalid"; valid=false; }
    static const std::set<std::string> actions={"arm","disarm","takeoff","hold","land","rtl","guided",
      "change_speed","reset_speed_min","change_course","precision_loiter_on","precision_loiter_off"};
    if(!actions.count(op.action)) { op.action="invalid"; valid=false; }
    if(fields.count("height")&&(!number(fields["height"],op.height)||op.action!="takeoff")) valid=false;
    const bool relative=op.action=="change_speed"||op.action=="change_course";
    if(relative!=bool(fields.count("delta"))||(relative&&!number(fields["delta"],op.delta))) valid=false;
    if(!mail.text.empty()&&mail.text.back()==',') valid=false;
    if(!valid) { report(op,"REJECTED","invalid_request"); return; }
    if(m_seen.count(op.id)||(m_queued&&m_queued->id==op.id)) { report(op,"DUPLICATE","id_already_seen"); return; }
    op.replay_id=public_id.empty()?op.id:public_id;
    if(controlRequestSeen(m_services,op.replay_id)) {report(op,"DUPLICATE",replayReason(ReplayResult::Seen));return;}
    if(m_pending||m_queued) { report(op,"REJECTED","busy"); return; }
    if(!validTime(mail)) {
      report(op,"REJECTED","stale_or_future_request"); return;
    }
    if(mail.published_at<=m_boundary) { report(op,"REJECTED","pre_session_request"); return; }
    op.started=mail.received_at;
    m_queued=op;
  }
  void onCommandSent(const CommandRequest& request,double at,bool accepted) override
  {
    expirePending(m_services.now());
    if(!m_pending||m_pending->sent_at||request.id!=m_pending->id||
       request.command!=m_pending->command||request.params!=m_pending->params||
       !std::isfinite(request.expires)||!std::isfinite(request.operation_expires)||
       request.expires>m_pending->handoff_expires||request.operation_expires>m_pending->expires||
       request.operation_expires<request.expires||
       !std::isfinite(at)||at<m_pending->started||at>=request.expires||at>m_services.now()) return;
    if(!accepted) { finish("FAILED","local_send_not_accepted"); return; }
    m_pending->sent_at=at;
  }
  void onParameterSent(const ParameterRequest& request,double at,bool accepted) override
  {
    expirePending(m_services.now());
    if(!m_pending||!m_pending->parameter||m_pending->parameter_sent_at) return;
    const auto& expected=*m_pending->parameter;
    if(request.id!=expected.id||request.name!=expected.name||request.action!=ParameterAction::Read||request.index!=-1||
       request.expires!=expected.expires||!std::isfinite(at)||at<m_pending->started||
       at>=expected.expires||at>m_services.now()) return;
    if(!accepted) { finish("FAILED","precision_parameter_send_failed"); return; }
    m_pending->parameter_sent_at=at;
  }
  void onMessage(const mavlink_message_t& message,double at) override
  { onMessageWithMetadata(message,{at,at,"READ_ACQUISITION"}); }
  void onMessageWithMetadata(const mavlink_message_t& message,const MessageTime& time) override
  {
    // First delivery after a deadline cannot revive a transaction, even if the
    // frame or newly observed state was queued before the owner resumed.
    expirePending(m_services.now());
    if(message.sysid==1&&message.compid==1) {
      if(m_pending&&modeInterrupted(*m_pending,message.msgid)) finish("CANCELLED","flight_authority_changed");
      if(m_queued&&modeInterrupted(*m_queued,message.msgid)) {
        report(*m_queued,"CANCELLED","flight_authority_changed");m_queued.reset();
      }
    }
    if(m_pending&&m_pending->parameter&&message.msgid==MAVLINK_MSG_ID_PARAM_VALUE) {
      observePrecisionParameter(message,time); return;
    }
    if(!m_pending||!m_pending->sent_at||message.msgid!=MAVLINK_MSG_ID_COMMAND_ACK||
       message.sysid!=1||message.compid!=1||!std::isfinite(time.read_at)||
       time.read_at<=*m_pending->sent_at||time.read_at>m_services.now()) return;
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&message,&ack);
    if(ack.command!=m_pending->command||(ack.target_system!=0&&ack.target_system!=245)||
       (ack.target_component!=0&&ack.target_component!=191)) return;
    // The ACK identifies a command number, but carries no operation ID or parameters.
    if(!m_pending->ack_observed&&m_services.now()<m_pending->handoff_expires) {
      m_pending->ack_observed=true;
      report(*m_pending,"ACK_OBSERVED","uncorrelated_ack_result_"+std::to_string(ack.result));
    }
  }
  void onReset(const std::string&) override
  {
    if(m_pending) finish("CANCELLED","connection_reset");
    if(m_queued) {
      report(*m_queued,"CANCELLED","connection_reset");
      if(m_queued->action!="dead_man_rtl"&&m_seen.size()<128) m_seen.insert(m_queued->id);
    }
    m_pending.reset(); m_queued.reset();
    m_boundary=m_services.now();
  }
  void onInputBoundary() override { expirePending(m_services.now()); }
  void iterate() override
  {
    const double now=m_services.now();
    if(m_queued) {
      Operation op=*m_queued; m_queued.reset();
      const auto reason=guard(op,now);
      if(!reason.empty()) { report(op,"REJECTED",reason); return; }
      if(op.action!="dead_man_rtl"&&m_seen.size()>=128) { report(op,"REJECTED","request_history_full"); return; }
      if(op.action!="dead_man_rtl") {
        const auto replay=reserveControlRequest(m_services,op.replay_id);
        if(replay!=ReplayResult::Reserved) {report(op,replayRejection(replay),replayReason(replay));return;}
      }
      op.started=now;
      op.expires=now+(op.action=="takeoff"?m_takeoff_timeout:(op.action=="land"||returnToLaunch(op)?m_land_timeout:8));
      if(op.action!="dead_man_rtl") m_seen.insert(op.id);
      report(op,"ADMITTED","policy_passed");
      if(op.action=="precision_loiter_on") m_pending=std::move(op);
      else submit(std::move(op),now);
    }
    expirePending(now);
    if(m_pending&&m_pending->command==0) { preparePrecision(now); return; }
    if(!m_pending||!m_pending->sent_at) return;
    auto& op=*m_pending;
    const auto& v=m_services.vehicle();
    const bool heartbeat=fresh(v.heartbeat_valid,v.heartbeat_at,now,2)&&v.heartbeat_at>*op.sent_at;
    const bool landed=fresh(v.landed_valid,v.landed_at,now,2)&&v.landed_at>*op.sent_at;
    const bool position=fresh(v.position_valid,v.position_at,now,.75)&&v.position_at>*op.sent_at;
    bool confirmed=false,complete=false;
    if(op.action=="arm") confirmed=complete=heartbeat&&v.armed;
    if(op.action=="disarm") confirmed=complete=heartbeat&&!v.armed;
    if(op.action=="guided") confirmed=complete=heartbeat&&v.mode==4;
    if(op.action=="hold") confirmed=complete=heartbeat&&v.mode==5;
    if(op.action=="takeoff") {
      confirmed=heartbeat&&v.armed&&landed&&v.in_air;
      complete=confirmed&&position&&v.relative_height>=op.height-.5;
      if(heartbeat&&(!v.armed||(v.mode!=4&&v.mode!=5))) { finish("FAILED","takeoff_interrupted"); return; }
    }
    if(op.action=="land"||returnToLaunch(op)) {
      confirmed=heartbeat&&(v.mode==(op.action=="land"?9u:6u));
      const bool on_ground=landed&&v.landed_state==MAV_LANDED_STATE_ON_GROUND&&!v.in_air;
      if(op.confirmed&&heartbeat&&!confirmed&&!on_ground) {finish("CANCELLED","landing_mode_interrupted");return;}
      complete=(confirmed||op.confirmed)&&on_ground;
    }
    if(op.action=="change_course") {
      if(heartbeat&&v.mode!=4) { finish("CANCELLED","guided_mode_lost"); return; }
      complete=position&&std::isfinite(v.heading)&&std::abs(std::remainder(v.heading-op.params[0],360.))<=5;
      confirmed=complete;
    }
    if(confirmed&&!op.confirmed) { op.confirmed=true; report(op,"CONFIRMED","post_submit_observation"); }
    if(complete) { finish("COMPLETED","observed_outcome"); return; }
  }
private:
  bool modeInterrupted(const Operation& op,uint32_t message_id) const
  {
    const bool precision=precisionToggle(op);
    const bool guided=speedAdjustment(op)||op.action=="change_course";
    const bool dead_man=op.action=="dead_man_rtl"&&!op.sent_at;
    if(!precision&&!guided&&!dead_man) return false;
    const auto& v=m_services.vehicle();
    if(message_id==MAVLINK_MSG_ID_HEARTBEAT)
      return !v.armed || (precision?v.mode!=5:guided?v.mode!=4:(v.mode!=4&&v.mode!=5));
    return message_id==MAVLINK_MSG_ID_EXTENDED_SYS_STATE&&(!v.landed_valid||!v.in_air);
  }
  bool validTime(const Mail& mail) const
  {
    const double now=m_services.now(),age=mail.received_at-mail.published_at;
    return std::isfinite(now)&&std::isfinite(age)&&age>=-.1&&age<=2&&
      mail.received_at<=now&&now-mail.received_at<=2;
  }
  void queueDeadMan(const Mail& mail)
  {
    if(m_dead_man_source.empty()||mail.source!=m_dead_man_source||!validTime(mail)||
       mail.published_at<=m_boundary||mail.published_at<=m_dead_man_at||
       (mail.numeric?mail.value!=1:mail.text!="true")) return;
    m_dead_man_at=mail.published_at;
    Operation op;op.id="deadman."+std::to_string(++m_dead_man_sequence);op.action="dead_man_rtl";
    const auto& v=m_services.vehicle();
    if((m_pending&&m_pending->action=="land")||(m_queued&&m_queued->action=="land")||
       (v.heartbeat_valid&&v.mode==9)) {report(op,"NO_OP","land_in_progress");return;}
    if(m_pending||m_queued||m_services.controlBusy()) {report(op,"REJECTED","busy");return;}
    const auto replay=reserveControlEvent(m_services);
    if(replay!=ReplayResult::Reserved) {report(op,replayRejection(replay),replayReason(replay));return;}
    op.started=mail.received_at;m_queued=op;
  }
  void submit(Operation op,double now)
  {
    CommandRequest request;request.id=op.id;
    const auto& v=m_services.vehicle();
    if(op.action=="arm"||op.action=="disarm") {
      request.command=MAV_CMD_COMPONENT_ARM_DISARM;
      request.params[0]=op.action=="arm"?1.f:0.f;
    } else if(op.action=="takeoff") {
      request.command=MAV_CMD_NAV_TAKEOFF;request.params[6]=static_cast<float>(op.height);
    } else if(op.action=="land") request.command=MAV_CMD_NAV_LAND;
    else if(returnToLaunch(op)) request.command=MAV_CMD_NAV_RETURN_TO_LAUNCH;
    else if(speedAdjustment(op)) {
      const double measured=std::hypot(v.north_velocity,v.east_velocity);
      const double speed=op.action=="reset_speed_min"?m_min_speed:std::max(measured,m_min_speed)+op.delta;
      if(!std::isfinite(speed)||speed<m_min_speed||speed>5) {report(op,"REJECTED","speed_outside_profile");return;}
      request.command=MAV_CMD_DO_CHANGE_SPEED;
      request.params[0]=1;request.params[1]=static_cast<float>(speed);request.params[2]=-1;
    } else if(op.action=="change_course") {
      request.command=MAV_CMD_CONDITION_YAW;
      request.params[0]=static_cast<float>(std::fmod(v.heading+op.delta+360.,360.));
      request.params[1]=30;
    } else if(precisionToggle(op)) {
      request.command=MAV_CMD_DO_AUX_FUNCTION;
      request.params[0]=precision_loiter_function;request.params[1]=op.action=="precision_loiter_on"?2.f:0.f;
    } else {
      request.command=MAV_CMD_DO_SET_MODE;
      request.params[0]=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
      request.params[1]=op.action=="guided"?4.f:5.f;
    }
    op.command=request.command;op.params=request.params;
    op.handoff_expires=request.expires=std::min(op.expires,now+m_command_window);
    request.operation_expires=op.expires;
    if(!m_services.sendCommand(request)) {report(op,"REJECTED","host_send_inhibited");return;}
    m_pending=std::move(op);report(*m_pending,"SUBMITTED","host_queue_admitted");
  }
  void preparePrecision(double now)
  {
    auto& op=*m_pending;
    if(!freshAirborneMode(5,now)||m_services.navigationGranted()||m_services.controlBusy()) {
      finish("CANCELLED","precision_loiter_prerequisites_lost");return;
    }
    if(op.precision_parameter==2) {
      auto ready=std::move(op);m_pending.reset();submit(std::move(ready),now);return;
    }
    if(op.parameter) return;
    ParameterRequest request;
    request.name=op.precision_parameter==0?"PLND_ENABLED":"PLND_TYPE";
    request.id=op.id+"/"+request.name;request.expires=std::min(op.expires,now+m_command_window);
    if(!m_services.parameter(request)) {finish("REJECTED","precision_parameter_read_inhibited");return;}
    op.parameter=request;
  }
  void observePrecisionParameter(const mavlink_message_t& message,const MessageTime& time)
  {
    auto& op=*m_pending;
    if(!op.parameter_sent_at||message.sysid!=1||message.compid!=1||!std::isfinite(time.read_at)||
       time.read_at<=*op.parameter_sent_at||time.read_at>m_services.now()) return;
    mavlink_param_value_t value{};mavlink_msg_param_value_decode(&message,&value);
    if(value.param_index!=UINT16_MAX) return;
    const std::string name(value.param_id,std::find(value.param_id,value.param_id+sizeof(value.param_id),'\0'));
    if(name!=op.parameter->name) return;
    const float decoded=value.param_value;
    if(value.param_type!=MAV_PARAM_TYPE_INT8||!std::isfinite(decoded)||std::trunc(decoded)!=decoded||
       (op.precision_parameter==0?decoded!=1:(decoded<1||decoded>4))) {
      finish("REJECTED","precision_landing_not_configured");return;
    }
    ++op.precision_parameter;op.parameter.reset();op.parameter_sent_at.reset();
  }
  void expirePending(double now)
  {
    if(!m_pending) return;
    const auto& op=*m_pending;
    if(!std::isfinite(now)||now<op.started) { finish("FAILED","clock_discontinuity"); return; }
    // A timeout retires local confirmation; it does not recall a command that
    // may already have reached the flight controller.
    if(op.command&&now>=op.handoff_expires) {
      if(!op.sent_at) { finish("TIMED_OUT","local_handoff_unavailable"); return; }
      if(speedAdjustment(op)) { finish("UNCONFIRMED","guided_speed_cap_not_observable"); return; }
      if(precisionToggle(op)) { finish("UNCONFIRMED","precision_aux_state_not_observable"); return; }
    }
    if(now>=op.expires) { finish("TIMED_OUT","operation_deadline"); return; }
    if(op.parameter&&now>=op.parameter->expires) {finish("TIMED_OUT","precision_parameter_unavailable");return;}
  }
  static bool fresh(bool valid,double stamp,double now,double max_age)
  { return valid&&std::isfinite(stamp)&&stamp<=now&&now-stamp<=max_age; }
  bool freshAirborneMode(unsigned mode,double now) const
  {
    const auto& v=m_services.vehicle();
    return fresh(v.heartbeat_valid,v.heartbeat_at,now,2)&&v.mode==mode&&v.armed&&v.in_air&&
      fresh(v.landed_valid,v.landed_at,now,2)&&fresh(v.position_valid,v.position_at,now,.5)&&
      fresh(v.estimator_valid,v.estimator_at,now,3)&&v.home_valid;
  }
  std::string guard(const Operation& op,double now) const
  {
    const auto& v=m_services.vehicle();
    if(!std::isfinite(now)||now<op.started||now-op.started>2) return "queued_request_expired";
    if(!fresh(v.heartbeat_valid,v.heartbeat_at,now,2)) return "heartbeat_unavailable";
    if(op.action=="arm") {
      if(v.armed) return "already_armed";
      if(!fresh(v.landed_valid,v.landed_at,now,2)||v.in_air||v.landed_state!=MAV_LANDED_STATE_ON_GROUND) return "ground_state_required";
      if(!fresh(v.estimator_valid,v.estimator_at,now,3)) return "estimator_unavailable";
    }
    if(op.action=="disarm") {
      if(!fresh(v.landed_valid,v.landed_at,now,2)||v.in_air||v.landed_state!=MAV_LANDED_STATE_ON_GROUND) return "ground_state_required";
      if(!v.armed) return "already_disarmed";
    }
    if(op.action=="takeoff") {
      if(op.height<1||op.height>20) return "height_outside_1_20m";
      if(!v.armed) return "armed_required";
      if(v.mode!=4) return "guided_required";
      if(!fresh(v.landed_valid,v.landed_at,now,2)||v.in_air||v.landed_state!=MAV_LANDED_STATE_ON_GROUND) return "ground_state_required";
      if(!fresh(v.position_valid,v.position_at,now,.75)||!v.home_valid) return "pose_home_required";
    }
    if(op.action=="hold"||returnToLaunch(op)) {
      if(!v.armed||!fresh(v.position_valid,v.position_at,now,.75)) return "armed_fresh_pose_required";
      if(returnToLaunch(op)&&!v.home_valid) return "home_required";
      if(returnToLaunch(op)&&(v.mode!=4&&v.mode!=5)) return v.mode==9?"land_in_progress":"guided_or_loiter_required";
      if(op.action=="dead_man_rtl"&&!freshAirborneMode(v.mode,now)) return "fresh_airborne_state_required";
    }
    if(speedAdjustment(op)||op.action=="change_course"||precisionToggle(op)) {
      if(precisionToggle(op)&&!m_precision_enabled) return "precision_loiter_disabled";
      if(!precisionToggle(op)&&!m_adjustments_enabled) return "guided_adjustments_disabled";
      if(m_services.navigationGranted()||m_services.controlBusy()) return "release_helm_and_control_first";
      if(!freshAirborneMode(precisionToggle(op)?5u:4u,now))
        return precisionToggle(op)?"fresh_airborne_loiter_required":"fresh_airborne_guided_required";
      if(op.action=="change_course"&&(!std::isfinite(v.heading)||v.heading<0||v.heading>=360||std::abs(op.delta)>360))
        return "heading_or_delta_invalid";
    }
    if(op.action=="land") {
      if(!v.armed) return "armed_required";
      if(v.mode!=3&&v.mode!=4&&v.mode!=5) return "routine_land_mode_inhibited";
    }
    return {};
  }
  void report(const Operation& op,const std::string& status,const std::string& reason)
  {
    std::ostringstream text;
    text<<"schema=1,id="<<op.id<<",action="<<op.action<<",status="<<status<<",reason="<<reason
        <<",at="<<std::fixed<<std::setprecision(6)<<m_services.now();
    m_services.publish("MAVMOOS_COMMAND_RESULT",text.str());
  }
  void finish(const std::string& status,const std::string& reason)
  {
    report(*m_pending,status,reason);
    if(m_pending->command) m_services.commandFinished(m_pending->id);
    m_pending.reset();
  }
  Services& m_services;
  std::optional<Operation> m_queued,m_pending;
  std::set<std::string> m_seen;
  double m_command_window=4,m_takeoff_timeout=45,m_land_timeout=60,m_default_height=5;
  double m_boundary=0;
  double m_min_speed=.25,m_dead_man_at=-1;
  std::string m_dead_man_source;
  uint64_t m_dead_man_sequence=0;
  bool m_precision_enabled=false,m_adjustments_enabled=false;
};
} // namespace
std::unique_ptr<Plugin> makeCopterCommands(Services& services)
{ return std::make_unique<Commands>(services); }
void deliverCopterNativeRequest(Plugin& commands,const Mail& mail,const std::string& public_id)
{ dynamic_cast<Commands&>(commands).receive(mail,public_id); }
} // namespace mavmoos::runtime
