/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: RoverCommands.cpp                               */
/*    DATE: November 6, 2026                                */
/************************************************************/
#include "ControlReplayService.h"
#include "RoverComponents.h"
#include "RoverNativeReplay.h"
#include <algorithm>
#include <cmath>
#include <cctype>
#include <iomanip>
#include <initializer_list>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>

namespace mavmoos::runtime {
namespace {
constexpr double pi=3.14159265358979323846;
constexpr double radians=pi/180;
constexpr double earth_radius=6371000;
constexpr double pose_limit=.5;
constexpr double arrival_radius=2.5;
constexpr double stopped_speed=.3;
constexpr double settled_dwell=1;
constexpr std::size_t history_limit=128;

std::string trim(const std::string& s)
{
  const auto a=s.find_first_not_of(" \t\r\n");
  return a==std::string::npos?std::string():s.substr(a,s.find_last_not_of(" \t\r\n")-a+1);
}
bool idAllowed(const std::string& id)
{
  return !id.empty()&&id.size()<=48&&std::all_of(id.begin(),id.end(),[](unsigned char c){
    return (c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_'||c=='-'||c=='.';
  });
}
bool fresh(bool valid,double stamp,double now,double ttl)
{ return valid&&std::isfinite(stamp)&&stamp>0&&now>=stamp&&now-stamp<=ttl; }
bool coordinate(double lat,double lon)
{ return std::isfinite(lat)&&std::isfinite(lon)&&std::abs(lat)<=90&&std::abs(lon)<=180; }
struct Waypoint {
  double latitude=0,longitude=0;
};
double distance(const Waypoint& from,const Waypoint& to)
{
  const double lat1=from.latitude*radians,lat2=to.latitude*radians;
  const double dlon=std::remainder(to.longitude-from.longitude,360.)*radians;
  const double half_lat=std::sin((lat2-lat1)/2),half_lon=std::sin(dlon/2);
  const double a=half_lat*half_lat+std::cos(lat1)*std::cos(lat2)*half_lon*half_lon;
  return 2*earth_radius*std::asin(std::sqrt(std::clamp(a,0.,1.)));
}
double course(const Waypoint& from,const Waypoint& to)
{
  const double lat1=from.latitude*radians,lat2=to.latitude*radians;
  const double dlon=std::remainder(to.longitude-from.longitude,360.)*radians;
  const double east=std::sin(dlon)*std::cos(lat2);
  const double north=std::cos(lat1)*std::sin(lat2)-std::sin(lat1)*std::cos(lat2)*std::cos(dlon);
  return std::fmod(std::atan2(east,north)/radians+360.,360.);
}
std::optional<double> number(const std::string& value,double minimum,double maximum)
{
  try {
    std::size_t parsed=0;
    const double result=std::stod(value,&parsed);
    if(parsed==value.size()&&std::isfinite(result)&&result>=minimum&&result<=maximum)return result;
  } catch(const std::invalid_argument&) {} catch(const std::out_of_range&) {}
  return std::nullopt;
}
using Fields=std::map<std::string,std::string>;
bool parseFields(const Mail& mail,Fields& fields)
{
  if(mail.numeric||mail.text.empty()||mail.text.size()>256||mail.text.back()==',')return false;
  std::istringstream stream(mail.text);
  std::string part;
  while(std::getline(stream,part,',')) {
    const auto equals=part.find('=');
    if(equals==std::string::npos||
       !fields.emplace(trim(part.substr(0,equals)),trim(part.substr(equals+1))).second)return false;
  }
  return true;
}
bool knownFields(const Fields& fields,std::initializer_list<const char*> keys)
{
  return std::all_of(fields.begin(),fields.end(),[&keys](const auto& pair){
    return std::find(keys.begin(),keys.end(),pair.first)!=keys.end();
  });
}
std::string field(const Fields& fields,const std::string& name)
{
  const auto found=fields.find(name);
  return found!=fields.end()?found->second:std::string{};
}
bool recentMail(const Mail& mail,double now,double ttl)
{
  const double age=mail.received_at-mail.published_at;
  return std::isfinite(now)&&std::isfinite(mail.received_at)&&std::isfinite(mail.published_at)&&
    age>=-.1&&age<=ttl&&now>=mail.received_at&&now-mail.received_at<=ttl;
}
struct Operation {
  std::string id="invalid",action="invalid",strategy="native",cancel_target;
  bool versioned=false,ack_observed=false,observe_only=false;
  double published=0,queued=0,submitted=0,checked_at=0,expires=0,speed=1;
  unsigned initial_mode=0;
  CommandRequest request;
  Waypoint goal,settled_position;
  double position_at=0;
  std::optional<double> sent_at,confirmed_at,first_target_at,last_target_at,settled_since;
  bool guided() const { return strategy=="guided"; }
  bool returning() const { return action=="rtl"||action=="return"; }
};
unsigned nativeMode(const Operation& op)
{
  if(op.action=="guided")return rover::guided_mode;
  if(op.action=="hold")return rover::hold_mode;
  if(op.action=="loiter")return rover::loiter_mode;
  return rover::rtl_mode;
}
struct HelmHandoff { Mail request;bool stop_window_opened=false; };
class RoverCommands final:public Plugin {
public:
  RoverCommands(Services& services,const VehicleControlBackend& backend,Plugin* helm)
    :m_services(services),m_backend(backend),m_helm(helm),m_boundary(services.now()),
     m_boat(backend.profile()=="ardurover_boat") {}
  std::string name() const override { return "rover_commands"; }
  std::vector<std::string> subscriptions() const override
  { return {"MAVMOOS_COMMAND","MAVMOOS_CONTROL_REQUEST"}; }
  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.capabilities={
      {"control.operations.native","MAVMOOS_CONTROL_REQUEST","schema=1,strategy=native"},
      {"control.operations.guided","MAVMOOS_CONTROL_REQUEST","schema=1,strategy=guided"},
      {"control.operations.cancel","MAVMOOS_CONTROL_REQUEST","schema=1,action=cancel"}};
    return result;
  }
  void configure(const std::string& key,const std::string& value) override
  {
    if(key=="native_replay_directory") {
      if(m_replay)throw std::invalid_argument("native replay directory is startup-only");
      m_replay=std::make_unique<rover::NativeReplay>(value);
    } else if(key=="shadow_mode") {
      if(value!="true"&&value!="false")throw std::invalid_argument("invalid "+key);
      m_shadow=value=="true";
    } else if(key=="freshness_timeout") {
      const auto parsed=number(value,.1,10);
      if(!parsed)throw std::invalid_argument("invalid "+key);
      m_helm_freshness=*parsed;
    } else if(key=="command_ack_timeout"||key=="rover_return_timeout") {
      const auto parsed=number(value,0,120);
      if(!parsed||*parsed==0)throw std::invalid_argument("invalid "+key);
      if(key=="command_ack_timeout")m_ack_timeout=*parsed;
      else m_return_timeout=*parsed;
    }
  }
  void onMail(const Mail& mail) override
  {
    if(mail.key=="MAVMOOS_NAV_CONTROL") { helmRequest(mail);return; }
    if(mail.key!="MAVMOOS_COMMAND"&&mail.key!="MAVMOOS_CONTROL_REQUEST")return;
    Operation op;
    op.versioned=mail.key=="MAVMOOS_CONTROL_REQUEST";
    double timeout=0;
    if(!parse(mail,op,timeout)) { report(op,"REJECTED","invalid_or_unsupported_rover_request");return; }
    if(m_seen.count(op.id)||m_cancel_ids.count(op.id)||(m_queued&&m_queued->id==op.id)) {
      report(op,"DUPLICATE","id_already_seen");return;
    }
    if(controlRequestSeen(m_services,op.id)||(m_replay&&m_replay->seen(op.id))) { report(op,"DUPLICATE","native_id_already_recorded");return; }
    const double now=m_services.now();
    if(!recentMail(mail,now,2)) { report(op,"REJECTED","stale_or_future_request");return; }
    if(mail.published_at<=m_boundary) { report(op,"REJECTED","pre_session_request");return; }
    op.published=mail.published_at;
    op.queued=mail.received_at;
    op.expires=op.published+timeout;
    if(now>=op.expires) { report(op,"REJECTED","original_operation_deadline");return; }
    expire();
    if(op.action=="cancel") { cancel(op);return; }
    if(m_pending||m_queued||m_helm_handoff||m_services.controlBusy()) { report(op,"REJECTED","busy");return; }
    if(m_seen.size()>=history_limit) { report(op,"REJECTED","request_history_full");return; }
    m_queued=op;
  }
  void onCommandSent(const CommandRequest& request,double at,bool accepted) override
  {
    if(!m_pending||m_pending->guided()||m_pending->sent_at)return;
    auto& op=*m_pending;
    const auto& expected=op.request;
    if(request.id!=expected.id||request.command!=expected.command||request.params!=expected.params||
       request.expires!=expected.expires||request.operation_expires!=expected.operation_expires||
       request.execution_id!=expected.execution_id||request.abort_id!=expected.abort_id)return;
    if(!accepted) { finish("FAILED","local_send_rejected_or_failed");return; }
    const double now=m_services.now();
    if(!std::isfinite(at)||!std::isfinite(now)||at<op.submitted||at>now||now>=request.expires) {
      finish("TIMED_OUT","local_send_outside_original_deadline",true);return;
    }
    op.sent_at=at;
    report(op,"SENT","local_os_acceptance");
  }
  void onMessage(const mavlink_message_t& message,double at) override
  { onMessageWithMetadata(message,{at,at,"READ_ACQUISITION"}); }
  void onMessageWithMetadata(const mavlink_message_t& message,const MessageTime& time) override
  {
    expire();
    const auto& v=m_services.vehicle();
    if(message.sysid==1&&message.compid==1&&message.msgid==MAVLINK_MSG_ID_HEARTBEAT&&
       v.mode==rover::manual_mode&&fresh(v.heartbeat_valid,v.heartbeat_at,m_services.now(),2)) {
      if(m_queued) {
        retireManualId(*m_queued);
        cancelQueued("pilot_manual_takeover");
      }
      if(m_pending)finish("CANCELLED","pilot_manual_takeover");
      return;
    }
    if(!m_pending)return;
    if(m_pending->guided()) { directContinuity();return; }
    if(message.sysid!=1||message.compid!=1)return;
    if(message.msgid==MAVLINK_MSG_ID_HEARTBEAT) { observeNative();return; }
    if(!m_pending->sent_at||message.msgid!=MAVLINK_MSG_ID_COMMAND_ACK)return;
    const auto sent=*m_pending->sent_at;
    if(!std::isfinite(time.effective_at)||!std::isfinite(time.read_at)||time.effective_at<=m_boundary||
       time.effective_at>time.read_at||time.read_at<=sent||time.read_at>m_services.now()||
       time.read_at>=sent+m_ack_timeout)return;
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&message,&ack);
    if(ack.command!=m_pending->request.command||(ack.target_system!=0&&ack.target_system!=245)||
       (ack.target_component!=0&&ack.target_component!=191))return;
    // COMMAND_ACK identifies the wire command, not the request or its parameters.
    // A delayed reply must never decide the outcome or extend the observation window.
    if(!m_pending->ack_observed) {
      m_pending->ack_observed=true;
      report(*m_pending,"ACK_OBSERVED","uncorrelated_ack_result_"+std::to_string(ack.result));
    }
  }
  void onReset(const std::string&) override
  {
    if(m_pending)finish("CANCELLED","connection_reset");
    if(m_queued)cancelQueued("connection_reset");
    m_helm_handoff.reset();
    m_boundary=m_services.now();
    m_helm_control_at=m_boundary;
  }
  void onInputBoundary() override
  {
    expire();
    if(!m_pending)return;
    if(m_pending->guided())directContinuity();
    else if(m_pending->action=="loiter") {
      const auto& v=m_services.vehicle();
      if(!m_backend.flightReady(v,m_services.now(),false))m_pending->settled_since.reset();
    }
  }
  void iterate() override
  {
    expire();
    advanceHelmHandoff();
    if(m_queued)startQueued();
    if(!m_pending)return;
    if(m_pending->guided())advanceDirect();
    else observeNative();
  }
private:
  bool parse(const Mail& mail,Operation& op,double& timeout) const
  {
    Fields fields;
    const bool well_formed=parseFields(mail,fields);
    const auto id=field(fields,"id"),action=field(fields,"action");
    if(idAllowed(id))op.id=id;
    static const std::set<std::string> names={"arm","disarm","guided","hold","loiter","rtl","return","goto","cancel"};
    if(names.count(action))op.action=action;
    if(!well_formed||!idAllowed(id))return false;
    if(!op.versioned) {
      if(!knownFields(fields,{"id","action"})||(action!="arm"&&action!="disarm"&&action!="guided"&&
         action!="hold"&&action!="rtl"&&!(m_boat&&action=="loiter"))) {
        op.action="invalid";return false;
      }
      timeout=nativeTimeout(op);
      return true;
    }
    if(field(fields,"schema")!="1")return false;
    if(action=="cancel") {
      op.strategy="cancel";
      op.cancel_target=field(fields,"target");
      timeout=2;
      return knownFields(fields,{"schema","id","action","target"})&&idAllowed(op.cancel_target);
    }
    const auto strategy=field(fields,"strategy");
    if(strategy!="native"&&strategy!="guided")return false;
    op.strategy=strategy;
    if(!op.guided()) {
      if(!knownFields(fields,{"schema","id","action","strategy","timeout"})||
         (action!="arm"&&action!="disarm"&&action!="guided"&&action!="hold"&&action!="return"&&
          !(m_boat&&action=="loiter")))return false;
      timeout=nativeTimeout(op);
    } else {
      if(action=="goto") {
        if(!knownFields(fields,{"schema","id","action","strategy","lat","lon","speed","timeout"}))return false;
        const auto lat=number(field(fields,"lat"),-90,90),lon=number(field(fields,"lon"),-180,180);
        if(!lat||!lon)return false;
        op.goal={*lat,*lon};
      } else if(action=="return") {
        if(!knownFields(fields,{"schema","id","action","strategy","speed","timeout"}))return false;
      } else if(action!="hold"||!knownFields(fields,{"schema","id","action","strategy","timeout"}))return false;
      if(fields.count("speed")) {
        const auto speed=number(field(fields,"speed"),.2,rover::speedLimit(m_boat));
        if(!speed)return false;
        op.speed=*speed;
      }
      timeout=action=="hold"?8:60;
    }
    if(fields.count("timeout")) {
      const auto requested=number(field(fields,"timeout"),1,120);
      if(!requested)return false;
      timeout=*requested;
    }
    return true;
  }
  double nativeTimeout(const Operation& op) const
  {
    if(op.returning())return m_return_timeout;
    if(op.action=="loiter")return 20;
    // Neutral thrust in native Hold leaves a boat to coast under hull drag.
    if(m_boat&&op.action=="hold")return 25;
    return 8;
  }
  void helmRequest(const Mail& mail)
  {
    if(!m_helm||mail.numeric||(mail.text!="acquire"&&mail.text!="release"))return;
    const double now=m_services.now();
    if(!recentMail(mail,now,m_helm_freshness)||now-mail.published_at>m_helm_freshness||
       mail.published_at<m_boundary||mail.published_at<=m_helm_control_at)return;
    const bool active_direct=m_pending&&m_pending->guided();
    const bool queued_direct=m_queued&&m_queued->guided();
    if((active_direct&&mail.published_at<=m_pending->submitted)||
       (queued_direct&&mail.published_at<=m_queued->published))return;
    m_helm_control_at=mail.published_at;
    if(m_helm_handoff) {
      if(mail.text=="release")m_helm_handoff.reset();
      else m_helm_handoff->request=mail;
      return;
    }
    if(active_direct) {
      finish("CANCELLED",mail.text=="release"?"operator_release":"helm_takeover");
      if(mail.text=="acquire")m_helm_handoff=HelmHandoff{mail};
      return;
    }
    if(queued_direct)cancelQueued(mail.text=="release"?"operator_release":"helm_takeover");
    if(mail.text=="acquire"&&(m_pending||m_services.controlBusy()))return;
    m_helm->onMail(mail);
  }
  void advanceHelmHandoff()
  {
    if(!m_helm_handoff)return;
    // Host egress follows component iteration. Leave one full tick for its
    // conditional zero before a fresh grant can clear that pending stop.
    if(!m_helm_handoff->stop_window_opened) {
      m_helm_handoff->stop_window_opened=true;return;
    }
    const auto request=m_helm_handoff->request;
    m_helm_handoff.reset();
    if(!m_services.controlBusy())m_helm->onMail(request);
  }
  void cancel(const Operation& request)
  {
    if(m_pending&&m_pending->id==request.cancel_target) {
      if(request.published<=m_pending->published) {
        report(request,"REJECTED","pre_operation_cancel");return;
      }
      const bool native=!m_pending->guided();
      finish("CANCELLED",native?"operator_cancel_effect_may_persist":"operator_cancel",native);
    } else if(m_queued&&m_queued->id==request.cancel_target) {
      if(request.published<=m_queued->published) {
        report(request,"REJECTED","pre_operation_cancel");return;
      }
      cancelQueued("operator_cancel");
    } else {
      report(request,"REJECTED","no_matching_operation");return;
    }
    // Each successful cancellation consumes a different operation in the bounded history.
    m_cancel_ids.insert(request.id);
    report(request,"COMPLETED","cancelled_matching_operation");
  }
  void cancelQueued(const std::string& reason)
  {
    m_seen.insert(m_queued->id);
    report(*m_queued,"CANCELLED",reason);
    m_queued.reset();
  }
  void startQueued()
  {
    auto op=*m_queued;
    m_queued.reset();
    const double now=m_services.now();
    if(!std::isfinite(now)||now<op.queued||now-op.queued>2) { report(op,"REJECTED","queued_request_expired");return; }
    if(now>=op.expires) { report(op,"REJECTED","original_operation_deadline");return; }
    if(m_services.controlBusy()) { report(op,"REJECTED","busy");return; }
    op.submitted=op.checked_at=now;
    if(op.guided())startDirect(op,now);
    else startNative(op,now);
  }
  void startDirect(Operation& op,double now)
  {
    const auto& v=m_services.vehicle();
    if(m_shadow) { report(op,"REJECTED","guided_requires_control_mode");return; }
    if(!directReady(v,now)) { report(op,"REJECTED","armed_guided_horizontal_state_required");return; }
    if(op.returning()) {
      if(!v.home_valid||!coordinate(v.home_latitude,v.home_longitude)) {
        report(op,"REJECTED","armed_home_required");return;
      }
      op.goal={v.home_latitude,v.home_longitude};
    }
    if(op.action!="hold"&&distance({v.latitude,v.longitude},op.goal)>100) {
      report(op,"REJECTED","goal_outside_100m_envelope");return;
    }
    const auto replay=reserveControlRequest(m_services,op.id);
    if(replay!=ReplayResult::Reserved) {report(op,replayRejection(replay),replayReason(replay));return;}
    m_seen.insert(op.id);
    report(op,"ADMITTED","rover_policy_passed");
    if(!m_services.acquireNavigation("rover-direct-"+op.id)) {
      report(op,"REJECTED","host_navigation_inhibited");return;
    }
    // Host acquisition invalidated the prior authority revision. Clear the
    // displaced producer's local decisions without touching this new grant.
    if(m_helm)m_helm->onReset("rover_direct_operation");
    m_pending=op;
    report(op,"SUBMITTED","host_navigation_granted");
  }
  void retireManualId(const Operation& op)
  {
    if(op.guided())return;
    m_seen.insert(op.id);
    // A retry loop must not revive an ID rejected during Manual when the
    // pilot subsequently selects Hold, even across bridge replacement.
    if(m_shadow)return;
    if(m_replay)m_replay->reserve(op.id);
    else reserveControlRequest(m_services,op.id);
  }
  void startNative(Operation& op,double now)
  {
    const auto& v=m_services.vehicle();
    if(!fresh(v.heartbeat_valid,v.heartbeat_at,now,2)) { report(op,"REJECTED","heartbeat_unavailable");return; }
    if(v.mode==rover::manual_mode) {
      retireManualId(op);
      report(op,"REJECTED","pilot_manual");return;
    }
    if(op.action=="arm"&&v.armed) { report(op,"REJECTED","already_armed");return; }
    if(op.action=="disarm"&&!v.armed) { report(op,"REJECTED","already_disarmed");return; }
    if(op.returning()&&(!v.armed||!v.home_valid)) { report(op,"REJECTED","armed_home_required");return; }
    CommandRequest request;
    request.id=op.id;
    if(op.action=="arm"||op.action=="disarm") {
      request.command=MAV_CMD_COMPONENT_ARM_DISARM;
      request.params[0]=op.action=="arm"?1.f:0.f;
    } else if(op.returning())request.command=MAV_CMD_NAV_RETURN_TO_LAUNCH;
    else {
      request.command=MAV_CMD_DO_SET_MODE;
      request.params[0]=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
      request.params[1]=nativeMode(op);
    }
    if(!m_backend.commandAllowed(request,v,now)) { report(op,"REJECTED","rover_policy_inhibited");return; }
    op.initial_mode=v.mode;
    op.goal={v.home_latitude,v.home_longitude};
    request.expires=std::min(now+m_ack_timeout,op.expires);
    request.operation_expires=op.expires;
    op.request=request;
    if(m_shadow) { report(op,"REJECTED","host_send_inhibited");return; }
    const auto* host_replay=dynamic_cast<const ControlReplayService*>(&m_services);
    if(host_replay && host_replay->controlReplayEnabled()) {
      const auto replay=reserveControlRequest(m_services,op.id);
      if(replay!=ReplayResult::Reserved) {report(op,replayRejection(replay),replayReason(replay));return;}
    } else {
      if(!m_replay) { report(op,"REJECTED","native_replay_directory_required");return; }
      using Replay=rover::NativeReplay::Result;
      const auto reserved=m_replay->reserve(op.id);
      if(reserved==Replay::Seen) { report(op,"DUPLICATE","native_id_already_recorded");return; }
      if(reserved!=Replay::Reserved) {
        report(op,"REJECTED",reserved==Replay::Full?"native_replay_history_full":"native_replay_unavailable");return;
      }
    }
    m_seen.insert(op.id);
    report(op,"ADMITTED","rover_policy_passed");
    if(request.command!=MAV_CMD_COMPONENT_ARM_DISARM&&v.mode==nativeMode(op)) {
      m_services.revokeNavigation("native_mode_observation");
      op.observe_only=true;
      m_pending=op;
      report(op,"OBSERVING","native_mode_already_active");
      return;
    }
    if(!m_services.sendCommand(request)) { report(op,"REJECTED","host_send_inhibited");return; }
    m_pending=op;
    report(op,"SUBMITTED","host_queue_admitted");
  }
  bool sameHome(const Operation& op,const Vehicle& v) const
  {
    return v.home_valid&&coordinate(v.home_latitude,v.home_longitude)&&
      distance({v.home_latitude,v.home_longitude},op.goal)<=.1;
  }
  bool directReady(const Vehicle& v,double now) const
  {
    return m_backend.flightReady(v,now,true)&&std::isfinite(v.north_velocity)&&std::isfinite(v.east_velocity);
  }
  bool directContinuity()
  {
    const auto& v=m_services.vehicle();
    const double now=m_services.now();
    if(fresh(v.heartbeat_valid,v.heartbeat_at,now,2)&&v.mode!=rover::guided_mode)
      finish("CANCELLED","external_mode_change");
    else if(!directReady(v,now))finish("CANCELLED","armed_guided_horizontal_state_lost");
    else if(m_pending->returning()&&!sameHome(*m_pending,v))finish("CANCELLED","home_reference_changed");
    else if(!m_services.navigationGranted())finish("CANCELLED","navigation_authority_lost");
    else return true;
    return false;
  }
  void advanceDirect()
  {
    if(!directContinuity())return;
    auto& op=*m_pending;
    const auto& v=m_services.vehicle();
    const double now=m_services.now();
    const Waypoint current{v.latitude,v.longitude};
    const bool hold=op.action=="hold";
    const double remaining=hold?0:distance(current,op.goal);
    const double speed=std::hypot(v.north_velocity,v.east_velocity);
    if(op.first_target_at&&v.position_at>*op.first_target_at&&v.position_at>op.position_at) {
      op.position_at=v.position_at;
      if(!op.confirmed_at) {
        op.confirmed_at=v.position_at;
        report(op,"CONFIRMED","post_target_admission_observation");
      }
      progress(op,remaining,speed);
      if(remaining<=arrival_radius&&std::isfinite(speed)&&speed<=stopped_speed) {
        if(!op.settled_since)op.settled_since=v.position_at;
        if(v.position_at-*op.settled_since>=settled_dwell) {
          finish("COMPLETED","observed_outcome");return;
        }
      } else op.settled_since.reset();
    }
    if(op.last_target_at&&now-*op.last_target_at<.1)return;
    NavigationTarget target;
    target.dimensions=NavigationDimensions::Planar;
    target.course=hold?v.heading:course(current,op.goal);
    target.speed=remaining<=arrival_radius?0:std::min(op.speed,std::max(.35,(remaining-1.5)*.45));
    target.expires=std::min(op.expires,now+.5);
    if(!m_services.navigation(target)) { finish("CANCELLED","host_navigation_rejected");return; }
    if(!op.first_target_at)op.first_target_at=now;
    op.last_target_at=now;
  }
  void observeNative()
  {
    const double now=m_services.now();
    const auto& v=m_services.vehicle();
    auto& op=*m_pending;
    if(op.returning()&&!sameHome(op,v)) {
      finish("CANCELLED","home_reference_changed",op.versioned&&op.sent_at.has_value());return;
    }
    const bool heartbeat=fresh(v.heartbeat_valid,v.heartbeat_at,now,2);
    if(!op.sent_at&&!op.observe_only) {
      const bool requested_mode=op.request.command!=MAV_CMD_COMPONENT_ARM_DISARM&&v.mode==nativeMode(op);
      if(heartbeat&&v.mode!=op.initial_mode&&!requested_mode)
        finish("CANCELLED","mode_changed_before_handoff");
      return;
    }
    const auto after=op.observe_only?op.submitted:*op.sent_at;
    const bool hb=heartbeat&&v.heartbeat_at>after;
    if(hb&&op.observe_only&&v.mode!=op.initial_mode) {
      finish("CANCELLED","native_mode_changed_during_observation");return;
    }
    const auto mode_at=op.confirmed_at.value_or(v.heartbeat_at);
    const bool pose=fresh(v.position_valid,v.position_at,now,pose_limit)&&
      v.position_at>after&&v.position_at>=mode_at;
    const double speed=std::hypot(v.north_velocity,v.east_velocity);
    bool confirmed=false,complete=false;
    if(op.action=="arm")confirmed=complete=hb&&v.armed;
    else if(op.action=="disarm")confirmed=complete=hb&&!v.armed;
    else if(op.action=="guided") {
      if(hb&&v.mode!=rover::guided_mode&&v.mode!=op.initial_mode) {
        finish("CANCELLED","guided_interrupted_by_mode");return;
      }
      confirmed=complete=hb&&v.mode==rover::guided_mode;
    }
    else {
      const unsigned mode=nativeMode(op);
      confirmed=hb&&v.mode==mode;
      if(hb&&!confirmed&&(op.confirmed_at||v.mode!=op.initial_mode)) {
        finish("CANCELLED",(op.returning()?"return":op.action)+std::string("_interrupted_by_mode"));return;
      }
      const Waypoint current{v.latitude,v.longitude};
      const double remaining=op.returning()?distance(current,op.goal):0;
      const bool stopped=confirmed&&pose&&std::isfinite(speed)&&speed<=stopped_speed;
      if(op.action=="loiter") {
        if(hb&&!v.armed) { finish("CANCELLED","loiter_disarmed");return; }
        if(!stopped||!m_backend.flightReady(v,now,false))op.settled_since.reset();
        else if(v.position_at>op.position_at) {
          if(!op.settled_since||distance(current,op.settled_position)>3) {
            op.settled_since=v.position_at;
            op.settled_position=current;
          }
          complete=v.position_at-*op.settled_since>=settled_dwell;
        }
      } else complete=stopped&&remaining<=arrival_radius;
      if(confirmed&&pose&&v.position_at>op.position_at) {
        op.position_at=v.position_at;
        if(op.versioned&&std::isfinite(speed))progress(op,remaining,speed);
      }
    }
    if(confirmed&&!op.confirmed_at) {
      op.confirmed_at=v.heartbeat_at;
      report(op,"CONFIRMED",op.observe_only?"post_request_observation":"post_handoff_observation");
    }
    if(complete)finish("COMPLETED","observed_outcome");
  }
  void expire()
  {
    const double now=m_services.now();
    // An old acquire may not change a newly observed pilot mode back to Guided.
    if(m_helm_handoff&&(!directReady(m_services.vehicle(),now)||now<m_helm_handoff->request.received_at))
      m_helm_handoff.reset();
    if(!m_pending)return;
    auto& op=*m_pending;
    if(!std::isfinite(now)||now<op.checked_at) {
      finish("FAILED","clock_discontinuity",op.sent_at.has_value());return;
    }
    op.checked_at=now;
    if(now>=op.expires) { finish("TIMED_OUT","operation_deadline");return; }
    if(op.guided()||op.observe_only)return;
    if(!op.sent_at) {
      if(now>=op.request.expires)finish("TIMED_OUT","local_send_receipt_missing",op.versioned);
    } else if(!op.confirmed_at&&now>=*op.sent_at+m_ack_timeout)
      finish("TIMED_OUT","command_observation_unavailable",op.versioned);
  }
  void report(const Operation& op,const std::string& status,const std::string& reason)
  {
    std::ostringstream out;
    out<<"schema=1,profile="<<m_backend.profile()<<",id="<<op.id<<",action="<<op.action;
    if(op.versioned)out<<",strategy="<<op.strategy;
    out<<",status="<<status<<",reason="<<reason<<std::fixed<<std::setprecision(9);
    if(op.versioned)out<<",expires="<<op.expires;
    out<<",at="<<m_services.now();
    m_services.publish(op.versioned?"MAVMOOS_CONTROL_RESULT":"MAVMOOS_COMMAND_RESULT",out.str());
  }
  void progress(const Operation& op,double remaining,double speed)
  {
    std::ostringstream out;
    out<<"schema=1,profile="<<m_backend.profile()<<",id="<<op.id<<",action="<<op.action
       <<",strategy="<<op.strategy<<std::fixed<<std::setprecision(9)
       <<",distance_m="<<remaining<<",speed_mps="<<speed
       <<",position_at="<<op.position_at<<",expires="<<op.expires<<",at="<<m_services.now();
    m_services.publish("MAVMOOS_CONTROL_PROGRESS",out.str());
  }
  void finish(const std::string& status,const std::string& reason,bool retain_native_lease=false)
  {
    const auto op=*m_pending;
    m_pending.reset();
    if(op.guided()) {
      if(m_services.navigationGranted())m_services.revokeNavigation("rover_direct:"+reason);
    } else if(!op.observe_only) {
      if(retain_native_lease)m_services.cancelCommand(op.id);
      else m_services.commandFinished(op.id);
    }
    report(op,status,reason);
  }
  Services& m_services;
  const VehicleControlBackend& m_backend;
  Plugin* m_helm;
  std::optional<Operation> m_queued,m_pending;
  std::optional<HelmHandoff> m_helm_handoff;
  std::set<std::string> m_seen,m_cancel_ids;
  std::unique_ptr<rover::NativeReplay> m_replay;
  double m_boundary=0,m_ack_timeout=4,m_return_timeout=90,m_helm_freshness=2,m_helm_control_at=0;
  const bool m_boat;
  bool m_shadow=false;
};
} // namespace
std::unique_ptr<Plugin> makeRoverCommands(Services& services,const VehicleControlBackend& backend,Plugin* helm)
{ return std::make_unique<RoverCommands>(services,backend,helm); }
} // namespace mavmoos::runtime
