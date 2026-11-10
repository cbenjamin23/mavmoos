/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ArduPlaneControlPlugin.cpp                       */
/*    DATE: November 6, 2026                                */
/************************************************************/
#include "ControlReplayService.h"
#include "PlaneComponents.h"
#include "PlaneHelmTasks.h"
#include "ControlComponents.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace mavmoos::runtime {
namespace {
constexpr std::size_t history_limit=128;
constexpr double admission_age_limit=2,operation_lifetime=4;
constexpr double heartbeat_age_limit=2;

std::string trim(const std::string& value)
{
  const auto first=value.find_first_not_of(" \t\r\n");
  return first==std::string::npos?"":value.substr(first,value.find_last_not_of(" \t\r\n")-first+1);
}

bool validId(const std::string& id)
{
  return !id.empty()&&id.size()<=48&&std::all_of(id.begin(),id.end(),[](unsigned char c) {
    return (c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='-'||c=='_'||c=='.';
  });
}

struct ModeOperation {
  std::string id,action;
  unsigned mode=0,origin=0;
  double delta=0;
  CommandRequest request;
  double submitted_at=0;
  std::optional<double> sent_at;
  std::optional<double> speed_seen_at;
  bool speed_acknowledged=false;
  bool submitted=false;
};

using Fields=std::map<std::string,std::string>;

struct DirectOperation {
  std::string id,action;
  NavigationTarget target;
  double deadline=0,first_offered=0,last_readback=0,last_position=0;
  double previous_bearing=0,orbit_sweep=0,starting_height=0;
  bool requested=false,tracking=false,completed=false,orbit_sample=false;
};

enum class FlightPhase { Select, Start, Execute, Disarm, Handback };
struct FlightOperation {
  std::string id,action,content;
  uint64_t revision=0;
  double deadline=0,wire_at=0,started_at=0,takeoff_height=0;
  FlightPhase phase=FlightPhase::Select;
  std::optional<CommandRequest> wire;
  bool admitted=false;
};

std::optional<Fields> fields(const Mail& mail)
{
  if(mail.numeric||mail.text.empty()||mail.text.size()>512||mail.text.back()==',')return std::nullopt;
  Fields result;
  std::istringstream input(mail.text);
  std::string part;
  while(std::getline(input,part,',')) {
    const auto equal=part.find('=');
    if(equal==std::string::npos||!result.emplace(trim(part.substr(0,equal)),trim(part.substr(equal+1))).second)
      return std::nullopt;
  }
  return result;
}

bool number(const std::string& value,double& result)
{
  try {
    std::size_t used=0;
    result=std::stod(value,&used);
    return used==value.size()&&std::isfinite(result);
  } catch(...) { return false; }
}

std::optional<ModeOperation> parse(const Mail& mail)
{
  auto values=fields(mail);
  if(!values||!values->count("id")||!values->count("action")||!validId(values->at("id")))return std::nullopt;
  ModeOperation operation;
  operation.id=values->at("id");
  operation.action=values->at("action");
  if(operation.action=="loiter"||operation.action=="rtl"||operation.action=="manual")
    operation.mode=operation.action=="loiter"?plane::loiter_mode:operation.action=="rtl"?plane::rtl_mode:plane::manual_mode;
  else if(operation.action=="arm"||operation.action=="disarm")
    operation.request.command=MAV_CMD_COMPONENT_ARM_DISARM;
  else if(operation.action=="change_speed"||operation.action=="reset_speed_min")
    operation.request.command=MAV_CMD_GUIDED_CHANGE_SPEED;
  else return std::nullopt;
  if(operation.action=="change_speed") {
    if(values->size()!=3||!values->count("delta")||!number(values->at("delta"),operation.delta)||
       operation.delta==0||std::abs(operation.delta)>8)return std::nullopt;
  } else if(values->size()!=2)return std::nullopt;
  return operation;
}

bool enabled(const std::string& key,const std::string& value)
{
  if(value!="true"&&value!="false")throw std::invalid_argument(key+" must be true/false");
  return value=="true";
}

class ArduPlaneControl final:public Plugin {
public:
  explicit ArduPlaneControl(Services& services)
    :m_services(services),m_backend(services.now()),m_tasks(services,m_backend,m_seen),
     m_helm(makeHelmAdapter(m_tasks.helmServices(),m_backend)),
     m_request_floor(services.now()) {}
  std::string name() const override { return "arduplane_control"; }
  const VehicleControlBackend* controlBackend() const override { return &m_backend; }

  PluginDescriptor descriptor() const override
  {
    PluginDescriptor descriptor;
    descriptor.profile=m_backend.profile();
    descriptor.exclusive_resources={"vehicle.control"};
    descriptor.request_owners={"MAVMOOS_COMMAND","MAVMOOS_NAV_CONTROL","MAVMOOS_CONTROL_REQUEST","MAVMOOS_MISSION_CONTROL"};
    descriptor.observation_streams={{MAVLINK_MSG_ID_GLOBAL_POSITION_INT,50000.f},{MAVLINK_MSG_ID_SYS_STATUS,500000.f},
      {MAVLINK_MSG_ID_EKF_STATUS_REPORT,250000.f},{MAVLINK_MSG_ID_HOME_POSITION,500000.f},
      {MAVLINK_MSG_ID_EXTENDED_SYS_STATE,500000.f},{MAVLINK_MSG_ID_AIRSPEED,250000.f},
      {MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT,200000.f}};
    descriptor.capabilities.push_back(
      {"control.observation.fixed_wing_readiness","","","observation"});
    if(m_native_enabled) {
      descriptor.capabilities.push_back({"control.hold.native_loiter","MAVMOOS_COMMAND","action=loiter"});
      descriptor.capabilities.push_back({"control.return.native_rtl","MAVMOOS_COMMAND","action=rtl"});
      descriptor.capabilities.push_back({"control.arm","MAVMOOS_COMMAND","action=arm"});
      descriptor.capabilities.push_back({"control.disarm","MAVMOOS_COMMAND","action=disarm"});
      descriptor.capabilities.push_back({"control.ground.manual","MAVMOOS_COMMAND","action=manual"});
      descriptor.capabilities.push_back({"control.guided.speed_adjustment","MAVMOOS_COMMAND","action=change_speed"});
      descriptor.capabilities.push_back({"control.guided.speed_reset","MAVMOOS_COMMAND","action=reset_speed_min"});
      descriptor.capabilities.push_back({"control.navigation.direct","MAVMOOS_CONTROL_REQUEST","action=goto,strategy=guided"});
      descriptor.capabilities.push_back({"control.hold.guided_orbit","MAVMOOS_CONTROL_REQUEST","action=hold,strategy=guided"});
      descriptor.capabilities.push_back({"control.guided.relative_adjustment","MAVMOOS_CONTROL_REQUEST",
        "action=adjust_course|adjust_altitude,strategy=guided,delta=<degrees|metres>"});
      descriptor.capabilities.push_back({"control.observation.home_marker","MAVMOOS_CONTROL_REQUEST","action=visualize_home"});
      descriptor.capabilities.push_back({"control.operation.cancel","MAVMOOS_CONTROL_REQUEST","action=cancel"});
      if(m_mission_enabled) {
        descriptor.capabilities.push_back({"control.mission.start","MAVMOOS_MISSION_CONTROL","schema=1,action=takeoff"});
        descriptor.capabilities.push_back({m_native_autoland?"control.land.native_autoland":"control.land.mission",
          "MAVMOOS_MISSION_CONTROL","schema=1,action=land"});
        descriptor.capabilities.push_back({"control.mission.release","MAVMOOS_MISSION_CONTROL","schema=1,action=release"});
        descriptor.observation_streams.push_back({MAVLINK_MSG_ID_MISSION_CURRENT,500000.f});
      }
    }
    if(m_helm_enabled) {
      descriptor.capabilities.push_back({"control.navigation.helm","MAVMOOS_NAV_CONTROL","acquire|release"});
      descriptor.capabilities.push_back({"control.hold.native_handback","","","internal"});
    }
    const auto task_capabilities=m_tasks.capabilities();
    descriptor.capabilities.insert(descriptor.capabilities.end(),task_capabilities.begin(),task_capabilities.end());
    return descriptor;
  }

  std::vector<std::string> subscriptions() const override
  {
    auto result=m_helm_enabled?m_helm->subscriptions():std::vector<std::string>{};
    if(m_helm_enabled) {
      result.push_back("MAVMOOS_CONTROL_REQUEST");
      result.push_back("MAVMOOS_HELM_TASK_READY");
    }
    if(m_native_enabled) {
      result.push_back("MAVMOOS_COMMAND");
      if(!m_helm_enabled)result.push_back("MAVMOOS_CONTROL_REQUEST");
      if(m_mission_enabled)result.push_back("MAVMOOS_MISSION_CONTROL");
    }
    return result;
  }

  void configure(const std::string& key,const std::string& value) override
  {
    m_tasks.configure(key,value);
    if(key=="control_native_actions") { m_native_enabled=enabled(key,value);return; }
    if(key=="control_helm") { m_helm_enabled=enabled(key,value);return; }
    if(key=="control_mission_execution") { m_mission_enabled=enabled(key,value);return; }
    if(key=="plane_native_autoland") { m_native_autoland=enabled(key,value);return; }
    if(key=="vehicle_profile"&&value!=m_backend.profile())
      throw std::invalid_argument("invalid ArduPlane control profile");
    if(key=="facing_policy"&&value!="course")
      throw std::invalid_argument("Plane profile follows ground course");
    if(key=="shadow_mode")m_shadow=enabled(key,value);
    m_helm->configure(key,value);
  }

  void onMail(const Mail& mail) override
  {
    if(mail.key=="MAVMOOS_COMMAND") {
      if(m_native_enabled)requestMode(mail);
      return;
    }
    if(mail.key=="MAVMOOS_CONTROL_REQUEST") {
      if(m_helm_enabled&&m_tasks.onRequest(mail,*m_helm,m_direct.has_value()||m_operation.has_value()||m_flight.has_value()))return;
      if(m_native_enabled)requestDirect(mail);
      return;
    }
    if(mail.key=="MAVMOOS_MISSION_CONTROL") {
      if(m_native_enabled&&m_mission_enabled)requestFlight(mail);
      return;
    }
    if(mail.key=="MAVMOOS_NAV_CONTROL"&&(!std::isfinite(mail.published_at)||mail.published_at>m_services.now()))return;
    if(m_helm_enabled&&m_tasks.onReadiness(mail))return;
    if(m_helm_enabled&&m_tasks.onNavControl(mail,*m_helm))return;
    if(m_direct&&mail.key=="MAVMOOS_NAV_CONTROL") {
      if(!mail.numeric&&mail.text=="release"&&liveMail(mail))finishDirect("CANCELLED","operator_release");
      return;
    }
    if(m_helm_enabled)m_helm->onMail(mail);
  }

  void onMessage(const mavlink_message_t& message,double at) override
  { onMessageWithMetadata(message,{at,at,"READ_ACQUISITION"}); }

  void onMessageWithMetadata(const mavlink_message_t& message,const MessageTime& time) override
  {
    m_backend.observe(message,time,m_services.vehicle(),m_services.now());
    if(message.sysid!=1||message.compid!=1)return;
    if(message.msgid==MAVLINK_MSG_ID_COMMAND_ACK)observeSpeedAck(message,time);
    if(message.msgid==MAVLINK_MSG_ID_MISSION_CURRENT&&time.effective_at>m_cursor_at&&
       plane::freshState(time.effective_at,m_services.now(),2)) {
      mavlink_mission_current_t cursor{};
      mavlink_msg_mission_current_decode(&message,&cursor);
      m_cursor=cursor.seq;m_cursor_at=time.effective_at;
    }
    if(message.msgid==MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT)observeDirectTarget(message,time);
    if(message.msgid==MAVLINK_MSG_ID_HEARTBEAT||message.msgid==MAVLINK_MSG_ID_EXTENDED_SYS_STATE||
       message.msgid==MAVLINK_MSG_ID_AIRSPEED)
      advanceMode();
  }

  void onCommandSent(const CommandRequest& request,double at,bool accepted) override
  {
    if(m_flight&&m_flight->wire&&request.id==m_flight->wire->id) {
      if(!accepted||!std::isfinite(at)||at<=0||at>=m_flight->wire->expires)
        finishFlight("FAILED","local_handoff_failed");
      else m_flight->wire_at=at;
      return;
    }
    if(!m_operation||!m_operation->submitted||m_operation->sent_at)return;
    const auto& expected=m_operation->request;
    if(request.id!=expected.id||request.command!=expected.command||request.params!=expected.params||
       request.expires!=expected.expires||request.operation_expires!=expected.operation_expires||
       !request.execution_id.empty()||!request.abort_id.empty())return;
    if(!accepted) { finish("FAILED","local_send_rejected_or_failed");return; }
    if(!std::isfinite(at)||at<m_operation->submitted_at||at>m_services.now()||at>=request.expires) {
      finish("TIMED_OUT","handoff_receipt_outside_deadline");return;
    }
    m_operation->sent_at=at;
    report(m_operation->id,m_operation->action,"SENT","local_os_acceptance");
  }

  void onReset(const std::string& reason) override
  {
    if(m_operation)finish("CANCELLED","connection_reset");
    if(m_direct)finishDirect("CANCELLED","connection_reset");
    if(m_flight)finishFlight("CANCELLED","connection_reset");
    m_release.reset();
    m_cursor_at=0;
    m_request_floor=std::max(m_request_floor,m_services.now());
    m_next_status=0;
    m_speed_target.reset();
    m_backend.reset(m_services.now());
    m_tasks.onReset(reason);
    if(m_helm_enabled)m_helm->onReset(reason);
  }

  void onInputBoundary() override
  {
    m_tasks.beforeHelm(*m_helm);
    if(m_helm_enabled)m_helm->onInputBoundary();
    advanceMode();
    checkDirect();
  }

  void iterate() override
  {
    advanceMode();
    if(m_operation&&!m_operation->submitted) {
      if(!operationReady(*m_operation)||m_services.controlBusy()||!m_services.sendCommand(m_operation->request))
        finish("REJECTED","host_mode_admission");
      else {
        m_operation->submitted_at=m_services.now();
        m_operation->submitted=true;
      }
    }
    m_tasks.beforeHelm(*m_helm);
    if(m_helm_enabled)m_helm->iterate();
    m_tasks.afterHelm(*m_helm);
    advanceDirect();
    advanceFlight();
    if(m_services.now()>=m_next_status)publishReadiness();
  }

private:
  struct Release {
    std::string id;
    double after=0,deadline=0;
  };

  bool liveMail(const Mail& mail) const
  {
    const auto now=m_services.now();
    return std::isfinite(mail.published_at)&&std::isfinite(mail.received_at)&&std::isfinite(now)&&
      mail.published_at>m_request_floor&&mail.published_at<=mail.received_at+.1&&
      now>=mail.received_at&&now-std::min(mail.published_at,mail.received_at)<admission_age_limit;
  }

  void requestFlight(const Mail& mail)
  {
    const auto values=fields(mail);
    if(!values||!values->count("schema")||values->at("schema")!="1"||!values->count("id")||
       !validId(values->at("id"))||!values->count("action")||!liveMail(mail)) {
      flightResult("invalid","invalid","REJECTED","invalid_or_stale_request");return;
    }
    const auto& id=values->at("id");
    const auto& action=values->at("action");
    if(m_shadow||m_flight||m_operation||m_release||m_seen.count(id)||m_seen.size()>=history_limit) {
      flightResult(id,action,"REJECTED","busy_or_duplicate");return;
    }
    if(action=="release") {
      if(values->size()!=4||!values->count("target")||values->at("target")!=m_retained||
         !releaseReady()) {
        flightResult(id,action,"REJECTED","independent_native_state_required");return;
      }
      const auto replay=reserveControlRequest(m_services,id);
      if(replay!=ReplayResult::Reserved) {flightResult(id,action,replayRejection(replay),replayReason(replay));return;}
      m_seen.insert(id);
      const auto now=m_services.now();
      if(!m_services.beginMissionExecutionAbort(id,m_retained,now+6)) {
        flightResult(id,action,"REJECTED","host_handback_denied");return;
      }
      m_release=Release{id,now,now+6};
      return;
    }
    double timeout=120;
    const bool timed=values->count("timeout");
    const auto proof=m_services.storedMissionProof();
    const auto& v=m_services.vehicle();
    const bool takeoff=action=="takeoff";
    const auto now=m_services.now();
    if((!takeoff&&action!="land")||values->size()!=6+std::size_t(timed)||
       !values->count("verification_id")||!values->count("revision")||!values->count("content_id")||
       (timed&&(!number(values->at("timeout"),timeout)||timeout<20||timeout>120))||
       !proof.valid||proof.verification_id!=values->at("verification_id")||
       std::to_string(proof.revision)!=values->at("revision")||proof.content_id!=values->at("content_id")||
       !plane::lifecyclePlan(proof.items,v)||!v.armed||
       (takeoff?(v.mode!=plane::manual_mode||!m_backend.groundReady(v,now)):
         (!m_backend.flightReady(v,now,false)||(v.mode!=plane::loiter_mode&&
           (v.mode!=plane::guided_mode||!m_services.navigationGranted()))))) {
      flightResult(id,action,"REJECTED","verified_plan_and_eligible_vehicle_required");return;
    }
    FlightOperation operation;
    operation.id=id;operation.action=action;operation.content=proof.content_id;
    operation.revision=proof.revision;
    operation.deadline=std::min(mail.published_at,mail.received_at)+timeout;
    operation.takeoff_height=proof.items[plane::takeoff_item].z;
    operation.phase=!takeoff&&m_native_autoland?FlightPhase::Start:FlightPhase::Select;
    const auto replay=reserveControlRequest(m_services,id);
    if(replay!=ReplayResult::Reserved) {flightResult(id,action,replayRejection(replay),replayReason(replay));return;}
    m_seen.insert(id);
    m_flight=std::move(operation);
    flightResult(id,action,"QUEUED","verified_plan_requested");
  }

  bool releaseReady() const
  {
    const auto now=m_services.now();
    const auto& v=m_services.vehicle();
    return (!v.armed&&v.mode==plane::manual_mode&&m_backend.groundReady(v,now))||
      ((v.mode==plane::loiter_mode||v.mode==plane::rtl_mode)&&m_backend.flightReady(v,now,false));
  }

  bool cursorIs(unsigned sequence,double after) const
  { return m_cursor==sequence&&m_cursor_at>after&&plane::freshState(m_cursor_at,m_services.now(),2); }

  void advanceFlight()
  {
    const auto now=m_services.now();
    const auto& v=m_services.vehicle();
    if(m_release) {
      if(now>=m_release->deadline) {
        flightResult(m_release->id,"release","TIMED_OUT","native_state_unconfirmed");m_release.reset();
      } else if(releaseReady()&&v.heartbeat_at>m_release->after&&m_services.finishMissionExecution(m_retained,"RELEASED")) {
        flightResult(m_release->id,"release","COMPLETED","observed_native_handback");
        m_release.reset();m_retained.clear();
      }
    }
    if(!m_flight)return;
    auto& operation=*m_flight;
    if(now>=operation.deadline) { finishFlight("TIMED_OUT","original_flight_deadline");return; }
    if(!operation.admitted) {
      if(!m_services.beginMissionExecution(operation.id,operation.revision,operation.content,operation.deadline)) {
        finishFlight("REJECTED","host_mission_admission");return;
      }
      operation.admitted=true;
    }
    if(!m_services.missionExecutionGranted(operation.id)) {
      finishFlight("CANCELLED","host_execution_fenced");return;
    }
    const bool takeoff=operation.action=="takeoff";
    const auto execution_mode=!takeoff&&m_native_autoland?plane::autoland_mode:plane::auto_mode;
    if(operation.phase==FlightPhase::Execute) {
      if(takeoff) {
        if(v.mode!=plane::auto_mode||!cursorIs(plane::takeoff_hold_item,operation.started_at)||
           !m_backend.flightReady(v,now,false)||v.relative_height<operation.takeoff_height-8)return;
        operation.phase=FlightPhase::Handback;
      } else {
        if(v.mode!=execution_mode||(!m_native_autoland&&!cursorIs(plane::landing_item,operation.started_at))||
           !m_backend.groundReady(v,now)||v.landed_at<=operation.started_at)return;
        operation.phase=v.armed?FlightPhase::Disarm:FlightPhase::Handback;
      }
    }
    if(!operation.wire) {
      queueFlightWire(takeoff);
      return;
    }
    if(operation.wire_at<=0)return;
    if(operation.phase==FlightPhase::Select&&
       cursorIs(takeoff?plane::takeoff_item:plane::approach_entry_item,operation.wire_at))
      nextFlightPhase(FlightPhase::Start);
    else if(operation.phase==FlightPhase::Start&&v.mode==execution_mode&&v.armed&&
            v.heartbeat_at>operation.wire_at&&plane::freshState(v.heartbeat_at,now,2)) {
      operation.started_at=operation.wire_at;
      nextFlightPhase(FlightPhase::Execute);
      flightResult(operation.id,operation.action,"CONFIRMED",execution_mode==plane::autoland_mode?
        "observed_native_autoland":"observed_native_auto");
    } else if(operation.phase==FlightPhase::Disarm&&!v.armed&&v.heartbeat_at>operation.wire_at&&
              m_backend.groundReady(v,now))
      nextFlightPhase(FlightPhase::Handback);
    else if(operation.phase==FlightPhase::Handback&&v.heartbeat_at>operation.wire_at&&
            (takeoff?(v.mode==plane::loiter_mode&&m_backend.flightReady(v,now,false)):
              (v.mode==plane::manual_mode&&!v.armed&&v.landed_at>operation.wire_at&&m_backend.groundReady(v,now)))) {
      if(m_services.finishMissionExecution(operation.id,"COMPLETED"))
        finishFlight("COMPLETED",takeoff?"observed_climb_and_loiter":"observed_touchdown_disarm_and_manual");
      else finishFlight("FAILED","host_handback_unconfirmed");
    } else if(now>=operation.wire_at+8)finishFlight("TIMED_OUT","post_handoff_effect_unconfirmed");
  }

  void queueFlightWire(bool takeoff)
  {
    auto& operation=*m_flight;
    CommandRequest request;
    request.id="plane-flight-"+operation.id+"-"+std::to_string(static_cast<unsigned>(operation.phase));
    request.execution_id=operation.id;
    request.expires=std::min(m_services.now()+1.5,operation.deadline);
    request.operation_expires=operation.deadline;
    switch(operation.phase) {
    case FlightPhase::Select:
      request.command=MAV_CMD_DO_SET_MISSION_CURRENT;
      request.params[0]=takeoff?plane::takeoff_item:plane::approach_entry_item;
      break;
    case FlightPhase::Start:
      if(!takeoff&&m_native_autoland) {
        request.command=MAV_CMD_DO_SET_MODE;
        request.params[0]=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        request.params[1]=plane::autoland_mode;
      } else request.command=MAV_CMD_MISSION_START;
      break;
    case FlightPhase::Disarm: request.command=MAV_CMD_COMPONENT_ARM_DISARM;break;
    case FlightPhase::Handback:
      request.command=MAV_CMD_DO_SET_MODE;
      request.params[0]=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
      request.params[1]=takeoff?plane::loiter_mode:plane::manual_mode;
      break;
    case FlightPhase::Execute:return;
    }
    if(!m_services.sendCommand(request)) { finishFlight("FAILED","host_phase_admission");return; }
    operation.wire=request;
  }

  void nextFlightPhase(FlightPhase phase)
  {
    auto& operation=*m_flight;
    m_services.commandFinished(operation.wire->id);
    operation.wire.reset();
    operation.wire_at=0;
    operation.phase=phase;
  }

  void finishFlight(const char* status,const char* reason)
  {
    const auto operation=std::move(*m_flight);
    m_flight.reset();
    if(operation.wire)m_services.commandFinished(operation.wire->id);
    if(operation.admitted&&std::string_view(status)!="COMPLETED") {
      if(!m_services.finishMissionExecution(operation.id,"REJECTED"))m_retained=operation.id;
    }
    flightResult(operation.id,operation.action,status,reason);
  }

  void flightResult(const std::string& id,const std::string& action,const char* status,const char* reason)
  {
    m_services.publish("MAVMOOS_MISSION_EXECUTION_RESULT","schema=1,profile=arduplane,id="+id+
      ",action="+action+",status="+status+",reason="+reason);
  }

  void requestDirect(const Mail& mail)
  {
    const auto values=fields(mail);
    if(!values||!values->count("schema")||values->at("schema")!="1"||
       !values->count("id")||!validId(values->at("id"))||!values->count("action")) {
      directResult("invalid","invalid","REJECTED","invalid_request");return;
    }
    const auto& id=values->at("id");
    const auto& action=values->at("action");
    if(!liveMail(mail)||m_seen.count(id)||m_seen.size()>=history_limit) {
      directResult(id,action,"REJECTED","stale_repeated_or_full");return;
    }
    if(action=="visualize_home") {
      if(values->size()!=3)directResult(id,action,"REJECTED","invalid_home_marker_request");
      else visualizeHome(id);
      return;
    }
    if(action=="cancel") {
      if(values->size()!=4||!values->count("target")||!m_direct||values->at("target")!=m_direct->id) {
        directResult(id,action,"REJECTED","unknown_direct_operation");return;
      }
      m_seen.insert(id);
      finishDirect("CANCELLED","explicit_cancel");
      directResult(id,action,"COMPLETED","local_ownership_released");
      return;
    }
    const auto& v=m_services.vehicle();
    const bool course_adjustment=action=="adjust_course",altitude_adjustment=action=="adjust_altitude";
    if(m_shadow||m_tasks.busy()||m_direct||m_operation||m_services.controlBusy()||m_services.navigationGranted()||
       v.mode!=plane::loiter_mode||!m_backend.flightReady(v,m_services.now(),false)||
       !values->count("strategy")||values->at("strategy")!="guided"||
       (action!="goto"&&action!="hold"&&!course_adjustment&&!altitude_adjustment)) {
      directResult(id,action,"REJECTED","idle_native_loiter_required");return;
    }
    DirectOperation operation;
    operation.id=id;
    operation.action=action;
    operation.target.kind=NavigationKind::GlobalPosition;
    operation.target.latitude=v.latitude;
    operation.target.longitude=v.longitude;
    operation.target.height=v.relative_height;
    operation.starting_height=v.relative_height;
    double timeout=60;
    std::size_t used=4;
    const auto optional=[&](const char* key,double& output) {
      const auto it=values->find(key);
      if(it==values->end())return true;
      ++used;
      return number(it->second,output);
    };
    if(course_adjustment||altitude_adjustment) {
      double delta=0;
      if(!optional("delta",delta)||std::abs(delta)<(course_adjustment?10:5)||
         std::abs(delta)>(course_adjustment?90:25)) {
        directResult(id,action,"REJECTED","relative_adjustment_outside_profile");return;
      }
      if(course_adjustment) {
        if(std::hypot(v.north_velocity,v.east_velocity)<12) {
          directResult(id,action,"REJECTED","measured_ground_track_required");return;
        }
        const double course=std::fmod(std::atan2(v.east_velocity,v.north_velocity)*180./std::acos(-1.)+delta+360.,360.);
        const auto target=m_backend.directCourseTarget(v,course);
        if(!target) {directResult(id,action,"REJECTED","adjusted_course_outside_profile");return;}
        operation.target=*target;
      } else operation.target.height+=delta;
    } else if(bool(values->count("lat"))!=bool(values->count("lon"))||
              (action=="goto"&&!values->count("lat"))||!optional("lat",operation.target.latitude)||
              !optional("lon",operation.target.longitude)||!optional("height",operation.target.height)) {
      directResult(id,action,"REJECTED","invalid_target_geometry");return;
    }
    if(!optional("timeout",timeout)||values->size()!=used||
       timeout<10||timeout>120||!m_backend.directTargetAllowed(operation.target,v)) {
      directResult(id,action,"REJECTED","invalid_geometry_or_timeout");return;
    }
    operation.deadline=std::min(mail.published_at,mail.received_at)+timeout;
    const auto replay=reserveControlRequest(m_services,id);
    if(replay!=ReplayResult::Reserved) {directResult(id,action,replayRejection(replay),replayReason(replay));return;}
    m_seen.insert(id);
    m_direct=std::move(operation);
    directResult(id,action,"QUEUED","owned_guided_operation");
  }

  void visualizeHome(const std::string& id)
  {
    const auto& v=m_services.vehicle();
    const auto now=m_services.now();
    double x=0,y=0;
    if(!v.heartbeat_valid||!v.home_valid||!plane::freshState(v.heartbeat_at,now,2)||
       !plane::freshState(v.home_at,now,3)||!m_services.projectGlobal(v.home_latitude,v.home_longitude,x,y)||
       !std::isfinite(x)||!std::isfinite(y)) {
      directResult(id,"visualize_home","REJECTED","fresh_projected_home_required");return;
    }
    m_seen.insert(id);
    std::ostringstream marker;
    marker<<std::setprecision(12)<<"x="<<x<<",y="<<y<<",label=MAVMOOS_HOME,type=gateway,width=5,active=true";
    m_services.publish("VIEW_MARKER",marker.str());
    directResult(id,"visualize_home","COMPLETED","fresh_home_marker_published");
  }

  void checkDirect()
  {
    if(!m_direct)return;
    const auto now=m_services.now();
    if(now>=m_direct->deadline)finishDirect("TIMED_OUT","original_operation_deadline");
    else if(m_direct->requested&&!m_services.navigationGranted())finishDirect("CANCELLED","guided_authority_lost");
  }

  void advanceDirect()
  {
    checkDirect();
    if(!m_direct)return;
    auto& operation=*m_direct;
    const auto now=m_services.now();
    const auto& v=m_services.vehicle();
    if(!operation.requested) {
      if(!m_services.acquireNavigation("plane-direct-"+operation.id)) {
        finishDirect("REJECTED","host_acquisition_denied");return;
      }
      operation.requested=true;
    }
    operation.target.expires=std::min(operation.deadline,now+1.5);
    if(v.mode!=plane::guided_mode)return;
    if(!m_services.navigation(operation.target)) { finishDirect("CANCELLED","target_admission_lost");return; }
    if(operation.first_offered==0)operation.first_offered=now;
    if(!plane::freshState(operation.last_readback,now,.75)||!m_backend.flightReady(v,now,true)||
       v.position_at<=operation.last_position)return;
    if(!operation.tracking) {
      operation.tracking=true;
      directResult(operation.id,operation.action,"ACTIVE","post_request_target_readback");
    }
    operation.last_position=v.position_at;
    const double radians=3.14159265358979323846/180.,metres=6378137.*radians;
    const double north=(v.latitude-operation.target.latitude)*metres;
    const double east=std::remainder(v.longitude-operation.target.longitude,360.)*metres*
      std::cos(operation.target.latitude*radians);
    const double distance=std::hypot(north,east);
    const bool altitude_adjustment=operation.action=="adjust_altitude";
    if(std::abs(v.relative_height-operation.target.height)>(altitude_adjustment?3:12)||distance>125) {
      operation.orbit_sample=false;operation.orbit_sweep=0;return;
    }
    if(altitude_adjustment) {
      const double delta=operation.target.height-operation.starting_height;
      const double progress=std::copysign(1.,delta)*(v.relative_height-operation.starting_height);
      if(progress>=std::abs(delta)-3)finishDirect("COMPLETED","observed_relative_altitude_change");
      return;
    }
    if(operation.action!="hold") {
      finishDirect("COMPLETED",operation.action=="adjust_course"?"observed_adjusted_course_leg":"observed_waypoint_arrival");
      return;
    }
    if(operation.completed)return;
    if(distance<42) { operation.orbit_sample=false;operation.orbit_sweep=0;return; }
    const auto bearing=std::atan2(east,north)/radians;
    if(operation.orbit_sample)
      operation.orbit_sweep+=std::remainder(bearing-operation.previous_bearing,360.);
    operation.orbit_sample=true;
    operation.previous_bearing=bearing;
    if(operation.orbit_sweep>=140) {
      operation.completed=true;
      directResult(operation.id,operation.action,"CONFIRMED","observed_guided_orbit");
    }
  }

  void observeDirectTarget(const mavlink_message_t& message,const MessageTime& time)
  {
    if(!m_direct||m_direct->first_offered<=0||time.effective_at<=m_direct->first_offered||
       time.effective_at<=m_direct->last_readback||!plane::freshState(time.effective_at,m_services.now(),.75))return;
    mavlink_position_target_global_int_t target{};
    mavlink_msg_position_target_global_int_decode(&message,&target);
    const auto& expected=m_direct->target;
    const auto& v=m_services.vehicle();
    if(target.coordinate_frame==MAV_FRAME_GLOBAL&&target.type_mask==0xFDF8&&v.home_valid&&
       target.lat_int==std::llround(expected.latitude*1e7)&&target.lon_int==std::llround(expected.longitude*1e7)&&
       std::abs(target.alt-v.home_msl-expected.height)<=.3)m_direct->last_readback=time.effective_at;
  }

  void finishDirect(const char* status,const char* reason)
  {
    const auto operation=std::move(*m_direct);
    m_direct.reset();
    if(operation.requested&&m_services.navigationGranted())m_services.revokeNavigation(reason);
    directResult(operation.id,operation.action,status,reason);
  }

  void directResult(const std::string& id,const std::string& action,const char* status,const char* reason)
  {
    m_services.publish("MAVMOOS_CONTROL_RESULT","schema=1,profile=arduplane,id="+id+
      ",action="+action+",status="+status+",reason="+reason);
  }

  bool ordinaryModeReady() const
  {
    return !m_shadow&&m_services.navigationGranted()&&
      m_backend.flightReady(m_services.vehicle(),m_services.now(),true);
  }

  bool operationReady(const ModeOperation& operation) const
  {
    if(operation.action=="manual")
      return !m_shadow&&!m_services.vehicle().armed&&!m_services.navigationGranted()&&
        m_backend.groundReady(m_services.vehicle(),m_services.now())&&
        (m_services.vehicle().mode==plane::manual_mode||m_services.vehicle().mode==plane::loiter_mode||
         m_services.vehicle().mode==plane::rtl_mode);
    if(operation.request.command==MAV_CMD_COMPONENT_ARM_DISARM)
      return !m_shadow&&!m_services.navigationGranted()&&m_services.vehicle().mode==plane::manual_mode&&
        m_backend.groundReady(m_services.vehicle(),m_services.now())&&
        (operation.action=="disarm"||m_services.vehicle().armed||m_backend.prearmReady(m_services.now()));
    if(operation.request.command==MAV_CMD_GUIDED_CHANGE_SPEED)return ordinaryModeReady();
    const auto& v=m_services.vehicle();
    return ordinaryModeReady()||(!m_shadow&&!m_services.navigationGranted()&&
      (v.mode==plane::loiter_mode||v.mode==plane::rtl_mode)&&m_backend.flightReady(v,m_services.now(),false));
  }

  void requestMode(const Mail& mail)
  {
    auto operation=parse(mail);
    if(!operation) { report("invalid","invalid","REJECTED","invalid_plane_mode_request");return; }
    if(!liveMail(mail)) {
      report(operation->id,operation->action,"REJECTED","stale_future_or_pre_session_request");return;
    }
    if(m_seen.count(operation->id)) {
      report(operation->id,operation->action,"DUPLICATE","id_already_seen");return;
    }
    if(m_operation||m_services.controlBusy()||m_seen.size()>=history_limit||!operationReady(*operation)) {
      report(operation->id,operation->action,"REJECTED","eligible_operator_mode_required");return;
    }

    const double original=std::min(mail.published_at,mail.received_at);
    auto& request=operation->request;
    request.id="plane-op-"+operation->id;
    request.expires=original+admission_age_limit;
    request.operation_expires=original+operation_lifetime;
    operation->origin=m_services.vehicle().mode;
    if(operation->action=="manual"&&m_services.vehicle().mode==plane::manual_mode) {
      m_seen.insert(operation->id);
      report(operation->id,operation->action,"NO_OP","already_grounded_manual");return;
    }
    if(request.command==MAV_CMD_COMPONENT_ARM_DISARM) {
      request.params[0]=operation->action=="arm"?1:0;
      if(m_services.vehicle().armed==(request.params[0]==1)) {
        m_seen.insert(operation->id);
        report(operation->id,operation->action,"NO_OP","already_in_requested_arm_state");return;
      }
    } else if(request.command==MAV_CMD_GUIDED_CHANGE_SPEED) {
      const double target=operation->action=="reset_speed_min"?plane::operator_min_airspeed:
        m_speed_target.value_or(plane::cruise_ground_intent)+operation->delta;
      request.params[1]=static_cast<float>(target);
      request.params[2]=2;
      request.operation_expires=original+20;
      if(!plane::speedCommand(request)) {
        report(operation->id,operation->action,"REJECTED","airspeed_outside_qualified_envelope");return;
      }
    } else {
      if(operation->origin==operation->mode) {
        m_seen.insert(operation->id);
        report(operation->id,operation->action,"NO_OP","already_in_requested_native_mode");return;
      }
      request.command=MAV_CMD_DO_SET_MODE;
      request.params[0]=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
      request.params[1]=static_cast<float>(operation->mode);
    }
    const auto replay=reserveControlRequest(m_services,operation->id);
    if(replay!=ReplayResult::Reserved) {report(operation->id,operation->action,replayRejection(replay),replayReason(replay));return;}
    m_seen.insert(operation->id);
    m_operation=std::move(operation);
    report(m_operation->id,m_operation->action,"QUEUED","bounded_mode_request");
  }

  void advanceMode()
  {
    const double now=m_services.now();
    const auto& v=m_services.vehicle();
    if(v.mode!=plane::guided_mode||!m_services.navigationGranted())m_speed_target.reset();
    if(!m_operation)return;
    auto& operation=*m_operation;
    if(!std::isfinite(now)||now>=operation.request.operation_expires||
       (!operation.sent_at&&now>=operation.request.expires)) {
      finish("TIMED_OUT",operation.sent_at?"post_handoff_mode_unconfirmed":"local_handoff_deadline");return;
    }
    if(!v.heartbeat_valid||!m_backend.identityCompatible(v.type,v.autopilot)||
       !std::isfinite(v.heartbeat_at)||v.heartbeat_at<=0||
       now<v.heartbeat_at||now-v.heartbeat_at>heartbeat_age_limit)return;
    if(plane::manualCommand(operation.request)) {
      if(!operationReady(operation)) { finish("CANCELLED","fresh_ground_mode_lost");return; }
      if(operation.sent_at&&v.heartbeat_at>*operation.sent_at&&v.mode==plane::manual_mode)
        finish("CONFIRMED","post_handoff_ground_manual");
      return;
    }
    if(plane::armCommand(operation.request)) {
      if(!m_backend.groundReady(v,now)||v.mode!=plane::manual_mode) {
        finish("CANCELLED","fresh_ground_mode_lost");return;
      }
      if(operation.sent_at&&v.heartbeat_at>*operation.sent_at&&v.armed==(operation.request.params[0]==1))
        finish("CONFIRMED","post_handoff_ground_arm_state");
      return;
    }
    if(plane::speedCommand(operation.request)) {
      if(!ordinaryModeReady()) { finish("CANCELLED","guided_authority_lost");return; }
      const auto speed=m_backend.airspeed();
      const auto at=m_backend.airspeedAt();
      if(!operation.sent_at||!operation.speed_acknowledged||at<=*operation.sent_at||!speed)return;
      if(std::abs(*speed-operation.request.params[1])>1.75) {
        operation.speed_seen_at.reset();return;
      }
      if(!operation.speed_seen_at)operation.speed_seen_at=at;
      else if(at-*operation.speed_seen_at>=1.5) {
        m_speed_target=operation.request.params[1];
        finish("COMPLETED","matching_ack_and_measured_airspeed");
      }
      return;
    }
    if(operation.sent_at&&v.heartbeat_at<=*operation.sent_at)return;
    if(operation.sent_at&&v.armed&&v.mode==operation.mode)
      finish("CONFIRMED","post_handoff_fc_mode");
    else if(!v.armed||v.mode!=operation.origin)
      finish("CANCELLED","independent_mode_or_arm_change");
  }

  void observeSpeedAck(const mavlink_message_t& message,const MessageTime& time)
  {
    if(!m_operation||!m_operation->sent_at||!plane::speedCommand(m_operation->request)||
       !std::isfinite(time.read_at)||time.read_at<=*m_operation->sent_at||time.read_at>m_services.now()||
       time.effective_at<*m_operation->sent_at-.15||!plane::freshState(time.effective_at,m_services.now(),2))return;
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&message,&ack);
    if(ack.command!=m_operation->request.command||ack.target_system!=245||ack.target_component!=191)return;
    if(ack.result==MAV_RESULT_ACCEPTED)m_operation->speed_acknowledged=true;
    else if(ack.result!=MAV_RESULT_IN_PROGRESS)finish("FAILED","airspeed_command_rejected_by_fc");
  }

  void finish(const char* status,const char* reason)
  {
    const auto operation=std::move(*m_operation);
    m_operation.reset();
    if(operation.submitted)m_services.commandFinished(operation.request.id);
    report(operation.id,operation.action,status,reason);
  }

  void report(const std::string& id,const std::string& action,const char* status,const char* reason)
  {
    std::ostringstream result;
    result<<"schema=1,profile=arduplane,id="<<id<<",action="<<action<<",status="<<status<<",reason="<<reason
          <<std::fixed<<std::setprecision(6)<<",at="<<m_services.now();
    m_services.publish("MAVMOOS_COMMAND_RESULT",result.str());
  }

  void publishReadiness()
  {
    const double now=m_services.now();
    m_next_status=now+.2;
    const auto& vehicle=m_services.vehicle();
    const auto* reason=m_backend.readiness(vehicle,now);
    const double age=m_backend.airspeedAt()>0?now-m_backend.airspeedAt():-1;
    std::ostringstream result;
    result<<"schema=1,profile=arduplane,flight_ready="<<(reason?"false":"true")
          <<",ready_reason="<<(reason?reason:"qualified_airborne_state")<<",mode="<<vehicle.mode
          <<",prearm_ready="<<(m_backend.prearmReady(now)?"true":"false")
          <<",readiness_scope=MODULE_LOCAL,host_gates=INDEPENDENT"
          <<std::fixed<<std::setprecision(3)<<",airspeed_mps="<<m_backend.airspeed().value_or(-1)
          <<",airspeed_age_s="<<age<<",cruise_ground_intent_mps="<<plane::cruise_ground_intent;
    m_services.publish("MAVMOOS_PLANE_STATUS",result.str());
  }

  Services& m_services;
  plane::Backend m_backend;
  std::set<std::string> m_seen;
  plane::HelmTasks m_tasks;
  std::unique_ptr<Plugin> m_helm;
  double m_request_floor=0,m_next_status=0;
  std::optional<ModeOperation> m_operation;
  std::optional<DirectOperation> m_direct;
  std::optional<double> m_speed_target;
  std::optional<FlightOperation> m_flight;
  std::optional<Release> m_release;
  std::string m_retained;
  double m_cursor_at=0;
  unsigned m_cursor=0;
  bool m_native_enabled=true,m_helm_enabled=true,m_shadow=false,m_mission_enabled=false,m_native_autoland=false;
};
} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi,mavmoos::runtime::Services* services)
{ return abi==mavmoos::runtime::plugin_abi&&services?new mavmoos::runtime::ArduPlaneControl(*services):nullptr; }

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin)
{ delete plugin; }
