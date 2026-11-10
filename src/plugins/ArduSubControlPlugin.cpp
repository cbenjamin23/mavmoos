/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ArduSubControlPlugin.cpp                         */
/*    DATE: November 6, 2026                                */
/************************************************************/
#include "ControlComponents.h"
#include "ControlReplayService.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace mavmoos::runtime {
namespace {
constexpr unsigned depth_hold=2, guided=4, surface=9, position_hold=16;
constexpr double position_age=.5, pressure_age=1.6, estimator_age=2, home_age=5;
constexpr double radians=3.14159265358979323846/180.;
constexpr auto pressure_bit=MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;

bool fresh(double now,double at,double limit)
{
  return std::isfinite(now)&&std::isfinite(at)&&at>0&&now>=at&&now-at<=limit;
}

double depth(const Vehicle& vehicle)
{
  // ArduSub reports an MSL estimate from the EKF origin and an independently
  // defined surface home. Its wire relative_alt has different origin semantics.
  return vehicle.home_msl-vehicle.msl_height;
}

uint32_t wireTime(double now)
{
  return static_cast<uint32_t>(static_cast<uint64_t>(std::fmod(now,4294967.296)*1000.));
}

ControlFrame encode(const mavlink_message_t& message)
{
  ControlFrame result;
  result.size=mavlink_msg_to_send_buffer(result.bytes.data(),&message);
  return result;
}

CommandRequest modeRequest(std::string id,unsigned mode,double deadline)
{
  CommandRequest request;
  request.id=std::move(id);
  request.command=MAV_CMD_DO_SET_MODE;
  request.params[0]=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  request.params[1]=static_cast<float>(mode);
  request.expires=deadline;
  return request;
}

class SubBackend final:public VehicleControlBackend {
public:
  std::string profile() const override { return "ardusub"; }

  bool identityCompatible(unsigned type,unsigned autopilot) const override
  {
    return type==MAV_TYPE_SUBMARINE && autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA;
  }

  void observePressure(const mavlink_message_t& message,double at)
  {
    if(!std::isfinite(at)||at<m_pressure_at)return;
    mavlink_sys_status_t status{};
    mavlink_msg_sys_status_decode(&message,&status);
    m_pressure_at=at;
    m_pressure_healthy=(status.onboard_control_sensors_present &
      status.onboard_control_sensors_enabled & status.onboard_control_sensors_health & pressure_bit)!=0;
  }

  void resetPressure() { m_pressure_healthy=false;m_pressure_at=-1; }

  bool pressureHealthy(double now) const
  {
    return m_pressure_healthy && fresh(now,m_pressure_at,pressure_age);
  }

  bool ready(const Vehicle& v,double now) const
  {
    return identityCompatible(v.type,v.autopilot) && v.armed &&
      v.heartbeat_valid && fresh(now,v.heartbeat_at,2) &&
      v.position_valid && fresh(now,v.position_at,position_age) &&
      std::isfinite(v.latitude) && std::abs(v.latitude)<=90 &&
      std::isfinite(v.longitude) && std::abs(v.longitude)<=180 &&
      v.home_valid && fresh(now,v.home_at,home_age) &&
      std::isfinite(v.home_msl) && std::isfinite(v.msl_height) &&
      depth(v)>=-.6 && depth(v)<=12 && pressureHealthy(now) &&
      v.estimator_valid && fresh(now,v.estimator_at,estimator_age) &&
      !(v.estimator_flags&(EKF_CONST_POS_MODE|EKF_UNINITIALIZED|EKF_GPS_GLITCHING));
  }

  bool flightReady(const Vehicle& v,double now,bool motion) const override
  {
    return ready(v,now) && activeModeAllowed(v,!motion);
  }

  bool acquisitionAllowed(const Vehicle& v,double now) const override
  {
    return v.mode==position_hold && ready(v,now);
  }

  bool activeModeAllowed(const Vehicle& v,bool grace) const override
  {
    return v.mode==guided || (grace && v.mode==position_hold);
  }

  std::optional<CommandRequest> acquisitionCommand(const Vehicle& v,const std::string& id,double now) const override
  {
    if(!acquisitionAllowed(v,now))return std::nullopt;
    return modeRequest("acquire-"+id,guided,now+2.5);
  }

  bool commandAllowed(const CommandRequest& r,const Vehicle& v,double now) const override
  {
    if(r.command!=MAV_CMD_DO_SET_MODE || r.params[0]!=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED ||
       !std::all_of(r.params.begin()+2,r.params.end(),[](float p){return p==0;}))return false;
    if(r.params[1]==guided)return acquisitionAllowed(v,now);
    if(r.params[1]==surface)return r.id==m_surface_id && v.mode==m_surface_from && surfaceReady(v,now);
    return r.params[1]==position_hold && flightReady(v,now,true);
  }

  bool surfaceReady(const Vehicle& v,double now) const
  {
    return ready(v,now) && depth(v)>=.5 && (v.mode==guided || v.mode==position_hold || v.mode==depth_hold);
  }

  void bindSurface(const std::string& id,unsigned mode) { m_surface_id=id;m_surface_from=mode; }
  void clearSurface() { m_surface_id.clear(); }

  bool navigationInputAllowed(const NavigationTarget& t) const override
  {
    return t.kind==NavigationKind::CourseSpeed && t.dimensions==NavigationDimensions::Spatial &&
      t.facing==FacingPolicy::Course && helmFieldAllowed(0,t.course) &&
      helmFieldAllowed(1,t.speed) && helmFieldAllowed(2,t.height);
  }

  bool helmFieldAllowed(unsigned index,double value) const override
  {
    return std::isfinite(value) && (index==0?std::abs(value)<=360:
      index==1?value>=0&&value<=1:index==2&&value>=0&&value<=8);
  }

  bool helmVehicleReady(const Vehicle& v,double now,double timeout,bool shadow,bool acquisition) const override
  {
    return ready(v,now) && fresh(now,v.position_at,timeout) &&
      (shadow || activeModeAllowed(v,acquisition));
  }

  std::optional<ControlFrame> navigationFrame(const NavigationTarget& t,const Vehicle& v,double now,
    uint64_t,mavlink_status_t& codec) const override
  {
    if(!std::isfinite(t.expires)||now>=t.expires||!flightReady(v,now,true)||!navigationInputAllowed(t))
      return std::nullopt;
    mavlink_set_position_target_local_ned_t target{};
    target.time_boot_ms=wireTime(now);
    target.target_system=1;target.target_component=1;
    target.coordinate_frame=MAV_FRAME_LOCAL_NED;
    target.type_mask=POSITION_TARGET_TYPEMASK_X_IGNORE|POSITION_TARGET_TYPEMASK_Y_IGNORE|
      POSITION_TARGET_TYPEMASK_Z_IGNORE|POSITION_TARGET_TYPEMASK_AX_IGNORE|
      POSITION_TARGET_TYPEMASK_AY_IGNORE|POSITION_TARGET_TYPEMASK_AZ_IGNORE|
      POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    const double course=t.course*radians;
    target.vx=static_cast<float>(t.speed*std::cos(course));
    target.vy=static_cast<float>(t.speed*std::sin(course));
    target.vz=static_cast<float>(std::clamp(.6*(t.height-depth(v)),-.4,.4));
    if(t.speed>.08)target.yaw=static_cast<float>(std::remainder(course,2.*3.14159265358979323846));
    else target.type_mask|=POSITION_TARGET_TYPEMASK_YAW_IGNORE;
    mavlink_message_t message{};
    mavlink_msg_set_position_target_local_ned_encode_status(245,191,&codec,&message,&target);
    return encode(message);
  }

  std::optional<ControlFrame> stopFrame(const Vehicle& v,double now,uint64_t,mavlink_status_t& codec) const override
  {
    if(!flightReady(v,now,true))return std::nullopt;
    // Zero offsets capture the current EKF position when the FC receives
    // the target. Sub's global setpoint handler can change native Depth Hold.
    mavlink_set_position_target_local_ned_t target{};
    target.time_boot_ms=wireTime(now);
    target.target_system=1;target.target_component=1;
    target.coordinate_frame=MAV_FRAME_LOCAL_OFFSET_NED;
    target.type_mask=POSITION_TARGET_TYPEMASK_VX_IGNORE|POSITION_TARGET_TYPEMASK_VY_IGNORE|
      POSITION_TARGET_TYPEMASK_VZ_IGNORE|POSITION_TARGET_TYPEMASK_AX_IGNORE|
      POSITION_TARGET_TYPEMASK_AY_IGNORE|POSITION_TARGET_TYPEMASK_AZ_IGNORE|
      POSITION_TARGET_TYPEMASK_YAW_IGNORE|POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    mavlink_message_t message{};
    mavlink_msg_set_position_target_local_ned_encode_status(245,191,&codec,&message,&target);
    return encode(message);
  }

private:
  double m_pressure_at=-1;
  bool m_pressure_healthy=false;
  std::string m_surface_id;
  unsigned m_surface_from=0;
};

class ArduSubControl final:public Plugin {
public:
  explicit ArduSubControl(Services& host)
    :m_host(host),m_helm(makeHelmAdapter(host,m_backend)),m_handback_at(host.now()) {}
  std::string name() const override { return "ardusub_control"; }
  const VehicleControlBackend* controlBackend() const override { return &m_backend; }

  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.profile=m_backend.profile();
    result.exclusive_resources={"vehicle.control"};
    result.request_owners={"MAVMOOS_NAV_CONTROL","MAVMOOS_SUB_HANDBACK"};
    result.observation_streams={{MAVLINK_MSG_ID_SYS_STATUS,250000.f},{MAVLINK_MSG_ID_GLOBAL_POSITION_INT,100000.f},
      {MAVLINK_MSG_ID_EKF_STATUS_REPORT,500000.f},{MAVLINK_MSG_ID_HOME_POSITION,1000000.f}};
    if(m_helm_enabled)result.capabilities={{"control.navigation.helm","MAVMOOS_NAV_CONTROL","acquire|release"},
      {"control.hold.guided_position","","","internal"},
      {"control.handback.position_hold","MAVMOOS_SUB_HANDBACK","hold"},
      {"control.surface.native","MAVMOOS_SUB_HANDBACK","surface"}};
    return result;
  }

  std::vector<std::string> subscriptions() const override
  {
    if(!m_helm_enabled)return {};
    auto keys=m_helm->subscriptions();
    keys.push_back("MAVMOOS_SUB_HANDBACK");
    return keys;
  }

  void configure(const std::string& key,const std::string& value) override
  {
    if(key=="vehicle_profile"&&value!=m_backend.profile())throw std::invalid_argument("invalid ArduSub profile");
    if(key=="control_native_actions"&&value!="false")throw std::invalid_argument("ArduSub has no general native command facet");
    if(key=="facing_policy"&&value!="course")throw std::invalid_argument("ArduSub profile supports course facing");
    if(key=="control_helm") {
      if(value!="true"&&value!="false")throw std::invalid_argument("invalid ArduSub helm setting");
      m_helm_enabled=value=="true";
    }
    m_helm->configure(key,value);
  }

  void onMail(const Mail& mail) override
  {
    if(!m_helm_enabled)return;
    if(mail.key!="MAVMOOS_SUB_HANDBACK") {
      m_helm->onMail(mail);
      return;
    }
    const double now=m_host.now();
    std::string action=mail.text,id;
    constexpr char prefix[]="schema=1,id=";
    if(action.rfind(prefix,0)==0) {
      const auto separator=action.find(",action=",sizeof(prefix)-1);
      if(separator==std::string::npos) return;
      id=action.substr(sizeof(prefix)-1,separator-(sizeof(prefix)-1));
      if(id.empty()||id.size()>48||id.find_first_not_of(
         "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-")!=std::string::npos) return;
      action=action.substr(separator+8);
    }
    const bool request_surface=action=="surface";
    if(mail.numeric || (action!="hold" && !request_surface) || !std::isfinite(mail.published_at) ||
       !std::isfinite(mail.received_at) || mail.published_at<=m_handback_at ||
       mail.published_at>mail.received_at+.1 || mail.received_at>now+.1 ||
       now-mail.published_at>=.5)return;
    m_handback_at=mail.published_at;
    if(m_pending)return;
    const auto& vehicle=m_host.vehicle();
    const bool owned=m_host.navigationGranted();
    const auto result_id=id.empty()?std::string(request_surface?"sub-surface-":"sub-hold-")+
      std::to_string(++m_request_number):id;
    if(!id.empty() && controlRequestSeen(m_host,id)) {
      publishResult(result_id,"DUPLICATE",replayReason(ReplayResult::Seen));return;
    }
    auto request=modeRequest(result_id,
      request_surface?surface:position_hold,std::min(now+.4,mail.published_at+.5));
    request.operation_expires=mail.published_at+(request_surface?60.:2.8);
    const bool eligible=request_surface?(m_backend.surfaceReady(vehicle,now) && (vehicle.mode!=guided || owned)):
      (owned && m_backend.flightReady(vehicle,now,true));
    if(eligible) {
      const auto replay=id.empty()?reserveControlEvent(m_host):reserveControlRequest(m_host,id);
      if(replay!=ReplayResult::Reserved) {publishResult(result_id,replayRejection(replay),replayReason(replay));return;}
    }
    if(eligible && request_surface)m_backend.bindSurface(request.id,vehicle.mode);
    if(!eligible || !m_host.sendCommand(request)) {
      if(request_surface)m_backend.clearSurface();
      publishResult(request.id,"REJECTED",request_surface?"owned_submerged_mode_required":"owned_guided_required");
      return;
    }
    m_pending=Pending{request.id,request.operation_expires,-1,request_surface,vehicle.mode};
    publishResult(request.id,"QUEUED","waiting_for_socket");
    if(owned) {
      auto release=mail;
      release.key="MAVMOOS_NAV_CONTROL";release.text="release";
      m_helm->onMail(release);
    }
  }

  void onMessage(const mavlink_message_t& message,double at) override
  {
    onMessageWithMetadata(message,{at,at,"READ_ACQUISITION"});
  }

  void onMessageWithMetadata(const mavlink_message_t& message,const MessageTime& time) override
  {
    if(message.sysid!=1 || message.compid!=1)return;
    if(message.msgid==MAVLINK_MSG_ID_SYS_STATUS) {
      m_backend.observePressure(message,time.effective_at);
      if(m_host.navigationGranted()&&!m_backend.pressureHealthy(m_host.now()))
        m_host.revokeNavigation("sub_depth_sensor_unhealthy");
    }
    if(message.msgid!=MAVLINK_MSG_ID_HEARTBEAT || !m_pending || m_pending->sent_at<0 ||
       time.effective_at<=m_pending->sent_at || m_host.now()>=m_pending->deadline)return;
    const auto& vehicle=m_host.vehicle();
    if(m_pending->request_surface) {
      if(!vehicle.armed || (m_pending->surface_seen_at<0 && vehicle.mode!=surface && vehicle.mode!=m_pending->from) ||
         (m_pending->surface_seen_at>=0 && vehicle.mode!=surface && vehicle.mode!=depth_hold)) {
        finish("PREEMPTED","external_mode_or_disarm");
      } else if(vehicle.heartbeat_valid && vehicle.mode==surface && m_pending->surface_seen_at<0) {
        m_pending->surface_seen_at=time.effective_at;
        publishResult(m_pending->id,"CONFIRMED","native_surface_observed");
      }
    } else if(vehicle.heartbeat_valid && vehicle.armed && vehicle.mode==position_hold)
      finish("CONFIRMED","native_position_hold_observed");
    else if(!vehicle.armed || vehicle.mode!=guided)
      finish("PREEMPTED","external_mode_or_disarm");
  }

  void onCommandSent(const CommandRequest& request,double at,bool accepted) override
  {
    if(m_helm_enabled)m_helm->onCommandSent(request,at,accepted);
    if(!m_pending || request.id!=m_pending->id)return;
    if(!accepted) {
      finish("FAILED","socket_not_accepted");
      return;
    }
    m_pending->sent_at=at;
    publishResult(request.id,"SENT",m_pending->request_surface?"waiting_for_native_surface":"waiting_for_native_position_hold");
  }

  void onReset(const std::string& reason) override
  {
    if(m_pending)finish("UNCONFIRMED","connection_reset");
    m_handback_at=m_host.now();
    m_backend.resetPressure();
    if(m_helm_enabled)m_helm->onReset(reason);
  }

  void onInputBoundary() override
  {
    checkProgress();
    if(m_helm_enabled)m_helm->onInputBoundary();
  }

  void iterate() override
  {
    if(m_helm_enabled)m_helm->iterate();
    checkProgress();
    if(m_host.now()-m_last_status<.2)return;
    m_last_status=m_host.now();
    m_host.publish("MAVMOOS_SUB_STATUS",std::string("schema=1,depth_reference=FC_HOME_SURFACE_MSL")+
      ",pressure_healthy="+(m_backend.pressureHealthy(m_host.now())?"true":"false")+
      ",ready="+(m_backend.ready(m_host.vehicle(),m_host.now())?"true":"false"));
  }

private:
  struct Pending {
    std::string id;
    double deadline,sent_at;
    bool request_surface;
    unsigned from;
    double surface_seen_at=-1, shallow_since=-1;
  };
  void checkProgress()
  {
    if(!m_pending)return;
    const double now=m_host.now();
    auto& pending=*m_pending;
    if(now>=pending.deadline || (pending.request_surface && pending.sent_at>=0 &&
       pending.surface_seen_at<0 && now>=pending.sent_at+3.)) {
      finish("UNCONFIRMED","confirmation_timeout");
      return;
    }
    if(!pending.request_surface || pending.surface_seen_at<0)return;
    const auto& v=m_host.vehicle();
    if(v.mode!=depth_hold || v.heartbeat_at<=pending.surface_seen_at || v.position_at<=pending.surface_seen_at ||
       !m_backend.ready(v,now) || depth(v)>.35) {
      pending.shallow_since=-1;
      return;
    }
    if(pending.shallow_since<0)pending.shallow_since=v.position_at;
    if(v.position_at-pending.shallow_since>=.5)finish("COMPLETED","native_surface_and_shallow_depth_hold_observed");
  }
  void publishResult(const std::string& id,const char* status,const char* reason)
  {
    m_host.publish("MAVMOOS_SUB_HANDBACK_RESULT","id="+id+",status="+status+",reason="+reason);
  }
  void finish(const char* status,const char* reason)
  {
    const std::string id=m_pending->id;
    if(m_pending->request_surface)m_backend.clearSurface();
    m_pending.reset();
    m_host.commandFinished(id);
    publishResult(id,status,reason);
  }
  Services& m_host;
  SubBackend m_backend;
  std::unique_ptr<Plugin> m_helm;
  std::optional<Pending> m_pending;
  double m_handback_at=-1,m_last_status=-1;
  unsigned m_request_number=0;
  bool m_helm_enabled=true;
};
} // namespace
} // namespace mavmoos::runtime

extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi,mavmoos::runtime::Services* services)
{
  return abi==mavmoos::runtime::plugin_abi && services?
    new mavmoos::runtime::ArduSubControl(*services):nullptr;
}

extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin) { delete plugin; }
