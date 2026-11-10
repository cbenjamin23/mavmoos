/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: CopterOperations.cpp                            */
/*    DATE: November 4, 2026                                */
/************************************************************/
#include "ControlReplayService.h"
#include "ControlComponents.h"
#include "CopterOperations.h"
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
constexpr double pi=3.14159265358979323846;
std::string trim(const std::string& value)
{
  const auto first=value.find_first_not_of(" \t\r\n");
  return first==std::string::npos?std::string{}:value.substr(first,value.find_last_not_of(" \t\r\n")-first+1);
}
bool identifier(const std::string& value)
{
  return !value.empty()&&value.size()<=40&&std::all_of(value.begin(),value.end(),[](unsigned char c){
    return (c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_'||c=='-'||c=='.';
  });
}
bool number(const std::string& value,double& result)
{
  try {std::size_t used=0;result=std::stod(value,&used);return used==value.size()&&std::isfinite(result);}
  catch(...) {return false;}
}
std::map<std::string,std::string> fields(const std::string& text,bool& valid,std::size_t maximum=512)
{
  if(text.empty()||text.size()>maximum) {valid=false;return {};}
  std::map<std::string,std::string> result;std::istringstream stream(text);std::string item;
  while(std::getline(stream,item,',')) {
    const auto split=item.find('=');
    if(split==std::string::npos) {valid=false;break;}
    const auto key=trim(item.substr(0,split)),value=trim(item.substr(split+1));
    if(key.empty()||value.empty()||!result.emplace(key,value).second) {valid=false;break;}
  }
  if(!text.empty()&&text.back()==',')valid=false;
  return result;
}
double distance(double a_lat,double a_lon,double b_lat,double b_lon)
{
  const double a=a_lat*pi/180.,b=b_lat*pi/180.;
  const double dlat=b-a,dlon=(b_lon-a_lon)*pi/180.;
  const double hav=std::pow(std::sin(dlat/2),2)+std::cos(a)*std::cos(b)*std::pow(std::sin(dlon/2),2);
  return 12742000.*std::asin(std::sqrt(std::clamp(hav,0.,1.)));
}
bool fresh(bool valid,double acquired,double now,double age)
{return valid&&std::isfinite(acquired)&&acquired>0&&acquired<=now&&now-acquired<=age;}
bool terminal(const std::string& status)
{return status=="COMPLETED"||status=="FAILED"||status=="TIMED_OUT"||status=="CANCELLED"||status=="REJECTED"||status=="DUPLICATE";}
struct Operation {
  std::string id,action,strategy,target,native_id;
  double latitude=0,longitude=0,height=5,speed=1.5,radius=1,timeout=60,heading=0;
  double delta=0;
  double published=0,started=0,expires=0,settled_at=-1,last_target=-1,guided_sent_at=-1;
  FacingPolicy facing=FacingPolicy::Course;
  Vehicle home;
  bool submitted=false,confirmed=false,needs_guided_entry=false;
  bool helmAdjustment() const
  { return strategy=="helm"&&(action=="adjust_altitude"||action=="adjust_speed"||action=="reset_speed_min"); }
  bool helmTask() const {return action=="survey"||action=="voronoi";}
};
class Operations final:public CopterOperations {
public:
  Operations(Services& services,Plugin& native,Plugin& helm,const VehicleControlBackend& backend)
    :m_services(services),m_native(native),m_helm(helm),m_backend(backend),m_boundary(services.now()),m_last_now(services.now()) {}
  std::string name() const override {return "copter_operations";}
  void setFacets(bool native_actions,bool helm) override {m_native_enabled=native_actions;m_helm_enabled=helm;}
  std::vector<Capability> capabilities() const override
  {
    std::vector<Capability> result;
    if(m_native_enabled) {
      result.push_back({"control.operations.native","MAVMOOS_CONTROL_REQUEST","schema=1,strategy=native","request"});
      result.push_back({"control.operations.guided","MAVMOOS_CONTROL_REQUEST","schema=1,strategy=guided","request"});
    }
    if(m_helm_enabled&&m_helm_configured) {
      result.push_back({"control.operations.helm","MAVMOOS_CONTROL_REQUEST","schema=1,strategy=helm","request"});
      if(m_helm_adjustments) result.push_back({"control.operations.helm.adjustment","MAVMOOS_CONTROL_REQUEST","schema=1,strategy=helm,action=adjust_speed|reset_speed_min|adjust_altitude","request"});
      for(const auto& task:m_helm_tasks)
        result.push_back({"control.operations.helm."+task,"MAVMOOS_CONTROL_REQUEST","schema=1,action="+task+",strategy=helm","request"});
    }
    if(m_native_enabled||(m_helm_enabled&&m_helm_configured))
      result.push_back({"control.operations.cancel","MAVMOOS_CONTROL_REQUEST","schema=1,action=cancel","request"});
    return result;
  }
  std::vector<std::string> subscriptions() const override
  {return m_native_enabled||m_helm_enabled?std::vector<std::string>{"MAVMOOS_CONTROL_REQUEST"}:std::vector<std::string>{};}
  void configure(const std::string& key,const std::string& value) override
  {
    if(key=="operation_helm_enabled") {
      if(value!="true"&&value!="false")throw std::invalid_argument("operation_helm_enabled must be true/false");
      m_helm_configured=value=="true";
    } else if(key=="operation_helm_adjustments") {
      if(value!="true"&&value!="false") throw std::invalid_argument("operation_helm_adjustments must be true/false");
      m_helm_adjustments=value=="true";
    } else if(key=="command_speed_min") {
      if(!number(value,m_min_speed)||m_min_speed<.1||m_min_speed>5)
        throw std::invalid_argument("command_speed_min outside .1..5m/s");
    } else if(key=="operation_helm_tasks") {
      std::set<std::string> tasks;
      std::istringstream stream(value);
      std::string entry;
      while(std::getline(stream,entry,',')) {
        const auto task=trim(entry);
        if((task!="survey"&&task!="voronoi")||!tasks.insert(task).second)
          throw std::invalid_argument("operation_helm_tasks must contain unique survey or voronoi tasks");
      }
      if(tasks.empty()||value.back()==',')
        throw std::invalid_argument("operation_helm_tasks must contain at least one task and no empty entries");
      m_helm_tasks=std::move(tasks);
    } else if(key=="operation_timeout"||key=="operation_radius"||key=="operation_speed") {
      double parsed=0;
      if(!number(value,parsed))throw std::invalid_argument("invalid "+key);
      if(key=="operation_timeout") {
        if(parsed<1||parsed>120)throw std::invalid_argument("operation_timeout outside1..120s");
        m_timeout=parsed;m_timeout_configured=true;
      } else if(key=="operation_radius") {
        if(parsed<.25||parsed>10)throw std::invalid_argument("operation_radius outside.25..10m");
        m_radius=parsed;
      } else {
        if(parsed<.1||parsed>5)throw std::invalid_argument("operation_speed outside.1..5m/s");
        m_speed=parsed;
      }
    } else if(key=="operation_helm_source") {
      if(value.empty()||value.size()>96)throw std::invalid_argument("invalid operation_helm_source");
      m_helm_source=value;m_helm.configure("helm_source",value);
    } else if(key=="helm_source") {
      m_helm_source=value;
    }
  }
  void onMail(const Mail& mail) override
  {
    if(mail.key!="MAVMOOS_CONTROL_REQUEST") {
      if(mail.key=="MAVMOOS_NAV_CONTROL"&&!mail.numeric&&validMail(mail)&&
         (mail.text=="acquire"||mail.text=="release")) {
        m_helm_request_at=std::max(m_helm_request_at,mail.published_at);
        m_helm_speed.reset();m_helm_height.reset();
        if(m_active)finish("CANCELLED","legacy_navigation_request",false);
      }
      if(m_helm_enabled&&m_helm_adjustments&&mail.source==m_helm_source&&mail.numeric&&
         validMail(mail)&&mail.published_at>m_helm_request_at&&m_services.navigationGranted()) {
        if(mail.key=="DESIRED_SPEED") rememberDemand(m_helm_speed,mail,1);
        if(mail.key=="DESIRED_ALTITUDE") rememberDemand(m_helm_height,mail,2);
      }
      return;
    }
    if(!m_native_enabled&&!m_helm_enabled)return;
    Operation op;std::string reason;
    if(!parse(mail,op,reason)) {report(op,"REJECTED",reason);return;}
    if(m_seen.count(op.id)||(m_queued&&m_queued->id==op.id)) {report(op,"DUPLICATE","id_already_seen");return;}
    if(op.action=="cancel") {
      const bool queued_target=m_queued&&m_queued->action!="cancel"&&m_queued->id==op.target;
      if(queued_target) {
        if(m_seen.size()<128)m_seen.insert(m_queued->id);
        m_last_cancelled=m_queued->id;
        report(*m_queued,"CANCELLED","cancelled_before_admission");m_queued.reset();
      }
      if(m_queued) {report(op,"REJECTED","request_pending");return;}
      if(!queued_target&&(!m_active||m_active->id!=op.target)) {report(op,"REJECTED","target_not_active");return;}
      m_queued=op;return;
    }
    const bool active_conflict=m_active&&!(op.helmAdjustment()&&m_active->helmTask());
    if(m_queued||(op.action!="visualize_home"&&(active_conflict||nativeQuarantined()||m_services.controlBusy()))) {
      report(op,"REJECTED","busy");return;
    }
    if(op.action!="visualize_home" && op.strategy!="native") {
      const auto replay=reserveControlRequest(m_services,op.id);
      if(replay!=ReplayResult::Reserved) {report(op,replayRejection(replay),replayReason(replay));return;}
    }
    m_queued=op;
  }
  void onReset(const std::string&) override
  {
    if(m_queued) {report(*m_queued,"CANCELLED","connection_reset");if(m_seen.size()<128)m_seen.insert(m_queued->id);m_queued.reset();}
    if(m_active)finish("CANCELLED","connection_reset");
    m_native_id.clear();m_native_until=0;m_native_accepted=false;m_native_terminal=false;
    m_helm_speed.reset();m_helm_height.reset();
    m_boundary=m_services.now();m_last_now=m_boundary;m_clock_fault=false;m_helm_phase="DISABLED";m_helm_at=-1;
  }
  void onInputBoundary() override {
    expireAndCheck();
    if(!m_services.navigationGranted()) {m_helm_speed.reset();m_helm_height.reset();}
  }
  void onCommandSent(const CommandRequest& request,double at,bool accepted) override
  {
    if(m_active&&m_active->needs_guided_entry&&accepted&&request.command==MAV_CMD_DO_SET_MODE&&
       request.id=="acquire-operation_"+m_active->id)
      m_active->guided_sent_at=at;
  }
  void iterate() override
  {
    if(!m_native_enabled&&!m_helm_enabled)return;
    expireAndCheck();
    if(m_queued) {
      Operation op=*m_queued;m_queued.reset();
      if(m_services.now()>=op.expires) {report(op,"REJECTED","request_deadline");return;}
      if(op.action!="cancel"&&m_seen.size()>=128) {report(op,"REJECTED","request_history_full");return;}
      if(m_seen.size()<128)m_seen.insert(op.id);
      if(op.action=="cancel") {
        if(m_active&&m_active->id==op.target) {
          m_last_cancelled=m_active->id;finish("CANCELLED","operator_cancel");
          report(op,"COMPLETED","local_cancellation_no_remote_recall");
        } else if(m_last_cancelled==op.target)report(op,"COMPLETED","cancelled_before_admission");
        else report(op,"REJECTED","target_not_active");
      } else start(op);
    }
    if(m_active&&m_active->strategy!="helm")updateNavigation();
  }
  void afterComponents() override
  {
    if(!m_native_enabled&&!m_helm_enabled)return;
    // The adapter publishes its current phase after checking mail/continuity.
    // Assess helm permission only then: a legitimate suspension revokes the
    // grant before its PAUSED status replaces the previous ACTIVE snapshot.
    if(m_active&&m_active->strategy=="helm")updateNavigation();
    std::ostringstream value;
    value<<"schema=1,active="<<(m_active?"true":"false")<<",id="<<(m_active?m_active->id:"none")
         <<",action="<<(m_active?m_active->action:"none")<<",strategy="<<(m_active?m_active->strategy:"none")
         <<",phase="<<(m_active?(m_active->confirmed?"ACTIVE":"ACQUIRING"):"IDLE")
         <<",native_quarantined="<<(nativeQuarantined()?"true":"false")
         <<",native_terminal="<<(m_native_terminal?"true":"false")
         <<",helm_configured="<<(m_helm_configured?"true":"false")
         <<",helm_source="<<m_helm_source
         <<",expires="<<std::fixed<<std::setprecision(9)<<(m_active?m_active->expires:0)
         <<",at="<<m_services.now();
    m_services.publish("MAVMOOS_CONTROL_STATUS",value.str());
  }
  void observePublication(const std::string& key,const std::string& value) override
  {
    if(key!="MAVMOOS_HELM_STATUS"&&key!="MAVMOOS_COMMAND_RESULT")return;
    if(key=="MAVMOOS_HELM_STATUS") {
      // The trusted adapter's first field is its phase. Later diagnostic values
      // may themselves contain commas (e.g. MissingDecVars:speed,course).
      // They are not a strict request-shaped map and must not hide LATCHED.
      if(value.size()>8192||value.rfind("state=",0)!=0)return;
      const auto end=value.find(',');const auto phase=value.substr(6,end==std::string::npos?end:end-6);
      if(phase!="ACTIVE"&&phase!="PAUSED"&&phase!="ACQUIRING"&&phase!="DISABLED"&&phase!="LATCHED")return;
      m_helm_phase=phase;m_helm_at=m_services.now();
      if(m_active&&m_active->strategy=="helm"&&m_helm_phase=="LATCHED")finish("CANCELLED","helm_latched");
      return;
    }
    bool valid=true;const auto data=fields(value,valid,8192);if(!valid)return;
    if(key!="MAVMOOS_COMMAND_RESULT"||!data.count("id")||!data.count("status"))return;
    const auto& id=data.at("id");const auto& status=data.at("status");
    if(status=="ADMITTED"&&m_active&&id!=m_active->native_id)finish("CANCELLED","legacy_native_request");
    if(id!=m_native_id)return;
    if(status=="ADMITTED"&&m_active&&m_active->native_id==id) {
      stopHelm();m_services.revokeNavigation("native_operation");
    }
    if(status=="SUBMITTED")m_native_accepted=true;
    if(!m_active||m_active->native_id!=id) {
      if(status=="COMPLETED"||status=="REJECTED"||status=="DUPLICATE")clearNative();
      else if(terminal(status))m_native_terminal=true;
      return;
    }
    if(status=="ADMITTED"||status=="SUBMITTED"||status=="ACK_OBSERVED"||status=="CONFIRMED") {
      report(*m_active,status,"native_"+status);
      if(status=="CONFIRMED")m_active->confirmed=true;
    }
    if(terminal(status)) {
      const bool uncertain=m_native_accepted&&status!="COMPLETED"&&status!="REJECTED"&&status!="DUPLICATE";
      if(!uncertain)clearNative();else m_native_terminal=true;
      finish(status=="DUPLICATE"?"REJECTED":status,"native_"+(data.count("reason")?data.at("reason"):status));
    }
  }
  bool boundHelmTaskTarget(NavigationTarget& target) const override
  {
    if(!m_active||!m_active->helmTask())return true;
    target.expires=std::min(target.expires,m_active->expires);
    return target.expires>m_services.now();
  }
  bool boundNativeCommand(CommandRequest& request) const override
  {
    if(request.id!=m_native_id)return true;
    if(!m_active||m_active->native_id!=request.id||m_services.now()>=m_active->expires)return false;
    request.expires=std::min(request.expires,m_active->expires);
    request.operation_expires=std::min(request.operation_expires,m_active->expires);
    return request.expires>m_services.now()&&request.operation_expires>m_services.now();
  }
  void nativeSubmitted(const CommandRequest& request,bool accepted) override
  {
    if(request.id==m_native_id&&accepted) {
      m_native_accepted=true;
      // The caller's bound is an upper limit; the legacy native action may
      // already have a shorter original operation lifetime.
      m_native_until=std::min(m_native_until,request.operation_expires);
    }
  }
  bool nativeFinished(const std::string& id) override
  {return id!=m_native_id||!nativeQuarantined();}
private:
  bool validMail(const Mail& mail) const
  {
    const double now=m_services.now(),age=mail.received_at-mail.published_at;
    return !m_clock_fault&&std::isfinite(now)&&now>=m_boundary&&std::isfinite(mail.received_at)&&std::isfinite(mail.published_at)&&
      mail.published_at>m_boundary&&age>=-.1&&age<=2&&mail.received_at<=now+.1&&now-mail.received_at<=2;
  }
  bool parse(const Mail& mail,Operation& op,std::string& reason) const
  {
    reason="invalid_request";bool valid=!mail.numeric;auto data=fields(mail.text,valid);
    op.id=data.count("id")?data["id"]:"invalid";op.action=data.count("action")?data["action"]:"invalid";
    if(!identifier(op.id))op.id="invalid";
    const std::set<std::string> allowed={"schema","id","action","strategy","lat","lon","height","facing","heading","timeout","radius","speed","target","delta"};
    for(const auto& entry:data)if(!allowed.count(entry.first))valid=false;
    const std::set<std::string> actions={"goto","hold","return","arm","disarm","takeoff","land","guided","cancel",
                                         "adjust_altitude","adjust_speed","reset_speed_min","survey","voronoi","visualize_home"};
    if(!actions.count(op.action)) {op.action="invalid";return false;}
    if(!valid||data["schema"]!="1"||!data.count("id")||!identifier(data["id"]))return false;
    if(!validMail(mail)) {reason="stale_future_or_pre_session_request";return false;}
    op.published=mail.published_at;op.started=mail.received_at;
    op.strategy=data.count("strategy")?data["strategy"]:((op.action=="goto"||op.action=="adjust_altitude")?"guided":"native");
    if(op.strategy!="native"&&op.strategy!="guided"&&op.strategy!="helm") {op.strategy="invalid";return false;}
    op.timeout=m_timeout_configured?m_timeout:(op.strategy=="native"&&(op.action=="arm"||op.action=="disarm"||op.action=="guided"||op.action=="hold")?8:op.action=="takeoff"?45:60);
    op.radius=m_radius;op.speed=m_speed;
    auto parsed=[&](const std::string& key,double& result){return !data.count(key)||number(data[key],result);};
    if(!parsed("timeout",op.timeout)||op.timeout<1||op.timeout>120||!parsed("radius",op.radius)||op.radius<.25||op.radius>10||
       !parsed("speed",op.speed)||op.speed<.1||op.speed>5||!parsed("height",op.height)||op.height<1||op.height>50||
       !parsed("heading",op.heading))return false;
    op.expires=op.published+op.timeout;
    if(op.expires<=m_services.now()) {reason="request_deadline";return false;}
    const auto onlyFields=[&](const std::set<std::string>& keys) {
      return std::all_of(data.begin(),data.end(),[&](const auto& entry){return keys.count(entry.first)!=0;});
    };
    if(op.action=="cancel") {
      if(!onlyFields({"schema","id","action","target","timeout"})||!data.count("target")||!identifier(data["target"]))return false;
      op.target=data["target"];op.strategy="local";return true;
    }
    if(op.action=="visualize_home") {
      op.strategy="local";
      return onlyFields({"schema","id","action","timeout"});
    }
    if(op.helmTask()) {
      if(op.strategy!="helm") {reason="unsupported_action_strategy";return false;}
      if(!m_helm_enabled||!m_helm_configured||!m_helm_tasks.count(op.action)) {reason="helm_task_not_configured";return false;}
      return onlyFields({"schema","id","action","strategy","timeout"});
    }
    if(op.helmAdjustment()) {
      if(!m_helm_enabled||!m_helm_configured||!m_helm_adjustments) {reason="helm_adjustment_not_configured";return false;}
      if(op.action=="reset_speed_min") return onlyFields({"schema","id","action","strategy","timeout"});
      return onlyFields({"schema","id","action","strategy","delta","timeout"})&&data.count("delta")&&number(data["delta"],op.delta);
    }
    if(op.action=="adjust_speed"||op.action=="reset_speed_min") {reason="unsupported_action_strategy";return false;}
    if(op.action=="adjust_altitude") {
      if(op.strategy!="guided") {reason="unsupported_action_strategy";return false;}
      if(!m_native_enabled) {reason="guided_strategy_disabled";return false;}
      if(!onlyFields({"schema","id","action","strategy","delta","timeout"})||!data.count("delta")||
         !number(data["delta"],op.delta)||std::abs(op.delta)<.1)return false;
      op.facing=FacingPolicy::Keep;
      return true;
    }
    if(data.count("target")||data.count("delta"))return false;
    if(op.strategy=="native") {
      if(!m_native_enabled) {reason="native_strategy_disabled";return false;}
      if(op.action=="goto") {reason="unsupported_action_strategy";return false;}
      for(const auto& item:data)if(item.first!="schema"&&item.first!="id"&&item.first!="action"&&item.first!="strategy"&&item.first!="timeout"&&!(item.first=="height"&&op.action=="takeoff"))return false;
      if(op.action=="takeoff"&&op.height>20)return false;
      return true;
    }
    if(op.strategy=="guided"&&!m_native_enabled) {reason="guided_strategy_disabled";return false;}
    if(op.strategy=="helm"&&(!m_helm_enabled||!m_helm_configured)) {reason="helm_operation_not_configured";return false;}
    if((op.action!="goto"&&op.action!="hold"&&op.action!="return")||(op.strategy=="guided"&&op.action=="return")) {reason="unsupported_action_strategy";return false;}
    if(data.count("facing")) {
      if(data["facing"]=="course")op.facing=FacingPolicy::Course;
      else if(data["facing"]=="keep")op.facing=FacingPolicy::Keep;
      else if(data["facing"]=="heading")op.facing=FacingPolicy::Heading;
      else return false;
    }
    if((op.facing==FacingPolicy::Heading)!=bool(data.count("heading")))return false;
    if(op.strategy=="helm"&&(data.count("facing")||data.count("heading"))) {reason="helm_facing_uses_startup_policy";return false;}
    if(op.strategy=="guided"&&data.count("speed")) {reason="guided_position_speed_is_fc_policy";return false;}
    if(op.action=="goto") {
      if(!data.count("lat")||!data.count("lon")||!data.count("height")||!parsed("lat",op.latitude)||!parsed("lon",op.longitude)||std::abs(op.latitude)>90||std::abs(op.longitude)>180)return false;
    } else {
      if(data.count("lat")||data.count("lon"))return false;
      if(!data.count("height"))op.height=m_services.vehicle().relative_height;
    }
    return true;
  }
  void start(Operation op)
  {
    const double now=m_services.now();op.started=now;
    if(op.action=="visualize_home") {visualizeHome(op);return;}
    if(m_services.controlBusy()||nativeQuarantined()) {report(op,"REJECTED","busy");return;}
    if(op.helmAdjustment()) {adjustHelmBehavior(op,now);return;}
    if(op.strategy=="native") {
      op.native_id="op."+op.id;m_native_id=op.native_id;m_native_until=op.expires;m_native_accepted=false;m_native_terminal=false;
      m_active=op;report(op,"SUBMITTING","delegated_native_policy_pending");
      Mail delegated;delegated.key="MAVMOOS_COMMAND";delegated.source="copter_operations";
      delegated.text="id="+op.native_id+",action="+(op.action=="return"?"rtl":op.action);
      if(op.action=="takeoff")delegated.text+=",height="+std::to_string(op.height);
      delegated.published_at=op.published;delegated.received_at=now;
      deliverCopterNativeRequest(m_native,delegated,op.id);return;
    }
    const auto& vehicle=m_services.vehicle();
    if(op.helmTask()) {startHelmTask(op,vehicle,now);return;}
    if(!m_backend.acquisitionAllowed(vehicle,now)) {report(op,"REJECTED","vehicle_not_ready_for_guided_control");return;}
    op.home=vehicle;
    if(op.action=="hold") {op.latitude=vehicle.latitude;op.longitude=vehicle.longitude;}
    if(op.action=="adjust_altitude") {
      op.latitude=vehicle.latitude;op.longitude=vehicle.longitude;
      op.height=vehicle.relative_height+op.delta;
    }
    if(op.action=="return") {op.latitude=vehicle.home_latitude;op.longitude=vehicle.home_longitude;}
    NavigationTarget target;target.kind=NavigationKind::GlobalPosition;target.latitude=op.latitude;target.longitude=op.longitude;target.height=op.height;
    target.facing=op.facing;target.facing_heading=op.heading;
    if(!m_backend.navigationInputAllowed(target)) {report(op,"REJECTED","target_outside_profile");return;}
    if(op.strategy=="helm") {
      double x=0,y=0;
      if(!m_services.projectGlobal(op.latitude,op.longitude,x,y)||!std::isfinite(x)||!std::isfinite(y)) {report(op,"REJECTED","configured_projection_required");return;}
      // One acquire clears the adapter cache. A same-stamp release followed by
      // acquire would correctly be rejected by its monotonic request boundary.
      m_services.revokeNavigation("helm_operation_start");
      std::ostringstream hold;hold<<std::setprecision(12)<<"station_pt=x="<<x<<",y="<<y<<",label=op_"<<op.id
        <<"#center_activate=false#inner_radius="<<op.radius*.5<<"#outer_radius="<<std::max(.5,op.radius*2)
        <<"#outer_speed="<<std::min(1.,op.speed)<<"#transit_speed="<<op.speed;
      m_services.publish("MAVMOOS_OPERATION_HOLD_UPDATE",hold.str());
      m_services.publish("MAVMOOS_OPERATION_ALTITUDE_UPDATE","altitude="+std::to_string(op.height));
      // Stock StationKeep supplies transit and arrival holding for one fixed
      // point. Completion is independently observed here, not a solver endflag.
      m_services.publish("MAVMOOS_HELM_OPERATION",op.action=="hold"?"hold":"route");
      m_helm_phase="ACQUIRING";m_helm_at=now;
      m_active=op;report(op,"ADMITTED","configured_stock_helm_strategy");
      helmControl("acquire");
      if(!m_services.navigationGranted())finish("REJECTED","host_acquisition_inhibited");
    } else {
      if(op.action=="adjust_altitude"&&m_services.navigationGranted()) {
        report(op,"REJECTED","release_helm_before_guided_adjustment");return;
      }
      op.needs_guided_entry=!m_backend.activeModeAllowed(vehicle,false);
      stopHelm();
      if(!m_services.acquireNavigation("operation_"+op.id)) {report(op,"REJECTED","host_acquisition_inhibited");return;}
      m_active=op;report(op,"ADMITTED","guided_position_strategy");
    }
  }
  void visualizeHome(const Operation& op)
  {
    const auto& vehicle=m_services.vehicle();
    if(!vehicle.home_valid||!fresh(vehicle.heartbeat_valid,vehicle.heartbeat_at,m_services.now(),2)) {
      report(op,"REJECTED","home_unavailable");return;
    }
    double x=0,y=0;
    if(!m_services.projectGlobal(vehicle.home_latitude,vehicle.home_longitude,x,y)||!std::isfinite(x)||!std::isfinite(y)) {
      report(op,"REJECTED","configured_projection_required");return;
    }
    std::ostringstream marker;
    marker<<std::setprecision(12)<<"x="<<x<<",y="<<y<<",label=MAVMOOS_HOME,type=gateway,width=5,active=true";
    m_services.publish("VIEW_MARKER",marker.str());
    report(op,"COMPLETED","home_marker_published");
  }
  void rememberDemand(std::optional<Mail>& retained,const Mail& mail,unsigned index)
  {
    if((!retained||mail.published_at>retained->published_at)&&m_backend.helmFieldAllowed(index,mail.value)) retained=mail;
  }
  void adjustHelmBehavior(const Operation& op,double now)
  {
    const bool altitude=op.action=="adjust_altitude";
    const auto& demand=altitude?m_helm_height:m_helm_speed;
    if(!m_services.navigationGranted()||m_helm_phase!="ACTIVE"||m_helm_at>now||now-m_helm_at>.5||
       !m_backend.flightReady(m_services.vehicle(),now,true)||!demand||
       demand->published_at>now||now-demand->published_at>.5) {
      report(op,"REJECTED","fresh_active_helm_demand_required");return;
    }
    const double value=op.action=="reset_speed_min"?m_min_speed:demand->value+op.delta;
    if(!m_backend.helmFieldAllowed(altitude?2:1,value)) {report(op,"REJECTED","adjustment_outside_profile");return;}
    m_services.publish(altitude?"MAVMOOS_OPERATION_ALTITUDE_UPDATE":"MAVMOOS_HELM_SPEED_UPDATE",
      std::string(altitude?"altitude=":"speed=")+std::to_string(value));
    report(op,"ACCEPTED","stock_behavior_update_published");
  }
  void startHelmTask(Operation op,const Vehicle& vehicle,double now)
  {
    if(!m_services.navigationGranted()||(m_helm_phase!="ACTIVE"&&m_helm_phase!="ACQUIRING")||
       m_helm_at>=now||now-m_helm_at>.5||now<=m_helm_request_at||!m_backend.flightReady(vehicle,now,true)) {
      report(op,"REJECTED","fresh_guided_helm_permission_required");return;
    }
    op.home=vehicle;
    // Retire any queued target and clear the adapter cache before selecting the
    // configured task. Only new stock-helm inputs can drive the new task.
    m_services.revokeNavigation("helm_task_switch");
    m_helm_speed.reset();m_helm_height.reset();
    m_helm_request_at=now;
    helmControl("acquire");
    if(!m_services.navigationGranted()) {report(op,"REJECTED","host_acquisition_inhibited");return;}
    m_helm_phase="ACQUIRING";m_helm_at=now;
    m_active=op;
    m_services.publish("MAVMOOS_HELM_OPERATION",op.action);
    report(op,"ADMITTED","configured_stock_helm_task");
  }
  void updateNavigation()
  {
    if(!m_active||m_active->strategy=="native")return;
    auto& op=*m_active;const double now=m_services.now();const auto& v=m_services.vehicle();
    if(!m_backend.flightReady(v,now,true))return;
    if(op.strategy=="helm"&&(m_helm_phase!="ACTIVE"||m_helm_at<op.started||now-m_helm_at>.5))return;
    if(!m_services.navigationGranted()) {finish("CANCELLED","navigation_authority_lost");return;}
    if(op.needs_guided_entry&&(op.guided_sent_at<0||v.heartbeat_at<=op.guided_sent_at))return;
    if(!op.confirmed) {op.confirmed=true;report(op,"CONFIRMED","current_guided_navigation_eligibility");}
    if(op.strategy=="guided"&&(op.last_target<0||now-op.last_target>=.095)) {
      NavigationTarget target;target.kind=NavigationKind::GlobalPosition;target.latitude=op.latitude;target.longitude=op.longitude;
      target.height=op.height;target.expires=std::min(op.expires,now+.5);target.facing=op.facing;target.facing_heading=op.heading;
      if(!m_services.navigation(target)) {finish("CANCELLED","host_target_inhibited");return;}
      op.last_target=now;
      if(!op.submitted) {op.submitted=true;report(op,"SUBMITTED","host_navigation_queue_admitted");}
    }
    if(op.action=="hold"||op.helmTask())return;
    const bool post= v.position_at>op.started&&fresh(v.position_valid,v.position_at,now,.5);
    const bool horizontal=post&&distance(v.latitude,v.longitude,op.latitude,op.longitude)<=op.radius;
    const double altitude_tolerance=op.action=="adjust_altitude"?std::min(.5,std::abs(op.delta)/2):.5;
    if(horizontal&&std::abs(v.relative_height-op.height)<=altitude_tolerance) {
      if(op.settled_at<0)op.settled_at=now;
      if(now-op.settled_at>=1)finish("COMPLETED","observed_goal_region_for_one_second");
    } else op.settled_at=-1;
  }
  void expireAndCheck()
  {
    const double now=m_services.now();
    if(m_clock_fault||!std::isfinite(now)||now<m_last_now) {
      m_clock_fault=true;
      if(m_active)finish("CANCELLED","clock_discontinuity");
      if(m_queued) {report(*m_queued,"CANCELLED","clock_discontinuity");m_queued.reset();}
      return;
    }
    m_last_now=now;
    if(!m_native_id.empty()&&now>=m_native_until) {
      m_services.commandFinished(m_native_id);
      // The reused native engine can have an earlier original bound than the
      // caller's upper limit. Publish the public terminal outcome before
      // retiring correlation, rather than discarding the helper's next timeout.
      if(m_active&&m_active->native_id==m_native_id)
        finish("TIMED_OUT",now>=m_active->expires?"original_operation_deadline":"native_original_deadline");
      clearNative();
    }
    if(!m_active)return;
    if(now>=m_active->expires) {finish("TIMED_OUT","original_operation_deadline");return;}
    if(m_active->strategy=="native")return;
    const auto& v=m_services.vehicle();const auto& home=m_active->home;
    if(!v.home_valid||v.home_latitude!=home.home_latitude||v.home_longitude!=home.home_longitude||v.home_msl!=home.home_msl) {finish("CANCELLED","home_reference_changed");return;}
    if(!m_backend.flightReady(v,now,false)||!v.armed||!v.in_air||
       !m_backend.activeModeAllowed(v,!m_active->helmTask()&&now-m_active->started<5)) {
      finish("CANCELLED","vehicle_prerequisite_lost");return;
    }
    if(now-m_active->started>=5&&!m_active->confirmed) {finish("TIMED_OUT","acquisition_readiness_deadline");return;}
    if(m_active->strategy=="guided"&&!m_services.navigationGranted()) {finish("CANCELLED","navigation_authority_lost");return;}
  }
  bool nativeQuarantined() const
  {return !m_native_id.empty()&&m_native_accepted&&m_services.now()<m_native_until;}
  void clearNative()
  {m_native_id.clear();m_native_until=0;m_native_accepted=false;m_native_terminal=false;}
  void helmControl(const std::string& value)
  {
    Mail mail;mail.key="MAVMOOS_NAV_CONTROL";mail.text=value;mail.source="copter_operations";
    mail.received_at=mail.published_at=m_services.now();
    m_helm_request_at=std::max(m_helm_request_at,mail.published_at);
    m_helm.onMail(mail);
  }
  void stopHelm()
  {
    m_services.publish("MAVMOOS_HELM_OPERATION","none");
    helmControl("release");
  }
  void finish(const std::string& status,const std::string& reason,bool release_helm=true)
  {
    if(!m_active)return;
    const auto op=*m_active;m_active.reset();
    if(op.strategy!="native") {
      if(release_helm)stopHelm();else m_services.publish("MAVMOOS_HELM_OPERATION","none");
      // A successful adapter release can leave a conditional host stop queued.
      // Keep it when closing a task or altitude adjustment.
      const bool keep_release_stop=op.helmTask()||op.action=="adjust_altitude";
      if(!keep_release_stop||!release_helm||m_services.navigationGranted())
        m_services.revokeNavigation("operation_"+status);
    }
    else if(status!="COMPLETED"&&!op.native_id.empty())m_services.cancelCommand(op.native_id);
    report(op,status,reason);
  }
  void report(const Operation& op,const std::string& status,const std::string& reason)
  {
    std::ostringstream value;
    value<<"schema=1,id="<<op.id<<",action="<<op.action<<",strategy="<<op.strategy
         <<",status="<<status<<",reason="<<reason<<",expires="<<std::fixed<<std::setprecision(9)<<op.expires
         <<",at="<<m_services.now();
    m_services.publish("MAVMOOS_CONTROL_RESULT",value.str());
  }
  Services& m_services;Plugin& m_native;Plugin& m_helm;const VehicleControlBackend& m_backend;
  std::optional<Operation> m_queued,m_active;std::set<std::string> m_seen,m_helm_tasks;
  bool m_native_enabled=true,m_helm_enabled=true,m_helm_configured=false,m_timeout_configured=false;
  bool m_helm_adjustments=false;
  double m_min_speed=.25;
  std::optional<Mail> m_helm_speed,m_helm_height;
  bool m_native_accepted=false,m_native_terminal=false,m_clock_fault=false;
  double m_boundary=0,m_last_now=0,m_timeout=60,m_radius=1,m_speed=1.5,m_native_until=0,m_helm_at=-1;
  double m_helm_request_at=-1;
  std::string m_native_id,m_last_cancelled,m_helm_source="pHelmIvP",m_helm_phase="DISABLED";
};
} // namespace
std::unique_ptr<CopterOperations> makeCopterOperations(Services& services,Plugin& native,Plugin& helm,
                                                     const VehicleControlBackend& backend)
{return std::make_unique<Operations>(services,native,helm,backend);}
} // namespace mavmoos::runtime
