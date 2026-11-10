/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: MissionPlugin.cpp                               */
/*    DATE: November 4, 2026                                */
/************************************************************/
#include "Plugin.h"
#include "QuadPlanePolicy.h"
#include "ArduPilotObservations.h"
#include "CopterFrame.h"
#include "ArduRoverMissionPolicy.h"
#include "FencePlan.h"
#include "PlaneOperationPolicy.h"
#include "Px4Offboard.h"
#include "Px4RoverPolicy.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>

namespace mavmoos::runtime {
namespace {
constexpr unsigned max_items=4096,max_history=128,max_retries=32,phase_retries=5;
struct PlanType {
  uint8_t value;
  const char* name;
  const char* snapshot;
};
constexpr PlanType plan_types[]={
  {MAV_MISSION_TYPE_MISSION,"flight","MAVMOOS_MISSION_INSPECTION_FLIGHT_SNAPSHOT"},
  {MAV_MISSION_TYPE_FENCE,"fence","MAVMOOS_MISSION_INSPECTION_FENCE_SNAPSHOT"},
  {MAV_MISSION_TYPE_RALLY,"rally","MAVMOOS_MISSION_INSPECTION_RALLY_SNAPSHOT"}
};
const PlanType* planType(const std::string& name)
{
  for(const auto& plan:plan_types) if(plan.name==name)return &plan;
  return nullptr;
}
bool validId(const std::string& text)
{
  return !text.empty()&&text.size()<=48&&std::all_of(text.begin(),text.end(),[](unsigned char c) {
    return (c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_'||c=='-'||c=='.';
  });
}
std::string trim(const std::string& text)
{
  const auto first=text.find_first_not_of(" \t\r\n");
  return first==std::string::npos?std::string{}:text.substr(first,text.find_last_not_of(" \t\r\n")-first+1);
}
bool number(const std::string& text,double& value)
{
  try {std::size_t used=0;value=std::stod(text,&used);return used==text.size()&&std::isfinite(value);}
  catch(...) {return false;}
}
bool integer(const std::string& text,int64_t low,int64_t high,int64_t& value)
{
  if(text.empty())return false;
  std::size_t start=text[0]=='-'?1:0;
  if(start==text.size()||!std::all_of(text.begin()+start,text.end(),[](unsigned char c){return c>='0'&&c<='9';}))return false;
  try {std::size_t used=0;value=std::stoll(text,&used);return used==text.size()&&value>=low&&value<=high;}
  catch(...) {return false;}
}
std::vector<std::string> split(const std::string& text,char separator,std::size_t limit)
{
  std::vector<std::string> result;std::istringstream stream(text);std::string part;
  while(std::getline(stream,part,separator)) {
    result.push_back(trim(part));
    if(result.size()>limit)return result;
  }
  if(!text.empty()&&text.back()==separator)result.emplace_back();
  return result;
}
bool finiteItem(const MissionItem& item)
{
  return std::all_of(item.params.begin(),item.params.end(),[](float x){return std::isfinite(x);})&&
    std::isfinite(item.z)&&item.current<=1&&item.autocontinue<=1;
}
bool inspectionItem(const MissionItem& item,uint8_t type)
{
  if(type==MAV_MISSION_TYPE_MISSION)return true;
  if(std::abs(double(item.x))>9e8||std::abs(double(item.y))>18e8)return false;
  const bool global=item.frame==MAV_FRAME_GLOBAL||item.frame==MAV_FRAME_GLOBAL_INT;
  if(type==MAV_MISSION_TYPE_RALLY)
    return item.command==MAV_CMD_NAV_RALLY_POINT &&
      (global||item.frame==MAV_FRAME_GLOBAL_RELATIVE_ALT||item.frame==MAV_FRAME_GLOBAL_RELATIVE_ALT_INT||
       item.frame==MAV_FRAME_GLOBAL_TERRAIN_ALT||item.frame==MAV_FRAME_GLOBAL_TERRAIN_ALT_INT);
  if(!global)return false;
  switch(item.command) {
    case MAV_CMD_NAV_FENCE_RETURN_POINT:return true;
    case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
    case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
      return item.params[0]>=3&&item.params[0]<=255&&std::trunc(item.params[0])==item.params[0];
    case MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
    case MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:return item.params[0]>0;
    default:return false;
  }
}
bool uploadItem(const MissionItem& item)
{
  if(!finiteItem(item)||item.autocontinue!=1)return false;
  if(item.seq==0)return item.command==MAV_CMD_NAV_WAYPOINT&&item.frame==MAV_FRAME_GLOBAL&&
    std::all_of(item.params.begin(),item.params.end(),[](float x){return x==0;})&&
    std::abs(double(item.x))<=9e8&&std::abs(double(item.y))<=18e8&&item.z>=-1000&&item.z<=20000;
  if(item.command==MAV_CMD_NAV_RETURN_TO_LAUNCH)return item.frame==MAV_FRAME_MISSION&&item.x==0&&item.y==0&&item.z==0&&
    std::all_of(item.params.begin(),item.params.end(),[](float x){return x==0;});
  if(item.command!=MAV_CMD_NAV_WAYPOINT&&item.command!=MAV_CMD_NAV_TAKEOFF&&item.command!=MAV_CMD_NAV_LAND)return false;
  if(item.frame!=MAV_FRAME_GLOBAL&&item.frame!=MAV_FRAME_GLOBAL_RELATIVE_ALT)return false;
  if(std::abs(double(item.x))>9e8||std::abs(double(item.y))>18e8||item.z< -1000||item.z>20000)return false;
  // This initial storage profile avoids lossy vehicle-specific mission params.
  if(item.command==MAV_CMD_NAV_WAYPOINT) {
    if(item.params[0]<0||item.params[0]>600||std::trunc(item.params[0])!=item.params[0])return false;
  } else if(item.params[0]!=0)return false;
  return item.params[1]==0&&item.params[2]==0&&item.params[3]==0;
}
enum class UploadProfile { Generic, Px4Rover, Plane, QuadPlane };
bool parseItems(const std::string& text,std::vector<MissionItem>& result,UploadProfile profile)
{
  const bool rover=profile==UploadProfile::Px4Rover;
  const bool quad=profile==UploadProfile::QuadPlane;
  const bool plane_profile=profile==UploadProfile::Plane;
  const auto rows=split(text,'|',rover?8:max_items);
  if(rows.empty()||rows.size()>(rover?8:max_items)||(rover&&rows.size()<2)||
     (quad&&rows.size()!=quadplane::plan_items)||
     (plane_profile&&rows.size()!=plane::lifecycle_items))return false;
  for(const auto& row:rows) {
    const auto values=split(row,':',12);if(values.size()!=12)return false;
    MissionItem item;int64_t value=0;
    if(!integer(values[0],0,max_items-1,value)||value!=int64_t(result.size()))return false;
    item.seq=uint16_t(value);
    if(!integer(values[1],0,65535,value))return false;
    item.command=uint16_t(value);
    if(!integer(values[2],0,255,value))return false;
    item.frame=uint8_t(value);
    if(!integer(values[3],0,1,value))return false;
    item.current=uint8_t(value);
    if(!integer(values[4],0,1,value))return false;
    item.autocontinue=uint8_t(value);
    for(unsigned i=0;i<4;++i) {double x=0;if(!number(values[5+i],x)||std::abs(x)>std::numeric_limits<float>::max())return false;item.params[i]=float(x);}
    if(!integer(values[9],INT32_MIN,INT32_MAX,value))return false;
    item.x=int32_t(value);
    if(!integer(values[10],INT32_MIN,INT32_MAX,value))return false;
    item.y=int32_t(value);
    double z=0;if(!number(values[11],z)||std::abs(z)>std::numeric_limits<float>::max())return false;item.z=float(z);
    if(!(rover?px4_rover::validWaypoint(item):quad?quadplane::validItem(item):
         plane_profile?plane::lifecycleItem(item):uploadItem(item)))return false;
    result.push_back(item);
  }
  return true;
}
bool parseFence(const std::string& text,std::vector<MissionItem>& result)
{
  const auto points=split(text,'|',maximum_fence_vertices);
  if(points.size()<3||points.size()>maximum_fence_vertices)return false;
  for(const auto& point:points) {
    const auto coordinates=split(point,':',2);
    int64_t lat=0,lon=0;
    if(coordinates.size()!=2||!integer(coordinates[0],-850000000,850000000,lat)||
       !integer(coordinates[1],-1800000000,1800000000,lon))return false;
    MissionItem item;
    item.seq=uint16_t(result.size());
    item.command=MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION;
    item.frame=MAV_FRAME_GLOBAL;item.autocontinue=0;
    item.params[0]=float(points.size());item.x=int32_t(lat);item.y=int32_t(lon);
    result.push_back(item);
  }
  return editableFence(result);
}

MissionItem decodeItem(const mavlink_mission_item_int_t& item)
{
  MissionItem result;result.seq=item.seq;result.command=item.command;result.frame=item.frame;
  result.current=item.current;result.autocontinue=item.autocontinue;
  result.params={item.param1,item.param2,item.param3,item.param4};result.x=item.x;result.y=item.y;result.z=item.z;
  return result;
}
bool sameItem(const MissionItem& a,const MissionItem& b,bool ignore_current=false,bool tolerance=false)
{
  if(a.seq!=b.seq||a.command!=b.command||a.frame!=b.frame||(!ignore_current&&a.current!=b.current)||
     a.autocontinue!=b.autocontinue||a.x!=b.x||a.y!=b.y)return false;
  for(unsigned i=0;i<4;++i)if(tolerance?std::abs(double(a.params[i])-b.params[i])>1e-5:a.params[i]!=b.params[i])return false;
  return tolerance?std::abs(double(a.z)-b.z)<=.010001:a.z==b.z;
}
bool samePx4Waypoint(MissionItem expected,const MissionItem& observed)
{
  if(px4_rover::relativeFrame(expected.frame) && px4_rover::relativeFrame(observed.frame))
    expected.frame=observed.frame;
  return sameItem(expected,observed,true);
}
bool sameUploadedItem(MissionItem expected,const MissionItem& observed)
{
  // AP_Mission.cpp at pinned 04cf6486: non-positional RTL reads with the
  // zero-initialized GLOBAL frame; LAND's stored false loiter_ccw returns +1.
  // Only these exact default representations are equivalent. Keep raw rows
  // unchanged and retain all unrelated frame/parameter comparisons.
  if(expected.command==MAV_CMD_NAV_RETURN_TO_LAUNCH&&expected.frame==MAV_FRAME_MISSION&&observed.frame==MAV_FRAME_GLOBAL)
    expected.frame=MAV_FRAME_GLOBAL;
  if(expected.command==MAV_CMD_NAV_LAND&&expected.params[3]==0&&observed.params[3]==1)
    expected.params[3]=1;
  return sameItem(expected,observed,true,true);
}
bool sameRequest(const MissionRequest& a,const MissionRequest& b)
{
  return a.id==b.id&&a.operation_id==b.operation_id&&a.action==b.action&&a.mission_type==b.mission_type&&
    a.seq==b.seq&&a.count==b.count&&a.result==b.result&&a.expires==b.expires&&a.operation_expires==b.operation_expires&&sameItem(a.item,b.item);
}
enum class Stage { DownloadCount,DownloadItem,DownloadAck,UploadCount,UploadItem,ClearAck };
enum class Download { Observation,CompareFenceBackup,VerifyUpload,VerifyClear };
struct Phase {
  MissionRequest request;
  double submitted=0,sent=0,reply_by=0;
  bool accepted=false;
};
struct FenceBackup {
  std::string id;
  std::vector<MissionItem> items;
  bool restore_required=false;
};
struct Operation {
  std::string id,action,backup_id;
  bool inspection=false;
  const PlanType* plan=nullptr;
  double expires=0;
  Stage stage=Stage::DownloadCount;
  Download download=Download::Observation;
  std::vector<MissionItem> upload,items;
  std::optional<Phase> phase;
  unsigned expected=0,seq=0,retries=0,current_retries=0,phases=0,legacy_requests=0;
  uint32_t opaque_id=0;
  bool mutation_may_have_applied=false,remote_accepted=false,readback_confirmed=false;
};
class Missions final:public Plugin {
public:
  explicit Missions(Services& services):m_services(services),m_boundary(services.now()),m_last_now(services.now()) {}
  std::string name() const override {return "missions";}
  PluginDescriptor descriptor() const override {
    PluginDescriptor d;d.profile="ardupilot_mission_int";
    d.capabilities={{"missions.download","MAVMOOS_MISSION_REQUEST","action=download"},
      {"missions.upload.simulator","MAVMOOS_MISSION_REQUEST","action=upload"},
      {"missions.clear.simulator","MAVMOOS_MISSION_REQUEST","action=clear"},
      {"missions.cancel","MAVMOOS_MISSION_REQUEST","action=cancel"},
      {"missions.inspect.flight","MAVMOOS_MISSION_REQUEST","schema=1,action=inspect,type=flight"},
      {"missions.inspect.fence","MAVMOOS_MISSION_REQUEST","schema=1,action=inspect,type=fence"},
      {"missions.inspect.rally","MAVMOOS_MISSION_REQUEST","schema=1,action=inspect,type=rally"},
      {"missions.progress","","","observation"}};
    if(m_fence_edit) {
      d.capabilities.push_back({"missions.fence.save","MAVMOOS_MISSION_REQUEST","schema=1,action=save,type=fence"});
      d.capabilities.push_back({"missions.fence.replace.simulator","MAVMOOS_MISSION_REQUEST","schema=1,action=replace,type=fence"});
      d.capabilities.push_back({"missions.fence.restore.simulator","MAVMOOS_MISSION_REQUEST","schema=1,action=restore,type=fence"});
    }
    d.exclusive_resources={"protocol.missions"};d.request_owners={"MAVMOOS_MISSION_REQUEST"};
    if(!m_px4_rover) d.observation_streams=arduPilotObservationStreams();
    return d;
  }
  std::vector<std::string> subscriptions() const override {return {"MAVMOOS_MISSION_REQUEST"};}
  void configure(const std::string& key,const std::string& value) override {
    if(key=="vehicle_profile") {
      m_px4_rover=value==px4::rover_profile;
      m_quadplane=value==quadplane::profile;
      m_plane=value=="arduplane";
      m_ardu_rover=value=="ardurover"||value=="ardurover_boat";
      return;
    }
    if(key=="control_mission_execution") {m_native_mission=value=="true";return;}
    if(key!="mission_timeout"&&key!="mission_response_timeout")return;
    double x=0;if(!number(value,x)||x<(key=="mission_timeout"?1:.25)||x>(key=="mission_timeout"?120:1.5))throw std::invalid_argument("invalid "+key);
    (key=="mission_timeout"?m_timeout:m_response_timeout)=x;
  }
  ModuleConfigResult configureModule(const std::vector<ModuleSetting>& settings) override {
    bool enabled=m_fence_edit;
    for(const auto& setting:settings) {
      if(setting.key!="fence_edit")
        return {ModuleConfigStatus::UnknownKey,setting.key,"unknown mission module option"};
      if(setting.value!="true"&&setting.value!="false")
        return {ModuleConfigStatus::Invalid,setting.key,"expected true or false"};
      enabled=setting.value=="true";
    }
    if(m_px4_rover && enabled)
      return {ModuleConfigStatus::Invalid,"fence_edit","requires the Copter storage profile"};
    m_fence_edit=enabled;
    return {};
  }
  void onMail(const Mail& mail) override {
    if(mail.key!="MAVMOOS_MISSION_REQUEST")return;
    Operation operation;operation.id="invalid";operation.action="invalid";
    std::map<std::string,std::string> fields;bool valid=!mail.numeric&&!mail.text.empty()&&mail.text.size()<=1024*1024;
    if(!valid){report(operation,"REJECTED","invalid_request_or_unsupported_upload_profile");return;}
    const auto parts=split(mail.text,',',7);
    if(parts.size()>7)valid=false;
    for(const auto& part:parts) {
      const auto at=part.find('=');if(at==std::string::npos){valid=false;break;}
      const auto key=trim(part.substr(0,at)),value=trim(part.substr(at+1));
      if((key!="id"&&key!="action"&&key!="items"&&key!="timeout"&&key!="schema"&&key!="type"&&key!="backup")||
         !fields.emplace(key,value).second){valid=false;break;}
    }
    if(fields.count("id")&&validId(fields["id"]))operation.id=fields["id"];else valid=false;
    const auto action=fields["action"];
    operation.inspection=fields.count("schema")||action=="inspect";
    const bool saving=action=="save",replacing=action=="replace",restoring=action=="restore";
    const bool fence_edit=saving||replacing||restoring;
    if(operation.inspection) {
      operation.plan=planType(fields["type"]);
      if(action=="inspect"||action=="cancel"||fence_edit)operation.action=action;else valid=false;
      if(fields["schema"]!="1"||!operation.plan)valid=false;
      if(fence_edit&&(!operation.plan||operation.plan->value!=MAV_MISSION_TYPE_FENCE))valid=false;
    } else {
      if(action=="download"||action=="upload"||action=="clear"||action=="cancel")operation.action=action;else valid=false;
      if(fields.count("type"))valid=false;
    }
    double duration=m_timeout;
    if(fields.count("timeout")&&(!number(fields["timeout"],duration)||duration<1||duration>120))valid=false;
    const auto upload_profile=m_px4_rover?UploadProfile::Px4Rover:m_quadplane?UploadProfile::QuadPlane:
      m_plane?UploadProfile::Plane:UploadProfile::Generic;
    if(replacing)valid=valid&&fields.count("items")&&parseFence(fields["items"],operation.upload);
    else if(operation.action=="upload")valid=valid&&fields.count("items")&&
      parseItems(fields["items"],operation.upload,upload_profile);
    else valid=valid&&!fields.count("items");
    if(replacing||restoring) {
      if(fields.count("backup")&&validId(fields["backup"]))operation.backup_id=fields["backup"];else valid=false;
    } else {
      if(fields.count("backup"))valid=false;
      if(saving)operation.backup_id=operation.id;
    }
    if(operation.action=="cancel"&&fields.count("timeout"))valid=false;
    if(!valid){report(operation,"REJECTED",operation.inspection?"invalid_inspection_request":"invalid_request_or_unsupported_upload_profile");return;}
    if(fence_edit&&!m_fence_edit){report(operation,"REJECTED","fence_edit_disabled");return;}
    const double now=m_services.now(),age=mail.received_at-mail.published_at;
    if(!std::isfinite(now)||!std::isfinite(mail.received_at)||!std::isfinite(mail.published_at)||!std::isfinite(age)||
       age<-.1||age>2||mail.received_at>now+.1||now-mail.received_at>2||mail.published_at<=m_boundary) {
      report(operation,"REJECTED","stale_future_or_pre_session_request");return;
    }
    if(m_quadplane&&action=="upload"&&(!quadplane::observed(m_services.vehicle(),now)||
       !quadplane::validPlan(operation.upload,m_services.vehicle()))) {
      report(operation,"REJECTED","quadplane_plan_outside_bound_home_geometry");return;
    }
    if(arduRoverExecution()&&operation.action=="upload"&&
       !ardu_rover_mission::planValid(operation.upload,m_services.vehicle())) {
      report(operation,"REJECTED","rover_plan_or_home_outside_profile");return;
    }
    if(operation.action=="cancel") {
      if(m_operation&&m_operation->id==operation.id&&m_operation->inspection==operation.inspection&&
         m_operation->plan==operation.plan)finish("CANCELLED","operator_cancelled");
      else report(operation,"REJECTED","operation_not_active");
      return;
    }
    if(m_seen.count(operation.id)){report(operation,"DUPLICATE","id_already_seen");return;}
    if(m_operation){report(operation,"REJECTED","busy");return;}
    if(m_seen.size()>=max_history){report(operation,"REJECTED","request_history_full");return;}
    if(m_plane&&operation.action=="upload"&&
       (!plane::freshState(m_services.vehicle().home_at,now,3)||!plane::lifecyclePlan(operation.upload,m_services.vehicle()))) {
      report(operation,"REJECTED","plane_runway_geometry_unqualified");return;
    }
    if(fence_edit&&(m_services.vehicle().type!=MAV_TYPE_QUADROTOR||!mutationReady(now))) {
      report(operation,"REJECTED","fence_edit_requires_disarmed_ground_copter");return;
    }
    if((saving||replacing)&&m_fence_backup&&m_fence_backup->restore_required) {
      report(operation,"REJECTED","verified_restore_required");return;
    }
    if((replacing||restoring)&&(!m_fence_backup||m_fence_backup->id!=operation.backup_id)) {
      report(operation,"REJECTED","matching_fence_backup_required");return;
    }
    operation.expires=mail.published_at+duration;
    if(operation.expires<=now){report(operation,"REJECTED","expired_request");return;}
    if(restoring)operation.upload=m_fence_backup->items;
    if(replacing)operation.download=Download::CompareFenceBackup;
    else if(operation.action=="upload"||(restoring&&!operation.upload.empty()))operation.stage=Stage::UploadCount;
    else if(operation.action=="clear"||restoring)operation.stage=Stage::ClearAck;
    operation.expected=unsigned(operation.upload.size());
    m_seen.insert(operation.id);m_operation=std::move(operation);
    report(*m_operation,"ADMITTED","request_validated_pending_live_policy");
  }
  void onReset(const std::string&) override {
    if(m_operation)finish("CANCELLED","connection_reset");
    m_fence_backup.reset();
    const double now=m_services.now();if(std::isfinite(now))m_boundary=std::max(m_boundary,now);
    m_progress_at=0;
    m_services.publish("MAVMOOS_MISSION_SNAPSHOT","{\"schema\":1,\"status\":\"invalidated\",\"reason\":\"connection_reset\",\"count\":0,\"items\":[]}");
    ++m_session_epoch;
    for(const auto& plan:plan_types)
      m_services.publish(plan.snapshot,"{\"schema\":1,\"status\":\"invalidated\",\"reason\":\"connection_reset\",\"type\":\""+
        std::string(plan.name)+"\",\"mission_type\":"+std::to_string(plan.value)+",\"session_epoch\":"+
        std::to_string(m_session_epoch)+",\"count\":0,\"items\":[]}");
    m_services.publish("MAVMOOS_MISSION_PROGRESS","schema=1,status=invalidated,reason=connection_reset");
  }
  void onInputBoundary() override {boundary();}
  void iterate() override {
    if(!boundary()||!m_operation)return;
    auto& op=*m_operation;
    if(op.phase) {
      if(!op.phase->accepted) {
        if(m_services.now()>=op.phase->request.expires)finish("TIMED_OUT","local_send_receipt_missing");
        return;
      }
      if(m_services.now()<op.phase->reply_by)return;
      retry("response_timeout");return;
    }
    submit();
  }
  void onMissionSent(const MissionRequest& request,double at,bool accepted) override {
    if(!m_operation||!m_operation->phase||!sameRequest(request,m_operation->phase->request))return;
    auto& op=*m_operation;auto& phase=*op.phase;
    if(phase.accepted)return;
    if(mutation(request.action)&&accepted)op.mutation_may_have_applied=true;
    if(!accepted){op.phase.reset();finish("FAILED","local_send_rejected_or_failed");return;}
    if(!std::isfinite(at)||at<phase.submitted||at>m_services.now()+.1||at>=request.expires||at>=op.expires) {
      finish("TIMED_OUT","local_send_outside_original_deadline");return;
    }
    phase.accepted=true;phase.sent=at;phase.reply_by=std::min(op.expires,at+m_response_timeout);
    report(op,"SENT","local_os_acceptance");
    if(op.stage==Stage::DownloadAck)completeDownload();
  }
  void onMessage(const mavlink_message_t& message,double at) override {
    onMessageWithMetadata(message,{at,at,"READ_ACQUISITION"});
  }
  void onMessageWithMetadata(const mavlink_message_t& message,const MessageTime& time) override {
    const double acquired=time.read_at,now=m_services.now();
    if(message.sysid!=1||message.compid!=1||!std::isfinite(acquired)||acquired<0||acquired>now+.1)return;
    const bool transfer_live=boundary();
    if(message.msgid==MAVLINK_MSG_ID_MISSION_CURRENT||message.msgid==MAVLINK_MSG_ID_MISSION_ITEM_REACHED)
      publishProgress(message,time,now);
    if(!transfer_live||!m_operation||!m_operation->phase||!m_operation->phase->accepted)return;
    const auto& phase=*m_operation->phase;
    if(acquired<=phase.sent||acquired>=phase.reply_by)return;
    if(m_operation->inspection&&(message.magic!=MAVLINK_STX||acquired<=m_boundary))return;
    if(message.msgid==MAVLINK_MSG_ID_MISSION_COUNT) {
      mavlink_mission_count_t count{};mavlink_msg_mission_count_decode(&message,&count);
      if(!target(count.target_system,count.target_component,count.mission_type))return;
      handleCount(count);return;
    }
    if(message.msgid==MAVLINK_MSG_ID_MISSION_ITEM_INT) {
      mavlink_mission_item_int_t item{};mavlink_msg_mission_item_int_decode(&message,&item);
      if(!target(item.target_system,item.target_component,item.mission_type))return;
      handleItem(decodeItem(item));return;
    }
    if(message.msgid==MAVLINK_MSG_ID_MISSION_REQUEST_INT||message.msgid==MAVLINK_MSG_ID_MISSION_REQUEST) {
      // Pinned ArduPilot uses MISSION_REQUEST (40) for every upload type.
      // The versioned path above still requires MAVLink2 with the exact type.
      if(m_operation->inspection&&(m_operation->backup_id.empty()||
         (m_operation->stage!=Stage::UploadCount&&m_operation->stage!=Stage::UploadItem)))return;
      unsigned seq=0;
      if(message.msgid==MAVLINK_MSG_ID_MISSION_REQUEST_INT) {
        mavlink_mission_request_int_t request{};mavlink_msg_mission_request_int_decode(&message,&request);
        if(!target(request.target_system,request.target_component,request.mission_type))return;
        seq=request.seq;
      } else {
        mavlink_mission_request_t request{};mavlink_msg_mission_request_decode(&message,&request);
        if(!target(request.target_system,request.target_component,request.mission_type))return;
        seq=request.seq;
        ++m_operation->legacy_requests;
      }
      handleRequest(seq);return;
    }
    if(message.msgid==MAVLINK_MSG_ID_MISSION_ACK) {
      mavlink_mission_ack_t ack{};mavlink_msg_mission_ack_decode(&message,&ack);
      if(!target(ack.target_system,ack.target_component,ack.mission_type))return;
      handleAck(ack.type);
    }
  }
private:
  bool arduRoverExecution() const {return m_ardu_rover&&m_native_mission;}
  static bool mutation(MissionAction action){return action==MissionAction::Count||action==MissionAction::Item||action==MissionAction::Clear;}
  bool target(unsigned system,unsigned component,unsigned type) const {
    return system==245&&component==191&&type==m_operation->phase->request.mission_type;
  }
  static std::string stamp(double value){std::ostringstream out;out<<std::fixed<<std::setprecision(9)<<value;return out.str();}
  bool heartbeat(double now) const {
    const auto& v=m_services.vehicle();
    return (m_px4_rover?v.autopilot==MAV_AUTOPILOT_PX4&&v.type==MAV_TYPE_GROUND_ROVER:
      v.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA) && v.heartbeat_valid &&
      v.heartbeat_at>=0 && now>=v.heartbeat_at && now-v.heartbeat_at<=2;
  }
  void publishProgress(const mavlink_message_t& message,const MessageTime& time,double now) {
    if(!heartbeat(now)||m_services.vehicle().heartbeat_at<=m_boundary||
       !std::isfinite(time.effective_at)||time.effective_at<=m_boundary||time.effective_at<m_progress_at||
       time.effective_at>time.read_at||time.read_at>now||now-time.effective_at>2||
       (time.age_basis!="READ_ACQUISITION"&&time.age_basis!="QUEUE_EMPTY_FLOOR"))return;
    m_progress_at=time.effective_at;
    const auto stamps=",effective_at="+stamp(time.effective_at)+",read_at="+stamp(time.read_at)+
      ",age_basis="+time.age_basis+",source_age=unknown";
    if(message.msgid==MAVLINK_MSG_ID_MISSION_CURRENT) {
      mavlink_mission_current_t current{};mavlink_msg_mission_current_decode(&message,&current);
      const auto total=current.total==0?"unknown":current.total==UINT16_MAX?"none":std::to_string(current.total);
      m_services.publish("MAVMOOS_MISSION_PROGRESS","schema=1,status=observed,kind=current,seq="+std::to_string(current.seq)+
        ",total="+total+",total_wire="+std::to_string(current.total)+
        ",total_excludes_home="+(m_px4_rover?"false":"true")+",mission_state="+
        (current.mission_state==0?"unknown":std::to_string(current.mission_state))+",mission_mode="+
        (current.mission_mode==0?"unknown":std::to_string(current.mission_mode))+stamps);
    } else {
      mavlink_mission_item_reached_t reached{};mavlink_msg_mission_item_reached_decode(&message,&reached);
      m_services.publish("MAVMOOS_MISSION_PROGRESS","schema=1,status=observed,kind=reached,seq="+std::to_string(reached.seq)+stamps);
    }
  }
  bool mutationReady(double now) const {
    const auto& v=m_services.vehicle();
    return heartbeat(now)&&!v.armed&&!m_services.navigationGranted()&&!m_services.controlBusy()&&
      (!supportedCopterType(v.type)||(v.landed_valid&&now>=v.landed_at&&now-v.landed_at<=2&&v.landed_state==MAV_LANDED_STATE_ON_GROUND));
  }
  bool boundary() {
    const double now=m_services.now();
    if(!std::isfinite(now)||now<m_last_now) {
      if(m_operation)finish("FAILED","clock_fault");
      m_boundary=std::max(m_boundary,m_last_now);return false;
    }
    m_last_now=now;
    if(!m_operation)return true;
    if(now>=m_operation->expires){finish("TIMED_OUT","original_operation_deadline");return false;}
    if(!heartbeat(now)||(m_operation->inspection&&m_services.vehicle().heartbeat_at<=m_boundary)) {
      finish("CANCELLED","heartbeat_unavailable");return false;
    }
    // New mutation stops when state changes. Independently safe readback may
    // still resolve an accepted write after external arming. Our own retained
    // mission lease is not a native command transition in controlBusy().
    const auto stage=m_operation->stage;
    if(arduRoverExecution()&&(stage==Stage::UploadCount||stage==Stage::UploadItem)&&
       !ardu_rover_mission::planValid(m_operation->upload,m_services.vehicle())) {
      finish("FAILED","rover_plan_or_home_outside_profile");return false;
    }
    if((stage==Stage::UploadCount||stage==Stage::UploadItem||stage==Stage::ClearAck)&&!mutationReady(now)) {
      finish("FAILED","mutation_state_no_longer_permitted");return false;
    }
    if(m_plane&&(stage==Stage::UploadCount||stage==Stage::UploadItem)&&
       (!plane::freshState(m_services.vehicle().home_at,now,3)||!plane::lifecyclePlan(m_operation->upload,m_services.vehicle()))) {
      finish("FAILED","plane_runway_geometry_lost");return false;
    }
    return true;
  }
  MissionRequest nextRequest() const {
    const auto& op=*m_operation;MissionRequest request;request.operation_id=op.id;request.operation_expires=op.expires;
    if(op.plan)request.mission_type=op.plan->value;
    switch(op.stage) {
      case Stage::DownloadCount:request.action=MissionAction::RequestList;break;
      case Stage::DownloadItem:request.action=MissionAction::RequestItem;request.seq=uint16_t(op.seq);break;
      case Stage::DownloadAck:request.action=MissionAction::Ack;break;
      case Stage::UploadCount:request.action=MissionAction::Count;request.count=uint16_t(op.upload.size());break;
      case Stage::UploadItem:
        request.action=MissionAction::Item;request.seq=uint16_t(op.seq);request.item=op.upload[op.seq];
        if(op.plan)request.count=uint16_t(op.upload.size());
        break;
      case Stage::ClearAck:request.action=MissionAction::Clear;break;
    }
    return request;
  }
  void submit() {
    auto& op=*m_operation;auto request=nextRequest();
    if(mutation(request.action)&&!mutationReady(m_services.now())){finish("REJECTED","mutation_requires_disarmed_idle_simulator");return;}
    request.id=op.id+"-"+std::to_string(++m_phase_nonce);
    request.expires=std::min(op.expires,m_services.now()+1);
    op.phase=Phase{request,m_services.now(),0,0,false};++op.phases;
    if(!m_services.mission(request)){op.phase.reset();finish("FAILED","host_admission_inhibited");return;}
    // Submission is not an OS send receipt or remote progress.
    report(op,"SUBMITTED","host_queue_admitted");
  }
  void retry(const std::string& reason) {
    auto& op=*m_operation;
    if(op.retries>=max_retries||op.current_retries>=phase_retries){finish("TIMED_OUT","bounded_retries_exhausted");return;}
    ++op.retries;++op.current_retries;op.phase.reset();
    report(op,"RETRY_DUE",reason);
  }
  void advance(Stage stage,unsigned seq=0) {
    auto& op=*m_operation;op.stage=stage;op.seq=seq;op.current_retries=0;op.phase.reset();
  }
  void handleCount(const mavlink_mission_count_t& count) {
    auto& op=*m_operation;
    if(op.stage==Stage::DownloadItem) {
      if(count.count!=op.expected)finish("INCOMPLETE","mission_count_changed");
      else if(op.opaque_id&&count.opaque_id&&count.opaque_id!=op.opaque_id)
        finish("INCOMPLETE","mission_plan_id_changed");
      return;
    }
    if(op.stage!=Stage::DownloadCount)return;
    if(count.count>max_items){finish("INCOMPLETE","mission_count_exceeds_bound");return;}
    if(op.download==Download::VerifyUpload&&count.count!=op.upload.size()){finish("FAILED","upload_readback_count_mismatch");return;}
    if(op.download==Download::VerifyClear&&count.count!=0){finish("FAILED","clear_readback_not_empty");return;}
    op.expected=count.count;op.opaque_id=count.opaque_id;op.items.clear();op.items.reserve(count.count);
    advance(count.count?Stage::DownloadItem:Stage::DownloadAck);
    report(op,"COUNT_RECEIVED","post_send_mission_count");
  }
  void handleItem(const MissionItem& item) {
    auto& op=*m_operation;if(op.stage!=Stage::DownloadItem)return;
    if(item.seq<op.items.size()) {
      if(!sameItem(item,op.items[item.seq],true))finish("INCOMPLETE","mission_item_changed");
      return;
    }
    if(item.seq!=op.seq)return;
    if(!finiteItem(item)){finish("INCOMPLETE","unsupported_nonfinite_or_flagged_mission_item");return;}
    if(op.plan&&!inspectionItem(item,op.plan->value)){finish("INCOMPLETE","invalid_plan_item_shape");return;}
    op.items.push_back(item);
    advance(op.items.size()==op.expected?Stage::DownloadAck:Stage::DownloadItem,unsigned(op.items.size()));
    report(op,"ITEM_RECEIVED","post_send_requested_item");
  }
  void handleRequest(unsigned seq) {
    auto& op=*m_operation;
    if(op.stage==Stage::UploadCount) {
      if(seq!=0)return;
      advance(Stage::UploadItem,0);return;
    }
    if(op.stage!=Stage::UploadItem)return;
    if(seq==op.seq) {retry("peer_re_requested_item");return;}
    if(seq==op.seq+1&&seq<op.upload.size())advance(Stage::UploadItem,seq);
  }
  void handleAck(unsigned result) {
    auto& op=*m_operation;
    if(result!=MAV_MISSION_ACCEPTED) {
      if(op.inspection&&op.stage==Stage::DownloadCount&&result==MAV_MISSION_UNSUPPORTED) {
        finish("UNSUPPORTED","remote_mission_type_unsupported");return;
      }
      // Pinned ArduPilot may request the next item again after rejecting a
      // duplicate previous item. Do not renew its reply deadline on that error.
      if(result==MAV_MISSION_INVALID_SEQUENCE&&op.stage==Stage::UploadItem)return;
      finish("FAILED","remote_mission_error_"+std::to_string(result));return;
    }
    if(op.stage==Stage::UploadItem&&op.seq+1==op.upload.size()) {
      op.remote_accepted=true;op.download=Download::VerifyUpload;
      report(op,"ACKNOWLEDGED","upload_accepted_readback_required");advance(Stage::DownloadCount);
    } else if(op.stage==Stage::ClearAck) {
      op.remote_accepted=true;op.download=Download::VerifyClear;
      report(op,"ACKNOWLEDGED","clear_accepted_readback_required");advance(Stage::DownloadCount);
    }
  }
  bool verifiedUpload() const {
    const auto& op=*m_operation;if(op.upload.size()!=op.items.size()||op.items.empty())return false;
    if(op.plan)return sameFence(op.upload,op.items);
    if(m_px4_rover) return op.opaque_id!=0 && px4_rover::validMission(op.items) &&
      std::equal(op.upload.begin(),op.upload.end(),op.items.begin(),samePx4Waypoint);
    // ArduPilot reads sequence zero from current HOME rather than the uploaded
    // placeholder. Its complete observed value is retained in the snapshot.
    if(op.items[0].seq!=0||op.items[0].command!=MAV_CMD_NAV_WAYPOINT||op.items[0].frame!=MAV_FRAME_GLOBAL)return false;
    for(unsigned i=1;i<op.items.size();++i)if(!sameUploadedItem(op.upload[i],op.items[i]))return false;
    return true;
  }
  static bool sameFence(const std::vector<MissionItem>& expected,const std::vector<MissionItem>& observed) {
    return expected.size()==observed.size()&&std::equal(expected.begin(),expected.end(),observed.begin(),
      [](const MissionItem& left,const MissionItem& right){return sameItem(left,right);});
  }
  void completeDownload() {
    auto& op=*m_operation;
    if(op.download==Download::CompareFenceBackup) {
      if(!m_fence_backup||!sameFence(m_fence_backup->items,op.items)) {
        snapshot(op,false);
        finish("FAILED","fence_changed_since_backup");
        return;
      }
      op.items.clear();op.expected=unsigned(op.upload.size());
      advance(Stage::UploadCount);
      report(op,"BASELINE_MATCHED","saved_fence_matches_current_readback");
      return;
    }
    if(op.action=="save") {
      if(!editableFence(op.items)) {
        snapshot(op,true);
        finish("FAILED","stored_fence_outside_edit_profile");
        return;
      }
      m_fence_backup=FenceBackup{op.id,op.items};
    }
    const bool verified=op.download!=Download::VerifyUpload||verifiedUpload();
    op.readback_confirmed=op.download!=Download::Observation&&verified;
    if(op.download==Download::VerifyUpload && op.readback_confirmed && !op.plan) {
      if(m_px4_rover) {
        auto* binding=dynamic_cast<Px4RoverMissionBinding*>(&m_services);
        if(!binding || !binding->recordPx4MissionProof(op.id,op.items,op.opaque_id)) {
          snapshot(op,true);
          finish("FAILED","rover_plan_binding_unavailable");
          return;
        }
      } else if(!m_services.recordStoredMissionProof(op.id,op.items)&&(m_quadplane||m_plane||arduRoverExecution())) {
        snapshot(op,true);
        finish("FAILED",m_quadplane?"quadplane_plan_binding_unavailable":
          m_plane?"plane_plan_binding_unavailable":"rover_plan_binding_unavailable");
        return;
      }
    } else if(op.download==Download::Observation&&(!op.plan||op.plan->value==MAV_MISSION_TYPE_MISSION)) {
      const auto proof=m_services.storedMissionProof();
      if(proof.valid) {
        bool same=proof.items.size()==op.items.size();
        // HOME is established by the FC and current flags change during flight.
        // Compare the stored executable rows rather than inventing an atomic
        // HOME/cursor/content snapshot from a sequential download.
        for(unsigned i=m_px4_rover?0:1;same&&i<op.items.size();++i)
          same=m_px4_rover?samePx4Waypoint(proof.items[i],op.items[i]):sameItem(proof.items[i],op.items[i],true);
        if(!same)m_services.invalidateStoredMissionProof("observed_stored_mission_changed");
      }
    }
    snapshot(op,verified);
    if(!verified){finish("FAILED","upload_readback_item_mismatch");return;}
    finish("COMPLETED",op.action=="save"?"fence_backup_saved":
      op.download==Download::VerifyUpload?(op.plan?"stored_fence_exact_match":"semantic_readback_matches_profile"):
      op.download==Download::VerifyClear?"clear_readback_empty":
      op.inspection?"complete_requested_plan_observation":"complete_requested_mission_observation");
  }
  void snapshot(const Operation& op,bool verified) {
    std::ostringstream out;out<<std::setprecision(9);
    const auto verification=op.download==Download::Observation?"observation":
      !verified?"different":op.plan?"exact_fence_match":"semantic_profile_match";
    out<<"{\"schema\":1,\"id\":"<<std::quoted(op.id)<<",\"status\":\"complete\",\"mission_type\":"
       <<unsigned(op.plan?op.plan->value:0)<<",\"count\":"<<op.items.size()
       <<",\"opaque_id\":"<<op.opaque_id<<",\"collected_at\":"<<m_services.now()<<",\"atomic\":false,\"source_age\":\"unknown\""
       <<",\"verification\":\""<<verification<<"\"";
    if(op.plan) {
      out<<",\"type\":\""<<op.plan->name<<"\",\"session_epoch\":"<<m_session_epoch;
      if(!op.backup_id.empty())out<<",\"backup_id\":"<<std::quoted(op.backup_id);
    } else
      out<<",\"home_reference_compared\":false,\"current_flag_compared\":false,\"altitude_tolerance_m\":0.010001"
         <<",\"supported_storage_normalizations\":{\"rtl_nonpositional_frame\":[2,0],\"land_default_param4\":[0,1]}";
    out<<",\"items\":[";
    for(unsigned i=0;i<op.items.size();++i) {
      const auto& item=op.items[i];if(i)out<<',';
      out<<"{\"seq\":"<<item.seq<<",\"command\":"<<item.command<<",\"frame\":"<<unsigned(item.frame)
         <<",\"current\":"<<unsigned(item.current)<<",\"autocontinue\":"<<unsigned(item.autocontinue)<<",\"params\":[";
      for(unsigned j=0;j<4;++j){if(j)out<<',';out<<item.params[j];}
      out<<"],\"x\":"<<item.x<<",\"y\":"<<item.y<<",\"z\":"<<item.z<<'}';
    }
    out<<"]}";m_services.publish(op.plan?op.plan->snapshot:"MAVMOOS_MISSION_SNAPSHOT",out.str());
  }
  void report(const Operation& op,const std::string& status,const std::string& reason) {
    std::ostringstream out;out<<"schema=1,id="<<op.id<<",action="<<op.action<<",status="<<status<<",reason="<<reason
      <<",received="<<op.items.size()<<",expected="<<op.expected<<",seq="<<op.seq<<",retries="<<op.retries
      <<",phases="<<op.phases<<",legacy_requests="<<op.legacy_requests
      <<",mutation_may_have_applied="<<(op.mutation_may_have_applied?"true":"false")
      <<",remote_accepted="<<(op.remote_accepted?"true":"false")<<",readback_confirmed="<<(op.readback_confirmed?"true":"false")
      <<",expires="<<stamp(op.expires)<<",at="<<stamp(m_services.now());
    if(op.inspection)
      out<<",type="<<(op.plan?op.plan->name:"invalid")<<",mission_type="<<(op.plan?std::to_string(op.plan->value):"unknown")
         <<",session_epoch="<<m_session_epoch;
    if(!op.backup_id.empty())
      out<<",backup_id="<<op.backup_id<<",restore_required="<<(m_fence_backup&&m_fence_backup->restore_required?"true":"false");
    m_services.publish(op.inspection?"MAVMOOS_MISSION_INSPECTION_RESULT":"MAVMOOS_MISSION_RESULT",out.str());
  }
  void finish(const std::string& status,const std::string& reason) {
    if(!m_operation)return;
    auto& op=*m_operation;
    if(op.phase&&mutation(op.phase->request.action)&&!op.phase->accepted)op.mutation_may_have_applied=true;
    if(m_fence_backup&&op.backup_id==m_fence_backup->id) {
      if(op.mutation_may_have_applied)m_fence_backup->restore_required=true;
      if(op.action=="restore"&&op.readback_confirmed)m_fence_backup->restore_required=false;
    }
    report(op,status,reason);const auto id=op.id;m_operation.reset();m_services.missionFinished(id);
  }
  Services& m_services;
  std::optional<Operation> m_operation;
  std::optional<FenceBackup> m_fence_backup;
  std::set<std::string> m_seen;
  bool m_fence_edit=false,m_px4_rover=false,m_quadplane=false,m_plane=false,m_ardu_rover=false,m_native_mission=false;
  double m_boundary=0,m_last_now=0,m_progress_at=0,m_timeout=60,m_response_timeout=1.5;
  uint64_t m_phase_nonce=0,m_session_epoch=0;
};
}
}
extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi,mavmoos::runtime::Services* services)
{return abi==mavmoos::runtime::plugin_abi&&services?new mavmoos::runtime::Missions(*services):nullptr;}
extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin){delete plugin;}
