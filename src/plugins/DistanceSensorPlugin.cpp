/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: DistanceSensorPlugin.cpp                        */
/*    DATE: November 4, 2026                                */
/************************************************************/
#include "Plugin.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace mavmoos::runtime {
namespace {
constexpr std::size_t sensor_capacity=8;
constexpr double pi=3.14159265358979323846, norm_tolerance=.001;
std::string boolean(bool value){return value?"true":"false";}
std::string decimal(double value){std::ostringstream out;out.imbue(std::locale::classic());out<<std::setprecision(17)<<value;return out.str();}
std::string jsonString(const std::string& value)
{
  static constexpr char hex[]="0123456789abcdef";std::string out="\"";
  for(unsigned char c:value){if(c=='"'||c=='\\'){out+='\\';out+=char(c);}else if(c<32||c>=127){out+="\\u00";out+=hex[c>>4];out+=hex[c&15];}else out+=char(c);}
  return out+'"';
}
std::string quantity(double value,bool known,const char* unit,const char* reason)
{
  known=known&&std::isfinite(value);
  return "{\"known\":"+boolean(known)+",\"value\":"+(known?decimal(value):"null")+",\"unit\":"+jsonString(unit)+",\"reason\":"+jsonString(reason)+"}";
}
double setting(const std::string& value,double low,double high)
{
  if(value.empty()||value.find_first_of(" \t\r\n")!=std::string::npos)throw std::invalid_argument("invalid distance_sensor setting");
  std::size_t used=0;const auto number=std::stod(value,&used);
  if(used!=value.size()||!std::isfinite(number)||number<low||number>high)throw std::invalid_argument("distance_sensor setting outside bounds");
  return number;
}
const char* typeName(unsigned value)
{
  constexpr std::array<const char*,5> names={"LASER","ULTRASOUND","INFRARED","RADAR","UNKNOWN"};
  return value<names.size()?names[value]:"UNRECOGNIZED";
}
const char* orientationName(unsigned value)
{
  constexpr std::array<const char*,41> names={"NONE","YAW_45","YAW_90","YAW_135","YAW_180","YAW_225","YAW_270","YAW_315",
    "ROLL_180","ROLL_180_YAW_45","ROLL_180_YAW_90","ROLL_180_YAW_135","PITCH_180","ROLL_180_YAW_225","ROLL_180_YAW_270","ROLL_180_YAW_315",
    "ROLL_90","ROLL_90_YAW_45","ROLL_90_YAW_90","ROLL_90_YAW_135","ROLL_270","ROLL_270_YAW_45","ROLL_270_YAW_90","ROLL_270_YAW_135",
    "PITCH_90","PITCH_270","PITCH_180_YAW_90","PITCH_180_YAW_270","ROLL_90_PITCH_90","ROLL_180_PITCH_90","ROLL_270_PITCH_90",
    "ROLL_90_PITCH_180","ROLL_270_PITCH_180","ROLL_90_PITCH_270","ROLL_180_PITCH_270","ROLL_270_PITCH_270","ROLL_90_PITCH_180_YAW_90",
    "ROLL_90_YAW_270","ROLL_90_PITCH_68_YAW_293","PITCH_315","ROLL_90_PITCH_315"};
  return value<names.size()?names[value]:value==MAV_SENSOR_ROTATION_CUSTOM?"CUSTOM":"UNRECOGNIZED";
}
std::string fov(float value,bool v2)
{
  const bool known=v2&&std::isfinite(value)&&value>0&&value<=2*pi;
  const char* reason=!v2?"mavlink1_extension_absent":!std::isfinite(value)?"nonfinite":value==0?"unknown_unset_or_omitted":!known?"outside_geometric_range":"reported_fov";
  return "{\"raw_rad\":"+(v2&&std::isfinite(value)?decimal(value):"null")+",\"quantity\":"+quantity(value,known,"rad",reason)+"}";
}
std::string quaternion(const std::array<double,4>& values,bool v2,bool custom)
{
  bool finite=true;double squared=0;std::string raw="[";
  for(unsigned i=0;i<values.size();++i){if(i)raw+=',';const bool valid=std::isfinite(values[i]);finite=finite&&valid;squared+=values[i]*values[i];raw+=v2&&valid?decimal(values[i]):"null";}
  raw+=']';const double norm=std::sqrt(squared);const bool unit=v2&&finite&&std::isfinite(norm)&&std::abs(norm-1)<=norm_tolerance;
  return "{\"raw_wxyz\":"+raw+",\"extension_available\":"+boolean(v2)+",\"finite\":"+boolean(v2&&finite)+",\"norm\":"+(v2&&finite&&std::isfinite(norm)?decimal(norm):"null")+
    ",\"unit_norm\":"+boolean(unit)+",\"required_for_custom_orientation\":"+boolean(custom)+",\"usable_for_custom_orientation\":"+boolean(custom&&unit)+
    ",\"reason\":"+jsonString(!v2?"mavlink1_extension_absent":!finite?"nonfinite":!unit?"zero_omitted_or_nonunit":custom?"reported_custom_orientation":"not_applied_for_standard_orientation")+"}";
}
constexpr unsigned obstacle_unknown = UINT16_MAX;

std::string obstacleData(const mavlink_message_t& message)
{
  mavlink_obstacle_distance_t packet{};
  mavlink_msg_obstacle_distance_decode(&message, &packet);
  const unsigned minimum = packet.min_distance, maximum = packet.max_distance;
  const unsigned increment_integer = packet.increment, frame = packet.frame, sensor_type = packet.sensor_type;
  const double increment_float = packet.increment_f, offset = packet.angle_offset;
  const double increment = increment_float != 0 ? increment_float : increment_integer;
  if (minimum > maximum || !std::isfinite(increment) || increment == 0 ||
      std::abs(increment) > 360 || !std::isfinite(offset))
    return {};

  const unsigned no_hit = maximum + 1;
  const bool explicit_no_hit = no_hit < obstacle_unknown;
  const char* frame_name = frame == MAV_FRAME_GLOBAL ? "GLOBAL" :
                           frame == MAV_FRAME_BODY_FRD ? "BODY_FRD" : "UNINTERPRETED";
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(9) << "{\"time_usec_raw\":\"" << uint64_t(packet.time_usec)
      << "\",\"remote_clock\":\"unix_or_boot_unmapped_uint64\",\"frame_raw\":" << frame
      << ",\"frame_name\":\"" << frame_name << "\",\"frame_explicit\":"
      << boolean(message.len == MAVLINK_MSG_ID_OBSTACLE_DISTANCE_LEN)
      << ",\"increment_deg_raw\":" << increment_integer << ",\"increment_f_deg_raw\":" << increment_float
      << ",\"increment_deg\":" << increment << ",\"increment_source\":\""
      << (increment_float != 0 ? "increment_f" : "increment") << "\""
      << ",\"angle_offset_deg\":" << offset << ",\"payload_length\":" << unsigned(message.len)
      << ",\"sensor_type_raw\":" << sensor_type << ",\"sensor_type_name\":\"" << typeName(sensor_type)
      << "\",\"min_distance_cm\":" << minimum << ",\"max_distance_cm\":" << maximum
      << ",\"no_hit_sentinel_cm\":" << (explicit_no_hit ? std::to_string(no_hit) : "null")
      << ",\"sectors\":[";
  for (std::size_t i = 0; i < MAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_DISTANCES_LEN; ++i) {
    const unsigned cm = packet.distances[i];
    const bool in_range = cm != obstacle_unknown && cm >= minimum && cm <= maximum;
    const bool no_obstacle = explicit_no_hit && cm == no_hit;
    const char* kind = cm == obstacle_unknown ? "unknown" : cm == 0 ? "contact" :
                       no_obstacle ? "no_hit" : in_range ? "in_range" : "out_of_range";
    if (i) out << ',';
    out << "{\"distance_cm_raw\":" << cm << ",\"classification\":\"" << kind << "\",\"distance_m\":";
    if (cm == 0 || in_range) out << cm / 100.;
    else out << "null";
    out << '}';
  }
  out << "]}";
  return out.str();
}

struct Sample{MessageTime time;uint64_t observation=0;std::string data;};
class DistanceSensor final:public Plugin {
public:
  explicit DistanceSensor(Services& services):m_services(services),m_boundary(services.now()),m_last_now(m_boundary)
  {if(!std::isfinite(m_boundary)||m_boundary<0){m_boundary=0;m_last_now=0;m_clock_fault=true;}}
  std::string name()const override{return "distance_sensor";}
  PluginDescriptor descriptor()const override
  {
    PluginDescriptor result;result.profile="ardupilot_distance_observations";
    result.capabilities={{"distance.observation","","","observation"},
                         {"distance.obstacle_sectors","","","observation"}};
    result.exclusive_resources={"observation.distance_sensor"};
    result.observation_streams={{MAVLINK_MSG_ID_DISTANCE_SENSOR,100000}};
    result.streamed_parameter_bench_compatible=true;return result;
  }
  void configure(const std::string& key,const std::string& value)override
  {if(key=="distance_sensor_timeout")m_timeout=setting(value,.25,10);else if(key=="distance_sensor_publish_interval")m_interval=setting(value,.02,2);}
  void onReset(const std::string& reason)override
  {
    const double now=m_services.now();m_clock_fault=!std::isfinite(now)||now<0||now<m_last_now;
    if(!m_clock_fault){m_boundary=std::max(m_boundary,now);m_last_now=now;}
    clear(reason);
  }
  void onInputBoundary()override{boundary();}
  void onMessageWithMetadata(const mavlink_message_t& message,const MessageTime& time)override
  {
    unsigned minimum=0,maximum=0;
    if(message.msgid==MAVLINK_MSG_ID_HEARTBEAT){minimum=MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN;maximum=MAVLINK_MSG_ID_HEARTBEAT_LEN;}
    else if(message.msgid==MAVLINK_MSG_ID_DISTANCE_SENSOR){minimum=MAVLINK_MSG_ID_DISTANCE_SENSOR_MIN_LEN;maximum=MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN;}
    else if(message.msgid==MAVLINK_MSG_ID_OBSTACLE_DISTANCE){minimum=MAVLINK_MSG_ID_OBSTACLE_DISTANCE_MIN_LEN;maximum=MAVLINK_MSG_ID_OBSTACLE_DISTANCE_LEN;}
    else return;
    if(!boundary()||!validTime(time)){++m_metadata_rejected;return;}
    if((message.magic==MAVLINK_STX&&(message.len==0||message.len>maximum||(message.incompat_flags&~MAVLINK_IFLAG_SIGNED)))||
       (message.magic==MAVLINK_STX_MAVLINK1&&(message.msgid>255||message.len!=minimum))||(message.magic!=MAVLINK_STX&&message.magic!=MAVLINK_STX_MAVLINK1))
      {++m_wire_rejected;return;}
    const double now=m_services.now();
    if(message.msgid==MAVLINK_MSG_ID_HEARTBEAT){
      mavlink_heartbeat_t p{};mavlink_msg_heartbeat_decode(&message,&p);const auto& v=m_services.vehicle();
      if(!heartbeatFresh(now)||now-time.effective_at>2||p.autopilot!=MAV_AUTOPILOT_ARDUPILOTMEGA||p.type!=v.type||p.autopilot!=v.autopilot||message.sysid==0||message.compid==0||
         (m_bound&&(message.sysid!=m_system||message.compid!=m_component))){++m_source_rejected;return;}
      if(m_bound&&(time.effective_at<m_heartbeat_at||time.read_at<m_heartbeat_read)){++m_order_rejected;return;}
      m_bound=true;m_system=message.sysid;m_component=message.compid;m_heartbeat_at=time.effective_at;m_heartbeat_read=time.read_at;return;
    }
    if(!m_bound||!heartbeatFresh(now)||now-m_heartbeat_at>2||message.sysid!=m_system||message.compid!=m_component){++m_source_rejected;return;}
    if(message.msgid==MAVLINK_MSG_ID_OBSTACLE_DISTANCE)observeObstacle(message,time);
    else observe(message,time);
  }
  void iterate()override
  {
    boundary();const double now=m_services.now();
    if(!std::isfinite(now)||(m_next_publish>=0&&now<m_next_publish))return;
    m_next_publish=now+m_interval;const auto values=sensors();
    m_services.publish("MAVMOOS_DISTANCE_SENSORS",values);
    m_services.publish("MAVMOOS_DISTANCE_STATUS","{\"schema\":1,\"session_epoch\":"+std::to_string(m_session)+",\"session_boundary\":"+decimal(m_boundary)+
      ",\"state\":"+jsonString(m_clock_fault?"CLOCK_FAULT":m_bound?"BOUND":"WAITING_HEARTBEAT")+",\"read_only\":true,\"authoritative_for_control\":false,\"atomic\":false,\"system_id\":"+(m_bound?std::to_string(m_system):"null")+
      ",\"component_id\":"+(m_bound?std::to_string(m_component):"null")+",\"reset_reason\":"+jsonString(m_reset_reason)+",\"metadata_rejected\":"+std::to_string(m_metadata_rejected)+
      ",\"source_rejected\":"+std::to_string(m_source_rejected)+",\"wire_shape_rejected\":"+std::to_string(m_wire_rejected)+",\"out_of_order_rejected\":"+std::to_string(m_order_rejected)+
      ",\"sensor_capacity_rejected\":"+std::to_string(m_capacity_rejected)+",\"sensors\":"+values+"}");
    m_services.publish("MAVMOOS_OBSTACLE_DISTANCE",obstacleObservation());
  }
private:
  bool heartbeatFresh(double now)const
  {const auto& v=m_services.vehicle();return v.heartbeat_valid&&v.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA&&std::isfinite(v.heartbeat_at)&&v.heartbeat_at>=0&&now>=v.heartbeat_at&&now-v.heartbeat_at<=2;}
  bool validTime(const MessageTime& time)const
  {return !m_clock_fault&&std::isfinite(time.effective_at)&&std::isfinite(time.read_at)&&time.effective_at>m_boundary&&time.read_at>=time.effective_at&&time.read_at<=m_services.now()&&
    (time.age_basis=="QUEUE_EMPTY_FLOOR"||(time.age_basis=="READ_ACQUISITION"&&time.effective_at==time.read_at));}
  bool boundary()
  {
    const double now=m_services.now();
    if(m_clock_fault||!std::isfinite(now)||now<0||now<m_last_now){if(!m_clock_fault)clear("clock_fault");m_clock_fault=true;return false;}
    m_last_now=now;
    if(m_bound&&(!heartbeatFresh(now)||now-m_heartbeat_at>2)){m_boundary=std::max(m_boundary,now);clear("heartbeat_expired");}
    return true;
  }
  void clear(const std::string& reason)
  {m_sensors.clear();m_obstacle={};m_bound=false;m_heartbeat_at=m_heartbeat_read=-1;m_reset_reason=reason;m_next_publish=-1;++m_session;}
  std::string record(const Sample& sample,const char* family="DISTANCE_SENSOR")const
  {
    const bool present=sample.observation!=0;
    const double now=m_services.now();const bool clock=present&&std::isfinite(now)&&now>=sample.time.effective_at;
    const double age=clock?now-sample.time.effective_at:0;
    const bool fresh=!m_clock_fault&&m_bound&&heartbeatFresh(now)&&clock&&age<=m_timeout;
    return "{\"schema\":1,\"session_epoch\":"+std::to_string(m_session)+",\"family\":"+jsonString(family)+",\"present\":"+boolean(present)+
      ",\"encoding_valid\":"+boolean(present)+",\"fresh\":"+boolean(fresh)+
      ",\"effective_at\":"+(present?decimal(sample.time.effective_at):"null")+",\"read_at\":"+(present?decimal(sample.time.read_at):"null")+",\"age_s\":"+(clock?decimal(age):"null")+
      ",\"age_basis\":"+jsonString(present?sample.time.age_basis:"UNKNOWN")+",\"age_is_upper_bound\":"+boolean(present&&sample.time.age_basis=="QUEUE_EMPTY_FLOOR")+
      ",\"source_age\":\"unknown\",\"source_quality\":\"unassessed\",\"observation_id\":"+std::to_string(sample.observation)+",\"data\":"+(present?sample.data:"null")+"}";
  }
  std::string obstacleObservation()const
  {
    auto result=record(m_obstacle,"OBSTACLE_DISTANCE");
    result.pop_back();
    return result+",\"read_only\":true,\"authoritative_for_control\":false,\"message_id\":330,\"system_id\":"+
      (m_bound?std::to_string(m_system):"null")+",\"component_id\":"+(m_bound?std::to_string(m_component):"null")+
      ",\"autopilot\":"+(m_bound?std::to_string(m_services.vehicle().autopilot):"null")+
      ",\"sample_timeout_s\":"+decimal(m_timeout)+",\"publish_interval_s\":"+decimal(m_interval)+
      ",\"clock_fault\":"+boolean(m_clock_fault)+",\"invalid_samples\":"+std::to_string(m_obstacle_rejected)+
      ",\"metadata_rejected\":"+std::to_string(m_metadata_rejected)+
      ",\"wire_shape_rejected\":"+std::to_string(m_wire_rejected)+",\"source_rejected\":"+std::to_string(m_source_rejected)+
      ",\"out_of_order_rejected\":"+std::to_string(m_order_rejected)+"}";
  }
  void observeObstacle(const mavlink_message_t& message,const MessageTime& time)
  {
    if (time.effective_at<m_heartbeat_at || time.read_at<m_heartbeat_read ||
        (m_obstacle.observation&&(time.effective_at<m_obstacle.time.effective_at||time.read_at<m_obstacle.time.read_at))) {
      ++m_order_rejected;
      return;
    }
    auto data=obstacleData(message);
    if (data.empty()) { ++m_obstacle_rejected;return; }
    m_obstacle={time,++m_observation,std::move(data)};
  }
  std::string sensors()const
  {
    std::string out="{\"schema\":1,\"session_epoch\":"+std::to_string(m_session)+",\"present\":"+boolean(!m_sensors.empty())+",\"atomic\":false,\"cache_capacity\":"+std::to_string(sensor_capacity)+",\"sensors\":[";
    bool first=true;for(const auto& item:m_sensors){if(!first)out+=',';first=false;out+=record(item.second);}return out+"]}";
  }
  void observe(const mavlink_message_t& message,const MessageTime& time)
  {
    mavlink_distance_sensor_t p{};mavlink_msg_distance_sensor_decode(&message,&p);
    // Packed wire fields are copied to aligned scalar/array values before STL use.
    const unsigned id=p.id,type=p.type,orientation=p.orientation,covariance=p.covariance,quality=p.signal_quality;
    const unsigned minimum=p.min_distance,maximum=p.max_distance,current=p.current_distance,stamp=p.time_boot_ms;
    const std::array<double,4> q={p.quaternion[0],p.quaternion[1],p.quaternion[2],p.quaternion[3]};
    auto found=m_sensors.find(id);
    if(found==m_sensors.end()){if(m_sensors.size()==sensor_capacity){++m_capacity_rejected;return;}found=m_sensors.emplace(id,Sample{}).first;}
    else if(time.effective_at<found->second.time.effective_at||time.read_at<found->second.time.read_at){++m_order_rejected;return;}
    const bool v2=message.magic==MAVLINK_STX,bounds=minimum<=maximum,within=bounds&&current>=minimum&&current<=maximum;
    const bool quality_known=v2&&quality>=2&&quality<=100,covariance_known=covariance!=0&&covariance!=255;
    const char* quality_reason=!v2?"mavlink1_extension_absent":quality==0?"unknown_unset_or_omitted":quality==1?"invalid_signal":quality>100?"outside_percentage_range":"reported_signal_quality";
    auto& sample=found->second;sample.time=time;sample.observation=++m_observation;
    sample.data="{\"id\":"+std::to_string(id)+",\"time_boot_ms\":"+std::to_string(stamp)+",\"remote_clock\":\"boot_ms_unmapped_uint32\",\"timestamp_semantics\":\"ardupilot_message_send_time_not_certified_measurement_time\""+
      ",\"wire_version\":"+std::to_string(v2?2:1)+",\"payload_length\":"+std::to_string(message.len)+",\"extension_bytes_observed\":"+boolean(v2&&message.len>MAVLINK_MSG_ID_DISTANCE_SENSOR_MIN_LEN)+
      ",\"min_distance_cm\":"+std::to_string(minimum)+",\"max_distance_cm\":"+std::to_string(maximum)+",\"current_distance_cm\":"+std::to_string(current)+
      ",\"min_distance_m\":"+quantity(minimum*.01,true,"m","reported_limit")+",\"max_distance_m\":"+quantity(maximum*.01,true,"m","reported_limit")+
      ",\"current_distance_m\":"+quantity(current*.01,true,"m","reported_distance")+",\"bounds_consistent\":"+boolean(bounds)+",\"current_within_reported_limits\":"+boolean(within)+
      ",\"type_raw\":"+std::to_string(type)+",\"type_known\":"+boolean(type<=MAV_DISTANCE_SENSOR_UNKNOWN)+",\"type_name\":"+jsonString(typeName(type))+
      ",\"orientation_raw\":"+std::to_string(orientation)+",\"orientation_known\":"+boolean(orientation<=40||orientation==MAV_SENSOR_ROTATION_CUSTOM)+",\"orientation_name\":"+jsonString(orientationName(orientation))+
      ",\"orientation_frame\":\"BODY_FRD\",\"distance_interpretation\":\"sensor_report_no_vertical_or_terrain_projection\",\"physical_accuracy\":\"unassessed\""+
      ",\"covariance\":{\"raw_cm2\":"+std::to_string(covariance)+",\"quantity\":"+quantity(covariance*.0001,covariance_known,"m^2",covariance==0?"ardupilot_zero_unknown_or_invalid":covariance==255?"wire_unknown":"reported_variance")+"}"+
      ",\"signal_quality\":{\"raw\":"+(v2?std::to_string(quality):"null")+",\"extension_available\":"+boolean(v2)+",\"quantity\":"+quantity(quality,quality_known,"percent",quality_reason)+"}"+
      ",\"horizontal_fov\":"+fov(p.horizontal_fov,v2)+",\"vertical_fov\":"+fov(p.vertical_fov,v2)+",\"quaternion\":"+quaternion(q,v2,orientation==MAV_SENSOR_ROTATION_CUSTOM)+"}";
  }
  Services& m_services;std::map<unsigned,Sample> m_sensors;
  Sample m_obstacle;
  uint64_t m_obstacle_rejected=0;
  bool m_bound=false,m_clock_fault=false;unsigned m_system=0,m_component=0;
  double m_boundary=0,m_last_now=0,m_heartbeat_at=-1,m_heartbeat_read=-1,m_timeout=1,m_interval=.1,m_next_publish=-1;
  uint64_t m_session=1,m_observation=0,m_metadata_rejected=0,m_source_rejected=0,m_wire_rejected=0,m_order_rejected=0,m_capacity_rejected=0;
  std::string m_reset_reason="startup";
};
}
}
extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi,mavmoos::runtime::Services* services)
{return abi==mavmoos::runtime::plugin_abi&&services?new mavmoos::runtime::DistanceSensor(*services):nullptr;}
extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin){delete plugin;}
