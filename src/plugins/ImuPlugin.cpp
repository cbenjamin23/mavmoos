/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ImuPlugin.cpp                                   */
/*    DATE: November 4, 2026                                */
/************************************************************/
#include "Plugin.h"
#include "ImuObservation.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace mavmoos::runtime {
namespace {
constexpr std::size_t sensor_capacity=8;
std::string boolean(bool value){return value?"true":"false";}
std::string decimal(double value){std::ostringstream out;out<<std::setprecision(17)<<value;return out.str();}
std::string jsonString(const std::string& value)
{
  static constexpr char hex[]="0123456789abcdef";std::string out="\"";
  for(unsigned char c:value){if(c=='"'||c=='\\'){out+='\\';out+=char(c);}else if(c<32||c>=127){out+="\\u00";out+=hex[c>>4];out+=hex[c&15];}else out+=char(c);}
  return out+'"';
}
double setting(const std::string& value,double low,double high)
{
  if(value.empty()||value.find_first_of(" \t\r\n")!=std::string::npos)throw std::invalid_argument("invalid imu setting");
  std::size_t used=0;const double result=std::stod(value,&used);
  if(used!=value.size()||!std::isfinite(result)||result<low||result>high)throw std::invalid_argument("imu setting outside bounds");
  return result;
}
struct Sample{MessageTime time;uint64_t observation=0;imu::Observation wire;bool remote_regressed=false;};
struct VibrationSample {
  MessageTime time;
  uint64_t observation=0;
  imu::VibrationReport wire;
  bool remote_regressed=false;
};
class Imu final:public Plugin {
public:
  explicit Imu(Services& services):m_services(services),m_boundary(services.now()),m_last_now(m_boundary)
  {if(!std::isfinite(m_boundary)||m_boundary<0){m_boundary=0;m_last_now=0;m_clock_fault=true;}}
  std::string name()const override{return "imu";}
  PluginDescriptor descriptor()const override
  {
    PluginDescriptor result;result.profile="ardupilot_raw_scaled_imu_observations";
    result.capabilities={{"imu.acceleration","","","observation"},{"imu.angular_velocity","","","observation"},{"imu.magnetic_field","","","observation"},{"imu.temperature","","","observation"},
                         {"imu.vibration","","","observation"}};
    result.exclusive_resources={"observation.imu"};
    result.observation_streams={{MAVLINK_MSG_ID_RAW_IMU,200000},{MAVLINK_MSG_ID_SCALED_IMU,200000},{MAVLINK_MSG_ID_SCALED_IMU2,200000},{MAVLINK_MSG_ID_SCALED_IMU3,200000}};
    result.streamed_parameter_bench_compatible=true;return result;
  }
  void configure(const std::string& key,const std::string& value)override
  {if(key=="imu_timeout")m_timeout=setting(value,.25,10);else if(key=="imu_publish_interval")m_interval=setting(value,.02,2);}
  void onReset(const std::string& reason)override
  {
    const double now=m_services.now();m_clock_fault=!std::isfinite(now)||now<0||now<m_last_now;
    if(!m_clock_fault){m_boundary=std::max(m_boundary,now);m_last_now=now;}
    clear(reason);
  }
  void onInputBoundary()override{boundary();}
  void onMessageWithMetadata(const mavlink_message_t& message,const MessageTime& time)override
  {
    const bool heartbeat=message.msgid==MAVLINK_MSG_ID_HEARTBEAT;
    const bool vibration=message.msgid==MAVLINK_MSG_ID_VIBRATION;
    if(!heartbeat&&!vibration&&message.msgid!=MAVLINK_MSG_ID_RAW_IMU&&message.msgid!=MAVLINK_MSG_ID_SCALED_IMU&&message.msgid!=MAVLINK_MSG_ID_SCALED_IMU2&&message.msgid!=MAVLINK_MSG_ID_SCALED_IMU3)return;
    if(!boundary()||!validTime(time)){++m_metadata_rejected;return;}
    const bool heartbeat_shape=(message.magic==MAVLINK_STX&&message.len>0&&message.len<=MAVLINK_MSG_ID_HEARTBEAT_LEN&&!(message.incompat_flags&~MAVLINK_IFLAG_SIGNED))||
      (message.magic==MAVLINK_STX_MAVLINK1&&message.len==MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN);
    if(heartbeat?!heartbeat_shape:!imu::shape(message)){++m_wire_rejected;return;}
    const double now=m_services.now();
    if(heartbeat){
      mavlink_heartbeat_t p{};mavlink_msg_heartbeat_decode(&message,&p);const auto& vehicle=m_services.vehicle();
      if(!heartbeatFresh(now)||now-time.effective_at>2||p.autopilot!=MAV_AUTOPILOT_ARDUPILOTMEGA||p.type!=vehicle.type||p.autopilot!=vehicle.autopilot||message.sysid!=1||message.compid!=1){++m_source_rejected;return;}
      if(m_bound&&(time.effective_at<m_heartbeat_at||time.read_at<m_heartbeat_read)){++m_order_rejected;return;}
      m_bound=true;m_heartbeat_at=time.effective_at;m_heartbeat_read=time.read_at;return;
    }
    if(!m_bound||!heartbeatFresh(now)||now-m_heartbeat_at>2||message.sysid!=1||message.compid!=1){++m_source_rejected;return;}
    if(vibration){
      observeVibration(message,time);
      return;
    }
    const auto decoded=imu::decode(message);if(!decoded){++m_wire_rejected;return;}
    const auto key=std::make_pair(decoded->message_id,decoded->sensor_instance);auto found=m_sensors.find(key);
    if(found==m_sensors.end()){
      if(m_sensors.size()==sensor_capacity){++m_capacity_rejected;return;}
      found=m_sensors.emplace(key,Sample{}).first;
    }else if(time.effective_at<found->second.time.effective_at||time.read_at<found->second.time.read_at){++m_order_rejected;return;}
    const bool regressed=found->second.observation&&decoded->remote_stamp<found->second.wire.remote_stamp;
    found->second={time,++m_observation,*decoded,regressed};
  }
  void iterate()override
  {
    boundary();const double now=m_services.now();
    if(!std::isfinite(now)||(m_next_publish>=0&&now<m_next_publish))return;
    m_next_publish=now+m_interval;const auto values=sensors();
    m_services.publish("MAVMOOS_IMU_SENSORS",values);
    m_services.publish("MAVMOOS_IMU_STATUS","{\"schema\":1,\"session_epoch\":"+std::to_string(m_session)+",\"session_boundary\":"+decimal(m_boundary)+
      ",\"state\":"+jsonString(m_clock_fault?"CLOCK_FAULT":m_bound?"BOUND":"WAITING_HEARTBEAT")+",\"read_only\":true,\"authoritative_for_control\":false,\"atomic\":false,\"family_selection\":\"independent_no_precedence_or_fusion\",\"system_id\":"+(m_bound?"1":"null")+
      ",\"component_id\":"+(m_bound?"1":"null")+",\"reset_reason\":"+jsonString(m_reset_reason)+",\"metadata_rejected\":"+std::to_string(m_metadata_rejected)+
      ",\"source_rejected\":"+std::to_string(m_source_rejected)+",\"wire_shape_rejected\":"+std::to_string(m_wire_rejected)+",\"out_of_order_rejected\":"+std::to_string(m_order_rejected)+
      ",\"sensor_capacity_rejected\":"+std::to_string(m_capacity_rejected)+",\"sensors\":"+values+"}");
    m_services.publish("MAVMOOS_IMU_VIBRATION",vibrationRecord(now));
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
  {m_sensors.clear();m_vibration.reset();m_bound=false;m_heartbeat_at=m_heartbeat_read=-1;m_reset_reason=reason;m_next_publish=-1;++m_session;}
  void observeVibration(const mavlink_message_t& message,const MessageTime& time)
  {
    if(time.effective_at<m_heartbeat_at||time.read_at<m_heartbeat_read||
       (m_vibration&&(time.effective_at<m_vibration->time.effective_at||time.read_at<m_vibration->time.read_at))){
      ++m_order_rejected;
      return;
    }
    const auto report=imu::decodeVibration(message);
    if(!report){
      ++m_wire_rejected;
      return;
    }
    const bool regressed=m_vibration&&report->time_usec<m_vibration->wire.time_usec;
    m_vibration=VibrationSample{time,++m_vibration_observation,*report,regressed};
  }
  std::string vibrationRecord(double now)const
  {
    const bool present=m_vibration.has_value();
    const double age=present?now-m_vibration->time.effective_at:0;
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out<<std::boolalpha<<std::setprecision(17)
       <<"{\"schema\":1,\"session_epoch\":"<<m_session<<",\"present\":"<<present
       <<",\"fresh\":"<<(present&&age<=m_timeout)
       <<",\"read_only\":true,\"authoritative_for_control\":false"
       <<",\"source_age\":\"unknown\",\"source_quality\":\"unassessed\""
       <<",\"system_id\":"<<(m_bound?"1":"null")<<",\"component_id\":"<<(m_bound?"1":"null")
       <<",\"autopilot\":"<<(m_bound?std::to_string(MAV_AUTOPILOT_ARDUPILOTMEGA):"null");
    if(present){
      const auto& sample=*m_vibration;
      out<<",\"observation_id\":"<<sample.observation<<",\"effective_at\":"<<sample.time.effective_at
         <<",\"read_at\":"<<sample.time.read_at<<",\"age_s\":"<<age
         <<",\"age_basis\":"<<jsonString(sample.time.age_basis)
         <<",\"age_is_upper_bound\":"<<(sample.time.age_basis=="QUEUE_EMPTY_FLOOR")
         <<",\"data\":"<<imu::data(sample.wire,sample.remote_regressed);
    }else{
      out<<",\"observation_id\":0,\"effective_at\":null,\"read_at\":null,\"age_s\":null"
         <<",\"age_basis\":\"UNKNOWN\",\"age_is_upper_bound\":false,\"data\":null";
    }
    out<<'}';
    return out.str();
  }
  std::string record(const Sample& p)const
  {
    const double now=m_services.now();const bool clock=std::isfinite(now)&&now>=p.time.effective_at;const double age=clock?now-p.time.effective_at:0;
    return "{\"schema\":1,\"session_epoch\":"+std::to_string(m_session)+",\"present\":true,\"encoding_valid\":true,\"fresh\":"+boolean(!m_clock_fault&&m_bound&&heartbeatFresh(now)&&clock&&age<=m_timeout)+
      ",\"effective_at\":"+decimal(p.time.effective_at)+",\"read_at\":"+decimal(p.time.read_at)+",\"age_s\":"+(clock?decimal(age):"null")+
      ",\"age_basis\":"+jsonString(p.time.age_basis)+",\"age_is_upper_bound\":"+boolean(p.time.age_basis=="QUEUE_EMPTY_FLOOR")+
      ",\"source_age\":\"unknown\",\"source_quality\":\"unassessed\",\"observation_id\":"+std::to_string(p.observation)+",\"data\":"+imu::data(p.wire,p.remote_regressed)+"}";
  }
  std::string sensors()const
  {
    std::string out="{\"schema\":1,\"session_epoch\":"+std::to_string(m_session)+",\"present\":"+boolean(!m_sensors.empty())+",\"atomic\":false,\"cache_capacity\":"+std::to_string(sensor_capacity)+",\"sensors\":[";
    bool first=true;for(const auto& item:m_sensors){if(!first)out+=',';first=false;out+=record(item.second);}return out+"]}";
  }
  Services& m_services;std::map<std::pair<uint32_t,unsigned>,Sample> m_sensors;
  std::optional<VibrationSample> m_vibration;
  bool m_bound=false,m_clock_fault=false;
  double m_boundary=0,m_last_now=0,m_heartbeat_at=-1,m_heartbeat_read=-1,m_timeout=1,m_interval=.1,m_next_publish=-1;
  uint64_t m_session=1,m_observation=0,m_metadata_rejected=0,m_source_rejected=0,m_wire_rejected=0,m_order_rejected=0,m_capacity_rejected=0;
  uint64_t m_vibration_observation=0;
  std::string m_reset_reason="startup";
};
}
}
extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi,mavmoos::runtime::Services* services)
{return abi==mavmoos::runtime::plugin_abi&&services?new mavmoos::runtime::Imu(*services):nullptr;}
extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin){delete plugin;}
