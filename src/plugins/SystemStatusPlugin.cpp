/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: SystemStatusPlugin.cpp                          */
/*    DATE: November 4, 2026                                */
/************************************************************/
#include "Plugin.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <iomanip>
#include <initializer_list>
#include <limits>
#include <locale>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace mavmoos::runtime {
namespace {
constexpr std::size_t battery_limit=8, mcu_limit=8, text_limit=4, text_chunks=16, event_limit=16;
std::string boolean(bool value) {return value?"true":"false";}
std::string decimal(double value)
{
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out<<std::setprecision(15)<<value;
  return out.str();
}
std::string hex(const std::string& value)
{
  static constexpr char digits[]="0123456789abcdef";std::string result;
  for(unsigned char byte:value){result+=digits[byte>>4];result+=digits[byte&15];}return result;
}
bool utf8(const std::string& text)
{
  for(std::size_t i=0;i<text.size();) {
    const auto lead=static_cast<unsigned char>(text[i]);
    if(lead<0x80){++i;continue;}
    unsigned count=0;uint32_t code=0,minimum=0;
    if(lead>=0xc2&&lead<=0xdf){count=2;code=lead&31;minimum=0x80;}
    else if(lead>=0xe0&&lead<=0xef){count=3;code=lead&15;minimum=0x800;}
    else if(lead>=0xf0&&lead<=0xf4){count=4;code=lead&7;minimum=0x10000;}
    else return false;
    if(i+count>text.size())return false;
    for(unsigned j=1;j<count;++j){const auto byte=static_cast<unsigned char>(text[i+j]);if((byte&0xc0)!=0x80)return false;code=(code<<6)|(byte&63);}
    if(code<minimum||code>0x10ffff||(code>=0xd800&&code<=0xdfff))return false;
    i+=count;
  }
  return true;
}
std::string jsonString(const std::string& value,bool preserve_utf8=true)
{
  static constexpr char digits[]="0123456789abcdef";std::string out="\"";
  preserve_utf8=preserve_utf8&&utf8(value);
  for(unsigned char c:value) {
    if(c=='"'||c=='\\'){out+='\\';out+=char(c);}
    else if(c<32||(!preserve_utf8&&c>=127)){out+="\\u00";out+=digits[c>>4];out+=digits[c&15];}
    else out+=char(c);
  }
  return out+'"';
}
std::string quantity(double value,bool known,const char* unit,const char* reason="wire_value")
{
  return "{\"known\":"+boolean(known)+",\"value\":"+(known?decimal(value):"null")+",\"unit\":"+jsonString(unit)+",\"reason\":"+jsonString(reason)+"}";
}
std::string flagReport(uint16_t raw,std::initializer_list<std::pair<uint16_t,const char*>> fields)
{
  uint16_t unknown=raw;
  std::string reported="{";
  for(const auto& [bit,name]:fields) {
    if(reported.size()>1) reported+=',';
    reported+=jsonString(name)+':'+boolean(raw&bit);
    unknown&=~bit;
  }
  return "{\"raw\":"+std::to_string(raw)+",\"unknown_mask\":"+std::to_string(unknown)+
    ",\"reported\":"+reported+"}}";
}
std::string estimatorFlags(uint16_t raw,bool ardupilot)
{
  // The position and velocity bits are shared. The fault bits belong to
  // different MAVLink enums even when their integer values overlap.
  return flagReport(raw,{
    {ESTIMATOR_ATTITUDE,"attitude"},{ESTIMATOR_VELOCITY_HORIZ,"horizontal_velocity"},
    {ESTIMATOR_VELOCITY_VERT,"vertical_velocity"},{ESTIMATOR_POS_HORIZ_REL,"horizontal_position_relative"},
    {ESTIMATOR_POS_HORIZ_ABS,"horizontal_position_absolute"},{ESTIMATOR_POS_VERT_ABS,"vertical_position_absolute"},
    {ESTIMATOR_POS_VERT_AGL,"vertical_position_agl"},{ESTIMATOR_CONST_POS_MODE,"constant_position_mode"},
    {ESTIMATOR_PRED_POS_HORIZ_REL,"predicted_horizontal_position_relative"},
    {ESTIMATOR_PRED_POS_HORIZ_ABS,"predicted_horizontal_position_absolute"},
    {ardupilot?uint16_t(EKF_UNINITIALIZED):uint16_t(ESTIMATOR_GPS_GLITCH),ardupilot?"uninitialized":"gps_glitch"},
    {ardupilot?uint16_t(EKF_GPS_GLITCHING):uint16_t(ESTIMATOR_ACCEL_ERROR),ardupilot?"gps_glitch":"accel_error"}
  });
}
std::string railVoltage(uint16_t millivolts)
{
  return quantity(millivolts*.001,millivolts!=0,"V",millivolts?"reported_rail":"zero_or_unreported");
}
bool validMetric(float value) {return std::isfinite(value)&&value>=0;}
std::string metric(float value,const char* unit,const char* unavailable=nullptr)
{
  if(unavailable) return quantity(0,false,unit,unavailable);
  if(!std::isfinite(value)) return quantity(0,false,unit,"nonfinite_metric");
  return quantity(value,value>=0,unit,value<0?"negative_metric":"reported_metric");
}
std::string diagnosticFloat(float value,const char* unit,bool zero_unknown=false,bool nonnegative=false)
{
  if(std::isnan(value)) return quantity(0,false,unit,"wire_nan");
  if(!std::isfinite(value)) return quantity(0,false,unit,"invalid_infinity");
  if(zero_unknown && value==0) return quantity(0,false,unit,"zero_unreported");
  if(nonnegative && value<0) return quantity(0,false,unit,"out_of_range");
  return quantity(value,true,unit);
}
template<std::size_t Size>
std::string boundedAscii(const char (&value)[Size])
{
  const auto* end=std::find(value,value+Size,'\0');
  const std::string bytes(value,end);
  const bool terminated=end!=value+Size;
  const bool text=std::all_of(bytes.begin(),bytes.end(),[](unsigned char c){return c>=32&&c<=126;});
  return "{\"raw_hex\":"+jsonString(hex(bytes))+",\"value\":"+
    (bytes.empty()||!text||!terminated?"null":jsonString(bytes))+
    ",\"state\":"+jsonString(bytes.empty()?"unreported":!terminated?"unterminated":text?"text":"non_ascii")+"}";
}
std::string generatorFlags(uint64_t bits)
{
  // The pinned common dialect assigns these 23 contiguous flag bits.
  constexpr const char* names[]={"off","ready","generating","charging","reduced_power","maximum_power",
    "overtemp_warning","overtemp_fault","electronics_overtemp_warning","electronics_overtemp_fault",
    "electronics_fault","power_source_fault","communication_warning","cooling_warning","power_rail_fault",
    "overcurrent_fault","battery_overcharge_current_fault","overvoltage_fault","battery_undervoltage_fault",
    "start_inhibited","maintenance_required","warming_up","idle"};
  static_assert((UINT64_C(1)<<(sizeof(names)/sizeof(names[0])-1))==MAV_GENERATOR_STATUS_FLAG_IDLE);
  std::string flags="{";
  uint64_t remaining=bits;
  for(unsigned i=0;i<sizeof(names)/sizeof(names[0]);++i) {
    const uint64_t flag=UINT64_C(1)<<i;
    if(i) flags+=',';
    flags+=jsonString(names[i])+':'+boolean(bits&flag);
    remaining&=~flag;
  }
  return "{\"raw_decimal\":"+jsonString(std::to_string(bits))+
    ",\"unknown_decimal\":"+jsonString(std::to_string(remaining))+",\"reported\":"+flags+"}}";
}
std::string masks(uint32_t present,uint32_t enabled,uint32_t health)
{
  return "{\"present\":"+std::to_string(present)+",\"enabled\":"+std::to_string(enabled)+",\"health\":"+std::to_string(health)+
    ",\"unhealthy_enabled_present\":"+std::to_string(present&enabled&~health)+",\"enabled_without_present\":"+std::to_string(enabled&~present)+
    ",\"healthy_without_present\":"+std::to_string(health&~present)+"}";
}
double number(const std::string& value,double low,double high)
{
  if(value.empty()||value.find_first_of(" \t\r\n")!=std::string::npos)throw std::invalid_argument("invalid system_status setting");
  std::size_t used=0;const double result=std::stod(value,&used);
  if(used!=value.size()||!std::isfinite(result)||result<low||result>high)throw std::invalid_argument("system_status setting outside bounds");
  return result;
}
struct Sample {
  bool present=false,encoding_valid=true;
  MessageTime time;
  uint64_t observation=0;
  std::string data;
};
struct Assembly {
  uint16_t id=0;uint8_t severity=0,next=1,last_seq=0;
  MessageTime first,last;
  double expires=0;
  std::string bytes,last_bytes;
};
class SystemStatus final:public Plugin {
public:
  explicit SystemStatus(Services& services):m_services(services),m_boundary(services.now()),m_last_now(services.now())
  {if(!std::isfinite(m_boundary)||m_boundary<0){m_boundary=0;m_last_now=0;m_clock_fault=true;}}
  std::string name() const override{return "system_status";}
  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;result.profile="ardupilot_system_observations";
    result.capabilities={{"status.system_sensors","","","observation"},{"status.battery","","","observation"},
      {"status.gps","","","observation"},{"status.gps.secondary","","","observation"},
      {"status.gps.rtk","","","observation"},{"status.text","","","observation"},
      {"status.power","","","observation"},{"status.battery_info","","","observation"},
      {"status.generator","","","observation"},{"status.mcu","","","observation"},
      {"status.ekf","","","observation"},{"status.estimator","","","observation"}};
    result.exclusive_resources={"observation.system_status"};
    result.observation_streams={{MAVLINK_MSG_ID_SYS_STATUS,1000000},{MAVLINK_MSG_ID_BATTERY_STATUS,1000000},{MAVLINK_MSG_ID_GPS_RAW_INT,500000}};
    result.streamed_parameter_bench_compatible=true;return result;
  }
  void configure(const std::string& key,const std::string& value) override
  {
    if(key=="status_timeout")m_timeout=number(value,.25,30);
    else if(key=="status_publish_interval")m_interval=number(value,.05,10);
    else if(key=="status_text_timeout")m_text_timeout=number(value,.25,10);
  }
  void onReset(const std::string& reason) override
  {
    // Reset cannot legitimize observations before the last reliable owner time.
    const auto now=m_services.now();m_clock_fault=!std::isfinite(now)||now<0||now<m_last_now;
    if(!m_clock_fault){m_boundary=std::max(m_boundary,now);m_last_now=now;}
    clear("connection_reset");m_reset_reason=reason;m_next_publish=-1;
  }
  void onInputBoundary() override{boundary();}
  void onMessageWithMetadata(const mavlink_message_t& message,const MessageTime& time) override
  {
    // Host owns peer selection, framing/CRC/signature policy. Reject impossible
    // decoded shapes here too; MAVLink 2 may legitimately trim below MIN_LEN.
    unsigned minimum=0,maximum=0;
    switch(message.msgid) {
      case MAVLINK_MSG_ID_HEARTBEAT:minimum=MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN;maximum=MAVLINK_MSG_ID_HEARTBEAT_LEN;break;
      case MAVLINK_MSG_ID_SYS_STATUS:minimum=MAVLINK_MSG_ID_SYS_STATUS_MIN_LEN;maximum=MAVLINK_MSG_ID_SYS_STATUS_LEN;break;
      case MAVLINK_MSG_ID_BATTERY_STATUS:minimum=MAVLINK_MSG_ID_BATTERY_STATUS_MIN_LEN;maximum=MAVLINK_MSG_ID_BATTERY_STATUS_LEN;break;
      case MAVLINK_MSG_ID_BATTERY_INFO:minimum=MAVLINK_MSG_ID_BATTERY_INFO_MIN_LEN;maximum=MAVLINK_MSG_ID_BATTERY_INFO_LEN;break;
      case MAVLINK_MSG_ID_GENERATOR_STATUS:minimum=MAVLINK_MSG_ID_GENERATOR_STATUS_MIN_LEN;maximum=MAVLINK_MSG_ID_GENERATOR_STATUS_LEN;break;
      case MAVLINK_MSG_ID_GPS_RAW_INT:minimum=MAVLINK_MSG_ID_GPS_RAW_INT_MIN_LEN;maximum=MAVLINK_MSG_ID_GPS_RAW_INT_LEN;break;
      case MAVLINK_MSG_ID_GPS2_RAW:minimum=MAVLINK_MSG_ID_GPS2_RAW_MIN_LEN;maximum=MAVLINK_MSG_ID_GPS2_RAW_LEN;break;
      case MAVLINK_MSG_ID_GPS_RTK:minimum=MAVLINK_MSG_ID_GPS_RTK_MIN_LEN;maximum=MAVLINK_MSG_ID_GPS_RTK_LEN;break;
      case MAVLINK_MSG_ID_GPS2_RTK:minimum=MAVLINK_MSG_ID_GPS2_RTK_MIN_LEN;maximum=MAVLINK_MSG_ID_GPS2_RTK_LEN;break;
      case MAVLINK_MSG_ID_STATUSTEXT:minimum=MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN;maximum=MAVLINK_MSG_ID_STATUSTEXT_LEN;break;
      case MAVLINK_MSG_ID_POWER_STATUS:minimum=MAVLINK_MSG_ID_POWER_STATUS_MIN_LEN;maximum=MAVLINK_MSG_ID_POWER_STATUS_LEN;break;
      case MAVLINK_MSG_ID_MCU_STATUS:maximum=MAVLINK_MSG_ID_MCU_STATUS_LEN;break;
      case MAVLINK_MSG_ID_EKF_STATUS_REPORT:minimum=MAVLINK_MSG_ID_EKF_STATUS_REPORT_MIN_LEN;maximum=MAVLINK_MSG_ID_EKF_STATUS_REPORT_LEN;break;
      case MAVLINK_MSG_ID_ESTIMATOR_STATUS:minimum=MAVLINK_MSG_ID_ESTIMATOR_STATUS_MIN_LEN;maximum=MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN;break;
      default:return;
    }
    if(!boundary()||!validTime(time)){++m_metadata_rejected;return;}
    if((message.magic==MAVLINK_STX&&(message.len==0||message.len>maximum||(message.incompat_flags&~MAVLINK_IFLAG_SIGNED)))||
       (message.magic==MAVLINK_STX_MAVLINK1&&(message.msgid>255||message.len!=minimum))||
       (message.magic!=MAVLINK_STX&&message.magic!=MAVLINK_STX_MAVLINK1)){++m_wire_rejected;return;}
    const auto now=m_services.now();
    if(message.msgid==MAVLINK_MSG_ID_HEARTBEAT) {
      mavlink_heartbeat_t heartbeat{};mavlink_msg_heartbeat_decode(&message,&heartbeat);
      const auto& vehicle=m_services.vehicle();
      if(!heartbeatFresh(now)||now-time.effective_at>2||heartbeat.autopilot!=MAV_AUTOPILOT_ARDUPILOTMEGA||heartbeat.type!=vehicle.type||heartbeat.autopilot!=vehicle.autopilot||message.sysid==0||message.compid==0){++m_source_rejected;return;}
      if(m_bound&&(message.sysid!=m_system||message.compid!=m_component)){++m_source_rejected;return;}
      if(m_bound&&(time.effective_at<m_heartbeat_at||time.read_at<m_heartbeat_read)){++m_order_rejected;return;}
      m_bound=true;m_system=message.sysid;m_component=message.compid;m_heartbeat_at=time.effective_at;m_heartbeat_read=time.read_at;return;
    }
    if(!m_bound||!heartbeatFresh(now)||now-m_heartbeat_at>2||message.sysid!=m_system||message.compid!=m_component){++m_source_rejected;return;}
    if(message.msgid==MAVLINK_MSG_ID_SYS_STATUS)handleSystem(message,time);
    else if(message.msgid==MAVLINK_MSG_ID_BATTERY_STATUS)handleBattery(message,time);
    else if(message.msgid==MAVLINK_MSG_ID_GPS_RAW_INT)handleGps(message,time);
    else if(message.msgid==MAVLINK_MSG_ID_STATUSTEXT)handleText(message,time);
    else if(time.effective_at<m_heartbeat_at||time.read_at<m_heartbeat_read) ++m_order_rejected;
    else if(message.msgid==MAVLINK_MSG_ID_GPS2_RAW)handleGps2(message,time);
    else if(message.msgid==MAVLINK_MSG_ID_GPS_RTK)
      handleRtk(m_rtk,message,time,mavlink_msg_gps_rtk_decode,"GPS_RTK_primary");
    else if(message.msgid==MAVLINK_MSG_ID_GPS2_RTK)
      handleRtk(m_rtk2,message,time,mavlink_msg_gps2_rtk_decode,"GPS2_RTK_secondary");
    else if(message.msgid==MAVLINK_MSG_ID_POWER_STATUS)handlePower(message,time);
    else if(message.msgid==MAVLINK_MSG_ID_BATTERY_INFO)handleBatteryInfo(message,time);
    else if(message.msgid==MAVLINK_MSG_ID_GENERATOR_STATUS)handleGenerator(message,time);
    else if(message.msgid==MAVLINK_MSG_ID_MCU_STATUS)handleMcu(message,time);
    else if(message.msgid==MAVLINK_MSG_ID_EKF_STATUS_REPORT)handleEkf(message,time);
    else if(message.msgid==MAVLINK_MSG_ID_ESTIMATOR_STATUS)handleEstimator(message,time);
  }
  void iterate() override
  {
    boundary();const double now=m_services.now();
    while(!m_events.empty()){m_services.publish("MAVMOOS_FC_TEXT",m_events.front());m_events.pop_front();}
    if(!std::isfinite(now)||(m_next_publish>=0&&now<m_next_publish))return;
    m_next_publish=now+m_interval;
    m_services.publish("MAVMOOS_SYS_STATUS",record("SYS_STATUS",m_system_status));
    m_services.publish("MAVMOOS_GPS_STATUS",record("GPS_RAW_INT",m_gps));
    const auto gps2=record("GPS2_RAW",m_gps2,true);
    const auto rtk=record("GPS_RTK",m_rtk,true);
    const auto rtk2=record("GPS2_RTK",m_rtk2,true);
    m_services.publish("MAVMOOS_GPS2_STATUS",gps2);
    m_services.publish("MAVMOOS_GPS_RTK",rtk);
    m_services.publish("MAVMOOS_GPS2_RTK",rtk2);
    if(!m_fallback.present&&m_batteries.empty())m_services.publish("MAVMOOS_BATTERY_STATUS",record("BATTERY_STATUS",Sample{}));
    if(m_fallback.present)m_services.publish("MAVMOOS_BATTERY_STATUS",record("SYS_STATUS_BATTERY",m_fallback));
    for(const auto& entry:m_batteries)m_services.publish("MAVMOOS_BATTERY_STATUS",record("BATTERY_STATUS",entry.second));
    std::string batteries="[";bool first=true;
    for(const auto& entry:m_batteries){if(!first)batteries+=',';first=false;batteries+=record("BATTERY_STATUS",entry.second);}batteries+=']';
    const auto power=record("POWER_STATUS",m_power),ekf=record("EKF_STATUS_REPORT",m_ekf);
    const auto estimator=record("ESTIMATOR_STATUS",m_estimator),mcus=mcuSnapshot();
    m_services.publish("MAVMOOS_POWER_STATUS",power);
    m_services.publish("MAVMOOS_BATTERY_INFO",batteryInfoSnapshot());
    m_services.publish("MAVMOOS_GENERATOR_STATUS",record("GENERATOR_STATUS",m_generator,true));
    m_services.publish("MAVMOOS_MCU_STATUS",mcus);
    m_services.publish("MAVMOOS_EKF_STATUS_REPORT",ekf);
    m_services.publish("MAVMOOS_ESTIMATOR_STATUS",estimator);
    const std::string state=m_clock_fault?"CLOCK_FAULT":m_bound?"BOUND":"WAITING_HEARTBEAT";
    m_services.publish("MAVMOOS_SYSTEM_STATUS","{\"schema\":1,\"session_epoch\":"+std::to_string(m_session)+",\"session_boundary\":"+decimal(m_boundary)+",\"state\":"+jsonString(state)+",\"read_only\":true,\"authoritative_for_control\":false,\"system_id\":"+(m_bound?std::to_string(m_system):"null")+
      ",\"component_id\":"+(m_bound?std::to_string(m_component):"null")+",\"reset_reason\":"+jsonString(m_reset_reason)+",\"metadata_rejected\":"+std::to_string(m_metadata_rejected)+
      ",\"source_rejected\":"+std::to_string(m_source_rejected)+",\"wire_shape_rejected\":"+std::to_string(m_wire_rejected)+",\"out_of_order_rejected\":"+std::to_string(m_order_rejected)+",\"battery_capacity_rejected\":"+std::to_string(m_battery_capacity)+
      ",\"text_capacity_rejected\":"+std::to_string(m_text_capacity)+",\"text_duplicates\":"+std::to_string(m_text_duplicates)+",\"text_events_dropped\":"+std::to_string(m_events_dropped)+
      ",\"text_assemblies\":"+std::to_string(m_text.size())+",\"battery_ids\":"+std::to_string(m_batteries.size())+",\"system\":"+record("SYS_STATUS",m_system_status)+
      ",\"gps\":"+record("GPS_RAW_INT",m_gps)+",\"sys_status_battery_fallback\":"+record("SYS_STATUS_BATTERY",m_fallback)+",\"batteries\":"+batteries+
      ",\"power\":"+power+",\"mcu\":"+mcus+",\"ekf\":"+ekf+",\"estimator\":"+estimator+
      ",\"gps2\":"+gps2+",\"gps_rtk\":"+rtk+",\"gps2_rtk\":"+rtk2+"}");
  }
private:
  bool heartbeatFresh(double now) const
  {
    const auto& v=m_services.vehicle();return v.heartbeat_valid&&v.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA&&std::isfinite(v.heartbeat_at)&&v.heartbeat_at>=0&&now>=v.heartbeat_at&&now-v.heartbeat_at<=2;
  }
  bool validTime(const MessageTime& time) const
  {
    return !m_clock_fault&&std::isfinite(time.effective_at)&&std::isfinite(time.read_at)&&time.effective_at>=0&&time.effective_at>m_boundary&&time.read_at>=time.effective_at&&time.read_at<=m_services.now()&&
      (time.age_basis=="QUEUE_EMPTY_FLOOR"||(time.age_basis=="READ_ACQUISITION"&&time.effective_at==time.read_at));
  }
  bool boundary()
  {
    const double now=m_services.now();
    if(m_clock_fault||!std::isfinite(now)||now<0||now<m_last_now){if(!m_clock_fault)clear("clock_fault");m_clock_fault=true;return false;}
    m_last_now=now;
    if(m_bound&&(!heartbeatFresh(now)||now-m_heartbeat_at>2)){m_boundary=std::max(m_boundary,now);clear("heartbeat_expired");m_next_publish=-1;}
    for(auto it=m_text.begin();it!=m_text.end();) {
      if(now>=it->second.expires){textOutcome(it->second,"incomplete","original_chunk_deadline");it=m_text.erase(it);}else ++it;
    }
    return true;
  }
  void clear(const std::string& reason)
  {
    m_events_dropped+=m_events.size();m_events.clear();
    for(const auto& entry:m_text)textOutcome(entry.second,"incomplete",reason);
    m_text.clear();m_batteries.clear();m_system_status={};m_gps={};m_fallback={};m_bound=false;m_heartbeat_at=m_heartbeat_read=-1;m_reset_reason=reason;m_next_publish=-1;++m_session;
    m_mcus.clear();m_battery_info.clear();m_generator={};m_power={};m_ekf={};m_estimator={};m_gps2={};m_rtk={};m_rtk2={};
    enqueue("{\"schema\":1,\"status\":\"invalidated\",\"reason\":"+jsonString(reason)+",\"session_epoch\":"+std::to_string(m_session)+"}");
  }
  bool newer(const Sample& sample,const MessageTime& time)
  {
    if(sample.present&&(time.effective_at<sample.time.effective_at||time.read_at<sample.time.read_at)){++m_order_rejected;return false;}return true;
  }
  void store(Sample& sample,const MessageTime& time,std::string data,bool valid=true)
  {sample.present=true;sample.encoding_valid=valid;sample.time=time;sample.data=std::move(data);sample.observation=++m_observation;}
  std::string record(const std::string& family,const Sample& sample,bool identify_source=false) const
  {
    const double now=m_services.now();const bool clock=std::isfinite(now)&&sample.present&&now>=sample.time.effective_at;
    const double age=clock?now-sample.time.effective_at:0;
    const bool fresh=sample.present&&!m_clock_fault&&m_bound&&heartbeatFresh(now)&&clock&&age<=m_timeout;
    const std::string source=identify_source?
      std::string(",\"read_only\":true,\"authoritative_for_control\":false,\"system_id\":")+
        (sample.present?std::to_string(m_system):"null")+",\"component_id\":"+
        (sample.present?std::to_string(m_component):"null")+",\"autopilot\":"+
        (sample.present?std::to_string(MAV_AUTOPILOT_ARDUPILOTMEGA):"null"):"";
    return "{\"schema\":1,\"session_epoch\":"+std::to_string(m_session)+",\"family\":"+jsonString(family)+",\"present\":"+boolean(sample.present)+",\"encoding_valid\":"+boolean(sample.present&&sample.encoding_valid)+
      ",\"fresh\":"+boolean(fresh)+",\"effective_at\":"+(sample.present?decimal(sample.time.effective_at):"null")+",\"read_at\":"+(sample.present?decimal(sample.time.read_at):"null")+
      ",\"age_s\":"+(clock?decimal(age):"null")+",\"age_basis\":"+(sample.present?jsonString(sample.time.age_basis):"null")+",\"age_is_upper_bound\":"+boolean(sample.present&&sample.time.age_basis=="QUEUE_EMPTY_FLOOR")+
      ",\"source_age\":\"unknown\",\"source_quality\":\"unassessed\",\"observation_id\":"+std::to_string(sample.observation)+source+",\"data\":"+(sample.present?sample.data:"null")+"}";
  }
  std::string mcuSnapshot() const
  {
    std::string instances="[";
    for(const auto& entry:m_mcus) {
      if(instances.size()>1) instances+=',';
      instances+=record("MCU_STATUS",entry.second);
    }
    return "{\"schema\":1,\"session_epoch\":"+std::to_string(m_session)+
      ",\"present\":"+boolean(!m_mcus.empty())+",\"atomic\":false,\"capacity\":"+std::to_string(mcu_limit)+
      ",\"capacity_rejected\":"+std::to_string(m_mcu_capacity)+",\"instances\":"+instances+"]}";
  }
  std::string batteryInfoSnapshot() const
  {
    std::string rows="[";
    for(const auto& [id,sample]:m_battery_info) {
      (void)id;
      if(rows.size()>1) rows+=',';
      rows+=record("BATTERY_INFO",sample,true);
    }
    return "{\"schema\":1,\"read_only\":true,\"authoritative_for_control\":false,\"atomic\":false,\"present\":"+
      boolean(!m_battery_info.empty())+",\"session_epoch\":"+std::to_string(m_session)+
      ",\"capacity\":"+std::to_string(battery_limit)+",\"capacity_rejected\":"+std::to_string(m_battery_info_capacity)+
      ",\"instances\":"+rows+"]}";
  }
  void handleBatteryInfo(const mavlink_message_t& message,const MessageTime& time)
  {
    mavlink_battery_info_t p{};
    mavlink_msg_battery_info_decode(&message,&p);
    auto found=m_battery_info.find(p.id);
    if(found==m_battery_info.end()) {
      if(m_battery_info.size()>=battery_limit){++m_battery_info_capacity;return;}
      found=m_battery_info.emplace(p.id,Sample{}).first;
    }
    if(!newer(found->second,time)) return;
    const auto nonnegative=[](float value,const char* unit){return diagnosticFloat(value,unit,true,true);};
    store(found->second,time,"{\"battery_id\":"+std::to_string(p.id)+
      ",\"function_raw\":"+std::to_string(p.battery_function)+",\"chemistry_raw\":"+std::to_string(p.type)+
      ",\"design_capacity_ah\":"+nonnegative(p.design_capacity,"Ah")+
      ",\"full_charge_capacity_ah\":"+diagnosticFloat(p.full_charge_capacity,"Ah",false,true)+
      ",\"nominal_voltage_v\":"+nonnegative(p.nominal_voltage,"V")+
      ",\"min_discharge_cell_voltage_v\":"+nonnegative(p.discharge_minimum_voltage,"V")+
      ",\"min_charge_cell_voltage_v\":"+nonnegative(p.charging_minimum_voltage,"V")+
      ",\"min_resting_cell_voltage_v\":"+nonnegative(p.resting_minimum_voltage,"V")+
      ",\"max_charged_cell_voltage_v\":"+nonnegative(p.charging_maximum_voltage,"V")+
      ",\"max_charge_current_a\":"+nonnegative(p.charging_maximum_current,"A")+
      ",\"max_discharge_current_a\":"+nonnegative(p.discharge_maximum_current,"A")+
      ",\"max_burst_discharge_current_a\":"+nonnegative(p.discharge_maximum_burst_current,"A")+
      ",\"state_of_health_pct\":"+quantity(p.state_of_health,p.state_of_health<=100,"pct",
        p.state_of_health==UINT8_MAX?"unreported":p.state_of_health>100?"out_of_range":"wire_value")+
      ",\"cycles\":"+quantity(p.cycle_count,p.cycle_count!=UINT16_MAX,"count",p.cycle_count==UINT16_MAX?"unreported":"wire_value")+
      ",\"weight_g\":"+quantity(p.weight,p.weight!=0,"g",p.weight==0?"zero_unreported":"wire_value")+
      ",\"cells_in_series\":"+quantity(p.cells_in_series,p.cells_in_series!=0,"count",p.cells_in_series==0?"zero_unreported":"wire_value")+
      ",\"serial_number\":"+boundedAscii(p.serial_number)+",\"device_name\":"+boundedAscii(p.name)+
      ",\"manufacture_date\":"+boundedAscii(p.manufacture_date)+
      ",\"payload_length\":"+std::to_string(message.len)+"}");
  }
  void handleGenerator(const mavlink_message_t& message,const MessageTime& time)
  {
    if(!newer(m_generator,time)) return;
    mavlink_generator_status_t p{};
    mavlink_msg_generator_status_decode(&message,&p);
    const auto sentinels=[](auto value,auto absent,const char* unit) {
      return quantity(value,value!=absent,unit,value==absent?"unreported":"wire_value");
    };
    store(m_generator,time,"{\"flags\":"+generatorFlags(p.status)+
      ",\"battery_current_a\":"+diagnosticFloat(p.battery_current,"A")+
      ",\"load_current_a\":"+diagnosticFloat(p.load_current,"A")+
      ",\"power_generated_w\":"+diagnosticFloat(p.power_generated,"W")+
      ",\"bus_voltage_v\":"+diagnosticFloat(p.bus_voltage,"V",false,true)+
      ",\"battery_current_setpoint_a\":"+diagnosticFloat(p.bat_current_setpoint,"A")+
      ",\"runtime_s\":"+sentinels(p.runtime,UINT32_MAX,"s")+
      ",\"maintenance_in_s\":"+sentinels(p.time_until_maintenance,INT32_MAX,"s")+
      ",\"generator_speed_rpm\":"+sentinels(p.generator_speed,UINT16_MAX,"rpm")+
      ",\"rectifier_temperature_c\":"+sentinels(p.rectifier_temperature,INT16_MAX,"degC")+
      ",\"generator_temperature_c\":"+sentinels(p.generator_temperature,INT16_MAX,"degC")+
      ",\"payload_length\":"+std::to_string(message.len)+"}");
  }
  void handlePower(const mavlink_message_t& message,const MessageTime& time)
  {
    if(!newer(m_power,time)) return;
    mavlink_power_status_t p{};
    mavlink_msg_power_status_decode(&message,&p);
    const auto flags=flagReport(p.flags,{
      {MAV_POWER_STATUS_BRICK_VALID,"brick_valid"},{MAV_POWER_STATUS_SERVO_VALID,"servo_valid"},
      {MAV_POWER_STATUS_USB_CONNECTED,"usb_connected"},{MAV_POWER_STATUS_PERIPH_OVERCURRENT,"peripheral_overcurrent"},
      {MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT,"high_power_peripheral_overcurrent"},
      {MAV_POWER_STATUS_CHANGED,"changed_since_boot"}
    });
    store(m_power,time,"{\"board_rail_mv\":"+std::to_string(p.Vcc)+",\"servo_rail_mv\":"+std::to_string(p.Vservo)+
      ",\"board_rail_v\":"+railVoltage(p.Vcc)+",\"servo_rail_v\":"+railVoltage(p.Vservo)+",\"flags\":"+flags+"}");
  }
  void handleMcu(const mavlink_message_t& message,const MessageTime& time)
  {
    mavlink_mcu_status_t p{};
    mavlink_msg_mcu_status_decode(&message,&p);
    const unsigned id=p.id;
    auto found=m_mcus.find(id);
    if(found==m_mcus.end()) {
      if(m_mcus.size()==mcu_limit) {++m_mcu_capacity;return;}
      found=m_mcus.try_emplace(id).first;
    }
    auto& sample=found->second;
    if(!newer(sample,time)) return;
    const bool comparable=p.MCU_voltage&&p.MCU_voltage_min&&p.MCU_voltage_max;
    const bool within=p.MCU_voltage_min<=p.MCU_voltage&&p.MCU_voltage<=p.MCU_voltage_max;
    store(sample,time,"{\"id\":"+std::to_string(id)+",\"temperature_cdeg\":"+std::to_string(p.MCU_temperature)+
      ",\"voltage_mv\":"+std::to_string(p.MCU_voltage)+",\"voltage_min_mv\":"+std::to_string(p.MCU_voltage_min)+
      ",\"voltage_max_mv\":"+std::to_string(p.MCU_voltage_max)+
      ",\"temperature_c\":"+quantity(p.MCU_temperature*.01,true,"degC","reported_mcu_temperature")+
      ",\"voltage_v\":"+railVoltage(p.MCU_voltage)+",\"voltage_min_v\":"+railVoltage(p.MCU_voltage_min)+
      ",\"voltage_max_v\":"+railVoltage(p.MCU_voltage_max)+
      ",\"reading_within_reported_range\":"+(comparable?boolean(within):"null")+"}");
  }
  void handleEkf(const mavlink_message_t& message,const MessageTime& time)
  {
    if(!newer(m_ekf,time)) return;
    mavlink_ekf_status_report_t p{};
    mavlink_msg_ekf_status_report_decode(&message,&p);
    const bool v1=message.magic==MAVLINK_STX_MAVLINK1;
    const bool airspeed_bytes=!v1&&message.len>MAVLINK_MSG_ID_EKF_STATUS_REPORT_MIN_LEN;
    const bool valid=validMetric(p.velocity_variance)&&validMetric(p.pos_horiz_variance)&&
      validMetric(p.pos_vert_variance)&&validMetric(p.compass_variance)&&validMetric(p.terrain_alt_variance)&&
      (!airspeed_bytes||validMetric(p.airspeed_variance));
    const char* airspeed_reason=v1?"mavlink1_extension_absent":
      p.airspeed_variance==0?"zero_or_unreported":nullptr;
    store(m_ekf,time,"{\"flags\":"+estimatorFlags(p.flags,true)+
      ",\"metric_semantics\":\"ardupilot_normalized_innovation_levels\",\"airspeed_extension_bytes\":"+boolean(airspeed_bytes)+
      ",\"velocity\":"+metric(p.velocity_variance,"ratio")+",\"horizontal_position\":"+metric(p.pos_horiz_variance,"ratio")+
      ",\"vertical_position\":"+metric(p.pos_vert_variance,"ratio")+",\"compass\":"+metric(p.compass_variance,"ratio")+
      ",\"terrain_altitude\":"+metric(p.terrain_alt_variance,"ratio",p.terrain_alt_variance==0?"zero_or_unused":nullptr)+
      ",\"airspeed\":"+metric(p.airspeed_variance,"ratio",airspeed_reason)+"}",valid);
  }
  void handleEstimator(const mavlink_message_t& message,const MessageTime& time)
  {
    if(!newer(m_estimator,time)) return;
    mavlink_estimator_status_t p{};
    mavlink_msg_estimator_status_decode(&message,&p);
    const bool valid=validMetric(p.vel_ratio)&&validMetric(p.pos_horiz_ratio)&&validMetric(p.pos_vert_ratio)&&
      validMetric(p.mag_ratio)&&validMetric(p.hagl_ratio)&&validMetric(p.tas_ratio)&&
      validMetric(p.pos_horiz_accuracy)&&validMetric(p.pos_vert_accuracy);
    store(m_estimator,time,"{\"flags\":"+estimatorFlags(p.flags,false)+
      ",\"time_usec_raw\":"+jsonString(std::to_string(p.time_usec))+",\"remote_clock\":\"boot_or_unix_unmapped\""+
      ",\"velocity_ratio\":"+metric(p.vel_ratio,"ratio")+",\"horizontal_position_ratio\":"+metric(p.pos_horiz_ratio,"ratio")+
      ",\"vertical_position_ratio\":"+metric(p.pos_vert_ratio,"ratio")+",\"magnetometer_ratio\":"+metric(p.mag_ratio,"ratio")+
      ",\"height_above_ground_ratio\":"+metric(p.hagl_ratio,"ratio")+",\"true_airspeed_ratio\":"+metric(p.tas_ratio,"ratio")+
      ",\"horizontal_accuracy_m\":"+metric(p.pos_horiz_accuracy,"m")+
      ",\"vertical_accuracy_m\":"+metric(p.pos_vert_accuracy,"m")+"}",valid);
  }
  void handleSystem(const mavlink_message_t& message,const MessageTime& time)
  {
    if(!newer(m_system_status,time))return;
    mavlink_sys_status_t p{};mavlink_msg_sys_status_decode(&message,&p);
    const bool extension=message.magic!=MAVLINK_STX_MAVLINK1&&message.len>MAVLINK_MSG_ID_SYS_STATUS_MIN_LEN;
    const bool prearm_present=(p.onboard_control_sensors_present&MAV_SYS_STATUS_PREARM_CHECK)!=0;
    const bool prearm_enabled=(p.onboard_control_sensors_enabled&MAV_SYS_STATUS_PREARM_CHECK)!=0;
    const bool prearm_healthy=(p.onboard_control_sensors_health&MAV_SYS_STATUS_PREARM_CHECK)!=0;
    store(m_system_status,time,"{\"sensor_masks\":"+masks(p.onboard_control_sensors_present,p.onboard_control_sensors_enabled,p.onboard_control_sensors_health)+
      ",\"extended_sensor_masks\":"+masks(p.onboard_control_sensors_present_extended,p.onboard_control_sensors_enabled_extended,p.onboard_control_sensors_health_extended)+
      ",\"prearm_check\":{\"present\":"+boolean(prearm_present)+",\"enabled\":"+boolean(prearm_enabled)+
      ",\"reported_healthy\":"+(prearm_present&&prearm_enabled?boolean(prearm_healthy):"null")+"}"+
      ",\"extension_bytes_observed\":"+boolean(extension)+",\"zero_extension_support\":\"not_distinguishable\",\"load_pct\":"+quantity(p.load/10.,p.load<=1000,"percent",p.load<=1000?"wire_value":"out_of_range")+
      ",\"drop_rate_pct\":"+quantity(p.drop_rate_comm/100.,p.drop_rate_comm<=10000,"percent",p.drop_rate_comm<=10000?"wire_value":"out_of_range")+
      ",\"errors_comm\":"+std::to_string(p.errors_comm)+",\"autopilot_error_counters\":["+std::to_string(p.errors_count1)+","+std::to_string(p.errors_count2)+","+std::to_string(p.errors_count3)+","+std::to_string(p.errors_count4)+"]}",p.load<=1000&&p.drop_rate_comm<=10000);
    store(m_fallback,time,"{\"origin\":\"SYS_STATUS\",\"battery_id\":null,\"identity\":\"ambiguous_sys_status_battery\",\"voltage_v\":"+quantity(p.voltage_battery/1000.,p.voltage_battery!=UINT16_MAX,"V",p.voltage_battery==UINT16_MAX?"unreported":"wire_value")+
      ",\"current_a\":"+quantity(p.current_battery/100.,p.current_battery!=-1,"A",p.current_battery==-1?"unreported":"wire_value_may_rollover")+
      ",\"remaining_pct\":"+quantity(p.battery_remaining,p.battery_remaining>=0&&p.battery_remaining<=100,"percent",p.battery_remaining==-1?"unreported":p.battery_remaining<0||p.battery_remaining>100?"out_of_range":"wire_value")+"}",p.battery_remaining>=-1&&p.battery_remaining<=100);
  }
  void handleBattery(const mavlink_message_t& message,const MessageTime& time)
  {
    mavlink_battery_status_t p{};mavlink_msg_battery_status_decode(&message,&p);
    auto existing=m_batteries.find(p.id);
    if(existing==m_batteries.end()&&m_batteries.size()>=battery_limit){++m_battery_capacity;return;}
    auto& sample=m_batteries[p.id];if(!newer(sample,time))return;
    std::string cells="[";double sum=0;unsigned count=0;bool gap=false,layout=true;
    for(unsigned i=0;i<14;++i) {
      const uint16_t raw=i<10?p.voltages[i]:p.voltages_ext[i-10];
      const bool known=raw!=UINT16_MAX&&(i<10||raw!=0);
      if(i)cells+=',';
      cells+="{\"slot\":"+std::to_string(i+1)+",\"raw_mv\":"+std::to_string(raw)+",\"voltage_v\":"+quantity(raw/1000.,known,"V",known?"wire_value":"unused_or_unreported")+"}";
      if(known){if(gap)layout=false;sum+=raw/1000.;++count;}else gap=true;
    }
    cells+=']';const bool percentage=p.battery_remaining>=0&&p.battery_remaining<=100;
    const bool fault_supported=p.charge_state==MAV_BATTERY_CHARGE_STATE_FAILED||p.charge_state==MAV_BATTERY_CHARGE_STATE_UNHEALTHY;
    const bool valid=layout&&p.battery_remaining>=-1&&p.battery_remaining<=100&&p.current_consumed>=-1&&p.energy_consumed>=-1&&p.time_remaining>=0;
    store(sample,time,"{\"origin\":\"BATTERY_STATUS\",\"battery_id\":"+std::to_string(p.id)+",\"function_raw\":"+std::to_string(p.battery_function)+",\"chemistry_raw\":"+std::to_string(p.type)+
      ",\"temperature_c\":"+quantity(p.temperature/100.,p.temperature!=INT16_MAX,"degC",p.temperature==INT16_MAX?"unreported":"wire_value")+
      ",\"current_a\":"+quantity(p.current_battery/100.,p.current_battery!=-1,"A",p.current_battery==-1?"unreported":"wire_value_may_rollover")+
      ",\"consumed_charge_mah\":"+quantity(p.current_consumed,p.current_consumed>=0,"mAh",p.current_consumed==-1?"unreported":p.current_consumed<0?"out_of_range":"wire_value")+
      ",\"consumed_energy_wh\":"+quantity(p.energy_consumed/36.,p.energy_consumed>=0,"Wh",p.energy_consumed==-1?"unreported":p.energy_consumed<0?"out_of_range":"wire_value")+
      ",\"remaining_pct\":"+quantity(p.battery_remaining,percentage,"percent",p.battery_remaining==-1?"unreported":!percentage?"out_of_range":"wire_value")+
      ",\"time_remaining_s\":"+quantity(p.time_remaining,p.time_remaining>0,"s",p.time_remaining==0?"unreported":p.time_remaining<0?"out_of_range":"wire_value")+
      ",\"voltage_slots\":"+cells+",\"voltage_layout_valid\":"+boolean(layout)+",\"reported_voltage_sum_v\":"+quantity(sum,count>0&&layout,"V",count==0?"unreported":!layout?"noncontiguous_slots":"sum_of_reported_slots")+
      ",\"reported_voltage_slots\":"+std::to_string(count)+",\"voltage_interpretation\":\"cells_or_split_pack_voltage\",\"extended_zero_note\":\"zero_is_unsupported_one_may_encode_zero\",\"extended_65535_note\":\"ardupilot_unused_sentinel\",\"charge_state_raw\":"+std::to_string(p.charge_state)+
      ",\"mode_raw\":"+std::to_string(p.mode)+",\"mode_zero_meaning\":\"normal_or_unsupported\",\"fault_bitmask_raw\":"+std::to_string(p.fault_bitmask)+",\"fault_reporting_supported\":"+boolean(fault_supported)+"}",valid);
  }
  void handleGps(const mavlink_message_t& message,const MessageTime& time)
  {
    if(!newer(m_gps,time)) return;
    mavlink_gps_raw_int_t packet{};
    mavlink_msg_gps_raw_int_decode(&message,&packet);
    storeGps(m_gps,time,packet,"GPS_RAW_INT_primary",message.len);
  }
  void handleGps2(const mavlink_message_t& message,const MessageTime& time)
  {
    if(!newer(m_gps2,time)) return;
    mavlink_gps2_raw_t packet{};
    mavlink_msg_gps2_raw_decode(&message,&packet);
    storeGps(m_gps2,time,packet,"GPS2_RAW_secondary",message.len);
  }
  template<typename Report>
  void storeGps(Sample& sample,const MessageTime& time,const Report& p,const char* receiver,
                unsigned payload_length)
  {
    constexpr bool secondary=std::is_same_v<Report,mavlink_gps2_raw_t>;
    const bool fix=p.fix_type<=GPS_FIX_TYPE_PPP;
    const bool coordinates=std::abs(double(p.lat))<=9e8&&std::abs(double(p.lon))<=18e8;
    const bool position=fix&&p.fix_type>=GPS_FIX_TYPE_2D_FIX&&coordinates;
    const bool altitude=fix&&p.fix_type>=GPS_FIX_TYPE_3D_FIX;
    const bool course=p.cog!=UINT16_MAX&&p.cog<36000;
    const bool yaw=p.yaw>0&&p.yaw<=36000;
    const auto extension=[payload_length](std::size_t offset,double value,bool known,const char* unit,
                                          const char* reason) {
      return !secondary||payload_length>offset?quantity(value,known,unit,reason):
        quantity(0,false,unit,"extension_not_on_wire");
    };
    std::string dgps;
    if constexpr(secondary) {
      dgps=",\"dgps_satellites_reported_raw\":"+std::to_string(p.dgps_numch)+
        ",\"dgps_age_reported_ms\":"+std::to_string(p.dgps_age)+
        ",\"dgps_availability\":"+jsonString(p.dgps_numch||p.dgps_age?"reported_unassessed":"zero_or_unreported");
    }
    store(sample,time,"{\"receiver\":"+jsonString(receiver)+
      ",\"fix_type_raw\":"+std::to_string(p.fix_type)+",\"fix_type_valid\":"+boolean(fix)+",\"remote_time_usec\":"+jsonString(std::to_string(p.time_usec))+
      ",\"remote_clock\":\"unmapped_boot_or_unix\",\"latitude_deg\":"+quantity(p.lat/1e7,position,"deg",position?"receiver_fix_and_encoding":"fix_or_coordinates_unavailable")+
      ",\"longitude_deg\":"+quantity(p.lon/1e7,position,"deg",position?"receiver_fix_and_encoding":"fix_or_coordinates_unavailable")+
      ",\"altitude_msl_m\":"+quantity(p.alt/1000.,altitude,"m",altitude?"receiver_fix":"no_3d_fix")+
      ",\"satellites\":"+quantity(p.satellites_visible,p.satellites_visible!=UINT8_MAX,"count",p.satellites_visible==UINT8_MAX?"unreported":"wire_value")+
      ",\"hdop\":"+quantity(p.eph/100.,p.eph!=UINT16_MAX,"ratio",p.eph==UINT16_MAX?"unreported":"wire_value")+
      ",\"vdop\":"+quantity(p.epv/100.,p.epv!=UINT16_MAX,"ratio",p.epv==UINT16_MAX?"unreported":"wire_value")+
      ",\"groundspeed_m_s\":"+quantity(p.vel/100.,p.vel!=UINT16_MAX,"m/s",p.vel==UINT16_MAX?"unreported":"wire_value")+
      ",\"course_deg\":"+quantity(p.cog/100.,course,"deg",p.cog==UINT16_MAX?"unreported":!course?"out_of_range":"wire_value_not_heading")+
      ",\"yaw_deg\":"+extension(offsetof(Report,yaw),p.yaw==36000?0:p.yaw/100.,yaw,"deg",p.yaw==0?"unsupported":p.yaw==UINT16_MAX?"configured_but_unavailable":!yaw?"out_of_range":"receiver_yaw")+
      ",\"ellipsoid_altitude_m\":"+extension(offsetof(Report,alt_ellipsoid),p.alt_ellipsoid/1000.,altitude&&p.alt_ellipsoid!=0,"m",p.alt_ellipsoid==0?"zero_or_unreported_extension":"receiver_fix")+
      ",\"horizontal_accuracy_m\":"+extension(offsetof(Report,h_acc),p.h_acc/1000.,p.h_acc!=0,"m",p.h_acc==0?"zero_or_unreported_extension":"wire_value")+
      ",\"vertical_accuracy_m\":"+extension(offsetof(Report,v_acc),p.v_acc/1000.,p.v_acc!=0,"m",p.v_acc==0?"zero_or_unreported_extension":"wire_value")+
      ",\"speed_accuracy_m_s\":"+extension(offsetof(Report,vel_acc),p.vel_acc/1000.,p.vel_acc!=0,"m/s",p.vel_acc==0?"zero_or_unreported_extension":"wire_value")+
      ",\"track_accuracy_deg\":"+extension(offsetof(Report,hdg_acc),p.hdg_acc/1e5,p.hdg_acc!=0,"deg",p.hdg_acc==0?"zero_or_unreported_extension":"wire_value")+
      dgps+"}",fix&&coordinates&&(p.cog==UINT16_MAX||course)&&(p.yaw==0||p.yaw==UINT16_MAX||yaw));
  }
  template<typename Report>
  void handleRtk(Sample& sample,const mavlink_message_t& message,const MessageTime& time,
                 void (*decode)(const mavlink_message_t*,Report*),const char* receiver)
  {
    if(!newer(sample,time)) return;
    Report p{};
    decode(&message,&p);
    const char* axes=nullptr;
    if(p.baseline_coords_type==RTK_BASELINE_COORDINATE_SYSTEM_ECEF) axes="ECEF_XYZ";
    if(p.baseline_coords_type==RTK_BASELINE_COORDINATE_SYSTEM_NED) axes="NORTH_EAST_DOWN";
    const auto raw="["+std::to_string(p.baseline_a_mm)+','+std::to_string(p.baseline_b_mm)+','+
      std::to_string(p.baseline_c_mm)+']';
    const auto metres=axes?"["+decimal(p.baseline_a_mm/1000.)+','+decimal(p.baseline_b_mm/1000.)+','+
      decimal(p.baseline_c_mm/1000.)+']':"null";
    store(sample,time,"{\"receiver\":"+jsonString(receiver)+
      ",\"time_last_baseline_ms_raw\":"+std::to_string(p.time_last_baseline_ms)+
      ",\"baseline_clock\":\"unmapped_boot_ms\",\"rtk_receiver_id_raw\":"+std::to_string(p.rtk_receiver_id)+
      ",\"gps_week_raw\":"+std::to_string(p.wn)+",\"gps_time_of_week_ms_raw\":"+std::to_string(p.tow)+
      ",\"rtk_health_raw\":"+std::to_string(p.rtk_health)+",\"rtk_rate_hz_raw\":"+std::to_string(p.rtk_rate)+
      ",\"rtk_satellites_raw\":"+std::to_string(p.nsats)+
      ",\"baseline_coords_type_raw\":"+std::to_string(p.baseline_coords_type)+
      ",\"baseline_axes\":"+(axes?jsonString(axes):"null")+
      ",\"baseline_mm_raw\":"+raw+",\"baseline_m\":"+metres+
      ",\"accuracy_raw\":"+std::to_string(p.accuracy)+",\"accuracy_unit\":\"unspecified\""+
      ",\"iar_num_hypotheses_raw\":"+std::to_string(p.iar_num_hypotheses)+
      ",\"receiver_metadata\":\"zero_can_be_unimplemented\",\"solution_quality\":\"unassessed\"}",axes!=nullptr);
  }
  void enqueue(std::string event)
  {if(m_events.size()>=event_limit){m_events.pop_front();++m_events_dropped;}m_events.push_back(std::move(event));}
  void textOutcome(const Assembly& text,const std::string& status,const std::string& reason)
  {
    const bool valid=utf8(text.bytes);const double now=m_services.now();
    enqueue("{\"schema\":1,\"session_epoch\":"+std::to_string(m_session)+",\"status\":"+jsonString(status)+",\"reason\":"+jsonString(reason)+",\"id\":"+std::to_string(text.id)+",\"severity_raw\":"+std::to_string(text.severity)+
      ",\"severity_valid\":"+boolean(text.severity<=MAV_SEVERITY_DEBUG)+",\"text\":"+jsonString(text.bytes,valid)+",\"utf8_valid\":"+boolean(valid)+",\"raw_hex\":"+jsonString(hex(text.bytes))+
      ",\"first_effective_at\":"+decimal(text.first.effective_at)+",\"last_effective_at\":"+decimal(text.last.effective_at)+",\"last_read_at\":"+decimal(text.last.read_at)+
      ",\"age_basis\":"+jsonString(text.first.age_basis)+",\"age_is_upper_bound\":"+boolean(text.first.age_basis=="QUEUE_EMPTY_FLOOR")+",\"age_s\":"+(std::isfinite(now)&&now>=text.last.effective_at?decimal(now-text.last.effective_at):"null")+",\"original_expires\":"+decimal(text.expires)+",\"published_at\":"+(std::isfinite(now)?decimal(now):"null")+",\"source_age\":\"unknown\",\"byte_count\":"+std::to_string(text.bytes.size())+"}");
  }
  void handleText(const mavlink_message_t& message,const MessageTime& time)
  {
    mavlink_statustext_t p{};mavlink_msg_statustext_decode(&message,&p);
    // Generated packets are packed; never forward a packed field by reference
    // into a standard-library template (id is at an odd byte offset).
    const unsigned id=p.id;
    const auto end=std::find(p.text,p.text+50,'\0');const bool final=end!=p.text+50;const std::string part(p.text,end);
    if(p.id==0) {
      Assembly value;value.severity=p.severity;value.first=value.last=time;value.bytes=part;value.expires=time.effective_at+m_text_timeout;
      textOutcome(value,m_services.now()<value.expires?"complete":"incomplete",m_services.now()<value.expires?"single_chunk_id_zero":"acquisition_expired");return;
    }
    auto found=m_text.find(id);
    if(found==m_text.end()) {
      if(p.chunk_seq!=0){Assembly value;value.id=p.id;value.severity=p.severity;value.first=value.last=time;value.bytes=part;value.expires=time.effective_at+m_text_timeout;textOutcome(value,"incomplete","missing_initial_chunk");return;}
      if(m_text.size()>=text_limit){++m_text_capacity;return;}
      Assembly value;value.id=p.id;value.severity=p.severity;value.first=value.last=time;value.expires=time.effective_at+m_text_timeout;value.bytes=part;value.last_bytes=part;
      if(m_services.now()>=value.expires){textOutcome(value,"incomplete","acquisition_expired");return;}
      if(final){textOutcome(value,"complete","terminated_initial_chunk");return;}
      m_text.emplace(id,std::move(value));return;
    }
    auto& value=found->second;
    if(time.effective_at<value.last.effective_at||time.read_at<value.last.read_at){++m_order_rejected;return;}
    if(p.chunk_seq==value.last_seq&&p.severity==value.severity&&part==value.last_bytes){++m_text_duplicates;return;}
    if(p.severity!=value.severity||p.chunk_seq!=value.next){textOutcome(value,"incomplete",p.severity!=value.severity?"severity_changed":"chunk_gap_or_conflicting_duplicate");m_text.erase(found);return;}
    if(value.next>=text_chunks){textOutcome(value,"incomplete","chunk_bound");m_text.erase(found);return;}
    value.bytes+=part;value.last_bytes=part;value.last_seq=p.chunk_seq;value.last=time;++value.next;
    if(final){textOutcome(value,"complete","terminated_chunk_sequence");m_text.erase(found);}
    else if(value.next>=text_chunks){textOutcome(value,"incomplete","chunk_bound_without_terminator");m_text.erase(found);}
  }
  Services& m_services;
  Sample m_system_status,m_gps,m_fallback,m_power,m_ekf,m_estimator;
  Sample m_gps2,m_rtk,m_rtk2,m_generator;
  std::map<unsigned,Sample> m_batteries,m_mcus,m_battery_info;
  std::map<unsigned,Assembly> m_text;
  std::deque<std::string> m_events;
  bool m_bound=false,m_clock_fault=false;
  unsigned m_system=0,m_component=0;
  double m_boundary=0,m_last_now=0,m_heartbeat_at=-1,m_heartbeat_read=-1,m_timeout=3,m_interval=.5,m_text_timeout=2,m_next_publish=-1;
  uint64_t m_session=1,m_observation=0,m_metadata_rejected=0,m_source_rejected=0,m_wire_rejected=0,m_order_rejected=0,m_battery_capacity=0,m_text_capacity=0,m_text_duplicates=0,m_events_dropped=0;
  uint64_t m_mcu_capacity=0,m_battery_info_capacity=0;
  std::string m_reset_reason="startup";
};
}
}
extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi,mavmoos::runtime::Services* services)
{return abi==mavmoos::runtime::plugin_abi&&services?new mavmoos::runtime::SystemStatus(*services):nullptr;}
extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin){delete plugin;}
