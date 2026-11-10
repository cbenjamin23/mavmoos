/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: RcIoPlugin.cpp                                  */
/*    DATE: November 4, 2026                                */
/************************************************************/
#include "Plugin.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>

namespace mavmoos::runtime {
namespace {
constexpr std::size_t port_capacity=8;
std::string boolean(bool value){return value?"true":"false";}
std::string decimal(double value){std::ostringstream out;out<<std::setprecision(17)<<value;return out.str();}
std::string jsonString(const std::string& value)
{
  static constexpr char hex[]="0123456789abcdef";std::string out="\"";
  for(unsigned char c:value){if(c=='"'||c=='\\'){out+='\\';out+=char(c);}else if(c<32||c>=127){out+="\\u00";out+=hex[c>>4];out+=hex[c&15];}else out+=char(c);}
  return out+'"';
}
std::string quantity(unsigned value,bool known,const char* unit,const char* reason)
{return "{\"known\":"+boolean(known)+",\"value\":"+(known?std::to_string(value):"null")+",\"unit\":"+jsonString(unit)+",\"reason\":"+jsonString(reason)+"}";}
double setting(const std::string& value,double low,double high)
{
  if(value.empty()||value.find_first_of(" \t\r\n")!=std::string::npos)throw std::invalid_argument("invalid rc_io setting");
  std::size_t used=0;const auto number=std::stod(value,&used);
  if(used!=value.size()||!std::isfinite(number)||number<low||number>high)throw std::invalid_argument("rc_io setting outside bounds");
  return number;
}
struct Sample{bool present=false;MessageTime time;uint64_t observation=0;std::string data;};
class RcIo final:public Plugin {
public:
  explicit RcIo(Services& services):m_services(services),m_boundary(services.now()),m_last_now(m_boundary)
  {if(!std::isfinite(m_boundary)||m_boundary<0){m_boundary=0;m_last_now=0;m_clock_fault=true;}}
  std::string name()const override{return "rc_io";}
  PluginDescriptor descriptor()const override
  {
    PluginDescriptor result;result.profile="ardupilot_rc_observations";
    result.capabilities={{"rc.input","","","observation"},{"rc.servo_output","","","observation"}};
    result.exclusive_resources={"observation.rc_io"};
    result.observation_streams={{MAVLINK_MSG_ID_RC_CHANNELS,100000},{MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,200000}};
    result.streamed_parameter_bench_compatible=true;return result;
  }
  void configure(const std::string& key,const std::string& value)override
  {if(key=="rc_io_timeout")m_timeout=setting(value,.25,10);else if(key=="rc_io_publish_interval")m_interval=setting(value,.02,2);}
  void onReset(const std::string& reason)override
  {
    // A reset is not permission to rewind the last trustworthy owner clock.
    const double now=m_services.now();m_clock_fault=!std::isfinite(now)||now<0||now<m_last_now;
    if(!m_clock_fault){m_boundary=std::max(m_boundary,now);m_last_now=now;}
    clear(reason);
  }
  void onInputBoundary()override{boundary();}
  void onMessageWithMetadata(const mavlink_message_t& message,const MessageTime& time)override
  {
    unsigned minimum=0,maximum=0;
    switch(message.msgid){
      case MAVLINK_MSG_ID_HEARTBEAT:minimum=MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN;maximum=MAVLINK_MSG_ID_HEARTBEAT_LEN;break;
      case MAVLINK_MSG_ID_RC_CHANNELS:minimum=MAVLINK_MSG_ID_RC_CHANNELS_MIN_LEN;maximum=MAVLINK_MSG_ID_RC_CHANNELS_LEN;break;
      case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:minimum=MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_MIN_LEN;maximum=MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN;break;
      default:return;
    }
    if(!boundary()||!validTime(time)){++m_metadata_rejected;return;}
    if((message.magic==MAVLINK_STX&&(message.len==0||message.len>maximum||(message.incompat_flags&~MAVLINK_IFLAG_SIGNED)))||
       (message.magic==MAVLINK_STX_MAVLINK1&&message.len!=minimum)||(message.magic!=MAVLINK_STX&&message.magic!=MAVLINK_STX_MAVLINK1))
      {++m_wire_rejected;return;}
    const double now=m_services.now();
    if(message.msgid==MAVLINK_MSG_ID_HEARTBEAT){
      mavlink_heartbeat_t p{};mavlink_msg_heartbeat_decode(&message,&p);const auto& vehicle=m_services.vehicle();
      if(!heartbeatFresh(now)||now-time.effective_at>2||p.autopilot!=MAV_AUTOPILOT_ARDUPILOTMEGA||p.type!=vehicle.type||p.autopilot!=vehicle.autopilot||message.sysid==0||message.compid==0||
        (m_bound&&(message.sysid!=m_system||message.compid!=m_component))){++m_source_rejected;return;}
      if(m_bound&&(time.effective_at<m_heartbeat_at||time.read_at<m_heartbeat_read)){++m_order_rejected;return;}
      m_bound=true;m_system=message.sysid;m_component=message.compid;m_heartbeat_at=time.effective_at;m_heartbeat_read=time.read_at;return;
    }
    if(!m_bound||!heartbeatFresh(now)||now-m_heartbeat_at>2||message.sysid!=m_system||message.compid!=m_component){++m_source_rejected;return;}
    if(message.msgid==MAVLINK_MSG_ID_RC_CHANNELS)input(message,time);else output(message,time);
  }
  void iterate()override
  {
    boundary();const double now=m_services.now();
    if(!std::isfinite(now)||(m_next_publish>=0&&now<m_next_publish))return;
    m_next_publish=now+m_interval;
    const auto inputs=record("RC_CHANNELS",m_input),outputs=ports();
    m_services.publish("MAVMOOS_RC_INPUT",inputs);m_services.publish("MAVMOOS_SERVO_OUTPUT",outputs);
    m_services.publish("MAVMOOS_RC_IO_STATUS","{\"schema\":1,\"session_epoch\":"+std::to_string(m_session)+",\"session_boundary\":"+decimal(m_boundary)+
      ",\"state\":"+jsonString(m_clock_fault?"CLOCK_FAULT":m_bound?"BOUND":"WAITING_HEARTBEAT")+",\"read_only\":true,\"authoritative_for_control\":false,\"atomic\":false,\"system_id\":"+(m_bound?std::to_string(m_system):"null")+
      ",\"component_id\":"+(m_bound?std::to_string(m_component):"null")+",\"reset_reason\":"+jsonString(m_reset_reason)+",\"metadata_rejected\":"+std::to_string(m_metadata_rejected)+
      ",\"source_rejected\":"+std::to_string(m_source_rejected)+",\"wire_shape_rejected\":"+std::to_string(m_wire_rejected)+",\"out_of_order_rejected\":"+std::to_string(m_order_rejected)+
      ",\"port_capacity_rejected\":"+std::to_string(m_port_rejected)+",\"input\":"+inputs+",\"servo_output\":"+outputs+"}");
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
  {m_input={};m_ports.clear();m_bound=false;m_heartbeat_at=m_heartbeat_read=-1;m_reset_reason=reason;m_next_publish=-1;++m_session;}
  bool newer(const Sample& sample,const MessageTime& time)
  {if(sample.present&&(time.effective_at<sample.time.effective_at||time.read_at<sample.time.read_at)){++m_order_rejected;return false;}return true;}
  void store(Sample& sample,const MessageTime& time,std::string data)
  {sample.present=true;sample.time=time;sample.observation=++m_observation;sample.data=std::move(data);}
  std::string record(const std::string& family,const Sample& sample)const
  {
    const double now=m_services.now();const bool clock=sample.present&&std::isfinite(now)&&now>=sample.time.effective_at;const double age=clock?now-sample.time.effective_at:0;
    const bool fresh=sample.present&&!m_clock_fault&&m_bound&&heartbeatFresh(now)&&clock&&age<=m_timeout;
    return "{\"schema\":1,\"session_epoch\":"+std::to_string(m_session)+",\"family\":"+jsonString(family)+",\"present\":"+boolean(sample.present)+",\"encoding_valid\":"+boolean(sample.present)+
      ",\"fresh\":"+boolean(fresh)+",\"effective_at\":"+(sample.present?decimal(sample.time.effective_at):"null")+",\"read_at\":"+(sample.present?decimal(sample.time.read_at):"null")+
      ",\"age_s\":"+(clock?decimal(age):"null")+",\"age_basis\":"+(sample.present?jsonString(sample.time.age_basis):"null")+",\"age_is_upper_bound\":"+boolean(sample.present&&sample.time.age_basis=="QUEUE_EMPTY_FLOOR")+
      ",\"source_age\":\"unknown\",\"source_quality\":\"unassessed\",\"observation_id\":"+std::to_string(sample.observation)+",\"data\":"+(sample.present?sample.data:"null")+"}";
  }
  std::string ports()const
  {
    std::string out="{\"schema\":1,\"session_epoch\":"+std::to_string(m_session)+",\"present\":"+boolean(!m_ports.empty())+",\"atomic\":false,\"cache_capacity\":"+std::to_string(port_capacity)+",\"ports\":[";
    bool first=true;for(const auto& port:m_ports){if(!first)out+=',';first=false;out+=record("SERVO_OUTPUT_RAW",port.second);}return out+"]}";
  }
  void input(const mavlink_message_t& message,const MessageTime& time)
  {
    if(!newer(m_input,time))return;
    mavlink_rc_channels_t p{};mavlink_msg_rc_channels_decode(&message,&p);
    const std::array<uint16_t,18> raw={p.chan1_raw,p.chan2_raw,p.chan3_raw,p.chan4_raw,p.chan5_raw,p.chan6_raw,p.chan7_raw,p.chan8_raw,p.chan9_raw,p.chan10_raw,p.chan11_raw,p.chan12_raw,p.chan13_raw,p.chan14_raw,p.chan15_raw,p.chan16_raw,p.chan17_raw,p.chan18_raw};
    const unsigned count=p.chancount,visible=std::min(count,18u),rssi=p.rssi;
    std::string channels="[";for(unsigned i=0;i<raw.size();++i){if(i)channels+=',';const bool reported=i<visible;
      channels+="{\"slot\":"+std::to_string(i+1)+",\"raw_us\":"+std::to_string(raw[i])+",\"reported_by_count\":"+boolean(reported)+",\"pwm_us\":"+quantity(raw[i],reported,"us",reported?"reported_channel":"outside_reported_count")+"}";}
    store(m_input,time,"{\"time_boot_ms\":"+std::to_string(p.time_boot_ms)+",\"remote_clock\":\"boot_ms_unmapped_uint32\",\"wire_version\":"+std::to_string(message.magic==MAVLINK_STX_MAVLINK1?1:2)+
      ",\"reported_channel_count\":"+std::to_string(count)+",\"visible_channel_count\":"+std::to_string(visible)+",\"channels_beyond_message\":"+boolean(count>18)+",\"channel_roles\":\"unmapped\",\"pulse_validity\":\"unassessed\",\"rssi\":{\"raw\":"+std::to_string(rssi)+
      ",\"quantity\":"+quantity(rssi,rssi!=255,"device_dependent",rssi==255?"invalid_or_unknown":"wire_rssi")+"},\"channels\":"+channels+"]}");
  }
  void output(const mavlink_message_t& message,const MessageTime& time)
  {
    mavlink_servo_output_raw_t p{};mavlink_msg_servo_output_raw_decode(&message,&p);
    // Copy packed members into aligned values before STL/reference-taking calls.
    const unsigned port=p.port,stamp=p.time_usec;
    auto found=m_ports.find(port);
    if(found==m_ports.end()){if(m_ports.size()==port_capacity){++m_port_rejected;return;}found=m_ports.emplace(port,Sample{}).first;}
    if(!newer(found->second,time))return;
    const std::array<uint16_t,16> raw={p.servo1_raw,p.servo2_raw,p.servo3_raw,p.servo4_raw,p.servo5_raw,p.servo6_raw,p.servo7_raw,p.servo8_raw,p.servo9_raw,p.servo10_raw,p.servo11_raw,p.servo12_raw,p.servo13_raw,p.servo14_raw,p.servo15_raw,p.servo16_raw};
    const bool v2=message.magic==MAVLINK_STX;std::string channels="[";
    for(unsigned i=0;i<raw.size();++i){if(i)channels+=',';const bool available=i<8||v2,known=available&&raw[i]!=0;
      const char* reason=!available?"mavlink1_extension_absent":raw[i]!=0?"reported_nonzero_output":i>=8?"zero_extension_unsupported_or_omitted_or_zero":"zero_disabled_or_unreported_or_zero";
      channels+="{\"slot\":"+std::to_string(i+1)+",\"raw_us\":"+(available?std::to_string(raw[i]):"null")+",\"extension\":"+boolean(i>=8)+",\"pwm_us\":"+quantity(raw[i],known,"us",reason)+"}";}
    store(found->second,time,"{\"time_usec\":"+std::to_string(stamp)+",\"remote_clock\":\"unmapped_uint32_us_no_epoch_inference\",\"port\":"+std::to_string(port)+",\"wire_version\":"+std::to_string(v2?2:1)+
      ",\"extension_bytes_observed\":"+boolean(v2&&message.len>MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_MIN_LEN)+",\"output_kind\":\"fc_command_observation\",\"physical_feedback\":false,\"actuator_mapping\":\"unmapped_port_and_slot\",\"pulse_validity\":\"unassessed\",\"channels\":"+channels+"]}");
  }
  Services& m_services;Sample m_input;std::map<unsigned,Sample> m_ports;
  bool m_bound=false,m_clock_fault=false;unsigned m_system=0,m_component=0;
  double m_boundary=0,m_last_now=0,m_heartbeat_at=-1,m_heartbeat_read=-1,m_timeout=1,m_interval=.1,m_next_publish=-1;
  uint64_t m_session=1,m_observation=0,m_metadata_rejected=0,m_source_rejected=0,m_wire_rejected=0,m_order_rejected=0,m_port_rejected=0;
  std::string m_reset_reason="startup";
};
}
}
extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi,mavmoos::runtime::Services* services)
{return abi==mavmoos::runtime::plugin_abi&&services?new mavmoos::runtime::RcIo(*services):nullptr;}
extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin){delete plugin;}
