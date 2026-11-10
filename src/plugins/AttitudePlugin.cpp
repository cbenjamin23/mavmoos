/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: AttitudePlugin.cpp                              */
/*    DATE: November 4, 2026                                */
/************************************************************/
#include "Plugin.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace mavmoos::runtime {
namespace {
constexpr double pi=3.14159265358979323846, norm_tolerance=.001;
std::string boolean(bool value){return value?"true":"false";}
std::string decimal(double value){std::ostringstream out;out<<std::setprecision(17)<<value;return out.str();}
std::string jsonString(const std::string& value)
{
  static constexpr char hex[]="0123456789abcdef";std::string result="\"";
  for(unsigned char c:value){if(c=='"'||c=='\\'){result+='\\';result+=char(c);}else if(c<32||c>=127){result+="\\u00";result+=hex[c>>4];result+=hex[c&15];}else result+=char(c);}
  return result+'"';
}
std::string quantity(double value,bool valid,const char* unit,const char* reason="wire_value")
{
  valid=valid&&std::isfinite(value);
  return "{\"known\":"+boolean(valid)+",\"value\":"+(valid?decimal(value):"null")+",\"unit\":"+jsonString(unit)+",\"reason\":"+jsonString(reason)+"}";
}
std::string vector(const std::array<double,3>& values)
{
  return "{\"x\":"+quantity(values[0],std::isfinite(values[0]),"rad/s",std::isfinite(values[0])?"wire_body_rate":"nonfinite")+
    ",\"y\":"+quantity(values[1],std::isfinite(values[1]),"rad/s",std::isfinite(values[1])?"wire_body_rate":"nonfinite")+
    ",\"z\":"+quantity(values[2],std::isfinite(values[2]),"rad/s",std::isfinite(values[2])?"wire_body_rate":"nonfinite")+"}";
}
bool finite(const std::array<double,3>& values){return std::all_of(values.begin(),values.end(),[](double x){return std::isfinite(x);});}
std::string array(const std::array<double,4>& values)
{
  std::string out="[";for(unsigned i=0;i<4;++i){if(i)out+=',';out+=std::isfinite(values[i])?decimal(values[i]):"null";}return out+']';
}
bool finite(const std::array<double,4>& values){return std::all_of(values.begin(),values.end(),[](double x){return std::isfinite(x);});}
double norm(const std::array<double,4>& values){return std::sqrt(values[0]*values[0]+values[1]*values[1]+values[2]*values[2]+values[3]*values[3]);}
bool unit(const std::array<double,4>& values){return finite(values)&&std::abs(norm(values)-1)<=norm_tolerance;}
double heading(double yaw){double result=std::fmod(yaw*180/pi,360);if(result<0)result+=360;return result>=360||result==0?0:result;}
std::string rpy(double roll,double pitch,double yaw,bool valid,bool source)
{
  const bool unique=valid&&std::abs(std::cos(pitch))>1e-6;
  return "{\"roll_rad\":"+quantity(roll,valid&&(source||unique),"rad",valid?(source?"wire_angle":unique?"derived_zyx":"pitch_singularity"):"invalid_orientation")+
    ",\"pitch_rad\":"+quantity(pitch,valid,"rad",valid?(source?"wire_angle":"derived_zyx"):"invalid_orientation")+
    ",\"yaw_rad\":"+quantity(yaw,valid&&(source||unique),"rad",valid?(source?"wire_angle":unique?"derived_zyx":"pitch_singularity"):"invalid_orientation")+
    ",\"source_yaw_heading_deg\":"+quantity(heading(yaw),unique,"deg",!valid?"invalid_orientation":!unique?"pitch_singularity":"yaw_wrapped_north_zero_clockwise")+
    ",\"rpy_unique\":"+boolean(unique)+"}";
}
double setting(const std::string& value,double low,double high)
{
  if(value.empty()||value.find_first_of(" \t\r\n")!=std::string::npos)throw std::invalid_argument("invalid attitude setting");
  std::size_t used=0;const auto result=std::stod(value,&used);
  if(used!=value.size()||!std::isfinite(result)||result<low||result>high)throw std::invalid_argument("attitude setting outside bounds");
  return result;
}
struct Sample{bool present=false,encoding_valid=false;MessageTime time;uint64_t observation=0;std::string data;};
class Attitude final:public Plugin {
public:
  explicit Attitude(Services& services):m_services(services),m_boundary(services.now()),m_last_now(m_boundary)
  {if(!std::isfinite(m_boundary)||m_boundary<0){m_boundary=0;m_last_now=0;m_clock_fault=true;}}
  std::string name()const override{return "attitude";}
  PluginDescriptor descriptor()const override
  {
    PluginDescriptor result;result.profile="ardupilot_attitude_observations";
    result.capabilities={{"attitude.euler","","","observation"},{"attitude.quaternion","","","observation"}};
    result.exclusive_resources={"observation.attitude"};
    result.observation_streams={{MAVLINK_MSG_ID_ATTITUDE,200000},{MAVLINK_MSG_ID_ATTITUDE_QUATERNION,200000}};
    result.streamed_parameter_bench_compatible=true;return result;
  }
  void configure(const std::string& key,const std::string& value)override
  {if(key=="attitude_timeout")m_timeout=setting(value,.25,10);else if(key=="attitude_publish_interval")m_interval=setting(value,.02,2);}
  void onReset(const std::string& reason)override
  {
    // Reset cannot legitimize observations before the last reliable owner time.
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
      case MAVLINK_MSG_ID_ATTITUDE:minimum=MAVLINK_MSG_ID_ATTITUDE_MIN_LEN;maximum=MAVLINK_MSG_ID_ATTITUDE_LEN;break;
      case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:minimum=MAVLINK_MSG_ID_ATTITUDE_QUATERNION_MIN_LEN;maximum=MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN;break;
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
    if(message.msgid==MAVLINK_MSG_ID_ATTITUDE)handleEuler(message,time);else handleQuaternion(message,time);
  }
  void iterate()override
  {
    boundary();const double now=m_services.now();
    if(!std::isfinite(now)||(m_next_publish>=0&&now<m_next_publish))return;
    m_next_publish=now+m_interval;
    m_services.publish("MAVMOOS_ATTITUDE_EULER",record("ATTITUDE",m_euler));
    m_services.publish("MAVMOOS_ATTITUDE_QUATERNION",record("ATTITUDE_QUATERNION",m_quaternion));
    m_services.publish("MAVMOOS_ATTITUDE_STATUS","{\"schema\":1,\"session_epoch\":"+std::to_string(m_session)+",\"session_boundary\":"+decimal(m_boundary)+
      ",\"state\":"+jsonString(m_clock_fault?"CLOCK_FAULT":m_bound?"BOUND":"WAITING_HEARTBEAT")+",\"read_only\":true,\"authoritative_for_control\":false,\"atomic\":false,\"family_selection\":\"independent_no_preference\",\"system_id\":"+(m_bound?std::to_string(m_system):"null")+
      ",\"component_id\":"+(m_bound?std::to_string(m_component):"null")+",\"reset_reason\":"+jsonString(m_reset_reason)+",\"metadata_rejected\":"+std::to_string(m_metadata_rejected)+
      ",\"source_rejected\":"+std::to_string(m_source_rejected)+",\"wire_shape_rejected\":"+std::to_string(m_wire_rejected)+",\"out_of_order_rejected\":"+std::to_string(m_order_rejected)+
      ",\"euler\":"+record("ATTITUDE",m_euler)+",\"quaternion\":"+record("ATTITUDE_QUATERNION",m_quaternion)+"}");
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
  {m_euler={};m_quaternion={};m_bound=false;m_heartbeat_at=m_heartbeat_read=-1;m_reset_reason=reason;m_next_publish=-1;++m_session;}
  bool newer(const Sample& sample,const MessageTime& time)
  {if(sample.present&&(time.effective_at<sample.time.effective_at||time.read_at<sample.time.read_at)){++m_order_rejected;return false;}return true;}
  void store(Sample& sample,const MessageTime& time,std::string data,bool valid)
  {sample.present=true;sample.encoding_valid=valid;sample.time=time;sample.observation=++m_observation;sample.data=std::move(data);}
  std::string record(const std::string& family,const Sample& sample)const
  {
    const double now=m_services.now();const bool clock=sample.present&&std::isfinite(now)&&now>=sample.time.effective_at;const double age=clock?now-sample.time.effective_at:0;
    const bool fresh=sample.present&&!m_clock_fault&&m_bound&&heartbeatFresh(now)&&clock&&age<=m_timeout;
    return "{\"schema\":1,\"session_epoch\":"+std::to_string(m_session)+",\"family\":"+jsonString(family)+",\"present\":"+boolean(sample.present)+",\"encoding_valid\":"+boolean(sample.present&&sample.encoding_valid)+
      ",\"fresh\":"+boolean(fresh)+",\"effective_at\":"+(sample.present?decimal(sample.time.effective_at):"null")+",\"read_at\":"+(sample.present?decimal(sample.time.read_at):"null")+
      ",\"age_s\":"+(clock?decimal(age):"null")+",\"age_basis\":"+(sample.present?jsonString(sample.time.age_basis):"null")+",\"age_is_upper_bound\":"+boolean(sample.present&&sample.time.age_basis=="QUEUE_EMPTY_FLOOR")+
      ",\"source_age\":\"unknown\",\"source_quality\":\"unassessed\",\"observation_id\":"+std::to_string(sample.observation)+",\"data\":"+(sample.present?sample.data:"null")+"}";
  }
  std::string common(uint32_t time,const std::array<double,3>& rates)const
  {return "\"time_boot_ms\":"+std::to_string(time)+",\"remote_clock\":\"boot_ms_unmapped_uint32\",\"reference_frame\":\"LOCAL_NED\",\"body_frame\":\"BODY_FRD\",\"estimate\":\"fused_fc_attitude\",\"angular_velocity_body_frd_rad_s\":"+vector(rates);}
  void handleEuler(const mavlink_message_t& message,const MessageTime& time)
  {
    if(!newer(m_euler,time))return;
    mavlink_attitude_t p{};mavlink_msg_attitude_decode(&message,&p);
    const std::array<double,3> rates={p.rollspeed,p.pitchspeed,p.yawspeed};
    const bool angles=std::isfinite(p.roll)&&std::isfinite(p.pitch)&&std::isfinite(p.yaw);
    // Preserve raw angles even outside the declared wire range. Only the
    // explicitly bounded finite representation may produce a display heading.
    const bool range=angles&&std::abs(p.roll)<=pi+1e-6&&std::abs(p.pitch)<=pi+1e-6&&std::abs(p.yaw)<=pi+1e-6;
    const bool wire_range=angles&&std::abs(p.roll)<=pi+1e-6&&std::abs(p.pitch)<=pi+1e-6&&std::abs(p.yaw)<=pi+1e-6;
    store(m_euler,time,"{"+common(p.time_boot_ms,rates)+",\"orientation_valid\":"+boolean(range)+",\"angular_velocity_valid\":"+boolean(finite(rates))+
      ",\"wire_declared_angle_range_conformant\":"+boolean(wire_range)+",\"profile_angle_range\":\"plusminus_pi_with_float_tolerance\",\"reported_rpy\":"+rpy(p.roll,p.pitch,p.yaw,range,true)+
      ",\"raw_angles_rad\":["+(std::isfinite(p.roll)?decimal(p.roll):"null")+","+(std::isfinite(p.pitch)?decimal(p.pitch):"null")+","+(std::isfinite(p.yaw)?decimal(p.yaw):"null")+"]}",range&&finite(rates));
  }
  void handleQuaternion(const mavlink_message_t& message,const MessageTime& time)
  {
    if(!newer(m_quaternion,time))return;
    mavlink_attitude_quaternion_t p{};mavlink_msg_attitude_quaternion_decode(&message,&p);
    const std::array<double,4> q={p.q1,p.q2,p.q3,p.q4},offset={p.repr_offset_q[0],p.repr_offset_q[1],p.repr_offset_q[2],p.repr_offset_q[3]};
    const std::array<double,3> rates={p.rollspeed,p.pitchspeed,p.yawspeed};const bool valid=unit(q),offset_zero=std::all_of(offset.begin(),offset.end(),[](double x){return x==0;});
    const bool offset_valid=offset_zero||unit(offset);const double length=finite(q)?norm(q):0;double roll=0,pitch=0,yaw=0;
    if(valid){const double w=q[0]/length,x=q[1]/length,y=q[2]/length,z=q[3]/length;
      roll=std::atan2(2*(w*x+y*z),1-2*(x*x+y*y));pitch=std::asin(std::clamp(2*(w*y-z*x),-1.,1.));yaw=std::atan2(2*(w*z+x*y),1-2*(y*y+z*z));}
    store(m_quaternion,time,"{"+common(p.time_boot_ms,rates)+",\"quaternion_wxyz\":"+array(q)+",\"quaternion_components_finite\":"+boolean(finite(q))+
      ",\"quaternion_norm\":"+(finite(q)?decimal(length):"null")+",\"unit_norm_tolerance\":"+decimal(norm_tolerance)+",\"orientation_valid\":"+boolean(valid)+",\"angular_velocity_valid\":"+boolean(finite(rates))+
      ",\"derived_rpy\":"+rpy(roll,pitch,yaw,valid,false)+",\"derived_normalized_within_tolerance\":"+boolean(valid)+",\"derived_convention\":\"hamilton_wxyz_zyx_in_source_ned_frd\",\"representation_offset\":{\"raw_wxyz\":"+array(offset)+
      ",\"status\":"+jsonString(offset_zero?"unsupported_or_omitted":offset_valid?"unit_offset_unapplied":"invalid_offset_unapplied")+",\"encoding_valid\":"+boolean(offset_valid)+",\"extension_bytes_observed\":"+boolean(message.magic!=MAVLINK_STX_MAVLINK1&&message.len>MAVLINK_MSG_ID_ATTITUDE_QUATERNION_MIN_LEN)+",\"display_adjustment_applied\":false}}",valid&&finite(rates)&&offset_valid);
  }
  Services& m_services;Sample m_euler,m_quaternion;
  bool m_bound=false,m_clock_fault=false;unsigned m_system=0,m_component=0;
  double m_boundary=0,m_last_now=0,m_heartbeat_at=-1,m_heartbeat_read=-1,m_timeout=1,m_interval=.1,m_next_publish=-1;
  uint64_t m_session=1,m_observation=0,m_metadata_rejected=0,m_source_rejected=0,m_wire_rejected=0,m_order_rejected=0;
  std::string m_reset_reason="startup";
};
}
}
extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi,mavmoos::runtime::Services* services)
{return abi==mavmoos::runtime::plugin_abi&&services?new mavmoos::runtime::Attitude(*services):nullptr;}
extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin){delete plugin;}
