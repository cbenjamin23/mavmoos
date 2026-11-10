#include "ImuObservation.h"
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>

namespace mavmoos::runtime::imu {
namespace {
std::string number(double x){std::ostringstream out;out<<std::setprecision(17)<<x;return out.str();}
std::string boolean(bool x){return x?"true":"false";}
std::string raw(const std::array<int16_t,3>& v)
{return "["+std::to_string(v[0])+","+std::to_string(v[1])+","+std::to_string(v[2])+"]";}
std::string vector(const std::array<int16_t,3>& v,double scale,const char* unit)
{
  std::string out="{\"unit\":\""+std::string(unit)+"\",\"conversion_known\":true,\"value\":[";
  for(unsigned n=0;n<3;++n){if(n)out+=',';out+=number(v[n]*scale);}return out+"]}";
}
template<class T> void axes(Observation& out,const T& p)
{
  out.acceleration={p.xacc,p.yacc,p.zacc};out.angular_velocity={p.xgyro,p.ygyro,p.zgyro};
  out.magnetic_field={p.xmag,p.ymag,p.zmag};out.temperature=p.temperature;
}
}
bool shape(const mavlink_message_t& m)
{
  unsigned minimum=0,maximum=0;
  switch(m.msgid){
    case MAVLINK_MSG_ID_RAW_IMU:minimum=MAVLINK_MSG_ID_RAW_IMU_MIN_LEN;maximum=MAVLINK_MSG_ID_RAW_IMU_LEN;break;
    case MAVLINK_MSG_ID_SCALED_IMU:minimum=MAVLINK_MSG_ID_SCALED_IMU_MIN_LEN;maximum=MAVLINK_MSG_ID_SCALED_IMU_LEN;break;
    case MAVLINK_MSG_ID_SCALED_IMU2:minimum=MAVLINK_MSG_ID_SCALED_IMU2_MIN_LEN;maximum=MAVLINK_MSG_ID_SCALED_IMU2_LEN;break;
    case MAVLINK_MSG_ID_SCALED_IMU3:minimum=MAVLINK_MSG_ID_SCALED_IMU3_MIN_LEN;maximum=MAVLINK_MSG_ID_SCALED_IMU3_LEN;break;
    case MAVLINK_MSG_ID_VIBRATION:minimum=MAVLINK_MSG_ID_VIBRATION_MIN_LEN;maximum=MAVLINK_MSG_ID_VIBRATION_LEN;break;
    default:return false;
  }
  // MAVLink2 legitimately truncates trailing zero bytes, including base fields.
  if(m.magic==MAVLINK_STX)return m.len>0&&m.len<=maximum&&!(m.incompat_flags&~MAVLINK_IFLAG_SIGNED);
  return m.magic==MAVLINK_STX_MAVLINK1&&m.len==minimum;
}
std::optional<Observation> decode(const mavlink_message_t& m)
{
  if(!shape(m))return std::nullopt;
  Observation out;out.message_id=m.msgid;out.payload_length=m.len;out.mavlink2=m.magic==MAVLINK_STX;
  switch(m.msgid){
    case MAVLINK_MSG_ID_RAW_IMU:{
      mavlink_raw_imu_t p{};mavlink_msg_raw_imu_decode(&m,&p);axes(out,p);out.family="RAW_IMU";
      out.remote_stamp=p.time_usec;out.sensor_instance=p.id;out.raw_id_extension=out.mavlink2;break;
    }
    case MAVLINK_MSG_ID_SCALED_IMU:{
      mavlink_scaled_imu_t p{};mavlink_msg_scaled_imu_decode(&m,&p);axes(out,p);out.family="SCALED_IMU";
      out.remote_stamp=p.time_boot_ms;out.sensor_instance=0;break;
    }
    case MAVLINK_MSG_ID_SCALED_IMU2:{
      mavlink_scaled_imu2_t p{};mavlink_msg_scaled_imu2_decode(&m,&p);axes(out,p);out.family="SCALED_IMU2";
      out.remote_stamp=p.time_boot_ms;out.sensor_instance=1;break;
    }
    case MAVLINK_MSG_ID_SCALED_IMU3:{
      mavlink_scaled_imu3_t p{};mavlink_msg_scaled_imu3_decode(&m,&p);axes(out,p);out.family="SCALED_IMU3";
      out.remote_stamp=p.time_boot_ms;out.sensor_instance=2;break;
    }
    default:return std::nullopt;
  }
  return out;
}
std::string data(const Observation& p,bool regressed)
{
  const bool raw_family=p.message_id==MAVLINK_MSG_ID_RAW_IMU, temperature_known=p.mavlink2&&p.temperature!=0;
  const char* identity=raw_family?(p.raw_id_extension?(p.sensor_instance?"reported_extension_id":"zero_default_or_omitted_extension_id"):"mavlink1_no_id_extension") : "message_family_logical_instance";
  const char* temp_reason=!p.mavlink2?"mavlink1_extension_absent":p.temperature==0?"zero_unknown_or_omitted":p.temperature==1?"reported_0_01C_protocol_zero_degree_encoding_ambiguous":"reported_temperature";
  return "{\"message_id\":"+std::to_string(p.message_id)+",\"family\":\""+p.family+"\",\"wire_version\":"+std::to_string(p.mavlink2?2:1)+
    ",\"payload_length\":"+std::to_string(p.payload_length)+",\"sensor_instance\":"+std::to_string(p.sensor_instance)+",\"identity_basis\":\""+identity+"\",\"physical_sensor_identity\":\"unverified\""+
    ",\"remote_timestamp\":"+std::to_string(p.remote_stamp)+",\"remote_clock\":\""+(raw_family?"ardupilot_boot_us_unmapped_uint64":"boot_ms_unmapped_uint32")+"\",\"remote_timestamp_regressed\":"+boolean(regressed)+
    ",\"timestamp_semantics\":\"ardupilot_message_send_time_not_certified_measurement_time\",\"body_frame\":\"BODY_FRD\",\"estimate\":\"sensor_report_not_fused_attitude\""+
    ",\"unit_basis\":\""+(raw_family?"ardupilot_raw_imu_scaled_sender_convention":"mavlink_scaled_imu_units")+"\",\"raw_units\":{\"acceleration\":\"milli-g\",\"angular_velocity\":\"mrad/s\",\"magnetic_field\":\"milligauss\"},\"sensor_presence\":\"unknown\",\"sensor_health\":\"unassessed\",\"physical_accuracy\":\"unassessed\",\"covariance\":null,\"gravity_removed\":false"+
    ",\"raw_acceleration\":"+raw(p.acceleration)+",\"raw_angular_velocity\":"+raw(p.angular_velocity)+",\"raw_magnetic_field\":"+raw(p.magnetic_field)+
    ",\"acceleration\":"+vector(p.acceleration,9.80665/1000.,"m/s^2")+",\"angular_velocity\":"+vector(p.angular_velocity,.001,"rad/s")+
    ",\"magnetic_field\":"+vector(p.magnetic_field,1e-7,"T")+",\"temperature\":{\"raw_cdegC\":"+(p.mavlink2?std::to_string(p.temperature):"null")+
    ",\"known\":"+boolean(temperature_known)+",\"value\":"+(temperature_known?number(p.temperature*.01):"null")+",\"unit\":\"degC\",\"reason\":\""+temp_reason+"\"}}";
}

std::optional<VibrationReport> decodeVibration(const mavlink_message_t& message)
{
  if(message.msgid!=MAVLINK_MSG_ID_VIBRATION||!shape(message))return std::nullopt;
  mavlink_vibration_t packet{};
  mavlink_msg_vibration_decode(&message,&packet);
  VibrationReport result;
  result.time_usec=packet.time_usec;
  result.levels={packet.vibration_x,packet.vibration_y,packet.vibration_z};
  result.clipping={packet.clipping_0,packet.clipping_1,packet.clipping_2};
  result.payload_length=message.len;
  result.mavlink2=message.magic==MAVLINK_STX;
  for(const float level:result.levels)
    if(!std::isfinite(level)||level<0)return std::nullopt;
  return result;
}

std::string data(const VibrationReport& report,bool regressed)
{
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out<<std::setprecision(17)<<"{\"message_id\":"<<MAVLINK_MSG_ID_VIBRATION
     <<",\"family\":\"VIBRATION\",\"wire_version\":"<<(report.mavlink2?2:1)
     <<",\"payload_length\":"<<report.payload_length<<",\"time_usec_raw\":\""<<report.time_usec<<'"'
     <<",\"remote_clock\":\"ardupilot_boot_us_unmapped_uint64\",\"remote_timestamp_regressed\":"<<boolean(regressed)
     <<",\"timestamp_semantics\":\"ardupilot_message_send_time_not_certified_measurement_time\""
     <<",\"vibration_m_s2\":["<<report.levels[0]<<','<<report.levels[1]<<','<<report.levels[2]<<']'
     <<",\"vibration_basis\":\"ardupilot_first_usable_accelerometer_filtered_residual\""
     <<",\"clipping_counts\":["<<report.clipping[0]<<','<<report.clipping[1]<<','<<report.clipping[2]<<']'
     <<",\"clipping_sensor_presence\":\"unknown\",\"sensor_health\":\"unassessed\"}";
  return out.str();
}
} // namespace mavmoos::runtime::imu
