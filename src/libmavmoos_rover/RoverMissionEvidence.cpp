#include "RoverMissionEvidence.h"
#include "RoverComponents.h"
#include <limits>
#include <string_view>

namespace mavmoos::runtime {
namespace {
using ardu_rover_mission::fresh;
using ardu_rover_mission::separation;
constexpr std::array<std::string_view,6> parameters={
  "MIS_RESTART","MIS_DONE_BEHAVE","WP_SPEED","WP_RADIUS","AUTO_KICKSTART","AUTO_TRIGGER_PIN"
};

bool profileValue(std::size_t index,const mavlink_param_value_t& value,bool boat)
{
  const auto p=value.param_value;
  if(!std::isfinite(p)) return false;
  switch(index) {
  case 0: return value.param_type==MAV_PARAM_TYPE_INT8 && p==0;
  case 1: return value.param_type==MAV_PARAM_TYPE_INT8 && p==(boat?1:0);
  case 2: return value.param_type==MAV_PARAM_TYPE_REAL32 && p>0 && p<=rover::speedLimit(boat);
  case 3: return value.param_type==MAV_PARAM_TYPE_REAL32 && p>0 && p<=2;
  case 4: return value.param_type==MAV_PARAM_TYPE_REAL32 && p==0;
  case 5: return value.param_type==MAV_PARAM_TYPE_INT8 && p==-1;
  default: return false;
  }
}
} // namespace

bool RoverMissionEvidence::configured(double now) const
{
  return !m_conflict && std::all_of(m_verified.begin(),m_verified.end(),[now](double at) {
    return fresh(at,now,30);
  });
}

bool RoverMissionEvidence::settled(double now,double after) const
{
  if(m_samples.size()<5 || !fresh(m_samples.back().at,now,.5)) return false;
  const auto& last=m_samples.back();
  for(std::size_t count=5;count<=m_samples.size();++count) {
    const auto first=m_samples.end()-static_cast<std::ptrdiff_t>(count);
    if(first->at<=after) return false;
    if(last.at-first->at<1 || last.boot_ms-first->boot_ms<1000) continue;
    const auto radius=m_boat?1.5:.6;
    return std::all_of(first,m_samples.end(),[&](const auto& sample) {
      return separation(first->latitude,first->longitude,sample.latitude,sample.longitude)<=radius;
    });
  }
  return false;
}

void RoverMissionEvidence::observePosition(const mavlink_message_t& message,double at)
{
  if(at<=m_last_at) return;
  mavlink_global_position_int_t packet{};
  mavlink_msg_global_position_int_decode(&message,&packet);
  if(m_boot_ms && packet.time_boot_ms==*m_boot_ms) return;
  const bool discontinuity=m_boot_ms &&
    (int32_t(packet.time_boot_ms-*m_boot_ms)<=0 || packet.time_boot_ms-*m_boot_ms>500 || at-m_last_at>.5);
  if(m_boot_ms && int32_t(packet.time_boot_ms-*m_boot_ms)<=0) {
    m_samples.clear();
    m_conflict=true;
    return;
  }
  m_boot_ms=packet.time_boot_ms;
  m_last_at=at;
  const double latitude=packet.lat*1e-7,longitude=packet.lon*1e-7;
  const bool slow=std::abs(latitude)<85 && std::abs(longitude)<=180 &&
    std::hypot(packet.vx,packet.vy)<=100*(m_boat?.45:.25);
  if(discontinuity || !slow) m_samples.clear();
  if(!slow) return;
  if(m_samples.size()>=80) m_samples.pop_front();
  m_samples.push_back({at,latitude,longitude,packet.time_boot_ms});
}

void RoverMissionEvidence::observeParameter(const mavlink_message_t& message,const MessageTime& time)
{
  if(message.len!=MAVLINK_MSG_ID_PARAM_VALUE_LEN) return;
  mavlink_param_value_t packet{};
  mavlink_msg_param_value_decode(&message,&packet);
  const std::string_view name(packet.param_id,std::find(packet.param_id,packet.param_id+16,'\0')-packet.param_id);
  const auto parameter=std::find(parameters.begin(),parameters.end(),name);
  if(parameter==parameters.end()) return;
  const auto index=static_cast<std::size_t>(parameter-parameters.begin());
  const bool receipt=m_read && m_read->index==index && m_read->sent_at>=0 &&
    time.read_at>m_read->sent_at && time.read_at<m_read->request.expires;
  const bool valid=profileValue(index,packet,m_boat);
  if(!valid && (receipt || (m_verified[index]>0 && time.effective_at>=m_verified[index]))) {
    m_conflict=true;
    m_read.reset();
    return;
  }
  // ArduPilot echoes the named read's signed -1 in this unsigned wire field.
  const bool index_matches=packet.param_index==std::numeric_limits<uint16_t>::max() ||
    packet.param_index<packet.param_count;
  if(m_conflict || !receipt || !valid || packet.param_count==0 || !index_matches) return;
  m_verified[index]=time.effective_at;
  m_read.reset();
}

void RoverMissionEvidence::observe(const mavlink_message_t& message,const MessageTime& time,double now)
{
  if(message.magic!=MAVLINK_STX || message.sysid!=1 || message.compid!=1 ||
     time.effective_at<=m_boundary || !fresh(time.effective_at,now,2) ||
     !std::isfinite(time.read_at) || time.read_at<time.effective_at || time.read_at>now ||
     (time.age_basis!="READ_ACQUISITION" && time.age_basis!="QUEUE_EMPTY_FLOOR")) return;
  if(message.msgid==MAVLINK_MSG_ID_GLOBAL_POSITION_INT && fresh(time.effective_at,now,.5))
    observePosition(message,time.effective_at);
  if(message.msgid==MAVLINK_MSG_ID_PARAM_VALUE) observeParameter(message,time);
}

void RoverMissionEvidence::readConfiguration(Services& services)
{
  const auto now=services.now();
  const auto& vehicle=services.vehicle();
  if(!vehicle.heartbeat_valid || !fresh(vehicle.heartbeat_at,now,2) ||
     m_conflict || now<m_next_read || (m_read && now<m_read->request.expires)) return;
  m_read.reset();
  const auto oldest=std::min_element(m_verified.begin(),m_verified.end());
  if(fresh(*oldest,now,10)) return;
  const auto index=static_cast<std::size_t>(oldest-m_verified.begin());
  ParameterRequest request;
  request.id="ardu-rover-mission-profile/"+std::to_string(++m_read_sequence);
  request.name=parameters[index];
  request.expires=now+1.6;
  m_next_read=now+.3;
  if(services.parameter(request)) m_read=Read{request,index};
}

void RoverMissionEvidence::parameterSent(const ParameterRequest& request,double at,bool accepted)
{
  if(!m_read || request.id!=m_read->request.id || request.name!=m_read->request.name ||
     request.action!=ParameterAction::Read || request.index!=-1 || request.expires!=m_read->request.expires) return;
  if(accepted && std::isfinite(at) && at>=m_boundary && at<request.expires) m_read->sent_at=at;
  else m_read.reset();
}

void RoverMissionEvidence::reset(double now)
{
  if(std::isfinite(now)) m_boundary=std::max(m_boundary,now);
  m_boot_ms.reset();
  m_last_at=0;
  m_samples.clear();
  m_verified.fill(0);
  m_read.reset();
  m_next_read=0;
  m_conflict=false;
}

} // namespace mavmoos::runtime
