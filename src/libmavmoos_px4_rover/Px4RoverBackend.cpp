#include "Px4RoverBackend.h"
#include "Px4Offboard.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string_view>

namespace mavmoos::runtime {
namespace {
constexpr unsigned mission=(4u<<16)|(4u<<24);
constexpr double max_speed=1.5, max_course_error=70., max_radius_m=100.;
constexpr double gps_age=1., mode_age=1.6, config_age=30., config_refresh=10.;
constexpr double stop_speed_m_s=.25, stop_wall_s=1.5, stop_excursion_m=1.;
constexpr uint64_t stop_firmware_us=1500000, gps_max_gap_us=450000;
constexpr std::size_t stop_sample_count=5, max_stop_samples=64;
constexpr unsigned max_reads=3;
constexpr std::array<std::string_view,4> profile_parameters={
  "SYS_AUTOSTART","MAV_FWDEXTSP","COM_OBL_RC_ACT","COM_OF_LOSS_T"
};

bool validPosition(double lat,double lon)
{
  return std::isfinite(lat)&&std::isfinite(lon)&&std::abs(lat)<89&&std::abs(lon)<=180;
}

double separation(double lat1,double lon1,double lat2,double lon2)
{
  const double north=lat2-lat1;
  const double east=std::remainder(lon2-lon1,360.)*std::cos((lat1+lat2)*.5*px4::radians);
  return std::hypot(north,east)*px4::metres_per_degree;
}

bool zeros(const CommandRequest& request,std::size_t first)
{
  return std::all_of(request.params.begin()+first,request.params.end(),[](float value){return value==0;});
}

bool modeRequest(const CommandRequest& request,float main,float sub)
{
  return request.command==MAV_CMD_DO_SET_MODE && request.params[0]==MAV_MODE_FLAG_CUSTOM_MODE_ENABLED &&
    request.params[1]==main && request.params[2]==sub && zeros(request,3);
}

bool profileValue(const mavlink_param_value_t& packet,std::size_t index)
{
  if(index==3) {
    return packet.param_type==MAV_PARAM_TYPE_REAL32 && std::isfinite(packet.param_value) &&
      packet.param_value>=.5f && packet.param_value<=1.5f;
  }
  // PX4 uses MAVLink bytewise integer encoding for onboard parameters.
  if(packet.param_type!=MAV_PARAM_TYPE_INT32) return false;
  int32_t value=0;
  std::memcpy(&value,&packet.param_value,sizeof(value));
  constexpr std::array<int32_t,3> expected={10045,1,5};
  return value==expected[index];
}
} // namespace

std::string Px4RoverBackend::profile() const
{ return "px4_rover_ackermann"; }

HelmProfile Px4RoverBackend::helmProfile() const
{ return {NavigationDimensions::Planar,false,true}; }

bool Px4RoverBackend::identityCompatible(unsigned type,unsigned autopilot) const
{ return type==MAV_TYPE_GROUND_ROVER && autopilot==MAV_AUTOPILOT_PX4; }

bool Px4RoverBackend::connected(const Vehicle& vehicle,double now) const
{
  return identityCompatible(vehicle.type,vehicle.autopilot) && vehicle.heartbeat_valid &&
    px4::fresh(vehicle.heartbeat_at,now,px4::rover_heartbeat_age);
}

bool Px4RoverBackend::gpsFresh(double now) const
{ return m_gps_valid && px4::fresh(m_gps_at,now,gps_age); }

bool Px4RoverBackend::verifiedConfig(double now) const
{
  return !m_conflict && std::all_of(m_config_at.begin(),m_config_at.end(),[now](double at) {
    return px4::fresh(at,now,config_age);
  });
}

bool Px4RoverBackend::stopped(double now,double after) const
{
  if(!gpsFresh(now)||!std::isfinite(after)||m_slow_samples.size()<stop_sample_count) return false;
  const auto& last=m_slow_samples.back();
  // Prefer the shortest sufficient suffix. An old displacement must not keep a
  // newly stationary rover blocked, and older samples cannot repair a bad suffix.
  for(std::size_t count=stop_sample_count;count<=m_slow_samples.size();++count) {
    const std::size_t begin=m_slow_samples.size()-count;
    const auto& first=m_slow_samples[begin];
    if(first.at<=after) return false;
    if(last.at-first.at<stop_wall_s || last.firmware_usec-first.firmware_usec<stop_firmware_us) continue;
    for(std::size_t i=begin;i<m_slow_samples.size();++i) {
      const auto& a=m_slow_samples[i];
      for(std::size_t j=i+1;j<m_slow_samples.size();++j) {
        const auto& b=m_slow_samples[j];
        if(separation(a.lat*1e-7,a.lon*1e-7,b.lat*1e-7,b.lon*1e-7)>stop_excursion_m) return false;
      }
    }
    return true;
  }
  return false;
}

bool Px4RoverBackend::holdObserved(double now,double after) const
{
  return m_mode_at>after && px4::fresh(m_mode_at,now,mode_age) &&
    m_active_mode==px4::hold && m_intended_mode==px4::hold;
}

bool Px4RoverBackend::missionObserved(double now,double after) const
{
  return m_mode_at>after && px4::fresh(m_mode_at,now,mode_age) &&
    m_active_mode==mission && m_intended_mode==mission;
}

bool Px4RoverBackend::groundStateReady(const Vehicle& vehicle,double now) const
{
  // The pinned Rover firmware reports IN_AIR while the armed ground rover
  // moves. Model readback and heartbeat identity establish the chassis here.
  return connected(vehicle,now) && vehicle.armed && gpsFresh(now) &&
    vehicle.position_valid && px4::fresh(vehicle.position_at,now,px4::position_age) &&
    vehicle.estimator_horizontal_valid && px4::fresh(vehicle.estimator_at,now,px4::estimator_age) &&
    vehicle.home_valid && px4::fresh(vehicle.home_at,now,8) &&
    validPosition(vehicle.latitude,vehicle.longitude) &&
    validPosition(vehicle.home_latitude,vehicle.home_longitude) &&
    separation(vehicle.home_latitude,vehicle.home_longitude,vehicle.latitude,vehicle.longitude)<=max_radius_m &&
    std::isfinite(vehicle.heading) && vehicle.heading>=0 && vehicle.heading<360;
}

bool Px4RoverBackend::missionReady(const Vehicle& vehicle,double now) const
{ return verifiedConfig(now) && groundStateReady(vehicle,now); }

bool Px4RoverBackend::activeModeAllowed(const Vehicle& vehicle,bool acquisition_grace) const
{
  if(acquisition_grace) return vehicle.mode==px4::offboard || vehicle.mode==px4::hold;
  return vehicle.mode==px4::offboard && m_active_mode==px4::offboard && m_intended_mode==px4::offboard;
}

bool Px4RoverBackend::flightReady(const Vehicle& vehicle,double now,bool motion) const
{
  return missionReady(vehicle,now) && activeModeAllowed(vehicle,!motion) &&
    (!motion || px4::fresh(m_mode_at,now,mode_age));
}

bool Px4RoverBackend::acquisitionAllowed(const Vehicle& vehicle,double now) const
{
  return vehicle.mode==px4::hold && flightReady(vehicle,now,false) &&
    holdObserved(now,-1) && stopped(now,-1);
}

std::optional<CommandRequest> Px4RoverBackend::acquisitionCommand(const Vehicle& vehicle,
                                                                 const std::string& id,double now) const
{
  if(!acquisitionAllowed(vehicle,now)) return std::nullopt;
  CommandRequest request;
  request.id="acquire-"+id;
  request.command=MAV_CMD_DO_SET_MODE;
  request.params[0]=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  request.params[1]=6;
  request.expires=now+.3;
  return request;
}

bool Px4RoverBackend::commandAllowed(const CommandRequest& request,const Vehicle& vehicle,double now) const
{ return modeRequest(request,6,0) && acquisitionAllowed(vehicle,now); }

bool Px4RoverBackend::missionExecutionCommandAllowed(const CommandRequest& request,const Vehicle& vehicle,double now) const
{
  if(modeRequest(request,4,3)) {
    // An existing mission lease may still attempt its bounded stop after lost
    // navigation evidence. A current external mode always takes precedence.
    return connected(vehicle,now) && vehicle.armed &&
      (vehicle.mode==mission || (vehicle.mode==px4::hold && px4::fresh(m_mode_at,now,mode_age) &&
                                m_active_mode==px4::hold && m_intended_mode==mission));
  }
  if(!request.abort_id.empty() || !acquisitionAllowed(vehicle,now)) return false;
  if(modeRequest(request,4,4)) return true;
  const float last=request.params[1];
  return request.command==MAV_CMD_MISSION_START && request.params[0]==0 &&
    last>=1 && last<=7 && std::trunc(last)==last && zeros(request,2);
}

bool Px4RoverBackend::navigationInputAllowed(const NavigationTarget& target) const
{
  return target.kind==NavigationKind::CourseSpeed && target.dimensions==NavigationDimensions::Planar &&
    target.facing==FacingPolicy::Course && helmFieldAllowed(0,target.course) && helmFieldAllowed(1,target.speed);
}

bool Px4RoverBackend::helmFieldAllowed(unsigned index,double value) const
{
  return std::isfinite(value) && (index==0 ? std::abs(value)<=360 : index==1 && value>=0 && value<=max_speed);
}

bool Px4RoverBackend::helmVehicleReady(const Vehicle& vehicle,double now,double position_timeout,
                                     bool shadow,bool acquisition) const
{
  const bool ready=shadow?groundStateReady(vehicle,now):flightReady(vehicle,now,!acquisition);
  return ready && px4::fresh(vehicle.position_at,now,position_timeout);
}

bool Px4RoverBackend::targetAllowed(const NavigationTarget& target,const Vehicle& vehicle,double now) const
{
  return navigationInputAllowed(target) && std::isfinite(target.expires) && now<target.expires &&
    flightReady(vehicle,now,true) && (target.speed==0 ||
      std::abs(std::remainder(target.course-vehicle.heading,360.))<=max_course_error);
}

std::optional<ControlFrame> Px4RoverBackend::navigationFrame(const NavigationTarget& target,const Vehicle& vehicle,
                                                            double now,uint64_t,mavlink_status_t& codec) const
{
  if(!targetAllowed(target,vehicle,now) || now<0 || !std::isfinite(now*1000.)) return std::nullopt;
  if(target.speed==0) return px4::velocityFrame(0,0,now,codec);
  const double angle=target.course*px4::radians;
  return px4::velocityFrame(static_cast<float>(target.speed*std::cos(angle)),
                            static_cast<float>(target.speed*std::sin(angle)),now,codec);
}

std::optional<ControlFrame> Px4RoverBackend::stopFrame(const Vehicle&,double,uint64_t,mavlink_status_t&) const
{ return std::nullopt; } // Withdrawing the stream hands the stop to PX4's verified loss policy.

void Px4RoverBackend::observeGps(const mavlink_message_t& message,double at)
{
  if(at<m_gps_at) return;
  mavlink_gps_raw_int_t packet{};
  mavlink_msg_gps_raw_int_decode(&message,&packet);
  if(packet.time_usec>0 && packet.time_usec==m_gps_usec) return;
  if(packet.time_usec==0 || packet.time_usec<m_gps_usec) {
    m_gps_at=at;
    m_gps_valid=false;
    m_slow_samples.clear();
    return;
  }
  const bool gap=m_gps_at>=0 && (at-m_gps_at>gps_age || packet.time_usec-m_gps_usec>gps_max_gap_us);
  m_gps_at=at;
  m_gps_usec=packet.time_usec;
  m_gps_valid=packet.fix_type>=GPS_FIX_TYPE_3D_FIX && packet.fix_type<=GPS_FIX_TYPE_PPP &&
    packet.satellites_visible>=6 && packet.satellites_visible<UINT8_MAX && packet.vel!=UINT16_MAX &&
    packet.h_acc>0 && packet.h_acc<=1500 && packet.vel_acc>0 && packet.vel_acc<=450 &&
    validPosition(packet.lat*1e-7,packet.lon*1e-7);
  const bool slow=m_gps_valid && packet.vel*.01<=stop_speed_m_s;
  if(gap||!slow) m_slow_samples.clear();
  if(!slow) return;
  if(m_slow_samples.size()==max_stop_samples) m_slow_samples.pop_front();
  m_slow_samples.push_back({at,packet.time_usec,packet.lat,packet.lon});
}

void Px4RoverBackend::observeParameter(const mavlink_message_t& message,const MessageTime& time)
{
  if(message.len!=MAVLINK_MSG_ID_PARAM_VALUE_LEN) return;
  mavlink_param_value_t packet{};
  mavlink_msg_param_value_decode(&message,&packet);
  const std::string_view name(packet.param_id,std::find(packet.param_id,packet.param_id+16,'\0')-packet.param_id);
  const auto found=std::find(profile_parameters.begin(),profile_parameters.end(),name);
  if(found==profile_parameters.end()) return;
  const std::size_t index=static_cast<std::size_t>(found-profile_parameters.begin());
  const bool matched_read=m_read && m_read->index==index && m_read->sent_at>=0 &&
    time.read_at>m_read->sent_at && time.read_at<m_read->request.expires;
  const bool matches=profileValue(packet,index);
  // A matching but unsolicited value cannot grant readiness. A later conflict
  // for an already confirmed setting retires the profile in this session.
  if(!matches && (matched_read || (m_config_at[index]>=0 && time.effective_at>=m_config_at[index]))) {
    m_conflict=true;
    m_read.reset();
    return;
  }
  if(m_conflict || !matched_read || !matches || packet.param_count==0 || packet.param_index>=packet.param_count) return;
  m_config_at[index]=time.effective_at;
  m_attempts[index]=0;
  m_read.reset();
}

void Px4RoverBackend::observe(const mavlink_message_t& message,const MessageTime& time,const Vehicle& vehicle,double now)
{
  if(message.sysid!=1 || message.compid!=1 || message.magic!=MAVLINK_STX || !connected(vehicle,now) ||
     !std::isfinite(time.read_at) || time.read_at<time.effective_at || time.read_at>now ||
     time.effective_at<m_observation_floor || time.effective_at<vehicle.heartbeat_at) return;
  if(message.msgid==MAVLINK_MSG_ID_GPS_RAW_INT && px4::fresh(time.effective_at,now,gps_age)) {
    observeGps(message,time.effective_at);
  } else if(message.msgid==MAVLINK_MSG_ID_CURRENT_MODE && time.effective_at>=m_mode_at &&
            px4::fresh(time.effective_at,now,mode_age)) {
    mavlink_current_mode_t packet{};
    mavlink_msg_current_mode_decode(&message,&packet);
    m_active_mode=packet.custom_mode;
    m_intended_mode=packet.intended_custom_mode;
    m_mode_at=time.effective_at;
  } else if(message.msgid==MAVLINK_MSG_ID_PARAM_VALUE && px4::fresh(time.effective_at,now,2)) {
    observeParameter(message,time);
  }
}

void Px4RoverBackend::readConfiguration(Services& services)
{
  const double now=services.now();
  if(!connected(services.vehicle(),now) || m_conflict || now<m_next_read) return;
  if(m_read && now<m_read->request.expires) return;
  m_read.reset();
  const auto oldest=std::min_element(m_config_at.begin(),m_config_at.end());
  if(px4::fresh(*oldest,now,config_refresh)) return;
  const auto index=static_cast<std::size_t>(oldest-m_config_at.begin());
  if(m_attempts[index]>=max_reads) return;
  ParameterRequest request;
  request.id="px4-rover-profile/"+std::to_string(++m_read_sequence);
  request.name=profile_parameters[index];
  request.action=ParameterAction::Read;
  request.expires=now+1.6;
  m_next_read=now+.25;
  if(services.parameter(request)) {
    ++m_attempts[index];
    m_read=ConfigurationRead{request,index};
  }
}

void Px4RoverBackend::parameterSent(const ParameterRequest& request,double sent_at,bool accepted)
{
  if(!m_read || request.id!=m_read->request.id || request.name!=m_read->request.name ||
     request.action!=ParameterAction::Read || request.index!=-1 || request.expires!=m_read->request.expires) return;
  if(accepted && std::isfinite(sent_at) && sent_at>=m_observation_floor && sent_at<request.expires) {
    m_read->sent_at=sent_at;
  } else m_read.reset();
}

void Px4RoverBackend::reset(double now)
{
  if(std::isfinite(now)) m_observation_floor=std::max(m_observation_floor,now);
  m_gps_valid=false;
  m_gps_at=-1;
  m_gps_usec=0;
  m_slow_samples.clear();
  m_mode_at=-1;
  m_active_mode=m_intended_mode=0;
  m_config_at.fill(-1);
  m_attempts.fill(0);
  m_next_read=0;
  m_conflict=false;
  m_read.reset();
}
} // namespace mavmoos::runtime
