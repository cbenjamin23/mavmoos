#include "ControlComponents.h"
#include "CopterModel.h"
#include "CopterMissionPolicy.h"
#include <cmath>
#include <limits>
#include <algorithm>

namespace mavmoos::runtime {
namespace {
bool fresh(double acquired,double now,double limit)
{
  return acquired>0 && now>=acquired && now-acquired<=limit;
}
bool helmFresh(double acquired,double now,double limit)
{
  return std::isfinite(acquired) && now>=acquired && now-acquired<=limit;
}
egress::Tick ns(double value)
{
  return static_cast<egress::Tick>(value*egress::second);
}
bool frameTime(double value)
{
  // The host supplies a valid monotonic clock. Reject invalid direct calls
  // before conversion to the P2 integer time representation.
  return std::isfinite(value) && value>=0 &&
    value<double(std::numeric_limits<egress::Tick>::max())/egress::second-4;
}
constexpr double pi=3.14159265358979323846;
double radians(double degrees) { return degrees*pi/180.; }
double heading(double degrees) { return std::fmod(std::fmod(degrees,360.)+360.,360.); }
std::optional<ControlFrame> positionFrame(const NavigationTarget& target,const Vehicle& vehicle,
  double now,mavlink_status_t& codec)
{
  mavlink_set_position_target_global_int_t value{};
  value.time_boot_ms=static_cast<uint32_t>(std::fmod(now*1000.,4294967296.));
  value.target_system=1;value.target_component=1;
  value.coordinate_frame=MAV_FRAME_GLOBAL_RELATIVE_ALT;
  value.type_mask=3576;
  value.lat_int=static_cast<int32_t>(std::llround(target.latitude*1e7));
  value.lon_int=static_cast<int32_t>(std::llround(target.longitude*1e7));
  value.alt=static_cast<float>(target.height);
  if(target.facing==FacingPolicy::Heading) {
    value.type_mask=static_cast<uint16_t>(value.type_mask&~1024u);
    value.yaw=static_cast<float>(radians(heading(target.facing_heading)));
  } else if(target.facing==FacingPolicy::Course) {
    const double start=radians(vehicle.latitude),goal=radians(target.latitude);
    const double delta=radians(std::remainder(target.longitude-vehicle.longitude,360.));
    const double hav=std::pow(std::sin((goal-start)/2),2)+std::cos(start)*std::cos(goal)*std::pow(std::sin(delta/2),2);
    if(12742000.*std::asin(std::sqrt(std::clamp(hav,0.,1.)))>.05) {
      value.type_mask=static_cast<uint16_t>(value.type_mask&~1024u);
      value.yaw=static_cast<float>(std::atan2(std::sin(delta)*std::cos(goal),
        std::cos(start)*std::sin(goal)-std::sin(start)*std::cos(goal)*std::cos(delta)));
    }
  }
  mavlink_message_t message{};
  mavlink_msg_set_position_target_global_int_encode_status(245,191,&codec,&message,&value);
  ControlFrame result;result.size=mavlink_msg_to_send_buffer(result.bytes.data(),&message);
  return result;
}
egress::Window window(double at,double life)
{
  return {static_cast<uint64_t>(at*1e6)+1,ns(at),ns(at+life)};
}
egress::VehicleState observations(const Vehicle& vehicle,uint64_t revision)
{
  egress::VehicleState state;
  state.revision=revision+1;state.home_revision=1;state.home_known=vehicle.home_valid;
  state.flight={egress::Flight{vehicle.mode==4,vehicle.armed},window(vehicle.heartbeat_at,2)};
  state.landed={vehicle.in_air?egress::Landed::InAir:egress::Landed::OnGround,window(vehicle.landed_at,2)};
  state.pose={egress::Pose{vehicle.relative_height,vehicle.north_velocity,vehicle.east_velocity,vehicle.down_velocity},window(vehicle.position_at,.5)};
  state.global={egress::GlobalPosition{vehicle.latitude,vehicle.longitude,vehicle.msl_height},window(vehicle.position_at,.5)};
  state.estimator_eligible={vehicle.estimator_valid,window(vehicle.estimator_at,3)};
  return state;
}
std::optional<ControlFrame> encoded(const egress::Build& built,double now,mavlink_status_t& codec)
{
  if(built.evaluation.reason!=egress::Reason::Ok)return std::nullopt;
  const auto frame=egress::encode(built.intent,{1,1,245,191,1,1},
    static_cast<uint32_t>(std::fmod(now*1000.,4294967296.)),codec);
  return ControlFrame{frame.bytes,frame.size};
}
class ArduCopterBackend final:public ArduCopterControlBackend {
public:
  std::string profile() const override {return "arducopter";}
  void bindFrameType(unsigned type) override {m_frame_type=type;}
  bool identityCompatible(unsigned type,unsigned autopilot) const override
  {
    return type==m_frame_type && autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA;
  }
  bool flightReady(const Vehicle& v,double now,bool motion) const override
  {
    return v.heartbeat_valid && fresh(v.heartbeat_at,now,2) &&
      v.position_valid && fresh(v.position_at,now,.5) &&
      v.landed_valid && fresh(v.landed_at,now,2) &&
      v.estimator_valid && fresh(v.estimator_at,now,3) &&
      v.home_valid && (!motion || (v.armed && v.in_air && v.mode==4));
  }
  bool acquisitionAllowed(const Vehicle& v,double now) const override
  {
    return flightReady(v,now,false) && v.armed && v.in_air && (v.mode==4 || v.mode==5);
  }
  bool activeModeAllowed(const Vehicle& v,bool acquisition_grace) const override
  {
    return v.mode==4 || (v.mode==5 && acquisition_grace);
  }
  std::optional<CommandRequest> acquisitionCommand(const Vehicle& v,const std::string& id,double now) const override
  {
    if(v.mode!=5)return std::nullopt;
    CommandRequest request;request.id="acquire-"+id;request.command=MAV_CMD_DO_SET_MODE;
    request.params[0]=1;request.params[1]=4;request.expires=now+5;
    return request;
  }
  bool commandAllowed(const CommandRequest& r,const Vehicle& v,double now) const override
  {
    // Generic ID, finite-parameter, heartbeat, deadline and capacity checks are
    // still independently performed by the host before this Copter policy.
    const auto& p=r.params;
    if(r.command==MAV_CMD_DO_CHANGE_SPEED)
      return flightReady(v,now,true) &&
        p[0]==1 && p[1]>=.1f && p[1]<=5 && p[2]==-1 &&
        std::all_of(p.begin()+3,p.end(),[](float value){return value==0;});
    if(r.command==MAV_CMD_CONDITION_YAW)
      return flightReady(v,now,true) && p[0]>=0 && p[0]<=360 && p[1]==30 &&
        std::all_of(p.begin()+2,p.end(),[](float value){return value==0;});
    if(r.command==MAV_CMD_DO_AUX_FUNCTION)
      return flightReady(v,now,false) &&
        v.armed && v.in_air && v.mode==5 && p[0]==39 && (p[1]==0||p[1]==2) &&
        std::all_of(p.begin()+2,p.end(),[](float value){return value==0;});
    if(r.command==MAV_CMD_NAV_RETURN_TO_LAUNCH &&
       (!flightReady(v,now,false)||!v.armed||!v.in_air||(v.mode!=4&&v.mode!=5))) return false;
    if(r.command==MAV_CMD_NAV_LAND && (!v.armed||(v.mode!=3&&v.mode!=4&&v.mode!=5))) return false;
    if(r.command!=MAV_CMD_COMPONENT_ARM_DISARM && r.command!=MAV_CMD_NAV_TAKEOFF &&
       r.command!=MAV_CMD_NAV_LAND && r.command!=MAV_CMD_NAV_RETURN_TO_LAUNCH &&
       r.command!=MAV_CMD_DO_SET_MODE)return false;
    if(r.command==MAV_CMD_COMPONENT_ARM_DISARM && (!v.landed_valid ||
       !fresh(v.landed_at,now,2) || v.landed_state!=MAV_LANDED_STATE_ON_GROUND || r.params[1]!=0))return false;
    if(r.command==MAV_CMD_COMPONENT_ARM_DISARM && r.params[0]!=0 && r.params[0]!=1)return false;
    if(r.command==MAV_CMD_COMPONENT_ARM_DISARM && r.params[0]==1 &&
       (!v.estimator_valid || !fresh(v.estimator_at,now,3)))return false;
    if(r.command==MAV_CMD_NAV_TAKEOFF && (!flightReady(v,now,false) || !v.armed || v.mode!=4 ||
       v.landed_state!=MAV_LANDED_STATE_ON_GROUND || r.params[6]<1 || r.params[6]>20))return false;
    if(r.command==MAV_CMD_DO_SET_MODE && (r.params[0]!=1 || (r.params[1]!=4 && r.params[1]!=5)))return false;
    return true;
  }
  bool missionExecutionCommandAllowed(const CommandRequest& r,const Vehicle& v,double now) const override
  {
    if(r.abort_id.size()) {
      return r.command==MAV_CMD_DO_SET_MODE && v.mode==3 && v.heartbeat_valid && fresh(v.heartbeat_at,now,2) &&
        r.params[0]==1 && r.params[1]==4 &&
        std::all_of(r.params.begin()+2,r.params.end(),[](float value){return value==0;});
    }
    if(!flightReady(v,now,false) || !v.armed) return false;
    const bool start_state=v.mode==4&&(v.in_air||copter_mission::ground(v));
    if(r.command==MAV_CMD_DO_SET_MISSION_CURRENT)
      return start_state && r.params[0]==1 &&
        std::all_of(r.params.begin()+1,r.params.end(),[](float value){return value==0;});
    if(r.command==MAV_CMD_MISSION_START)
      return start_state && std::all_of(r.params.begin(),r.params.end(),[](float value){return value==0;});
    if(r.command==MAV_CMD_DO_PAUSE_CONTINUE)
      return v.mode==3 && v.in_air && (r.params[0]==0||r.params[0]==1) &&
        std::all_of(r.params.begin()+1,r.params.end(),[](float value){return value==0;});
    return false;
  }
  bool navigationInputAllowed(const NavigationTarget& target) const override
  {
    if(target.dimensions!=NavigationDimensions::Spatial ||
       (target.facing!=FacingPolicy::Course&&target.facing!=FacingPolicy::Keep&&target.facing!=FacingPolicy::Heading)||
       (target.facing==FacingPolicy::Heading&&!std::isfinite(target.facing_heading))||
       !std::isfinite(target.height)||target.height<1||target.height>50)return false;
    if(target.kind==NavigationKind::GlobalPosition)
      return std::isfinite(target.latitude)&&std::abs(target.latitude)<=90&&
        std::isfinite(target.longitude)&&std::abs(target.longitude)<=180;
    return target.kind==NavigationKind::CourseSpeed&&std::isfinite(target.course)&&
      std::isfinite(target.speed)&&target.speed>=0&&target.speed<=5;
  }
  bool helmFieldAllowed(unsigned index,double value) const override
  {
    if(!std::isfinite(value))return false;
    if(index==0)return true;
    if(index==1)return value>=0 && value<=5;
    return index==2 && value>=1 && value<=50;
  }
  bool helmVehicleReady(const Vehicle& v,double now,double position_timeout,bool shadow,bool allow_acquisition) const override
  {
    return v.heartbeat_valid && v.position_valid && v.landed_valid && v.armed && v.in_air &&
      helmFresh(v.heartbeat_at,now,2) && helmFresh(v.position_at,now,position_timeout) &&
      helmFresh(v.landed_at,now,2) && (shadow || activeModeAllowed(v,allow_acquisition));
  }
  std::optional<ControlFrame> navigationFrame(const NavigationTarget& target,const Vehicle& vehicle,
    double now,uint64_t revision,mavlink_status_t& codec) const override
  {
    if(!navigationInputAllowed(target) || !frameTime(now) || !frameTime(target.expires) ||
       target.expires<=now || !flightReady(vehicle,now,true))return std::nullopt;
    if(target.kind==NavigationKind::GlobalPosition)return positionFrame(target,vehicle,now,codec);
    egress::NavInput input;input.revision=revision+1;input.home_revision=1;
    const egress::Window intent_window{revision+1,ns(now-.000001),ns(target.expires)};
    input.course_deg={heading(target.course),intent_window};
    input.speed_mps={target.speed,intent_window};input.relative_altitude_m={target.height,intent_window};
    auto built=egress::navigation(input,observations(vehicle,revision),{5,0,50},1,ns(now),0);
    if(target.facing==FacingPolicy::Keep) {
      built.intent.mask=static_cast<uint16_t>(built.intent.mask|1024u);built.intent.yaw=0;
    } else if(target.facing==FacingPolicy::Heading) {
      built.intent.mask=static_cast<uint16_t>(built.intent.mask&~1024u);
      built.intent.yaw=radians(heading(target.facing_heading));
    }
    return encoded(built,now,codec);
  }
  std::optional<ControlFrame> stopFrame(const Vehicle& vehicle,double now,uint64_t revision,mavlink_status_t& codec) const override
  {
    if(!frameTime(now) || !flightReady(vehicle,now,true))return std::nullopt;
    return encoded(egress::hold(observations(vehicle,revision),{5,0,50},1,ns(now)),now,codec);
  }
private:
  unsigned m_frame_type=MAV_TYPE_QUADROTOR;
};
}
std::unique_ptr<ArduCopterControlBackend> makeArduCopterBackend()
{
  return std::make_unique<ArduCopterBackend>();
}
} // namespace mavmoos::runtime
