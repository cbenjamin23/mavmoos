#include "ControlComponents.h"
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>
using namespace mavmoos::runtime;
namespace {
void require(bool ok,const char* why) {if(!ok)throw std::runtime_error(why);}
bool near(double actual,double expected,double tolerance=1e-5) {return std::abs(actual-expected)<=tolerance;}
Vehicle vehicle(double now=100)
{
  Vehicle v;v.heartbeat_valid=v.position_valid=v.home_valid=v.landed_valid=v.estimator_valid=true;
  v.armed=v.in_air=true;v.mode=4;v.landed_state=MAV_LANDED_STATE_IN_AIR;
  v.type=MAV_TYPE_QUADROTOR;v.autopilot=MAV_AUTOPILOT_ARDUPILOTMEGA;
  v.heartbeat_at=v.position_at=v.home_at=v.landed_at=v.estimator_at=now;
  v.latitude=42.358456;v.longitude=-71.087589;v.msl_height=15;v.relative_height=5;
  v.home_latitude=v.latitude;v.home_longitude=v.longitude;v.home_msl=10;v.estimator_flags=51;
  return v;
}
Vehicle ground(double now=100,bool armed=false)
{
  auto v=vehicle(now);v.armed=armed;v.in_air=false;v.landed_state=MAV_LANDED_STATE_ON_GROUND;return v;
}
mavlink_message_t decode(const ControlFrame& frame)
{
  require(frame.size>0&&frame.size<=frame.bytes.size(),"invalid bounded frame size");
  mavlink_message_t result{},buffer{};mavlink_status_t parser{},status{};unsigned complete=0;
  for(std::size_t n=0;n<frame.size;++n)if(mavlink_frame_char_buffer(&buffer,&parser,frame.bytes[n],&result,&status)==MAVLINK_FRAMING_OK)++complete;
  require(complete==1,"frame failed generated MAVLink roundtrip");
  require(result.sysid==245&&result.compid==191,"tested source binding changed");return result;
}
mavlink_set_position_target_local_ned_t nav(const VehicleControlBackend& backend,const NavigationTarget& target,
  const Vehicle& v,double now=100,uint64_t revision=1)
{
  mavlink_status_t codec{};const auto frame=backend.navigationFrame(target,v,now,revision,codec);
  require(bool(frame),"eligible navigation frame absent");auto message=decode(*frame);
  require(message.msgid==MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED,"wrong NAV message type");
  mavlink_set_position_target_local_ned_t result{};mavlink_msg_set_position_target_local_ned_decode(&message,&result);
  require(result.target_system==1&&result.target_component==1&&result.coordinate_frame==MAV_FRAME_LOCAL_NED,"tested NAV binding/frame changed");
  return result;
}
}
int main()
{
  using Case=std::pair<const char*,std::function<void()>>;
  const auto backend=makeArduCopterBackend();require(bool(backend),"factory returned null");
  const std::vector<Case> cases={
    {"identity and profile are explicit",[&]{
      require(backend->profile()=="arducopter","profile ID changed");
      require(backend->identityCompatible(MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_ARDUPILOTMEGA),"tested identity denied");
      require(!backend->identityCompatible(MAV_TYPE_GROUND_ROVER,MAV_AUTOPILOT_ARDUPILOTMEGA)&&
        !backend->identityCompatible(MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_PX4),"untested control profile claimed compatible");
    }},
    {"flight readiness separates observations from motion",[&]{
      auto v=vehicle();require(backend->flightReady(v,100,false)&&backend->flightReady(v,100,true),"ready Copter denied");
      v=ground();v.mode=6;require(backend->flightReady(v,100,false)&&!backend->flightReady(v,100,true),"observations require motion or motion ignores state");
      for(auto field:{&Vehicle::heartbeat_valid,&Vehicle::position_valid,&Vehicle::home_valid,&Vehicle::landed_valid,&Vehicle::estimator_valid}) {
        v=vehicle();v.*field=false;require(!backend->flightReady(v,100,false),"missing prerequisite accepted");
      }
      v=vehicle();v.mode=5;require(!backend->flightReady(v,100,true)&&backend->acquisitionAllowed(v,100),"Loiter confused with eligible motion");
    }},
    {"original observation ages and future stamps",[&]{
      const std::vector<std::pair<double Vehicle::*,double>> stamps={{&Vehicle::heartbeat_at,2},{&Vehicle::position_at,.5},{&Vehicle::landed_at,2},{&Vehicle::estimator_at,3}};
      for(const auto& entry:stamps) {
        auto v=vehicle();v.*entry.first=100-entry.second;require(backend->flightReady(v,100,false),"inclusive host age boundary changed");
        v.*entry.first-=.00001;require(!backend->flightReady(v,100,false),"expired observation accepted");
        v.*entry.first=100.00001;require(!backend->flightReady(v,100,false),"future observation accepted");
        v.*entry.first=std::numeric_limits<double>::quiet_NaN();require(!backend->flightReady(v,100,false),"nonfinite observation accepted");
      }
      auto v=vehicle();v.home_at=1;require(backend->flightReady(v,100,false),"cached bound home gained an unintended age limit");
      require(!backend->flightReady(v,std::numeric_limits<double>::quiet_NaN(),false),"invalid clock accepted");
    }},
    {"acquisition and active mode grace stay distinct",[&]{
      auto v=vehicle();require(backend->acquisitionAllowed(v,100)&&backend->activeModeAllowed(v,false),"Guided acquisition denied");
      require(!backend->acquisitionCommand(v,"guided",100),"already Guided generated a mode request");
      v.mode=5;require(backend->acquisitionAllowed(v,100)&&!backend->activeModeAllowed(v,false)&&backend->activeModeAllowed(v,true),"Loiter grace policy changed");
      const auto proposal=backend->acquisitionCommand(v,"user",100);require(bool(proposal),"Loiter lacks Guided proposal");
      require(proposal->id=="acquire-user"&&proposal->command==MAV_CMD_DO_SET_MODE&&proposal->params[0]==1&&proposal->params[1]==4&&proposal->expires==105&&proposal->operation_expires==0,"acquisition proposal changed");
      v.mode=6;require(!backend->acquisitionAllowed(v,100)&&!backend->activeModeAllowed(v,true)&&!backend->acquisitionCommand(v,"rtl",100),"RTL takeover became acquirable");
      v=vehicle();v.armed=false;require(!backend->acquisitionAllowed(v,100),"unarmed acquisition accepted");v.armed=true;v.in_air=false;
      require(!backend->acquisitionAllowed(v,100),"ground acquisition accepted");
    }},
    {"Copter command allowlist and mode mapping",[&]{
      CommandRequest request;request.command=MAV_CMD_DO_SET_MODE;request.params[0]=1;
      for(float mode:{4.f,5.f}){request.params[1]=mode;require(backend->commandAllowed(request,vehicle(),100),"supported mode rejected");}
      for(float mode:{0.f,6.f,9.f}){request.params[1]=mode;require(!backend->commandAllowed(request,vehicle(),100),"unsupported explicit mode allowed");}
      request.params[1]=4;request.params[0]=0;require(!backend->commandAllowed(request,vehicle(),100),"missing custom-mode flag allowed");
      request.command=MAV_CMD_SET_MESSAGE_INTERVAL;require(!backend->commandAllowed(request,vehicle(),100),"protocol command inherited vehicle-control policy");
      for(auto command:{MAV_CMD_NAV_LAND,MAV_CMD_NAV_RETURN_TO_LAUNCH}){request.command=command;require(backend->commandAllowed(request,vehicle(),100),"supported lifecycle command rejected");}
    }},
    {"ARM DISARM retain final ground and estimator guards",[&]{
      CommandRequest request;request.command=MAV_CMD_COMPONENT_ARM_DISARM;request.params[0]=1;
      auto v=ground();require(backend->commandAllowed(request,v,100),"ground ARM denied");
      v.landed_state=MAV_LANDED_STATE_IN_AIR;require(!backend->commandAllowed(request,v,100),"airborne ARM admitted");
      v=ground();v.estimator_valid=false;require(!backend->commandAllowed(request,v,100),"ARM ignored estimator");
      request.params[0]=0;require(backend->commandAllowed(request,v,100),"DISARM gained an estimator requirement");
      v.landed_at=97;require(!backend->commandAllowed(request,v,100),"stale-ground DISARM admitted");
      v=ground();request.params[1]=21196;require(!backend->commandAllowed(request,v,100),"forced disarm admitted");
      request.params[1]=0;request.params[0]=2;require(!backend->commandAllowed(request,v,100),"invalid ARM parameter admitted");
    }},
    {"TAKEOFF retains Guided ground pose home and height guards",[&]{
      CommandRequest request;request.command=MAV_CMD_NAV_TAKEOFF;auto v=ground(100,true);
      for(float height:{1.f,20.f}){request.params[6]=height;require(backend->commandAllowed(request,v,100),"height boundary rejected");}
      for(float height:{.99f,20.01f}){request.params[6]=height;require(!backend->commandAllowed(request,v,100),"height out of envelope admitted");}
      request.params[6]=5;v.mode=5;require(!backend->commandAllowed(request,v,100),"Loiter TAKEOFF admitted");
      v=ground(100,true);v.armed=false;require(!backend->commandAllowed(request,v,100),"unarmed TAKEOFF admitted");
      v=ground(100,true);v.position_valid=false;require(!backend->commandAllowed(request,v,100),"pose-free TAKEOFF admitted");
      v=ground(100,true);v.home_valid=false;require(!backend->commandAllowed(request,v,100),"home-free TAKEOFF admitted");
    }},
    {"profile policy never replaces host authorization",[&]{
      CommandRequest request;request.command=MAV_CMD_NAV_LAND;request.id="";request.expires=0;
      require(backend->commandAllowed(request,vehicle(),100),"profile unexpectedly owns generic request ID/deadline policy");
      NavigationTarget target{90,1,5,0};require(backend->navigationInputAllowed(target),"quantity validator unexpectedly owns expiry policy");
      mavlink_status_t codec{};require(!backend->navigationFrame(target,vehicle(),100,1,codec),"expired input reached an encoded NAV frame");
    }},
    {"navigation quantities and scalar fields reject nonfinite data",[&]{
      for(double bad:{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()}) {
        for(unsigned field=0;field<3;++field)require(!backend->helmFieldAllowed(field,bad),"nonfinite helm scalar accepted");
        for(unsigned field=0;field<3;++field) {
          NavigationTarget target{90,1,5,101};if(field==0)target.course=bad;else if(field==1)target.speed=bad;else target.height=bad;
          require(!backend->navigationInputAllowed(target),"nonfinite navigation quantity accepted");
        }
      }
      require(backend->helmFieldAllowed(0,-450)&&backend->helmFieldAllowed(1,0)&&backend->helmFieldAllowed(1,5)&&backend->helmFieldAllowed(2,1)&&backend->helmFieldAllowed(2,50),"scalar boundaries changed");
      require(!backend->helmFieldAllowed(3,1)&&!backend->helmFieldAllowed(1,-.01)&&!backend->helmFieldAllowed(1,5.01)&&!backend->helmFieldAllowed(2,.99)&&!backend->helmFieldAllowed(2,50.01),"invalid scalar accepted");
    }},
    {"helm readiness remains a separate weaker predicate",[&]{
      auto v=vehicle();v.estimator_valid=v.home_valid=false;
      require(backend->helmVehicleReady(v,100,.5,false,false)&&!backend->flightReady(v,100,true),"module readiness silently became host authority");
      v.mode=5;require(!backend->helmVehicleReady(v,100,.5,false,false)&&backend->helmVehicleReady(v,100,.5,false,true),"acquisition preparation mode policy changed");
      v.mode=6;require(backend->helmVehicleReady(v,100,.5,true,false)&&!backend->helmVehicleReady(v,100,.5,false,true),"shadow preparation mode semantics changed");
      v=vehicle();v.position_at=99.4;require(backend->helmVehicleReady(v,100,.75,false,false)&&!backend->helmVehicleReady(v,100,.5,false,false),"configurable module pose age ignored");
      v=vehicle();v.armed=false;require(!backend->helmVehicleReady(v,100,.5,true,true),"shadow readiness ignored armed state");
      v=vehicle(0);require(backend->helmVehicleReady(v,0,.5,false,false)&&!backend->flightReady(v,0,false),"old host/module zero-stamp distinction changed");
    }},
    {"cardinal course velocity and course facing roundtrip",[&]{
      constexpr double pi=3.14159265358979323846;
      for(unsigned course:{0u,90u,180u,270u}) {
        const auto wire=nav(*backend,{double(course),2,5,101},vehicle());
        require(near(wire.vx,2*std::cos(course*pi/180))&&near(wire.vy,2*std::sin(course*pi/180))&&wire.vz==0,"course velocity mapping changed");
        require(wire.type_mask==2503&&near(wire.yaw,course*pi/180),"moving facing/mask changed");
      }
      const auto wrapped=nav(*backend,{450,2,5,101},vehicle());require(near(wrapped.vx,0)&&near(wrapped.vy,2),"baseline target-course wrap changed");
    }},
    {"altitude feedback and vertical clamp roundtrip",[&]{
      for(const auto& sample:std::vector<std::pair<double,double>>{{1,1},{4,.5},{5,0},{6,-.5},{50,-1}}) {
        const auto wire=nav(*backend,{90,1,sample.first,101},vehicle());require(near(wire.vz,sample.second),"home-relative altitude feedback/clamp changed");
      }
    }},
    {"zero and near-zero speed preserve altitude but leave yaw alone",[&]{
      for(double speed:{0.,.05}) {
        const auto wire=nav(*backend,{270,speed,6,101},vehicle());
        require(wire.type_mask==3527&&wire.yaw==0&&near(wire.vz,-.5),"idle yaw/altitude policy changed");
      }
      require(nav(*backend,{270,.05001,6,101},vehicle()).type_mask==2503,"moving threshold changed");
    }},
    {"encoding failure preserves original expiry and codec sequence",[&]{
      mavlink_status_t codec{};auto v=vehicle();
      require(!backend->navigationFrame({90,1,5,100},v,100,1,codec),"expired target encoded");
      require(!backend->navigationFrame({90,1,5,std::numeric_limits<double>::quiet_NaN()},v,100,1,codec),"nonfinite target expiry encoded");
      require(!backend->navigationFrame({90,1,5,101},v,std::numeric_limits<double>::infinity(),1,codec),"nonfinite clock encoded");
      require(!backend->navigationFrame({90,1,5,1e300},vehicle(1e300),1e300,1,codec),"unrepresentable model time converted to integer");
      require(!backend->navigationFrame({90,1,5,101},v,100,std::numeric_limits<uint64_t>::max(),codec),"zero wrapped revision accepted");
      require(codec.current_tx_seq==0,"failed build consumed wire sequence");
      const auto one=backend->navigationFrame({90,1,5,100.4},v,100,1,codec);
      const auto two=backend->navigationFrame({90,1,5,100.4},v,100.2,2,codec);
      require(one&&two&&decode(*one).seq==0&&decode(*two).seq==1,"host-owned codec sequencing changed");
      require(!backend->navigationFrame({90,1,5,100.4},v,100.4,3,codec),"encoding renewed the original expiry");
    }},
    {"P2 strict sample expiry remains distinct from inclusive host readiness",[&]{
      auto v=vehicle();mavlink_status_t codec{};
      require(backend->flightReady(v,100.5,true),"host boundary unexpectedly tightened");
      require(!backend->navigationFrame({90,1,5,101},v,100.5,1,codec)&&!backend->stopFrame(v,100.5,1,codec),"P2 original sample expiry was renewed");
      require(bool(backend->navigationFrame({90,1,5,101},v,100.49999,1,codec)),"unexpired sample rejected");
    }},
    {"frame prerequisites and nonfinite pose stay guarded",[&]{
      auto v=vehicle();mavlink_status_t codec{};v.mode=5;
      require(!backend->navigationFrame({90,1,5,101},v,100,1,codec)&&!backend->stopFrame(v,100,1,codec),"Loiter encoded a Guided target");
      v=vehicle();v.north_velocity=std::numeric_limits<double>::quiet_NaN();require(!backend->navigationFrame({90,1,5,101},v,100,1,codec),"nonfinite pose encoded");
      v=vehicle();v.relative_height=51;require(!backend->navigationFrame({90,1,5,101},v,100,1,codec),"pose outside P2 envelope encoded");
      require(!backend->navigationFrame({90,1,0,101},vehicle(),100,1,codec),"invalid requested height encoded");
    }},
    {"stop response uses current global position and MSL height",[&]{
      auto v=vehicle();mavlink_status_t codec{};const auto frame=backend->stopFrame(v,100,1,codec);require(bool(frame),"eligible stop response absent");
      const auto message=decode(*frame);require(message.msgid==MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT,"stop became native Loiter command");
      mavlink_set_position_target_global_int_t wire{};mavlink_msg_set_position_target_global_int_decode(&message,&wire);
      require(wire.target_system==1&&wire.target_component==1&&wire.coordinate_frame==MAV_FRAME_GLOBAL_INT&&wire.type_mask==3576,"stop binding/frame/mask changed");
      require(wire.lat_int==423584560&&wire.lon_int==-710875890&&wire.alt==15,"stop confused MSL and home-relative altitude");
      v.latitude=91;require(!backend->stopFrame(v,100,1,codec),"invalid global latitude encoded");
      v=vehicle();v.msl_height=std::numeric_limits<double>::quiet_NaN();require(!backend->stopFrame(v,100,1,codec),"nonfinite global altitude encoded");
    }},
    {"long uptime keeps explicit uint32 wire timestamp wrap",[&]{
      const double now=5000000.;const auto wire=nav(*backend,{90,1,5,now+1},vehicle(now),now);
      require(wire.time_boot_ms==705032704U,"wire uptime conversion changed");
    }}
  };
  unsigned passed=0;
  try {for(const auto& item:cases){item.second();++passed;std::cout<<"PASS "<<item.first<<'\n';}}
  catch(const std::exception& error){std::cerr<<"FAIL "<<cases[passed].first<<": "<<error.what()<<'\n';return 1;}
  std::cout<<"PASS "<<passed<<" pure Copter backend groups; no Services, socket or vehicle I/O\n";return 0;
}
