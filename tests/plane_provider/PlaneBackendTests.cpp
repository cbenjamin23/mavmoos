#include "PlaneFixture.h"
#include <limits>

using namespace plane_fixture;

namespace {
void update(plane::Backend& backend,const Vehicle& vehicle,double at)
{
  backend.observe(state(),{at,at,"READ_ACQUISITION"},vehicle,at);
  backend.observe(sensor(),{at,at,"READ_ACQUISITION"},vehicle,at);
}

mavlink_command_int_t decode(const ControlFrame& frame)
{
  mavlink_message_t message{},parser{};
  mavlink_status_t status{},result{};
  unsigned frames=0;
  for(std::size_t byte=0;byte<frame.size;++byte) {
    const auto outcome=mavlink_frame_char_buffer(&parser,&status,frame.bytes[byte],&message,&result);
    if(outcome==MAVLINK_FRAMING_OK)++frames;
  }
  require(frames==1&&message.msgid==MAVLINK_MSG_ID_COMMAND_INT&&message.magic==MAVLINK_STX,
    "expected one MAVLink 2 COMMAND_INT frame");
  require(message.sysid==245&&message.compid==191,"unexpected bridge identity");
  mavlink_command_int_t decoded{};
  mavlink_msg_command_int_decode(&message,&decoded);
  return decoded;
}

void checkWireAndBounds()
{
  plane::Backend backend(1);
  auto vehicle=airborne(2,plane::loiter_mode);
  update(backend,vehicle,2);
  auto acquire=backend.acquisitionCommand(vehicle,"fixture",2);
  require(acquire&&acquire->command==MAV_CMD_DO_SET_MODE&&acquire->params[1]==plane::guided_mode,
    "Loiter acquisition must request Guided through the host");
  require(backend.commandAllowed(*acquire,vehicle,2),"typed Guided acquisition rejected");
  mavlink_status_t codec{};
  require(!backend.navigationFrame(intent(3),vehicle,2,0,codec),"navigation encoded before Guided");

  vehicle.mode=plane::guided_mode;
  require(backend.acquisitionAllowed(vehicle,2)&&!backend.acquisitionCommand(vehicle,"resume",2),
    "Guided can only resume without a second mode effect");
  auto frame=backend.navigationFrame(intent(3,90),vehicle,2,0,codec);
  require(frame.has_value(),"qualified course did not encode");
  const auto point=decode(*frame);
  require(point.target_system==1&&point.target_component==1&&point.command==MAV_CMD_DO_REPOSITION&&
    point.frame==MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,"invalid fixed-wing Guided target");
  require(point.param1==-1&&point.param2==0&&point.param3==0&&point.param4==0&&point.z==75,
    "target introduced mode change, radius/yaw override or speed request");
  require(std::abs(point.x/1e7-43.)<1e-5&&point.y/1e7>-70.997&&point.y/1e7<-70.996,
    "eastward 300 m lookahead geometry is incorrect");
  require(!backend.stopFrame(vehicle,2,0,codec),"fixed-wing stop must use native handback");

  auto rejected=intent(3);
  rejected.speed=0;
  require(!backend.navigationInputAllowed(rejected),"zero-speed fixed-wing target accepted");
  rejected=intent(3);rejected.speed=23;
  require(!backend.navigationInputAllowed(rejected),"out-of-profile speed accepted");
  rejected=intent(3);rejected.height=44;
  require(!backend.navigationInputAllowed(rejected),"unsafe height accepted");
  rejected=intent(3);rejected.facing=FacingPolicy::Heading;
  require(!backend.navigationInputAllowed(rejected),"independent yaw accepted");
  rejected=intent(3);rejected.kind=NavigationKind::GlobalPosition;
  require(!backend.navigationInputAllowed(rejected),"unqualified direct point accepted");
  auto direct=rejected;
  direct.latitude=43.001;
  direct.longitude=-71.001;
  const auto direct_frame=backend.navigationFrame(direct,vehicle,2,0,codec);
  require(direct_frame&&decode(*direct_frame).x==430010000&&decode(*direct_frame).y==-710010000,
    "exact operator waypoint was not encoded within the envelope");
  direct.latitude+=.02;
  require(!backend.navigationFrame(direct,vehicle,2,0,codec),"direct waypoint escaped the orbit reserve");
  require(!backend.navigationFrame(intent(2),vehicle,2,0,codec),"expired motion target encoded");
  vehicle.latitude+=.009;
  require(backend.flightReady(vehicle,2,true)&&!backend.navigationFrame(intent(3),vehicle,2,0,codec),
    "outbound point failed to reserve its orbit margin");
  require(backend.navigationFrame(intent(3,180),vehicle,2,0,codec).has_value(),
    "safe inbound point rejected near envelope edge");

  vehicle=airborne(2,plane::guided_mode);
  vehicle.longitude=vehicle.home_longitude=179.999;
  frame=backend.navigationFrame(intent(3,90),vehicle,2,0,codec);
  require(frame&&decode(*frame).y<0,"valid lookahead did not cross antimeridian");
}

void checkObservationAndHandback()
{
  plane::Backend backend(1);
  auto vehicle=airborne(2,plane::guided_mode);
  update(backend,vehicle,2);
  require(backend.flightReady(vehicle,2,true),"valid readiness evidence rejected");
  backend.observe(sensor(18,AIRSPEED_SENSOR_UNHEALTHY,1),{2.1,2.1,"READ_ACQUISITION"},vehicle,2.1);
  backend.observe(sensor(18,AIRSPEED_SENSOR_UNHEALTHY,0,1,4),{2.1,2.1,"READ_ACQUISITION"},vehicle,2.1);
  require(backend.airspeed()==22.,"unselected airspeed source changed readiness");
  backend.observe(sensor(20,AIRSPEED_SENSOR_UNHEALTHY),{2.1,2.2,"QUEUE_EMPTY_FLOOR"},vehicle,2.2);
  require(!backend.flightReady(vehicle,2.2,true),"new unhealthy selected sensor did not revoke readiness");
  backend.observe(sensor(),{2.05,2.25,"QUEUE_EMPTY_FLOOR"},vehicle,2.25);
  require(!backend.airspeed(),"older effective age replaced invalidation");
  backend.observe(sensor(),{2.2,2.15,"QUEUE_EMPTY_FLOOR"},vehicle,2.25);
  require(!backend.airspeed(),"inverted time renewed readiness");
  backend.observe(sensor(22,0),{2.3,2.3,"READ_ACQUISITION"},vehicle,2.3);
  require(!backend.airspeed(),"zero-truncated USING bit was invented");
  backend.observe(sensor(std::numeric_limits<float>::quiet_NaN()),{2.4,2.4,"READ_ACQUISITION"},vehicle,2.4);
  require(!backend.airspeed(),"non-finite CAS qualified");

  auto loiter=plane::modeRequest("stop",plane::loiter_mode,3);
  const auto rtl=plane::modeRequest("return",plane::rtl_mode,3);
  require(backend.commandAllowed(loiter,vehicle,2.4)&&!backend.commandAllowed(rtl,vehicle,2.4),
    "degraded readiness permits only a bounded stop-only Loiter at host discretion");
  loiter.params[2]=1;
  require(!backend.commandAllowed(loiter,vehicle,2.4),"mode command smuggled extra parameters");

  backend.reset(2.5);
  vehicle=airborne(2.6,plane::guided_mode);
  backend.observe(sensor(),{2.4,2.6,"QUEUE_EMPTY_FLOOR"},vehicle,2.6);
  backend.observe(state(),{2.6,2.6,"READ_ACQUISITION"},vehicle,2.6);
  require(!backend.flightReady(vehicle,2.6,true),"reset-era sensor survived the observation floor");
  update(backend,vehicle,2.6);
  require(backend.flightReady(vehicle,2.6,true),"new observation did not restore eligibility");

  auto invalid=vehicle;invalid.armed=false;
  require(!backend.flightReady(invalid,2.6,true),"disarmed sample qualified");
  invalid=vehicle;invalid.type=MAV_TYPE_QUADROTOR;
  require(!backend.flightReady(invalid,2.6,true),"wrong firmware profile qualified");
  invalid=vehicle;invalid.estimator_flags|=EKF_GPS_GLITCHING;
  require(!backend.flightReady(invalid,2.6,true),"failed estimator qualified");
  invalid=vehicle;invalid.home_at=0;
  require(!backend.flightReady(invalid,2.6,true),"stale home qualified");
  invalid=vehicle;invalid.msl_height=250;
  require(!backend.flightReady(invalid,2.6,true),"inconsistent height datum qualified");
  backend.observe(state(MAV_VTOL_STATE_FW),{2.6,2.6,"READ_ACQUISITION"},vehicle,2.6);
  require(!backend.flightReady(vehicle,2.6,true),"QuadPlane state qualified as conventional Plane");

  vehicle=airborne(4.3,plane::guided_mode);
  backend.observe(state(),{4.3,4.3,"READ_ACQUISITION"},vehicle,4.3);
  require(!backend.flightReady(vehicle,4.3,true),"stale original airspeed received a new expiry");
}

void checkGroundAndAirspeedCommands()
{
  plane::Backend backend(1);
  auto vehicle=airborne(2,plane::manual_mode);
  vehicle.armed=vehicle.in_air=false;
  vehicle.landed_state=MAV_LANDED_STATE_ON_GROUND;
  vehicle.msl_height=vehicle.home_msl;
  vehicle.relative_height=vehicle.north_velocity=0;
  backend.observe(state(MAV_VTOL_STATE_UNDEFINED,MAV_LANDED_STATE_ON_GROUND),{2,2,"READ_ACQUISITION"},vehicle,2);
  CommandRequest arm;
  arm.command=MAV_CMD_COMPONENT_ARM_DISARM;
  arm.params[0]=1;
  require(!backend.commandAllowed(arm,vehicle,2),"ARM admitted without selected prearm evidence");
  mavlink_sys_status_t prearm{};
  prearm.onboard_control_sensors_present=MAV_SYS_STATUS_PREARM_CHECK;
  prearm.onboard_control_sensors_enabled=MAV_SYS_STATUS_PREARM_CHECK;
  prearm.onboard_control_sensors_health=MAV_SYS_STATUS_PREARM_CHECK;
  mavlink_message_t observation{};
  mavlink_msg_sys_status_encode(1,1,&observation,&prearm);
  backend.observe(observation,{2,2,"READ_ACQUISITION"},vehicle,2);
  require(backend.commandAllowed(arm,vehicle,2),"non-forced disarmed ground ARM was denied");
  auto manual=plane::modeRequest("manual",plane::manual_mode,3);
  vehicle.mode=plane::rtl_mode;
  require(backend.commandAllowed(manual,vehicle,2)&&!backend.commandAllowed(arm,vehicle,2),
    "disarmed RTL permitted arming or denied a grounded Manual handoff");
  vehicle.armed=true;
  require(!backend.commandAllowed(manual,vehicle,2),"armed RTL authorized a Manual handoff");
  vehicle.armed=false;
  vehicle.mode=plane::manual_mode;
  arm.params[1]=21196;
  require(!backend.commandAllowed(arm,vehicle,2),"forced ground ARM was admitted");
  arm.params[1]=0;
  require(!backend.commandAllowed(arm,vehicle,2.6),"old ground position authorized ARM");
  vehicle.armed=true;arm.params[0]=0;
  require(backend.commandAllowed(arm,vehicle,2),"observed ground did not permit disarm");
  vehicle.north_velocity=4;
  require(!backend.commandAllowed(arm,vehicle,2),"moving aircraft authorized disarm");

  vehicle=airborne(3,plane::guided_mode);
  update(backend,vehicle,3);
  CommandRequest speed;
  speed.command=MAV_CMD_GUIDED_CHANGE_SPEED;
  speed.params[1]=18;speed.params[2]=2;
  require(backend.commandAllowed(speed,vehicle,3),"qualified Guided speed was denied");
  speed.params[1]=14;
  require(!backend.commandAllowed(speed,vehicle,3),"unsafe speed was admitted");
  speed.params[1]=18;
  vehicle.mode=plane::loiter_mode;
  require(!backend.commandAllowed(speed,vehicle,3),"Guided speed escaped its owned mode");
}
} // namespace

int main()
{
  try {
    checkWireAndBounds();
    checkObservationAndHandback();
    checkGroundAndAirspeedCommands();
    std::cout<<"Plane backend geometry, wire, freshness, and degraded handback passed\n";
  } catch(const std::exception& error) {
    std::cerr<<error.what()<<'\n';
    return 1;
  }
}
