#include "Px4Offboard.h"
#include "Px4RoverBackend.h"
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace mavmoos::runtime;
namespace {
constexpr unsigned mission=(4u<<16)|(4u<<24);
constexpr unsigned manual=1u<<16;
constexpr uint32_t first_gps_time=1000000;

void require(bool ok,const char* reason)
{ if(!ok) throw std::runtime_error(reason); }

class Harness final : public Services {
public:
  double now() const override { return clock; }
  const Vehicle& vehicle() const override { return state; }
  void publish(const std::string&,const std::string&) override {}
  void publish(const std::string&,double) override {}
  bool sendCommand(const CommandRequest&) override { return false; }
  bool acquireNavigation(const std::string&) override { return false; }
  void revokeNavigation(const std::string&) override {}
  bool navigation(const NavigationTarget&) override { return false; }
  bool parameter(const ParameterRequest& request) override
  {
    last_read=request;
    return true;
  }

  void at(double stamp)
  {
    clock=stamp;
    state.heartbeat_valid=state.position_valid=state.home_valid=state.estimator_horizontal_valid=true;
    state.heartbeat_at=state.position_at=state.home_at=state.estimator_at=stamp;
  }

  void gps(uint64_t firmware_usec,uint16_t speed=10,int32_t lat=470000000,uint32_t velocity_accuracy=400,
           uint8_t sysid=1)
  {
    mavlink_gps_raw_int_t packet{};
    packet.time_usec=firmware_usec;
    packet.lat=lat;
    packet.lon=80000000;
    packet.vel=speed;
    packet.fix_type=GPS_FIX_TYPE_3D_FIX;
    packet.satellites_visible=10;
    packet.h_acc=900;
    packet.vel_acc=velocity_accuracy;
    mavlink_message_t message{};
    mavlink_msg_gps_raw_int_encode(sysid,1,&message,&packet);
    backend.observe(message,{clock,clock},state,clock);
  }

  void currentMode(unsigned active,unsigned intended)
  {
    mavlink_current_mode_t packet{};
    packet.custom_mode=active;
    packet.intended_custom_mode=intended;
    mavlink_message_t message{};
    mavlink_msg_current_mode_encode(1,1,&message,&packet);
    backend.observe(message,{clock,clock},state,clock);
  }

  void param(const char* name,float value,uint8_t type,uint8_t sysid=1)
  {
    mavlink_param_value_t packet{};
    std::strncpy(packet.param_id,name,sizeof(packet.param_id));
    packet.param_value=value;
    packet.param_type=type;
    packet.param_count=100;
    packet.param_index=1;
    mavlink_message_t message{};
    mavlink_msg_param_value_encode(sysid,1,&message,&packet);
    backend.observe(message,{clock,clock},state,clock);
  }

  void confirmConfig()
  {
    struct Setting { const char* name; int32_t int_value; float float_value; };
    const Setting settings[]={{"SYS_AUTOSTART",10045,0},{"MAV_FWDEXTSP",1,0},
                              {"COM_OBL_RC_ACT",5,0},{"COM_OF_LOSS_T",0,1}};
    for(const auto& setting:settings) {
      backend.readConfiguration(*this);
      require(last_read && last_read->name==setting.name && last_read->action==ParameterAction::Read,
              "unexpected profile configuration read");
      float value=setting.float_value;
      uint8_t type=MAV_PARAM_TYPE_REAL32;
      if(setting.int_value) {
        std::memcpy(&value,&setting.int_value,sizeof(value));
        type=MAV_PARAM_TYPE_INT32;
      }
      // Neither an unsolicited correct value nor a foreign reply advances the series.
      param(setting.name,value,type);
      backend.parameterSent(*last_read,clock,true);
      at(clock+.05);
      param(setting.name,value,type,42);
      at(clock+.05);
      param(setting.name,value,type);
      at(clock+.25);
    }
  }

  void quiet(double start,uint64_t firmware_start=first_gps_time)
  {
    for(unsigned i=0;i<9;++i) {
      at(start+i*.25);
      gps(firmware_start+i*250000u,12,470000000+(i%2)*9);
    }
  }

  Px4RoverBackend backend;
  Vehicle state;
  double clock=10;
  std::optional<ParameterRequest> last_read;
};

void vehicle(Harness& harness)
{
  auto& state=harness.state;
  state.autopilot=MAV_AUTOPILOT_PX4;
  state.type=MAV_TYPE_GROUND_ROVER;
  state.armed=true;
  state.in_air=true; // PX4 reports this while the armed ground rover drives.
  state.mode=px4::hold;
  state.latitude=state.home_latitude=47;
  state.longitude=state.home_longitude=8;
  state.heading=358;
  harness.at(10);
}

mavlink_set_position_target_local_ned_t velocity(const ControlFrame& frame)
{
  mavlink_message_t parser{},message{};
  mavlink_status_t status{},decoded_status{};
  unsigned messages=0;
  for(std::size_t i=0;i<frame.size;++i) {
    if(mavlink_frame_char_buffer(&parser,&status,frame.bytes[i],&message,&decoded_status)==MAVLINK_FRAMING_OK) {
      ++messages;
    }
  }
  require(messages==1&&message.msgid==MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED,"not one velocity message");
  mavlink_set_position_target_local_ned_t target{};
  mavlink_msg_set_position_target_local_ned_decode(&message,&target);
  require(target.target_system==1&&target.target_component==1&&target.coordinate_frame==MAV_FRAME_LOCAL_NED&&
          target.type_mask==0x0DC7&&target.vz==0,"unexpected Ackermann wire shape");
  return target;
}

void dwellAndMotion()
{
  Harness h;
  vehicle(h);
  h.confirmConfig();
  require(h.backend.verifiedConfig(h.clock),"four accepted exact profile reads did not qualify the model");
  require(!h.backend.acquisitionAllowed(h.state,h.clock),"acquired without a stop and Hold evidence");
  h.quiet(12);
  h.currentMode(px4::hold,px4::offboard);
  require(h.backend.stopped(h.clock,11)&&!h.backend.stopped(h.clock,13),"stop dwell reused a pre-boundary interval");
  require(!h.backend.acquisitionAllowed(h.state,h.clock),"retained Offboard intent admitted acquisition");
  h.currentMode(px4::hold,px4::hold);
  require(h.backend.acquisitionAllowed(h.state,h.clock),"qualified armed Hold did not acquire");
  auto mode=h.backend.acquisitionCommand(h.state,"stock",h.clock);
  require(mode && h.backend.commandAllowed(*mode,h.state,h.clock),"valid Offboard mode command rejected");
  mode->params[2]=4;
  require(!h.backend.commandAllowed(*mode,h.state,h.clock),"wrong mode sub-command admitted");

  CommandRequest start;
  start.command=MAV_CMD_MISSION_START;
  start.params[1]=1;
  require(h.backend.missionExecutionCommandAllowed(start,h.state,h.clock),"verified armed Hold cannot start");
  start.params[1]=1.5f;
  require(!h.backend.missionExecutionCommandAllowed(start,h.state,h.clock),"fractional final waypoint admitted");
  h.state.armed=false;
  start.params[1]=1;
  require(!h.backend.missionExecutionCommandAllowed(start,h.state,h.clock),"disarmed mission start admitted");
  h.state.armed=true;
  h.state.mode=px4::offboard;
  NavigationTarget target;
  target.dimensions=NavigationDimensions::Planar;
  target.course=2;
  target.speed=1;
  target.expires=h.clock+1;
  mavlink_status_t codec{};
  require(!h.backend.navigationFrame(target,h.state,h.clock,1,codec),
          "Offboard heartbeat alone allowed motion");
  h.currentMode(px4::offboard,px4::offboard);
  auto motion=h.backend.navigationFrame(target,h.state,h.clock,1,codec);
  require(motion.has_value(),"within-envelope motion did not encode");
  const auto packet=velocity(*motion);
  require(packet.vx>.99 && packet.vy>.03 && packet.vy<.04,"wrong local course vector");
  target.course=140;
  require(!h.backend.targetAllowed(target,h.state,h.clock),"excessive steering error passed final policy");
  target.speed=0;
  auto zero=h.backend.navigationFrame(target,h.state,h.clock,1,codec);
  require(zero.has_value()&&velocity(*zero).vx==0&&velocity(*zero).vy==0,"zero speed sent motion");
}

void invalidation()
{
  Harness h;
  vehicle(h);
  h.confirmConfig();
  h.quiet(12);
  const uint64_t last_time=first_gps_time+8u*250000u;
  h.at(14.9);
  h.gps(last_time);
  h.at(15.1);
  require(!h.backend.stopped(h.clock,-1),"a repeated producer stamp extended the evidence lifetime");
  h.quiet(16,last_time+1500000);
  require(h.backend.stopped(h.clock,-1),"advancing fresh evidence did not requalify");
  h.at(18.1);
  h.gps(last_time+3400000,0,470000000,400,42);
  require(h.backend.stopped(h.clock,-1),"foreign GPS invalidated selected evidence");
  h.gps(last_time+3400000);
  require(!h.backend.stopped(h.clock,-1),"producer clock regression kept a stop proof");
  h.quiet(19,last_time+5000000);
  require(h.backend.stopped(h.clock,-1),"fresh stopped samples after regression did not qualify");
  h.at(21.25);
  h.gps(last_time+7250000,0,470000000,0);
  require(!h.backend.stopped(h.clock,-1),"missing reported uncertainty kept stop proof");
  h.quiet(22,last_time+8000000);
  h.at(24.25);
  h.gps(last_time+10250000,0,470000200);
  require(!h.backend.stopped(h.clock,-1),"position excursion kept stop proof");
  h.at(25);
  h.param("COM_OF_LOSS_T",2,MAV_PARAM_TYPE_REAL32);
  require(!h.backend.verifiedConfig(h.clock),"contradictory verified fallback parameter was ignored");

  CommandRequest stop;
  stop.command=MAV_CMD_DO_SET_MODE;
  stop.params[0]=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  stop.params[1]=4;
  stop.params[2]=3;
  stop.abort_id="abort";
  h.state.mode=manual;
  require(!h.backend.missionExecutionCommandAllowed(stop,h.state,h.clock),"a stop command could take over Manual");
  h.state.mode=mission;
  require(h.backend.missionExecutionCommandAllowed(stop,h.state,h.clock),"uncertain active Mission cannot request stop");
  h.backend.reset(h.clock);
  require(!h.backend.verifiedConfig(h.clock)&&!h.backend.stopped(h.clock,-1)&&
          !h.backend.holdObserved(h.clock,-1),"reset retained old session evidence");
}

void configurationUncertainty()
{
  Harness h;
  vehicle(h);
  h.backend.readConfiguration(h);
  require(h.last_read && h.last_read->name=="SYS_AUTOSTART","startup read missing");
  const auto initial=*h.last_read;
  int32_t exact=10045;
  float wire=0;
  std::memcpy(&wire,&exact,sizeof(wire));
  h.at(10.05);
  h.param("SYS_AUTOSTART",wire,MAV_PARAM_TYPE_INT32);
  h.at(10.3);
  h.backend.readConfiguration(h);
  require(h.last_read->id==initial.id,"unsolicited value advanced profile verification");
  h.at(initial.expires+.05);
  h.backend.parameterSent(initial,h.clock,true);
  h.param("SYS_AUTOSTART",wire,MAV_PARAM_TYPE_INT32);
  h.backend.readConfiguration(h);
  require(h.last_read->id!=initial.id && h.last_read->name=="SYS_AUTOSTART",
          "late local handoff or reply advanced profile verification");

  h.backend.reset(h.clock);
  h.confirmConfig();
  require(h.backend.verifiedConfig(h.clock),"new session could not establish profile");
  h.at(h.clock+31);
  require(!h.backend.verifiedConfig(h.clock),"old profile evidence survived without fresh readback");
}
} // namespace

int main()
{
  try {
    dwellAndMotion();
    invalidation();
    configurationUncertainty();
    std::cout<<"PX4 Ackermann backend evidence and command checks passed\n";
  } catch(const std::exception& error) {
    std::cerr<<error.what()<<'\n';
    return 1;
  }
  return 0;
}
