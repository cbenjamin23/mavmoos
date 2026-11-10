// Pure Rover backend/transaction checks. No socket, MOOS process or simulator.
#include "RoverComponents.h"
#include "NativeReplayDirectory.h"
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>
using namespace mavmoos::runtime;
namespace {
void require(bool value,const std::string& message){if(!value)throw std::runtime_error(message);}
void close(double a,double b,const std::string& why){require(std::abs(a-b)<1e-5,why);}
Vehicle fresh(double at=100)
{
  Vehicle v;v.type=MAV_TYPE_GROUND_ROVER;v.autopilot=MAV_AUTOPILOT_ARDUPILOTMEGA;
  v.heartbeat_valid=v.position_valid=v.estimator_horizontal_valid=true;v.armed=true;v.mode=rover::guided_mode;
  v.heartbeat_at=v.position_at=v.estimator_at=at;
  v.estimator_flags=EKF_ATTITUDE|EKF_VELOCITY_HORIZ|EKF_POS_HORIZ_ABS;
  v.latitude=42.358456;v.longitude=-71.087589;v.heading=123;
  v.relative_height=v.msl_height=v.down_velocity=std::numeric_limits<double>::quiet_NaN();
  return v;
}
NavigationTarget target(double course=90,double speed=2)
{
  NavigationTarget n;n.course=course;n.speed=speed;n.expires=100.5;
  n.dimensions=NavigationDimensions::Planar;n.height=std::numeric_limits<double>::quiet_NaN();return n;
}
mavlink_set_position_target_global_int_t decode(const ControlFrame& frame)
{
  mavlink_message_t message{};mavlink_status_t status{};bool seen=false;
  for(std::size_t i=0;i<frame.size;++i)if(mavlink_parse_char(0,frame.bytes[i],&message,&status))seen=true;
  require(seen&&message.msgid==MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT,"standard global target frame missing");
  require(message.sysid==245&&message.compid==191,"wrong source identity");
  mavlink_set_position_target_global_int_t result{};mavlink_msg_set_position_target_global_int_decode(&message,&result);
  require(result.target_system==1&&result.target_component==1,"wrong fixed target");return result;
}
struct Fake final:Services {
  NativeReplayDirectory replay;
  double time=100;Vehicle state=fresh();bool admit=true;
  std::vector<CommandRequest> commands;std::vector<std::string> results,finished;
  double now() const override{return time;}const Vehicle& vehicle() const override{return state;}
  void publish(const std::string& key,const std::string& text) override{if(key=="MAVMOOS_COMMAND_RESULT")results.push_back(text);}
  void publish(const std::string&,double) override{}
  bool sendCommand(const CommandRequest& r) override{commands.push_back(r);return admit;}
  bool acquireNavigation(const std::string&) override{return false;}
  void revokeNavigation(const std::string&) override{}
  bool navigation(const NavigationTarget&) override{return false;}
  void commandFinished(const std::string& id) override{finished.push_back(id);}
  void update(double at){time=at;state.heartbeat_at=state.position_at=state.estimator_at=at;}
  void mail(Plugin& plugin,const std::string& text,double at=100.1){update(at);Mail m;m.key="MAVMOOS_COMMAND";m.text=text;m.received_at=m.published_at=at;plugin.onMail(m);}
  std::unique_ptr<Plugin> configured(const VehicleControlBackend& backend) {
    auto commands=makeRoverCommands(*this,backend);
    commands->configure("native_replay_directory",replay.path.string());
    return commands;
  }
  void sent(Plugin& plugin){time+=.001;plugin.onCommandSent(commands.back(),time,true);}
  bool has(const std::string& value)const{for(const auto& r:results)if(r.find(value)!=std::string::npos)return true;return false;}
};
void ack(Plugin& plugin,uint16_t command,uint8_t result,double at)
{
  mavlink_message_t m{};mavlink_command_ack_t p{};p.command=command;p.result=result;
  mavlink_msg_command_ack_encode(1,1,&m,&p);plugin.onMessage(m,at);
}
}
int main()
{
  try {
    auto backend=makeArduRoverBackend();unsigned count=0;
    const auto boat=makeArduRoverBackend(true);
    const auto test=[&](const char* label,const std::function<void()>& run){run();++count;std::cout<<"PASS "<<label<<'\n';};
    test("planar profile needs no altitude home landed or airborne state",[&]{
      auto v=fresh();const auto h=backend->helmProfile();require(h.dimensions==NavigationDimensions::Planar&&!h.requires_height&&!h.requires_home,"wrong helm contract");
      require(!v.home_valid&&!v.landed_valid&&!v.estimator_valid&&!v.in_air,"fixture accidentally supplies Copter prerequisites");
      require(backend->flightReady(v,100,true)&&backend->helmVehicleReady(v,100,.5,false,false),"Copter prerequisites leaked into Rover");
    });
    test("exact ground Rover ArduPilot identity excludes boat copter and PX4",[&]{
      require(backend->identityCompatible(10,3),"ground identity denied");
      for(const auto& p:std::vector<std::pair<unsigned,unsigned>>{{2,3},{11,3},{10,12},{0,3}})require(!backend->identityCompatible(p.first,p.second),"unsupported identity accepted");
    });
    test("explicit boat profile binds surface boat ArduPilot and retains planar prerequisites",[&]{
      require(backend->profile()=="ardurover"&&boat->profile()=="ardurover_boat","wrong profile label");
      require(boat->identityCompatible(MAV_TYPE_SURFACE_BOAT,MAV_AUTOPILOT_ARDUPILOTMEGA),"boat identity denied");
      require(!boat->identityCompatible(MAV_TYPE_GROUND_ROVER,MAV_AUTOPILOT_ARDUPILOTMEGA),"ground identity admitted by boat profile");
      require(!boat->identityCompatible(MAV_TYPE_SURFACE_BOAT,MAV_AUTOPILOT_PX4),"non-ArduPilot boat admitted");
      auto v=fresh();
      require(!boat->flightReady(v,100,true),"ground observations satisfy boat readiness");
      v.type=MAV_TYPE_SURFACE_BOAT;
      const auto profile=boat->helmProfile();
      require(profile.dimensions==NavigationDimensions::Planar&&!profile.requires_height&&!profile.requires_home,
              "wrong planar boat helm requirements");
      require(boat->flightReady(v,100,true)&&boat->helmVehicleReady(v,100,.5,false,false),"boat needs vertical evidence");
      v.estimator_flags&=~unsigned(EKF_POS_HORIZ_ABS);
      require(!boat->flightReady(v,100,true),"boat accepted missing horizontal estimator flag");
    });
    test("horizontal freshness rejects stale future invalid clocks",[&]{
      for(const auto field:{0,1,2}){auto v=fresh();if(field==0)v.heartbeat_at=97.9;if(field==1)v.position_at=99.49;if(field==2)v.estimator_at=96.9;require(!backend->flightReady(v,100,true),"stale prerequisite accepted");}
      for(double stamp:{101.,-1.,std::numeric_limits<double>::quiet_NaN()}){auto v=fresh();v.position_at=stamp;require(!backend->flightReady(v,100,true),"invalid stamp accepted");}
    });
    test("horizontal estimator flags required vertical flags irrelevant",[&]{
      for(unsigned bit:{unsigned(EKF_ATTITUDE),unsigned(EKF_VELOCITY_HORIZ),unsigned(EKF_POS_HORIZ_ABS)}){auto v=fresh();v.estimator_flags&=~bit;require(!backend->flightReady(v,100,true),"missing horizontal flag accepted");}
      auto v=fresh();v.estimator_flags|=EKF_UNINITIALIZED;require(!backend->flightReady(v,100,true),"uninitialized estimator accepted");
      v=fresh();v.estimator_horizontal_valid=false;require(!backend->flightReady(v,100,true),"invalid horizontal variance accepted");
    });
    test("unknown horizontal pose or heading prevents motion",[&]{
      for(unsigned i=0;i<3;++i){auto v=fresh();if(i==0)v.latitude=91;if(i==1)v.heading=NAN;if(i==2)v.position_valid=false;require(!backend->flightReady(v,100,true),"invalid navigation observation accepted");}
    });
    test("Guided acquisition from Hold proposes Rover mode15 only",[&]{
      auto v=fresh();require(backend->acquisitionAllowed(v,100)&&!backend->acquisitionCommand(v,"one",100),"already Guided setup");
      v.mode=4;require(backend->acquisitionAllowed(v,100),"Hold acquisition denied");const auto r=backend->acquisitionCommand(v,"one",100);
      require(r&&r->command==MAV_CMD_DO_SET_MODE&&r->params[0]==1&&r->params[1]==15&&r->expires==105,"wrong Rover setup command");
      v.mode=0;require(!backend->acquisitionAllowed(v,100)&&!backend->activeModeAllowed(v,true),"manual takeover accepted");
      v.mode=4;require(backend->activeModeAllowed(v,true)&&!backend->activeModeAllowed(v,false),"Hold grace invalid");
    });
    test("deliberate boat acquisition from native Loiter requests Guided with a fixed deadline",[&]{
      auto v=fresh();
      v.type=MAV_TYPE_SURFACE_BOAT;
      v.mode=rover::loiter_mode;
      const auto setup=boat->acquisitionCommand(v,"loiter",100);
      require(boat->acquisitionAllowed(v,100)&&setup,"fresh armed boat Loiter could not request Guided");
      require(setup->id=="acquire-loiter"&&setup->command==MAV_CMD_DO_SET_MODE&&
              setup->params[0]==MAV_MODE_FLAG_CUSTOM_MODE_ENABLED&&setup->params[1]==rover::guided_mode&&setup->expires==105,
              "wrong boat Loiter acquisition command");
      require(boat->activeModeAllowed(v,true)&&!boat->activeModeAllowed(v,false),"Loiter allowed outside acquisition grace");
      require(boat->helmVehicleReady(v,100,.5,false,true)&&!boat->flightReady(v,100,true),
              "setup and actual motion eligibility conflated");
      require(!boat->acquisitionCommand(v,"stale",101),"stale boat Loiter proposed setup");
      v.armed=false;
      require(!boat->acquisitionAllowed(v,100)&&!boat->acquisitionCommand(v,"unarmed",100),"disarmed boat Loiter proposed setup");
      v.armed=true;
      for (unsigned mode:{0u,rover::rtl_mode}) {
        v.mode=mode;
        require(!boat->acquisitionAllowed(v,100)&&!boat->activeModeAllowed(v,true),
                "external mode accepted for boat acquisition");
      }
      v.mode=rover::hold_mode;
      require(boat->acquisitionAllowed(v,100)&&boat->acquisitionCommand(v,"hold",100),"boat Hold acquisition denied");
      v.mode=rover::loiter_mode;
      v.type=MAV_TYPE_GROUND_ROVER;
      require(!backend->acquisitionAllowed(v,100)&&!backend->acquisitionCommand(v,"ground",100),
              "ground profile allowed Loiter acquisition");
    });
    test("motion requires armed Guided without airborne requirement",[&]{
      auto v=fresh();v.armed=false;require(backend->flightReady(v,100,false)&&!backend->flightReady(v,100,true),"arming distinction");
      v.armed=true;v.mode=11;require(!backend->flightReady(v,100,true),"RTL accepted NAV");
    });
    test("native command allowlist rejects flight and force commands",[&]{
      auto v=fresh();CommandRequest r;r.command=MAV_CMD_NAV_TAKEOFF;require(!backend->commandAllowed(r,v,100),"TAKEOFF allowed");r.command=MAV_CMD_NAV_LAND;require(!backend->commandAllowed(r,v,100),"LAND allowed");
      r.command=MAV_CMD_COMPONENT_ARM_DISARM;r.params[0]=0;r.params[1]=21196;require(!backend->commandAllowed(r,v,100),"forced disarm allowed");
      r.params[1]=0;require(backend->commandAllowed(r,v,100),"normal disarm denied");r.params[0]=1;require(!backend->commandAllowed(r,v,100),"already armed ARM allowed");v.armed=false;require(backend->commandAllowed(r,v,100),"normal horizontal-ready ARM denied");
    });
    test("Rover mode values are not Copter aliases",[&]{
      auto v=fresh();CommandRequest r;r.command=MAV_CMD_DO_SET_MODE;r.params[0]=1;
      for(float mode:{4.f,15.f}){r.params[1]=mode;require(backend->commandAllowed(r,v,100),"supported Rover mode denied");}
      for(float mode:{5.f,6.f,9.f}){r.params[1]=mode;require(!backend->commandAllowed(r,v,100),"Copter or unsupported mode allowed");}
    });
    test("native boat Loiter requires fresh armed horizontal evidence",[&]{
      auto v=fresh();
      v.type=MAV_TYPE_SURFACE_BOAT;
      CommandRequest r;
      r.command=MAV_CMD_DO_SET_MODE;
      r.params[0]=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
      r.params[1]=rover::loiter_mode;
      require(boat->commandAllowed(r,v,100)&&!backend->commandAllowed(r,v,100),"fresh native Loiter misclassified");
      require(!boat->commandAllowed(r,v,100.6),"stale native Loiter allowed");
      v.armed=false;
      require(!boat->commandAllowed(r,v,100),"disarmed native Loiter allowed");
      r.params[1]=rover::hold_mode;
      require(boat->commandAllowed(r,v,100),"native Hold acquired the Loiter readiness restriction");
    });
    test("RTL requires bound home but course speed does not",[&]{
      auto v=fresh();CommandRequest r;r.command=MAV_CMD_NAV_RETURN_TO_LAUNCH;require(!backend->commandAllowed(r,v,100),"RTL without home allowed");
      v.home_valid=true;v.home_latitude=42;v.home_longitude=-71;require(backend->commandAllowed(r,v,100),"RTL with home denied");
    });
    test("input bounds and independent facing reject explicitly",[&]{
      auto n=target();require(backend->navigationInputAllowed(n),"valid planar target denied");
      for(double speed:{-1.,3.01,std::numeric_limits<double>::quiet_NaN()}){n=target(90,speed);require(!backend->navigationInputAllowed(n),"invalid speed accepted");}
      n=target();n.dimensions=NavigationDimensions::Spatial;require(!backend->navigationInputAllowed(n),"spatial target accepted");
      n=target();n.kind=NavigationKind::GlobalPosition;require(!backend->navigationInputAllowed(n),"unsupported global goto accepted");
      for(auto policy:{FacingPolicy::Keep,FacingPolicy::Heading}){n=target();n.facing=policy;require(!backend->navigationInputAllowed(n),"independent facing accepted");}
      require(backend->helmFieldAllowed(0,-720)&&backend->helmFieldAllowed(1,3)&&!backend->helmFieldAllowed(2,0),"helm quantity policy");
    });
    test("cardinal course speed becomes exact horizontal velocity",[&]{
      auto v=fresh();mavlink_status_t codec{};
      for(const auto& row:std::vector<std::array<double,3>>{{0,2,0},{90,0,2},{180,-2,0},{270,0,-2},{-270,0,2}}){
        const auto f=backend->navigationFrame(target(row[0]),v,100,1,codec);require(bool(f),"target encode failed");const auto p=decode(*f);
        close(p.vx,row[1],"north velocity");close(p.vy,row[2],"east velocity");require(p.coordinate_frame==MAV_FRAME_GLOBAL&&p.type_mask==0x0DE7,"wrong planar wire mask/frame");
        require(p.alt==0&&p.vz==0&&p.afx==0&&p.afy==0&&p.afz==0,"unrequested vertical/acceleration fields");}
    });
    test("true zero has no creep and preserves present steering heading",[&]{
      auto v=fresh();mavlink_status_t codec{};auto f=backend->navigationFrame(target(90,0),v,100,2,codec);require(bool(f),"zero denied");const auto p=decode(*f);
      require(p.vx==0&&p.vy==0&&p.vz==0&&p.type_mask==0x09E7,"epsilon creep or wrong zero mask");close(p.yaw,123*3.14159265358979323846/180,"zero heading");
    });
    test("original target expiry and manual mode deny frame construction",[&]{
      auto v=fresh();mavlink_status_t codec{};auto n=target();n.expires=100;require(!backend->navigationFrame(n,v,100,1,codec),"expired frame built");
      n=target();v.mode=0;require(!backend->navigationFrame(n,v,100,1,codec),"manual frame built");
    });
    test("loss stop is one horizontal zero target with no altitude/home",[&]{
      auto v=fresh();mavlink_status_t codec{};const auto f=backend->stopFrame(v,100,5,codec);require(bool(f),"eligible stop missing");const auto p=decode(*f);require(p.vx==0&&p.vy==0&&p.alt==0,"stop injects motion or height");
      v.position_at=99;require(!backend->stopFrame(v,100,6,codec),"stale stop constructed");
    });
    test("boat frames bound speed and send fresh armed Guided velocity or true zero",[&]{
      auto v=fresh();
      v.type=MAV_TYPE_SURFACE_BOAT;
      mavlink_status_t codec{};
      const double above=std::nextafter(rover::boat_maximum_speed,rover::maximum_speed);
      require(boat->helmFieldAllowed(1,rover::boat_maximum_speed)&&!boat->helmFieldAllowed(1,above)&&
              !boat->helmFieldAllowed(2,0),"boat helm field bounds");
      require(!boat->navigationFrame(target(90,above),v,100,1,codec),"overspeed boat frame constructed");
      const auto motion=boat->navigationFrame(target(270,rover::boat_maximum_speed),v,100,2,codec);
      require(bool(motion),"boat motion frame missing");
      const auto velocity=decode(*motion);
      close(velocity.vx,0,"boat north velocity");
      close(velocity.vy,-rover::boat_maximum_speed,"boat east velocity");
      require(velocity.coordinate_frame==MAV_FRAME_GLOBAL&&velocity.type_mask==0x0DE7&&
              velocity.alt==0&&velocity.vz==0,"wrong boat motion frame");
      const auto stop=boat->stopFrame(v,100,3,codec);
      require(bool(stop),"boat stop frame missing");
      const auto zero=decode(*stop);
      require(zero.type_mask==0x09E7&&zero.vx==0&&zero.vy==0&&zero.vz==0,"boat stop generated motion");
      close(zero.yaw,v.heading*3.14159265358979323846/180,"boat stop changed heading");
      for (unsigned mode:{rover::hold_mode,rover::loiter_mode,rover::rtl_mode}) {
        v.mode=mode;
        require(!boat->navigationFrame(target(),v,100,4,codec)&&!boat->stopFrame(v,100,4,codec),
                "boat frame constructed outside Guided");
      }
      v.mode=rover::guided_mode;
      require(!boat->navigationFrame(target(),v,100.5,5,codec),"expired boat frame constructed");
      require(!boat->stopFrame(v,100.6,5,codec),"stale boat stop frame constructed");
      v.armed=false;
      require(!boat->navigationFrame(target(),v,100,6,codec)&&!boat->stopFrame(v,100,6,codec),
              "disarmed boat frame constructed");
    });
    test("strict native parser rejects flight height force and duplicates",[&]{
      Fake s;auto p=s.configured(*backend);
      for(const auto& text:{"id=a,action=takeoff","id=b,action=land","id=c,action=hold,height=5","id=d,action=disarm,force=true","id=e,id=f,action=arm"})s.mail(*p,text);
      p->iterate();require(s.commands.empty()&&s.has("REJECTED"),"unsupported request submitted");
    });
    test("native Guided uses mode15 and ACK alone does not complete",[&]{
      Fake s;s.state.mode=4;auto p=s.configured(*backend);s.mail(*p,"id=guide,action=guided");p->iterate();
      require(s.commands.size()==1&&s.commands[0].params[1]==15,"wrong native Guided");
      s.sent(*p);s.update(100.15);ack(*p,MAV_CMD_DO_SET_MODE,MAV_RESULT_ACCEPTED,s.time);
      p->iterate();require(s.has("ACK_OBSERVED")&&!s.has("COMPLETED"),"ACK missing or completed mode change");
      s.update(100.2);s.state.mode=15;p->iterate();require(s.has("COMPLETED")&&s.finished==std::vector<std::string>{"guide"},"fresh observed Guided did not complete");
      s.mail(*p,"id=guide,action=guided",100.3);p->iterate();require(s.commands.size()==1&&s.has("DUPLICATE"),"duplicate resubmitted");
    });
    test("native ARM uses horizontal evidence without landed/home/height",[&]{
      Fake s;s.state.armed=false;auto p=s.configured(*backend);s.mail(*p,"id=arm,action=arm");p->iterate();require(s.commands.size()==1&&s.commands[0].params[0]==1&&s.commands[0].params[1]==0,"normal ARM not submitted");
      s.sent(*p);s.update(100.2);s.state.armed=true;p->iterate();require(s.has("COMPLETED"),"ARM fresh heartbeat outcome missing");
    });
    test("native Hold confirms mode then requires fresh low-speed evidence",[&]{
      Fake s;auto p=s.configured(*backend);s.mail(*p,"id=hold,action=hold");p->iterate();require(s.commands[0].params[1]==4,"wrong Hold mode");
      s.sent(*p);s.update(100.2);s.state.mode=4;s.state.north_velocity=1;p->iterate();require(s.has("CONFIRMED")&&!s.has("COMPLETED"),"moving hold completed");
      s.update(100.3);s.state.north_velocity=.1;p->iterate();require(s.has("COMPLETED"),"stopped Hold did not complete");
    });
    test("RTL home arrival is distinct from ACK/mode and takeover cancels",[&]{
      Fake s;s.state.home_valid=true;s.state.home_latitude=s.state.latitude;s.state.home_longitude=s.state.longitude;auto p=s.configured(*backend);
      s.mail(*p,"id=rtl,action=rtl");p->iterate();require(s.commands[0].command==MAV_CMD_NAV_RETURN_TO_LAUNCH,"wrong RTL command");
      s.sent(*p);s.update(100.15);ack(*p,MAV_CMD_NAV_RETURN_TO_LAUNCH,MAV_RESULT_ACCEPTED,s.time);
      s.update(100.2);s.state.mode=11;s.state.north_velocity=1;p->iterate();require(!s.has("COMPLETED"),"moving RTL completed");
      s.update(100.3);s.state.mode=0;p->iterate();require(s.has("CANCELLED")&&s.has("return_interrupted_by_mode"),"pilot takeover not cancelled");
    });
    test("RTL completes only stopped at bound home and home changes cancel",[&]{
      Fake s;s.state.home_valid=true;s.state.home_latitude=s.state.latitude;s.state.home_longitude=s.state.longitude;auto p=s.configured(*backend);s.mail(*p,"id=return,action=rtl");p->iterate();
      s.sent(*p);s.update(100.2);s.state.mode=11;p->iterate();require(s.has("COMPLETED"),"stopped home arrival not completed");
      s.mail(*p,"id=return2,action=rtl",100.3);p->iterate();s.update(100.4);s.state.home_latitude+=.001;p->iterate();require(s.has("home_reference_changed"),"changed home not cancelled");
    });
    test("RTL from Hold retains the original publication deadline",[&]{
      Fake s;s.state.mode=4;s.state.home_valid=true;s.state.home_latitude=s.state.latitude;s.state.home_longitude=s.state.longitude;
      auto p=s.configured(*backend);p->configure("rover_return_timeout","1");
      s.mail(*p,"id=return,action=rtl");s.update(100.3);p->iterate();
      require(s.commands.size()==1&&s.commands.back().operation_expires==101.1,
              "queueing renewed RTL lifetime");
      s.sent(*p);s.update(100.4);p->iterate();
      require(!s.has("CANCELLED")&&!s.has("CONFIRMED"),"unchanged starting mode interrupted RTL");
      s.update(s.commands.back().operation_expires);p->onInputBoundary();
      require(s.has("TIMED_OUT,reason=operation_deadline"),"original deadline did not retire RTL");
    });
    test("confirmed RTL cancels on external Guided as well as Manual",[&]{
      Fake s;s.state.home_valid=true;s.state.home_latitude=s.state.latitude;s.state.home_longitude=s.state.longitude;
      auto p=s.configured(*backend);s.mail(*p,"id=return,action=rtl");p->iterate();
      s.sent(*p);s.update(100.2);s.state.mode=11;s.state.north_velocity=1;p->iterate();require(s.has("CONFIRMED")&&!s.has("COMPLETED"),"RTL confirmation missing");
      s.update(100.3);s.state.mode=15;p->iterate();require(s.has("CANCELLED")&&s.has("return_interrupted_by_mode"),"Guided transition did not cancel confirmed RTL");
    });
    test("pilot mode cancels a pending Hold outcome without replay",[&]{
      Fake s;auto p=s.configured(*backend);s.mail(*p,"id=hold,action=hold");p->iterate();
      s.sent(*p);s.update(100.2);s.state.mode=4;s.state.north_velocity=1;p->iterate();require(s.has("CONFIRMED")&&!s.has("COMPLETED"),"Hold confirmation missing");
      s.update(100.3);s.state.mode=0;p->iterate();require(s.has("CANCELLED")&&s.has("hold_interrupted_by_mode"),"pilot mode did not cancel pending Hold");
      s.update(100.4);s.state.mode=4;s.state.north_velocity=0;p->iterate();require(!s.has("COMPLETED")&&s.commands.size()==1,"cancelled Hold replayed");
    });
    test("late denial for retired Hold cannot fail a newer Guided",[&]{
      Fake s;auto p=s.configured(*backend);
      s.mail(*p,"id=old,action=hold");p->iterate();s.sent(*p);
      s.update(104.2);p->onInputBoundary();
      require(s.has("id=old,action=hold,status=TIMED_OUT"),"old request did not retire");
      s.state.mode=4;
      s.mail(*p,"id=new,action=guided",104.3);p->iterate();s.sent(*p);
      s.update(104.4);ack(*p,MAV_CMD_DO_SET_MODE,MAV_RESULT_DENIED,s.time);
      require(s.has("status=ACK_OBSERVED,reason=uncorrelated_ack_result_2")&&!s.has("status=FAILED"),
              "denial decided a newer request");
      s.update(104.5);s.state.mode=15;p->iterate();
      require(s.has("id=new,action=guided,status=COMPLETED")&&s.commands.size()==2,
              "fresh Guided outcome did not close exactly once");
    });
    test("unbound ACKs cannot renew observation deadlines or revive operations",[&]{
      for(const uint8_t result:{MAV_RESULT_ACCEPTED,MAV_RESULT_IN_PROGRESS,MAV_RESULT_DENIED}) {
        Fake s;auto p=s.configured(*backend);
        s.mail(*p,"id=timeout,action=hold");p->iterate();s.sent(*p);
        const auto deadline=s.time+4;
        s.update(deadline-.01);ack(*p,MAV_CMD_DO_SET_MODE,result,s.time);
        require(!s.has("status=FAILED")&&!s.has("status=COMPLETED")&&!s.has("status=TIMED_OUT"),
                "an ACK became a terminal outcome");
        s.update(deadline);p->onInputBoundary();
        require(s.has("status=TIMED_OUT,reason=command_observation_unavailable"),"original observation deadline moved");
        ack(*p,MAV_CMD_DO_SET_MODE,MAV_RESULT_ACCEPTED,s.time);p->iterate();
        require(s.commands.size()==1&&!s.has("COMPLETED"),"late ACK retried or revived");
        s.mail(*p,"id=reset,action=hold",deadline+.1);p->iterate();p->onReset("vehicle_connection_lost");
        require(s.has("CANCELLED")&&s.finished.back()=="reset","reset did not retire lease");
      }
    });
    test("native retained expired and host-inhibited requests never execute",[&]{
      Fake s;auto p=s.configured(*backend);Mail m;m.key="MAVMOOS_COMMAND";m.text="id=old,action=hold";m.received_at=100;m.published_at=99;p->onMail(m);p->iterate();require(s.commands.empty()&&s.has("pre_session_request"),"retained request executed");
      s.admit=false;s.mail(*p,"id=blocked,action=hold");p->iterate();require(s.has("host_send_inhibited"),"host rejection hidden");
    });
    test("native DISARM completion needs a new heartbeat but no flight state",[&]{
      Fake s;auto p=s.configured(*backend);s.mail(*p,"id=disarm,action=disarm");p->iterate();require(s.commands.size()==1&&s.commands[0].params[0]==0,"normal DISARM absent");
      s.sent(*p);s.state.armed=false;p->iterate();require(!s.has("COMPLETED"),"cached pre-command heartbeat completed DISARM");
      s.update(100.2);p->iterate();require(s.has("COMPLETED"),"new disarmed HB did not complete");
    });
    std::cout<<"PASS "<<count<<" Rover backend/transaction groups; fake services only\n";return 0;
  }catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
