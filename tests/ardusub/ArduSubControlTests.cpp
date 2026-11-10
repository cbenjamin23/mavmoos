#include "Runtime.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace mavmoos::runtime;
namespace {
constexpr unsigned guided=4, position_hold=16, manual=19;
constexpr auto pressure=MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;

void require(bool condition,const std::string& message)
{
  if(!condition)throw std::runtime_error(message);
}

struct Rig {
  double now=10;
  unsigned mode=position_hold;
  bool healthy=true,armed=true,cancel_next_target=false;
  bool refuse_next_mode=false;
  int32_t home=10000,position_msl=8750;
  std::vector<mavlink_message_t> sent;
  std::vector<std::pair<std::string,std::string>> publications;
  Runtime host;

  explicit Rig(const char* module,bool shadow=false):
    host([this](const auto& key,const auto& text,double value,bool number) {
      publications.emplace_back(key,number?std::to_string(value):text);
    },[](const auto*,std::size_t){return false;},true,shadow,[this]{return now;},
      MAV_TYPE_SUBMARINE,MAV_AUTOPILOT_ARDUPILOTMEGA)
  {
    require(host.load(module,"ardusub_control"),host.lastConfigurationError());
    host.configure("vehicle_profile","ardusub");
    host.configure("control_native_actions","false");
    host.configure("control_helm","true");
    host.configure("shadow_mode",shadow?"true":"false");
    host.configure("freshness_timeout","0.5");
    host.setFirmwareRequestsEnabled(false);
    host.setBoundedSend([this](const uint8_t* bytes,std::size_t size,double,const Runtime::SendEligibility& gate) {
      mavlink_message_t message{},buffer{};
      mavlink_status_t state{},status{};
      bool decoded=false;
      for(std::size_t i=0;i<size;++i)
        decoded=mavlink_frame_char_buffer(&buffer,&state,bytes[i],&message,&status)==MAVLINK_FRAMING_OK;
      require(decoded && message.sysid==245 && message.compid==191,"bad ArduSub wire frame");
      if(cancel_next_target && message.msgid==MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) {
        cancel_next_target=false;
        host.revokeNavigation("test_socket_boundary");
      }
      if(!gate(now))return false;
      if(refuse_next_mode && message.msgid==MAVLINK_MSG_ID_COMMAND_LONG) {
        mavlink_command_long_t command{};
        mavlink_msg_command_long_decode(&message,&command);
        if(command.command==MAV_CMD_DO_SET_MODE) {
          refuse_next_mode=false;
          return false;
        }
      }
      sent.push_back(message);
      return true;
    });
  }

  void heartbeat(double effective=-1,uint8_t system=1,uint8_t component=1)
  {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(system,component,&message,MAV_TYPE_SUBMARINE,MAV_AUTOPILOT_ARDUPILOTMEGA,
      MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|(armed?MAV_MODE_FLAG_SAFETY_ARMED:0),mode,MAV_STATE_ACTIVE);
    host.message(message,effective<0?now:effective,now);
  }

  void observations(bool heartbeat_now=true)
  {
    if(heartbeat_now)heartbeat();
    mavlink_message_t message{};
    const float q[]{1,0,0,0};
    mavlink_msg_home_position_pack(1,1,&message,423584560,-710875890,home,0,0,0,q,0,0,0,1);
    host.message(message,now);
    // The Sub wire relative_alt convention is intentionally incompatible.
    // Motion depth must use home MSL minus current MSL instead.
    mavlink_msg_global_position_int_pack(1,1,&message,static_cast<uint32_t>(now*1000),
      423584570,-710875880,position_msl,5000,0,0,0,0);
    host.message(message,now);
    mavlink_msg_ekf_status_report_pack(1,1,&message,51,0,0,0,0,0,0);
    host.message(message,now);
    mavlink_sys_status_t status{};
    status.onboard_control_sensors_present=status.onboard_control_sensors_enabled=pressure;
    if(healthy)status.onboard_control_sensors_health=pressure;
    mavlink_msg_sys_status_encode(1,1,&message,&status);
    host.message(message,now);
  }

  void mail(const char* key,const char* value,const char* source="operator")
  { host.mail({key,value,source,0,now,now,false}); }

  void field(const char* key,double value,const char* source)
  { host.mail({key,"",source,value,now,now,true}); }

  void prepare()
  {
    observations();
    mail("IVPHELM_STATE","DRIVE","pHelmIvP");
    mail("IVPHELM_ALLSTOP","clear","pHelmIvP");
  }

  void helm(bool depth=true,const char* source="pHelmIvP")
  {
    mail("IVPHELM_STATE","DRIVE",source);
    field("DESIRED_HEADING",90,source);
    field("DESIRED_SPEED",.7,source);
    field(depth?"DESIRED_DEPTH":"DESIRED_ALTITUDE",2,source);
  }

  void tick(bool feed=true,bool depth=true,const char* source="pHelmIvP")
  {
    now+=.11;
    observations();
    if(feed)helm(depth,source);
    host.tick(now);
  }

  std::size_t count(unsigned message) const
  { return std::count_if(sent.begin(),sent.end(),[&](const auto& packet){return packet.msgid==message;}); }

  std::size_t localFrames(unsigned frame) const
  {
    return std::count_if(sent.begin(),sent.end(),[&](const auto& packet){
      if(packet.msgid!=MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED)return false;
      mavlink_set_position_target_local_ned_t target{};
      mavlink_msg_set_position_target_local_ned_decode(&packet,&target);
      return target.coordinate_frame==frame;
    });
  }

  std::size_t modeRequests(unsigned desired) const
  {
    return std::count_if(sent.begin(),sent.end(),[&](const auto& packet){
      if(packet.msgid!=MAVLINK_MSG_ID_COMMAND_LONG)return false;
      mavlink_command_long_t command{};
      mavlink_msg_command_long_decode(&packet,&command);
      if(command.command!=MAV_CMD_DO_SET_MODE)return false;
      require(command.param1==1 && command.target_system==1 && command.target_component==1 &&
              command.confirmation==0,"unscoped Sub mode command");
      return command.param2==desired;
    });
  }

  const mavlink_message_t& latest(unsigned id) const
  {
    const auto item=std::find_if(sent.rbegin(),sent.rend(),[&](const auto& packet){return packet.msgid==id;});
    require(item!=sent.rend(),"missing expected Sub wire frame");
    return *item;
  }

  bool published(const std::string& key,const std::string& fragment) const
  {
    return std::any_of(publications.begin(),publications.end(),[&](const auto& item){
      return item.first==key && item.second.find(fragment)!=std::string::npos;
    });
  }

  void acquire()
  {
    const auto before=modeRequests(guided);
    const auto motion=localFrames(MAV_FRAME_LOCAL_NED);
    mail("MAVMOOS_NAV_CONTROL","acquire");
    tick(false);
    require(modeRequests(guided)==before+1 && localFrames(MAV_FRAME_LOCAL_NED)==motion,
            "Sub did not request exactly one Guided entry from Position Hold");
  }

  void startGuided()
  {
    prepare();acquire();mode=guided;tick();tick();
    require(host.navigationGranted() && localFrames(MAV_FRAME_LOCAL_NED)>0,
            "Sub fixture did not enter observed Guided with fresh inputs");
  }
};

void workflow(const char* module)
{
  Rig rig(module);rig.prepare();
  const auto subscriptions=rig.host.subscriptions();
  require(std::find(subscriptions.begin(),subscriptions.end(),"DESIRED_DEPTH")!=subscriptions.end() &&
          std::find(subscriptions.begin(),subscriptions.end(),"DESIRED_ALTITUDE")==subscriptions.end(),
          "Sub did not select the stock depth decision variable");
  rig.acquire();
  const double handed_off=rig.now;
  rig.mode=guided;
  rig.now+=.12;rig.heartbeat(handed_off-.01);rig.observations(false);rig.helm(false);rig.host.tick(rig.now);
  require(rig.localFrames(MAV_FRAME_LOCAL_NED)==0,
          "buffered pre-send Guided observation conferred motion");
  rig.tick(true,false);
  require(rig.localFrames(MAV_FRAME_LOCAL_NED)==0,"altitude mail supplied Sub depth");
  rig.tick(true,true,"other_helm");
  require(rig.localFrames(MAV_FRAME_LOCAL_NED)==0,"non-helm mail supplied Sub depth");
  rig.tick();rig.tick();
  mavlink_set_position_target_local_ned_t motion{};
  mavlink_msg_set_position_target_local_ned_decode(&rig.latest(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED),&motion);
  require(motion.coordinate_frame==MAV_FRAME_LOCAL_NED && motion.target_system==1 && motion.target_component==1 &&
          (motion.type_mask&7)==7 && (motion.type_mask&56)==0 && std::abs(motion.vx)<.001 &&
          std::abs(motion.vy-.7)<.001 && std::abs(motion.vz-.4)<.001,
          "wrong east/down velocity at final handoff");

  auto before=rig.localFrames(MAV_FRAME_LOCAL_NED);
  rig.now+=.03;rig.mail("IVPHELM_ALLSTOP","NothingToDo","pHelmIvP");rig.tick(false);
  mavlink_set_position_target_local_ned_t hold{};
  mavlink_msg_set_position_target_local_ned_decode(&rig.latest(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED),&hold);
  require(hold.coordinate_frame==MAV_FRAME_LOCAL_OFFSET_NED && hold.type_mask==3576 &&
          hold.target_system==1 && hold.target_component==1 &&
          hold.x==0 && hold.y==0 && hold.z==0 &&
          rig.count(MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT)==0 &&
          rig.localFrames(MAV_FRAME_LOCAL_NED)==before,
          "bounded normal pause did not capture current FC position and depth");
  rig.now+=.03;rig.mail("IVPHELM_ALLSTOP","clear","pHelmIvP");rig.tick();rig.tick();
  require(rig.localFrames(MAV_FRAME_LOCAL_NED)>before && rig.modeRequests(guided)==1,
          "normal pause did not resume the confirmed mode");
  rig.now+=.03;
  rig.mail("MAVMOOS_SUB_HANDBACK","loiter");
  rig.host.mail({"MAVMOOS_SUB_HANDBACK","hold","operator",0,rig.now-.6,rig.now,false});
  rig.host.mail({"MAVMOOS_SUB_HANDBACK","hold","operator",0,rig.now,rig.now,true});
  require(rig.host.navigationGranted() && !rig.published("MAVMOOS_SUB_HANDBACK_RESULT","status=QUEUED"),
          "unsupported, expired or numeric handback mail retired valid control");
  rig.mail("MAVMOOS_SUB_HANDBACK","hold");
  before=rig.localFrames(MAV_FRAME_LOCAL_NED);
  require(rig.modeRequests(position_hold)==0 && !rig.host.navigationGranted() &&
          rig.published("MAVMOOS_SUB_HANDBACK_RESULT","status=QUEUED"),
          "handback admission did not fence motion before sending the mode request");
  rig.now+=.02;rig.mail("MAVMOOS_SUB_HANDBACK","hold");
  rig.tick();
  require(rig.modeRequests(position_hold)==1 && !rig.host.navigationGranted() &&
          rig.published("MAVMOOS_SUB_HANDBACK_RESULT","status=SENT") &&
          !rig.published("MAVMOOS_SUB_HANDBACK_RESULT","status=CONFIRMED"),
          "explicit handback did not fence helm or misreported socket acceptance");
  const auto sent_at=rig.now;
  mavlink_message_t ack{};
  mavlink_msg_command_ack_pack(1,1,&ack,MAV_CMD_DO_SET_MODE,MAV_RESULT_ACCEPTED,0,0,245,191);
  rig.now+=.02;rig.host.message(ack,rig.now);
  rig.mode=position_hold;
  rig.now+=.02;rig.heartbeat(-1,1,42);
  rig.now+=.02;rig.heartbeat(sent_at);
  require(!rig.published("MAVMOOS_SUB_HANDBACK_RESULT","status=CONFIRMED"),
          "an ACK, other component or buffered mode heartbeat confirmed handback");
  rig.tick();
  require(rig.published("MAVMOOS_SUB_HANDBACK_RESULT","status=CONFIRMED") &&
          rig.modeRequests(position_hold)==1 && rig.localFrames(MAV_FRAME_LOCAL_NED)==before,
          "fresh native Position Hold did not confirm the single handback");
}

void operatorRelease(const char* module)
{
  Rig pending(module);pending.prepare();pending.now+=.02;
  pending.mail("MAVMOOS_NAV_CONTROL","acquire");
  require(pending.host.navigationGranted() && pending.modeRequests(guided)==0,
          "release fixture did not queue a Guided request");
  pending.now+=.02;pending.mail("MAVMOOS_NAV_CONTROL","release");
  pending.tick();pending.mode=guided;pending.tick();pending.tick();
  require(!pending.host.navigationGranted() && pending.modeRequests(guided)==0 &&
          pending.count(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED)==0,
          "release before Guided handoff left a mode request or target");

  Rig active(module);active.startGuided();
  const auto motion=active.localFrames(MAV_FRAME_LOCAL_NED);
  active.now+=.02;active.mail("MAVMOOS_NAV_CONTROL","release");
  active.now+=.02;active.mail("MAVMOOS_NAV_CONTROL","release");
  active.tick();active.tick();
  active.now+=.02;active.mail("MAVMOOS_NAV_CONTROL","acquire");
  active.tick();
  require(!active.host.navigationGranted() && active.localFrames(MAV_FRAME_LOCAL_OFFSET_NED)==1 &&
          active.localFrames(MAV_FRAME_LOCAL_NED)==motion && active.modeRequests(guided)==1,
          "release lost its single capture or restored control from Guided");
  active.mode=position_hold;active.tick();
  require(!active.host.navigationGranted() && active.localFrames(MAV_FRAME_LOCAL_NED)==motion,
          "external Position Hold restored a released grant");
  active.now+=.02;active.acquire();
  active.mode=guided;active.tick();
  require(active.localFrames(MAV_FRAME_LOCAL_NED)==motion,
          "reacquisition used helm inputs from before the fresh Guided boundary");
  active.tick();
  require(active.host.navigationGranted() && active.localFrames(MAV_FRAME_LOCAL_NED)>motion,
          "explicit reacquisition and fresh depth tuple did not restore control");
}

void handbackFailures(const char* module)
{
  constexpr auto result="MAVMOOS_SUB_HANDBACK_RESULT";
  Rig unowned(module);unowned.prepare();unowned.now+=.02;
  unowned.mail("MAVMOOS_SUB_HANDBACK","hold");unowned.tick();
  require(unowned.modeRequests(position_hold)==0 &&
          unowned.published(result,"status=REJECTED,reason=owned_guided_required"),
          "unowned handback did not reject locally");

  Rig transport(module);transport.startGuided();
  const auto motion=transport.localFrames(MAV_FRAME_LOCAL_NED);
  transport.refuse_next_mode=true;
  transport.now+=.02;transport.mail("MAVMOOS_SUB_HANDBACK","hold");
  transport.tick();transport.tick();
  require(!transport.refuse_next_mode && transport.modeRequests(position_hold)==0 &&
          transport.published(result,"status=FAILED,reason=socket_not_accepted") &&
          !transport.published(result,"status=SENT") && !transport.host.navigationGranted() &&
          transport.localFrames(MAV_FRAME_LOCAL_NED)==motion,
          "refused handback was retried, reported as sent or returned authority to the helm");

  Rig external(module);external.startGuided();external.now+=.02;
  external.mail("MAVMOOS_SUB_HANDBACK","hold");external.tick();
  external.mode=manual;external.tick();
  require(external.modeRequests(position_hold)==1 &&
          external.published(result,"status=PREEMPTED,reason=external_mode_or_disarm") &&
          !external.published(result,"status=CONFIRMED") && !external.host.navigationGranted(),
          "external Manual during handback did not preempt the pending result");

  Rig timeout(module);timeout.startGuided();timeout.now+=.02;
  timeout.mail("MAVMOOS_SUB_HANDBACK","hold");timeout.tick();
  for(unsigned i=0;i<27;++i)timeout.tick();
  timeout.mode=position_hold;timeout.tick();
  require(timeout.modeRequests(position_hold)==1 &&
          timeout.published(result,"status=UNCONFIRMED,reason=confirmation_timeout") &&
          !timeout.published(result,"status=CONFIRMED") && !timeout.host.navigationGranted(),
          "expired handback retried or treated a late mode as completion");
}

void nativeSurface(const char* module)
{
  constexpr auto request="MAVMOOS_SUB_HANDBACK",result="MAVMOOS_SUB_HANDBACK_RESULT";
  constexpr unsigned surface=9,depth_hold=2;
  Rig active(module);active.startGuided();active.now+=.02;
  const auto motion=active.localFrames(MAV_FRAME_LOCAL_NED);
  active.mail(request,"surface");active.tick();
  require(active.modeRequests(surface)==1 && !active.host.navigationGranted() &&
          active.published(result,"status=SENT,reason=waiting_for_native_surface"),
          "surface did not retire the helm grant and send one addressed mode request");
  active.mode=surface;active.heartbeat(-1,1,2);
  require(!active.published(result,"status=CONFIRMED"),"foreign Surface confirmed the request");
  active.tick();
  require(active.published(result,"status=CONFIRMED,reason=native_surface_observed") &&
          !active.published(result,"status=COMPLETED"),"mode entry and completed ascent were conflated");
  active.mode=depth_hold;
  for(unsigned i=0;i<7;++i)active.tick();
  require(!active.published(result,"status=COMPLETED"),"deep native Depth Hold completed the ascent");
  active.position_msl=9850;
  for(unsigned i=0;i<7;++i)active.tick();
  require(active.published(result,"status=COMPLETED,reason=native_surface_and_shallow_depth_hold_observed") &&
          active.modeRequests(surface)==1 && active.localFrames(MAV_FRAME_LOCAL_NED)==motion &&
          !active.host.navigationGranted(),"fresh shallow dwell did not complete one native ascent without resuming helm");

  Rig takeover(module);takeover.startGuided();takeover.now+=.02;
  takeover.mail(request,"surface");takeover.mode=depth_hold;takeover.tick();
  require(takeover.modeRequests(surface)==0 && takeover.published(result,"status=FAILED") &&
          !takeover.host.navigationGranted(),"new native mode did not fence the queued Surface request");

  Rig rejected(module);rejected.mode=manual;rejected.prepare();rejected.now+=.02;
  rejected.mail(request,"surface");rejected.tick();
  require(rejected.modeRequests(surface)==0 && rejected.published(result,"status=REJECTED"),
          "Surface was admitted during Manual takeover");

  Rig timeout(module);timeout.mode=depth_hold;timeout.prepare();timeout.now+=.02;
  timeout.mail(request,"surface");timeout.tick();
  for(unsigned i=0;i<29;++i)timeout.tick();
  require(timeout.modeRequests(surface)==1 && timeout.published(result,"status=UNCONFIRMED") &&
          !timeout.published(result,"status=CONFIRMED"),"missing native Surface evidence retried or reported ascent");
}

void faults(const char* module)
{
  Rig pressure_fault(module);pressure_fault.healthy=false;pressure_fault.prepare();
  pressure_fault.mail("MAVMOOS_NAV_CONTROL","acquire");pressure_fault.tick();
  require(!pressure_fault.host.navigationGranted() && pressure_fault.modeRequests(guided)==0,
          "bad depth sensor admitted Guided");
  Rig pressure_loss(module);pressure_loss.prepare();pressure_loss.acquire();pressure_loss.mode=guided;
  pressure_loss.tick();pressure_loss.tick();
  const auto pressure_motion=pressure_loss.localFrames(MAV_FRAME_LOCAL_NED);
  require(pressure_motion>0,"pressure-loss fixture never achieved Guided motion");
  pressure_loss.healthy=false;pressure_loss.tick();
  require(!pressure_loss.host.navigationGranted() &&
          pressure_loss.localFrames(MAV_FRAME_LOCAL_NED)==pressure_motion &&
          pressure_loss.localFrames(MAV_FRAME_LOCAL_OFFSET_NED)==0,
          "loss of the depth sensor permitted motion or a capture with invalid evidence");
  pressure_loss.healthy=true;pressure_loss.tick();pressure_loss.tick();
  require(pressure_loss.localFrames(MAV_FRAME_LOCAL_NED)==pressure_motion,
          "recovered pressure evidence revived a retired grant");
  Rig takeover(module);takeover.prepare();takeover.acquire();takeover.mode=guided;takeover.tick();takeover.tick();
  const auto last=takeover.localFrames(MAV_FRAME_LOCAL_NED);
  takeover.mode=manual;takeover.tick();takeover.tick();
  require(!takeover.host.navigationGranted() && takeover.localFrames(MAV_FRAME_LOCAL_NED)==last &&
          takeover.localFrames(MAV_FRAME_LOCAL_OFFSET_NED)==0,
          "external takeover still permitted setpoints");
  Rig stale(module);stale.prepare();stale.acquire();stale.mode=guided;stale.tick();stale.tick();
  for(unsigned i=0;i<9;++i)stale.tick(false);
  require(!stale.host.navigationGranted() && stale.localFrames(MAV_FRAME_LOCAL_OFFSET_NED)==1 &&
          stale.count(MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT)==0,
          "expired helm lease did not retire once into a position/depth hold");
  const auto before=stale.localFrames(MAV_FRAME_LOCAL_NED);
  stale.tick();stale.tick();
  require(stale.localFrames(MAV_FRAME_LOCAL_NED)==before,"recovered helm mail reused expired grant");
  Rig final_gate(module);final_gate.prepare();final_gate.acquire();final_gate.mode=guided;
  final_gate.cancel_next_target=true;final_gate.tick();final_gate.tick();
  require(final_gate.localFrames(MAV_FRAME_LOCAL_NED)==0,"final-boundary retirement leaked motion");
  Rig shadow(module,true);shadow.prepare();shadow.mail("MAVMOOS_NAV_CONTROL","acquire");
  shadow.tick();shadow.tick();
  require(shadow.sent.empty() && shadow.published("MAVMOOS_SHADOW_DEPTH","2.000000"),
          "Sub shadow mode did not preserve depth intent without effects");
}
}

int main(int argc,char** argv)
{
  try {
    require(argc==2,"usage: ardusub_control_tests MODULE");
    workflow(argv[1]);operatorRelease(argv[1]);handbackFailures(argv[1]);nativeSurface(argv[1]);faults(argv[1]);
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<error.what()<<'\n';
    return 1;
  }
}
