#include "Runtime.h"
#include "PlaneOperationPolicy.h"
#include "plane_provider/PlaneFixture.h"
#include <algorithm>
#include <array>
#include <cstring>
#include <functional>
#include <string_view>

using namespace plane_fixture;

namespace {
struct Parameter { const char* name; uint8_t type; float value; };
constexpr std::array<Parameter,16> parameters{{
  {"Q_ENABLE",2,0},{"SYSID_MYGCS",4,245},{"FS_GCS_ENABL",2,1},{"FS_LONG_ACTN",2,1},
  {"FS_LONG_TIMEOUT",9,5},{"RTL_AUTOLAND",2,0},{"RTL_ALTITUDE",9,65},
  {"RTL_RADIUS",4,0},{"WP_LOITER_RAD",4,80},{"AIRSPEED_CRUISE",9,22},
  {"AIRSPEED_MIN",4,10},{"AIRSPEED_MAX",4,30},{"ARSPD_USE",2,1},
  {"ARSPD_TYPE",2,100},{"ARSPD_PRIMARY",2,0},{"ARSPD2_TYPE",2,0}
}};

class Rig {
public:
  explicit Rig(const char* path,bool simulation=true,bool shadow=false)
    :runtime([this](const auto& key,const auto& value,double,bool numeric){
               if(!numeric) publications.emplace_back(key,value);
             },[](const auto*,std::size_t){return false;},
             simulation,shadow,{},MAV_TYPE_FIXED_WING,MAV_AUTOPILOT_ARDUPILOTMEGA)
  {
    require(runtime.load(path,"arduplane_control"),"cannot load real Plane provider");
    runtime.configure("shadow_mode",shadow?"true":"false");
    runtime.setBoundedSend([this](const uint8_t* bytes,std::size_t size,double,const Runtime::SendEligibility& eligible) {
      mavlink_message_t packet{},buffer{};
      mavlink_status_t codec{},result{};
      bool decoded=false;
      for(std::size_t i=0;i<size;++i)
        decoded=mavlink_frame_char_buffer(&buffer,&codec,bytes[i],&packet,&result)==MAVLINK_FRAMING_OK;
      require(decoded&&packet.sysid==245&&packet.compid==191,"unexpected wire framing");
      if(before_send) before_send(packet);
      if(!eligible(clock)) return false;
      sent.push_back(packet);
      return true;
    });
  }

  void heartbeat(unsigned selected,double effective=-1)
  {
    mavlink_message_t packet{};
    mavlink_msg_heartbeat_pack(1,1,&packet,MAV_TYPE_FIXED_WING,MAV_AUTOPILOT_ARDUPILOTMEGA,
      MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|MAV_MODE_FLAG_SAFETY_ARMED,selected,MAV_STATE_ACTIVE);
    runtime.message(packet,effective<0?clock:effective,clock,"QUEUE_EMPTY_FLOOR");
  }

  void update()
  {
    heartbeat(mode);
    mavlink_message_t packet{};
    mavlink_msg_global_position_int_pack(1,1,&packet,1,430000000,-710000000,200000+relative_mm,relative_mm,2200,0,0,0);
    runtime.message(packet,clock);
    const float q[]{1,0,0,0};
    mavlink_msg_home_position_pack(1,1,&packet,430000000,-710000000,200000,0,0,0,q,0,0,0,0);
    runtime.message(packet,clock);
    mavlink_msg_ekf_status_report_pack(1,1,&packet,55,.1,.1,.1,.1,0,0);
    runtime.message(packet,clock);
    runtime.message(state(),clock);
    runtime.message(sensor(),clock);
  }

  void step(double delta=.05)
  {
    clock+=delta;
    update();
    runtime.tick(clock);
  }

  void text(const std::string& key,const std::string& value,const std::string& source="operator")
  {
    runtime.mail({key,value,source,0,clock,clock,false});
  }

  void helm(double course=90)
  {
    const std::string source="pHelmIvP";
    text("IVPHELM_STATE","DRIVE",source);
    text("IVPHELM_ALLSTOP","clear",source);
    runtime.mail({"DESIRED_HEADING","",source,course,clock,clock,true});
    runtime.mail({"DESIRED_SPEED","",source,22,clock,clock,true});
    runtime.mail({"DESIRED_ALTITUDE","",source,75,clock,clock,true});
  }

  bool commandResult(const std::string& id,const std::string& status) const
  {
    return std::any_of(publications.begin(),publications.end(),[&](const auto& report) {
      return report.first=="MAVMOOS_COMMAND_RESULT" &&
        report.second.find(",id="+id+",")!=std::string::npos &&
        report.second.find(",status="+status+",")!=std::string::npos;
    });
  }

  bool directResult(const std::string& id,const std::string& status) const
  {
    return std::any_of(publications.begin(),publications.end(),[&](const auto& report) {
      return report.first=="MAVMOOS_CONTROL_RESULT"&&report.second.find(",id="+id+",")!=std::string::npos&&
        report.second.find(",status="+status+",")!=std::string::npos;
    });
  }

  void qualify(bool mismatch=false)
  {
    for(std::size_t i=0;i<parameters.size();++i) {
      const auto first=sent.size();
      for(int tick=0;tick<12 && std::none_of(sent.begin()+first,sent.end(),[](const auto& p) {
          return p.msgid==MAVLINK_MSG_ID_PARAM_REQUEST_READ;
        });++tick) step();
      const auto request=std::find_if(sent.begin()+first,sent.end(),[](const auto& p) {
        return p.msgid==MAVLINK_MSG_ID_PARAM_REQUEST_READ;
      });
      if(request==sent.end()) throw std::runtime_error("pinned profile read missing: "+runtime.status());
      mavlink_param_request_read_t read{};
      mavlink_msg_param_request_read_decode(&*request,&read);
      const auto& expected=parameters[i];
      require(read.param_index==-1 && std::string_view(read.param_id,std::strlen(expected.name))==expected.name,
        "profile read for unexpected key");
      const double before_send=clock-.01;
      clock+=.01;
      mavlink_message_t value{};
      mavlink_msg_param_value_pack(1,1,&value,read.param_id,expected.value+(mismatch?1:0),
        expected.type,100,UINT16_MAX);
      // The queue's conservative age floor can precede a quick reply's request.
      runtime.message(value,before_send,clock,"QUEUE_EMPTY_FLOOR");
      if(mismatch) break;
    }
    step();
    require(has(mismatch?"plane_profile=MISMATCH":"plane_profile=VERIFIED"),"profile values not reconciled");
  }

  void acquire()
  {
    require(runtime.acquireNavigation("initial"),"qualified Loiter acquisition rejected");
    step();
    require(countMode(plane::guided_mode)==1,"expected exactly one Guided mode request");
    mode=plane::guided_mode;
    clock+=.05;
    heartbeat(mode,clock-.06);
    require(has("plane_guided_confirmed=0"),"pre-handoff observation confirmed Guided");
    step();
    require(has("plane_guided_confirmed=1"),"fresh post-handoff Guided missing");
  }

  mavlink_command_int_t point()
  {
    require(runtime.navigation(intent(clock+1.2)),"fresh Plane intent rejected");
    step(.15);
    return lastPoint();
  }

  mavlink_command_int_t lastPoint() const
  {
    const auto last=std::find_if(sent.rbegin(),sent.rend(),[](const auto& p){
      return p.msgid==MAVLINK_MSG_ID_COMMAND_INT;
    });
    require(last!=sent.rend(),"Plane point missing");
    mavlink_command_int_t target{};
    mavlink_msg_command_int_decode(&*last,&target);
    require(target.command==MAV_CMD_DO_REPOSITION&&target.frame==MAV_FRAME_GLOBAL_RELATIVE_ALT_INT&&target.param2==0,
      "point changed mode or wire format");
    return target;
  }

  void readback(const mavlink_command_int_t& target,int32_t latitude_offset=0)
  {
    clock+=.04;
    mavlink_message_t packet{};
    mavlink_msg_position_target_global_int_pack(1,1,&packet,1,MAV_FRAME_GLOBAL,0xFDF8,
      target.x+latitude_offset,target.y,200+target.z,0,0,0,0,0,0,0,0);
    runtime.message(packet,clock);
  }

  std::size_t count(unsigned id) const
  {
    return std::count_if(sent.begin(),sent.end(),[id](const auto& p){return p.msgid==id;});
  }

  std::size_t countMode(unsigned destination) const
  {
    return std::count_if(sent.begin(),sent.end(),[destination](const auto& packet){
      if(packet.msgid!=MAVLINK_MSG_ID_COMMAND_LONG) return false;
      mavlink_command_long_t p{};
      mavlink_msg_command_long_decode(&packet,&p);
      return p.command==MAV_CMD_DO_SET_MODE && p.param2==destination;
    });
  }

  bool has(const std::string& text) const { return runtime.status().find(text)!=std::string::npos; }

  std::vector<mavlink_message_t> sent;
  std::vector<std::pair<std::string,std::string>> publications;
  std::function<void(const mavlink_message_t&)> before_send;
  double clock=10;
  unsigned mode=plane::loiter_mode;
  int32_t relative_mm=75000;
  Runtime runtime;
};

void testProfile(const char* path)
{
  Rig wrong(path);
  wrong.qualify(true);
  require(wrong.count(MAVLINK_MSG_ID_HEARTBEAT)==0&&!wrong.runtime.acquireNavigation("bad"),
    "mismatched profile activated the watchdog or authority");
  for(const bool simulation:{false,true}) {
    Rig passive(path,simulation,true);
    passive.step();
    require(passive.sent.empty()&&!passive.has("plane_watchdog=ENABLED"),"passive Plane produced egress");
  }
  Rig restart(path);
  restart.mode=plane::guided_mode;
  restart.qualify();
  require(restart.count(MAVLINK_MSG_ID_HEARTBEAT)==0&&!restart.runtime.acquireNavigation("restart"),
    "restart in retained Guided sustained watchdog or acquired authority");
  restart.mode=plane::loiter_mode;
  restart.step();
  require(restart.count(MAVLINK_MSG_ID_HEARTBEAT)==1&&restart.has("plane_watchdog_started=1"),
    "independent native handback did not start watched heartbeat");
  auto copied=plane::modeRequest("copied",plane::guided_mode,restart.clock+1);
  require(!restart.runtime.sendCommand(copied),"copied Guided params bypassed acquisition");
}

void testNativeAutolandProfile(const char* path)
{
  for(const bool matching:{false,true}) {
    Rig rig(path);
    rig.runtime.configure("control_mission_execution","true");
    rig.runtime.configure("plane_native_autoland","true");
    for(int tick=0;tick<12&&rig.count(MAVLINK_MSG_ID_COMMAND_LONG)==0;++tick)rig.step();
    require(rig.count(MAVLINK_MSG_ID_COMMAND_LONG)==1&&rig.count(MAVLINK_MSG_ID_PARAM_REQUEST_READ)==0,
      "modern Plane parameters began without a version request");
    const auto& frame=*std::find_if(rig.sent.begin(),rig.sent.end(),[](const auto& packet) {
      return packet.msgid==MAVLINK_MSG_ID_COMMAND_LONG;
    });
    mavlink_command_long_t request{};
    mavlink_msg_command_long_decode(&frame,&request);
    require(request.command==MAV_CMD_REQUEST_MESSAGE&&request.param1==MAVLINK_MSG_ID_AUTOPILOT_VERSION&&
      request.target_system==1&&request.target_component==1,"version query was not addressed to the selected FC");
    mavlink_autopilot_version_t version{};
    version.flight_sw_version=0x040702ffu;
    constexpr std::string_view matching_hash="fb773567",wrong_hash="00000000";
    const auto hash=matching?matching_hash:wrong_hash;
    std::copy(hash.begin(),hash.end(),version.flight_custom_version);
    mavlink_message_t packet{};
    mavlink_msg_autopilot_version_encode(1,2,&packet,&version);
    rig.clock+=.01;
    rig.runtime.message(packet,rig.clock);
    require(!rig.has("plane_profile=MISMATCH"),"unselected component poisoned the native version check");
    mavlink_msg_autopilot_version_encode(1,1,&packet,&version);
    rig.clock+=.01;
    rig.runtime.message(packet,rig.clock-.05,rig.clock,"QUEUE_EMPTY_FLOOR");
    for(int tick=0;tick<12;++tick)rig.step();
    require(rig.count(MAVLINK_MSG_ID_HEARTBEAT)==0,"version alone started the Plane watchdog");
    require(matching?(rig.count(MAVLINK_MSG_ID_PARAM_REQUEST_READ)>0&&!rig.has("plane_profile=MISMATCH")):
                     (rig.count(MAVLINK_MSG_ID_PARAM_REQUEST_READ)==0&&rig.has("plane_profile=MISMATCH")),
      "wrong modern version was accepted or the selected pinned version was discarded");
  }

  Vehicle v;
  v.home_latitude=42.358456;v.home_longitude=-71.087589;
  std::vector<MissionItem> plan(plane::lifecycle_items);
  auto& approach=plan[plane::final_approach_item];
  approach.x=423541000;approach.y=-710875890;approach.z=45;
  auto& touchdown=plan[plane::landing_item];
  touchdown.x=423584560;touchdown.y=-710875890;
  require(plane::nativeLandingPlan(plan,v,0)&&plane::nativeLandingPlan(plan,v,359),
    "verified native runway did not match the grounded launch heading");
  require(!plane::nativeLandingPlan(plan,v,180),"reciprocal runway was treated as the native approach");
}

void testHandoff(const char* path)
{
  Rig rig(path);
  rig.qualify();rig.acquire();
  const auto target=rig.point();
  require(!rig.runtime.suspendNavigation(rig.clock+1),"pause admitted unconfirmed target");
  rig.readback(target,100);
  require(rig.has("plane_target=AWAIT_READBACK"),"different target falsely verified");
  rig.readback(target);
  require(rig.has("plane_target=VERIFIED"),"matching target did not verify");
  require(rig.runtime.suspendNavigation(rig.clock+1),"verified point did not admit bounded handoff");
  rig.step(.2); // Includes a new conventional/airborne EXTENDED_SYS_STATE pair.
  require(rig.has("resume_permit=true")&&rig.count(MAVLINK_MSG_ID_COMMAND_INT)==1,
    "fresh telemetry disrupted a normal pause or emitted a point");
  require(rig.runtime.resumeNavigation("resume"),"same-mode fresh resume rejected");
  require(rig.runtime.navigation(intent(rig.clock+.9)),"fresh intent after resume rejected");
  rig.runtime.revokeNavigation("operator_release");
  rig.step();
  require(rig.countMode(plane::loiter_mode)==1&&rig.count(MAVLINK_MSG_ID_COMMAND_INT)==1,
    "release failed to fence a pending point and request exactly one Loiter");
  rig.mode=plane::loiter_mode;
  rig.step();
  require(rig.has("plane_handback=CONFIRMED")&&!rig.runtime.navigationGranted(),
    "post-handoff Loiter missing or old grant returned");
}

void testPauseExpiry(const char* path)
{
  Rig rig(path);
  rig.qualify();rig.acquire();
  const auto target=rig.point();
  rig.readback(target);
  const double original_pause_end=rig.clock+.85;
  require(rig.runtime.suspendNavigation(original_pause_end),"verified point did not admit the pause");
  for(int i=0;i<4;++i) {
    rig.step(.2);
    rig.readback(target);
  }
  rig.step(.07);
  require(rig.clock>original_pause_end && !rig.runtime.resumeNavigation("late") &&
    rig.countMode(plane::loiter_mode)==1 && rig.count(MAVLINK_MSG_ID_COMMAND_INT)==1,
    "fresh target readbacks renewed the expired pause or old point");
}

void testLossAndFinalGate(const char* path)
{
  Rig loss(path);
  loss.qualify();loss.acquire();
  loss.point();
  for(int i=0;i<22;++i) {
    if(loss.runtime.navigationGranted())
      require(loss.runtime.navigation(intent(loss.clock+1.3,90)),"new input was rejected before loss");
    loss.step(.1);
  }
  require(loss.count(MAVLINK_MSG_ID_COMMAND_INT)==1&&loss.countMode(plane::loiter_mode)==1,
    "unconfirmed point retried or did not trigger a one-shot native handback");
  for(int i=0;i<38;++i) loss.step(.1);
  const auto watched=loss.count(MAVLINK_MSG_ID_HEARTBEAT);
  loss.step(.2);
  require(loss.has("plane_handback=UNCONFIRMED")&&loss.has("plane_watchdog=WITHHELD")&&
    loss.count(MAVLINK_MSG_ID_HEARTBEAT)==watched&&!loss.runtime.navigationGranted(),
    "unconfirmed handback extended watchdog or authority");

  Rig race(path);
  race.qualify();race.acquire();
  race.runtime.revokeNavigation("operator_release");
  race.before_send=[&](const mavlink_message_t& packet){
    if(packet.msgid!=MAVLINK_MSG_ID_COMMAND_LONG) return;
    race.clock+=.001;
    race.mode=5;
    race.heartbeat(race.mode);
  };
  race.step();
  require(race.countMode(plane::loiter_mode)==0&&race.has("plane_handback=EXTERNAL_MODE")&&
    race.has("plane_watchdog=ENABLED"),"late pilot takeover lost ownership at actual handoff");

  Rig expiry(path);
  expiry.qualify();expiry.acquire();
  require(expiry.runtime.navigation(intent(expiry.clock+.2)),"short test intent rejected");
  expiry.before_send=[&](const mavlink_message_t& packet){
    if(packet.msgid==MAVLINK_MSG_ID_COMMAND_INT) expiry.clock+=.2;
  };
  expiry.step(.11);
  require(expiry.count(MAVLINK_MSG_ID_COMMAND_INT)==0&&!expiry.runtime.navigationGranted(),
    "expired point crossed the last socket check");
}

void acquireFromStockMail(Rig& rig)
{
  const auto requests=rig.countMode(plane::guided_mode);
  const auto points=rig.count(MAVLINK_MSG_ID_COMMAND_INT);
  rig.helm();
  rig.clock+=.01;
  rig.text("MAVMOOS_NAV_CONTROL","acquire");
  rig.step();
  require(rig.countMode(plane::guided_mode)==requests+1 &&
          rig.count(MAVLINK_MSG_ID_COMMAND_INT)==points,
          "explicit stock acquisition did not send only the Guided mode request");
  rig.mode=plane::guided_mode;
  rig.step();
  require(rig.has("plane_guided_confirmed=1") && rig.count(MAVLINK_MSG_ID_COMMAND_INT)==points,
          "pre-acquisition stock fields produced a point after Guided");
  rig.clock+=.01;
  rig.helm();
  rig.step(.15);
  require(rig.count(MAVLINK_MSG_ID_COMMAND_INT)==points+1,"fresh stock helm fields did not produce a point");
}

void testExplicitNativeTransitions(const char* path)
{
  const std::array<std::pair<std::string,unsigned>,2> actions{{
    {"loiter",plane::loiter_mode},{"rtl",plane::rtl_mode}
  }};
  for(const auto& [action,destination]:actions) {
    Rig rig(path);
    rig.qualify();
    acquireFromStockMail(rig);
    rig.readback(rig.lastPoint());
    require(rig.runtime.navigation(intent(rig.clock+1.8,180)),"replacement Plane point did not enter the queue");
    rig.clock+=.01;
    rig.text("MAVMOOS_COMMAND","id=native,action="+action);
    rig.step(.1);
    require(rig.countMode(destination)==1 && rig.countMode(plane::loiter_mode)==(action=="loiter"?1:0) &&
            rig.count(MAVLINK_MSG_ID_COMMAND_INT)==1 && !rig.runtime.navigationGranted() &&
            rig.commandResult("native","SENT") && rig.runtime.controlBusy(),
            "explicit native command did not retire the pending point and reserve its transition");

    const auto handed_at=rig.clock;
    rig.clock+=.01;
    rig.text("MAVMOOS_NAV_CONTROL","acquire");
    rig.helm();
    mavlink_message_t ack{};
    mavlink_msg_command_ack_pack(1,1,&ack,MAV_CMD_DO_SET_MODE,MAV_RESULT_ACCEPTED,0,0,245,191);
    rig.runtime.message(ack,rig.clock);
    rig.clock+=.01;
    rig.heartbeat(destination,handed_at-.001);
    require(!rig.commandResult("native","CONFIRMED") && rig.runtime.controlBusy() &&
            !rig.runtime.navigationGranted() && rig.countMode(plane::guided_mode)==1,
            "ACK, old mode evidence or a competing acquire completed the transition");

    rig.mode=destination;
    rig.step();
    require(rig.commandResult("native","CONFIRMED") && !rig.runtime.controlBusy(),
            "a fresh independent mode did not close the explicit transition");
    rig.mode=plane::guided_mode;
    rig.step();
    rig.clock+=.01;
    rig.helm();
    rig.step(.15);
    require(!rig.runtime.navigationGranted() && rig.count(MAVLINK_MSG_ID_COMMAND_INT)==1,
            "fresh stock input restored the grant after a native transition");
    rig.mode=plane::loiter_mode;
    rig.step();
    // The last point was sent less than one rate-limit interval ago.
    rig.step(.8);
    acquireFromStockMail(rig);
  }
}

void testPilotTakeoverDuringExplicitSend(const char* path)
{
  Rig rig(path);
  rig.qualify();
  acquireFromStockMail(rig);
  rig.readback(rig.lastPoint());
  rig.clock+=.01;
  rig.text("MAVMOOS_COMMAND","id=return,action=rtl");
  bool serialized=false;
  rig.before_send=[&](const mavlink_message_t& packet) {
    if(packet.msgid!=MAVLINK_MSG_ID_COMMAND_LONG) return;
    mavlink_command_long_t command{};
    mavlink_msg_command_long_decode(&packet,&command);
    if(command.command!=MAV_CMD_DO_SET_MODE || command.param2!=plane::rtl_mode) return;
    serialized=true;
    rig.clock+=.001;
    rig.mode=5; // Pilot FBWA arrives after encoding and before the socket gate.
    rig.heartbeat(rig.mode);
  };
  rig.step();
  require(serialized && rig.countMode(plane::rtl_mode)==0 && rig.countMode(plane::loiter_mode)==0 &&
          !rig.commandResult("return","SENT") && rig.commandResult("return","CANCELLED") &&
          !rig.runtime.navigationGranted() && !rig.runtime.controlBusy() && rig.has("plane_watchdog=ENABLED"),
          "late pilot takeover did not withdraw the encoded explicit RTL");
  rig.mode=plane::guided_mode;
  rig.step();
  rig.clock+=.01;
  rig.helm();
  rig.step(.2);
  require(rig.count(MAVLINK_MSG_ID_COMMAND_INT)==1 && rig.countMode(plane::rtl_mode)==0,
          "old point or RTL reappeared after a late pilot takeover");
}

void testNativeLoiterAndReturnWithoutHelm(const char* path)
{
  Rig rig(path);
  rig.qualify();
  rig.text("MAVMOOS_COMMAND","id=return-native,action=rtl");
  rig.step();
  require(rig.countMode(plane::rtl_mode)==1&&!rig.runtime.navigationGranted(),"native Loiter could not request RTL");
  rig.mode=plane::rtl_mode;
  rig.step();
  require(rig.commandResult("return-native","CONFIRMED"),"native RTL lacked fresh FC confirmation");
  rig.text("MAVMOOS_COMMAND","id=return-noop,action=rtl");
  require(rig.commandResult("return-noop","NO_OP")&&rig.countMode(plane::rtl_mode)==1,
    "already-active RTL reissued a command");
  rig.clock+=.01;
  rig.text("MAVMOOS_COMMAND","id=loiter-native,action=loiter");
  rig.step();
  require(rig.countMode(plane::loiter_mode)==1,"native RTL could not hand back to Loiter");
  rig.mode=plane::loiter_mode;
  rig.step();
  require(rig.commandResult("loiter-native","CONFIRMED"),"native Loiter lacked fresh FC confirmation");
}

void testDirectRelativeAdjustments(const char* path)
{
  Rig course(path);
  course.qualify();
  course.text("MAVMOOS_CONTROL_REQUEST","schema=1,id=bad,action=adjust_course,strategy=guided,delta=95");
  require(course.directResult("bad","REJECTED"),"unbounded course adjustment was admitted");
  course.text("MAVMOOS_CONTROL_REQUEST","schema=1,id=course,action=adjust_course,strategy=guided,delta=45");
  course.step();
  require(course.countMode(plane::guided_mode)==1,"direct adjustment did not acquire qualified Guided");
  course.mode=plane::guided_mode;
  course.step(.15);
  const auto course_point=course.lastPoint();
  require(course_point.x>430010000&&course_point.x<430030000&&course_point.y>-709980000&&course_point.z==75,
    "relative course did not project the measured northbound track to a bounded northeast point");
  course.text("MAVMOOS_CONTROL_REQUEST","schema=1,id=overlap,action=adjust_altitude,strategy=guided,delta=12");
  require(course.directResult("overlap","REJECTED"),"new adjustment replaced existing Guided ownership");

  Rig altitude(path);
  altitude.qualify();
  altitude.text("MAVMOOS_CONTROL_REQUEST","schema=1,id=altitude,action=adjust_altitude,strategy=guided,delta=12");
  altitude.step();
  altitude.mode=plane::guided_mode;
  altitude.step(.15);
  const auto altitude_point=altitude.lastPoint();
  require(altitude_point.x==430000000&&altitude_point.y==-710000000&&altitude_point.z==87,
    "relative altitude did not retain the initial center and bind the target height");
  altitude.readback(altitude_point);
  altitude.step();
  require(!altitude.directResult("altitude","COMPLETED"),"target readback alone completed the altitude nudge");
  altitude.relative_mm=85000;
  altitude.step();
  require(altitude.directResult("altitude","COMPLETED"),"observed relative altitude did not complete the nudge");

  Rig takeover(path);
  takeover.qualify();
  takeover.text("MAVMOOS_CONTROL_REQUEST","schema=1,id=fenced,action=adjust_altitude,strategy=guided,delta=12");
  takeover.step();
  takeover.mode=plane::guided_mode;
  bool reached_handoff=false;
  takeover.before_send=[&](const mavlink_message_t& packet) {
    if(packet.msgid!=MAVLINK_MSG_ID_COMMAND_INT)return;
    reached_handoff=true;
    takeover.mode=5;takeover.clock+=.001;takeover.heartbeat(5);
  };
  takeover.step(.15);
  takeover.step();
  require(reached_handoff&&takeover.count(MAVLINK_MSG_ID_COMMAND_INT)==0&&takeover.directResult("fenced","CANCELLED"),
    "encoded adjustment crossed the final boundary after pilot takeover");
}

void testPlaneTaskWrapperOwnership(const char* path)
{
  Rig rig(path);
  rig.runtime.configure("operation_helm_enabled","true");
  rig.runtime.configure("operation_helm_tasks","survey");
  rig.qualify();
  require(rig.runtime.hasCapability("control.operations.helm.survey"),"configured Plane task was not advertised");
  rig.text("MAVMOOS_CONTROL_REQUEST","schema=1,id=survey,action=survey,strategy=helm,timeout=20");
  rig.step();
  require(rig.directResult("survey","SELECTED")&&!rig.runtime.navigationGranted()&&rig.countMode(plane::guided_mode)==0,
    "selection itself acquired vehicle authority");
  rig.text("MAVMOOS_CONTROL_REQUEST","schema=1,id=blocked,action=adjust_altitude,strategy=guided,delta=12");
  require(rig.directResult("blocked","REJECTED"),"selected Plane task did not exclude direct ownership");
  rig.helm();rig.text("MAVMOOS_NAV_CONTROL","acquire");
  require(rig.directResult("survey","ACQUIRE_REJECTED")&&!rig.runtime.navigationGranted(),
    "stock scalars allowed acquisition without a selected behavior witness");
  rig.clock+=.01;
  rig.text("MAVMOOS_HELM_TASK_READY","survey","pHelmIvP");
  acquireFromStockMail(rig);
  require(rig.directResult("survey","ACTIVE"),"fresh stock adapter state did not activate the selected task");
  rig.clock+=.01;
  rig.text("MAVMOOS_CONTROL_REQUEST","schema=1,id=cancel-task,action=cancel,target=survey");
  require(rig.directResult("survey","CANCELLED")&&rig.directResult("cancel-task","COMPLETED")&&
    !rig.runtime.navigationGranted(),"task cancellation left stock ownership granted");

  Rig future(path);
  future.runtime.configure("operation_helm_enabled","true");
  future.runtime.configure("operation_helm_tasks","survey");
  future.qualify();
  const double expires=future.clock+6;
  future.text("MAVMOOS_CONTROL_REQUEST","schema=1,id=expires,action=survey,strategy=helm,timeout=6");
  while(future.clock+.2<expires)future.step(.2);
  future.clock=expires;
  future.runtime.mail({"MAVMOOS_NAV_CONTROL","acquire","operator",0,expires+.05,expires,false});
  future.step();
  require(future.directResult("expires","TIMED_OUT")&&!future.runtime.navigationGranted()&&
    future.countMode(plane::guided_mode)==0,"future acquire crossed the exact task retirement boundary");
}
} // namespace

int main(int argc,char** argv)
{
  try {
    require(argc==2,"usage: plane_runtime_tests path/to/mavmoos_arduplane_control.so");
    testProfile(argv[1]);
    testNativeAutolandProfile(argv[1]);
    testHandoff(argv[1]);
    testPauseExpiry(argv[1]);
    testLossAndFinalGate(argv[1]);
    testExplicitNativeTransitions(argv[1]);
    testPilotTakeoverDuringExplicitSend(argv[1]);
    testNativeLoiterAndReturnWithoutHelm(argv[1]);
    testDirectRelativeAdjustments(argv[1]);
    testPlaneTaskWrapperOwnership(argv[1]);
    std::cout<<"PASS Plane profile, stock helm and native transitions, pause, cancellation and final send gates\n";
  } catch(const std::exception& error) {
    std::cerr<<error.what()<<'\n';
    return 1;
  }
}
