/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: FirmwareRuntimeTests.cpp                        */
/*    DATE: November 4, 2026                                */
/************************************************************/
// Actual Runtime/final eligibility with fake time and I/O; no socket or FC.
#include "Runtime.h"
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>
using namespace mavmoos::runtime;
namespace {
void require(bool value,const char* why){if(!value)throw std::runtime_error(why);}
mavlink_message_t decode(const uint8_t* data,std::size_t size)
{
  mavlink_message_t buffer{},message{};mavlink_status_t state{},result{};bool found=false;
  for(std::size_t i=0;i<size;++i)if(mavlink_frame_char_buffer(&buffer,&state,data[i],&message,&result)==MAVLINK_FRAMING_OK)found=true;
  require(found,"outgoing generated frame/CRC invalid");return message;
}
int command(const mavlink_message_t& message)
{
  if(message.msgid!=MAVLINK_MSG_ID_COMMAND_LONG)return -1;
  mavlink_command_long_t value{};mavlink_msg_command_long_decode(&message,&value);return value.command;
}
uint64_t epoch(const std::string& json)
{
  const auto at=json.find("\"epoch\":");require(at!=std::string::npos,"missing epoch");return std::stoull(json.substr(at+8));
}
struct Rig {
  double time=100;bool accept=true;std::vector<mavlink_message_t> sent;
  std::map<std::string,std::string> latest;std::function<void()> before,after;
  Runtime host;
  Rig(bool simulation=true,bool shadow=false,bool enabled=true):host(
    [&](const auto& key,const auto& text,double,bool numeric){if(!numeric)latest[key]=text;},
    [](const uint8_t*,std::size_t){throw std::runtime_error("unbounded sender used");return false;},
    simulation,shadow,[&]{return time;})
  {
    host.setFirmwareRequestsEnabled(enabled);
    host.setBoundedSend([&](const uint8_t* data,std::size_t size,double,const Runtime::SendEligibility& eligible){
      const auto message=decode(data,size);const bool discovery=command(message)==MAV_CMD_REQUEST_MESSAGE;
      if(discovery&&before)before();
      const bool allowed=eligible(time);
      const bool accepted=allowed&&accept;
      if(accepted)sent.push_back(message);
      if(discovery&&after)after();
      return accepted;
    });
  }
  void heartbeat(unsigned system=1,unsigned type=2)
  {
    mavlink_message_t message{};mavlink_status_t codec{};
    mavlink_msg_heartbeat_pack_status(system,1,&codec,&message,type,3,0,4,4);host.message(message,time);
  }
  void version(uint32_t value=(4u<<24)|(7u<<16))
  {
    mavlink_autopilot_version_t data{};data.flight_sw_version=value;data.capabilities=UINT64_C(0xfedcba9876543210);
    mavlink_message_t message{};mavlink_status_t codec{};mavlink_msg_autopilot_version_encode_status(1,1,&codec,&message,&data);host.message(message,time);
  }
  void flight()
  {
    mavlink_message_t m{};mavlink_status_t codec{};
    mavlink_msg_heartbeat_pack_status(1,1,&codec,&m,2,3,128,4,4);host.message(m,time);
    mavlink_msg_global_position_int_pack_status(1,1,&codec,&m,0,423584560,-710875890,15000,5000,0,100,0,9000);host.message(m,time);
    mavlink_msg_extended_sys_state_pack_status(1,1,&codec,&m,0,2);host.message(m,time);
    mavlink_home_position_t home{};home.latitude=423584560;home.longitude=-710875890;home.altitude=10000;
    mavlink_msg_home_position_encode_status(1,1,&codec,&m,&home);host.message(m,time);
    mavlink_ekf_status_report_t estimator{};estimator.flags=51;
    mavlink_msg_ekf_status_report_encode_status(1,1,&codec,&m,&estimator);host.message(m,time);
  }
  void tick(double at){time=at;host.tick(time);}
  std::size_t requests() const {std::size_t count=0;for(const auto& message:sent)if(command(message)==512)++count;return count;}
  std::string status()const{return latest.at("MAVMOOS_FIRMWARE_STATUS");}
};
}
int main(int argc,char** argv)
{
  try {
    require(argc==3,"telemetry/control module arguments required");unsigned groups=0;
    auto test=[&](const char* label,const auto& body){body();++groups;std::cout<<"PASS "<<label<<'\n';};
    test("bounded request uses standard 512/148 and exact target",[]{
      Rig r;r.time=100.01;r.heartbeat();r.tick(r.time);require(r.requests()==1,"missing discovery request");
      for(const auto& message:r.sent)if(command(message)==512){mavlink_command_long_t p{};mavlink_msg_command_long_decode(&message,&p);
        require(message.sysid==245&&message.compid==191&&p.target_system==1&&p.target_component==1&&p.param1==148&&p.param2==0&&p.param7==0,"nonstandard request envelope");}
    });
    test("library active requests require explicit opt-in",[]{Rig r(true,false,false);r.time=100.01;r.heartbeat();r.tick(r.time);require(r.requests()==0,"implicit library discovery");});
    test("receive-only and shadow retain passive identity with zero bytes",[]{for(const auto mode:{0,1}){
      Rig r(mode==1,mode==1);r.time=100.01;r.heartbeat();r.time=100.02;r.version();r.tick(100.21);
      require(r.sent.empty(),"RX/shadow transmitted");require(r.status().find("\"state\":\"OBSERVED\"")!=std::string::npos,"passive version missing");require(!r.host.navigationGranted(),"identity granted authority");
    }});
    test("pre-heartbeat version cannot bind and response stops reads",[]{Rig r;r.time=100.01;r.version();r.tick(r.time);require(r.status().find("\"present\":false")!=std::string::npos,"prebound identity used");
      r.time=100.1;r.heartbeat();r.tick(r.time);r.time=100.2;r.version();r.tick(100.4);require(r.requests()==1,"response did not stop requests");
      require(r.status().find("\"capabilities_raw_hex\":\"0xfedcba9876543210\"")!=std::string::npos,"64-bit capabilities lost");});
    test("heartbeat gap clears old firmware before fresh inputs",[]{Rig r;r.time=100.01;r.heartbeat();r.time=100.02;r.version();r.tick(100.21);const auto old=epoch(r.status());
      r.tick(103);r.time=103.1;r.heartbeat();r.tick(103.3);require(epoch(r.status())>old&&r.status().find("\"present\":false")!=std::string::npos,"gap retained old firmware");});
    test("explicit reset cannot reuse old epoch during final proof",[]{Rig r;r.time=100.01;r.heartbeat();r.before=[&]{r.host.disconnect(r.time,"test_reset");r.time+=.01;r.heartbeat();};r.tick(100.02);
      require(r.requests()==0,"old encoded request survived reset+fresh heartbeat");});
    test("host disabling discovery wins at final eligibility",[]{Rig r;r.time=100.01;r.heartbeat();r.before=[&]{r.host.setFirmwareRequestsEnabled(false);};r.tick(100.02);require(r.requests()==0,"disabled discovery crossed handoff");});
    test("original deadline is rechecked after endpoint proof",[]{Rig r;r.time=100.01;r.heartbeat();r.before=[&]{r.time=110.01;};r.tick(100.01);require(r.requests()==0,"expired discovery sent");});
    test("numeric clock fault at final eligibility cancels discovery",[]{Rig r;r.time=100.01;r.heartbeat();r.before=[&]{r.time=std::numeric_limits<double>::quiet_NaN();};r.tick(100.02);
      require(r.requests()==0&&!r.host.vehicle().heartbeat_valid,"clock fault retained request/session");});
    test("accepted read followed by bad clock stays visible",[]{Rig r;r.time=100.01;r.heartbeat();r.after=[&]{r.time=std::numeric_limits<double>::infinity();};r.tick(100.02);
      require(r.requests()==1,"actual acceptance concealed");const auto event=r.latest.at("MAVMOOS_FIRMWARE_REQUEST_RESULT");require(event.find("\"local_result\":\"LOCAL_ACCEPTED\"")!=std::string::npos&&event.find("\"clock_valid\":false")!=std::string::npos,"false post-return timing outcome");});
    test("missing response cannot exceed bounded idempotent reads",[]{Rig r;for(unsigned i=1;i<700;++i){r.time=100+i*.02;r.heartbeat();r.host.tick(r.time);}require(r.requests()==3,"missing response exceeded three requests");require(r.status().find("\"state\":\"TIMED_OUT\"")!=std::string::npos,"missing original expiry");});
    test("local failure retries stay bounded and do not grant control",[]{Rig r;r.accept=false;for(unsigned i=1;i<100;++i){r.time=100+i*.02;r.heartbeat();r.host.tick(r.time);}require(r.requests()==0&&!r.host.navigationGranted(),"rejected send counted as accepted/authority");
      require(r.status().find("\"attempts\":3")!=std::string::npos&&r.status().find("\"state\":\"EXHAUSTED\"")!=std::string::npos,"unbounded local retry");});
    test("version discovery and streams keep one protocol budget",[&]{Rig r;require(r.host.load(argv[1]),"telemetry load failed");unsigned streams=0;
      for(unsigned step=1;step<100;++step){r.time=100+step*.02;r.heartbeat();const auto before=r.sent.size();r.host.tick(r.time);unsigned bulk=0;
        for(std::size_t i=before;i<r.sent.size();++i){const int c=command(r.sent[i]);if(c==511||c==512){++bulk;if(c==511)++streams;}}
        require(bulk<=1,"observation protocol budget doubled");}
      require(r.requests()==1&&streams==4,"shared lane starved or repeated a read/setup family");});
    test("mismatched and foreign heartbeat cannot start requests",[]{for(unsigned mode:{0,1}){Rig r;r.time=100.01;r.heartbeat(mode?1:2,mode?10:2);r.tick(r.time);require(r.requests()==0,"foreign/mismatched binding transmitted");}});
    test("control stays first while all protocol families make progress",[&]{
      Rig r;require(r.host.load(argv[1])&&r.host.load(argv[2]),"load control/telemetry");
      r.host.configure("control_native_actions","false");r.host.configure("control_helm","false");
      r.time=100.01;r.flight();require(r.host.acquireNavigation("priority"),"navigation grant");
      for(unsigned i=0;i<4;++i){ParameterRequest p;p.id="p"+std::to_string(i);p.name="FRAME_CLASS";p.expires=102;require(r.host.parameter(p),"parameter admission");
        MissionRequest m;m.id="m"+std::to_string(i);m.operation_id="read";m.action=MissionAction::RequestList;m.expires=102;m.operation_expires=105;require(r.host.mission(m),"mission admission");}
      unsigned parameters=0,missions=0,streams=0,navigation=0;
      for(unsigned step=0;step<90;++step){r.time=100.02+step*.02;r.flight();require(r.host.navigation({90,1,5,r.time+.4}),"navigation target");const auto before=r.sent.size();r.host.tick(r.time);unsigned bulk=0;
        for(std::size_t i=before;i<r.sent.size();++i){const auto& m=r.sent[i];const int cmd=command(m);
          if(m.msgid==84){require(bulk==0,"bulk preceded control");++navigation;}
          if(m.msgid==20){++parameters;++bulk;}if(m.msgid==43){++missions;++bulk;}
          if(cmd==511){++streams;++bulk;}if(cmd==512)++bulk;
        }require(bulk<=1,"total protocol budget doubled");}
      require(parameters==4&&missions==4&&streams==4&&r.requests()==1&&navigation>=4&&r.host.navigationGranted(),"protocol family starved or navigation was revoked");
    });
    test("valid owner clock and new heartbeat permit read-only rediscovery",[]{
      Rig r;r.time=100.01;r.heartbeat();r.time=100.02;r.version();r.tick(100.2);const auto before=epoch(r.status());
      r.tick(std::numeric_limits<double>::quiet_NaN());
      require(r.status().find("\"present\":false")!=std::string::npos&&epoch(r.status())>before,"raw clock fault retained firmware");
      r.time=100.3;r.heartbeat();r.tick(100.31);
      require(r.requests()==1&&!r.host.navigationGranted()&&r.status().find("\"present\":false")!=std::string::npos,"read-only recovery reused identity or granted control");
      r.time=100.4;r.version();r.tick(100.6);
      require(r.status().find("\"state\":\"OBSERVED\"")!=std::string::npos&&!r.host.navigationGranted(),"new firmware observation failed or acquired control");
    });
    std::cout<<"PASS "<<groups<<" actual Runtime firmware groups; fake I/O only\n";return 0;
  } catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
