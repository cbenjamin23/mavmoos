/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ProtocolNativeDriver.cpp                        */
/*    DATE: November 4, 2026                                */
/************************************************************/
// Disarmed service bench: actual Runtime/modules and guarded native UDP.
// Mail requests are direct fixture proposals, not a MOOSDB/helm demonstration.
#include "ProtocolSupport.h"
#include "Config.h"
#include "Telemetry.h"
#include "UdpReceiver.h"
#include <chrono>
#include <iostream>
#include <thread>
using namespace protocol_faults;
namespace {
unsigned argument(const char* text,unsigned low,unsigned high){unsigned n=0;require(mavmoos::parseUnsigned(text,low,high,n),"invalid integer argument");return n;}
std::string parameterName(const mavlink_message_t& m){mavlink_param_request_read_t p{};mavlink_msg_param_request_read_decode(&m,&p);return std::string(p.param_id,std::find(p.param_id,p.param_id+16,'\0'));}
bool allowed(const mavlink_message_t& m){return m.msgid==0||m.msgid==20||m.msgid==23||m.msgid==43||m.msgid==44||m.msgid==45||m.msgid==47||m.msgid==51||m.msgid==73||(m.msgid==76&&command(m)==MAV_CMD_SET_MESSAGE_INTERVAL);}
}
int main(int argc,char** argv){try{
  require(argc==7,"usage: protocol_native_driver BIND_PORT PEER_PORT NATIVE_PID PARAMETERS MISSIONS wind_readback_loss|mission_ack_loss");
  const auto bind=argument(argv[1],1024,65535),peer=argument(argv[2],1024,65535),pid=argument(argv[3],1,2147483647);
  const std::string scenario=argv[6];require(scenario=="wind_readback_loss"||scenario=="mission_ack_loss","unsupported scenario");
  require(mavmoos::UdpReceiver::isSitlEndpoint(int(pid),uint16_t(peer),"arducopter"),"exact owned native endpoint not proven");
  mavmoos::RxConfig config;config.bind_address=config.peer_address="127.0.0.1";config.bind_port=uint16_t(bind);config.peer_port=uint16_t(peer);
  config.target_system=config.target_component=1;config.expected_type=2;config.expected_autopilot=3;
  require(mavmoos::validateConfig(config).empty(),"invalid receive configuration");
  mavmoos::UdpReceiver udp;require(udp.open(config),"owned UDP bind failed");
  const double started=mavmoos::monotonicSeconds();double now=started,last_poll=started,settled_at=0;
  std::string phase="WAITING_GROUND",failure;std::map<std::string,std::map<std::string,std::string>> results,values;
  std::map<std::string,std::string> snapshots;std::vector<Publication> publications;std::vector<Frame> frames;
  bool inject=false,readback_sent=false,saw_armed=false,forbidden=false,done=false;
  unsigned dropped=0,delivered=0,received=0;
  const auto emit=[&](const std::string& text){std::cout<<text<<'\n';std::cout.flush();};
  Runtime runtime([&](const auto& key,const auto& text,double,bool numeric){
    if(numeric)return;const double at=mavmoos::monotonicSeconds();publications.push_back({key,text,at});
    emit("{\"type\":\"publication\",\"phase\":"+quote(phase)+",\"key\":"+quote(key)+",\"text\":"+quote(text)+",\"at\":"+decimal(at)+"}");
    const auto f=fields(text);if((key=="MAVMOOS_PARAM_RESULT"||key=="MAVMOOS_MISSION_RESULT")&&f.count("id"))results[f.at("id")]=f;
    if(key=="MAVMOOS_PARAM_VALUE"&&f.count("id"))values[f.at("id")]=f;
    if(key=="MAVMOOS_MISSION_SNAPSHOT")snapshots[key]=text;
  },[](const uint8_t*,std::size_t){return false;},true,false,mavmoos::monotonicSeconds);
  runtime.setBoundedSend([&](const uint8_t* bytes,std::size_t size,double expires,const Runtime::SendEligibility& live){
    const auto m=decode(bytes,size);if(!allowed(m)){forbidden=true;return false;}
    const double before=mavmoos::monotonicSeconds();const bool accepted=udp.sendSimulationFrame(bytes,size,int(pid),expires,live);
    const double after=mavmoos::monotonicSeconds();frames.push_back({m,before,expires,accepted});
    emit("{\"type\":\"send\",\"phase\":"+quote(phase)+",\"receipt_at\":"+decimal(after)+",\"frame\":"+frameJson(frames.back())+"}");
    if(accepted&&inject&&scenario=="wind_readback_loss"&&m.msgid==20&&parameterName(m)=="SIM_WIND_SPD")readback_sent=true;
    return accepted;
  });
  require(runtime.load(argv[4])&&runtime.load(argv[5]),"actual protocol MODULE load failed");
  runtime.configure("parameter_timeout","2");runtime.configure("mission_timeout","6");runtime.configure("mission_response_timeout","0.5");
  mavmoos::Telemetry telemetry(config,mavmoos::newSessionId());
  telemetry.setTimedMessageHandler([&](const mavlink_message_t& m,double effective,double read){
    ++received;bool drop=false;
    if(m.msgid==0){mavlink_heartbeat_t p{};mavlink_msg_heartbeat_decode(&m,&p);if(p.base_mode&MAV_MODE_FLAG_SAFETY_ARMED)saw_armed=true;}
    if(inject&&scenario=="wind_readback_loss"&&readback_sent&&m.msgid==MAVLINK_MSG_ID_PARAM_VALUE){mavlink_param_value_t p{};mavlink_msg_param_value_decode(&m,&p);drop=std::string(p.param_id,std::find(p.param_id,p.param_id+16,'\0'))=="SIM_WIND_SPD";}
    if(inject&&scenario=="mission_ack_loss"&&m.msgid==MAVLINK_MSG_ID_MISSION_ACK)drop=true;
    emit("{\"type\":\"receive\",\"phase\":"+quote(phase)+",\"decision\":"+quote(drop?"DROP_AT_PARSED_RECEIPT_BOUNDARY":"DELIVER")+",\"effective_at\":"+decimal(effective)+",\"read_at\":"+decimal(read)+",\"wire_hex\":"+quote(wire(m))+",\"msgid\":"+std::to_string(m.msgid)+"}");
    if(drop){++dropped;return;}++delivered;runtime.message(m,effective,read,"QUEUE_EMPTY_FLOOR");
  });
  const auto mail=[&](const std::string& key,const std::string& text){const double at=mavmoos::monotonicSeconds();Mail m;m.key=key;m.text=text;m.source="protocol_native_fixture";m.published_at=m.received_at=at;runtime.mail(m);emit("{\"type\":\"request\",\"phase\":"+quote(phase)+",\"key\":"+quote(key)+",\"text\":"+quote(text)+",\"at\":"+decimal(at)+",\"boundary\":\"direct Mail proposal; no MOOSDB\"}");};
  const auto terminal=[&](const std::string& id,const std::string& want){const auto found=results.find(id);return found!=results.end()&&found->second.at("status")==want;};
  const auto value=[&](const std::string& id){require(values.count(id)&&values[id].count("value"),"missing native parameter value");return std::stod(values[id].at("value"));};
  emit("{\"type\":\"start\",\"scenario\":"+quote(scenario)+",\"at\":"+decimal(started)+",\"scope\":\"owned disarmed native Runtime/MODULE UDP service bench; direct fixture Mail; no helm or physical route\",\"native_pid\":"+std::to_string(pid)+",\"peer_port\":"+std::to_string(peer)+",\"drop_boundary\":\"after validated native parse, before Runtime dispatch; original metadata retained; no artificial release/delay\"}");
  while(!done&&mavmoos::monotonicSeconds()-started<90){
    now=mavmoos::monotonicSeconds();require(now>=last_poll,"fixture clock regressed");last_poll=now;
    require(runtime.checkpoint(now),"Runtime checkpoint failed");udp.poll(telemetry);now=mavmoos::monotonicSeconds();
    require(!saw_armed&&!forbidden,"native service bench armed or proposed forbidden effects");
    const auto& vehicle=runtime.vehicle();
    const bool ground=vehicle.heartbeat_valid&&now-vehicle.heartbeat_at<=2&&!vehicle.armed&&vehicle.landed_valid&&now-vehicle.landed_at<=2&&vehicle.landed_state==1;
    if(phase=="WAITING_GROUND"&&ground&&(scenario=="wind_readback_loss"||vehicle.home_valid)){
      if(scenario=="wind_readback_loss"){phase="BASELINE_READ";mail("MAVMOOS_PARAM_REQUEST","id=wind-baseline,action=read,name=SIM_WIND_SPD");}
      else{phase="UPLOAD_WITH_ACK_DROP";inject=true;mail("MAVMOOS_MISSION_REQUEST","id=mission-loss,action=upload,timeout=6,items="+rows());}
    }else if(phase=="BASELINE_READ"&&terminal("wind-baseline","COMPLETED")){
      require(std::abs(value("wind-baseline"))<1e-6,"wind baseline not zero");phase="WRITE_WITH_READBACK_DROP";inject=true;mail("MAVMOOS_PARAM_REQUEST","id=wind-loss,action=write,name=SIM_WIND_SPD,value=0.25,type=REAL32");
    }else if(phase=="WRITE_WITH_READBACK_DROP"&&terminal("wind-loss","TIMED_OUT")){
      require(dropped>0&&results["wind-loss"]["write_may_have_applied"]=="true"&&results["wind-loss"]["readback_confirmed"]=="false","missing wind uncertainty/drop evidence");inject=false;phase="FRESH_WIND_READ";mail("MAVMOOS_PARAM_REQUEST","id=wind-check,action=read,name=SIM_WIND_SPD");
    }else if(phase=="FRESH_WIND_READ"&&terminal("wind-check","COMPLETED")){
      require(std::abs(value("wind-check")-.25)<1e-6,"fresh native wind read did not recover applied mutation");phase="VERIFIED_WIND_RESTORE";mail("MAVMOOS_PARAM_REQUEST","id=wind-restore,action=write,name=SIM_WIND_SPD,value=0,type=REAL32");
    }else if(phase=="VERIFIED_WIND_RESTORE"&&terminal("wind-restore","COMPLETED")){
      require(std::abs(value("wind-restore"))<1e-6&&results["wind-restore"]["readback_confirmed"]=="true","wind not verified restored");phase="FINAL_GROUND";settled_at=now;
    }else if(phase=="UPLOAD_WITH_ACK_DROP"&&terminal("mission-loss","TIMED_OUT")){
      require(dropped>0&&results["mission-loss"]["mutation_may_have_applied"]=="true"&&results["mission-loss"]["remote_accepted"]=="false"&&results["mission-loss"]["readback_confirmed"]=="false","missing mission uncertainty/drop evidence");inject=false;phase="EXPLICIT_REMOTE_SETTLEMENT_WAIT";settled_at=now;
    }else if(phase=="EXPLICIT_REMOTE_SETTLEMENT_WAIT"&&now-settled_at>=8.5){
      phase="FRESH_STORED_PLAN_READ";mail("MAVMOOS_MISSION_REQUEST","id=mission-check,action=download,timeout=10");
    }else if(phase=="FRESH_STORED_PLAN_READ"&&terminal("mission-check","COMPLETED")){
      require(snapshots["MAVMOOS_MISSION_SNAPSHOT"].find("\"count\":2")!=std::string::npos,"fresh native stored plan count differs");phase="VERIFIED_MISSION_CLEAR";mail("MAVMOOS_MISSION_REQUEST","id=mission-clear,action=clear,timeout=10");
    }else if(phase=="VERIFIED_MISSION_CLEAR"&&terminal("mission-clear","COMPLETED")){
      require(results["mission-clear"]["readback_confirmed"]=="true"&&snapshots["MAVMOOS_MISSION_SNAPSHOT"].find("\"count\":0")!=std::string::npos,"mission clear not independently read back empty");phase="FINAL_GROUND";settled_at=now;
    }else if(phase=="FINAL_GROUND"&&ground&&now-settled_at>=1){done=true;}
    runtime.tick(mavmoos::monotonicSeconds());std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  require(done,"native protocol fixture exceeded90s scenario bound at "+phase);
  require(!runtime.navigationGranted()&&!runtime.controlBusy(),"service bench acquired motion authority");
  unsigned sets=0;for(const auto& f:frames)if(f.accepted&&f.message.msgid==23)++sets;
  require(scenario!="wind_readback_loss"||sets==2,"wind fixture replayed SET instead of one mutation and one restore");
  emit("{\"type\":\"summary\",\"result\":\"PASS\",\"scenario\":"+quote(scenario)+",\"duration_s\":"+decimal(mavmoos::monotonicSeconds()-started)+",\"received\":"+std::to_string(received)+",\"delivered\":"+std::to_string(delivered)+",\"dropped\":"+std::to_string(dropped)+",\"accepted_parameter_sets\":"+std::to_string(sets)+",\"never_armed\":true,\"navigation_granted\":false,\"physical\":false,\"fault_kind\":\"parsed receipt omission; no network latency claim\",\"runtime\":"+quote(runtime.status())+"}");
  udp.close();return 0;
}catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}}
