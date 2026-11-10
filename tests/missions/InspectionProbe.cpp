// Test-only operator for the native and receive-only mission inspection benches.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include <ardupilotmega/mavlink.h>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
constexpr auto request_key="MAVMOOS_MISSION_REQUEST";
constexpr auto result_key="MAVMOOS_MISSION_INSPECTION_RESULT";
constexpr auto runtime_key="MAVMOOS_RUNTIME_STATUS";

double now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::map<std::string,std::string> fields(const std::string& text)
{
  std::istringstream input(text);
  std::map<std::string,std::string> result;
  std::string field;
  while(std::getline(input,field,',')) {
    const auto separator=field.find('=');
    if(separator!=std::string::npos)result.emplace(field.substr(0,separator),field.substr(separator+1));
  }
  return result;
}

bool terminal(const std::string& status)
{
  return status=="COMPLETED"||status=="FAILED"||status=="TIMED_OUT"||status=="UNSUPPORTED"||
    status=="REJECTED"||status=="CANCELLED"||status=="DUPLICATE"||status=="INCOMPLETE";
}

void frame(const mavlink_message_t& message)
{
  uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
  const auto length=mavlink_msg_to_send_buffer(bytes,&message);
  for(unsigned i=0;i<length;++i)
    std::cout<<std::hex<<std::setfill('0')<<std::setw(2)<<unsigned(bytes[i]);
  std::cout<<'\n';
}

void frames()
{
  mavlink_status_t codec{};
  for(unsigned i=0;i<192;++i) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack_status(1,1,&codec,&message,2,3,0,0,MAV_STATE_STANDBY);
    frame(message);
    mavlink_msg_heartbeat_pack_status(1,2,&codec,&message,2,3,128,3,MAV_STATE_ACTIVE);
    frame(message);
    for(uint8_t type=0;type<3;++type) {
      mavlink_mission_count_t count{};
      count.target_system=type==0?246:245;
      count.target_component=191;
      count.mission_type=type;
      mavlink_msg_mission_count_encode_status(1,1,&codec,&message,&count);
      frame(message);
    }
  }
}

struct Step {std::string id,type,expected;};
int run(unsigned db,const std::string& mode,const std::filesystem::path& gate)
{
  const bool native=mode=="native";
  if(!native&&mode!="loopback")throw std::runtime_error("unknown inspection probe mode");
  CMOOSCommClient client;
  const auto started=now();
  const auto log=[&](const std::string& kind,const std::string& key,const std::string& value) {
    std::cout<<"{\"elapsed\":"<<std::fixed<<std::setprecision(6)<<now()-started
      <<",\"type\":"<<std::quoted(kind)<<",\"key\":"<<std::quoted(key)
      <<",\"value\":"<<std::quoted(value)<<"}"<<std::endl;
  };
  client.SetQuiet(true);
  if(!client.Run("127.0.0.1",db,"mavmoos_mission_inspection_probe",50))
    throw std::runtime_error("MOOS client failed");

  std::vector<Step> steps;
  for(const std::string type:{"flight","fence","rally"})
    steps.push_back({type+"-empty",type,native?"COMPLETED":"FAILED"});
  if(native) {
    for(const std::string type:{"flight","fence","rally"})
      steps.push_back({type+"-seeded",type,"COMPLETED"});
  } else {
    steps.push_back({"fence-empty","fence","DUPLICATE"});
    steps.push_back({"fence-new-session","fence","FAILED"});
  }
  bool registered=false,composed=false,gate_reported=false,pending=false;
  bool snapshot_seen=false,result_seen=false;
  std::size_t index=0;
  double runtime_at=0,requested_at=0;
  std::map<std::string,std::string> runtime;
  while(now()-started<34) {
    if(client.IsConnected()&&!registered) {
      for(const auto* key:{"MAVMOOS_CAPABILITIES",runtime_key,result_key,"MAVMOOS_MISSION_TX_RESULT",
          "MAVMOOS_MISSION_INSPECTION_FLIGHT_SNAPSHOT","MAVMOOS_MISSION_INSPECTION_FENCE_SNAPSHOT",
          "MAVMOOS_MISSION_INSPECTION_RALLY_SNAPSHOT","MAVMOOS_MISSION_RESULT","MAVMOOS_MISSION_SNAPSHOT",
          "MAVMOOS_STORED_MISSION_PROOF","IVPHELM_STATE","MAVMOOS_HELM_STATUS"})
        client.Register(key,0);
      registered=true;
    }
    MOOSMSG_LIST mail;
    client.Fetch(mail);
    for(const auto& message:mail) {
      const auto& key=message.GetKey();
      if(key=="IVPHELM_STATE"||key=="MAVMOOS_HELM_STATUS")
        throw std::runtime_error("unexpected helm in mission inspection fixture");
      if(message.GetSource()!="iMAVLink"||!message.IsString())continue;
      const auto& value=message.GetString();
      const std::string kind=key=="MAVMOOS_CAPABILITIES"?"capabilities":key==runtime_key?"runtime":
        key==result_key?"result":key=="MAVMOOS_MISSION_TX_RESULT"?"tx":
        key.find("_SNAPSHOT")!=std::string::npos?"snapshot":"other";
      log(kind,key,value);
      if(key=="MAVMOOS_CAPABILITIES")composed=true;
      if(key==runtime_key) {
        runtime=fields(value);runtime_at=now();
        if(runtime["armed"]!="0"||runtime["permission"]!="false"||runtime["nav_tx_accepted"]!="0"||
           runtime["mission_mutations"]!="0"||runtime["control_profile"]!="none")
          throw std::runtime_error("runtime reports unexpected control or mutation");
      }
      if(!pending)continue;
      const auto& step=steps[index];
      if(kind=="snapshot"&&value.find("\"status\":\"complete\"")!=std::string::npos&&
         value.find("\"id\":\""+step.id+"\"")!=std::string::npos)snapshot_seen=true;
      if(key!=result_key)continue;
      auto result=fields(value);
      if(result["id"]!=step.id||result["type"]!=step.type||!terminal(result["status"]))continue;
      if(result["status"]!=step.expected||
         (step.expected=="FAILED"&&result["reason"]!="host_admission_inhibited"))
        throw std::runtime_error("unexpected inspection outcome for "+step.id+" "+value);
      result_seen=true;
    }
    if(pending&&result_seen&&(steps[index].expected!="COMPLETED"||snapshot_seen)) {
      pending=false;
      ++index;
    }
    if(index==steps.size()) {
      log("workflow","result","PASS");
      client.Close();
      return 0;
    }
    if(index==3&&!gate_reported) {
      log("phase","gate","waiting");
      gate_reported=true;
    }
    if(pending&&now()-requested_at>7)
      throw std::runtime_error("inspection result deadline for "+steps[index].id);
    const bool ready=composed&&runtime["heartbeat_valid"]=="1"&&now()-runtime_at<.4;
    if(!pending&&ready&&(index<3||std::filesystem::is_regular_file(gate))) {
      const auto& step=steps[index];
      const std::string text="schema=1,id="+step.id+",action=inspect,type="+step.type+",timeout=6";
      if(!client.Notify(request_key,text))throw std::runtime_error("failed to post inspection request");
      log("request",request_key,text);
      pending=true;
      snapshot_seen=result_seen=false;
      requested_at=now();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  throw std::runtime_error("bounded inspection probe did not complete");
}
}

int main(int argc,char** argv)
{
  try {
    if(argc==2&&std::string(argv[1])=="--frames"){frames();return 0;}
    if(argc!=4)throw std::runtime_error("usage: mission_inspection_probe DB_PORT native|loopback GATE_FILE");
    return run(std::stoul(argv[1]),argv[2],argv[3]);
  } catch(const std::exception& error) {
    std::cerr<<"FAIL "<<error.what()<<'\n';
    return 1;
  }
}
