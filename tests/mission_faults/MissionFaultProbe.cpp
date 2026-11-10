// Disarmed mission-data fault coordinator. All MAVLink effects use the provider.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
double now() {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
std::map<std::string,std::string> fields(const std::string& text)
{
  std::map<std::string,std::string> result;std::istringstream in(text);std::string part;
  while(std::getline(in,part,',')) {const auto at=part.find('=');if(at!=std::string::npos)result[part.substr(0,at)]=part.substr(at+1);}
  return result;
}
bool truth(const std::string& value){return value=="true"||value=="1";}
std::string quoted(const std::string& value)
{
  std::string result="\"";
  for(unsigned char c:value) {
    if(c=='"'||c=='\\')result+='\\';
    if(c=='\n')result+="\\n";else if(c=='\r')result+="\\r";else if(c=='\t')result+="\\t";else result+=char(c);
  }
  return result+'"';
}
unsigned count(const std::string& text)
{
  const auto at=text.find("\"count\":");if(at==std::string::npos)throw std::runtime_error("snapshot count missing");
  return std::stoul(text.substr(at+8));
}
std::string snapshotId(const std::string& text)
{
  const auto at=text.find("\"id\":\"");if(at==std::string::npos)return {};
  const auto begin=at+6,end=text.find('"',begin);return end==std::string::npos?std::string{}:text.substr(begin,end-begin);
}
std::string upload()
{
  std::ostringstream out;out<<"id=mf-upload,action=upload,timeout=30,items=0:16:0:0:1:0:0:0:0:423584560:-710875890:10";
  for(unsigned seq=1;seq<64;++seq)out<<'|'<<seq<<":16:3:0:1:0:0:0:0:"<<423584560+seq*10<<':'<<-710875890+int(seq)*5<<":5";
  return out.str();
}
}
int main(int argc,char** argv)
{
  CMOOSCommClient client;const double started=now();
  auto log=[&](const std::string& type,const std::string& key,const std::string& value) {
    std::cout<<"{\"elapsed\":"<<std::fixed<<std::setprecision(9)<<now()-started<<",\"type\":"<<quoted(type)
             <<",\"key\":"<<quoted(key)<<",\"value\":"<<quoted(value)<<"}\n"<<std::flush;
  };
  auto request=[&](const std::string& text){client.Notify("MAVMOOS_MISSION_REQUEST",text);log("request","MAVMOOS_MISSION_REQUEST",text);};
  try {
    if(argc!=4)throw std::runtime_error("usage: mission_fault_probe DB_PORT cancel|reset CONTROL_DIRECTORY");
    const std::string scenario=argv[2],control=argv[3];if(scenario!="cancel"&&scenario!="reset")throw std::runtime_error("invalid scenario");
    client.SetQuiet(true);if(!client.Run("127.0.0.1",std::stoi(argv[1]),"mavmoos_mission_fault_probe",100))throw std::runtime_error("MOOS client failed");
    enum class Phase {Ready,Baseline,Uploading,Retired,Duplicate,Busy,Settlement,Readback,Clear,Done};
    Phase phase=Phase::Ready;double phaseAt=started,retiredAt=-1,retiredHostAt=-1,uploadExpires=-1;bool registered=false,triggered=false,retired=false,duplicate=false;
    std::vector<double> oldAcceptedAt;
    double runtimeAt=-1,landedAt=-1;unsigned lastSent=0;std::map<std::string,std::string> runtime,landed,home,status,reason,snapshots;
    auto transition=[&](Phase next,const std::string& name){phase=next;phaseAt=now();log("phase",name,"entered");};
    auto completed=[&](const std::string& id){return status[id]=="COMPLETED";};
    auto check=[&](const std::string& id){if(status[id]=="FAILED"||status[id]=="REJECTED"||status[id]=="TIMED_OUT"||status[id]=="INCOMPLETE")throw std::runtime_error(id+":"+status[id]+":"+reason[id]);};
    auto benchReady=[&] {
      const double at=now();return runtimeAt>0&&landedAt>0&&at-runtimeAt<.5&&at-landedAt<.5&&truth(runtime["heartbeat_valid"])&&
        truth(runtime["landed_valid"])&&truth(runtime["home_valid"])&&!truth(runtime["armed"])&&!truth(runtime["permission"])&&
        landed["encoding_valid"]=="true"&&landed["freshness"]=="FRESH"&&landed["landed_state"]=="1"&&home["encoding_valid"]=="true"&&home["identity"]=="EXPECTED";
    };
    while(now()-started<125) {
      if(client.IsConnected()&&!registered) {
        for(const auto& key:{"MAVMOOS_CAPABILITIES","MAVMOOS_RUNTIME_STATUS","MAVMOOS_LANDED_STATE","MAVMOOS_HOME_POSITION",
            "MAVMOOS_MISSION_RESULT","MAVMOOS_MISSION_SNAPSHOT","MAVMOOS_MISSION_TX_RESULT","MAVMOOS_HELM_STATUS","IVPHELM_STATE","MAVMOOS_COMMAND_RESULT"})client.Register(key,0);
        registered=true;
      }
      MOOSMSG_LIST mail;client.Fetch(mail);
      for(const auto& message:mail) {
        const auto key=message.GetKey();
        if(key=="IVPHELM_STATE"||key=="MAVMOOS_HELM_STATUS"||key=="MAVMOOS_COMMAND_RESULT")throw std::runtime_error("data-only bench received a control interface publication");
        if(message.GetSource()!="iMAVLink"||!message.IsString())continue;
        const auto text=message.GetString();const auto data=fields(text);
        if(key=="MAVMOOS_RUNTIME_STATUS") {runtime=data;runtimeAt=now();log("runtime",key,text);}
        else if(key=="MAVMOOS_LANDED_STATE") {landed=data;landedAt=now();log("landed",key,text);}
        else if(key=="MAVMOOS_HOME_POSITION") {home=data;log("home",key,text);}
        else if(key=="MAVMOOS_CAPABILITIES")log("capabilities",key,text);
        else if(key=="MAVMOOS_MISSION_SNAPSHOT") {
          log("snapshot",key,text);
          if(text.find("\"status\":\"complete\"")!=std::string::npos)snapshots[snapshotId(text)]=text;
        } else if(key=="MAVMOOS_MISSION_TX_RESULT") {
          log("tx",key,text);
          const double producerAt=std::stod(data.at("at"));if(!std::isfinite(producerAt)||producerAt<0)throw std::runtime_error("invalid host send receipt time");
          if(data.at("operation_id")=="mf-upload"&&data.at("status")=="LOCAL_ACCEPTED") {
            oldAcceptedAt.push_back(producerAt);
            if(retired&&producerAt>=retiredHostAt)throw std::runtime_error("retired upload accepted at/after terminal host time");
          }
        } else if(key=="MAVMOOS_MISSION_RESULT") {
          log("result",key,text);const auto id=data.at("id"),state=data.at("status");status[id]=state;reason[id]=data.at("reason");
          if(id=="mf-upload") {
            if(state!="DUPLICATE"&&data.at("action")=="upload") {
              const double expires=std::stod(data.at("expires"));if(!std::isfinite(expires)||expires<=0)throw std::runtime_error("invalid original expiry");
              if(uploadExpires<0)uploadExpires=expires;else if(expires!=uploadExpires)throw std::runtime_error("upload expiry renewed");
            }
            if(state=="SENT") {
              if(retired)throw std::runtime_error("retired upload resumed");lastSent=std::stoul(data.at("seq"));
            }
            if(retired&&(state=="ADMITTED"||state=="SUBMITTED"||state=="RETRY_DUE"||state=="ACKNOWLEDGED"||state=="COMPLETED"))throw std::runtime_error("retired upload was replayed");
            if(state=="COMPLETED"||state=="ACKNOWLEDGED")throw std::runtime_error("large upload completed before cancellation");
            if(state=="CANCELLED"&&!retired) {
              const auto expected=scenario=="cancel"?"operator_cancelled":"connection_reset";
              if(reason[id]!=expected||!truth(data.at("mutation_may_have_applied"))||truth(data.at("remote_accepted"))||truth(data.at("readback_confirmed")))throw std::runtime_error("retirement lost partial-write uncertainty");
              retiredHostAt=std::stod(data.at("at"));if(!std::isfinite(retiredHostAt)||retiredHostAt<0)throw std::runtime_error("invalid terminal host time");
              for(const auto acceptedAt:oldAcceptedAt)if(acceptedAt>=retiredHostAt)throw std::runtime_error("retired upload accepted at/after terminal host time");
              retired=true;retiredAt=now();log("assertion","retirement",text);
            }
            if(state=="DUPLICATE")duplicate=true;
          }
        }
      }
      if(truth(runtime["armed"])||truth(runtime["permission"])||(!runtime["nav_tx_accepted"].empty()&&runtime["nav_tx_accepted"]!="0")||(!runtime["control_profile"].empty()&&runtime["control_profile"]!="none"))throw std::runtime_error("data-only mission fault acquired control");
      if(phase==Phase::Ready&&benchReady()) {
        log("assertion","stable_home_ground_fixture","home_initialized=true,armed=false,permission=false");request("id=mf-baseline,action=clear,timeout=20");transition(Phase::Baseline,"baseline_clear");
      } else if(phase==Phase::Baseline) {
        check("mf-baseline");if(completed("mf-baseline")&&snapshots.count("mf-baseline")) {
          if(count(snapshots["mf-baseline"])!=0)throw std::runtime_error("baseline not empty");request(upload());transition(Phase::Uploading,"partial_upload");
        }
      } else if(phase==Phase::Uploading) {
        check("mf-upload");if(lastSent>=4&&!triggered) {
          log("assertion","partial_trigger","sent_seq="+std::to_string(lastSent)+",requested_count=64");
          if(scenario=="cancel")request("id=mf-upload,action=cancel");
          else {std::ofstream marker(control+"/pause.request");marker<<"seq="<<lastSent<<'\n';marker.close();if(!marker)throw std::runtime_error("pause marker failed");log("injection_request","bridge_pause","requested");}
          triggered=true;transition(Phase::Retired,"await_retirement");
        }
      } else if(phase==Phase::Retired&&retired) {
        request(upload());transition(Phase::Duplicate,"duplicate_rejection");
      } else if(phase==Phase::Duplicate&&duplicate&&benchReady()) {
        log("assertion","old_id_not_replayed","status=DUPLICATE");request("id=mf-busy-read,action=download,timeout=5");transition(Phase::Busy,"remote_still_receiving");
      } else if(phase==Phase::Busy) {
        if(status["mf-busy-read"]=="FAILED") {
          if(reason["mf-busy-read"]!="remote_mission_error_14")throw std::runtime_error("busy remote had unexpected outcome: "+reason["mf-busy-read"]);
          log("assertion","remote_busy_after_local_retirement","status=FAILED,reason=remote_mission_error_14");transition(Phase::Settlement,"settlement_wait");
        } else if(!status["mf-busy-read"].empty()&&(status["mf-busy-read"]=="COMPLETED"||status["mf-busy-read"]=="TIMED_OUT"||status["mf-busy-read"]=="CANCELLED"||status["mf-busy-read"]=="REJECTED"))throw std::runtime_error("busy read did not observe remote denial");
      } else if(phase==Phase::Settlement&&now()-retiredAt>=10.5&&benchReady()) {
        log("assertion","settlement_wait","elapsed_since_retirement_s="+std::to_string(now()-retiredAt)+",native_source_timeout_s=8,remote_timeout_observed=false");
        request("id=mf-readback,action=download,timeout=20");transition(Phase::Readback,"fresh_readback");
      } else if(phase==Phase::Readback) {
        check("mf-readback");if(completed("mf-readback")&&snapshots.count("mf-readback")) {
          const auto size=count(snapshots["mf-readback"]);if(size<=1||size>=64)throw std::runtime_error("readback did not expose a partial prefix");
          log("assertion","partial_remote_storage","count="+std::to_string(size)+",requested_count=64,rollback=false");request("id=mf-final-clear,action=clear,timeout=20");transition(Phase::Clear,"final_verified_clear");
        }
      } else if(phase==Phase::Clear) {
        check("mf-final-clear");if(completed("mf-final-clear")&&snapshots.count("mf-final-clear")) {
          if(count(snapshots["mf-final-clear"])!=0)throw std::runtime_error("final clear not empty");
          if(runtime["mission_queue"]=="0"&&runtime["mission_operations"]=="0"&&runtime["mission_mutations"]=="0") {
            log("assertion","final_idle","mission_queue=0,mission_operations=0,mission_mutations=0,armed=false,permission=false");transition(Phase::Done,"done");
          }
        }
      }
      if(phase==Phase::Done){log("workflow","result","PASS");client.Close();return 0;}
      if(phase!=Phase::Ready&&now()-phaseAt>26)throw std::runtime_error("phase exceeded26s");
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    throw std::runtime_error("probe exceeded125s");
  } catch(const std::exception& error) {log("workflow","result",std::string("FAIL: ")+error.what());client.Close();return 1;}
}
