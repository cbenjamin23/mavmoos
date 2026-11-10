// Test-only native bench coordinator. All effects go through the MOOS provider.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <thread>
namespace {
double now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
std::map<std::string,std::string> fields(const std::string& text){std::map<std::string,std::string> r;std::istringstream in(text);std::string p;while(std::getline(in,p,',')){const auto i=p.find('=');if(i!=std::string::npos)r[p.substr(0,i)]=p.substr(i+1);}return r;}
bool truth(const std::string& s){return s=="1"||s=="true";}
std::string quote(const std::string& text){std::string r="\"";for(char c:text){if(c=='"'||c=='\\')r+='\\';if(c=='\n')r+="\\n";else if(c=='\r')r+="\\r";else r+=c;}return r+'"';}
unsigned count(const std::string& text){const auto at=text.find("\"count\":");if(at==std::string::npos)throw std::runtime_error("snapshot count missing");return std::stoul(text.substr(at+8));}
}
int main(int argc,char** argv){
  CMOOSCommClient client;const double started=now();
  auto log=[&](const std::string& type,const std::string& key,const std::string& value){std::cout<<"{\"elapsed\":"<<std::fixed<<std::setprecision(6)<<now()-started<<",\"type\":"<<quote(type)<<",\"key\":"<<quote(key)<<",\"value\":"<<quote(value)<<"}\n";std::cout.flush();};
  auto request=[&](const std::string& id,const std::string& action){const auto text="id="+id+",action="+action;client.Notify("MAVMOOS_MISSION_REQUEST",text);log("request",id,text);};
  try{
    if(argc!=2)throw std::runtime_error("usage: mission_moos_probe DB_PORT");
    client.SetQuiet(true);if(!client.Run("127.0.0.1",std::stoi(argv[1]),"mavmoos_mission_probe",50))throw std::runtime_error("MOOS client failed");
    enum class Phase {Ready,Initial,BenchUpload,Upload,Downloaded,BenchClear,Clear,Empty,Unsupported,Done};
    Phase phase=Phase::Ready;double phase_at=started;bool registered=false;double runtime_at=-1,landed_at=-1;
    std::map<std::string,std::string> runtime,landed,status,reasons,snapshots;std::string runtime_text,landed_text;
    auto transition=[&](Phase next,const std::string& text){phase=next;phase_at=now();log("phase",text,"entered");};
    auto completed=[&](const std::string& id){return status[id]=="COMPLETED";};
    auto check=[&](const std::string& id){if(status[id]=="FAILED"||status[id]=="REJECTED"||status[id]=="TIMED_OUT"||status[id]=="INCOMPLETE"||status[id]=="CANCELLED")throw std::runtime_error(id+":"+status[id]+":"+reasons[id]);};
    auto benchReady=[&]{
      const double at=now();if(runtime_at<0||landed_at<0||at-runtime_at>.5||at-landed_at>.5||!truth(runtime["heartbeat_valid"])||
        !truth(runtime["landed_valid"])||truth(runtime["armed"])||truth(runtime["permission"])||
        landed["present"]!="true"||landed["encoding_valid"]!="true"||landed["freshness"]!="FRESH"||landed["landed_state"]!="1")return false;
      try{const double stamp=std::stod(landed["rx_mono_s"]);return std::isfinite(stamp)&&at>=stamp&&at-stamp<1.5;}catch(...){return false;}
    };
    while(now()-started<100){
      if(client.IsConnected()&&!registered){for(const auto& variable:{"MAVMOOS_CAPABILITIES","MAVMOOS_STATUS","MAVMOOS_RUNTIME_STATUS","MAVMOOS_LANDED_STATE","MAVMOOS_MISSION_RESULT","MAVMOOS_MISSION_SNAPSHOT","MAVMOOS_MISSION_PROGRESS","MAVMOOS_MISSION_TX_RESULT","IVPHELM_STATE","MAVMOOS_HELM_STATUS"})client.Register(variable,0);registered=true;}
      MOOSMSG_LIST mail;client.Fetch(mail);
      for(const auto& message:mail){
        if(message.GetKey()=="IVPHELM_STATE"||message.GetKey()=="MAVMOOS_HELM_STATUS")throw std::runtime_error("mission-only bench unexpectedly ran helm");
        if(message.GetSource()!="iMAVLink"||!message.IsString())continue;
        const auto key=message.GetKey(),text=message.GetString();const auto f=fields(text);
        if(key=="MAVMOOS_RUNTIME_STATUS"){runtime=f;runtime_at=now();runtime_text=text;log("runtime",key,text);}
        else if(key=="MAVMOOS_LANDED_STATE"){landed=f;landed_at=now();landed_text=text;log("landed",key,text);}
        else if(key=="MAVMOOS_MISSION_RESULT"){status[f.at("id")]=f.at("status");reasons[f.at("id")]=f.at("reason");log("result",key,text);}
        else if(key=="MAVMOOS_MISSION_SNAPSHOT"&&text.find("\"status\":\"complete\"")!=std::string::npos){
          const auto begin=text.find("\"id\":\"");if(begin==std::string::npos)throw std::runtime_error("snapshot id missing");const auto end=text.find('"',begin+6);snapshots[text.substr(begin+6,end-(begin+6))]=text;log("snapshot",key,text);
        }else log(key=="MAVMOOS_CAPABILITIES"?"capabilities":"observation",key,text);
      }
      if(truth(runtime["armed"])||truth(runtime["permission"]))throw std::runtime_error("mission-data bench armed or acquired NAV");
      if(phase==Phase::Ready){if(truth(runtime["heartbeat_valid"])){log("assertion","download_without_control","home="+runtime["home_valid"]+",estimator="+runtime["estimator_valid"]+",permission="+runtime["permission"]);request("missions-initial","download");transition(Phase::Initial,"initial_download");}else if(now()-phase_at>15)throw std::runtime_error("heartbeat never ready");}
      else if(phase==Phase::Initial){check("missions-initial");if(completed("missions-initial")&&snapshots.count("missions-initial")){if(count(snapshots["missions-initial"])!=0)throw std::runtime_error("fresh --wipe simulator did not have empty mission");transition(Phase::BenchUpload,"wait_disarmed_ground_before_upload");}}
      else if(phase==Phase::BenchUpload){if(benchReady()){
        log("assertion","bench_ready_upload","runtime="+runtime_text+";landed="+landed_text);
        request("missions-upload","upload,items=0:16:0:0:1:0:0:0:0:423584560:-710875890:10|1:16:3:0:1:0:0:0:0:423584660:-710875890:5|2:16:3:0:1:0:0:0:0:423584760:-710875790:5");transition(Phase::Upload,"upload_and_readback");}}
      else if(phase==Phase::Upload){check("missions-upload");if(completed("missions-upload")&&snapshots.count("missions-upload")){if(count(snapshots["missions-upload"])!=3)throw std::runtime_error("upload/readback count mismatch");log("assertion","upload_verified","three items including dynamic HOME; supervisor validates observed waypoint semantics");request("missions-download","download");transition(Phase::Downloaded,"independent_module_download");}}
      else if(phase==Phase::Downloaded){check("missions-download");if(completed("missions-download")&&snapshots.count("missions-download")){if(count(snapshots["missions-download"])!=3)throw std::runtime_error("subsequent download count mismatch");transition(Phase::BenchClear,"wait_disarmed_ground_before_clear");}}
      else if(phase==Phase::BenchClear){if(benchReady()){log("assertion","bench_ready_clear","runtime="+runtime_text+";landed="+landed_text);request("missions-clear","clear");transition(Phase::Clear,"clear_and_empty_readback");}}
      else if(phase==Phase::Clear){check("missions-clear");if(completed("missions-clear")&&snapshots.count("missions-clear")){if(count(snapshots["missions-clear"])!=0)throw std::runtime_error("clear not empty");request("missions-empty","download");transition(Phase::Empty,"final_empty_download");}}
      else if(phase==Phase::Empty){check("missions-empty");if(completed("missions-empty")&&snapshots.count("missions-empty")){if(count(snapshots["missions-empty"])!=0)throw std::runtime_error("final mission not empty");request("missions-start-denied","start");transition(Phase::Unsupported,"execution_request_not_supported");}}
      else if(phase==Phase::Unsupported){if(status["missions-start-denied"]=="REJECTED"){log("assertion","execution_not_exposed","start rejected by mission data provider; no control owner loaded");transition(Phase::Done,"done");}}
      else if(phase==Phase::Done){log("pass","mission_data_workflow","empty download; upload+mandatory readback; independent download; clear+empty readback; final download; no helm/NAV/arm");client.Close();return 0;}
      if(phase!=Phase::Ready&&now()-phase_at>(phase==Phase::Upload?25:15))throw std::runtime_error("mission bench phase timeout;runtime="+runtime_text+";landed="+landed_text);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    throw std::runtime_error("mission bench whole timeout");
  }catch(const std::exception& error){log("fail","mission_data_workflow",error.what());client.Close();return 1;}
}
