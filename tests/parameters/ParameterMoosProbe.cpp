#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
namespace {
double now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
std::map<std::string,std::string> fields(const std::string& text){std::map<std::string,std::string> r;std::istringstream in(text);std::string p;while(std::getline(in,p,',')){auto i=p.find('=');if(i!=std::string::npos)r[p.substr(0,i)]=p.substr(i+1);}return r;}
std::string quote(const std::string& text){std::string r="\"";for(char c:text){if(c=='"'||c=='\\')r+='\\';if(c=='\n')r+="\\n";else if(c=='\r')r+="\\r";else r+=c;}return r+'"';}
bool truth(const std::string& s){return s=="true"||s=="1";}
std::string number(double x){std::ostringstream s;s<<std::setprecision(9)<<x;return s.str();}
}
int main(int argc,char** argv){
  CMOOSCommClient client;const double start=now();bool original_known=false,may_have_written=false,restored=false;
  double original=0;
  auto log=[&](const std::string& type,const std::string& key,const std::string& value){std::cout<<"{\"elapsed\":"<<std::fixed<<std::setprecision(6)<<now()-start<<",\"type\":"<<quote(type)<<",\"key\":"<<quote(key)<<",\"value\":"<<quote(value)<<"}\n";std::cout.flush();};
  auto request=[&](const std::string& id,const std::string& action){const std::string text="id="+id+",action="+action;client.Notify("MAVMOOS_PARAM_REQUEST",text);log("request",id,text);};
  try {
    if(argc!=2)throw std::runtime_error("usage: parameter_moos_probe DB_PORT");
    client.SetQuiet(true);if(!client.Run("127.0.0.1",std::stoi(argv[1]),"mavmoos_parameter_probe",50))throw std::runtime_error("MOOS startup failed");
    enum class Phase {Ready,Frame,BenchList,List,Original,BenchWrite,Write,After,BenchRestore,Restore,Restored,Missing,Unsafe,Done};
    Phase phase=Phase::Ready;double phase_at=start;bool registered=false,seen_helm=false;
    std::map<std::string,std::string> runtime,landed,status,reason;
    std::string runtime_text,landed_text;
    double runtime_at=-1,landed_at=-1,last_runtime_log=-1;
    std::map<std::string,double> values;
    std::map<std::string,unsigned> expected,received;
    std::set<unsigned> list_indices;
    bool snapshot_received=false;
    auto transition=[&](Phase next,const std::string& text){phase=next;phase_at=now();log("phase",text,"entered");};
    auto completed=[&](const std::string& id){return status[id]=="COMPLETED";};
    auto check=[&](const std::string& id){if(status[id]=="FAILED"||status[id]=="REJECTED"||status[id]=="TIMED_OUT"||status[id]=="INCOMPLETE"||status[id]=="CANCELLED")throw std::runtime_error(id+":"+status[id]+":"+reason[id]);};
    auto benchReady=[&]{
      const double at=now();
      if(!client.IsConnected()||runtime_at<0||landed_at<0||at-runtime_at>.5||at-landed_at>.5||
         !truth(runtime["heartbeat_valid"])||!truth(runtime["landed_valid"])||truth(runtime["armed"])||
         truth(runtime["in_air"])||truth(runtime["permission"])||landed["present"]!="true"||
         landed["encoding_valid"]!="true"||landed["freshness"]!="FRESH"||landed["landed_state"]!="1")return false;
      // Both original acquisition and this probe use C++ steady_clock on this host.
      // Keep margin for MOOS delivery and the host's unchanged two-second guard.
      try {const double stamp=std::stod(landed["rx_mono_s"]),age=at-stamp;return std::isfinite(stamp)&&age>=0&&age<1.5;}
      catch(...){return false;}
    };
    auto logBenchReady=[&](const std::string& operation){
      log("assertion","bench_ready_"+operation,"runtime="+runtime_text+";landed="+landed_text);
    };
    while(now()-start<110){
      if(client.IsConnected()&&!registered){for(const auto& key:{"MAVMOOS_CAPABILITIES","MAVMOOS_STATUS","MAVMOOS_RUNTIME_STATUS","MAVMOOS_LANDED_STATE","MAVMOOS_PARAM_RESULT","MAVMOOS_PARAM_VALUE","MAVMOOS_PARAM_SNAPSHOT","IVPHELM_STATE","MAVMOOS_HELM_STATUS"})client.Register(key,0);registered=true;}
      MOOSMSG_LIST mail;client.Fetch(mail);
      for(const auto& message:mail){
        if(message.GetKey()=="IVPHELM_STATE"||message.GetKey()=="MAVMOOS_HELM_STATUS")seen_helm=true;
        if(message.GetSource()!="iMAVLink"||!message.IsString())continue;
        const auto key=message.GetKey(),text=message.GetString();auto f=fields(text);
        if(key=="MAVMOOS_CAPABILITIES")log("capabilities",key,text);
        if(key=="MAVMOOS_RUNTIME_STATUS"){
          runtime=std::move(f);runtime_at=now();runtime_text=text;
          if(last_runtime_log<0||now()-last_runtime_log>=.2){log("runtime_status",key,text);last_runtime_log=now();}
        }
        if(key=="MAVMOOS_LANDED_STATE"){landed=std::move(f);landed_at=now();landed_text=text;log("landed_status",key,text);}
        if(key=="MAVMOOS_STATUS")log("rx_status",key,text);
        if(key=="MAVMOOS_PARAM_RESULT"){status[f["id"]]=f["status"];reason[f["id"]]=f["reason"];expected[f["id"]]=std::stoul(f["expected"]);received[f["id"]]=std::stoul(f["received"]);log("result",key,text);}
        if(key=="MAVMOOS_PARAM_VALUE"){values[f["id"]]=std::stod(f["value"]);if(f["id"]=="params-list"&&f["phase"]=="list")list_indices.insert(std::stoul(f["index"]));log("value",key,text);}
        if(key=="MAVMOOS_PARAM_SNAPSHOT"&&text.find("\"id\":\"params-list\"")!=std::string::npos&&text.find("\"status\":\"complete\"")!=std::string::npos){snapshot_received=true;log("snapshot",key,text);}
      }
      if(seen_helm)throw std::runtime_error("parameter-only fixture unexpectedly ran a helm module");
      if(truth(runtime["permission"])||truth(runtime["armed"]))throw std::runtime_error("parameter test acquired navigation or armed");
      if(phase==Phase::Ready){
        if(truth(runtime["heartbeat_valid"])){log("assertion","read_without_navigation_prerequisites","home="+runtime["home_valid"]+",estimator="+runtime["estimator_valid"]+",permission="+runtime["permission"]);request("params-frame","read,name=FRAME_CLASS");transition(Phase::Frame,"read_frame");}
        else if(now()-phase_at>15)throw std::runtime_error("heartbeat never ready");
      }else if(phase==Phase::Frame){check("params-frame");if(completed("params-frame")){if(values["params-frame"]!=1)throw std::runtime_error("unexpected frame class");transition(Phase::BenchList,"wait_ground_before_streamed_list");}}
      else if(phase==Phase::BenchList){if(benchReady()){logBenchReady("streamed_list");request("params-list","list");transition(Phase::List,"list");}}
      else if(phase==Phase::List){check("params-list");if(completed("params-list")&&snapshot_received){if(expected["params-list"]<100||received["params-list"]!=expected["params-list"])throw std::runtime_error("not a complete native parameter table");log("assertion","complete_parameter_table","expected="+std::to_string(expected["params-list"])+",module_received="+std::to_string(received["params-list"])+",moos_indices_seen="+std::to_string(list_indices.size())+",snapshot_received=true; supervisor validates every JSON row");request("params-original","read,name=SIM_WIND_SPD");transition(Phase::Original,"original_wind");}}
      else if(phase==Phase::Original){check("params-original");if(completed("params-original")){original=values["params-original"];original_known=true;log("assertion","original_wind",number(original));transition(Phase::BenchWrite,"wait_ground_before_wind_write");}}
      else if(phase==Phase::BenchWrite){if(benchReady()){logBenchReady("wind_write");may_have_written=true;request("params-write","write,name=SIM_WIND_SPD,value=0.25,type=REAL32");transition(Phase::Write,"write_wind");}}
      else if(phase==Phase::Write){check("params-write");if(completed("params-write")){request("params-after","read,name=SIM_WIND_SPD");transition(Phase::After,"read_after_write");}}
      else if(phase==Phase::After){check("params-after");if(completed("params-after")){if(values["params-after"]!=.25)throw std::runtime_error("read after write mismatch");transition(Phase::BenchRestore,"wait_ground_before_wind_restore");}}
      else if(phase==Phase::BenchRestore){if(benchReady()){logBenchReady("wind_restore");request("params-restore","write,name=SIM_WIND_SPD,value="+number(original));transition(Phase::Restore,"restore_wind");}}
      else if(phase==Phase::Restore){check("params-restore");if(completed("params-restore")){request("params-restored","read,name=SIM_WIND_SPD");transition(Phase::Restored,"read_restored");}}
      else if(phase==Phase::Restored){check("params-restored");if(completed("params-restored")){if(values["params-restored"]!=original)throw std::runtime_error("restore read mismatch");restored=true;request("params-missing","read,name=NO_SUCH_PARAM");transition(Phase::Missing,"missing_parameter");}}
      else if(phase==Phase::Missing){if(status["params-missing"]=="FAILED"&&reason["params-missing"].rfind("parameter_error_",0)==0){log("assertion","missing_parameter_error",reason["params-missing"]);request("params-unsafe","write,name=ATC_RAT_RLL_P,value=0.1");transition(Phase::Unsafe,"reject_unsafe_write");}else if(status["params-missing"]=="TIMED_OUT"){log("assertion","missing_parameter_timeout",reason["params-missing"]);request("params-unsafe","write,name=ATC_RAT_RLL_P,value=0.1");transition(Phase::Unsafe,"reject_unsafe_write");}}
      else if(phase==Phase::Unsafe){if(status["params-unsafe"]=="REJECTED"){log("assertion","unsafe_write_rejected",reason["params-unsafe"]);transition(Phase::Done,"done");}}
      else if(phase==Phase::Done){log("pass","parameter_workflow","no helm/no navigation; native read/list; write+explicit readback; independent read; restore; missing/error; unsafe write rejected");client.Close();return 0;}
      if(phase!=Phase::Ready&&now()-phase_at>(phase==Phase::List?25:12))throw std::runtime_error("parameter phase deadline; runtime="+runtime_text+";landed="+landed_text);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    throw std::runtime_error("parameter workflow deadline");
  }catch(const std::exception& error){
    log("fail","parameter_workflow",error.what());
    if(original_known&&may_have_written&&!restored&&client.IsConnected()){
      request("params-cleanup","write,name=SIM_WIND_SPD,value="+number(original));
      const double deadline=now()+5;bool cleanup=false;
      while(now()<deadline&&!cleanup){MOOSMSG_LIST mail;client.Fetch(mail);for(const auto& m:mail)if(m.GetSource()=="iMAVLink"&&m.GetKey()=="MAVMOOS_PARAM_RESULT"&&m.IsString()){auto f=fields(m.GetString());log("cleanup",m.GetKey(),m.GetString());cleanup=f["id"]=="params-cleanup"&&f["status"]=="COMPLETED";}std::this_thread::sleep_for(std::chrono::milliseconds(20));}
      log("cleanup","restore_attempt",cleanup?"confirmed":"not_confirmed; fixture teardown will destroy isolated simulator");
    }
    client.Close();return 1;
  }
}
