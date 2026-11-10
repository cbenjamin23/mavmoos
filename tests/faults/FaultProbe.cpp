#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"
#include "../faults_coordination/MarkerWait.h"
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace {
double mono() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
bool yes(const std::string& s) { return s=="1"||s=="true"; }
std::map<std::string,std::string> fields(const std::string& s) {
  std::map<std::string,std::string> r;std::istringstream in(s);std::string p;
  while(std::getline(in,p,',')){auto at=p.find('=');if(at!=std::string::npos)r[p.substr(0,at)]=p.substr(at+1);}
  return r;
}
std::string quote(const std::string& s) {
  std::string r="\"";for(char c:s){if(c=='"'||c=='\\')r+='\\';if(c=='\n')r+="\\n";else if(c=='\r')r+="\\r";else r+=c;}return r+'"';
}
struct Probe {
  CMOOSCommClient client;
  std::filesystem::path control;
  std::map<std::string,std::string> runtime,result,reason,tx_status,ingress;
  std::map<std::string,unsigned> tx_count;
  std::string helm,last_runtime,last_helm,stock_allstop;
  bool registered=false,connected=false;
  double start=mono(),runtime_at=-1,last_summary=-1;
  void log(const std::string& kind,const std::string& key,const std::string& value) {
    const auto now=mono();
    std::cout<<std::setprecision(12)<<"{\"elapsed\":"<<now-start<<",\"mono\":"<<now<<",\"type\":"<<quote(kind)<<",\"key\":"<<quote(key)<<",\"value\":"<<quote(value)<<"}\n";
    std::cout.flush();
  }
  void notify(const std::string& key,const std::string& value) { client.Notify(key,value);log("operator",key,value); }
  void pump() {
    const bool present=client.IsConnected();
    if(present!=connected){log("connection","MOOSDB",present?"connected":"disconnected");connected=present;}
    if(!present){registered=false;runtime_at=-1;return;}
    if(!registered){
      for(const char* key:{"MAVMOOS_CAPABILITIES","MAVMOOS_RUNTIME_STATUS","MAVMOOS_HELM_STATUS","MAVMOOS_COMMAND_RESULT","MAVMOOS_TX_RESULT","MAVMOOS_STATUS","IVPHELM_ALLSTOP"})client.Register(key,0);
      registered=true;runtime.clear();helm.clear();stock_allstop.clear();runtime_at=-1;
    }
    MOOSMSG_LIST mail;client.Fetch(mail);
    for(const auto& msg:mail){
      if(msg.GetSource()=="pHelmIvP"&&msg.GetKey()=="IVPHELM_ALLSTOP"&&msg.IsString()){
        stock_allstop=msg.GetString();log("stock_helm","IVPHELM_ALLSTOP",stock_allstop);continue;
      }
      if(msg.GetSource()!="iMAVLink"||!msg.IsString())continue;
      auto key=msg.GetKey(),text=msg.GetString();
      if(key=="MAVMOOS_CAPABILITIES"){log("capabilities",key,text);}
      else if(key=="MAVMOOS_RUNTIME_STATUS"){runtime=fields(text);runtime_at=mono();last_runtime=text;}
      else if(key=="MAVMOOS_STATUS") ingress=fields(text);
      else if(key=="MAVMOOS_HELM_STATUS"){helm=text;last_helm=text;}
      else if(key=="MAVMOOS_COMMAND_RESULT"){
        auto p=fields(text);result[p["id"]]=p["status"];reason[p["id"]]=p["reason"];
        log("command_result",key,text);
      } else if(key=="MAVMOOS_TX_RESULT"){
        auto p=fields(text);++tx_count[p["id"]];tx_status[p["id"]]=p["status"];log("tx_result",key,text);
      }
    }
    if(mono()-last_summary>.25){
      if(!last_runtime.empty())log("runtime","state",last_runtime);
      if(!last_helm.empty())log("helm","state",last_helm);
      last_summary=mono();
    }
  }
  void observe(double seconds) { auto until=mono()+seconds;while(mono()<until){pump();std::this_thread::sleep_for(std::chrono::milliseconds(20));} }
  void wait(const std::function<bool()>& predicate,double timeout,const std::string& why) {
    const auto until=mono()+timeout;
    while(mono()<until){pump();if(predicate())return;std::this_thread::sleep_for(std::chrono::milliseconds(20));}
    throw std::runtime_error(why+"; runtime="+last_runtime+"; helm="+last_helm);
  }
  bool freshStatus() const { return connected&&runtime_at>0&&mono()-runtime_at<.75; }
  bool ready() {return freshStatus()&&yes(runtime["heartbeat_valid"])&&yes(runtime["position_valid"])&&yes(runtime["home_valid"])&&yes(runtime["estimator_valid"])&&yes(runtime["landed_valid"]);}
  bool active() {return ready()&&yes(runtime["permission"])&&runtime["fc_mode"]=="4"&&helm.find("state=ACTIVE")!=std::string::npos;}
  unsigned nav() {return runtime.count("nav_tx_accepted")?std::stoul(runtime["nav_tx_accepted"]):0;}
  void marker(const std::string& name){std::ofstream(control/name)<<mono()<<'\n';log("coordinator",name,"requested");}
  bool markerExists(const std::string& name){return std::filesystem::exists(control/name);}
  void waitMarker(const std::string& name,double timeout,const std::string& why) {
    if(!mavmoos::test::waitForMarker([&]{return markerExists(name);},timeout,mono,
        [](double seconds){std::this_thread::sleep_for(std::chrono::duration<double>(seconds));}))
      throw std::runtime_error(why+"; marker="+name+"; last_runtime="+last_runtime+"; last_helm="+last_helm);
    log("coordinator_ack",name,"observed; producer creation time not inferred");
  }
  void command(const std::string& id,const std::string& action,double timeout=10) {
    notify("MAVMOOS_COMMAND","id="+id+",action="+action);
    wait([&]{auto s=result[id];if(s=="FAILED"||s=="TIMED_OUT"||s=="REJECTED"||s=="CANCELLED")throw std::runtime_error(id+":"+s+":"+reason[id]);return s=="COMPLETED";},timeout,"command completion timeout "+id);
  }
};
}
int main(int argc,char** argv){
  Probe p;
  try{
    if(argc!=4)throw std::runtime_error("usage: fault_probe DB_PORT SCENARIO CONTROL_DIR");
    const std::string scenario=argv[2];p.control=argv[3];std::filesystem::create_directories(p.control);
    p.client.SetQuiet(true);if(!p.client.Run("127.0.0.1",std::stoi(argv[1]),"mavmoos_fault_probe",30))throw std::runtime_error("MOOS client start");
    p.log("phase","ready",scenario);
    p.wait([&]{return p.ready();},55,"initial vehicle readiness");
    if(yes(p.runtime["armed"])||yes(p.runtime["in_air"]))throw std::runtime_error("simulator did not start disarmed/on-ground");
    if(scenario=="command_timeout"){
      p.marker("inject.request");p.waitMarker("injected",3,"inject acknowledgement");
      p.notify("MAVMOOS_COMMAND","id=fault-late-guided,action=guided");
      p.wait([&]{if(p.result["fault-late-guided"]=="REJECTED")throw std::runtime_error("command rejected before timeout test: "+p.reason["fault-late-guided"]);return p.result["fault-late-guided"]=="TIMED_OUT";},1.3,"missing ACK/state timeout");
      if(p.tx_count["fault-late-guided"]!=1||p.tx_status["fault-late-guided"]!="LOCAL_ACCEPTED")throw std::runtime_error("timeout transaction did not make exactly one locally accepted attempt");
      p.marker("restore.request");p.waitMarker("restored",4,"restore acknowledgement");
      p.wait([&]{return p.ready();},8,"fresh telemetry after ACK delay");p.observe(2);
      if(p.result["fault-late-guided"]!="TIMED_OUT"||p.tx_count["fault-late-guided"]!=1||yes(p.runtime["permission"]))throw std::runtime_error("late ACK/state revived old transaction or hidden retry");
      p.log("assertion","late_command_outcome","TIMED_OUT stays terminal; one local attempt; remote mode may have changed after timeout");
      p.command("fault-fresh-guided","guided");
      p.log("pass",scenario,"late result did not complete old operation; a fresh explicit request worked");
      p.client.Close();return 0;
    }
    p.command("fault-guided","guided");p.command("fault-arm","arm");p.command("fault-takeoff","takeoff,height=5",48);
    p.notify("MOOS_MANUAL_OVERRIDE","false");p.notify("DEPLOY","true");
    const double original_acquire_time=MOOSTime();
    if(!p.client.Notify("MAVMOOS_NAV_CONTROL",std::string("acquire"),original_acquire_time))
      throw std::runtime_error("initial acquisition publication failed");
    p.log("operator","MAVMOOS_NAV_CONTROL","acquire");
    p.wait([&]{return p.active();},8,"initial helm acquisition");p.observe(1.5);
    if(!p.active()||p.nav()<5)throw std::runtime_error("navigation never established before fault");
    p.log("assertion","pre_fault_navigation","ACTIVE and local navigation acceptance observed");
    p.marker("inject.request");p.waitMarker("injected",3,"inject acknowledgement");
    p.observe(3.3);
    if(scenario!="moosdb_restart"){
      p.wait([&]{return p.freshStatus()&&!yes(p.runtime["permission"]);},2,"navigation not revoked during fault");
      const auto stopped=p.nav();p.observe(.8);if(p.nav()!=stopped)throw std::runtime_error("navigation continued after observed revocation");
      p.log("assertion","fault_revocation","NAV accepted counter stopped while runtime remains observable");
    }
    const auto before_restore_nav=p.nav();
    p.marker("restore.request");p.waitMarker("restored",6,"restore acknowledgement");
    if(scenario=="simulator_restart"){
      p.observe(3);
      if(!p.freshStatus()||yes(p.runtime["permission"])||p.nav()!=before_restore_nav)throw std::runtime_error("replacement simulator revived old bound session");
      p.log("pass",scenario,"replacement native process cannot reuse old configured PID/peer; explicit rebind required");
      p.client.Close();return 0;
    }
    p.wait([&]{return p.ready();},15,"fresh observations did not return after fault");
    const auto stopped=p.nav();p.observe(2);
    if(yes(p.runtime["permission"])||p.nav()!=stopped||p.helm.find("state=LATCHED")==std::string::npos)throw std::runtime_error("fresh returning data automatically revived navigation");
    p.log("assertion","no_automatic_reacquisition","fresh data returned for 2s while NAV stayed inhibited");
    if(scenario=="bridge_pause") {
      if(!p.ingress.count("queue_stale_dropped") || std::stoull(p.ingress["queue_stale_dropped"])==0)
        throw std::runtime_error("bridge pause did not expose discarded socket backlog");
      const auto diagnostic=fields(p.helm);
      if(diagnostic.at("next_action")!="EXPLICIT_ACQUIRE_WITH_NEW_INPUTS")throw std::runtime_error("recovery advice did not request explicit acquisition");
      p.log("assertion","bridge_pause_backlog", "old socket traffic discarded; current observations returned without reclaiming permission");
      p.log("ingress","status",p.ingress["queue_stale_dropped"]+" stale datagrams discarded");
    }
    if(scenario=="moosdb_restart"){
      // A new MOOSDB has lost event-driven ALLSTOP state. Deliberately use
      // stock operator interfaces to resynchronize it; this is not a state query.
      p.notify("MOOS_MANUAL_OVERRIDE","true");
      p.wait([&]{return p.stock_allstop=="ManualOverride";},4,"stock helm did not confirm operator stop after DB restart");
      p.notify("MOOS_MANUAL_OVERRIDE","false");
      p.wait([&]{return p.stock_allstop=="clear";},4,"stock helm did not republish clear after explicit resynchronization");
      p.observe(.7);
      if(!p.freshStatus()||yes(p.runtime["permission"])||p.nav()!=stopped)throw std::runtime_error("helm resynchronization implicitly reacquired navigation");
      p.log("assertion","explicit_helm_resynchronization","observed stock ManualOverride then clear; navigation still inhibited");
      if(!p.client.Notify("MAVMOOS_NAV_CONTROL",std::string("acquire"),original_acquire_time))
        throw std::runtime_error("old acquire replay publication failed");
      p.log("operator_replay","MAVMOOS_NAV_CONTROL","acquire,published="+std::to_string(original_acquire_time));
      p.observe(.8);
      if(!p.freshStatus()||yes(p.runtime["permission"])||p.nav()!=stopped)
        throw std::runtime_error("an acquire replayed with its original MOOS timestamp reopened authority");
      p.log("assertion","old_acquire_inhibited","acquire replay kept its original pre-restart timestamp");
    }
    p.notify("DEPLOY","true");p.notify("MAVMOOS_NAV_CONTROL","acquire");
    p.wait([&]{return p.active();},8,"fresh explicit reacquisition failed");p.observe(.5);
    if(p.nav()<=stopped)throw std::runtime_error("explicit reacquisition did not make fresh navigation attempts");
    p.log("assertion","explicit_reacquisition","fresh operator request restored eligible navigation");
    p.notify("DEPLOY","false");p.command("fault-land","land",65);
    if(yes(p.runtime["armed"]))p.command("fault-disarm","disarm");
    p.wait([&]{return p.freshStatus()&&!yes(p.runtime["armed"])&&!yes(p.runtime["in_air"])&&!yes(p.runtime["permission"]);},3,"final ground/disarmed state");
    p.log("pass",scenario,"revocation, fresh-data inhibition, explicit reacquire and land/disarm passed");
    p.client.Close();return 0;
  }catch(const std::exception& e){p.log("fail","fault_probe",e.what());p.client.Close();return 1;}
}
