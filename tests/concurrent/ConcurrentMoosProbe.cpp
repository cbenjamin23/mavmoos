#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include "../firmware/MissionFixtureShape.h"
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace {
double mono() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
bool yes(const std::string& s) { return s=="1"||s=="true"; }
std::map<std::string,std::string> fields(const std::string& s) {
  std::map<std::string,std::string> r;std::istringstream in(s);std::string part;
  while(std::getline(in,part,',')){auto at=part.find('=');if(at!=std::string::npos)r[part.substr(0,at)]=part.substr(at+1);}
  return r;
}
std::string quote(const std::string& s) {
  std::string r="\"";for(unsigned char c:s){if(c=='"'||c=='\\'){r+='\\';r+=static_cast<char>(c);}else if(c=='\n')r+="\\n";else if(c=='\r')r+="\\r";else if(c>=32)r+=static_cast<char>(c);}return r+'"';
}
struct Probe {
  CMOOSCommClient client;
  std::filesystem::path control;
  std::string scenario,helm,heartbeat,home;
  mavmoos::test::MissionFixtureShape mission_fixture{"bound_home_only"};
  std::map<std::string,std::string> runtime,ingress,commands,command_reason,params,param_reason,missions,mission_reason;
  std::map<std::string,double> values;
  std::map<std::string,unsigned> param_tx;
  std::map<unsigned,unsigned> mission_tx;
  std::set<unsigned> list_indices;
  unsigned list_expected=0,last_nav=0,list_nav_start=0;
  bool registered=false,monitor=false,loss=false,injection_requested=false,injection_observed=false,snapshot=false,vehicle_session_reset_seen=false,paced_progress_seen=false,mission_snapshot=false;
  double started=mono(),runtime_at=-1,last_summary=-1,last_nav_update=-1,max_nav_update_gap=0,injection_observed_at=-1;
  void log(const std::string& type,const std::string& key,const std::string& value) {
    std::cout<<std::setprecision(16)<<"{\"elapsed\":"<<mono()-started<<",\"mono\":"<<mono()<<",\"type\":"<<quote(type)<<",\"key\":"<<quote(key)<<",\"value\":"<<quote(value)<<"}\n";std::cout.flush();
  }
  void notify(const std::string& key,const std::string& value) {client.Notify(key,value);log("request",key,value);}
  bool terminal(const std::string& id) {return params[id]=="COMPLETED"||params[id]=="INCOMPLETE"||params[id]=="TIMED_OUT"||params[id]=="CANCELLED"||params[id]=="FAILED"||params[id]=="REJECTED";}
  bool statusFresh() {return client.IsConnected()&&runtime_at>0&&mono()-runtime_at<.75;}
  bool ready() {return statusFresh()&&yes(runtime["heartbeat_valid"])&&yes(runtime["position_valid"])&&yes(runtime["landed_valid"])&&yes(runtime["home_valid"])&&yes(runtime["estimator_valid"]);}
  bool active() {return ready()&&yes(runtime["permission"])&&runtime["fc_mode"]=="4"&&helm.find("state=ACTIVE")!=std::string::npos;}
  unsigned nav() {return runtime.count("nav_tx_accepted")?std::stoul(runtime["nav_tx_accepted"]):0;}
  void writeListState() {
    {std::ofstream(control/"list-state.tmp")<<"{\"pending\":"<<((terminal("concurrent-list")||snapshot)?"false":"true")<<",\"seen_indices\":"<<list_indices.size()<<",\"expected\":"<<list_expected<<",\"max_index\":"<<(list_indices.empty()?0:*list_indices.rbegin())<<"}\n";}
    std::filesystem::rename(control/"list-state.tmp",control/"list-state.json");
  }
  void pump() {
    if(!client.IsConnected()) {registered=false;runtime_at=-1;return;}
    if(!registered) {
      for(const char* key:{"MAVMOOS_CAPABILITIES","MAVMOOS_RUNTIME_STATUS","MAVMOOS_STATUS","MAVMOOS_HEARTBEAT","MAVMOOS_HOME_POSITION","MAVMOOS_HELM_STATUS","MAVMOOS_COMMAND_RESULT","MAVMOOS_PARAM_RESULT","MAVMOOS_PARAM_VALUE","MAVMOOS_PARAM_SNAPSHOT","MAVMOOS_PARAM_TX_RESULT","MAVMOOS_MISSION_RESULT","MAVMOOS_MISSION_SNAPSHOT","MAVMOOS_MISSION_TX_RESULT"})client.Register(key,0);
      registered=true;
    }
    MOOSMSG_LIST mail;client.Fetch(mail);
    for(const auto& m:mail) {
      if(m.GetSource()!="iMAVLink"||!m.IsString())continue;
      const auto key=m.GetKey(),text=m.GetString();auto f=fields(text);
      if(key=="MAVMOOS_CAPABILITIES") {log("capabilities",key,text);}
      else if(key=="MAVMOOS_RUNTIME_STATUS") {runtime=f;runtime_at=mono();const auto count=nav();if(count<last_nav)throw std::runtime_error("NAV counter regressed");if(count>last_nav){if(monitor&&last_nav_update>0)max_nav_update_gap=std::max(max_nav_update_gap,mono()-last_nav_update);last_nav_update=mono();last_nav=count;log("nav_counter",key,text);}}
      else if(key=="MAVMOOS_STATUS")ingress=f;
      else if(key=="MAVMOOS_HEARTBEAT")heartbeat=text;
      else if(key=="MAVMOOS_HOME_POSITION") {home=text;log("home_observation",key,text);}
      else if(key=="MAVMOOS_HELM_STATUS") {helm=text;if(monitor&&text.find("reason=reset:vehicle_connection_lost,")!=std::string::npos)vehicle_session_reset_seen=true;}
      else if(key=="MAVMOOS_COMMAND_RESULT") {commands[f["id"]]=f["status"];command_reason[f["id"]]=f["reason"];log("command_result",key,text);}
      else if(key=="MAVMOOS_PARAM_RESULT") {params[f["id"]]=f["status"];param_reason[f["id"]]=f["reason"];log("parameter_result",key,text);if(f["id"]=="concurrent-list"&&f["list_mode"]!="paced")throw std::runtime_error("concurrent fixture did not select paced list mode");if(injection_observed&&f["id"]=="concurrent-list"&&f["status"]=="COMPLETED"&&std::stod(f["at"])<=injection_observed_at)throw std::runtime_error("list completed before bridge pause; interruption fixture invalid");}
      else if(key=="MAVMOOS_PARAM_VALUE") {values[f["id"]]=std::stod(f["value"]);if(f["id"]=="concurrent-list"&&(f["phase"]=="list"||f["phase"]=="paced_list")){list_indices.insert(std::stoul(f["index"]));list_expected=std::stoul(f["count"]);if(f["phase"]=="paced_list")paced_progress_seen=true;}log("parameter_value",key,text);}
      else if(key=="MAVMOOS_PARAM_TX_RESULT") {++param_tx[f["id"]];log("parameter_tx",key,text);}
      else if(key=="MAVMOOS_PARAM_SNAPSHOT"&&text.find("\"id\":\"concurrent-list\"")!=std::string::npos&&text.find("\"status\":\"complete\"")!=std::string::npos) {snapshot=true;log("snapshot",key,text);}
      else if(key=="MAVMOOS_MISSION_RESULT") {missions[f["id"]]=f["status"];mission_reason[f["id"]]=f["reason"];log("mission_result",key,text);}
      else if(key=="MAVMOOS_MISSION_SNAPSHOT"&&text.find("\"id\":\"concurrent-mission-read\"")!=std::string::npos&&text.find("\"status\":\"complete\"")!=std::string::npos) {mission_snapshot=true;log("mission_snapshot",key,text);}
      else if(key=="MAVMOOS_MISSION_TX_RESULT") {
        log("mission_tx",key,text);
        if(scenario=="mission_read") {
          if(f["operation_id"]!="concurrent-mission-read"||f["status"]!="LOCAL_ACCEPTED"||f["late"]!="false"||f["clock_valid"]!="true"||!f.count("msgid"))throw std::runtime_error("unexpected mission send outcome");
          const auto id=std::stoul(f["msgid"]);if(!mission_fixture.allows(id))throw std::runtime_error("read-only mission scenario emitted a mutation or undeclared item request");
          ++mission_tx[id];
        }
      }
    }
    if(monitor) {
      if(!yes(runtime["permission"])||helm.find("state=LATCHED")!=std::string::npos)loss=true;
      if(scenario=="bulk"&&loss)throw std::runtime_error("bulk parameter transfer interrupted active navigation: "+helm);
      if(scenario=="mission_read"&&(!active()||runtime["mission_mutations"]!="0"))throw std::runtime_error("mission/parameter read interrupted active navigation or held mutation lease: "+runtimeText()+";"+helm);
      if(scenario!="mission_read")writeListState();
      if(scenario=="bridge_pause"&&!injection_requested&&!snapshot&&list_expected>100&&list_indices.size()>0&&*list_indices.rbegin()<list_expected-50&&!terminal("concurrent-list")) {
        std::ofstream(control/"inject.request")<<std::setprecision(16)<<mono()<<'\n';injection_requested=true;log("coordinator","inject.request","list is pending with indexed progress");
      }
      if(injection_requested&&!injection_observed&&std::filesystem::exists(control/"injected")) {
        injection_observed=true;injection_observed_at=mono();
        if(terminal("concurrent-list")||snapshot)throw std::runtime_error("list terminal before injected pause observed");
        log("coordinator","injected_observed","pending list confirmed while app stopped");
      }
    }
    if(mono()-last_summary>.2) {if(!runtime.empty())log("runtime","status",runtimeText());if(!helm.empty())log("helm","status",helm);last_summary=mono();}
  }
  std::string runtimeText() {std::string r;for(const auto& p:runtime){if(!r.empty())r+=',';r+=p.first+'='+p.second;}return r;}
  void wait(const std::function<bool()>& condition,double timeout,const std::string& why) {
    const auto until=mono()+timeout;while(mono()<until){pump();if(condition())return;std::this_thread::sleep_for(std::chrono::milliseconds(10));}
    throw std::runtime_error(why+"; runtime="+runtimeText()+"; helm="+helm);
  }
  void observe(double seconds) {const auto until=mono()+seconds;while(mono()<until){pump();std::this_thread::sleep_for(std::chrono::milliseconds(10));}}
  void command(const std::string& id,const std::string& action,double timeout=10) {
    notify("MAVMOOS_COMMAND","id="+id+",action="+action);
    wait([&]{if(commands[id]=="FAILED"||commands[id]=="TIMED_OUT"||commands[id]=="REJECTED"||commands[id]=="CANCELLED")throw std::runtime_error(id+":"+commands[id]+":"+command_reason[id]);return commands[id]=="COMPLETED";},timeout,"command completion timeout "+id);
  }
  void read(const std::string& id) {
    notify("MAVMOOS_PARAM_REQUEST","id="+id+",action=read,name=SIM_WIND_SPD");
    wait([&]{return terminal(id)&&(params[id]!="COMPLETED"||values.count(id));},10,"parameter read timeout");
    if(params[id]!="COMPLETED")throw std::runtime_error("read failed "+id+":"+params[id]+":"+param_reason[id]);
  }
  void readMissionAndParameter(double original) {
    if(home.empty())throw std::runtime_error("bound FC HOME observation missing before read");
    list_nav_start=nav();monitor=true;last_nav_update=mono();const auto began=mono();
    std::ofstream(control/"transfer.started")<<std::setprecision(16)<<began<<'\n';
    notify("MAVMOOS_MISSION_REQUEST","id=concurrent-mission-read,action=download,timeout=15");
    notify("MAVMOOS_PARAM_REQUEST","id=concurrent-parameter-read,action=read,name=SIM_WIND_SPD");
    wait([&]{
      const auto state=missions["concurrent-mission-read"];
      if(state=="FAILED"||state=="CANCELLED"||state=="REJECTED"||state=="TIMED_OUT"||state=="INCOMPLETE")throw std::runtime_error("mission read failed:"+state+":"+mission_reason["concurrent-mission-read"]);
      if(terminal("concurrent-parameter-read")&&params["concurrent-parameter-read"]!="COMPLETED")throw std::runtime_error("concurrent parameter read failed:"+param_reason["concurrent-parameter-read"]);
      return state=="COMPLETED"&&mission_snapshot&&mission_fixture.complete(mission_tx)&&
        params["concurrent-parameter-read"]=="COMPLETED"&&values.count("concurrent-parameter-read")&&param_tx.count("concurrent-parameter-read/initial");
    },18,"mission and ordinary parameter reads did not complete");
    const auto completed=mono();std::ofstream(control/"reads.completed")<<std::setprecision(16)<<completed<<'\n';
    if(values["concurrent-parameter-read"]!=original||!mission_fixture.complete(mission_tx)||!param_tx.count("concurrent-parameter-read/initial"))throw std::runtime_error("read-only transfer values or declared send evidence missing");
    log("assertion","read_services_complete","mission_status=COMPLETED,parameter_status=COMPLETED,service_window_s="+std::to_string(completed-began)+",nav_at_completion="+std::to_string(nav())+",mission_mutations="+runtime["mission_mutations"]);
    // An empty transfer can finish inside one FC observation interval. Keep
    // this separate continuation window explicit, instead of claiming a long
    // protocol transfer from observer samples collected after its completion.
    observe(2);
    if(loss||!active()||nav()<=list_nav_start||max_nav_update_gap>.75)throw std::runtime_error("navigation failed before/during/after mission read");
    std::ofstream(control/"transfer.finished")<<std::setprecision(16)<<mono()<<'\n';
    log("assertion","mission_read_continuity","post_read_observation_s="+std::to_string(mono()-completed)+",mission_request_list_sends="+std::to_string(mission_tx[43])+",mission_request_item_sends="+std::to_string(mission_tx[51])+",mission_ack_sends="+std::to_string(mission_tx[47])+",parameter_read_sends="+std::to_string(param_tx["concurrent-parameter-read/initial"])+",no_mutation=true");
    log("assertion","transfer_outcome","status=COMPLETED,nav_before="+std::to_string(list_nav_start)+",nav_after="+std::to_string(nav())+",loss_seen=false,max_observed_nav_counter_update_gap_s="+std::to_string(max_nav_update_gap));
    monitor=false;
  }
};
}
int main(int argc,char** argv) {
  Probe p;
  try {
    if(argc!=4&&argc!=5)throw std::runtime_error("usage: concurrent_moos_probe DB_PORT SCENARIO CONTROL_DIR [DECLARED_MISSION_SHAPE]");
    p.scenario=argv[2];if(p.scenario!="bulk"&&p.scenario!="mission_default"&&p.scenario!="bridge_pause"&&p.scenario!="mission_read")throw std::runtime_error("unsupported scenario");
    if(p.scenario=="mission_read") {
      if(argc!=5)throw std::runtime_error("mission_read requires a source-declared fixture shape");
      p.mission_fixture=mavmoos::test::MissionFixtureShape(argv[4]);
      p.log("fixture","mission_shape",p.mission_fixture.name());
    } else if(argc!=4)throw std::runtime_error("unexpected mission shape for non-mission scenario");
    p.control=argv[3];std::filesystem::create_directories(p.control);p.client.SetQuiet(true);
    if(!p.client.Run("127.0.0.1",std::stoi(argv[1]),"mavmoos_concurrent_probe",50))throw std::runtime_error("MOOS connection failed");
    p.wait([&]{return p.ready();},55,"initial vehicle readiness");
    if(yes(p.runtime["armed"])||yes(p.runtime["in_air"]))throw std::runtime_error("simulator did not start disarmed/on-ground");
    p.command("concurrent-guided","guided");p.command("concurrent-arm","arm");p.command("concurrent-takeoff","takeoff,height=5",48);
    p.notify("MOOS_MANUAL_OVERRIDE","false");p.notify("DEPLOY","true");p.notify("MAVMOOS_NAV_CONTROL","acquire");
    p.wait([&]{return p.active();},8,"initial NAV acquisition");p.observe(2);
    if(!p.active()||p.nav()<8)throw std::runtime_error("initial navigation not established");
    p.read("wind-before");const auto original=p.values["wind-before"];
    if(p.scenario!="mission_read") {
    p.notify("MAVMOOS_PARAM_REQUEST","id=inflight-write,action=write,name=SIM_WIND_SPD,value=0.25,type=REAL32");
    p.wait([&]{return p.terminal("inflight-write");},3,"in-flight SET rejection");
    if(p.params["inflight-write"]!="REJECTED"||p.param_reason["inflight-write"]!="simulation_wind_write_requires_disarmed_ground"||p.param_tx.count("inflight-write/initial"))throw std::runtime_error("in-flight write was not rejected before transport");
    p.log("assertion","inflight_write_rejected","ground/arming policy rejected request; no parameter send receipt");
    }
    p.wait([&]{auto h=fields(p.heartbeat);return p.active()&&h.count("rx_mono_s")&&mono()-std::stod(h["rx_mono_s"])<.25;},4,"fresh heartbeat before concurrent transfer");
    if(p.scenario=="mission_read")p.readMissionAndParameter(original);
    else {
    p.list_nav_start=p.nav();p.monitor=true;p.last_nav_update=mono();
    std::ofstream(p.control/"transfer.started")<<std::setprecision(16)<<mono()<<'\n';
    p.notify("MAVMOOS_PARAM_REQUEST","id=concurrent-list,action=list");
    p.wait([&]{return p.terminal("concurrent-list");},65,"bounded list terminal result");
    std::ofstream(p.control/"transfer.finished")<<std::setprecision(16)<<mono()<<'\n';
    const auto terminal=p.params["concurrent-list"];
    if(!p.paced_progress_seen)throw std::runtime_error("paced indexed progress was not observed");
    if(p.scenario=="bulk"&&(terminal!="COMPLETED"||!p.snapshot||p.nav()<=p.list_nav_start))throw std::runtime_error("bulk list failed completeness or during-transfer NAV progress");
    if(terminal=="CANCELLED"&&(p.scenario=="mission_default"||p.scenario=="bridge_pause")&&p.param_reason["concurrent-list"]=="connection_reset") {
      p.wait([&]{return p.vehicle_session_reset_seen&&!yes(p.runtime["permission"])&&p.helm.find("state=LATCHED")!=std::string::npos;},3,"cancelled transfer lacked audited vehicle-session loss");
      p.log("assertion",p.scenario=="bridge_pause"?"interrupted_session_loss":"characterized_default_session_loss","connection_reset cancelled the old list; vehicle_connection_lost latched NAV; no complete table is claimed; supervisor must audit primary heartbeat expiry for bridge_pause");
    } else if(terminal!="COMPLETED"&&terminal!="INCOMPLETE")throw std::runtime_error("unexpected list terminal result: "+terminal+":"+p.param_reason["concurrent-list"]);
    if(terminal=="COMPLETED"&&!p.snapshot)p.wait([&]{return p.snapshot;},2,"complete list snapshot missing");
    if(p.scenario=="bridge_pause"&&(!p.injection_observed||!std::filesystem::exists(p.control/"restored")))throw std::runtime_error("pending list interruption was not demonstrated");
    p.log("assertion","transfer_outcome","status="+terminal+",nav_before="+std::to_string(p.list_nav_start)+",nav_after="+std::to_string(p.nav())+",loss_seen="+(p.loss?"true":"false")+",max_observed_nav_counter_update_gap_s="+std::to_string(p.max_nav_update_gap));
    p.monitor=false;
    }
    p.wait([&]{return p.ready();},10,"fresh state after transfer");
    if(p.loss||p.scenario=="bridge_pause") {
      p.observe(1);if(yes(p.runtime["permission"])||p.helm.find("state=LATCHED")==std::string::npos)throw std::runtime_error("fresh data automatically restored lost NAV permission");
      const auto stopped=p.nav();p.observe(.5);if(p.nav()!=stopped)throw std::runtime_error("NAV continued after latched transfer loss");
      p.notify("MAVMOOS_NAV_CONTROL","acquire");p.wait([&]{return p.active();},8,"explicit NAV recovery");p.observe(.5);
      if(p.nav()<=stopped)throw std::runtime_error("explicit acquire did not restore NAV progress");
      p.log("assertion","explicit_recovery","fresh returning observations did not resume NAV; new acquire restored it");
    } else {const auto prior=p.nav();p.observe(1);if(!p.active()||p.nav()<=prior)throw std::runtime_error("NAV did not continue after transfer");}
    p.read("wind-after");if(p.values["wind-after"]!=original)throw std::runtime_error("simulator wind changed despite rejected write");
    if(p.param_tx.count("inflight-write/initial"))throw std::runtime_error("rejected write later reached transport");
    p.log("assertion","wind_unchanged",std::to_string(original));
    p.notify("DEPLOY","false");p.command("concurrent-land","land",65);
    if(yes(p.runtime["armed"]))p.command("concurrent-disarm","disarm");
    p.wait([&]{return p.statusFresh()&&!yes(p.runtime["armed"])&&!yes(p.runtime["in_air"])&&!yes(p.runtime["permission"]);},3,"final disarmed/on-ground");
    p.log("pass",p.scenario,"concurrent parameter/navigation workflow and independent explicit recovery complete");p.client.Close();return 0;
  } catch(const std::exception& e) {p.log("fail",p.scenario,e.what());p.client.Close();return 1;}
}
