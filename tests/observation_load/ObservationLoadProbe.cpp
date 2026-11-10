/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ObservationLoadProbe.cpp                        */
/*    DATE: November 4, 2026                                */
/************************************************************/
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include "FinishBoundary.h"
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
double mono(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
bool yes(const std::string& s){return s=="1"||s=="true";}
std::map<std::string,std::string> fields(const std::string& text)
{
  std::map<std::string,std::string> result;std::istringstream in(text);std::string item;
  while(std::getline(in,item,',')){const auto at=item.find('=');if(at!=std::string::npos)result[item.substr(0,at)]=item.substr(at+1);}
  return result;
}
std::string quote(const std::string& text)
{
  static constexpr char hex[]="0123456789abcdef";std::string result="\"";
  for(unsigned char c:text){if(c=='"'||c=='\\'){result+='\\';result+=char(c);}else if(c<32){result+="\\u00";result+=hex[c>>4];result+=hex[c&15];}else result+=char(c);}return result+'"';
}
struct Probe {
  CMOOSCommClient client;
  std::filesystem::path control;
  std::map<std::string,std::string> runtime,command_status,command_reason,parameter_status,mission_status;
  std::map<std::string,double> values;
  std::string helm;
  mavmoos::test::MissionFixtureShape mission_fixture{"bound_home_only"};
  std::map<unsigned,unsigned> mission_tx;
  bool registered=false,monitor=false,snapshot=false,mission_snapshot=false;
  double start=mono(),runtime_at=-1,last_runtime=-1,last_nav_update=-1,max_nav_gap=0;
  unsigned nav_previous=0,nav_begin=0,records=0;
  void log(const std::string& type,const std::string& key,const std::string& value,const std::string& source="probe") {
    if(++records>50000)throw std::runtime_error("probe record bound exceeded");
    std::cout<<std::setprecision(16)<<"{\"elapsed\":"<<mono()-start<<",\"type\":"<<quote(type)<<",\"key\":"<<quote(key)
      <<",\"source\":"<<quote(source)<<",\"value\":"<<quote(value)<<"}\n";std::cout.flush();
  }
  void notify(const std::string& key,const std::string& value){client.Notify(key,value);log("request",key,value);}
  std::string marker(const std::string& key) {
    std::ostringstream content;content<<std::setprecision(16)<<mono()<<'\n';
    {std::ofstream output(control/(key+".tmp"));output<<content.str();output.close();if(!output)throw std::runtime_error("phase marker write failed");}
    std::filesystem::rename(control/(key+".tmp"),control/key);log("phase",key,"nav_tx_accepted="+std::to_string(nav()));
    return content.str();
  }
  std::string finishMarker() {
    if(std::filesystem::exists(control/"load.finished.ack"))throw std::runtime_error("preexisting finish acknowledgement");
    return marker("load.finished");
  }
  bool statusFresh(){return client.IsConnected()&&runtime_at>0&&mono()-runtime_at<.75;}
  bool ready(){return statusFresh()&&yes(runtime["heartbeat_valid"])&&yes(runtime["position_valid"])&&yes(runtime["home_valid"])&&yes(runtime["landed_valid"])&&yes(runtime["estimator_valid"]);}
  bool active(){return ready()&&yes(runtime["permission"])&&runtime["fc_mode"]=="4"&&helm.find("state=ACTIVE")!=std::string::npos;}
  unsigned nav(){return runtime.count("nav_tx_accepted")?std::stoul(runtime["nav_tx_accepted"]):0;}
  static bool failed(const std::string& value){return value=="FAILED"||value=="CANCELLED"||value=="TIMED_OUT"||value=="INCOMPLETE"||value=="REJECTED";}
  void pump() {
    if(!client.IsConnected()){registered=false;runtime_at=-1;if(monitor)throw std::runtime_error("MOOS disconnected during observation load");return;}
    if(!registered){for(const char* key:{"MAVMOOS_CAPABILITIES","MAVMOOS_RUNTIME_STATUS","MAVMOOS_HELM_STATUS","MAVMOOS_HOME_POSITION",
      "MAVMOOS_COMMAND_RESULT","MAVMOOS_PARAM_RESULT","MAVMOOS_PARAM_VALUE","MAVMOOS_PARAM_SNAPSHOT","MAVMOOS_PARAM_TX_RESULT",
      "MAVMOOS_MISSION_RESULT","MAVMOOS_MISSION_SNAPSHOT","MAVMOOS_MISSION_TX_RESULT","MAVMOOS_SYS_STATUS","MAVMOOS_BATTERY_STATUS",
      "MAVMOOS_GPS_STATUS","MAVMOOS_SYSTEM_STATUS","MAVMOOS_FC_TEXT","MAVMOOS_ATTITUDE_EULER","MAVMOOS_ATTITUDE_QUATERNION","MAVMOOS_ATTITUDE_STATUS"})client.Register(key,0);registered=true;}
    MOOSMSG_LIST mail;client.Fetch(mail);
    for(const auto& message:mail) {
      if(message.GetSource()!="iMAVLink"||!message.IsString())continue;
      const auto key=message.GetKey(),text=message.GetString();
      if(key=="MAVMOOS_RUNTIME_STATUS") {
        runtime=fields(text);runtime_at=mono();const auto count=nav();
        if(count<nav_previous)throw std::runtime_error("navigation send counter regressed");
        if(count>nav_previous){if(monitor&&last_nav_update>0)max_nav_gap=std::max(max_nav_gap,mono()-last_nav_update);last_nav_update=mono();nav_previous=count;log("nav_counter",key,text,"iMAVLink");}
        if(mono()-last_runtime>.2){log("runtime",key,text,"iMAVLink");last_runtime=mono();}
      } else if(key=="MAVMOOS_HELM_STATUS"){helm=text;log("helm",key,text,"iMAVLink");}
      else if(key=="MAVMOOS_COMMAND_RESULT"){const auto f=fields(text);command_status[f.at("id")]=f.at("status");command_reason[f.at("id")]=f.at("reason");log("command_result",key,text,"iMAVLink");}
      else if(key=="MAVMOOS_PARAM_RESULT"){const auto f=fields(text);parameter_status[f.at("id")]=f.at("status");log("parameter_result",key,text,"iMAVLink");}
      else if(key=="MAVMOOS_PARAM_VALUE"){const auto f=fields(text);values[f.at("id")]=std::stod(f.at("value"));log("parameter_value",key,text,"iMAVLink");}
      else if(key=="MAVMOOS_PARAM_SNAPSHOT"){if(text.find("\"id\":\"observation-list\"")!=std::string::npos&&text.find("\"status\":\"complete\"")!=std::string::npos)snapshot=true;log("parameter_snapshot",key,text,"iMAVLink");}
      else if(key=="MAVMOOS_MISSION_RESULT"){const auto f=fields(text);mission_status[f.at("id")]=f.at("status");log("mission_result",key,text,"iMAVLink");}
      else if(key=="MAVMOOS_MISSION_SNAPSHOT"){if(text.find("\"id\":\"observation-mission\"")!=std::string::npos&&text.find("\"status\":\"complete\"")!=std::string::npos)mission_snapshot=true;log("mission_snapshot",key,text,"iMAVLink");}
      else if(key=="MAVMOOS_PARAM_TX_RESULT")log("parameter_tx",key,text,"iMAVLink");
      else if(key=="MAVMOOS_MISSION_TX_RESULT") {
        const auto f=fields(text);
        if(f.at("operation_id")!="observation-mission"||f.at("status")!="LOCAL_ACCEPTED"||f.at("late")!="false"||f.at("clock_valid")!="true")throw std::runtime_error("unexpected observation mission send result");
        const auto id=std::stoul(f.at("msgid"));
        if(!mission_fixture.allows(id))throw std::runtime_error("observation mission emitted undeclared item/mutation request");
        ++mission_tx[id];log("mission_tx",key,text,"iMAVLink");
      }
      else if(key=="MAVMOOS_HOME_POSITION")log("home",key,text,"iMAVLink");
      else if(key=="MAVMOOS_CAPABILITIES")log("catalog",key,text,"iMAVLink");
      else log("observation",key,text,"iMAVLink");
    }
    if(monitor&&(!active()||runtime["mission_mutations"]!="0"))throw std::runtime_error("observation load lost active stock helm or acquired mutation lease");
  }
  void wait(const std::function<bool()>& predicate,double seconds,const std::string& why) {
    const double end=mono()+seconds;while(mono()<end){pump();if(predicate())return;std::this_thread::sleep_for(std::chrono::milliseconds(10));}throw std::runtime_error(why);
  }
  void observe(double seconds){const double end=mono()+seconds;while(mono()<end){pump();std::this_thread::sleep_for(std::chrono::milliseconds(10));}}
  void finishAck(const std::string& token,double started) {
    const auto observed=mavmoos::test::waitForFinishAck(token,started+3,[&]() -> std::optional<std::string> {
      const auto path=control/"load.finished.ack";
      if(!std::filesystem::exists(path))return std::nullopt;
      std::ifstream input(path,std::ios::binary);char data[129];input.read(data,sizeof(data));const auto size=input.gcount();
      if(input.bad()||!input.is_open()||size>128)throw std::runtime_error("finish acknowledgement unreadable/oversized");
      return std::string(data,static_cast<std::size_t>(size));
    },[&]{pump();},[]{return mono();},[](double seconds){std::this_thread::sleep_for(std::chrono::duration<double>(seconds));});
    if(!observed)throw std::runtime_error("finish acknowledgement missed fixed3s deadline");
    max_nav_gap=std::max(max_nav_gap,*observed-last_nav_update);
    if(!monitor||!active()||max_nav_gap>.75)throw std::runtime_error("finish acknowledgement lost navigation continuity");
    static constexpr char digits[]="0123456789abcdef";std::string hex;
    for(const unsigned char c:token){hex+=digits[c>>4];hex+=digits[c&15];}
    std::ostringstream value;value<<std::setprecision(16)<<"token_hex="<<hex<<",wait_s="<<*observed-started
      <<",timeout_s=3,nav_tx_accepted="<<nav()<<",max_nav_counter_update_gap_s="<<max_nav_gap;
    log("assertion","finish_boundary_ack",value.str());
  }
  void command(const std::string& id,const std::string& action,double seconds=10) {
    notify("MAVMOOS_COMMAND","id="+id+",action="+action);
    wait([&]{if(failed(command_status[id]))throw std::runtime_error(id+":"+command_status[id]+":"+command_reason[id]);return command_status[id]=="COMPLETED";},seconds,"command timeout "+id);
  }
  void read(const std::string& id) {
    notify("MAVMOOS_PARAM_REQUEST","id="+id+",action=read,name=SIM_WIND_SPD");
    wait([&]{if(failed(parameter_status[id]))throw std::runtime_error("parameter read failed "+id);return parameter_status[id]=="COMPLETED"&&values.count(id);},10,"parameter read timeout "+id);
  }
};
}
int main(int argc,char** argv) {
  Probe p;
  try {
    if(argc!=4)throw std::runtime_error("usage: observation_load_probe MOOS_PORT CONTROL_DIR DECLARED_MISSION_SHAPE");
    p.mission_fixture=mavmoos::test::MissionFixtureShape(argv[3]);
    p.log("assertion","mission_fixture_shape",p.mission_fixture.name());
    p.control=argv[2];std::filesystem::create_directories(p.control);p.client.SetQuiet(true);
    if(!p.client.Run("127.0.0.1",std::stoi(argv[1]),"mavmoos_observation_load_probe",50))throw std::runtime_error("MOOS startup failed");
    p.wait([&]{return p.ready();},55,"initial FC readiness timeout");
    if(yes(p.runtime["armed"])||yes(p.runtime["in_air"]))throw std::runtime_error("native fixture did not begin disarmed/on-ground");
    p.command("observation-guided","guided");p.command("observation-arm","arm");p.command("observation-takeoff","takeoff,height=5",48);
    p.notify("MOOS_MANUAL_OVERRIDE","false");p.notify("DEPLOY","true");p.notify("MAVMOOS_NAV_CONTROL","acquire");
    p.wait([&]{return p.active();},8,"stock helm acquisition timeout");p.observe(2);
    p.read("observation-before");const double wind=p.values["observation-before"];
    if(!p.active()||p.nav()<8)throw std::runtime_error("stock helm movement not established");
    p.nav_begin=p.nav();p.monitor=true;p.last_nav_update=mono();p.marker("load.started");
    p.notify("MAVMOOS_PARAM_REQUEST","id=observation-list,action=list");
    p.notify("MAVMOOS_MISSION_REQUEST","id=observation-mission,action=download,timeout=15");
    p.wait([&]{
      if(Probe::failed(p.parameter_status["observation-list"])||Probe::failed(p.mission_status["observation-mission"]))throw std::runtime_error("concurrent read service failed");
      return p.parameter_status["observation-list"]=="COMPLETED"&&p.snapshot&&p.mission_status["observation-mission"]=="COMPLETED"&&p.mission_snapshot&&p.mission_fixture.complete(p.mission_tx);
    },65,"bounded concurrent read completion timeout");
    p.marker("reads.completed");
    p.observe(5);p.max_nav_gap=std::max(p.max_nav_gap,mono()-p.last_nav_update);
    const double finish_started=mono();
    const auto finish_token=p.finishMarker();
    if(!p.active()||p.nav()<=p.nav_begin||p.max_nav_gap>.75)throw std::runtime_error("stock helm navigation continuity failed");
    p.log("assertion","load_continuity","nav_before="+std::to_string(p.nav_begin)+",nav_after="+std::to_string(p.nav())+",max_nav_counter_update_gap_s="+std::to_string(p.max_nav_gap)+",mission_mutations="+p.runtime["mission_mutations"]);
    p.finishAck(finish_token,finish_started);
    p.monitor=false;p.read("observation-after");
    if(!std::isfinite(wind)||p.values["observation-after"]!=wind)throw std::runtime_error("simulator wind changed during read-only work");
    p.log("assertion","wind_unchanged",std::to_string(wind));
    p.notify("DEPLOY","false");p.command("observation-land","land",65);
    if(yes(p.runtime["armed"]))p.command("observation-disarm","disarm");
    p.wait([&]{return p.statusFresh()&&!yes(p.runtime["armed"])&&!yes(p.runtime["in_air"])&&!yes(p.runtime["permission"]);},3,"final ground/disarmed timeout");
    p.log("pass","observation_load","stock helm and read services completed with observational modules");p.client.Close();return 0;
  } catch(const std::exception& error){p.log("fail","observation_load",error.what());p.client.Close();return 1;}
}
