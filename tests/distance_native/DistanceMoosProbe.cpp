/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: DistanceMoosProbe.cpp                            */
/*    DATE: November 4, 2026                                */
/************************************************************/
// Ordinary client for the existing stock-helm mission and command interface.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
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
double mono(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
bool yes(const std::string& s){return s=="1"||s=="true";}
std::map<std::string,std::string> fields(const std::string& value)
{std::map<std::string,std::string> result;std::istringstream in(value);std::string p;while(std::getline(in,p,',')){auto at=p.find('=');if(at!=std::string::npos)result[p.substr(0,at)]=p.substr(at+1);}return result;}
std::string quote(const std::string& text)
{constexpr char hex[]="0123456789abcdef";std::string out="\"";for(unsigned char c:text){if(c=='"'||c=='\\'){out+='\\';out+=char(c);}else if(c<32){out+="\\u00";out+=hex[c>>4];out+=hex[c&15];}else out+=char(c);}return out+'"';}
struct Probe {
  CMOOSCommClient client;std::filesystem::path control;
  std::map<std::string,std::string> runtime,commands,reasons;
  double start=mono(),runtime_at=-1,last_runtime=-1,height=0,height_at=-1;
  std::string helm;bool registered=false,monitor=false;unsigned records=0;
  void log(const std::string& type,const std::string& key,const std::string& value,const std::string& source="probe")
  {if(++records>30000)throw std::runtime_error("probe record bound exceeded");std::cout<<std::setprecision(17)<<"{\"elapsed\":"<<mono()-start<<",\"type\":"<<quote(type)<<",\"key\":"<<quote(key)<<",\"source\":"<<quote(source)<<",\"value\":"<<quote(value)<<"}\n";std::cout.flush();}
  void notify(const std::string& key,const std::string& value){client.Notify(key,value);log("request",key,value);}
  void marker(const std::string& key)
  {std::ofstream(control/(key+".tmp"))<<std::setprecision(17)<<mono()<<'\n';std::filesystem::rename(control/(key+".tmp"),control/key);log("phase",key,"nav_tx_accepted="+runtime["nav_tx_accepted"]);}
  bool ready(){return client.IsConnected()&&runtime_at>0&&mono()-runtime_at<.75&&yes(runtime.at("heartbeat_valid"))&&yes(runtime.at("position_valid"))&&yes(runtime.at("home_valid"))&&yes(runtime.at("landed_valid"))&&yes(runtime.at("estimator_valid"));}
  bool active(){return ready()&&yes(runtime.at("permission"))&&runtime.at("fc_mode")=="4"&&helm.find("state=ACTIVE")!=std::string::npos;}
  void pump()
  {
    if(!client.IsConnected()){registered=false;runtime_at=-1;if(monitor)throw std::runtime_error("MOOS disconnect during range flight");return;}
    if(!registered){for(const char* key:{"MAVMOOS_CAPABILITIES","MAVMOOS_RUNTIME_STATUS","MAVMOOS_HELM_STATUS","MAVMOOS_COMMAND_RESULT","MAVMOOS_DISTANCE_SENSORS","MAVMOOS_DISTANCE_STATUS","NAV_ALTITUDE"})client.Register(key,0);registered=true;}
    MOOSMSG_LIST mail;client.Fetch(mail);
    for(const auto& message:mail) {
      if(message.GetSource()!="iMAVLink")continue;const auto key=message.GetKey();
      if(key=="NAV_ALTITUDE"&&message.IsDouble()){height=message.GetDouble();height_at=mono();log("height",key,std::to_string(height),"iMAVLink");continue;}
      if(!message.IsString())continue;const auto text=message.GetString();
      if(key=="MAVMOOS_RUNTIME_STATUS"){runtime=fields(text);runtime_at=mono();if(mono()-last_runtime>=.1){log("runtime",key,text,"iMAVLink");last_runtime=mono();}}
      else if(key=="MAVMOOS_HELM_STATUS"){helm=text;log("helm",key,text,"iMAVLink");}
      else if(key=="MAVMOOS_COMMAND_RESULT"){const auto value=fields(text);commands[value.at("id")]=value.at("status");reasons[value.at("id")]=value.at("reason");log("command_result",key,text,"iMAVLink");}
      else log(key=="MAVMOOS_CAPABILITIES"?"catalog":"observation",key,text,"iMAVLink");
    }
    if(monitor&&!active())throw std::runtime_error("range observer flight lost stock-helm authority/freshness");
  }
  void wait(const std::function<bool()>& predicate,double limit,const std::string& why)
  {const double until=mono()+limit;while(mono()<until){pump();if(predicate())return;std::this_thread::sleep_for(std::chrono::milliseconds(10));}throw std::runtime_error(why);}
  void observe(double duration)
  {const double until=mono()+duration;while(mono()<until){pump();std::this_thread::sleep_for(std::chrono::milliseconds(10));}}
  void command(const std::string& id,const std::string& action,double limit=10)
  {notify("MAVMOOS_COMMAND","id="+id+",action="+action);wait([&]{const auto state=commands[id];if(state=="FAILED"||state=="CANCELLED"||state=="TIMED_OUT"||state=="REJECTED")throw std::runtime_error(id+":"+state+":"+reasons[id]);return state=="COMPLETED";},limit,"command completion timeout "+id);}
  void settle(double target)
  {double since=-1;wait([&]{if(active()&&height_at>0&&mono()-height_at<.5&&std::isfinite(height)&&std::abs(height-target)<.25){if(since<0)since=mono();return mono()-since>=2;}since=-1;return false;},24,"height did not settle at "+std::to_string(target));}
  void window(const std::string& name){marker(name+".started");observe(4);marker(name+".finished");}
};
}
int main(int argc,char** argv)
{
  Probe p;try {
    if(argc!=3)throw std::runtime_error("usage: distance_moos_probe MOOS_PORT CONTROL_DIRECTORY");p.control=argv[2];std::filesystem::create_directories(p.control);p.client.SetQuiet(true);
    if(!p.client.Run("127.0.0.1",std::stoi(argv[1]),"mavmoos_distance_probe",50))throw std::runtime_error("MOOS startup failed");
    p.wait([&]{return p.ready();},55,"initial FC readiness timeout");
    if(yes(p.runtime["armed"])||yes(p.runtime["in_air"]))throw std::runtime_error("range fixture did not start disarmed/on-ground");
    p.marker("ground.started");p.observe(3);p.marker("ground.finished");
    p.command("range-guided","guided");p.command("range-arm","arm");p.command("range-takeoff","takeoff,height=5",48);
    p.notify("MOOS_MANUAL_OVERRIDE","false");p.notify("DEPLOY","true");p.notify("MAVMOOS_NAV_CONTROL","acquire");
    p.wait([&]{return p.active();},8,"stock helm acquisition timeout");p.monitor=true;p.settle(5);p.window("height.low");
    p.notify("ALTITUDE_UPDATE","altitude=8");p.settle(8);p.window("height.high");
    p.notify("ALTITUDE_UPDATE","altitude=5");p.settle(5);p.window("height.return");
    p.monitor=false;p.notify("DEPLOY","false");p.command("range-land","land",65);
    if(yes(p.runtime["armed"]))p.command("range-disarm","disarm");
    p.wait([&]{return p.runtime_at>0&&mono()-p.runtime_at<.75&&!yes(p.runtime["armed"])&&!yes(p.runtime["in_air"])&&!yes(p.runtime["permission"]);},3,"final ground/disarmed timeout");
    p.log("pass","distance_flight","stock helm5/8/5m via ordinary ALTITUDE_UPDATE; range observations never own NAV/permission");p.client.Close();return 0;
  }catch(const std::exception& error){p.log("fail","distance_flight",error.what());p.client.Close();return 1;}
}
