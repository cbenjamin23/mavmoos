/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: CopterScenarioProbe.cpp                         */
/*    DATE: November 4, 2026                                */
/************************************************************/
// Ordinary MOOS client; all effects use the application's public operation API.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include "../observation_load/FinishBoundary.h"
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
bool yes(const std::string& text){return text=="true"||text=="1";}
std::string quote(const std::string& text)
{constexpr char hex[]="0123456789abcdef";std::string out="\"";for(unsigned char c:text){if(c=='"'||c=='\\'){out+='\\';out+=char(c);}else if(c<32){out+="\\u00";out+=hex[c>>4];out+=hex[c&15];}else out+=char(c);}return out+'"';}
std::map<std::string,std::string> fields(const std::string& text)
{std::map<std::string,std::string> result;std::istringstream in(text);std::string item;while(std::getline(in,item,',')){const auto at=item.find('=');if(at==std::string::npos||!result.emplace(item.substr(0,at),item.substr(at+1)).second)throw std::runtime_error("malformed/duplicate KV result");}return result;}

struct Probe {
  CMOOSCommClient client;
  std::filesystem::path control;
  double start=mono(),runtime_at=-1,last_runtime=-1,nav_at=-1,home_lat=0,home_lon=0;
  bool registered=false,have_home=false;
  std::map<std::string,std::string> runtime,terminal,reasons;
  std::map<std::string,std::set<std::string>> statuses;
  std::map<std::string,double> numbers,expires;
  unsigned records=0,boundary_counter=0;

  void log(const std::string& type,const std::string& key,const std::string& value,const std::string& source="probe")
  {if(++records>160000)throw std::runtime_error("probe record bound exceeded");std::cout<<std::setprecision(17)<<"{\"elapsed\":"<<mono()-start<<",\"type\":"<<quote(type)<<",\"key\":"<<quote(key)<<",\"source\":"<<quote(source)<<",\"value\":"<<quote(value)<<"}\n";std::cout.flush();}
  void marker(const std::string& key,const std::string& extra="")
  {std::ofstream(control/(key+".tmp"))<<std::setprecision(17)<<mono()<<'\n';std::filesystem::rename(control/(key+".tmp"),control/key);log("phase",key,"nav_tx_accepted="+runtime["nav_tx_accepted"]+extra);}
  std::string hold_marker(const std::string& key,const std::string& extra="")
  {
    if(std::filesystem::exists(control/(key+".ack")))throw std::runtime_error("preexisting hold acknowledgement");
    std::ostringstream out;out<<"hold:"<<key<<':'<<++boundary_counter<<':'<<std::setprecision(17)<<mono()<<'\n';
    const auto token=out.str();
    {std::ofstream file(control/(key+".tmp"));file<<token;if(!file)throw std::runtime_error("hold marker write failed");}
    std::filesystem::rename(control/(key+".tmp"),control/key);
    log("phase",key,"nav_tx_accepted="+runtime["nav_tx_accepted"]+extra);return token;
  }
  void hold_ack(const std::string& key,const std::string& token,double started,double deadline,const std::function<void()>& condition)
  {
    const auto acknowledged=mavmoos::test::waitForFinishAck(token,deadline,[&]()->std::optional<std::string>{
      std::ifstream input(control/(key+".ack"));if(!input)return std::nullopt;
      std::string text;char c;while(text.size()<=128&&input.get(c))text+=c;
      if(input.bad()||text.size()>128)throw std::runtime_error("hold acknowledgement unreadable/oversized");return text;
    },[&]{pump();condition();},mono,[](double seconds){std::this_thread::sleep_for(std::chrono::duration<double>(seconds));});
    if(!acknowledged)throw std::runtime_error("hold acknowledgement missed fixed3s deadline");
    if(++records>160000)throw std::runtime_error("probe record bound exceeded");
    std::cout<<std::setprecision(17)<<"{\"elapsed\":"<<mono()-start<<",\"type\":\"boundary_ack\",\"key\":"<<quote(key)
      <<",\"source\":\"probe\",\"value\":"<<quote(token)<<",\"wait_started_elapsed\":"<<started-start
      <<",\"deadline_elapsed\":"<<deadline-start<<",\"ack_observed_elapsed\":"<<*acknowledged-start<<"}\n";std::cout.flush();
  }
  void notify(const std::string& key,const std::string& value)
  {if(!client.Notify(key,value))throw std::runtime_error("MOOS Notify failed");log("request",key,value);}
  bool ready()
  {return client.IsConnected()&&runtime_at>0&&mono()-runtime_at<.75&&yes(runtime["heartbeat_valid"])&&yes(runtime["position_valid"])&&yes(runtime["home_valid"])&&yes(runtime["landed_valid"])&&yes(runtime["estimator_valid"])&&nav_at>0&&mono()-nav_at<.5;}
  bool flying()
  {return ready()&&yes(runtime["armed"])&&yes(runtime["in_air"])&&runtime["fc_mode"]=="4";}
  void pump()
  {
    if(!client.IsConnected()){if(registered)throw std::runtime_error("MOOS disconnected during scenario");return;}
    if(!registered){for(const char* key:{"MAVMOOS_CAPABILITIES","MAVMOOS_RUNTIME_STATUS","MAVMOOS_CONTROL_RESULT","MAVMOOS_CONTROL_STATUS","MAVMOOS_HELM_STATUS","MAVMOOS_FIRMWARE_STATUS","MAVMOOS_HOME_POSITION","NAV_X","NAV_Y","NAV_ALTITUDE","NAV_SAMPLE_RECEIVED_AT","DESIRED_HEADING","DESIRED_SPEED","DESIRED_ALTITUDE"})client.Register(key,0);registered=true;}
    MOOSMSG_LIST mail;client.Fetch(mail);
    for(const auto& message:mail){const auto key=message.GetKey(),source=message.GetSource();
      if(source=="pHelmIvP"&&message.IsDouble()&&key.rfind("DESIRED_",0)==0){log("helm_scalar",key,std::to_string(message.GetDouble()),source);continue;}
      if(source!="iMAVLink")continue;
      if(message.IsDouble()&&key.rfind("NAV_",0)==0){numbers[key]=message.GetDouble();nav_at=mono();log("navigation",key,std::to_string(message.GetDouble()),source);continue;}
      if(!message.IsString())continue;const auto text=message.GetString();
      if(key=="MAVMOOS_RUNTIME_STATUS"){runtime=fields(text);runtime_at=mono();if(mono()-last_runtime>=.1){log("runtime",key,text,source);last_runtime=mono();}}
      else if(key=="MAVMOOS_CONTROL_RESULT"){const auto result=fields(text);const auto& id=result.at("id");const auto expiry=std::stod(result.at("expires"));if(!std::isfinite(expiry)||expiry<=0)throw std::runtime_error("invalid operation expiry");if(expires.count(id)&&expires[id]!=expiry)throw std::runtime_error("operation original deadline changed");expires[id]=expiry;statuses[id].insert(result.at("status"));terminal[id]=result.at("status");reasons[id]=result.at("reason");log("operation_result",key,text,source);}
      else if(key=="MAVMOOS_HOME_POSITION"){const auto data=fields(text);if(yes(data.at("encoding_valid"))){home_lat=std::stod(data.at("lat_deg"));home_lon=std::stod(data.at("lon_deg"));have_home=true;}log("home",key,text,source);}
      else log(key=="MAVMOOS_CAPABILITIES"?"catalog":key=="MAVMOOS_FIRMWARE_STATUS"?"firmware":"state",key,text,source);
    }
  }
  void wait(const std::function<bool()>& predicate,double limit,const std::string& reason)
  {const auto end=mono()+limit;while(mono()<end){pump();if(predicate())return;std::this_thread::sleep_for(std::chrono::milliseconds(10));}throw std::runtime_error(reason);}
  void check(const std::string& id)
  {const auto& state=terminal[id];if(state=="REJECTED"||state=="FAILED"||state=="TIMED_OUT"||state=="CANCELLED")throw std::runtime_error(id+":"+state+":"+reasons[id]);}
  void request(const std::string& id,const std::string& body)
  {notify("MAVMOOS_CONTROL_REQUEST","schema=1,id="+id+","+body);}
  void command(const std::string& id,const std::string& body,double wait_limit=12)
  {request(id,body);wait([&]{check(id);return statuses[id].count("COMPLETED")>0;},wait_limit,"operation completion timeout: "+id);}
  std::string point(double east,double north)
  {std::ostringstream out;out<<std::setprecision(14)<<",lat="<<home_lat+north/111195.<<",lon="<<home_lon+east/(111195.*std::cos(home_lat*3.14159265358979323846/180.))<<",height=5";return out.str();}
  void observe(double duration,const std::function<bool()>& condition,const std::string& why)
  {const auto end=mono()+duration;while(mono()<end){pump();if(!condition())throw std::runtime_error(why);std::this_thread::sleep_for(std::chrono::milliseconds(10));}}
  void hold_window(const std::string& cycle,const std::string& strategy)
  {
    const auto id=cycle+"-hold-"+strategy;
    request(id,"action=hold,strategy="+strategy+",timeout=20");
    wait([&]{check(id);return statuses[id].count("CONFIRMED")&&flying()&&yes(runtime["permission"]);},10,"hold was not confirmed: "+id);
    std::ostringstream anchor;anchor<<std::setprecision(14)<<",anchor_x="<<numbers.at("NAV_X")<<",anchor_y="<<numbers.at("NAV_Y")<<",height=5,strategy="<<strategy;
    const auto begin=cycle+".hold_"+strategy+".started",end=cycle+".hold_"+strategy+".finished";
    const auto maintained=[&]{check(id);if(!flying()||!yes(runtime["permission"]))throw std::runtime_error("hold acknowledgement lost current vehicle permission/state");};
    const auto started=mono();const auto start_token=hold_marker(begin,anchor.str());
    hold_ack(begin,start_token,started,started+3,maintained);
    observe(4,[&]{check(id);return flying()&&yes(runtime["permission"]);},"hold lost current vehicle permission/state");
    const auto finished=mono();const auto finish_token=hold_marker(end);
    hold_ack(end,finish_token,finished,finished+3,maintained);
    command(cycle+"-cancel-"+strategy,"action=cancel,target="+id);
    wait([&]{return statuses[id].count("CANCELLED")&&ready()&&!yes(runtime["permission"]);},3,"hold cancellation did not revoke local permission");
    marker(cycle+".cancel_"+strategy+".started");
    observe(1.2,[&]{return flying()&&!yes(runtime["permission"]);},"cancel silently regained local navigation");
    marker(cycle+".cancel_"+strategy+".finished");
  }
  void cycle(unsigned ordinal)
  {
    const std::string prefix="cycle"+std::to_string(ordinal),id="c"+std::to_string(ordinal);
    wait([&]{return ready()&&have_home&&numbers.count("NAV_X")&&numbers.count("NAV_Y")&&!yes(runtime["armed"])&&!yes(runtime["in_air"])&&!yes(runtime["permission"]);},55,"fresh disarmed/on-ground readiness timeout");
    if(ordinal==1)notify("MOOS_MANUAL_OVERRIDE","false");
    marker(prefix+".ground.started");observe(2,[&]{return ready()&&!yes(runtime["armed"])&&!yes(runtime["in_air"]);},"ground state changed");marker(prefix+".ground.finished");
    command(id+"-guided","action=guided,timeout=10");command(id+"-arm","action=arm,timeout=10");
    marker(prefix+".takeoff.started");command(id+"-takeoff","action=takeoff,height=5,timeout=45",48);marker(prefix+".takeoff.finished");
    marker(prefix+".route1.started");command(id+"-route1","action=goto,strategy=helm"+point(10,0)+",speed=1.5,radius=1.5,timeout=45",48);marker(prefix+".route1.finished");
    hold_window(prefix,"helm");
    marker(prefix+".route2.started");command(id+"-route2","action=goto,strategy=helm"+point(7,8)+",speed=1.5,radius=1.5,timeout=45",48);marker(prefix+".route2.finished");
    hold_window(prefix,"guided");
    std::ostringstream home;home<<std::setprecision(14)<<",home_lat="<<home_lat<<",home_lon="<<home_lon;
    marker(prefix+".return.started",home.str());command(id+"-return","action=return,strategy=helm,height=5,speed=1.5,radius=1.5,timeout=45",48);marker(prefix+".return.finished");
    marker(prefix+".land.started");command(id+"-land","action=land,timeout=60",65);
    if(yes(runtime["armed"]))command(id+"-disarm","action=disarm,timeout=10");
    wait([&]{return ready()&&!yes(runtime["armed"])&&!yes(runtime["in_air"])&&!yes(runtime["permission"]);},5,"landing did not end disarmed/on ground");
    marker(prefix+".land.finished");observe(2,[&]{return ready()&&!yes(runtime["armed"])&&!yes(runtime["in_air"])&&!yes(runtime["permission"]);},"final ground state changed");
    marker(prefix+".finished");
  }
};
}

int main(int argc,char** argv)
{
  Probe probe;std::string scenario="unknown";
  try{
    if(argc!=4)throw std::runtime_error("usage: copter_scenario_probe MOOS_PORT CONTROL_DIRECTORY SCENARIO");
    scenario=argv[3];if(scenario!="route_hold_return"&&scenario!="wind_repeated_cycle")throw std::runtime_error("unsupported scenario");
    const int port=std::stoi(argv[1]);if(port<1||port>65535)throw std::runtime_error("invalid MOOS port");
    probe.control=argv[2];std::filesystem::create_directories(probe.control);probe.client.SetQuiet(true);
    if(!probe.client.Run("127.0.0.1",port,"mavmoos_copter_scenario_probe",50))throw std::runtime_error("MOOS client startup failed");
    probe.cycle(1);if(scenario=="wind_repeated_cycle")probe.cycle(2);
    probe.log("pass",scenario,"complete lifecycle, stock helm route, held-target windows and explicit cancellation; no physical vehicle");
    probe.client.Close();return 0;
  }catch(const std::exception& error){probe.log("fail",scenario,error.what());probe.client.Close();return 1;}
}
