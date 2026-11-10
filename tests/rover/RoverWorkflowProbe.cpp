// Owned ground-Rover workflow through MOOS only. No direct MAVLink connection.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace {
double monotonic()
{ return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
std::map<std::string,std::string> fields(const std::string& text)
{
  std::map<std::string,std::string> result;std::istringstream in(text);std::string part;
  while(std::getline(in,part,',')) {
    const auto split=part.find('=');
    if(split!=std::string::npos)result[part.substr(0,split)]=part.substr(split+1);
  }
  return result;
}
bool enabled(const std::string& text){return text=="true"||text=="1";}
double number(const std::string& text,double fallback=-1)
{try{return std::stod(text);}catch(...){return fallback;}}
std::string quote(const std::string& text)
{
  std::string result="\"";
  for(unsigned char c:text) {
    if(c=='"'||c=='\\'){result+='\\';result+=static_cast<char>(c);}
    else if(c=='\n')result+="\\n";
    else if(c=='\r')result+="\\r";
    else if(c=='\t')result+="\\t";
    else if(c>=32)result+=static_cast<char>(c);
  }
  return result+'"';
}
}

int main(int argc,char** argv)
{
  CMOOSCommClient client;std::ofstream file;const double start=monotonic();
  auto log=[&](const std::string& type,const std::string& key,const std::string& value) {
    std::ostringstream row;row<<"{\"elapsed\":"<<std::fixed<<std::setprecision(6)<<monotonic()-start
      <<",\"type\":"<<quote(type)<<",\"key\":"<<quote(key)<<",\"value\":"<<quote(value)<<"}";
    std::cout<<row.str()<<'\n';std::cout.flush();if(file){file<<row.str()<<'\n';file.flush();}
  };
  try {
    if(argc!=4)throw std::runtime_error("usage: rover_workflow_probe MOOSDB_PORT JSONL_OUTPUT CONTROL_DIRECTORY");
    const int port=std::stoi(argv[1]);if(port<1||port>65535)throw std::runtime_error("invalid MOOSDB port");
    file.open(argv[2]);if(!file)throw std::runtime_error("cannot open probe output");
    const std::filesystem::path control=argv[3];
    if(!std::filesystem::is_directory(control))throw std::runtime_error("owned fixture control directory missing");
    client.SetQuiet(true);
    if(!client.Run("127.0.0.1",port,"mavmoos_rover_workflow",30))throw std::runtime_error("MOOS connection start failed");
    enum class Phase {Ready,RejectFlight,Guided,Arm,Route,Hold,HoldStable,Reacquire,RouteAgain,
      PilotTakeover,PilotStable,GuidedAgain,AcquireAgain,RouteRecovered,Return,FinalHold,Disarm,Final};
    Phase phase=Phase::Ready;double phase_at=start,x0=0,y0=0,maximum=0;
    bool registered=false;unsigned acquires=0;
    std::map<std::string,std::string> runtime,result_status,result_reason;
    std::map<std::string,double> values,value_at;std::set<std::string> helm_outputs;std::string helm;
    auto transition=[&](Phase next,const std::string& name){phase=next;phase_at=monotonic();log("phase",name,"entered");};
    auto command=[&](const std::string& id,const std::string& action){
      const std::string text="id="+id+",action="+action;client.Notify("MAVMOOS_COMMAND",text);log("command","MAVMOOS_COMMAND",text);
    };
    auto acquire=[&]{++acquires;client.Notify("MAVMOOS_NAV_CONTROL","acquire");log("operator","acquire",std::to_string(acquires));};
    auto begin_motion=[&]{x0=values["NAV_X"];y0=values["NAV_Y"];};
    auto travel=[&]{return std::hypot(values["NAV_X"]-x0,values["NAV_Y"]-y0);};
    auto done=[&](const std::string& id){return result_status[id]=="COMPLETED";};
    auto failure=[&](const std::string& id){const auto status=result_status[id];
      if(status=="REJECTED"||status=="FAILED"||status=="TIMED_OUT"||status=="CANCELLED")
        throw std::runtime_error(id+":"+status+":"+result_reason[id]);
    };
    log("phase","ready","waiting for horizontal state; altitude and airborne state are not prerequisites");
    while(monotonic()-start<180) {
      const double now=monotonic(),elapsed=now-phase_at;
      if(client.IsConnected()&&!registered) {
        for(const auto* key:{"MAVMOOS_CAPABILITIES","MAVMOOS_RUNTIME_STATUS","MAVMOOS_COMMAND_RESULT","MAVMOOS_TX_RESULT",
          "MAVMOOS_HELM_STATUS","MAVMOOS_ESTIMATOR","NAV_X","NAV_Y","NAV_SPEED","DESIRED_HEADING","DESIRED_SPEED","DESIRED_ALTITUDE"})client.Register(key,0);
        registered=true;client.Notify("DEPLOY","false");
      }
      MOOSMSG_LIST mail;client.Fetch(mail);
      for(const auto& message:mail) {
        const auto key=message.GetKey(),source=message.GetSource();
        if(source=="iMAVLink"&&message.IsString()) {
          if(key=="MAVMOOS_RUNTIME_STATUS")runtime=fields(message.GetString());
          if(key=="MAVMOOS_HELM_STATUS")helm=message.GetString();
          if(key=="MAVMOOS_COMMAND_RESULT") {
            const auto parsed=fields(message.GetString());
            if(parsed.count("id")&&parsed.count("status")){result_status[parsed.at("id")]=parsed.at("status");result_reason[parsed.at("id")]=parsed.count("reason")?parsed.at("reason"):"unknown";}
          }
          log("observation",key,message.GetString());
        }
        if(source=="iMAVLink"&&message.IsDouble()&&key.rfind("NAV_",0)==0){values[key]=message.GetDouble();value_at[key]=now;}
        if(source=="pHelmIvP"&&message.IsDouble()&&key.rfind("DESIRED_",0)==0){helm_outputs.insert(key);log("stock_helm",key,std::to_string(message.GetDouble()));}
      }
      const unsigned flags=static_cast<unsigned>(number(runtime["estimator_flags"],0));
      const bool ready=enabled(runtime["heartbeat_valid"])&&enabled(runtime["position_valid"])&&
        enabled(runtime["estimator_horizontal_valid"])&&(flags&19)==19&&!(flags&1024)&&
        number(runtime["estimator_age_s"])>=0&&number(runtime["estimator_age_s"])<=3;
      const bool fresh_nav=values.count("NAV_X")&&values.count("NAV_Y")&&now-value_at["NAV_X"]<=.5&&now-value_at["NAV_Y"]<=.5;
      const bool permission=enabled(runtime["permission"]),armed=enabled(runtime["armed"]);
      const bool guided=runtime["fc_mode"]=="15";
      const bool active=permission&&guided&&helm.find("state=ACTIVE")!=std::string::npos;
      if(helm_outputs.count("DESIRED_ALTITUDE"))throw std::runtime_error("2D stock helm unexpectedly published altitude");
      if(phase==Phase::Ready) {
        if(ready&&fresh_nav) {
          if(armed)throw std::runtime_error("owned simulator did not start disarmed");
          command("rover-reject-flight","takeoff");transition(Phase::RejectFlight,"reject_flight_action");
        }else if(elapsed>65)throw std::runtime_error("horizontal vehicle readiness timeout");
      }else if(phase==Phase::RejectFlight) {
        if(result_status["rover-reject-flight"]=="REJECTED"){
          if(result_reason["rover-reject-flight"]!="invalid_or_unsupported_rover_request")throw std::runtime_error("wrong unsupported flight rejection");
          log("assertion","flight_action_rejected","passed");command("rover-guided","guided");transition(Phase::Guided,"guided");
        }else if(elapsed>5)throw std::runtime_error("unsupported flight action was not rejected");
      }else if(phase==Phase::Guided) {
        failure("rover-guided");if(done("rover-guided")){command("rover-arm","arm");transition(Phase::Arm,"arm");}
        else if(elapsed>12)throw std::runtime_error("Guided outcome timeout");
      }else if(phase==Phase::Arm) {
        failure("rover-arm");if(done("rover-arm")){
          begin_motion();client.Notify("MOOS_MANUAL_OVERRIDE","false");client.Notify("DEPLOY","true");acquire();transition(Phase::Route,"stock_helm_route");
        }else if(elapsed>12)throw std::runtime_error("Arm outcome timeout");
      }else if(phase==Phase::Route) {
        if(fresh_nav)maximum=std::max(maximum,travel());
        if(elapsed>=3&&maximum>=4&&active&&helm_outputs.size()==2){
          log("assertion","two_dimensional_helm_motion_m",std::to_string(maximum));command("rover-hold","hold");transition(Phase::Hold,"native_hold");
        }else if(elapsed>25)throw std::runtime_error("stock helm did not move four metres: "+helm);
      }else if(phase==Phase::Hold) {
        failure("rover-hold");if(done("rover-hold")){
          if(permission||runtime["fc_mode"]!="4")throw std::runtime_error("Hold completed with navigation permission or wrong mode");
          transition(Phase::HoldStable,"native_hold_stable");
        }else if(elapsed>12)throw std::runtime_error("Hold outcome timeout");
      }else if(phase==Phase::HoldStable) {
        if(permission||runtime["fc_mode"]!="4")throw std::runtime_error("helm recaptured native Hold without explicit acquire");
        if(elapsed>=1){log("assertion","native_hold_blocks_helm_recapture","passed with DEPLOY still true");begin_motion();acquire();transition(Phase::Reacquire,"acquire_from_hold");}
      }else if(phase==Phase::Reacquire) {
        if(active)transition(Phase::RouteAgain,"route_after_hold");else if(elapsed>8)throw std::runtime_error("explicit acquire from Hold failed: "+helm);
      }else if(phase==Phase::RouteAgain) {
        if(fresh_nav&&travel()>=2&&active){
          std::ofstream marker(control/"takeover.request");marker<<"native RC mode switch requested\n";marker.close();
          log("operator","fixture_pilot","request exact owned native RC8 Manual takeover");transition(Phase::PilotTakeover,"pilot_takeover");
        }else if(elapsed>15)throw std::runtime_error("route after Hold did not move");
      }else if(phase==Phase::PilotTakeover) {
        if(runtime["fc_mode"]=="0"&&!permission&&helm.find("state=LATCHED")!=std::string::npos)transition(Phase::PilotStable,"pilot_manual_stable");
        else if(elapsed>6)throw std::runtime_error("pilot Manual takeover not observed: "+helm);
      }else if(phase==Phase::PilotStable) {
        if(runtime["fc_mode"]!="0"||permission)throw std::runtime_error("pilot Manual control automatically recaptured");
        if(elapsed>=1.2){log("assertion","pilot_takeover_latches","passed; fresh helm fields stayed inhibited");std::filesystem::remove(control/"takeover.request");transition(Phase::GuidedAgain,"explicit_guided_after_pilot");}
      }else if(phase==Phase::GuidedAgain) {
        if(runtime["fc_mode"]=="4"){begin_motion();acquire();transition(Phase::AcquireAgain,"explicit_reacquire_after_pilot");}
        else if(elapsed>12)throw std::runtime_error("pilot did not select Hold");
      }else if(phase==Phase::AcquireAgain) {
        if(active)transition(Phase::RouteRecovered,"route_recovered");else if(elapsed>8)throw std::runtime_error("explicit pilot recovery failed: "+helm);
      }else if(phase==Phase::RouteRecovered) {
        if(fresh_nav&&travel()>=1.5&&active&&enabled(runtime["home_valid"])){
          if(acquires!=3)throw std::runtime_error("unexpected acquire count");
          log("assertion","explicit_pilot_recovery_motion_m",std::to_string(travel()));client.Notify("DEPLOY","false");
          command("rover-rtl","rtl");transition(Phase::Return,"native_return");
        }else if(elapsed>15)throw std::runtime_error("recovered route/home unavailable");
      }else if(phase==Phase::Return) {
        failure("rover-rtl");if(done("rover-rtl")){
          if(permission||runtime["fc_mode"]!="11")throw std::runtime_error("RTL completed with wrong authority/mode");
          log("assertion","native_return_observed","home-bound fixture outcome completed");command("rover-final-hold","hold");transition(Phase::FinalHold,"final_hold");
        }else if(elapsed>95)throw std::runtime_error("native return timeout");
      }else if(phase==Phase::FinalHold) {
        failure("rover-final-hold");if(done("rover-final-hold")){command("rover-disarm","disarm");transition(Phase::Disarm,"disarm");}
        else if(elapsed>12)throw std::runtime_error("final Hold timeout");
      }else if(phase==Phase::Disarm) {
        failure("rover-disarm");if(done("rover-disarm"))transition(Phase::Final,"final");else if(elapsed>12)throw std::runtime_error("disarm outcome timeout");
      }else if(phase==Phase::Final&&elapsed>=1) {
        if(armed||permission||!fresh_nav||!values.count("NAV_SPEED")||now-value_at["NAV_SPEED"]>.5||
           !std::isfinite(values["NAV_SPEED"])||values["NAV_SPEED"]>.4)throw std::runtime_error("final disarmed stop not observed");
        log("result","PASS","stock 2D helm; native Hold; RC Manual takeover; explicit reacquisition; native RTL; disarm; no hardware");client.Close();return 0;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error("global 180 second workflow deadline exceeded");
  }catch(const std::exception& error){log("result","FAIL",error.what());client.Close();return 1;}
}
