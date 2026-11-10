#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cmath>
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
  std::map<std::string,std::string> result;
  std::istringstream in(text); std::string part;
  while(std::getline(in,part,',')) {
    const auto split=part.find('=');
    if(split!=std::string::npos) result[part.substr(0,split)]=part.substr(split+1);
  }
  return result;
}
bool enabled(const std::string& text) { return text=="true"||text=="1"; }
std::string quote(const std::string& text)
{
  std::string result="\"";
  for(unsigned char c:text) {
    if(c=='"'||c=='\\') { result+='\\'; result+=static_cast<char>(c); }
    else if(c=='\n') result+="\\n";
    else if(c=='\r') result+="\\r";
    else if(c=='\t') result+="\\t";
    else if(c>=32) result+=static_cast<char>(c);
  }
  return result+'"';
}
}
int main(int argc,char** argv)
{
  CMOOSCommClient client;
  std::ofstream file;
  double start=monotonic();
  auto log=[&](const std::string& type,const std::string& key,const std::string& value) {
    std::ostringstream row;
    row<<"{\"elapsed\":"<<std::fixed<<std::setprecision(6)<<monotonic()-start
       <<",\"type\":"<<quote(type)<<",\"key\":"<<quote(key)<<",\"value\":"<<quote(value)<<"}";
    std::cout<<row.str()<<'\n'; std::cout.flush();
    if(file) { file<<row.str()<<'\n'; file.flush(); }
  };
  try {
    if(argc<2||argc>3) throw std::runtime_error("args: MOOSDB port [JSONL output, --release-demo, --pilot-guided-handback or --frame-lifecycle]");
    const bool pilot_handback=argc==3&&std::string(argv[2])=="--pilot-guided-handback";
    const bool release_demo=pilot_handback||(argc==3&&std::string(argv[2])=="--release-demo");
    const bool frame_lifecycle=argc==3&&std::string(argv[2])=="--frame-lifecycle";
    const int port=std::stoi(argv[1]);
    if(port<1||port>65535) throw std::runtime_error("invalid MOOSDB port");
    if(argc==3&&!release_demo&&!frame_lifecycle) { file.open(argv[2]); if(!file) throw std::runtime_error("cannot open output"); }
    client.SetQuiet(true);
    if(!client.Run("127.0.0.1",port,"mavmoos_sitl_workflow",30)) throw std::runtime_error("MOOS connection start failed");
    enum class Phase { Ready,RejectTakeoff,Guided,Arm,GroundDisarm,Rearm,Takeoff,Route,ReadInFlight,NormalPause,NormalResume,
      ExpiredPause,ExpiredRestored,ExpiredReacquire,ExpiredResumed,WaitForStation,StationSettle,
      PrepareHold,Hold,HoldStable,PilotHandback,GuidedFenced,Reacquire,Resume,Return,ReturnTravel,ReturnGroundDisarm,ReturnGuided,
      ReturnArm,ReturnTakeoff,Land,Disarm,Final };
    Phase phase=Phase::Ready;
    double phase_at=start,x0=0,y0=0,max_travel=0,last_nav=0,hold_at=0,normal_pause_at=0;
    unsigned acquire_requests=0,normal_acquire_requests=0;
    bool registered=false,sent_disarm=false;
    double hold_min_height=0,hold_max_height=0,last_runtime=-1;
    unsigned long long nav_at_read=0,nav_at_station=0,nav_at_takeover=0,nav_at_return=0;
    double return_distance=0;
    bool return_progress=false;
    const char* pilot_ready_path=std::getenv("MAVMOOS_SITL_FIXTURE_PILOT_READY");
    std::map<std::string,std::string> runtime,result_status,result_reason;
    std::map<std::string,double> numbers;
    std::set<std::string> helm_outputs,post_takeover_outputs;
    std::string last_helm,active_behaviors,read_state;
    bool route_complete=false,read_value=false;
    unsigned read_handoffs=0;
    auto transition=[&](Phase next,const std::string& name) {
      phase=next; phase_at=monotonic(); log("phase",name,"entered");
    };
    auto command=[&](const std::string& id,const std::string& action) {
      const std::string value="id="+id+",action="+action;
      client.Notify("MAVMOOS_COMMAND",value); log("command","MAVMOOS_COMMAND",value);
    };
    auto acquire_navigation=[&](const std::string& reason) {
      ++acquire_requests;
      client.Notify("MAVMOOS_NAV_CONTROL","acquire");
      log("operator","nav_control",reason+"; acquire_request_count="+std::to_string(acquire_requests));
    };
    auto nav=[&]() {
      const auto count=runtime.find("nav_tx_accepted");
      return count==runtime.end()?0ull:std::stoull(count->second);
    };
    auto active=[&]() {
      return monotonic()-last_runtime<.75&&enabled(runtime["permission"])&&runtime["fc_mode"]=="4"&&
        last_helm.find("state=ACTIVE")!=std::string::npos;
    };
    auto begin_normal_pause=[&](double at) {
      normal_acquire_requests=acquire_requests; normal_pause_at=at;
      client.Notify("DEPLOY","false");
      log("operator","deploy","false; begin bounded normal handoff");
      transition(Phase::NormalPause,"normal_pause");
    };
    auto done=[&](const std::string& id) { return result_status[id]=="COMPLETED"; };
    auto check_failure=[&](const std::string& id) {
      const auto status=result_status[id];
      if(status=="FAILED"||status=="TIMED_OUT"||status=="REJECTED")
        throw std::runtime_error(id+":"+status+":"+result_reason[id]);
    };
    log("phase","ready","waiting_for_fresh_vehicle_state");
    while(monotonic()-start<180) {
      const double now=monotonic();
      if(client.IsConnected()&&!registered) {
        for(const char* key:{"MAVMOOS_CAPABILITIES","MAVMOOS_RUNTIME_STATUS","MAVMOOS_COMMAND_RESULT","MAVMOOS_TX_RESULT",
            "MAVMOOS_HELM_STATUS","NAV_X","NAV_Y","NAV_ALTITUDE","DESIRED_HEADING","DESIRED_SPEED","DESIRED_ALTITUDE"})
          client.Register(key,0);
        if(release_demo)
          for(const char* key:{"ROUTE_COMPLETE","IVPHELM_BHV_ACTIVE","MAVMOOS_PARAM_RESULT",
                               "MAVMOOS_PARAM_VALUE","MAVMOOS_PARAM_TX_RESULT"})
            if(!client.Register(key,0)) throw std::runtime_error("release demo subscription failed");
        registered=true;
        client.Notify("DEPLOY","false");
      }
      MOOSMSG_LIST mail; client.Fetch(mail);
      for(const auto& message:mail) {
        const auto key=message.GetKey(),source=message.GetSource();
        if(source=="iMAVLink"&&message.IsString()) {
          if(key=="MAVMOOS_RUNTIME_STATUS") { runtime=fields(message.GetString()); last_runtime=now; }
          if(key=="MAVMOOS_COMMAND_RESULT") {
            const auto parsed=fields(message.GetString());
            if(parsed.count("id")&&parsed.count("status")) {
              result_status[parsed.at("id")]=parsed.at("status");
              result_reason[parsed.at("id")]=parsed.count("reason")?parsed.at("reason"):"unknown";
            }
          }
          if(key=="MAVMOOS_HELM_STATUS") last_helm=message.GetString();
          if(release_demo&&key=="MAVMOOS_PARAM_RESULT") {
            auto value=fields(message.GetString());
            if(value["id"]!="release-wind"||value["action"]!="read"||value["write_may_have_applied"]!="false")
              throw std::runtime_error("unexpected parameter operation in release demo");
            read_state=value["status"];
          }
          if(release_demo&&key=="MAVMOOS_PARAM_VALUE") {
            auto value=fields(message.GetString());
            if(value["id"]!="release-wind"||value["name"]!="SIM_WIND_SPD"||value["phase"]!="read"||
               std::stod(value["value"])!=0)
              throw std::runtime_error("wrong value for the simulator's pinned wind parameter");
            read_value=true;
          }
          if(release_demo&&key=="MAVMOOS_PARAM_TX_RESULT") {
            auto value=fields(message.GetString());
            if(value["id"]!="release-wind/initial"||value["status"]!="LOCAL_ACCEPTED"||
               value["late"]!="false"||value["clock_valid"]!="true"||++read_handoffs!=1)
              throw std::runtime_error("unexpected parameter read handoff");
          }
          log("observation",key,message.GetString());
        }
        if(source=="iMAVLink"&&message.IsDouble()&&key.rfind("NAV_",0)==0) {
          numbers[key]=message.GetDouble(); last_nav=now;
          if(key=="NAV_ALTITUDE") log("navigation",key,std::to_string(message.GetDouble()));
        }
        if(source=="pHelmIvP"&&message.IsDouble()&&key.rfind("DESIRED_",0)==0) {
          helm_outputs.insert(key); log("stock_helm",key,std::to_string(message.GetDouble()));
          if(phase==Phase::HoldStable||phase==Phase::GuidedFenced) post_takeover_outputs.insert(key);
        }
        if(release_demo&&source=="pHelmIvP"&&message.IsString()&&
           (key=="ROUTE_COMPLETE"||key=="IVPHELM_BHV_ACTIVE")) {
          if(key=="ROUTE_COMPLETE") route_complete=message.GetString()=="true";
          else active_behaviors=message.GetString();
          log("stock_helm",key,message.GetString());
        }
      }
      if(phase==Phase::Hold||phase==Phase::HoldStable||phase==Phase::Reacquire) {
        if(now-last_nav>0.5) throw std::runtime_error("Loiter height observation stale");
        hold_min_height=std::min(hold_min_height,numbers["NAV_ALTITUDE"]);
        hold_max_height=std::max(hold_max_height,numbers["NAV_ALTITUDE"]);
        if(hold_max_height-hold_min_height>0.5) throw std::runtime_error("Loiter pilot-neutral height varied more than0.5m");
      }
      const bool ready=enabled(runtime["heartbeat_valid"])&&enabled(runtime["position_valid"])&&
        enabled(runtime["home_valid"])&&enabled(runtime["estimator_valid"])&&enabled(runtime["landed_valid"]);
      if(phase==Phase::Ready) {
        if(ready&&numbers.count("NAV_X")&&numbers.count("NAV_Y")&&now-last_nav<1) {
          if(enabled(runtime["armed"])||enabled(runtime["in_air"])) throw std::runtime_error("owned simulator did not start disarmed/on-ground");
          command("workflow-reject-unarmed","takeoff,height=5"); transition(Phase::RejectTakeoff,"reject_unarmed_takeoff");
        } else if(now-phase_at>50) throw std::runtime_error("vehicle readiness timeout");
      } else if(phase==Phase::RejectTakeoff) {
        if(result_status["workflow-reject-unarmed"]=="REJECTED") {
          if(result_reason["workflow-reject-unarmed"]!="armed_required") throw std::runtime_error("unexpected unarmed takeoff rejection reason");
          command("workflow-guided","guided"); transition(Phase::Guided,"guided");
        } else if(now-phase_at>5) throw std::runtime_error("unarmed takeoff was not rejected");
      } else if(phase==Phase::Guided) {
        check_failure("workflow-guided");
        if(done("workflow-guided")) { command("workflow-arm","arm"); transition(Phase::Arm,"arm"); }
      } else if(phase==Phase::Arm) {
        check_failure("workflow-arm");
        if(done("workflow-arm")) {
          if(frame_lifecycle) {
            command("workflow-ground-disarm","disarm"); transition(Phase::GroundDisarm,"ground_disarm");
          } else {
            command("workflow-takeoff","takeoff,height=5"); transition(Phase::Takeoff,"takeoff");
          }
        }
      } else if(phase==Phase::GroundDisarm) {
        check_failure("workflow-ground-disarm");
        if(done("workflow-ground-disarm")) {
          if(enabled(runtime["armed"])||enabled(runtime["in_air"])) throw std::runtime_error("ground disarm lacked fresh state");
          command("workflow-rearm","arm"); transition(Phase::Rearm,"rearm");
        }
      } else if(phase==Phase::Rearm) {
        check_failure("workflow-rearm");
        if(done("workflow-rearm")) {
          command("workflow-takeoff","takeoff,height=5"); transition(Phase::Takeoff,"takeoff");
        }
      } else if(phase==Phase::Takeoff) {
        check_failure("workflow-takeoff");
        if(done("workflow-takeoff")) {
          if(!enabled(runtime["armed"])||!enabled(runtime["in_air"])) throw std::runtime_error("takeoff completed without observed airborne state");
          x0=numbers["NAV_X"]; y0=numbers["NAV_Y"];
          client.Notify("MOOS_MANUAL_OVERRIDE","false"); client.Notify("DEPLOY","true");
          acquire_navigation("initial explicit acquire");
          log("operator","deploy","true; explicit acquire"); transition(Phase::Route,"route");
        }
      } else if(phase==Phase::Route) {
        const double travel=std::hypot(numbers["NAV_X"]-x0,numbers["NAV_Y"]-y0);
        max_travel=std::max(max_travel,travel);
        if(now-phase_at>5&&max_travel>=4&&helm_outputs.size()==3&&enabled(runtime["permission"])&&runtime["fc_mode"]=="4") {
          log("assertion","route_distance_m",std::to_string(max_travel));
          if(release_demo) {
            if(!active()||route_complete||nav()==0||active_behaviors.find("copter_route")==std::string::npos||
               active_behaviors.find("route_finished_hold")==std::string::npos)
              throw std::runtime_error("parameter read requires the stock waypoint and overlapping station objectives");
            nav_at_read=nav();
            const std::string request="id=release-wind,action=read,name=SIM_WIND_SPD";
            if(!client.Notify("MAVMOOS_PARAM_REQUEST",request)) throw std::runtime_error("parameter request publication failed");
            log("request","MAVMOOS_PARAM_REQUEST",request);
            transition(Phase::ReadInFlight,"read_in_flight");
          } else begin_normal_pause(now);
        } else if(now-phase_at>28) throw std::runtime_error("stock helm route did not move four metres: "+last_helm);
      } else if(phase==Phase::ReadInFlight) {
        if(!active()||route_complete) throw std::runtime_error("parameter read interrupted active route");
        if(read_state=="FAILED"||read_state=="REJECTED"||read_state=="INCOMPLETE"||
           read_state=="TIMED_OUT"||read_state=="CANCELLED") throw std::runtime_error("parameter read failed: "+read_state);
        if(read_state=="COMPLETED"&&read_value&&read_handoffs==1&&nav()>nav_at_read&&now-phase_at>=.5) {
          log("assertion","parameter_read_during_route","name=SIM_WIND_SPD,value=0,nav_before="+
              std::to_string(nav_at_read)+",nav_after="+std::to_string(nav()));
          begin_normal_pause(now);
        } else if(now-phase_at>5) throw std::runtime_error("bounded in-flight parameter read did not complete");
      } else if(phase==Phase::NormalPause) {
        if(last_helm.find("state=LATCHED")!=std::string::npos)
          throw std::runtime_error("normal handoff latched before restoration: "+last_helm);
        if(last_helm.find("state=PAUSED")!=std::string::npos&&!enabled(runtime["permission"])) {
          client.Notify("DEPLOY","true");
          log("operator","deploy","true; restore normal handoff without acquire");
          transition(Phase::NormalResume,"normal_resume");
        } else if(now-phase_at>0.5) throw std::runtime_error("normal pause not observed within fixed 0.5s handoff window");
      } else if(phase==Phase::NormalResume) {
        if(last_helm.find("state=LATCHED")!=std::string::npos)
          throw std::runtime_error("normal handoff failed to resume in fixed window: "+last_helm);
        if(last_helm.find("state=ACTIVE")!=std::string::npos&&enabled(runtime["permission"])&&runtime["fc_mode"]=="4") {
          if(acquire_requests!=normal_acquire_requests) throw std::runtime_error("normal handoff required an extra acquire");
          log("assertion","normal_handoff_automatic_resume",std::string("passed; request_to_resume_s=")+std::to_string(now-normal_pause_at));
          client.Notify("DEPLOY","false");
          log("operator","deploy","false; exceed recovery window");
          transition(Phase::ExpiredPause,"expired_pause");
        } else if(now-phase_at>1) throw std::runtime_error("normal handoff resume not observed");
      } else if(phase==Phase::ExpiredPause) {
        if(now-phase_at>=0.7&&last_helm.find("state=LATCHED")!=std::string::npos&&!enabled(runtime["permission"])) {
          if(last_helm.find("reason=handoff_window_expired")==std::string::npos)
            throw std::runtime_error("long pause latched for an unexpected reason: "+last_helm);
          client.Notify("DEPLOY","true");
          log("operator","deploy","true; returning fields must not revive expired permission");
          transition(Phase::ExpiredRestored,"expired_fields_restored");
        } else if(now-phase_at>2) throw std::runtime_error("expired pause did not latch/inhibit navigation: "+last_helm);
      } else if(phase==Phase::ExpiredRestored) {
        if(last_helm.find("state=LATCHED")==std::string::npos||enabled(runtime["permission"]))
          throw std::runtime_error("fresh helm fields automatically revived expired handoff permission");
        if(now-phase_at>=0.5) {
          log("assertion","expired_handoff_requires_acquire","passed; fresh fields remained inhibited for at least0.5s");
          acquire_navigation("explicit acquire after expired handoff");
          transition(Phase::ExpiredReacquire,"expired_handoff_reacquire");
        }
      } else if(phase==Phase::ExpiredReacquire) {
        if(last_helm.find("state=ACTIVE")!=std::string::npos&&enabled(runtime["permission"])&&runtime["fc_mode"]=="4") {
          log("assertion","expired_handoff_explicit_reacquire","passed");
          transition(Phase::ExpiredResumed,"expired_handoff_resumed");
        } else if(now-phase_at>5) throw std::runtime_error("explicit acquire after expired handoff failed: "+last_helm);
      } else if(phase==Phase::ExpiredResumed) {
        if(runtime["fc_mode"]!="4"||!enabled(runtime["permission"]))
          throw std::runtime_error("restored navigation did not remain eligible");
        if(now-phase_at>0.3) {
          if(release_demo) transition(Phase::WaitForStation,"wait_for_station");
          else transition(Phase::PrepareHold,"prepare_hold");
        }
      } else if(phase==Phase::WaitForStation) {
        if(last_helm.find("state=LATCHED")!=std::string::npos)
          throw std::runtime_error("route-to-station transition latched navigation authority");
        if(active()&&route_complete&&active_behaviors.find("route_finished_hold")!=std::string::npos&&
           active_behaviors.find("copter_route")==std::string::npos) {
          nav_at_station=nav();
          transition(Phase::StationSettle,"station_settle");
        } else if(now-phase_at>28) throw std::runtime_error("stock waypoint did not transition to StationKeep");
      } else if(phase==Phase::StationSettle) {
        if(!active()||active_behaviors.find("route_finished_hold")==std::string::npos)
          throw std::runtime_error("stock StationKeep failed to hold navigation authority");
        if(now-phase_at>=4) {
          if(nav()<=nav_at_station) throw std::runtime_error("StationKeep produced no accepted navigation");
          log("assertion","stock_stationkeeping","route_complete=true,active=route_finished_hold");
          transition(Phase::PrepareHold,"prepare_hold");
        }
      } else if(phase==Phase::PrepareHold) {
        // The runner's separate pilot fixture first corroborates armed/airborne
        // height from its receive-only FC observer, then centers RC3 on the owned
        // native SITL RC port. This marker is local preparation, not FC receipt.
        std::string pilot_state;
        if(pilot_ready_path) { std::ifstream ready_file(pilot_ready_path); std::getline(ready_file,pilot_state); }
        if(pilot_state=="neutral-ready"&&ready&&enabled(runtime["armed"])&&enabled(runtime["in_air"])&&
           runtime["fc_mode"]=="4"&&now-last_nav<0.5&&numbers["NAV_ALTITUDE"]>=4) {
          hold_min_height=hold_max_height=numbers["NAV_ALTITUDE"];
          if(release_demo) transition(Phase::Hold,"external_takeover");
          else { command("workflow-hold","hold"); transition(Phase::Hold,"hold"); }
        } else if(now-phase_at>5) throw std::runtime_error("owned fixture pilot was not prepared before Loiter");
      } else if(phase==Phase::Hold) {
        if(!release_demo) check_failure("workflow-hold");
        const bool external=runtime["fc_mode"]=="5"&&!enabled(runtime["permission"])&&
          last_helm.find("state=LATCHED")!=std::string::npos&&
          last_helm.find("reason=external_mode_change,")!=std::string::npos;
        if((release_demo&&external)||(!release_demo&&done("workflow-hold"))) {
          nav_at_takeover=nav(); hold_at=now; transition(Phase::HoldStable,"hold_stable");
        }
      } else if(phase==Phase::HoldStable) {
        if(runtime["fc_mode"]!="5"||enabled(runtime["permission"])) throw std::runtime_error("late helm recaptured control during native Loiter hold");
        if(release_demo&&(nav()!=nav_at_takeover||now-last_runtime>.75))
          throw std::runtime_error("pilot takeover did not keep old navigation fenced");
        if(now-hold_at>=1.5) {
          if(release_demo&&post_takeover_outputs.size()!=3)
            throw std::runtime_error("fresh stock inputs were not observed during pilot hold");
          log("assertion","no_automatic_recapture","passed");
          log("assertion","loiter_hold_height_spread_m",std::to_string(hold_max_height-hold_min_height));
          if(pilot_handback) {
            post_takeover_outputs.clear();
            transition(Phase::PilotHandback,"external_guided_handback");
          } else {
            acquire_navigation("explicit acquire after native Loiter");
            log("operator","nav_control","explicit reacquire"); transition(Phase::Reacquire,"reacquire");
          }
        }
      } else if(phase==Phase::PilotHandback||phase==Phase::GuidedFenced) {
        if(enabled(runtime["permission"])||nav()!=nav_at_takeover||now-last_runtime>.75)
          throw std::runtime_error("pilot Guided handback revived old helm authority");
        if(phase==Phase::PilotHandback&&runtime["fc_mode"]=="4") {
          transition(Phase::GuidedFenced,"guided_fenced");
        } else if(phase==Phase::GuidedFenced) {
          if(runtime["fc_mode"]!="4") throw std::runtime_error("pilot Guided mode did not persist");
          if(now-phase_at>=2) {
            if(post_takeover_outputs.size()!=3)
              throw std::runtime_error("fresh stock inputs were not observed after pilot Guided handback");
            log("assertion","pilot_guided_handback_fenced","passed; nav_tx_accepted="+std::to_string(nav()));
            acquire_navigation("explicit acquire after pilot Guided handback");
            log("operator","nav_control","explicit reacquire"); transition(Phase::Reacquire,"reacquire");
          }
        }
      } else if(phase==Phase::Reacquire) {
        if(enabled(runtime["permission"])&&runtime["fc_mode"]=="4"&&last_helm.find("state=ACTIVE")!=std::string::npos)
          transition(Phase::Resume,"resume");
        else if(now-phase_at>8) throw std::runtime_error("explicit navigation reacquisition failed: "+last_helm);
      } else if(phase==Phase::Resume) {
        if(runtime["fc_mode"]!="4"||!enabled(runtime["permission"])) throw std::runtime_error("reacquired helm navigation did not remain eligible");
        if(now-phase_at>2) {
          if(release_demo&&(nav()<=nav_at_takeover||acquire_requests!=3))
            throw std::runtime_error("fresh navigation did not resume after the third explicit acquisition");
          if(frame_lifecycle) {
            return_distance=std::hypot(numbers["NAV_X"]-x0,numbers["NAV_Y"]-y0);
            if(return_distance<2) throw std::runtime_error("native RTL began too close to home");
            command("workflow-rtl","rtl"); transition(Phase::Return,"return");
          } else {
            client.Notify("DEPLOY","false"); command("workflow-land","land"); transition(Phase::Land,"land");
          }
        }
      } else if(phase==Phase::Return) {
        check_failure("workflow-rtl");
        if(result_status["workflow-rtl"]=="CONFIRMED"&&runtime["fc_mode"]=="6") {
          if(enabled(runtime["permission"])) throw std::runtime_error("native RTL left helm authority active");
          nav_at_return=nav(); transition(Phase::ReturnTravel,"return_travel");
        }
      } else if(phase==Phase::ReturnTravel) {
        check_failure("workflow-rtl");
        if(enabled(runtime["permission"])||nav()!=nav_at_return||
           (!done("workflow-rtl")&&runtime["fc_mode"]!="6"))
          throw std::runtime_error("stock helm changed authority or sent motion during native RTL");
        const auto distance=std::hypot(numbers["NAV_X"]-x0,numbers["NAV_Y"]-y0);
        if(!return_progress&&now-last_nav<.5&&return_distance-distance>=.75) {
          return_progress=true;
          log("assertion","native_homeward_progress_m",std::to_string(return_distance-distance));
        }
        if(done("workflow-rtl")&&!enabled(runtime["in_air"])) {
          if(!return_progress) throw std::runtime_error("native RTL landed without observed homeward progress");
          client.Notify("DEPLOY","false");
          if(enabled(runtime["armed"])) {
            command("workflow-rtl-disarm","disarm"); transition(Phase::ReturnGroundDisarm,"return_disarm");
          } else {
            command("workflow-guided-handback","guided"); transition(Phase::ReturnGuided,"return_guided");
          }
        }
      } else if(phase==Phase::ReturnGroundDisarm) {
        check_failure("workflow-rtl-disarm");
        if(done("workflow-rtl-disarm")&&!enabled(runtime["armed"])) {
          command("workflow-guided-handback","guided"); transition(Phase::ReturnGuided,"return_guided");
        }
      } else if(phase==Phase::ReturnGuided) {
        check_failure("workflow-guided-handback");
        if(done("workflow-guided-handback")&&runtime["fc_mode"]=="4") {
          if(enabled(runtime["permission"])||nav()!=nav_at_return)
            throw std::runtime_error("native Guided handback restored a retired helm grant");
          command("workflow-final-arm","arm"); transition(Phase::ReturnArm,"final_arm");
        }
      } else if(phase==Phase::ReturnArm) {
        check_failure("workflow-final-arm");
        if(done("workflow-final-arm")) {
          command("workflow-final-takeoff","takeoff,height=5"); transition(Phase::ReturnTakeoff,"final_takeoff");
        }
      } else if(phase==Phase::ReturnTakeoff) {
        check_failure("workflow-final-takeoff");
        if(done("workflow-final-takeoff")) {
          if(enabled(runtime["permission"])||nav()!=nav_at_return||!enabled(runtime["in_air"]))
            throw std::runtime_error("final takeoff lacked fresh airborne state or restored helm authority");
          command("workflow-land","land"); transition(Phase::Land,"land");
        }
      } else if(phase==Phase::Land) {
        check_failure("workflow-land");
        if(done("workflow-land")&&!enabled(runtime["in_air"])) {
          if(enabled(runtime["permission"])) throw std::runtime_error("landing left helm permission enabled");
          if(enabled(runtime["armed"])) { command("workflow-disarm","disarm"); sent_disarm=true; transition(Phase::Disarm,"disarm"); }
          else transition(Phase::Final,"final");
        }
      } else if(phase==Phase::Disarm) {
        if(done("workflow-disarm")||(result_status["workflow-disarm"]=="REJECTED"&&result_reason["workflow-disarm"]=="already_disarmed"))
          transition(Phase::Final,"final");
        else check_failure("workflow-disarm");
      } else if(phase==Phase::Final) {
        if(!enabled(runtime["armed"])&&!enabled(runtime["in_air"])&&!enabled(runtime["permission"])) {
          log("pass","full_copter_sitl_workflow","unarmed takeoff rejected; guided; arm; takeoff; stock helm route >=4m; bounded normal handoff auto-resumed; expired handoff latched until explicit acquire; Loiter without recapture; explicit reacquire; land; disarmed="+std::string(sent_disarm?"explicit":"automatic"));
          client.Close(); return 0;
        }
      }
      const double phase_limit=(phase==Phase::Takeoff||phase==Phase::ReturnTakeoff)?48:
        (phase==Phase::Land||phase==Phase::ReturnTravel)?65:12;
      if(phase!=Phase::Ready&&phase!=Phase::Route&&phase!=Phase::WaitForStation&&now-phase_at>phase_limit)
        throw std::runtime_error("workflow phase deadline");
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error("180-second workflow deadline");
  } catch(const std::exception& error) {
    log("fail","workflow",error.what()); client.Close(); return 1;
  }
}
