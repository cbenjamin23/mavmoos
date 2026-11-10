/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: CopterOperationProbe.cpp                        */
/*    DATE: November 4, 2026                                */
/************************************************************/
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
namespace {
double monotonic() {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
std::string quote(const std::string& text) {
  std::string result="\"";
  for(unsigned char c:text) {
    if(c=='"'||c=='\\') {result+='\\';result+=static_cast<char>(c);}
    else if(c=='\n')result+="\\n";else if(c=='\r')result+="\\r";else if(c=='\t')result+="\\t";else if(c>=32)result+=static_cast<char>(c);
  }
  return result+'"';
}
std::map<std::string,std::string> fields(const std::string& text) {
  std::map<std::string,std::string> result;std::istringstream stream(text);std::string part;
  while(std::getline(stream,part,',')) {const auto at=part.find('=');if(at!=std::string::npos)result[part.substr(0,at)]=part.substr(at+1);}
  return result;
}
bool yes(const std::string& value) {return value=="true"||value=="1";}
}
int main(int argc,char** argv) {
 CMOOSCommClient client;const double started=monotonic();
 auto log=[&](const std::string& type,const std::string& key,const std::string& value,double moos_at=-1) {
   std::cout<<"{\"elapsed\":"<<std::fixed<<std::setprecision(6)<<monotonic()-started<<",\"mono\":"<<monotonic()<<",\"type\":"<<quote(type)<<",\"key\":"<<quote(key)<<",\"value\":"<<quote(value);
   if(moos_at>=0)std::cout<<",\"moos_at\":"<<std::setprecision(9)<<moos_at;
   std::cout<<"}\n";std::cout.flush();
 };
 try {
  const bool parity=argc==5&&std::string(argv[2])=="--parity";
  if(argc!=2&&!parity)throw std::runtime_error("usage: copter_operation_probe MOOSDB_PORT [--parity BEACON_LAT BEACON_LON]");
  const double beacon_lat=parity?std::stod(argv[3]):0,beacon_lon=parity?std::stod(argv[4]):0;
  if(parity&&(!std::isfinite(beacon_lat)||!std::isfinite(beacon_lon)||std::abs(beacon_lat)>89||std::abs(beacon_lon)>180))
    throw std::runtime_error("invalid fixture beacon coordinates");
  const int port=std::stoi(argv[1]);if(port<1||port>65535)throw std::runtime_error("invalid MOOSDB port");
  client.SetQuiet(true);if(!client.Run("127.0.0.1",port,"mavmoos_copter_operations",40))throw std::runtime_error("MOOS client start failed");
  enum class Phase {Ready,Rejected,Guided,Arm,Takeoff,ResetSpeed,ChangeSpeed,ChangeCourse,AdjustAltitude,VisualizeHome,
                    HelmSeed,HelmAdjustSpeed,HelmAdjustAltitude,Survey,SurveyCancel,HelmReseed,Voronoi,VoronoiCancel,
                    GuidedGoto,GuidedHold,GuidedHoldCancel,HelmGoto,HelmHold,HelmHoldCancel,HelmReturn,
                    PrepareHold,NativeHold,NativeHoldStable,PrecisionOn,PrecisionOff,PrecisionRetreat,
                    PrepareDisabledHold,DisabledHold,DisabledDwell,StageRtl,Rtl,Disarm,Final};
  Phase phase=Phase::Ready;double phase_at=started,last_nav=0,home_lat=0,home_lon=0,hold_min=0,hold_max=0;
  double precision_since=-1;
  bool registered=false,have_home=false,home_marker=false;std::set<std::string> helm_outputs;
  std::map<std::string,std::string> runtime,last_status,reasons;
  std::map<std::string,std::set<std::string>> statuses;
  std::map<std::string,double> numbers;
  struct Sample {double value=0,at=-1;};
  std::map<std::string,Sample> helm_values;
  Sample speed_update,height_update;
  std::string helm_state,helm_selector;
  double selector_at=-1,phase_moos=-1,expected_tune=0;
  const char* pilot_ready=std::getenv("MAVMOOS_SITL_FIXTURE_PILOT_READY");
  auto transition=[&](Phase next,const std::string& name){phase=next;phase_at=monotonic();log("phase",name,"entered");};
  auto operation=[&](const std::string& id,const std::string& body){const auto request="schema=1,id="+id+","+body;client.Notify("MAVMOOS_CONTROL_REQUEST",request);log("request","MAVMOOS_CONTROL_REQUEST",request);};
  auto command=[&](const std::string& id,const std::string& body){
    const auto request="id="+id+","+body;
    client.Notify("MAVMOOS_COMMAND",request);log("request","MAVMOOS_COMMAND",request);
  };
  auto done=[&](const std::string& id){return statuses[id].count("COMPLETED")!=0;};
  auto confirmed=[&](const std::string& id){return statuses[id].count("CONFIRMED")!=0;};
  auto check=[&](const std::string& id){const auto& s=last_status[id];if(s=="REJECTED"||s=="TIMED_OUT"||s=="FAILED"||s=="CANCELLED")throw std::runtime_error(id+":"+s+":"+reasons[id]);};
  auto point=[&](double east,double north){std::ostringstream value;value<<std::setprecision(12)<<",lat="<<home_lat+north/111195.<<",lon="<<home_lon+east/(111195.*std::cos(home_lat*3.14159265358979323846/180.))<<",height=5";return value.str();};
  auto beacon_distance=[&](double now){
    if(now-last_nav>.5||!numbers.count("NAV_LAT")||!numbers.count("NAV_LONG"))return std::numeric_limits<double>::infinity();
    const double north=111195.*(numbers.at("NAV_LAT")-beacon_lat);
    const double east=111195.*std::cos(beacon_lat*3.14159265358979323846/180.)*(numbers.at("NAV_LONG")-beacon_lon);
    return std::hypot(north,east);
  };
  auto guidedGoto=[&]{operation("ops-guided-goto","action=goto,strategy=guided"+point(7,0)+",facing=keep,timeout=35");transition(Phase::GuidedGoto,"guided_goto");};
  auto stageRtl=[&]{operation("ops-rtl-stage","action=goto,strategy=guided"+point(-8,0)+",timeout=35");transition(Phase::StageRtl,"rtl_staging_goto");};
  auto notify=[&](const std::string& key,const std::string& value){client.Notify(key,value);log("request",key,value);};
  auto phaseBoundary=[&](Phase next,const std::string& name){phase_moos=MOOSTime();transition(next,name);};
  auto freshHelm=[&](double after){
    if(helm_state!="ACTIVE")return false;
    const double now=MOOSTime();
    for(const char* key:{"DESIRED_HEADING","DESIRED_SPEED","DESIRED_ALTITUDE"}) {
      const auto item=helm_values.find(key);
      if(item==helm_values.end()||!std::isfinite(item->second.value)||item->second.at<=after||
         item->second.at>now+.1||now-item->second.at>.5)return false;
    }
    return true;
  };
  const std::string rtl_id=parity?"deadman.1":"ops-native-rtl";
  log("phase","ready","waiting_for_fresh_vehicle_state");
  while(monotonic()-started<300) {
    const double now=monotonic();
    if(client.IsConnected()&&!registered) {
      for(const char* key:{"MAVMOOS_CAPABILITIES","MAVMOOS_RUNTIME_STATUS","MAVMOOS_CONTROL_RESULT","MAVMOOS_CONTROL_STATUS","MAVMOOS_COMMAND_RESULT","MAVMOOS_TX_RESULT","MAVMOOS_HELM_STATUS","MAVMOOS_HOME_POSITION","NAV_X","NAV_Y","NAV_LAT","NAV_LONG","NAV_ALTITUDE","NAV_SPEED","DESIRED_HEADING","DESIRED_SPEED","DESIRED_ALTITUDE","MAVMOOS_HELM_OPERATION","MAVMOOS_OPERATION_ROUTE_UPDATE","MAVMOOS_OPERATION_HOLD_UPDATE"})client.Register(key,0);
      if(parity)for(const char* key:{"VIEW_MARKER","MAVMOOS_PARAM_TX_RESULT","MAVMOOS_HELM_SPEED_UPDATE","MAVMOOS_OPERATION_ALTITUDE_UPDATE"})client.Register(key,0);
      registered=true;
    }
    MOOSMSG_LIST mail;client.Fetch(mail);
    for(const auto& item:mail) {
      const auto key=item.GetKey(),source=item.GetSource();
      if(source=="iMAVLink"&&item.IsString()) {
        const auto value=item.GetString();log("observation",key,value,parity?item.GetTime():-1);
        if(key=="MAVMOOS_RUNTIME_STATUS")runtime=fields(value);
        if(key=="MAVMOOS_CONTROL_RESULT"||(parity&&key=="MAVMOOS_COMMAND_RESULT")) {const auto f=fields(value);if(f.count("id")&&f.count("status")) {last_status[f.at("id")]=f.at("status");statuses[f.at("id")].insert(f.at("status"));reasons[f.at("id")]=f.count("reason")?f.at("reason"):"unknown";}}
        if(parity&&key=="VIEW_MARKER"&&phase==Phase::VisualizeHome) {const auto f=fields(value);home_marker=f.count("label")&&f.at("label")=="MAVMOOS_HOME";}
        if(parity&&key=="MAVMOOS_HELM_STATUS")helm_state=fields(value)["state"];
        if(parity&&key=="MAVMOOS_HELM_OPERATION"&&item.GetTime()>=selector_at) {helm_selector=value;selector_at=item.GetTime();}
        if(parity&&key=="MAVMOOS_HELM_SPEED_UPDATE")speed_update={std::stod(fields(value).at("speed")),item.GetTime()};
        if(parity&&key=="MAVMOOS_OPERATION_ALTITUDE_UPDATE")height_update={std::stod(fields(value).at("altitude")),item.GetTime()};
        if(key=="MAVMOOS_HOME_POSITION") {const auto f=fields(value);if(f.count("lat_deg")&&f.count("lon_deg")&&f.count("encoding_valid")&&yes(f.at("encoding_valid"))) {home_lat=std::stod(f.at("lat_deg"));home_lon=std::stod(f.at("lon_deg"));have_home=true;}}
      }
      if(source=="iMAVLink"&&item.IsDouble()&&key.rfind("NAV_",0)==0) {numbers[key]=item.GetDouble();last_nav=now;if(key=="NAV_ALTITUDE")log("navigation",key,std::to_string(item.GetDouble()));}
      if(source=="pHelmIvP"&&item.IsDouble()&&key.rfind("DESIRED_",0)==0) {
        helm_outputs.insert(key);log("stock_helm",key,std::to_string(item.GetDouble()),parity?item.GetTime():-1);
        if(parity&&item.GetTime()>=helm_values[key].at)helm_values[key]={item.GetDouble(),item.GetTime()};
      }
    }
    const bool ready=yes(runtime["heartbeat_valid"])&&yes(runtime["position_valid"])&&yes(runtime["home_valid"])&&yes(runtime["estimator_valid"])&&yes(runtime["landed_valid"]);
    if(phase==Phase::Ready) {
      if(ready&&have_home&&numbers.count("NAV_X")&&numbers.count("NAV_Y")&&now-last_nav<.5) {
        if(yes(runtime["armed"])||yes(runtime["in_air"]))throw std::runtime_error("simulator did not begin disarmed/on-ground");
        operation("ops-reject-ground","action=hold,strategy=guided");transition(Phase::Rejected,"reject_ground_navigation");
      } else if(now-phase_at>60)throw std::runtime_error("vehicle readiness timeout");
    } else if(phase==Phase::Rejected) {
      if(last_status["ops-reject-ground"]=="REJECTED") {operation("ops-guided","action=guided,timeout=10");transition(Phase::Guided,"guided");}
      else if(now-phase_at>5)throw std::runtime_error("ground navigation not rejected");
    } else if(phase==Phase::Guided) {
      check("ops-guided");if(done("ops-guided")) {operation("ops-arm","action=arm,timeout=10");transition(Phase::Arm,"arm");}
    } else if(phase==Phase::Arm) {
      check("ops-arm");if(done("ops-arm")) {operation("ops-takeoff","action=takeoff,height=5,timeout=45");transition(Phase::Takeoff,"takeoff");}
    } else if(phase==Phase::Takeoff) {
      check("ops-takeoff");if(done("ops-takeoff")) {
        if(parity) {command("parity-reset-speed","action=reset_speed_min");transition(Phase::ResetSpeed,"parity_reset_speed");}
        else guidedGoto();
      }
    } else if(phase==Phase::ResetSpeed) {
      check("parity-reset-speed");if(last_status["parity-reset-speed"]=="UNCONFIRMED") {
        command("parity-change-speed","action=change_speed,delta=.3");transition(Phase::ChangeSpeed,"parity_change_speed");
      }
    } else if(phase==Phase::ChangeSpeed) {
      check("parity-change-speed");if(last_status["parity-change-speed"]=="UNCONFIRMED") {
        command("parity-change-course","action=change_course,delta=20");transition(Phase::ChangeCourse,"parity_change_course");
      }
    } else if(phase==Phase::ChangeCourse) {
      check("parity-change-course");if(done("parity-change-course")) {
        operation("ops-adjust-altitude","action=adjust_altitude,strategy=guided,delta=1,timeout=20");
        transition(Phase::AdjustAltitude,"parity_adjust_altitude");
      }
    } else if(phase==Phase::AdjustAltitude) {
      check("ops-adjust-altitude");if(done("ops-adjust-altitude")) {
        operation("ops-visualize-home","action=visualize_home,timeout=5");transition(Phase::VisualizeHome,"parity_visualize_home");
      }
    } else if(phase==Phase::VisualizeHome) {
      check("ops-visualize-home");if(done("ops-visualize-home")&&home_marker) {
        phaseBoundary(Phase::HelmSeed,"parity_helm_seed");
        notify("PARITY_HELM_SEED","true");notify("MAVMOOS_NAV_CONTROL","acquire");
      }
    } else if(phase==Phase::HelmSeed) {
      if(freshHelm(phase_moos)) {
        expected_tune=helm_values.at("DESIRED_SPEED").value+.5;
        phaseBoundary(Phase::HelmAdjustSpeed,"parity_helm_adjust_speed");
        operation("ops-helm-adjust-speed","action=adjust_speed,strategy=helm,delta=.5,timeout=10");
      }
    } else if(phase==Phase::HelmAdjustSpeed) {
      check("ops-helm-adjust-speed");
      if(last_status["ops-helm-adjust-speed"]=="ACCEPTED"&&speed_update.at>phase_moos&&freshHelm(speed_update.at)&&
         std::abs(speed_update.value-expected_tune)<.01&&std::abs(helm_values.at("DESIRED_SPEED").value-expected_tune)<.11) {
        expected_tune=helm_values.at("DESIRED_ALTITUDE").value+1;
        phaseBoundary(Phase::HelmAdjustAltitude,"parity_helm_adjust_altitude");
        operation("ops-helm-adjust-altitude","action=adjust_altitude,strategy=helm,delta=1,timeout=10");
      }
    } else if(phase==Phase::HelmAdjustAltitude) {
      check("ops-helm-adjust-altitude");
      if(last_status["ops-helm-adjust-altitude"]=="ACCEPTED"&&height_update.at>phase_moos&&freshHelm(height_update.at)&&
         std::abs(height_update.value-expected_tune)<.01&&std::abs(helm_values.at("DESIRED_ALTITUDE").value-expected_tune)<.11) {
        phaseBoundary(Phase::Survey,"parity_helm_survey");
        operation("ops-survey","action=survey,strategy=helm,timeout=12");
      }
    } else if(phase==Phase::Survey) {
      check("ops-survey");
      if(confirmed("ops-survey")&&helm_selector=="survey"&&selector_at>phase_moos&&freshHelm(selector_at)) {
        phaseBoundary(Phase::SurveyCancel,"parity_helm_survey_cancel");
        operation("ops-survey-cancel","action=cancel,target=ops-survey");
      }
    } else if(phase==Phase::SurveyCancel) {
      check("ops-survey-cancel");
      if(done("ops-survey-cancel")&&statuses["ops-survey"].count("CANCELLED")&&helm_selector=="none"&&selector_at>phase_moos) {
        phaseBoundary(Phase::HelmReseed,"parity_helm_reseed");
        notify("MAVMOOS_NAV_CONTROL","acquire");
      }
    } else if(phase==Phase::HelmReseed) {
      if(freshHelm(phase_moos)) {
        phaseBoundary(Phase::Voronoi,"parity_helm_voronoi");
        operation("ops-voronoi","action=voronoi,strategy=helm,timeout=12");
      }
    } else if(phase==Phase::Voronoi) {
      check("ops-voronoi");
      if(confirmed("ops-voronoi")&&helm_selector=="voronoi"&&selector_at>phase_moos&&freshHelm(selector_at)) {
        phaseBoundary(Phase::VoronoiCancel,"parity_helm_voronoi_cancel");
        operation("ops-voronoi-cancel","action=cancel,target=ops-voronoi");
      }
    } else if(phase==Phase::VoronoiCancel) {
      check("ops-voronoi-cancel");
      if(done("ops-voronoi-cancel")&&statuses["ops-voronoi"].count("CANCELLED")&&helm_selector=="none"&&selector_at>phase_moos) {
        notify("PARITY_HELM_SEED","false");guidedGoto();
      }
    } else if(phase==Phase::GuidedGoto) {
      check("ops-guided-goto");if(done("ops-guided-goto")) {operation("ops-guided-hold","action=hold,strategy=guided,facing=heading,heading=90,timeout=15");transition(Phase::GuidedHold,"guided_hold");}
    } else if(phase==Phase::GuidedHold) {
      check("ops-guided-hold");if(confirmed("ops-guided-hold")&&now-phase_at>=1.5) {operation("ops-guided-cancel","action=cancel,target=ops-guided-hold");transition(Phase::GuidedHoldCancel,"guided_hold_cancel");}
    } else if(phase==Phase::GuidedHoldCancel) {
      check("ops-guided-cancel");if(done("ops-guided-cancel")&&statuses["ops-guided-hold"].count("CANCELLED")) {operation("ops-helm-goto","action=goto,strategy=helm"+point(2,5)+",speed=1,timeout=40");transition(Phase::HelmGoto,"helm_goto");}
    } else if(phase==Phase::HelmGoto) {
      check("ops-helm-goto");if(done("ops-helm-goto")) {if(helm_outputs.size()!=3)throw std::runtime_error("all three ordinary helm scalars not observed");operation("ops-helm-hold","action=hold,strategy=helm,timeout=15");transition(Phase::HelmHold,"helm_hold");}
    } else if(phase==Phase::HelmHold) {
      check("ops-helm-hold");if(confirmed("ops-helm-hold")&&now-phase_at>=1.5) {operation("ops-helm-cancel","action=cancel,target=ops-helm-hold");transition(Phase::HelmHoldCancel,"helm_hold_cancel");}
    } else if(phase==Phase::HelmHoldCancel) {
      check("ops-helm-cancel");if(done("ops-helm-cancel")&&statuses["ops-helm-hold"].count("CANCELLED")) {operation("ops-helm-return","action=return,strategy=helm,height=5,speed=1,timeout=35");transition(Phase::HelmReturn,"helm_return");}
    } else if(phase==Phase::HelmReturn) {
      check("ops-helm-return");if(done("ops-helm-return")) {
        if(runtime["fc_mode"]!="4"||!yes(runtime["armed"])||!yes(runtime["in_air"]))throw std::runtime_error("helm return was silently mapped to landing/RTL");
        log("assertion","helm_return_semantics","home region reached while armed/airborne Guided; no landing implied");transition(Phase::PrepareHold,"prepare_hold");
      }
    } else if(phase==Phase::PrepareHold) {
      if(pilot_ready&&std::ifstream(pilot_ready).good()) {hold_min=hold_max=numbers["NAV_ALTITUDE"];operation("ops-native-hold","action=hold,strategy=native,timeout=10");transition(Phase::NativeHold,"native_hold");}
      else if(now-phase_at>6)throw std::runtime_error("scoped neutral pilot readiness unavailable");
    } else if(phase==Phase::NativeHold) {
      check("ops-native-hold");if(done("ops-native-hold")&&runtime["fc_mode"]=="5")transition(Phase::NativeHoldStable,"native_hold_stable");
    } else if(phase==Phase::NativeHoldStable) {
      if(now-last_nav>.5)throw std::runtime_error("native Loiter height stale");
      hold_min=std::min(hold_min,numbers["NAV_ALTITUDE"]);hold_max=std::max(hold_max,numbers["NAV_ALTITUDE"]);
      if(hold_max-hold_min>.5)throw std::runtime_error("native Loiter height spread exceeds0.5m");
      if(now-phase_at>=(parity?3.:1.5)) {
        log("assertion","native_loiter_height_spread_m",std::to_string(hold_max-hold_min));
        if(parity) {
          if(runtime["fc_mode"]!="5")throw std::runtime_error("precision test requires native Loiter");
          const double from_beacon=beacon_distance(now);
          if(!std::isfinite(from_beacon)||from_beacon<3.5)throw std::runtime_error("uncommanded capture or missing offset Loiter evidence");
          command("parity-precision-on","action=precision_loiter_on");transition(Phase::PrecisionOn,"parity_precision_on");
        } else stageRtl();
      }
    } else if(phase==Phase::PrecisionOn) {
      check("parity-precision-on");
      if(runtime["fc_mode"]!="5")throw std::runtime_error("left native Loiter during precision enable");
      if(last_status["parity-precision-on"]=="UNCONFIRMED"&&beacon_distance(now)<=1.35&&
         numbers.count("NAV_SPEED")&&numbers.at("NAV_SPEED")>=0&&numbers.at("NAV_SPEED")<=.3) {
        if(precision_since<0)precision_since=now;
        if(now-precision_since>=2.) {
          log("assertion","precision_capture_distance_m",std::to_string(beacon_distance(now)));
          command("parity-precision-off","action=precision_loiter_off");transition(Phase::PrecisionOff,"parity_precision_off");
        }
      } else precision_since=-1;
      if(now-phase_at>25)throw std::runtime_error("Precision Loiter did not acquire and dwell at the offset beacon");
    } else if(phase==Phase::PrecisionOff) {
      check("parity-precision-off");
      if(runtime["fc_mode"]!="5")throw std::runtime_error("left native Loiter during precision disable");
      if(last_status["parity-precision-off"]=="UNCONFIRMED") {
        operation("ops-precision-retreat","action=goto,strategy=guided"+point(0,0)+",timeout=25");
        transition(Phase::PrecisionRetreat,"parity_precision_retreat");
      }
    } else if(phase==Phase::PrecisionRetreat) {
      check("ops-precision-retreat");
      if(done("ops-precision-retreat"))transition(Phase::PrepareDisabledHold,"parity_precision_prepare_disabled");
    } else if(phase==Phase::PrepareDisabledHold) {
      if(pilot_ready&&std::ifstream(pilot_ready).good()) {
        operation("ops-precision-disabled-hold","action=hold,strategy=native,timeout=10");
        transition(Phase::DisabledHold,"parity_precision_disabled_hold");
      } else if(now-phase_at>6)throw std::runtime_error("scoped neutral pilot readiness unavailable after precision retreat");
    } else if(phase==Phase::DisabledHold) {
      check("ops-precision-disabled-hold");
      if(done("ops-precision-disabled-hold")&&runtime["fc_mode"]=="5")transition(Phase::DisabledDwell,"parity_precision_disabled_dwell");
    } else if(phase==Phase::DisabledDwell) {
      const double from_beacon=beacon_distance(now);
      if(runtime["fc_mode"]!="5"||!std::isfinite(from_beacon)||from_beacon<3.5)
        throw std::runtime_error("disabled Precision Loiter did not hold away from the beacon");
      if(now-phase_at>=3.)stageRtl();
    } else if(phase==Phase::StageRtl) {
      check("ops-rtl-stage");if(done("ops-rtl-stage")) {
        if(parity) {
          client.Notify("DEAD_MAN_POST_INTERRUPT",std::string("true"));log("request","DEAD_MAN_POST_INTERRUPT","true");
        } else operation(rtl_id,"action=return,strategy=native,timeout=60");
        transition(Phase::Rtl,"native_rtl");
      }
    } else if(phase==Phase::Rtl) {
      check(rtl_id);if(done(rtl_id)) {
        if(yes(runtime["in_air"]))throw std::runtime_error("native RTL completed without ground evidence");
        if(yes(runtime["armed"])) {operation("ops-disarm","action=disarm,timeout=10");transition(Phase::Disarm,"disarm");}
        else transition(Phase::Final,"final");
      }
    } else if(phase==Phase::Disarm) {
      check("ops-disarm");if(done("ops-disarm"))transition(Phase::Final,"final");
    } else if(phase==Phase::Final) {
      if(yes(runtime["armed"])||yes(runtime["in_air"]))throw std::runtime_error("final state not disarmed/on-ground");
      if(now-phase_at>=1) {log("result","workflow","PASS: native/Guided/stock-helm operations, cancellations, native RTL and final ground/disarmed");client.Close();return 0;}
    }
    if(phase>=Phase::HelmSeed&&phase<=Phase::VoronoiCancel&&now-phase_at>10)
      throw std::runtime_error("parity helm task or update deadline exceeded");
    if(now-phase_at>65)throw std::runtime_error("phase deadline exceeded");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error("complete operation workflow exceeded300s");
 }catch(const std::exception& error){log("result","workflow",std::string("FAIL: ")+error.what());client.Close();return 1;}
}
