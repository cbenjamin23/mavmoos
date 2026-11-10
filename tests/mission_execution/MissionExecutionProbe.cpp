// Test-only ordinary MOOS client. Every effect uses an explicit public request.
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
#include <optional>
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
  double start=mono(),runtime_at=-1,nav_at=-1,speed_at=-1,speed=0,home_lat=0,home_lon=0,home_msl=0;
  bool registered=false,have_home=false,verified_snapshot=false;
  std::map<std::string,std::string> runtime,proof,execution;
  std::map<std::string,std::map<std::string,std::string>> latest;
  std::map<std::string,std::string> mission_progress;
  bool verified_typed_snapshot=false;
  std::map<std::string,std::set<std::string>> states;
  std::map<std::string,double> expires;
  std::map<std::string,double> publication_attempts,local_deadlines,navigation,navigation_at;
  unsigned records=0,boundary_counter=0;

  void log(const std::string& type,const std::string& key,const std::string& value,const std::string& source="probe")
  {if(++records>120000)throw std::runtime_error("probe record bound exceeded");std::cout<<std::setprecision(17)<<"{\"elapsed\":"<<mono()-start<<",\"type\":"<<quote(type)<<",\"key\":"<<quote(key)<<",\"source\":"<<quote(source)<<",\"value\":"<<quote(value)<<"}\n";std::cout.flush();}
  void notify(const std::string& key,const std::string& value)
  {
    const bool timed=key=="MAVMOOS_CONTROL_REQUEST"||key=="MAVMOOS_MISSION_CONTROL"||key=="MAVMOOS_MISSION_REQUEST";
    const auto attempted=mono();std::string id;
    if(timed){const auto data=fields(value);id=data.at("id");const auto timeout=std::stod(data.at("timeout"));
      if(!std::isfinite(timeout)||timeout<=0||publication_attempts.count(id))throw std::runtime_error("invalid/repeated local request deadline");
      publication_attempts[id]=attempted;local_deadlines[id]=attempted+timeout;}
    if(!client.Notify(key,value))throw std::runtime_error("MOOS Notify failed");
    if(!timed){log("request",key,value);return;}
    if(++records>120000)throw std::runtime_error("probe record bound exceeded");
    std::cout<<std::setprecision(17)<<"{\"elapsed\":"<<mono()-start<<",\"type\":\"request\",\"key\":"<<quote(key)<<",\"value\":"<<quote(value)
      <<",\"source\":\"probe\",\"local_publication_attempt_elapsed\":"<<attempted-start
      <<",\"local_original_expiry_elapsed\":"<<local_deadlines.at(id)-start<<",\"local_clock_scope\":\"probe_publication_attempt_not_host_expiry\"}\n";std::cout.flush();
  }
  std::string marker(const std::string& key)
  {
    std::ostringstream value;
    value<<"phase:"<<key<<':'<<++boundary_counter<<':'<<std::setprecision(17)<<mono()<<'\n';
    const auto token=value.str();
    const auto temporary=control/(key+".tmp");
    if(std::filesystem::exists(control/key)||std::filesystem::exists(control/(key+".ack")))
      throw std::runtime_error("preexisting phase marker/ack");
    {
      std::ofstream stream(temporary);
      if(!(stream<<token)) throw std::runtime_error("marker write failed");
    }
    // A supervisor can retire this probe as soon as the marker is visible.
    log("phase",key,"nav_tx_accepted="+runtime["nav_tx_accepted"]);
    std::filesystem::rename(temporary,control/key);
    return token;
  }
  bool ready()
  {return client.IsConnected()&&runtime_at>0&&mono()-runtime_at<.75&&yes(runtime.at("heartbeat_valid"))&&yes(runtime.at("position_valid"))&&yes(runtime.at("home_valid"))&&yes(runtime.at("landed_valid"))&&yes(runtime.at("estimator_valid"))&&nav_at>0&&mono()-nav_at<.5;}
  bool autoOwned()
  {return ready()&&yes(runtime.at("armed"))&&yes(runtime.at("in_air"))&&runtime.at("fc_mode")=="3"&&!yes(runtime.at("permission"))&&execution.count("lease_retained")&&yes(execution.at("lease_retained"));}
  static bool failed(const std::string& status)
  {return status=="FAILED"||status=="REJECTED"||status=="TIMED_OUT"||status=="INCOMPLETE"||status=="LOST"||status=="CANCELLED";}
  void check(const std::string& id)
  {if(latest.count(id)&&failed(latest.at(id).at("status")))throw std::runtime_error(id+":"+latest[id]["status"]+":"+latest[id]["reason"]);}
  void pump()
  {
    if(!client.IsConnected()){if(registered)throw std::runtime_error("MOOS disconnected during scenario");return;}
    if(!registered){for(const char* key:{"MAVMOOS_CAPABILITIES","MAVMOOS_RUNTIME_STATUS","MAVMOOS_CONTROL_RESULT","MAVMOOS_CONTROL_STATUS","MAVMOOS_HELM_STATUS","MAVMOOS_FIRMWARE_STATUS","MAVMOOS_HOME_POSITION","MAVMOOS_STORED_MISSION_PROOF","MAVMOOS_MISSION_RESULT","MAVMOOS_MISSION_SNAPSHOT","MAVMOOS_MISSION_TX_RESULT","MAVMOOS_MISSION_EXECUTION_RESULT","MAVMOOS_MISSION_EXECUTION_STATUS","MAVMOOS_MISSION_EXECUTION_PROGRESS","NAV_X","NAV_Y","NAV_LAT","NAV_LONG","NAV_SPEED","NAV_ALTITUDE","DESIRED_HEADING","DESIRED_SPEED","DESIRED_ALTITUDE"})client.Register(key,0);registered=true;}
    MOOSMSG_LIST mail;client.Fetch(mail);
    for(const auto& message:mail){const auto key=message.GetKey(),source=message.GetSource();
      if(source=="pHelmIvP"&&message.IsDouble()&&key.rfind("DESIRED_",0)==0){log("helm_scalar",key,std::to_string(message.GetDouble()),source);continue;}
      if(source!="iMAVLink")continue;
      if(message.IsDouble()&&key.rfind("NAV_",0)==0){nav_at=mono();navigation[key]=message.GetDouble();navigation_at[key]=nav_at;if(key=="NAV_SPEED"){speed=message.GetDouble();speed_at=mono();}log("navigation",key,std::to_string(message.GetDouble()),source);continue;}
      if(!message.IsString())continue;const auto text=message.GetString();
      if(key=="MAVMOOS_RUNTIME_STATUS"){runtime=fields(text);runtime_at=mono();log("runtime",key,text,source);}
      else if(key=="MAVMOOS_CONTROL_RESULT"||key=="MAVMOOS_MISSION_RESULT"||key=="MAVMOOS_MISSION_EXECUTION_RESULT"){
        const auto value=fields(text);const auto& id=value.at("id");const auto at=std::stod(value.at("expires"));if(!std::isfinite(at)||at<=0)throw std::runtime_error("invalid result original expiry");if(expires.count(id)&&expires[id]!=at)throw std::runtime_error("original request expiry changed");expires[id]=at;states[id].insert(value.at("status"));latest[id]=value;
        log(key=="MAVMOOS_CONTROL_RESULT"?"control_result":key=="MAVMOOS_MISSION_RESULT"?"mission_result":"execution_result",key,text,source);
      } else if(key=="MAVMOOS_STORED_MISSION_PROOF"){proof=fields(text);log("proof",key,text,source);}
      else if(key=="MAVMOOS_MISSION_EXECUTION_STATUS"){execution=fields(text);log("execution_status",key,text,source);}
      else if(key=="MAVMOOS_MISSION_EXECUTION_PROGRESS"){mission_progress=fields(text);log("execution_progress",key,text,source);}
      else if(key=="MAVMOOS_HOME_POSITION"){const auto data=fields(text);if(yes(data.at("encoding_valid"))){home_lat=std::stod(data.at("lat_deg"));home_lon=std::stod(data.at("lon_deg"));home_msl=std::stod(data.at("alt_msl_m"));have_home=std::isfinite(home_lat)&&std::isfinite(home_lon)&&std::isfinite(home_msl);}log("home",key,text,source);}
      else if(key=="MAVMOOS_MISSION_SNAPSHOT"){
        if(text.find("\"id\":\"upload\"")!=std::string::npos&&text.find("\"verification\":\"semantic_profile_match\"")!=std::string::npos){
          verified_snapshot|=text.find("\"count\":3")!=std::string::npos;
          verified_typed_snapshot|=text.find("\"count\":5")!=std::string::npos;
        }
        log("snapshot",key,text,source);
      }
      else log(key=="MAVMOOS_CAPABILITIES"?"catalog":key=="MAVMOOS_FIRMWARE_STATUS"?"firmware":"state",key,text,source);
    }
  }
  void wait(const std::function<bool()>& predicate,double limit,const std::string& reason)
  {const auto end=mono()+limit;while(mono()<end){pump();if(mono()>=end)break;if(predicate())return;std::this_thread::sleep_for(std::chrono::milliseconds(10));}throw std::runtime_error(reason);}
  void command(const std::string& id,const std::string& body,double limit=12)
  {notify("MAVMOOS_CONTROL_REQUEST","schema=1,id="+id+","+body);wait([&]{check(id);return states[id].count("COMPLETED");},limit,"control completion timeout:"+id);}
  void missionControl(const std::string& id,const std::string& action)
  {notify("MAVMOOS_MISSION_CONTROL","schema=1,id="+id+",action="+action+",target=run,timeout=8");}
  void observe(double duration,const std::function<bool()>& condition,const std::string& why)
  {const auto end=mono()+duration;while(mono()<end){pump();if(!condition())throw std::runtime_error(why);std::this_thread::sleep_for(std::chrono::milliseconds(10));}}
  double phaseDeadline(const std::string& owner,double began)const
  {
    const auto original=local_deadlines.at(owner);
    if(!std::isfinite(began)||began<publication_attempts.at(owner)||began>=original)throw std::runtime_error("original local request deadline expired");
    return std::min(began+3,original);
  }
  double phaseAck(const std::string& key,const std::string& token,const std::string& owner,double began,double deadline,const std::function<void()>& maintained)
  {
    const auto at=mavmoos::test::waitForFinishAck(token,deadline,[&]()->std::optional<std::string>{
      std::ifstream input(control/(key+".ack"));if(!input)return std::nullopt;std::string text;char c;while(text.size()<=128&&input.get(c))text+=c;
      if(input.bad()||text.size()>128)throw std::runtime_error("phase acknowledgement unreadable/oversized");return text;
    },[&]{pump();if(mono()>=local_deadlines.at(owner))throw std::runtime_error("original local request deadline expired");maintained();},mono,[](double seconds){std::this_thread::sleep_for(std::chrono::duration<double>(seconds));});
    if(!at)throw std::runtime_error("phase acknowledgement missed original bounded deadline");
    if(++records>120000)throw std::runtime_error("probe record bound exceeded");
    std::cout<<std::setprecision(17)<<"{\"elapsed\":"<<mono()-start<<",\"type\":\"phase_ack\",\"key\":"<<quote(key)<<",\"value\":"<<quote(token)
      <<",\"source\":\"probe\",\"original_request_id\":"<<quote(owner)<<",\"local_publication_attempt_elapsed\":"<<publication_attempts.at(owner)-start
      <<",\"local_original_expiry_elapsed\":"<<local_deadlines.at(owner)-start<<",\"wait_started_elapsed\":"<<began-start
      <<",\"deadline_elapsed\":"<<deadline-start<<",\"ack_observed_elapsed\":"<<*at-start<<"}\n";std::cout.flush();return *at;
  }
  double acknowledgedMarker(const std::string& key,const std::string& owner,const std::function<void()>& maintained)
  {const auto began=mono(),deadline=phaseDeadline(owner,began);const auto token=marker(key);return phaseAck(key,token,owner,began,deadline,maintained);}
  void observeWithin(double duration,const std::string& owner,const std::function<bool()>& condition,const std::string& why)
  {
    const auto end=mono()+duration,original=local_deadlines.at(owner);
    if(end>=original)throw std::runtime_error("insufficient original request lifetime for observation");
    while(mono()<end){pump();if(mono()>=original)throw std::runtime_error("original local request deadline expired");if(!condition())throw std::runtime_error(why);std::this_thread::sleep_for(std::chrono::milliseconds(10));}
    if(mono()>=original)throw std::runtime_error("original local request deadline expired");
  }
  void waitWithin(double limit,const std::string& owner,const std::function<bool()>& condition,const std::string& why)
  {
    const auto deadline=std::min(mono()+limit,local_deadlines.at(owner));
    while(mono()<deadline){pump();if(mono()>=deadline)break;if(condition()){if(mono()>=deadline)break;return;}std::this_thread::sleep_for(std::chrono::milliseconds(10));}
    throw std::runtime_error(why);
  }
  bool returnSampleRegion(double latitude,double longitude,double now)
  {
    if(!have_home||!std::isfinite(now)||std::abs(home_lat-latitude)>1e-7||std::abs(home_lon-longitude)>1e-7)return false;
    for(const char* key:{"NAV_LAT","NAV_LONG","NAV_ALTITUDE"})if(!navigation.count(key)||!std::isfinite(navigation[key])||!std::isfinite(navigation_at[key])||navigation_at[key]>now||now-navigation_at[key]>.5)return false;
    constexpr double radians=3.14159265358979323846/180.;
    const auto distance=6371000.*std::hypot((navigation["NAV_LAT"]-latitude)*radians,(navigation["NAV_LONG"]-longitude)*radians*std::cos(latitude*radians));
    return distance<=2.5&&std::abs(navigation["NAV_ALTITUDE"]-5)<=.6;
  }
  bool returned(double latitude,double longitude)
  {return ready()&&yes(runtime.at("armed"))&&yes(runtime.at("in_air"))&&runtime.at("fc_mode")=="4"&&!yes(runtime.at("permission"))&&execution["lease_retained"]=="false"&&returnSampleRegion(latitude,longitude,mono());}
  void finishAutoWindow()
  {
    const auto began=mono();const auto at=acknowledgedMarker("auto.finished","run",[&]{check("run");if(!autoOwned())throw std::runtime_error("AUTO ownership lost before coordinator boundary");});
    log("assertion","auto_finish_ack","wait_s="+std::to_string(at-began)+",timeout_s=3");
  }
  void runTyped()
  {
    marker("upload.started");
    constexpr double metres_per_degree=111195.;
    const auto east=1/(metres_per_degree*std::cos(home_lat*3.14159265358979323846/180.));
    const auto north=1/metres_per_degree;
    std::ostringstream rows;
    rows<<"id=upload,action=upload,timeout=30,items=";
    auto row=[&](unsigned seq,unsigned cmd,unsigned dwell,double lat,double lon,double altitude){
      if(seq)rows<<'|';
      rows<<seq<<':'<<cmd<<':'<<(seq?3:0)<<":0:1:"<<dwell<<":0:0:0:"
          <<std::llround(lat*1e7)<<':'<<std::llround(lon*1e7)<<':'<<std::setprecision(9)<<altitude;
    };
    row(0,16,0,home_lat,home_lon,home_msl);
    row(1,22,0,home_lat,home_lon,5);
    row(2,16,3,home_lat+14*north,home_lon,5);
    row(3,16,0,home_lat+22*north,home_lon+8*east,5);
    row(4,21,0,home_lat,home_lon+6*east,0);
    notify("MAVMOOS_MISSION_REQUEST",rows.str());
    wait([&]{check("upload");return states["upload"].count("COMPLETED")&&verified_typed_snapshot&&
      proof["valid"]=="true"&&proof["verification_id"]=="upload"&&proof["count"]=="5";},
      33,"typed TAKEOFF/WAYPOINT/LAND readback proof missing");
    marker("upload.finished");
    const auto revision=proof.at("revision"),content=proof.at("content_id");
    command("guided","action=guided,timeout=10");command("arm","action=arm,timeout=10");
    wait([&]{return ready()&&yes(runtime.at("armed"))&&!yes(runtime.at("in_air"))&&runtime.at("fc_mode")=="4";},
      3,"explicit disarmed-to-armed ground Guided state missing");
    const auto nav=runtime.at("nav_tx_accepted");
    notify("MOOS_MANUAL_OVERRIDE","false");
    marker("takeoff.started");
    notify("MAVMOOS_MISSION_CONTROL","schema=1,id=run,action=start,verification_id=upload,content_id="+content+",proof_revision="+revision+",timeout=110");
    wait([&]{check("run");return states["run"].count("ACTIVE")&&ready()&&runtime.at("fc_mode")=="3";},
      14,"verified grounded AUTO start missing");
    marker("auto.started");
    wait([&]{check("run");return autoOwned()&&navigation["NAV_ALTITUDE"]>=4.4&&mission_progress["seq"]=="2";},
      25,"native stored takeoff did not climb and reach first transit cursor");
    marker("takeoff.finished");
    missionControl("pause","pause");
    wait([&]{check("run");check("pause");return states["pause"].count("COMPLETED")&&autoOwned()&&
      mission_progress["mission_state"]=="4";},10,"observed mission pause missing");
    double settled=-1;
    wait([&]{check("run");if(!autoOwned()||speed_at<=0||mono()-speed_at>.5||speed>.35)settled=-1;
      else if(settled<0)settled=mono();return settled>0&&mono()-settled>=1;},8,"paused Copter did not settle");
    const auto paused=[&]{return autoOwned()&&mission_progress["mission_state"]=="4"&&
      runtime.at("nav_tx_accepted")==nav&&mono()-speed_at<=.5&&speed<=.35;};
    const auto maintain_pause=[&]{if(!paused())throw std::runtime_error("paused AUTO lost hold or leaked helm");};
    acknowledgedMarker("auto.paused","run",maintain_pause);
    observeWithin(1.6,"run",paused,"paused AUTO lost hold or leaked helm");
    acknowledgedMarker("resume.requested","run",maintain_pause);
    missionControl("resume","resume");
    wait([&]{check("run");check("resume");return states["resume"].count("COMPLETED")&&autoOwned()&&
      latest["resume"]["reason"]=="observed_active_same_cursor";},10,"same-cursor explicit resume missing");
    marker("auto.resumed");
    waitWithin(77,"run",[&]{check("run");return states["run"].count("COMPLETED")&&ready()&&
      !yes(runtime.at("armed"))&&!yes(runtime.at("in_air"))&&runtime.at("fc_mode")=="3"&&
      execution["lease_retained"]=="true";},"native stored LAND did not touch down and disarm with retained lease");
    if(runtime.at("nav_tx_accepted")!=nav||!std::filesystem::exists(control.parent_path()/"auto-recovery"/"auto-may-continue"))
      throw std::runtime_error("AUTO leaked helm egress or retired its durable lease before handback");
    marker("land.finished");
    marker("abort.started");missionControl("abort","abort");
    wait([&]{check("abort");return states["abort"].count("COMPLETED")&&ready()&&runtime.at("fc_mode")=="4"&&
      !yes(runtime.at("armed"))&&!yes(runtime.at("permission"))&&execution["lease_retained"]=="false";},
      11,"observed ground Guided handback missing");
    marker("abort.finished");
    if(std::filesystem::exists(control.parent_path()/"auto-recovery"/"auto-may-continue"))
      throw std::runtime_error("explicit ground handback did not retire marker");
    observe(2.3,[&]{return ready()&&!yes(runtime.at("armed"))&&!yes(runtime.at("in_air"))&&
      !yes(runtime.at("permission"))&&runtime.at("nav_tx_accepted")==nav;},"post-handback ground invariants changed");
    marker("finished");
    log("pass","auto_typed_lifecycle","bound stored TAKEOFF, waypoint dwell, pause/resume, LAND/disarm and explicit handback");
  }
  void run(const std::string& scenario)
  {
    wait([&]{return ready()&&have_home&&!yes(runtime["armed"])&&!yes(runtime["in_air"])&&!yes(runtime["permission"]);},55,"fresh disarmed/on-ground readiness timeout");
    if(scenario=="auto_typed_lifecycle"){runTyped();return;}
    marker("upload.started");
    std::ostringstream rows;rows<<"id=upload,action=upload,timeout=30,items=";
    const double factor=111195.,latitude=home_lat+25/factor,longitude=home_lon+10/(factor*std::cos(home_lat*3.14159265358979323846/180.));
    auto row=[&](unsigned n,double lat,double lon,double altitude){if(n)rows<<'|';rows<<n<<":16:"<<(n?3:0)<<":0:1:0:0:0:0:"<<std::llround(lat*1e7)<<':'<<std::llround(lon*1e7)<<':'<<std::setprecision(9)<<altitude;};
    row(0,home_lat,home_lon,home_msl);row(1,latitude,home_lon,5);row(2,latitude,longitude,5);
    notify("MAVMOOS_MISSION_REQUEST",rows.str());
    wait([&]{check("upload");return states["upload"].count("COMPLETED")&&verified_snapshot&&proof["valid"]=="true"&&proof["verification_id"]=="upload"&&proof["count"]=="3";},33,"verified upload/typed proof publication timeout");marker("upload.finished");
    const auto revision=proof.at("revision"),content=proof.at("content_id");
    command("guided","action=guided,timeout=10");command("arm","action=arm,timeout=10");
    marker("takeoff.started");command("takeoff","action=takeoff,height=5,timeout=45",48);marker("takeoff.finished");
    notify("MOOS_MANUAL_OVERRIDE","false");
    notify("MAVMOOS_MISSION_CONTROL","schema=1,id=run,action=start,verification_id=upload,content_id="+content+",proof_revision="+revision+",timeout=90");
    wait([&]{check("run");return states["run"].count("ACTIVE")&&autoOwned();},13,"observed AUTO activation timeout");marker("auto.started");
    const auto nav=runtime["nav_tx_accepted"];
    if(scenario=="auto_pause_abort"){
      missionControl("pause","pause");wait([&]{check("run");check("pause");return states["pause"].count("COMPLETED")&&states["run"].count("PAUSED")&&autoOwned();},10,"observed AUTO pause timeout");
      double settled=-1;
      wait([&]{check("run");if(!autoOwned())throw std::runtime_error("AUTO lost during pause braking");const auto now=mono();if(!std::isfinite(speed)||speed<0||speed_at<=0||now-speed_at>.5||speed>.3)settled=-1;else if(settled<0)settled=now;return settled>0&&now-settled>=1;},8,"bounded pause braking/settling timeout");
      log("assertion","pause_settled","speed_mps="+std::to_string(speed)+",settled_s=1,timeout_s=8");
      const auto maintained=[&]{check("run");if(!autoOwned()||execution["phase"]!="PAUSED"||runtime["nav_tx_accepted"]!=nav)throw std::runtime_error("paused AUTO/quiet Guided stream not retained");};
      acknowledgedMarker("auto.paused","run",maintained);
      observeWithin(3,"run",[&]{maintained();return true;},"paused AUTO/quiet Guided stream not retained");
    } else {
      wait([&]{check("run");return states["run"].count("COMPLETED")&&autoOwned();},80,"observed AUTO final reached/settled completion timeout");
      const auto maintained=[&]{check("run");if(!autoOwned()||execution["phase"]!="COMPLETED"||runtime["nav_tx_accepted"]!=nav)throw std::runtime_error("completed AUTO lease/quiet Guided stream not retained");};
      acknowledgedMarker("auto.completed","run",maintained);
      observeWithin(3,"run",[&]{maintained();return true;},"completed AUTO lease/quiet Guided stream not retained");
    }
    finishAutoWindow();marker("abort.started");missionControl("abort","abort");
    wait([&]{check("abort");return states["abort"].count("COMPLETED")&&ready()&&runtime["fc_mode"]=="4"&&!yes(runtime["permission"])&&execution["lease_retained"]=="false";},11,"observed Guided handback/lease release timeout");marker("abort.finished");
    marker("return.started");const auto return_latitude=home_lat,return_longitude=home_lon;
    command("return","action=return,strategy=helm,height=5,speed=1.5,radius=1.5,timeout=45",48);
    waitWithin(2,"return",[&]{check("return");return returned(return_latitude,return_longitude);},"fresh Guided returned observation not ready");
    const auto maintained_return=[&]{check("return");if(!returned(return_latitude,return_longitude))throw std::runtime_error("returned Guided observation lost freshness/home/height/ownership");};
    acknowledgedMarker("return.arrived","return",maintained_return);
    observeWithin(3,"return",[&]{maintained_return();return true;},"returned Guided observation lost freshness/home/height/ownership");
    acknowledgedMarker("return.finished","return",maintained_return);
    marker("land.started");command("land","action=land,timeout=60",65);if(yes(runtime["armed"]))command("disarm","action=disarm,timeout=10");
    wait([&]{return ready()&&!yes(runtime["armed"])&&!yes(runtime["in_air"])&&!yes(runtime["permission"]);},5,"final disarmed/on-ground timeout");marker("land.finished");
    observe(2,[&]{return ready()&&!yes(runtime["armed"])&&!yes(runtime["in_air"])&&!yes(runtime["permission"]);},"final ground/disarmed state changed");marker("finished");
    log("pass",scenario,"explicit verified upload; separate Guided/ARM/TAKEOFF; observed AUTO outcome; explicit abort; stock helm return; LAND/disarm");
  }
};
}
int main(int argc,char** argv)
{
  Probe probe;std::string scenario="unknown";
  try{
    if(argc!=4)throw std::runtime_error("usage: mission_execution_probe MOOS_PORT CONTROL_DIRECTORY SCENARIO");scenario=argv[3];if(scenario!="auto_complete_handback"&&scenario!="auto_pause_abort"&&scenario!="auto_typed_lifecycle")throw std::runtime_error("unsupported scenario");
    const int port=std::stoi(argv[1]);if(port<1||port>65535)throw std::runtime_error("invalid MOOS port");probe.control=argv[2];std::filesystem::create_directories(probe.control);probe.client.SetQuiet(true);
    if(!probe.client.Run("127.0.0.1",port,"mavmoos_mission_execution_probe",50))throw std::runtime_error("MOOS client startup failed");probe.run(scenario);probe.client.Close();return 0;
  }catch(const std::exception& error){probe.log("fail",scenario,error.what());probe.client.Close();return 1;}
}
