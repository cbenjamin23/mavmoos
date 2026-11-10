// Ordinary MOOS client for the owned native restart case.
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include "../observation_load/FinishBoundary.h"
#include "../protocol_faults/ProtocolSupport.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <set>
#include <thread>

namespace {
using protocol_faults::fields;
using protocol_faults::quote;
using protocol_faults::require;
using Fields=std::map<std::string,std::string>;
constexpr char runtime_key[]="MAVMOOS_RUNTIME_STATUS";
constexpr char recovery_key[]="MAVMOOS_AUTO_RECOVERY_STATUS";
constexpr char proof_key[]="MAVMOOS_STORED_MISSION_PROOF";
constexpr char handback_key[]="MAVMOOS_AUTO_RECOVERY_REQUEST";

double mono()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
bool yes(const Fields& value,const char* key)
{
  const auto found=value.find(key);
  return found!=value.end() && (found->second=="1" || found->second=="true");
}
std::string get(const Fields& value,const char* key)
{
  const auto found=value.find(key);
  return found==value.end() ? std::string{} : found->second;
}
struct Record {Fields data;double received=-1;};

class Probe {
public:
  CMOOSCommClient client;
  std::filesystem::path control;
  bool verify_motion=false;

  void run()
  {
    wait(165,"original airborne AUTO and verified mission",[&] {
      const auto& runtime=latest[runtime_key].data;
      return recent(runtime_key) && recent(recovery_key) && healthy(runtime,"3") &&
        yes(runtime,"mission_execution_held") && get(latest[recovery_key].data,"state")=="OWNED" &&
        yes(latest[proof_key].data,"valid") && get(latest[proof_key].data,"verification_id")=="upload" &&
        outcome("run")=="ACTIVE";
    });
    const auto old_session=get(latest[recovery_key].data,"session");
    const auto old_proof=latest[proof_key].data;
    require(!old_session.empty()&&!get(old_proof,"content_id").empty(),"original process/proof identity missing");
    boundary("restart.ready",old_session,12);

    wait(18,"new bridge observing the same airborne AUTO",[&] {
      const auto& state=latest[recovery_key].data;
      const auto& runtime=latest[runtime_key].data;
      return recent(runtime_key) && recent(recovery_key) && get(state,"session")!=old_session &&
        !get(state,"session").empty() && get(state,"state")=="UNRESOLVED" &&
        get(state,"marker")=="present" && yes(state,"effects_blocked") &&
        healthy(runtime,"3") && !yes(runtime,"mission_execution_held") &&
        get(latest[proof_key].data,"valid")=="false";
    });
    const auto session=get(latest[recovery_key].data,"session");
    log("phase","restart.observed",session);
    accepted_tx.clear();
    handback("wrong_session",old_session);
    expect("wrong_session","REJECTED",3);
    require(reason("wrong_session")=="wrong_process_session","old process selector was not rejected");
    handback("in_auto",session);
    expect("in_auto","REJECTED",3);
    require(reason("in_auto")=="fresh_prior_non_auto_observation_required","AUTO handback rejection reason differs");

    send("MAVMOOS_NAV_CONTROL","acquire");
    wait(3,"helm acquisition rejected",[&] {
      return recent("MAVMOOS_HELM_STATUS") &&
        get(latest["MAVMOOS_HELM_STATUS"].data,"reason")=="acquisition_rejected";
    });
    operation("blocked_land","land");
    expect("blocked_land","REJECTED",4);
    require(reason("blocked_land")=="busy","LAND request did not encounter recovery gate");
    send("MAVMOOS_MISSION_REQUEST","id=blocked_clear,action=clear");
    expect("blocked_clear","FAILED",4);
    require(reason("blocked_clear")=="mutation_state_no_longer_permitted"&&
      get(results.at("blocked_clear"),"mutation_may_have_applied")=="false",
      "CLEAR mutation denial reason or effect evidence differs");
    send("MAVMOOS_PARAM_REQUEST","id=blocked_write,action=write,name=SIM_WIND_SPD,value=1");
    expect("blocked_write","REJECTED",4);
    require(reason("blocked_write")=="simulation_wind_write_requires_disarmed_ground"&&
      get(results.at("blocked_write"),"write_may_have_applied")=="false",
      "parameter mutation denial reason or effect evidence differs");
    send("MAVMOOS_PARAM_REQUEST","id=read_frame,action=read,name=FRAME_CLASS");
    expect("read_frame","COMPLETED",8);
    send("MAVMOOS_MISSION_REQUEST","id=read_mission,action=download,timeout=12");
    expect("read_mission","COMPLETED",13);
    require(recent(runtime_key)&&healthy(latest[runtime_key].data,"3")&&
      get(latest[recovery_key].data,"state")=="UNRESOLVED","airborne AUTO or fence lost before takeover");
    boundary("takeover.request",session,5);

    wait(10,"fresh non-AUTO evidence while still blocked",[&] {
      const auto& state=latest[recovery_key].data;
      return recent(runtime_key)&&recent(recovery_key)&&healthy(latest[runtime_key].data,"4") &&
        get(state,"session")==session && get(state,"state")=="UNRESOLVED" &&
        yes(state,"ready_for_handback") && yes(state,"effects_blocked");
    });
    operation("before_handback","guided");
    expect("before_handback","REJECTED",4);
    require(reason("before_handback")=="busy","Guided request did not encounter recovery gate");
    send("MAVMOOS_MISSION_CONTROL","schema=1,id=old_proof,action=start,verification_id=upload,content_id="+
      old_proof.at("content_id")+",proof_revision="+old_proof.at("revision")+",timeout=5");
    expect("old_proof","REJECTED",4);
    require(reason("old_proof")=="current_verified_guided_entry_required",
      "old proof rejection reason differs");
    boundary("handback.ready",session,5);

    bool released=false;
    for(unsigned attempt=0;attempt<3&&!released;++attempt) {
      wait(2,"fresh handback candidate",[&] {
        return recent(recovery_key)&&yes(latest[recovery_key].data,"ready_for_handback");
      });
      const auto id="handback"+std::to_string(attempt);
      handback(id,session);
      wait(3,"handback response",[&]{return !outcome(id).empty();});
      released=outcome(id)=="RELEASED";
      require(released||reason(id)=="fresh_prior_non_auto_observation_required","unexpected handback failure");
    }
    require(released,"explicit handback not accepted");
    const auto readyToAcquire=[&] {
      const auto& runtime=latest[runtime_key].data;
      return recent(runtime_key)&&healthy(runtime,"4")&&yes(runtime,"position_valid")&&
        yes(runtime,"home_valid")&&yes(runtime,"estimator_valid");
    };
    wait(8,"fresh ordinary context after handback",[&] {
      return readyToAcquire()&&recent(recovery_key)&&get(latest[recovery_key].data,"state")=="READY" &&
        get(latest[proof_key].data,"valid")=="false";
    });
    const auto previous_nav=get(latest[runtime_key].data,"nav_tx_accepted");
    const auto until=mono()+1;
    while(mono()<until) {
      pump();
      const auto& runtime=latest[runtime_key].data;
      require(!yes(runtime,"permission")&&get(runtime,"nav_tx_accepted")==previous_nav,
        "old acquisition or Guided output resumed after handback");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    wait(3,"fresh flight context for new acquisition",readyToAcquire);
    require(accepted_tx==std::set<std::pair<std::string,std::string>>{
        {"MAVMOOS_MISSION_TX_RESULT","read_mission"},{"MAVMOOS_PARAM_TX_RESULT","read_frame/initial"}},
      "accepted handoffs during recovery differed from the two read-only transactions");
    log("phase","handback.completed",session);
    if(verify_motion) {
      operation("post_recovery_loiter","hold");
      expect("post_recovery_loiter","COMPLETED",6);
      wait(3,"fresh native Loiter before new acquisition",[&] {
        return recent(runtime_key)&&healthy(latest[runtime_key].data,"5");
      });
    }
    send("MAVMOOS_NAV_CONTROL","acquire");
    wait(5,"new explicit navigation grant",[&] {
      return recent(runtime_key)&&yes(latest[runtime_key].data,"permission");
    });
    log("phase","new_acquisition",session);
    if(verify_motion) {
      send("MAVMOOS_RESTART_GUIDANCE","true");
      wait(8,"new stock helm target after recovery",[&] {
        const auto& runtime=latest[runtime_key].data;
        return recent(runtime_key)&&yes(runtime,"permission")&&
          get(runtime,"nav_tx_accepted")!=previous_nav&&recent("MAVMOOS_HELM_STATUS")&&
          get(latest["MAVMOOS_HELM_STATUS"].data,"state")=="ACTIVE";
      });
      boundary("guidance.active",session,12);
    }
    send("MAVMOOS_NAV_CONTROL","release");
    wait(3,"explicit navigation release",[&] {
      return recent(runtime_key)&&!yes(latest[runtime_key].data,"permission");
    });
    if(verify_motion)send("MAVMOOS_RESTART_GUIDANCE","false");
    operation("land_after","land",60);
    expect("land_after","COMPLETED",65);
    wait(6,"ground and disarmed",[&] {
      const auto& runtime=latest[runtime_key].data;
      return recent(runtime_key)&&yes(runtime,"heartbeat_valid")&&yes(runtime,"landed_valid")&&
        !yes(runtime,"armed")&&!yes(runtime,"in_air")&&!yes(runtime,"permission");
    });
    log("pass","auto_restart_recovery",session);
  }

  void log(const char* kind,const std::string& key,const std::string& value,
           std::optional<double> published=std::nullopt,const char* source="iMAVLink")
  {
    require(++records<=100000,"probe record bound exceeded");
    std::cout<<std::setprecision(17)<<"{\"elapsed\":"<<mono()-started
      <<",\"kind\":"<<quote(kind)<<",\"key\":"<<quote(key)<<",\"value\":"<<quote(value);
    if(published) std::cout<<",\"source\":"<<quote(source)<<",\"published_at\":"<<*published;
    std::cout<<"}\n";
    std::cout.flush();
  }

private:
  double started=mono();
  unsigned records=0;
  bool registered=false;
  std::map<std::string,Record> latest;
  std::map<std::string,Fields> results;
  std::set<std::pair<std::string,std::string>> accepted_tx;

  void pump()
  {
    if(!client.IsConnected()) {
      require(!registered,"MOOSDB connection lost across restart");
      return;
    }
    if(!registered) {
      for(const char* key:{runtime_key,recovery_key,proof_key,"MAVMOOS_HELM_STATUS",
        "MAVMOOS_MISSION_EXECUTION_RESULT","MAVMOOS_CONTROL_RESULT","MAVMOOS_MISSION_RESULT",
        "MAVMOOS_PARAM_RESULT","MAVMOOS_AUTO_RECOVERY_RESULT","MAVMOOS_CAPABILITIES",
        "MAVMOOS_TX_RESULT","MAVMOOS_MISSION_TX_RESULT","MAVMOOS_PARAM_TX_RESULT"})
        client.Register(key,0);
      if(verify_motion)
        for(const char* key:{"MAVMOOS_PARAM_VALUE","DESIRED_HEADING","DESIRED_SPEED","DESIRED_ALTITUDE"})
          client.Register(key,0);
      registered=true;
    }
    MOOSMSG_LIST mail;
    client.Fetch(mail);
    for(const auto& message:mail) {
      const auto key=message.GetKey();
      if(verify_motion&&message.GetSource()=="pHelmIvP"&&message.IsDouble()&&key.rfind("DESIRED_",0)==0) {
        log("helm",key,std::to_string(message.GetDouble()),message.GetTime(),"pHelmIvP");
        continue;
      }
      if(message.GetSource()!="iMAVLink" || !message.IsString()) continue;
      const auto text=message.GetString();
      require(std::isfinite(message.GetTime()),"nonfinite MOOS publication time");
      log("mail",key,text,message.GetTime());
      if(key=="MAVMOOS_CAPABILITIES") continue;
      auto parsed=fields(text);
      if(key=="MAVMOOS_TX_RESULT"||key=="MAVMOOS_MISSION_TX_RESULT"||key=="MAVMOOS_PARAM_TX_RESULT") {
        if(get(parsed,"status")=="LOCAL_ACCEPTED")
          accepted_tx.emplace(key,get(parsed,key=="MAVMOOS_MISSION_TX_RESULT"?"operation_id":"id"));
        continue;
      }
      latest[key]={parsed,mono()};
      const auto id=get(parsed,"id");
      if(!id.empty() && parsed.count("status")) results[id]=std::move(parsed);
    }
  }
  bool recent(const std::string& key)const
  {
    const auto found=latest.find(key);
    return found!=latest.end() && mono()-found->second.received<1.3;
  }
  static bool healthy(const Fields& runtime,const char* mode)
  {
    return yes(runtime,"heartbeat_valid")&&yes(runtime,"landed_valid")&&yes(runtime,"armed")&&
      yes(runtime,"in_air")&&get(runtime,"fc_mode")==mode&&!yes(runtime,"permission");
  }
  void wait(double timeout,const std::string& context,const std::function<bool()>& ready)
  {
    const auto end=mono()+timeout;
    while(mono()<end) {
      pump();
      if(mono()<end && ready()) return;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    throw std::runtime_error("timeout waiting for "+context);
  }
  void send(const std::string& key,const std::string& value)
  {
    require(client.Notify(key,value),"MOOS request publication failed");
    log("request",key,value);
  }
  std::string outcome(const std::string& id)const
  {const auto it=results.find(id);return it==results.end()?"":get(it->second,"status");}
  std::string reason(const std::string& id)const
  {const auto it=results.find(id);return it==results.end()?"":get(it->second,"reason");}
  void expect(const std::string& id,const char* expected,double seconds)
  {
    wait(seconds,id,[&]{
      const auto state=outcome(id);
      if(state==expected) return true;
      if(state=="COMPLETED"||state=="RELEASED"||state=="REJECTED"||state=="FAILED"||
         state=="TIMED_OUT"||state=="INCOMPLETE"||state=="LOST"||state=="CANCELLED")
        throw std::runtime_error(id+" returned "+state+" "+reason(id));
      return false;
    });
  }
  void handback(const std::string& id,const std::string& session)
  {send(handback_key,"schema=1,id="+id+",action=handback,session="+session);}
  void operation(const std::string& id,const std::string& action,int timeout=5)
  {send("MAVMOOS_CONTROL_REQUEST","schema=1,id="+id+",action="+action+",timeout="+std::to_string(timeout));}
  void boundary(const std::string& name,const std::string& session,double timeout)
  {
    const auto token=name+":"+session+"\n";
    const auto path=control/name,temporary=control/(name+".tmp");
    require(!std::filesystem::exists(path)&&!std::filesystem::exists(control/(name+".ack")),"existing boundary file");
    {std::ofstream stream(temporary);require(bool(stream<<token),"boundary publication failed");}
    std::filesystem::rename(temporary,path);
    log("boundary",name,token);
    const auto reply=mavmoos::test::waitForFinishAck(token,mono()+timeout,[&]() -> std::optional<std::string> {
      std::ifstream stream(control/(name+".ack"));
      if(!stream) return std::nullopt;
      std::string text;
      char c;
      while(text.size()<=128&&stream.get(c))text+=c;
      require(!stream.bad()&&text.size()<=128,"invalid boundary acknowledgement");
      return text;
    },[&]{pump();},mono,[](double seconds){std::this_thread::sleep_for(std::chrono::duration<double>(seconds));});
    require(reply.has_value(),"boundary acknowledgement expired");
    log("boundary_ack",name,token);
  }
};
}

int main(int argc,char** argv)
{
  Probe probe;
  try {
    require(argc==3 || (argc==4 && std::string(argv[3])=="--verify-motion"),
      "usage: auto_recovery_probe MOOS_PORT CONTROL_DIRECTORY [--verify-motion]");
    probe.verify_motion=argc==4;
    const int port=std::stoi(argv[1]);
    require(port>0&&port<=65535,"invalid MOOS port");
    probe.control=argv[2];
    probe.client.SetQuiet(true);
    require(probe.client.Run("127.0.0.1",port,"auto_recovery_probe",50),"MOOS client startup failed");
    probe.run();
    probe.client.Close();
    return 0;
  } catch(const std::exception& error) {
    probe.log("fail","auto_restart_recovery",error.what());
    probe.client.Close();
    return 1;
  }
}
