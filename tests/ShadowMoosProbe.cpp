#include "Fixtures.h"
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <set>
#include <thread>

int main(int argc,char** argv)
{
  using namespace fixture;
  try {
    check(argc==4,"args: DB port / bridge UDP port / owned peer UDP port");
    const int db=std::stoi(argv[1]);
    const auto bridge=static_cast<uint16_t>(std::stoi(argv[2]));
    const auto peer_port=static_cast<uint16_t>(std::stoi(argv[3]));
    Socket peer(peer_port);
    Encoder encoder;
    CMOOSCommClient client;
    client.SetQuiet(true);
    check(client.Run("127.0.0.1",db,"mavmoos_shadow_fixture",30),"MOOS client start");
    std::map<std::string,std::string> records;
    std::set<std::string> helm_outputs;
    std::map<std::string,double> helm_at;
    double status_at=0,shadow_at=0,pause_at=0,resume_deadline=0,reacquire_at=0;
    double fresh_guidance_at=0,latched_at=0,shadow_speed_at=0;
    enum class Step { Initial, NormalPause, NormalResume, OverridePause, Override, Fresh, Reacquire, Release };
    Step step=Step::Initial;
    unsigned outbound=0;
    bool registered=false,acquired=false,helm_state_seen=false,helm_stop_seen=false;
    double last_packet=-1,active_at=-1;
    const double started=mavmoos::monotonicSeconds();
    while(mavmoos::monotonicSeconds()-started<12) {
      const double now=mavmoos::monotonicSeconds();
      if(client.IsConnected()&&!registered) {
        for(const char* key:{"MAVMOOS_CAPABILITIES","MAVMOOS_STATUS","MAVMOOS_RUNTIME_STATUS","MAVMOOS_HELM_STATUS",
            "DESIRED_HEADING","DESIRED_SPEED","DESIRED_ALTITUDE","IVPHELM_STATE","IVPHELM_ALLSTOP",
            "MAVMOOS_SHADOW_COURSE","MAVMOOS_SHADOW_SPEED","MAVMOOS_SHADOW_ALTITUDE"}) client.Register(key,0);
        registered=true;
      }
      if(now-last_packet>=0.1) {
        peer.send(encoder.heartbeat(),bridge);
        mavlink_global_position_int_t position{};
        position.time_boot_ms=static_cast<uint32_t>((now-started)*1000);
        position.lat=423584560;position.lon=-710875890;
        position.alt=15000;position.relative_alt=5000;position.hdg=0;
        peer.send(encoder.encode(position,mavlink_msg_global_position_int_encode_status),bridge);
        peer.send(encoder.landed(MAV_LANDED_STATE_IN_AIR),bridge);
        mavlink_home_position_t home{};
        home.latitude=position.lat;home.longitude=position.lon;home.altitude=10000;
        peer.send(encoder.encode(home,mavlink_msg_home_position_encode_status),bridge);
        mavlink_ekf_status_report_t estimator{};
        estimator.flags=831;estimator.pos_horiz_variance=0.1;estimator.pos_vert_variance=0.1;
        peer.send(encoder.encode(estimator,mavlink_msg_ekf_status_report_encode_status),bridge);
        last_packet=now;
      }
      MOOSMSG_LIST mail;client.Fetch(mail);
      for(const auto& message:mail) {
        const auto& key=message.GetKey();
        const double at=message.GetTime();
        if(message.GetSource()=="pHelmIvP"&&message.IsString()) {
          if(key=="IVPHELM_STATE"||key=="IVPHELM_ALLSTOP")
            helm_at[message.GetString()]=at;
          if(key=="IVPHELM_STATE") helm_state_seen=true;
          if(key=="IVPHELM_ALLSTOP") helm_stop_seen=true;
        }
        if(message.GetSource()=="iMAVLink"&&message.IsDouble()&&key.rfind("MAVMOOS_SHADOW_",0)==0) {
          shadow_at=std::max(shadow_at,at);
          if(key=="MAVMOOS_SHADOW_SPEED" && message.GetDouble()>0) shadow_speed_at=at;
        }
        if(message.GetSource()=="iMAVLink"&&message.IsString()) {
          records[message.GetKey()]=message.GetString();
          if(key=="MAVMOOS_HELM_STATUS") {
            status_at=at;
            std::cout << "HELM\t" << now-started << '\t' << message.GetString() << '\n';
          }
        }
        if(message.GetSource()=="pHelmIvP" && message.IsDouble() &&
           ((key=="DESIRED_HEADING" && std::isfinite(message.GetDouble())) ||
            (key=="DESIRED_SPEED" && message.GetDouble()>0) ||
            (key=="DESIRED_ALTITUDE" && message.GetDouble()==5))) {
          helm_outputs.insert(key);
          helm_at[key]=at;
        }
      }
      const auto runtime=fields(records["MAVMOOS_RUNTIME_STATUS"]);
      if(records.count("MAVMOOS_STATUS"))
        check(fields(records["MAVMOOS_STATUS"])["state"]!="CONFIG_ERROR",
              "shadow app configuration rejected: "+records["MAVMOOS_STATUS"]);
      auto value=[&](const std::string& key) { const auto it=runtime.find(key);return it!=runtime.end()?it->second:std::string(); };
      // Wait for actual producer initialization; otherwise its initialize DEPLOY=false
      // may overwrite an early operator post while behavior libraries are loading.
      if(!acquired&&helm_state_seen&&helm_stop_seen&&value("position_valid")=="1"&&value("home_valid")=="1"&&value("estimator_valid")=="1") {
        client.Notify("MOOS_MANUAL_OVERRIDE","false");client.Notify("DEPLOY","true");
        client.Notify("MAVMOOS_NAV_CONTROL","acquire");acquired=true;
        std::cout << "OPERATOR\tacquire\n";
      }
      if(records["MAVMOOS_HELM_STATUS"].find("state=ACTIVE")!=std::string::npos&&active_at<0) {
        active_at=now;
        check(records["MAVMOOS_HELM_STATUS"].find("shadow=true")!=std::string::npos,"active module not labelled shadow");
        const auto helm=fields(records["MAVMOOS_HELM_STATUS"]);
        check(helm.at("readiness")=="READY" && helm.at("next_action")=="NONE" && helm.at("blocked_by")=="none", "active readiness diagnostics inconsistent");
        check(helm.at("readiness_scope")=="MODULE_LOCAL", "readiness authority scope missing");
        std::cout << "ASSERT\tshadow_active_with_real_stock_helm\n";
      }
      outbound+=peer.drain();
      check(outbound==0,"shadow mode emitted a UDP datagram");
      const auto helm=fields(records["MAVMOOS_HELM_STATUS"]);
      auto stateIs=[&](const char* phase) { return !helm.empty() && helm.at("state")==phase; };
      if(step==Step::Initial && active_at>0 && now-active_at>=0.7) {
        check(helm_outputs.size()==3,"not all stock helm outputs observed");
        check(value("tx_attempts")=="0"&&value("tx_accepted")=="0","shadow send ledger not zero");
        check(value("permission")=="false","shadow gained effectful permission");
        client.Notify("DEPLOY","false");
        step=Step::NormalPause;
      } else if(step==Step::NormalPause && stateIs("PAUSED") && helm.at("allstop")=="NothingToDo") {
        client.Notify("DEPLOY","true");
        step=Step::NormalResume;
      } else if(step==Step::NormalResume && stateIs("ACTIVE") && helm.at("reason")=="resumed_after_normal_handoff") {
        check(helm_at["clear"]>helm_at["NothingToDo"],"normal resume lacked a new helm decision");
        std::cout << "ASSERT\tnormal_behavior_handoff_resumed_without_acquire\n";
        client.Notify("DEPLOY","false");
        step=Step::OverridePause;
      } else if(step==Step::OverridePause && stateIs("PAUSED") && helm.at("allstop")=="NothingToDo") {
        // Status time and the helm's publication times share the unwarped MOOS clock.
        pause_at=status_at;
        resume_deadline=pause_at+std::stod(helm.at("handoff_remaining_s"));
        client.Notify("MOOS_MANUAL_OVERRIDE","true");
        step=Step::Override;
      } else if(step==Step::Override && helm_at["PARK"]>pause_at && helm_at["ManualOverride"]>pause_at) {
        // Restore the mission without granting control. Only pHelmIvP writes the helm outputs.
        client.Notify("DEPLOY","true");
        client.Notify("MOOS_MANUAL_OVERRIDE","false");
        step=Step::Fresh;
      } else if(step==Step::Fresh) {
        const double hard_stop_at=std::max(helm_at["ManualOverride"],helm_at["PARK"]);
        if(std::min({helm_at["DRIVE"],helm_at["clear"],helm_at["DESIRED_HEADING"],
                     helm_at["DESIRED_SPEED"],helm_at["DESIRED_ALTITUDE"]})>hard_stop_at && fresh_guidance_at==0) {
          fresh_guidance_at=std::max({helm_at["DRIVE"],helm_at["clear"],helm_at["DESIRED_HEADING"],
                                     helm_at["DESIRED_SPEED"],helm_at["DESIRED_ALTITUDE"]});
          check(fresh_guidance_at<resume_deadline,"fresh real helm guidance arrived outside the normal handoff window");
        }
        if(fresh_guidance_at>0 && status_at>fresh_guidance_at && helm.at("allstop")=="clear") {
          check(stateIs("LATCHED"),"hard stop during handoff allowed automatic navigation recovery");
          check(helm.at("reason")=="helm_not_driving"||helm.at("reason")=="allstop:ManualOverride",
                "navigation latched for a reason other than the helm override");
          check(shadow_at<=pause_at,"bridge produced a shadow target before explicit reacquisition");
          if(latched_at==0) latched_at=status_at;
          // Observe multiple bridge and helm cycles after guidance returns.
          if(status_at-latched_at>=0.3) {
            std::cout << "HANDOFF\tfresh_inputs_after_pause_s=" << fresh_guidance_at-pause_at
                      << ",window_remaining_at_pause_s=" << resume_deadline-pause_at
                      << ",latched_after_clear_s=" << status_at-latched_at
                      << ",reason=" << helm.at("reason") << '\n';
            reacquire_at=MOOSTime();
            client.Notify("MAVMOOS_NAV_CONTROL","acquire");
            step=Step::Reacquire;
          }
        }
      } else if(step==Step::Reacquire && stateIs("ACTIVE") && status_at>reacquire_at && shadow_speed_at>reacquire_at) {
        std::cout << "ASSERT\thandoff_auto_resume_and_hard_stop_reacquire\n";
        client.Notify("MAVMOOS_NAV_CONTROL","release");
        step=Step::Release;
      }
      if(step==Step::Release && stateIs("DISABLED")) {
        check(records.count("MAVMOOS_CAPABILITIES"),"capability catalog missing");
        std::cout << "CAPTURE\tMAVMOOS_CAPABILITIES\t" << records.at("MAVMOOS_CAPABILITIES") << '\n';
        std::cout << "CAPTURE\toutbound_datagrams=" << outbound << '\n';
        std::cout << "PASS real stock helm eligible shadow targets, explicit release, zero UDP output\n";
        client.Close();return 0;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
    throw std::runtime_error("shadow fixture deadline: "+records["MAVMOOS_HELM_STATUS"]+" / "+records["MAVMOOS_RUNTIME_STATUS"]);
  } catch(const std::exception& e) { std::cerr << "FAIL " << e.what() << '\n';return 1; }
}
