// Cross-provider ownership checks using the real Runtime/loaded DSOs. No I/O socket.
#include "ExecutionConcurrencySupport.h"

using namespace execution_concurrency;
int main(int argc,char** argv)
{
  try {
    require(argc==4,"usage: execution_concurrency_tests CONTROL_DSO MISSION_DSO PARAMETERS_DSO");
    const std::vector<std::string> modules={argv[1],argv[2],argv[3]};unsigned passed=0;
    const auto test=[&](const std::string& name,const auto& run) {
      for(const auto* frame:{"quad","hexa","octa"}) {
        const auto label=name+"/"+frame;
        Rig r(modules,frame);
        try { run(r);r.emit(label);++passed; }
        catch(const std::exception& e) {r.emit(label,false,e.what());throw;}
      }
    };
    test("mission_mutation_needs_fresh_ground_state",[](Rig& r){
      r.report_landed=false;r.tick(2.1);
      r.mission("stale-ground","clear");r.tick();
      require(r.outcome("MAVMOOS_MISSION_RESULT","stale-ground","FAILED")&&
              r.count(MAVLINK_MSG_ID_MISSION_CLEAR_ALL)==0,"stale ground state allowed mission mutation");
      r.report_landed=true;r.tick();r.upload();
    });
    test("enabled_AUTO_survives_same_ID_read_completion_and_HOME_current_normalization",[](Rig& r){
      r.running();r.plan[0].x+=77;r.plan[0].z+=10;r.plan[1].current=1;
      r.mission("run");r.parameter("run");
      r.until([&]{return r.outcome("MAVMOOS_MISSION_RESULT","run","COMPLETED")&&r.outcome("MAVMOOS_PARAM_RESULT","run","COMPLETED");});
      r.owned();require(r.sameProof(),"readback normalization renewed or replaced verified proof");r.immutableDeadline();
      require(r.commands(MAV_CMD_MISSION_START)==1&&r.commands(MAV_CMD_DO_SET_MODE)==0&&r.count(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED)==0,"readonly completion caused control effect");
    });
    test("mission_cancel_and_unsupported_parameter_cancel_cannot_release_AUTO_or_command_ID_history",[](Rig& r){
      r.running();const auto phase=r.last("MAVMOOS_TX_RESULT").at("id");require(phase.rfind("mx.",0)==0,"actual execution wire ID not observed");
      r.hold_missions=r.hold_parameters=true;r.mission(phase);r.parameter("run");
      r.until([&]{return !r.deferred_missions.empty()&&!r.deferred_parameters.empty();});
      r.mission(phase,"cancel");r.parameter("run","cancel");r.owned();
      require(r.outcome("MAVMOOS_MISSION_RESULT",phase,"CANCELLED")&&r.outcome("MAVMOOS_PARAM_RESULT","run","REJECTED"),"data cancellation outcomes not explicit");
      r.releaseMissions();r.releaseParameters();r.tick();r.owned();
      require(r.outcome("MAVMOOS_PARAM_RESULT","run","COMPLETED")&&r.sameProof(),"unsupported cancel destroyed valid readonly work or proof");
      CommandRequest duplicate;duplicate.id=phase;duplicate.execution_id="run";duplicate.command=MAV_CMD_DO_PAUSE_CONTINUE;duplicate.expires=r.time+1;duplicate.operation_expires=r.originalDeadline();
      require(!r.host.sendCommand(duplicate),"mission namespace cancellation cleared retired command ID");r.immutableDeadline();
    });
    test("fresh_progress_republished_JSON_and_read_completion_do_not_renew_original_AUTO_expiry",[](Rig& r){
      r.running(2);const auto expires=r.originalDeadline();
      r.mail("MAVMOOS_STORED_MISSION_PROOF","schema=1,valid=true,content_id="+r.original_proof.content_id+",revision="+std::to_string(r.original_proof.revision)+",verified_at="+std::to_string(r.time+100),"iMAVLink");
      r.hold_parameters=r.hold_missions=true;r.parameter("expiry-read");r.mission("expiry-read");
      r.until([&]{return !r.deferred_parameters.empty()&&!r.deferred_missions.empty();});
      while(r.time<expires+.05)r.tick();
      require(r.outcome("MAVMOOS_MISSION_EXECUTION_RESULT","run","TIMED_OUT")&&!r.host.missionExecutionGranted("run")&&r.host.controlBusy(),"fresh data or publication renewed AUTO expiry/released ownership");
      r.releaseMissions();r.releaseParameters();
      r.until([&]{return r.outcome("MAVMOOS_PARAM_RESULT","expiry-read","COMPLETED")&&r.outcome("MAVMOOS_MISSION_RESULT","expiry-read","COMPLETED");});
      require(r.sameProof()&&r.host.controlBusy()&&!r.host.navigationGranted()&&r.commands(MAV_CMD_MISSION_START)==1&&r.commands(MAV_CMD_DO_SET_MODE)==0,"post-expiry reads resumed/stopped control or renewed proof");r.immutableDeadline();
    });
    test("explicit_abort_has_command_priority_over_dual_read_traffic_and_read_completion_does_not_acquire_helm",[](Rig& r){
      r.running();r.parameter("parallel-table","list");r.mission("parallel-mission");
      r.mail("MAVMOOS_MISSION_CONTROL","schema=1,id=stop,action=abort,target=run,timeout=8");const auto begin=r.frames.size();r.tick();
      require(r.frames.size()>begin&&r.frames[begin].accepted&&command(r.frames[begin].message)==MAV_CMD_DO_SET_MODE,"readonly lane preceded explicit stop command");
      require(r.frames.size()-begin<=2,"dual readonly features doubled shared protocol budget including observation requests");
      r.until([&]{return r.outcome("MAVMOOS_MISSION_EXECUTION_RESULT","stop","COMPLETED")&&r.outcome("MAVMOOS_PARAM_RESULT","parallel-table","COMPLETED")&&r.outcome("MAVMOOS_MISSION_RESULT","parallel-mission","COMPLETED");});
      require(r.outcome("MAVMOOS_MISSION_EXECUTION_RESULT","run","CANCELLED")&&!r.host.controlBusy()&&!r.host.navigationGranted()&&r.commands(MAV_CMD_DO_SET_MODE)==1&&r.commands(MAV_CMD_MISSION_START)==1,"observed abort/read completion changed ownership or replayed control");r.immutableDeadline();
    });
    test("enabled_coordinator_start_revoked_at_final_gate_does_not_transmit_while_reads_finish",[](Rig& r){
      r.upload();r.airborne=true;r.mission("final-read");r.parameter("final-read");bool revoked=false;
      r.before_gate=[&](const auto& message){if(command(message)==MAV_CMD_MISSION_START&&!revoked){revoked=true;r.host.invalidateStoredMissionProof("fault_at_final_start_gate");}};
      r.requestStart();r.until([&]{return revoked&&r.outcome("MAVMOOS_MISSION_EXECUTION_RESULT","run","FAILED");});
      r.until([&]{return r.outcome("MAVMOOS_MISSION_RESULT","final-read","COMPLETED")&&r.outcome("MAVMOOS_PARAM_RESULT","final-read","COMPLETED");});
      require(r.commands(MAV_CMD_MISSION_START)==0&&r.commands(MAV_CMD_MISSION_START,false)==1&&r.host.controlBusy()&&!r.host.missionExecutionGranted("run")&&!r.host.navigationGranted()&&!r.host.storedMissionProof().valid,"revoked start crossed gate or readonly completion reacquired/reverified");
      for(unsigned n=0;n<10;++n)r.tick();require(r.commands(MAV_CMD_MISSION_START,false)==1,"failed start automatically retried");r.immutableDeadline();
    });
    test("reset_during_same_ID_read_ACK_cancels_provider_work_without_erasing_sent_AUTO_uncertainty",[](Rig& r){
      r.running();r.mission("run");r.hold_parameters=true;r.parameter("run");bool reset=false;
      r.before_gate=[&](const auto& message){if(message.msgid==MAVLINK_MSG_ID_MISSION_ACK&&!reset){reset=true;r.host.disconnect(r.time,"test_read_ack_reset");r.fresh();}};
      r.until([&]{return reset;});r.releaseParameters();for(unsigned n=0;n<5;++n)r.tick();
      require(r.outcome("MAVMOOS_MISSION_RESULT","run","CANCELLED")&&r.outcome("MAVMOOS_PARAM_RESULT","run","CANCELLED")&&r.outcome("MAVMOOS_MISSION_EXECUTION_RESULT","run","LOST"),"reset scopes did not cancel independently");
      require(!r.host.storedMissionProof().valid&&r.host.controlBusy()&&!r.host.missionExecutionGranted("run")&&!r.host.navigationGranted(),"reset discarded unresolved AUTO or reused proof");
      r.parameter("new-session-read");r.until([&]{return r.outcome("MAVMOOS_PARAM_RESULT","new-session-read","COMPLETED");});
      require(r.host.controlBusy()&&r.commands(MAV_CMD_MISSION_START)==1&&r.commands(MAV_CMD_DO_SET_MODE)==0,"new read resumed or stopped unresolved AUTO");r.immutableDeadline();
    });
    test("actual_Runtime_preserves_fast_ACK_read_after_send_with_pre_send_queue_floor",[](Rig& r){
      r.queue_floor_ack=true;r.running();require(!r.ack_windows.empty(),"receive-order fixture did not exercise bounded ACK metadata");
      require(!r.outcome("MAVMOOS_MISSION_EXECUTION_RESULT","run","COMPLETED"),"start ACK was promoted into whole execution completion");
      r.mail("MAVMOOS_MISSION_CONTROL","schema=1,id=fast-pause,action=pause,target=run,timeout=8");
      r.until([&]{return r.outcome("MAVMOOS_MISSION_EXECUTION_RESULT","fast-pause","COMPLETED");});r.owned();
      r.mail("MAVMOOS_MISSION_CONTROL","schema=1,id=fast-stop,action=abort,target=run,timeout=8");
      r.until([&]{return r.outcome("MAVMOOS_MISSION_EXECUTION_RESULT","fast-stop","COMPLETED");});
      for(const auto& window:r.ack_windows)require(window[0]<window[1]&&window[1]<window[2],"fixture did not preserve queuefloor/send/read ordering");
      require(r.commands(MAV_CMD_DO_SET_MISSION_CURRENT)==1&&r.commands(MAV_CMD_MISSION_START)==1&&r.commands(MAV_CMD_DO_PAUSE_CONTINUE)==1&&r.commands(MAV_CMD_DO_SET_MODE)==1&&r.outcome("MAVMOOS_MISSION_EXECUTION_RESULT","run","CANCELLED")&&!r.host.controlBusy()&&!r.host.navigationGranted(),"fast ACK flow did not preserve four distinct observed phases and explicit handback");r.immutableDeadline();
    });
    std::cout<<"{\"type\":\"summary\",\"result\":\"PASS\",\"groups\":"<<passed<<",\"scope\":\"actual Runtime and three loaded DSOs; deterministic fake I/O; no sockets or vehicle\"}\n";return 0;
  }catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
