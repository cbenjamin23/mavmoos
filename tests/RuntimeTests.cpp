#include "Runtime.h"
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>
using namespace mavmoos::runtime;
namespace {
void check(bool b,const char* why) { if(!b) throw std::runtime_error(why); }
std::string control_module;
struct Rig {
  double clock=100;
  unsigned clock_calls=0;
  std::function<double(unsigned)> clock_sample;
  std::vector<mavlink_message_t> sent;
  Runtime runtime;
  Rig(bool simulation=true,bool shadow=false,bool control=true):runtime([](const auto&,const auto&,double,bool){},
    [this](const uint8_t* data,std::size_t size) {
      mavlink_message_t msg{},buffer{}; mavlink_status_t status{},result{};
      for(std::size_t i=0;i<size;++i) if(mavlink_frame_char_buffer(&buffer,&status,data[i],&msg,&result)==MAVLINK_FRAMING_OK) sent.push_back(msg);
      return true;
    },simulation,shadow,[this]{ ++clock_calls; return clock_sample?clock_sample(clock_calls):clock; }) {
    if(control) {
      check(runtime.load(control_module),"load Copter control provider");
      runtime.configure("control_native_actions","false");runtime.configure("control_helm","false");
    }
    refresh();
  }
  void refresh(unsigned mode=4,bool armed=true,unsigned landed=2) {
    mavlink_message_t m{};
    mavlink_msg_heartbeat_pack(1,1,&m,2,3,armed?128:0,mode,4); runtime.message(m,clock);
    mavlink_msg_global_position_int_pack(1,1,&m,1000,423584560,-710875890,15000,5000,0,0,0,9000); runtime.message(m,clock);
    mavlink_msg_extended_sys_state_pack(1,1,&m,0,landed); runtime.message(m,clock);
    mavlink_home_position_t home{};home.latitude=423584560;home.longitude=-710875890;home.altitude=10000;
    mavlink_msg_home_position_encode(1,1,&m,&home);runtime.message(m,clock);
    mavlink_ekf_status_report_t ekf{};ekf.flags=51;
    mavlink_msg_ekf_status_report_encode(1,1,&m,&ekf);runtime.message(m,clock);
  }
  unsigned count(unsigned type) const { unsigned n=0;for(const auto& m:sent) if(m.msgid==type)++n;return n; }
  unsigned commandCount(unsigned command) const {
    unsigned count=0;
    for(const auto& m:sent) if(m.msgid==MAVLINK_MSG_ID_COMMAND_LONG) {
      mavlink_command_long_t p{};mavlink_msg_command_long_decode(&m,&p);
      if(p.command==command) ++count;
    }
    return count;
  }
  void warmup() { runtime.tick(clock); sent.clear(); }
  void afterPacking(double first,double final) {
    clock_calls=0;
    clock_sample=[first,final](unsigned call){return call==1?first:final;};
  }
};
}
int main(int argc,char** argv)
{
  try {
    check(argc==2,"usage: runtime_tests CONTROL_MODULE");control_module=argv[1];
    { Rig r(true,false,false);CommandRequest c{"no-provider",MAV_CMD_DO_SET_MODE,{},101};c.params[0]=1;c.params[1]=4;
      check(!r.runtime.acquireNavigation("missing")&&!r.runtime.navigation({90,1,5,101})&&!r.runtime.sendCommand(c),"missing provider allowed control");
      r.runtime.tick(r.clock);check(r.count(84)==0&&r.commandCount(MAV_CMD_DO_SET_MODE)==0,"missing provider emitted motion work"); }
    { Rig r(false); CommandRequest c{"arm",MAV_CMD_COMPONENT_ARM_DISARM,{},101};
      check(!r.runtime.sendCommand(c),"RX_ONLY command admitted");r.runtime.tick(100);check(r.sent.empty(),"RX_ONLY transmitted"); }
    { Rig r(true,true); check(r.runtime.acquireNavigation("shadow"),"shadow acquisition");
      r.runtime.navigation({90,1,5,101});r.runtime.tick(100);check(r.sent.empty(),"SHADOW transmitted"); }
    { Rig r; check(r.runtime.acquireNavigation("a"),"acquire failed");
      check(r.runtime.navigation({90,1,5,101}),"firstnav");check(r.runtime.navigation({0,1,5,101}),"latestnav");
      r.runtime.tick(100);check(r.count(84)==1,"latest-only target not enforced");
      mavlink_set_position_target_local_ned_t target{};
      for(const auto& m:r.sent)if(m.msgid==84)mavlink_msg_set_position_target_local_ned_decode(&m,&target);
      check(target.vx>.99 && std::abs(target.vy)<.01,"old waiting target sent");
      r.clock=100.1;r.runtime.revokeNavigation("test");check(!r.runtime.navigation({90,1,5,101}),"revoked navigation admitted");
      r.runtime.tick(r.clock);check(r.count(84)==1,"revocation failed"); }
    { Rig r;check(r.runtime.acquireNavigation("b"),"acquire");r.runtime.navigation({90,1,5,100.05});
      r.clock=100.1;r.runtime.tick(r.clock);check(r.count(84)==0,"expired navigation sent"); }
    { Rig r;check(r.runtime.acquireNavigation("c"),"acquire");r.runtime.navigation({90,1,5,101});
      r.clock=100.1;r.refresh(6);r.runtime.tick(r.clock);check(r.count(84)==0,"external mode takeover not respected");
      check(!r.runtime.acquireNavigation("rtl"),"RTL recapture permitted"); }
    { Rig r;r.refresh(4,false,1);CommandRequest c{"arm",MAV_CMD_COMPONENT_ARM_DISARM,{},101};c.params[0]=1;
      check(r.runtime.sendCommand(c),"arm queue admission");r.clock=100.1;r.refresh(4,true,2);
      r.runtime.tick(r.clock);bool armed_command=false;
      for(const auto& m:r.sent)if(m.msgid==76){mavlink_command_long_t p{};mavlink_msg_command_long_decode(&m,&p);if(p.command==400)armed_command=true;}
      check(!armed_command,"queued arm bypassed final landed guard"); }
    { Rig r;check(r.runtime.acquireNavigation("d"),"acquire");r.clock=103.1;r.runtime.tick(r.clock);
      check(!r.runtime.navigation({90,1,5,104}),"stale vehicle navigation admitted");r.clock=104;r.refresh();
      check(!r.runtime.navigation({90,1,5,105}),"reconnect revived permission"); }
    { Rig r;r.warmup();r.clock=100.1;r.refresh();
      check(r.runtime.acquireNavigation("packing-expiry"),"packing expiry acquire");
      check(r.runtime.navigation({90,1,5,100.15}),"packing expiry admission");
      r.afterPacking(100.1,100.2);r.runtime.tick(100.1);
      check(r.clock_calls>=2,"NAV test did not reach post-encoding clock sample");
      check(r.count(84)==0,"NAV expired during encoding was transmitted"); }
    { Rig r;r.warmup();r.clock=100.1;r.refresh();
      CommandRequest c{"mode-expiry",MAV_CMD_DO_SET_MODE,{},100.15};c.params[0]=1;c.params[1]=4;
      check(r.runtime.sendCommand(c),"command packing expiry admission");
      r.afterPacking(100.1,100.2);r.runtime.tick(100.1);
      check(r.clock_calls>=2,"COMMAND test did not reach post-encoding clock sample");
      check(r.commandCount(MAV_CMD_DO_SET_MODE)==0,"COMMAND expired during encoding was transmitted"); }
    { Rig r;r.warmup();r.clock=100.1;r.refresh();
      check(r.runtime.acquireNavigation("revoke-during-packing"),"final permission fixture acquire");
      check(r.runtime.navigation({90,1,5,101}),"final permission fixture target");
      r.clock_calls=0;r.clock_sample=[&r](unsigned call) {
        if(call==2)r.runtime.revokeNavigation("test_final_gate_revocation");
        return 100.1;
      };
      r.runtime.tick(100.1);
      check(r.clock_calls>=2&&r.count(84)==0,"NAV permission revoked during encoding/final clock still emitted a target");
      check(r.runtime.status().find("permission=false")!=std::string::npos,"final permission fixture retained authority");
      check(r.runtime.vehicle().heartbeat_valid&&r.runtime.vehicle().position_valid,"permission fixture also lost fresh FC state");
    }
    { Rig r;r.refresh(4,false,1);r.warmup();
      CommandRequest c{"cancel-at-handoff",MAV_CMD_COMPONENT_ARM_DISARM,{},101};c.params[0]=1;c.operation_expires=105;
      check(r.runtime.sendCommand(c),"handoff ARM admission");
      r.clock_calls=0;r.clock_sample=[&r](unsigned call) {if(call==2)r.runtime.cancelCommand("cancel-at-handoff");return 100.;};
      r.runtime.tick(100);
      check(r.clock_calls>=2&&r.commandCount(MAV_CMD_COMPONENT_ARM_DISARM)==0,"cancelled dequeued ARM still sent");
      check(r.runtime.controlBusy(),"handoff cancellation erased uncertainty lease");
    }
    { Rig r;r.refresh(5);r.warmup();check(r.runtime.acquireNavigation("cancel-acquisition"),"handoff acquisition");
      r.clock_calls=0;r.clock_sample=[&r](unsigned call) {if(call==2)r.runtime.revokeNavigation("handoff_cancel");return 100.;};
      r.runtime.tick(100);
      check(r.clock_calls>=2&&r.commandCount(MAV_CMD_DO_SET_MODE)==0,"revoked dequeued acquisition still sent");
    }
    { Rig r;r.warmup();r.clock=100.1;r.refresh();check(r.runtime.acquireNavigation("old-grant"),"old grant");
      check(r.runtime.navigation({90,1,5,101}),"old grant target");
      r.clock_calls=0;r.clock_sample=[&r](unsigned call) {
        if(call==2) {r.runtime.revokeNavigation("new_boundary");check(r.runtime.acquireNavigation("new-grant"),"new grant");}
        return 100.1;
      };
      r.runtime.tick(100.1);
      check(r.clock_calls>=2&&r.runtime.navigationGranted()&&r.count(84)==0,"old target crossed a new authority grant");
    }
    { Rig r;r.warmup();check(r.runtime.acquireNavigation("hold-old-grant"),"hold fixture grant");r.runtime.revokeNavigation("capture_hold");
      r.clock_calls=0;r.clock_sample=[&r](unsigned call) {if(call==2)check(r.runtime.acquireNavigation("hold-new-grant"),"hold fixture reacquire");return 100.;};
      r.runtime.tick(100);
      check(r.clock_calls>=2&&r.count(86)==0,"old recovery hold crossed a new authority grant");
    }
    for(const double bad:{100.05,std::numeric_limits<double>::quiet_NaN()}) {
      Rig r;r.warmup();r.clock=100.1;r.refresh();
      check(r.runtime.acquireNavigation("bad-final-clock"),"bad clock acquire");
      check(r.runtime.navigation({90,1,5,101}),"bad clock admission");
      r.afterPacking(100.1,bad);r.runtime.tick(100.1);
      check(r.clock_calls>=2 && r.count(84)==0,"bad final NAV clock allowed effect");
      check(!r.runtime.vehicle().heartbeat_valid,"bad clock did not invalidate vehicle session");
    }
    for(const double bad:{100.05,std::numeric_limits<double>::quiet_NaN()}) {
      Rig r;r.warmup();r.clock=100.1;r.refresh();
      CommandRequest c{"mode-clock",MAV_CMD_DO_SET_MODE,{},101};c.params[0]=1;c.params[1]=4;
      check(r.runtime.sendCommand(c),"bad command clock admission");
      r.afterPacking(100.1,bad);r.runtime.tick(100.1);
      check(r.clock_calls>=2 && r.commandCount(MAV_CMD_DO_SET_MODE)==0,"bad final COMMAND clock allowed effect");
      check(!r.runtime.vehicle().heartbeat_valid,"bad command clock retained vehicle session");
    }
    { Rig r;r.refresh(4,false,1);r.warmup();r.clock=101.9;
      mavlink_message_t heartbeat{};
      mavlink_msg_heartbeat_pack(1,1,&heartbeat,2,3,0,4,4);r.runtime.message(heartbeat,r.clock);
      CommandRequest c{"arm-stale-landed",MAV_CMD_COMPONENT_ARM_DISARM,{},103};c.params[0]=1;
      check(r.runtime.sendCommand(c),"ARM stale-landed fixture admission");
      r.afterPacking(101.9,102.1);r.runtime.tick(101.9);
      check(r.clock_calls>=2,"ARM test did not reach final clock sample");
      check(r.commandCount(MAV_CMD_COMPONENT_ARM_DISARM)==0,"ARM used landed evidence that expired during packing");
      check(r.runtime.vehicle().heartbeat_valid,"ARM stale-landed fixture also lost heartbeat"); }
    { Rig r;r.refresh(4,true,1);r.warmup();r.clock=100.4;
      CommandRequest c{"takeoff-stale-pose",MAV_CMD_NAV_TAKEOFF,{},101};c.params[6]=5;
      check(r.runtime.sendCommand(c),"TAKEOFF stale-pose fixture admission");
      r.afterPacking(100.4,100.6);r.runtime.tick(100.4);
      check(r.clock_calls>=2,"TAKEOFF test did not reach final clock sample");
      check(r.commandCount(MAV_CMD_NAV_TAKEOFF)==0,"TAKEOFF used pose that expired during packing");
      check(r.runtime.vehicle().heartbeat_valid && r.runtime.vehicle().landed_valid,
        "TAKEOFF stale-pose fixture also lost other prerequisites"); }
    { Rig r; r.clock=5000000.; r.refresh();
      check(r.runtime.acquireNavigation("long-uptime"),"long-uptime acquisition");
      check(r.runtime.navigation({90,1,5,r.clock+1}),"long-uptime navigation");
      r.runtime.tick(r.clock); check(r.count(84)==1,"long-uptime target missing");
      for(const auto& m:r.sent) if(m.msgid==84) {
        mavlink_set_position_target_local_ned_t p{};mavlink_msg_set_position_target_local_ned_decode(&m,&p);
        check(p.time_boot_ms==705032704U,"wire timestamp failed explicit uint32 wrap");
      } }
    { Rig r(true,false,false);r.runtime.disconnect();
      mavlink_message_t heartbeat{};mavlink_msg_heartbeat_pack(1,1,&heartbeat,2,3,0,4,4);
      r.runtime.message(heartbeat,r.clock);
      ParameterRequest p;p.id="read-only";p.name="ABCDEFGHIJKLMNOP";p.expires=108;
      check(r.runtime.parameter(p),"heartbeat-only parameter read denied");r.runtime.tick(r.clock);
      check(r.count(MAVLINK_MSG_ID_PARAM_REQUEST_READ)==1,"heartbeat-only read not sent");
      for(const auto& m:r.sent)if(m.msgid==MAVLINK_MSG_ID_PARAM_REQUEST_READ) {
        mavlink_param_request_read_t wire{};mavlink_msg_param_request_read_decode(&m,&wire);
        check(std::string(wire.param_id,16)==p.name&&wire.param_index==-1,"16-byte name wire encoding changed");
      }
      ParameterRequest list;list.id="list";list.action=ParameterAction::List;list.expires=120;
      check(!r.runtime.parameter(list),"legacy streaming admitted without observed ground state");
      r.refresh(4,false,1);check(r.runtime.parameter(list),"explicit ground-bench list denied");r.runtime.tick(r.clock);
      check(r.count(MAVLINK_MSG_ID_PARAM_REQUEST_LIST)==1,"parameter list not sent");
    }
    { ParameterRequest p;p.id="read";p.name="FRAME_CLASS";p.expires=108;
      Rig rx(false),shadow(true,true);check(!rx.runtime.parameter(p)&&!shadow.runtime.parameter(p),"disabled mode admitted parameter output");
      Rig r;for(int i=0;i<4;++i){p.id="read"+std::to_string(i);check(r.runtime.parameter(p),"bounded lane fixture denied");}
      check(!r.runtime.parameter(p),"duplicate queued parameter admitted");p.id="overflow";
      check(!r.runtime.parameter(p),"parameter lane exceeded capacity");
      r.runtime.disconnect();r.refresh();r.runtime.tick(r.clock);
      check(r.count(MAVLINK_MSG_ID_PARAM_REQUEST_READ)==0,"disconnect revived parameter work");
    }
    { Rig r;ParameterRequest p;p.id="write";p.action=ParameterAction::Write;p.name="SIM_WIND_SPD";p.value=.25f;p.expires=108;
      check(!r.runtime.parameter(p),"armed airborne write admitted");r.refresh(4,false,1);
      auto bad=p;bad.name="SYSID_THISMAV";check(!r.runtime.parameter(bad),"write outside simulation allowlist admitted");
      bad=p;bad.value=21;check(!r.runtime.parameter(bad),"out-of-range write admitted");
      bad=p;bad.type=MAV_PARAM_TYPE_UINT32;check(!r.runtime.parameter(bad),"wrong write encoding admitted");
      check(r.runtime.parameter(p),"disarmed ground write denied");r.refresh(4,true,2);r.runtime.tick(r.clock);
      check(r.count(MAVLINK_MSG_ID_PARAM_SET)==0,"queued write bypassed final state guard");
    }
    { Rig r;r.warmup();ParameterRequest p;p.id="read-expiry";p.name="FRAME_CLASS";p.expires=100.15;
      check(r.runtime.parameter(p),"parameter packing expiry fixture denied");r.afterPacking(100.1,100.2);r.runtime.tick(100.1);
      check(r.count(MAVLINK_MSG_ID_PARAM_REQUEST_READ)==0,"parameter expired during packing was sent");
    }
    { Rig r;r.warmup();ParameterRequest p;p.id="coexist";p.name="FRAME_CLASS";p.expires=108;
      check(r.runtime.parameter(p)&&r.runtime.acquireNavigation("parallel"),"parallel parameter/nav fixture denied");
      check(r.runtime.navigation({90,1,5,101}),"parallel navigation denied");r.runtime.tick(r.clock);
      check(r.count(84)==1&&r.count(MAVLINK_MSG_ID_PARAM_REQUEST_READ)==1,"parameter lane replaced or starved navigation");
    }
    { Rig r;check(!r.runtime.suspendNavigation(100.5),"suspension without a held grant admitted");
      check(r.runtime.acquireNavigation("pause"),"normal pause fixture acquire");
      check(r.runtime.suspendNavigation(100.5),"bounded pause denied");r.clock=100.1;r.refresh();
      check(!r.runtime.suspendNavigation(101),"repeated pause extended original permit");
      r.runtime.revokeNavigation("operator_release");check(!r.runtime.resumeNavigation("old"),"release retained a resume permit");
    }
    { Rig r;check(r.runtime.acquireNavigation("pause-expiry"),"expiry fixture acquire");
      check(r.runtime.suspendNavigation(100.2),"expiry fixture pause");r.clock=100.2;r.refresh();
      check(!r.runtime.resumeNavigation("late"),"expired handoff permit resumed");
    }
    { Rig r;check(r.runtime.acquireNavigation("command-boundary"),"command fixture acquire");
      CommandRequest c{"hold",MAV_CMD_DO_SET_MODE,{},101};c.params[0]=1;c.params[1]=5;c.operation_expires=108;
      auto bad=c;bad.operation_expires=100.5;
      check(!r.runtime.sendCommand(bad),"operation deadline before send deadline admitted");
      check(r.runtime.navigation({90,1,5,101}),"rejected command revoked existing NAV");
      check(r.runtime.sendCommand(c),"valid command denied");
      check(!r.runtime.acquireNavigation("too-early")&&!r.runtime.resumeNavigation("old"),"pending command allowed authority recapture");
      r.runtime.commandFinished("hold");
      check(!r.runtime.navigation({90,1,5,101}),"terminal command automatically restored NAV");
      check(r.runtime.acquireNavigation("deliberate"),"fresh explicit acquisition denied after terminal command");
      r.runtime.tick(r.clock);check(r.commandCount(MAV_CMD_DO_SET_MODE)==0,"terminal unsent command survived queue cancellation");
    }
    { Rig r;CommandRequest c{"expiring",MAV_CMD_DO_SET_MODE,{},100.1};c.params[0]=1;c.params[1]=5;c.operation_expires=100.2;
      check(r.runtime.sendCommand(c),"expiry command fixture denied");r.clock=100.2;r.refresh();
      check(r.runtime.acquireNavigation("after-expiry"),"expired command gate did not release");r.runtime.tick(r.clock);
      check(r.commandCount(MAV_CMD_DO_SET_MODE)==0,"expired waiting command emitted after new acquisition");
    }
    { Rig r;check(r.runtime.acquireNavigation("hold-cancel"),"hold fixture acquire");
      r.runtime.revokeNavigation("operator_release");check(r.runtime.acquireNavigation("new"),"reacquisition denied");
      r.runtime.navigation({90,1,5,101});r.runtime.tick(r.clock);
      check(r.count(MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT)==0&&r.count(84)==1,"obsolete conditional hold survived reacquisition");
    }
    { Rig r;check(r.runtime.acquireNavigation("repeated-release"),"repeated release fixture acquire");
      r.runtime.revokeNavigation("rover_direct:operator_release");
      r.clock=100.1;r.refresh();r.runtime.revokeNavigation("operator_release");
      r.runtime.tick(r.clock);
      check(r.count(MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT)==1&&!r.runtime.navigationGranted(),
            "repeated operator release erased the one bounded stop");
      r.runtime.revokeNavigation("operator_release");r.runtime.tick(r.clock);
      check(r.count(MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT)==1,"repeated release sent another stop");
    }
    for(bool mode_takeover:{false,true}) {
      Rig r;check(r.runtime.acquireNavigation("stop-boundary"),"stop boundary fixture acquire");
      r.runtime.revokeNavigation("operator_release");
      r.clock=mode_takeover?100.1:100.5;
      if(mode_takeover)r.refresh(5);
      r.refresh();r.runtime.revokeNavigation("operator_release");r.runtime.tick(r.clock);
      check(r.count(MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT)==0,
            "repeated release revived a stop after mode takeover or original expiry");
    }
    { Rig r(true,true);check(r.runtime.acquireNavigation("shadow-pause"),"shadow acquire");
      check(r.runtime.suspendNavigation(100.5)&&r.runtime.resumeNavigation("shadow-resume"),"logical shadow handoff failed");
      r.runtime.tick(r.clock);check(r.sent.empty()&&r.runtime.status().find("permission=false")!=std::string::npos,"shadow grant became effectful permission");
    }
    { Rig r;CommandRequest c{"lease",MAV_CMD_DO_SET_MODE,{},101};c.params[0]=1;c.params[1]=4;c.operation_expires=108;
      for(int i=0;i<4;++i){c.id="lease"+std::to_string(i);check(r.runtime.sendCommand(c),"lease capacity fixture denied");}
      c.id="overflow";check(!r.runtime.sendCommand(c),"external command lease capacity exceeded");
      r.runtime.commandFinished("lease0");check(r.runtime.sendCommand(c),"terminal command did not free bounded capacity");
    }
    { Rig r;r.refresh(4,false,1);
      ParameterRequest p;p.id="queued-write";p.action=ParameterAction::Write;p.name="SIM_WIND_SPD";p.value=.25f;p.expires=108;
      check(r.runtime.parameter(p),"pre-ARM parameter fixture denied");
      CommandRequest c{"own-arm",MAV_CMD_COMPONENT_ARM_DISARM,{},101};c.params[0]=1;c.operation_expires=108;
      check(r.runtime.sendCommand(c),"ARM transition fixture denied");r.runtime.tick(r.clock);
      check(r.commandCount(MAV_CMD_COMPONENT_ARM_DISARM)==1&&r.count(MAVLINK_MSG_ID_PARAM_SET)==0,"queued SET followed own ARM while armed telemetry was still old");
      ParameterRequest read;read.id="during-command";read.name="FRAME_CLASS";read.expires=108;
      check(r.runtime.parameter(read),"command interlock blocked independent read");r.runtime.tick(r.clock);r.runtime.tick(r.clock);
      check(r.count(MAVLINK_MSG_ID_PARAM_REQUEST_READ)==1,"independent read did not progress during command lease");
    }
    { Rig r;for(unsigned i=0;i<6;++i){r.clock=100+i*.11;r.refresh();r.runtime.tick(r.clock);}unsigned intervals=0;
      for(const auto& m:r.sent)if(m.msgid==MAVLINK_MSG_ID_COMMAND_LONG){mavlink_command_long_t p{};mavlink_msg_command_long_decode(&m,&p);
        if(p.command!=MAV_CMD_SET_MESSAGE_INTERVAL)continue;
        const float expected=p.param1==33?50000.f:p.param1==245?250000.f:500000.f;
        check((p.param1==33||p.param1==245||p.param1==193||p.param1==242)&&p.param2==expected,"Copter critical stream profile lacks parameter-transfer margin");++intervals;
      }
      check(intervals==4,"critical stream request family count changed");
    }
    { Rig r;check(r.runtime.acquireNavigation("paused-disarm"),"disarm fixture acquire");
      check(r.runtime.suspendNavigation(100.5),"disarm fixture pause");
      r.clock=100.1;r.refresh(4,false,2);r.clock=100.2;r.refresh();
      check(!r.runtime.resumeNavigation("old-disarm"),"disarm then fresh armed state revived permit");
      check(r.runtime.acquireNavigation("explicit-after-disarm"),"fresh explicit acquisition after disarm denied");
    }
    { Rig r;check(r.runtime.acquireNavigation("paused-landed"),"landed fixture acquire");
      check(r.runtime.suspendNavigation(100.5),"landed fixture pause");
      r.clock=100.1;r.refresh(4,true,1);r.clock=100.2;r.refresh();
      check(!r.runtime.resumeNavigation("old-landed"),"landed then fresh airborne state revived permit");
    }
    { Rig r;r.refresh(5);check(r.runtime.acquireNavigation("loiter-acquire"),"explicit Loiter acquisition denied");
      check(!r.runtime.suspendNavigation(100.5),"pre-Guided acquisition issued normal resume permit");
      r.runtime.tick(r.clock);check(r.commandCount(MAV_CMD_DO_SET_MODE)==1,"Loiter-to-Guided acquisition was cancelled");
      r.clock=100.1;r.refresh();check(r.runtime.suspendNavigation(100.5),"Guided suspension denied");
      check(r.runtime.resumeNavigation("valid-normal-resume"),"eligible normal resume denied");
    }
    std::cout<<"PASS 39 runtime groups: control/freshness guards, independent parameters, bounded resume permits, cancellation through handoff, command/write interlocks and measured Copter stream profile\n";
    return 0;
  } catch(const std::exception& e) { std::cerr<<"FAIL "<<e.what()<<'\n';return 1; }
}
