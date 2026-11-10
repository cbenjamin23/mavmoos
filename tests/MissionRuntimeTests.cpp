#include "Runtime.h"
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace mavmoos::runtime;
namespace {
void check(bool value,const char* reason) { if(!value) throw std::runtime_error(reason); }
struct Rig {
  std::vector<mavlink_message_t> sent;
  std::vector<std::pair<std::string,std::string>> publications;
  Runtime host;
  explicit Rig(const std::string& control={},bool simulation=true,bool shadow=false,
               std::function<double()> clock={})
    :host([&](const auto& key,const auto& text,double,bool){publications.emplace_back(key,text);},
          [&](const uint8_t* data,std::size_t size) {
            mavlink_message_t rx{},message{};mavlink_status_t status{},result{};bool parsed=false;
            for(std::size_t i=0;i<size;++i)
              if(mavlink_frame_char_buffer(&rx,&status,data[i],&message,&result)==MAVLINK_FRAMING_OK) parsed=true;
            check(parsed,"outbound frame did not decode");sent.push_back(message);return true;
          },simulation,shadow,std::move(clock))
  {
    host.checkpoint(100);
    if(!control.empty()) {
      check(host.load(control),"load real control provider");
      host.configure("control_native_actions","false");host.configure("control_helm","false");
    }
    observations(100,false);
  }
  void observations(double at,bool airborne) {
    mavlink_message_t message{};mavlink_status_t codec{};
    mavlink_msg_heartbeat_pack_status(1,1,&codec,&message,2,3,airborne?128:0,4,MAV_STATE_ACTIVE);host.message(message,at);
    mavlink_msg_global_position_int_pack_status(1,1,&codec,&message,0,420000000,-710000000,15000,5000,0,0,0,9000);host.message(message,at);
    mavlink_msg_extended_sys_state_pack_status(1,1,&codec,&message,0,airborne?2:1);host.message(message,at);
    mavlink_home_position_t home{};home.latitude=420000000;home.longitude=-710000000;home.altitude=10000;
    mavlink_msg_home_position_encode_status(1,1,&codec,&message,&home);host.message(message,at);
    mavlink_ekf_status_report_t ekf{};ekf.flags=51;ekf.pos_horiz_variance=ekf.pos_vert_variance=0;
    mavlink_msg_ekf_status_report_encode_status(1,1,&codec,&message,&ekf);host.message(message,at);
  }
  std::vector<unsigned> protocol() const {
    std::vector<unsigned> ids;
    for(const auto& message:sent)
      if(message.msgid==MAVLINK_MSG_ID_PARAM_REQUEST_READ || message.msgid==MAVLINK_MSG_ID_MISSION_REQUEST_LIST ||
         message.msgid==MAVLINK_MSG_ID_MISSION_REQUEST_INT || message.msgid==MAVLINK_MSG_ID_MISSION_COUNT ||
         message.msgid==MAVLINK_MSG_ID_MISSION_ITEM_INT || message.msgid==MAVLINK_MSG_ID_MISSION_CLEAR_ALL ||
         message.msgid==MAVLINK_MSG_ID_MISSION_ACK) ids.push_back(message.msgid);
    return ids;
  }
};
MissionRequest request(MissionAction action=MissionAction::RequestList,std::string id="phase-1") {
  MissionRequest result;result.id=std::move(id);result.operation_id="operation-1";result.action=action;
  result.expires=101;result.operation_expires=110;return result;
}
CommandRequest arm() {
  CommandRequest result;result.id="native-arm";result.command=MAV_CMD_COMPONENT_ARM_DISARM;
  result.params[0]=1;result.expires=101;result.operation_expires=105;return result;
}
}
int main(int argc,char** argv) {
  try {
    check(argc==2,"usage: mission_runtime_tests CONTROL_MODULE");const std::string control=argv[1];unsigned passed=0;
    auto test=[&](const char* name,const auto& run){run();++passed;std::cout<<"PASS "<<name<<'\n';};
    test("read protocol works without control provider or flight prerequisites",[&]{
      Rig r;check(r.host.mission(request()),"mission read admission");r.host.tick(100.1);
      check(r.protocol()==std::vector<unsigned>{43},"read wire type");
      mavlink_mission_request_list_t decoded{};
      for(const auto& m:r.sent)if(m.msgid==43)mavlink_msg_mission_request_list_decode(&m,&decoded);
      check(decoded.target_system==1 && decoded.target_component==1 && decoded.mission_type==0,"fixed mission route");
      check(!r.host.navigationGranted(),"read acquired navigation");
    });
    test("receive-only and shadow deny every mission effect",[&]{
      for(bool shadow:{false,true}) {Rig r({},false,shadow);check(!r.host.mission(request()),"non-effectful mode admitted protocol");r.host.tick(100.1);check(r.sent.empty(),"non-effectful mode transmitted");}
    });
    test("mission mutation fences native commands and acquisition until terminal",[&]{
      Rig r(control);check(r.host.mission(request(MissionAction::Count)),"count admission");
      check(!r.host.sendCommand(arm()),"native command bypassed mutation lease");
      r.observations(100.01,true);check(!r.host.acquireNavigation("during-upload"),"acquire bypassed mutation lease");
      check(!r.host.controlBusy(),"mission client self-inhibited on its own lease");
      r.host.missionFinished("operation-1");check(r.host.acquireNavigation("after-upload"),"terminal did not release mutation lease");
      r.host.tick(100.02);check(r.protocol().empty(),"retired count leaked after terminal");
    });
    test("pending native transition inhibits mutation but not reads",[&]{
      Rig r(control);check(r.host.sendCommand(arm()),"arm queue admission");
      check(!r.host.mission(request(MissionAction::Count)),"mutation admitted during pending ARM");
      check(r.host.mission(request()),"read incorrectly requires control idleness");
    });
    test("external arming rejects waiting mutation while readback remains possible",[&]{
      Rig r;check(r.host.mission(request(MissionAction::Count)),"initial count");r.observations(100.01,true);r.host.tick(100.02);
      check(r.protocol().empty(),"queued mutation sent after external arming");
      auto read=request(MissionAction::RequestItem,"readback");check(r.host.mission(read),"post-arm readback denied");r.host.tick(100.03);
      check(r.protocol()==std::vector<unsigned>{51},"readback did not use INT request");
    });
    test("phase and operation expiry cannot be renewed under an active operation",[&]{
      Rig r;auto first=request();check(r.host.mission(first),"initial phase");
      auto later=request(MissionAction::RequestItem,"later");later.operation_expires=111;
      check(!r.host.mission(later),"operation deadline renewed");
      later.operation_expires=110;later.expires=112;check(!r.host.mission(later),"phase outlived operation");
      later.expires=103;check(!r.host.mission(later),"unbounded local phase lifetime");
      later.expires=100;check(!r.host.mission(later),"expired phase admitted");
    });
    test("bounded queue and untagged remote protocol serialize operations",[&]{
      Rig r;for(unsigned i=0;i<4;++i)check(r.host.mission(request(MissionAction::RequestItem,"phase-"+std::to_string(i))),"bounded phase admission");
      check(!r.host.mission(request(MissionAction::RequestItem,"overflow")),"queue overflow");
      Rig other;check(other.host.mission(request()),"first operation");auto second=request();second.id="other-phase";second.operation_id="other-operation";
      check(!other.host.mission(second),"ambiguous concurrent mission operation admitted");
    });
    test("wire fields cannot disguise execution or clear another plan family",[&]{
      Rig r;auto item=request(MissionAction::Item);item.item.current=2;
      check(!r.host.mission(item),"Guided current=2 escaped motion authority");
      item.item.current=0;item.item.params[0]=NAN;check(!r.host.mission(item),"nonfinite mission payload");
      item=request(MissionAction::Clear);item.mission_type=MAV_MISSION_TYPE_ALL;check(!r.host.mission(item),"clear-all families bypassed profile");
      item=request(MissionAction::Item);item.item.x=INT32_MAX;check(!r.host.mission(item),"invalid positional latitude");
    });
    test("typed inspection binds the host operation and permits only typed read phases",[&]{
      for(const uint8_t type:{MAV_MISSION_TYPE_FENCE,MAV_MISSION_TYPE_RALLY}) {
        Rig r;
        auto item=request(MissionAction::RequestItem,"item");item.mission_type=type;item.seq=3;
        auto ack=request(MissionAction::Ack,"ack");ack.mission_type=type;
        check(!r.host.mission(item)&&!r.host.mission(ack),"typed phases opened an unbound operation");
        auto list=request();list.mission_type=type;check(r.host.mission(list),"typed list denied");
        auto forged=ack;forged.mission_type=MAV_MISSION_TYPE_MISSION;
        check(!r.host.mission(forged),"same-ID ACK crossed type");
        forged=ack;forged.result=MAV_MISSION_OPERATION_CANCELLED;
        check(!r.host.mission(forged),"typed read permitted unsolicited abort ACK");
        auto mutation=request(MissionAction::Count,"mutate");
        check(!r.host.mission(mutation),"typed operation became a flight mutation");
        for(const auto action:{MissionAction::Count,MissionAction::Item,MissionAction::Clear}) {
          mutation.action=action;mutation.mission_type=type;
          check(!r.host.mission(mutation),"typed storage mutation escaped");
        }
        forged=item;forged.operation_expires+=1;check(!r.host.mission(forged),"typed deadline renewed");
        check(r.host.mission(item)&&r.host.mission(ack),"typed phases rejected");
        for(unsigned i=1;i<=3;++i)r.host.tick(100+i*.1);
        check(r.protocol()==std::vector<unsigned>({43,51,47}),"unexpected typed wire message");
        mavlink_mission_request_list_t list_wire{};
        mavlink_mission_request_int_t item_wire{};
        mavlink_mission_ack_t ack_wire{};
        for(const auto& frame:r.sent) {
          if(frame.msgid==MAVLINK_MSG_ID_HEARTBEAT)continue;
          check(frame.magic==MAVLINK_STX&&frame.sysid==245&&frame.compid==191,"wrong typed sender/version");
          if(frame.msgid==43)mavlink_msg_mission_request_list_decode(&frame,&list_wire);
          if(frame.msgid==51)mavlink_msg_mission_request_int_decode(&frame,&item_wire);
          if(frame.msgid==47)mavlink_msg_mission_ack_decode(&frame,&ack_wire);
        }
        check(list_wire.target_system==1&&list_wire.target_component==1&&list_wire.mission_type==type&&
              item_wire.target_system==1&&item_wire.target_component==1&&item_wire.mission_type==type&&item_wire.seq==3&&
              ack_wire.target_system==1&&ack_wire.target_component==1&&ack_wire.mission_type==type&&ack_wire.type==0,
              "wrong typed wire binding");
      }
      Rig flight;
      check(flight.host.mission(request(MissionAction::Count)),"legacy mutation denied");
      auto forged=request(MissionAction::Ack,"foreign-ack");forged.mission_type=MAV_MISSION_TYPE_FENCE;
      check(!flight.host.mission(forged)&&!flight.host.recordStoredMissionProof(forged.operation_id,{}),
            "typed ACK crossed flight storage proof boundary");
      Rig flight_read;
      check(flight_read.host.mission(request())&&!flight_read.host.mission(request(MissionAction::Count,"switch")),
            "flight read escalated into a storage mutation");
    });
    test("fixed item encoding and waiting cancellation",[&]{
      Rig r;auto item=request(MissionAction::Item);item.item.seq=3;item.item.x=420000001;item.item.y=-710000001;item.item.z=7;
      item.item.params={1,2,3,4};check(r.host.mission(item),"item admission");r.host.tick(100.1);
      check(r.protocol()==std::vector<unsigned>{73},"item encoded wrong MAVLink message");
      mavlink_mission_item_int_t decoded{};for(const auto& m:r.sent)if(m.msgid==73)mavlink_msg_mission_item_int_decode(&m,&decoded);
      check(decoded.seq==3 && decoded.x==item.item.x && decoded.y==item.item.y && decoded.z==7 && decoded.param4==4 && decoded.target_system==1 && decoded.target_component==1,"item wire mismatch");
      auto clear=request(MissionAction::Clear,"clear");check(r.host.mission(clear),"clear admission");r.host.missionFinished(clear.operation_id);r.host.tick(100.2);
      check(r.protocol().size()==1,"cancelled waiting clear leaked");
    });
    test("one protocol packet per tick fairly alternates missions and parameters",[&]{
      Rig r;for(unsigned i=0;i<3;++i) {
        auto m=request(MissionAction::RequestItem,"mission-"+std::to_string(i));check(r.host.mission(m),"mission queue");
        ParameterRequest p;p.id="parameter-"+std::to_string(i);p.name="FRAME_CLASS";p.expires=101;check(r.host.parameter(p),"parameter queue");
      }
      for(unsigned i=1;i<=6;++i) {r.host.tick(100+i*.1);check(r.protocol().size()==i,"protocol budget changed");}
      check(r.protocol()==std::vector<unsigned>({20,51,20,51,20,51}),"protocol lane starved a feature");
    });
    test("active navigation keeps priority and mutation stays inhibited",[&]{
      Rig r(control);r.observations(100.01,true);check(r.host.acquireNavigation("helm"),"navigation acquire");
      check(r.host.navigation({90,1,5,101}),"navigation target");check(r.host.mission(request()),"mission read while navigating");
      check(!r.host.mission(request(MissionAction::Count,"mutate")),"flight mutation admitted");r.host.tick(100.02);
      // A ready observation setup may take this first bulk slot. Mission work
      // must follow within one arbitration cycle while NAV retains priority.
      r.host.tick(100.04);r.host.tick(100.06);
      std::vector<unsigned> effects;for(const auto& m:r.sent)if(m.msgid==84||m.msgid==43)effects.push_back(m.msgid);
      check(effects==std::vector<unsigned>({84,43}) && r.host.navigationGranted(),"mission read preempted navigation");
    });
    test("final sampled clock rejects expiry after encoding",[&]{
      double clock=99.97;Rig r({},true,false,[&]{clock+=.03;return clock;});
      auto phase=request();phase.expires=100.05;check(r.host.mission(phase),"short phase admission");r.host.tick(100.02);
      check(r.protocol().empty(),"encoded mission packet escaped original expiry");
    });
    test("reset clears protocol work and mutation leases",[&]{
      Rig r(control);check(r.host.mission(request(MissionAction::Count)),"initial mutation");r.host.disconnect(100.1);
      r.observations(100.2,false);check(r.host.sendCommand(arm()),"reset retained old mutation lease");r.host.tick(100.21);
      check(r.protocol().empty(),"old mission phase survived reset");
    });
    test("retiring a mission at final handoff prevents a dequeued clear",[&]{
      Runtime* host=nullptr;unsigned calls=0;bool cancel=false;
      Rig r({},true,false,[&]{if(cancel&&++calls==2)host->missionFinished("operation-1");return 100.;});
      host=&r.host;r.host.tick(100);r.sent.clear();
      check(r.host.mission(request(MissionAction::Clear)),"clear handoff admission");
      calls=0;cancel=true;r.host.tick(100);
      check(calls>=2&&r.protocol().empty(),"retired dequeued CLEAR still sent");
    });
    test("waiting command cancellation preserves its uncertainty lease",[&]{
      Rig r(control);check(r.host.sendCommand(arm()),"command queue");r.host.cancelCommand("native-arm");
      check(r.host.controlBusy(),"waiting cancellation retired uncertainty lease");r.host.tick(100.1);
      for(const auto& m:r.sent)if(m.msgid==76) {mavlink_command_long_t c{};mavlink_msg_command_long_decode(&m,&c);check(c.command!=MAV_CMD_COMPONENT_ARM_DISARM,"cancelled ARM emitted");}
      r.host.commandFinished("native-arm");check(!r.host.controlBusy(),"explicit terminal did not retire command lease");
    });
    std::cout<<"PASS "<<passed<<" mission host and cancellation groups; fake I/O only\n";return 0;
  } catch(const std::exception& error) {std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
