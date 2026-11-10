/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ProtocolFaultTests.cpp                           */
/*    DATE: November 4, 2026                                */
/************************************************************/
// Actual Runtime/loaded providers; fake clock/I/O, no socket or FC.
#include "ProtocolSupport.h"
#include <functional>
#include <iostream>
#include <set>
using namespace protocol_faults;
namespace {
std::string parameters,missions,control;
struct Rig {
  double time=100;bool airborne=false,fail_parameter=false;unsigned mode=4;
  std::vector<Publication> publications;std::vector<Frame> frames;
  std::vector<std::pair<mavlink_message_t,double>> receipts;
  std::function<void(const mavlink_message_t&)> proof;
  mavlink_status_t codec{};Runtime host;
  explicit Rig(bool with_control=false,bool simulation=true,bool shadow=false):host([&](const auto& key,const auto& text,double,bool numeric){if(!numeric)publications.push_back({key,text,time});},
      [](const uint8_t*,std::size_t){throw std::runtime_error("unbounded sender unexpectedly used");return false;},simulation,shadow,[&]{return time;})
  {
    require(host.load(parameters)&&host.load(missions),"load protocol providers");
    if(with_control){require(host.load(control),"load control provider");host.configure("control_helm","false");}
    host.configure("parameter_timeout","1");host.configure("mission_timeout","3");host.configure("mission_response_timeout","0.25");
    host.setBoundedSend([&](const uint8_t* bytes,std::size_t size,double expires,const Runtime::SendEligibility& live){
      const auto m=decode(bytes,size);if(proof)proof(m);const bool accepted=live(time)&&time<expires&&!(fail_parameter&&(m.msgid==20||m.msgid==23));
      frames.push_back({m,time,expires,accepted});return accepted;
    });time+=.001;fresh();
  }
  void fresh(){mavlink_message_t m{};mavlink_msg_heartbeat_pack_status(1,1,&codec,&m,2,3,airborne?128:0,mode,4);host.message(m,time);
    mavlink_msg_global_position_int_pack_status(1,1,&codec,&m,1000,423584560,-710875890,15000,5000,0,0,0,9000);host.message(m,time);
    mavlink_msg_extended_sys_state_pack_status(1,1,&codec,&m,0,airborne?2:1);host.message(m,time);
    mavlink_home_position_t h{};h.latitude=423584560;h.longitude=-710875890;h.altitude=10000;mavlink_msg_home_position_encode_status(1,1,&codec,&m,&h);host.message(m,time);
    mavlink_ekf_status_report_t e{};e.flags=51;mavlink_msg_ekf_status_report_encode_status(1,1,&codec,&m,&e);host.message(m,time);}
  void tick(double delta=.02){time+=delta;fresh();host.tick(time);}
  void pump(const std::function<bool()>& done,double seconds=2,double interval=.02){const double until=time+seconds;while(!done()&&time<until)tick(interval);require(done(),"bounded fixture condition not reached");}
  void mail(const std::string& key,const std::string& text){time+=.001;fresh();Mail m;m.key=key;m.text=text;m.source="protocol_fault_fixture";m.published_at=m.received_at=time;host.mail(m);}
  void param(const std::string& id,const std::string& action="read",const std::string& tail=",name=FRAME_CLASS"){mail("MAVMOOS_PARAM_REQUEST","id="+id+",action="+action+tail);}
  void mission(const std::string& id,const std::string& action="download",const std::string& tail=""){mail("MAVMOOS_MISSION_REQUEST","id="+id+",action="+action+tail);}
  unsigned count(unsigned id,bool accepted=true)const{return unsigned(std::count_if(frames.begin(),frames.end(),[&](const auto& f){return f.message.msgid==id&&(!accepted||f.accepted);}));}
  unsigned commands(unsigned id)const{return unsigned(std::count_if(frames.begin(),frames.end(),[&](const auto& f){return f.accepted&&command(f.message)==id;}));}
  bool outcome(const std::string& id,const std::string& status)const{for(const auto& p:publications){auto f=fields(p.text);if(f["id"]==id&&f["status"]==status)return true;}return false;}
  std::map<std::string,std::string> result(const std::string& id)const{for(auto p=publications.rbegin();p!=publications.rend();++p){auto f=fields(p->text);if(f["id"]==id&&f.count("status"))return f;}throw std::runtime_error("missing result");}
  std::string last(const std::string& key)const{for(auto p=publications.rbegin();p!=publications.rend();++p)if(p->key==key)return p->text;return {};}
  void input(mavlink_message_t m){time+=.001;fresh();receipts.emplace_back(m,time);host.message(m,time);}
  mavlink_message_t valueMessage(const std::string& name,float value=1,unsigned idx=UINT16_MAX,unsigned total=4,unsigned sys=1){mavlink_param_value_t p{};std::copy(name.begin(),name.end(),p.param_id);p.param_value=value;p.param_type=MAV_PARAM_TYPE_REAL32;p.param_index=uint16_t(idx);p.param_count=uint16_t(total);mavlink_message_t m{};mavlink_msg_param_value_encode_status(uint8_t(sys),1,&codec,&m,&p);return m;}
  void value(const std::string& name,float value=1,unsigned idx=UINT16_MAX,unsigned total=4,unsigned sys=1){input(valueMessage(name,value,idx,total,sys));}
  void missionCount(unsigned n,unsigned target=245){mavlink_mission_count_t p{};p.target_system=uint8_t(target);p.target_component=191;p.count=uint16_t(n);mavlink_message_t m{};mavlink_msg_mission_count_encode_status(1,1,&codec,&m,&p);input(m);}
  void missionRequest(unsigned seq){mavlink_mission_request_int_t p{};p.target_system=245;p.target_component=191;p.seq=uint16_t(seq);mavlink_message_t m{};mavlink_msg_mission_request_int_encode_status(1,1,&codec,&m,&p);input(m);}
  void item(MissionItem item){mavlink_mission_item_int_t p{};p.target_system=245;p.target_component=191;p.seq=item.seq;p.command=item.command;p.frame=item.frame;p.current=item.current;p.autocontinue=item.autocontinue;p.x=item.x;p.y=item.y;p.z=item.z;mavlink_message_t m{};mavlink_msg_mission_item_int_encode_status(1,1,&codec,&m,&p);input(m);}
  void missionAck(unsigned result=MAV_MISSION_ACCEPTED){mavlink_mission_ack_t p{};p.target_system=245;p.target_component=191;p.type=uint8_t(result);mavlink_message_t m{};mavlink_msg_mission_ack_encode_status(1,1,&codec,&m,&p);input(m);}
  void commandAck(unsigned result=MAV_RESULT_ACCEPTED){mavlink_command_ack_t p{};p.target_system=245;p.target_component=191;p.command=MAV_CMD_DO_SET_MODE;p.result=uint8_t(result);mavlink_message_t m{};mavlink_msg_command_ack_encode_status(1,1,&codec,&m,&p);input(m);}
  void throughUpload(const std::string& id){mission(id,"upload",",items="+rows());pump([&]{return count(44)==1;});missionRequest(0);pump([&]{return count(73)==1;});missionRequest(1);pump([&]{return count(73)==2;});}
  void emit(const std::string& name,const std::string& extra="")const{
    std::cout<<"{\"type\":\"case\",\"name\":"<<quote(name)<<",\"result\":\"PASS\",\"scope\":\"actual Runtime/MODULEs with fake clock and I/O; no socket/FC\",\"runtime\":"<<quote(host.status())<<",\"publications\":[";
    bool comma=false;for(const auto& p:publications){if(comma)std::cout<<',';comma=true;std::cout<<publicationJson(p);}std::cout<<"],\"frames\":[";comma=false;
    for(const auto& f:frames){if(comma)std::cout<<',';comma=true;std::cout<<frameJson(f);}std::cout<<"],\"receipts\":[";comma=false;
    for(const auto& r:receipts){if(comma)std::cout<<',';comma=true;std::cout<<"{\"msgid\":"<<r.first.msgid<<",\"wire_hex\":"<<quote(wire(r.first))<<",\"effective_at\":"<<decimal(r.second)<<",\"read_at\":"<<decimal(r.second)<<'}';}std::cout<<']'<<extra<<"}\n";
  }
};
}
int main(int argc,char** argv){try{
  require(argc==4,"usage: protocol_fault_tests PARAMETERS MISSIONS CONTROL");parameters=argv[1];missions=argv[2];control=argv[3];unsigned passed=0;
  auto test=[&](const std::string& name,const auto& fn){Rig r;fn(r);r.emit(name);++passed;};
  test("named_read_loss_original_deadline",[](Rig& r){r.param("lost");r.pump([&]{return r.count(20)==1;});r.pump([&]{return r.outcome("lost","TIMED_OUT");});require(!r.outcome("lost","COMPLETED")&&r.count(20)==1,"lost named read retried/completed");});
  test("write_echo_loss_retains_uncertainty",[](Rig& r){r.param("lost-echo","write",",name=SIM_WIND_SPD,value=0.25,type=REAL32");r.pump([&]{return r.count(23)==1;});r.pump([&]{return r.outcome("lost-echo","TIMED_OUT");});auto f=r.result("lost-echo");require(f["write_may_have_applied"]=="true"&&f["readback_confirmed"]=="false"&&r.count(23)==1&&r.count(20)==0,"lost echo lost uncertainty or retried SET");});
  test("write_readback_loss_never_replays_set",[](Rig& r){r.param("lost-readback","write",",name=SIM_WIND_SPD,value=0.25,type=REAL32");r.pump([&]{return r.count(23)==1;});r.value("SIM_WIND_SPD",.25f);r.pump([&]{return r.count(20)==1;});r.pump([&]{return r.outcome("lost-readback","TIMED_OUT");});auto f=r.result("lost-readback");require(f["write_may_have_applied"]=="true"&&f["readback_confirmed"]=="false"&&r.count(23)==1,"lost readback became confirmed/replayed");});
  test("write_mismatch_fails_without_retry",[](Rig& r){r.param("mismatch","write",",name=SIM_WIND_SPD,value=0.25,type=REAL32");r.pump([&]{return r.count(23)==1;});r.value("SIM_WIND_SPD",.5f);require(r.outcome("mismatch","FAILED")&&!r.outcome("mismatch","COMPLETED")&&r.count(23)==1,"mismatched echo accepted");});
  test("foreign_and_wrong_name_reply_cannot_complete_read",[](Rig& r){r.param("named");r.pump([&]{return r.count(20)==1;});r.value("FRAME_CLASS",1,UINT16_MAX,4,2);r.value("OTHER",1);require(!r.outcome("named","COMPLETED"),"unrelated reply completed");r.value("FRAME_CLASS",1);require(r.outcome("named","COMPLETED"),"correct read never completed");});
  test("paced_inventory_reorder_duplicate_and_unrequested",[](Rig& r){r.param("table","list","");r.pump([&]{return r.count(20)==1;});r.value("P0",0,0,4);r.pump([&]{std::set<unsigned> x;for(const auto& f:r.frames)if(f.accepted&&f.message.msgid==20)x.insert(index(f.message));return x.size()==4;});r.value("UNREQUESTED",42,42,43);r.value("P3",3,3,4);r.value("P3",3,3,4);r.value("P1",1,1,4);require(!r.outcome("table","COMPLETED"),"duplicate inflated completed inventory");r.value("P2",2,2,4);require(r.outcome("table","COMPLETED")&&r.count(20)==4&&r.last("MAVMOOS_PARAM_SNAPSHOT").find("\"count\":4")!=std::string::npos,"out-of-order table failed");});
  test("paced_inventory_changed_count_is_incomplete",[](Rig& r){r.param("shape","list","");r.pump([&]{return r.count(20)==1;});r.value("P0",0,0,3);r.pump([&]{return r.count(20)>=2;});r.value("P1",1,1,4);require(r.outcome("shape","INCOMPLETE")&&!r.outcome("shape","COMPLETED"),"changed inventory count accepted");});
  test("reset_during_parameter_readback_handoff_fences_old_work",[](Rig& r){r.param("reset-write","write",",name=SIM_WIND_SPD,value=0.25,type=REAL32");r.pump([&]{return r.count(23)==1;});r.value("SIM_WIND_SPD",.25f);bool reset=false;r.proof=[&](const auto& m){if(m.msgid==20&&!reset){reset=true;r.host.disconnect(r.time);r.fresh();}};r.pump([&]{return reset;});r.value("SIM_WIND_SPD",.25f);require(r.outcome("reset-write","CANCELLED")&&!r.outcome("reset-write","COMPLETED")&&r.count(20)==0,"old readback crossed new session");r.param("reset-write","write",",name=SIM_WIND_SPD,value=0.25,type=REAL32");require(r.outcome("reset-write","DUPLICATE"),"reset allowed old ID replay");});
  test("mission_count_loss_has_bounded_recovery",[](Rig& r){r.mission("no-count");r.pump([&]{return r.outcome("no-count","TIMED_OUT");},4);require(r.count(43)>=2&&r.count(43)<=6&&!r.outcome("no-count","COMPLETED"),"mission count recovery unbounded or false completion");});
  test("mission_reordered_future_item_does_not_advance",[](Rig& r){r.mission("order");r.pump([&]{return r.count(43)==1;});r.missionCount(2);r.pump([&]{return r.count(51)==1;});r.item(waypoint());require(!r.outcome("order","COMPLETED")&&r.count(47)==0,"future mission item advanced operation");r.item(home());r.pump([&]{return r.count(51)==2;});r.item(home());r.item(waypoint());r.pump([&]{return r.outcome("order","COMPLETED");});require(r.count(47)==1,"read completion ACK missing/duplicated");});
  test("mission_final_ack_loss_retains_mutation_uncertainty",[](Rig& r){r.throughUpload("ack-loss");r.pump([&]{return r.outcome("ack-loss","TIMED_OUT");},4);auto f=r.result("ack-loss");require(f["mutation_may_have_applied"]=="true"&&f["remote_accepted"]=="false"&&f["readback_confirmed"]=="false"&&!r.outcome("ack-loss","COMPLETED"),"lost upload ACK became confirmed success");require(r.count(73)<=7&&r.count(43)==0,"upload ACK loss recovery unbounded/readback assumed");});
  test("mission_cancel_during_readback_keeps_remote_acceptance",[](Rig& r){r.throughUpload("cancel-readback");r.missionAck();r.pump([&]{return r.count(43)==1;});r.mission("cancel-readback","cancel");r.missionCount(2);r.tick();auto f=r.result("cancel-readback");require(f["status"]=="CANCELLED"&&f["remote_accepted"]=="true"&&f["mutation_may_have_applied"]=="true"&&f["readback_confirmed"]=="false"&&r.count(51)==0,"cancelled readback resumed/forgot remote mutation");});
  test("mission_readback_cancel_at_encoded_handoff",[](Rig& r){r.throughUpload("cancel-final");r.missionAck();bool cancelled=false;r.proof=[&](const auto& m){if(m.msgid==43&&!cancelled){cancelled=true;r.mission("cancel-final","cancel");}};r.pump([&]{return cancelled;});require(r.count(43)==0&&r.outcome("cancel-final","CANCELLED"),"encoded cancelled readback reached I/O");});
  test("mission_readback_mismatch_is_not_verified",[](Rig& r){r.throughUpload("bad-plan");r.missionAck();r.pump([&]{return r.count(43)==1;});r.missionCount(2);r.pump([&]{return r.count(51)==1;});r.item(home());r.pump([&]{return r.count(51)==2;});auto x=waypoint();x.z=7;r.item(x);r.pump([&]{return r.outcome("bad-plan","FAILED");});require(r.result("bad-plan")["readback_confirmed"]=="false"&&!r.outcome("bad-plan","COMPLETED"),"changed stored mission verified");});
  test("mission_reset_rejects_old_phase_and_id_replay",[](Rig& r){r.mission("old");r.pump([&]{return r.count(43)==1;});r.host.disconnect(r.time);r.fresh();r.missionCount(2);r.tick();require(r.outcome("old","CANCELLED")&&r.count(51)==0,"old count revived reset phase");r.mission("old");require(r.outcome("old","DUPLICATE"),"old mission ID replay accepted");});
  {Rig r; r.param("old-read");r.pump([&]{return r.count(20)==1;});auto old_reply=r.valueMessage("FRAME_CLASS",1);r.pump([&]{return r.outcome("old-read","TIMED_OUT");});r.param("new-read");r.pump([&]{return r.count(20)==2;});r.input(old_reply);require(r.outcome("new-read","COMPLETED"),"documented untagged same-name ambiguity changed");r.emit("untagged_same_name_reply_causality_is_unproven",",\"ambiguity_observed\":true,\"remote_causality_proven\":false,\"wire_operation_id_present\":false");++passed;}
  {Rig r(true);r.airborne=true;r.fresh();require(r.host.acquireNavigation("nav"),"initial NAV grant");r.param("nav-loss");const auto before=r.frames.size();for(unsigned i=0;i<75;++i){r.time+=.02;r.fresh();require(r.host.navigation({90,1,5,r.time+.2}),"NAV proposal");r.host.tick(r.time);}require(r.outcome("nav-loss","TIMED_OUT")&&r.host.navigationGranted()&&r.count(84)>=10,"read timeout preempted NAV");for(std::size_t i=before;i<r.frames.size();++i)require(r.frames[i].at<r.frames[i].expires||!r.frames[i].accepted,"accepted expired frame");r.emit("protocol_timeout_preserves_independent_navigation");++passed;}
  {Rig r(true);r.host.configure("command_ack_timeout","0.3");r.airborne=true;r.fresh();require(r.host.acquireNavigation("nav"),"command fixture grant");r.mail("MAVMOOS_COMMAND","id=hold-lost,action=hold");r.pump([&]{return r.commands(MAV_CMD_DO_SET_MODE)==1;});require(!r.host.navigationGranted()&&r.host.controlBusy(),"command did not retire NAV/retain lease");r.pump([&]{return r.outcome("hold-lost","TIMED_OUT");},9,.2);r.mode=5;r.commandAck();r.tick();require(!r.outcome("hold-lost","COMPLETED")&&!r.host.navigationGranted()&&!r.host.controlBusy(),"late ACK/state revived expired command or NAV");r.emit("missing_command_ack_and_state_times_out_without_reacquisition");++passed;}
  {Rig r(true);r.host.configure("command_ack_timeout","0.3");r.airborne=true;r.fresh();r.mail("MAVMOOS_COMMAND","id=hold-observed,action=hold");r.pump([&]{return r.commands(MAV_CMD_DO_SET_MODE)==1;});r.mode=5;r.tick();require(r.outcome("hold-observed","COMPLETED")&&!r.outcome("hold-observed","ACK_OBSERVED")&&!r.host.navigationGranted(),"fresh observed mode required an ACK or acquired NAV");r.emit("lost_command_ack_can_complete_on_fresh_observed_outcome");++passed;}
  {Rig r(true);r.host.configure("command_ack_timeout","0.3");r.airborne=true;r.fresh();r.mail("MAVMOOS_COMMAND","id=hold-no-state,action=hold");r.pump([&]{return r.commands(MAV_CMD_DO_SET_MODE)==1;});r.commandAck();r.tick(.4);require(r.outcome("hold-no-state","ACK_OBSERVED")&&!r.outcome("hold-no-state","COMPLETED")&&r.host.controlBusy(),"ACK alone completed native operation");r.pump([&]{return r.outcome("hold-no-state","TIMED_OUT");},9,.2);require(r.commands(MAV_CMD_DO_SET_MODE)==1&&!r.outcome("hold-no-state","COMPLETED")&&!r.host.navigationGranted(),"ACK-only operation retried/completed/reacquired");r.emit("command_ack_alone_does_not_prove_observed_completion");++passed;}
  for(bool shadow:{false,true}){Rig r(false,false,shadow);r.param("disabled-param");r.mission("disabled-mission");r.tick();require(r.frames.empty()&&!r.host.navigationGranted(),"disabled mode emitted protocol effects");r.emit(shadow?"shadow_protocol_effects_denied":"receive_only_protocol_effects_denied");++passed;}
  std::cout<<"{\"type\":\"summary\",\"result\":\"PASS\",\"groups\":"<<passed<<",\"scope\":\"fake clock/I/O only; no network or vehicle execution\"}\n";return 0;
}catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}}
