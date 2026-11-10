// Actual mission provider with fake host I/O. No socket, MOOSDB or FC is opened.
#include "Plugin.h"
#include <algorithm>
#include <cmath>
#include <dlfcn.h>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>
using namespace mavmoos::runtime;
extern "C" Plugin* mavmoos_create_plugin(unsigned,Services*);
extern "C" void mavmoos_destroy_plugin(Plugin*);
namespace {
void require(bool condition,const std::string& text){if(!condition)throw std::runtime_error(text);}
CreatePlugin create=mavmoos_create_plugin;DestroyPlugin destroy=mavmoos_destroy_plugin;
struct Host final:Services {
  double clock=100;Vehicle state;bool accept=true,nav=false,busy=false;
  std::vector<MissionRequest> requests;
  std::vector<std::pair<std::string,std::string>> records;
  std::vector<std::string> finished;
  unsigned control_effects=0;
  Host(){fresh();ground();}
  void fresh(){state.heartbeat_valid=true;state.heartbeat_at=clock;state.landed_at=clock;}
  void ground(){state.type=2;state.autopilot=3;state.armed=false;state.landed_valid=true;state.landed_state=1;state.landed_at=clock;}
  double now()const override{return clock;}
  const Vehicle& vehicle()const override{return state;}
  void publish(const std::string& key,const std::string& value)override{records.emplace_back(key,value);}
  void publish(const std::string&,double)override{}
  bool sendCommand(const CommandRequest&)override{++control_effects;return false;}
  bool acquireNavigation(const std::string&)override{++control_effects;return false;}
  void revokeNavigation(const std::string&)override{++control_effects;}
  bool navigation(const NavigationTarget&)override{++control_effects;return false;}
  bool navigationGranted()const override{return nav;}
  bool controlBusy()const override{return busy;}
  bool mission(const MissionRequest& request)override{if(!accept)return false;requests.push_back(request);return true;}
  void missionFinished(const std::string& id)override{finished.push_back(id);}
  bool has(const std::string& text)const{return std::any_of(records.begin(),records.end(),[&](const auto& r){return r.second.find(text)!=std::string::npos;});}
  unsigned count(const std::string& variable)const{return unsigned(std::count_if(records.begin(),records.end(),[&](const auto& r){return r.first==variable;}));}
  std::string last(const std::string& variable)const{for(auto i=records.rbegin();i!=records.rend();++i)if(i->first==variable)return i->second;return {};}
};
MissionItem home(){MissionItem x;x.seq=0;x.frame=MAV_FRAME_GLOBAL;x.x=423584560;x.y=-710875890;x.z=10;return x;}
MissionItem waypoint(unsigned seq=1){MissionItem x;x.seq=uint16_t(seq);x.x=423584560+int32_t(seq)*100;x.y=-710875890;x.z=5;return x;}
std::string rows(const std::vector<MissionItem>& items){
  std::ostringstream out;
  for(unsigned i=0;i<items.size();++i){const auto& x=items[i];if(i)out<<'|';out<<x.seq<<':'<<x.command<<':'<<unsigned(x.frame)<<':'<<unsigned(x.current)<<':'<<unsigned(x.autocontinue);for(auto p:x.params)out<<':'<<p;out<<':'<<x.x<<':'<<x.y<<':'<<x.z;}
  return out.str();
}
struct Rig {
  Host host;Plugin* plugin=create(plugin_abi,&host);
  Rig(){require(plugin,"factory rejected ABI");}
  ~Rig(){destroy(plugin);}
  void advance(double delta=.01,bool refresh=true){host.clock+=delta;if(refresh)host.fresh();plugin->onInputBoundary();plugin->iterate();}
  void submit(const std::string& text){host.clock+=.01;host.fresh();Mail m;m.key="MAVMOOS_MISSION_REQUEST";m.text=text;m.source="operator";m.published_at=m.received_at=host.clock;plugin->onMail(m);plugin->iterate();}
  void sent(bool accepted=true){require(!host.requests.empty(),"no pending request");host.clock+=.001;plugin->onMissionSent(host.requests.back(),host.clock,accepted);}
  void receipt(unsigned index,bool accepted=true){require(index<host.requests.size(),"bad receipt index");host.clock+=.001;plugin->onMissionSent(host.requests[index],host.clock,accepted);}
  void message(mavlink_message_t& msg){host.clock+=.001;plugin->onMessage(msg,host.clock);}
  void count(unsigned count,unsigned target_system=245,unsigned target_component=191,unsigned type=0,uint32_t opaque=0,bool legacy=false){
    mavlink_mission_count_t data{};data.count=uint16_t(count);data.target_system=uint8_t(target_system);data.target_component=uint8_t(target_component);data.mission_type=uint8_t(type);data.opaque_id=opaque;
    mavlink_message_t msg{};mavlink_status_t codec{};
    if(legacy)codec.flags=MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    mavlink_msg_mission_count_encode_status(1,1,&codec,&msg,&data);message(msg);
  }
  void item(MissionItem x,unsigned target_system=245,unsigned type=0){
    mavlink_mission_item_int_t data{};data.target_system=uint8_t(target_system);data.target_component=191;data.mission_type=uint8_t(type);
    data.seq=x.seq;data.command=x.command;data.frame=x.frame;data.current=x.current;data.autocontinue=x.autocontinue;
    data.param1=x.params[0];data.param2=x.params[1];data.param3=x.params[2];data.param4=x.params[3];data.x=x.x;data.y=x.y;data.z=x.z;
    mavlink_message_t msg{};mavlink_msg_mission_item_int_encode(1,1,&msg,&data);message(msg);
  }
  void request(unsigned seq,bool legacy=false,unsigned target_system=245,unsigned type=0){
    mavlink_message_t msg{};
    if(legacy){mavlink_mission_request_t data{};data.seq=uint16_t(seq);data.target_system=uint8_t(target_system);data.target_component=191;data.mission_type=uint8_t(type);mavlink_msg_mission_request_encode(1,1,&msg,&data);}
    else{mavlink_mission_request_int_t data{};data.seq=uint16_t(seq);data.target_system=uint8_t(target_system);data.target_component=191;data.mission_type=uint8_t(type);mavlink_msg_mission_request_int_encode(1,1,&msg,&data);}
    message(msg);
  }
  void ack(unsigned type=MAV_MISSION_ACCEPTED,unsigned target_system=245,unsigned mission_type=0){
    mavlink_mission_ack_t data{};data.type=uint8_t(type);data.target_system=uint8_t(target_system);data.target_component=191;data.mission_type=uint8_t(mission_type);mavlink_message_t msg{};mavlink_msg_mission_ack_encode(1,1,&msg,&data);message(msg);
  }
  void download(const std::vector<MissionItem>& items){
    sent();count(unsigned(items.size()));advance();
    for(const auto& x:items){require(host.requests.back().action==MissionAction::RequestItem&&host.requests.back().seq==x.seq,"wrong indexed request");sent();item(x);advance();}
    require(host.requests.back().action==MissionAction::Ack,"download ACK not queued");
  }
  void uploadThroughAck(const std::vector<MissionItem>& items,bool legacy=false){
    submit("id=upload,action=upload,items="+rows(items));require(host.requests.back().action==MissionAction::Count,"missing upload count");sent();
    for(unsigned seq=0;seq<items.size();++seq){request(seq,legacy);advance();require(host.requests.back().action==MissionAction::Item&&host.requests.back().seq==seq,"wrong upload item");sent();}
    ack();advance();require(host.requests.back().action==MissionAction::RequestList,"accepted upload did not require download");
  }
};
}
int main(int argc,char** argv){
  void* module=nullptr;
  try {
    if(argc==2){module=dlopen(argv[1],RTLD_NOW|RTLD_LOCAL|RTLD_NODELETE);require(module,"dlopen mission provider failed");create=reinterpret_cast<CreatePlugin>(dlsym(module,"mavmoos_create_plugin"));destroy=reinterpret_cast<DestroyPlugin>(dlsym(module,"mavmoos_destroy_plugin"));require(create&&destroy,"module exports missing");}
    unsigned passed=0;auto test=[&](const std::string& name,const std::function<void()>& run){run();++passed;std::cout<<"PASS "<<name<<'\n';};
    test("ABI, independent descriptor and zero constructor effects",[]{
      Host h;require(!create(plugin_abi-1,&h)&&!create(plugin_abi+1,&h)&&!create(plugin_abi,nullptr),"wrong ABI/context accepted");
      auto* p=create(plugin_abi,&h);require(p&&p->name()=="missions"&&!p->controlBackend(),"wrong independent role");
      const auto d=p->descriptor();require(d.profile=="ardupilot_mission_int"&&d.request_owners==std::vector<std::string>{"MAVMOOS_MISSION_REQUEST"}&&d.exclusive_resources==std::vector<std::string>{"protocol.missions"},"bad descriptor");
      require(p->subscriptions()==std::vector<std::string>{"MAVMOOS_MISSION_REQUEST"}&&d.capabilities.size()==8,"wrong supported surface");
      require(h.requests.empty()&&h.control_effects==0,"discovery created effects");destroy(p);
    });
    test("strict malformed requests and unsupported upload operations",[]{
      for(const auto& text:std::vector<std::string>{"","id=x,action=start","id=x,action=download,items=","id=x,action=download,extra=1","id=x,action=download,id=y","id=bad/id,action=download","id=x,action=download,","id=x,action=clear,timeout=nan","id=x,action=clear,timeout=121","id=x,action=upload,items="}) {
        Rig r;r.submit(text);require(r.host.requests.empty()&&r.host.has("status=REJECTED"),"malformed request accepted: "+text);
      }
      for(unsigned bad:{MAV_CMD_DO_SET_SERVO,MAV_CMD_DO_JUMP}){Rig r;auto x=waypoint();x.command=uint16_t(bad);r.submit("id=x,action=upload,items="+rows({home(),x}));require(r.host.requests.empty(),"unsupported item uploaded");}
      for(unsigned field=0;field<5;++field){Rig r;auto x=waypoint();if(field==0)x.current=2;if(field==1)x.frame=MAV_FRAME_LOCAL_NED;if(field==2)x.seq=2;if(field==3)x.params[3]=std::numeric_limits<float>::quiet_NaN();if(field==4)x.x=900000001;r.submit("id=x,action=upload,items="+rows({home(),x}));require(r.host.requests.empty(),"bad upload profile accepted");}
    });
    test("versioned inspection accepts exact read and cancel requests only",[]{
      const auto rejected="MAVMOOS_MISSION_INSPECTION_RESULT";
      for(const auto* text:{"id=x,action=inspect,type=flight","schema=2,id=x,action=inspect,type=flight",
          "schema=1,id=x,action=inspect,type=all","schema=1,id=x,action=inspect,type=1",
          "schema=1,id=x,action=inspect,type=fence,items=","schema=1,id=x,action=clear,type=fence",
          "schema=1,id=x,action=upload,type=rally","schema=1,id=x,action=inspect,type=fence,type=rally",
          "schema=1,id=x,action=cancel,type=fence,timeout=5"}) {
        Rig r;r.submit(text);require(r.host.requests.empty()&&r.host.last(rejected).find("status=REJECTED")!=std::string::npos,
                                     std::string("bad inspection accepted: ")+text);
      }
    });
    test("typed fence collects only exact addressed rows, and emits a matching final ACK",[]{
      Rig r;const auto type=MAV_MISSION_TYPE_FENCE;const auto key="MAVMOOS_MISSION_INSPECTION_FENCE_SNAPSHOT";
      r.submit("schema=1,id=f,action=inspect,type=fence");
      require(r.host.requests.size()==1&&r.host.requests.back().mission_type==type,"list type binding");
      r.count(2,245,191,type);require(!r.host.has("COUNT_RECEIVED"),"pre-send count advanced");
      r.sent();r.count(2,245,191,MAV_MISSION_TYPE_RALLY);r.count(2,245,10,type);r.count(2,241,191,type);
      require(!r.host.has("COUNT_RECEIVED"),"wrong type/recipient count advanced");
      r.count(2,245,191,type,9);r.advance();
      require(r.host.requests.back().action==MissionAction::RequestItem&&r.host.requests.back().seq==0,"wrong item phase");
      MissionItem first=home();first.command=MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;first.z=0;first.params[0]=25;first.autocontinue=0;
      auto second=first;second.seq=1;second.command=MAV_CMD_NAV_FENCE_RETURN_POINT;second.params[0]=0;
      r.sent();r.item(second,245,type);r.item(first,245,MAV_MISSION_TYPE_RALLY);
      require(!r.host.has("ITEM_RECEIVED"),"wrong type/reordered row advanced");
      r.item(first,245,type);r.advance();r.sent();r.item(first,245,type);r.item(second,245,type);r.advance();
      require(r.host.requests.back().action==MissionAction::Ack&&r.host.requests.back().result==MAV_MISSION_ACCEPTED&&
              !r.host.count(key)&&!r.host.has("COMPLETED"),"invalid or premature finish");
      r.sent();
      const auto snapshot=r.host.last(key);
      require(r.host.has("status=COMPLETED")&&snapshot.find("\"mission_type\":1")!=std::string::npos&&
              snapshot.find("\"count\":2")!=std::string::npos&&snapshot.find("\"opaque_id\":9")!=std::string::npos&&
              snapshot.find("\"params\":[25,0,0,0]")!=std::string::npos&&
              r.host.count("MAVMOOS_MISSION_SNAPSHOT")==0,"bad separated fence snapshot");
      for(const auto& q:r.host.requests)require(q.mission_type==type&&q.operation_expires==r.host.requests[0].operation_expires,"phase lost type/deadline");
    });
    test("typed empty, unsupported, and legacy framing stay distinct",[]{
      const auto type=MAV_MISSION_TYPE_RALLY;const auto key="MAVMOOS_MISSION_INSPECTION_RALLY_SNAPSHOT";
      Rig r;r.submit("schema=1,id=empty,action=inspect,type=rally");r.sent();
      r.ack(MAV_MISSION_UNSUPPORTED,245,MAV_MISSION_TYPE_FENCE);
      require(!r.host.has("UNSUPPORTED"),"foreign unsupported ack advanced");
      r.count(0,245,191,type);r.advance();r.sent();
      require(r.host.has("status=COMPLETED")&&r.host.last(key).find("\"count\":0")!=std::string::npos,"empty support missing");
      r.submit("schema=1,id=unsupported,action=inspect,type=rally");r.sent();r.ack(MAV_MISSION_UNSUPPORTED,245,type);
      require(r.host.has("status=UNSUPPORTED")&&r.host.count(key)==1,"unsupported overwrote the last complete snapshot");
      Rig flight;flight.submit("schema=1,id=f,action=inspect,type=flight");flight.sent();flight.count(0,245,191,0,0,true);
      require(!flight.host.has("COUNT_RECEIVED"),"MAVLink 1 advanced versioned inspection");
      flight.count(0);flight.advance();flight.sent();
      require(flight.host.has("status=COMPLETED")&&flight.host.count("MAVMOOS_MISSION_INSPECTION_FLIGHT_SNAPSHOT")==1,"valid MAVLink 2 flight count failed");
    });
    test("typed retries, cancellation and reset respect the original operation and family",[]{
      const auto type=MAV_MISSION_TYPE_RALLY;const auto key="MAVMOOS_MISSION_INSPECTION_RALLY_SNAPSHOT";
      Rig r;r.submit("schema=1,id=r,action=inspect,type=rally,timeout=4");r.sent();
      const auto first=r.host.requests.front();
      r.advance(1.51);r.count(0,245,191,type);r.advance();
      const auto retry=r.host.requests.back();
      require(r.host.has("RETRY_DUE")&&retry.id!=first.id&&retry.mission_type==type&&
              retry.operation_expires==first.operation_expires&&!r.host.has("COUNT_RECEIVED"),"late reply renewed or advanced");
      r.sent();r.submit("schema=1,id=r,action=cancel,type=fence");r.submit("id=r,action=cancel");
      require(!r.host.has("status=CANCELLED")&&r.host.has("operation_not_active"),"foreign cancel retired inspection");
      r.submit("schema=1,id=r,action=cancel,type=rally");r.count(0,245,191,type);r.advance();
      require(r.host.requests.size()==2&&r.host.has("status=CANCELLED")&&!r.host.count(key),"cancel leaked send/snapshot");
      r.submit("schema=1,id=r2,action=inspect,type=rally");r.sent();r.plugin->onReset("connection_reset");
      r.count(0,245,191,type);r.advance();
      require(r.host.last(key).find("\"status\":\"invalidated\"")!=std::string::npos&&
              r.host.count("MAVMOOS_MISSION_INSPECTION_FENCE_SNAPSHOT")==1&&
              r.host.count("MAVMOOS_MISSION_INSPECTION_FLIGHT_SNAPSHOT")==1,"reset missed plan invalidation");
      r.submit("schema=1,id=r2,action=inspect,type=rally");require(r.host.has("status=DUPLICATE"),"reset allowed replay");
      Rig late;late.submit("schema=1,id=late,action=inspect,type=rally,timeout=1");late.sent();late.advance(1.01);
      late.count(0,245,191,type);
      require(late.host.has("original_operation_deadline")&&!late.host.count(key),"operation completed past fixed deadline");
    });
    test("typed plans reject foreign commands or invalid coordinate and shape values",[]{
      for(unsigned fault=0;fault<4;++fault) {
        Rig r;const auto type=fault==3?MAV_MISSION_TYPE_FENCE:MAV_MISSION_TYPE_RALLY;
        r.submit(std::string("schema=1,id=bad,action=inspect,type=")+(fault==3?"fence":"rally"));
        r.sent();r.count(1,245,191,type);r.advance();r.sent();
        MissionItem bad=home();bad.command=fault==0?MAV_CMD_NAV_WAYPOINT:fault==3?MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:MAV_CMD_NAV_RALLY_POINT;
        if(fault==1)bad.x=INT32_MAX;
        if(fault==2)bad.frame=MAV_FRAME_LOCAL_NED;
        if(fault==3)bad.params[0]=2;
        r.item(bad,245,type);r.advance();
        require(r.host.has("invalid_plan_item_shape")&&r.host.requests.size()==2&&!r.host.has("status=COMPLETED"),"malformed typed row published");
      }
    });
    test("retained stale future numeric and oversized requests reject",[]{
      for(unsigned kind=0;kind<5;++kind){Rig r;Mail m;m.key="MAVMOOS_MISSION_REQUEST";m.text="id=x,action=download";m.received_at=100.01;m.published_at=100.01;r.host.clock=100.01;
        if(kind==0)m.published_at=100;if(kind==1)m.published_at=97;if(kind==2)m.published_at=101;if(kind==3)m.numeric=true;if(kind==4)m.text=std::string(1024*1024+1,'x');r.plugin->onMail(m);r.plugin->iterate();require(r.host.requests.empty()&&r.host.has("status=REJECTED"),"invalid metadata accepted");}
    });
    test("download independent of home estimator arming and NAV",[]{Rig r;r.host.state.home_valid=r.host.state.estimator_valid=r.host.state.position_valid=r.host.state.landed_valid=false;r.host.state.armed=true;r.host.nav=true;r.host.busy=true;r.submit("id=read,action=download");require(r.host.requests.size()==1&&r.host.requests[0].action==MissionAction::RequestList&&r.host.control_effects==0,"read depends on motion authority");});
    test("mutation requires current disarmed idle profile evidence",[]{
      for(unsigned kind=0;kind<5;++kind){Rig r;if(kind==0)r.host.state.armed=true;if(kind==1)r.host.nav=true;if(kind==2)r.host.busy=true;if(kind==3)r.host.state.landed_valid=false;if(kind==4)r.host.state.landed_state=2;r.submit("id=clear,action=clear");require(r.host.requests.empty(),"unsafe mutation submitted");}
      Rig rover;rover.host.state.type=10;rover.host.state.landed_valid=false;rover.submit("id=clear,action=clear");require(rover.host.requests.size()==1,"Rover needs fabricated landed evidence");
    });
    test("local admission failure has no new remote mutation claim",[]{Rig r;r.host.accept=false;r.submit("id=clear,action=clear");require(r.host.has("reason=host_admission_inhibited")&&r.host.has("mutation_may_have_applied=false")&&r.host.finished==std::vector<std::string>{"clear"},"admission rejection hidden/uncertain");});
    test("receipt opens response phase and empty download ACK is explicit",[]{Rig r;r.submit("id=read,action=download");r.count(0);require(r.host.requests.size()==1&&!r.host.has("COUNT_RECEIVED"),"pre-send reply unlocked phase");r.sent();r.count(0);r.advance();require(!r.host.has("status=COMPLETED")&&r.host.requests.back().action==MissionAction::Ack,"empty read completed without ACK receipt");r.sent();require(r.host.has("status=COMPLETED")&&r.host.last("MAVMOOS_MISSION_SNAPSHOT").find("\"count\":0")!=std::string::npos,"empty download incomplete");});
    test("ordered download discards future and identical repeated items",[]{Rig r;r.submit("id=read,action=download");r.sent();r.count(3);r.advance();r.sent();r.item(waypoint(2));require(!r.host.has("ITEM_RECEIVED"),"future item advanced sequence");r.item(home());r.advance();r.sent();r.item(home());r.item(waypoint(1));r.advance();r.sent();r.item(waypoint(2));r.advance();require(r.host.requests.back().action==MissionAction::Ack&&!r.host.has("status=COMPLETED"),"wrong download ending");r.sent();const auto s=r.host.last("MAVMOOS_MISSION_SNAPSHOT");require(s.find("\"count\":3")!=std::string::npos&&s.find("\"seq\":0")<s.find("\"seq\":2")&&r.host.has("status=COMPLETED"),"snapshot incomplete/unsorted");});
    test("wrong source target mission type and exact receipt are ignored",[]{
      Rig r;r.submit("id=read,action=download");auto wrong=r.host.requests[0];wrong.operation_id="other";r.plugin->onMissionSent(wrong,r.host.clock,true);r.count(0);require(!r.host.has("COUNT_RECEIVED"),"wrong callback opened phase");r.sent();r.count(0,99);r.count(0,245,99);r.count(0,245,191,MAV_MISSION_TYPE_FENCE);require(!r.host.has("COUNT_RECEIVED"),"foreign count accepted");
      mavlink_mission_count_t count{};count.target_system=245;count.target_component=191;mavlink_message_t msg{};mavlink_msg_mission_count_encode(2,1,&msg,&count);r.message(msg);require(!r.host.has("COUNT_RECEIVED"),"foreign sender accepted");r.count(0);r.advance();r.sent();require(r.host.has("status=COMPLETED"),"valid response rejected");
    });
    test("changed count duplicate data invalid flags and nonfinite data fail",[]{
      {Rig r;r.submit("id=x,action=download");r.sent();r.count(4097);require(r.host.has("mission_count_exceeds_bound"),"oversized table accepted");}
      {Rig r;r.submit("id=x,action=download");r.sent();r.count(2);r.advance();r.sent();r.count(3);require(r.host.has("mission_count_changed"),"count mutation ignored");}
      {Rig r;r.submit("id=x,action=download");r.sent();r.count(2);r.advance();r.sent();r.item(home());r.advance();r.sent();auto changed=home();changed.x+=1;r.item(changed);require(r.host.has("mission_item_changed"),"changed duplicate accepted");}
      for(bool nonfinite:{false,true}){Rig r;r.submit("id=x,action=download");r.sent();r.count(1);r.advance();r.sent();auto x=home();if(nonfinite)x.z=std::numeric_limits<float>::infinity();else x.current=2;r.item(x);require(r.host.has("unsupported_nonfinite_or_flagged_mission_item"),"invalid observed item accepted");}
    });
    test("response timeout retries fixed phase and original operation deadline",[]{
      Rig r;r.submit("id=x,action=download,timeout=4");const auto first=r.host.requests[0];r.sent();r.advance(1.51);require(r.host.requests.size()==1&&r.host.has("RETRY_DUE"),"response retry not staged");r.advance();const auto retry=r.host.requests.back();require(retry.id!=first.id&&retry.operation_id==first.operation_id&&retry.operation_expires==first.operation_expires&&retry.expires<=retry.operation_expires,"retry renewed operation or reused phase");
      r.plugin->onMissionSent(first,r.host.clock,true);r.count(0);require(!r.host.has("COUNT_RECEIVED"),"old receipt unlocked retry");r.sent();r.count(0);r.advance();r.sent();require(r.host.has("status=COMPLETED"),"retry cannot complete");
    });
    test("bounded per-phase retry budget and no hidden receipt retry",[]{
      Rig r;r.submit("id=silent,action=download");for(unsigned i=0;i<6;++i){r.sent();r.advance(1.51);r.advance();}require(r.host.requests.size()==6&&r.host.has("bounded_retries_exhausted"),"unbounded retries");
      Rig missing;missing.submit("id=missing,action=download");missing.advance(1.01);require(missing.host.requests.size()==1&&missing.host.has("local_send_receipt_missing"),"missing receipt auto-retried");
    });
    test("known local failure is terminal and duplicate callback cannot reverse acceptance",[]{
      Rig failed;failed.submit("id=c,action=clear");failed.sent(false);failed.advance();require(failed.host.requests.size()==1&&failed.host.has("local_send_rejected_or_failed")&&failed.host.has("mutation_may_have_applied=false"),"known failed mutation claimed effect/retried");
      Rig duplicate;duplicate.submit("id=x,action=download");duplicate.sent();duplicate.receipt(0,false);duplicate.count(0);duplicate.advance();duplicate.sent();require(duplicate.host.has("status=COMPLETED")&&!duplicate.host.has("local_send_rejected_or_failed"),"duplicate receipt changed accepted outcome");
    });
    test("global retry cap survives successful upload phase advances",[]{
      Rig r;std::vector<MissionItem> items{home()};for(unsigned i=1;i<8;++i)items.push_back(waypoint(i));
      r.submit("id=bounded,action=upload,items="+rows(items));r.sent();r.request(0);r.advance();
      for(unsigned seq=0;seq<6;++seq){for(unsigned retry=0;retry<5;++retry){r.sent();r.request(seq);r.advance();}r.sent();r.request(seq+1);r.advance();}
      for(unsigned retry=0;retry<3;++retry){r.sent();r.request(6);r.advance();}
      require(r.host.requests.size()==40&&r.host.has("retries=32")&&r.host.has("bounded_retries_exhausted"),"global retry budget reset across phases");
    });
    test("wrong receipt payload action and original deadline cannot unlock phase",[]{
      for(unsigned field=0;field<3;++field){Rig r;r.submit("id=x,action=download");auto wrong=r.host.requests[0];if(field==0)wrong.action=MissionAction::Ack;if(field==1)wrong.operation_expires+=1;if(field==2)wrong.count=1;r.plugin->onMissionSent(wrong,r.host.clock,true);r.count(0);require(!r.host.has("COUNT_RECEIVED"),"modified phase receipt accepted");}
    });
    test("deadline wins before late reply and accepted late mutation keeps uncertainty",[]{
      Rig late;late.submit("id=x,action=download,timeout=1");late.sent();late.advance(1.01);late.count(0);require(late.host.has("original_operation_deadline")&&!late.host.has("status=COMPLETED"),"reply renewed original deadline");
      Rig mutation;mutation.submit("id=x,action=clear,timeout=1");mutation.host.clock+=1.01;mutation.sent();require(mutation.host.has("local_send_outside_original_deadline")&&mutation.host.has("mutation_may_have_applied=true"),"late accepted write lost uncertainty");
    });
    test("upload accepts legacy peer requests but requires independent readback",[]{
      Rig r;const auto items=std::vector<MissionItem>{home(),waypoint()};r.uploadThroughAck(items,true);
      require(r.host.has("upload_accepted_readback_required")&&!r.host.has("status=COMPLETED"),"ACK completed upload without readback");
      auto observed=items;observed[0].x=0;observed[0].y=0;observed[0].z=0;observed[0].current=1;observed[1].z+=.005f;
      r.download(observed);r.sent();require(r.host.has("status=COMPLETED")&&r.host.has("readback_confirmed=true")&&r.host.has("legacy_requests=2"),"verified upload failed");
      const auto snapshot=r.host.last("MAVMOOS_MISSION_SNAPSHOT");require(snapshot.find("\"home_reference_compared\":false")!=std::string::npos&&snapshot.find("\"verification\":\"semantic_profile_match\"")!=std::string::npos,"readback tolerance/home exception hidden");
      const double deadline=r.host.requests[0].operation_expires;for(const auto& q:r.host.requests)require(q.operation_expires==deadline&&q.expires<=deadline,"upload/readback renewed deadline");
      require(r.host.finished==std::vector<std::string>{"upload"}&&r.host.control_effects==0,"lease not retired once or motion requested");
    });
    test("ordinary current flag is metadata while Guided current two remains forbidden",[]{Rig r;auto items=std::vector<MissionItem>{home(),waypoint()};items[0].current=1;r.uploadThroughAck(items);items[0].current=0;r.download(items);r.sent();require(r.host.has("status=COMPLETED")&&r.host.control_effects==0,"QGC HOME current metadata was treated as execution authority");});
    test("pinned ArduPilot LAND and non-positional RTL storage defaults verify explicitly",[]{
      Rig r;auto takeoff=waypoint(1);takeoff.command=MAV_CMD_NAV_TAKEOFF;auto land=waypoint(2);land.command=MAV_CMD_NAV_LAND;
      MissionItem rtl;rtl.seq=3;rtl.command=MAV_CMD_NAV_RETURN_TO_LAUNCH;rtl.frame=MAV_FRAME_MISSION;rtl.z=0;
      const auto items=std::vector<MissionItem>{home(),takeoff,land,rtl};r.uploadThroughAck(items);
      for(const auto& request:r.host.requests)if(request.action==MissionAction::Item){if(request.seq==2)require(request.item.params[3]==0,"canonical comparison changed uploaded LAND");if(request.seq==3)require(request.item.frame==MAV_FRAME_MISSION,"canonical comparison changed uploaded RTL");}
      auto observed=items;observed[2].params[3]=1;observed[3].frame=MAV_FRAME_GLOBAL;r.download(observed);r.sent();
      require(r.host.has("status=COMPLETED")&&r.host.has("readback_confirmed=true"),"known AP storage defaults falsely failed");
      const auto snapshot=r.host.last("MAVMOOS_MISSION_SNAPSHOT");require(snapshot.find("\"seq\":3,\"command\":20,\"frame\":0")!=std::string::npos&&snapshot.find("\"params\":[0,0,0,1]")!=std::string::npos&&snapshot.find("\"supported_storage_normalizations\"")!=std::string::npos,"normalization erased raw data or metadata");
    });
    test("storage default equivalence never ignores other frames parameters or item kinds",[]{
      for(unsigned field=0;field<9;++field){Rig r;auto land=waypoint(1);land.command=MAV_CMD_NAV_LAND;MissionItem rtl;rtl.seq=2;rtl.command=MAV_CMD_NAV_RETURN_TO_LAUNCH;rtl.frame=MAV_FRAME_MISSION;rtl.z=0;
        auto items=std::vector<MissionItem>{home(),land,rtl};if(field==3)items[1].command=MAV_CMD_NAV_WAYPOINT;if(field==4)items[1].command=MAV_CMD_NAV_TAKEOFF;
        r.uploadThroughAck(items);auto observed=items;observed[1].params[3]=1;observed[2].frame=MAV_FRAME_GLOBAL;
        if(field==0)observed[2].frame=MAV_FRAME_GLOBAL_RELATIVE_ALT;if(field==1)observed[1].params[3]=-1;if(field==2)observed[1].params[3]=.5;
        if(field==5)observed[2].params[0]=1;if(field==6)observed[2].x=1;if(field==7)observed[1].frame=MAV_FRAME_GLOBAL;if(field==8)observed[1].params[3]=1.00001f;
        r.download(observed);r.sent();require(r.host.has("upload_readback_item_mismatch")&&!r.host.has("status=COMPLETED")&&r.host.has("readback_confirmed=false"),"unrelated semantic mismatch normalized away");
      }
    });
    test("lost final upload ACK is bounded and never fabricates verification",[]{Rig r;r.submit("id=lost,action=upload,items="+rows({home()}));r.sent();r.request(0);r.advance();for(unsigned i=0;i<6;++i){r.sent();r.advance(1.51);r.advance();}require(r.host.has("bounded_retries_exhausted")&&r.host.has("mutation_may_have_applied=true")&&!r.host.has("status=COMPLETED"),"missing ACK fabricated success or retried forever");});
    test("upload early ACK and skipped peer sequence cannot complete or skip data",[]{Rig r;r.submit("id=x,action=upload,items="+rows({home(),waypoint()}));r.ack();r.sent();r.ack();r.request(1);r.advance();require(r.host.requests.size()==1&&!r.host.has("ACKNOWLEDGED"),"early ACK/skipped request accepted");r.request(0);r.advance();require(r.host.requests.back().seq==0,"sequence zero not required");r.sent();r.ack();require(!r.host.has("ACKNOWLEDGED"),"partial upload ACK accepted");});
    test("upload duplicate request retransmission is bounded and invalid-sequence error does not advance",[]{Rig r;r.submit("id=x,action=upload,items="+rows({home(),waypoint()}));r.sent();r.request(0);r.advance();r.sent();r.request(0);r.advance();require(r.host.requests.back().action==MissionAction::Item&&r.host.requests.back().seq==0&&r.host.has("retries=1"),"peer duplicate did not resend current item");r.sent();r.ack(MAV_MISSION_INVALID_SEQUENCE);require(!r.host.has("status=FAILED")&&!r.host.has("ACKNOWLEDGED"),"recoverable sequence error failed/advanced");r.request(1);r.advance();require(r.host.requests.back().seq==1,"next legitimate request rejected");});
    test("negative remote ACKs explicit and foreign ACKs do not complete clear",[]{Rig r;r.submit("id=x,action=clear");r.sent();r.ack(MAV_MISSION_ACCEPTED,99);r.ack(MAV_MISSION_ACCEPTED,245,MAV_MISSION_TYPE_RALLY);require(!r.host.has("ACKNOWLEDGED"),"foreign ACK accepted");r.ack(MAV_MISSION_DENIED);require(r.host.has("remote_mission_error_14")&&r.host.has("mutation_may_have_applied=true"),"negative remote result hidden");});
    test("upload readback count or semantic mismatch never reports verified completion",[]{
      {Rig r;r.uploadThroughAck({home(),waypoint()});r.sent();r.count(1);require(r.host.has("upload_readback_count_mismatch")&&!r.host.has("status=COMPLETED"),"wrong count verified");}
      for(unsigned field=0;field<4;++field){Rig r;auto expected=std::vector<MissionItem>{home(),waypoint()};r.uploadThroughAck(expected);auto changed=expected;if(field==0)changed[1].x+=1;if(field==1)changed[1].frame=0;if(field==2)changed[1].params[0]=1;if(field==3)changed[1].z+=.1f;r.download(changed);r.sent();require(r.host.has("upload_readback_item_mismatch")&&!r.host.has("status=COMPLETED")&&r.host.has("readback_confirmed=false"),"mismatched data verified");}
    });
    test("clear ACK is followed by explicit empty readback and ACK receipt",[]{Rig r;r.submit("id=c,action=clear");r.sent();r.ack();r.advance();require(r.host.requests.back().action==MissionAction::RequestList&&!r.host.has("status=COMPLETED"),"clear ACK completed without readback");r.sent();r.count(0);r.advance();require(r.host.requests.back().action==MissionAction::Ack,"empty readback ACK missing");r.sent();require(r.host.has("clear_readback_empty")&&r.host.has("readback_confirmed=true"),"clear verification failed");Rig bad;bad.submit("id=c,action=clear");bad.sent();bad.ack();bad.advance();bad.sent();bad.count(1);require(bad.host.has("clear_readback_not_empty")&&!bad.host.has("status=COMPLETED"),"nonempty clear verified");});
    test("external arming during mutation stops more sends without rollback",[]{Rig r;r.submit("id=u,action=upload,items="+rows({home(),waypoint()}));r.sent();r.host.state.armed=true;r.advance();require(r.host.requests.size()==1&&r.host.has("mutation_state_no_longer_permitted")&&r.host.has("mutation_may_have_applied=true"),"post-count arming continued mutation or lost uncertainty");});
    test("readback can resolve accepted upload after external arming",[]{Rig r;const auto items=std::vector<MissionItem>{home(),waypoint()};r.uploadThroughAck(items);r.host.state.armed=true;r.host.state.landed_valid=false;r.download(items);r.sent();require(r.host.has("status=COMPLETED")&&r.host.has("readback_confirmed=true"),"safe readback inhibited by external state change");});
    test("cancel removes operation lease, ignores late callbacks, and never starts mission",[]{Rig r;r.submit("id=u,action=upload,items="+rows({home(),waypoint()}));const auto old=r.host.requests[0];r.submit("id=u,action=cancel");r.plugin->onMissionSent(old,r.host.clock,true);r.advance();require(r.host.requests.size()==1&&r.host.has("status=CANCELLED")&&r.host.has("mutation_may_have_applied=true")&&r.host.finished==std::vector<std::string>{"u"},"cancel left pending work/lease or hid uncertainty");r.submit("id=u,action=cancel");require(r.host.has("operation_not_active"),"unknown cancel accepted");require(r.host.control_effects==0,"data cancellation controlled vehicle");});
    test("reset invalidates snapshots and retains request ID history",[]{Rig r;r.submit("id=r,action=download");r.sent();r.plugin->onReset("test");r.count(0);r.advance();require(r.host.has("status=CANCELLED")&&r.host.last("MAVMOOS_MISSION_SNAPSHOT").find("\"invalidated\"")!=std::string::npos&&!r.host.has("status=COMPLETED"),"reset revived old operation/snapshot");r.submit("id=r,action=download");require(r.host.has("status=DUPLICATE"),"reset erased replay history");r.submit("id=new,action=download");require(r.host.requests.size()==2,"fresh new operation after boundary blocked");});
    test("heartbeat and clock loss retire old operations before inbound success",[]{Rig heartbeat;heartbeat.submit("id=r,action=download");heartbeat.sent();heartbeat.host.clock+=2.01;heartbeat.count(0);require(heartbeat.host.has("heartbeat_unavailable")&&!heartbeat.host.has("status=COMPLETED"),"data concealed old heartbeat expiry");Rig clock;clock.submit("id=r,action=download");clock.sent();clock.host.clock-=1;clock.plugin->onInputBoundary();require(clock.host.has("clock_fault"),"clock rollback ignored");});
    test("mission progress observations grant no authority",[]{Rig r;r.advance();mavlink_mission_current_t current{};current.seq=3;current.total=7;current.mission_state=3;mavlink_message_t msg{};mavlink_msg_mission_current_encode(1,1,&msg,&current);r.message(msg);mavlink_mission_item_reached_t reached{};reached.seq=2;mavlink_msg_mission_item_reached_encode(1,1,&msg,&reached);r.message(msg);require(r.host.count("MAVMOOS_MISSION_PROGRESS")==2&&r.host.has("source_age=unknown")&&r.host.requests.empty()&&r.host.control_effects==0,"progress has effects or false source time");});
    test("mission progress retains reset boundary and acquisition order",[]{
      Rig r;r.host.clock=101;r.plugin->onReset("connection_reset");
      mavlink_mission_current_t current{};current.seq=3;
      mavlink_message_t message{};mavlink_msg_mission_current_encode(1,1,&message,&current);
      r.host.clock=101.2;r.host.state.heartbeat_valid=false;
      r.plugin->onMessageWithMetadata(message,{101.1,101.2,"QUEUE_EMPTY_FLOOR"});
      r.host.fresh();r.host.clock=101.3;
      r.plugin->onMessageWithMetadata(message,{100.9,101.3,"QUEUE_EMPTY_FLOOR"});
      require(r.host.count("MAVMOOS_MISSION_PROGRESS")==1,"unbound or pre-reset progress replaced invalidation");
      r.plugin->onMessageWithMetadata(message,{101.2,101.3,"QUEUE_EMPTY_FLOOR"});
      const auto fresh=r.host.last("MAVMOOS_MISSION_PROGRESS");
      require(fresh.find("effective_at=101.200000000,read_at=101.300000000,age_basis=QUEUE_EMPTY_FLOOR")!=std::string::npos,
              "progress lost original acquisition and read timestamps");
      r.host.clock=101.4;
      r.plugin->onMessageWithMetadata(message,{101.1,101.4,"QUEUE_EMPTY_FLOOR"});
      require(r.host.count("MAVMOOS_MISSION_PROGRESS")==2,"older progress displaced a newer observation");
    });
    test("mission download rejects a changed known plan ID at the same count",[]{
      Rig r;r.submit("id=r,action=download");r.sent();r.count(2,245,191,0,7);r.advance();r.sent();
      r.count(2,245,191,0,0);
      require(!r.host.has("status=INCOMPLETE"),"unknown plan ID was interpreted as a change");
      r.count(2,245,191,0,8);r.item(home());r.advance();
      require(r.host.has("status=INCOMPLETE,reason=mission_plan_id_changed")&&
              r.host.count("MAVMOOS_MISSION_SNAPSHOT")==0&&r.host.finished==std::vector<std::string>{"r"},
              "conflicting plan ID did not retire the old snapshot and lease");
    });
    test("progress unknown versus empty and unsupported autopilot stay explicit",[]{Rig r;r.advance();mavlink_mission_current_t current{};mavlink_message_t msg{};mavlink_msg_mission_current_encode(1,1,&msg,&current);r.message(msg);require(r.host.has("total=unknown,total_wire=0")&&r.host.has("mission_state=unknown"),"unsupported extension fabricated empty plan/state");current.total=UINT16_MAX;mavlink_msg_mission_current_encode(1,1,&msg,&current);r.message(msg);require(r.host.has("total=none,total_wire=65535"),"empty mission sentinel hidden");r.host.state.autopilot=MAV_AUTOPILOT_PX4;r.submit("id=wrong,action=download");require(r.host.requests.empty(),"ArduPilot semantics applied to unsupported autopilot");});
    test("busy duplicate history and configuration remain bounded",[]{
      Rig busy;busy.submit("id=one,action=download");busy.submit("id=one,action=download");busy.submit("id=two,action=download");require(busy.host.requests.size()==1&&busy.host.has("status=DUPLICATE")&&busy.host.has("reason=busy"),"busy/duplicate admitted");
      Rig r;for(unsigned i=0;i<128;++i){r.submit("id=x"+std::to_string(i)+",action=download");r.download({});r.sent();}r.submit("id=overflow,action=download");require(r.host.has("request_history_full")&&r.host.requests.size()==256,"request history unbounded");
      for(const auto& value:{"0","121","nan"}){bool rejected=false;try{r.plugin->configure("mission_timeout",value);}catch(const std::invalid_argument&){rejected=true;}require(rejected,"invalid timeout configured");}
    });
    test("maximum indexed snapshot remains bounded and no post-expiry work",[]{Rig r;r.plugin->configure("mission_timeout","120");r.submit("id=full,action=download");r.sent();r.count(4096);r.advance();for(unsigned i=0;i<4096;++i){r.sent();r.item(i==0?home():waypoint(i));r.advance(.001);}r.sent();const auto s=r.host.last("MAVMOOS_MISSION_SNAPSHOT");require(r.host.has("status=COMPLETED")&&s.find("\"count\":4096")!=std::string::npos&&s.find("\"seq\":4095")!=std::string::npos&&s.size()<2*1024*1024&&r.host.requests.size()==4098,"maximum snapshot wrong/unbounded");});
    std::cout<<"PASS "<<passed<<" mission provider groups; actual provider, fake I/O only\n";
    if(module)dlclose(module);return 0;
  }catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';if(module)dlclose(module);return 1;}
}
