#pragma once
#include "../protocol_faults/ProtocolSupport.h"
#include "CopterFrame.h"
#include <functional>
#include <iostream>

namespace execution_concurrency {
using namespace mavmoos::runtime;
using protocol_faults::require;
using protocol_faults::fields;
using protocol_faults::Frame;
using protocol_faults::Publication;
using protocol_faults::command;

struct Rig {
  double time=100;
  const unsigned vehicle_type;
  bool report_landed=true;
  bool airborne=false,hold_missions=false,hold_parameters=false,queue_floor_ack=false;
  unsigned mode=4,mission_state=MISSION_STATE_NOT_STARTED;
  std::vector<Frame> frames;
  std::vector<Publication> publications;
  std::vector<MissionItem> plan;
  std::vector<mavlink_message_t> deferred_missions,deferred_parameters;
  std::vector<std::array<double,3>> ack_windows;
  std::function<void(const mavlink_message_t&)> before_gate;
  mavlink_status_t codec{};
  Runtime host;
  std::size_t processed=0;
  StoredMissionProof original_proof;
  explicit Rig(const std::vector<std::string>& modules,const char* frame="quad")
    :vehicle_type(copterFrameType(frame)),host(
    [this](const auto& key,const auto& text,double,bool numeric){if(!numeric)publications.push_back({key,text,time});},
    [](const uint8_t*,std::size_t)->bool{throw std::runtime_error("unbounded sender used");},true,false,[this]{return time;},vehicle_type)
  {
    for(const auto& module:modules)require(host.load(module),"actual DSO load failed");
    host.configure("copter_frame",frame);
    host.configure("control_native_actions","true");host.configure("control_mission_execution","true");
    host.configure("parameter_timeout","5");host.configure("mission_timeout","15");
    host.setBoundedSend([this](const uint8_t* bytes,std::size_t size,double expires,const Runtime::SendEligibility& live){
      const auto message=protocol_faults::decode(bytes,size);
      if(before_gate)before_gate(message);
      const bool accepted=live(time)&&time<expires;frames.push_back({message,time,expires,accepted});return accepted;
    });
    for(unsigned n=0;n<3;++n){auto item=protocol_faults::home();item.seq=n;if(n){item.frame=MAV_FRAME_GLOBAL_RELATIVE_ALT;item.x+=100*n;item.z=5;}plan.push_back(item);}
    time+=.001;fresh();
  }
  void fresh()
  {
    mavlink_message_t m{};mavlink_msg_heartbeat_pack_status(1,1,&codec,&m,vehicle_type,3,airborne?128:0,mode,4);host.message(m,time);
    mavlink_msg_global_position_int_pack_status(1,1,&codec,&m,1000,423584560,-710875890,airborne?15000:10000,airborne?5000:0,0,0,0,9000);host.message(m,time);
    if(report_landed) {mavlink_msg_extended_sys_state_pack_status(1,1,&codec,&m,0,airborne?2:1);host.message(m,time);}
    mavlink_home_position_t h{};h.latitude=423584560;h.longitude=-710875890;h.altitude=10000;mavlink_msg_home_position_encode_status(1,1,&codec,&m,&h);host.message(m,time);
    mavlink_ekf_status_report_t e{};e.flags=51;mavlink_msg_ekf_status_report_encode_status(1,1,&codec,&m,&e);host.message(m,time);
  }
  void current()
  {mavlink_mission_current_t p{};p.seq=1;p.total=2;p.mission_state=mission_state;p.mission_mode=mode==3?1:0;mavlink_message_t m{};mavlink_msg_mission_current_encode_status(1,1,&codec,&m,&p);host.message(m,time);}
  void input(const mavlink_message_t& m)
  {time+=.001;fresh();host.message(m,time);}
  void mail(const std::string& key,const std::string& text,const std::string& source="concurrency_fixture")
  {time+=.001;fresh();Mail m;m.key=key;m.text=text;m.source=source;m.published_at=m.received_at=time;host.mail(m);}
  void parameter(const std::string& id,const std::string& action="read")
  {mail("MAVMOOS_PARAM_REQUEST","id="+id+",action="+action+(action=="read"?",name=FRAME_CLASS":""));}
  void mission(const std::string& id,const std::string& action="download")
  {mail("MAVMOOS_MISSION_REQUEST","id="+id+",action="+action);}
  std::map<std::string,std::string> last(const std::string& key,const std::string& id="")const
  {for(auto p=publications.rbegin();p!=publications.rend();++p)if(p->key==key){auto f=fields(p->text);if(id.empty()||f["id"]==id)return f;}return {};}
  bool outcome(const std::string& key,const std::string& id,const std::string& state)const
  {for(const auto& p:publications)if(p.key==key){const auto f=fields(p.text);if(f.count("id")&&f.at("id")==id&&f.at("status")==state)return true;}return false;}
  unsigned count(unsigned id,bool accepted=true)const
  {return unsigned(std::count_if(frames.begin(),frames.end(),[&](const auto& f){return f.message.msgid==id&&(!accepted||f.accepted);}));}
  unsigned commands(unsigned id,bool accepted=true)const
  {return unsigned(std::count_if(frames.begin(),frames.end(),[&](const auto& f){return command(f.message)==id&&(!accepted||f.accepted);}));}
  void answerParameter(const mavlink_message_t& out)
  {
    require(out.msgid==MAVLINK_MSG_ID_PARAM_REQUEST_READ,"readonly parameter profile emitted other protocol");
    mavlink_param_request_read_t r{};mavlink_msg_param_request_read_decode(&out,&r);mavlink_param_value_t p{};
    const std::string name=r.param_index<0?std::string(r.param_id,std::find(r.param_id,r.param_id+16,'\0')):"P"+std::to_string(r.param_index);
    std::copy(name.begin(),name.end(),p.param_id);p.param_count=4;p.param_index=r.param_index<0?UINT16_MAX:uint16_t(r.param_index);p.param_type=MAV_PARAM_TYPE_REAL32;p.param_value=r.param_index<0?1.f:float(r.param_index);
    mavlink_message_t m{};mavlink_msg_param_value_encode_status(1,1,&codec,&m,&p);input(m);
  }
  void answerMission(const mavlink_message_t& out)
  {
    mavlink_message_t m{};
    if(out.msgid==MAVLINK_MSG_ID_MISSION_COUNT){mavlink_mission_request_int_t p{};p.target_system=245;p.target_component=191;mavlink_msg_mission_request_int_encode_status(1,1,&codec,&m,&p);}
    else if(out.msgid==MAVLINK_MSG_ID_MISSION_ITEM_INT){mavlink_mission_item_int_t sent{};mavlink_msg_mission_item_int_decode(&out,&sent);
      if(sent.seq+1<plan.size()){mavlink_mission_request_int_t p{};p.target_system=245;p.target_component=191;p.seq=sent.seq+1;mavlink_msg_mission_request_int_encode_status(1,1,&codec,&m,&p);}
      else {mavlink_mission_ack_t p{};p.target_system=245;p.target_component=191;p.type=MAV_MISSION_ACCEPTED;mavlink_msg_mission_ack_encode_status(1,1,&codec,&m,&p);}}
    else if(out.msgid==MAVLINK_MSG_ID_MISSION_REQUEST_LIST){mavlink_mission_count_t p{};p.target_system=245;p.target_component=191;p.count=uint16_t(plan.size());mavlink_msg_mission_count_encode_status(1,1,&codec,&m,&p);}
    else if(out.msgid==MAVLINK_MSG_ID_MISSION_REQUEST_INT){mavlink_mission_request_int_t r{};mavlink_msg_mission_request_int_decode(&out,&r);require(r.seq<plan.size(),"mission item outside fixture");const auto& x=plan[r.seq];mavlink_mission_item_int_t p{};p.target_system=245;p.target_component=191;p.seq=x.seq;p.command=x.command;p.frame=x.frame;p.current=x.current;p.autocontinue=x.autocontinue;p.param1=x.params[0];p.param2=x.params[1];p.param3=x.params[2];p.param4=x.params[3];p.x=x.x;p.y=x.y;p.z=x.z;mavlink_msg_mission_item_int_encode_status(1,1,&codec,&m,&p);}
    else return;
    input(m);
  }
  void answerCommand(const mavlink_message_t& out,double sent_at)
  {
    const auto type=command(out);
    if(type==MAV_CMD_MISSION_START){mode=3;mission_state=MISSION_STATE_ACTIVE;}
    else if(type==MAV_CMD_DO_PAUSE_CONTINUE)mission_state=MISSION_STATE_PAUSED;
    else if(type==MAV_CMD_DO_SET_MODE){mode=4;mission_state=MISSION_STATE_NOT_STARTED;}
    mavlink_command_ack_t p{};p.command=uint16_t(type);p.result=MAV_RESULT_ACCEPTED;p.target_system=245;p.target_component=191;mavlink_message_t m{};mavlink_msg_command_ack_encode_status(1,1,&codec,&m,&p);
    if(queue_floor_ack){time+=.001;fresh();const double floor=sent_at-.0005;ack_windows.push_back({floor,sent_at,time});host.message(m,floor,time,"QUEUE_EMPTY_FLOOR");}else input(m);
    time+=.001;fresh();current();
  }
  void replies()
  {
    while(processed<frames.size()){
      const auto frame=frames[processed++];if(!frame.accepted)continue;const auto& m=frame.message;
      if(m.msgid==MAVLINK_MSG_ID_COMMAND_LONG)answerCommand(m,frame.at);
      else if(m.msgid==MAVLINK_MSG_ID_PARAM_REQUEST_READ){if(hold_parameters)deferred_parameters.push_back(m);else answerParameter(m);}
      else if(m.msgid==MAVLINK_MSG_ID_MISSION_COUNT||m.msgid==MAVLINK_MSG_ID_MISSION_ITEM_INT||m.msgid==MAVLINK_MSG_ID_MISSION_REQUEST_LIST||m.msgid==MAVLINK_MSG_ID_MISSION_REQUEST_INT){if(hold_missions)deferred_missions.push_back(m);else answerMission(m);}
    }
  }
  void tick(double delta=.02)
  {time+=delta;fresh();if(airborne)current();host.tick(time);replies();}
  void until(const std::function<bool()>& done,double seconds=5)
  {const auto end=time+seconds;while(!done()&&time<end)tick();require(done(),"bounded fake scenario condition missing");}
  void releaseMissions(){hold_missions=false;const auto saved=deferred_missions;deferred_missions.clear();for(const auto& m:saved)answerMission(m);}
  void releaseParameters(){hold_parameters=false;const auto saved=deferred_parameters;deferred_parameters.clear();for(const auto& m:saved)answerParameter(m);}
  void upload()
  {
    std::ostringstream text;text<<"id=verified,action=upload,items=";
    for(const auto& x:plan){if(x.seq)text<<'|';text<<x.seq<<":16:"<<unsigned(x.frame)<<":0:1:0:0:0:0:"<<x.x<<':'<<x.y<<':'<<x.z;}
    mail("MAVMOOS_MISSION_REQUEST",text.str());until([&]{return outcome("MAVMOOS_MISSION_RESULT","verified","COMPLETED");});
    original_proof=host.storedMissionProof();require(original_proof.valid,"accepted upload readback did not establish typed proof");
  }
  void requestStart(double life=10)
  {airborne=true;mail("MAVMOOS_MISSION_CONTROL","schema=1,id=run,action=start,verification_id=verified,content_id="+original_proof.content_id+",proof_revision="+std::to_string(original_proof.revision)+",timeout="+std::to_string(life));}
  void running(double life=10)
  {upload();requestStart(life);until([&]{return outcome("MAVMOOS_MISSION_EXECUTION_RESULT","run","ACTIVE");});owned();require(commands(MAV_CMD_MISSION_START)==1,"actual coordinator start effect missing/duplicated");}
  void owned()const
  {require(host.controlBusy()&&host.missionExecutionGranted("run")&&!host.navigationGranted(),"AUTO persistent ownership lost or helm granted");}
  bool sameProof()const
  {const auto p=host.storedMissionProof();if(!p.valid||p.revision!=original_proof.revision||p.session_epoch!=original_proof.session_epoch||p.content_id!=original_proof.content_id||p.verification_id!=original_proof.verification_id||p.verified_at!=original_proof.verified_at||p.items.size()!=original_proof.items.size())return false;for(unsigned i=0;i<p.items.size();++i){const auto& a=p.items[i];const auto& b=original_proof.items[i];if(a.seq!=b.seq||a.command!=b.command||a.frame!=b.frame||a.current!=b.current||a.autocontinue!=b.autocontinue||a.params!=b.params||a.x!=b.x||a.y!=b.y||a.z!=b.z)return false;}return true;}
  double originalDeadline()const
  {for(const auto& p:publications)if(p.key=="MAVMOOS_MISSION_EXECUTION_RESULT"){const auto f=fields(p.text);if(f.at("id")=="run"&&f.at("status")=="ADMITTED")return std::stod(f.at("expires"));}throw std::runtime_error("original AUTO deadline missing");}
  void immutableDeadline()const
  {std::string original;for(const auto& p:publications)if(p.key=="MAVMOOS_MISSION_EXECUTION_RESULT"){const auto f=fields(p.text);if(f.at("id")!="run")continue;if(original.empty())original=f.at("expires");require(original==f.at("expires"),"AUTO original deadline changed");}}
  void emit(const std::string& name,bool pass=true,const std::string& error="")const
  {
    std::cout<<"{\"name\":"<<protocol_faults::quote(name)<<",\"result\":"<<protocol_faults::quote(pass?"PASS":"FAIL")<<",\"scope\":\"actual Runtime and enabled AUTO/control/mission/parameter DSOs; fake clock and bounded I/O only; no socket/FC\",\"error\":"<<protocol_faults::quote(error)<<",\"runtime\":"<<protocol_faults::quote(host.status())<<",\"frames\":[";
    bool comma=false;for(const auto& f:frames){if(comma)std::cout<<',';comma=true;std::cout<<protocol_faults::frameJson(f);}std::cout<<"],\"publications\":[";comma=false;for(const auto& p:publications){if(comma)std::cout<<',';comma=true;std::cout<<protocol_faults::publicationJson(p);}std::cout<<"],\"protocol_ack_windows\":[";comma=false;for(const auto& p:ack_windows){if(comma)std::cout<<',';comma=true;std::cout<<"{\"effective_at\":"<<protocol_faults::decimal(p[0])<<",\"sent_at\":"<<protocol_faults::decimal(p[1])<<",\"read_at\":"<<protocol_faults::decimal(p[2])<<'}';}std::cout<<"]}\n";
  }
};
} // namespace execution_concurrency
