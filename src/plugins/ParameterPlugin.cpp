/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                  */
/*    FILE: ParameterPlugin.cpp                              */
/*    DATE: November 3, 2026                                 */
/************************************************************/
#include "Plugin.h"
#include "ArduPilotObservations.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>

namespace mavmoos::runtime {
namespace {
bool idValid(const std::string& s)
{
  if(s.empty()||s.size()>48) return false;
  for(unsigned char c:s) if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_'||c=='-'||c=='.')) return false;
  return true;
}
bool nameValid(const std::string& s)
{
  if(s.empty()||s.size()>16) return false;
  for(unsigned char c:s) if(!((c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_')) return false;
  return true;
}
bool parseNumber(const std::string& s,double& out)
{
  try { std::size_t used=0;out=std::stod(s,&used);return used==s.size()&&std::isfinite(out); }
  catch(...) { return false; }
}
std::string trim(const std::string& s)
{
  const auto first=s.find_first_not_of(" \t\r\n");
  return first==std::string::npos?std::string{}:s.substr(first,s.find_last_not_of(" \t\r\n")-first+1);
}
bool validValue(const mavlink_param_value_t& v)
{
  if(!std::isfinite(v.param_value)) return false;
  if(v.param_type==MAV_PARAM_TYPE_REAL32) return true;
  if(std::trunc(v.param_value)!=v.param_value) return false;
  if(v.param_type==MAV_PARAM_TYPE_INT8) return v.param_value>=-128&&v.param_value<=127;
  if(v.param_type==MAV_PARAM_TYPE_INT16) return v.param_value>=-32768&&v.param_value<=32767;
  // C-cast wire values can round an INT32 endpoint; mark large values potentially lossy.
  return v.param_type==MAV_PARAM_TYPE_INT32&&v.param_value>=-2147483648.&&v.param_value<=2147483648.;
}
struct Value { std::string name; float value=0;uint8_t type=0;double acquired_at=0; };
enum class Stage { Queued, AwaitInitialSend, Read, List, Paced, Echo, NeedReadback, AwaitReadbackSend, Readback };
enum class ListMode { Paced, Streamed };
struct IndexPhase {
  std::string wire_id;
  double submitted_at=0,send_by=0,sent_at=0,reply_by=0;
  bool awaiting_receipt=true,accepted=false,retry_due=false;
};
struct Operation {
  std::string id,action,name,wire_id;
  float value=0;
  double expires=0,sent_at=0,last_value=0,last_recovery=0;
  Stage stage=Stage::Queued;
  unsigned expected=0,recoveries=0;
  ListMode list_mode=ListMode::Paced;
  unsigned next_index=1,phase_count=0;
  std::map<unsigned,IndexPhase> outstanding;
  bool recovery_waiting=false,write_may_have_applied=false,readback_confirmed=false;
  std::map<unsigned,Value> entries;
  std::map<std::string,unsigned> names;
};
class Parameters final:public Plugin {
public:
  explicit Parameters(Services& s):m_services(s),m_boundary(s.now()) {}
  std::string name() const override { return "parameters"; }
  PluginDescriptor descriptor() const override {
    PluginDescriptor d; d.profile="ardupilot_c_cast";
    d.capabilities={{"parameters.read","MAVMOOS_PARAM_REQUEST","action=read"},
      {"parameters.list","MAVMOOS_PARAM_REQUEST","action=list"},
      {"parameters.write.sim_wind","MAVMOOS_PARAM_REQUEST","action=write,name=SIM_WIND_SPD"}};
    d.exclusive_resources={"protocol.parameters"}; d.request_owners={"MAVMOOS_PARAM_REQUEST"};
    d.observation_streams=arduPilotObservationStreams();
    d.streamed_parameter_bench_compatible=true; return d;
  }
  std::vector<std::string> subscriptions() const override { return {"MAVMOOS_PARAM_REQUEST"}; }
  void configure(const std::string& key,const std::string& text) override
  {
    if(key=="parameter_list_mode") {
      if(text!="paced"&&text!="streamed")throw std::invalid_argument("invalid parameter_list_mode");
      m_list_mode=text=="paced"?ListMode::Paced:ListMode::Streamed;return;
    }
    if(key!="parameter_timeout"&&key!="parameter_list_timeout") return;
    double value=0;
    if(!parseNumber(text,value)||value<1||value>60) throw std::invalid_argument("invalid "+key);
    if(key=="parameter_timeout")m_timeout=value;
    else {m_list_timeout=value;m_list_timeout_explicit=true;}
  }
  void onMail(const Mail& mail) override
  {
    if(mail.key!="MAVMOOS_PARAM_REQUEST") return;
    Operation op;op.id="invalid";op.action="invalid";
    if(mail.numeric||mail.text.empty()||mail.text.size()>256||mail.text.back()==',') {
      report(op,"REJECTED","invalid_request");return;
    }
    std::map<std::string,std::string> fields;
    bool valid=true;
    std::istringstream in(mail.text);std::string part;
    while(std::getline(in,part,',')) {
      const auto split=part.find('=');
      if(split==std::string::npos){valid=false;break;}
      const auto key=trim(part.substr(0,split)),value=trim(part.substr(split+1));
      if((key!="id"&&key!="action"&&key!="name"&&key!="value"&&key!="type")||!fields.emplace(key,value).second){valid=false;break;}
    }
    if(fields.count("id")&&idValid(fields["id"]))op.id=fields["id"];else valid=false;
    if(fields.count("action")&&(fields["action"]=="read"||fields["action"]=="list"||fields["action"]=="write"))op.action=fields["action"];else valid=false;
    if(fields.count("name"))op.name=fields["name"];
    if(op.action=="list")valid=valid&&!fields.count("name")&&!fields.count("value")&&!fields.count("type");
    else valid=valid&&nameValid(op.name);
    if(op.action=="write") {
      double parsed=0;valid=valid&&fields.count("value")&&parseNumber(fields["value"],parsed)&&parsed>=0&&parsed<=20;
      op.value=static_cast<float>(parsed);
      valid=valid&&(!fields.count("type")||fields["type"]=="REAL32"||fields["type"]=="9");
    } else valid=valid&&!fields.count("value")&&!fields.count("type");
    if(!valid){report(op,"REJECTED","invalid_request");return;}
    if(m_seen.count(op.id)||(m_operation&&m_operation->id==op.id)){report(op,"DUPLICATE","id_already_seen");return;}
    if(m_operation){report(op,"REJECTED","busy");return;}
    const double now=m_services.now(),age=mail.received_at-mail.published_at;
    if(!std::isfinite(now)||!std::isfinite(mail.received_at)||!std::isfinite(mail.published_at)||
       !std::isfinite(age)||age<-.1||age>2||mail.received_at>now+.1||now-mail.received_at>2||mail.published_at<=m_boundary) {
      report(op,"REJECTED","stale_future_or_pre_session_request");return;
    }
    if(m_seen.size()>=128){report(op,"REJECTED","request_history_full");return;}
    op.list_mode=m_list_mode;
    const double list_timeout=m_list_timeout_explicit?m_list_timeout:(op.list_mode==ListMode::Paced?60:20);
    op.expires=mail.published_at+(op.action=="list"?list_timeout:m_timeout);
    if(op.expires<=now){report(op,"REJECTED","expired_request");return;}
    m_seen.insert(op.id);m_operation=std::move(op);
  }
  void onReset(const std::string&) override
  {
    if(m_operation) {
      // A reset may occur between the syscall and its callback. Absence of the
      // receipt in this stage cannot prove that a submitted SET stayed local.
      auto& op=*m_operation;
      if(op.action=="write"&&op.stage==Stage::AwaitInitialSend)op.write_may_have_applied=true;
      finish("CANCELLED",op.write_may_have_applied?"connection_reset_write_may_have_applied":"connection_reset");
    }
    m_services.publish("MAVMOOS_PARAM_SNAPSHOT","{\"schema\":1,\"status\":\"invalidated\",\"reason\":\"connection_reset\",\"count\":0,\"values\":[]}");
    m_boundary=m_services.now();
  }
  void onParameterSent(const ParameterRequest& request,double sent_at,bool accepted) override
  {
    if(!m_operation)return;
    auto& op=*m_operation;
    if(op.stage==Stage::Paced) {
      for(auto& entry:op.outstanding) {
        auto& phase=entry.second;
        if(request.id!=phase.wire_id||!phase.awaiting_receipt)continue;
        if(request.action!=ParameterAction::Read||request.index!=static_cast<int16_t>(entry.first)||!request.name.empty())return;
        phase.awaiting_receipt=false;
        if(!accepted){finish("INCOMPLETE","paced_local_send_rejected_or_failed");return;}
        if(!std::isfinite(sent_at)||sent_at<phase.submitted_at||sent_at>=phase.send_by||sent_at>=op.expires) {
          phase.retry_due=true;report(op,"INDEX_TIMEOUT","paced_send_receipt_outside_window");return;
        }
        phase.accepted=true;phase.sent_at=sent_at;phase.reply_by=std::min(op.expires,sent_at+1);
        report(op,"INDEX_SENT","local_os_acceptance");return;
      }
      return;
    }
    if(request.id!=op.wire_id)return;
    if(!op.recovery_waiting&&op.stage!=Stage::AwaitInitialSend&&op.stage!=Stage::AwaitReadbackSend)return;
    if(request.action==ParameterAction::Write&&accepted)op.write_may_have_applied=true;
    if(!std::isfinite(sent_at)||sent_at<op.sent_at||sent_at>=op.expires){finish("TIMED_OUT","local_send_outside_original_deadline");return;}
    if(!accepted){finish("FAILED","local_send_rejected_or_failed");return;}
    op.sent_at=sent_at;
    if(op.recovery_waiting){op.recovery_waiting=false;report(op,"RECOVERY_SENT","local_os_acceptance");return;}
    if(op.stage==Stage::AwaitInitialSend) {
      op.stage=op.action=="list"?Stage::List:op.action=="write"?Stage::Echo:Stage::Read;
      op.last_value=sent_at;
    } else if(op.stage==Stage::AwaitReadbackSend)op.stage=Stage::Readback;
    else return;
    report(op,"SENT","local_os_acceptance");
  }
  void onMessage(const mavlink_message_t& message,double acquired) override
  {
    if(!m_operation||message.sysid!=1||message.compid!=1)return;
    auto& op=*m_operation;
    if(op.stage==Stage::Paced) { pacedValue(op,message,acquired);return; }
    if(!std::isfinite(acquired)||acquired<=op.sent_at||acquired>=op.expires)return;
    if(op.stage!=Stage::Read&&op.stage!=Stage::List&&op.stage!=Stage::Echo&&op.stage!=Stage::Readback)return;
    if(message.msgid==MAVLINK_MSG_ID_PARAM_ERROR) {
      mavlink_param_error_t error{};mavlink_msg_param_error_decode(&message,&error);
      std::string name(error.param_id,std::find(error.param_id,error.param_id+16,'\0'));
      if((error.target_system!=0&&error.target_system!=245)||(error.target_component!=0&&error.target_component!=191))return;
      if(op.action!="list"&&name==op.name)finish("FAILED","parameter_error_"+std::to_string(error.error));
      return;
    }
    if(message.msgid!=MAVLINK_MSG_ID_PARAM_VALUE)return;
    mavlink_param_value_t v{};mavlink_msg_param_value_decode(&message,&v);
    std::string name(v.param_id,std::find(v.param_id,v.param_id+16,'\0'));
    if(!nameValid(name))return;
    if(op.action!="list"&&name!=op.name)return;
    if(op.stage==Stage::List&&v.param_index==UINT16_MAX) {
      if(validValue(v))publishValue(op,v,name,"unindexed_notification_not_list_progress");
      return;
    }
    if(!validValue(v)){finish(op.action=="list"?"INCOMPLETE":"FAILED","unsupported_or_invalid_c_cast_value");return;}
    if(op.stage==Stage::List) {
      if(v.param_count==0||v.param_count>4096||v.param_index>=v.param_count){publishValue(op,v,name,"invalid_table_shape");finish("INCOMPLETE","invalid_parameter_table_shape");return;}
      if(op.expected&&op.expected!=v.param_count){finish("INCOMPLETE","parameter_count_changed");return;}
      op.expected=v.param_count;
      auto existing=op.entries.find(v.param_index);
      if((existing!=op.entries.end()&&existing->second.name!=name)||
         (op.names.count(name)&&op.names[name]!=v.param_index)){finish("INCOMPLETE","parameter_index_name_changed");return;}
      const bool first=existing==op.entries.end();
      op.entries[v.param_index]={name,v.param_value,v.param_type,acquired};op.names[name]=v.param_index;op.last_value=acquired;
      if(first)publishValue(op,v,name,"list");
      if(op.entries.size()==op.expected) {
        if(!publishSnapshot(op)){finish("INCOMPLETE","snapshot_size_limit");return;}
        finish("COMPLETED","complete_indexed_table_and_snapshot_published");
      }
      return;
    }
    if(op.stage==Stage::Read){publishValue(op,v,name,"read");finish("COMPLETED","post_send_parameter_value");return;}
    if(v.param_type!=MAV_PARAM_TYPE_REAL32||v.param_value!=op.value){publishValue(op,v,name,"mismatch");finish("FAILED","write_or_readback_mismatch");return;}
    if(op.stage==Stage::Echo){publishValue(op,v,name,"write_echo");op.stage=Stage::NeedReadback;report(op,"WRITE_ECHO","matching_value_not_yet_readback");return;}
    op.readback_confirmed=true;
    publishValue(op,v,name,"readback");finish("COMPLETED","explicit_readback_matches");
  }
  void iterate() override
  {
    if(!m_operation)return;
    auto& op=*m_operation;const double now=m_services.now();
    if(!std::isfinite(now)||now<m_boundary){finish("FAILED","clock_discontinuity");return;}
    if(now>=op.expires){finish(op.action=="list"?"INCOMPLETE":"TIMED_OUT","original_operation_deadline");return;}
    const auto& v=m_services.vehicle();
    if(!v.heartbeat_valid||!fresh(v.heartbeat_at,now,2)){finish("FAILED","heartbeat_unavailable");return;}
    if(op.action=="write"&&op.stage==Stage::Queued&&(op.name!="SIM_WIND_SPD"||v.armed||!v.landed_valid||
       !fresh(v.landed_at,now,2)||v.landed_state!=MAV_LANDED_STATE_ON_GROUND||v.in_air)) {
      finish("REJECTED","simulation_wind_write_requires_disarmed_ground");return;
    }
    if(op.stage==Stage::Queued) {
      report(op,"ADMITTED","operation_policy_passed");
      if(op.action=="list"&&op.list_mode==ListMode::Paced) {
        op.stage=Stage::Paced;queueIndex(op,0,false,now);return;
      }
      ParameterRequest r;r.id=op.id+"/initial";r.name=op.name;r.value=op.value;r.expires=op.expires;
      r.action=op.action=="list"?ParameterAction::List:op.action=="write"?ParameterAction::Write:ParameterAction::Read;
      op.stage=Stage::AwaitInitialSend;op.wire_id=r.id;
      if(!m_services.parameter(r)){finish("REJECTED","host_admission_inhibited");return;}
      report(op,"SUBMITTED","host_queue_admitted");return;
    }
    if(op.stage==Stage::Paced) {
      for(auto& entry:op.outstanding) {
        auto& phase=entry.second;
        if((phase.awaiting_receipt&&now>=phase.send_by)||(phase.accepted&&now>=phase.reply_by))
          phase.retry_due=true;
      }
      for(const auto& entry:op.outstanding)if(entry.second.retry_due) {
        if(op.recoveries>=32){finish("INCOMPLETE","bounded_paced_recovery_exhausted");return;}
        queueIndex(op,entry.first,true,now);return;
      }
      if(op.expected&&op.outstanding.size()<4&&op.next_index<op.expected) {
        const unsigned index=op.next_index++;queueIndex(op,index,false,now);
      }
      return;
    }
    if(op.stage==Stage::NeedReadback) {
      ParameterRequest r;r.id=op.id+"/readback";r.name=op.name;r.expires=op.expires;
      op.wire_id=r.id;op.stage=Stage::AwaitReadbackSend;
      if(!m_services.parameter(r)){finish("FAILED","readback_host_admission_inhibited");return;}
      report(op,"READBACK_SUBMITTED","host_queue_admitted");return;
    }
    if(op.stage==Stage::List&&!op.recovery_waiting&&now-op.last_value>=.5&&now-op.last_recovery>=.1) {
      if(op.recoveries>=32){finish("INCOMPLETE","bounded_recovery_exhausted");return;}
      ParameterRequest r;r.id=op.id+"/missing"+std::to_string(++op.recoveries);r.expires=op.expires;
      if(op.expected) {
        unsigned missing=0;while(missing<op.expected&&op.entries.count(missing))++missing;
        r.index=static_cast<int16_t>(missing);
      } else r.action=ParameterAction::List;
      op.wire_id=r.id;op.recovery_waiting=true;op.last_recovery=now;
      if(!m_services.parameter(r)){finish("INCOMPLETE","recovery_host_admission_inhibited");return;}
      report(op,"RECOVERY_SUBMITTED","bounded_missing_index_request");
    }
  }
private:
  void queueIndex(Operation& op,unsigned index,bool retry,double now)
  {
    if(index>=4096||(!op.outstanding.count(index)&&op.outstanding.size()>=4)||op.phase_count>=4096+32) {
      finish("INCOMPLETE","paced_request_limit");return;
    }
    if(retry)++op.recoveries;
    IndexPhase phase;phase.submitted_at=now;phase.send_by=std::min(op.expires,now+1);
    phase.wire_id=op.id+"/index"+std::to_string(index)+"/phase"+std::to_string(++op.phase_count);
    op.outstanding[index]=phase;
    ParameterRequest request;request.id=phase.wire_id;request.index=static_cast<int16_t>(index);
    request.expires=phase.send_by;
    if(!m_services.parameter(request)){finish("INCOMPLETE","paced_host_admission_inhibited");return;}
    report(op,retry?"INDEX_RETRY_SUBMITTED":"INDEX_SUBMITTED","paced_index_request");
  }
  void pacedValue(Operation& op,const mavlink_message_t& message,double acquired)
  {
    if(!std::isfinite(acquired)||acquired>=op.expires)return;
    if(message.msgid==MAVLINK_MSG_ID_PARAM_ERROR) {
      mavlink_param_error_t error{};mavlink_msg_param_error_decode(&message,&error);
      if((error.target_system!=0&&error.target_system!=245)||(error.target_component!=0&&error.target_component!=191)||error.param_index<0)return;
      const auto requested=op.outstanding.find(static_cast<unsigned>(error.param_index));
      if(requested==op.outstanding.end())return;
      const auto& phase=requested->second;
      if(phase.accepted&&!phase.retry_due&&acquired>phase.sent_at&&acquired<phase.reply_by)
        finish("INCOMPLETE","parameter_error_"+std::to_string(error.error));
      return;
    }
    if(message.msgid!=MAVLINK_MSG_ID_PARAM_VALUE)return;
    mavlink_param_value_t v{};mavlink_msg_param_value_decode(&message,&v);
    // Unsolicited/change notifications and completed indices cannot advance a paced table.
    auto requested=op.outstanding.find(v.param_index);
    if(v.param_index==UINT16_MAX||requested==op.outstanding.end()||op.entries.count(v.param_index))return;
    const auto& phase=requested->second;
    // PARAM_VALUE has no request nonce: this proves a post-receipt observation
    // of the requested index, not which same-index request caused it. A retry
    // may leave a bounded, harmless READ already queued in the host.
    if(!phase.accepted||phase.retry_due||acquired<=phase.sent_at||acquired>=phase.reply_by)return;
    const std::string name(v.param_id,std::find(v.param_id,v.param_id+16,'\0'));
    if(!nameValid(name)||!validValue(v)){finish("INCOMPLETE","unsupported_or_invalid_c_cast_value");return;}
    if(v.param_count==0||v.param_count>4096||v.param_index>=v.param_count){finish("INCOMPLETE","invalid_parameter_table_shape");return;}
    if(op.expected&&op.expected!=v.param_count){finish("INCOMPLETE","parameter_count_changed");return;}
    op.expected=v.param_count;
    if(op.names.count(name)&&op.names[name]!=v.param_index){finish("INCOMPLETE","parameter_index_name_changed");return;}
    op.entries[v.param_index]={name,v.param_value,v.param_type,acquired};op.names[name]=v.param_index;
    op.outstanding.erase(requested);publishValue(op,v,name,"paced_list");
    if(op.entries.size()==op.expected) {
      if(!publishSnapshot(op)){finish("INCOMPLETE","snapshot_size_limit");return;}
      finish("COMPLETED","complete_indexed_table_and_snapshot_published");
    }
  }
  static bool fresh(double stamp,double now,double limit){return std::isfinite(stamp)&&stamp<=now&&now-stamp<=limit;}
  void report(const Operation& op,const std::string& status,const std::string& reason)
  {
    std::ostringstream s;s<<"schema=1,id="<<op.id<<",action="<<op.action<<",status="<<status<<",reason="<<reason
      <<",received="<<op.entries.size()<<",expected="<<op.expected<<",recovery_attempts="<<op.recoveries
      <<",list_mode="<<(op.list_mode==ListMode::Paced?"paced":"streamed")
      <<",outstanding="<<op.outstanding.size()<<",request_attempts="<<op.phase_count
      <<",write_may_have_applied="<<(op.write_may_have_applied?"true":"false")
      <<",readback_confirmed="<<(op.readback_confirmed?"true":"false")
      <<",expires="<<std::setprecision(16)<<op.expires<<",at="<<m_services.now();
    m_services.publish("MAVMOOS_PARAM_RESULT",s.str());
  }
  void publishValue(const Operation& op,const mavlink_param_value_t& v,const std::string& name,const std::string& phase)
  {
    std::ostringstream s;s<<"schema=1,id="<<op.id<<",name="<<name<<",value="<<std::setprecision(std::numeric_limits<float>::max_digits10)
      <<v.param_value<<",type="<<unsigned(v.param_type)<<",index="<<v.param_index<<",count="<<v.param_count
      <<",phase="<<phase<<",encoding=ARDUPILOT_C_CAST,integer_precision="
      <<(v.param_type!=MAV_PARAM_TYPE_REAL32&&std::abs(v.param_value)>16777216.?"possibly_lossy":"wire_exact");
    m_services.publish("MAVMOOS_PARAM_VALUE",s.str());
  }
  bool publishSnapshot(const Operation& op)
  {
    // One MOOS value preserves the complete table despite per-key stream coalescing.
    // Names/IDs use restricted ASCII, so no user-controlled JSON escaping is needed.
    std::ostringstream s;
    s<<"{\"schema\":1,\"id\":\""<<op.id<<"\",\"status\":\"complete\",\"encoding\":\"ARDUPILOT_C_CAST\","
      <<"\"time_basis\":\"bridge_monotonic\",\"list_mode\":\""<<(op.list_mode==ListMode::Paced?"paced":"streamed")
      <<"\",\"completed_at\":"<<std::setprecision(16)<<m_services.now()
      <<",\"count\":"<<op.expected<<",\"values\":[";
    bool first=true;
    for(const auto& entry:op.entries) {
      if(!first)s<<',';
      first=false;
      const auto& value=entry.second;
      s<<"{\"index\":"<<entry.first<<",\"name\":\""<<value.name<<"\",\"value\":"
        <<std::setprecision(std::numeric_limits<float>::max_digits10)<<value.value
        <<",\"type\":"<<unsigned(value.type)<<",\"acquired_at\":"<<std::setprecision(16)<<value.acquired_at
        <<",\"integer_precision\":\""
        <<(value.type!=MAV_PARAM_TYPE_REAL32&&std::abs(value.value)>16777216.?"possibly_lossy":"wire_exact")<<"\"}";
    }
    s<<"]}";
    const auto snapshot=s.str();
    if(snapshot.size()>1024*1024)return false;
    m_services.publish("MAVMOOS_PARAM_SNAPSHOT",snapshot);
    return true;
  }
  void finish(const std::string& status,const std::string& reason){report(*m_operation,status,reason);m_operation.reset();}
  Services& m_services;
  std::optional<Operation> m_operation;
  std::set<std::string> m_seen;
  double m_boundary=0,m_timeout=8,m_list_timeout=20;
  ListMode m_list_mode=ListMode::Paced;
  bool m_list_timeout_explicit=false;
};
}
}
extern "C" mavmoos::runtime::Plugin* mavmoos_create_plugin(unsigned abi,mavmoos::runtime::Services* services)
{return abi==mavmoos::runtime::plugin_abi&&services?new mavmoos::runtime::Parameters(*services):nullptr;}
extern "C" void mavmoos_destroy_plugin(mavmoos::runtime::Plugin* plugin){delete plugin;}
