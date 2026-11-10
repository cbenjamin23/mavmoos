#include "Plugin.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>
using namespace mavmoos::runtime;
extern "C" Plugin* mavmoos_create_plugin(unsigned,Services*);
extern "C" void mavmoos_destroy_plugin(Plugin*);
namespace {
void require(bool ok,const std::string& why){if(!ok)throw std::runtime_error(why);}
struct Fake final:Services {
  double clock=100;
  Vehicle state;
  bool accept=true;
  unsigned unrelated_effects=0;
  std::vector<ParameterRequest> requests;
  std::vector<std::string> records;
  Fake(){fresh();}
  void fresh(){state.heartbeat_valid=true;state.heartbeat_at=clock;}
  void ground(){state.armed=false;state.in_air=false;state.landed_valid=true;state.landed_at=clock;state.landed_state=1;}
  double now()const override{return clock;}
  const Vehicle& vehicle()const override{return state;}
  void publish(const std::string& k,const std::string& v)override{records.push_back(k+":"+v);}
  void publish(const std::string&,double)override{}
  bool sendCommand(const CommandRequest&)override{++unrelated_effects;return false;}
  bool acquireNavigation(const std::string&)override{++unrelated_effects;return false;}
  void revokeNavigation(const std::string&)override{++unrelated_effects;}
  bool navigation(const NavigationTarget&)override{++unrelated_effects;return false;}
  bool parameter(const ParameterRequest& r)override{if(accept)requests.push_back(r);return accept;}
  bool contains(const std::string& s)const{for(const auto& r:records)if(r.find(s)!=std::string::npos)return true;return false;}
  unsigned count(const std::string& s)const{unsigned n=0;for(const auto& r:records)if(r.find(s)!=std::string::npos)++n;return n;}
};
struct Rig {
  Fake host;
  std::unique_ptr<Plugin,DestroyPlugin> plugin{mavmoos_create_plugin(plugin_abi,&host),mavmoos_destroy_plugin};
  explicit Rig(bool legacy_streamed=true){if(legacy_streamed)plugin->configure("parameter_list_mode","streamed");}
  void submit(const std::string& text,double age=0,bool numeric=false){
    host.clock+=.01;host.fresh();
    Mail m;m.key="MAVMOOS_PARAM_REQUEST";m.text=text;m.numeric=numeric;
    m.received_at=host.clock;m.published_at=host.clock-age;
    plugin->onMail(m);plugin->iterate();
  }
  void receipt(std::size_t request,bool accepted=true){require(request<host.requests.size(),"no request to acknowledge");host.clock+=.001;plugin->onParameterSent(host.requests[request],host.clock,accepted);}
  void sent(bool accepted=true){receipt(host.requests.size()-1,accepted);}
  void value(const std::string& name,float value=1,uint8_t type=MAV_PARAM_TYPE_REAL32,unsigned index=UINT16_MAX,unsigned count=100,uint8_t source=1){
    host.clock+=.001;
    mavlink_param_value_t v{};std::copy_n(name.data(),std::min(std::size_t(16),name.size()),v.param_id);
    v.param_value=value;v.param_type=type;v.param_index=index;v.param_count=count;
    mavlink_message_t m{};mavlink_msg_param_value_encode(source,1,&m,&v);plugin->onMessage(m,host.clock);
  }
  void advance(double n){host.clock+=n;host.fresh();host.state.landed_at=host.clock;plugin->iterate();}
};
}
int main(){
  using Test=std::pair<const char*,std::function<void()>>;
  const std::vector<Test> tests={
    {"factory_abi_and_module_identity",[]{Rig r;require(!mavmoos_create_plugin(plugin_abi+1,&r.host),"ABI mismatch accepted");require(!mavmoos_create_plugin(plugin_abi,nullptr),"null services accepted");require(r.plugin->name()=="parameters","wrong name");}},
    {"strict_request_tokens_and_names",[]{Rig r;for(const auto& s:{"id=x,action=read,name=ABCDEFGHIJKLMNOPQ","id=x,action=read,name=lower","id=x,action=read,name=X,other=y","id=x,id=y,action=list","id=x,action=list,name=X","id=x,action=write,name=SIM_WIND_SPD,value=nan","id=x,action=write,name=SIM_WIND_SPD,value=1,type=INT32","id=x,action=list,"})r.submit(s);require(r.host.requests.empty(),"invalid request reached wire service");}},
    {"stale_future_numeric_and_retained_requests",[]{Rig r;r.submit("id=stale,action=list",2.1);r.submit("id=future,action=list",-.11);r.submit("id=numeric,action=list",0,true);r.host.clock=100;r.submit("id=retained,action=list",.02);require(r.host.requests.empty(),"invalid timing/type admitted");}},
    {"read_without_helm_armed_home_or_pose",[]{Rig r;r.submit("id=r,action=read,name=FRAME_CLASS");r.sent();r.value("FRAME_CLASS",1,MAV_PARAM_TYPE_INT8);require(r.host.contains("status=COMPLETED")&&r.host.contains("index=65535"),"named read did not complete");require(r.host.unrelated_effects==0,"parameter operation touched navigation/command");}},
    {"receipt_required_and_unrelated_values_ignored",[]{Rig r;r.submit("id=r,action=read,name=FRAME_CLASS");r.value("FRAME_CLASS");require(!r.host.contains("status=COMPLETED"),"reply before send receipt completed");r.sent();r.value("OTHER");r.value("FRAME_CLASS",1,9,0,100,2);require(!r.host.contains("status=COMPLETED"),"unrelated response completed");r.value("FRAME_CLASS");require(r.host.contains("status=COMPLETED"),"valid response absent");}},
    {"exact_sixteen_byte_name",[]{Rig r;r.submit("id=r,action=read,name=ABCDEFGHIJKLMNOP");r.sent();r.value("ABCDEFGHIJKLMNOP");require(r.host.contains("status=COMPLETED"),"16-byte non-null name unsupported");}},
    {"list_out_of_order_and_duplicate",[]{Rig r;r.submit("id=l,action=list");r.sent();r.value("C",3,9,2,3);r.value("A",1,9,0,3);r.value("C",3,9,2,3);require(!r.host.contains("status=COMPLETED"),"partial list completed");r.value("B",2,9,1,3);require(r.host.contains("received=3,expected=3")&&r.host.count("MAVMOOS_PARAM_VALUE:")==3,"list accounting wrong");}},
    {"complete_snapshot_is_atomic_bounded_and_reset_invalidated",[]{
      Rig r;r.submit("id=full,action=list");r.sent();
      for(unsigned i=0;i<4095;++i)r.value("PARAM_"+std::to_string(i),static_cast<float>(i),9,i,4096);
      require(!r.host.contains("MAVMOOS_PARAM_SNAPSHOT:"),"partial table published as complete snapshot");
      r.value("PARAM_4095",4095,9,4095,4096);
      require(r.host.count("MAVMOOS_PARAM_SNAPSHOT:")==1,"complete table not published atomically");
      const auto& snapshot=r.host.records[r.host.records.size()-2];
      require(snapshot.size()<1024*1024&&snapshot.find("\"count\":4096")!=std::string::npos,"snapshot not bounded/tagged");
      std::size_t at=0;unsigned rows=0;while((at=snapshot.find("\"index\":",at))!=std::string::npos){++rows;++at;}
      require(rows==4096&&snapshot.find("PARAM_4095")!=std::string::npos,"snapshot lost indexed entries");
      r.plugin->onReset("test");require(r.host.contains("\"status\":\"invalidated\""),"reset retained valid cache snapshot");
    }},
    {"list_missing_index_bounded_recovery",[]{Rig r;r.submit("id=l,action=list");r.sent();r.value("A",1,9,0,3);r.value("C",3,9,2,3);const double deadline=r.host.requests[0].expires;r.advance(.6);require(r.host.requests.back().index==1&&r.host.requests.back().action==ParameterAction::Read,"missing index not requested");require(r.host.requests.back().expires==deadline,"recovery extended original expiry");r.sent();r.value("B",2,9,1,3);require(r.host.contains("status=COMPLETED"),"recovered list incomplete");}},
    {"list_table_change_rejected",[]{Rig r;r.submit("id=l,action=list");r.sent();r.value("A",1,9,0,3);r.value("B",2,9,1,4);require(r.host.contains("reason=parameter_count_changed"),"changed count accepted");}},
    {"list_index_name_inconsistency_rejected",[]{Rig r;r.submit("id=l,action=list");r.sent();r.value("A",1,9,0,3);r.value("A",1,9,1,3);require(r.host.contains("reason=parameter_index_name_changed"),"same name at different index accepted");}},
    {"list_shape_limits",[]{for(unsigned count:{0u,4097u}){Rig r;r.submit("id=l,action=list");r.sent();r.value("A",1,9,0,count);require(r.host.contains("status=INCOMPLETE"),"invalid table count accepted");}Rig r;r.submit("id=l,action=list");r.sent();r.value("A",1,9,3,3);require(r.host.contains("status=INCOMPLETE"),"out-of-range list index accepted");}},
    {"unindexed_notifications_do_not_corrupt_list",[]{Rig r;r.submit("id=l,action=list");r.sent();r.value("A",1,9,0,2);r.value("CHANGED",1,9,UINT16_MAX,2);require(!r.host.contains("status=INCOMPLETE")&&!r.host.contains("status=COMPLETED"),"unindexed change notification corrupted list");r.value("B",2,9,1,2);require(r.host.contains("status=COMPLETED")&&r.host.contains("received=2,expected=2"),"list not complete after indexed values");}},
    {"unsupported_unindexed_notification_cannot_fail_streamed_inventory",[]{
      Rig r;r.submit("id=l,action=list");r.sent();r.value("A",1,9,0,2);
      r.value("OTHER",1,MAV_PARAM_TYPE_UINT32,UINT16_MAX,2);
      require(!r.host.contains("status=INCOMPLETE")&&!r.host.contains("name=OTHER"),"unsupported change notification interrupted inventory");
      r.value("B",2,9,1,2);
      require(r.host.contains("status=COMPLETED")&&r.host.contains("received=2,expected=2"),"list lost valid indexed values");
    }},
    {"bounded_recovery_without_any_responses",[]{Rig r;r.submit("id=l,action=list");r.sent();const auto expiry=r.host.requests[0].expires;for(unsigned i=0;i<33;++i){r.advance(.51);if(r.host.requests.size()>i+1)r.sent();}require(r.host.requests.size()==33&&r.host.contains("reason=bounded_recovery_exhausted"),"unbounded recovery");for(const auto& q:r.host.requests)require(q.expires==expiry,"retry renewed deadline");}},
    {"original_operation_deadline",[]{Rig r;r.submit("id=r,action=read,name=FRAME_CLASS");r.sent();r.advance(8.1);require(r.host.contains("status=TIMED_OUT"),"missing read never expired");r.value("FRAME_CLASS");require(!r.host.contains("status=COMPLETED"),"late reply completed expired operation");}},
    {"late_and_failed_send_receipts",[]{Rig r;r.submit("id=r,action=list");r.sent(false);require(r.host.contains("reason=local_send_rejected_or_failed"),"local failure hidden");Rig late;late.submit("id=r,action=read,name=FRAME_CLASS");late.host.clock+=9;late.sent();require(late.host.contains("reason=local_send_outside_original_deadline"),"late accepted send renewed deadline");}},
    {"late_accepted_write_retains_remote_uncertainty",[]{Rig r;r.host.ground();r.submit("id=w,action=write,name=SIM_WIND_SPD,value=0.25");r.host.clock+=9;r.sent();require(r.host.contains("status=TIMED_OUT")&&r.host.contains("write_may_have_applied=true,readback_confirmed=false"),"late accepted SET claimed no remote effect");}},
    {"write_is_allowlisted_disarmed_ground_only",[]{Rig r;r.host.ground();r.host.state.armed=true;r.submit("id=w1,action=write,name=SIM_WIND_SPD,value=0.25");r.host.state.armed=false;r.host.state.landed_valid=false;r.submit("id=w2,action=write,name=SIM_WIND_SPD,value=0.25");r.host.ground();r.submit("id=w3,action=write,name=ATC_RAT_RLL_P,value=0.25");require(r.host.requests.empty(),"unsafe write submitted");}},
    {"write_echo_is_not_completion_and_explicit_readback",[]{Rig r;r.host.ground();r.submit("id=w,action=write,name=SIM_WIND_SPD,value=0.25");r.sent();const auto expiry=r.host.requests[0].expires;r.value("SIM_WIND_SPD",.25);require(r.host.contains("status=WRITE_ECHO")&&!r.host.contains("status=COMPLETED"),"echo falsely completed write");r.plugin->iterate();require(r.host.requests.back().action==ParameterAction::Read&&r.host.requests.back().expires==expiry,"readback not inherited deadline");r.value("SIM_WIND_SPD",.25);require(!r.host.contains("status=COMPLETED"),"duplicate echo completed pre-send readback");r.sent();r.value("SIM_WIND_SPD",.25);require(r.host.contains("reason=explicit_readback_matches"),"explicit readback failed");}},
    {"write_mismatch_is_terminal",[]{Rig r;r.host.ground();r.submit("id=w,action=write,name=SIM_WIND_SPD,value=0.25");r.sent();r.value("SIM_WIND_SPD",0);require(r.host.contains("reason=write_or_readback_mismatch"),"failed set hidden");require(r.host.requests.size()==1,"write retried implicitly");}},
    {"readback_can_resolve_write_after_external_arm",[]{Rig r;r.host.ground();r.submit("id=w,action=write,name=SIM_WIND_SPD,value=0.25");r.sent();r.value("SIM_WIND_SPD",.25);r.host.state.armed=true;r.host.state.landed_valid=false;r.plugin->iterate();require(r.host.requests.size()==2&&r.host.requests.back().action==ParameterAction::Read,"safe readback inhibited by post-write flight state");r.sent();r.value("SIM_WIND_SPD",.25);require(r.host.contains("status=COMPLETED"),"readback failed after state change");}},
    {"reset_cancels_and_rejects_retained_work",[]{Rig r;r.host.ground();r.submit("id=w,action=write,name=SIM_WIND_SPD,value=0.25");r.sent();r.plugin->onReset("test");r.value("SIM_WIND_SPD",.25);require(r.host.contains("status=CANCELLED")&&r.host.contains("write_may_have_applied")&&!r.host.contains("status=COMPLETED"),"reset lost uncertainty");r.submit("id=w,action=write,name=SIM_WIND_SPD,value=0.25");require(r.host.contains("status=DUPLICATE"),"old id replayed");}},
    {"c_cast_types_and_lossy_integer_labelling",[]{Rig r;r.submit("id=r,action=read,name=BIG_INT");r.sent();r.value("BIG_INT",33554432,MAV_PARAM_TYPE_INT32);require(r.host.contains("integer_precision=possibly_lossy"),"lossy int claimed exact");Rig bad;bad.submit("id=r,action=read,name=BAD");bad.sent();bad.value("BAD",1.5,MAV_PARAM_TYPE_INT16);require(bad.host.contains("status=FAILED"),"fractional integer accepted");}},
    {"parameter_error_target_and_name_filtering",[]{Rig r;r.submit("id=r,action=read,name=NO_SUCH_PARAM");r.sent();mavlink_param_error_t e{};std::copy_n("NO_SUCH_PARAM",13,e.param_id);e.target_system=99;e.target_component=191;e.error=1;e.param_index=-1;mavlink_message_t m{};mavlink_msg_param_error_encode(1,1,&m,&e);r.host.clock+=.01;r.plugin->onMessage(m,r.host.clock);require(!r.host.contains("status=FAILED"),"foreign target error accepted");e.target_system=245;mavlink_msg_param_error_encode(1,1,&m,&e);r.plugin->onMessage(m,r.host.clock);require(r.host.contains("reason=parameter_error_1"),"parameter error not surfaced");}},
    {"host_backpressure_and_duplicate_busy",[]{Rig r;r.host.accept=false;r.submit("id=no,action=list");require(r.host.contains("reason=host_admission_inhibited"),"host rejection hidden");r.host.accept=true;r.submit("id=yes,action=list");r.submit("id=yes,action=list");r.submit("id=busy,action=list");require(r.host.contains("status=DUPLICATE")&&r.host.contains("reason=busy")&&r.host.requests.size()==1,"duplicate/busy admitted");}},
    {"paced_default_seed_and_original_timeout",[]{
      Rig r(false);r.submit("id=p,action=list");
      require(r.host.requests.size()==1,"seed not queued exactly once");const auto q=r.host.requests[0];
      require(q.action==ParameterAction::Read&&q.index==0&&q.name.empty(),"paced seed is not indexed READ zero");
      require(std::abs(q.expires-101.01)<1e-6&&r.host.contains("expires=160.01"),"phase/original deadline not distinct");
      r.value("P0",0,9,0,2);r.value("P1",1,9,1,2);
      require(r.host.count("MAVMOOS_PARAM_VALUE:")==0,"unsent/unrequested row advanced list");
      r.sent();r.value("CHANGE",1,9,UINT16_MAX,2);r.value("P1",1,9,1,2);r.value("P0",0,9,0,2);
      require(r.host.count("MAVMOOS_PARAM_VALUE:")==1&&!r.host.contains("status=COMPLETED"),"seed/notification gating wrong");
      r.advance(.01);r.sent();r.value("P1",1,9,1,2);
      require(r.host.contains("status=COMPLETED")&&r.host.contains("\"list_mode\":\"paced\""),"paced table snapshot absent");
    }},
    {"paced_configuration_order_and_operation_snapshot",[]{
      for(bool timeout_first:{false,true}) {
        Rig r(false);
        if(timeout_first)r.plugin->configure("parameter_list_timeout","7");
        r.plugin->configure("parameter_list_mode","streamed");
        if(!timeout_first)r.plugin->configure("parameter_list_timeout","7");
        r.submit("id=ordered,action=list");require(r.host.requests[0].action==ParameterAction::List&&std::abs(r.host.requests[0].expires-107.01)<1e-6,"explicit timeout depends on config order");
      }
      Rig legacy(false);legacy.plugin->configure("parameter_list_mode","streamed");legacy.submit("id=legacy,action=list");
      require(std::abs(legacy.host.requests[0].expires-120.01)<1e-6,"streamed default timeout changed");
      Rig r(false);r.submit("id=frozen,action=list");r.plugin->configure("parameter_list_mode","streamed");r.plugin->configure("parameter_list_timeout","1");
      r.sent();r.value("P0",0,9,0,2);r.advance(.01);
      require(r.host.requests.back().action==ParameterAction::Read&&r.host.requests.back().index==1&&r.host.contains("expires=160.01"),"active operation followed changed mode/timeout");
      bool bad=false;try{r.plugin->configure("parameter_list_mode","bulk");}catch(const std::invalid_argument&){bad=true;}require(bad,"unknown list mode accepted");
    }},
    {"paced_window_counts_pending_receipts_and_one_phase_per_iteration",[]{
      Rig r(false);r.submit("id=w,action=list");r.sent();r.value("P0",0,9,0,8);
      for(unsigned n=1;n<=4;++n){const auto before=r.host.requests.size();r.advance(.01);require(r.host.requests.size()==before+1&&r.host.requests.back().index==static_cast<int16_t>(n),"phase budget/index order wrong");}
      r.advance(.01);require(r.host.requests.size()==5,"more than four unreceipted indices admitted");
      r.value("P1",1,9,1,8);require(r.host.count("MAVMOOS_PARAM_VALUE:")==1,"unreceipted index counted");
      // A newer phase receipt must not reject the older outstanding phase's reply.
      r.receipt(1);r.receipt(2);r.value("P1",1,9,1,8);r.value("P1",1,9,1,8);
      require(r.host.count("MAVMOOS_PARAM_VALUE:")==2,"duplicate or per-operation send time corrupted progress");
      r.advance(.01);require(r.host.requests.size()==6&&r.host.requests.back().index==5,"released slot not filled");
      for(const auto& q:r.host.requests)require(q.action==ParameterAction::Read,"paced mode emitted full-list or SET");
    }},
    {"paced_requested_out_of_order_completion_and_snapshot_order",[]{
      Rig r(false);r.submit("id=order,action=list");r.sent();r.value("P0",0,9,0,5);
      for(unsigned n=1;n<5;++n)r.advance(.01);
      for(unsigned n=1;n<5;++n)r.receipt(n);
      for(unsigned n:{4u,2u,1u,3u})r.value("P"+std::to_string(n),n,9,n,5);
      require(r.host.contains("status=COMPLETED")&&r.host.count("MAVMOOS_PARAM_VALUE:")==5,"out-of-order requested rows failed");
      std::string snapshot;for(const auto& s:r.host.records)if(s.find("MAVMOOS_PARAM_SNAPSHOT:")==0)snapshot=s;
      require(snapshot.find("\"index\":0")<snapshot.find("\"index\":4"),"snapshot not indexed order");
    }},
    {"paced_exact_phase_receipt_and_late_retry_response",[]{
      Rig r(false);r.submit("id=retry,action=list");const auto old=r.host.requests[0];
      r.advance(1.01);require(r.host.requests.size()==2&&r.host.requests[1].id!=old.id,"receipt timeout did not create a distinct phase");
      r.plugin->onParameterSent(old,r.host.clock,true);r.value("P0",0,9,0,1);
      require(!r.host.contains("status=COMPLETED"),"old phase receipt unlocked retry");
      auto wrong=r.host.requests[1];wrong.index=1;r.plugin->onParameterSent(wrong,r.host.clock,true);
      r.value("P0",0,9,0,1);require(!r.host.contains("status=COMPLETED"),"wrong-index callback unlocked phase");
      r.sent();r.value("P0",0,9,0,1);require(r.host.contains("status=COMPLETED"),"current accepted phase failed");
    }},
    {"paced_response_timeout_and_fixed_whole_deadline",[]{
      Rig r(false);r.plugin->configure("parameter_list_timeout","3");r.submit("id=deadline,action=list");
      r.host.clock+=.5;r.sent();const double first_send=r.host.clock;
      r.host.clock=first_send+.999;r.host.fresh();r.plugin->iterate();require(r.host.requests.size()==1,"response budget measured from submission instead of send");
      r.advance(.002);require(r.host.requests.size()==2,"response RTO did not retry");
      require(r.host.requests[1].expires<103.01&&r.host.contains("expires=103.01"),"retry renewed original bound");
      r.host.clock=103.01;r.host.fresh();r.plugin->iterate();r.value("P0",0,9,0,1);
      require(r.host.contains("reason=original_operation_deadline")&&!r.host.contains("status=COMPLETED"),"original deadline renewed by retry");
    }},
    {"paced_total_retry_cap_includes_seed",[]{
      Rig r(false);r.submit("id=silent,action=list");
      for(unsigned n=0;n<33;++n)r.advance(1.01);
      require(r.host.requests.size()==33&&r.host.contains("reason=bounded_paced_recovery_exhausted"),"seed retry cap unbounded");
      for(const auto& q:r.host.requests)require(q.action==ParameterAction::Read&&q.index==0&&q.expires<=160.01,"seed retry changed request or original deadline");
    }},
    {"paced_invalid_shape_value_and_table_change",[]{
      for(unsigned count:{0u,4097u}){Rig r(false);r.submit("id=bad,action=list");r.sent();r.value("P0",0,9,0,count);require(r.host.contains("reason=invalid_parameter_table_shape"),"invalid seed count accepted");}
      for(bool invalid_name:{false,true}){Rig r(false);r.submit("id=bad,action=list");r.sent();r.value(invalid_name?"lower":"P0",1.5,MAV_PARAM_TYPE_INT16,0,1);require(r.host.contains("status=INCOMPLETE"),"unsupported data accepted");}
      for(bool changed_count:{false,true}){Rig r(false);r.submit("id=changed,action=list");r.sent();r.value("P0",0,9,0,2);r.advance(.01);r.sent();r.value(changed_count?"P1":"P0",1,9,1,changed_count?3:2);require(r.host.contains(changed_count?"reason=parameter_count_changed":"reason=parameter_index_name_changed"),"changed table accepted");}
    }},
    {"paced_indexed_errors_require_current_receipt_and_target",[]{
      Rig r(false);r.submit("id=missing,action=list");mavlink_param_error_t e{};e.param_index=0;e.error=1;e.target_system=245;e.target_component=191;
      mavlink_message_t m{};mavlink_msg_param_error_encode(1,1,&m,&e);r.host.clock+=.001;r.plugin->onMessage(m,r.host.clock);
      require(!r.host.contains("status=INCOMPLETE"),"pre-receipt error terminated operation");r.sent();e.target_system=99;mavlink_msg_param_error_encode(1,1,&m,&e);r.host.clock+=.001;r.plugin->onMessage(m,r.host.clock);
      require(!r.host.contains("status=INCOMPLETE"),"foreign target error terminated operation");e.target_system=245;mavlink_msg_param_error_encode(1,1,&m,&e);r.host.clock+=.001;r.plugin->onMessage(m,r.host.clock);
      require(r.host.contains("reason=parameter_error_1"),"matching indexed error not explicit");
    }},
    {"paced_reset_discards_window_and_local_failures_are_terminal",[]{
      Rig r(false);r.submit("id=reset,action=list");const auto old=r.host.requests[0];r.sent();r.value("P0",0,9,0,3);r.advance(.01);r.plugin->onReset("test");
      r.plugin->onParameterSent(old,r.host.clock,true);r.value("P1",1,9,1,3);r.advance(.01);
      require(r.host.requests.size()==2&&r.host.contains("status=CANCELLED")&&!r.host.contains("status=COMPLETED"),"reset retained outstanding work");
      r.submit("id=reset,action=list");require(r.host.contains("status=DUPLICATE"),"reset erased request history");
      Rig rejected(false);rejected.host.accept=false;rejected.submit("id=rejected,action=list");require(rejected.host.contains("reason=paced_host_admission_inhibited"),"host admission hidden");
      Rig failed(false);failed.submit("id=failed,action=list");failed.sent(false);require(failed.host.contains("reason=paced_local_send_rejected_or_failed"),"local I/O failure hidden");
    }},
    {"paced_late_local_failure_is_terminal_without_retry",[]{
      Rig r(false);r.submit("id=latefailed,action=list");r.host.clock+=1.1;r.sent(false);r.advance(.01);
      require(r.host.requests.size()==1&&r.host.contains("reason=paced_local_send_rejected_or_failed")&&!r.host.contains("INDEX_RETRY_SUBMITTED"),"known local failure was retried after receipt deadline");
    }},
    {"paced_full_table_has_bounded_initial_requests_and_snapshot",[]{
      Rig r(false);r.submit("id=fullpaced,action=list");
      for(unsigned index=0;index<4096;++index){require(r.host.requests.size()==index+1&&r.host.requests.back().index==static_cast<int16_t>(index),"initial indexed coverage wrong");r.sent();r.value("PARAM_"+std::to_string(index),index,9,index,4096);if(index<4095)r.advance(.001);}
      require(r.host.requests.size()==4096&&r.host.contains("status=COMPLETED")&&r.host.contains("\"count\":4096"),"max table incomplete or unbounded");
    }},
    {"bounded_request_history",[]{Rig r;for(unsigned i=0;i<128;++i){r.submit("id=r"+std::to_string(i)+",action=read,name=FRAME_CLASS");r.sent();r.value("FRAME_CLASS");}r.submit("id=overflow,action=list");require(r.host.requests.size()==128&&r.host.contains("reason=request_history_full"),"request history unbounded");}}
  };
  unsigned passed=0;
  try {for(const auto& test:tests){test.second();++passed;std::cout<<"PASS "<<test.first<<'\n';}}
  catch(const std::exception& error){std::cerr<<"FAIL "<<tests[passed].first<<": "<<error.what()<<'\n';return 1;}
  std::cout<<"PASS "<<passed<<" parameter cases\n";return 0;
}
