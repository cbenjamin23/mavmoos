#include "FirmwareDiscovery.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
using namespace mavmoos::runtime;
namespace {
void need(bool value,const char* why){if(!value)throw std::runtime_error(why);}
bool contains(const std::string& value,const std::string& text){return value.find(text)!=std::string::npos;}
mavlink_message_t hb(unsigned type=2,unsigned stack=3,unsigned system=1,unsigned component=1)
{
  mavlink_message_t message{};mavlink_status_t codec{};
  mavlink_msg_heartbeat_pack_status(system,component,&codec,&message,type,stack,0,4,MAV_STATE_ACTIVE);return message;
}
mavlink_message_t version(uint32_t raw=0x04070000,uint64_t caps=UINT64_MAX)
{
  mavlink_message_t message{};mavlink_status_t codec{};mavlink_autopilot_version_t value{};
  value.flight_sw_version=raw;value.capabilities=caps;value.uid=1337;
  const std::string custom="04cf6486";std::copy(custom.begin(),custom.end(),value.flight_custom_version);
  mavlink_msg_autopilot_version_encode_status(1,1,&codec,&message,&value);return message;
}
void feed(FirmwareDiscovery& f,const mavlink_message_t& msg,double at)
{f.onMessage(msg,MessageTime{at,at,"READ_ACQUISITION"},at);}
void bind(FirmwareDiscovery& f,double at=100){feed(f,hb(),at);}
std::string learn(FirmwareDiscovery& f,double at=100.1){bind(f,at-.1);feed(f,version(),at);return f.status(at);}
void refresh(FirmwareDiscovery& f,double at){feed(f,hb(),at);}
}
int main()
{
  unsigned count=0;const auto test=[&](const char* name,const std::function<void()>& fn){
    try{fn();++count;std::cout<<"{\"case\":\""<<name<<"\",\"result\":\"PASS\"}\n";}
    catch(const std::exception& e){std::cerr<<name<<": "<<e.what()<<'\n';throw;}
  };
  try {
    test("initial_unknown",[]{FirmwareDiscovery f;auto s=f.status(1);need(contains(s,"WAITING_HEARTBEAT")&&contains(s,"\"reported\":null")&&contains(s,"\"authoritative_for_control\":false"),"initial evidence invented");});
    test("passive_observation",[]{FirmwareDiscovery f;auto s=learn(f);need(contains(s,"\"present\":true")&&contains(s,"\"fresh\":true")&&contains(s,"\"qualification\":\"not_evaluated\""),"passive observation missing");need(!f.take(100.2,false),"passive emitted probe");});
    test("prebound_version",[]{FirmwareDiscovery f;feed(f,version(),100);bind(f,100.1);need(contains(f.status(100.1),"\"present\":false"),"prebound version retained");});
    test("foreign_sources",[]{FirmwareDiscovery f;bind(f);auto v=version();v.sysid=2;feed(f,v,100.1);v.sysid=1;v.compid=2;feed(f,v,100.2);need(contains(f.status(100.2),"\"present\":false")&&contains(f.status(100.2),"\"foreign\":2"),"foreign source bound");});
    test("wrong_heartbeat_identity",[]{FirmwareDiscovery f;bind(f);feed(f,hb(10),100.1);feed(f,version(),100.2);need(contains(f.status(100.2),"WAITING_HEARTBEAT"),"wrong vehicle heartbeat retained binding");});
    test("rover_expected_identity",[]{FirmwareDiscovery f(10,3);feed(f,hb(10),100);feed(f,version(),100.1);need(contains(f.status(100.1),"\"present\":true"),"explicit Rover context could not observe");});
    test("reset_floor",[]{FirmwareDiscovery f;learn(f);f.reset(101,"restart");f.onMessage(hb(),MessageTime{100.9,101.1,"QUEUE_EMPTY_FLOOR"},101.1);feed(f,version(),101.2);need(contains(f.status(101.2),"WAITING_HEARTBEAT"),"old queued heartbeat rebound");});
    test("heartbeat_expiry",[]{FirmwareDiscovery f;learn(f);need(contains(f.status(102.01),"WAITING_HEARTBEAT")&&contains(f.status(102.01),"\"reported\":null"),"stale heartbeat retained version");});
    test("post_gap_new_binding",[]{FirmwareDiscovery f;learn(f);const auto epoch=f.epoch();f.inputBoundary(103);bind(f,103.1);feed(f,version(),103.2);need(f.epoch()>epoch&&contains(f.status(103.2),"\"present\":true"),"fresh session did not rebind");});
    test("invalid_numeric_clocks",[]{for(double bad:std::array<double,4>{-1.,NAN,INFINITY,-INFINITY}){FirmwareDiscovery f;learn(f);need(!f.inputBoundary(bad)&&f.clockFault(),"invalid clock accepted");need(!f.take(101,true),"catchup cleared fault");}});
    test("lower_reset_rejected",[]{FirmwareDiscovery f;learn(f);f.reset(100.05,"lower");need(f.clockFault(),"reset moved reliable time backwards");});
    test("exact_highwater_reset_recovers",[]{FirmwareDiscovery f;learn(f);f.inputBoundary(99);f.reset(100.1,"valid_reset");bind(f,100.2);feed(f,version(),100.3);need(!f.clockFault()&&contains(f.status(100.3),"\"present\":true"),"valid explicit reset did not recover");});
    test("bad_metadata_basis",[]{FirmwareDiscovery f;bind(f);f.onMessage(version(),MessageTime{100.1,100.1,"FABRICATED"},100.1);need(contains(f.status(100.1),"\"present\":false"),"unknown clock basis accepted");});
    test("reversed_metadata",[]{FirmwareDiscovery f;bind(f);f.onMessage(version(),MessageTime{100.2,100.1,"QUEUE_EMPTY_FLOOR"},100.2);need(contains(f.status(100.2),"\"present\":false"),"reversed acquisition accepted");});
    test("future_read_metadata",[]{FirmwareDiscovery f;bind(f);f.onMessage(version(),MessageTime{100.1,100.2,"QUEUE_EMPTY_FLOOR"},100.1);need(contains(f.status(100.1),"\"present\":false"),"future read accepted");});
    test("read_basis_cannot_hide_queue_age",[]{FirmwareDiscovery f;bind(f);f.onMessage(version(),MessageTime{100.1,100.2,"READ_ACQUISITION"},100.2);need(contains(f.status(100.2),"\"present\":false"),"contradictory read basis accepted");});
    test("original_metadata_precision",[]{FirmwareDiscovery f;bind(f);const double at=100.123456789123;f.onMessage(version(),MessageTime{at,100.2,"QUEUE_EMPTY_FLOOR"},100.2);auto s=f.status(100.3);need(contains(s,"100.123456789123")&&contains(s,"\"age_is_upper_bound\":true"),"original acquisition rounded/renewed");std::cout<<"{\"snapshot\":"<<s<<"}\n";});
    test("stale_reference_retained",[]{FirmwareDiscovery f;learn(f);for(int i=101;i<=131;++i)refresh(f,i);auto s=f.status(131);need(contains(s,"\"state\":\"STALE\"")&&contains(s,"\"present\":true")&&contains(s,"\"fresh\":false"),"reference age hidden or renewed");});
    test("capability_uint64_precision",[]{FirmwareDiscovery f;auto s=learn(f);need(contains(s,"\"capabilities_raw_decimal\":\"18446744073709551615\"")&&contains(s,"0xffffffffffffffff"),"capability integer lost precision");});
    test("ardupilot_ascii_prefix",[]{FirmwareDiscovery f;auto s=learn(f);need(contains(s,"\"flight_custom_version_ascii\":\"04cf6486\"")&&contains(s,"not_full_git_revision"),"AP custom version reinterpreted as full Git revision");});
    test("opaque_custom_bytes",[]{FirmwareDiscovery f;bind(f);auto msg=version();mavlink_autopilot_version_t v{};mavlink_msg_autopilot_version_decode(&msg,&v);v.flight_custom_version[2]=255;mavlink_status_t c{};mavlink_msg_autopilot_version_encode_status(1,1,&c,&msg,&v);feed(f,msg,100.1);need(contains(f.status(100.1),"\"flight_custom_version_ascii\":null"),"opaque custom bytes became text");});
    test("zero_version_is_unknown",[]{FirmwareDiscovery f;bind(f);feed(f,version(0),100.1);need(contains(f.status(100.1),"\"version_known\":false"),"zero version invented");});
    test("trimmed_extensions",[]{FirmwareDiscovery f;bind(f);auto v=version();v.len=MAVLINK_MSG_ID_AUTOPILOT_VERSION_MIN_LEN;feed(f,v,100.1);need(contains(f.status(100.1),"\"uid2_extension_bytes_present\":false"),"omitted UID2 became available");});
    test("mavlink1_base_version",[]{FirmwareDiscovery f;bind(f);auto v=version();v.magic=MAVLINK_STX_MAVLINK1;v.len=MAVLINK_MSG_ID_AUTOPILOT_VERSION_MIN_LEN;feed(f,v,100.1);need(contains(f.status(100.1),"\"present\":true")&&contains(f.status(100.1),"\"uid2_extension_bytes_present\":false"),"base v1 decoding failed");});
    test("capability_change_is_not_identity",[]{FirmwareDiscovery f;learn(f);feed(f,version(0x04070000,1),100.2);need(f.identityRevision()==0,"capability update inferred firmware replacement");});
    test("firmware_change_diagnostic",[]{FirmwareDiscovery f;learn(f);feed(f,version(0x040603ff),100.2);need(f.identityRevision()==1&&contains(f.status(100.2),"\"authoritative_for_control\":false"),"identity change missing or granted authority");});
    test("old_version_not_renewed",[]{FirmwareDiscovery f;learn(f,100.2);f.onMessage(version(0x040603ff),MessageTime{100.1,100.3,"QUEUE_EMPTY_FLOOR"},100.3);need(f.identityRevision()==0&&contains(f.status(100.3),"\"patch\":0"),"old acquired version replaced current");});
    test("requests_disabled",[]{FirmwareDiscovery f;bind(f);need(!f.ready(100.1,false)&&!f.take(100.1,false),"disabled mode sends discovery");});
    test("request_shape_original_deadline",[]{FirmwareDiscovery f;bind(f);auto a=f.take(100.1,true);need(a&&a->command==512&&a->message_id==148&&a->attempt==1&&a->expires==110.1&&a->epoch==f.epoch(),"wrong discovery request witness");});
    test("altered_witnesses_rejected",[]{FirmwareDiscovery f;bind(f);auto a=*f.take(100.1,true);for(int n=0;n<6;++n){auto b=a;if(n==0)++b.epoch;if(n==1)++b.request_id;if(n==2)++b.attempt;if(n==3)b.expires+=1;if(n==4)++b.command;if(n==5)++b.message_id;need(!f.eligible(b,100.2,true),"altered work eligible");}need(f.eligible(a,100.2,true),"actual witness rejected");});
    test("local_false_retry_bounded",[]{FirmwareDiscovery f;bind(f);double now=100.1;for(unsigned i=1;i<=3;++i){auto a=f.take(now,true);need(a&&a->attempt==i,"retry count wrong");need(f.complete(*a,now,false).matched_current,"local rejection receipt unmatched");now+=.21;}need(!f.take(now,true)&&contains(f.status(now),"EXHAUSTED"),"retry bound renewed");});
    test("accepted_read_retry_spacing",[]{FirmwareDiscovery f;bind(f);auto a=*f.take(100.1,true);f.complete(a,100.1,true);refresh(f,101);need(!f.ready(102.09,true),"accepted read retried too soon");refresh(f,102.1);auto b=f.take(102.1,true);need(b&&b->expires==a.expires&&b->attempt==2,"retry renewed deadline");});
    test("three_reads_no_response_timeout",[]{FirmwareDiscovery f;bind(f);for(int i=0;i<3;++i){const auto now=100.1+i*2.1;if(i)refresh(f,now-1);refresh(f,now);auto a=f.take(now,true);need(bool(a),"idempotent read missing");f.complete(*a,now,true);}for(int i=105;i<=110;++i)refresh(f,i);f.inputBoundary(110.1);need(contains(f.status(110.1),"TIMED_OUT")&&!f.take(110.1,true),"discovery did not expire");});
    test("ack_is_not_version",[]{FirmwareDiscovery f;bind(f);auto a=*f.take(100.1,true);f.complete(a,100.1,true);mavlink_command_ack_t ack{};ack.command=512;ack.result=MAV_RESULT_ACCEPTED;mavlink_message_t msg{};mavlink_status_t c{};mavlink_msg_command_ack_encode_status(1,1,&c,&msg,&ack);feed(f,msg,100.2);need(contains(f.status(100.2),"\"present\":false")&&contains(f.status(100.2),"WAITING_RESPONSE"),"ACK fabricated version completion");});
    test("response_retires_inflight",[]{FirmwareDiscovery f;bind(f);auto a=*f.take(100.1,true);feed(f,version(),100.2);need(!f.eligible(a,100.2,true)&&!f.take(100.3,true),"received version left old request eligible");});
    test("unsolicited_version_avoids_probe",[]{FirmwareDiscovery f;learn(f);need(!f.take(100.2,true),"already observed version probed");});
    test("reset_retires_witness",[]{FirmwareDiscovery f;bind(f);auto a=*f.take(100.1,true);f.reset(100.2,"restart");bind(f,100.3);need(!f.eligible(a,100.4,true),"old session request survived reset");});
    test("heartbeat_loss_retires_witness",[]{FirmwareDiscovery f;bind(f);auto a=*f.take(100.1,true);f.inputBoundary(102.1);need(!f.eligible(a,102.1,true),"stale binding request survived");});
    test("effects_lost_before_handoff",[]{FirmwareDiscovery f;bind(f);auto a=*f.take(100.1,true);need(!f.eligible(a,100.2,false),"changed host effect permission ignored");});
    test("final_clock_fault_retires_witness",[]{FirmwareDiscovery f;bind(f);auto a=*f.take(100.1,true);need(!f.eligible(a,NAN,true)&&f.clockFault(),"final clock fault ignored");});
    test("late_version_preserves_timeout",[]{FirmwareDiscovery f;bind(f);auto a=*f.take(100.1,true);f.complete(a,100.1,true);for(int i=101;i<=111;++i)refresh(f,i);feed(f,version(),111.1);auto s=f.status(111.1);need(contains(s,"\"present\":true")&&contains(s,"TIMED_OUT"),"late observation rewrote timeout");});
    test("obsolete_local_acceptance_reported",[]{FirmwareDiscovery f;bind(f);auto a=*f.take(100.1,true);f.reset(100.2,"reset_in_send");auto c=f.complete(a,100.3,true);need(c.local_accepted&&!c.matched_current,"obsolete actual return hidden");});
    test("unrepresentable_deadline_denied",[]{FirmwareDiscovery f;bind(f,1e308);need(!f.take(1e308,true)&&contains(f.status(1e308),"UNREPRESENTABLE_DEADLINE"),"rounded infinite request window accepted");});
    test("finite_status_after_invalid_clock",[]{FirmwareDiscovery f;learn(f);f.inputBoundary(NAN);auto s=f.status(INFINITY);need(!contains(s,":nan")&&!contains(s,":inf")&&!contains(s,":-inf")&&contains(s,"CLOCK_FAULT"),"invalid JSON numeric clock published");std::cout<<"{\"snapshot\":"<<s<<"}\n";});
    std::cout<<"{\"result\":\"PASS\",\"groups\":"<<count<<",\"scope\":\"pure discovery state, no socket or vehicle\"}\n";return 0;
  } catch(const std::exception&) {return 1;}
}
