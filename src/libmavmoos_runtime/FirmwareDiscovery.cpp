/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: FirmwareDiscovery.cpp                           */
/*    DATE: November 4, 2026                                */
/************************************************************/
#include "FirmwareDiscovery.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace mavmoos::runtime {
namespace {
std::string boolean(bool v) { return v?"true":"false"; }
std::string number(double v) { std::ostringstream out;out<<std::setprecision(17)<<v;return out.str(); }
std::string quote(const std::string& value)
{
  constexpr char hex[]="0123456789abcdef";std::string result="\"";
  for(unsigned char c:value) {
    if(c=='"'||c=='\\'){result+='\\';result+=char(c);}
    else if(c<32||c>=127){result+="\\u00";result+=hex[c>>4];result+=hex[c&15];}
    else result+=char(c);
  }
  return result+'"';
}
template<std::size_t N> std::string hexBytes(const uint8_t (&bytes)[N])
{
  std::ostringstream out;out<<std::hex<<std::setfill('0');
  for(const auto b:bytes)out<<std::setw(2)<<unsigned(b);
  return out.str();
}
template<std::size_t N> std::string ascii(const uint8_t (&bytes)[N])
{
  std::size_t end=N;while(end&&bytes[end-1]==0)--end;
  if(!end)return "null";
  for(std::size_t i=0;i<end;++i)if(bytes[i]<32||bytes[i]>126)return "null";
  return quote(std::string(reinterpret_cast<const char*>(bytes),end));
}
bool sameIdentity(const mavlink_autopilot_version_t& a,const mavlink_autopilot_version_t& b)
{
  // Capability changes are observations, not proof of a different firmware.
  return a.flight_sw_version==b.flight_sw_version&&
    std::equal(std::begin(a.flight_custom_version),std::end(a.flight_custom_version),std::begin(b.flight_custom_version))&&
    a.board_version==b.board_version&&a.vendor_id==b.vendor_id&&a.product_id==b.product_id;
}
}
FirmwareDiscovery::FirmwareDiscovery(unsigned type,unsigned autopilot,uint8_t system,uint8_t component)
  :m_expected_type(type),m_expected_autopilot(autopilot),m_system(system),m_component(component) {}

void FirmwareDiscovery::invalidate(const std::string& reason,bool fault)
{
  if(m_epoch==std::numeric_limits<uint64_t>::max())fault=true;
  else ++m_epoch;
  m_clock_fault=fault;m_bound=m_present=m_started=false;m_inflight.reset();
  m_bound_at=m_heartbeat_at=-1;m_version={};m_payload_length=m_magic=0;
  m_attempts=0;m_expires=m_next_attempt=0;m_reason=reason.substr(0,128);
  m_request_state=fault?"CLOCK_FAULT":"NOT_STARTED";
}
bool FirmwareDiscovery::clock(double now)
{
  if(m_clock_fault)return false;
  if(!std::isfinite(now)||now<0||now<m_now) {
    invalidate("clock_fault",true);return false;
  }
  m_now=now;return true;
}
void FirmwareDiscovery::reset(double now,const std::string& reason)
{
  const bool valid=std::isfinite(now)&&now>=0&&now>=m_now&&m_epoch<std::numeric_limits<uint64_t>::max();
  invalidate(valid?reason:"clock_fault",!valid);
  if(valid){m_now=now;m_floor=std::max(m_floor,now);}
}
bool FirmwareDiscovery::inputBoundary(double now)
{
  if(!clock(now))return false;
  if(m_bound&&now-m_heartbeat_at>heartbeat_timeout) {
    invalidate("heartbeat_stale");m_floor=std::max(m_floor,now);
  }
  if(m_started&&m_expires>0&&now>=m_expires&&m_request_state!="OBSERVED"&&m_request_state!="EXHAUSTED") {
    m_request_state="TIMED_OUT";m_inflight.reset();
  }
  return true;
}
bool FirmwareDiscovery::metadata(const MessageTime& t,double maximum_age)
{
  const bool known=t.age_basis=="QUEUE_EMPTY_FLOOR"||
    (t.age_basis=="READ_ACQUISITION"&&t.effective_at==t.read_at);
  if(!known||!std::isfinite(t.effective_at)||!std::isfinite(t.read_at)||
     t.effective_at<0||t.effective_at<=m_floor||t.read_at<t.effective_at||
     t.read_at>m_now||m_now-t.effective_at>maximum_age) {++m_rejected;return false;}
  return true;
}
void FirmwareDiscovery::onMessage(const mavlink_message_t& msg,const MessageTime& time,double now)
{
  if(!inputBoundary(now))return;
  if(msg.msgid!=MAVLINK_MSG_ID_HEARTBEAT&&msg.msgid!=MAVLINK_MSG_ID_AUTOPILOT_VERSION)return;
  if(msg.sysid!=m_system||msg.compid!=m_component){++m_foreign;return;}
  if(!metadata(time,msg.msgid==MAVLINK_MSG_ID_HEARTBEAT?heartbeat_timeout:version_freshness))return;
  if(msg.msgid==MAVLINK_MSG_ID_HEARTBEAT) {
    mavlink_heartbeat_t hb{};mavlink_msg_heartbeat_decode(&msg,&hb);
    if(hb.type!=m_expected_type||hb.autopilot!=m_expected_autopilot){invalidate("identity_mismatch");m_floor=now;return;}
    if(time.effective_at<m_heartbeat_at){++m_rejected;return;}
    if(!m_bound){m_bound=true;m_bound_at=time.effective_at;m_reason="waiting_for_version";}
    m_heartbeat_at=time.effective_at;return;
  }
  if(!m_bound||time.effective_at<m_bound_at){++m_prebound;return;}
  if(m_present&&time.effective_at<m_effective_at){++m_rejected;return;}
  mavlink_autopilot_version_t value{};mavlink_msg_autopilot_version_decode(&msg,&value);
  if(m_present&&!sameIdentity(m_version,value))++m_identity_revision;
  m_present=true;m_version=value;m_effective_at=time.effective_at;m_read_at=time.read_at;
  m_age_basis=time.age_basis;m_payload_length=msg.len;m_magic=msg.magic;++m_observation_id;
  m_reason="observed";
  // A late observation remains useful, but cannot rewrite a terminal timeout.
  if(!m_started||now<m_expires) {
    m_request_state="OBSERVED";m_inflight.reset();
  }
}
bool FirmwareDiscovery::advance(double now,bool effects)
{
  m_effects=effects;
  if(!inputBoundary(now))return false;
  if(!effects||!m_bound||m_present)return false;
  if(!m_started) {
    const auto until=now+request_lifetime;
    if(!std::isfinite(until)||until<=now||now+response_spacing<=now||now+local_rejection_backoff<=now) {
      m_request_state="UNREPRESENTABLE_DEADLINE";m_started=true;return false;
    }
    m_started=true;m_expires=until;m_next_attempt=now;m_request_state="QUEUED";
  }
  return now<m_expires&&m_request_state!="EXHAUSTED"&&m_request_state!="TIMED_OUT"&&
    !m_inflight&&m_attempts<max_attempts&&now>=m_next_attempt;
}
bool FirmwareDiscovery::ready(double now,bool effects){return advance(now,effects);}
std::optional<FirmwareDiscovery::Attempt> FirmwareDiscovery::take(double now,bool effects)
{
  if(!advance(now,effects))return {};
  if(m_request_id==std::numeric_limits<uint64_t>::max()){m_request_state="EXHAUSTED";return {};}
  m_inflight=Attempt{m_epoch,++m_request_id,++m_attempts,m_expires};
  m_request_state="IN_FLIGHT";return m_inflight;
}
bool FirmwareDiscovery::matches(const Attempt& a) const
{
  return m_inflight&&a.epoch==m_epoch&&a.epoch==m_inflight->epoch&&a.request_id==m_inflight->request_id&&
    a.attempt==m_inflight->attempt&&a.expires==m_inflight->expires&&
    a.command==MAV_CMD_REQUEST_MESSAGE&&a.message_id==MAVLINK_MSG_ID_AUTOPILOT_VERSION;
}
bool FirmwareDiscovery::eligible(const Attempt& a,double now,bool effects)
{
  if(!inputBoundary(now))return false;
  return effects&&m_bound&&!m_present&&matches(a)&&now<a.expires;
}
FirmwareDiscovery::Completion FirmwareDiscovery::complete(const Attempt& a,double now,bool accepted)
{
  Completion result;result.local_accepted=accepted;result.clock_valid=inputBoundary(now);
  result.late=result.clock_valid&&now>=a.expires;
  if(!result.clock_valid||!matches(a)){result.state=m_request_state;return result;}
  result.matched_current=true;m_inflight.reset();
  if(accepted){++m_local_accepted;m_next_attempt=now+response_spacing;m_request_state="WAITING_RESPONSE";}
  else {++m_local_rejected;m_next_attempt=now+local_rejection_backoff;m_request_state=m_attempts>=max_attempts?"EXHAUSTED":"RETRY_PENDING";}
  if(result.late)m_request_state="TIMED_OUT";
  result.state=m_request_state;return result;
}
std::string FirmwareDiscovery::status(double now)
{
  inputBoundary(now);
  const bool current=m_bound&&m_present&&!m_clock_fault;
  const double age=current?m_now-m_effective_at:0;
  std::ostringstream out;out<<std::setprecision(17)
    <<"{\"schema\":1,\"epoch\":"<<m_epoch<<",\"state\":"<<quote(m_clock_fault?"CLOCK_FAULT":!m_bound?"WAITING_HEARTBEAT":!m_present?"WAITING_VERSION":age>version_freshness?"STALE":"OBSERVED")
    <<",\"reason\":"<<quote(m_reason)<<",\"read_only\":true,\"authoritative_for_control\":false"
    <<",\"qualification\":\"not_evaluated\",\"source_age\":\"unknown\",\"system_id\":"<<unsigned(m_system)<<",\"component_id\":"<<unsigned(m_component)
    <<",\"present\":"<<boolean(current)<<",\"fresh\":"<<boolean(current&&age<=version_freshness)
    <<",\"observation_id\":"<<m_observation_id<<",\"identity_revision\":"<<m_identity_revision
    <<",\"effective_at\":"<<(current?number(m_effective_at):"null")<<",\"read_at\":"<<(current?number(m_read_at):"null")
    <<",\"age_s\":"<<(current?number(age):"null")<<",\"age_basis\":"<<quote(m_age_basis)
    <<",\"age_is_upper_bound\":"<<boolean(current&&m_age_basis=="QUEUE_EMPTY_FLOOR")
    <<",\"retained_until_session_reset\":true,\"reported\":";
  if(!current)out<<"null";
  else {
    std::ostringstream caps;caps<<"0x"<<std::hex<<std::setfill('0')<<std::setw(16)<<m_version.capabilities;
    const auto version=m_version.flight_sw_version;
    out<<"{\"flight_sw_version_raw\":"<<version<<",\"version_known\":"<<boolean(version!=0)
      <<",\"major\":"<<(version>>24)<<",\"minor\":"<<((version>>16)&255)<<",\"patch\":"<<((version>>8)&255)<<",\"release_type_raw\":"<<(version&255)
      <<",\"flight_custom_version_raw_hex\":"<<quote(hexBytes(m_version.flight_custom_version))
      <<",\"flight_custom_version_ascii\":"<<ascii(m_version.flight_custom_version)
      <<",\"custom_version_semantics\":\"ardupilot_ascii_prefix_or_opaque_bytes_not_full_git_revision\""
      <<",\"capabilities_raw_hex\":"<<quote(caps.str())<<",\"capabilities_raw_decimal\":"<<quote(std::to_string(m_version.capabilities))
      <<",\"capability_assessment\":\"reported_bits_not_behavior_qualification\""
      <<",\"middleware_sw_version_raw\":"<<m_version.middleware_sw_version<<",\"os_sw_version_raw\":"<<m_version.os_sw_version
      <<",\"board_version_raw\":"<<m_version.board_version<<",\"vendor_id_raw\":"<<m_version.vendor_id<<",\"product_id_raw\":"<<m_version.product_id
      <<",\"uid_raw_decimal\":"<<quote(std::to_string(m_version.uid))<<",\"uid2_raw_hex\":"<<quote(hexBytes(m_version.uid2))
      <<",\"uid2_extension_bytes_present\":"<<boolean(m_magic==MAVLINK_STX&&m_payload_length>MAVLINK_MSG_ID_AUTOPILOT_VERSION_MIN_LEN)<<"}";
  }
  out<<",\"discovery\":{\"effects_allowed\":"<<boolean(m_effects)<<",\"state\":"<<quote(m_effects?m_request_state:"DISABLED")
    <<",\"request_kind\":\"MAV_CMD_REQUEST_MESSAGE\",\"command\":512,\"message_id\":148,\"epoch\":"<<m_epoch
    <<",\"request_id\":"<<m_request_id<<",\"attempts\":"<<m_attempts<<",\"max_attempts\":"<<max_attempts
    <<",\"expires\":"<<(m_started?number(m_expires):"null")<<",\"local_accepted\":"<<m_local_accepted<<",\"local_rejected\":"<<m_local_rejected
    <<",\"ack_correlation\":\"not_inferred\",\"response_causality\":\"not_proven\"}"
    <<",\"rejected\":"<<m_rejected<<",\"foreign\":"<<m_foreign<<",\"prebound\":"<<m_prebound<<"}";
  return out.str();
}
} // namespace mavmoos::runtime
