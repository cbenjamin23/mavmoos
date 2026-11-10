/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ObservationRequests.cpp                         */
/*    DATE: November 4, 2026                                */
/************************************************************/
#include "ObservationRequests.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace mavmoos::runtime {
namespace {
std::string boolean(bool value) { return value?"true":"false"; }
std::string jsonString(const std::string& value)
{
  constexpr char hex[]="0123456789abcdef";std::string result="\"";
  for(unsigned char c:value) {
    if(c=='"'||c=='\\') { result+='\\';result+=char(c); }
    else if(c<32||c>=127) { result+="\\u00";result+=hex[c>>4];result+=hex[c&15]; }
    else result+=char(c);
  }
  return result+'"';
}
bool same(const ObservationRequests::Attempt& a,const ObservationRequests::Attempt& b)
{
  return a.epoch==b.epoch&&a.message_id==b.message_id&&a.interval_us==b.interval_us&&
    a.attempt==b.attempt&&a.expires==b.expires;
}
}

void ObservationRequests::nextEpoch()
{
  if(m_epoch==std::numeric_limits<uint64_t>::max()) {
    m_epoch_exhausted=true;m_clock_fault=true;m_reason="generation_exhausted";
  } else ++m_epoch;
}

bool ObservationRequests::observeClock(double now)
{
  if(m_clock_fault||m_epoch_exhausted) return false;
  if(!std::isfinite(now)||now<0||now<m_now) {
    nextEpoch();m_clock_fault=true;m_reason="clock_fault";m_inflight.reset();
    for(std::size_t i=0;i<m_size;++i)
      if(m_demands[i].state==State::Queued||m_demands[i].state==State::InFlight)m_demands[i].state=State::Cancelled;
    return false;
  }
  m_now=now;return true;
}

void ObservationRequests::reset(double now,const std::string& reason)
{
  m_last_reset_retired=0;
  for(std::size_t i=0;i<m_size;++i)
    m_last_reset_retired+=m_demands[i].state==State::Queued||m_demands[i].state==State::InFlight||m_demands[i].state==State::Cancelled;
  nextEpoch();m_demands={};m_size=m_cursor=m_input_count=0;m_inflight.reset();
  m_started=m_rejected=false;m_admitted_at=m_expires=m_next_global=0;
  m_reason=reason.substr(0,128);
  // A reset is not permission to reinterpret an earlier point on this clock.
  const bool valid=std::isfinite(now)&&now>=0&&now>=m_now&&!m_epoch_exhausted;
  m_clock_fault=!valid;
  if(valid)m_now=now;else m_reason=m_epoch_exhausted?"generation_exhausted":"clock_fault";
}

bool ObservationRequests::start(const std::vector<ObservationStream>& streams,double now)
{
  if(!advance(now)||m_started) return false;
  nextEpoch();if(m_epoch_exhausted)return false;
  m_started=true;m_input_count=streams.size();m_admitted_at=now;m_expires=now+batch_lifetime;
  m_next_global=now;m_size=m_cursor=0;m_inflight.reset();
  const auto reject=[this](const char* reason) {m_rejected=true;m_reason=reason;m_size=0;return false;};
  if(!std::isfinite(m_expires)||m_expires<=now)return reject("unrepresentable_deadline");
  if(now+global_spacing<=now||now+rejection_backoff<=now)return reject("unrepresentable_pacing");
  if(streams.size()>max_input_demands)return reject("input_capacity");
  for(const auto& stream:streams) {
    if(stream.message_id>0xffffff||!std::isfinite(stream.interval_us)||
       stream.interval_us<10000||stream.interval_us>60000000)return reject("invalid_demand");
    std::size_t index=0;
    while(index<m_size&&m_demands[index].message_id!=stream.message_id)++index;
    if(index<m_size) {m_demands[index].interval_us=std::min(m_demands[index].interval_us,stream.interval_us);continue;}
    if(m_size==max_demands)return reject("demand_capacity");
    auto& d=m_demands[m_size++];d={};d.message_id=stream.message_id;d.interval_us=stream.interval_us;
    d.not_before=now;d.expires=m_expires;
  }
  m_reason=m_size?"admitted":"empty_plan";return true;
}

bool ObservationRequests::advance(double now)
{
  if(!observeClock(now))return false;
  for(std::size_t i=0;i<m_size;++i) {
    auto& d=m_demands[i];
    if(now>=d.expires&&(d.state==State::Queued||d.state==State::InFlight)) {
      d.state=State::Expired;d.deadline_elapsed=true;
    }
  }
  return true;
}

std::optional<std::size_t> ObservationRequests::pendingIndex() const
{
  if(!m_started||m_rejected||m_clock_fault||m_inflight||m_now<m_next_global)return {};
  for(std::size_t offset=0;offset<m_size;++offset) {
    const auto index=(m_cursor+offset)%m_size;const auto& d=m_demands[index];
    if(d.state==State::Queued&&m_now>=d.not_before&&m_now<d.expires)return index;
  }
  return {};
}

bool ObservationRequests::ready(double now)
{return advance(now)&&pendingIndex().has_value();}

std::optional<ObservationRequests::Attempt> ObservationRequests::take(double now)
{
  if(!advance(now))return {};
  const auto index=pendingIndex();if(!index)return {};
  auto& d=m_demands[*index];d.state=State::InFlight;++d.attempts;m_cursor=(*index+1)%m_size;
  m_next_global=now+global_spacing;
  m_inflight=Attempt{m_epoch,d.message_id,d.interval_us,d.attempts,d.expires};
  return m_inflight;
}

bool ObservationRequests::matches(const Attempt& attempt) const
{return m_inflight&&attempt.epoch==m_epoch&&same(attempt,*m_inflight);}

bool ObservationRequests::eligible(const Attempt& attempt,double now)
{
  return advance(now)&&matches(attempt)&&now<attempt.expires;
}

ObservationRequests::Completion ObservationRequests::complete(const Attempt& attempt,double now,bool accepted)
{
  Completion result;result.local_accepted=accepted;result.clock_valid=advance(now);
  result.late=result.clock_valid&&now>=attempt.expires;
  if(!result.clock_valid||!matches(attempt))return result;
  auto it=std::find_if(m_demands.begin(),m_demands.begin()+m_size,[&](const Demand& d){return d.message_id==attempt.message_id;});
  if(it==m_demands.begin()+m_size)return result;
  result.matched_current=true;m_inflight.reset();m_next_global=std::max(m_next_global,now+global_spacing);
  // An actual local acceptance stays visible even if the post-return clock
  // sample crossed expiry. It is terminal, never permission for a retry.
  if(accepted) {it->state=State::LocalAccepted;it->accepted_receipt_late=result.late;}
  else if(result.late) {it->state=State::Expired;it->deadline_elapsed=true;}
  else if(it->attempts>=max_attempts)it->state=State::Exhausted;
  else {it->state=State::Queued;it->not_before=now+rejection_backoff;}
  result.terminal_state=stateName(*it);return result;
}

const ObservationRequests::Demand& ObservationRequests::demand(std::size_t index) const
{
  if(index>=m_size)throw std::out_of_range("observation demand index");
  return m_demands[index];
}

std::string ObservationRequests::stateName(const Demand& d)
{
  switch(d.state) {
    case State::Queued:return d.attempts?"RETRY_PENDING":"QUEUED";
    case State::InFlight:return "IN_FLIGHT";
    case State::LocalAccepted:return "LOCAL_ACCEPTED";
    case State::Exhausted:return "EXHAUSTED";
    case State::Expired:return "EXPIRED";
    case State::Cancelled:return "CANCELLED";
  }
  return "UNKNOWN";
}

std::string ObservationRequests::status() const
{
  std::size_t queued=0,accepted=0,exhausted=0,expired=0,cancelled=0,late=0;
  unsigned attempts=0;
  for(std::size_t i=0;i<m_size;++i) {
    const auto& d=m_demands[i];queued+=d.state==State::Queued;accepted+=d.state==State::LocalAccepted;
    exhausted+=d.state==State::Exhausted;expired+=d.state==State::Expired;cancelled+=d.state==State::Cancelled;
    late+=d.accepted_receipt_late;attempts+=d.attempts;
  }
  const std::string phase=m_clock_fault?"CLOCK_FAULT":!m_started?"IDLE":m_rejected?"REJECTED":
    m_inflight?"AWAITING_LOCAL_RESULT":queued?"ACTIVE":(expired||exhausted||cancelled)?"SETTLED_LOCAL_WITH_FAILURES":"SETTLED_LOCAL";
  std::ostringstream s;s<<std::setprecision(17)<<"{\"schema\":1,\"epoch\":"<<m_epoch<<",\"state\":"<<jsonString(phase)
    <<",\"reason\":"<<jsonString(m_reason)<<",\"started\":"<<boolean(m_started)<<",\"clock_valid\":"<<boolean(!m_clock_fault)
    <<",\"input_demands\":"<<m_input_count<<",\"demands\":"<<m_size<<",\"queued\":"<<queued
    <<",\"in_flight\":"<<boolean(m_inflight.has_value())<<",\"local_accepted\":"<<accepted
    <<",\"exhausted\":"<<exhausted<<",\"expired\":"<<expired<<",\"cancelled\":"<<cancelled
    <<",\"last_reset_retired\":"<<m_last_reset_retired<<",\"accepted_receipt_late\":"<<late<<",\"handoff_attempts\":"<<attempts
    <<",\"admitted_at\":";
  if(m_started)s<<m_admitted_at;else s<<"null";
  s<<",\"expires\":";
  if(m_started&&std::isfinite(m_expires)&&m_expires>m_admitted_at)s<<m_expires;else s<<"null";
  s<<",\"remote_application\":\"unconfirmed\",\"ack_correlation\":\"not_inferred\",\"items\":[";
  for(std::size_t i=0;i<m_size;++i) {
    const auto& d=m_demands[i];if(i)s<<',';
    s<<"{\"message_id\":"<<d.message_id<<",\"interval_us\":"<<d.interval_us<<",\"attempt\":"<<d.attempts
      <<",\"state\":"<<jsonString(stateName(d))<<",\"not_before\":"<<d.not_before<<",\"expires\":"<<d.expires
      <<",\"deadline_elapsed\":"<<boolean(d.deadline_elapsed)<<",\"accepted_receipt_late\":"<<boolean(d.accepted_receipt_late)<<'}';
  }
  return s.str()+"]}";
}
} // namespace mavmoos::runtime
