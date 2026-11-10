#include "Runtime.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace mavmoos::runtime {
namespace {
constexpr double cleanup_grace=2;

bool logIdentifier(const std::string& value)
{
  return !value.empty() && value.size()<=96 &&
    std::all_of(value.begin(),value.end(),[](unsigned char c) {
      return (c>='a'&&c<='z') || (c>='A'&&c<='Z') ||
        (c>='0'&&c<='9') || c=='_' || c=='-' || c=='.' || c=='/';
    });
}
}

bool Runtime::flightLogGroundSafe() const
{
  return m_expected_type==MAV_TYPE_QUADROTOR && m_expected_autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA &&
    m_vehicle.heartbeat_valid && fresh(m_vehicle.heartbeat_at,2) && !m_vehicle.armed &&
    m_vehicle.landed_valid && fresh(m_vehicle.landed_at,2) &&
    m_vehicle.landed_state==MAV_LANDED_STATE_ON_GROUND;
}

bool Runtime::flightLogEligible(const FlightLogRequest& r) const
{
  if(!m_configuration_valid || !m_simulation || m_shadow || m_files_bench || !flightLogGroundSafe() ||
     m_nav_permission || m_shadow_grant || m_resume_until>m_now || m_hold || controlBusy() ||
     !m_mission_mutations.empty() ||
     !logIdentifier(r.id) || !logIdentifier(r.operation_id) || !std::isfinite(r.expires) ||
     !std::isfinite(r.operation_expires) || r.expires<=m_now || r.expires-m_now>2) return false;
  if(m_flight_log && (m_flight_log->id!=r.operation_id || m_flight_log->expires!=r.operation_expires))
    return false;
  if(r.action==FlightLogAction::End)
    return m_flight_log && m_flight_log->handed_off && r.expires<=r.operation_expires+cleanup_grace;
  if((m_flight_log && m_flight_log->end_only) || r.expires>r.operation_expires ||
     r.operation_expires-m_now>120) return false;
  switch(r.action) {
  case FlightLogAction::List:
    return r.first<=r.last && (r.last-r.first<128 || (r.first==0 && r.last==0xffff));
  case FlightLogAction::Data:
    return r.log_id>0 && r.count>0 && r.count<=3600 && uint64_t(r.offset)+r.count<=64*1024*1024;
  default:
    return false;
  }
}

bool Runtime::flightLog(const FlightLogRequest& r)
{
  if(configurationEffectAttempt()) return false;
  expireCommandLeases();expireMissionOperations();refreshFlightLog();
  if(!flightLogEligible(r)) return false;
  if(r.action==FlightLogAction::End) {
    // END supersedes any data intent in this operation.
    flightLogFinished(r.operation_id);
  } else if(m_flight_log_pending || m_flight_log_handoff) return false;
  if(!m_flight_log) m_flight_log=FlightLogOperation{r.operation_id,r.operation_expires};
  m_flight_log_pending=r;
  return true;
}

void Runtime::flightLogFinished(const std::string& id)
{
  if(configurationEffectAttempt() || !m_flight_log || m_flight_log->id!=id) return;
  if(m_flight_log_pending && m_flight_log_pending->action!=FlightLogAction::End) m_flight_log_pending.reset();
  if(m_flight_log_handoff && m_flight_log_handoff->action!=FlightLogAction::End) m_flight_log_handoff.reset();
  if(m_flight_log->handed_off) m_flight_log->end_only=true;
  else resetFlightLog();
}

void Runtime::resetFlightLog()
{
  m_flight_log.reset();
  m_flight_log_pending.reset();
  m_flight_log_handoff.reset();
}

void Runtime::refreshFlightLog()
{
  if(!m_flight_log) return;
  if(!flightLogGroundSafe() || m_now>=m_flight_log->expires+cleanup_grace) resetFlightLog();
  else if(!m_flight_log->end_only && m_now>=m_flight_log->expires) {
    const auto id=m_flight_log->id;
    flightLogFinished(id);
  }
}

void Runtime::flushFlightLog()
{
  const auto r=*m_flight_log_pending;
  m_flight_log_pending.reset();
  m_flight_log_handoff=r;
  mavlink_message_t msg{};
  switch(r.action) {
  case FlightLogAction::List:
    mavlink_msg_log_request_list_pack_status(245,191,&m_codec,&msg,1,1,r.first,r.last);
    break;
  case FlightLogAction::Data:
    mavlink_msg_log_request_data_pack_status(245,191,&m_codec,&msg,1,1,r.log_id,r.offset,r.count);
    break;
  case FlightLogAction::End:
    mavlink_msg_log_request_end_pack_status(245,191,&m_codec,&msg,1,1);
    break;
  }
  if(!sampleClock()) return;
  const auto live=[this,&r] {
    return m_flight_log && m_flight_log_handoff && m_flight_log_handoff->id==r.id &&
      flightLogEligible(r);
  };
  const bool accepted=live() && send(msg,r.expires,live);
  m_flight_log_handoff.reset();
  if(accepted && m_flight_log && m_flight_log->id==r.operation_id && m_flight_log->expires==r.operation_expires) {
    if(r.action==FlightLogAction::End) resetFlightLog();
    else m_flight_log->handed_off=true;
  }
  const bool clock_ok=sampleClock();
  std::ostringstream at;at<<std::setprecision(17)<<m_now;
  publish("MAVMOOS_FLIGHT_LOG_TX_RESULT","id="+r.id+",operation_id="+r.operation_id+
    ",msgid="+std::to_string(msg.msgid)+",at="+at.str()+
    ",status="+(accepted?"LOCAL_ACCEPTED":"NOT_ACCEPTED")+
    ",late="+(m_now>=r.expires?"true":"false")+",clock_valid="+(clock_ok?"true":"false"));
  if(clock_ok) m_host.flightLogSent(r,m_now,accepted);
}
} // namespace mavmoos::runtime
