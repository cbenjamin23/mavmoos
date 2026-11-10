/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: RuntimeStream.cpp                               */
/*    DATE: November 4, 2026                                */
/************************************************************/
#include "Runtime.h"
#include <iomanip>
#include <sstream>

namespace mavmoos::runtime {
void Runtime::publishStreamStatus()
{
  publish("MAVMOOS_STREAM_STATUS",m_stream_requests.status());
}

bool Runtime::flushObservationRequest()
{
  if(!sampleClock()) return false;
  const auto attempt=m_stream_requests.take(m_now);
  if(!attempt) { publishStreamStatus(); return false; }
  // Copy the epoch/attempt/deadline before endpoint proof. That proof can reset
  // or reconfigure the host, and an old encoded frame must stay fenced out.
  const auto work=*attempt;
  mavlink_message_t message{};
  mavlink_msg_command_long_pack_status(245,191,&m_codec,&message,1,1,MAV_CMD_SET_MESSAGE_INTERVAL,0,
    work.message_id,work.interval_us,0,0,0,0,0);
  const auto live=[this,work] {
    const bool rover_recovery_observation=px4RoverProfile() &&
      (work.message_id==MAVLINK_MSG_ID_CURRENT_MODE || work.message_id==MAVLINK_MSG_ID_GPS_RAW_INT);
    const bool ardu_rover_recovery_observation=arduRoverMissionPolicy() &&
      (work.message_id==MAVLINK_MSG_ID_GLOBAL_POSITION_INT || work.message_id==MAVLINK_MSG_ID_EKF_STATUS_REPORT);
    const bool quadplane_recovery_observation=quadPlaneProfile() &&
      (work.message_id==MAVLINK_MSG_ID_GLOBAL_POSITION_INT || work.message_id==MAVLINK_MSG_ID_HOME_POSITION ||
       work.message_id==MAVLINK_MSG_ID_EXTENDED_SYS_STATE);
    return (!autoRecoveryBlocksEffects() || rover_recovery_observation || ardu_rover_recovery_observation ||
            quadplane_recovery_observation) &&
      m_vehicle.heartbeat_valid && fresh(m_vehicle.heartbeat_at,2) &&
      m_stream_requests.eligible(work,m_now);
  };
  const bool accepted=send(message,work.expires,live);
  const bool clock_ok=sampleClock();
  const auto result=m_stream_requests.complete(work,m_now,accepted);
  const bool clock_valid=clock_ok&&result.clock_valid;
  // Always report the actual local return, including acceptance followed by a
  // reset/late receipt. This is not FC acknowledgement or application evidence.
  std::ostringstream event;event<<std::setprecision(17)
    <<"{\"schema\":1,\"epoch\":"<<work.epoch<<",\"message_id\":"<<work.message_id
    <<",\"interval_us\":"<<work.interval_us<<",\"attempt\":"<<work.attempt
    <<",\"expires\":"<<work.expires<<",\"local_result\":\""<<(accepted?"LOCAL_ACCEPTED":"NOT_ACCEPTED")
    <<"\",\"matched_current\":"<<(result.matched_current?"true":"false")
    <<",\"state\":\""<<result.terminal_state<<"\",\"clock_valid\":"<<(clock_valid?"true":"false")
    <<",\"post_return_at\":";
  if(clock_valid) event<<m_now; else event<<"null";
  event<<",\"late\":";
  if(clock_valid) event<<(result.late?"true":"false"); else event<<"null";
  event<<",\"remote_application\":\"unconfirmed\",\"ack_correlation\":\"not_inferred\"}";
  publish("MAVMOOS_STREAM_REQUEST_RESULT",event.str());
  publishStreamStatus();
  return clock_ok;
}
} // namespace mavmoos::runtime
