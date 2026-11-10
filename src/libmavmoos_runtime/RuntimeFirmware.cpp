/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: RuntimeFirmware.cpp                             */
/*    DATE: November 4, 2026                                */
/************************************************************/
#include "Runtime.h"
#include <iomanip>
#include <sstream>

namespace mavmoos::runtime {
bool Runtime::firmwareEffectsAllowed() const
{
  return m_configuration_valid&&m_firmware_requests_enabled&&m_simulation&&!m_shadow&&
    m_vehicle.heartbeat_valid&&fresh(m_vehicle.heartbeat_at,2);
}

void Runtime::setFirmwareRequestsEnabled(bool enabled)
{
  m_firmware_requests_enabled=enabled;
  m_firmware.ready(m_now,firmwareEffectsAllowed());
}

void Runtime::publishFirmwareStatus(bool force)
{
  if(!force&&m_last_firmware_publish>=0&&m_now>=m_last_firmware_publish&&
     m_now-m_last_firmware_publish<.2) return;
  m_firmware.ready(m_now,firmwareEffectsAllowed());
  publish("MAVMOOS_FIRMWARE_STATUS",m_firmware.status(m_now));
  m_last_firmware_publish=m_now;
}

bool Runtime::flushFirmwareRequest()
{
  if(!sampleClock()) return false;
  const auto attempt=m_firmware.take(m_now,firmwareEffectsAllowed());
  if(!attempt) { publishFirmwareStatus(true); return false; }
  const auto work=*attempt;
  mavlink_message_t message{};
  mavlink_msg_command_long_pack_status(245,191,&m_codec,&message,1,1,work.command,0,
    work.message_id,0,0,0,0,0,0);
  const auto live=[this,work] {
    return m_firmware.eligible(work,m_now,firmwareEffectsAllowed());
  };
  const bool accepted=send(message,work.expires,live);
  const bool clock_ok=sampleClock();
  const auto result=m_firmware.complete(work,m_now,accepted);
  std::ostringstream event;event<<std::setprecision(17)
    <<"{\"schema\":1,\"epoch\":"<<work.epoch<<",\"request_id\":"<<work.request_id
    <<",\"attempt\":"<<work.attempt<<",\"command\":"<<work.command<<",\"message_id\":"<<work.message_id
    <<",\"expires\":"<<work.expires<<",\"local_result\":\""<<(accepted?"LOCAL_ACCEPTED":"NOT_ACCEPTED")
    <<"\",\"matched_current\":"<<(result.matched_current?"true":"false")
    <<",\"clock_valid\":"<<(clock_ok&&result.clock_valid?"true":"false")
    <<",\"response_causality\":\"not_proven\",\"qualification\":\"not_evaluated\"}";
  publish("MAVMOOS_FIRMWARE_REQUEST_RESULT",event.str());
  publishFirmwareStatus(true);
  return clock_ok;
}
} // namespace mavmoos::runtime
