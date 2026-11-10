#include "Runtime.h"
#include <algorithm>
#include <stdexcept>

namespace mavmoos::runtime {
namespace {
constexpr double ground_age_limit_s=2;
constexpr double send_interval_s=.05;
}

void Runtime::enableExternalNavigationBench()
{
  auto names=moduleNames();
  std::sort(names.begin(),names.end());
  if(!m_startup_blocked || !m_module_configuration_complete || m_external_navigation_bench ||
     !m_configuration_valid || !m_simulation || m_shadow || m_backend || m_firmware_requests_enabled ||
     m_expected_type!=MAV_TYPE_QUADROTOR || m_expected_autopilot!=MAV_AUTOPILOT_ARDUPILOTMEGA ||
     !m_host.hasCapability(external_navigation_capability) ||
     names!=std::vector<std::string>{"external_navigation","telemetry"})
    throw std::invalid_argument("ExternalNav requires the dedicated disarmed Copter SITL bench");
  m_external_navigation_bench=true;
}

bool Runtime::externalNavigationReady() const
{
  return m_external_navigation_bench && !m_startup_blocked && !m_configuration_callbacks &&
    m_configuration_valid && m_simulation && !m_shadow && !m_backend &&
    m_vehicle.type==MAV_TYPE_QUADROTOR && m_vehicle.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA &&
    m_vehicle.heartbeat_valid && fresh(m_vehicle.heartbeat_at,ground_age_limit_s) && !m_vehicle.armed &&
    m_vehicle.landed_valid && fresh(m_vehicle.landed_at,ground_age_limit_s) &&
    m_vehicle.landed_state==MAV_LANDED_STATE_ON_GROUND;
}

bool Runtime::beginExternalNavigation(uint64_t run_id,double expires)
{
  if(configurationEffectAttempt())return false;
  refreshExternalNavigation();
  if(!externalNavigationReady() || m_external_lease || !run_id || run_id<=m_external_high_run ||
     !std::isfinite(expires) || expires<=m_now || expires-m_now>external_navigation_lease_s) return false;
  m_external_lease=ExternalNavigationLease{run_id,expires,m_now,m_now};
  m_external_high_run=run_id;
  return true;
}

bool Runtime::externalNavigationEligible(const ExternalNavigationSample& sample) const
{
  return externalNavigationReady() && m_external_lease && m_external_lease->run_id==sample.run_id &&
    m_now<m_external_lease->expires && m_now-m_external_lease->last_measurement<external_navigation_silence_s &&
    sample.capture_at>=m_external_lease->started && externalNavigationCaptureTime(sample.capture_us)>m_external_lease->started &&
    sample.expires<=m_external_lease->expires && validExternalNavigationSample(sample,m_now);
}

bool Runtime::externalNavigation(const ExternalNavigationSample& sample)
{
  if(configurationEffectAttempt())return false;
  refreshExternalNavigation();
  if(!externalNavigationEligible(sample) || sample.sequence<=m_external_lease->sequence ||
     sample.capture_us<=m_external_lease->capture_us) return false;
  m_external_lease->sequence=sample.sequence;
  m_external_lease->capture_us=sample.capture_us;
  m_external_lease->last_measurement=std::min(sample.capture_at,externalNavigationCaptureTime(sample.capture_us));
  if(m_external_pending) ++m_external_replaced;
  m_external_pending=sample;
  return true;
}

void Runtime::stopExternalNavigation(uint64_t run_id)
{
  if(configurationEffectAttempt())return;
  if(m_external_lease && m_external_lease->run_id==run_id) resetExternalNavigation();
}

void Runtime::resetExternalNavigation()
{
  m_external_pending.reset();
  m_external_handoff.reset();
  m_external_lease.reset();
}

void Runtime::refreshExternalNavigation()
{
  if(m_external_lease && (!externalNavigationReady() || m_now>=m_external_lease->expires ||
     m_now-m_external_lease->last_measurement>=external_navigation_silence_s)) resetExternalNavigation();
}

bool Runtime::externalNavigationWaiting() const
{
  return m_external_pending && !m_external_handoff && m_now-m_external_last_send>=send_interval_s;
}

void Runtime::flushExternalNavigation()
{
  if(!externalNavigationWaiting())return;
  const auto sample=*m_external_pending;
  m_external_pending.reset();
  m_external_handoff=sample;
  const auto connection=m_connection_revision;
  const auto eligible=[this,&sample] {
    return m_external_handoff && m_external_handoff->run_id==sample.run_id &&
      m_external_handoff->sequence==sample.sequence &&
      m_external_lease && m_external_lease->sequence==sample.sequence &&
      externalNavigationEligible(sample);
  };
  if(!sampleClock())return;
  bool accepted=false;
  if(eligible()) {
    const double expires=std::min({sample.expires,m_external_lease->expires,
      m_external_lease->last_measurement+external_navigation_silence_s,
      m_vehicle.heartbeat_at+ground_age_limit_s,m_vehicle.landed_at+ground_age_limit_s});
    mavlink_message_t message{};
    const auto packet=makeExternalNavigationOdometry(sample);
    mavlink_msg_odometry_encode_status(245,191,&m_codec,&message,&packet);
    accepted=send(message,expires,eligible);
    // Endpoint verification can take time. Pace from the completed handoff.
    if(!sampleClock())return;
    m_external_last_send=m_now;
  }
  m_external_handoff.reset();
  if(connection!=m_connection_revision)return;
  refreshExternalNavigation();
  if(accepted) ++m_external_accepted;
  m_host.externalNavigationSent(sample,m_now,accepted);
}

} // namespace mavmoos::runtime
