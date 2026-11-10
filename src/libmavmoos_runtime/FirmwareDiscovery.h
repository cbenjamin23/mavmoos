/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: FirmwareDiscovery.h                             */
/*    DATE: November 4, 2026                                */
/************************************************************/
#pragma once
#include "Plugin.h"
#include <cstdint>
#include <optional>
#include <string>

namespace mavmoos::runtime {
// Shared owner-thread observation and request state only: no transport, control
// authority, readiness verdict or claim that reported capabilities are tested.
class FirmwareDiscovery {
public:
  static constexpr unsigned max_attempts=3;
  static constexpr double request_lifetime=10, response_spacing=2,
    local_rejection_backoff=.2, heartbeat_timeout=2, version_freshness=30;
  struct Attempt {
    uint64_t epoch=0,request_id=0;
    unsigned attempt=0;
    double expires=0;
    uint16_t command=MAV_CMD_REQUEST_MESSAGE;
    uint32_t message_id=MAVLINK_MSG_ID_AUTOPILOT_VERSION;
  };
  struct Completion {
    bool matched_current=false,local_accepted=false,late=false,clock_valid=false;
    std::string state="UNMATCHED_RECEIPT";
  };
  explicit FirmwareDiscovery(unsigned expected_type=2,unsigned expected_autopilot=3,
                             uint8_t target_system=1,uint8_t target_component=1);
  void reset(double now,const std::string& reason);
  bool inputBoundary(double now);
  void onMessage(const mavlink_message_t&,const MessageTime&,double now);
  bool ready(double now,bool effects_allowed);
  std::optional<Attempt> take(double now,bool effects_allowed);
  bool eligible(const Attempt&,double now,bool effects_allowed);
  Completion complete(const Attempt&,double now,bool local_accepted);
  std::string status(double now);
  uint64_t epoch() const { return m_epoch; }
  uint64_t identityRevision() const { return m_identity_revision; }
  bool clockFault() const { return m_clock_fault; }
private:
  bool clock(double now);
  bool metadata(const MessageTime&,double maximum_age);
  void invalidate(const std::string&,bool clock_fault=false);
  bool advance(double now,bool effects_allowed);
  bool matches(const Attempt&) const;
  unsigned m_expected_type,m_expected_autopilot;
  uint8_t m_system,m_component;
  bool m_clock_fault=false,m_bound=false,m_present=false,m_started=false,m_effects=false;
  uint64_t m_epoch=1,m_request_id=0,m_observation_id=0,m_identity_revision=0;
  uint64_t m_rejected=0,m_foreign=0,m_prebound=0,m_local_accepted=0,m_local_rejected=0;
  double m_now=-1,m_floor=-1,m_bound_at=-1,m_heartbeat_at=-1,m_effective_at=0,m_read_at=0;
  double m_expires=0,m_next_attempt=0;
  unsigned m_attempts=0;
  std::string m_age_basis="READ_ACQUISITION",m_reason="not_bound",m_request_state="NOT_STARTED";
  mavlink_autopilot_version_t m_version{};
  uint8_t m_payload_length=0,m_magic=0;
  std::optional<Attempt> m_inflight;
};
} // namespace mavmoos::runtime
