/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: Telemetry.h                                     */
/*    DATE: October 29, 2026                                */
/************************************************************/
#pragma once

#include "Config.h"
#include <ardupilotmega/mavlink.h>
#include <array>
#include <deque>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace mavmoos {
enum class Family : std::size_t { Heartbeat, Position, Landed, Home, Estimator, Count };

struct Observation {
  bool present = false;
  bool encoding_valid = false;
  std::string reason = "absent";
  std::string fields;
  double received_at = 0;
  double read_at = 0;
  std::string age_basis = "READ_ACQUISITION";
  uint64_t receipt_id = 0;
  uint64_t datagram_id = 0;
  uint8_t wire_sequence = 0;
};

struct DecodeStats {
  uint64_t frames = 0;
  uint64_t bad_frames = 0;
  uint64_t incomplete_datagrams = 0;
  uint64_t empty_or_noise = 0;
  uint64_t wrong_target = 0;
  uint64_t unsupported_flags = 0;
  uint64_t unsupported_messages = 0;
  uint64_t unbound_reports = 0;
  uint64_t duplicates = 0;
  uint64_t observations = 0;
  uint64_t invalid_observations = 0;
  uint64_t invalid_timestamps = 0;
};

class Telemetry {
public:
  Telemetry(const RxConfig& config, std::string session);
  void ingest(const uint8_t* bytes, std::size_t size, double received_at,
              uint64_t datagram_id);
  void ingest(const uint8_t* bytes, std::size_t size, double effective_at,
              uint64_t datagram_id, double read_at, const std::string& age_basis);
  const Observation& observation(Family family) const;
  std::string record(Family family, double now) const;
  std::string identityState() const;
  bool heartbeatFresh(double now) const;
  void setMessageHandler(std::function<void(const mavlink_message_t&,double)> handler) {
    m_handler=[handler=std::move(handler)](const auto& msg,double effective,double) { handler(msg,effective); };
  }
  void setTimedMessageHandler(std::function<void(const mavlink_message_t&,double,double)> handler) { m_handler=std::move(handler); }
  // Only the app's verified opt-in operation bench enables component replies.
  // Passive observers keep their original smaller route.
  void enablePayloadResponses(bool camera,bool gimbal) {
    m_camera_responses=camera && m_config.camera_component;
    m_gimbal_responses=gimbal && m_config.gimbal_component;
  }
  const DecodeStats& stats() const { return m_stats; }
  static const char* variable(Family family);
  static std::vector<Family> families();

private:
  struct Seen { std::string bytes; double received_at; };
  void accept(const mavlink_message_t& message, double received_at, uint64_t datagram_id,
              double read_at, const std::string& age_basis);
  static bool duplicate(std::deque<Seen>& seen, const mavlink_message_t& message, double received_at);
  RxConfig m_config;
  std::string m_session;
  std::array<Observation, static_cast<std::size_t>(Family::Count)> m_reports{};
  std::deque<Seen> m_seen;
  std::deque<Seen> m_camera_seen, m_gimbal_seen, m_remote_id_seen, m_component_metadata_seen;
  DecodeStats m_stats;
  bool m_identity_seen = false;
  bool m_identity_matches = false;
  bool m_camera_responses=false,m_gimbal_responses=false;
  std::function<void(const mavlink_message_t&,double,double)> m_handler;
};
} // namespace mavmoos
