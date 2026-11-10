/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: UdpReceiver.h                                   */
/*    DATE: October 29, 2026                                */
/************************************************************/
#pragma once

#include "Config.h"
#include <array>
#include <cstdint>
#include <functional>

namespace mavmoos {
class Telemetry;

struct ReceiveStats {
  uint64_t datagrams = 0;
  uint64_t bytes = 0;
  uint64_t wrong_peer = 0;
  uint64_t truncated = 0;
  uint64_t errors = 0;
  uint64_t budget_hits = 0;
  uint64_t polls = 0;
  uint64_t empty_events = 0;
  uint64_t queue_stale_dropped = 0;
  uint64_t queue_clock_dropped = 0;
  double queue_epoch_at = -1;
  double last_read_at = -1;
  double last_effective_at = -1;
  double last_queue_age_upper_s = -1;
  double max_queue_age_upper_s = -1;
  double last_poll_gap_s = -1;
  double max_poll_gap_s = -1;
  bool draining = false;
  int last_errno = 0;
};

// A socket proven empty before a recvmsg call bounds every later queued
// packet's socket-queue admission from below. This portable bound is
// conservative, not an OS packet timestamp or source-generation timestamp.
class QueueEpoch {
public:
  enum class Decision { Accept, Stale, ClockInvalid };
  void reset(double before_bind);
  Decision received(double before_read, double after_read, double max_age);
  bool empty(double before_read, double after_read);
  double floor() const { return m_floor; }
  bool draining() const { return m_draining; }
private:
  bool ordered(double before_read, double after_read) const;
  double m_floor = -1;
  double m_last_read = -1;
  bool m_draining = false;
};

// RX is the default. Effectful I/O is restricted to an explicitly owned SITL endpoint.
class UdpReceiver {
public:
  UdpReceiver() = default;
  ~UdpReceiver();
  UdpReceiver(const UdpReceiver&) = delete;
  UdpReceiver& operator=(const UdpReceiver&) = delete;
  bool open(const RxConfig& config);
  void close();
  void poll(Telemetry& telemetry);
  bool sendSimulationFrame(const uint8_t* bytes, std::size_t size, int sitl_pid,
    double expires,const std::function<bool(double)>& eligible);
  static bool isSitlProcess(int pid,const std::string& profile="arducopter");
  static bool isSitlEndpoint(int pid, uint16_t peer_port,const std::string& profile="arducopter");
  bool isOpen() const { return m_socket >= 0; }
  const ReceiveStats& stats() const { return m_stats; }
  static constexpr std::size_t max_datagram_bytes = 4096;

private:
  int m_socket = -1;
  RxConfig m_config;
  uint32_t m_peer_ipv4 = 0;
  ReceiveStats m_stats;
  QueueEpoch m_epoch;
  double m_last_poll_at = -1;
  std::array<uint8_t, max_datagram_bytes> m_buffer{};
};
} // namespace mavmoos
