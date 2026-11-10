/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ObservationRequests.h                           */
/*    DATE: November 4, 2026                                */
/************************************************************/
#pragma once

#include "Plugin.h"
#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace mavmoos::runtime {

// Single-owner local handoff scheduling only. No socket, FC ACK correlation,
// authority, source-freshness inference or remote-application promise.
class ObservationRequests {
public:
  static constexpr std::size_t max_demands=32, max_input_demands=1024;
  static constexpr unsigned max_attempts=3;
  static constexpr double batch_lifetime=10, global_spacing=.1, rejection_backoff=.2;

  struct Attempt {
    uint64_t epoch=0;
    uint32_t message_id=0;
    float interval_us=0;
    unsigned attempt=0;
    double expires=0;
  };
  struct Completion {
    bool matched_current=false;
    bool local_accepted=false;
    bool late=false; // Post-return receipt time, not the actual syscall time.
    bool clock_valid=false;
    // Resulting local state; RETRY_PENDING is deliberately nonterminal.
    std::string terminal_state="UNMATCHED_RECEIPT";
  };
  enum class State { Queued, InFlight, LocalAccepted, Exhausted, Expired, Cancelled };
  struct Demand {
    uint32_t message_id=0;
    float interval_us=0;
    unsigned attempts=0;
    State state=State::Queued;
    double not_before=0, expires=0;
    bool deadline_elapsed=false, accepted_receipt_late=false;
  };

  // Admission is attempted once per reset, even when rejected or empty. Input
  // duplicates merge to the shortest interval without changing first-seen order.
  bool start(const std::vector<ObservationStream>& streams,double now);
  void reset(double now,const std::string& reason);
  bool started() const { return m_started; }
  bool advance(double now);
  bool ready(double now);
  std::optional<Attempt> take(double now);
  bool eligible(const Attempt&,double now);
  Completion complete(const Attempt&,double now,bool local_accepted);

  uint64_t epoch() const { return m_epoch; }
  bool clockFault() const { return m_clock_fault; }
  bool inFlight() const { return m_inflight.has_value(); }
  std::size_t size() const { return m_size; }
  const Demand& demand(std::size_t index) const;
  std::string status() const;

private:
  bool observeClock(double now);
  void nextEpoch();
  bool matches(const Attempt&) const;
  std::optional<std::size_t> pendingIndex() const;
  static std::string stateName(const Demand&);
  std::array<Demand,max_demands> m_demands{};
  std::size_t m_size=0,m_cursor=0,m_input_count=0,m_last_reset_retired=0;
  std::optional<Attempt> m_inflight;
  uint64_t m_epoch=0;
  bool m_started=false,m_rejected=false,m_clock_fault=false,m_epoch_exhausted=false;
  double m_now=-1,m_admitted_at=0,m_expires=0,m_next_global=0;
  std::string m_reason="not_started";
};
} // namespace mavmoos::runtime
