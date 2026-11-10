/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ManagedEgress.h                                 */
/*    DATE: October 29, 2026                                */
/************************************************************/
#pragma once
#include "CopterModel.h"
#include "DatagramIo.h"
#include <memory>
#include <thread>

namespace mavmoos::egress {
class ManagedEgress final {
public:
  static constexpr std::size_t result_capacity = 16, probe_capacity = 4;
  ManagedEgress(Binding binding, Envelope envelope, Clock& clock, std::unique_ptr<DatagramIo> io);
  ~ManagedEgress() = default;
  ManagedEgress(const ManagedEgress&) = delete;
  ManagedEgress& operator=(const ManagedEgress&) = delete;

  bool updateVehicle(const VehicleState& state);
  Reason enable(const Enable& request);
  Reason grantResponse(const ResponseGrant& grant);
  Admission submitNavigation(uint64_t id, Scope scope, uint64_t token, const NavInput& input);
  Admission submitResponse(uint64_t id, Scope scope, uint64_t token);
  Admission submitProbe(uint64_t id, Scope scope, uint64_t token, uint32_t sequence, Tick expires);
  bool cancel(uint64_t id); // waiting-only; false means not waiting, never remote recall
  uint64_t requestFence(Scope scope, FenceClasses classes = FenceClasses::Routine);
  void pump(); // apply priority fence, bounded purge, at most ONE write attempt; no timer/retry
  Fence close();
  ReceiveBatch receiveBatch(); // at most eight reads; keeps RX alive through a routine fence
  std::optional<Result> popResult();
  Stats stats() const;
  Fence fence() const;
  std::size_t outstanding() const;
  uint16_t localPort() const;

private:
  struct Work { Result result; uint64_t token = 0; Intent intent; };
  struct Call {
    explicit Call(ManagedEgress& owner);
    ~Call();
    ManagedEgress& engine;
  };
  void checkAccess() const;
  Tick now();
  Admission admit(uint64_t id, Scope scope, uint64_t token, Build build, uint64_t input_revision,
                  Tick at, const NavInput* input = nullptr);
  Reason requestCheck(uint64_t id, Scope scope, uint64_t token, Kind kind, Tick now);
  Evaluation gate(const Work& work, Tick now) const;
  void finish(Work work, Outcome outcome, Reason reason, Tick now, uint64_t related = 0);
  uint64_t requestFenceInternal(Tick now, FenceClasses classes = FenceClasses::Routine);
  void applyFence(Tick now);
  void expireWaiting(Tick now);
  std::size_t pendingCount() const;

  const Binding m_binding;
  const Envelope m_envelope;
  Clock& m_clock;
  std::unique_ptr<DatagramIo> m_io;
  const std::thread::id m_owner;
  bool m_busy = false, m_closed = false, m_clock_fault = false;
  bool m_revoked = true, m_fence_pending = false, m_active = false;
  bool m_all_revoked = false;
  bool m_response_granted = false, m_response_attempted = false, m_scope_residual = false;
  Tick m_started = 0, m_last_now = 0, m_raw_now = 0, m_boundary = 0;
  uint64_t m_last_id = 0, m_last_token = 0, m_fence_counter = 0;
  uint64_t m_scope_accepted = 0, m_last_accepted = 0;
  Scope m_scope;
  Enable m_enable;
  ResponseGrant m_response;
  VehicleState m_vehicle;
  Fence m_fence;
  Stats m_stats;
  unsigned m_nav_streak = 0;
  std::optional<Work> m_navigation, m_hold;
  std::array<Work, probe_capacity> m_probes{};
  std::size_t m_probe_count = 0;
  std::array<Result, result_capacity> m_results{};
  std::size_t m_result_head = 0, m_result_size = 0;
  mavlink_status_t m_codec{};
};
} // namespace mavmoos::egress
