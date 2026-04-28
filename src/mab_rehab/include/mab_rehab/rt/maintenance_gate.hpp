#pragma once

#include <atomic>

namespace mab_rehab
{

/// Lock-free handshake between the RT read/write loop and non-RT
/// maintenance code. The RT thread observes the state on every cycle
/// and early-returns when maintenance is active; the maintenance thread
/// requests a handoff, waits for the RT side to acknowledge (by setting
/// Acquired), does its work, then releases.
///
/// States form a ring:
///   Running         — RT owns the transport, maintenance not pending
///   HandoffRequested— maintenance asked to take over; RT will Ack next cycle
///   Acquired        — RT has observed the request and will skip read/write
///   Released        — maintenance is done; RT resumes on next cycle
///
/// All transitions are compare-exchange, so no mutex is ever taken on the
/// RT thread. This is the only synchronization primitive between the RT
/// and non-RT halves of the plugin.
class MaintenanceGate
{
public:
  enum class State : uint8_t
  {
    Running,
    HandoffRequested,
    Acquired,
    Released,
  };

  MaintenanceGate() = default;

  // --- Maintenance side (non-RT) ---

  /// Ask the RT thread to hand off the bus. Non-blocking.
  /// Returns true if the request was accepted (state was Running).
  bool request_handoff();

  /// Wait (with timeout) until the RT thread acknowledges the handoff.
  /// Returns true if Acquired state was reached. Runs on a non-RT thread.
  bool wait_for_acquired(int timeout_ms);

  /// Mark the handoff as released; the RT thread will resume on its
  /// next cycle. Always succeeds.
  void release();

  // --- RT side (hot path) ---

  /// Called at the top of every RT read()/write(). If the state is
  /// HandoffRequested, transitions to Acquired and returns true (meaning
  /// "skip this cycle"). If Acquired or Released is observed, also returns
  /// true until state returns to Running. No allocation, no blocking.
  bool rt_should_skip_cycle();

  State load() const { return state_.load(std::memory_order_acquire); }

private:
  std::atomic<State> state_{State::Running};
};

}  // namespace mab_rehab
