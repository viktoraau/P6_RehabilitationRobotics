#include "mab_rehab/rt/maintenance_gate.hpp"

#include <chrono>
#include <thread>

namespace mab_rehab
{

bool MaintenanceGate::request_handoff()
{
  State expected = State::Running;
  return state_.compare_exchange_strong(
    expected, State::HandoffRequested,
    std::memory_order_acq_rel, std::memory_order_acquire);
}

bool MaintenanceGate::wait_for_acquired(const int timeout_ms)
{
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (state_.load(std::memory_order_acquire) != State::Acquired) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

void MaintenanceGate::release()
{
  state_.store(State::Released, std::memory_order_release);
}

bool MaintenanceGate::rt_should_skip_cycle()
{
  State expected = State::HandoffRequested;
  // If a handoff is pending, accept it.
  if (state_.compare_exchange_strong(
      expected, State::Acquired,
      std::memory_order_acq_rel, std::memory_order_acquire))
  {
    return true;
  }
  // If we have already handed off, keep skipping until Released.
  const auto current = state_.load(std::memory_order_acquire);
  if (current == State::Acquired) {
    return true;
  }
  if (current == State::Released) {
    // Resume on next cycle; this cycle is still considered "maintenance".
    state_.store(State::Running, std::memory_order_release);
    return true;
  }
  return false;
}

}  // namespace mab_rehab
