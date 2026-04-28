#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace mab_rehab
{

/// Per-drive failure counters and worst-case read/write time tracking.
/// Counters are written from the RT thread (atomic increments only);
/// HealthMonitor reads them from a non-RT timer and decides when to log,
/// raise a warning, or request a fail-safe.
///
/// Escalation ladder (configurable):
///   read_fail_count  > warn_threshold   -> log WARN throttled
///   read_fail_count  > fault_threshold  -> request controller deactivation
///   write_fail_count > warn_threshold   -> log WARN throttled
///   write_fail_count > fault_threshold  -> request controller deactivation
///
/// The "request controller deactivation" action is delegated via a user-supplied
/// callback so HealthMonitor itself has no knowledge of ControllerSwitcher —
/// this keeps the dependency graph acyclic.
class HealthMonitor
{
public:
  struct DriveStats
  {
    std::atomic<uint32_t> read_fail_count{0};
    std::atomic<uint32_t> write_fail_count{0};
    std::atomic<uint32_t> consecutive_read_fails{0};
    std::atomic<uint32_t> consecutive_write_fails{0};
    std::atomic<uint32_t> worst_read_us{0};
    std::atomic<uint32_t> worst_write_us{0};
  };

  struct Config
  {
    uint32_t warn_threshold = 10;
    uint32_t fault_threshold = 50;
    int poll_rate_hz = 10;
  };

  using FaultHandler = std::function<void(const std::string & reason)>;

  HealthMonitor(
    rclcpp::Node::SharedPtr node,
    std::vector<std::string> joint_names,
    Config config,
    FaultHandler on_fault);

  /// Per-drive stats, one slot per joint, indexed in the same order as
  /// the joint_names passed to the ctor. Stable pointer — the RT thread
  /// holds this and calls the atomics directly, no mutex.
  DriveStats & stats(size_t joint_index) { return stats_.at(joint_index); }

  /// Start the non-RT timer.
  void start();

  /// Stop the timer. Safe to call from any non-RT context.
  void stop();

private:
  void poll();

  rclcpp::Node::SharedPtr node_;
  std::vector<std::string> joint_names_;
  Config config_;
  FaultHandler on_fault_;
  std::vector<DriveStats> stats_;
  std::atomic<bool> fault_raised_{false};
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mab_rehab
