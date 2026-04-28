#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

#include "MD.hpp"

namespace mab_rehab
{

/// Async worker that performs MD::setMotionMode() on a non-RT thread.
/// The RT perform_command_mode_switch() call enqueues a request and
/// returns immediately (~µs). The worker drains the queue on its own
/// thread, so the 10ms-per-drive cost of setMotionMode() never lands
/// on the RT loop.
///
/// During the latency window between enqueue and apply, the RT write()
/// should skip commanding that drive (the active_mode atomic is
/// the single source of truth — the RT writes check it before sending).
class ModeSwitchWorker
{
public:
  using GetDriveFn = std::function<mab::MD *(size_t joint_index)>;

  explicit ModeSwitchWorker(GetDriveFn get_drive);
  ~ModeSwitchWorker();

  ModeSwitchWorker(const ModeSwitchWorker &) = delete;
  ModeSwitchWorker & operator=(const ModeSwitchWorker &) = delete;

  /// Start the worker thread. Idempotent.
  void start();

  /// Stop and join. Idempotent.
  void stop();

  /// RT-safe. Enqueues a mode change; returns immediately.
  void request(size_t joint_index, mab::MdMode_E target_mode);

  /// RT-safe. Non-zero while a mode switch for this joint is pending.
  uint32_t pending_count(size_t joint_index) const;

private:
  struct Request
  {
    size_t joint_index;
    mab::MdMode_E target_mode;
  };

  void run();

  GetDriveFn get_drive_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::queue<Request> queue_;
  // Pending counter per joint; sized lazily on first request.
  mutable std::mutex pending_mutex_;
  std::vector<std::atomic<uint32_t>> pending_;
};

}  // namespace mab_rehab
