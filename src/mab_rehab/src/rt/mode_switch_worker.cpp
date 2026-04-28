#include "mab_rehab/rt/mode_switch_worker.hpp"

namespace mab_rehab
{

ModeSwitchWorker::ModeSwitchWorker(GetDriveFn get_drive)
: get_drive_(std::move(get_drive))
{
}

ModeSwitchWorker::~ModeSwitchWorker()
{
  stop();
}

void ModeSwitchWorker::start()
{
  if (thread_.joinable()) {
    return;
  }
  stop_.store(false, std::memory_order_release);
  thread_ = std::thread([this]() { run(); });
}

void ModeSwitchWorker::stop()
{
  if (!thread_.joinable()) {
    return;
  }
  stop_.store(true, std::memory_order_release);
  queue_cv_.notify_all();
  thread_.join();
}

void ModeSwitchWorker::request(const size_t joint_index, const mab::MdMode_E target_mode)
{
  {
    std::lock_guard<std::mutex> pending_lock(pending_mutex_);
    if (joint_index >= pending_.size()) {
      // Grow the pending vector; this is a non-RT operation,
      // but the request() call itself is cold-path (only fires on
      // controller mode switch, not per RT cycle).
      std::vector<std::atomic<uint32_t>> grown(joint_index + 1);
      for (size_t i = 0; i < pending_.size(); ++i) {
        grown[i].store(pending_[i].load(std::memory_order_relaxed),
          std::memory_order_relaxed);
      }
      pending_ = std::move(grown);
    }
    pending_[joint_index].fetch_add(1, std::memory_order_acq_rel);
  }

  {
    std::lock_guard<std::mutex> queue_lock(queue_mutex_);
    queue_.push(Request{joint_index, target_mode});
  }
  queue_cv_.notify_one();
}

uint32_t ModeSwitchWorker::pending_count(const size_t joint_index) const
{
  std::lock_guard<std::mutex> pending_lock(pending_mutex_);
  if (joint_index >= pending_.size()) {
    return 0;
  }
  return pending_[joint_index].load(std::memory_order_acquire);
}

void ModeSwitchWorker::run()
{
  while (!stop_.load(std::memory_order_acquire)) {
    Request req;
    {
      std::unique_lock<std::mutex> queue_lock(queue_mutex_);
      queue_cv_.wait(
        queue_lock, [this]() {
          return stop_.load(std::memory_order_acquire) || !queue_.empty();
        });
      if (stop_.load(std::memory_order_acquire)) {
        break;
      }
      req = queue_.front();
      queue_.pop();
    }

    if (auto * drive = get_drive_(req.joint_index)) {
      // Ignore result — HealthMonitor tracks drive failures separately.
      // A failed mode switch will surface as write failures from the RT loop.
      drive->setMotionMode(req.target_mode);
    }

    {
      std::lock_guard<std::mutex> pending_lock(pending_mutex_);
      if (req.joint_index < pending_.size()) {
        pending_[req.joint_index].fetch_sub(1, std::memory_order_acq_rel);
      }
    }
  }
}

}  // namespace mab_rehab
