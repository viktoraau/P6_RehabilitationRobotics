#include "mab_rehab/rt/health_monitor.hpp"

#include <chrono>

namespace mab_rehab
{

HealthMonitor::HealthMonitor(
  rclcpp::Node::SharedPtr node,
  std::vector<std::string> joint_names,
  Config config,
  FaultHandler on_fault)
: node_(std::move(node)),
  joint_names_(std::move(joint_names)),
  config_(config),
  on_fault_(std::move(on_fault)),
  stats_(joint_names_.size())
{
}

void HealthMonitor::start()
{
  if (!node_ || timer_ || config_.poll_rate_hz <= 0) {
    return;
  }
  const auto period =
    std::chrono::milliseconds(1000 / config_.poll_rate_hz);
  timer_ = node_->create_wall_timer(period, [this]() { poll(); });
}

void HealthMonitor::stop()
{
  timer_.reset();
}

void HealthMonitor::poll()
{
  for (size_t i = 0; i < stats_.size(); ++i) {
    auto & s = stats_[i];
    const auto consecutive_reads =
      s.consecutive_read_fails.load(std::memory_order_relaxed);
    const auto consecutive_writes =
      s.consecutive_write_fails.load(std::memory_order_relaxed);

    if (consecutive_reads > config_.warn_threshold ||
      consecutive_writes > config_.warn_threshold)
    {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Joint %s degraded: %u consecutive read fails, %u consecutive write fails",
        joint_names_[i].c_str(), consecutive_reads, consecutive_writes);
    }

    if ((consecutive_reads > config_.fault_threshold ||
      consecutive_writes > config_.fault_threshold) &&
      !fault_raised_.exchange(true))
    {
      if (on_fault_) {
        on_fault_(
          "Joint " + joint_names_[i] + " exceeded fault threshold (" +
          std::to_string(config_.fault_threshold) + " consecutive failures)");
      }
    }
  }
}

}  // namespace mab_rehab
