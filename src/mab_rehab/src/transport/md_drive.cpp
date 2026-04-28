#include "mab_rehab/transport/md_drive.hpp"

#include <chrono>
#include <cmath>
#include <thread>
#include <utility>

#include "mab_rehab/maintenance/md_config_uploader.hpp"

namespace mab_rehab
{

namespace
{

constexpr float kCommandChangeEpsilon = 1e-5F;
constexpr auto kCommandRefreshPeriod = std::chrono::milliseconds(100);

bool changed(const std::optional<float> & last, const float next)
{
  return !last.has_value() || std::fabs(*last - next) > kCommandChangeEpsilon;
}

}  // namespace

MdDrive::MdDrive(JointConfig config, mab::Candle * candle, HealthMonitor::DriveStats * stats)
: config_(std::move(config)),
  candle_(candle),
  stats_(stats)
{
}

bool MdDrive::init()
{
  operational_ = false;
  md_ = std::make_unique<mab::MD>(config_.can_id, candle_);
  if (md_->init() != mab::MD::Error_t::OK) {
    md_.reset();
    return false;
  }
  return true;
}

bool MdDrive::configure(
  const bool save_to_flash, const double can_watchdog_ms,
  const bool safety_limits_enabled, std::string & error_out)
{
  if (!md_) {
    error_out = "MD handle is null";
    return false;
  }

  // 1. Upload the full motor config from the .cfg file (overrides most registers).
  if (!config_.config_path.empty()) {
    if (!upload_md_config(*md_, config_.config_path, error_out, /*save_to_flash=*/false)) {
      return false;
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // 2. Mirror the active safety envelope into the drive so the MD enforces
  //    the same bounds even if a ROS-side clamp is bypassed.
  if (safety_limits_enabled) {
    if (std::isfinite(config_.safety_position_min)) {
      md_->m_mdRegisters.positionLimitMin = static_cast<float>(config_.safety_position_min);
      if (md_->writeRegisters(md_->m_mdRegisters.positionLimitMin) != mab::MD::Error_t::OK) {
        error_out = "Failed to set safety position min";
        return false;
      }
    }
    if (std::isfinite(config_.safety_position_max)) {
      md_->m_mdRegisters.positionLimitMax = static_cast<float>(config_.safety_position_max);
      if (md_->writeRegisters(md_->m_mdRegisters.positionLimitMax) != mab::MD::Error_t::OK) {
        error_out = "Failed to set safety position max";
        return false;
      }
    }
    if (std::isfinite(config_.safety_max_torque)) {
      md_->m_mdRegisters.maxTorque = static_cast<float>(config_.safety_max_torque);
      if (md_->writeRegisters(md_->m_mdRegisters.maxTorque) != mab::MD::Error_t::OK) {
        error_out = "Failed to set safety max torque";
        return false;
      }
    }
    if (std::isfinite(config_.safety_max_velocity)) {
      md_->m_mdRegisters.maxVelocity = static_cast<float>(config_.safety_max_velocity);
      if (md_->writeRegisters(md_->m_mdRegisters.maxVelocity) != mab::MD::Error_t::OK) {
        error_out = "Failed to set safety max velocity";
        return false;
      }
    }
    if (std::isfinite(config_.safety_max_acceleration)) {
      md_->m_mdRegisters.maxAcceleration =
        static_cast<float>(config_.safety_max_acceleration);
      if (md_->writeRegisters(md_->m_mdRegisters.maxAcceleration) != mab::MD::Error_t::OK) {
        error_out = "Failed to set safety max acceleration";
        return false;
      }
    }
    if (std::isfinite(config_.safety_max_deceleration)) {
      md_->m_mdRegisters.maxDeceleration =
        static_cast<float>(config_.safety_max_deceleration);
      if (md_->writeRegisters(md_->m_mdRegisters.maxDeceleration) != mab::MD::Error_t::OK) {
        error_out = "Failed to set safety max deceleration";
        return false;
      }
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // 3. Override PID gains from the parsed JointConfig. This is the single
  //    source of truth for controller tuning — any .cfg gains are overwritten.
  if (md_->setPositionPIDparam(
      static_cast<float>(config_.position_pid_kp),
      static_cast<float>(config_.position_pid_ki),
      static_cast<float>(config_.position_pid_kd),
      static_cast<float>(config_.position_pid_windup)) != mab::MD::Error_t::OK)
  {
    error_out = "Failed to set position PID gains";
    return false;
  }
  if (md_->setVelocityPIDparam(
      static_cast<float>(config_.velocity_pid_kp),
      static_cast<float>(config_.velocity_pid_ki),
      static_cast<float>(config_.velocity_pid_kd),
      static_cast<float>(config_.velocity_pid_windup)) != mab::MD::Error_t::OK)
  {
    error_out = "Failed to set velocity PID gains";
    return false;
  }
  if (md_->setImpedanceParams(
      static_cast<float>(config_.impedance_kp),
      static_cast<float>(config_.impedance_kd)) != mab::MD::Error_t::OK)
  {
    error_out = "Failed to set impedance gains";
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  // 4. Friction compensation — dynamic (motorFriction) and static (motorStiction).
  md_->m_mdRegisters.motorFriction = static_cast<float>(config_.motor_friction);
  if (md_->writeRegisters(md_->m_mdRegisters.motorFriction) != mab::MD::Error_t::OK) {
    error_out = "Failed to set motor friction";
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  md_->m_mdRegisters.motorStiction = static_cast<float>(config_.motor_stiction);
  if (md_->writeRegisters(md_->m_mdRegisters.motorStiction) != mab::MD::Error_t::OK) {
    error_out = "Failed to set motor stiction";
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  if (std::isfinite(can_watchdog_ms)) {
    md_->m_mdRegisters.canWatchdog = static_cast<u16>(std::lround(can_watchdog_ms));
    if (md_->writeRegisters(md_->m_mdRegisters.canWatchdog) != mab::MD::Error_t::OK) {
      error_out = "Failed to set CAN watchdog";
      return false;
    }
  }

  if (save_to_flash && md_->save() != mab::MD::Error_t::OK) {
    error_out = "Failed to save configuration to flash";
    return false;
  }

  return true;
}

bool MdDrive::apply_startup_reference(const bool reference_enabled)
{
  position_offset_ = 0.0;
  if (!reference_enabled || !std::isfinite(config_.startup_reference_position)) {
    return true;
  }

  if (md_->readRegisters(md_->m_mdRegisters.mainEncoderPosition) != mab::MD::Error_t::OK) {
    return false;
  }
  const double raw_position =
    static_cast<double>(md_->m_mdRegisters.mainEncoderPosition.value);
  position_offset_ = config_.startup_reference_position - raw_position;
  return true;
}

bool MdDrive::enable()
{
  operational_ = md_ && md_->enable() == mab::MD::Error_t::OK;
  return operational_;
}

bool MdDrive::disable()
{
  operational_ = false;
  return md_ && md_->disable() == mab::MD::Error_t::OK;
}

bool MdDrive::zero() { return md_ && md_->zero() == mab::MD::Error_t::OK; }

const MdDrive::StateSnapshot & MdDrive::read_state(const bool read_torque) noexcept
{
  if (!md_) {
    state_.fresh = false;
    return state_;
  }

  // Single attempt — retry policy is 1/cycle, counted in the RT thread,
  // and escalated by HealthMonitor on the non-RT thread.
  const auto result = read_torque ?
    md_->readRegisters(
    md_->m_mdRegisters.mainEncoderPosition,
    md_->m_mdRegisters.mainEncoderVelocity,
    md_->m_mdRegisters.motorTorque) :
    md_->readRegisters(
    md_->m_mdRegisters.mainEncoderPosition,
    md_->m_mdRegisters.mainEncoderVelocity);

  if (result != mab::MD::Error_t::OK) {
    state_.fresh = false;
    if (stats_) {
      stats_->read_fail_count.fetch_add(1, std::memory_order_relaxed);
      stats_->consecutive_read_fails.fetch_add(1, std::memory_order_relaxed);
    }
    return state_;
  }

  state_.position =
    static_cast<double>(md_->m_mdRegisters.mainEncoderPosition.value) + position_offset_;
  state_.velocity = static_cast<double>(md_->m_mdRegisters.mainEncoderVelocity.value);
  if (read_torque) {
    state_.effort = static_cast<double>(md_->m_mdRegisters.motorTorque.value);
  }
  state_.fresh = true;
  if (stats_) {
    stats_->consecutive_read_fails.store(0, std::memory_order_relaxed);
  }
  return state_;
}

void MdDrive::read_voltage() noexcept
{
  if (!md_) { return; }
  if (md_->readRegisters(md_->m_mdRegisters.dcBusVoltage) != mab::MD::Error_t::OK) {
    return;  // keep last-known voltage
  }
  state_.voltage = static_cast<double>(md_->m_mdRegisters.dcBusVoltage.value);
}

void MdDrive::set_position_command(const double radians) noexcept
{
  position_command_ = radians;
}

void MdDrive::set_velocity_command(const double rad_per_s) noexcept
{
  velocity_command_ = rad_per_s;
}

void MdDrive::set_effort_command(const double nm) noexcept
{
  effort_command_ = nm;
}

void MdDrive::write_command(const rclcpp::Time & /*now_stamp*/) noexcept
{
  if (!md_) { return; }

  const auto mode = active_mode_.load(std::memory_order_acquire);
  if (mode == CommandMode::Idle) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const bool refresh_due = now - last_write_time_ >= kCommandRefreshPeriod;

  // The startup-reference offset was applied at read time, so the
  // command from the controller is already in "joint space" (matches state).
  // Convert back to motor space by subtracting the offset.
  const auto position_cmd_f = static_cast<float>(position_command_ - position_offset_);
  const auto velocity_cmd_f = static_cast<float>(velocity_command_);
  const auto effort_cmd_f = static_cast<float>(effort_command_);

  mab::MD::Error_t result = mab::MD::Error_t::OK;
  bool sent = false;

  if (mode == CommandMode::Position) {
    if (changed(last_sent_position_, position_cmd_f) ||
      changed(last_sent_velocity_, velocity_cmd_f) || refresh_due)
    {
      md_->m_mdRegisters.targetPosition = position_cmd_f;
      md_->m_mdRegisters.targetVelocity = velocity_cmd_f;
      result = md_->writeRegisters(
        md_->m_mdRegisters.targetPosition,
        md_->m_mdRegisters.targetVelocity);
      sent = true;
      if (result == mab::MD::Error_t::OK) {
        last_sent_position_ = position_cmd_f;
        last_sent_velocity_ = velocity_cmd_f;
      }
    }
  } else if (mode == CommandMode::Velocity) {
    if (changed(last_sent_velocity_, velocity_cmd_f) || refresh_due) {
      md_->m_mdRegisters.targetVelocity = velocity_cmd_f;
      result = md_->writeRegisters(md_->m_mdRegisters.targetVelocity);
      sent = true;
      if (result == mab::MD::Error_t::OK) {
        last_sent_velocity_ = velocity_cmd_f;
      }
    }
  } else {  // Impedance
    if (changed(last_sent_effort_, effort_cmd_f) || refresh_due) {
      md_->m_mdRegisters.targetTorque = effort_cmd_f;
      result = md_->writeRegisters(md_->m_mdRegisters.targetTorque);
      sent = true;
      if (result == mab::MD::Error_t::OK) {
        last_sent_effort_ = effort_cmd_f;
      }
    }
  }

  if (!sent) {
    return;
  }

  if (result == mab::MD::Error_t::OK) {
    last_write_time_ = now;
    if (stats_) {
      stats_->consecutive_write_fails.store(0, std::memory_order_relaxed);
    }
  } else if (stats_) {
    stats_->write_fail_count.fetch_add(1, std::memory_order_relaxed);
    stats_->consecutive_write_fails.fetch_add(1, std::memory_order_relaxed);
  }
}

// ---------------------------------------------------------------------------
// Parallel-write API (fire + collect, two-pass pattern)
// ---------------------------------------------------------------------------

std::future<mab::MD::Error_t> MdDrive::fire_write_async() noexcept
{
  if (!md_) { return {}; }

  const auto mode = active_mode_.load(std::memory_order_acquire);
  if (mode == CommandMode::Idle) { return {}; }

  const auto now = std::chrono::steady_clock::now();
  const bool refresh_due = now - last_write_time_ >= kCommandRefreshPeriod;

  // Startup-reference offset: command is in joint space; convert to motor space.
  const auto position_cmd_f = static_cast<float>(position_command_ - position_offset_);
  const auto velocity_cmd_f = static_cast<float>(velocity_command_);
  const auto effort_cmd_f   = static_cast<float>(effort_command_);

  if (mode == CommandMode::Position) {
    if (!changed(last_sent_position_, position_cmd_f) &&
      !changed(last_sent_velocity_, velocity_cmd_f) && !refresh_due)
    {
      return {};
    }
    pending_mode_       = mode;
    pending_position_f_ = position_cmd_f;
    pending_velocity_f_ = velocity_cmd_f;
    pending_write_time_ = now;
    md_->m_mdRegisters.targetPosition = position_cmd_f;
    md_->m_mdRegisters.targetVelocity = velocity_cmd_f;
    // Registers are serialised synchronously inside writeRegistersAsync before
    // the async thread is spawned, so modifying m_mdRegisters afterwards is safe.
    return md_->writeRegistersAsync(
      md_->m_mdRegisters.targetPosition,
      md_->m_mdRegisters.targetVelocity);
  } else if (mode == CommandMode::Velocity) {
    if (!changed(last_sent_velocity_, velocity_cmd_f) && !refresh_due) { return {}; }
    pending_mode_       = mode;
    pending_velocity_f_ = velocity_cmd_f;
    pending_write_time_ = now;
    md_->m_mdRegisters.targetVelocity = velocity_cmd_f;
    return md_->writeRegistersAsync(md_->m_mdRegisters.targetVelocity);
  } else {  // Impedance
    if (!changed(last_sent_effort_, effort_cmd_f) && !refresh_due) { return {}; }
    pending_mode_     = mode;
    pending_effort_f_ = effort_cmd_f;
    pending_write_time_ = now;
    md_->m_mdRegisters.targetTorque = effort_cmd_f;
    return md_->writeRegistersAsync(md_->m_mdRegisters.targetTorque);
  }
}

void MdDrive::collect_write_result(std::future<mab::MD::Error_t> pending) noexcept
{
  if (!pending.valid()) { return; }

  const auto result = pending.get();

  if (result == mab::MD::Error_t::OK) {
    if (pending_mode_ == CommandMode::Position) {
      last_sent_position_ = pending_position_f_;
      last_sent_velocity_ = pending_velocity_f_;
    } else if (pending_mode_ == CommandMode::Velocity) {
      last_sent_velocity_ = pending_velocity_f_;
    } else {
      last_sent_effort_ = pending_effort_f_;
    }
    last_write_time_ = pending_write_time_;
    if (stats_) {
      stats_->consecutive_write_fails.store(0, std::memory_order_relaxed);
    }
  } else if (stats_) {
    stats_->write_fail_count.fetch_add(1, std::memory_order_relaxed);
    stats_->consecutive_write_fails.fetch_add(1, std::memory_order_relaxed);
  }
}

}  // namespace mab_rehab
