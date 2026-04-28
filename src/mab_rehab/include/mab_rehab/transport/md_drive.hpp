#pragma once

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <optional>
#include <string>

#include "MD.hpp"
#include "candle.hpp"
#include "candle_types.hpp"

#include "mab_rehab/config/joint_config.hpp"
#include "mab_rehab/rt/health_monitor.hpp"

namespace mab_rehab
{

/// One MAB MD drive, owning its own mab::MD handle and its per-cycle
/// state cache. The RT thread calls read_state() / write_command()
/// exactly once per cycle; neither method retries, allocates, or logs.
///
/// Failures are reported by incrementing the linked HealthMonitor stats;
/// the cached state/command lingers on failure ("hold last good value"),
/// matching the safety semantics agreed for rehab operation.
class MdDrive
{
public:
  enum class CommandMode : uint8_t
  {
    Idle,
    Position,
    Velocity,
    Impedance,
  };

  struct StateSnapshot
  {
    double position = 0.0;   // radians, after startup-reference offset
    double velocity = 0.0;   // rad/s
    double effort = 0.0;     // Nm (only populated when read_torque=true)
    double voltage = 0.0;    // V (refreshed on telemetry cycles)
    bool fresh = false;      // true if this cycle's read succeeded
  };

  MdDrive(JointConfig config, mab::Candle * candle, HealthMonitor::DriveStats * stats);

  const JointConfig & config() const { return config_; }

  /// The underlying mab::MD handle. Non-null after init() succeeds.
  /// Only maintenance code should touch this directly.
  mab::MD * md() { return md_.get(); }

  /// One-shot setup: creates the MD handle and pings the drive.
  /// Returns true if the drive responded.
  bool init();

  /// True once the drive completed the activation sequence and accepted
  /// enable. Drives that fail activation may still answer register reads,
  /// but they must be excluded from command/state loops.
  bool is_operational() const noexcept { return operational_; }

  /// Upload the per-joint .cfg file, optionally mirror the active safety
  /// envelope into the MD limit registers, and override PID/impedance gains
  /// with JointConfig values. Cold-path; runs in on_activate().
  bool configure(
    bool save_to_flash, double can_watchdog_ms, bool safety_limits_enabled,
    std::string & error_out);

  /// Apply the startup reference offset based on the drive's current
  /// raw position. Called once per on_activate() after configure().
  bool apply_startup_reference(bool reference_enabled);

  /// Enable the drive controllers (disable on deactivate).
  bool enable();
  bool disable();

  /// Zero the drive encoder. Cold-path.
  bool zero();

  // --- RT hot path ---

  /// Read one state snapshot. Single SPI transaction per registered
  /// register; no retry. Updates internal cache and returns a const
  /// reference to it. Increments stats_->read_fail_count on failure.
  /// read_torque controls whether the torque register is included.
  const StateSnapshot & read_state(bool read_torque) noexcept;

  /// Refresh the cached voltage. Called at a fixed low-rate cadence so
  /// the control loop still exports voltage without exposing a runtime knob.
  void read_voltage() noexcept;

  /// Last snapshot from read_state(), without performing a new transfer.
  /// Fresh==false if the most recent read failed. RT-safe.
  const StateSnapshot & cached_state() const noexcept { return state_; }

  /// Parallel-write API (two-pass alternative to write_command):
  ///
  ///  (1) fire_write_async() — applies the same change-detect / refresh logic
  ///      as write_command(), stages the register values, dispatches the CAN
  ///      frame on a background thread via writeRegistersAsync(), and returns
  ///      the future.  Returns an invalid future when there is nothing to send.
  ///
  ///  (2) collect_write_result() — blocks until the future completes, then
  ///      updates last_sent_* cache and error stats exactly as write_command()
  ///      does.  No-op when passed an invalid future.
  ///
  /// Caller pattern in write():
  ///   Pass 1 — for each drive:  futures[i] = drive.fire_write_async();
  ///   Pass 2 — for each drive:  drive.collect_write_result(std::move(futures[i]));
  std::future<mab::MD::Error_t> fire_write_async() noexcept;
  void collect_write_result(std::future<mab::MD::Error_t> pending) noexcept;

  /// Write the current command to the drive, taking active_mode_ into
  /// account. If the command hasn't changed since last cycle and the
  /// refresh window hasn't expired, the write is skipped (mirrors the
  /// current kCommandRefreshPeriod logic).
  void write_command(const rclcpp::Time & now_stamp) noexcept;

  /// Forget the last successfully written command so the next cycle
  /// re-sends the freshly selected mode's preload targets.
  void reset_write_cache() noexcept
  {
    last_sent_position_.reset();
    last_sent_velocity_.reset();
    last_sent_effort_.reset();
    last_write_time_ = {};
  }

  /// RT-safe. Set the target command. Called by the plugin's write()
  /// before write_command(); never blocks.
  void set_position_command(double radians) noexcept;
  void set_velocity_command(double rad_per_s) noexcept;
  void set_effort_command(double nm) noexcept;

  /// RT-safe. Flip the active mode. ModeSwitchWorker does the SPI
  /// write asynchronously; this flag only tells write_command() which
  /// target register to use.
  void set_active_mode(CommandMode mode) noexcept
  {
    active_mode_.store(mode, std::memory_order_release);
  }
  CommandMode active_mode() const noexcept
  {
    return active_mode_.load(std::memory_order_acquire);
  }

  /// Position offset applied to raw MD readings so that state_position
  /// matches the URDF frame. Set by apply_startup_reference().
  double position_offset() const noexcept { return position_offset_; }

private:
  JointConfig config_;
  mab::Candle * candle_{nullptr};
  HealthMonitor::DriveStats * stats_{nullptr};

  std::unique_ptr<mab::MD> md_;
  StateSnapshot state_{};
  double position_offset_{0.0};
  bool operational_{false};

  std::atomic<CommandMode> active_mode_{CommandMode::Idle};

  // Command cache (RT-side-written, read only by write_command()).
  double position_command_{0.0};
  double velocity_command_{0.0};
  double effort_command_{0.0};
  std::optional<float> last_sent_position_{};
  std::optional<float> last_sent_velocity_{};
  std::optional<float> last_sent_effort_{};
  std::chrono::steady_clock::time_point last_write_time_{};

  // Pending async-write state: written by fire_write_async(), read by collect_write_result().
  CommandMode pending_mode_{CommandMode::Idle};
  float pending_position_f_{0.0f};
  float pending_velocity_f_{0.0f};
  float pending_effort_f_{0.0f};
  std::chrono::steady_clock::time_point pending_write_time_{};
};

}  // namespace mab_rehab
