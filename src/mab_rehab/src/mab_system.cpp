#include "mab_rehab/mab_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace mab_rehab
{

namespace
{

constexpr double kFallbackControlPeriodSeconds = 0.01;
constexpr double kClampEpsilon = 1e-9;
constexpr int kColdPathIoRetryAttempts = 3;
constexpr auto kColdPathIoRetryDelay = std::chrono::milliseconds(2);
constexpr size_t kVoltageTelemetryDivider = 10;

double clamp_symmetric(const double value, const double limit)
{
  if (!std::isfinite(limit) || limit <= 0.0) {
    return value;
  }
  return std::clamp(value, -limit, limit);
}

bool parse_joint_interface_name(
  const std::string & interface_name, std::string & joint_name,
  std::string & command_interface)
{
  const auto separator = interface_name.find('/');
  if (separator == std::string::npos || separator == 0 ||
    separator + 1 >= interface_name.size())
  {
    return false;
  }
  joint_name = interface_name.substr(0, separator);
  command_interface = interface_name.substr(separator + 1);
  return true;
}

template<typename Fn>
bool run_with_cold_path_retries(Fn && fn)
{
  for (int attempt = 1; attempt <= kColdPathIoRetryAttempts; ++attempt) {
    if (fn() == mab::MD::Error_t::OK) {
      return true;
    }
    if (attempt < kColdPathIoRetryAttempts) {
      std::this_thread::sleep_for(kColdPathIoRetryDelay);
    }
  }
  return false;
}

}  // namespace

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

MABSystemHardware::CallbackReturn MABSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  const auto base_result = hardware_interface::SystemInterface::on_init(params);
  if (base_result != CallbackReturn::SUCCESS) {
    return base_result;
  }

  try {
    hardware_config_ = HardwareConfig::parse(info_.hardware_parameters);
    joint_configs_ = JointConfig::parse_all(info_.joints);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Failed to parse hardware configuration: %s", error.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_logger(),
    "MAB rehab hardware configured: %zu joints on %s at %s (fast_mode=%s)",
    joint_configs_.size(), hardware_config_.bus.c_str(),
    hardware_config_.data_rate.c_str(),
    hardware_config_.fast_mode ? "true" : "false");

  return CallbackReturn::SUCCESS;
}

MABSystemHardware::CallbackReturn MABSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string error;
  if (!transport_.connect(hardware_config_, error)) {
    RCLCPP_ERROR(get_logger(), "Transport connect failed: %s", error.c_str());
    return CallbackReturn::ERROR;
  }

  // Instantiate non-RT service infrastructure now that a ROS node is available.
  auto node = get_node();
  if (!node) {
    RCLCPP_ERROR(get_logger(), "Hardware lifecycle node is not available.");
    return CallbackReturn::ERROR;
  }

  controller_switcher_ = std::make_unique<ControllerSwitcher>(node);

  std::vector<std::string> joint_names;
  joint_names.reserve(joint_configs_.size());
  for (const auto & j : joint_configs_) {
    joint_names.push_back(j.name);
  }

  HealthMonitor::Config hm_config;
  health_monitor_ = std::make_unique<HealthMonitor>(
    node, joint_names, hm_config,
    [this](const std::string & reason) {
      RCLCPP_ERROR(
        get_logger(), "Health monitor raised fault: %s", reason.c_str());
      // TODO: call ControllerSwitcher to deactivate the active controller.
    });

  return CallbackReturn::SUCCESS;
}

MABSystemHardware::CallbackReturn MABSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string error;

  // --- Set up PDS ---
  if (!pds_.setup(hardware_config_, transport_.candle(), error)) {
    RCLCPP_ERROR(get_logger(), "PDS setup failed: %s", error.c_str());
    transport_.disconnect();
    return CallbackReturn::ERROR;
  }

  // --- Create drives + allocate stats slots ---
  drives_.clear();
  drives_.reserve(joint_configs_.size());
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    auto * stats = health_monitor_ ? &health_monitor_->stats(i) : nullptr;
    drives_.push_back(
      std::make_unique<MdDrive>(joint_configs_[i], transport_.candle(), stats));
  }

  // --- Init, configure, enable each drive ---
  size_t connected_drives = 0;
  for (size_t i = 0; i < drives_.size(); ++i) {
    auto & drive = *drives_[i];
    const auto & cfg = joint_configs_[i];

    if (cfg.can_id == 0) {
      RCLCPP_INFO(
        get_logger(), "Skipping drive %s because can_id is 0", cfg.name.c_str());
      continue;
    }

    if (!drive.init()) {
      RCLCPP_WARN(
        get_logger(), "Failed to init drive %u for %s", cfg.can_id, cfg.name.c_str());
      if (!hardware_config_.allow_no_connected_drives) {
        transport_.disconnect();
        return CallbackReturn::ERROR;
      }
      continue;
    }
    drive.md()->clearErrors();

    std::string cfg_error;
    if (!drive.configure(
        hardware_config_.save_md_configuration_to_flash,
        hardware_config_.md_can_watchdog_ms,
        hardware_config_.safety_limits_enabled,
        cfg_error))
    {
      RCLCPP_WARN(
        get_logger(), "Failed to configure drive %s: %s",
        cfg.name.c_str(), cfg_error.c_str());
      if (!hardware_config_.allow_no_connected_drives) {
        transport_.disconnect();
        return CallbackReturn::ERROR;
      }
      continue;
    }

    if (!drive.apply_startup_reference(hardware_config_.startup_reference_enabled)) {
      RCLCPP_WARN(
        get_logger(), "Failed to apply startup reference for drive %s", cfg.name.c_str());
    } else if (hardware_config_.startup_reference_enabled &&
      std::isfinite(cfg.startup_reference_position))
    {
      RCLCPP_INFO(
        get_logger(),
        "Startup reference enabled for %s: parked at %.6f rad; offset=%.6f rad",
        cfg.name.c_str(), cfg.startup_reference_position, drive.position_offset());
    }

    if (!drive.enable()) {
      RCLCPP_WARN(
        get_logger(), "Failed to enable drive %s", cfg.name.c_str());
      if (!hardware_config_.allow_no_connected_drives) {
        transport_.disconnect();
        return CallbackReturn::ERROR;
      }
      continue;
    }

    if (hardware_config_.startup_in_idle_mode) {
      drive.md()->setMotionMode(mab::MdMode_E::IDLE);
      drive.set_active_mode(MdDrive::CommandMode::Idle);
    }

    ++connected_drives;
  }

  if (connected_drives == 0 && !hardware_config_.allow_no_connected_drives) {
    RCLCPP_ERROR(get_logger(), "No drives could be activated.");
    pds_.teardown();
    transport_.disconnect();
    return CallbackReturn::ERROR;
  }

  // Pre-allocate the futures buffer so write() never touches the heap.
  write_futures_.clear();
  write_futures_.resize(drives_.size());

  // --- Spin up RT primitives ---
  // NOTE: ModeSwitchWorker is intentionally not started. Mode switches stay
  // synchronous and take transport_mutex_ so the shared SPI/CANdle bus is
  // never touched concurrently by the hot path and a cold-path switch.

  if (health_monitor_) {
    health_monitor_->start();
  }

  // --- Non-RT services ---
  auto node = get_node();
  if (node && controller_switcher_) {
    maintenance_services_ = std::make_unique<MaintenanceServices>(
      node, gate_, transport_mutex_, *controller_switcher_,
      [this](const std::string & name) { return find_drive(name); },
      hardware_config_);
  }

  // --- Seed state so the controller_manager has valid initial values ---
  (void)read(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.0));
  for (auto & drive : drives_) {
    if (!drive->md() || !drive->is_operational()) { continue; }
    const auto & snap = drive->read_state(/*read_torque=*/true);
    // Seed commands to match state to avoid startup jumps.
    drive->set_position_command(snap.position);
    drive->set_velocity_command(0.0);
    drive->set_effort_command(0.0);
  }

  RCLCPP_INFO(
    get_logger(),
    "Activated mab_rehab hardware. Connected drives: %zu/%zu",
    connected_drives, drives_.size());
  return CallbackReturn::SUCCESS;
}

MABSystemHardware::CallbackReturn MABSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (health_monitor_) {
    health_monitor_->stop();
  }
  maintenance_services_.reset();

  for (auto & drive : drives_) {
    if (drive && drive->md()) {
      drive->md()->setMotionMode(mab::MdMode_E::IDLE);
      drive->disable();
    }
  }
  if (hardware_config_.disable_power_stage_on_deactivate) {
    pds_.disable_power_stage();
  }

  return CallbackReturn::SUCCESS;
}

MABSystemHardware::CallbackReturn MABSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  drives_.clear();
  write_futures_.clear();
  pds_.teardown();
  transport_.disconnect();
  health_monitor_.reset();
  controller_switcher_.reset();
  return CallbackReturn::SUCCESS;
}

MABSystemHardware::CallbackReturn MABSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return on_cleanup(rclcpp_lifecycle::State());
}

// ---------------------------------------------------------------------------
// RT hot path
// ---------------------------------------------------------------------------

hardware_interface::return_type MABSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (gate_.rt_should_skip_cycle()) {
    return hardware_interface::return_type::OK;
  }

  // A non-RT mode switch or maintenance operation currently owns the bus.
  // Skip this cycle instead of blocking the control loop.
  std::unique_lock<std::mutex> transport_lock(transport_mutex_, std::try_to_lock);
  if (!transport_lock.owns_lock()) {
    return hardware_interface::return_type::OK;
  }

  const bool read_torque = !hardware_config_.fast_mode;
  const bool telemetry_cycle = (read_cycle_counter_ % kVoltageTelemetryDivider) == 0;

  size_t failed = 0;
  for (auto & drive : drives_) {
    if (!drive || !drive->md() || !drive->is_operational()) { continue; }
    const auto & snap = drive->read_state(read_torque);
    if (!snap.fresh) {
      ++failed;
    } else {
      publish_state_from_snapshot(*drive, snap);
    }
  }

  // Keep voltage telemetry at the old 10-cycle cadence, but without exposing
  // telemetry_divider as a public runtime parameter.
  if (telemetry_cycle && failed == 0) {
    for (auto & drive : drives_) {
      if (
        !drive || !drive->md() || !drive->is_operational() ||
        !drive->config().has_voltage_state)
      {
        continue;
      }
      drive->read_voltage();
      set_state(drive->config().name + "/voltage", drive->cached_state().voltage);
    }
    if (!info_.sensors.empty() && pds_.is_power_stage_exported()) {
      if (const auto voltage = pds_.read_bus_voltage(); voltage.has_value()) {
        set_state(info_.sensors.front().name + "/bus_voltage", *voltage);
      }
    }
  }

  ++read_cycle_counter_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MABSystemHardware::write(
  const rclcpp::Time & now, const rclcpp::Duration & /*period*/)
{
  if (gate_.rt_should_skip_cycle()) {
    return hardware_interface::return_type::OK;
  }

  // A non-RT mode switch or maintenance operation currently owns the bus.
  // Skip this cycle instead of blocking the control loop.
  std::unique_lock<std::mutex> transport_lock(transport_mutex_, std::try_to_lock);
  if (!transport_lock.owns_lock()) {
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < drives_.size(); ++i) {
    auto & drive = *drives_[i];
    const auto & cfg = joint_configs_[i];
    if (!drive.md() || !drive.is_operational()) { continue; }
    if (drive.active_mode() == MdDrive::CommandMode::Idle) { continue; }

    double position_cmd = cfg.has_position_command ?
      get_command<double>(cfg.name + "/" + hardware_interface::HW_IF_POSITION) : 0.0;
    double velocity_cmd = cfg.has_velocity_command ?
      get_command<double>(cfg.name + "/" + hardware_interface::HW_IF_VELOCITY) : 0.0;
    double effort_cmd = cfg.has_effort_command ?
      get_command<double>(cfg.name + "/" + hardware_interface::HW_IF_EFFORT) : 0.0;

    const auto & snap = drive.cached_state();
    if (!std::isfinite(position_cmd)) { position_cmd = snap.position; }
    if (!std::isfinite(velocity_cmd)) { velocity_cmd = 0.0; }
    if (!std::isfinite(effort_cmd)) { effort_cmd = 0.0; }

    if (hardware_config_.safety_limits_enabled) {
      apply_safety_clamp(cfg, snap, position_cmd, velocity_cmd, effort_cmd);
    }

    drive.set_position_command(position_cmd);
    drive.set_velocity_command(velocity_cmd);
    drive.set_effort_command(effort_cmd);
    write_futures_[i] = drive.fire_write_async();
  }

  // Pass 2: collect all async results. All CAN frames were dispatched
  // concurrently in Pass 1; this loop only waits for completions.
  for (size_t i = 0; i < drives_.size(); ++i) {
    drives_[i]->collect_write_result(std::move(write_futures_[i]));
  }

  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Command mode switch (RT-safe via ModeSwitchWorker)
// ---------------------------------------------------------------------------

hardware_interface::return_type MABSystemHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  auto validate = [this](const std::string & interface_name) {
      std::string joint_name;
      std::string command_interface;
      if (!parse_joint_interface_name(interface_name, joint_name, command_interface)) {
        return true;
      }
      if (command_interface != hardware_interface::HW_IF_POSITION &&
        command_interface != hardware_interface::HW_IF_VELOCITY &&
        command_interface != hardware_interface::HW_IF_EFFORT)
      {
        return true;
      }
      if (!find_drive(joint_name)) {
        RCLCPP_ERROR(
          get_logger(), "Mode switch references unknown joint %s", joint_name.c_str());
        return false;
      }
      return true;
    };

  if (!std::all_of(start_interfaces.begin(), start_interfaces.end(), validate) ||
    !std::all_of(stop_interfaces.begin(), stop_interfaces.end(), validate))
  {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MABSystemHardware::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  struct Request
  {
    bool position = false;
    bool velocity = false;
    bool effort = false;
  };
  std::vector<Request> per_joint(drives_.size());

  for (const auto & interface_name : start_interfaces) {
    std::string joint_name;
    std::string command_interface;
    if (!parse_joint_interface_name(interface_name, joint_name, command_interface)) {
      continue;
    }
    const auto it = std::find_if(
      joint_configs_.begin(), joint_configs_.end(),
      [&joint_name](const JointConfig & j) { return j.name == joint_name; });
    if (it == joint_configs_.end()) { continue; }
    const size_t index = std::distance(joint_configs_.begin(), it);

    if (command_interface == hardware_interface::HW_IF_POSITION) {
      per_joint[index].position = true;
    } else if (command_interface == hardware_interface::HW_IF_VELOCITY) {
      per_joint[index].velocity = true;
    } else if (command_interface == hardware_interface::HW_IF_EFFORT) {
      per_joint[index].effort = true;
    }
  }

  std::lock_guard<std::mutex> transport_lock(transport_mutex_);

  for (size_t i = 0; i < drives_.size(); ++i) {
    const auto & req = per_joint[i];
    if (!req.position && !req.velocity && !req.effort) { continue; }
    auto & drive = *drives_[i];
    const auto & cfg = joint_configs_[i];
    auto * md = drive.md();

    MdDrive::CommandMode target;
    mab::MdMode_E sdk_mode;
    if (req.effort && !req.position && !req.velocity) {
      target = MdDrive::CommandMode::Impedance;
      sdk_mode = mab::MdMode_E::IMPEDANCE;
    } else if (req.velocity && !req.position && !req.effort) {
      target = MdDrive::CommandMode::Velocity;
      sdk_mode = mab::MdMode_E::VELOCITY_PID;
    } else {
      target = MdDrive::CommandMode::Position;
      sdk_mode = mab::MdMode_E::POSITION_PID;
    }

    if (drive.active_mode() == target) { continue; }

    if (!md || !drive.is_operational()) {
      drive.set_active_mode(target);
      RCLCPP_WARN(
        get_logger(),
        "Skipping motion-mode switch for non-operational joint %s (requested: %s)",
        cfg.name.c_str(),
        target == MdDrive::CommandMode::Impedance ? "IMPEDANCE" :
        target == MdDrive::CommandMode::Velocity ? "VELOCITY_PID" : "POSITION_PID");
      continue;
    }

    if (target == MdDrive::CommandMode::Impedance) {
      if (!run_with_cold_path_retries(
          [&]() {
            return md->setImpedanceParams(
              static_cast<float>(cfg.impedance_kp),
              static_cast<float>(cfg.impedance_kd));
          }))
      {
        RCLCPP_ERROR(
          get_logger(), "Failed to set impedance gains for joint %s after %d attempts",
          cfg.name.c_str(), kColdPathIoRetryAttempts);
        return hardware_interface::return_type::ERROR;
      }
    }

    // perform_command_mode_switch() runs under transport_mutex_ so the
    // read/write hot path cannot race the shared SPI/CANdle bus while we
    // update the drive mode and preload targets.
    if (!run_with_cold_path_retries([&]() { return md->setMotionMode(sdk_mode); })) {
      RCLCPP_ERROR(
        get_logger(), "Failed to switch joint %s to %s after %d attempts",
        cfg.name.c_str(),
        target == MdDrive::CommandMode::Impedance ? "IMPEDANCE" :
        target == MdDrive::CommandMode::Velocity ? "VELOCITY_PID" : "POSITION_PID",
        kColdPathIoRetryAttempts);
      return hardware_interface::return_type::ERROR;
    }

    const auto & snap = drive.cached_state();
    if (target == MdDrive::CommandMode::Position) {
      const double hold_position = hardware_config_.safety_limits_enabled ?
        std::clamp(snap.position, cfg.safety_position_min, cfg.safety_position_max) :
        snap.position;
      drive.set_position_command(hold_position);
      drive.set_velocity_command(0.0);
      drive.set_effort_command(0.0);
      md->m_mdRegisters.targetPosition =
        static_cast<float>(hold_position - drive.position_offset());
      md->m_mdRegisters.targetVelocity = 0.0F;
      if (!run_with_cold_path_retries(
          [&]() {
            return md->writeRegisters(
              md->m_mdRegisters.targetPosition,
              md->m_mdRegisters.targetVelocity);
          }))
      {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to preload target position/velocity for joint %s after %d attempts",
          cfg.name.c_str(), kColdPathIoRetryAttempts);
        return hardware_interface::return_type::ERROR;
      }
      if (cfg.has_position_command) {
        set_command(cfg.name + "/" + hardware_interface::HW_IF_POSITION, hold_position);
      }
    } else if (target == MdDrive::CommandMode::Velocity) {
      drive.set_position_command(snap.position);
      drive.set_velocity_command(0.0);
      drive.set_effort_command(0.0);
      md->m_mdRegisters.targetVelocity = 0.0F;
      if (!run_with_cold_path_retries(
          [&]() { return md->writeRegisters(md->m_mdRegisters.targetVelocity); }))
      {
        RCLCPP_ERROR(
          get_logger(), "Failed to preload target velocity for joint %s after %d attempts",
          cfg.name.c_str(), kColdPathIoRetryAttempts);
        return hardware_interface::return_type::ERROR;
      }
      if (cfg.has_position_command) {
        set_command(cfg.name + "/" + hardware_interface::HW_IF_POSITION, snap.position);
      }
    } else {
      drive.set_position_command(snap.position);
      drive.set_velocity_command(0.0);
      drive.set_effort_command(0.0);
      md->m_mdRegisters.targetTorque = 0.0F;
      if (!run_with_cold_path_retries(
          [&]() { return md->writeRegisters(md->m_mdRegisters.targetTorque); }))
      {
        RCLCPP_ERROR(
          get_logger(), "Failed to preload target torque for joint %s after %d attempts",
          cfg.name.c_str(), kColdPathIoRetryAttempts);
        return hardware_interface::return_type::ERROR;
      }
      if (cfg.has_position_command) {
        set_command(cfg.name + "/" + hardware_interface::HW_IF_POSITION, snap.position);
      }
    }

    if (cfg.has_velocity_command) {
      set_command(cfg.name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
    }
    if (cfg.has_effort_command) {
      set_command(cfg.name + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
    }

    drive.reset_write_cache();
    drive.set_active_mode(target);
    RCLCPP_INFO(
      get_logger(), "Joint %s command mode switched to %s",
      cfg.name.c_str(),
      target == MdDrive::CommandMode::Impedance ? "IMPEDANCE" :
      target == MdDrive::CommandMode::Velocity ? "VELOCITY_PID" : "POSITION_PID");
  }

  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

MdDrive * MABSystemHardware::find_drive(const std::string & joint_name)
{
  for (auto & drive : drives_) {
    if (drive && drive->config().name == joint_name) {
      return drive.get();
    }
  }
  return nullptr;
}

void MABSystemHardware::publish_state_from_snapshot(
  const MdDrive & drive, const MdDrive::StateSnapshot & snap)
{
  const auto & cfg = drive.config();
  if (cfg.has_position_state) {
    set_state(cfg.name + "/" + hardware_interface::HW_IF_POSITION, snap.position);
  }
  if (cfg.has_velocity_state) {
    set_state(cfg.name + "/" + hardware_interface::HW_IF_VELOCITY, snap.velocity);
  }
  if (cfg.has_effort_state) {
    set_state(cfg.name + "/" + hardware_interface::HW_IF_EFFORT, snap.effort);
  }
  if (cfg.has_gear_ratio_state) {
    set_state(cfg.name + "/gear_ratio", cfg.gear_ratio);
  }
}

void MABSystemHardware::apply_safety_clamp(
  const JointConfig & joint, const MdDrive::StateSnapshot & state,
  double & position_cmd, double & velocity_cmd, double & effort_cmd)
{
  position_cmd = std::clamp(
    position_cmd, joint.safety_position_min, joint.safety_position_max);
  velocity_cmd = clamp_symmetric(velocity_cmd, joint.safety_max_velocity);
  effort_cmd = clamp_symmetric(effort_cmd, joint.safety_max_torque);

  // Stop velocity commands that drive into the position envelope.
  if (state.position <= joint.safety_position_min + kClampEpsilon && velocity_cmd < 0.0) {
    velocity_cmd = 0.0;
  }
  if (state.position >= joint.safety_position_max - kClampEpsilon && velocity_cmd > 0.0) {
    velocity_cmd = 0.0;
  }
}

}  // namespace mab_rehab

PLUGINLIB_EXPORT_CLASS(mab_rehab::MABSystemHardware, hardware_interface::SystemInterface)
