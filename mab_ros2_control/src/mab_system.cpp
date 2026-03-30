#include "mab_ros2_control/mab_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace mab_ros2_control
{

namespace
{
constexpr int kPdsInitAttempts = 5;
constexpr auto kPdsRetryDelay = std::chrono::milliseconds(150);
constexpr int kJointReadRetryAttempts = 3;
constexpr int kJointWriteRetryAttempts = 3;
constexpr float kCommandChangeEpsilon = 1e-5F;

double clamp_symmetric(const double value, const double limit)
{
  if (!std::isfinite(limit) || limit <= 0.0) {
    return value;
  }
  return std::clamp(value, -limit, limit);
}

template <typename T>
bool is_sdk_ok(const T result)
{
  return result == T::OK;
}

bool is_command_interface_claimed(const std::string & interface_name)
{
  return (
    interface_name.ends_with("/" + std::string(hardware_interface::HW_IF_POSITION)) ||
    interface_name.ends_with("/" + std::string(hardware_interface::HW_IF_VELOCITY)) ||
    interface_name.ends_with("/" + std::string(hardware_interface::HW_IF_EFFORT)) ||
    interface_name == hardware_interface::HW_IF_POSITION ||
    interface_name == hardware_interface::HW_IF_VELOCITY ||
    interface_name == hardware_interface::HW_IF_EFFORT);
}

bool has_meaningful_change(
  const std::optional<float> & previous_value, const float next_value,
  const float epsilon = kCommandChangeEpsilon)
{
  return !previous_value.has_value() || std::fabs(*previous_value - next_value) > epsilon;
}

bool parse_joint_interface_name(
  const std::string & interface_name, std::string & joint_name, std::string & command_interface)
{
  const auto separator = interface_name.find('/');
  if (
    separator == std::string::npos || separator == 0 ||
    separator + 1 >= interface_name.size())
  {
    return false;
  }

  joint_name = interface_name.substr(0, separator);
  command_interface = interface_name.substr(separator + 1);
  return true;
}

class ScopedMaintenanceFlag
{
public:
  explicit ScopedMaintenanceFlag(std::atomic<bool> & flag)
  : flag_(flag)
  {
    flag_.store(true);
  }

  ~ScopedMaintenanceFlag()
  {
    flag_.store(false);
  }

  ScopedMaintenanceFlag(const ScopedMaintenanceFlag &) = delete;
  ScopedMaintenanceFlag & operator=(const ScopedMaintenanceFlag &) = delete;

private:
  std::atomic<bool> & flag_;
};

}  // namespace

void MABSystemHardware::CandleDeleter::operator()(mab::Candle * candle) const
{
  mab::detachCandle(candle);
}

hardware_interface::CallbackReturn MABSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  const auto base_result = hardware_interface::SystemInterface::on_init(params);
  if (base_result != CallbackReturn::SUCCESS) {
    return base_result;
  }

  try {
    const auto hardware_result = parse_hardware_parameters();
    if (hardware_result != CallbackReturn::SUCCESS) {
      return hardware_result;
    }

    const auto joint_result = parse_joint_parameters();
    if (joint_result != CallbackReturn::SUCCESS) {
      return joint_result;
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Failed to parse hardware configuration: %s", error.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_logger(), "Configured MAB hardware with %zu joints on %s at %s (fast_mode=%s)",
    joints_.size(), bus_.c_str(), data_rate_.c_str(), fast_mode_ ? "true" : "false");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MABSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  setup_maintenance_interfaces();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MABSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!connect_candle()) {
    return CallbackReturn::ERROR;
  }

  if (!setup_pds()) {
    disconnect_candle();
    return CallbackReturn::ERROR;
  }

  size_t connected_drives = 0;
  for (auto & joint : joints_) {
    if (joint.can_id == 0) {
      joint.connected = false;
      joint.md.reset();
      RCLCPP_INFO(
        get_logger(), "Skipping drive activation for %s because can_id is set to 0.",
        joint.name.c_str());
      continue;
    }

    if (activate_drive(joint)) {
      ++connected_drives;
      continue;
    }

    joint.connected = false;
    joint.md.reset();
    if (!allow_no_connected_drives_) {
      disable_all_drives();
      disable_power_stage();
      disconnect_candle();
      return CallbackReturn::ERROR;
    }
  }

  if (connected_drives == 0 && !allow_no_connected_drives_) {
    RCLCPP_ERROR(get_logger(), "No MD drives could be activated.");
    disable_power_stage();
    disconnect_candle();
    return CallbackReturn::ERROR;
  }

  if (read(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.0)) !=
      hardware_interface::return_type::OK)
  {
    RCLCPP_WARN(get_logger(), "Initial hardware read failed during activation.");
  }

  if (seed_joint_commands_from_state() != hardware_interface::return_type::OK) {
    RCLCPP_WARN(get_logger(), "Failed to seed joint commands from the current state.");
  }

  if (hold_position_on_activate_ms_ > 0) {
    command_hold_until_ =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(hold_position_on_activate_ms_);
    command_hold_notice_emitted_ = false;
    RCLCPP_INFO(
      get_logger(),
      "Holding current joint positions for %d ms after activation to avoid startup transients.",
      hold_position_on_activate_ms_);
  } else {
    command_hold_until_.reset();
    command_hold_notice_emitted_ = false;
  }

  RCLCPP_INFO(
    get_logger(), "Activated MAB hardware. Connected drives: %zu/%zu",
    connected_drives, joints_.size());
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MABSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  disable_all_drives();
  if (disable_power_stage_on_deactivate_) {
    disable_power_stage();
  }
  command_hold_until_.reset();
  command_hold_notice_emitted_ = false;
  maintenance_active_ = false;
  disconnect_candle();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MABSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  disable_all_drives();
  disable_power_stage();
  command_hold_until_.reset();
  command_hold_notice_emitted_ = false;
  maintenance_active_ = false;
  reset_maintenance_interfaces();
  disconnect_candle();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MABSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  disable_all_drives();
  disable_power_stage();
  command_hold_until_.reset();
  command_hold_notice_emitted_ = false;
  maintenance_active_ = false;
  reset_maintenance_interfaces();
  disconnect_candle();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MABSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (maintenance_active_.load()) {
    return hardware_interface::return_type::OK;
  }

  const bool is_telemetry_cycle =
    (read_cycle_counter_ % static_cast<size_t>(std::max(1, telemetry_divider_)) == 0);

  for (auto & joint : joints_) {
    if (!joint.connected || !joint.md) {
      continue;
    }

    // --- Core state read (pos, vel, torque) with retries ---
    bool read_ok = false;

    for (int attempt = 1; attempt <= kJointReadRetryAttempts; ++attempt) {
      const auto read_result = (joint.has_effort_state && !fast_mode_) ?
        joint.md->readRegisters(
        joint.md->m_mdRegisters.mainEncoderPosition,
        joint.md->m_mdRegisters.mainEncoderVelocity,
        joint.md->m_mdRegisters.motorTorque) :
        joint.md->readRegisters(
        joint.md->m_mdRegisters.mainEncoderPosition,
        joint.md->m_mdRegisters.mainEncoderVelocity);

      if (read_result == mab::MD::Error_t::OK) {
        read_ok = true;
        joint.consecutive_read_failures = 0;
        break;
      }
    }

    if (!read_ok) {
      ++joint.consecutive_read_failures;
      // Never deactivate hardware on read failures — hold the last valid state.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Read failed for drive %u (%s) after %d retries; holding last state "
        "(%zu consecutive failures).",
        joint.can_id, joint.name.c_str(), kJointReadRetryAttempts,
        joint.consecutive_read_failures);
      // State interfaces keep the previously published values — skip update.
      // Fall through to voltage read below (it's independent).
    } else {
      joint.state_position =
        static_cast<double>(joint.md->m_mdRegisters.mainEncoderPosition.value) * joint.gear_ratio;
      joint.state_velocity =
        static_cast<double>(joint.md->m_mdRegisters.mainEncoderVelocity.value) * joint.gear_ratio;
      if (joint.has_effort_state && !fast_mode_) {
        joint.state_effort =
          static_cast<double>(joint.md->m_mdRegisters.motorTorque.value) / joint.gear_ratio;
      }

      if (joint.has_position_state) {
        set_state(joint.name + "/" + hardware_interface::HW_IF_POSITION, joint.state_position);
      }
      if (joint.has_velocity_state) {
        set_state(joint.name + "/" + hardware_interface::HW_IF_VELOCITY, joint.state_velocity);
      }
      if (joint.has_effort_state) {
        set_state(joint.name + "/" + hardware_interface::HW_IF_EFFORT, joint.state_effort);
      }
    }

    // --- Per-joint voltage read on the telemetry divider cycle ---
    if (joint.has_voltage_state && is_telemetry_cycle) {
      if (joint.md->readRegisters(joint.md->m_mdRegisters.dcBusVoltage) ==
        mab::MD::Error_t::OK)
      {
        joint.state_voltage =
          static_cast<double>(joint.md->m_mdRegisters.dcBusVoltage.value);
        set_state(joint.name + "/voltage", joint.state_voltage);
      }
      // On failure the previous voltage sample is retained — no action needed.
    }
  }

  ++read_cycle_counter_;
  if (
    use_pds_ && power_stage_state_.exported && is_telemetry_cycle)
  {
    read_power_stage_telemetry();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MABSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (maintenance_active_.load()) {
    return hardware_interface::return_type::OK;
  }

  const auto now = std::chrono::steady_clock::now();
  bool command_hold_active = false;
  if (command_hold_until_) {
    if (now < *command_hold_until_) {
      command_hold_active = true;
      if (!command_hold_notice_emitted_) {
        RCLCPP_INFO(
          get_logger(),
          "Startup command hold is active; writing present-position hold targets.");
        command_hold_notice_emitted_ = true;
      }
    } else {
      RCLCPP_INFO(get_logger(), "Startup command hold released; accepting controller commands.");
      command_hold_until_.reset();
      command_hold_notice_emitted_ = false;
    }
  }

  for (auto & joint : joints_) {
    if (!joint.connected || !joint.md) {
      continue;
    }

    double joint_position_command = joint.last_position_command;
    double joint_velocity_command = joint.last_velocity_command;
    double joint_effort_command = joint.last_effort_command;

    if (!command_hold_active) {
      if (joint.has_position_command) {
        joint_position_command =
          get_command<double>(joint.name + "/" + hardware_interface::HW_IF_POSITION);
      }
      if (joint.has_velocity_command) {
        joint_velocity_command =
          get_command<double>(joint.name + "/" + hardware_interface::HW_IF_VELOCITY);
      }
      if (joint.has_effort_command) {
        joint_effort_command =
          get_command<double>(joint.name + "/" + hardware_interface::HW_IF_EFFORT);
      }
    } else {
      // During the hold window, do not touch drive targets; just keep the last seeded values.
      continue;
    }

    if (!std::isfinite(joint_position_command)) {
      joint_position_command = joint.state_position;
    }
    if (!std::isfinite(joint_velocity_command)) {
      joint_velocity_command = 0.0;
    }
    if (!std::isfinite(joint_effort_command)) {
      joint_effort_command = 0.0;
    }

    joint_position_command = std::clamp(
      joint_position_command, joint.limit_position_min, joint.limit_position_max);
    joint_velocity_command = clamp_symmetric(joint_velocity_command, joint.limit_max_velocity);
    joint_effort_command = clamp_symmetric(joint_effort_command, joint.limit_max_torque);

    joint.last_position_command = joint_position_command;
    joint.last_velocity_command = joint_velocity_command;
    joint.last_effort_command = joint_effort_command;

    const double motor_position_command = joint_position_command / joint.gear_ratio;
    const double motor_velocity_command = joint_velocity_command / joint.gear_ratio;
    const double motor_effort_command = joint_effort_command * joint.gear_ratio;
    const auto motor_position_command_f = static_cast<float>(motor_position_command);
    const auto motor_velocity_command_f = static_cast<float>(motor_velocity_command);
    const auto motor_effort_command_f = static_cast<float>(motor_effort_command);

    if (joint.active_command_mode == JointData::CommandMode::POSITION) {
      const bool position_changed =
        has_meaningful_change(joint.last_sent_motor_position_command, motor_position_command_f);
      const bool velocity_changed =
        has_meaningful_change(joint.last_sent_motor_velocity_command, motor_velocity_command_f);

      if (position_changed || velocity_changed) {
        joint.md->m_mdRegisters.targetPosition = motor_position_command_f;
        joint.md->m_mdRegisters.targetVelocity = motor_velocity_command_f;

        bool write_ok = false;
        for (int attempt = 1; attempt <= kJointWriteRetryAttempts; ++attempt) {
          if (joint.md->writeRegisters(
              joint.md->m_mdRegisters.targetPosition,
              joint.md->m_mdRegisters.targetVelocity) == mab::MD::Error_t::OK)
          {
            write_ok = true;
            joint.consecutive_write_failures = 0;
            break;
          }
        }
        if (!write_ok) {
          ++joint.consecutive_write_failures;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Write failed for drive %u (%s) target position/velocity after %d retries "
            "(%zu consecutive failures); holding last command.",
            joint.can_id, joint.name.c_str(), kJointWriteRetryAttempts,
            joint.consecutive_write_failures);
        } else {
          joint.last_sent_motor_position_command = motor_position_command_f;
          joint.last_sent_motor_velocity_command = motor_velocity_command_f;
        }
      }
    } else if (joint.active_command_mode == JointData::CommandMode::VELOCITY) {
      if (has_meaningful_change(joint.last_sent_motor_velocity_command, motor_velocity_command_f)) {
        joint.md->m_mdRegisters.targetVelocity = motor_velocity_command_f;

        bool write_ok = false;
        for (int attempt = 1; attempt <= kJointWriteRetryAttempts; ++attempt) {
          if (joint.md->writeRegisters(joint.md->m_mdRegisters.targetVelocity) ==
            mab::MD::Error_t::OK)
          {
            write_ok = true;
            joint.consecutive_write_failures = 0;
            break;
          }
        }
        if (!write_ok) {
          ++joint.consecutive_write_failures;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Write failed for drive %u (%s) target velocity after %d retries "
            "(%zu consecutive failures); holding last command.",
            joint.can_id, joint.name.c_str(), kJointWriteRetryAttempts,
            joint.consecutive_write_failures);
        } else {
          joint.last_sent_motor_velocity_command = motor_velocity_command_f;
        }
      }
    } else {
      // IMPEDANCE mode: write torque target (feedforward torque)
      if (has_meaningful_change(joint.last_sent_motor_effort_command, motor_effort_command_f)) {
        joint.md->m_mdRegisters.targetTorque = motor_effort_command_f;

        bool write_ok = false;
        for (int attempt = 1; attempt <= kJointWriteRetryAttempts; ++attempt) {
          if (joint.md->writeRegisters(joint.md->m_mdRegisters.targetTorque) ==
            mab::MD::Error_t::OK)
          {
            write_ok = true;
            joint.consecutive_write_failures = 0;
            break;
          }
        }
        if (!write_ok) {
          ++joint.consecutive_write_failures;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Write failed for drive %u (%s) target torque after %d retries "
            "(%zu consecutive failures); holding last command.",
            joint.can_id, joint.name.c_str(), kJointWriteRetryAttempts,
            joint.consecutive_write_failures);
        } else {
          joint.last_sent_motor_effort_command = motor_effort_command_f;
        }
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MABSystemHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  auto validate_interface = [this](const std::string & interface_name) -> bool {
      std::string joint_name;
      std::string command_interface;
      if (!parse_joint_interface_name(interface_name, joint_name, command_interface)) {
        return true;
      }

      if (
        command_interface != hardware_interface::HW_IF_POSITION &&
        command_interface != hardware_interface::HW_IF_VELOCITY &&
        command_interface != hardware_interface::HW_IF_EFFORT)
      {
        return true;
      }

      if (!find_joint_by_name(joint_name)) {
        RCLCPP_ERROR(
          get_logger(), "Command mode switch references unknown joint '%s' in interface '%s'",
          joint_name.c_str(), interface_name.c_str());
        return false;
      }

      return true;
    };

  const auto is_valid_start = std::all_of(
    start_interfaces.begin(), start_interfaces.end(),
    [&validate_interface](const auto & interface_name) {
      return validate_interface(interface_name);
    });
  if (!is_valid_start) {
    return hardware_interface::return_type::ERROR;
  }

  const auto is_valid_stop = std::all_of(
    stop_interfaces.begin(), stop_interfaces.end(),
    [&validate_interface](const auto & interface_name) {
      return validate_interface(interface_name);
    });
  if (!is_valid_stop) {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MABSystemHardware::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  struct ModeRequest
  {
    bool position = false;
    bool velocity = false;
    bool effort = false;
  };

  std::vector<ModeRequest> mode_requests(joints_.size());

  auto find_joint_index = [this](const std::string & joint_name) -> std::optional<size_t> {
      for (size_t index = 0; index < joints_.size(); ++index) {
        if (joints_[index].name == joint_name) {
          return index;
        }
      }
      return std::nullopt;
    };

  for (const auto & interface_name : start_interfaces) {
    std::string joint_name;
    std::string command_interface;
    if (!parse_joint_interface_name(interface_name, joint_name, command_interface)) {
      continue;
    }

    if (
      command_interface != hardware_interface::HW_IF_POSITION &&
      command_interface != hardware_interface::HW_IF_VELOCITY &&
      command_interface != hardware_interface::HW_IF_EFFORT)
    {
      continue;
    }

    const auto joint_index = find_joint_index(joint_name);
    if (!joint_index) {
      continue;
    }

    auto & request = mode_requests[*joint_index];
    if (command_interface == hardware_interface::HW_IF_POSITION) {
      request.position = true;
    } else if (command_interface == hardware_interface::HW_IF_VELOCITY) {
      request.velocity = true;
    } else {
      request.effort = true;
    }
  }

  for (size_t index = 0; index < joints_.size(); ++index) {
    const auto & request = mode_requests[index];
    if (!request.position && !request.velocity && !request.effort) {
      continue;
    }

    auto & joint = joints_[index];
    JointData::CommandMode requested_mode;
    if (request.effort && !request.position && !request.velocity) {
      requested_mode = JointData::CommandMode::IMPEDANCE;
    } else if (request.velocity && !request.position && !request.effort) {
      requested_mode = JointData::CommandMode::VELOCITY;
    } else {
      requested_mode = JointData::CommandMode::POSITION;
    }
    if (joint.active_command_mode == requested_mode) {
      continue;
    }

    if (!joint.connected || !joint.md) {
      joint.active_command_mode = requested_mode;
      RCLCPP_WARN(
        get_logger(),
        "Skipping motion-mode switch for disconnected joint %s (requested: %s)",
        joint.name.c_str(),
        requested_mode == JointData::CommandMode::IMPEDANCE ? "IMPEDANCE" :
        requested_mode == JointData::CommandMode::VELOCITY ? "VELOCITY_PID" : "POSITION_PID");
      continue;
    }

    if (requested_mode == JointData::CommandMode::POSITION) {
      if (joint.md->setMotionMode(mab::MdMode_E::POSITION_PID) != mab::MD::Error_t::OK) {
        RCLCPP_ERROR(
          get_logger(), "Failed to switch %s (%u) to POSITION_PID",
          joint.name.c_str(), joint.can_id);
        return hardware_interface::return_type::ERROR;
      }

      const double hold_position = std::clamp(
        joint.state_position, joint.limit_position_min, joint.limit_position_max);
      joint.md->m_mdRegisters.targetPosition = static_cast<float>(hold_position / joint.gear_ratio);
      joint.md->m_mdRegisters.targetVelocity = 0.0F;

      if (
        joint.md->writeRegisters(
          joint.md->m_mdRegisters.targetPosition,
          joint.md->m_mdRegisters.targetVelocity) != mab::MD::Error_t::OK)
      {
        RCLCPP_ERROR(
          get_logger(), "Failed to preload target position/velocity for %s (%u)",
          joint.name.c_str(), joint.can_id);
        return hardware_interface::return_type::ERROR;
      }
    } else if (requested_mode == JointData::CommandMode::VELOCITY) {
      if (joint.md->setMotionMode(mab::MdMode_E::VELOCITY_PID) != mab::MD::Error_t::OK) {
        RCLCPP_ERROR(
          get_logger(), "Failed to switch %s (%u) to VELOCITY_PID",
          joint.name.c_str(), joint.can_id);
        return hardware_interface::return_type::ERROR;
      }

      joint.md->m_mdRegisters.targetVelocity = 0.0F;
      if (joint.md->writeRegisters(joint.md->m_mdRegisters.targetVelocity) != mab::MD::Error_t::OK)
      {
        RCLCPP_ERROR(
          get_logger(), "Failed to preload target velocity for %s (%u)",
          joint.name.c_str(), joint.can_id);
        return hardware_interface::return_type::ERROR;
      }
    } else {
      // IMPEDANCE mode: set kp=0, kd=0 initially (pure torque), then switch to IMPEDANCE
      if (joint.md->setImpedanceParams(
            static_cast<float>(joint.impedance_kp),
            static_cast<float>(joint.impedance_kd)) != mab::MD::Error_t::OK)
      {
        RCLCPP_ERROR(
          get_logger(), "Failed to set impedance params for %s (%u)",
          joint.name.c_str(), joint.can_id);
        return hardware_interface::return_type::ERROR;
      }

      if (joint.md->setMotionMode(mab::MdMode_E::IMPEDANCE) != mab::MD::Error_t::OK) {
        RCLCPP_ERROR(
          get_logger(), "Failed to switch %s (%u) to IMPEDANCE",
          joint.name.c_str(), joint.can_id);
        return hardware_interface::return_type::ERROR;
      }

      joint.md->m_mdRegisters.targetTorque = 0.0F;
      if (joint.md->writeRegisters(joint.md->m_mdRegisters.targetTorque) != mab::MD::Error_t::OK)
      {
        RCLCPP_ERROR(
          get_logger(), "Failed to preload target torque for %s (%u)",
          joint.name.c_str(), joint.can_id);
        return hardware_interface::return_type::ERROR;
      }
    }

    joint.active_command_mode = requested_mode;
    joint.last_position_command = std::clamp(
      joint.state_position, joint.limit_position_min, joint.limit_position_max);
    joint.last_velocity_command = 0.0;
    joint.last_effort_command = 0.0;
    if (requested_mode == JointData::CommandMode::POSITION) {
      joint.last_sent_motor_position_command = joint.md->m_mdRegisters.targetPosition.value;
      joint.last_sent_motor_velocity_command = joint.md->m_mdRegisters.targetVelocity.value;
      joint.last_sent_motor_effort_command.reset();
    } else if (requested_mode == JointData::CommandMode::VELOCITY) {
      joint.last_sent_motor_position_command.reset();
      joint.last_sent_motor_velocity_command = joint.md->m_mdRegisters.targetVelocity.value;
      joint.last_sent_motor_effort_command.reset();
    } else {
      joint.last_sent_motor_position_command.reset();
      joint.last_sent_motor_velocity_command.reset();
      joint.last_sent_motor_effort_command = joint.md->m_mdRegisters.targetTorque.value;
    }
    set_command(joint.name + "/" + hardware_interface::HW_IF_POSITION, joint.last_position_command);
    set_command(joint.name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
    if (joint.has_effort_command) {
      set_command(joint.name + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
    }

    RCLCPP_INFO(
      get_logger(), "Joint %s command mode switched to %s",
      joint.name.c_str(),
      requested_mode == JointData::CommandMode::IMPEDANCE ? "IMPEDANCE" :
      requested_mode == JointData::CommandMode::VELOCITY ? "VELOCITY_PID" : "POSITION_PID");
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn MABSystemHardware::parse_hardware_parameters()
{
  const auto & parameters = info_.hardware_parameters;

  if (const auto it = parameters.find("bus"); it != parameters.end()) {
    bus_ = it->second;
  }
  if (const auto it = parameters.find("data_rate"); it != parameters.end()) {
    data_rate_ = it->second;
  }
  if (const auto it = parameters.find("use_pds"); it != parameters.end()) {
    use_pds_ = parse_bool(it->second, true);
  }
  if (const auto it = parameters.find("use_regular_can_frames"); it != parameters.end()) {
    use_regular_can_frames_ = parse_bool(it->second, true);
  }
  if (const auto it = parameters.find("pds_id"); it != parameters.end()) {
    pds_id_ = std::stoi(it->second);
  }
  if (const auto it = parameters.find("power_stage_socket"); it != parameters.end()) {
    power_stage_socket_ = std::stoi(it->second);
  }
  if (const auto it = parameters.find("fast_mode"); it != parameters.end()) {
    fast_mode_ = parse_bool(it->second, false);
  }
  if (const auto it = parameters.find("telemetry_divider"); it != parameters.end()) {
    telemetry_divider_ = std::max(1, std::stoi(it->second));
  }
  if (const auto it = parameters.find("auto_enable_power_stage"); it != parameters.end()) {
    auto_enable_power_stage_ = parse_bool(it->second, true);
  }
  if (const auto it = parameters.find("disable_power_stage_on_deactivate"); it != parameters.end()) {
    disable_power_stage_on_deactivate_ = parse_bool(it->second, true);
  }
  if (const auto it = parameters.find("zero_on_activate"); it != parameters.end()) {
    zero_on_activate_ = parse_bool(it->second, false);
  }
  if (const auto it = parameters.find("allow_no_connected_drives"); it != parameters.end()) {
    allow_no_connected_drives_ = parse_bool(it->second, true);
  }
  if (const auto it = parameters.find("save_md_configuration_to_flash"); it != parameters.end()) {
    save_md_configuration_to_flash_ = parse_bool(it->second, false);
  }
  if (const auto it = parameters.find("md_can_watchdog_ms"); it != parameters.end()) {
    md_can_watchdog_ms_ = parse_double(it->second);
  }
  if (const auto it = parameters.find("hold_position_on_activate_ms"); it != parameters.end()) {
    hold_position_on_activate_ms_ = std::max(0, std::stoi(it->second));
  }
  if (const auto it = parameters.find("maintenance_reload_enabled"); it != parameters.end()) {
    maintenance_reload_enabled_ = parse_bool(it->second, true);
  }
  if (const auto it = parameters.find("maintenance_restore_controller"); it != parameters.end()) {
    maintenance_restore_controller_ = it->second;
  }
  if (const auto it = parameters.find("maintenance_service_timeout_ms"); it != parameters.end()) {
    maintenance_service_timeout_ms_ = std::max(100, std::stoi(it->second));
  }

  power_stage_state_.exported = !info_.sensors.empty();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MABSystemHardware::parse_joint_parameters()
{
  joints_.clear();
  joints_.reserve(info_.joints.size());

  for (const auto & joint_info : info_.joints) {
    JointData joint;
    joint.name = joint_info.name;
    joint.motor_name = get_required_param(joint_info.parameters, "motor_name", joint.name);
    joint.can_id = static_cast<mab::canId_t>(std::stoul(
      get_required_param(joint_info.parameters, "can_id", joint.name)));
    joint.gear_ratio = parse_double(
      get_required_param(joint_info.parameters, "gear_ratio", joint.name));
    if (!std::isfinite(joint.gear_ratio) || joint.gear_ratio <= 0.0) {
      throw std::runtime_error("Invalid gear_ratio for " + joint.name);
    }

    joint.motor_pole_pairs = parse_double(
      get_required_param(joint_info.parameters, "pole_pairs", joint.name));
    joint.motor_kv = parse_double(
      get_required_param(joint_info.parameters, "kv", joint.name));
    joint.torque_constant = parse_double(
      get_required_param(joint_info.parameters, "torque_constant", joint.name));
    joint.max_current = parse_double(
      get_required_param(joint_info.parameters, "max_current", joint.name));
    joint.torque_bandwidth_hz = parse_double(
      get_required_param(joint_info.parameters, "torque_bandwidth_hz", joint.name));
    joint.friction = parse_double(
      get_required_param(joint_info.parameters, "friction", joint.name));
    joint.stiction = parse_double(
      get_required_param(joint_info.parameters, "stiction", joint.name));
    joint.motor_calibration_mode = parse_double(
      get_required_param(joint_info.parameters, "motor_calibration_mode", joint.name));
    joint.shutdown_temp = parse_double(
      get_required_param(joint_info.parameters, "shutdown_temp", joint.name));

    joint.limit_max_torque = parse_double(
      get_required_param(joint_info.parameters, "limit_max_torque", joint.name));
    joint.limit_max_velocity = parse_double(
      get_required_param(joint_info.parameters, "limit_max_velocity", joint.name));
    joint.limit_position_min = parse_double(
      get_required_param(joint_info.parameters, "limit_position_min", joint.name));
    joint.limit_position_max = parse_double(
      get_required_param(joint_info.parameters, "limit_position_max", joint.name));
    joint.limit_max_acceleration = parse_double(
      get_required_param(joint_info.parameters, "limit_max_acceleration", joint.name));
    joint.limit_max_deceleration = parse_double(
      get_required_param(joint_info.parameters, "limit_max_deceleration", joint.name));

    joint.profile_velocity = parse_double(
      get_required_param(joint_info.parameters, "profile_velocity", joint.name));
    joint.profile_acceleration = parse_double(
      get_required_param(joint_info.parameters, "profile_acceleration", joint.name));
    joint.profile_deceleration = parse_double(
      get_required_param(joint_info.parameters, "profile_deceleration", joint.name));
    joint.profile_quick_stop_deceleration = parse_double(
      get_required_param(joint_info.parameters, "profile_quick_stop_deceleration", joint.name));

    joint.output_encoder = parse_double(
      get_required_param(joint_info.parameters, "output_encoder", joint.name));
    joint.output_encoder_default_baud = parse_double(
      get_required_param(joint_info.parameters, "output_encoder_default_baud", joint.name));
    joint.output_encoder_mode = parse_double(
      get_required_param(joint_info.parameters, "output_encoder_mode", joint.name));
    joint.output_encoder_calibration_mode = parse_double(
      get_required_param(joint_info.parameters, "output_encoder_calibration_mode", joint.name));

    joint.position_pid_kp = parse_double(
      get_required_param(joint_info.parameters, "position_pid_kp", joint.name));
    joint.position_pid_ki = parse_double(
      get_required_param(joint_info.parameters, "position_pid_ki", joint.name));
    joint.position_pid_kd = parse_double(
      get_required_param(joint_info.parameters, "position_pid_kd", joint.name));
    joint.position_pid_windup = parse_double(
      get_required_param(joint_info.parameters, "position_pid_windup", joint.name));

    joint.velocity_pid_kp = parse_double(
      get_required_param(joint_info.parameters, "velocity_pid_kp", joint.name));
    joint.velocity_pid_ki = parse_double(
      get_required_param(joint_info.parameters, "velocity_pid_ki", joint.name));
    joint.velocity_pid_kd = parse_double(
      get_required_param(joint_info.parameters, "velocity_pid_kd", joint.name));
    joint.velocity_pid_windup = parse_double(
      get_required_param(joint_info.parameters, "velocity_pid_windup", joint.name));

    joint.impedance_kp = parse_double(
      get_required_param(joint_info.parameters, "impedance_kp", joint.name));
    joint.impedance_kd = parse_double(
      get_required_param(joint_info.parameters, "impedance_kd", joint.name));
    joint.user_gpio_configuration = parse_double(
      get_required_param(joint_info.parameters, "user_gpio_configuration", joint.name));
    joint.reverse_direction = parse_double(
      get_required_param(joint_info.parameters, "reverse_direction", joint.name));
    joint.shunt_resistance = parse_double(
      get_required_param(joint_info.parameters, "shunt_resistance", joint.name));

    joint.has_position_command = has_interface(
      joint_info.command_interfaces, hardware_interface::HW_IF_POSITION);
    joint.has_velocity_command = has_interface(
      joint_info.command_interfaces, hardware_interface::HW_IF_VELOCITY);
    joint.has_effort_command = has_interface(
      joint_info.command_interfaces, hardware_interface::HW_IF_EFFORT);
    joint.has_position_state = has_interface(
      joint_info.state_interfaces, hardware_interface::HW_IF_POSITION);
    joint.has_velocity_state = has_interface(
      joint_info.state_interfaces, hardware_interface::HW_IF_VELOCITY);
    joint.has_effort_state = has_interface(
      joint_info.state_interfaces, hardware_interface::HW_IF_EFFORT);
    joint.has_voltage_state = has_interface(joint_info.state_interfaces, "voltage");

    if (!joint.has_position_command || !joint.has_velocity_command) {
      throw std::runtime_error(
        "Joint " + joint.name + " must export position and velocity command interfaces");
    }
    if (!joint.has_position_state || !joint.has_velocity_state || !joint.has_effort_state) {
      throw std::runtime_error(
        "Joint " + joint.name + " must export position, velocity and effort state interfaces");
    }

    joints_.push_back(std::move(joint));
  }

  if (joints_.empty()) {
    RCLCPP_ERROR(get_logger(), "No joints were defined for the MAB hardware.");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

bool MABSystemHardware::connect_candle()
{
  if (candle_) {
    return true;
  }

  const auto data_rate = parse_data_rate(data_rate_);
  if (!data_rate) {
    RCLCPP_ERROR(get_logger(), "Unsupported CAN data rate: %s", data_rate_.c_str());
    return false;
  }

  const auto bus_type = parse_bus_type(bus_);
  if (!bus_type) {
    RCLCPP_ERROR(get_logger(), "Unsupported CANdle bus type: %s", bus_.c_str());
    return false;
  }

  try {
    std::unique_ptr<mab::I_CommunicationInterface> bus_handle;
    switch (*bus_type) {
      case mab::candleTypes::USB:
        bus_handle = std::make_unique<mab::USB>(mab::Candle::CANDLE_VID, mab::Candle::CANDLE_PID);
        break;
      case mab::candleTypes::SPI:
        bus_handle = std::make_unique<mab::SPI>();
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Unhandled bus type.");
        return false;
    }

    if (bus_handle->connect() != mab::I_CommunicationInterface::Error_t::OK) {
      RCLCPP_ERROR(get_logger(), "Failed to connect to the CANdle transport.");
      return false;
    }

    CandlePtr candle(new mab::Candle(*data_rate, std::move(bus_handle), use_regular_can_frames_));
    if (candle->init() != mab::candleTypes::Error_t::OK) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize the CANdle device.");
      return false;
    }
    candle_ = std::move(candle);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "CANdle connection failed: %s", error.what());
    return false;
  }

  return true;
}

void MABSystemHardware::disconnect_candle()
{
  power_stage_.reset();
  pds_.reset();
  for (auto & joint : joints_) {
    joint.connected = false;
    joint.md.reset();
  }
  candle_.reset();
}

bool MABSystemHardware::setup_pds()
{
  if (!use_pds_) {
    return true;
  }

  const auto socket = parse_socket_index(power_stage_socket_);
  if (!socket) {
    RCLCPP_ERROR(get_logger(), "Invalid power_stage_socket value: %d", power_stage_socket_);
    return false;
  }

  bool module_probe_succeeded = false;
  bool pds_liveness_confirmed = false;
  u32 bus_voltage_mv = 0;

  for (int attempt = 1; attempt <= kPdsInitAttempts; ++attempt) {
    pds_ = std::make_unique<mab::Pds>(static_cast<u16>(pds_id_), candle_.get());

    if (pds_->init() == mab::PdsModule::error_E::OK) {
      module_probe_succeeded = true;
      break;
    }

    if (pds_->getBusVoltage(bus_voltage_mv) == mab::PdsModule::error_E::OK) {
      pds_liveness_confirmed = true;
      RCLCPP_WARN(
        get_logger(),
        "PDS %d answered a bus-voltage request (%.2f V), but module discovery failed on attempt %d/%d.",
        pds_id_, static_cast<double>(bus_voltage_mv) / 1000.0, attempt, kPdsInitAttempts);
      break;
    }

    RCLCPP_WARN(
      get_logger(), "Failed to initialize PDS %d on attempt %d/%d",
      pds_id_, attempt, kPdsInitAttempts);
    std::this_thread::sleep_for(kPdsRetryDelay);
  }

  if (module_probe_succeeded) {
    power_stage_ = pds_->attachPowerStage(*socket);
    if (!power_stage_) {
      RCLCPP_ERROR(
        get_logger(), "Failed to attach power stage on socket %d", power_stage_socket_);
      return false;
    }
  } else if (pds_liveness_confirmed) {
    auto pds_can_id = std::make_shared<u16>(static_cast<u16>(pds_id_));
    power_stage_ = std::make_shared<mab::PowerStage>(*socket, candle_.get(), pds_can_id);
    RCLCPP_WARN(
      get_logger(),
      "Proceeding with direct power-stage attachment on socket %d because PDS module discovery is unreliable.",
      power_stage_socket_);
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to initialize PDS %d", pds_id_);
    return false;
  }

  if (auto_enable_power_stage_) {
    if (power_stage_->enable() != mab::PdsModule::error_E::OK) {
      RCLCPP_ERROR(
        get_logger(), "Failed to enable the power stage on socket %d", power_stage_socket_);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }

  return true;
}

bool MABSystemHardware::activate_drive(JointData & joint)
{
  joint.md = std::make_unique<mab::MD>(joint.can_id, candle_.get());
  if (joint.md->init() != mab::MD::Error_t::OK) {
    RCLCPP_WARN(
      get_logger(), "Failed to initialize drive %u for %s", joint.can_id, joint.name.c_str());
    return false;
  }

  if (joint.md->clearErrors() != mab::MD::Error_t::OK) {
    RCLCPP_WARN(
      get_logger(), "Failed to clear latched errors for drive %u", joint.can_id);
  }

  if (!configure_drive(joint)) {
    RCLCPP_WARN(
      get_logger(), "Failed to configure drive %u for %s", joint.can_id, joint.name.c_str());
    return false;
  }

  const auto current_motor_position = joint.md->getPosition();
  if (current_motor_position.second != mab::MD::Error_t::OK) {
    RCLCPP_WARN(
      get_logger(), "Failed to read current position for drive %u", joint.can_id);
    return false;
  }

  joint.md->m_mdRegisters.targetPosition = current_motor_position.first;
  joint.md->m_mdRegisters.targetVelocity = 0.0F;
  if (
    joint.md->writeRegisters(
      joint.md->m_mdRegisters.targetPosition,
      joint.md->m_mdRegisters.targetVelocity) != mab::MD::Error_t::OK)
  {
    RCLCPP_WARN(
      get_logger(), "Failed to preload target position/velocity for drive %u", joint.can_id);
    return false;
  }

  if (joint.md->setMotionMode(mab::MdMode_E::POSITION_PID) != mab::MD::Error_t::OK) {
    RCLCPP_WARN(
      get_logger(), "Failed to set POSITION_PID for drive %u", joint.can_id);
    return false;
  }
  joint.active_command_mode = JointData::CommandMode::POSITION;

  if (zero_on_activate_ && joint.md->zero() != mab::MD::Error_t::OK) {
    RCLCPP_WARN(get_logger(), "Failed to zero drive %u", joint.can_id);
    return false;
  }

  if (joint.md->enable() != mab::MD::Error_t::OK) {
    RCLCPP_WARN(get_logger(), "Failed to enable drive %u", joint.can_id);
    return false;
  }

  joint.connected = true;
  joint.consecutive_read_failures = 0;
  joint.last_sent_motor_position_command = joint.md->m_mdRegisters.targetPosition.value;
  joint.last_sent_motor_velocity_command = joint.md->m_mdRegisters.targetVelocity.value;
  joint.last_sent_motor_effort_command.reset();
  joint.state_position = static_cast<double>(current_motor_position.first) * joint.gear_ratio;
  joint.state_velocity = 0.0;
  joint.last_position_command = joint.state_position;
  joint.last_velocity_command = 0.0;
  return true;
}

bool MABSystemHardware::configure_drive(JointData & joint)
{
  auto & md = *joint.md;
  const auto write_register = [&md](auto & reg) {
    return md.writeRegisters(reg) == mab::MD::Error_t::OK;
  };

  if (md.setCurrentLimit(static_cast<float>(joint.max_current)) != mab::MD::Error_t::OK) {
    return false;
  }

  if (md.setTorqueBandwidth(static_cast<u16>(std::lround(joint.torque_bandwidth_hz))) !=
      mab::MD::Error_t::OK)
  {
    return false;
  }

  if (md.setPositionPIDparam(
        static_cast<float>(joint.position_pid_kp),
        static_cast<float>(joint.position_pid_ki),
        static_cast<float>(joint.position_pid_kd),
        static_cast<float>(joint.position_pid_windup)) != mab::MD::Error_t::OK)
  {
    return false;
  }

  if (md.setVelocityPIDparam(
        static_cast<float>(joint.velocity_pid_kp),
        static_cast<float>(joint.velocity_pid_ki),
        static_cast<float>(joint.velocity_pid_kd),
        static_cast<float>(joint.velocity_pid_windup)) != mab::MD::Error_t::OK)
  {
    return false;
  }

  if (md.setImpedanceParams(
        static_cast<float>(joint.impedance_kp),
        static_cast<float>(joint.impedance_kd)) != mab::MD::Error_t::OK)
  {
    return false;
  }

  md.m_mdRegisters.motorPolePairs = static_cast<u32>(std::lround(joint.motor_pole_pairs));
  md.m_mdRegisters.motorGearRatio = static_cast<float>(joint.gear_ratio);
  md.m_mdRegisters.motorKt = static_cast<float>(joint.torque_constant);
  md.m_mdRegisters.motorIMax = static_cast<float>(joint.max_current);
  md.m_mdRegisters.motorTorqueBandwidth =
    static_cast<u16>(std::lround(joint.torque_bandwidth_hz));
  md.m_mdRegisters.motorFriction = static_cast<float>(joint.friction);
  md.m_mdRegisters.motorStiction = static_cast<float>(joint.stiction);
  md.m_mdRegisters.motorKV = static_cast<u16>(std::lround(joint.motor_kv));
  md.m_mdRegisters.motorCalibrationMode =
    static_cast<u8>(std::lround(joint.motor_calibration_mode));
  if (!write_register(md.m_mdRegisters.motorPolePairs)) { return false; }
  if (!write_register(md.m_mdRegisters.motorGearRatio)) { return false; }
  if (!write_register(md.m_mdRegisters.motorKt)) { return false; }
  if (!write_register(md.m_mdRegisters.motorIMax)) { return false; }
  if (!write_register(md.m_mdRegisters.motorTorqueBandwidth)) { return false; }
  if (!write_register(md.m_mdRegisters.motorFriction)) { return false; }
  if (!write_register(md.m_mdRegisters.motorStiction)) { return false; }
  if (!write_register(md.m_mdRegisters.motorKV)) { return false; }
  if (!write_register(md.m_mdRegisters.motorCalibrationMode)) { return false; }

  md.m_mdRegisters.auxEncoder = static_cast<u8>(std::lround(joint.output_encoder));
  md.m_mdRegisters.auxEncoderDefaultBaud =
    static_cast<u32>(std::llround(joint.output_encoder_default_baud));
  md.m_mdRegisters.auxEncoderMode = static_cast<u8>(std::lround(joint.output_encoder_mode));
  md.m_mdRegisters.auxEncoderCalibrationMode =
    static_cast<u8>(std::lround(joint.output_encoder_calibration_mode));
  if (!write_register(md.m_mdRegisters.auxEncoder)) { return false; }
  if (!write_register(md.m_mdRegisters.auxEncoderDefaultBaud)) { return false; }
  if (!write_register(md.m_mdRegisters.auxEncoderMode)) { return false; }
  if (!write_register(md.m_mdRegisters.auxEncoderCalibrationMode)) { return false; }

  md.m_mdRegisters.positionLimitMin = static_cast<float>(joint.limit_position_min);
  md.m_mdRegisters.positionLimitMax = static_cast<float>(joint.limit_position_max);
  md.m_mdRegisters.maxTorque = static_cast<float>(joint.limit_max_torque);
  md.m_mdRegisters.maxVelocity = static_cast<float>(joint.limit_max_velocity);
  md.m_mdRegisters.maxAcceleration = static_cast<float>(joint.limit_max_acceleration);
  md.m_mdRegisters.maxDeceleration = static_cast<float>(joint.limit_max_deceleration);
  if (!write_register(md.m_mdRegisters.positionLimitMin)) { return false; }
  if (!write_register(md.m_mdRegisters.positionLimitMax)) { return false; }
  if (!write_register(md.m_mdRegisters.maxTorque)) { return false; }
  if (!write_register(md.m_mdRegisters.maxVelocity)) { return false; }
  if (!write_register(md.m_mdRegisters.maxAcceleration)) { return false; }
  if (!write_register(md.m_mdRegisters.maxDeceleration)) { return false; }

  md.m_mdRegisters.profileVelocity = static_cast<float>(joint.profile_velocity);
  md.m_mdRegisters.profileAcceleration = static_cast<float>(joint.profile_acceleration);
  md.m_mdRegisters.profileDeceleration = static_cast<float>(joint.profile_deceleration);
  md.m_mdRegisters.quickStopDeceleration =
    static_cast<float>(joint.profile_quick_stop_deceleration);
  if (!write_register(md.m_mdRegisters.profileVelocity)) { return false; }
  if (!write_register(md.m_mdRegisters.profileAcceleration)) { return false; }
  if (!write_register(md.m_mdRegisters.profileDeceleration)) { return false; }
  if (!write_register(md.m_mdRegisters.quickStopDeceleration)) { return false; }

  md.m_mdRegisters.userGpioConfiguration =
    static_cast<u8>(std::lround(joint.user_gpio_configuration));
  md.m_mdRegisters.reverseDirection = static_cast<u8>(std::lround(joint.reverse_direction));
  md.m_mdRegisters.shuntResistance = static_cast<float>(joint.shunt_resistance);
  md.m_mdRegisters.motorShutdownTemp = static_cast<u8>(std::lround(joint.shutdown_temp));
  if (!write_register(md.m_mdRegisters.userGpioConfiguration)) { return false; }
  if (!write_register(md.m_mdRegisters.reverseDirection)) { return false; }
  if (!write_register(md.m_mdRegisters.shuntResistance)) { return false; }
  if (!write_register(md.m_mdRegisters.motorShutdownTemp)) { return false; }

  if (std::isfinite(md_can_watchdog_ms_)) {
    md.m_mdRegisters.canWatchdog = static_cast<u16>(std::lround(md_can_watchdog_ms_));
    if (md.writeRegisters(md.m_mdRegisters.canWatchdog) != mab::MD::Error_t::OK) {
      return false;
    }
  }

  if (save_md_configuration_to_flash_ && md.save() != mab::MD::Error_t::OK) {
    return false;
  }

  return true;
}

void MABSystemHardware::disable_all_drives()
{
  for (auto & joint : joints_) {
    if (joint.md) {
      joint.md->disable();
    }
    joint.connected = false;
    joint.last_sent_motor_position_command.reset();
    joint.last_sent_motor_velocity_command.reset();
    joint.last_sent_motor_effort_command.reset();
  }
}

void MABSystemHardware::disable_power_stage()
{
  if (power_stage_) {
    power_stage_->disable();
  }
}

hardware_interface::return_type MABSystemHardware::read_power_stage_telemetry()
{
  if (!use_pds_ || !pds_ || !power_stage_ || info_.sensors.empty()) {
    return hardware_interface::return_type::OK;
  }

  u32 bus_voltage_mv = 0;

  if (pds_->getBusVoltage(bus_voltage_mv) != mab::PdsModule::error_E::OK) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "PDS telemetry read failed (bus_voltage); continuing without shutting down hardware");
    return hardware_interface::return_type::OK;
  }

  power_stage_state_.bus_voltage = static_cast<double>(bus_voltage_mv);

  const std::string sensor_prefix = info_.sensors.front().name + "/";
  set_state(sensor_prefix + "bus_voltage", power_stage_state_.bus_voltage);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MABSystemHardware::seed_joint_commands_from_state()
{
  for (auto & joint : joints_) {
    if (!joint.has_position_command || !joint.has_velocity_command) {
      continue;
    }

    joint.last_position_command = joint.state_position;
    joint.last_velocity_command = 0.0;
    joint.last_effort_command = 0.0;
    set_command(joint.name + "/" + hardware_interface::HW_IF_POSITION, joint.state_position);
    set_command(joint.name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
    if (joint.has_effort_command) {
      set_command(joint.name + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
    }
  }

  return hardware_interface::return_type::OK;
}

void MABSystemHardware::setup_maintenance_interfaces()
{
  auto node = get_node();
  if (!node) {
    RCLCPP_WARN(get_logger(), "Skipping maintenance service setup because hardware node is unavailable.");
    return;
  }

  if (!maintenance_callback_group_) {
    maintenance_callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  }

  if (!run_calibration_service_) {
    run_calibration_service_ = node->create_service<mab_ros2_control::srv::RunCalibration>(
      "/mab/maintenance/run_calibration",
      std::bind(
        &MABSystemHardware::handle_run_calibration, this, std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS(), maintenance_callback_group_);
  }
  if (!set_torque_bandwidth_service_) {
    set_torque_bandwidth_service_ =
      node->create_service<mab_ros2_control::srv::SetTorqueBandwidth>(
      "/mab/maintenance/set_torque_bandwidth",
      std::bind(
        &MABSystemHardware::handle_set_torque_bandwidth, this, std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS(), maintenance_callback_group_);
  }
  if (!run_drive_tests_service_) {
    run_drive_tests_service_ = node->create_service<mab_ros2_control::srv::RunDriveTests>(
      "/mab/maintenance/run_drive_tests",
      std::bind(
        &MABSystemHardware::handle_run_drive_tests, this, std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS(), maintenance_callback_group_);
  }

  if (!list_controllers_client_) {
    list_controllers_client_ =
      node->create_client<controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers", rclcpp::ServicesQoS(),
      maintenance_callback_group_);
  }
  if (!switch_controller_client_) {
    switch_controller_client_ =
      node->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller", rclcpp::ServicesQoS(),
      maintenance_callback_group_);
  }
}

void MABSystemHardware::reset_maintenance_interfaces()
{
  run_calibration_service_.reset();
  set_torque_bandwidth_service_.reset();
  run_drive_tests_service_.reset();
  maintenance_callback_group_.reset();
  list_controllers_client_.reset();
  switch_controller_client_.reset();
}

MABSystemHardware::JointData * MABSystemHardware::find_joint_by_name(const std::string & joint_name)
{
  auto joint_it = std::find_if(joints_.begin(), joints_.end(), [&joint_name](auto & candidate) {
    return candidate.name == joint_name;
  });
  if (joint_it == joints_.end()) {
    return nullptr;
  }
  return &(*joint_it);
}

bool MABSystemHardware::ensure_controller_clients_ready(std::string & message)
{
  if (!list_controllers_client_ || !switch_controller_client_) {
    message = "Controller management clients are not initialized.";
    return false;
  }

  const auto timeout = std::chrono::milliseconds(maintenance_service_timeout_ms_);
  if (!list_controllers_client_->wait_for_service(timeout)) {
    message = "Timed out waiting for /controller_manager/list_controllers.";
    return false;
  }
  if (!switch_controller_client_->wait_for_service(timeout)) {
    message = "Timed out waiting for /controller_manager/switch_controller.";
    return false;
  }

  return true;
}

bool MABSystemHardware::switch_controllers(
  const std::vector<std::string> & activate_controllers,
  const std::vector<std::string> & deactivate_controllers, std::string & message)
{
  if (activate_controllers.empty() && deactivate_controllers.empty()) {
    return true;
  }

  if (!ensure_controller_clients_ready(message)) {
    return false;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->activate_controllers = activate_controllers;
  request->deactivate_controllers = deactivate_controllers;
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  request->activate_asap = false;
  request->timeout.sec = maintenance_service_timeout_ms_ / 1000;
  request->timeout.nanosec = static_cast<uint32_t>((maintenance_service_timeout_ms_ % 1000) * 1'000'000);

  auto future = switch_controller_client_->async_send_request(request);
  const auto wait_result = future.wait_for(std::chrono::milliseconds(maintenance_service_timeout_ms_));
  if (wait_result != std::future_status::ready) {
    message = "SwitchController request timed out.";
    return false;
  }

  const auto response = future.get();
  if (!response) {
    message = "SwitchController returned an empty response.";
    return false;
  }
  if (!response->ok) {
    message = response->message.empty() ? "SwitchController returned failure." : response->message;
    return false;
  }

  return true;
}

std::optional<std::string> MABSystemHardware::find_active_command_controller(std::string & message)
{
  message.clear();
  if (!ensure_controller_clients_ready(message)) {
    return std::nullopt;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto future = list_controllers_client_->async_send_request(request);
  const auto wait_result = future.wait_for(std::chrono::milliseconds(maintenance_service_timeout_ms_));
  if (wait_result != std::future_status::ready) {
    message = "ListControllers request timed out.";
    return std::nullopt;
  }

  const auto response = future.get();
  if (!response) {
    message = "ListControllers returned an empty response.";
    return std::nullopt;
  }

  std::vector<std::string> active_command_controllers;
  for (const auto & controller : response->controller) {
    if (controller.state != "active") {
      continue;
    }
    const bool claims_command = std::any_of(
      controller.claimed_interfaces.begin(), controller.claimed_interfaces.end(),
      [](const auto & claimed_interface) { return is_command_interface_claimed(claimed_interface); });
    if (claims_command) {
      active_command_controllers.push_back(controller.name);
    }
  }

  if (active_command_controllers.empty()) {
    return std::nullopt;
  }

  if (active_command_controllers.size() > 1) {
    std::ostringstream stream;
    for (size_t index = 0; index < active_command_controllers.size(); ++index) {
      stream << active_command_controllers[index];
      if (index + 1 < active_command_controllers.size()) {
        stream << ", ";
      }
    }
    message =
      "More than one active command controller was detected. Expected one, found: " +
      stream.str();
    return std::nullopt;
  }

  return active_command_controllers.front();
}

std::string MABSystemHardware::resolve_restore_controller(
  const std::optional<std::string> & previous_controller) const
{
  if (uppercase(maintenance_restore_controller_) == "PREVIOUS") {
    return previous_controller.value_or("");
  }
  return maintenance_restore_controller_;
}

bool MABSystemHardware::run_calibration_sequence(JointData & joint, std::string & message)
{
  if (!joint.connected || !joint.md) {
    message = "Requested joint is not connected.";
    return false;
  }

  auto & md = *joint.md;
  if (md.disable() != mab::MD::Error_t::OK) {
    message = "Failed to disable drive before calibration.";
    return false;
  }

  bool sequence_ok = true;
  std::string operation_message;

  md.m_mdRegisters.runCalibrateCmd = 1;
  if (md.writeRegisters(md.m_mdRegisters.runCalibrateCmd) != mab::MD::Error_t::OK) {
    sequence_ok = false;
    operation_message = "Failed to send main calibration command.";
  }

  if (sequence_ok) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    md.m_mdRegisters.runCalibratePiGains = 1;
    if (md.writeRegisters(md.m_mdRegisters.runCalibratePiGains) != mab::MD::Error_t::OK) {
      sequence_ok = false;
      operation_message = "Failed to send PI gain calibration command.";
    }
  }

  if (md.clearErrors() != mab::MD::Error_t::OK) {
    RCLCPP_WARN(get_logger(), "Failed to clear errors after calibration for %s", joint.name.c_str());
  }

  bool restore_ok = true;
  if (md.setMotionMode(mab::MdMode_E::POSITION_PID) != mab::MD::Error_t::OK) {
    restore_ok = false;
    if (operation_message.empty()) {
      operation_message = "Calibration completed, but failed to restore POSITION_PID mode.";
    } else {
      operation_message += " Also failed to restore POSITION_PID mode.";
    }
  } else {
    joint.active_command_mode = JointData::CommandMode::POSITION;
  }
  if (md.enable() != mab::MD::Error_t::OK) {
    restore_ok = false;
    if (operation_message.empty()) {
      operation_message = "Calibration completed, but failed to re-enable drive.";
    } else {
      operation_message += " Also failed to re-enable drive.";
    }
  }

  if (!sequence_ok || !restore_ok) {
    message = operation_message.empty() ? "Calibration failed." : operation_message;
    return false;
  }

  message = "Main and PI calibration commands were sent successfully.";
  return true;
}

bool MABSystemHardware::run_drive_tests_sequence(JointData & joint, std::string & message)
{
  if (!joint.connected || !joint.md) {
    message = "Requested joint is not connected.";
    return false;
  }

  auto & md = *joint.md;
  if (md.disable() != mab::MD::Error_t::OK) {
    message = "Failed to disable drive before running tests.";
    return false;
  }

  bool sequence_ok = true;
  std::string operation_message;

  md.m_mdRegisters.runTestMainEncoderCmd = 1;
  if (md.writeRegisters(md.m_mdRegisters.runTestMainEncoderCmd) != mab::MD::Error_t::OK) {
    sequence_ok = false;
    operation_message = "Failed to trigger main encoder test.";
  }

  if (sequence_ok) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    md.m_mdRegisters.runTestAuxEncoderCmd = 1;
    if (md.writeRegisters(md.m_mdRegisters.runTestAuxEncoderCmd) != mab::MD::Error_t::OK) {
      sequence_ok = false;
      operation_message = "Failed to trigger auxiliary encoder test.";
    }
  }

  if (md.clearErrors() != mab::MD::Error_t::OK) {
    RCLCPP_WARN(get_logger(), "Failed to clear errors after tests for %s", joint.name.c_str());
  }

  bool restore_ok = true;
  if (md.setMotionMode(mab::MdMode_E::POSITION_PID) != mab::MD::Error_t::OK) {
    restore_ok = false;
    if (operation_message.empty()) {
      operation_message = "Drive tests completed, but failed to restore POSITION_PID mode.";
    } else {
      operation_message += " Also failed to restore POSITION_PID mode.";
    }
  } else {
    joint.active_command_mode = JointData::CommandMode::POSITION;
  }
  if (md.enable() != mab::MD::Error_t::OK) {
    restore_ok = false;
    if (operation_message.empty()) {
      operation_message = "Drive tests completed, but failed to re-enable drive.";
    } else {
      operation_message += " Also failed to re-enable drive.";
    }
  }

  if (!sequence_ok || !restore_ok) {
    message = operation_message.empty() ? "Drive test operation failed." : operation_message;
    return false;
  }

  message = "Main and auxiliary encoder test commands were sent successfully.";
  return true;
}

bool MABSystemHardware::perform_with_controller_reload(
  JointData & joint, const std::function<bool(JointData &, std::string &)> & operation,
  std::string & message)
{
  std::optional<std::string> previous_controller;
  if (maintenance_reload_enabled_) {
    std::string active_controller_message;
    previous_controller = find_active_command_controller(active_controller_message);
    if (!active_controller_message.empty()) {
      message = active_controller_message;
      return false;
    }

    if (previous_controller) {
      std::string deactivate_message;
      if (!switch_controllers({}, {*previous_controller}, deactivate_message)) {
        message = "Failed to deactivate active controller '" + *previous_controller + "': " +
          deactivate_message;
        return false;
      }
    }
  }

  bool operation_ok = false;
  std::string operation_message;
  {
    ScopedMaintenanceFlag scoped_maintenance(maintenance_active_);
    try {
      operation_ok = operation(joint, operation_message);
    } catch (const std::exception & error) {
      message = std::string("Maintenance operation threw an exception: ") + error.what();
      return false;
    }
  }

  const auto seed_result = seed_joint_commands_from_state();
  if (seed_result != hardware_interface::return_type::OK) {
    RCLCPP_WARN(get_logger(), "Failed to reseed joint commands after maintenance operation.");
  }

  bool restore_ok = true;
  std::string restore_message;
  if (maintenance_reload_enabled_) {
    const auto controller_to_restore = resolve_restore_controller(previous_controller);
    if (!controller_to_restore.empty()) {
      restore_ok = switch_controllers({controller_to_restore}, {}, restore_message);
    }
  }

  if (!operation_ok) {
    message = operation_message;
    if (!restore_ok) {
      message += " Also failed to restore controller: " + restore_message;
    }
    return false;
  }
  if (!restore_ok) {
    message = "Operation succeeded, but restoring controller failed: " + restore_message;
    return false;
  }
  if (seed_result != hardware_interface::return_type::OK) {
    message = operation_message + " Warning: failed to reseed joint commands.";
    return false;
  }

  message = operation_message;
  return true;
}

void MABSystemHardware::handle_run_calibration(
  const std::shared_ptr<mab_ros2_control::srv::RunCalibration::Request> request,
  std::shared_ptr<mab_ros2_control::srv::RunCalibration::Response> response)
{
  std::scoped_lock lock(maintenance_mutex_);

  auto * joint = find_joint_by_name(request->joint_name);
  if (!joint) {
    response->success = false;
    response->message = "Unknown joint name: " + request->joint_name;
    return;
  }

  std::string message;
  const bool success = perform_with_controller_reload(
    *joint,
    [this](JointData & selected_joint, std::string & operation_message) {
      return run_calibration_sequence(selected_joint, operation_message);
    },
    message);

  response->success = success;
  response->message = message;
}

void MABSystemHardware::handle_set_torque_bandwidth(
  const std::shared_ptr<mab_ros2_control::srv::SetTorqueBandwidth::Request> request,
  std::shared_ptr<mab_ros2_control::srv::SetTorqueBandwidth::Response> response)
{
  std::scoped_lock lock(maintenance_mutex_);

  auto * joint = find_joint_by_name(request->joint_name);
  if (!joint) {
    response->success = false;
    response->message = "Unknown joint name: " + request->joint_name;
    return;
  }
  if (!joint->connected || !joint->md) {
    response->success = false;
    response->message = "Requested joint is not connected.";
    return;
  }

  mab::MD::Error_t torque_result = mab::MD::Error_t::UNKNOWN_ERROR;
  {
    ScopedMaintenanceFlag scoped_maintenance(maintenance_active_);
    torque_result = joint->md->setTorqueBandwidth(request->bandwidth_hz);
  }

  if (torque_result != mab::MD::Error_t::OK) {
    response->success = false;
    response->message =
      "Failed to set torque bandwidth register (value may be out of supported range).";
    return;
  }

  joint->torque_bandwidth_hz = static_cast<double>(request->bandwidth_hz);
  response->success = true;
  response->message = "Torque bandwidth updated.";
}

void MABSystemHardware::handle_run_drive_tests(
  const std::shared_ptr<mab_ros2_control::srv::RunDriveTests::Request> request,
  std::shared_ptr<mab_ros2_control::srv::RunDriveTests::Response> response)
{
  std::scoped_lock lock(maintenance_mutex_);

  auto * joint = find_joint_by_name(request->joint_name);
  if (!joint) {
    response->success = false;
    response->message = "Unknown joint name: " + request->joint_name;
    return;
  }

  std::string message;
  const bool success = perform_with_controller_reload(
    *joint,
    [this](JointData & selected_joint, std::string & operation_message) {
      return run_drive_tests_sequence(selected_joint, operation_message);
    },
    message);

  response->success = success;
  response->message = message;
}

bool MABSystemHardware::parse_bool(const std::string & value, const bool default_value)
{
  const auto normalized = uppercase(value);
  if (normalized == "TRUE" || normalized == "1" || normalized == "YES" || normalized == "ON") {
    return true;
  }
  if (normalized == "FALSE" || normalized == "0" || normalized == "NO" || normalized == "OFF") {
    return false;
  }
  return default_value;
}

double MABSystemHardware::parse_double(const std::string & value)
{
  return std::stod(value);
}

std::string MABSystemHardware::uppercase(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::toupper(character));
  });
  return value;
}

std::optional<mab::CANdleDatarate_E> MABSystemHardware::parse_data_rate(const std::string & value)
{
  const auto normalized = uppercase(value);
  if (normalized == "1M" || normalized == "1000000") {
    return mab::CAN_DATARATE_1M;
  }
  if (normalized == "2M" || normalized == "2000000") {
    return mab::CAN_DATARATE_2M;
  }
  if (normalized == "5M" || normalized == "5000000") {
    return mab::CAN_DATARATE_5M;
  }
  if (normalized == "8M" || normalized == "8000000") {
    return mab::CAN_DATARATE_8M;
  }
  return std::nullopt;
}

std::optional<mab::candleTypes::busTypes_t> MABSystemHardware::parse_bus_type(
  const std::string & value)
{
  const auto normalized = uppercase(value);
  if (normalized == "USB") {
    return mab::candleTypes::USB;
  }
  if (normalized == "SPI") {
    return mab::candleTypes::SPI;
  }
  return std::nullopt;
}

std::optional<mab::socketIndex_E> MABSystemHardware::parse_socket_index(const int socket_index)
{
  switch (socket_index) {
    case 1:
      return mab::socketIndex_E::SOCKET_1;
    case 2:
      return mab::socketIndex_E::SOCKET_2;
    case 3:
      return mab::socketIndex_E::SOCKET_3;
    case 4:
      return mab::socketIndex_E::SOCKET_4;
    case 5:
      return mab::socketIndex_E::SOCKET_5;
    case 6:
      return mab::socketIndex_E::SOCKET_6;
    default:
      return std::nullopt;
  }
}

bool MABSystemHardware::has_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces, const std::string & name)
{
  return std::any_of(interfaces.begin(), interfaces.end(), [&name](const auto & interface_info) {
    return interface_info.name == name;
  });
}

}  // namespace mab_ros2_control

PLUGINLIB_EXPORT_CLASS(
  mab_ros2_control::MABSystemHardware, hardware_interface::SystemInterface)
