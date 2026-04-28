#pragma once

#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "mab_types.hpp"

namespace mab_rehab
{

/// Per-joint configuration parsed from the ros2_control <joint> tag.
/// Slim version of the legacy mab_ros2_control JointData — only holds
/// fields that are actually consumed by the code. Everything motor-level
/// (PID gains, motion profiles, encoder, GPIO) is uploaded from the
/// per-joint .cfg file; safety/startup are loaded from YAML by the launch
/// file and injected as xacro args.
struct JointConfig
{
  std::string name;

  // Wiring
  mab::canId_t can_id = 0;
  std::string config_path;   // path to the MD motor .cfg for this joint
  double gear_ratio = 1.0;

  // Startup reference (optional): position the joint is assumed parked at
  // when ros2_control activates. NaN means "no offset, use raw MD position".
  double startup_reference_position = std::numeric_limits<double>::quiet_NaN();

  // Safety envelope (applied in the write path when safety_limits_enabled).
  double safety_position_min = -std::numeric_limits<double>::infinity();
  double safety_position_max = std::numeric_limits<double>::infinity();
  double safety_max_velocity = std::numeric_limits<double>::infinity();
  double safety_max_acceleration = std::numeric_limits<double>::infinity();
  double safety_max_deceleration = std::numeric_limits<double>::infinity();
  double safety_max_torque = std::numeric_limits<double>::infinity();

  // PID / impedance gains that override whatever the .cfg file uploaded.
  // Sourced from config/pid_tuning.yaml through the launch file.
  double position_pid_kp = 0.0;
  double position_pid_ki = 0.0;
  double position_pid_kd = 0.0;
  double position_pid_windup = 0.0;
  double velocity_pid_kp = 0.0;
  double velocity_pid_ki = 0.0;
  double velocity_pid_kd = 0.0;
  double velocity_pid_windup = 0.0;
  double impedance_kp = 0.0;
  double impedance_kd = 0.0;
  double motor_friction = 0.0;  // dynamic (viscous) friction, MD register motorFriction
  double motor_stiction = 0.0;  // static (breakaway) friction, MD register motorStiction

  // Which interfaces the URDF declared for this joint.
  bool has_position_command = false;
  bool has_velocity_command = false;
  bool has_effort_command = false;
  bool has_position_state = false;
  bool has_velocity_state = false;
  bool has_effort_state = false;
  bool has_voltage_state = false;
  bool has_gear_ratio_state = false;

  /// Parse from a ros2_control <joint> block. Throws std::runtime_error
  /// on missing required fields (can_id, gear_ratio) or malformed values.
  static JointConfig parse(const hardware_interface::ComponentInfo & joint_info);

  /// Parse all joints in info_. Throws on the first bad joint.
  static std::vector<JointConfig> parse_all(
    const std::vector<hardware_interface::ComponentInfo> & joints);
};

}  // namespace mab_rehab
