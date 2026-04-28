#include "mab_rehab/config/joint_config.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace mab_rehab
{

namespace
{

double parse_double_or(const std::string & value, double default_value)
{
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    return default_value;
  }
}

template <typename Map>
const std::string * find(const Map & parameters, const std::string & key)
{
  const auto it = parameters.find(key);
  return (it == parameters.end()) ? nullptr : &it->second;
}

template <typename Map>
const std::string & require(const Map & parameters, const std::string & key, const std::string & joint)
{
  const auto it = parameters.find(key);
  if (it == parameters.end()) {
    throw std::runtime_error("Missing required parameter '" + key + "' for joint " + joint);
  }
  return it->second;
}

bool has_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces, const std::string & name)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [&name](const hardware_interface::InterfaceInfo & info) { return info.name == name; });
}

}  // namespace

JointConfig JointConfig::parse(const hardware_interface::ComponentInfo & joint_info)
{
  JointConfig joint;
  joint.name = joint_info.name;

  const auto & params = joint_info.parameters;

  // Required fields.
  joint.can_id = static_cast<mab::canId_t>(
    std::stoul(require(params, "can_id", joint.name)));

  if (auto * v = find(params, "config_path")) {
    joint.config_path = *v;
  }

  joint.gear_ratio = parse_double_or(require(params, "gear_ratio", joint.name), 1.0);
  if (!std::isfinite(joint.gear_ratio) || joint.gear_ratio <= 0.0) {
    throw std::runtime_error("Invalid gear_ratio for joint " + joint.name);
  }

  // Optional — startup reference. NaN means "no startup reference for this joint".
  if (auto * v = find(params, "startup_reference_position")) {
    joint.startup_reference_position = parse_double_or(
      *v, std::numeric_limits<double>::quiet_NaN());
  }

  // Safety envelope — all optional; defaults are ±inf so they do nothing
  // unless a value is provided via the safety_limits.yaml.
  if (auto * v = find(params, "safety_position_min")) {
    joint.safety_position_min = parse_double_or(*v, joint.safety_position_min);
  }
  if (auto * v = find(params, "safety_position_max")) {
    joint.safety_position_max = parse_double_or(*v, joint.safety_position_max);
  }
  if (auto * v = find(params, "safety_max_velocity")) {
    joint.safety_max_velocity = parse_double_or(*v, joint.safety_max_velocity);
  }
  if (auto * v = find(params, "safety_max_acceleration")) {
    joint.safety_max_acceleration = parse_double_or(*v, joint.safety_max_acceleration);
  }
  if (auto * v = find(params, "safety_max_deceleration")) {
    joint.safety_max_deceleration = parse_double_or(*v, joint.safety_max_deceleration);
  }
  if (auto * v = find(params, "safety_max_torque")) {
    joint.safety_max_torque = parse_double_or(*v, joint.safety_max_torque);
  }
  if (joint.safety_position_min > joint.safety_position_max) {
    std::swap(joint.safety_position_min, joint.safety_position_max);
  }

  // PID / impedance — sourced from config/pid_tuning.yaml.
  if (auto * v = find(params, "position_pid_kp")) {
    joint.position_pid_kp = parse_double_or(*v, 0.0);
  }
  if (auto * v = find(params, "position_pid_ki")) {
    joint.position_pid_ki = parse_double_or(*v, 0.0);
  }
  if (auto * v = find(params, "position_pid_kd")) {
    joint.position_pid_kd = parse_double_or(*v, 0.0);
  }
  if (auto * v = find(params, "position_pid_windup")) {
    joint.position_pid_windup = parse_double_or(*v, 0.0);
  }
  if (auto * v = find(params, "velocity_pid_kp")) {
    joint.velocity_pid_kp = parse_double_or(*v, 0.0);
  }
  if (auto * v = find(params, "velocity_pid_ki")) {
    joint.velocity_pid_ki = parse_double_or(*v, 0.0);
  }
  if (auto * v = find(params, "velocity_pid_kd")) {
    joint.velocity_pid_kd = parse_double_or(*v, 0.0);
  }
  if (auto * v = find(params, "velocity_pid_windup")) {
    joint.velocity_pid_windup = parse_double_or(*v, 0.0);
  }
  if (auto * v = find(params, "impedance_kp")) {
    joint.impedance_kp = parse_double_or(*v, 0.0);
  }
  if (auto * v = find(params, "impedance_kd")) {
    joint.impedance_kd = parse_double_or(*v, 0.0);
  }
  if (auto * v = find(params, "motor_friction")) {
    joint.motor_friction = parse_double_or(*v, 0.0);
  }
  if (auto * v = find(params, "motor_stiction")) {
    joint.motor_stiction = parse_double_or(*v, 0.0);
  }

  // Interface introspection.
  joint.has_position_command =
    has_interface(joint_info.command_interfaces, hardware_interface::HW_IF_POSITION);
  joint.has_velocity_command =
    has_interface(joint_info.command_interfaces, hardware_interface::HW_IF_VELOCITY);
  joint.has_effort_command =
    has_interface(joint_info.command_interfaces, hardware_interface::HW_IF_EFFORT);
  joint.has_position_state =
    has_interface(joint_info.state_interfaces, hardware_interface::HW_IF_POSITION);
  joint.has_velocity_state =
    has_interface(joint_info.state_interfaces, hardware_interface::HW_IF_VELOCITY);
  joint.has_effort_state =
    has_interface(joint_info.state_interfaces, hardware_interface::HW_IF_EFFORT);
  joint.has_voltage_state = has_interface(joint_info.state_interfaces, "voltage");
  joint.has_gear_ratio_state = has_interface(joint_info.state_interfaces, "gear_ratio");

  if (!joint.has_position_command || !joint.has_velocity_command) {
    throw std::runtime_error(
            "Joint " + joint.name +
            " must export position and velocity command interfaces");
  }
  if (!joint.has_position_state || !joint.has_velocity_state) {
    throw std::runtime_error(
            "Joint " + joint.name + " must export position and velocity state interfaces");
  }

  return joint;
}

std::vector<JointConfig> JointConfig::parse_all(
  const std::vector<hardware_interface::ComponentInfo> & joints)
{
  std::vector<JointConfig> result;
  result.reserve(joints.size());
  for (const auto & joint_info : joints) {
    result.push_back(parse(joint_info));
  }
  return result;
}

}  // namespace mab_rehab
