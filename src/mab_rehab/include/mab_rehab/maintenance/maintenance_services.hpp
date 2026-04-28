#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "mab_rehab/srv/run_calibration.hpp"
#include "mab_rehab/srv/run_drive_tests.hpp"
#include "mab_rehab/srv/set_torque_bandwidth.hpp"

#include "mab_rehab/maintenance/controller_switcher.hpp"
#include "mab_rehab/rt/maintenance_gate.hpp"
#include "mab_rehab/transport/md_drive.hpp"

namespace mab_rehab
{

class HardwareConfig;

/// Hosts the three maintenance ROS services (calibration, drive tests,
/// set torque bandwidth) on a dedicated callback group so they never
/// block the RT thread. Uses MaintenanceGate to pause read/write during
/// the operation and ControllerSwitcher to restore the active controller.
class MaintenanceServices
{
public:
  using DriveLookup = std::function<MdDrive *(const std::string & joint_name)>;

  MaintenanceServices(
    rclcpp::Node::SharedPtr node,
    MaintenanceGate & gate,
    std::mutex & transport_mutex,
    ControllerSwitcher & switcher,
    DriveLookup drive_lookup,
    const HardwareConfig & hardware_config);

  ~MaintenanceServices() = default;

  MaintenanceServices(const MaintenanceServices &) = delete;
  MaintenanceServices & operator=(const MaintenanceServices &) = delete;

private:
  void handle_run_calibration(
    const std::shared_ptr<srv::RunCalibration::Request> request,
    std::shared_ptr<srv::RunCalibration::Response> response);
  void handle_run_drive_tests(
    const std::shared_ptr<srv::RunDriveTests::Request> request,
    std::shared_ptr<srv::RunDriveTests::Response> response);
  void handle_set_torque_bandwidth(
    const std::shared_ptr<srv::SetTorqueBandwidth::Request> request,
    std::shared_ptr<srv::SetTorqueBandwidth::Response> response);

  /// Run `operation` with the MaintenanceGate held and the active
  /// controller deactivated. On completion, restore the controller.
  bool run_scoped(
    const std::string & joint_name,
    const std::function<bool(MdDrive &, std::string &)> & operation,
    std::string & message);

  rclcpp::Node::SharedPtr node_;
  MaintenanceGate & gate_;
  std::mutex & transport_mutex_;
  ControllerSwitcher & switcher_;
  DriveLookup drive_lookup_;
  const HardwareConfig & config_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Service<srv::RunCalibration>::SharedPtr run_calibration_srv_;
  rclcpp::Service<srv::RunDriveTests>::SharedPtr run_drive_tests_srv_;
  rclcpp::Service<srv::SetTorqueBandwidth>::SharedPtr set_torque_bandwidth_srv_;

  std::mutex maintenance_mutex_;  // serializes service calls against each other
};

}  // namespace mab_rehab
