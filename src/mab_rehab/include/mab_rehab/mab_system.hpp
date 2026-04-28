#pragma once

#include <atomic>
#include <future>
#include <mutex>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "mab_rehab/config/hardware_config.hpp"
#include "mab_rehab/config/joint_config.hpp"
#include "mab_rehab/maintenance/controller_switcher.hpp"
#include "mab_rehab/maintenance/maintenance_services.hpp"
#include "mab_rehab/power/pds_manager.hpp"
#include "mab_rehab/rt/health_monitor.hpp"
#include "mab_rehab/rt/maintenance_gate.hpp"
#include "mab_rehab/rt/mode_switch_worker.hpp"
#include "mab_rehab/transport/candle_transport.hpp"
#include "mab_rehab/transport/md_drive.hpp"

namespace mab_rehab
{

/// ros2_control SystemInterface for the 3-axis rehab robot.
///
/// Orchestrates the decomposed modules; owns no SDK state directly.
/// read() and write() are the only methods that run on the RT thread;
/// everything else (maintenance services, health monitoring, mode
/// switches) runs on non-RT executors via explicit handoff primitives.
class MABSystemHardware : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

private:
  // --- Configuration (populated in on_init) ---
  HardwareConfig hardware_config_;
  std::vector<JointConfig> joint_configs_;

  // --- Transport / drives / power ---
  CandleTransport transport_;
  std::vector<std::unique_ptr<MdDrive>> drives_;
  PdsManager pds_;

  // --- RT primitives ---
  MaintenanceGate gate_;
  std::unique_ptr<HealthMonitor> health_monitor_;
  std::unique_ptr<ModeSwitchWorker> mode_switch_worker_;
  std::mutex transport_mutex_;

  // --- Non-RT services ---
  std::unique_ptr<ControllerSwitcher> controller_switcher_;
  std::unique_ptr<MaintenanceServices> maintenance_services_;

  // --- RT state ---
  size_t read_cycle_counter_{0};
  // Pre-allocated per-drive futures for the two-pass parallel write path.
  // Sized to drives_.size() in on_activate; never reallocated in the hot path.
  std::vector<std::future<mab::MD::Error_t>> write_futures_;

  // --- Helpers ---
  MdDrive * find_drive(const std::string & joint_name);
  void publish_state_from_snapshot(const MdDrive & drive, const MdDrive::StateSnapshot & snap);
  static void apply_safety_clamp(
    const JointConfig & joint, const MdDrive::StateSnapshot & state,
    double & position_cmd, double & velocity_cmd, double & effort_cmd);
};

}  // namespace mab_rehab
