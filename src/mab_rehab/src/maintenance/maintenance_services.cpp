#include "mab_rehab/maintenance/maintenance_services.hpp"

#include <chrono>
#include <thread>

#include "mab_rehab/config/hardware_config.hpp"

namespace mab_rehab
{

namespace
{

bool run_calibration_on(MdDrive & drive, std::string & message)
{
  auto * md = drive.md();
  if (!md) {
    message = "Drive handle is null";
    return false;
  }

  if (md->disable() != mab::MD::Error_t::OK) {
    message = "Failed to disable drive before calibration.";
    return false;
  }

  md->m_mdRegisters.runCalibrateCmd = 1;
  if (md->writeRegisters(md->m_mdRegisters.runCalibrateCmd) != mab::MD::Error_t::OK) {
    message = "Failed to send main calibration command.";
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  md->m_mdRegisters.runCalibratePiGains = 1;
  if (md->writeRegisters(md->m_mdRegisters.runCalibratePiGains) != mab::MD::Error_t::OK) {
    message = "Failed to send PI gain calibration command.";
    return false;
  }

  md->clearErrors();
  if (md->setMotionMode(mab::MdMode_E::POSITION_PID) != mab::MD::Error_t::OK) {
    message = "Calibration completed, but failed to restore POSITION_PID mode.";
    return false;
  }
  drive.set_active_mode(MdDrive::CommandMode::Position);
  if (md->enable() != mab::MD::Error_t::OK) {
    message = "Calibration completed, but failed to re-enable drive.";
    return false;
  }

  message = "Main and PI calibration commands sent successfully.";
  return true;
}

bool run_drive_tests_on(MdDrive & drive, std::string & message)
{
  auto * md = drive.md();
  if (!md) {
    message = "Drive handle is null";
    return false;
  }

  if (md->disable() != mab::MD::Error_t::OK) {
    message = "Failed to disable drive before running tests.";
    return false;
  }

  md->m_mdRegisters.runTestMainEncoderCmd = 1;
  if (md->writeRegisters(md->m_mdRegisters.runTestMainEncoderCmd) != mab::MD::Error_t::OK) {
    message = "Failed to trigger main encoder test.";
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  md->m_mdRegisters.runTestAuxEncoderCmd = 1;
  if (md->writeRegisters(md->m_mdRegisters.runTestAuxEncoderCmd) != mab::MD::Error_t::OK) {
    message = "Failed to trigger auxiliary encoder test.";
    return false;
  }

  md->clearErrors();
  if (md->setMotionMode(mab::MdMode_E::POSITION_PID) != mab::MD::Error_t::OK) {
    message = "Drive tests completed, but failed to restore POSITION_PID mode.";
    return false;
  }
  drive.set_active_mode(MdDrive::CommandMode::Position);
  if (md->enable() != mab::MD::Error_t::OK) {
    message = "Drive tests completed, but failed to re-enable drive.";
    return false;
  }

  message = "Main and auxiliary encoder test commands sent successfully.";
  return true;
}

bool set_torque_bandwidth_on(MdDrive & drive, uint16_t bandwidth, std::string & message)
{
  auto * md = drive.md();
  if (!md) {
    message = "Drive handle is null";
    return false;
  }

  if (md->setTorqueBandwidth(bandwidth) != mab::MD::Error_t::OK) {
    message = "Failed to set torque bandwidth.";
    return false;
  }
  message = "Torque bandwidth updated.";
  return true;
}

}  // namespace

MaintenanceServices::MaintenanceServices(
  rclcpp::Node::SharedPtr node,
  MaintenanceGate & gate,
  std::mutex & transport_mutex,
  ControllerSwitcher & switcher,
  DriveLookup drive_lookup,
  const HardwareConfig & hardware_config)
: node_(std::move(node)),
  gate_(gate),
  transport_mutex_(transport_mutex),
  switcher_(switcher),
  drive_lookup_(std::move(drive_lookup)),
  config_(hardware_config)
{
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  run_calibration_srv_ = node_->create_service<srv::RunCalibration>(
    "~/run_calibration",
    std::bind(
      &MaintenanceServices::handle_run_calibration, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    callback_group_);

  run_drive_tests_srv_ = node_->create_service<srv::RunDriveTests>(
    "~/run_drive_tests",
    std::bind(
      &MaintenanceServices::handle_run_drive_tests, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    callback_group_);

  set_torque_bandwidth_srv_ = node_->create_service<srv::SetTorqueBandwidth>(
    "~/set_torque_bandwidth",
    std::bind(
      &MaintenanceServices::handle_set_torque_bandwidth, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    callback_group_);
}

bool MaintenanceServices::run_scoped(
  const std::string & joint_name,
  const std::function<bool(MdDrive &, std::string &)> & operation,
  std::string & message)
{
  std::lock_guard<std::mutex> mtx(maintenance_mutex_);

  auto * drive = drive_lookup_(joint_name);
  if (drive == nullptr) {
    message = "Unknown joint name: " + joint_name;
    return false;
  }

  // 1. Remember the active command controller so we can restore it later.
  std::optional<std::string> previous_controller;
  if (config_.maintenance_reload_enabled) {
    std::string scan_message;
    previous_controller = switcher_.find_active_command_controller(scan_message);
    if (!scan_message.empty()) {
      RCLCPP_WARN(node_->get_logger(), "%s", scan_message.c_str());
    }
    if (previous_controller.has_value()) {
      std::string switch_message;
      if (!switcher_.switch_controllers({}, {*previous_controller}, switch_message)) {
        message = "Failed to deactivate controller before maintenance: " + switch_message;
        return false;
      }
    }
  }

  // 2. Request the RT thread to hand off the bus and wait for ack.
  if (!gate_.request_handoff()) {
    message = "Maintenance gate is already engaged.";
    return false;
  }
  if (!gate_.wait_for_acquired(config_.maintenance_service_timeout_ms)) {
    message = "Timed out waiting for RT thread to acknowledge maintenance handoff.";
    gate_.release();
    return false;
  }

  // 3. Run the actual operation.
  bool op_ok = false;
  {
    std::lock_guard<std::mutex> transport_lock(transport_mutex_);
    op_ok = operation(*drive, message);
  }

  // 4. Release the gate (RT resumes).
  gate_.release();

  // 5. Restore the previous controller (if one was active).
  if (config_.maintenance_reload_enabled) {
    const std::string restore_name =
      switcher_.resolve_restore_controller(
        config_.maintenance_restore_controller, previous_controller);
    if (!restore_name.empty()) {
      std::string switch_message;
      if (!switcher_.switch_controllers({restore_name}, {}, switch_message)) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Failed to restore controller '%s' after maintenance: %s",
          restore_name.c_str(), switch_message.c_str());
        if (op_ok) {
          message += " (warning: failed to restore controller '" + restore_name + "')";
        }
      }
    }
  }

  return op_ok;
}

void MaintenanceServices::handle_run_calibration(
  const std::shared_ptr<srv::RunCalibration::Request> request,
  std::shared_ptr<srv::RunCalibration::Response> response)
{
  response->success = run_scoped(
    request->joint_name,
    [](MdDrive & drive, std::string & message) {
      return run_calibration_on(drive, message);
    },
    response->message);
}

void MaintenanceServices::handle_run_drive_tests(
  const std::shared_ptr<srv::RunDriveTests::Request> request,
  std::shared_ptr<srv::RunDriveTests::Response> response)
{
  response->success = run_scoped(
    request->joint_name,
    [](MdDrive & drive, std::string & message) {
      return run_drive_tests_on(drive, message);
    },
    response->message);
}

void MaintenanceServices::handle_set_torque_bandwidth(
  const std::shared_ptr<srv::SetTorqueBandwidth::Request> request,
  std::shared_ptr<srv::SetTorqueBandwidth::Response> response)
{
  const uint16_t bandwidth = request->bandwidth_hz;
  response->success = run_scoped(
    request->joint_name,
    [bandwidth](MdDrive & drive, std::string & message) {
      return set_torque_bandwidth_on(drive, bandwidth, message);
    },
    response->message);
}

}  // namespace mab_rehab
