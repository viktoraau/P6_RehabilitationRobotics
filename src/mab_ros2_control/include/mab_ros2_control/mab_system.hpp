#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "mab_ros2_control/srv/run_calibration.hpp"
#include "mab_ros2_control/srv/run_drive_tests.hpp"
#include "mab_ros2_control/srv/set_torque_bandwidth.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "MD.hpp"
#include "USB.hpp"
#include "SPI.hpp"
#include "candle.hpp"
#include "candle_types.hpp"
#include "mab_types.hpp"
#include "pds.hpp"
#include "pds_types.hpp"
#include "power_stage.hpp"

namespace mab_ros2_control
{

class MABSystemHardware : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

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
  struct JointData
  {
    enum class CommandMode
    {
      POSITION,
      VELOCITY,
      IMPEDANCE
    };

    std::string name;
    std::string motor_name;

    mab::canId_t can_id = 0;
    double gear_ratio = 1.0;

    double motor_pole_pairs = 0.0;
    double motor_kv = 0.0;
    double torque_constant = 0.0;
    double max_current = 0.0;
    double torque_bandwidth_hz = 0.0;
    double friction = 0.0;
    double stiction = 0.0;
    double motor_calibration_mode = 0.0;
    double shutdown_temp = 0.0;

    double limit_max_torque = 0.0;
    double limit_max_velocity = 0.0;
    double limit_position_min = 0.0;
    double limit_position_max = 0.0;
    double limit_max_acceleration = 0.0;
    double limit_max_deceleration = 0.0;

    double profile_velocity = 0.0;
    double profile_acceleration = 0.0;
    double profile_deceleration = 0.0;
    double profile_quick_stop_deceleration = 0.0;

    double output_encoder = 0.0;
    double output_encoder_default_baud = 0.0;
    double output_encoder_mode = 0.0;
    double output_encoder_calibration_mode = 0.0;

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

    double user_gpio_configuration = 0.0;
    double reverse_direction = 0.0;
    double shunt_resistance = 0.0;

    bool has_position_command = false;
    bool has_velocity_command = false;
    bool has_effort_command = false;
    bool has_position_state = false;
    bool has_velocity_state = false;
    bool has_effort_state = false;
    bool has_voltage_state = false;

    bool connected = false;
    CommandMode active_command_mode = CommandMode::POSITION;
    double last_position_command = 0.0;
    double last_velocity_command = 0.0;
    double last_effort_command = 0.0;
    std::optional<float> last_sent_motor_position_command;
    std::optional<float> last_sent_motor_velocity_command;
    std::optional<float> last_sent_motor_effort_command;
    double state_position = 0.0;
    double state_velocity = 0.0;
    double state_effort = 0.0;
    double state_voltage = 0.0;
    size_t consecutive_read_failures = 0;
    size_t consecutive_write_failures = 0;

    std::unique_ptr<mab::MD> md;
  };

  struct PowerStageState
  {
    bool exported = false;
    double bus_voltage = 0.0;
  };

  struct CandleDeleter
  {
    void operator()(mab::Candle * candle) const;
  };

  using CandlePtr = std::unique_ptr<mab::Candle, CandleDeleter>;

  CallbackReturn parse_hardware_parameters();
  CallbackReturn parse_joint_parameters();

  bool connect_candle();
  void disconnect_candle();
  bool setup_pds();
  bool activate_drive(JointData & joint);
  bool configure_drive(JointData & joint);
  void disable_all_drives();
  void disable_power_stage();
  void setup_maintenance_interfaces();
  void reset_maintenance_interfaces();

  JointData * find_joint_by_name(const std::string & joint_name);
  std::optional<std::string> find_active_command_controller(std::string & message);
  std::string resolve_restore_controller(
    const std::optional<std::string> & previous_controller) const;
  bool ensure_controller_clients_ready(std::string & message);
  bool switch_controllers(
    const std::vector<std::string> & activate_controllers,
    const std::vector<std::string> & deactivate_controllers, std::string & message);
  bool run_calibration_sequence(JointData & joint, std::string & message);
  bool run_drive_tests_sequence(JointData & joint, std::string & message);
  bool perform_with_controller_reload(
    JointData & joint, const std::function<bool(JointData &, std::string &)> & operation,
    std::string & message);

  void handle_run_calibration(
    const std::shared_ptr<mab_ros2_control::srv::RunCalibration::Request> request,
    std::shared_ptr<mab_ros2_control::srv::RunCalibration::Response> response);
  void handle_set_torque_bandwidth(
    const std::shared_ptr<mab_ros2_control::srv::SetTorqueBandwidth::Request> request,
    std::shared_ptr<mab_ros2_control::srv::SetTorqueBandwidth::Response> response);
  void handle_run_drive_tests(
    const std::shared_ptr<mab_ros2_control::srv::RunDriveTests::Request> request,
    std::shared_ptr<mab_ros2_control::srv::RunDriveTests::Response> response);

  hardware_interface::return_type read_power_stage_telemetry();
  hardware_interface::return_type seed_joint_commands_from_state();

  static bool parse_bool(const std::string & value, bool default_value = false);
  static double parse_double(const std::string & value);
  static std::string uppercase(std::string value);
  static std::optional<mab::CANdleDatarate_E> parse_data_rate(const std::string & value);
  static std::optional<mab::candleTypes::busTypes_t> parse_bus_type(const std::string & value);
  static std::optional<mab::socketIndex_E> parse_socket_index(int socket_index);
  static bool has_interface(
    const std::vector<hardware_interface::InterfaceInfo> & interfaces,
    const std::string & name);

  template <typename MapT>
  static std::string get_required_param(
    const MapT & parameters, const std::string & key, const std::string & context)
  {
    const auto it = parameters.find(key);
    if (it == parameters.end()) {
      throw std::runtime_error("Missing parameter '" + key + "' for " + context);
    }
    return it->second;
  }

  std::vector<JointData> joints_;
  PowerStageState power_stage_state_;

  std::string bus_ = "USB";
  std::string data_rate_ = "1M";
  bool use_pds_ = true;
  bool use_regular_can_frames_ = true;
  int pds_id_ = 100;
  int power_stage_socket_ = 2;
  bool fast_mode_ = false;
  int telemetry_divider_ = 10;
  bool auto_enable_power_stage_ = true;
  bool disable_power_stage_on_deactivate_ = true;
  bool zero_on_activate_ = false;
  bool allow_no_connected_drives_ = true;
  bool save_md_configuration_to_flash_ = false;
  double md_can_watchdog_ms_ = std::numeric_limits<double>::quiet_NaN();
  int hold_position_on_activate_ms_ = 3000;
  bool maintenance_reload_enabled_ = true;
  std::string maintenance_restore_controller_ = "previous";
  int maintenance_service_timeout_ms_ = 3000;

  size_t read_cycle_counter_ = 0;
  std::optional<std::chrono::steady_clock::time_point> command_hold_until_;
  bool command_hold_notice_emitted_ = false;
  std::atomic<bool> maintenance_active_{false};
  std::mutex maintenance_mutex_;

  CandlePtr candle_{nullptr};
  std::unique_ptr<mab::Pds> pds_;
  std::shared_ptr<mab::PowerStage> power_stage_;
  rclcpp::Service<mab_ros2_control::srv::RunCalibration>::SharedPtr run_calibration_service_;
  rclcpp::Service<mab_ros2_control::srv::SetTorqueBandwidth>::SharedPtr
    set_torque_bandwidth_service_;
  rclcpp::Service<mab_ros2_control::srv::RunDriveTests>::SharedPtr run_drive_tests_service_;
  rclcpp::CallbackGroup::SharedPtr maintenance_callback_group_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr
    list_controllers_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    switch_controller_client_;
};

}  // namespace mab_ros2_control
