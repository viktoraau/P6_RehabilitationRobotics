#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mab_rehab
{

/// Thin client around /controller_manager/list_controllers and
/// /controller_manager/switch_controller. Used by maintenance
/// services to deactivate/restore the active command controller
/// around a calibration or drive-test run.
class ControllerSwitcher
{
public:
  explicit ControllerSwitcher(rclcpp::Node::SharedPtr node);

  /// Lazily create the service clients. Returns true once both clients
  /// exist and are ready (or waits up to timeout_ms). On failure,
  /// error_out is populated.
  bool ensure_ready(int timeout_ms, std::string & error_out);

  /// Return the name of the currently-active command controller
  /// (anything that claims position/velocity/effort interfaces),
  /// or std::nullopt if none is active.
  std::optional<std::string> find_active_command_controller(std::string & error_out);

  /// Activate `activate` and deactivate `deactivate` via a single
  /// switch_controller call. Uses STRICT strictness and sync activation.
  bool switch_controllers(
    const std::vector<std::string> & activate,
    const std::vector<std::string> & deactivate,
    std::string & error_out);

  /// Resolve the "what should I restore?" token:
  ///   "previous" -> whatever was active when maintenance started
  ///   "none"     -> leave deactivated
  ///   <name>     -> always switch to this controller
  std::string resolve_restore_controller(
    const std::string & token,
    const std::optional<std::string> & previous) const;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  int default_timeout_ms_{3000};
};

}  // namespace mab_rehab
