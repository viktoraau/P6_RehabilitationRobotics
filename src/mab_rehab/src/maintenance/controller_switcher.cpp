#include "mab_rehab/maintenance/controller_switcher.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>
#include <string_view>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace mab_rehab
{

namespace
{

bool is_command_interface_claimed(const std::string & interface_name)
{
  static const std::string kSlashPos =
    std::string("/") + hardware_interface::HW_IF_POSITION;
  static const std::string kSlashVel =
    std::string("/") + hardware_interface::HW_IF_VELOCITY;
  static const std::string kSlashEff =
    std::string("/") + hardware_interface::HW_IF_EFFORT;
  return interface_name.ends_with(kSlashPos) ||
         interface_name.ends_with(kSlashVel) ||
         interface_name.ends_with(kSlashEff);
}

std::string upper(std::string v)
{
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {
    return std::toupper(c);
  });
  return v;
}

}  // namespace

ControllerSwitcher::ControllerSwitcher(rclcpp::Node::SharedPtr node)
: node_(std::move(node))
{
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  list_client_ = node_->create_client<controller_manager_msgs::srv::ListControllers>(
    "/controller_manager/list_controllers",
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    callback_group_);
  switch_client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller",
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    callback_group_);
}

bool ControllerSwitcher::ensure_ready(const int timeout_ms, std::string & error_out)
{
  default_timeout_ms_ = timeout_ms;
  const auto timeout = std::chrono::milliseconds(timeout_ms);
  if (!list_client_->wait_for_service(timeout)) {
    error_out = "Timed out waiting for /controller_manager/list_controllers.";
    return false;
  }
  if (!switch_client_->wait_for_service(timeout)) {
    error_out = "Timed out waiting for /controller_manager/switch_controller.";
    return false;
  }
  return true;
}

std::optional<std::string> ControllerSwitcher::find_active_command_controller(
  std::string & error_out)
{
  error_out.clear();
  if (!ensure_ready(default_timeout_ms_, error_out)) {
    return std::nullopt;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto future = list_client_->async_send_request(request);
  if (future.wait_for(std::chrono::milliseconds(default_timeout_ms_)) !=
    std::future_status::ready)
  {
    error_out = "ListControllers request timed out.";
    return std::nullopt;
  }
  const auto response = future.get();
  if (!response) {
    error_out = "ListControllers returned an empty response.";
    return std::nullopt;
  }

  std::vector<std::string> active;
  for (const auto & controller : response->controller) {
    if (controller.state != "active") { continue; }
    const bool claims_command = std::any_of(
      controller.claimed_interfaces.begin(), controller.claimed_interfaces.end(),
      [](const auto & c) { return is_command_interface_claimed(c); });
    if (claims_command) {
      active.push_back(controller.name);
    }
  }

  if (active.empty()) { return std::nullopt; }
  if (active.size() > 1) {
    std::ostringstream stream;
    for (size_t i = 0; i < active.size(); ++i) {
      stream << active[i];
      if (i + 1 < active.size()) { stream << ", "; }
    }
    error_out = "More than one active command controller detected: " + stream.str();
    return std::nullopt;
  }
  return active.front();
}

bool ControllerSwitcher::switch_controllers(
  const std::vector<std::string> & activate,
  const std::vector<std::string> & deactivate,
  std::string & error_out)
{
  if (activate.empty() && deactivate.empty()) {
    return true;
  }
  if (!ensure_ready(default_timeout_ms_, error_out)) {
    return false;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->activate_controllers = activate;
  request->deactivate_controllers = deactivate;
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  request->activate_asap = false;
  request->timeout.sec = default_timeout_ms_ / 1000;
  request->timeout.nanosec =
    static_cast<uint32_t>((default_timeout_ms_ % 1000) * 1'000'000);

  auto future = switch_client_->async_send_request(request);
  if (future.wait_for(std::chrono::milliseconds(default_timeout_ms_)) !=
    std::future_status::ready)
  {
    error_out = "SwitchController request timed out.";
    return false;
  }
  const auto response = future.get();
  if (!response) {
    error_out = "SwitchController returned an empty response.";
    return false;
  }
  if (!response->ok) {
    error_out = response->message.empty() ?
      "SwitchController returned failure." : response->message;
    return false;
  }
  return true;
}

std::string ControllerSwitcher::resolve_restore_controller(
  const std::string & token,
  const std::optional<std::string> & previous) const
{
  if (upper(token) == "PREVIOUS") {
    return previous.value_or("");
  }
  if (upper(token) == "NONE") {
    return "";
  }
  return token;
}

}  // namespace mab_rehab
