#include "mab_rehab/config/hardware_config.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>

namespace mab_rehab
{

namespace
{

std::string uppercase(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return std::toupper(c);
  });
  return value;
}

bool parse_bool(const std::string & value, const bool default_value)
{
  const auto upper = uppercase(value);
  if (upper == "TRUE" || upper == "1" || upper == "YES" || upper == "ON") {
    return true;
  }
  if (upper == "FALSE" || upper == "0" || upper == "NO" || upper == "OFF") {
    return false;
  }
  return default_value;
}

double parse_double(const std::string & value)
{
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

int parse_int(const std::string & value, const int default_value)
{
  try {
    return std::stoi(value);
  } catch (const std::exception &) {
    return default_value;
  }
}

template <typename Map>
const std::string * find_param(const Map & parameters, const std::string & key)
{
  const auto it = parameters.find(key);
  return (it == parameters.end()) ? nullptr : &it->second;
}

}  // namespace

HardwareConfig HardwareConfig::parse(
  const std::unordered_map<std::string, std::string> & parameters)
{
  HardwareConfig config;

  if (auto * v = find_param(parameters, "bus")) {
    config.bus = uppercase(*v);
  }
  if (auto * v = find_param(parameters, "data_rate")) {
    config.data_rate = uppercase(*v);
  }
  if (auto * v = find_param(parameters, "fast_mode")) {
    config.fast_mode = parse_bool(*v, false);
  }

  if (auto * v = find_param(parameters, "use_pds")) {
    config.use_pds = parse_bool(*v, true);
  }
  if (auto * v = find_param(parameters, "pds_id")) {
    config.pds_id = parse_int(*v, 100);
  }
  if (auto * v = find_param(parameters, "power_stage_socket")) {
    config.power_stage_socket = parse_int(*v, 2);
  }
  if (auto * v = find_param(parameters, "use_regular_can_frames")) {
    config.use_regular_can_frames = parse_bool(*v, true);
  }
  if (auto * v = find_param(parameters, "auto_enable_power_stage")) {
    config.auto_enable_power_stage = parse_bool(*v, true);
  }
  if (auto * v = find_param(parameters, "disable_power_stage_on_deactivate")) {
    config.disable_power_stage_on_deactivate = parse_bool(*v, true);
  }

  if (auto * v = find_param(parameters, "zero_on_activate")) {
    config.zero_on_activate = parse_bool(*v, false);
  }
  if (auto * v = find_param(parameters, "startup_in_idle_mode")) {
    config.startup_in_idle_mode = parse_bool(*v, true);
  }
  if (auto * v = find_param(parameters, "startup_reference_enabled")) {
    config.startup_reference_enabled = parse_bool(*v, false);
  }
  if (auto * v = find_param(parameters, "allow_no_connected_drives")) {
    config.allow_no_connected_drives = parse_bool(*v, true);
  }
  if (auto * v = find_param(parameters, "save_md_configuration_to_flash")) {
    config.save_md_configuration_to_flash = parse_bool(*v, false);
  }
  if (auto * v = find_param(parameters, "hold_position_on_activate_ms")) {
    config.hold_position_on_activate_ms = parse_int(*v, 0);
  }

  if (auto * v = find_param(parameters, "md_can_watchdog_ms")) {
    config.md_can_watchdog_ms = parse_double(*v);
  }
  if (auto * v = find_param(parameters, "safety_limits_enabled")) {
    config.safety_limits_enabled = parse_bool(*v, false);
  }

  if (auto * v = find_param(parameters, "maintenance_reload_enabled")) {
    config.maintenance_reload_enabled = parse_bool(*v, true);
  }
  if (auto * v = find_param(parameters, "maintenance_restore_controller")) {
    config.maintenance_restore_controller = *v;
  }
  if (auto * v = find_param(parameters, "maintenance_service_timeout_ms")) {
    config.maintenance_service_timeout_ms = parse_int(*v, 3000);
  }

  return config;
}

std::optional<mab::candleTypes::busTypes_t> HardwareConfig::bus_type() const
{
  if (bus == "USB") {
    return mab::candleTypes::busTypes_t::USB;
  }
  if (bus == "SPI") {
    return mab::candleTypes::busTypes_t::SPI;
  }
  return std::nullopt;
}

std::optional<mab::CANdleDatarate_E> HardwareConfig::datarate() const
{
  if (data_rate == "1M") {
    return mab::CANdleDatarate_E::CAN_DATARATE_1M;
  }
  if (data_rate == "2M") {
    return mab::CANdleDatarate_E::CAN_DATARATE_2M;
  }
  if (data_rate == "5M") {
    return mab::CANdleDatarate_E::CAN_DATARATE_5M;
  }
  if (data_rate == "8M") {
    return mab::CANdleDatarate_E::CAN_DATARATE_8M;
  }
  return std::nullopt;
}

std::optional<mab::socketIndex_E> HardwareConfig::power_stage_socket_index() const
{
  switch (power_stage_socket) {
    case 1: return mab::socketIndex_E::SOCKET_1;
    case 2: return mab::socketIndex_E::SOCKET_2;
    case 3: return mab::socketIndex_E::SOCKET_3;
    case 4: return mab::socketIndex_E::SOCKET_4;
    case 5: return mab::socketIndex_E::SOCKET_5;
    case 6: return mab::socketIndex_E::SOCKET_6;
    default: return std::nullopt;
  }
}

}  // namespace mab_rehab
