#pragma once

#include <chrono>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>

#include "candle_types.hpp"
#include "mab_types.hpp"
#include "pds_types.hpp"

namespace mab_rehab
{

/// Bus-wide hardware configuration parsed from the ros2_control URDF tag.
/// Populated once in on_init(); read-only thereafter. No methods on the
/// RT hot path consume this struct directly — it is projected into the
/// individual transport/power modules during on_configure().
struct HardwareConfig
{
  // --- Transport ---
  std::string bus = "SPI";
  std::string data_rate = "5M";
  bool fast_mode = false;

  // --- PDS / power stage ---
  bool use_pds = true;
  int pds_id = 100;
  int power_stage_socket = 2;
  bool use_regular_can_frames = true;
  bool auto_enable_power_stage = true;
  bool disable_power_stage_on_deactivate = true;

  // --- Startup behaviour ---
  bool zero_on_activate = false;
  bool startup_in_idle_mode = true;
  bool startup_reference_enabled = false;
  bool allow_no_connected_drives = true;
  bool save_md_configuration_to_flash = false;
  int hold_position_on_activate_ms = 0;

  // --- Safety / watchdogs ---
  double md_can_watchdog_ms = std::numeric_limits<double>::quiet_NaN();
  bool safety_limits_enabled = false;

  // --- Maintenance ---
  bool maintenance_reload_enabled = true;
  std::string maintenance_restore_controller = "previous";
  int maintenance_service_timeout_ms = 3000;

  /// Parse from the ros2_control <hardware> param map. Throws std::runtime_error
  /// on a malformed value; missing keys fall back to the defaults above.
  static HardwareConfig parse(
    const std::unordered_map<std::string, std::string> & parameters);

  /// Resolve bus/data_rate strings into SDK enums. Returns std::nullopt
  /// on unknown tokens.
  std::optional<mab::candleTypes::busTypes_t> bus_type() const;
  std::optional<mab::CANdleDatarate_E> datarate() const;
  std::optional<mab::socketIndex_E> power_stage_socket_index() const;
};

}  // namespace mab_rehab
