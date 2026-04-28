// md_config_uploader.cpp — Upload a .cfg file to an MD drive.
//
// Ported verbatim from mab_ros2_control with only namespace changes.
// Mirrors the upload logic from CANdle SDK's candletool:
//   candletool/src/md_cli.cpp (lines 520-577 and 1252-1342)

#include "mab_rehab/maintenance/md_config_uploader.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>
#include <variant>

// Provide mab::trim() (declared in utilities.hpp, defined in candletool's
// utilities.cpp which is not linked). Required by md_cfg_map.hpp inlines.
namespace mab
{
std::string trim(const std::string_view s)
{
  auto start = std::find_if_not(s.begin(), s.end(), ::isspace);
  auto end = std::find_if_not(s.rbegin(), s.rend(), ::isspace).base();
  return (start < end) ? std::string(start, end) : "";
}
}  // namespace mab

#include "md_cfg_map.hpp"
#include "mini/ini.h"

namespace mab_rehab
{

namespace
{

bool register_write_by_address(
  mab::MD & md, uint16_t address, const std::string & value)
{
  const std::string trimmed = mab::trim(value);

  mab::MDRegisters_S regs;
  std::variant<int64_t, float, std::string> reg_value;

  if (trimmed.find_first_not_of("-0123456789.f") == std::string::npos && !trimmed.empty()) {
    if (trimmed.find('.') != std::string::npos) {
      reg_value = std::stof(trimmed);
    } else {
      reg_value = std::stoll(trimmed);
    }
  } else {
    reg_value = trimmed;
  }

  bool found_register = false;
  bool write_ok = false;

  auto write_if_match = [&]<typename T>(mab::MDRegisterEntry_S<T> reg) {
      if (reg.m_regAddress != address) {
        return;
      }
      found_register = true;

      if constexpr (std::is_arithmetic_v<T>) {
        if (std::holds_alternative<int64_t>(reg_value)) {
          reg.value = static_cast<T>(std::get<int64_t>(reg_value));
        } else if (std::holds_alternative<float>(reg_value)) {
          reg.value = static_cast<T>(std::get<float>(reg_value));
        } else {
          return;
        }
        write_ok = (md.writeRegisters(reg) == mab::MD::Error_t::OK);
      } else if constexpr (std::is_same_v<std::decay_t<T>, char *>) {
        if (!std::holds_alternative<std::string>(reg_value)) {
          return;
        }
        const auto & str = std::get<std::string>(reg_value);
        if (str.length() > sizeof(reg.value) + 1) {
          return;
        }
        std::copy(str.data(), str.data() + str.length(), reg.value);
        write_ok = (md.writeRegisters(reg) == mab::MD::Error_t::OK);
      }
    };

  regs.forEachRegister(write_if_match);

  return found_register && write_ok;
}

}  // namespace

bool upload_md_config(
  mab::MD & md, const std::string & config_path, std::string & error_out,
  const bool save_to_flash)
{
  std::string resolved_path = config_path;
  if (resolved_path.find('/') == std::string::npos) {
#ifdef DEFAULT_CANDLETOOL_CONFIG_DIR
    resolved_path = std::string(DEFAULT_CANDLETOOL_CONFIG_DIR) + resolved_path;
#else
    resolved_path = "/etc/candletool/config/motors/" + resolved_path;
#endif
  }

  mINI::INIFile config_file(resolved_path);
  mINI::INIStructure ini;
  if (!config_file.read(ini)) {
    error_out = "Could not read configuration file: " + resolved_path;
    return false;
  }

  mab::MDConfigMap cfg_map;
  for (auto & [address, cfg_element] : cfg_map.m_map) {
    const auto & section = cfg_element.m_tomlSection;
    const auto & key = cfg_element.m_tomlKey;

    auto it = ini[std::string(section)][std::string(key)];
    if (it.empty()) {
      continue;
    }

    if (!cfg_element.setFromReadable(it)) {
      error_out = "Could not parse value for [" + std::string(section) + "] " +
        std::string(key) + " = '" + it + "'";
      return false;
    }

    if (!register_write_by_address(md, address, cfg_element.m_value)) {
      std::ostringstream addr_hex;
      addr_hex << "0x" << std::hex << address;
      error_out = "Failed to write register " + addr_hex.str() +
        " ([" + std::string(section) + "] " + std::string(key) + ")";
      return false;
    }
  }

  if (save_to_flash) {
    if (md.save() != mab::MD::Error_t::OK) {
      error_out = "Could not save configuration to flash";
      return false;
    }
  }

  return true;
}

}  // namespace mab_rehab
