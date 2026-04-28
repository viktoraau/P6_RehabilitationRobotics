#include "mab_rehab/power/pds_manager.hpp"

#include <chrono>
#include <thread>

namespace mab_rehab
{

namespace
{
constexpr int kPdsInitAttempts = 5;
constexpr auto kPdsRetryDelay = std::chrono::milliseconds(150);
}  // namespace

bool PdsManager::setup(
  const HardwareConfig & config, mab::Candle * candle, std::string & error_out)
{
  config_ = config;
  candle_ = candle;

  if (!config_.use_pds) {
    return true;
  }

  const auto socket = config_.power_stage_socket_index();
  if (!socket.has_value()) {
    error_out = "Invalid power_stage_socket value: " +
      std::to_string(config_.power_stage_socket);
    return false;
  }

  bool module_probe_succeeded = false;
  bool pds_liveness_confirmed = false;
  u32 bus_voltage_mv = 0;

  for (int attempt = 1; attempt <= kPdsInitAttempts; ++attempt) {
    pds_ = std::make_unique<mab::Pds>(static_cast<u16>(config_.pds_id), candle_);
    if (pds_->init() == mab::PdsModule::error_E::OK) {
      module_probe_succeeded = true;
      break;
    }

    if (pds_->getBusVoltage(bus_voltage_mv) == mab::PdsModule::error_E::OK) {
      pds_liveness_confirmed = true;
      break;
    }

    std::this_thread::sleep_for(kPdsRetryDelay);
  }

  if (module_probe_succeeded) {
    power_stage_ = pds_->attachPowerStage(*socket);
    if (!power_stage_) {
      error_out = "Failed to attach power stage on socket " +
        std::to_string(config_.power_stage_socket);
      return false;
    }
  } else if (pds_liveness_confirmed) {
    auto pds_can_id = std::make_shared<u16>(static_cast<u16>(config_.pds_id));
    power_stage_ = std::make_shared<mab::PowerStage>(*socket, candle_, pds_can_id);
  } else {
    error_out = "Failed to initialize PDS " + std::to_string(config_.pds_id);
    return false;
  }

  power_stage_exported_ = true;

  if (config_.auto_enable_power_stage) {
    if (!enable_power_stage()) {
      error_out = "Failed to enable the power stage on socket " +
        std::to_string(config_.power_stage_socket);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }

  return true;
}

void PdsManager::teardown()
{
  if (power_stage_ && config_.disable_power_stage_on_deactivate) {
    disable_power_stage();
  }
  power_stage_.reset();
  pds_.reset();
  candle_ = nullptr;
  power_stage_exported_ = false;
}

bool PdsManager::enable_power_stage()
{
  if (!power_stage_) { return false; }
  return power_stage_->enable() == mab::PdsModule::error_E::OK;
}

bool PdsManager::disable_power_stage()
{
  if (!power_stage_) { return false; }
  return power_stage_->disable() == mab::PdsModule::error_E::OK;
}

std::optional<double> PdsManager::read_bus_voltage() noexcept
{
  if (!pds_) {
    return std::nullopt;
  }
  u32 bus_voltage_mv = 0;
  if (pds_->getBusVoltage(bus_voltage_mv) != mab::PdsModule::error_E::OK) {
    return std::nullopt;
  }
  return static_cast<double>(bus_voltage_mv);
}

}  // namespace mab_rehab
