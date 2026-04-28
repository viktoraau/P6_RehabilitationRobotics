#pragma once

#include <memory>
#include <string>

#include "candle.hpp"
#include "pds.hpp"
#include "power_stage.hpp"

#include "mab_rehab/config/hardware_config.hpp"

namespace mab_rehab
{

/// Owns the MAB PDS + power stage. Cold-path setup and lifecycle actions,
/// plus the RT-safe bus-voltage read used by telemetry publication.
class PdsManager
{
public:
  PdsManager() = default;

  /// Attach to the PDS on the given Candle bus. init_attempts is the
  /// cold-path retry budget (typically 5). Returns true on success.
  bool setup(const HardwareConfig & config, mab::Candle * candle, std::string & error_out);

  /// Disable the power stage and drop SDK handles. Safe to call idempotently.
  void teardown();

  /// Enable / disable the power stage output. Cold-path.
  bool enable_power_stage();
  bool disable_power_stage();

  /// Read DC bus voltage. RT-safe: one SPI transaction, no retry, no log.
  /// Returns std::nullopt on transport failure; caller holds last-known value.
  std::optional<double> read_bus_voltage() noexcept;

  bool is_power_stage_exported() const { return power_stage_exported_; }

private:
  HardwareConfig config_{};
  mab::Candle * candle_{nullptr};
  std::unique_ptr<mab::Pds> pds_;
  std::shared_ptr<mab::PowerStage> power_stage_;
  bool power_stage_exported_{false};
};

}  // namespace mab_rehab
