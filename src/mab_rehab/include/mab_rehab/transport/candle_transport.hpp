#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "candle.hpp"
#include "candle_types.hpp"

#include "mab_rehab/config/hardware_config.hpp"

namespace mab_rehab
{

/// Single-owner wrapper around mab::Candle. Cold-path API only —
/// the RT thread never calls into this class. connect() / disconnect()
/// are the only allocation points for bus handles.
class CandleTransport
{
public:
  CandleTransport() = default;
  ~CandleTransport();

  CandleTransport(const CandleTransport &) = delete;
  CandleTransport & operator=(const CandleTransport &) = delete;

  /// Open the bus with the resolved configuration. On success the
  /// returned pointer is non-null until disconnect() is called.
  /// On failure, error_out is populated and candle() stays null.
  bool connect(const HardwareConfig & config, std::string & error_out);

  /// Close the bus. Safe to call when not connected.
  void disconnect();

  /// Raw Candle handle for SDK calls. Valid only between connect() and
  /// disconnect(). Nullptr otherwise.
  mab::Candle * candle() { return candle_.get(); }
  const mab::Candle * candle() const { return candle_.get(); }

  bool is_connected() const { return static_cast<bool>(candle_); }

private:
  struct CandleDeleter
  {
    void operator()(mab::Candle * candle) const;
  };

  std::unique_ptr<mab::Candle, CandleDeleter> candle_;
};

}  // namespace mab_rehab
