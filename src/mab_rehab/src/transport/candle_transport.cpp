#include "mab_rehab/transport/candle_transport.hpp"

#include <exception>
#include <memory>

#include "SPI.hpp"
#include "USB.hpp"
#include "I_communication_interface.hpp"

namespace mab_rehab
{

void CandleTransport::CandleDeleter::operator()(mab::Candle * candle) const
{
  if (candle != nullptr) {
    mab::detachCandle(candle);
  }
}

CandleTransport::~CandleTransport()
{
  disconnect();
}

bool CandleTransport::connect(const HardwareConfig & config, std::string & error_out)
{
  const auto datarate = config.datarate();
  if (!datarate.has_value()) {
    error_out = "Unsupported CAN data rate: " + config.data_rate;
    return false;
  }

  const auto bus_type = config.bus_type();
  if (!bus_type.has_value()) {
    error_out = "Unsupported CANdle bus type: " + config.bus;
    return false;
  }

  std::unique_ptr<mab::I_CommunicationInterface> bus_handle;
  switch (*bus_type) {
    case mab::candleTypes::busTypes_t::USB:
      bus_handle = std::make_unique<mab::USB>(
        mab::Candle::CANDLE_VID, mab::Candle::CANDLE_PID);
      break;
    case mab::candleTypes::busTypes_t::SPI:
      bus_handle = std::make_unique<mab::SPI>();
      break;
    default:
    error_out = "Unhandled bus type: " + config.bus;
    return false;
  }

  try {
    if (bus_handle->connect() != mab::I_CommunicationInterface::Error_t::OK) {
      error_out = "Failed to open bus " + config.bus;
      return false;
    }

    auto raw = std::unique_ptr<mab::Candle, CandleDeleter>(
      new mab::Candle(*datarate, std::move(bus_handle), config.use_regular_can_frames));
    if (raw->init() != mab::candleTypes::Error_t::OK) {
      error_out = "Failed to initialize CANdle on " + config.bus + " at " + config.data_rate;
      return false;
    }

    candle_ = std::move(raw);
    return true;
  } catch (const std::exception & error) {
    error_out = error.what();
    return false;
  }
}

void CandleTransport::disconnect()
{
  candle_.reset();
}

}  // namespace mab_rehab
