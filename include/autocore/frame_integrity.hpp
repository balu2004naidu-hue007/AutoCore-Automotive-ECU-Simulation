#pragma once

#include "autocore/can_frame.hpp"
#include <cstddef>
#include <cstdint>

namespace autocore {

// CRC-8/SAE-J1850: poly 0x1D, init 0xFF, refin/refout false, xorout 0xFF.
// Returns 0 for a null pointer with non-zero length (invalid input).
std::uint8_t crc8SaeJ1850(const std::uint8_t* data, std::size_t length) noexcept;

// AutoCore telemetry convention: bytes 0..5 are signals/reserved, byte 6 is
// an 8-bit rolling counter, and byte 7 is CRC over bytes 0..6.
void sealTelemetryFrame(CanFrame& frame, std::uint8_t counter);
bool verifyTelemetryFrame(const CanFrame& frame) noexcept;

} // namespace autocore
