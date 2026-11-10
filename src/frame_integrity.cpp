#include "autocore/frame_integrity.hpp"
#include <stdexcept>

namespace autocore {

std::uint8_t crc8SaeJ1850(const std::uint8_t* data, std::size_t length) noexcept {
    // A null pointer is valid only for an empty input. Return the neutral error
    // sentinel for invalid calls rather than dereferencing caller-supplied null.
    if (data == nullptr && length != 0U) return 0U;
    std::uint8_t crc = 0xFFU;
    for (std::size_t i = 0; i < length; ++i) {
        crc = static_cast<std::uint8_t>(crc ^ data[i]);
        for (unsigned bit = 0; bit < 8U; ++bit) {
            const unsigned shifted = static_cast<unsigned>(crc) << 1U;
            crc = (crc & 0x80U) != 0U
                ? static_cast<std::uint8_t>(shifted ^ 0x1DU)
                : static_cast<std::uint8_t>(shifted);
        }
    }
    return static_cast<std::uint8_t>(crc ^ 0xFFU);
}

void sealTelemetryFrame(CanFrame& frame, std::uint8_t counter) {
    if (frame.dlc != 8U || frame.extended || frame.remote)
        throw std::invalid_argument("AutoCore telemetry integrity requires an 8-byte standard data frame");
    frame.data[6] = counter;
    frame.data[7] = crc8SaeJ1850(frame.data.data(), 7U);
}

bool verifyTelemetryFrame(const CanFrame& frame) noexcept {
    return frame.dlc == 8U && !frame.extended && !frame.remote &&
           frame.data[7] == crc8SaeJ1850(frame.data.data(), 7U);
}

} // namespace autocore
