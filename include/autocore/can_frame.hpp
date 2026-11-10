#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace autocore {

struct CanFrame {
    std::uint32_t id{0};
    std::uint8_t dlc{0};
    std::array<std::uint8_t, 8> data{};
    bool extended{false};
    bool remote{false};

    void validate() const {
        if (id > (extended ? 0x1FFFFFFFU : 0x7FFU)) {
            throw std::invalid_argument("CAN identifier exceeds selected frame format");
        }
        if (dlc > 8U) {
            throw std::invalid_argument("Classic CAN DLC cannot exceed 8 bytes");
        }
    }
};

inline std::uint16_t readU16BE(const CanFrame& frame, std::size_t offset) {
    // Subtract only after checking offset to avoid size_t overflow on hostile indices.
    if (frame.dlc > frame.data.size() || offset >= frame.dlc ||
        static_cast<std::size_t>(frame.dlc) - offset < 2U)
        throw std::out_of_range("CAN payload read out of range");
    return static_cast<std::uint16_t>((static_cast<std::uint16_t>(frame.data[offset]) << 8U) |
                                      static_cast<std::uint16_t>(frame.data[offset + 1U]));
}

inline void writeU16BE(CanFrame& frame, std::size_t offset, std::uint16_t value) {
    if (offset >= frame.data.size() || frame.data.size() - offset < 2U)
        throw std::out_of_range("CAN payload write out of range");
    frame.data[offset] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
    frame.data[offset + 1U] = static_cast<std::uint8_t>(value & 0xFFU);
}

} // namespace autocore
