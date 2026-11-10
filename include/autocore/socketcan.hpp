#pragma once

#include "autocore/can_frame.hpp"

#if defined(__linux__)
#include <cstdint>
#include <string>
#include <optional>
#include <utility>
#include <vector>

namespace autocore {

// Linux SocketCAN transport. This adapter moves raw Classic CAN frames between
// AutoCore and a Linux CAN network interface (for example, a vcan interface).
// It does not configure bitrate or create interfaces; those are OS operations.
class SocketCanInterface {
public:
    SocketCanInterface() = default;
    ~SocketCanInterface();

    SocketCanInterface(const SocketCanInterface&) = delete;
    SocketCanInterface& operator=(const SocketCanInterface&) = delete;

    SocketCanInterface(SocketCanInterface&& other) noexcept;
    SocketCanInterface& operator=(SocketCanInterface&& other) noexcept;

    void open(const std::string& interface_name, bool loopback = true);
    void close() noexcept;
    bool isOpen() const noexcept { return socket_fd_ >= 0; }
    void send(const CanFrame& frame);
    // Returns no frame on timeout. Throws std::runtime_error on transport errors.
    std::optional<CanFrame> receive(std::uint32_t timeout_ms);
    void setFilters(const std::vector<std::pair<std::uint32_t, std::uint32_t>>& filters);

private:
    int socket_fd_{-1};
};

} // namespace autocore
#endif
