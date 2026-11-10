#pragma once

#include <cstdint>
#include <string_view>

namespace autocore {

enum class CanNode : std::uint8_t { Engine = 0, Battery = 1, Body = 2, Diagnostics = 3 };
enum class CanControllerState : std::uint8_t { ErrorActive, ErrorPassive, BusOff };

struct CanControllerSnapshot {
    std::uint16_t transmit_error_counter{0U};
    std::uint16_t receive_error_counter{0U};
    CanControllerState state{CanControllerState::ErrorActive};
    std::uint64_t transmit_attempts{0U};
    std::uint64_t transmit_successes{0U};
    std::uint64_t transmit_errors{0U};
    std::uint64_t receive_successes{0U};
    std::uint64_t receive_errors{0U};
    bool recovery_in_progress{false};
    std::uint16_t recovery_sequences_seen{0U};
    std::uint64_t bus_off_recoveries{0U};
};

class CanController {
public:
    static constexpr std::uint16_t error_passive_threshold = 128U;
    static constexpr std::uint16_t bus_off_threshold = 256U;
    static constexpr std::uint16_t bus_off_recovery_sequences_required = 128U;

    void recordTransmitSuccess() noexcept;
    void recordTransmitError() noexcept;
    void recordReceiveSuccess() noexcept;
    void recordReceiveError() noexcept;

    // Starts the modeled CAN bus-off recovery sequence. The network layer requires
    // all persistent TX error injectors to be disabled before calling this function.
    bool beginBusOffRecovery() noexcept;
    // One call represents one observed 11-recessive-bit recovery opportunity in
    // this discrete educational model. It is not a bit-level bus simulator.
    bool observeBusOffRecoverySequence(bool observed) noexcept;

    void reset() noexcept;
    CanControllerSnapshot snapshot() const noexcept;
    CanControllerState state() const noexcept { return state_; }

private:
    void updateState() noexcept;
    static void incrementSaturating(std::uint64_t& value) noexcept;

    std::uint16_t tec_{0U};
    std::uint16_t rec_{0U};
    CanControllerState state_{CanControllerState::ErrorActive};
    std::uint64_t tx_attempts_{0U};
    std::uint64_t tx_successes_{0U};
    std::uint64_t tx_errors_{0U};
    std::uint64_t rx_successes_{0U};
    std::uint64_t rx_errors_{0U};
    bool recovery_in_progress_{false};
    std::uint16_t recovery_sequences_seen_{0U};
    std::uint64_t bus_off_recoveries_{0U};
};

std::string_view canNodeKey(CanNode node) noexcept;
std::string_view canNodeDisplayName(CanNode node) noexcept;
std::string_view canControllerStateName(CanControllerState state) noexcept;

} // namespace autocore
