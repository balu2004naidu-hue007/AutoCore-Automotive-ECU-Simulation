#include "autocore/can_controller.hpp"

#include <limits>

namespace autocore {

void CanController::incrementSaturating(std::uint64_t& value) noexcept {
    if (value != std::numeric_limits<std::uint64_t>::max()) ++value;
}

void CanController::updateState() noexcept {
    if (tec_ >= bus_off_threshold) {
        state_ = CanControllerState::BusOff;
        recovery_in_progress_ = false;
        recovery_sequences_seen_ = 0U;
    } else if (tec_ >= error_passive_threshold || rec_ >= error_passive_threshold) {
        state_ = CanControllerState::ErrorPassive;
    } else {
        state_ = CanControllerState::ErrorActive;
    }
}

void CanController::recordTransmitSuccess() noexcept {
    if (state_ == CanControllerState::BusOff) return;
    incrementSaturating(tx_attempts_);
    incrementSaturating(tx_successes_);
    if (tec_ > 0U) --tec_;
    updateState();
}

void CanController::recordTransmitError() noexcept {
    if (state_ == CanControllerState::BusOff) return;
    incrementSaturating(tx_attempts_);
    incrementSaturating(tx_errors_);
    const auto next = static_cast<unsigned>(tec_) + 8U;
    tec_ = static_cast<std::uint16_t>(next >= bus_off_threshold ? bus_off_threshold : next);
    updateState();
}

void CanController::recordReceiveSuccess() noexcept {
    if (state_ == CanControllerState::BusOff) return;
    incrementSaturating(rx_successes_);
    if (rec_ > error_passive_threshold - 1U) rec_ = error_passive_threshold - 1U;
    else if (rec_ > 0U) --rec_;
    updateState();
}

void CanController::recordReceiveError() noexcept {
    if (state_ == CanControllerState::BusOff) return;
    incrementSaturating(rx_errors_);
    if (rec_ < 255U) ++rec_;
    updateState();
}

bool CanController::beginBusOffRecovery() noexcept {
    if (state_ != CanControllerState::BusOff || recovery_in_progress_) return false;
    recovery_in_progress_ = true;
    recovery_sequences_seen_ = 0U;
    return true;
}

bool CanController::observeBusOffRecoverySequence(bool observed) noexcept {
    if (!recovery_in_progress_ || state_ != CanControllerState::BusOff || !observed)
        return false;
    if (recovery_sequences_seen_ < bus_off_recovery_sequences_required)
        ++recovery_sequences_seen_;
    if (recovery_sequences_seen_ < bus_off_recovery_sequences_required) return false;

    tec_ = 0U;
    rec_ = 0U;
    state_ = CanControllerState::ErrorActive;
    recovery_in_progress_ = false;
    recovery_sequences_seen_ = bus_off_recovery_sequences_required;
    incrementSaturating(bus_off_recoveries_);
    return true;
}

void CanController::reset() noexcept {
    tec_ = 0U;
    rec_ = 0U;
    state_ = CanControllerState::ErrorActive;
    tx_attempts_ = 0U;
    tx_successes_ = 0U;
    tx_errors_ = 0U;
    rx_successes_ = 0U;
    rx_errors_ = 0U;
    recovery_in_progress_ = false;
    recovery_sequences_seen_ = 0U;
    bus_off_recoveries_ = 0U;
}

CanControllerSnapshot CanController::snapshot() const noexcept {
    return CanControllerSnapshot{tec_, rec_, state_, tx_attempts_, tx_successes_, tx_errors_,
        rx_successes_, rx_errors_, recovery_in_progress_, recovery_sequences_seen_,
        bus_off_recoveries_};
}

std::string_view canNodeKey(CanNode node) noexcept {
    switch (node) {
        case CanNode::Engine: return "ENGINE";
        case CanNode::Battery: return "BATTERY";
        case CanNode::Body: return "BODY";
        case CanNode::Diagnostics: return "DIAGNOSTICS";
    }
    return "UNKNOWN";
}

std::string_view canNodeDisplayName(CanNode node) noexcept {
    switch (node) {
        case CanNode::Engine: return "Engine ECU";
        case CanNode::Battery: return "Battery ECU";
        case CanNode::Body: return "Body ECU";
        case CanNode::Diagnostics: return "Diagnostics ECU";
    }
    return "Unknown ECU";
}

std::string_view canControllerStateName(CanControllerState state) noexcept {
    switch (state) {
        case CanControllerState::ErrorActive: return "ERROR_ACTIVE";
        case CanControllerState::ErrorPassive: return "ERROR_PASSIVE";
        case CanControllerState::BusOff: return "BUS_OFF";
    }
    return "UNKNOWN";
}

} // namespace autocore
