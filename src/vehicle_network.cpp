#include "autocore/vehicle_network.hpp"
#include "autocore/isotp.hpp"
#include "autocore/frame_integrity.hpp"
#include <algorithm>
#include <stdexcept>
#include <limits>

namespace autocore {

std::optional<CanNode> VehicleNetwork::nodeForFrame(const CanFrame& frame) noexcept {
    if (frame.extended) return std::nullopt;
    switch (frame.id) {
        case 0x100U: return CanNode::Engine;
        case 0x110U: return CanNode::Battery;
        case 0x120U: return CanNode::Body;
        case 0x7E8U: return CanNode::Diagnostics;
        default: return std::nullopt;
    }
}

void VehicleNetwork::recordFrameExchange(const CanFrame& frame) {
    const auto sender = nodeForFrame(frame);
    if (sender) controllers_[nodeIndex(*sender)].recordTransmitSuccess();

    for (std::size_t index = 0U; index < controllers_.size(); ++index) {
        const auto node = static_cast<CanNode>(index);
        if (sender && node == *sender) continue;
        if (controllers_[index].state() == CanControllerState::BusOff) continue;
        if (rx_error_injection_[index]) controllers_[index].recordReceiveError();
        else controllers_[index].recordReceiveSuccess();
    }
}

void VehicleNetwork::observeRecoveryOpportunities(std::uint64_t now_ms) noexcept {
    if (recovery_observation_started_ && now_ms <= last_recovery_observation_ms_) return;
    recovery_observation_started_ = true;
    last_recovery_observation_ms_ = now_ms;
    // One distinct scheduler-cycle observation represents an eligible
    // 11-recessive-bit idle sequence in this discrete model. A persistent TX
    // error injector on any node prevents recovery progress.
    const bool no_active_tx_faults = std::none_of(tx_error_injection_.begin(), tx_error_injection_.end(),
        [](bool enabled) { return enabled; });
    for (auto& controller : controllers_) {
        if (!controller.snapshot().recovery_in_progress) continue;
        controller.observeBusOffRecoverySequence(no_active_tx_faults);
    }
}

void VehicleNetwork::tick(std::uint64_t now_ms) {
    if (!scheduler_started_) {
        scheduler_started_ = true;
        next_engine_ms_ = now_ms;
        next_battery_ms_ = now_ms;
        next_body_ms_ = now_ms;
    }

    const auto scheduleNext = [now_ms](std::uint64_t& next_due) {
        next_due = now_ms > std::numeric_limits<std::uint64_t>::max() - telemetry_period_ms_
            ? std::numeric_limits<std::uint64_t>::max() : now_ms + telemetry_period_ms_;
    };

    if (now_ms >= next_engine_ms_) {
        auto& controller = controllers_[nodeIndex(CanNode::Engine)];
        if (engine_online_ && controller.state() != CanControllerState::BusOff) {
            if (tx_error_injection_[nodeIndex(CanNode::Engine)]) controller.recordTransmitError();
            else engine_.publish(bus_);
        }
        scheduleNext(next_engine_ms_);
    }
    if (now_ms >= next_battery_ms_) {
        auto& controller = controllers_[nodeIndex(CanNode::Battery)];
        if (battery_online_ && controller.state() != CanControllerState::BusOff) {
            if (tx_error_injection_[nodeIndex(CanNode::Battery)]) controller.recordTransmitError();
            else battery_.publish(bus_);
        }
        scheduleNext(next_battery_ms_);
    }
    if (now_ms >= next_body_ms_) {
        auto& controller = controllers_[nodeIndex(CanNode::Body)];
        if (controller.state() != CanControllerState::BusOff) {
            if (tx_error_injection_[nodeIndex(CanNode::Body)]) controller.recordTransmitError();
            else body_.publish(bus_);
        }
        scheduleNext(next_body_ms_);
    }

    last_frames_ = bus_.arbitrate();
    for (const auto& frame : last_frames_) {
        recordFrameExchange(frame);
        diagnostics_.observe(frame, now_ms);
    }
    diagnostics_.checkTimeouts(now_ms);
    observeRecoveryOpportunities(now_ms);
}

bool VehicleNetwork::isNodeOnline(CanNode node) const noexcept {
    switch (node) {
        case CanNode::Engine:
            return engine_online_ && controllers_[nodeIndex(node)].state() != CanControllerState::BusOff;
        case CanNode::Battery:
            return battery_online_ && controllers_[nodeIndex(node)].state() != CanControllerState::BusOff;
        case CanNode::Body:
        case CanNode::Diagnostics:
            return controllers_[nodeIndex(node)].state() != CanControllerState::BusOff;
    }
    return false;
}

const CanController& VehicleNetwork::controller(CanNode node) const noexcept {
    return controllers_[nodeIndex(node)];
}

CanControllerSnapshot VehicleNetwork::controllerSnapshot(CanNode node) const noexcept {
    return controller(node).snapshot();
}

bool VehicleNetwork::txErrorInjectionEnabled(CanNode node) const noexcept {
    return tx_error_injection_[nodeIndex(node)];
}

bool VehicleNetwork::rxErrorInjectionEnabled(CanNode node) const noexcept {
    return rx_error_injection_[nodeIndex(node)];
}

void VehicleNetwork::setTxErrorInjection(CanNode node, bool enabled) noexcept {
    tx_error_injection_[nodeIndex(node)] = enabled;
}

void VehicleNetwork::setRxErrorInjection(CanNode node, bool enabled) noexcept {
    rx_error_injection_[nodeIndex(node)] = enabled;
}

bool VehicleNetwork::injectTransmitErrorBurst(CanNode node, std::uint16_t count) noexcept {
    if (count == 0U || count > 64U) return false;
    auto& controller = controllers_[nodeIndex(node)];
    if (controller.state() == CanControllerState::BusOff) return false;
    for (std::uint16_t i = 0U; i < count; ++i) controller.recordTransmitError();
    return true;
}

bool VehicleNetwork::injectReceiveErrorBurst(CanNode node, std::uint16_t count) noexcept {
    if (count == 0U || count > 255U) return false;
    auto& controller = controllers_[nodeIndex(node)];
    if (controller.state() == CanControllerState::BusOff) return false;
    for (std::uint16_t i = 0U; i < count; ++i) controller.recordReceiveError();
    return true;
}

bool VehicleNetwork::startBusOffRecovery(CanNode node) noexcept {
    const bool no_active_tx_faults = std::none_of(tx_error_injection_.begin(), tx_error_injection_.end(),
        [](bool enabled) { return enabled; });
    if (!no_active_tx_faults) return false;
    return controllers_[nodeIndex(node)].beginBusOffRecovery();
}

void VehicleNetwork::injectCorruptedEngineTelemetry(std::uint64_t now_ms) {
    auto& controller = controllers_[nodeIndex(CanNode::Engine)];
    if (controller.state() == CanControllerState::BusOff) return;
    engine_.publish(bus_);
    last_frames_ = bus_.arbitrate();
    for (auto& frame : last_frames_) {
        if (frame.id == 0x100U && frame.dlc == 8U) {
            frame.data[7] ^= 0x5AU;
            recordFrameExchange(frame);
            diagnostics_.observe(frame, now_ms);
        }
    }
    diagnostics_.checkTimeouts(now_ms);
}

void VehicleNetwork::injectRollingCounterError(std::uint64_t now_ms) {
    auto& controller = controllers_[nodeIndex(CanNode::Engine)];
    if (controller.state() == CanControllerState::BusOff) return;
    engine_.publish(bus_);
    last_frames_ = bus_.arbitrate();
    for (auto& frame : last_frames_) {
        if (frame.id == 0x100U && frame.dlc == 8U) {
            const auto corrupted_counter = static_cast<std::uint8_t>(frame.data[6] + 5U);
            sealTelemetryFrame(frame, corrupted_counter);
            recordFrameExchange(frame);
            diagnostics_.observe(frame, now_ms);
        }
    }
    diagnostics_.checkTimeouts(now_ms);
}

CanFrame VehicleNetwork::requestUds(const CanFrame& request, std::uint64_t now_ms) {
    auto& diagnostic_controller = controllers_[nodeIndex(CanNode::Diagnostics)];
    if (diagnostic_controller.state() == CanControllerState::BusOff) return CanFrame{};
    bus_.transmit(request);
    const auto requests = bus_.arbitrate();
    if (requests.empty()) return CanFrame{};
    if (tx_error_injection_[nodeIndex(CanNode::Diagnostics)]) {
        diagnostic_controller.recordTransmitError();
        return CanFrame{};
    }
    const auto response = diagnostics_.handleUdsRequest(requests.front(), engine_.rpm(),
        engine_.temperature(), battery_.voltageMv(), now_ms);
    if (response.dlc != 0U) {
        diagnostic_controller.recordTransmitSuccess();
        bus_.transmit(response);
    }
    const auto responses = bus_.arbitrate();
    return responses.empty() ? CanFrame{} : responses.front();
}

std::vector<std::uint8_t> VehicleNetwork::requestUdsPayload(
    const std::vector<std::uint8_t>& request, std::uint64_t now_ms) {
    auto& diagnostic_controller = controllers_[nodeIndex(CanNode::Diagnostics)];
    if (diagnostic_controller.state() == CanControllerState::BusOff) return {};
    if (tx_error_injection_[nodeIndex(CanNode::Diagnostics)]) {
        diagnostic_controller.recordTransmitError();
        return {};
    }
    auto response = diagnostics_.handleUdsPayload(request, engine_.rpm(), engine_.temperature(),
                                                   battery_.voltageMv(), now_ms);
    if (!response.empty()) diagnostic_controller.recordTransmitSuccess();
    return response;
}

std::vector<CanFrame> VehicleNetwork::requestUdsIsoTp(
    const std::vector<CanFrame>& request_frames, std::uint64_t now_ms) {
    IsoTpReceiver receiver(0x7E0U, 0x7E8U);
    std::vector<std::uint8_t> request_payload;
    bool complete = false;
    for (const auto& frame : request_frames) {
        const auto result = receiver.consume(frame, now_ms);
        if (result.status == IsoTpStatus::Error || result.status == IsoTpStatus::TimedOut)
            throw std::invalid_argument("Invalid or timed-out ISO-TP diagnostic request");
        if (result.status == IsoTpStatus::Complete) {
            request_payload = result.payload;
            complete = true;
            break;
        }
    }
    if (!complete) throw std::invalid_argument("Incomplete ISO-TP diagnostic request");
    const auto response_payload = requestUdsPayload(request_payload, now_ms);
    if (response_payload.empty()) return {};
    return isoTpSegment(response_payload, 0x7E8U);
}

} // namespace autocore
