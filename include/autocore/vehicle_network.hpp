#pragma once

#include "autocore/diagnostics.hpp"
#include "autocore/ecus.hpp"
#include "autocore/can_controller.hpp"
#include "autocore/can_frame.hpp"
#include <array>
#include <vector>
#include <cstdint>
#include <optional>

namespace autocore {

class VehicleNetwork {
public:
    void tick(std::uint64_t now_ms);
    CanFrame requestUds(const CanFrame& request, std::uint64_t now_ms);
    std::vector<std::uint8_t> requestUdsPayload(const std::vector<std::uint8_t>& request,
                                               std::uint64_t now_ms);
    // Complete in-process ISO-TP transaction. Flow-control frames are generated
    // by the receiver/segmenter but pacing is not wall-clock simulated here.
    std::vector<CanFrame> requestUdsIsoTp(const std::vector<CanFrame>& request_frames,
                                          std::uint64_t now_ms);

    EngineEcu& engine() noexcept { return engine_; }
    const EngineEcu& engine() const noexcept { return engine_; }
    BatteryEcu& battery() noexcept { return battery_; }
    const BatteryEcu& battery() const noexcept { return battery_; }
    BodyEcu& body() noexcept { return body_; }
    const BodyEcu& body() const noexcept { return body_; }
    DiagnosticsEcu& diagnostics() noexcept { return diagnostics_; }
    const DiagnosticsEcu& diagnostics() const noexcept { return diagnostics_; }

    void setEngineOnline(bool online) noexcept { engine_online_ = online; }
    void setBatteryOnline(bool online) noexcept { battery_online_ = online; }
    bool isNodeOnline(CanNode node) const noexcept;
    const CanController& controller(CanNode node) const noexcept;
    CanControllerSnapshot controllerSnapshot(CanNode node) const noexcept;
    bool txErrorInjectionEnabled(CanNode node) const noexcept;
    bool rxErrorInjectionEnabled(CanNode node) const noexcept;
    void setTxErrorInjection(CanNode node, bool enabled) noexcept;
    void setRxErrorInjection(CanNode node, bool enabled) noexcept;
    bool injectTransmitErrorBurst(CanNode node, std::uint16_t count) noexcept;
    bool injectReceiveErrorBurst(CanNode node, std::uint16_t count) noexcept;
    bool startBusOffRecovery(CanNode node) noexcept;

    std::uint64_t canFramesTransmitted() const noexcept { return bus_.transmittedCount(); }
    const std::vector<CanFrame>& lastFrames() const noexcept { return last_frames_; }
    void injectCorruptedEngineTelemetry(std::uint64_t now_ms);
    void injectRollingCounterError(std::uint64_t now_ms);

private:
    static constexpr std::size_t nodeIndex(CanNode node) noexcept {
        return static_cast<std::size_t>(node);
    }
    static std::optional<CanNode> nodeForFrame(const CanFrame& frame) noexcept;
    void recordFrameExchange(const CanFrame& frame);
    void observeRecoveryOpportunities(std::uint64_t now_ms) noexcept;

    CanBus bus_;
    EngineEcu engine_;
    BatteryEcu battery_;
    BodyEcu body_;
    DiagnosticsEcu diagnostics_;
    std::array<CanController, 4U> controllers_{};
    std::array<bool, 4U> tx_error_injection_{};
    std::array<bool, 4U> rx_error_injection_{};
    bool engine_online_{true};
    bool battery_online_{true};
    bool scheduler_started_{false};
    bool recovery_observation_started_{false};
    std::uint64_t last_recovery_observation_ms_{0U};
    std::uint64_t next_engine_ms_{0};
    std::uint64_t next_battery_ms_{0};
    std::uint64_t next_body_ms_{0};
    std::vector<CanFrame> last_frames_;
    static constexpr std::uint64_t telemetry_period_ms_{100U};
};

} // namespace autocore
