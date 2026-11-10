#pragma once

#include "autocore/can_frame.hpp"
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace autocore {

struct DiagnosticTroubleCode {
    std::uint32_t code;
    std::string name;
    std::string description;
    bool active{true};
    std::uint8_t status{0x00U};
    std::uint8_t consecutive_failure_samples{0U};
};

struct DtcFreezeFrame {
    std::uint16_t engine_rpm{0U};
    std::int16_t engine_temp_c{0};
    std::uint16_t vehicle_speed_kph{0U};
    std::uint16_t battery_mv{0U};
    std::uint64_t timestamp_ms{0U};
};

class DiagnosticsEcu {
public:
    void observe(const CanFrame& frame, std::uint64_t now_ms);
    void checkTimeouts(std::uint64_t now_ms);
    // Dashboard fault-injection latches keep integrity faults visible until recovery.
    void setInjectedCrcFault(bool active);
    void setInjectedCounterFault(bool active);
    std::vector<std::uint8_t> handleUdsPayload(const std::vector<std::uint8_t>& request,
                                               std::uint16_t engine_rpm,
                                               std::int16_t engine_temp_c,
                                               std::uint16_t battery_mv,
                                               std::uint64_t now_ms);
    CanFrame handleUdsRequest(const CanFrame& request,
                              std::uint16_t engine_rpm,
                              std::int16_t engine_temp_c,
                              std::uint16_t battery_mv,
                              std::uint64_t now_ms);
    const std::map<std::uint32_t, DiagnosticTroubleCode>& dtcs() const noexcept { return dtcs_; }
    std::uint64_t lastTesterPresentMs() const noexcept { return last_tester_present_ms_; }
    const std::map<std::uint32_t, DtcFreezeFrame>& freezeFrames() const noexcept { return freeze_frames_; }

private:
    void clearDtcInformation();
    void setDtc(std::uint32_t code, const std::string& name, const std::string& description,
                bool active, std::uint64_t observation_ms);
    bool validateTelemetryFrame(const CanFrame& frame, std::uint64_t now_ms);
    std::map<std::uint32_t, DiagnosticTroubleCode> dtcs_;
    std::map<std::uint32_t, DtcFreezeFrame> freeze_frames_;
    std::map<std::uint32_t, std::uint64_t> last_dtc_observation_ms_;
    std::map<std::uint32_t, std::uint8_t> last_rolling_counter_;
    // After a CRC-rejected frame, re-establish the next trusted counter baseline
    // instead of reporting a cascading false sequence gap.
    std::map<std::uint32_t, bool> counter_baseline_required_;
    std::map<std::uint32_t, bool> crc_fault_by_ecu_;
    std::map<std::uint32_t, bool> counter_fault_by_ecu_;
    std::uint64_t last_engine_ms_{0};
    std::uint64_t last_battery_ms_{0};
    std::uint64_t last_tester_present_ms_{0};
    std::uint16_t latest_engine_rpm_{0U};
    std::int16_t latest_engine_temp_c_{0};
    std::uint16_t latest_vehicle_speed_kph_{0U};
    std::uint16_t latest_battery_mv_{0U};
    std::uint64_t latest_snapshot_ms_{0U};
    bool injected_crc_fault_{false};
    bool injected_counter_fault_{false};
    bool seen_engine_{false};
    bool seen_battery_{false};
    std::uint8_t diagnostic_session_{0x01};
};

} // namespace autocore
