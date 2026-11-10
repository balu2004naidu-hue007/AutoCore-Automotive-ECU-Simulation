#include "autocore/diagnostics.hpp"
#include "autocore/isotp.hpp"
#include "autocore/frame_integrity.hpp"
#include <algorithm>

namespace autocore {
namespace {
std::vector<std::uint8_t> negativeResponse(std::uint8_t service, std::uint8_t code) {
    return {0x7FU, service, code};
}
}

void DiagnosticsEcu::setDtc(std::uint32_t code, const std::string& name,
                            const std::string& description, bool active,
                            std::uint64_t observation_ms) {
    auto it = dtcs_.find(code);
    if (it == dtcs_.end()) {
        it = dtcs_.emplace(code, DiagnosticTroubleCode{code, name, description, false, 0U}).first;
    }

    auto& dtc = it->second;
    if (active && !dtc.active) {
        // A newly detected fault captures the operating context once. Repeated
        // observations do not overwrite the original event snapshot.
        freeze_frames_[code] = DtcFreezeFrame{latest_engine_rpm_, latest_engine_temp_c_,
            latest_vehicle_speed_kph_, latest_battery_mv_, latest_snapshot_ms_};
    }
    dtc.active = active;
    if (active) {
        // Simplified diagnostic debounce: first failed sample is pending; three
        // consecutive failed monitor samples promote the DTC to confirmed.
        // Bit 0 testFailed, bit 1 failedThisOperationCycle, bit 2 pendingDTC,
        // bit 3 confirmedDTC and bit 5 testFailedSinceLastClear.
        const auto last_sample = last_dtc_observation_ms_.find(code);
        if (last_sample == last_dtc_observation_ms_.end() || last_sample->second != observation_ms) {
            if (dtc.consecutive_failure_samples < 3U) ++dtc.consecutive_failure_samples;
            last_dtc_observation_ms_[code] = observation_ms;
        }
        dtc.status = static_cast<std::uint8_t>(dtc.status | 0x23U);
        if (dtc.consecutive_failure_samples >= 3U) {
            dtc.status = static_cast<std::uint8_t>((dtc.status | 0x08U) &
                static_cast<std::uint8_t>(~0x04U));
        } else if ((dtc.status & 0x08U) == 0U) {
            dtc.status = static_cast<std::uint8_t>(dtc.status | 0x04U);
        }
    } else {
        dtc.consecutive_failure_samples = 0U;
        last_dtc_observation_ms_.erase(code);
        // Keep historical confirmed/failed-since-clear information after the
        // monitor recovers; clear only current-failure and pending bits.
        dtc.status = static_cast<std::uint8_t>(dtc.status &
            static_cast<std::uint8_t>(~0x05U));
    }
}

void DiagnosticsEcu::clearDtcInformation() {
    for (auto& entry : dtcs_) {
        entry.second.active = false;
        entry.second.status = 0U;
        entry.second.consecutive_failure_samples = 0U;
    }
    freeze_frames_.clear();
    last_dtc_observation_ms_.clear();
    crc_fault_by_ecu_.clear();
    counter_fault_by_ecu_.clear();
}

void DiagnosticsEcu::setInjectedCrcFault(bool active) {
    injected_crc_fault_ = active;
}

void DiagnosticsEcu::setInjectedCounterFault(bool active) {
    injected_counter_fault_ = active;
}

bool DiagnosticsEcu::validateTelemetryFrame(const CanFrame& frame, std::uint64_t now_ms) {
    const bool crc_valid = verifyTelemetryFrame(frame);
    crc_fault_by_ecu_[frame.id] = !crc_valid;
    const bool any_crc_fault = injected_crc_fault_ || std::any_of(crc_fault_by_ecu_.begin(), crc_fault_by_ecu_.end(),
        [](const auto& entry) { return entry.second; });
    setDtc(0xA00001U, "AC0001", "ECU telemetry CRC integrity failure", any_crc_fault, now_ms);
    if (!crc_valid) {
        // A frame rejected for CRC cannot be used to establish counter continuity.
        // However, the next good frame must become a fresh baseline; otherwise
        // one dropped/corrupted frame creates a misleading second counter DTC.
        counter_baseline_required_[frame.id] = true;
        return false;
    }

    const auto previous = last_rolling_counter_.find(frame.id);
    const bool resynchronize = counter_baseline_required_[frame.id];
    const bool counter_error = !resynchronize && previous != last_rolling_counter_.end() &&
        frame.data[6] != static_cast<std::uint8_t>(previous->second + 1U);
    counter_baseline_required_[frame.id] = false;
    counter_fault_by_ecu_[frame.id] = counter_error;
    const bool any_counter_fault = injected_counter_fault_ || std::any_of(counter_fault_by_ecu_.begin(), counter_fault_by_ecu_.end(),
        [](const auto& entry) { return entry.second; });
    setDtc(0xA00002U, "AC0002", "ECU telemetry rolling-counter sequence error", any_counter_fault, now_ms);
    last_rolling_counter_[frame.id] = frame.data[6];
    return true;
}

void DiagnosticsEcu::observe(const CanFrame& frame, std::uint64_t now_ms) {
    if (frame.id != 0x100U && frame.id != 0x110U && frame.id != 0x120U) return;
    if (!validateTelemetryFrame(frame, now_ms)) return;
    if (frame.id == 0x100U && frame.dlc >= 8U) {
        seen_engine_ = true;
        last_engine_ms_ = now_ms;
        const auto temp = static_cast<std::int16_t>(readU16BE(frame, 2));
        latest_engine_rpm_ = readU16BE(frame, 0U);
        latest_engine_temp_c_ = temp;
        latest_vehicle_speed_kph_ = readU16BE(frame, 4U);
        latest_snapshot_ms_ = now_ms;
        setDtc(0x0217U, "P0217", "Engine coolant temperature too high", temp > 115, now_ms);
    } else if (frame.id == 0x110U && frame.dlc >= 8U) {
        seen_battery_ = true;
        last_battery_ms_ = now_ms;
        const auto voltage = readU16BE(frame, 0);
        latest_battery_mv_ = voltage;
        latest_snapshot_ms_ = now_ms;
        setDtc(0x0562U, "P0562", "System voltage low", voltage < 11000U, now_ms);
    }
}

void DiagnosticsEcu::checkTimeouts(std::uint64_t now_ms) {
    if (seen_engine_ && now_ms >= last_engine_ms_ && now_ms - last_engine_ms_ > 500U) {
        setDtc(0x0100U, "U0100", "Lost communication with engine ECU", true, now_ms);
    } else if (seen_engine_) {
        setDtc(0x0100U, "U0100", "Lost communication with engine ECU", false, now_ms);
    }
    if (seen_battery_ && now_ms >= last_battery_ms_ && now_ms - last_battery_ms_ > 500U) {
        setDtc(0x0111U, "U0111", "Lost communication with battery energy control module", true, now_ms);
    } else if (seen_battery_) {
        setDtc(0x0111U, "U0111", "Lost communication with battery energy control module", false, now_ms);
    }
}

std::vector<std::uint8_t> DiagnosticsEcu::handleUdsPayload(
    const std::vector<std::uint8_t>& request, std::uint16_t engine_rpm,
    std::int16_t engine_temp_c, std::uint16_t battery_mv, std::uint64_t now_ms) {
    if (request.empty()) return negativeResponse(0x00U, 0x13U);
    const std::uint8_t service = request[0];
    switch (service) {
    case 0x10U: { // Diagnostic Session Control
        if (request.size() != 2U) return negativeResponse(service, 0x13U);
        const std::uint8_t session = request[1];
        if (session != 0x01U && session != 0x03U)
            return negativeResponse(service, 0x12U);
        diagnostic_session_ = session;
        return {0x50U, diagnostic_session_, 0x00U, 0x32U, 0x01U, 0xF4U};
    }
    case 0x11U: { // ECU Reset (simulation acknowledgement only)
        if (request.size() != 2U) return negativeResponse(service, 0x13U);
        if (request[1] < 0x01U || request[1] > 0x03U)
            return negativeResponse(service, 0x12U);
        return {0x51U, request[1], 0x00U};
    }
    case 0x22U: { // Read Data By Identifier
        if (request.size() != 3U) return negativeResponse(service, 0x13U);
        const auto did = static_cast<std::uint16_t>(
            (static_cast<std::uint16_t>(request[1]) << 8U) | request[2]);
        std::uint16_t value = 0U;
        if (did == 0xF40CU) value = engine_rpm;
        else if (did == 0xF405U) value = static_cast<std::uint16_t>(engine_temp_c);
        else if (did == 0xF411U) value = battery_mv;
        else return negativeResponse(service, 0x31U);
        return {0x62U, request[1], request[2],
                static_cast<std::uint8_t>((value >> 8U) & 0xFFU),
                static_cast<std::uint8_t>(value & 0xFFU)};
    }
    case 0x14U: { // Clear Diagnostic Information
        if (request.size() != 4U) return negativeResponse(service, 0x13U);
        const std::uint32_t group = (static_cast<std::uint32_t>(request[1]) << 16U) |
            (static_cast<std::uint32_t>(request[2]) << 8U) | request[3];
        if (group != 0xFFFFFFU) return negativeResponse(service, 0x31U);
        clearDtcInformation();
        return {0x54U};
    }
    case 0x19U: { // Read DTC Information
        if (request.size() < 2U) return negativeResponse(service, 0x13U);
        if (request[1] == 0x01U) {
            if (request.size() != 3U) return negativeResponse(service, 0x13U);
            const std::uint8_t status_mask = request[2];
            const auto matching_count = static_cast<std::uint16_t>(std::count_if(
                dtcs_.begin(), dtcs_.end(), [status_mask](const auto& entry) {
                    return (entry.second.status & status_mask) != 0U;
                }));
            return {0x59U, 0x01U, 0x2FU, 0x01U,
                    static_cast<std::uint8_t>((matching_count >> 8U) & 0xFFU),
                    static_cast<std::uint8_t>(matching_count & 0xFFU)};
        }
        if (request[1] == 0x02U) {
            if (request.size() != 3U) return negativeResponse(service, 0x13U);
            const std::uint8_t status_mask = request[2];
            std::vector<std::uint8_t> response{0x59U, 0x02U, 0x2FU};
            for (const auto& entry : dtcs_) {
                const auto& dtc = entry.second;
                const std::uint8_t status = dtc.status;
                if ((status & status_mask) == 0U) continue;
                response.push_back(static_cast<std::uint8_t>((dtc.code >> 16U) & 0xFFU));
                response.push_back(static_cast<std::uint8_t>((dtc.code >> 8U) & 0xFFU));
                response.push_back(static_cast<std::uint8_t>(dtc.code & 0xFFU));
                response.push_back(status);
            }
            return response;
        }
        if (request[1] == 0x04U) {
            if (request.size() != 6U) return negativeResponse(service, 0x13U);
            const std::uint32_t code = (static_cast<std::uint32_t>(request[2]) << 16U) |
                (static_cast<std::uint32_t>(request[3]) << 8U) | request[4];
            const std::uint8_t record_number = request[5];
            const auto dtc = dtcs_.find(code);
            const auto snapshot = freeze_frames_.find(code);
            if (dtc == dtcs_.end() || snapshot == freeze_frames_.end())
                return negativeResponse(service, 0x31U);
            if (record_number != 0x01U)
                return negativeResponse(service, 0x31U);
            const auto& frame = snapshot->second;
            // Snapshot record 0x01: DIDs F40C (RPM), F405 (temperature),
            // F40D (vehicle speed), F411 (battery voltage), and F4A0 (time ms).
            return {0x59U, 0x04U, request[2], request[3], request[4],
                dtc->second.status, record_number, 0x05U, // Five DIDs are present in snapshot record 0x01.
                0xF4U, 0x0CU, static_cast<std::uint8_t>(frame.engine_rpm >> 8U),
                static_cast<std::uint8_t>(frame.engine_rpm & 0xFFU),
                0xF4U, 0x05U, static_cast<std::uint8_t>(frame.engine_temp_c >> 8U),
                static_cast<std::uint8_t>(frame.engine_temp_c & 0xFF),
                0xF4U, 0x0DU, static_cast<std::uint8_t>(frame.vehicle_speed_kph >> 8U),
                static_cast<std::uint8_t>(frame.vehicle_speed_kph & 0xFFU),
                0xF4U, 0x11U, static_cast<std::uint8_t>(frame.battery_mv >> 8U),
                static_cast<std::uint8_t>(frame.battery_mv & 0xFFU),
                0xF4U, 0xA0U,
                static_cast<std::uint8_t>((frame.timestamp_ms >> 24U) & 0xFFU),
                static_cast<std::uint8_t>((frame.timestamp_ms >> 16U) & 0xFFU),
                static_cast<std::uint8_t>((frame.timestamp_ms >> 8U) & 0xFFU),
                static_cast<std::uint8_t>(frame.timestamp_ms & 0xFFU)};
        }
        return negativeResponse(service, 0x12U);
    }
    case 0x3EU: { // Tester Present
        if (request.size() != 2U) return negativeResponse(service, 0x13U);
        const std::uint8_t subfunction = request[1];
        if ((subfunction & 0x7FU) != 0x00U) return negativeResponse(service, 0x12U);
        last_tester_present_ms_ = now_ms;
        if ((subfunction & 0x80U) != 0U) return {}; // Suppress positive response.
        return {0x7EU, 0x00U};
    }
    default:
        return negativeResponse(service, 0x11U);
    }
}

CanFrame DiagnosticsEcu::handleUdsRequest(const CanFrame& request,
                                           std::uint16_t engine_rpm,
                                           std::int16_t engine_temp_c,
                                           std::uint16_t battery_mv,
                                           std::uint64_t now_ms) {
    const auto makeSingleFrame = [](const std::vector<std::uint8_t>& payload) {
        CanFrame response{};
        response.id = 0x7E8U;
        if (payload.empty()) return response;
        const auto frames = isoTpSegment(payload, response.id);
        return frames.front();
    };
    if (request.id != 0x7E0U || request.extended)
        return makeSingleFrame(negativeResponse(0x00U, 0x11U));
    if (request.remote || request.dlc == 0U || request.dlc > request.data.size() ||
        (request.data[0] >> 4U) != 0U)
        return makeSingleFrame(negativeResponse(request.dlc > 1U && request.dlc <= request.data.size()
            ? request.data[1] : 0U, 0x13U));
    const std::size_t length = request.data[0] & 0x0FU;
    if (length == 0U || length > 7U || length + 1U > request.dlc)
        return makeSingleFrame(negativeResponse(request.dlc > 1U ? request.data[1] : 0U, 0x13U));
    std::vector<std::uint8_t> payload(request.data.begin() + 1,
        request.data.begin() + 1 + static_cast<std::ptrdiff_t>(length));
    return makeSingleFrame(handleUdsPayload(payload, engine_rpm, engine_temp_c, battery_mv, now_ms));
}

} // namespace autocore
