#include "autocore/vehicle_network.hpp"
#include "autocore/isotp.hpp"
#include "autocore/frame_integrity.hpp"
#include "autocore/can_message_database.hpp"
#include "autocore/can_controller.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <utility>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace {
using namespace autocore;

std::string quote(const std::string& value) {
    std::ostringstream out;
    out << '"';
    for (char raw_ch : value) {
        const auto ch = static_cast<unsigned char>(raw_ch);
        switch (ch) {
            case '"': out << "\\\""; break;
            case '\\': out << "\\\\"; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default:
                if (ch < 0x20U) out << ' ';
                else out << static_cast<char>(ch);
        }
    }
    out << '"';
    return out.str();
}

std::string hexId(std::uint32_t value, int width = 3) {
    std::ostringstream out;
    out << "0x" << std::uppercase << std::hex << std::setw(width)
        << std::setfill('0') << value;
    return out.str();
}

std::string bytesHex(const CanFrame& frame) {
    std::ostringstream out;
    for (std::size_t i = 0; i < frame.dlc; ++i) {
        if (i) out << ' ';
        out << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<unsigned>(frame.data[i]);
    }
    return out.str();
}

std::string payloadHex(const std::vector<std::uint8_t>& bytes) {
    std::ostringstream out;
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        if (i) out << ' ';
        out << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<unsigned>(bytes[i]);
    }
    return out.str();
}

std::optional<CanNode> parseCanNode(std::string_view value) {
    if (value == "ENGINE") return CanNode::Engine;
    if (value == "BATTERY") return CanNode::Battery;
    if (value == "BODY") return CanNode::Body;
    if (value == "DIAGNOSTICS") return CanNode::Diagnostics;
    return std::nullopt;
}

std::vector<std::string> splitColonFields(const std::string& value) {
    std::vector<std::string> fields;
    std::size_t start = 0U;
    while (start <= value.size()) {
        const auto end = value.find(':', start);
        fields.push_back(value.substr(start, end == std::string::npos ? std::string::npos : end - start));
        if (end == std::string::npos) break;
        start = end + 1U;
    }
    return fields;
}

bool parseBoundedCount(const std::string& text, std::uint16_t minimum,
                       std::uint16_t maximum, std::uint16_t& result) {
    if (text.empty()) return false;
    unsigned int parsed_value = 0U;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), parsed_value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
        parsed_value < minimum || parsed_value > maximum) return false;
    result = static_cast<std::uint16_t>(parsed_value);
    return true;
}

constexpr std::array<CanNode, 4U> allCanNodes{{
    CanNode::Engine, CanNode::Battery, CanNode::Body, CanNode::Diagnostics
}};

struct Sample {
    std::uint64_t time_ms{};
    std::uint16_t rpm{};
    std::int16_t temp{};
    std::uint16_t speed{};
    double voltage{};
    double current{};
};

struct FrameRecord {
    std::uint64_t time_ms{};
    CanFrame frame{};
};

struct Event {
    std::uint64_t time_ms{};
    std::string level;
    std::string message;
};

class Simulation {
public:
    explicit Simulation(CanMessageDatabase database) : message_database_(std::move(database)) { reset(); }

    void reset() {
        vehicle_ = VehicleNetwork{};
        running_ = true;
        time_ms_ = 0;
        phase_ = 0.0;
        fault_overtemp_ = false;
        fault_low_voltage_ = false;
        fault_engine_offline_ = false;
        fault_battery_offline_ = false;
        fault_crc_ = false;
        fault_counter_ = false;
        can_filter_active_ = false;
        can_filter_id_ = 0U;
        can_filter_mask_ = 0U;
        can_filter_accepted_ = 0U;
        can_filter_rejected_ = 0U;
        throttle_ = 35;
        vehicle_.engine().setRpm(1150U);
        vehicle_.engine().setTemperature(90);
        vehicle_.engine().setSpeed(18U);
        vehicle_.battery().setVoltageMv(12500U);
        vehicle_.battery().setCurrentMa(1450);
        vehicle_.battery().setTemperature(29);
        vehicle_.body().setHeadlights(true);
        vehicle_.body().setDoorOpen(false);
        vehicle_.tick(time_ms_);
        for (std::size_t i = 0U; i < allCanNodes.size(); ++i)
            last_controller_states_[i] = vehicle_.controller(allCanNodes[i]).state();
        samples_.clear();
        frames_.clear();
        events_.clear();
        last_uds_request_.clear();
        last_uds_response_.clear();
        log("INFO", "Simulation initialized; four logical ECUs online");
        capture();
        captureFrames();
        last_dtc_state_.clear();
    }

    void advance() {
        if (!running_) return;
        time_ms_ += 100U;
        phase_ += 0.1;
        const double wave = std::sin(phase_ * 0.72);
        const auto rpm = static_cast<std::uint16_t>(
            std::clamp(1250.0 + 720.0 * wave + throttle_ * 7.0, 650.0, 5200.0));
        const auto speed = static_cast<std::uint16_t>(
            std::clamp(42.0 + 24.0 * std::sin(phase_ * 0.24), 0.0, 180.0));
        const auto temperature = static_cast<std::int16_t>(
            std::clamp(89.0 + 3.4 * std::sin(phase_ * 0.055), 70.0, 110.0));
        const auto voltage = static_cast<std::uint16_t>(
            std::clamp(12520.0 + 95.0 * std::sin(phase_ * 0.19) - throttle_ * 1.5,
                       9000.0, 15000.0));
        const auto current = static_cast<std::int16_t>(
            std::clamp(1100.0 + throttle_ * 24.0 + 260.0 * std::sin(phase_ * 0.41),
                       -5000.0, 12000.0));

        if (!fault_engine_offline_) {
            vehicle_.engine().setRpm(rpm);
            vehicle_.engine().setSpeed(speed);
            vehicle_.engine().setTemperature(fault_overtemp_ ? 126 : temperature);
        }
        if (!fault_battery_offline_) {
            vehicle_.battery().setVoltageMv(fault_low_voltage_ ? 10450U : voltage);
            vehicle_.battery().setCurrentMa(current);
            vehicle_.battery().setTemperature(static_cast<std::int16_t>(29.0 + 2.0 * std::sin(phase_ * 0.13)));
        }
        vehicle_.setEngineOnline(!fault_engine_offline_);
        vehicle_.setBatteryOnline(!fault_battery_offline_);
        vehicle_.tick(time_ms_);
        detectControllerTransitions();
        capture();
        captureFrames();
        detectDtcTransitions();
    }

    void execute(const std::string& command) {
        if (command == "START") {
            if (!running_) { running_ = true; log("INFO", "Simulation resumed"); }
        } else if (command == "PAUSE") {
            if (running_) { running_ = false; log("WARN", "Simulation paused by operator"); }
        } else if (command == "RESET") {
            reset();
        } else if (command == "FAULT:OVER_TEMP") {
            resumeForScenario();
            fault_overtemp_ = true; log("FAULT", "Engine coolant overtemperature injected (126 C)");
        } else if (command == "FAULT:LOW_VOLTAGE") {
            resumeForScenario();
            fault_low_voltage_ = true; log("FAULT", "Battery undervoltage injected (10.45 V)");
        } else if (command == "FAULT:SENSOR_DEMO") {
            resumeForScenario();
            fault_overtemp_ = true;
            fault_low_voltage_ = true;
            log("FAULT", "Paired sensor demo injected: engine coolant 126 C and battery voltage 10.45 V");
        } else if (command == "FAULT:ENGINE_OFFLINE") {
            resumeForScenario();
            fault_engine_offline_ = true; vehicle_.setEngineOnline(false);
            log("FAULT", "Engine ECU transmission disabled");
        } else if (command == "FAULT:BATTERY_OFFLINE") {
            resumeForScenario();
            fault_battery_offline_ = true; vehicle_.setBatteryOnline(false);
            log("FAULT", "Battery ECU transmission disabled");
        } else if (command == "FAULT:CRC") {
            resumeForScenario();
            fault_crc_ = true;
            vehicle_.diagnostics().setInjectedCrcFault(true);
            vehicle_.injectCorruptedEngineTelemetry(time_ms_);
            captureFrames();
            log("FAULT", "Engine telemetry CRC fault latched until recovery (CAN ID 0x100)");
            detectDtcTransitions();
        } else if (command == "FAULT:COUNTER") {
            resumeForScenario();
            fault_counter_ = true;
            vehicle_.diagnostics().setInjectedCounterFault(true);
            vehicle_.injectRollingCounterError(time_ms_);
            captureFrames();
            log("FAULT", "Engine telemetry rolling-counter fault latched until recovery (CAN ID 0x100)");
            detectDtcTransitions();
        } else if (command == "RECOVER:ALL") {
            resumeForScenario();
            fault_overtemp_ = false;
            fault_low_voltage_ = false;
            fault_engine_offline_ = false;
            fault_battery_offline_ = false;
            fault_crc_ = false;
            fault_counter_ = false;
            vehicle_.diagnostics().setInjectedCrcFault(false);
            vehicle_.diagnostics().setInjectedCounterFault(false);
            vehicle_.setEngineOnline(true);
            vehicle_.setBatteryOnline(true);
            log("INFO", "All injected faults released; ECU telemetry will confirm recovery");
        } else if (command.rfind("CAN_CONTROLLER:", 0U) == 0U) {
            const auto fields = splitColonFields(command.substr(15U));
            if (fields.size() < 2U || fields.size() > 3U) {
                log("ERROR", "Invalid CAN controller command format");
            } else {
                const auto node = parseCanNode(fields[1]);
                if (!node) {
                    log("ERROR", "Unknown CAN controller node; use ENGINE, BATTERY, BODY or DIAGNOSTICS");
                } else if (fields[0] == "TX_BURST" || fields[0] == "RX_BURST") {
                    std::uint16_t count = 0U;
                    const auto max_count = fields[0] == "TX_BURST" ? 64U : 255U;
                    if (fields.size() != 3U || !parseBoundedCount(fields[2], 1U,
                            static_cast<std::uint16_t>(max_count), count)) {
                        log("ERROR", fields[0] == "TX_BURST"
                            ? "TX burst count must be an integer from 1 to 64"
                            : "RX burst count must be an integer from 1 to 255");
                    } else {
                        const bool accepted = fields[0] == "TX_BURST"
                            ? vehicle_.injectTransmitErrorBurst(*node, count)
                            : vehicle_.injectReceiveErrorBurst(*node, count);
                        if (!accepted) log("ERROR", "Error burst rejected: controller is Bus-Off or count is invalid");
                        else log("FAULT", std::string(canNodeDisplayName(*node)) + " injected " +
                            std::to_string(count) + (fields[0] == "TX_BURST" ? " transmit error event(s)" : " receive error event(s)"));
                    }
                } else if (fields[0] == "TX_INJECT" || fields[0] == "RX_INJECT") {
                    if (fields.size() != 3U || (fields[2] != "ON" && fields[2] != "OFF")) {
                        log("ERROR", "Controller injection command must end in ON or OFF");
                    } else {
                        const bool enabled = fields[2] == "ON";
                        if (fields[0] == "TX_INJECT") vehicle_.setTxErrorInjection(*node, enabled);
                        else vehicle_.setRxErrorInjection(*node, enabled);
                        log(enabled ? "FAULT" : "INFO", std::string(canNodeDisplayName(*node)) +
                            (fields[0] == "TX_INJECT" ? " TX error injection " : " RX error injection ") +
                            (enabled ? "ENABLED" : "DISABLED"));
                    }
                } else if (fields[0] == "RECOVER" && fields.size() == 2U) {
                    resumeForScenario();
                    if (vehicle_.startBusOffRecovery(*node)) {
                        log("RECOVERY", std::string(canNodeDisplayName(*node)) +
                            " Bus-Off recovery started; waiting for 128 modeled 11-recessive-bit sequences");
                    } else {
                        log("ERROR", "Bus-Off recovery cannot start: node must be Bus-Off and all TX error injectors must be disabled");
                    }
                } else {
                    log("ERROR", "Unsupported CAN controller operation");
                }
            }
            detectControllerTransitions();
        } else if (command == "CAN_FILTER:CLEAR") {
            can_filter_active_ = false;
            can_filter_id_ = 0U;
            can_filter_mask_ = 0U;
            can_filter_accepted_ = 0U;
            can_filter_rejected_ = 0U;
            frames_.clear();
            log("INFO", "CAN receive filter cleared; all configured message IDs accepted by monitor");
        } else if (command.rfind("CAN_FILTER:", 0) == 0) {
            const auto value = command.substr(11U);
            const auto separator = value.find(':');
            if (separator == std::string::npos) {
                log("ERROR", "Invalid CAN filter command; expected CAN_FILTER:ID:MASK");
                return;
            }
            const auto id_text = std::string_view(value).substr(0U, separator);
            const auto mask_text = std::string_view(value).substr(separator + 1U);
            std::uint32_t id = 0U;
            std::uint32_t mask = 0U;
            const auto parse_hex = [](std::string_view text, std::uint32_t& output) {
                if (text.rfind("0x", 0U) == 0U) text.remove_prefix(2U);
                if (text.empty()) return false;
                const auto parsed = std::from_chars(text.data(), text.data() + text.size(), output, 16);
                return parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size();
            };
            if (!parse_hex(id_text, id) || !parse_hex(mask_text, mask) ||
                id > 0x7FFU || mask > 0x7FFU) {
                log("ERROR", "CAN acceptance filter requires 11-bit hexadecimal ID and mask values");
                return;
            }
            can_filter_active_ = true;
            can_filter_id_ = id;
            can_filter_mask_ = mask;
            can_filter_accepted_ = 0U;
            can_filter_rejected_ = 0U;
            frames_.clear();
            log("INFO", "CAN acceptance filter set: ID " + hexId(id) + " MASK " + hexId(mask));
        } else if (command.rfind("THROTTLE:", 0) == 0) {
            const std::string value_text = command.substr(9);
            int value = 0;
            const auto parsed = std::from_chars(value_text.data(), value_text.data() + value_text.size(), value);
            if (value_text.empty() || parsed.ec != std::errc{} ||
                parsed.ptr != value_text.data() + value_text.size()) {
                log("ERROR", "Invalid throttle command; expected an integer from 0 to 100");
                return;
            }
            if (value < 0 || value > 100) {
                log("ERROR", "Throttle command rejected; value must be from 0 to 100");
                return;
            }
            throttle_ = value;
            log("INFO", "Driver demand set to " + std::to_string(throttle_) + "%");
        } else if (command == "LIGHTS:TOGGLE") {
            vehicle_.body().setHeadlights(!vehicle_.body().headlights());
            log("INFO", std::string("Headlights ") + (vehicle_.body().headlights() ? "ON" : "OFF"));
        } else if (command == "DOOR:TOGGLE") {
            vehicle_.body().setDoorOpen(!vehicle_.body().doorOpen());
            log("INFO", std::string("Driver door ") + (vehicle_.body().doorOpen() ? "OPEN" : "CLOSED"));
        } else if (command.rfind("UDS:DID:", 0) == 0) {
            const auto did = command.substr(8);
            std::vector<std::uint8_t> request{0x22U};
            if (did == "F40C") { request.push_back(0xF4U); request.push_back(0x0CU); }
            else if (did == "F405") { request.push_back(0xF4U); request.push_back(0x05U); }
            else if (did == "F411") { request.push_back(0xF4U); request.push_back(0x11U); }
            else { log("ERROR", "Unsupported DID selected"); return; }
            runUds(request);
        } else if (command == "UDS:DTC") {
            runUds({0x19U, 0x02U, 0x2FU});
        } else if (command.rfind("UDS:FREEZE:", 0) == 0) {
            const auto code_text = command.substr(11);
            if (code_text.size() != 6U) { log("ERROR", "DTC code must contain six hexadecimal digits"); return; }
            std::uint32_t code = 0U;
            const auto parsed = std::from_chars(code_text.data(), code_text.data() + code_text.size(), code, 16);
            if (parsed.ec != std::errc{} || parsed.ptr != code_text.data() + code_text.size() ||
                code > 0xFFFFFFU) {
                log("ERROR", "Invalid hexadecimal DTC code");
            } else {
                runUds({0x19U, 0x04U,
                    static_cast<std::uint8_t>((code >> 16U) & 0xFFU),
                    static_cast<std::uint8_t>((code >> 8U) & 0xFFU),
                    static_cast<std::uint8_t>(code & 0xFFU), 0x01U});
            }
        } else if (command == "UDS:SESSION") {
            runUds({0x10U, 0x03U});
        } else if (command == "UDS:RESET") {
            runUds({0x11U, 0x01U});
        } else if (command == "UDS:TESTER") {
            runUds({0x3EU, 0x00U});
        } else if (command == "UDS:CLEAR") {
            runUds({0x14U, 0xFFU, 0xFFU, 0xFFU});
            log("WARN", "UDS ClearDiagnosticInformation request executed");
        } else {
            log("ERROR", "Unknown dashboard command rejected");
        }
    }

    std::string stateJson() const {
        std::ostringstream out;
        out << std::fixed << std::setprecision(2);
        const auto& d = vehicle_.diagnostics().dtcs();
        const auto active = static_cast<std::size_t>(std::count_if(d.begin(), d.end(),
            [](const auto& item) { return item.second.active; }));
        const auto stored = static_cast<std::size_t>(std::count_if(d.begin(), d.end(),
            [](const auto& item) { return item.second.status != 0U; }));
        const auto rpm = vehicle_.engine().rpm();
        const auto temp = vehicle_.engine().temperature();
        const auto speed = vehicle_.engine().speed();
        const double voltage = static_cast<double>(vehicle_.battery().voltageMv()) / 1000.0;
        const double current = static_cast<double>(vehicle_.battery().currentMa()) / 1000.0;
        out << "{\"running\":" << (running_ ? "true" : "false")
            << ",\"time_ms\":" << time_ms_
            << ",\"throttle\":" << throttle_
            << ",\"injected_faults\":{\"overtemperature\":" << (fault_overtemp_ ? "true" : "false")
            << ",\"low_voltage\":" << (fault_low_voltage_ ? "true" : "false")
            << ",\"engine_offline\":" << (fault_engine_offline_ ? "true" : "false")
            << ",\"battery_offline\":" << (fault_battery_offline_ ? "true" : "false")
            << ",\"crc\":" << (fault_crc_ ? "true" : "false")
            << ",\"counter\":" << (fault_counter_ ? "true" : "false") << "}"
            << ",\"engine\":{\"online\":" << (vehicle_.isNodeOnline(CanNode::Engine) && !fault_engine_offline_ ? "true" : "false")
            << ",\"rpm\":" << rpm << ",\"temperature_c\":" << temp
            << ",\"speed_kph\":" << speed << "}"
            << ",\"battery\":{\"online\":" << (vehicle_.isNodeOnline(CanNode::Battery) && !fault_battery_offline_ ? "true" : "false")
            << ",\"voltage_v\":" << voltage << ",\"current_a\":" << current
            << ",\"temperature_c\":" << vehicle_.battery().temperature() << "}"
            << ",\"body\":{\"online\":" << (vehicle_.isNodeOnline(CanNode::Body) ? "true" : "false") << ",\"headlights\":"
            << (vehicle_.body().headlights() ? "true" : "false")
            << ",\"door_open\":" << (vehicle_.body().doorOpen() ? "true" : "false") << "}"
            << ",\"diagnostics\":{\"online\":" << (vehicle_.isNodeOnline(CanNode::Diagnostics) ? "true" : "false") << ",\"active_faults\":" << active
            << ",\"dtc_count\":" << stored << ",\"dtc_catalog_count\":" << d.size() << ",\"dtcs\":[";
        bool first = true;
        for (const auto& item : d) {
            if (!first) out << ',';
            first = false;
            const auto& dtc = item.second;
            out << "{\"code\":" << quote(dtc.name)
                << ",\"description\":" << quote(dtc.description)
                << ",\"active\":" << (dtc.active ? "true" : "false")
                << ",\"status\":" << static_cast<unsigned>(dtc.status)
                << ",\"pending\":" << ((dtc.status & 0x04U) != 0U ? "true" : "false")
                << ",\"confirmed\":" << ((dtc.status & 0x08U) != 0U ? "true" : "false")
                << ",\"failure_samples\":" << static_cast<unsigned>(dtc.consecutive_failure_samples);
            const auto ff = vehicle_.diagnostics().freezeFrames().find(item.first);
            if (ff != vehicle_.diagnostics().freezeFrames().end()) {
                out << ",\"freeze_frame\":{\"rpm\":" << ff->second.engine_rpm
                    << ",\"temperature_c\":" << ff->second.engine_temp_c
                    << ",\"speed_kph\":" << ff->second.vehicle_speed_kph
                    << ",\"battery_v\":" << static_cast<double>(ff->second.battery_mv) / 1000.0
                    << ",\"timestamp_ms\":" << ff->second.timestamp_ms << '}';
            } else out << ",\"freeze_frame\":null";
            out << '}';
        }
        out << "]}"
            << ",\"can\":{\"frames_total\":" << vehicle_.canFramesTransmitted()
            << ",\"monitor\":{\"filter_active\":" << (can_filter_active_ ? "true" : "false")
            << ",\"filter_id\":" << quote(hexId(can_filter_id_))
            << ",\"filter_mask\":" << quote(hexId(can_filter_mask_))
            << ",\"accepted\":" << can_filter_accepted_
            << ",\"rejected\":" << can_filter_rejected_ << "}"
            << ",\"message_database\":{\"message_count\":" << message_database_.messages().size()
            << ",\"signal_count\":" << message_database_.signalCount() << ",\"messages\":[";
        first = true;
        for (const auto& message : message_database_.messages()) {
            if (!first) out << ',';
            first = false;
            out << "{\"name\":" << quote(message.name)
                << ",\"id\":" << quote(hexId(message.id))
                << ",\"period_ms\":";
            if (message.period_ms) out << *message.period_ms; else out << "null";
            out << ",\"dlc\":" << static_cast<unsigned>(message.dlc)
                << ",\"signals\":" << message.signals.size() << '}';
        }
        out << "]},\"controller_health\":{\"error_active_count\":";
        const auto count_state = [this](CanControllerState target) {
            return static_cast<std::size_t>(std::count_if(allCanNodes.begin(), allCanNodes.end(),
                [this, target](CanNode node) { return vehicle_.controller(node).state() == target; }));
        };
        out << count_state(CanControllerState::ErrorActive)
            << ",\"error_passive_count\":" << count_state(CanControllerState::ErrorPassive)
            << ",\"bus_off_count\":" << count_state(CanControllerState::BusOff)
            << ",\"controllers\":[";
        for (std::size_t i = 0U; i < allCanNodes.size(); ++i) {
            if (i != 0U) out << ',';
            const auto node = allCanNodes[i];
            const auto snap = vehicle_.controllerSnapshot(node);
            out << "{\"key\":" << quote(std::string(canNodeKey(node)))
                << ",\"name\":" << quote(std::string(canNodeDisplayName(node)))
                << ",\"tec\":" << snap.transmit_error_counter
                << ",\"rec\":" << snap.receive_error_counter
                << ",\"state\":" << quote(std::string(canControllerStateName(snap.state)))
                << ",\"tx_attempts\":" << snap.transmit_attempts
                << ",\"tx_successes\":" << snap.transmit_successes
                << ",\"tx_errors\":" << snap.transmit_errors
                << ",\"rx_successes\":" << snap.receive_successes
                << ",\"rx_errors\":" << snap.receive_errors
                << ",\"tx_error_injection\":" << (vehicle_.txErrorInjectionEnabled(node) ? "true" : "false")
                << ",\"rx_error_injection\":" << (vehicle_.rxErrorInjectionEnabled(node) ? "true" : "false")
                << ",\"recovery_in_progress\":" << (snap.recovery_in_progress ? "true" : "false")
                << ",\"recovery_sequences_seen\":" << snap.recovery_sequences_seen
                << ",\"recovery_sequences_required\":" << CanController::bus_off_recovery_sequences_required
                << ",\"bus_off_recoveries\":" << snap.bus_off_recoveries << '}';
        }
        out << "]},\"last_frames\":[";
        first = true;
        for (const auto& rec : frames_) {
            if (!canFrameAccepted(rec.frame)) continue;
            if (!first) out << ',';
            first = false;
            const auto* definition = message_database_.findById(rec.frame.id);
            out << "{\"time_ms\":" << rec.time_ms
                << ",\"id\":" << quote(hexId(rec.frame.id, rec.frame.extended ? 8 : 3))
                << ",\"name\":" << quote(definition ? definition->name : "UNMAPPED")
                << ",\"dlc\":" << static_cast<unsigned>(rec.frame.dlc)
                << ",\"data\":" << quote(bytesHex(rec.frame))
                << ",\"valid_crc\":" << (verifyTelemetryFrame(rec.frame) ? "true" : "false") << '}';
        }
        out << "]}"
            << ",\"history\":[";
        first = true;
        for (const auto& sample : samples_) {
            if (!first) out << ',';
            first = false;
            out << "{\"time_ms\":" << sample.time_ms << ",\"rpm\":" << sample.rpm
                << ",\"temperature_c\":" << sample.temp << ",\"speed_kph\":" << sample.speed
                << ",\"voltage_v\":" << (static_cast<double>(sample.voltage) / 1000.0)
                << ",\"current_a\":" << (static_cast<double>(sample.current) / 1000.0) << '}';
        }
        out << "]"
            << ",\"events\":[";
        first = true;
        for (const auto& event : events_) {
            if (!first) out << ',';
            first = false;
            out << "{\"time_ms\":" << event.time_ms << ",\"level\":" << quote(event.level)
                << ",\"message\":" << quote(event.message) << '}';
        }
        out << "]"
            << ",\"uds\":{\"request\":" << quote(last_uds_request_)
            << ",\"response\":" << quote(last_uds_response_) << "}}";
        return out.str();
    }

private:
    void resumeForScenario() {
        if (!running_) {
            running_ = true;
            log("INFO", "Simulation automatically resumed so fault injection and recovery can be monitored");
        }
    }

    void log(const std::string& level, const std::string& message) {
        events_.push_back(Event{time_ms_, level, message});
        if (events_.size() > 100U) events_.erase(events_.begin());
    }
    void capture() {
        samples_.push_back(Sample{time_ms_, vehicle_.engine().rpm(), vehicle_.engine().temperature(),
            vehicle_.engine().speed(), static_cast<double>(vehicle_.battery().voltageMv()),
            static_cast<double>(vehicle_.battery().currentMa())});
        if (samples_.size() > 120U) samples_.erase(samples_.begin());
    }
    bool canFrameAccepted(const CanFrame& frame) const noexcept {
        return !can_filter_active_ || (frame.id & can_filter_mask_) == (can_filter_id_ & can_filter_mask_);
    }
    void captureFrames() {
        for (const auto& frame : vehicle_.lastFrames()) {
            if (canFrameAccepted(frame)) ++can_filter_accepted_;
            else ++can_filter_rejected_;
            frames_.push_back(FrameRecord{time_ms_, frame});
            if (frames_.size() > 100U) frames_.erase(frames_.begin());
        }
    }
    void detectDtcTransitions() {
        for (const auto& item : vehicle_.diagnostics().dtcs()) {
            const auto& dtc = item.second;
            const std::uint8_t current_state = dtc.active
                ? static_cast<std::uint8_t>((dtc.status & 0x08U) != 0U ? 2U : 1U)
                : static_cast<std::uint8_t>(dtc.status != 0U ? 3U : 0U);
            const auto previous = last_dtc_state_.find(item.first);
            const std::uint8_t previous_state = previous == last_dtc_state_.end() ? 0U : previous->second;

            if (current_state == 1U && previous_state != 1U && previous_state != 2U) {
                log("WARN", dtc.name + " pending: " + dtc.description +
                    " (" + std::to_string(dtc.consecutive_failure_samples) + "/3 failed samples)");
            } else if (current_state == 2U && previous_state != 2U) {
                log("FAULT", dtc.name + " confirmed: " + dtc.description);
            } else if (current_state == 3U && (previous_state == 1U || previous_state == 2U)) {
                log("RECOVERY", dtc.name + " recovered; historical status retained: " + dtc.description);
            } else if (current_state == 0U && previous_state == 3U) {
                log("INFO", dtc.name + " diagnostic history cleared");
            }
            last_dtc_state_[item.first] = current_state;
        }
    }
    void detectControllerTransitions() {
        for (std::size_t i = 0U; i < allCanNodes.size(); ++i) {
            const auto node = allCanNodes[i];
            const auto snapshot = vehicle_.controllerSnapshot(node);
            const auto previous = last_controller_states_[i];
            if (snapshot.state != previous) {
                const auto name = std::string(canNodeDisplayName(node));
                if (snapshot.state == CanControllerState::ErrorPassive) {
                    log("WARN", name + " CAN controller entered ERROR PASSIVE (TEC=" +
                        std::to_string(snapshot.transmit_error_counter) + ", REC=" +
                        std::to_string(snapshot.receive_error_counter) + ")");
                } else if (snapshot.state == CanControllerState::BusOff) {
                    log("FAULT", name + " CAN controller entered BUS-OFF and stopped network transmission");
                } else if (previous == CanControllerState::BusOff) {
                    log("RECOVERY", name + " CAN controller recovered to ERROR ACTIVE after 128 modeled idle sequences");
                } else {
                    log("INFO", name + " CAN controller returned to ERROR ACTIVE");
                }
                last_controller_states_[i] = snapshot.state;
            }
        }
    }

    void runUds(const std::vector<std::uint8_t>& request) {
        try {
            last_uds_request_ = payloadHex(request);
            const auto tx = isoTpSegment(request, 0x7E0U);
            const auto response_frames = vehicle_.requestUdsIsoTp(tx, time_ms_);
            if (response_frames.empty()) {
                last_uds_response_ = "NO RESPONSE";
            } else {
                IsoTpReceiver receiver(0x7E8U, 0x7E0U);
                std::vector<std::uint8_t> payload;
                for (const auto& frame : response_frames) {
                    const auto result = receiver.consume(frame, time_ms_);
                    if (result.status == IsoTpStatus::Complete) payload = result.payload;
                }
                last_uds_response_ = payload.empty() ? "INVALID ISO-TP RESPONSE" : payloadHex(payload);
            }
            log("INFO", "UDS request [" + last_uds_request_ + "] -> [" + last_uds_response_ + "]");
            detectDtcTransitions();
        } catch (const std::exception& ex) {
            last_uds_response_ = std::string("ERROR: ") + ex.what();
            log("ERROR", last_uds_response_);
        }
    }

    CanMessageDatabase message_database_;
    VehicleNetwork vehicle_;
    bool can_filter_active_{false};
    std::uint32_t can_filter_id_{0U};
    std::uint32_t can_filter_mask_{0U};
    std::uint64_t can_filter_accepted_{0U};
    std::uint64_t can_filter_rejected_{0U};
    bool running_{true};
    bool fault_overtemp_{false};
    bool fault_low_voltage_{false};
    bool fault_engine_offline_{false};
    bool fault_battery_offline_{false};
    bool fault_crc_{false};
    bool fault_counter_{false};
    int throttle_{35};
    std::uint64_t time_ms_{0};
    double phase_{0.0};
    std::vector<Sample> samples_;
    std::vector<FrameRecord> frames_;
    std::vector<Event> events_;
    std::map<std::uint32_t, std::uint8_t> last_dtc_state_;
    std::array<CanControllerState, 4U> last_controller_states_{};
    std::string last_uds_request_;
    std::string last_uds_response_;
};
} // namespace

int main(int argc, char** argv) {
    try {
        std::string matrix_path = "config/can_messages.csv";
        for (int i = 1; i < argc; ++i) {
            const std::string argument = argv[i];
            if (argument == "--can-matrix" && i + 1 < argc) {
                matrix_path = argv[++i];
            } else {
                std::cerr << "Usage: autocore_engine [--can-matrix PATH]\n";
                return 2;
            }
        }
        auto database = CanMessageDatabase::loadCsv(matrix_path);
        Simulation simulation(std::move(database));
        std::string command;
        while (std::getline(std::cin, command)) {
            if (command == "TICK") simulation.advance();
            else if (command != "STATE") simulation.execute(command);
            std::cout << simulation.stateJson() << '\n' << std::flush;
        }
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "AutoCore engine startup failed: " << ex.what() << '\n';
        return 1;
    }
}
