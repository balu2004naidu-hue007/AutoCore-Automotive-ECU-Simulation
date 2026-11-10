#include "autocore/can_bus.hpp"
#include "autocore/vehicle_network.hpp"
#include "autocore/isotp.hpp"
#include "autocore/frame_integrity.hpp"
#include "autocore/can_message_database.hpp"
#include "autocore/can_controller.hpp"
#if defined(AUTOCORE_HAS_SOCKETCAN)
#include "autocore/socketcan.hpp"
#endif
#include <cstdlib>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <algorithm>
#include <iostream>
#include <string>
#include <stdexcept>

namespace {
int failures = 0;
void check(bool condition, const std::string& message) {
    if (condition) std::cout << "[PASS] " << message << '\n';
    else { std::cerr << "[FAIL] " << message << '\n'; ++failures; }
}

autocore::CanFrame uds(std::initializer_list<std::uint8_t> bytes) {
    autocore::CanFrame frame{};
    frame.id = 0x7E0U;
    frame.dlc = static_cast<std::uint8_t>(bytes.size());
    std::size_t i = 0;
    for (const auto byte : bytes) frame.data[i++] = byte;
    return frame;
}
}

int main() {
    using namespace autocore;
    {
        CanController controller;
        for (unsigned i = 0U; i < 16U; ++i) controller.recordTransmitError();
        auto snapshot = controller.snapshot();
        check(snapshot.transmit_error_counter == 128U &&
              snapshot.state == CanControllerState::ErrorPassive,
              "CAN controller enters Error Passive when TEC reaches 128");
        controller.recordTransmitSuccess();
        check(controller.snapshot().transmit_error_counter == 127U &&
              controller.state() == CanControllerState::ErrorActive,
              "successful transmission decrements TEC and can restore Error Active");

        controller.reset();
        for (unsigned i = 0U; i < 32U; ++i) controller.recordTransmitError();
        snapshot = controller.snapshot();
        check(snapshot.transmit_error_counter == 256U &&
              snapshot.state == CanControllerState::BusOff,
              "CAN controller enters Bus-Off when TEC exceeds the 8-bit error-counter range");
        check(controller.beginBusOffRecovery(),
              "Bus-Off controller accepts an explicit recovery request");
        for (unsigned i = 0U; i < 127U; ++i)
            controller.observeBusOffRecoverySequence(true);
        check(controller.state() == CanControllerState::BusOff &&
              controller.snapshot().recovery_sequences_seen == 127U,
              "Bus-Off recovery remains pending before the 128th idle-sequence observation");
        controller.observeBusOffRecoverySequence(false);
        check(controller.snapshot().recovery_sequences_seen == 127U,
              "Bus-Off recovery progress does not advance without an eligible idle sequence");
        controller.observeBusOffRecoverySequence(true);
        snapshot = controller.snapshot();
        check(snapshot.state == CanControllerState::ErrorActive && snapshot.transmit_error_counter == 0U &&
              snapshot.receive_error_counter == 0U && snapshot.bus_off_recoveries == 1U,
              "128 modeled idle-sequence observations recover Bus-Off and reset TEC/REC");

        controller.reset();
        for (unsigned i = 0U; i < 128U; ++i) controller.recordReceiveError();
        check(controller.snapshot().receive_error_counter == 128U &&
              controller.state() == CanControllerState::ErrorPassive,
              "CAN controller enters Error Passive when REC reaches 128");
        controller.recordReceiveSuccess();
        check(controller.snapshot().receive_error_counter == 127U &&
              controller.state() == CanControllerState::ErrorActive,
              "successful reception returns a passive REC to the active boundary");
        controller.reset();
        for (unsigned i = 0U; i < 200U; ++i) controller.recordReceiveError();
        controller.recordReceiveSuccess();
        check(controller.snapshot().receive_error_counter == 127U &&
              controller.state() == CanControllerState::ErrorActive,
              "a successful receive clamps a REC above 127 to the error-active boundary");
        controller.reset();
        for (unsigned i = 0U; i < 300U; ++i) controller.recordReceiveError();
        check(controller.snapshot().receive_error_counter == 255U &&
              controller.state() == CanControllerState::ErrorPassive,
              "REC saturates at 255 without wrapping or incorrectly entering Bus-Off");
        controller.recordTransmitSuccess();
        check(controller.snapshot().transmit_error_counter == 0U,
              "a successful transmit at TEC zero cannot underflow the counter");
    }
    {
        VehicleNetwork vehicle;
        vehicle.tick(0U);
        check(vehicle.injectTransmitErrorBurst(CanNode::Engine, 24U) &&
              vehicle.controller(CanNode::Engine).state() == CanControllerState::ErrorPassive,
              "Engine ECU controller can be driven into Error Passive using a transmit-error burst");
        check(vehicle.injectTransmitErrorBurst(CanNode::Engine, 8U) &&
              vehicle.controller(CanNode::Engine).state() == CanControllerState::BusOff &&
              !vehicle.isNodeOnline(CanNode::Engine),
              "additional transmit errors drive the Engine ECU into Bus-Off and mark it offline");
        const auto total_before = vehicle.canFramesTransmitted();
        vehicle.tick(100U);
        check(vehicle.canFramesTransmitted() == total_before + 2U &&
              std::none_of(vehicle.lastFrames().begin(), vehicle.lastFrames().end(),
                  [](const CanFrame& frame) { return frame.id == 0x100U; }),
              "Bus-Off Engine ECU stops transmitting while Battery and Body ECUs keep communicating");
        vehicle.setTxErrorInjection(CanNode::Battery, true);
        check(!vehicle.startBusOffRecovery(CanNode::Engine),
              "Bus-Off recovery waits while any node has a persistent TX error injector enabled");
        vehicle.setTxErrorInjection(CanNode::Battery, false);
        check(vehicle.startBusOffRecovery(CanNode::Engine),
              "Bus-Off Engine ECU accepts recovery after all TX error injectors are disabled");
        const auto before_first_recovery_tick = vehicle.controllerSnapshot(CanNode::Engine).recovery_sequences_seen;
        vehicle.tick(200U);
        const auto after_first_recovery_tick = vehicle.controllerSnapshot(CanNode::Engine).recovery_sequences_seen;
        vehicle.tick(200U);
        check(after_first_recovery_tick == before_first_recovery_tick + 1U &&
              vehicle.controllerSnapshot(CanNode::Engine).recovery_sequences_seen == after_first_recovery_tick,
              "repeated calls at the same simulation timestamp cannot double-count Bus-Off recovery sequences");
        for (std::uint64_t i = 3U; i < 130U; ++i) vehicle.tick(i * 100U);
        check(vehicle.controllerSnapshot(CanNode::Engine).recovery_sequences_seen == 128U &&
              vehicle.controller(CanNode::Engine).state() == CanControllerState::ErrorActive,
              "network recovery completes after exactly 128 distinct modeled idle-sequence observations");
        check(vehicle.isNodeOnline(CanNode::Engine),
              "Engine ECU returns online after successful Bus-Off recovery");
        vehicle.tick(13000U);
        check(std::any_of(vehicle.lastFrames().begin(), vehicle.lastFrames().end(),
                  [](const CanFrame& frame) { return frame.id == 0x100U; }),
              "recovered Engine ECU resumes periodic CAN telemetry");

        VehicleNetwork rx_vehicle;
        rx_vehicle.tick(0U);
        check(rx_vehicle.injectReceiveErrorBurst(CanNode::Battery, 128U) &&
              rx_vehicle.controller(CanNode::Battery).state() == CanControllerState::ErrorPassive,
              "Battery ECU receive-error burst raises REC to Error Passive threshold");
        rx_vehicle.tick(100U);
        check(rx_vehicle.controllerSnapshot(CanNode::Battery).receive_error_counter < 128U,
              "valid CAN receptions decrement the Battery ECU REC after error injection stops");
    }
    {
        const std::string matrix_path = std::string(AUTOCORE_SOURCE_DIR) + "/config/can_messages.csv";
        const auto database = CanMessageDatabase::loadCsv(matrix_path);
        const auto* engine_message = database.findById(0x100U);
        check(database.messages().size() == 5U && database.signalCount() == 16U,
              "runtime CAN message matrix loads all five messages and sixteen signal definitions");
        check(engine_message != nullptr && engine_message->name == "ENGINE_STATUS" &&
              engine_message->dlc == 8U && engine_message->period_ms == 100U &&
              engine_message->signals.size() == 5U,
              "CAN message matrix exposes validated Engine ECU ID, period, DLC and signals");
        bool missing_file_rejected = false;
        try { (void)CanMessageDatabase::loadCsv(matrix_path + ".missing"); }
        catch (const std::runtime_error&) { missing_file_rejected = true; }
        check(missing_file_rejected, "CAN message database rejects a missing configuration file");
        const auto temp_base = std::filesystem::temp_directory_path() /
            ("autocore_matrix_test_" + std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()));
        std::ifstream source(matrix_path);
        const std::string valid_csv((std::istreambuf_iterator<char>(source)),
                                    std::istreambuf_iterator<char>());
        const auto rejects_csv = [&](const std::string& suffix, const std::string& contents) {
            const auto path = temp_base.string() + suffix;
            { std::ofstream output(path); output << contents; }
            bool rejected = false;
            try { (void)CanMessageDatabase::loadCsv(path); }
            catch (const std::runtime_error&) { rejected = true; }
            std::error_code ignored;
            std::filesystem::remove(path, ignored);
            return rejected;
        };
        auto duplicate_signal = valid_csv;
        const auto first_row_start = duplicate_signal.find('\n') + 1U;
        const auto first_row_end = duplicate_signal.find('\n', first_row_start);
        duplicate_signal.insert(duplicate_signal.find('\n', first_row_end + 1U),
                                duplicate_signal.substr(first_row_start, first_row_end - first_row_start));
        check(rejects_csv("_duplicate.csv", duplicate_signal),
              "CAN message database rejects duplicate signal definitions");
        auto invalid_dlc = valid_csv;
        const auto dlc_field = invalid_dlc.find("ENGINE_STATUS,0x100,100,8,");
        if (dlc_field != std::string::npos) invalid_dlc.replace(dlc_field, std::string("ENGINE_STATUS,0x100,100,8,").size(), "ENGINE_STATUS,0x100,100,9,");
        check(rejects_csv("_dlc.csv", invalid_dlc),
              "CAN message database rejects signal layouts beyond the Classic CAN DLC");
        auto invalid_scale = valid_csv;
        const auto scale_field = invalid_scale.find(",1,0,rpm,");
        if (scale_field != std::string::npos) invalid_scale.replace(scale_field, std::string(",1,0,rpm,").size(), ",nan,0,rpm,");
        check(rejects_csv("_scale.csv", invalid_scale),
              "CAN message database rejects non-finite signal scaling values");
        check(database.findById(0x7FFU) == nullptr,
              "CAN message database returns no definition for an unmapped identifier");
    }
    {
        CanFrame standard{};
        standard.id = 0x7FFU;
        standard.dlc = 8U;
        standard.validate();
        check(true, "CAN frame accepts the maximum standard 11-bit identifier and Classic CAN DLC");

        CanFrame extended{};
        extended.id = 0x1FFFFFFFU;
        extended.extended = true;
        extended.dlc = 8U;
        extended.validate();
        check(true, "CAN frame accepts the maximum extended 29-bit identifier");

        bool invalid_standard_id_rejected = false;
        try { CanFrame bad{}; bad.id = 0x800U; bad.validate(); }
        catch (const std::invalid_argument&) { invalid_standard_id_rejected = true; }
        check(invalid_standard_id_rejected, "CAN frame rejects an 11-bit identifier overflow");

        bool invalid_dlc_rejected = false;
        try { CanFrame bad{}; bad.dlc = 9U; bad.validate(); }
        catch (const std::invalid_argument&) { invalid_dlc_rejected = true; }
        check(invalid_dlc_rejected, "Classic CAN frame rejects DLC greater than eight bytes");

        CanFrame payload{};
        payload.dlc = 8U;
        writeU16BE(payload, 6U, 0xA55AU);
        check(readU16BE(payload, 6U) == 0xA55AU,
              "Big-endian CAN payload helpers round-trip the last two data bytes");

        bool read_bounds_rejected = false;
        try { (void)readU16BE(payload, static_cast<std::size_t>(-1)); }
        catch (const std::out_of_range&) { read_bounds_rejected = true; }
        check(read_bounds_rejected, "CAN payload reader rejects out-of-range indices without integer wraparound");

        bool invalid_dlc_read_rejected = false;
        payload.dlc = 255U;
        try { (void)readU16BE(payload, 7U); }
        catch (const std::out_of_range&) { invalid_dlc_read_rejected = true; }
        check(invalid_dlc_read_rejected, "CAN payload reader rejects a DLC larger than the fixed data array");
        payload.dlc = 8U;

        bool write_bounds_rejected = false;
        try { writeU16BE(payload, static_cast<std::size_t>(-1), 0x1234U); }
        catch (const std::out_of_range&) { write_bounds_rejected = true; }
        check(write_bounds_rejected, "CAN payload writer rejects out-of-range indices without integer wraparound");
    }
#if defined(AUTOCORE_HAS_SOCKETCAN)
    {
        SocketCanInterface interface;
        bool closed_send_rejected = false;
        try { CanFrame frame{}; interface.send(frame); }
        catch (const std::logic_error&) { closed_send_rejected = true; }
        check(closed_send_rejected && !interface.isOpen(),
              "SocketCAN adapter rejects transmit before interface is opened");
        bool closed_receive_rejected = false;
        try { (void)interface.receive(0U); }
        catch (const std::logic_error&) { closed_receive_rejected = true; }
        check(closed_receive_rejected,
              "SocketCAN adapter rejects receive before interface is opened");
        bool closed_filter_rejected = false;
        try { interface.setFilters({}); }
        catch (const std::logic_error&) { closed_filter_rejected = true; }
        check(closed_filter_rejected,
              "SocketCAN adapter rejects filter changes before interface is opened");
    }
#endif
    {
        const std::vector<std::uint8_t> payload{
            0x59U, 0x02U, 0xFFU, 0x01U, 0x23U, 0x45U, 0x67U, 0x89U,
            0xABU, 0xCDU, 0xEFU, 0x10U, 0x20U, 0x30U, 0x40U, 0x50U,
            0x60U, 0x70U, 0x80U, 0x90U
        };
        const auto frames = isoTpSegment(payload, 0x7E8U);
        IsoTpReceiver receiver(0x7E8U, 0x7E0U);
        const auto first = receiver.consume(frames.at(0), 10U);
        check(frames.size() == 3U && first.status == IsoTpStatus::FlowControlRequired &&
              first.flow_control.has_value() && first.flow_control->id == 0x7E0U &&
              first.flow_control->data[0] == 0x30U,
              "ISO-TP First Frame requests Continue-To-Send flow control");
        CanFrame extended_first = frames.front();
        extended_first.extended = true;
        check(receiver.consume(extended_first, 11U).status == IsoTpStatus::Ignored,
              "ISO-TP normal-addressing receiver ignores extended-ID frames");
        CanFrame invalid_dlc_frame{};
        invalid_dlc_frame.id = 0x7E8U;
        invalid_dlc_frame.dlc = 9U;
        invalid_dlc_frame.data[0] = 0x01U;
        IsoTpReceiver invalid_dlc_receiver(0x7E8U, 0x7E0U);
        check(invalid_dlc_receiver.consume(invalid_dlc_frame, 12U).status == IsoTpStatus::Error &&
              !invalid_dlc_receiver.inProgress(),
              "ISO-TP receiver rejects an invalid DLC before accessing the fixed payload buffer");
        auto second = receiver.consume(frames.at(1), 20U);
        auto third = receiver.consume(frames.at(2), 30U);
        check(second.status == IsoTpStatus::InProgress && third.status == IsoTpStatus::Complete &&
              third.payload == payload,
              "ISO-TP reassembles a multi-frame payload across Consecutive Frames");

        const auto single_frames = isoTpSegment({0x22U, 0xF4U, 0x0CU}, 0x7E0U);
        IsoTpReceiver single_receiver(0x7E0U, 0x7E8U);
        const auto single = single_receiver.consume(single_frames.front(), 40U);
        check(single.status == IsoTpStatus::Complete &&
              single.payload == std::vector<std::uint8_t>({0x22U, 0xF4U, 0x0CU}),
              "ISO-TP Single Frame encodes and decodes a short UDS payload");

        IsoTpReceiver sequence_receiver(0x7E8U, 0x7E0U);
        sequence_receiver.consume(frames.at(0), 50U);
        CanFrame bad_sequence = frames.at(1);
        bad_sequence.data[0] = 0x22U;
        const auto sequence_error = sequence_receiver.consume(bad_sequence, 60U);
        check(sequence_error.status == IsoTpStatus::Error && !sequence_receiver.inProgress(),
              "ISO-TP rejects an unexpected Consecutive Frame sequence number");

        IsoTpReceiver timeout_receiver(0x7E8U, 0x7E0U, 100U);
        timeout_receiver.consume(frames.at(0), 100U);
        const auto timeout = timeout_receiver.checkTimeout(201U);
        check(timeout.status == IsoTpStatus::TimedOut && !timeout_receiver.inProgress(),
              "ISO-TP aborts an incomplete transfer after its configured timeout");
        std::vector<std::uint8_t> maximum_payload(4095U);
        std::uint32_t state = 0xC0FFEEU;
        for (auto& byte : maximum_payload) {
            state = state * 1664525U + 1013904223U;
            byte = static_cast<std::uint8_t>((state >> 24U) & 0xFFU);
        }
        const auto maximum_frames = isoTpSegment(maximum_payload, 0x7E0U);
        IsoTpReceiver maximum_receiver(0x7E0U, 0x7E8U, 1000U);
        IsoTpResult maximum_result{};
        for (std::size_t i = 0; i < maximum_frames.size(); ++i)
            maximum_result = maximum_receiver.consume(maximum_frames[i], i);
        check(maximum_result.status == IsoTpStatus::Complete &&
              maximum_result.payload == maximum_payload,
              "ISO-TP round-trips the maximum 4095-byte payload across sequence-number wraparound");
        IsoTpTransmitter paced_transmitter(0x7E0U, 0x7E8U);
        std::vector<std::uint8_t> paced_payload(25U, 0x5AU);
        const auto first_outbound = paced_transmitter.start(paced_payload);
        CanFrame flow_control{};
        flow_control.id = 0x7E8U;
        flow_control.dlc = 3U;
        flow_control.data[0] = 0x30U;
        flow_control.data[1] = 2U;
        flow_control.data[2] = 1U;
        check(first_outbound.size() == 1U && paced_transmitter.status() == IsoTpTxStatus::AwaitingFlowControl &&
              paced_transmitter.consumeFlowControl(flow_control, 1000U) == IsoTpTxStatus::Ready,
              "ISO-TP transmitter waits for and accepts a Continue-To-Send flow-control frame");
        const auto cf1 = paced_transmitter.nextFrame(1000U);
        const auto too_early = paced_transmitter.nextFrame(1999U);
        const auto cf2 = paced_transmitter.nextFrame(2000U);
        check(cf1.has_value() && cf1->data[0] == 0x21U && !too_early.has_value() &&
              cf2.has_value() && cf2->data[0] == 0x22U &&
              paced_transmitter.status() == IsoTpTxStatus::AwaitingFlowControl,
              "ISO-TP transmitter enforces STmin and the negotiated block size");
        flow_control.data[1] = 0U;
        flow_control.data[2] = 0U;
        const auto cf3 = paced_transmitter.consumeFlowControl(flow_control, 2000U) == IsoTpTxStatus::Ready
            ? paced_transmitter.nextFrame(2000U) : std::nullopt;
        check(cf3.has_value() && cf3->data[0] == 0x23U &&
              paced_transmitter.status() == IsoTpTxStatus::Complete,
              "ISO-TP transmitter resumes after a second flow-control block and completes");

        IsoTpTransmitter wait_transmitter(0x7E0U, 0x7E8U, 2U);
        wait_transmitter.start(std::vector<std::uint8_t>(8U, 0x11U));
        flow_control.data[0] = 0x31U;
        check(wait_transmitter.consumeFlowControl(flow_control, 0U) == IsoTpTxStatus::AwaitingFlowControl &&
              wait_transmitter.consumeFlowControl(flow_control, 1U) == IsoTpTxStatus::AwaitingFlowControl &&
              wait_transmitter.consumeFlowControl(flow_control, 2U) == IsoTpTxStatus::Error,
              "ISO-TP transmitter aborts after exceeding the configured WAIT-frame limit");

        IsoTpTransmitter overflow_transmitter(0x7E0U, 0x7E8U);
        overflow_transmitter.start(std::vector<std::uint8_t>(8U, 0x22U));
        flow_control.data[0] = 0x32U;
        check(overflow_transmitter.consumeFlowControl(flow_control, 0U) == IsoTpTxStatus::Error,
              "ISO-TP transmitter aborts when peer flow control reports OVERFLOW");

        IsoTpTransmitter submillisecond_transmitter(0x7E0U, 0x7E8U);
        submillisecond_transmitter.start(std::vector<std::uint8_t>(15U, 0x33U));
        flow_control.data[0] = 0x30U;
        flow_control.data[1] = 0U;
        flow_control.data[2] = 0xF1U;
        submillisecond_transmitter.consumeFlowControl(flow_control, 500U);
        const auto sub_cf1 = submillisecond_transmitter.nextFrame(500U);
        const auto sub_early = submillisecond_transmitter.nextFrame(599U);
        const auto sub_cf2 = submillisecond_transmitter.nextFrame(600U);
        check(sub_cf1.has_value() && !sub_early.has_value() && sub_cf2.has_value() &&
              submillisecond_transmitter.status() == IsoTpTxStatus::Complete,
              "ISO-TP transmitter decodes and enforces 100-microsecond STmin values");
    }
    {
        const std::uint8_t check_vector[] = {'1','2','3','4','5','6','7','8','9'};
        check(crc8SaeJ1850(check_vector, sizeof(check_vector)) == 0x4BU,
              "CRC-8/SAE-J1850 matches the standard 123456789 check vector");
        check(crc8SaeJ1850(nullptr, 0U) == 0U && crc8SaeJ1850(nullptr, 1U) == 0U,
              "CRC helper safely handles null input with zero or non-zero length");

        EngineEcu engine;
        CanBus engine_bus;
        engine.publish(engine_bus);
        auto frames = engine_bus.arbitrate();
        check(frames.size() == 1U && frames[0].dlc == 8U &&
              frames[0].data[6] == 0U && verifyTelemetryFrame(frames[0]),
              "Engine ECU publishes an 8-byte telemetry frame with rolling counter and CRC");
        CanFrame remote_telemetry = frames[0];
        remote_telemetry.remote = true;
        check(!verifyTelemetryFrame(remote_telemetry),
              "Telemetry integrity verifier rejects remote-request frames");

        DiagnosticsEcu diagnostics;
        diagnostics.observe(frames[0], 0U);
        CanFrame corrupted = frames[0];
        corrupted.data[0] ^= 0x01U;
        diagnostics.observe(corrupted, 10U);
        check(diagnostics.dtcs().at(0xA00001U).active,
              "Diagnostics ECU detects corrupted telemetry CRC and activates AC0001");

        BatteryEcu battery;
        CanBus battery_bus;
        battery.publish(battery_bus);
        diagnostics.observe(battery_bus.arbitrate().front(), 20U);
        check(diagnostics.dtcs().at(0xA00001U).active,
              "CRC fault remains active while another ECU sends valid telemetry");

        engine.publish(engine_bus);
        const auto recovered_frame = engine_bus.arbitrate().front();
        diagnostics.observe(recovered_frame, 30U);
        check(!diagnostics.dtcs().at(0xA00001U).active,
              "Valid telemetry from the affected ECU clears its CRC fault");
        check(!diagnostics.dtcs().at(0xA00002U).active,
              "A CRC-rejected frame causes counter re-synchronization, not a cascading false counter DTC");

        CanFrame sequence_gap = recovered_frame;
        sealTelemetryFrame(sequence_gap, 7U);
        diagnostics.observe(sequence_gap, 40U);
        check(diagnostics.dtcs().at(0xA00002U).active,
              "Diagnostics ECU detects a rolling-counter sequence gap");
        CanFrame sequence_recovery = recovered_frame;
        sealTelemetryFrame(sequence_recovery, 8U);
        diagnostics.observe(sequence_recovery, 50U);
        check(!diagnostics.dtcs().at(0xA00002U).active,
              "Expected rolling-counter sequence clears AC0002");
    }
    {
        CanBus bus;
        CanFrame low{}; low.id = 0x700U; low.dlc = 1U;
        CanFrame high{}; high.id = 0x100U; high.dlc = 1U;
        bus.transmit(low); bus.transmit(high);
        const auto frames = bus.arbitrate();
        check(frames.size() == 2U && frames[0].id == 0x100U && frames[1].id == 0x700U,
              "CAN arbitration prioritizes the lower identifier");
        CanFrame extended{};
        extended.id = (0x123U << 18U) | 0x2U;
        extended.extended = true;
        extended.dlc = 1U;
        CanFrame standard{};
        standard.id = 0x123U;
        standard.dlc = 1U;
        bus.transmit(extended);
        bus.transmit(standard);
        const auto format_priority = bus.arbitrate();
        check(format_priority.size() == 2U && !format_priority[0].extended &&
              format_priority[1].extended,
              "standard CAN frame wins over extended frame with matching 11-bit base ID");

        CanFrame remote{};
        remote.id = 0x321U;
        remote.dlc = 1U;
        remote.remote = true;
        CanFrame data{};
        data.id = 0x321U;
        data.dlc = 1U;
        bus.transmit(remote);
        bus.transmit(data);
        const auto rtr_priority = bus.arbitrate();
        check(rtr_priority.size() == 2U && !rtr_priority[0].remote &&
              rtr_priority[1].remote,
              "CAN data frame wins over remote frame with matching identifier");
    }
    {
        VehicleNetwork vehicle;
        vehicle.tick(0U);
        check(vehicle.canFramesTransmitted() == 3U,
              "periodic ECU scheduler publishes all telemetry immediately on first tick");
        vehicle.tick(50U);
        check(vehicle.canFramesTransmitted() == 3U,
              "periodic ECU scheduler does not publish before the 100 ms deadline");
        vehicle.tick(100U);
        check(vehicle.canFramesTransmitted() == 6U,
              "periodic ECU scheduler publishes the next telemetry cycle at 100 ms");
        vehicle.tick(450U);
        check(vehicle.canFramesTransmitted() == 9U,
              "periodic ECU scheduler emits once after a delayed tick without burst catch-up");
        vehicle.tick(500U);
        check(vehicle.canFramesTransmitted() == 9U,
              "periodic ECU scheduler retains the next deadline after delayed service");
        vehicle.tick(550U);
        check(vehicle.canFramesTransmitted() == 12U,
              "periodic ECU scheduler resumes at the next 100 ms deadline");
    }
    {
        DiagnosticsEcu diagnostics;
        CanFrame extended_request = uds({0x03U, 0x22U, 0xF4U, 0x0CU});
        extended_request.extended = true;
        const auto extended_response = diagnostics.handleUdsRequest(
            extended_request, 1500U, 90, 12500U, 10U);
        check(extended_response.data[1] == 0x7FU && extended_response.data[2] == 0x00U &&
              extended_response.data[3] == 0x11U,
              "UDS CAN endpoint rejects extended-ID requests on its standard 11-bit address");

        CanFrame oversized_request = uds({0x03U, 0x22U, 0xF4U, 0x0CU});
        oversized_request.dlc = 9U;
        const auto oversized_response = diagnostics.handleUdsRequest(
            oversized_request, 1500U, 90, 12500U, 11U);
        check(oversized_response.data[1] == 0x7FU && oversized_response.data[2] == 0x00U &&
              oversized_response.data[3] == 0x13U,
              "UDS CAN endpoint rejects a DLC larger than the Classic CAN payload buffer");
    }
    {
        VehicleNetwork vehicle;
        vehicle.engine().setRpm(2200U);
        vehicle.engine().setTemperature(95);
        vehicle.battery().setVoltageMv(12400U);
        vehicle.tick(100U);
        check(vehicle.canFramesTransmitted() == 3U, "three ECU telemetry frames reach the CAN bus per tick");
        auto response = vehicle.requestUds(uds({0x03U, 0x22U, 0xF4U, 0x0CU}), 110U);
        check(response.id == 0x7E8U && response.data[0] == 0x05U &&
              response.dlc == 6U && response.data[1] == 0x62U &&
              readU16BE(response, 4U) == 2200U, "UDS 0x22 reads engine RPM DID F40C");
        response = vehicle.requestUds(uds({0x02U, 0x10U, 0x03U}), 120U);
        check(response.data[1] == 0x50U && response.data[2] == 0x03U,
              "UDS 0x10 switches diagnostic session");
        response = vehicle.requestUds(uds({0x02U, 0x11U, 0x01U}), 125U);
        check(response.data[1] == 0x51U, "UDS 0x11 returns simulated ECU Reset acknowledgement");
        response = vehicle.requestUds(uds({0x03U, 0x19U, 0x01U, 0x01U}), 127U);
        check(response.data[1] == 0x59U && response.data[0] == 0x06U &&
              readU16BE(response, 5U) == 0U,
              "UDS 0x19 subfunction 0x01 reports matching DTC count by status mask");
        response = vehicle.requestUds(uds({0x02U, 0x3EU, 0x00U}), 130U);
        check(response.data[1] == 0x7EU && vehicle.diagnostics().lastTesterPresentMs() == 130U,
              "UDS 0x3E Tester Present is acknowledged");
        response = vehicle.requestUds(uds({0x03U, 0x22U, 0xABU, 0xCDU}), 140U);
        check(response.data[1] == 0x7FU && response.data[3] == 0x31U,
              "unsupported DID returns UDS request-out-of-range negative response");
    }
    {
        VehicleNetwork vehicle;
        vehicle.engine().setTemperature(125);
        vehicle.tick(0U);
        const auto& dtcs = vehicle.diagnostics().dtcs();
        const auto pending = dtcs.at(0x0217U).status;
        check(dtcs.at(0x0217U).active && (pending & 0x04U) != 0U &&
              (pending & 0x08U) == 0U,
              "first failed monitor sample creates a pending rather than immediately confirmed DTC");
        vehicle.tick(100U);
        check((vehicle.diagnostics().dtcs().at(0x0217U).status & 0x08U) == 0U,
              "second consecutive failure sample remains pending");
        vehicle.tick(200U);
        const auto confirmed = vehicle.diagnostics().dtcs().at(0x0217U).status;
        check((confirmed & 0x08U) != 0U && (confirmed & 0x04U) == 0U,
              "third consecutive failure sample promotes the DTC to confirmed");
        vehicle.engine().setTemperature(90);
        vehicle.tick(300U);
        const auto recovered = vehicle.diagnostics().dtcs().at(0x0217U).status;
        check(!vehicle.diagnostics().dtcs().at(0x0217U).active &&
              (recovered & 0x01U) == 0U && (recovered & 0x08U) != 0U,
              "recovery clears current failure while preserving confirmed historical status");
    }
    {
        VehicleNetwork crc_vehicle;
        crc_vehicle.diagnostics().setInjectedCrcFault(true);
        crc_vehicle.tick(0U);
        const auto crc_code = 0xA00001U;
        check(crc_vehicle.diagnostics().dtcs().at(crc_code).consecutive_failure_samples == 1U &&
              (crc_vehicle.diagnostics().dtcs().at(crc_code).status & 0x04U) != 0U,
              "multiple ECU frames at one timestamp count as one CRC monitor sample");
        crc_vehicle.tick(100U);
        check(crc_vehicle.diagnostics().dtcs().at(crc_code).consecutive_failure_samples == 2U &&
              (crc_vehicle.diagnostics().dtcs().at(crc_code).status & 0x08U) == 0U,
              "CRC DTC remains pending after the second distinct monitor timestamp");
        crc_vehicle.tick(200U);
        check((crc_vehicle.diagnostics().dtcs().at(crc_code).status & 0x08U) != 0U,
              "CRC DTC confirms only after three distinct 100 ms monitor timestamps");

        VehicleNetwork counter_vehicle;
        counter_vehicle.diagnostics().setInjectedCounterFault(true);
        counter_vehicle.tick(0U);
        const auto counter_code = 0xA00002U;
        check(counter_vehicle.diagnostics().dtcs().at(counter_code).consecutive_failure_samples == 1U &&
              (counter_vehicle.diagnostics().dtcs().at(counter_code).status & 0x04U) != 0U,
              "multiple ECU frames at one timestamp count as one rolling-counter monitor sample");
        counter_vehicle.tick(100U);
        check(counter_vehicle.diagnostics().dtcs().at(counter_code).consecutive_failure_samples == 2U &&
              (counter_vehicle.diagnostics().dtcs().at(counter_code).status & 0x08U) == 0U,
              "rolling-counter DTC remains pending after the second distinct monitor timestamp");
        counter_vehicle.tick(200U);
        check((counter_vehicle.diagnostics().dtcs().at(counter_code).status & 0x08U) != 0U,
              "rolling-counter DTC confirms only after three distinct 100 ms monitor timestamps");
    }
    {
        VehicleNetwork vehicle;
        vehicle.engine().setTemperature(125);
        vehicle.battery().setVoltageMv(10500U);
        vehicle.tick(0U);
        vehicle.tick(100U);
        vehicle.tick(200U);
        const auto& dtcs = vehicle.diagnostics().dtcs();
        check(dtcs.count(0x0217U) == 1U && dtcs.at(0x0217U).active,
              "engine over-temperature creates active P0217 DTC");
        check(dtcs.count(0x0562U) == 1U && dtcs.at(0x0562U).active,
              "low system voltage creates active P0562 DTC");
        vehicle.engine().setTemperature(90);
        vehicle.battery().setVoltageMv(12500U);
        vehicle.tick(300U);
        check(!vehicle.diagnostics().dtcs().at(0x0217U).active &&
              !vehicle.diagnostics().dtcs().at(0x0562U).active,
              "normal sensor readings clear active temperature and voltage DTC status");
        check((vehicle.diagnostics().dtcs().at(0x0217U).status & 0x01U) == 0U &&
              (vehicle.diagnostics().dtcs().at(0x0217U).status & 0x2AU) == 0x2AU,
              "recovered DTC retains historical failure and confirmation status bits");
    }
    {
        VehicleNetwork vehicle;
        vehicle.engine().setRpm(2350U);
        vehicle.engine().setTemperature(126);
        vehicle.engine().setSpeed(57U);
        vehicle.battery().setVoltageMv(10800U);
        vehicle.tick(250U);
        vehicle.tick(350U);
        vehicle.tick(450U);

        const auto& dtcs_before_clear = vehicle.diagnostics().dtcs();
        const auto& freeze_frames = vehicle.diagnostics().freezeFrames();
        check(dtcs_before_clear.at(0x0217U).active &&
              (dtcs_before_clear.at(0x0217U).status & 0x2BU) == 0x2BU,
              "newly detected DTC records active, failed-this-cycle, confirmed and since-clear status bits");
        check(freeze_frames.count(0x0217U) == 1U &&
              freeze_frames.at(0x0217U).engine_rpm == 2350U &&
              freeze_frames.at(0x0217U).engine_temp_c == 126 &&
              freeze_frames.at(0x0217U).vehicle_speed_kph == 57U,
              "DTC freeze frame captures engine operating conditions at fault detection");
        vehicle.engine().setRpm(3000U);
        vehicle.tick(550U);
        check(vehicle.diagnostics().freezeFrames().at(0x0217U).engine_rpm == 2350U,
              "repeated active fault observations preserve the original first-fault snapshot");

        const auto freeze_request = isoTpSegment({0x19U, 0x04U, 0x00U, 0x02U, 0x17U, 0x01U}, 0x7E0U);
        const auto freeze_response = vehicle.requestUdsIsoTp(freeze_request, 260U);
        IsoTpReceiver freeze_receiver(0x7E8U, 0x7E0U);
        IsoTpResult freeze_result{};
        for (std::size_t i = 0U; i < freeze_response.size(); ++i)
            freeze_result = freeze_receiver.consume(freeze_response[i], 261U + i);
        check(freeze_result.status == IsoTpStatus::Complete &&
              freeze_result.payload.size() >= 16U && freeze_result.payload[0] == 0x59U &&
              freeze_result.payload[1] == 0x04U && freeze_result.payload[2] == 0x00U &&
              freeze_result.payload[3] == 0x02U && freeze_result.payload[4] == 0x17U,
              "UDS 0x19 subfunction 0x04 returns a multi-frame DTC freeze-frame record");
        check(freeze_result.payload[7] == 0x05U &&
              freeze_result.payload[8] == 0xF4U && freeze_result.payload[9] == 0x0CU &&
              freeze_result.payload[10] == 0x09U && freeze_result.payload[11] == 0x2EU,
              "freeze-frame response declares five identifiers and includes the captured 2350 rpm value");

        const auto clear_response = vehicle.requestUdsPayload({0x14U, 0xFFU, 0xFFU, 0xFFU}, 270U);
        check(clear_response == std::vector<std::uint8_t>{0x54U} &&
              !vehicle.diagnostics().dtcs().at(0x0217U).active &&
              vehicle.diagnostics().dtcs().at(0x0217U).status == 0U &&
              vehicle.diagnostics().freezeFrames().empty(),
              "UDS 0x14 clears DTC status and stored freeze-frame records");
        const auto unsupported_clear = vehicle.requestUdsPayload({0x14U, 0x00U, 0x00U, 0x01U}, 271U);
        check(unsupported_clear.size() == 3U && unsupported_clear[0] == 0x7FU &&
              unsupported_clear[2] == 0x31U,
              "UDS 0x14 rejects unsupported DTC groups with request-out-of-range NRC");
        vehicle.engine().setRpm(3100U);
        vehicle.tick(650U);
        check(vehicle.diagnostics().dtcs().at(0x0217U).active &&
              vehicle.diagnostics().freezeFrames().at(0x0217U).engine_rpm == 3100U,
              "a cleared fault creates a fresh freeze frame when detected again");
    }
    {
        VehicleNetwork vehicle;
        vehicle.tick(0U);
        vehicle.setEngineOnline(false);
        vehicle.tick(600U);
        const auto& dtcs = vehicle.diagnostics().dtcs();
        check(dtcs.count(0x0100U) == 1U && dtcs.at(0x0100U).active,
              "engine communication timeout creates active U0100 DTC");
        vehicle.setEngineOnline(true);
        vehicle.tick(700U);
        check(!vehicle.diagnostics().dtcs().at(0x0100U).active,
              "engine communication recovery clears active timeout status");
    }
    {
        VehicleNetwork vehicle;
        vehicle.tick(0U);
        vehicle.setBatteryOnline(false);
        vehicle.tick(600U);
        const auto& dtcs = vehicle.diagnostics().dtcs();
        check(dtcs.count(0x0111U) == 1U && dtcs.at(0x0111U).active,
              "battery ECU communication timeout creates active U0111 DTC");
        vehicle.setBatteryOnline(true);
        vehicle.tick(700U);
        check(!vehicle.diagnostics().dtcs().at(0x0111U).active,
              "battery ECU communication recovery clears active timeout status");
    }
    {
        VehicleNetwork vehicle;
        vehicle.engine().setTemperature(125);
        vehicle.battery().setVoltageMv(10500U);
        vehicle.tick(0U);
        vehicle.setEngineOnline(false);
        vehicle.setBatteryOnline(false);
        vehicle.tick(600U);
        const auto count_payload = vehicle.requestUdsPayload({0x19U, 0x01U, 0x01U}, 610U);
        check(count_payload.size() == 6U && count_payload[0] == 0x59U &&
              count_payload[1] == 0x01U && count_payload[4] == 0U &&
              count_payload[5] == 4U,
              "UDS 0x19 status-mask count includes active sensor and communication DTCs");

        const auto request_frames = isoTpSegment({0x19U, 0x02U, 0x01U}, 0x7E0U);
        const auto response_frames = vehicle.requestUdsIsoTp(request_frames, 620U);
        check(response_frames.size() == 3U &&
              (response_frames.front().data[0] >> 4U) == 0x01U,
              "UDS 0x19 subfunction 0x02 returns multi-frame DTC records over ISO-TP");
        IsoTpReceiver response_receiver(0x7E8U, 0x7E0U);
        auto reassembled = response_receiver.consume(response_frames.front(), 621U);
        for (std::size_t i = 1U; i < response_frames.size(); ++i)
            reassembled = response_receiver.consume(response_frames[i], 621U + i);
        check(reassembled.status == IsoTpStatus::Complete &&
              reassembled.payload.size() == 19U && reassembled.payload[0] == 0x59U &&
              reassembled.payload[1] == 0x02U,
              "ISO-TP reassembles DTC status records from the UDS response");
        check(reassembled.payload[2] == 0x2FU,
              "UDS DTC record response declares the modeled status availability mask");

        const auto long_request = isoTpSegment(
            {0x99U, 0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U, 0x07U}, 0x7E0U);
        const auto long_request_response = vehicle.requestUdsIsoTp(long_request, 625U);
        check(long_request.size() == 2U && long_request_response.size() == 1U &&
              long_request_response[0].data[1] == 0x7FU &&
              long_request_response[0].data[2] == 0x99U,
              "UDS dispatcher accepts a reassembled multi-frame ISO-TP request");

        const auto unsupported = vehicle.requestUdsPayload({0x19U, 0x7FU}, 630U);
        check(unsupported.size() == 3U && unsupported[0] == 0x7FU &&
              unsupported[2] == 0x12U,
              "unsupported UDS 0x19 subfunction returns subfunction-not-supported NRC");
        const auto suppressed = vehicle.requestUdsPayload({0x3EU, 0x80U}, 640U);
        check(suppressed.empty() && vehicle.diagnostics().lastTesterPresentMs() == 640U,
              "UDS Tester Present suppress-positive-response bit is honored");
    }
    {
        VehicleNetwork vehicle;
        vehicle.tick(0U);
        vehicle.setTxErrorInjection(CanNode::Engine, true);
        for (std::uint64_t tick = 1U; tick <= 32U; ++tick) vehicle.tick(tick * 100U);
        const auto engine_controller = vehicle.controllerSnapshot(CanNode::Engine);
        check(engine_controller.transmit_errors == 32U &&
              engine_controller.transmit_error_counter == 256U &&
              engine_controller.state == CanControllerState::BusOff,
              "persistent Engine ECU TX error injection accumulates one +8 TEC event per 100 ms transmission attempt");
        check(vehicle.canFramesTransmitted() >= 3U + 64U,
              "Battery and Body ECUs continue to transmit while Engine ECU TX is fault-injected");
        vehicle.setTxErrorInjection(CanNode::Engine, false);
        check(vehicle.startBusOffRecovery(CanNode::Engine),
              "persistent TX injection can be disabled before starting Bus-Off recovery");
    }
    {
        VehicleNetwork vehicle;
        vehicle.injectTransmitErrorBurst(CanNode::Diagnostics, 32U);
        const auto response = vehicle.requestUdsPayload({0x22U, 0xF4U, 0x0CU}, 0U);
        check(vehicle.controller(CanNode::Diagnostics).state() == CanControllerState::BusOff &&
              response.empty(),
              "Diagnostics ECU Bus-Off prevents UDS responses until controller recovery");
    }
    {
        VehicleNetwork vehicle;
        constexpr std::uint64_t cycles = 10000U;
        for (std::uint64_t cycle = 0; cycle < cycles; ++cycle)
            vehicle.tick(cycle * 100U);
        const auto active_dtcs = std::count_if(vehicle.diagnostics().dtcs().begin(),
            vehicle.diagnostics().dtcs().end(), [](const auto& entry) { return entry.second.active; });
        check(vehicle.canFramesTransmitted() == cycles * 3U && active_dtcs == 0,
              "10,000-cycle ECU network stress test preserves telemetry integrity and counter wraparound");
    }
    {
        VehicleNetwork vehicle;
        vehicle.tick(1000U);
        vehicle.setEngineOnline(false);
        vehicle.tick(900U); // Simulated clock correction must not underflow elapsed time.
        const auto& dtcs = vehicle.diagnostics().dtcs();
        check(dtcs.count(0x0100U) == 1U && !dtcs.at(0x0100U).active,
              "backward simulation timestamp does not create a false communication timeout");
    }
    std::cout << "\nTest summary: " << (failures == 0 ? "ALL PASS" : "FAILURES DETECTED")
              << " (" << failures << " failed)\n";
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
