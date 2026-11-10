#include "autocore/ecus.hpp"
#include "autocore/frame_integrity.hpp"

namespace autocore {

void EngineEcu::publish(CanBus& bus) {
    CanFrame frame{};
    frame.id = 0x100;
    frame.dlc = 8;
    writeU16BE(frame, 0, rpm_);
    writeU16BE(frame, 2, static_cast<std::uint16_t>(temperature_c_));
    writeU16BE(frame, 4, speed_kph_);
    sealTelemetryFrame(frame, rolling_counter_++);
    bus.transmit(frame);
}

void BatteryEcu::publish(CanBus& bus) {
    CanFrame frame{};
    frame.id = 0x110;
    frame.dlc = 8;
    writeU16BE(frame, 0, voltage_mv_);
    writeU16BE(frame, 2, static_cast<std::uint16_t>(current_ma_));
    writeU16BE(frame, 4, static_cast<std::uint16_t>(temperature_c_));
    sealTelemetryFrame(frame, rolling_counter_++);
    bus.transmit(frame);
}

void BodyEcu::publish(CanBus& bus) {
    CanFrame frame{};
    frame.id = 0x120;
    frame.dlc = 8;
    frame.data[0] = static_cast<std::uint8_t>((headlights_ ? 0x01U : 0U) | (door_open_ ? 0x02U : 0U));
    frame.data[1] = 0U; // Reserved for future body-control signals.
    sealTelemetryFrame(frame, rolling_counter_++);
    bus.transmit(frame);
}

} // namespace autocore
