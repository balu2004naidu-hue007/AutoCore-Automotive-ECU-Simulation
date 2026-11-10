#pragma once

#include "autocore/can_bus.hpp"
#include <cstdint>
#include <string>

namespace autocore {

class EngineEcu {
public:
    void setRpm(std::uint16_t value) noexcept { rpm_ = value; }
    void setTemperature(std::int16_t value) noexcept { temperature_c_ = value; }
    void setSpeed(std::uint16_t value) noexcept { speed_kph_ = value; }
    std::uint16_t rpm() const noexcept { return rpm_; }
    std::int16_t temperature() const noexcept { return temperature_c_; }
    std::uint16_t speed() const noexcept { return speed_kph_; }
    void publish(CanBus& bus);

private:
    std::uint16_t rpm_{850};
    std::int16_t temperature_c_{90};
    std::uint16_t speed_kph_{0};
    std::uint8_t rolling_counter_{0};
};

class BatteryEcu {
public:
    void setVoltageMv(std::uint16_t value) noexcept { voltage_mv_ = value; }
    void setCurrentMa(std::int16_t value) noexcept { current_ma_ = value; }
    void setTemperature(std::int16_t value) noexcept { temperature_c_ = value; }
    std::uint16_t voltageMv() const noexcept { return voltage_mv_; }
    std::int16_t currentMa() const noexcept { return current_ma_; }
    std::int16_t temperature() const noexcept { return temperature_c_; }
    void publish(CanBus& bus);

private:
    std::uint16_t voltage_mv_{12600};
    std::int16_t current_ma_{1200};
    std::int16_t temperature_c_{28};
    std::uint8_t rolling_counter_{0};
};

class BodyEcu {
public:
    void setHeadlights(bool value) noexcept { headlights_ = value; }
    void setDoorOpen(bool value) noexcept { door_open_ = value; }
    bool headlights() const noexcept { return headlights_; }
    bool doorOpen() const noexcept { return door_open_; }
    void publish(CanBus& bus);

private:
    bool headlights_{false};
    bool door_open_{false};
    std::uint8_t rolling_counter_{0};
};

} // namespace autocore
