#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace autocore {

struct CanSignalDefinition {
    std::string name;
    std::uint8_t start_byte{0};
    std::uint8_t length{0};
    double scale{1.0};
    double offset{0.0};
    std::string unit;
    std::string description;
};

struct CanMessageDefinition {
    std::string name;
    std::uint32_t id{0};
    std::optional<std::uint32_t> period_ms;
    std::uint8_t dlc{0};
    std::vector<CanSignalDefinition> signals;
};

// Loads and validates AutoCore's CSV CAN message matrix. Each CSV row defines
// one signal within a message; messages are grouped by name and CAN identifier.
class CanMessageDatabase {
public:
    static CanMessageDatabase loadCsv(const std::string& path);

    const std::vector<CanMessageDefinition>& messages() const noexcept { return messages_; }
    const CanMessageDefinition* findById(std::uint32_t id) const noexcept;
    std::size_t signalCount() const noexcept { return signal_count_; }

private:
    std::vector<CanMessageDefinition> messages_;
    std::size_t signal_count_{0};
};

} // namespace autocore
