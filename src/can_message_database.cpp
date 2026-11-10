#include "autocore/can_message_database.hpp"

#include <charconv>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string_view>

namespace autocore {
namespace {

std::vector<std::string> splitCsvRow(const std::string& line, std::size_t line_number) {
    // The checked-in matrix intentionally uses simple unquoted CSV fields.
    // Reject quotes rather than silently misparse a future quoted field.
    if (line.find('"') != std::string::npos) {
        throw std::runtime_error("CAN matrix line " + std::to_string(line_number) +
                                 ": quoted CSV fields are not supported");
    }
    std::vector<std::string> fields;
    std::size_t start = 0;
    for (;;) {
        const auto comma = line.find(',', start);
        fields.push_back(line.substr(start, comma == std::string::npos ? comma : comma - start));
        if (comma == std::string::npos) break;
        start = comma + 1U;
    }
    return fields;
}

std::uint32_t parseUnsigned(std::string_view text, int base, const std::string& field,
                            std::size_t line_number) {
    std::uint32_t value = 0;
    if (text.empty()) {
        throw std::runtime_error("CAN matrix line " + std::to_string(line_number) +
                                 ": empty " + field);
    }
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value, base);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) {
        throw std::runtime_error("CAN matrix line " + std::to_string(line_number) +
                                 ": invalid " + field);
    }
    return value;
}

double parseDouble(const std::string& text, const std::string& field, std::size_t line_number) {
    std::size_t consumed = 0;
    double value = 0.0;
    try {
        value = std::stod(text, &consumed);
    } catch (const std::exception&) {
        throw std::runtime_error("CAN matrix line " + std::to_string(line_number) +
                                 ": invalid " + field);
    }
    if (consumed != text.size() || !std::isfinite(value)) {
        throw std::runtime_error("CAN matrix line " + std::to_string(line_number) +
                                 ": invalid " + field);
    }
    return value;
}

} // namespace

CanMessageDatabase CanMessageDatabase::loadCsv(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot open CAN message matrix: " + path);

    constexpr std::string_view expected_header =
        "name,can_id,period_ms,dlc,signal,start_byte,length,scale,offset,unit,description";
    std::string line;
    if (!std::getline(input, line) || line != expected_header) {
        throw std::runtime_error("CAN message matrix has a missing or unsupported header");
    }

    CanMessageDatabase database;
    std::size_t line_number = 1U;
    while (std::getline(input, line)) {
        ++line_number;
        if (line.empty()) continue;
        const auto fields = splitCsvRow(line, line_number);
        if (fields.size() != 11U) {
            throw std::runtime_error("CAN matrix line " + std::to_string(line_number) +
                                     ": expected 11 columns");
        }
        if (fields[0].empty() || fields[4].empty()) {
            throw std::runtime_error("CAN matrix line " + std::to_string(line_number) +
                                     ": message and signal names are required");
        }

        const auto id_text = std::string_view(fields[1]).substr(
            fields[1].rfind("0x", 0) == 0 ? 2U : 0U);
        const auto id = parseUnsigned(id_text, 16, "CAN identifier", line_number);
        if (id > 0x7FFU) {
            throw std::runtime_error("CAN matrix line " + std::to_string(line_number) +
                                     ": this Classic CAN matrix accepts 11-bit identifiers only");
        }

        std::optional<std::uint32_t> period;
        if (fields[2] != "on-demand") {
            const auto parsed_period = parseUnsigned(fields[2], 10, "period_ms", line_number);
            if (parsed_period == 0U) {
                throw std::runtime_error("CAN matrix line " + std::to_string(line_number) +
                                         ": periodic messages must have a non-zero period");
            }
            period = parsed_period;
        }
        const auto dlc_value = parseUnsigned(fields[3], 10, "DLC", line_number);
        const auto start_value = parseUnsigned(fields[5], 10, "start_byte", line_number);
        const auto length_value = parseUnsigned(fields[6], 10, "signal length", line_number);
        if (dlc_value == 0U || dlc_value > 8U || length_value == 0U ||
            start_value >= dlc_value || length_value > dlc_value - start_value) {
            throw std::runtime_error("CAN matrix line " + std::to_string(line_number) +
                                     ": signal layout exceeds Classic CAN DLC");
        }

        auto existing = database.messages_.end();
        for (auto it = database.messages_.begin(); it != database.messages_.end(); ++it) {
            if (it->name == fields[0] || it->id == id) {
                existing = it;
                break;
            }
        }
        if (existing == database.messages_.end()) {
            database.messages_.push_back(CanMessageDefinition{
                fields[0], id, period, static_cast<std::uint8_t>(dlc_value), {}});
            existing = std::prev(database.messages_.end());
        } else if (existing->name != fields[0] || existing->id != id ||
                   existing->period_ms != period || existing->dlc != dlc_value) {
            throw std::runtime_error("CAN matrix line " + std::to_string(line_number) +
                                     ": inconsistent message definition for name or CAN ID");
        }

        for (const auto& signal : existing->signals) {
            if (signal.name == fields[4]) {
                throw std::runtime_error("CAN matrix line " + std::to_string(line_number) +
                                         ": duplicate signal name in message");
            }
        }

        existing->signals.push_back(CanSignalDefinition{
            fields[4], static_cast<std::uint8_t>(start_value),
            static_cast<std::uint8_t>(length_value),
            parseDouble(fields[7], "scale", line_number),
            parseDouble(fields[8], "offset", line_number), fields[9], fields[10]});
        ++database.signal_count_;
    }

    if (database.messages_.empty()) throw std::runtime_error("CAN message matrix contains no messages");
    return database;
}

const CanMessageDefinition* CanMessageDatabase::findById(std::uint32_t id) const noexcept {
    for (const auto& message : messages_) {
        if (message.id == id) return &message;
    }
    return nullptr;
}

} // namespace autocore
