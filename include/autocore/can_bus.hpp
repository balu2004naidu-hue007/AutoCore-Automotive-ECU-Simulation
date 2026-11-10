#pragma once

#include "autocore/can_frame.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace autocore {

class CanBus {
public:
    void transmit(const CanFrame& frame);
    std::vector<CanFrame> arbitrate();
    std::size_t pending() const noexcept { return pending_.size(); }
    std::uint64_t transmittedCount() const noexcept { return transmitted_count_; }

private:
    std::vector<CanFrame> pending_;
    std::uint64_t transmitted_count_{0};
};

} // namespace autocore
