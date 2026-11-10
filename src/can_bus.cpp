#include "autocore/can_bus.hpp"
#include <algorithm>

namespace autocore {

void CanBus::transmit(const CanFrame& frame) {
    frame.validate();
    pending_.push_back(frame);
}

std::vector<CanFrame> CanBus::arbitrate() {
    // Lower numerical CAN identifiers have higher arbitration priority.
    std::stable_sort(pending_.begin(), pending_.end(), [](const CanFrame& lhs, const CanFrame& rhs) {
        // CAN arbitration is a bitwise priority contest, not simply a numeric
        // comparison of the stored identifier. Compare the shared 11-bit base
        // first; a standard frame wins over an extended frame with that base
        // because its IDE/SRR arbitration bits are dominant/recessive.
        const std::uint32_t lhs_base = lhs.extended ? (lhs.id >> 18U) : lhs.id;
        const std::uint32_t rhs_base = rhs.extended ? (rhs.id >> 18U) : rhs.id;
        if (lhs_base != rhs_base) return lhs_base < rhs_base;
        if (lhs.extended != rhs.extended) return !lhs.extended;
        if (lhs.extended && lhs.id != rhs.id) return lhs.id < rhs.id;
        // For equal identifiers, a data frame wins over a remote frame.
        if (lhs.remote != rhs.remote) return !lhs.remote;
        return false; // Preserve enqueue order for identical arbitration fields.
    });
    auto result = std::move(pending_);
    pending_.clear();
    transmitted_count_ += static_cast<std::uint64_t>(result.size());
    return result;
}

} // namespace autocore
