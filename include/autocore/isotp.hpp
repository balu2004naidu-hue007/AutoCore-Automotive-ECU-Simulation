#pragma once

#include "autocore/can_frame.hpp"
#include <cstdint>
#include <optional>
#include <vector>

namespace autocore {

enum class IsoTpStatus {
    Ignored,
    Complete,
    InProgress,
    FlowControlRequired,
    Error,
    TimedOut
};

struct IsoTpResult {
    IsoTpStatus status{IsoTpStatus::Ignored};
    std::vector<std::uint8_t> payload;
    std::optional<CanFrame> flow_control;
};

// Classic CAN ISO-TP receiver for normal addressing, 12-bit First Frame lengths.
// It reassembles Single/First/Consecutive Frames and emits Continue-To-Send flow control.
class IsoTpReceiver {
public:
    IsoTpReceiver(std::uint32_t receive_id, std::uint32_t flow_control_id,
                  std::uint64_t timeout_ms = 1000U);

    IsoTpResult consume(const CanFrame& frame, std::uint64_t now_ms);
    IsoTpResult checkTimeout(std::uint64_t now_ms);
    void reset() noexcept;
    bool inProgress() const noexcept { return expected_length_ != 0U; }

private:
    std::uint32_t receive_id_;
    std::uint32_t flow_control_id_;
    std::uint64_t timeout_ms_;
    std::uint64_t last_frame_ms_{0};
    std::size_t expected_length_{0};
    std::uint8_t next_sequence_{1};
    std::vector<std::uint8_t> payload_;
};


enum class IsoTpTxStatus {
    Idle,
    AwaitingFlowControl,
    Ready,
    Complete,
    Error
};

// Stateful Classic CAN ISO-TP transmitter. The caller supplies the flow-control
// frames received from the peer and polls nextFrame() using a monotonic clock
// expressed in microseconds. This enforces block size and STmin pacing.
class IsoTpTransmitter {
public:
    IsoTpTransmitter(std::uint32_t transmit_id, std::uint32_t flow_control_receive_id,
                     std::uint8_t max_wait_frames = 3U);

    std::vector<CanFrame> start(const std::vector<std::uint8_t>& payload);
    IsoTpTxStatus consumeFlowControl(const CanFrame& frame, std::uint64_t now_us);
    std::optional<CanFrame> nextFrame(std::uint64_t now_us);
    void reset() noexcept;
    IsoTpTxStatus status() const noexcept { return status_; }

private:
    std::uint32_t transmit_id_;
    std::uint32_t flow_control_receive_id_;
    std::uint8_t max_wait_frames_;
    IsoTpTxStatus status_{IsoTpTxStatus::Idle};
    std::vector<std::uint8_t> payload_;
    std::size_t offset_{0};
    std::uint8_t next_sequence_{1};
    std::uint8_t block_size_{0};
    std::uint8_t block_count_{0};
    std::uint8_t wait_frames_{0};
    std::uint64_t stmin_us_{0};
    std::uint64_t next_send_time_us_{0};
};

// Segment a UDS payload into Classic CAN ISO-TP frames. Flow-control pacing is
// handled by the caller; this helper produces the protocol frames in order.
std::vector<CanFrame> isoTpSegment(const std::vector<std::uint8_t>& payload,
                                   std::uint32_t transmit_id);

} // namespace autocore
