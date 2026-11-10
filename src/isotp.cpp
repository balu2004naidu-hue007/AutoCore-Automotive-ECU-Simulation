#include "autocore/isotp.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace autocore {
namespace {
CanFrame makeFlowControl(std::uint32_t id) {
    CanFrame frame{};
    frame.id = id;
    frame.dlc = 3U;
    frame.data[0] = 0x30U; // Flow Status: Continue To Send.
    frame.data[1] = 0x00U; // Block size: unlimited.
    frame.data[2] = 0x00U; // STmin: no additional delay requested.
    return frame;
}
}

IsoTpReceiver::IsoTpReceiver(std::uint32_t receive_id, std::uint32_t flow_control_id,
                             std::uint64_t timeout_ms)
    : receive_id_(receive_id), flow_control_id_(flow_control_id), timeout_ms_(timeout_ms) {
    CanFrame id_check{};
    id_check.id = receive_id_;
    id_check.dlc = 0U;
    id_check.validate();
    id_check.id = flow_control_id_;
    id_check.validate();
    if (timeout_ms_ == 0U) throw std::invalid_argument("ISO-TP timeout must be non-zero");
}

void IsoTpReceiver::reset() noexcept {
    expected_length_ = 0U;
    next_sequence_ = 1U;
    payload_.clear();
    last_frame_ms_ = 0U;
}

IsoTpResult IsoTpReceiver::checkTimeout(std::uint64_t now_ms) {
    if (inProgress() && now_ms >= last_frame_ms_ && now_ms - last_frame_ms_ > timeout_ms_) {
        reset();
        return {IsoTpStatus::TimedOut, {}, std::nullopt};
    }
    return {inProgress() ? IsoTpStatus::InProgress : IsoTpStatus::Ignored, {}, std::nullopt};
}

IsoTpResult IsoTpReceiver::consume(const CanFrame& frame, std::uint64_t now_ms) {
    if (frame.id != receive_id_ || frame.extended || frame.remote || frame.dlc == 0U) {
        return {IsoTpStatus::Ignored, {}, std::nullopt};
    }
    try { frame.validate(); }
    catch (const std::exception&) { reset(); return {IsoTpStatus::Error, {}, std::nullopt}; }

    if (inProgress()) {
        const auto timeout = checkTimeout(now_ms);
        if (timeout.status == IsoTpStatus::TimedOut) return timeout;
    }

    const std::uint8_t frame_type = static_cast<std::uint8_t>(frame.data[0] >> 4U);
    if (frame_type == 0U) { // Single Frame
        const std::size_t length = frame.data[0] & 0x0FU;
        if (length == 0U || length > 7U || length + 1U > frame.dlc) {
            reset();
            return {IsoTpStatus::Error, {}, std::nullopt};
        }
        reset();
        return {IsoTpStatus::Complete,
                std::vector<std::uint8_t>(frame.data.begin() + 1,
                                          frame.data.begin() + 1 + static_cast<std::ptrdiff_t>(length)),
                std::nullopt};
    }

    if (frame_type == 1U) { // First Frame
        if (frame.dlc != 8U) { reset(); return {IsoTpStatus::Error, {}, std::nullopt}; }
        const std::size_t length = (static_cast<std::size_t>(frame.data[0] & 0x0FU) << 8U) |
                                   static_cast<std::size_t>(frame.data[1]);
        if (length <= 7U || length > 4095U) { reset(); return {IsoTpStatus::Error, {}, std::nullopt}; }
        expected_length_ = length;
        payload_.assign(frame.data.begin() + 2, frame.data.end());
        last_frame_ms_ = now_ms;
        next_sequence_ = 1U;
        return {IsoTpStatus::FlowControlRequired, {}, makeFlowControl(flow_control_id_)};
    }

    if (frame_type == 2U) { // Consecutive Frame
        if (!inProgress() || frame.dlc < 2U || (frame.data[0] & 0x0FU) != next_sequence_) {
            reset();
            return {IsoTpStatus::Error, {}, std::nullopt};
        }
        const std::size_t remaining = expected_length_ - payload_.size();
        const std::size_t available = static_cast<std::size_t>(frame.dlc - 1U);
        const std::size_t copy_count = std::min(remaining, available);
        payload_.insert(payload_.end(), frame.data.begin() + 1,
                        frame.data.begin() + 1 + static_cast<std::ptrdiff_t>(copy_count));
        last_frame_ms_ = now_ms;
        next_sequence_ = static_cast<std::uint8_t>((next_sequence_ + 1U) & 0x0FU);
        if (payload_.size() == expected_length_) {
            auto complete = std::move(payload_);
            reset();
            return {IsoTpStatus::Complete, std::move(complete), std::nullopt};
        }
        return {IsoTpStatus::InProgress, {}, std::nullopt};
    }

    // Flow Control is consumed by a transmitter, not this receiver.
    return {IsoTpStatus::Ignored, {}, std::nullopt};
}

IsoTpTransmitter::IsoTpTransmitter(std::uint32_t transmit_id,
                                       std::uint32_t flow_control_receive_id,
                                       std::uint8_t max_wait_frames)
    : transmit_id_(transmit_id), flow_control_receive_id_(flow_control_receive_id),
      max_wait_frames_(max_wait_frames) {
    CanFrame id_check{};
    id_check.id = transmit_id_;
    id_check.validate();
    id_check.id = flow_control_receive_id_;
    id_check.validate();
    if (max_wait_frames_ == 0U)
        throw std::invalid_argument("ISO-TP maximum WAIT frame count must be non-zero");
}

void IsoTpTransmitter::reset() noexcept {
    status_ = IsoTpTxStatus::Idle;
    payload_.clear();
    offset_ = 0U;
    next_sequence_ = 1U;
    block_size_ = 0U;
    block_count_ = 0U;
    wait_frames_ = 0U;
    stmin_us_ = 0U;
    next_send_time_us_ = 0U;
}

std::vector<CanFrame> IsoTpTransmitter::start(const std::vector<std::uint8_t>& payload) {
    reset();
    if (payload.empty() || payload.size() > 4095U)
        throw std::invalid_argument("ISO-TP payload length must be between 1 and 4095 bytes");
    payload_ = payload;

    if (payload_.size() <= 7U) {
        CanFrame single{};
        single.id = transmit_id_;
        single.dlc = static_cast<std::uint8_t>(payload_.size() + 1U);
        single.data[0] = static_cast<std::uint8_t>(payload_.size());
        std::copy(payload_.begin(), payload_.end(), single.data.begin() + 1);
        status_ = IsoTpTxStatus::Complete;
        return {single};
    }

    CanFrame first{};
    first.id = transmit_id_;
    first.dlc = 8U;
    first.data[0] = static_cast<std::uint8_t>(0x10U | ((payload_.size() >> 8U) & 0x0FU));
    first.data[1] = static_cast<std::uint8_t>(payload_.size() & 0xFFU);
    std::copy_n(payload_.begin(), 6U, first.data.begin() + 2);
    offset_ = 6U;
    status_ = IsoTpTxStatus::AwaitingFlowControl;
    return {first};
}

IsoTpTxStatus IsoTpTransmitter::consumeFlowControl(const CanFrame& frame,
                                                    std::uint64_t now_us) {
    if (status_ != IsoTpTxStatus::AwaitingFlowControl) return status_;
    if (frame.id != flow_control_receive_id_ || frame.extended || frame.remote || frame.dlc < 3U) {
        status_ = IsoTpTxStatus::Error;
        return status_;
    }
    try { frame.validate(); }
    catch (const std::exception&) { status_ = IsoTpTxStatus::Error; return status_; }
    if ((frame.data[0] >> 4U) != 0x03U) {
        status_ = IsoTpTxStatus::Error;
        return status_;
    }

    const std::uint8_t flow_status = static_cast<std::uint8_t>(frame.data[0] & 0x0FU);
    if (flow_status == 0x01U) { // WAIT
        if (wait_frames_ >= max_wait_frames_) status_ = IsoTpTxStatus::Error;
        else ++wait_frames_;
        return status_;
    }
    if (flow_status == 0x02U) { // OVERFLOW / abort
        status_ = IsoTpTxStatus::Error;
        return status_;
    }
    if (flow_status != 0x00U) {
        status_ = IsoTpTxStatus::Error;
        return status_;
    }

    const std::uint8_t stmin = frame.data[2];
    if (stmin <= 0x7FU) stmin_us_ = static_cast<std::uint64_t>(stmin) * 1000U;
    else if (stmin >= 0xF1U && stmin <= 0xF9U)
        stmin_us_ = static_cast<std::uint64_t>(stmin - 0xF0U) * 100U;
    else {
        status_ = IsoTpTxStatus::Error;
        return status_;
    }

    block_size_ = frame.data[1];
    block_count_ = 0U;
    wait_frames_ = 0U;
    next_send_time_us_ = now_us;
    status_ = IsoTpTxStatus::Ready;
    return status_;
}

std::optional<CanFrame> IsoTpTransmitter::nextFrame(std::uint64_t now_us) {
    if (status_ != IsoTpTxStatus::Ready || now_us < next_send_time_us_ ||
        offset_ >= payload_.size()) return std::nullopt;

    CanFrame consecutive{};
    consecutive.id = transmit_id_;
    consecutive.dlc = 8U;
    consecutive.data[0] = static_cast<std::uint8_t>(0x20U | next_sequence_);
    const std::size_t count = std::min<std::size_t>(7U, payload_.size() - offset_);
    std::copy_n(payload_.begin() + static_cast<std::ptrdiff_t>(offset_), count,
                consecutive.data.begin() + 1);
    offset_ += count;
    next_sequence_ = static_cast<std::uint8_t>((next_sequence_ + 1U) & 0x0FU);
    ++block_count_;

    if (offset_ >= payload_.size()) status_ = IsoTpTxStatus::Complete;
    else if (block_size_ != 0U && block_count_ >= block_size_)
        status_ = IsoTpTxStatus::AwaitingFlowControl;
    else
        next_send_time_us_ = now_us > std::numeric_limits<std::uint64_t>::max() - stmin_us_
            ? std::numeric_limits<std::uint64_t>::max() : now_us + stmin_us_;
    return consecutive;
}

std::vector<CanFrame> isoTpSegment(const std::vector<std::uint8_t>& payload,
                                   std::uint32_t transmit_id) {
    CanFrame id_check{};
    id_check.id = transmit_id;
    id_check.dlc = 0U;
    id_check.validate();
    if (payload.empty() || payload.size() > 4095U) {
        throw std::invalid_argument("ISO-TP payload length must be between 1 and 4095 bytes");
    }
    std::vector<CanFrame> frames;
    if (payload.size() <= 7U) {
        CanFrame frame{};
        frame.id = transmit_id;
        frame.dlc = static_cast<std::uint8_t>(payload.size() + 1U);
        frame.data[0] = static_cast<std::uint8_t>(payload.size());
        std::copy(payload.begin(), payload.end(), frame.data.begin() + 1);
        frames.push_back(frame);
        return frames;
    }

    CanFrame first{};
    first.id = transmit_id;
    first.dlc = 8U;
    first.data[0] = static_cast<std::uint8_t>(0x10U | ((payload.size() >> 8U) & 0x0FU));
    first.data[1] = static_cast<std::uint8_t>(payload.size() & 0xFFU);
    std::copy_n(payload.begin(), 6, first.data.begin() + 2);
    frames.push_back(first);

    std::size_t offset = 6U;
    std::uint8_t sequence = 1U;
    while (offset < payload.size()) {
        CanFrame consecutive{};
        consecutive.id = transmit_id;
        consecutive.dlc = 8U;
        consecutive.data[0] = static_cast<std::uint8_t>(0x20U | sequence);
        const std::size_t count = std::min<std::size_t>(7U, payload.size() - offset);
        std::copy_n(payload.begin() + static_cast<std::ptrdiff_t>(offset), count,
                    consecutive.data.begin() + 1);
        frames.push_back(consecutive);
        offset += count;
        sequence = static_cast<std::uint8_t>((sequence + 1U) & 0x0FU);
    }
    return frames;
}

} // namespace autocore
