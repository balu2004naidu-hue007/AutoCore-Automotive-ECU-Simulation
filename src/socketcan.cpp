#include "autocore/socketcan.hpp"

#if defined(__linux__)
#include <cerrno>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>

namespace autocore {
namespace {
[[noreturn]] void throwSystemError(const char* operation) {
    throw std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
}

canid_t toLinuxCanId(const CanFrame& frame) {
    frame.validate();
    canid_t id = frame.id;
    if (frame.extended) id |= CAN_EFF_FLAG;
    if (frame.remote) id |= CAN_RTR_FLAG;
    return id;
}
}

SocketCanInterface::~SocketCanInterface() { close(); }

SocketCanInterface::SocketCanInterface(SocketCanInterface&& other) noexcept
    : socket_fd_(std::exchange(other.socket_fd_, -1)) {}

SocketCanInterface& SocketCanInterface::operator=(SocketCanInterface&& other) noexcept {
    if (this != &other) {
        close();
        socket_fd_ = std::exchange(other.socket_fd_, -1);
    }
    return *this;
}

void SocketCanInterface::open(const std::string& interface_name, bool loopback) {
    if (isOpen()) throw std::logic_error("SocketCAN interface is already open");
    if (interface_name.empty() || interface_name.size() >= IFNAMSIZ)
        throw std::invalid_argument("Invalid SocketCAN interface name");

    const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
    if (fd < 0) throwSystemError("socket(PF_CAN)");

    int loopback_value = loopback ? 1 : 0;
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK,
                     &loopback_value, sizeof(loopback_value)) < 0) {
        const int saved_errno = errno;
        ::close(fd);
        errno = saved_errno;
        throwSystemError("setsockopt(CAN_RAW_LOOPBACK)");
    }

    ifreq request{};
    std::memcpy(request.ifr_name, interface_name.c_str(), interface_name.size() + 1U);
    if (::ioctl(fd, SIOCGIFINDEX, &request) < 0) {
        const int saved_errno = errno;
        ::close(fd);
        errno = saved_errno;
        throwSystemError("ioctl(SIOCGIFINDEX)");
    }

    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        const int saved_errno = errno;
        ::close(fd);
        errno = saved_errno;
        throwSystemError("bind(SocketCAN)");
    }
    socket_fd_ = fd;
}

void SocketCanInterface::close() noexcept {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

void SocketCanInterface::send(const CanFrame& frame) {
    if (!isOpen()) throw std::logic_error("SocketCAN interface is not open");
    can_frame raw{};
    raw.can_id = toLinuxCanId(frame);
    raw.can_dlc = frame.dlc;
    if (!frame.remote) std::memcpy(raw.data, frame.data.data(), frame.dlc);
    const auto written = ::write(socket_fd_, &raw, sizeof(raw));
    if (written < 0) throwSystemError("write(SocketCAN)");
    if (written != static_cast<ssize_t>(sizeof(raw)))
        throw std::runtime_error("SocketCAN write returned a short frame");
}

std::optional<CanFrame> SocketCanInterface::receive(std::uint32_t timeout_ms) {
    if (!isOpen()) throw std::logic_error("SocketCAN interface is not open");
    pollfd descriptor{};
    descriptor.fd = socket_fd_;
    descriptor.events = POLLIN;
    const int bounded_timeout = timeout_ms > static_cast<std::uint32_t>(INT32_MAX)
        ? INT32_MAX : static_cast<int>(timeout_ms);
    int ready;
    do { ready = ::poll(&descriptor, 1, bounded_timeout); }
    while (ready < 0 && errno == EINTR);
    if (ready < 0) throwSystemError("poll(SocketCAN)");
    if (ready == 0) return std::nullopt;
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
        throw std::runtime_error("SocketCAN socket reported an error condition");
    if ((descriptor.revents & POLLIN) == 0) return std::nullopt;

    can_frame raw{};
    const auto count = ::read(socket_fd_, &raw, sizeof(raw));
    if (count < 0) throwSystemError("read(SocketCAN)");
    if (count != static_cast<ssize_t>(sizeof(raw)))
        throw std::runtime_error("SocketCAN read returned a short frame");
    if (raw.can_dlc > CAN_MAX_DLEN)
        throw std::runtime_error("SocketCAN received invalid Classic CAN DLC");

    CanFrame frame{};
    frame.extended = (raw.can_id & CAN_EFF_FLAG) != 0U;
    frame.remote = (raw.can_id & CAN_RTR_FLAG) != 0U;
    frame.id = frame.extended ? (raw.can_id & CAN_EFF_MASK) : (raw.can_id & CAN_SFF_MASK);
    frame.dlc = raw.can_dlc;
    if (!frame.remote) std::memcpy(frame.data.data(), raw.data, frame.dlc);
    frame.validate();
    return frame;
}

void SocketCanInterface::setFilters(
    const std::vector<std::pair<std::uint32_t, std::uint32_t>>& filters) {
    if (!isOpen()) throw std::logic_error("SocketCAN interface is not open");
    std::vector<can_filter> native_filters;
    native_filters.reserve(filters.size());
    for (const auto& filter : filters) {
        const std::uint32_t id = filter.first;
        const std::uint32_t mask = filter.second;
        if (id > CAN_EFF_MASK || mask > CAN_EFF_MASK)
            throw std::invalid_argument("SocketCAN filter ID or mask is out of range");
        if (id <= CAN_SFF_MASK) {
            if (mask > CAN_SFF_MASK)
                throw std::invalid_argument("Standard SocketCAN filter mask exceeds 11 bits");
            native_filters.push_back(can_filter{static_cast<canid_t>(id),
                static_cast<canid_t>(mask | CAN_EFF_FLAG)});
        } else {
            native_filters.push_back(can_filter{static_cast<canid_t>(id | CAN_EFF_FLAG),
                static_cast<canid_t>(mask | CAN_EFF_FLAG)});
        }
    }
    const void* data = native_filters.empty() ? nullptr : native_filters.data();
    const socklen_t length = static_cast<socklen_t>(native_filters.size() * sizeof(can_filter));
    if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, data, length) < 0)
        throwSystemError("setsockopt(CAN_RAW_FILTER)");
}

} // namespace autocore
#endif
