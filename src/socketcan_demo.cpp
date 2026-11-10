#include "autocore/socketcan.hpp"

#if defined(__linux__)
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    const std::string interface_name = argc > 1 ? argv[1] : "vcan0";
    try {
        autocore::SocketCanInterface receiver;
        autocore::SocketCanInterface transmitter;
        receiver.open(interface_name);
        transmitter.open(interface_name);
        receiver.setFilters({{0x321U, 0x7FFU}});

        autocore::CanFrame outgoing{};
        outgoing.id = 0x321U;
        outgoing.dlc = 4U;
        outgoing.data[0] = 0xA1U;
        outgoing.data[1] = 0xB2U;
        outgoing.data[2] = 0xC3U;
        outgoing.data[3] = 0xD4U;
        transmitter.send(outgoing);

        const auto incoming = receiver.receive(1000U);
        if (!incoming) {
            std::cerr << "Timed out waiting for CAN loopback on " << interface_name << '\n';
            return 2;
        }
        std::cout << "SocketCAN loopback PASS on " << interface_name << " | ID 0x"
                  << std::hex << incoming->id << " | DLC " << std::dec
                  << static_cast<unsigned>(incoming->dlc) << " | data";
        for (std::size_t i = 0; i < incoming->dlc; ++i)
            std::cout << ' ' << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<unsigned>(incoming->data[i]);
        std::cout << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "SocketCAN demo error: " << error.what() << '\n';
        return 1;
    }
}
#else
int main() { return 2; }
#endif
