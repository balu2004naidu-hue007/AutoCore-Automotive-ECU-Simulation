#include "autocore/vehicle_network.hpp"
#include "autocore/isotp.hpp"
#include <iomanip>
#include <iostream>

namespace {
void printActiveDtcs(const autocore::VehicleNetwork& vehicle) {
    bool found = false;
    for (const auto& entry : vehicle.diagnostics().dtcs()) {
        if (!entry.second.active) continue;
        if (found) std::cout << ", ";
        std::cout << entry.second.name;
        found = true;
    }
    if (!found) std::cout << "none";
    std::cout << '\n';
}

void printSnapshot(const char* label, const autocore::VehicleNetwork& vehicle) {
    std::cout << "\n[" << label << "]\n"
              << "  Engine: " << vehicle.engine().rpm() << " rpm, "
              << vehicle.engine().temperature() << " C, "
              << vehicle.engine().speed() << " km/h\n"
              << "  Battery: " << std::fixed << std::setprecision(2)
              << static_cast<double>(vehicle.battery().voltageMv()) / 1000.0 << " V\n"
              << "  CAN frames transmitted: " << vehicle.canFramesTransmitted() << '\n'
              << "  Active DTCs: ";
    printActiveDtcs(vehicle);
}
}

int main() {
    using namespace autocore;
    VehicleNetwork vehicle;

    std::cout << "AutoCore | Automotive Multi-ECU Communication and Diagnostics Platform\n"
              << "=====================================================================\n"
              << "Four logical ECUs | Classic CAN | ISO-TP | UDS | CRC + rolling counters\n";

    vehicle.engine().setRpm(1850U);
    vehicle.engine().setTemperature(92);
    vehicle.engine().setSpeed(48U);
    vehicle.battery().setVoltageMv(12450U);
    vehicle.battery().setCurrentMa(2300);
    vehicle.body().setHeadlights(true);
    vehicle.tick(0U);
    printSnapshot("Normal operation", vehicle);

    vehicle.engine().setTemperature(125);
    vehicle.battery().setVoltageMv(10500U);
    vehicle.tick(100U);
    printSnapshot("Sensor fault injection", vehicle);

    vehicle.engine().setTemperature(90);
    vehicle.battery().setVoltageMv(12500U);
    vehicle.tick(200U);
    printSnapshot("Sensor recovery", vehicle);

    vehicle.setEngineOnline(false);
    vehicle.tick(800U);
    printSnapshot("Engine ECU communication loss", vehicle);

    vehicle.setEngineOnline(true);
    vehicle.tick(900U);
    printSnapshot("Engine ECU communication recovery", vehicle);

    CanFrame uds_request{};
    uds_request.id = 0x7E0U;
    uds_request.dlc = 4U;
    uds_request.data[0] = 0x03U;
    uds_request.data[1] = 0x22U;
    uds_request.data[2] = 0xF4U;
    uds_request.data[3] = 0x0CU;
    const auto uds_response = vehicle.requestUds(uds_request, 910U);
    std::cout << "\n[UDS ReadDataByIdentifier F40C]\n  Response CAN ID: 0x"
              << std::hex << uds_response.id << " | bytes:";
    for (std::size_t i = 0; i < uds_response.dlc; ++i)
        std::cout << ' ' << std::setw(2) << std::setfill('0')
                  << static_cast<unsigned>(uds_response.data[i]);
    std::cout << std::dec << "\n";

    const auto freeze_request = isoTpSegment(
        {0x19U, 0x04U, 0x00U, 0x02U, 0x17U, 0x01U}, 0x7E0U);
    const auto freeze_response = vehicle.requestUdsIsoTp(freeze_request, 920U);
    std::cout << "\n[UDS ReadDTCFreezeFrame P0217]\n  ISO-TP response frames: "
              << freeze_response.size() << "\n";

    const auto clear_response = vehicle.requestUdsPayload(
        {0x14U, 0xFFU, 0xFFU, 0xFFU}, 930U);
    std::cout << "\n[UDS ClearDiagnosticInformation]\n  Positive response: "
              << (!clear_response.empty() && clear_response[0] == 0x54U ? "0x54" : "not received")
              << "\n  Stored freeze frames after clear: "
              << vehicle.diagnostics().freezeFrames().size() << "\n\nDemo complete.\n";
    return 0;
}
