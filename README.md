# AutoCore — Automotive Multi-ECU Communication & Diagnostics Platform

**AutoCore v1.6.1** is a laptop-based C++17 simulation of a small automotive electronic network. It models four logical ECUs communicating through a virtual Classic CAN network and provides diagnostic services, fault injection, CAN controller fault confinement, Bus-Off recovery, and a local interactive dashboard.

> **Scope:** This is an educational software simulation. It is not connected to a real vehicle, does not implement the electrical/bit-level CAN physical layer, and is not an OEM-certified diagnostic stack.

## Download the complete source

The complete Windows-ready source package is included in this repository:

- **[AutoCore v1.6.1 Windows source ZIP](AutoCore_v1.6.1_windows_startup_fix.zip)**

The ZIP contains the complete C++ source tree, headers, dashboard, Python local server, tests, configuration, Windows launcher, scripts, documentation, gitignore file, and MIT license.

## Features

### Four simulated ECUs

| ECU | Simulated responsibilities |
|---|---|
| Engine ECU | RPM, coolant temperature, vehicle speed |
| Battery ECU | Voltage, current, battery temperature |
| Body ECU | Headlights and door state |
| Diagnostics ECU | Communication monitoring, DTC storage, UDS requests |

### Communication and diagnostics

- Classic CAN frame model with standard and extended identifiers
- Identifier-based frame arbitration
- Runtime-loaded CAN message and signal configuration
- CAN monitor filters
- CRC-8/SAE-J1850 frame integrity checks
- Rolling-counter freshness monitoring
- ISO-TP single-frame and multi-frame transport helpers
- Selected UDS services: Diagnostic Session Control (0x10), ECU Reset (0x11), Clear DTC Information (0x14), Read DTC Information (0x19), Read Data By Identifier (0x22), and Tester Present (0x3E)
- DTC lifecycle with pending, confirmed, and recovered states
- Freeze-frame data capture

### CAN controller fault confinement (advanced feature)

Each simulated ECU has independent transmit and receive error counters (TEC/REC) and controller states:

- Error Active
- Error Passive
- Bus-Off
- Bus-Off isolation and controlled recovery progress

The controller state model uses discrete simulation events. It is intended to teach and demonstrate CAN fault-confinement concepts, not to reproduce bit timing or all physical-controller implementation details.

### Fault injection and dashboard

- Six diagnostic scenarios: engine overtemperature, battery undervoltage, Engine ECU communication loss, Battery ECU communication loss, CRC corruption, and rolling-counter error
- TEC/REC error injection and controller-state monitoring
- Local browser dashboard with Overview, CAN Network, CAN Controller Health, Diagnostics, Scenario Lab, and Event Log views
- Telemetry history export

## Architecture

```text
Browser dashboard
      |
      | HTTP on 127.0.0.1
      v
Python local dashboard server
      |
      | JSON command/state interface
      v
C++17 simulation engine
      |
      +-- Engine ECU
      +-- Battery ECU
      +-- Body ECU
      +-- Diagnostics ECU
      |
      v
Virtual Classic CAN network
```

## Windows quick start

Requirements:
- Windows 10 or Windows 11
- Internet access for first-time dependency setup
- Visual Studio C++ Build Tools / MSVC
- CMake
- Python 3.10 or newer
- A modern browser (Chrome or Edge)

1. Download and extract the ZIP.
2. Open the extracted AutoCore folder.
3. Double-click Run_AutoCore.bat.
4. Allow the setup script to install/check missing build tools if prompted.
5. Wait for configuration, compilation, and tests to finish.
6. Open **http://127.0.0.1:8765/** in your browser if it does not open automatically.
7. Keep the launcher terminal open while using the dashboard. Press Ctrl+C in it to stop the application.

Do not open dashboard/index.html directly. The page requires the local Python server and running C++ simulation engine.

## Guided demonstration

1. **Overview:** confirm the four ECUs are online and telemetry changes.
2. **CAN Network:** inspect frame identifiers, payloads, timestamps, and filters.
3. **Scenario Lab:** run the guided diagnostic self-test; expected result is 7/7 PASS.
4. **Diagnostics:** inspect DTC records and request freeze-frame data.
5. **CAN Controller Health:** select Engine ECU and inject 24 transmit errors to reach Error Passive from a clean counter state.
6. Reset the simulation/counters, then inject 32 transmit errors to reach Bus-Off.
7. Confirm Engine ECU frames stop while Battery and Body messages continue.
8. Disable continuous transmit-error injection and start Bus-Off recovery.
9. Observe recovery progress to 128/128, Error Active state, and resumed Engine ECU transmission.
10. **Event Log:** review the timeline of error, isolation, diagnostics, and recovery events.

The TEC/REC values and recovery progression are simulated. The 128-step recovery counter is a discrete educational model rather than bit-accurate observation of a physical CAN bus.

## Verification

The v1.6 development verification included:
- GCC Release build with warnings treated as errors
- Clang Release build with warnings treated as errors
- AddressSanitizer and UndefinedBehaviorSanitizer runs
- C++ regression tests
- HTTP-to-C++ integration tests
- Fresh ZIP extraction and build checks

The v1.6.1 package adds a more tolerant dashboard startup readiness window and improved server-log reporting for Windows launch diagnosis. The project was also run successfully by the author on a Windows laptop.

Test counts from the v1.6.0 development verification: 122 C++ regression assertions and 17 HTTP-to-C++ integration groups. These are project test counts, not a claim of formal standards conformance.

## Source layout

After extracting the package:

```text
AutoCore/
├── CMakeLists.txt
├── Run_AutoCore.bat
├── README.md
├── VERSION.txt
├── include/autocore/     # C++ interfaces
├── src/                  # Simulation engine and protocol modules
├── dashboard/            # Browser UI and local Python server
├── tests/                # C++ and HTTP integration tests
├── config/               # CAN message/signal configuration
├── scripts/              # Build and optional SocketCAN helpers
└── docs/                 # Architecture, diagnostics, report and guides
```

## License

MIT License. See LICENSE inside the source package.

## Disclaimer

AutoCore is an educational simulation and prototyping project. It has not been validated against a physical vehicle, production CAN controller, OEM requirements, or automotive safety/certification processes. Do not use it to control or diagnose a real vehicle.
