# Code map — where to inspect AutoCore v1.6.1

| File | What to study |
|---|---|
| `CMakeLists.txt` | CMake targets, C++17 settings, compiler warnings, sanitizers and CTest registration |
| `include/autocore/can_frame.hpp` | CAN frame structure, identifier/DLC validation and safe big-endian payload access |
| `include/autocore/can_bus.hpp`, `src/can_bus.cpp` | CAN transmit queue and frame-level arbitration ordering |
| `include/autocore/ecus.hpp`, `src/ecus.cpp` | Engine, Battery and Body ECU signal state and telemetry encoding |
| `include/autocore/can_controller.hpp`, `src/can_controller.cpp` | Per-node TEC/REC counters, Error Active/Error Passive/Bus-Off transitions and Bus-Off recovery model |
| `include/autocore/vehicle_network.hpp`, `src/vehicle_network.cpp` | Periodic ECU scheduling, per-controller transmit isolation, receive accounting, error injection and distinct-timestamp recovery scheduling |
| `include/autocore/frame_integrity.hpp`, `src/frame_integrity.cpp` | CRC-8/SAE-J1850 and rolling-counter encoding/verification |
| `include/autocore/isotp.hpp`, `src/isotp.cpp` | ISO-TP receiver/transmitter, segmentation, flow-control and timeout handling |
| `include/autocore/diagnostics.hpp`, `src/diagnostics.cpp` | Six DTC monitors, status lifecycle, timeout tracking, CRC/counter recovery, freeze frames and UDS service handling |
| `src/engine.cpp` | Persistent C++ simulation engine, drive cycle, dashboard commands, fault injection, event/frame/history buffers and JSON state serialization |
| `dashboard/server.py` | Loopback-only HTTP bridge, action allow-list, input validation, child-process lifecycle, health endpoint and CSV export |
| `dashboard/index.html` | Six-page dashboard UI, live charts, CAN monitor, Controller Health page, DTC catalogue, UDS controls, scenario lab and guided self-test |
| `tests/test_autocore.cpp` | C++ unit/regression assertions, protocol tests and 10,000-cycle network stress test |
| `tests/test_dashboard_integration.py` | Real localhost HTTP server plus compiled C++ engine workflow, including CAN Error Passive, Bus-Off, peer continuity and recovery |
| `scripts/build_and_test.ps1` | Windows dependency setup/build/test helper used by the batch launcher |
| `scripts/build_and_test.sh` | Linux/macOS-style CMake build/test helper |
| `docs/can_fault_confinement.md` | TEC/REC rules, controller-state transitions, Bus-Off isolation, recovery sequence, demo and limitations |
| `docs/` | Architecture, diagnostics, dashboard usage, verification, project report and presentation notes |
