# AutoCore v1.6.1 — Automotive Multi-ECU Communication, Diagnostics and CAN Controller Fault Confinement

## Abstract

AutoCore is a laptop-based educational automotive embedded-systems simulator implemented in C++17. It models Engine, Battery, Body and Diagnostics electronic control units on an in-process Classic CAN network. ECU telemetry is validated using CRC-8/SAE-J1850 and rolling counters. The Diagnostics ECU monitors sensor thresholds, communication timeouts and telemetry integrity; it stores diagnostic trouble codes and first-fault snapshots. Selected UDS services use ISO-TP request/response helpers.

AutoCore v1.6.1 adds one software CAN controller per ECU. Each controller tracks TEC/REC, transmit/receive statistics, Error Active, Error Passive and Bus-Off states. Fault injection can drive a node into Error Passive or Bus-Off. A Bus-Off node stops transmitting while healthy peers continue; the Diagnostics ECU can detect Engine/Battery communication loss. A visible 128-observation recovery model returns the node to Error Active.

The persistent C++ simulation engine is the source of truth. A Python standard-library server binds to `127.0.0.1`, hosts the browser dashboard and relays validated operator commands to the C++ process. Six dashboard pages expose live telemetry, CAN frames, controller health, diagnostics, scenarios and events.

## Problem statement and purpose

Vehicle functions are distributed across ECUs. Engineers must reason about periodic data exchange, network loading, signal integrity, controller error confinement, diagnostic trouble codes and recovery. Physical vehicle and ECU test benches may be expensive or unavailable to students. AutoCore provides a repeatable laptop-based environment to study these workflows without a physical vehicle, ECU or CAN transceiver.

The project is intended to demonstrate embedded software architecture, CAN frame handling, controller-level fault confinement, selected ISO-TP/UDS concepts, DTC lifecycle, fault injection, recovery and test automation. It is not intended to control or diagnose a real vehicle.

## Objectives and implementation

1. Implement independent Engine, Battery and Body signal producers plus a Diagnostics ECU in C++17.
2. Model Classic CAN frame validation and frame-level arbitration.
3. Schedule 100 ms telemetry and detect missing ECU messages.
4. Implement ISO-TP segmentation/reassembly and flow-control helpers.
5. Implement selected UDS session, reset acknowledgement, DTC, DID, clear-DTC and Tester Present services.
6. Detect overtemperature, undervoltage, CRC and rolling-counter faults; store first-fault snapshots.
7. Load and validate a runtime CSV message/signal matrix and provide monitor-side ID/mask filtering.
8. Qualify DTCs as pending and confirm them after three distinct consecutive monitor observations.
9. Implement independent per-ECU TEC/REC models and Error Active/Error Passive/Bus-Off state transitions.
10. Isolate Bus-Off transmitters while keeping healthy peer ECUs operating.
11. Provide error injection and a repeatable 128-observation Bus-Off recovery workflow.
12. Connect the C++ engine to a live browser dashboard through a localhost-only Python bridge.
13. Verify the implementation with C++ regressions and real HTTP-to-C++ integration tests.

## System architecture

```text
Browser dashboard (HTML / JavaScript)
       | HTTP JSON on 127.0.0.1
       v
Python standard-library localhost bridge
       | JSON-lines commands and state
       v
Persistent C++17 simulation engine
       |
       v
VehicleNetwork + frame-level Classic CAN bus
   |          |          |             |
Engine ECU  Battery ECU  Body ECU  Diagnostics ECU
   |          |          |             |
   +----------+----------+-------------+
              Per-ECU controller models
              TEC / REC / Error Active / Error Passive / Bus-Off
```

The C++ process owns the simulation clock, ECU signals, CAN controller state, frames, DTC state, freeze frames and event history. The Python server hosts HTML, validates actions against an allow-list, forwards JSON-line commands to the engine and exports telemetry CSV. The browser polls state every 500 ms; the server advances simulation every 100 ms while running.

## ECU and CAN design

| ECU | Identifier | Signals / responsibilities |
|---|---:|---|
| Engine | `0x100` | RPM, coolant temperature, vehicle speed |
| Battery | `0x110` | Voltage, current, temperature |
| Body | `0x120` | Headlights and door state |
| Diagnostics | Request `0x7E0`, response `0x7E8` | Telemetry supervision, DTCs, freeze frames and UDS |

Telemetry uses eight-byte Classic CAN frames. Engine and Battery payloads contain raw signals, a rolling counter and CRC. The C++ engine loads `config/can_messages.csv`, validates five message definitions and sixteen signal layouts, and maps frame IDs to symbolic message names in the dashboard. The 11-bit ID/mask filter is monitor-only; it does not block Diagnostics ECU input.

## CAN controller fault confinement

Each of the four logical ECUs owns an independent software `CanController`.

| State | Modeled condition | Behavior |
|---|---|---|
| Error Active | TEC < 128 and REC < 128 | Normal participation in the virtual network |
| Error Passive | TEC >= 128 or REC >= 128, while TEC < 256 | Node can continue communicating |
| Bus-Off | TEC >= 256 | Node is prevented from transmitting until recovery |

Counter updates in this model are: TX error +8 TEC; successful TX -1 TEC when nonzero; RX error +1 REC; successful RX decrements REC, with REC above 127 reduced to 127 on the next successful receive. TEC is capped at 256 and REC at 255. Lifetime counters saturate rather than wrap.

The dashboard supports 24-event and 32-event TX bursts, a 128-event RX burst, persistent TX/RX injectors and recovery. From a clean baseline, 24 TX errors produce TEC 192 and Error Passive; 32 TX errors produce TEC 256 and Bus-Off. A Bus-Off Engine ECU stops publishing `0x100`; Battery and Body continue. Missing Engine telemetry can then produce U0100. Recovery requires every continuous TX injector to be off and advances once per distinct 100 ms scheduler tick. At 128 observations, TEC/REC reset and the controller returns to Error Active.

This is a discrete educational representation of CAN fault confinement. The 100 ms tick is not physical time for eleven recessive bit periods. No bit-level timing, ACK/error flags, bus voltage, wiring, sample point or physical controller is modeled.

## Diagnostics and transport

ISO-TP helpers implement Single, First and Consecutive Frames, reassembly, sequence checking, receiver timeout and stateful flow-control handling (Continue-To-Send, WAIT, OVERFLOW, block size and STmin). Dashboard UDS actions use an in-process transaction helper and do not simulate real-time bus pacing.

Selected UDS services:

- `0x10` Diagnostic Session Control (default and extended sessions).
- `0x11` ECU Reset acknowledgement (simulated only).
- `0x14` Clear Diagnostic Information for the modeled all-DTC group.
- `0x19` Read DTC Information (status-mask count, records and freeze-frame record `0x01`).
- `0x22` selected DIDs: `F40C` RPM, `F405` coolant temperature, `F411` battery voltage.
- `0x3E` Tester Present.

Six DTC monitors:

| DTC | Detection condition |
|---|---|
| `P0217` | Coolant temperature > 115 °C |
| `P0562` | Battery voltage < 11.0 V |
| `U0100` | Engine telemetry timeout > 500 ms after presence established |
| `U0111` | Battery telemetry timeout > 500 ms after presence established |
| `AC0001` | CRC-8/SAE-J1850 mismatch |
| `AC0002` | Unexpected modulo-256 rolling-counter successor |

`AC0001` and `AC0002` are project-specific educational codes. All monitors use three consecutive distinct-simulation-timestamp failed observations: first pending, third confirmed. Multiple ECU frames at one timestamp count as one sample. Recovery clears current/pending status; simplified confirmed/history status remains until service `0x14` clears the record store. The first pending transition captures a snapshot of RPM, temperature, speed, battery voltage and simulation time.

## Dashboard and demonstration

The six dashboard views are:

1. Overview — live signals, charts, ECU availability, current fault count and event preview.
2. CAN Network — frames, IDs, symbolic message names, DLC, payload, CRC, message matrix, ID/mask filter and CSV export.
3. CAN Controller Health — per-ECU TEC/REC, state, TX/RX counters, error injectors, Bus-Off and recovery progress.
4. Diagnostics — UDS console, DTC records, status and freeze-frame reading.
5. Scenario Lab — start/pause/reset, throttle/body controls, six DTC fault injectors and guided seven-group diagnostic self-test.
6. Event Log — timestamped actions, UDS requests, DTC transitions and controller Error Passive/Bus-Off/recovery events.

The diagnostic self-test exercises the six DTC monitors plus a UDS group and expects 7/7 PASS. The CAN controller fault-confinement workflow is a separate demonstration: reset baseline, inject 24 TX errors to reach Error Passive, reset, inject 32 TX errors to reach Bus-Off, verify peer continuity and U0100, disable TX injectors, start recovery and watch 128/128, then confirm Engine telemetry resumes. A 128 RX-error burst shows REC-driven Error Passive.

## Build and execution

Windows: extract the ZIP and double-click `Run_AutoCore.bat`. The bootstrap checks CMake, Visual Studio C++ Build Tools and Python 3.10+, attempts installation through `winget`, configures `build-windows-v1.6.1`, builds, runs CTest and launches `http://127.0.0.1:8765/`. First-time dependency installation needs an internet connection and may request UAC approval. The native Windows build and first launch have not been executed in the Linux development environment; they remain the target-laptop acceptance check.

Linux:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-Werror
cmake --build build --parallel
ctest --test-dir build --output-on-failure
python3 dashboard/server.py --engine ./build/autocore_engine --open
```

## Verification summary

The v1.6.1 source was built from a clean configuration with GCC Release and Clang Release, with warnings treated as errors. The C++ regression executable produced 122 passing assertions. Both CTest targets passed under GCC, Clang and GCC AddressSanitizer/UndefinedBehaviorSanitizer. The actual Python localhost server and compiled C++ engine integration test passed 17 groups, including TX/RX Error Passive, Bus-Off isolation, continued peer ECU traffic, U0100 qualification, 128-step recovery and invalid API input handling. Python server/test syntax and dashboard JavaScript syntax were checked separately. The exact release ZIP was rebuilt and extracted for a separate compile/test pass.

A full visual Chromium session with the live local server could not be verified in this development container: headless Chromium did not complete the local navigation within the available execution window. Native Windows installation/MSVC and browser operation on the target laptop also remain unverified. These boundaries are documented in `verification_report.md`.

## Limitations and safety

No physical CAN transceiver, ECU or vehicle was connected. Frame arbitration is complete-frame-level; bit timing, bit stuffing, ACK slots, physical error flags and electrical behavior are not modeled. TEC/REC and controller transitions model principal fault-confinement concepts, not every hardware-specific CAN controller exception. Bus-Off recovery counts discrete observations rather than physical recessive bits. Selected UDS/ISO-TP behavior only; no ISO/OEM certification is claimed. `0x11` is a simulated acknowledgement. Do not use AutoCore to control or diagnose a real vehicle.
