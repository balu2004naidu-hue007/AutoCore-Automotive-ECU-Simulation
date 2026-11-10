# AutoCore v1.6.1

## Automotive Multi-ECU Communication, Diagnostics and CAN Fault-Confinement Platform

AutoCore is a laptop-based educational automotive embedded-systems simulator implemented in portable C++17. It models four logical ECUs on an in-process Classic CAN network and provides a live browser dashboard connected to the actual C++ simulation engine through a localhost-only Python bridge.

The project covers periodic ECU telemetry, frame arbitration, a runtime CAN message matrix, CRC and rolling-counter checks, ISO-TP helpers, a selected UDS diagnostic subset, DTC monitoring and freeze frames, fault injection, and **CAN controller fault confinement with Error Active, Error Passive, Bus-Off and recovery states**.

> **Scope:** AutoCore is an educational host-side simulation. It does not connect to a real vehicle, physical CAN controller or ECU. It is not an OEM diagnostic tool and is not ISO-conformance certified. The controller recovery sequence is a discrete approximation, not a bit-accurate electrical CAN implementation.

## 1. Purpose and problem solved

Modern vehicles distribute sensing, actuation and supervision across multiple Electronic Control Units (ECUs). ECUs exchange periodic CAN messages. Embedded engineers need to understand not only how data moves across the network, but also how faults are detected, how a faulty controller is isolated, how diagnostic trouble codes are reported, and how communication is restored.

Building a physical test bench can require ECUs, transceivers, CAN interfaces, wiring and diagnostic equipment. AutoCore provides a reproducible laptop-based laboratory for demonstrating those concepts without buying vehicle hardware. You can run normal traffic, inject sensor and communication faults, observe TEC/REC counters and controller state transitions, inspect UDS/DTC responses, recover a Bus-Off node, and export telemetry.

The project is for learning and demonstration. It must not be used to diagnose, control or make safety decisions about a real vehicle.

## 2. Release 1.6.1 highlights

- Windows startup robustness: dashboard integration readiness window increased from 3 to 20 seconds to accommodate first-run executable scanning, with captured server logs reported immediately if startup exits.

- Adds an independent CAN controller model for Engine, Battery, Body and Diagnostics ECUs.
- Tracks Transmit Error Counter (TEC), Receive Error Counter (REC), transmit/receive successes and errors, controller state and Bus-Off recovery progress.
- Models Error Passive when TEC or REC reaches 128; models Bus-Off when TEC reaches 256.
- Bus-Off ECUs stop transmitting. Other non-Bus-Off ECUs continue to communicate.
- Adds transmit/receive error bursts and continuous error-injection controls for each simulated controller.
- Adds a Bus-Off recovery action requiring all continuous TX error injectors to be disabled; the dashboard and localhost API reject premature recovery requests. Recovery completes after 128 distinct eligible virtual scheduler-cycle observations.
- Bus-Off of Engine/ Battery can trigger the existing U0100/U0111 communication diagnostic monitors after their timeout and three-sample qualification.
- Adds a dedicated **CAN Controller Health** dashboard page with four-controller state summary, TEC/REC counters, injection status, TX/RX statistics and recovery progress.
- Adds controller state-transition events to the Event Log.
- Retains the previous runtime CAN message database, ID/mask monitor filter, six DTC monitors, selected UDS services, ISO-TP helpers, guided diagnostic self-test and CSV export.

## 3. System architecture

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
              Per-ECU CAN controller models
              TEC / REC / Error Active / Error Passive / Bus-Off
```

The C++ process is the single source of truth. The Python server hosts the dashboard, validates a fixed set of HTTP actions and forwards commands to the persistent C++ process. The browser polls the engine state and sends user actions; it does not invent its own vehicle telemetry.

## 4. Simulated ECUs

| ECU | Telemetry / responsibility | CAN identifiers |
|---|---|---|
| Engine ECU | RPM, coolant temperature, vehicle speed | `0x100` |
| Battery ECU | Voltage, current, battery temperature | `0x110` |
| Body ECU | Headlights and driver-door state | `0x120` |
| Diagnostics ECU | Network supervision, DTCs, freeze frames and UDS services | Request `0x7E0`, response `0x7E8` |

ECU telemetry is scheduled every 100 ms of simulation time. The drive-cycle model changes engine and vehicle signals and responds to throttle demand.

## 5. CAN network and controller fault confinement

### Classic CAN network features

- Standard 11-bit and extended 29-bit identifier frame structures.
- Classic CAN DLC validation and payload limit of eight bytes.
- Frame-level arbitration ordering.
- CRC-8/SAE-J1850 validation for the modeled telemetry payloads.
- Eight-bit rolling-counter sequence checks.
- Runtime CSV message matrix loaded from `config/can_messages.csv`: five message definitions and sixteen signal entries.
- CAN Network dashboard ID/mask filter for the monitor display. This filter does not block messages from the Diagnostics ECU.

### Controller states and thresholds

Each simulated ECU has its own controller counters and state:

| Controller state | Modeled condition | Network behavior |
|---|---|---|
| Error Active | TEC < 128 and REC < 128 | Normal simulated participation |
| Error Passive | TEC >= 128 or REC >= 128, while TEC < 256 | May continue to transmit and receive |
| Bus-Off | TEC >= 256 | Removed from simulated transmission until recovery |

Modeled counter updates:

- A transmit error adds 8 to TEC.
- A successful transmission decrements TEC by 1 when TEC is nonzero.
- A receive error adds 1 to REC.
- A successful receive decrements REC by 1; if REC is above 127, the modeled successful receive reduces it to 127.
- TEC is capped at 256 and REC at 255.

These rules demonstrate the principal fault-confinement transitions. They are not a complete reproduction of every CAN controller exception, bit-level error-flag rule or hardware implementation detail.

### Bus-Off recovery model

The recovery control is available only when the selected controller is Bus-Off and every continuous TX error injector has been disabled. The simulator then counts 128 eligible recovery observations. One distinct 100 ms virtual scheduler cycle represents one modeled observation of an 11-recessive-bit idle sequence. Repeated calls at the same simulation timestamp do not increment progress. On the 128th observation the model resets TEC/REC, returns the controller to Error Active and records a recovery event.

This makes the controller lifecycle visible and repeatable in a laptop simulation. It does **not** simulate the actual electrical bus, recessive-bit timing, ACK slots, error flags or a physical transceiver.

## 6. Diagnostics and transport

### ISO-TP helpers

The transport library includes Single Frame, First Frame and Consecutive Frame segmentation/reassembly, sequence checks, timeout handling and flow-control helpers for Continue-To-Send, WAIT, OVERFLOW, block size and STmin. Dashboard diagnostic actions use an in-process request/response helper; real-time bus pacing is not modeled.

### Selected UDS services

| SID | Service | Demonstration |
|---|---|---|
| `0x10` | Diagnostic Session Control | Default / extended session selection |
| `0x11` | ECU Reset | Simulated acknowledgement only |
| `0x14` | Clear Diagnostic Information | Clears the modeled DTC group |
| `0x19` | Read DTC Information | DTC count, records and freeze-frame snapshot |
| `0x22` | Read Data By Identifier | RPM `F40C`, coolant `F405`, battery voltage `F411` |
| `0x3E` | Tester Present | Tester-present timestamp and suppress-positive-response behavior |

### DTC monitors

| DTC | Condition |
|---|---|
| `P0217` | Engine coolant temperature above 115 °C |
| `P0562` | Battery system voltage below 11.0 V |
| `U0100` | Engine ECU telemetry timeout (>500 ms after presence established) |
| `U0111` | Battery ECU telemetry timeout (>500 ms after presence established) |
| `AC0001` | CRC-8/SAE-J1850 mismatch |
| `AC0002` | Rolling counter is not the expected modulo-256 successor |

`AC0001` and `AC0002` are AutoCore-specific demonstration codes, not manufacturer-assigned DTCs. Each monitor uses a three-consecutive-distinct-sample qualification policy: sample one is Pending 1/3, sample two is Pending 2/3, sample three is Confirmed. Multiple frames at the same simulation timestamp count as one observation. Recovery clears current/pending failure status while retaining simplified history until UDS service `0x14` clears the records. The first pending transition captures the modeled operating values and simulation timestamp.

A Bus-Off Engine or Battery ECU stops telemetry. The Diagnostics ECU then detects the missing ECU after its communication timeout and qualifies `U0100` or `U0111` using the same DTC sample policy. When the node recovers and valid telemetry returns, the active communication DTC recovers.

## 7. Dashboard pages and what to inspect

The dashboard has six pages:

1. **Overview** — live RPM, coolant temperature, speed, battery values, time-series charts, ECU online status, frame totals and recent events.
2. **CAN Network** — frame timestamp, CAN ID, symbolic message name, DLC, payload bytes, CRC status, loaded message matrix and standard 11-bit ID/mask monitor filter.
3. **CAN Controller Health** — active/passive/Bus-Off summary; select an ECU; inspect TEC, REC, TX/RX counters; inject TX/RX errors; start Bus-Off recovery; monitor the 128-step progress bar.
4. **Diagnostics** — selected UDS services, raw request/response, DTC table, pending/confirmed/recovered status, six-monitor catalogue and freeze-frame reading.
5. **Scenario Lab** — start/pause/reset, throttle, body controls, six sensor/communication/integrity injectors and the guided 7-group diagnostic self-test.
6. **Event Log** — timestamped operator commands, DTC transitions, controller Error Passive/Bus-Off transitions and successful recovery events.

### Recommended guided demonstration (existing diagnostics)

1. Open `http://127.0.0.1:8765/` and observe the changing telemetry.
2. Click **Run self-test**. Do not operate other controls while it is running.
3. Wait for **7 / 7 PASS**. It exercises the six DTC monitors and a UDS service group, then restores the simulation.
4. Open Diagnostics. Recovered historical records may be visible while Active Faults is zero; this is the healthy post-test state.
5. Click **Inject P0217 + P0562**. Observe Pending and then Confirmed status after three distinct monitor samples.
6. Read DTC records and read the P0217 freeze frame.
7. Click **Recover all**, then use **Clear Diagnostic Information (0x14)** if you want to clear historical records too.

### Recommended CAN controller fault-confinement demonstration

Start from a reset, healthy simulation. Open **CAN Controller Health**.

**A. Error Passive by transmit errors**
1. Select **Engine ECU**.
2. Click **24 TX errors · Error Passive**. Each modeled TX error adds 8, so TEC reaches 192 (above the 128 threshold).
3. The Engine controller should show `ERROR_PASSIVE`. It may still transmit; successful transmissions reduce TEC by 1 over time.

**B. Bus-Off and network isolation**
1. To make the outcome deterministic, use **Reset simulation + counters** before the Bus-Off demonstration, then select Engine ECU.
2. Click **32 TX errors · Bus-Off**. Thirty-two modeled errors add 256 to TEC.
3. Engine ECU should show `BUS_OFF`, and the Engine ECU online state should become Offline.
4. Open CAN Network. Engine telemetry should stop. Battery and Body frames should continue to appear.
5. Open Diagnostics after the communication timeout/qualification interval. `U0100` should become Pending and then Confirmed.

**C. Bus-Off recovery**
1. Return to CAN Controller Health and keep Engine selected.
2. Ensure all continuous TX injectors are disabled. If any are enabled, switch them off.
3. Click **Start Bus-Off recovery**.
4. Inspect the recovery counter: it should advance from 0 to 128, one observation per distinct 100 ms simulation cycle (approximately 12.8 seconds of virtual running time).
5. At 128/128 the Engine controller should return to Error Active and the Engine ECU should resume transmitting.
6. After valid telemetry returns, `U0100` should recover; its simplified historical entry remains until DTC clear.
7. Check Event Log for Bus-Off and recovery transition messages.

**D. Receive error confinement**
1. Reset the simulation, select Battery ECU and click **128 RX errors · Error Passive**.
2. REC should reach 128 and the Battery controller should enter Error Passive.
3. Subsequent valid received frames reduce the modeled REC; inspect the counter and state.

**E. Continuous error injection**
1. Select an ECU and enable continuous TX errors. Each scheduled transmit attempt adds 8 to TEC; the controller eventually enters Bus-Off and stops transmitting.
2. Disable the injector before starting recovery.
3. Continuous RX error injection adds 1 to REC for each received frame; disable it to allow normal receive success to reduce REC.

The CAN controller model and the guided DTC self-test are separate demonstrations. Use **Reset simulation + counters** between scenarios when you need a clean baseline.

## 8. Windows installation, build and launch

Requirements: Windows 10/11, internet for first-time bootstrap, Windows Package Manager (`winget`), CMake, Visual Studio 2022 C++ Build Tools and Python 3.10+. The launcher attempts to install missing tools through `winget`; installer/UAC approval may be required.

1. Download `AutoCore_v1.6.1_complete_windows.zip`.
2. Extract it into a new folder, e.g. `Documents\Projects\AutoCore_v1.6.1`.
3. Open the extracted `AutoCore` folder.
4. Double-click `Run_AutoCore.bat`.
5. Allow the first-time dependency setup to run. If Windows requests a restart, restart and run the batch file again.
6. The launcher configures `build-windows-v1.6.1`, builds the C++ code and runs CTest.
7. On success it starts the C++ engine and Python dashboard server, then opens the browser.
8. Dashboard URL: `http://127.0.0.1:8765/`. Keep the terminal window open; press Ctrl+C there to stop the application.

Do not double-click `dashboard\index.html`. It must be served by the local Python bridge so its API requests can reach the C++ process.

### View and understand the source code

Open VS Code → **File → Open Folder** → select the extracted `AutoCore` folder.

| File | What to look for |
|---|---|
| `src/can_controller.cpp` | TEC/REC counter updates, thresholds and Bus-Off recovery state machine |
| `include/autocore/can_controller.hpp` | Controller state and snapshot definitions |
| `src/vehicle_network.cpp` | Per-ECU controller ownership, transmit isolation, receive accounting and recovery scheduling |
| `src/engine.cpp` | Dashboard commands, events and controller JSON state |
| `dashboard/server.py` | Validated HTTP action routing to the C++ engine |
| `dashboard/index.html` | CAN Controller Health page, fault-injection controls and live register table |
| `tests/test_autocore.cpp` | Counter thresholds, state transitions, Bus-Off isolation and recovery edge cases |
| `tests/test_dashboard_integration.py` | Real localhost HTTP-to-C++ Bus-Off/recovery workflow |

## 9. Build and test manually (optional)

From PowerShell in the project folder, after CMake, MSVC and Python are installed:

```powershell
cmake -S . -B build-windows-v1.6.1 -DCMAKE_BUILD_TYPE=Release
cmake --build build-windows-v1.6.1 --config Release --parallel
ctest --test-dir build-windows-v1.6.1 -C Release --output-on-failure
python dashboard/server.py --engine build-windows-v1.6.1/Release/autocore_engine.exe --open
```

The launcher also checks the single-configuration executable path when a generator does not use a `Release` subfolder.

## 10. Verification and engineering boundaries

See `docs/verification_report.md` for the test matrix. The release source is built and tested in the available Linux environment; the exact Windows launcher has not been run on the user's physical Windows laptop. The native MSVC build and first-time `winget` bootstrap remain target-machine acceptance steps.

The CAN controller model uses software counters and complete-frame events. The Bus-Off recovery counter advances once for each distinct 100 ms simulation tick while recovery is enabled and continuous TX injectors are disabled. It labels each tick as one modeled eligible 11-recessive-bit observation. This is intentionally not bit-level timing. The simulation does not model bit stuffing, actual ACK slots, error flags, oscillator/bit timing, transceiver electrical behavior, bus wiring or a physical controller.

Other limitations: selected UDS/ISO-TP behavior only; no OEM/ISO certification; no physical vehicle or ECU validation; dashboard UDS transactions use an in-process helper and do not pace actual CAN frames; the Linux SocketCAN adapter is optional and separate from the dashboard. ECU Reset is an acknowledgement in the model, not a real process reset.

## 11. License

See `LICENSE`.
