# Changelog

## v1.6.1 — 8 November 2026

- Fixed Windows first-run dashboard integration test timing: the initial localhost readiness wait now allows up to 20 seconds for C++ process startup and Windows endpoint-protection scanning, instead of failing after 3 seconds.
- Captured dashboard-server startup output in the integration test and report early process exits with their traceback/log rather than a generic timeout.
- Preserved the v1.6.0 C++ engine and CAN controller behavior; this is a Windows startup/test-harness reliability patch.

## v1.6.0 — 8 November 2026

- Added independent TEC/REC controller models for Engine, Battery, Body and Diagnostics ECUs.
- Added Error Active, Error Passive and Bus-Off state transitions and controller statistics.
- Added per-node TX/RX error bursts and persistent error injection.
- Bus-Off controllers stop transmitting while healthy peers continue network traffic.
- Added a 128-observation Bus-Off recovery model gated on all persistent TX injectors being disabled. Recovery progresses only on distinct virtual scheduler timestamps.
- Added the CAN Controller Health dashboard page with TEC/REC progress, state, counters, injection controls and recovery progress.
- Added controller-state event logging and end-to-end tests for passive, bus-off, network isolation, communication DTC and recovery.
- Updated Windows launcher, version-specific build directory, run instructions, report, architecture and presentation.

# AutoCore changelog

## v1.5.0 — 8 November 2026

- Made `config/can_messages.csv` a runtime input: the C++ engine loads and validates all five Classic CAN message definitions and sixteen signal definitions before starting; missing or invalid matrices fail with a diagnostic error.
- Added symbolic message names and the loaded message/signal catalogue to the live CAN monitor.
- Added a standard 11-bit ID/mask acceptance filter for the CAN monitor, with accepted/rejected counters and invalid-ID/mask rejection. The filter is intentionally scoped to the UI monitor; it does not suppress traffic seen by the Diagnostics ECU.
- Added a three-consecutive-sample DTC qualification policy: pending on the first failed observation and confirmed on the third. Recovery clears current and pending bits while retaining simplified historical bits until UDS `0x14` clears the modeled DTC store.
- Improved monitor catalogue and Scenario Lab cards to distinguish pending, confirmed, recovered and not-set states.
- Added separate pending, confirmed, recovered and cleared DTC event-log transitions.
- Added regression tests for duplicate CAN signals, invalid DLC layouts, non-finite scale values, missing matrix startup, live filter behavior and DTC debounce.
- Updated the project report, protocol notes, architecture, dashboard guide and presentation outline for the v1.5.0 behavior.

## v1.4.2 — 8 November 2026

- Added exact integration checks for the UDS 0x19 0x04 response header, DTC identifier, five-identifier count and all five freeze-frame DIDs.
- Rejected extended-ID and oversized-DLC requests at the low-level UDS Classic CAN endpoint before parsing payload bytes.
- Hardened the optional SocketCAN receiver to return safely when `poll()` reports readiness without `POLLIN`.
- Kept the self-test CRC monitor check tied to the persistent injected-fault latch and active AC0001 DTC.
- Expanded the audited regression suite to 78 C++ assertions and retained 14 real HTTP-to-C++ integration groups.
- Reverified clean GCC and Clang Release builds with warnings treated as errors, plus GCC AddressSanitizer/UndefinedBehaviorSanitizer builds.
- Verified the actual dashboard DOM and full 7/7 guided self-test control flow in Chromium using a controlled API fixture: all five views, six recovered DTC rows, zero active faults after recovery and no JavaScript page exceptions.

## 1.4.1 — verification and diagnostic payload correction

- Corrected UDS ReadDTCInformation subfunction 0x04 to include the NumberOfIdentifiers byte before freeze-frame DIDs.
- Enhanced the dashboard to decode freeze-frame RPM, coolant temperature, vehicle speed, battery voltage and simulation timestamp.
- Made the guided CRC self-test verify the persistent C++ fault latch plus the detected AC0001 DTC, avoiding timing dependence on a single 100 ms frame sample.
- Added malformed ISO-TP DLC and null CRC-input tests.
- Kept the C++ engine as the sole simulation state source and preserved the localhost-only dashboard bridge.

## v1.4.1 — 8 November 2026

- Added a globally accessible **Run self-test** control and an Overview-page self-test entry point.
- Added a six-monitor diagnostics catalogue that clearly distinguishes a healthy `NOT SET` monitor from an active or recovered DTC record.
- Added a guided 7-group diagnostic self-test covering P0217, P0562, U0100, U0111, AC0001, AC0002, two freeze frames, three DIDs, Diagnostic Session Control, Tester Present, simulated ECU Reset acknowledgement and recovery.
- Improved live fault hints and monitor cards to show normal, injected/monitoring, active and recovered/stored states.
- Fixed a CRC-rejected telemetry frame causing a cascading false rolling-counter sequence DTC by re-baselining the next trusted frame.
- Improved C++ command parsing with strict `from_chars` validation for throttle and freeze-frame codes.
- Hardened CAN payload access against invalid DLC values and integer-overflow indices; added regression assertions.
- Hardened the Python localhost bridge with bounded stderr capture, explicit child-process lifecycle cleanup, request content-type validation, port-range validation, improved health diagnostics and startup cleanup if the HTTP port cannot be bound.
- Expanded integration tests for body controls, reset behavior, throttle boundaries, invalid API requests and the complete guided fault/UDS/recovery workflow.
- Updated the Windows launcher to use a version-specific `build-windows-v1.4.1` directory to avoid stale/incompatible build caches.

## v1.3.2 — 8 November 2026

- Fixed CRC and rolling-counter demo faults being cleared by the next valid telemetry frame before the dashboard refresh could display them. These operator-injected faults are latched in Diagnostics ECU state until the operator selects Recover all.
- Added a one-click paired sensor-fault demo that raises coolant to 126 C and battery voltage to 10.45 V, producing P0217 and P0562 together.
- Added live Scenario Lab and Diagnostics hints showing active DTCs and explaining the normal no-fault startup state.
- Added success feedback after fault injection and recovery commands.
- Extended dashboard integration tests to verify persistence across multiple polling intervals, recovery, paired sensor DTCs and positive UDS DTC response.

## v1.3.1 — 8 November 2026

- Added one-click Windows dependency bootstrap to `Run_AutoCore.bat`.
- Detects CMake, MSVC C++ Build Tools and Python 3.10+; installs missing tools using Windows Package Manager (`winget`).
- Uses the Visual Studio C++ workload and recommended components, including MSVC x64/x86 tools and CMake integration.
- Uses a separate Windows build directory to avoid reusing Linux-generated CMake caches.
- Native Windows installation and execution still require first-run verification on a Windows host.

## v1.3.0 — 8 November 2026

- Replaced the static HTML report as the primary user interface with a continuously updating browser dashboard.
- Added persistent C++ JSON-lines simulation engine; C++ remains the source of truth for ECU values, telemetry, DTC state, CAN frames and UDS responses.
- Added local Python standard-library HTTP bridge bound to loopback (`127.0.0.1`), with strict action allow-list and range validation.
- Added deterministic time-varying drive-cycle signals and throttle control.
- Added dashboard pages for overview, CAN network, diagnostics, scenario lab and event log.
- Added telemetry charts, ECU connectivity cards, CRC status, DTC register and freeze-frame summaries.
- Added dashboard controls for start/pause/reset, throttle, body signals, sensor faults, ECU offline scenarios, CRC corruption and rolling-counter sequence faults.
- Added UDS controls for DID read, DTC read, extended diagnostic session, simulated ECU reset acknowledgement, Tester Present and clear-DTC information.
- Added rolling telemetry CSV export and bounded recent event/frame/history buffers.
- Added an end-to-end Python integration test covering the browser/API-to-C++ simulation path.

## v1.2.0 — 8 November 2026

- Added UDS `0x14` clear-all diagnostic information service for the modeled DTC group.
- Added simplified DTC status lifecycle bits and first-fault freeze-frame capture.
- Added UDS `0x19 0x04` freeze-frame retrieval with multi-frame ISO-TP response.
- Added a self-contained static HTML report generator.

## v1.1.0 — 8 November 2026

- Added Linux SocketCAN adapter and optional virtual-CAN loopback example.
- Improved frame arbitration ordering.
- Integrated ISO-TP reassembly/segmentation into the UDS request/response helper.
- Added stateful ISO-TP transmitter with flow-control, block-size and STmin handling.
- Added UDS `0x19` status-mask reporting and Tester Present behavior.
- Added 100 ms periodic ECU telemetry, rolling counters and CRC-8/SAE-J1850 integrity checks.

## v1.0.0 — 8 November 2026

- Initial C++17 multi-ECU CAN simulator with Engine, Battery, Body and Diagnostics models.
- Added basic Classic CAN frames, telemetry, DTC detection, build system and regression tests.
