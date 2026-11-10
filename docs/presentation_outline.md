# AutoCore v1.6.1 — presentation outline

## Slide 1 — Title
AutoCore: Automotive Multi-ECU Communication, Diagnostics and CAN Controller Fault Confinement.

## Slide 2 — Problem and motivation
Vehicles distribute sensing and functions across ECUs. Embedded engineers need to understand ECU telemetry, CAN communication, diagnostics, fault recognition, network fault containment and recovery. AutoCore provides an interactive laptop-based lab without physical vehicle hardware.

## Slide 3 — Objectives
Implement a C++17 model of four ECUs, Classic CAN frame handling, ISO-TP helpers, selected UDS services, six DTC monitors, CAN controller TEC/REC state transitions, fault injection/recovery, automated verification and a live browser dashboard.

## Slide 4 — Software architecture
Show Browser → loopback Python bridge → persistent C++ engine → VehicleNetwork / virtual CAN → four ECUs and per-node controllers. Explain that the C++ engine is the single source of truth.

## Slide 5 — ECU and CAN message network
Present Engine `0x100`, Battery `0x110`, Body `0x120`, UDS request `0x7E0` and response `0x7E8`. Explain 100 ms periodic telemetry, runtime CSV message matrix (5 messages / 16 signals), monitor-only ID/mask filtering, eight-byte telemetry layout, rolling counter and CRC-8/SAE-J1850.

## Slide 6 — Transport and diagnostics
Explain ISO-TP Single/First/Consecutive Frames and flow-control helpers. List selected UDS services `0x10`, `0x11`, `0x14`, `0x19`, `0x22`, `0x3E`, six DTC monitors and first-fault freeze frame.

## Slide 7 — CAN fault confinement
Explain TEC and REC. Error Active: TEC/REC below 128. Error Passive: TEC or REC at least 128. Bus-Off: TEC reaches 256. One TX error adds 8 TEC; one RX error adds 1 REC. A Bus-Off ECU stops transmitting while healthy peer ECUs continue.

## Slide 8 — Bus-Off recovery demo
Reset baseline, inject 32 TX errors into Engine (TEC reaches 256), verify Engine frames stop while Battery/Body continue, inspect U0100, disable all persistent TX injectors, start recovery and observe 128/128 eligible sequence observations, then verify Engine returns to Error Active. Explicitly call the 100 ms discrete sequence model an educational approximation, not bit-accurate timing.

## Slide 9 — Live dashboard
Demonstrate Overview, CAN Network, CAN Controller Health, Diagnostics, Scenario Lab and Event Log. Show live signals, frame IDs/payloads, TEC/REC, state transitions, DTCs, UDS requests and CSV export.

## Slide 10 — Guided tests and verification
Show the diagnostic self-test (7/7 PASS), then report 122 C++ regression assertions, 2/2 CTest targets and 17-group real HTTP-to-C++ integration workflow. Include GCC/Clang and sanitizer results only when backed by the final verification report.

## Slide 11 — Limitations and conclusion
State clearly: all ECUs and CAN controllers are simulated; the network uses frame-level arbitration; Bus-Off recovery uses a discrete 128-observation model; selected UDS/ISO-TP services only; no physical CAN, vehicle, OEM or ISO certification. Conclude that the project demonstrates embedded communication, controller fault confinement, diagnostic response and recovery in a repeatable laptop environment.
