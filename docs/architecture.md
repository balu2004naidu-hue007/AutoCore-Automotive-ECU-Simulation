# AutoCore v1.6.1 system architecture

## Runtime architecture

AutoCore uses a persistent C++17 simulation engine, a Python standard-library localhost bridge and a browser dashboard. The C++ engine is the source of truth for simulated signals, ECUs, CAN controller states, diagnostics and events.

```text
+----------------------------------------------------------------+
| Browser dashboard: dashboard/index.html                       |
| Overview | CAN Monitor | Controller Health | Diagnostics       |
| Scenario Lab | Event Log | Live Charts | TEC/REC | DTC/UDS       |
+-------------------------------+--------------------------------+
                                | HTTP JSON on 127.0.0.1
                                v
+----------------------------------------------------------------+
| dashboard/server.py                                            |
| HTML host | allow-listed API | validation | CSV | health        |
| 100 ms virtual tick | C++ child-process / JSON-lines bridge    |
+-------------------------------+--------------------------------+
                                | stdin/stdout JSON lines
                                v
+----------------------------------------------------------------+
| src/engine.cpp — persistent C++ simulation controller          |
| drive cycle | ECU controls | fault commands | events | JSON     |
+-------------------------------+--------------------------------+
                                v
+----------------------------------------------------------------+
| VehicleNetwork + in-process frame-level Classic CAN bus        |
| Per-node CAN controllers: TEC / REC / state / recovery         |
+----------------+----------------+----------------+--------------+
| Engine ECU     | Battery ECU    | Body ECU       | Diagnostics  |
| CAN 0x100      | CAN 0x110      | CAN 0x120      | UDS 0x7E0/8  |
+----------------+----------------+----------------+--------------+
```

## Responsibilities

| Component | Responsibility |
|---|---|
| Engine ECU | RPM, coolant temperature and vehicle speed telemetry |
| Battery ECU | Voltage, current and battery temperature telemetry |
| Body ECU | Headlight and driver-door signals |
| Diagnostics ECU | Integrity validation, sensor thresholds, communication timeouts, DTC state, freeze frames and UDS |
| `CanBus` | Frame validation, queue and complete-frame arbitration |
| `CanController` | Per-node TEC/REC, Error Active/Passive/Bus-Off state, TX/RX statistics and Bus-Off recovery progress |
| `VehicleNetwork` | 100 ms ECU publication, per-controller transmit isolation, receive accounting, error injection, recovery gating and diagnostics observation |
| `CanMessageDatabase` | Runtime CSV definition loading, validation and symbolic message lookup |
| `Simulation` in `engine.cpp` | Drive cycle, dashboard commands, injected-fault state, bounded histories, JSON state and state-transition event log |
| `server.py` | Loopback HTTP server, allow-listed action validation, engine lifecycle, health checks and CSV export |
| `index.html` | Six dashboard pages, telemetry graphs, CAN frame monitor, TEC/REC controller view, DTC/UDS console, scenario controls and event log |

## CAN controller state model

Each logical ECU owns an independent `CanController` instance.

- Error Active: TEC < 128 and REC < 128.
- Error Passive: TEC >= 128 or REC >= 128 while TEC < 256.
- Bus-Off: TEC >= 256. The ECU is removed from simulated transmission.

Counter updates in the educational model:

- Transmit error adds 8 to TEC.
- Successful transmit decrements nonzero TEC by 1.
- Receive error adds 1 to REC.
- Successful receive decrements REC by 1, with REC above 127 reduced to 127 on the next successful receive.
- TEC and REC are bounded; lifetime event counters saturate.

A Bus-Off controller does not publish its ECU telemetry. Other non-Bus-Off ECUs still publish. The Diagnostics ECU can therefore detect the missing Engine/Battery ECU messages and qualify U0100/U0111 after the configured timeout and three distinct monitor observations.

## Bus-Off recovery scheduling

The operator can start recovery only for a Bus-Off controller and only after every continuous TX error injector in the network has been disabled. `VehicleNetwork::observeRecoveryOpportunities` advances recovery once per distinct simulation timestamp. Each distinct 100 ms virtual scheduler tick models one eligible observation of an 11-recessive-bit sequence. Repeated calls at the same or earlier timestamp do not count. At 128 observations, TEC/REC reset and the controller returns to Error Active.

This is not a physical CAN recovery implementation. It makes the high-level fault-confinement and recovery workflow observable in a deterministic laptop simulation without claiming bit timing or electrical accuracy.

## Runtime CAN matrix and monitor filtering

At C++ startup, `config/can_messages.csv` is loaded and validated. It currently defines five messages (`ENGINE_STATUS`, `BATTERY_STATUS`, `BODY_STATUS`, `UDS_REQUEST`, `UDS_RESPONSE`) and sixteen signal entries. The matrix validates identifiers, periods, DLC and signal layout, publishes metadata to the dashboard and maps captured frame IDs to symbolic message names. ECU encoders remain explicit C++ logic; this is not a DBC code generator.

The CAN Network page's 11-bit ID/mask filter affects the monitor display and accepted/rejected view counters only. It does not suppress the frames the Diagnostics ECU receives.

## Dashboard data path

The Python server advances the C++ simulation every 100 ms while running. The browser polls JSON state at 500 ms. UI actions are validated by the HTTP server and sent as JSON-line commands to the persistent C++ process. Controller TEC/REC values and states in the browser come from the C++ model, not a separate JavaScript simulation.

## Protocol and timing boundaries

The CAN bus is complete-frame-level, not bit-level. Error injection is applied to controller counters rather than emitting physical error flags. Bus-Off recovery counts a discrete scheduler-tick representation of the required bus-idle sequence. ISO-TP helper framing is tested, but browser UDS actions use the in-process helper and do not simulate real-time frame pacing. Selected UDS/ISO-TP behavior is educational only.
