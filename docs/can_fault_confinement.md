# CAN Controller Fault Confinement — AutoCore v1.6.1

## Purpose

This module demonstrates the controller-level fault-confinement concept used by CAN nodes. Each logical ECU owns an independent controller state with a Transmit Error Counter (TEC), Receive Error Counter (REC), transmit/receive statistics and one of three states: Error Active, Error Passive or Bus-Off.

This is a software model for learning and repeatable laptop demonstrations. It is not a bit-accurate CAN controller, an implementation of a specific MCU peripheral, or a physical bus simulator.

## Controller state model

| State | Entry condition in this model | Behavior |
|---|---|---|
| Error Active | TEC < 128 and REC < 128 | ECU may transmit and receive normally |
| Error Passive | TEC >= 128 or REC >= 128, while TEC < 256 | ECU remains able to transmit and receive |
| Bus-Off | TEC >= 256 | ECU is prevented from transmitting; it remains isolated until recovery |

TEC is capped at 256; REC is capped at 255. The simplified counter updates are:

- Transmit error: TEC += 8.
- Successful transmission: TEC -= 1 if TEC is nonzero.
- Receive error: REC += 1.
- Successful reception: REC -= 1 while REC is 1..127; if REC is above 127, the first successful reception reduces it to 127.

The module also saturates lifetime event counters instead of allowing unsigned integer wraparound.

## Fault injection

The CAN Controller Health page provides these controls for the selected ECU:

- **24 TX errors:** adds 24 transmit error events. Starting from TEC 0, TEC becomes 192 and the controller enters Error Passive.
- **32 TX errors:** adds 32 transmit error events. Starting from TEC 0, TEC reaches 256 and the controller enters Bus-Off.
- **128 RX errors:** sets REC to 128 from a clean baseline, entering Error Passive.
- **Continuous TX errors:** each scheduled telemetry attempt fails and adds 8 TEC. The ECU eventually enters Bus-Off and stops publishing telemetry.
- **Continuous RX errors:** each incoming frame observed by that receiver adds 1 REC. Disable this injector to let valid receptions reduce REC.

The burst controls represent injected controller error events. They do not generate physical CAN error flags or reproduce bit-level retransmission/ACK behavior.

## Network isolation and diagnostics

When a node becomes Bus-Off, its periodic telemetry producer is suppressed. Other non-Bus-Off ECUs continue transmitting. For Engine or Battery Bus-Off, the Diagnostics ECU detects the missing telemetry after the configured >500 ms timeout (once ECU presence has previously been established) and qualifies U0100 or U0111 using the project's three-distinct-sample policy.

When the node recovers and valid telemetry returns, the current communication DTC recovers. Its simplified historical record remains until the operator invokes UDS service 0x14.

## Recovery model

1. The selected controller must be Bus-Off.
2. All persistent TX error-injection toggles across the network must be disabled.
3. The operator starts recovery from the dashboard.
4. One distinct 100 ms simulation tick represents one eligible observation of an 11-recessive-bit idle sequence.
5. Repeated state updates at the same simulation timestamp do not advance the counter.
6. After 128 observations, TEC and REC reset to zero, the controller becomes Error Active and the recovery counter increments.

At nominal simulation speed this is approximately 12.8 seconds of virtual running time. Pausing the simulation pauses recovery progress. This is an explanatory discrete model: real controller recovery depends on observing the bus-level sequence and physical timing, not a software timer of 12.8 seconds.

## Dashboard workflow

1. Open **CAN Controller Health**.
2. Select Engine ECU.
3. Click **24 TX errors · Error Passive**; verify TEC 192 and Error Passive.
4. Reset the simulation before a separate scenario.
5. Select Engine ECU and click **32 TX errors · Bus-Off**; verify TEC 256 and Bus-Off.
6. Open CAN Network; verify Engine frames stop while Battery and Body messages continue.
7. Wait for U0100 to qualify in Diagnostics.
8. Return to Controller Health; ensure every continuous TX injector is off.
9. Start Bus-Off recovery and watch 0/128 to 128/128.
10. Verify Engine returns to Error Active and telemetry resumes. Check Event Log for transition entries.
11. Reset; select Battery and click 128 RX errors to demonstrate REC-driven Error Passive.

## Implementation map

- `include/autocore/can_controller.hpp`: controller enums, snapshots, thresholds and API.
- `src/can_controller.cpp`: counter updates, state transition and recovery logic.
- `include/autocore/vehicle_network.hpp` / `src/vehicle_network.cpp`: one controller per ECU, error injection, transmit isolation, receive accounting and distinct-timestamp recovery scheduling.
- `src/engine.cpp`: dashboard command parser, state JSON and transition events.
- `dashboard/server.py`: validated localhost actions.
- `dashboard/index.html`: controller health page, controls and live table.
- `tests/test_autocore.cpp`: thresholds, counter recovery, network isolation and timing edge cases.
- `tests/test_dashboard_integration.py`: actual HTTP-to-C++ workflow through Bus-Off and recovery.

## Deliberate limitations

The software does not implement CAN bit timing, sample points, bit stuffing, ACK slots, error-active/passive flag bit patterns, overload frames, physical bus-off pin behavior, transceiver standby, oscillator tolerance, a real controller register map, or all exception rules that vary by controller implementation. The 128 recovery observations are explicitly a discrete educational representation of the protocol concept. Do not describe this as bit-accurate, hardware-tested or certified.
