# AutoCore v1.6.1 — Diagnostics implementation notes

## UDS services currently represented

| SID | Service | Current behavior |
|---|---|---|
| `0x10` | Diagnostic Session Control | Accepts default (`0x01`) and extended (`0x03`) sessions; returns positive `0x50` response |
| `0x11` | ECU Reset | Validates supported reset types and returns `0x51`; acknowledgement is simulated only |
| `0x22` | Read Data By Identifier | Supports `F40C` RPM, `F405` engine coolant temperature, and `F411` battery voltage |
| `0x14` | Clear Diagnostic Information | Clears the modeled all-DTC group `FFFFFF`, status bytes and snapshots |
| `0x19` | Read DTC Information | `0x01` counts DTCs matching a status mask; `0x02` returns DTC records/status; `0x04` retrieves snapshot record 1 for a specified DTC |
| `0x3E` | Tester Present | Updates tester-present timestamp, acknowledges normal requests and honors suppress-positive-response bit |

The simplified DTC status byte models bit 0 (`testFailed`), bit 1 (`testFailedThisOperationCycle`), bit 2 (`pendingDTC`), bit 3 (`confirmedDTC`), and bit 5 (`testFailedSinceLastClear`). The advertised status availability mask is `0x2F`. A monitor must report three consecutive failed observations before confirmation: the first sample sets pending, the second remains pending, and the third confirms the DTC and clears pending. A healthy sample resets the consecutive-failure count and clears current/pending bits. Confirmed and failed-since-clear history remain until UDS `0x14` clears the modeled store. This is an educational debounce policy; aging, permanent DTCs and multi-operation-cycle aging are not implemented. The DTC format identifier is `0x01` (three-byte DTC representation).

On a DTC's inactive-to-active transition, the Diagnostics ECU stores snapshot record `0x01` with engine RPM (`F40C`), engine temperature (`F405`), vehicle speed (`F40D`), battery voltage (`F411`), and simulation timestamp (`F4A0`). UDS `0x19 0x04` returns that record as a multi-frame ISO-TP response. The first snapshot is retained until the DTC is cleared; repeated active observations do not overwrite it. This is an educational snapshot schema, not an OEM-defined record layout.

The ISO-TP transport utility supports Classic CAN Single Frames, 12-bit First Frame lengths, Consecutive Frames, sequence validation, receiver timeouts, and a stateful transmitter. The transmitter processes Continue-To-Send, WAIT, and OVERFLOW flow-control statuses; enforces peer block size; and supports STmin values in milliseconds (`0x00`–`0x7F`) and 100-microsecond increments (`0xF1`–`0xF9`). Its caller supplies a monotonic microsecond clock. The in-process UDS transaction helper reassembles segmented requests and segments long responses without simulating real-time bus pacing.

## DTC catalogue

| Code | Detection condition | Recovery behavior |
|---|---|---|
| `P0217` | Engine coolant temperature > 115 °C | Clears active status after a later valid engine frame returns within threshold |
| `P0562` | Battery voltage < 11,000 mV | Clears active status after a later valid battery frame returns within threshold |
| `U0100` | Engine telemetry absent for > 500 ms after first valid frame | A later engine frame clears active status |
| `U0111` | Battery telemetry absent for > 500 ms after first valid frame | A later battery frame clears active status |
| `AC0001` | CRC-8/SAE-J1850 validation fails on monitored ECU telemetry | Normal telemetry clears the monitor; dashboard-injected demo fault is latched until **Recover all** |
| `AC0002` | Monitored ECU rolling counter is not the expected modulo-256 successor | Correct sequencing clears the monitor; dashboard-injected demo fault is latched until **Recover all** |

`AC0001` and `AC0002` are AutoCore-specific demonstration codes, not manufacturer-assigned DTCs. Following a CRC-rejected frame, the next valid frame becomes a fresh counter baseline. This prevents a single corrupted/dropped frame from creating a second, misleading rolling-counter DTC. The current implementation stores DTC identity, simplified status and one first-fault snapshot per DTC. It does not model all ISO 14229 DTC lifecycle semantics, aging counters, multiple snapshot records or permanent DTC behavior.

## DTC qualification and transitions

- `pending` is set after the first failed monitor observation.
- The second consecutive failed observation remains pending.
- The third consecutive failed observation promotes the DTC to confirmed and clears the pending bit.
- A passing observation clears current-test-failed and pending bits, and resets the consecutive sample counter. Confirmed/history bits remain until UDS `0x14` clears the modeled diagnostic store.
- Sensor-monitor observations run on 100 ms ECU telemetry, so the configured three-sample qualification is about 300 ms of sustained violation. Communication DTCs first require the documented >500 ms timeout, then use the same three-sample monitor qualification.
- The first fault/pending transition captures the freeze frame once; subsequent observations do not overwrite it.

## Dashboard self-test

The v1.6.1 dashboard offers a guided **Run self-test** sequence with seven test groups:

1. P0217 engine overtemperature and UDS DTC/freeze-frame read.
2. P0562 battery undervoltage and UDS DTC/freeze-frame read.
3. U0100 Engine ECU communication timeout.
4. U0111 Battery ECU communication timeout.
5. AC0001 CRC integrity fault, persistence across dashboard polling and recovery.
6. AC0002 rolling-counter sequence fault, persistence and recovery.
7. UDS DIDs (`F40C`, `F405`, `F411`), extended diagnostic session, Tester Present and simulated ECU Reset acknowledgement.

The guided flow recovers each injection and ends with a DTC read. Recovered historical DTC entries remain visible until the operator sends clear-DTC service `0x14`. A healthy startup with zero active faults is the expected baseline; the Diagnostics page's six-monitor catalogue separately shows which monitor definitions are implemented and their current state.


### Freeze-frame response layout correction

Subfunction `0x19 0x04` includes the required NumberOfIdentifiers byte (`0x05`) before the five DIDs in record `0x01`: F40C, F405, F40D, F411 and F4A0. The five-DID layout is verified by a unit test and the live dashboard decodes those values.


## CAN controller fault confinement relationship

AutoCore v1.6.1 also models per-ECU CAN controller TEC/REC and Error Active, Error Passive and Bus-Off states. Bus-Off prevents the selected ECU from transmitting. For the Engine and Battery ECUs, missing telemetry can trigger U0100 or U0111 after the >500 ms presence timeout and the same three-distinct-observation DTC qualification. When the node completes the modeled Bus-Off recovery and valid telemetry resumes, the communication DTC recovers. Controller counters and recovery progress are shown on the separate CAN Controller Health dashboard page. See `can_fault_confinement.md` for the counter rules and limitations.
