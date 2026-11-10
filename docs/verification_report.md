# AutoCore v1.6.1 verification report

## Release build and automated test results

This v1.6.1 patch changes the Windows integration-test startup wait from 3 seconds to 20 seconds and preserves server logs for early-exit diagnostics. This addresses the failure signature reported on the target Windows laptop, where `autocore_dashboard_integration` timed out while waiting for the initial `/api/state` endpoint. The updated source was re-run in the available Linux environment; native Windows confirmation remains necessary.

The v1.6.1 source was built from clean CMake configurations in the available Linux environment. GCC 14.2 Release and Clang 17 Release builds both used C++17 with compiler warnings treated as errors. The GCC AddressSanitizer/UndefinedBehaviorSanitizer Debug build also completed.

The C++ regression executable printed **122 `[PASS]` assertions** and no failures. CTest passed both registered targets:

- `autocore_tests` — C++ regression suite.
- `autocore_dashboard_integration` — Python localhost HTTP bridge connected to the compiled C++ engine.

Both CTest targets passed under GCC Release, Clang Release and GCC AddressSanitizer/UndefinedBehaviorSanitizer. The integration workflow reported **17 groups passed**. It launches the actual engine and Python server on an ephemeral loopback port, sends HTTP actions and checks returned C++ state.

## Verification matrix

| Verification | Result |
|---|---|
| GCC 14.2 Release, warnings treated as errors | PASS |
| Clang 17 Release, warnings treated as errors | PASS |
| GCC AddressSanitizer + UndefinedBehaviorSanitizer | PASS |
| C++ regression executable | PASS — 122 assertions |
| CTest | PASS — 2/2 registered targets under GCC, Clang and sanitizer builds |
| HTTP-to-C++ integration | PASS — 17 groups |
| 10,000-cycle network stress test | PASS |
| Python server/test syntax (`py_compile`) | PASS |
| Dashboard inline JavaScript syntax (`node --check`) | PASS |
| Clean release ZIP extraction, compile and CTest | PASS |
| Full visual Chromium session with live localhost server | NOT VERIFIED — headless Chromium did not complete the loopback navigation within the available environment window |
| Native Windows MSVC build / winget bootstrap | NOT RUN — requires a Windows target-machine acceptance run |
| Physical CAN interface / real ECU | NOT PERFORMED — outside laptop-only scope |

## New controller fault-confinement coverage

The C++ tests exercise:

- TEC reaches Error Passive at 128 and Bus-Off at 256.
- TEC decreases after a successful transmit and can return Error Active.
- REC reaches Error Passive at 128 and falls after successful reception.
- A REC above 127 is reduced to 127 on the next successful receive.
- A Bus-Off node cannot transmit and accepts a recovery request only from Bus-Off.
- Recovery requires 128 eligible observations; an ineligible observation does not count.
- Network-level recovery counts once per distinct simulation timestamp, not repeatedly at one timestamp.
- Continuous TX error injection on any node prevents recovery until disabled.
- Bus-Off Engine stops its telemetry while Battery and Body continue to transmit.
- Engine can return online after exactly 128 distinct recovery observations.
- Bus-Off Diagnostics controller suppresses UDS responses.
- Persistent Engine TX errors accumulate one +8 TEC event per 100 ms scheduled attempt.

The integration test checks the same behavior through the actual HTTP server and compiled C++ engine: baseline state, TX-driven Error Passive, RX-driven Error Passive, Engine Bus-Off, Engine transmission isolation, continued Battery/Body messages, U0100 timeout/confirmation, recovery initiation, 128-step completion, Engine return to the network, continuous injector controls and malformed API input rejection.

## Existing feature coverage

The C++ regression suite and HTTP integration test also cover CAN frame identifier/DLC handling, frame arbitration, runtime message-matrix validation, ID/mask filter behavior, CRC and rolling-counter integrity, DTC pending/confirmed/recovered lifecycle, timestamp-qualified debounce, six DTC conditions, UDS positive/negative response paths, ISO-TP segmentation/reassembly/flow-control edge cases, freeze-frame layout, body controls, throttle boundaries, pause/reset, CSV/API behavior and a 10,000-cycle network stress test.

## Syntax and package checks

- `python3 -m py_compile dashboard/server.py tests/test_dashboard_integration.py` completed without syntax errors.
- The inline script extracted from `dashboard/index.html` passed Node.js `--check`. A structural HTML/JS wiring audit found 91 unique DOM IDs (no duplicates), and all 84 literal JavaScript element-ID references matched existing elements; all six sidebar navigation entries matched a page container.
- The release ZIP was extracted to a fresh directory, compiled and tested independently. The package excludes generated Linux build directories so Windows configures its own build directory.

## Reproduce a Release build (Linux)

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-Werror
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Sanitizer build:

```bash
cmake -S . -B build-sanitize -DCMAKE_BUILD_TYPE=Debug -DAUTOCORE_ENABLE_SANITIZERS=ON -DCMAKE_CXX_FLAGS=-Werror
cmake --build build-sanitize --parallel
ctest --test-dir build-sanitize --output-on-failure
```

Python syntax:

```bash
python3 -m py_compile dashboard/server.py tests/test_dashboard_integration.py
```

The dashboard has no third-party Python package dependencies; it uses the Python standard library and browser-native JavaScript.

## Windows acceptance status

The Windows launcher checks for CMake, Visual Studio C++ Build Tools and Python 3.10+, and attempts to install missing dependencies via `winget`. It configures `build-windows-v1.6.1`, builds C++ targets, runs CTest and starts the localhost dashboard. The available development environment is Linux, so dependency installation through winget, native MSVC compilation and the first `Run_AutoCore.bat` execution on the target laptop have **not** been performed here. The packaged bootstrap and launcher are included, but the first run on Windows remains an external acceptance step.

## Engineering scope and limitations

- No physical CAN controller, transceiver, vehicle, production ECU or external diagnostic tester was used.
- The CAN network arbitrates complete frames. Bit timing, bit stuffing, ACK slots, physical error flags, bus voltage, wiring, electrical behavior and real retransmission are not simulated.
- TEC/REC and Error Active/Error Passive/Bus-Off model principal fault-confinement concepts. This is not a vendor-specific hardware controller and does not reproduce every exception rule.
- Bus-Off recovery advances one step per distinct 100 ms virtual scheduler tick and labels that step as one modeled eligible observation of an 11-recessive-bit sequence. This is a visible discrete approximation, not physical bit-level timing.
- Error injection modifies controller counters; it does not generate actual bus error flags.
- ISO-TP helpers and selected UDS services are educational subsets; no ISO 15765, ISO 14229 or OEM conformance is claimed.
- Dashboard UDS transactions use an in-process helper and bypass real-time bus pacing.
- UDS `0x11` returns a simulated reset acknowledgement; it does not restart a real ECU or host process.
- The optional Linux SocketCAN adapter is separate from the browser simulation and has not been validated against a physical CAN bus.
- AutoCore is an educational simulator. It must not be used to control or diagnose a real vehicle.
