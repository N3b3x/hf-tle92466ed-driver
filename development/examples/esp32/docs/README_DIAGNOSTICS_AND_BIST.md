# Diagnostics & BIST Example — TLE92466ED

## Table of Contents

1. [Overview](#overview)
2. [Purpose](#purpose)
3. [Hardware Requirements](#hardware-requirements)
4. [APIs Demonstrated](#apis-demonstrated)
5. [Building and Running](#building-and-running)
6. [Expected Output](#expected-output)
7. [Test Phases](#test-phases)
8. [Troubleshooting](#troubleshooting)
9. [Related Documentation](#related-documentation)

---

## Overview

The **Diagnostics & BIST** example exercises all diagnostic, self-test, and fault-mask APIs added
in Phase 5 of the driver refactor. No actual load is required — the example runs entirely from SPI
communication and verifies that the on-chip built-in self-test infrastructure is reachable and
returns sensible results.

Because no hardware load is needed, this example is registered with `ci_enabled: true` in
`app_config.yml` and runs automatically in the GitHub Actions CI matrix.

---

## Purpose

1. **Verify SFF_BIST** reachability and correct decode of the DONE/PASS/UERR/CERR bits.
2. **Read PIN_STAT** and print the actual DRV0/DRV1/EN/FAULTN/FAULTN_FB pin states.
3. **Inspect OperationState** and map it to a human-readable string.
4. **Exercise RunSupplyMonitorSelfTest()** through all four test phases.
5. **Demonstrate SetFaultMask()** by suppressing then re-enabling a specific fault source.

---

## Hardware Requirements

| Item | Specification |
|------|--------------|
| ESP32-S3 board | Any board matching `esp32_tle92466ed_test_config.hpp` pin map |
| TLE92466ED board | VIO = 3.3 V, VBAT any valid value (5.5–41 V) |
| Load | **Not required** — all tests work without a connected load |

---

## APIs Demonstrated

| API | Phase | Purpose in Example |
|-----|-------|-------------------|
| `Init()` | — | Initialize driver |
| `EnterMissionModeChecked()` | 2 | Safe mission-mode entry |
| `GetOperationState()` | 2 | Print current driver state |
| `ReadPinStatus()` | 5 | Decode DRV0/DRV1/EN/FAULTN/FAULTN_FB |
| `RunSffBist()` | 5 | Trigger BIST, decode DONE/PASS/UERR/CERR |
| `RunSupplyMonitorSelfTest()` | 2 | Four-phase supply-monitor self-test |
| `SetFaultMask()` | 5 | Suppress then restore Ch0Error on FAULTN |
| `HasAnyFault()` | — | Verify fault state after each phase |
| `ClearFaults()` | — | Clear transient faults between phases |

---

## Building and Running

```bash
# From the repo root
cd examples/esp32

# Build
python3 scripts/build.py --app diagnostics_and_bist --target esp32s3 --idf-version release/v5.5

# Flash + monitor
python3 scripts/flash.py --app diagnostics_and_bist --port /dev/ttyUSB0
idf.py -C build-app-diagnostics_and_bist-type-Debug-target-esp32s3-idf-release_v5.5 monitor
```

---

## Expected Output

```
[DIAG] === Diagnostics & BIST Example ===

[DIAG] --- OperationState ---
[DIAG] State: Mission

[DIAG] --- PIN_STAT snapshot ---
[DIAG] DRV0=0  DRV1=0  EN=1  FAULTN=1  FAULTN_FB=1  raw=0x0030

[DIAG] --- SFF_BIST ---
[DIAG] BIST done=1  pass=1  UERR=0  CERR=0  [PASS]

[DIAG] --- Supply Monitor Self-Test ---
[DIAG] UV/OV swap:  PASS
[DIAG] V1V5 UV:     PASS
[DIAG] V1V5 OV:     PASS
[DIAG] OT test:     PASS
[DIAG] Overall:     PASS

[DIAG] --- Fault Mask Demo ---
[DIAG] Ch0Error suppressed on FAULTN  (mask bit cleared)
[DIAG] Ch0Error re-enabled on FAULTN  (mask bit restored)

[DIAG] All diagnostics completed successfully.
```

---

## Test Phases

### Phase 1 — OperationState

Calls `GetOperationState()` and maps the result to a string:

| `OperationState` | String |
|-----------------|--------|
| `Reset` | `"Reset"` |
| `Config` | `"Config"` |
| `Mission` | `"Mission"` |
| `CriticalFault` | `"CriticalFault"` |

### Phase 2 — PIN_STAT Snapshot

Calls `ReadPinStatus()` and prints each decoded field. Useful for verifying that the EN and FAULTN
wiring is correct before enabling channels.

### Phase 3 — SFF_BIST

1. Calls `RunSffBist(20)` (20 ms timeout).
2. Checks `result.done == true` — if not, the BIST did not complete within the window.
3. Checks `result.pass == true` — if not, the IC reported an internal register failure.
4. Logs UERR and CERR (uncorrectable / correctable register errors).

> **Note**: A failing SFF_BIST does not indicate a driver bug — it indicates a hardware fault in the
> IC's register array. Replace the TLE92466ED if BIST fails repeatedly.

### Phase 4 — Supply Monitor Self-Test

Calls `RunSupplyMonitorSelfTest()` which internally executes four sub-tests by writing specific
GLOBAL_CONFIG test-mode bits, waiting 1 ms, checking `HasAnyFault()`, then clearing and restoring
the configuration. The returned `SupplyMonitorSelfTestResult` struct captures the pass/fail of each
phase.

### Phase 5 — Fault Mask Demo

1. Calls `SetFaultMask(MaskableFault::Ch0Error, false)` to suppress CH0 error contribution to
   FAULTN.
2. Logs the mask change.
3. Calls `SetFaultMask(MaskableFault::Ch0Error, true)` to restore default behavior.

This pattern is useful in applications where a specific fault source is expected during a transient
operating condition and should not trip the hardware FAULTN output.

---

## Troubleshooting

### `RunSffBist()` returns `TimeoutError`

- The BIST DONE bit did not assert within `timeout_ms`. The default 20 ms should be sufficient.
- Check that VBAT and VIO are within spec. BIST requires valid power supplies.
- Increase `timeout_ms` to 50 ms and retry.

### `RunSffBist()` returns `pass=false`

- The IC has an internal register memory fault. This is a hardware issue, not a driver issue.
- Power-cycle and retry. If it persists, the device should be replaced.

### `RunSupplyMonitorSelfTest()` returns a phase as failing

- Each sub-test injects a test condition and expects a fault. If no fault appears, the supply
  monitor circuit for that domain may be non-functional.
- Verify VBAT and VIO are at nominal levels before running the self-test.

### `SetFaultMask()` has no effect on FAULTN pin

- Some fault sources are always routed to FAULTN regardless of the mask (e.g., VDD undervoltage).
  See the FAULT_MASK0/1/2 register descriptions in the TLE92466ED datasheet for non-maskable faults.

---

## Related Documentation

- [API Reference](../../docs/api_reference.md)
- [Hardware Setup](../../docs/hardware_setup.md)
- [Driver Integration Test](README_DRIVER_INTEGRATION_TEST.md)
- [Resistive-Load Bench](README_RESISTIVE_LOAD_BENCH.md)
