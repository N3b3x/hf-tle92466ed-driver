# Resistive-Load Bench Example — TLE92466ED

## Table of Contents

1. [Overview](#overview)
2. [Purpose](#purpose)
3. [Hardware Requirements](#hardware-requirements)
4. [Hardware Setup](#hardware-setup)
5. [Configuration](#configuration)
6. [Building and Running](#building-and-running)
7. [Expected Output](#expected-output)
8. [APIs Demonstrated](#apis-demonstrated)
9. [Troubleshooting](#troubleshooting)
10. [Related Documentation](#related-documentation)

---

## Overview

The **Resistive-Load Bench** example validates the TLE92466ED ICC current-regulation loop against a
known resistive load. A 300 Ω power resistor is connected between CH0 and GND with VBAT ≈ 12 V,
giving a theoretical maximum of 40 mA. The driver sweeps ICC setpoints from 5 mA to 40 mA in 5 mA
steps, holds each for 200 ms, reads an atomic feedback snapshot, and checks that the measured
average current is within ±2.5 mA of the setpoint.

### Key Features

- **Atomic Feedback**: Uses `ReadChannelFeedback()` with FB_FRZ/FB_UPD handshake for a coherent
  current/duty-cycle/VBAT snapshot.
- **Supply Readback**: Reads and logs VBAT, VIO, VDD, and die temperature before enabling outputs.
- **Phase 2 & 6 APIs**: Demonstrates `EnterMissionModeChecked()`, `ReadAllSupplyVoltages()`, and
  `ReadChannelFeedback()`.
- **±2.5 mA Tolerance Check**: Each step asserts measured current vs setpoint and prints PASS/FAIL.
- **Dither Enabled**: 2 mA amplitude at 200 Hz to exercise the dither path alongside ICC.

---

## Purpose

1. **Quantify ICC accuracy** at discrete operating points with a well-defined resistive load.
2. **Exercise the atomic-feedback API** introduced in Phase 6 (FB_FRZ/FB_UPD protocol).
3. **Validate supply-voltage readback** before enabling outputs (Phase 2 APIs).
4. **Provide a repeatable bench procedure** that can be run whenever register settings change.

---

## Hardware Requirements

| Item | Specification |
|------|--------------|
| ESP32-S3 board | Any board matching `esp32_tle92466ed_test_config.hpp` pin map |
| TLE92466ED eval / custom board | VIO = 3.3 V, VBAT = 12 V |
| Power resistor, CH0 | 300 Ω, ≥ 3 W rating (dissipates up to 0.5 W at 40 mA / 12 V) |
| VBAT supply | 12 V ± 5 %, current limited to 200 mA |
| USB/UART adapter | For ESP-IDF monitor (`idf.py monitor`) |

> **Safety note**: The 300 Ω resistor dissipates V²/R = 144/300 ≈ 0.5 W at full setpoint. Use a
> wirewound or metal-oxide power resistor rated ≥ 3 W and mount it with adequate thermal clearance.

---

## Hardware Setup

```text
ESP32-S3            TLE92466ED
────────────────────────────────
SPI3_MISO GPIO35 ── SO
SPI3_MOSI GPIO37 ── SI
SPI3_SCLK GPIO36 ── SCK
GPIO4            ── CSN
GPIO6            ── RESN
GPIO5            ── EN
GPIO16           ── FAULTN

TLE92466ED CH0 OUT ──┬── 300 Ω (3 W) ── GND
                     └── (no inductive load)
VBAT: 12 V DC
VIO:   3.3 V (from ESP32-S3 3V3 rail)
```

Pin constants are defined in `main/esp32_tle92466ed_test_config.hpp`. Update that file if your
board differs.

---

## Configuration

The example requires no Kconfig changes. The following constants are hard-coded at the top of
`main/resistive_load_bench.cpp`:

| Constant | Default | Meaning |
|----------|---------|---------|
| `BENCH_VBAT_MV` | 12 000 | Expected VBAT in mV (for sanity check) |
| `BENCH_RESISTANCE_OHM` | 300 | Load resistor value |
| `BENCH_PWM_PERIOD_US` | 1 000 | ICC PWM period (1 kHz) |
| `BENCH_DITHER_AMPLITUDE_MA` | 2 | Dither amplitude in mA |
| `BENCH_DITHER_FREQ_HZ` | 200 | Dither frequency in Hz |
| `BENCH_HOLD_MS` | 200 | Hold time at each setpoint step |
| `BENCH_FB_TIMEOUT_MS` | 50 | Feedback freeze/poll timeout |
| `BENCH_TOLERANCE_MA` | 2.5f | Acceptable error vs setpoint |
| Setpoint sweep | 5 … 40 mA | 5 mA steps (8 points) |

---

## Building and Running

```bash
# From the repo root
cd examples/esp32

# Build
python3 scripts/build.py --app resistive_load_bench --target esp32s3 --idf-version release/v5.5

# Flash + monitor
python3 scripts/flash.py --app resistive_load_bench --port /dev/ttyUSB0
idf.py -C build-app-resistive_load_bench-type-Debug-target-esp32s3-idf-release_v5.5 monitor
```

Or use the standard ESP-IDF flow after selecting `resistive_load_bench` in `app_config.yml`.

---

## Expected Output

```
[BENCH] === Resistive-Load Bench ===
[BENCH] Supply: VBAT=11987 mV  VIO=3301 mV  VDD=4998 mV  Tj=27.4 °C
[BENCH] Setpoint= 5 mA | AvgI=  4.8 mA | Duty=  3.2 % | VBAT=11985 mV | [PASS]
[BENCH] Setpoint=10 mA | AvgI=  9.9 mA | Duty=  6.5 % | VBAT=11982 mV | [PASS]
[BENCH] Setpoint=15 mA | AvgI= 15.1 mA | Duty=  9.8 % | VBAT=11980 mV | [PASS]
[BENCH] Setpoint=20 mA | AvgI= 19.8 mA | Duty= 12.9 % | VBAT=11979 mV | [PASS]
[BENCH] Setpoint=25 mA | AvgI= 25.2 mA | Duty= 16.3 % | VBAT=11977 mV | [PASS]
[BENCH] Setpoint=30 mA | AvgI= 29.7 mA | Duty= 19.4 % | VBAT=11975 mV | [PASS]
[BENCH] Setpoint=35 mA | AvgI= 34.9 mA | Duty= 22.7 % | VBAT=11973 mV | [PASS]
[BENCH] Setpoint=40 mA | AvgI= 40.1 mA | Duty= 26.0 % | VBAT=11971 mV | [PASS]
[BENCH] Result: 8/8 steps PASS
```

A **FAIL** line means the ICC loop did not settle within the tolerance in the hold window; see
[Troubleshooting](#troubleshooting).

---

## APIs Demonstrated

| API | Phase | Purpose in Example |
|-----|-------|-------------------|
| `Init()` | — | Initialize driver and SPI |
| `ConfigurePwmPeriod()` | — | Set 1 kHz ICC PWM |
| `ConfigureDither()` | — | 2 mA / 200 Hz dither |
| `EnterMissionModeChecked()` | 2 | Safe mission-mode entry with fault check |
| `ReadAllSupplyVoltages()` | 2 | Log VBAT/VIO/VDD/Tj before enabling |
| `SetCurrentSetpoint()` | — | Set each sweep step |
| `EnableChannel()` | — | Enable/disable CH0 |
| `ReadChannelFeedback()` | 6 | Atomic FB_FRZ/FB_UPD snapshot |

---

## Troubleshooting

### FAIL at low setpoints (< 10 mA)

- ICC PWM frequency may be too high for small duty cycles; try `ConfigurePwmPeriod(2000)` (500 Hz).
- Increase `BENCH_HOLD_MS` to 400 ms to allow the integrator more settle time.

### `ReadChannelFeedback()` returns `TimeoutError`

- The feedback freeze handshake (FB_FRZ/FB_UPD) timed out. Increase `BENCH_FB_TIMEOUT_MS` to 100 ms.
- Verify that CH0 is in ICC mode and the channel is enabled before calling.

### All setpoints read ≈ 0 mA

- Check that CH0 OUT is connected to the load. A floating output will show near-zero current.
- Verify EN pin is high (active high global enable).

### `EnterMissionModeChecked()` returns `FaultDetected`

- Check VBAT is within 5.5–41 V range.
- Check VIO matches the configured level (default 3.3 V).
- Call `PrintAllFaults()` to identify the specific fault source.

---

## Related Documentation

- [API Reference](../../docs/api_reference.md)
- [Hardware Setup](../../docs/hardware_setup.md)
- [Driver Integration Test](README_DRIVER_INTEGRATION_TEST.md)
- [Diagnostics & BIST](README_DIAGNOSTICS_AND_BIST.md)
