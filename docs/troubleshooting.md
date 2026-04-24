---
layout: default
title: "Troubleshooting"
description: "Common issues and solutions for the TLE92466ED driver"
nav_order: 8
parent: "📚 Documentation"
permalink: /docs/troubleshooting/
---

# Troubleshooting

This guide helps you diagnose and resolve common issues when using the TLE92466ED driver.

## Common Error Messages

### Error: Initialization Failed

**Symptoms:**

- `Init()` returns `DriverError::NotInitialized` or `DriverError::DeviceNotResponding`
- Driver not responding to SPI

**Causes:**

- SPI interface not properly initialized
- Hardware connections incorrect
- Power supply issues
- RESN pin held low

**Solutions:**

1. **Verify SPI Interface**: Ensure SPI interface is initialized before creating driver
2. **Check Connections**: Verify all SPI connections (SCK, SI, SO, CSN)
3. **Verify Power**: Check power supply voltages (VBAT: 5.5V-41V, VIO: 3.0V-5.5V)
4. **Check RESN Pin**: Ensure RESN is released (HIGH) for SPI communication
5. **Verify SPI Mode**: Ensure SPI Mode 1 (CPOL=0, CPHA=1)
6. **Check SPI Speed**: Try lower SPI speed (e.g., 1 MHz)

---

### Error: Communication Error / CRC Error

**Symptoms:**

- `DriverError::CRCError` or `DriverError::SPIFrameError` returned
- No response from device
- CRC mismatches

**Causes:**

- SPI configuration incorrect
- Signal integrity issues
- CS timing problems
- CRC calculation mismatch

**Solutions:**

1. **Check SPI Mode**: Ensure SPI Mode 1 (CPOL=0, CPHA=1)
2. **Verify Speed**: Try lower SPI speed (e.g., 1 MHz, max 10 MHz)
3. **Check CS Timing**: Verify CS assertion/deassertion timing
4. **Verify Connections**: Check all SPI connections are secure
5. **Check CRC**: Verify CRC-8 (SAE J1850) calculation matches device
6. **Check Frame Format**: Ensure 32-bit frames are used

---

### Error: Channel Not Working

**Symptoms:**

- Channel enabled but no output
- Current not flowing
- No voltage on OUTx pin

**Causes:**

- Channel not properly configured
- Channel not enabled
- Load not connected correctly
- Fault condition
- Wrong mode (Config vs Mission)

**Solutions:**

1. **Check Mode**: Ensure in Mission Mode (`EnterMissionMode()`)
2. **Check Configuration**: Verify channel configuration is correct
3. **Check Enable State**: Ensure channel is enabled via `EnableChannel()`
4. **Verify Load**: Check load connections (OUTx to load, load to VBAT)
5. **Check Faults**: Read fault status to identify issues
6. **Check Channel Mode**: Ensure channel mode is set (e.g., ICC mode)

---

### Error: Overcurrent Protection

**Symptoms:**

- `ChannelDiagnostics.overcurrent` is true
- Channel disabled automatically
- Current reading shows fault

**Causes:**

- Load current exceeds limits
- Short circuit
- Incorrect current settings
- Parallel mode not configured correctly

**Solutions:**

1. **Check Load**: Verify load is within specifications (2A max single, 4A parallel)
2. **Reduce Current**: Lower current setpoint
3. **Check for Shorts**: Verify no short circuits in wiring
4. **Clear Fault**: Read diagnostics to clear, then reconfigure
5. **Check Parallel Mode**: If using parallel, ensure `SetParallelOperation()` called

---

### Error: Open Load Detection

**Symptoms:**

- `ChannelDiagnostics.open_load` is true
- No current flow
- Load not detected

**Causes:**

- Load not connected
- Broken connection
- Load impedance too high
- Open load threshold too sensitive

**Solutions:**

1. **Check Connections**: Verify load is properly connected
2. **Check Wiring**: Inspect for broken wires
3. **Verify Load**: Ensure load impedance is appropriate
4. **Adjust Threshold**: Increase open load threshold if false positives

---

### Error: Wrong Mode

**Symptoms:**

- `DriverError::WrongMode` returned
- Operation not allowed

**Causes:**

- Attempting channel control in Config Mode
- Attempting configuration in Mission Mode
- Mode transition not completed

**Solutions:**

1. **Check Current Mode**: Use `IsMissionMode()` or `IsConfigMode()`
2. **Enter Correct Mode**:
   - For channel control: `EnterMissionMode()`
   - For configuration: `EnterConfigMode()`
3. **Wait for Transition**: Allow time for mode transition to complete

---

### Error: Invalid Parameter

**Symptoms:**

- `DriverError::InvalidParameter` returned
- Invalid channel number
- Parameter out of range

**Causes:**

- Channel number out of range (must be 0-5)
- Current value out of range
- Invalid configuration values

**Solutions:**

1. **Check Channel Number**: Use `Channel::CH0` through `Channel::CH5`
2. **Check Current Range**:
   - Single mode: 0-2000 mA
   - Parallel mode: 0-4000 mA
3. **Verify Parameters**: Check all configuration values are within valid ranges

---

### Error: Device Not Responding

**Symptoms:**

- `DriverError::DeviceNotResponding` returned
- No SPI response
- Timeout errors

**Causes:**

- SPI bus not working
- Device not powered
- CS line issue
- RESN pin held low

**Solutions:**

1. **Check Power**: Verify VBAT and VIO are within range
2. **Check RESN**: Ensure RESN is HIGH (not in reset)
3. **Check CS**: Verify CS line is properly controlled
4. **Check SPI Bus**: Test SPI bus with simple read/write
5. **Check Connections**: Verify all SPI connections

---

### Error: Wrong Device ID

**Symptoms:**

- `DriverError::WrongDeviceID` returned
- Device ID mismatch

**Causes:**

- Wrong device connected
- Device not fully powered up
- Communication error

**Solutions:**

1. **Verify Device**: Check that TLE92466ED is connected
2. **Wait for Power-Up**: Allow time for device power-up
3. **Check Communication**: Verify SPI communication is working
4. **Read ID Manually**: Use `GetIcVersion()` to check device ID

---

## Debugging Tips

### Enable Logging

```cpp
// Enable driver logging (if supported by your SPI interface)
hal->SetLogLevel(tle92466ed::LogLevel::Debug);
```cpp

### Read Device Status

```cpp
if (auto status = driver.GetDeviceStatus(); status) {
    printf("Config Mode: %d\n", status->config_mode);
    printf("Init Done: %d\n", status->init_done);
    printf("Any Fault: %d\n", status->any_fault);
}
```cpp

### Read All Faults

```cpp
if (auto faults = driver.GetAllFaults(); faults) {
    driver.PrintAllFaults();  // Prints formatted fault report
}
```cpp

### Verify Device

```cpp
if (auto verified = driver.VerifyDevice(); verified && *verified) {
    printf("Device verified\n");
} else {
    printf("Device verification failed\n");
}
```text

## Common Configuration Mistakes

1. **Forgetting to Enter Mission Mode**: Channels can only be enabled in Mission Mode
2. **Configuring in Wrong Mode**: Channel configuration requires Config Mode
3. **Not Setting Channel Mode**: Must set channel mode before setting current
4. **Parallel Mode Mismatch**: Forgetting to set `parallel_mode = true` when using parallel pairs
5. **Current Out of Range**: Setting current above 2000mA in single mode

## Next Steps

- Review [Hardware Setup](hardware_setup.md) for wiring verification
- Check [Platform Integration](platform_integration.md) for SPI interface issues
- See [Examples](examples.md) for working code samples

---

**Navigation**
⬅️ [Examples](examples.md) | [Back to Index](index.md)

---

## Phase 2–6 Extended API Issues

---

### Error: Clock Configuration — PLL Parameters Out of Range

**Symptoms:**

- `ConfigureClockSource()` returns `DriverError::InvalidParameter`
- System clock unstable or ICC loop not locking

**Causes:**

- External clock frequency outside 1–8 MHz PLL input range
- Calculated FBDIV does not fit in 9 bits (exceeds 511)

**Solutions:**

1. **Use external clock within 1–8 MHz**: `ConfigureClockSource(ClockSource::ExternalClockPll, 4000000)` (4 MHz)
2. **Check calculated FBDIV**: `FBDIV = round(28e6 / f_clk_hz) - 1` must be = 511 (satisfied for f_clk_hz = ~55 kHz, which is always true in the 1–8 MHz range)
3. **Use internal oscillator** if no external clock is available: `ConfigureClockSource(ClockSource::InternalOscillator)`

---

### Error: Integrator Limits — Constraint Violation

**Symptoms:**

- `SetIntegratorLimits()` returns `DriverError::InvalidParameter`
- ICC loop unstable or output current oscillating

**Causes:**

- `auto_lim_ma` = `lim_abs_ma + 3`: violates datasheet requirement that the auto-limit must exceed the absolute limit by at least 3 mA (register LSB units)

**Solutions:**

1. **Increase the gap**: ensure `auto_lim_ma >= lim_abs_ma + 4`
   ```cpp
   // Correct: gap = 20 mA
   driver.SetIntegratorLimits(100, 120);  // lim_abs=100, auto_lim=120
   ```
2. **Use defaults**: if `SetIntegratorLimits()` is not called, the reset values satisfy the constraint automatically

---

### Error: EnterMissionModeChecked — Fault on Entry

**Symptoms:**

- `EnterMissionModeChecked()` returns `DriverError::FaultDetected`
- `EnterMissionMode()` succeeds but a fault is present immediately after

**Causes:**

- VBAT outside UV/OV threshold window configured in GLOBAL_CONFIG
- VIO level mismatch (e.g. `VioLevel::V5_0` selected but only 3.3 V present)
- FAULTN pin stuck low due to prior fault not cleared

**Solutions:**

1. **Check supply voltages**: call `ReadAllSupplyVoltages()` *before* `EnterMissionModeChecked()` and verify VBAT is within your UV/OV window
2. **Clear prior faults**: call `ClearFaults()` after power-on before attempting mission mode
3. **Match VIO level**: ensure `SetVioLevel()` matches your hardware (default is `V3_3`)
4. **Increase settle time**: pass a larger `settle_ms` (e.g. 50) to allow supplies to stabilize

---

### Error: ReadChannelFeedback — Timeout

**Symptoms:**

- `ReadChannelFeedback()` returns `DriverError::TimeoutError`
- Feedback snapshot never coherent

**Causes:**

- Channel is disabled or not in ICC mode — the feedback registers are not updated
- `timeout_ms` too short for the configured PWM period (snapshot takes at least one full PWM period to update)
- FB_UPD bit not asserting because FB_FRZ was written to the wrong channel index

**Solutions:**

1. **Enable the channel first**: ensure `EnableChannel(ch, true)` has been called and the channel is in `ChannelMode::ICC`
2. **Increase timeout**: increase `timeout_ms` to at least `2 × PWM_period_ms`
   - For 1 kHz PWM (1 ms period) use `timeout_ms = 5` (default)
   - For 100 Hz PWM (10 ms period) use `timeout_ms = 30`
3. **Verify ICC is settled**: wait at least 200 ms after changing setpoint before reading feedback

---

### Error: RunSffBist — Not Done / Fail

**Symptoms:**

- `RunSffBist()` returns a result with `done=false` (timeout) or `pass=false`
- `uncorrectable_reg_err=true` or `correctable_reg_err=true`

**Causes:**

- `done=false`: BIST did not complete within `timeout_ms` — supply voltage marginal or timeout too short
- `pass=false` + `uncorrectable_reg_err=true`: hardware register memory fault in the IC
- `pass=false` + `correctable_reg_err=true`: single-bit error corrected by ECC — may be transient

**Solutions:**

1. **Increase timeout**: pass `timeout_ms = 50` if the default 10 ms is insufficient
2. **Verify supplies**: VBAT and VIO must be stable within spec for BIST to run
3. **Power-cycle and retry**: if `correctable_reg_err=true` on a single run, retry — transient upsets can occur
4. **Replace device**: if `uncorrectable_reg_err=true` persists across multiple power cycles, the IC has a permanent fault

---

### Error: RunSupplyMonitorSelfTest — Phase Failure

**Symptoms:**

- `RunSupplyMonitorSelfTest()` returns a result with one or more sub-test fields `false`
- `overall_pass=false`

**Causes:**

- Sub-test expected a fault response from the supply monitor but none was generated
- Supplies at marginal levels may prevent the test-mode injection from exceeding thresholds
- Fault was not cleared between sub-tests (inter-test state contamination)

**Solutions:**

1. **Run at nominal voltages**: ensure VBAT ≈ 12 V or 24 V and VIO ≈ 3.3 V or 5 V — do not run at supply extremes
2. **Do not run while loads are enabled**: disable all channels before calling `RunSupplyMonitorSelfTest()`
3. **Check CRC is enabled**: CRC-protected SPI is required for the test-mode write to take effect reliably; call `SetCrcEnabled(true)` before the self-test

---

**Navigation**
?? Back to [Documentation Index](index.md)
