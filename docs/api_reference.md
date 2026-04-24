---
layout: default
title: "📖 API Reference"
description: "Complete API reference for the TLE92466ED driver"
nav_order: 6
parent: "📚 Documentation"
permalink: /docs/api_reference/
---

# API Reference

Complete reference documentation for all public methods and types in the TLE92466ED driver.

## Source Code

- **Main Header**: [`inc/tle92466ed.hpp`](../inc/tle92466ed.hpp)
- **SPI Interface**: [`inc/tle92466ed_spi_interface.hpp`](../inc/tle92466ed_spi_interface.hpp)
- **Registers**: [`inc/tle92466ed_registers.hpp`](../inc/tle92466ed_registers.hpp)
- **Template Implementation**: [`src/tle92466ed.ipp`](../src/tle92466ed.ipp)

## Core Class

### `Driver<CommType>`

Main driver class for interfacing with the TLE92466ED Six-Channel Low-Side Solenoid Driver IC.

**Template Parameter**: `CommType` - Your SPI interface implementation (must inherit from `tle92466ed::SpiInterface<CommType>`)

**Location**: [`inc/tle92466ed.hpp#L289`](../inc/tle92466ed.hpp#L289)

**Constructor:**

```cpp
explicit Driver(CommType& comm) noexcept;
```text

**Location**: [`inc/tle92466ed.hpp#L300`](../inc/tle92466ed.hpp#L300)

## Methods

### Initialization and Mode Control

| Method | Signature | Location |
|--------|-----------|----------|
| `Init()` | `DriverResult<void> Init() noexcept` | [`inc/tle92466ed.hpp#L343`](../inc/tle92466ed.hpp#L343) |
| `EnterMissionMode()` | `DriverResult<void> EnterMissionMode() noexcept` | [`inc/tle92466ed.hpp#L356`](../inc/tle92466ed.hpp#L356) |
| `EnterConfigMode()` | `DriverResult<void> EnterConfigMode() noexcept` | [`inc/tle92466ed.hpp#L367`](../inc/tle92466ed.hpp#L367) |
| `IsMissionMode()` | `bool IsMissionMode() const noexcept` | [`inc/tle92466ed.hpp#L373`](../inc/tle92466ed.hpp#L373) |
| `IsConfigMode()` | `bool IsConfigMode() const noexcept` | [`inc/tle92466ed.hpp#L381`](../inc/tle92466ed.hpp#L381) |
| `IsInitialized()` | `bool IsInitialized() const noexcept` | [`inc/tle92466ed.hpp#L785`](../inc/tle92466ed.hpp#L785) |

### Global Configuration

| Method | Signature | Location |
|--------|-----------|----------|
| `ConfigureGlobal()` | `DriverResult<void> ConfigureGlobal(const GlobalConfig& config) noexcept` | [`inc/tle92466ed.hpp#L397`](../inc/tle92466ed.hpp#L397) |
| `SetCrcEnabled()` | `DriverResult<void> SetCrcEnabled(bool enabled) noexcept` | [`inc/tle92466ed.hpp#L405`](../inc/tle92466ed.hpp#L405) |
| `SetVbatThresholds()` | `DriverResult<void> SetVbatThresholds(float uv_voltage, float ov_voltage) noexcept` | [`inc/tle92466ed.hpp#L421`](../inc/tle92466ed.hpp#L421) |
| `SetVbatThresholdsRaw()` | `DriverResult<void> SetVbatThresholdsRaw(uint8_t uv_threshold, uint8_t ov_threshold) noexcept` | [`inc/tle92466ed.hpp#L433`](../inc/tle92466ed.hpp#L433) |

### Channel Control

| Method | Signature | Location |
|--------|-----------|----------|
| `EnableChannel()` | `DriverResult<void> EnableChannel(Channel channel, bool enabled) noexcept` | [`inc/tle92466ed.hpp#L448`](../inc/tle92466ed.hpp#L448) |
| `EnableChannels()` | `DriverResult<void> EnableChannels(uint8_t channel_mask) noexcept` | [`inc/tle92466ed.hpp#L456`](../inc/tle92466ed.hpp#L456) |
| `EnableAllChannels()` | `DriverResult<void> EnableAllChannels() noexcept` | [`inc/tle92466ed.hpp#L461`](../inc/tle92466ed.hpp#L461) |
| `DisableAllChannels()` | `DriverResult<void> DisableAllChannels() noexcept` | [`inc/tle92466ed.hpp#L466`](../inc/tle92466ed.hpp#L466) |
| `SetChannelMode()` | `DriverResult<void> SetChannelMode(Channel channel, ChannelMode mode) noexcept` | [`inc/tle92466ed.hpp#L476`](../inc/tle92466ed.hpp#L476) |
| `SetParallelOperation()` | `DriverResult<void> SetParallelOperation(ParallelPair pair, bool enabled) noexcept` | [`inc/tle92466ed.hpp#L486`](../inc/tle92466ed.hpp#L486) |

### Current Control

| Method | Signature | Location |
|--------|-----------|----------|
| `SetCurrentSetpoint()` | `DriverResult<void> SetCurrentSetpoint(Channel channel, uint16_t current_ma, bool parallel_mode = false) noexcept` | [`inc/tle92466ed.hpp#L511`](../inc/tle92466ed.hpp#L511) |
| `GetCurrentSetpoint()` | `DriverResult<uint16_t> GetCurrentSetpoint(Channel channel, bool parallel_mode = false) noexcept` | [`inc/tle92466ed.hpp#L521`](../inc/tle92466ed.hpp#L521) |

### PWM Configuration

| Method | Signature | Location |
|--------|-----------|----------|
| `ConfigurePwmPeriod()` | `DriverResult<void> ConfigurePwmPeriod(Channel channel, float period_us) noexcept` | [`inc/tle92466ed.hpp#L542`](../inc/tle92466ed.hpp#L542) |
| `ConfigurePwmPeriodRaw()` | `DriverResult<void> ConfigurePwmPeriodRaw(Channel channel, uint8_t period_mantissa, uint8_t period_exponent, bool low_freq_range = false) noexcept` | [`inc/tle92466ed.hpp#L559`](../inc/tle92466ed.hpp#L559) |

### Dither Configuration

| Method | Signature | Location |
|--------|-----------|----------|
| `ConfigureDither()` | `DriverResult<void> ConfigureDither(Channel channel, float amplitude_ma, float frequency_hz, bool parallel_mode = false) noexcept` | [`inc/tle92466ed.hpp#L583`](../inc/tle92466ed.hpp#L583) |
| `ConfigureDitherRaw()` | `DriverResult<void> ConfigureDitherRaw(Channel channel, uint16_t step_size, uint8_t num_steps, uint8_t flat_steps) noexcept` | [`inc/tle92466ed.hpp#L604`](../inc/tle92466ed.hpp#L604) |

### Channel Configuration

| Method | Signature | Location |
|--------|-----------|----------|
| `ConfigureChannel()` | `DriverResult<void> ConfigureChannel(Channel channel, const ChannelConfig& config) noexcept` | [`inc/tle92466ed.hpp#L615`](../inc/tle92466ed.hpp#L615) |

### Status and Diagnostics

| Method | Signature | Location |
|--------|-----------|----------|
| `GetDeviceStatus()` | `DriverResult<DeviceStatus> GetDeviceStatus() noexcept` | [`inc/tle92466ed.hpp#L627`](../inc/tle92466ed.hpp#L627) |
| `GetChannelDiagnostics()` | `DriverResult<ChannelDiagnostics> GetChannelDiagnostics(Channel channel) noexcept` | [`inc/tle92466ed.hpp#L635`](../inc/tle92466ed.hpp#L635) |
| `GetAverageCurrent()` | `DriverResult<uint16_t> GetAverageCurrent(Channel channel, bool parallel_mode = false) noexcept` | [`inc/tle92466ed.hpp#L644`](../inc/tle92466ed.hpp#L644) |
| `GetDutyCycle()` | `DriverResult<uint16_t> GetDutyCycle(Channel channel) noexcept` | [`inc/tle92466ed.hpp#L653`](../inc/tle92466ed.hpp#L653) |

### Voltage Monitoring

| Method | Signature | Location |
|--------|-----------|----------|
| `GetVbatVoltage()` | `DriverResult<uint16_t> GetVbatVoltage() noexcept` | [`inc/tle92466ed.hpp#L660`](../inc/tle92466ed.hpp#L660) |
| `GetVioVoltage()` | `DriverResult<uint16_t> GetVioVoltage() noexcept` | [`inc/tle92466ed.hpp#L667`](../inc/tle92466ed.hpp#L667) |
| `GetVddVoltage()` | `DriverResult<uint16_t> GetVddVoltage() noexcept` | [`inc/tle92466ed.hpp#L674`](../inc/tle92466ed.hpp#L674) |
| `GetVbatThresholds()` | `DriverResult<void> GetVbatThresholds(uint16_t& uv_threshold, uint16_t& ov_threshold) noexcept` | [`inc/tle92466ed.hpp#L683`](../inc/tle92466ed.hpp#L683) |

### Fault Management

| Method | Signature | Location |
|--------|-----------|----------|
| `ClearFaults()` | `DriverResult<void> ClearFaults() noexcept` | [`inc/tle92466ed.hpp#L698`](../inc/tle92466ed.hpp#L698) |
| `HasAnyFault()` | `DriverResult<bool> HasAnyFault() noexcept` | [`inc/tle92466ed.hpp#L705`](../inc/tle92466ed.hpp#L705) |
| `GetAllFaults()` | `DriverResult<FaultReport> GetAllFaults() noexcept` | [`inc/tle92466ed.hpp#L716`](../inc/tle92466ed.hpp#L716) |
| `PrintAllFaults()` | `DriverResult<void> PrintAllFaults() noexcept` | [`inc/tle92466ed.hpp#L727`](../inc/tle92466ed.hpp#L727) |
| `IsFault()` | `DriverResult<bool> IsFault(bool print_faults = false) noexcept` | [`inc/tle92466ed.hpp#L895`](../inc/tle92466ed.hpp#L895) |

### Watchdog Management

| Method | Signature | Location |
|--------|-----------|----------|
| `ReloadSpiWatchdog()` | `DriverResult<void> ReloadSpiWatchdog(uint16_t reload_value) noexcept` | [`inc/tle92466ed.hpp#L753`](../inc/tle92466ed.hpp#L753) |

### Device Information

| Method | Signature | Location |
|--------|-----------|----------|
| `GetIcVersion()` | `DriverResult<uint16_t> GetIcVersion() noexcept` | [`inc/tle92466ed.hpp#L764`](../inc/tle92466ed.hpp#L764) |
| `GetChipId()` | `DriverResult<std::array<uint16_t, 3>> GetChipId() noexcept` | [`inc/tle92466ed.hpp#L771`](../inc/tle92466ed.hpp#L771) |
| `VerifyDevice()` | `DriverResult<bool> VerifyDevice() noexcept` | [`inc/tle92466ed.hpp#L778`](../inc/tle92466ed.hpp#L778) |

### GPIO Control

| Method | Signature | Location |
|--------|-----------|----------|
| `SetReset()` | `DriverResult<void> SetReset(bool reset) noexcept` | [`inc/tle92466ed.hpp#L807`](../inc/tle92466ed.hpp#L807) |
| `HoldReset()` | `DriverResult<void> HoldReset() noexcept` | [`inc/tle92466ed.hpp#L818`](../inc/tle92466ed.hpp#L818) |
| `ReleaseReset()` | `DriverResult<void> ReleaseReset() noexcept` | [`inc/tle92466ed.hpp#L831`](../inc/tle92466ed.hpp#L831) |
| `SetEnable()` | `DriverResult<void> SetEnable(bool enable) noexcept` | [`inc/tle92466ed.hpp#L849`](../inc/tle92466ed.hpp#L849) |
| `Enable()` | `DriverResult<void> Enable() noexcept` | [`inc/tle92466ed.hpp#L860`](../inc/tle92466ed.hpp#L860) |
| `Disable()` | `DriverResult<void> Disable() noexcept` | [`inc/tle92466ed.hpp#L873`](../inc/tle92466ed.hpp#L873) |

### Register Access

| Method | Signature | Location |
|--------|-----------|----------|
| `ReadRegister()` | `DriverResult<uint32_t> ReadRegister(uint16_t address, uint8_t retries = 0) noexcept` | [`inc/tle92466ed.hpp#L911`](../inc/tle92466ed.hpp#L911) |
| `WriteRegister()` | `DriverResult<void> WriteRegister(uint16_t address, uint16_t value, uint8_t retries = 0) noexcept` | [`inc/tle92466ed.hpp#L928`](../inc/tle92466ed.hpp#L928) |
| `ModifyRegister()` | `DriverResult<void> ModifyRegister(uint16_t address, uint16_t mask, uint16_t value, uint8_t retries = 0) noexcept` | [`inc/tle92466ed.hpp#L940`](../inc/tle92466ed.hpp#L940) |

### System Control

| Method | Signature | Location |
|--------|-----------|----------|
| `SoftwareReset()` | `DriverResult<void> SoftwareReset() noexcept` | [`inc/tle92466ed.hpp#L737`](../inc/tle92466ed.hpp#L737) |

---

## Phase 2-6 Extended APIs

### Power, Clock & State Management

These methods expose the full clock-configuration PLL, supply-voltage readback, VIO level control, and a safe mission-mode entry check.

| Method | Signature | Description |
|--------|-----------|-------------|
| `ConfigureClockSource()` | `DriverResult<void> ConfigureClockSource(ClockSource src, uint32_t f_clk_hz = 0) noexcept` | Configure internal OSC or external clock with PLL (calculates REFDIV/FBDIV automatically from `f_clk_hz`) |
| `GetSystemClockHz()` | `static constexpr uint32_t GetSystemClockHz() noexcept` | Returns `CLK_DIV::F_SYS_TARGET_HZ` (28 MHz) |
| `ReadAllSupplyVoltages()` | `DriverResult<SupplyVoltages> ReadAllSupplyVoltages() noexcept` | Read VBAT, VIO, VDD in mV and junction temperature in °C |
| `GetCentralTemperatureCelsius()` | `DriverResult<float> GetCentralTemperatureCelsius() noexcept` | Read central die temperature in °C from ADC register |
| `SetVioLevel()` | `DriverResult<void> SetVioLevel(VioLevel level) noexcept` | Switch GLOBAL_CONFIG VIO field between 3.3 V and 5.0 V |
| `GetOperationState()` | `OperationState GetOperationState() const noexcept` | Inspect current driver state: Reset, Config, Mission, or CriticalFault |
| `EnterMissionModeChecked()` | `DriverResult<void> EnterMissionModeChecked(uint32_t settle_ms = 10) noexcept` | Enter mission mode, wait `settle_ms`, verify no fault, return error if fault detected |
| `RunSupplyMonitorSelfTest()` | `DriverResult<SupplyMonitorSelfTestResult> RunSupplyMonitorSelfTest() noexcept` | Execute four-phase supply-monitor self-test (UV/OV swap, V1V5 UV/OV, OT_TEST); restores config on completion |

### ICC Integrator Tuning

Fine-grained control over the ICC integrator: limits, Ki gain, manual on-time mode, and threshold seeding from live feedback.

| Method | Signature | Description |
|--------|-----------|-------------|
| `SetIntegratorLimits()` | `DriverResult<void> SetIntegratorLimits(uint16_t lim_abs_ma, uint16_t auto_lim_ma) noexcept` | Write INTEGRATOR_LIMIT; enforces datasheet constraint `auto_lim > lim_abs + 3` |
| `SetPwmControllerKi()` | `DriverResult<void> SetPwmControllerKi(uint8_t ki_param) noexcept` | Write the 4-bit Ki parameter into `PERIOD::PWM_CTRL_PARAM[15:12]` |
| `SetMinIntegratorThreshold()` | `DriverResult<void> SetMinIntegratorThreshold(Channel ch, int8_t int_thresh) noexcept` | Write the signed 8-bit integrator threshold floor in CTRL_INT_THRESH |
| `SetManualOnTimeMode()` | `DriverResult<void> SetManualOnTimeMode(Channel ch, uint32_t ton_ns) noexcept` | Set manual on-time (fit mantissa in 10 bits by iterating EXP 0-15), zeroes PERIOD_MANT |
| `SeedIntegratorThresholdFromFeedback()` | `DriverResult<void> SeedIntegratorThresholdFromFeedback(Channel ch) noexcept` | Read FB_INT_THRESH feedback and write its value back to CTRL_INT_THRESH to warm-start the integrator |

### Advanced Dither

| Method | Signature | Description |
|--------|-----------|-------------|
| `SetDitherAdvanced()` | `DriverResult<void> SetDitherAdvanced(Channel ch, const DitherSetup& setup, bool parallel_mode = false) noexcept` | Write all dither registers in one call: DITHER_CLK_DIV + DITHER_STEP + DITHER_CTRL with sync, deep-dither, and fast-measure-mode flags |

### Diagnostics Suite

| Method | Signature | Description |
|--------|-----------|-------------|
| `SetOlsgTimeout()` | `DriverResult<void> SetOlsgTimeout(Channel ch, uint8_t timeout_code) noexcept` | Write 6-bit OLSG_TIMEOUT into TON register bits [15:10] |
| `SetOffStateDiagnostics()` | `DriverResult<void> SetOffStateDiagnostics(Channel ch, bool enable, DiagCurrent current = DiagCurrent::I_190UA) noexcept` | Enable/disable open-load off-state diagnostic injection current |
| `RunSffBist()` | `DriverResult<BistResult> RunSffBist(uint32_t timeout_ms = 10) noexcept` | Trigger SFF_BIST, poll DONE bit up to `timeout_ms`, return pass/fail and correctable/uncorrectable error flags |
| `ReadPinStatus()` | `DriverResult<PinStatus> ReadPinStatus() noexcept` | Read PIN_STAT central register and decode DRV0, DRV1, EN, FAULTN, FAULTN_FB |
| `SetFaultMask()` | `DriverResult<void> SetFaultMask(MaskableFault fault, bool contribute_to_faultn) noexcept` | Enable or suppress a specific fault source in FAULT_MASK0/1/2 |

### Atomic Channel Feedback

| Method | Signature | Description |
|--------|-----------|-------------|
| `ReadChannelFeedback()` | `DriverResult<ChannelFeedback> ReadChannelFeedback(Channel ch, uint32_t timeout_ms = 5) noexcept` | Freeze feedback with FB_FRZ, poll FB_UPD every 200 µs until coherent snapshot, read DC/VBAT/I_AVG/IMIN_IMAX/PERIOD_MIN_MAX/INT_THRESH, release freeze |
| `ReadAllChannelFeedback()` | `DriverResult<std::array<ChannelFeedback, 6>> ReadAllChannelFeedback(uint32_t timeout_ms = 5) noexcept` | Call `ReadChannelFeedback()` for all 6 channels, return array |
| `GetCalibrationAvgCurrent_mA()` | `DriverResult<int32_t> GetCalibrationAvgCurrent_mA(Channel ch) noexcept` | Return only the average current field from `ReadChannelFeedback()` without allocating the full struct on the call stack |

## Types

### Enumerations

| Type | Values | Location |
|------|--------|----------|
| `DriverError` | `None`, `NotInitialized`, `HardwareError`, `InvalidChannel`, `InvalidParameter`, `DeviceNotResponding`, `WrongDeviceID`, `RegisterError`, `CRCError`, `FaultDetected`, `ConfigurationError`, `TimeoutError`, `WrongMode`, `SPIFrameError`, `WriteToReadOnly` | [`inc/tle92466ed.hpp#L79`](../inc/tle92466ed.hpp#L79) |
| `Channel` | `CH0`, `CH1`, `CH2`, `CH3`, `CH4`, `CH5`, `COUNT` | [`inc/tle92466ed_registers.hpp#L1053`](../inc/tle92466ed_registers.hpp#L1053) |
| `ChannelMode` | `OFF`, `ICC`, `DIRECT_DRIVE_SPI`, `DIRECT_DRIVE_DRV0`, `DIRECT_DRIVE_DRV1`, `FREE_RUN_MEAS` | [`inc/tle92466ed_registers.hpp#L1067`](../inc/tle92466ed_registers.hpp#L1067) |
| `ParallelPair` | `NONE`, `CH0_CH3`, `CH1_CH2`, `CH4_CH5` | [`inc/tle92466ed_registers.hpp#L1099`](../inc/tle92466ed_registers.hpp#L1099) |
| `SlewRate` | `SLOW_1V0_US`, `MEDIUM_2V5_US`, `FAST_5V0_US`, `FASTEST_10V0_US` | [`inc/tle92466ed_registers.hpp#L1079`](../inc/tle92466ed_registers.hpp#L1079) |
| `DiagCurrent` | `I_80UA`, `I_190UA`, `I_720UA`, `I_1250UA` | [`inc/tle92466ed_registers.hpp#L1089`](../inc/tle92466ed_registers.hpp#L1089) |
| `ClockSource` | `InternalOscillator`, `ExternalClockPll` | [`inc/tle92466ed_registers.hpp`](../inc/tle92466ed_registers.hpp) |
| `VioLevel` | `V3_3` (= 0), `V5_0` (= 1) | [`inc/tle92466ed_registers.hpp`](../inc/tle92466ed_registers.hpp) |
| `OperationState` | `Reset`, `Config`, `Mission`, `CriticalFault` | [`inc/tle92466ed_registers.hpp`](../inc/tle92466ed_registers.hpp) |
| `FastMeasureMode` | `FullPeriod` (= 0), `HalfPeriod` (= 1), `QuarterPeriod` (= 2) | [`inc/tle92466ed_registers.hpp`](../inc/tle92466ed_registers.hpp) |
| `MaskableFault` | `Ch0Error`…`Ch5Error`, `EnPin`, `SupplyNokInternal`, `SupplyNokExternal` (MASK0); `Ch0Warning`…`Ch5Warning`, `CentralOtWarning`, `CentralOtError`, `ClockLow` (MASK1); `VbatUv`, `VbatOv`, `VioUv`, `VioOv`, `VddUv`, `VddOv` (MASK2). Upper byte encodes register index; low 16 bits encode mask bit. | [`inc/tle92466ed_registers.hpp`](../inc/tle92466ed_registers.hpp) |

### Structures

| Type | Description | Location |
|------|-------------|----------|
| `ChannelConfig` | Channel configuration structure | [`inc/tle92466ed.hpp#L109`](../inc/tle92466ed.hpp#L109) |
| `GlobalConfig` | Global configuration structure | [`inc/tle92466ed.hpp#L252`](../inc/tle92466ed.hpp#L252) |
| `DeviceStatus` | Global device status structure | [`inc/tle92466ed.hpp#L128`](../inc/tle92466ed.hpp#L128) |
| `ChannelDiagnostics` | Channel diagnostic information | [`inc/tle92466ed.hpp#L163`](../inc/tle92466ed.hpp#L163) |
| `FaultReport` | Comprehensive fault report structure | [`inc/tle92466ed.hpp#L192`](../inc/tle92466ed.hpp#L192) |
| `SupplyVoltages` | `vbat_mV`, `vio_mV`, `vdd_mV` (uint16_t), `temperature_c` (float) — returned by `ReadAllSupplyVoltages()` | [`inc/tle92466ed_registers.hpp`](../inc/tle92466ed_registers.hpp) |
| `DitherSetup` | `amplitude_mA`, `frequency_Hz`, `sync_with_pwm`, `sync_with_setpoint`, `deep_dither` (bool), `fast_measure` (FastMeasureMode) — passed to `SetDitherAdvanced()` | [`inc/tle92466ed_registers.hpp`](../inc/tle92466ed_registers.hpp) |
| `BistResult` | `done`, `pass`, `uncorrectable_reg_err`, `correctable_reg_err` (bool), `raw` (uint16_t) — returned by `RunSffBist()` | [`inc/tle92466ed_registers.hpp`](../inc/tle92466ed_registers.hpp) |
| `PinStatus` | `drv0`, `drv1`, `en`, `faultn_driver`, `faultn_fb` (bool), `raw` (uint16_t) — returned by `ReadPinStatus()` | [`inc/tle92466ed_registers.hpp`](../inc/tle92466ed_registers.hpp) |
| `ChannelFeedback` | `avg_current_mA` (int32_t), `duty_cycle_permyriad` (uint16_t), `avg_vbat_mV` (uint32_t), `imin_mA`/`imax_mA` (int32_t), `period_min_us`/`period_max_us` (uint32_t), `int_thresh_seed` (int16_t), `period_seq`/`quad_seq` (uint8_t) — returned by `ReadChannelFeedback()` | [`inc/tle92466ed_registers.hpp`](../inc/tle92466ed_registers.hpp) |
| `SupplyMonitorSelfTestResult` | `uv_ov_swap_ok`, `v1v5_uv_ok`, `v1v5_ov_ok`, `ot_test_ok`, `overall_pass` (bool) — returned by `RunSupplyMonitorSelfTest()` | [`inc/tle92466ed_registers.hpp`](../inc/tle92466ed_registers.hpp) |

### Type Aliases

| Type | Definition | Location |
|------|------------|----------|
| `DriverResult<T>` | `std::expected<T, DriverError>` | [`inc/tle92466ed.hpp#L100`](../inc/tle92466ed.hpp#L100) |

---

**Navigation**
⬅️ [Configuration](configuration.md) | [Next: Examples ➡️](examples.md) | [Back to Index](index.md)
