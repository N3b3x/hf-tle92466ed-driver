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

### Structures

| Type | Description | Location |
|------|-------------|----------|
| `ChannelConfig` | Channel configuration structure | [`inc/tle92466ed.hpp#L109`](../inc/tle92466ed.hpp#L109) |
| `GlobalConfig` | Global configuration structure | [`inc/tle92466ed.hpp#L252`](../inc/tle92466ed.hpp#L252) |
| `DeviceStatus` | Global device status structure | [`inc/tle92466ed.hpp#L128`](../inc/tle92466ed.hpp#L128) |
| `ChannelDiagnostics` | Channel diagnostic information | [`inc/tle92466ed.hpp#L163`](../inc/tle92466ed.hpp#L163) |
| `FaultReport` | Comprehensive fault report structure | [`inc/tle92466ed.hpp#L192`](../inc/tle92466ed.hpp#L192) |

### Type Aliases

| Type | Definition | Location |
|------|------------|----------|
| `DriverResult<T>` | `std::expected<T, DriverError>` | [`inc/tle92466ed.hpp#L100`](../inc/tle92466ed.hpp#L100) |

---

**Navigation**
⬅️ [Configuration](configuration.md) | [Next: Examples ➡️](examples.md) | [Back to Index](index.md)
