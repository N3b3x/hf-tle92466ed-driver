---
layout: default
title: "💡 Examples"
description: "Complete example walkthroughs for the TLE92466ED driver"
nav_order: 7
parent: "📚 Documentation"
permalink: /docs/examples/
---

# Examples

This guide provides complete, working examples demonstrating various use cases for the TLE92466ED driver.

## Example 1: Basic Solenoid Control

This example shows basic solenoid control with current regulation.

```cpp
#include "tle92466ed.hpp"
#include "esp32_tle_comm_interface.hpp"

void app_main() {
    // 1. Create SPI interface
    auto hal = CreateEsp32TleCommInterface();
    if (!hal || hal->Init().has_value() == false) {
        printf("HAL initialization failed\n");
        return;
    }
    
    // 2. Create driver
    tle92466ed::Driver driver(*hal);
    
    // 3. Initialize
    if (auto result = driver.Init(); !result) {
        printf("Initialization failed\n");
        return;
    }
    
    // 4. Configure channel 0 for 1.5A current control
    driver.SetChannelMode(tle92466ed::Channel::CH0, tle92466ed::ChannelMode::ICC);
    driver.SetCurrentSetpoint(tle92466ed::Channel::CH0, 1500); // 1.5A
    
    // 5. Enter Mission Mode
    driver.EnterMissionMode();
    
    // 6. Enable channel
    driver.EnableChannel(tle92466ed::Channel::CH0, true);
    
    // 7. Monitor current
    while (true) {
        if (auto current = driver.GetAverageCurrent(tle92466ed::Channel::CH0); current) {
            printf("Channel 0 current: %u mA\n", *current);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```cpp

## Example 2: Parallel Operation (High Current)

This example demonstrates parallel channel operation for higher current capability.

```cpp
void configure_high_current_solenoid() {
    auto hal = CreateEsp32TleCommInterface();
    tle92466ed::Driver driver(*hal);
    driver.Init();
    
    // Enter Config Mode
    driver.EnterConfigMode();
    
    // Enable parallel operation CH0+CH3
    driver.SetParallelOperation(tle92466ed::ParallelPair::CH0_CH3, true);
    
    // Configure both channels identically
    tle92466ed::ChannelConfig config;
    config.mode = tle92466ed::ChannelMode::ICC;
    config.current_setpoint_ma = 3500;  // 3.5A total (parallel mode)
    config.slew_rate = tle92466ed::SlewRate::MEDIUM_2V5_US;
    
    driver.ConfigureChannel(tle92466ed::Channel::CH0, config);
    driver.ConfigureChannel(tle92466ed::Channel::CH3, config);
    
    // Enter Mission Mode and enable
    driver.EnterMissionMode();
    driver.EnableChannel(tle92466ed::Channel::CH0, true);
    driver.EnableChannel(tle92466ed::Channel::CH3, true);
    
    printf("High-current mode active: 3.5A\n");
}
```cpp

## Example 3: Fault Monitoring

This example shows how to monitor and handle faults.

```cpp
void monitor_faults() {
    tle92466ed::Driver driver(*hal);
    driver.Init();
    driver.EnterMissionMode();
    
    // Enable channel
    driver.EnableChannel(tle92466ed::Channel::CH0, true);
    
    while (true) {
        // Check for faults
        if (auto has_fault = driver.HasAnyFault(); has_fault && *has_fault) {
            // Get detailed fault report
            if (auto faults = driver.GetAllFaults(); faults) {
                auto& report = *faults;
                
                if (report.vbat_uv) {
                    printf("VBAT undervoltage detected!\n");
                }
                
                if (report.channels[0].overcurrent) {
                    printf("Channel 0 overcurrent!\n");
                    driver.DisableAllChannels();
                }
                
                // Clear faults after handling
                driver.ClearFaults();
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```cpp

## Example 4: Multi-Channel Control

This example demonstrates controlling multiple channels independently.

```cpp
void multi_channel_control() {
    tle92466ed::Driver driver(*hal);
    driver.Init();
    driver.EnterMissionMode();
    
    // Configure multiple channels with different currents
    driver.SetChannelMode(tle92466ed::Channel::CH0, tle92466ed::ChannelMode::ICC);
    driver.SetChannelMode(tle92466ed::Channel::CH1, tle92466ed::ChannelMode::ICC);
    driver.SetChannelMode(tle92466ed::Channel::CH2, tle92466ed::ChannelMode::ICC);
    
    driver.SetCurrentSetpoint(tle92466ed::Channel::CH0, 1000); // 1A
    driver.SetCurrentSetpoint(tle92466ed::Channel::CH1, 1500); // 1.5A
    driver.SetCurrentSetpoint(tle92466ed::Channel::CH2, 800);  // 0.8A
    
    // Enable all channels
    driver.EnableChannels(0x07); // Bits 0,1,2 = CH0, CH1, CH2
    
    // Monitor all channels
    while (true) {
        for (uint8_t ch = 0; ch < 3; ch++) {
            if (auto current = driver.GetAverageCurrent(static_cast<tle92466ed::Channel>(ch)); current) {
                printf("CH%d: %u mA\n", ch, *current);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```text

## Example 5: PWM and Dither Configuration

This example shows advanced PWM and dither configuration.

```cpp
void advanced_current_control() {
    tle92466ed::Driver driver(*hal);
    driver.Init();
    driver.EnterConfigMode();
    
    // Configure channel 0
    driver.SetChannelMode(tle92466ed::Channel::CH0, tle92466ed::ChannelMode::ICC);
    
    // Set PWM period to 100 µs
    driver.ConfigurePwmPeriod(tle92466ed::Channel::CH0, 100.0f);
    
    // Configure dither: 50mA amplitude, 1000Hz frequency
    driver.ConfigureDither(tle92466ed::Channel::CH0, 50.0f, 1000.0f);
    
    // Set current
    driver.SetCurrentSetpoint(tle92466ed::Channel::CH0, 1500);
    
    driver.EnterMissionMode();
    driver.EnableChannel(tle92466ed::Channel::CH0, true);
}
```text

## Example 6: Complete Channel Configuration

This example shows complete channel configuration using ChannelConfig structure.

```cpp
void complete_channel_setup() {
    tle92466ed::Driver driver(*hal);
    driver.Init();
    driver.EnterConfigMode();
    
    // Complete channel configuration
    tle92466ed::ChannelConfig config;
    config.mode = tle92466ed::ChannelMode::ICC;
    config.current_setpoint_ma = 1500;
    config.slew_rate = tle92466ed::SlewRate::MEDIUM_2V5_US;
    config.diag_current = tle92466ed::DiagCurrent::I_80UA;
    config.open_load_threshold = 3;  // 3/8 threshold
    config.pwm_period_mantissa = 100;
    config.pwm_period_exponent = 3;
    config.deep_dither_enabled = false;
    config.dither_step_size = 50;
    config.dither_steps = 10;
    config.dither_flat = 5;
    
    driver.ConfigureChannel(tle92466ed::Channel::CH0, config);
    
    driver.EnterMissionMode();
    driver.EnableChannel(tle92466ed::Channel::CH0, true);
}
```cpp

## Example 7: Voltage Monitoring

This example demonstrates monitoring supply voltages.

```cpp
void monitor_voltages() {
    tle92466ed::Driver driver(*hal);
    driver.Init();
    
    while (true) {
        // Read voltages
        if (auto vbat = driver.GetVbatVoltage(); vbat) {
            printf("VBAT: %u mV\n", *vbat);
        }
        
        if (auto vio = driver.GetVioVoltage(); vio) {
            printf("VIO: %u mV\n", *vio);
        }
        
        if (auto vdd = driver.GetVddVoltage(); vdd) {
            printf("VDD: %u mV\n", *vdd);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```cpp

## Next Steps

- Review the [API Reference](api_reference.md) for all available methods
- Check [Configuration](configuration.md) for customization options
- See [Troubleshooting](troubleshooting.md) for common issues

---

**Navigation**
⬅️ [Configuration](configuration.md) | [Next: Troubleshooting ➡️](troubleshooting.md) | [Back to Index](index.md)


## Bench Examples (Phase 7)

### Resistive-Load Bench

File: `examples/esp32/main/resistive_load_bench.cpp`

Connect a 300 Ohm power resistor between CH0 and GND with VBAT = 12 V (theoretical 40 mA max). The example sweeps setpoints 5...40 mA in 5 mA steps and uses the atomic `ReadChannelFeedback()` path (Phase 6) to verify each step is within +/-2.5 mA of the setpoint. Reports VBAT, duty cycle and IMIN/IMAX per step.

### Diagnostics + BIST

File: `examples/esp32/main/diagnostics_and_bist.cpp`

No external load required. Exercises `RunSffBist()`, `ReadPinStatus()`, `GetOperationState()`, `RunSupplyMonitorSelfTest()` and `SetFaultMask()`.

## New High-Level APIs (Phase 2-6)

| API | Purpose |
|-----|---------|
| `ConfigureClockSource(ClockSource, f_clk_Hz)` | Select internal osc vs external+PLL (CLK_DIV register) |
| `ReadAllSupplyVoltages()` | Atomic VBAT/VIO/VDD/temperature snapshot |
| `GetCentralTemperatureCelsius()` | Die temperature in degC from FB_VOLTAGE2 |
| `SetVioLevel(VioLevel)` | Switch VIO between 3.3 V and 5.0 V |
| `GetOperationState()` | Returns Reset / Config / Mission / CriticalFault |
| `EnterMissionModeChecked(timeout_ms)` | Polls FB_STAT.INIT_DONE after mode switch |
| `RunSupplyMonitorSelfTest()` | UV/OV swap, V1V5 UV/OV, OT_TEST sequence |
| `SetIntegratorLimits(ch, lim, auto_lim)` | INTEGRATOR_LIMIT with enforcement of datasheet constraint |
| `SetPwmControllerKi(ch, ki)` | 4-bit PWM controller proportional gain |
| `SetMinIntegratorThreshold(ch, int8)` | Signed MIN_INT_THRESH in CTRL register |
| `SetManualOnTimeMode(ch, on_time_us)` | DIRECT_DRIVE_SPI with constant on-time via TON |
| `SeedIntegratorThresholdFromFeedback(ch)` | Seeds ICC integrator from FB_INT_THRESH |
| `SetDitherAdvanced(ch, DitherSetup, parallel)` | Superset of ConfigureDither with sync + fast-meas |
| `SetOlsgTimeout(ch, code)` | 6-bit OLSG watchdog timeout in TON register |
| `SetOffStateDiagnostics(ch, oc, ol_fixed)` | CH_CONFIG: OC_DIAG_EN + OL_TH_FIXED |
| `RunSffBist(timeout_ms)` | Safe-state logic-BIST, returns BistResult |
| `ReadPinStatus()` | Snapshot of DRV0/DRV1/EN/FAULTN/FAULTN_FB |
| `SetFaultMask(MaskableFault, bool)` | Enable/mask per-source FAULTN contribution |
| `ReadChannelFeedback(ch, timeout_ms)` | Atomic FB_FRZ/FB_UPD snapshot: avg I, DC, VBAT, IMIN/IMAX, Tmin/Tmax, int_thresh |
| `ReadAllChannelFeedback(timeout_ms)` | Same for all six channels |
| `GetCalibrationAvgCurrent_mA(ch)` | High-res signed FB_I_AVG_s16 for calibration |

