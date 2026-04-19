# TLE92466ED ESP32-S3 Examples

This directory contains comprehensive examples demonstrating the
TLE92466ED driver on **ESP32-S3**. The example is intentionally pinned
to a single MCU/pinout so the build, flash, and verification flow stays
predictable; if you want to retarget another ESP32-S3 board, edit the
constants in `main/esp32_tle92466ed_test_config.hpp` (single source of
truth for every pin used by the example).

## 🎯 Overview

The examples exercise the TLE92466ED six-channel low-side solenoid
driver from a freshly powered-on board through:

- A **header-only ESP32-S3 SPI bus adapter** (`main/esp32_tle92466ed_bus.hpp`)
  that maps the driver's CRTP `SpiInterface` onto ESP-IDF's `spi_master`.
- A **comprehensive integration test** (`driver_integration_test.cpp`)
  that walks every API call group — config, channel control, current,
  PWM, dither, diagnostics, faults, watchdog, parallel-mode.
- A **real-hardware solenoid control test** (`solenoid_control_test.cpp`)
  that uses an ADC potentiometer to ramp current through two
  solenoids (one single-channel, one parallel pair).
- A **build/flash automation layer** (`scripts/`) that reads the
  declarative `app_config.yml` and orchestrates ESP-IDF.

## 🔧 Hardware Requirements

### ESP32-S3 development board
- Any ESP32-S3 dev board with the GPIOs in the table below available.
- USB-C cable for programming + native USB-Serial-JTAG console.

### TLE92466ED Connections (default reference pinout)

| TLE92466ED Pin | ESP32-S3 GPIO | Function |
|----------------|---------------|----------|
| MOSI   | GPIO37 | SPI3 Data Out (master → slave) |
| MISO   | GPIO35 | SPI3 Data In (slave → master) |
| SCLK   | GPIO36 | SPI3 Clock |
| CS     | GPIO4  | Chip Select (active-LOW) |
| RESN   | GPIO6  | Reset (active-LOW; HIGH = released) |
| EN     | GPIO5  | Output stage enable (active-HIGH) |
| FAULTN | GPIO16 | Fault output (active-LOW, open-drain) |
| DRV0   | GPIO7  | External drive control 0 (optional) |
| DRV1   | GPIO15 | External drive control 1 (optional) |
| VDD    | 3.3V   | Logic supply |
| VBAT   | 12-24V | Power supply for output stage |
| GND    | GND    | Ground (common to MCU and supply) |

### Load Connections
Connect your solenoids/loads to the TLE92466ED output channels (OUT0-OUT5) with appropriate current ratings.

## 🛠️ Console / serial output

The included `sdkconfig.defaults.esp32s3` pins the primary console to
the **ESP32-S3 native USB-Serial-JTAG bridge** (the device that
enumerates as `/dev/ttyACM0` on Linux / `COMx` on Windows). UART0 is
not used as the primary console because its default pins (GPIO43/44)
are usually not routed to the host on USB-only ESP32-S3 dev boards.

## 🚀 Quick Start

### 1. Prerequisites

```bash
# Install ESP-IDF v5.5
curl -fsSL https://raw.githubusercontent.com/espressif/esp-idf/master/tools/install.sh | bash
source ~/esp/esp-idf/export.sh

# Verify installation
idf.py --version
```text

### 2. Setup Repository

```bash
# Clone and setup
git clone --recursive https://github.com/n3b3x/hf-tle92466ed-driver.git
cd hf-tle92466ed-driver/examples/esp32

# Initialize build environment
./scripts/setup_repo.sh
```text

### 3. Build and Flash

```bash
# Build basic usage example (default)
./scripts/build_app.sh basic_usage Release

# Flash to ESP32-C6
./scripts/flash_app.sh basic_usage Release

# Monitor output
idf.py monitor
```text

## 📱 Available Examples

### 🟢 Basic Examples

#### `driver_integration_test`
**Comprehensive Driver Integration Test Suite**
- Complete driver API validation (40+ tests)
- 13 test sections covering all functionality
- Initialization, configuration, control, diagnostics
- Multi-channel and parallel operation testing
- Error condition validation
- Comprehensive telemetry testing
- Professional test framework with automatic result tracking
- **No actual solenoids required** (driver test only)

```bash
./scripts/build_app.sh driver_integration_test Debug
./scripts/flash_app.sh driver_integration_test Debug
```

**Documentation**: [Driver Integration Test](docs/readme_driver_integration_test.md)

#### `solenoid_control_test`
**Real Hardware Solenoid Control Test**
- Two solenoids: one single channel, one parallel pair
- ADC-based current control (0-3.3V maps to 0-100% current)
- Independent min/max current limits per solenoid
- Real-time current adjustment based on ADC reading
- Comprehensive real-time telemetry (every 1 second)
- Proper parallel channel operation validation
- **Requires actual solenoids and power supplies**

```bash
./scripts/build_app.sh solenoid_control_test Debug
./scripts/flash_app.sh solenoid_control_test Debug
```

**Documentation**: [Solenoid Control Test](docs/readme_solenoid_control_test.md)

### 🟡 Advanced Examples

#### `parallel_mode`
**High-current parallel channel operation**
- Channel pairing (CH0/CH3, CH1/CH2, CH4/CH5)
- 4A current control
- Load balancing
- Thermal management

#### `diagnostics_test`
**Comprehensive fault detection**
- Overcurrent detection
- Temperature monitoring
- Open load detection
- Short circuit detection
- Real-time fault reporting

#### `pwm_dither`
**Precision control with PWM and dither**
- Configurable PWM frequency
- Dither amplitude control
- Noise reduction techniques
- Precision current shaping

### 🔵 Application Examples

#### `automotive_demo`
**Automotive solenoid control**
- Transmission solenoid patterns
- Fuel injector timing
- Valve control with feedback
- Safety interlocks

#### `industrial_valve`
**Industrial valve control**
- Proportional valve control
- Position feedback integration
- Flow rate control
- PID control implementation

### 🟠 Testing Examples

#### `stress_test`
**Reliability and stress testing**
- Continuous operation testing
- Thermal cycling
- Communication error injection
- Recovery mechanisms

#### `calibration`
**Current calibration and accuracy**
- Current measurement calibration
- Temperature compensation
- Accuracy validation
- Factory calibration routines

## 🛠️ Build System

### App Configuration

All examples are configured in `app_config.yml`:

```yaml
apps:
  basic_usage:
    description: "Basic TLE92466ED usage example"
    source_file: "BasicUsageExample.cpp"
    category: "basic"
    idf_versions: ["release/v5.5"]
    build_types: ["Debug", "Release"]
```text

### Build Commands

```bash
# List available apps
python3 scripts/get_app_info.py list

# Get app information
python3 scripts/get_app_info.py info basic_usage

# Build specific app and type
./scripts/build_app.sh <app_name> <build_type>

# Flash specific app
./scripts/flash_app.sh <app_name> <build_type>

# Clean build
./scripts/build_app.sh <app_name> <build_type> --clean
```text

### Build Types

- **Debug**: Full debugging symbols, verbose logging, assertions
- **Release**: Optimized for performance and size
- **RelWithDebInfo**: Release with debug symbols
- **MinSizeRel**: Optimized for minimum size

## 📊 Configuration

### Hardware Configuration File

All hardware configuration is defined in `main/TLE92466ED_TestConfig.hpp`. This is the
**actual** configuration used by the HAL and examples.

**Important**: The `app_config.yml` file only contains application definitions and build
configuration, NOT hardware settings.

### SPI Configuration

Edit `main/TLE92466ED_TestConfig.hpp` to change SPI pins:

```cpp
struct SPIPins {
    static constexpr int MISO = 2;   // GPIO2 - SPI MISO
    static constexpr int MOSI = 7;   // GPIO7 - SPI MOSI
    static constexpr int SCLK = 6;   // GPIO6 - SPI Clock
    static constexpr int CS = 10;    // GPIO10 - Chip Select
};

struct SPIParams {
    static constexpr int FREQUENCY = 1000000;  // 1MHz
    static constexpr int MODE = 0;             // SPI Mode 0
};
```text

### Current Limits

Defined in `main/TLE92466ED_TestConfig.hpp`:

```cpp
struct CurrentLimits {
    static constexpr uint16_t SINGLE_CHANNEL_MAX = 2000;   // 2A per channel
    static constexpr uint16_t PARALLEL_CHANNEL_MAX = 4000; // 4A parallel mode
};
```text

### Hardware Specifications

Defined in `main/TLE92466ED_TestConfig.hpp`:

```cpp
struct SupplyVoltage {
    static constexpr float VBAT_MIN = 8.0f;    // Minimum VBAT (V)
    static constexpr float VBAT_MAX = 28.0f;   // Maximum VBAT (V)
};

struct Temperature {
    static constexpr int JUNCTION_MAX = 150;   // Maximum temp (°C)
};
```text

## 🔍 Debugging

### Serial Monitor

```bash
# Monitor with automatic port detection
idf.py monitor

# Monitor specific port
idf.py monitor -p /dev/ttyUSB0

# Monitor with filtering
idf.py monitor | grep "TLE92466ED"
```text

### Log Levels

Set in `sdkconfig` or via menuconfig:

```text
CONFIG_LOG_DEFAULT_LEVEL_DEBUG=y  # Debug level
CONFIG_TLE92466ED_DEBUG_ENABLED=y # Driver debug
```text

### Common Issues

1. **SPI Communication Errors**
   - Check wiring connections
   - Verify power supply (VBAT 12-24V, VDD 3.3V)
   - Ensure proper grounding

2. **Current Control Issues**
   - Verify load connections
   - Check current limits in configuration
   - Monitor diagnostics for faults

3. **Build Errors**
   - Ensure ESP-IDF v5.5 is installed
   - Run `./scripts/setup_repo.sh`
   - Check C++20 compiler support

## 📚 Documentation

### Example Documentation

Each example has detailed documentation in `docs/`:

- `docs/readme_driver_integration_test.md`
- `docs/README_MULTI_CHANNEL.md`
- `docs/README_PARALLEL_MODE.md`
- `docs/README_DIAGNOSTICS.md`
- And more...

### API Documentation

- [Driver API Reference](../../docs/07_Driver_API.md)
- [HAL Implementation Guide](../../docs/08_HAL_Implementation.md)
- [Register Map](../../docs/03_Register_Map.md)

## 🧪 Testing

### Unit Tests

```bash
# Run all tests
./scripts/build_app.sh stress_test Debug
./scripts/flash_app.sh stress_test Debug
```text

### Performance Testing

```bash
# Performance monitoring
./scripts/build_app.sh multi_channel Release
./scripts/flash_app.sh multi_channel Release
```text

### Continuous Integration

The examples are automatically tested in CI with:
- Multiple ESP-IDF versions
- Debug and Release builds
- Hardware-in-the-loop testing

## 🤝 Contributing

1. **Adding New Examples**
   - Create source file in `main/`
   - Add entry to `app_config.yml`
   - Create documentation in `docs/`
   - Test with both Debug and Release builds

2. **Modifying HAL**
   - Update `ESP32C6_HAL.hpp` and `ESP32C6_HAL.cpp`
   - Ensure compatibility with all examples
   - Test SPI communication thoroughly

3. **Configuration Changes**
   - Update `app_config.yml` for new settings
   - Modify `sdkconfig` for ESP32-C6 specific changes
   - Document changes in README

## 📄 License

This software is released into the **Public Domain**. You can use, modify, and distribute
it freely without any restrictions.

## 🔗 Resources

- [TLE92466ED Datasheet](../../Datasheet/)
- [ESP32-C6 Technical Reference](https://www.espressif.com/sites/default/files/documentation/esp32-c6_technical_reference_manual_en.pdf)
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [GitHub Repository](https://github.com/n3b3x/hf-tle92466ed-driver)

---

**Version**: 2.0.0 | **Target**: ESP32-C6 | **ESP-IDF**: v5.5 | **Last Updated**: 2025-10-21
