---
layout: default
title: "🔧 Platform Integration"
description: "How to implement the SPI interface for your platform"
nav_order: 4
parent: "📚 Documentation"
permalink: /docs/platform_integration/
---

# Platform Integration Guide

This guide explains how to implement the hardware abstraction interface for the TLE92466ED driver on your platform.

## Understanding the SPI Interface

The TLE92466ED driver uses a **CRTP SPI interface** (`SpiInterface<Derived>`) for
hardware abstraction. Your platform adapter inherits from the template with itself
as the type parameter — there is no virtual dispatch overhead.

### Why This Design?

#### 1. **Hardware Portability**

- Write driver code once, run on any platform
- Easy migration between MCUs (ESP32, STM32, Arduino, etc.)
- Platform-specific code isolated in one adapter class

#### 2. **Type Safety**

- Compile-time interface checking via CRTP and (optionally) `SpiInterfaceLike`
- Clear contract for required methods
- Catch implementation errors at compile time

#### 3. **Modern C++20**

- Uses `tle::expected` for error handling (aliases to `std::expected` on C++23, polyfill on C++20)
- All functions `noexcept` for embedded safety
- Zero-overhead abstractions

### Pipelined Register Reads (Critical)

The TLE92466ED SPI protocol is **pipelined**: a read command's reply appears on MISO
during the *next* chip-select window, not the command frame itself. Every register
read therefore costs **two 32-bit frames**:

1. **Command frame** — sends the read address (MISO is not yet the requested data)
2. **Dummy frame** — clocks out the latched reply from frame 1

The driver's `SpiInterface::Read()` and `Write()` helpers send both frames through
`TransferMulti()` in **one CS assertion**. Your adapter must:

- Keep CS asserted for the entire two-frame sequence
- **Not** release bus ownership between frames on a shared multi-slave SPI bus

If another slave's transaction lands between the command and dummy frames, the pipeline
slot is consumed by the wrong device. Symptoms include all-zero reads, wrong register
values, and grossly inflated feedback currents (a near-zero `TP_MANT` divisor once
produced ~2000 mA on a ~115 mA channel).

Implement `TransferMulti()` so consecutive words share one CS window. Use
`Transfer32()` only for single-frame cases outside the driver's register helpers.

### How It Works

```cpp
#include "tle92466ed_spi_interface.hpp"

class MySpi : public tle92466ed::SpiInterface<MySpi> {
public:
    tle92466ed::CommResult<void> Init() noexcept { /* SPI + GPIO init */ return {}; }
    tle92466ed::CommResult<void> Deinit() noexcept { return {}; }

    tle92466ed::CommResult<uint32_t> Transfer32(uint32_t tx) noexcept {
        // Single 32-bit full-duplex transfer with CS managed here
        return rx_word;
    }

    tle92466ed::CommResult<void> TransferMulti(
        std::span<const uint32_t> tx,
        std::span<uint32_t> rx) noexcept {
        // CS LOW → Transfer32 for each word → CS HIGH
        // Required: both frames of a pipelined read stay in this window
        return {};
    }

    tle92466ed::CommResult<void> Delay(uint32_t us) noexcept { return {}; }
    tle92466ed::CommResult<void> Configure(const tle92466ed::SPIConfig& cfg) noexcept { return {}; }
    bool IsReady() const noexcept { return true; }
    tle92466ed::CommError GetLastError() const noexcept { return tle92466ed::CommError::None; }
    tle92466ed::CommResult<void> ClearErrors() noexcept { return {}; }
    tle92466ed::CommResult<void> GpioSet(tle92466ed::CtrlPin pin,
                                         tle92466ed::GpioSignal sig) noexcept { return {}; }
    tle92466ed::CommResult<tle92466ed::GpioSignal> GpioRead(tle92466ed::CtrlPin pin) noexcept {
        return tle92466ed::GpioSignal::INACTIVE;
    }
    void Log(tle92466ed::LogLevel level, const char* tag,
             const char* fmt, va_list args) noexcept {}
};
```

## Interface Definition

Implement every method below on your `SpiInterface<Derived>` adapter:

| Method | Purpose |
|--------|---------|
| `Init()` / `Deinit()` | Initialize and release SPI + GPIO |
| `Transfer32()` | One full-duplex 32-bit frame (CS managed by adapter) |
| `TransferMulti()` | **Required:** consecutive frames in one CS window (pipelined reads) |
| `Delay()` | Microsecond delay |
| `Configure()` | Apply `SPIConfig` (Mode 1, up to 10 MHz) |
| `IsReady()` | True when hardware is initialized |
| `GetLastError()` / `ClearErrors()` | Error state |
| `GpioSet()` / `GpioRead()` | RESN, EN, FAULTN control |
| `Log()` | Optional printf-style logging hook |

## Implementation Steps

### Step 1: Create Your Implementation Class

```cpp
#include "tle92466ed_spi_interface.hpp"

class MyPlatformSPI : public tle92466ed::SpiInterface<MyPlatformSPI> {
public:
    tle92466ed::CommResult<uint32_t> Transfer32(uint32_t tx_data) noexcept {
        // Assert CS, clock 32 bits full-duplex, deassert CS
        return rx_word;
    }

    tle92466ed::CommResult<void> TransferMulti(
        std::span<const uint32_t> tx_data,
        std::span<uint32_t> rx_data) noexcept {
        if (tx_data.size() != rx_data.size()) {
            return tle::unexpected(tle92466ed::CommError::InvalidParameter);
        }
        // CS LOW — transfer each word without releasing CS — CS HIGH
        return {};
    }

    // ... Init, Delay, Configure, GpioSet, GpioRead, Log, etc.
};

### Step 2: Platform-Specific Examples

#### ESP32 (ESP-IDF)

```cpp
#include "driver/spi_master.h"
#include "tle92466ed_spi_interface.hpp"

class Esp32SPIBus : public tle92466ed::SpiInterface<Esp32SPIBus> {
private:
    spi_device_handle_t spi_device_;

public:
    Esp32SPIBus(spi_host_device_t host, const spi_device_interface_config_t& config) {
        spi_bus_add_device(host, &config, &spi_device_);
    }

    tle92466ed::CommResult<uint32_t> Transfer32(uint32_t tx_data) noexcept {
        spi_transaction_t trans = {};
        trans.length = 32;
        trans.tx_data = &tx_data;
        uint32_t rx_data = 0;
        trans.rx_data = &rx_data;
        if (spi_device_transmit(spi_device_, &trans) != ESP_OK) {
            return tle::unexpected(tle92466ed::CommError::BusError);
        }
        return rx_data;
    }

    tle92466ed::CommResult<void> TransferMulti(
        std::span<const uint32_t> tx_data,
        std::span<uint32_t> rx_data) noexcept {
        if (tx_data.size() != rx_data.size()) {
            return tle::unexpected(tle92466ed::CommError::InvalidParameter);
        }
        for (size_t i = 0; i < tx_data.size(); ++i) {
            auto r = Transfer32(tx_data[i]);
            if (!r) return tle::unexpected(r.error());
            rx_data[i] = *r;
        }
        return {};
    }

    tle92466ed::CommResult<void> Delay(uint32_t us) noexcept {
        esp_rom_delay_us(us);
        return {};
    }
    // ... remaining SpiInterface methods
};

#### STM32 (HAL)

```cpp
#include "stm32f4xx_hal.h"
#include "tle92466ed_spi_interface.hpp"

extern SPI_HandleTypeDef hspi1;

class STM32SPIBus : public tle92466ed::SpiInterface<STM32SPIBus> {
public:
    tle92466ed::CommResult<uint32_t> Transfer32(uint32_t tx_data) noexcept {
        uint32_t rx_data = 0;
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
            &hspi1, reinterpret_cast<uint8_t*>(&tx_data),
            reinterpret_cast<uint8_t*>(&rx_data), 4, 100);
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
        if (status != HAL_OK) {
            return tle::unexpected(tle92466ed::CommError::BusError);
        }
        return rx_data;
    }

    tle92466ed::CommResult<void> TransferMulti(
        std::span<const uint32_t> tx_data,
        std::span<uint32_t> rx_data) noexcept {
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
        for (size_t i = 0; i < tx_data.size(); ++i) {
            uint32_t rx_word = 0;
            HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
                &hspi1, reinterpret_cast<uint8_t*>(const_cast<uint32_t*>(&tx_data[i])),
                reinterpret_cast<uint8_t*>(&rx_word), 4, 100);
            if (status != HAL_OK) {
                HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
                return tle::unexpected(tle92466ed::CommError::BusError);
            }
            rx_data[i] = rx_word;
        }
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
        return {};
    }
    // ... remaining SpiInterface methods
};

#### Arduino

```cpp
#include <SPI.h>
#include "tle92466ed_spi_interface.hpp"

class ArduinoSPIBus : public tle92466ed::SpiInterface<ArduinoSPIBus> {
public:
    explicit ArduinoSPIBus(uint8_t cs_pin) : cs_pin_(cs_pin) {
        pinMode(cs_pin_, OUTPUT);
        digitalWrite(cs_pin_, HIGH);
        SPI.begin();
    }

    tle92466ed::CommResult<uint32_t> Transfer32(uint32_t tx_data) noexcept {
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
        digitalWrite(cs_pin_, LOW);
        uint32_t rx_data = 0;
        auto* tx_bytes = reinterpret_cast<uint8_t*>(&tx_data);
        auto* rx_bytes = reinterpret_cast<uint8_t*>(&rx_data);
        for (int i = 0; i < 4; ++i) {
            rx_bytes[i] = SPI.transfer(tx_bytes[i]);
        }
        digitalWrite(cs_pin_, HIGH);
        SPI.endTransaction();
        return rx_data;
    }

    tle92466ed::CommResult<void> TransferMulti(
        std::span<const uint32_t> tx_data,
        std::span<uint32_t> rx_data) noexcept {
        if (tx_data.size() != rx_data.size()) {
            return tle::unexpected(tle92466ed::CommError::InvalidParameter);
        }
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
        digitalWrite(cs_pin_, LOW);
        for (size_t i = 0; i < tx_data.size(); ++i) {
            uint32_t rx_word = 0;
            auto* tx_bytes = reinterpret_cast<const uint8_t*>(&tx_data[i]);
            auto* rx_bytes = reinterpret_cast<uint8_t*>(&rx_word);
            for (int b = 0; b < 4; ++b) {
                rx_bytes[b] = SPI.transfer(tx_bytes[b]);
            }
            rx_data[i] = rx_word;
        }
        digitalWrite(cs_pin_, HIGH);
        SPI.endTransaction();
        return {};
    }
    // ... remaining SpiInterface methods
};

## SPI Frame Format

The TLE92466ED uses **32-bit SPI frames** with CRC-8 (SAE J1850):

### Write Frame Format

```cpp
Bits 31-24 | Bits 23-17 | Bit 16 | Bits 15-0
-----------+------------+--------+-----------
CRC (8-bit)| Address(7) |  R/W   | Data (16)
           |            |  1=W   |
```

### Read Frame Format

```text
Bits 31-24 | Bits 23-17 | Bit 16 | Bits 15-0
-----------+------------+--------+-----------
CRC (8-bit)| Don't Care |  R/W   | Address (16-bit)
           |            |  0=R   |
```

### Read / Write Pipeline

Register access always uses **two consecutive 32-bit frames** per transaction:

```text
CS ──┐                              ┌──
     └─ Frame1 (command) ─ Frame2 (dummy/reply) ─┘
              ↑                      ↑
         MOSI: read addr         MOSI: don't care
         MISO: not valid yet      MISO: latched reply
```

`TransferMulti()` must keep CS low across both frames. On a shared bus, do not call
`Transfer32()` twice with CS toggling between them for driver register reads — use
`TransferMulti()` or equivalent.

### SPI Configuration

- **Mode**: SPI Mode 1 (CPOL=0, CPHA=1)
- **Speed**: Up to 10 MHz
- **Bit Order**: MSB first
- **CS Polarity**: Active low
- **Frame Size**: 32 bits (4 bytes)

## Control Pins (Optional)

The TLE92466ED has optional control pins that can be implemented:

- **RESN**: Active-low reset pin
- **EN**: Active-high enable pin
- **FAULTN**: Active-low fault output (open drain)

```cpp
auto GpioSet(tle92466ed::CtrlPin pin, tle92466ed::GpioSignal signal) noexcept 
    -> std::expected<void, tle92466ed::CommError> override {
    switch (pin) {
        case tle92466ed::CtrlPin::RESN:
            // Set RESN pin (active low)
            gpio_set_level(resn_pin_, signal == tle92466ed::GpioSignal::ACTIVE ? 0 : 1);
            break;
        case tle92466ed::CtrlPin::EN:
            // Set EN pin (active high)
            gpio_set_level(en_pin_, signal == tle92466ed::GpioSignal::ACTIVE ? 1 : 0);
            break;
        default:
            return tle::unexpected(tle92466ed::CommError::InvalidParameter);
    }
    return {};
}
```cpp

## Error Handling

All methods return `tle::expected<T, CommError>`. Handle errors like this:

```cpp
if (auto result = spi.TransferMulti(tx_span, rx_span); !result) {
    switch (result.error()) {
        case tle92466ed::CommError::BusError:
            break;
        case tle92466ed::CommError::Timeout:
            break;
        default:
            break;
    }
}
```

## Testing Your Implementation

1. Create a simple test that transfers a known frame
2. Verify CRC calculation matches expected values
3. Test error conditions (timeout, bus error, etc.)
4. Verify timing requirements are met

## Next Steps

- Review the [API Reference](api_reference.md) for driver methods
- Check [Examples](examples.md) for complete usage examples
- See [Troubleshooting](troubleshooting.md) for common issues

---

**Navigation**
⬅️ [Hardware Setup](hardware_setup.md) | [Next: Configuration ➡️](configuration.md) | [Back to Index](index.md)
