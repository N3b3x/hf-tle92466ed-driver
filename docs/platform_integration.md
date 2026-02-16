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

The TLE92466ED driver uses a **polymorphic SPI interface** design for hardware abstraction.
This design choice provides several critical benefits for embedded systems:

### Why This Design?

#### 1. **Hardware Portability**

- Write driver code once, run on any platform
- Easy migration between MCUs (ESP32, STM32, Arduino, etc.)
- Platform-specific code isolated in one class

#### 2. **Type Safety**

- Compile-time interface checking
- Clear contract for required methods
- Catch implementation errors at compile time

#### 3. **Modern C++20**

- Uses `tle::expected` for error handling (aliases to `std::expected` on C++23, polyfill on C++20)
- All functions `noexcept` for embedded safety
- Zero-overhead abstractions

### How It Works

```cpp
// Base interface class (from tle92466ed_spi_interface.hpp)
class SpiInterface {
public:
    virtual auto spiTransfer(std::span<const uint8_t> txData, 
                             std::span<uint8_t> rxData) noexcept 
        -> std::expected<void, CommError> = 0;
    
    virtual void delayMicroseconds(uint32_t us) noexcept = 0;
};

// Your implementation
class MySpi : public SpiInterface {
public:
    auto spiTransfer(std::span<const uint8_t> txData, 
                     std::span<uint8_t> rxData) noexcept 
        -> std::expected<void, CommError> override {
        // Your platform-specific SPI code
        return {};
    }
    
    void delayMicroseconds(uint32_t us) noexcept override {
        // Your delay implementation
    }
};
```cpp

## Interface Definition

The TLE92466ED driver requires you to implement the following interface:

```cpp
class SpiInterface {
public:
    // Required methods (implement all of these)
    virtual auto spiTransfer(std::span<const uint8_t> txData, 
                             std::span<uint8_t> rxData) noexcept 
        -> std::expected<void, CommError> = 0;
    
    virtual void delayMicroseconds(uint32_t us) noexcept = 0;
    
    // Optional control pin methods
    virtual auto GpioSet(CtrlPin pin, GpioSignal signal) noexcept 
        -> std::expected<void, CommError>;
    
    virtual auto GpioRead(CtrlPin pin) noexcept 
        -> std::expected<GpioSignal, CommError>;
};
```cpp

## Implementation Steps

### Step 1: Create Your Implementation Class

```cpp
#include "tle92466ed_spi_interface.hpp"

class MyPlatformSPI : public tle92466ed::SpiInterface<MyPlatformSPI> {
private:
    // Your platform-specific members
    spi_device_handle_t spi_device_;  // Example for ESP32
    
public:
    // Constructor
    MyPlatformSPI(spi_device_handle_t device) : spi_device_(device) {}
    
    // Implement required methods
    auto spiTransfer(std::span<const uint8_t> txData, 
                     std::span<uint8_t> rxData) noexcept 
        -> std::expected<void, tle92466ed::CommError> override {
        // Your transfer code - must handle 32-bit frames
        // TLE92466ED uses 32-bit SPI frames with CRC-8
        return {};
    }
    
    void delayMicroseconds(uint32_t us) noexcept override {
        // Your delay implementation
    }
};
```cpp

### Step 2: Platform-Specific Examples

#### ESP32 (ESP-IDF)

```cpp
#include "driver/spi_master.h"
#include "tle92466ed_spi_interface.hpp"

class Esp32SPIBus : public tle92466ed::SpiInterface<Esp32SPIBus> {
private:
    spi_device_handle_t spi_device_;
    gpio_num_t cs_pin_;
    
public:
    Esp32SPIBus(spi_host_device_t host, const spi_device_interface_config_t& config, 
                gpio_num_t cs) {
        spi_bus_add_device(host, &config, &spi_device_);
        cs_pin_ = cs;
    }
    
    auto spiTransfer(std::span<const uint8_t> txData, 
                     std::span<uint8_t> rxData) noexcept 
        -> std::expected<void, tle92466ed::CommError> override {
        // TLE92466ED uses 32-bit frames
        if (txData.size() < 4 || rxData.size() < 4) {
            return tle::unexpected(tle92466ed::CommError::InvalidParameter);
        }
        
        spi_transaction_t trans = {};
        trans.length = 32;  // 32 bits per frame
        trans.tx_buffer = txData.data();
        trans.rx_buffer = rxData.data();
        
        esp_err_t ret = spi_device_transmit(spi_device_, &trans);
        if (ret != ESP_OK) {
            return tle::unexpected(tle92466ed::CommError::BusError);
        }
        
        return {};
    }
    
    void delayMicroseconds(uint32_t us) noexcept override {
        esp_rom_delay_us(us);
    }
};
```cpp

#### STM32 (HAL)

```cpp
#include "stm32f4xx_hal.h"
#include "tle92466ed_spi_interface.hpp"

extern SPI_HandleTypeDef hspi1;

class STM32SPIBus : public tle92466ed::SpiInterface<STM32SPIBus> {
private:
    GPIO_TypeDef* cs_port_;
    uint16_t cs_pin_;
    
public:
    STM32SPIBus(GPIO_TypeDef* cs_port, uint16_t cs_pin) 
        : cs_port_(cs_port), cs_pin_(cs_pin) {}
    
    auto spiTransfer(std::span<const uint8_t> txData, 
                     std::span<uint8_t> rxData) noexcept 
        -> std::expected<void, tle92466ed::CommError> override {
        // Assert CS
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
        
        // Transfer 32-bit frame (4 bytes)
        HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, 
            const_cast<uint8_t*>(txData.data()), 
            rxData.data(), 
            4, 
            100);
        
        // Deassert CS
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
        
        if (status != HAL_OK) {
            return tle::unexpected(tle92466ed::CommError::BusError);
        }
        
        return {};
    }
    
    void delayMicroseconds(uint32_t us) noexcept override {
        // STM32 HAL delay (approximate)
        HAL_Delay((us + 999) / 1000);  // Convert to milliseconds
    }
};
```cpp

#### Arduino

```cpp
#include <SPI.h>
#include "tle92466ed_spi_interface.hpp"

class ArduinoSPIBus : public tle92466ed::SpiInterface<ArduinoSPIBus> {
private:
    uint8_t cs_pin_;
    
public:
    ArduinoSPIBus(uint8_t cs_pin) : cs_pin_(cs_pin) {
        pinMode(cs_pin_, OUTPUT);
        digitalWrite(cs_pin_, HIGH);
        SPI.begin();
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    }
    
    auto spiTransfer(std::span<const uint8_t> txData, 
                     std::span<uint8_t> rxData) noexcept 
        -> std::expected<void, tle92466ed::CommError> override {
        digitalWrite(cs_pin_, LOW);
        
        // Transfer 32-bit frame (4 bytes)
        for (size_t i = 0; i < 4; i++) {
            rxData[i] = SPI.transfer(txData[i]);
        }
        
        digitalWrite(cs_pin_, HIGH);
        return {};
    }
    
    void delayMicroseconds(uint32_t us) noexcept override {
        delayMicroseconds(us);
    }
};
```cpp

## SPI Frame Format

The TLE92466ED uses **32-bit SPI frames** with CRC-8 (SAE J1850):

### Write Frame Format

```cpp
Bits 31-24 | Bits 23-17 | Bit 16 | Bits 15-0
-----------+------------+--------+-----------
CRC (8-bit)| Address(7) |  R/W   | Data (16)
           |            |  1=W   |
```cpp

### Read Frame Format

```cpp
Bits 31-24 | Bits 23-17 | Bit 16 | Bits 15-0
-----------+------------+--------+-----------
CRC (8-bit)| Don't Care |  R/W   | Address (16-bit)
           |            |  0=R   |
```text

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

All methods return `std::expected<T, CommError>`. Handle errors like this:

```cpp
if (auto result = spi_interface.spiTransfer(tx, rx); !result) {
    // Handle error
    switch (result.error()) {
        case tle92466ed::CommError::BusError:
            // SPI bus error
            break;
        case tle92466ed::CommError::Timeout:
            // Operation timeout
            break;
        default:
            // Other errors
            break;
    }
}
```cpp

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
