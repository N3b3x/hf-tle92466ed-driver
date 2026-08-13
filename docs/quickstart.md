---
layout: default
title: "⚡ Quick Start"
description: "Get up and running with the TLE92466ED driver in minutes"
nav_order: 2
parent: "📚 Documentation"
permalink: /docs/quickstart/
---

# Quick Start

This guide will get you up and running with the TLE92466ED driver in just a few steps.

## Prerequisites

- [Driver installed](installation.md)
- [Hardware wired](hardware_setup.md)
- [SPI interface implemented](platform_integration.md)

## Minimal Example

Here's a complete working example:

```cpp
#include "tle92466ed.hpp"

// 1. Implement the SPI interface (see platform_integration.md)
class MySpi : public tle92466ed::SpiInterface<MySpi> {
public:
    tle92466ed::CommResult<uint32_t> Transfer32(uint32_t tx_data) noexcept {
        // One 32-bit full-duplex frame; manage CS here
        return rx_word;
    }
    tle92466ed::CommResult<void> TransferMulti(
        std::span<const uint32_t> tx,
        std::span<uint32_t> rx) noexcept {
        // Keep CS asserted for all words (required for pipelined reads)
        return {};
    }
    // ... Init, Delay, Configure, GpioSet, GpioRead, Log, etc.
};

// 2. Create instances
MySpi spi;
tle92466ed::Driver driver(spi);

// 3. Initialize
if (auto result = driver.Init(); !result) {
    // Handle error
    return;
}

// 4. Enter mission mode
driver.EnterMissionMode();

// 5. Configure channel 0 for 1.5A current control
driver.SetChannelMode(tle92466ed::Channel::CH0, tle92466ed::ChannelMode::ICC);
driver.SetCurrentSetpoint(tle92466ed::Channel::CH0, 1500); // 1500 mA

// 6. Enable channel
driver.EnableChannel(tle92466ed::Channel::CH0, true);
```cpp

## Step-by-Step Explanation

### Step 1: Include the Header

```cpp
#include "tle92466ed.hpp"
```cpp

This includes the main driver class and all necessary types.

### Step 2: Implement the SPI Interface

You need to implement the `SpiInterface` for your platform. See [Platform Integration](platform_integration.md) for detailed examples.

```cpp
class MySpi : public tle92466ed::SpiInterface<MySpi> {
public:
    tle92466ed::CommResult<uint32_t> Transfer32(uint32_t tx_data) noexcept {
        return rx_word;
    }
    tle92466ed::CommResult<void> TransferMulti(
        std::span<const uint32_t> tx,
        std::span<uint32_t> rx) noexcept {
        return {};
    }
    // ... remaining required methods
};
```

### Step 3: Create Driver Instance

```cpp
MySpi spi;
tle92466ed::Driver driver(spi);
```cpp

The constructor takes a reference to your SPI interface implementation.

### Step 4: Initialize

```cpp
if (auto result = driver.Init(); !result) {
    // Handle error - result.error() contains DriverError
    return;
}
```cpp

### Step 5: Enter Mission Mode

```cpp
driver.EnterMissionMode();
```text

Mission mode enables channel control. Configuration changes require Config Mode.

### Step 6: Configure Channel

```cpp
driver.SetChannelMode(tle92466ed::Channel::CH0, tle92466ed::ChannelMode::ICC);
driver.SetCurrentSetpoint(tle92466ed::Channel::CH0, 1500); // 1500 mA
```cpp

### Step 7: Enable Channel

```cpp
driver.EnableChannel(tle92466ed::Channel::CH0, true);
```cpp

## Complete Example with Error Handling

```cpp
#include "tle92466ed.hpp"

class MySpi : public tle92466ed::SpiInterface<MySpi> {
    // ... SPI implementation
};

void app_main() {
    MySpi spi;
    tle92466ed::Driver driver(spi);
    
    // Initialize
    if (auto result = driver.Init(); !result) {
        printf("Initialization failed: %d\n", static_cast<int>(result.error()));
        return;
    }
    
    // Enter mission mode
    if (auto result = driver.EnterMissionMode(); !result) {
        printf("Failed to enter mission mode\n");
        return;
    }
    
    // Configure channel 0
    if (auto result = driver.SetChannelMode(tle92466ed::Channel::CH0, 
                                            tle92466ed::ChannelMode::ICC); !result) {
        printf("Failed to set channel mode\n");
        return;
    }
    
    if (auto result = driver.SetCurrentSetpoint(tle92466ed::Channel::CH0, 1500); !result) {
        printf("Failed to set current\n");
        return;
    }
    
    // Enable channel
    driver.EnableChannel(tle92466ed::Channel::CH0, true);
    
    // Read diagnostics
    if (auto diag = driver.GetChannelDiagnostics(tle92466ed::Channel::CH0); diag) {
        printf("Channel 0 current: %u mA\n", diag->average_current);
    }
}
```cpp

## Expected Output

When running this example, you should see:

```cpp
Channel 0 current: 1500 mA
```cpp

## Troubleshooting

If you encounter issues:

- **Compilation errors**: Check that you've implemented all required `SpiInterface` methods (`Transfer32`, `TransferMulti`, `Init`, etc.)
- **Initialization fails**: Verify SPI connections and hardware setup
- **Channel not working**: Check channel configuration and enable state
- **See**: [Troubleshooting](troubleshooting.md) for common issues

## Next Steps

- Explore [Examples](examples.md) for more advanced usage
- Review the [API Reference](api_reference.md) for all available methods
- Check [Configuration](configuration.md) for customization options

---

**Navigation**
⬅️ [Installation](installation.md) | [Next: Hardware Setup ➡️](hardware_setup.md) | [Back to Index](index.md)
