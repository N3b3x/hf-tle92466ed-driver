---
layout: default
title: "🛠️ Installation"
description: "Installation and integration instructions for the TLE92466ED driver"
nav_order: 1
parent: "📚 Documentation"
permalink: /docs/installation/
---

# Installation

This guide covers how to obtain and integrate the TLE92466ED driver library into your project.

## Prerequisites

Before installing the driver, ensure you have:

- **C++20 Compiler**: GCC 10+, Clang 11+, or MSVC 2022+ (C++23 `std::expected` is used automatically if available)
- **Build System**: Make, CMake, or ESP-IDF (depending on your platform)
- **Platform SDK**: ESP-IDF, STM32 HAL, Arduino, or your platform's SPI driver

## Obtaining the Source

### Option 1: Git Clone

```bash
git clone https://github.com/n3b3x/hf-tle92466ed-driver.git
cd hf-tle92466ed-driver
```text

### Option 2: Copy Files

Copy the following files into your project:

```cpp
inc/
  ├── tle92466ed.hpp
  ├── tle92466ed_spi_interface.hpp
  ├── tle92466ed_registers.hpp
  └── tle92466ed_expected.hpp
src/
  └── tle92466ed.cpp
```cpp

## Integration Methods

### Using CMake

Add the driver as a subdirectory in your `CMakeLists.txt`:

```cmake
add_subdirectory(external/hf-tle92466ed-driver)
target_link_libraries(your_target PRIVATE hf_tle92466ed)
target_include_directories(your_target PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/external/hf-tle92466ed-driver/inc
)
```cpp

### Using ESP-IDF Component

The driver can be used as an ESP-IDF component. Add it to your `components` directory:

```cmake
# In your main CMakeLists.txt
idf_component_register(
    SRCS "your_code.cpp"
    INCLUDE_DIRS "."
    REQUIRES hf_tle92466ed
)
```cpp

### Manual Integration

1. Copy the driver files to your project
2. Add the `inc/` directory to your include path
3. Include the header:

   ```cpp
   #include "tle92466ed.hpp"
```cpp

4. Compile with C++20 support:

   ```bash
   g++ -std=c++20 -I inc/ your_code.cpp src/tle92466ed.cpp
```cpp

## Verification

To verify the installation:

1. Include the header in a test file:

   ```cpp
   #include "tle92466ed.hpp"
```cpp

2. Compile a simple test:

   ```bash
   g++ -std=c++20 -I inc/ -c src/tle92466ed.cpp -o test.o
```cpp

3. If compilation succeeds, the library is properly installed.

## Next Steps

- Follow the [Quick Start](quickstart.md) guide to create your first application
- Review [Hardware Setup](hardware_setup.md) for wiring instructions
- Check [Platform Integration](platform_integration.md) to implement the SPI interface

---

**Navigation**
⬅️ [Back to Index](index.md) | [Next: Quick Start ➡️](quickstart.md)
