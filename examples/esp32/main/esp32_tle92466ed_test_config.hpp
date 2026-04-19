/**
 * @file tle92466ed_test_config.hpp
 * @brief Hardware configuration for the TLE92466ED driver examples on ESP32-S3.
 *
 * @details
 *   The example projects in this folder target an ESP32-S3 wired as follows:
 *
 *     SPI3_HOST  (GPIO matrix; SPI3 has no IOMUX path on ESP32-S3)
 *       MISO  = GPIO35
 *       MOSI  = GPIO37
 *       SCLK  = GPIO36
 *       CS    = GPIO4    (active-LOW; routed through GPIO matrix)
 *
 *     TLE92466ED control pins
 *       RESN   = GPIO6   (active-LOW: LOW = device in reset, HIGH = released)
 *       EN     = GPIO5   (active-HIGH: enables the output power stage)
 *       FAULTN = GPIO16  (active-LOW input, open-drain — internal pull-up)
 *       DRV0   = GPIO7   (external drive control; not used by every test)
 *       DRV1   = GPIO15  (external drive control; not used by every test)
 *
 *     ADC reference (only used by `solenoid_control_test`)
 *       PIN   = GPIO1    (ADC1_CH0 on ESP32-S3)
 *
 *   These pins are all in `[0..48]` and have no strapping role on ESP32-S3,
 *   so any custom board with the same wiring will work without changes.
 *   To re-target a different ESP32-S3 board, edit the `SPIPins` / `ControlPins`
 *   constants below.
 *
 * @author N3b3x
 * @date 2025-10-21
 * @version 3.0.0
 */

#pragma once

#include <cstdint>

#include "sdkconfig.h"

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#  error "This example targets ESP32-S3 only. Run `idf.py set-target esp32s3` before building."
#endif

//==============================================================================
// COMPILE-TIME CONFIGURATION FLAGS
//==============================================================================

/**
 * @brief Enable detailed SPI transaction logging
 *
 * @details
 * When enabled (set to 1), the Esp32Tle92466edSpiBus will log detailed
 * information about each SPI transaction including:
 * - TX/RX frame bytes (both little-endian and MSB-first formats)
 * - CRC calculation and verification
 * - Frame field parsing (address, data, status, reply mode, etc.)
 *
 * When disabled (set to 0), only basic error logging is performed.
 *
 * Default: 0 (disabled) - Set to 1 to enable for debugging
 */
#ifndef ESP32_TLE_COMM_ENABLE_DETAILED_SPI_LOGGING
#define ESP32_TLE_COMM_ENABLE_DETAILED_SPI_LOGGING 0
#endif

namespace TLE92466ED_TestConfig {

/**
 * @brief SPI bus pin assignment (ESP32-S3 / SPI3_HOST via GPIO matrix).
 *
 * The trio MISO/MOSI/SCLK can be shared with other devices on the same
 * SPI3 bus; each slave gets its own dedicated CS line.
 */
struct SPIPins {
    static constexpr uint8_t MISO = 35;  ///< GPIO35 - SPI3 MISO (slave SDO)
    static constexpr uint8_t MOSI = 37;  ///< GPIO37 - SPI3 MOSI (master to slave)
    static constexpr uint8_t SCLK = 36;  ///< GPIO36 - SPI3 SCK
    static constexpr uint8_t CS   = 4;   ///< GPIO4  - active-LOW chip select
};

/**
 * @brief Control GPIO pins for the TLE92466ED.
 *
 * RESN must be HIGH for the device to be operational. EN gates the
 * output power stage only — SPI register access works in Config Mode
 * with EN LOW.
 */
struct ControlPins {
    static constexpr uint8_t RESN   = 6;   ///< GPIO6  - Reset (active LOW; HIGH = released)
    static constexpr uint8_t EN     = 5;   ///< GPIO5  - Enable (active HIGH; gates output stage)
    static constexpr uint8_t FAULTN = 16;  ///< GPIO16 - FAULTN input (active LOW, open-drain)
    static constexpr uint8_t DRV0   = 7;   ///< GPIO7  - DRV0 external drive control
    static constexpr uint8_t DRV1   = 15;  ///< GPIO15 - DRV1 external drive control
};

/**
 * @brief ADC configuration for the solenoid control test.
 *
 * The solenoid_control_test app reads an analog control voltage and
 * maps it to the per-channel current setpoint percentage.
 */
struct ADCConfig {
    static constexpr uint8_t PIN = 1;           ///< GPIO1 (ADC1_CH0 on ESP32-S3)
    static constexpr float VREF_MV = 3300.0f;   ///< Reference voltage (mV)
    static constexpr float MIN_VOLTAGE = 0.0f;  ///< Minimum input voltage (V)
    static constexpr float MAX_VOLTAGE = 3.3f;  ///< Maximum input voltage (V)
};

/**
 * @brief SPI Communication Parameters
 *
 * The TLE92466ED supports SPI frequencies up to 8MHz.
 * We use 1MHz for reliable communication with standard wiring.
 *
 * CS Timing Requirements (per TLE92466ED datasheet):
 * - tCSS (CS setup): CS must be asserted (LOW) at least 50ns BEFORE first SCK edge
 * - tCSH (CS hold): CS must remain asserted at least 50ns AFTER last SCK edge
 * - tCSI (CS inactive): CS must be deasserted (HIGH) at least 100ns between transactions
 *
 * cs_ena_pretrans and cs_ena_posttrans are in clock cycles.
 * At 1MHz: 1 cycle = 1000ns, so 2 cycles = 2000ns (well above 50ns minimum)
 */
struct SPIParams {
    static constexpr uint32_t FREQUENCY = 1000000;   ///< 1MHz SPI frequency
    static constexpr uint8_t MODE = 1;                ///< SPI Mode 1 (CPOL=0, CPHA=1)
    static constexpr uint8_t QUEUE_SIZE = 1;          ///< Transaction queue size
    static constexpr uint8_t CS_ENA_PRETRANS = 1;     ///< CS asserted N clock cycles before transaction
    static constexpr uint8_t CS_ENA_POSTTRANS = 1;    ///< CS held N clock cycles after transaction
};

/**
 * @brief Current Control Limits (milliamps)
 *
 * These are hardware limits from the TLE92466ED datasheet.
 * Single channel: 0-2000mA
 * Parallel mode: 0-4000mA (channels paired)
 */
struct CurrentLimits {
    static constexpr uint16_t SINGLE_CHANNEL_MIN = 0;      ///< Minimum current (mA)
    static constexpr uint16_t SINGLE_CHANNEL_MAX = 2000;   ///< Maximum single channel current (mA)
    static constexpr uint16_t PARALLEL_CHANNEL_MAX = 4000; ///< Maximum parallel channel current (mA)
    static constexpr uint16_t RESOLUTION = 61;             ///< Current resolution (μA per LSB)
};

/**
 * @brief Supply Voltage Specifications (volts)
 *
 * VBAT: Main power supply for the load outputs
 * VDD: Logic supply (3.3V for ESP32-S3)
 */
struct SupplyVoltage {
    static constexpr float VBAT_MIN = 8.0f;    ///< Minimum VBAT voltage (V)
    static constexpr float VBAT_NOM = 12.0f;   ///< Nominal VBAT voltage (V)
    static constexpr float VBAT_MAX = 28.0f;   ///< Maximum VBAT voltage (V)
    static constexpr float VDD_NOM = 3.3f;     ///< Logic supply voltage (V)
};

/**
 * @brief Temperature Specifications (celsius)
 *
 * Operating temperature range and limits from datasheet.
 */
struct Temperature {
    static constexpr int16_t OPERATING_MIN = -40;   ///< Minimum operating temperature (°C)
    static constexpr int16_t OPERATING_MAX = 150;   ///< Maximum operating temperature (°C)
    static constexpr int16_t JUNCTION_MAX = 150;    ///< Maximum junction temperature (°C)
    static constexpr int16_t WARNING_THRESHOLD = 130; ///< Temperature warning threshold (°C)
};

/**
 * @brief Timing Parameters (microseconds)
 *
 * Timing requirements from the TLE92466ED datasheet.
 */
struct Timing {
    static constexpr uint16_t CS_SETUP_US = 1;          ///< CS setup time before SCLK (μs)
    static constexpr uint16_t CS_HOLD_US = 1;           ///< CS hold time after SCLK (μs)
    static constexpr uint16_t INTER_FRAME_US = 10;      ///< Minimum time between frames (μs)
    static constexpr uint16_t POWER_ON_DELAY_MS = 50;   ///< Power-on initialization delay (ms)
    static constexpr uint16_t RESET_DELAY_MS = 10;      ///< Reset pulse duration (ms)
};

/**
 * @brief Diagnostic Thresholds
 *
 * Thresholds for fault detection and diagnostics.
 */
struct Diagnostics {
    static constexpr uint16_t OVERCURRENT_THRESHOLD_MA = 2100;  ///< Overcurrent fault threshold (mA)
    static constexpr uint16_t POLL_INTERVAL_MS = 100;           ///< Diagnostic polling interval (ms)
    static constexpr uint8_t MAX_RETRY_COUNT = 3;               ///< Maximum communication retries
};

/**
 * @brief Test Configuration
 *
 * Default parameters for testing and calibration.
 */
struct TestConfig {
    static constexpr uint16_t DEFAULT_TEST_CURRENT = 500;       ///< Default test current (mA)
    static constexpr uint16_t TEST_DURATION_MS = 5000;          ///< Test duration (ms)
    static constexpr uint16_t RAMP_STEP_MA = 100;               ///< Current ramp step size (mA)
    static constexpr uint16_t RAMP_STEP_DELAY_MS = 500;         ///< Delay between ramp steps (ms)
};

/**
 * @brief Application-specific Configuration
 *
 * Configuration values that can be adjusted per application.
 */
struct AppConfig {
    // Logging
    static constexpr bool ENABLE_DEBUG_LOGGING = true;     ///< Enable detailed debug logs
    static constexpr bool ENABLE_SPI_LOGGING = false;      ///< Enable SPI transaction logs

    // Performance
    static constexpr bool ENABLE_PERFORMANCE_MONITORING = true;  ///< Enable performance metrics
    static constexpr uint16_t STATS_REPORT_INTERVAL_MS = 10000;  ///< Statistics reporting interval

    // Error handling
    static constexpr bool ENABLE_AUTO_RECOVERY = true;     ///< Enable automatic error recovery
    static constexpr uint8_t MAX_ERROR_COUNT = 10;         ///< Maximum errors before failsafe
};

} // namespace TLE92466ED_TestConfig

/**
 * @brief Hardware configuration validation
 *
 * Compile-time checks to ensure configuration is valid.
 */
static_assert(TLE92466ED_TestConfig::CurrentLimits::SINGLE_CHANNEL_MAX <= 2000,
              "Single channel current exceeds hardware limit of 2000mA");

static_assert(TLE92466ED_TestConfig::CurrentLimits::PARALLEL_CHANNEL_MAX <= 4000,
              "Parallel channel current exceeds hardware limit of 4000mA");

static_assert(TLE92466ED_TestConfig::SPIParams::FREQUENCY <= 8000000,
              "SPI frequency exceeds TLE92466ED maximum of 8MHz");

static_assert(TLE92466ED_TestConfig::SPIParams::MODE == 1,
              "TLE92466ED requires SPI Mode 1 (CPOL=0, CPHA=1)");

/**
 * @brief Helper macro for compile-time configuration validation
 */
#define TLE92466ED_VALIDATE_CURRENT(current_ma) \
    static_assert((current_ma) <= TLE92466ED_TestConfig::CurrentLimits::SINGLE_CHANNEL_MAX, \
                  "Current exceeds maximum limit")

/**
 * @brief Helper macro for GPIO pin validation (ESP32-S3: 0..48 inclusive).
 */
#define TLE92466ED_VALIDATE_GPIO(pin) \
    static_assert((pin) >= 0 && (pin) <= 48, "Invalid GPIO pin number for ESP32-S3 (0..48)")
