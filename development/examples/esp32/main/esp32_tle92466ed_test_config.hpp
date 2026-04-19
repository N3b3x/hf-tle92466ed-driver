/**
 * @file tle92466ed_test_config.hpp
 * @brief Hardware configuration for TLE92466ED driver examples (multi-target).
 *
 * Per-target pin / SPI-host selection lives behind `CONFIG_IDF_TARGET_*`
 * guards so the same example sources build cleanly for either:
 *
 *   * ESP32-C6  — Infineon TLE92466ED dev kit on a standalone C6
 *                 board (default upstream reference).
 *   * ESP32-S3  — HardFOC Flux V1 board, where the TLE92466ED shares
 *                 the SPI3 bus with the MAX22200 and is wired through
 *                 the GPIO matrix on GPIO35-38 / CS=GPIO4.
 *
 * Pick the build target with `idf.py set-target esp32s3` (or
 * `esp32c6`) before building. The host SPI peripheral is also chosen
 * per-target in `esp32_tle92466ed_bus.hpp`.
 *
 * @author N3b3x
 * @date 2025-10-21
 * @version 2.1.0
 */

#pragma once

#include <cstdint>

#include "sdkconfig.h"

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
 * @brief SPI pin assignment for the TLE92466ED.
 *
 * Pins are selected per build target so the driver's example projects
 * can run on multiple boards from the same source tree.
 */
struct SPIPins {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    // ─── HardFOC Flux V1 (ESP32-S3 + SPI3_HOST via GPIO matrix) ───────
    // Shared SPI3 bus — MAX22200 lives on the same MISO/MOSI/SCK trio
    // with its own CS=GPIO38; TLE92466ED CS is GPIO4.
    static constexpr uint8_t MISO = 35;
    static constexpr uint8_t MOSI = 37;
    static constexpr uint8_t SCLK = 36;
    static constexpr uint8_t CS   = 4;
#else  // ESP32-C6 reference dev kit (upstream default)
    static constexpr uint8_t MISO = 2;          ///< GPIO2 - SPI MISO (Master In Slave Out)
    static constexpr uint8_t MOSI = 7;          ///< GPIO7 - SPI MOSI (Master Out Slave In)
    static constexpr uint8_t SCLK = 6;          ///< GPIO6 - SPI Clock
    static constexpr uint8_t CS   = 18;         ///< GPIO18 - Chip Select (active low)
#endif
};

/**
 * @brief Control GPIO Pins for TLE92466ED
 * 
 * These pins control device operation and status monitoring.
 * Set to -1 if not connected/configured.
 */
struct ControlPins {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    // ─── HardFOC Flux V1 ──────────────────────────────────────────────
    // Matches hf_functional_pin_config_flux_v1.hpp.
    static constexpr uint8_t RESN   = 6;        ///< GPIO6  - Reset pin (active LOW)
    static constexpr uint8_t EN     = 5;        ///< GPIO5  - Enable pin (active HIGH, gates output stage)
    static constexpr uint8_t FAULTN = 16;       ///< GPIO16 - FAULTN input (active LOW, open-drain)
    static constexpr uint8_t DRV0   = 7;        ///< GPIO7  - DRV0 external drive control
    static constexpr uint8_t DRV1   = 15;       ///< GPIO15 - DRV1 external drive control
#else  // ESP32-C6 reference dev kit
    static constexpr uint8_t RESN = 21;         ///< GPIO21 - Reset pin (active low, must be HIGH for operation)
    static constexpr uint8_t EN = 20;           ///< GPIO20 - Enable pin (active high, enables outputs)
    static constexpr uint8_t FAULTN = 19;       ///< GPIO19 - Fault pin (active low, indicates fault condition)
    static constexpr uint8_t DRV0 = 22;         ///< GPIO pin for DRV0 external drive control
    static constexpr uint8_t DRV1 = 23;         ///< GPIO pin for DRV1 external drive control
#endif
};

/**
 * @brief ADC Configuration for Solenoid Control Test
 * 
 * ADC pin used for reading control voltage (0-3.3V) to control solenoid currents.
 * This is used in the solenoid control test to map ADC reading to current percentage.
 */
struct ADCConfig {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    static constexpr uint8_t PIN = 1;           ///< GPIO1 (ADC1_CH0 on ESP32-S3) — only used by solenoid_control_test
#else
    static constexpr uint8_t PIN = 0;           ///< GPIO0 - ADC input pin (ADC1_CH0)
#endif
    static constexpr float VREF_MV = 3300.0f;   ///< Reference voltage in millivolts (3.3V)
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
 * VDD: Logic supply (3.3V for ESP32-C6)
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
 * @brief Helper macro for GPIO pin validation (per-target ranges).
 */
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define TLE92466ED_VALIDATE_GPIO(pin) \
    static_assert((pin) >= 0 && (pin) <= 48, "Invalid GPIO pin number for ESP32-S3 (0..48)")
#else
#define TLE92466ED_VALIDATE_GPIO(pin) \
    static_assert((pin) >= 0 && (pin) < 30, "Invalid GPIO pin number for ESP32-C6")
#endif

