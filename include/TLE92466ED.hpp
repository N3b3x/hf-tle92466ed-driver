/**
 * @file TLE92466ED.hpp
 * @brief Main driver class for TLE92466ED Six-Channel Low-Side Solenoid Driver IC
 * @author AI Generated Driver
 * @date 2025-10-20
 * @version 2.0.0
 *
 * @details
 * Comprehensive driver for the Infineon TLE92466ED IC. This driver provides
 * a complete, feature-rich interface to all capabilities of the IC including:
 *
 * **Core Features:**
 * - 6 independent low-side solenoid output channels
 * - Integrated Current Control (ICC) with 15-bit resolution
 * - Current range: 0-2A register scale (1.5A typical continuous) single channel
 *                  0-4A register scale (2.7A typical continuous) parallel mode
 * - 32-bit SPI with CRC-8 (SAE J1850)
 * - Hardware-agnostic design via polymorphic HAL
 *
 * **Current Control Features:**
 * - Precision current regulation (15-bit resolution = 0.061mA steps)
 * - PWM frequency control for current regulation
 * - Integrator-based current controller
 * - Dither support (configurable amplitude and frequency)
 * - Load current feedback and monitoring
 *
 * **Channel Operation Modes:**
 * - ICC Mode: Integrated Current Control with precise regulation
 * - Direct Drive: SPI-controlled on-time
 * - External Drive: DRV0/DRV1 pin-controlled
 * - Free-running Measurement: Continuous current sensing
 *
 * **Parallel Operation:**
 * - Channels can be paired: 0/3, 1/2, 4/5
 * - Doubles current capability (up to 4A)
 *
 * **Protection & Diagnostics:**
 * - Open-load detection (ON and OFF states)
 * - Short-to-ground detection
 * - Over-current protection
 * - Over-temperature (warning and shutdown)
 * - Under/over-voltage monitoring (VBAT, VIO, VDD)
 * - SPI watchdog
 * - Clock watchdog
 * - CRC error detection
 *
 * **Usage Example:**
 * @code{.cpp}
 * MyPlatformHAL hal;
 * TLE92466ED::Driver driver(hal);
 * 
 * // Initialize
 * DriverError error;
 * if (!driver.init(&error)) {
 *     // Handle error
 * }
 * 
 * if (!driver.enter_mission_mode(&error)) {
 *     // Handle error
 * }
 * 
 * // Configure channel 0 for 1.5A current control
 * if (!driver.set_channel_mode(Channel::CH0, ChannelMode::ICC, &error)) {
 *     // Handle error
 * }
 * 
 * if (!driver.set_current_setpoint(Channel::CH0, 1500, &error)) {
 *     // Handle error
 * }
 * 
 * // Enable channel
 * if (!driver.enable_channel(Channel::CH0, true, &error)) {
 *     // Handle error
 * }
 * 
 * // Monitor
 * uint16_t current;
 * if (driver.get_average_current(Channel::CH0, &current, &error)) {
 *     // Use current value
 * }
 * @endcode
 *
 * @copyright
 * This is free and unencumbered software released into the public domain.
 */

#ifndef TLE92466ED_HPP
#define TLE92466ED_HPP

#include "TLE92466ED_HAL.hpp"
#include "TLE92466ED_Registers.hpp"

// Conditional includes based on C++ standard
#if __cpp_lib_expected >= 202202L
#include <expected>
#endif

#include <array>

#if __cpp_lib_chrono >= 201907L
#include <chrono>
#include <thread>
#endif

namespace TLE92466ED {

/**
 * @brief Driver error codes
 */
enum class DriverError : uint8_t {
    None = 0,             ///< No error
    NotInitialized,       ///< Driver not initialized
    HardwareError,        ///< HAL communication error
    InvalidChannel,       ///< Invalid channel number
    InvalidParameter,     ///< Invalid parameter value
    DeviceNotResponding,  ///< Device not responding to SPI
    WrongDeviceID,        ///< Incorrect device ID read
    RegisterError,        ///< Register read/write error
    CRCError,             ///< CRC mismatch in SPI communication
    FaultDetected,        ///< Device fault detected
    ConfigurationError,   ///< Configuration failed
    TimeoutError,         ///< Operation timeout
    WrongMode,            ///< Operation not allowed in current mode
    SPIFrameError,        ///< SPI frame error from device
    WriteToReadOnly       ///< Attempted write to read-only register
};

// Conditional result type based on C++ standard
#if __cpp_lib_expected >= 202202L
/**
 * @brief Driver operation result type (C++23 std::expected)
 */
template<typename T>
using DriverResult = std::expected<T, DriverError>;
#else
/**
 * @brief Driver operation result type (C++11 fallback)
 * 
 * For C++11 compatibility, we use a simple struct with success flag
 * and optional error code. Users should check the success flag before
 * accessing the value.
 */
template<typename T>
struct DriverResult {
    bool success;
    T value;
    DriverError error_code;
    
    DriverResult() : success(false), value(), error_code(DriverError::None) {}
    DriverResult(const T& val) : success(true), value(val), error_code(DriverError::None) {}
    DriverResult(DriverError err) : success(false), value(), error_code(err) {}
    
    // Conversion operators for C++23 compatibility
    operator bool() const noexcept { return success; }
    const T& operator*() const noexcept { return value; }
    T& operator*() noexcept { return value; }
    const T* operator->() const noexcept { return &value; }
    T* operator->() noexcept { return &value; }
    DriverError error() const noexcept { return error_code; }
};
#endif

/**
 * @brief Channel configuration structure
 * 
 * @details
 * Contains all configurable parameters for an output channel.
 */
struct ChannelConfig {
    ChannelMode mode;              ///< Channel operation mode
    uint16_t current_setpoint_ma;  ///< Current setpoint in mA (0-2000 or 0-4000)
    SlewRate slew_rate;            ///< Output slew rate
    DiagCurrent diag_current;      ///< OFF-state diagnostic current
    uint8_t open_load_threshold;   ///< OL threshold (0=disabled, 1-7 = 1/8 to 7/8)
    uint16_t pwm_period_mantissa;  ///< PWM period mantissa
    uint8_t pwm_period_exponent;   ///< PWM period exponent
    bool auto_limit_disabled;      ///< Disable auto-limit feature
    bool olsg_warning_enabled;     ///< Enable OLSG warning
    bool deep_dither_enabled;      ///< Enable deep dither
    uint16_t dither_step_size;     ///< Dither amplitude step size
    uint8_t dither_steps;          ///< Number of dither steps
    uint8_t dither_flat;           ///< Flat period steps
    
    // C++11 constructor
    ChannelConfig() 
        : mode(ChannelMode::ICC)
        , current_setpoint_ma(0)
        , slew_rate(SlewRate::MEDIUM_2V5_US)
        , diag_current(DiagCurrent::I_80UA)
        , open_load_threshold(3)
        , pwm_period_mantissa(0)
        , pwm_period_exponent(0)
        , auto_limit_disabled(false)
        , olsg_warning_enabled(false)
        , deep_dither_enabled(false)
        , dither_step_size(0)
        , dither_steps(0)
        , dither_flat(0) {}
};

/**
 * @brief Global device status structure
 */
struct DeviceStatus {
    bool config_mode;            ///< In config mode (vs mission mode)
    bool init_done;              ///< Initialization complete
    bool any_fault;              ///< Any fault condition present
    
    // Supply voltage faults
    bool vbat_uv;                ///< VBAT undervoltage
    bool vbat_ov;                ///< VBAT overvoltage
    bool vio_uv;                 ///< VIO undervoltage
    bool vio_ov;                 ///< VIO overvoltage
    bool vdd_uv;                 ///< VDD undervoltage
    bool vdd_ov;                 ///< VDD overvoltage
    
    // Temperature
    bool ot_warning;             ///< Over-temperature warning
    bool ot_error;               ///< Over-temperature error
    
    // Other faults
    bool clock_fault;            ///< Clock fault
    bool spi_wd_error;           ///< SPI watchdog error
    bool por_event;              ///< Power-on reset occurred
    bool reset_event;            ///< External reset occurred
    
    // Internal diagnostics
    bool supply_nok_internal;    ///< Internal supply fault
    bool supply_nok_external;    ///< External supply fault
    
    // Voltage readings
    uint16_t vbat_voltage;       ///< VBAT voltage (raw value)
    uint16_t vio_voltage;        ///< VIO voltage (raw value)
    
    // C++11 constructor
    DeviceStatus() 
        : config_mode(true)
        , init_done(false)
        , any_fault(false)
        , vbat_uv(false)
        , vbat_ov(false)
        , vio_uv(false)
        , vio_ov(false)
        , vdd_uv(false)
        , vdd_ov(false)
        , clock_fault(false)
        , ot_warning(false)
        , ot_error(false)
        , por_event(false)
        , reset_event(false)
        , supply_nok_internal(false)
        , supply_nok_external(false)
        , vbat_voltage(0)
        , vio_voltage(0) {}
};

/**
 * @brief Channel diagnostic information
 */
struct ChannelDiagnostics {
    // Error flags
    bool overcurrent;            ///< Over-current detected
    bool short_to_ground;        ///< Short to ground
    bool open_load;              ///< Open load
    bool over_temperature;       ///< Channel over-temperature
    bool open_load_short_ground; ///< Open load or short to ground
    
    // Warning flags
    bool ot_warning;             ///< Over-temperature warning
    bool current_regulation_warning; ///< Current regulation warning
    bool pwm_regulation_warning; ///< PWM regulation warning
    bool olsg_warning;           ///< OLSG warning
    
    // Measurements
    uint16_t average_current;    ///< Average current (raw value)
    uint16_t duty_cycle;         ///< PWM duty cycle (raw value)
    uint16_t min_current;        ///< Minimum current
    uint16_t max_current;        ///< Maximum current
    uint16_t vbat_feedback;      ///< VBAT feedback
    
    // C++11 constructor
    ChannelDiagnostics()
        : overcurrent(false)
        , short_to_ground(false)
        , open_load(false)
        , over_temperature(false)
        , open_load_short_ground(false)
        , ot_warning(false)
        , current_regulation_warning(false)
        , pwm_regulation_warning(false)
        , olsg_warning(false)
        , average_current(0)
        , duty_cycle(0)
        , min_current(0)
        , max_current(0)
        , vbat_feedback(0) {}
};

/**
 * @brief Global configuration structure
 */
struct GlobalConfig {
    bool crc_enabled;            ///< Enable CRC checking
    bool spi_watchdog_enabled;   ///< Enable SPI watchdog
    bool clock_watchdog_enabled; ///< Enable clock watchdog
    bool vio_5v;                 ///< VIO voltage (false=3.3V, true=5.0V)
    uint8_t vbat_uv_threshold;   ///< VBAT UV threshold (0.16208V per LSB)
    uint8_t vbat_ov_threshold;   ///< VBAT OV threshold (0.16208V per LSB)
    uint16_t spi_watchdog_reload;///< SPI watchdog reload value
    
    // C++11 constructor
    GlobalConfig()
        : crc_enabled(true)
        , spi_watchdog_enabled(true)
        , clock_watchdog_enabled(true)
        , vio_5v(false)
        , vbat_uv_threshold(25)
        , vbat_ov_threshold(255)
        , spi_watchdog_reload(1000) {}
};

/**
 * @brief Main TLE92466ED driver class
 * 
 * @details
 * This is the primary driver class that provides complete control over the
 * TLE92466ED IC. It manages SPI communication (32-bit with CRC), configuration,
 * current control, monitoring, and diagnostics for all six output channels.
 *
 * **Thread Safety:**
 * This class is NOT thread-safe by default. External synchronization is required
 * for multi-threaded access.
 *
 * **Resource Management:**
 * Uses RAII principles - init() must be called explicitly but cleanup is automatic.
 *
 * **Error Handling:**
 * Uses conditional error handling based on C++ standard:
 * - C++23: std::expected for robust error handling
 * - C++11: Simple error codes for maximum compatibility
 *
 * @par Initialization Sequence:
 * 1. Construct driver with HAL reference
 * 2. Call init() to initialize hardware and verify device
 * 3. Call enter_mission_mode() to enable channel control
 * 4. Configure channels with set_channel_mode() and configure_channel()
 * 5. Set current with set_current_setpoint()
 * 6. Enable outputs with enable_channel()
 * 7. Monitor with get diagnostics functions
 */
class Driver {
public:
    /**
     * @brief Construct driver with HAL interface
     * 
     * @param hal Reference to hardware abstraction layer
     * 
     * @pre HAL must remain valid for the lifetime of the Driver
     * @post Driver is constructed but not initialized
     */
    explicit Driver(HAL& hal) noexcept : hal_(hal), initialized_(false), mission_mode_(false) {
        channel_enable_cache_ = 0;
        for (size_t i = 0; i < 6; ++i) {
            channel_setpoints_[i] = 0;
        }
    }

    /**
     * @brief Destructor - ensures clean shutdown
     */
    ~Driver() noexcept {
        if (initialized_) {
            // Best effort shutdown - ignore errors
            DriverError error;
            (void)disable_all_channels(&error);
        }
    }

    // Prevent copying
    Driver(const Driver&);
    Driver& operator=(const Driver&);

    // Delete move operations (contains reference member)
    Driver(Driver&&);
    Driver& operator=(Driver&&);

    //==========================================================================
    // INITIALIZATION AND MODE CONTROL
    //==========================================================================

    /**
     * @brief Initialize the driver and hardware
     * 
     * @details
     * Performs complete initialization sequence:
     * 1. Initialize HAL (SPI peripheral)
     * 2. Verify device communication
     * 3. Read and verify device ID
     * 4. Apply default configuration (in Config Mode)
     * 5. Clear any power-on faults
     *
     * After init(), device is in Config Mode. Call enter_mission_mode() to enable outputs.
     *
     * @return DriverResult<void> Success or error code
     * @retval DriverError::HardwareError HAL initialization failed
     * @retval DriverError::DeviceNotResponding No SPI response
     * @retval DriverError::WrongDeviceID Device ID mismatch
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> init() noexcept;

    /**
     * @brief Enter Mission Mode (enables channel control)
     * 
     * @details
     * Transitions from Config Mode to Mission Mode. Channel outputs can only
     * be enabled in Mission Mode. Most configuration registers can only be
     * written in Config Mode.
     *
     * @return DriverResult<void> Success or error code
     * @retval DriverError::NotInitialized Driver not initialized
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> enter_mission_mode() noexcept;

    /**
     * @brief Enter Config Mode (allows configuration changes)
     * 
     * @details
     * Transitions from Mission Mode to Config Mode. All channel outputs
     * are automatically disabled when entering Config Mode.
     *
     * @return DriverResult<void> Success or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> enter_config_mode() noexcept;

    /**
     * @brief Check if in mission mode
     * @return true if in mission mode, false if in config mode
     */
    bool is_mission_mode() const noexcept {
        return mission_mode_;
    }

    //==========================================================================
    // GLOBAL CONFIGURATION
    //==========================================================================

    /**
     * @brief Configure global device settings
     * 
     * @param config Global configuration structure
     * @return DriverResult<void> Success or error code
     * @retval DriverError::NotInitialized Driver not initialized
     * @retval DriverError::WrongMode Must be in Config Mode
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> configure_global(const GlobalConfig& config) noexcept;

    /**
     * @brief Enable/disable CRC checking
     * 
     * @param enabled true to enable CRC
     * @return DriverResult<void> Success or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> set_crc_enabled(bool enabled) noexcept;

    /**
     * @brief Configure VBAT under/overvoltage thresholds
     * 
     * @param uv_threshold UV threshold (V_BAT_UV = value * 0.16208V)
     * @param ov_threshold OV threshold (V_BAT_OV = value * 0.16208V)
     * @return DriverResult<void> Success or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> set_vbat_thresholds(uint8_t uv_threshold, uint8_t ov_threshold) noexcept;

    //==========================================================================
    // CHANNEL CONTROL
    //==========================================================================

    /**
     * @brief Enable or disable a channel
     * 
     * @param channel Channel to control
     * @param enabled true to enable, false to disable
     * @return DriverResult<void> Success or error code
     * @retval DriverError::WrongMode Must be in Mission Mode
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> enable_channel(Channel channel, bool enabled) noexcept;

    /**
     * @brief Enable or disable multiple channels
     * 
     * @param channel_mask Bitmask where bit N enables channel N
     * @return DriverResult<void> Success or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> enable_channels(uint8_t channel_mask) noexcept;

    /**
     * @brief Enable all channels
     * @return DriverResult<void> Success or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> enable_all_channels() noexcept;

    /**
     * @brief Disable all channels
     * @return DriverResult<void> Success or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> disable_all_channels() noexcept;

    /**
     * @brief Set channel operation mode
     * 
     * @param channel Channel to configure
     * @param mode Operation mode (ICC, Direct Drive, etc.)
     * @return DriverResult<void> Success or error code
     * @retval DriverError::WrongMode Must be in Config Mode
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> set_channel_mode(Channel channel, ChannelMode mode) noexcept;

    /**
     * @brief Configure channel for parallel operation
     * 
     * @param pair Parallel pair to configure (0/3, 1/2, or 4/5)
     * @param enabled true to enable parallel operation
     * @return DriverResult<void> Success or error code
     * @retval DriverError::WrongMode Must be in Config Mode
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> set_parallel_operation(ParallelPair pair, bool enabled) noexcept;

    //==========================================================================
    // CURRENT CONTROL (ICC MODE)
    //==========================================================================

    /**
     * @brief Set current setpoint for channel
     * 
     * @param channel Channel to configure
     * @param current_ma Desired current in milliamperes (0-2000 single, 0-4000 parallel)
     * @param parallel_mode Set true if channel is in parallel mode
     * @return DriverResult<void> Success or error code
     * 
     * @note Current is regulated by the Integrated Current Controller (ICC)
     * @note Resolution: 15-bit (0.061mA per LSB in single mode)
     * 
     * @par Current Limits (from datasheet):
     * - **Single channel**: 1.5A typical continuous, 2.0A absolute maximum
     * - **Parallel channels**: 2.7A typical continuous, 4.0A absolute maximum
     * 
     * @warning Setting currents above typical continuous ratings may result in
     *          thermal limiting, reduced accuracy, or current regulation at the
     *          device's natural limit rather than the requested setpoint.
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> set_current_setpoint(Channel channel, uint16_t current_ma, bool parallel_mode = false) noexcept;

    /**
     * @brief Get current setpoint for channel
     * 
     * @param channel Channel to query
     * @param parallel_mode true if channel is in parallel mode
     * @return DriverResult<uint16_t> Current in mA or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<uint16_t> get_current_setpoint(Channel channel, bool parallel_mode = false) noexcept;

    /**
     * @brief Configure PWM parameters for ICC
     * 
     * @param channel Channel to configure
     * @param period_mantissa PWM period mantissa
     * @param period_exponent PWM period exponent (0-7)
     * @param low_freq_range Enable low frequency range (8x multiplier)
     * @return DriverResult<void> Success or error code
     * 
     * @details PWM period: t_PWM = mantissa * 2^exponent * (1/f_sys)
     *          If low_freq_range: t_PWM = mantissa * 8 * 2^exponent * (1/f_sys)
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> configure_pwm_period(Channel channel, uint8_t period_mantissa, uint8_t period_exponent, bool low_freq_range = false) noexcept;

    /**
     * @brief Configure dither parameters
     * 
     * @param channel Channel to configure
     * @param step_size Dither amplitude (I_dither = steps * step_size * 2A / 32767)
     * @param num_steps Number of steps in quarter period
     * @param flat_steps Number of flat clock cycles at top/bottom
     * @return DriverResult<void> Success or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> configure_dither(Channel channel, uint16_t step_size, uint8_t num_steps, uint8_t flat_steps) noexcept;

    /**
     * @brief Configure channel slew rate and diagnostics
     * 
     * @param channel Channel to configure
     * @param config Channel configuration
     * @return DriverResult<void> Success or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> configure_channel(Channel channel, const ChannelConfig& config) noexcept;

    //==========================================================================
    // STATUS AND DIAGNOSTICS
    //==========================================================================

    /**
     * @brief Get global device status
     * 
     * @return DriverResult<DeviceStatus> Device status or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<DeviceStatus> get_device_status() noexcept;

    /**
     * @brief Get channel diagnostic information
     * 
     * @param channel Channel to query
     * @return DriverResult<ChannelDiagnostics> Diagnostics or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<ChannelDiagnostics> get_channel_diagnostics(Channel channel) noexcept;

    /**
     * @brief Get average current for a channel
     * 
     * @param channel Channel to query
     * @param parallel_mode true if in parallel mode
     * @return DriverResult<uint16_t> Average current in mA or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<uint16_t> get_average_current(Channel channel, bool parallel_mode = false) noexcept;

    /**
     * @brief Get PWM duty cycle for a channel
     * 
     * @param channel Channel to query
     * @return DriverResult<uint16_t> Duty cycle (raw 16-bit value) or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<uint16_t> get_duty_cycle(Channel channel) noexcept;

    /**
     * @brief Get VBAT voltage
     * 
     * @return DriverResult<uint16_t> VBAT in millivolts or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<uint16_t> get_vbat_voltage() noexcept;

    /**
     * @brief Get VIO voltage  
     * 
     * @return DriverResult<uint16_t> VIO in millivolts or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<uint16_t> get_vio_voltage() noexcept;

    //==========================================================================
    // FAULT MANAGEMENT
    //==========================================================================

    /**
     * @brief Clear all fault flags
     * 
     * @details
     * Clears latched fault conditions. Does not clear active faults.
     *
     * @return DriverResult<void> Success or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> clear_faults() noexcept;

    /**
     * @brief Check if any fault exists
     * 
     * @return DriverResult<bool> Fault status or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<bool> has_any_fault() noexcept;

    /**
     * @brief Software reset of the device
     * 
     * @details
     * Resets all registers to default values. init() must be called again.
     *
     * @return DriverResult<void> Success or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> software_reset() noexcept;

    //==========================================================================
    // WATCHDOG MANAGEMENT
    //==========================================================================

    /**
     * @brief Reload SPI watchdog counter
     * 
     * @details
     * Must be called periodically when SPI watchdog is enabled to prevent
     * watchdog timeout.
     *
     * @param reload_value Reload value (watchdog period)
     * @return DriverResult<void> Success or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> reload_spi_watchdog(uint16_t reload_value) noexcept;

    //==========================================================================
    // DEVICE INFORMATION
    //==========================================================================

    /**
     * @brief Read IC version and ID
     * 
     * @return DriverResult<uint16_t> ICVID register value or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<uint16_t> get_ic_version() noexcept;

    /**
     * @brief Read unique chip ID
     * 
     * @return DriverResult<std::array<uint16_t, 3>> Three 16-bit ID registers or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<std::array<uint16_t, 3>> get_chip_id() noexcept;

    /**
     * @brief Verify device ID matches expected value
     * 
     * @return DriverResult<bool> Validation result or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<bool> verify_device() noexcept;

    /**
     * @brief Check if driver is initialized
     * 
     * @return true if initialized and ready
     */
    bool is_initialized() const noexcept {
        return initialized_;
    }

    //==========================================================================
    // REGISTER ACCESS (Advanced)
    //==========================================================================

    /**
     * @brief Read 16-bit register
     * 
     * @param address Register address (10-bit)
     * @param verify_crc If true, verify CRC in response
     * @return DriverResult<uint16_t> Register value or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<uint16_t> read_register(uint16_t address, bool verify_crc = true) noexcept;

    /**
     * @brief Write 16-bit register
     * 
     * @param address Register address (10-bit)
     * @param value Value to write (16-bit)
     * @param verify_crc If true, verify CRC in response
     * @return DriverResult<void> Success or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> write_register(uint16_t address, uint16_t value, bool verify_crc = true) noexcept;

    /**
     * @brief Modify register bits
     * 
     * @param address Register address
     * @param mask Bit mask for modification
     * @param value New bit values
     * @return DriverResult<void> Success or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    DriverResult<void> modify_register(uint16_t address, uint16_t mask, uint16_t value) noexcept;

private:
    //==========================================================================
    // PRIVATE METHODS
    //==========================================================================

    /**
     * @brief Transfer SPI frame with CRC calculation and verification
     */
    DriverResult<void> transfer_frame(const SPIFrame& tx_frame, SPIFrame* rx_frame, bool verify_crc = true) noexcept;

    /**
     * @brief Validate channel number
     */
    bool is_valid_channel_internal(Channel channel) const noexcept {
        return is_valid_channel(channel);
    }

    /**
     * @brief Check if driver is initialized
     */
    DriverResult<void> check_initialized() const noexcept {
        if (!initialized_) {
            return DriverError::NotInitialized;
        }
        return DriverResult<void>();
    }

    /**
     * @brief Check if in mission mode
     */
    DriverResult<void> check_mission_mode() const noexcept {
        if (!mission_mode_) {
            return DriverError::WrongMode;
        }
        return DriverResult<void>();
    }

    /**
     * @brief Check if in config mode
     */
    DriverResult<void> check_config_mode() const noexcept {
        if (mission_mode_) {
            return DriverError::WrongMode;
        }
        return DriverResult<void>();
    }

    /**
     * @brief Apply default configuration after initialization
     */
    DriverResult<void> apply_default_config() noexcept;

    /**
     * @brief Parse SPI status from reply frame
     */
    DriverResult<void> check_spi_status(const SPIFrame& rx_frame) noexcept;

    /**
     * @brief Check if channel is currently in parallel mode
     * @param channel Channel to check
     * @return DriverResult<bool> Parallel status or error code
     */
    DriverResult<bool> is_channel_parallel(Channel channel) noexcept;

    //==========================================================================
    // MEMBER VARIABLES
    //==========================================================================

    HAL& hal_;                              ///< Hardware abstraction layer
    bool initialized_;                      ///< Initialization status
    bool mission_mode_;                     ///< Mission mode flag (vs config mode)
    uint16_t channel_enable_cache_;         ///< Cached channel enable state
    std::array<uint16_t, 6> channel_setpoints_; ///< Cached current setpoints
};

} // namespace TLE92466ED

#endif // TLE92466ED_HPP