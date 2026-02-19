/**
 * @file tle92466ed.hpp
 * @brief Main driver class for TLE92466ED Six-Channel Low-Side Solenoid Driver IC
 * @copyright Copyright (c) 2024-2025 HardFOC. All rights reserved.
 */
#pragma once
#include <array>

#include "tle92466ed_expected.hpp"
#include "tle92466ed_spi_interface.hpp"
#include "tle92466ed_registers.hpp"
#include "tle92466ed_version.h"

namespace tle92466ed {

/**
 * @brief Driver error codes
 */
enum class DriverError : uint8_t {
  None = 0,            ///< No error
  NotInitialized,      ///< Driver not initialized
  HardwareError,       ///< Communication interface error
  InvalidChannel,      ///< Invalid channel number
  InvalidParameter,    ///< Invalid parameter value
  DeviceNotResponding, ///< Device not responding to SPI
  WrongDeviceID,       ///< Incorrect device ID read
  RegisterError,       ///< Register read/write error
  CRCError,            ///< CRC mismatch in SPI communication
  FaultDetected,       ///< Device fault detected
  ConfigurationError,  ///< Configuration failed
  TimeoutError,        ///< Operation timeout
  WrongMode,           ///< Operation not allowed in current mode
  SPIFrameError,       ///< SPI frame error from device
  WriteToReadOnly      ///< Attempted write to read-only register
};

/**
 * @brief Driver result type
 */
template <typename T>
using DriverResult = tle::expected<T, DriverError>;

/**
 * @brief Channel configuration structure
 *
 * @details
 * Contains all configurable parameters for an output channel.
 */
struct ChannelConfig {
  ChannelMode mode{ChannelMode::ICC};            ///< Channel operation mode
  uint16_t current_setpoint_ma{0};               ///< Current setpoint in mA (0-2000 or 0-4000)
  SlewRate slew_rate{SlewRate::MEDIUM_2V5_US};   ///< Output slew rate
  DiagCurrent diag_current{DiagCurrent::I_80UA}; ///< OFF-state diagnostic current
  uint8_t open_load_threshold{3};                ///< OL threshold (0=disabled, 1-7 = 1/8 to 7/8)
  uint16_t pwm_period_mantissa{0};               ///< PWM period mantissa
  uint8_t pwm_period_exponent{0};                ///< PWM period exponent
  bool auto_limit_disabled{false};               ///< Disable auto-limit feature
  bool olsg_warning_enabled{false};              ///< Enable OLSG warning
  bool deep_dither_enabled{false};               ///< Enable deep dither
  uint16_t dither_step_size{0};                  ///< Dither amplitude step size
  uint8_t dither_steps{0};                       ///< Number of dither steps
  uint8_t dither_flat{0};                        ///< Flat period steps
};

/**
 * @brief Global device status structure
 */
struct DeviceStatus {
  bool config_mode{true}; ///< In config mode (vs mission mode)
  bool init_done{false};  ///< Initialization complete
  bool any_fault{false};  ///< Any fault condition present

  // Supply voltage faults
  bool vbat_uv{false}; ///< VBAT undervoltage
  bool vbat_ov{false}; ///< VBAT overvoltage
  bool vio_uv{false};  ///< VIO undervoltage
  bool vio_ov{false};  ///< VIO overvoltage
  bool vdd_uv{false};  ///< VDD undervoltage
  bool vdd_ov{false};  ///< VDD overvoltage

  // Temperature
  bool ot_warning{false}; ///< Over-temperature warning
  bool ot_error{false};   ///< Over-temperature error

  // Other faults
  bool clock_fault{false};  ///< Clock fault
  bool spi_wd_error{false}; ///< SPI watchdog error
  bool por_event{false};    ///< Power-on reset occurred
  bool reset_event{false};  ///< External reset occurred

  // Internal diagnostics
  bool supply_nok_internal{false}; ///< Internal supply fault
  bool supply_nok_external{false}; ///< External supply fault

  // Voltage readings
  uint16_t vbat_voltage{0}; ///< VBAT voltage (raw value)
  uint16_t vio_voltage{0};  ///< VIO voltage (raw value)
};

/**
 * @brief Channel diagnostic information
 */
struct ChannelDiagnostics {
  // Error flags
  bool overcurrent{false};            ///< Over-current detected
  bool short_to_ground{false};        ///< Short to ground
  bool open_load{false};              ///< Open load
  bool over_temperature{false};       ///< Channel over-temperature
  bool open_load_short_ground{false}; ///< Open load or short to ground

  // Warning flags
  bool ot_warning{false};                 ///< Over-temperature warning
  bool current_regulation_warning{false}; ///< Current regulation warning
  bool pwm_regulation_warning{false};     ///< PWM regulation warning
  bool olsg_warning{false};               ///< OLSG warning

  // Measurements
  uint16_t average_current{0}; ///< Average current (raw value)
  uint16_t duty_cycle{0};      ///< PWM duty cycle (raw value)
  uint16_t min_current{0};     ///< Minimum current
  uint16_t max_current{0};     ///< Maximum current
  uint16_t vbat_feedback{0};   ///< VBAT feedback
};

/**
 * @brief Comprehensive fault report structure
 *
 * @details
 * Contains all fault information from GLOBAL_DIAG0, GLOBAL_DIAG1, GLOBAL_DIAG2,
 * and per-channel fault diagnostics.
 */
struct FaultReport {
  bool any_fault{false}; ///< Any fault condition present

  // External Supply Faults (GLOBAL_DIAG0)
  bool vbat_uv{false}; ///< VBAT undervoltage
  bool vbat_ov{false}; ///< VBAT overvoltage
  bool vio_uv{false};  ///< VIO undervoltage
  bool vio_ov{false};  ///< VIO overvoltage
  bool vdd_uv{false};  ///< VDD undervoltage
  bool vdd_ov{false};  ///< VDD overvoltage

  // Internal Supply Faults (GLOBAL_DIAG1)
  bool vr_iref_uv{false}; ///< Internal bias current undervoltage
  bool vr_iref_ov{false}; ///< Internal bias current overvoltage
  bool vdd2v5_uv{false};  ///< Internal 2.5V supply undervoltage
  bool vdd2v5_ov{false};  ///< Internal 2.5V supply overvoltage
  bool ref_uv{false};     ///< Internal reference undervoltage
  bool ref_ov{false};     ///< Internal reference overvoltage
  bool vpre_ov{false};    ///< Internal pre-regulator overvoltage
  bool hvadc_err{false};  ///< Internal monitoring ADC error

  // System Faults (GLOBAL_DIAG0)
  bool clock_fault{false};  ///< Clock fault
  bool spi_wd_error{false}; ///< SPI watchdog error

  // Temperature Faults (GLOBAL_DIAG0)
  bool ot_error{false};   ///< Central over-temperature error
  bool ot_warning{false}; ///< Central over-temperature warning

  // Reset Events (GLOBAL_DIAG0)
  bool por_event{false};   ///< Power-on reset event
  bool reset_event{false}; ///< External reset event

  // Memory/ECC Faults (GLOBAL_DIAG2)
  bool reg_ecc_err{false}; ///< Register ECC error
  bool otp_ecc_err{false}; ///< OTP ECC error
  bool otp_virgin{false};  ///< OTP virgin/unconfigured

  // Channel-specific faults (per channel)
  struct ChannelFaults {
    bool has_fault{false};                  ///< Any fault on this channel
    bool overcurrent{false};                ///< Over-current
    bool short_to_ground{false};            ///< Short to ground
    bool open_load{false};                  ///< Open load
    bool over_temperature{false};           ///< Over-temperature
    bool open_load_short_ground{false};     ///< Open load or short to ground
    bool ot_warning{false};                 ///< Over-temperature warning
    bool current_regulation_warning{false}; ///< Current regulation warning
    bool pwm_regulation_warning{false};     ///< PWM regulation warning
    bool olsg_warning{false};               ///< OLSG warning
  };

  std::array<ChannelFaults, 6> channels{}; ///< Faults for each channel (CH0-CH5)

  // Summary flags from FB_STAT
  bool supply_nok_internal{false}; ///< Internal supply fault summary
  bool supply_nok_external{false}; ///< External supply fault summary
};

/**
 * @brief Global configuration structure
 */
struct GlobalConfig {
  bool crc_enabled{true};             ///< Enable CRC checking
  bool spi_watchdog_enabled{true};    ///< Enable SPI watchdog
  bool clock_watchdog_enabled{true};  ///< Enable clock watchdog
  bool vio_5v{false};                 ///< VIO voltage (false=3.3V, true=5.0V)
  float vbat_uv_voltage{4.0F};        ///< VBAT UV threshold voltage in volts (default: ~4V)
  float vbat_ov_voltage{41.0F};       ///< VBAT OV threshold voltage in volts (default: ~41V)
  uint16_t spi_watchdog_reload{1000}; ///< SPI watchdog reload value
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
 * Uses std::expected for robust error handling without exceptions.
 *
 * @par Initialization Sequence:
 * 1. Construct driver with CommInterface reference
 * 2. Call init() to initialize hardware and verify device
 * 3. Call enter_mission_mode() to enable channel control
 * 4. Configure channels with set_channel_mode() and configure_channel()
 * 5. Set current with set_current_setpoint()
 * 6. Enable outputs with enable_channel()
 * 7. Monitor with get diagnostics functions
 */
template <typename CommType>
class Driver {
public:
  /**
   * @brief Construct driver with communication interface
   *
   * @param comm Reference to communication interface
   *
   * @pre CommInterface must remain valid for the lifetime of the Driver
   * @post Driver is constructed but not initialized
   */
  explicit Driver(CommType& comm) noexcept
      : comm_(comm) {}

  /**
   * @brief Destructor - ensures clean shutdown
   */
  ~Driver() noexcept {
    if (initialized_) {
      // Best effort shutdown - ignore errors
      (void)DisableAllChannels();
    }
  }

  // Prevent copying
  Driver(const Driver&) = delete;
  Driver& operator=(const Driver&) = delete;

  // Delete move operations (contains reference member)
  Driver(Driver&&) noexcept = delete;
  Driver& operator=(Driver&&) noexcept = delete;

  //==========================================================================
  // INITIALIZATION AND MODE CONTROL
  //==========================================================================

  /**
   * @brief Initialize the driver and hardware
   *
   * @details
   * Performs complete initialization sequence:
   * 1. Initialize CommInterface (SPI peripheral)
   * 2. Verify device communication
   * 3. Read and verify device ID
   * 4. Apply default configuration (in Config Mode)
   * 5. Clear any power-on faults
   *
   * After init(), device is in Config Mode. Call enter_mission_mode() to enable outputs.
   *
   * @return DriverResult<void> Success or error code
   * @retval DriverError::HardwareError CommInterface initialization failed
   * @retval DriverError::DeviceNotResponding No SPI response
   * @retval DriverError::WrongDeviceID Device ID mismatch
   */
  [[nodiscard]] DriverResult<void> Init() noexcept;

  /**
   * @brief Enter Mission Mode (enables channel control)
   *
   * @details
   * Transitions from Config Mode to Mission Mode. Channel outputs can only
   * be enabled in Mission Mode. Most configuration registers can only be
   * written in Config Mode.
   *
   * @return DriverResult<void> Success or error
   * @retval DriverError::NotInitialized Driver not initialized
   */
  [[nodiscard]] DriverResult<void> EnterMissionMode() noexcept;

  /**
   * @brief Enter Config Mode (allows configuration changes)
   *
   * @details
   * Transitions from Mission Mode to Config Mode. All channel outputs
   * are automatically disabled when entering Config Mode.
   *
   * @return DriverResult<void> Success or error
   */
  [[nodiscard]] DriverResult<void> EnterConfigMode() noexcept;

  /**
   * @brief Check if in mission mode
   * @return true if in mission mode, false if in config mode
   */
  [[nodiscard]] bool IsMissionMode() const noexcept {
    return mission_mode_;
  }

  /**
   * @brief Check if in config mode
   * @return true if in config mode, false if in mission mode
   */
  [[nodiscard]] bool IsConfigMode() const noexcept {
    return !mission_mode_;
  }

  //==========================================================================
  // GLOBAL CONFIGURATION
  //==========================================================================

  /**
   * @brief Configure global device settings
   *
   * @param config Global configuration structure
   * @return DriverResult<void> Success or error
   * @retval DriverError::NotInitialized Driver not initialized
   * @retval DriverError::WrongMode Must be in Config Mode
   */
  [[nodiscard]] DriverResult<void> ConfigureGlobal(const GlobalConfig& config) noexcept;

  /**
   * @brief Enable/disable CRC checking
   *
   * @param enabled true to enable CRC
   * @return DriverResult<void> Success or error
   */
  [[nodiscard]] DriverResult<void> SetCrcEnabled(bool enabled) noexcept;

  /**
   * @brief Set VBAT under/overvoltage thresholds from voltage values (High-Level API)
   *
   * @param uv_voltage UV threshold voltage in volts (0V to ~41.4V)
   * @param ov_voltage OV threshold voltage in volts (0V to ~41.4V)
   * @return DriverResult<void> Success or error
   *
   * @details
   * Automatically calculates register values from voltage.
   * Formula: register_value = voltage / 0.16208V
   *
   * @note This is the recommended API for most users. Use SetVbatThresholdsRaw()
   *       only if you need direct control over register values.
   */
  [[nodiscard]] DriverResult<void> SetVbatThresholds(float uv_voltage, float ov_voltage) noexcept;

  /**
   * @brief Set VBAT under/overvoltage thresholds (Low-Level API)
   *
   * @param uv_threshold UV threshold register value (0-255, V_BAT_UV = value * 0.16208V)
   * @param ov_threshold OV threshold register value (0-255, V_BAT_OV = value * 0.16208V)
   * @return DriverResult<void> Success or error
   *
   * @note For most users, prefer SetVbatThresholds(uv_voltage, ov_voltage) which
   *       automatically calculates these values from voltage.
   */
  [[nodiscard]] DriverResult<void> SetVbatThresholdsRaw(uint8_t uv_threshold,
                                                        uint8_t ov_threshold) noexcept;

  //==========================================================================
  // CHANNEL CONTROL
  //==========================================================================

  /**
   * @brief Enable or disable a channel
   *
   * @param channel Channel to control
   * @param enabled true to enable, false to disable
   * @return DriverResult<void> Success or error
   * @retval DriverError::WrongMode Must be in Mission Mode
   */
  [[nodiscard]] DriverResult<void> EnableChannel(Channel channel, bool enabled) noexcept;

  /**
   * @brief Enable or disable multiple channels
   *
   * @param channel_mask Bitmask where bit N enables channel N
   * @return DriverResult<void> Success or error
   */
  [[nodiscard]] DriverResult<void> EnableChannels(uint8_t channel_mask) noexcept;

  /**
   * @brief Enable all channels
   */
  [[nodiscard]] DriverResult<void> EnableAllChannels() noexcept;

  /**
   * @brief Disable all channels
   */
  [[nodiscard]] DriverResult<void> DisableAllChannels() noexcept;

  /**
   * @brief Set channel operation mode
   *
   * @param channel Channel to configure
   * @param mode Operation mode (ICC, Direct Drive, etc.)
   * @return DriverResult<void> Success or error
   * @retval DriverError::WrongMode Must be in Config Mode
   */
  [[nodiscard]] DriverResult<void> SetChannelMode(Channel channel, ChannelMode mode) noexcept;

  /**
   * @brief Configure channel for parallel operation
   *
   * @param pair Parallel pair to configure (0/3, 1/2, or 4/5)
   * @param enabled true to enable parallel operation
   * @return DriverResult<void> Success or error
   * @retval DriverError::WrongMode Must be in Config Mode
   */
  [[nodiscard]] DriverResult<void> SetParallelOperation(ParallelPair pair, bool enabled) noexcept;

  //==========================================================================
  // CURRENT CONTROL (ICC MODE)
  //==========================================================================

  /**
   * @brief Set current setpoint for channel
   *
   * @param channel Channel to configure
   * @param current_ma Desired current in milliamperes (0-2000 single, 0-4000 parallel)
   * @param parallel_mode Set true if channel is in parallel mode
   * @return DriverResult<void> Success or error
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
  [[nodiscard]] DriverResult<void> SetCurrentSetpoint(Channel channel, uint16_t current_ma,
                                                      bool parallel_mode = false) noexcept;

  /**
   * @brief Get current setpoint for channel
   *
   * @param channel Channel to query
   * @param parallel_mode true if channel is in parallel mode
   * @return DriverResult<uint16_t> Current in mA or error
   */
  [[nodiscard]] DriverResult<uint16_t> GetCurrentSetpoint(Channel channel,
                                                          bool parallel_mode = false) noexcept;

  /**
   * @brief Configure PWM period from desired period in microseconds (High-Level API)
   *
   * @param channel Channel to configure
   * @param period_us Desired PWM period in microseconds
   * @return DriverResult<void> Success or error
   *
   * @details
   * Automatically calculates mantissa, exponent, and low_freq_range to achieve
   * the desired period. Valid range: ~0.125 µs to ~32.64 ms.
   *
   * **Formula**: T_pwm = PERIOD_MANT × 2^PERIOD_EXP × (1/f_sys)
   *              Low Freq: T_pwm = PERIOD_MANT × 8 × 2^PERIOD_EXP × (1/f_sys)
   *              Where f_sys ≈ 8 MHz
   *
   * @note This is the recommended API for most users. Use ConfigurePwmPeriodRaw()
   *       only if you need direct control over register values.
   */
  [[nodiscard]] DriverResult<void> ConfigurePwmPeriod(Channel channel, float period_us) noexcept;

  /**
   * @brief Configure PWM parameters for ICC (Low-Level API)
   *
   * @param channel Channel to configure
   * @param period_mantissa PWM period mantissa (0-255)
   * @param period_exponent PWM period exponent (0-7)
   * @param low_freq_range Enable low frequency range (8x multiplier)
   * @return DriverResult<void> Success or error
   *
   * @details PWM period: t_PWM = mantissa * 2^exponent * (1/f_sys)
   *          If low_freq_range: t_PWM = mantissa * 8 * 2^exponent * (1/f_sys)
   *
   * @note For most users, prefer ConfigurePwmPeriod(period_us) which automatically
   *       calculates these values from a desired period.
   */
  [[nodiscard]] DriverResult<void> ConfigurePwmPeriodRaw(Channel channel, uint8_t period_mantissa,
                                                         uint8_t period_exponent,
                                                         bool low_freq_range = false) noexcept;

  /**
   * @brief Configure dither from amplitude and frequency (High-Level API)
   *
   * @param channel Channel to configure
   * @param amplitude_ma Desired dither amplitude in milliamperes
   * @param frequency_hz Desired dither frequency in Hz
   * @param parallel_mode true if channel is in parallel mode (affects max current)
   * @return DriverResult<void> Success or error
   *
   * @details
   * Automatically calculates step_size, num_steps, and flat_steps to achieve
   * the desired amplitude and frequency.
   *
   * **Formulas**:
   * - I_dither = STEPS × STEP_SIZE × 2A / 32767
   * - T_dither = [4×STEPS + 2×FLAT] × t_ref_clk
   *
   * @note This is the recommended API for most users. Use ConfigureDitherRaw()
   *       only if you need direct control over register values.
   */
  [[nodiscard]] DriverResult<void> ConfigureDither(Channel channel, float amplitude_ma,
                                                   float frequency_hz,
                                                   bool parallel_mode = false) noexcept;

  /**
   * @brief Configure dither parameters (Low-Level API)
   *
   * @param channel Channel to configure
   * @param step_size Dither step size (0-4095)
   * @param num_steps Number of steps in quarter period (0-255)
   * @param flat_steps Number of flat clock cycles at top/bottom (0-255)
   * @return DriverResult<void> Success or error
   *
   * @details
   * **Formulas**:
   * - I_dither = STEPS × STEP_SIZE × 2A / 32767
   * - T_dither = [4×STEPS + 2×FLAT] × t_ref_clk
   *
   * @note For most users, prefer ConfigureDither(amplitude_ma, frequency_hz) which
   *       automatically calculates these values from user-friendly parameters.
   */
  [[nodiscard]] DriverResult<void> ConfigureDitherRaw(Channel channel, uint16_t step_size,
                                                      uint8_t num_steps,
                                                      uint8_t flat_steps) noexcept;

  /**
   * @brief Configure channel slew rate and diagnostics
   *
   * @param channel Channel to configure
   * @param config Channel configuration
   * @return DriverResult<void> Success or error
   */
  [[nodiscard]] DriverResult<void> ConfigureChannel(Channel channel,
                                                    const ChannelConfig& config) noexcept;

  //==========================================================================
  // STATUS AND DIAGNOSTICS
  //==========================================================================

  /**
   * @brief Get global device status
   *
   * @return DriverResult<DeviceStatus> Device status or error
   */
  [[nodiscard]] DriverResult<DeviceStatus> GetDeviceStatus() noexcept;

  /**
   * @brief Get channel diagnostic information
   *
   * @param channel Channel to query
   * @return DriverResult<ChannelDiagnostics> Diagnostics or error
   */
  [[nodiscard]] DriverResult<ChannelDiagnostics> GetChannelDiagnostics(Channel channel) noexcept;

  /**
   * @brief Get average current for a channel
   *
   * @param channel Channel to query
   * @param parallel_mode true if in parallel mode
   * @return DriverResult<uint16_t> Average current in mA or error
   */
  [[nodiscard]] DriverResult<uint16_t> GetAverageCurrent(Channel channel,
                                                         bool parallel_mode = false) noexcept;

  /**
   * @brief Get PWM duty cycle for a channel
   *
   * @param channel Channel to query
   * @return DriverResult<uint16_t> Duty cycle (raw 16-bit value)
   */
  [[nodiscard]] DriverResult<uint16_t> GetDutyCycle(Channel channel) noexcept;

  /**
   * @brief Get VBAT voltage
   *
   * @return DriverResult<uint16_t> VBAT in millivolts or error
   */
  [[nodiscard]] DriverResult<uint16_t> GetVbatVoltage() noexcept;

  /**
   * @brief Get VIO voltage
   *
   * @return DriverResult<uint16_t> VIO in millivolts or error
   */
  [[nodiscard]] DriverResult<uint16_t> GetVioVoltage() noexcept;

  /**
   * @brief Get VDD voltage
   *
   * @return DriverResult<uint16_t> VDD in millivolts or error
   */
  [[nodiscard]] DriverResult<uint16_t> GetVddVoltage() noexcept;

  /**
   * @brief Get VBAT thresholds
   *
   * @param uv_threshold Output parameter for UV threshold in millivolts
   * @param ov_threshold Output parameter for OV threshold in millivolts
   * @return DriverResult<void> Success or error
   */
  [[nodiscard]] DriverResult<void> GetVbatThresholds(uint16_t& uv_threshold,
                                                     uint16_t& ov_threshold) noexcept;

  //==========================================================================
  // FAULT MANAGEMENT
  //==========================================================================

  /**
   * @brief Clear all fault flags
   *
   * @details
   * Clears latched fault conditions. Does not clear active faults.
   *
   * @return DriverResult<void> Success or error
   */
  [[nodiscard]] DriverResult<void> ClearFaults() noexcept;

  /**
   * @brief Check if any fault exists
   *
   * @return DriverResult<bool> true if any fault exists
   */
  [[nodiscard]] DriverResult<bool> HasAnyFault() noexcept;

  /**
   * @brief Get comprehensive fault report
   *
   * @details
   * Reads all fault registers (GLOBAL_DIAG0, GLOBAL_DIAG1, GLOBAL_DIAG2)
   * and all channel fault registers to provide a complete fault report.
   *
   * @return DriverResult<FaultReport> Complete fault report or error
   */
  [[nodiscard]] DriverResult<FaultReport> GetAllFaults() noexcept;

  /**
   * @brief Print all detected faults to log
   *
   * @details
   * Reads all faults and prints them in a formatted, easy-to-read format.
   * Only prints faults that are actually detected.
   *
   * @return DriverResult<void> Success or error
   */
  [[nodiscard]] DriverResult<void> PrintAllFaults() noexcept;

  /**
   * @brief Software reset of the device
   *
   * @details
   * Resets all registers to default values. init() must be called again.
   *
   * @return DriverResult<void> Success or error
   */
  [[nodiscard]] DriverResult<void> SoftwareReset() noexcept;

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
   * @return DriverResult<void> Success or error
   */
  [[nodiscard]] DriverResult<void> ReloadSpiWatchdog(uint16_t reload_value) noexcept;

  //==========================================================================
  // DEVICE INFORMATION
  //==========================================================================

  /**
   * @brief Read IC version and ID
   *
   * @return DriverResult<uint16_t> ICVID register value
   */
  [[nodiscard]] DriverResult<uint16_t> GetIcVersion() noexcept;

  /**
   * @brief Read unique chip ID
   *
   * @return DriverResult<std::array<uint16_t, 3>> Three 16-bit ID registers
   */
  [[nodiscard]] DriverResult<std::array<uint16_t, 3>> GetChipId() noexcept;

  /**
   * @brief Verify device ID matches expected value
   *
   * @return DriverResult<bool> true if ID matches
   */
  [[nodiscard]] DriverResult<bool> VerifyDevice() noexcept;

  /**
   * @brief Check if driver is initialized
   *
   * @return true if initialized and ready
   */
  [[nodiscard]] bool IsInitialized() const noexcept {
    return initialized_;
  }

  //==========================================================================
  // GPIO CONTROL (Reset, Enable, Fault Status)
  //==========================================================================

  /**
   * @brief Hold device in reset or release reset
   *
   * @details
   * Controls the RESN (reset) pin. When in reset, the device is held in
   * a reset state and all registers are reset to default values.
   *
   * @param reset If true, hold device in reset (RESN LOW). If false, release reset (RESN HIGH).
   * @return DriverResult<void> Success or error
   * @retval DriverError::HardwareError GPIO control failed
   *
   * @note RESN must be released (reset=false) for SPI communication to work.
   * @note Holding device in reset will disable all channels and reset registers.
   */
  [[nodiscard]] DriverResult<void> SetReset(bool reset) noexcept;

  /**
   * @brief Hold device in reset
   *
   * @details
   * Convenience function to hold device in reset state.
   * Equivalent to SetReset(true).
   *
   * @return DriverResult<void> Success or error
   */
  [[nodiscard]] DriverResult<void> HoldReset() noexcept {
    return SetReset(true);
  }

  /**
   * @brief Release device from reset
   *
   * @details
   * Convenience function to release device from reset state.
   * Equivalent to SetReset(false).
   *
   * @return DriverResult<void> Success or error
   */
  [[nodiscard]] DriverResult<void> ReleaseReset() noexcept {
    return SetReset(false);
  }

  /**
   * @brief Enable or disable output channels
   *
   * @details
   * Controls the EN (enable) pin. When disabled, all output channels
   * are disabled regardless of channel enable register settings.
   *
   * @param enable If true, enable outputs (EN HIGH). If false, disable outputs (EN LOW).
   * @return DriverResult<void> Success or error
   * @retval DriverError::HardwareError GPIO control failed
   *
   * @note This affects all channels. Individual channel control is via EnableChannel().
   * @note EN only affects output channels, not SPI communication.
   */
  [[nodiscard]] DriverResult<void> SetEnable(bool enable) noexcept;

  /**
   * @brief Enable output channels
   *
   * @details
   * Convenience function to enable outputs.
   * Equivalent to SetEnable(true).
   *
   * @return DriverResult<void> Success or error
   */
  [[nodiscard]] DriverResult<void> Enable() noexcept {
    return SetEnable(true);
  }

  /**
   * @brief Disable output channels
   *
   * @details
   * Convenience function to disable outputs.
   * Equivalent to SetEnable(false).
   *
   * @return DriverResult<void> Success or error
   */
  [[nodiscard]] DriverResult<void> Disable() noexcept {
    return SetEnable(false);
  }

  /**
   * @brief Check if device fault is detected
   *
   * @details
   * Reads the FAULTN pin to check if a fault condition is present.
   * FAULTN is an active-low signal, so LOW = fault detected.
   *
   * @param print_faults If true and fault is detected, automatically calls PrintAllFaults()
   *                     to display detailed fault information. Default: false (no automatic
   * printing)
   *
   * @return DriverResult<bool> true if fault detected, false if no fault, or error
   * @retval DriverError::HardwareError GPIO read failed
   *
   * @note This reads the hardware FAULTN pin. When a fault is detected, print_faults is true,
   *       and the driver is initialized, detailed fault information is automatically printed
   *       via PrintAllFaults().
   */
  [[nodiscard]] DriverResult<bool> IsFault(bool print_faults = false) noexcept;

  //==========================================================================
  // REGISTER ACCESS (Advanced)
  //==========================================================================

  /**
   * @brief Read 16-bit register
   *
   * @param address Register address (10-bit)
   * @param verify_crc Override CRC verification (default: uses internal CRC enable state)
   * @return DriverResult<uint32_t> Register value (16-bit or 22-bit depending on reply mode) or
   * error
   * @note If verify_crc is not explicitly provided, uses internal CRC enable state
   *       which tracks GLOBAL_CONFIG::CRC_EN. Set to false to override (e.g., during init).
   */
  [[nodiscard]] DriverResult<uint32_t> ReadRegister(uint16_t address,
                                                    bool verify_crc = false) noexcept;

  /**
   * @brief Write 16-bit register
   *
   * @param address Register address (10-bit)
   * @param value Value to write (16-bit)
   * @param verify_crc Override CRC verification (default: uses internal CRC enable state)
   * @param verify_write If true, read back register to verify write succeeded (default: true)
   * @return DriverResult<void> Success or error
   * @note If verify_crc is not explicitly provided, uses internal CRC enable state
   *       which tracks GLOBAL_CONFIG::CRC_EN. Set to false to override (e.g., during init).
   * @note If verify_write is true, reads back the register after write and logs a warning
   *       if the read value doesn't match. Some registers may be write-only (e.g., GLOBAL_CONFIG),
   *       in which case verification will fail gracefully.
   */
  [[nodiscard]] DriverResult<void> WriteRegister(uint16_t address, uint16_t value,
                                                 bool verify_crc = false,
                                                 bool verify_write = true) noexcept;

  /**
   * @brief Modify register bits
   *
   * @param address Register address
   * @param mask Bit mask for modification
   * @param value New bit values
   * @return DriverResult<void> Success or error
   */
  [[nodiscard]] DriverResult<void> ModifyRegister(uint16_t address, uint16_t mask,
                                                  uint16_t value) noexcept;

  // ===========================================================================
  // Driver Version
  // ===========================================================================

  /** @brief Get the compiled driver version string. */
  static constexpr const char* GetDriverVersion() noexcept {
    return HF_TLE92466ED_VERSION_STRING;
  }

  /** @brief Get the compiled driver major version number. */
  static constexpr uint8_t GetDriverVersionMajor() noexcept {
    return HF_TLE92466ED_VERSION_MAJOR;
  }

  /** @brief Get the compiled driver minor version number. */
  static constexpr uint8_t GetDriverVersionMinor() noexcept {
    return HF_TLE92466ED_VERSION_MINOR;
  }

  /** @brief Get the compiled driver patch version number. */
  static constexpr uint8_t GetDriverVersionPatch() noexcept {
    return HF_TLE92466ED_VERSION_PATCH;
  }

private:
  //==========================================================================
  // PRIVATE METHODS
  //==========================================================================

  /**
   * @brief Transfer SPI frame with CRC calculation and verification
   */
  [[nodiscard]] DriverResult<SPIFrame> transferFrame(const SPIFrame& tx_frame,
                                                     bool verify_crc = true) noexcept;

  /**
   * @brief Validate channel number
   */
  [[nodiscard]] constexpr bool isValidChannelInternal(Channel channel) const noexcept {
    return IsValidChannel(channel);
  }

  /**
   * @brief Check if driver is initialized
   */
  [[nodiscard]] DriverResult<void> checkInitialized() const noexcept {
    if (!initialized_) {
      return tle::unexpected(DriverError::NotInitialized);
    }
    return {};
  }

  /**
   * @brief Check if in mission mode
   */
  [[nodiscard]] DriverResult<void> checkMissionMode() const noexcept {
    if (!mission_mode_) {
      return tle::unexpected(DriverError::WrongMode);
    }
    return {};
  }

  /**
   * @brief Check if in config mode
   */
  [[nodiscard]] DriverResult<void> checkConfigMode() const noexcept {
    if (mission_mode_) {
      return tle::unexpected(DriverError::WrongMode);
    }
    return {};
  }

  /**
   * @brief Apply default configuration after initialization
   */
  [[nodiscard]] DriverResult<void> applyDefaultConfig() noexcept;

  /**
   * @brief Clear faults without checking initialization status (used during Init)
   */
  [[nodiscard]] DriverResult<void> clearFaultsInternal() noexcept;

  /**
   * @brief Set VBAT thresholds without checking initialization status (used during Init)
   * @param uv_voltage Under-voltage threshold in volts
   * @param ov_voltage Over-voltage threshold in volts
   * @return DriverResult<void> Success or error
   */
  [[nodiscard]] DriverResult<void> setVbatThresholdsInternal(float uv_voltage,
                                                             float ov_voltage) noexcept;

  /**
   * @brief Parse SPI status from reply frame
   */
  [[nodiscard]] DriverResult<void> checkSpiStatus(const SPIFrame& rx_frame) noexcept;

  /**
   * @brief Check if channel is currently in parallel mode
   * @param channel Channel to check
   * @return DriverResult<bool> true if channel is paralleled, false otherwise
   */
  [[nodiscard]] DriverResult<bool> isChannelParallel(Channel channel) noexcept;

  /**
   * @brief Diagnose clock configuration by reading CLK_DIV register
   *
   * @details
   * Reads and logs the CLK_DIV register to help diagnose clock-related
   * critical faults. This is called during initialization.
   */
  void diagnoseClockConfiguration() noexcept;

  //==========================================================================
  // MEMBER VARIABLES
  //==========================================================================

  CommType& comm_;         ///< Communication interface
  bool initialized_{false};       ///< Initialization status
  bool mission_mode_{false};      ///< Mission mode flag (vs config mode)
  bool crc_enabled_{false};       ///< CRC enable state (tracks GLOBAL_CONFIG::CRC_EN)
  bool vio_5v_mode_{false};       ///< VIO mode state (tracks GLOBAL_CONFIG::VIO_SEL, false=3.3V, true=5V)
  uint16_t ch_ctrl_cache_{0U}; ///< Cached CH_CTRL register value (reads return 0x0000)
  uint16_t channel_enable_cache_{0U};             ///< Cached channel enable state
  std::array<uint16_t, 6> channel_setpoints_; ///< Cached current setpoints
};

// Include template implementation (must be inside namespace before it closes)
#define TLE92466ED_HEADER_INCLUDED
// NOLINTNEXTLINE(bugprone-suspicious-include) - Intentional: template implementation file
#include "../src/tle92466ed.ipp"
#undef TLE92466ED_HEADER_INCLUDED

// Public API: Get driver version string
inline const char* GetDriverVersion() noexcept {
  return HF_TLE92466ED_VERSION_STRING;
}

} // namespace tle92466ed
