/**
 * @file tle92466ed.hpp
 * @brief Main driver class for TLE92466ED Six-Channel Low-Side Solenoid Driver IC
 * @copyright Copyright (c) 2024-2025 HardFOC. All rights reserved.
 */
#pragma once
#include <array>
#include <span>

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
  WriteToReadOnly,     ///< Attempted write to read-only register
  FeedbackNotReady     ///< Averaged-feedback measurement window not established
};

/**
 * @note `FeedbackNotReady` is returned by `GetAverageCurrent()` when
 *       `FB_FEEDBACK::HasValidMeasurementWindow()` is false for the channel's
 *       `FB_DC` read (see `FB_FEEDBACK::kMinValidTpMant`). Typical causes:
 *       `DITHER_CLK_DIV` still at its POR value of 0 (so `Tmeas` never runs),
 *       feedback frozen via `FB_FRZ`, or a pipelined reply spliced on a shared
 *       bus. Reporting it explicitly stops a near-zero divisor from being
 *       published as a plausible-looking current.
 */

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
  // Error flags (DIAG_ERR_CHGRx, \u00a75.3.2.12)
  bool overcurrent{false};            ///< Over-current detected
  bool short_to_ground{false};        ///< Short to ground
  bool open_load{false};              ///< Open load
  bool over_temperature{false};       ///< Channel over-temperature error (OTE)
  bool open_load_short_ground{false}; ///< Open load or short to ground (OLSG pre-check)

  // Warning flags (DIAG_WARN_CHGRx, \u00a75.3.2.13)
  bool ot_warning{false};                 ///< Channel over-temperature warning (OTW)
  bool current_regulation_warning{false}; ///< ICC current regulation warning (I_REG)
  bool pwm_regulation_warning{false};     ///< ICC PWM regulation warning (PWM_REG)
  bool olsg_warning{false};               ///< OLSG warning (OLSG_WARN)
  bool olsg_check_not_performed{false};   ///< OLSG check not yet performed (OLSG_WARN_CHK_NOK; POR=1)

  // Measurements (raw 22-bit reply low halves unless noted)
  uint16_t average_current{0}; ///< Raw FB_I_AVG bits — use GetAverageCurrent() for mA
  uint16_t duty_cycle{0};      ///< Raw FB_DC bits — use GetDutyCycle() for permyriad
  int16_t  min_current_mA{0};  ///< Minimum load current in mA (signed; FB_IMIN_IMAX.IMIN)
  int16_t  max_current_mA{0};  ///< Maximum load current in mA (signed; FB_IMIN_IMAX.IMAX)
  uint16_t vbat_feedback{0};   ///< VBAT feedback (raw FB_VBAT)
};

/**
 * @brief Latched error bits for one channel (DIAG_ERR_CHGRx only).
 *
 * The error half of @ref ChannelDiagnostics without the warning registers or
 * the four feedback reads, so a supervisory loop can refresh fault state for
 * every channel in three register reads instead of thirty-six.
 */
struct ChannelErrorFlags {
  bool open_load_short_ground{false}; ///< OLSG pre-check (latched)
  bool open_load{false};              ///< OL
  bool overcurrent{false};            ///< OC
  bool short_to_ground{false};        ///< SG
  bool over_temperature{false};       ///< OTE
};

/**
 * @brief One channel's decoded average current from a batched freeze/read.
 *
 * @details Decoded from the FB_I_AVG / FB_DC mantissa ratio (datasheet
 * Equation 22), which is the path §4.10.6 requires for current supervision.
 */
struct ChannelCurrentSample {
  int32_t iavg_ma{0};        ///< Signed average load current, milliamps
  uint16_t tp_mant{0};       ///< FB_DC TP_MANT — the measurement window itself
  uint16_t to_mant{0};       ///< FB_DC TO_MANT — last PWM cycle on-time (not the average)
  bool window_valid{false};  ///< Both reads succeeded and TP_MANT is credible
};

/**
 * @brief Result of one @ref Tle92466edDriver::SampleChannelCurrents call.
 */
struct CurrentFeedbackBatch {
  uint8_t requested_mask{0}; ///< Channels the caller asked for
  uint8_t update_mask{0};    ///< FB_UPD: channels with a completed window
  uint8_t valid_mask{0};     ///< Channels whose sample is trustworthy
  std::array<ChannelCurrentSample, 6> channels{};
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
   * @param perform_hardware_reset When true (default), pulse RESN low then
   *        release before SPI identity. When false, leave RESN released (HIGH)
   *        and only re-run EN settle + SPI verify — used by shared-bus retries
   *        so the eval RESET LED is not hammered every few hundred ms.
   * @return DriverResult<void> Success or error code
   * @retval DriverError::HardwareError CommInterface initialization failed
   * @retval DriverError::DeviceNotResponding No SPI response
   * @retval DriverError::WrongDeviceID Device ID mismatch
   */
  [[nodiscard]] DriverResult<void> Init(bool perform_hardware_reset = true) noexcept;

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
   * @param channel    Channel to configure
   * @param period_us  Desired PWM period in microseconds
   * @return DriverResult<void> Success or DriverError::InvalidParameter
   *
   * @details
   * The TLE92466ED's PWM frequency controller is specified to operate
   * between 110 Hz and 4 kHz (datasheet Electrical Characteristics).
   * In `period_us` terms that is a usable range of:
   *
   *   - Standard range (LOW_FREQ_RANGE_EN = 0):
   *       250 µs ≤ period_us ≤ 9090 µs   (= 4 kHz .. 110 Hz)
   *   - Low-frequency range (LOW_FREQ_RANGE_EN = 1):
   *       2000 µs ≤ period_us ≤ 72727 µs (= 500 Hz .. 13.75 Hz)
   *
   * The driver auto-selects the appropriate range. Periods outside the
   * combined min/max (250 µs – 72.7 ms) return `InvalidParameter`.
   * Periods that fall in the gap between the two ranges (9.1 – 2 ms,
   * i.e. 110 Hz – 500 Hz) are accepted but logged as a warning since
   * the PWM-control loop is not specified there.
   *
   * **Formulas**:
   *   T_pwm = PERIOD_MANT × 2^PERIOD_EXP × (1/f_sys)
   *   Low-range: T_pwm = PERIOD_MANT × 8 × 2^PERIOD_EXP × (1/f_sys)
   *   f_sys ≈ 8 MHz
   *
   * @note Reaching the ultrasonic range (>20 kHz) is not possible with
   *       this chip family — 4 kHz is the hardware ceiling.
   * @note This is the recommended API for most users. Use
   *       `ConfigurePwmPeriodRaw()` only if you need direct control
   *       over register values.
   *
   * @see PERIOD::kSpecMinPeriod_us, PERIOD::kSpecMaxPeriod_us,
   *      PERIOD::kSpecLowRangeMinPeriod_us, PERIOD::kSpecLowRangeMaxPeriod_us
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
   * @brief Configure the per-channel dither reference clock (DITHER_CLK_DIV).
   *
   * Per datasheet §5.3.3.7, DITHER_CLK_DIV (offset 0x0004 within each
   * channel bank) sets the dither reference clock used by both the
   * dither generator and the averaged-feedback engine:
   *
   *     tref_clk = MANT × 2^EXP / fSYS
   *
   * **POR reset value is 0x0000** which makes tref_clk = 0, the dither
   * period TDither = 0, and (in ICC mode) the measurement period
   * Tmeas = TDither = 0 — so FB_DC / FB_I_AVG / FB_VBAT NEVER update.
   * The high-level `ConfigureDither()` API calls this function
   * automatically with values derived from the requested dither
   * frequency. Use this API directly if you need a specific tref_clk
   * (e.g. to control the FB update rate independently from the dither
   * amplitude/period).
   *
   * @param channel               Channel to configure
   * @param t_ref_clk_us          Desired reference clock period in microseconds.
   *                              The driver picks the closest representable
   *                              MANT/EXP and logs the actual value used.
   *                              Must be > 0; minimum 0.125 µs (1/fSYS).
   * @param dither_pwm_sync       If true, synchronise dither period start with
   *                              PWM rising edge (DITHER_PWM_SYNC_EN).
   * @param dither_setpoint_sync  If true, restart dither period on every
   *                              setpoint change (DITHER_SETPOINT_SYNC_EN);
   *                              required for the lockstep-feedback mode
   *                              described in datasheet §4.7.5.
   * @return DriverResult<void> Success or error
   */
  [[nodiscard]] DriverResult<void> ConfigureDitherClock(
      Channel channel,
      float t_ref_clk_us,
      bool dither_pwm_sync = false,
      bool dither_setpoint_sync = false) noexcept;

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
   * @brief Get average load current for a channel, in milliamps.
   *
   * @details
   * Reads the compressed mantissa/exponent feedback path (`FB_I_AVG` and
   * `FB_DC`, both 22-bit reply frames) and decodes
   * `Iavg = 4 A × <I_AVG_MANT> / <TP_MANT>` per datasheet §4.10.2.
   * Issues two pipelined SPI reads per register (four frames total).
   *
   * Before decoding, validates the measurement window via
   * `FB_FEEDBACK::HasValidMeasurementWindow()` (minimum
   * `FB_FEEDBACK::kMinValidTpMant` in `TP_MANT` from `FB_DC`).
   *
   * @param channel       Channel to query
   * @param parallel_mode Currently unused — the mantissa-ratio formula
   *                      is identical for parallel-paired channels;
   *                      kept for API stability.
   * @return Average load current in mA on success.
   * @retval DriverError::FeedbackNotReady `TP_MANT` too small for a
   *         trustworthy mantissa ratio (averaging window not established,
   *         feedback frozen, or corrupted pipelined reply).
   * @retval DriverError::InvalidChannel Channel out of range.
   * @retval DriverError::NotInitialized Driver not initialized.
   *
   * @note Uses `FB_I_AVG` (mantissa ratio with `TP_MANT` from `FB_DC`), not
   *       `FB_I_AVG_s16`. For calibration-grade signed current see
   *       `GetCalibrationAvgCurrent_mA()`.
   *
   * @note Negative load current (recirculation) is clamped to 0 in this
   *       legacy unsigned return type.
   *
   * @warning This path freezes the channel and polls `FB_UPD` with
   *          `Delay()` for up to ~8 ms. It is a diagnostic / console helper.
   *          Real-time control must use @ref SampleChannelCurrents, which
   *          never blocks.
   *
   * @see FB_FEEDBACK::kMinValidTpMant, FB_FEEDBACK::HasValidMeasurementWindow
   * @see SampleChannelCurrents
   */
  [[nodiscard]] DriverResult<uint16_t> GetAverageCurrent(Channel channel,
                                                         bool parallel_mode = false) noexcept;

  /**
   * @brief Get PWM duty cycle for a channel, in permyriad (0..10000).
   *
   * @details
   * Reads `FB_DC` (22-bit reply) and decodes `DC = <TO_MANT> / <TP_MANT>`
   * per datasheet §4.10.2, then scales to permyriad so the legacy
   * `uint16_t` return type retains 0.01 % precision.
   *
   * @param channel Channel to query
   * @return Duty cycle in permyriad (0 = 0.00 %, 10000 = 100.00 %).
   *
   * @note Does not return `DriverError::FeedbackNotReady`. When `TP_MANT`
   *       is 0 or the window is invalid, the decoded duty is 0 permyriad.
   *       Use `GetAverageCurrent()` or `ReadChannelFeedback()` when an
   *       explicit not-ready indication is required.
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
  // PHASE 2 - CLOCK, POWER, STATE (D10, D29)
  //==========================================================================

  /**
   * @brief Configure the device clock source (internal osc vs external+PLL).
   *
   * @param source     Desired clock source.
   * @param f_clk_Hz   External clock rate in Hz (required if source is
   *                   ExternalClockPll; ignored otherwise). Supported
   *                   range is 1\u20138 MHz per datasheet \u00a74.1.
   * @return DriverResult<void> Success or error.
   *
   * Writes CLK_DIV (0x0019) in Config Mode. Caller is responsible for
   * waiting the datasheet-specified PLL lock time (\u2248500 \u00b5s).
   */
  [[nodiscard]] DriverResult<void> ConfigureClockSource(ClockSource source,
                                                         uint32_t f_clk_Hz = 0) noexcept;

  /**
   * @brief Get the nominal system clock (always 28 MHz after PLL lock).
   */
  [[nodiscard]] static constexpr uint32_t GetSystemClockHz() noexcept {
    return CLK_DIV::F_SYS_TARGET_HZ;
  }

  /**
   * @brief Atomic readout of VBAT, VIO, VDD and die temperature.
   *
   * Reads FB_VOLTAGE1 and FB_VOLTAGE2 and decodes them via the
   * VOLTAGE_FEEDBACK namespace helpers. Result is stable even while the
   * device is operating in Mission Mode.
   */
  [[nodiscard]] DriverResult<SupplyVoltages> ReadAllSupplyVoltages() noexcept;

  /**
   * @brief Read only the central die temperature in \u00b0C.
   */
  [[nodiscard]] DriverResult<float> GetCentralTemperatureCelsius() noexcept;

  /**
   * @brief Select VIO voltage level (3.3 V or 5.0 V).
   *
   * Updates GLOBAL_CONFIG.VIO_SEL and caches the selection so that
   * subsequent supply-voltage decodes pick the correct LSB.
   * Must be called in Config Mode.
   */
  [[nodiscard]] DriverResult<void> SetVioLevel(VioLevel level) noexcept;

  /**
   * @brief Read the coarse operational state.
   */
  [[nodiscard]] DriverResult<OperationState> GetOperationState() noexcept;

  /**
   * @brief Enter Mission Mode and verify by polling FB_STAT.INIT_DONE.
   * @param timeout_ms Maximum time to wait for INIT_DONE.
   */
  [[nodiscard]] DriverResult<void> EnterMissionModeChecked(uint32_t timeout_ms = 10U) noexcept;

  /**
   * @brief Run the built-in supply-monitor self-test sequence.
   *
   * Exercises the UV/OV swap, the 1.5 V monitor UV/OV trips and the
   * over-temperature test logic. Restores the original GLOBAL_CONFIG on
   * exit. Device must be in Config Mode.
   */
  [[nodiscard]] DriverResult<SupplyMonitorSelfTestResult>
  RunSupplyMonitorSelfTest() noexcept;

  //==========================================================================
  // PHASE 3 - ICC INTEGRATOR & THRESHOLD API (D11, D12, D13, D14, D16)
  //==========================================================================

  /**
   * @brief Set ICC integrator clamp limits (INTEGRATOR_LIMIT, offset 0x03).
   *
   * @param channel                Channel to configure.
   * @param lim_value_abs          Primary integrator limit magnitude (0\u20131023).
   * @param auto_lim_value_abs     Auto-limit setpoint delta magnitude (0\u201331).
   *
   * The driver enforces the datasheet constraint that
   * auto_lim_value_abs must be greater than MIN_INT_THRESH+3
   * (as currently cached in the channel's CTRL register).
   */
  [[nodiscard]] DriverResult<void> SetIntegratorLimits(Channel channel,
                                                       uint16_t lim_value_abs,
                                                       uint8_t  auto_lim_value_abs) noexcept;

  /**
   * @brief Set the PWM controller proportional gain (KI, PERIOD.PWM_CTRL_PARAM).
   *
   * @param channel Channel to configure.
   * @param ki      4-bit PWM controller parameter (0\u201315).
   */
  [[nodiscard]] DriverResult<void> SetPwmControllerKi(Channel channel, uint8_t ki) noexcept;

  /**
   * @brief Set the minimum integrator threshold (CTRL.MIN_INT_THRESH).
   *
   * @param channel          Channel to configure.
   * @param min_int_thresh   Signed 8-bit minimum integrator threshold.
   */
  [[nodiscard]] DriverResult<void> SetMinIntegratorThreshold(Channel channel,
                                                             int8_t min_int_thresh) noexcept;

  /**
   * @brief Switch to manual on-time mode (DIRECT_DRIVE_SPI) with fixed TON.
   *
   * @param channel     Channel to configure.
   * @param on_time_us  Desired constant on-time in microseconds.
   *
   * Writes TON.TON_MANT and sets CTRL_INT_THRESH.PERIOD_MANT=0 which
   * disables the ICC integrator update path.
   */
  [[nodiscard]] DriverResult<void> SetManualOnTimeMode(Channel channel,
                                                       float on_time_us) noexcept;

  /**
   * @brief Seed the ICC integrator from the current feedback reading.
   *
   * Reads FB_INT_THRESH and writes it back to CTRL_INT_THRESH.INT_THRESH
   * so that a subsequent setpoint change doesn't wind up the integrator.
   */
  [[nodiscard]] DriverResult<void> SeedIntegratorThresholdFromFeedback(Channel channel) noexcept;

  //==========================================================================
  // PHASE 4 - DITHER COMPLETENESS (D15, D18)
  //==========================================================================

  /**
   * @brief High-level dither configuration (superset of ConfigureDither).
   *
   * Programs DITHER_CLK_DIV + DITHER_STEP + DITHER_CTRL atomically to
   * realize the requested amplitude/frequency and sync/fast-measure
   * options. Clears any residual deep-dither / fast-meas flags.
   */
  [[nodiscard]] DriverResult<void> SetDitherAdvanced(Channel channel,
                                                     const DitherSetup& setup,
                                                     bool parallel_mode = false) noexcept;

  //==========================================================================
  // PHASE 5 - DIAGNOSTICS COMPLETENESS (D17, D19, D20, D21, D22, D23)
  //==========================================================================

  /**
   * @brief Configure open-load/short-to-ground watchdog timeout.
   *
   * @param channel             Channel to configure.
   * @param olsg_timeout_code   6-bit OLSG timeout value (0\u201363).
   */
  [[nodiscard]] DriverResult<void> SetOlsgTimeout(Channel channel,
                                                  uint8_t olsg_timeout_code) noexcept;

  /**
   * @brief Enable / disable per-channel off-state OC diagnosis and fixed OL threshold.
   *
   * @param channel            Channel to configure.
   * @param oc_diag_enabled    true to enable OC diagnosis while channel is OFF
   * @param ol_th_fixed        Fixed open-load threshold (0\u201363; 0 = disabled)
   */
  [[nodiscard]] DriverResult<void> SetOffStateDiagnostics(Channel channel,
                                                          bool oc_diag_enabled,
                                                          uint8_t ol_th_fixed) noexcept;

  /**
   * @brief Run the safe-state logic-BIST (SFF_BIST).
   */
  [[nodiscard]] DriverResult<BistResult> RunSffBist(uint32_t timeout_ms = 10U) noexcept;

  /**
   * @brief Read the input pin status register (PIN_STAT).
   */
  [[nodiscard]] DriverResult<PinStatus> ReadPinStatus() noexcept;

  /**
   * @brief Enable or mask a specific fault source from asserting FAULTN.
   */
  [[nodiscard]] DriverResult<void> SetFaultMask(MaskableFault fault, bool enable) noexcept;

  //==========================================================================
  // PHASE 6 - ATOMIC FEEDBACK READOUT (D24, D27, D28)
  //==========================================================================

  /**
   * @brief Read a coherent feedback snapshot for one channel.
   *
   * @details
   * Workflow (datasheet §4.10.5):
   *   1. Freeze feedback for this channel (`FB_FRZ`).
   *   2. Poll `FB_UPD` for the update bit for this channel.
   *   3. Read `FB_DC` / `FB_VBAT` / `FB_I_AVG` / `FB_IMIN_IMAX` /
   *      `FB_PERIOD_MIN_MAX` / `FB_INT_THRESH`.
   *   4. Release the freeze.
   *
   * `avg_current_mA` is decoded from `FB_I_AVG` + `FB_DC` via the mantissa
   * ratio (`FB_FEEDBACK::ComputeAverageCurrent_mA`). It does not gate on
   * `HasValidMeasurementWindow()` — callers must interpret near-zero
   * `TP_MANT` themselves or prefer `GetAverageCurrent()` for explicit
   * `FeedbackNotReady` handling.
   *
   * @param channel     Channel to read.
   * @param timeout_ms  Maximum time to wait for `FB_UPD`.
   * @return Coherent `ChannelFeedback` snapshot or error.
   * @retval DriverError::TimeoutError Update bit did not assert in time.
   */
  [[nodiscard]] DriverResult<ChannelFeedback> ReadChannelFeedback(Channel channel,
                                                                  uint32_t timeout_ms = 10U) noexcept;

  /**
   * @brief Read coherent feedback for all six channels in a single freeze window.
   *
   * @param timeout_ms Maximum time to wait for each channel's `FB_UPD` poll.
   * @return Array of six `ChannelFeedback` structures or error.
   */
  [[nodiscard]] DriverResult<std::array<ChannelFeedback, 6>>
  ReadAllChannelFeedback(uint32_t timeout_ms = 20U) noexcept;

  /**
   * @brief Calibration-grade signed average current from `FB_I_AVG_s16`.
   *
   * @details
   * Reads the per-channel `FB_I_AVG_s16` register (offset 0x0204) and
   * decodes the 17-bit signed field via `FB_I_AVG_s16::ToMilliamps()`.
   * This path is distinct from `GetAverageCurrent()`, which uses the
   * compressed `FB_I_AVG` mantissa ratio with `TP_MANT` from `FB_DC`.
   *
   * @param channel Channel to query.
   * @return Signed average current in mA, or error.
   *
   * @note `FB_I_AVG_s16` is intended for calibration workflows. The datasheet
   *       notes the frame may not be valid while the channel is still settling,
   *       so this returns `DriverError::FeedbackNotReady` when the register
   *       still holds the mid-scale placeholder
   *       (@ref FB_I_AVG_s16::IsSettlingFrame).
   *
   * @warning That placeholder check is the only validity gate here. Unlike
   *          @ref GetAverageCurrent() this accessor does not confirm the
   *          channel has an established measurement window, so prefer
   *          `GetAverageCurrent()` for control and telemetry and keep this one
   *          for calibration.
   *
   * @see FB_I_AVG_s16, GetAverageCurrent()
   */
  [[nodiscard]] DriverResult<int32_t>
  GetCalibrationAvgCurrent_mA(Channel channel) noexcept;

  //==========================================================================
  // BATCHED NON-BLOCKING FEEDBACK (real-time control path)
  //==========================================================================

  /**
   * @brief Read FB_UPD — bit N set when channel N has a completed window.
   *
   * @details One central 16-bit read covers all six channels (datasheet
   * §5.3.2.10). This is the chip's own "is the data fresh" signal and the
   * first step of the Figure 22 readout; polling it beats guessing at Tmeas
   * with a wall clock, and it costs one register access for the whole device.
   */
  [[nodiscard]] DriverResult<uint8_t> GetFeedbackUpdateMask() noexcept;

  /**
   * @brief Set FB_FRZ to @p channel_mask (bit N freezes channel N).
   * @note Freezing holds FB_DC, FB_VBAT, FB_I_AVG, FB_PERIOD_MIN_MAX and
   *       FB_IMIN_IMAX. FB_I_AVG_s16 is free-running and unaffected (§4.10.5).
   */
  [[nodiscard]] DriverResult<void> FreezeFeedback(uint8_t channel_mask) noexcept;

  /**
   * @brief Clear FB_FRZ for every channel, resuming feedback updates.
   * @note Also clears the corresponding FB_UPD bits (§4.10.5).
   */
  [[nodiscard]] DriverResult<void> ReleaseFeedbackFreeze() noexcept;

  /**
   * @brief Coherent average current for several channels in one freeze window.
   *
   * @param channel_mask Bit N requests channel N (bits above 5 ignored).
   * @return Batch with per-channel current and the masks describing what was
   *         actually usable. Never blocks and never polls.
   *
   * @details
   * Implements the datasheet Figure 22 readout for a *set* of channels rather
   * than one at a time:
   *   1. read FB_UPD once — which channels have a fresh window,
   *   2. freeze exactly those channels,
   *   3. one pipelined burst reading FB_DC + FB_I_AVG for each of them,
   *   4. release the freeze.
   *
   * That is four bus transactions regardless of how many channels are in the
   * mask, because step 3 is a single @c ReadMulti chain. Refreshing all six
   * channels costs 4 transactions / 20 frames instead of the 24 transactions /
   * 72 frames a per-channel loop needs.
   *
   * Channels whose FB_UPD bit is clear are skipped rather than reported as
   * failures — no new window has completed, so the caller's previous sample is
   * still the most recent truth.
   *
   * @warning Unlike @ref ReadChannelFeedback this does not wait for FB_UPD. It
   *          is meant to be called at the control-loop rate: whatever is ready
   *          is sampled, the rest is picked up next tick. That is what makes it
   *          usable from a 2 ms thread that also owns actuator writes.
   */
  [[nodiscard]] DriverResult<CurrentFeedbackBatch>
  SampleChannelCurrents(uint8_t channel_mask) noexcept;

  /**
   * @brief Latched error bits for all six channels.
   *
   * @details DIAG_ERR_CHGR0..2 each cover a channel pair, so this is one
   * pipelined burst of three reads for the whole device.
   *
   * @warning Do not merge this into the same chain as the 22-bit feedback
   *          registers. These are 16-bit replies; interleaving widths in one
   *          pipeline desynchronised the reply slots on the bench and left
   *          every isolated coil misclassified.
   */
  [[nodiscard]] DriverResult<void>
  ReadAllChannelErrorFlags(std::array<ChannelErrorFlags, 6>& out) noexcept;

  /**
   * @brief True when DITHER_CLK_DIV encodes a running measurement clock.
   *
   * @details The POR value is 0, which makes tref_clk — and therefore Tmeas —
   * zero, so FB_DC / FB_I_AVG / FB_VBAT never update. Confirming this before
   * trusting feedback is a chip invariant, not caller policy.
   */
  [[nodiscard]] DriverResult<bool> IsMeasurementClockRunning(Channel channel) noexcept;

  /**
   * @brief Configure dither and confirm the measurement clock took.
   *
   * @param channel Channel to configure.
   * @param amplitude_ma Dither amplitude; 0 skips dither programming.
   * @param frequency_hz Dither frequency.
   * @param max_attempts Configure/read-back attempts before giving up.
   * @return true when DITHER_CLK_DIV read back as running; false when it never
   *         confirmed (the caller decides whether to proceed anyway — the clock
   *         may still be running from a POR or an earlier write that this
   *         device's SPI read cannot confirm).
   */
  [[nodiscard]] DriverResult<bool> EnsureDitherRunning(Channel channel,
                                                       float amplitude_ma,
                                                       float frequency_hz,
                                                       uint8_t max_attempts = 3U) noexcept;

  /**
   * @brief Last CH_CTRL value this driver wrote (OP_MODE + EN_CHx).
   *
   * @details CH_CTRL reads back as 0x0000 on some parts/buses, so the driver
   * keeps an authoritative shadow. Callers must use this instead of keeping a
   * second shadow of their own.
   */
  [[nodiscard]] uint16_t GetChannelControlShadow() const noexcept { return ch_ctrl_cache_; }

  /**
   * @brief Choose which channels may pull FAULTN via FAULT_MASK0/1.
   *
   * @param channel_mask Bit N = channel N contributes to FAULTN.
   * @details Which channels are populated is product wiring, so the mask is an
   *          argument; how that maps onto FAULT_MASK0/1 bit positions is chip
   *          protocol and stays here.
   */
  [[nodiscard]] DriverResult<void> SetFaultContributionMask(uint8_t channel_mask) noexcept;

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
   * @brief Read several registers in one pipelined chain.
   *
   * @param[in]  addresses Register addresses (at most @c CommType::kMaxPipelinedReads).
   * @param[out] values    Decoded values; entries that failed validation are 0.
   * @param[out] valid_mask Bit @c i set when @p values[i] is trustworthy.
   *
   * @details N reads cost N+2 SPI frames here instead of 3N, because the
   * pipeline is opened once for the whole burst. Individual replies are still
   * validated independently, so a single corrupt frame costs one entry rather
   * than the batch.
   *
   * @warning Every address in one call must answer with the same reply width.
   *          Mixing 16-bit and 22-bit registers in a single chain desynchronised
   *          reply slots on the bench.
   */
  [[nodiscard]] DriverResult<void> ReadRegisterMulti(std::span<const uint16_t> addresses,
                                                     std::span<uint32_t> values,
                                                     uint16_t& valid_mask) noexcept;

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
   *
   * @details Delegates to `comm_.Read()` / `comm_.Write()`, each of which
   *          performs the chip's two-frame pipelined transaction via
   *          `TransferMulti()`.
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
   * @brief Check if a channel is the slave half of an enabled parallel pair.
   * Slave channels are CH2, CH3, CH5 when their pair is parallelized; most
   * per-channel writes are rejected on the slave.
   */
  [[nodiscard]] DriverResult<bool> isSlaveChannel(Channel channel) noexcept;

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
  uint16_t cached_icvid_{0U}; ///< ICVID latched at VerifyDevice (live re-read may flake)
  std::array<uint16_t, 6> channel_setpoints_; ///< Cached current setpoints
};

// Include template implementation (must be inside namespace before it closes)
#define TLE92466ED_HEADER_INCLUDED
// NOLINTNEXTLINE(bugprone-suspicious-include) - Intentional: template implementation file
#include "../src/tle92466ed.ipp"
#undef TLE92466ED_HEADER_INCLUDED

// Public API: Get driver version string
/** @brief Return the compiled driver version string (same as `Driver::GetDriverVersion()`). */
inline const char* GetDriverVersion() noexcept {
  return HF_TLE92466ED_VERSION_STRING;
}

} // namespace tle92466ed
