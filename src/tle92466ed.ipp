/**
 * @file tle92466ed.ipp
 * @brief Template implementation of TLE92466ED driver class
 * @copyright Copyright (c) 2024-2025 HardFOC. All rights reserved.
 */
#ifndef TLE92466ED_IMPL
#define TLE92466ED_IMPL

// Always include the header to get the template class definition
// The guard TLE92466ED_IMPL prevents this file from being processed twice
#ifdef TLE92466ED_HEADER_INCLUDED
// Already included from header - the class definition is available in the current context
// We're inside the namespace, so we can access the template class
#else
// Not included from header (shouldn't happen for template implementation)
#include "../inc/tle92466ed.hpp"
namespace tle92466ed {
#endif

// Note: When included from header, this file is processed inside namespace tle92466ed
//       When compiled directly (shouldn't happen), we open the namespace here

//==============================================================================
// INITIALIZATION
//==============================================================================

template <typename CommType>
DriverResult<void> Driver<CommType>::Init() noexcept {
  // 1. Initialize CommInterface (GPIO and SPI bus only)
  if (auto result = comm_.Init(); !result) {
    return tle::unexpected(DriverError::HardwareError);
  }

  // 2. Perform device reset sequence
  // RESN is active low: LOW = reset, HIGH = normal operation
  // EN is active high: HIGH = enabled, LOW = disabled
  // We keep EN disabled during initialization - user must explicitly enable
  comm_.Log(LogLevel::Info, "TLE92466ED", "Performing device reset sequence...");

  // Step 1: Ensure EN is LOW (disabled) during reset
  if (auto result = SetEnable(false); !result) {
    comm_.Log(LogLevel::Warn, "TLE92466ED",
              "Failed to set EN pin LOW (error: %u) - continuing anyway",
              static_cast<unsigned>(result.error()));
  }

  // Step 2: Hold device in reset (LOW)
  if (auto result = SetReset(true); !result) {
    comm_.Log(LogLevel::Error, "TLE92466ED", "Failed to hold device in reset (error: %u)",
              static_cast<unsigned>(result.error()));
    return tle::unexpected(DriverError::HardwareError);
  }
  comm_.Log(LogLevel::Info, "TLE92466ED", "  RESN set LOW (device in reset)");

  // Step 3: Wait for reset pulse duration (minimum 10ms per datasheet)
  if (auto result = comm_.Delay(10000); !result) { // 10ms = 10000 microseconds
    return tle::unexpected(DriverError::HardwareError);
  }

  // Step 4: Release reset (HIGH)
  if (auto result = SetReset(false); !result) {
    comm_.Log(LogLevel::Error, "TLE92466ED", "Failed to release device from reset (error: %u)",
              static_cast<unsigned>(result.error()));
    return tle::unexpected(DriverError::HardwareError);
  }
  comm_.Log(LogLevel::Info, "TLE92466ED", "  RESN set HIGH (device released from reset)");

  // Step 5: Wait for device to stabilize after reset release (minimum 10ms per datasheet)
  if (auto result = comm_.Delay(10000); !result) { // 10ms = 10000 microseconds
    return tle::unexpected(DriverError::HardwareError);
  }

  comm_.Log(LogLevel::Info, "TLE92466ED",
            "✅ Device reset sequence completed (EN remains disabled)");

  // 3. Read and diagnose CLK_DIV register to check clock configuration
  // This helps diagnose clock-related critical faults early
  diagnoseClockConfiguration();

  // 4. Verify device communication by reading IC version
  auto verify_result = VerifyDevice();
  if (!verify_result) {
    return tle::unexpected(verify_result.error());
  }
  if (!*verify_result) {
    return tle::unexpected(DriverError::WrongDeviceID);
  }

  // 5. Device starts in Config Mode after power-up
  mission_mode_ = false;

  // 5a. Fail-fast on uncalibrated / virgin silicon. Per datasheet \u00a75.3.2.11
  //     p.66, GLOBAL_DIAG2.OTP_VIRGIN=1 means the device OTP trim is not
  //     programmed, so all ICC current-regulation math is invalid. Refuse to
  //     initialize so downstream code doesn't silently drive garbage currents.
  if (auto diag2_result = ReadRegister(CentralReg::GLOBAL_DIAG2); diag2_result) {
    if ((*diag2_result & GLOBAL_DIAG2::OTP_VIRGIN) != 0) {
      comm_.Log(LogLevel::Error, "TLE92466ED",
                "OTP_VIRGIN=1 \u2014 device OTP trim not programmed; refusing to initialize");
      return tle::unexpected(DriverError::ConfigurationError);
    }
  }

  // 6. Apply default configuration
  if (auto result = applyDefaultConfig(); !result) {
    return tle::unexpected(result.error());
  }

  // 7. Clear any power-on reset flags (skip initialization check during Init)
  if (auto result = clearFaultsInternal(); !result) {
    return tle::unexpected(result.error());
  }

  // 8. Initialize cached state
  ch_ctrl_cache_ = 0; // CH_CTRL cache (reads return 0x0000, so we track state here)
  channel_enable_cache_ = 0;
  vio_5v_mode_ = false; // Default to 3.3V mode (VIO_SEL=0)
  channel_setpoints_.fill(0);
  crc_enabled_ = false; // CRC starts disabled until user explicitly enables it

  initialized_ = true;
  return {};
}

template <typename CommType>
DriverResult<void> Driver<CommType>::applyDefaultConfig() noexcept {
  // Configure GLOBAL_CONFIG: Enable CRC and clock watchdog
  // Note: SPI watchdog is DISABLED by default because it requires periodic reloading
  //       If enabled without periodic reload, the device will timeout and enter Config Mode
  //       User should enable SPI watchdog only if they can guarantee periodic reloading
  // Note: VIO_SEL is NOT set (defaults to 0 = 3.3V mode) to match typical use case
  // If user needs 5V mode, they should call ConfigureGlobal() with vio_5v=true
  uint16_t global_cfg =
      GLOBAL_CONFIG::CRC_EN |
      // GLOBAL_CONFIG::SPI_WD_EN |  // Disabled by default - requires periodic reload
      GLOBAL_CONFIG::CLK_WD_EN;
  // VIO_SEL = 0 (3.3V mode) - bit 14 is NOT set, ensuring 3.3V mode
  // This prevents false VIO undervoltage faults when using 3.3V supply
  // Note: VIO thresholds are FIXED hardware values (not programmable)
  //       We can only select 3.3V or 5V mode via VIO_SEL bit
  //       - 3.3V mode: UV=2.6-3.0V, OV=3.6-4.1V (typical: 2.8V, 3.85V)
  //       - 5V mode: UV=3.7-4.5V, OV=5.5-6.4V (typical: 4.1V, 5.95V)

  if (auto result = WriteRegister(CentralReg::GLOBAL_CONFIG, global_cfg, false); !result) {
    return tle::unexpected(result.error());
  }

  // Update internal CRC enable state (CRC_EN is enabled in default config)
  crc_enabled_ = true;

  // Set default VBAT thresholds (UV=7V, OV=40V)
  // Use internal version that doesn't check initialization (called during Init)
  if (auto result = setVbatThresholdsInternal(7.0f, 40.0f); !result) {
    return tle::unexpected(result.error());
  }

  // Configure all channels with default settings (ICC mode, 1V/us slew, disabled)
  for (uint8_t ch = 0; ch < static_cast<uint8_t>(Channel::COUNT); ++ch) {
    auto channel = static_cast<Channel>(ch);
    uint16_t ch_base = GetChannelBase(channel);

    // Set mode to ICC (0x0001)
    if (auto result = WriteRegister(ch_base + ChannelReg::MODE,
                                    static_cast<uint16_t>(ChannelMode::ICC), false);
        !result) {
      return tle::unexpected(result.error());
    }

    // Set default CH_CONFIG (slew rate 2.5V/us, OL disabled)
    if (auto result =
            WriteRegister(ch_base + ChannelReg::CH_CONFIG, CH_CONFIG::SLEWR_2V5_US, false);
        !result) {
      return tle::unexpected(result.error());
    }

    // Set current setpoint to 0
    if (auto result = WriteRegister(ch_base + ChannelReg::SETPOINT, 0, false); !result) {
      return tle::unexpected(result.error());
    }
  }

  // Note: SPI watchdog is disabled by default, so no need to reload here
  // If user enables SPI watchdog via ConfigureGlobal(), they must call ReloadSpiWatchdog()
  // periodically

  return {};
}

//==========================================================================
// MODE CONTROL
//==========================================================================

template <typename CommType>
DriverResult<void> Driver<CommType>::EnterMissionMode() noexcept {
  if (auto result = checkInitialized(); !result) {
    return result;
  }

  comm_.Log(LogLevel::Info, "TLE92466ED", "Entering Mission Mode");

  // Set OP_MODE bit in CH_CTRL register
  // Note: CH_CTRL reads return 0x0000, so we use cached value
  ch_ctrl_cache_ |= CH_CTRL::OP_MODE;
  if (auto result = WriteRegister(CentralReg::CH_CTRL, ch_ctrl_cache_, false, false); !result) {
    return tle::unexpected(result.error());
  }

  mission_mode_ = true;
  comm_.Log(LogLevel::Info, "TLE92466ED", "✅ Mission Mode entered");
  return {};
}

template <typename CommType>
DriverResult<void> Driver<CommType>::EnterConfigMode() noexcept {
  if (auto result = checkInitialized(); !result) {
    return result;
  }

  comm_.Log(LogLevel::Info, "TLE92466ED", "Entering Config Mode");

  // Clear OP_MODE bit in CH_CTRL register
  // Note: CH_CTRL reads return 0x0000, so we use cached value
  ch_ctrl_cache_ &= ~CH_CTRL::OP_MODE;
  if (auto result = WriteRegister(CentralReg::CH_CTRL, ch_ctrl_cache_, false, false); !result) {
    return tle::unexpected(result.error());
  }

  mission_mode_ = false;
  comm_.Log(LogLevel::Info, "TLE92466ED", "✅ Config Mode entered");
  return {};
}

//==========================================================================
// GLOBAL CONFIGURATION
//==========================================================================

template <typename CommType>
DriverResult<void> Driver<CommType>::ConfigureGlobal(const GlobalConfig& config) noexcept {
  if (auto result = checkInitialized(); !result) {
    return result;
  }

  // Must be in config mode to change global configuration
  if (auto result = checkConfigMode(); !result) {
    return result;
  }

  comm_.Log(LogLevel::Info, "TLE92466ED",
            "Configuring global settings: CRC=%s, SPI_WD=%s, CLK_WD=%s, VIO_5V=%s, "
            "UV=%.2fV, OV=%.2fV, WD_Reload=%u",
            config.crc_enabled ? "enabled" : "disabled",
            config.spi_watchdog_enabled ? "enabled" : "disabled",
            config.clock_watchdog_enabled ? "enabled" : "disabled",
            config.vio_5v ? "true" : "false", config.vbat_uv_voltage, config.vbat_ov_voltage,
            config.spi_watchdog_reload);

  // Check if VIO_SEL is changing (use internal tracking since GLOBAL_CONFIG is write-only)
  // When VIO_SEL changes, VIO fault thresholds change, so we should clear VIO fault flags
  bool vio_sel_changed = (vio_5v_mode_ != config.vio_5v);

  // Build GLOBAL_CONFIG register value
  uint16_t global_cfg = 0;
  if (config.clock_watchdog_enabled) {
    global_cfg |= GLOBAL_CONFIG::CLK_WD_EN;
  }
  if (config.spi_watchdog_enabled) {
    global_cfg |= GLOBAL_CONFIG::SPI_WD_EN;
  }
  if (config.crc_enabled) {
    global_cfg |= GLOBAL_CONFIG::CRC_EN;
  }
  if (config.vio_5v) {
    global_cfg |= GLOBAL_CONFIG::VIO_SEL;
  }

  if (auto result = WriteRegister(CentralReg::GLOBAL_CONFIG, global_cfg); !result) {
    return tle::unexpected(result.error());
  }

  // Update internal CRC enable state
  crc_enabled_ = config.crc_enabled;

  // Update internal VIO mode tracking
  vio_5v_mode_ = config.vio_5v;

  // Clear VIO fault flags when VIO_SEL changes (thresholds change, old fault state invalid)
  if (vio_sel_changed) {
    comm_.Log(LogLevel::Info, "TLE92466ED", "VIO_SEL changed, clearing VIO fault flags");
    // Write 1 to clear VIO_UV and VIO_OV bits in GLOBAL_DIAG0
    if (auto result = WriteRegister(CentralReg::GLOBAL_DIAG0,
                                    GLOBAL_DIAG0::VIO_UV | GLOBAL_DIAG0::VIO_OV, false);
        !result) {
      comm_.Log(LogLevel::Warn, "TLE92466ED",
                "Failed to clear VIO fault flags after VIO_SEL change");
      // Don't fail the operation, just log warning
    }
  }

  // Configure VBAT thresholds
  if (auto result = SetVbatThresholds(config.vbat_uv_voltage, config.vbat_ov_voltage); !result) {
    return tle::unexpected(result.error());
  }

  // Configure SPI watchdog reload (mask to 11-bit field)
  if (config.spi_watchdog_enabled) {
    uint16_t wd_reload_value = WD_RELOAD::MaskValue(config.spi_watchdog_reload);
    if (auto result = WriteRegister(CentralReg::WD_RELOAD, wd_reload_value); !result) {
      return tle::unexpected(result.error());
    }
  }

  return {};
}

template <typename CommType>
DriverResult<void> Driver<CommType>::SetCrcEnabled(bool enabled) noexcept {
  if (auto result = checkInitialized(); !result) {
    return result;
  }

  comm_.Log(LogLevel::Info, "TLE92466ED", "Setting CRC enabled: %s", enabled ? "true" : "false");

  auto result = ModifyRegister(CentralReg::GLOBAL_CONFIG, GLOBAL_CONFIG::CRC_EN,
                               enabled ? GLOBAL_CONFIG::CRC_EN : 0);

  if (result) {
    // Update internal CRC enable state only if register write succeeded
    crc_enabled_ = enabled;
    comm_.Log(LogLevel::Info, "TLE92466ED", "CRC enabled state updated: %s",
              enabled ? "true" : "false");
  }

  return result;
}

template <typename CommType>
DriverResult<void> Driver<CommType>::SetVbatThresholds(float uv_voltage, float ov_voltage) noexcept {
  if (auto result = checkInitialized(); !result) {
    return result;
  }

  comm_.Log(LogLevel::Info, "TLE92466ED", "Setting VBAT thresholds: UV=%.2fV, OV=%.2fV",
            uv_voltage, ov_voltage);

  return setVbatThresholdsInternal(uv_voltage, ov_voltage);
}

template <typename CommType>
DriverResult<void> Driver<CommType>::setVbatThresholdsInternal(float uv_voltage, float ov_voltage) noexcept {
  // Validate voltage range
  if (uv_voltage < 0.0F || uv_voltage > 41.4F || ov_voltage < 0.0F || ov_voltage > 41.4F) {
    return tle::unexpected(DriverError::InvalidParameter);
  }

  // Calculate register values from voltage
  uint8_t uv_threshold = VBAT_THRESHOLD::CalculateFromVoltage(uv_voltage);
  uint8_t ov_threshold = VBAT_THRESHOLD::CalculateFromVoltage(ov_voltage);

  // Check if calculation was successful (non-zero values)
  if (uv_threshold == 0 && uv_voltage > 0.0F) {
    return tle::unexpected(DriverError::InvalidParameter);
  }
  if (ov_threshold == 0 && ov_voltage > 0.0F) {
    return tle::unexpected(DriverError::InvalidParameter);
  }

  uint16_t value = (static_cast<uint16_t>(ov_threshold) << 8) | uv_threshold;
  if (auto result = WriteRegister(CentralReg::VBAT_TH, value, false); !result) {
    return result; // Don't verify CRC during init
  }

  // Clear VBAT fault flags when thresholds change (old fault state is no longer valid)
  // Write 1 to clear VBAT_UV and VBAT_OV bits in GLOBAL_DIAG0
  if (auto result = WriteRegister(CentralReg::GLOBAL_DIAG0,
                                  GLOBAL_DIAG0::VBAT_UV | GLOBAL_DIAG0::VBAT_OV, false);
      !result) {
    comm_.Log(LogLevel::Warn, "TLE92466ED",
              "Failed to clear VBAT fault flags after threshold change");
    // Don't fail the operation, just log warning
  }

  return {};
}

template <typename CommType>
DriverResult<void> Driver<CommType>::SetVbatThresholdsRaw(uint8_t uv_threshold,
                                                uint8_t ov_threshold) noexcept {
  if (auto result = checkInitialized(); !result) {
    return result;
  }

  float uv_voltage = VBAT_THRESHOLD::CalculateVoltage(uv_threshold);
  float ov_voltage = VBAT_THRESHOLD::CalculateVoltage(ov_threshold);
  comm_.Log(LogLevel::Info, "TLE92466ED",
            "Setting VBAT thresholds (raw): UV_reg=%u (%.2fV), OV_reg=%u (%.2fV)", uv_threshold,
            uv_voltage, ov_threshold, ov_voltage);

  uint16_t value = (static_cast<uint16_t>(ov_threshold) << 8) | uv_threshold;
  return WriteRegister(CentralReg::VBAT_TH, value);
}

//==========================================================================
// CHANNEL CONTROL
//==========================================================================

template <typename CommType>
DriverResult<void> Driver<CommType>::EnableChannel(Channel channel, bool enabled) noexcept {
  if (auto result = checkInitialized(); !result) {
    return result;
  }

  // Channel enable can only be changed in Mission Mode
  if (auto result = checkMissionMode(); !result) {
    comm_.Log(LogLevel::Error, "TLE92466ED",
              "Cannot enable/disable channel: Device must be in Mission Mode (currently in Config "
              "Mode). Call EnterMissionMode() first.");
    return result;
  }

  if (!isValidChannelInternal(channel)) {
    return tle::unexpected(DriverError::InvalidChannel);
  }

  comm_.Log(LogLevel::Info, "TLE92466ED", "Set channel enable: Channel=%s, enabled=%s",
            ToString(channel), enabled ? "true" : "false");

  uint16_t mask = CH_CTRL::ChannelMask(ToIndex(channel));

  if (enabled) {
    channel_enable_cache_ |= mask;
  } else {
    channel_enable_cache_ &= ~mask;
  }

  // Build full CH_CTRL value: preserve OP_MODE and parallel bits, update channel enable bits
  uint16_t ch_ctrl_value = ch_ctrl_cache_ & ~CH_CTRL::ALL_CH_MASK; // Clear channel bits
  ch_ctrl_value |= channel_enable_cache_; // Set channel enable bits from cache

  // CH_CTRL write verification is disabled because reads return 0x0000 (known device behavior)
  // We track state in ch_ctrl_cache_ and channel_enable_cache_ instead
  ch_ctrl_cache_ = ch_ctrl_value;
  return WriteRegister(CentralReg::CH_CTRL, ch_ctrl_value, false, false);
}

template <typename CommType>
DriverResult<void> Driver<CommType>::EnableChannels(uint8_t channel_mask) noexcept {
  if (auto result = checkInitialized(); !result) {
    return result;
  }

  if (auto result = checkMissionMode(); !result) {
    return result;
  }

  // Mask to valid channels only (bits 0-5)
  channel_mask &= CH_CTRL::ALL_CH_MASK;
  channel_enable_cache_ = channel_mask;

  comm_.Log(LogLevel::Info, "TLE92466ED", "Set channel enable mask: 0x%02X (", channel_mask);
  bool first = true;
  for (uint8_t ch = 0; ch < 6; ++ch) {
    if ((channel_mask & (1 << ch)) != 0) {
      if (!first) {
        comm_.Log(LogLevel::Info, "TLE92466ED", ", ");
      }
      comm_.Log(LogLevel::Info, "TLE92466ED", "%s", ToString(static_cast<Channel>(ch)));
      first = false;
    }
  }
  comm_.Log(LogLevel::Info, "TLE92466ED", ")");

  // Build full CH_CTRL value: preserve OP_MODE and parallel bits, update channel enable bits
  uint16_t ch_ctrl_value = ch_ctrl_cache_ & ~CH_CTRL::ALL_CH_MASK; // Clear channel bits
  ch_ctrl_value |= channel_mask;                                   // Set new channel enable bits

  // CH_CTRL write verification is disabled because reads return 0x0000 (known device behavior)
  // We track state in ch_ctrl_cache_ and channel_enable_cache_ instead
  ch_ctrl_cache_ = ch_ctrl_value;
  return WriteRegister(CentralReg::CH_CTRL, ch_ctrl_value, false, false);
}

template <typename CommType>
DriverResult<void> Driver<CommType>::EnableAllChannels() noexcept {
  comm_.Log(LogLevel::Info, "TLE92466ED", "Enabling all channels");
  return EnableChannels(CH_CTRL::ALL_CH_MASK);
}

template <typename CommType>
DriverResult<void> Driver<CommType>::DisableAllChannels() noexcept {
  comm_.Log(LogLevel::Info, "TLE92466ED", "Disabling all channels");
  return EnableChannels(0);
}

template <typename CommType>
DriverResult<void> Driver<CommType>::SetChannelMode(Channel channel, ChannelMode mode) noexcept {
  if (auto result = checkInitialized(); !result) {
    return result;
  }

  // Mode can only be changed in Config Mode
  if (auto result = checkConfigMode(); !result) {
    return result;
  }

  if (!isValidChannelInternal(channel)) {
    return tle::unexpected(DriverError::InvalidChannel);
  }

  uint16_t ch_addr = GetChannelRegister(channel, ChannelReg::MODE);

  comm_.Log(LogLevel::Info, "TLE92466ED", "Setting channel mode: Channel=%s, Mode=%s (0x%04X)",
            ToString(channel), ToString(mode), static_cast<uint16_t>(mode));

  return WriteRegister(ch_addr, static_cast<uint16_t>(mode));
}

template <typename CommType>
DriverResult<void> Driver<CommType>::SetParallelOperation(ParallelPair pair, bool enabled) noexcept {
  if (auto result = checkInitialized(); !result) {
    return result;
  }

  // Parallel operation can only be changed in Config Mode
  if (auto result = checkConfigMode(); !result) {
    return result;
  }

  uint16_t mask = 0;
  switch (pair) {
  case ParallelPair::CH0_CH3:
    mask = CH_CTRL::CH_PAR_0_3;
    break;
  case ParallelPair::CH1_CH2:
    mask = CH_CTRL::CH_PAR_1_2;
    break;
  case ParallelPair::CH4_CH5:
    mask = CH_CTRL::CH_PAR_4_5;
    break;
  default:
    return tle::unexpected(DriverError::InvalidParameter);
  }

  comm_.Log(LogLevel::Info, "TLE92466ED", "Setting parallel operation: Pair=%s, Enabled=%s",
            ToString(pair), enabled ? "true" : "false");

  // Build full CH_CTRL value: preserve OP_MODE and channel enable bits, update parallel bits
  uint16_t ch_ctrl_value = ch_ctrl_cache_ & ~CH_CTRL::ALL_PAR_MASK; // Clear parallel bits
  if (enabled) {
    ch_ctrl_value |= mask; // Set parallel bit
  }
  // If disabled, parallel bit is already cleared

  // CH_CTRL write verification is disabled because reads return 0x0000 (known device behavior)
  // We track state in ch_ctrl_cache_ instead
  ch_ctrl_cache_ = ch_ctrl_value;
  return WriteRegister(CentralReg::CH_CTRL, ch_ctrl_value, false, false);
}

//==========================================================================
// CURRENT CONTROL
//==========================================================================

template <typename CommType>
DriverResult<void> Driver<CommType>::SetCurrentSetpoint(Channel channel, uint16_t current_ma,
                                              bool parallel_mode) noexcept {

  if (auto result = checkInitialized(); !result) {
    return result;
  }

  if (!isValidChannelInternal(channel)) {
    return tle::unexpected(DriverError::InvalidChannel);
  }

  // Validate current range (using absolute register scale)
  // Note: Datasheet typical continuous limits are ~1.5A single, ~2.7A parallel
  // but register scale allows up to 2A/4A for transient operation
  uint16_t max_current = parallel_mode ? 4000 : 2000;
  if (current_ma > max_current) {
    return tle::unexpected(DriverError::InvalidParameter);
  }

  // Calculate setpoint register value
  uint16_t target = SETPOINT::CalculateTarget(current_ma, parallel_mode);

  // Cache the setpoint
  channel_setpoints_[ToIndex(channel)] = target;

  // Write to SETPOINT register
  uint16_t ch_addr = GetChannelRegister(channel, ChannelReg::SETPOINT);

  comm_.Log(LogLevel::Info, "TLE92466ED",
            "Setting current setpoint: Channel=%s, Current=%u mA, Target=0x%04X, Parallel=%s",
            ToString(channel), current_ma, target, parallel_mode ? "true" : "false");

  return WriteRegister(ch_addr, target);
}

template <typename CommType>
DriverResult<uint16_t> Driver<CommType>::GetCurrentSetpoint(Channel channel, bool parallel_mode) noexcept {

  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }

  if (!isValidChannelInternal(channel)) {
    return tle::unexpected(DriverError::InvalidChannel);
  }

  // Read SETPOINT register
  uint16_t ch_addr = GetChannelRegister(channel, ChannelReg::SETPOINT);
  auto result = ReadRegister(ch_addr);
  if (!result) {
    return tle::unexpected(result.error());
  }

  // Convert to current in mA
  uint16_t target = *result & SETPOINT::TARGET_MASK;
  uint16_t current_ma = SETPOINT::CalculateCurrent(target, parallel_mode);

  return current_ma;
}

template <typename CommType>
DriverResult<void> Driver<CommType>::ConfigurePwmPeriod(Channel channel, float period_us) noexcept {

  if (auto result = checkInitialized(); !result) {
    return result;
  }

  if (!isValidChannelInternal(channel)) {
    return tle::unexpected(DriverError::InvalidChannel);
  }

  // Reject periods outside the chip's combined supported range
  // (250 µs – 72727 µs across the standard and low-frequency ranges).
  if (period_us < PERIOD::kSpecCombinedMinPeriod_us ||
      period_us > PERIOD::kSpecCombinedMaxPeriod_us) {
    comm_.Log(LogLevel::Error, "TLE92466ED",
              "ConfigurePwmPeriod: %.3f µs is outside chip-supported range "
              "[%.0f µs .. %.0f µs] (corresponds to %.0f Hz .. %.2f Hz)",
              static_cast<double>(period_us),
              static_cast<double>(PERIOD::kSpecCombinedMinPeriod_us),
              static_cast<double>(PERIOD::kSpecCombinedMaxPeriod_us),
              static_cast<double>(PERIOD::kSpecMaxFrequency_Hz),
              static_cast<double>(PERIOD::kSpecLowRangeMinFreq_Hz));
    return tle::unexpected(DriverError::InvalidParameter);
  }

  // Warn (but don't reject) for periods that fall in the gap between the
  // standard range max (9090 µs ≈ 110 Hz) and the low-range min (2000 µs
  // ≈ 500 Hz). The chip will pick whichever range the auto-encoder lands
  // in; both are within the gap and are still valid to write, just with
  // sub-spec frequency-control loop performance.
  if (period_us > PERIOD::kSpecMaxPeriod_us &&
      period_us < PERIOD::kSpecLowRangeMinPeriod_us) {
    comm_.Log(LogLevel::Warn, "TLE92466ED",
              "ConfigurePwmPeriod: %.3f µs falls in the gap between the "
              "standard (≤ %.0f µs) and low-frequency (≥ %.0f µs) ranges; "
              "PWM-control loop accuracy may be degraded",
              static_cast<double>(period_us),
              static_cast<double>(PERIOD::kSpecMaxPeriod_us),
              static_cast<double>(PERIOD::kSpecLowRangeMinPeriod_us));
  }

  // Calculate register values from desired period
  auto config = PERIOD::CalculateFromPeriodUs(period_us);

  // Check if calculation was successful (mantissa != 0)
  if (config.mantissa == 0) {
    comm_.Log(LogLevel::Error, "TLE92466ED",
              "ConfigurePwmPeriod: %.3f µs cannot be encoded into the "
              "PERIOD register (mantissa underflow)",
              static_cast<double>(period_us));
    return tle::unexpected(DriverError::InvalidParameter);
  }

  // Compute the actual period the chip will use after register quantization;
  // log it so the operator sees what they're really getting.
  const float actual_period_us = config.CalculatePeriodUs();
  const float actual_freq_hz   = 1.0e6F / actual_period_us;

  // Build PERIOD register value
  uint16_t value = PERIOD::BuildRegisterValue(config);

  comm_.Log(LogLevel::Info, "TLE92466ED",
            "Configuring PWM period: Channel=%s, Requested=%.3f µs (%.1f Hz), "
            "Actual=%.3f µs (%.1f Hz), Mantissa=%u, Exponent=%u, LowRange=%s, "
            "Register=0x%04X",
            ToString(channel),
            static_cast<double>(period_us),
            static_cast<double>(1.0e6F / period_us),
            static_cast<double>(actual_period_us),
            static_cast<double>(actual_freq_hz),
            config.mantissa, config.exponent,
            config.low_freq_range ? "true" : "false", value);

  uint16_t ch_addr = GetChannelRegister(channel, ChannelReg::PERIOD);
  return WriteRegister(ch_addr, value);
}

template <typename CommType>
DriverResult<void> Driver<CommType>::ConfigurePwmPeriodRaw(Channel channel, uint8_t period_mantissa,
                                                 uint8_t period_exponent,
                                                 bool low_freq_range) noexcept {

  if (auto result = checkInitialized(); !result) {
    return result;
  }

  if (!isValidChannelInternal(channel)) {
    return tle::unexpected(DriverError::InvalidChannel);
  }

  // Build PERIOD register value
  uint16_t value = period_mantissa |
                   ((period_exponent & PERIOD::EXP_VALUE_MASK) << PERIOD::EXP_SHIFT) |
                   (low_freq_range ? PERIOD::LOW_FREQ_BIT : 0);

  comm_.Log(LogLevel::Info, "TLE92466ED",
            "Configuring PWM period (raw): Channel=%s, Mantissa=%u, Exponent=%u, "
            "LowFreq=%s, Register=0x%04X",
            ToString(channel), period_mantissa, period_exponent, low_freq_range ? "true" : "false",
            value);

  uint16_t ch_addr = GetChannelRegister(channel, ChannelReg::PERIOD);
  return WriteRegister(ch_addr, value);
}

template <typename CommType>
DriverResult<void> Driver<CommType>::ConfigureDitherClock(Channel channel,
                                                          float t_ref_clk_us,
                                                          bool dither_pwm_sync,
                                                          bool dither_setpoint_sync) noexcept {
  if (auto result = checkInitialized(); !result) {
    return result;
  }
  if (!isValidChannelInternal(channel)) {
    return tle::unexpected(DriverError::InvalidChannel);
  }
  if (t_ref_clk_us <= 0.0F) {
    comm_.Log(LogLevel::Error, "TLE92466ED",
              "ConfigureDitherClock: t_ref_clk_us must be > 0 (got %.3f)",
              static_cast<double>(t_ref_clk_us));
    return tle::unexpected(DriverError::InvalidParameter);
  }

  auto cfg = DITHER_CLK_DIV::CalculateFromTrefClkUs(t_ref_clk_us);
  cfg.dither_pwm_sync_en      = dither_pwm_sync;
  cfg.dither_setpoint_sync_en = dither_setpoint_sync;

  const float    actual_us = cfg.CalculateTrefClkUs();
  const uint16_t reg_value = cfg.ToRegister();

  comm_.Log(LogLevel::Info, "TLE92466ED",
            "Configuring dither clock: Channel=%s, Requested=%.3f µs, "
            "Actual=%.3f µs, MANT=%u, EXP=%u, PwmSync=%s, SetpointSync=%s, "
            "Register=0x%04X",
            ToString(channel),
            static_cast<double>(t_ref_clk_us),
            static_cast<double>(actual_us),
            cfg.mantissa, cfg.exponent,
            dither_pwm_sync       ? "true" : "false",
            dither_setpoint_sync  ? "true" : "false",
            reg_value);

  const uint16_t addr = GetChannelRegister(channel, ChannelReg::DITHER_CLK_DIV);
  return WriteRegister(addr, reg_value);
}

template <typename CommType>
DriverResult<void> Driver<CommType>::ConfigureDither(Channel channel, float amplitude_ma, float frequency_hz,
                                           bool parallel_mode) noexcept {

  if (auto result = checkInitialized(); !result) {
    return result;
  }

  if (!isValidChannelInternal(channel)) {
    return tle::unexpected(DriverError::InvalidChannel);
  }

  // Validate parameters
  if (amplitude_ma < 0.0F || frequency_hz <= 0.0F) {
    return tle::unexpected(DriverError::InvalidParameter);
  }

  // Check if channel is in parallel mode (if not specified, detect it)
  if (!parallel_mode) {
    auto parallel_result = isChannelParallel(channel);
    if (parallel_result) {
      parallel_mode = *parallel_result;
    }
  }

  //
  // Pick a tref_clk that lands on the requested dither frequency with
  // fixed STEPS=16 / FLAT=2 sub-cycle counts (= 4×16 + 2×2 = 68 sub-
  // cycles per dither period), then write DITHER_CLK_DIV before
  // computing the amplitude scaling. Per datasheet §5.3.3.7 the chip's
  // POR default of DITHER_CLK_DIV = 0x0000 leaves tref_clk = 0 and the
  // averaged-feedback engine disabled; without this register being
  // programmed, FB_DC / FB_I_AVG / FB_VBAT stay at 0 forever even
  // though the chip is happily driving the load.
  //
  // Use a fixed STEPS/FLAT pair here so that:
  //   1) tref_clk and the dither registers agree on what TDither is
  //      (the helper's auto-adjust would land on STEPS=255 for low
  //      frequencies, blowing up the measurement period);
  //   2) the resulting Tmeas is a sensible 16 PWM cycles' worth at
  //      typical PWM frequencies (a couple of ms), giving the chip's
  //      averager enough samples for stable readings without making
  //      telemetry update too slowly for a control loop.
  //
  constexpr uint8_t kDitherSteps = 16;
  constexpr uint8_t kDitherFlat  = 2;
  constexpr float   kPeriodUnits =
      4.0F * static_cast<float>(kDitherSteps) +
      2.0F * static_cast<float>(kDitherFlat);     // 68 sub-cycles
  const float dither_period_us = 1'000'000.0F / frequency_hz;
  const float t_ref_clk_us     = dither_period_us / kPeriodUnits;

  if (auto result = ConfigureDitherClock(channel, t_ref_clk_us); !result) {
    return result;
  }

  // Compute step_size from the requested amplitude using the FIXED steps
  // count above (not the helper's adjustable one):
  //   I_dither = STEPS × STEP_SIZE × max_current / 32767
  //  ⇒ STEP_SIZE = amplitude_ma × 32767 / (STEPS × max_current)
  const uint32_t max_current_ma = parallel_mode ? 4000U : 2000U;
  const float step_size_f =
      (amplitude_ma * 32767.0F) /
      (static_cast<float>(kDitherSteps) * static_cast<float>(max_current_ma));
  uint16_t step_size = static_cast<uint16_t>(std::lround(step_size_f));
  if (step_size > DITHER_CTRL::STEP_SIZE_MASK) {
    step_size = DITHER_CTRL::STEP_SIZE_MASK;
  }
  // For amplitudes that round down to 0 LSB, force a 1 so dither stays
  // ON (the chip needs STEP_SIZE > 0 for the averager to run; even
  // STEP_SIZE = 1 produces sub-mA current excursion that's typically
  // negligible against the load and quantization noise).
  if (step_size == 0 && amplitude_ma > 0.0F) {
    step_size = 1;
  }

  comm_.Log(LogLevel::Info, "TLE92466ED",
            "Configuring dither: Channel=%s, Amplitude=%.2f mA (~%.2f mA actual), "
            "Frequency=%.2f Hz, StepSize=%u, NumSteps=%u, FlatSteps=%u, "
            "Parallel=%s",
            ToString(channel),
            static_cast<double>(amplitude_ma),
            static_cast<double>(static_cast<float>(step_size) *
                                static_cast<float>(kDitherSteps) *
                                static_cast<float>(max_current_ma) / 32767.0F),
            static_cast<double>(frequency_hz),
            step_size, kDitherSteps, kDitherFlat,
            parallel_mode ? "true" : "false");

  return ConfigureDitherRaw(channel, step_size, kDitherSteps, kDitherFlat);
}

template <typename CommType>
DriverResult<void> Driver<CommType>::ConfigureDitherRaw(Channel channel, uint16_t step_size,
                                              uint8_t num_steps, uint8_t flat_steps) noexcept {

  if (auto result = checkInitialized(); !result) {
    return result;
  }

  if (!isValidChannelInternal(channel)) {
    return tle::unexpected(DriverError::InvalidChannel);
  }

  uint16_t ch_base = GetChannelBase(channel);

  comm_.Log(LogLevel::Info, "TLE92466ED",
            "Configuring dither (raw): Channel=%s, StepSize=%u, NumSteps=%u, FlatSteps=%u",
            ToString(channel), step_size, num_steps, flat_steps);

  // Configure DITHER_CTRL (step size)
  uint16_t ctrl_value = step_size & DITHER_CTRL::STEP_SIZE_MASK;
  if (auto result = WriteRegister(ch_base + ChannelReg::DITHER_CTRL, ctrl_value); !result) {
    return tle::unexpected(result.error());
  }

  // Configure DITHER_STEP (steps and flat period)
  uint16_t step_value = flat_steps | (static_cast<uint16_t>(num_steps) << DITHER_STEP::STEPS_SHIFT);
  if (auto result = WriteRegister(ch_base + ChannelReg::DITHER_STEP, step_value); !result) {
    return tle::unexpected(result.error());
  }

  return {};
}

template <typename CommType>
DriverResult<void> Driver<CommType>::ConfigureChannel(Channel channel, const ChannelConfig& config) noexcept {

  if (auto result = checkInitialized(); !result) {
    return result;
  }

  // Most configuration requires Config Mode
  if (auto result = checkConfigMode(); !result) {
    return result;
  }

  if (!isValidChannelInternal(channel)) {
    return tle::unexpected(DriverError::InvalidChannel);
  }

  comm_.Log(LogLevel::Info, "TLE92466ED",
            "Configuring channel: %s, Mode=%s, Current=%u mA, "
            "SlewRate=%s, DiagCurrent=%s, OL_Threshold=%u",
            ToString(channel), ToString(config.mode), config.current_setpoint_ma,
            ToString(config.slew_rate), ToString(config.diag_current), config.open_load_threshold);

  uint16_t ch_base = GetChannelBase(channel);

  // 1. Set channel mode
  if (auto result = WriteRegister(ch_base + ChannelReg::MODE, static_cast<uint16_t>(config.mode));
      !result) {
    return tle::unexpected(result.error());
  }

  // 2. Set current setpoint with parallel mode detection
  auto parallel_result = isChannelParallel(channel);
  bool is_parallel = parallel_result.value_or(false); // Default to false if can't determine
  uint16_t target = SETPOINT::CalculateTarget(config.current_setpoint_ma, is_parallel);
  if (config.auto_limit_disabled) {
    target |= SETPOINT::AUTO_LIMIT_DIS;
  }
  if (auto result = WriteRegister(ch_base + ChannelReg::SETPOINT, target); !result) {
    return tle::unexpected(result.error());
  }

  // 3. Configure CH_CONFIG register
  uint16_t ch_cfg = static_cast<uint16_t>(config.slew_rate) |
                    (static_cast<uint16_t>(config.diag_current) << 2) |
                    (static_cast<uint16_t>(config.open_load_threshold & CH_CONFIG::OL_TH_VALUE_MASK)
                     << CH_CONFIG::OL_TH_SHIFT);

  if (auto result = WriteRegister(ch_base + ChannelReg::CH_CONFIG, ch_cfg); !result) {
    return tle::unexpected(result.error());
  }

  // 3a. Configure OLSG warning enable if requested (bit 14 of CTRL register)
  if (config.olsg_warning_enabled) {
    if (auto result = ModifyRegister(ch_base + ChannelReg::CTRL, CH_CTRL_REG::OLSG_WARN_EN,
                                     CH_CTRL_REG::OLSG_WARN_EN);
        !result) {
      return tle::unexpected(result.error());
    }
  }

  // 4. Configure PWM if specified
  // Note: ChannelConfig still uses low-level parameters for backward compatibility
  // New code should use ConfigurePwmPeriod(period_us) directly
  if (config.pwm_period_mantissa > 0) {
    if (auto result = ConfigurePwmPeriodRaw(channel, config.pwm_period_mantissa,
                                            config.pwm_period_exponent, false);
        !result) {
      return tle::unexpected(result.error());
    }
  }

  // 5. Configure dither if specified
  // Note: ChannelConfig still uses low-level parameters for backward compatibility
  // New code should use ConfigureDither(amplitude_ma, frequency_hz) directly
  if (config.dither_step_size > 0) {
    if (auto result = ConfigureDitherRaw(channel, config.dither_step_size, config.dither_steps,
                                         config.dither_flat);
        !result) {
      return tle::unexpected(result.error());
    }

    // 5a. Enable deep dither if requested (bit 13 of DITHER_CTRL)
    if (config.deep_dither_enabled) {
      if (auto result = ModifyRegister(ch_base + ChannelReg::DITHER_CTRL, DITHER_CTRL::DEEP_DITHER,
                                       DITHER_CTRL::DEEP_DITHER);
          !result) {
        return tle::unexpected(result.error());
      }
    }
  }

  return {};
}

//==========================================================================
// STATUS AND DIAGNOSTICS
//==========================================================================

template <typename CommType>
DriverResult<DeviceStatus> Driver<CommType>::GetDeviceStatus() noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }

  DeviceStatus status{};

  // Read GLOBAL_DIAG0
  auto diag0_result = ReadRegister(CentralReg::GLOBAL_DIAG0);
  if (!diag0_result) {
    return tle::unexpected(diag0_result.error());
  }

  uint16_t diag0 = *diag0_result;
  status.vbat_uv = (diag0 & GLOBAL_DIAG0::VBAT_UV) != 0;
  status.vbat_ov = (diag0 & GLOBAL_DIAG0::VBAT_OV) != 0;
  status.vio_uv = (diag0 & GLOBAL_DIAG0::VIO_UV) != 0;
  status.vio_ov = (diag0 & GLOBAL_DIAG0::VIO_OV) != 0;
  status.vdd_uv = (diag0 & GLOBAL_DIAG0::VDD_UV) != 0;
  status.vdd_ov = (diag0 & GLOBAL_DIAG0::VDD_OV) != 0;
  status.clock_fault = (diag0 & GLOBAL_DIAG0::CLK_NOK) != 0;
  status.ot_error = (diag0 & GLOBAL_DIAG0::COTERR) != 0;
  status.ot_warning = (diag0 & GLOBAL_DIAG0::COTWARN) != 0;
  status.reset_event = (diag0 & GLOBAL_DIAG0::RES_EVENT) != 0;
  status.por_event = (diag0 & GLOBAL_DIAG0::POR_EVENT) != 0;
  status.spi_wd_error = (diag0 & GLOBAL_DIAG0::SPI_WD_ERR) != 0;

  status.any_fault = (diag0 & GLOBAL_DIAG0::FAULT_MASK) != 0;

  // Read FB_STAT for additional status
  auto fb_stat_result = ReadRegister(CentralReg::FB_STAT);
  if (fb_stat_result) {
    uint16_t fb_stat = *fb_stat_result;
    status.supply_nok_internal = (fb_stat & FB_STAT::SUP_NOK_INT) != 0;
    status.supply_nok_external = (fb_stat & FB_STAT::SUP_NOK_EXT) != 0;
    status.init_done = (fb_stat & FB_STAT::INIT_DONE) != 0;
  }

  // Read CH_CTRL to get mode
  auto ch_ctrl_result = ReadRegister(CentralReg::CH_CTRL);
  if (ch_ctrl_result) {
    status.config_mode = (*ch_ctrl_result & CH_CTRL::OP_MODE) == 0;
  }

  // Read voltage feedbacks
  // FB_VOLTAGE1 contains VIO and VDD (22-bit reply frame)
  auto fb_voltage1_result = ReadRegister(CentralReg::FB_VOLTAGE1);
  if (fb_voltage1_result) {
    status.vio_voltage = VOLTAGE_FEEDBACK::ExtractVioMillivolts(*fb_voltage1_result);
    // Note: VDD is also available in FB_VOLTAGE1 but not stored in DeviceStatus
  }

  // FB_VOLTAGE2 contains VBAT and temperature (22-bit reply frame)
  auto fb_voltage2_result = ReadRegister(CentralReg::FB_VOLTAGE2);
  if (fb_voltage2_result) {
    status.vbat_voltage = VOLTAGE_FEEDBACK::ExtractVbatMillivolts(*fb_voltage2_result);
  }

  return status;
}

template <typename CommType>
DriverResult<ChannelDiagnostics> Driver<CommType>::GetChannelDiagnostics(Channel channel) noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }

  if (!isValidChannelInternal(channel)) {
    return tle::unexpected(DriverError::InvalidChannel);
  }

  ChannelDiagnostics diag{};
  const uint8_t  ch_idx = ToIndex(channel);

  // ---- DIAG_ERR_CHGR (per-pair register, bits at 8*y offset) ----
  // Per datasheet \u00a75.3.2.12 p.67 there are only 3 groups (CHGR0..CHGR2),
  // each covering a channel pair. The correct bit layout for each channel is
  //   OLSG=0, OL=1, OC=2, SG=3, OTE=4 (all shifted by 8*y).
  const uint16_t diag_err_addr = DIAG_ERR_CHGR::AddressForChannel(ch_idx);
  auto diag_err_result = ReadRegister(diag_err_addr);
  if (diag_err_result) {
    const uint16_t diag_err = *diag_err_result;
    diag.open_load_short_ground =
        (diag_err & DIAG_ERR_CHGR::ChannelBitMask(ch_idx, DIAG_ERR_CHGR::BIT_OLSG)) != 0;
    diag.open_load =
        (diag_err & DIAG_ERR_CHGR::ChannelBitMask(ch_idx, DIAG_ERR_CHGR::BIT_OL))   != 0;
    diag.overcurrent =
        (diag_err & DIAG_ERR_CHGR::ChannelBitMask(ch_idx, DIAG_ERR_CHGR::BIT_OC))   != 0;
    diag.short_to_ground =
        (diag_err & DIAG_ERR_CHGR::ChannelBitMask(ch_idx, DIAG_ERR_CHGR::BIT_SG))   != 0;
    diag.over_temperature =
        (diag_err & DIAG_ERR_CHGR::ChannelBitMask(ch_idx, DIAG_ERR_CHGR::BIT_OTE))  != 0;
  }

  // ---- DIAG_WARN_CHGR (per-pair register, bits at 8*y offset) ----
  // Per datasheet \u00a75.3.2.13 p.68, bits are
  //   PWM_REG=0, I_REG=1, OTW=2, OLSG_WARN=3, OLSG_WARN_CHK_NOK=4.
  const uint16_t diag_warn_addr = DIAG_WARN_CHGR::AddressForChannel(ch_idx);
  auto diag_warn_result = ReadRegister(diag_warn_addr);
  if (diag_warn_result) {
    const uint16_t diag_warn = *diag_warn_result;
    diag.pwm_regulation_warning =
        (diag_warn & DIAG_WARN_CHGR::ChannelBitMask(ch_idx, DIAG_WARN_CHGR::BIT_PWM_REG))      != 0;
    diag.current_regulation_warning =
        (diag_warn & DIAG_WARN_CHGR::ChannelBitMask(ch_idx, DIAG_WARN_CHGR::BIT_I_REG))        != 0;
    diag.ot_warning =
        (diag_warn & DIAG_WARN_CHGR::ChannelBitMask(ch_idx, DIAG_WARN_CHGR::BIT_OTW))          != 0;
    diag.olsg_warning =
        (diag_warn & DIAG_WARN_CHGR::ChannelBitMask(ch_idx, DIAG_WARN_CHGR::BIT_OLSG_WARN))    != 0;
    diag.olsg_check_not_performed =
        (diag_warn & DIAG_WARN_CHGR::ChannelBitMask(ch_idx, DIAG_WARN_CHGR::BIT_OLSG_CHK_NOK)) != 0;
  }

  // ---- Raw feedback reads (full 22-bit decode lives in Phase 6) ----
  const uint16_t ch_base = GetChannelBase(channel);

  if (auto r = ReadRegister(ch_base + ChannelReg::FB_I_AVG); r) {
    diag.average_current = static_cast<uint16_t>(*r & 0xFFFFu);
  }
  if (auto r = ReadRegister(ch_base + ChannelReg::FB_DC); r) {
    diag.duty_cycle = static_cast<uint16_t>(*r & 0xFFFFu);
  }
  if (auto r = ReadRegister(ch_base + ChannelReg::FB_VBAT); r) {
    diag.vbat_feedback = static_cast<uint16_t>(*r & 0xFFFFu);
  }

  // FB_IMIN_IMAX decode: two 10-bit SIGNED fields, each scaled by 4000/511 mA.
  // Per datasheet \u00a75.3.3.17 p.101. The previous code used an 8-bit unsigned
  // split which produced nonsense values \u2014 fully replaced here.
  if (auto r = ReadRegister(ch_base + ChannelReg::FB_IMIN_IMAX); r) {
    const uint32_t minmax_raw = *r;
    const int32_t imin_ma = FB_IMIN_IMAX::IMin_mA(minmax_raw);
    const int32_t imax_ma = FB_IMIN_IMAX::IMax_mA(minmax_raw);
    // Clamp into int16_t (valid range ~\u00b14007 mA, comfortably inside int16_t).
    diag.min_current_mA = static_cast<int16_t>(
        std::max<int32_t>(-32768, std::min<int32_t>(32767, imin_ma)));
    diag.max_current_mA = static_cast<int16_t>(
        std::max<int32_t>(-32768, std::min<int32_t>(32767, imax_ma)));
  }

  return diag;
}

template <typename CommType>
DriverResult<uint16_t> Driver<CommType>::GetAverageCurrent(Channel channel, bool parallel_mode) noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }
  if (!isValidChannelInternal(channel)) {
    return tle::unexpected(DriverError::InvalidChannel);
  }
  (void)parallel_mode;  // mantissa-ratio formula is identical for parallel mode

  // Per datasheet §4.10.2:  Iavg = 4 A × <I_AVG_MANT> / <TP_MANT>
  // I_AVG_MANT lives in FB_I_AVG (22-bit reply, signed two's-complement)
  // TP_MANT lives in FB_DC (22-bit reply, unsigned)
  // Both registers must be read for the same channel to compute Iavg.
  const uint16_t i_avg_addr = GetChannelRegister(channel, ChannelReg::FB_I_AVG);
  const uint16_t fb_dc_addr = GetChannelRegister(channel, ChannelReg::FB_DC);

  auto i_avg_res = ReadRegister(i_avg_addr);
  if (!i_avg_res) return tle::unexpected(i_avg_res.error());
  auto fb_dc_res = ReadRegister(fb_dc_addr);
  if (!fb_dc_res) return tle::unexpected(fb_dc_res.error());

  // Decode mantissas and compute current in mA. Returns int32_t (signed)
  // so we can detect negative readings (recirculation current); clamp to
  // unsigned for the legacy uint16_t return type.
  const int32_t i_ma = FB_FEEDBACK::ComputeAverageCurrent_mA(*i_avg_res, *fb_dc_res);
  const int32_t i_clamped = (i_ma < 0) ? 0 : (i_ma > 0xFFFF ? 0xFFFF : i_ma);
  return static_cast<uint16_t>(i_clamped);
}

template <typename CommType>
DriverResult<uint16_t> Driver<CommType>::GetDutyCycle(Channel channel) noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }
  if (!isValidChannelInternal(channel)) {
    return tle::unexpected(DriverError::InvalidChannel);
  }

  // Per datasheet §4.10.2:  DC = <TO_MANT> / <TP_MANT>
  // Both fields live in FB_DC (22-bit reply). Returned as a permyriad
  // value (0..10000 = 0.0..100.0%) so the caller can pick the precision
  // they need without losing fractional duty in the legacy uint16_t API.
  const uint16_t addr = GetChannelRegister(channel, ChannelReg::FB_DC);
  auto fb_dc_res = ReadRegister(addr);
  if (!fb_dc_res) return tle::unexpected(fb_dc_res.error());

  const float dc = FB_FEEDBACK::ComputeDutyCycle(*fb_dc_res);  // 0.0 .. 1.0
  const uint16_t permyriad = static_cast<uint16_t>(dc * 10000.0f + 0.5f);
  return permyriad;
}

template <typename CommType>
DriverResult<uint16_t> Driver<CommType>::GetVbatVoltage() noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }

  // FB_VOLTAGE2 contains VBAT (22-bit reply frame)
  // VBAT is in bits [21:11], formula: V_BAT = 41.47 V × <VBAT>/2047
  auto result = ReadRegister(CentralReg::FB_VOLTAGE2);
  if (!result) {
    return tle::unexpected(result.error());
  }

  // Extract VBAT voltage in millivolts
  return VOLTAGE_FEEDBACK::ExtractVbatMillivolts(*result);
}

template <typename CommType>
DriverResult<uint16_t> Driver<CommType>::GetVioVoltage() noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }

  // FB_VOLTAGE1 contains VIO (22-bit reply frame)
  // VIO is in bits [10:0], formula: V_IO = 0.0034534 V × <VIO>
  auto result = ReadRegister(CentralReg::FB_VOLTAGE1);
  if (!result) {
    return tle::unexpected(result.error());
  }

  // Extract VIO voltage in millivolts
  return VOLTAGE_FEEDBACK::ExtractVioMillivolts(*result);
}

template <typename CommType>
DriverResult<uint16_t> Driver<CommType>::GetVddVoltage() noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }

  // FB_VOLTAGE1 contains VDD (22-bit reply frame)
  // VDD is in bits [21:11], formula: V_DD = 0.0034534 V × <VDD>
  auto result = ReadRegister(CentralReg::FB_VOLTAGE1);
  if (!result) {
    return tle::unexpected(result.error());
  }

  // Extract VDD voltage in millivolts
  return VOLTAGE_FEEDBACK::ExtractVddMillivolts(*result);
}

template <typename CommType>
DriverResult<void> Driver<CommType>::GetVbatThresholds(uint16_t& uv_threshold,
                                             uint16_t& ov_threshold) noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }

  // Read VBAT thresholds from VBAT_TH register
  auto vbat_th_result = ReadRegister(CentralReg::VBAT_TH);
  if (!vbat_th_result) {
    return tle::unexpected(vbat_th_result.error());
  }

  uint16_t vbat_th = *vbat_th_result;
  uint8_t uv_th = (vbat_th >> 0) & 0xFF;
  uint8_t ov_th = (vbat_th >> 8) & 0xFF;

  uv_threshold = static_cast<uint16_t>(std::lround(VBAT_THRESHOLD::CalculateVoltage(uv_th) * 1000.0F));
  ov_threshold = static_cast<uint16_t>(std::lround(VBAT_THRESHOLD::CalculateVoltage(ov_th) * 1000.0F));

  return {};
}

//==========================================================================
// FAULT MANAGEMENT
//==========================================================================

template <typename CommType>
DriverResult<void> Driver<CommType>::ClearFaults() noexcept {
  if (auto result = checkInitialized(); !result) {
    return result;
  }

  comm_.Log(LogLevel::Info, "TLE92466ED", "Clearing all fault flags");
  return clearFaultsInternal();
}

template <typename CommType>
DriverResult<void> Driver<CommType>::clearFaultsInternal() noexcept {
  // Write 1s to clear fault bits in GLOBAL_DIAG0 (rwh type - clear on write 1)
  // Note: Fault flags are latched. Writing 1 clears the latch, but if the underlying
  // condition still exists (or existed recently), the fault may be re-asserted immediately.
  // For voltage faults, the voltage must be within valid range for the fault to clear.
  // Some faults may have hysteresis (trigger at one voltage, clear at different voltage).
  if (auto result = WriteRegister(CentralReg::GLOBAL_DIAG0, GLOBAL_DIAG0::CLEAR_ALL); !result) {
    return tle::unexpected(result.error());
  }

  // Clear GLOBAL_DIAG1
  if (auto result = WriteRegister(CentralReg::GLOBAL_DIAG1, GLOBAL_DIAG1::CLEAR_ALL); !result) {
    return tle::unexpected(result.error());
  }

  // Clear GLOBAL_DIAG2. Per datasheet \u00a74.9.7, OTP_ECC_ERR requires TWO
  // consecutive write-1-to-clear cycles before the latched flag is released.
  // We always issue the second write unconditionally; it's a no-op for bits
  // that were cleared by the first write.
  if (auto result = WriteRegister(CentralReg::GLOBAL_DIAG2, GLOBAL_DIAG2::CLEAR_ALL); !result) {
    return tle::unexpected(result.error());
  }
  if (auto result = WriteRegister(CentralReg::GLOBAL_DIAG2, GLOBAL_DIAG2::OTP_ECC_ERR); !result) {
    return tle::unexpected(result.error());
  }

  // Clear per-pair DIAG_ERR_CHGRx registers (OLSG/OL/OC/SG/OTE).
  // Per datasheet \u00a74.9.6 the OC flag specifically needs TWO consecutive
  // write-1-to-clear cycles. The second pass covers both channels of every
  // pair via DIAG_ERR_CHGR::OC_BOTH_MASK.
  for (uint16_t g = 0; g < 3; ++g) {
    const uint16_t addr = static_cast<uint16_t>(CentralReg::DIAG_ERR_CHGR0 + g);
    if (auto result = WriteRegister(addr, 0xFFFFu); !result) {
      return tle::unexpected(result.error());
    }
    if (auto result = WriteRegister(addr, DIAG_ERR_CHGR::OC_BOTH_MASK); !result) {
      return tle::unexpected(result.error());
    }
  }

  // Clear per-pair DIAG_WARN_CHGRx registers.
  for (uint16_t g = 0; g < 3; ++g) {
    const uint16_t addr = static_cast<uint16_t>(CentralReg::DIAG_WARN_CHGR0 + g);
    if (auto result = WriteRegister(addr, 0xFFFFu); !result) {
      return tle::unexpected(result.error());
    }
  }

  return {};
}

template <typename CommType>
DriverResult<bool> Driver<CommType>::HasAnyFault() noexcept {
  auto status_result = GetDeviceStatus();
  if (!status_result) {
    return tle::unexpected(status_result.error());
  }

  return status_result->any_fault;
}

template <typename CommType>
DriverResult<FaultReport> Driver<CommType>::GetAllFaults() noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }

  FaultReport report{};

  // Read GLOBAL_DIAG0
  auto diag0_result = ReadRegister(CentralReg::GLOBAL_DIAG0);
  if (!diag0_result) {
    return tle::unexpected(diag0_result.error());
  }
  uint16_t diag0 = *diag0_result;

  // External supply faults
  report.vbat_uv = (diag0 & GLOBAL_DIAG0::VBAT_UV) != 0;
  report.vbat_ov = (diag0 & GLOBAL_DIAG0::VBAT_OV) != 0;
  report.vio_uv = (diag0 & GLOBAL_DIAG0::VIO_UV) != 0;
  report.vio_ov = (diag0 & GLOBAL_DIAG0::VIO_OV) != 0;
  report.vdd_uv = (diag0 & GLOBAL_DIAG0::VDD_UV) != 0;
  report.vdd_ov = (diag0 & GLOBAL_DIAG0::VDD_OV) != 0;

  // System faults
  report.clock_fault = (diag0 & GLOBAL_DIAG0::CLK_NOK) != 0;
  report.spi_wd_error = (diag0 & GLOBAL_DIAG0::SPI_WD_ERR) != 0;

  // Temperature faults
  report.ot_error = (diag0 & GLOBAL_DIAG0::COTERR) != 0;
  report.ot_warning = (diag0 & GLOBAL_DIAG0::COTWARN) != 0;

  // Reset events
  report.reset_event = (diag0 & GLOBAL_DIAG0::RES_EVENT) != 0;
  report.por_event = (diag0 & GLOBAL_DIAG0::POR_EVENT) != 0;

  // Read GLOBAL_DIAG1
  auto diag1_result = ReadRegister(CentralReg::GLOBAL_DIAG1);
  if (diag1_result) {
    uint16_t diag1 = *diag1_result;
    report.vr_iref_uv = (diag1 & GLOBAL_DIAG1::VR_IREF_UV) != 0;
    report.vr_iref_ov = (diag1 & GLOBAL_DIAG1::VR_IREF_OV) != 0;
    report.vdd2v5_uv = (diag1 & GLOBAL_DIAG1::VDD2V5_UV) != 0;
    report.vdd2v5_ov = (diag1 & GLOBAL_DIAG1::VDD2V5_OV) != 0;
    report.ref_uv = (diag1 & GLOBAL_DIAG1::REF_UV) != 0;
    report.ref_ov = (diag1 & GLOBAL_DIAG1::REF_OV) != 0;
    report.vpre_ov = (diag1 & GLOBAL_DIAG1::VPRE_OV) != 0;
    report.hvadc_err = (diag1 & GLOBAL_DIAG1::HVADC_ERR) != 0;
  }

  // Read GLOBAL_DIAG2
  auto diag2_result = ReadRegister(CentralReg::GLOBAL_DIAG2);
  if (diag2_result) {
    uint16_t diag2 = *diag2_result;
    report.reg_ecc_err = (diag2 & GLOBAL_DIAG2::REG_ECC_ERR) != 0;
    report.otp_ecc_err = (diag2 & GLOBAL_DIAG2::OTP_ECC_ERR) != 0;
    report.otp_virgin = (diag2 & GLOBAL_DIAG2::OTP_VIRGIN) != 0;
  }

  // Read FB_STAT for summary flags
  auto fb_stat_result = ReadRegister(CentralReg::FB_STAT);
  if (fb_stat_result) {
    uint16_t fb_stat = *fb_stat_result;
    report.supply_nok_internal = (fb_stat & FB_STAT::SUP_NOK_INT) != 0;
    report.supply_nok_external = (fb_stat & FB_STAT::SUP_NOK_EXT) != 0;
  }

  // Read channel-specific faults. Per \u00a75.3.2.12/13 each DIAG register covers
  // a PAIR of channels via an 8*y bit offset. Use DIAG_*_CHGR::AddressForChannel
  // and DIAG_*_CHGR::ChannelBitMask to pick the correct group and bits.
  for (uint8_t ch = 0; ch < 6; ++ch) {
    // --- Errors (DIAG_ERR_CHGRx) ---
    const uint16_t err_addr = DIAG_ERR_CHGR::AddressForChannel(ch);
    auto diag_err_result = ReadRegister(err_addr);
    if (diag_err_result) {
      const uint16_t diag_err = *diag_err_result;
      report.channels[ch].open_load_short_ground =
          (diag_err & DIAG_ERR_CHGR::ChannelBitMask(ch, DIAG_ERR_CHGR::BIT_OLSG)) != 0;
      report.channels[ch].open_load =
          (diag_err & DIAG_ERR_CHGR::ChannelBitMask(ch, DIAG_ERR_CHGR::BIT_OL))   != 0;
      report.channels[ch].overcurrent =
          (diag_err & DIAG_ERR_CHGR::ChannelBitMask(ch, DIAG_ERR_CHGR::BIT_OC))   != 0;
      report.channels[ch].short_to_ground =
          (diag_err & DIAG_ERR_CHGR::ChannelBitMask(ch, DIAG_ERR_CHGR::BIT_SG))   != 0;
      report.channels[ch].over_temperature =
          (diag_err & DIAG_ERR_CHGR::ChannelBitMask(ch, DIAG_ERR_CHGR::BIT_OTE))  != 0;
    }

    // --- Warnings (DIAG_WARN_CHGRx) ---
    const uint16_t warn_addr = DIAG_WARN_CHGR::AddressForChannel(ch);
    auto diag_warn_result = ReadRegister(warn_addr);
    if (diag_warn_result) {
      const uint16_t diag_warn = *diag_warn_result;
      report.channels[ch].pwm_regulation_warning =
          (diag_warn & DIAG_WARN_CHGR::ChannelBitMask(ch, DIAG_WARN_CHGR::BIT_PWM_REG))   != 0;
      report.channels[ch].current_regulation_warning =
          (diag_warn & DIAG_WARN_CHGR::ChannelBitMask(ch, DIAG_WARN_CHGR::BIT_I_REG))     != 0;
      report.channels[ch].ot_warning =
          (diag_warn & DIAG_WARN_CHGR::ChannelBitMask(ch, DIAG_WARN_CHGR::BIT_OTW))       != 0;
      report.channels[ch].olsg_warning =
          (diag_warn & DIAG_WARN_CHGR::ChannelBitMask(ch, DIAG_WARN_CHGR::BIT_OLSG_WARN)) != 0;
      // OLSG_WARN_CHK_NOK is surfaced via ChannelDiagnostics; FaultReport's
      // per-channel bitset does not yet have a dedicated field. Intentionally
      // not wired here so a POR-default CHK_NOK=1 is not reported as a fault.
    }

    // Check if channel has any fault
    report.channels[ch].has_fault =
        report.channels[ch].overcurrent || report.channels[ch].short_to_ground ||
        report.channels[ch].open_load || report.channels[ch].over_temperature ||
        report.channels[ch].open_load_short_ground || report.channels[ch].ot_warning ||
        report.channels[ch].current_regulation_warning ||
        report.channels[ch].pwm_regulation_warning || report.channels[ch].olsg_warning;
  }

  // Determine if any fault exists
  report.any_fault =
      report.vbat_uv || report.vbat_ov || report.vio_uv || report.vio_ov || report.vdd_uv ||
      report.vdd_ov || report.vr_iref_uv || report.vr_iref_ov || report.vdd2v5_uv ||
      report.vdd2v5_ov || report.ref_uv || report.ref_ov || report.vpre_ov || report.hvadc_err ||
      report.clock_fault || report.spi_wd_error || report.ot_error || report.ot_warning ||
      report.reg_ecc_err || report.otp_ecc_err || report.otp_virgin || report.supply_nok_internal ||
      report.supply_nok_external || report.channels[0].has_fault || report.channels[1].has_fault ||
      report.channels[2].has_fault || report.channels[3].has_fault ||
      report.channels[4].has_fault || report.channels[5].has_fault;

  return report;
}

/**
 * @brief Get VIO thresholds (fixed hardware values, not programmable)
 *
 * @details
 * VIO thresholds are fixed hardware values that depend on VIO_SEL bit setting.
 * These thresholds are NOT stored in registers and cannot be read or programmed.
 * Values are from datasheet Table 9 (Electrical characteristics power supply).
 *
 * @param uv_threshold Output: UV threshold in millivolts
 * @param ov_threshold Output: OV threshold in millivolts
 * @param vio_5v VIO_SEL setting (false=3.3V mode, true=5V mode)
 *
 * @note
 * - 3.3V mode (VIO_SEL=0): UV = 2.6-3.0V (using 2.8V), OV = 3.6-4.1V (using 3.85V)
 * - 5V mode (VIO_SEL=1): UV = 3.7-4.5V (using 4.1V), OV = 5.5-6.4V (using 5.95V)
 *
 * The datasheet provides min-max ranges, not typical values. We use mid-range estimates.
 */
static void getVioThresholds(uint16_t& uv_threshold, uint16_t& ov_threshold, bool vio_5v) noexcept {
  if (vio_5v) {
    // 5V mode: VIO_UV,5V,TH = 3.7-4.5V, VIO_OV,5V,TH = 5.5-6.4V
    uv_threshold = 4100; // Mid-range estimate (3.7-4.5V range)
    ov_threshold = 5950; // Mid-range estimate (5.5-6.4V range)
  } else {
    // 3.3V mode: VIO_UV,3V3,TH = 2.6-3.0V, VIO_OV,3V3,TH = 3.6-4.1V
    uv_threshold = 2800; // Mid-range estimate (2.6-3.0V range)
    ov_threshold = 3850; // Mid-range estimate (3.6-4.1V range)
  }
}

/**
 * @brief Get VDD thresholds (fixed hardware values, not programmable)
 *
 * @details
 * VDD thresholds are fixed hardware values that are NOT stored in registers
 * and cannot be read or programmed. Values are from datasheet Table 9
 * (Electrical characteristics power supply).
 *
 * @param uv_threshold Output: UV threshold in millivolts
 * @param ov_threshold Output: OV threshold in millivolts
 *
 * @note
 * VDD_UV,TH = 3.7-4.5V (using 4.1V mid-range)
 * VDD_OV,TH = 5.5-6.4V (using 5.95V mid-range)
 *
 * The datasheet provides min-max ranges, not typical values. We use mid-range estimates.
 */
static void getVddThresholds(uint16_t& uv_threshold, uint16_t& ov_threshold) noexcept {
  // VDD thresholds (fixed hardware): VDD_UV,TH = 3.7-4.5V, VDD_OV,TH = 5.5-6.4V
  uv_threshold = 4100; // Mid-range estimate (3.7-4.5V range)
  ov_threshold = 5950; // Mid-range estimate (5.5-6.4V range)
}

template <typename CommType>
DriverResult<void> Driver<CommType>::PrintAllFaults() noexcept {
  auto fault_result = GetAllFaults();
  if (!fault_result) {
    return tle::unexpected(fault_result.error());
  }

  const FaultReport& report = *fault_result;

  if (!report.any_fault) {
    comm_.Log(LogLevel::Info, "TLE92466ED", "✅ No faults detected - All systems normal");
    return {};
  }

  // Read voltage measurements and thresholds for voltage-related faults
  uint16_t vbat_mv = 0;
  uint16_t vio_mv = 0;
  uint16_t vdd_mv = 0;
  uint16_t vbat_uv_th_mv = 0;
  uint16_t vbat_ov_th_mv = 0;
  uint16_t vio_uv_th_mv = 0;
  uint16_t vio_ov_th_mv = 0;
  uint16_t vdd_uv_th_mv = 0;
  uint16_t vdd_ov_th_mv = 0;
  bool vio_5v = false;

  // Read current voltages
  if (auto vbat_result = GetVbatVoltage(); vbat_result) {
    vbat_mv = *vbat_result;
  }
  if (auto vio_result = GetVioVoltage(); vio_result) {
    vio_mv = *vio_result;
  }
  if (auto vdd_result = GetVddVoltage(); vdd_result) {
    vdd_mv = *vdd_result;
  }

  // Read VBAT thresholds
  if (auto result = GetVbatThresholds(vbat_uv_th_mv, vbat_ov_th_mv); !result) {
    // If reading fails, thresholds remain 0
  }

  // Determine VIO thresholds based on VIO_SEL setting
  // Note: GLOBAL_CONFIG may be write-only, so we can't reliably read it back
  // We default to 3.3V mode (VIO_SEL=0) which is set in applyDefaultConfig()
  // If user needs 5V mode, they should call ConfigureGlobal() with vio_5v=true
  // For now, we'll try to read it, but default to 3.3V if read fails or returns unexpected value
  // vio_5v is already declared above, just update it
  if (auto global_config_result = ReadRegister(CentralReg::GLOBAL_CONFIG); global_config_result) {
    // Try to read VIO_SEL bit, but don't trust it if it's write-only
    vio_5v = (*global_config_result & GLOBAL_CONFIG::VIO_SEL) != 0;
    // If read returns 0x4005 (default), it might be the power-on default, not what we wrote
    // So we'll use it as a hint, but the actual setting is what we wrote in applyDefaultConfig()
    if (*global_config_result == 0x4005) {
      // This is the datasheet default (5V mode), but we wrote 3.3V mode in applyDefaultConfig()
      // So trust our write, not the read
      vio_5v = false;
      comm_.Log(LogLevel::Info, "TLE92466ED",
                "GLOBAL_CONFIG read returned default 0x4005, using 3.3V mode (as written in "
                "applyDefaultConfig)");
    } else {
      comm_.Log(LogLevel::Info, "TLE92466ED", "Read GLOBAL_CONFIG: 0x%04X, VIO_SEL=%s",
                *global_config_result, vio_5v ? "5V" : "3.3V");
    }
  } else {
    comm_.Log(LogLevel::Info, "TLE92466ED",
              "GLOBAL_CONFIG read failed, assuming 3.3V mode (as written in applyDefaultConfig)");
  }
  getVioThresholds(vio_uv_th_mv, vio_ov_th_mv, vio_5v);

  // Get VDD thresholds (fixed values)
  getVddThresholds(vdd_uv_th_mv, vdd_ov_th_mv);

  // Print header
  comm_.Log(LogLevel::Warn, "TLE92466ED",
            "╔══════════════════════════════════════════════════════════════════════════════╗");
  comm_.Log(LogLevel::Warn, "TLE92466ED",
            "║                          FAULT DETECTION REPORT                              ║");
  comm_.Log(LogLevel::Warn, "TLE92466ED",
            "╠══════════════════════════════════════════════════════════════════════════════╣");

  // External Supply Faults
  bool has_external_faults = report.vbat_uv || report.vbat_ov || report.vio_uv || report.vio_ov ||
                             report.vdd_uv || report.vdd_ov;
  if (has_external_faults) {
    comm_.Log(LogLevel::Warn, "TLE92466ED", "║ External Supply Faults:");
    if (report.vbat_uv) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ VBAT Undervoltage");
      if (vbat_mv > 0 && vbat_uv_th_mv > 0) {
        comm_.Log(LogLevel::Warn, "TLE92466ED", "║     Current: %u mV | UV Threshold: %u mV",
                  vbat_mv, vbat_uv_th_mv);
      }
    }
    if (report.vbat_ov) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ VBAT Overvoltage");
      if (vbat_mv > 0 && vbat_ov_th_mv > 0) {
        comm_.Log(LogLevel::Warn, "TLE92466ED", "║     Current: %u mV | OV Threshold: %u mV",
                  vbat_mv, vbat_ov_th_mv);
      }
    }
    if (report.vio_uv) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ VIO Undervoltage");
      if (vio_mv > 0) {
        comm_.Log(LogLevel::Warn, "TLE92466ED",
                  "║     Current: %u mV | UV Threshold: %u mV (fixed hw, est)", vio_mv,
                  vio_uv_th_mv);
        // Note: VIO thresholds have a range (2.6-3.0V for 3.3V mode, 3.7-4.5V for 5V mode)
        // The actual threshold may be anywhere in this range, and there may be hysteresis
        // If fault flag is set, hardware detected the condition - voltage may have been lower
        // when fault triggered, or threshold may be higher than our estimate
        if (vio_mv > vio_uv_th_mv) {
          comm_.Log(
              LogLevel::Info, "TLE92466ED",
              "║     Note: Current voltage is above estimated threshold, but fault flag is set.");
          comm_.Log(
              LogLevel::Info, "TLE92466ED",
              "║     This may indicate: (1) voltage was lower when fault triggered, (2) actual");
          comm_.Log(
              LogLevel::Info, "TLE92466ED",
              "║     threshold is higher than estimate, or (3) hysteresis in fault detection.");
        }
      }
    }
    if (report.vio_ov) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ VIO Overvoltage");
      if (vio_mv > 0) {
        comm_.Log(LogLevel::Warn, "TLE92466ED",
                  "║     Current: %u mV | OV Threshold: %u mV (fixed hw, est)", vio_mv,
                  vio_ov_th_mv);
      }
    }
    if (report.vdd_uv) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ VDD Undervoltage");
      if (vdd_mv > 0) {
        comm_.Log(LogLevel::Warn, "TLE92466ED",
                  "║     Current: %u mV | UV Threshold: %u mV (fixed hw, est)", vdd_mv,
                  vdd_uv_th_mv);
      }
    }
    if (report.vdd_ov) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ VDD Overvoltage");
      if (vdd_mv > 0) {
        comm_.Log(LogLevel::Warn, "TLE92466ED",
                  "║     Current: %u mV | OV Threshold: %u mV (fixed hw, est)", vdd_mv,
                  vdd_ov_th_mv);
      }
    }
  }

  // Internal Supply Faults
  bool has_internal_faults = report.vr_iref_uv || report.vr_iref_ov || report.vdd2v5_uv ||
                             report.vdd2v5_ov || report.ref_uv || report.ref_ov || report.vpre_ov ||
                             report.hvadc_err;
  if (has_internal_faults) {
    comm_.Log(LogLevel::Warn, "TLE92466ED", "║ Internal Supply Faults:");
    if (report.vr_iref_uv) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ Internal Bias Current Undervoltage");
    }
    if (report.vr_iref_ov) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ Internal Bias Current Overvoltage");
    }
    if (report.vdd2v5_uv) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ Internal 2.5V Supply Undervoltage");
    }
    if (report.vdd2v5_ov) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ Internal 2.5V Supply Overvoltage");
    }
    if (report.ref_uv) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ Internal Reference Undervoltage");
    }
    if (report.ref_ov) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ Internal Reference Overvoltage");
    }
    if (report.vpre_ov) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ Internal Pre-Regulator Overvoltage");
    }
    if (report.hvadc_err) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ Internal Monitoring ADC Error");
    }
  }

  // System Faults
  if (report.clock_fault || report.spi_wd_error) {
    comm_.Log(LogLevel::Warn, "TLE92466ED", "║ System Faults:");
    if (report.clock_fault) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ Clock Fault");
    }
    if (report.spi_wd_error) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ SPI Watchdog Error");
    }
  }

  // Temperature Faults
  if (report.ot_error || report.ot_warning) {
    comm_.Log(LogLevel::Warn, "TLE92466ED", "║ Temperature Faults:");
    if (report.ot_error) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ Central Over-Temperature Error");
    }
    if (report.ot_warning) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ⚠️  Central Over-Temperature Warning");
    }
  }

  // Reset Events
  if (report.por_event || report.reset_event) {
    comm_.Log(LogLevel::Info, "TLE92466ED", "║ Reset Events:");
    if (report.por_event) {
      comm_.Log(LogLevel::Info, "TLE92466ED", "║   ℹ️  Power-On Reset Event");
    }
    if (report.reset_event) {
      comm_.Log(LogLevel::Info, "TLE92466ED", "║   ℹ️  External Reset Event (RESN pin)");
    }
  }

  // Memory/ECC Faults
  if (report.reg_ecc_err || report.otp_ecc_err || report.otp_virgin) {
    comm_.Log(LogLevel::Warn, "TLE92466ED", "║ Memory/ECC Faults:");
    if (report.reg_ecc_err) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ Register ECC Error");
    }
    if (report.otp_ecc_err) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ OTP ECC Error");
    }
    if (report.otp_virgin) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ⚠️  OTP Virgin/Unconfigured");
    }
  }

  // Summary Flags
  if (report.supply_nok_internal || report.supply_nok_external) {
    comm_.Log(LogLevel::Warn, "TLE92466ED", "║ Supply Summary:");
    if (report.supply_nok_external) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ External Supply Fault Summary");
    }
    if (report.supply_nok_internal) {
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   ❌ Internal Supply Fault Summary");
    }
  }

  // Channel-specific faults
  bool has_channel_faults = false;
  for (uint8_t ch = 0; ch < 6; ++ch) {
    if (report.channels[ch].has_fault) {
      if (!has_channel_faults) {
        comm_.Log(LogLevel::Warn, "TLE92466ED", "║ Channel Faults:");
        has_channel_faults = true;
      }
      comm_.Log(LogLevel::Warn, "TLE92466ED", "║   Channel %u:", ch);
      if (report.channels[ch].overcurrent) {
        comm_.Log(LogLevel::Warn, "TLE92466ED", "║     ❌ Over-Current");
      }
      if (report.channels[ch].short_to_ground) {
        comm_.Log(LogLevel::Warn, "TLE92466ED", "║     ❌ Short to Ground");
      }
      if (report.channels[ch].open_load) {
        comm_.Log(LogLevel::Warn, "TLE92466ED", "║     ❌ Open Load");
      }
      if (report.channels[ch].over_temperature) {
        comm_.Log(LogLevel::Warn, "TLE92466ED", "║     ❌ Over-Temperature");
      }
      if (report.channels[ch].open_load_short_ground) {
        comm_.Log(LogLevel::Warn, "TLE92466ED", "║     ❌ Open Load or Short to Ground");
      }
      if (report.channels[ch].ot_warning) {
        comm_.Log(LogLevel::Warn, "TLE92466ED", "║     ⚠️  Over-Temperature Warning");
      }
      if (report.channels[ch].current_regulation_warning) {
        comm_.Log(LogLevel::Warn, "TLE92466ED", "║     ⚠️  Current Regulation Warning");
      }
      if (report.channels[ch].pwm_regulation_warning) {
        comm_.Log(LogLevel::Warn, "TLE92466ED", "║     ⚠️  PWM Regulation Warning");
      }
      if (report.channels[ch].olsg_warning) {
        comm_.Log(LogLevel::Warn, "TLE92466ED", "║     ⚠️  OLSG Warning");
      }
    }
  }

  comm_.Log(LogLevel::Warn, "TLE92466ED",
            "╚══════════════════════════════════════════════════════════════════════════════╝");

  return {};
}

template <typename CommType>
DriverResult<void> Driver<CommType>::SoftwareReset() noexcept {
  comm_.Log(LogLevel::Info, "TLE92466ED",
            "Performing software reset (entering config mode and clearing channel enable cache)");
  // Software reset would require toggling RESN pin or power cycle
  // This IC doesn't have a software reset register
  // Return to config mode and clear channel enable cache
  // Note: Cannot disable channels in Config Mode (requires Mission Mode)
  //       So we just clear the cache - channels will be disabled when entering Mission Mode

  if (auto result = EnterConfigMode(); !result) {
    return result;
  }

  // Clear channel enable cache (channels are automatically disabled in Config Mode)
  channel_enable_cache_ = 0;
  // Also clear channel enable bits in ch_ctrl_cache_ (but keep OP_MODE and parallel bits)
  ch_ctrl_cache_ &= ~CH_CTRL::ALL_CH_MASK;

  comm_.Log(LogLevel::Info, "TLE92466ED",
            "✅ Software reset completed (Config Mode entered, channel cache cleared)");
  return {};
}

//==========================================================================
// WATCHDOG MANAGEMENT
//==========================================================================

template <typename CommType>
DriverResult<void> Driver<CommType>::ReloadSpiWatchdog(uint16_t reload_value) noexcept {
  if (auto result = checkInitialized(); !result) {
    return result;
  }

  // Mask to 11-bit field (bits 10:0) per datasheet
  uint16_t masked_value = WD_RELOAD::MaskValue(reload_value);

  comm_.Log(LogLevel::Info, "TLE92466ED",
            "Reloading SPI watchdog: ReloadValue=%u (masked to 0x%03X)", reload_value,
            masked_value);

  // Note: Writing any non-zero value clears SPI_WD_ERR if it was set
  return WriteRegister(CentralReg::WD_RELOAD, masked_value);
}

//==========================================================================
// DEVICE INFORMATION
//==========================================================================

template <typename CommType>
DriverResult<uint16_t> Driver<CommType>::GetIcVersion() noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }

  return ReadRegister(CentralReg::ICVID);
}

template <typename CommType>
DriverResult<std::array<uint16_t, 3>> Driver<CommType>::GetChipId() noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }

  std::array<uint16_t, 3> chip_id;

  auto id0_result = ReadRegister(CentralReg::CHIPID0);
  if (!id0_result) {
    return tle::unexpected(id0_result.error());
  }
  chip_id[0] = *id0_result;

  auto id1_result = ReadRegister(CentralReg::CHIPID1);
  if (!id1_result) {
    return tle::unexpected(id1_result.error());
  }
  chip_id[1] = *id1_result;

  auto id2_result = ReadRegister(CentralReg::CHIPID2);
  if (!id2_result) {
    return tle::unexpected(id2_result.error());
  }
  chip_id[2] = *id2_result;

  return chip_id;
}

template <typename CommType>
DriverResult<bool> Driver<CommType>::VerifyDevice() noexcept {
  // Read ICVID register to verify device is responding and check device type
  auto id_result = ReadRegister(CentralReg::ICVID, false); // Don't verify CRC during init

  if (!id_result) {
    comm_.Log(LogLevel::Error, "TLE92466ED",
              "Device verification failed: Failed to read ICVID register (error: %u)",
              static_cast<unsigned>(id_result.error()));
    return tle::unexpected(id_result.error());
  }

  uint16_t icvid = *id_result;

  // Check if we got a valid response (not all zeros or all ones)
  if (icvid == 0x0000 || icvid == 0xFFFF) {
    comm_.Log(LogLevel::Error, "TLE92466ED",
              "Device verification failed: Invalid ICVID response (0x%04X)", icvid);
    return DriverResult<bool>{false};
  }

  // Validate device ID
  bool valid = DeviceID::IsValidDevice(icvid);

  // Extract and log device information
  uint8_t device_type = DeviceID::GetDeviceType(icvid);
  uint8_t revision = DeviceID::GetRevision(icvid);

  if (valid) {
    comm_.Log(LogLevel::Info, "TLE92466ED",
              "Device verified: ICVID=0x%04X, Type=0x%02X, Revision=0x%02X", icvid, device_type,
              revision);
  } else {
    comm_.Log(LogLevel::Warn, "TLE92466ED",
              "Device verification: ICVID=0x%04X (Type=0x%02X, Rev=0x%02X) - Unknown device type",
              icvid, device_type, revision);
  }

  return static_cast<bool>(valid);
}

//==========================================================================
// REGISTER ACCESS
//==========================================================================

template <typename CommType>
DriverResult<uint32_t> Driver<CommType>::ReadRegister(uint16_t address, bool verify_crc) noexcept {
  if (!comm_.IsReady()) {
    return tle::unexpected(DriverError::HardwareError);
  }

  // Use internal CRC enable state by default
  // verify_crc=false allows override to disable CRC verification (e.g., during init)
  // verify_crc=true allows override to force CRC verification
  bool should_verify_crc = verify_crc ? true : crc_enabled_;

  // Use CommInterface Read function (handles frame construction, CRC, and transfer)
  auto result = comm_.Read(address, should_verify_crc);
  if (!result) {
    // Map CommInterface error to driver error
    switch (result.error()) {
    case CommError::Timeout:
      return tle::unexpected(DriverError::TimeoutError);
    case CommError::CRCError:
      return tle::unexpected(DriverError::CRCError);
    case CommError::BusError:
    case CommError::TransferError:
      return tle::unexpected(DriverError::HardwareError);
    default:
      return tle::unexpected(DriverError::HardwareError);
    }
  }

  return *result;
}

template <typename CommType>
DriverResult<void> Driver<CommType>::WriteRegister(uint16_t address, uint16_t value, bool verify_crc,
                                         bool verify_write) noexcept {
  if (!comm_.IsReady()) {
    return tle::unexpected(DriverError::HardwareError);
  }

  // Use internal CRC enable state by default
  // verify_crc=false allows override to disable CRC verification (e.g., during init)
  // verify_crc=true allows override to force CRC verification
  bool should_verify_crc = verify_crc ? true : crc_enabled_;

  // Use CommInterface Write function (handles frame construction, CRC, and transfer)
  auto result = comm_.Write(address, value, should_verify_crc);
  if (!result) {
    // Map CommInterface error to driver error
    switch (result.error()) {
    case CommError::Timeout:
      return tle::unexpected(DriverError::TimeoutError);
    case CommError::CRCError:
      return tle::unexpected(DriverError::CRCError);
    case CommError::BusError:
    case CommError::TransferError:
      return tle::unexpected(DriverError::HardwareError);
    default:
      return tle::unexpected(DriverError::HardwareError);
    }
  }

  // Read back register to verify write succeeded
  if (verify_write) {
    // Small delay to ensure write has propagated (some registers may need time)
    comm_.Delay(1); // 1ms delay

    auto read_result = ReadRegister(address, verify_crc);
    if (read_result) {
      auto read_value = static_cast<uint16_t>(*read_result);

      // Special handling for known problematic registers
      // CH_CTRL (0x0000): Reads may return 0x0000 even after write due to device behavior
      // GLOBAL_CONFIG (0x0002): Write-only, reads return default or previous value
      // GLOBAL_DIAGx (0x0003-0x0005): Write-1-to-clear, reads return current fault state
      bool known_issue = false;
      const char* reason = nullptr;

      if (address == CentralReg::CH_CTRL) {
        // CH_CTRL is readable per datasheet, but may return 0x0000 in some cases
        // This is a known device behavior - the write succeeds but read-back may not reflect it
        // immediately We track CH_CTRL state in cache (ch_ctrl_cache_) for this reason
        known_issue = true;
        reason = "CH_CTRL may return 0x0000 on read (known device behavior, write succeeds)";
      } else if (address == CentralReg::GLOBAL_CONFIG) {
        known_issue = true;
        reason = "GLOBAL_CONFIG is write-only, reads return default/previous value";
      } else if (address == CentralReg::WD_RELOAD) {
        // WD_RELOAD counter is constantly decremented by the watchdog timer
        // Read value will be less than or equal to written value (may have decremented)
        // This is expected behavior - the watchdog is actively counting down
        known_issue = true;
        reason =
            "WD_RELOAD counter decrements continuously (read value <= written value is expected)";
      } else if (address == CentralReg::GLOBAL_DIAG0 || address == CentralReg::GLOBAL_DIAG1 ||
                 address == CentralReg::GLOBAL_DIAG2) {
        // These are write-1-to-clear registers, reads return current fault state
        // Mismatch is expected when clearing faults (writing 0xFFFF to clear, but read shows
        // current faults)
        known_issue = true;
        reason = "GLOBAL_DIAGx are write-1-to-clear, reads return current fault state";
      } else if (address == CentralReg::DIAG_ERR_CHGR0 ||
                 address == CentralReg::DIAG_ERR_CHGR1 ||
                 address == CentralReg::DIAG_ERR_CHGR2) {
        // Per-pair error diagnostics (datasheet §5.3.2.12): write-1-to-clear
        // and read returns the current latched fault state for the channel
        // pair. The driver clears these by writing 0xFFFF or per-bit OC masks
        // — a read-back showing 0x0000 (no faults) or unrelated bits is
        // expected behavior, not a bus failure.
        known_issue = true;
        reason = "DIAG_ERR_CHGRx are write-1-to-clear, reads return current fault state";
      } else if (address == CentralReg::DIAG_WARN_CHGR0 ||
                 address == CentralReg::DIAG_WARN_CHGR1 ||
                 address == CentralReg::DIAG_WARN_CHGR2) {
        // Per-pair warnings (datasheet §5.3.2.13). Same write-1-to-clear
        // semantics. POR default is 0x1010 (OLSG_WARN_CHK_NOK set on both
        // channels) — read-back of 0x1010 immediately after clearing is
        // expected until at least one OLSG check window has run.
        known_issue = true;
        reason = "DIAG_WARN_CHGRx are write-1-to-clear, POR=0x1010 until first OLSG check";
      }

      if (read_value != value) {
        if (known_issue) {
          comm_.Log(LogLevel::Debug, "TLE92466ED",
                    "Write verification mismatch (expected): Address=0x%04X, Written=0x%04X, "
                    "Read=0x%04X\n"
                    "  %s",
                    address, value, read_value, reason);
        } else {
          comm_.Log(LogLevel::Warn, "TLE92466ED",
                    "Write verification failed: Address=0x%04X, Written=0x%04X, Read=0x%04X\n"
                    "  (This may be normal for write-only or special registers)",
                    address, value, read_value);
        }
      } else {
        comm_.Log(LogLevel::Debug, "TLE92466ED", "Write verified: Address=0x%04X, Value=0x%04X",
                  address, value);
      }
    } else {
      // Read failed - this might be expected for write-only registers
      comm_.Log(LogLevel::Debug, "TLE92466ED",
                "Write verification read failed for address 0x%04X (may be write-only)", address);
    }
  }

  return {};
}

template <typename CommType>
DriverResult<void> Driver<CommType>::ModifyRegister(uint16_t address, uint16_t mask,
                                          uint16_t value) noexcept {

  // Read current value
  auto read_result = ReadRegister(address);
  if (!read_result) {
    return tle::unexpected(read_result.error());
  }

  // Modify bits
  uint16_t new_value = (*read_result & ~mask) | (value & mask);

  // Write back
  return WriteRegister(address, new_value);
}

//==========================================================================
// PRIVATE METHODS
//==========================================================================

template <typename CommType>
DriverResult<SPIFrame> Driver<CommType>::transferFrame(const SPIFrame& tx_frame, bool verify_crc) noexcept {
  // Transfer 32-bit frame via CommInterface
  auto comm_result = comm_.Transfer32(tx_frame.word);
  if (!comm_result) {
    // Map CommInterface error to driver error
    switch (comm_result.error()) {
    case CommError::Timeout:
      return tle::unexpected(DriverError::TimeoutError);
    case CommError::CRCError:
      return tle::unexpected(DriverError::CRCError);
    case CommError::TransferError:
    case CommError::BusError:
      return tle::unexpected(DriverError::HardwareError);
    default:
      return tle::unexpected(DriverError::RegisterError);
    }
  }

  SPIFrame rx_frame{};
  rx_frame.word = *comm_result;

  // Verify CRC if requested
  if (verify_crc) {
    if (!VerifyFrameCrc(rx_frame)) {
      return tle::unexpected(DriverError::CRCError);
    }
  }

  // Check SPI status in reply
  if (auto result = checkSpiStatus(rx_frame); !result) {
    return tle::unexpected(result.error());
  }

  return rx_frame;
}

template <typename CommType>
DriverResult<void> Driver<CommType>::checkSpiStatus(const SPIFrame& rx_frame) noexcept {
  // Status field only exists in 16-bit reply frames
  if (rx_frame.rx_common.reply_mode != 0x00) {
    // For non-16-bit frames, check if it's a critical fault
    if (rx_frame.rx_common.reply_mode == 0x02) {
      // Critical fault frame - extract and log fault flags
      auto fault_flags = CriticalFaultFlags::Extract(rx_frame);
      // Return hardware error for critical faults
      return tle::unexpected(DriverError::HardwareError);
    }
    // 22-bit reply frames don't have status field, assume OK
    return {};
  }

  // 16-bit reply frame - check status field
  auto status = static_cast<SPIStatus>(rx_frame.rx_16bit.status);

  switch (status) {
  case SPIStatus::NO_ERROR:
    return {};
  case SPIStatus::SPI_FRAME_ERROR:
    return tle::unexpected(DriverError::SPIFrameError);
  case SPIStatus::CRC_ERROR:
    return tle::unexpected(DriverError::CRCError);
  case SPIStatus::WRITE_RO_REG:
    return tle::unexpected(DriverError::WriteToReadOnly);
  case SPIStatus::INTERNAL_BUS_FAULT:
    return tle::unexpected(DriverError::RegisterError);
  default:
    return tle::unexpected(DriverError::RegisterError);
  }
}

template <typename CommType>
DriverResult<bool> Driver<CommType>::isChannelParallel(Channel channel) noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }

  if (!isValidChannelInternal(channel)) {
    return tle::unexpected(DriverError::InvalidChannel);
  }

  // CH_CTRL is write-only on silicon \u2014 read-back always returns 0x0000.
  // Use the driver-maintained shadow (ch_ctrl_cache_) to answer parallel-pair
  // queries. Any API that mutates the parallel-pair bits MUST keep the cache
  // synchronized (see SetParallelPair() in the channel-configuration section).
  const uint16_t ch_ctrl = ch_ctrl_cache_;
  const uint8_t  ch_index = ToIndex(channel);

  switch (ch_index) {
  case 0:
  case 3:
    return DriverResult<bool>{(ch_ctrl & CH_CTRL::CH_PAR_0_3) != 0};
  case 1:
  case 2:
    return DriverResult<bool>{(ch_ctrl & CH_CTRL::CH_PAR_1_2) != 0};
  case 4:
  case 5:
    return DriverResult<bool>{(ch_ctrl & CH_CTRL::CH_PAR_4_5) != 0};
  default:
    return DriverResult<bool>{false};
  }
}

//==========================================================================
// GPIO CONTROL (Reset, Enable, Fault Status)
//==========================================================================

template <typename CommType>
DriverResult<void> Driver<CommType>::SetReset(bool reset) noexcept {
  comm_.Log(LogLevel::Info, "TLE92466ED", "Setting reset pin: %s",
            reset ? "LOW (in reset)" : "HIGH (released)");
  // RESN is active LOW. The bus implementation already maps GpioSignal
  // to the correct electrical level for the pin's active state:
  //   GpioSet(RESN, ACTIVE)   → physical LOW  (chip held in reset)
  //   GpioSet(RESN, INACTIVE) → physical HIGH (chip released)
  //
  // So `reset==true` (caller wants chip IN reset, line LOW) must
  // pass ACTIVE through to the bus; `reset==false` (caller wants chip
  // released, line HIGH) must pass INACTIVE. The previous mapping was
  // INVERTED — driver thought it was releasing the chip but was
  // actually putting it into reset, leaving SDO permanently low and
  // the chip silent on every SPI transaction.
  GpioSignal signal = reset ? GpioSignal::ACTIVE : GpioSignal::INACTIVE;

  auto result = comm_.GpioSet(CtrlPin::RESN, signal);
  if (!result) {
    return tle::unexpected(DriverError::HardwareError);
  }

  return {};
}

template <typename CommType>
DriverResult<void> Driver<CommType>::SetEnable(bool enable) noexcept {
  comm_.Log(LogLevel::Info, "TLE92466ED", "Setting enable pin: %s",
            enable ? "HIGH (enabled)" : "LOW (disabled)");
  // EN is active high: enable=true means enable outputs (GPIO HIGH), enable=false means disable
  // (GPIO LOW)
  GpioSignal signal = enable ? GpioSignal::ACTIVE : GpioSignal::INACTIVE;

  auto result = comm_.GpioSet(CtrlPin::EN, signal);
  if (!result) {
    return tle::unexpected(DriverError::HardwareError);
  }

  return {};
}

template <typename CommType>
DriverResult<bool> Driver<CommType>::IsFault(bool print_faults) noexcept {
  auto result = comm_.GpioRead(CtrlPin::FAULTN);
  if (!result) {
    return tle::unexpected(DriverError::HardwareError);
  }

  // FAULTN is active low: ACTIVE means fault detected, INACTIVE means no fault
  bool fault_detected = (*result == GpioSignal::ACTIVE);

  // If fault is detected and print_faults is true, automatically print detailed fault report
  // Only print if driver is initialized (PrintAllFaults requires initialization)
  if (fault_detected && print_faults && initialized_) {
    comm_.Log(LogLevel::Warn, "TLE92466ED",
              "⚠️  Fault detected on FAULTN pin - Printing detailed fault report:");
    comm_.Log(LogLevel::Warn, "TLE92466ED", "");
    if (auto print_result = PrintAllFaults(); !print_result) {
      comm_.Log(LogLevel::Warn, "TLE92466ED",
                "⚠️  Failed to print detailed fault report: error code %u",
                static_cast<unsigned>(print_result.error()));
    }
  }

  return DriverResult<bool>{fault_detected};
}

//==========================================================================
// DIAGNOSTIC HELPERS
//==========================================================================

template <typename CommType>
void Driver<CommType>::diagnoseClockConfiguration() noexcept {
  // Read CLK_DIV register to check clock configuration
  // This helps diagnose clock-related critical faults early
  auto clk_div_result = ReadRegister(CentralReg::CLK_DIV, false); // Don't verify CRC during init

  if (!clk_div_result) {
    comm_.Log(LogLevel::Warn, "TLE92466ED",
              "Failed to read CLK_DIV register (error: %u) - continuing anyway",
              static_cast<unsigned>(clk_div_result.error()));
    return;
  }

  auto clk_div = static_cast<uint16_t>(*clk_div_result);

  // Parse CLK_DIV register fields
  bool ext_clk = (clk_div & 0x8000) != 0;     // Bit 15: EXT_CLK
  uint8_t pll_refdiv = (clk_div >> 9) & 0x3F; // Bits 14:9: PLL_REFDIV (6 bits)
  uint16_t pll_fbdiv = clk_div & 0x01FF;      // Bits 8:0: PLL_FBDIV (9 bits)

  comm_.Log(LogLevel::Info, "TLE92466ED",
            "═══════════════════════════════════════════════════════════");
  comm_.Log(LogLevel::Info, "TLE92466ED", "CLK_DIV Register (0x0019): 0x%04X", clk_div);
  comm_.Log(LogLevel::Info, "TLE92466ED", "  Bit 15 (EXT_CLK): %d (%s)", ext_clk ? 1 : 0,
            ext_clk ? "External Clock (CLK-pin)" : "Internal Oscillator");
  comm_.Log(LogLevel::Info, "TLE92466ED", "  Bits 14:9 (PLL_REFDIV): %d (0x%02X)", pll_refdiv,
            pll_refdiv);
  comm_.Log(LogLevel::Info, "TLE92466ED", "  Bits 8:0 (PLL_FBDIV): %d (0x%03X)", pll_fbdiv,
            pll_fbdiv);

  // Calculate system clock frequency if external clock is used
  if (ext_clk && pll_refdiv > 0 && pll_fbdiv > 0) {
    // fSYS = fCLK * (PLL_FBDIV) / (2 * PLL_REFDIV)
    // We don't know fCLK, but we can show the divider ratio
    float divider_ratio = static_cast<float>(pll_fbdiv) / (2.0F * static_cast<float>(pll_refdiv));
    comm_.Log(LogLevel::Info, "TLE92466ED", "  PLL Divider Ratio: %.3f (fSYS = fCLK * %.3f)",
              divider_ratio, divider_ratio);
    comm_.Log(LogLevel::Info, "TLE92466ED",
              "  Note: fCLK is the external clock frequency on CLK-pin");

    // Show expected fSYS for common fCLK values
    comm_.Log(LogLevel::Info, "TLE92466ED", "  Expected fSYS for common fCLK values:");
    for (float fclk_mhz = 1.0F; fclk_mhz <= 8.0F; fclk_mhz += 0.5F) {
      float fsys_mhz = fclk_mhz * divider_ratio;
      comm_.Log(LogLevel::Info, "TLE92466ED", "    fCLK=%.1f MHz -> fSYS=%.2f MHz", fclk_mhz,
                fsys_mhz);
    }
  } else if (!ext_clk) {
    comm_.Log(LogLevel::Info, "TLE92466ED", "  Using Internal Oscillator (PLL dividers ignored)");
    comm_.Log(LogLevel::Info, "TLE92466ED",
              "  System clock fSYS is generated from internal oscillator");
  } else {
    comm_.Log(LogLevel::Warn, "TLE92466ED",
              "  ⚠️  Invalid PLL divider values (PLL_REFDIV=%d, PLL_FBDIV=%d)", pll_refdiv,
              pll_fbdiv);
    comm_.Log(LogLevel::Warn, "TLE92466ED", "  This may cause clock watchdog faults!");
  }
  comm_.Log(LogLevel::Info, "TLE92466ED",
            "═══════════════════════════════════════════════════════════");
}

#ifdef TLE92466ED_HEADER_INCLUDED
// Included from header - namespace is already open, don't close it

//============================================================================
// PHASE 2-6 IMPLEMENTATIONS (clock/power/state, ICC integrator, dither,
// diagnostics completeness, atomic feedback readout)
//============================================================================

// ---------------------------- Phase 2 helpers -------------------------------

template <typename CommType>
DriverResult<void>
Driver<CommType>::ConfigureClockSource(ClockSource source, uint32_t f_clk_Hz) noexcept {
  if (auto r = checkInitialized(); !r) return tle::unexpected(r.error());
  if (auto r = checkConfigMode();   !r) return tle::unexpected(r.error());

  uint16_t clk_div_value = CLK_DIV::INTERNAL_OSC;
  if (source == ClockSource::ExternalClockPll) {
    if (f_clk_Hz < 1'000'000UL || f_clk_Hz > 8'000'000UL) {
      return tle::unexpected(DriverError::InvalidParameter);
    }
    auto cfg = CLK_DIV::CalculatePllFromExternalHz(f_clk_Hz);
    if (cfg.fbdiv == 0 && cfg.actual_f_sys_hz == 0) {
      return tle::unexpected(DriverError::InvalidParameter);
    }
    clk_div_value = CLK_DIV::BuildExternalPll(cfg.refdiv, cfg.fbdiv);
    comm_.Log(LogLevel::Info, "TLE92466ED",
              "ConfigureClockSource: external PLL (fCLK=%lu Hz, R=%u, N=%u, fSYS~=%lu Hz)",
              static_cast<unsigned long>(f_clk_Hz), cfg.refdiv, cfg.fbdiv,
              static_cast<unsigned long>(cfg.actual_f_sys_hz));
  } else {
    comm_.Log(LogLevel::Info, "TLE92466ED", "ConfigureClockSource: internal oscillator");
  }
  return WriteRegister(CentralReg::CLK_DIV, clk_div_value, false, false);
}

template <typename CommType>
DriverResult<SupplyVoltages> Driver<CommType>::ReadAllSupplyVoltages() noexcept {
  if (auto r = checkInitialized(); !r) return tle::unexpected(r.error());

  auto v1 = ReadRegister(CentralReg::FB_VOLTAGE1, false);
  if (!v1) return tle::unexpected(v1.error());
  auto v2 = ReadRegister(CentralReg::FB_VOLTAGE2, false);
  if (!v2) return tle::unexpected(v2.error());

  SupplyVoltages out{};
  out.vio_mV        = VOLTAGE_FEEDBACK::ExtractVioMillivolts(v1.value());
  out.vdd_mV        = VOLTAGE_FEEDBACK::ExtractVddMillivolts(v1.value());
  out.vbat_mV       = VOLTAGE_FEEDBACK::ExtractVbatMillivolts(v2.value());
  out.temperature_c = VOLTAGE_FEEDBACK::TemperatureCelsiusFromFbVoltage2(v2.value());
  return out;
}

template <typename CommType>
DriverResult<float> Driver<CommType>::GetCentralTemperatureCelsius() noexcept {
  auto r = ReadAllSupplyVoltages();
  if (!r) return tle::unexpected(r.error());
  return r->temperature_c;
}

template <typename CommType>
DriverResult<void> Driver<CommType>::SetVioLevel(VioLevel level) noexcept {
  if (auto r = checkInitialized(); !r) return tle::unexpected(r.error());
  if (auto r = checkConfigMode();   !r) return tle::unexpected(r.error());

  uint16_t base = GLOBAL_CONFIG::CRC_EN | GLOBAL_CONFIG::CLK_WD_EN;
  if (level == VioLevel::V5_0) base |= GLOBAL_CONFIG::VIO_SEL;
  auto w = WriteRegister(CentralReg::GLOBAL_CONFIG, base, false, false);
  if (!w) return tle::unexpected(w.error());
  vio_5v_mode_ = (level == VioLevel::V5_0);
  return {};
}

template <typename CommType>
DriverResult<OperationState> Driver<CommType>::GetOperationState() noexcept {
  if (!initialized_) return OperationState::Reset;
  auto fb = ReadRegister(CentralReg::FB_STAT, false);
  if (!fb) return tle::unexpected(fb.error());
  const bool init_done = (fb.value() & FB_STAT::INIT_DONE) != 0;
  if (!init_done) return OperationState::Reset;
  if (!mission_mode_) return OperationState::Config;
  // A critical-fault reply mode would surface via SPI frame error; approximate
  // via FAULTN pin sampling through IsFault().
  auto flt = IsFault(false);
  if (flt && flt.value()) return OperationState::CriticalFault;
  return OperationState::Mission;
}

template <typename CommType>
DriverResult<void>
Driver<CommType>::EnterMissionModeChecked(uint32_t timeout_ms) noexcept {
  if (auto r = EnterMissionMode(); !r) return r;
  const uint32_t step_us = 500U;
  const uint32_t steps   = (timeout_ms * 1000U) / step_us + 1U;
  for (uint32_t i = 0; i < steps; ++i) {
    auto fb = ReadRegister(CentralReg::FB_STAT, false);
    if (!fb) return tle::unexpected(fb.error());
    if (fb.value() & FB_STAT::INIT_DONE) return {};
    (void)comm_.Delay(step_us);
  }
  return tle::unexpected(DriverError::TimeoutError);
}

template <typename CommType>
DriverResult<SupplyMonitorSelfTestResult>
Driver<CommType>::RunSupplyMonitorSelfTest() noexcept {
  if (auto r = checkInitialized(); !r) return tle::unexpected(r.error());
  if (auto r = checkConfigMode();   !r) return tle::unexpected(r.error());

  SupplyMonitorSelfTestResult result{};

  const uint16_t base    = GLOBAL_CONFIG::CRC_EN | GLOBAL_CONFIG::CLK_WD_EN
                         | (vio_5v_mode_ ? GLOBAL_CONFIG::VIO_SEL : 0);
  struct Phase { uint16_t mask; bool* slot; const char* name; };
  const Phase phases[] = {
      {GLOBAL_CONFIG::UV_OV_SWAP,   &result.uv_ov_swap_ok, "UV_OV_SWAP"},
      {GLOBAL_CONFIG::V1V5_UV_TEST, &result.v1v5_uv_ok,    "V1V5_UV_TEST"},
      {GLOBAL_CONFIG::V1V5_OV_TEST, &result.v1v5_ov_ok,    "V1V5_OV_TEST"},
      {GLOBAL_CONFIG::OT_TEST,      &result.ot_test_ok,    "OT_TEST"},
  };

  bool overall = true;
  for (const auto& p : phases) {
    if (auto w = WriteRegister(CentralReg::GLOBAL_CONFIG,
                                static_cast<uint16_t>(base | p.mask), false, false); !w) {
      return tle::unexpected(w.error());
    }
    (void)comm_.Delay(1000); // 1 ms settle
    auto flt = HasAnyFault();
    // Expected: fault is ASSERTED during the test phase.
    const bool ok = flt && flt.value();
    *p.slot = ok;
    overall = overall && ok;
    // Clear so next phase starts clean.
    (void)ClearFaults();
  }
  // Restore base configuration
  (void)WriteRegister(CentralReg::GLOBAL_CONFIG, base, false, false);
  (void)ClearFaults();
  result.overall_pass = overall;
  return result;
}

// ---------------------------- Phase 3 helpers -------------------------------

template <typename CommType>
DriverResult<void>
Driver<CommType>::SetIntegratorLimits(Channel channel, uint16_t lim_value_abs,
                                      uint8_t auto_lim_value_abs) noexcept {
  if (auto r = checkInitialized(); !r) return tle::unexpected(r.error());
  if (!isValidChannelInternal(channel)) return tle::unexpected(DriverError::InvalidChannel);
  if (lim_value_abs     > INTEGRATOR_LIMIT::LIM_VALUE_ABS_MASK
      || auto_lim_value_abs > (INTEGRATOR_LIMIT::AUTO_LIM_VALUE_ABS_MASK
                                >> INTEGRATOR_LIMIT::AUTO_LIM_VALUE_ABS_SHIFT)) {
    return tle::unexpected(DriverError::InvalidParameter);
  }
  // Enforce AUTO_LIM > MIN_INT_THRESH+3 per datasheet \u00a74.6.2.3
  auto ctrl = ReadRegister(GetChannelRegister(channel, ChannelReg::CTRL), false);
  if (ctrl) {
    const int8_t min_int = static_cast<int8_t>(ctrl.value() & CH_CTRL_REG::MIN_INT_THRESH_MASK);
    if (static_cast<int>(auto_lim_value_abs) <= (min_int + 3)) {
      return tle::unexpected(DriverError::InvalidParameter);
    }
  }
  const uint16_t val = INTEGRATOR_LIMIT::Build(lim_value_abs, auto_lim_value_abs);
  return WriteRegister(GetChannelRegister(channel, ChannelReg::INTEGRATOR_LIMIT), val);
}

template <typename CommType>
DriverResult<void>
Driver<CommType>::SetPwmControllerKi(Channel channel, uint8_t ki) noexcept {
  if (auto r = checkInitialized(); !r) return tle::unexpected(r.error());
  if (!isValidChannelInternal(channel)) return tle::unexpected(DriverError::InvalidChannel);
  if (ki > 0x0F) return tle::unexpected(DriverError::InvalidParameter);
  const uint16_t addr = GetChannelRegister(channel, ChannelReg::PERIOD);
  auto cur = ReadRegister(addr, false);
  if (!cur) return tle::unexpected(cur.error());
  const uint16_t updated = static_cast<uint16_t>(
      (cur.value() & ~PERIOD::PWM_CTRL_PARAM_MASK)
      | ((static_cast<uint16_t>(ki) << PERIOD::PWM_CTRL_PARAM_SHIFT)
         & PERIOD::PWM_CTRL_PARAM_MASK));
  return WriteRegister(addr, updated);
}

template <typename CommType>
DriverResult<void>
Driver<CommType>::SetMinIntegratorThreshold(Channel channel, int8_t min_int_thresh) noexcept {
  if (auto r = checkInitialized(); !r) return tle::unexpected(r.error());
  if (!isValidChannelInternal(channel)) return tle::unexpected(DriverError::InvalidChannel);
  const uint16_t addr = GetChannelRegister(channel, ChannelReg::CTRL);
  auto cur = ReadRegister(addr, false);
  if (!cur) return tle::unexpected(cur.error());
  const uint16_t updated = static_cast<uint16_t>(
      (cur.value() & ~CH_CTRL_REG::MIN_INT_THRESH_MASK)
      | (static_cast<uint16_t>(static_cast<uint8_t>(min_int_thresh))
         & CH_CTRL_REG::MIN_INT_THRESH_MASK));
  return WriteRegister(addr, updated);
}

template <typename CommType>
DriverResult<void>
Driver<CommType>::SetManualOnTimeMode(Channel channel, float on_time_us) noexcept {
  if (auto r = checkInitialized(); !r) return tle::unexpected(r.error());
  if (!isValidChannelInternal(channel)) return tle::unexpected(DriverError::InvalidChannel);
  if (on_time_us <= 0.0F) return tle::unexpected(DriverError::InvalidParameter);

  // TON.TON_MANT uses DITHER_CLK_DIV.EXP as its exponent. Pick the
  // smallest EXP such that mantissa fits in 10 bits.
  uint8_t  exp  = 0;
  uint32_t mant = 0;
  for (uint8_t e = 0; e <= 15; ++e) {
    const float divisor_us = static_cast<float>(1ULL << e) * PERIOD::F_SYS_PERIOD_US;
    const float m = on_time_us / divisor_us;
    if (m <= static_cast<float>(TON::TON_MANT_MASK) && m >= 1.0F) {
      mant = static_cast<uint32_t>(m + 0.5F);
      exp  = e;
      break;
    }
  }
  if (mant == 0) return tle::unexpected(DriverError::InvalidParameter);

  // Ensure DITHER_CLK_DIV.EXP matches our selection.
  auto clkdiv_cur = ReadRegister(GetChannelRegister(channel, ChannelReg::DITHER_CLK_DIV), false);
  if (!clkdiv_cur) return tle::unexpected(clkdiv_cur.error());
  const uint16_t clkdiv_new = static_cast<uint16_t>(
      (clkdiv_cur.value() & ~DITHER_CLK_DIV::EXP_MASK)
      | ((static_cast<uint16_t>(exp) << DITHER_CLK_DIV::EXP_SHIFT) & DITHER_CLK_DIV::EXP_MASK));
  if (auto w = WriteRegister(GetChannelRegister(channel, ChannelReg::DITHER_CLK_DIV),
                              clkdiv_new); !w) {
    return tle::unexpected(w.error());
  }

  if (auto w = WriteRegister(GetChannelRegister(channel, ChannelReg::TON),
                              TON::Build(static_cast<uint16_t>(mant), 0u)); !w) {
    return tle::unexpected(w.error());
  }
  // Zero PERIOD_MANT in CTRL_INT_THRESH to enter manual on-time mode.
  return WriteRegister(GetChannelRegister(channel, ChannelReg::CTRL_INT_THRESH),
                        CTRL_INT_THRESH::Build(0, 0));
}

template <typename CommType>
DriverResult<void>
Driver<CommType>::SeedIntegratorThresholdFromFeedback(Channel channel) noexcept {
  if (auto r = checkInitialized(); !r) return tle::unexpected(r.error());
  if (!isValidChannelInternal(channel)) return tle::unexpected(DriverError::InvalidChannel);
  auto fb = ReadRegister(GetChannelRegister(channel, ChannelReg::FB_INT_THRESH), false);
  if (!fb) return tle::unexpected(fb.error());
  const int16_t seed16 = FB_INT_THRESH::Extract(fb.value());
  const int8_t  seed8  = static_cast<int8_t>(seed16 > 127 ? 127 : seed16 < -128 ? -128 : seed16);
  return WriteRegister(GetChannelRegister(channel, ChannelReg::CTRL_INT_THRESH),
                        CTRL_INT_THRESH::Build(seed8, 0));
}

// ---------------------------- Phase 4 helpers -------------------------------

template <typename CommType>
DriverResult<void>
Driver<CommType>::SetDitherAdvanced(Channel channel, const DitherSetup& setup,
                                    bool parallel_mode) noexcept {
  if (auto r = checkInitialized(); !r) return tle::unexpected(r.error());
  if (!isValidChannelInternal(channel)) return tle::unexpected(DriverError::InvalidChannel);
  if (setup.amplitude_mA < 0.0F || setup.frequency_Hz <= 0.0F) {
    return tle::unexpected(DriverError::InvalidParameter);
  }

  // Use the existing high-level ConfigureDither path to compute step/clk div.
  if (auto r = ConfigureDither(channel, setup.amplitude_mA, setup.frequency_Hz,
                                parallel_mode); !r) {
    return r;
  }

  // Layer the sync + deep + fast-meas flags on top of what ConfigureDither
  // produced.
  const uint16_t clkdiv_addr = GetChannelRegister(channel, ChannelReg::DITHER_CLK_DIV);
  auto clkdiv_cur = ReadRegister(clkdiv_addr, false);
  if (!clkdiv_cur) return tle::unexpected(clkdiv_cur.error());
  uint16_t clkdiv_new = clkdiv_cur.value()
                      & ~(DITHER_CLK_DIV::DITHER_PWM_SYNC_EN_BIT
                          | DITHER_CLK_DIV::DITHER_SETPOINT_SYNC_EN_BIT);
  if (setup.sync_with_pwm)      clkdiv_new |= DITHER_CLK_DIV::DITHER_PWM_SYNC_EN_BIT;
  if (setup.sync_with_setpoint) clkdiv_new |= DITHER_CLK_DIV::DITHER_SETPOINT_SYNC_EN_BIT;
  if (auto w = WriteRegister(clkdiv_addr, clkdiv_new); !w) return tle::unexpected(w.error());

  const uint16_t ctrl_addr = GetChannelRegister(channel, ChannelReg::DITHER_CTRL);
  auto ctrl_cur = ReadRegister(ctrl_addr, false);
  if (!ctrl_cur) return tle::unexpected(ctrl_cur.error());
  uint16_t ctrl_new = ctrl_cur.value()
                    & ~(DITHER_CTRL::DEEP_DITHER | DITHER_CTRL::FAST_MEAS_MASK);
  if (setup.deep_dither) ctrl_new |= DITHER_CTRL::DEEP_DITHER;
  switch (setup.fast_measure) {
    case FastMeasureMode::HalfPeriod:    ctrl_new |= DITHER_CTRL::FAST_MEAS_HALF; break;
    case FastMeasureMode::QuarterPeriod: ctrl_new |= DITHER_CTRL::FAST_MEAS_QUAD; break;
    case FastMeasureMode::FullPeriod:
    default:                             ctrl_new |= DITHER_CTRL::FAST_MEAS_DITH; break;
  }
  return WriteRegister(ctrl_addr, ctrl_new);
}

// ---------------------------- Phase 5 helpers -------------------------------

template <typename CommType>
DriverResult<bool> Driver<CommType>::isSlaveChannel(Channel channel) noexcept {
  const uint8_t idx = ToIndex(channel);
  auto par = isChannelParallel(channel);
  if (!par) return tle::unexpected(par.error());
  if (!par.value()) return DriverResult<bool>{false};
  // Within a parallel pair the \u201cslave\u201d is the odd-indexed half when
  // pairing is (ch,ch^1); per datasheet master/slave convention CH2, CH3, CH5
  // are slaves when their respective pairs are parallelised.
  return DriverResult<bool>{idx == 2 || idx == 3 || idx == 5};
}

template <typename CommType>
DriverResult<void>
Driver<CommType>::SetOlsgTimeout(Channel channel, uint8_t olsg_timeout_code) noexcept {
  if (auto r = checkInitialized(); !r) return tle::unexpected(r.error());
  if (!isValidChannelInternal(channel)) return tle::unexpected(DriverError::InvalidChannel);
  if (olsg_timeout_code > 0x3F) return tle::unexpected(DriverError::InvalidParameter);
  const uint16_t addr = GetChannelRegister(channel, ChannelReg::TON);
  auto cur = ReadRegister(addr, false);
  if (!cur) return tle::unexpected(cur.error());
  const uint16_t updated = static_cast<uint16_t>(
      (cur.value() & ~TON::OLSG_TIMEOUT_MASK)
      | ((static_cast<uint16_t>(olsg_timeout_code) << TON::OLSG_TIMEOUT_SHIFT)
         & TON::OLSG_TIMEOUT_MASK));
  return WriteRegister(addr, updated);
}

template <typename CommType>
DriverResult<void>
Driver<CommType>::SetOffStateDiagnostics(Channel channel, bool oc_diag_enabled,
                                         uint8_t ol_th_fixed) noexcept {
  if (auto r = checkInitialized(); !r) return tle::unexpected(r.error());
  if (!isValidChannelInternal(channel)) return tle::unexpected(DriverError::InvalidChannel);
  if (ol_th_fixed > 0x3F) return tle::unexpected(DriverError::InvalidParameter);
  const uint16_t addr = GetChannelRegister(channel, ChannelReg::CH_CONFIG);
  auto cur = ReadRegister(addr, false);
  if (!cur) return tle::unexpected(cur.error());
  uint16_t updated = static_cast<uint16_t>(
      cur.value() & ~(CH_CONFIG::OC_DIAG_EN | CH_CONFIG::OL_TH_FIXED_MASK));
  if (oc_diag_enabled) updated |= CH_CONFIG::OC_DIAG_EN;
  updated |= static_cast<uint16_t>(
      (static_cast<uint16_t>(ol_th_fixed) << CH_CONFIG::OL_TH_FIXED_SHIFT)
      & CH_CONFIG::OL_TH_FIXED_MASK);
  return WriteRegister(addr, updated);
}

template <typename CommType>
DriverResult<BistResult> Driver<CommType>::RunSffBist(uint32_t timeout_ms) noexcept {
  if (auto r = checkInitialized(); !r) return tle::unexpected(r.error());
  if (auto r = checkConfigMode();   !r) return tle::unexpected(r.error());

  BistResult res{};
  if (auto w = WriteRegister(CentralReg::SFF_BIST, SFF_BIST::EN, false, false); !w) {
    return tle::unexpected(w.error());
  }
  const uint32_t step_us = 500U;
  const uint32_t steps   = (timeout_ms * 1000U) / step_us + 1U;
  for (uint32_t i = 0; i < steps; ++i) {
    auto r = ReadRegister(CentralReg::SFF_BIST, false);
    if (!r) return tle::unexpected(r.error());
    res.raw = static_cast<uint16_t>(r.value());
    if (res.raw & SFF_BIST::DONE) {
      res.done                   = true;
      res.pass                   = (res.raw & SFF_BIST::FAIL) == 0;
      res.uncorrectable_reg_err  = (res.raw & SFF_BIST::UERR) != 0;
      res.correctable_reg_err    = (res.raw & SFF_BIST::CERR) != 0;
      return res;
    }
    (void)comm_.Delay(step_us);
  }
  return tle::unexpected(DriverError::TimeoutError);
}

template <typename CommType>
DriverResult<PinStatus> Driver<CommType>::ReadPinStatus() noexcept {
  if (auto r = checkInitialized(); !r) return tle::unexpected(r.error());
  auto r = ReadRegister(CentralReg::PIN_STAT, false);
  if (!r) return tle::unexpected(r.error());
  PinStatus out{};
  out.raw           = static_cast<uint16_t>(r.value());
  out.drv0          = (out.raw & PIN_STAT::DRV0) != 0;
  out.drv1          = (out.raw & PIN_STAT::DRV1) != 0;
  out.en            = (out.raw & PIN_STAT::EN)   != 0;
  out.faultn_driver = (out.raw & PIN_STAT::FAULTN) != 0;
  out.faultn_fb     = (out.raw & PIN_STAT::FAULTN_FB) != 0;
  return out;
}

template <typename CommType>
DriverResult<void>
Driver<CommType>::SetFaultMask(MaskableFault fault, bool enable) noexcept {
  if (auto r = checkInitialized(); !r) return tle::unexpected(r.error());
  if (auto r = checkConfigMode();   !r) return tle::unexpected(r.error());
  const uint8_t idx = FaultMaskIndex(fault);
  const uint16_t bit = FaultMaskBit(fault);
  uint16_t addr = CentralReg::FAULT_MASK0;
  switch (idx) {
    case 0: addr = CentralReg::FAULT_MASK0; break;
    case 1: addr = CentralReg::FAULT_MASK1; break;
    case 2: addr = CentralReg::FAULT_MASK2; break;
    default: return tle::unexpected(DriverError::InvalidParameter);
  }
  auto cur = ReadRegister(addr, false);
  if (!cur) return tle::unexpected(cur.error());
  const uint16_t updated = static_cast<uint16_t>(
      enable ? (cur.value() | bit) : (cur.value() & ~bit));
  return WriteRegister(addr, updated);
}

// ---------------------------- Phase 6 helpers -------------------------------

namespace detail {
constexpr uint16_t FbBitForChannel(Channel ch) noexcept {
  return static_cast<uint16_t>(1u << ToIndex(ch));
}
} // namespace detail

template <typename CommType>
DriverResult<ChannelFeedback>
Driver<CommType>::ReadChannelFeedback(Channel channel, uint32_t timeout_ms) noexcept {
  if (auto r = checkInitialized(); !r) return tle::unexpected(r.error());
  if (!isValidChannelInternal(channel)) return tle::unexpected(DriverError::InvalidChannel);

  const uint16_t bit = detail::FbBitForChannel(channel);

  // Freeze this channel's feedback registers.
  if (auto w = WriteRegister(CentralReg::FB_FRZ, bit, false, false); !w) {
    return tle::unexpected(w.error());
  }

  // Poll FB_UPD for this channel.
  const uint32_t step_us = 200U;
  const uint32_t steps   = (timeout_ms * 1000U) / step_us + 1U;
  bool updated = false;
  for (uint32_t i = 0; i < steps; ++i) {
    auto r = ReadRegister(CentralReg::FB_UPD, false);
    if (!r) {
      (void)WriteRegister(CentralReg::FB_FRZ, 0, false, false);
      return tle::unexpected(r.error());
    }
    if (r.value() & bit) { updated = true; break; }
    (void)comm_.Delay(step_us);
  }

  ChannelFeedback fb{};
  auto cleanup = [&](auto& res) {
    (void)WriteRegister(CentralReg::FB_FRZ, 0, false, false);
    return res;
  };
  if (!updated) {
    auto err = tle::unexpected(DriverError::TimeoutError);
    return cleanup(err);
  }

  auto fb_dc    = ReadRegister(GetChannelRegister(channel, ChannelReg::FB_DC),   false);
  auto fb_vbat  = ReadRegister(GetChannelRegister(channel, ChannelReg::FB_VBAT), false);
  auto fb_iavg  = ReadRegister(GetChannelRegister(channel, ChannelReg::FB_I_AVG),false);
  auto fb_imm   = ReadRegister(GetChannelRegister(channel, ChannelReg::FB_IMIN_IMAX), false);
  auto fb_pmm   = ReadRegister(GetChannelRegister(channel, ChannelReg::FB_PERIOD_MIN_MAX), false);
  auto fb_th    = ReadRegister(GetChannelRegister(channel, ChannelReg::FB_INT_THRESH), false);

  if (!fb_dc)  { auto e = tle::unexpected(fb_dc.error());  return cleanup(e); }
  if (!fb_vbat){ auto e = tle::unexpected(fb_vbat.error());return cleanup(e); }
  if (!fb_iavg){ auto e = tle::unexpected(fb_iavg.error());return cleanup(e); }

  fb.duty_cycle_permyriad = FB_FEEDBACK::ComputeDutyCycle(fb_dc.value());
  fb.avg_current_mA       = FB_FEEDBACK::ComputeAverageCurrent_mA(fb_iavg.value(), fb_dc.value());
  fb.avg_vbat_mV          = FB_FEEDBACK::ComputeVbatChannel_mV(fb_vbat.value(), fb_dc.value());
  if (fb_imm) {
    fb.imin_mA = FB_IMIN_IMAX::IMin_mA(fb_imm.value());
    fb.imax_mA = FB_IMIN_IMAX::IMax_mA(fb_imm.value());
  }
  if (fb_pmm) {
    fb.period_min_us = FB_PERIOD_MIN_MAX::PeriodMinMicroseconds(fb_pmm.value());
    fb.period_max_us = FB_PERIOD_MIN_MAX::PeriodMaxMicroseconds(fb_pmm.value());
  }
  if (fb_th) {
    fb.int_thresh_seed = FB_INT_THRESH::Extract(fb_th.value());
  }
  fb.period_seq = FB_FEEDBACK::ExtractMeasExp(fb_iavg.value());

  // Release freeze.
  (void)WriteRegister(CentralReg::FB_FRZ, 0, false, false);
  return fb;
}

template <typename CommType>
DriverResult<std::array<ChannelFeedback, 6>>
Driver<CommType>::ReadAllChannelFeedback(uint32_t timeout_ms) noexcept {
  std::array<ChannelFeedback, 6> out{};
  for (uint8_t i = 0; i < 6; ++i) {
    auto r = ReadChannelFeedback(static_cast<Channel>(i), timeout_ms);
    if (!r) return tle::unexpected(r.error());
    out[i] = r.value();
  }
  return out;
}

template <typename CommType>
DriverResult<int32_t>
Driver<CommType>::GetCalibrationAvgCurrent_mA(Channel channel) noexcept {
  if (auto r = checkInitialized(); !r) return tle::unexpected(r.error());
  if (!isValidChannelInternal(channel)) return tle::unexpected(DriverError::InvalidChannel);
  auto r = ReadRegister(GetChannelRegister(channel, ChannelReg::FB_I_AVG_s16), false);
  if (!r) return tle::unexpected(r.error());
  return FB_I_AVG_s16::ToMilliamps(r.value());
}

#else
// Compiled directly (shouldn't happen) - close the namespace
} // namespace tle92466ed
#endif

#endif // TLE92466ED_IMPL
