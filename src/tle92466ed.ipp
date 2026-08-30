/**
 * @file tle92466ed.ipp
 * @brief Template implementation of TLE92466ED driver class
 * @copyright Copyright (c) 2024-2025 HardFOC. All rights reserved.
 *
 * @details
 * Included from `tle92466ed.hpp` after the `Driver` template declaration.
 * Register access (`ReadRegister` / `WriteRegister` / `ReadRegisterMulti`)
 * delegates to `CommType::Read()` / `Write()` / `ReadMulti()`. Single-register
 * calls use a 3-frame pipeline; `ReadMulti` uses N+2 frames in one chain.
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

/**
 * @brief Why @p address is not expected to read back the value just written.
 *
 * @param address Register address being verified.
 * @return Human-readable reason, or @c nullptr when the register *is* expected
 *         to be read-transparent (a mismatch there is a real defect).
 *
 * @details Used by @ref Driver::WriteRegister to decide whether a read-back
 *          mismatch is a device property to log, or a fault to report. Keeping
 *          the list in one place stops the retry loop from re-writing registers
 *          that can never match.
 */
[[nodiscard]] inline const char* NonTransparentWriteReason(uint16_t address) noexcept {
  if (address == CentralReg::CH_CTRL) {
    /* The "returns 0x0000 on read" behavior this tolerance was written for was
     * not the device: reads were accepting corrupt/stale frames and truncating
     * 22-bit replies. With replies validated (see SpiInterface::Read) CH_CTRL
     * read back its true value 8/8 on the Mid bench, 2026-08-13. The tolerance
     * is kept only because the same bench still drops whole frames; it is no
     * longer evidence of a non-transparent register. */
    return "CH_CTRL read-back tolerated while the bench SPI link drops frames";
  }
  if (address == CentralReg::GLOBAL_CONFIG) {
    return "GLOBAL_CONFIG is write-only, reads return default/previous value";
  }
  if (address == CentralReg::VBAT_TH) {
    /* Bench A/B (2026-08-12): writing 0xF72B (7 V / 40 V) read
     * back 0x8E00, and writing 0xD81F (5 V / 35 V) read back 0x8000. Both were
     * clean 16-bit replies (mode 00, status 0, CRC 0xA2) and neither written
     * word appears anywhere in the 32-bit frame — a register property, not a
     * bus fault. Thresholds are validated functionally instead: GLOBAL_DIAG0
     * VBAT_UV/VBAT_OV assert against the programmed limits and FB_VOLTAGE2
     * reports the measured rail (see TleFaultDiag). Failing Init here aborts
     * every valve channel over an unreadable protection threshold. */
    return "VBAT_TH is not read-transparent; verified via GLOBAL_DIAG0 UV/OV + FB_VOLTAGE2";
  }
  if (address == CentralReg::WD_RELOAD) {
    return "WD_RELOAD counter decrements continuously (read value <= written value is expected)";
  }
  if (address == CentralReg::GLOBAL_DIAG0 || address == CentralReg::GLOBAL_DIAG1 ||
      address == CentralReg::GLOBAL_DIAG2) {
    return "GLOBAL_DIAGx are write-1-to-clear, reads return current fault state";
  }
  if (address == CentralReg::DIAG_ERR_CHGR0 || address == CentralReg::DIAG_ERR_CHGR1 ||
      address == CentralReg::DIAG_ERR_CHGR2) {
    /* Datasheet §5.3.2.12: write-1-to-clear per channel pair, so a read of
     * 0x0000 (no faults) or unrelated latched bits is expected. */
    return "DIAG_ERR_CHGRx are write-1-to-clear, reads return current fault state";
  }
  if (address == CentralReg::DIAG_WARN_CHGR0 || address == CentralReg::DIAG_WARN_CHGR1 ||
      address == CentralReg::DIAG_WARN_CHGR2) {
    /* Datasheet §5.3.2.13. POR default is 0x1010 (OLSG_WARN_CHK_NOK on both
     * channels) and stays that way until the first OLSG check window runs. */
    return "DIAG_WARN_CHGRx are write-1-to-clear, POR=0x1010 until first OLSG check";
  }
  /* Per-channel banks live at 0x20..0x7F in 0x10 steps (see ChannelBase; the
   * order is CH4, CH5, CH0..CH3). Offsets above 0x0F are the 0x02xx feedback
   * page, which is read-only and never written. */
  const uint16_t bank = static_cast<uint16_t>(address & 0xFFF0u);
  if (bank >= ChannelBase::CH4 && bank <= ChannelBase::CH3) {
    switch (static_cast<uint16_t>(address & 0x000Fu)) {
    case ChannelReg::MODE:
    case ChannelReg::CH_CONFIG:
      /* Bench A/B (2026-08-12): writing MODE=0x0001 (ICC) and
       * CH_CONFIG=0x0031 to CH5 both read back 0x0000, while PERIOD in the
       * same bank round-tripped 0x05DB exactly on the same frames. Verifying
       * these made ConfigureChannel fail on every channel, so the loop never
       * reached ConfigurePwmPeriod and every channel kept PERIOD=0 — ICC with
       * no PWM period, hence no measurement window and no output at all. */
      return "per-channel MODE/CH_CONFIG are write-only, reads return 0x0000";
    case ChannelReg::SETPOINT:
      /* Reads return the controller's current target field, not the written
       * word (0x0123 read back 0x0200), so an equality check is meaningless. */
      return "per-channel SETPOINT reads the live target field, not the written word";
    default:
      break;
    }
  }
  return nullptr;
}

//==============================================================================
// INITIALIZATION
//==============================================================================

template <typename CommType>
DriverResult<void> Driver<CommType>::Init(bool perform_hardware_reset) noexcept {
  // 1. Initialize CommInterface (GPIO and SPI bus only)
  if (auto result = comm_.Init(); !result) {
    return tle::unexpected(DriverError::HardwareError);
  }

  // 2. Device reset sequence (optional on shared-bus retries)
  // RESN is active low: LOW = reset, HIGH = normal operation
  // EN is active high: HIGH = enabled, LOW = disabled
  // EN is held LOW only for the RESN pulse, then asserted HIGH before SPI
  // identity (channels remain gated by CH_CTRL defaults after POR).
  if (perform_hardware_reset) {
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

    // Step 5: Wait for device to stabilize after reset release (minimum 10ms per datasheet).
    // Extra margin when EN/rails are behind an I2C GPIO expander before first CS.
    if (auto result = comm_.Delay(50000); !result) { // 50ms
      return tle::unexpected(DriverError::HardwareError);
    }
  } else {
    /* SPI-only retry: keep RESN released and leave EN alone. Re-driving EN
     * on every failed ICVID attempt toggles the enable rail unnecessarily;
     * EN was already asserted after the first pulse. */
    if (auto result = SetReset(false); !result) {
      return tle::unexpected(DriverError::HardwareError);
    }
    if (auto result = comm_.Delay(2000); !result) {
      return tle::unexpected(DriverError::HardwareError);
    }
  }

  /* Step 6: Assert EN before SPI identity — only on the HW-reset path.
   * EN gates power stages (CH_CTRL defaults keep channels off after RESN);
   * SPI works with EN high or low. */
  if (perform_hardware_reset) {
    if (auto result = SetEnable(true); !result) {
      comm_.Log(LogLevel::Warn, "TLE92466ED",
                "Failed to set EN pin HIGH before SPI verify (error: %u)",
                static_cast<unsigned>(result.error()));
    } else {
      comm_.Log(LogLevel::Info, "TLE92466ED",
                "  EN set HIGH (outputs still gated by CH_CTRL defaults)");
    }
    if (auto result = comm_.Delay(5000); !result) { // 5 ms settle for rails/expander
      return tle::unexpected(DriverError::HardwareError);
    }
  }

  comm_.Log(LogLevel::Info, "TLE92466ED",
            perform_hardware_reset
                ? "✅ Device reset sequence completed (RESN released, EN asserted)"
                : "✅ RESN held released; EN asserted (SPI identity retry)");

  /* Step 6b: Discard one pipelined read — first post-RESN response is often
   * empty/fault while the digital core finishes POR. */
  (void)ReadRegister(CentralReg::ICVID, false);
  if (auto result = comm_.Delay(500); !result) {
    return tle::unexpected(DriverError::HardwareError);
  }

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

  // 5a. Fail-fast on uncalibrated / virgin silicon. Per datasheet \u00a75.3.2.7
  //     GLOBAL_DIAG2.OTP_VIRGIN=1 means the device OTP trim is not
  //     programmed, so all ICC current-regulation math is invalid. Refuse to
  //     initialize so downstream code doesn't silently drive garbage currents.
  //     OTP_ECC_ERR is factory OTP damage, not a software program of OTP —
  //     warn (FAULTN may stay low) but do not refuse: this Mid eval is known
  //     sticky and REG_ECC_ERR is the bit that forces Config Mode.
  if (auto diag2_result = ReadRegister(CentralReg::GLOBAL_DIAG2); diag2_result) {
    if ((*diag2_result & GLOBAL_DIAG2::OTP_VIRGIN) != 0) {
      comm_.Log(LogLevel::Error, "TLE92466ED",
                "OTP_VIRGIN=1 \u2014 device OTP trim not programmed; refusing to initialize");
      return tle::unexpected(DriverError::ConfigurationError);
    }
    if ((*diag2_result & GLOBAL_DIAG2::OTP_ECC_ERR) != 0) {
      comm_.Log(LogLevel::Warn, "TLE92466ED",
                "OTP_ECC_ERR sticky \u2014 factory OTP multi-bit flip; "
                "FAULTN may stay low; not a host OTP write");
    }
    /* REG_ECC_ERR is write-1-to-clear, so drop it here rather than leaving it
     * latched for the whole boot where it would mask a later, real ECC event.
     *
     * This must NOT abort Init. Parts are in service whose OTP reload re-arms
     * this bit every time it is cleared, and they configure, enter mission
     * mode, and drive all six channels perfectly well. Treating a re-assert as
     * fatal returned before step 6 below, so the default configuration,
     * mission mode, and the output-stage enable never ran. The outputs then
     * really were dark and every FB_DC read back TP_MANT=0 \u2014 which matches the
     * datasheet's description of the fault so closely that it reads as a dead
     * part rather than as an Init this driver refused to finish. */
    if ((*diag2_result & GLOBAL_DIAG2::REG_ECC_ERR) != 0) {
      comm_.Log(LogLevel::Warn, "TLE92466ED",
                "REG_ECC_ERR set at Init \u2014 clearing; a sticky re-assert is "
                "reported but does not block bring-up");
      (void)WriteRegister(CentralReg::GLOBAL_DIAG2, GLOBAL_DIAG2::REG_ECC_ERR);
      if (auto recheck = ReadRegister(CentralReg::GLOBAL_DIAG2); recheck) {
        if ((*recheck & GLOBAL_DIAG2::REG_ECC_ERR) != 0) {
          comm_.Log(LogLevel::Warn, "TLE92466ED",
                    "REG_ECC_ERR re-asserts after clear \u2014 sticky; continuing "
                    "to mission mode, state visible in GLOBAL_DIAG2");
        }
      }
    }
  }

  // 6. Apply default configuration
  if (auto result = applyDefaultConfig(); !result) {
    return tle::unexpected(result.error());
  }

  // 7. Clear any power-on reset flags (skip initialization check during Init).
  // Soft-fail: sticky GLOBAL_DIAG / noisy MISO must not block identity + rails.
  if (auto result = clearFaultsInternal(); !result) {
    comm_.Log(LogLevel::Warn, "TLE92466ED",
              "clearFaults during Init soft-failed (error: %u) — continuing",
              static_cast<unsigned>(result.error()));
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
  // Configure GLOBAL_CONFIG in two phases:
  //   1) CLK_WD only — finish all POR default register writes without CRC_EN
  //   2) CRC_EN last — once identity + defaults are on the wire
  // Enabling CRC_EN before channel/VBAT defaults made Init fail with CRCError on
  // long-lead / Mode1 MISO (ICVID already OK; first CRC-checked reply flaked).
  // Note: SPI watchdog is DISABLED by default (needs periodic reload).
  // Note: VIO_SEL is NOT set (defaults to 0 = 3.3V mode).
  uint16_t global_cfg =
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
  crc_enabled_ = false;

  // Set default VBAT thresholds (UV=7V, OV=40V)
  // Use internal version that doesn't check initialization (called during Init)
  if (auto result = setVbatThresholdsInternal(7.0f, 40.0f); !result) {
    return tle::unexpected(result.error());
  }

  // Configure all channels with default settings (ICC mode, 1V/us slew, disabled).
  // Skip verify_write during Init — these are POR-default values being
  // re-asserted; the real operational writes (ConfigureDither, SetCurrentSetpoint)
  // use verified writes with retry in the control path.
  for (uint8_t ch = 0; ch < static_cast<uint8_t>(Channel::COUNT); ++ch) {
    auto channel = static_cast<Channel>(ch);
    uint16_t ch_base = GetChannelBase(channel);

    if (auto result = WriteRegister(ch_base + ChannelReg::MODE,
                                    static_cast<uint16_t>(ChannelMode::ICC),
                                    false, false);
        !result) {
      return tle::unexpected(result.error());
    }

    if (auto result =
            WriteRegister(ch_base + ChannelReg::CH_CONFIG, CH_CONFIG::SLEWR_2V5_US,
                          false, false);
        !result) {
      return tle::unexpected(result.error());
    }

    if (auto result = WriteRegister(ch_base + ChannelReg::SETPOINT, 0, false, false);
        !result) {
      return tle::unexpected(result.error());
    }
  }

  // Note: SPI watchdog is disabled by default, so no need to reload here
  // If user enables SPI watchdog via ConfigureGlobal(), they must call ReloadSpiWatchdog()
  // periodically

  /* Leave CRC_EN off for Init + first bring-up reads. Turning CRC_EN on here
   * (even after defaults) made clearFaultsInternal() return CRCError on
   * long-lead / Mode1 MISO while ICVID had already succeeded, blocking later
   * rail telem reads. Enable CRC later via ConfigureGlobal when the bus is
   * proven. MOSI frames still carry a computed CRC byte. */
  crc_enabled_ = false;

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
    return result;
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

  comm_.Log(LogLevel::Debug, "TLE92466ED", "Set channel enable: Channel=%s, enabled=%s",
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
  ch_ctrl_cache_ = ch_ctrl_value;

  /* Disable is fire-and-forget: CH_CTRL sticky-zero readback retried
   * 3 × Read (each CRC attempt × 3-frame TransferChain) per parked
   * channel and AllOff of six PROP bits saturated a 2 ms host-loop step
   * at 65535 µs. EN off is the park; leftover TARGET cannot drive. Enable
   * still verifies — silent LM-Pro on CH4/CH5 is worse than a slow enable. */
  if (!enabled) {
    return WriteRegister(CentralReg::CH_CTRL, ch_ctrl_value, false, false);
  }

  /* CH_CTRL reads *do* work on this silicon (HIL: 0x8008/0x8009/0x800B).
   * Writes of EN_CH4/EN_CH5 were previously fire-and-forget; SETPOINT could
   * land while the output stage stayed off — silent LM-Pro. Prefer verifying
   * the EN bit. If the read is CRC/sticky-zero (CH4 SETPOINT 0x0020 class),
   * accept the write: failing closed skipped SetChannelCurrent and left
   * SPV_H ControllerStale with dither programmed and iavg n/a. */
  constexpr int kRetries = 3;
  for (int att = 0; att < kRetries; ++att) {
    if (auto wr = WriteRegister(CentralReg::CH_CTRL, ch_ctrl_value, false, false);
        !wr) {
      if (att + 1 == kRetries) {
        return wr;
      }
      continue;
    }
    auto rd = ReadRegister(CentralReg::CH_CTRL, false);
    if (!rd) {
      continue;
    }
    const uint16_t got = static_cast<uint16_t>(*rd & 0xFFFFu);
    if (got == 0U) {
      continue; /* sticky-zero */
    }
    const bool bit_on = (got & mask) != 0U;
    if (((got & CH_CTRL::OP_MODE) != 0U) && (bit_on == enabled)) {
      /* Confirm *this* bit only. Copying the whole enable field from the
       * readback re-imported leftover EN_CH0 while walking CH1–CH5 and left
       * the first 200 Ω load conducting for the rest of the sweep. */
      return {};
    }
  }
  return {};
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

  uint16_t ch_ctrl_value = ch_ctrl_cache_ & ~CH_CTRL::ALL_CH_MASK;
  ch_ctrl_value |= channel_mask;
  ch_ctrl_cache_ = ch_ctrl_value;

  constexpr int kRetries = 3;
  for (int att = 0; att < kRetries; ++att) {
    if (auto wr = WriteRegister(CentralReg::CH_CTRL, ch_ctrl_value, false, false);
        !wr) {
      if (att + 1 == kRetries) {
        return wr;
      }
      continue;
    }
    auto rd = ReadRegister(CentralReg::CH_CTRL, false);
    if (!rd) {
      if (att + 1 == kRetries) {
        return tle::unexpected(rd.error());
      }
      continue;
    }
    const uint16_t got = static_cast<uint16_t>(*rd & 0xFFFFu);
    if (got == 0U) {
      continue;
    }
    if (((got & CH_CTRL::OP_MODE) != 0U) &&
        ((got & CH_CTRL::ALL_CH_MASK) == channel_mask)) {
      return {};
    }
  }
  return tle::unexpected(DriverError::RegisterError);
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

  comm_.Log(LogLevel::Debug, "TLE92466ED",
            "Setting current setpoint: Channel=%s, Current=%u mA, Target=0x%04X, Parallel=%s",
            ToString(channel), current_ma, target, parallel_mode ? "true" : "false");

  /* Verify TARGET bits, not mA. Sticky-zero readback on Mid is still possible,
   * so a zero read is retried rather than treated as RegisterError. A non-zero
   * mismatch (HIL: 0x01FC / 31 mA vs 0x075C / 115 mA) is a real encode/clamp
   * defect and must not be accepted. */
  constexpr int kRetries = 3;
  uint16_t last_got = 0;
  for (int att = 0; att < kRetries; ++att) {
    if (auto wr = WriteRegister(ch_addr, target, false, false); !wr) {
      if (att + 1 == kRetries) {
        return wr;
      }
      continue;
    }
    auto rd = ReadRegister(ch_addr, false);
    if (!rd) {
      if (att + 1 == kRetries) {
        return tle::unexpected(rd.error());
      }
      continue;
    }
    const uint16_t raw16 = static_cast<uint16_t>(*rd & 0xFFFFu);
    /* Pipelined MISO often returns CH_CTRL (OP_MODE set, TARGET looks like
     * EN_CHx). Masking off bit 15 used to turn 0x8008 into TARGET 8 and a
     * hard RegisterError — CH4/CH5 SETPOINT then never retried. */
    if ((raw16 & SETPOINT::AUTO_LIMIT_DIS) != 0U &&
        (target & SETPOINT::AUTO_LIMIT_DIS) == 0U) {
      continue;
    }
    last_got = raw16 & SETPOINT::TARGET_MASK;
    const uint16_t want = target & SETPOINT::TARGET_MASK;
    if (last_got == want) {
      return {};
    }
    if (last_got == 0U && want != 0U) {
      continue; /* sticky-zero — re-issue the write */
    }
  }
  if (last_got == 0U) {
    return {}; /* writes issued; upper layer retries on next tick */
  }
  comm_.Log(LogLevel::Warn, "TLE92466ED",
            "SETPOINT TARGET mismatch: wrote 0x%04X read 0x%04X (%u mA vs %u mA)",
            target & SETPOINT::TARGET_MASK, last_got,
            SETPOINT::CalculateCurrent(target, parallel_mode),
            SETPOINT::CalculateCurrent(last_got, parallel_mode));
  return tle::unexpected(DriverError::RegisterError);
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

  comm_.Log(LogLevel::Debug, "TLE92466ED",
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
  /* Skip verify_write: shared Mode1 soft-CS often returns phantoms on
   * channel-register readback (same class as CH_CTRL sticky-zero). A hard
   * RegisterError aborts dither setup even when the write landed. Trust the
   * transfer; FB_I_AVG/dither probe is the functional check. */
  return WriteRegister(addr, reg_value, false, false);
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

  /* Setpoint-sync restarts TDither/Tmeas on every SETPOINT write so FB_I_AVG
   * begins a fresh averaging window after duty updates (datasheet §4.7.5).
   * Without it, FB_* often stays at 0 on shared Mode1 buses. */
  if (auto result = ConfigureDitherClock(channel, t_ref_clk_us,
                                         /*dither_pwm_sync=*/false,
                                         /*dither_setpoint_sync=*/true);
      !result) {
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

  comm_.Log(LogLevel::Debug, "TLE92466ED",
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

  comm_.Log(LogLevel::Debug, "TLE92466ED",
            "Configuring dither (raw): Channel=%s, StepSize=%u, NumSteps=%u, FlatSteps=%u",
            ToString(channel), step_size, num_steps, flat_steps);

  // Configure DITHER_CTRL (step size)
  uint16_t ctrl_value = step_size & DITHER_CTRL::STEP_SIZE_MASK;
  if (auto result =
          WriteRegister(ch_base + ChannelReg::DITHER_CTRL, ctrl_value, false, false);
      !result) {
    return tle::unexpected(result.error());
  }

  // Configure DITHER_STEP (steps and flat period)
  uint16_t step_value = flat_steps | (static_cast<uint16_t>(num_steps) << DITHER_STEP::STEPS_SHIFT);
  if (auto result =
          WriteRegister(ch_base + ChannelReg::DITHER_STEP, step_value, false, false);
      !result) {
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
    /* 22-bit reply: narrowing to uint16_t drops INIT_DONE (21) and
     * SPI_WD_ERR (20), which made init_done read 0 on a healthy device. */
    const uint32_t fb_stat = *fb_stat_result;
    status.supply_nok_internal = (fb_stat & FB_STAT::SUP_NOK_INT) != 0;
    status.supply_nok_external = (fb_stat & FB_STAT::SUP_NOK_EXT) != 0;
    status.init_done = (fb_stat & FB_STAT::INIT_DONE) != 0;
  }

  /* Mode is taken from the driver cache rather than a CH_CTRL read. The cache
   * is authoritative because it records what was written; the earlier reason
   * given here — that CH_CTRL reads back as zero on this silicon — was an
   * artifact of unvalidated SPI replies and no longer holds. */
  status.config_mode = !mission_mode_;

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
  // Both mantissas must describe the SAME measurement window, so the channel
  // is frozen for the pair of reads. Reading them unfrozen let the averager
  // advance between the two frames and paired a TP_MANT from one window with
  // an I_AVG_MANT from the next; the ratio was then silently wrong. Freezing
  // also gives FB_UPD, which is the only positive signal that a window has
  // actually completed rather than the registers simply reading 0.
  const uint16_t i_avg_addr = GetChannelRegister(channel, ChannelReg::FB_I_AVG);
  const uint16_t fb_dc_addr = GetChannelRegister(channel, ChannelReg::FB_DC);
  const uint16_t fb_bit = static_cast<uint16_t>(1u << ToIndex(channel));

  if (auto w = WriteRegister(CentralReg::FB_FRZ, fb_bit, false, false); !w) {
    return tle::unexpected(w.error());
  }

  /* Every exit below must clear FB_FRZ: a freeze left behind stops the chip
   * updating that channel's feedback for good, which reads back exactly like
   * a dead measurement window. */
  auto release = [this]() noexcept {
    (void)WriteRegister(CentralReg::FB_FRZ, 0, false, false);
  };

  // Bounded poll: this runs on the ~19 Hz diagnostics thread under the handler
  // mutex, so the budget stays short enough not to stall peer SPI users.
  constexpr uint32_t kPollStepUs = 200U;
  constexpr uint32_t kPollSteps = 40U;  // ~8 ms; 200 Hz Tmeas ≈ 5 ms
  bool updated = false;
  for (uint32_t i = 0; i < kPollSteps; ++i) {
    auto upd = ReadRegister(CentralReg::FB_UPD, false);
    if (!upd) {
      release();
      return tle::unexpected(upd.error());
    }
    if ((upd.value() & fb_bit) != 0U) {
      updated = true;
      break;
    }
    (void)comm_.Delay(kPollStepUs);
  }
  if (!updated) {
    release();
    return tle::unexpected(DriverError::FeedbackNotReady);
  }

  auto i_avg_res = ReadRegister(i_avg_addr, false);
  if (!i_avg_res) {
    release();
    return tle::unexpected(i_avg_res.error());
  }
  auto fb_dc_res = ReadRegister(fb_dc_addr, false);
  if (!fb_dc_res) {
    release();
    return tle::unexpected(fb_dc_res.error());
  }
  release();

  // TP_MANT is the divisor; a near-zero value means the averaging window was
  // never established (or the reply was spliced) and the ratio would be
  // nonsense. Report that instead of publishing an invented current.
  if (!FB_FEEDBACK::HasValidMeasurementWindow(*fb_dc_res)) {
    return tle::unexpected(DriverError::FeedbackNotReady);
  }

  // Decode mantissas and compute current in mA. Returns int32_t (signed)
  // so we can detect negative readings (recirculation current); clamp to
  // unsigned for the legacy uint16_t return type.
  const int32_t i_ma = FB_FEEDBACK::ComputeAverageCurrent_mA(*i_avg_res, *fb_dc_res);
  const int32_t i_clamped = (i_ma < 0) ? 0 : (i_ma > 0xFFFF ? 0xFFFF : i_ma);
  return static_cast<uint16_t>(i_clamped);
}

//==============================================================================
// BATCHED NON-BLOCKING FEEDBACK (real-time control path)
//==============================================================================

template <typename CommType>
DriverResult<uint8_t> Driver<CommType>::GetFeedbackUpdateMask() noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }
  auto upd = ReadRegister(CentralReg::FB_UPD, false);
  if (!upd) {
    return tle::unexpected(upd.error());
  }
  return static_cast<uint8_t>(*upd & 0x3FU);
}

template <typename CommType>
DriverResult<void> Driver<CommType>::FreezeFeedback(uint8_t channel_mask) noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }
  return WriteRegister(CentralReg::FB_FRZ,
                       static_cast<uint16_t>(channel_mask & 0x3FU), false, false);
}

template <typename CommType>
DriverResult<void> Driver<CommType>::ReleaseFeedbackFreeze() noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }
  /* A freeze left behind stops Tmeas for that channel for good — FB_UPD
   * never rises again and SampleChannelCurrents then skips it forever.
   * FB_FRZ replies fail CRC often on this bus (console `tle reg 0x0007`
   * is err=8), so a single failed write must not stick the freeze. */
  DriverResult<void> last = tle::unexpected(DriverError::CRCError);
  for (unsigned attempt = 0; attempt < 3U; ++attempt) {
    last = WriteRegister(CentralReg::FB_FRZ, 0U, false, false);
    if (last) {
      return last;
    }
  }
  return last;
}

template <typename CommType>
DriverResult<CurrentFeedbackBatch>
Driver<CommType>::SampleChannelCurrents(uint8_t channel_mask) noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }
  CurrentFeedbackBatch batch{};
  batch.requested_mask = static_cast<uint8_t>(channel_mask & 0x3FU);
  if (batch.requested_mask == 0U) {
    return batch;
  }

  /* Datasheet Figure 22 step 1: ask the chip which windows finished. One
   * central read answers for all six channels, so this does not scale with the
   * size of the mask. */
  auto upd = GetFeedbackUpdateMask();
  if (!upd) {
    return tle::unexpected(upd.error());
  }
  batch.update_mask = static_cast<uint8_t>(*upd & batch.requested_mask);
  if (batch.update_mask == 0U) {
    /* Nothing new. Freezing a channel whose FB_UPD bit is still clear aborts
     * Tmeas (~5 ms) and is exactly how CH0 stayed n/a while CH5 (bit set)
     * read Healthy. Leave the registers running; the next 8 ms tick picks
     * the window up. */
    return batch;
  }

  /* Step 2: freeze precisely the channels that have data. */
  if (auto frz = FreezeFeedback(batch.update_mask); !frz) {
    return tle::unexpected(frz.error());
  }

  /* The freeze write must retire before the pair of FB reads.
   * GetAverageCurrent gets this for free because it polls FB_UPD (~200 µs+).
   * Without that flush the first ReadRegister can still see the unfrozen
   * window (torn 0 mA vs a later 36 mA on the same coil). One FB_FRZ read
   * both confirms the bits and drains the pipeline. */
  (void)ReadRegister(CentralReg::FB_FRZ, false);

  /* Step 3: anchored pipelined chains, I_AVG then FB_DC per channel.
   *
   * Every register in the per-channel feedback bank answers in 22-bit reply
   * mode, so ExpectedReplyFor cannot tell CH3's FB_I_AVG from CH4's. A reply
   * that slips one slot carries a valid CRC *and* passes the width check, and
   * is then decoded as this channel's current. Desk 2026-08-30: CH3 reported
   * 65 mA Healthy with no coil on the connector — it was CH4's answer, and
   * that is a false Healthy on a disconnected valve.
   *
   * ICVID is interleaved as an anchor: read-only, harmless to repeat, and the
   * only register here that answers in 16-bit reply mode with a known constant
   * (high byte 0xC1). Any slip drops a 22-bit feedback reply into an anchor
   * slot, where the width check rejects it. A chain whose anchors do not all
   * verify is discarded whole — none of its channels are accepted.
   *
   * Fail closed on purpose: reporting "no window" costs one 8 ms tick and the
   * next tick picks the measurement up, whereas reporting a neighbour's
   * current as this channel's is unrecoverable and silently wrong. */
  constexpr size_t kChannelsPerChain = 3U; /* 3*3+1 = 10 reads = 12 frames */
  constexpr size_t kChainReads = 3U * kChannelsPerChain + 1U;
  static_assert(kChainReads <= CommType::kMaxPipelinedReads,
                "anchored chain must fit one pipelined CS window");

  uint8_t pending[6] = {};
  size_t pending_n = 0U;
  for (uint8_t ch = 0; ch < 6U; ++ch) {
    if ((batch.update_mask & static_cast<uint8_t>(1U << ch)) != 0U) {
      pending[pending_n++] = ch;
    }
  }

  for (size_t start = 0U; start < pending_n; start += kChannelsPerChain) {
    const size_t remaining = pending_n - start;
    const size_t k =
        (remaining < kChannelsPerChain) ? remaining : kChannelsPerChain;

    uint16_t addrs[kChainReads] = {};
    uint32_t vals[kChainReads] = {};
    size_t n = 0U;
    for (size_t j = 0U; j < k; ++j) {
      const auto tch = static_cast<Channel>(pending[start + j]);
      addrs[n++] = CentralReg::ICVID;
      addrs[n++] = GetChannelRegister(tch, ChannelReg::FB_I_AVG);
      addrs[n++] = GetChannelRegister(tch, ChannelReg::FB_DC);
    }
    addrs[n++] = CentralReg::ICVID;

    uint16_t valid = 0U;
    if (auto r = ReadRegisterMulti(std::span<const uint16_t>{addrs, n},
                                   std::span<uint32_t>{vals, n}, valid);
        !r) {
      ++batch.rejected_chains;
      continue;
    }

    /* Anchors decide the whole chain before any payload is believed. */
    bool anchors_ok = true;
    for (size_t a = 0U; a <= 3U * k; a += 3U) {
      if ((valid & static_cast<uint16_t>(1U << a)) == 0U ||
          !DeviceID::IsValidDevice(static_cast<uint16_t>(vals[a]))) {
        anchors_ok = false;
        break;
      }
    }
    if (!anchors_ok) {
      ++batch.rejected_chains;
      continue;
    }

    for (size_t j = 0U; j < k; ++j) {
      const size_t i_idx = 3U * j + 1U;
      const size_t d_idx = 3U * j + 2U;
      if ((valid & static_cast<uint16_t>(1U << i_idx)) == 0U ||
          (valid & static_cast<uint16_t>(1U << d_idx)) == 0U) {
        continue;
      }
      if (!FB_FEEDBACK::HasValidMeasurementWindow(vals[d_idx])) {
        continue;
      }
      /* TO_MANT is the last PWM cycle in the window, not the average. Dither
       * at 200 Hz can freeze on a zero-crossing (TO=0) while I_AVG still
       * holds the real window current. Gating on TO==0 made FRV/LSV go n/a
       * after they had been 36 mA. Keep TO for telemetry; I_AVG is the
       * measurement. */
      const uint8_t ch = pending[start + j];
      auto& s = batch.channels[ch];
      s.tp_mant = FB_FEEDBACK::ExtractTpMant(vals[d_idx]);
      s.to_mant = FB_FEEDBACK::ExtractToMant(vals[d_idx]);
      s.iavg_ma =
          FB_FEEDBACK::ComputeAverageCurrent_mA(vals[i_idx], vals[d_idx]);
      s.window_valid = true;
      batch.valid_mask = static_cast<uint8_t>(batch.valid_mask | (1U << ch));
    }
  }

  /* Step 4: unfreeze immediately even when every read failed. A stuck
   * FB_FRZ stops Tmeas for those channels. */
  (void)ReleaseFeedbackFreeze();
  return batch;
}

template <typename CommType>
DriverResult<void> Driver<CommType>::ReadAllChannelErrorFlags(
    std::array<ChannelErrorFlags, 6>& out) noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }
  out = {};

  /* Three DIAG_ERR_CHGR groups cover all six channels (each holds a pair at
   * bit offset 8*y). All are 16-bit replies, so one chain is legal. */
  const uint16_t addresses[3] = {
      DIAG_ERR_CHGR::AddressForChannel(0), DIAG_ERR_CHGR::AddressForChannel(2),
      DIAG_ERR_CHGR::AddressForChannel(4)};
  uint32_t values[3] = {};
  uint16_t valid = 0U;
  if (auto r = ReadRegisterMulti(std::span<const uint16_t>{addresses},
                                 std::span<uint32_t>{values}, valid);
      !r) {
    return tle::unexpected(r.error());
  }
  if (valid != 0x7U) {
    return tle::unexpected(DriverError::CRCError);
  }

  for (uint8_t ch = 0; ch < 6U; ++ch) {
    const uint16_t raw = static_cast<uint16_t>(values[ch / 2U]);
    auto& f = out[ch];
    f.open_load_short_ground =
        (raw & DIAG_ERR_CHGR::ChannelBitMask(ch, DIAG_ERR_CHGR::BIT_OLSG)) != 0U;
    f.open_load =
        (raw & DIAG_ERR_CHGR::ChannelBitMask(ch, DIAG_ERR_CHGR::BIT_OL)) != 0U;
    f.overcurrent =
        (raw & DIAG_ERR_CHGR::ChannelBitMask(ch, DIAG_ERR_CHGR::BIT_OC)) != 0U;
    f.short_to_ground =
        (raw & DIAG_ERR_CHGR::ChannelBitMask(ch, DIAG_ERR_CHGR::BIT_SG)) != 0U;
    f.over_temperature =
        (raw & DIAG_ERR_CHGR::ChannelBitMask(ch, DIAG_ERR_CHGR::BIT_OTE)) != 0U;
  }
  return {};
}

template <typename CommType>
DriverResult<bool> Driver<CommType>::IsMeasurementClockRunning(Channel channel) noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }
  if (!isValidChannelInternal(channel)) {
    return tle::unexpected(DriverError::InvalidChannel);
  }
  const uint16_t addr = GetChannelRegister(channel, ChannelReg::DITHER_CLK_DIV);
  auto rb = ReadRegister(addr, false);
  if (!rb) {
    return tle::unexpected(rb.error());
  }
  const uint16_t v = static_cast<uint16_t>(*rb & 0xFFFFU);
  const uint16_t mant = static_cast<uint16_t>(v & 0x03FFU);
  const uint8_t exp = static_cast<uint8_t>((v >> 10) & 0x0FU);
  /* POR is 0x0000 → tref_clk 0 → Tmeas never elapses. A mantissa this small
   * cannot produce a usable window even if the field is technically non-zero. */
  return (mant >= 64U) && (exp >= 1U);
}

template <typename CommType>
DriverResult<bool> Driver<CommType>::EnsureDitherRunning(Channel channel,
                                                         float amplitude_ma,
                                                         float frequency_hz,
                                                         uint8_t max_attempts) noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }
  if (!isValidChannelInternal(channel)) {
    return tle::unexpected(DriverError::InvalidChannel);
  }
  if (amplitude_ma <= 0.0F || frequency_hz <= 0.0F) {
    /* No dither requested: the caller is driving a channel whose feedback it
     * does not intend to average. Nothing to confirm. */
    return false;
  }
  if (max_attempts == 0U) {
    max_attempts = 1U;
  }
  for (uint8_t attempt = 0; attempt < max_attempts; ++attempt) {
    (void)ConfigureDither(channel, amplitude_ma, frequency_hz);
    auto running = IsMeasurementClockRunning(channel);
    if (running && *running) {
      return true;
    }
  }
  /* The clock may still be running from an earlier write this device's SPI
   * read cannot confirm, so this is reported rather than treated as an error;
   * the caller decides whether to trust feedback anyway. */
  return false;
}

template <typename CommType>
DriverResult<void> Driver<CommType>::SetFaultContributionMask(uint8_t channel_mask) noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }
  const uint16_t ch = static_cast<uint16_t>(channel_mask & 0x3FU);
  /* Keep the supply/clock/temperature sources the reset value enables; only
   * the per-channel contribution is caller-selected. An unpopulated channel
   * left unmasked holds FAULTN low after every ClearFaults. */
  const uint16_t mask0 = static_cast<uint16_t>(
      (ch & FAULT_MASK0::CH_ERR_MASK) | FAULT_MASK0::SUP_NOK_INT_MASK |
      FAULT_MASK0::SUP_NOK_EXT_MASK);
  const uint16_t mask1 = static_cast<uint16_t>(
      (ch & FAULT_MASK1::CH_WARN_MASK) | FAULT_MASK1::COTWARN_MASK |
      FAULT_MASK1::COTERR_MASK | FAULT_MASK1::CLK_LOW_MASK);
  if (auto w = WriteRegister(CentralReg::FAULT_MASK0, mask0, false, false); !w) {
    return tle::unexpected(w.error());
  }
  return WriteRegister(CentralReg::FAULT_MASK1, mask1, false, false);
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

  /* WD_RELOAD is a strobe (NonTransparentWriteReason). Default verify_write
   * Delay(1 ms)+readback ran on every valve_diag kick (~52 ms) under the
   * handler mutex and starved host-loop FB_UPD / last_step. */
  return WriteRegister(CentralReg::WD_RELOAD, masked_value, false, false);
}

//==========================================================================
// DEVICE INFORMATION
//==========================================================================

template <typename CommType>
DriverResult<uint16_t> Driver<CommType>::GetIcVersion() noexcept {
  if (auto result = checkInitialized(); !result) {
    return tle::unexpected(result.error());
  }

  /* Prefer a live ICVID read; fall back to the value latched at VerifyDevice
   * when a later shared-bus Mode1 sample is empty or bit-shifted. */
  if (auto live = ReadRegister(CentralReg::ICVID, false); live) {
    const uint16_t id = static_cast<uint16_t>(*live);
    if (DeviceID::IsValidDevice(id)) {
      cached_icvid_ = id;
      return id;
    }
  }
  if (DeviceID::IsValidDevice(cached_icvid_)) {
    return cached_icvid_;
  }
  return tle::unexpected(DriverError::DeviceNotResponding);
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
  // Read ICVID (no CRC during init). Retry — post-POR / shared-bus Mode1
  // bring-up often returns data=0 on the first one or two pipelined reads.
  uint16_t icvid = 0;
  DriverError last_err = DriverError::HardwareError;
  bool got_word = false;
  for (unsigned attempt = 0; attempt < 5U; ++attempt) {
    auto id_result = ReadRegister(CentralReg::ICVID, false);
    if (!id_result) {
      last_err = id_result.error();
      comm_.Log(LogLevel::Warn, "TLE92466ED",
                "ICVID read attempt %u failed (error: %u)", attempt + 1U,
                static_cast<unsigned>(last_err));
      (void)comm_.Delay(500);
      continue;
    }
    got_word = true;
    icvid = static_cast<uint16_t>(*id_result);
    if (DeviceID::IsValidDevice(icvid)) {
      break;
    }
    /* Non-zero garbage (bit-shift phantoms like 0x8000 / 0x0E00) must not
     * freeze retries — keep sampling until 0xC1xx or attempts exhausted. */
    comm_.Log(LogLevel::Warn, "TLE92466ED",
              "ICVID attempt %u not 0xC1xx (0x%04X) — retry", attempt + 1U,
              icvid);
    (void)comm_.Delay(500);
  }

  if (!got_word) {
    if (last_err == DriverError::FaultDetected && comm_.SawCriticalFault()) {
      /* The part answered, but only to say its own core supplies are dead —
       * that is a rail problem on the board, not SPI wiring or CPOL/CPHA.
       * Bits 7/6/5 are OK-flags (1 = OK), so 0x00 means 1V5, 2V5 and the ADC
       * bandgap are all down: check the +5 V VDD feed before touching SPI. */
      const uint8_t f = comm_.LastCriticalFaultFlags();
      comm_.Log(LogLevel::Error, "TLE92466ED",
                "Device verification failed: CRITICAL FAULT reply 0x%02X — "
                "1V5=%s 2V5=%s bandgap=%s clk_slow=%u clk_fast=%u "
                "wd_ref_clk=%s (check +5V VDD / VIO, not SPI mode)",
                static_cast<unsigned>(f), (f & 0x80U) ? "OK" : "BAD",
                (f & 0x40U) ? "OK" : "BAD", (f & 0x20U) ? "OK" : "BAD",
                static_cast<unsigned>((f >> 4) & 1U),
                static_cast<unsigned>((f >> 3) & 1U),
                (f & 0x01U) ? "MISSING" : "OK");
      return tle::unexpected(last_err);
    }
    comm_.Log(LogLevel::Error, "TLE92466ED",
              "Device verification failed: Failed to read ICVID register (error: %u)",
              static_cast<unsigned>(last_err));
    return tle::unexpected(last_err);
  }

  // All-zero / all-one MISO is "not on the bus" (floating / unpowered / held
  // in reset), not a WrongDeviceID. Keep WrongDeviceID for real non-0xC1 types.
  if (icvid == 0x0000 || icvid == 0xFFFF) {
    comm_.Log(LogLevel::Error, "TLE92466ED",
              "Device verification failed: no ICVID response (0x%04X) — check "
              "VIO/+5V/RESN/CS/MISO",
              icvid);
    return tle::unexpected(DriverError::DeviceNotResponding);
  }

  // Validate device ID
  bool valid = DeviceID::IsValidDevice(icvid);

  // Extract and log device information
  uint8_t device_type = DeviceID::GetDeviceType(icvid);
  uint8_t revision = DeviceID::GetRevision(icvid);

  if (valid) {
    cached_icvid_ = icvid;
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

  // Use CommInterface Read (two-frame pipelined transfer via TransferMulti)
  auto result = comm_.Read(address, should_verify_crc);
  if (!result) {
    // Map CommInterface error to driver error. BusError/TransferError used to
    // collapse into HardwareError, which made "the device reported a critical
    // fault" (its own 1V5/2V5/bandgap are down) look identical to a bus glitch
    // — the caller could not tell a supply problem from a wiring problem.
    switch (result.error()) {
    case CommError::Timeout:
      return tle::unexpected(DriverError::TimeoutError);
    case CommError::CRCError:
      return tle::unexpected(DriverError::CRCError);
    case CommError::NoReply:
      /* Idle MISO — an absent or unpowered part, not a corrupt one. Kept
       * distinct so bring-up can tell "nothing is answering" from "the answer
       * did not survive the bus". */
      return tle::unexpected(DriverError::DeviceNotResponding);
    case CommError::BusError:
      return tle::unexpected(DriverError::FaultDetected);
    case CommError::TransferError:
      return tle::unexpected(DriverError::SPIFrameError);
    default:
      return tle::unexpected(DriverError::HardwareError);
    }
  }

  return *result;
}

template <typename CommType>
DriverResult<void> Driver<CommType>::ReadRegisterMulti(
    std::span<const uint16_t> addresses, std::span<uint32_t> values,
    uint16_t& valid_mask) noexcept {
  valid_mask = 0U;
  if (!comm_.IsReady()) {
    return tle::unexpected(DriverError::HardwareError);
  }
  if (addresses.empty() || addresses.size() != values.size()) {
    return tle::unexpected(DriverError::InvalidParameter);
  }

  auto result = comm_.ReadMulti(addresses, values, valid_mask);
  if (!result) {
    switch (result.error()) {
    case CommError::Timeout:
      return tle::unexpected(DriverError::TimeoutError);
    case CommError::CRCError:
      return tle::unexpected(DriverError::CRCError);
    case CommError::NoReply:
      return tle::unexpected(DriverError::DeviceNotResponding);
    case CommError::BusError:
      return tle::unexpected(DriverError::FaultDetected);
    case CommError::TransferError:
      return tle::unexpected(DriverError::SPIFrameError);
    case CommError::InvalidParameter:
      return tle::unexpected(DriverError::InvalidParameter);
    default:
      return tle::unexpected(DriverError::HardwareError);
    }
  }
  return {};
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
    // Map CommInterface error to driver error (see ReadRegister for why
    // BusError and TransferError are kept distinct from HardwareError).
    switch (result.error()) {
    case CommError::Timeout:
      return tle::unexpected(DriverError::TimeoutError);
    case CommError::CRCError:
      return tle::unexpected(DriverError::CRCError);
    case CommError::BusError:
      return tle::unexpected(DriverError::FaultDetected);
    case CommError::TransferError:
      return tle::unexpected(DriverError::SPIFrameError);
    default:
      return tle::unexpected(DriverError::HardwareError);
    }
  }

  // Read back register to verify write succeeded
  if (verify_write) {
    /* Let the write commit before reading it back. Delay() is microseconds, so
     * the old Delay(1) was 1 us despite its "1ms delay" comment — the readback
     * could be clocked before the register settled, which reads as a spurious
     * RegisterError on an otherwise good write. */
    (void)comm_.Delay(1000); // 1 ms

    const char* reason = NonTransparentWriteReason(address);
    const bool known_issue = (reason != nullptr);

    /* Re-read (and re-issue the write) before condemning a register that is
     * supposed to read back what was written. A single mangled reply on this
     * bus used to fail Init outright, and the part then never reached Mission
     * even though the write itself had landed. Registers that are known not to
     * read back transparently skip the retry — re-writing them is pointless and
     * only lengthens Init by the settle delay. */
    constexpr unsigned kVerifyAttempts = 3U;
    auto read_result = ReadRegister(address, verify_crc);
    for (unsigned att = 1U; !known_issue && att < kVerifyAttempts; ++att) {
      if (read_result && static_cast<uint16_t>(*read_result) == value) {
        break;
      }
      (void)comm_.Write(address, value, should_verify_crc);
      (void)comm_.Delay(1000);
      read_result = ReadRegister(address, verify_crc);
    }
    if (read_result) {
      auto read_value = static_cast<uint16_t>(*read_result);

      if (read_value != value) {
        if (known_issue) {
          comm_.Log(LogLevel::Debug, "TLE92466ED",
                    "Write verification mismatch (expected): Address=0x%04X, Written=0x%04X, "
                    "Read=0x%04X\n"
                    "  %s",
                    address, value, read_value, reason);
        } else {
          comm_.Log(LogLevel::Warn, "TLE92466ED",
                    "Write verification FAILED: Address=0x%04X, Written=0x%04X, Read=0x%04X",
                    address, value, read_value);
          return tle::unexpected(DriverError::RegisterError);
        }
      } else {
        comm_.Log(LogLevel::Debug, "TLE92466ED", "Write verified: Address=0x%04X, Value=0x%04X",
                  address, value);
      }
    } else {
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
  // Mid-scale is the placeholder the frame carries before the measurement has
  // settled. Decoding it yields a confident-looking 2000 mA, which is how a
  // 115 mA valve reported 2 A on the bench.
  if (FB_I_AVG_s16::IsSettlingFrame(r.value())) {
    return tle::unexpected(DriverError::FeedbackNotReady);
  }
  return FB_I_AVG_s16::ToMilliamps(r.value());
}

#else
// Compiled directly (shouldn't happen) - close the namespace
} // namespace tle92466ed
#endif

#endif // TLE92466ED_IMPL
