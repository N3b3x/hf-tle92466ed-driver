/**
 * @file tle92466ed_registers.hpp
 * @brief Register definitions and bit field mappings for TLE92466ED
 * @copyright Copyright (c) 2024-2025 HardFOC. All rights reserved.
 */
#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace tle92466ed {

//==============================================================================
// REGISTER ADDRESSES - CENTRAL/GLOBAL REGISTERS
//==============================================================================

/**
 * @brief Central and global register addresses
 *
 * @details
 * These registers control global device configuration, status, and diagnostics.
 * All addresses are 10-bit (0x0000 - 0x03FF).
 */
namespace CentralReg {
constexpr uint16_t CH_CTRL = 0x0000;       ///< Channel Control Register
constexpr uint16_t GLOBAL_CONFIG = 0x0002; ///< Global Configuration Register
constexpr uint16_t GLOBAL_DIAG0 = 0x0003;  ///< Global Diagnosis Register 0
constexpr uint16_t GLOBAL_DIAG1 = 0x0004;  ///< Global Diagnosis Register 1
constexpr uint16_t GLOBAL_DIAG2 = 0x0005;  ///< Global Diagnosis Register 2
constexpr uint16_t VBAT_TH = 0x0006;       ///< VBAT Threshold Register
constexpr uint16_t FB_FRZ = 0x0007;        ///< Feedback Freeze Register
constexpr uint16_t FB_UPD = 0x0008;        ///< Feedback Update Register
constexpr uint16_t WD_RELOAD = 0x0009;     ///< SPI Watchdog Reload Register

// Channel Group Diagnosis (per datasheet §5.3.2.12 / §5.3.2.13)
// There are ONLY 3 groups. Each group covers a pair of channels via
// an internal y=0/y=1 field offset:
//   CHGR0 covers CH0 (y=0) + CH1 (y=1)
//   CHGR1 covers CH2 (y=0) + CH3 (y=1)
//   CHGR2 covers CH4 (y=0) + CH5 (y=1)
// Use DIAG_ERR_CHGR::AddressForChannel(ch) / DIAG_WARN_CHGR::AddressForChannel(ch).
// Addresses 0x000D..0x000F and 0x0013..0x0015 are NOT DIAG registers.
constexpr uint16_t DIAG_ERR_CHGR0 = 0x000A; ///< Diag Error CHGR0 (CH0+CH1)
constexpr uint16_t DIAG_ERR_CHGR1 = 0x000B; ///< Diag Error CHGR1 (CH2+CH3)
constexpr uint16_t DIAG_ERR_CHGR2 = 0x000C; ///< Diag Error CHGR2 (CH4+CH5)

constexpr uint16_t DIAG_WARN_CHGR0 = 0x0010; ///< Diag Warn CHGR0 (CH0+CH1)
constexpr uint16_t DIAG_WARN_CHGR1 = 0x0011; ///< Diag Warn CHGR1 (CH2+CH3)
constexpr uint16_t DIAG_WARN_CHGR2 = 0x0012; ///< Diag Warn CHGR2 (CH4+CH5)

constexpr uint16_t FAULT_MASK0 = 0x0016; ///< Fault Mask Register 0
constexpr uint16_t FAULT_MASK1 = 0x0017; ///< Fault Mask Register 1
constexpr uint16_t FAULT_MASK2 = 0x0018; ///< Fault Mask Register 2
constexpr uint16_t CLK_DIV = 0x0019;     ///< Clock Control Register
constexpr uint16_t SFF_BIST = 0x003F;    ///< BIST Register

// Feedback/Status Registers
constexpr uint16_t ICVID = 0x0200;       ///< IC Version and ID
constexpr uint16_t PIN_STAT = 0x0201;    ///< Pin Status Register
constexpr uint16_t FB_STAT = 0x0202;     ///< Feedback Status Register
constexpr uint16_t FB_VOLTAGE1 = 0x0203; ///< Feedback Voltage Register 1
constexpr uint16_t FB_VOLTAGE2 = 0x0204; ///< Feedback Voltage Register 2
constexpr uint16_t CHIPID0 = 0x0205;     ///< Unique Chip ID Register 0
constexpr uint16_t CHIPID1 = 0x0206;     ///< Unique Chip ID Register 1
constexpr uint16_t CHIPID2 = 0x0207;     ///< Unique Chip ID Register 2
} // namespace CentralReg

//==============================================================================
// DEVICE IDENTIFICATION CONSTANTS
//==============================================================================

/**
 * @brief Device identification and version information
 *
 * @details
 * The ICVID register contains device type and silicon revision information.
 * Format: [15:8] = Device Type, [7:0] = Silicon Revision
 *
 * @note Exact values should be verified against specific datasheet revision.
 *       The TLE92466ED family may have multiple device variants.
 */
namespace DeviceID {
constexpr uint16_t DEVICE_TYPE_MASK = 0xFF00; ///< Device type mask [15:8]
constexpr uint16_t REVISION_MASK = 0x00FF;    ///< Silicon revision mask [7:0]

// Expected manufacturer/device-type code for TLE92466ED (upper byte of ICVID).
// Per datasheet §5.3.2.19 p.75 the high byte is fixed at 0xC1 (Infineon
// manufacturer ID for this family); the low byte is the silicon revision
// and may vary by part lot. VerifyDevice() accepts any 0xC1?? value.
constexpr uint8_t EXPECTED_TYPE_92466ED = 0xC1; ///< Expected manufacturer/type byte

// Minimum supported silicon revision
constexpr uint8_t MIN_REVISION = 0x00; ///< Minimum silicon revision

/**
 * @brief Check if ICVID value is valid for TLE92466ED.
 * @param icvid Value read from ICVID register.
 * @return true if upper byte matches 0xC1 (any revision).
 */
[[nodiscard]] constexpr bool IsValidDevice(uint16_t icvid) noexcept {
  // Reject obviously broken reads (all-zero / all-one).
  if (icvid == 0x0000 || icvid == 0xFFFF) return false;
  const uint8_t device_type = static_cast<uint8_t>((icvid >> 8) & 0xFF);
  return device_type == EXPECTED_TYPE_92466ED;
}

/**
 * @brief Extract device type from ICVID
 */
[[nodiscard]] constexpr uint8_t GetDeviceType(uint16_t icvid) noexcept {
  return (icvid >> 8) & 0xFF;
}

/**
 * @brief Extract silicon revision from ICVID
 */
[[nodiscard]] constexpr uint8_t GetRevision(uint16_t icvid) noexcept {
  return icvid & 0xFF;
}
} // namespace DeviceID

//==============================================================================
// CHANNEL REGISTER OFFSETS
//==============================================================================

/**
 * @brief Per-channel register base addresses
 *
 * @details
 * Each channel has its own set of registers at a specific base address.
 * Per datasheet Table 25 (Register Address Space - Channel):
 * - CH0: 0x0040
 * - CH1: 0x0050
 * - CH2: 0x0060
 * - CH3: 0x0070
 * - CH4: 0x0020
 * - CH5: 0x0030
 *
 * Note: Channels are NOT in sequential order. CH4 and CH5 have lower addresses.
 */
namespace ChannelBase {
constexpr uint16_t CH0 = 0x0040; ///< Channel 0 base address
constexpr uint16_t CH1 = 0x0050; ///< Channel 1 base address
constexpr uint16_t CH2 = 0x0060; ///< Channel 2 base address
constexpr uint16_t CH3 = 0x0070; ///< Channel 3 base address
constexpr uint16_t CH4 = 0x0020; ///< Channel 4 base address
constexpr uint16_t CH5 = 0x0030; ///< Channel 5 base address
} // namespace ChannelBase

/**
 * @brief Per-channel register offsets (add to channel base address)
 */
namespace ChannelReg {
constexpr uint16_t SETPOINT = 0x0000;          ///< Current Setpoint Register
constexpr uint16_t CTRL = 0x0001;              ///< Control Register
constexpr uint16_t PERIOD = 0x0002;            ///< ICC PWM Frequency Controller
constexpr uint16_t INTEGRATOR_LIMIT = 0x0003;  ///< ICC Integrator Limitation
constexpr uint16_t DITHER_CLK_DIV = 0x0004;    ///< Dither Clock Register
constexpr uint16_t DITHER_STEP = 0x0005;       ///< Dither Step Register
constexpr uint16_t DITHER_CTRL = 0x0006;       ///< Dither Control Register
constexpr uint16_t CH_CONFIG = 0x0007;         ///< Channel Configuration
constexpr uint16_t MODE = 0x000C;              ///< Channel Mode Register
constexpr uint16_t TON = 0x000D;               ///< On-Time Register
constexpr uint16_t CTRL_INT_THRESH = 0x000E;   ///< ICC Integrator Threshold
constexpr uint16_t FB_DC = 0x0200;             ///< Feedback Duty Cycle
constexpr uint16_t FB_VBAT = 0x0201;           ///< Feedback Average VBAT
constexpr uint16_t FB_I_AVG = 0x0202;          ///< Feedback Average Current
constexpr uint16_t FB_IMIN_IMAX = 0x0203;      ///< Feedback Min/Max Current
constexpr uint16_t FB_I_AVG_s16 = 0x0204;      ///< Feedback signed Current (16-bit)
constexpr uint16_t FB_INT_THRESH = 0x0205;     ///< Feedback ICC Integrator Threshold
constexpr uint16_t FB_PERIOD_MIN_MAX = 0x0206; ///< Feedback Min/Max PWM Period
} // namespace ChannelReg

//==============================================================================
// CH_CTRL REGISTER (0x0000) - Channel Control
//==============================================================================

/**
 * @brief CH_CTRL register bit definitions
 *
 * @details
 * Main channel enable/disable and parallel operation configuration.
 * Channel enable bits can only be set in Mission Mode.
 * Parallel mode bits can only be set in Config Mode.
 *
 * @par Bit Map:
 * @verbatim
 * Bit 15  : OP_MODE     - Operation mode (0=Config, 1=Mission)
 * Bit 14  : CH_PAR_1_2  - Parallel operation CH1/CH2
 * Bit 13  : CH_PAR_0_3  - Parallel operation CH0/CH3
 * Bit 12  : CH_PAR_4_5  - Parallel operation CH4/CH5
 * Bits 11-6: Reserved
 * Bit 5   : EN_CH5      - Enable Channel 5
 * Bit 4   : EN_CH4      - Enable Channel 4
 * Bit 3   : EN_CH3      - Enable Channel 3
 * Bit 2   : EN_CH2      - Enable Channel 2
 * Bit 1   : EN_CH1      - Enable Channel 1
 * Bit 0   : EN_CH0      - Enable Channel 0
 * @endverbatim
 *
 * Default: 0x0000
 */
namespace CH_CTRL {
constexpr uint16_t EN_CH0 = (1 << 0);      ///< Enable Channel 0
constexpr uint16_t EN_CH1 = (1 << 1);      ///< Enable Channel 1
constexpr uint16_t EN_CH2 = (1 << 2);      ///< Enable Channel 2
constexpr uint16_t EN_CH3 = (1 << 3);      ///< Enable Channel 3
constexpr uint16_t EN_CH4 = (1 << 4);      ///< Enable Channel 4
constexpr uint16_t EN_CH5 = (1 << 5);      ///< Enable Channel 5
constexpr uint16_t CH_PAR_4_5 = (1 << 12); ///< Parallel CH4/CH5
constexpr uint16_t CH_PAR_0_3 = (1 << 13); ///< Parallel CH0/CH3
constexpr uint16_t CH_PAR_1_2 = (1 << 14); ///< Parallel CH1/CH2
constexpr uint16_t OP_MODE = (1 << 15);    ///< Operation Mode

constexpr uint16_t ALL_CH_MASK = 0x003F;  ///< All channel bits
constexpr uint16_t ALL_PAR_MASK = 0x7000; ///< All parallel bits

constexpr uint16_t DEFAULT = 0x0000; ///< Default value

/// Config Mode (OP_MODE=0)
constexpr uint16_t CONFIG_MODE = 0x0000;
/// Mission Mode (OP_MODE=1)
constexpr uint16_t MISSION_MODE = OP_MODE;

/**
 * @brief Get channel enable bit mask
 */
[[nodiscard]] constexpr uint16_t ChannelMask(uint8_t channel) noexcept {
  return (channel < 6) ? static_cast<uint16_t>(1 << channel) : 0;
}
} // namespace CH_CTRL

//==============================================================================
// GLOBAL_CONFIG REGISTER (0x0002) - Global Configuration
//==============================================================================

/**
 * @brief GLOBAL_CONFIG register bit definitions
 *
 * @details
 * Global configuration register (write only in Config Mode).
 *
 * @par Bit Map:
 * @verbatim
 * Bit 14  : VIO_SEL        - VIO voltage selection (0=3.3V, 1=5.0V)
 * Bit 13  : UV_OV_SWAP     - UV/OV swap test
 * Bit 12  : OT_TEST        - Over-temperature test
 * Bit 5   : V1V5_OV_TEST   - 1.5V overvoltage test
 * Bit 4   : V1V5_UV_TEST   - 1.5V undervoltage test
 * Bit 2   : CRC_EN         - CRC check enable
 * Bit 1   : SPI_WD_EN      - SPI watchdog enable
 * Bit 0   : CLK_WD_EN      - Clock watchdog enable
 * @endverbatim
 *
 * Default: 0x4005
 */
namespace GLOBAL_CONFIG {
constexpr uint16_t CLK_WD_EN = (1 << 0);    ///< Clock watchdog enable
constexpr uint16_t SPI_WD_EN = (1 << 1);    ///< SPI watchdog enable
constexpr uint16_t CRC_EN = (1 << 2);       ///< CRC check enable
constexpr uint16_t V1V5_UV_TEST = (1 << 4); ///< 1.5V UV test
constexpr uint16_t V1V5_OV_TEST = (1 << 5); ///< 1.5V OV test
constexpr uint16_t OT_TEST = (1 << 12);     ///< Over-temp test
constexpr uint16_t UV_OV_SWAP = (1 << 13);  ///< UV/OV swap test
constexpr uint16_t VIO_SEL = (1 << 14);     ///< VIO select (0=3.3V, 1=5V)

constexpr uint16_t DEFAULT = 0x4005; ///< Default value
} // namespace GLOBAL_CONFIG

//==============================================================================
// GLOBAL_DIAG0 REGISTER (0x0003) - Global Diagnosis Register 0
//==============================================================================

/**
 * @brief GLOBAL_DIAG0 register bit definitions
 *
 * @details
 * Global diagnosis register for supply voltages, reset events, and faults.
 *
 * @par Bit Map:
 * @verbatim
 * Bit 14  : SPI_WD_ERR  - SPI watchdog error
 * Bit 10  : POR_EVENT   - Power-on reset event
 * Bit 9   : RES_EVENT   - Reset event (RESN pin)
 * Bit 8   : COTWARN     - Central over-temp warning
 * Bit 7   : COTERR      - Central over-temp error
 * Bit 6   : CLK_NOK     - Clock fault
 * Bit 5   : VDD_OV      - VDD overvoltage
 * Bit 4   : VDD_UV      - VDD undervoltage
 * Bit 3   : VIO_OV      - VIO overvoltage
 * Bit 2   : VIO_UV      - VIO undervoltage
 * Bit 1   : VBAT_OV     - VBAT overvoltage
 * Bit 0   : VBAT_UV     - VBAT undervoltage
 * @endverbatim
 *
 * Default: 0x0600
 */
namespace GLOBAL_DIAG0 {
constexpr uint16_t VBAT_UV = (1 << 0);     ///< VBAT undervoltage
constexpr uint16_t VBAT_OV = (1 << 1);     ///< VBAT overvoltage
constexpr uint16_t VIO_UV = (1 << 2);      ///< VIO undervoltage
constexpr uint16_t VIO_OV = (1 << 3);      ///< VIO overvoltage
constexpr uint16_t VDD_UV = (1 << 4);      ///< VDD undervoltage
constexpr uint16_t VDD_OV = (1 << 5);      ///< VDD overvoltage
constexpr uint16_t CLK_NOK = (1 << 6);     ///< Clock fault
constexpr uint16_t COTERR = (1 << 7);      ///< Central OT error
constexpr uint16_t COTWARN = (1 << 8);     ///< Central OT warning
constexpr uint16_t RES_EVENT = (1 << 9);   ///< Reset event
constexpr uint16_t POR_EVENT = (1 << 10);  ///< Power-on reset
constexpr uint16_t SPI_WD_ERR = (1 << 14); ///< SPI watchdog error

constexpr uint16_t DEFAULT = 0x0600;    ///< Default value
constexpr uint16_t FAULT_MASK = 0x47FF; ///< All fault bits
constexpr uint16_t CLEAR_ALL = 0xFFFF;  ///< Clear all bits (write-to-clear)
} // namespace GLOBAL_DIAG0

//==============================================================================
// WD_RELOAD REGISTER (0x0009) - SPI Watchdog Reload Register
//==============================================================================

/**
 * @brief WD_RELOAD register bit definitions
 *
 * @details
 * SPI Watchdog Counter Reload Register.
 * The watchdog counter is decremented with fSPI,WD. If it reaches 0, SPI_WD_ERR is set
 * and the device enters Config Mode. The register must be reloaded periodically.
 *
 * @par Bit Map:
 * @verbatim
 * Bits 15-11: Reserved
 * Bits 10-0 : WD_TIME - Reload value (11-bit)
 * @endverbatim
 *
 * @par Formula:
 * <WD_TIME> = rounddown( t_SPI_WD * f_SYS / 2^14 )
 *
 * @par Timeout:
 * t_SPI_WD = <WD_TIME> / f_SPI_WD
 *
 * Default: 0x0001
 */
namespace WD_RELOAD {
constexpr uint16_t WD_TIME_MASK = 0x07FF; ///< 11-bit mask for WD_TIME field (bits 10:0)
constexpr uint16_t WD_TIME_MAX = 0x07FF;  ///< Maximum WD_TIME value (2047)
constexpr uint16_t DEFAULT = 0x0001;      ///< Default value

/**
 * @brief Mask WD_TIME value to valid 11-bit range
 * @param value Raw value to mask
 * @return Masked value (bits 10:0)
 */
[[nodiscard]] constexpr uint16_t MaskValue(uint16_t value) noexcept {
  return value & WD_TIME_MASK;
}
} // namespace WD_RELOAD

//==============================================================================
// GLOBAL_DIAG1 REGISTER (0x0004) - Global Diagnosis Register 1
//==============================================================================

/**
 * @brief GLOBAL_DIAG1 register bit definitions
 *
 * @details
 * Internal voltage and reference diagnostics.
 *
 * @par Bit Map:
 * @verbatim
 * Bit 15  : HVADC_ERR   - HV ADC error
 * Bit 6   : VPRE_OV     - Pre-regulator overvoltage
 * Bit 5   : REF_OV      - Reference overvoltage
 * Bit 4   : REF_UV      - Reference undervoltage
 * Bit 3   : VDD2V5_OV   - 2.5V supply overvoltage
 * Bit 2   : VDD2V5_UV   - 2.5V supply undervoltage
 * Bit 1   : VR_IREF_OV  - Bias current overvoltage
 * Bit 0   : VR_IREF_UV  - Bias current undervoltage
 * @endverbatim
 *
 * Default: 0x0000
 */
namespace GLOBAL_DIAG1 {
constexpr uint16_t VR_IREF_UV = (1 << 0); ///< Bias current UV
constexpr uint16_t VR_IREF_OV = (1 << 1); ///< Bias current OV
constexpr uint16_t VDD2V5_UV = (1 << 2);  ///< 2.5V supply UV
constexpr uint16_t VDD2V5_OV = (1 << 3);  ///< 2.5V supply OV
constexpr uint16_t REF_UV = (1 << 4);     ///< Reference UV
constexpr uint16_t REF_OV = (1 << 5);     ///< Reference OV
constexpr uint16_t VPRE_OV = (1 << 6);    ///< Pre-reg OV
constexpr uint16_t HVADC_ERR = (1 << 15); ///< HV ADC error

constexpr uint16_t DEFAULT = 0x0000;   ///< Default value
constexpr uint16_t CLEAR_ALL = 0xFFFF; ///< Clear all bits (write-to-clear)
} // namespace GLOBAL_DIAG1

//==============================================================================
// DIAG_ERR_CHGRx REGISTERS (0x000A..0x000C) - Per-Channel-Pair Error Diagnostics
//==============================================================================

/**
 * @brief DIAG_ERR_CHGR bit layout and address helpers.
 *
 * @details
 * Per datasheet \u00a75.3.2.12 p.67. There are only THREE groups, each
 * covering a channel pair (low channel = y=0, high channel = y=1).
 * The per-channel bit offset is `8*y`:
 *   bit 8*y+0  OLSG  Open-Load or Short-to-Ground (latched pre-check)
 *   bit 8*y+1  OL    Open-Load
 *   bit 8*y+2  OC    Overcurrent  (requires TWO consecutive clear writes)
 *   bit 8*y+3  SG    Short-to-Ground
 *   bit 8*y+4  OTE   Channel overtemperature error
 *   bits 8*y+5..7 reserved
 *
 * Example: DIAG_ERR_CHGR0 holds CH0 error flags in bits [4:0] and
 *          CH1 error flags in bits [12:8].
 */
namespace DIAG_ERR_CHGR {
constexpr uint8_t BIT_OLSG = 0; ///< Open-Load / Short-to-Ground
constexpr uint8_t BIT_OL   = 1; ///< Open-Load
constexpr uint8_t BIT_OC   = 2; ///< Overcurrent (double-clear)
constexpr uint8_t BIT_SG   = 3; ///< Short-to-Ground
constexpr uint8_t BIT_OTE  = 4; ///< Channel overtemperature error

/// 5-bit mask of all error bits for one channel (before 8*y shift).
constexpr uint16_t PER_CHANNEL_MASK = 0x001Fu;

/// 16-bit mask of both-channel OC bits in a group (bits 2 and 10).
constexpr uint16_t OC_BOTH_MASK = (1u << BIT_OC) | (1u << (BIT_OC + 8));

/// Register address for the group covering a channel.
[[nodiscard]] constexpr uint16_t AddressForChannel(uint8_t channel) noexcept {
  return static_cast<uint16_t>(CentralReg::DIAG_ERR_CHGR0 + (channel / 2));
}

/// Bit shift (0 or 8) selecting the channel within its group.
[[nodiscard]] constexpr uint8_t YShiftForChannel(uint8_t channel) noexcept {
  return static_cast<uint8_t>((channel & 1u) * 8u);
}

/// Build a single-bit test mask for a given channel and bit index.
[[nodiscard]] constexpr uint16_t ChannelBitMask(uint8_t channel, uint8_t bit_index) noexcept {
  return static_cast<uint16_t>(1u << (bit_index + YShiftForChannel(channel)));
}

/// Clear mask writing 1s only to the OC bit(s) of the affected channel.
[[nodiscard]] constexpr uint16_t OcClearMaskForChannel(uint8_t channel) noexcept {
  return ChannelBitMask(channel, BIT_OC);
}
} // namespace DIAG_ERR_CHGR

//==============================================================================
// DIAG_WARN_CHGRx REGISTERS (0x0010..0x0012) - Per-Channel-Pair Warnings
//==============================================================================

/**
 * @brief DIAG_WARN_CHGR bit layout and address helpers.
 *
 * @details
 * Per datasheet \u00a75.3.2.13 p.68. Three groups only; bit offset is `8*y`:
 *   bit 8*y+0  PWM_REG_WARN        ICC PWM regulation warning
 *   bit 8*y+1  I_REG_WARN          ICC current regulation warning
 *   bit 8*y+2  OTW                 Channel overtemperature warning
 *   bit 8*y+3  OLSG_WARN           Open-Load/Short-to-Ground warning
 *   bit 8*y+4  OLSG_WARN_CHK_NOK   OLSG check not yet performed (POR=1)
 *
 * POR default is 0x1010 because the two CHK_NOK bits come up set.
 */
namespace DIAG_WARN_CHGR {
constexpr uint8_t BIT_PWM_REG      = 0;
constexpr uint8_t BIT_I_REG        = 1;
constexpr uint8_t BIT_OTW          = 2;
constexpr uint8_t BIT_OLSG_WARN    = 3;
constexpr uint8_t BIT_OLSG_CHK_NOK = 4;

constexpr uint16_t PER_CHANNEL_MASK = 0x001Fu;

/// Register address for the group covering a channel.
[[nodiscard]] constexpr uint16_t AddressForChannel(uint8_t channel) noexcept {
  return static_cast<uint16_t>(CentralReg::DIAG_WARN_CHGR0 + (channel / 2));
}

/// Bit shift (0 or 8) selecting the channel within its group.
[[nodiscard]] constexpr uint8_t YShiftForChannel(uint8_t channel) noexcept {
  return static_cast<uint8_t>((channel & 1u) * 8u);
}

/// Build a single-bit test mask for a given channel and bit index.
[[nodiscard]] constexpr uint16_t ChannelBitMask(uint8_t channel, uint8_t bit_index) noexcept {
  return static_cast<uint16_t>(1u << (bit_index + YShiftForChannel(channel)));
}
} // namespace DIAG_WARN_CHGR

//==============================================================================
// GLOBAL_DIAG2 REGISTER (0x0005) - Global Diagnosis Register 2
//==============================================================================

/**
 * @brief GLOBAL_DIAG2 register bit definitions
 */
namespace GLOBAL_DIAG2 {
constexpr uint16_t REG_ECC_ERR = (1 << 1); ///< Register ECC error
constexpr uint16_t OTP_ECC_ERR = (1 << 3); ///< OTP ECC error
constexpr uint16_t OTP_VIRGIN = (1 << 4);  ///< OTP virgin/unconfigured

constexpr uint16_t DEFAULT = 0x0000;   ///< Default value
constexpr uint16_t CLEAR_ALL = 0xFFFF; ///< Clear all bits (write-to-clear)
} // namespace GLOBAL_DIAG2

//==============================================================================
// FB_STAT REGISTER (0x0202) - Feedback Status
//==============================================================================

/**
 * @brief FB_STAT register bit definitions
 *
 * @details
 * General feedback and status information.
 */
/**
 * @warning FB_STAT is a **22-bit** reply register. @c INIT_DONE (21) and
 *          @c SPI_WD_ERR (20) sit above bit 15, so a caller that narrows the
 *          @c ReadRegister result to @c uint16_t silently loses them and reads
 *          @c INIT_DONE as 0 forever. Keep these masks 32-bit.
 *
 * Bit positions per datasheet Rev. 1.2 (2022-02-01) §5.3.2.21, Table 23.
 * Reset value is 0x200638 — note @c INIT_DONE is set out of reset.
 */
namespace FB_STAT {
constexpr uint32_t DIAG_WARN_CHGR0 = (1u << 0);  ///< DIAG_WARN_CHGR0 status
constexpr uint32_t DIAG_WARN_CHGR1 = (1u << 1);  ///< DIAG_WARN_CHGR1 status
constexpr uint32_t DIAG_WARN_CHGR2 = (1u << 2);  ///< DIAG_WARN_CHGR2 status
constexpr uint32_t OLSG_WARN_CHK_NOK_CHGR0 = (1u << 3);  ///< OL/SG check group 0
constexpr uint32_t OLSG_WARN_CHK_NOK_CHGR1 = (1u << 4);  ///< OL/SG check group 1
constexpr uint32_t OLSG_WARN_CHK_NOK_CHGR2 = (1u << 5);  ///< OL/SG check group 2
constexpr uint32_t CLK_NOK_STAT = (1u << 6);  ///< Clock fault status
constexpr uint32_t COTERR = (1u << 7);        ///< Central overtemperature error
constexpr uint32_t COTWARN = (1u << 8);       ///< Central overtemperature warning
constexpr uint32_t RES_EVENT = (1u << 9);     ///< Reset event (RESN pin)
constexpr uint32_t POR_EVENT = (1u << 10);    ///< Power-on reset event
constexpr uint32_t DATA_ERR = (1u << 11);     ///< OTP_ECC_ERR / OTP_VIRGIN / HV_ADC_ERR
constexpr uint32_t SUP_NOK_EXT = (1u << 12);  ///< VIO/VDD/VBAT UV or OV
constexpr uint32_t SUP_NOK_INT = (1u << 13);  ///< Internal rail UV or OV
constexpr uint32_t ERR_CHGR0 = (1u << 14);    ///< DIAG_ERR_CHGR1 status
constexpr uint32_t ERR_CHGR1 = (1u << 15);    ///< DIAG_ERR_CHGR2 status
constexpr uint32_t ERR_CHGR2 = (1u << 16);    ///< DIAG_ERR_CHGR3 status
constexpr uint32_t SPI_WD_ERR = (1u << 20);   ///< SPI watchdog error status
constexpr uint32_t INIT_DONE = (1u << 21);    ///< Chip initialization done
} // namespace FB_STAT

//==============================================================================
// CHANNEL SETPOINT REGISTER - Per Channel
//==============================================================================

/**
 * @brief SETPOINT register bit definitions (per channel)
 *
 * @details
 * 15-bit current setpoint with auto-limit disable.
 *
 * **Current Calculation:**
 * - Single mode: I_set = 2A * TARGET / 32767
 * - Parallel mode: I_set = 4A * TARGET / 32767
 *
 * **Current Capability (per datasheet):**
 * - Single channel:
 *   - Typical continuous: ~1.5 A
 *   - Absolute maximum (register scale): 2.0 A
 * - Parallel channels (paired):
 *   - Typical continuous: ~2.7 A
 *   - Absolute maximum (register scale): 4.0 A
 *
 * @warning The driver uses the full register scale (2A/4A) as maximum setpoint
 *          values, but the device will naturally limit to its thermal/electrical
 *          capacity (~1.5A single, ~2.7A parallel). Setting higher values may
 *          result in current regulation at the device limit rather than setpoint.
 *
 * @par Bit Map:
 * @verbatim
 * Bit 15    : AUTO_LIMIT_DIS - Auto-limit disable
 * Bits 14-0 : TARGET - Current setpoint value
 * @endverbatim
 */
namespace SETPOINT {
constexpr uint16_t TARGET_MASK = 0x7FFF;       ///< Target current mask
constexpr uint16_t AUTO_LIMIT_DIS = (1 << 15); ///< Disable auto-limit

constexpr uint16_t DEFAULT = 0x0000; ///< Default (0A)

/// Maximum safe target value (datasheet saturates above 0x6000)
constexpr uint16_t MAX_TARGET = 0x6000;

/**
 * @brief Calculate setpoint value for desired current
 * @param current_ma Desired current in milliamperes (0-2000 single, 0-4000 parallel)
 * @param parallel_mode true if channel is in parallel mode
 * @return Setpoint register value
 *
 * @note Uses rounding to minimize quantization error (adds half of max_current before division)
 */
[[nodiscard]] constexpr uint16_t CalculateTarget(uint16_t current_ma,
                                                 bool parallel_mode = false) noexcept {
  uint32_t max_current = parallel_mode ? 4000 : 2000;
  // Integer math: adding (max_current/2) before division is equivalent to (val / max_current + 0.5)
  // for rounding up (uint32_t(current_ma) * 32767UL + max_current / 2) / max_current  <=>
  // (current_ma * 32767 / max_current + 0.5)
  uint32_t target = (static_cast<uint32_t>(current_ma) * 32767UL + (max_current / 2)) / max_current;
  // Saturate at MAX_TARGET
  target = std::min(target, static_cast<uint32_t>(MAX_TARGET));
  return static_cast<uint16_t>(target);
}

/**
 * @brief Calculate current from setpoint value
 * @param target Setpoint register value
 * @param parallel_mode true if channel is in parallel mode
 * @return Current in milliamperes
 *
 * @note Uses rounding to minimize quantization error (adds half of 32767 before division)
 */
[[nodiscard]] constexpr uint16_t CalculateCurrent(uint16_t target,
                                                  bool parallel_mode = false) noexcept {
  uint32_t max_current = parallel_mode ? 4000 : 2000;
  // Use rounding: add half of 32767 before division to minimize quantization error
  uint32_t current = (static_cast<uint32_t>(target & TARGET_MASK) * max_current + 16383) / 32767UL;
  return static_cast<uint16_t>(current);
}

// Datasheet §5.3.3.3: I_set = 2 A * TARGET / (2^15-1); 0x6000 saturates at 1.5 A.
// A 9-bit TARGET (0x1FF) decodes as 31 mA — the HIL "clamped to 31 mA" signature
// when SETPOINT readback is actually FB_IMIN (10-bit signed, max +511) or a
// truncated field rather than the 15-bit TARGET. CH_CTRL 0x8020 (Mission+CH5)
// decodes as 2 mA — the leftover from a MakeRead(0) dummy.
static_assert(CalculateTarget(115) == 1884, "115 mA → TARGET 0x075C");
static_assert(CalculateCurrent(1884) == 115, "TARGET 0x075C → 115 mA");
static_assert(CalculateCurrent(0x01FF) == 31, "9-bit max TARGET decodes as 31 mA");
static_assert(CalculateCurrent(0x0020) == 2, "CH_CTRL EN_CH5 leftover decodes as 2 mA");
} // namespace SETPOINT

//==============================================================================
// PERIOD REGISTER - PWM Period Configuration
//==============================================================================

/**
 * @brief PERIOD register bit definitions and helper functions
 *
 * @details
 * PWM period configuration for ICC mode.
 *
 * @par Bit Map:
 * @verbatim
 * Bit 11   : LOW_FREQ - Low frequency range (8x multiplier)
 * Bits 10-8: PERIOD_EXP - Period exponent (0-7)
 * Bits 7-0 : PERIOD_MANT - Period mantissa (0-255)
 * @endverbatim
 *
 * **Formulas**:
 * - Standard: T_pwm = PERIOD_MANT × 2^PERIOD_EXP × (1/f_sys)
 * - Low Freq: T_pwm = PERIOD_MANT × 8 × 2^PERIOD_EXP × (1/f_sys)
 * - Where f_sys = 28 MHz (PLL-stabilized, see datasheet §4.2.1 Table 10 p.16),
 *   so 1/f_sys ≈ 35.714 ns (≈ 1/28 µs).
 */
namespace PERIOD {
constexpr uint16_t MANT_MASK = 0x00FF;       ///< Mantissa mask (bits 7:0)
constexpr uint16_t EXP_MASK = 0x0700;        ///< Exponent mask (bits 10:8)
constexpr uint16_t EXP_SHIFT = 8;            ///< Exponent shift
constexpr uint8_t EXP_VALUE_MASK = 0x07;     ///< Exponent value mask (3 bits: 0-7)
constexpr uint16_t LOW_FREQ_BIT = (1 << 11); ///< Low frequency range bit

/// PWM controller proportional gain lives in bits [15:12] of PERIOD per
/// datasheet \u00a75.3.3.3. 4-bit KI value.
constexpr uint16_t PWM_CTRL_PARAM_SHIFT = 12;
constexpr uint16_t PWM_CTRL_PARAM_MASK  = 0xF000;

constexpr uint32_t F_SYS_HZ = 28'000'000UL;           ///< System clock frequency (28 MHz)
constexpr float F_SYS_PERIOD_US = 1.0F / 28.0F;       ///< System clock period (≈35.71 ns)

//
// Datasheet-spec PWM frequency range
// ----------------------------------
// Per datasheet (Electrical Characteristics, parameter "Target PWM frequency
// fPWM"): the PWM frequency controller is specified to operate between
// 110 Hz and 4000 Hz. Setting a value outside this range will not be
// rejected by the chip — it simply isn't guaranteed to work.
//
// With the LOW_FREQ_RANGE_EN bit set, the period is multiplied by 8, so
// the supported frequency range is 8× lower (≈13.75 Hz – 500 Hz).
//
// Periods are derived from the formula  T_pwm = 1 / f_pwm.
//
// Note: the chip family cannot drive PWM above 4 kHz, so reaching the
// ultrasonic range (>20 kHz) is not possible with the TLE92466ED.
//
constexpr float kSpecMinFrequency_Hz   = 110.0F;     ///< Datasheet minimum f_PWM
constexpr float kSpecMaxFrequency_Hz   = 4000.0F;    ///< Datasheet maximum f_PWM
constexpr float kSpecMinPeriod_us      = 1.0e6F / kSpecMaxFrequency_Hz;  ///< 250 µs
constexpr float kSpecMaxPeriod_us      = 1.0e6F / kSpecMinFrequency_Hz;  ///< 9090.9 µs

constexpr float kSpecLowRangeMinFreq_Hz = kSpecMinFrequency_Hz / 8.0F;   ///< 13.75 Hz
constexpr float kSpecLowRangeMaxFreq_Hz = kSpecMaxFrequency_Hz / 8.0F;   ///< 500 Hz
constexpr float kSpecLowRangeMinPeriod_us =
    1.0e6F / kSpecLowRangeMaxFreq_Hz;                                    ///< 2000 µs
constexpr float kSpecLowRangeMaxPeriod_us =
    1.0e6F / kSpecLowRangeMinFreq_Hz;                                    ///< 72727 µs

/// Combined min period across both ranges (250 µs).
constexpr float kSpecCombinedMinPeriod_us = kSpecMinPeriod_us;
/// Combined max period across both ranges (low-range 72.7 ms).
constexpr float kSpecCombinedMaxPeriod_us = kSpecLowRangeMaxPeriod_us;

/**
 * @brief Calculate PWM period register values from desired period in microseconds
 *
 * @param period_us Desired PWM period in microseconds
 * @return Structure containing mantissa, exponent, and low_freq_range
 *
 * @details
 * Automatically selects the best combination of mantissa, exponent, and low_freq_range
 * to achieve the desired period. Tries standard range first, then low frequency range
 * if needed for longer periods.
 *
 * Valid range: ~0.125 µs to ~32.64 ms
 */
struct PeriodConfig {
  uint8_t mantissa;    ///< Period mantissa (0-255)
  uint8_t exponent;    ///< Period exponent (0-7)
  bool low_freq_range; ///< Low frequency range enabled

  /**
   * @brief Calculate actual period from register values
   * @return Actual period in microseconds
   */
  [[nodiscard]] constexpr float CalculatePeriodUs() const noexcept {
    float base_period =
        static_cast<float>(mantissa) * static_cast<float>(1ULL << exponent) * F_SYS_PERIOD_US;
    return low_freq_range ? (base_period * 8.0F) : base_period;
  }
};

/**
 * @brief Calculate period configuration from desired period in microseconds
 * @param period_us Desired PWM period in microseconds
 * @return PeriodConfig structure, or invalid if period is out of range
 */
[[nodiscard]] inline PeriodConfig CalculateFromPeriodUs(float period_us) noexcept {
  PeriodConfig config{};

  // Try standard range first
  for (uint8_t exp = 0; exp <= 7; ++exp) {
    float divisor = static_cast<float>(1ULL << exp) * F_SYS_PERIOD_US;
    float mantissa_f = period_us / divisor;

    if (mantissa_f <= 255.0F && mantissa_f >= 1.0F) {
      config.mantissa = static_cast<uint8_t>(std::lround(mantissa_f)); // Round
      config.exponent = exp;
      config.low_freq_range = false;
      return config;
    }
  }

  // Try low frequency range (8x multiplier)
  for (uint8_t exp = 0; exp <= 7; ++exp) {
    float divisor = static_cast<float>(1ULL << exp) * F_SYS_PERIOD_US * 8.0F;
    float mantissa_f = period_us / divisor;

    if (mantissa_f <= 255.0F && mantissa_f >= 1.0F) {
      config.mantissa = static_cast<uint8_t>(std::lround(mantissa_f)); // Round
      config.exponent = exp;
      config.low_freq_range = true;
      return config;
    }
  }

  // Out of range - return invalid (mantissa = 0)
  config.mantissa = 0;
  config.exponent = 0;
  config.low_freq_range = false;
  return config;
}

/**
 * @brief Build PERIOD register value from configuration
 * @param config Period configuration
 * @return Register value
 */
[[nodiscard]] constexpr uint16_t BuildRegisterValue(const PeriodConfig& config) noexcept {
  return config.mantissa | ((config.exponent & 0x07) << EXP_SHIFT) |
         (config.low_freq_range ? LOW_FREQ_BIT : 0);
}
} // namespace PERIOD

//==============================================================================
// CHANNEL CTRL REGISTER - Per Channel
//==============================================================================

/**
 * @brief Channel CTRL register bit definitions
 *
 * @par Bit Map:
 * @verbatim
 * Bit 14    : OLSG_WARN_EN      - OLSG warning enable
 * Bits 13-9 : OLSG_WARN_WINDOW  - OLSG warning window
 * Bit 8     : PWM_PERIOD_CALC_MODE - PWM period calc mode
 * Bits 7-0  : MIN_INT_THRESH    - Minimum integrator threshold
 * @endverbatim
 */
namespace CH_CTRL_REG {
constexpr uint16_t MIN_INT_THRESH_MASK = 0x00FF;    ///< Min threshold mask
constexpr uint16_t PWM_PERIOD_CALC_MODE = (1 << 8); ///< PWM calc mode
constexpr uint16_t OLSG_WARN_WINDOW_MASK = 0x3E00;  ///< OLSG window mask
constexpr uint16_t OLSG_WARN_WINDOW_SHIFT = 9;      ///< OLSG window shift
constexpr uint16_t OLSG_WARN_EN = (1 << 14);        ///< OLSG warn enable

constexpr uint16_t DEFAULT = 0x4600; ///< Default value
} // namespace CH_CTRL_REG

//==============================================================================
// CH_CONFIG REGISTER - Per Channel Configuration
//==============================================================================

/**
 * @brief CH_CONFIG register bit definitions
 *
 * @details
 * Channel configuration for slew rate, diagnostics, and open load detection.
 *
 * @par Bit Map:
 * @verbatim
 * Bits 15-14: OFF_DIAG_CH  - OFF-state diagnostic control
 * Bit 13    : OC_DIAG_EN   - OC diagnosis in OFF-state
 * Bits 12-7 : OL_TH_FIXED  - Fixed open load threshold
 * Bits 6-4  : OL_TH        - Relative open load threshold
 * Bits 3-2  : I_DIAG       - OFF-state diagnostic current
 * Bits 1-0  : SLEWR        - Slew rate control
 * @endverbatim
 *
 * Default: 0x0003
 */
namespace CH_CONFIG {
// Slew rate control [1:0]
constexpr uint16_t SLEWR_1V0_US = 0b00;  ///< 1.0 V/µs
constexpr uint16_t SLEWR_2V5_US = 0b01;  ///< 2.5 V/µs
constexpr uint16_t SLEWR_5V0_US = 0b10;  ///< 5.0 V/µs
constexpr uint16_t SLEWR_10V0_US = 0b11; ///< 10.0 V/µs
constexpr uint16_t SLEWR_MASK = 0x0003;  ///< Slew rate mask

// OFF-state diagnostic current [3:2]
constexpr uint16_t I_DIAG_80UA = (0 << 2);   ///< 80 µA
constexpr uint16_t I_DIAG_190UA = (1 << 2);  ///< 190 µA
constexpr uint16_t I_DIAG_720UA = (2 << 2);  ///< 720 µA
constexpr uint16_t I_DIAG_1250UA = (3 << 2); ///< 1250 µA
constexpr uint16_t I_DIAG_MASK = 0x000C;     ///< I_DIAG mask

// Open load threshold relative to setpoint [6:4]
constexpr uint16_t OL_TH_DISABLED = (0 << 4); ///< OL detection disabled
constexpr uint16_t OL_TH_1_8 = (1 << 4);      ///< 1/8 of setpoint
constexpr uint16_t OL_TH_2_8 = (2 << 4);      ///< 2/8 of setpoint
constexpr uint16_t OL_TH_3_8 = (3 << 4);      ///< 3/8 of setpoint
constexpr uint16_t OL_TH_4_8 = (4 << 4);      ///< 4/8 of setpoint
constexpr uint16_t OL_TH_5_8 = (5 << 4);      ///< 5/8 of setpoint
constexpr uint16_t OL_TH_6_8 = (6 << 4);      ///< 6/8 of setpoint
constexpr uint16_t OL_TH_7_8 = (7 << 4);      ///< 7/8 of setpoint
constexpr uint16_t OL_TH_MASK = 0x0070;       ///< OL threshold mask
constexpr uint8_t OL_TH_VALUE_MASK = 0x07;    ///< OL threshold value mask (3 bits: 0-7)
constexpr uint8_t OL_TH_SHIFT = 4;            ///< OL threshold shift in register

// Fixed open load threshold [12:7]
constexpr uint16_t OL_TH_FIXED_SHIFT = 7;
constexpr uint16_t OL_TH_FIXED_MASK = 0x1F80;

constexpr uint16_t OC_DIAG_EN = (1 << 13); ///< OC diag in OFF state

// OFF-state diagnostic control [15:14]
constexpr uint16_t OFF_DIAG_ENABLED = (0 << 14); ///< OFF diag enabled
constexpr uint16_t OFF_DIAG_LS_ONLY = (1 << 14); ///< Low side current only
constexpr uint16_t OFF_DIAG_HS_ONLY = (2 << 14); ///< High side current only
constexpr uint16_t OFF_DIAG_MASK = 0xC000;       ///< OFF diag mask

constexpr uint16_t DEFAULT = 0x0003; ///< Default value
} // namespace CH_CONFIG

//==============================================================================
// MODE REGISTER - Channel Mode
//==============================================================================

/**
 * @brief Channel MODE register bit definitions
 *
 * @details
 * Channel operation mode selection (write only in Config Mode).
 *
 * @par Bit Map:
 * @verbatim
 * Bits 3-0: CH_MODE - Channel operation mode
 * @endverbatim
 */
namespace CH_MODE {
constexpr uint16_t OFF = 0x0000;               ///< Channel off
constexpr uint16_t ICC_CURRENT_CTRL = 0x0001;  ///< ICC current control
constexpr uint16_t DIRECT_DRIVE_SPI = 0x0002;  ///< Direct drive via SPI
constexpr uint16_t DIRECT_DRIVE_DRV0 = 0x0003; ///< Direct drive via DRV0 pin
constexpr uint16_t DIRECT_DRIVE_DRV1 = 0x0004; ///< Direct drive via DRV1 pin
constexpr uint16_t FREE_RUN_MEAS = 0x000C;     ///< Free running measurement
constexpr uint16_t MODE_MASK = 0x000F;         ///< Mode mask

constexpr uint16_t DEFAULT = OFF; ///< Default (off)
} // namespace CH_MODE

//==============================================================================
// TON REGISTER - Direct-Drive On-Time (Per Channel, offset 0x000D)
//==============================================================================

/**
 * @brief TON register bit definitions.
 *
 * @details
 * Per datasheet \u00a75.3.3.12 p.96. Provides:
 *   bits  9:0   TON_MANT        On-time mantissa (exponent comes from
 *                               DITHER_CLK_DIV.EXP, NOT PERIOD.EXP).
 *   bits 15:10  OLSG_TIMEOUT    Open-load / short-to-ground check timeout.
 *
 * Used in DIRECT_DRIVE_SPI mode to control constant on-time without dither.
 * Formula: t_ON = TON_MANT * 2^EXP / fSYS (EXP from per-channel dither clock).
 */
namespace TON {
constexpr uint16_t TON_MANT_MASK      = 0x03FFu; ///< 10-bit mantissa mask
constexpr uint16_t OLSG_TIMEOUT_SHIFT = 10u;     ///< OLSG timeout shift
constexpr uint16_t OLSG_TIMEOUT_MASK  = 0xFC00u; ///< 6-bit OLSG timeout mask

constexpr uint16_t DEFAULT = 0x0000;             ///< POR default

/// Pack TON_MANT + OLSG_TIMEOUT into the raw register value.
[[nodiscard]] constexpr uint16_t Build(uint16_t ton_mant, uint8_t olsg_timeout) noexcept {
  return static_cast<uint16_t>((ton_mant & TON_MANT_MASK)
       | ((static_cast<uint16_t>(olsg_timeout) << OLSG_TIMEOUT_SHIFT) & OLSG_TIMEOUT_MASK));
}
} // namespace TON

//==============================================================================
// INTEGRATOR_LIMIT REGISTER - ICC Integrator Limit (Per Channel, offset 0x0003)
//==============================================================================

/**
 * @brief INTEGRATOR_LIMIT register bit definitions.
 *
 * @details
 * Per datasheet \u00a75.3.3.4 p.89. Sets the ICC integrator clamp used when
 * recovering from disturbances:
 *   bits  9:0   LIM_VALUE_ABS        Primary integrator limit (|lim|)
 *   bits 14:10  AUTO_LIM_VALUE_ABS   Auto-limit setpoint delta (|auto_lim|)
 *
 * Constraint (\u00a74.6.2.3): AUTO_LIM_VALUE_ABS must be greater than
 * MIN_INT_THRESH+3 to avoid integrator windup on small setpoint changes.
 */
namespace INTEGRATOR_LIMIT {
constexpr uint16_t LIM_VALUE_ABS_MASK      = 0x03FFu; ///< 10-bit
constexpr uint16_t AUTO_LIM_VALUE_ABS_SHIFT = 10u;
constexpr uint16_t AUTO_LIM_VALUE_ABS_MASK  = 0x7C00u; ///< 5-bit

constexpr uint16_t DEFAULT = 0x0000;

[[nodiscard]] constexpr uint16_t Build(uint16_t lim_value_abs,
                                       uint8_t  auto_lim_value_abs) noexcept {
  return static_cast<uint16_t>((lim_value_abs & LIM_VALUE_ABS_MASK)
       | ((static_cast<uint16_t>(auto_lim_value_abs) << AUTO_LIM_VALUE_ABS_SHIFT)
          & AUTO_LIM_VALUE_ABS_MASK));
}
} // namespace INTEGRATOR_LIMIT

//==============================================================================
// CTRL_INT_THRESH REGISTER - ICC Integrator Threshold (Per Channel, offset 0x000E)
//==============================================================================

/**
 * @brief CTRL_INT_THRESH register bit definitions.
 *
 * @details
 * Per datasheet \u00a75.3.3.13 p.97. Seeds the integrator + latches the
 * current PWM period mantissa for fast transient response.
 *   bits 15:8   PERIOD_MANT         11-bit mantissa (only 8 fit here; the
 *                                   extra bits live in PERIOD.MANT; setting
 *                                   PERIOD_MANT=0 switches the channel to
 *                                   manual on-time mode using TON.TON_MANT.)
 *   bits  7:0   INT_THRESH          Signed 8-bit integrator seed value.
 */
namespace CTRL_INT_THRESH {
constexpr uint16_t INT_THRESH_MASK    = 0x00FFu; ///< signed 8-bit
constexpr uint16_t PERIOD_MANT_SHIFT  = 8u;
constexpr uint16_t PERIOD_MANT_MASK   = 0xFF00u; ///< upper 8 bits of period_mant

constexpr uint16_t DEFAULT = 0x0000;

/// Build raw value. int_thresh is signed (int8_t range).
[[nodiscard]] constexpr uint16_t Build(int8_t int_thresh, uint8_t period_mant_hi) noexcept {
  return static_cast<uint16_t>((static_cast<uint16_t>(static_cast<uint8_t>(int_thresh))
          & INT_THRESH_MASK)
       | ((static_cast<uint16_t>(period_mant_hi) << PERIOD_MANT_SHIFT) & PERIOD_MANT_MASK));
}
} // namespace CTRL_INT_THRESH

//==============================================================================
// CLK_DIV REGISTER (0x0019) - Central Clock Source and PLL Divider
//==============================================================================

/**
 * @brief CLK_DIV bit definitions.
 *
 * @details
 * Per datasheet \u00a75.3.2.16 p.71. Selects between external clock and the
 * internal oscillator and configures the PLL generating fSYS from the
 * external clock (when EXT_CLK=1):
 *   bit 15       EXT_CLK        0 = internal oscillator, 1 = external + PLL
 *   bits 14:9    PLL_REFDIV     Reference divider R  (fREF = fCLK / (R+1))
 *   bits  8:0    PLL_FBDIV      Feedback divider  N  (fSYS = fREF * (N+1))
 *
 * Resulting system clock must equal 28 MHz \u00b1 tolerances for ICC math to
 * be valid. Typical external clock is 4 or 8 MHz.
 */
namespace CLK_DIV {
constexpr uint16_t EXT_CLK_BIT      = (1u << 15);
constexpr uint16_t PLL_REFDIV_SHIFT = 9u;
constexpr uint16_t PLL_REFDIV_MASK  = 0x7E00u; // 6-bit
constexpr uint16_t PLL_FBDIV_MASK   = 0x01FFu; // 9-bit

constexpr uint32_t F_SYS_TARGET_HZ  = 28'000'000UL; ///< Target fSYS

/// Pack the external-clock PLL configuration.
[[nodiscard]] constexpr uint16_t BuildExternalPll(uint8_t pll_refdiv,
                                                  uint16_t pll_fbdiv) noexcept {
  return static_cast<uint16_t>(EXT_CLK_BIT
       | ((static_cast<uint16_t>(pll_refdiv) << PLL_REFDIV_SHIFT) & PLL_REFDIV_MASK)
       | (pll_fbdiv & PLL_FBDIV_MASK));
}

/// Internal-oscillator configuration (no PLL).
constexpr uint16_t INTERNAL_OSC = 0x0000u;

/// Compute the divider pair (R,N) for a given external fCLK aiming at 28 MHz.
/// Simple heuristic: set R=0 (no reference division) and N=(28e6/fCLK)-1.
/// Caller should check fCLK is in the supported 1\u20138 MHz range.
struct PllConfig { uint8_t refdiv; uint16_t fbdiv; uint32_t actual_f_sys_hz; };
[[nodiscard]] inline PllConfig CalculatePllFromExternalHz(uint32_t f_clk_hz) noexcept {
  PllConfig cfg{0u, 0u, 0u};
  if (f_clk_hz == 0) return cfg;
  const uint32_t n_plus_1 = (F_SYS_TARGET_HZ + (f_clk_hz / 2)) / f_clk_hz;
  if (n_plus_1 == 0 || n_plus_1 > 512) return cfg;
  cfg.refdiv        = 0u;
  cfg.fbdiv         = static_cast<uint16_t>(n_plus_1 - 1u);
  cfg.actual_f_sys_hz = f_clk_hz * n_plus_1;
  return cfg;
}
} // namespace CLK_DIV

//==============================================================================
// SFF_BIST REGISTER (0x003F) - Safe State / BIST Control
//==============================================================================

/**
 * @brief SFF_BIST bit definitions.
 *
 * @details
 * Per datasheet \u00a75.3.2.18 p.74. Triggers the built-in safe-state
 * logic-BIST and surfaces the result status:
 *   bit 0  SFF_BIST_EN    Start / run BIST (write 1 to trigger)
 *   bit 1  SFF_BIST_DONE  1 = BIST completed
 *   bit 2  SFF_BIST_FAIL  1 = BIST detected a stuck-at / logic fault
 *   bit 3  SFF_BIST_UERR  1 = Uncorrectable register error during BIST
 *   bit 4  SFF_BIST_CERR  1 = Correctable register error during BIST
 */
namespace SFF_BIST {
constexpr uint16_t EN   = (1u << 0);
constexpr uint16_t DONE = (1u << 1);
constexpr uint16_t FAIL = (1u << 2);
constexpr uint16_t UERR = (1u << 3);
constexpr uint16_t CERR = (1u << 4);

constexpr uint16_t DEFAULT = 0x0000;
} // namespace SFF_BIST

//==============================================================================
// PIN_STAT REGISTER (0x0201) - Input Pin Status (read-only)
//==============================================================================

/**
 * @brief PIN_STAT bit definitions.
 *
 * @details
 * Per datasheet \u00a75.3.2.20 p.76. Read-only snapshot of the driver's
 * input control pins and the FAULTN output feedback:
 *   bit 0  DRV0        Level of DRV0 input pin
 *   bit 1  DRV1        Level of DRV1 input pin
 *   bit 4  EN          Level of EN pin
 *   bit 5  FAULTN      Driver side of the FAULTN open-drain output
 *   bit 6  FAULTN_FB   External feedback of FAULTN (1 = line high)
 */
namespace PIN_STAT {
constexpr uint16_t DRV0      = (1u << 0);
constexpr uint16_t DRV1      = (1u << 1);
constexpr uint16_t EN        = (1u << 4);
constexpr uint16_t FAULTN    = (1u << 5);
constexpr uint16_t FAULTN_FB = (1u << 6);
} // namespace PIN_STAT

//==============================================================================
// FAULT_MASK0/1/2 REGISTERS (0x0016-0x0018) - Contribution to FAULTN
//==============================================================================

/**
 * @brief FAULT_MASK bit definitions.
 *
 * @details
 * Per datasheet \u00a75.3.2.14-5.3.2.17 p.69-72. Bit=1 means the condition
 * pulls FAULTN low; bit=0 masks it out. Organized across three registers
 * for error/warn/supply domains.
 */
namespace FAULT_MASK0 {
constexpr uint16_t CH_ERR_MASK     = 0x003Fu;   ///< Per-channel error (CH0..5)
constexpr uint16_t EN_PIN_MASK     = (1u << 13);
constexpr uint16_t SUP_NOK_INT_MASK = (1u << 14);
constexpr uint16_t SUP_NOK_EXT_MASK = (1u << 15);
constexpr uint16_t DEFAULT          = 0xC03Fu;
} // namespace FAULT_MASK0

namespace FAULT_MASK1 {
constexpr uint16_t CH_WARN_MASK = 0x003Fu;    ///< Per-channel warning (CH0..5)
constexpr uint16_t COTWARN_MASK = (1u << 12);
constexpr uint16_t COTERR_MASK  = (1u << 13);
constexpr uint16_t CLK_LOW_MASK = (1u << 14);
constexpr uint16_t DEFAULT      = 0x703Fu;
} // namespace FAULT_MASK1

namespace FAULT_MASK2 {
constexpr uint16_t VBAT_UV_MASK = (1u << 0);
constexpr uint16_t VBAT_OV_MASK = (1u << 1);
constexpr uint16_t VIO_UV_MASK  = (1u << 2);
constexpr uint16_t VIO_OV_MASK  = (1u << 3);
constexpr uint16_t VDD_UV_MASK  = (1u << 4);
constexpr uint16_t VDD_OV_MASK  = (1u << 5);
constexpr uint16_t DEFAULT      = 0x003Fu;
} // namespace FAULT_MASK2

//==============================================================================
// DITHER CONTROL REGISTER - Per Channel
//==============================================================================

/**
 * @brief DITHER_CTRL register bit definitions
 *
 * @par Bit Map:
 * @verbatim
 * Bits 15-14: FAST_MEAS   - Fast measurement period
 * Bit 13    : DEEP_DITHER - Deep dither feature
 * Bits 11-0 : STEP_SIZE   - Dither step size
 * @endverbatim
 */
namespace DITHER_CTRL {
constexpr uint16_t STEP_SIZE_MASK = 0x0FFF;    ///< Step size mask
constexpr uint16_t DEEP_DITHER = (1 << 13);    ///< Deep dither enable
constexpr uint16_t FAST_MEAS_DITH = (0 << 14); ///< Dither period
constexpr uint16_t FAST_MEAS_HALF = (1 << 14); ///< Half dither period
constexpr uint16_t FAST_MEAS_QUAD = (2 << 14); ///< Quarter dither period
constexpr uint16_t FAST_MEAS_MASK = 0xC000;    ///< Fast meas mask

constexpr uint16_t DEFAULT = 0x0000; ///< Default value
} // namespace DITHER_CTRL

//==============================================================================
// DITHER STEP REGISTER - Per Channel
//==============================================================================

/**
 * @brief DITHER_STEP register bit definitions
 *
 * @par Bit Map:
 * @verbatim
 * Bits 15-8: STEPS - Number of dither steps
 * Bits 7-0 : FLAT  - Flat period on top/bottom
 * @endverbatim
 */
namespace DITHER_STEP {
constexpr uint16_t FLAT_MASK = 0x00FF;  ///< Flat period mask
constexpr uint16_t STEPS_SHIFT = 8;     ///< Steps shift
constexpr uint16_t STEPS_MASK = 0xFF00; ///< Steps mask

constexpr uint16_t DEFAULT = 0x0000; ///< Default value
} // namespace DITHER_STEP

//==============================================================================
// DITHER HELPER FUNCTIONS
//==============================================================================

//==============================================================================
// DITHER_CLK_DIV REGISTER - Per Channel  (offset 0x0004 within channel bank)
//==============================================================================

/**
 * @brief DITHER_CLK_DIV register bitfield definitions and helpers
 *
 * @details
 * Per datasheet §5.3.3.7 (Dither Clock Register, offset 0x0004):
 *
 *   Bit 15      DITHER_SETPOINT_SYNC_EN  (0 = disabled, 1 = enabled)
 *   Bit 14      DITHER_PWM_SYNC_EN       (0 = disabled, 1 = enabled)
 *   Bits 13:10  EXP                      (4-bit exponent, 0..15)
 *   Bits 9:0    MANT                     (10-bit mantissa, 0..1023)
 *
 * The dither reference clock is `tref_clk = MANT × 2^EXP / fSYS`. From this,
 * the dither period is `TDither = (4×STEPS + 2×FLAT) × tref_clk` and the
 * averaged feedback measurement period is `Tmeas = TDither` in ICC mode
 * (per §4.4.2 "Channel Modes").
 *
 * **POR reset value is 0x0000** which gives MANT=0, EXP=0 → tref_clk = 0,
 * which means TDither = 0 and the per-channel feedback averager NEVER runs
 * (FB_DC / FB_I_AVG / FB_VBAT stay at 0 forever). DITHER_CLK_DIV must
 * therefore be explicitly programmed before any channel feedback can be
 * read back from those registers.
 */
namespace DITHER_CLK_DIV {
constexpr uint16_t MANT_MASK         = 0x03FFu;       ///< 10-bit mantissa mask
constexpr uint16_t EXP_SHIFT         = 10u;           ///< Exponent bit position
constexpr uint16_t EXP_MASK          = 0x3C00u;       ///< 4-bit exponent mask
constexpr uint8_t  EXP_VALUE_MASK    = 0x0Fu;         ///< Exponent value (4 bits)
constexpr uint16_t DITHER_PWM_SYNC_EN_BIT      = (1u << 14);
constexpr uint16_t DITHER_SETPOINT_SYNC_EN_BIT = (1u << 15);

constexpr uint32_t F_SYS_HZ          = 28'000'000UL;  ///< System clock (28 MHz, §4.2.1)
constexpr float    F_SYS_PERIOD_US   = 1.0F / 28.0F;  ///< 1 / fSYS (µs, ≈35.71 ns)

/// Encoded DITHER_CLK_DIV configuration.
struct ClkDivConfig {
  uint16_t mantissa;                  ///< MANT field (0..1023)
  uint8_t  exponent;                  ///< EXP field  (0..15)
  bool     dither_pwm_sync_en;        ///< DITHER_PWM_SYNC_EN
  bool     dither_setpoint_sync_en;   ///< DITHER_SETPOINT_SYNC_EN

  /// Compute the resulting tref_clk in microseconds.
  [[nodiscard]] constexpr float CalculateTrefClkUs() const noexcept {
    return static_cast<float>(mantissa)
         * static_cast<float>(1ULL << exponent)
         * F_SYS_PERIOD_US;
  }

  /// Pack into the raw 16-bit register value.
  [[nodiscard]] constexpr uint16_t ToRegister() const noexcept {
    return (mantissa & MANT_MASK)
         | ((static_cast<uint16_t>(exponent) & EXP_VALUE_MASK) << EXP_SHIFT)
         | (dither_pwm_sync_en      ? DITHER_PWM_SYNC_EN_BIT      : 0u)
         | (dither_setpoint_sync_en ? DITHER_SETPOINT_SYNC_EN_BIT : 0u);
  }
};

/**
 * @brief Pick MANT/EXP that approximate a desired tref_clk in microseconds.
 *
 * Tries every exponent 0..15 and rounds MANT to the nearest valid value
 * (1..1023). Returns the closest representable tref_clk.
 *
 * @note MANT is forced ≥ 1 (MANT=0 disables the chip's averager).
 *
 * @param  t_ref_clk_us  Desired reference clock period in microseconds.
 * @return ClkDivConfig with the dither-sync flags zeroed; the caller can
 *         OR them on as needed.
 */
[[nodiscard]] inline ClkDivConfig CalculateFromTrefClkUs(float t_ref_clk_us) noexcept {
  ClkDivConfig best{};
  best.mantissa = 1;
  best.exponent = 0;
  best.dither_pwm_sync_en      = false;
  best.dither_setpoint_sync_en = false;

  if (t_ref_clk_us <= 0.0F) return best;

  float best_err = 1e30F;
  for (uint8_t exp = 0; exp <= 15; ++exp) {
    const float divisor = static_cast<float>(1ULL << exp) * F_SYS_PERIOD_US;
    const float mant_f  = t_ref_clk_us / divisor;
    if (mant_f < 1.0F)    continue;       // mantissa would round to 0
    if (mant_f > 1023.0F) continue;       // out of 10-bit range
    const uint16_t mant = static_cast<uint16_t>(mant_f + 0.5F);
    const float    actual = static_cast<float>(mant) * divisor;
    const float    err    = std::fabs(actual - t_ref_clk_us);
    if (err < best_err) {
      best_err = err;
      best.mantissa = mant;
      best.exponent = exp;
    }
  }
  return best;
}

}  // namespace DITHER_CLK_DIV

/**
 * @brief Dither configuration helper functions
 *
 * @details
 * Provides high-level functions to calculate dither parameters from
 * user-friendly values.
 *
 * **Formulas**:
 * - I_dither = STEPS × STEP_SIZE × 2A / 32767
 * - T_dither = [4×STEPS + 2×FLAT] × t_ref_clk
 *
 * Where t_ref_clk is set by the per-channel DITHER_CLK_DIV register
 * (see DITHER_CLK_DIV namespace above). The chip's POR default of
 * DITHER_CLK_DIV = 0x0000 makes t_ref_clk = 0, so any channel that
 * needs feedback values populated MUST have DITHER_CLK_DIV programmed
 * to a non-zero MANT/EXP combination first.
 */
namespace DITHER {
constexpr float F_SYS_HZ = 28'000'000.0F;                ///< System clock frequency (28 MHz)
constexpr float DEFAULT_T_REF_CLK_US = 1.0F / 28.0F;     ///< tref_clk at MANT=1, EXP=0 (≈35.71 ns)

/**
 * @brief Dither configuration structure
 */
struct DitherConfig {
  uint16_t step_size; ///< Dither step size (0-4095)
  uint8_t num_steps;  ///< Number of steps in quarter period (0-255)
  uint8_t flat_steps; ///< Flat period steps at top/bottom (0-255)

  /**
   * @brief Calculate dither amplitude in mA
   * @param parallel_mode true if channel is in parallel mode
   * @return Dither amplitude in milliamperes
   */
  [[nodiscard]] constexpr float CalculateAmplitudeMa(bool parallel_mode = false) const noexcept {
    uint32_t max_current = parallel_mode ? 4000 : 2000;
    float amplitude = (static_cast<float>(num_steps) * static_cast<float>(step_size) *
                       static_cast<float>(max_current)) /
                      32767.0F;
    return amplitude;
  }

  /**
   * @brief Calculate dither period in microseconds
   * @param t_ref_clk_us Reference clock period in microseconds (default: 0.125 µs)
   * @return Dither period in microseconds
   */
  [[nodiscard]] constexpr float CalculatePeriodUs(
      float t_ref_clk_us = DEFAULT_T_REF_CLK_US) const noexcept {
    return (4.0F * static_cast<float>(num_steps) + 2.0F * static_cast<float>(flat_steps)) *
           t_ref_clk_us;
  }
};

/**
 * @brief Calculate dither configuration from amplitude and frequency
 *
 * @param amplitude_ma Desired dither amplitude in milliamperes
 * @param frequency_hz Desired dither frequency in Hz
 * @param parallel_mode true if channel is in parallel mode
 * @param t_ref_clk_us Reference clock period in microseconds (default: 0.125 µs)
 * @return DitherConfig structure
 *
 * @details
 * Automatically calculates step_size, num_steps, and flat_steps to achieve
 * the desired amplitude and frequency. Uses reasonable defaults for steps
 * and flat period if not specified.
 */
[[nodiscard]] inline DitherConfig CalculateFromAmplitudeFrequency(
    float amplitude_ma,  // Current amplitude in milliamperes
    float frequency_hz,  // Dither frequency in hertz
    bool parallel_mode = false,
    float t_ref_clk_us = DEFAULT_T_REF_CLK_US) noexcept {

  DitherConfig config{};

  // Calculate desired period from frequency
  float period_us = 1'000'000.0F / frequency_hz;

  // Calculate dither period from formula: T_dither = [4×STEPS + 2×FLAT] × t_ref_clk
  // Use reasonable defaults: num_steps = 16, flat_steps = 2
  constexpr uint8_t default_steps = 16;
  constexpr uint8_t default_flat = 2;

  float calculated_period = (4.0F * default_steps + 2.0F * default_flat) * t_ref_clk_us;

  // If calculated period doesn't match desired, adjust steps
  if (calculated_period < period_us * 0.9F || calculated_period > period_us * 1.1F) {
    // Adjust num_steps to get closer to desired period
    float target_steps = (period_us / t_ref_clk_us - 2.0F * default_flat) / 4.0F;
    // Clamp to valid range before casting to uint8_t
    target_steps = std::max(1.0F, target_steps);
    target_steps = std::min(255.0F, target_steps);
    config.num_steps = static_cast<uint8_t>(std::lround(target_steps));
  } else {
    config.num_steps = default_steps;
  }

  config.flat_steps = default_flat;

  // Calculate step_size from amplitude formula: I_dither = STEPS × STEP_SIZE × 2A / 32767
  uint32_t max_current = parallel_mode ? 4000 : 2000;
  float step_size_f = (amplitude_ma * 32767.0F) /
                      (static_cast<float>(config.num_steps) * static_cast<float>(max_current));

  config.step_size = static_cast<uint16_t>(std::lround(step_size_f));
  config.step_size = std::min(config.step_size, static_cast<uint16_t>(DITHER_CTRL::STEP_SIZE_MASK));

  return config;
}
} // namespace DITHER

//==============================================================================
// VBAT THRESHOLD HELPER FUNCTIONS
//==============================================================================

/**
 * @brief VBAT threshold helper functions
 *
 * @details
 * Provides functions to convert between voltage values and register values.
 *
 * **Formula**: V_BAT = register_value × 0.16208V
 *              register_value = V_BAT / 0.16208V
 *
 * Valid range: 0V to ~41.4V (255 × 0.16208V)
 */
namespace VBAT_THRESHOLD {
constexpr float LSB_VOLTAGE = 0.16208F; ///< Voltage per LSB (0.16208V)
constexpr float MIN_VOLTAGE = 0.0F;     ///< Minimum voltage (0V)
constexpr float MAX_VOLTAGE = 41.4F;    ///< Maximum voltage (255 × 0.16208V)

/**
 * @brief Calculate register value from voltage
 * @param voltage_volts Voltage in volts
 * @return Register value (0-255), or 0 if out of range
 */
[[nodiscard]] constexpr uint8_t CalculateFromVoltage(float voltage_volts) noexcept {
  if (voltage_volts < MIN_VOLTAGE || voltage_volts > MAX_VOLTAGE) {
    return 0;
  }
  float register_value_f = voltage_volts / LSB_VOLTAGE;
  // Clamp to valid range before casting to uint8_t
  register_value_f = std::max(0.0F, register_value_f);
  register_value_f = std::min(255.0F, register_value_f);
  return static_cast<uint8_t>(std::lround(register_value_f)); // Round
}

/**
 * @brief Calculate voltage from register value
 * @param register_value Register value (0-255)
 * @return Voltage in volts
 */
[[nodiscard]] constexpr float CalculateVoltage(uint8_t register_value) noexcept {
  return static_cast<float>(register_value) * LSB_VOLTAGE;
}
} // namespace VBAT_THRESHOLD

//==============================================================================
// VOLTAGE FEEDBACK HELPER FUNCTIONS
//==============================================================================

/**
 * @brief Voltage feedback helper functions for FB_VOLTAGE1 and FB_VOLTAGE2
 *
 * @details
 * FB_VOLTAGE1 (0x0203) contains VIO and VDD measurements (22-bit reply frame):
 * - VIO: bits [10:0] (11 bits) - Formula: V_IO = 0.0034534 V × <VIO>
 * - VDD: bits [21:11] (11 bits) - Formula: V_DD = 0.0034534 V × <VDD>
 *
 * FB_VOLTAGE2 (0x0204) contains VBAT and temperature (22-bit reply frame):
 * - VBAT: bits [21:11] (11 bits) - Formula: V_BAT = 41.47 V × <VBAT>/(2^11-1) = 41.47 V ×
 * <VBAT>/2047
 * - TEMP_VALUE: bits [10:0] (11 bits) - Temperature feedback
 *
 * @note These registers return 22-bit reply frames, so the full 22-bit value must be extracted.
 */
namespace VOLTAGE_FEEDBACK {
// VIO/VDD conversion constants (from FB_VOLTAGE1)
constexpr float VIO_VDD_LSB_VOLTAGE = 0.0034534F; ///< Voltage per LSB for VIO/VDD (0.0034534V)
constexpr uint32_t VIO_VDD_MASK = 0x7FF;          ///< 11-bit mask (bits 10:0)
constexpr uint32_t VDD_SHIFT = 11;                ///< VDD is in bits [21:11]

// VBAT conversion constants (from FB_VOLTAGE2)
constexpr float VBAT_MAX_VOLTAGE = 41.47F; ///< Maximum VBAT voltage (41.47V)
constexpr uint32_t VBAT_MAX_COUNT = 2047;  ///< Maximum count (2^11 - 1)
constexpr uint32_t VBAT_MASK = 0x7FF;      ///< 11-bit mask (bits 10:0)
constexpr uint32_t VBAT_SHIFT = 11;        ///< VBAT is in bits [21:11]

/**
 * @brief Extract VIO voltage from FB_VOLTAGE1 register (22-bit value)
 * @param register_value 22-bit register value from FB_VOLTAGE1
 * @return VIO voltage in millivolts
 */
[[nodiscard]] constexpr uint16_t ExtractVioMillivolts(uint32_t register_value) noexcept {
  uint32_t vio_raw = (register_value >> 0) & VIO_VDD_MASK; // Bits [10:0]
  float vio_volts = static_cast<float>(vio_raw) * VIO_VDD_LSB_VOLTAGE;
  return static_cast<uint16_t>(std::lround(vio_volts * 1000.0F)); // Convert to mV and round
}

/**
 * @brief Extract VDD voltage from FB_VOLTAGE1 register (22-bit value)
 * @param register_value 22-bit register value from FB_VOLTAGE1
 * @return VDD voltage in millivolts
 */
[[nodiscard]] constexpr uint16_t ExtractVddMillivolts(uint32_t register_value) noexcept {
  uint32_t vdd_raw = (register_value >> VDD_SHIFT) & VIO_VDD_MASK; // Bits [21:11]
  float vdd_volts = static_cast<float>(vdd_raw) * VIO_VDD_LSB_VOLTAGE;
  return static_cast<uint16_t>(std::lround(vdd_volts * 1000.0F)); // Convert to mV and round
}

/**
 * @brief Extract VBAT voltage from FB_VOLTAGE2 register (22-bit value)
 * @param register_value 22-bit register value from FB_VOLTAGE2
 * @return VBAT voltage in millivolts
 */
[[nodiscard]] constexpr uint16_t ExtractVbatMillivolts(uint32_t register_value) noexcept {
  uint32_t vbat_raw = (register_value >> VBAT_SHIFT) & VBAT_MASK; // Bits [21:11]
  float vbat_volts =
      (static_cast<float>(vbat_raw) / static_cast<float>(VBAT_MAX_COUNT)) * VBAT_MAX_VOLTAGE;
  return static_cast<uint16_t>(std::lround(vbat_volts * 1000.0F)); // Convert to mV and round
}

/**
 * @brief Extract temperature value from FB_VOLTAGE2 register (22-bit value)
 * @param register_value 22-bit register value from FB_VOLTAGE2
 * @return Temperature raw value (bits [10:0])
 */
[[nodiscard]] constexpr uint16_t ExtractTemperatureRaw(uint32_t register_value) noexcept {
  return static_cast<uint16_t>((register_value >> 0) & VBAT_MASK); // Bits [10:0]
}

/**
 * @brief Convert a raw TEMP_VALUE (11-bit) to die temperature in \u00b0C.
 *
 * @details
 * Per datasheet \u00a75.3.2.22 p.79 (FB_VOLTAGE2, TEMP_VALUE):
 *     TFB_Central = (<TEMP_VALUE> \u00b7 0.000593 V \u2212 0.819 V) / \u22120.0016 V/\u00b0C
 *
 * The slope is negative, so a larger raw value yields a lower temperature.
 *
 * @param temp_value Raw 11-bit TEMP_VALUE field from FB_VOLTAGE2[10:0].
 * @return Central die temperature in degrees Celsius (can be negative).
 */
[[nodiscard]] constexpr float TemperatureCelsiusFromRaw(uint16_t temp_value) noexcept {
  constexpr float K_VPERLSB  = 0.000593F;   // V per LSB
  constexpr float K_OFFSET_V = 0.819F;      // V offset
  constexpr float K_SLOPE_VPERC = -0.0016F; // V per \u00b0C (negative slope)
  const float v = static_cast<float>(temp_value & 0x7FFu) * K_VPERLSB - K_OFFSET_V;
  return v / K_SLOPE_VPERC;
}

/**
 * @brief Convert a raw 22-bit FB_VOLTAGE2 read to die temperature in \u00b0C.
 */
[[nodiscard]] constexpr float TemperatureCelsiusFromFbVoltage2(uint32_t register_value) noexcept {
  return TemperatureCelsiusFromRaw(ExtractTemperatureRaw(register_value));
}
} // namespace VOLTAGE_FEEDBACK

//==============================================================================
// FB_IMIN_IMAX DECODER - Per-Channel Signed Min/Max Load Current
//==============================================================================

/**
 * @brief Decoder for the FB_IMIN_IMAX register (ChannelReg::FB_IMIN_IMAX).
 *
 * @details
 * Per datasheet \u00a75.3.3.17 p.101, FB_IMIN_IMAX is a 22-bit reply frame
 * containing two 10-bit SIGNED (two's-complement) fields:
 *
 *   bits  9:0   IMIN   I_min = <IMIN> \u00b7 4 A / (2^9 \u2212 1) = <IMIN> \u00b7 4 A / 511
 *   bits 19:10  IMAX   I_max = <IMAX> \u00b7 4 A / 511
 *   bits 21:20  reserved
 *
 * Full-scale range is approximately \u00b14 A (values 512\u2026\u22121 map to
 * \u22124.007\u2026+4.0 A). Negative values represent recirculation current.
 *
 * @note The previous driver decoded these as unsigned 8-bit halves; that
 *       rendered all min/max reports garbage. Always route through this
 *       namespace.
 */
namespace FB_IMIN_IMAX {
constexpr uint32_t IMIN_SHIFT = 0u;
constexpr uint32_t IMIN_MASK  = 0x3FFu;      // 10-bit
constexpr uint32_t IMAX_SHIFT = 10u;
constexpr uint32_t IMAX_MASK  = 0x3FFu;      // 10-bit
constexpr int32_t  FULL_SCALE_mA = 4000;     // 4 A full scale
constexpr int32_t  FULL_SCALE_COUNT = 511;   // 2^9 - 1

/// Sign-extend a 10-bit two's-complement field to int16_t.
[[nodiscard]] constexpr int16_t SignExtend10(uint32_t v) noexcept {
  const uint32_t masked = v & 0x3FFu;
  return static_cast<int16_t>((masked & 0x200u) ? (masked | 0xFC00u) : masked);
}

/// Extract signed 10-bit IMIN field from raw FB_IMIN_IMAX read.
[[nodiscard]] constexpr int16_t ExtractIMin(uint32_t raw) noexcept {
  return SignExtend10((raw >> IMIN_SHIFT) & IMIN_MASK);
}

/// Extract signed 10-bit IMAX field from raw FB_IMIN_IMAX read.
[[nodiscard]] constexpr int16_t ExtractIMax(uint32_t raw) noexcept {
  return SignExtend10((raw >> IMAX_SHIFT) & IMAX_MASK);
}

/// Convert a signed 10-bit IMIN/IMAX field value to milliamperes.
[[nodiscard]] constexpr int32_t FieldToMilliamps(int16_t field) noexcept {
  // I [mA] = field * 4000 / 511 (round toward zero)
  return (static_cast<int32_t>(field) * FULL_SCALE_mA) / FULL_SCALE_COUNT;
}

/// Decode IMIN in mA from a raw FB_IMIN_IMAX read.
[[nodiscard]] constexpr int32_t IMin_mA(uint32_t raw) noexcept {
  return FieldToMilliamps(ExtractIMin(raw));
}

/// Decode IMAX in mA from a raw FB_IMIN_IMAX read.
[[nodiscard]] constexpr int32_t IMax_mA(uint32_t raw) noexcept {
  return FieldToMilliamps(ExtractIMax(raw));
}
} // namespace FB_IMIN_IMAX

//==============================================================================
// PER-CHANNEL FEEDBACK HELPERS — FB_DC, FB_I_AVG, FB_VBAT
//==============================================================================

/**
 * @brief Per-channel averaged feedback decoders for FB_DC, FB_I_AVG, FB_VBAT.
 *
 * @details
 * Per datasheet §4.10.2 "Average feedback values", each per-channel feedback
 * register is a 22-bit reply frame containing mantissa fields. The averaged
 * quantities are reconstructed as ratios of mantissa values:
 *
 *   FB_DC (offset 0x200 within channel bank):
 *     bits [10:0]  = TP_MANT  (Period Mantissa,  unsigned 11-bit)
 *     bits [21:11] = TO_MANT  (On-time Mantissa, unsigned 11-bit)
 *     →  Duty cycle (fraction)  =  TO_MANT / TP_MANT
 *     →  T_meas = TP_MANT × 2^EXP / fSYS    (EXP from FB_VBAT or FB_I_AVG)
 *     →  t_ON   = TO_MANT × 2^EXP / fSYS
 *
 *   FB_I_AVG (offset 0x202 within channel bank):
 *     bits [10:0]  = I_AVG_MANT  (signed 11-bit, two's complement)
 *     bits [15:12] = EXP         (4-bit measurement exponent)
 *     →  Iavg = 4 A × signed(I_AVG_MANT) / TP_MANT
 *
 *   FB_VBAT (offset 0x201 within channel bank):
 *     bits [10:0]  = VBAT_AVG_MANT (unsigned 11-bit)
 *     bits [15:12] = EXP            (4-bit measurement exponent)
 *     →  VBAT = 41.47 V × VBAT_AVG_MANT / TP_MANT
 *
 * @note To compute average current or duty cycle in engineering units,
 *       BOTH FB_DC and FB_I_AVG must be read for the same channel
 *       (TP_MANT lives in FB_DC, I_AVG_MANT in FB_I_AVG).
 */
namespace FB_FEEDBACK {

constexpr uint32_t MANT11_MASK     = 0x7FFu;          ///< 11-bit mantissa mask
constexpr uint32_t TP_MANT_SHIFT   = 0u;              ///< FB_DC: TP_MANT in [10:0]
constexpr uint32_t TO_MANT_SHIFT   = 11u;             ///< FB_DC: TO_MANT in [21:11]
constexpr uint32_t I_AVG_MANT_SHIFT  = 0u;            ///< FB_I_AVG: I_AVG_MANT in [10:0]
constexpr uint32_t VBAT_MANT_SHIFT   = 0u;            ///< FB_VBAT: VBAT_AVG_MANT in [10:0]
constexpr uint32_t MEAS_EXP_SHIFT    = 12u;           ///< FB_VBAT/FB_I_AVG: EXP in [15:12]
constexpr uint32_t MEAS_EXP_MASK     = 0xFu;          ///< 4-bit exponent mask

constexpr float I_AVG_FULL_SCALE_AMPS = 4.0f;         ///< Iavg formula scale (4 A)
constexpr float VBAT_FULL_SCALE_VOLTS = 41.47f;       ///< VBAT formula scale (41.47 V)

/// Sign-extend an 11-bit two's-complement value into int16_t.
[[nodiscard]] constexpr int16_t SignExtend11(uint32_t v) noexcept {
  const uint32_t masked = v & MANT11_MASK;
  return static_cast<int16_t>((masked & 0x400u) ? (masked | 0xFFFFF800u) : masked);
}

/// Extract TP_MANT (Period Mantissa, 11 bits unsigned) from a raw FB_DC read.
[[nodiscard]] constexpr uint16_t ExtractTpMant(uint32_t fb_dc_raw) noexcept {
  return static_cast<uint16_t>((fb_dc_raw >> TP_MANT_SHIFT) & MANT11_MASK);
}

/// Extract TO_MANT (On-time Mantissa, 11 bits unsigned) from a raw FB_DC read.
[[nodiscard]] constexpr uint16_t ExtractToMant(uint32_t fb_dc_raw) noexcept {
  return static_cast<uint16_t>((fb_dc_raw >> TO_MANT_SHIFT) & MANT11_MASK);
}

/// Extract signed I_AVG_MANT (11-bit two's-complement) from FB_I_AVG.
[[nodiscard]] constexpr int16_t ExtractIAvgMant(uint32_t fb_i_avg_raw) noexcept {
  return SignExtend11((fb_i_avg_raw >> I_AVG_MANT_SHIFT) & MANT11_MASK);
}

/// Extract the 4-bit measurement exponent from FB_VBAT or FB_I_AVG.
[[nodiscard]] constexpr uint8_t ExtractMeasExp(uint32_t fb_vbat_or_iavg_raw) noexcept {
  return static_cast<uint8_t>((fb_vbat_or_iavg_raw >> MEAS_EXP_SHIFT) & MEAS_EXP_MASK);
}

/**
 * @brief Compute duty cycle (0.0 – 1.0) from a raw FB_DC value.
 * @return Duty cycle as a float in [0.0, 1.0]; returns 0 if TP_MANT is 0.
 */
[[nodiscard]] constexpr float ComputeDutyCycle(uint32_t fb_dc_raw) noexcept {
  const uint16_t tp = ExtractTpMant(fb_dc_raw);
  if (tp == 0) return 0.0f;
  return static_cast<float>(ExtractToMant(fb_dc_raw)) / static_cast<float>(tp);
}

/**
 * @brief Smallest TP_MANT that makes the mantissa-ratio formula meaningful.
 *
 * The averaged current is `4000 mA x I_AVG_MANT / TP_MANT`, so TP_MANT is the
 * divisor *and* sets the resolution: 4000/TP_MANT mA per I_AVG_MANT count. At
 * 64 counts that is ~62 mA/count, already coarse; below it a single count of
 * noise swings the result by hundreds of mA. TP_MANT lands in that range only
 * when the measurement window never ran (DITHER_CLK_DIV at its POR value of 0,
 * or feedback frozen by FB_FRZ) or when the reply frame was spliced on a
 * shared bus — never during healthy operation.
 */
constexpr uint16_t kMinValidTpMant = 64u;

/// True when FB_DC reports a measurement period usable for ratio decoding.
[[nodiscard]] constexpr bool HasValidMeasurementWindow(uint32_t fb_dc_raw) noexcept {
  return ExtractTpMant(fb_dc_raw) >= kMinValidTpMant;
}

/**
 * @brief Compute average current in mA (signed) from FB_I_AVG and FB_DC.
 * @param  fb_i_avg_raw  Raw 22-bit FB_I_AVG read (provides I_AVG_MANT)
 * @param  fb_dc_raw     Raw 22-bit FB_DC read    (provides TP_MANT for the divisor)
 * @return Average load current in mA (signed); returns 0 if TP_MANT is 0.
 * @warning Callers should gate on @ref HasValidMeasurementWindow first; this
 *          function only guards against division by zero, not against the
 *          near-zero divisors that produce wildly overstated currents.
 */
[[nodiscard]] inline int32_t ComputeAverageCurrent_mA(uint32_t fb_i_avg_raw,
                                                      uint32_t fb_dc_raw) noexcept {
  const uint16_t tp = ExtractTpMant(fb_dc_raw);
  if (tp == 0) return 0;
  const int16_t i_mant = ExtractIAvgMant(fb_i_avg_raw);
  // Iavg [A]  = 4 A × I_AVG_MANT / TP_MANT
  // Iavg [mA] = 4000 × I_AVG_MANT / TP_MANT
  return static_cast<int32_t>(
      (static_cast<int32_t>(i_mant) * 4000) / static_cast<int32_t>(tp));
}

/**
 * @brief Compute averaged battery voltage in mV from FB_VBAT and FB_DC.
 * @param  fb_vbat_raw   Raw 22-bit FB_VBAT read (provides VBAT_AVG_MANT)
 * @param  fb_dc_raw     Raw 22-bit FB_DC read   (provides TP_MANT for the divisor)
 * @return Battery voltage in mV; returns 0 if TP_MANT is 0.
 */
[[nodiscard]] inline uint32_t ComputeVbatChannel_mV(uint32_t fb_vbat_raw,
                                                    uint32_t fb_dc_raw) noexcept {
  const uint16_t tp = ExtractTpMant(fb_dc_raw);
  if (tp == 0) return 0;
  const uint16_t vbat_mant = static_cast<uint16_t>(
      (fb_vbat_raw >> VBAT_MANT_SHIFT) & MANT11_MASK);
  // VBAT [V]  = 41.47 V × VBAT_AVG_MANT / TP_MANT
  // VBAT [mV] = 41470  × VBAT_AVG_MANT / TP_MANT
  return (static_cast<uint32_t>(vbat_mant) * 41470u)
       / static_cast<uint32_t>(tp);
}

}  // namespace FB_FEEDBACK

//==============================================================================
// FB_I_AVG_s16 REGISTER (Per Channel, offset 0x0204) - High-res signed average
//==============================================================================

/**
 * @brief FB_I_AVG_s16 decoder.
 *
 * @details
 * Per datasheet \u00a75.3.3.21 p.104. Provides a high-resolution signed
 * average current for calibration workflows. Reply frame is 22-bit:
 *   bits 16:0   I_AVG_s16 (signed 17-bit)  I = 4000 mA * val / 65535
 *   bits 21:20  TIME_STAMP (2-bit rolling counter, quad-seq in fast meas)
 *
 * NOTE: this is distinct from FB_I_AVG (offset 0x0202) which is compressed
 * mantissa/exponent. Use FB_I_AVG_s16 only for calibration since the
 * frame is not valid while the chip is still settling.
 */
namespace FB_I_AVG_s16 {
constexpr uint32_t I_AVG_MASK         = 0x0001FFFFu; ///< 17-bit
constexpr uint32_t I_AVG_SIGN_BIT     = (1u << 16);
constexpr uint32_t TIME_STAMP_SHIFT   = 20u;
constexpr uint32_t TIME_STAMP_MASK    = 0x00300000u; ///< 2-bit

constexpr int32_t  FULL_SCALE_mA      = 4000;
constexpr int32_t  FULL_SCALE_COUNT   = 65535;

/// Sign-extend the 17-bit signed field.
[[nodiscard]] constexpr int32_t SignExtend17(uint32_t raw17) noexcept {
  const uint32_t m = raw17 & I_AVG_MASK;
  return (m & I_AVG_SIGN_BIT) ? static_cast<int32_t>(m | 0xFFFE0000u)
                              : static_cast<int32_t>(m);
}

/**
 * @brief Raw code the frame carries before a measurement has settled.
 *
 * Exactly half of @ref FULL_SCALE_COUNT, so it decodes to a plausible-looking
 * 2000 mA. Observed on a 115 mA proportional valve whenever this register was
 * read before the channel had an established measurement window.
 */
constexpr uint32_t SETTLING_CODE = 0x8000u;

/**
 * @brief True when the frame is the pre-measurement placeholder.
 *
 * @warning Mid-scale is also the code an honest +2000 mA reading would use.
 *          That is out of range for every channel on this product (full flow
 *          is 115 mA), so rejecting it is correct here; a design driving loads
 *          near 2 A must gate on the measurement window instead.
 */
[[nodiscard]] constexpr bool IsSettlingFrame(uint32_t reply_22bit) noexcept {
  return (reply_22bit & I_AVG_MASK) == SETTLING_CODE;
}

/**
 * @brief Decode signed 17-bit raw to milliamps.
 * @warning Gate on @ref IsSettlingFrame first — this decoder cannot tell a
 *          settled measurement from the power-on placeholder.
 */
[[nodiscard]] constexpr int32_t ToMilliamps(uint32_t reply_22bit) noexcept {
  const int32_t s = SignExtend17(reply_22bit);
  return (s * FULL_SCALE_mA) / FULL_SCALE_COUNT;
}

/// Extract the 2-bit rolling timestamp.
[[nodiscard]] constexpr uint8_t ExtractTimestamp(uint32_t reply_22bit) noexcept {
  return static_cast<uint8_t>((reply_22bit & TIME_STAMP_MASK) >> TIME_STAMP_SHIFT);
}
} // namespace FB_I_AVG_s16

//==============================================================================
// FB_INT_THRESH REGISTER (Per Channel, offset 0x0205) - Integrator Threshold Readback
//==============================================================================

/**
 * @brief FB_INT_THRESH feedback reader.
 *
 * @details
 * Per datasheet \u00a75.3.3.22 p.105. Read-only snapshot of the current
 * integrator threshold value (signed 8-bit, sign-extended to 16-bit).
 * Used to monitor the auto-seeded integrator after setpoint changes.
 */
namespace FB_INT_THRESH {
constexpr uint32_t INT_THRESH_MASK = 0x0000FFFFu;

[[nodiscard]] constexpr int16_t Extract(uint32_t reply_22bit) noexcept {
  return static_cast<int16_t>(reply_22bit & INT_THRESH_MASK);
}
} // namespace FB_INT_THRESH

//==============================================================================
// FB_PERIOD_MIN_MAX REGISTER (Per Channel, offset 0x0206) - PWM Period Bounds
//==============================================================================

/**
 * @brief FB_PERIOD_MIN_MAX decoder.
 *
 * @details
 * Per datasheet \u00a75.3.3.23 p.106. Reports the minimum and maximum PWM
 * period observed during the last measurement window in 256-cycle units
 * of fSYS:
 *   bits  9:0   PERIOD_MIN  T_MIN = PERIOD_MIN * 256 / fSYS
 *   bits 19:10  PERIOD_MAX  T_MAX = PERIOD_MAX * 256 / fSYS
 */
namespace FB_PERIOD_MIN_MAX {
constexpr uint32_t PERIOD_MIN_MASK  = 0x000003FFu;
constexpr uint32_t PERIOD_MAX_SHIFT = 10u;
constexpr uint32_t PERIOD_MAX_MASK  = 0x000FFC00u;
constexpr uint32_t TICK_CYCLES      = 256u;

/// Convert a raw 10-bit count to microseconds (fSYS = 28 MHz).
[[nodiscard]] constexpr uint32_t ToMicroseconds(uint16_t raw_count) noexcept {
  // T_us = count * 256 / 28 = count * 64 / 7 (exact)
  return (static_cast<uint32_t>(raw_count) * TICK_CYCLES) / 28u;
}

[[nodiscard]] constexpr uint16_t ExtractMin(uint32_t reply_22bit) noexcept {
  return static_cast<uint16_t>(reply_22bit & PERIOD_MIN_MASK);
}
[[nodiscard]] constexpr uint16_t ExtractMax(uint32_t reply_22bit) noexcept {
  return static_cast<uint16_t>((reply_22bit & PERIOD_MAX_MASK) >> PERIOD_MAX_SHIFT);
}

[[nodiscard]] constexpr uint32_t PeriodMinMicroseconds(uint32_t reply_22bit) noexcept {
  return ToMicroseconds(ExtractMin(reply_22bit));
}
[[nodiscard]] constexpr uint32_t PeriodMaxMicroseconds(uint32_t reply_22bit) noexcept {
  return ToMicroseconds(ExtractMax(reply_22bit));
}
} // namespace FB_PERIOD_MIN_MAX

//==============================================================================
// HELPER ENUMERATIONS
//==============================================================================

/**
 * @brief Channel enumeration
 */
enum class Channel : uint8_t {
  CH0 = 0, ///< Channel 0
  CH1 = 1, ///< Channel 1
  CH2 = 2, ///< Channel 2
  CH3 = 3, ///< Channel 3
  CH4 = 4, ///< Channel 4
  CH5 = 5, ///< Channel 5

  COUNT = 6 ///< Number of channels (not a valid channel selector)
};

/**
 * @brief Channel operation mode
 */
enum class ChannelMode : uint8_t {
  OFF = 0x0,               ///< Channel off
  ICC = 0x1,               ///< Integrated Current Control
  DIRECT_DRIVE_SPI = 0x2,  ///< Direct drive via SPI TON register
  DIRECT_DRIVE_DRV0 = 0x3, ///< Direct drive via DRV0 pin
  DIRECT_DRIVE_DRV1 = 0x4, ///< Direct drive via DRV1 pin
  FREE_RUN_MEAS = 0xC      ///< Free running measurement mode
};

/**
 * @brief Slew rate enumeration
 */
enum class SlewRate : uint8_t {
  SLOW_1V0_US = 0,    ///< 1.0 V/µs
  MEDIUM_2V5_US = 1,  ///< 2.5 V/µs
  FAST_5V0_US = 2,    ///< 5.0 V/µs
  FASTEST_10V0_US = 3 ///< 10.0 V/µs
};

/**
 * @brief OFF-state diagnostic current
 */
enum class DiagCurrent : uint8_t {
  I_80UA = 0,  ///< 80 µA
  I_190UA = 1, ///< 190 µA
  I_720UA = 2, ///< 720 µA
  I_1250UA = 3 ///< 1250 µA
};

/**
 * @brief Parallel operation pairs
 */
enum class ParallelPair : uint8_t {
  NONE = 0,    ///< No parallel operation
  CH0_CH3 = 1, ///< Channels 0 and 3 paralleled
  CH1_CH2 = 2, ///< Channels 1 and 2 paralleled
  CH4_CH5 = 3  ///< Channels 4 and 5 paralleled
};

//==============================================================================
// ADDRESS CALCULATION HELPERS
//==============================================================================

/**
 * @brief Get channel base address
 * @param channel Channel number (0-5)
 * @return Base address for channel registers
 *
 * @details
 * Per datasheet Table 25, channel base addresses are:
 * - CH0: 0x0040
 * - CH1: 0x0050
 * - CH2: 0x0060
 * - CH3: 0x0070
 * - CH4: 0x0020
 * - CH5: 0x0030
 *
 * Note: Channels are NOT in sequential order, so we use a lookup table.
 */
[[nodiscard]] constexpr uint16_t GetChannelBase(Channel channel) noexcept {
  switch (channel) {
  case Channel::CH0:
    return ChannelBase::CH0;
  case Channel::CH1:
    return ChannelBase::CH1;
  case Channel::CH2:
    return ChannelBase::CH2;
  case Channel::CH3:
    return ChannelBase::CH3;
  case Channel::CH4:
    return ChannelBase::CH4;
  case Channel::CH5:
    return ChannelBase::CH5;
  default:
    return 0x0000; // Invalid channel
  }
}

/**
 * @brief Get channel register address
 * @param channel Channel number
 * @param offset Register offset from channel base
 * @return Complete register address
 */
[[nodiscard]] constexpr uint16_t GetChannelRegister(Channel channel, uint16_t offset) noexcept {
  return GetChannelBase(channel) + offset;
}

/**
 * @brief Reply-frame width a register is required to answer with.
 * @see ExpectedReplyFor
 */
enum class ExpectedReply : uint8_t {
  Bits16, ///< Reply mode 00B — 16-bit data plus a status field.
  Bits22  ///< Reply mode 01B — 22-bit data, no status field.
};

/**
 * @brief Reply width the device must use when answering a read of @p address.
 *
 * @param address Register address passed to a read.
 * @return @c Bits22 for the feedback/status registers that answer in reply
 *         mode 01B; @c Bits16 for everything else.
 *
 * @details The pipelined bus does not echo the address in a reply, so a reply
 *          that arrives in the wrong slot is otherwise indistinguishable from
 *          the register that was asked for — and it carries a **valid CRC**,
 *          so CRC alone cannot reject it. Bench measurement (Portenta Mid,
 *          2026-08-13) had 25 % of `FB_STAT` reads return the preceding
 *          `ICVID` reply, which decoded as plausible-looking status bits.
 *          Comparing the reply mode against this table catches the subset of
 *          those where the two registers differ in width, which covers every
 *          feedback register the current-control path depends on.
 *
 * @note Width is a necessary, not sufficient, check: a stale reply from
 *       another register of the *same* width still passes. Reads are retried
 *       until a well-formed reply arrives rather than trusted on first sight.
 */
[[nodiscard]] constexpr ExpectedReply ExpectedReplyFor(uint16_t address) noexcept {
  if (address == CentralReg::FB_STAT || address == CentralReg::FB_VOLTAGE1 ||
      address == CentralReg::FB_VOLTAGE2) {
    return ExpectedReply::Bits22;
  }
  /* Per-channel feedback bank: base (0x20..0x70) + 0x200. FB_I_AVG_s16 (+4)
   * and FB_INT_THRESH (+5) are 16-bit; the rest of the bank is 22-bit. */
  if (address > CentralReg::FB_VOLTAGE2) {
    const uint16_t base = static_cast<uint16_t>((address - 0x0200U) & 0x00F0U);
    const uint16_t offset = static_cast<uint16_t>(address & 0x000FU);
    const bool in_channel_bank =
        (address >= 0x0220U) && (address <= 0x0276U) && (base >= 0x0020U) &&
        (base <= 0x0070U) && (offset <= 0x0006U);
    if (in_channel_bank && offset != 0x0004U && offset != 0x0005U) {
      return ExpectedReply::Bits22;
    }
  }
  return ExpectedReply::Bits16;
}

/**
 * @brief Convert channel to index
 */
[[nodiscard]] constexpr uint8_t ToIndex(Channel ch) noexcept {
  return static_cast<uint8_t>(ch);
}

//==============================================================================
// ENUM TO_STRING HELPERS
//==============================================================================

/**
 * @brief Convert ChannelMode enum to string
 * @param mode Channel mode
 * @return String representation
 */
[[nodiscard]] inline const char* ToString(ChannelMode mode) noexcept {
  switch (mode) {
  case ChannelMode::OFF:
    return "OFF";
  case ChannelMode::ICC:
    return "ICC";
  case ChannelMode::DIRECT_DRIVE_SPI:
    return "DIRECT_DRIVE_SPI";
  case ChannelMode::DIRECT_DRIVE_DRV0:
    return "DIRECT_DRIVE_DRV0";
  case ChannelMode::DIRECT_DRIVE_DRV1:
    return "DIRECT_DRIVE_DRV1";
  case ChannelMode::FREE_RUN_MEAS:
    return "FREE_RUN_MEAS";
  default:
    return "UNKNOWN";
  }
}

/**
 * @brief Convert SlewRate enum to string
 * @param rate Slew rate
 * @return String representation
 */
[[nodiscard]] inline const char* ToString(SlewRate rate) noexcept {
  switch (rate) {
  case SlewRate::SLOW_1V0_US:
    return "SLOW_1V0_US";
  case SlewRate::MEDIUM_2V5_US:
    return "MEDIUM_2V5_US";
  case SlewRate::FAST_5V0_US:
    return "FAST_5V0_US";
  case SlewRate::FASTEST_10V0_US:
    return "FASTEST_10V0_US";
  default:
    return "UNKNOWN";
  }
}

/**
 * @brief Convert DiagCurrent enum to string
 * @param current Diagnostic current
 * @return String representation
 */
[[nodiscard]] inline const char* ToString(DiagCurrent current) noexcept {
  switch (current) {
  case DiagCurrent::I_80UA:
    return "I_80UA";
  case DiagCurrent::I_190UA:
    return "I_190UA";
  case DiagCurrent::I_720UA:
    return "I_720UA";
  case DiagCurrent::I_1250UA:
    return "I_1250UA";
  default:
    return "UNKNOWN";
  }
}

/**
 * @brief Convert ParallelPair enum to string
 * @param pair Parallel pair
 * @return String representation
 */
[[nodiscard]] inline const char* ToString(ParallelPair pair) noexcept {
  switch (pair) {
  case ParallelPair::NONE:
    return "NONE";
  case ParallelPair::CH0_CH3:
    return "CH0_CH3";
  case ParallelPair::CH1_CH2:
    return "CH1_CH2";
  case ParallelPair::CH4_CH5:
    return "CH4_CH5";
  default:
    return "UNKNOWN";
  }
}

/**
 * @brief Convert Channel enum to string
 * @param channel Channel
 * @return String representation
 */
[[nodiscard]] inline const char* ToString(Channel channel) noexcept {
  switch (channel) {
  case Channel::CH0:
    return "CH0";
  case Channel::CH1:
    return "CH1";
  case Channel::CH2:
    return "CH2";
  case Channel::CH3:
    return "CH3";
  case Channel::CH4:
    return "CH4";
  case Channel::CH5:
    return "CH5";
  default:
    return "UNKNOWN";
  }
}

/**
 * @brief Validate channel number
 */
[[nodiscard]] constexpr bool IsValidChannel(Channel ch) noexcept {
  return ToIndex(ch) < static_cast<uint8_t>(Channel::COUNT);
}

//==============================================================================
// CRC CALCULATION (SAE J1850)
//==============================================================================

/**
 * @brief Calculate SAE J1850 CRC-8
 *
 * @details
 * Polynomial: 0x1D (x^8 + x^4 + x^3 + x^2 + 1)
 * Initial value: 0xFF
 * Final XOR: 0xFF
 *
 * @param data Pointer to data bytes
 * @param length Number of bytes
 * @return CRC-8 value
 */
[[nodiscard]] constexpr uint8_t CalculateCrc8J1850(const uint8_t* data,
                                                   std::size_t length) noexcept {
  constexpr uint8_t POLY = 0x1D;
  uint8_t crc = 0xFF;

  for (std::size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if ((crc & 0x80) != 0) {
        crc = (crc << 1) ^ POLY;
      } else {
        crc = (crc << 1);
      }
    }
  }

  return crc ^ 0xFF;
}

/**
 * @brief Calculate CRC for SPI frame
 * @param frame SPI frame (CRC field should be 0)
 * @return Calculated CRC value
 *
 * @details Datasheet Rev 1.2 §5.1.2 fixes the CRC byte sequence as
 * frame[7:0], then frame[15:8], then frame[23:16] — i.e. little-endian byte
 * order, which is the *reverse* of the MSB-first wire order. On a
 * little-endian target that is exactly bytes 0..2 of the frame word.
 */
[[nodiscard]] inline uint8_t CalculateFrameCrc(const tle92466ed::SPIFrame& frame) noexcept {
  static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__,
                "CRC byte sequence (datasheet §5.1.2) assumes little-endian frame storage");
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) - Required for hardware register byte access
  const auto* bytes = reinterpret_cast<const uint8_t*>(&frame);
  // Calculate CRC on bytes 0-2 (excluding CRC byte itself at position 3)
  return CalculateCrc8J1850(bytes, 3);
}

/**
 * @brief Verify CRC in received frame
 * @param frame Received SPI frame
 * @return true if CRC is valid
 */
[[nodiscard]] inline bool VerifyFrameCrc(const tle92466ed::SPIFrame& frame) noexcept {
  tle92466ed::SPIFrame temp = frame;
  uint8_t received_crc = temp.tx_fields.crc;
  temp.tx_fields.crc = 0;
  uint8_t calculated_crc{}; // Initialize to zero
  calculated_crc = CalculateFrameCrc(temp);
  return (received_crc == calculated_crc);
}

//==============================================================================
// PHASE 2 / 4 / 5 / 6 - SHARED ENUMS AND STRUCTS
//==============================================================================

/**
 * @brief Clock source selection for ConfigureClockSource().
 */
enum class ClockSource : uint8_t {
  InternalOscillator, ///< Use internal ring oscillator (fSYS \u2248 28 MHz \u00b1 10 %)
  ExternalClockPll    ///< Use external clock on DRV0 or dedicated pin, synth via PLL
};

/**
 * @brief VIO voltage-level selection for SetVioLevel().
 */
enum class VioLevel : uint8_t { V3_3 = 0, V5_0 = 1 };

/**
 * @brief High-level supply-voltage snapshot returned by ReadAllSupplyVoltages().
 */
struct SupplyVoltages {
  uint16_t vbat_mV{0};          ///< Battery supply in millivolts
  uint16_t vio_mV{0};           ///< Logic supply in millivolts
  uint16_t vdd_mV{0};           ///< Internal analog supply in millivolts
  float    temperature_c{0.0F}; ///< Central die temperature in \u00b0C
};

/**
 * @brief Coarse operational state returned by GetOperationState().
 */
enum class OperationState : uint8_t {
  Reset,        ///< Held in reset (RESN low)
  Config,       ///< Powered and initialized, but not in mission mode
  Mission,      ///< Outputs operational
  CriticalFault ///< Critical-fault reply flag set; requires reset/clear
};

/**
 * @brief Fast-measurement mode for dither-synchronous feedback.
 */
enum class FastMeasureMode : uint8_t {
  FullPeriod    = 0, ///< Measurement window = full dither period
  HalfPeriod    = 1, ///< Measurement window = half dither period
  QuarterPeriod = 2  ///< Measurement window = quarter dither period
};

/**
 * @brief High-level dither configuration (Phase 4 superset of ConfigureDither).
 */
struct DitherSetup {
  float           amplitude_mA{0.0F};      ///< Peak-to-peak dither amplitude in mA
  float           frequency_Hz{0.0F};      ///< Dither frequency in Hz
  bool            sync_with_pwm{false};    ///< Restart dither on each PWM rising edge
  bool            sync_with_setpoint{false}; ///< Restart dither on setpoint change
  bool            deep_dither{false};      ///< Enable deep-dither (extended amplitude range)
  FastMeasureMode fast_measure{FastMeasureMode::FullPeriod}; ///< Feedback averaging window
};

/**
 * @brief Result of RunSffBist().
 */
struct BistResult {
  bool done{false};                  ///< BIST completed
  bool pass{false};                  ///< BIST passed (fail==0)
  bool uncorrectable_reg_err{false};
  bool correctable_reg_err{false};
  uint16_t raw{0};                   ///< Raw SFF_BIST register value for debug
};

/**
 * @brief Snapshot returned by `ReadPinStatus()`.
 */
struct PinStatus {
  bool drv0{false};           ///< DRV0 input pin level (PIN_STAT bit 0)
  bool drv1{false};           ///< DRV1 input pin level (PIN_STAT bit 1)
  bool en{false};             ///< EN input pin level (PIN_STAT bit 4)
  bool faultn_driver{false};  ///< FAULTN output as driven by the IC (PIN_STAT bit 5)
  bool faultn_fb{false};      ///< External FAULTN line feedback (PIN_STAT bit 6)
  uint16_t raw{0};            ///< Raw PIN_STAT register value
};

/**
 * @brief Maskable fault sources for SetFaultMask().
 *
 * Each enumerator packs a FAULT_MASK register index (0/1/2) in the upper
 * byte and the bit-mask within that register in the lower 16 bits.
 */
enum class MaskableFault : uint32_t {
  // FAULT_MASK0 — per-channel error contribution
  Ch0Error          = (0u << 24) | (1u << 0),  ///< CH0 error → FAULTN
  Ch1Error          = (0u << 24) | (1u << 1),  ///< CH1 error → FAULTN
  Ch2Error          = (0u << 24) | (1u << 2),  ///< CH2 error → FAULTN
  Ch3Error          = (0u << 24) | (1u << 3),  ///< CH3 error → FAULTN
  Ch4Error          = (0u << 24) | (1u << 4),  ///< CH4 error → FAULTN
  Ch5Error          = (0u << 24) | (1u << 5),  ///< CH5 error → FAULTN
  EnPin             = (0u << 24) | (1u << 13), ///< EN pin fault → FAULTN
  SupplyNokInternal = (0u << 24) | (1u << 14), ///< Internal supply NOK → FAULTN
  SupplyNokExternal = (0u << 24) | (1u << 15), ///< External supply NOK → FAULTN
  // FAULT_MASK1 — per-channel warning and system warnings
  Ch0Warning        = (1u << 24) | (1u << 0),  ///< CH0 warning → FAULTN
  Ch1Warning        = (1u << 24) | (1u << 1),  ///< CH1 warning → FAULTN
  Ch2Warning        = (1u << 24) | (1u << 2),  ///< CH2 warning → FAULTN
  Ch3Warning        = (1u << 24) | (1u << 3),  ///< CH3 warning → FAULTN
  Ch4Warning        = (1u << 24) | (1u << 4),  ///< CH4 warning → FAULTN
  Ch5Warning        = (1u << 24) | (1u << 5),  ///< CH5 warning → FAULTN
  CentralOtWarning  = (1u << 24) | (1u << 12), ///< Central OT warning → FAULTN
  CentralOtError    = (1u << 24) | (1u << 13), ///< Central OT error → FAULTN
  ClockLow          = (1u << 24) | (1u << 14), ///< Clock too slow → FAULTN
  // FAULT_MASK2 — supply UV/OV contribution
  VbatUv            = (2u << 24) | (1u << 0),  ///< VBAT undervoltage → FAULTN
  VbatOv            = (2u << 24) | (1u << 1),  ///< VBAT overvoltage → FAULTN
  VioUv             = (2u << 24) | (1u << 2),  ///< VIO undervoltage → FAULTN
  VioOv             = (2u << 24) | (1u << 3),  ///< VIO overvoltage → FAULTN
  VddUv             = (2u << 24) | (1u << 4),  ///< VDD undervoltage → FAULTN
  VddOv             = (2u << 24) | (1u << 5),  ///< VDD overvoltage → FAULTN
};

/** @brief Extract FAULT_MASK register index (0..2) from a `MaskableFault` value. */
[[nodiscard]] constexpr uint8_t  FaultMaskIndex(MaskableFault f) noexcept {
  return static_cast<uint8_t>((static_cast<uint32_t>(f) >> 24) & 0x3u);
}
/** @brief Extract the bit mask within the target FAULT_MASK register. */
[[nodiscard]] constexpr uint16_t FaultMaskBit(MaskableFault f) noexcept {
  return static_cast<uint16_t>(static_cast<uint32_t>(f) & 0xFFFFu);
}

/**
 * @brief High-resolution per-channel feedback snapshot (Phase 6).
 *
 * @details Populated by `ReadChannelFeedback()`. Average current uses the
 *          `FB_I_AVG` mantissa ratio with `TP_MANT` from `FB_DC` — not
 *          `FB_I_AVG_s16` (see `GetCalibrationAvgCurrent_mA()`).
 */
struct ChannelFeedback {
  int32_t  avg_current_mA{0};          ///< Average current from FB_I_AVG/FB_DC ratio (mA, signed)
  uint16_t duty_cycle_permyriad{0};    ///< Duty cycle 0..10000 (= 0.00..100.00 %)
  uint32_t avg_vbat_mV{0};             ///< Channel-referenced VBAT average (mV)
  int32_t  imin_mA{0};                 ///< Minimum current in measurement window (mA)
  int32_t  imax_mA{0};                 ///< Maximum current in measurement window (mA)
  uint32_t period_min_us{0};           ///< Shortest PWM period observed (µs)
  uint32_t period_max_us{0};           ///< Longest  PWM period observed (µs)
  int16_t  int_thresh_seed{0};         ///< Current ICC integrator threshold readback
  uint8_t  period_seq{0};              ///< Measurement exponent sequence (from FB_I_AVG EXP)
  uint8_t  quad_seq{0};                ///< Quarter-period sequence counter (reserved)
};

/**
 * @brief Result of RunSupplyMonitorSelfTest().
 */
struct SupplyMonitorSelfTestResult {
  bool uv_ov_swap_ok{false};
  bool v1v5_uv_ok{false};
  bool v1v5_ov_ok{false};
  bool ot_test_ok{false};
  bool overall_pass{false};
};

} // namespace tle92466ed
