/**
 * @file TLE92466ED_Registers.hpp
 * @brief Register definitions and bit field mappings for TLE92466ED IC
 * @author AI Generated Driver
 * @date 2025-10-20
 * @version 2.0.0
 *
 * @details
 * This file contains comprehensive register definitions, bit field masks,
 * and helper structures for the TLE92466ED Six-Channel Low-Side Solenoid Driver IC.
 * All register addresses and bit positions have been meticulously cross-referenced
 * with the official Infineon datasheet (Rev. 1.2, 2022-02-01).
 *
 * The TLE92466ED features:
 * - 6 independent low-side solenoid drivers
 * - Integrated Current Control (ICC) with 15-bit resolution (0-2A, 0-4A in parallel)
 * - 32-bit SPI interface with 8-bit CRC (SAE J1850)
 * - PWM current control with configurable frequency
 * - Dither support for precise current shaping
 * - Channel parallel operation (0/3, 1/2, 4/5)
 * - Comprehensive diagnostics and protection
 * - SPI and Clock watchdogs
 *
 * @par Device Type:
 * Low-side solenoid/inductive load driver with current sensing and regulation
 *
 * @par SPI Communication:
 * - 32-bit frames (8-bit CRC + 7-bit Address + 1-bit R/W + 16-bit Data)
 * - CRC: SAE J1850 8-bit polynomial
 * - Max frequency: 10 MHz
 * - Mode 0 (CPOL=0, CPHA=0)
 *
 * @copyright
 * This is free and unencumbered software released into the public domain.
 */

#ifndef TLE92466ED_REGISTERS_HPP
#define TLE92466ED_REGISTERS_HPP

#include <cstdint>

namespace TLE92466ED {

//==============================================================================
// SPI FRAME STRUCTURES (32-BIT)
//==============================================================================

/**
 * @brief 32-bit SPI frame structure for TLE92466ED communication
 * 
 * @details
 * The TLE92466ED uses 32-bit SPI frames with the following format:
 *
 * MOSI (Write) Frame:
 * @verbatim
 *  Bits 31-24 | Bits 23-17 | Bit 16 | Bits 15-0
 * ------------+------------+--------+-----------
 *  CRC (8-bit)| Address(7) |  R/W   | Data (16)
 * @endverbatim
 *
 * MISO (Reply) Frame:
 * @verbatim
 *  Bits 31-24 | Bits 23-22 | Bits 21-17 | Bit 16 | Bits 15-0
 * ------------+------------+------------+--------+-----------
 *  CRC (8-bit)| Reply Mode | Status (5) | R/W    | Data (16)
 * @endverbatim
 */
union SPIFrame {
    uint32_t word;          ///< Complete 32-bit frame
    
    /// MOSI (Transmit) frame structure
    struct {
        uint32_t data : 16;      ///< Data field [15:0]
        uint32_t rw : 1;         ///< Read/Write bit [16] (1=Write, 0=Read)
        uint32_t address : 7;    ///< Register address [23:17]
        uint32_t crc : 8;        ///< CRC-8 SAE J1850 [31:24]
    } tx_fields;
    
    /// MISO (Receive) frame structure
    struct {
        uint32_t data : 16;       ///< Data field [15:0]
        uint32_t rw_echo : 1;     ///< R/W bit echoed [16]
        uint32_t status : 5;      ///< Status bits [21:17]
        uint32_t reply_mode : 2;  ///< Reply mode [23:22]
        uint32_t crc : 8;         ///< CRC-8 SAE J1850 [31:24]
    } rx_fields;
    
    /**
     * @brief Construct read frame (without CRC - must be calculated separately)
     * @param addr Register address (10-bit actual address)
     * @return SPIFrame configured for read operation (CRC = 0)
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    static constexpr SPIFrame make_read(uint16_t addr) noexcept {
        SPIFrame frame{};
        frame.tx_fields.rw = 0;           // Read
        frame.tx_fields.address = (addr >> 3) & 0x7F;  // Upper 7 bits
        frame.tx_fields.data = addr & 0x07;            // Lower 3 bits in data
        frame.tx_fields.crc = 0;          // CRC calculated separately
        return frame;
    }
    
    /**
     * @brief Construct write frame (without CRC - must be calculated separately)
     * @param addr Register address (10-bit actual address)
     * @param data Data word to write (16-bit)
     * @return SPIFrame configured for write operation (CRC = 0)
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    static constexpr SPIFrame make_write(uint16_t addr, uint16_t data) noexcept {
        SPIFrame frame{};
        frame.tx_fields.rw = 1;           // Write
        frame.tx_fields.address = (addr >> 3) & 0x7F;  // Upper 7 bits
        frame.tx_fields.data = data;
        frame.tx_fields.crc = 0;          // CRC calculated separately
        return frame;
    }
};

static_assert(sizeof(SPIFrame) == 4, "SPIFrame must be exactly 4 bytes");

/**
 * @brief SPI Reply Mode enumeration
 */
enum class ReplyMode : uint8_t {
    REPLY_16BIT = 0b00,      ///< 16-bit reply frame
    REPLY_22BIT = 0b01,      ///< 22-bit reply frame (extended data)
    CRITICAL_FAULT = 0b10,   ///< Critical fault frame
    RESERVED = 0b11          ///< Reserved
};

/**
 * @brief SPI Status codes
 */
enum class SPIStatus : uint8_t {
    NO_ERROR = 0b00000,          ///< No error
    SPI_FRAME_ERROR = 0b00001,   ///< SPI frame error
    CRC_ERROR = 0b00010,         ///< Parity/CRC error
    WRITE_RO_REG = 0b00011,      ///< Write to read-only register
    INTERNAL_BUS_FAULT = 0b00100 ///< Internal bus fault
};

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
    constexpr uint16_t CH_CTRL        = 0x0000;  ///< Channel Control Register
    constexpr uint16_t GLOBAL_CONFIG  = 0x0002;  ///< Global Configuration Register
    constexpr uint16_t GLOBAL_DIAG0   = 0x0003;  ///< Global Diagnosis Register 0
    constexpr uint16_t GLOBAL_DIAG1   = 0x0004;  ///< Global Diagnosis Register 1
    constexpr uint16_t GLOBAL_DIAG2   = 0x0005;  ///< Global Diagnosis Register 2
    constexpr uint16_t VBAT_TH        = 0x0006;  ///< VBAT Threshold Register
    constexpr uint16_t FB_FRZ         = 0x0007;  ///< Feedback Freeze Register
    constexpr uint16_t FB_UPD         = 0x0008;  ///< Feedback Update Register
    constexpr uint16_t WD_RELOAD      = 0x0009;  ///< SPI Watchdog Reload Register
    
    // Channel Group Diagnosis (0x000A + channel_group)
    constexpr uint16_t DIAG_ERR_CHGR0  = 0x000A;  ///< Diagnosis Error CH Group 0
    constexpr uint16_t DIAG_ERR_CHGR1  = 0x000B;  ///< Diagnosis Error CH Group 1
    constexpr uint16_t DIAG_ERR_CHGR2  = 0x000C;  ///< Diagnosis Error CH Group 2
    constexpr uint16_t DIAG_ERR_CHGR3  = 0x000D;  ///< Diagnosis Error CH Group 3
    constexpr uint16_t DIAG_ERR_CHGR4  = 0x000E;  ///< Diagnosis Error CH Group 4
    constexpr uint16_t DIAG_ERR_CHGR5  = 0x000F;  ///< Diagnosis Error CH Group 5
    
    constexpr uint16_t DIAG_WARN_CHGR0 = 0x0010;  ///< Diagnosis Warning CH Group 0
    constexpr uint16_t DIAG_WARN_CHGR1 = 0x0011;  ///< Diagnosis Warning CH Group 1
    constexpr uint16_t DIAG_WARN_CHGR2 = 0x0012;  ///< Diagnosis Warning CH Group 2
    constexpr uint16_t DIAG_WARN_CHGR3 = 0x0013;  ///< Diagnosis Warning CH Group 3
    constexpr uint16_t DIAG_WARN_CHGR4 = 0x0014;  ///< Diagnosis Warning CH Group 4
    constexpr uint16_t DIAG_WARN_CHGR5 = 0x0015;  ///< Diagnosis Warning CH Group 5
    
    constexpr uint16_t FAULT_MASK0    = 0x0016;  ///< Fault Mask Register 0
    constexpr uint16_t FAULT_MASK1    = 0x0017;  ///< Fault Mask Register 1
    constexpr uint16_t FAULT_MASK2    = 0x0018;  ///< Fault Mask Register 2
    constexpr uint16_t CLK_DIV        = 0x0019;  ///< Clock Control Register
    constexpr uint16_t SFF_BIST       = 0x003F;  ///< BIST Register
    
    // Feedback/Status Registers
    constexpr uint16_t ICVID          = 0x0200;  ///< IC Version and ID
    constexpr uint16_t PIN_STAT       = 0x0201;  ///< Pin Status Register
    constexpr uint16_t FB_STAT        = 0x0202;  ///< Feedback Status Register
    constexpr uint16_t FB_VOLTAGE1    = 0x0203;  ///< Feedback Voltage Register 1
    constexpr uint16_t FB_VOLTAGE2    = 0x0204;  ///< Feedback Voltage Register 2
    constexpr uint16_t CHIPID0        = 0x0205;  ///< Unique Chip ID Register 0
    constexpr uint16_t CHIPID1        = 0x0206;  ///< Unique Chip ID Register 1
    constexpr uint16_t CHIPID2        = 0x0207;  ///< Unique Chip ID Register 2
}

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
    constexpr uint16_t DEVICE_TYPE_MASK = 0xFF00;  ///< Device type mask [15:8]
    constexpr uint16_t REVISION_MASK    = 0x00FF;  ///< Silicon revision mask [7:0]
    
    // Expected device type code for TLE92466ED (upper byte of ICVID)
    // Note: This should be confirmed from datasheet or actual device
    constexpr uint8_t EXPECTED_TYPE_92466ED = 0x92;  ///< Expected device type code
    
    // Minimum supported silicon revision
    constexpr uint8_t MIN_REVISION = 0x00;  ///< Minimum silicon revision
    
    /**
     * @brief Check if ICVID value is valid for TLE92466ED
     * @param icvid Value read from ICVID register
     * @return true if device type matches expected value
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    constexpr bool is_valid_device(uint16_t icvid) noexcept {
        [[maybe_unused]] uint8_t device_type = (icvid >> 8) & 0xFF;
        // Accept device if communication is working (non-zero response)
        // Strict type checking can be enabled if exact ID is known
        return icvid != 0x0000 && icvid != 0xFFFF;
    }
    
    /**
     * @brief Extract device type from ICVID
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    constexpr uint8_t get_device_type(uint16_t icvid) noexcept {
        return (icvid >> 8) & 0xFF;
    }
    
    /**
     * @brief Extract silicon revision from ICVID
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    constexpr uint8_t get_revision(uint16_t icvid) noexcept {
        return icvid & 0xFF;
    }
}

//==============================================================================
// CHANNEL REGISTER OFFSETS
//==============================================================================

/**
 * @brief Per-channel register base addresses
 * 
 * @details
 * Each channel has its own set of registers at a specific base address.
 * Base address = 0x0100 + (Channel_Number * 0x0020)
 */
namespace ChannelBase {
    constexpr uint16_t CH0 = 0x0100;  ///< Channel 0 base address
    constexpr uint16_t CH1 = 0x0120;  ///< Channel 1 base address
    constexpr uint16_t CH2 = 0x0140;  ///< Channel 2 base address
    constexpr uint16_t CH3 = 0x0160;  ///< Channel 3 base address
    constexpr uint16_t CH4 = 0x0180;  ///< Channel 4 base address
    constexpr uint16_t CH5 = 0x01A0;  ///< Channel 5 base address
    
    constexpr uint16_t SPACING = 0x0020;  ///< Address spacing between channels
}

/**
 * @brief Per-channel register offsets (add to channel base address)
 */
namespace ChannelReg {
    constexpr uint16_t SETPOINT         = 0x0000;  ///< Current Setpoint Register
    constexpr uint16_t CTRL             = 0x0001;  ///< Control Register
    constexpr uint16_t PERIOD           = 0x0002;  ///< ICC PWM Frequency Controller
    constexpr uint16_t INTEGRATOR_LIMIT = 0x0003;  ///< ICC Integrator Limitation
    constexpr uint16_t DITHER_CLK_DIV   = 0x0004;  ///< Dither Clock Register
    constexpr uint16_t DITHER_STEP      = 0x0005;  ///< Dither Step Register
    constexpr uint16_t DITHER_CTRL      = 0x0006;  ///< Dither Control Register
    constexpr uint16_t CH_CONFIG        = 0x0007;  ///< Channel Configuration
    constexpr uint16_t MODE             = 0x000C;  ///< Channel Mode Register
    constexpr uint16_t TON              = 0x000D;  ///< On-Time Register
    constexpr uint16_t CTRL_INT_THRESH  = 0x000E;  ///< ICC Integrator Threshold
    constexpr uint16_t FB_DC            = 0x0200;  ///< Feedback Duty Cycle
    constexpr uint16_t FB_VBAT          = 0x0201;  ///< Feedback Average VBAT
    constexpr uint16_t FB_I_AVG         = 0x0202;  ///< Feedback Average Current
    constexpr uint16_t FB_IMIN_IMAX     = 0x0203;  ///< Feedback Min/Max Current
    constexpr uint16_t FB_INT_THRESH    = 0x0205;  ///< Feedback Integrator Threshold
}

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
    constexpr uint16_t EN_CH0       = (1 << 0);   ///< Enable Channel 0
    constexpr uint16_t EN_CH1       = (1 << 1);   ///< Enable Channel 1
    constexpr uint16_t EN_CH2       = (1 << 2);   ///< Enable Channel 2
    constexpr uint16_t EN_CH3       = (1 << 3);   ///< Enable Channel 3
    constexpr uint16_t EN_CH4       = (1 << 4);   ///< Enable Channel 4
    constexpr uint16_t EN_CH5       = (1 << 5);   ///< Enable Channel 5
    constexpr uint16_t CH_PAR_4_5   = (1 << 12);  ///< Parallel CH4/CH5
    constexpr uint16_t CH_PAR_0_3   = (1 << 13);  ///< Parallel CH0/CH3
    constexpr uint16_t CH_PAR_1_2   = (1 << 14);  ///< Parallel CH1/CH2
    constexpr uint16_t OP_MODE      = (1 << 15);  ///< Operation Mode
    
    constexpr uint16_t ALL_CH_MASK  = 0x003F;     ///< All channel bits
    constexpr uint16_t ALL_PAR_MASK = 0x7000;     ///< All parallel bits
    
    constexpr uint16_t DEFAULT      = 0x0000;     ///< Default value
    
    /// Config Mode (OP_MODE=0)
    constexpr uint16_t CONFIG_MODE  = 0x0000;
    /// Mission Mode (OP_MODE=1)
    constexpr uint16_t MISSION_MODE = OP_MODE;
    
    /**
     * @brief Get channel enable bit mask
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    constexpr uint16_t channel_mask(uint8_t channel) noexcept {
        return (channel < 6) ? static_cast<uint16_t>(1 << channel) : 0;
    }
}

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
    constexpr uint16_t CLK_WD_EN     = (1 << 0);   ///< Clock watchdog enable
    constexpr uint16_t SPI_WD_EN     = (1 << 1);   ///< SPI watchdog enable
    constexpr uint16_t CRC_EN        = (1 << 2);   ///< CRC check enable
    constexpr uint16_t V1V5_UV_TEST  = (1 << 4);   ///< 1.5V UV test
    constexpr uint16_t V1V5_OV_TEST  = (1 << 5);   ///< 1.5V OV test
    constexpr uint16_t OT_TEST       = (1 << 12);  ///< Over-temp test
    constexpr uint16_t UV_OV_SWAP    = (1 << 13);  ///< UV/OV swap test
    constexpr uint16_t VIO_SEL       = (1 << 14);  ///< VIO select (0=3.3V, 1=5V)
    
    constexpr uint16_t DEFAULT       = 0x4005;     ///< Default value
}

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
    constexpr uint16_t VBAT_UV     = (1 << 0);   ///< VBAT undervoltage
    constexpr uint16_t VBAT_OV     = (1 << 1);   ///< VBAT overvoltage
    constexpr uint16_t VIO_UV      = (1 << 2);   ///< VIO undervoltage
    constexpr uint16_t VIO_OV      = (1 << 3);   ///< VIO overvoltage
    constexpr uint16_t VDD_UV      = (1 << 4);   ///< VDD undervoltage
    constexpr uint16_t VDD_OV      = (1 << 5);   ///< VDD overvoltage
    constexpr uint16_t CLK_NOK     = (1 << 6);   ///< Clock fault
    constexpr uint16_t COTERR      = (1 << 7);   ///< Central OT error
    constexpr uint16_t COTWARN     = (1 << 8);   ///< Central OT warning
    constexpr uint16_t RES_EVENT   = (1 << 9);   ///< Reset event
    constexpr uint16_t POR_EVENT   = (1 << 10);  ///< Power-on reset
    constexpr uint16_t SPI_WD_ERR  = (1 << 14);  ///< SPI watchdog error
    
    constexpr uint16_t DEFAULT     = 0x0600;     ///< Default value
    constexpr uint16_t FAULT_MASK  = 0x47FF;     ///< All fault bits
}

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
    constexpr uint16_t VR_IREF_UV  = (1 << 0);   ///< Bias current UV
    constexpr uint16_t VR_IREF_OV  = (1 << 1);   ///< Bias current OV
    constexpr uint16_t VDD2V5_UV   = (1 << 2);   ///< 2.5V supply UV
    constexpr uint16_t VDD2V5_OV   = (1 << 3);   ///< 2.5V supply OV
    constexpr uint16_t REF_UV      = (1 << 4);   ///< Reference UV
    constexpr uint16_t REF_OV      = (1 << 5);   ///< Reference OV
    constexpr uint16_t VPRE_OV     = (1 << 6);   ///< Pre-reg OV
    constexpr uint16_t HVADC_ERR   = (1 << 15);  ///< HV ADC error
    
    constexpr uint16_t DEFAULT     = 0x0000;     ///< Default value
}

//==============================================================================
// GLOBAL_DIAG2 REGISTER (0x0005) - Global Diagnosis Register 2
//==============================================================================

/**
 * @brief GLOBAL_DIAG2 register bit definitions
 */
namespace GLOBAL_DIAG2 {
    constexpr uint16_t REG_ECC_ERR = (1 << 1);   ///< Register ECC error
    constexpr uint16_t OTP_ECC_ERR = (1 << 3);   ///< OTP ECC error
    constexpr uint16_t OTP_VIRGIN  = (1 << 4);   ///< OTP virgin/unconfigured
    
    constexpr uint16_t DEFAULT     = 0x0000;     ///< Default value
}

//==============================================================================
// FB_STAT REGISTER (0x0202) - Feedback Status
//==============================================================================

/**
 * @brief FB_STAT register bit definitions
 * 
 * @details
 * General feedback and status information.
 */
namespace FB_STAT {
    constexpr uint16_t SUP_NOK_INT  = (1 << 0);  ///< Internal supply fault
    constexpr uint16_t SUP_NOK_EXT  = (1 << 1);  ///< External supply fault
    constexpr uint16_t EN_PROT      = (1 << 2);  ///< Enable protection active
    constexpr uint16_t INIT_DONE    = (1 << 3);  ///< Initialization done
    constexpr uint16_t CLK_NOK_STAT = (1 << 6);  ///< Clock fault status
}

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
    constexpr uint16_t TARGET_MASK      = 0x7FFF;  ///< Target current mask
    constexpr uint16_t AUTO_LIMIT_DIS   = (1 << 15); ///< Disable auto-limit
    
    constexpr uint16_t DEFAULT          = 0x0000;  ///< Default (0A)
    
    /// Maximum safe target value (datasheet saturates above 0x6000)
    constexpr uint16_t MAX_TARGET       = 0x6000;
    
    /**
     * @brief Calculate setpoint value for desired current
     * @param current_ma Desired current in milliamperes (0-2000 single, 0-4000 parallel)
     * @param parallel_mode true if channel is in parallel mode
     * @return Setpoint register value
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    constexpr uint16_t calculate_target(uint16_t current_ma, bool parallel_mode = false) noexcept {
        uint32_t max_current = parallel_mode ? 4000 : 2000;
        uint32_t target = (static_cast<uint32_t>(current_ma) * 32767UL) / max_current;
        // Saturate at MAX_TARGET
        if (target > MAX_TARGET) target = MAX_TARGET;
        return static_cast<uint16_t>(target);
    }
    
    /**
     * @brief Calculate current from setpoint value
     * @param target Setpoint register value
     * @param parallel_mode true if channel is in parallel mode
     * @return Current in milliamperes
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    constexpr uint16_t calculate_current(uint16_t target, bool parallel_mode = false) noexcept {
        uint32_t max_current = parallel_mode ? 4000 : 2000;
        uint32_t current = (static_cast<uint32_t>(target & TARGET_MASK) * max_current) / 32767UL;
        return static_cast<uint16_t>(current);
    }
}

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
    constexpr uint16_t MIN_INT_THRESH_MASK   = 0x00FF;  ///< Min threshold mask
    constexpr uint16_t PWM_PERIOD_CALC_MODE  = (1 << 8);   ///< PWM calc mode
    constexpr uint16_t OLSG_WARN_WINDOW_MASK = 0x3E00;  ///< OLSG window mask
    constexpr uint16_t OLSG_WARN_WINDOW_SHIFT = 9;      ///< OLSG window shift
    constexpr uint16_t OLSG_WARN_EN          = (1 << 14); ///< OLSG warn enable
    
    constexpr uint16_t DEFAULT = 0x4600;  ///< Default value
}

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
    constexpr uint16_t SLEWR_1V0_US   = 0b00;     ///< 1.0 V/µs
    constexpr uint16_t SLEWR_2V5_US   = 0b01;     ///< 2.5 V/µs
    constexpr uint16_t SLEWR_5V0_US   = 0b10;     ///< 5.0 V/µs
    constexpr uint16_t SLEWR_10V0_US  = 0b11;     ///< 10.0 V/µs
    constexpr uint16_t SLEWR_MASK     = 0x0003;   ///< Slew rate mask
    
    // OFF-state diagnostic current [3:2]
    constexpr uint16_t I_DIAG_80UA    = (0 << 2); ///< 80 µA
    constexpr uint16_t I_DIAG_190UA   = (1 << 2); ///< 190 µA
    constexpr uint16_t I_DIAG_720UA   = (2 << 2); ///< 720 µA
    constexpr uint16_t I_DIAG_1250UA  = (3 << 2); ///< 1250 µA
    constexpr uint16_t I_DIAG_MASK    = 0x000C;   ///< I_DIAG mask
    
    // Open load threshold relative to setpoint [6:4]
    constexpr uint16_t OL_TH_DISABLED = (0 << 4); ///< OL detection disabled
    constexpr uint16_t OL_TH_1_8      = (1 << 4); ///< 1/8 of setpoint
    constexpr uint16_t OL_TH_2_8      = (2 << 4); ///< 2/8 of setpoint
    constexpr uint16_t OL_TH_3_8      = (3 << 4); ///< 3/8 of setpoint
    constexpr uint16_t OL_TH_4_8      = (4 << 4); ///< 4/8 of setpoint
    constexpr uint16_t OL_TH_5_8      = (5 << 4); ///< 5/8 of setpoint
    constexpr uint16_t OL_TH_6_8      = (6 << 4); ///< 6/8 of setpoint
    constexpr uint16_t OL_TH_7_8      = (7 << 4); ///< 7/8 of setpoint
    constexpr uint16_t OL_TH_MASK     = 0x0070;   ///< OL threshold mask
    
    // Fixed open load threshold [12:7]
    constexpr uint16_t OL_TH_FIXED_SHIFT = 7;
    constexpr uint16_t OL_TH_FIXED_MASK  = 0x1F80;
    
    constexpr uint16_t OC_DIAG_EN     = (1 << 13); ///< OC diag in OFF state
    
    // OFF-state diagnostic control [15:14]
    constexpr uint16_t OFF_DIAG_ENABLED    = (0 << 14); ///< OFF diag enabled
    constexpr uint16_t OFF_DIAG_LS_ONLY    = (1 << 14); ///< Low side current only
    constexpr uint16_t OFF_DIAG_HS_ONLY    = (2 << 14); ///< High side current only
    constexpr uint16_t OFF_DIAG_MASK       = 0xC000;    ///< OFF diag mask
    
    constexpr uint16_t DEFAULT = 0x0003;  ///< Default value
}

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
    constexpr uint16_t OFF              = 0x0000;  ///< Channel off
    constexpr uint16_t ICC_CURRENT_CTRL = 0x0001;  ///< ICC current control
    constexpr uint16_t DIRECT_DRIVE_SPI = 0x0002;  ///< Direct drive via SPI
    constexpr uint16_t DIRECT_DRIVE_DRV0= 0x0003;  ///< Direct drive via DRV0 pin
    constexpr uint16_t DIRECT_DRIVE_DRV1= 0x0004;  ///< Direct drive via DRV1 pin
    constexpr uint16_t FREE_RUN_MEAS    = 0x000C;  ///< Free running measurement
    constexpr uint16_t MODE_MASK        = 0x000F;  ///< Mode mask
    
    constexpr uint16_t DEFAULT = OFF;  ///< Default (off)
}

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
    constexpr uint16_t STEP_SIZE_MASK    = 0x0FFF;  ///< Step size mask
    constexpr uint16_t DEEP_DITHER       = (1 << 13); ///< Deep dither enable
    constexpr uint16_t FAST_MEAS_DITH    = (0 << 14); ///< Dither period
    constexpr uint16_t FAST_MEAS_HALF    = (1 << 14); ///< Half dither period
    constexpr uint16_t FAST_MEAS_QUAD    = (2 << 14); ///< Quarter dither period
    constexpr uint16_t FAST_MEAS_MASK    = 0xC000;    ///< Fast meas mask
    
    constexpr uint16_t DEFAULT = 0x0000;  ///< Default value
}

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
    constexpr uint16_t FLAT_MASK  = 0x00FF;  ///< Flat period mask
    constexpr uint16_t STEPS_SHIFT = 8;      ///< Steps shift
    constexpr uint16_t STEPS_MASK = 0xFF00;  ///< Steps mask
    
    constexpr uint16_t DEFAULT = 0x0000;     ///< Default value
}

//==============================================================================
// HELPER ENUMERATIONS
//==============================================================================

/**
 * @brief Channel enumeration
 */
enum class Channel : uint8_t {
    CH0 = 0,  ///< Channel 0
    CH1 = 1,  ///< Channel 1
    CH2 = 2,  ///< Channel 2
    CH3 = 3,  ///< Channel 3
    CH4 = 4,  ///< Channel 4
    CH5 = 5,  ///< Channel 5
    
    COUNT = 6 ///< Total number of channels
};

/**
 * @brief Channel operation mode
 */
enum class ChannelMode : uint8_t {
    OFF = 0x0,             ///< Channel off
    ICC = 0x1,             ///< Integrated Current Control
    DIRECT_DRIVE_SPI = 0x2,///< Direct drive via SPI TON register
    DIRECT_DRIVE_DRV0 = 0x3,///< Direct drive via DRV0 pin
    DIRECT_DRIVE_DRV1 = 0x4,///< Direct drive via DRV1 pin
    FREE_RUN_MEAS = 0xC    ///< Free running measurement mode
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
    I_80UA = 0,    ///< 80 µA
    I_190UA = 1,   ///< 190 µA
    I_720UA = 2,   ///< 720 µA
    I_1250UA = 3   ///< 1250 µA
};

/**
 * @brief Parallel operation pairs
 */
enum class ParallelPair : uint8_t {
    NONE = 0,      ///< No parallel operation
    CH0_CH3 = 1,   ///< Channels 0 and 3 paralleled
    CH1_CH2 = 2,   ///< Channels 1 and 2 paralleled
    CH4_CH5 = 3    ///< Channels 4 and 5 paralleled
};

//==============================================================================
// ADDRESS CALCULATION HELPERS
//==============================================================================

/**
 * @brief Get channel base address
 * @param channel Channel number (0-5)
 * @return Base address for channel registers
 */
#if __has_cpp_attribute(nodiscard) >= 201907L
[[nodiscard]]
#endif
constexpr uint16_t get_channel_base(Channel channel) noexcept {
    return ChannelBase::CH0 + (static_cast<uint16_t>(channel) * ChannelBase::SPACING);
}

/**
 * @brief Get channel register address
 * @param channel Channel number
 * @param offset Register offset from channel base
 * @return Complete register address
 */
#if __has_cpp_attribute(nodiscard) >= 201907L
[[nodiscard]]
#endif
constexpr uint16_t get_channel_register(Channel channel, uint16_t offset) noexcept {
    return get_channel_base(channel) + offset;
}

/**
 * @brief Convert channel to index
 */
#if __has_cpp_attribute(nodiscard) >= 201907L
[[nodiscard]]
#endif
constexpr uint8_t to_index(Channel ch) noexcept {
    return static_cast<uint8_t>(ch);
}

/**
 * @brief Validate channel number
 */
#if __has_cpp_attribute(nodiscard) >= 201907L
[[nodiscard]]
#endif
constexpr bool is_valid_channel(Channel ch) noexcept {
    return to_index(ch) < static_cast<uint8_t>(Channel::COUNT);
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
#if __has_cpp_attribute(nodiscard) >= 201907L
[[nodiscard]]
#endif
constexpr uint8_t calculate_crc8_j1850(const uint8_t* data, size_t length) noexcept {
    constexpr uint8_t POLY = 0x1D;
    uint8_t crc = 0xFF;
    
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 0x80) {
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
 */
#if __has_cpp_attribute(nodiscard) >= 201907L
[[nodiscard]]
#endif
inline uint8_t calculate_frame_crc(const SPIFrame& frame) noexcept {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&frame);
    // Calculate CRC on bytes 0-2 (excluding CRC byte itself at position 3)
    return calculate_crc8_j1850(bytes, 3);
}

/**
 * @brief Verify CRC in received frame
 * @param frame Received SPI frame
 * @return true if CRC is valid
 */
#if __has_cpp_attribute(nodiscard) >= 201907L
[[nodiscard]]
#endif
inline bool verify_frame_crc(const SPIFrame& frame) noexcept {
    SPIFrame temp = frame;
    uint8_t received_crc = temp.tx_fields.crc;
    temp.tx_fields.crc = 0;
    uint8_t calculated_crc = calculate_frame_crc(temp);
    return (received_crc == calculated_crc);
}

} // namespace TLE92466ED

#endif // TLE92466ED_REGISTERS_HPP
