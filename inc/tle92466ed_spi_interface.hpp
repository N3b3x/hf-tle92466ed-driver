/**
 * @file tle92466ed_spi_interface.hpp
 * @brief Communication Interface base class for TLE92466ED driver
 * @copyright Copyright (c) 2024-2025 HardFOC. All rights reserved.
 * 
 * @details
 * This file defines the hardware abstraction layer (CRTP `SpiInterface`) for the
 * TLE92466ED Six-Channel Low-Side Solenoid Driver IC. Platform adapters inherit
 * from `SpiInterface<Derived>` and implement the required transfer and GPIO hooks.
 *
 * **Pipelined SPI reads:** register reads are a two-frame protocol. Frame 1 sends
 * the read command; the chip latches the reply and presents it on MISO during
 * frame 2 (a dummy transfer of the *same* address). `Read()` and `Write()` issue
 * both frames through `TransferMulti()` so CS stays asserted and the bus stays
 * owned across the pair. The dummy is a read of address 0; repeating the
 * accessed address instead makes the part reply with a reserved reply-mode and
 * identity never validates (bench 2026-08-12).
 * On a shared multi-slave bus, an adapter that deasserts CS or releases the bus
 * between the two frames allows another device's transaction to consume the
 * pipeline slot — reads then return zeros or another slave's data.
 *
 * The TLE92466ED uses **32-bit SPI communication** with the following structure:
 * - MOSI: 32-bit frame (CRC[31:24] + Address[23:17] + R/W[16] + Data[15:0])
 * - MISO: 32-bit frame (CRC[31:24] + ReplyMode[23:22] + Status[21:17] + R/W[16] + Data[15:0])
 * - CS: Chip select (active low)
 * - SCLK: Serial clock (up to 10 MHz)
 * - CRC: SAE J1850 8-bit CRC
 *
 */
#pragma once
#include <cstdarg>
#include <cstdint>
#include <span>

#include "tle92466ed_expected.hpp"

namespace tle92466ed {

/**
 * @brief Error codes for communication interface operations
 *
 * This enumeration defines all possible error conditions that can occur
 * during hardware communication with the TLE92466ED IC.
 */
enum class CommError : uint8_t {
  None = 0,         ///< No error occurred
  BusError,         ///< SPI bus communication error
  Timeout,          ///< Operation timed out
  InvalidParameter, ///< Invalid parameter passed to function
  TransferError,    ///< Data transfer failed
  HardwareNotReady, ///< Hardware not initialized or ready
  BufferOverflow,   ///< Buffer size exceeded
  CRCError,         ///< CRC mismatch error
  /**
   * @brief MISO carried no frame at all (all-zero or all-ones word).
   *
   * Distinct from @c CRCError so a disconnected or unpowered device is not
   * reported as a corrupt one. Previously an empty word was returned as a
   * *successful* read of value 0, which every caller then decoded as real
   * register content.
   */
  NoReply,
  UnknownError ///< Unknown error occurred
};

/**
 * @brief Control pin enumeration for TLE92466ED
 *
 * These pins are used for device control and status monitoring.
 * The naming convention matches the standardized GPIO interface used
 * across all HardFOC drivers.
 */
enum class CtrlPin : uint8_t {
  RESN,  ///< Reset pin (active low) - Must be HIGH for device operation
  EN,    ///< Enable pin (active high) - Enables/disables output channels
  FAULTN ///< Fault pin (active low) - Indicates device fault condition
};

/**
 * @brief GPIO signal states with board-agnostic naming.
 *
 * Defines the logical active/inactive state of control pins.
 * The actual HIGH/LOW GPIO level depends on whether the pin is active-high or active-low.
 *
 * @note This represents the logical state (active/inactive), not the physical GPIO level.
 *       The CommInterface implementation handles the active-high vs active-low conversion.
 */
enum class GpioSignal : uint8_t {
  INACTIVE = 0, ///< Inactive signal state (logical inactive)
  ACTIVE = 1    ///< Active signal state (logical active)
};

/**
 * @brief Log severity levels for driver logging
 *
 * Defines the severity levels for logging messages from the driver.
 * Implementations should map these to their platform-specific logging systems.
 */
enum class LogLevel : uint8_t {
  Error = 0, ///< Error messages (highest severity)
  Warn,      ///< Warning messages
  Info,      ///< Informational messages
  Debug,     ///< Debug messages (lowest severity)
  Verbose    ///< Verbose messages (lowest severity, most detailed)
};

/**
 * @brief Result type for communication interface operations
 *
 * @tparam T The success type
 *
 * This provides a modern, safe way to return either a success value or an error.
 * Uses std::expected on C++23, or a lightweight polyfill on C++20.
 */
template <typename T>
using CommResult = tle::expected<T, CommError>;

/**
 * @brief SPI transaction configuration
 *
 * Defines the configuration parameters for SPI communication.
 * CPOL=0, CPHA=1 (SPI Mode 1) for TLE92466ED
 *
 * @details
 * From TLE92466ED datasheet:
 * "The falling edge of CSN indicates the beginning of a data access.
 * Data is sampled in on line SI at the falling edge of SCK and shifted out
 * on line SO at the rising edge of SCK. Each
 * access must be terminated by a rising edge of CSN."
 */
struct SPIConfig {
  uint32_t frequency{1'000'000}; ///< SPI clock frequency in Hz (max 10 MHz for TLE92466ED)
  uint8_t mode{1};               ///< SPI mode (CPOL=0, CPHA=1 for TLE92466ED)
  bool msb_first{true};          ///< MSB first transmission
  uint32_t timeout_ms{100};      ///< Transaction timeout in milliseconds
};

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
 *             |            |  1=W   |
 * @endverbatim
 *
 * MOSI (Read) Frame:
 * @verbatim
 *  Bits 31-24 | Bits 23-17 | Bit 16 | Bits 15-0
 * ------------+------------+--------+-----------
 *  CRC (8-bit)| Don't Care |  R/W   | Address (16-bit)
 *             |            |  0=R   |
 * @endverbatim
 *
 * Note: Full 16-bit address is placed in bits [15:0] for read operations.
 *
 * MISO (Reply) Frame - 16-bit Reply (Reply Mode = 00B):
 * @verbatim
 *  Bits 31-24 | Bits 23-22 | Bits 21-17 | Bit 16 | Bits 15-0
 * ------------+------------+------------+--------+-----------
 *  CRC (8-bit)| Reply Mode | Status (5) | R/W    | Data (16)
 *             |    00B     |            | Echo   |
 * @endverbatim
 *
 * MISO (Reply) Frame - 22-bit Reply (Reply Mode = 01B):
 * @verbatim
 *  Bits 31-24 | Bits 23-22 | Bits 21-0
 * ------------+------------+-----------
 *  CRC (8-bit)| Reply Mode | Data (22-bit)
 *             |    01B     |
 * @endverbatim
 *
 * MISO (Reply) Frame - Critical Fault (Reply Mode = 10B):
 * @verbatim
 *  Bits 31-24 | Bits 23-22 | Bits 21-8 | Bits 7-0
 * ------------+------------+----------+----------
 *  Don't Care | Reply Mode | Don't    | Fault
 *             |    10B     | Care     | Flags
 * @endverbatim
 *
 * Fault Flags (bits 7:0):
 * - Bit 7: 1V5 supply (1=OK, 0=NOT OK)
 * - Bit 6: 2V5 supply (1=OK, 0=NOT OK)
 * - Bit 5: ADC Bandgap (1=OK, 0=NOT OK)
 * - Bit 4: CLK_TOO_SLOW (1=YES, 0=NO)
 * - Bit 3: CLK_TOO_FAST (1=YES, 0=NO)
 * - Bit 2: DIG_CLK_TOO_SLOW (1=YES, 0=NO)
 * - Bit 1: DIG_CLK_TOO_FAST (1=YES, 0=NO)
 * - Bit 0: WD_REF_CLK (1=MISSING, 0=OK)
 */
union SPIFrame {
  uint32_t word; ///< Complete 32-bit frame

  /// MOSI (Transmit) frame structure
  struct {
    uint32_t data : 16;   ///< Data field [15:0]
    uint32_t rw : 1;      ///< Read/Write bit [16] (1=Write, 0=Read)
    uint32_t address : 7; ///< Register address [23:17]
    uint32_t crc : 8;     ///< CRC-8 SAE J1850 [31:24]
  } tx_fields;

  /// MISO (Receive) frame structure - common fields
  struct {
    uint32_t _data_low : 16;   ///< Lower data bits [15:0] (interpretation depends on reply mode)
    uint32_t _field_16 : 1;    ///< Field at bit 16 (interpretation depends on reply mode)
    uint32_t _field_17_21 : 5; ///< Fields at bits 21:17 (interpretation depends on reply mode)
    uint32_t reply_mode : 2;   ///< Reply mode [23:22] - determines frame type
    uint32_t crc : 8;          ///< CRC-8 SAE J1850 [31:24]
  } rx_common;

  /// MISO 16-bit Reply Frame (Reply Mode = 00B)
  struct {
    uint32_t data : 16;      ///< Data field [15:0]
    uint32_t rw_echo : 1;    ///< R/W bit echoed [16]
    uint32_t status : 5;     ///< Status bits [21:17]
    uint32_t reply_mode : 2; ///< Reply mode [23:22] = 00B
    uint32_t crc : 8;        ///< CRC-8 SAE J1850 [31:24]
  } rx_16bit;

  /// MISO 22-bit Reply Frame (Reply Mode = 01B)
  struct {
    uint32_t data : 22;      ///< 22-bit data [21:0]
    uint32_t reply_mode : 2; ///< Reply mode [23:22] = 01B
    uint32_t crc : 8;        ///< CRC-8 SAE J1850 [31:24]
  } rx_22bit;

  /// MISO Critical Fault Frame (Reply Mode = 10B)
  struct {
    uint32_t fault_flags : 8; ///< Fault flags [7:0]
    uint32_t _reserved : 8;   ///< Reserved/Don't care [15:8]
    uint32_t _reserved2 : 1;  ///< Reserved/Don't care [16]
    uint32_t _reserved3 : 5;  ///< Reserved/Don't care [21:17]
    uint32_t reply_mode : 2;  ///< Reply mode [23:22] = 10B
    uint32_t _dont_care : 8;  ///< Don't care [31:24] (not CRC in fault frames)
  } rx_fault;

  /**
   * @brief Construct read frame (without CRC - must be calculated separately)
   * @param addr Register address (10-bit actual address, 0x000-0x3FF)
   * @return SPIFrame configured for read operation (CRC = 0)
   *
   * @details
   * Read frame format per datasheet:
   * @verbatim
   *  Bits 31-24 | Bits 23-17 | Bit 16 | Bits 15-0
   * ------------+------------+--------+-----------
   *  CRC (8-bit)| Don't Care |  R/W   | Address (7-bit)
   *             |            |  0=R   |
   * @endverbatim
   *
   * Note: read frames carry the FULL 16-bit address in bits [15:0]; bits
   * [23:17] are don't-care (datasheet §5.2.3.2). This is deliberately
   * different from the write frame, which has only a 7-bit address field.
   * That asymmetry is why read-only status registers at 0x0200+ are
   * reachable at all — they cannot be expressed in a write frame.
   */
  [[nodiscard]] static constexpr SPIFrame MakeRead(uint16_t addr) noexcept {
    SPIFrame frame{};
    frame.tx_fields.rw = 0;                 // Read (bit 16)
    frame.tx_fields.address = 0;            // Bits 23:17 are don't care for reads
    frame.tx_fields.data = (addr & 0xFFFF); // 16-bit address in bits 15:0 (full 16-bit address)
    frame.tx_fields.crc = 0;                // CRC calculated separately
    return frame;
  }

  /**
   * @brief Construct write frame (without CRC - must be calculated separately)
   * @param addr Register address (10-bit actual address, 0x000-0x3FF)
   * @param data Data word to write (16-bit)
   * @return SPIFrame configured for write operation (CRC = 0)
   *
   * @details
   * Write frame format per datasheet:
   * @verbatim
   *  Bits 31-24 | Bits 23-17 | Bit 16 | Bits 15-0
   * ------------+------------+--------+-----------
   *  CRC (8-bit)| Address(7) |  R/W   | Data (16)
   *             |            |  1=W   |
   * @endverbatim
   *
   * Note: the write address field is only 7 bits wide (datasheet §5.2.3.1),
   * so `addr & 0x7F` is not a truncation — every writable register on this
   * device lives at 0x000..0x07F (central config 0x00..0x3F, channel config
   * CH4 0x20.. through CH3 0x70..0x7E). Registers at 0x0200+ are read-only
   * by construction: they cannot be addressed by a write frame at all.
   */
  [[nodiscard]] static constexpr SPIFrame MakeWrite(uint16_t addr, uint16_t data) noexcept {
    SPIFrame frame{};
    frame.tx_fields.rw = 1;                  // Write (bit 16)
    frame.tx_fields.address = (addr) & 0x7F; // Lower 7 bits [23:17]
    frame.tx_fields.data = data;             // Data in bits [15:0]
    frame.tx_fields.crc = 0;                 // CRC calculated separately
    return frame;
  }
};

static_assert(sizeof(SPIFrame) == 4, "SPIFrame must be exactly 4 bytes");

/**
 * @brief Helper structure for critical fault frame flags
 *
 * @details
 * Critical fault flags from bits 7:0 of critical fault reply frame.
 * Per datasheet:
 * - Bit 7: 1V5 supply (1B=OK, 0B=NOT OK)
 * - Bit 6: 2V5 supply (1B=OK, 0B=NOT OK)
 * - Bit 5: ADC Bandgap (1B=OK, 0B=NOT OK)
 * - Bit 4: CLK_TOO_SLOW (1B=YES, 0B=NO)
 * - Bit 3: CLK_TOO_FAST (1B=YES, 0B=NO)
 * - Bit 2: DIG_CLK_TOO_SLOW (1B=YES, 0B=NO)
 * - Bit 1: DIG_CLK_TOO_FAST (1B=YES, 0B=NO)
 * - Bit 0: WD_REF_CLK (1B=MISSING, 0B=OK)
 */
struct CriticalFaultFlags {
  bool wd_ref_clk_missing : 1; ///< Bit 0: Clock watchdog reference clock missing (1=MISSING, 0=OK)
  bool dig_clk_too_fast : 1;   ///< Bit 1: Digital clock too fast (1=YES, 0=NO)
  bool dig_clk_too_slow : 1;   ///< Bit 2: Digital clock too slow (1=YES, 0=NO)
  bool clk_too_fast : 1;       ///< Bit 3: Clock too fast (1=YES, 0=NO)
  bool clk_too_slow : 1;       ///< Bit 4: Clock too slow (1=YES, 0=NO)
  bool adc_bandgap_ok : 1;     ///< Bit 5: ADC Bandgap OK (1=OK, 0=NOT OK)
  bool supply_2v5_ok : 1;      ///< Bit 6: 2V5 supply OK (1=OK, 0=NOT OK)
  bool supply_1v5_ok : 1;      ///< Bit 7: 1V5 supply OK (1=OK, 0=NOT OK)

  /**
   * @brief Extract fault flags from critical fault frame
   * @param frame Critical fault frame (reply_mode must be 10B)
   * @return CriticalFaultFlags structure
   */
  [[nodiscard]] static constexpr CriticalFaultFlags Extract(const SPIFrame& frame) noexcept {
    CriticalFaultFlags flags{};
    uint8_t fault_byte = frame.rx_fault.fault_flags;
    flags.wd_ref_clk_missing = (fault_byte & 0x01) != 0;
    flags.dig_clk_too_fast = (fault_byte & 0x02) != 0;
    flags.dig_clk_too_slow = (fault_byte & 0x04) != 0;
    flags.clk_too_fast = (fault_byte & 0x08) != 0;
    flags.clk_too_slow = (fault_byte & 0x10) != 0;
    flags.adc_bandgap_ok = (fault_byte & 0x20) != 0;
    flags.supply_2v5_ok = (fault_byte & 0x40) != 0;
    flags.supply_1v5_ok = (fault_byte & 0x80) != 0;
    return flags;
  }
};

/**
 * @brief SPI Reply Mode enumeration
 */
enum class ReplyMode : uint8_t {
  REPLY_16BIT = 0b00,    ///< 16-bit reply frame
  REPLY_22BIT = 0b01,    ///< 22-bit reply frame (extended data)
  CRITICAL_FAULT = 0b10, ///< Critical fault frame
  RESERVED = 0b11        ///< Reserved
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

/**
 * @brief CRTP-based Communication Interface template class
 *
 * @details
 * This template class provides a hardware-agnostic interface for SPI communication
 * using the CRTP (Curiously Recurring Template Pattern) for compile-time polymorphism.
 * Platform-specific implementations should inherit from this template with themselves
 * as the template parameter.
 *
 * Benefits of CRTP:
 * - Compile-time polymorphism (no virtual function overhead)
 * - Static dispatch instead of dynamic dispatch
 * - Better optimization opportunities for the compiler
 *
 * The CommInterface uses modern C++20/23 features including:
 * - Concepts for compile-time constraints
 * - std::span for safe array access
 * - std::expected for error handling
 * - uint32_t for time management (microseconds)
 *
 * @par 32-Bit SPI Communication:
 * The TLE92466ED requires 32-bit SPI frames. Implementations must:
 * - Transfer 4 bytes (32 bits) per transaction
 * - Maintain MSB-first byte order
 * - Support full-duplex operation
 * - Calculate and verify CRC-8 (SAE J1850)
 *
 * @par Example Implementation:
 * @code{.cpp}
 * class MyPlatformCommInterface : public tle92466ed::SpiInterface<MyPlatformCommInterface> {
 * public:
 *     CommResult<uint32_t> Transfer32(uint32_t data) noexcept {
 *         uint32_t result = spi_transfer_32bit(data);
 *         if (spi_error()) {
 *             return tle::unexpected(CommError::TransferError);
 *         }
 *         return result;
 *     }
 *     // ... implement other required methods
 * };
 * @endcode
 *
 * @par Thread Safety:
 * Implementations must ensure thread-safety for multi-threaded environments.
 *
 * @par Hardware Requirements:
 * - SPI peripheral capable of 32-bit transfers (or 4x 8-bit)
 * - Minimum frequency: 100 kHz
 * - Maximum frequency: 10 MHz
 * - Support for SPI Mode 1 (CPOL=0, CPHA=1)
 * - CRC calculation capability (hardware or software)
 *
 * @tparam Derived The derived class type (CRTP pattern)
 */
template <typename Derived>
class SpiInterface {
public:

  /**
   * @brief Initialize the hardware interface
   *
   * @details
   * This function should initialize the SPI peripheral, configure GPIO pins,
   * and prepare the hardware for communication. It should be called before
   * any other CommInterface functions.
   *
   * @return CommResult<void> Success or error code
   * @retval CommError::None Initialization successful
   * @retval CommError::HardwareNotReady Hardware initialization failed
   * @retval CommError::InvalidParameter Invalid configuration
   */
  [[nodiscard]] CommResult<void> Init() noexcept {
    return static_cast<Derived*>(this)->Init();
  }

  /**
   * @brief Deinitialize the hardware interface
   *
   * @details
   * Releases hardware resources and disables the SPI peripheral. Should be
   * called when the driver is no longer needed.
   *
   * @return CommResult<void> Success or error code
   */
  [[nodiscard]] CommResult<void> Deinit() noexcept {
    return static_cast<Derived*>(this)->Deinit();
  }

  /**
   * @brief Transfer 32-bit data via SPI (full-duplex)
   *
   * @details
   * Performs a full-duplex SPI transaction, simultaneously sending and
   * receiving 32 bits of data. This is the primary communication method
   * for the TLE92466ED.
   *
   * The TLE92466ED requires 32-bit SPI frames with the following format:
   * - Bits [31:24]: CRC-8 (SAE J1850)
   * - Bits [23:17]: Register address (7 bits of 10-bit address)
   * - Bit [16]: R/W (1=Write, 0=Read)
   * - Bits [15:0]: Data (16 bits)
   *
   * @param[in] tx_data The 32-bit data to transmit
   * @return CommResult<uint32_t> Received 32-bit data or error
   * @retval CommError::TransferError SPI transfer failed
   * @retval CommError::Timeout Transfer timeout
   *
   * @par Timing Requirements:
   * - CS must be asserted (low) before transfer and deasserted (high) after
   * - CS must be held low during entire 32-bit transfer
   * - Minimum CS inactive time between transfers: 100ns
   * - Data sampled on rising edge (CPHA=1)
   *
   * @note Chip select (CS) management must be handled internally by this function.
   *       The implementation should assert CS before the transfer and deassert it after.
   *       CRC calculation is handled by the driver layer, not CommInterface.
   */
  [[nodiscard]] CommResult<uint32_t> Transfer32(uint32_t tx_data) noexcept {
    return static_cast<Derived*>(this)->Transfer32(tx_data);
  }

  /**
   * @brief Transfer multiple 32-bit words via SPI in one CS assertion window
   *
   * @details
   * Performs consecutive full-duplex 32-bit transfers while CS remains
   * asserted. The driver relies on this for pipelined register access: each
   * `Read()` / `Write()` passes exactly two words (command frame, then dummy
   * frame that clocks out the latched reply).
   *
   * @param[in] tx_data Span of transmit data (32-bit words)
   * @param[out] rx_data Span to store received data (32-bit words)
   * @return CommResult<void> Success or error code
   * @retval CommError::InvalidParameter Buffer size mismatch
   * @retval CommError::TransferError Transfer failed
   *
   * @pre tx_data.size() == rx_data.size()
   * @pre Both spans must be valid for the duration of the transfer
   *
   * @warning **Platform requirement:** on shared SPI buses, the adapter must
   *          keep CS asserted and must not yield bus ownership between the
   *          command and dummy frames of a pipelined read. Releasing the bus
   *          mid-pair is a common cause of all-zero or cross-slave corrupted
   *          register values (including inflated feedback currents).
   */
  [[nodiscard]] CommResult<void> TransferMulti(std::span<const uint32_t> tx_data,
                                               std::span<uint32_t> rx_data) noexcept {
    return static_cast<Derived*>(this)->TransferMulti(tx_data, rx_data);
  }

  /**
   * @brief Delay for specified duration
   *
   * @details
   * Provides a hardware-specific delay implementation. Required for timing
   * constraints such as reset pulse width and power-up delays.
   *
   * @param[in] microseconds Duration to delay in microseconds
   * @return CommResult<void> Success or error code
   *
   * @par Timing Requirements:
   * - Reset pulse width: minimum 1µs
   * - Power-up delay: minimum 1ms
   */
  [[nodiscard]] CommResult<void> Delay(uint32_t microseconds) noexcept {
    return static_cast<Derived*>(this)->Delay(microseconds);
  }

  /**
   * @brief Configure SPI parameters
   *
   * @details
   * Updates the SPI configuration. Can be called at runtime to adjust
   * communication parameters.
   *
   * @param[in] config New SPI configuration
   * @return CommResult<void> Success or error code
   * @retval CommError::InvalidParameter Invalid configuration
   *
   * @par TLE92466ED SPI Requirements:
   * - Frequency: 100 kHz - 10 MHz
   * - Mode: 1 (CPOL=0, CPHA=1) - Data sampled on falling edge of SCK, shifted on rising edge
   * - Bit order: MSB first
   * - Frame size: 32 bits (4 bytes)
   */
  [[nodiscard]] CommResult<void> Configure(const SPIConfig& config) noexcept {
    return static_cast<Derived*>(this)->Configure(config);
  }

  /**
   * @brief Check if hardware is ready for communication
   *
   * @details
   * Verifies that the hardware interface is initialized and ready for
   * SPI transactions.
   *
   * @return true if ready, false otherwise
   */
  [[nodiscard]] bool IsReady() const noexcept {
    return static_cast<const Derived*>(this)->IsReady();
  }

  /**
   * @brief Get the last error that occurred
   *
   * @details
   * Retrieves the most recent error code. Useful for debugging and
   * error recovery.
   *
   * @return CommError The last error code
   */
  [[nodiscard]] CommError GetLastError() const noexcept {
    return static_cast<const Derived*>(this)->GetLastError();
  }

  /**
   * @brief Clear any pending errors
   *
   * @details
   * Resets the error state. Should be called after handling an error
   * condition and before retrying operations.
   *
   * @return CommResult<void> Success or error code
   */
  [[nodiscard]] CommResult<void> ClearErrors() noexcept {
    return static_cast<Derived*>(this)->ClearErrors();
  }

  /**
   * @brief Set GPIO control pin signal state (output control).
   *
   * @details
   * Controls the state of TLE92466ED control pins (RESN, EN).
   * The implementation must map ACTIVE/INACTIVE to the appropriate physical
   * levels based on board design.
   *
   * @param pin Control pin to set (RESN or EN)
   * @param signal The desired signal state (ACTIVE or INACTIVE)
   * @return CommResult<void> Success or error code
   * @retval CommError::InvalidParameter Invalid pin or signal
   * @retval CommError::HardwareNotReady Hardware not initialized
   *
   * @par Pin Behavior:
   * - RESN (active low): ACTIVE = not in reset (GPIO HIGH), INACTIVE = in reset (GPIO LOW)
   * - EN (active high): ACTIVE = enabled (GPIO HIGH), INACTIVE = disabled (GPIO LOW)
   *
   * @note RESN must be ACTIVE for SPI communication to work.
   * @note EN only affects output channels, not SPI communication.
   */
  [[nodiscard]] CommResult<void> GpioSet(CtrlPin pin, GpioSignal signal) noexcept {
    return static_cast<Derived*>(this)->GpioSet(pin, signal);
  }

  /**
   * @brief Read GPIO control pin signal state (input state).
   *
   * @details
   * Reads the current state of TLE92466ED control pins (FAULTN).
   * The implementation must map physical levels to ACTIVE/INACTIVE based
   * on board design.
   *
   * @param pin Control pin to read (FAULTN)
   * @return CommResult<GpioSignal> Pin signal state (ACTIVE or INACTIVE) or error
   * @retval CommError::InvalidParameter Invalid pin (only FAULTN can be read)
   * @retval CommError::HardwareNotReady Hardware not initialized
   *
   * @par Pin Behavior:
   * - FAULTN (active low): ACTIVE = fault detected (GPIO LOW), INACTIVE = no fault (GPIO HIGH)
   *
   * @note Only FAULTN can be read. RESN and EN are output-only.
   */
  [[nodiscard]] CommResult<GpioSignal> GpioRead(CtrlPin pin) noexcept {
    return static_cast<Derived*>(this)->GpioRead(pin);
  }

  /**
   * @brief Set GPIO pin to active state (convenience method).
   * @param pin The control pin to set active
   * @return CommResult<void> Success or error code
   */
  [[nodiscard]] CommResult<void> GpioSetActive(CtrlPin pin) noexcept {
    return GpioSet(pin, GpioSignal::ACTIVE);
  }

  /**
   * @brief Set GPIO pin to inactive state (convenience method).
   * @param pin The control pin to set inactive
   * @return CommResult<void> Success or error code
   */
  [[nodiscard]] CommResult<void> GpioSetInactive(CtrlPin pin) noexcept {
    return GpioSet(pin, GpioSignal::INACTIVE);
  }

  /**
   * @brief Log a message with specified severity level and tag
   *
   * @details
   * Platform-specific logging implementation. The driver uses this to log
   * diagnostic information, errors, warnings, and debug messages.
   *
   * @param level Log severity level
   * @param tag Tag/component name for the log message (e.g., "TLE92466ED")
   * @param format Format string (printf-style)
   * @param ... Variable arguments for format string
   *
   * @note Implementations should use platform-specific logging (e.g., ESP_LOG for ESP32)
   * @note The format string and arguments follow printf-style formatting
   */
  void Log(LogLevel level, const char* tag, const char* format, ...) noexcept {
    va_list args{};
    va_start(args, format);
    static_cast<Derived*>(this)->Log(level, tag, format, args);
    va_end(args);
  }

  /**
   * @brief Read a register from the TLE92466ED (High-Level API)
   *
   * @param address Register address (10-bit, 0x000-0x3FF)
   * @param verify_crc If true, verify CRC in response (default: true)
   * @return CommResult<uint32_t> Register value (16-bit or 22-bit depending on reply mode) or error
   *
   * @details
   * This function handles the complete read operation:
   * - Constructs the read frame with address
   * - Calculates and adds CRC
   * - Performs first SPI transfer (sends command, receives dummy data)
   * - Performs second SPI transfer (sends dummy command, receives actual response)
   * - Parses the response based on reply mode
   * - Verifies CRC if requested
   *
   * @note **Pipelined reply:** frame 1 transmits the read command (MISO is not
   *       yet valid for that address); frame 2 is a dummy read that clocks out
   *       the latched reply. Both frames are sent via `TransferMulti()` in one
   *       CS window — adapters must not split them on shared buses.
   *
   * @note **Replies are validated, not trusted.** A reply is accepted only if
   *       its CRC checks out *and* its reply mode matches the width the
   *       register is defined to answer with (@ref ExpectedReplyFor). Reads
   *       are retried until one qualifies. An unreadable register therefore
   *       returns an error instead of a plausible-looking value.
   *
   * @warning @p verify_crc no longer disables validation; it is kept only for
   *          API compatibility. Callers that passed `false` were the reason
   *          corrupt and stale frames were decoded as register content.
   *
   * @retval CommError::CRCError Frames arrived but none were well formed.
   * @retval CommError::NoReply  MISO was idle for every attempt.
   *
   * @note Frame construction and CRC calculation are handled automatically.
   *       External code should typically use the Driver API; this method is
   *       available for advanced use cases.
   */
  [[nodiscard]] CommResult<uint32_t> Read(uint16_t address, bool verify_crc = true) noexcept;

  /** @brief Largest burst @ref ReadMulti will issue in one CS-session chain. */
  static constexpr size_t kMaxPipelinedReads = 12;

  /**
   * @brief Read several registers in ONE pipelined chain.
   *
   * @param[in]  addresses Register addresses, at most @ref kMaxPipelinedReads.
   * @param[out] values    Decoded register contents, same length as @p addresses.
   *                       Entries whose reply failed validation are left as 0.
   * @param[out] valid_mask Bit @c i set when @p values[i] is trustworthy.
   *                       Never null-checked away: callers must inspect it.
   * @return Success when the chain transferred; individual replies may still
   *         have been rejected, which @p valid_mask reports.
   *
   * @details
   * The device answers each command on the NEXT CS window, so a burst of N
   * reads needs N+2 frames — one leading dummy to absorb whatever reply was
   * outstanding, the N commands, and one trailing dummy to clock out the last
   * reply. Reply @c i therefore lands in frame @c i+2.
   *
   * That is the whole point of this call: @ref Read spends 3 frames per
   * register because it opens and closes the pipeline every time, so N reads
   * cost 3N frames. Here they cost N+2 — for the six-channel FB_DC/FB_I_AVG
   * sweep that is 14 frames instead of 36, inside one bus-ownership window.
   *
   * Replies are validated exactly as in @ref Read (CRC plus the reply width
   * @ref ExpectedReplyFor defines for that address). No retry is attempted:
   * re-running the whole burst to recover one bad frame costs more than the
   * caller re-reading that single register with @ref Read, and a desynchronised
   * chain fails validation wholesale rather than returning plausible garbage.
   *
   * @warning The adapter's @c TransferMulti must keep one bus-ownership window
   *          across the entire chain. An adapter that falls back to per-frame
   *          transfers breaks the pipeline; the reply validation will reject
   *          the results rather than let them through.
   */
  [[nodiscard]] CommResult<void> ReadMulti(std::span<const uint16_t> addresses,
                                           std::span<uint32_t> values,
                                           uint16_t& valid_mask) noexcept;

  /**
   * @brief Write a register to the TLE92466ED (High-Level API)
   *
   * @param address Register address (10-bit, 0x000-0x3FF)
   * @param value Data value to write (16-bit)
   * @param verify_crc If true, verify CRC in response (default: true)
   * @return CommResult<void> Success or error
   *
   * @details
   * This function handles the complete write operation:
   * - Constructs the write frame with address and data
   * - Calculates and adds CRC
   * - Performs first SPI transfer (sends command, receives dummy data)
   * - Performs second SPI transfer (sends dummy command, receives actual response)
   * - Verifies CRC if requested
   *
   * @note Writes use the same two-frame pipeline as reads: frame 1 sends the
   *       write command; frame 2 clocks out the status reply. Both frames must
   *       stay in one `TransferMulti()` CS window on shared buses.
   *
   * @note Frame construction and CRC calculation are handled automatically.
   */
  [[nodiscard]] CommResult<void> Write(uint16_t address, uint16_t value,
                                       bool verify_crc = true) noexcept;

  /**
   * @brief Fault byte from the most recent critical-fault reply frame.
   *
   * @details
   * A reply with `reply_mode == 10B` is the device reporting that its own
   * supplies/clocks are unusable; the payload byte says which (see
   * @ref CriticalFaultFlags). `Read()` / `Write()` surface that as
   * `CommError::BusError`, which on its own is indistinguishable from a bus
   * glitch — this accessor preserves *why* the device refused to answer.
   *
   * @return Last fault byte, or 0 if no critical-fault frame has been seen.
   * @note Zero is also a meaningful fault byte (all supply-OK bits clear);
   *       pair with @ref SawCriticalFault.
   */
  [[nodiscard]] uint8_t LastCriticalFaultFlags() const noexcept {
    return last_critical_fault_flags_;
  }

  /** @brief True once a critical-fault reply frame has been decoded. */
  [[nodiscard]] bool SawCriticalFault() const noexcept {
    return saw_critical_fault_;
  }

  /**
   * @brief Prevent copying
   */
  SpiInterface(const SpiInterface&) = delete;
  SpiInterface& operator=(const SpiInterface&) = delete;

protected:
  /**
   * @brief Protected constructor to prevent direct instantiation
   *
   * @details
   * This class can only be instantiated through derived classes.
   */
  SpiInterface() = default;

  /**
   * @brief Allow moving
   */
  SpiInterface(SpiInterface&&) noexcept = default;
  SpiInterface& operator=(SpiInterface&&) noexcept = default;

  /** @brief Latch a critical-fault reply so callers can report the cause. */
  void NoteCriticalFault(const SPIFrame& frame) noexcept {
    last_critical_fault_flags_ = static_cast<uint8_t>(frame.rx_fault.fault_flags);
    saw_critical_fault_ = true;
  }

  uint8_t last_critical_fault_flags_{0}; ///< @see LastCriticalFaultFlags
  bool saw_critical_fault_{false};       ///< @see SawCriticalFault

  /**
   * @brief Protected destructor
   * @note Derived classes can have public destructors
   */
  ~SpiInterface() = default;
};

#if __cpp_concepts >= 201907L
#include <concepts>
/**
 * @brief Concept to verify a type implements the CommInterface interface
 *
 * @tparam T Type to check
 *
 * @details
 * This C++20 concept ensures at compile-time that a class properly
 * implements the CommInterface interface. Provides better error messages than
 * traditional template constraints.
 *
 * @note Requires C++20 concepts support. Guarded by __cpp_concepts.
 */
template <typename T>
concept SpiInterfaceLike =
    requires(T comm, uint32_t data, SPIConfig cfg, CtrlPin pin, GpioSignal signal) {
      { comm.Init() } -> std::same_as<CommResult<void>>;
      { comm.Transfer32(data) } -> std::same_as<CommResult<uint32_t>>;
      { comm.IsReady() } -> std::same_as<bool>;
      { comm.Configure(cfg) } -> std::same_as<CommResult<void>>;
      { comm.GpioSet(pin, signal) } -> std::same_as<CommResult<void>>;
      { comm.GpioRead(pin) } -> std::same_as<CommResult<GpioSignal>>;
    };
#endif // __cpp_concepts

} // namespace tle92466ed

// Include registers header for CRC functions (after SPIFrame is defined)
#include "tle92466ed_registers.hpp"

namespace tle92466ed {

//==============================================================================
// INLINE IMPLEMENTATIONS
//==============================================================================

template <typename Derived>
inline CommResult<uint32_t> SpiInterface<Derived>::Read(uint16_t address, bool verify_crc) noexcept {
  // Create read frame
  SPIFrame tx_frame = SPIFrame::MakeRead(address);

  // Calculate and set CRC
  tx_frame.tx_fields.crc = CalculateFrameCrc(tx_frame);

  /* Pipelined SPI: the reply to a command only appears on the NEXT CS window,
   * so the device always has one reply outstanding. That makes a bare
   * command+dummy pair depend on what the previous transaction left behind —
   * one dropped or extra frame anywhere shifts every later read by one slot,
   * and the caller silently gets the *previous* register's value (bench
   * 2026-08-12: PIN_STAT returning CH_CTRL's 0x8000, FB_VOLTAGE2 returning 0).
   *
   * Sending a leading dummy makes the transaction self-synchronising: it
   * absorbs whatever reply was outstanding, so the reply to `tx_frame` is
   * always in the last slot regardless of prior bus state.
   *
   * The dummies address 0, NOT `address`. Re-sending the same read as the
   * dummy queues a second reply and the part answers with a reserved
   * reply-mode instead of the register (bench 2026-08-12: ICVID never
   * validated, DriverError::SPIFrameError every attempt; MakeRead(0) restored
   * identity on the same hardware). TransferMulti keeps one bus-ownership
   * window across all three frames on shared multi-slave buses. */
  SPIFrame dummy_frame = SPIFrame::MakeRead(0);
  dummy_frame.tx_fields.crc = CalculateFrameCrc(dummy_frame);

  const uint32_t tx_words[3] = {dummy_frame.word, tx_frame.word,
                                dummy_frame.word};

  /* A reply is only believed when it is well formed: CRC valid *and* the reply
   * width the register is defined to answer with. Bench measurement (Portenta
   * Mid, 2026-08-13, 150 frames) found 4 % of frames corrupt and 25 % of
   * FB_STAT reads carrying the preceding ICVID reply — the latter with a valid
   * CRC, so CRC alone does not reject it. Both used to be accepted as data,
   * which is the origin of the "sticky-zero CH_CTRL", the impossible IMAX, the
   * open-load reported on a fitted resistor, and INIT_DONE reading 0 on a
   * healthy device. Two CRC attempts keep freeze-complete (2× FB_UPD +
   * FB_I_AVG + FB_DC) inside the 2 ms InnerControl budget; four attempts ×
   * Mode1 IdleAll+30 µs per frame ran last_step ~9–11 ms on hold ticks.
   * ICVID Init has its own recovery path. */
  constexpr uint8_t kReadAttempts = 2;
  const ExpectedReply expected = ExpectedReplyFor(address);
  uint32_t rx_all[3] = {};
  uint32_t last_reply = 0;
  uint32_t last_flush = 0;
  bool saw_any_frame = false;

  for (uint8_t attempt = 0; attempt < kReadAttempts; ++attempt) {
    auto multi = static_cast<Derived*>(this)->TransferMulti(
        std::span<const uint32_t>{tx_words}, std::span<uint32_t>{rx_all});
    if (!multi) {
      return tle::unexpected(multi.error());
    }
    /* Slot 2 answers the command; slot 1 answers the flush dummy and is only
     * kept as a fallback for the ICVID scan below. */
    last_reply = rx_all[2];
    last_flush = rx_all[1];

    SPIFrame candidate{};
    candidate.word = last_reply;

    /* Undriven MISO — nothing to validate, and retrying is pointless if the
     * part is absent, but cheap if a single window was missed. */
    if (last_reply == 0U || last_reply == 0xFFFFFFFFU) {
      continue;
    }
    saw_any_frame = true;

    if (!VerifyFrameCrc(candidate)) {
      continue;
    }

    if (candidate.rx_common.reply_mode == 0x02) {
      /* Critical fault frame (UV / clock / supplies). The device is answering
       * truthfully that it cannot serve the read, so this is not retried. */
      NoteCriticalFault(candidate);
      return tle::unexpected(CommError::BusError);
    }

    const bool width_ok =
        (expected == ExpectedReply::Bits22) ? (candidate.rx_common.reply_mode == 0x01)
                                            : (candidate.rx_common.reply_mode == 0x00);
    if (!width_ok) {
      continue; /* Another register's reply landed in this slot. */
    }

    return (candidate.rx_common.reply_mode == 0x01)
               ? static_cast<uint32_t>(candidate.rx_22bit.data)
               : static_cast<uint32_t>(candidate.rx_16bit.data);
  }

  const uint32_t rx_words[2] = {last_flush, last_reply};

  /* ICVID-only: recover a valid device ID from a misaligned 32-bit MISO word
   * on long soft-CS / Mode1 buses (data may sit in bits[23:8], or appear one
   * bit early as 0x82xx/0x80xx/0xC0xx). Restricted to ICVID so lane noise
   * cannot false-match 0xC1xx inside other registers. */
  if (address == CentralReg::ICVID) {
    auto extract_icvid_candidate = [](uint32_t word) noexcept -> uint16_t {
      if (word == 0U || word == 0xFFFFFFFFU) {
        return 0;
      }
      SPIFrame f{};
      f.word = word;
      const uint16_t cands[] = {
          static_cast<uint16_t>(f.rx_16bit.data),
          static_cast<uint16_t>((word >> 8) & 0xFFFFU),
          static_cast<uint16_t>((word >> 16) & 0xFFFFU),
          static_cast<uint16_t>(word & 0xFFFFU),
      };
      for (uint16_t c : cands) {
        if (DeviceID::IsValidDevice(c)) {
          return c;
        }
        /* One-bit-early Mode1 phantoms of a real 0xC1xx ID. */
        if ((c & 0xFF00U) == 0x8200U || (c & 0xFF00U) == 0x8000U ||
            (c & 0xFF00U) == 0xC000U) {
          const uint16_t unshift =
              static_cast<uint16_t>((static_cast<uint32_t>(c) << 1) & 0xFFFFU);
          if (DeviceID::IsValidDevice(unshift)) {
            return unshift;
          }
        }
      }
      return 0;
    };
    for (uint32_t word : {rx_words[1], rx_words[0]}) {
      const uint16_t id = extract_icvid_candidate(word);
      if (id != 0U) {
        return static_cast<uint32_t>(id);
      }
    }
  }

  /* Every attempt produced a frame that could not be trusted. Reporting the
   * failure is the whole point: the caller can retry, hold its last known
   * value, or degrade — all of which beat decoding a corrupt word as state.
   *
   * `verify_crc` no longer gates this. It used to let callers opt out of
   * validation, and every diagnostic path did (`ReadRegister(addr, false)`),
   * which is why corrupt and stale frames reached the decoders in the first
   * place. The parameter is retained for API compatibility. */
  (void)verify_crc;
  return tle::unexpected(saw_any_frame ? CommError::CRCError
                                       : CommError::NoReply);
}

template <typename Derived>
inline CommResult<void> SpiInterface<Derived>::ReadMulti(
    std::span<const uint16_t> addresses, std::span<uint32_t> values,
    uint16_t& valid_mask) noexcept {
  valid_mask = 0U;
  const size_t n = addresses.size();
  if (n == 0U || n > kMaxPipelinedReads || values.size() != n) {
    return tle::unexpected(CommError::InvalidParameter);
  }

  SPIFrame dummy_frame = SPIFrame::MakeRead(0);
  dummy_frame.tx_fields.crc = CalculateFrameCrc(dummy_frame);

  /* [flush dummy][cmd 0..n-1][trailing dummy]: the reply to command i is
   * clocked out during frame i+2. See the ReadMulti doc comment. */
  uint32_t tx_words[kMaxPipelinedReads + 2] = {};
  uint32_t rx_words[kMaxPipelinedReads + 2] = {};
  tx_words[0] = dummy_frame.word;
  for (size_t i = 0; i < n; ++i) {
    SPIFrame cmd = SPIFrame::MakeRead(addresses[i]);
    cmd.tx_fields.crc = CalculateFrameCrc(cmd);
    tx_words[i + 1] = cmd.word;
    values[i] = 0U;
  }
  tx_words[n + 1] = dummy_frame.word;

  const size_t frames = n + 2U;
  auto multi = static_cast<Derived*>(this)->TransferMulti(
      std::span<const uint32_t>{tx_words, frames},
      std::span<uint32_t>{rx_words, frames});
  if (!multi) {
    return tle::unexpected(multi.error());
  }

  for (size_t i = 0; i < n; ++i) {
    SPIFrame candidate{};
    candidate.word = rx_words[i + 2U];
    if (candidate.word == 0U || candidate.word == 0xFFFFFFFFU) {
      continue; /* Undriven MISO. */
    }
    if (!VerifyFrameCrc(candidate)) {
      continue;
    }
    if (candidate.rx_common.reply_mode == 0x02) {
      /* Critical fault (UV / clock / supplies). The device is telling us it
       * cannot serve any of these reads, so abandon the whole burst. */
      NoteCriticalFault(candidate);
      return tle::unexpected(CommError::BusError);
    }
    const ExpectedReply expected = ExpectedReplyFor(addresses[i]);
    const bool width_ok = (expected == ExpectedReply::Bits22)
                              ? (candidate.rx_common.reply_mode == 0x01)
                              : (candidate.rx_common.reply_mode == 0x00);
    if (!width_ok) {
      continue; /* Another register's reply landed in this slot. */
    }
    values[i] = (candidate.rx_common.reply_mode == 0x01)
                    ? static_cast<uint32_t>(candidate.rx_22bit.data)
                    : static_cast<uint32_t>(candidate.rx_16bit.data);
    valid_mask = static_cast<uint16_t>(valid_mask | (1U << i));
  }
  return {};
}

template <typename Derived>
inline CommResult<void> SpiInterface<Derived>::Write(uint16_t address, uint16_t value,
                                                      bool verify_crc) noexcept {
  // Create write frame
  SPIFrame tx_frame = SPIFrame::MakeWrite(address, value);

  // Calculate and set CRC
  tx_frame.tx_fields.crc = CalculateFrameCrc(tx_frame);

  /* Leading flush dummy then a trailing dummy to clock the write response —
   * same self-synchronising pattern as Read(), so the status bits checked
   * below belong to this write and not to whatever ran before it. (See Read()
   * for why the dummies must not repeat the accessed address.) */
  SPIFrame dummy_frame = SPIFrame::MakeRead(0);
  dummy_frame.tx_fields.crc = CalculateFrameCrc(dummy_frame);

  const uint32_t tx_words[3] = {dummy_frame.word, tx_frame.word,
                                dummy_frame.word};
  uint32_t rx_words[3] = {};
  auto multi = static_cast<Derived*>(this)->TransferMulti(
      std::span<const uint32_t>{tx_words}, std::span<uint32_t>{rx_words});
  if (!multi) {
    return tle::unexpected(multi.error());
  }

  // Response to the write frame lands in the last slot.
  SPIFrame rx_frame{};
  rx_frame.word = rx_words[2];

  // Verify CRC if requested
  if (verify_crc && !VerifyFrameCrc(rx_frame)) {
    return tle::unexpected(CommError::CRCError);
  }

  // Check for errors in status field (for 16-bit reply frames)
  if (rx_frame.rx_common.reply_mode == 0x00) {
    // Check status field for errors
    if (rx_frame.rx_16bit.status != 0x00) {
      // Status indicates an error
      return tle::unexpected(CommError::TransferError);
    }
  } else if (rx_frame.rx_common.reply_mode == 0x02) {
    // Critical fault frame — keep the cause byte for the caller.
    NoteCriticalFault(rx_frame);
    return tle::unexpected(CommError::BusError);
  }

  return {};
}

} // namespace tle92466ed
