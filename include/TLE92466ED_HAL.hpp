/**
 * @file TLE92466ED_HAL.hpp
 * @brief Hardware Abstraction Layer (HAL) base class for TLE92466ED driver
 * @author AI Generated Driver
 * @date 2025-10-20
 * @version 2.0.0
 *
 * @details
 * This file defines the hardware abstraction layer interface for the
 * TLE92466ED Six-Channel Low-Side Solenoid Driver IC. The HAL provides a
 * polymorphic interface that allows the driver to work with any hardware
 * platform by implementing the virtual transmission functions.
 *
 * The TLE92466ED uses **32-bit SPI communication** with the following structure:
 * - MOSI: 32-bit frame (CRC[31:24] + Address[23:17] + R/W[16] + Data[15:0])
 * - MISO: 32-bit frame (CRC[31:24] + ReplyMode[23:22] + Status[21:17] + R/W[16] + Data[15:0])
 * - CS: Chip select (active low)
 * - SCLK: Serial clock (up to 10 MHz)
 * - CRC: SAE J1850 8-bit CRC
 *
 * @copyright
 * This is free and unencumbered software released into the public domain.
 */

#ifndef TLE92466ED_HAL_HPP
#define TLE92466ED_HAL_HPP

#include <cstdint>

// Conditional includes based on C++ standard
#if __cpp_lib_expected >= 202202L
#include <expected>
#endif

#if __cpp_lib_span >= 202002L
#include <span>
#endif

#if __cpp_concepts >= 201907L
#include <concepts>
#endif

namespace TLE92466ED {

/**
 * @brief Error codes for HAL operations
 * 
 * This enumeration defines all possible error conditions that can occur
 * during hardware communication with the TLE92466ED IC.
 */
enum class HALError : uint8_t {
    None = 0,                 ///< No error occurred
    BusError,                 ///< SPI bus communication error
    Timeout,                  ///< Operation timed out
    InvalidParameter,         ///< Invalid parameter passed to function
    ChipselectError,          ///< Chip select control failed
    TransferError,            ///< Data transfer failed
    HardwareNotReady,         ///< Hardware not initialized or ready
    BufferOverflow,           ///< Buffer size exceeded
    CRCError,                 ///< CRC mismatch error
    UnknownError              ///< Unknown error occurred
};

// Conditional result type based on C++ standard
#if __cpp_lib_expected >= 202202L
/**
 * @brief HAL operation result type (C++23 std::expected)
 */
template<typename T>
using HALResult = std::expected<T, HALError>;
#else
/**
 * @brief HAL operation result type (C++11 fallback)
 * 
 * For C++11 compatibility, we use a simple struct with success flag
 * and optional error code. Users should check the success flag before
 * accessing the value.
 */
template<typename T>
struct HALResult {
    bool success;
    T value;
    HALError error_code;
    
    HALResult() : success(false), value(), error_code(HALError::None) {}
    HALResult(const T& val) : success(true), value(val), error_code(HALError::None) {}
    HALResult(HALError err) : success(false), value(), error_code(err) {}
    
    // Conversion operators for C++23 compatibility
    operator bool() const noexcept { return success; }
    const T& operator*() const noexcept { return value; }
    T& operator*() noexcept { return value; }
    const T* operator->() const noexcept { return &value; }
    T* operator->() noexcept { return &value; }
    HALError error() const noexcept { return error_code; }
};
#endif

/**
 * @brief SPI transaction configuration
 * 
 * Defines the configuration parameters for SPI communication.
 */
struct SPIConfig {
    uint32_t frequency;        ///< SPI clock frequency in Hz (max 10 MHz for TLE92466ED)
    uint8_t mode;              ///< SPI mode (CPOL=0, CPHA=0 for TLE92466ED)
    uint8_t bits_per_word;     ///< Bits per word (8-bit, transfer 4 bytes for 32-bit frame)
    bool msb_first;            ///< MSB first transmission
    uint32_t timeout_ms;       ///< Transaction timeout in milliseconds
    
    // C++11 constructor with conditional digit separators
    SPIConfig() : frequency(1000000), mode(0), bits_per_word(8), msb_first(true), timeout_ms(100) {}
};

/**
 * @brief Abstract Hardware Abstraction Layer (HAL) base class
 * 
 * @details
 * This pure virtual base class defines the interface that must be implemented
 * for hardware-specific SPI communication. Users must derive from this class
 * and implement the virtual functions for their specific hardware platform
 * (e.g., STM32, ESP32, Arduino, Linux, etc.).
 *
 * The HAL uses conditional C++ features and is designed for embedded systems:
 * - C++23: std::expected for robust error handling
 * - C++11: Simple error codes for maximum compatibility
 * - C++20: std::span for safe array access
 * - C++11: Pointer + size for maximum compatibility
 * - noexcept functions for embedded safety
 * - Minimal dependencies
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
 * class MyPlatformHAL : public TLE92466ED::HAL {
 * public:
 *     HALResult<uint32_t> transfer32(uint32_t tx_data) noexcept override {
 *         uint32_t result = spi_transfer_32bit(tx_data);
 *         if (spi_error()) {
 *             return HALError::TransferError;
 *         }
 *         return result;
 *     }
 *     // ... implement other virtual functions
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
 * - Support for SPI Mode 0 (CPOL=0, CPHA=0)
 * - CRC calculation capability (hardware or software)
 */
class HAL {
public:
    /**
     * @brief Virtual destructor for polymorphic behavior
     */
    virtual ~HAL() {}

    /**
     * @brief Initialize the hardware interface
     * 
     * @details
     * This function should initialize the SPI peripheral, configure GPIO pins,
     * and prepare the hardware for communication. It should be called before
     * any other HAL functions.
     *
     * @return HALResult<void> Success or error code
     * @retval HALError::None Initialization successful
     * @retval HALError::HardwareNotReady Hardware initialization failed
     * @retval HALError::InvalidParameter Invalid configuration
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    virtual HALResult<void> init() noexcept = 0;

    /**
     * @brief Deinitialize the hardware interface
     * 
     * @details
     * Releases hardware resources and disables the SPI peripheral. Should be
     * called when the driver is no longer needed.
     *
     * @return HALResult<void> Success or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    virtual HALResult<void> deinit() noexcept = 0;

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
     * @return HALResult<uint32_t> Received 32-bit data or error code
     * @retval HALError::TransferError SPI transfer failed
     * @retval HALError::Timeout Transfer timeout
     *
     * @par Timing Requirements:
     * - CS must be held low during entire 32-bit transfer
     * - Minimum CS inactive time between transfers: 100ns
     * - Data sampled on rising edge (CPHA=0)
     *
     * @note CRC calculation is handled by the driver layer, not HAL
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    virtual HALResult<uint32_t> transfer32(uint32_t tx_data) noexcept = 0;

    /**
     * @brief Transfer multiple 32-bit words via SPI
     * 
     * @details
     * Performs multiple consecutive SPI transfers efficiently. Useful for
     * reading or writing multiple registers in sequence.
     *
     * @param[in] tx_data Pointer to transmit data (32-bit words)
     * @param[out] rx_data Pointer to store received data (32-bit words)
     * @param[in] count Number of 32-bit words to transfer
     * @return HALResult<void> Success or error code
     * @retval HALError::InvalidParameter Invalid parameters
     * @retval HALError::TransferError Transfer failed
     *
     * @pre tx_data and rx_data must be valid for count words
     */
#if __cpp_lib_span >= 202002L
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    virtual HALResult<void> transfer_multi(
        std::span<const uint32_t> tx_data,
        std::span<uint32_t> rx_data) noexcept = 0;
#else
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    virtual HALResult<void> transfer_multi(
        const uint32_t* tx_data,
        uint32_t* rx_data,
        size_t count) noexcept = 0;
#endif

    /**
     * @brief Assert (activate) chip select
     * 
     * @details
     * Pulls the CS line low to select the TLE92466ED for communication.
     * Must be called before SPI transfers in manual CS mode.
     *
     * @return HALResult<void> Success or error code
     * @retval HALError::ChipselectError CS control failed
     *
     * @note Some implementations may handle CS automatically in transfer32()
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    virtual HALResult<void> chip_select() noexcept = 0;

    /**
     * @brief Deassert (deactivate) chip select
     * 
     * @details
     * Pulls the CS line high to deselect the TLE92466ED after communication.
     * Must be called after SPI transfers in manual CS mode.
     *
     * @return HALResult<void> Success or error code
     * @retval HALError::ChipselectError CS control failed
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    virtual HALResult<void> chip_deselect() noexcept = 0;

    /**
     * @brief Delay for specified duration
     * 
     * @details
     * Provides a hardware-specific delay implementation. Required for timing
     * constraints such as reset pulse width and power-up delays.
     *
     * @param[in] microseconds Duration to delay in microseconds
     * @return HALResult<void> Success or error code
     *
     * @par Timing Requirements:
     * - Reset pulse width: minimum 1µs
     * - Power-up delay: minimum 1ms
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    virtual HALResult<void> delay(uint32_t microseconds) noexcept = 0;

    /**
     * @brief Configure SPI parameters
     * 
     * @details
     * Updates the SPI configuration. Can be called at runtime to adjust
     * communication parameters.
     *
     * @param[in] config New SPI configuration
     * @return HALResult<void> Success or error code
     * @retval HALError::InvalidParameter Invalid configuration
     *
     * @par TLE92466ED SPI Requirements:
     * - Frequency: 100 kHz - 10 MHz
     * - Mode: 0 (CPOL=0, CPHA=0)
     * - Bit order: MSB first
     * - Frame size: 32 bits (4 bytes)
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    virtual HALResult<void> configure(const SPIConfig& config) noexcept = 0;

    /**
     * @brief Check if hardware is ready for communication
     * 
     * @details
     * Verifies that the hardware interface is initialized and ready for
     * SPI transactions.
     *
     * @return true if ready, false otherwise
     */
    virtual bool is_ready() const noexcept = 0;

    /**
     * @brief Get the last error that occurred
     * 
     * @details
     * Retrieves the most recent error code. Useful for debugging and
     * error recovery.
     *
     * @return HALError The last error code
     */
    virtual HALError get_last_error() const noexcept = 0;

    /**
     * @brief Clear any pending errors
     * 
     * @details
     * Resets the error state. Should be called after handling an error
     * condition and before retrying operations.
     *
     * @return HALResult<void> Success or error code
     */
#if __has_cpp_attribute(nodiscard) >= 201907L
    [[nodiscard]]
#endif
    virtual HALResult<void> clear_errors() noexcept = 0;

protected:
    /**
     * @brief Protected constructor to prevent direct instantiation
     * 
     * @details
     * This class can only be instantiated through derived classes.
     */
    HAL() {}

    // Prevent copying
    HAL(const HAL&);
    HAL& operator=(const HAL&);
};

// Conditional concept definition
#if __cpp_concepts >= 201907L
/**
 * @brief Concept for HAL interface validation
 * 
 * @details
 * Compile-time check to ensure a class implements the HAL interface correctly.
 * This helps catch implementation errors at compile time.
 */
template<typename T>
concept HALInterface = std::is_base_of_v<HAL, T> && requires(T hal, uint32_t data, SPIConfig cfg) {
    { hal.init() } -> std::same_as<HALResult<void>>;
    { hal.deinit() } -> std::same_as<HALResult<void>>;
    { hal.transfer32(data) } -> std::same_as<HALResult<uint32_t>>;
    { hal.chip_select() } -> std::same_as<HALResult<void>>;
    { hal.chip_deselect() } -> std::same_as<HALResult<void>>;
    { hal.delay(data) } -> std::same_as<HALResult<void>>;
    { hal.configure(cfg) } -> std::same_as<HALResult<void>>;
    { hal.is_ready() } -> std::same_as<bool>;
    { hal.get_last_error() } -> std::same_as<HALError>;
    { hal.clear_errors() } -> std::same_as<HALResult<void>>;
};
#endif

} // namespace TLE92466ED

#endif // TLE92466ED_HAL_HPP