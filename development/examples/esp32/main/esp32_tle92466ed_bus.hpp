/**
 * @file esp32_tle92466ed_bus.hpp
 * @brief ESP32 Communication Interface implementation for TLE92466ED driver
 * 
 * This file provides the ESP32-specific implementation of the TLE92466ED CommInterface.
 * It handles SPI communication and timing functions required by the driver.
 * 
 * @author N3b3x
 * @date 2025-10-21
 * @version 2.0.0
 */

#pragma once

#include "tle92466ed_spi_interface.hpp"
#include "esp32_tle92466ed_test_config.hpp"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <memory>
#include <cstdarg>
#include <span>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tle92466ed_registers.hpp"

using namespace tle92466ed;

namespace {
    [[nodiscard]] constexpr inline uint32_t byte_swap_32(uint32_t value) noexcept {
        return ((value & 0xFF000000U) >> 24) |
               ((value & 0x00FF0000U) >> 8) |
               ((value & 0x0000FF00U) << 8) |
               ((value & 0x000000FFU) << 24);
    }
}

/**
 * @class Esp32Tle92466edSpiBus
 * @brief ESP32 implementation of the TLE92466ED CommInterface
 * 
 * This class provides the platform-specific implementation for ESP32,
 * handling SPI communication with proper timing and error handling.
 */
class Esp32Tle92466edSpiBus : public tle92466ed::SpiInterface<Esp32Tle92466edSpiBus> {
public:
    // Make base class Log method accessible
    using SpiInterface<Esp32Tle92466edSpiBus>::Log;
    /**
     * @brief SPI configuration structure for ESP32
     */
    struct SPIConfig {
        spi_host_device_t host = SPI2_HOST;     ///< SPI host (SPI2_HOST for ESP32-C6)
        int16_t miso_pin = 2;                   ///< MISO pin (GPIO2, -1 = not configured)
        int16_t mosi_pin = 7;                   ///< MOSI pin (GPIO7, -1 = not configured)
        int16_t sclk_pin = 6;                   ///< SCLK pin (GPIO6, -1 = not configured)
        int16_t cs_pin = 10;                    ///< CS pin (GPIO10, -1 = not configured)
        int16_t resn_pin = -1;                  ///< RESN pin (active low, -1 = not configured)
        int16_t en_pin = -1;                    ///< EN pin (active high, -1 = not configured)
        int16_t faultn_pin = -1;                ///< FAULTN pin (active low, -1 = not configured)
        int16_t drv0_pin = -1;                  ///< DRV0 pin (external drive control, -1 = not configured)
        int16_t drv1_pin = -1;                  ///< DRV1 pin (external drive control, -1 = not configured)
        uint32_t frequency = 1000000;           ///< SPI frequency (1MHz)
        uint8_t mode = 1;                       ///< SPI mode (1 = CPOL=0, CPHA=1)
        uint8_t queue_size = 1;                 ///< Transaction queue size
        uint8_t cs_ena_pretrans = 1;            ///< CS asserted N clock cycles before transaction
        uint8_t cs_ena_posttrans = 1;           ///< CS held N clock cycles after transaction
    };

    /**
     * @brief Constructor with default SPI configuration
     */
    Esp32Tle92466edSpiBus() : Esp32Tle92466edSpiBus(SPIConfig{}) {}

    /**
     * @brief Constructor with custom SPI configuration
     * @param config SPI configuration parameters
     */
    explicit Esp32Tle92466edSpiBus(const SPIConfig& config) noexcept : config_(config) {
        ESP_LOGI(TAG, "Esp32Tle92466edSpiBus created with SPI config:");
        ESP_LOGI(TAG, "  Host: %d", static_cast<int>(config_.host));
        ESP_LOGI(TAG, "  MISO: GPIO%d", config_.miso_pin);
        ESP_LOGI(TAG, "  MOSI: GPIO%d", config_.mosi_pin);
        ESP_LOGI(TAG, "  SCLK: GPIO%d", config_.sclk_pin);
        ESP_LOGI(TAG, "  CS: GPIO%d", config_.cs_pin);
        ESP_LOGI(TAG, "  Frequency: %d Hz", config_.frequency);
        const char* mode_desc;
        switch (config_.mode) {
            case 0: mode_desc = "CPOL=0, CPHA=0"; break;
            case 1: mode_desc = "CPOL=0, CPHA=1"; break;
            case 2: mode_desc = "CPOL=1, CPHA=0"; break;
            case 3: mode_desc = "CPOL=1, CPHA=1"; break;
            default: mode_desc = "Invalid"; break;
        }
        ESP_LOGI(TAG, "  Mode: %d (%s)", config_.mode, mode_desc);
        if (config_.resn_pin >= 0) ESP_LOGI(TAG, "  RESN: GPIO%d", config_.resn_pin);
        if (config_.en_pin >= 0) ESP_LOGI(TAG, "  EN: GPIO%d", config_.en_pin);
        if (config_.faultn_pin >= 0) ESP_LOGI(TAG, "  FAULTN: GPIO%d", config_.faultn_pin);
        if (config_.drv0_pin >= 0) ESP_LOGI(TAG, "  DRV0: GPIO%d", config_.drv0_pin);
        if (config_.drv1_pin >= 0) ESP_LOGI(TAG, "  DRV1: GPIO%d", config_.drv1_pin);
    }

    /**
     * @brief Destructor - cleans up SPI resources
     * 
     * Defined inline to avoid incomplete type issues with std::unique_ptr
     */
    ~Esp32Tle92466edSpiBus() noexcept {
        if (spi_device_ != nullptr) {
            spi_bus_remove_device(spi_device_);
            spi_device_ = nullptr;
        }
        
        if (initialized_) {
            spi_bus_free(config_.host);
            initialized_ = false;
        }
    }

    /**
     * @brief Initialize the CommInterface (must be called before use)
     * @return CommResult<void> Success or error
     */
    auto Init() noexcept -> CommResult<void> {
        if (initialized_) {
            ESP_LOGW(TAG, "CommInterface already initialized");
            return {};
        }
        ESP_LOGI(TAG, "Initializing Esp32Tle92466edSpiBus...");
        if (auto result = initializeGPIO(); !result) {
            ESP_LOGE(TAG, "Failed to initialize GPIO pins");
            return std::unexpected(result.error());
        }
        if (auto result = initializeSPI(); !result) {
            ESP_LOGE(TAG, "Failed to initialize SPI bus");
            return std::unexpected(result.error());
        }
        if (auto result = addSPIDevice(); !result) {
            ESP_LOGE(TAG, "Failed to add SPI device");
            spi_bus_free(config_.host);
            return std::unexpected(result.error());
        }
        initialized_ = true;
        ESP_LOGI(TAG, "Esp32Tle92466edSpiBus initialized successfully");
        return {};
    }

    /**
     * @brief Deinitialize the CommInterface
     * @return CommResult<void> Success or error
     */
    auto Deinit() noexcept -> CommResult<void> {
        if (!initialized_) {
            ESP_LOGW(TAG, "CommInterface not initialized");
            return {};
        }
        ESP_LOGI(TAG, "Deinitializing Esp32Tle92466edSpiBus...");
        if (spi_device_ != nullptr) {
            spi_bus_remove_device(spi_device_);
            spi_device_ = nullptr;
        }
        spi_bus_free(config_.host);
        initialized_ = false;
        ESP_LOGI(TAG, "Esp32Tle92466edSpiBus deinitialized successfully");
        return {};
    }

    /**
     * @brief Transfer 32-bit data via SPI (full-duplex)
     * @param tx_data The 32-bit data to transmit
     * @return CommResult<uint32_t> Received 32-bit data or error
     */
    auto Transfer32(uint32_t tx_data) noexcept -> CommResult<uint32_t> {
        if (!initialized_) {
            ESP_LOGE(TAG, "CommInterface not initialized");
            return std::unexpected(CommError::HardwareNotReady);
        }
        uint32_t tx_data_swapped = byte_swap_32(tx_data);
        uint32_t rx_data_raw = 0;
        spi_transaction_t trans = {};
        trans.length = 32;
        trans.tx_buffer = &tx_data_swapped;
        trans.rx_buffer = &rx_data_raw;

#if ESP32_TLE_COMM_ENABLE_DETAILED_SPI_LOGGING
        uint8_t* tx_bytes_orig = reinterpret_cast<uint8_t*>(&tx_data);
        uint8_t* tx_bytes_swapped = reinterpret_cast<uint8_t*>(&tx_data_swapped);
        uint8_t crc_tx = tx_bytes_orig[3];
        uint8_t addr_byte = tx_bytes_orig[2];
        uint8_t data_high = tx_bytes_orig[1];
        uint8_t data_low = tx_bytes_orig[0];
        uint8_t address_upper = (addr_byte >> 1) & 0x7F;
        bool is_write = (addr_byte & 0x01) != 0;
        uint16_t data_16bit = (static_cast<uint16_t>(data_high) << 8) | data_low;
        ESP_LOGI(TAG, "SPI TX Frame: 0x%08X | Swapped: 0x%08X", tx_data, tx_data_swapped);
        SPIFrame tx_frame_for_crc;
        tx_frame_for_crc.word = tx_data;
        uint8_t tx_crc_received = tx_frame_for_crc.tx_fields.crc;
        tx_frame_for_crc.tx_fields.crc = 0;
        uint8_t tx_crc_calculated = CalculateFrameCrc(tx_frame_for_crc);
        bool tx_crc_match = (tx_crc_received == tx_crc_calculated);
        ESP_LOGI(TAG, "  CRC=0x%02X calc=0x%02X %s | Addr=0x%02X %s | Data=0x%04X",
                 tx_crc_received, tx_crc_calculated, tx_crc_match ? "OK" : "MISMATCH",
                 address_upper, is_write ? "W" : "R", data_16bit);
#endif
        esp_err_t ret = spi_device_transmit(spi_device_, &trans);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI transfer failed: %s", esp_err_to_name(ret));
            return std::unexpected(CommError::TransferError);
        }
        uint32_t rx_data = byte_swap_32(rx_data_raw);
#if ESP32_TLE_COMM_ENABLE_DETAILED_SPI_LOGGING
        SPIFrame rx_frame;
        rx_frame.word = rx_data;
        uint8_t reply_mode = rx_frame.rx_common.reply_mode;
        const char* reply_mode_str;
        switch (reply_mode) {
            case 0x00: reply_mode_str = "16-bit Reply"; break;
            case 0x01: reply_mode_str = "22-bit Reply"; break;
            case 0x02: reply_mode_str = "Critical Fault"; break;
            default: reply_mode_str = "Unknown"; break;
        }
        ESP_LOGI(TAG, "SPI RX Frame: 0x%08X (raw: 0x%08X) | %s", rx_data, rx_data_raw, reply_mode_str);
        if (reply_mode != 0x02) {
            bool rx_crc_valid = VerifyFrameCrc(rx_frame);
            ESP_LOGI(TAG, "  CRC %s", rx_crc_valid ? "OK" : "MISMATCH");
        }
        if (reply_mode == 0x00) {
            ESP_LOGI(TAG, "  Status=0x%02X RW=%d Data=0x%04X",
                     rx_frame.rx_16bit.status, rx_frame.rx_16bit.rw_echo, rx_frame.rx_16bit.data);
        } else if (reply_mode == 0x01) {
            ESP_LOGI(TAG, "  22-bit Data=0x%06X", rx_frame.rx_22bit.data);
        } else if (reply_mode == 0x02) {
            auto fault_flags = CriticalFaultFlags::Extract(rx_frame);
            ESP_LOGI(TAG, "  Faults: 1V5=%d 2V5=%d BG=%d CLK_SLOW=%d CLK_FAST=%d",
                     fault_flags.supply_1v5_ok, fault_flags.supply_2v5_ok,
                     fault_flags.adc_bandgap_ok, fault_flags.clk_too_slow, fault_flags.clk_too_fast);
        }
#endif
        return rx_data;
    }

    /**
     * @brief Transfer multiple 32-bit words via SPI
     * @param tx_data Span of transmit data (32-bit words)
     * @param rx_data Span to store received data (32-bit words)
     * @return CommResult<void> Success or error
     */
    auto TransferMulti(std::span<const uint32_t> tx_data,
                       std::span<uint32_t> rx_data) noexcept -> CommResult<void> {
        if (!initialized_) {
            ESP_LOGE(TAG, "CommInterface not initialized");
            return std::unexpected(CommError::HardwareNotReady);
        }
        if (tx_data.size() != rx_data.size()) {
            ESP_LOGE(TAG, "Buffer size mismatch: tx=%zu, rx=%zu", tx_data.size(), rx_data.size());
            return std::unexpected(CommError::InvalidParameter);
        }
        for (size_t i = 0; i < tx_data.size(); ++i) {
            if (auto result = Transfer32(tx_data[i]); !result) {
                return std::unexpected(result.error());
            } else {
                rx_data[i] = *result;
            }
        }
        return {};
    }

    /**
     * @brief Delay for specified duration
     * @param microseconds Duration to delay in microseconds
     * @return CommResult<void> Success or error
     */
    auto Delay(uint32_t microseconds) noexcept -> CommResult<void> {
        if (microseconds == 0) return {};
        if (microseconds < 1000) {
            int64_t start_time = esp_timer_get_time();
            while ((esp_timer_get_time() - start_time) < microseconds) {}
        } else {
            vTaskDelay(pdMS_TO_TICKS(microseconds / 1000));
            uint32_t remaining_us = microseconds % 1000;
            if (remaining_us > 0) {
                int64_t start_time = esp_timer_get_time();
                while ((esp_timer_get_time() - start_time) < remaining_us) {}
            }
        }
        return {};
    }

    /**
     * @brief Configure SPI parameters
     * @param config New SPI configuration
     * @return CommResult<void> Success or error
     */
    auto Configure(const tle92466ed::SPIConfig& config) noexcept -> CommResult<void> {
        ESP_LOGW(TAG, "SPI configuration update requested - not fully implemented");
        return {};
    }

    /**
     * @brief Check if hardware is ready for communication
     * @return true if ready, false otherwise
     */
    bool IsReady() const noexcept {
        return initialized_ && (spi_device_ != nullptr);
    }

    /**
     * @brief Get the last error that occurred
     * @return CommError The last error code
     */
    CommError GetLastError() const noexcept {
        return last_error_;
    }

    /**
     * @brief Clear any pending errors
     * @return CommResult<void> Success or error
     */
    auto ClearErrors() noexcept -> CommResult<void> {
        last_error_ = CommError::None;
        return {};
    }

    /**
     * @brief Get the current SPI configuration
     * @return Current SPI configuration
     */
    auto getConfig() const noexcept -> const SPIConfig& { return config_; }

    /**
     * @brief Set GPIO control pin signal
     * @param pin Control pin to set (RESN or EN)
     * @param signal GPIO signal level (ACTIVE or INACTIVE)
     * @return CommResult<void> Success or error
     * 
     * @note The mapping from GpioSignal to physical GPIO level is the responsibility
     *       of this bus implementation based on the board's active-level design.
     *       For TLE92466ED: RESN is active-low, EN is active-high.
     */
    auto GpioSet(CtrlPin pin, GpioSignal signal) noexcept -> CommResult<void> {
        if (!IsReady()) {
            last_error_ = CommError::HardwareNotReady;
            return std::unexpected(CommError::HardwareNotReady);
        }
        int gpio_pin = -1;
        switch (pin) {
            case CtrlPin::RESN: gpio_pin = config_.resn_pin; break;
            case CtrlPin::EN: gpio_pin = config_.en_pin; break;
            case CtrlPin::FAULTN:
                last_error_ = CommError::InvalidParameter;
                return std::unexpected(CommError::InvalidParameter);
        }
        if (gpio_pin < 0) {
            ESP_LOGE(TAG, "GPIO pin not configured for %s", pin == CtrlPin::RESN ? "RESN" : "EN");
            last_error_ = CommError::InvalidParameter;
            return std::unexpected(CommError::InvalidParameter);
        }
        // Map GpioSignal to physical level based on pin active-level:
        // RESN: active-low  → ACTIVE=0, INACTIVE=1
        // EN:   active-high → ACTIVE=1, INACTIVE=0
        int gpio_level;
        switch (pin) {
            case CtrlPin::RESN:
                gpio_level = (signal == GpioSignal::ACTIVE) ? 0 : 1;  // Active-low
                break;
            case CtrlPin::EN:
                gpio_level = (signal == GpioSignal::ACTIVE) ? 1 : 0;  // Active-high
                break;
            default:
                return std::unexpected(CommError::InvalidParameter);
        }
        esp_err_t ret = gpio_set_level(static_cast<gpio_num_t>(gpio_pin), gpio_level);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set GPIO%d level: %s", gpio_pin, esp_err_to_name(ret));
            last_error_ = CommError::BusError;
            return std::unexpected(CommError::BusError);
        }
        ESP_LOGD(TAG, "Set %s pin (GPIO%d) to %s",
                 pin == CtrlPin::RESN ? "RESN" : "EN", gpio_pin,
                 signal == GpioSignal::ACTIVE ? "ACTIVE" : "INACTIVE");
        return {};
    }

    /**
     * @brief Read GPIO control pin signal
     * @param pin Control pin to read (FAULTN)
     * @return CommResult<GpioSignal> Pin signal level or error
     * 
     * @note The mapping from physical GPIO level to GpioSignal is the responsibility
     *       of this bus implementation. FAULTN is active-low: physical 0 = ACTIVE.
     */
    auto GpioRead(CtrlPin pin) noexcept -> CommResult<GpioSignal> {
        if (!IsReady()) {
            last_error_ = CommError::HardwareNotReady;
            return std::unexpected(CommError::HardwareNotReady);
        }
        if (pin != CtrlPin::FAULTN) {
            last_error_ = CommError::InvalidParameter;
            return std::unexpected(CommError::InvalidParameter);
        }
        int gpio_pin = config_.faultn_pin;
        if (gpio_pin < 0) {
            ESP_LOGE(TAG, "FAULTN GPIO pin not configured");
            last_error_ = CommError::InvalidParameter;
            return std::unexpected(CommError::InvalidParameter);
        }
        int gpio_level = gpio_get_level(static_cast<gpio_num_t>(gpio_pin));
        // FAULTN is active-low: physical 0 = fault active
        GpioSignal signal = (gpio_level == 0) ? GpioSignal::ACTIVE : GpioSignal::INACTIVE;
        ESP_LOGD(TAG, "Read FAULTN pin (GPIO%d): %s", gpio_pin,
                 signal == GpioSignal::ACTIVE ? "FAULT" : "NO FAULT");
        return signal;
    }

    /**
     * @brief Log a message with specified severity level and tag (ESP_LOG implementation)
     * @param level Log severity level
     * @param tag Tag/component name for the log message
     * @param format Format string (printf-style)
     * @param args va_list of arguments
     */
    void Log(LogLevel level, const char* tag, const char* format, va_list args) noexcept {
        esp_log_level_t esp_level;
        switch (level) {
            case LogLevel::Error:   esp_level = ESP_LOG_ERROR; break;
            case LogLevel::Warn:    esp_level = ESP_LOG_WARN; break;
            case LogLevel::Info:    esp_level = ESP_LOG_INFO; break;
            case LogLevel::Debug:   esp_level = ESP_LOG_DEBUG; break;
            case LogLevel::Verbose: esp_level = ESP_LOG_VERBOSE; break;
            default:                esp_level = ESP_LOG_INFO; break;
        }
        esp_log_writev(esp_level, tag, format, args);
    }

    /**
     * @brief Check if CommInterface is initialized
     * @return true if initialized, false otherwise
     */
    auto isInitialized() const noexcept -> bool { return initialized_; }

private:
    SPIConfig config_;                          ///< SPI configuration
    spi_device_handle_t spi_device_ = nullptr;  ///< SPI device handle
    bool initialized_ = false;                  ///< Initialization state
    CommError last_error_ = CommError::None;    ///< Last error that occurred
    
    static constexpr const char* TAG = "Esp32TleComm"; ///< Logging tag

    /**
     * @brief Initialize GPIO pins (RESN, EN, FAULTN)
     * @return CommResult<void> Success or error
     */
    auto initializeGPIO() noexcept -> CommResult<void> {
        auto configure_output_pin = [this](int16_t pin, const char* name, int initial_level) -> CommResult<void> {
            if (pin < 0) return {};
            gpio_config_t cfg = {
                .pin_bit_mask = (1ULL << pin),
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE
            };
            if (gpio_config(&cfg) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to configure %s pin (GPIO%d)", name, pin);
                return std::unexpected(CommError::HardwareNotReady);
            }
            gpio_set_level(static_cast<gpio_num_t>(pin), initial_level);
            ESP_LOGI(TAG, "%s pin (GPIO%d) initialized, level=%d", name, pin, initial_level);
            return {};
        };
        if (auto r = configure_output_pin(config_.resn_pin, "RESN", 0); !r) return r;
        if (auto r = configure_output_pin(config_.en_pin, "EN", 0); !r) return r;
        if (auto r = configure_output_pin(config_.drv0_pin, "DRV0", 0); !r) return r;
        if (auto r = configure_output_pin(config_.drv1_pin, "DRV1", 0); !r) return r;
        if (config_.faultn_pin >= 0) {
            gpio_config_t cfg = {
                .pin_bit_mask = (1ULL << config_.faultn_pin),
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = GPIO_PULLUP_ENABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE
            };
            if (gpio_config(&cfg) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to configure FAULTN pin (GPIO%d)", config_.faultn_pin);
                return std::unexpected(CommError::HardwareNotReady);
            }
            ESP_LOGI(TAG, "FAULTN pin (GPIO%d) initialized as input", config_.faultn_pin);
        }
        return {};
    }

    /**
     * @brief Initialize SPI bus
     * @return CommResult<void> Success or error
     */
    auto initializeSPI() noexcept -> CommResult<void> {
        spi_bus_config_t bus_config = {
            .mosi_io_num = config_.mosi_pin,
            .miso_io_num = config_.miso_pin,
            .sclk_io_num = config_.sclk_pin,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .data4_io_num = -1,
            .data5_io_num = -1,
            .data6_io_num = -1,
            .data7_io_num = -1,
            .max_transfer_sz = 32,
            .flags = SPICOMMON_BUSFLAG_MASTER
        };
        esp_err_t ret = spi_bus_initialize(config_.host, &bus_config, SPI_DMA_DISABLED);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI bus initialization failed: %s", esp_err_to_name(ret));
            return std::unexpected(CommError::HardwareNotReady);
        }
        ESP_LOGI(TAG, "SPI bus initialized successfully");
        return {};
    }

    /**
     * @brief Add SPI device to the bus
     * @return CommResult<void> Success or error  
     */
    auto addSPIDevice() noexcept -> CommResult<void> {
        uint8_t spi_mode = static_cast<uint8_t>(config_.mode);
        if (spi_mode > 3) {
            ESP_LOGE(TAG, "Invalid SPI mode %d, using Mode 1", spi_mode);
            spi_mode = 1;
        }
        spi_device_interface_config_t dev_config = {};
        dev_config.command_bits = 0;
        dev_config.address_bits = 0;
        dev_config.dummy_bits = 0;
        dev_config.mode = spi_mode;
        dev_config.clock_source = SPI_CLK_SRC_DEFAULT;
        dev_config.duty_cycle_pos = 128;
        dev_config.cs_ena_pretrans = config_.cs_ena_pretrans;
        dev_config.cs_ena_posttrans = config_.cs_ena_posttrans;
        dev_config.clock_speed_hz = config_.frequency;
        dev_config.input_delay_ns = 0;
        dev_config.spics_io_num = config_.cs_pin;
        dev_config.flags = 0;
        dev_config.queue_size = config_.queue_size;
        dev_config.pre_cb = nullptr;
        dev_config.post_cb = nullptr;
        esp_err_t ret = spi_bus_add_device(config_.host, &dev_config, &spi_device_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
            return std::unexpected(CommError::HardwareNotReady);
        }
        ESP_LOGI(TAG, "SPI device added successfully with Mode %d", spi_mode);
        return {};
    }
};

/**
 * @brief Create a configured Esp32Tle92466edSpiBus instance for TLE92466ED
 * @return Unique pointer to configured CommInterface instance
 * 
 * This function uses the configuration from TLE92466ED_TestConfig.hpp
 * 
 * Note: Defined in .cpp file to avoid incomplete type issues with std::unique_ptr
 */
inline auto CreateEsp32Tle92466edSpiBus() noexcept -> std::unique_ptr<Esp32Tle92466edSpiBus> {
    using namespace TLE92466ED_TestConfig;
    Esp32Tle92466edSpiBus::SPIConfig config;
    config.host = SPI2_HOST;
    config.miso_pin = SPIPins::MISO;
    config.mosi_pin = SPIPins::MOSI;
    config.sclk_pin = SPIPins::SCLK;
    config.cs_pin = SPIPins::CS;
    config.resn_pin = ControlPins::RESN;
    config.en_pin = ControlPins::EN;
    config.faultn_pin = ControlPins::FAULTN;
    config.drv0_pin = ControlPins::DRV0;
    config.drv1_pin = ControlPins::DRV1;
    config.frequency = SPIParams::FREQUENCY;
    config.mode = SPIParams::MODE;
    config.queue_size = SPIParams::QUEUE_SIZE;
    config.cs_ena_pretrans = SPIParams::CS_ENA_PRETRANS;
    config.cs_ena_posttrans = SPIParams::CS_ENA_POSTTRANS;
    auto interface = std::make_unique<Esp32Tle92466edSpiBus>(config);
    if (auto result = interface->Init(); !result) {
        ESP_LOGE("TLE_Factory", "Failed to initialize CommInterface");
        return nullptr;
    }
    return interface;
}

