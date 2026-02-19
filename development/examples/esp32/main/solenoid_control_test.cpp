/**
 * @file solenoid_control_test.cpp
 * @brief Real Hardware Solenoid Control Test with ADC-Based Current Control
 * 
 * This test demonstrates real-world solenoid control using actual hardware:
 * - Two solenoids: one single channel, one parallel pair
 * - ADC-based current control (0-3.3V maps to 0-100% current range)
 * - Independent min/max current limits per solenoid
 * - Real-time current adjustment based on ADC reading
 * - Proper parallel channel operation validation
 * 
 * Hardware Requirements:
 * - Two solenoids connected to TLE92466ED outputs
 * - ADC input (potentiometer or voltage source) on ESP32-C6 ADC pin
 * - Proper power supply for solenoids (VBAT)
 * 
 * @author N3b3x
 * @date 2025-10-21
 * @version 2.0.0
 */

#include <stdio.h>
#include <memory>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "tle92466ed.hpp"
#include "esp32_tle92466ed_bus.hpp"
#include "esp32_tle92466ed_test_config.hpp"

using namespace tle92466ed;
using namespace TLE92466ED_TestConfig;

static const char* TAG = "SolenoidControl";

//=============================================================================
// CONFIGURATION
//=============================================================================

/**
 * @brief ADC Configuration
 * 
 * ESP32-C6 ADC1 Channel 0 maps to GPIO0
 */
struct ADCConfig {
    static constexpr adc_unit_t UNIT = ADC_UNIT_1;           ///< ADC unit (ADC1)
    static constexpr adc_channel_t CHANNEL = ADC_CHANNEL_0;  ///< ADC channel 0 (GPIO0 on ESP32-C6)
    static constexpr adc_atten_t ATTEN = ADC_ATTEN_DB_12;    ///< 0-3.3V range (12dB attenuation)
    static constexpr adc_bitwidth_t BITWIDTH = ADC_BITWIDTH_12; ///< 12-bit resolution
    static constexpr uint32_t SAMPLE_RATE_HZ = 1000;         ///< 1kHz sampling rate
    static constexpr float VREF_MV = 3300.0f;                 ///< Reference voltage (3.3V)
    static constexpr float ADC_MAX = 4095.0f;                 ///< Maximum ADC value (12-bit)
    static constexpr int GPIO_PIN = 0;                        ///< GPIO pin number (GPIO0)
};

/**
 * @brief Solenoid 1 Configuration (Single Channel)
 */
struct Solenoid1Config {
    static constexpr Channel CHANNEL = Channel::CH0;        ///< Single channel
    static constexpr uint16_t MIN_CURRENT_MA = 200;          ///< Minimum current (mA)
    static constexpr uint16_t MAX_CURRENT_MA = 1500;        ///< Maximum current (mA)
    static constexpr bool PARALLEL_MODE = false;             ///< Single channel mode
};

/**
 * @brief Solenoid 2 Configuration (Parallel Pair)
 */
struct Solenoid2Config {
    static constexpr ParallelPair PAIR = ParallelPair::CH1_CH2; ///< Parallel pair
    static constexpr Channel PRIMARY_CHANNEL = Channel::CH1;     ///< Primary channel for control
    static constexpr uint16_t MIN_CURRENT_MA = 400;              ///< Minimum current (mA)
    static constexpr uint16_t MAX_CURRENT_MA = 3000;             ///< Maximum current (mA)
    static constexpr bool PARALLEL_MODE = true;                  ///< Parallel mode
};

//=============================================================================
// GLOBAL RESOURCES
//=============================================================================

static std::unique_ptr<Esp32Tle92466edSpiBus> g_hal;
static tle92466ed::Driver<Esp32Tle92466edSpiBus>* g_driver = nullptr;
static adc_oneshot_unit_handle_t g_adc_handle = nullptr;
static adc_cali_handle_t g_adc_cali_handle = nullptr;

//=============================================================================
// ADC FUNCTIONS
//=============================================================================

/**
 * @brief Initialize ADC for voltage reading
 */
static bool initialize_adc() noexcept {
    ESP_LOGI(TAG, "Initializing ADC...");
    
    // ADC unit configuration
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADCConfig::UNIT,
    };
    
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &g_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return false;
    }
    
    // ADC channel configuration
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADCConfig::ATTEN,
        .bitwidth = ADCConfig::BITWIDTH,
    };
    
    ret = adc_oneshot_config_channel(g_adc_handle, ADCConfig::CHANNEL, &chan_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(g_adc_handle);
        g_adc_handle = nullptr;
        return false;
    }
    
    // ADC calibration (optional but recommended for accuracy)
    bool calibrated = false;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADCConfig::UNIT,
        .atten = ADCConfig::ATTEN,
        .bitwidth = ADCConfig::BITWIDTH,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &g_adc_cali_handle);
    if (ret == ESP_OK) {
        calibrated = true;
        ESP_LOGI(TAG, "ADC calibration enabled (curve fitting)");
    }
#endif
    
    if (!calibrated) {
        ESP_LOGW(TAG, "ADC calibration not available, using raw values");
    }
    
    ESP_LOGI(TAG, "✅ ADC initialized successfully");
    ESP_LOGI(TAG, "  GPIO Pin: GPIO%d", ADCConfig::GPIO_PIN);
    ESP_LOGI(TAG, "  ADC Unit: ADC%d", static_cast<int>(ADCConfig::UNIT));
    ESP_LOGI(TAG, "  ADC Channel: CH%d", static_cast<int>(ADCConfig::CHANNEL));
    ESP_LOGI(TAG, "  Range: 0-%.1fV", ADCConfig::VREF_MV / 1000.0f);
    ESP_LOGI(TAG, "  Resolution: 12-bit (0-4095)");
    
    return true;
}

/**
 * @brief Read ADC voltage in millivolts
 */
static float read_adc_voltage_mv() noexcept {
    if (!g_adc_handle) {
        return 0.0f;
    }
    
    int adc_raw = 0;
    esp_err_t ret = adc_oneshot_read(g_adc_handle, ADCConfig::CHANNEL, &adc_raw);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read ADC: %s", esp_err_to_name(ret));
        return 0.0f;
    }
    
    // Convert to voltage
    float voltage_mv = 0.0f;
    
    if (g_adc_cali_handle) {
        // Use calibrated value if available
        int voltage_mv_int = 0;
        ret = adc_cali_raw_to_voltage(g_adc_cali_handle, adc_raw, &voltage_mv_int);
        if (ret == ESP_OK) {
            voltage_mv = static_cast<float>(voltage_mv_int);
        } else {
            // Fallback to raw calculation
            voltage_mv = (static_cast<float>(adc_raw) / ADCConfig::ADC_MAX) * ADCConfig::VREF_MV;
        }
    } else {
        // Raw calculation
        voltage_mv = (static_cast<float>(adc_raw) / ADCConfig::ADC_MAX) * ADCConfig::VREF_MV;
    }
    
    return voltage_mv;
}

/**
 * @brief Read ADC percentage (0.0 to 100.0)
 */
static float read_adc_percentage() noexcept {
    float voltage_mv = read_adc_voltage_mv();
    float percentage = (voltage_mv / ADCConfig::VREF_MV) * 100.0f;
    
    // Clamp to 0-100%
    if (percentage < 0.0f) percentage = 0.0f;
    if (percentage > 100.0f) percentage = 100.0f;
    
    return percentage;
}

/**
 * @brief Calculate current from percentage for a solenoid
 */
template<typename SolenoidConfig>
static uint16_t calculate_current_from_percentage(float percentage) noexcept {
    float current_range = static_cast<float>(SolenoidConfig::MAX_CURRENT_MA - SolenoidConfig::MIN_CURRENT_MA);
    float current_ma = static_cast<float>(SolenoidConfig::MIN_CURRENT_MA) + (current_range * percentage / 100.0f);
    
    // Round to nearest integer
    return static_cast<uint16_t>(std::round(current_ma));
}

//=============================================================================
// SOLENOID CONTROL FUNCTIONS
//=============================================================================

/**
 * @brief Initialize solenoids
 */
static bool initialize_solenoids() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Initializing solenoids...");
    
    // Enter Config Mode for configuration
    if (auto result = g_driver->EnterConfigMode(); !result) {
        ESP_LOGE(TAG, "Failed to enter Config Mode");
        return false;
    }
    
    // Configure Solenoid 1 (Single Channel)
    ESP_LOGI(TAG, "Configuring Solenoid 1 (Single Channel - %s)...", ToString(Solenoid1Config::CHANNEL));
    if (auto result = g_driver->SetChannelMode(Solenoid1Config::CHANNEL, ChannelMode::ICC); !result) {
        ESP_LOGE(TAG, "Failed to set channel mode for Solenoid 1");
        return false;
    }
    
    // Configure Solenoid 2 (Parallel Pair)
    ESP_LOGI(TAG, "Configuring Solenoid 2 (Parallel Pair - %s)...", ToString(Solenoid2Config::PAIR));
    if (auto result = g_driver->SetParallelOperation(Solenoid2Config::PAIR, true); !result) {
        ESP_LOGE(TAG, "Failed to enable parallel operation for Solenoid 2");
        return false;
    }
    if (auto result = g_driver->SetChannelMode(Solenoid2Config::PRIMARY_CHANNEL, ChannelMode::ICC); !result) {
        ESP_LOGE(TAG, "Failed to set channel mode for Solenoid 2");
        return false;
    }
    
    // Enter Mission Mode
    if (auto result = g_driver->EnterMissionMode(); !result) {
        ESP_LOGE(TAG, "Failed to enter Mission Mode");
        return false;
    }
    
    // Enable outputs
    if (auto result = g_driver->Enable(); !result) {
        ESP_LOGE(TAG, "Failed to enable outputs");
        return false;
    }
    
    ESP_LOGI(TAG, "✅ Solenoids initialized");
    ESP_LOGI(TAG, "  Solenoid 1: %s, Range: %u-%u mA", 
             ToString(Solenoid1Config::CHANNEL),
             Solenoid1Config::MIN_CURRENT_MA,
             Solenoid1Config::MAX_CURRENT_MA);
    ESP_LOGI(TAG, "  Solenoid 2: %s (Parallel), Range: %u-%u mA",
             ToString(Solenoid2Config::PAIR),
             Solenoid2Config::MIN_CURRENT_MA,
             Solenoid2Config::MAX_CURRENT_MA);
    
    return true;
}

/**
 * @brief Print comprehensive telemetry for a solenoid
 */
static void print_solenoid_telemetry(const char* name, Channel channel, bool parallel_mode) noexcept {
    if (!g_driver) {
        return;
    }
    
    // Get current setpoint
    uint16_t setpoint_ma = 0;
    if (auto result = g_driver->GetCurrentSetpoint(channel, parallel_mode); result) {
        setpoint_ma = *result;
    }
    
    // Get average current
    uint16_t avg_current_ma = 0;
    if (auto result = g_driver->GetAverageCurrent(channel, parallel_mode); result) {
        avg_current_ma = *result;
    }
    
    // Get duty cycle
    uint16_t duty_cycle = 0;
    if (auto result = g_driver->GetDutyCycle(channel); result) {
        duty_cycle = *result;
    }
    
    // Get diagnostics
    ChannelDiagnostics diag{};
    if (auto result = g_driver->GetChannelDiagnostics(channel); result) {
        diag = *result;
    }
    
    ESP_LOGI(TAG, "  %s (%s):", name, ToString(channel));
    ESP_LOGI(TAG, "    Setpoint: %u mA | Actual: %u mA | Duty: %u (0x%04X)",
             setpoint_ma, avg_current_ma, duty_cycle, duty_cycle);
    
    // Show current error
    if (setpoint_ma > 0) {
        int16_t current_error = static_cast<int16_t>(avg_current_ma) - static_cast<int16_t>(setpoint_ma);
        float error_percent = (static_cast<float>(current_error) / static_cast<float>(setpoint_ma)) * 100.0f;
        ESP_LOGI(TAG, "    Current Error: %+d mA (%.1f%%)", current_error, error_percent);
    }
    
    // Show diagnostics
    if (diag.overcurrent || diag.short_to_ground || diag.open_load || 
        diag.over_temperature || diag.open_load_short_ground) {
        ESP_LOGW(TAG, "    ⚠️  Faults:");
        if (diag.overcurrent) ESP_LOGW(TAG, "      - Over-current");
        if (diag.short_to_ground) ESP_LOGW(TAG, "      - Short to Ground");
        if (diag.open_load) ESP_LOGW(TAG, "      - Open Load");
        if (diag.over_temperature) ESP_LOGW(TAG, "      - Over-temperature");
        if (diag.open_load_short_ground) ESP_LOGW(TAG, "      - Open Load/Short to Ground");
    }
    
    if (diag.ot_warning || diag.current_regulation_warning || 
        diag.pwm_regulation_warning || diag.olsg_warning) {
        ESP_LOGW(TAG, "    ⚠️  Warnings:");
        if (diag.ot_warning) ESP_LOGW(TAG, "      - Over-temperature Warning");
        if (diag.current_regulation_warning) ESP_LOGW(TAG, "      - Current Regulation Warning");
        if (diag.pwm_regulation_warning) ESP_LOGW(TAG, "      - PWM Regulation Warning");
        if (diag.olsg_warning) ESP_LOGW(TAG, "      - OLSG Warning");
    }
    
    if (!diag.overcurrent && !diag.short_to_ground && !diag.open_load && 
        !diag.over_temperature && !diag.open_load_short_ground &&
        !diag.ot_warning && !diag.current_regulation_warning && 
        !diag.pwm_regulation_warning && !diag.olsg_warning) {
        ESP_LOGI(TAG, "    ✅ Status: Normal");
    }
}

/**
 * @brief Print device-level telemetry
 */
static void print_device_telemetry() noexcept {
    if (!g_driver) {
        return;
    }
    
    // Get device status
    DeviceStatus status{};
    if (auto result = g_driver->GetDeviceStatus(); result) {
        status = *result;
    }
    
    // Get voltages
    uint16_t vbat_mv = 0;
    if (auto result = g_driver->GetVbatVoltage(); result) {
        vbat_mv = *result;
    }
    
    uint16_t vio_mv = 0;
    if (auto result = g_driver->GetVioVoltage(); result) {
        vio_mv = *result;
    }
    
    // Get fault status with automatic fault reporting enabled
    bool fault_pin = false;
    if (auto result = g_driver->IsFault(true); result) {
        fault_pin = *result;
        // Note: IsFault(true) automatically calls PrintAllFaults() when fault is detected
    }
    
    ESP_LOGI(TAG, "  Device Status:");
    ESP_LOGI(TAG, "    Mode: %s | Init: %s | Fault Pin: %s",
             status.config_mode ? "Config" : "Mission",
             status.init_done ? "Done" : "Pending",
             fault_pin ? "FAULT" : "OK");
    ESP_LOGI(TAG, "    VBAT: %u mV | VIO: %u mV", vbat_mv, vio_mv);
    
    if (status.any_fault) {
        ESP_LOGW(TAG, "    ⚠️  Device Faults:");
        if (status.vbat_uv) ESP_LOGW(TAG, "      - VBAT Undervoltage");
        if (status.vbat_ov) ESP_LOGW(TAG, "      - VBAT Overvoltage");
        if (status.vio_uv) ESP_LOGW(TAG, "      - VIO Undervoltage");
        if (status.vio_ov) ESP_LOGW(TAG, "      - VIO Overvoltage");
        if (status.vdd_uv) ESP_LOGW(TAG, "      - VDD Undervoltage");
        if (status.vdd_ov) ESP_LOGW(TAG, "      - VDD Overvoltage");
        if (status.ot_warning) ESP_LOGW(TAG, "      - Over-temperature Warning");
        if (status.ot_error) ESP_LOGW(TAG, "      - Over-temperature Error");
        if (status.clock_fault) ESP_LOGW(TAG, "      - Clock Fault");
        if (status.spi_wd_error) ESP_LOGW(TAG, "      - SPI Watchdog Error");
    }
}

/**
 * @brief Update solenoid currents based on ADC reading
 */
static void update_solenoid_currents() noexcept {
    if (!g_driver) {
        return;
    }
    
    // Read ADC percentage
    float adc_percentage = read_adc_percentage();
    float voltage_mv = read_adc_voltage_mv();
    
    // Calculate currents
    uint16_t current1_ma = calculate_current_from_percentage<Solenoid1Config>(adc_percentage);
    uint16_t current2_ma = calculate_current_from_percentage<Solenoid2Config>(adc_percentage);
    
    // Update Solenoid 1 (Single Channel)
    if (auto result = g_driver->SetCurrentSetpoint(
            Solenoid1Config::CHANNEL, 
            current1_ma, 
            Solenoid1Config::PARALLEL_MODE); !result) {
        ESP_LOGW(TAG, "Failed to set current for Solenoid 1");
    }
    
    // Update Solenoid 2 (Parallel Pair)
    if (auto result = g_driver->SetCurrentSetpoint(
            Solenoid2Config::PRIMARY_CHANNEL, 
            current2_ma, 
            Solenoid2Config::PARALLEL_MODE); !result) {
        ESP_LOGW(TAG, "Failed to set current for Solenoid 2");
    }
}

/**
 * @brief Enable/disable solenoids
 */
static void set_solenoids_enabled(bool enabled) noexcept {
    if (!g_driver) {
        return;
    }
    
    if (enabled) {
        // Enable Solenoid 1
        if (auto result = g_driver->EnableChannel(Solenoid1Config::CHANNEL, true); !result) {
            ESP_LOGE(TAG, "Failed to enable Solenoid 1");
            return;
        }
        
        // Enable Solenoid 2 (both channels in parallel pair)
        // Note: In parallel mode, enabling the primary channel enables both
        if (auto result = g_driver->EnableChannel(Solenoid2Config::PRIMARY_CHANNEL, true); !result) {
            ESP_LOGE(TAG, "Failed to enable Solenoid 2");
            return;
        }
        
        ESP_LOGI(TAG, "✅ Solenoids enabled");
    } else {
        // Disable both
        if (auto result = g_driver->EnableChannel(Solenoid1Config::CHANNEL, false); !result) {
            ESP_LOGE(TAG, "Failed to disable Solenoid 1");
        }
        if (auto result = g_driver->EnableChannel(Solenoid2Config::PRIMARY_CHANNEL, false); !result) {
            ESP_LOGE(TAG, "Failed to disable Solenoid 2");
        }
        
        ESP_LOGI(TAG, "✅ Solenoids disabled");
    }
}

//=============================================================================
// MAIN APPLICATION
//=============================================================================

extern "C" void app_main() {
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║         TLE92466ED SOLENOID CONTROL TEST - ESP32-C6                          ║");
    ESP_LOGI(TAG, "║                    Real Hardware Solenoid Testing                            ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "Driver version: %s", tle92466ed::GetDriverVersion());
    
    ESP_LOGI(TAG, "Target: %s", CONFIG_IDF_TARGET);
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Initialize HAL
    ESP_LOGI(TAG, "Initializing HAL...");
    g_hal = CreateEsp32Tle92466edSpiBus();
    if (!g_hal) {
        ESP_LOGE(TAG, "Failed to create HAL instance");
        return;
    }
    
    if (auto result = g_hal->Init(); !result) {
        ESP_LOGE(TAG, "Failed to initialize HAL");
        return;
    }
    ESP_LOGI(TAG, "✅ HAL initialized");
    
    // Initialize Driver
    ESP_LOGI(TAG, "Initializing TLE92466ED driver...");
    g_driver = new tle92466ed::Driver<Esp32Tle92466edSpiBus>(*g_hal);
    if (!g_driver) {
        ESP_LOGE(TAG, "Failed to create driver instance");
        return;
    }
    
    if (auto result = g_driver->Init(); !result) {
        ESP_LOGE(TAG, "Failed to initialize driver");
        return;
    }
    ESP_LOGI(TAG, "✅ Driver initialized");
    
    // Initialize ADC
    if (!initialize_adc()) {
        ESP_LOGE(TAG, "Failed to initialize ADC");
        return;
    }
    
    // Initialize Solenoids
    if (!initialize_solenoids()) {
        ESP_LOGE(TAG, "Failed to initialize solenoids");
        return;
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║                    SOLENOID CONTROL TEST STARTED                             ║");
    ESP_LOGI(TAG, "╠══════════════════════════════════════════════════════════════════════════════╣");
    ESP_LOGI(TAG, "║ Control: Adjust ADC input (GPIO%d) to control solenoid currents              ║", 
             ADCConfig::GPIO_PIN);
    ESP_LOGI(TAG, "║ Range: 0-3.3V maps to 0-100%% current for both solenoids                      ║");
    ESP_LOGI(TAG, "║ Solenoid 1: %s (%u-%u mA)                                                    ║",
             ToString(Solenoid1Config::CHANNEL),
             Solenoid1Config::MIN_CURRENT_MA,
             Solenoid1Config::MAX_CURRENT_MA);
    ESP_LOGI(TAG, "║ Solenoid 2: %s Parallel (%u-%u mA)                                          ║",
             ToString(Solenoid2Config::PAIR),
             Solenoid2Config::MIN_CURRENT_MA,
             Solenoid2Config::MAX_CURRENT_MA);
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    
    // Enable solenoids
    set_solenoids_enabled(true);
    
    // Main control loop
    ESP_LOGI(TAG, "Starting control loop (updating every 100ms)...");
    ESP_LOGI(TAG, "Telemetry will be displayed every 1 second");
    ESP_LOGI(TAG, "Press reset to stop");
    ESP_LOGI(TAG, "");
    
    const TickType_t update_interval = pdMS_TO_TICKS(100); // 100ms update rate
    const TickType_t telemetry_interval = pdMS_TO_TICKS(1000); // 1 second telemetry update
    
    TickType_t last_telemetry_time = xTaskGetTickCount();
    
    while (true) {
        // Update currents based on ADC
        update_solenoid_currents();
        
        // Print comprehensive telemetry every second
        TickType_t current_time = xTaskGetTickCount();
        if ((current_time - last_telemetry_time) >= telemetry_interval) {
            last_telemetry_time = current_time;
            
            // Read ADC
            float adc_percentage = read_adc_percentage();
            float voltage_mv = read_adc_voltage_mv();
            
            // Get current setpoints
            uint16_t setpoint1_ma = 0;
            uint16_t setpoint2_ma = 0;
            if (auto result = g_driver->GetCurrentSetpoint(Solenoid1Config::CHANNEL, false); result) {
                setpoint1_ma = *result;
            }
            if (auto result = g_driver->GetCurrentSetpoint(Solenoid2Config::PRIMARY_CHANNEL, true); result) {
                setpoint2_ma = *result;
            }
            
            ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
            ESP_LOGI(TAG, "║                        REAL-TIME TELEMETRY                                    ║");
            ESP_LOGI(TAG, "╠══════════════════════════════════════════════════════════════════════════════╣");
            ESP_LOGI(TAG, "  ADC Input: %.1f mV (%.1f%%)", voltage_mv, adc_percentage);
            ESP_LOGI(TAG, "  Target Currents: Solenoid 1=%u mA, Solenoid 2=%u mA", setpoint1_ma, setpoint2_ma);
            ESP_LOGI(TAG, "");
            
            // Print device telemetry
            print_device_telemetry();
            ESP_LOGI(TAG, "");
            
            // Print solenoid telemetry
            print_solenoid_telemetry("Solenoid 1", Solenoid1Config::CHANNEL, Solenoid1Config::PARALLEL_MODE);
            ESP_LOGI(TAG, "");
            print_solenoid_telemetry("Solenoid 2", Solenoid2Config::PRIMARY_CHANNEL, Solenoid2Config::PARALLEL_MODE);
            
            ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");
            ESP_LOGI(TAG, "");
        }
        
        vTaskDelay(update_interval);
    }
    
    // Cleanup (never reached, but good practice)
    set_solenoids_enabled(false);
    
    if (g_adc_cali_handle) {
        adc_cali_delete_scheme_curve_fitting(g_adc_cali_handle);
        g_adc_cali_handle = nullptr;
    }
    
    if (g_adc_handle) {
        adc_oneshot_del_unit(g_adc_handle);
        g_adc_handle = nullptr;
    }
    
    if (g_driver) {
        delete g_driver;
        g_driver = nullptr;
    }
    
    g_hal.reset();
}

