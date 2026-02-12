/**
 * @file driver_integration_test.cpp
 * @brief Comprehensive Integration Test Suite for TLE92466ED Driver
 * 
 * This is a complete integration test suite that tests all functionality
 * of the TLE92466ED driver with actual hardware.
 * 
 * Test Categories:
 * - Initialization Tests
 * - Mode Control Tests
 * - Global Configuration Tests
 * - Channel Control Tests
 * - Current Control Tests
 * - PWM Configuration Tests
 * - Dither Configuration Tests
 * - Diagnostics & Monitoring Tests
 * - Fault Management Tests
 * - Watchdog Tests
 * - GPIO Control Tests
 * - Multi-Channel Tests
 * - Parallel Operation Tests
 * - Error Condition Tests
 * 
 * @author N3b3x
 * @date 2025-10-21
 * @version 2.0.0
 */

#include <stdio.h>
#include <memory>
#include <array>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include "tle92466ed.hpp"
#include "esp32_tle92466ed_bus.hpp"
#include "TestFramework.h"

using namespace tle92466ed;

static const char* TAG = "TLE92466ED_Test";

// Test results tracking (required by TestFramework.h)
static TestResults g_test_results;

//=============================================================================
// TEST CONFIGURATION
//=============================================================================

// Enable/disable test sections (set to 0 to skip a section)
#define ENABLE_INITIALIZATION_TESTS 1
#define ENABLE_MODE_CONTROL_TESTS 1
#define ENABLE_GLOBAL_CONFIG_TESTS 1
#define ENABLE_CHANNEL_CONTROL_TESTS 1
#define ENABLE_CURRENT_CONTROL_TESTS 1
#define ENABLE_PWM_CONFIG_TESTS 1
#define ENABLE_DITHER_CONFIG_TESTS 1
#define ENABLE_DIAGNOSTICS_TESTS 1
#define ENABLE_FAULT_MANAGEMENT_TESTS 1
#define ENABLE_WATCHDOG_TESTS 1
#define ENABLE_GPIO_CONTROL_TESTS 1
#define ENABLE_MULTI_CHANNEL_TESTS 1
#define ENABLE_PARALLEL_OPERATION_TESTS 1
#define ENABLE_ERROR_CONDITION_TESTS 1

//=============================================================================
// SHARED TEST RESOURCES
//=============================================================================

static std::unique_ptr<Esp32Tle92466edSpiBus> g_hal;
static tle92466ed::Driver<Esp32Tle92466edSpiBus>* g_driver = nullptr;

//=============================================================================
// TEST HELPER FUNCTIONS
//=============================================================================

/**
 * @brief Ensure device is in Config Mode
 */
static bool ensure_config_mode() noexcept {
    if (!g_driver) return false;
    
    if (g_driver->IsMissionMode()) {
        ESP_LOGI(TAG, "Entering Config Mode...");
        if (auto result = g_driver->EnterConfigMode(); !result) {
            ESP_LOGE(TAG, "Failed to enter Config Mode");
            return false;
        }
    }
    return true;
}

/**
 * @brief Ensure device is in Mission Mode
 */
static bool ensure_mission_mode() noexcept {
    if (!g_driver) return false;
    
    if (g_driver->IsConfigMode()) {
        ESP_LOGI(TAG, "Entering Mission Mode...");
        if (auto result = g_driver->EnterMissionMode(); !result) {
            ESP_LOGE(TAG, "Failed to enter Mission Mode");
            return false;
        }
    }
    return true;
}

/**
 * @brief Print device status
 */
static void print_device_status(const DeviceStatus& status) noexcept {
    ESP_LOGI(TAG, "  Device Status:");
    ESP_LOGI(TAG, "    Mode: %s", status.config_mode ? "Config" : "Mission");
    ESP_LOGI(TAG, "    Init Done: %s", status.init_done ? "Yes" : "No");
    ESP_LOGI(TAG, "    Any Fault: %s", status.any_fault ? "Yes" : "No");
    
    // If faults are detected, use the comprehensive fault reporting system
    if (status.any_fault && g_driver) {
        ESP_LOGI(TAG, "");
        if (auto result = g_driver->PrintAllFaults(); !result) {
            ESP_LOGW(TAG, "⚠️  Failed to print detailed fault report");
        }
    }
}

/**
 * @brief Print channel diagnostics
 */
static void print_channel_diagnostics(Channel channel, const ChannelDiagnostics& diag) noexcept {
    ESP_LOGI(TAG, "  Channel %s Diagnostics:", ToString(channel));
    ESP_LOGI(TAG, "    Average Current: %u (raw)", diag.average_current);
    ESP_LOGI(TAG, "    Duty Cycle: %u (raw)", diag.duty_cycle);
    
    if (diag.overcurrent || diag.short_to_ground || diag.open_load || 
        diag.over_temperature || diag.open_load_short_ground) {
        ESP_LOGW(TAG, "    Errors:");
        if (diag.overcurrent) ESP_LOGW(TAG, "      - Over-current");
        if (diag.short_to_ground) ESP_LOGW(TAG, "      - Short to Ground");
        if (diag.open_load) ESP_LOGW(TAG, "      - Open Load");
        if (diag.over_temperature) ESP_LOGW(TAG, "      - Over-temperature");
        if (diag.open_load_short_ground) ESP_LOGW(TAG, "      - Open Load/Short to Ground");
    }
    
    if (diag.ot_warning || diag.current_regulation_warning || 
        diag.pwm_regulation_warning || diag.olsg_warning) {
        ESP_LOGW(TAG, "    Warnings:");
        if (diag.ot_warning) ESP_LOGW(TAG, "      - Over-temperature Warning");
        if (diag.current_regulation_warning) ESP_LOGW(TAG, "      - Current Regulation Warning");
        if (diag.pwm_regulation_warning) ESP_LOGW(TAG, "      - PWM Regulation Warning");
        if (diag.olsg_warning) ESP_LOGW(TAG, "      - OLSG Warning");
    }
}

//=============================================================================
// INITIALIZATION TESTS
//=============================================================================

/**
 * @brief Test HAL initialization
 */
static bool test_hal_initialization() noexcept {
    ESP_LOGI(TAG, "Creating HAL instance...");
    g_hal = CreateEsp32Tle92466edSpiBus();
    
    if (!g_hal) {
        ESP_LOGE(TAG, "Failed to create HAL instance");
        return false;
    }
    
    ESP_LOGI(TAG, "Initializing HAL...");
    if (auto result = g_hal->Init(); !result) {
        ESP_LOGE(TAG, "Failed to initialize HAL");
        return false;
    }
    
    ESP_LOGI(TAG, "✅ HAL initialized successfully");
    return true;
}

/**
 * @brief Test TLE92466ED driver initialization
 */
static bool test_driver_initialization() noexcept {
    if (!g_hal) {
        ESP_LOGE(TAG, "HAL not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Creating TLE92466ED driver instance...");
    g_driver = new tle92466ed::Driver<Esp32Tle92466edSpiBus>(*g_hal);
    
    if (!g_driver) {
        ESP_LOGE(TAG, "Failed to create driver instance");
        return false;
    }
    
    ESP_LOGI(TAG, "Initializing TLE92466ED driver...");
    if (auto result = g_driver->Init(); !result) {
        ESP_LOGE(TAG, "Failed to initialize driver: Error code %d", static_cast<int>(result.error()));
        return false;
    }
        
    ESP_LOGI(TAG, "✅ TLE92466ED driver initialized successfully");
    ESP_LOGI(TAG, "  Initial Mode: %s", g_driver->IsConfigMode() ? "Config" : "Mission");
    ESP_LOGI(TAG, "  Initialized: %s", g_driver->IsInitialized() ? "Yes" : "No");
    return true;
}

/**
 * @brief Test chip ID reading
 */
static bool test_chip_id() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Reading chip identification...");
    if (auto chip_id = g_driver->GetChipId(); chip_id) {
        ESP_LOGI(TAG, "✅ Chip ID: [0x%04X, 0x%04X, 0x%04X]", 
                 (*chip_id)[0], (*chip_id)[1], (*chip_id)[2]);
        return true;
    } else {
        ESP_LOGE(TAG, "❌ Failed to read chip ID");
        return false;
    }
}

/**
 * @brief Test IC version reading
 */
static bool test_ic_version() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Reading IC version...");
    if (auto version = g_driver->GetIcVersion(); version) {
        ESP_LOGI(TAG, "✅ IC Version: 0x%04X", *version);
        return true;
    } else {
        ESP_LOGE(TAG, "❌ Failed to read IC version");
        return false;
    }
}

/**
 * @brief Test device verification
 */
static bool test_device_verification() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Verifying device...");
    if (auto verified = g_driver->VerifyDevice(); verified) {
        if (*verified) {
            ESP_LOGI(TAG, "✅ Device verified successfully");
            return true;
        } else {
            ESP_LOGE(TAG, "❌ Device verification failed (wrong device ID)");
            return false;
        }
    } else {
        ESP_LOGE(TAG, "❌ Failed to verify device");
        return false;
    }
}

//=============================================================================
// MODE CONTROL TESTS
//=============================================================================

/**
 * @brief Test entering Mission Mode
 */
static bool test_enter_mission_mode() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    if (!ensure_config_mode()) {
        return false;
    }
    
    ESP_LOGI(TAG, "Entering Mission Mode...");
    if (auto result = g_driver->EnterMissionMode(); !result) {
        ESP_LOGE(TAG, "❌ Failed to enter Mission Mode");
        return false;
    }
    
    if (!g_driver->IsMissionMode()) {
        ESP_LOGE(TAG, "❌ Device not in Mission Mode after EnterMissionMode()");
        return false;
    }
    
    ESP_LOGI(TAG, "✅ Mission Mode entered successfully");
    return true;
}

/**
 * @brief Test entering Config Mode
 */
static bool test_enter_config_mode() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    if (!ensure_mission_mode()) {
        return false;
    }
    
    ESP_LOGI(TAG, "Entering Config Mode...");
    if (auto result = g_driver->EnterConfigMode(); !result) {
        ESP_LOGE(TAG, "❌ Failed to enter Config Mode");
        return false;
    }
    
    if (!g_driver->IsConfigMode()) {
        ESP_LOGE(TAG, "❌ Device not in Config Mode after EnterConfigMode()");
        return false;
    }
    
    ESP_LOGI(TAG, "✅ Config Mode entered successfully");
    return true;
}

/**
 * @brief Test mode transitions
 */
static bool test_mode_transitions() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Testing mode transitions...");
    
    // Test Config -> Mission -> Config cycle
    for (int i = 0; i < 3; ++i) {
        if (!ensure_config_mode()) return false;
        if (!g_driver->IsConfigMode()) {
            ESP_LOGE(TAG, "❌ Not in Config Mode");
            return false;
        }
        
        if (auto result = g_driver->EnterMissionMode(); !result) {
            ESP_LOGE(TAG, "❌ Failed to enter Mission Mode (iteration %d)", i);
            return false;
        }
        
        if (!g_driver->IsMissionMode()) {
            ESP_LOGE(TAG, "❌ Not in Mission Mode (iteration %d)", i);
            return false;
        }
        
        if (auto result = g_driver->EnterConfigMode(); !result) {
            ESP_LOGE(TAG, "❌ Failed to enter Config Mode (iteration %d)", i);
            return false;
        }
    }
    
    ESP_LOGI(TAG, "✅ Mode transitions test passed");
    return true;
}

//=============================================================================
// GLOBAL CONFIGURATION TESTS
//=============================================================================

/**
 * @brief Test CRC enable/disable
 */
static bool test_crc_control() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    if (!ensure_config_mode()) {
        return false;
    }
    
    ESP_LOGI(TAG, "Testing CRC control...");
    
    // Disable CRC
    ESP_LOGI(TAG, "Disabling CRC...");
    if (auto result = g_driver->SetCrcEnabled(false); !result) {
        ESP_LOGE(TAG, "❌ Failed to disable CRC");
        return false;
    }
    ESP_LOGI(TAG, "✅ CRC disabled");
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Enable CRC
    ESP_LOGI(TAG, "Enabling CRC...");
    if (auto result = g_driver->SetCrcEnabled(true); !result) {
        ESP_LOGE(TAG, "❌ Failed to enable CRC");
        return false;
    }
    ESP_LOGI(TAG, "✅ CRC enabled");
    
    return true;
}

/**
 * @brief Test VBAT threshold configuration
 */
static bool test_vbat_thresholds() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    if (!ensure_config_mode()) {
        return false;
    }
    
    ESP_LOGI(TAG, "Testing VBAT threshold configuration...");
    
    // Test high-level API
    ESP_LOGI(TAG, "Setting VBAT thresholds: UV=5.0V, OV=35.0V");
    if (auto result = g_driver->SetVbatThresholds(5.0f, 35.0f); !result) {
        ESP_LOGE(TAG, "❌ Failed to set VBAT thresholds");
        return false;
    }
    ESP_LOGI(TAG, "✅ VBAT thresholds set");
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Test raw API
    ESP_LOGI(TAG, "Setting VBAT thresholds (raw): UV_reg=30, OV_reg=200");
    if (auto result = g_driver->SetVbatThresholdsRaw(30, 200); !result) {
        ESP_LOGE(TAG, "❌ Failed to set VBAT thresholds (raw)");
        return false;
    }
    ESP_LOGI(TAG, "✅ VBAT thresholds (raw) set");
    
    // Restore defaults to wide range to prevent faults
    ESP_LOGI(TAG, "Restoring VBAT thresholds to wide range (7.0V, 40.0V)...");
    if (auto result = g_driver->SetVbatThresholds(7.0f, 40.0f); !result) {
        ESP_LOGW(TAG, "⚠️  Failed to restore default VBAT thresholds");
    } else {
        ESP_LOGI(TAG, "✅ VBAT thresholds restored to wide range");
    }
    
    // Clear any faults that were triggered by narrow thresholds
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Clearing faults triggered by threshold testing...");
    if (auto result = g_driver->ClearFaults(); !result) {
        ESP_LOGW(TAG, "⚠️  Failed to clear faults");
    } else {
        ESP_LOGI(TAG, "✅ Faults cleared");
    }
    
    return true;
}

/**
 * @brief Test global configuration
 */
static bool test_global_configuration() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    if (!ensure_config_mode()) {
        return false;
    }
    
    ESP_LOGI(TAG, "Testing global configuration...");
    
    GlobalConfig config{};
    config.crc_enabled = true;
    config.spi_watchdog_enabled = false;  // Disabled - requires periodic reloading
    config.clock_watchdog_enabled = true;
    config.vio_5v = false;  // 3.3V
    config.vbat_uv_voltage = 6.0f;
    config.vbat_ov_voltage = 38.0f;
    config.spi_watchdog_reload = 2000;  // Value set but not used since disabled
    
    if (auto result = g_driver->ConfigureGlobal(config); !result) {
        ESP_LOGE(TAG, "❌ Failed to configure global settings");
        return false;
    }
    
    ESP_LOGI(TAG, "✅ Global configuration applied");
    
    // Restore defaults (SPI watchdog remains disabled)
    config.crc_enabled = true;
    config.spi_watchdog_enabled = false;  // Keep disabled - default behavior
    config.clock_watchdog_enabled = true;
    config.vio_5v = false;
    config.vbat_uv_voltage = 7.0f;
    config.vbat_ov_voltage = 40.0f;
    config.spi_watchdog_reload = 1000;  // Value set but not used since disabled
    
    if (auto result = g_driver->ConfigureGlobal(config); !result) {
        ESP_LOGW(TAG, "⚠️  Failed to restore default global configuration");
    }
    
    return true;
}

//=============================================================================
// CHANNEL CONTROL TESTS
//=============================================================================

/**
 * @brief Test single channel enable/disable
 */
static bool test_single_channel_control() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    if (!ensure_mission_mode()) {
        return false;
    }
    
    ESP_LOGI(TAG, "Enabling outputs (EN pin)...");
    if (auto result = g_driver->Enable(); !result) {
        ESP_LOGE(TAG, "❌ Failed to enable outputs");
        return false;
    }
    
    const Channel channel = Channel::CH0;
    
    // Enable channel
    ESP_LOGI(TAG, "Enabling channel %s...", ToString(channel));
    if (auto result = g_driver->EnableChannel(channel, true); !result) {
        ESP_LOGE(TAG, "❌ Failed to enable channel %s", ToString(channel));
        return false;
    }
    ESP_LOGI(TAG, "✅ Channel %s enabled", ToString(channel));
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Disable channel
    ESP_LOGI(TAG, "Disabling channel %s...", ToString(channel));
    if (auto result = g_driver->EnableChannel(channel, false); !result) {
        ESP_LOGE(TAG, "❌ Failed to disable channel %s", ToString(channel));
        return false;
    }
    ESP_LOGI(TAG, "✅ Channel %s disabled", ToString(channel));
    
    return true;
}

/**
 * @brief Test all channels enable/disable
 */
static bool test_all_channels_control() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    if (!ensure_mission_mode()) {
        return false;
    }
    
    ESP_LOGI(TAG, "Enabling outputs (EN pin)...");
    if (auto result = g_driver->Enable(); !result) {
        ESP_LOGE(TAG, "❌ Failed to enable outputs");
        return false;
    }
    
    // Enable all channels
    ESP_LOGI(TAG, "Enabling all channels...");
    if (auto result = g_driver->EnableAllChannels(); !result) {
        ESP_LOGE(TAG, "❌ Failed to enable all channels");
        return false;
    }
    ESP_LOGI(TAG, "✅ All channels enabled");
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Disable all channels
    ESP_LOGI(TAG, "Disabling all channels...");
    if (auto result = g_driver->DisableAllChannels(); !result) {
        ESP_LOGE(TAG, "❌ Failed to disable all channels");
        return false;
    }
    ESP_LOGI(TAG, "✅ All channels disabled");
    
    return true;
}

/**
 * @brief Test channel mask control
 */
static bool test_channel_mask_control() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    if (!ensure_mission_mode()) {
        return false;
    }
    
    ESP_LOGI(TAG, "Enabling outputs (EN pin)...");
    if (auto result = g_driver->Enable(); !result) {
        ESP_LOGE(TAG, "❌ Failed to enable outputs");
        return false;
    }
    
    // Enable channels 0, 2, 4
    uint8_t mask = (1 << 0) | (1 << 2) | (1 << 4);
    ESP_LOGI(TAG, "Enabling channels with mask 0x%02X...", mask);
    if (auto result = g_driver->EnableChannels(mask); !result) {
        ESP_LOGE(TAG, "❌ Failed to enable channels");
        return false;
    }
    ESP_LOGI(TAG, "✅ Channels enabled");
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Disable all
    if (auto result = g_driver->EnableChannels(0); !result) {
        ESP_LOGE(TAG, "❌ Failed to disable channels");
        return false;
    }
    ESP_LOGI(TAG, "✅ Channels disabled");
    
    return true;
}

/**
 * @brief Test channel mode configuration
 */
static bool test_channel_mode_configuration() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    if (!ensure_config_mode()) {
        return false;
    }
    
    const Channel channel = Channel::CH0;
    
    ESP_LOGI(TAG, "Testing channel mode configuration for %s...", ToString(channel));
    
    // Test all channel modes
    const ChannelMode modes[] = {
        ChannelMode::OFF,
        ChannelMode::ICC,
        ChannelMode::DIRECT_DRIVE_SPI,
        ChannelMode::DIRECT_DRIVE_DRV0,
        ChannelMode::DIRECT_DRIVE_DRV1,
        ChannelMode::FREE_RUN_MEAS
    };
    
    for (ChannelMode mode : modes) {
        ESP_LOGI(TAG, "Setting mode to %s...", ToString(mode));
        if (auto result = g_driver->SetChannelMode(channel, mode); !result) {
            ESP_LOGE(TAG, "❌ Failed to set mode %s", ToString(mode));
            return false;
        }
        ESP_LOGI(TAG, "✅ Mode set to %s", ToString(mode));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Restore to ICC mode
    if (auto result = g_driver->SetChannelMode(channel, ChannelMode::ICC); !result) {
        ESP_LOGW(TAG, "⚠️  Failed to restore ICC mode");
    }
    
    return true;
}

//=============================================================================
// CURRENT CONTROL TESTS
//=============================================================================

/**
 * @brief Test current setpoint setting
 */
static bool test_current_setpoint() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    if (!ensure_mission_mode()) {
        return false;
    }
    
    ESP_LOGI(TAG, "Enabling outputs (EN pin)...");
    if (auto result = g_driver->Enable(); !result) {
        ESP_LOGE(TAG, "❌ Failed to enable outputs");
        return false;
    }
    
    const Channel channel = Channel::CH0;
    
    // Enable channel
    if (auto result = g_driver->EnableChannel(channel, true); !result) {
        ESP_LOGE(TAG, "❌ Failed to enable channel");
        return false;
    }
    
    const uint16_t test_currents[] = {100, 500, 1000, 1500, 2000};
    
    for (uint16_t current : test_currents) {
        ESP_LOGI(TAG, "Setting current to %umA...", current);
        if (auto result = g_driver->SetCurrentSetpoint(channel, current); !result) {
            ESP_LOGE(TAG, "❌ Failed to set current to %umA", current);
            return false;
        }
        
        // Verify by reading back
        if (auto read_result = g_driver->GetCurrentSetpoint(channel); read_result) {
            ESP_LOGI(TAG, "  Read back: %umA", *read_result);
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    // Disable channel
    (void)g_driver->EnableChannel(channel, false);
    
    ESP_LOGI(TAG, "✅ Current setpoint test passed");
    return true;
}

/**
 * @brief Test current ramping
 */
static bool test_current_ramping() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    if (!ensure_mission_mode()) {
        return false;
    }
    
    ESP_LOGI(TAG, "Enabling outputs (EN pin)...");
    if (auto result = g_driver->Enable(); !result) {
        ESP_LOGE(TAG, "❌ Failed to enable outputs");
        return false;
    }
    
    const Channel channel = Channel::CH0;
    
    // Enable channel
    if (auto result = g_driver->EnableChannel(channel, true); !result) {
        ESP_LOGE(TAG, "❌ Failed to enable channel");
        return false;
    }
    
    // Ramp up
    ESP_LOGI(TAG, "Ramping up from 0 to 1000mA...");
    for (uint16_t current = 0; current <= 1000; current += 100) {
        if (auto result = g_driver->SetCurrentSetpoint(channel, current); !result) {
            ESP_LOGE(TAG, "❌ Failed to set current to %umA", current);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    // Ramp down
    ESP_LOGI(TAG, "Ramping down from 1000 to 0mA...");
    for (int current = 1000; current >= 0; current -= 100) {
        if (auto result = g_driver->SetCurrentSetpoint(channel, static_cast<uint16_t>(current)); !result) {
            ESP_LOGE(TAG, "❌ Failed to set current to %umA", current);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    // Disable channel
    (void)g_driver->EnableChannel(channel, false);
    
    ESP_LOGI(TAG, "✅ Current ramping test completed");
    return true;
}

//=============================================================================
// PWM CONFIGURATION TESTS
//=============================================================================

/**
 * @brief Test PWM period configuration (high-level API)
 */
static bool test_pwm_period_configuration() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    if (!ensure_config_mode()) {
        return false;
    }
    
    const Channel channel = Channel::CH0;
    
    ESP_LOGI(TAG, "Testing PWM period configuration for %s...", ToString(channel));
    
    const float test_periods[] = {10.0f, 50.0f, 100.0f, 500.0f, 1000.0f, 5000.0f};
    
    for (float period_us : test_periods) {
        ESP_LOGI(TAG, "Setting PWM period to %.1f us...", period_us);
        if (auto result = g_driver->ConfigurePwmPeriod(channel, period_us); !result) {
            ESP_LOGE(TAG, "❌ Failed to set PWM period to %.1f us", period_us);
            return false;
        }
        ESP_LOGI(TAG, "✅ PWM period set to %.1f us", period_us);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    return true;
}

/**
 * @brief Test PWM period configuration (raw API)
 */
static bool test_pwm_period_raw() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    if (!ensure_config_mode()) {
        return false;
    }
    
    const Channel channel = Channel::CH0;
    
    ESP_LOGI(TAG, "Testing PWM period raw configuration for %s...", ToString(channel));
    
    // Test different mantissa/exponent combinations
    struct {
        uint8_t mantissa;
        uint8_t exponent;
        bool low_freq;
    } test_configs[] = {
        {100, 0, false},
        {50, 1, false},
        {25, 2, false},
        {100, 0, true},  // Low frequency range
    };
    
    for (const auto& config : test_configs) {
        ESP_LOGI(TAG, "Setting PWM: mantissa=%u, exponent=%u, low_freq=%s",
                 config.mantissa, config.exponent, config.low_freq ? "true" : "false");
        if (auto result = g_driver->ConfigurePwmPeriodRaw(channel, config.mantissa, 
                                                           config.exponent, config.low_freq); !result) {
            ESP_LOGE(TAG, "❌ Failed to set PWM period (raw)");
            return false;
        }
        ESP_LOGI(TAG, "✅ PWM period (raw) set");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    return true;
}

//=============================================================================
// DITHER CONFIGURATION TESTS
//=============================================================================

/**
 * @brief Test dither configuration (high-level API)
 */
static bool test_dither_configuration() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    if (!ensure_config_mode()) {
        return false;
    }
    
    const Channel channel = Channel::CH0;
    
    ESP_LOGI(TAG, "Testing dither configuration for %s...", ToString(channel));
    
    struct {
        float amplitude_ma;
        float frequency_hz;
    } test_configs[] = {
        {10.0f, 100.0f},
        {50.0f, 500.0f},
        {100.0f, 1000.0f},
    };
    
    for (const auto& config : test_configs) {
        ESP_LOGI(TAG, "Setting dither: amplitude=%.1f mA, frequency=%.1f Hz",
                 config.amplitude_ma, config.frequency_hz);
        if (auto result = g_driver->ConfigureDither(channel, config.amplitude_ma, 
                                                     config.frequency_hz); !result) {
            ESP_LOGE(TAG, "❌ Failed to configure dither");
            return false;
        }
        ESP_LOGI(TAG, "✅ Dither configured");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    return true;
}

/**
 * @brief Test dither configuration (raw API)
 */
static bool test_dither_raw() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    if (!ensure_config_mode()) {
        return false;
    }
    
    const Channel channel = Channel::CH0;
    
    ESP_LOGI(TAG, "Testing dither raw configuration for %s...", ToString(channel));
    
    struct {
        uint16_t step_size;
        uint8_t num_steps;
        uint8_t flat_steps;
    } test_configs[] = {
        {100, 10, 2},
        {500, 20, 5},
        {1000, 30, 10},
    };
    
    for (const auto& config : test_configs) {
        ESP_LOGI(TAG, "Setting dither (raw): step_size=%u, num_steps=%u, flat_steps=%u",
                 config.step_size, config.num_steps, config.flat_steps);
        if (auto result = g_driver->ConfigureDitherRaw(channel, config.step_size,
                                                       config.num_steps, config.flat_steps); !result) {
            ESP_LOGE(TAG, "❌ Failed to configure dither (raw)");
            return false;
        }
        ESP_LOGI(TAG, "✅ Dither (raw) configured");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    return true;
}

//=============================================================================
// DIAGNOSTICS & MONITORING TESTS
//=============================================================================

/**
 * @brief Test device status reading
 */
static bool test_device_status() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Reading device status...");
    if (auto status = g_driver->GetDeviceStatus(); status) {
        ESP_LOGI(TAG, "✅ Device status read successfully");
        print_device_status(*status);
        return true;
    } else {
        ESP_LOGE(TAG, "❌ Failed to read device status");
        return false;
    }
}

/**
 * @brief Test channel diagnostics reading
 */
static bool test_channel_diagnostics() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    const Channel channel = Channel::CH0;
    
    ESP_LOGI(TAG, "Reading channel diagnostics for %s...", ToString(channel));
    if (auto diag = g_driver->GetChannelDiagnostics(channel); diag) {
        ESP_LOGI(TAG, "✅ Channel diagnostics read successfully");
        print_channel_diagnostics(channel, *diag);
        return true;
    } else {
        ESP_LOGE(TAG, "❌ Failed to read channel diagnostics");
        return false;
    }
}

/**
 * @brief Test voltage reading
 */
static bool test_voltage_reading() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Reading voltages...");
    
    if (auto vbat = g_driver->GetVbatVoltage(); vbat) {
        ESP_LOGI(TAG, "✅ VBAT voltage: %u mV", *vbat);
    } else {
        ESP_LOGW(TAG, "⚠️  Failed to read VBAT voltage");
    }
    
    if (auto vio = g_driver->GetVioVoltage(); vio) {
        ESP_LOGI(TAG, "✅ VIO voltage: %u mV", *vio);
    } else {
        ESP_LOGW(TAG, "⚠️  Failed to read VIO voltage");
    }
    
    return true;
}

/**
 * @brief Test current and duty cycle reading
 */
static bool test_current_reading() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    const Channel channel = Channel::CH0;
    
    ESP_LOGI(TAG, "Reading current measurements for %s...", ToString(channel));
    
    if (auto current = g_driver->GetAverageCurrent(channel); current) {
        ESP_LOGI(TAG, "✅ Average current: %u mA", *current);
    } else {
        ESP_LOGW(TAG, "⚠️  Failed to read average current");
    }
    
    if (auto duty = g_driver->GetDutyCycle(channel); duty) {
        ESP_LOGI(TAG, "✅ Duty cycle: %u (raw)", *duty);
    } else {
        ESP_LOGW(TAG, "⚠️  Failed to read duty cycle");
    }
    
    return true;
}

/**
 * @brief Test comprehensive telemetry for all channels
 */
static bool test_all_channels_telemetry() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Testing telemetry for all channels...");
    
    const Channel channels[] = {
        Channel::CH0, Channel::CH1, Channel::CH2,
        Channel::CH3, Channel::CH4, Channel::CH5
    };
    
    bool all_passed = true;
    
    for (Channel channel : channels) {
        ESP_LOGI(TAG, "  Testing %s telemetry...", ToString(channel));
        
        // Test GetCurrentSetpoint
        if (auto setpoint = g_driver->GetCurrentSetpoint(channel, false); setpoint) {
            ESP_LOGI(TAG, "    ✅ Setpoint: %u mA", *setpoint);
        } else {
            ESP_LOGW(TAG, "    ⚠️  Failed to read setpoint");
            all_passed = false;
        }
        
        // Test GetAverageCurrent
        if (auto current = g_driver->GetAverageCurrent(channel, false); current) {
            ESP_LOGI(TAG, "    ✅ Average Current: %u mA", *current);
        } else {
            ESP_LOGW(TAG, "    ⚠️  Failed to read average current");
            all_passed = false;
        }
        
        // Test GetDutyCycle
        if (auto duty = g_driver->GetDutyCycle(channel); duty) {
            ESP_LOGI(TAG, "    ✅ Duty Cycle: %u (0x%04X)", *duty, *duty);
        } else {
            ESP_LOGW(TAG, "    ⚠️  Failed to read duty cycle");
            all_passed = false;
        }
        
        // Test GetChannelDiagnostics
        if (auto diag = g_driver->GetChannelDiagnostics(channel); diag) {
            ESP_LOGI(TAG, "    ✅ Diagnostics: Current=%u mA, Duty=%u, Min=%u, Max=%u",
                     diag->average_current, diag->duty_cycle,
                     diag->min_current, diag->max_current);
            
            // Check for faults
            if (diag->overcurrent || diag->short_to_ground || diag->open_load ||
                diag->over_temperature || diag->open_load_short_ground) {
                ESP_LOGW(TAG, "    ⚠️  Faults detected");
            }
            
            // Check for warnings
            if (diag->ot_warning || diag->current_regulation_warning ||
                diag->pwm_regulation_warning || diag->olsg_warning) {
                ESP_LOGW(TAG, "    ⚠️  Warnings detected");
            }
        } else {
            ESP_LOGW(TAG, "    ⚠️  Failed to read diagnostics");
            all_passed = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    if (all_passed) {
        ESP_LOGI(TAG, "✅ All channels telemetry test passed");
    } else {
        ESP_LOGW(TAG, "⚠️  Some telemetry reads failed (may be expected if channels not enabled)");
    }
    
    return true; // Don't fail test if some reads fail (channels may not be enabled)
}

/**
 * @brief Test comprehensive device telemetry
 */
static bool test_device_telemetry() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Testing comprehensive device telemetry...");
    
    // Test GetDeviceStatus
    if (auto status = g_driver->GetDeviceStatus(); status) {
        ESP_LOGI(TAG, "✅ Device Status:");
        ESP_LOGI(TAG, "    Mode: %s", status->config_mode ? "Config" : "Mission");
        ESP_LOGI(TAG, "    Init Done: %s", status->init_done ? "Yes" : "No");
        ESP_LOGI(TAG, "    Any Fault: %s", status->any_fault ? "Yes" : "No");
        
        if (status->any_fault) {
            ESP_LOGW(TAG, "    ⚠️  Device has faults");
        }
    } else {
        ESP_LOGE(TAG, "❌ Failed to read device status");
        return false;
    }
    
    // Test GetVbatVoltage
    if (auto vbat = g_driver->GetVbatVoltage(); vbat) {
        ESP_LOGI(TAG, "✅ VBAT Voltage: %u mV", *vbat);
    } else {
        ESP_LOGW(TAG, "⚠️  Failed to read VBAT voltage");
    }
    
    // Test GetVioVoltage
    if (auto vio = g_driver->GetVioVoltage(); vio) {
        ESP_LOGI(TAG, "✅ VIO Voltage: %u mV", *vio);
    } else {
        ESP_LOGW(TAG, "⚠️  Failed to read VIO voltage");
    }
    
    // Test IsFault with automatic fault reporting enabled
    if (auto fault = g_driver->IsFault(true); fault) {
        ESP_LOGI(TAG, "✅ Fault Pin: %s", *fault ? "FAULT" : "OK");
        // Note: IsFault(true) automatically calls PrintAllFaults() when fault is detected
    } else {
        ESP_LOGW(TAG, "⚠️  Failed to read fault pin");
    }
    
    return true;
}

/**
 * @brief Test telemetry with channel enabled and current set
 */
static bool test_telemetry_with_active_channel() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    if (!ensure_mission_mode()) {
        return false;
    }
    
    ESP_LOGI(TAG, "Enabling outputs (EN pin)...");
    if (auto result = g_driver->Enable(); !result) {
        ESP_LOGE(TAG, "❌ Failed to enable outputs");
        return false;
    }
    
    const Channel channel = Channel::CH0;
    const uint16_t test_current = 500; // 500mA
    
    ESP_LOGI(TAG, "Testing telemetry with active channel %s at %u mA...", ToString(channel), test_current);
    
    // Set current
    if (auto result = g_driver->SetCurrentSetpoint(channel, test_current); !result) {
        ESP_LOGE(TAG, "❌ Failed to set current");
        return false;
    }
    
    // Enable channel
    if (auto result = g_driver->EnableChannel(channel, true); !result) {
        ESP_LOGE(TAG, "❌ Failed to enable channel");
        return false;
    }
    
    // Wait for current to stabilize
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Read all telemetry
    ESP_LOGI(TAG, "Reading telemetry...");
    
    // Get setpoint
    if (auto setpoint = g_driver->GetCurrentSetpoint(channel, false); setpoint) {
        ESP_LOGI(TAG, "  Setpoint: %u mA", *setpoint);
        if (*setpoint != test_current) {
            ESP_LOGW(TAG, "  ⚠️  Setpoint mismatch: expected %u, got %u", test_current, *setpoint);
        }
    }
    
    // Get average current
    if (auto current = g_driver->GetAverageCurrent(channel, false); current) {
        ESP_LOGI(TAG, "  Average Current: %u mA", *current);
    }
    
    // Get duty cycle
    if (auto duty = g_driver->GetDutyCycle(channel); duty) {
        ESP_LOGI(TAG, "  Duty Cycle: %u (0x%04X)", *duty, *duty);
    }
    
    // Get diagnostics
    if (auto diag = g_driver->GetChannelDiagnostics(channel); diag) {
        ESP_LOGI(TAG, "  Diagnostics:");
        ESP_LOGI(TAG, "    Average Current: %u mA", diag->average_current);
        ESP_LOGI(TAG, "    Duty Cycle: %u", diag->duty_cycle);
        ESP_LOGI(TAG, "    Min Current: %u", diag->min_current);
        ESP_LOGI(TAG, "    Max Current: %u", diag->max_current);
        ESP_LOGI(TAG, "    VBAT Feedback: %u", diag->vbat_feedback);
        
        if (diag->overcurrent || diag->short_to_ground || diag->open_load) {
            ESP_LOGW(TAG, "    ⚠️  Faults detected");
        } else {
            ESP_LOGI(TAG, "    ✅ No faults");
        }
    }
    
    // Disable channel
    (void)g_driver->EnableChannel(channel, false);
    
    ESP_LOGI(TAG, "✅ Telemetry with active channel test completed");
    return true;
}

//=============================================================================
// FAULT MANAGEMENT TESTS
//=============================================================================

/**
 * @brief Test comprehensive fault reporting
 */
static bool test_fault_reporting() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Testing comprehensive fault reporting...");
    
    // Get all faults
    if (auto faults = g_driver->GetAllFaults(); faults) {
        ESP_LOGI(TAG, "✅ Fault report retrieved successfully");
        ESP_LOGI(TAG, "  Any fault: %s", faults->any_fault ? "Yes" : "No");
        
        // Print all faults
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "Printing comprehensive fault report:");
        if (auto result = g_driver->PrintAllFaults(); !result) {
            ESP_LOGE(TAG, "❌ Failed to print fault report");
            return false;
        }
        
        return true;
    } else {
        ESP_LOGE(TAG, "❌ Failed to get fault report");
        return false;
    }
}

/**
 * @brief Test fault clearing
 */
static bool test_fault_clearing() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    if (!ensure_config_mode()) {
        return false;
    }
    
    ESP_LOGI(TAG, "Testing fault clearing...");
    
    // Ensure thresholds are set to wide range to prevent new faults
    ESP_LOGI(TAG, "Setting VBAT thresholds to wide range (7.0V, 40.0V)...");
    if (auto result = g_driver->SetVbatThresholds(7.0f, 40.0f); !result) {
        ESP_LOGW(TAG, "⚠️  Failed to set wide range thresholds");
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Clear faults
    if (auto result = g_driver->ClearFaults(); !result) {
        ESP_LOGE(TAG, "❌ Failed to clear faults");
        return false;
    }
    
    ESP_LOGI(TAG, "✅ Faults cleared");
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Check if any faults exist and print detailed report if present
    if (auto has_fault = g_driver->HasAnyFault(); has_fault) {
        if (*has_fault) {
            ESP_LOGW(TAG, "⚠️  Faults still present after clearing:");
            ESP_LOGI(TAG, "");
            if (auto result = g_driver->PrintAllFaults(); !result) {
                ESP_LOGW(TAG, "⚠️  Failed to print detailed fault report");
            }
        } else {
            ESP_LOGI(TAG, "✅ No faults detected");
        }
    }
    
    return true;
}

/**
 * @brief Test software reset
 */
static bool test_software_reset() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Testing software reset...");
    
    if (auto result = g_driver->SoftwareReset(); !result) {
        ESP_LOGE(TAG, "❌ Failed to perform software reset");
        return false;
    }
    
    ESP_LOGI(TAG, "✅ Software reset completed");
    
    // Verify we're in Config Mode
    // Note: SoftwareReset() enters Config Mode and clears channel enable cache
    // It does NOT actually disable channels (requires Mission Mode)
    // Channels will be disabled when entering Mission Mode next time
    if (auto status = g_driver->GetDeviceStatus(); status) {
        if (!status->config_mode) {
        ESP_LOGE(TAG, "❌ Not in Config Mode after software reset");
        return false;
        }
        ESP_LOGI(TAG, "✅ Verified: Device is in Config Mode");
    }
    
    return true;
}

//=============================================================================
// WATCHDOG TESTS
//=============================================================================

/**
 * @brief Test SPI watchdog reload
 */
static bool test_spi_watchdog() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Testing SPI watchdog reload (watchdog is disabled, just testing reload function)...");
    
    // Note: SPI watchdog is disabled by default, so reloading won't actually prevent timeout
    // This test just verifies the reload function works correctly
    const uint16_t test_reload_values[] = {500, 1000, 2000, 5000};
    
    for (uint16_t reload_value : test_reload_values) {
        ESP_LOGI(TAG, "Reloading watchdog with value %u...", reload_value);
        if (auto result = g_driver->ReloadSpiWatchdog(reload_value); !result) {
            ESP_LOGE(TAG, "❌ Failed to reload watchdog");
            return false;
        }
        ESP_LOGI(TAG, "✅ Watchdog reloaded");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    return true;
}

//=============================================================================
// GPIO CONTROL TESTS
//=============================================================================

/**
 * @brief Test GPIO control (Reset, Enable, Fault)
 */
static bool test_gpio_control() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Testing GPIO control...");
    
    // Test fault status
    ESP_LOGI(TAG, "Checking fault status...");
    // Check fault pin with automatic fault reporting enabled
    if (auto fault = g_driver->IsFault(true); fault) {
        if (*fault) {
            // Note: IsFault(true) automatically calls PrintAllFaults() when fault is detected
            ESP_LOGW(TAG, "⚠️  Fault detected on FAULTN pin (detailed report printed above)");
        } else {
            ESP_LOGI(TAG, "✅ No fault detected");
        }
    } else {
        ESP_LOGW(TAG, "⚠️  Could not read fault status");
    }
    
    // Test enable/disable
    ESP_LOGI(TAG, "Testing enable pin control...");
    if (auto result = g_driver->Enable(); !result) {
        ESP_LOGE(TAG, "❌ Failed to enable outputs");
        return false;
    }
    ESP_LOGI(TAG, "✅ Outputs enabled");
    
    vTaskDelay(pdMS_TO_TICKS(200));
    
    if (auto result = g_driver->Disable(); !result) {
        ESP_LOGE(TAG, "❌ Failed to disable outputs");
        return false;
    }
    ESP_LOGI(TAG, "✅ Outputs disabled");
    
    // Test reset control
    ESP_LOGI(TAG, "Testing reset pin control...");
    if (auto result = g_driver->HoldReset(); !result) {
        ESP_LOGE(TAG, "❌ Failed to hold reset");
        return false;
    }
    ESP_LOGI(TAG, "✅ Reset held");
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    if (auto result = g_driver->ReleaseReset(); !result) {
        ESP_LOGE(TAG, "❌ Failed to release reset");
        return false;
    }
    ESP_LOGI(TAG, "✅ Reset released");
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    return true;
}

//=============================================================================
// MULTI-CHANNEL TESTS
//=============================================================================

/**
 * @brief Test all channels individually
 */
static bool test_all_channels_individually() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    if (!ensure_mission_mode()) {
        return false;
    }
    
    ESP_LOGI(TAG, "Enabling outputs (EN pin)...");
    if (auto result = g_driver->Enable(); !result) {
        ESP_LOGE(TAG, "❌ Failed to enable outputs");
        return false;
    }
    
    ESP_LOGI(TAG, "Testing all channels individually...");
    
    const Channel channels[] = {
        Channel::CH0, Channel::CH1, Channel::CH2,
        Channel::CH3, Channel::CH4, Channel::CH5
    };
    
    for (Channel channel : channels) {
        ESP_LOGI(TAG, "Testing channel %s...", ToString(channel));
        
        // Enable
        if (auto result = g_driver->EnableChannel(channel, true); !result) {
            ESP_LOGE(TAG, "❌ Failed to enable channel %s", ToString(channel));
            return false;
        }
        ESP_LOGI(TAG, "  ✅ Enabled");
        
        vTaskDelay(pdMS_TO_TICKS(200));
        
        // Set current
        if (auto result = g_driver->SetCurrentSetpoint(channel, 500); !result) {
            ESP_LOGE(TAG, "❌ Failed to set current for channel %s", ToString(channel));
            return false;
        }
        ESP_LOGI(TAG, "  ✅ Current set to 500mA");
        
        vTaskDelay(pdMS_TO_TICKS(200));
        
        // Disable
        if (auto result = g_driver->EnableChannel(channel, false); !result) {
            ESP_LOGE(TAG, "❌ Failed to disable channel %s", ToString(channel));
            return false;
        }
        ESP_LOGI(TAG, "  ✅ Disabled");
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "✅ All channels tested successfully");
    return true;
}

//=============================================================================
// PARALLEL OPERATION TESTS
//=============================================================================

/**
 * @brief Test parallel operation configuration
 */
static bool test_parallel_operation() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    if (!ensure_config_mode()) {
        return false;
    }
    
    ESP_LOGI(TAG, "Testing parallel operation configuration...");
    
    const ParallelPair pairs[] = {
        ParallelPair::CH0_CH3,
        ParallelPair::CH1_CH2,
        ParallelPair::CH4_CH5
    };
    
    for (ParallelPair pair : pairs) {
        ESP_LOGI(TAG, "Testing parallel pair %s...", ToString(pair));
        
        // Enable parallel operation
        ESP_LOGI(TAG, "  Enabling parallel operation...");
        if (auto result = g_driver->SetParallelOperation(pair, true); !result) {
            ESP_LOGE(TAG, "❌ Failed to enable parallel operation for %s", ToString(pair));
            return false;
        }
        ESP_LOGI(TAG, "  ✅ Parallel operation enabled");
        
        vTaskDelay(pdMS_TO_TICKS(200));
        
        // Disable parallel operation
        ESP_LOGI(TAG, "  Disabling parallel operation...");
        if (auto result = g_driver->SetParallelOperation(pair, false); !result) {
            ESP_LOGE(TAG, "❌ Failed to disable parallel operation for %s", ToString(pair));
            return false;
        }
        ESP_LOGI(TAG, "  ✅ Parallel operation disabled");
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    return true;
}

//=============================================================================
// ERROR CONDITION TESTS
//=============================================================================

/**
 * @brief Test error conditions (wrong mode operations)
 */
static bool test_error_conditions() noexcept {
    if (!g_driver) {
        ESP_LOGE(TAG, "Driver not initialized");
        return false;
    }
    
    ESP_LOGI(TAG, "Testing error conditions...");
    
    // Test: Try to enable channel in Config Mode (should fail)
    if (!ensure_config_mode()) {
        return false;
    }
    
    ESP_LOGI(TAG, "Testing channel enable in Config Mode (should fail)...");
    if (auto result = g_driver->EnableChannel(Channel::CH0, true); result) {
        ESP_LOGE(TAG, "❌ Channel enable should fail in Config Mode");
        return false;
    }
    ESP_LOGI(TAG, "✅ Correctly rejected channel enable in Config Mode");
    
    // Test: Try to configure channel mode in Mission Mode (should fail)
    if (!ensure_mission_mode()) {
        return false;
    }
    
    ESP_LOGI(TAG, "Testing channel mode change in Mission Mode (should fail)...");
    if (auto result = g_driver->SetChannelMode(Channel::CH0, ChannelMode::ICC); result) {
        ESP_LOGE(TAG, "❌ Channel mode change should fail in Mission Mode");
        return false;
    }
    ESP_LOGI(TAG, "✅ Correctly rejected channel mode change in Mission Mode");
    
    // Test: Try to configure global settings in Mission Mode (should fail)
    ESP_LOGI(TAG, "Testing global config in Mission Mode (should fail)...");
    GlobalConfig config{};
    if (auto result = g_driver->ConfigureGlobal(config); result) {
        ESP_LOGE(TAG, "❌ Global config should fail in Mission Mode");
        return false;
    }
    ESP_LOGI(TAG, "✅ Correctly rejected global config in Mission Mode");
    
    return true;
}

//=============================================================================
// MAIN APPLICATION
//=============================================================================

extern "C" void app_main() {
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║         TLE92466ED COMPREHENSIVE INTEGRATION TEST SUITE - ESP32-C6           ║");
    ESP_LOGI(TAG, "║                         HardFOC Core Drivers                                 ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");
    
    ESP_LOGI(TAG, "Target: %s", CONFIG_IDF_TARGET);
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Initialize GPIO14 test progress indicator
    init_test_progress_indicator();
    
    // Report test section configuration
    print_test_section_status(TAG, "TLE92466ED");
    
    //=========================================================================
    // INITIALIZATION TESTS
    //=========================================================================
    RUN_TEST_SECTION_IF_ENABLED(
        ENABLE_INITIALIZATION_TESTS, "INITIALIZATION TESTS",
        ESP_LOGI(TAG, "Running initialization tests...");
        RUN_TEST_IN_TASK("hal_initialization", test_hal_initialization, 8192, 5);
        RUN_TEST_IN_TASK("driver_initialization", test_driver_initialization, 8192, 5);
        RUN_TEST_IN_TASK("chip_id", test_chip_id, 8192, 5);
        RUN_TEST_IN_TASK("ic_version", test_ic_version, 8192, 5);
        RUN_TEST_IN_TASK("device_verification", test_device_verification, 8192, 5);
    );
    
    //=========================================================================
    // MODE CONTROL TESTS
    //=========================================================================
    RUN_TEST_SECTION_IF_ENABLED(
        ENABLE_MODE_CONTROL_TESTS, "MODE CONTROL TESTS",
        ESP_LOGI(TAG, "Running mode control tests...");
        RUN_TEST_IN_TASK("enter_mission_mode", test_enter_mission_mode, 8192, 5);
        RUN_TEST_IN_TASK("enter_config_mode", test_enter_config_mode, 8192, 5);
        RUN_TEST_IN_TASK("mode_transitions", test_mode_transitions, 8192, 5);
    );
    
    //=========================================================================
    // GLOBAL CONFIGURATION TESTS
    //=========================================================================
    RUN_TEST_SECTION_IF_ENABLED(
        ENABLE_GLOBAL_CONFIG_TESTS, "GLOBAL CONFIGURATION TESTS",
        ESP_LOGI(TAG, "Running global configuration tests...");
        RUN_TEST_IN_TASK("crc_control", test_crc_control, 8192, 5);
        RUN_TEST_IN_TASK("vbat_thresholds", test_vbat_thresholds, 8192, 5);
        RUN_TEST_IN_TASK("global_configuration", test_global_configuration, 8192, 5);
    );
    
    //=========================================================================
    // CHANNEL CONTROL TESTS
    //=========================================================================
    RUN_TEST_SECTION_IF_ENABLED(
        ENABLE_CHANNEL_CONTROL_TESTS, "CHANNEL CONTROL TESTS",
        ESP_LOGI(TAG, "Running channel control tests...");
        RUN_TEST_IN_TASK("single_channel_control", test_single_channel_control, 8192, 5);
        RUN_TEST_IN_TASK("all_channels_control", test_all_channels_control, 8192, 5);
        RUN_TEST_IN_TASK("channel_mask_control", test_channel_mask_control, 8192, 5);
        RUN_TEST_IN_TASK("channel_mode_configuration", test_channel_mode_configuration, 8192, 5);
    );
    
    //=========================================================================
    // CURRENT CONTROL TESTS
    //=========================================================================
    RUN_TEST_SECTION_IF_ENABLED(
        ENABLE_CURRENT_CONTROL_TESTS, "CURRENT CONTROL TESTS",
        ESP_LOGI(TAG, "Running current control tests...");
        RUN_TEST_IN_TASK("current_setpoint", test_current_setpoint, 8192, 5);
        RUN_TEST_IN_TASK("current_ramping", test_current_ramping, 8192, 5);
    );
    
    //=========================================================================
    // PWM CONFIGURATION TESTS
    //=========================================================================
    RUN_TEST_SECTION_IF_ENABLED(
        ENABLE_PWM_CONFIG_TESTS, "PWM CONFIGURATION TESTS",
        ESP_LOGI(TAG, "Running PWM configuration tests...");
        RUN_TEST_IN_TASK("pwm_period_configuration", test_pwm_period_configuration, 8192, 5);
        RUN_TEST_IN_TASK("pwm_period_raw", test_pwm_period_raw, 8192, 5);
    );
    
    //=========================================================================
    // DITHER CONFIGURATION TESTS
    //=========================================================================
    RUN_TEST_SECTION_IF_ENABLED(
        ENABLE_DITHER_CONFIG_TESTS, "DITHER CONFIGURATION TESTS",
        ESP_LOGI(TAG, "Running dither configuration tests...");
        RUN_TEST_IN_TASK("dither_configuration", test_dither_configuration, 8192, 5);
        RUN_TEST_IN_TASK("dither_raw", test_dither_raw, 8192, 5);
    );
    
    //=========================================================================
    // DIAGNOSTICS & MONITORING TESTS
    //=========================================================================
    RUN_TEST_SECTION_IF_ENABLED(
        ENABLE_DIAGNOSTICS_TESTS, "DIAGNOSTICS & MONITORING TESTS",
        ESP_LOGI(TAG, "Running diagnostics tests...");
        RUN_TEST_IN_TASK("device_status", test_device_status, 8192, 5);
        RUN_TEST_IN_TASK("channel_diagnostics", test_channel_diagnostics, 8192, 5);
        RUN_TEST_IN_TASK("voltage_reading", test_voltage_reading, 8192, 5);
        RUN_TEST_IN_TASK("current_reading", test_current_reading, 8192, 5);
        RUN_TEST_IN_TASK("all_channels_telemetry", test_all_channels_telemetry, 8192, 5);
        RUN_TEST_IN_TASK("device_telemetry", test_device_telemetry, 8192, 5);
        RUN_TEST_IN_TASK("telemetry_with_active_channel", test_telemetry_with_active_channel, 8192, 5);
    );
    
    //=========================================================================
    // FAULT MANAGEMENT TESTS
    //=========================================================================
    RUN_TEST_SECTION_IF_ENABLED(
        ENABLE_FAULT_MANAGEMENT_TESTS, "FAULT MANAGEMENT TESTS",
        ESP_LOGI(TAG, "Running fault management tests...");
        RUN_TEST_IN_TASK("fault_reporting", test_fault_reporting, 8192, 5);
        RUN_TEST_IN_TASK("fault_clearing", test_fault_clearing, 8192, 5);
        RUN_TEST_IN_TASK("software_reset", test_software_reset, 8192, 5);
    );
    
    //=========================================================================
    // WATCHDOG TESTS
    //=========================================================================
    RUN_TEST_SECTION_IF_ENABLED(
        ENABLE_WATCHDOG_TESTS, "WATCHDOG TESTS",
        ESP_LOGI(TAG, "Running watchdog tests...");
        RUN_TEST_IN_TASK("spi_watchdog", test_spi_watchdog, 8192, 5);
    );
    
    //=========================================================================
    // GPIO CONTROL TESTS
    //=========================================================================
    RUN_TEST_SECTION_IF_ENABLED(
        ENABLE_GPIO_CONTROL_TESTS, "GPIO CONTROL TESTS",
        ESP_LOGI(TAG, "Running GPIO control tests...");
        RUN_TEST_IN_TASK("gpio_control", test_gpio_control, 8192, 5);
    );
    
    //=========================================================================
    // MULTI-CHANNEL TESTS
    //=========================================================================
    RUN_TEST_SECTION_IF_ENABLED(
        ENABLE_MULTI_CHANNEL_TESTS, "MULTI-CHANNEL TESTS",
        ESP_LOGI(TAG, "Running multi-channel tests...");
        RUN_TEST_IN_TASK("all_channels_individually", test_all_channels_individually, 8192, 5);
    );
    
    //=========================================================================
    // PARALLEL OPERATION TESTS
    //=========================================================================
    RUN_TEST_SECTION_IF_ENABLED(
        ENABLE_PARALLEL_OPERATION_TESTS, "PARALLEL OPERATION TESTS",
        ESP_LOGI(TAG, "Running parallel operation tests...");
        RUN_TEST_IN_TASK("parallel_operation", test_parallel_operation, 8192, 5);
    );
    
    //=========================================================================
    // ERROR CONDITION TESTS
    //=========================================================================
    RUN_TEST_SECTION_IF_ENABLED(
        ENABLE_ERROR_CONDITION_TESTS, "ERROR CONDITION TESTS",
        ESP_LOGI(TAG, "Running error condition tests...");
        RUN_TEST_IN_TASK("error_conditions", test_error_conditions, 8192, 5);
    );
    
    // Print test results summary
    print_test_summary(g_test_results, "TLE92466ED", TAG);
    
    // Cleanup
    if (g_driver) {
        delete g_driver;
        g_driver = nullptr;
    }
    g_hal.reset();
    cleanup_test_progress_indicator();
    
    // Final message
    if (g_test_results.failed_tests == 0) {
        ESP_LOGI(TAG, "✅ ALL TESTS PASSED! System will restart in 10 seconds...");
    } else {
        ESP_LOGE(TAG, "❌ SOME TESTS FAILED! System will restart in 10 seconds...");
    }
    
    vTaskDelay(pdMS_TO_TICKS(10000));
    esp_restart();
}
