/**
 * @file basic_usage.cpp
 * @brief Basic usage example for TLE92466ED driver
 * @author AI Generated Driver
 * @date 2025-10-20
 * @version 2.0.0
 *
 * @details
 * Demonstrates basic initialization and current control of the TLE92466ED IC.
 * This example shows how to use the Integrated Current Control (ICC) mode
 * to drive solenoid loads with precise current regulation.
 */

#include "TLE92466ED.hpp"
#include "example_hal.hpp"
#include <iostream>
// Conditional includes based on C++ standard
#if __cpp_lib_chrono >= 201907L
#include <thread>
#include <chrono>
#endif

using namespace TLE92466ED;
// Removed chrono usage - using uint32_t for microseconds instead

/**
 * @brief Basic usage example - Current Control Mode
 */
int main() {
    std::cout << "========================================\n";
    std::cout << "TLE92466ED Driver - Basic Usage Example\n";
    std::cout << "========================================\n\n";

    // 1. Create HAL instance for your platform
    std::cout << "Creating HAL instance...\n";
    ExampleHAL hal;

    // 2. Create driver instance
    std::cout << "Creating driver instance...\n";
    Driver driver(hal);

    // 3. Initialize driver (device starts in Config Mode)
    std::cout << "\nInitializing driver...\n";
    auto init_result = driver.init();
    if (!init_result) {
        std::cerr << "ERROR: Driver initialization failed!\n";
        return 1;
    }
    std::cout << "✓ Driver initialized successfully (Config Mode)\n";

    // 4. Configure global settings
    std::cout << "\nConfiguring global settings...\n";
    GlobalConfig global_config{
        .crc_enabled = true,
        .spi_watchdog_enabled = true,
        .clock_watchdog_enabled = true,
        .vio_5v = false,  // 3.3V logic
        .vbat_uv_threshold = 43,   // ~7V (43 * 0.16208V)
        .vbat_ov_threshold = 247,  // ~40V
        .spi_watchdog_reload = 1000
    };

    if (auto result = driver.configure_global(global_config); !result) {
        std::cerr << "ERROR: Global configuration failed!\n";
        return 1;
    }
    std::cout << "✓ Global configuration complete\n";

    // 5. Configure Channel 0 for ICC current control
    std::cout << "\nConfiguring Channel 0...\n";
    ChannelConfig ch0_config{
        .mode = ChannelMode::ICC,        // Integrated Current Control
        .current_setpoint_ma = 1500,     // 1.5A setpoint
        .slew_rate = SlewRate::MEDIUM_2V5_US,
        .diag_current = DiagCurrent::I_190UA,
        .open_load_threshold = 3,        // 3/8 of setpoint
        .pwm_period_mantissa = 100,
        .pwm_period_exponent = 4,
        .auto_limit_disabled = false,
        .olsg_warning_enabled = true,
        .deep_dither_enabled = false,
        .dither_step_size = 0,  // No dither for this example
        .dither_steps = 0,
        .dither_flat = 0
    };

    if (auto result = driver.configure_channel(Channel::CH0, ch0_config); !result) {
        std::cerr << "ERROR: Channel 0 configuration failed!\n";
        return 1;
    }
    std::cout << "✓ Channel 0 configured (ICC mode, 1.5A setpoint)\n";

    // 6. Enter Mission Mode to enable channel control
    std::cout << "\nEntering Mission Mode...\n";
    if (auto result = driver.enter_mission_mode(); !result) {
        std::cerr << "ERROR: Failed to enter Mission Mode!\n";
        return 1;
    }
    std::cout << "✓ Mission Mode active - channels can now be enabled\n";

    // 7. Enable Channel 0
    std::cout << "\nEnabling Channel 0...\n";
    if (auto result = driver.enable_channel(Channel::CH0, true); !result) {
        std::cerr << "ERROR: Failed to enable Channel 0!\n";
        return 1;
    }
    std::cout << "✓ Channel 0 enabled - current regulation active at 1.5A\n";

    // 8. Monitor for a few seconds
    std::cout << "\nMonitoring for 5 seconds...\n";
    for (int i = 0; i < 5; ++i) {
#if __cpp_lib_chrono >= 201907L
        std::this_thread::sleep_for(std::chrono::seconds(1));
#else
        // C++11 fallback - simple delay loop
        for (volatile int i = 0; i < 1000000; ++i) {
            // Busy wait delay
        }
#endif

        // Reload SPI watchdog
        (void)driver.reload_spi_watchdog(1000);

        // Get average current
        if (auto current = driver.get_average_current(Channel::CH0)) {
            std::cout << "  [" << i+1 << "s] Average current: " << *current << " mA\n";
        }

        // Check for faults
        if (auto diag = driver.get_channel_diagnostics(Channel::CH0)) {
            if (diag->overcurrent) {
                std::cout << "  WARNING: Over-current detected!\n";
            }
            if (diag->open_load) {
                std::cout << "  WARNING: Open load detected!\n";
            }
            if (diag->short_to_ground) {
                std::cout << "  ERROR: Short to ground detected!\n";
                break;
            }
        }
    }

    // 9. Demonstrate setpoint change
    std::cout << "\nChanging setpoint to 1.0A...\n";
    if (auto result = driver.set_current_setpoint(Channel::CH0, 1000); !result) {
        std::cerr << "ERROR: Failed to set current!\n";
    } else {
        std::cout << "✓ Setpoint updated to 1.0A\n";
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    // 10. Check global device status
    std::cout << "\nChecking device status...\n";
    if (auto status = driver.get_device_status()) {
        std::cout << "  Device Status:\n";
        std::cout << "    Mode: " << (status->config_mode ? "Config" : "Mission") << "\n";
        std::cout << "    Init Done: " << (status->init_done ? "YES" : "NO") << "\n";
        std::cout << "    Any Fault: " << (status->any_fault ? "YES" : "NO") << "\n";
        std::cout << "    VBAT UV: " << (status->vbat_uv ? "YES" : "NO") << "\n";
        std::cout << "    VBAT OV: " << (status->vbat_ov ? "YES" : "NO") << "\n";
        std::cout << "    OT Warning: " << (status->ot_warning ? "YES" : "NO") << "\n";
        std::cout << "    OT Error: " << (status->ot_error ? "YES" : "NO") << "\n";
        std::cout << "    SPI WD Error: " << (status->spi_wd_error ? "YES" : "NO") << "\n";
    }

    // 11. Get voltage readings
    if (auto vbat = driver.get_vbat_voltage()) {
        std::cout << "  VBAT: " << *vbat << " (raw value)\n";
    }

    // 12. Disable channel
    std::cout << "\nDisabling Channel 0...\n";
    if (auto result = driver.enable_channel(Channel::CH0, false); !result) {
        std::cerr << "ERROR: Failed to disable Channel 0!\n";
    } else {
        std::cout << "✓ Channel 0 disabled\n";
    }

    // 13. Return to config mode
    std::cout << "\nReturning to Config Mode...\n";
    if (auto result = driver.enter_config_mode(); !result) {
        std::cerr << "ERROR: Failed to enter Config Mode!\n";
    } else {
        std::cout << "✓ Config Mode active\n";
    }

    std::cout << "\n========================================\n";
    std::cout << "Example completed successfully!\n";
    std::cout << "========================================\n";
    
    return 0;
}
