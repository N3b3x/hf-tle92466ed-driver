/**
 * @file TLE92466ED.cpp
 * @brief Implementation of TLE92466ED driver class
 * @author AI Generated Driver
 * @date 2025-10-20
 * @version 2.0.0
 *
 * @details
 * This file contains the complete implementation of all driver methods
 * for the TLE92466ED IC. All methods include comprehensive error checking,
 * CRC calculation/verification, and validation.
 *
 * @copyright
 * This is free and unencumbered software released into the public domain.
 */

#include "TLE92466ED.hpp"

namespace TLE92466ED {

//==============================================================================
// INITIALIZATION
//==============================================================================

DriverResult<void> Driver::init() noexcept {
    // 1. Initialize HAL
#if __cpp_lib_expected >= 202202L
    if (auto result = hal_.init(); !result) {
        return std::unexpected(DriverError::HardwareError);
    }
#else
    auto init_result = hal_.init();
    if (!init_result) {
        return DriverError::HardwareError;
    }
#endif

    // 2. Wait for device power-up (minimum 1ms per datasheet)
#if __cpp_lib_expected >= 202202L
    if (auto result = hal_.delay(2000); !result) {  // 2ms = 2000 microseconds
        return std::unexpected(DriverError::HardwareError);
    }
#else
    auto delay_result = hal_.delay(2000);  // 2ms = 2000 microseconds
    if (!delay_result) {
        return DriverError::HardwareError;
    }
#endif

    // 3. Verify device communication by reading IC version
    auto verify_result = verify_device();
    if (!verify_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(verify_result.error());
#else
        return verify_result.error_code;
#endif
    }
    if (!*verify_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(DriverError::WrongDeviceID);
#else
        return DriverError::WrongDeviceID;
#endif
    }

    // 4. Device starts in Config Mode after power-up
    mission_mode_ = false;

    // 5. Apply default configuration
    auto config_result = apply_default_config();
    if (!config_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(config_result.error());
#else
        return config_result.error_code;
#endif
    }

    // 6. Clear any power-on reset flags
    auto clear_result = clear_faults();
    if (!clear_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(clear_result.error());
#else
        return clear_result.error_code;
#endif
    }

    // 7. Initialize cached state
    channel_enable_cache_ = 0;
    for (size_t i = 0; i < 6; ++i) {
        channel_setpoints_[i] = 0;
    }

    initialized_ = true;
    return DriverResult<void>();
}

DriverResult<void> Driver::apply_default_config() noexcept {
    // Configure GLOBAL_CONFIG: Enable CRC, SPI watchdog, clock watchdog, 3.3V VIO
    uint16_t global_cfg = GLOBAL_CONFIG::CRC_EN | 
                          GLOBAL_CONFIG::SPI_WD_EN | 
                          GLOBAL_CONFIG::CLK_WD_EN;
    
    auto write_result = write_register(CentralReg::GLOBAL_CONFIG, global_cfg, false);
    if (!write_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(write_result.error());
#else
        return write_result.error_code;
#endif
    }

    // Set default VBAT thresholds (UV=7V, OV=40V approximately)
    // UV threshold: 7V / 0.16208V = ~43 (0x2B)
    // OV threshold: 40V / 0.16208V = ~247 (0xF7)
    uint16_t vbat_th = (0xF7 << 8) | 0x2B;
    write_result = write_register(CentralReg::VBAT_TH, vbat_th, false);
    if (!write_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(write_result.error());
#else
        return write_result.error_code;
#endif
    }

    // Configure all channels with default settings (ICC mode, 1V/us slew, disabled)
    for (uint8_t ch = 0; ch < static_cast<uint8_t>(Channel::COUNT); ++ch) {
        Channel channel = static_cast<Channel>(ch);
        uint16_t ch_base = get_channel_base(channel);
        
        // Set mode to ICC (0x0001)
        write_result = write_register(ch_base + ChannelReg::MODE, 
                           static_cast<uint16_t>(ChannelMode::ICC), false);
        if (!write_result) {
#if __cpp_lib_expected >= 202202L
            return std::unexpected(write_result.error());
#else
            return write_result.error_code;
#endif
        }
        
        // Set default CH_CONFIG (slew rate 2.5V/us, OL disabled)
        write_result = write_register(ch_base + ChannelReg::CH_CONFIG, 
                           CH_CONFIG::SLEWR_2V5_US, false);
        if (!write_result) {
#if __cpp_lib_expected >= 202202L
            return std::unexpected(write_result.error());
#else
            return write_result.error_code;
#endif
        }
        
        // Set current setpoint to 0
        write_result = write_register(ch_base + ChannelReg::SETPOINT, 0, false);
        if (!write_result) {
#if __cpp_lib_expected >= 202202L
            return std::unexpected(write_result.error());
#else
            return write_result.error_code;
#endif
        }
    }

    // Reload SPI watchdog
    write_result = write_register(CentralReg::WD_RELOAD, 1000, false);
    if (!write_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(write_result.error());
#else
        return write_result.error_code;
#endif
    }

    return DriverResult<void>();
}

//==========================================================================
// MODE CONTROL
//==========================================================================

DriverResult<void> Driver::enter_mission_mode() noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    // Set OP_MODE bit in CH_CTRL register
    auto modify_result = modify_register(CentralReg::CH_CTRL, 
                         CH_CTRL::OP_MODE, 
                         CH_CTRL::OP_MODE);
    if (!modify_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(modify_result.error());
#else
        return modify_result.error_code;
#endif
    }

    mission_mode_ = true;
    return DriverResult<void>();
}

DriverResult<void> Driver::enter_config_mode() noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    // Clear OP_MODE bit in CH_CTRL register
    auto modify_result = modify_register(CentralReg::CH_CTRL, 
                         CH_CTRL::OP_MODE, 
                         0);
    if (!modify_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(modify_result.error());
#else
        return modify_result.error_code;
#endif
    }

    mission_mode_ = false;
    return DriverResult<void>();
}

//==========================================================================
// GLOBAL CONFIGURATION
//==========================================================================

DriverResult<void> Driver::configure_global(const GlobalConfig& config) noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    // Must be in config mode to change global configuration
    auto mode_check = check_config_mode();
    if (!mode_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(mode_check.error());
#else
        return mode_check.error_code;
#endif
    }

    // Build GLOBAL_CONFIG register value
    uint16_t global_cfg = 0;
    if (config.clock_watchdog_enabled) global_cfg |= GLOBAL_CONFIG::CLK_WD_EN;
    if (config.spi_watchdog_enabled) global_cfg |= GLOBAL_CONFIG::SPI_WD_EN;
    if (config.crc_enabled) global_cfg |= GLOBAL_CONFIG::CRC_EN;
    if (config.vio_5v) global_cfg |= GLOBAL_CONFIG::VIO_SEL;

    auto write_result = write_register(CentralReg::GLOBAL_CONFIG, global_cfg, true);
    if (!write_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(write_result.error());
#else
        return write_result.error_code;
#endif
    }

    // Configure VBAT thresholds
    auto vbat_result = set_vbat_thresholds(config.vbat_uv_threshold, 
                             config.vbat_ov_threshold);
    if (!vbat_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(vbat_result.error());
#else
        return vbat_result.error_code;
#endif
    }

    // Configure SPI watchdog reload
    if (config.spi_watchdog_enabled) {
        write_result = write_register(CentralReg::WD_RELOAD, 
                           config.spi_watchdog_reload, true);
        if (!write_result) {
#if __cpp_lib_expected >= 202202L
            return std::unexpected(write_result.error());
#else
            return write_result.error_code;
#endif
        }
    }

    return DriverResult<void>();
}

DriverResult<void> Driver::set_crc_enabled(bool enabled) noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    auto modify_result = modify_register(CentralReg::GLOBAL_CONFIG,
                          GLOBAL_CONFIG::CRC_EN,
                          enabled ? GLOBAL_CONFIG::CRC_EN : 0);
    if (!modify_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(modify_result.error());
#else
        return modify_result.error_code;
#endif
    }

    return DriverResult<void>();
}

DriverResult<void> Driver::set_vbat_thresholds(uint8_t uv_threshold, uint8_t ov_threshold) noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    uint16_t value = (static_cast<uint16_t>(ov_threshold) << 8) | uv_threshold;
    auto write_result = write_register(CentralReg::VBAT_TH, value, true);
    if (!write_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(write_result.error());
#else
        return write_result.error_code;
#endif
    }

    return DriverResult<void>();
}

//==========================================================================
// CHANNEL CONTROL
//==========================================================================

DriverResult<void> Driver::enable_channel(Channel channel, bool enabled) noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    // Channel enable can only be changed in Mission Mode
    auto mode_check = check_mission_mode();
    if (!mode_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(mode_check.error());
#else
        return mode_check.error_code;
#endif
    }

    if (!is_valid_channel_internal(channel)) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(DriverError::InvalidChannel);
#else
        return DriverError::InvalidChannel;
#endif
    }

    uint16_t mask = CH_CTRL::channel_mask(to_index(channel));

    if (enabled) {
        channel_enable_cache_ |= mask;
    } else {
        channel_enable_cache_ &= ~mask;
    }

    auto modify_result = modify_register(CentralReg::CH_CTRL, mask, enabled ? mask : 0);
    if (!modify_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(modify_result.error());
#else
        return modify_result.error_code;
#endif
    }

    return DriverResult<void>();
}

DriverResult<void> Driver::enable_channels(uint8_t channel_mask) noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    auto mode_check = check_mission_mode();
    if (!mode_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(mode_check.error());
#else
        return mode_check.error_code;
#endif
    }

    // Mask to valid channels only (bits 0-5)
    channel_mask &= CH_CTRL::ALL_CH_MASK;
    channel_enable_cache_ = channel_mask;

    auto modify_result = modify_register(CentralReg::CH_CTRL, CH_CTRL::ALL_CH_MASK, channel_mask);
    if (!modify_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(modify_result.error());
#else
        return modify_result.error_code;
#endif
    }

    return DriverResult<void>();
}

DriverResult<void> Driver::enable_all_channels() noexcept {
    return enable_channels(CH_CTRL::ALL_CH_MASK);
}

DriverResult<void> Driver::disable_all_channels() noexcept {
    return enable_channels(0);
}

DriverResult<void> Driver::set_channel_mode(Channel channel, ChannelMode mode) noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    // Mode can only be changed in Config Mode
    auto mode_check = check_config_mode();
    if (!mode_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(mode_check.error());
#else
        return mode_check.error_code;
#endif
    }

    if (!is_valid_channel_internal(channel)) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(DriverError::InvalidChannel);
#else
        return DriverError::InvalidChannel;
#endif
    }

    uint16_t ch_addr = get_channel_register(channel, ChannelReg::MODE);
    auto write_result = write_register(ch_addr, static_cast<uint16_t>(mode), true);
    if (!write_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(write_result.error());
#else
        return write_result.error_code;
#endif
    }

    return DriverResult<void>();
}

DriverResult<void> Driver::set_parallel_operation(ParallelPair pair, bool enabled) noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    // Parallel operation can only be changed in Config Mode
    auto mode_check = check_config_mode();
    if (!mode_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(mode_check.error());
#else
        return mode_check.error_code;
#endif
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
#if __cpp_lib_expected >= 202202L
            return std::unexpected(DriverError::InvalidParameter);
#else
            return DriverError::InvalidParameter;
#endif
    }

    auto modify_result = modify_register(CentralReg::CH_CTRL, mask, enabled ? mask : 0);
    if (!modify_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(modify_result.error());
#else
        return modify_result.error_code;
#endif
    }

    return DriverResult<void>();
}

//==========================================================================
// CURRENT CONTROL
//==========================================================================

DriverResult<void> Driver::set_current_setpoint(Channel channel, uint16_t current_ma, bool parallel_mode) noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    if (!is_valid_channel_internal(channel)) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(DriverError::InvalidChannel);
#else
        return DriverError::InvalidChannel;
#endif
    }

    // Validate current range (using absolute register scale)
    // Note: Datasheet typical continuous limits are ~1.5A single, ~2.7A parallel
    // but register scale allows up to 2A/4A for transient operation
    uint16_t max_current = parallel_mode ? 4000 : 2000;
    if (current_ma > max_current) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(DriverError::InvalidParameter);
#else
        return DriverError::InvalidParameter;
#endif
    }

    // Calculate setpoint register value
    uint16_t target = SETPOINT::calculate_target(current_ma, parallel_mode);
    
    // Cache the setpoint
    channel_setpoints_[to_index(channel)] = target;

    // Write to SETPOINT register
    uint16_t ch_addr = get_channel_register(channel, ChannelReg::SETPOINT);
    auto write_result = write_register(ch_addr, target, true);
    if (!write_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(write_result.error());
#else
        return write_result.error_code;
#endif
    }

    return DriverResult<void>();
}

DriverResult<uint16_t> Driver::get_current_setpoint(Channel channel, bool parallel_mode) noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    if (!is_valid_channel_internal(channel)) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(DriverError::InvalidChannel);
#else
        return DriverError::InvalidChannel;
#endif
    }

    // Read SETPOINT register
    uint16_t ch_addr = get_channel_register(channel, ChannelReg::SETPOINT);
    auto read_result = read_register(ch_addr, true);
    if (!read_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(read_result.error());
#else
        return read_result.error_code;
#endif
    }

    // Convert to current in mA
    uint16_t target = *read_result & SETPOINT::TARGET_MASK;
    uint16_t current_ma = SETPOINT::calculate_current(target, parallel_mode);

    return current_ma;
}

DriverResult<void> Driver::configure_pwm_period(Channel channel, uint8_t period_mantissa, uint8_t period_exponent, bool low_freq_range) noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    if (!is_valid_channel_internal(channel)) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(DriverError::InvalidChannel);
#else
        return DriverError::InvalidChannel;
#endif
    }

    // Build PERIOD register value
    uint16_t value = period_mantissa | 
                    ((period_exponent & 0x07) << 8) |
                    (low_freq_range ? (1 << 11) : 0);

    uint16_t ch_addr = get_channel_register(channel, ChannelReg::PERIOD);
    auto write_result = write_register(ch_addr, value, true);
    if (!write_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(write_result.error());
#else
        return write_result.error_code;
#endif
    }

    return DriverResult<void>();
}

DriverResult<void> Driver::configure_dither(Channel channel, uint16_t step_size, uint8_t num_steps, uint8_t flat_steps) noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    if (!is_valid_channel_internal(channel)) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(DriverError::InvalidChannel);
#else
        return DriverError::InvalidChannel;
#endif
    }

    uint16_t ch_base = get_channel_base(channel);

    // Configure DITHER_CTRL (step size)
    uint16_t ctrl_value = step_size & DITHER_CTRL::STEP_SIZE_MASK;
    auto write_result = write_register(ch_base + ChannelReg::DITHER_CTRL, ctrl_value, true);
    if (!write_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(write_result.error());
#else
        return write_result.error_code;
#endif
    }

    // Configure DITHER_STEP (steps and flat period)
    uint16_t step_value = flat_steps | (static_cast<uint16_t>(num_steps) << DITHER_STEP::STEPS_SHIFT);
    write_result = write_register(ch_base + ChannelReg::DITHER_STEP, step_value, true);
    if (!write_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(write_result.error());
#else
        return write_result.error_code;
#endif
    }

    return DriverResult<void>();
}

DriverResult<void> Driver::configure_channel(Channel channel, const ChannelConfig& config) noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    // Most configuration requires Config Mode
    auto mode_check = check_config_mode();
    if (!mode_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(mode_check.error());
#else
        return mode_check.error_code;
#endif
    }

    if (!is_valid_channel_internal(channel)) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(DriverError::InvalidChannel);
#else
        return DriverError::InvalidChannel;
#endif
    }

    uint16_t ch_base = get_channel_base(channel);

    // 1. Set channel mode
    auto write_result = write_register(ch_base + ChannelReg::MODE, 
                        static_cast<uint16_t>(config.mode), true);
    if (!write_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(write_result.error());
#else
        return write_result.error_code;
#endif
    }

    // 2. Set current setpoint with parallel mode detection
    auto parallel_result = is_channel_parallel(channel);
    if (!parallel_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(parallel_result.error());
#else
        return parallel_result.error_code;
#endif
    }
    bool is_parallel = *parallel_result;
    uint16_t target = SETPOINT::calculate_target(config.current_setpoint_ma, is_parallel);
    if (config.auto_limit_disabled) {
        target |= SETPOINT::AUTO_LIMIT_DIS;
    }
    write_result = write_register(ch_base + ChannelReg::SETPOINT, target, true);
    if (!write_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(write_result.error());
#else
        return write_result.error_code;
#endif
    }

    // 3. Configure CH_CONFIG register
    uint16_t ch_cfg = static_cast<uint16_t>(config.slew_rate) |
                     (static_cast<uint16_t>(config.diag_current) << 2) |
                     (static_cast<uint16_t>(config.open_load_threshold & 0x07) << 4);
    
    write_result = write_register(ch_base + ChannelReg::CH_CONFIG, ch_cfg, true);
    if (!write_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(write_result.error());
#else
        return write_result.error_code;
#endif
    }

    // 3a. Configure OLSG warning enable if requested (bit 14 of CTRL register)
    if (config.olsg_warning_enabled) {
        auto modify_result = modify_register(ch_base + ChannelReg::CTRL, 
                            CH_CTRL_REG::OLSG_WARN_EN,
                            CH_CTRL_REG::OLSG_WARN_EN);
        if (!modify_result) {
#if __cpp_lib_expected >= 202202L
            return std::unexpected(modify_result.error());
#else
            return modify_result.error_code;
#endif
        }
    }

    // 4. Configure PWM if specified
    if (config.pwm_period_mantissa > 0) {
        auto pwm_result = configure_pwm_period(channel, 
                                 config.pwm_period_mantissa,
                                 config.pwm_period_exponent,
                                 false);
        if (!pwm_result) {
#if __cpp_lib_expected >= 202202L
            return std::unexpected(pwm_result.error());
#else
            return pwm_result.error_code;
#endif
        }
    }

    // 5. Configure dither if specified
    if (config.dither_step_size > 0) {
        auto dither_result = configure_dither(channel,
                             config.dither_step_size,
                             config.dither_steps,
                             config.dither_flat);
        if (!dither_result) {
#if __cpp_lib_expected >= 202202L
            return std::unexpected(dither_result.error());
#else
            return dither_result.error_code;
#endif
        }
        
        // 5a. Enable deep dither if requested (bit 13 of DITHER_CTRL)
        if (config.deep_dither_enabled) {
            auto modify_result = modify_register(ch_base + ChannelReg::DITHER_CTRL,
                                DITHER_CTRL::DEEP_DITHER,
                                DITHER_CTRL::DEEP_DITHER);
            if (!modify_result) {
#if __cpp_lib_expected >= 202202L
                return std::unexpected(modify_result.error());
#else
                return modify_result.error_code;
#endif
            }
        }
    }

    return DriverResult<void>();
}

//==========================================================================
// STATUS AND DIAGNOSTICS
//==========================================================================

DriverResult<DeviceStatus> Driver::get_device_status() noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    // Initialize status structure
    DeviceStatus status;

    // Read GLOBAL_DIAG0
    auto diag0_result = read_register(CentralReg::GLOBAL_DIAG0, true);
    if (!diag0_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(diag0_result.error());
#else
        return diag0_result.error_code;
#endif
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
    auto fb_stat_result = read_register(CentralReg::FB_STAT, true);
    if (fb_stat_result) {
        uint16_t fb_stat = *fb_stat_result;
        status.supply_nok_internal = (fb_stat & FB_STAT::SUP_NOK_INT) != 0;
        status.supply_nok_external = (fb_stat & FB_STAT::SUP_NOK_EXT) != 0;
        status.init_done = (fb_stat & FB_STAT::INIT_DONE) != 0;
    }

    // Read CH_CTRL to get mode
    auto ch_ctrl_result = read_register(CentralReg::CH_CTRL, true);
    if (ch_ctrl_result) {
        uint16_t ch_ctrl = *ch_ctrl_result;
        status.config_mode = (ch_ctrl & CH_CTRL::OP_MODE) == 0;
    }

    // Read voltage feedbacks
    auto fb_voltage1_result = read_register(CentralReg::FB_VOLTAGE1, true);
    if (fb_voltage1_result) {
        status.vbat_voltage = *fb_voltage1_result;
    }

    auto fb_voltage2_result = read_register(CentralReg::FB_VOLTAGE2, true);
    if (fb_voltage2_result) {
        status.vio_voltage = *fb_voltage2_result;
    }

    return status;
}

DriverResult<ChannelDiagnostics> Driver::get_channel_diagnostics(Channel channel) noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    if (!is_valid_channel_internal(channel)) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(DriverError::InvalidChannel);
#else
        return DriverError::InvalidChannel;
#endif
    }

    // Initialize diagnostics structure
    ChannelDiagnostics diag;

    // Read DIAG_ERR register for this channel group
    auto diag_err_result = read_register(CentralReg::DIAG_ERR_CHGR0 + to_index(channel), true);
    if (diag_err_result) {
        uint16_t diag_err = *diag_err_result;
        // Parse error flags (bit positions from datasheet Table in page 67)
        diag.overcurrent = (diag_err & (1 << 0)) != 0;         // OC bit
        diag.short_to_ground = (diag_err & (1 << 1)) != 0;     // SG bit
        diag.open_load = (diag_err & (1 << 2)) != 0;           // OL bit
        diag.over_temperature = (diag_err & (1 << 3)) != 0;    // OTE bit
        diag.open_load_short_ground = (diag_err & (1 << 4)) != 0; // OLSG bit
    }

    // Read DIAG_WARN register for warnings
    auto diag_warn_result = read_register(CentralReg::DIAG_WARN_CHGR0 + to_index(channel), true);
    if (diag_warn_result) {
        uint16_t diag_warn = *diag_warn_result;
        diag.ot_warning = (diag_warn & (1 << 0)) != 0;
        diag.current_regulation_warning = (diag_warn & (1 << 1)) != 0;
        diag.pwm_regulation_warning = (diag_warn & (1 << 2)) != 0;
        diag.olsg_warning = (diag_warn & (1 << 3)) != 0;
    }

    // Read feedback values
    uint16_t ch_base = get_channel_base(channel);
    
    auto fb_i_avg_result = read_register(ch_base + ChannelReg::FB_I_AVG, true);
    if (fb_i_avg_result) {
        diag.average_current = *fb_i_avg_result;
    }

    auto fb_dc_result = read_register(ch_base + ChannelReg::FB_DC, true);
    if (fb_dc_result) {
        diag.duty_cycle = *fb_dc_result;
    }

    auto fb_vbat_result = read_register(ch_base + ChannelReg::FB_VBAT, true);
    if (fb_vbat_result) {
        diag.vbat_feedback = *fb_vbat_result;
    }

    // Read min/max current feedback (FB_IMIN_IMAX register)
    // Register format: [15:8] = I_MAX, [7:0] = I_MIN
    auto fb_minmax_result = read_register(ch_base + ChannelReg::FB_IMIN_IMAX, true);
    if (fb_minmax_result) {
        uint16_t fb_minmax = *fb_minmax_result;
        diag.min_current = fb_minmax & 0x00FF;        // Lower 8 bits
        diag.max_current = (fb_minmax >> 8) & 0x00FF; // Upper 8 bits
    }

    return diag;
}

DriverResult<uint16_t> Driver::get_average_current(Channel channel, bool parallel_mode) noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    if (!is_valid_channel_internal(channel)) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(DriverError::InvalidChannel);
#else
        return DriverError::InvalidChannel;
#endif
    }

    uint16_t ch_addr = get_channel_register(channel, ChannelReg::FB_I_AVG);
    auto read_result = read_register(ch_addr, true);
    if (!read_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(read_result.error());
#else
        return read_result.error_code;
#endif
    }

    // Convert raw value to mA
    // Based on datasheet: similar calculation to setpoint
    uint16_t current_ma = SETPOINT::calculate_current(*read_result, parallel_mode);
    return current_ma;
}

DriverResult<uint16_t> Driver::get_duty_cycle(Channel channel) noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    if (!is_valid_channel_internal(channel)) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(DriverError::InvalidChannel);
#else
        return DriverError::InvalidChannel;
#endif
    }

    uint16_t ch_addr = get_channel_register(channel, ChannelReg::FB_DC);
    auto read_result = read_register(ch_addr, true);
    if (!read_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(read_result.error());
#else
        return read_result.error_code;
#endif
    }

    return *read_result;
}

DriverResult<uint16_t> Driver::get_vbat_voltage() noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    auto read_result = read_register(CentralReg::FB_VOLTAGE1, true);
    if (!read_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(read_result.error());
#else
        return read_result.error_code;
#endif
    }

    // Convert to millivolts (formula from datasheet)
    // V_BAT measurement encoding needs datasheet formula
    return *read_result; // Return raw value for now
}

DriverResult<uint16_t> Driver::get_vio_voltage() noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    auto read_result = read_register(CentralReg::FB_VOLTAGE2, true);
    if (!read_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(read_result.error());
#else
        return read_result.error_code;
#endif
    }

    return *read_result; // Return raw value
}

//==========================================================================
// FAULT MANAGEMENT
//==========================================================================

DriverResult<void> Driver::clear_faults() noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    // Write 1s to clear fault bits in GLOBAL_DIAG0 (rwh type - clear on write 1)
    auto write_result = write_register(CentralReg::GLOBAL_DIAG0, 0xFFFF, true);
    if (!write_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(write_result.error());
#else
        return write_result.error_code;
#endif
    }

    // Clear GLOBAL_DIAG1
    write_result = write_register(CentralReg::GLOBAL_DIAG1, 0xFFFF, true);
    if (!write_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(write_result.error());
#else
        return write_result.error_code;
#endif
    }

    // Clear GLOBAL_DIAG2
    write_result = write_register(CentralReg::GLOBAL_DIAG2, 0xFFFF, true);
    if (!write_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(write_result.error());
#else
        return write_result.error_code;
#endif
    }

    return DriverResult<void>();
}

DriverResult<bool> Driver::has_any_fault() noexcept {
    auto status_result = get_device_status();
    if (!status_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(status_result.error());
#else
        return status_result.error_code;
#endif
    }

    return status_result->any_fault;
}

DriverResult<void> Driver::software_reset() noexcept {
    // Software reset would require toggling RESN pin or power cycle
    // This IC doesn't have a software reset register
    // Return to config mode and disable all channels instead
    auto config_result = enter_config_mode();
    if (!config_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(config_result.error());
#else
        return config_result.error_code;
#endif
    }

    auto disable_result = disable_all_channels();
    if (!disable_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(disable_result.error());
#else
        return disable_result.error_code;
#endif
    }

    return DriverResult<void>();
}

//==========================================================================
// WATCHDOG MANAGEMENT
//==========================================================================

DriverResult<void> Driver::reload_spi_watchdog(uint16_t reload_value) noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    auto write_result = write_register(CentralReg::WD_RELOAD, reload_value, true);
    if (!write_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(write_result.error());
#else
        return write_result.error_code;
#endif
    }

    return DriverResult<void>();
}

//==========================================================================
// DEVICE INFORMATION
//==========================================================================

DriverResult<uint16_t> Driver::get_ic_version() noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    return read_register(CentralReg::ICVID, true);
}

DriverResult<std::array<uint16_t, 3>> Driver::get_chip_id() noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    std::array<uint16_t, 3> chip_id;
    
    auto id0_result = read_register(CentralReg::CHIPID0, true);
    if (!id0_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(id0_result.error());
#else
        return id0_result.error_code;
#endif
    }
    chip_id[0] = *id0_result;

    auto id1_result = read_register(CentralReg::CHIPID1, true);
    if (!id1_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(id1_result.error());
#else
        return id1_result.error_code;
#endif
    }
    chip_id[1] = *id1_result;

    auto id2_result = read_register(CentralReg::CHIPID2, true);
    if (!id2_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(id2_result.error());
#else
        return id2_result.error_code;
#endif
    }
    chip_id[2] = *id2_result;

    return chip_id;
}

DriverResult<bool> Driver::verify_device() noexcept {
    // Read ICVID register to verify device is responding and check device type
    auto icvid_result = read_register(CentralReg::ICVID, false); // Don't verify CRC during init
    if (!icvid_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(icvid_result.error());
#else
        return icvid_result.error_code;
#endif
    }
    
    // Validate device ID (checks for valid response: not 0x0000 or 0xFFFF)
    bool is_valid = DeviceID::is_valid_device(*icvid_result);
    
    return is_valid;
}

//==========================================================================
// REGISTER ACCESS
//==========================================================================

DriverResult<uint16_t> Driver::read_register(uint16_t address, bool verify_crc) noexcept {
    if (!hal_.is_ready()) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(DriverError::HardwareError);
#else
        return DriverError::HardwareError;
#endif
    }

    // Create read frame
    SPIFrame tx_frame = SPIFrame::make_read(address);
    
    // Calculate and set CRC
    tx_frame.tx_fields.crc = calculate_frame_crc(tx_frame);

    // Transfer frame
    SPIFrame rx_frame;
    auto transfer_result = transfer_frame(tx_frame, &rx_frame, verify_crc);
    if (!transfer_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(transfer_result.error());
#else
        return transfer_result.error_code;
#endif
    }

    // Extract data from response (copy from bit-field to avoid reference issue)
    return rx_frame.rx_fields.data;
}

DriverResult<void> Driver::write_register(uint16_t address, uint16_t value, bool verify_crc) noexcept {
    if (!hal_.is_ready()) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(DriverError::HardwareError);
#else
        return DriverError::HardwareError;
#endif
    }

    // Create write frame
    SPIFrame tx_frame = SPIFrame::make_write(address, value);
    
    // Calculate and set CRC
    tx_frame.tx_fields.crc = calculate_frame_crc(tx_frame);

    // Transfer frame
    SPIFrame rx_frame;
    auto transfer_result = transfer_frame(tx_frame, &rx_frame, verify_crc);
    if (!transfer_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(transfer_result.error());
#else
        return transfer_result.error_code;
#endif
    }

    return DriverResult<void>();
}

DriverResult<void> Driver::modify_register(uint16_t address, uint16_t mask, uint16_t value) noexcept {
    // Read current value
    auto read_result = read_register(address, true);
    if (!read_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(read_result.error());
#else
        return read_result.error_code;
#endif
    }

    // Modify bits
    uint16_t new_value = (*read_result & ~mask) | (value & mask);

    // Write back
    return write_register(address, new_value, true);
}

//==========================================================================
// PRIVATE METHODS
//==========================================================================

DriverResult<void> Driver::transfer_frame(const SPIFrame& tx_frame, SPIFrame* rx_frame, bool verify_crc) noexcept {
    // Transfer 32-bit frame via HAL
#if __cpp_lib_expected >= 202202L
    auto result = hal_.transfer32(tx_frame.word);
    if (!result) {
        // Map HAL error to driver error
        switch (result.error()) {
            case HALError::Timeout:
                return std::unexpected(DriverError::TimeoutError);
            case HALError::CRCError:
                return std::unexpected(DriverError::CRCError);
            case HALError::TransferError:
            case HALError::BusError:
                return std::unexpected(DriverError::HardwareError);
            default:
                return std::unexpected(DriverError::RegisterError);
        }
    }
    rx_frame->word = *result;
#else
    auto result = hal_.transfer32(tx_frame.word);
    if (!result) {
        // Map HAL error to driver error
        switch (result.error_code) {
            case HALError::Timeout:
                return DriverError::TimeoutError;
            case HALError::CRCError:
                return DriverError::CRCError;
            case HALError::TransferError:
            case HALError::BusError:
                return DriverError::HardwareError;
            default:
                return DriverError::RegisterError;
        }
    }
    rx_frame->word = *result;
#endif

    // Verify CRC if requested
    if (verify_crc) {
        if (!verify_frame_crc(*rx_frame)) {
#if __cpp_lib_expected >= 202202L
            return std::unexpected(DriverError::CRCError);
#else
            return DriverError::CRCError;
#endif
        }
    }

    // Check SPI status in reply
    auto status_result = check_spi_status(*rx_frame);
    if (!status_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(status_result.error());
#else
        return status_result.error_code;
#endif
    }

    return DriverResult<void>();
}

DriverResult<void> Driver::check_spi_status(const SPIFrame& rx_frame) noexcept {
    SPIStatus status = static_cast<SPIStatus>(rx_frame.rx_fields.status);

    switch (status) {
        case SPIStatus::NO_ERROR:
            return DriverResult<void>();
        case SPIStatus::SPI_FRAME_ERROR:
#if __cpp_lib_expected >= 202202L
            return std::unexpected(DriverError::SPIFrameError);
#else
            return DriverError::SPIFrameError;
#endif
        case SPIStatus::CRC_ERROR:
#if __cpp_lib_expected >= 202202L
            return std::unexpected(DriverError::CRCError);
#else
            return DriverError::CRCError;
#endif
        case SPIStatus::WRITE_RO_REG:
#if __cpp_lib_expected >= 202202L
            return std::unexpected(DriverError::WriteToReadOnly);
#else
            return DriverError::WriteToReadOnly;
#endif
        case SPIStatus::INTERNAL_BUS_FAULT:
#if __cpp_lib_expected >= 202202L
            return std::unexpected(DriverError::RegisterError);
#else
            return DriverError::RegisterError;
#endif
        default:
#if __cpp_lib_expected >= 202202L
            return std::unexpected(DriverError::RegisterError);
#else
            return DriverError::RegisterError;
#endif
    }
}

DriverResult<bool> Driver::is_channel_parallel(Channel channel) noexcept {
    auto init_check = check_initialized();
    if (!init_check) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(init_check.error());
#else
        return init_check.error_code;
#endif
    }

    if (!is_valid_channel_internal(channel)) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(DriverError::InvalidChannel);
#else
        return DriverError::InvalidChannel;
#endif
    }

    // Read CH_CTRL to check parallel configuration bits
    auto ch_ctrl_result = read_register(CentralReg::CH_CTRL, true);
    if (!ch_ctrl_result) {
#if __cpp_lib_expected >= 202202L
        return std::unexpected(ch_ctrl_result.error());
#else
        return ch_ctrl_result.error_code;
#endif
    }

    uint8_t ch_index = to_index(channel);
    uint16_t ch_ctrl = *ch_ctrl_result;

    // Check which parallel pair this channel belongs to
    bool is_parallel = false;
    switch (ch_index) {
        case 0:
        case 3:
            is_parallel = (ch_ctrl & CH_CTRL::CH_PAR_0_3) != 0;
            break;
        case 1:
        case 2:
            is_parallel = (ch_ctrl & CH_CTRL::CH_PAR_1_2) != 0;
            break;
        case 4:
        case 5:
            is_parallel = (ch_ctrl & CH_CTRL::CH_PAR_4_5) != 0;
            break;
        default:
            is_parallel = false;
            break;
    }

    return is_parallel;
}

} // namespace TLE92466ED