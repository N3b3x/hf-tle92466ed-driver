/**
 * @file lm_pro_cycle_test.cpp
 * @brief Real-hardware Parker LM-Pro proportional valve cycle test on
 *        TLE92466ED CH5 (driver channel 6 in 1-indexed nomenclature).
 *
 * @details
 *   Drives a single Parker LM-Pro 24 V proportional valve back and forth
 *   through its full 0 → 115 mA full-flow current range, with continuous
 *   per-step telemetry (setpoint readback, per-channel fault flags) and
 *   periodic device-level supply / temperature scrape so we can verify
 *   on the bench that:
 *
 *     * the chip is communicating cleanly (this is the same test path
 *       that exposed the inverted-RESN bug earlier),
 *     * the setpoint we write is what the chip echoes back,
 *     * no spurious fault flags fire as we ramp,
 *     * EN is the only thing gating the output stage (we toggle it on
 *       once at boot via Driver::Enable()),
 *     * the LM-Pro plunger physically moves in step with the ramp
 *       (audible / observable on the test rig).
 *
 *   See "Note on readback APIs" inside this file regarding the live
 *   FB_I_AVG / FB_DC readbacks (a known driver decoding issue — the
 *   chip is driving correctly, the readback path needs a fix).
 *
 *   Datasheet anchors (values cribbed from `WhValveCatalog::kSpecLMPro_24V`):
 *     * Coil resistance ~148 Ω  → ~115 mA at full open at 24 V
 *     * Recommended PWM frequency: ≥ 5 kHz
 *     * No benefit from dither (linear-motor design)
 *
 *   Default cycle profile: triangle wave
 *     0 → 115 mA in 10 mA steps, 1 s per step
 *     115 → 0 mA reverse, same cadence
 *     2 s hold at 0 between cycles
 *     telemetry @ 10 Hz throughout
 *
 *   Override at the top of the file for a different LM-Pro variant or
 *   to run a single-shot ramp.
 *
 * @par Wiring (matches the project's ESP32-S3 reference board)
 *   See `main/esp32_tle92466ed_test_config.hpp`. SPI3_HOST, MISO=GPIO35,
 *   MOSI=GPIO37, SCLK=GPIO36, CS=GPIO4, RESN=GPIO6, EN=GPIO5,
 *   FAULTN=GPIO16. The LM-Pro coil sits between OUT5 and VBAT (low-side
 *   sourcing).
 *
 * @par Build
 *   From examples/esp32:
 *       ./scripts/build_app.sh lm_pro_cycle_test Debug
 *       ./scripts/flash_app.sh flash_monitor lm_pro_cycle_test Debug
 *
 * @author HardFOC
 * @date   2026
 */

#include <cinttypes>
#include <cmath>
#include <memory>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "tle92466ed.hpp"
#include "esp32_tle92466ed_bus.hpp"
#include "esp32_tle92466ed_test_config.hpp"

// NB: do NOT alias `tle` — that namespace already exists in the driver
// (`tle::expected` / `tle::unexpected`). Use the existing convention.
using namespace tle92466ed;
using namespace TLE92466ED_TestConfig;

static const char* TAG = "LMProCycle";

//==============================================================================
// TEST CONFIGURATION  (edit here for a different LM-Pro variant / cadence)
//==============================================================================

namespace cfg {

// ─── Channel under test ────────────────────────────────────────────────
constexpr Channel kChannel = Channel::CH5;   ///< 6th TLE channel (1-indexed)

// ─── Datasheet specs (LM-Pro 24 V) ─────────────────────────────────────
constexpr uint16_t kFullFlowCurrent_mA = 115;          ///< 115 mA at 24 V into 148 Ω
constexpr uint16_t kSafetyCap_mA       = 130;          ///< Hard hardware-side limit
constexpr float    kPwmPeriod_us       = 200.0f;       ///< 200 µs → 5 kHz (datasheet floor)

// ─── Cycle profile ─────────────────────────────────────────────────────
constexpr uint16_t kRampMin_mA       = 0;
constexpr uint16_t kRampMax_mA       = kFullFlowCurrent_mA;   // 0..115 mA
constexpr uint16_t kRampStep_mA      = 10;
constexpr uint32_t kStepDwell_ms     = 1000;                  ///< 1 s per step
constexpr uint32_t kCycleHold_ms     = 2000;                  ///< 2 s hold at 0 between cycles
constexpr uint32_t kCycleCount       = 0;                     ///< 0 = run forever

// ─── Telemetry cadence ─────────────────────────────────────────────────
constexpr uint32_t kTelemetryPeriod_ms = 100;                 ///< 10 Hz

}  // namespace cfg

static_assert(cfg::kRampMax_mA <= cfg::kSafetyCap_mA,
              "Ramp peak exceeds safety cap; bump kSafetyCap_mA or lower kRampMax_mA.");

//==============================================================================
// GLOBAL RESOURCES
//==============================================================================

static std::unique_ptr<Esp32Tle92466edSpiBus>          g_bus;
static std::unique_ptr<Driver<Esp32Tle92466edSpiBus>> g_driver;

//==============================================================================
// HELPERS
//==============================================================================

static const char* channel_name(Channel ch) noexcept {
    switch (ch) {
        case Channel::CH0: return "CH0 (OUT0)";
        case Channel::CH1: return "CH1 (OUT1)";
        case Channel::CH2: return "CH2 (OUT2)";
        case Channel::CH3: return "CH3 (OUT3)";
        case Channel::CH4: return "CH4 (OUT4)";
        case Channel::CH5: return "CH5 (OUT5)";
        default:           return "?";
    }
}

// Note on readback APIs:
//   - `GetCurrentSetpoint`           → reads the setpoint we just wrote;
//                                      returns the live setpoint in mA. ✅
//   - `GetVbatVoltage / GetVioVoltage` → return engineering units (mV);
//                                        confirmed working on the bench. ✅
//   - `GetAverageCurrent (FB_I_AVG)` / `GetDutyCycle (FB_DC)`
//     ── return 0 in single-channel ICC mode for CH5 even when the chip
//        is visibly driving the coil. Verified by physical plunger
//        movement on the LM-Pro test rig. Likely a driver-level register
//        decoding issue (FB_* registers may be 22-bit reply frames that
//        need different parsing than the 16-bit ReadRegister path
//        currently uses). Tracked as a follow-up driver fix; this test
//        intentionally still calls the APIs so the regression is visible
//        if anyone investigates it later.

//==============================================================================
// TELEMETRY TASK  (10 Hz scrape of channel + device diagnostics)
//==============================================================================

static volatile bool g_telemetry_running = false;

static void telemetry_task(void* /*arg*/) noexcept {
    ESP_LOGI(TAG, "[telemetry] starting (period=%u ms)",
             static_cast<unsigned>(cfg::kTelemetryPeriod_ms));

    uint32_t tick_count    = 0;
    uint32_t fault_dump_n  = 0;

    while (g_telemetry_running && g_driver) {
        // ─── Live measurements (decoded engineering units) ─────────────
        // Use the *dedicated* APIs for live values; they read the right
        // FB_I_AVG / FB_DC / FB_VOLTAGE registers and return engineering
        // units. `GetChannelDiagnostics` is for the per-flag fault state.
        const auto sp_r      = g_driver->GetCurrentSetpoint(cfg::kChannel, false);
        const auto i_avg_r   = g_driver->GetAverageCurrent  (cfg::kChannel, false);
        const auto duty_r    = g_driver->GetDutyCycle       (cfg::kChannel);
        const auto diag_r    = g_driver->GetChannelDiagnostics(cfg::kChannel);

        const uint16_t setpoint_ma = sp_r    ? *sp_r    : 0;
        const uint16_t i_avg_ma    = i_avg_r ? *i_avg_r : 0;
        const uint16_t duty_raw    = duty_r  ? *duty_r  : 0;

        if (diag_r) {
            const auto& d = *diag_r;
            // Compute duty as a percentage of full-scale (0xFFFF = 100 %).
            const float duty_pct = (static_cast<float>(duty_raw) * 100.0f)
                                 / 65535.0f;
            ESP_LOGI(TAG,
                     "[t=%4u s+%03u] sp=%3u mA  i_avg=%3u mA  duty=%5.1f%% (0x%04X)  "
                     "OC=%d SG=%d OL=%d OT=%d  warn{ot=%d ireg=%d preg=%d olsg=%d}",
                     static_cast<unsigned>(tick_count * cfg::kTelemetryPeriod_ms / 1000U),
                     static_cast<unsigned>(tick_count * cfg::kTelemetryPeriod_ms % 1000U),
                     setpoint_ma, i_avg_ma,
                     static_cast<double>(duty_pct), duty_raw,
                     d.overcurrent, d.short_to_ground, d.open_load, d.over_temperature,
                     d.ot_warning, d.current_regulation_warning,
                     d.pwm_regulation_warning, d.olsg_warning);
        } else {
            ESP_LOGW(TAG, "[telemetry] GetChannelDiagnostics failed (err=%u)",
                     static_cast<unsigned>(diag_r.error()));
        }

        // Every 10th tick (~1 Hz) also dump device-level status / supply rails
        // in engineering units (GetVbatVoltage / GetVioVoltage return mV).
        if ((tick_count % 10U) == 0U) {
            const auto st     = g_driver->GetDeviceStatus();
            const auto vbat_r = g_driver->GetVbatVoltage();
            const auto vio_r  = g_driver->GetVioVoltage();
            const float vbat_v = vbat_r ? (*vbat_r / 1000.0f) : 0.0f;
            const float vio_v  = vio_r  ? (*vio_r  / 1000.0f) : 0.0f;
            if (st) {
                ESP_LOGI(TAG,
                         "  device: cfg=%d init=%d any_fault=%d  "
                         "VBAT=%5.2f V (uv=%d ov=%d)  "
                         "VIO=%4.2f V (uv=%d ov=%d)  "
                         "vdd[uv=%d ov=%d]  ot[warn=%d err=%d]",
                         st->config_mode, st->init_done, st->any_fault,
                         static_cast<double>(vbat_v), st->vbat_uv, st->vbat_ov,
                         static_cast<double>(vio_v),  st->vio_uv,  st->vio_ov,
                         st->vdd_uv,  st->vdd_ov,
                         st->ot_warning, st->ot_error);
            }
        }

        // Every 100th tick (~10 s) dump the full FaultReport.
        if ((tick_count % 100U) == 0U) {
            const auto fr = g_driver->GetAllFaults();
            if (fr) {
                ++fault_dump_n;
                ESP_LOGI(TAG, "  ─── FaultReport #%u (any=%d) ─────────────────────────",
                         static_cast<unsigned>(fault_dump_n), fr->any_fault);
                ESP_LOGI(TAG, "    supplies: vbat[uv=%d ov=%d] vio[uv=%d ov=%d] vdd[uv=%d ov=%d]",
                         fr->vbat_uv, fr->vbat_ov, fr->vio_uv, fr->vio_ov, fr->vdd_uv, fr->vdd_ov);
                ESP_LOGI(TAG, "    internal: vr_iref[uv=%d ov=%d] vdd2v5[uv=%d ov=%d] ref[uv=%d ov=%d] vpre_ov=%d hvadc=%d",
                         fr->vr_iref_uv, fr->vr_iref_ov, fr->vdd2v5_uv, fr->vdd2v5_ov,
                         fr->ref_uv, fr->ref_ov, fr->vpre_ov, fr->hvadc_err);
                ESP_LOGI(TAG, "    system  : clock=%d spi_wd=%d ot[err=%d warn=%d] reset[por=%d]",
                         fr->clock_fault, fr->spi_wd_error,
                         fr->ot_error, fr->ot_warning, fr->por_event);
            }
        }

        ++tick_count;
        vTaskDelay(pdMS_TO_TICKS(cfg::kTelemetryPeriod_ms));
    }

    ESP_LOGI(TAG, "[telemetry] stopping after %u ticks", static_cast<unsigned>(tick_count));
    vTaskDelete(nullptr);
}

//==============================================================================
// SETUP
//==============================================================================

static bool create_bus_and_driver() noexcept {
    g_bus = CreateEsp32Tle92466edSpiBus();
    if (!g_bus) {
        ESP_LOGE(TAG, "Failed to create Esp32Tle92466edSpiBus");
        return false;
    }

    g_driver = std::make_unique<Driver<Esp32Tle92466edSpiBus>>(*g_bus);
    if (!g_driver) {
        ESP_LOGE(TAG, "Failed to allocate Driver");
        g_bus.reset();
        return false;
    }
    return true;
}

static bool initialize_driver() noexcept {
    ESP_LOGI(TAG, "Initializing TLE92466ED driver...");
    auto rc = g_driver->Init();
    if (!rc) {
        ESP_LOGE(TAG, "Driver Init failed: %u", static_cast<unsigned>(rc.error()));
        return false;
    }
    ESP_LOGI(TAG, "✅ Driver initialized");
    return true;
}

static bool configure_channel() noexcept {
    ESP_LOGI(TAG, "Configuring %s for ICC + %.0f µs PWM (%.1f kHz) full-flow=%u mA",
             channel_name(cfg::kChannel),
             static_cast<double>(cfg::kPwmPeriod_us),
             static_cast<double>(1000.0f / cfg::kPwmPeriod_us),
             cfg::kFullFlowCurrent_mA);

    // ConfigureChannel pushes mode/slew/diag/PWM in one transaction.
    ChannelConfig ch_cfg{};
    ch_cfg.mode                  = ChannelMode::ICC;
    ch_cfg.current_setpoint_ma   = 0;                          // start at 0
    ch_cfg.slew_rate             = SlewRate::MEDIUM_2V5_US;
    ch_cfg.diag_current          = DiagCurrent::I_80UA;
    ch_cfg.open_load_threshold   = 3;                          // 3/8 of FS
    ch_cfg.olsg_warning_enabled  = true;
    ch_cfg.deep_dither_enabled   = false;
    ch_cfg.dither_step_size      = 0;                          // dither off (LM-Pro)
    ch_cfg.dither_steps          = 0;
    ch_cfg.dither_flat           = 0;

    if (auto rc = g_driver->ConfigureChannel(cfg::kChannel, ch_cfg); !rc) {
        ESP_LOGE(TAG, "ConfigureChannel failed: %u", static_cast<unsigned>(rc.error()));
        return false;
    }

    if (auto rc = g_driver->ConfigurePwmPeriod(cfg::kChannel, cfg::kPwmPeriod_us); !rc) {
        ESP_LOGE(TAG, "ConfigurePwmPeriod failed: %u", static_cast<unsigned>(rc.error()));
        return false;
    }

    return true;
}

static bool enter_mission_and_enable() noexcept {
    if (auto rc = g_driver->EnterMissionMode(); !rc) {
        ESP_LOGE(TAG, "EnterMissionMode failed: %u", static_cast<unsigned>(rc.error()));
        return false;
    }
    ESP_LOGI(TAG, "✅ Mission mode entered");

    // Drive the chip's physical EN pin HIGH — Driver::Enable() asserts EN
    // via the bus's GpioSet(EN, ACTIVE). Without EN HIGH the output stage
    // stays gated off and the coil sees no current regardless of the
    // setpoint.
    if (auto rc = g_driver->Enable(); !rc) {
        ESP_LOGE(TAG, "Driver::Enable() failed: %u", static_cast<unsigned>(rc.error()));
        return false;
    }
    ESP_LOGI(TAG, "✅ EN pin HIGH (output stage powered)");

    // NOTE: we deliberately do NOT call EnableChannel() here. The proven
    // solenoid_control_test in this same folder skips it too — the chip
    // auto-enables a channel the moment SetCurrentSetpoint() writes a
    // non-zero current. An explicit EnableChannel write to CH_CTRL can
    // race with the OP_MODE bit and silently knock the chip out of
    // Mission Mode, after which setpoints are accepted but never applied
    // (visible symptom: GetAverageCurrent / GetDutyCycle stuck at 0
    // even though the coil is being driven).

    return true;
}

//==============================================================================
// CYCLE DRIVER
//==============================================================================

static void run_cycle(uint32_t cycle_index) noexcept {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "═══ Cycle %u: ramp UP %u → %u mA ═══════════════════════════",
             static_cast<unsigned>(cycle_index + 1U),
             cfg::kRampMin_mA, cfg::kRampMax_mA);

    for (uint16_t sp = cfg::kRampMin_mA; sp <= cfg::kRampMax_mA; sp += cfg::kRampStep_mA) {
        ESP_LOGI(TAG, ">>> setpoint = %3u mA", sp);
        if (auto rc = g_driver->SetCurrentSetpoint(cfg::kChannel, sp, false); !rc) {
            ESP_LOGE(TAG, "SetCurrentSetpoint(%u mA) failed: %u",
                     sp, static_cast<unsigned>(rc.error()));
        }
        vTaskDelay(pdMS_TO_TICKS(cfg::kStepDwell_ms));
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "═══ Cycle %u: ramp DOWN %u → %u mA ═══════════════════════════",
             static_cast<unsigned>(cycle_index + 1U),
             cfg::kRampMax_mA, cfg::kRampMin_mA);

    for (int32_t sp = static_cast<int32_t>(cfg::kRampMax_mA);
         sp >= static_cast<int32_t>(cfg::kRampMin_mA); sp -= cfg::kRampStep_mA) {
        ESP_LOGI(TAG, ">>> setpoint = %3d mA", static_cast<int>(sp));
        if (auto rc = g_driver->SetCurrentSetpoint(cfg::kChannel, static_cast<uint16_t>(sp), false); !rc) {
            ESP_LOGE(TAG, "SetCurrentSetpoint(%d mA) failed: %u",
                     static_cast<int>(sp), static_cast<unsigned>(rc.error()));
        }
        vTaskDelay(pdMS_TO_TICKS(cfg::kStepDwell_ms));
    }

    ESP_LOGI(TAG, "═══ Cycle %u: hold at 0 mA for %u ms ════════════════════════",
             static_cast<unsigned>(cycle_index + 1U),
             static_cast<unsigned>(cfg::kCycleHold_ms));
    if (auto rc = g_driver->SetCurrentSetpoint(cfg::kChannel, 0U, false); !rc) {
        ESP_LOGE(TAG, "SetCurrentSetpoint(0) failed: %u", static_cast<unsigned>(rc.error()));
    }
    vTaskDelay(pdMS_TO_TICKS(cfg::kCycleHold_ms));
}

//==============================================================================
// app_main
//==============================================================================

extern "C" void app_main() {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   TLE92466ED — Parker LM-Pro 24 V proportional cycle test            ║");
    ESP_LOGI(TAG, "║   target: %s   profile: 0..%u..0 mA, %u mA step, %u ms/step          ║",
             channel_name(cfg::kChannel),
             cfg::kRampMax_mA,
             cfg::kRampStep_mA,
             static_cast<unsigned>(cfg::kStepDwell_ms));
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════╝");

    if (!create_bus_and_driver()) return;
    if (!initialize_driver())     return;
    if (!configure_channel())     return;
    if (!enter_mission_and_enable()) return;

    // Spawn the telemetry task BEFORE the ramp loop so we get the first
    // few snapshots while the channel is still at 0 mA.
    g_telemetry_running = true;
    BaseType_t r = xTaskCreate(telemetry_task, "tle_tlm", 4096, nullptr,
                               tskIDLE_PRIORITY + 2, nullptr);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "Failed to create telemetry task");
    }

    // Give the telemetry one tick before the first ramp.
    vTaskDelay(pdMS_TO_TICKS(cfg::kTelemetryPeriod_ms));

    uint32_t cycle = 0;
    while (cfg::kCycleCount == 0U || cycle < cfg::kCycleCount) {
        run_cycle(cycle);
        ++cycle;
    }

    ESP_LOGI(TAG, "All %u cycles complete — disabling channel and stopping telemetry",
             static_cast<unsigned>(cycle));
    (void)g_driver->SetCurrentSetpoint(cfg::kChannel, 0U, false);
    (void)g_driver->EnableChannel(cfg::kChannel, false);
    g_telemetry_running = false;
}
