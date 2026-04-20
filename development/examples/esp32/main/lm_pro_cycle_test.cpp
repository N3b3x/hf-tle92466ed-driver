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
 *   Datasheet anchors (values cribbed from `WhValveCatalog::kSpecLMPro_24V`):
 *     * Coil resistance ~148 Ω  → ~115 mA at full open at 24 V
 *     * Maximum PWM frequency on TLE92466ED: 4 kHz (datasheet limit).
 *       The chip cannot drive above 4 kHz, so the PWM is inherently
 *       audible. Use slew-rate-control + dither for the smoothest
 *       audible output you can get.
 *     * No additional dither benefit from the LM-Pro coil itself
 *       (linear-motor design with low static friction).
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
// Datasheet PWM range is 110 Hz – 4 kHz (Equation 4 / parameter table).
// Above 4 kHz the chip's PWM frequency controller cannot regulate. Use
// the highest setting it accepts (250 µs = 4 kHz) for the least audible
// drive — getting truly out of audible range (>20 kHz) is not possible
// with this chip family.
constexpr float    kPwmPeriod_us       = 250.0f;       ///< 250 µs → 4 kHz (datasheet ceiling)

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

// Readback API notes (datasheet §4.10.2 averaged feedback values):
//   - `GetCurrentSetpoint()`       → reads the setpoint we wrote, in mA.
//   - `GetVbatVoltage / GetVioVoltage` → device-level supply rails in mV
//                                        (decoded from FB_VOLTAGE1/2's
//                                        22-bit reply frames).
//   - `GetAverageCurrent()`        → reads BOTH FB_I_AVG (I_AVG_MANT)
//                                    and FB_DC (TP_MANT) per channel and
//                                    decodes Iavg = 4 A × I_AVG_MANT /
//                                    TP_MANT. Returns mA (signed values
//                                    clamped to 0 in the unsigned API).
//   - `GetDutyCycle()`             → reads FB_DC and decodes
//                                    DC = TO_MANT / TP_MANT. Returns
//                                    permyriad (0..10000 = 0..100.00 %).

//==============================================================================
// TELEMETRY TASK  (10 Hz scrape of channel + device diagnostics)
//==============================================================================

static volatile bool g_telemetry_running = false;

static void telemetry_task(void* /*arg*/) noexcept {
    ESP_LOGI(TAG, "[telemetry] starting (period=%u ms)",
             static_cast<unsigned>(cfg::kTelemetryPeriod_ms));

    uint32_t tick_count    = 0;
    uint32_t fault_dump_n  = 0;

    // For the FB_DC + FB_I_AVG raw dump we need to know the per-channel
    // base address. From the driver internals: each per-channel register
    // bank lives at  ChannelBase[ch] + ChannelReg::offset. CH5's base
    // is 0x0030 (per the chip's non-sequential bank table), so:
    //   FB_DC[CH5]    = 0x0030 + 0x0200 = 0x0230
    //   FB_I_AVG[CH5] = 0x0030 + 0x0202 = 0x0232
    constexpr uint16_t kFbDcAddr   = 0x0230;
    constexpr uint16_t kFbIavgAddr = 0x0232;

    // Host-side moving averages for current and duty.
    //   - The chip's FB_I_AVG averages over Tmeas ≈ 20 ms (TDither), so
    //     our 100 ms telemetry tick already gets one fully-averaged
    //     sample; an 8-tap MA on the host side just smooths bench noise.
    //   - FB_DC's TO_MANT/TP_MANT ratio is a single-PWM-cycle snapshot
    //     (chip uses EXP=1 for the duty encoding regardless of what we
    //     program) so duty bounces rapidly between 0 % and 100 %; a
    //     longer host-side MA gives a usable steady-state read for
    //     comparison against the setpoint.
    constexpr int kMaTaps = 8;
    int   ma_idx_i        = 0;
    int   ma_idx_d        = 0;
    int   ma_count_i      = 0;
    int   ma_count_d      = 0;
    int32_t  ma_i_buf[kMaTaps]    = {};
    uint32_t ma_d_buf[kMaTaps]    = {};
    auto push_ma_i = [&](int32_t s) {
        ma_i_buf[ma_idx_i] = s;
        ma_idx_i = (ma_idx_i + 1) % kMaTaps;
        if (ma_count_i < kMaTaps) ++ma_count_i;
    };
    auto push_ma_d = [&](uint32_t s) {
        ma_d_buf[ma_idx_d] = s;
        ma_idx_d = (ma_idx_d + 1) % kMaTaps;
        if (ma_count_d < kMaTaps) ++ma_count_d;
    };
    auto mean_i = [&]() -> int32_t {
        if (ma_count_i == 0) return 0;
        int64_t sum = 0;
        for (int k = 0; k < ma_count_i; ++k) sum += ma_i_buf[k];
        return static_cast<int32_t>(sum / ma_count_i);
    };
    auto mean_d = [&]() -> uint32_t {
        if (ma_count_d == 0) return 0;
        uint64_t sum = 0;
        for (int k = 0; k < ma_count_d; ++k) sum += ma_d_buf[k];
        return static_cast<uint32_t>(sum / ma_count_d);
    };

    while (g_telemetry_running && g_driver) {
        const auto sp_r      = g_driver->GetCurrentSetpoint(cfg::kChannel, false);
        const auto i_avg_r   = g_driver->GetAverageCurrent  (cfg::kChannel, false);
        const auto duty_r    = g_driver->GetDutyCycle       (cfg::kChannel);
        const auto diag_r    = g_driver->GetChannelDiagnostics(cfg::kChannel);

        // Raw FB register dump — proves whether the chip is actually
        // populating the feedback path or whether our decode is wrong.
        const auto fb_dc_raw_r   = g_driver->ReadRegister(kFbDcAddr);
        const auto fb_iavg_raw_r = g_driver->ReadRegister(kFbIavgAddr);

        const uint16_t setpoint_ma = sp_r    ? *sp_r    : 0;
        const uint16_t i_avg_ma    = i_avg_r ? *i_avg_r : 0;
        const uint16_t duty_raw    = duty_r  ? *duty_r  : 0;
        const uint32_t fb_dc_raw   = fb_dc_raw_r   ? *fb_dc_raw_r   : 0;
        const uint32_t fb_iavg_raw = fb_iavg_raw_r ? *fb_iavg_raw_r : 0;

        // Push raw samples into the host-side moving-average filter.
        push_ma_i(static_cast<int32_t>(i_avg_ma));
        push_ma_d(static_cast<uint32_t>(duty_raw));
        const int32_t  i_avg_ma_smooth   = mean_i();
        const uint32_t duty_raw_smooth   = mean_d();

        if (diag_r) {
            const auto& d = *diag_r;
            const float duty_pct_raw    = static_cast<float>(duty_raw)        / 100.0f;
            const float duty_pct_smooth = static_cast<float>(duty_raw_smooth) / 100.0f;
            // Compare raw vs MA-smoothed alongside setpoint so it's
            // immediately obvious whether the chip is tracking the
            // request and where the noise floor lives.
            ESP_LOGI(TAG,
                     "[t=%4u s+%03u] sp=%3u mA  "
                     "i=%3d mA (avg %3d)  duty=%5.2f%% (avg %5.2f%%)  "
                     "FB_DC=0x%06" PRIX32 " FB_IAVG=0x%06" PRIX32 "  "
                     "OC=%d SG=%d OL=%d OT=%d",
                     static_cast<unsigned>(tick_count * cfg::kTelemetryPeriod_ms / 1000U),
                     static_cast<unsigned>(tick_count * cfg::kTelemetryPeriod_ms % 1000U),
                     setpoint_ma,
                     static_cast<int>(i_avg_ma), static_cast<int>(i_avg_ma_smooth),
                     static_cast<double>(duty_pct_raw),
                     static_cast<double>(duty_pct_smooth),
                     fb_dc_raw, fb_iavg_raw,
                     d.overcurrent, d.short_to_ground, d.open_load, d.over_temperature);
            (void)d;  // remaining flags suppressed for line length
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

    ChannelConfig ch_cfg{};
    ch_cfg.mode                  = ChannelMode::ICC;
    ch_cfg.current_setpoint_ma   = 0;                          // start at 0
    ch_cfg.slew_rate             = SlewRate::MEDIUM_2V5_US;
    ch_cfg.diag_current          = DiagCurrent::I_80UA;
    ch_cfg.open_load_threshold   = 3;                          // 3/8 of FS
    ch_cfg.olsg_warning_enabled  = true;
    ch_cfg.deep_dither_enabled   = false;
    // Skip the legacy step/flat fields so ConfigureChannel doesn't write
    // dither registers — we do that explicitly below with the high-level
    // ConfigureDither() API.
    ch_cfg.dither_step_size      = 0;
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

    // Configure dither — required for telemetry even on the LM-Pro.
    //   Per datasheet §4.4.2 + §5.3.3.7, in ICC mode the chip's averaged
    //   feedback measurement period Tmeas equals the dither period
    //   TDither, which itself is built from a per-channel reference
    //   clock tref_clk programmed in DITHER_CLK_DIV. The chip POR
    //   default of DITHER_CLK_DIV = 0x0000 makes tref_clk = 0 and the
    //   feedback averager NEVER runs (FB_DC / FB_I_AVG / FB_VBAT stay 0).
    //
    //   The high-level `ConfigureDither(amplitude_ma, frequency_hz)` API
    //   now also writes DITHER_CLK_DIV automatically, picking a tref_clk
    //   that lands the requested frequency with the helper's default
    //   STEPS=16 / FLAT=2 sub-cycle counts.
    //
    //   The LM-Pro is a low-static-friction linear motor that does NOT
    //   need dither for plunger movement, AND it operates in a small
    //   current range (0–115 mA). A large dither here causes:
    //     - the chip's PWM duty to swing wildly (FB_DC reads bounce
    //       between 0 % and 100 % across the dither period),
    //     - the actual coil current to oscillate around the setpoint,
    //       making the readback noisy and the position unstable.
    //
    //   Use a tiny dither — small enough to not perturb position, large
    //   enough that the chip's averager has a non-zero Tmeas window:
    //     amplitude = 1 mA  (≈ 1 % of full-scale current)
    //     frequency = 50 Hz (TDither = 20 ms = 80 PWM cycles at 4 kHz,
    //                        plenty of cycles for a stable FB average)
    constexpr float kDitherAmplitude_mA = 1.0f;
    constexpr float kDitherFrequency_Hz = 50.0f;
    if (auto rc = g_driver->ConfigureDither(cfg::kChannel,
                                            kDitherAmplitude_mA,
                                            kDitherFrequency_Hz); !rc) {
        ESP_LOGE(TAG, "ConfigureDither failed: %u",
                 static_cast<unsigned>(rc.error()));
        return false;
    }
    ESP_LOGI(TAG,
             "✅ Dither: %.1f mA @ %.0f Hz "
             "(20 ms TDither = Tmeas → ~80 PWM cycles averaged per FB update)",
             static_cast<double>(kDitherAmplitude_mA),
             static_cast<double>(kDitherFrequency_Hz));

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

    // Explicitly enable the channel via CH_CTRL.EN_CHx. Per datasheet
    // §4.7.x and §4.8.1, "the respective channel must be activated by
    // setting <EN_CH> bit to 1 and a target current setpoint value
    // different to zero" before the chip will drive the output stage.
    if (auto rc = g_driver->EnableChannel(cfg::kChannel, true); !rc) {
        ESP_LOGE(TAG, "EnableChannel(%s, true) failed: %u",
                 channel_name(cfg::kChannel),
                 static_cast<unsigned>(rc.error()));
        return false;
    }
    ESP_LOGI(TAG, "✅ Channel %s enabled (CH_CTRL.EN_CHx set)",
             channel_name(cfg::kChannel));

    // Clear the feedback-freeze register so the chip starts updating
    // FB_DC / FB_I_AVG / FB_VBAT for every channel. Per datasheet §4.10
    // and FB_FRZ register description (0x0007): "Setting the <CH> bit in
    // the FB_FRZ register to 0 ... enables the update of the feedback
    // values." Reset value of FB_FRZ on this chip leaves channels
    // frozen, which is why FB_* reads come back as 0 even when the chip
    // is actively driving the coil. Driver doesn't expose an API for
    // this register, so we write it directly.
    constexpr uint16_t kFbFrzAddr = 0x0007;
    if (auto rd = g_driver->ReadRegister(kFbFrzAddr); rd) {
        ESP_LOGI(TAG, "  FB_FRZ before clear: 0x%06" PRIX32, *rd);
    }
    if (auto wr = g_driver->WriteRegister(kFbFrzAddr, 0x0000); !wr) {
        ESP_LOGW(TAG, "WriteRegister(FB_FRZ, 0) failed: %u",
                 static_cast<unsigned>(wr.error()));
    } else {
        ESP_LOGI(TAG, "✅ FB_FRZ cleared — feedback update enabled on all channels");
    }
    if (auto rd = g_driver->ReadRegister(kFbFrzAddr); rd) {
        ESP_LOGI(TAG, "  FB_FRZ after  clear: 0x%06" PRIX32, *rd);
    }

    return true;
}

//==============================================================================
// SETPOINT-TRACKING CHARACTERIZATION
//==============================================================================

/// Per-setpoint sampling parameters.
namespace track {
constexpr uint32_t kSettleMs       = 300;   ///< Wait after setpoint write before sampling
constexpr int      kNumSamples     = 50;    ///< Samples per setpoint
constexpr uint32_t kSampleGapMs    = 10;    ///< 10 ms between samples → 500 ms window
constexpr float    kBandPercent    = 2.0f;  ///< Pass / fail tracking band (±%)
}  // namespace track

/// Apply `setpoint_ma`, settle, sample current N times, print summary.
static void characterize_setpoint(uint16_t setpoint_ma) noexcept {
    if (auto rc = g_driver->SetCurrentSetpoint(cfg::kChannel, setpoint_ma, false); !rc) {
        ESP_LOGE(TAG, "SetCurrentSetpoint(%u mA) failed: %u",
                 setpoint_ma, static_cast<unsigned>(rc.error()));
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(track::kSettleMs));

    int32_t  i_min       = INT32_MAX;
    int32_t  i_max       = INT32_MIN;
    int64_t  i_sum       = 0;
    int64_t  i_sumsq     = 0;
    int      n_collected = 0;
    int      n_in_band   = 0;
    const int32_t band_lo = static_cast<int32_t>(setpoint_ma)
                          - static_cast<int32_t>(setpoint_ma * track::kBandPercent / 100.0f + 0.5f);
    const int32_t band_hi = static_cast<int32_t>(setpoint_ma)
                          + static_cast<int32_t>(setpoint_ma * track::kBandPercent / 100.0f + 0.5f);

    for (int s = 0; s < track::kNumSamples; ++s) {
        const auto i_r = g_driver->GetAverageCurrent(cfg::kChannel, false);
        if (i_r) {
            const int32_t i_ma = static_cast<int32_t>(*i_r);
            ++n_collected;
            i_sum   += i_ma;
            i_sumsq += static_cast<int64_t>(i_ma) * i_ma;
            if (i_ma < i_min) i_min = i_ma;
            if (i_ma > i_max) i_max = i_ma;
            if (setpoint_ma > 0 &&
                i_ma >= band_lo && i_ma <= band_hi) ++n_in_band;
        }
        vTaskDelay(pdMS_TO_TICKS(track::kSampleGapMs));
    }

    if (n_collected == 0) {
        ESP_LOGW(TAG, "  sp=%3u mA — no samples collected!", setpoint_ma);
        return;
    }

    const double mean      = static_cast<double>(i_sum) / n_collected;
    const double variance  = static_cast<double>(i_sumsq) / n_collected - mean * mean;
    const double stddev    = (variance > 0.0) ? std::sqrt(variance) : 0.0;
    const double err_abs   = mean - static_cast<double>(setpoint_ma);
    const double err_pct   = (setpoint_ma > 0)
        ? (err_abs / static_cast<double>(setpoint_ma) * 100.0)
        : 0.0;
    const double in_band_pct =
        (setpoint_ma > 0) ? (100.0 * n_in_band / n_collected) : 100.0;

    // Pass/fail flag against the ±band_percent tolerance.
    const bool pass = setpoint_ma == 0 ||
                      (std::fabs(err_pct) <= static_cast<double>(track::kBandPercent) &&
                       in_band_pct >= 70.0);

    ESP_LOGI(TAG,
             "%s sp=%3u mA  mean=%6.2f mA  err=%+5.2f mA (%+5.2f %%)  "
             "stddev=%5.2f mA  range=[%3d..%3d]  in-±%.1f%%-band=%3.0f%% (%d/%d)",
             pass ? "  ✅" : "  ⚠️",
             setpoint_ma,
             mean,
             err_abs, err_pct,
             stddev,
             static_cast<int>(i_min), static_cast<int>(i_max),
             static_cast<double>(track::kBandPercent),
             in_band_pct,
             n_in_band, n_collected);
}

//==============================================================================
// CYCLE DRIVER
//==============================================================================

static void run_cycle(uint32_t cycle_index) noexcept {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "═══ Cycle %u: tracking-accuracy sweep %u → %u mA "
                  "(±%.1f %% band, %d samples per step over %u ms) ═══",
             static_cast<unsigned>(cycle_index + 1U),
             cfg::kRampMin_mA, cfg::kRampMax_mA,
             static_cast<double>(track::kBandPercent),
             track::kNumSamples,
             static_cast<unsigned>(track::kNumSamples * track::kSampleGapMs));

    // Disable the steady-state telemetry task during characterization so
    // its concurrent SPI reads don't perturb our sample timing.
    g_telemetry_running = false;
    vTaskDelay(pdMS_TO_TICKS(50));

    for (uint16_t sp = cfg::kRampMin_mA; sp <= cfg::kRampMax_mA; sp += cfg::kRampStep_mA) {
        characterize_setpoint(sp);
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "═══ Cycle %u: ramp DOWN ═══", static_cast<unsigned>(cycle_index + 1U));
    for (int32_t sp = static_cast<int32_t>(cfg::kRampMax_mA);
         sp >= static_cast<int32_t>(cfg::kRampMin_mA); sp -= cfg::kRampStep_mA) {
        characterize_setpoint(static_cast<uint16_t>(sp));
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
