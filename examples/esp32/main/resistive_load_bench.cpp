/**
 * @file resistive_load_bench.cpp
 * @brief Bench-level functional validation of the TLE92466ED driver against a
 *        300 \u03a9 resistive load on a single channel.
 *
 * Hardware:
 *   - TLE92466ED evaluation board
 *   - VBAT \u2248 12 V
 *   - 300 \u03a9 power resistor wired between channel output and GND
 *
 * Theoretical current at 100 % duty: I = VBAT / R = 12 V / 300 \u03a9 = 40 mA
 *
 * This example sweeps the ICC setpoint from 5 mA to 40 mA in 5 mA steps,
 * holds each step for 200 ms, and then reads the coherent feedback snapshot
 * via ReadChannelFeedback() (atomic FB_FRZ/FB_UPD path added in Phase 6).
 * It prints the programmed vs measured current and VBAT for each step so
 * the operator can quickly confirm \u00b12.5 mA accuracy without needing an
 * external multimeter.
 *
 * Exit criterion: measured current for every step is within \u00b12.5 mA of
 * the setpoint or limited by VBAT/R (whichever is smaller).
 */

#include <stdio.h>
#include <memory>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "tle92466ed.hpp"
#include "esp32_tle92466ed_bus.hpp"
#include "esp32_tle92466ed_test_config.hpp"

using namespace tle92466ed;
using namespace TLE92466ED_TestConfig;

static const char* TAG = "ResistiveBench";

static constexpr Channel  kBenchChannel          = Channel::CH0;
static constexpr uint32_t kLoadOhms              = 300U;
static constexpr uint16_t kSetpointStepsMa[]     = {5, 10, 15, 20, 25, 30, 35, 40};
static constexpr uint32_t kHoldMs                = 200U;
static constexpr int32_t  kAccuracyToleranceMa   = 2500; // \u00b12.5 mA

extern "C" void app_main() {
  ESP_LOGI(TAG, "TLE92466ED Resistive-Load Bench (300 \u03a9 on CH0)");

  auto bus = std::make_unique<Esp32Tle92466edSpiBus>();
  if (auto r = bus->Init(); !r) {
    ESP_LOGE(TAG, "Bus init failed: %d", static_cast<int>(r.error()));
    return;
  }

  auto drv = std::make_unique<Driver<Esp32Tle92466edSpiBus>>(*bus);

  if (auto r = drv->SetReset(true); !r)  { ESP_LOGE(TAG, "reset LOW failed");  return; }
  vTaskDelay(pdMS_TO_TICKS(5));
  if (auto r = drv->SetReset(false); !r) { ESP_LOGE(TAG, "reset HIGH failed"); return; }
  vTaskDelay(pdMS_TO_TICKS(5));

  if (auto r = drv->Init(); !r) {
    ESP_LOGE(TAG, "Driver Init failed: %d", static_cast<int>(r.error()));
    return;
  }

  // Report supply voltages + die temperature before enabling outputs.
  if (auto sv = drv->ReadAllSupplyVoltages(); sv) {
    ESP_LOGI(TAG, "VBAT=%u mV  VIO=%u mV  VDD=%u mV  Tj=%.1f C",
             sv->vbat_mV, sv->vio_mV, sv->vdd_mV,
             static_cast<double>(sv->temperature_c));
  }

  // Configure channel 0 for ICC with a reasonable PWM period + dither so the
  // ICC integrator has something to measure.
  if (auto r = drv->SetChannelMode(kBenchChannel, ChannelMode::ICC); !r) {
    ESP_LOGE(TAG, "SetChannelMode failed"); return;
  }
  if (auto r = drv->ConfigurePwmPeriod(kBenchChannel, 1000.0F /* 1 kHz */); !r) {
    ESP_LOGE(TAG, "ConfigurePwmPeriod failed"); return;
  }
  if (auto r = drv->ConfigureDither(kBenchChannel, /*amp=*/2.0F, /*freq=*/200.0F); !r) {
    ESP_LOGE(TAG, "ConfigureDither failed"); return;
  }

  if (auto r = drv->EnterMissionModeChecked(20U); !r) {
    ESP_LOGE(TAG, "EnterMissionModeChecked failed: %d", static_cast<int>(r.error()));
    return;
  }

  if (auto r = drv->EnableChannel(kBenchChannel, true); !r) {
    ESP_LOGE(TAG, "EnableChannel failed"); return;
  }

  ESP_LOGI(TAG, "Sweeping setpoints across 300 \u03a9 load (theoretical max \u2248 %lu mA)",
           static_cast<unsigned long>(12000U / kLoadOhms));

  uint32_t passes = 0;
  uint32_t total  = 0;
  for (uint16_t sp : kSetpointStepsMa) {
    ++total;
    if (auto r = drv->SetCurrentSetpoint(kBenchChannel, sp); !r) {
      ESP_LOGE(TAG, "SetCurrentSetpoint(%u) failed", sp); continue;
    }
    vTaskDelay(pdMS_TO_TICKS(kHoldMs));

    auto fb = drv->ReadChannelFeedback(kBenchChannel, /*timeout_ms=*/50);
    if (!fb) {
      ESP_LOGW(TAG, "  setpoint=%u mA: feedback read failed (%d)", sp,
               static_cast<int>(fb.error()));
      continue;
    }
    const int32_t measured = fb->avg_current_mA;
    const int32_t diff_ma  = measured - static_cast<int32_t>(sp);
    const bool    ok       = (std::abs(diff_ma) * 1000) <= kAccuracyToleranceMa;
    ESP_LOGI(TAG,
             "  SP=%2u mA  I_avg=%4ld mA  DC=%4u \u2030  VBAT=%lu mV  Imin=%ld mA  Imax=%ld mA  %s",
             sp, static_cast<long>(measured),
             fb->duty_cycle_permyriad,
             static_cast<unsigned long>(fb->avg_vbat_mV),
             static_cast<long>(fb->imin_mA),
             static_cast<long>(fb->imax_mA),
             ok ? "PASS" : "FAIL");
    if (ok) ++passes;
  }

  ESP_LOGI(TAG, "Resistive-load bench complete: %lu / %lu steps within tolerance",
           static_cast<unsigned long>(passes),
           static_cast<unsigned long>(total));

  (void)drv->DisableAllChannels();
  (void)drv->EnterConfigMode();
}
