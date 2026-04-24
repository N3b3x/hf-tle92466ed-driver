/**
 * @file diagnostics_and_bist.cpp
 * @brief Validates Phase 5 diagnostic APIs: SFF_BIST, PIN_STAT, FAULT_MASK,
 *        off-state OC/OL diagnostics and the supply-monitor self-test.
 *
 * Hardware:
 *   - TLE92466ED evaluation board
 *   - No load required (BIST + self-tests are purely internal)
 *
 * Expected output:
 *   - BIST: done=1, fail=0
 *   - PIN_STAT: reports EN and FAULTN state
 *   - Supply monitor self-test: all four phases report overall pass
 *   - Masking a fault removes its contribution to FAULTN
 */

#include <stdio.h>
#include <memory>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "tle92466ed.hpp"
#include "esp32_tle92466ed_bus.hpp"

using namespace tle92466ed;

static const char* TAG = "DiagBist";

extern "C" void app_main() {
  ESP_LOGI(TAG, "TLE92466ED Diagnostics + BIST Example");

  auto bus = std::make_unique<Esp32TLE92466ED_Bus>();
  if (auto r = bus->Init(); !r) {
    ESP_LOGE(TAG, "Bus init failed"); return;
  }
  auto drv = std::make_unique<Driver<Esp32TLE92466ED_Bus>>(*bus);

  (void)drv->SetReset(true);
  vTaskDelay(pdMS_TO_TICKS(5));
  (void)drv->SetReset(false);
  vTaskDelay(pdMS_TO_TICKS(5));

  if (auto r = drv->Init(); !r) {
    ESP_LOGE(TAG, "Init failed"); return;
  }

  // 1. PIN_STAT snapshot
  if (auto ps = drv->ReadPinStatus(); ps) {
    ESP_LOGI(TAG, "PIN_STAT  drv0=%d drv1=%d en=%d faultn=%d faultn_fb=%d (raw=0x%04X)",
             ps->drv0, ps->drv1, ps->en, ps->faultn_driver, ps->faultn_fb, ps->raw);
  }

  // 2. Operation state readback
  if (auto s = drv->GetOperationState(); s) {
    const char* name = s.value() == OperationState::Reset         ? "Reset"
                     : s.value() == OperationState::Config        ? "Config"
                     : s.value() == OperationState::Mission       ? "Mission"
                     : s.value() == OperationState::CriticalFault ? "CriticalFault"
                                                                  : "?";
    ESP_LOGI(TAG, "OperationState = %s", name);
  }

  // 3. SFF BIST
  if (auto r = drv->RunSffBist(20U); r) {
    ESP_LOGI(TAG, "SFF_BIST  done=%d pass=%d uerr=%d cerr=%d raw=0x%04X",
             r->done, r->pass, r->uncorrectable_reg_err, r->correctable_reg_err, r->raw);
  } else {
    ESP_LOGW(TAG, "SFF_BIST run failed: %d", static_cast<int>(r.error()));
  }

  // 4. Supply-monitor self-test (flexes UV/OV swap + V1V5 + OT_TEST)
  if (auto r = drv->RunSupplyMonitorSelfTest(); r) {
    ESP_LOGI(TAG, "SupplyMonitorSelfTest  swap=%d v1v5_uv=%d v1v5_ov=%d ot=%d overall=%d",
             r->uv_ov_swap_ok, r->v1v5_uv_ok, r->v1v5_ov_ok, r->ot_test_ok,
             r->overall_pass);
  } else {
    ESP_LOGW(TAG, "SupplyMonitorSelfTest failed: %d", static_cast<int>(r.error()));
  }

  // 5. Mask CH0 error out of FAULTN, then restore
  if (auto r = drv->SetFaultMask(MaskableFault::Ch0Error, false); !r) {
    ESP_LOGW(TAG, "Mask CH0Error off failed");
  }
  vTaskDelay(pdMS_TO_TICKS(1));
  if (auto r = drv->SetFaultMask(MaskableFault::Ch0Error, true); !r) {
    ESP_LOGW(TAG, "Restore CH0Error mask failed");
  }

  ESP_LOGI(TAG, "Diagnostics + BIST example complete");
}
