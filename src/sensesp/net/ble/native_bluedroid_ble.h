#ifndef SENSESP_NET_BLE_NATIVE_BLE_H_
#define SENSESP_NET_BLE_NATIVE_BLE_H_

// Concrete BLEProvisioner for chips with a native BT controller
// (ESP32, ESP32-C3, ESP32-C5, ESP32-S3, etc.) using the Arduino-ESP32
// BLE library (works with both NimBLE and Bluedroid backends).

#include <Arduino.h>

#include "soc/soc_caps.h"
#include "sdkconfig.h"

// Guard: only available on chips with native BLE (not esp_hosted remote BLE)
#if defined(SOC_BLE_SUPPORTED) && \
    !defined(CONFIG_ESP_HOSTED_ENABLE_BT_BLUEDROID) && \
    !defined(CONFIG_ESP_HOSTED_ENABLE_BT_NIMBLE)

#include <atomic>

#include "sensesp/net/ble/ble_provisioner.h"

namespace sensesp {

class NativeBLE;

struct NativeBLEConfig {
  using ProvisionerType = NativeBLE;

  bool active_scan = true;
  uint32_t scan_interval_ms = 100;
  uint32_t scan_window_ms = 100;
};

class NativeBLE : public BLEProvisioner {
 public:
  explicit NativeBLE(const NativeBLEConfig& config = {});
  ~NativeBLE() override;

  NativeBLE(const NativeBLE&) = delete;
  NativeBLE& operator=(const NativeBLE&) = delete;

  bool start_scan() override;
  bool stop_scan() override;
  bool is_scanning() const override;
  String mac_address() const override;
  uint32_t scan_hit_count() const override { return scan_hit_count_; }

 private:
  NativeBLEConfig config_;
  std::atomic<bool> scanning_{false};
  std::atomic<uint32_t> scan_hit_count_{0};

  friend class ScanCallbacks;

  static void scan_complete_cb(void* result);
};

}  // namespace sensesp

#endif  // SOC_BLE_SUPPORTED && !CONFIG_ESP_HOSTED_*

#endif  // SENSESP_NET_BLE_NATIVE_BLE_H_
