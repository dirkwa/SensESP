#include "sensesp/net/ble/native_bluedroid_ble.h"

#if defined(SOC_BLE_SUPPORTED) && \
    !defined(CONFIG_ESP_HOSTED_ENABLE_BT_BLUEDROID) && \
    !defined(CONFIG_ESP_HOSTED_ENABLE_BT_NIMBLE)

#include <BLEAdvertisedDevice.h>
#include <BLEDevice.h>
#include <BLEScan.h>

#include "esp_log.h"

namespace sensesp {

namespace {
constexpr const char* kTag = "ble_native";
NativeBLE* g_instance = nullptr;
}  // namespace

// Arduino BLE scan callback — fires for every advertisement received.
class ScanCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    if (!g_instance) return;

    g_instance->scan_hit_count_.fetch_add(1, std::memory_order_relaxed);

    BLEAdvertisement ad;
    ad.address = String(dev.getAddress().toString().c_str());
    ad.address_type = dev.getAddressType();
    ad.rssi = dev.getRSSI();
    if (dev.haveName()) {
      ad.name = String(dev.getName().c_str());
    }
    // Raw advertisement data
    uint8_t* payload = dev.getPayload();
    size_t payload_len = dev.getPayloadLength();
    if (payload && payload_len > 0) {
      ad.adv_data.assign(payload, payload + payload_len);
    }
    ad.received_at_ms = millis();

    g_instance->emit(ad);
  }
};

static ScanCallbacks g_scan_callbacks;

NativeBLE::NativeBLE(const NativeBLEConfig& config)
    : config_(config) {
  g_instance = this;

  ESP_LOGI(kTag, "Initialising native BLE (Arduino BLE library)");
  BLEDevice::init("");

  ESP_LOGI(kTag, "BLE initialised. active=%d itvl=%ums win=%ums",
           (int)config_.active_scan,
           (unsigned)config_.scan_interval_ms,
           (unsigned)config_.scan_window_ms);
}

NativeBLE::~NativeBLE() {
  stop_scan();
  g_instance = nullptr;
}

bool NativeBLE::start_scan() {
  if (scanning_.load()) return true;

  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(&g_scan_callbacks, /*wantDuplicates=*/true);
  scan->setActiveScan(config_.active_scan);
  scan->setInterval(config_.scan_interval_ms);
  scan->setWindow(config_.scan_window_ms);

  // Start continuous scan (0 = scan forever)
  bool ok = scan->start(0, false);
  if (ok) {
    scanning_.store(true);
    ESP_LOGI(kTag, "BLE scan started");
  } else {
    ESP_LOGE(kTag, "BLE scan start failed");
  }
  return ok;
}

bool NativeBLE::stop_scan() {
  if (!scanning_.load()) return true;
  BLEDevice::getScan()->stop();
  scanning_.store(false);
  ESP_LOGI(kTag, "BLE scan stopped");
  return true;
}

bool NativeBLE::is_scanning() const { return scanning_.load(); }

String NativeBLE::mac_address() const {
  return String(BLEDevice::getAddress().toString().c_str());
}

}  // namespace sensesp

#endif  // SOC_BLE_SUPPORTED && !CONFIG_ESP_HOSTED_*
