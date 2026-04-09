#include "sensesp/net/ble/esp_hosted_bluedroid_ble.h"

#if defined(CONFIG_BT_BLUEDROID_ENABLED) && \
    defined(CONFIG_ESP_HOSTED_ENABLE_BT_BLUEDROID)

#include <string.h>

#include "esp_bluedroid_hci.h"
#include "esp_bt_device.h"
#include "esp_log.h"

// Arduino-ESP32 helper that brings the ESP-Hosted BT controller up.
// Declared in esp32-hal-hosted.h — on P4 this pulls in esp_hosted's
// SDIO transport and (for slave fw >= 2.6.0) calls
// esp_hosted_bt_controller_init() + _enable() internally.
extern "C" {
#include "esp32-hal-hosted.h"
}

// esp_hosted's Bluedroid VHCI driver functions. These are not
// declared in esp_bluedroid_hci.h (that's the Bluedroid-side
// operations struct, not the transport-side implementation). The
// esp_hosted component ships esp_hosted_bluedroid.h with the
// declarations but it is not on the default include path when we
// build against the library as a managed component, so declare them
// manually here — this matches what esp_hosted's own examples
// (examples/host_bluedroid_host_only/main/main.c) do.
extern "C" {
void hosted_hci_bluedroid_open(void);
void hosted_hci_bluedroid_close(void);
void hosted_hci_bluedroid_send(uint8_t* data, uint16_t len);
bool hosted_hci_bluedroid_check_send_available(void);
esp_err_t hosted_hci_bluedroid_register_host_callback(
    const esp_bluedroid_hci_driver_callbacks_t* callback);
}

namespace sensesp {

namespace {
constexpr const char* kTag = "ble_prov";

// Convert a millisecond scan interval/window to the BT spec units
// (0.625 ms per unit). Clamped to the valid range [0x0004, 0x4000]
// per Bluetooth Core Spec.
uint16_t ms_to_scan_units(uint32_t ms) {
  uint32_t units = (ms * 1000U) / 625U;
  if (units < 0x0004U) {
    units = 0x0004U;
  } else if (units > 0x4000U) {
    units = 0x4000U;
  }
  return static_cast<uint16_t>(units);
}

// Format a 6-byte BD address as "AA:BB:CC:DD:EE:FF".
String format_bda(const uint8_t* bda) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", bda[0], bda[1],
           bda[2], bda[3], bda[4], bda[5]);
  return String(buf);
}

// Extract the complete-local-name AD field from an adv data payload,
// or return an empty String if not present.
String extract_local_name(const uint8_t* adv_data, size_t len) {
  uint8_t name_len = 0;
  uint8_t* name = esp_ble_resolve_adv_data(const_cast<uint8_t*>(adv_data),
                                           ESP_BLE_AD_TYPE_NAME_CMPL, &name_len);
  if (name != nullptr && name_len > 0) {
    // esp_ble_resolve_adv_data returns a pointer into the buffer, not a
    // null-terminated string.
    char tmp[32];
    size_t copy = name_len < sizeof(tmp) - 1 ? name_len : sizeof(tmp) - 1;
    memcpy(tmp, name, copy);
    tmp[copy] = '\0';
    return String(tmp);
  }
  return String("");
}

}  // namespace

EspHostedBluedroidBLE* EspHostedBluedroidBLE::instance_ = nullptr;

EspHostedBluedroidBLE::EspHostedBluedroidBLE(
    const EspHostedBluedroidBLEConfig& config)
    : config_(config) {
  if (instance_ != nullptr) {
    ESP_LOGE(kTag,
             "Another EspHostedBluedroidBLE instance already exists. "
             "Bluedroid's GAP callback is process-global; only one "
             "instance is supported at a time.");
    return;
  }
  instance_ = this;

  ESP_LOGI(kTag, "Initialising ESP-Hosted BT controller");
  if (!hostedInitBLE()) {
    ESP_LOGE(kTag, "hostedInitBLE() failed — C6 slave not responding?");
    return;
  }

  ESP_LOGI(kTag, "Opening hosted HCI VHCI transport");
  hosted_hci_bluedroid_open();

  // Register the esp_hosted VHCI driver with Bluedroid. This is the
  // step Arduino-ESP32's hostedInitBLE() helper intentionally omits
  // (because it was written for NimBLE, which attaches to the HCI
  // transport automatically). Without this registration
  // esp_bluedroid_enable() below would post its enable command to
  // the BTC task and then block forever on future_await() because
  // no HCI responses would come back from the controller.
  static const esp_bluedroid_hci_driver_operations_t kHostedHciOps = {
      .send = hosted_hci_bluedroid_send,
      .check_send_available = hosted_hci_bluedroid_check_send_available,
      .register_host_callback = hosted_hci_bluedroid_register_host_callback,
  };

  esp_err_t err = esp_bluedroid_attach_hci_driver(&kHostedHciOps);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "esp_bluedroid_attach_hci_driver failed: %s",
             esp_err_to_name(err));
    return;
  }

  ESP_LOGI(kTag, "Initialising Bluedroid host stack");
  err = esp_bluedroid_init();
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "esp_bluedroid_init failed: %s", esp_err_to_name(err));
    return;
  }

  err = esp_bluedroid_enable();
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "esp_bluedroid_enable failed: %s", esp_err_to_name(err));
    return;
  }

  err = esp_ble_gap_register_callback(&EspHostedBluedroidBLE::gap_event_trampoline);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "esp_ble_gap_register_callback failed: %s",
             esp_err_to_name(err));
    return;
  }

  // Build the extended-scan parameters struct. The IDF Bluedroid
  // build for ESP32-P4 is configured with
  // CONFIG_BT_BLE_50_FEATURES_SUPPORTED=y (the default for any
  // non-ESP32 chip), which compiles out the legacy
  // esp_ble_gap_set_scan_params / esp_ble_gap_start_scanning entry
  // points entirely — only the extended-scan equivalents are
  // linkable. We use those.
  const uint16_t itvl = ms_to_scan_units(config_.scan_interval_ms);
  const uint16_t win = ms_to_scan_units(config_.scan_window_ms);
  const esp_ble_scan_type_t type =
      config_.active_scan ? BLE_SCAN_TYPE_ACTIVE : BLE_SCAN_TYPE_PASSIVE;

  scan_params_ = {
      .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
      .filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
      .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
      .cfg_mask = ESP_BLE_GAP_EXT_SCAN_CFG_UNCODE_MASK,
      .uncoded_cfg = {.scan_type = type, .scan_interval = itvl, .scan_window = win},
      .coded_cfg = {.scan_type = type, .scan_interval = 0, .scan_window = 0},
  };

  bt_stack_up_.store(true);
  ESP_LOGI(kTag,
           "Bluedroid host stack up. active=%d itvl=%ums win=%ums. Call "
           "start_scan() to begin scanning.",
           static_cast<int>(config_.active_scan),
           static_cast<unsigned>(config_.scan_interval_ms),
           static_cast<unsigned>(config_.scan_window_ms));
}

EspHostedBluedroidBLE::~EspHostedBluedroidBLE() {
  if (scanning_.load()) {
    esp_ble_gap_stop_ext_scan();
  }
  // Intentionally not calling esp_bluedroid_disable() / _deinit()
  // here. Arduino-ESP32's ETH teardown has a similar comment: the
  // IDF teardown paths for stateful subsystems do not reliably
  // restore an initial state that a subsequent re-init can build
  // on, so SensESP provisioners are expected to live as long as
  // the app does.
  instance_ = nullptr;
}

bool EspHostedBluedroidBLE::start_scan() {
  if (!bt_stack_up_.load()) {
    ESP_LOGW(kTag, "start_scan() called but BT stack is not up");
    return false;
  }
  if (scanning_.load()) {
    return true;
  }

  // Set scan params first. The actual scan start happens in the
  // SCAN_PARAMS_COMPLETE event handler, to ensure the controller
  // has accepted the params before we try to start.
  esp_err_t err = esp_ble_gap_set_ext_scan_params(&scan_params_);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "esp_ble_gap_set_ext_scan_params failed: %s",
             esp_err_to_name(err));
    return false;
  }
  return true;
}

bool EspHostedBluedroidBLE::stop_scan() {
  if (!scanning_.load()) {
    return true;
  }
  esp_err_t err = esp_ble_gap_stop_ext_scan();
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "esp_ble_gap_stop_ext_scan failed: %s",
             esp_err_to_name(err));
    return false;
  }
  // scanning_ flag is cleared in the STOP_COMPLETE event handler.
  return true;
}

bool EspHostedBluedroidBLE::is_scanning() const { return scanning_.load(); }

String EspHostedBluedroidBLE::mac_address() const {
  if (!bt_stack_up_.load()) {
    return String("");
  }
  const uint8_t* bda = esp_bt_dev_get_address();
  if (bda == nullptr) {
    return String("");
  }
  return format_bda(bda);
}

void EspHostedBluedroidBLE::gap_event_trampoline(
    esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
  if (instance_ == nullptr) {
    return;
  }
  instance_->handle_gap_event(event, param);
}

void EspHostedBluedroidBLE::handle_gap_event(
    esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
  switch (event) {
    case ESP_GAP_BLE_SET_EXT_SCAN_PARAMS_COMPLETE_EVT: {
      if (param->set_ext_scan_params.status != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(kTag, "SET_EXT_SCAN_PARAMS failed, status=%d",
                 param->set_ext_scan_params.status);
        return;
      }
      scan_params_set_.store(true);
      // duration=0, period=0 → scan forever until stopped.
      esp_err_t err = esp_ble_gap_start_ext_scan(0, 0);
      if (err != ESP_OK) {
        ESP_LOGE(kTag, "esp_ble_gap_start_ext_scan failed: %s",
                 esp_err_to_name(err));
      }
      break;
    }
    case ESP_GAP_BLE_EXT_SCAN_START_COMPLETE_EVT: {
      if (param->ext_scan_start.status != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(kTag, "EXT_SCAN_START failed, status=%d",
                 param->ext_scan_start.status);
        return;
      }
      scanning_.store(true);
      ESP_LOGI(kTag, "Extended scan started");
      break;
    }
    case ESP_GAP_BLE_EXT_SCAN_STOP_COMPLETE_EVT: {
      scanning_.store(false);
      ESP_LOGI(kTag, "Extended scan stopped");
      break;
    }
    case ESP_GAP_BLE_EXT_ADV_REPORT_EVT: {
      const auto& r = param->ext_adv_report.params;
      scan_hit_count_.fetch_add(1, std::memory_order_relaxed);

      BLEAdvertisement ad;
      ad.address = format_bda(r.addr);
      ad.address_type = static_cast<uint8_t>(r.addr_type);
      ad.rssi = r.rssi;
      ad.name = extract_local_name(r.adv_data, r.adv_data_len);
      ad.adv_data.assign(r.adv_data, r.adv_data + r.adv_data_len);
      ad.received_at_ms = millis();

      // Emit through the inherited ValueProducer<BLEAdvertisement>
      // so any observer that connect_to'd us receives it.
      this->emit(ad);
      break;
    }
    default:
      // Many other events (adv-complete, period-sync, etc.) that we
      // do not currently care about. Log at debug level only.
      ESP_LOGD(kTag, "Unhandled GAP event %d", static_cast<int>(event));
      break;
  }
}

}  // namespace sensesp

#endif  // CONFIG_BT_BLUEDROID_ENABLED && CONFIG_ESP_HOSTED_ENABLE_BT_BLUEDROID
