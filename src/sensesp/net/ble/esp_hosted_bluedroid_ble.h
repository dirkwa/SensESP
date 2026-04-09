#ifndef SENSESP_NET_BLE_ESP_HOSTED_BLUEDROID_BLE_H_
#define SENSESP_NET_BLE_ESP_HOSTED_BLUEDROID_BLE_H_

// Concrete BLEProvisioner implementation for ESP32-P4 (and other
// chips without a native BT controller) using:
//
//   * Bluedroid as the BT host stack, and
//   * esp_hosted's Bluedroid VHCI driver as the HCI transport,
//     routing HCI traffic over SDIO / SPI / UART to an onboard
//     ESP32-C6 (or similar) companion chip running esp-hosted-mcu.
//
// The whole header compiles to nothing unless both prerequisites are
// present in the sdkconfig. On a classic ESP32 build with native BT
// + NimBLE, or on an ESP32-C6 standalone build, or on any target that
// does not enable esp_hosted's Bluedroid VHCI path, this class is
// simply not declared — and attempting to instantiate it from user
// code produces a clean compile-time error at the use site rather
// than a silent runtime failure.

#include <Arduino.h>

#if defined(CONFIG_BT_BLUEDROID_ENABLED) && \
    defined(CONFIG_ESP_HOSTED_ENABLE_BT_BLUEDROID)

#include <atomic>

// Bluedroid public headers — these are available wherever
// CONFIG_BT_BLUEDROID_ENABLED is set.
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

#include "sensesp/net/ble/ble_provisioner.h"

namespace sensesp {

/**
 * @brief Configuration for EspHostedBluedroidBLE.
 *
 * Currently empty — the scan parameters (active vs passive, interval,
 * window, filter policy) are pinned to values that match ESPHome's
 * known-working P4 bluetooth_proxy config, and the transport setup
 * comes entirely from the sdkconfig (esp_hosted component options).
 *
 * The struct exists even though it has no fields so that the builder
 * pattern stays consistent with EthernetConfig / WiFiProvisioner and
 * so that future tuning options (active/passive scan choice, scan
 * interval tuning, filter policy selection) can be added without
 * breaking the API for existing users.
 *
 * ## Usage
 *
 *     builder.set_ble(EspHostedBluedroidBLEConfig{});
 *
 * Or, equivalently, via the convenience factory:
 *
 *     builder.set_ble(EspHostedBluedroidBLEConfig::active_scan());
 */
class EspHostedBluedroidBLE;

struct EspHostedBluedroidBLEConfig {
  /// Tag used by SensESPAppBuilder::set_ble() (templated) to find
  /// the corresponding provisioner implementation type.
  using ProvisionerType = EspHostedBluedroidBLE;

  /// False for passive scan (RX only), true for active (scanner
  /// solicits scan responses from peers). Passive is the default
  /// because active scan requires the C6 to TX SCAN_REQ over the
  /// SDIO bridge, which is unreliable on current esp_hosted firmware
  /// and causes intermittent advertisement silence. Most BLE
  /// advertisers broadcast all useful data in the advertisement
  /// itself — scan responses typically add only extra manufacturer
  /// data that the gateway doesn't need.
  bool active_scan = false;

  /// Scan interval in milliseconds. The controller schedules a scan
  /// window once per interval. ESPHome's known-working P4+C6
  /// bluetooth_proxy uses 320ms (512 BLE units). Higher duty cycles
  /// (window closer to interval) can overwhelm the C6's SDIO HCI
  /// transport and cause scan stalls. The BLESignalKGateway's scan
  /// watchdog handles stalls by restarting, but a conservative duty
  /// cycle minimises how often restarts are needed.
  uint32_t scan_interval_ms = 320;

  /// Scan window in milliseconds. Each interval, the scanner
  /// listens for this many ms. At 160ms window / 320ms interval
  /// (~50% duty cycle) the passive scanner discovers most
  /// advertisers within seconds while keeping SDIO HCI traffic
  /// manageable for the C6.
  uint32_t scan_window_ms = 160;
};

/**
 * @brief BLE provisioner using Bluedroid + esp_hosted VHCI.
 *
 * Constructor bring-up sequence:
 *
 *   1. hostedInitBLE()               — esp_hosted controller init
 *   2. hosted_hci_bluedroid_open()   — open VHCI transport
 *   3. esp_bluedroid_attach_hci_driver(...)
 *   4. esp_bluedroid_init()
 *   5. esp_bluedroid_enable()
 *   6. esp_ble_gap_register_callback(gap_event_trampoline)
 *
 * At the end of the constructor the BT stack is up and ready to
 * scan, but scanning is NOT started — the caller must explicitly
 * call start_scan() once observers are attached.
 *
 * GAP events from Bluedroid are dispatched through a static
 * trampoline function that forwards to the singleton instance_
 * pointer. Only one EspHostedBluedroidBLE instance can exist at a
 * time (constructor asserts this) because Bluedroid's GAP callback
 * registration is a process-global singleton.
 */
class EspHostedBluedroidBLE : public BLEProvisioner {
 public:
  explicit EspHostedBluedroidBLE(
      const EspHostedBluedroidBLEConfig& config = {});
  ~EspHostedBluedroidBLE() override;

  EspHostedBluedroidBLE(const EspHostedBluedroidBLE&) = delete;
  EspHostedBluedroidBLE& operator=(const EspHostedBluedroidBLE&) = delete;

  // -- BLEProvisioner --
  bool start_scan() override;
  bool stop_scan() override;
  bool is_scanning() const override;
  String mac_address() const override;
  uint32_t scan_hit_count() const override { return scan_hit_count_; }
  bool reset_bt_controller() override;

 private:
  // Called from the static GAP trampoline on behalf of this instance.
  void handle_gap_event(esp_gap_ble_cb_event_t event,
                        esp_ble_gap_cb_param_t* param);

  // Static trampoline registered with esp_ble_gap_register_callback.
  // Forwards to instance_->handle_gap_event() if instance_ is set.
  static void gap_event_trampoline(esp_gap_ble_cb_event_t event,
                                   esp_ble_gap_cb_param_t* param);

  // Singleton pointer used by the static trampoline. Set in the
  // constructor, cleared in the destructor.
  static EspHostedBluedroidBLE* instance_;

  EspHostedBluedroidBLEConfig config_;
  std::atomic<bool> bt_stack_up_{false};
  std::atomic<bool> scan_params_set_{false};
  std::atomic<bool> scanning_{false};
  std::atomic<uint32_t> scan_hit_count_{0};

  // Cached scan parameters struct — must outlive the scan since
  // Bluedroid stores a reference during active scanning.
  esp_ble_scan_params_t scan_params_{};
};

}  // namespace sensesp

#endif  // CONFIG_BT_BLUEDROID_ENABLED && CONFIG_ESP_HOSTED_ENABLE_BT_BLUEDROID

#endif  // SENSESP_NET_BLE_ESP_HOSTED_BLUEDROID_BLE_H_
