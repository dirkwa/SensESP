/**
 * @file signalk_ble_gateway.cpp
 * @brief BLE-to-Ethernet gateway for Signal K
 *
 * Scans for BLE advertisements and forwards raw manufacturer data to the
 * bt-sensors-plugin-sk Signal K plugin over HTTP.  The plugin handles
 * device identification, decryption, parsing, and Signal K delta emission.
 *
 * The gateway is device-agnostic — it forwards all BLE advertisements that
 * contain manufacturer data.  The plugin decides which devices it supports
 * (Victron, Ruuvi, Xiaomi, etc.).
 *
 * Why this architecture?
 * - Zero parsing code on the ESP32 — the plugin already has it
 * - Encryption keys are configured on the server (plugin UI), not firmware
 * - All supported sensor types work immediately
 * - Ethernet + BLE coexist perfectly — no WiFi radio contention
 *
 * Requirements:
 * - PoE Ethernet ESP32 board (e.g. Olimex ESP32-POE-ISO)
 * - bt-sensors-plugin-sk with remote gateway support
 * - NimBLE-Arduino library (added via platformio.ini)
 *
 * @see https://github.com/naugehyde/bt-sensors-plugin-sk
 */

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <NimBLEDevice.h>
#include <esp_log.h>

#include "sensesp_app_builder.h"

using namespace sensesp;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// How often to POST collected advertisements to the plugin (ms)
static constexpr unsigned long POST_INTERVAL_MS = 2000;

// ---------------------------------------------------------------------------
// Advertisement buffer (shared between BLE task and main loop)
// ---------------------------------------------------------------------------

struct BleAdvertisement {
  std::string mac;
  std::string name;
  int rssi;
  uint16_t mfr_id;             // manufacturer ID (little-endian from BLE)
  std::vector<uint8_t> mfr_data;  // raw data after the 2-byte manufacturer ID
};

static std::vector<BleAdvertisement> pending_ads;
static SemaphoreHandle_t ads_mutex;

// ---------------------------------------------------------------------------
// Plugin URL — derived from Signal K server discovered via mDNS
// ---------------------------------------------------------------------------

static String plugin_api_url;

/**
 * Try to build the plugin API URL from the SK WebSocket client's server
 * address/port.  Called periodically until successful.
 */
static void resolve_plugin_url() {
  if (plugin_api_url.length() > 0) return;

  auto ws = SensESPApp::get()->get_ws_client();
  if (!ws) return;

  String addr = ws->get_server_address();
  uint16_t port = ws->get_server_port();
  if (addr.length() == 0 || port == 0) return;

  plugin_api_url =
      "http://" + addr + ":" + String(port) +
      "/plugins/bt-sensors-plugin-sk/api/gateway/advertisements";

  ESP_LOGI("BLE-GW", "Plugin URL resolved: %s", plugin_api_url.c_str());
}

// ---------------------------------------------------------------------------
// NimBLE scan callback
// ---------------------------------------------------------------------------

class BLEScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* device) override {
    if (!device->haveManufacturerData()) return;

    std::string mfr_raw = device->getManufacturerData();
    const uint8_t* data = reinterpret_cast<const uint8_t*>(mfr_raw.data());
    size_t len = mfr_raw.size();

    // Need at least 2-byte manufacturer ID + 1 byte payload
    if (len < 3) return;

    // Manufacturer ID is first 2 bytes (little-endian)
    uint16_t mfr_id = data[0] | (data[1] << 8);

    BleAdvertisement adv;
    adv.mac = device->getAddress().toString();
    adv.name = device->haveName() ? device->getName() : "";
    adv.rssi = device->getRSSI();
    adv.mfr_id = mfr_id;
    adv.mfr_data.assign(data + 2, data + len);

    xSemaphoreTake(ads_mutex, portMAX_DELAY);

    // Deduplicate: keep latest per MAC
    auto it = std::find_if(
        pending_ads.begin(), pending_ads.end(),
        [&](const BleAdvertisement& a) { return a.mac == adv.mac; });

    if (it != pending_ads.end()) {
      *it = std::move(adv);
    } else {
      pending_ads.push_back(std::move(adv));
    }

    xSemaphoreGive(ads_mutex);
  }
};

// ---------------------------------------------------------------------------
// HTTP POST to plugin
// ---------------------------------------------------------------------------

static String bytes_to_hex(const std::vector<uint8_t>& data) {
  String hex;
  hex.reserve(data.size() * 2);
  for (uint8_t b : data) {
    char buf[3];
    snprintf(buf, sizeof(buf), "%02x", b);
    hex += buf;
  }
  return hex;
}

static void send_advertisements() {
  resolve_plugin_url();
  if (plugin_api_url.length() == 0) return;

  xSemaphoreTake(ads_mutex, portMAX_DELAY);
  if (pending_ads.empty()) {
    xSemaphoreGive(ads_mutex);
    return;
  }
  auto ads = std::move(pending_ads);
  pending_ads.clear();
  xSemaphoreGive(ads_mutex);

  JsonDocument doc;
  doc["gateway_id"] = SensESPBaseApp::get_hostname();

  JsonArray devices = doc["devices"].to<JsonArray>();
  for (const auto& adv : ads) {
    JsonObject dev = devices.add<JsonObject>();

    String mac = adv.mac.c_str();
    mac.toUpperCase();
    dev["mac"] = mac;
    dev["rssi"] = adv.rssi;
    if (!adv.name.empty()) {
      dev["name"] = String(adv.name.c_str());
    }

    JsonObject mfr = dev["manufacturer_data"].to<JsonObject>();
    // Key = decimal manufacturer ID, value = hex payload after the ID
    mfr[String(adv.mfr_id)] = bytes_to_hex(adv.mfr_data);
  }

  String body;
  serializeJson(doc, body);

  HTTPClient http;
  http.begin(plugin_api_url);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(5000);

  int code = http.POST(body);
  if (code == 200) {
    ESP_LOGD("BLE-GW", "Forwarded %u device(s)", (unsigned)ads.size());
  } else {
    ESP_LOGW("BLE-GW", "POST failed: HTTP %d", code);
  }
  http.end();
}

// ---------------------------------------------------------------------------
// Arduino setup / loop
// ---------------------------------------------------------------------------

void setup() {
  SetupLogging();

  ads_mutex = xSemaphoreCreateMutex();

  SensESPAppBuilder builder;
  auto sensesp_app = builder.set_hostname("signalk-ble-gw")
                         ->disable_wifi()
                         ->set_ethernet(EthernetConfig::aptinex_isolpoe())
                         ->enable_ota("thisismyota")
                         ->get_app();

  // Initialize NimBLE scanner (observer role only — WiFi is off)
  NimBLEDevice::init("");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new BLEScanCallbacks(), true);  // wantDuplicates
  scan->setActiveScan(false);   // passive = sufficient for advertisements
  scan->setInterval(100);       // scan interval (ms)
  scan->setWindow(99);          // scan window (ms), <= interval
  scan->setDuplicateFilter(0);  // 0 = report duplicates
  scan->start(0);               // 0 = scan continuously
  // NimBLE resets IDF log levels during init, so this must come after start()
  esp_log_level_set("NimBLEScan", ESP_LOG_WARN);

  // Periodically forward collected advertisements to the plugin
  event_loop()->onRepeat(POST_INTERVAL_MS, send_advertisements);

  ESP_LOGI("BLE-GW", "Signal K BLE Gateway started");
}

void loop() { event_loop()->tick(); }
