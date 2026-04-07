/**
 * @file main.cpp
 * @brief Signal K BLE gateway running on Waveshare ESP32-P4-WIFI6-POE-ETH.
 *
 * Hardware:
 *   - ESP32-P4 main MCU with native RMII Ethernet (M2 EthernetProvisioner)
 *   - ESP32-C6 companion chip via SDIO running esp_hosted_mcu slave
 *     firmware, providing BLE 5 to the P4 (the P4 itself has no native
 *     radio). The slave firmware must be flashed before this gateway
 *     will work — see ../p4_c6_ota_updater for the one-shot updater
 *     that does it from the P4 over the same ethernet link.
 *
 * Software:
 *   - SensESP framework: hostname, ethernet provisioner, web UI, OTA,
 *     SK WebSocket client (for the access-request / token flow), event
 *     loop. All from this PR's transport-agnostic networking refactor.
 *   - h2zero/esp-nimble-cpp 2.5.0 (IDF managed component, not the
 *     Arduino flavour) — provides NimBLEDevice / NimBLEScan / NimBLEClient
 *     etc. classes with ESP-Hosted BT VHCI support on ESP32-P4. The
 *     NimBLE-Arduino flavour does NOT compile on P4 because of an
 *     unconditional esp_bt.h include — see h2zero/NimBLE-Arduino#906.
 *   - Arduino-ESP32 3.3.7's esp32-hal-hosted.c provides hostedInitBLE()
 *     which we call before NimBLEDevice::init() to bring up the C6's
 *     Bluetooth controller via esp_hosted_bt_controller_init/enable.
 *
 * Protocol — speaks the signalk-server `ble-provider-api` dual-channel:
 *   - HTTP POST  /signalk/v2/api/ble/gateway/advertisements
 *     Bearer JWT auth, JSON {gateway_id, devices:[{mac, rssi, name,
 *     manufacturer_data:{<decimal_company_id>:"<hex>"}}]}.  Posted at
 *     POST_INTERVAL_MS from a background FreeRTOS task.
 *   - WebSocket  /signalk/v2/api/ble/gateway/ws?token=<JWT>
 *     Bidirectional GATT control: server sends gatt_subscribe /
 *     gatt_write / gatt_close, gateway responds with gatt_connected /
 *     gatt_data / gatt_disconnected / gatt_error / status. Up to
 *     MAX_GATT_SESSIONS concurrent peripheral connections, each driven
 *     by a GATTSession with retry/backoff state machine.
 *
 * The gateway piggybacks on SensESP's SKWSClient for the SignalK
 * access-request token flow: SKWSClient handles authentication and
 * stores the token, the gateway reads it via get_auth_token() and uses
 * the same Bearer credential for both the HTTP POST channel and the
 * GATT WebSocket.
 */

#include <HTTPClient.h>
#include <NimBLEDevice.h>
#include <esp_websocket_client.h>

#include <atomic>
#include <map>
#include <string>

#include "esp32-hal-hosted.h"
#include "sensesp/net/ethernet_provisioner.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"

using namespace sensesp;

// Keep BT memory — NimBLE needs it but Arduino doesn't detect external lib
extern "C" bool btInUse() { return true; }

// Default hostname / gateway_id. Override via -D GATEWAY_HOSTNAME='"…"'
// in platformio.ini build_flags when running multiple gateways on the
// same Signal K server.
#ifndef GATEWAY_HOSTNAME
#define GATEWAY_HOSTNAME "signalk-ble-gw"
#endif

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

static constexpr unsigned long POST_INTERVAL_MS = 2000;

// Server-side API paths (v2 — owned by server, not bt-sensors plugin)
static constexpr const char* SK_ADV_PATH =
    "/signalk/v2/api/ble/gateway/advertisements";
static constexpr const char* SK_WS_PATH_BASE =
    "/signalk/v2/api/ble/gateway/ws?token=";

static constexpr int MAX_GATT_SESSIONS = 3;
static constexpr unsigned long STATUS_INTERVAL_MS = 30000;
static constexpr unsigned long GATT_RECONNECT_DELAY_MS = 3000;
static constexpr int GATT_MAX_RETRIES = 5;

// ---------------------------------------------------------------------------
// SensESP app reference — provides server address, token, event loop
// ---------------------------------------------------------------------------

static std::shared_ptr<SKWSClient> sk_ws_client;

// ---------------------------------------------------------------------------
// Utility: hex conversions
// ---------------------------------------------------------------------------

static String bytes_to_hex(const uint8_t* data, size_t len) {
  String hex;
  hex.reserve(len * 2);
  for (size_t i = 0; i < len; i++) {
    char buf[3];
    snprintf(buf, sizeof(buf), "%02x", data[i]);
    hex += buf;
  }
  return hex;
}

static String bytes_to_hex(const std::vector<uint8_t>& data) {
  return bytes_to_hex(data.data(), data.size());
}

static std::vector<uint8_t> hex_to_bytes(const char* hex, size_t hex_len) {
  std::vector<uint8_t> bytes;
  bytes.reserve(hex_len / 2);
  for (size_t i = 0; i + 1 < hex_len; i += 2) {
    char pair[3] = { hex[i], hex[i + 1], 0 };
    bytes.push_back((uint8_t)strtoul(pair, nullptr, 16));
  }
  return bytes;
}

// ---------------------------------------------------------------------------
// Advertisement buffer (shared between BLE scan callback and HTTP task)
// ---------------------------------------------------------------------------

struct BleAdvertisement {
  std::string mac;
  std::string name;
  int rssi;
  uint16_t mfr_id;
  std::vector<uint8_t> mfr_data;
};

static std::vector<BleAdvertisement> pending_ads;
static SemaphoreHandle_t ads_mutex;

// Persistent map of MAC -> NimBLE address type, populated by the scan
// callback and read by the GATT connection logic when the server asks
// us to connect to a device. Without this we'd default to address
// type 0 (public), which fails for the very common case of devices
// using random resolvable addresses (Apple, Ruuvi, anything with BLE
// privacy enabled). Protected by addr_type_mutex because the scan
// callback runs on the BLE host task and the GATT logic runs on the
// main event loop.
static std::map<std::string, uint8_t> known_addr_types;
static SemaphoreHandle_t addr_type_mutex;

static uint8_t lookup_addr_type(const std::string& mac) {
  uint8_t type = 0;  // public, sensible fallback
  xSemaphoreTake(addr_type_mutex, portMAX_DELAY);
  auto it = known_addr_types.find(mac);
  if (it != known_addr_types.end()) {
    type = it->second;
  }
  xSemaphoreGive(addr_type_mutex);
  return type;
}

// Mutex protecting the gatt_sessions[] array. Acquired by handlers
// running on either the main loop or the esp_websocket_client event
// task (which dispatches WS messages on its own FreeRTOS task).
static SemaphoreHandle_t gatt_sessions_mutex;

// ---------------------------------------------------------------------------
// NimBLE scan callback
// ---------------------------------------------------------------------------

class BLEScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* device) override {
    BleAdvertisement adv;
    adv.mac = device->getAddress().toString();
    adv.name = device->haveName() ? device->getName() : "";
    adv.rssi = device->getRSSI();
    adv.mfr_id = 0;

    // Remember the device's BLE address type so a later GATT connect
    // can use the right value (devices with random addresses fail
    // silently if you try to connect with type=0/public).
    {
      uint8_t type = device->getAddress().getType();
      xSemaphoreTake(addr_type_mutex, portMAX_DELAY);
      known_addr_types[adv.mac] = type;
      xSemaphoreGive(addr_type_mutex);
    }

    if (device->haveManufacturerData()) {
      std::string mfr_raw = device->getManufacturerData();
      const uint8_t* data = reinterpret_cast<const uint8_t*>(mfr_raw.data());
      size_t len = mfr_raw.size();
      if (len >= 3) {
        adv.mfr_id = data[0] | (data[1] << 8);
        adv.mfr_data.assign(data + 2, data + len);
      }
    }

    xSemaphoreTake(ads_mutex, portMAX_DELAY);

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
// Custom WebSocket for BLE GATT commands (separate from SKWSClient)
// ---------------------------------------------------------------------------

static esp_websocket_client_handle_t ble_ws_client = nullptr;
// Mutex protecting ble_ws_client pointer lifecycle. Both senders
// (ws_send_json) and the destroy path (destroy_custom_websocket) take
// this so a destroy can never run concurrently with an in-flight send.
static SemaphoreHandle_t ble_ws_client_mutex = nullptr;
// std::atomic so the WebSocket event task can safely set it while
// the main loop, BLE host task, and HTTP POST task all read it.
static std::atomic<bool> ble_ws_connected{false};

// Set true once hostedInitBLE() + NimBLEDevice::init() have completed
// successfully. The HTTP POST task checks this before touching the
// NimBLEScan singleton — if the BLE stack failed to initialise (most
// commonly because the C6 slave firmware is missing), the gateway
// degrades gracefully into a network-only / no-op state instead of
// dereferencing an uninitialised NimBLE object.
static std::atomic<bool> ble_initialized{false};

// Forward declarations
static void ws_send_json(JsonDocument& doc);
static void handle_ws_message(uint8_t* payload, size_t length);
static void send_hello();
static void closeAllGATTSessions();

static void ble_ws_event_handler(void* handler_args, esp_event_base_t base,
                                  int32_t event_id, void* event_data) {
  esp_websocket_event_data_t* data = (esp_websocket_event_data_t*)event_data;
  switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
      ESP_LOGI("BLE-WS", "Connected");
      ble_ws_connected = true;
      closeAllGATTSessions();
      send_hello();
      break;
    case WEBSOCKET_EVENT_DISCONNECTED:
      ESP_LOGI("BLE-WS", "Disconnected");
      ble_ws_connected = false;
      break;
    case WEBSOCKET_EVENT_DATA:
      if (data->op_code == 0x01 && data->data_len > 0) {
        handle_ws_message((uint8_t*)data->data_ptr, data->data_len);
      }
      break;
    case WEBSOCKET_EVENT_ERROR:
      ESP_LOGW("BLE-WS", "Error");
      ble_ws_connected = false;
      break;
    default:
      break;
  }
}

static void destroy_custom_websocket() {
  // Mark disconnected FIRST so any concurrent senders drop their
  // payload before we even take the lock.
  ble_ws_connected = false;
  if (ble_ws_client_mutex) {
    xSemaphoreTake(ble_ws_client_mutex, portMAX_DELAY);
  }
  if (ble_ws_client) {
    esp_websocket_client_stop(ble_ws_client);
    esp_websocket_client_destroy(ble_ws_client);
    ble_ws_client = nullptr;
  }
  if (ble_ws_client_mutex) {
    xSemaphoreGive(ble_ws_client_mutex);
  }
}

static void init_custom_websocket() {
  destroy_custom_websocket();

  if (!sk_ws_client) return;

  String token = sk_ws_client->get_auth_token();
  String addr = sk_ws_client->get_server_address();
  uint16_t port = sk_ws_client->get_server_port();

  if (addr.isEmpty()) return;

  String url = String("ws://") + addr + ":" + String(port) +
               SK_WS_PATH_BASE + token;

  // Don't log the token even when SK security is off (it is empty in
  // that case, but future deployments with real auth would otherwise
  // leak the JWT to any serial/syslog consumer).
  ESP_LOGI("BLE-WS", "Connecting to ws://%s:%u%s<redacted-token>",
           addr.c_str(), (unsigned)port, SK_WS_PATH_BASE);

  esp_websocket_client_config_t config = {};
  config.uri = url.c_str();
  config.task_stack = 4096;
  config.buffer_size = 1024;

  ble_ws_client = esp_websocket_client_init(&config);
  if (!ble_ws_client) {
    ESP_LOGE("BLE-WS", "Failed to init WebSocket client");
    return;
  }
  esp_websocket_register_events(ble_ws_client, WEBSOCKET_EVENT_ANY,
                                ble_ws_event_handler, nullptr);
  esp_websocket_client_start(ble_ws_client);
}

static void ws_send_json(JsonDocument& doc) {
  if (!ble_ws_connected) return;
  String msg;
  serializeJson(doc, msg);
  // Take the client mutex for the entire send so destroy_custom_websocket()
  // cannot free the handle under our feet. We re-check ble_ws_client
  // inside the critical section because the flag could have flipped
  // between the atomic check above and taking the lock.
  if (!ble_ws_client_mutex) return;
  xSemaphoreTake(ble_ws_client_mutex, portMAX_DELAY);
  if (ble_ws_client == nullptr) {
    xSemaphoreGive(ble_ws_client_mutex);
    return;
  }
  // Bounded timeout (500 ms) so a stalled or slow socket can never
  // wedge the BLE host task — gatt_data callbacks fire from the
  // NimBLE task and call this directly. portMAX_DELAY would let a
  // slow client back-pressure the entire BLE stack.
  int sent = esp_websocket_client_send_text(ble_ws_client, msg.c_str(),
                                            msg.length(), pdMS_TO_TICKS(500));
  xSemaphoreGive(ble_ws_client_mutex);
  if (sent < 0) {
    ESP_LOGW("BLE-WS", "send_text dropped (%d): %.*s",
             sent, std::min((int)msg.length(), 80), msg.c_str());
  }
}

// ---------------------------------------------------------------------------
// HTTP POST to server (advertisements)
// ---------------------------------------------------------------------------

static void send_advertisements() {
  if (!sk_ws_client || !sk_ws_client->is_connected()) return;

  String token = sk_ws_client->get_auth_token();
  String addr = sk_ws_client->get_server_address();
  uint16_t port = sk_ws_client->get_server_port();

  if (addr.isEmpty()) return;

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

    if (!adv.mfr_data.empty()) {
      JsonObject mfr = dev["manufacturer_data"].to<JsonObject>();
      mfr[String(adv.mfr_id)] = bytes_to_hex(adv.mfr_data);
    }
  }

  String body;
  serializeJson(doc, body);

  String url = String("http://") + addr + ":" + String(port) + SK_ADV_PATH;

  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", String("Bearer ") + token);
  http.addHeader("Connection", "close");
  http.setTimeout(3000);

  int code = http.POST(body);
  if (code == 200) {
    ESP_LOGI("BLE-GW", "POST: forwarded %u device(s), heap=%u",
             (unsigned)ads.size(), ESP.getFreeHeap());
  } else if (code == 401 || code == 403) {
    ESP_LOGW("BLE-GW", "POST: auth rejected (HTTP %d) — restarting SK connection", code);
    http.end();
    sk_ws_client->restart();
    return;
  } else {
    ESP_LOGW("BLE-GW", "POST failed: HTTP %d, heap=%u", code, ESP.getFreeHeap());
  }
  http.end();
}

// ---------------------------------------------------------------------------
// HTTP POST background task (runs independently from main loop)
// ---------------------------------------------------------------------------

static void http_post_task(void* param) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(POST_INTERVAL_MS));

    // If init_ble_scanner() bailed out (e.g. C6 slave firmware
    // missing), the NimBLE singletons were never created — touching
    // them here would crash. The gateway then degrades into a
    // network-only no-op until the user runs the OTA updater and
    // reflashes this firmware.
    if (!ble_initialized) {
      continue;
    }

    // Watchdog: restart scan if NimBLE stopped it (e.g. after GATT activity)
    NimBLEScan* scan = NimBLEDevice::getScan();
    if (!scan->isScanning()) {
      ESP_LOGI("BLE-GW", "BLE scan stopped — restarting");
      scan->start(0);
    }

    if (sk_ws_client && sk_ws_client->is_connected()) {
      send_advertisements();
    }

    uint32_t heap = ESP.getFreeHeap();
    if (heap < 20000) {
      ESP_LOGW("BLE-GW", "Low heap: %u bytes", heap);
    }
  }
}

// ---------------------------------------------------------------------------
// GATT Session — manages one BLE connection to a peripheral
// ---------------------------------------------------------------------------

struct PollEntry {
  String uuid;
  unsigned long interval_ms;
  unsigned long last_poll;
};

struct PeriodicWriteEntry {
  String uuid;
  std::vector<uint8_t> data;
  unsigned long interval_ms;
  unsigned long last_write;
};

struct InitWrite {
  String uuid;
  std::vector<uint8_t> data;
};

enum GATTSessionState {
  GATT_IDLE,
  GATT_CONNECTING,
  GATT_DISCOVERING,
  GATT_SUBSCRIBING,
  GATT_ACTIVE,
  GATT_RECONNECTING,
  GATT_ERROR
};

class GATTSession {
public:
  String session_id;
  String mac;
  String service_uuid;
  std::vector<String> notify_uuids;
  std::vector<PollEntry> poll_entries;
  std::vector<InitWrite> init_writes;
  std::vector<PeriodicWriteEntry> periodic_writes;

  GATTSessionState state = GATT_IDLE;
  NimBLEClient* client = nullptr;
  NimBLERemoteService* remote_service = nullptr;
  int retries = 0;
  unsigned long reconnect_at = 0;

  bool isActive() const { return state != GATT_IDLE; }

  void start(const String& sid, JsonObject& cmd) {
    session_id = sid;
    mac = cmd["mac"].as<String>();
    service_uuid = cmd["service"].as<String>();

    notify_uuids.clear();
    if (cmd["notify"].is<JsonArray>()) {
      for (JsonVariant v : cmd["notify"].as<JsonArray>()) {
        notify_uuids.push_back(v.as<String>());
      }
    }

    init_writes.clear();
    if (cmd["init"].is<JsonArray>()) {
      for (JsonObject iw : cmd["init"].as<JsonArray>()) {
        InitWrite entry;
        entry.uuid = iw["uuid"].as<String>();
        String hex = iw["data"].as<String>();
        entry.data = hex_to_bytes(hex.c_str(), hex.length());
        init_writes.push_back(std::move(entry));
      }
    }

    poll_entries.clear();
    if (cmd["poll"].is<JsonArray>()) {
      for (JsonObject pe : cmd["poll"].as<JsonArray>()) {
        PollEntry entry;
        entry.uuid = pe["uuid"].as<String>();
        entry.interval_ms = pe["interval_ms"] | 5000;
        entry.last_poll = 0;
        poll_entries.push_back(std::move(entry));
      }
    }

    periodic_writes.clear();
    if (cmd["periodic_write"].is<JsonArray>()) {
      for (JsonObject pw : cmd["periodic_write"].as<JsonArray>()) {
        PeriodicWriteEntry entry;
        entry.uuid = pw["uuid"].as<String>();
        String hex = pw["data"].as<String>();
        entry.data = hex_to_bytes(hex.c_str(), hex.length());
        entry.interval_ms = pw["interval_ms"] | 5000;
        entry.last_write = 0;
        periodic_writes.push_back(std::move(entry));
      }
    }

    retries = 0;
    doConnect();
  }

  void doConnect() {
    state = GATT_CONNECTING;
    // Look up the BLE address type from the scanner's persistent map.
    // Falls back to type 0 (public) if we have never seen this device
    // — the connect will then most likely fail and we will retry on
    // a later scan once the address type is known.
    uint8_t addr_type = lookup_addr_type(std::string(mac.c_str()));
    ESP_LOGI("GATT", "[%s] connecting to %s (addr_type=%d)...",
             session_id.c_str(), mac.c_str(), (int)addr_type);

    if (!client) {
      client = NimBLEDevice::createClient();
    }

    NimBLEAddress addr(std::string(mac.c_str()), addr_type);
    if (!client->connect(addr)) {
      ESP_LOGW("GATT", "[%s] connect failed", session_id.c_str());
      handleConnectFailure();
      return;
    }

    ESP_LOGI("GATT", "[%s] connected, discovering service %s",
             session_id.c_str(), service_uuid.c_str());
    state = GATT_DISCOVERING;

    remote_service = client->getService(service_uuid.c_str());
    if (!remote_service) {
      ESP_LOGW("GATT", "[%s] service not found", session_id.c_str());
      sendError("Service " + service_uuid + " not found");
      close();
      return;
    }

    // Run any init writes the server asked for. Abort the session
    // setup if any of them fails — leaving the peripheral half-
    // configured and emitting subscribe/poll traffic against an
    // un-initialised state machine is worse than a clean failure
    // the server can retry.
    for (const auto& iw : init_writes) {
      auto chr = remote_service->getCharacteristic(iw.uuid.c_str());
      if (!chr) {
        ESP_LOGW("GATT", "[%s] init write target %s not found",
                 session_id.c_str(), iw.uuid.c_str());
        sendError("Init write characteristic " + iw.uuid + " not found");
        close();
        return;
      }
      bool ok = chr->writeValue(iw.data.data(), iw.data.size(), true);
      if (!ok) {
        ESP_LOGW("GATT", "[%s] init write to %s failed",
                 session_id.c_str(), iw.uuid.c_str());
        sendError("Init write to " + iw.uuid + " failed");
        close();
        return;
      }
      ESP_LOGI("GATT", "[%s] init write to %s OK",
               session_id.c_str(), iw.uuid.c_str());
    }

    state = GATT_SUBSCRIBING;
    for (const auto& uuid : notify_uuids) {
      auto chr = remote_service->getCharacteristic(uuid.c_str());
      if (chr && chr->canNotify()) {
        String sid = session_id;
        String cuuid = uuid;
        chr->subscribe(true,
          [sid, cuuid](NimBLERemoteCharacteristic* pChr,
                       uint8_t* data, size_t length, bool isNotify) {
            JsonDocument doc;
            doc["type"] = "gatt_data";
            doc["session_id"] = sid;
            doc["uuid"] = cuuid;
            doc["data"] = bytes_to_hex(data, length);
            ws_send_json(doc);
          });
        ESP_LOGI("GATT", "[%s] subscribed to %s", session_id.c_str(), uuid.c_str());
      } else {
        ESP_LOGW("GATT", "[%s] char %s not found or not notifiable",
                 session_id.c_str(), uuid.c_str());
      }
    }

    state = GATT_ACTIVE;
    retries = 0;

    JsonDocument doc;
    doc["type"] = "gatt_connected";
    doc["session_id"] = session_id;
    doc["mac"] = mac;
    ws_send_json(doc);

    ESP_LOGI("GATT", "[%s] active", session_id.c_str());
  }

  void handleConnectFailure() {
    retries++;
    if (retries >= GATT_MAX_RETRIES) {
      sendError("Connection failed after " + String(GATT_MAX_RETRIES) + " retries");
      close();
      return;
    }
    state = GATT_RECONNECTING;
    reconnect_at = millis() + GATT_RECONNECT_DELAY_MS * retries;
    ESP_LOGI("GATT", "[%s] retry %d/%d in %lums",
             session_id.c_str(), retries, GATT_MAX_RETRIES,
             GATT_RECONNECT_DELAY_MS * retries);
  }

  void handleWrite(const String& uuid, const String& hex_data) {
    if (state != GATT_ACTIVE || !remote_service) return;
    auto chr = remote_service->getCharacteristic(uuid.c_str());
    if (chr) {
      auto data = hex_to_bytes(hex_data.c_str(), hex_data.length());
      chr->writeValue(data.data(), data.size(), true);
    }
  }

  void loop() {
    // Wraparound-safe comparison: subtract and cast to signed so the
    // check still works after millis() wraps (every ~49 days). The
    // direct `millis() >= reconnect_at` form would briefly fail right
    // after the wrap.
    if (state == GATT_RECONNECTING &&
        (long)(millis() - reconnect_at) >= 0) {
      doConnect();
      return;
    }

    if (state == GATT_ACTIVE && client && !client->isConnected()) {
      ESP_LOGI("GATT", "[%s] peripheral disconnected", session_id.c_str());
      JsonDocument doc;
      doc["type"] = "gatt_disconnected";
      doc["session_id"] = session_id;
      doc["reason"] = "peripheral_disconnected";
      ws_send_json(doc);

      remote_service = nullptr;
      handleConnectFailure();
      return;
    }

    if (state != GATT_ACTIVE || !remote_service) return;

    unsigned long now = millis();

    for (auto& pe : poll_entries) {
      if (now - pe.last_poll >= pe.interval_ms) {
        pe.last_poll = now;
        auto chr = remote_service->getCharacteristic(pe.uuid.c_str());
        if (chr && chr->canRead()) {
          auto val = chr->readValue();
          JsonDocument doc;
          doc["type"] = "gatt_data";
          doc["session_id"] = session_id;
          doc["uuid"] = pe.uuid;
          doc["data"] = bytes_to_hex(
              reinterpret_cast<const uint8_t*>(val.data()), val.size());
          ws_send_json(doc);
        }
      }
    }

    for (auto& pw : periodic_writes) {
      if (now - pw.last_write >= pw.interval_ms) {
        pw.last_write = now;
        auto chr = remote_service->getCharacteristic(pw.uuid.c_str());
        if (chr) {
          chr->writeValue(pw.data.data(), pw.data.size(), true);
        }
      }
    }
  }

  void close() {
    if (client) {
      if (client->isConnected()) {
        client->disconnect();
      }
      NimBLEDevice::deleteClient(client);
      client = nullptr;
    }
    remote_service = nullptr;
    state = GATT_IDLE;
    session_id = "";
    mac = "";
    notify_uuids.clear();
    poll_entries.clear();
    init_writes.clear();
    periodic_writes.clear();
    ESP_LOGI("GATT", "Session closed");
  }

  void sendError(const String& error) {
    JsonDocument doc;
    doc["type"] = "gatt_error";
    doc["session_id"] = session_id;
    doc["error"] = error;
    ws_send_json(doc);
  }
};

// ---------------------------------------------------------------------------
// GATT Manager — manages all sessions
//
// All access to gatt_sessions[] (and the GATTSession instances inside)
// is serialised behind gatt_sessions_mutex. This is needed because the
// scheduling looks like:
//
//   - the SensESP main event loop runs gatt_loop() every 10 ms on the
//     application task,
//   - the esp_websocket_client event handler dispatches incoming
//     ble-provider-api messages on its own dedicated FreeRTOS task,
//   - the BLE host task fires NimBLE notification callbacks (which
//     emit gatt_data over the WS) on yet another task.
//
// All three can touch a session simultaneously without the mutex.
// Wrapping the helper functions below means the only thing the rest
// of the file needs to do is take the mutex itself when it iterates
// gatt_sessions[] directly (currently never — all access goes through
// these helpers or through the WS handler, which holds the lock).
// ---------------------------------------------------------------------------

static GATTSession gatt_sessions[MAX_GATT_SESSIONS];

class GATTSessionLock {
 public:
  GATTSessionLock() { xSemaphoreTake(gatt_sessions_mutex, portMAX_DELAY); }
  ~GATTSessionLock() { xSemaphoreGive(gatt_sessions_mutex); }
  GATTSessionLock(const GATTSessionLock&) = delete;
  GATTSessionLock& operator=(const GATTSessionLock&) = delete;
};

static int activeGATTCount() {
  GATTSessionLock lock;
  int count = 0;
  for (int i = 0; i < MAX_GATT_SESSIONS; i++) {
    if (gatt_sessions[i].isActive()) count++;
  }
  return count;
}

static GATTSession* findSession(const String& session_id) {
  // Note: this returns a raw pointer into the global array. The caller
  // must already hold gatt_sessions_mutex (or be on a code path that
  // serialises with the loop) for the returned pointer to remain valid.
  // The WS handler is currently the only caller, and it grabs the lock
  // before calling this.
  for (int i = 0; i < MAX_GATT_SESSIONS; i++) {
    if (gatt_sessions[i].session_id == session_id) {
      return &gatt_sessions[i];
    }
  }
  return nullptr;
}

static GATTSession* findFreeSlot() {
  // Same locking contract as findSession() above — caller must hold
  // gatt_sessions_mutex.
  for (int i = 0; i < MAX_GATT_SESSIONS; i++) {
    if (!gatt_sessions[i].isActive()) {
      return &gatt_sessions[i];
    }
  }
  return nullptr;
}

static void closeAllGATTSessions() {
  GATTSessionLock lock;
  for (int i = 0; i < MAX_GATT_SESSIONS; i++) {
    if (gatt_sessions[i].isActive()) {
      gatt_sessions[i].close();
    }
  }
}

static void gatt_loop() {
  GATTSessionLock lock;
  for (int i = 0; i < MAX_GATT_SESSIONS; i++) {
    if (gatt_sessions[i].isActive()) {
      gatt_sessions[i].loop();
    }
  }
}

// ---------------------------------------------------------------------------
// WebSocket message handlers
// ---------------------------------------------------------------------------

static void send_hello() {
  JsonDocument doc;
  doc["type"]                    = "hello";
  doc["gateway_id"]              = SensESPBaseApp::get_hostname();
  doc["firmware"]                = "2.0.0";
  doc["max_gatt_connections"]    = MAX_GATT_SESSIONS;
  doc["active_gatt_connections"] = activeGATTCount();
  // Use the active interface MAC (ETH on this board) via the
  // transport-agnostic NetworkProvisioner from M2's refactor. This
  // gives the server a stable hardware identifier for the gateway
  // without needing a separate device-id helper in the SensESP core.
  doc["mac"] = SensESPApp::get()->get_network_provisioner()->mac_address();
  doc["hostname"]                = SensESPBaseApp::get_hostname();
  ws_send_json(doc);
  ESP_LOGI("BLE-WS", "Sent hello");
}

static void send_status() {
  JsonDocument doc;
  doc["type"]                    = "status";
  doc["gateway_id"]              = SensESPBaseApp::get_hostname();
  doc["uptime"]                  = millis() / 1000;
  doc["free_heap"]               = ESP.getFreeHeap();
  doc["active_gatt_connections"] = activeGATTCount();
  doc["max_gatt_connections"]    = MAX_GATT_SESSIONS;
  ws_send_json(doc);
}

static void handle_ws_message(uint8_t* payload, size_t length) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    ESP_LOGW("BLE-WS", "JSON parse error: %s", err.c_str());
    return;
  }

  const char* type = doc["type"];
  if (!type) return;

  if (strcmp(type, "hello_ack") == 0) {
    ESP_LOGI("BLE-WS", "Hello acknowledged");

  } else if (strcmp(type, "gatt_subscribe") == 0) {
    String session_id = doc["session_id"].as<String>();
    ESP_LOGI("BLE-WS", "gatt_subscribe session=%s", session_id.c_str());

    GATTSessionLock lock;  // serialise with gatt_loop() on the main task
    GATTSession* slot = findFreeSlot();
    if (!slot) {
      JsonDocument err_doc;
      err_doc["type"] = "gatt_error";
      err_doc["session_id"] = session_id;
      err_doc["error"] = "No free GATT slots";
      ws_send_json(err_doc);
      return;
    }

    JsonObject cmd = doc.as<JsonObject>();
    slot->start(session_id, cmd);

  } else if (strcmp(type, "gatt_write") == 0) {
    String session_id = doc["session_id"].as<String>();
    GATTSessionLock lock;
    GATTSession* session = findSession(session_id);
    if (session) {
      session->handleWrite(
          doc["uuid"].as<String>(),
          doc["data"].as<String>());
    }

  } else if (strcmp(type, "gatt_close") == 0) {
    String session_id = doc["session_id"].as<String>();
    ESP_LOGI("BLE-WS", "gatt_close session=%s", session_id.c_str());
    GATTSessionLock lock;
    GATTSession* session = findSession(session_id);
    if (session) {
      session->close();
    }
  }
}

// ---------------------------------------------------------------------------
// BLE scanner initialization
// ---------------------------------------------------------------------------

static void init_ble_scanner() {
  ads_mutex = xSemaphoreCreateMutex();
  addr_type_mutex = xSemaphoreCreateMutex();
  gatt_sessions_mutex = xSemaphoreCreateMutex();
  ble_ws_client_mutex = xSemaphoreCreateMutex();
  if (!ads_mutex || !addr_type_mutex || !gatt_sessions_mutex ||
      !ble_ws_client_mutex) {
    ESP_LOGE("BLE-GW",
             "Failed to allocate mutexes — out of memory. Aborting BLE "
             "scanner init; gateway will run in network-only mode.");
    return;
  }

  // Bring up the ESP32-C6 companion chip's Bluetooth controller via
  // ESP-Hosted before NimBLEDevice tries to talk to it. The Arduino
  // core wrapper handles SDIO pin config, esp_hosted_init,
  // connect_to_slave, esp_hosted_bt_controller_init, and _enable.
  // The matching slave firmware on the C6 must already be flashed
  // (see ../p4_c6_ota_updater for the one-shot updater).
  if (!hostedInitBLE()) {
    ESP_LOGE("BLE-GW",
             "hostedInitBLE() failed — the ESP32-C6 is not responding "
             "or the slave firmware is missing/too old. Run the "
             "p4_c6_ota_updater example first to flash esp_hosted_mcu "
             "v2.11.6 onto the C6.");
    return;
  }
  uint32_t maj = 0, min = 0, pat = 0;
  hostedGetSlaveVersion(&maj, &min, &pat);
  ESP_LOGI("BLE-GW", "ESP-Hosted slave firmware: %u.%u.%u",
           (unsigned)maj, (unsigned)min, (unsigned)pat);

  NimBLEDevice::init("");

  NimBLEScan* scan = NimBLEDevice::getScan();
  // Static lifetime — NimBLEScan::setScanCallbacks() does not take
  // ownership of the pointer, it just stores it. Allocating with
  // `new` here would leak on every restart of the scanner.
  static BLEScanCallbacks scan_callbacks;
  scan->setScanCallbacks(&scan_callbacks, true);
  scan->setActiveScan(false);
  scan->setInterval(200);
  scan->setWindow(100);
  scan->setDuplicateFilter(0);
  scan->start(0);
  esp_log_level_set("NimBLEScan", ESP_LOG_WARN);
  ESP_LOGI("BLE-GW", "BLE scan started");

  // Mark the BLE stack as ready so the HTTP POST task and other
  // consumers know it is safe to touch NimBLE singletons.
  ble_initialized.store(true);
}

// ---------------------------------------------------------------------------
// Arduino setup / loop
// ---------------------------------------------------------------------------

// Note: the classic-ESP32 + Aptinex IsolPoE variant of this gateway
// included an `__attribute__((constructor))` that drove GPIO17 HIGH
// before Arduino init, because the LAN8720 oscillator-enable was wired
// to GPIO17 on that board. On the Waveshare ESP32-P4-WIFI6-POE-ETH:
//   - The PHY is IP101GRI, not LAN8720, with a different reset pin
//     (GPIO51) handled by the Arduino ETH driver itself.
//   - GPIO17 is the SDIO D3 line to the onboard ESP32-C6 companion
//     chip. Driving it as a plain GPIO would actively break the
//     ESP-Hosted SDIO bus and prevent BLE from coming up.
// So no early GPIO hack here — the Waveshare hardware Just Works
// with the EthernetConfig::waveshare_esp32p4_poe() preset.

void setup() {
  SetupLogging(ESP_LOG_INFO);

  // Build SensESP application:
  // - Native RMII Ethernet via Waveshare ESP32-P4-WIFI6-POE-ETH preset
  //   (IP101GRI PHY, M2 EthernetProvisioner)
  // - WiFi disabled (the P4 has no native radio anyway, and we don't
  //   ask the C6 to handle WiFi — only BLE — to keep the SDIO bus
  //   bandwidth budget for BLE traffic)
  // - Signal K server discovered via mDNS (or configured via web UI)
  // - OTA enabled (over the same ethernet link)
  SensESPAppBuilder builder;
  auto sensesp_app = builder.set_hostname(GATEWAY_HOSTNAME)
      ->set_ethernet(EthernetConfig::waveshare_esp32p4_poe())
      ->disable_wifi()
      ->enable_ota("ble-gw-ota")
      ->get_app();

  sk_ws_client = sensesp_app->get_ws_client();

  // Initialize BLE scanner
  init_ble_scanner();

  // React to SKWSClient connection state changes
  sk_ws_client->connect_to(
      new LambdaConsumer<SKWSConnectionState>(
          [](SKWSConnectionState state) {
            if (state == SKWSConnectionState::kSKWSConnected) {
              ESP_LOGI("BLE-GW", "SK server connected — starting BLE gateway services");
              init_custom_websocket();
            }
            if (state == SKWSConnectionState::kSKWSDisconnected) {
              ESP_LOGI("BLE-GW", "SK server disconnected — stopping BLE gateway services");
              destroy_custom_websocket();
              closeAllGATTSessions();
            }
          }));

  // Start HTTP POST in a background FreeRTOS task
  xTaskCreate(http_post_task, "http_post", 8192, NULL, 1, NULL);

  // Periodic status over custom WS
  event_loop()->onRepeat(STATUS_INTERVAL_MS, []() {
    if (ble_ws_connected) {
      send_status();
    }
  });

  // GATT session loop (reconnects, periodic reads/writes)
  event_loop()->onRepeat(10, []() {
    gatt_loop();
  });

  ESP_LOGI("BLE-GW", "Setup complete, heap=%u", ESP.getFreeHeap());
}

void loop() {
  event_loop()->tick();
}
