/**
 * @file main.cpp
 * @brief BLE-to-Ethernet gateway for Signal K
 *
 * Scans for BLE advertisements and forwards raw manufacturer data to the
 * Signal K server over HTTP.  The server handles device identification,
 * decryption, parsing, and Signal K delta emission.
 *
 * Also maintains a WebSocket connection for GATT commands: the server can
 * instruct this gateway to connect to a BLE peripheral, subscribe to
 * notification characteristics, and stream the data back over the WebSocket.
 *
 * Auth: Signal K access request flow — token stored in NVS (survives reboot).
 * Source ID: MAC address + hostname sent in WebSocket hello message.
 *
 * Standalone firmware — no SensESP framework.  Uses:
 * - Arduino ESP32 ETH for Ethernet (Aptinex IsolPoE board)
 * - NimBLE-Arduino for BLE scanning + GATT client
 * - HTTPClient + ArduinoJson for POSTing to the server API
 * - WebSocketsClient for bidirectional GATT command channel
 * - Preferences (ESP32 NVS) for persisting the JWT token across reboots
 */

#define ETH_PHY_TYPE    ETH_PHY_LAN8720
#define ETH_PHY_ADDR    1
#define ETH_PHY_MDC     23
#define ETH_PHY_MDIO    18
#define ETH_PHY_POWER   -1                // Don't let driver touch GPIO17
#define ETH_CLK_MODE    ETH_CLOCK_GPIO0_IN

#include <ETH.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <WebSocketsClient.h>
#include <Preferences.h>

#define OSC_ENABLE_PIN  17  // 50MHz crystal oscillator enable

// Keep BT memory — NimBLE needs it but Arduino doesn't detect external lib
extern "C" bool btInUse() { return true; }

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

static constexpr const char* SK_SERVER_HOST = "192.168.0.122";
static constexpr uint16_t SK_SERVER_PORT = 4000;
static constexpr unsigned long POST_INTERVAL_MS = 2000;
#ifndef GATEWAY_HOSTNAME
#define GATEWAY_HOSTNAME "signalk-ble-gw"
#endif

// Server-side API paths (v2 — owned by server, not bt-sensors plugin)
static constexpr const char* SK_ADV_PATH =
    "/signalk/v2/api/ble/gateway/advertisements";
static constexpr const char* SK_WS_PATH_BASE =
    "/signalk/v2/api/ble/gateway/ws?token=";

// Signal K access request paths (v1 — standard SK auth)
static constexpr const char* SK_ACCESS_REQUEST_PATH =
    "/signalk/v1/access/requests";
static constexpr const char* SK_LOGIN_STATUS_PATH =
    "/skServer/loginStatus";

static constexpr int MAX_GATT_SESSIONS = 3;
static constexpr unsigned long STATUS_INTERVAL_MS = 30000;
static constexpr unsigned long GATT_RECONNECT_DELAY_MS = 3000;
static constexpr int GATT_MAX_RETRIES = 5;

// NVS namespace and key for persisting the JWT token
static constexpr const char* NVS_NAMESPACE = "ble_gw";
static constexpr const char* NVS_TOKEN_KEY  = "sk_token";

// Poll interval for access request approval
static constexpr unsigned long AUTH_POLL_INTERVAL_MS = 5000;

// ---------------------------------------------------------------------------
// Auth state machine
// ---------------------------------------------------------------------------

enum class AuthState {
  CHECK_SECURITY,   // GET /skServer/loginStatus — skip auth if not required
  LOAD_TOKEN,       // Try to load token from NVS
  REQUEST_ACCESS,   // POST /signalk/v1/access/requests
  POLLING,          // GET polling href, waiting for APPROVED
  AUTHENTICATED,    // Have valid token (or security disabled)
  AUTH_DENIED       // Permanent failure
};

static AuthState auth_state = AuthState::CHECK_SECURITY;
static String    sk_token;          // Current JWT (empty = not authenticated)
static String    auth_poll_href;    // href returned by server for polling
static unsigned long auth_last_poll = 0;

// ---------------------------------------------------------------------------
// Ethernet state
// ---------------------------------------------------------------------------

static bool eth_connected = false;

void onEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("Ethernet Started");
      ETH.setHostname(GATEWAY_HOSTNAME);
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("Ethernet Link Connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("Ethernet Got IP: ");
      Serial.println(ETH.localIP());
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
      Serial.println("Ethernet Lost IP");
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("Ethernet Disconnected");
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("Ethernet Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// NVS helpers
// ---------------------------------------------------------------------------

static Preferences prefs;

static String nvs_load_token() {
  prefs.begin(NVS_NAMESPACE, true); // read-only
  String token = prefs.getString(NVS_TOKEN_KEY, "");
  prefs.end();
  return token;
}

static void nvs_save_token(const String& token) {
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putString(NVS_TOKEN_KEY, token);
  prefs.end();
  Serial.println("Auth: token saved to NVS");
}

static void nvs_clear_token() {
  prefs.begin(NVS_NAMESPACE, false);
  prefs.remove(NVS_TOKEN_KEY);
  prefs.end();
  Serial.println("Auth: NVS token cleared");
}

// ---------------------------------------------------------------------------
// Source identification helpers
// ---------------------------------------------------------------------------

// Returns the Ethernet MAC address as "AA:BB:CC:DD:EE:FF"
static String get_mac_address() {
  return ETH.macAddress();
}

// Returns stable clientId for access requests: "ble-gw-<MAC>"
static String get_client_id() {
  return String("ble-gw-") + get_mac_address();
}

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
// Advertisement buffer (shared between BLE task and main loop)
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
// WebSocket client (declared here so send_advertisements can reference them)
// ---------------------------------------------------------------------------

static WebSocketsClient ws;
static bool ws_initialized = false;

// ---------------------------------------------------------------------------
// HTTP POST to server (advertisements)
// ---------------------------------------------------------------------------

static void send_advertisements() {
  if (!eth_connected) return;

  xSemaphoreTake(ads_mutex, portMAX_DELAY);
  if (pending_ads.empty()) {
    xSemaphoreGive(ads_mutex);
    return;
  }
  auto ads = std::move(pending_ads);
  pending_ads.clear();
  xSemaphoreGive(ads_mutex);

  JsonDocument doc;
  doc["gateway_id"] = GATEWAY_HOSTNAME;

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

  String url = String("http://") + SK_SERVER_HOST + ":" +
               String(SK_SERVER_PORT) + SK_ADV_PATH;

  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", String("Bearer ") + sk_token);
  http.setTimeout(3000);

  int code = http.POST(body);
  if (code == 200) {
    Serial.printf("POST: forwarded %u device(s), heap=%u\n",
                  (unsigned)ads.size(), ESP.getFreeHeap());
  } else if (code == 401 || code == 403) {
    Serial.printf("POST: auth rejected (HTTP %d) — clearing token, re-checking security\n", code);
    http.end();
    nvs_clear_token();
    sk_token = "";
    ws_initialized = false;  // force WS reconnect with new token after re-auth
    ws.disconnect();
    auth_state = AuthState::CHECK_SECURITY;
    return;
  } else {
    Serial.printf("POST failed: HTTP %d, heap=%u\n", code, ESP.getFreeHeap());
  }
  http.end();
}

// ---------------------------------------------------------------------------
// HTTP POST background task (runs independently from main loop)
// ---------------------------------------------------------------------------

static void http_post_task(void* param) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(POST_INTERVAL_MS));

    // Watchdog: restart scan if NimBLE stopped it (e.g. after GATT activity)
    NimBLEScan* scan = NimBLEDevice::getScan();
    if (!scan->isScanning()) {
      Serial.println("BLE scan stopped — restarting");
      scan->start(0);
    }

    if (auth_state == AuthState::AUTHENTICATED) {
      send_advertisements();
    }
    uint32_t heap = ESP.getFreeHeap();
    if (heap < 20000) {
      Serial.printf("WARNING: Low heap: %u bytes\n", heap);
    }
  }
}

// ---------------------------------------------------------------------------
// Signal K access request flow
// ---------------------------------------------------------------------------

// GET /skServer/loginStatus → check if server requires authentication.
// Returns true if auth is required, false if security is disabled.
static bool check_security_required() {
  String url = String("http://") + SK_SERVER_HOST + ":" +
               String(SK_SERVER_PORT) + SK_LOGIN_STATUS_PATH;

  HTTPClient http;
  http.begin(url);
  http.setTimeout(5000);
  int code = http.GET();
  String resp = http.getString();
  http.end();

  if (code != 200) {
    Serial.printf("Auth: loginStatus failed HTTP %d — assuming auth required\n", code);
    return true;
  }

  JsonDocument doc;
  if (deserializeJson(doc, resp) != DeserializationError::Ok) {
    Serial.println("Auth: loginStatus parse error — assuming auth required");
    return true;
  }

  bool required = doc["authenticationRequired"] | true;
  Serial.printf("Auth: authenticationRequired=%s\n", required ? "true" : "false");
  return required;
}

// POST /signalk/v1/access/requests → receive href to poll
// Returns true if request submitted (or already pending), false on hard error.
static bool submit_access_request() {
  String clientId = get_client_id();
  Serial.printf("Auth: submitting access request, clientId=%s\n", clientId.c_str());

  JsonDocument req_doc;
  req_doc["clientId"]    = clientId;
  req_doc["description"] = String("BLE Gateway ") + GATEWAY_HOSTNAME;
  req_doc["permissions"] = "readwrite";
  String req_body;
  serializeJson(req_doc, req_body);

  String url = String("http://") + SK_SERVER_HOST + ":" +
               String(SK_SERVER_PORT) + SK_ACCESS_REQUEST_PATH;

  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(5000);

  int code = http.POST(req_body);
  String resp = http.getString();
  http.end();

  Serial.printf("Auth: access request response: HTTP %d: %s\n", code, resp.c_str());

  if (code == 202 || code == 400) {
    // 202 = new request accepted, 400 may mean request already pending
    // In both cases, parse href from response body
    JsonDocument resp_doc;
    DeserializationError err = deserializeJson(resp_doc, resp);
    if (err) {
      Serial.printf("Auth: failed to parse response: %s\n", err.c_str());
      return false;
    }
    const char* href = resp_doc["href"];
    if (!href) {
      // 400 with no href — genuine error
      Serial.println("Auth: server rejected request (no href in response)");
      return false;
    }
    auth_poll_href = String(href);
    auth_state = AuthState::POLLING;
    auth_last_poll = 0; // poll immediately
    Serial.printf("Auth: polling href: %s\n", auth_poll_href.c_str());
    return true;
  }

  Serial.printf("Auth: unexpected response: HTTP %d\n", code);
  return false;
}

// Poll the access request href.
// Returns true when done (APPROVED or DENIED), false while still pending.
static bool poll_access_request() {
  unsigned long now = millis();
  if (now - auth_last_poll < AUTH_POLL_INTERVAL_MS) {
    return false;
  }
  auth_last_poll = now;

  String url = String("http://") + SK_SERVER_HOST + ":" +
               String(SK_SERVER_PORT) + auth_poll_href;

  HTTPClient http;
  http.begin(url);
  http.setTimeout(5000);
  int code = http.GET();
  String resp = http.getString();
  http.end();

  if (code != 200) {
    Serial.printf("Auth: poll failed: HTTP %d\n", code);
    if (code == 404 || code == 500) {
      // Request no longer exists (server restarted) — submit a new one
      Serial.println("Auth: stale poll href, re-submitting access request");
      auth_poll_href = "";
      auth_state = AuthState::REQUEST_ACCESS;
    }
    return false;
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, resp);
  if (err) {
    Serial.printf("Auth: poll parse error: %s\n", err.c_str());
    return false;
  }

  const char* state = doc["state"];
  if (!state) return false;

  if (strcmp(state, "PENDING") == 0) {
    Serial.println("Auth: waiting for admin approval...");
    return false;
  }

  if (strcmp(state, "COMPLETED") == 0) {
    // Token is nested: { "accessRequest": { "permission": "APPROVED", "token": "..." } }
    const char* permission = doc["accessRequest"]["permission"];
    if (permission && strcmp(permission, "APPROVED") == 0) {
      const char* token = doc["accessRequest"]["token"];
      if (token) {
        sk_token = String(token);
        nvs_save_token(sk_token);
        auth_state = AuthState::AUTHENTICATED;
        Serial.println("Auth: APPROVED — token saved, proceeding");
        return true;
      }
    }
    // DENIED or no token
    Serial.printf("Auth: request completed with permission=%s — DENIED\n",
                  permission ? permission : "null");
    auth_state = AuthState::AUTH_DENIED;
    return true;
  }

  return false;
}

// Run auth state machine — call this from loop() when eth_connected.
// Returns true when authenticated and ready to proceed.
static bool auth_loop() {
  switch (auth_state) {
    case AuthState::CHECK_SECURITY:
      if (!check_security_required()) {
        Serial.println("Auth: security disabled — proceeding without token");
        auth_state = AuthState::AUTHENTICATED;
      } else {
        auth_state = AuthState::LOAD_TOKEN;
      }
      return false;

    case AuthState::LOAD_TOKEN:
      sk_token = nvs_load_token();
      if (sk_token.length() > 0) {
        Serial.println("Auth: loaded token from NVS");
        auth_state = AuthState::AUTHENTICATED;
      } else {
        Serial.println("Auth: no stored token, requesting access");
        auth_state = AuthState::REQUEST_ACCESS;
      }
      return false;

    case AuthState::REQUEST_ACCESS:
      if (submit_access_request()) {
        return false; // submitted, now polling
      }
      // Retry after delay
      delay(5000);
      return false;

    case AuthState::POLLING:
      poll_access_request();
      return false;

    case AuthState::AUTHENTICATED:
      return true;

    case AuthState::AUTH_DENIED:
      Serial.println("Auth: DENIED — halting. Please reset and request new access.");
      delay(30000);
      return false;
  }
  return false;
}

// ---------------------------------------------------------------------------
// GATT Session — manages one BLE connection to a peripheral
// ---------------------------------------------------------------------------

class GATTSession;
static void ws_send_json(JsonDocument& doc);

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

    // Parse notify UUIDs
    notify_uuids.clear();
    if (cmd["notify"].is<JsonArray>()) {
      for (JsonVariant v : cmd["notify"].as<JsonArray>()) {
        notify_uuids.push_back(v.as<String>());
      }
    }

    // Parse init writes
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

    // Parse poll entries
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

    // Parse periodic writes
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
    Serial.printf("GATT [%s] connecting to %s...\n",
                  session_id.c_str(), mac.c_str());

    if (!client) {
      client = NimBLEDevice::createClient();
    }

    NimBLEAddress addr(std::string(mac.c_str()), 0);
    if (!client->connect(addr)) {
      Serial.printf("GATT [%s] connect failed\n", session_id.c_str());
      handleConnectFailure();
      return;
    }

    Serial.printf("GATT [%s] connected, discovering service %s\n",
                  session_id.c_str(), service_uuid.c_str());
    state = GATT_DISCOVERING;

    remote_service = client->getService(service_uuid.c_str());
    if (!remote_service) {
      Serial.printf("GATT [%s] service not found\n", session_id.c_str());
      sendError("Service " + service_uuid + " not found");
      close();
      return;
    }

    // Execute init writes
    for (const auto& iw : init_writes) {
      auto chr = remote_service->getCharacteristic(iw.uuid.c_str());
      if (chr) {
        chr->writeValue(iw.data.data(), iw.data.size(), true);
        Serial.printf("GATT [%s] init write to %s\n",
                      session_id.c_str(), iw.uuid.c_str());
      }
    }

    // Subscribe to notification characteristics
    state = GATT_SUBSCRIBING;
    for (const auto& uuid : notify_uuids) {
      auto chr = remote_service->getCharacteristic(uuid.c_str());
      if (chr && chr->canNotify()) {
        // Capture session_id and uuid by value for the lambda
        String sid = session_id;
        String cuuid = uuid;
        chr->subscribe(true,
          [sid, cuuid](NimBLERemoteCharacteristic* pChr,
                       uint8_t* data, size_t length, bool isNotify) {
            // Send notification data over WebSocket
            JsonDocument doc;
            doc["type"] = "gatt_data";
            doc["session_id"] = sid;
            doc["uuid"] = cuuid;
            doc["data"] = bytes_to_hex(data, length);
            ws_send_json(doc);
          });
        Serial.printf("GATT [%s] subscribed to %s\n",
                      session_id.c_str(), uuid.c_str());
      } else {
        Serial.printf("GATT [%s] char %s not found or not notifiable\n",
                      session_id.c_str(), uuid.c_str());
      }
    }

    state = GATT_ACTIVE;
    retries = 0;

    // Send connected message
    JsonDocument doc;
    doc["type"] = "gatt_connected";
    doc["session_id"] = session_id;
    doc["mac"] = mac;
    ws_send_json(doc);

    Serial.printf("GATT [%s] active\n", session_id.c_str());
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
    Serial.printf("GATT [%s] retry %d/%d in %lums\n",
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
    if (state == GATT_RECONNECTING && millis() >= reconnect_at) {
      doConnect();
      return;
    }

    if (state == GATT_ACTIVE && client && !client->isConnected()) {
      // Peripheral disconnected
      Serial.printf("GATT [%s] peripheral disconnected\n", session_id.c_str());
      JsonDocument doc;
      doc["type"] = "gatt_disconnected";
      doc["session_id"] = session_id;
      doc["reason"] = "peripheral_disconnected";
      ws_send_json(doc);

      remote_service = nullptr;
      handleConnectFailure();  // Will reconnect with backoff
      return;
    }

    if (state != GATT_ACTIVE || !remote_service) return;

    unsigned long now = millis();

    // Poll reads
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

    // Periodic writes
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
    Serial.printf("GATT session closed\n");
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
// ---------------------------------------------------------------------------

static GATTSession gatt_sessions[MAX_GATT_SESSIONS];

static int activeGATTCount() {
  int count = 0;
  for (int i = 0; i < MAX_GATT_SESSIONS; i++) {
    if (gatt_sessions[i].isActive()) count++;
  }
  return count;
}

static GATTSession* findSession(const String& session_id) {
  for (int i = 0; i < MAX_GATT_SESSIONS; i++) {
    if (gatt_sessions[i].session_id == session_id) {
      return &gatt_sessions[i];
    }
  }
  return nullptr;
}

static GATTSession* findFreeSlot() {
  for (int i = 0; i < MAX_GATT_SESSIONS; i++) {
    if (!gatt_sessions[i].isActive()) {
      return &gatt_sessions[i];
    }
  }
  return nullptr;
}

static void closeAllGATTSessions() {
  for (int i = 0; i < MAX_GATT_SESSIONS; i++) {
    if (gatt_sessions[i].isActive()) {
      gatt_sessions[i].close();
    }
  }
}

static void gatt_loop() {
  for (int i = 0; i < MAX_GATT_SESSIONS; i++) {
    if (gatt_sessions[i].isActive()) {
      gatt_sessions[i].loop();
    }
  }
}

// ---------------------------------------------------------------------------
// WebSocket client
// ---------------------------------------------------------------------------

static bool ws_connected = false;

static void ws_send_json(JsonDocument& doc) {
  if (!ws_connected) return;
  String msg;
  serializeJson(doc, msg);
  ws.sendTXT(msg);
}

static void send_hello() {
  JsonDocument doc;
  doc["type"]                   = "hello";
  doc["gateway_id"]             = GATEWAY_HOSTNAME;
  doc["firmware"]               = "1.0.0";
  doc["max_gatt_connections"]   = MAX_GATT_SESSIONS;
  doc["active_gatt_connections"] = activeGATTCount();
  doc["mac"]                    = get_mac_address();
  doc["hostname"]               = GATEWAY_HOSTNAME;
  ws_send_json(doc);
  Serial.println("WS: sent hello");
}

static void send_status() {
  JsonDocument doc;
  doc["type"]                   = "status";
  doc["gateway_id"]             = GATEWAY_HOSTNAME;
  doc["uptime"]                 = millis() / 1000;
  doc["free_heap"]              = ESP.getFreeHeap();
  doc["active_gatt_connections"] = activeGATTCount();
  doc["max_gatt_connections"]   = MAX_GATT_SESSIONS;
  ws_send_json(doc);
}

static void handle_ws_message(uint8_t* payload, size_t length) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.printf("WS: JSON parse error: %s\n", err.c_str());
    return;
  }

  const char* type = doc["type"];
  if (!type) return;

  if (strcmp(type, "hello_ack") == 0) {
    Serial.println("WS: hello acknowledged");

  } else if (strcmp(type, "gatt_subscribe") == 0) {
    String session_id = doc["session_id"].as<String>();
    Serial.printf("WS: gatt_subscribe session=%s\n", session_id.c_str());

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
    GATTSession* session = findSession(session_id);
    if (session) {
      session->handleWrite(
          doc["uuid"].as<String>(),
          doc["data"].as<String>());
    }

  } else if (strcmp(type, "gatt_close") == 0) {
    String session_id = doc["session_id"].as<String>();
    Serial.printf("WS: gatt_close session=%s\n", session_id.c_str());
    GATTSession* session = findSession(session_id);
    if (session) {
      session->close();
    }
  }
}

static void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED: {
      // WebSocketsClient passes the WS close code as a uint16 in payload
      // when length == 2.  A 4401 close means the server rejected our token.
      uint16_t close_code = 0;
      if (length == 2 && payload) {
        close_code = (payload[0] << 8) | payload[1];
      }
      if (close_code == 4401) {
        Serial.println("WS: token rejected by server — clearing token, re-auth");
        nvs_clear_token();
        sk_token = "";
        ws_initialized = false;
        auth_state = AuthState::CHECK_SECURITY;
      } else {
        Serial.println("WS: disconnected");
      }
      ws_connected = false;
      break;
    }

    case WStype_CONNECTED:
      Serial.printf("WS: connected to %s\n", (char*)payload);
      ws_connected = true;
      // Close any stale GATT sessions from a previous server session.
      // The new server has no record of them and will re-subscribe fresh.
      closeAllGATTSessions();
      send_hello();
      break;

    case WStype_TEXT:
      handle_ws_message(payload, length);
      break;

    case WStype_PING:
    case WStype_PONG:
      break;

    default:
      break;
  }
}

// Initialize WebSocket with the current token.
// Safe to call multiple times — ws.begin() reinitializes cleanly.
static void init_websocket() {
  String ws_path = String(SK_WS_PATH_BASE) + sk_token;
  ws.begin(SK_SERVER_HOST, SK_SERVER_PORT, ws_path);
  ws.onEvent(webSocketEvent);
  ws.setReconnectInterval(5000);
  ws_initialized = true;
  Serial.printf("WS: connecting to %s:%u%s\n",
                SK_SERVER_HOST, SK_SERVER_PORT, ws_path.c_str());
}

// ---------------------------------------------------------------------------
// Arduino setup / loop
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.printf("=== BLE Gateway %s ===\n", GATEWAY_HOSTNAME);
  Serial.printf("Heap: %u bytes free\n", ESP.getFreeHeap());
  Serial.printf("Flash: %u bytes, SDK: %s\n", ESP.getFlashChipSize(), ESP.getSdkVersion());

  // Enable 50MHz crystal oscillator before ETH init
  Serial.println("Enabling oscillator...");
  pinMode(OSC_ENABLE_PIN, OUTPUT);
  digitalWrite(OSC_ENABLE_PIN, HIGH);
  delay(50);

  Serial.println("Starting Ethernet...");
  WiFi.mode(WIFI_OFF);
  Network.onEvent(onEvent);
  ETH.begin();
  Serial.printf("Heap after ETH: %u\n", ESP.getFreeHeap());

  ads_mutex = xSemaphoreCreateMutex();

  // Initialize NimBLE scanner
  Serial.println("Starting NimBLE...");
  NimBLEDevice::init("");
  Serial.printf("Heap after NimBLE: %u\n", ESP.getFreeHeap());

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new BLEScanCallbacks(), true);
  scan->setActiveScan(false);
  scan->setInterval(200);   // 200ms interval
  scan->setWindow(100);     // 100ms window → 50% duty cycle
  scan->setDuplicateFilter(0);
  scan->start(0);
  Serial.println("BLE scan started");

  // Start HTTP POST in a background task so it doesn't block ws.loop()
  xTaskCreate(http_post_task, "http_post", 8192, NULL, 1, NULL);
  Serial.println("HTTP POST task created");

  Serial.printf("Setup complete! Heap: %u\n", ESP.getFreeHeap());
  // Auth and WebSocket init happen in loop() once Ethernet is up
}

static unsigned long last_status = 0;

void loop() {
  unsigned long now = millis();

  if (!eth_connected) {
    delay(100);
    return;
  }

  // Run auth state machine
  bool authenticated = auth_loop();

  // Once authenticated, initialize WebSocket if not yet done
  // (or reinitialize after token refresh)
  if (authenticated && !ws_initialized) {
    init_websocket();
  }

  // Service WebSocket (must be called frequently — never block this loop)
  if (ws_initialized) {
    ws.loop();
  }

  // Service GATT sessions (reconnects, periodic writes, poll reads)
  gatt_loop();

  // Send periodic status over WebSocket
  if (ws_connected && now - last_status >= STATUS_INTERVAL_MS) {
    last_status = now;
    send_status();
  }

  delay(1);
}
