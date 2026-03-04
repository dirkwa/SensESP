/**
 * @file main.cpp
 * @brief BLE-to-Ethernet gateway for Signal K
 *
 * Scans for BLE advertisements and forwards raw manufacturer data to the
 * bt-sensors-plugin-sk Signal K plugin over HTTP.  The plugin handles
 * device identification, decryption, parsing, and Signal K delta emission.
 *
 * Standalone firmware — no SensESP framework.  Uses:
 * - Arduino ESP32 ETH for Ethernet (Aptinex IsolPoE board)
 * - NimBLE-Arduino for BLE scanning
 * - HTTPClient + ArduinoJson for POSTing to the plugin API
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

#define OSC_ENABLE_PIN  17  // 50MHz crystal oscillator enable

// Keep BT memory — NimBLE needs it but Arduino doesn't detect external lib
extern "C" bool btInUse() { return true; }

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

static constexpr const char* SK_SERVER_HOST = "192.168.0.122";
static constexpr uint16_t SK_SERVER_PORT = 4000;
static constexpr unsigned long POST_INTERVAL_MS = 2000;
static constexpr const char* GATEWAY_HOSTNAME = "signalk-ble-gw";

// ---------------------------------------------------------------------------
// Ethernet state
// ---------------------------------------------------------------------------

static bool eth_connected = false;
static String plugin_api_url;

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
      plugin_api_url = String("http://") + SK_SERVER_HOST + ":" +
                       String(SK_SERVER_PORT) +
                       "/plugins/bt-sensors-plugin-sk/api/gateway/advertisements";
      Serial.print("Plugin URL: ");
      Serial.println(plugin_api_url);
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
      Serial.println("Ethernet Lost IP");
      eth_connected = false;
      plugin_api_url = "";
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("Ethernet Disconnected");
      eth_connected = false;
      plugin_api_url = "";
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("Ethernet Stopped");
      eth_connected = false;
      plugin_api_url = "";
      break;
    default:
      break;
  }
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
    if (!device->haveManufacturerData()) return;

    std::string mfr_raw = device->getManufacturerData();
    const uint8_t* data = reinterpret_cast<const uint8_t*>(mfr_raw.data());
    size_t len = mfr_raw.size();

    if (len < 3) return;

    uint16_t mfr_id = data[0] | (data[1] << 8);

    BleAdvertisement adv;
    adv.mac = device->getAddress().toString();
    adv.name = device->haveName() ? device->getName() : "";
    adv.rssi = device->getRSSI();
    adv.mfr_id = mfr_id;
    adv.mfr_data.assign(data + 2, data + len);

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

    JsonObject mfr = dev["manufacturer_data"].to<JsonObject>();
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
    Serial.printf("Forwarded %u device(s)\n", (unsigned)ads.size());
  } else {
    Serial.printf("POST failed: HTTP %d\n", code);
  }
  http.end();
}

// ---------------------------------------------------------------------------
// Arduino setup / loop
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("Signal K BLE Gateway starting...");

  // Enable 50MHz crystal oscillator before ETH init
  pinMode(OSC_ENABLE_PIN, OUTPUT);
  digitalWrite(OSC_ENABLE_PIN, HIGH);
  delay(50);

  WiFi.mode(WIFI_OFF);
  Network.onEvent(onEvent);
  ETH.begin();

  ads_mutex = xSemaphoreCreateMutex();

  // Initialize NimBLE scanner
  NimBLEDevice::init("");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new BLEScanCallbacks(), true);
  scan->setActiveScan(false);
  scan->setInterval(100);
  scan->setWindow(99);
  scan->setDuplicateFilter(0);
  scan->start(0);

  Serial.println("BLE scanning started, waiting for Ethernet...");
}

static unsigned long last_post = 0;

void loop() {
  unsigned long now = millis();
  if (now - last_post >= POST_INTERVAL_MS) {
    last_post = now;
    send_advertisements();
  }
  delay(1);
}
