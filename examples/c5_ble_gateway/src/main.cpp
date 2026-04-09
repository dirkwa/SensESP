/**
 * @file main.cpp
 * @brief SensESP BLE gateway on ESP32-C5 (native BLE + WiFi).
 *
 * The ESP32-C5 has a native BT controller — BLE scanning is
 * rock-solid (~15 hits/sec) with no stalls, unlike the P4+C6
 * SDIO bridge setup.
 *
 * Uses framework = espidf, arduino so we own the sdkconfig
 * (fixes the PSRAM crash with pioarduino's prebuilt Arduino libs).
 */

#include "sensesp/net/ble/ble_signalk_gateway.h"
#include "sensesp/net/ble/native_bluedroid_ble.h"
#include "sensesp_app_builder.h"

using namespace sensesp;

static std::shared_ptr<NativeBLE> g_ble;
static std::shared_ptr<BLESignalKGateway> g_gateway;

void setup() {
  SetupLogging(ESP_LOG_INFO);

  SensESPAppBuilder builder;
  auto app = builder.set_hostname(GATEWAY_HOSTNAME)
                 ->set_wifi_client("MOIN", "Moin2018!")
                 ->enable_ota("c5-ble-gw-ota")
                 ->set_sk_server("192.168.0.148", 3000)
                 ->get_app();

  g_ble = std::make_shared<NativeBLE>();

  g_gateway =
      std::make_shared<BLESignalKGateway>(g_ble, app->get_ws_client());
  g_gateway->start();

  event_loop()->onRepeat(5000, []() {
    ESP_LOGI(
        "GW",
        "alive — uptime=%lus heap=%u ble_hits=%u ble_scan=%d gw_rx=%u "
        "gw_posted=%u gw_dropped=%u post_ok=%u post_fail=%u ws_up=%d",
        (unsigned long)(millis() / 1000), (unsigned)ESP.getFreeHeap(),
        (unsigned)(g_ble ? g_ble->scan_hit_count() : 0),
        (int)(g_ble ? g_ble->is_scanning() : false),
        (unsigned)(g_gateway ? g_gateway->advertisements_received() : 0),
        (unsigned)(g_gateway ? g_gateway->advertisements_posted() : 0),
        (unsigned)(g_gateway ? g_gateway->advertisements_dropped() : 0),
        (unsigned)(g_gateway ? g_gateway->http_post_success() : 0),
        (unsigned)(g_gateway ? g_gateway->http_post_fail() : 0),
        (int)(g_gateway ? g_gateway->control_ws_connected() : false));
  });
}

void loop() { event_loop()->tick(); }
