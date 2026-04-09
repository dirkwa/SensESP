/**
 * @file main.cpp
 * @brief SensESP P4 + Bluedroid BLE gateway example.
 *
 * Minimal SensESP app that exercises the full BLE-gateway stack built
 * on top of the new `framework = espidf, arduino` build mode:
 *
 *   1. SensESP core with native RMII Ethernet on the Waveshare
 *      ESP32-P4-WIFI6-POE-ETH board (via the EthernetProvisioner
 *      introduced in #940).
 *   2. EspHostedBluedroidBLE — a BLEProvisioner that brings up the
 *      Bluedroid host stack on top of esp_hosted's Bluedroid VHCI
 *      transport, routing HCI traffic to the onboard ESP32-C6
 *      companion chip.
 *   3. BLESignalKGateway — bridges BLE advertisements to
 *      signalk-server's ble-provider-api via an HTTP POST channel
 *      plus a separate control WebSocket, both authenticated with
 *      the JWT obtained by the main SensESP SK connection.
 *
 * The gateway class is a first-class SensESP component living under
 * sensesp/net/ble/. Downstream users who want a BLE gateway do not
 * have to copy any protocol code into their own sketches — they just
 * instantiate EspHostedBluedroidBLE + BLESignalKGateway and call
 * start(), as shown below.
 *
 * ## Known limitation — C6 slave firmware BLE forwarding
 *
 * The Arduino-ESP32-distributed esp_hosted C6 slave firmware
 * (esp32c6-v2.11.6.bin) does not currently appear to forward LE
 * Advertising Reports over HCI to the P4 host, under either
 * NimBLE or Bluedroid. The same silence was reproduced on two
 * separate Waveshare boards. Until that is resolved upstream in
 * esp-hosted-mcu, this example will boot cleanly and successfully
 * connect to signalk-server (the hello handshake works, status
 * frames are sent, HTTP POST plumbing is verified), but the POSTed
 * advertisement batches will always be empty. See the example's
 * README.md for details.
 *
 * ## Verification checklist
 *
 *   1. Device boots, Ethernet link up, DHCP lease obtained
 *   2. Main SensESP SK websocket connects to signalk-server
 *   3. BLE gateway control WS connects at /signalk/v2/api/ble/gateway/ws
 *   4. Hello message sent on the control WS, hello_ack received
 *   5. Status messages sent every 30 seconds
 *   6. HTTP POSTs every 2 seconds (currently empty bodies due to
 *      the C6 slave firmware issue)
 *   7. Heartbeat log line every 5 seconds shows scan_hit_count
 */

#include "sensesp/net/ble/ble_signalk_gateway.h"
#include "sensesp/net/ble/esp_hosted_bluedroid_ble.h"
#include "sensesp/net/ethernet_provisioner.h"
#include "sensesp_app_builder.h"

using namespace sensesp;

// Kept alive for the lifetime of the app. The gateway keeps a
// shared_ptr to both, so these could also be function-local — they
// are file-static only so the heartbeat lambda below can touch their
// counters for logging.
static std::shared_ptr<EspHostedBluedroidBLE> g_ble;
static std::shared_ptr<BLESignalKGateway> g_gateway;

void setup() {
  SetupLogging(ESP_LOG_INFO);

  SensESPAppBuilder builder;
  auto app = builder.set_hostname(GATEWAY_HOSTNAME)
                 ->set_ethernet(EthernetConfig::waveshare_esp32p4_poe())
                 ->disable_wifi()
                 ->enable_ota("bluedroid-poc-ota")
                 ->set_sk_server("192.168.0.148", 3000)
                 ->get_app();

  g_ble = std::make_shared<EspHostedBluedroidBLE>();

  g_gateway =
      std::make_shared<BLESignalKGateway>(g_ble, app->get_ws_client());
  g_gateway->start();

  event_loop()->onRepeat(5000, []() {
    ESP_LOGI(
        "POC",
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
