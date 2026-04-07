/**
 * @file main.cpp
 * @brief One-shot updater for the ESP32-C6 ESP-Hosted-MCU slave firmware
 *        on the Waveshare ESP32-P4-WIFI6-POE-ETH board.
 *
 * Why this exists: the Waveshare board ships from the factory with a
 * C6 image that does not respond to esp_hosted RPC version queries
 * (slave_version reads as 0.0.0). The host-side esp_hosted component
 * we get from Arduino-ESP32 3.3.7 is v2.11.6 and expects a matching
 * slave. Without flashing, esp_hosted_bt_controller_init() can't be
 * called and BLE-via-C6 doesn't work.
 *
 * The Arduino-ESP32 core has a built-in slave OTA API
 * (hostedHasUpdate / hostedBeginUpdate / hostedWriteUpdate /
 * hostedEndUpdate / hostedActivateUpdate) that flashes a new C6
 * binary over the SDIO bus from the P4 side. The matching binary is
 * published by Espressif at
 * https://espressif.github.io/arduino-esp32/hosted/<target>-v<x>.<y>.<z>.bin
 * — the URL is computed by hostedGetUpdateURL() from the host's
 * version + the build's CONFIG_ESP_HOSTED_IDF_SLAVE_TARGET (esp32c6
 * on this board).
 *
 * This sketch:
 *   1. Brings up native ethernet via SensESP's EthernetProvisioner
 *      (Waveshare P4 PoE config, same as the M2 example)
 *   2. Waits for NetworkState = kNetworkConnected (DHCP lease obtained)
 *   3. Calls hostedInitBLE() — initialises the SDIO link to the C6
 *   4. Calls hostedHasUpdate() — queries slave version, returns true
 *      because host (2.11.6) > slave (0.0.0 or whatever's there)
 *   5. Downloads the matching slave binary over HTTP from the
 *      Espressif Arduino-ESP32 hosted CDN
 *   6. Streams the binary in chunks through hostedWriteUpdate()
 *   7. Calls hostedEndUpdate() + hostedActivateUpdate()
 *   8. Reboots the P4 (which power-cycles the C6 too via the
 *      esp_hosted reset GPIO)
 *
 * After running this once successfully, the C6 will respond to
 * esp_hosted RPC and the next firmware (the BLE gateway) can use
 * hostedInitBLE() + esp_hosted_bt_controller_init() to bring up BLE.
 *
 * One-shot intent: this is not the gateway. Flash it, run it, watch
 * the serial log for "DONE", then flash the BLE gateway firmware on
 * top.
 */

#include <Arduino.h>
#include <HTTPClient.h>
#include <esp_chip_info.h>
#include <esp_heap_caps.h>

#include "esp32-hal-hosted.h"
#include "sensesp/net/ethernet_provisioner.h"
#include "sensesp_app_builder.h"

using namespace sensesp;

static std::shared_ptr<SensESPApp> app;
static bool ota_attempted = false;

static void print_chip_info() {
  esp_chip_info_t info;
  esp_chip_info(&info);
  Serial.printf("[c6_ota] chip model=%d rev=%d cores=%d\n",
                (int)info.model, (int)info.revision, (int)info.cores);
  Serial.printf("[c6_ota] free_heap=%u free_psram=%u\n",
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

static void print_versions(const char* tag) {
  uint32_t hmaj = 0, hmin = 0, hpat = 0;
  uint32_t smaj = 0, smin = 0, spat = 0;
  hostedGetHostVersion(&hmaj, &hmin, &hpat);
  hostedGetSlaveVersion(&smaj, &smin, &spat);
  Serial.printf("[c6_ota] %s host=%u.%u.%u slave=%u.%u.%u\n", tag,
                (unsigned)hmaj, (unsigned)hmin, (unsigned)hpat,
                (unsigned)smaj, (unsigned)smin, (unsigned)spat);
}

static bool perform_ota_update() {
  Serial.println("[c6_ota] === starting C6 slave OTA update ===");

  if (!hostedInitBLE()) {
    Serial.println("[c6_ota] hostedInitBLE() failed — cannot proceed");
    return false;
  }
  print_versions("post-init");

  // hostedHasUpdate() refreshes the slave version internally and
  // logs the URL it would use for the update. It returns true when
  // host > slave, which is exactly the case we want to fix.
  if (!hostedHasUpdate()) {
    Serial.println(
        "[c6_ota] hostedHasUpdate() returned false. Either the slave is "
        "already at the host's version or the slave is newer than the "
        "host. Nothing to do.");
    return true;  // not an error
  }

  // hostedGetUpdateURL() computes the same URL hostedHasUpdate logged.
  // Defensively null-check the return value: the Arduino-ESP32
  // implementation always returns a non-null buffer today, but the API
  // contract is loose enough that future versions could legitimately
  // return null when the host version is unknown.
  const char* url = hostedGetUpdateURL();
  if (url == nullptr || url[0] == '\0') {
    Serial.println("[c6_ota] hostedGetUpdateURL() returned no URL — cannot proceed");
    return false;
  }
  Serial.printf("[c6_ota] update URL: %s\n", url);

  // Download the slave binary in streaming mode and feed it to the
  // hosted updater chunk by chunk.
  HTTPClient http;
  http.setReuse(false);
  http.setTimeout(30000);
  if (!http.begin(String(url))) {
    Serial.println("[c6_ota] http.begin() failed");
    return false;
  }
  // Allow Espressif's GitHub Pages to redirect; HTTPClient handles
  // 301/302 transparently when followRedirects is set.
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    Serial.printf("[c6_ota] HTTP GET failed: %d\n", code);
    http.end();
    return false;
  }

  int total = http.getSize();
  Serial.printf("[c6_ota] downloading %d bytes...\n", total);

  if (!hostedBeginUpdate()) {
    Serial.println("[c6_ota] hostedBeginUpdate() failed");
    http.end();
    return false;
  }

  WiFiClient* stream = http.getStreamPtr();
  if (!stream) {
    Serial.println("[c6_ota] http.getStreamPtr() returned null — connection lost");
    hostedEndUpdate();  // best-effort cleanup of the SDIO OTA state
    http.end();
    return false;
  }

  uint8_t buf[1024];
  int written = 0;
  unsigned long t0 = millis();
  // Overall watchdog: a misbehaving server could hold the connection
  // open without ever sending data, which would keep http.connected()
  // true and leave us spinning in delay(10) forever. Cap the entire
  // download at 2 minutes — the real image is only ~1.2 MB so even
  // on a slow link this is plenty.
  static constexpr unsigned long kDownloadTimeoutMs = 120000;

  while (http.connected() && (total < 0 || written < total)) {
    if (millis() - t0 > kDownloadTimeoutMs) {
      Serial.printf("[c6_ota] download stalled — timeout after %lu ms "
                    "(%d/%d bytes)\n",
                    millis() - t0, written, total);
      hostedEndUpdate();
      http.end();
      return false;
    }
    size_t avail = stream->available();
    if (avail == 0) {
      delay(10);
      continue;
    }
    int n = stream->readBytes(buf, std::min(avail, sizeof(buf)));
    if (n <= 0) {
      delay(10);
      continue;
    }
    if (!hostedWriteUpdate(buf, n)) {
      Serial.println("[c6_ota] hostedWriteUpdate() failed");
      hostedEndUpdate();  // best-effort cleanup of the SDIO OTA state
      http.end();
      return false;
    }
    written += n;
    if ((written % 16384) == 0 || (total > 0 && written == total)) {
      if (total > 0) {
        Serial.printf("[c6_ota]   %d / %d bytes\n", written, total);
      } else {
        Serial.printf("[c6_ota]   %d bytes (chunked)\n", written);
      }
    }
  }
  http.end();

  unsigned long ms = millis() - t0;
  Serial.printf("[c6_ota] download+stream complete: %d bytes in %lu ms\n",
                written, ms);

  if (!hostedEndUpdate()) {
    Serial.println("[c6_ota] hostedEndUpdate() failed");
    return false;
  }

  Serial.println("[c6_ota] activating new slave firmware...");
  if (!hostedActivateUpdate()) {
    Serial.println(
        "[c6_ota] hostedActivateUpdate() failed (may be expected on a "
        "very old slave; firmware should still be flashed)");
    // Continue to reboot — the activate may legitimately fail when the
    // slave was previously running an incompatible image.
  }

  Serial.println("[c6_ota] === DONE — rebooting in 3 seconds ===");
  return true;
}

static void on_network_connected(NetworkState state) {
  if (state != kNetworkConnected || ota_attempted) return;
  ota_attempted = true;

  // Defer the OTA work onto the next event-loop tick so we don't
  // block any in-progress event handlers.
  event_loop()->onDelay(500, []() {
    auto provisioner = app->get_network_provisioner();
    Serial.printf("[c6_ota] network up: %s (gw %s, mac %s)\n",
                  provisioner->local_ip().toString().c_str(),
                  provisioner->gateway_ip().toString().c_str(),
                  provisioner->mac_address().c_str());

    bool ok = perform_ota_update();
    Serial.printf("[c6_ota] OTA result: %s\n", ok ? "OK" : "FAIL");
    delay(3000);
    ESP.restart();
  });
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("[c6_ota] boot");
  print_chip_info();
  print_versions("pre-init");

  // Build a SensESP app whose only job is to bring up ethernet so the
  // OTA fetch over HTTP works. We don't need a Signal K connection at
  // all for this utility, but the SensESP setup() flow expects an SK
  // server reference, so we let it use a dummy address — it will fail
  // to connect but that doesn't block the network state producer.
  SensESPAppBuilder builder;
  app = builder.set_hostname("p4-c6-ota")
            ->set_ethernet(EthernetConfig::waveshare_esp32p4_poe())
            ->get_app();

  // Subscribe to network state transitions so we kick off OTA exactly
  // once when DHCP completes.
  app->get_network_state_producer()->connect_to(
      new LambdaConsumer<NetworkState>(on_network_connected));

  Serial.println("[c6_ota] waiting for ethernet link + DHCP...");
}

void loop() { event_loop()->tick(); }
