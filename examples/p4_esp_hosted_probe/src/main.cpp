/**
 * @file main.cpp
 * @brief ESP-Hosted-MCU smoke test on Waveshare ESP32-P4-WIFI6-POE-ETH.
 *
 * First step of M3. Uses Arduino-ESP32 3.3.7's built-in
 * `hostedInitBLE()` wrapper (in `esp32-hal-hosted.c`) to bring up the
 * ESP32-C6 companion chip via SDIO, run `esp_hosted_connect_to_slave()`,
 * and initialise the Bluetooth controller.
 *
 * The Arduino core's wrapper handles:
 *   - SDIO pin configuration from the esp32p4 variant's
 *     BOARD_SDIO_ESP_HOSTED_* macros (CLK=18, CMD=19, D0..D3=14..17, RESET=54)
 *   - esp_hosted_sdio_set_config + esp_hosted_init + connect_to_slave
 *   - esp_hosted_bt_controller_init + enable (once slave >= v2.6.0)
 *   - Reading slave firmware version
 *   - Optional C6 slave OTA update check
 *
 * All we have to do is flip `CONFIG_ESP_HOSTED_ENABLE_BT_NIMBLE=y` in
 * sdkconfig.defaults (which this project does) and call `hostedInitBLE()`.
 *
 * Success criteria:
 *   1. hostedInitBLE() returns true
 *   2. Slave firmware version is reported (non-zero)
 *   3. hostedIsInitialized() and hostedIsBLEActive() return true
 *
 * Failure modes we expect to diagnose:
 *   - SDIO bus comes up but the C6 doesn't respond → slave firmware absent
 *     or incompatible → need to flash esp_hosted_mcu slave onto the C6
 *   - SDIO config fails outright → pin wiring doesn't match board variant
 *     defaults → fix the variant or use hostedSetPins() before Init
 *   - Slave version < 2.6.0 → BT controller init path skipped → need to
 *     update the C6 firmware
 *
 * Deliberately minimal: no SensESP, no networking, no BLE scan. Just
 * prove the C6 link works before we build the gateway on top of it.
 */

#include <Arduino.h>
#include <esp_chip_info.h>
#include <esp_heap_caps.h>

#include "esp32-hal-hosted.h"

static void print_header() {
  esp_chip_info_t info;
  esp_chip_info(&info);
  Serial.printf("[hosted] chip model=%d rev=%d cores=%d\n",
                (int)info.model, (int)info.revision, (int)info.cores);
  Serial.printf("[hosted] free_heap=%u free_psram=%u\n",
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

static void report_pins() {
  int8_t clk, cmd, d0, d1, d2, d3, rst;
  hostedGetPins(&clk, &cmd, &d0, &d1, &d2, &d3, &rst);
  Serial.printf("[hosted] SDIO pins: clk=%d cmd=%d d0=%d d1=%d d2=%d d3=%d reset=%d\n",
                clk, cmd, d0, d1, d2, d3, rst);
}

static void report_versions() {
  uint32_t major = 0, minor = 0, patch = 0;
  hostedGetHostVersion(&major, &minor, &patch);
  Serial.printf("[hosted] host esp_hosted version: %u.%u.%u\n",
                (unsigned)major, (unsigned)minor, (unsigned)patch);
  hostedGetSlaveVersion(&major, &minor, &patch);
  Serial.printf("[hosted] slave firmware version: %u.%u.%u%s\n",
                (unsigned)major, (unsigned)minor, (unsigned)patch,
                (major == 0 && minor == 0 && patch == 0)
                    ? " (unknown — slave may not have responded)" : "");
}

static void probe_hosted_ble() {
  Serial.println("[hosted] calling hostedInitBLE()...");
  report_pins();

  bool ok = hostedInitBLE();
  if (!ok) {
    Serial.println("[hosted] FAIL: hostedInitBLE() returned false");
    Serial.println(
        "[hosted] Possible causes: C6 slave firmware absent or too old "
        "(need >= 2.6.0 for BT), SDIO bus pin mismatch, or the C6 is "
        "held in reset.");
    Serial.println(
        "[hosted] Next step: flash the esp_hosted_mcu slave firmware to "
        "the ESP32-C6 via the P4's USB-C pass-through download mode "
        "(C6_IO9 low at power-on + P4 in download mode, then flash via "
        "C6_U0RXD / C6_U0TXD).");
    report_versions();
    return;
  }

  Serial.println("[hosted] hostedInitBLE() OK");
  report_versions();

  Serial.printf("[hosted] hostedIsInitialized=%d hostedIsBLEActive=%d\n",
                (int)hostedIsInitialized(), (int)hostedIsBLEActive());

  Serial.println("[hosted] PASS: C6 + BT controller are up — proceed with BLE gateway");
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // let the host re-enumerate USB-CDC on the CH340 bridge

  Serial.println("[hosted] boot");
  print_header();
  probe_hosted_ble();
}

void loop() {
  static uint32_t tick = 0;
  Serial.printf("[hosted] heartbeat %u init=%d ble=%d\n",
                (unsigned)tick++, (int)hostedIsInitialized(),
                (int)hostedIsBLEActive());
  delay(5000);
}
