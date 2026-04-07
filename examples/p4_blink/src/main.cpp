/**
 * @file main.cpp
 * @brief ESP32-P4 smoke test on Waveshare ESP32-P4-WIFI6-POE-ETH.
 *
 * Deliberately minimal: no SensESP, no WiFi, no networking — this test
 * isolates the toolchain + board + USB-CDC boot path. The fact that the
 * SensESP library itself compiles and links for ESP32-P4 is verified
 * separately by the top-level [env:pioarduino_esp32p4] env in
 * ../../platformio.ini, which links a full SensESP binary with --undefined
 * setup/loop, proving all SensESP + Arduino-ESP32 3.3.7 code paths resolve.
 *
 * Expected output on /dev/ttyACM0 at 115200:
 *     [p4_blink] boot
 *     [p4_blink] chip model=18 rev=0 cores=2
 *     [p4_blink] free_heap=... free_psram=...
 *     [p4_blink] tick 0
 *     [p4_blink] tick 1
 *     ...
 */

#include <Arduino.h>
#include <esp_chip_info.h>
#include <esp_heap_caps.h>

void setup() {
  Serial.begin(115200);
  // USB-CDC on boot — give the host ~1s to re-enumerate before printing.
  delay(1000);

  Serial.println("[p4_blink] boot");

  esp_chip_info_t info;
  esp_chip_info(&info);
  Serial.printf("[p4_blink] chip model=%d rev=%d cores=%d\n",
                (int)info.model, (int)info.revision, (int)info.cores);

  Serial.printf("[p4_blink] free_heap=%u free_psram=%u\n",
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

void loop() {
  static uint32_t tick = 0;
  Serial.printf("[p4_blink] tick %u\n", (unsigned)tick++);
  delay(1000);
}
