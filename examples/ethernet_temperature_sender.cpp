/**
 * @file ethernet_temperature_sender.cpp
 * @brief Example: SensESP over Ethernet (PoE ESP32 boards)
 *
 * This example demonstrates SensESP running over a wired Ethernet connection
 * instead of WiFi. It uses the same sensor framework and Signal K integration
 * as WiFi sketches — the only difference is the network configuration.
 *
 * Wired Ethernet is ideal for marine installations:
 * - More reliable than WiFi in electrically noisy engine rooms
 * - PoE provides power and data over a single cable
 * - No SSID/password management needed — just plug in and go
 * - BLE coexistence: with WiFi disabled, the ESP32's shared RF module is
 *   exclusively available for BLE (better range, no coexistence issues)
 *
 * Tested boards:
 * - Olimex ESP32-POE-ISO (LAN8710A, PoE)
 * - Olimex ESP32-GATEWAY (LAN8710A, no PoE)
 * - wESP32 (LAN8720A, PoE)
 * - WT32-ETH01 (LAN8720A, no PoE)
 *
 * To use a different board, provide the correct EthernetConfig for your
 * PHY chip and pin mapping, or use the designated presets.
 */

#include "sensesp/sensors/analog_input.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/linear.h"
#include "sensesp_app_builder.h"

using namespace sensesp;

void setup() {
  SetupLogging();

  SensESPAppBuilder builder;
  auto sensesp_app = builder
      .set_hostname("engine-room-temp")
      // Ethernet configuration for Olimex ESP32-POE-ISO
      // Replace with the preset or custom config for your board.
      .set_ethernet(EthernetConfig::olimex_esp32_poe_iso())
      // Optional: WiFi AP for configuration via phone/laptop
      // .set_wifi_access_point("SensESP", "configure")
      .enable_ota("ota_password")
      .get_app();

  // Read analog temperature sensor on GPIO36
  auto* analog_input = new AnalogInput(36, 1000);

  // Convert ADC reading to temperature (calibrate for your sensor)
  const float scale = 0.1;   // degrees per ADC unit
  const float offset = -50.0;

  analog_input
      ->connect_to(new Linear(scale, offset))
      ->connect_to(new SKOutputFloat(
          "propulsion.mainEngine.exhaustTemperature",
          "/sensors/engine_room/exhaust_temp"));
}

void loop() { event_loop()->tick(); }
