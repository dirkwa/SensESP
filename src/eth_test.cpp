// Minimal Aptinex IsolPoE Ethernet test — no SensESP framework.
//
// GPIO17 = oscillator enable (4.7k pulldown → off by default).
// ETH_CLK_MODE = ETH_CLOCK_GPIO0_IN: 50MHz clock from oscillator into GPIO0.
// GPIO0 is a boot strapping pin — briefly driven LOW during boot.
// Fix: power-cycle the oscillator AFTER boot strapping completes so LAN8720
// gets a clean clock edge, then pass power=-1 so driver doesn't touch GPIO17.

#include <ETH.h>
#include <WiFi.h>
#include "driver/gpio.h"

#define OSC_EN_PIN 17

static bool eth_connected = false;

void onEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH: Started");
      ETH.setHostname("eth-test");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH: Link up");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("ETH: Got IP: %s  MAC: %s\n",
                    ETH.localIP().toString().c_str(),
                    ETH.macAddress().c_str());
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
      Serial.println("ETH: Lost IP");
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH: Link down");
      eth_connected = false;
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nAptinex IsolPoE ETH test starting...");

  // Power-cycle the 50MHz oscillator now that boot strapping is done.
  // GPIO0 was held LOW during boot; cycling the oscillator gives LAN8720
  // a clean clock after GPIO0 is released as a normal input.
  gpio_set_direction((gpio_num_t)OSC_EN_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)OSC_EN_PIN, 0);
  delay(10);
  gpio_set_level((gpio_num_t)OSC_EN_PIN, 1);
  delay(50);  // oscillator start-up time
  Serial.println("ETH: oscillator power-cycled, clock should be clean");

  WiFi.mode(WIFI_OFF);
  Network.onEvent(onEvent);
  // Pass power=-1: we manage GPIO17 ourselves above, driver must not touch it.
  ETH.begin(ETH_PHY_ADDR, -1, ETH_PHY_MDC, ETH_PHY_MDIO,
            ETH_PHY_TYPE, ETH_CLK_MODE);
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last > 5000) {
    last = millis();
    Serial.printf("uptime=%lus  linked=%d  hasIP=%d  ip=%s\n",
                  millis() / 1000,
                  (int)ETH.linkUp(),
                  (int)ETH.hasIP(),
                  ETH.localIP().toString().c_str());
  }
}
