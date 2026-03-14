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
#include "esp_private/esp_gpio_reserve.h"   // esp_gpio_revoke()
#include "esp_private/gpio.h"               // gpio_iomux_input()
#include "soc/io_mux_reg.h"                 // FUNC_GPIO0_EMAC_TX_CLK, IO_MUX_GPIO0_REG

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

  // IDF v5 marks GPIO0 as reserved (strapping pin).
  // emac_esp_iomux_rmii_clk_input() calls esp_gpio_revoke() to unreserve it,
  // but if that fails (e.g. already reserved by boot ROM tracking), it skips
  // the gpio_iomux_input() call entirely — EMAC gets no clock → zero TX.
  // Pre-revoke GPIO0 here so the EMAC driver finds it unreserved.
  esp_gpio_revoke(BIT64(GPIO_NUM_0));
  Serial.println("ETH: GPIO0 reservation revoked");

  // Let the ETH driver manage GPIO17 (oscillator enable) via the power parameter.
  // Factory firmware (IDF v4.4) passes power=17 to ETH.begin and it works.
  WiFi.mode(WIFI_OFF);
  Network.onEvent(onEvent);
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO,
            OSC_EN_PIN, ETH_CLK_MODE);
  Serial.println("ETH: ETH.begin() called with power=17");

  // Force GPIO0 IOMUX to EMAC clock input function (IDF v5 reservation bug workaround).
  // emac_esp_iomux_rmii_clk_input() may skip gpio_iomux_input() if its internal
  // esp_gpio_revoke() fails — leaving EMAC with no clock → zero TX.
  // Calling this after ETH.begin() overrides whatever the driver left in place.
  gpio_iomux_input(GPIO_NUM_0, FUNC_GPIO0_EMAC_TX_CLK, 0);
  Serial.println("ETH: GPIO0 IOMUX forced to EMAC_TX_CLK (func=5)");
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last > 5000) {
    last = millis();
    Serial.printf("uptime=%lus  linked=%d  hasIP=%d  ip=%s  MAC=%s\n",
                  millis() / 1000,
                  (int)ETH.linkUp(),
                  (int)ETH.hasIP(),
                  ETH.localIP().toString().c_str(),
                  ETH.macAddress().c_str());
  }
}
