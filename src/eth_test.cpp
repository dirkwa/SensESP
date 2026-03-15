// Minimal Aptinex IsolPoE Ethernet test — no SensESP framework.
//
// GPIO17 = LAN8720 nRST / oscillator enable (4.7k pulldown → LOW by default).
//   LOW  → PHY in reset, oscillator off
//   HIGH → PHY running, oscillator on
//
// IDF v5 uses the power pin as PHY nRST: pulses it LOW then HIGH.
// That is exactly what GPIO17 needs — pass power=17 to ETH.begin() and let
// IDF handle the reset sequence. The oscillator comes up together with the PHY.
//
// ETH_CLK_MODE = ETH_CLOCK_GPIO0_IN: 50MHz clock from oscillator into GPIO0.

#include <ETH.h>
#include <WiFi.h>

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

  WiFi.mode(WIFI_OFF);
  Network.onEvent(onEvent);
  // power=17: IDF v5 pulses GPIO17 LOW (reset) then HIGH (run), which also
  // enables the oscillator before the EMAC clock is needed.
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO,
            17, ETH_CLK_MODE);
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
