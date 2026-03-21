// Bare minimum Ethernet test — no register manipulation, no event handler tricks.
// Just ETH.begin() and wait for DHCP.
#include <ETH.h>
#include <WiFi.h>

static bool eth_connected = false;

void onEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH: Started");
      ETH.setHostname("eth-bare-test");
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
  Serial.println("\nBare ETH test starting...");

  WiFi.mode(WIFI_OFF);
  Network.onEvent(onEvent);

  // Pass power=17 so IDF manages GPIO17 as PHY power/reset pin.
  // The factory Aptinex example uses ETH_PHY_POWER=17.
  // Do NOT manually set GPIO17 — let the IDF handle the power sequence.
  Serial.println("Calling ETH.begin(power=17)...");
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO,
            17, ETH_CLK_MODE);
  Serial.println("ETH.begin() returned");
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last > 5000) {
    last = millis();
    Serial.printf("uptime=%lus  linked=%d  ip=%s  MAC=%s\n",
                  millis() / 1000,
                  (int)ETH.linkUp(),
                  ETH.localIP().toString().c_str(),
                  ETH.macAddress().c_str());
  }
}
