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

  // Enable oscillator / PHY power via GPIO17
  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH);
  delay(300);

  WiFi.mode(WIFI_OFF);
  Network.onEvent(onEvent);

  Serial.println("Calling ETH.begin()...");
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO,
            -1, ETH_CLK_MODE);
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
