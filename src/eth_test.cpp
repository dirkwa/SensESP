// Minimal Aptinex IsolPoE Ethernet test — no SensESP framework.
//
// GPIO17 = LAN8720 nRST / oscillator enable (4.7k pulldown → LOW by default).
//   LOW  → PHY in reset, oscillator off
//   HIGH → PHY running, oscillator on
//
// The LAN8720 requires REFCLK (50MHz) to be present DURING its reset sequence
// for RMII to initialise correctly. GPIO17 controls both the oscillator and the
// PHY reset, so we cannot use IDF's built-in reset pulse (which drives GPIO17
// LOW, killing the clock). Instead:
//   1. Enable oscillator (GPIO17 HIGH) and let it stabilise.
//   2. Manually pulse nRST by briefly driving GPIO17 LOW then HIGH again,
//      keeping the pulse short enough that the oscillator recovers before the
//      PHY samples REFCLK.
//   3. Call ETH.begin() with power=-1 so IDF does not touch GPIO17 again.
//
// ETH_CLK_MODE = ETH_CLOCK_GPIO0_IN: 50MHz clock from oscillator into GPIO0.

#include <ETH.h>
#include <WiFi.h>

#define PHY_RST_PIN 17

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

  // Step 1: enable oscillator and let it stabilise.
  pinMode(PHY_RST_PIN, OUTPUT);
  digitalWrite(PHY_RST_PIN, HIGH);
  delay(50);
  Serial.println("ETH: oscillator on");

  // Step 2: brief reset pulse — LOW for 10ms then HIGH.
  // Oscillator recovers in ~1ms; LAN8720 nRST min pulse is 100us.
  // With REFCLK present during de-assertion the PHY initialises RMII correctly.
  digitalWrite(PHY_RST_PIN, LOW);
  delay(10);
  digitalWrite(PHY_RST_PIN, HIGH);
  delay(50);  // wait for PHY to complete internal reset
  Serial.println("ETH: PHY reset done");

  WiFi.mode(WIFI_OFF);
  Network.onEvent(onEvent);
  // power=-1: IDF must not touch GPIO17 again after our manual reset.
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO,
            -1, ETH_CLK_MODE);
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
