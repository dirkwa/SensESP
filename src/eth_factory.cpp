// Exact replica of the Aptinex factory example code.
// Source: https://www.hackster.io/515498/getting-started-with-aptinex-isolpoe-esp32-prodev-kit-b6945a
//
// Uses the pioarduino (v3) API but with the exact same pin definitions
// and initialization sequence as the factory example.

#include <ETH.h>

#define ETH_CLK_MODE_   ETH_CLOCK_GPIO0_IN
#define ETH_POWER_PIN   17
#define ETH_TYPE        ETH_PHY_LAN8720
#define ETH_ADDR        1
#define ETH_MDC_PIN     23
#define ETH_MDIO_PIN    18

static bool eth_connected = false;

void WiFiEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      ETH.setHostname("aptinex-test");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("ETH MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      Serial.print(", ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nAptinex factory-style ETH test");

  WiFi.onEvent(WiFiEvent);

  // This is the exact call pattern from the factory example,
  // adapted for the pioarduino v3 API parameter order:
  // v2: ETH.begin(addr, power, mdc, mdio, type, clk_mode)
  // v3: ETH.begin(type, addr, mdc, mdio, power, clk_mode)
  ETH.begin(ETH_TYPE, ETH_ADDR, ETH_MDC_PIN, ETH_MDIO_PIN,
            ETH_POWER_PIN, ETH_CLK_MODE_);

  Serial.println("ETH.begin() done");
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last > 5000) {
    last = millis();
    Serial.printf("uptime=%lus  linked=%d  ip=%s\n",
                  millis() / 1000,
                  (int)ETH.linkUp(),
                  ETH.localIP().toString().c_str());
  }
}
