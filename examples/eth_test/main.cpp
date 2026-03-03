/**
 * @file main.cpp
 * @brief Aptinex IsolPoE Ethernet test — official approach
 *
 * Uses defines-before-include pattern per Espressif ETH_LAN8720 example
 * and Aptinex official sample. Lets the driver handle PHY power pin.
 */

#include <Arduino.h>

// Aptinex IsolPoE pin config — MUST be before #include <ETH.h>
#define ETH_PHY_TYPE    ETH_PHY_LAN8720
#define ETH_PHY_ADDR    1
#define ETH_PHY_MDC     23
#define ETH_PHY_MDIO    18
#define ETH_PHY_POWER   17
#define ETH_CLK_MODE    ETH_CLOCK_GPIO0_IN

#include <ETH.h>

static bool eth_connected = false;

void onEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      ETH.setHostname("aptinex-test");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("ETH Got IP: ");
      Serial.println(ETH.localIP());
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
      Serial.println("ETH Lost IP");
      eth_connected = false;
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
  delay(2000);
  Serial.println("\n\n=== Aptinex Official Approach ===");
  Serial.printf("PHY_TYPE=%d ADDR=%d MDC=%d MDIO=%d POWER=%d CLK=%d\n",
                ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO,
                ETH_PHY_POWER, ETH_CLK_MODE);

  Network.onEvent(onEvent);
  ETH.begin();

  Serial.println("ETH.begin() done, waiting...");
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last > 5000) {
    last = millis();
    Serial.printf("[%lus] connected=%d IP=%s MAC=%s\n",
                  millis() / 1000, (int)eth_connected,
                  ETH.localIP().toString().c_str(),
                  ETH.macAddress().c_str());
  }
}
