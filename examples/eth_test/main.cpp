/**
 * @file main.cpp
 * @brief Minimal Ethernet test for Aptinex IsolPoE ESP32
 *
 * No BLE, no SensESP — just raw ETH + DHCP to isolate networking.
 */

#include <Arduino.h>
#include <ETH.h>

// Aptinex IsolPoE pin configuration
#define ETH_PHY_TYPE   ETH_PHY_LAN8720
#define ETH_PHY_ADDR   1
#define ETH_PHY_MDC    23
#define ETH_PHY_MDIO   18
#define ETH_PHY_POWER  17
#define ETH_CLK_MODE   ETH_CLOCK_GPIO0_IN

void onEthEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH: Started");
      ETH.setHostname("eth-test");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.printf("ETH: Link up (%s, %d Mbps)\n",
                     ETH.fullDuplex() ? "full duplex" : "half duplex",
                     ETH.linkSpeed());
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("ETH: Got IP %s\n",
                     ETH.localIP().toString().c_str());
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
      Serial.println("ETH: Lost IP");
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH: Link down");
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH: Stopped");
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== Aptinex Ethernet Test (DHCP) ===");

  // Register event handler BEFORE ETH.begin
  Network.onEvent(onEthEvent);

  // Manually power on the PHY — GPIO17 is a power enable, not a reset pin.
  Serial.printf("Powering PHY via GPIO%d\n", ETH_PHY_POWER);
  pinMode(ETH_PHY_POWER, OUTPUT);
  digitalWrite(ETH_PHY_POWER, HIGH);
  delay(500);

  // No ETH.config() — use DHCP (default)

  // Start Ethernet with power=-1 (we handle power manually)
  Serial.println("Starting ETH.begin()...");
  bool ok = ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC,
                       ETH_PHY_MDIO, -1, ETH_CLK_MODE);

  Serial.printf("ETH.begin() returned: %s\n", ok ? "true" : "false");
}

void loop() {
  static unsigned long last_print = 0;
  if (millis() - last_print > 3000) {
    last_print = millis();
    Serial.printf("[%lu] linkUp=%d hasIP=%d IP=%s\n",
                  millis() / 1000,
                  ETH.linkUp(),
                  (int)ETH.hasIP(),
                  ETH.localIP().toString().c_str());
  }
}
