/**
 * @file main.cpp
 * @brief Minimal Ethernet test for Aptinex IsolPoE ESP32
 *
 * Strips away BLE, SensESP, Signal K — just raw ETH with static IP.
 * Tests whether the Ethernet link stays up and IP connectivity works.
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

// Static IP
static const IPAddress local_ip(192, 168, 0, 120);
static const IPAddress gateway(192, 168, 0, 1);
static const IPAddress netmask(255, 255, 255, 0);
static const IPAddress dns_server(192, 168, 0, 1);

static bool eth_connected = false;

void onEthEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH: Started");
      ETH.setHostname("eth-test");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH: Link up");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("ETH: Got IP %s (link %s, %d Mbps)\n",
                     ETH.localIP().toString().c_str(),
                     ETH.fullDuplex() ? "full duplex" : "half duplex",
                     ETH.linkSpeed());
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
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH: Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== Aptinex Ethernet Test ===");

  // Register event handler BEFORE ETH.begin
  Network.onEvent(onEthEvent);

  // Manually power on the PHY — GPIO17 is a power enable, not a reset pin.
  // Passing it to ETH.begin() would cause a reset-style toggle that kills power.
  Serial.printf("Powering PHY via GPIO%d\n", ETH_PHY_POWER);
  pinMode(ETH_PHY_POWER, OUTPUT);
  digitalWrite(ETH_PHY_POWER, HIGH);
  delay(500);

  // Configure static IP before starting the driver
  ETH.config(local_ip, gateway, netmask, dns_server);

  // Start Ethernet with power=-1 (we handle power manually)
  Serial.println("Starting ETH.begin()...");
  bool ok = ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC,
                       ETH_PHY_MDIO, -1, ETH_CLK_MODE);

  Serial.printf("ETH.begin() returned: %s\n", ok ? "true" : "false");
  Serial.printf("ETH.linkUp() = %s\n", ETH.linkUp() ? "true" : "false");
}

void loop() {
  static unsigned long last_print = 0;
  if (millis() - last_print > 5000) {
    last_print = millis();
    Serial.printf("[%lu] linkUp=%d hasIP=%d IP=%s\n",
                  millis() / 1000,
                  ETH.linkUp(),
                  (int)ETH.hasIP(),
                  ETH.localIP().toString().c_str());
  }
}
