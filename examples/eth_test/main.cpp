/*
 * Arduino V3 Sample Code for Aptinex IsolPoE - ESP32 ProDev Kit
 *
 * This sketch initializes the Ethernet controller (LAN8720),
 * requests an IPv4 address via DHCP, disables WiFi, and prints
 * the assigned IP address to the Serial Monitor.
 */

// IMPORTANT: Define Ethernet PHY configuration BEFORE including <ETH.h>
// These pin definitions are specific to the Aptinex IsolPoE board
#define ETH_PHY_TYPE    ETH_PHY_LAN8720  // The board uses the LAN8720 PHY
#define ETH_PHY_ADDR    1                 // PHY address for LAN8720 on this board
#define ETH_PHY_MDC     23                // MDC pin
#define ETH_PHY_MDIO    18                // MDIO pin
#define ETH_PHY_POWER   17                // Power enable pin for the PHY
#define ETH_CLK_MODE    ETH_CLOCK_GPIO0_IN // Clock configuration

// Include the core Ethernet library
#include <ETH.h>
#include <WiFi.h> // Included to explicitly turn off WiFi

// Flag to indicate when Ethernet is connected and has an IP
static bool eth_connected = false;

// Event handler function, called automatically on network events
void onEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("Ethernet Started");
      // Set a hostname for DHCP
      ETH.setHostname("aptinex-esp32-poe");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("Ethernet Link Connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      // This is the key event: DHCP has successfully assigned an IP
      Serial.print("Ethernet Got IP from DHCP: ");
      Serial.println(ETH.localIP());
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
      Serial.println("Ethernet Lost IP");
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("Ethernet Disconnected");
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("Ethernet Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  // Small delay to allow Serial to stabilize, especially on some USB connections
  delay(1000);
  Serial.println();
  Serial.println("Booting Aptinex IsolPoE ESP32...");

  // 1. Disable WiFi Radio
  // This ensures the device operates as a pure Ethernet client
  WiFi.mode(WIFI_OFF);
  Serial.println("WiFi Disabled.");

  // 2. Register the network event handler
  // This must be done before ETH.begin()
  Network.onEvent(onEvent);

  // 3. Initialize Ethernet
  // The ETH.begin() function uses the macros defined at the top.
  // It will attempt to get an IP address via DHCP automatically.
  // No static IP configuration is provided, so it acts as a DHCP client.
  ETH.begin();
  Serial.println("Ethernet initialization started. Requesting IP via DHCP...");
}

void loop() {
  // Your main code logic goes here.
  // The eth_connected flag can be used to check if the network is ready.
  if (eth_connected) {
    // For example, you could print the IP every minute
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 60000) { // Print every 60 seconds
      lastPrint = millis();
      Serial.print("Current IP Address: ");
      Serial.println(ETH.localIP());
    }
  } else {
    // Optionally handle the disconnected state
    static unsigned long lastWarn = 0;
    if (millis() - lastWarn > 10000) { // Warn every 10 seconds if not connected
      lastWarn = millis();
      Serial.println("Waiting for Ethernet link and DHCP...");
    }
  }
  delay(100);
}
