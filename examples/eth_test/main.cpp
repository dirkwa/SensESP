/*
 * Arduino V3 Ethernet Test for Aptinex IsolPoE - ESP32 ProDev Kit
 *
 * GPIO17 on this board is the 50MHz crystal oscillator enable pin,
 * NOT a PHY reset pin. The ESP-IDF driver's reset sequence (LOW pulse
 * on the power pin) would disable the oscillator, removing the RMII
 * reference clock that LAN8720 needs during reset. So we set
 * ETH_PHY_POWER=-1 and manage the oscillator manually.
 */

#define ETH_PHY_TYPE    ETH_PHY_LAN8720
#define ETH_PHY_ADDR    1
#define ETH_PHY_MDC     23
#define ETH_PHY_MDIO    18
#define ETH_PHY_POWER   -1                // Don't let driver touch GPIO17
#define ETH_CLK_MODE    ETH_CLOCK_GPIO0_IN

#include <ETH.h>
#include <WiFi.h>

#define OSC_ENABLE_PIN  17  // 50MHz crystal oscillator enable

static bool eth_connected = false;

void onEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("Ethernet Started");
      ETH.setHostname("aptinex-esp32-poe");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("Ethernet Link Connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("Ethernet Got IP: ");
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
  Serial.begin(115200);
  delay(300);  // Let power rails stabilize
  Serial.println();
  Serial.println("Booting Aptinex IsolPoE ESP32...");

  // Enable the 50MHz crystal oscillator BEFORE starting Ethernet
  pinMode(OSC_ENABLE_PIN, OUTPUT);
  digitalWrite(OSC_ENABLE_PIN, HIGH);
  delay(50);  // Wait for oscillator to stabilize
  Serial.println("50MHz oscillator enabled.");

  WiFi.mode(WIFI_OFF);
  Serial.println("WiFi disabled.");

  Network.onEvent(onEvent);

  ETH.begin();
  Serial.println("Ethernet initialization started...");
}

void loop() {
  if (eth_connected) {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 60000) {
      lastPrint = millis();
      Serial.print("IP: ");
      Serial.println(ETH.localIP());
    }
  } else {
    static unsigned long lastWarn = 0;
    if (millis() - lastWarn > 10000) {
      lastWarn = millis();
      Serial.println("Waiting for Ethernet link and DHCP...");
    }
  }
  delay(1);  // yield to LWIP/system tasks without blocking long
}
