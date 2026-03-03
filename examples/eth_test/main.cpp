/**
 * @file main.cpp
 * @brief Minimal Ethernet test for Aptinex IsolPoE ESP32
 *
 * No BLE, no SensESP — just raw ETH + static IP after link-up.
 */

#include <Arduino.h>
#include <ETH.h>
#include <HTTPClient.h>

// Aptinex IsolPoE pin configuration
#define ETH_PHY_TYPE   ETH_PHY_LAN8720
#define ETH_PHY_ADDR   1
#define ETH_PHY_MDC    23
#define ETH_PHY_MDIO   18
#define ETH_PHY_POWER  17
#define ETH_CLK_MODE   ETH_CLOCK_GPIO0_IN

// Static IP config
static const IPAddress local_ip(192, 168, 0, 120);
static const IPAddress gateway_ip(192, 168, 0, 1);
static const IPAddress netmask(255, 255, 255, 0);
static const IPAddress dns_ip(192, 168, 0, 1);

// Target to test connectivity
static const char* TEST_URL = "http://192.168.0.122:4000/signalk";

static volatile bool link_up = false;
static volatile bool got_ip = false;

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
      link_up = true;
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("ETH: Got IP %s\n", ETH.localIP().toString().c_str());
      got_ip = true;
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
      Serial.println("ETH: Lost IP");
      got_ip = false;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH: Link down");
      link_up = false;
      got_ip = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH: Stopped");
      link_up = false;
      got_ip = false;
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== Aptinex Ethernet Test (static IP) ===");

  Network.onEvent(onEthEvent);

  // Manually power on PHY
  Serial.printf("Powering PHY via GPIO%d\n", ETH_PHY_POWER);
  pinMode(ETH_PHY_POWER, OUTPUT);
  digitalWrite(ETH_PHY_POWER, HIGH);
  delay(500);

  // Start Ethernet (DHCP by default, we'll override after link-up)
  Serial.println("Starting ETH.begin()...");
  bool ok = ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC,
                       ETH_PHY_MDIO, -1, ETH_CLK_MODE);
  Serial.printf("ETH.begin() returned: %s\n", ok ? "true" : "false");

  // Wait for link
  Serial.println("Waiting for link...");
  for (int i = 0; i < 100 && !link_up; i++) {
    delay(100);
  }

  if (!link_up) {
    Serial.println("ERROR: No link after 10s");
    return;
  }

  // Now assign static IP (after link is up and driver is running)
  Serial.println("Assigning static IP 192.168.0.120...");
  ETH.config(local_ip, gateway_ip, netmask, dns_ip);

  // Wait briefly for GOT_IP event
  for (int i = 0; i < 30 && !got_ip; i++) {
    delay(100);
  }

  Serial.printf("hasIP=%d IP=%s\n", (int)ETH.hasIP(),
                ETH.localIP().toString().c_str());
}

void loop() {
  static unsigned long last_print = 0;
  static int attempt = 0;

  if (millis() - last_print > 5000) {
    last_print = millis();
    attempt++;

    Serial.printf("\n[%lu] linkUp=%d hasIP=%d IP=%s MAC=%s\n",
                  millis() / 1000, (int)link_up, (int)got_ip,
                  ETH.localIP().toString().c_str(),
                  ETH.macAddress().c_str());

    // Try an HTTP GET to test connectivity
    if (got_ip && attempt > 1) {
      Serial.printf("Testing HTTP GET %s ...\n", TEST_URL);
      HTTPClient http;
      http.begin(TEST_URL);
      http.setTimeout(3000);
      int code = http.GET();
      if (code > 0) {
        Serial.printf("HTTP %d — connectivity works!\n", code);
      } else {
        Serial.printf("HTTP error: %s\n", http.errorToString(code).c_str());
      }
      http.end();
    }
  }
}
