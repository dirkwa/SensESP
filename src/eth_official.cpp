// Official ETH_LAN8720 example from espressif/arduino-esp32, adapted for Aptinex.
// Source: https://github.com/espressif/arduino-esp32/blob/master/libraries/Ethernet/examples/ETH_LAN8720/ETH_LAN8720.ino
//
// Only change from official: ETH_PHY_ADDR=1, ETH_PHY_POWER=17 for Aptinex board.

// Important to be defined BEFORE including ETH.h for ETH.begin() to work.
#define ETH_PHY_TYPE  ETH_PHY_LAN8720
#define ETH_PHY_ADDR  1
#define ETH_PHY_MDC   23
#define ETH_PHY_MDIO  18
#define ETH_PHY_POWER -1
#define ETH_CLK_MODE  ETH_CLOCK_GPIO0_IN

#include <ETH.h>

static bool eth_connected = false;

void onEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      ETH.setHostname("esp32-ethernet");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED: Serial.println("ETH Connected"); break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.println("ETH Got IP");
      Serial.println(ETH);
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
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  // GPIO17 = oscillator enable on Aptinex board. Must be HIGH before ETH.begin()
  // so the 50MHz clock is present during EMAC init. Do NOT pass power=17 to
  // ETH.begin() — the IDF would pulse it LOW (PHY reset), killing the clock.
  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH);
  delay(500);  // wait for oscillator + PHY power-on reset
  Serial.println("GPIO17 HIGH, oscillator on");
  Network.onEvent(onEvent);
  ETH.begin();
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
