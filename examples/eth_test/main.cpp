/**
 * @file main.cpp
 * @brief Minimal Ethernet test for Aptinex IsolPoE ESP32
 *
 * Diagnoses RMII clock issue: MDIO works but data path doesn't.
 */

#include <Arduino.h>
#include <ETH.h>
#include <driver/gpio.h>
#include <esp_eth_phy_802_3.h>

// Aptinex IsolPoE pin configuration
#define ETH_PHY_TYPE   ETH_PHY_LAN8720
#define ETH_PHY_ADDR   1
#define ETH_PHY_MDC    23
#define ETH_PHY_MDIO   18
#define ETH_PHY_POWER  17
#define ETH_CLK_MODE   ETH_CLOCK_GPIO0_IN

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
    default:
      break;
  }
}

void dumpPhyRegisters() {
  Serial.println("\n--- LAN8720 PHY Registers ---");

  // Reg 0: Basic Control Register
  uint16_t bcr = ETH.phyRead(0x00);
  Serial.printf("Reg 0  BCR:   0x%04X", bcr);
  if (bcr & 0x8000) Serial.print(" [RESET]");
  if (bcr & 0x4000) Serial.print(" [LOOPBACK]");
  if (bcr & 0x2000) Serial.print(" [100Mbps]");
  if (bcr & 0x1000) Serial.print(" [AN_ENABLE]");
  if (bcr & 0x0800) Serial.print(" [POWER_DOWN]");
  if (bcr & 0x0200) Serial.print(" [RESTART_AN]");
  if (bcr & 0x0100) Serial.print(" [FULL_DUPLEX]");
  Serial.println();

  // Reg 1: Basic Status Register
  uint16_t bsr = ETH.phyRead(0x01);
  Serial.printf("Reg 1  BSR:   0x%04X", bsr);
  if (bsr & 0x0004) Serial.print(" [LINK_UP]");
  else Serial.print(" [LINK_DOWN]");
  if (bsr & 0x0020) Serial.print(" [AN_COMPLETE]");
  if (bsr & 0x0010) Serial.print(" [REMOTE_FAULT]");
  Serial.println();

  // Reg 2,3: PHY ID
  uint16_t id1 = ETH.phyRead(0x02);
  uint16_t id2 = ETH.phyRead(0x03);
  Serial.printf("Reg 2,3 PHY ID: 0x%04X:0x%04X\n", id1, id2);

  // Reg 4: Auto-negotiation advertisement
  uint16_t anar = ETH.phyRead(0x04);
  Serial.printf("Reg 4  ANAR:  0x%04X\n", anar);

  // Reg 5: Auto-negotiation link partner ability
  uint16_t anlpar = ETH.phyRead(0x05);
  Serial.printf("Reg 5  ANLPAR:0x%04X\n", anlpar);

  // Reg 17: Mode Control/Status (LAN8720-specific)
  uint16_t mcsr = ETH.phyRead(0x11);
  Serial.printf("Reg 17 MCSR:  0x%04X", mcsr);
  if (mcsr & 0x0002) Serial.print(" [ENERGYON]");
  Serial.println();

  // Reg 18: Special Modes
  uint16_t smr = ETH.phyRead(0x12);
  Serial.printf("Reg 18 SMR:   0x%04X (PHY addr=%d, mode=%d)\n",
                smr, smr & 0x1F, (smr >> 5) & 0x07);

  // Reg 26: PHY Special Control/Status Indication
  uint16_t pscsi = ETH.phyRead(0x1A);
  Serial.printf("Reg 26 PSCSI: 0x%04X\n", pscsi);

  // Reg 31: PHY Special Control/Status
  uint16_t pscsr = ETH.phyRead(0x1F);
  Serial.printf("Reg 31 PSCSR: 0x%04X", pscsr);
  if (pscsr & 0x1000) Serial.print(" [AUTODONE]");
  int speed_ind = (pscsr >> 2) & 0x07;
  const char* speeds[] = {"?","10HD","100HD","?","?","10FD","100FD","?"};
  Serial.printf(" [%s]", speeds[speed_ind]);
  Serial.println();

  Serial.println("-----------------------------\n");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== Aptinex Ethernet Diag ===");

  Network.onEvent(onEthEvent);

  // Manually power on PHY
  Serial.printf("Powering PHY via GPIO%d\n", ETH_PHY_POWER);
  pinMode(ETH_PHY_POWER, OUTPUT);
  digitalWrite(ETH_PHY_POWER, HIGH);
  delay(500);

  // Ensure GPIO0 has no internal pull-up/pull-down that could interfere
  // with the external 50 MHz clock signal
  Serial.println("Clearing GPIO0 pull-up/pull-down...");
  gpio_pullup_dis(GPIO_NUM_0);
  gpio_pulldown_dis(GPIO_NUM_0);

  // Start Ethernet
  Serial.println("Starting ETH.begin()...");
  bool ok = ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC,
                       ETH_PHY_MDIO, -1, ETH_CLK_MODE);
  Serial.printf("ETH.begin() returned: %s\n", ok ? "true" : "false");

  // Wait a moment for PHY to settle
  delay(2000);

  // Dump PHY registers
  dumpPhyRegisters();

  // Try static IP
  if (link_up) {
    Serial.println("Assigning static IP 192.168.0.120...");
    ETH.config(IPAddress(192, 168, 0, 120),
               IPAddress(192, 168, 0, 1),
               IPAddress(255, 255, 255, 0),
               IPAddress(192, 168, 0, 1));

    for (int i = 0; i < 30 && !got_ip; i++) delay(100);
    Serial.printf("hasIP=%d IP=%s\n", (int)ETH.hasIP(),
                  ETH.localIP().toString().c_str());
  }
}

void loop() {
  static unsigned long last_print = 0;
  if (millis() - last_print > 10000) {
    last_print = millis();
    Serial.printf("\n[%lu] linkUp=%d hasIP=%d IP=%s MAC=%s\n",
                  millis() / 1000, (int)link_up, (int)got_ip,
                  ETH.localIP().toString().c_str(),
                  ETH.macAddress().c_str());
    dumpPhyRegisters();
  }
}
