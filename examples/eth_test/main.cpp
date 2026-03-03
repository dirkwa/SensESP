/**
 * @file main.cpp
 * @brief Minimal Ethernet test for Aptinex IsolPoE ESP32
 *
 * Diagnoses RMII data path: PHY link is up but no frames traverse.
 * Reads EMAC DMA/MAC debug registers and attempts outbound ping.
 */

#include <Arduino.h>
#include <ETH.h>
#include <driver/gpio.h>
#include <esp_eth_driver.h>
#include <ping/ping_sock.h>
#include <soc/emac_dma_struct.h>
#include <soc/emac_mac_struct.h>

// Aptinex IsolPoE pin configuration
#define ETH_PHY_TYPE   ETH_PHY_LAN8720
#define ETH_PHY_ADDR   1
#define ETH_PHY_MDC    23
#define ETH_PHY_MDIO   18
#define ETH_PHY_POWER  17
#define ETH_CLK_MODE   ETH_CLOCK_GPIO0_IN

static volatile bool link_up = false;
static volatile bool got_ip = false;

// Read a PHY register via ESP-IDF ioctl (ETHClass has no phyRead)
static uint16_t phyRead(uint32_t reg) {
  uint32_t value = 0;
  esp_eth_phy_reg_rw_data_t rw = { .reg_addr = reg, .reg_value_p = &value };
  esp_eth_ioctl(ETH.handle(), ETH_CMD_READ_PHY_REG, &rw);
  return (uint16_t)value;
}

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

  uint16_t bcr = phyRead(0x00);
  Serial.printf("Reg 0  BCR:   0x%04X", bcr);
  if (bcr & 0x2000) Serial.print(" [100Mbps]");
  if (bcr & 0x1000) Serial.print(" [AN_ENABLE]");
  if (bcr & 0x0100) Serial.print(" [FULL_DUPLEX]");
  Serial.println();

  uint16_t bsr = phyRead(0x01);
  Serial.printf("Reg 1  BSR:   0x%04X", bsr);
  if (bsr & 0x0004) Serial.print(" [LINK_UP]");
  else Serial.print(" [LINK_DOWN]");
  if (bsr & 0x0020) Serial.print(" [AN_COMPLETE]");
  Serial.println();

  uint16_t anlpar = phyRead(0x05);
  Serial.printf("Reg 5  ANLPAR:0x%04X\n", anlpar);

  uint16_t pscsr = phyRead(0x1F);
  Serial.printf("Reg 31 PSCSR: 0x%04X", pscsr);
  if (pscsr & 0x1000) Serial.print(" [AUTODONE]");
  int speed_ind = (pscsr >> 2) & 0x07;
  const char* speeds[] = {"?","10HD","100HD","?","?","10FD","100FD","?"};
  Serial.printf(" [%s]", speeds[speed_ind]);
  Serial.println();
}

void dumpEmacState() {
  Serial.println("--- EMAC DMA/MAC State ---");

  // DMA status register
  uint32_t st = EMAC_DMA.dmastatus.val;
  Serial.printf("DMA Status:    0x%08lX\n", st);

  int tx_ps = EMAC_DMA.dmastatus.trans_proc_state;
  int rx_ps = EMAC_DMA.dmastatus.recv_proc_state;
  const char* tx_s[] = {"Stopped","FetchDesc","Reserved","WaitStatus",
                         "Suspended","CloseDesc","WriteTS","ReadBuf"};
  const char* rx_s[] = {"Stopped","FetchDesc","Reserved","WaitPkt",
                         "Suspended","CloseDesc","WriteTS","XferBuf"};
  Serial.printf("  TX: %d (%s)  RX: %d (%s)\n",
                tx_ps, tx_s[tx_ps], rx_ps, rx_s[rx_ps]);

  // Error flags
  if (EMAC_DMA.dmastatus.trans_undflow)
    Serial.println("  *** TX UNDERFLOW ***");
  if (EMAC_DMA.dmastatus.recv_ovflow)
    Serial.println("  *** RX OVERFLOW ***");
  if (EMAC_DMA.dmastatus.fatal_bus_err_int)
    Serial.printf("  *** FATAL BUS ERROR (type=%d) ***\n",
                  EMAC_DMA.dmastatus.error_bits);
  if (EMAC_DMA.dmastatus.abn_int_summ)
    Serial.println("  *** ABNORMAL INT ***");

  // Missed frame counter (clears on read)
  uint32_t mf = EMAC_DMA.dmamissedfr.val;
  uint16_t missed = mf & 0xFFFF;
  uint16_t overflow = (mf >> 17) & 0x7FF;
  Serial.printf("Missed frames: %u  Overflow: %u\n", missed, overflow);

  // MAC debug register
  uint32_t dbg = EMAC_MAC.emacdebug.val;
  Serial.printf("MAC Debug:     0x%08lX\n", dbg);
  Serial.printf("  RX engine active: %lu  TX engine active: %lu\n",
                EMAC_MAC.emacdebug.macrpes, EMAC_MAC.emacdebug.mactpes);
  Serial.printf("  RX FIFO level: %lu  TX FIFO not empty: %lu\n",
                EMAC_MAC.emacdebug.mtlrffls, EMAC_MAC.emacdebug.mtltfnes);
  Serial.printf("  TX FC state: %lu  TX FIFO read state: %lu\n",
                EMAC_MAC.emacdebug.mactfcs, EMAC_MAC.emacdebug.mtltfrcs);

  // MAC config - check TX and RX enable
  uint32_t cfg = EMAC_MAC.gmacconfig.val;
  Serial.printf("MAC Config:    0x%08lX (TX=%lu RX=%lu FES=%lu DM=%lu)\n",
                cfg,
                EMAC_MAC.gmacconfig.tx, EMAC_MAC.gmacconfig.rx,
                EMAC_MAC.gmacconfig.fespeed, EMAC_MAC.gmacconfig.duplex);

  Serial.println("-----------------------------");
}

// Ping callbacks
static void ping_success_cb(esp_ping_handle_t hdl, void *args) {
  uint32_t elapsed;
  esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed, sizeof(elapsed));
  Serial.printf("PING reply in %lu ms\n", elapsed);
}

static void ping_timeout_cb(esp_ping_handle_t hdl, void *args) {
  Serial.println("PING timeout");
}

static void ping_end_cb(esp_ping_handle_t hdl, void *args) {
  uint32_t transmitted, received;
  esp_ping_get_profile(hdl, ESP_PING_PROF_REQUEST, &transmitted, sizeof(transmitted));
  esp_ping_get_profile(hdl, ESP_PING_PROF_REPLY, &received, sizeof(received));
  Serial.printf("PING done: %lu TX, %lu RX\n", transmitted, received);
  esp_ping_delete_session(hdl);
}

static void startPing(const char* target) {
  ip_addr_t addr;
  addr.type = IPADDR_TYPE_V4;
  addr.u_addr.ip4.addr = ipaddr_addr(target);

  esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
  cfg.target_addr = addr;
  cfg.count = 5;
  cfg.interval_ms = 1000;
  cfg.timeout_ms = 1000;

  esp_ping_callbacks_t cbs = {};
  cbs.on_ping_success = ping_success_cb;
  cbs.on_ping_timeout = ping_timeout_cb;
  cbs.on_ping_end = ping_end_cb;

  esp_ping_handle_t ping;
  esp_err_t err = esp_ping_new_session(&cfg, &cbs, &ping);
  if (err != ESP_OK) {
    Serial.printf("Ping session failed: %s\n", esp_err_to_name(err));
    return;
  }
  Serial.printf("Pinging %s ...\n", target);
  esp_ping_start(ping);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== Aptinex Ethernet Diag v2 ===");

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

  // Wait for link
  delay(2000);

  // Dump everything
  dumpPhyRegisters();
  dumpEmacState();

  // Assign static IP
  if (link_up) {
    Serial.println("Assigning static IP 192.168.0.120...");
    ETH.config(IPAddress(192, 168, 0, 120),
               IPAddress(192, 168, 0, 1),
               IPAddress(255, 255, 255, 0),
               IPAddress(192, 168, 0, 1));

    for (int i = 0; i < 30 && !got_ip; i++) delay(100);
    Serial.printf("hasIP=%d IP=%s\n", (int)ETH.hasIP(),
                  ETH.localIP().toString().c_str());

    // Ping gateway — forces ARP + ICMP TX, shows if data path works
    if (got_ip) {
      delay(500);
      Serial.println("\n=== Pre-ping EMAC state ===");
      dumpEmacState();
      startPing("192.168.0.147");  // RPi5
    }
  }
}

void loop() {
  static unsigned long last_print = 0;
  if (millis() - last_print > 15000) {
    last_print = millis();
    Serial.printf("\n[%lu] linkUp=%d hasIP=%d IP=%s MAC=%s\n",
                  millis() / 1000, (int)link_up, (int)got_ip,
                  ETH.localIP().toString().c_str(),
                  ETH.macAddress().c_str());
    dumpPhyRegisters();
    dumpEmacState();
  }
}
