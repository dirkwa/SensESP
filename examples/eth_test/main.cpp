/**
 * @file main.cpp
 * @brief Minimal Ethernet test for Aptinex IsolPoE ESP32
 *
 * Waits for link, assigns static IP, pings, dumps EMAC state.
 */

#include <Arduino.h>
#include <ETH.h>
#include <driver/gpio.h>
#include <esp_eth_driver.h>
#include <ping/ping_sock.h>
#include <soc/emac_dma_struct.h>
#include <soc/emac_mac_struct.h>
#include <soc/emac_ext_struct.h>

// Aptinex IsolPoE pin configuration
#define ETH_PHY_TYPE   ETH_PHY_LAN8720
#define ETH_PHY_ADDR   1
#define ETH_PHY_MDC    23
#define ETH_PHY_MDIO   18
#define ETH_PHY_POWER  17
#define ETH_CLK_MODE   ETH_CLOCK_GPIO0_IN

static volatile bool eth_started = false;
static volatile bool link_up = false;
static volatile bool got_ip = false;

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
      eth_started = true;
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.printf("ETH: Link up (%s, %d Mbps) at %lums\n",
                     ETH.fullDuplex() ? "full duplex" : "half duplex",
                     ETH.linkSpeed(), millis());
      link_up = true;
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("ETH: Got IP %s at %lums\n",
                     ETH.localIP().toString().c_str(), millis());
      got_ip = true;
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
      Serial.println("ETH: Lost IP");
      got_ip = false;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.printf("ETH: Link DOWN at %lums\n", millis());
      link_up = false;
      got_ip = false;
      break;
    default:
      break;
  }
}

void dumpAll() {
  Serial.println("\n--- PHY ---");
  uint16_t bsr = phyRead(0x01);
  Serial.printf("BSR:  0x%04X %s %s\n", bsr,
                (bsr & 0x0004) ? "LINK_UP" : "LINK_DOWN",
                (bsr & 0x0020) ? "AN_DONE" : "");
  Serial.printf("ANLPAR: 0x%04X  PSCSR: 0x%04X\n", phyRead(0x05), phyRead(0x1F));

  Serial.println("--- EMAC ---");
  uint32_t cfg = EMAC_MAC.gmacconfig.val;
  Serial.printf("MAC Config: 0x%08lX  TX=%lu RX=%lu FES=%lu DM=%lu\n",
                cfg,
                EMAC_MAC.gmacconfig.tx, EMAC_MAC.gmacconfig.rx,
                EMAC_MAC.gmacconfig.fespeed, EMAC_MAC.gmacconfig.duplex);

  Serial.printf("DMA Status: 0x%08lX  TX_state=%d RX_state=%d\n",
                EMAC_DMA.dmastatus.val,
                EMAC_DMA.dmastatus.trans_proc_state,
                EMAC_DMA.dmastatus.recv_proc_state);

  uint32_t dbg = EMAC_MAC.emacdebug.val;
  Serial.printf("MAC Debug:  0x%08lX  TX_active=%lu RX_active=%lu FIFO_rx=%lu FIFO_tx_ne=%lu\n",
                dbg,
                EMAC_MAC.emacdebug.mactpes, EMAC_MAC.emacdebug.macrpes,
                EMAC_MAC.emacdebug.mtlrffls, EMAC_MAC.emacdebug.mtltfnes);

  if (EMAC_DMA.dmastatus.trans_undflow) Serial.println("  *** TX UNDERFLOW ***");
  if (EMAC_DMA.dmastatus.fatal_bus_err_int) Serial.println("  *** FATAL BUS ERROR ***");
  if (EMAC_DMA.dmastatus.abn_int_summ) Serial.println("  *** ABNORMAL INT ***");

  Serial.printf("Missed: %lu  Overflow: %lu\n",
                EMAC_DMA.dmamissedfr.missed_fc, EMAC_DMA.dmamissedfr.overflow_fc);

  // EMAC EXT clock configuration
  Serial.println("--- CLK ---");
  Serial.printf("ex_clk_ctrl:    0x%08lX  ext_en=%lu int_en=%lu clk_en=%lu\n",
                EMAC_EXT.ex_clk_ctrl.val,
                EMAC_EXT.ex_clk_ctrl.ext_en, EMAC_EXT.ex_clk_ctrl.int_en,
                EMAC_EXT.ex_clk_ctrl.clk_en);
  Serial.printf("ex_oscclk_conf: 0x%08lX  clk_sel=%lu\n",
                EMAC_EXT.ex_oscclk_conf.val, EMAC_EXT.ex_oscclk_conf.clk_sel);
  Serial.printf("ex_phyinf_conf: 0x%08lX  phy_intf_sel=%lu\n",
                EMAC_EXT.ex_phyinf_conf.val, EMAC_EXT.ex_phyinf_conf.phy_intf_sel);
  Serial.printf("ex_clkout_conf: 0x%08lX\n", EMAC_EXT.ex_clkout_conf.val);
  Serial.println("---");
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
  if (esp_ping_new_session(&cfg, &cbs, &ping) == ESP_OK) {
    Serial.printf("Pinging %s ...\n", target);
    esp_ping_start(ping);
  } else {
    Serial.println("Failed to create ping session");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== Aptinex Ethernet Diag v4 ===");

  Network.onEvent(onEthEvent);

  // Ensure GPIO0 is clean input before PHY powers up
  // (LAN8720 samples nINT/REFCLKO at reset for clock mode strap)
  gpio_reset_pin(GPIO_NUM_0);
  gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
  gpio_pullup_dis(GPIO_NUM_0);
  gpio_pulldown_dis(GPIO_NUM_0);

  // Power-cycle PHY: ensure clean reset
  pinMode(ETH_PHY_POWER, OUTPUT);
  digitalWrite(ETH_PHY_POWER, LOW);
  delay(100);
  Serial.println("Powering PHY ON...");
  digitalWrite(ETH_PHY_POWER, HIGH);
  delay(500);  // LAN8720 needs ~200ms PLL lock after power-on

  // Start Ethernet
  bool ok = ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC,
                       ETH_PHY_MDIO, -1, ETH_CLK_MODE);
  Serial.printf("ETH.begin() = %s\n", ok ? "true" : "false");

  // Wait up to 10s for link
  Serial.println("Waiting for link...");
  for (int i = 0; i < 100 && !link_up; i++) {
    if (i % 10 == 0) Serial.printf("  %ds linkUp=%d BSR=0x%04X\n",
                                     i/10, (int)link_up, phyRead(0x01));
    delay(100);
  }

  if (!link_up) {
    Serial.println("*** NO LINK after 10s ***");
    dumpAll();
    return;
  }

  Serial.println("\n=== Link is UP — checking EMAC state ===");
  dumpAll();

  // Assign static IP
  Serial.println("Assigning static IP 192.168.0.120...");
  ETH.config(IPAddress(192, 168, 0, 120),
             IPAddress(192, 168, 0, 1),
             IPAddress(255, 255, 255, 0),
             IPAddress(192, 168, 0, 1));

  for (int i = 0; i < 50 && !got_ip; i++) delay(100);

  if (got_ip) {
    Serial.printf("Got IP: %s\n", ETH.localIP().toString().c_str());
    Serial.println("\n=== Post-IP EMAC state ===");
    dumpAll();
    delay(500);
    startPing("192.168.0.147");
  } else {
    Serial.println("*** No IP after 5s ***");
    dumpAll();
  }
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last > 10000) {
    last = millis();
    Serial.printf("\n[%lus] link=%d ip=%d %s MAC=%s\n",
                  millis()/1000, (int)link_up, (int)got_ip,
                  ETH.localIP().toString().c_str(),
                  ETH.macAddress().c_str());
    dumpAll();
  }
}
