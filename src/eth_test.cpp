// Minimal Aptinex IsolPoE Ethernet test — no SensESP framework.
//
// GPIO17 = LAN8720 nRST / oscillator enable (4.7k pulldown → LOW by default).
//
// Two clock modes are supported, selected by ETH_CLK_MODE build flag:
//
// ETH_CLOCK_GPIO0_IN (0): external 50MHz oscillator → GPIO0 RMII input.
//   GPIO17 = oscillator enable + PHY nRST.
//   lib/Ethernet/ETH.cpp has a patch that re-applies GPIO0 IOMUX after
//   perimanClearPinBus() resets it, so the clock is present during DMA init.
//
// ETH_CLOCK_GPIO17_OUT (3): ESP32 generates 50MHz on GPIO17.
//   GPIO17 is the clock output — no external oscillator, no PHY reset pin.
//   IDF passes power=-1; PHY self-resets on power-up.

#include <ETH.h>
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_private/esp_gpio_reserve.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_reg.h"
#include "lwip/netif.h"
#include "lwip/dhcp.h"
#include "esp_system.h"
#include "rom/rtc.h"
#include "soc/dport_access.h"

#define PHY_RST_PIN 17  // only used when ETH_CLK_MODE == ETH_CLOCK_GPIO0_IN

static bool eth_connected = false;

void onEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START: {
      // The real IOMUX fix is in lib/Ethernet/ETH.cpp: it re-applies GPIO0
      // IOMUX immediately after perimanClearPinBus(), before DMA init runs.
      // This event fires after esp_eth_start() returns (DMA already running),
      // so this re-apply is just a safety belt.
      esp_gpio_revoke(BIT64(GPIO_NUM_0));
      REG_SET_FIELD(IO_MUX_GPIO0_REG, MCU_SEL, FUNC_GPIO0_EMAC_TX_CLK);
      PIN_INPUT_ENABLE(IO_MUX_GPIO0_REG);
      CLEAR_PERI_REG_MASK(IO_MUX_GPIO0_REG, FUN_PD);
      CLEAR_PERI_REG_MASK(IO_MUX_GPIO0_REG, FUN_PU);
      // For RMII external clock input: IDF only sets ext_en (bit0), int_en=0.
      // clk_en (bit5) is NOT set by IDF — removing it to test if it blocks TX.
      REG_WRITE(0x3FF69808, 0x01);  // ext_en=1 only
      // ex_oscclk_conf.clk_sel (bit24) must be 1 for external GPIO0 clock.
      REG_SET_BIT(0x3FF69804, BIT(24));
      Serial.printf("ETH: Started (IOMUX MCU_SEL=%d need 5, clk_ctrl=0x%08x oscclk=0x%08x)\n",
                    (int)REG_GET_FIELD(IO_MUX_GPIO0_REG, MCU_SEL),
                    REG_READ(0x3FF69808), REG_READ(0x3FF69804));
      Serial.printf("ETH: GPIO19(TXD0) IOMUX=0x%08x MCU_SEL=%d (need 5)\n",
                    REG_READ(IO_MUX_GPIO19_REG),
                    (int)REG_GET_FIELD(IO_MUX_GPIO19_REG, MCU_SEL));
      Serial.printf("ETH: GPIO21(TX_EN)IOMUX=0x%08x MCU_SEL=%d (need 5)\n",
                    REG_READ(IO_MUX_GPIO21_REG),
                    (int)REG_GET_FIELD(IO_MUX_GPIO21_REG, MCU_SEL));
      Serial.printf("ETH: GPIO22(TXD1) IOMUX=0x%08x MCU_SEL=%d (need 5)\n",
                    REG_READ(IO_MUX_GPIO22_REG),
                    (int)REG_GET_FIELD(IO_MUX_GPIO22_REG, MCU_SEL));
      Serial.printf("ETH: GPIO25(RXD0) IOMUX=0x%08x MCU_SEL=%d (need 5)\n",
                    REG_READ(IO_MUX_GPIO25_REG),
                    (int)REG_GET_FIELD(IO_MUX_GPIO25_REG, MCU_SEL));
      Serial.printf("ETH: GPIO26(RXD1) IOMUX=0x%08x MCU_SEL=%d (need 5)\n",
                    REG_READ(IO_MUX_GPIO26_REG),
                    (int)REG_GET_FIELD(IO_MUX_GPIO26_REG, MCU_SEL));
      Serial.printf("ETH: GPIO27(RXDV) IOMUX=0x%08x MCU_SEL=%d (need 5)\n",
                    REG_READ(IO_MUX_GPIO27_REG),
                    (int)REG_GET_FIELD(IO_MUX_GPIO27_REG, MCU_SEL));
      Serial.printf("ETH: GPIO18(MDIO) IOMUX=0x%08x MCU_SEL=%d\n",
                    REG_READ(IO_MUX_GPIO18_REG),
                    (int)REG_GET_FIELD(IO_MUX_GPIO18_REG, MCU_SEL));
      Serial.printf("ETH: GPIO23(MDC)  IOMUX=0x%08x MCU_SEL=%d\n",
                    REG_READ(IO_MUX_GPIO23_REG),
                    (int)REG_GET_FIELD(IO_MUX_GPIO23_REG, MCU_SEL));
      // Raw MDIO read of PHY addr=1, regs 2+3 (PHY ID1+ID2) via EMAC MAC registers.
      #define EMAC_GMIIADDR 0x3FF6A010
      #define EMAC_GMIIDATA 0x3FF6A014
      REG_WRITE(EMAC_GMIIADDR, (1<<11)|(2<<6)|(0<<2)|(0<<1)|(1<<0)); // PHY=1,reg=2,CR=0,read,busy
      uint32_t t = millis();
      while ((REG_READ(EMAC_GMIIADDR) & 0x1) && (millis()-t < 10)) {}
      uint32_t phy_id1 = REG_READ(EMAC_GMIIDATA);
      REG_WRITE(EMAC_GMIIADDR, (1<<11)|(3<<6)|(0<<2)|(0<<1)|(1<<0)); // PHY=1,reg=3,CR=0,read,busy
      t = millis();
      while ((REG_READ(EMAC_GMIIADDR) & 0x1) && (millis()-t < 10)) {}
      uint32_t phy_id2 = REG_READ(EMAC_GMIIDATA);
      Serial.printf("ETH: MDIO PHY ID1=0x%04x  ID2=0x%04x (expect 0x0007 0xC0F0 for LAN8720)\n",
                    phy_id1 & 0xFFFF, phy_id2 & 0xFFFF);
      ETH.setHostname("eth-test");
      break;
    }
    case ARDUINO_EVENT_ETH_CONNECTED: {
      uint32_t mac_cr = REG_READ(0x3FF6A000);
      Serial.printf("ETH: Link up  TX_LIST=0x%08x  RX_LIST=0x%08x  OP_MODE=0x%08x\n",
                    REG_READ(0x3FF69010), REG_READ(0x3FF6900C), REG_READ(0x3FF69018));
      Serial.printf("ETH: MAC_CR=0x%08x (TX=%d RX=%d DM=%d FES=%d)  FLOW=0x%08x\n",
                    mac_cr,
                    (int)((mac_cr >> 3) & 1),
                    (int)((mac_cr >> 2) & 1),
                    (int)((mac_cr >> 11) & 1),
                    (int)((mac_cr >> 14) & 1),
                    REG_READ(0x3FF6A018));
      #ifndef EMAC_GMIIADDR
      #define EMAC_GMIIADDR 0x3FF6A010
      #define EMAC_GMIIDATA 0x3FF6A014
      #endif
      auto mdio_rd = [](int reg) -> uint16_t {
        REG_WRITE(EMAC_GMIIADDR, (1<<11)|(reg<<6)|(0<<2)|(0<<1)|(1<<0));
        uint32_t t2 = millis(); while ((REG_READ(EMAC_GMIIADDR)&1) && (millis()-t2<10)) {}
        return (uint16_t)(REG_READ(EMAC_GMIIDATA) & 0xFFFF);
      };
      uint16_t bmcr  = mdio_rd(0);
      uint16_t bmsr  = mdio_rd(1);
      uint16_t anar  = mdio_rd(4);
      uint16_t anlpar= mdio_rd(5);
      uint16_t pscsr = mdio_rd(31);  // Special Control/Status (LAN8720 reg31)
      Serial.printf("ETH: PHY BMSR=0x%04x (link=%d aneg=%d)  BMCR=0x%04x\n",
                    bmsr, (int)((bmsr>>2)&1), (int)((bmsr>>5)&1), bmcr);
      Serial.printf("ETH: PHY ANAR=0x%04x  ANLPAR=0x%04x\n", anar, anlpar);
      Serial.printf("ETH: PHY PSCSR(31)=0x%04x  speed_ind=%d\n",
                    pscsr, (pscsr>>2)&7);

      // Re-apply clk_ctrl.
      REG_WRITE(0x3FF69808, 0x01);  // ext_en=1 only (IDF default)
      REG_SET_BIT(0x3FF69804, BIT(24));

      // If DMA is in TX_STATE=6 (TTSE timestamp-write deadlock), unblock it by
      // stopping and restarting the TX DMA. Clearing ST aborts state=6 and puts
      // DMA into "stopped" state; re-setting ST makes it re-read from the head
      // of the TX descriptor ring. We also flush the TX FIFO (FTF) while ST=0
      // to drain any stale frame data.
      {
        uint32_t tx_state = (REG_READ(0x3FF69014) >> 20) & 7;
        Serial.printf("ETH: link-up  TX_STATE=%d  OP_MODE=0x%08x\n",
                      (int)tx_state, REG_READ(0x3FF69018));
        if (tx_state != 0) {
          Serial.printf("ETH: TX_STATE=%d -- stopping TX DMA to unblock\n", (int)tx_state);
          // Step 1: stop TX DMA (clear ST bit13)
          REG_CLR_BIT(0x3FF69018, BIT(13));
          uint32_t t_stop = millis();
          while (((REG_READ(0x3FF69014) >> 20) & 7) != 0 && (millis() - t_stop < 10)) {}
          Serial.printf("ETH: after ST=0  TX_STATE=%d  OP_MODE=0x%08x\n",
                        (int)((REG_READ(0x3FF69014) >> 20) & 7), REG_READ(0x3FF69018));
          // Step 2: flush TX FIFO while DMA is stopped
          REG_SET_BIT(0x3FF69018, BIT(20));  // FTF
          uint32_t t_flush = millis();
          while ((REG_READ(0x3FF69018) & BIT(20)) && (millis() - t_flush < 10)) {}
          Serial.printf("ETH: after FTF  OP_MODE=0x%08x  TX_FIFO_ne=%d\n",
                        REG_READ(0x3FF69018),
                        (int)((REG_READ(0x3FF6A024) >> 24) & 1));
          // Step 3: restart TX DMA (set ST bit13)
          REG_SET_BIT(0x3FF69018, BIT(13));
          // Step 4: write poll demand to wake DMA
          REG_WRITE(0x3FF69004, 1);
          delayMicroseconds(100);
          Serial.printf("ETH: after ST=1+poll  TX_STATE=%d  OP_MODE=0x%08x  STATUS=0x%08x\n",
                        (int)((REG_READ(0x3FF69014) >> 20) & 7),
                        REG_READ(0x3FF69018), REG_READ(0x3FF69014));
        }
      }
      // Try setting mii_clk_tx_en (bit3) and mii_clk_rx_en (bit4) in ex_clk_ctrl
      // in addition to ext_en (bit0). These are normally only set in MII mode,
      // but may be needed to gate the MTL TX FIFO read clock in RMII mode too.
      Serial.printf("ETH: ex_clk_ctrl before mii_clk_en patch=0x%08x\n", REG_READ(0x3FF69808));
      REG_SET_BIT(0x3FF69808, BIT(3) | BIT(4));  // mii_clk_tx_en=1, mii_clk_rx_en=1
      Serial.printf("ETH: ex_clk_ctrl after  mii_clk_en patch=0x%08x\n", REG_READ(0x3FF69808));

      // MAC loopback test: briefly enable MII loopback (MAC_CR bit12) to test
      // whether the MAC TX state machine activates at all. In loopback, TX data
      // is fed directly back into RX without going to the RMII wire. If MMC_TX
      // increments during loopback, the MAC TX engine works and the problem is
      // the RMII output path. If MMC_TX stays zero, the MAC TX itself is broken.
      {
        Serial.printf("ETH: loopback test -- MAC_CR before=0x%08x  MMC_TX before=%u\n",
                      REG_READ(0x3FF6A000), REG_READ(0x3FF6A118));
        REG_SET_BIT(0x3FF6A000, BIT(12));  // LM=1: enable MII loopback
        delayMicroseconds(200);
        REG_WRITE(0x3FF69004, 1);           // poll demand
        uint32_t t_lb = millis();
        while (REG_READ(0x3FF6A118) == 0 && (millis() - t_lb < 500)) {
          delayMicroseconds(100);
        }
        uint32_t mmc_lb = REG_READ(0x3FF6A118);
        uint32_t dbg_lb = REG_READ(0x3FF6A024);
        REG_CLR_BIT(0x3FF6A000, BIT(12));  // LM=0: disable loopback
        Serial.printf("ETH: loopback test done  MMC_TX=%u  MAC_DEBUG=0x%08x  (tpes=%d tfc=%d fifo_ne=%d)\n",
                      mmc_lb, dbg_lb,
                      (int)((dbg_lb >> 16) & 1),
                      (int)((dbg_lb >> 17) & 3),
                      (int)((dbg_lb >> 24) & 1));
        Serial.printf("ETH: %s\n", mmc_lb > 0
          ? "LOOPBACK OK: MAC TX works, problem is RMII output path"
          : "LOOPBACK FAIL: MAC TX engine not activating at all");
      }

      // Poll MAC_DEBUG, TX_BUF and MMC_TX rapidly for 5s to catch MAC TX activity.
      // MAC_DEBUG bits [18:17] = mactfcs (TX frame controller state),
      //   bit16=mactpes (TX protocol engine active), bit24=mtltfnes (FIFO not empty).
      // MMC_TX gb_frames at 0x3FF6A118 increments on each transmitted frame.
      Serial.printf("ETH: polling MAC_DEBUG+TX_BUF for 5s (ST=%d SR=%d)\n",
                    (int)((REG_READ(0x3FF69018) >> 13) & 1),
                    (int)((REG_READ(0x3FF69018) >> 1) & 1));
      {
        uint32_t last_txbuf = 0, last_status = 0, last_macdebug = 0, last_mmctx = 0;
        uint32_t t_poll = millis();
        while (millis() - t_poll < 5000) {
          uint32_t txbuf    = REG_READ(0x3FF69050);
          uint32_t status   = REG_READ(0x3FF69014);
          uint32_t macdebug = REG_READ(0x3FF6A024);
          uint32_t mmctx    = REG_READ(0x3FF6A118);  // MMC TX gb_frames
          if (txbuf != last_txbuf || status != last_status ||
              macdebug != last_macdebug || mmctx != last_mmctx) {
            Serial.printf("  t+%lums  TX_BUF=0x%08x  TX_ST=%d  DBG=0x%08x(tpes=%d tfc=%d fifo_ne=%d fifo_rd=%d)  MMC_TX=%u\n",
                          millis() - t_poll, txbuf, (int)((status >> 20) & 7),
                          macdebug,
                          (int)((macdebug >> 16) & 1),   // mactpes
                          (int)((macdebug >> 17) & 3),   // mactfcs
                          (int)((macdebug >> 24) & 1),   // mtltfnes
                          (int)((macdebug >> 20) & 3),   // mtltfrcs
                          mmctx);
            last_txbuf = txbuf; last_status = status;
            last_macdebug = macdebug; last_mmctx = mmctx;
          }
          delayMicroseconds(100);
        }
        Serial.printf("ETH: poll done  TX_BUF=0x%08x  MMC_TX=%u\n",
                      REG_READ(0x3FF69050), REG_READ(0x3FF6A118));
      }
      break;
    }
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("ETH: Got IP: %s  MAC: %s\n",
                    ETH.localIP().toString().c_str(),
                    ETH.macAddress().c_str());
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
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  esp_reset_reason_t reset_reason = esp_reset_reason();
  Serial.printf("\nAptinex IsolPoE ETH test starting... reset_reason=%d\n", (int)reset_reason);

#if ETH_CLK_MODE == ETH_CLOCK_GPIO0_IN
  // External oscillator mode: GPIO17 = oscillator enable + PHY nRST.
  delay(200);  // let GPIO0 strapping pin settle after cold boot
  pinMode(PHY_RST_PIN, OUTPUT);
  digitalWrite(PHY_RST_PIN, HIGH);
  delay(100);
  Serial.println("ETH: oscillator on");

  // Pre-apply GPIO0 IOMUX so the clock is present from the start.
  // lib/Ethernet/ETH.cpp re-applies it after perimanClearPinBus() resets it.
  esp_gpio_revoke(BIT64(GPIO_NUM_0));
  REG_SET_FIELD(IO_MUX_GPIO0_REG, MCU_SEL, FUNC_GPIO0_EMAC_TX_CLK);
  PIN_INPUT_ENABLE(IO_MUX_GPIO0_REG);
  CLEAR_PERI_REG_MASK(IO_MUX_GPIO0_REG, FUN_PD);
  CLEAR_PERI_REG_MASK(IO_MUX_GPIO0_REG, FUN_PU);
  Serial.printf("ETH: GPIO0 IOMUX set  MCU_SEL=%d (need 5)\n",
                (int)REG_GET_FIELD(IO_MUX_GPIO0_REG, MCU_SEL));

  digitalWrite(PHY_RST_PIN, LOW);
  delay(10);
  digitalWrite(PHY_RST_PIN, HIGH);
  delay(50);
  Serial.println("ETH: PHY reset done");
#else
  // Internal clock mode: IDF drives GPIO17 as clock output.
  Serial.println("ETH: using internal clock on GPIO17");
#endif

  WiFi.mode(WIFI_OFF);
  Network.onEvent(onEvent);
  Serial.printf("ETH: free heap=%u  largest_block=%u\n",
                esp_get_free_heap_size(),
                heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL));

  Serial.printf("ETH: EMAC_EX clk_ctrl=0x%08x  phyinf=0x%08x (before begin)\n",
                REG_READ(0x3FF69808), REG_READ(0x3FF6980C));

  Serial.printf("ETH: calling ETH.begin clk_mode=%d\n", (int)ETH_CLK_MODE);
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO,
            -1, ETH_CLK_MODE);

  Serial.printf("ETH: after ETH.begin  TX_LIST=0x%08x  RX_LIST=0x%08x\n",
                REG_READ(0x3FF69010), REG_READ(0x3FF6900C));
  Serial.printf("ETH: EMAC_EX clk_ctrl=0x%08x  phyinf=0x%08x (after begin)\n",
                REG_READ(0x3FF69808), REG_READ(0x3FF6980C));
  Serial.printf("ETH: DMA BUS_MODE=0x%08x (alt_desc=%d)  OP_MODE=0x%08x  STATUS=0x%08x\n",
                REG_READ(0x3FF69000), (int)((REG_READ(0x3FF69000)>>7)&1),
                REG_READ(0x3FF69018), REG_READ(0x3FF69014));
  Serial.printf("ETH: MAC_CR=0x%08x (TX=%d RX=%d) after begin\n",
                REG_READ(0x3FF6A000),
                (int)((REG_READ(0x3FF6A000)>>3)&1),
                (int)((REG_READ(0x3FF6A000)>>2)&1));
  // ETH.cpp calls ETH_MAC_ESP_CMD_CLEAR_TDES0_CFG_BITS after esp_eth_start(),
  // which clears TTSE (bit30) from the DMA's TDES0 template. Verify here.
  Serial.printf("ETH: DMA TX_STATE=%d (should not be 6 after TTSE fix)\n",
                (int)((REG_READ(0x3FF69014) >> 20) & 7));
}

void loop() {
  if (millis() > 20000 && !ETH.linkUp()) {
    Serial.println("ETH: no link after 20s — hard reset");
    delay(100);
    esp_restart();
  }

  static unsigned long last = 0;
  if (millis() - last > 5000) {
    last = millis();
    Serial.printf("uptime=%lus  linked=%d  hasIP=%d  ip=%s  MAC=%s\n",
                  millis() / 1000,
                  (int)ETH.linkUp(),
                  (int)ETH.hasIP(),
                  ETH.localIP().toString().c_str(),
                  ETH.macAddress().c_str());

    uint32_t dma_status  = REG_READ(0x3FF69014);
    uint32_t dma_op_mode = REG_READ(0x3FF69018);
    uint32_t dma_tx_list = REG_READ(0x3FF69010);
    uint32_t dma_tx_desc = REG_READ(0x3FF69048);  // dmatxcurrdesc
    uint32_t dma_tx_buf  = REG_READ(0x3FF69050);  // dmatxcurraddr_buf
    uint32_t dma_rx_list = REG_READ(0x3FF6900C);
    uint32_t dma_rx_desc = REG_READ(0x3FF6904C);  // dmarxcurrdesc
    uint32_t mac_cr    = REG_READ(0x3FF6A000);
    uint32_t mac_debug = REG_READ(0x3FF6A024);  // emacdebug
    uint32_t mac_intr  = REG_READ(0x3FF6A038);  // MAC interrupt status
    uint32_t phyinf_val = REG_READ(0x3FF6980C);
    uint32_t oscclk_val = REG_READ(0x3FF69804);
    Serial.printf("  EMAC_EX: clkout=0x%08x  oscclk=0x%08x (clk_sel=%d)  clk_ctrl=0x%08x  phyinf=0x%08x (intf=%d need 4)\n",
                  REG_READ(0x3FF69800), oscclk_val, (int)((oscclk_val>>24)&1),
                  REG_READ(0x3FF69808), phyinf_val, (int)((phyinf_val>>13)&7));
    Serial.printf("  MMC_CTRL=0x%08x\n", REG_READ(0x3FF6A100));
    Serial.printf("  MMC_TX: gb_frames=%08x gb_bytes=%08x g_bytes=%08x g_frames=%08x\n",
                  REG_READ(0x3FF6A118), REG_READ(0x3FF6A114),
                  REG_READ(0x3FF6A164), REG_READ(0x3FF6A168));
    Serial.printf("  MMC_TX: bcast_g=%08x mcast_g=%08x 64oct=%08x\n",
                  REG_READ(0x3FF6A120), REG_READ(0x3FF6A124), REG_READ(0x3FF6A128));
    Serial.printf("  MMC_RX: gb_frames=%08x gb_bytes=%08x g_bytes=%08x g_frames=%08x crc=%08x\n",
                  REG_READ(0x3FF6A188), REG_READ(0x3FF6A184),
                  REG_READ(0x3FF6A1D8), REG_READ(0x3FF6A1DC), REG_READ(0x3FF6A1C0));
    Serial.printf("  MAC_DEBUG=0x%08x (TX_proto=%d TX_fc=%d TX_fifo_rd=%d TX_fifo_ne=%d TX_fifo_wr=%d)\n",
                  mac_debug,
                  (int)((mac_debug >> 16) & 1),
                  (int)((mac_debug >> 17) & 3),
                  (int)((mac_debug >> 20) & 3),
                  (int)((mac_debug >> 24) & 1),
                  (int)((mac_debug >> 22) & 1));
    Serial.printf("  MAC_STATUS=0x%08x (speed=%d need 1, duplex=%d need 1)\n",
                  REG_READ(0x3FF6A0D8),
                  (int)((REG_READ(0x3FF6A0D8)>>1)&3),
                  (int)(REG_READ(0x3FF6A0D8)&1));
    Serial.printf("  MAC_INTR=0x%08x  DMA_INTR_EN=0x%08x\n", mac_intr, REG_READ(0x3FF6901C));
    Serial.printf("  DMA TX_STATE=%d  OP_MODE=0x%08x  TX_LIST=0x%08x  TX_DESC=0x%08x  TX_BUF=0x%08x\n",
                  (int)((dma_status >> 20) & 0x7),
                  dma_op_mode,
                  dma_tx_list, dma_tx_desc, dma_tx_buf);
    Serial.printf("  DMA STATUS=0x%08x (TI=%d TPS=%d TBU=%d TU=%d TJT=%d UNF=%d RI=%d RBU=%d RPS=%d RWT=%d ETI=%d FBI=%d ERI=%d AIS=%d NIS=%d err=%d)\n",
                  dma_status,
                  (int)(dma_status & BIT(0)),           // TI  transmit interrupt
                  (int)((dma_status >> 1) & 1),         // TPS transmit process stopped
                  (int)((dma_status >> 2) & 1),         // TBU transmit buffer unavailable
                  (int)((dma_status >> 3) & 1),         // TJT transmit jabber timeout
                  (int)((dma_status >> 4) & 1),         // OVF receive overflow
                  (int)((dma_status >> 5) & 1),         // UNF transmit underflow
                  (int)((dma_status >> 6) & 1),         // RI  receive interrupt
                  (int)((dma_status >> 7) & 1),         // RBU receive buffer unavailable
                  (int)((dma_status >> 8) & 1),         // RPS receive process stopped
                  (int)((dma_status >> 9) & 1),         // RWT receive watchdog timeout
                  (int)((dma_status >> 10) & 1),        // ETI early transmit interrupt
                  (int)((dma_status >> 13) & 1),        // FBI fatal bus error
                  (int)((dma_status >> 14) & 1),        // ERI early receive interrupt
                  (int)((dma_status >> 15) & 1),        // AIS abnormal interrupt summary
                  (int)((dma_status >> 16) & 1),        // NIS normal interrupt summary
                  (int)((dma_status >> 23) & 7));       // error bits [25:23]
    Serial.printf("  DMA RX_LIST=0x%08x  RX_DESC=0x%08x\n", dma_rx_list, dma_rx_desc);
    Serial.printf("  DMA MISSED_FRAMES=0x%08x  dmatxcurraddr=0x%08x  dmarxcurraddr=0x%08x\n",
                  REG_READ(0x3FF69020), REG_READ(0x3FF69050), REG_READ(0x3FF69054));
    Serial.printf("  MAC_CR=0x%08x (TX=%d RX=%d DM=%d FES=%d)  OP flush=%d sfwd=%d ST=%d SR=%d\n",
                  mac_cr,
                  (int)((mac_cr >> 3) & 1),
                  (int)((mac_cr >> 2) & 1),
                  (int)((mac_cr >> 11) & 1),
                  (int)((mac_cr >> 14) & 1),
                  (int)((dma_op_mode >> 20) & 1),
                  (int)((dma_op_mode >> 21) & 1),
                  (int)((dma_op_mode >> 13) & 1),
                  (int)((dma_op_mode >>  1) & 1));
    Serial.printf("  MAC_TS_CTRL=0x%08x  (bit0=ts_en bit8=all_frames)\n",
                  REG_READ(0x3FF6A700));
    if (dma_tx_desc >= 0x3FF00000 && dma_tx_desc <= 0x3FFFFFFF) {
      volatile uint32_t* d = (volatile uint32_t*)dma_tx_desc;
      Serial.printf("  TX_DESC @0x%08x  DES0=0x%08x DES1=0x%08x DES2=0x%08x DES3=0x%08x OWN=%d IC=%d TTSE=%d\n",
                    dma_tx_desc, d[0], d[1], d[2], d[3],
                    (int)(d[0] >> 31), (int)((d[0] >> 30) & 1), (int)((d[0] >> 25) & 1));
    }
    if (dma_tx_list >= 0x3FF00000 && dma_tx_list <= 0x3FFFFFFF) {
      volatile uint32_t* d = (volatile uint32_t*)dma_tx_list;
      Serial.printf("  TX[0] DES0..5: %08x %08x %08x %08x | %08x %08x\n",
                    d[0], d[1], d[2], d[3], d[4], d[5]);
    }
    static bool tx_ring_dumped = false;
    if (!tx_ring_dumped && dma_tx_list >= 0x3FF00000 && dma_tx_list <= 0x3FFFFFFF) {
      tx_ring_dumped = true;
      volatile uint32_t* tx0d = (volatile uint32_t*)dma_tx_list;
      uint32_t tx0_buf = tx0d[2];
      if (tx0_buf >= 0x3FF00000 && tx0_buf <= 0x3FFFFFFF) {
        volatile uint8_t* buf = (volatile uint8_t*)tx0_buf;
        Serial.printf("  TX[0] buf @0x%08x first 32 bytes:\n    ", tx0_buf);
        for (int i = 0; i < 32; i++) Serial.printf("%02x ", buf[i]);
        Serial.println();
      }
      Serial.println("  TX descriptor ring dump:");
      uint32_t desc = dma_tx_list;
      for (int i = 0; i < 32; i++) {
        if (desc < 0x3FF00000 || desc > 0x3FFFFFFF) break;
        volatile uint32_t* d = (volatile uint32_t*)desc;
        Serial.printf("    TX[%2d] @0x%08x  DES0=0x%08x DES1=0x%08x DES2=0x%08x DES3=0x%08x  OWN=%d IC=%d TTSE=%d\n",
                      i, desc, d[0], d[1], d[2], d[3], (int)(d[0]>>31), (int)((d[0]>>30)&1), (int)((d[0]>>25)&1));
        if (d[3] == dma_tx_list) { Serial.println("    (ring end)"); break; }
        desc = d[3];
      }
    }
    static bool rx_ring_dumped = false;
    if (!rx_ring_dumped && dma_rx_list >= 0x3FF00000 && dma_rx_list <= 0x3FFFFFFF) {
      rx_ring_dumped = true;
      Serial.println("  RX descriptor ring dump:");
      uint32_t desc = dma_rx_list;
      for (int i = 0; i < 32; i++) {
        if (desc < 0x3FF00000 || desc > 0x3FFFFFFF) break;
        volatile uint32_t* d = (volatile uint32_t*)desc;
        Serial.printf("    RX[%2d] @0x%08x  DES0=0x%08x DES1=0x%08x DES2=0x%08x DES3=0x%08x\n",
                      i, desc, d[0], d[1], d[2], d[3]);
        if (d[3] == dma_rx_list) { Serial.println("    (ring end)"); break; }
        desc = d[3];
      }
    }
    for (struct netif* nif = netif_list; nif != NULL; nif = nif->next) {
      Serial.printf("  netif %c%c%d  flags=0x%02x  ip=%s\n",
                    nif->name[0], nif->name[1], nif->num,
                    nif->flags,
                    ip4addr_ntoa(&nif->ip_addr.u_addr.ip4));
      struct dhcp* d = netif_dhcp_data(nif);
      if (d) {
        Serial.printf("    DHCP state=%d tries=%d\n", (int)d->state, (int)d->tries);
      }
    }
  }
}
