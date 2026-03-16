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
      // mii_clk_tx_en (bit3) + mii_clk_rx_en (bit4) must be set for the 50MHz
      // REF_CLK to reach the RMII TX/RX clock domains. IDF v5
      // emac_ll_clock_enable_rmii_input() only sets ext_en (bit0).
      // Apply here (ETH_START) so it's in place before DHCP queues any TX.
      REG_WRITE(0x3FF69808, REG_READ(0x3FF69808) | (1<<3) | (1<<4) | (1<<5));
      // ex_oscclk_conf.clk_sel (bit20 of 0x3FF69804) must be 1 to route the
      // external GPIO0 clock through the RMII clock mux. Observed to read 0
      // despite emac_ll_clock_enable_rmii_input() setting it; force it here.
      REG_SET_BIT(0x3FF69804, BIT(20));
      Serial.printf("ETH: Started (IOMUX MCU_SEL=%d need 5, clk_ctrl=0x%08x oscclk=0x%08x)\n",
                    (int)REG_GET_FIELD(IO_MUX_GPIO0_REG, MCU_SEL),
                    REG_READ(0x3FF69808), REG_READ(0x3FF69804));
      // Check GPIO18 (MDIO) and GPIO23 (MDC) IOMUX state: MCU_SEL=2 = GPIO matrix
      // RMII TX pins must be MCU_SEL=5 (EMAC IOMUX), not 2 (GPIO matrix)
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
      // MAC block at 0x3FF6A000; emacgmiiaddr=+0x10, emacmiidata=+0x14
      // bit0=miibusy, bit1=miiwrite, bits[5:2]=CR, bits[10:6]=miireg, bits[15:11]=miidev
      // CR=0 → div42 for 80MHz APB → ~1.9MHz MDIO
      #define EMAC_GMIIADDR 0x3FF6A010
      #define EMAC_GMIIDATA 0x3FF6A014
      // read: miiwrite=0, miibusy=1
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
      // PHY BMSR (reg1) and BMCR (reg0) via MDIO
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
      uint16_t anar  = mdio_rd(4);   // AN advertise
      uint16_t anlpar= mdio_rd(5);   // AN link partner
      uint16_t pscsr = mdio_rd(31);  // Special Control/Status (LAN8720 reg31)
      Serial.printf("ETH: PHY BMSR=0x%04x (link=%d aneg=%d)  BMCR=0x%04x\n",
                    bmsr, (int)((bmsr>>2)&1), (int)((bmsr>>5)&1), bmcr);
      Serial.printf("ETH: PHY ANAR=0x%04x  ANLPAR=0x%04x\n", anar, anlpar);
      // PSCSR bits[4:2]: 001=10H,010=10F,101=100H,110=100F
      Serial.printf("ETH: PHY PSCSR(31)=0x%04x  speed_ind=%d\n",
                    pscsr, (pscsr>>2)&7);
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
  // FUNC_GPIO0_EMAC_TX_CLK=5 is the correct IOMUX function for RMII REF_CLK input.
  // (Despite the confusing name, this is the input path — confirmed from IDF source.)
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
  // No oscillator enable needed; PHY self-resets on power-up.
  Serial.println("ETH: using internal clock on GPIO17");
#endif

  WiFi.mode(WIFI_OFF);
  Network.onEvent(onEvent);
  Serial.printf("ETH: free heap=%u  largest_block=%u\n",
                esp_get_free_heap_size(),
                heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL));

  // EMAC_EXT base=0x3FF69800: +0x00=ex_clkout_conf, +0x04=ex_oscclk_conf,
  //   +0x08=ex_clk_ctrl (bit0=ext_en, bit1=int_en), +0x0C=ex_phyinf_conf
  Serial.printf("ETH: EMAC_EX clk_ctrl=0x%08x  phyinf=0x%08x (before begin)\n",
                REG_READ(0x3FF69808), REG_READ(0x3FF6980C));

  Serial.printf("ETH: calling ETH.begin clk_mode=%d\n", (int)ETH_CLK_MODE);
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO,
            -1, ETH_CLK_MODE);

  Serial.printf("ETH: after ETH.begin  TX_LIST=0x%08x  RX_LIST=0x%08x\n",
                REG_READ(0x3FF69010), REG_READ(0x3FF6900C));
  // DPORT_WIFI_CLK_EN bit14=EMAC, EMAC_EX PHYINF bit5=RMII
  // EMAC_DMA_BUS_MODE=0x3FF69000, EMAC_DMA_OP_MODE=0x3FF69018
  Serial.printf("ETH: EMAC_EX clk_ctrl=0x%08x  phyinf=0x%08x (after begin)\n",
                REG_READ(0x3FF69808), REG_READ(0x3FF6980C));
  Serial.printf("ETH: DMA BUS_MODE=0x%08x (alt_desc=%d)  OP_MODE=0x%08x  STATUS=0x%08x\n",
                REG_READ(0x3FF69000), (int)((REG_READ(0x3FF69000)>>7)&1),
                REG_READ(0x3FF69018), REG_READ(0x3FF69014));
  Serial.printf("ETH: MAC_CR=0x%08x (TX=%d RX=%d) after begin\n",
                REG_READ(0x3FF6A000),
                (int)((REG_READ(0x3FF6A000)>>3)&1),
                (int)((REG_READ(0x3FF6A000)>>2)&1));

  // Step 6: walk the TX descriptor ring and fix any DES3 (next descriptor
  // pointer) that points outside the descriptor ring (into heap/lwIP buffers).
  // The last descriptor's DES3 must wrap back to TX_LIST_BASE.
  uint32_t tx_base = REG_READ(0x3FF69010);
  if (tx_base != 0) {
    // Find the last descriptor in the ring (DES3 points outside DRAM or wraps)
    uint32_t desc = tx_base;
    uint32_t prev = 0;
    int count = 0;
    while (count < 64) {
      volatile uint32_t* d = (volatile uint32_t*)desc;
      uint32_t des3 = d[3];
      if (des3 == tx_base) { // already a good ring
        Serial.printf("ETH: TX ring already good (%d descriptors)\n", count);
        break;
      }
      if (des3 < 0x3FF00000 || des3 > 0x3FFFFFFF) {
        // DES3 is null/invalid — this is the last descriptor, fix it to wrap
        Serial.printf("ETH: fixing TX ring end: desc@0x%08x DES3=0x%08x -> 0x%08x\n",
                      desc, des3, tx_base);
        d[3] = tx_base;
        Serial.printf("ETH: TX ring fixed (%d descriptors)\n", count + 1);
        break;
      }
      prev = desc;
      desc = des3;
      count++;
    }
  }
}

void loop() {
  // Watchdog: hard-reset via esp_restart() rather than ETH.end()/ETH.begin()
  // to avoid leaking the EMAC interrupt on repeated restarts.
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
    // MAC base 0x3FF6A000: +0=maccr, +0x24=emacdebug
    uint32_t mac_cr    = REG_READ(0x3FF6A000);
    uint32_t mac_debug = REG_READ(0x3FF6A024);  // emacdebug
    uint32_t mac_intr  = REG_READ(0x3FF6A038);  // MAC interrupt status
    // EMAC_EX: clkout_conf=+0, oscclk_conf=+4, clk_ctrl=+8, phyinf=+C
    // clk_ctrl bits: ext_en(0) int_en(1) rx_125_clk_en(2) mii_clk_tx_en(3) mii_clk_rx_en(4) clk_en(5)
    // phyinf bits[15:13]=phy_intf_sel (need 4=RMII), oscclk bit20=clk_sel (need 1 for ext clock)
    uint32_t phyinf_val = REG_READ(0x3FF6980C);
    uint32_t oscclk_val = REG_READ(0x3FF69804);
    Serial.printf("  EMAC_EX: clkout=0x%08x  oscclk=0x%08x (clk_sel=%d)  clk_ctrl=0x%08x  phyinf=0x%08x (intf=%d need 4)\n",
                  REG_READ(0x3FF69800), oscclk_val, (int)((oscclk_val>>20)&1),
                  REG_READ(0x3FF69808), phyinf_val, (int)((phyinf_val>>13)&7));
    // MMC control at 0x3FF6A100: bit0=reset, bit1=stop_rollover, bit2=reset_on_read, bit3=freeze
    // Standard GMAC MMC TX counter layout (base 0x3FF6A100):
    //   +0x14=tx_octetcount_gb  +0x18=tx_framecount_gb  +0x64=tx_octetcount_g  +0x68=tx_framecount_g
    //   +0x20=tx_broadcastframe_g +0x24=tx_multicastframe_g +0x28=tx_64octets_gb
    Serial.printf("  MMC_CTRL=0x%08x\n", REG_READ(0x3FF6A100));
    Serial.printf("  MMC_TX: gb_frames=%08x gb_bytes=%08x g_bytes=%08x g_frames=%08x\n",
                  REG_READ(0x3FF6A118), REG_READ(0x3FF6A114),
                  REG_READ(0x3FF6A164), REG_READ(0x3FF6A168));
    Serial.printf("  MMC_TX: bcast_g=%08x mcast_g=%08x 64oct=%08x\n",
                  REG_READ(0x3FF6A120), REG_READ(0x3FF6A124), REG_READ(0x3FF6A128));
    // Standard GMAC MMC RX counter layout (base 0x3FF6A100+0x80=0x3FF6A180):
    //   +0x88=rx_framecount_gb +0x84=rx_octetcount_gb +0xD4=rx_broadcastframe_g
    //   +0xC0=rx_crc_err +0xD8=rx_octetcount_g +0xDC=rx_framecount_g
    Serial.printf("  MMC_RX: gb_frames=%08x gb_bytes=%08x g_bytes=%08x g_frames=%08x crc=%08x\n",
                  REG_READ(0x3FF6A188), REG_READ(0x3FF6A184),
                  REG_READ(0x3FF6A1D8), REG_READ(0x3FF6A1DC), REG_READ(0x3FF6A1C0));
    // emacdebug correct bit layout (from emac_mac_struct.h):
    //   [0]=macrpes, [2:1]=macrffcs, [4]=mtlrfwcas, [6:5]=mtlrfrcs, [9:8]=mtlrffls
    //   [16]=mactpes(TX proto eng), [18:17]=mactfcs(TX frame ctrl), [19]=mactp(TX pause)
    //   [21:20]=mtltfrcs(TX FIFO rd ctrl: 0=idle,1=read,2=wait_status,3=flush)
    //   [22]=mtltfwcs(TX FIFO wr ctrl active), [24]=mtltfnes(TX FIFO not empty), [25]=mtltsffs
    Serial.printf("  MAC_DEBUG=0x%08x (TX_proto=%d TX_fc=%d TX_fifo_rd=%d TX_fifo_ne=%d TX_fifo_wr=%d)\n",
                  mac_debug,
                  (int)((mac_debug >> 16) & 1),   // mactpes: TX MAC protocol engine active
                  (int)((mac_debug >> 17) & 3),   // mactfcs: TX frame ctrl state (0=idle,1=wait,2=pause,3=xfer)
                  (int)((mac_debug >> 20) & 3),   // mtltfrcs: TX FIFO read ctrl state
                  (int)((mac_debug >> 24) & 1),   // mtltfnes: TX FIFO not empty
                  (int)((mac_debug >> 22) & 1));  // mtltfwcs: TX FIFO write ctrl active
    Serial.printf("  MAC_INTR=0x%08x  DMA_INTR_EN=0x%08x\n", mac_intr, REG_READ(0x3FF6901C));
    Serial.printf("  DMA TX_STATE=%d  OP_MODE=0x%08x  TX_LIST=0x%08x  TX_DESC=0x%08x  TX_BUF=0x%08x\n",
                  (int)((dma_status >> 20) & 0x7),
                  dma_op_mode,
                  dma_tx_list, dma_tx_desc, dma_tx_buf);
    Serial.printf("  DMA STATUS=0x%08x  RX_LIST=0x%08x  RX_DESC=0x%08x\n",
                  dma_status, dma_rx_list, dma_rx_desc);
    Serial.printf("  DMA MISSED_FRAMES=0x%08x  CUR_HOST_TX=0x%08x  CUR_HOST_RX=0x%08x\n",
                  REG_READ(0x3FF69020), REG_READ(0x3FF69054), REG_READ(0x3FF69058));
    // DMA op_mode bits: bit1=SR(rx_run), bit13=ST(tx_run), bit20=flush_tx_fifo, bit21=tx_store_fwd
    Serial.printf("  MAC_CR=0x%08x (TX=%d RX=%d DM=%d FES=%d)  OP flush=%d sfwd=%d ST=%d SR=%d\n",
                  mac_cr,
                  (int)((mac_cr >> 3) & 1),       // tx enable
                  (int)((mac_cr >> 2) & 1),       // rx enable
                  (int)((mac_cr >> 11) & 1),      // duplex
                  (int)((mac_cr >> 14) & 1),      // fast eth speed
                  (int)((dma_op_mode >> 20) & 1), // flush_tx_fifo
                  (int)((dma_op_mode >> 21) & 1), // tx_store_forward
                  (int)((dma_op_mode >> 13) & 1), // ST: DMA TX run
                  (int)((dma_op_mode >>  1) & 1));// SR: DMA RX run
    // If MAC TX is not enabled, force-enable it and kick the DMA TX poll demand.
    // This tells us whether MAC_CR TX=0 is the reason TX never fires.
    if (!((mac_cr >> 3) & 1)) {
      Serial.println("  *** MAC TX not enabled! Force-enabling MAC_CR bit3 + DMA TX poll demand ***");
      REG_SET_BIT(0x3FF6A000, BIT(3));  // gmacconfig.tx = 1
      REG_WRITE(0x3FF69004, 1);         // dmatxpolldemand = 1 (resume)
      Serial.printf("  MAC_CR after fix=0x%08x\n", REG_READ(0x3FF6A000));
    }
    Serial.printf("  DPORT: WIFI_CLK_EN=0x%08x  CORE_RST=0x%08x\n",
                  DPORT_READ_PERI_REG(0x3FF000CC), DPORT_READ_PERI_REG(0x3FF0D0D0));
    // MAC Timestamp control at 0x3FF6A700: bit0=ts_enable, bit8=ts_all_frames
    // If bit0=1 the DMA writes timestamps to TDES4/TDES5 (words 4+5 of each TX descriptor).
    // Standard 4-word descriptors don't have these words — DMA corrupts adjacent memory.
    Serial.printf("  MAC_TS_CTRL=0x%08x  (bit0=ts_en bit8=all_frames)\n",
                  REG_READ(0x3FF6A700));

    if (dma_tx_desc >= 0x3FF00000 && dma_tx_desc <= 0x3FFFFFFF) {
      volatile uint32_t* d = (volatile uint32_t*)dma_tx_desc;
      Serial.printf("  TX_DESC @0x%08x  DES0=0x%08x DES1=0x%08x DES2=0x%08x DES3=0x%08x OWN=%d\n",
                    dma_tx_desc, d[0], d[1], d[2], d[3], (int)(d[0] >> 31));
    }
    // Check TX[0] for TDES4/TDES5 (timestamp words — written by DMA if timestamping enabled)
    if (dma_tx_list >= 0x3FF00000 && dma_tx_list <= 0x3FFFFFFF) {
      volatile uint32_t* d = (volatile uint32_t*)dma_tx_list;
      Serial.printf("  TX[0] DES0..5: %08x %08x %08x %08x | %08x %08x\n",
                    d[0], d[1], d[2], d[3], d[4], d[5]);
    }
    // Dump all TX descriptors once (only on first poll)
    static bool tx_ring_dumped = false;
    if (!tx_ring_dumped && dma_tx_list >= 0x3FF00000 && dma_tx_list <= 0x3FFFFFFF) {
      tx_ring_dumped = true;
      // Dump first 32 bytes of TX[0] buffer (should be Ethernet frame: dst MAC, src MAC, ethertype...)
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
        Serial.printf("    TX[%2d] @0x%08x  DES0=0x%08x DES1=0x%08x DES2=0x%08x DES3=0x%08x\n",
                      i, desc, d[0], d[1], d[2], d[3]);
        if (d[3] == dma_tx_list) { Serial.println("    (ring end)"); break; }
        desc = d[3];
      }
    }
    // Dump all RX descriptors once (only on first poll)
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
    // Dump lwIP netif list + DHCP state
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
