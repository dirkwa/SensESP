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
      REG_SET_FIELD(IO_MUX_GPIO0_REG, MCU_SEL, FUNC_GPIO0_CLK_OUT1);
      PIN_INPUT_ENABLE(IO_MUX_GPIO0_REG);
      CLEAR_PERI_REG_MASK(IO_MUX_GPIO0_REG, FUN_PD);
      CLEAR_PERI_REG_MASK(IO_MUX_GPIO0_REG, FUN_PU);
      Serial.printf("ETH: Started (IOMUX confirmed, MCU_SEL=%d)\n",
                    (int)REG_GET_FIELD(IO_MUX_GPIO0_REG, MCU_SEL));
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
  REG_SET_FIELD(IO_MUX_GPIO0_REG, MCU_SEL, FUNC_GPIO0_CLK_OUT1);
  PIN_INPUT_ENABLE(IO_MUX_GPIO0_REG);
  CLEAR_PERI_REG_MASK(IO_MUX_GPIO0_REG, FUN_PD);
  CLEAR_PERI_REG_MASK(IO_MUX_GPIO0_REG, FUN_PU);
  Serial.printf("ETH: GPIO0 IOMUX set  MCU_SEL=%d\n",
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
  Serial.printf("ETH: DMA BUS_MODE=0x%08x  OP_MODE=0x%08x  STATUS=0x%08x\n",
                REG_READ(0x3FF69000), REG_READ(0x3FF69018), REG_READ(0x3FF69014));

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
    uint32_t dma_tx_list = REG_READ(0x3FF69010);
    uint32_t dma_tx_desc = REG_READ(0x3FF69048);  // dmatxcurrdesc
    uint32_t dma_tx_buf  = REG_READ(0x3FF69050);  // dmatxcurraddr_buf
    uint32_t dma_rx_list = REG_READ(0x3FF6900C);
    uint32_t dma_rx_desc = REG_READ(0x3FF6904C);  // dmarxcurrdesc
    // MAC base 0x3FF6A000: +0=maccr (tx=bit3,rx=bit2), +0x10=gmiiaddr, +0x18=flowctrl
    uint32_t mac_cr   = REG_READ(0x3FF6A000);
    uint32_t mac_ff   = REG_READ(0x3FF6A004);  // frame filter
    uint32_t mac_flow = REG_READ(0x3FF6A018);
    // EMAC_EX: clkout_conf=+0, oscclk_conf=+4, clk_ctrl=+8, phyinf=+C
    Serial.printf("  EMAC_EX: clkout=0x%08x  oscclk=0x%08x  clk_ctrl=0x%08x\n",
                  REG_READ(0x3FF69800), REG_READ(0x3FF69804), REG_READ(0x3FF69808));
    // MAC MMC TX counters: base 0x3FF6A100
    Serial.printf("  MMC_TX: good_frames=0x%08x  good_bytes=0x%08x  errors=0x%08x\n",
                  REG_READ(0x3FF6A114), REG_READ(0x3FF6A118), REG_READ(0x3FF6A108));
    Serial.printf("  DMA TX_STATE=%d  ST=%d  TX_LIST=0x%08x  TX_DESC=0x%08x  TX_BUF=0x%08x\n",
                  (int)((dma_status >> 20) & 0x7),
                  (int)((REG_READ(0x3FF69018) >> 13) & 1),
                  dma_tx_list, dma_tx_desc, dma_tx_buf);
    Serial.printf("  DMA STATUS=0x%08x  RX_LIST=0x%08x  RX_DESC=0x%08x\n",
                  dma_status, dma_rx_list, dma_rx_desc);
    Serial.printf("  MAC_CR=0x%08x (TX=%d RX=%d DM=%d FES=%d)\n",
                  mac_cr,
                  (int)((mac_cr >> 3) & 1),   // tx enable
                  (int)((mac_cr >> 2) & 1),   // rx enable
                  (int)((mac_cr >> 11) & 1),  // duplex
                  (int)((mac_cr >> 14) & 1)); // fast eth speed
    if (dma_tx_desc >= 0x3FF00000 && dma_tx_desc <= 0x3FFFFFFF) {
      volatile uint32_t* d = (volatile uint32_t*)dma_tx_desc;
      Serial.printf("  TX_DESC @0x%08x  DES0=0x%08x DES1=0x%08x DES2=0x%08x DES3=0x%08x OWN=%d\n",
                    dma_tx_desc, d[0], d[1], d[2], d[3], (int)(d[0] >> 31));
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
