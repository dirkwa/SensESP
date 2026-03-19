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
      // Correct MAC register offsets (base 0x3FF6A000).
      // struct emac_mac_dev_t layout (confirmed from emac_mac_struct.h):
      //   +0x000 = gmacconfig  (TX bit3, RX bit2, duplex bit11, FES bit14, loopback bit12)
      //   +0x004 = gmacff
      //   +0x008 = reserved_1008
      //   +0x00C = reserved_100c
      //   +0x010 = emacgmiiaddr (MDIO addr)
      //   +0x014 = emacmiidata
      //   +0x018 = gmacfc      (flow control)
      //   +0x01C = reserved_101c
      //   +0x020 = reserved_1020
      //   +0x024 = emacdebug   (MAC debug state machine)
      //   +0x028 = pmt_rwuffr
      //   +0x02C = pmt_csr     (pwrdwn bit0, mgkpkten bit1, rwkpkten bit2)
      //   +0x030 = gmaclpi_crs (pls bit17 = link status seen by MAC TX)
      //   +0x034 = gmaclpitimerscontrol
      //   +0x038 = emacints
      //   +0x03C = emacintmask
      //   +0x040 = emacaddr0high, +0x044 = emacaddr0low
      #define GMACCONFIG   0x3FF6A000
      #define GMACFC       0x3FF6A018
      #define EMACDEBUG    0x3FF6A024
      #define PMT_RWUFFR   0x3FF6A028
      #define PMT_CSR      0x3FF6A02C
      #define GMACLPI_CRS  0x3FF6A030
      #define EMACADDR0H   0x3FF6A040
      #define EMACADDR0L   0x3FF6A044
      #ifndef EMAC_GMIIADDR
      #define EMAC_GMIIADDR 0x3FF6A010
      #define EMAC_GMIIDATA 0x3FF6A014
      #endif

      uint32_t mac_cr = REG_READ(GMACCONFIG);
      Serial.printf("ETH: Link up  TX_LIST=0x%08x  RX_LIST=0x%08x  OP_MODE=0x%08x\n",
                    REG_READ(0x3FF69010), REG_READ(0x3FF6900C), REG_READ(0x3FF69018));
      Serial.printf("ETH: gmacconfig=0x%08x (TX=%d RX=%d DM=%d FES=%d)  FLOW=0x%08x\n",
                    mac_cr,
                    (int)((mac_cr >> 3) & 1),
                    (int)((mac_cr >> 2) & 1),
                    (int)((mac_cr >> 11) & 1),
                    (int)((mac_cr >> 14) & 1),
                    REG_READ(GMACFC));
      // LPI control register: pls (bit17) = link status. If 0, MAC won't TX.
      uint32_t lpi = REG_READ(GMACLPI_CRS);
      Serial.printf("ETH: gmaclpi_crs=0x%08x  pls=%d (1=link ok, 0=MAC thinks link down)\n",
                    lpi, (int)((lpi >> 17) & 1));

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

      // pmt_csr (MAC+0x02C = 0x3FF6A02C): if pwrdwn (bit0) is set, MAC RX drops
      // frames and TX is gated. Clear by writing 0 (disables mgkpkten/rwkpkten too).
      {
        uint32_t pmt = REG_READ(PMT_CSR);
        Serial.printf("ETH: pmt_csr=0x%08x  PD=%d MPFE=%d WFE=%d MPR=%d WFR=%d\n",
                      pmt, (int)(pmt&1), (int)((pmt>>1)&1), (int)((pmt>>2)&1),
                      (int)((pmt>>4)&1), (int)((pmt>>8)&1));
        if (pmt & 1) {
          Serial.printf("ETH: MAC power-down active! Clearing PMT_CSR PD bit.\n");
          REG_WRITE(PMT_CSR, 0x00000000);
          delayMicroseconds(100);
          Serial.printf("ETH: pmt_csr after clear=0x%08x\n", REG_READ(PMT_CSR));
        }
      }

      // Set pls=1 in gmaclpi_crs so MAC TX engine sees the link as up.
      // Also ensure gmacconfig has FES=1 (100M) and DM=1 (full-duplex).
      {
        // Set pls=1 (bit17): tell MAC TX the link is up
        REG_SET_BIT(GMACLPI_CRS, BIT(17));
        Serial.printf("ETH: gmaclpi_crs after pls=1: 0x%08x  pls=%d\n",
                      REG_READ(GMACLPI_CRS), (int)((REG_READ(GMACLPI_CRS)>>17)&1));

        // Fix gmacconfig speed/duplex from PHY autoneg result
        uint32_t cr = REG_READ(GMACCONFIG);
        // LAN8720 PSCSR speed_ind: 1=10H, 5=100H, 6=100F, 2=10F
        bool is_100  = (((pscsr>>2)&7)==5 || ((pscsr>>2)&7)==6);
        bool is_full = (((pscsr>>2)&7)==6 || ((pscsr>>2)&7)==2);
        if (is_100)  cr |= BIT(14); else cr &= ~BIT(14);
        if (is_full) cr |= BIT(11); else cr &= ~BIT(11);
        REG_WRITE(GMACCONFIG, cr);
        delayMicroseconds(100);
        Serial.printf("ETH: gmacconfig after fix=0x%08x (FES=%d DM=%d TX=%d RX=%d)\n",
                      REG_READ(GMACCONFIG),
                      (int)((REG_READ(GMACCONFIG)>>14)&1),
                      (int)((REG_READ(GMACCONFIG)>>11)&1),
                      (int)((REG_READ(GMACCONFIG)>>3)&1),
                      (int)((REG_READ(GMACCONFIG)>>2)&1));
      }

      // Clear MAC_TS_CTRL — IDF v5 sets TSENALL (bit13) during esp_eth_start().
      Serial.printf("ETH: MAC_TS_CTRL before=0x%08x\n", REG_READ(0x3FF6A700));
      REG_WRITE(0x3FF6A700, 0x00000000);
      Serial.printf("ETH: MAC_TS_CTRL after =0x%08x  TX_STATE=%d  OP_MODE=0x%08x\n",
                    REG_READ(0x3FF6A700),
                    (int)((REG_READ(0x3FF69014)>>20)&7),
                    REG_READ(0x3FF69018));
      Serial.printf("ETH: ex_clk_ctrl=0x%08x\n", REG_READ(0x3FF69808));

      // Injected-frame loopback test.
      // The DMA already consumed the IDF-queued frames (all OWN=0) before this
      // handler runs, so poll demand with LM=1 finds no OWN=1 descriptors and
      // never actually sends anything. We must inject a fresh frame ourselves.
      //
      // Strategy: stop DMA, build a minimal 64-byte Ethernet frame in a static
      // buffer, point TX[0] at it with OWN=1 and FS+LS set, then restart with
      // LM=1 (MAC loopback). The DMA sends it and the MAC loops it back to RX.
      // If MMC_TX increments, MAC TX works. If not, it's truly broken.
      {
        static uint8_t lb_buf[64] __attribute__((aligned(4)));
        // Build a minimal broadcast frame: dst=FF:FF:FF:FF:FF:FF, src=our MAC,
        // EtherType=0x0800, payload=zeros.  CRC appended by MAC (DC=0).
        memset(lb_buf, 0, sizeof(lb_buf));
        // dst MAC
        lb_buf[0]=0xff; lb_buf[1]=0xff; lb_buf[2]=0xff;
        lb_buf[3]=0xff; lb_buf[4]=0xff; lb_buf[5]=0xff;
        // src MAC (read from hardware register)
        uint32_t mac_lo = REG_READ(EMACADDR0L);  // emacaddr0low
        uint32_t mac_hi = REG_READ(EMACADDR0H);  // emacaddr0high
        lb_buf[6]  = (mac_lo >>  0) & 0xff;
        lb_buf[7]  = (mac_lo >>  8) & 0xff;
        lb_buf[8]  = (mac_lo >> 16) & 0xff;
        lb_buf[9]  = (mac_lo >> 24) & 0xff;
        lb_buf[10] = (mac_hi >>  0) & 0xff;
        lb_buf[11] = (mac_hi >>  8) & 0xff;
        lb_buf[12] = 0x08; lb_buf[13] = 0x00;  // EtherType IPv4

        uint32_t tx_list = REG_READ(0x3FF69010);
        Serial.printf("ETH: inject loopback -- TX_LIST=0x%08x  buf=0x%08x\n",
                      tx_list, (uint32_t)lb_buf);

        // Stop TX DMA
        REG_CLR_BIT(0x3FF69018, BIT(13));  // ST=0
        uint32_t t0 = millis();
        while (((REG_READ(0x3FF69014)>>20)&7) != 0 && millis()-t0 < 20) {}
        // Flush TX FIFO
        REG_SET_BIT(0x3FF69018, BIT(20));
        t0 = millis();
        while ((REG_READ(0x3FF69018) & BIT(20)) && millis()-t0 < 20) {}

        // Overwrite TX[0] descriptor to point at our buffer.
        // Keep the same 8-word enhanced descriptor layout.
        // TDES0: OWN=1, IC=0, LS=1, FS=1, DC=0, TTSE=0, all others 0
        // TDES1: TBS1=64, TBS2=0
        // TDES2: buf1 address
        // TDES3: next descriptor (TX[1])
        // TDES4..7: zero (no timestamp)
        if (tx_list >= 0x3FF00000 && tx_list <= 0x3FFFFFFF) {
          volatile uint32_t* d = (volatile uint32_t*)tx_list;
          uint32_t next = d[3];  // preserve original next-desc pointer
          d[4] = 0; d[5] = 0; d[6] = 0; d[7] = 0;  // clear timestamp words
          d[2] = (uint32_t)lb_buf;
          d[1] = 64;  // TBS1=64, TBS2=0
          d[3] = next;
          // Write TDES0 last: OWN=1, LS=1(bit29), FS=1(bit28)
          d[0] = 0x80000000 | BIT(29) | BIT(28);  // OWN+LS+FS, DC=0, TTSE=0
          Serial.printf("ETH: TX[0] injected: DES0=0x%08x DES1=0x%08x DES2=0x%08x DES3=0x%08x\n",
                        d[0], d[1], d[2], d[3]);
        }

        // Clear stale MMC counts
        (void)REG_READ(0x3FF6A118);
        (void)REG_READ(0x3FF6A150);
        (void)REG_READ(0x3FF6A168);
        (void)REG_READ(0x3FF6A16C);
        (void)REG_READ(0x3FF6A178);

        // Send frame for real (no loopback) — reset DMA to ring start, restart.
        // gmacconfig LM=bit12; ensure it's 0 (real TX, not loopback).
        // Writing TX_LIST while ST=0 resets the DMA fetch pointer to TX[0].
        REG_CLR_BIT(GMACCONFIG, BIT(12));  // LM=0 (no loopback — real TX)
        REG_WRITE(0x3FF69010, tx_list);    // reset DMA pointer to TX[0]
        REG_SET_BIT(0x3FF69018, BIT(13));  // ST=1
        REG_WRITE(0x3FF69004, 1);           // poll demand

        uint32_t t_lb = millis();
        uint32_t dbg_peak = 0, stall_count = 0, dbg_stall = 0;
        while (REG_READ(0x3FF6A118) == 0 && (millis() - t_lb < 1000)) {
          uint32_t d = REG_READ(EMACDEBUG);
          if (d > dbg_peak) dbg_peak = d;
          if (((d >> 20) & 3) == 2) { stall_count++; dbg_stall = d; }
          delayMicroseconds(10);
        }

        uint32_t mmc_gb  = REG_READ(0x3FF6A118);
        uint32_t mmc_ufl = REG_READ(0x3FF6A150);
        uint32_t mmc_jab = REG_READ(0x3FF6A168);
        uint32_t mmc_g   = REG_READ(0x3FF6A16C);
        uint32_t mmc_car = REG_READ(0x3FF6A178);
        uint32_t dbg_end = REG_READ(EMACDEBUG);

        // Read back TX[0] descriptor status
        uint32_t des0_after = 0;
        if (tx_list >= 0x3FF00000 && tx_list <= 0x3FFFFFFF) {
          volatile uint32_t* d = (volatile uint32_t*)tx_list;
          des0_after = d[0];
        }

        Serial.printf("ETH: inject loopback done  MMC_GB=%u  MMC_GOOD=%u  DBG_peak=0x%08x\n",
                      mmc_gb, mmc_g, dbg_peak);
        Serial.printf("ETH:   peak(tpes=%d tfc=%d mtltfrcs=%d fifo_ne=%d)  stall=%u\n",
                      (int)((dbg_peak>>16)&1), (int)((dbg_peak>>17)&3),
                      (int)((dbg_peak>>20)&3), (int)((dbg_peak>>24)&1),
                      stall_count);
        Serial.printf("ETH:   TX[0] DES0 after=0x%08x  OWN=%d  ErrSummary=%d  Deferred=%d\n",
                      des0_after, (int)(des0_after>>31),
                      (int)((des0_after>>15)&1), (int)(des0_after&1));
        Serial.printf("ETH:   TDES0 status bits: UflowErr=%d ExcDef=%d CollCnt=%d VLan=%d ExcColl=%d LateColl=%d NoCarr=%d LossCarr=%d\n",
                      (int)((des0_after>>1)&1),(int)((des0_after>>2)&1),(int)((des0_after>>3)&0xf),
                      (int)((des0_after>>7)&1),(int)((des0_after>>8)&1),(int)((des0_after>>9)&1),
                      (int)((des0_after>>10)&1),(int)((des0_after>>11)&1));
        Serial.printf("ETH:   MMC errors: underflow=%u jabber=%u carrier=%u\n",
                      mmc_ufl, mmc_jab, mmc_car);
        Serial.printf("ETH: %s\n", (mmc_gb > 0 || mmc_g > 0)
          ? "LOOPBACK OK: MAC TX works"
          : "LOOPBACK FAIL: MAC TX not completing frames");

      // Raw MAC register scan: dump 0x3FF6A000..0x3FF6A060 to confirm actual offsets.
      Serial.printf("ETH: MAC raw reg scan (0x3FF6A000-0x3FF6A060):\n");
      for (int off = 0; off <= 0x60; off += 4) {
        uint32_t v = REG_READ(0x3FF6A000 + off);
        Serial.printf("  MAC+0x%03x = 0x%08x\n", off, v);
      }
      // Also scan MAC TS area (0x3FF6A700)
      Serial.printf("ETH: MAC TS area:\n");
      for (int off = 0x700; off <= 0x720; off += 4) {
        uint32_t v = REG_READ(0x3FF6A000 + off);
        Serial.printf("  MAC+0x%03x = 0x%08x\n", off, v);
      }
      }  // end inject block

      // Poll MAC_DEBUG, TX_BUF and MMC_TX rapidly for 5s to catch MAC TX activity.
      // MAC_DEBUG bits [18:17] = mactfcs (TX frame controller state),
      //   bit16=mactpes (TX protocol engine active), bit24=mtltfnes (FIFO not empty).
      // MMC_TX gb_frames at 0x3FF6A118 increments on each transmitted frame.
      Serial.printf("ETH: polling MAC_DEBUG+TX_BUF for 5s (ST=%d SR=%d)\n",
                    (int)((REG_READ(0x3FF69018) >> 13) & 1),
                    (int)((REG_READ(0x3FF69018) >> 1) & 1));
      {
        uint32_t last_txbuf = 0, last_status = 0, last_dbg = 0, last_pmt = 0, last_lpi = 0, last_mmctx = 0;
        uint32_t t_poll = millis();
        while (millis() - t_poll < 5000) {
          uint32_t txbuf  = REG_READ(0x3FF69050);
          uint32_t status = REG_READ(0x3FF69014);
          uint32_t dbg    = REG_READ(0x3FF6A024);  // emacdebug (confirmed)
          uint32_t pmt    = REG_READ(0x3FF6A02C);  // pmt_csr (confirmed)
          uint32_t lpi    = REG_READ(0x3FF6A030);  // gmaclpi_crs (confirmed)
          uint32_t mmctx  = REG_READ(0x3FF6A118);
          if (txbuf != last_txbuf || status != last_status || dbg != last_dbg ||
              pmt != last_pmt || lpi != last_lpi || mmctx != last_mmctx) {
            Serial.printf("  t+%lums  TX_BUF=0x%08x  TX_ST=%d  DBG=0x%08x  PMT=0x%08x(pd=%d)  LPI=0x%08x(pls=%d)  MMC=%u\n",
                          millis() - t_poll, txbuf, (int)((status >> 20) & 7),
                          dbg, pmt, (int)(pmt&1), lpi, (int)((lpi>>17)&1), mmctx);
            last_txbuf = txbuf; last_status = status; last_dbg = dbg;
            last_pmt = pmt; last_lpi = lpi; last_mmctx = mmctx;
          }
          delayMicroseconds(100);
        }
        Serial.printf("ETH: poll done  TX_BUF=0x%08x  MMC_TX=%u  DBG=0x%08x  PMT=0x%08x  LPI=0x%08x\n",
                      REG_READ(0x3FF69050), REG_READ(0x3FF6A118),
                      REG_READ(0x3FF6A024), REG_READ(0x3FF6A02C), REG_READ(0x3FF6A030));
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

  // PHY reset: GPIO17 LOW also disables the 50MHz oscillator, which removes
  // the RMII REF_CLK from GPIO0. Re-apply the IOMUX after the pulse.
  digitalWrite(PHY_RST_PIN, LOW);
  delay(10);
  digitalWrite(PHY_RST_PIN, HIGH);
  // Re-apply GPIO0 IOMUX after oscillator restart (clock was absent during reset)
  esp_gpio_revoke(BIT64(GPIO_NUM_0));
  REG_SET_FIELD(IO_MUX_GPIO0_REG, MCU_SEL, FUNC_GPIO0_EMAC_TX_CLK);
  PIN_INPUT_ENABLE(IO_MUX_GPIO0_REG);
  CLEAR_PERI_REG_MASK(IO_MUX_GPIO0_REG, FUN_PD);
  CLEAR_PERI_REG_MASK(IO_MUX_GPIO0_REG, FUN_PU);
  delay(300);  // LAN8720 needs >100ms after nRST=HIGH before MDIO is ready
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
  Serial.printf("ETH: gmacconfig=0x%08x (TX=%d RX=%d) after begin\n",
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
    uint32_t mac_cr    = REG_READ(0x3FF6A000);  // gmacconfig
    uint32_t mac_debug = REG_READ(0x3FF6A024);  // emacdebug (confirmed from struct)
    uint32_t mac_intr  = REG_READ(0x3FF6A038);  // emacints (confirmed from struct)
    uint32_t phyinf_val = REG_READ(0x3FF6980C);
    uint32_t oscclk_val = REG_READ(0x3FF69804);
    Serial.printf("  EMAC_EX: clkout=0x%08x  oscclk=0x%08x (clk_sel=%d)  clk_ctrl=0x%08x  phyinf=0x%08x (intf=%d need 4)\n",
                  REG_READ(0x3FF69800), oscclk_val, (int)((oscclk_val>>24)&1),
                  REG_READ(0x3FF69808), phyinf_val, (int)((phyinf_val>>13)&7));
    Serial.printf("  MMC_CTRL=0x%08x\n", REG_READ(0x3FF6A100));
    {
      // MMC registers are read-clear: capture once then print.
      uint32_t mmc_gb    = REG_READ(0x3FF6A118);
      uint32_t mmc_gbB   = REG_READ(0x3FF6A114);
      uint32_t mmc_gB    = REG_READ(0x3FF6A164);
      uint32_t mmc_g     = REG_READ(0x3FF6A16C);
      uint32_t mmc_bcast = REG_READ(0x3FF6A120);
      uint32_t mmc_mcast = REG_READ(0x3FF6A124);
      uint32_t mmc_64oct = REG_READ(0x3FF6A128);
      uint32_t mmc_uflow = REG_READ(0x3FF6A150);
      uint32_t mmc_jab   = REG_READ(0x3FF6A168);
      uint32_t mmc_car   = REG_READ(0x3FF6A178);
      uint32_t mtl_dbg   = REG_READ(0x3FF6A908);
      Serial.printf("  MMC_TX: gb_frames=%08x gb_bytes=%08x g_bytes=%08x g_frames=%08x\n",
                    mmc_gb, mmc_gbB, mmc_gB, mmc_g);
      Serial.printf("  MMC_TX: bcast_g=%08x mcast_g=%08x 64oct=%08x\n",
                    mmc_bcast, mmc_mcast, mmc_64oct);
      Serial.printf("  MMC_TX errors: underflow=%08x jabber=%08x carrier=%08x\n",
                    mmc_uflow, mmc_jab, mmc_car);
      Serial.printf("  MTL_TXFIFO_DBG=0x%08x (nempty=%d aempty=%d full=%d)\n",
                    mtl_dbg, (int)(mtl_dbg & 1), (int)((mtl_dbg>>1)&1), (int)((mtl_dbg>>4)&1));
    }
    Serial.printf("  MMC_RX: gb_frames=%08x gb_bytes=%08x g_bytes=%08x g_frames=%08x crc=%08x\n",
                  REG_READ(0x3FF6A188), REG_READ(0x3FF6A184),
                  REG_READ(0x3FF6A1D8), REG_READ(0x3FF6A1DC), REG_READ(0x3FF6A1C0));
    Serial.printf("  MAC_DEBUG=0x%08x (TX_proto=%d TX_fc=%d TX_fifo_rd=%d TX_fifo_ne=%d TX_fifo_wr=%d)\n",
                  mac_debug,
                  (int)((mac_debug >> 16) & 1),
                  (int)((mac_debug >> 17) & 3),
                  (int)((mac_debug >> 20) & 7),   // mtltfrcs 3-bit [22:20]
                  (int)((mac_debug >> 24) & 1),
                  (int)((mac_debug >> 22) & 1));
    // pmt_csr and gmaclpi_crs (confirmed offsets from emac_mac_struct.h)
    uint32_t pmt_val = REG_READ(0x3FF6A02C);  // pmt_csr: pwrdwn=bit0, mgkpkten=bit1, rwkpkten=bit2
    uint32_t lpi_crs = REG_READ(0x3FF6A030);  // gmaclpi_crs: pls=bit17
    Serial.printf("  pmt_csr=0x%08x  pwrdwn=%d  gmaclpi_crs=0x%08x  pls=%d (1=MAC sees link up)\n",
                  pmt_val, (int)(pmt_val&1), lpi_crs, (int)((lpi_crs>>17)&1));
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
    Serial.printf("  gmacconfig=0x%08x (TX=%d RX=%d DM=%d FES=%d)  OP flush=%d sfwd=%d ST=%d SR=%d\n",
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
