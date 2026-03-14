// Minimal Aptinex IsolPoE Ethernet test — no SensESP framework.
//
// GPIO17 = oscillator enable (4.7k pulldown → off by default).
// ETH_CLK_MODE = ETH_CLOCK_GPIO0_IN: 50MHz clock from oscillator into GPIO0.
// GPIO0 is a boot strapping pin — briefly driven LOW during boot.
//
// IDF v5 change: power pin is used as PHY nRST (pulsed LOW then HIGH).
// On this board GPIO17 is oscillator enable — pulsing it LOW kills the clock
// during PHY reset, preventing LAN8720 RMII init. Pass power=-1 and drive
// GPIO17 HIGH manually before ETH.begin() so the oscillator is always on.

#include <ETH.h>
#include <WiFi.h>
#include "driver/gpio.h"
#include "esp_private/esp_gpio_reserve.h"   // esp_gpio_revoke()
#include "soc/io_mux_reg.h"                 // FUNC_GPIO0_EMAC_TX_CLK, IO_MUX_GPIO0_REG
#include "soc/emac_ext_struct.h"             // EMAC_EXT, emac_ext_dev_t
#include "soc/gpio_reg.h"                    // GPIO_IN_REG

#define OSC_EN_PIN 17

static bool eth_connected = false;

void onEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH: Started");
      ETH.setHostname("eth-test");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH: Link up");
      break;
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
  Serial.println("\nAptinex IsolPoE ETH test starting...");

  // Enable oscillator: GPIO17 HIGH drives the 50MHz oscillator on.
  // Do this BEFORE ETH.begin() so the LAN8720 has a stable RMII clock
  // throughout init. Pass power=-1 to ETH.begin() so IDF v5 doesn't
  // pulse GPIO17 LOW (as PHY reset) and kill the oscillator mid-init.
  pinMode(OSC_EN_PIN, OUTPUT);
  digitalWrite(OSC_EN_PIN, HIGH);
  delay(10);  // let oscillator stabilise
  Serial.println("ETH: GPIO17 HIGH (oscillator on)");

  // Pre-configure GPIO0 as EMAC RMII clock input BEFORE ETH.begin().
  // IDF v5 marks GPIO0 as reserved (strapping pin) and perimanClearPinBus()
  // inside ETH.begin() may reset it. By setting IOMUX here AND after, we
  // ensure the clock is present when esp_eth_start() launches the DMA.
  esp_gpio_revoke(BIT64(GPIO_NUM_0));
  REG_SET_FIELD(IO_MUX_GPIO0_REG, MCU_SEL, FUNC_GPIO0_EMAC_TX_CLK);
  PIN_INPUT_ENABLE(IO_MUX_GPIO0_REG);
  CLEAR_PERI_REG_MASK(IO_MUX_GPIO0_REG, FUN_PD);
  CLEAR_PERI_REG_MASK(IO_MUX_GPIO0_REG, FUN_PU);
  Serial.printf("ETH: GPIO0 pre-configured  IO_MUX=0x%08x  MCU_SEL=%d\n",
                REG_READ(IO_MUX_GPIO0_REG),
                (int)REG_GET_FIELD(IO_MUX_GPIO0_REG, MCU_SEL));

  WiFi.mode(WIFI_OFF);
  Network.onEvent(onEvent);
  // power=-1: don't let IDF touch GPIO17
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO,
            -1, ETH_CLK_MODE);

  // Re-apply IOMUX after ETH.begin() in case perimanClearPinBus reset it.
  REG_SET_FIELD(IO_MUX_GPIO0_REG, MCU_SEL, FUNC_GPIO0_EMAC_TX_CLK);
  PIN_INPUT_ENABLE(IO_MUX_GPIO0_REG);
  CLEAR_PERI_REG_MASK(IO_MUX_GPIO0_REG, FUN_PD);
  CLEAR_PERI_REG_MASK(IO_MUX_GPIO0_REG, FUN_PU);
  Serial.printf("ETH: GPIO0 post-ETH.begin  IO_MUX=0x%08x  MCU_SEL=%d  GPIO17=%d\n",
                REG_READ(IO_MUX_GPIO0_REG),
                (int)REG_GET_FIELD(IO_MUX_GPIO0_REG, MCU_SEL),
                (int)digitalRead(OSC_EN_PIN));

  // Walk TX descriptor ring and clear OWN on any descriptor with invalid
  // buffer address (outside DRAM 0x3FF00000-0x3FFFFFFF). These are garbage
  // descriptors left from EMAC DMA running without a working clock at boot,
  // causing bus errors. Must stop TX DMA first.
  uint32_t tx_list = REG_READ(0x3FF69010);
  uint32_t opmode  = REG_READ(0x3FF69018);
  REG_WRITE(0x3FF69018, opmode & ~(1u << 13));  // stop TX DMA
  ets_delay_us(200);
  int fixed = 0;
  uint32_t desc = tx_list;
  for (int i = 0; i < 64; i++) {  // ring has at most 64 descriptors
    if (desc < 0x3FF00000 || desc > 0x3FFFFFFF) break;
    volatile uint32_t* d = (volatile uint32_t*)desc;
    uint32_t des0 = d[0];
    uint32_t des2 = d[2];  // buffer address
    uint32_t des3 = d[3];  // next descriptor
    bool own     = (des0 >> 31) & 1;
    bool buf_bad = des2 < 0x3FF00000 || des2 > 0x3FFFFFFF;
    if (own && buf_bad) {
      d[0] = des0 & ~0x80000000u;  // clear OWN
      fixed++;
    }
    if (des3 == tx_list || des3 < 0x3FF00000 || des3 > 0x3FFFFFFF) break;
    desc = des3;
  }
  // Restart TX DMA from ring base
  REG_WRITE(0x3FF69010, tx_list);
  REG_WRITE(0x3FF69018, opmode | (1u << 13));
  REG_WRITE(0x3FF69004, 1);  // poll demand
  Serial.printf("ETH: TX ring scrubbed (%d bad descriptors fixed), DMA restarted\n", fixed);
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last > 5000) {
    last = millis();
    Serial.printf("uptime=%lus  linked=%d  hasIP=%d  ip=%s  MAC=%s\n",
                  millis() / 1000,
                  (int)ETH.linkUp(),
                  (int)ETH.hasIP(),
                  ETH.localIP().toString().c_str(),
                  ETH.macAddress().c_str());

    // EMAC EXT clock registers — base address of EMAC_EXT peripheral
    // ex_clkout_conf @ +0x00, ex_oscclk_conf @ +0x04, ex_clk_ctrl @ +0x08, ex_phyinf_conf @ +0x0C
    uint32_t ext_base    = (uint32_t)&EMAC_EXT;
    uint32_t clkout_conf = REG_READ(ext_base + 0x00);
    uint32_t oscclk_conf = REG_READ(ext_base + 0x04);
    uint32_t clk_ctrl    = REG_READ(ext_base + 0x08);
    uint32_t phyinf_conf = REG_READ(ext_base + 0x0C);
    Serial.printf("  EMAC_EXT base=0x%08x\n", ext_base);
    Serial.printf("  ex_clkout=0x%08x  ex_oscclk=0x%08x  ex_clk_ctrl=0x%08x  ex_phyinf=0x%08x\n",
                  clkout_conf, oscclk_conf, clk_ctrl, phyinf_conf);
    // For ETH_CLOCK_GPIO0_IN: ext_en=1 int_en=0 clk_sel=1 phy_intf_sel=4
    Serial.printf("  ext_en=%d int_en=%d clk_sel=%d phy_intf_sel=%d (want 1,0,1,4)\n",
                  (int)(clk_ctrl & 1),
                  (int)((clk_ctrl >> 1) & 1),
                  (int)((oscclk_conf >> 24) & 1),
                  (int)((phyinf_conf >> 13) & 7));

    uint32_t dma_status   = REG_READ(0x3FF69014);
    uint32_t dma_tx_list  = REG_READ(0x3FF69010);  // TX desc list base addr (set at init)
    uint32_t dma_tx_curr  = REG_READ(0x3FF69050);  // current TX desc pointer
    Serial.printf("  DMA_STATUS=0x%08x  TX_STATE=%d  RX_STATE=%d\n",
                  dma_status,
                  (int)((dma_status >> 20) & 0x7),
                  (int)((dma_status >> 17) & 0x7));
    Serial.printf("  TX_LIST_BASE=0x%08x  TX_CURR_DESC=0x%08x\n",
                  dma_tx_list, dma_tx_curr);
    // Dump current TX desc AND ring base desc to compare
    auto dump_desc = [](const char* label, uint32_t addr) {
      if (addr >= 0x3FF00000 && addr <= 0x3FFFFFFF) {
        uint32_t d0 = *((volatile uint32_t*)(addr + 0));
        uint32_t d1 = *((volatile uint32_t*)(addr + 4));
        uint32_t d2 = *((volatile uint32_t*)(addr + 8));
        uint32_t d3 = *((volatile uint32_t*)(addr + 12));
        Serial.printf("  %s @0x%08x: DES0=0x%08x DES1=0x%08x DES2=0x%08x DES3=0x%08x OWN=%d\n",
                      label, addr, d0, d1, d2, d3, (int)(d0 >> 31));
      }
    };
    dump_desc("TX_CURR", dma_tx_curr);
    dump_desc("TX_BASE", dma_tx_list);

    // TX_CURR is in random heap; TX_BASE has a real descriptor (OWN=0, size=0x15e).
    // OWN=0 means driver hasn't handed it to DMA yet, OR DMA processed with error.
    // Force DMA TX back to ring base: stop ST, write list ptr, restart ST.
    static bool dma_reset_done = false;
    if (!dma_reset_done) {
      // Set OWN=1 on TX_BASE descriptor to hand it to DMA
      volatile uint32_t* des0_ptr = (volatile uint32_t*)dma_tx_list;
      uint32_t des0 = *des0_ptr;
      Serial.printf("  TX_BASE DES0 before=0x%08x\n", des0);
      if (des0 != 0 && !(des0 >> 31)) {
        // Has content but OWN=0 — set OWN=1
        *des0_ptr = des0 | 0x80000000u;
        Serial.printf("  TX_BASE DES0 after =0x%08x (OWN set)\n", *des0_ptr);
      }
      // Redirect DMA TX to ring base and kick it
      uint32_t opmode = REG_READ(0x3FF69018);
      REG_WRITE(0x3FF69018, opmode & ~(1u << 13));  // stop TX DMA
      ets_delay_us(100);
      REG_WRITE(0x3FF69010, dma_tx_list);            // reset list ptr
      REG_WRITE(0x3FF69018, opmode | (1u << 13));    // start TX DMA
      REG_WRITE(0x3FF69004, 1);                      // poll demand
      dma_reset_done = true;
      Serial.printf("  DMA TX redirected to TX_LIST_BASE + poll demand\n");
    }

    // Sample GPIO0 100 times rapidly to check if oscillator is actually toggling.
    // With 50MHz signal we should see a mix of 0s and 1s; if stuck = no clock.
    // Temporarily switch GPIO0 to GPIO input mode to sample it.
    REG_SET_FIELD(IO_MUX_GPIO0_REG, MCU_SEL, 2);  // func=2 = GPIO
    PIN_INPUT_ENABLE(IO_MUX_GPIO0_REG);
    int ones = 0;
    for (int i = 0; i < 100; i++) {
      ones += (int)((REG_READ(GPIO_IN_REG) >> 0) & 1);
    }
    // Restore EMAC clock function
    REG_SET_FIELD(IO_MUX_GPIO0_REG, MCU_SEL, FUNC_GPIO0_EMAC_TX_CLK);
    PIN_INPUT_ENABLE(IO_MUX_GPIO0_REG);
    CLEAR_PERI_REG_MASK(IO_MUX_GPIO0_REG, FUN_PD);
    CLEAR_PERI_REG_MASK(IO_MUX_GPIO0_REG, FUN_PU);
    Serial.printf("  GPIO0 sample: %d/100 ones (50MHz = ~50, stuck-high=100, stuck-low=0)\n", ones);

  }
}
