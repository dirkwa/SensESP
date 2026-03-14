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

    uint32_t dma_status = REG_READ(0x3FF69014);
    Serial.printf("  DMA_STATUS=0x%08x  TX_STATE=%d  RX_STATE=%d\n",
                  dma_status,
                  (int)((dma_status >> 20) & 0x7),
                  (int)((dma_status >> 17) & 0x7));

  }
}
