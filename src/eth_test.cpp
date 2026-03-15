// Minimal Aptinex IsolPoE Ethernet test — no SensESP framework.
//
// GPIO17 = LAN8720 nRST / oscillator enable (4.7k pulldown → LOW by default).
//   LOW  → PHY in reset, oscillator off
//   HIGH → PHY running, oscillator on
//
// The LAN8720 requires REFCLK (50MHz) to be present DURING its reset sequence
// for RMII to initialise correctly. Sequence:
//   1. Enable oscillator (GPIO17 HIGH) and let it stabilise.
//   2. Pre-configure GPIO0 IOMUX as EMAC RMII clock input so the clock is
//      present before ETH.begin() starts the EMAC.
//   3. Pulse nRST (GPIO17 LOW→HIGH briefly) with clock running.
//   4. Call ETH.begin() with power=-1 so IDF does not touch GPIO17 again.
//   5. Re-apply GPIO0 IOMUX after ETH.begin() in case it was reset.

#include <ETH.h>
#include <WiFi.h>
#include "esp_private/esp_gpio_reserve.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_reg.h"
#include "lwip/netif.h"
#include "lwip/dhcp.h"

#define PHY_RST_PIN 17

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

static void gpio0_as_rmii_clk() {
  esp_gpio_revoke(BIT64(GPIO_NUM_0));
  REG_SET_FIELD(IO_MUX_GPIO0_REG, MCU_SEL, FUNC_GPIO0_EMAC_TX_CLK);
  PIN_INPUT_ENABLE(IO_MUX_GPIO0_REG);
  CLEAR_PERI_REG_MASK(IO_MUX_GPIO0_REG, FUN_PD);
  CLEAR_PERI_REG_MASK(IO_MUX_GPIO0_REG, FUN_PU);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nAptinex IsolPoE ETH test starting...");

  // Step 1: enable oscillator and let it stabilise.
  pinMode(PHY_RST_PIN, OUTPUT);
  digitalWrite(PHY_RST_PIN, HIGH);
  delay(50);
  Serial.println("ETH: oscillator on");

  // Step 2: configure GPIO0 as RMII clock input while oscillator is running.
  gpio0_as_rmii_clk();
  Serial.printf("ETH: GPIO0 IOMUX set  MCU_SEL=%d\n",
                (int)REG_GET_FIELD(IO_MUX_GPIO0_REG, MCU_SEL));

  // Step 3: pulse PHY reset with clock present.
  digitalWrite(PHY_RST_PIN, LOW);
  delay(10);
  digitalWrite(PHY_RST_PIN, HIGH);
  delay(50);
  Serial.println("ETH: PHY reset done");

  // Step 4: start Ethernet — power=-1 so IDF leaves GPIO17 alone.
  WiFi.mode(WIFI_OFF);
  Network.onEvent(onEvent);
  Serial.printf("ETH: calling ETH.begin clk_mode=%d\n", (int)ETH_CLK_MODE);
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO,
            -1, ETH_CLK_MODE);

  // Step 5: ETH.begin() calls perimanClearPinBus(GPIO0) which resets IOMUX.
  // Re-apply immediately AND keep polling until the EMAC DMA list is non-zero,
  // which confirms the EMAC driver has started and is using the clock.
  for (int i = 0; i < 200; i++) {  // up to 2 seconds
    gpio0_as_rmii_clk();
    if (REG_READ(0x3FF69010) != 0) break;
    delay(10);
  }
  Serial.printf("ETH: GPIO0 IOMUX after ETH.begin  MCU_SEL=%d  GPIO17=%d  TX_LIST=0x%08x\n",
                (int)REG_GET_FIELD(IO_MUX_GPIO0_REG, MCU_SEL),
                (int)digitalRead(PHY_RST_PIN),
                REG_READ(0x3FF69010));

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
      if (des3 == tx_base) { // normal ring end
        break;
      }
      if (des3 < 0x3FF00000 || des3 > 0x3FFFFFFF) {
        // DES3 points outside valid descriptor SRAM — this is the broken link
        Serial.printf("ETH: fixing broken TX ring: desc@0x%08x DES3=0x%08x -> 0x%08x\n",
                      desc, des3, tx_base);
        d[3] = tx_base;
        break;
      }
      prev = desc;
      desc = des3;
      count++;
    }
    Serial.printf("ETH: TX ring walk done (%d descriptors)\n", count);
  }
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

    uint32_t dma_status  = REG_READ(0x3FF69014);
    uint32_t dma_tx_list = REG_READ(0x3FF69010);
    uint32_t dma_tx_curr = REG_READ(0x3FF69050);
    Serial.printf("  DMA TX_STATE=%d  TX_LIST=0x%08x  TX_CURR=0x%08x\n",
                  (int)((dma_status >> 20) & 0x7),
                  dma_tx_list, dma_tx_curr);
    if (dma_tx_curr >= 0x3FF00000 && dma_tx_curr <= 0x3FFFFFFF) {
      uint32_t d0 = *((volatile uint32_t*)(dma_tx_curr + 0));
      uint32_t d2 = *((volatile uint32_t*)(dma_tx_curr + 8));
      uint32_t d3 = *((volatile uint32_t*)(dma_tx_curr + 12));
      Serial.printf("  TX_CURR desc: DES0=0x%08x DES2=0x%08x DES3=0x%08x OWN=%d\n",
                    d0, d2, d3, (int)(d0 >> 31));
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
