// Minimal Aptinex IsolPoE Ethernet test — no SensESP framework.
//
// GPIO17 = LAN8720 nRST / oscillator enable (4.7k pulldown → LOW by default).
//
// Two clock modes are supported, selected by ETH_CLK_MODE build flag:
//
// ETH_CLOCK_GPIO0_IN (0): external 50MHz oscillator → GPIO0 RMII input.
//   GPIO17 = oscillator enable + PHY nRST.
//   Requires IOMUX workaround (perimanClearPinBus resets GPIO0 during init).
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

#define PHY_RST_PIN 17  // only used when ETH_CLK_MODE == ETH_CLOCK_GPIO0_IN

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
  esp_reset_reason_t reset_reason = esp_reset_reason();
  Serial.printf("\nAptinex IsolPoE ETH test starting... reset_reason=%d\n", (int)reset_reason);

#if ETH_CLK_MODE == ETH_CLOCK_GPIO0_IN
  // External oscillator mode: GPIO17 = oscillator enable + PHY nRST.
  delay(200);  // let GPIO0 strapping pin settle after cold boot
  pinMode(PHY_RST_PIN, OUTPUT);
  digitalWrite(PHY_RST_PIN, HIGH);
  delay(100);
  Serial.println("ETH: oscillator on");

  gpio0_as_rmii_clk();
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
  Serial.printf("ETH: calling ETH.begin clk_mode=%d\n", (int)ETH_CLK_MODE);
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO,
            -1, ETH_CLK_MODE);

#if ETH_CLK_MODE == ETH_CLOCK_GPIO0_IN
  // Re-apply IOMUX immediately after ETH.begin() and wait for DMA to init.
  for (int i = 0; i < 3000; i++) {
    gpio0_as_rmii_clk();
    if (REG_READ(0x3FF69010) != 0) break;
    vTaskDelay(pdMS_TO_TICKS(2));
    if (i % 500 == 499)
      Serial.printf("ETH: waiting for EMAC DMA init... %ds\n", (i + 1) * 2 / 1000);
  }
#endif
  Serial.printf("ETH: after ETH.begin  TX_LIST=0x%08x  RX_LIST=0x%08x\n",
                REG_READ(0x3FF69010), REG_READ(0x3FF6900C));

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
    uint32_t dma_tx_curr = REG_READ(0x3FF69050);
    uint32_t dma_rx_list = REG_READ(0x3FF6900C);
    uint32_t dma_rx_curr = REG_READ(0x3FF69054);
    Serial.printf("  DMA TX_STATE=%d  TX_LIST=0x%08x  TX_CURR=0x%08x\n",
                  (int)((dma_status >> 20) & 0x7),
                  dma_tx_list, dma_tx_curr);
    Serial.printf("  RX_LIST=0x%08x  RX_CURR=0x%08x\n", dma_rx_list, dma_rx_curr);
    if (dma_tx_curr >= 0x3FF00000 && dma_tx_curr <= 0x3FFFFFFF) {
      uint32_t d0 = *((volatile uint32_t*)(dma_tx_curr + 0));
      uint32_t d2 = *((volatile uint32_t*)(dma_tx_curr + 8));
      uint32_t d3 = *((volatile uint32_t*)(dma_tx_curr + 12));
      Serial.printf("  TX_CURR desc: DES0=0x%08x DES2=0x%08x DES3=0x%08x OWN=%d\n",
                    d0, d2, d3, (int)(d0 >> 31));
    }
    // Dump all TX descriptors once (only on first poll)
    static bool tx_ring_dumped = false;
    if (!tx_ring_dumped && dma_tx_list >= 0x3FF00000 && dma_tx_list <= 0x3FFFFFFF) {
      tx_ring_dumped = true;
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
