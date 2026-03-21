// Raw IDF Ethernet test — bypasses Arduino ETH.cpp entirely.
// Calls esp_eth_* APIs directly, same as the factory firmware would.
#include <Arduino.h>
#include <WiFi.h>
#include "esp_eth.h"
#include "esp_eth_mac.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_mac.h"

static const char *TAG = "eth_raw";
static bool got_ip = false;

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data) {
  switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
      Serial.println("ETH: Link up");
      break;
    case ETHERNET_EVENT_DISCONNECTED:
      Serial.println("ETH: Link down");
      break;
    case ETHERNET_EVENT_START:
      Serial.println("ETH: Started");
      break;
    case ETHERNET_EVENT_STOP:
      Serial.println("ETH: Stopped");
      break;
    default:
      break;
  }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data) {
  ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
  Serial.printf("ETH: Got IP: " IPSTR "\n", IP2STR(&event->ip_info.ip));
  got_ip = true;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nRaw IDF ETH test starting...");

  // GPIO17 controls oscillator enable AND LAN8720 nRST on the Aptinex board.
  // In GPIO0_OUT mode, the ESP32 APLL generates 50MHz on GPIO0.
  // Keep oscillator OFF (GPIO17 LOW) to avoid two clocks fighting on GPIO0.
  // The LAN8720 will get its clock from GPIO0 if it's wired to CLKIN.
  // Pulse HIGH briefly then LOW to ensure PHY gets a clean power-on reset.
  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = (1ULL << 17);
  io_conf.mode = GPIO_MODE_OUTPUT;
  gpio_config(&io_conf);
  gpio_set_level(GPIO_NUM_17, 1);  // PHY power on + oscillator on
  delay(100);
  gpio_set_level(GPIO_NUM_17, 0);  // PHY reset + oscillator off
  delay(50);
  gpio_set_level(GPIO_NUM_17, 1);  // PHY out of reset, oscillator on
  delay(300);                       // wait for PHY + oscillator
  Serial.println("ETH: GPIO17 pulsed (PHY reset done, oscillator on)");

  // Initialize TCP/IP and event loop
  esp_err_t ret;
  ret = esp_netif_init();
  Serial.printf("esp_netif_init: %d\n", ret);

  ret = esp_event_loop_create_default();
  Serial.printf("esp_event_loop_create_default: %d\n", ret);

  // Create default Ethernet netif
  esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
  esp_netif_t *eth_netif = esp_netif_new(&netif_cfg);

  // MAC config
  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
  esp32_emac_config.smi_gpio.mdc_num = GPIO_NUM_23;
  esp32_emac_config.smi_gpio.mdio_num = GPIO_NUM_18;
  // Use GPIO0_OUT mode: ESP32 APLL generates 50MHz on GPIO0.
  // The Aptinex board's external oscillator also feeds the LAN8720 XI pin
  // via GPIO17, so both clock sources are active. Green LED was flashing
  // with this configuration in earlier tests.
  esp32_emac_config.clock_config.rmii.clock_mode = EMAC_CLK_OUT;
  esp32_emac_config.clock_config.rmii.clock_gpio = EMAC_APPL_CLK_OUT_GPIO;

  esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
  if (!mac) {
    Serial.println("ERROR: esp_eth_mac_new_esp32 failed");
    return;
  }

  // PHY config
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  phy_config.phy_addr = 1;
  phy_config.reset_gpio_num = -1;  // no PHY reset pin managed by IDF

  esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);
  if (!phy) {
    Serial.println("ERROR: esp_eth_phy_new_lan87xx failed");
    return;
  }

  // Install ETH driver
  esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
  esp_eth_handle_t eth_handle = NULL;
  ret = esp_eth_driver_install(&config, &eth_handle);
  Serial.printf("esp_eth_driver_install: %d\n", ret);
  if (ret != ESP_OK) return;

  // Attach to netif
  esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle));

  // Register event handlers
  esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL);
  esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL);

  // Start
  ret = esp_eth_start(eth_handle);
  Serial.printf("esp_eth_start: %d\n", ret);
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last > 5000) {
    last = millis();
    Serial.printf("uptime=%lus  got_ip=%d\n", millis() / 1000, (int)got_ip);
  }
}
