#ifndef SENSESP_NET_ETHERNET_PROVISIONER_H_
#define SENSESP_NET_ETHERNET_PROVISIONER_H_

#include <ETH.h>

#include "sensesp_base_app.h"

namespace sensesp {

/**
 * @brief Ethernet pin configuration for RMII PHYs (LAN8720, RTL8201, etc.)
 *
 * Most ESP32 Ethernet boards use an RMII PHY connected to the ESP32's
 * internal Ethernet MAC. The pin mapping varies by board. Common presets
 * are provided as static factory methods.
 */
struct EthernetConfig {
  eth_phy_type_t phy_type = ETH_PHY_LAN8720;
  int32_t phy_addr = 0;
  int mdc = 23;
  int mdio = 18;
  int power = -1;  // -1 = no power pin
  eth_clock_mode_t clk_mode = ETH_CLOCK_GPIO0_IN;

  // Optional static IP configuration (DHCP by default)
  bool use_dhcp = true;
  IPAddress ip;
  IPAddress gateway;
  IPAddress netmask = IPAddress(255, 255, 255, 0);
  IPAddress dns = IPAddress(0, 0, 0, 0);  // 0.0.0.0 = use DHCP-provided DNS

  // Board presets
  static EthernetConfig olimex_esp32_poe_iso() {
    // power=-1: PHY is powered by PoE/USB, no GPIO power control needed.
    // GPIO12 is a strapping pin (MTDI) — letting the driver toggle it
    // causes flash-voltage conflicts and PHY power-up timeouts.
    return {ETH_PHY_LAN8720, 0, 23, 18, -1, ETH_CLOCK_GPIO17_OUT};
  }
  static EthernetConfig olimex_esp32_gateway() {
    return {ETH_PHY_LAN8720, 0, 23, 18, 5, ETH_CLOCK_GPIO17_OUT};
  }
  static EthernetConfig wesp32() {
    return {ETH_PHY_LAN8720, 0, 16, 17, -1, ETH_CLOCK_GPIO0_IN};
  }
  static EthernetConfig wt32_eth01() {
    return {ETH_PHY_LAN8720, 1, 23, 18, 16, ETH_CLOCK_GPIO0_IN};
  }
};

/**
 * @brief Ethernet network provisioner.
 *
 * Initializes the ESP32's Ethernet interface with the specified PHY
 * configuration. Supports DHCP (default) and static IP. The
 * NetworkStateProducer automatically picks up Ethernet events —
 * this class only needs to call ETH.begin().
 *
 * Usage with the builder:
 * @code
 *   builder.set_ethernet({
 *     .phy_type = ETH_PHY_LAN8720,
 *     .phy_addr = 0,
 *     .mdc = 23,
 *     .mdio = 18,
 *     .power = -1,
 *     .clk_mode = ETH_CLOCK_GPIO0_IN,
 *   });
 * @endcode
 *
 * Or with a board preset:
 * @code
 *   builder.set_ethernet(EthernetConfig::olimex_esp32_poe_iso());
 * @endcode
 */
class EthernetProvisioner {
 public:
  explicit EthernetProvisioner(const EthernetConfig& config) {
    String hostname = SensESPBaseApp::get_hostname();

    ESP_LOGI(__FILENAME__,
             "Initializing Ethernet (PHY type=%d, addr=%d, MDC=%d, MDIO=%d)",
             config.phy_type, config.phy_addr, config.mdc, config.mdio);

    if (!config.use_dhcp) {
      ETH.config(config.ip, config.gateway, config.netmask, config.dns);
    }

    bool started = ETH.begin(config.phy_type, config.phy_addr, config.mdc,
                             config.mdio, config.power, config.clk_mode);

    if (!started) {
      ESP_LOGE(__FILENAME__, "Failed to initialize Ethernet interface");
      return;
    }

    ETH.setHostname(hostname.c_str());
    ESP_LOGI(__FILENAME__, "Ethernet interface initialized");
  }
};

}  // namespace sensesp

#endif  // SENSESP_NET_ETHERNET_PROVISIONER_H_
