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

  // Extra delay (ms) before PHY init. Use for PoE boards where the PD power
  // delivery negotiation completes after the ESP32 has already started booting.
  // Leave at 0 for USB-powered or wall-wart boards.
  unsigned int poe_stabilize_ms = 0;

  // Board presets
  static EthernetConfig olimex_esp32_poe_iso() {
    // GPIO12 controls 3.3V power to the isolated PHY section via FETs.
    // The provisioner handles the power-on manually with a proper delay,
    // then passes power=-1 to ETH.begin() to skip the driver's too-fast
    // reset-style toggle.
    EthernetConfig c{ETH_PHY_LAN8720, 0, 23, 18, 12, ETH_CLOCK_GPIO17_OUT};
    c.poe_stabilize_ms = 1000;
    return c;
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
  static EthernetConfig aptinex_isolpoe() {
    // External 50 MHz crystal oscillator on GPIO0; PHY power on GPIO17.
    EthernetConfig c{ETH_PHY_LAN8720, 1, 23, 18, 17, ETH_CLOCK_GPIO0_IN};
    c.poe_stabilize_ms = 1000;
    return c;
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

    // On PoE boards, the switch-side power delivery negotiation can complete
    // after the ESP32 has already started booting. Wait before touching the PHY.
    if (config.poe_stabilize_ms > 0) {
      ESP_LOGI(__FILENAME__, "Waiting %ums for PoE power to stabilize",
               config.poe_stabilize_ms);
      delay(config.poe_stabilize_ms);
    }

    ESP_LOGI(__FILENAME__,
             "Initializing Ethernet (PHY type=%d, addr=%d, MDC=%d, MDIO=%d, "
             "power=%d, clk=%d)",
             config.phy_type, config.phy_addr, config.mdc, config.mdio,
             config.power, config.clk_mode);

    if (!config.use_dhcp) {
      ETH.config(config.ip, config.gateway, config.netmask, config.dns);
    }

    // If a power pin is specified, drive it HIGH manually and give the PHY
    // time to stabilize.  Then pass power=-1 to ETH.begin() so the driver
    // doesn't do a reset-style low→high→low toggle that would cut power.
    int power_for_driver = config.power;
    if (config.power >= 0) {
      ESP_LOGI(__FILENAME__, "Powering PHY via GPIO%d", config.power);
      pinMode(config.power, OUTPUT);
      digitalWrite(config.power, HIGH);
      delay(500);  // let PHY power rail stabilize
      power_for_driver = -1;
    }

    bool started = ETH.begin(config.phy_type, config.phy_addr, config.mdc,
                             config.mdio, power_for_driver, config.clk_mode);

    if (!started) {
      ESP_LOGE(__FILENAME__, "Failed to initialize Ethernet interface");
      return;
    }

    ETH.setHostname(hostname.c_str());
    ESP_LOGI(__FILENAME__, "Ethernet interface initialized, waiting for link...");

    // Wait for physical link (up to 60 seconds).
    // On PoE boards the switch-side power negotiation can take well over 10s,
    // keeping the PHY oscillator unpowered until it completes.
    for (int i = 0; i < 600 && !ETH.linkUp(); i++) {
      if (i % 100 == 99) {
        ESP_LOGW(__FILENAME__, "Still waiting for Ethernet link... (%ds)", (i + 1) / 10);
      }
      delay(100);
    }

    if (!ETH.linkUp()) {
      ESP_LOGE(__FILENAME__,
               "Ethernet link not established after 60 s — check cable");
      return;
    }

    ESP_LOGI(__FILENAME__, "Ethernet link up (%s, %s), waiting for DHCP...",
             ETH.fullDuplex() ? "full duplex" : "half duplex",
             ETH.linkSpeed() == 100 ? "100Mbps" : "10Mbps");

    // Wait for DHCP to assign an IP address (up to 45 seconds).
    // Switches with spanning tree (STP) hold the port in listening/learning
    // state for ~30 s before forwarding — DHCP Discovers are dropped during
    // this period, so 15 s was too short on such networks.
    for (int i = 0; i < 450 && !ETH.hasIP(); i++) {
      delay(100);
    }

    if (ETH.hasIP()) {
      ESP_LOGI(__FILENAME__, "Ethernet IP: %s", ETH.localIP().toString().c_str());
    } else {
      ESP_LOGW(__FILENAME__, "DHCP timeout — no IP address after 45 s");
    }
  }
};

}  // namespace sensesp

#endif  // SENSESP_NET_ETHERNET_PROVISIONER_H_
