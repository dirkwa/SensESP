#ifndef SENSESP_NET_NETWORK_STATE_H_
#define SENSESP_NET_NETWORK_STATE_H_

namespace sensesp {

/**
 * @brief Transport-agnostic network connectivity state.
 *
 * Represents the state of the device's network connectivity regardless of
 * the underlying transport (WiFi, Ethernet, PPP, etc.).
 *
 * The WiFi-specific enum value names are preserved as aliases so that
 * existing code using WiFiState::kWifiConnectedToAP etc. continues to
 * compile without changes.
 */
enum class NetworkState {
  // Canonical transport-agnostic names
  kNoInterface = 0,    // No network interface configured
  kDisconnected,       // Interface(s) configured but no IP / link down
  kConnected,          // At least one interface has a routable IP
  kAPModeActivated,    // WiFi AP active (provisioning / captive portal)

  // Backward-compatible aliases (map to the canonical names above)
  kWifiNoAP = kNoInterface,
  kWifiDisconnected = kDisconnected,
  kWifiConnectedToAP = kConnected,
  kWifiManagerActivated = kAPModeActivated,
  kWifiAPModeActivated = kAPModeActivated,
};

// Type aliases so existing code using WiFiState / WifiState compiles.
using WiFiState = NetworkState;
using WifiState = NetworkState;

}  // namespace sensesp

#endif  // SENSESP_NET_NETWORK_STATE_H_
