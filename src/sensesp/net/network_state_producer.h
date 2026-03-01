#ifndef SENSESP_NET_NETWORK_STATE_PRODUCER_H_
#define SENSESP_NET_NETWORK_STATE_PRODUCER_H_

#include <Network.h>
#include <WiFi.h>

#include "sensesp/net/network_state.h"
#include "sensesp/system/valueproducer.h"

namespace sensesp {

/**
 * @brief Unified network state producer for all interface types.
 *
 * Listens to Network.onEvent() for WiFi STA, WiFi AP, Ethernet, and PPP
 * events and emits a transport-agnostic NetworkState. This replaces the
 * WiFi-only WiFiStateProducer.
 *
 * The producer tracks whether any interface has obtained an IP address.
 * It reports kConnected as soon as at least one interface is online, and
 * kDisconnected only when all interfaces have lost connectivity.
 */
class NetworkStateProducer : public ValueProducer<NetworkState> {
 public:
  NetworkStateProducer() {
    this->output_ = NetworkState::kNoInterface;

    setup_callbacks();

    // Emit the initial state once the event loop starts
    event_loop()->onDelay(0, [this]() { this->emit(this->output_); });
  }

  ~NetworkStateProducer() { remove_callbacks(); }

  NetworkStateProducer(NetworkStateProducer& other) = delete;
  void operator=(const NetworkStateProducer&) = delete;

 protected:
  network_event_id_t sta_got_ip_id_;
  network_event_id_t sta_disconnected_id_;
  network_event_id_t ap_start_id_;
  network_event_id_t ap_stop_id_;
  network_event_id_t eth_got_ip_id_;
  network_event_id_t eth_disconnected_id_;

  void setup_callbacks() {
    // WiFi STA events
    sta_got_ip_id_ = Network.onEvent(
        [this](arduino_event_id_t event, arduino_event_info_t info) {
          on_interface_connected("WiFi STA");
        },
        ARDUINO_EVENT_WIFI_STA_GOT_IP);

    sta_disconnected_id_ = Network.onEvent(
        [this](arduino_event_id_t event, arduino_event_info_t info) {
          on_interface_disconnected("WiFi STA");
        },
        ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

    // WiFi AP events
    ap_start_id_ = Network.onEvent(
        [this](arduino_event_id_t event, arduino_event_info_t info) {
          on_ap_activated();
        },
        ARDUINO_EVENT_WIFI_AP_START);

    ap_stop_id_ = Network.onEvent(
        [this](arduino_event_id_t event, arduino_event_info_t info) {
          on_interface_disconnected("WiFi AP");
        },
        ARDUINO_EVENT_WIFI_AP_STOP);

    // Ethernet events
    eth_got_ip_id_ = Network.onEvent(
        [this](arduino_event_id_t event, arduino_event_info_t info) {
          on_interface_connected("Ethernet");
        },
        ARDUINO_EVENT_ETH_GOT_IP);

    eth_disconnected_id_ = Network.onEvent(
        [this](arduino_event_id_t event, arduino_event_info_t info) {
          on_interface_disconnected("Ethernet");
        },
        ARDUINO_EVENT_ETH_DISCONNECTED);
  }

  void remove_callbacks() {
    Network.removeEvent(sta_got_ip_id_);
    Network.removeEvent(sta_disconnected_id_);
    Network.removeEvent(ap_start_id_);
    Network.removeEvent(ap_stop_id_);
    Network.removeEvent(eth_got_ip_id_);
    Network.removeEvent(eth_disconnected_id_);
  }

  void on_interface_connected(const char* iface_name) {
    ESP_LOGI(__FILENAME__, "%s connected", iface_name);
    this->emit(NetworkState::kConnected);
  }

  void on_interface_disconnected(const char* iface_name) {
    ESP_LOGI(__FILENAME__, "%s disconnected", iface_name);
    // Only report disconnected if no interface is online
    if (!Network.isOnline()) {
      this->emit(NetworkState::kDisconnected);
    }
  }

  void on_ap_activated() {
    ESP_LOGI(__FILENAME__, "WiFi AP activated, SSID: %s",
             WiFi.softAPSSID().c_str());
    ESP_LOGI(__FILENAME__, "AP IP address: %s",
             WiFi.softAPIP().toString().c_str());
    // Delay the emit — AP setup happens synchronously and the callback
    // may fire before all startables have been initialized.
    event_loop()->onDelay(
        0, [this]() { this->emit(NetworkState::kAPModeActivated); });
  }
};

// Backward compatibility alias
using WiFiStateProducer = NetworkStateProducer;

}  // namespace sensesp

#endif  // SENSESP_NET_NETWORK_STATE_PRODUCER_H_
