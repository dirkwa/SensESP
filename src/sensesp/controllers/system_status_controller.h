#ifndef _SYSTEM_STATUS_CONTROLLER_H_
#define _SYSTEM_STATUS_CONTROLLER_H_

#include "sensesp/net/network_state.h"
#include "sensesp/signalk/signalk_ws_client.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/valueproducer.h"

namespace sensesp {

enum class SystemStatus {
  // Transport-agnostic names
  kNoNetwork = 100,
  kNetworkDisconnected,
  kNetworkAPMode,
  kSKWSDisconnected,
  kSKWSAuthorizing,
  kSKWSConnecting,
  kSKWSConnected,

  // Backward-compatible aliases
  kWifiNoAP = kNoNetwork,
  kWifiDisconnected = kNetworkDisconnected,
  kWifiManagerActivated = kNetworkAPMode,
};

/**
 * @brief Base class for a controller that can react to system status events.
 *
 * Consumes NetworkState (from any provisioner) and SKWSConnectionState
 * to produce a unified SystemStatus that drives LED indicators and
 * system monitoring.
 */
class SystemStatusController : public ValueProducer<SystemStatus> {
 public:
  SystemStatusController() {}

  ValueConsumer<NetworkState>& get_network_state_consumer() {
    return network_state_consumer_;
  }

  // Backward compatibility alias
  ValueConsumer<NetworkState>& get_wifi_state_consumer() {
    return get_network_state_consumer();
  }

  ValueConsumer<SKWSConnectionState>& get_ws_connection_state_consumer() {
    return ws_connection_state_consumer_;
  }

 protected:
  void set_network_state(const NetworkState& new_value) {
    switch (new_value) {
      case NetworkState::kNoInterface:
        this->update_state(SystemStatus::kNoNetwork);
        break;
      case NetworkState::kDisconnected:
        this->update_state(SystemStatus::kNetworkDisconnected);
        break;
      case NetworkState::kConnected:
        this->update_state(SystemStatus::kSKWSDisconnected);
        break;
      case NetworkState::kAPModeActivated:
        this->update_state(SystemStatus::kNetworkAPMode);
        break;
    }
  }

  void set_sk_ws_connection_state(const SKWSConnectionState& new_value) {
    switch (new_value) {
      case SKWSConnectionState::kSKWSDisconnected:
        if (current_state_ != SystemStatus::kNetworkDisconnected &&
            current_state_ != SystemStatus::kNoNetwork &&
            current_state_ != SystemStatus::kNetworkAPMode) {
          // Network disconnection states override the higher level protocol
          this->update_state(SystemStatus::kSKWSDisconnected);
        }
        break;
      case SKWSConnectionState::kSKWSConnecting:
        this->update_state(SystemStatus::kSKWSConnecting);
        break;
      case SKWSConnectionState::kSKWSAuthorizing:
        this->update_state(SystemStatus::kSKWSAuthorizing);
        break;
      case SKWSConnectionState::kSKWSConnected:
        this->update_state(SystemStatus::kSKWSConnected);
        break;
    }
  }

  LambdaConsumer<NetworkState> network_state_consumer_{
      [this](const NetworkState& new_value) {
        this->set_network_state(new_value);
      }};
  LambdaConsumer<SKWSConnectionState> ws_connection_state_consumer_{
      [this](const SKWSConnectionState& new_value) {
        this->set_sk_ws_connection_state(new_value);
      }};

  void update_state(const SystemStatus new_state) {
    current_state_ = new_state;
    this->emit(new_state);
  }

 private:
  SystemStatus current_state_ = SystemStatus::kNoNetwork;
};

}  // namespace sensesp

#endif
