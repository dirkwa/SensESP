#include "sensesp/net/ble/ble_signalk_gateway.h"

#include <HTTPClient.h>

#include "esp_log.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/signalk/signalk_ws_client.h"

namespace sensesp {

namespace {
constexpr const char* kTag = "ble_gw";

constexpr const char* kAdvertisementsPath =
    "/signalk/v2/api/ble/gateway/advertisements";
constexpr const char* kControlWsPathPrefix =
    "/signalk/v2/api/ble/gateway/ws?token=";

String bytes_to_hex(const std::vector<uint8_t>& data) {
  String out;
  out.reserve(data.size() * 2);
  for (uint8_t b : data) {
    char tmp[3];
    snprintf(tmp, sizeof(tmp), "%02X", b);
    out += tmp;
  }
  return out;
}

}  // namespace

BLESignalKGateway::BLESignalKGateway(std::shared_ptr<BLEProvisioner> ble,
                                     std::shared_ptr<SKWSClient> sk_client,
                                     Config config)
    : ble_provisioner_(std::move(ble)),
      sk_client_(std::move(sk_client)),
      config_(config) {
  pending_ads_mutex_ = xSemaphoreCreateMutex();
  control_ws_mutex_ = xSemaphoreCreateMutex();
  pending_ads_.reserve(config_.max_pending_ads);
}

BLESignalKGateway::~BLESignalKGateway() {
  stop();
  if (pending_ads_mutex_ != nullptr) {
    vSemaphoreDelete(pending_ads_mutex_);
    pending_ads_mutex_ = nullptr;
  }
  if (control_ws_mutex_ != nullptr) {
    vSemaphoreDelete(control_ws_mutex_);
    control_ws_mutex_ = nullptr;
  }
}

void BLESignalKGateway::start() {
  if (started_.exchange(true)) {
    return;
  }
  if (!ble_provisioner_ || !sk_client_) {
    ESP_LOGE(kTag, "start() called with null provisioner or SK client");
    started_.store(false);
    return;
  }

  ESP_LOGI(kTag, "Starting BLE SignalK gateway services");

  // Attach an observer to the BLE provisioner's ValueProducer. Each
  // received advertisement triggers on_advertisement(), which reads
  // via get() and buffers into pending_ads_ for the HTTP POST task.
  ble_provisioner_->attach([this]() { this->on_advertisement(); });

  // Hook the SK connection state producer. SKWSClient inherits from
  // ValueProducer<SKWSConnectionState> so we can connect_to() it
  // directly. When SK connects we want to (re)start the control WS;
  // when it disconnects we want to tear it down so it does not keep
  // trying to reach a dead server.
  sk_client_->connect_to(
      new LambdaConsumer<SKWSConnectionState>([this](SKWSConnectionState s) {
        if (s == SKWSConnectionState::kSKWSConnected) {
          ESP_LOGI(kTag,
                   "SK main WS connected — starting BLE gateway control WS");
          sk_connected_.store(true);
          init_control_ws();
        } else {
          ESP_LOGI(kTag,
                   "SK main WS disconnected — tearing down control WS");
          sk_connected_.store(false);
          destroy_control_ws();
        }
      }));

  // Belt-and-suspenders: if the SK WS is already connected at the
  // moment we started (e.g. because SensESPApp::setup() has already
  // completed the SK bring-up before our start() was called), kick
  // the control WS manually. Otherwise we would have to wait for the
  // next state transition.
  if (sk_client_->is_connected()) {
    ESP_LOGI(kTag,
             "SK main WS already connected at start — kicking control WS");
    sk_connected_.store(true);
    init_control_ws();
  }

  // Start background POST task.
  post_task_should_run_.store(true);
  xTaskCreate(&BLESignalKGateway::post_task_entry, "ble_gw_post", 8192, this,
              1, &post_task_);

  // Start scanning on the provided BLE provisioner.
  if (!ble_provisioner_->is_scanning()) {
    ble_provisioner_->start_scan();
  }
}

void BLESignalKGateway::stop() {
  if (!started_.exchange(false)) {
    return;
  }

  ESP_LOGI(kTag, "Stopping BLE SignalK gateway services");

  // Stop POST task. The task loop checks post_task_should_run_ on
  // each iteration and exits cleanly when it flips to false.
  post_task_should_run_.store(false);
  // Intentionally NOT calling vTaskDelete here. The task will
  // delete itself by calling vTaskDelete(nullptr) at the end of
  // post_task_loop() so we avoid races with the task still holding
  // the pending_ads_ mutex.
  post_task_ = nullptr;

  destroy_control_ws();

  // The BLE provisioner outlives the gateway — we only stop scanning
  // if we were the ones who started it. For simplicity in this first
  // cut, always stop the scanner on gateway stop. Users who want
  // scanning to continue after stop() can call ble->start_scan()
  // again afterwards.
  if (ble_provisioner_ && ble_provisioner_->is_scanning()) {
    ble_provisioner_->stop_scan();
  }

  // Note: we do not detach the ble_provisioner_ observer. The
  // Observable::attach API does not currently support detaching by
  // functor identity (only by int ID returned from attach), and the
  // cost of letting a dead-gateway callback run once is minimal —
  // on_advertisement() will see started_==false and early-return.
}

void BLESignalKGateway::on_advertisement() {
  if (!started_.load()) {
    return;
  }
  adv_received_count_.fetch_add(1, std::memory_order_relaxed);

  const BLEAdvertisement& ad = ble_provisioner_->get();

  if (xSemaphoreTake(pending_ads_mutex_, pdMS_TO_TICKS(50)) != pdTRUE) {
    // Could not grab the buffer mutex quickly enough — drop this
    // advertisement rather than block the GAP event callback.
    adv_dropped_count_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  if (pending_ads_.size() >= config_.max_pending_ads) {
    // Buffer full — drop oldest to keep newest. Cheap approximation:
    // drop the first half so we do not ping-pong on every new ad.
    const size_t keep = config_.max_pending_ads / 2;
    pending_ads_.erase(pending_ads_.begin(),
                       pending_ads_.begin() + (pending_ads_.size() - keep));
    adv_dropped_count_.fetch_add(pending_ads_.size() - keep,
                                 std::memory_order_relaxed);
  }
  pending_ads_.push_back(ad);
  xSemaphoreGive(pending_ads_mutex_);
}

void BLESignalKGateway::init_control_ws() {
  if (xSemaphoreTake(control_ws_mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
    return;
  }

  // Tear down any previous instance first.
  if (control_ws_ != nullptr) {
    esp_websocket_client_stop(control_ws_);
    esp_websocket_client_destroy(control_ws_);
    control_ws_ = nullptr;
  }

  String token = sk_client_->get_auth_token();
  String addr = sk_client_->get_server_address();
  uint16_t port = sk_client_->get_server_port();

  if (addr.length() == 0 || port == 0) {
    ESP_LOGW(kTag, "SK server address not available; deferring control WS");
    xSemaphoreGive(control_ws_mutex_);
    return;
  }

  String url = String("ws://") + addr + ":" + String(port) +
               kControlWsPathPrefix + token;

  ESP_LOGI(kTag, "Connecting control WS to ws://%s:%u%s?token=<redacted>",
           addr.c_str(), static_cast<unsigned>(port),
           "/signalk/v2/api/ble/gateway/ws");

  esp_websocket_client_config_t cfg = {};
  cfg.uri = url.c_str();
  cfg.task_stack = 4096;
  cfg.buffer_size = 1024;

  control_ws_ = esp_websocket_client_init(&cfg);
  if (control_ws_ == nullptr) {
    ESP_LOGE(kTag, "esp_websocket_client_init failed");
    xSemaphoreGive(control_ws_mutex_);
    return;
  }

  esp_websocket_register_events(control_ws_, WEBSOCKET_EVENT_ANY,
                                &BLESignalKGateway::control_ws_event_trampoline,
                                this);
  esp_websocket_client_start(control_ws_);

  xSemaphoreGive(control_ws_mutex_);
}

void BLESignalKGateway::destroy_control_ws() {
  if (xSemaphoreTake(control_ws_mutex_, pdMS_TO_TICKS(200)) != pdTRUE) {
    return;
  }
  if (control_ws_ != nullptr) {
    esp_websocket_client_stop(control_ws_);
    esp_websocket_client_destroy(control_ws_);
    control_ws_ = nullptr;
  }
  ws_connected_.store(false);
  xSemaphoreGive(control_ws_mutex_);
}

void BLESignalKGateway::send_hello() {
  if (!ws_connected_.load() || control_ws_ == nullptr) {
    return;
  }
  JsonDocument doc;
  doc["type"] = "hello";
  doc["gateway_id"] = SensESPBaseApp::get_hostname();
  doc["firmware"] = config_.firmware_version;
  doc["max_gatt_connections"] = config_.max_gatt_sessions;
  doc["active_gatt_connections"] = 0;
  doc["mac"] = ble_provisioner_ ? ble_provisioner_->mac_address() : String("");
  doc["hostname"] = SensESPBaseApp::get_hostname();

  String msg;
  serializeJson(doc, msg);
  esp_websocket_client_send_text(control_ws_, msg.c_str(), msg.length(),
                                 portMAX_DELAY);
  ESP_LOGI(kTag, "Sent hello");
}

void BLESignalKGateway::send_status() {
  if (!ws_connected_.load() || control_ws_ == nullptr) {
    return;
  }
  JsonDocument doc;
  doc["type"] = "status";
  doc["gateway_id"] = SensESPBaseApp::get_hostname();
  doc["uptime"] = millis() / 1000;
  doc["free_heap"] = ESP.getFreeHeap();
  doc["active_gatt_connections"] = 0;
  doc["max_gatt_connections"] = config_.max_gatt_sessions;
  doc["scan_hits"] = ble_provisioner_ ? ble_provisioner_->scan_hit_count() : 0;
  doc["post_success"] = http_post_success_.load();
  doc["post_fail"] = http_post_fail_.load();

  String msg;
  serializeJson(doc, msg);
  esp_websocket_client_send_text(control_ws_, msg.c_str(), msg.length(),
                                 portMAX_DELAY);
}

void BLESignalKGateway::handle_control_ws_message(uint8_t* payload,
                                                  size_t length) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    ESP_LOGW(kTag, "Control WS JSON parse error: %s", err.c_str());
    return;
  }
  const char* type = doc["type"];
  if (type == nullptr) {
    return;
  }
  if (strcmp(type, "hello_ack") == 0) {
    ESP_LOGI(kTag, "Hello acknowledged by server");
  } else if (strcmp(type, "gatt_subscribe") == 0 ||
             strcmp(type, "gatt_write") == 0 ||
             strcmp(type, "gatt_close") == 0) {
    ESP_LOGW(kTag,
             "Received '%s' command but GATT client support is not yet "
             "implemented — ignoring",
             type);
  } else {
    ESP_LOGD(kTag, "Unhandled control message type: %s", type);
  }
}

void BLESignalKGateway::post_pending_advertisements() {
  if (!sk_connected_.load()) {
    return;
  }

  String token = sk_client_->get_auth_token();
  String addr = sk_client_->get_server_address();
  uint16_t port = sk_client_->get_server_port();
  if (addr.length() == 0 || port == 0) {
    return;
  }
  // An empty token is valid when the server has security disabled
  // (signalk-server returns 404 on access-request and SKWSClient
  // proceeds without a token). In that case we still POST but
  // without the Authorization header.

  // Drain the pending buffer under the mutex.
  std::vector<BLEAdvertisement> to_post;
  if (xSemaphoreTake(pending_ads_mutex_, pdMS_TO_TICKS(200)) != pdTRUE) {
    return;
  }
  to_post.swap(pending_ads_);
  xSemaphoreGive(pending_ads_mutex_);

  if (to_post.empty()) {
    return;
  }

  // Build the JSON batch.
  JsonDocument doc;
  doc["gateway_id"] = SensESPBaseApp::get_hostname();
  JsonArray devices = doc["devices"].to<JsonArray>();
  for (const auto& ad : to_post) {
    JsonObject dev = devices.add<JsonObject>();
    dev["mac"] = ad.address;
    dev["rssi"] = ad.rssi;
    if (ad.name.length() > 0) {
      dev["name"] = ad.name;
    }
    if (!ad.adv_data.empty()) {
      dev["adv_data"] = bytes_to_hex(ad.adv_data);
    }
  }

  String body;
  serializeJson(doc, body);

  String url = String("http://") + addr + ":" + String(port) +
               kAdvertisementsPath;

  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  if (token.length() > 0) {
    http.addHeader("Authorization", String("Bearer ") + token);
  }
  http.addHeader("Connection", "close");
  http.setTimeout(3000);

  int code = http.POST(body);
  if (code == 200) {
    adv_posted_count_.fetch_add(to_post.size(), std::memory_order_relaxed);
    http_post_success_.fetch_add(1, std::memory_order_relaxed);
    ESP_LOGI(kTag, "POST: forwarded %u adv, heap=%u",
             static_cast<unsigned>(to_post.size()),
             static_cast<unsigned>(ESP.getFreeHeap()));
  } else if (code == 401 || code == 403) {
    http_post_fail_.fetch_add(1, std::memory_order_relaxed);
    ESP_LOGW(kTag,
             "POST: auth rejected (HTTP %d) — restarting main SK connection",
             code);
    http.end();
    sk_client_->restart();
    return;
  } else {
    http_post_fail_.fetch_add(1, std::memory_order_relaxed);
    ESP_LOGW(kTag, "POST failed: HTTP %d, heap=%u", code,
             static_cast<unsigned>(ESP.getFreeHeap()));
  }
  http.end();
}

void BLESignalKGateway::post_task_entry(void* arg) {
  static_cast<BLESignalKGateway*>(arg)->post_task_loop();
}

void BLESignalKGateway::post_task_loop() {
  unsigned long last_status_ms = 0;
  while (post_task_should_run_.load()) {
    vTaskDelay(pdMS_TO_TICKS(config_.post_interval_ms));
    post_pending_advertisements();

    unsigned long now = millis();
    if (now - last_status_ms >= config_.status_interval_ms) {
      send_status();
      last_status_ms = now;
    }
  }
  // Self-delete so we do not leak a FreeRTOS task handle after stop().
  vTaskDelete(nullptr);
}

void BLESignalKGateway::control_ws_event_trampoline(void* handler_args,
                                                    esp_event_base_t /*base*/,
                                                    int32_t event_id,
                                                    void* event_data) {
  static_cast<BLESignalKGateway*>(handler_args)
      ->handle_control_ws_event(event_id, event_data);
}

void BLESignalKGateway::handle_control_ws_event(int32_t event_id,
                                                void* event_data) {
  auto* data = static_cast<esp_websocket_event_data_t*>(event_data);
  switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
      ESP_LOGI(kTag, "Control WS connected");
      ws_connected_.store(true);
      ws_connected_count_.fetch_add(1, std::memory_order_relaxed);
      send_hello();
      break;
    case WEBSOCKET_EVENT_DISCONNECTED:
      ESP_LOGI(kTag, "Control WS disconnected");
      ws_connected_.store(false);
      break;
    case WEBSOCKET_EVENT_DATA:
      if (data->op_code == 0x01 && data->data_len > 0) {
        // data_ptr is const char*; cast away the const for the JSON
        // parser which does not mutate the buffer.
        handle_control_ws_message(
            reinterpret_cast<uint8_t*>(const_cast<char*>(data->data_ptr)),
            data->data_len);
      }
      break;
    case WEBSOCKET_EVENT_ERROR:
      ESP_LOGW(kTag, "Control WS error");
      ws_connected_.store(false);
      break;
    default:
      break;
  }
}

}  // namespace sensesp
