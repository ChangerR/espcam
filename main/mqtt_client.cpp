#include "mqtt_client.hpp"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include <cstring>
#include <sstream>
#include <iomanip>

static const char *TAG = "MQTTClient";

MQTTClient* MQTTClient::instance_ = nullptr;

MQTTClient::MQTTClient() 
    : client_(nullptr)
    , current_state_(ConnectionState::DISCONNECTED)
{
    instance_ = this;
}

MQTTClient::~MQTTClient() {
    disconnect();
    if (client_) {
        esp_mqtt_client_destroy(client_);
    }
    instance_ = nullptr;
}

esp_err_t MQTTClient::initialize(const MQTTConfig& config) {
    ESP_LOGI(TAG, "Initializing MQTT client");
    
    config_ = config;
    
    if (config_.client_id.empty()) {
        config_.client_id = generateClientId();
    }
    
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = config_.broker_url.c_str();
    mqtt_cfg.broker.address.port = config_.port;
    
    mqtt_cfg.credentials.username = config_.username.empty() ? nullptr : config_.username.c_str();
    mqtt_cfg.credentials.authentication.password = config_.password.empty() ? nullptr : config_.password.c_str();
    mqtt_cfg.credentials.client_id = config_.client_id.c_str();
    
    mqtt_cfg.session.keepalive = config_.keepalive;
    mqtt_cfg.session.disable_clean_session = false;
    
    mqtt_cfg.network.disable_auto_reconnect = false;
    mqtt_cfg.network.timeout_ms = 10000;
    mqtt_cfg.network.refresh_connection_after_ms = 20000;
    
    if (config_.use_ssl) {
        mqtt_cfg.broker.verification.use_global_ca_store = true;
        mqtt_cfg.broker.verification.common_name = nullptr;
        mqtt_cfg.broker.verification.skip_cert_common_name_check = false;
    }
    
    client_ = esp_mqtt_client_init(&mqtt_cfg);
    if (!client_) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }
    
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client_, MQTT_EVENT_ANY, mqttEventHandler, this));
    
    setState(ConnectionState::DISCONNECTED, "MQTT client initialized");
    ESP_LOGI(TAG, "MQTT client initialized with broker: %s", config_.broker_url.c_str());
    
    return ESP_OK;
}

esp_err_t MQTTClient::connect() {
    if (!client_) {
        ESP_LOGE(TAG, "MQTT client not initialized");
        return ESP_FAIL;
    }
    
    if (current_state_ == ConnectionState::CONNECTED) {
        ESP_LOGW(TAG, "MQTT client already connected");
        return ESP_OK;
    }
    
    setState(ConnectionState::CONNECTING, "Connecting to MQTT broker");
    ESP_LOGI(TAG, "Connecting to MQTT broker: %s", config_.broker_url.c_str());
    
    esp_err_t result = esp_mqtt_client_start(client_);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client");
        setState(ConnectionState::ERROR, "Failed to start MQTT client");
        return result;
    }
    
    return ESP_OK;
}

esp_err_t MQTTClient::disconnect() {
    if (!client_) {
        return ESP_OK;
    }
    
    if (current_state_ == ConnectionState::DISCONNECTED) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Disconnecting from MQTT broker");
    esp_err_t result = esp_mqtt_client_stop(client_);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop MQTT client");
    }
    
    setState(ConnectionState::DISCONNECTED, "Disconnected from MQTT broker");
    return result;
}

esp_err_t MQTTClient::publish(const std::string& topic, const std::string& payload, int qos, bool retain) {
    if (!client_ || current_state_ != ConnectionState::CONNECTED) {
        ESP_LOGE(TAG, "MQTT client not connected");
        return ESP_FAIL;
    }
    
    ESP_LOGD(TAG, "Publishing to topic: %s, payload: %s", topic.c_str(), payload.c_str());
    
    int msg_id = esp_mqtt_client_publish(client_, topic.c_str(), payload.c_str(), payload.length(),
                                        qos, retain);
    
    if (msg_id == -1) {
        ESP_LOGE(TAG, "Failed to publish message to topic: %s", topic.c_str());
        return ESP_FAIL;
    }
    
    ESP_LOGD(TAG, "Message published successfully, msg_id: %d", msg_id);
    return ESP_OK;
}

esp_err_t MQTTClient::subscribe(const std::string& topic, int qos) {
    if (!client_ || current_state_ != ConnectionState::CONNECTED) {
        ESP_LOGE(TAG, "MQTT client not connected");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Subscribing to topic: %s", topic.c_str());
    
    int msg_id = esp_mqtt_client_subscribe(client_, topic.c_str(), qos);
    
    if (msg_id == -1) {
        ESP_LOGE(TAG, "Failed to subscribe to topic: %s", topic.c_str());
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Subscribed to topic: %s, msg_id: %d", topic.c_str(), msg_id);
    return ESP_OK;
}

esp_err_t MQTTClient::unsubscribe(const std::string& topic) {
    if (!client_ || current_state_ != ConnectionState::CONNECTED) {
        ESP_LOGE(TAG, "MQTT client not connected");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Unsubscribing from topic: %s", topic.c_str());
    
    int msg_id = esp_mqtt_client_unsubscribe(client_, topic.c_str());
    
    if (msg_id == -1) {
        ESP_LOGE(TAG, "Failed to unsubscribe from topic: %s", topic.c_str());
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Unsubscribed from topic: %s, msg_id: %d", topic.c_str(), msg_id);
    return ESP_OK;
}

esp_err_t MQTTClient::publishHeartbeat() {
    std::string topic = getHeartbeatTopic();
    std::string payload = "{\"status\":\"alive\",\"timestamp\":" + std::to_string(esp_timer_get_time() / 1000) + "}";
    
    return publish(topic, payload, 0, false);
}

esp_err_t MQTTClient::subscribeToDeviceTopics() {
    esp_err_t result = ESP_OK;
    
    std::string command_topic = getCommandTopic();
    if (subscribe(command_topic, 1) != ESP_OK) {
        result = ESP_FAIL;
    }
    
    return result;
}

void MQTTClient::setMessageCallback(MessageCallback callback) {
    message_callback_ = callback;
}

void MQTTClient::setStateCallback(StateCallback callback) {
    state_callback_ = callback;
}

std::string MQTTClient::getHeartbeatTopic() const {
    return "devices/" + config_.device_serial + "/heartbeat";
}

std::string MQTTClient::getCommandTopic() const {
    return "devices/" + config_.device_serial + "/cmd";
}

std::string MQTTClient::getStatusTopic() const {
    return "devices/" + config_.device_serial + "/status";
}

void MQTTClient::mqttEventHandler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    MQTTClient* self = static_cast<MQTTClient*>(handler_args);
    if (!self) {
        ESP_LOGE(TAG, "MQTT handler called with null instance");
        return;
    }
    
    esp_mqtt_event_handle_t event = static_cast<esp_mqtt_event_handle_t>(event_data);
    self->handleMQTTEvent(event);
}

void MQTTClient::handleMQTTEvent(esp_mqtt_event_handle_t event) {
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT client connected");
            setState(ConnectionState::CONNECTED, "Connected to MQTT broker");
            subscribeToDeviceTopics();
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT client disconnected");
            setState(ConnectionState::DISCONNECTED, "Disconnected from MQTT broker");
            break;
            
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT subscription successful, msg_id: %d", event->msg_id);
            break;
            
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT unsubscription successful, msg_id: %d", event->msg_id);
            break;
            
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGD(TAG, "MQTT message published successfully, msg_id: %d", event->msg_id);
            break;
            
        case MQTT_EVENT_DATA:
            ESP_LOGD(TAG, "MQTT message received");
            if (message_callback_) {
                std::string topic(event->topic, event->topic_len);
                std::string payload(event->data, event->data_len);
                message_callback_(topic, payload);
            }
            break;
            
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error occurred");
            setState(ConnectionState::ERROR, "MQTT error occurred");
            break;
            
        default:
            ESP_LOGD(TAG, "MQTT event: %d", event->event_id);
            break;
    }
}

void MQTTClient::setState(ConnectionState state, const std::string& message) {
    if (current_state_ != state) {
        current_state_ = state;
        ESP_LOGI(TAG, "MQTT state changed to: %d, message: %s", static_cast<int>(state), message.c_str());
        
        if (state_callback_) {
            state_callback_(state, message);
        }
    }
}

std::string MQTTClient::generateClientId() {
    uint8_t mac[6];
    esp_err_t ret = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    
    std::stringstream ss;
    ss << "ESP32P4CAM_";
    
    if (ret != ESP_OK) {
        // ESP32P4 doesn't have WiFi MAC, use base MAC or chip ID
        ESP_LOGW(TAG, "WiFi MAC not available, using base MAC");
        ret = esp_read_mac(mac, ESP_MAC_BASE);
        
        if (ret != ESP_OK) {
            // If base MAC also fails, use chip ID
            ESP_LOGW(TAG, "Base MAC not available, using chip ID");
            uint64_t chip_id = esp_random();
            for (int i = 0; i < 6; ++i) {
                mac[i] = (chip_id >> (i * 8)) & 0xFF;
            }
        }
    }
    
    for (int i = 0; i < 6; ++i) {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(mac[i]);
    }
    
    return ss.str();
}