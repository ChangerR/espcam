#pragma once

#include <string>
#include <functional>
#include <memory>
#include "mqtt_client.h"
#include "esp_err.h"

class MQTTClient {
public:
    enum class ConnectionState {
        DISCONNECTED,
        CONNECTING,
        CONNECTED,
        ERROR
    };

    using MessageCallback = std::function<void(const std::string& topic, const std::string& payload)>;
    using StateCallback = std::function<void(ConnectionState state, const std::string& message)>;

    struct MQTTConfig {
        std::string broker_url;
        std::string username;
        std::string password;
        std::string client_id;
        std::string device_serial;
        int port = 1883;
        bool use_ssl = false;
        int keepalive = 60;
        int session_expiry_interval = 3600;
    };

    MQTTClient();
    ~MQTTClient();

    esp_err_t initialize(const MQTTConfig& config);
    esp_err_t connect();
    esp_err_t disconnect();
    
    esp_err_t publish(const std::string& topic, const std::string& payload, int qos = 0, bool retain = false);
    esp_err_t subscribe(const std::string& topic, int qos = 0);
    esp_err_t unsubscribe(const std::string& topic);
    
    esp_err_t publishHeartbeat();
    esp_err_t subscribeToDeviceTopics();
    
    void setMessageCallback(MessageCallback callback);
    void setStateCallback(StateCallback callback);
    
    ConnectionState getState() const { return current_state_; }
    bool isConnected() const { return current_state_ == ConnectionState::CONNECTED; }
    
    std::string getHeartbeatTopic() const;
    std::string getCommandTopic() const;
    std::string getStatusTopic() const;

private:
    static void mqttEventHandler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
    void handleMQTTEvent(esp_mqtt_event_handle_t event);
    
    void setState(ConnectionState state, const std::string& message = "");
    std::string generateClientId();

    esp_mqtt_client_handle_t client_;
    MQTTConfig config_;
    ConnectionState current_state_;
    MessageCallback message_callback_;
    StateCallback state_callback_;
    
    static MQTTClient* instance_;
};