#pragma once

#include <memory>
#include <string>
#include "wifi_provisioning.hpp"
#include "flash_storage.hpp"
#include "mqtt_client.hpp"
#include "camera_controller.hpp"
#include "h264_encoder.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

class NetworkCamera {
public:
    enum class SystemState {
        INITIALIZING,
        PROVISIONING,
        CONNECTING_WIFI,
        CONNECTING_MQTT,
        RUNNING,
        IDLE,
        ERROR
    };

    NetworkCamera();
    ~NetworkCamera();

    esp_err_t initialize();
    esp_err_t start();
    esp_err_t stop();
    
    SystemState getState() const { return current_state_; }

private:
    void onWiFiProvisioningStateChanged(WiFiProvisioning::ProvisioningState state, const std::string& message);
    void onMQTTStateChanged(MQTTClient::ConnectionState state, const std::string& message);
    void onCameraStateChanged(CameraController::CameraState state, const std::string& message);
    void onMQTTMessage(const std::string& topic, const std::string& payload);
    void onCameraFrame(camera_fb_t* frame);
    void onH264Frame(const H264Encoder::FrameInfo& frame);
    void onH264StateChanged(H264Encoder::EncoderState state, const std::string& message);
    
    static void heartbeatTimerCallback(TimerHandle_t timer);
    void sendHeartbeat();
    
    void setState(SystemState state);
    void handleCommand(const std::string& topic, const std::string& payload);
    
    esp_err_t initializeComponents();
    esp_err_t startProvisioning();
    esp_err_t connectToWiFi();
    esp_err_t connectToMQTT();
    esp_err_t startCamera();
    
    std::unique_ptr<WiFiProvisioning> wifi_provisioning_;
    std::unique_ptr<FlashStorage> flash_storage_;
    std::unique_ptr<MQTTClient> mqtt_client_;
    std::unique_ptr<CameraController> camera_controller_;
    std::unique_ptr<H264Encoder> h264_encoder_;
    
    SystemState current_state_;
    std::string device_serial_;
    TimerHandle_t heartbeat_timer_;
    
    static NetworkCamera* instance_;
};