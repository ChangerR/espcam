#include "network_camera.hpp"
#include "esp_log.h"
#include "cJSON.h"
#include <functional>

static const char *TAG = "NetworkCamera";

NetworkCamera* NetworkCamera::instance_ = nullptr;

NetworkCamera::NetworkCamera() 
    : current_state_(SystemState::INITIALIZING)
    , device_serial_(CONFIG_ESPCAM_DEVICE_SERIAL)
    , heartbeat_timer_(nullptr)
{
    instance_ = this;
}

NetworkCamera::~NetworkCamera() {
    stop();
    instance_ = nullptr;
}

esp_err_t NetworkCamera::initialize() {
    ESP_LOGI(TAG, "Initializing network camera system");
    
    setState(SystemState::INITIALIZING);
    
    esp_err_t ret = initializeComponents();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize components");
        setState(SystemState::ERROR);
        return ret;
    }
    
    ESP_LOGI(TAG, "Network camera system initialized");
    return ESP_OK;
}

esp_err_t NetworkCamera::start() {
    ESP_LOGI(TAG, "Starting network camera");
    
    // Check if WiFi is already provisioned
    if (wifi_provisioning_->isProvisioned()) {
        ESP_LOGI(TAG, "WiFi already provisioned, connecting...");
        return connectToWiFi();
    } else {
        ESP_LOGI(TAG, "WiFi not provisioned, starting provisioning mode");
        return startProvisioning();
    }
}

esp_err_t NetworkCamera::stop() {
    ESP_LOGI(TAG, "Stopping network camera");
    
    if (heartbeat_timer_) {
        xTimerStop(heartbeat_timer_, 0);
        xTimerDelete(heartbeat_timer_, 0);
        heartbeat_timer_ = nullptr;
    }
    
    if (h264_encoder_) {
        h264_encoder_->stop();
        h264_encoder_->deinitialize();
    }
    
    if (camera_controller_) {
        camera_controller_->stopStreaming();
        camera_controller_->deinitialize();
    }
    
    if (mqtt_client_) {
        mqtt_client_->disconnect();
    }
    
    if (wifi_provisioning_) {
        wifi_provisioning_->stopProvisioning();
    }
    
    setState(SystemState::INITIALIZING);
    return ESP_OK;
}

esp_err_t NetworkCamera::initializeComponents() {
    // Initialize flash storage
    flash_storage_ = std::make_unique<FlashStorage>();
    ESP_ERROR_CHECK(flash_storage_->initialize());
    
    // Load or set device serial number
    std::string stored_serial, device_name;
    if (!flash_storage_->loadDeviceConfig(stored_serial, device_name)) {
        ESP_LOGI(TAG, "Setting default device serial: %s", device_serial_.c_str());
        flash_storage_->saveDeviceConfig(device_serial_, "ESPCam_Device");
    } else {
        device_serial_ = stored_serial;
        ESP_LOGI(TAG, "Loaded device serial: %s", device_serial_.c_str());
    }
    
    // Initialize WiFi provisioning
    wifi_provisioning_ = std::make_unique<WiFiProvisioning>();
    ESP_ERROR_CHECK(wifi_provisioning_->initialize());
    wifi_provisioning_->setCallback(
        std::bind(&NetworkCamera::onWiFiProvisioningStateChanged, this, 
                 std::placeholders::_1, std::placeholders::_2)
    );
    
    // Initialize MQTT client
    mqtt_client_ = std::make_unique<MQTTClient>();
    mqtt_client_->setStateCallback(
        std::bind(&NetworkCamera::onMQTTStateChanged, this,
                 std::placeholders::_1, std::placeholders::_2)
    );
    mqtt_client_->setMessageCallback(
        std::bind(&NetworkCamera::onMQTTMessage, this,
                 std::placeholders::_1, std::placeholders::_2)
    );
    
    // Initialize camera controller
    camera_controller_ = std::make_unique<CameraController>();
    CameraController::CameraConfig camera_config;
    ESP_ERROR_CHECK(camera_controller_->initialize(camera_config));
    camera_controller_->setStateCallback(
        std::bind(&NetworkCamera::onCameraStateChanged, this,
                 std::placeholders::_1, std::placeholders::_2)
    );
    
    // Initialize H.264 encoder
    h264_encoder_ = std::make_unique<H264Encoder>();
    H264Encoder::EncoderConfig encoder_config;
    encoder_config.width = 640;  // Reduce resolution for initial testing
    encoder_config.height = 480;
    encoder_config.fps = 15;     // Reduce FPS for lower memory usage
    encoder_config.bitrate = 1000000; // 1 Mbps
    encoder_config.profile = H264Encoder::Profile::HIGH;
    encoder_config.input_buffer_count = 1;  // Minimal buffer count for testing
    encoder_config.output_buffer_count = 2; // Minimal buffer count
    encoder_config.max_frame_size = 64 * 1024; // 64KB max frame for testing
    
    ESP_ERROR_CHECK(h264_encoder_->initialize(encoder_config));
    h264_encoder_->setFrameCallback(
        std::bind(&NetworkCamera::onH264Frame, this, std::placeholders::_1)
    );
    h264_encoder_->setStateCallback(
        std::bind(&NetworkCamera::onH264StateChanged, this,
                 std::placeholders::_1, std::placeholders::_2)
    );
    
    // Create heartbeat timer
    heartbeat_timer_ = xTimerCreate(
        "heartbeat_timer",
        pdMS_TO_TICKS(CONFIG_ESPCAM_HEARTBEAT_INTERVAL * 1000),
        pdTRUE,  // Auto-reload
        this,
        heartbeatTimerCallback
    );
    
    if (!heartbeat_timer_) {
        ESP_LOGE(TAG, "Failed to create heartbeat timer");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "All components initialized successfully");
    return ESP_OK;
}

esp_err_t NetworkCamera::startProvisioning() {
    ESP_LOGI(TAG, "Starting WiFi provisioning");
    setState(SystemState::PROVISIONING);
    
    return wifi_provisioning_->startProvisioning();
}

esp_err_t NetworkCamera::connectToWiFi() {
    ESP_LOGI(TAG, "Connecting to WiFi");
    setState(SystemState::CONNECTING_WIFI);
    
    bool connected = wifi_provisioning_->connectToSavedWiFi();
    if (!connected) {
        ESP_LOGE(TAG, "Failed to connect to saved WiFi");
        setState(SystemState::ERROR);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t NetworkCamera::connectToMQTT() {
    ESP_LOGI(TAG, "Connecting to MQTT broker");
    setState(SystemState::CONNECTING_MQTT);
    
    MQTTClient::MQTTConfig mqtt_config;
    
    // Try to load MQTT config from flash, otherwise use defaults
    if (!flash_storage_->loadMQTTConfig(mqtt_config.broker_url, mqtt_config.username, mqtt_config.password)) {
        mqtt_config.broker_url = CONFIG_ESPCAM_MQTT_BROKER_URL;
        mqtt_config.username = "";
        mqtt_config.password = "";
        ESP_LOGI(TAG, "Using default MQTT configuration");
    } else {
        ESP_LOGI(TAG, "Loaded MQTT configuration from flash");
    }
    
    mqtt_config.device_serial = device_serial_;
    mqtt_config.client_id = "espcam_" + device_serial_;
    
    ESP_ERROR_CHECK(mqtt_client_->initialize(mqtt_config));
    return mqtt_client_->connect();
}

esp_err_t NetworkCamera::startCamera() {
    ESP_LOGI(TAG, "Starting camera streaming and H.264 encoding");
    
    // Start H.264 encoder first
    esp_err_t ret = h264_encoder_->start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start H.264 encoder: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Start camera streaming
    ret = camera_controller_->startStreaming(
        std::bind(&NetworkCamera::onCameraFrame, this, std::placeholders::_1)
    );
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start camera streaming: %s", esp_err_to_name(ret));
        h264_encoder_->stop();
        return ret;
    }
    
    ESP_LOGI(TAG, "Camera and H.264 encoder started successfully");
    return ESP_OK;
}

void NetworkCamera::onWiFiProvisioningStateChanged(WiFiProvisioning::ProvisioningState state, const std::string& message) {
    ESP_LOGI(TAG, "WiFi provisioning state: %d - %s", (int)state, message.c_str());
    
    switch (state) {
        case WiFiProvisioning::ProvisioningState::CREDENTIALS_RECEIVED:
            ESP_LOGI(TAG, "WiFi credentials received, switching to STA mode");
            // Stop the provisioning AP and HTTP server
            wifi_provisioning_->stopProvisioning();
            // Start connecting to the configured WiFi
            connectToWiFi();
            break;
            
        case WiFiProvisioning::ProvisioningState::CONNECTED:
            ESP_LOGI(TAG, "WiFi connected successfully");
            connectToMQTT();
            break;
            
        case WiFiProvisioning::ProvisioningState::FAILED:
            setState(SystemState::ERROR);
            break;
            
        default:
            break;
    }
}

void NetworkCamera::onMQTTStateChanged(MQTTClient::ConnectionState state, const std::string& message) {
    ESP_LOGI(TAG, "MQTT state: %d - %s", (int)state, message.c_str());
    
    switch (state) {
        case MQTTClient::ConnectionState::CONNECTED:
            setState(SystemState::RUNNING);
            startCamera();
            xTimerStart(heartbeat_timer_, 0);
            sendHeartbeat(); // Send initial heartbeat
            break;
            
        case MQTTClient::ConnectionState::DISCONNECTED:
        case MQTTClient::ConnectionState::ERROR:
            xTimerStop(heartbeat_timer_, 0);
            if (h264_encoder_) {
                h264_encoder_->stop();
            }
            if (camera_controller_) {
                camera_controller_->stopStreaming();
            }
            if (current_state_ == SystemState::RUNNING) {
                setState(SystemState::CONNECTING_MQTT);
                // Attempt reconnection
                vTaskDelay(pdMS_TO_TICKS(5000));
                mqtt_client_->connect();
            }
            break;
            
        default:
            break;
    }
}

void NetworkCamera::onCameraStateChanged(CameraController::CameraState state, const std::string& message) {
    ESP_LOGI(TAG, "Camera state: %d - %s", (int)state, message.c_str());
    
    if (state == CameraController::CameraState::ERROR && current_state_ == SystemState::RUNNING) {
        ESP_LOGE(TAG, "Camera error in running state");
        // Could attempt camera restart here
    }
}

void NetworkCamera::onMQTTMessage(const std::string& topic, const std::string& payload) {
    ESP_LOGI(TAG, "MQTT message received on topic: %s", topic.c_str());
    handleCommand(topic, payload);
}

void NetworkCamera::onCameraFrame(camera_fb_t* frame) {
    if (!frame) {
        return;
    }
    
    ESP_LOGD(TAG, "Frame captured: %zu bytes, %dx%d, format: %d", 
             frame->len, frame->width, frame->height, frame->format);
    
    // Send frame to H.264 encoder for hardware encoding
    if (h264_encoder_ && h264_encoder_->isEncoding()) {
        esp_err_t ret = h264_encoder_->encodeFrame(frame);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to encode frame: %s", esp_err_to_name(ret));
        }
    }
    
    // Optionally publish frame metadata (not the actual frame data)
    if (mqtt_client_->isConnected()) {
        cJSON* frame_info = cJSON_CreateObject();
        cJSON_AddNumberToObject(frame_info, "timestamp", esp_log_timestamp());
        cJSON_AddNumberToObject(frame_info, "size", frame->len);
        cJSON_AddNumberToObject(frame_info, "width", frame->width);
        cJSON_AddNumberToObject(frame_info, "height", frame->height);
        cJSON_AddNumberToObject(frame_info, "format", frame->format);
        
        char* json_string = cJSON_Print(frame_info);
        if (json_string) {
            std::string status_topic = mqtt_client_->getStatusTopic();
            mqtt_client_->publish(status_topic + "/frame_info", json_string, 0, false);
            free(json_string);
        }
        cJSON_Delete(frame_info);
    }
}

void NetworkCamera::heartbeatTimerCallback(TimerHandle_t timer) {
    NetworkCamera* self = static_cast<NetworkCamera*>(pvTimerGetTimerID(timer));
    if (self) {
        self->sendHeartbeat();
    }
}

void NetworkCamera::sendHeartbeat() {
    if (!mqtt_client_->isConnected()) {
        return;
    }
    
    ESP_LOGD(TAG, "Sending heartbeat");
    
    esp_err_t result = mqtt_client_->publishHeartbeat();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send heartbeat");
    }
}

void NetworkCamera::handleCommand(const std::string& topic, const std::string& payload) {
    ESP_LOGI(TAG, "Handling command from topic: %s", topic.c_str());
    
    // Parse command from topic (e.g., devices/SERIAL/commands/COMMAND)
    size_t last_slash = topic.find_last_of('/');
    if (last_slash == std::string::npos) {
        ESP_LOGW(TAG, "Invalid command topic format");
        return;
    }
    
    std::string command = topic.substr(last_slash + 1);
    ESP_LOGI(TAG, "Command: %s, Payload: %s", command.c_str(), payload.c_str());
    
    cJSON* json = cJSON_Parse(payload.c_str());
    if (!json) {
        ESP_LOGW(TAG, "Invalid JSON payload");
        return;
    }
    
    if (command == "capture") {
        camera_fb_t* frame = camera_controller_->captureFrame();
        if (frame) {
            // In a real implementation, you would encode and send the frame
            ESP_LOGI(TAG, "Frame captured on command: %zu bytes", frame->len);
            camera_controller_->returnFrame(frame);
        }
    } else if (command == "settings") {
        cJSON* quality = cJSON_GetObjectItem(json, "quality");
        if (quality && cJSON_IsNumber(quality)) {
            camera_controller_->setJpegQuality(quality->valueint);
        }
        
        cJSON* effect = cJSON_GetObjectItem(json, "effect");
        if (effect && cJSON_IsNumber(effect)) {
            camera_controller_->setSpecialEffect(effect->valueint);
        }
    } else if (command == "restart") {
        ESP_LOGI(TAG, "Restart command received");
        esp_restart();
    }
    
    cJSON_Delete(json);
}

void NetworkCamera::onH264Frame(const H264Encoder::FrameInfo& frame) {
    ESP_LOGD(TAG, "H.264 frame encoded: %zu bytes, keyframe: %d, frame: %lu",
             frame.size, frame.is_keyframe, (unsigned long)frame.frame_number);
    
    // Publish encoded H.264 frame via MQTT
    if (mqtt_client_->isConnected()) {
        std::string video_topic = mqtt_client_->getStatusTopic() + "/video";
        
        // For MQTT publishing, we might want to base64 encode the data or use binary publishing
        // For now, just publish the frame statistics
        cJSON* h264_info = cJSON_CreateObject();
        cJSON_AddNumberToObject(h264_info, "timestamp", frame.timestamp_us);
        cJSON_AddNumberToObject(h264_info, "size", frame.size);
        cJSON_AddNumberToObject(h264_info, "frame_number", frame.frame_number);
        cJSON_AddBoolToObject(h264_info, "keyframe", frame.is_keyframe);
        
        char* json_string = cJSON_Print(h264_info);
        if (json_string) {
            mqtt_client_->publish(video_topic, json_string, 0, false);
            free(json_string);
        }
        cJSON_Delete(h264_info);
        
        // TODO: In a real implementation, you might:
        // 1. Store H.264 frames in a ring buffer for HTTP streaming
        // 2. Send frames via WebRTC for real-time streaming
        // 3. Upload to cloud storage for recording
        // 4. Stream via RTMP/RTSP protocols
    }
}

void NetworkCamera::onH264StateChanged(H264Encoder::EncoderState state, const std::string& message) {
    ESP_LOGI(TAG, "H.264 encoder state: %d - %s", (int)state, message.c_str());
    
    // Handle encoder state changes if needed
    if (state == H264Encoder::EncoderState::ERROR) {
        ESP_LOGE(TAG, "H.264 encoder error, attempting restart");
        // Could implement error recovery here
    }
}

void NetworkCamera::setState(SystemState state) {
    current_state_ = state;
    ESP_LOGI(TAG, "System state changed to: %d", (int)state);
}