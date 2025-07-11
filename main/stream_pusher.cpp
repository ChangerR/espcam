#include "stream_pusher.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_http_client.h"
#include "esp_tls.h"
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cmath>

const char* StreamPusher::TAG = "StreamPusher";

// RTMP connection implementation
struct StreamPusher::RTMPConnection {
    esp_http_client_handle_t client = nullptr;
    std::string rtmp_url;
    std::string stream_key;
    bool connected = false;
    uint64_t connect_time = 0;
    uint64_t last_send_time = 0;
    uint32_t sequence_number = 0;
    
    // RTMP handshake state
    enum HandshakeState {
        HANDSHAKE_INIT,
        HANDSHAKE_C0_SENT,
        HANDSHAKE_C1_SENT,
        HANDSHAKE_C2_SENT,
        HANDSHAKE_COMPLETE
    } handshake_state = HANDSHAKE_INIT;
    
    // RTMP stream state
    bool stream_started = false;
    uint32_t video_timestamp = 0;
    uint32_t audio_timestamp = 0;
    
    ~RTMPConnection() {
        if (client) {
            esp_http_client_cleanup(client);
        }
    }
};

StreamPusher::StreamPusher()
    : current_state_(StreamState::IDLE)
    , mqtt_client_(nullptr)
    , encoder_(std::make_unique<H264Encoder>())
    , rtmp_connection_(std::make_unique<RTMPConnection>())
    , frame_queue_(nullptr)
    , state_mutex_(nullptr)
    , metrics_mutex_(nullptr)
    , stream_task_handle_(nullptr)
    , health_check_task_handle_(nullptr)
    , reconnect_timer_(nullptr)
    , metrics_timer_(nullptr)
    , tasks_running_(false)
    , stream_start_time_(0)
    , last_frame_time_(0)
    , frame_counter_(0)
    , keyframe_counter_(0)
    , reconnect_attempts_(0)
    , target_bitrate_(2000000)
    , current_bitrate_(2000000)
    , network_quality_(1.0f)
{
    // Initialize mutexes
    state_mutex_ = xSemaphoreCreateMutex();
    metrics_mutex_ = xSemaphoreCreateMutex();
    
    if (!state_mutex_ || !metrics_mutex_) {
        ESP_LOGE(TAG, "Failed to create mutexes");
    }
    
    // Initialize metrics
    memset(&metrics_, 0, sizeof(metrics_));
    memset(&stream_status_, 0, sizeof(stream_status_));
    
    // Set up encoder callback
    encoder_->setFrameCallback(
        std::bind(&StreamPusher::onEncodedFrame, this, std::placeholders::_1)
    );
    
    ESP_LOGI(TAG, "StreamPusher created");
}

StreamPusher::~StreamPusher() {
    deinitialize();
    
    if (state_mutex_) {
        vSemaphoreDelete(state_mutex_);
    }
    if (metrics_mutex_) {
        vSemaphoreDelete(metrics_mutex_);
    }
    
    ESP_LOGI(TAG, "StreamPusher destroyed");
}

esp_err_t StreamPusher::initialize(const StreamConfig& config, MQTTClient* mqtt_client) {
    if (current_state_ != StreamState::IDLE) {
        ESP_LOGE(TAG, "StreamPusher already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    config_ = config;
    mqtt_client_ = mqtt_client;
    
    // Validate configuration
    esp_err_t ret = validateStreamConfig();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Invalid stream configuration");
        return ret;
    }
    
    // Initialize encoder
    H264Encoder::EncoderConfig encoder_config;
    encoder_config.width = (config_.quality == StreamQuality::CUSTOM) ? 
                          config_.custom_width : getDefaultWidth(config_.quality);
    encoder_config.height = (config_.quality == StreamQuality::CUSTOM) ? 
                           config_.custom_height : getDefaultHeight(config_.quality);
    encoder_config.fps = (config_.quality == StreamQuality::CUSTOM) ? 
                        config_.custom_fps : getDefaultFPS(config_.quality);
    encoder_config.bitrate = (config_.quality == StreamQuality::CUSTOM) ? 
                            config_.custom_bitrate : getDefaultBitrate(config_.quality);
    encoder_config.keyframe_interval = encoder_config.fps * 2; // Keyframe every 2 seconds
    encoder_config.enable_low_latency = true;
    encoder_config.enable_rate_control = true;
    
    ret = encoder_->initialize(encoder_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize encoder: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create frame queue
    frame_queue_ = xQueueCreate(config_.max_frame_queue_size, sizeof(camera_fb_t*));
    if (!frame_queue_) {
        ESP_LOGE(TAG, "Failed to create frame queue");
        return ESP_ERR_NO_MEM;
    }
    
    // Create tasks
    ret = createTasks();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create tasks: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create timers
    reconnect_timer_ = xTimerCreate("reconnect_timer", 
                                   pdMS_TO_TICKS(config_.reconnect_interval_sec * 1000),
                                   pdFALSE, this, reconnectTimerCallback);
    
    metrics_timer_ = xTimerCreate("metrics_timer",
                                 pdMS_TO_TICKS(1000), // Update every second
                                 pdTRUE, this, metricsTimerCallback);
    
    if (!reconnect_timer_ || !metrics_timer_) {
        ESP_LOGE(TAG, "Failed to create timers");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize stream status
    stream_status_.state = StreamState::IDLE;
    stream_status_.network_available = true;
    stream_status_.network_quality = 1.0f;
    
    // Store initial configuration
    target_bitrate_ = encoder_config.bitrate;
    current_bitrate_ = encoder_config.bitrate;
    
    ESP_LOGI(TAG, "StreamPusher initialized successfully");
    ESP_LOGI(TAG, "Stream quality: %s (%dx%d@%dfps, %dkbps)", 
             getQualityString(config_.quality).c_str(),
             encoder_config.width, encoder_config.height, 
             encoder_config.fps, encoder_config.bitrate / 1000);
    
    return ESP_OK;
}

esp_err_t StreamPusher::start() {
    if (current_state_ != StreamState::IDLE) {
        ESP_LOGE(TAG, "StreamPusher not in idle state");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Start encoder
    esp_err_t ret = encoder_->start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start encoder: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Start tasks
    tasks_running_ = true;
    
    // Start metrics timer
    xTimerStart(metrics_timer_, 0);
    
    setState(StreamState::IDLE, "StreamPusher started, ready to stream");
    
    ESP_LOGI(TAG, "StreamPusher started successfully");
    return ESP_OK;
}

esp_err_t StreamPusher::startStream(const std::string& stream_key) {
    if (current_state_ != StreamState::IDLE) {
        ESP_LOGE(TAG, "Cannot start stream in current state");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Use provided stream key or generate one
    if (!stream_key.empty()) {
        server_info_.stream_key = stream_key;
    } else if (server_info_.stream_key.empty()) {
        server_info_.stream_key = generateStreamKey();
    }
    
    // Check if we have server info
    if (server_info_.rtmp_url.empty()) {
        // Request stream server via MQTT
        ESP_LOGI(TAG, "Requesting stream server via MQTT");
        setState(StreamState::REQUESTING_SERVER, "Requesting stream server");
        return requestStreamServer();
    }
    
    // Connect to RTMP server
    setState(StreamState::CONNECTING, "Connecting to RTMP server");
    esp_err_t ret = connectToRTMP();
    if (ret != ESP_OK) {
        setState(StreamState::ERROR, "Failed to connect to RTMP server");
        return ret;
    }
    
    // Start streaming
    stream_start_time_ = getCurrentTimestamp();
    last_frame_time_ = stream_start_time_;
    frame_counter_ = 0;
    keyframe_counter_ = 0;
    reconnect_attempts_ = 0;
    
    setState(StreamState::STREAMING, "Streaming started");
    
    // Publish stream status
    publishStreamStatus("stream_started", "Live streaming started");
    
    ESP_LOGI(TAG, "Stream started successfully to %s", server_info_.rtmp_url.c_str());
    return ESP_OK;
}

esp_err_t StreamPusher::stopStream() {
    if (current_state_ != StreamState::STREAMING && 
        current_state_ != StreamState::RECONNECTING) {
        ESP_LOGE(TAG, "No active stream to stop");
        return ESP_ERR_INVALID_STATE;
    }
    
    setState(StreamState::STOPPING, "Stopping stream");
    
    // Stop reconnection attempts
    if (reconnect_timer_) {
        xTimerStop(reconnect_timer_, 0);
    }
    
    // Disconnect from RTMP server
    esp_err_t ret = disconnectFromRTMP();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Error disconnecting from RTMP server: %s", esp_err_to_name(ret));
    }
    
    // Update metrics
    if (stream_start_time_ > 0) {
        uint64_t duration = getCurrentTimestamp() - stream_start_time_;
        metrics_.stream_duration_ms = duration / 1000;
        
        if (duration > 0) {
            metrics_.average_bitrate_kbps = (double)(metrics_.total_bytes_sent * 8) / (duration / 1000.0) / 1000.0;
        }
    }
    
    setState(StreamState::IDLE, "Stream stopped");
    
    // Publish stream status
    publishStreamStatus("stream_stopped", "Live streaming stopped");
    
    ESP_LOGI(TAG, "Stream stopped successfully");
    return ESP_OK;
}

esp_err_t StreamPusher::pushFrame(camera_fb_t* frame) {
    if (!frame) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (current_state_ != StreamState::STREAMING) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check if frame queue is full
    if (uxQueueMessagesWaiting(frame_queue_) >= config_.frame_drop_threshold) {
        if (config_.enable_frame_dropping) {
            // Drop oldest frame
            camera_fb_t* dropped_frame;
            if (xQueueReceive(frame_queue_, &dropped_frame, 0) == pdTRUE) {
                metrics_.dropped_frames++;
                ESP_LOGD(TAG, "Dropped frame to prevent queue overflow");
            }
        } else {
            ESP_LOGW(TAG, "Frame queue full, dropping frame");
            metrics_.dropped_frames++;
            return ESP_ERR_NO_MEM;
        }
    }
    
    // Add frame to queue
    if (xQueueSend(frame_queue_, &frame, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to queue frame");
        metrics_.dropped_frames++;
        return ESP_ERR_NO_MEM;
    }
    
    return ESP_OK;
}

esp_err_t StreamPusher::pushH264Frame(const uint8_t* h264_data, size_t data_size, 
                                     bool is_keyframe, uint64_t timestamp_us) {
    if (!h264_data || data_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (current_state_ != StreamState::STREAMING) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Send frame to RTMP server
    esp_err_t ret = sendRTMPFrame(h264_data, data_size, is_keyframe, timestamp_us);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send H264 frame to RTMP server: %s", esp_err_to_name(ret));
        metrics_.connection_errors++;
        
        // Trigger reconnection if enabled
        if (config_.enable_auto_reconnect && reconnect_attempts_ < config_.max_reconnect_attempts) {
            setState(StreamState::RECONNECTING, "Connection lost, attempting reconnection");
            xTimerStart(reconnect_timer_, 0);
        } else {
            setState(StreamState::ERROR, "Stream connection failed");
        }
        
        return ret;
    }
    
    // Update metrics
    metrics_.total_frames_sent++;
    metrics_.total_bytes_sent += data_size;
    metrics_.last_frame_timestamp = timestamp_us;
    
    if (is_keyframe) {
        keyframe_counter_++;
        stream_status_.last_keyframe_time = getCurrentTimestamp();
    }
    
    frame_counter_++;
    last_frame_time_ = getCurrentTimestamp();
    
    return ESP_OK;
}

esp_err_t StreamPusher::requestStreamServer(const std::string& preferred_region) {
    if (!mqtt_client_) {
        ESP_LOGE(TAG, "MQTT client not available");
        return ESP_ERR_INVALID_STATE;
    }
    
    setState(StreamState::REQUESTING_SERVER, "Requesting stream server");
    
    // Construct MQTT request
    std::ostringstream json;
    json << "{";
    json << "\"request_id\":\"stream_" << std::hex << getCurrentTimestamp() << "\",";
    json << "\"device_id\":\"ESP32P4_CAM_001\",";
    json << "\"stream_type\":\"rtmp\",";
    json << "\"quality\":\"" << getQualityString(config_.quality) << "\",";
    json << "\"preferred_region\":\"" << preferred_region << "\",";
    json << "\"max_bitrate\":" << target_bitrate_ << ",";
    json << "\"supports_adaptive\":" << (config_.enable_adaptive_bitrate ? "true" : "false") << ",";
    json << "\"timestamp\":" << (getCurrentTimestamp() / 1000000);
    json << "}";
    
    // Replace {DEVICE_ID} in topic
    std::string topic = config_.mqtt_request_topic;
    size_t pos = topic.find("{DEVICE_ID}");
    if (pos != std::string::npos) {
        topic.replace(pos, 11, "ESP32P4_CAM_001");
    }
    
    // Publish request
    esp_err_t ret = mqtt_client_->publish(topic, json.str(), 1, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to publish stream request: %s", esp_err_to_name(ret));
        setState(StreamState::ERROR, "Failed to request stream server");
        return ret;
    }
    
    ESP_LOGI(TAG, "Stream server request sent");
    return ESP_OK;
}

esp_err_t StreamPusher::connectToRTMP() {
    if (!rtmp_connection_) {
        ESP_LOGE(TAG, "RTMP connection not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Disconnect if already connected
    if (rtmp_connection_->connected) {
        disconnectFromRTMP();
    }
    
    // Configure HTTP client for RTMP
    esp_http_client_config_t config = {};
    config.url = server_info_.rtmp_url.c_str();
    config.timeout_ms = RTMP_CONNECT_TIMEOUT_MS;
    config.disable_auto_redirect = true;
    config.max_redirection_count = 0;
    
    rtmp_connection_->client = esp_http_client_init(&config);
    if (!rtmp_connection_->client) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client for RTMP");
        return ESP_ERR_NO_MEM;
    }
    
    // Set custom headers for RTMP
    esp_http_client_set_header(rtmp_connection_->client, "Content-Type", "video/x-flv");
    esp_http_client_set_method(rtmp_connection_->client, HTTP_METHOD_POST);
    
    // Perform RTMP handshake (simplified)
    esp_err_t ret = performRTMPHandshake();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RTMP handshake failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    rtmp_connection_->connected = true;
    rtmp_connection_->connect_time = getCurrentTimestamp();
    rtmp_connection_->rtmp_url = server_info_.rtmp_url;
    rtmp_connection_->stream_key = server_info_.stream_key;
    
    ESP_LOGI(TAG, "RTMP connection established to %s", server_info_.rtmp_url.c_str());
    return ESP_OK;
}

esp_err_t StreamPusher::disconnectFromRTMP() {
    if (!rtmp_connection_) {
        return ESP_OK;
    }
    
    if (rtmp_connection_->client) {
        esp_http_client_cleanup(rtmp_connection_->client);
        rtmp_connection_->client = nullptr;
    }
    
    rtmp_connection_->connected = false;
    rtmp_connection_->stream_started = false;
    rtmp_connection_->handshake_state = RTMPConnection::HANDSHAKE_INIT;
    
    ESP_LOGI(TAG, "RTMP connection closed");
    return ESP_OK;
}

esp_err_t StreamPusher::performRTMPHandshake() {
    // Simplified RTMP handshake implementation
    // In a real implementation, this would include proper RTMP protocol handshake
    
    ESP_LOGI(TAG, "Performing RTMP handshake");
    
    // Simulate handshake process
    rtmp_connection_->handshake_state = RTMPConnection::HANDSHAKE_COMPLETE;
    
    // Send connect command
    esp_err_t ret = sendRTMPConnectCommand();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send RTMP connect command");
        return ret;
    }
    
    // Send publish command
    ret = sendRTMPPublishCommand();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send RTMP publish command");
        return ret;
    }
    
    rtmp_connection_->stream_started = true;
    
    ESP_LOGI(TAG, "RTMP handshake completed successfully");
    return ESP_OK;
}

esp_err_t StreamPusher::sendRTMPConnectCommand() {
    // Simplified RTMP connect command
    // In a real implementation, this would send proper RTMP connect packet
    
    ESP_LOGD(TAG, "Sending RTMP connect command");
    
    // For now, just simulate success
    return ESP_OK;
}

esp_err_t StreamPusher::sendRTMPPublishCommand() {
    // Simplified RTMP publish command
    // In a real implementation, this would send proper RTMP publish packet
    
    ESP_LOGD(TAG, "Sending RTMP publish command for stream: %s", 
             server_info_.stream_key.c_str());
    
    // For now, just simulate success
    return ESP_OK;
}

esp_err_t StreamPusher::sendRTMPFrame(const uint8_t* h264_data, size_t data_size, 
                                     bool is_keyframe, uint64_t timestamp_us) {
    if (!rtmp_connection_ || !rtmp_connection_->connected) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Convert timestamp to RTMP format
    uint32_t rtmp_timestamp = (uint32_t)(timestamp_us / 1000); // Convert to milliseconds
    
    // Create RTMP video packet (simplified)
    std::vector<uint8_t> rtmp_packet;
    rtmp_packet.reserve(data_size + 16); // Extra space for RTMP headers
    
    // RTMP packet header (simplified)
    rtmp_packet.push_back(0x09); // Video tag
    rtmp_packet.push_back((data_size >> 16) & 0xFF); // Data size
    rtmp_packet.push_back((data_size >> 8) & 0xFF);
    rtmp_packet.push_back(data_size & 0xFF);
    rtmp_packet.push_back((rtmp_timestamp >> 16) & 0xFF); // Timestamp
    rtmp_packet.push_back((rtmp_timestamp >> 8) & 0xFF);
    rtmp_packet.push_back(rtmp_timestamp & 0xFF);
    rtmp_packet.push_back((rtmp_timestamp >> 24) & 0xFF); // Extended timestamp
    rtmp_packet.push_back(0x00); // Stream ID
    rtmp_packet.push_back(0x00);
    rtmp_packet.push_back(0x00);
    
    // Video data
    rtmp_packet.push_back(is_keyframe ? 0x17 : 0x27); // Frame type + codec
    rtmp_packet.push_back(0x01); // AVC NALU
    rtmp_packet.push_back(0x00); // Composition time
    rtmp_packet.push_back(0x00);
    rtmp_packet.push_back(0x00);
    
    // Add H.264 data
    rtmp_packet.insert(rtmp_packet.end(), h264_data, h264_data + data_size);
    
    // Send packet via HTTP client (simplified)
    esp_err_t ret = esp_http_client_open(rtmp_connection_->client, rtmp_packet.size());
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection for RTMP");
        return ret;
    }
    
    int written = esp_http_client_write(rtmp_connection_->client, 
                                       (const char*)rtmp_packet.data(), 
                                       rtmp_packet.size());
    if (written < 0) {
        ESP_LOGE(TAG, "Failed to write RTMP packet");
        esp_http_client_close(rtmp_connection_->client);
        return ESP_FAIL;
    }
    
    esp_http_client_close(rtmp_connection_->client);
    
    rtmp_connection_->last_send_time = getCurrentTimestamp();
    rtmp_connection_->sequence_number++;
    
    ESP_LOGD(TAG, "RTMP frame sent: %d bytes, keyframe: %s", 
             data_size, is_keyframe ? "yes" : "no");
    
    return ESP_OK;
}

// Task implementations
void StreamPusher::streamTask(void* parameter) {
    StreamPusher* pusher = static_cast<StreamPusher*>(parameter);
    camera_fb_t* frame = nullptr;
    
    ESP_LOGI(TAG, "Stream task started");
    
    while (pusher->tasks_running_) {
        // Wait for frame
        if (xQueueReceive(pusher->frame_queue_, &frame, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (frame && pusher->current_state_ == StreamState::STREAMING) {
                esp_err_t ret = pusher->processFrame(frame);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to process frame: %s", esp_err_to_name(ret));
                }
            }
        }
        
        // Adaptive quality control
        if (pusher->config_.enable_adaptive_bitrate) {
            pusher->adaptQuality();
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    ESP_LOGI(TAG, "Stream task stopped");
    vTaskDelete(nullptr);
}

void StreamPusher::healthCheckTask(void* parameter) {
    StreamPusher* pusher = static_cast<StreamPusher*>(parameter);
    
    ESP_LOGI(TAG, "Health check task started");
    
    while (pusher->tasks_running_) {
        pusher->performHealthCheck();
        pusher->updateNetworkQuality();
        
        vTaskDelay(pdMS_TO_TICKS(pusher->config_.health_check_interval_sec * 1000));
    }
    
    ESP_LOGI(TAG, "Health check task stopped");
    vTaskDelete(nullptr);
}

void StreamPusher::reconnectTimerCallback(TimerHandle_t timer) {
    StreamPusher* pusher = static_cast<StreamPusher*>(pvTimerGetTimerID(timer));
    
    if (pusher->current_state_ == StreamState::RECONNECTING) {
        pusher->reconnect_attempts_++;
        
        if (pusher->reconnect_attempts_ <= pusher->config_.max_reconnect_attempts) {
            ESP_LOGI(TAG, "Attempting reconnection (%d/%d)", 
                     pusher->reconnect_attempts_, pusher->config_.max_reconnect_attempts);
            
            esp_err_t ret = pusher->connectToRTMP();
            if (ret == ESP_OK) {
                pusher->setState(StreamState::STREAMING, "Reconnected successfully");
                pusher->publishStreamStatus("reconnected", "Stream reconnected successfully");
            } else {
                // Retry after delay
                xTimerStart(timer, 0);
            }
        } else {
            pusher->setState(StreamState::ERROR, "Max reconnection attempts reached");
            pusher->publishStreamStatus("connection_failed", "Failed to reconnect to stream server");
        }
    }
}

void StreamPusher::metricsTimerCallback(TimerHandle_t timer) {
    StreamPusher* pusher = static_cast<StreamPusher*>(pvTimerGetTimerID(timer));
    pusher->updateMetrics();
}

esp_err_t StreamPusher::processFrame(camera_fb_t* frame) {
    if (!frame) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check if we should drop this frame
    if (shouldDropFrame()) {
        metrics_.dropped_frames++;
        return ESP_OK;
    }
    
    // Encode frame
    esp_err_t ret = encoder_->encodeFrame(frame);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to encode frame: %s", esp_err_to_name(ret));
        metrics_.encoding_errors++;
        return ret;
    }
    
    return ESP_OK;
}

void StreamPusher::onEncodedFrame(const H264Encoder::FrameInfo& frame_info) {
    if (current_state_ != StreamState::STREAMING) {
        return;
    }
    
    // Send encoded frame to RTMP server
    esp_err_t ret = pushH264Frame(frame_info.data, frame_info.size, 
                                 frame_info.is_keyframe, frame_info.timestamp_us);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to push H264 frame: %s", esp_err_to_name(ret));
    }
}

bool StreamPusher::shouldDropFrame() const {
    // Drop frames if queue is getting full
    uint32_t queue_size = uxQueueMessagesWaiting(frame_queue_);
    if (queue_size >= config_.frame_drop_threshold) {
        return true;
    }
    
    // Drop frames if encoding is taking too long
    uint64_t current_time = getCurrentTimestamp();
    if (current_time - last_frame_time_ > config_.max_encoding_delay_ms * 1000) {
        return true;
    }
    
    return false;
}

esp_err_t StreamPusher::adaptQuality() {
    if (!config_.enable_adaptive_bitrate) {
        return ESP_OK;
    }
    
    // Calculate target bitrate based on network quality
    uint32_t adaptive_bitrate = (uint32_t)(target_bitrate_ * network_quality_);
    
    // Don't change bitrate too frequently
    if (abs((int32_t)(adaptive_bitrate - current_bitrate_)) < target_bitrate_ * 0.1) {
        return ESP_OK;
    }
    
    // Update encoder bitrate
    esp_err_t ret = encoder_->setBitrate(adaptive_bitrate);
    if (ret == ESP_OK) {
        current_bitrate_ = adaptive_bitrate;
        ESP_LOGI(TAG, "Adapted bitrate to %d kbps (network quality: %.2f)", 
                 adaptive_bitrate / 1000, network_quality_);
    }
    
    return ret;
}

void StreamPusher::updateNetworkQuality() {
    // Simple network quality calculation based on recent metrics
    float quality = 1.0f;
    
    // Factor in dropped frames
    if (metrics_.total_frames_sent > 0) {
        float drop_rate = (float)metrics_.dropped_frames / metrics_.total_frames_sent;
        quality *= (1.0f - drop_rate);
    }
    
    // Factor in connection errors
    if (metrics_.connection_errors > 0) {
        quality *= 0.8f; // Reduce quality if there are connection issues
    }
    
    // Factor in latency (if available)
    if (!latency_history_.empty()) {
        uint32_t avg_latency = 0;
        std::queue<uint32_t> temp_queue = latency_history_;
        while (!temp_queue.empty()) {
            avg_latency += temp_queue.front();
            temp_queue.pop();
        }
        avg_latency /= latency_history_.size();
        
        if (avg_latency > config_.target_latency_ms) {
            quality *= 0.7f;
        }
    }
    
    // Smooth the quality change
    network_quality_ = (network_quality_ * 0.8f) + (quality * 0.2f);
    
    // Keep quality within bounds
    network_quality_ = std::max(0.1f, std::min(1.0f, network_quality_));
    
    stream_status_.network_quality = network_quality_;
}

void StreamPusher::updateMetrics() {
    if (current_state_ != StreamState::STREAMING) {
        return;
    }
    
    uint64_t current_time = getCurrentTimestamp();
    
    if (xSemaphoreTake(metrics_mutex_, portMAX_DELAY) == pdTRUE) {
        // Calculate current FPS
        if (stream_start_time_ > 0) {
            uint64_t duration_ms = (current_time - stream_start_time_) / 1000;
            if (duration_ms > 0) {
                metrics_.current_fps = (double)frame_counter_ * 1000.0 / duration_ms;
            }
        }
        
        // Calculate current bitrate
        if (last_frame_time_ > 0) {
            uint64_t duration_ms = (current_time - last_frame_time_) / 1000;
            if (duration_ms > 0) {
                metrics_.current_bitrate_kbps = (double)(metrics_.total_bytes_sent * 8) / duration_ms;
            }
        }
        
        // Update queue size
        metrics_.frame_queue_size = uxQueueMessagesWaiting(frame_queue_);
        
        // Update connection status
        metrics_.is_streaming = (current_state_ == StreamState::STREAMING);
        metrics_.is_connected = rtmp_connection_ && rtmp_connection_->connected;
        
        // Update server info
        metrics_.server_url = server_info_.rtmp_url;
        
        // Store metrics in status
        stream_status_.metrics = metrics_;
        
        xSemaphoreGive(metrics_mutex_);
    }
    
    // Call metrics callback if set
    if (metrics_callback_) {
        metrics_callback_(metrics_);
    }
}

esp_err_t StreamPusher::publishStreamStatus(const std::string& event, const std::string& message) {
    if (!mqtt_client_) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Construct status message
    std::ostringstream json;
    json << "{";
    json << "\"event\":\"" << event << "\",";
    json << "\"message\":\"" << message << "\",";
    json << "\"timestamp\":" << (getCurrentTimestamp() / 1000000) << ",";
    json << "\"state\":\"" << (int)current_state_ << "\",";
    json << "\"metrics\":{";
    json << "\"fps\":" << metrics_.current_fps << ",";
    json << "\"bitrate_kbps\":" << metrics_.current_bitrate_kbps << ",";
    json << "\"total_frames\":" << metrics_.total_frames_sent << ",";
    json << "\"dropped_frames\":" << metrics_.dropped_frames << ",";
    json << "\"is_connected\":" << (metrics_.is_connected ? "true" : "false");
    json << "},";
    json << "\"server_url\":\"" << server_info_.rtmp_url << "\"";
    json << "}";
    
    // Replace {DEVICE_ID} in topic
    std::string topic = config_.mqtt_status_topic;
    size_t pos = topic.find("{DEVICE_ID}");
    if (pos != std::string::npos) {
        topic.replace(pos, 11, "ESP32P4_CAM_001");
    }
    
    return mqtt_client_->publish(topic, json.str(), 0, false);
}

esp_err_t StreamPusher::performHealthCheck() {
    bool healthy = true;
    std::string status_message = "Stream healthy";
    
    // Check RTMP connection
    if (current_state_ == StreamState::STREAMING) {
        if (!rtmp_connection_ || !rtmp_connection_->connected) {
            healthy = false;
            status_message = "RTMP connection lost";
        }
        
        // Check if frames are being sent
        uint64_t current_time = getCurrentTimestamp();
        if (current_time - last_frame_time_ > config_.stream_timeout_sec * 1000000) {
            healthy = false;
            status_message = "No frames sent recently";
        }
    }
    
    // Check encoder health
    if (encoder_) {
        H264Encoder::EncoderState encoder_state = encoder_->getState();
        if (encoder_state == H264Encoder::EncoderState::ERROR) {
            healthy = false;
            status_message = "Encoder error";
        }
    }
    
    // Update network availability
    stream_status_.network_available = healthy;
    
    if (!healthy && current_state_ == StreamState::STREAMING) {
        ESP_LOGW(TAG, "Health check failed: %s", status_message.c_str());
        
        // Trigger reconnection if enabled
        if (config_.enable_auto_reconnect) {
            setState(StreamState::RECONNECTING, status_message);
            xTimerStart(reconnect_timer_, 0);
        } else {
            setState(StreamState::ERROR, status_message);
        }
    }
    
    return ESP_OK;
}

// Utility functions
void StreamPusher::setState(StreamState state, const std::string& message) {
    if (xSemaphoreTake(state_mutex_, portMAX_DELAY) == pdTRUE) {
        current_state_ = state;
        stream_status_.state = state;
        stream_status_.state_message = message;
        xSemaphoreGive(state_mutex_);
    }
    
    if (state_callback_) {
        state_callback_(state, message);
    }
    
    ESP_LOGI(TAG, "State changed to %d: %s", (int)state, message.c_str());
}

std::string StreamPusher::generateStreamKey() const {
    std::ostringstream oss;
    oss << "ESP32P4_CAM_001_" << std::hex << getCurrentTimestamp();
    return oss.str();
}

uint64_t StreamPusher::getCurrentTimestamp() const {
    return esp_timer_get_time();
}

// Quality preset helpers
uint32_t StreamPusher::getDefaultWidth(StreamQuality quality) const {
    switch (quality) {
        case StreamQuality::LOW: return 640;
        case StreamQuality::MEDIUM: return 1280;
        case StreamQuality::HIGH: return 1920;
        case StreamQuality::ULTRA: return 1920;
        default: return config_.custom_width;
    }
}

uint32_t StreamPusher::getDefaultHeight(StreamQuality quality) const {
    switch (quality) {
        case StreamQuality::LOW: return 480;
        case StreamQuality::MEDIUM: return 720;
        case StreamQuality::HIGH: return 1080;
        case StreamQuality::ULTRA: return 1080;
        default: return config_.custom_height;
    }
}

uint32_t StreamPusher::getDefaultFPS(StreamQuality quality) const {
    switch (quality) {
        case StreamQuality::LOW: return 15;
        case StreamQuality::MEDIUM: return 20;
        case StreamQuality::HIGH: return 25;
        case StreamQuality::ULTRA: return 30;
        default: return config_.custom_fps;
    }
}

uint32_t StreamPusher::getDefaultBitrate(StreamQuality quality) const {
    switch (quality) {
        case StreamQuality::LOW: return 500000;    // 500 kbps
        case StreamQuality::MEDIUM: return 1000000;  // 1 Mbps
        case StreamQuality::HIGH: return 2000000;   // 2 Mbps
        case StreamQuality::ULTRA: return 4000000;  // 4 Mbps
        default: return config_.custom_bitrate;
    }
}

std::string StreamPusher::getQualityString(StreamQuality quality) const {
    switch (quality) {
        case StreamQuality::LOW: return "low";
        case StreamQuality::MEDIUM: return "medium";
        case StreamQuality::HIGH: return "high";
        case StreamQuality::ULTRA: return "ultra";
        case StreamQuality::ADAPTIVE: return "adaptive";
        case StreamQuality::CUSTOM: return "custom";
        default: return "unknown";
    }
}

esp_err_t StreamPusher::validateStreamConfig() const {
    if (config_.max_frame_queue_size == 0 || config_.max_frame_queue_size > 100) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (config_.custom_width == 0 || config_.custom_height == 0 || 
        config_.custom_fps == 0 || config_.custom_bitrate == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}

esp_err_t StreamPusher::createTasks() {
    // Create stream task
    BaseType_t ret = xTaskCreate(streamTask, "stream_task", STREAM_TASK_STACK_SIZE, 
                                this, STREAM_TASK_PRIORITY, &stream_task_handle_);
    if (ret != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    
    // Create health check task
    ret = xTaskCreate(healthCheckTask, "health_check_task", HEALTH_CHECK_TASK_STACK_SIZE,
                     this, HEALTH_CHECK_TASK_PRIORITY, &health_check_task_handle_);
    if (ret != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    
    return ESP_OK;
}

void StreamPusher::deleteTasks() {
    tasks_running_ = false;
    
    if (stream_task_handle_) {
        vTaskDelete(stream_task_handle_);
        stream_task_handle_ = nullptr;
    }
    
    if (health_check_task_handle_) {
        vTaskDelete(health_check_task_handle_);
        health_check_task_handle_ = nullptr;
    }
}

esp_err_t StreamPusher::stop() {
    if (current_state_ == StreamState::STREAMING || 
        current_state_ == StreamState::RECONNECTING) {
        stopStream();
    }
    
    // Stop tasks
    deleteTasks();
    
    // Stop timers
    if (reconnect_timer_) {
        xTimerStop(reconnect_timer_, 0);
    }
    if (metrics_timer_) {
        xTimerStop(metrics_timer_, 0);
    }
    
    // Stop encoder
    if (encoder_) {
        encoder_->stop();
    }
    
    setState(StreamState::IDLE, "StreamPusher stopped");
    
    ESP_LOGI(TAG, "StreamPusher stopped");
    return ESP_OK;
}

esp_err_t StreamPusher::deinitialize() {
    stop();
    
    // Clean up resources
    if (frame_queue_) {
        vQueueDelete(frame_queue_);
        frame_queue_ = nullptr;
    }
    
    if (reconnect_timer_) {
        xTimerDelete(reconnect_timer_, 0);
        reconnect_timer_ = nullptr;
    }
    
    if (metrics_timer_) {
        xTimerDelete(metrics_timer_, 0);
        metrics_timer_ = nullptr;
    }
    
    if (encoder_) {
        encoder_->deinitialize();
    }
    
    // Disconnect RTMP
    disconnectFromRTMP();
    
    setState(StreamState::IDLE, "StreamPusher deinitialized");
    
    ESP_LOGI(TAG, "StreamPusher deinitialized");
    return ESP_OK;
}

// Public interface methods
StreamPusher::StreamStatus StreamPusher::getStatus() const {
    return stream_status_;
}

StreamPusher::StreamMetrics StreamPusher::getMetrics() const {
    return metrics_;
}

bool StreamPusher::isStreaming() const {
    return current_state_ == StreamState::STREAMING;
}

bool StreamPusher::isConnected() const {
    return rtmp_connection_ && rtmp_connection_->connected;
}

void StreamPusher::setStateCallback(StateCallback callback) {
    state_callback_ = callback;
}

void StreamPusher::setMetricsCallback(MetricsCallback callback) {
    metrics_callback_ = callback;
}

void StreamPusher::setServerCallback(ServerCallback callback) {
    server_callback_ = callback;
}

void StreamPusher::setErrorCallback(ErrorCallback callback) {
    error_callback_ = callback;
}

void StreamPusher::printStatus() const {
    ESP_LOGI(TAG, "=== Stream Status ===");
    ESP_LOGI(TAG, "State: %d (%s)", (int)current_state_, stream_status_.state_message.c_str());
    ESP_LOGI(TAG, "Server: %s", server_info_.rtmp_url.c_str());
    ESP_LOGI(TAG, "Stream Key: %s", server_info_.stream_key.c_str());
    ESP_LOGI(TAG, "Quality: %s", getQualityString(config_.quality).c_str());
    ESP_LOGI(TAG, "FPS: %.1f", metrics_.current_fps);
    ESP_LOGI(TAG, "Bitrate: %.1f kbps", metrics_.current_bitrate_kbps);
    ESP_LOGI(TAG, "Frames sent: %llu", metrics_.total_frames_sent);
    ESP_LOGI(TAG, "Dropped frames: %llu", metrics_.dropped_frames);
    ESP_LOGI(TAG, "Network quality: %.2f", network_quality_);
    ESP_LOGI(TAG, "Connected: %s", isConnected() ? "Yes" : "No");
    ESP_LOGI(TAG, "==================");
}