#pragma once

#include <string>
#include <functional>
#include <memory>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include "esp_err.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "mqtt_client.hpp"
#include "h264_encoder.hpp"

// RTMP Stream Pusher for ESP32P4 Network Camera
// Handles real-time H.264 video streaming to RTMP servers
// Supports MQTT-based stream server discovery and configuration
class StreamPusher {
public:
    enum class StreamState {
        IDLE,                    // Not streaming
        REQUESTING_SERVER,       // Requesting stream server via MQTT
        CONNECTING,             // Connecting to RTMP server
        STREAMING,              // Actively streaming
        RECONNECTING,           // Reconnecting after disconnect
        ERROR,                  // Error state
        STOPPING                // Stopping stream
    };

    enum class StreamQuality {
        LOW,                    // 480p@15fps, 500kbps
        MEDIUM,                 // 720p@20fps, 1Mbps  
        HIGH,                   // 1080p@25fps, 2Mbps
        ULTRA,                  // 1080p@30fps, 4Mbps
        ADAPTIVE,               // Adaptive bitrate based on network
        CUSTOM                  // Custom settings
    };

    enum class StreamTrigger {
        MANUAL,                 // Manual start/stop
        MOTION_BASED,           // Start on motion detection
        SCHEDULED,              // Based on schedule
        ALWAYS_ON,              // Continuous streaming
        ON_DEMAND               // Stream when requested via MQTT
    };

    struct StreamConfig {
        // RTMP server configuration
        std::string rtmp_base_url = "rtmp://your-server.com/live/";
        std::string stream_key = "";
        std::string backup_rtmp_url = "";
        
        // MQTT configuration for server discovery
        std::string mqtt_request_topic = "devices/{DEVICE_ID}/stream/request";
        std::string mqtt_response_topic = "devices/{DEVICE_ID}/stream/response";  
        std::string mqtt_status_topic = "devices/{DEVICE_ID}/stream/status";
        std::string mqtt_control_topic = "devices/{DEVICE_ID}/stream/control";
        
        // Stream quality settings
        StreamQuality quality = StreamQuality::HIGH;
        uint32_t custom_width = 1920;
        uint32_t custom_height = 1080;
        uint32_t custom_fps = 30;
        uint32_t custom_bitrate = 2000000;  // 2 Mbps
        
        // Streaming behavior
        StreamTrigger trigger = StreamTrigger::MANUAL;
        bool enable_audio = false;          // Audio streaming (future)
        bool enable_adaptive_bitrate = true;
        bool enable_auto_reconnect = true;
        uint32_t reconnect_interval_sec = 10;
        uint32_t max_reconnect_attempts = 5;
        
        // Buffer settings
        uint32_t stream_buffer_size = 256 * 1024;  // 256KB
        uint32_t max_frame_queue_size = 30;
        uint32_t frame_drop_threshold = 20;
        
        // Health monitoring
        uint32_t health_check_interval_sec = 30;
        uint32_t stream_timeout_sec = 60;
        bool enable_bandwidth_monitoring = true;
        
        // Performance optimization
        bool enable_frame_dropping = true;
        uint32_t max_encoding_delay_ms = 200;
        uint32_t target_latency_ms = 3000;  // 3 seconds
    };

    struct StreamServerInfo {
        std::string rtmp_url;
        std::string stream_key;
        std::string server_name;
        std::string server_region;
        uint32_t max_bitrate = 0;
        uint32_t recommended_fps = 30;
        bool supports_adaptive = false;
        uint64_t expires_at = 0;
        std::string token;
    };

    struct StreamMetrics {
        uint64_t total_frames_sent = 0;
        uint64_t total_bytes_sent = 0;
        uint64_t dropped_frames = 0;
        uint64_t encoding_errors = 0;
        uint64_t connection_errors = 0;
        uint64_t stream_duration_ms = 0;
        uint64_t last_frame_timestamp = 0;
        
        double current_fps = 0.0;
        double current_bitrate_kbps = 0.0;
        double average_bitrate_kbps = 0.0;
        uint32_t current_latency_ms = 0;
        uint32_t frame_queue_size = 0;
        
        bool is_streaming = false;
        bool is_connected = false;
        std::string server_url;
        std::string connection_info;
        std::string last_error;
    };

    struct StreamStatus {
        StreamState state = StreamState::IDLE;
        std::string state_message;
        StreamMetrics metrics;
        StreamServerInfo server_info;
        uint64_t stream_start_time = 0;
        uint64_t last_keyframe_time = 0;
        uint32_t reconnect_attempts = 0;
        bool network_available = true;
        float network_quality = 1.0f;  // 0.0 - 1.0
    };

    using StateCallback = std::function<void(StreamState state, const std::string& message)>;
    using MetricsCallback = std::function<void(const StreamMetrics& metrics)>;
    using ServerCallback = std::function<void(const StreamServerInfo& server_info, bool success)>;
    using ErrorCallback = std::function<void(const std::string& error_code, const std::string& message)>;

    StreamPusher();
    ~StreamPusher();

    // Lifecycle management
    esp_err_t initialize(const StreamConfig& config, MQTTClient* mqtt_client);
    esp_err_t start();
    esp_err_t stop();
    esp_err_t deinitialize();

    // Streaming control
    esp_err_t startStream(const std::string& stream_key = "");
    esp_err_t stopStream();
    esp_err_t pauseStream();
    esp_err_t resumeStream();
    esp_err_t reconnectStream();

    // Frame input
    esp_err_t pushFrame(camera_fb_t* frame);
    esp_err_t pushH264Frame(const uint8_t* h264_data, size_t data_size, 
                           bool is_keyframe, uint64_t timestamp_us);

    // Server discovery and configuration
    esp_err_t requestStreamServer(const std::string& preferred_region = "");
    esp_err_t setStreamServer(const StreamServerInfo& server_info);
    esp_err_t setCustomRTMPUrl(const std::string& rtmp_url, const std::string& stream_key);

    // Quality control
    esp_err_t setStreamQuality(StreamQuality quality);
    esp_err_t setCustomQuality(uint32_t width, uint32_t height, uint32_t fps, uint32_t bitrate);
    esp_err_t setAdaptiveBitrate(bool enable);
    esp_err_t adjustBitrate(uint32_t target_bitrate);

    // Monitoring and status
    StreamStatus getStatus() const;
    StreamMetrics getMetrics() const;
    bool isStreaming() const;
    bool isConnected() const;
    float getNetworkQuality() const;

    // Callbacks
    void setStateCallback(StateCallback callback);
    void setMetricsCallback(MetricsCallback callback);
    void setServerCallback(ServerCallback callback);
    void setErrorCallback(ErrorCallback callback);

    // Control commands (via MQTT)
    esp_err_t handleControlCommand(const std::string& command, const std::string& params);

    // Debugging and diagnostics
    void printStatus() const;
    esp_err_t performHealthCheck();
    std::string generateDiagnosticReport() const;

private:
    // Internal state
    StreamConfig config_;
    StreamState current_state_;
    StreamStatus stream_status_;
    StreamMetrics metrics_;
    StreamServerInfo server_info_;
    MQTTClient* mqtt_client_;

    // Frame processing
    std::unique_ptr<H264Encoder> encoder_;
    QueueHandle_t frame_queue_;
    SemaphoreHandle_t state_mutex_;
    SemaphoreHandle_t metrics_mutex_;
    
    // RTMP connection
    struct RTMPConnection;
    std::unique_ptr<RTMPConnection> rtmp_connection_;
    
    // Tasks and timers
    TaskHandle_t stream_task_handle_;
    TaskHandle_t health_check_task_handle_;
    TimerHandle_t reconnect_timer_;
    TimerHandle_t metrics_timer_;
    volatile bool tasks_running_;

    // Callbacks
    StateCallback state_callback_;
    MetricsCallback metrics_callback_;
    ServerCallback server_callback_;
    ErrorCallback error_callback_;

    // Stream management
    uint64_t stream_start_time_;
    uint64_t last_frame_time_;
    uint32_t frame_counter_;
    uint32_t keyframe_counter_;
    uint32_t reconnect_attempts_;
    
    // Quality adaptation
    uint32_t target_bitrate_;
    uint32_t current_bitrate_;
    float network_quality_;
    std::queue<uint32_t> latency_history_;
    std::queue<double> bitrate_history_;

    // Task functions
    static void streamTask(void* parameter);
    static void healthCheckTask(void* parameter);
    static void reconnectTimerCallback(TimerHandle_t timer);
    static void metricsTimerCallback(TimerHandle_t timer);

    // RTMP implementation
    esp_err_t connectToRTMP();
    esp_err_t disconnectFromRTMP();
    esp_err_t sendRTMPFrame(const uint8_t* h264_data, size_t data_size, 
                           bool is_keyframe, uint64_t timestamp_us);
    esp_err_t sendRTMPMetadata();
    bool isRTMPConnected() const;

    // MQTT communication
    esp_err_t publishStreamRequest(const std::string& preferred_region);
    esp_err_t publishStreamStatus(const std::string& event, const std::string& message);
    esp_err_t handleStreamResponse(const std::string& response_json);
    esp_err_t handleControlMessage(const std::string& command_json);

    // Frame processing
    esp_err_t processFrame(camera_fb_t* frame);
    esp_err_t encodeAndSendFrame(camera_fb_t* frame);
    bool shouldDropFrame() const;
    esp_err_t adaptQuality();

    // Network monitoring
    void updateNetworkQuality();
    void updateMetrics();
    esp_err_t performNetworkDiagnostics();

    // Utility functions
    void setState(StreamState state, const std::string& message = "");
    std::string generateStreamKey() const;
    std::string getQualityString(StreamQuality quality) const;
    std::string formatBitrate(uint32_t bitrate_bps) const;
    std::string formatDuration(uint64_t duration_ms) const;
    uint64_t getCurrentTimestamp() const;

    // Configuration helpers
    esp_err_t setupQualitySettings();
    esp_err_t validateStreamConfig() const;
    esp_err_t createTasks();
    void deleteTasks();

    // Constants
    static const uint32_t STREAM_TASK_STACK_SIZE = 8192;
    static const UBaseType_t STREAM_TASK_PRIORITY = 6;
    static const uint32_t HEALTH_CHECK_TASK_STACK_SIZE = 4096;
    static const UBaseType_t HEALTH_CHECK_TASK_PRIORITY = 3;
    static const uint32_t FRAME_QUEUE_SIZE = 32;
    static const uint32_t RTMP_CONNECT_TIMEOUT_MS = 10000;
    static const uint32_t RTMP_SEND_TIMEOUT_MS = 5000;
    static const uint32_t LATENCY_HISTORY_SIZE = 10;
    static const uint32_t BITRATE_HISTORY_SIZE = 30;
    
    static const char* TAG;
};