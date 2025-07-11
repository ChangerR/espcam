#pragma once

#include <string>
#include <functional>
#include <memory>
#include "esp_err.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ESP32P4 H.264 hardware encoder interface
// Note: ESP32P4 has dedicated H.264 encoding hardware
class H264Encoder {
public:
    enum class EncoderState {
        UNINITIALIZED,
        INITIALIZED,
        ENCODING,
        ERROR
    };

    enum class Profile {
        BASELINE = 0,
        MAIN = 1,
        HIGH = 2
    };

    enum class RateControlMode {
        CBR = 0,  // Constant Bitrate
        VBR = 1,  // Variable Bitrate
        CQP = 2   // Constant QP
    };

    struct EncoderConfig {
        // Video parameters
        uint32_t width = 1280;
        uint32_t height = 720;
        uint32_t fps = 30;
        uint32_t bitrate = 2000000;  // 2 Mbps default
        uint32_t gop_size = 30;      // I-frame interval
        
        // H.264 parameters
        Profile profile = Profile::HIGH;
        uint32_t level = 31;  // Level 3.1
        RateControlMode rc_mode = RateControlMode::CBR;
        uint32_t qp_min = 10;
        uint32_t qp_max = 51;
        uint32_t qp_init = 26;
        
        // Buffer parameters
        uint32_t input_buffer_count = 4;
        uint32_t output_buffer_count = 8;
        uint32_t max_frame_size = 256 * 1024;  // 256KB max frame
        
        // Performance parameters
        bool low_latency_mode = false;
        bool enable_cabac = true;
        uint32_t slice_count = 1;
    };

    struct FrameInfo {
        uint8_t* data;
        size_t size;
        uint64_t timestamp_us;
        bool is_keyframe;
        uint32_t frame_number;
    };

    using FrameCallback = std::function<void(const FrameInfo& frame)>;
    using StateCallback = std::function<void(EncoderState state, const std::string& message)>;

    H264Encoder();
    ~H264Encoder();

    // Encoder lifecycle
    esp_err_t initialize(const EncoderConfig& config);
    esp_err_t deinitialize();
    esp_err_t start();
    esp_err_t stop();

    // Frame processing
    esp_err_t encodeFrame(camera_fb_t* frame);
    esp_err_t forceKeyFrame();
    
    // Configuration
    esp_err_t setBitrate(uint32_t bitrate);
    esp_err_t setFramerate(uint32_t fps);
    esp_err_t setQuality(uint32_t qp);
    
    // Callbacks
    void setFrameCallback(FrameCallback callback);
    void setStateCallback(StateCallback callback);
    
    // Status
    EncoderState getState() const { return current_state_; }
    bool isEncoding() const { return current_state_ == EncoderState::ENCODING; }
    
    // Statistics
    struct Statistics {
        uint64_t frames_encoded = 0;
        uint64_t bytes_encoded = 0;
        uint64_t keyframes_generated = 0;
        uint32_t current_bitrate = 0;
        uint32_t current_fps = 0;
        double avg_encoding_time_ms = 0.0;
        uint32_t dropped_frames = 0;
    };
    
    Statistics getStatistics() const;
    void resetStatistics();
    
    // Hardware capabilities
    static bool isHardwareSupported();
    static uint32_t getMaxWidth();
    static uint32_t getMaxHeight();
    static uint32_t getMaxBitrate();

private:
    // Hardware encoder interface
    struct HWEncoder;
    
    // Internal methods
    esp_err_t initializeHardware();
    esp_err_t configureEncoder();
    esp_err_t startHardwareEncoder();
    esp_err_t stopHardwareEncoder();
    
    // Frame processing
    static void encodingTask(void* parameter);
    esp_err_t processInputFrame(camera_fb_t* frame);
    esp_err_t processOutputFrame();
    
    // Buffer management
    esp_err_t allocateBuffers();
    void deallocateBuffers();
    uint8_t* getInputBuffer();
    void returnInputBuffer(uint8_t* buffer);
    uint8_t* getOutputBuffer();
    void returnOutputBuffer(uint8_t* buffer);
    
    // Utilities
    void setState(EncoderState state, const std::string& message = "");
    void updateStatistics(size_t frame_size, uint64_t encoding_time_us, bool is_keyframe);
    uint64_t getCurrentTimestamp();
    
    EncoderConfig config_;
    EncoderState current_state_;
    FrameCallback frame_callback_;
    StateCallback state_callback_;
    
    // Hardware encoder handle
    std::unique_ptr<HWEncoder> hw_encoder_;
    
    // FreeRTOS synchronization
    TaskHandle_t encoding_task_handle_;
    QueueHandle_t input_queue_;
    QueueHandle_t output_queue_;
    SemaphoreHandle_t state_mutex_;
    volatile bool encoding_active_;
    
    // Buffer pools
    uint8_t** input_buffers_;
    uint8_t** output_buffers_;
    QueueHandle_t input_buffer_pool_;
    QueueHandle_t output_buffer_pool_;
    
    // Statistics
    mutable SemaphoreHandle_t stats_mutex_;
    Statistics statistics_;
    uint64_t last_stats_update_;
    
    // Task configuration
    static const uint32_t ENCODING_TASK_STACK_SIZE = 8192;
    static const UBaseType_t ENCODING_TASK_PRIORITY = 6;  // High priority for real-time encoding
    static const uint32_t QUEUE_SIZE = 16;
    
    static const char* TAG;
};

// Input frame structure for encoding queue
struct EncodingFrame {
    camera_fb_t* frame;
    uint64_t timestamp_us;
    bool force_keyframe;
};