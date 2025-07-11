#include "h264_encoder.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstring>
#include <algorithm>

// ESP32P4 specific H.264 hardware encoder headers
// Note: These would be provided by ESP-IDF for ESP32P4
extern "C" {
// Mock hardware encoder interface - replace with actual ESP32P4 H.264 API
typedef void* esp_h264_encoder_handle_t;

typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t fps;
    uint32_t bitrate;
    uint32_t gop_size;
    uint32_t profile;
    uint32_t level;
    uint32_t rc_mode;
} esp_h264_encoder_config_t;

typedef struct {
    uint8_t* data;
    size_t size;
    uint64_t pts;
    bool is_keyframe;
} esp_h264_frame_t;

// Mock hardware encoder functions - replace with actual ESP32P4 API
esp_err_t esp_h264_encoder_create(const esp_h264_encoder_config_t* config, esp_h264_encoder_handle_t* handle);
esp_err_t esp_h264_encoder_destroy(esp_h264_encoder_handle_t handle);
esp_err_t esp_h264_encoder_start(esp_h264_encoder_handle_t handle);
esp_err_t esp_h264_encoder_stop(esp_h264_encoder_handle_t handle);
esp_err_t esp_h264_encoder_encode_frame(esp_h264_encoder_handle_t handle, const uint8_t* input, size_t input_size, esp_h264_frame_t* output);
esp_err_t esp_h264_encoder_set_bitrate(esp_h264_encoder_handle_t handle, uint32_t bitrate);
esp_err_t esp_h264_encoder_force_keyframe(esp_h264_encoder_handle_t handle);
}

const char* H264Encoder::TAG = "H264Encoder";

// Hardware encoder wrapper
struct H264Encoder::HWEncoder {
    esp_h264_encoder_handle_t handle;
    bool initialized;
    
    HWEncoder() : handle(nullptr), initialized(false) {}
};

H264Encoder::H264Encoder()
    : current_state_(EncoderState::UNINITIALIZED)
    , hw_encoder_(std::make_unique<HWEncoder>())
    , encoding_task_handle_(nullptr)
    , input_queue_(nullptr)
    , output_queue_(nullptr)
    , state_mutex_(nullptr)
    , encoding_active_(false)
    , input_buffers_(nullptr)
    , output_buffers_(nullptr)
    , input_buffer_pool_(nullptr)
    , output_buffer_pool_(nullptr)
    , stats_mutex_(nullptr)
    , last_stats_update_(0)
{
    // Initialize mutexes
    state_mutex_ = xSemaphoreCreateMutex();
    stats_mutex_ = xSemaphoreCreateMutex();
    
    if (!state_mutex_ || !stats_mutex_) {
        ESP_LOGE(TAG, "Failed to create mutexes");
    }
    
    // Initialize statistics
    memset(&statistics_, 0, sizeof(statistics_));
}

H264Encoder::~H264Encoder() {
    deinitialize();
    
    if (state_mutex_) {
        vSemaphoreDelete(state_mutex_);
    }
    if (stats_mutex_) {
        vSemaphoreDelete(stats_mutex_);
    }
}

esp_err_t H264Encoder::initialize(const EncoderConfig& config) {
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_FAIL;
    }
    
    esp_err_t ret = ESP_OK;
    
    if (current_state_ != EncoderState::UNINITIALIZED) {
        ESP_LOGW(TAG, "Encoder already initialized");
        xSemaphoreGive(state_mutex_);
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing H.264 hardware encoder");
    config_ = config;
    
    // Validate configuration
    if (config_.width > getMaxWidth() || config_.height > getMaxHeight()) {
        ESP_LOGE(TAG, "Resolution %dx%d exceeds hardware limits", config_.width, config_.height);
        ret = ESP_ERR_INVALID_ARG;
        goto cleanup;
    }
    
    if (config_.bitrate > getMaxBitrate()) {
        ESP_LOGE(TAG, "Bitrate %d exceeds hardware limits", config_.bitrate);
        ret = ESP_ERR_INVALID_ARG;
        goto cleanup;
    }
    
    // Initialize hardware encoder
    ret = initializeHardware();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize hardware encoder");
        goto cleanup;
    }
    
    // Allocate buffers
    ret = allocateBuffers();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to allocate buffers");
        goto cleanup;
    }
    
    // Create queues
    input_queue_ = xQueueCreate(QUEUE_SIZE, sizeof(EncodingFrame));
    output_queue_ = xQueueCreate(QUEUE_SIZE, sizeof(FrameInfo));
    
    if (!input_queue_ || !output_queue_) {
        ESP_LOGE(TAG, "Failed to create queues");
        ret = ESP_FAIL;
        goto cleanup;
    }
    
    setState(EncoderState::INITIALIZED, "H.264 encoder initialized");
    ESP_LOGI(TAG, "H.264 encoder initialized: %dx%d@%dfps, %d bps", 
             config_.width, config_.height, config_.fps, config_.bitrate);

cleanup:
    if (ret != ESP_OK) {
        deallocateBuffers();
        setState(EncoderState::ERROR, "Failed to initialize encoder");
    }
    
    xSemaphoreGive(state_mutex_);
    return ret;
}

esp_err_t H264Encoder::deinitialize() {
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_FAIL;
    }
    
    if (current_state_ == EncoderState::UNINITIALIZED) {
        xSemaphoreGive(state_mutex_);
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Deinitializing H.264 encoder");
    
    // Stop encoding if active
    if (encoding_active_) {
        stop();
    }
    
    // Clean up hardware
    if (hw_encoder_ && hw_encoder_->initialized) {
        esp_h264_encoder_destroy(hw_encoder_->handle);
        hw_encoder_->initialized = false;
    }
    
    // Clean up queues
    if (input_queue_) {
        vQueueDelete(input_queue_);
        input_queue_ = nullptr;
    }
    if (output_queue_) {
        vQueueDelete(output_queue_);
        output_queue_ = nullptr;
    }
    
    // Deallocate buffers
    deallocateBuffers();
    
    setState(EncoderState::UNINITIALIZED, "H.264 encoder deinitialized");
    
    xSemaphoreGive(state_mutex_);
    return ESP_OK;
}

esp_err_t H264Encoder::start() {
    if (current_state_ != EncoderState::INITIALIZED) {
        ESP_LOGE(TAG, "Encoder not initialized");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Starting H.264 encoding");
    
    // Start hardware encoder
    esp_err_t ret = startHardwareEncoder();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start hardware encoder");
        return ret;
    }
    
    // Create encoding task
    encoding_active_ = true;
    
    BaseType_t result = xTaskCreate(
        encodingTask,
        "h264_encoder",
        ENCODING_TASK_STACK_SIZE,
        this,
        ENCODING_TASK_PRIORITY,
        &encoding_task_handle_
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create encoding task");
        encoding_active_ = false;
        stopHardwareEncoder();
        return ESP_FAIL;
    }
    
    setState(EncoderState::ENCODING, "H.264 encoding started");
    resetStatistics();
    
    return ESP_OK;
}

esp_err_t H264Encoder::stop() {
    if (current_state_ != EncoderState::ENCODING) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Stopping H.264 encoding");
    
    // Stop encoding task
    encoding_active_ = false;
    
    if (encoding_task_handle_) {
        // Signal task to stop and wait
        vTaskDelay(pdMS_TO_TICKS(100));
        vTaskDelete(encoding_task_handle_);
        encoding_task_handle_ = nullptr;
    }
    
    // Stop hardware encoder
    stopHardwareEncoder();
    
    setState(EncoderState::INITIALIZED, "H.264 encoding stopped");
    
    return ESP_OK;
}

esp_err_t H264Encoder::encodeFrame(camera_fb_t* frame) {
    if (current_state_ != EncoderState::ENCODING) {
        ESP_LOGE(TAG, "Encoder not in encoding state");
        return ESP_FAIL;
    }
    
    if (!frame || !frame->buf || frame->len == 0) {
        ESP_LOGE(TAG, "Invalid frame");
        return ESP_ERR_INVALID_ARG;
    }
    
    EncodingFrame enc_frame = {
        .frame = frame,
        .timestamp_us = getCurrentTimestamp(),
        .force_keyframe = false
    };
    
    if (xQueueSend(input_queue_, &enc_frame, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Input queue full, dropping frame");
        if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
            statistics_.dropped_frames++;
            xSemaphoreGive(stats_mutex_);
        }
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t H264Encoder::forceKeyFrame() {
    if (current_state_ != EncoderState::ENCODING) {
        return ESP_FAIL;
    }
    
    return esp_h264_encoder_force_keyframe(hw_encoder_->handle);
}

esp_err_t H264Encoder::setBitrate(uint32_t bitrate) {
    if (bitrate > getMaxBitrate()) {
        return ESP_ERR_INVALID_ARG;
    }
    
    config_.bitrate = bitrate;
    
    if (hw_encoder_ && hw_encoder_->initialized) {
        return esp_h264_encoder_set_bitrate(hw_encoder_->handle, bitrate);
    }
    
    return ESP_OK;
}

esp_err_t H264Encoder::setFramerate(uint32_t fps) {
    if (fps > 60) {  // Reasonable limit
        return ESP_ERR_INVALID_ARG;
    }
    
    config_.fps = fps;
    // Note: Framerate change might require encoder restart
    return ESP_OK;
}

esp_err_t H264Encoder::setQuality(uint32_t qp) {
    if (qp < config_.qp_min || qp > config_.qp_max) {
        return ESP_ERR_INVALID_ARG;
    }
    
    config_.qp_init = qp;
    return ESP_OK;
}

void H264Encoder::setFrameCallback(FrameCallback callback) {
    frame_callback_ = callback;
}

void H264Encoder::setStateCallback(StateCallback callback) {
    state_callback_ = callback;
}

H264Encoder::Statistics H264Encoder::getStatistics() const {
    Statistics stats = {};
    
    if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        stats = statistics_;
        xSemaphoreGive(stats_mutex_);
    }
    
    return stats;
}

void H264Encoder::resetStatistics() {
    if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(&statistics_, 0, sizeof(statistics_));
        last_stats_update_ = getCurrentTimestamp();
        xSemaphoreGive(stats_mutex_);
    }
}

bool H264Encoder::isHardwareSupported() {
    // ESP32P4 has dedicated H.264 encoding hardware
    return true;
}

uint32_t H264Encoder::getMaxWidth() {
    return 1920;  // ESP32P4 supports up to 1080p
}

uint32_t H264Encoder::getMaxHeight() {
    return 1080;
}

uint32_t H264Encoder::getMaxBitrate() {
    return 20000000;  // 20 Mbps
}

esp_err_t H264Encoder::initializeHardware() {
    if (!isHardwareSupported()) {
        ESP_LOGE(TAG, "H.264 hardware encoding not supported on this chip");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    esp_h264_encoder_config_t hw_config = {
        .width = config_.width,
        .height = config_.height,
        .fps = config_.fps,
        .bitrate = config_.bitrate,
        .gop_size = config_.gop_size,
        .profile = static_cast<uint32_t>(config_.profile),
        .level = config_.level,
        .rc_mode = static_cast<uint32_t>(config_.rc_mode)
    };
    
    esp_err_t ret = esp_h264_encoder_create(&hw_config, &hw_encoder_->handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create hardware encoder: %s", esp_err_to_name(ret));
        return ret;
    }
    
    hw_encoder_->initialized = true;
    ESP_LOGI(TAG, "Hardware encoder created successfully");
    
    return ESP_OK;
}

esp_err_t H264Encoder::startHardwareEncoder() {
    if (!hw_encoder_ || !hw_encoder_->initialized) {
        return ESP_FAIL;
    }
    
    return esp_h264_encoder_start(hw_encoder_->handle);
}

esp_err_t H264Encoder::stopHardwareEncoder() {
    if (!hw_encoder_ || !hw_encoder_->initialized) {
        return ESP_OK;
    }
    
    return esp_h264_encoder_stop(hw_encoder_->handle);
}

void H264Encoder::encodingTask(void* parameter) {
    H264Encoder* self = static_cast<H264Encoder*>(parameter);
    ESP_LOGI(TAG, "Encoding task started");
    
    EncodingFrame input_frame;
    
    while (self->encoding_active_) {
        // Wait for input frame
        if (xQueueReceive(self->input_queue_, &input_frame, pdMS_TO_TICKS(100)) == pdTRUE) {
            uint64_t start_time = self->getCurrentTimestamp();
            
            // Encode frame using hardware encoder
            esp_h264_frame_t output_frame;
            esp_err_t ret = esp_h264_encoder_encode_frame(
                self->hw_encoder_->handle,
                input_frame.frame->buf,
                input_frame.frame->len,
                &output_frame
            );
            
            if (ret == ESP_OK && output_frame.data && output_frame.size > 0) {
                // Create frame info for callback
                FrameInfo frame_info = {
                    .data = output_frame.data,
                    .size = output_frame.size,
                    .timestamp_us = input_frame.timestamp_us,
                    .is_keyframe = output_frame.is_keyframe,
                    .frame_number = 0  // Will be set by updateStatistics
                };
                
                // Update statistics
                uint64_t encoding_time = self->getCurrentTimestamp() - start_time;
                self->updateStatistics(output_frame.size, encoding_time, output_frame.is_keyframe);
                
                // Set frame number
                frame_info.frame_number = self->statistics_.frames_encoded;
                
                // Call frame callback if set
                if (self->frame_callback_) {
                    self->frame_callback_(frame_info);
                }
                
                ESP_LOGD(TAG, "Encoded frame: %zu bytes, keyframe: %d, time: %llu us",
                         output_frame.size, output_frame.is_keyframe, encoding_time);
            } else {
                ESP_LOGE(TAG, "Failed to encode frame: %s", esp_err_to_name(ret));
                if (xSemaphoreTake(self->stats_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
                    self->statistics_.dropped_frames++;
                    xSemaphoreGive(self->stats_mutex_);
                }
            }
        }
    }
    
    ESP_LOGI(TAG, "Encoding task ended");
    vTaskDelete(nullptr);
}

esp_err_t H264Encoder::allocateBuffers() {
    // Allocate input buffer pool
    input_buffers_ = new uint8_t*[config_.input_buffer_count];
    input_buffer_pool_ = xQueueCreate(config_.input_buffer_count, sizeof(uint8_t*));
    
    if (!input_buffers_ || !input_buffer_pool_) {
        return ESP_ERR_NO_MEM;
    }
    
    for (uint32_t i = 0; i < config_.input_buffer_count; i++) {
        input_buffers_[i] = new uint8_t[config_.width * config_.height * 3 / 2]; // YUV420
        if (!input_buffers_[i]) {
            return ESP_ERR_NO_MEM;
        }
        xQueueSend(input_buffer_pool_, &input_buffers_[i], 0);
    }
    
    // Allocate output buffer pool
    output_buffers_ = new uint8_t*[config_.output_buffer_count];
    output_buffer_pool_ = xQueueCreate(config_.output_buffer_count, sizeof(uint8_t*));
    
    if (!output_buffers_ || !output_buffer_pool_) {
        return ESP_ERR_NO_MEM;
    }
    
    for (uint32_t i = 0; i < config_.output_buffer_count; i++) {
        output_buffers_[i] = new uint8_t[config_.max_frame_size];
        if (!output_buffers_[i]) {
            return ESP_ERR_NO_MEM;
        }
        xQueueSend(output_buffer_pool_, &output_buffers_[i], 0);
    }
    
    ESP_LOGI(TAG, "Allocated %d input and %d output buffers", 
             config_.input_buffer_count, config_.output_buffer_count);
    
    return ESP_OK;
}

void H264Encoder::deallocateBuffers() {
    // Clean up input buffers
    if (input_buffers_) {
        for (uint32_t i = 0; i < config_.input_buffer_count; i++) {
            delete[] input_buffers_[i];
        }
        delete[] input_buffers_;
        input_buffers_ = nullptr;
    }
    
    if (input_buffer_pool_) {
        vQueueDelete(input_buffer_pool_);
        input_buffer_pool_ = nullptr;
    }
    
    // Clean up output buffers
    if (output_buffers_) {
        for (uint32_t i = 0; i < config_.output_buffer_count; i++) {
            delete[] output_buffers_[i];
        }
        delete[] output_buffers_;
        output_buffers_ = nullptr;
    }
    
    if (output_buffer_pool_) {
        vQueueDelete(output_buffer_pool_);
        output_buffer_pool_ = nullptr;
    }
}

void H264Encoder::setState(EncoderState state, const std::string& message) {
    current_state_ = state;
    ESP_LOGI(TAG, "State changed to %d: %s", (int)state, message.c_str());
    
    if (state_callback_) {
        state_callback_(state, message);
    }
}

void H264Encoder::updateStatistics(size_t frame_size, uint64_t encoding_time_us, bool is_keyframe) {
    if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        statistics_.frames_encoded++;
        statistics_.bytes_encoded += frame_size;
        
        if (is_keyframe) {
            statistics_.keyframes_generated++;
        }
        
        // Update average encoding time
        double alpha = 0.1;  // Smoothing factor
        double encoding_time_ms = encoding_time_us / 1000.0;
        statistics_.avg_encoding_time_ms = 
            alpha * encoding_time_ms + (1.0 - alpha) * statistics_.avg_encoding_time_ms;
        
        // Update current bitrate and FPS (every second)
        uint64_t now = getCurrentTimestamp();
        if (now - last_stats_update_ >= 1000000) {  // 1 second
            uint64_t time_diff = now - last_stats_update_;
            statistics_.current_bitrate = (statistics_.bytes_encoded * 8 * 1000000) / time_diff;
            statistics_.current_fps = (statistics_.frames_encoded * 1000000) / time_diff;
            last_stats_update_ = now;
        }
        
        xSemaphoreGive(stats_mutex_);
    }
}

uint64_t H264Encoder::getCurrentTimestamp() {
    return esp_timer_get_time();
}