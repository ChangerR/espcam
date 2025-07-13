#include "h264_encoder.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include <cstring>
#include <algorithm>

// ESP32P4 H.264 hardware encoder (simplified implementation)
// Note: Full hardware H.264 encoding support may require newer ESP-IDF versions
extern "C" {
#include "soc/soc_caps.h"
}

const char* H264Encoder::TAG = "H264Encoder";

// ESP32P4 hardware encoder wrapper
struct H264Encoder::HWEncoder {
    esp_h264_enc_handle_t encoder_handle;
    esp_h264_enc_cfg_hw_t encoder_config;
    esp_h264_enc_in_frame_t input_frame;
    esp_h264_enc_out_frame_t output_frame;
    bool initialized;
    uint32_t frame_count;
    
    HWEncoder() : encoder_handle(nullptr), encoder_config({}), 
                  input_frame({}), output_frame({}),
                  initialized(false), frame_count(0) {
        // Initialize configuration with ESP32P4 hardware encoder format
        encoder_config.pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY;  // Hardware encoder format
    }
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
    statistics_ = {};
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
    
    ESP_LOGI(TAG, "Initializing H.264 encoder");
    config_ = config;
    
    // Check if hardware encoding is supported
    if (!isHardwareSupported()) {
        ESP_LOGW(TAG, "Hardware H.264 encoding not supported, falling back to simulation mode");
        // For now, we'll initialize a minimal software simulation
        ret = initializeSoftwareSimulation();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize software simulation");
            goto cleanup;
        }
    } else {
        ESP_LOGI(TAG, "Using ESP32P4 hardware H.264 encoder");
        
        // Validate configuration for hardware
        if (config_.width > getMaxWidth() || config_.height > getMaxHeight()) {
            ESP_LOGE(TAG, "Resolution %lux%lu exceeds hardware limits", (unsigned long)config_.width, (unsigned long)config_.height);
            ret = ESP_ERR_INVALID_ARG;
            goto cleanup;
        }
        
        if (config_.bitrate > getMaxBitrate()) {
            ESP_LOGE(TAG, "Bitrate %lu exceeds hardware limits", (unsigned long)config_.bitrate);
            ret = ESP_ERR_INVALID_ARG;
            goto cleanup;
        }
        
        // Initialize hardware encoder
        ret = initializeHardware();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize hardware encoder, falling back to simulation");
            ret = initializeSoftwareSimulation();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to initialize software simulation");
                goto cleanup;
            }
        }
    }
    
    // Allocate buffers (only if not in simulation mode)
    if (hw_encoder_->initialized && hw_encoder_->encoder_handle) {
        ret = allocateBuffers();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to allocate buffers");
            goto cleanup;
        }
    } else {
        ESP_LOGI(TAG, "Skipping buffer allocation in simulation mode");
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
    ESP_LOGI(TAG, "H.264 encoder initialized: %lux%lu@%lufps, %lu bps", 
             (unsigned long)config_.width, (unsigned long)config_.height, (unsigned long)config_.fps, (unsigned long)config_.bitrate);

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
        // Free aligned buffers
        if (hw_encoder_->input_frame.raw_data.buffer) {
            esp_h264_free(hw_encoder_->input_frame.raw_data.buffer);
            hw_encoder_->input_frame.raw_data.buffer = nullptr;
        }
        if (hw_encoder_->output_frame.raw_data.buffer) {
            esp_h264_free(hw_encoder_->output_frame.raw_data.buffer);
            hw_encoder_->output_frame.raw_data.buffer = nullptr;
        }
        
        // Delete hardware encoder instance
        if (hw_encoder_->encoder_handle) {
            esp_h264_enc_del(hw_encoder_->encoder_handle);
            hw_encoder_->encoder_handle = nullptr;
        }
        
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
    
    // For now, return OK as keyframe forcing might require specific codec control
    ESP_LOGW(TAG, "Force keyframe requested - feature not yet implemented");
    return ESP_OK;
}

esp_err_t H264Encoder::setBitrate(uint32_t bitrate) {
    if (bitrate > getMaxBitrate()) {
        return ESP_ERR_INVALID_ARG;
    }
    
    config_.bitrate = bitrate;
    
    // Bitrate control might require codec reconfiguration
    ESP_LOGI(TAG, "Bitrate changed to %lu bps", (unsigned long)bitrate);
    return ESP_OK;
}

esp_err_t H264Encoder::processInputFrame(camera_fb_t* frame) {
    // This method is deprecated - frame processing is now handled in performHardwareEncoding
    ESP_LOGW(TAG, "processInputFrame is deprecated - use performHardwareEncoding instead");
    return ESP_OK;
}


esp_err_t H264Encoder::performHardwareEncoding(camera_fb_t* frame, size_t* output_size, bool* is_keyframe) {
    if (!frame || !output_size || !is_keyframe) {
        return ESP_ERR_INVALID_ARG;
    }
    
    hw_encoder_->frame_count++;
    
    // Check if hardware encoder is available
    if (hw_encoder_->initialized && hw_encoder_->encoder_handle) {
        // Hardware encoding path
        
        // Copy frame data to input buffer (convert format if needed)
        esp_err_t ret = convertAndCopyFrame(frame);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to convert frame format: %s", esp_err_to_name(ret));
            return ret;
        }
        
        // Set timestamp for input frame
        hw_encoder_->input_frame.pts = hw_encoder_->frame_count;
        
        // Reset output frame
        hw_encoder_->output_frame.length = 0;
        hw_encoder_->output_frame.frame_type = ESP_H264_FRAME_TYPE_P;
        
        // Perform hardware encoding
        esp_h264_err_t h264_err = esp_h264_enc_process(hw_encoder_->encoder_handle, 
                                                       &hw_encoder_->input_frame, 
                                                       &hw_encoder_->output_frame);
        
        if (h264_err != ESP_H264_ERR_OK) {
            ESP_LOGE(TAG, "Hardware encoding failed: %d", h264_err);
            return ESP_FAIL;
        }
        
        // Extract results
        *output_size = hw_encoder_->output_frame.length;
        *is_keyframe = (hw_encoder_->output_frame.frame_type == ESP_H264_FRAME_TYPE_I);
        
        ESP_LOGD(TAG, "Hardware encoding: frame %lu, %s, input: %zu bytes, output: %zu bytes",
                 (unsigned long)hw_encoder_->frame_count, 
                 *is_keyframe ? "I-frame" : "P-frame",
                 frame->len, *output_size);
        
    } else {
        // Software simulation path
        
        // Simulate H.264 encoding (basic compression simulation)
        size_t simulated_output_size = frame->len / 10; // Simulate ~10:1 compression ratio
        if (simulated_output_size < 1024) simulated_output_size = 1024; // Minimum size
        if (simulated_output_size > config_.max_frame_size) simulated_output_size = config_.max_frame_size;
        
        *output_size = simulated_output_size;
        *is_keyframe = (hw_encoder_->frame_count % config_.gop_size == 1); // I-frame every GOP
        
        ESP_LOGD(TAG, "Software simulation: frame %lu, %s, input: %zu bytes, simulated output: %zu bytes",
                 (unsigned long)hw_encoder_->frame_count, 
                 *is_keyframe ? "I-frame" : "P-frame",
                 frame->len, *output_size);
    }
    
    return ESP_OK;
}

esp_err_t H264Encoder::convertAndCopyFrame(camera_fb_t* frame) {
    if (!frame) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Skip format conversion in simulation mode
    if (!hw_encoder_->input_frame.raw_data.buffer) {
        ESP_LOGD(TAG, "Skipping format conversion in simulation mode");
        return ESP_OK;
    }
    
    // ESP32P4 hardware encoder expects ESP_H264_RAW_FMT_O_UYY_E_VYY format
    // This is a specific YUV420 packed format required by the hardware
    // Camera typically provides JPEG or RGB format
    
    if (frame->format == PIXFORMAT_JPEG) {
        ESP_LOGE(TAG, "JPEG format conversion not implemented yet");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    // For RGB565 or other formats, we need to convert to ESP_H264_RAW_FMT_O_UYY_E_VYY
    if (frame->format == PIXFORMAT_RGB565) {
        return convertRGB565ToOUYYEVYY(frame);
    }
    
    // For YUV422 formats
    if (frame->format == PIXFORMAT_YUV422) {
        return convertYUV422ToOUYYEVYY(frame);
    }
    
    ESP_LOGE(TAG, "Unsupported frame format: %d", frame->format);
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t H264Encoder::convertRGB565ToYUV420(camera_fb_t* frame) {
    // Basic RGB565 to YUV420 conversion
    // This is a simplified implementation - real conversion needs proper color space transformation
    
    uint16_t* rgb_data = (uint16_t*)frame->buf;
    uint8_t* yuv_data = hw_encoder_->input_frame.raw_data.buffer;
    
    size_t pixels = config_.width * config_.height;
    
    // Convert RGB565 to YUV420 (simplified)
    for (size_t i = 0; i < pixels; i++) {
        uint16_t rgb = rgb_data[i];
        uint8_t r = (rgb >> 11) << 3;
        uint8_t g = ((rgb >> 5) & 0x3F) << 2;
        uint8_t b = (rgb & 0x1F) << 3;
        
        // Simple RGB to Y conversion
        yuv_data[i] = (uint8_t)(0.299f * r + 0.587f * g + 0.114f * b);
    }
    
    // Fill U and V planes with neutral values (simplified)
    memset(yuv_data + pixels, 128, pixels / 2);
    
    ESP_LOGD(TAG, "Converted RGB565 to YUV420");
    return ESP_OK;
}

esp_err_t H264Encoder::convertRGB565ToOUYYEVYY(camera_fb_t* frame) {
    // Convert RGB565 to ESP_H264_RAW_FMT_O_UYY_E_VYY format
    // This format has specific line ordering: odd lines U Y Y, even lines V Y Y
    
    uint16_t* rgb_data = (uint16_t*)frame->buf;
    uint8_t* output_data = hw_encoder_->input_frame.raw_data.buffer;
    
    size_t width = config_.width;
    size_t height = config_.height;
    
    ESP_LOGD(TAG, "Converting RGB565 to O_UYY_E_VYY format (%zux%zu)", width, height);
    
    for (size_t y = 0; y < height; y++) {
        size_t line_offset = y * width * 3 / 2;  // Each line uses 1.5 bytes per pixel on average
        
        for (size_t x = 0; x < width; x += 2) {  // Process 2 pixels at a time
            // Get RGB values for 2 adjacent pixels
            uint16_t rgb1 = rgb_data[y * width + x];
            uint16_t rgb2 = (x + 1 < width) ? rgb_data[y * width + x + 1] : rgb1;
            
            // Extract RGB components
            uint8_t r1 = (rgb1 >> 11) << 3, g1 = ((rgb1 >> 5) & 0x3F) << 2, b1 = (rgb1 & 0x1F) << 3;
            uint8_t r2 = (rgb2 >> 11) << 3, g2 = ((rgb2 >> 5) & 0x3F) << 2, b2 = (rgb2 & 0x1F) << 3;
            
            // Convert to YUV
            uint8_t y1 = (uint8_t)(0.299f * r1 + 0.587f * g1 + 0.114f * b1);
            uint8_t y2 = (uint8_t)(0.299f * r2 + 0.587f * g2 + 0.114f * b2);
            uint8_t u = (uint8_t)(128 - 0.169f * r1 - 0.331f * g1 + 0.500f * b1);
            uint8_t v = (uint8_t)(128 + 0.500f * r1 - 0.419f * g1 - 0.081f * b1);
            
            // Store in O_UYY_E_VYY format
            if (y % 2 == 0) {  // Even lines: U Y Y pattern
                output_data[line_offset + (x / 2) * 3] = u;
                output_data[line_offset + (x / 2) * 3 + 1] = y1;
                output_data[line_offset + (x / 2) * 3 + 2] = y2;
            } else {  // Odd lines: V Y Y pattern
                output_data[line_offset + (x / 2) * 3] = v;
                output_data[line_offset + (x / 2) * 3 + 1] = y1;
                output_data[line_offset + (x / 2) * 3 + 2] = y2;
            }
        }
    }
    
    ESP_LOGD(TAG, "Converted RGB565 to O_UYY_E_VYY format");
    return ESP_OK;
}

esp_err_t H264Encoder::convertYUV422ToOUYYEVYY(camera_fb_t* frame) {
    // Convert YUV422 to ESP_H264_RAW_FMT_O_UYY_E_VYY format
    // YUV422 input format: Y U Y V Y U Y V...
    // O_UYY_E_VYY output: odd lines U Y Y, even lines V Y Y
    
    uint8_t* yuv422_data = frame->buf;
    uint8_t* output_data = hw_encoder_->input_frame.raw_data.buffer;
    
    size_t width = config_.width;
    size_t height = config_.height;
    
    ESP_LOGD(TAG, "Converting YUV422 to O_UYY_E_VYY format (%zux%zu)", width, height);
    
    for (size_t y = 0; y < height; y++) {
        size_t line_offset = y * width * 3 / 2;
        
        for (size_t x = 0; x < width; x += 2) {
            // Extract YUV422 data: Y U Y V for 2 pixels
            uint8_t y1 = yuv422_data[(y * width + x) * 2];
            uint8_t u = yuv422_data[(y * width + x) * 2 + 1];
            uint8_t y2 = (x + 1 < width) ? yuv422_data[(y * width + x + 1) * 2] : y1;
            uint8_t v = (x + 1 < width) ? yuv422_data[(y * width + x + 1) * 2 + 1] : u;
            
            // Store in O_UYY_E_VYY format
            if (y % 2 == 0) {  // Even lines: U Y Y pattern
                output_data[line_offset + (x / 2) * 3] = u;
                output_data[line_offset + (x / 2) * 3 + 1] = y1;
                output_data[line_offset + (x / 2) * 3 + 2] = y2;
            } else {  // Odd lines: V Y Y pattern
                output_data[line_offset + (x / 2) * 3] = v;
                output_data[line_offset + (x / 2) * 3 + 1] = y1;
                output_data[line_offset + (x / 2) * 3 + 2] = y2;
            }
        }
    }
    
    ESP_LOGD(TAG, "Converted YUV422 to O_UYY_E_VYY format");
    return ESP_OK;
}

esp_err_t H264Encoder::convertYUV422ToYUV420(camera_fb_t* frame) {
    // Convert YUV422 to YUV420 by subsampling chroma
    uint8_t* yuv422_data = frame->buf;
    uint8_t* yuv420_data = hw_encoder_->input_frame.raw_data.buffer;
    
    size_t width = config_.width;
    size_t height = config_.height;
    
    // Copy Y plane directly
    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            yuv420_data[y * width + x] = yuv422_data[(y * width + x) * 2];
        }
    }
    
    // Subsample UV plane
    uint8_t* u_plane = yuv420_data + width * height;
    uint8_t* v_plane = u_plane + (width * height / 4);
    
    for (size_t y = 0; y < height; y += 2) {
        for (size_t x = 0; x < width; x += 2) {
            u_plane[(y/2) * (width/2) + (x/2)] = yuv422_data[(y * width + x) * 2 + 1];
            v_plane[(y/2) * (width/2) + (x/2)] = yuv422_data[(y * width + x) * 2 + 3];
        }
    }
    
    ESP_LOGD(TAG, "Converted YUV422 to YUV420");
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
        statistics_ = {};
        last_stats_update_ = getCurrentTimestamp();
        xSemaphoreGive(stats_mutex_);
    }
}

bool H264Encoder::isHardwareSupported() {
    // Check if ESP32P4 hardware H.264 encoding is available
    // This may depend on specific chip revision and ESP-IDF version
    
    // Try to create a minimal hardware encoder instance to test support
    esp_h264_enc_cfg_hw_t test_config = {};
    test_config.pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY;
    test_config.gop = 30;
    test_config.fps = 30;
    test_config.res.width = 320;
    test_config.res.height = 240;
    test_config.rc.bitrate = 1000000;
    test_config.rc.qp_min = 10;
    test_config.rc.qp_max = 51;
    
    esp_h264_enc_handle_t test_handle = nullptr;
    esp_h264_err_t h264_err = esp_h264_enc_hw_new(&test_config, &test_handle);
    
    if (h264_err == ESP_H264_ERR_OK && test_handle != nullptr) {
        // Cleanup test encoder
        esp_h264_enc_del(test_handle);
        ESP_LOGI(TAG, "ESP32P4 hardware H.264 encoder is supported");
        return true;
    } else {
        ESP_LOGW(TAG, "ESP32P4 hardware H.264 encoder not available (error: %d)", h264_err);
        return false;
    }
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
    ESP_LOGI(TAG, "Initializing ESP32P4 hardware H.264 encoder");
    
    // Configure hardware encoder parameters
    hw_encoder_->encoder_config.pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY;  // Hardware encoder format
    hw_encoder_->encoder_config.gop = config_.gop_size;
    hw_encoder_->encoder_config.fps = config_.fps;
    hw_encoder_->encoder_config.res.width = config_.width;
    hw_encoder_->encoder_config.res.height = config_.height;
    hw_encoder_->encoder_config.rc.bitrate = config_.bitrate;
    hw_encoder_->encoder_config.rc.qp_min = config_.qp_min;
    hw_encoder_->encoder_config.rc.qp_max = config_.qp_max;
    
    ESP_LOGI(TAG, "H.264 config - Resolution: %lux%lu, FPS: %lu, Bitrate: %lu, GOP: %lu", 
             (unsigned long)config_.width, (unsigned long)config_.height, (unsigned long)config_.fps, (unsigned long)config_.bitrate, (unsigned long)config_.gop_size);
    
    // Create hardware encoder instance
    esp_h264_err_t h264_err = esp_h264_enc_hw_new(&hw_encoder_->encoder_config, &hw_encoder_->encoder_handle);
    if (h264_err != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "Failed to create hardware encoder: %d", h264_err);
        return ESP_FAIL;
    }
    
    // Calculate buffer sizes (align to 16 bytes for hardware requirements)
    uint16_t aligned_width = ((config_.width + 15) >> 4) << 4;
    uint16_t aligned_height = ((config_.height + 15) >> 4) << 4;
    size_t input_buffer_size = aligned_width * aligned_height * 3 / 2; // YUV420 equivalent for ESP_H264_RAW_FMT_O_UYY_E_VYY
    size_t output_buffer_size = config_.max_frame_size;
    
    ESP_LOGI(TAG, "Allocating aligned buffers - Input: %zu bytes, Output: %zu bytes", 
             input_buffer_size, output_buffer_size);
    
    // Allocate input buffer (16-byte aligned for hardware)
    uint32_t actual_input_size = 0;
    hw_encoder_->input_frame.raw_data.buffer = (uint8_t*)esp_h264_aligned_calloc(16, 1, input_buffer_size, 
                                                                                &actual_input_size, 
                                                                                ESP_H264_MEM_INTERNAL);
    hw_encoder_->input_frame.raw_data.len = actual_input_size;
    
    if (!hw_encoder_->input_frame.raw_data.buffer) {
        ESP_LOGE(TAG, "Failed to allocate input buffer (%zu bytes)", input_buffer_size);
        esp_h264_enc_del(hw_encoder_->encoder_handle);
        return ESP_ERR_NO_MEM;
    }
    
    // Allocate output buffer (16-byte aligned for hardware)
    uint32_t actual_output_size = 0;
    hw_encoder_->output_frame.raw_data.buffer = (uint8_t*)esp_h264_aligned_calloc(16, 1, output_buffer_size,
                                                                                 &actual_output_size,
                                                                                 ESP_H264_MEM_INTERNAL);
    hw_encoder_->output_frame.raw_data.len = actual_output_size;
    
    if (!hw_encoder_->output_frame.raw_data.buffer) {
        ESP_LOGE(TAG, "Failed to allocate output buffer (%zu bytes)", output_buffer_size);
        esp_h264_free(hw_encoder_->input_frame.raw_data.buffer);
        esp_h264_enc_del(hw_encoder_->encoder_handle);
        return ESP_ERR_NO_MEM;
    }
    
    hw_encoder_->frame_count = 0;
    hw_encoder_->initialized = true;
    
    ESP_LOGI(TAG, "ESP32P4 hardware H.264 encoder initialized successfully");
    ESP_LOGI(TAG, "Allocated buffers - Input: %lu bytes, Output: %lu bytes", 
             (unsigned long)actual_input_size, (unsigned long)actual_output_size);
    
    return ESP_OK;
}

esp_err_t H264Encoder::initializeSoftwareSimulation() {
    ESP_LOGI(TAG, "Initializing H.264 software simulation mode");
    
    // For software simulation, we don't need hardware encoder handles
    // Just ensure the hw_encoder structure is reset
    if (hw_encoder_) {
        hw_encoder_->encoder_handle = nullptr;
        hw_encoder_->initialized = false;
        hw_encoder_->frame_count = 0;
        
        // No need to allocate hardware buffers for simulation
        hw_encoder_->input_frame.raw_data.buffer = nullptr;
        hw_encoder_->input_frame.raw_data.len = 0;
        hw_encoder_->output_frame.raw_data.buffer = nullptr;
        hw_encoder_->output_frame.raw_data.len = 0;
    }
    
    ESP_LOGI(TAG, "H.264 software simulation initialized successfully");
    return ESP_OK;
}

esp_err_t H264Encoder::startHardwareEncoder() {
    if (!hw_encoder_) {
        return ESP_FAIL;
    }
    
    // Check if we're in hardware mode
    if (hw_encoder_->initialized && hw_encoder_->encoder_handle) {
        // Open the hardware encoder
        esp_h264_err_t h264_err = esp_h264_enc_open(hw_encoder_->encoder_handle);
        if (h264_err != ESP_H264_ERR_OK) {
            ESP_LOGE(TAG, "Failed to open hardware encoder: %d", h264_err);
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "ESP32P4 hardware H.264 encoder started");
    } else {
        ESP_LOGI(TAG, "H.264 software simulation mode started");
    }
    
    return ESP_OK;
}

esp_err_t H264Encoder::stopHardwareEncoder() {
    if (!hw_encoder_) {
        return ESP_OK;
    }
    
    // Check if we're in hardware mode
    if (hw_encoder_->initialized && hw_encoder_->encoder_handle) {
        // Close the hardware encoder
        esp_h264_err_t h264_err = esp_h264_enc_close(hw_encoder_->encoder_handle);
        if (h264_err != ESP_H264_ERR_OK) {
            ESP_LOGE(TAG, "Failed to close hardware encoder: %d", h264_err);
        }
        ESP_LOGI(TAG, "ESP32P4 hardware H.264 encoder stopped");
    } else {
        ESP_LOGI(TAG, "H.264 software simulation mode stopped");
    }
    
    return ESP_OK;
}

void H264Encoder::encodingTask(void* parameter) {
    H264Encoder* self = static_cast<H264Encoder*>(parameter);
    ESP_LOGI(TAG, "Encoding task started");
    
    EncodingFrame input_frame;
    
    while (self->encoding_active_) {
        // Wait for input frame
        if (xQueueReceive(self->input_queue_, &input_frame, pdMS_TO_TICKS(100)) == pdTRUE) {
            uint64_t start_time = self->getCurrentTimestamp();
            
            // Frame format conversion and encoding is now handled in performHardwareEncoding
            
            // Hardware H.264 encoding
            size_t output_size = 0;
            bool is_keyframe = false;
            
            // Perform hardware encoding
            esp_err_t ret = self->performHardwareEncoding(input_frame.frame, &output_size, &is_keyframe);
            
            if (ret == ESP_OK && output_size > 0) {
                // Create frame info for callback
                FrameInfo frame_info = {
                    .data = self->hw_encoder_->output_frame.raw_data.buffer, // May be null in simulation mode
                    .size = output_size,
                    .timestamp_us = input_frame.timestamp_us,
                    .is_keyframe = is_keyframe,
                    .frame_number = 0  // Will be set by updateStatistics
                };
                
                // Update statistics
                uint64_t encoding_time = self->getCurrentTimestamp() - start_time;
                self->updateStatistics(output_size, encoding_time, is_keyframe);
                
                // Set frame number
                frame_info.frame_number = self->statistics_.frames_encoded;
                
                // Call frame callback if set
                if (self->frame_callback_) {
                    self->frame_callback_(frame_info);
                }
                
                ESP_LOGD(TAG, "Simulated H.264 frame: %zu bytes, keyframe: %d, time: %llu us",
                         output_size, is_keyframe, encoding_time);
            } else {
                ESP_LOGE(TAG, "Failed to simulate encoding: %s", esp_err_to_name(ret));
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
    // Initialize pointers to null for safe cleanup
    input_buffers_ = nullptr;
    output_buffers_ = nullptr;
    input_buffer_pool_ = nullptr;
    output_buffer_pool_ = nullptr;
    
    // Allocate input buffer pool
    input_buffers_ = new(std::nothrow) uint8_t*[config_.input_buffer_count];
    if (!input_buffers_) {
        ESP_LOGE(TAG, "Failed to allocate input buffer array");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize all pointers to nullptr for safe cleanup
    for (uint32_t i = 0; i < config_.input_buffer_count; i++) {
        input_buffers_[i] = nullptr;
    }
    
    input_buffer_pool_ = xQueueCreate(config_.input_buffer_count, sizeof(uint8_t*));
    if (!input_buffer_pool_) {
        ESP_LOGE(TAG, "Failed to create input buffer pool");
        return ESP_ERR_NO_MEM;
    }
    
    size_t input_buffer_size = config_.width * config_.height * 3 / 2; // YUV420
    ESP_LOGI(TAG, "Allocating %lu input buffers of %zu bytes each", (unsigned long)config_.input_buffer_count, input_buffer_size);
    for (uint32_t i = 0; i < config_.input_buffer_count; i++) {
        input_buffers_[i] = (uint8_t*)heap_caps_malloc(input_buffer_size, MALLOC_CAP_DEFAULT);
        if (!input_buffers_[i]) {
            ESP_LOGE(TAG, "Failed to allocate input buffer %lu (%zu bytes)", (unsigned long)i, input_buffer_size);
            ESP_LOGE(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
            return ESP_ERR_NO_MEM;
        }
        xQueueSend(input_buffer_pool_, &input_buffers_[i], 0);
    }
    
    // Allocate output buffer pool
    output_buffers_ = new(std::nothrow) uint8_t*[config_.output_buffer_count];
    if (!output_buffers_) {
        ESP_LOGE(TAG, "Failed to allocate output buffer array");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize all pointers to nullptr for safe cleanup
    for (uint32_t i = 0; i < config_.output_buffer_count; i++) {
        output_buffers_[i] = nullptr;
    }
    
    output_buffer_pool_ = xQueueCreate(config_.output_buffer_count, sizeof(uint8_t*));
    if (!output_buffer_pool_) {
        ESP_LOGE(TAG, "Failed to create output buffer pool");
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "Allocating %lu output buffers of %lu bytes each", (unsigned long)config_.output_buffer_count, (unsigned long)config_.max_frame_size);
    for (uint32_t i = 0; i < config_.output_buffer_count; i++) {
        output_buffers_[i] = (uint8_t*)heap_caps_malloc(config_.max_frame_size, MALLOC_CAP_DEFAULT);
        if (!output_buffers_[i]) {
            ESP_LOGE(TAG, "Failed to allocate output buffer %lu (%lu bytes)", (unsigned long)i, (unsigned long)config_.max_frame_size);
            ESP_LOGE(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
            return ESP_ERR_NO_MEM;
        }
        xQueueSend(output_buffer_pool_, &output_buffers_[i], 0);
    }
    
    ESP_LOGI(TAG, "Allocated %lu input and %lu output buffers", 
             (unsigned long)config_.input_buffer_count, (unsigned long)config_.output_buffer_count);
    
    return ESP_OK;
}

void H264Encoder::deallocateBuffers() {
    ESP_LOGD(TAG, "Deallocating buffers");
    
    // Clean up input buffer pool first (prevents queue operations on freed memory)
    if (input_buffer_pool_) {
        vQueueDelete(input_buffer_pool_);
        input_buffer_pool_ = nullptr;
    }
    
    // Clean up input buffers
    if (input_buffers_) {
        for (uint32_t i = 0; i < config_.input_buffer_count; i++) {
            if (input_buffers_[i]) {
                heap_caps_free(input_buffers_[i]);
                input_buffers_[i] = nullptr;
            }
        }
        delete[] input_buffers_;
        input_buffers_ = nullptr;
    }
    
    // Clean up output buffer pool first
    if (output_buffer_pool_) {
        vQueueDelete(output_buffer_pool_);
        output_buffer_pool_ = nullptr;
    }
    
    // Clean up output buffers
    if (output_buffers_) {
        for (uint32_t i = 0; i < config_.output_buffer_count; i++) {
            if (output_buffers_[i]) {
                heap_caps_free(output_buffers_[i]);
                output_buffers_[i] = nullptr;
            }
        }
        delete[] output_buffers_;
        output_buffers_ = nullptr;
    }
    
    ESP_LOGD(TAG, "Buffer deallocation completed");
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