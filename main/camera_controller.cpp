#include "camera_controller.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

static const char *TAG = "CameraController";

CameraController::CameraController() 
    : current_state_(CameraState::UNINITIALIZED)
    , streaming_task_handle_(nullptr)
    , streaming_active_(false)
{
}

CameraController::~CameraController() {
    deinitialize();
}

esp_err_t CameraController::initialize(const CameraConfig& config) {
    ESP_LOGI(TAG, "Initializing camera controller");
    
    config_ = config;
    
    camera_config_t camera_config = {
        .pin_pwdn = config_.pin_pwdn,
        .pin_reset = config_.pin_reset,
        .pin_xclk = config_.pin_xclk,
        .pin_sccb_sda = config_.pin_sccb_sda,
        .pin_sccb_scl = config_.pin_sccb_scl,
        .pin_d7 = config_.pin_d7,
        .pin_d6 = config_.pin_d6,
        .pin_d5 = config_.pin_d5,
        .pin_d4 = config_.pin_d4,
        .pin_d3 = config_.pin_d3,
        .pin_d2 = config_.pin_d2,
        .pin_d1 = config_.pin_d1,
        .pin_d0 = config_.pin_d0,
        .pin_vsync = config_.pin_vsync,
        .pin_href = config_.pin_href,
        .pin_pclk = config_.pin_pclk,
        .xclk_freq_hz = config_.xclk_freq_hz,
        .ledc_timer = config_.ledc_timer,
        .ledc_channel = config_.ledc_channel,
        .pixel_format = config_.pixel_format,
        .frame_size = config_.frame_size,
        .jpeg_quality = config_.jpeg_quality,
        .fb_count = config_.fb_count,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = (camera_grab_mode_t)config_.grab_mode,
        .sccb_i2c_port = 0
    };
    
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        setState(CameraState::ERROR, "Camera initialization failed");
        return err;
    }
    
    sensor_t* sensor = esp_camera_sensor_get();
    if (sensor != nullptr) {
        sensor->set_vflip(sensor, 1);       // Flip vertically
        sensor->set_hmirror(sensor, 1);     // Mirror horizontally
        sensor->set_brightness(sensor, 0);   // Brightness (-2 to 2)
        sensor->set_contrast(sensor, 0);     // Contrast (-2 to 2)
        sensor->set_saturation(sensor, 0);   // Saturation (-2 to 2)
        sensor->set_special_effect(sensor, 0); // No special effect
        sensor->set_whitebal(sensor, 1);     // Enable white balance
        sensor->set_awb_gain(sensor, 1);     // Enable AWB gain
        sensor->set_wb_mode(sensor, 0);      // Auto white balance mode
        sensor->set_exposure_ctrl(sensor, 1); // Enable exposure control
        sensor->set_aec2(sensor, 0);         // Disable AEC2
        sensor->set_ae_level(sensor, 0);     // AE level (-2 to 2)
        sensor->set_aec_value(sensor, 300);  // AEC value
        sensor->set_gain_ctrl(sensor, 1);    // Enable gain control
        sensor->set_agc_gain(sensor, 0);     // AGC gain (0 to 30)
        sensor->set_gainceiling(sensor, (gainceiling_t)0); // Gain ceiling
        sensor->set_bpc(sensor, 0);          // Disable BPC
        sensor->set_wpc(sensor, 1);          // Enable WPC
        sensor->set_raw_gma(sensor, 1);      // Enable raw GMA
        sensor->set_lenc(sensor, 1);         // Enable lens correction
        sensor->set_dcw(sensor, 1);          // Enable DCW
        sensor->set_colorbar(sensor, 0);     // Disable color bar
        
        ESP_LOGI(TAG, "Camera sensor configured");
    }
    
    setState(CameraState::INITIALIZED, "Camera initialized successfully");
    ESP_LOGI(TAG, "Camera initialized with frame size: %dx%d", 
             camera_config.frame_size == FRAMESIZE_SVGA ? 800 : 640,
             camera_config.frame_size == FRAMESIZE_SVGA ? 600 : 480);
    
    return ESP_OK;
}

esp_err_t CameraController::deinitialize() {
    if (current_state_ == CameraState::UNINITIALIZED) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Deinitializing camera");
    
    stopStreaming();
    
    esp_err_t err = esp_camera_deinit();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera deinit failed with error 0x%x", err);
        setState(CameraState::ERROR, "Camera deinitialization failed");
        return err;
    }
    
    setState(CameraState::UNINITIALIZED, "Camera deinitialized");
    return ESP_OK;
}

camera_fb_t* CameraController::captureFrame() {
    if (current_state_ != CameraState::INITIALIZED && current_state_ != CameraState::STREAMING) {
        ESP_LOGE(TAG, "Camera not initialized");
        return nullptr;
    }
    
    camera_fb_t* frame = esp_camera_fb_get();
    if (!frame) {
        ESP_LOGE(TAG, "Failed to capture frame");
        setState(CameraState::ERROR, "Frame capture failed");
        return nullptr;
    }
    
    ESP_LOGD(TAG, "Frame captured: %zu bytes, format: %d", frame->len, frame->format);
    return frame;
}

void CameraController::returnFrame(camera_fb_t* frame) {
    if (frame) {
        esp_camera_fb_return(frame);
        ESP_LOGD(TAG, "Frame buffer returned");
    }
}

esp_err_t CameraController::startStreaming(FrameCallback callback) {
    if (current_state_ != CameraState::INITIALIZED) {
        ESP_LOGE(TAG, "Camera not initialized for streaming");
        return ESP_FAIL;
    }
    
    if (streaming_active_) {
        ESP_LOGW(TAG, "Streaming already active");
        return ESP_OK;
    }
    
    frame_callback_ = callback;
    streaming_active_ = true;
    
    BaseType_t result = xTaskCreate(
        streamingTask,
        "camera_stream",
        4096,
        this,
        5,
        &streaming_task_handle_
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create streaming task");
        streaming_active_ = false;
        setState(CameraState::ERROR, "Failed to start streaming");
        return ESP_FAIL;
    }
    
    setState(CameraState::STREAMING, "Camera streaming started");
    ESP_LOGI(TAG, "Camera streaming started");
    return ESP_OK;
}

esp_err_t CameraController::stopStreaming() {
    if (!streaming_active_) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Stopping camera streaming");
    streaming_active_ = false;
    
    if (streaming_task_handle_) {
        vTaskDelete(streaming_task_handle_);
        streaming_task_handle_ = nullptr;
    }
    
    setState(CameraState::INITIALIZED, "Camera streaming stopped");
    return ESP_OK;
}

void CameraController::streamingTask(void* parameter) {
    CameraController* self = static_cast<CameraController*>(parameter);
    
    ESP_LOGI(TAG, "Streaming task started");
    
    while (self->streaming_active_) {
        camera_fb_t* frame = self->captureFrame();
        if (frame) {
            if (self->frame_callback_) {
                self->frame_callback_(frame);
            }
            self->returnFrame(frame);
        } else {
            ESP_LOGE(TAG, "Failed to capture frame in streaming mode");
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        vTaskDelay(pdMS_TO_TICKS(33)); // ~30 FPS
    }
    
    ESP_LOGI(TAG, "Streaming task ended");
    vTaskDelete(nullptr);
}

esp_err_t CameraController::setFrameSize(framesize_t size) {
    sensor_t* sensor = esp_camera_sensor_get();
    if (sensor && sensor->set_framesize) {
        int result = sensor->set_framesize(sensor, size);
        if (result == 0) {
            config_.frame_size = size;
            ESP_LOGI(TAG, "Frame size changed to: %d", size);
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "Failed to set frame size");
    return ESP_FAIL;
}

esp_err_t CameraController::setPixelFormat(pixformat_t format) {
    sensor_t* sensor = esp_camera_sensor_get();
    if (sensor && sensor->set_pixformat) {
        int result = sensor->set_pixformat(sensor, format);
        if (result == 0) {
            config_.pixel_format = format;
            ESP_LOGI(TAG, "Pixel format changed to: %d", format);
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "Failed to set pixel format");
    return ESP_FAIL;
}

esp_err_t CameraController::setJpegQuality(int quality) {
    if (quality < 0 || quality > 63) {
        ESP_LOGE(TAG, "Invalid JPEG quality: %d (should be 0-63)", quality);
        return ESP_FAIL;
    }
    
    sensor_t* sensor = esp_camera_sensor_get();
    if (sensor && sensor->set_quality) {
        int result = sensor->set_quality(sensor, quality);
        if (result == 0) {
            config_.jpeg_quality = quality;
            ESP_LOGI(TAG, "JPEG quality changed to: %d", quality);
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "Failed to set JPEG quality");
    return ESP_FAIL;
}

esp_err_t CameraController::setSpecialEffect(int effect) {
    sensor_t* sensor = esp_camera_sensor_get();
    if (sensor && sensor->set_special_effect) {
        int result = sensor->set_special_effect(sensor, effect);
        if (result == 0) {
            ESP_LOGI(TAG, "Special effect changed to: %d", effect);
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "Failed to set special effect");
    return ESP_FAIL;
}

esp_err_t CameraController::setWhiteBalance(int wb_mode) {
    sensor_t* sensor = esp_camera_sensor_get();
    if (sensor && sensor->set_wb_mode) {
        int result = sensor->set_wb_mode(sensor, wb_mode);
        if (result == 0) {
            ESP_LOGI(TAG, "White balance mode changed to: %d", wb_mode);
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "Failed to set white balance mode");
    return ESP_FAIL;
}

esp_err_t CameraController::setExposureCtrl(int ae_level) {
    sensor_t* sensor = esp_camera_sensor_get();
    if (sensor && sensor->set_ae_level) {
        int result = sensor->set_ae_level(sensor, ae_level);
        if (result == 0) {
            ESP_LOGI(TAG, "Exposure level changed to: %d", ae_level);
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "Failed to set exposure level");
    return ESP_FAIL;
}

esp_err_t CameraController::setGainCtrl(int gain_level) {
    sensor_t* sensor = esp_camera_sensor_get();
    if (sensor && sensor->set_agc_gain) {
        int result = sensor->set_agc_gain(sensor, gain_level);
        if (result == 0) {
            ESP_LOGI(TAG, "Gain level changed to: %d", gain_level);
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "Failed to set gain level");
    return ESP_FAIL;
}

void CameraController::setStateCallback(StateCallback callback) {
    state_callback_ = callback;
}

void CameraController::setState(CameraState state, const std::string& message) {
    current_state_ = state;
    ESP_LOGD(TAG, "State changed to %d: %s", (int)state, message.c_str());
    
    if (state_callback_) {
        state_callback_(state, message);
    }
}