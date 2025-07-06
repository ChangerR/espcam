#pragma once

#include <functional>
#include <memory>
#include "esp_camera.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class CameraController {
public:
    enum class CameraState {
        UNINITIALIZED,
        INITIALIZED,
        STREAMING,
        ERROR
    };

    struct CameraConfig {
        // Pin configuration for ESP32P4
        int pin_pwdn = -1;      // Power down pin
        int pin_reset = -1;     // Reset pin  
        int pin_xclk = 15;      // External clock pin
        int pin_sccb_sda = 4;   // I2C SDA pin
        int pin_sccb_scl = 5;   // I2C SCL pin
        int pin_d7 = 16;        // Data pin 7
        int pin_d6 = 17;        // Data pin 6
        int pin_d5 = 18;        // Data pin 5
        int pin_d4 = 12;        // Data pin 4
        int pin_d3 = 10;        // Data pin 3
        int pin_d2 = 8;         // Data pin 2
        int pin_d1 = 9;         // Data pin 1
        int pin_d0 = 11;        // Data pin 0
        int pin_vsync = 6;      // VSYNC pin
        int pin_href = 7;       // HREF pin
        int pin_pclk = 13;      // Pixel clock pin
        
        // Camera settings
        pixformat_t pixel_format = PIXFORMAT_JPEG;
        framesize_t frame_size = FRAMESIZE_SVGA;
        int jpeg_quality = 12;  // 0-63, lower means higher quality
        size_t fb_count = 2;    // Frame buffer count
        bool grab_mode = 0;     // When enabled, camera takes continuous pictures
        
        // Clock settings
        int xclk_freq_hz = 20000000;
        ledc_timer_t ledc_timer = LEDC_TIMER_0;
        ledc_channel_t ledc_channel = LEDC_CHANNEL_0;
    };

    using FrameCallback = std::function<void(camera_fb_t* frame)>;
    using StateCallback = std::function<void(CameraState state, const std::string& message)>;

    CameraController();
    ~CameraController();

    esp_err_t initialize(const CameraConfig& config);
    esp_err_t deinitialize();
    
    camera_fb_t* captureFrame();
    void returnFrame(camera_fb_t* frame);
    
    esp_err_t startStreaming(FrameCallback callback);
    esp_err_t stopStreaming();
    
    esp_err_t setFrameSize(framesize_t size);
    esp_err_t setPixelFormat(pixformat_t format);
    esp_err_t setJpegQuality(int quality);
    
    esp_err_t setSpecialEffect(int effect);
    esp_err_t setWhiteBalance(int wb_mode);
    esp_err_t setExposureCtrl(int ae_level);
    esp_err_t setGainCtrl(int gain_level);
    
    bool isInitialized() const { return current_state_ != CameraState::UNINITIALIZED; }
    bool isStreaming() const { return current_state_ == CameraState::STREAMING; }
    CameraState getState() const { return current_state_; }
    
    void setStateCallback(StateCallback callback);

private:
    static void streamingTask(void* parameter);
    void setState(CameraState state, const std::string& message = "");
    
    CameraConfig config_;
    CameraState current_state_;
    FrameCallback frame_callback_;
    StateCallback state_callback_;
    
    TaskHandle_t streaming_task_handle_;
    bool streaming_active_;
};