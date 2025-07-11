#pragma once

#include <string>
#include <functional>
#include <memory>
#include <vector>
#include "esp_err.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Advanced motion detection with scene change analysis
// Designed for intelligent recording control based on actual content changes
class MotionDetector {
public:
    enum class DetectionState {
        IDLE,           // No motion detected
        MOTION,         // Active motion detected
        COOLDOWN        // Post-motion cooldown period
    };

    enum class DetectionMode {
        SIMPLE,         // Basic frame difference
        ADVANCED,       // Multi-region analysis with noise filtering
        ADAPTIVE        // Self-adjusting sensitivity based on environment
    };

    struct DetectionConfig {
        // Basic detection parameters
        DetectionMode mode = DetectionMode::ADVANCED;
        uint32_t sensitivity = 50;              // 0-100, motion sensitivity
        uint32_t noise_threshold = 10;          // Ignore small changes (noise)
        uint32_t min_motion_pixels = 100;      // Minimum pixels changed for motion
        float motion_percentage_threshold = 2.0f; // Minimum % of frame changed
        
        // Temporal filtering
        uint32_t motion_debounce_ms = 1000;     // Motion must persist for this duration
        uint32_t no_motion_timeout_ms = 30000;  // Auto-stop recording after this time
        uint32_t cooldown_period_ms = 5000;     // Cooldown after motion stops
        
        // Advanced detection
        bool enable_region_analysis = true;     // Divide frame into regions
        uint32_t detection_regions_x = 8;       // Horizontal regions
        uint32_t detection_regions_y = 6;       // Vertical regions
        uint32_t min_active_regions = 2;        // Minimum regions with motion
        
        // Adaptive parameters
        bool enable_adaptive_sensitivity = true;
        uint32_t adaptation_period_ms = 60000;  // Adapt every minute
        float adaptation_factor = 0.1f;         // Adaptation speed (0.0-1.0)
        
        // Environmental compensation
        bool enable_lighting_compensation = true;
        bool enable_noise_filtering = true;
        uint32_t noise_filter_strength = 3;     // Noise filter kernel size
        
        // Performance settings
        uint32_t frame_skip_count = 1;          // Process every N frames
        bool enable_downscaling = true;         // Downscale for faster processing
        uint32_t downscale_factor = 2;          // Downscale factor (2 = 1/4 pixels)
    };

    struct DetectionResult {
        DetectionState state = DetectionState::IDLE;
        float motion_percentage = 0.0f;
        uint32_t motion_pixels = 0;
        uint32_t active_regions = 0;
        uint64_t last_motion_time = 0;
        uint64_t motion_duration_ms = 0;
        uint64_t no_motion_duration_ms = 0;
        bool recording_should_start = false;
        bool recording_should_stop = false;
        float average_scene_brightness = 0.0f;
        float scene_change_factor = 0.0f;
    };

    struct Statistics {
        uint64_t total_frames_processed = 0;
        uint64_t motion_events_detected = 0;
        uint64_t false_positives_filtered = 0;
        uint64_t total_motion_time_ms = 0;
        uint64_t total_idle_time_ms = 0;
        float avg_motion_percentage = 0.0f;
        float current_sensitivity = 50.0f;
        uint32_t adaptive_adjustments = 0;
        double avg_processing_time_ms = 0.0f;
    };

    using MotionCallback = std::function<void(const DetectionResult& result)>;
    using StateCallback = std::function<void(DetectionState state, const std::string& message)>;

    MotionDetector();
    ~MotionDetector();

    // Lifecycle management
    esp_err_t initialize(const DetectionConfig& config);
    esp_err_t deinitialize();
    esp_err_t start();
    esp_err_t stop();

    // Frame processing
    esp_err_t processFrame(camera_fb_t* frame);
    DetectionResult getLastResult() const;
    
    // Configuration
    esp_err_t updateConfig(const DetectionConfig& config);
    esp_err_t setSensitivity(uint32_t sensitivity);
    esp_err_t setMotionTimeout(uint32_t timeout_ms);
    esp_err_t enableAdaptiveSensitivity(bool enable);
    
    // Callbacks
    void setMotionCallback(MotionCallback callback);
    void setStateCallback(StateCallback callback);
    
    // Status and statistics
    DetectionState getState() const { return current_state_; }
    bool isMotionDetected() const { return current_state_ == DetectionState::MOTION; }
    Statistics getStatistics() const;
    void resetStatistics();
    
    // Debugging and analysis
    esp_err_t saveDebugFrame(const std::string& filename = "");
    void printDetectionStatus() const;
    std::vector<uint8_t> getMotionMask() const;  // For visualization
    
    // Calibration
    esp_err_t calibrateBackground(uint32_t calibration_frames = 30);
    esp_err_t resetBackground();
    bool isCalibrated() const { return background_calibrated_; }

private:
    // Frame processing algorithms
    esp_err_t preprocessFrame(camera_fb_t* frame, std::vector<uint8_t>& processed);
    esp_err_t detectSimpleMotion(const std::vector<uint8_t>& current, const std::vector<uint8_t>& reference);
    esp_err_t detectAdvancedMotion(const std::vector<uint8_t>& current, const std::vector<uint8_t>& reference);
    esp_err_t detectAdaptiveMotion(const std::vector<uint8_t>& current, const std::vector<uint8_t>& reference);
    
    // Image processing utilities
    void convertToGrayscale(camera_fb_t* frame, std::vector<uint8_t>& gray);
    void applyNoiseFilter(std::vector<uint8_t>& image, uint32_t kernel_size);
    void downscaleImage(const std::vector<uint8_t>& input, std::vector<uint8_t>& output, uint32_t factor);
    float calculateBrightness(const std::vector<uint8_t>& image);
    void updateBackground(const std::vector<uint8_t>& current, float learning_rate = 0.05f);
    
    // Region-based analysis
    esp_err_t analyzeRegions(const std::vector<uint8_t>& diff_image);
    uint32_t countMotionPixels(const std::vector<uint8_t>& diff_image, uint32_t threshold);
    void markMotionRegions(std::vector<uint8_t>& motion_mask);
    
    // Temporal filtering and state management
    void updateDetectionState(const DetectionResult& current_result);
    void applyTemporalFiltering();
    bool shouldTriggerMotion(const DetectionResult& result);
    bool shouldStopMotion();
    
    // Adaptive sensitivity
    void updateAdaptiveSensitivity();
    void adjustSensitivity(float environment_factor);
    float calculateEnvironmentFactor();
    
    // Statistics and monitoring
    void updateStatistics(const DetectionResult& result, uint64_t processing_time_us);
    void setState(DetectionState state, const std::string& message = "");
    uint64_t getCurrentTimestamp();
    
    // Configuration and state
    DetectionConfig config_;
    DetectionState current_state_;
    DetectionResult last_result_;
    
    // Frame data
    std::vector<uint8_t> background_frame_;
    std::vector<uint8_t> previous_frame_;
    std::vector<uint8_t> current_frame_processed_;
    std::vector<uint8_t> difference_frame_;
    std::vector<uint8_t> motion_mask_;
    
    // Frame properties
    uint32_t frame_width_;
    uint32_t frame_height_;
    uint32_t processed_width_;
    uint32_t processed_height_;
    bool background_calibrated_;
    
    // Region analysis data
    std::vector<bool> region_motion_flags_;
    std::vector<float> region_motion_levels_;
    
    // Temporal state
    uint64_t last_motion_timestamp_;
    uint64_t motion_start_timestamp_;
    uint64_t last_frame_timestamp_;
    uint64_t state_change_timestamp_;
    uint32_t consecutive_motion_frames_;
    uint32_t consecutive_idle_frames_;
    
    // Adaptive sensitivity data
    float current_sensitivity_;
    uint64_t last_adaptation_time_;
    std::vector<float> recent_motion_levels_;
    uint32_t adaptation_sample_count_;
    
    // Callbacks
    MotionCallback motion_callback_;
    StateCallback state_callback_;
    
    // Statistics
    mutable SemaphoreHandle_t stats_mutex_;
    Statistics statistics_;
    
    // Frame processing optimization
    uint32_t frame_counter_;
    bool processing_enabled_;
    
    static const char* TAG;
};