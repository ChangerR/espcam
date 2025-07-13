#include "motion_detector.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <inttypes.h>

const char* MotionDetector::TAG = "MotionDetector";

MotionDetector::MotionDetector()
    : current_state_(DetectionState::IDLE)
    , frame_width_(0)
    , frame_height_(0)
    , processed_width_(0)
    , processed_height_(0)
    , background_calibrated_(false)
    , last_motion_timestamp_(0)
    , motion_start_timestamp_(0)
    , last_frame_timestamp_(0)
    , state_change_timestamp_(0)
    , consecutive_motion_frames_(0)
    , consecutive_idle_frames_(0)
    , current_sensitivity_(50.0f)
    , last_adaptation_time_(0)
    , adaptation_sample_count_(0)
    , stats_mutex_(nullptr)
    , frame_counter_(0)
    , processing_enabled_(false)
{
    stats_mutex_ = xSemaphoreCreateMutex();
    if (!stats_mutex_) {
        ESP_LOGE(TAG, "Failed to create statistics mutex");
    }
    
    statistics_ = {};
    last_result_ = {};
}

MotionDetector::~MotionDetector() {
    deinitialize();
    
    if (stats_mutex_) {
        vSemaphoreDelete(stats_mutex_);
    }
}

esp_err_t MotionDetector::initialize(const DetectionConfig& config) {
    ESP_LOGI(TAG, "Initializing motion detector");
    
    config_ = config;
    current_sensitivity_ = config_.sensitivity;
    current_state_ = DetectionState::IDLE;
    state_change_timestamp_ = getCurrentTimestamp();
    
    // Initialize timing
    last_frame_timestamp_ = 0;
    last_motion_timestamp_ = 0;
    motion_start_timestamp_ = 0;
    last_adaptation_time_ = getCurrentTimestamp();
    
    // Reset counters
    frame_counter_ = 0;
    consecutive_motion_frames_ = 0;
    consecutive_idle_frames_ = 0;
    adaptation_sample_count_ = 0;
    
    // Clear sample history
    recent_motion_levels_.clear();
    recent_motion_levels_.reserve(100);  // Store last 100 motion level samples
    
    ESP_LOGI(TAG, "Motion detector initialized: mode=%d, sensitivity=%.1f, timeout=%" PRIu32 " ms",
             (int)config_.mode, current_sensitivity_, config_.no_motion_timeout_ms);
    
    return ESP_OK;
}

esp_err_t MotionDetector::deinitialize() {
    ESP_LOGI(TAG, "Deinitializing motion detector");
    
    processing_enabled_ = false;
    
    // Clear all frame data
    background_frame_.clear();
    previous_frame_.clear();
    current_frame_processed_.clear();
    difference_frame_.clear();
    motion_mask_.clear();
    region_motion_flags_.clear();
    region_motion_levels_.clear();
    recent_motion_levels_.clear();
    
    background_calibrated_ = false;
    current_state_ = DetectionState::IDLE;
    
    return ESP_OK;
}

esp_err_t MotionDetector::start() {
    ESP_LOGI(TAG, "Starting motion detection");
    processing_enabled_ = true;
    resetStatistics();
    return ESP_OK;
}

esp_err_t MotionDetector::stop() {
    ESP_LOGI(TAG, "Stopping motion detection");
    processing_enabled_ = false;
    return ESP_OK;
}

esp_err_t MotionDetector::processFrame(camera_fb_t* frame) {
    if (!processing_enabled_ || !frame || !frame->buf) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint64_t process_start = getCurrentTimestamp();
    
    // Skip frames if configured
    if (++frame_counter_ % (config_.frame_skip_count + 1) != 0) {
        return ESP_OK;
    }
    
    // Initialize frame dimensions on first frame
    if (frame_width_ == 0 || frame_height_ == 0) {
        frame_width_ = frame->width;
        frame_height_ = frame->height;
        
        if (config_.enable_downscaling) {
            processed_width_ = frame_width_ / config_.downscale_factor;
            processed_height_ = frame_height_ / config_.downscale_factor;
        } else {
            processed_width_ = frame_width_;
            processed_height_ = frame_height_;
        }
        
        // Initialize buffers
        size_t processed_size = processed_width_ * processed_height_;
        background_frame_.resize(processed_size, 0);
        previous_frame_.resize(processed_size, 0);
        current_frame_processed_.resize(processed_size, 0);
        difference_frame_.resize(processed_size, 0);
        motion_mask_.resize(processed_size, 0);
        
        // Initialize region analysis
        if (config_.enable_region_analysis) {
            size_t region_count = config_.detection_regions_x * config_.detection_regions_y;
            region_motion_flags_.resize(region_count, false);
            region_motion_levels_.resize(region_count, 0.0f);
        }
        
        ESP_LOGI(TAG, "Frame dimensions: %" PRIu32 "x%" PRIu32 ", processed: %" PRIu32 "x%" PRIu32 "",
                 frame_width_, frame_height_, processed_width_, processed_height_);
    }
    
    // Preprocess current frame
    esp_err_t ret = preprocessFrame(frame, current_frame_processed_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Frame preprocessing failed");
        return ret;
    }
    
    // Initialize background if not calibrated
    if (!background_calibrated_) {
        background_frame_ = current_frame_processed_;
        previous_frame_ = current_frame_processed_;
        background_calibrated_ = true;
        ESP_LOGI(TAG, "Background initialized");
        return ESP_OK;
    }
    
    // Perform motion detection based on configured mode
    switch (config_.mode) {
        case DetectionMode::SIMPLE:
            ret = detectSimpleMotion(current_frame_processed_, previous_frame_);
            break;
        case DetectionMode::ADVANCED:
            ret = detectAdvancedMotion(current_frame_processed_, background_frame_);
            break;
        case DetectionMode::ADAPTIVE:
            ret = detectAdaptiveMotion(current_frame_processed_, background_frame_);
            break;
        default:
            ret = ESP_ERR_INVALID_ARG;
            break;
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Motion detection failed");
        return ret;
    }
    
    // Update temporal filtering and state
    applyTemporalFiltering();
    updateDetectionState(last_result_);
    
    // Update adaptive sensitivity if enabled
    if (config_.enable_adaptive_sensitivity) {
        updateAdaptiveSensitivity();
    }
    
    // Update background model slowly
    updateBackground(current_frame_processed_, 0.02f);
    
    // Store current frame as previous for next iteration
    previous_frame_ = current_frame_processed_;
    last_frame_timestamp_ = getCurrentTimestamp();
    
    // Update statistics
    uint64_t processing_time = getCurrentTimestamp() - process_start;
    updateStatistics(last_result_, processing_time);
    
    // Trigger motion callback if set
    if (motion_callback_) {
        motion_callback_(last_result_);
    }
    
    ESP_LOGD(TAG, "Motion detection: %.1f%% motion, %" PRIu32 " pixels, state=%d",
             last_result_.motion_percentage, last_result_.motion_pixels, (int)last_result_.state);
    
    return ESP_OK;
}

esp_err_t MotionDetector::preprocessFrame(camera_fb_t* frame, std::vector<uint8_t>& processed) {
    // Convert to grayscale
    std::vector<uint8_t> grayscale;
    convertToGrayscale(frame, grayscale);
    
    // Downscale if enabled
    if (config_.enable_downscaling && config_.downscale_factor > 1) {
        downscaleImage(grayscale, processed, config_.downscale_factor);
    } else {
        processed = grayscale;
    }
    
    // Apply noise filtering if enabled
    if (config_.enable_noise_filtering && config_.noise_filter_strength > 0) {
        applyNoiseFilter(processed, config_.noise_filter_strength);
    }
    
    return ESP_OK;
}

esp_err_t MotionDetector::detectAdvancedMotion(const std::vector<uint8_t>& current, const std::vector<uint8_t>& reference) {
    // Calculate frame difference
    for (size_t i = 0; i < current.size(); i++) {
        int diff = abs((int)current[i] - (int)reference[i]);
        
        // Apply noise threshold
        if (diff < config_.noise_threshold) {
            difference_frame_[i] = 0;
        } else {
            difference_frame_[i] = std::min(255, diff * 2);  // Amplify differences
        }
    }
    
    // Count motion pixels
    uint32_t motion_pixels = countMotionPixels(difference_frame_, current_sensitivity_);
    float motion_percentage = (float)motion_pixels * 100.0f / difference_frame_.size();
    
    // Analyze regions if enabled
    uint32_t active_regions = 0;
    if (config_.enable_region_analysis) {
        analyzeRegions(difference_frame_);
        
        // Count active regions
        for (bool region_active : region_motion_flags_) {
            if (region_active) {
                active_regions++;
            }
        }
    }
    
    // Calculate scene brightness and change factor
    float brightness = calculateBrightness(current);
    float scene_change = motion_percentage;
    
    // Update detection result
    last_result_.motion_percentage = motion_percentage;
    last_result_.motion_pixels = motion_pixels;
    last_result_.active_regions = active_regions;
    last_result_.average_scene_brightness = brightness;
    last_result_.scene_change_factor = scene_change;
    
    // Determine if motion threshold is met
    bool motion_detected = false;
    
    if (config_.enable_region_analysis) {
        // Region-based detection
        motion_detected = (active_regions >= config_.min_active_regions) &&
                         (motion_pixels >= config_.min_motion_pixels) &&
                         (motion_percentage >= config_.motion_percentage_threshold);
    } else {
        // Simple threshold detection
        motion_detected = (motion_pixels >= config_.min_motion_pixels) &&
                         (motion_percentage >= config_.motion_percentage_threshold);
    }
    
    // Update motion state
    if (motion_detected) {
        if (current_state_ != DetectionState::MOTION) {
            consecutive_motion_frames_++;
            consecutive_idle_frames_ = 0;
        }
    } else {
        consecutive_motion_frames_ = 0;
        consecutive_idle_frames_++;
    }
    
    return ESP_OK;
}

esp_err_t MotionDetector::detectSimpleMotion(const std::vector<uint8_t>& current, const std::vector<uint8_t>& reference) {
    // Simple frame difference
    uint32_t motion_pixels = 0;
    
    for (size_t i = 0; i < current.size(); i++) {
        int diff = abs((int)current[i] - (int)reference[i]);
        if (diff > current_sensitivity_) {
            motion_pixels++;
            difference_frame_[i] = 255;
        } else {
            difference_frame_[i] = 0;
        }
    }
    
    float motion_percentage = (float)motion_pixels * 100.0f / current.size();
    
    last_result_.motion_percentage = motion_percentage;
    last_result_.motion_pixels = motion_pixels;
    last_result_.active_regions = 0;
    last_result_.average_scene_brightness = calculateBrightness(current);
    last_result_.scene_change_factor = motion_percentage;
    
    return ESP_OK;
}

esp_err_t MotionDetector::detectAdaptiveMotion(const std::vector<uint8_t>& current, const std::vector<uint8_t>& reference) {
    // Start with advanced detection
    esp_err_t ret = detectAdvancedMotion(current, reference);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Adaptive threshold adjustment
    float environment_factor = calculateEnvironmentFactor();
    float adaptive_threshold = current_sensitivity_ * environment_factor;
    
    // Re-evaluate motion with adaptive threshold
    uint32_t adaptive_motion_pixels = countMotionPixels(difference_frame_, adaptive_threshold);
    float adaptive_motion_percentage = (float)adaptive_motion_pixels * 100.0f / difference_frame_.size();
    
    // Update result with adaptive values
    last_result_.motion_pixels = adaptive_motion_pixels;
    last_result_.motion_percentage = adaptive_motion_percentage;
    
    return ESP_OK;
}

void MotionDetector::convertToGrayscale(camera_fb_t* frame, std::vector<uint8_t>& gray) {
    gray.resize(frame->width * frame->height);
    
    if (frame->format == PIXFORMAT_GRAYSCALE) {
        // Already grayscale
        memcpy(gray.data(), frame->buf, gray.size());
    } else if (frame->format == PIXFORMAT_RGB565) {
        // Convert RGB565 to grayscale
        uint16_t* rgb565 = (uint16_t*)frame->buf;
        for (size_t i = 0; i < gray.size(); i++) {
            uint16_t pixel = rgb565[i];
            uint8_t r = (pixel >> 11) & 0x1F;
            uint8_t g = (pixel >> 5) & 0x3F;
            uint8_t b = pixel & 0x1F;
            
            // Convert to 8-bit and apply grayscale formula
            r = (r * 255) / 31;
            g = (g * 255) / 63;
            b = (b * 255) / 31;
            
            gray[i] = (uint8_t)(0.299f * r + 0.587f * g + 0.114f * b);
        }
    } else {
        // For other formats, use simple averaging
        // This is a fallback - specific format handling should be added
        for (size_t i = 0; i < gray.size(); i++) {
            gray[i] = frame->buf[i % frame->len];
        }
    }
}

void MotionDetector::applyNoiseFilter(std::vector<uint8_t>& image, uint32_t kernel_size) {
    if (kernel_size < 3 || kernel_size % 2 == 0) {
        return;  // Invalid kernel size
    }
    
    std::vector<uint8_t> filtered = image;
    int half_kernel = kernel_size / 2;
    
    for (int y = half_kernel; y < (int)processed_height_ - half_kernel; y++) {
        for (int x = half_kernel; x < (int)processed_width_ - half_kernel; x++) {
            int sum = 0;
            int count = 0;
            
            // Apply kernel
            for (int ky = -half_kernel; ky <= half_kernel; ky++) {
                for (int kx = -half_kernel; kx <= half_kernel; kx++) {
                    int px = x + kx;
                    int py = y + ky;
                    sum += image[py * processed_width_ + px];
                    count++;
                }
            }
            
            filtered[y * processed_width_ + x] = sum / count;
        }
    }
    
    image = filtered;
}

void MotionDetector::downscaleImage(const std::vector<uint8_t>& input, std::vector<uint8_t>& output, uint32_t factor) {
    uint32_t new_width = frame_width_ / factor;
    uint32_t new_height = frame_height_ / factor;
    
    output.resize(new_width * new_height);
    
    for (uint32_t y = 0; y < new_height; y++) {
        for (uint32_t x = 0; x < new_width; x++) {
            // Average pixels in the factor x factor region
            int sum = 0;
            int count = 0;
            
            for (uint32_t sy = y * factor; sy < (y + 1) * factor && sy < frame_height_; sy++) {
                for (uint32_t sx = x * factor; sx < (x + 1) * factor && sx < frame_width_; sx++) {
                    sum += input[sy * frame_width_ + sx];
                    count++;
                }
            }
            
            output[y * new_width + x] = (count > 0) ? (sum / count) : 0;
        }
    }
}

float MotionDetector::calculateBrightness(const std::vector<uint8_t>& image) {
    uint64_t sum = 0;
    for (uint8_t pixel : image) {
        sum += pixel;
    }
    return (float)sum / image.size();
}

void MotionDetector::updateBackground(const std::vector<uint8_t>& current, float learning_rate) {
    for (size_t i = 0; i < background_frame_.size(); i++) {
        float bg = background_frame_[i];
        float curr = current[i];
        background_frame_[i] = (uint8_t)(bg * (1.0f - learning_rate) + curr * learning_rate);
    }
}

esp_err_t MotionDetector::analyzeRegions(const std::vector<uint8_t>& diff_image) {
    if (!config_.enable_region_analysis) {
        return ESP_OK;
    }
    
    uint32_t region_width = processed_width_ / config_.detection_regions_x;
    uint32_t region_height = processed_height_ / config_.detection_regions_y;
    
    for (uint32_t ry = 0; ry < config_.detection_regions_y; ry++) {
        for (uint32_t rx = 0; rx < config_.detection_regions_x; rx++) {
            uint32_t region_idx = ry * config_.detection_regions_x + rx;
            
            uint32_t motion_pixels = 0;
            uint32_t total_pixels = 0;
            
            // Count motion pixels in this region
            for (uint32_t y = ry * region_height; y < (ry + 1) * region_height && y < processed_height_; y++) {
                for (uint32_t x = rx * region_width; x < (rx + 1) * region_width && x < processed_width_; x++) {
                    total_pixels++;
                    if (diff_image[y * processed_width_ + x] > current_sensitivity_) {
                        motion_pixels++;
                    }
                }
            }
            
            float region_motion_percentage = (float)motion_pixels * 100.0f / total_pixels;
            region_motion_levels_[region_idx] = region_motion_percentage;
            region_motion_flags_[region_idx] = (region_motion_percentage > config_.motion_percentage_threshold);
        }
    }
    
    return ESP_OK;
}

uint32_t MotionDetector::countMotionPixels(const std::vector<uint8_t>& diff_image, uint32_t threshold) {
    uint32_t count = 0;
    for (uint8_t pixel : diff_image) {
        if (pixel > threshold) {
            count++;
        }
    }
    return count;
}

void MotionDetector::applyTemporalFiltering() {
    uint64_t now = getCurrentTimestamp();
    
    // Calculate durations
    if (last_motion_timestamp_ > 0) {
        last_result_.no_motion_duration_ms = (now - last_motion_timestamp_) / 1000;
    }
    
    if (motion_start_timestamp_ > 0 && current_state_ == DetectionState::MOTION) {
        last_result_.motion_duration_ms = (now - motion_start_timestamp_) / 1000;
    }
    
    // Determine recording control flags
    last_result_.recording_should_start = false;
    last_result_.recording_should_stop = false;
    
    // Check if we should start recording (motion detected and persisted)
    if (shouldTriggerMotion(last_result_) && current_state_ != DetectionState::MOTION) {
        last_result_.recording_should_start = true;
        last_motion_timestamp_ = now;
        if (motion_start_timestamp_ == 0) {
            motion_start_timestamp_ = now;
        }
    }
    
    // Check if we should stop recording (no motion for configured timeout)
    if (shouldStopMotion() && current_state_ == DetectionState::MOTION) {
        last_result_.recording_should_stop = true;
        motion_start_timestamp_ = 0;
    }
    
    last_result_.last_motion_time = last_motion_timestamp_;
}

bool MotionDetector::shouldTriggerMotion(const DetectionResult& result) {
    // Motion must exceed thresholds and persist for debounce period
    bool motion_detected = (result.motion_percentage >= config_.motion_percentage_threshold) &&
                          (result.motion_pixels >= config_.min_motion_pixels);
    
    if (config_.enable_region_analysis) {
        motion_detected = motion_detected && (result.active_regions >= config_.min_active_regions);
    }
    
    // Apply debounce filtering
    if (motion_detected) {
        uint64_t now = getCurrentTimestamp();
        if (consecutive_motion_frames_ == 0) {
            motion_start_timestamp_ = now;
        }
        
        uint64_t motion_duration = now - motion_start_timestamp_;
        return motion_duration >= (config_.motion_debounce_ms * 1000);
    }
    
    return false;
}

bool MotionDetector::shouldStopMotion() {
    uint64_t now = getCurrentTimestamp();
    
    // Stop if no motion detected for timeout period
    if (last_motion_timestamp_ > 0) {
        uint64_t no_motion_duration = (now - last_motion_timestamp_) / 1000;  // Convert to ms
        return no_motion_duration >= config_.no_motion_timeout_ms;
    }
    
    return false;
}

void MotionDetector::updateDetectionState(const DetectionResult& result) {
    DetectionState new_state = current_state_;
    std::string state_message;
    
    uint64_t now = getCurrentTimestamp();
    
    switch (current_state_) {
        case DetectionState::IDLE:
            if (result.recording_should_start) {
                new_state = DetectionState::MOTION;
                state_message = "Motion detected - starting recording";
                last_motion_timestamp_ = now;
            }
            break;
            
        case DetectionState::MOTION:
            if (result.recording_should_stop) {
                new_state = DetectionState::COOLDOWN;
                state_message = "No motion timeout - stopping recording";
                state_change_timestamp_ = now;
            } else if (result.motion_percentage > 0) {
                // Update last motion time while in motion state
                last_motion_timestamp_ = now;
            }
            break;
            
        case DetectionState::COOLDOWN:
            // Stay in cooldown for configured period
            if ((now - state_change_timestamp_) >= (config_.cooldown_period_ms * 1000)) {
                new_state = DetectionState::IDLE;
                state_message = "Cooldown completed";
            } else if (result.recording_should_start) {
                // Motion detected during cooldown - return to motion state
                new_state = DetectionState::MOTION;
                state_message = "Motion detected during cooldown";
                last_motion_timestamp_ = now;
            }
            break;
    }
    
    if (new_state != current_state_) {
        setState(new_state, state_message);
    }
    
    last_result_.state = current_state_;
}

void MotionDetector::updateAdaptiveSensitivity() {
    uint64_t now = getCurrentTimestamp();
    
    // Add current motion level to recent samples
    recent_motion_levels_.push_back(last_result_.motion_percentage);
    if (recent_motion_levels_.size() > 100) {
        recent_motion_levels_.erase(recent_motion_levels_.begin());
    }
    
    // Adapt sensitivity periodically
    if ((now - last_adaptation_time_) >= (config_.adaptation_period_ms * 1000)) {
        float environment_factor = calculateEnvironmentFactor();
        adjustSensitivity(environment_factor);
        last_adaptation_time_ = now;
        
        if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
            statistics_.adaptive_adjustments++;
            statistics_.current_sensitivity = current_sensitivity_;
            xSemaphoreGive(stats_mutex_);
        }
    }
}

float MotionDetector::calculateEnvironmentFactor() {
    if (recent_motion_levels_.empty()) {
        return 1.0f;
    }
    
    // Calculate average motion level
    float avg_motion = 0.0f;
    for (float level : recent_motion_levels_) {
        avg_motion += level;
    }
    avg_motion /= recent_motion_levels_.size();
    
    // Environment factor based on motion history
    // High motion environment = lower sensitivity needed
    // Low motion environment = higher sensitivity needed
    if (avg_motion > config_.motion_percentage_threshold * 2) {
        return 0.8f;  // Reduce sensitivity in high motion environment
    } else if (avg_motion < config_.motion_percentage_threshold * 0.5f) {
        return 1.2f;  // Increase sensitivity in low motion environment
    }
    
    return 1.0f;  // No adjustment needed
}

void MotionDetector::adjustSensitivity(float environment_factor) {
    float new_sensitivity = current_sensitivity_ * environment_factor;
    
    // Apply adaptation factor for gradual changes
    current_sensitivity_ = current_sensitivity_ * (1.0f - config_.adaptation_factor) +
                          new_sensitivity * config_.adaptation_factor;
    
    // Clamp to valid range
    current_sensitivity_ = std::max(1.0f, std::min(100.0f, current_sensitivity_));
    
    ESP_LOGD(TAG, "Adaptive sensitivity updated: %.1f (factor: %.2f)",
             current_sensitivity_, environment_factor);
}

void MotionDetector::updateStatistics(const DetectionResult& result, uint64_t processing_time_us) {
    if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        statistics_.total_frames_processed++;
        
        if (result.state == DetectionState::MOTION) {
            statistics_.motion_events_detected++;
            statistics_.total_motion_time_ms += 33;  // Approximate frame time
        } else {
            statistics_.total_idle_time_ms += 33;
        }
        
        // Update averages
        float alpha = 0.1f;  // Smoothing factor
        statistics_.avg_motion_percentage = 
            statistics_.avg_motion_percentage * (1.0f - alpha) + result.motion_percentage * alpha;
        
        double processing_time_ms = processing_time_us / 1000.0;
        statistics_.avg_processing_time_ms = 
            statistics_.avg_processing_time_ms * (1.0 - alpha) + processing_time_ms * alpha;
        
        statistics_.current_sensitivity = current_sensitivity_;
        
        xSemaphoreGive(stats_mutex_);
    }
}

void MotionDetector::setState(DetectionState state, const std::string& message) {
    current_state_ = state;
    ESP_LOGI(TAG, "State changed to %d: %s", (int)state, message.c_str());
    
    if (state_callback_) {
        state_callback_(state, message);
    }
}

uint64_t MotionDetector::getCurrentTimestamp() {
    return esp_timer_get_time();
}

MotionDetector::DetectionResult MotionDetector::getLastResult() const {
    return last_result_;
}

MotionDetector::Statistics MotionDetector::getStatistics() const {
    Statistics stats = {};
    
    if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        stats = statistics_;
        xSemaphoreGive(stats_mutex_);
    }
    
    return stats;
}

void MotionDetector::resetStatistics() {
    if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        statistics_ = {};
        statistics_.current_sensitivity = current_sensitivity_;
        xSemaphoreGive(stats_mutex_);
    }
}

esp_err_t MotionDetector::setSensitivity(uint32_t sensitivity) {
    if (sensitivity > 100) {
        return ESP_ERR_INVALID_ARG;
    }
    
    config_.sensitivity = sensitivity;
    current_sensitivity_ = sensitivity;
    
    ESP_LOGI(TAG, "Sensitivity updated to: %" PRIu32 "", sensitivity);
    return ESP_OK;
}

esp_err_t MotionDetector::setMotionTimeout(uint32_t timeout_ms) {
    config_.no_motion_timeout_ms = timeout_ms;
    ESP_LOGI(TAG, "Motion timeout updated to: %" PRIu32 " ms", timeout_ms);
    return ESP_OK;
}

void MotionDetector::printDetectionStatus() const {
    ESP_LOGI(TAG, "=== Motion Detection Status ===");
    ESP_LOGI(TAG, "State: %d", (int)current_state_);
    ESP_LOGI(TAG, "Motion: %.1f%% (%" PRIu32 " pixels)", 
             last_result_.motion_percentage, last_result_.motion_pixels);
    ESP_LOGI(TAG, "Active regions: %" PRIu32 "/%" PRIu32 "", 
             last_result_.active_regions, (config_.detection_regions_x * config_.detection_regions_y));
    ESP_LOGI(TAG, "Current sensitivity: %.1f", current_sensitivity_);
    ESP_LOGI(TAG, "No motion duration: %llu ms", last_result_.no_motion_duration_ms);
    ESP_LOGI(TAG, "Motion duration: %llu ms", last_result_.motion_duration_ms);
    ESP_LOGI(TAG, "Recording should start: %s", last_result_.recording_should_start ? "YES" : "NO");
    ESP_LOGI(TAG, "Recording should stop: %s", last_result_.recording_should_stop ? "YES" : "NO");
    
    Statistics stats = getStatistics();
    ESP_LOGI(TAG, "Total frames: %llu", stats.total_frames_processed);
    ESP_LOGI(TAG, "Motion events: %llu", stats.motion_events_detected);
    ESP_LOGI(TAG, "Avg processing time: %.2f ms", stats.avg_processing_time_ms);
    ESP_LOGI(TAG, "==============================");
}