#include "video_recorder.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <algorithm>

const char* VideoRecorder::TAG = "VideoRecorder";

VideoRecorder::VideoRecorder()
    : encoder_(std::make_unique<H264Encoder>())
    , muxer_(std::make_unique<TSMuxer>())
    , motion_detector_(std::make_unique<MotionDetector>())
    , cloud_uploader_(std::make_unique<CloudUploader>())
    , task_manager_(TaskManager::getInstance())
    , mqtt_client_(nullptr)
    , current_state_(RecorderState::UNINITIALIZED)
    , frame_queue_(nullptr)
    , state_mutex_(nullptr)
    , stats_mutex_(nullptr)
    , maintenance_timer_(nullptr)
    , recording_task_handle_(nullptr)
    , motion_task_handle_(nullptr)
    , schedule_task_handle_(nullptr)
    , tasks_running_(false)
    , recording_start_time_(0)
    , last_frame_time_(0)
    , frame_counter_(0)
    , last_file_scan_time_(0)
    , motion_recording_active_(false)
    , last_motion_recording_start_(0)
    , last_motion_recording_stop_(0)
{
    // Initialize mutexes
    state_mutex_ = xSemaphoreCreateMutex();
    stats_mutex_ = xSemaphoreCreateMutex();
    
    if (!state_mutex_ || !stats_mutex_) {
        ESP_LOGE(TAG, "Failed to create mutexes");
    }
    
    // Initialize statistics and current recording info
    memset(&statistics_, 0, sizeof(statistics_));
    memset(&current_recording_, 0, sizeof(current_recording_));
    
    // Set up component callbacks
    encoder_->setStateCallback(
        std::bind(&VideoRecorder::onEncoderStateChanged, this,
                 std::placeholders::_1, std::placeholders::_2)
    );
    
    encoder_->setFrameCallback(
        std::bind(&VideoRecorder::onEncodedFrame, this,
                 std::placeholders::_1)
    );
    
    muxer_->setStateCallback(
        std::bind(&VideoRecorder::onMuxerStateChanged, this,
                 std::placeholders::_1, std::placeholders::_2)
    );
    
    muxer_->setFileCallback(
        std::bind(&VideoRecorder::onFileCompleted, this,
                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
    );
    
    // Set up motion detector callback
    motion_detector_->setMotionCallback(
        std::bind(&VideoRecorder::onMotionDetected, this,
                 std::placeholders::_1)
    );
    
    // Set up cloud uploader callbacks
    cloud_uploader_->setProgressCallback(
        std::bind(&VideoRecorder::onUploadProgress, this,
                 std::placeholders::_1)
    );
    
    cloud_uploader_->setCompletionCallback(
        std::bind(&VideoRecorder::onUploadCompleted, this,
                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
    );
}

VideoRecorder::~VideoRecorder() {
    deinitialize();
    
    if (state_mutex_) {
        vSemaphoreDelete(state_mutex_);
    }
    if (stats_mutex_) {
        vSemaphoreDelete(stats_mutex_);
    }
    
    // Motion detector cleanup is handled by unique_ptr
}

esp_err_t VideoRecorder::initialize(const RecorderConfig& config) {
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_FAIL;
    }
    
    if (current_state_ != RecorderState::UNINITIALIZED) {
        ESP_LOGW(TAG, "Recorder already initialized");
        xSemaphoreGive(state_mutex_);
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing video recorder");
    config_ = config;
    
    // Create output directory
    esp_err_t ret = createOutputDirectory();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create output directory");
        xSemaphoreGive(state_mutex_);
        return ret;
    }
    
    // Validate storage space
    ret = validateStorageSpace();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Insufficient storage space");
        xSemaphoreGive(state_mutex_);
        return ret;
    }
    
    // Setup quality settings
    ret = setupQualitySettings();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup quality settings");
        xSemaphoreGive(state_mutex_);
        return ret;
    }
    
    // Initialize encoder
    H264Encoder::EncoderConfig encoder_config;
    encoder_config.width = (config_.quality_preset == QualityPreset::CUSTOM) ? 
                          config_.custom_width : 1920;
    encoder_config.height = (config_.quality_preset == QualityPreset::CUSTOM) ? 
                           config_.custom_height : 1080;
    encoder_config.fps = (config_.quality_preset == QualityPreset::CUSTOM) ? 
                        config_.custom_fps : 30;
    encoder_config.bitrate = (config_.quality_preset == QualityPreset::CUSTOM) ? 
                            config_.custom_bitrate : 4000000;
    encoder_config.input_buffer_count = config_.frame_buffer_count;
    
    ret = encoder_->initialize(encoder_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize encoder");
        xSemaphoreGive(state_mutex_);
        return ret;
    }
    
    // Initialize muxer
    TSMuxer::TSConfig muxer_config;
    muxer_config.output_path = config_.output_directory;
    muxer_config.max_file_size_mb = config_.max_file_size_mb;
    muxer_config.max_duration_sec = config_.max_file_duration_min * 60;
    muxer_config.enable_file_rotation = config_.enable_auto_rotation;
    
    ret = muxer_->initialize(muxer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize muxer");
        encoder_->deinitialize();
        xSemaphoreGive(state_mutex_);
        return ret;
    }
    
    // Initialize motion detector if motion-based recording is enabled
    if (config_.enable_motion_detection) {
        MotionDetector::DetectionConfig motion_config;
        motion_config.mode = config_.motion_mode;
        motion_config.sensitivity = config_.motion_threshold;
        motion_config.motion_debounce_ms = config_.motion_debounce_ms;
        motion_config.no_motion_timeout_ms = config_.no_motion_timeout_ms;
        motion_config.enable_adaptive_sensitivity = config_.enable_adaptive_motion;
        motion_config.motion_percentage_threshold = 2.0f;
        motion_config.min_motion_pixels = 100;
        motion_config.enable_region_analysis = true;
        motion_config.detection_regions_x = 8;
        motion_config.detection_regions_y = 6;
        motion_config.min_active_regions = 2;
        
        ret = motion_detector_->initialize(motion_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize motion detector");
            encoder_->deinitialize();
            muxer_->deinitialize();
            xSemaphoreGive(state_mutex_);
            return ret;
        }
        
        ret = motion_detector_->start();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start motion detector");
            motion_detector_->deinitialize();
            encoder_->deinitialize();
            muxer_->deinitialize();
            xSemaphoreGive(state_mutex_);
            return ret;
        }
        
        ESP_LOGI(TAG, "Motion detector initialized with sensitivity %d, timeout %d ms",
                 config_.motion_threshold, config_.no_motion_timeout_ms);
    }
    
    // Create frame queue
    frame_queue_ = xQueueCreate(FRAME_QUEUE_SIZE, sizeof(camera_fb_t*));
    if (!frame_queue_) {
        ESP_LOGE(TAG, "Failed to create frame queue");
        encoder_->deinitialize();
        muxer_->deinitialize();
        xSemaphoreGive(state_mutex_);
        return ESP_FAIL;
    }
    
    // Create maintenance timer
    maintenance_timer_ = xTimerCreate(
        "maintenance_timer",
        pdMS_TO_TICKS(MAINTENANCE_INTERVAL_MS),
        pdTRUE,  // Auto-reload
        this,
        maintenanceTimerCallback
    );
    
    if (!maintenance_timer_) {
        ESP_LOGE(TAG, "Failed to create maintenance timer");
        vQueueDelete(frame_queue_);
        encoder_->deinitialize();
        muxer_->deinitialize();
        xSemaphoreGive(state_mutex_);
        return ESP_FAIL;
    }
    
    // Create tasks
    ret = createTasks();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create tasks");
        xTimerDelete(maintenance_timer_, 0);
        vQueueDelete(frame_queue_);
        encoder_->deinitialize();
        muxer_->deinitialize();
        xSemaphoreGive(state_mutex_);
        return ret;
    }
    
    // Start maintenance timer
    xTimerStart(maintenance_timer_, 0);
    
    setState(RecorderState::INITIALIZED, "Video recorder initialized");
    resetStatistics();
    
    ESP_LOGI(TAG, "Video recorder initialized: %dx%d@%dfps, %s mode",
             encoder_config.width, encoder_config.height, encoder_config.fps,
             (config_.mode == RecordingMode::CONTINUOUS) ? "continuous" :
             (config_.mode == RecordingMode::SCHEDULED) ? "scheduled" :
             (config_.mode == RecordingMode::MOTION_BASED) ? "motion-based" : "manual");
    
    xSemaphoreGive(state_mutex_);
    return ESP_OK;
}

esp_err_t VideoRecorder::deinitialize() {
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_FAIL;
    }
    
    if (current_state_ == RecorderState::UNINITIALIZED) {
        xSemaphoreGive(state_mutex_);
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Deinitializing video recorder");
    
    // Stop and deinitialize motion detector
    if (motion_detector_) {
        motion_detector_->stop();
        motion_detector_->deinitialize();
    }
    
    // Stop and deinitialize cloud uploader
    if (cloud_uploader_) {
        cloud_uploader_->stop();
        cloud_uploader_->deinitialize();
    }
    
    // Stop recording if active
    if (current_state_ == RecorderState::RECORDING) {
        stopRecording();
    }
    
    // Stop maintenance timer
    if (maintenance_timer_) {
        xTimerStop(maintenance_timer_, 0);
        xTimerDelete(maintenance_timer_, 0);
        maintenance_timer_ = nullptr;
    }
    
    // Delete tasks
    deleteTasks();
    
    // Clean up components
    if (encoder_) {
        encoder_->deinitialize();
    }
    if (muxer_) {
        muxer_->deinitialize();
    }
    
    // Clean up queue
    if (frame_queue_) {
        vQueueDelete(frame_queue_);
        frame_queue_ = nullptr;
    }
    
    setState(RecorderState::UNINITIALIZED, "Video recorder deinitialized");
    
    xSemaphoreGive(state_mutex_);
    return ESP_OK;
}

esp_err_t VideoRecorder::startRecording(const std::string& filename) {
    if (current_state_ != RecorderState::INITIALIZED) {
        ESP_LOGE(TAG, "Recorder not initialized");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Starting video recording");
    
    // Start encoder
    esp_err_t ret = encoder_->start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start encoder");
        return ret;
    }
    
    // Start muxer
    ret = muxer_->startRecording(filename);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start muxer");
        encoder_->stop();
        return ret;
    }
    
    // Initialize recording info
    recording_start_time_ = getCurrentTimestamp();
    current_recording_.start_timestamp_us = recording_start_time_;
    current_recording_.filename = muxer_->getCurrentFilename();
    current_recording_.is_complete = false;
    
    frame_counter_ = 0;
    last_frame_time_ = recording_start_time_;
    
    setState(RecorderState::RECORDING, "Recording started");
    
    return ESP_OK;
}

esp_err_t VideoRecorder::stopRecording() {
    if (current_state_ != RecorderState::RECORDING && current_state_ != RecorderState::PAUSED) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Stopping video recording");
    
    // Stop encoder
    encoder_->stop();
    
    // Stop muxer
    muxer_->stopRecording();
    
    // Finalize recording info
    current_recording_.end_timestamp_us = getCurrentTimestamp();
    current_recording_.duration_ms = (current_recording_.end_timestamp_us - current_recording_.start_timestamp_us) / 1000;
    current_recording_.frame_count = frame_counter_;
    current_recording_.is_complete = true;
    
    // Update statistics
    if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        statistics_.total_recordings++;
        statistics_.total_recording_time_ms += current_recording_.duration_ms;
        statistics_.total_frames_recorded += current_recording_.frame_count;
        xSemaphoreGive(stats_mutex_);
    }
    
    setState(RecorderState::INITIALIZED, "Recording stopped");
    
    // Notify file completion
    if (file_callback_) {
        file_callback_(current_recording_);
    }
    
    return ESP_OK;
}

esp_err_t VideoRecorder::pauseRecording() {
    if (current_state_ != RecorderState::RECORDING) {
        ESP_LOGE(TAG, "Not currently recording");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Pausing video recording");
    setState(RecorderState::PAUSED, "Recording paused");
    
    return ESP_OK;
}

esp_err_t VideoRecorder::resumeRecording() {
    if (current_state_ != RecorderState::PAUSED) {
        ESP_LOGE(TAG, "Not currently paused");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Resuming video recording");
    setState(RecorderState::RECORDING, "Recording resumed");
    
    return ESP_OK;
}

esp_err_t VideoRecorder::recordFrame(camera_fb_t* frame) {
    if (!frame || !frame->buf || frame->len == 0) {
        ESP_LOGE(TAG, "Invalid frame");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check if we should be recording
    if (!shouldRecord()) {
        return ESP_OK;  // Not an error, just not recording
    }
    
    // Add frame to processing queue
    if (xQueueSend(frame_queue_, &frame, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Frame queue full, dropping frame");
        if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
            statistics_.dropped_frames++;
            xSemaphoreGive(stats_mutex_);
        }
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t VideoRecorder::forceKeyFrame() {
    if (current_state_ != RecorderState::RECORDING) {
        return ESP_FAIL;
    }
    
    return encoder_->forceKeyFrame();
}

esp_err_t VideoRecorder::processFrame(camera_fb_t* frame) {
    if (current_state_ == RecorderState::PAUSED) {
        return ESP_OK;  // Skip processing when paused
    }
    
    // Motion detection if enabled
    if (config_.enable_motion_detection && config_.mode == RecordingMode::MOTION_BASED) {
        esp_err_t ret = detectMotion(frame);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Motion detection failed");
        }
    }
    
    // Encode frame
    esp_err_t ret = encoder_->encodeFrame(frame);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to encode frame");
        if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
            statistics_.encoding_errors++;
            xSemaphoreGive(stats_mutex_);
        }
        return ret;
    }
    
    // Update frame statistics
    frame_counter_++;
    updateStatistics(true);
    
    return ESP_OK;
}

bool VideoRecorder::shouldRecord() const {
    if (current_state_ != RecorderState::RECORDING) {
        return false;
    }
    
    switch (config_.mode) {
        case RecordingMode::CONTINUOUS:
            return true;
            
        case RecordingMode::SCHEDULED:
            return isScheduledTime();
            
        case RecordingMode::MOTION_BASED:
            return isMotionDetected();
            
        case RecordingMode::MANUAL:
        default:
            return true;  // Manual mode records when explicitly started
    }
}

bool VideoRecorder::isScheduledTime() const {
    if (config_.schedule.empty()) {
        return false;
    }
    
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    
    for (const auto& entry : config_.schedule) {
        if (!entry.enabled) {
            continue;
        }
        
        // Check day of week (0 = Sunday)
        uint8_t day_bit = 1 << tm.tm_wday;
        if (!(entry.days_of_week & day_bit)) {
            continue;
        }
        
        // Check time range
        uint32_t current_minutes = tm.tm_hour * 60 + tm.tm_min;
        uint32_t start_minutes = entry.start_hour * 60 + entry.start_minute;
        uint32_t end_minutes = entry.end_hour * 60 + entry.end_minute;
        
        if (start_minutes <= end_minutes) {
            // Same day range
            if (current_minutes >= start_minutes && current_minutes <= end_minutes) {
                return true;
            }
        } else {
            // Overnight range
            if (current_minutes >= start_minutes || current_minutes <= end_minutes) {
                return true;
            }
        }
    }
    
    return false;
}

esp_err_t VideoRecorder::detectMotion(camera_fb_t* frame) {
    // Use the advanced motion detector if enabled
    if (config_.enable_motion_detection && motion_detector_) {
        return motion_detector_->processFrame(frame);
    }
    
    return ESP_OK;
}

void VideoRecorder::recordingTask(void* parameter) {
    VideoRecorder* self = static_cast<VideoRecorder*>(parameter);
    ESP_LOGI(TAG, "Recording task started");
    
    camera_fb_t* frame;
    
    while (self->tasks_running_) {
        // Wait for frame
        if (xQueueReceive(self->frame_queue_, &frame, pdMS_TO_TICKS(100)) == pdTRUE) {
            esp_err_t ret = self->processFrame(frame);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to process frame");
            }
        }
    }
    
    ESP_LOGI(TAG, "Recording task ended");
    vTaskDelete(nullptr);
}

void VideoRecorder::onEncoderStateChanged(H264Encoder::EncoderState state, const std::string& message) {
    ESP_LOGI(TAG, "Encoder state changed: %d - %s", (int)state, message.c_str());
    
    if (state == H264Encoder::EncoderState::ERROR) {
        logError("Encoder error: " + message);
        setState(RecorderState::ERROR, "Encoder error");
    }
}

void VideoRecorder::onEncodedFrame(const H264Encoder::FrameInfo& frame) {
    // Pass encoded frame to muxer
    esp_err_t ret = muxer_->writeFrame(frame);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write frame to muxer");
        if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
            statistics_.file_errors++;
            xSemaphoreGive(stats_mutex_);
        }
    }
}

void VideoRecorder::onMuxerStateChanged(TSMuxer::MuxerState state, const std::string& message) {
    ESP_LOGI(TAG, "Muxer state changed: %d - %s", (int)state, message.c_str());
    
    if (state == TSMuxer::MuxerState::ERROR) {
        logError("Muxer error: " + message);
        setState(RecorderState::ERROR, "Muxer error");
    }
}

void VideoRecorder::onFileCompleted(const std::string& filename, uint64_t size_bytes, uint64_t duration_ms) {
    ESP_LOGI(TAG, "File completed: %s (%s, %s)", 
             filename.c_str(), 
             formatFileSize(size_bytes).c_str(),
             formatDuration(duration_ms).c_str());
    
    // Update file list cache
    recording_files_.clear();
    last_file_scan_time_ = 0;  // Force rescan
    
    // Update statistics
    if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        statistics_.total_bytes_recorded += size_bytes;
        xSemaphoreGive(stats_mutex_);
    }
}

void VideoRecorder::onMotionDetected(const MotionDetector::DetectionResult& result) {
    // Handle motion detection events for intelligent recording control
    uint64_t now = getCurrentTimestamp();
    
    if (config_.mode != RecordingMode::MOTION_BASED) {
        return;  // Only handle motion events in motion-based mode
    }
    
    // Update statistics
    if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (result.recording_should_start && !motion_recording_active_) {
            statistics_.motion_events++;
        }
        xSemaphoreGive(stats_mutex_);
    }
    
    // Handle automatic recording start
    if (result.recording_should_start && !motion_recording_active_) {
        ESP_LOGI(TAG, "Motion detected - auto-starting recording (%.1f%% motion, %d pixels)",
                 result.motion_percentage, result.motion_pixels);
        
        // Start recording automatically
        if (current_state_ == RecorderState::INITIALIZED) {
            std::string auto_filename = generateFilename();
            esp_err_t ret = startRecording(auto_filename);
            if (ret == ESP_OK) {
                motion_recording_active_ = true;
                last_motion_recording_start_ = now;
                ESP_LOGI(TAG, "Auto-started motion recording: %s", auto_filename.c_str());
                
                // Notify via state callback with motion context
                if (state_callback_) {
                    std::string motion_info = "Motion-based recording started (" + 
                                            std::to_string(result.motion_percentage) + "% motion, " +
                                            std::to_string(result.motion_pixels) + " pixels, " +
                                            std::to_string(result.active_regions) + " regions)";
                    state_callback_(RecorderState::RECORDING, motion_info);
                }
            } else {
                ESP_LOGE(TAG, "Failed to auto-start motion recording");
            }
        }
    }
    
    // Handle automatic recording stop
    if (result.recording_should_stop && motion_recording_active_) {
        ESP_LOGI(TAG, "No motion timeout - auto-stopping recording (%.1f s no motion)",
                 result.no_motion_duration_ms / 1000.0f);
        
        // Stop recording automatically
        if (current_state_ == RecorderState::RECORDING) {
            esp_err_t ret = stopRecording();
            if (ret == ESP_OK) {
                motion_recording_active_ = false;
                last_motion_recording_stop_ = now;
                float recording_duration = (now - last_motion_recording_start_) / 1000000.0f;
                ESP_LOGI(TAG, "Auto-stopped motion recording after %.1f seconds", recording_duration);
                
                // Notify via state callback with motion context
                if (state_callback_) {
                    std::string motion_info = "Motion-based recording stopped (" + 
                                            std::to_string(result.no_motion_duration_ms / 1000.0f) + "s no motion, " +
                                            std::to_string(recording_duration) + "s total duration)";
                    state_callback_(RecorderState::INITIALIZED, motion_info);
                }
            } else {
                ESP_LOGE(TAG, "Failed to auto-stop motion recording");
            }
        }
    }
    
    // Log detailed motion status periodically
    static uint64_t last_status_log = 0;
    if ((now - last_status_log) > 5000000) {  // Every 5 seconds
        ESP_LOGI(TAG, "Motion status: %.1f%% motion, %d regions active, state=%d, recording=%s",
                 result.motion_percentage, result.active_regions, (int)result.state,
                 motion_recording_active_ ? "active" : "inactive");
        last_status_log = now;
    }
}

esp_err_t VideoRecorder::createTasks() {
    tasks_running_ = true;
    
    // Create recording task
    BaseType_t result = xTaskCreate(
        recordingTask,
        "video_recorder",
        RECORDING_TASK_STACK_SIZE,
        this,
        RECORDING_TASK_PRIORITY,
        &recording_task_handle_
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create recording task");
        tasks_running_ = false;
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Video recorder tasks created");
    return ESP_OK;
}

void VideoRecorder::deleteTasks() {
    tasks_running_ = false;
    
    if (recording_task_handle_) {
        vTaskDelay(pdMS_TO_TICKS(100));  // Allow task to finish
        vTaskDelete(recording_task_handle_);
        recording_task_handle_ = nullptr;
    }
    
    ESP_LOGI(TAG, "Video recorder tasks deleted");
}

void VideoRecorder::maintenanceTimerCallback(TimerHandle_t timer) {
    VideoRecorder* self = static_cast<VideoRecorder*>(pvTimerGetTimerID(timer));
    if (self) {
        self->cleanupOldFiles();
        self->performHealthCheck();
    }
}

esp_err_t VideoRecorder::setupQualitySettings() {
    // Quality presets are handled in the initialize method
    // This method can be extended for more complex quality setup
    return ESP_OK;
}

esp_err_t VideoRecorder::createOutputDirectory() {
    struct stat st;
    if (stat(config_.output_directory.c_str(), &st) == 0) {
        if (S_ISDIR(st.st_mode)) {
            return ESP_OK;  // Directory exists
        } else {
            ESP_LOGE(TAG, "Output path exists but is not a directory: %s", config_.output_directory.c_str());
            return ESP_FAIL;
        }
    }
    
    // Create directory
    if (mkdir(config_.output_directory.c_str(), 0755) == 0) {
        ESP_LOGI(TAG, "Created output directory: %s", config_.output_directory.c_str());
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to create output directory: %s", config_.output_directory.c_str());
    return ESP_FAIL;
}

esp_err_t VideoRecorder::validateStorageSpace() {
    struct statvfs vfs;
    if (statvfs(config_.output_directory.c_str(), &vfs) != 0) {
        ESP_LOGE(TAG, "Failed to check storage space");
        return ESP_FAIL;
    }
    
    uint64_t available_bytes = vfs.f_bavail * vfs.f_frsize;
    uint64_t required_bytes = config_.max_file_size_mb * 1024 * 1024;
    
    if (available_bytes < required_bytes) {
        ESP_LOGE(TAG, "Insufficient storage: %llu MB available, %llu MB required",
                 available_bytes / (1024 * 1024), required_bytes / (1024 * 1024));
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Storage validation passed: %llu MB available", available_bytes / (1024 * 1024));
    return ESP_OK;
}

void VideoRecorder::setState(RecorderState state, const std::string& message) {
    current_state_ = state;
    ESP_LOGI(TAG, "State changed to %d: %s", (int)state, message.c_str());
    
    if (state_callback_) {
        state_callback_(state, message);
    }
}

void VideoRecorder::updateStatistics(bool frame_processed) {
    if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        uint64_t now = getCurrentTimestamp();
        
        if (frame_processed) {
            statistics_.current_frames++;
            
            // Update FPS calculation
            if (last_frame_time_ > 0) {
                uint64_t frame_interval = now - last_frame_time_;
                if (frame_interval > 0) {
                    double fps = 1000000.0 / frame_interval;  // Convert to FPS
                    statistics_.current_fps = statistics_.current_fps * 0.9 + fps * 0.1;  // Smoothed FPS
                }
            }
            last_frame_time_ = now;
        }
        
        // Update current recording duration
        if (recording_start_time_ > 0) {
            statistics_.current_duration_ms = (now - recording_start_time_) / 1000;
        }
        
        // Update current file size from muxer
        statistics_.current_file_size = muxer_->getCurrentFileSize();
        
        // Calculate average FPS and bitrate
        if (statistics_.total_recording_time_ms > 0) {
            statistics_.avg_fps = (double)statistics_.total_frames_recorded * 1000.0 / statistics_.total_recording_time_ms;
            statistics_.avg_bitrate_mbps = (double)statistics_.total_bytes_recorded * 8.0 / 
                                         (statistics_.total_recording_time_ms * 1000.0);
        }
        
        xSemaphoreGive(stats_mutex_);
    }
}

uint64_t VideoRecorder::getCurrentTimestamp() {
    return esp_timer_get_time();
}

std::string VideoRecorder::formatDuration(uint64_t duration_ms) const {
    uint64_t seconds = duration_ms / 1000;
    uint64_t minutes = seconds / 60;
    uint64_t hours = minutes / 60;
    
    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(2) << hours << ":"
        << std::setfill('0') << std::setw(2) << (minutes % 60) << ":"
        << std::setfill('0') << std::setw(2) << (seconds % 60);
    
    return oss.str();
}

std::string VideoRecorder::formatFileSize(uint64_t size_bytes) const {
    if (size_bytes < 1024) {
        return std::to_string(size_bytes) + " B";
    } else if (size_bytes < 1024 * 1024) {
        return std::to_string(size_bytes / 1024) + " KB";
    } else if (size_bytes < 1024 * 1024 * 1024) {
        return std::to_string(size_bytes / (1024 * 1024)) + " MB";
    } else {
        return std::to_string(size_bytes / (1024 * 1024 * 1024)) + " GB";
    }
}

void VideoRecorder::logError(const std::string& error) {
    ESP_LOGE(TAG, "%s", error.c_str());
    
    if (error_callback_) {
        error_callback_(error);
    }
}

VideoRecorder::Statistics VideoRecorder::getStatistics() const {
    Statistics stats = {};
    
    if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        stats = statistics_;
        xSemaphoreGive(stats_mutex_);
    }
    
    return stats;
}

void VideoRecorder::resetStatistics() {
    if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(&statistics_, 0, sizeof(statistics_));
        xSemaphoreGive(stats_mutex_);
    }
}

esp_err_t VideoRecorder::cleanupOldFiles() {
    // Implementation for cleaning up old files based on max_total_files
    // This would scan the directory and delete oldest files if limit exceeded
    ESP_LOGD(TAG, "Cleanup old files (not yet implemented)");
    return ESP_OK;
}

esp_err_t VideoRecorder::performHealthCheck() {
    // Check encoder health
    if (encoder_ && encoder_->getState() == H264Encoder::EncoderState::ERROR) {
        ESP_LOGE(TAG, "Health check: Encoder in error state");
        return ESP_FAIL;
    }
    
    // Check muxer health
    if (muxer_ && muxer_->getState() == TSMuxer::MuxerState::ERROR) {
        ESP_LOGE(TAG, "Health check: Muxer in error state");
        return ESP_FAIL;
    }
    
    // Check storage space
    esp_err_t ret = validateStorageSpace();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Health check: Low storage space");
    }
    
    return ESP_OK;
}

bool VideoRecorder::isMotionDetected() const {
    if (config_.enable_motion_detection && motion_detector_) {
        return motion_detector_->isMotionDetected();
    }
    return false;
}

esp_err_t VideoRecorder::enableMotionDetection(bool enable) {
    config_.enable_motion_detection = enable;
    
    if (!motion_detector_) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (enable) {
        esp_err_t ret = motion_detector_->start();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Motion detection enabled");
        }
        return ret;
    } else {
        esp_err_t ret = motion_detector_->stop();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Motion detection disabled");
        }
        return ret;
    }
}

esp_err_t VideoRecorder::setMotionThreshold(uint32_t threshold) {
    if (threshold > 100) {
        return ESP_ERR_INVALID_ARG;
    }
    
    config_.motion_threshold = threshold;
    
    if (motion_detector_) {
        return motion_detector_->setSensitivity(threshold);
    }
    
    return ESP_OK;
}

esp_err_t VideoRecorder::setMotionTimeout(uint32_t timeout_ms) {
    config_.no_motion_timeout_ms = timeout_ms;
    
    if (motion_detector_) {
        return motion_detector_->setMotionTimeout(timeout_ms);
    }
    
    return ESP_OK;
}

esp_err_t VideoRecorder::setMotionMode(MotionDetector::DetectionMode mode) {
    config_.motion_mode = mode;
    
    // If motion detector is initialized, we would need to reinitialize it
    // For now, just update the config - the mode will apply on next initialization
    ESP_LOGI(TAG, "Motion detection mode set to %d (effective on next initialization)", (int)mode);
    
    return ESP_OK;
}

MotionDetector::DetectionResult VideoRecorder::getMotionStatus() const {
    if (motion_detector_) {
        return motion_detector_->getLastResult();
    }
    
    MotionDetector::DetectionResult empty_result = {};
    return empty_result;
}

void VideoRecorder::onUploadProgress(const CloudUploader::UploadProgress& progress) {
    ESP_LOGD(TAG, "Upload progress: %s - %.1f%% (%llu/%llu bytes, %d KB/s)",
             progress.file_path.c_str(), progress.progress_percentage,
             progress.bytes_uploaded, progress.total_bytes, progress.upload_speed_kbps);
}

void VideoRecorder::onUploadCompleted(const std::string& file_path, bool success, const std::string& cloud_url) {
    if (success) {
        ESP_LOGI(TAG, "Cloud upload completed successfully: %s -> %s", file_path.c_str(), cloud_url.c_str());
    } else {
        ESP_LOGE(TAG, "Cloud upload failed: %s", file_path.c_str());
    }
}

VideoRecorder::MotionRecordingStatus VideoRecorder::getMotionRecordingStatus() const {
    MotionRecordingStatus status;
    
    // Basic recording info
    status.motion_recording_enabled = config_.enable_motion_detection;
    status.motion_recording_active = motion_recording_active_;
    status.motion_sensitivity = config_.motion_threshold;
    status.motion_timeout_ms = config_.no_motion_timeout_ms;
    
    // Motion mode string
    switch (config_.motion_mode) {
        case MotionDetector::DetectionMode::SIMPLE:
            status.motion_mode = "simple";
            break;
        case MotionDetector::DetectionMode::ADVANCED:
            status.motion_mode = "advanced";
            break;
        case MotionDetector::DetectionMode::ADAPTIVE:
            status.motion_mode = "adaptive";
            break;
        default:
            status.motion_mode = "unknown";
            break;
    }
    
    // Current recording filename
    if (current_state_ == RecorderState::RECORDING) {
        status.current_filename = current_recording_.filename;
        
        // Calculate recording duration
        uint64_t now = getCurrentTimestamp();
        if (recording_start_time_ > 0) {
            status.recording_duration_ms = (now - recording_start_time_) / 1000;
        }
    }
    
    // Motion detection details
    if (motion_detector_ && config_.enable_motion_detection) {
        MotionDetector::DetectionResult motion_result = motion_detector_->getLastResult();
        status.motion_currently_detected = motion_detector_->isMotionDetected();
        status.current_motion_percentage = motion_result.motion_percentage;
        status.current_motion_pixels = motion_result.motion_pixels;
        status.active_motion_regions = motion_result.active_regions;
        status.no_motion_duration_ms = motion_result.no_motion_duration_ms;
        status.motion_duration_ms = motion_result.motion_duration_ms;
    }
    
    return status;
}

esp_err_t VideoRecorder::enableCloudUpload(bool enable, MQTTClient* mqtt_client) {
    config_.enable_cloud_upload = enable;
    
    if (enable && mqtt_client) {
        mqtt_client_ = mqtt_client;
        
        if (cloud_uploader_ && current_state_ != RecorderState::UNINITIALIZED) {
            CloudUploader::UploadConfig upload_config;
            upload_config.auto_upload_new_recordings = config_.auto_upload_motion_recordings;
            upload_config.default_priority = config_.upload_priority;
            upload_config.delete_after_upload = config_.delete_after_upload;
            
            esp_err_t ret = cloud_uploader_->initialize(upload_config, mqtt_client_);
            if (ret == ESP_OK) {
                ret = cloud_uploader_->start();
                ESP_LOGI(TAG, "Cloud upload %s", (ret == ESP_OK) ? "enabled" : "failed to start");
            }
            return ret;
        }
    } else if (!enable && cloud_uploader_) {
        cloud_uploader_->stop();
        cloud_uploader_->deinitialize();
        ESP_LOGI(TAG, "Cloud upload disabled");
    }
    
    return ESP_OK;
}

esp_err_t VideoRecorder::uploadFileToCloud(const std::string& file_path, CloudUploader::UploadPriority priority) {
    if (!config_.enable_cloud_upload || !cloud_uploader_) {
        ESP_LOGE(TAG, "Cloud upload not enabled");
        return ESP_ERR_INVALID_STATE;
    }
    
    return cloud_uploader_->uploadFile(file_path, priority);
}

CloudUploader::Statistics VideoRecorder::getUploadStatistics() const {
    if (cloud_uploader_) {
        return cloud_uploader_->getStatistics();
    }
    
    CloudUploader::Statistics empty_stats = {};
    return empty_stats;
}

CloudUploader::UploadProgress VideoRecorder::getCurrentUploadProgress() const {
    if (cloud_uploader_) {
        return cloud_uploader_->getCurrentProgress();
    }
    
    CloudUploader::UploadProgress empty_progress = {};
    return empty_progress;
}

void VideoRecorder::publishUploadStatus(const std::string& event, const std::string& file_path, 
                                       const std::string& cloud_url, const std::string& error_message) {
    // Create upload status JSON
    std::ostringstream json;
    json << "{";
    json << "\"event\":\"" << event << "\",";
    json << "\"file_path\":\"" << file_path << "\",";
    json << "\"timestamp\":" << (getCurrentTimestamp() / 1000000) << ",";
    
    if (!cloud_url.empty()) {
        json << "\"cloud_url\":\"" << cloud_url << "\",";
    }
    
    if (!error_message.empty()) {
        json << "\"error_message\":\"" << error_message << "\",";
    }
    
    // Add file info
    size_t pos = file_path.find_last_of("/\\");
    std::string filename = (pos != std::string::npos) ? file_path.substr(pos + 1) : file_path;
    json << "\"filename\":\"" << filename << "\"";
    json << "}";
    
    std::string topic = "devices/ESP32P4_CAM_001/status/upload";
    std::string payload = json.str();
    
    ESP_LOGI(TAG, "Publishing upload status: %s", payload.c_str());
    // mqtt_client_->publish(topic, payload, 1, false);
}

void VideoRecorder::printStatus() const {
    ESP_LOGI(TAG, "=== Video Recorder Status ===");
    ESP_LOGI(TAG, "State: %d", (int)current_state_);
    ESP_LOGI(TAG, "Mode: %d", (int)config_.mode);
    
    Statistics stats = getStatistics();
    ESP_LOGI(TAG, "Total recordings: %llu", stats.total_recordings);
    ESP_LOGI(TAG, "Total bytes: %s", formatFileSize(stats.total_bytes_recorded).c_str());
    ESP_LOGI(TAG, "Total time: %s", formatDuration(stats.total_recording_time_ms).c_str());
    ESP_LOGI(TAG, "Dropped frames: %llu", stats.dropped_frames);
    ESP_LOGI(TAG, "Current FPS: %.1f", stats.current_fps);
    
    // Motion-based recording status
    if (config_.mode == RecordingMode::MOTION_BASED) {
        MotionRecordingStatus motion_status = getMotionRecordingStatus();
        ESP_LOGI(TAG, "=== Motion Recording Status ===");
        ESP_LOGI(TAG, "Motion detection enabled: %s", motion_status.motion_recording_enabled ? "YES" : "NO");
        ESP_LOGI(TAG, "Motion currently detected: %s", motion_status.motion_currently_detected ? "YES" : "NO");
        ESP_LOGI(TAG, "Motion recording active: %s", motion_status.motion_recording_active ? "YES" : "NO");
        ESP_LOGI(TAG, "Current motion: %.1f%% (%d pixels, %d regions)",
                 motion_status.current_motion_percentage, motion_status.current_motion_pixels, 
                 motion_status.active_motion_regions);
        ESP_LOGI(TAG, "No motion duration: %llu ms", motion_status.no_motion_duration_ms);
        ESP_LOGI(TAG, "Motion duration: %llu ms", motion_status.motion_duration_ms);
        ESP_LOGI(TAG, "Recording duration: %llu ms", motion_status.recording_duration_ms);
        ESP_LOGI(TAG, "Motion sensitivity: %d", motion_status.motion_sensitivity);
        ESP_LOGI(TAG, "Motion timeout: %d ms", motion_status.motion_timeout_ms);
        ESP_LOGI(TAG, "Motion mode: %s", motion_status.motion_mode.c_str());
        if (!motion_status.current_filename.empty()) {
            ESP_LOGI(TAG, "Current file: %s", motion_status.current_filename.c_str());
        }
        
        // Print motion detector detailed status if available
        if (motion_detector_) {
            motion_detector_->printDetectionStatus();
        }
    }
    
    ESP_LOGI(TAG, "============================");
}