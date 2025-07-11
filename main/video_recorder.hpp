#pragma once

#include <string>
#include <functional>
#include <memory>
#include <vector>
#include "esp_err.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "h264_encoder.hpp"
#include "ts_muxer.hpp"
#include "motion_detector.hpp"
#include "cloud_uploader.hpp"
#include "stream_pusher.hpp"
#include "task_manager.hpp"

// High-level video recorder that orchestrates H.264 encoding and TS muxing
// Provides complete recording pipeline from camera frames to TS files
class VideoRecorder {
public:
    enum class RecorderState {
        UNINITIALIZED,
        INITIALIZED,
        RECORDING,
        PAUSED,
        ERROR
    };

    enum class RecordingMode {
        CONTINUOUS,     // Record continuously until stopped
        SCHEDULED,      // Record based on schedule
        MOTION_BASED,   // Record when motion detected
        MANUAL          // Manual start/stop only
    };

    enum class QualityPreset {
        LOW_QUALITY,    // 480p@15fps, 1Mbps
        MEDIUM_QUALITY, // 720p@24fps, 2Mbps
        HIGH_QUALITY,   // 1080p@30fps, 4Mbps
        ULTRA_QUALITY,  // 1080p@60fps, 8Mbps
        CUSTOM          // User-defined settings
    };

    struct RecorderConfig {
        // Recording settings
        RecordingMode mode = RecordingMode::MANUAL;
        QualityPreset quality_preset = QualityPreset::HIGH_QUALITY;
        std::string output_directory = "/sdcard/recordings/";
        
        // Custom quality settings (used when preset = CUSTOM)
        uint32_t custom_width = 1920;
        uint32_t custom_height = 1080;
        uint32_t custom_fps = 30;
        uint32_t custom_bitrate = 4000000;  // 4 Mbps
        
        // File management
        bool enable_auto_rotation = true;
        uint64_t max_file_size_mb = 1024;   // 1GB
        uint32_t max_file_duration_min = 60; // 60 minutes
        uint32_t max_total_files = 100;     // Keep max 100 files
        bool delete_oldest_when_full = true;
        
        // Performance settings
        uint32_t frame_buffer_count = 8;
        uint32_t encoding_queue_size = 32;
        bool enable_frame_dropping = true;
        uint32_t max_encoding_delay_ms = 500;
        
        // Motion detection (for motion-based recording)
        bool enable_motion_detection = false;
        uint32_t motion_threshold = 50;     // 0-100
        uint32_t motion_debounce_ms = 2000; // 2 seconds
        uint32_t no_motion_timeout_ms = 30000; // Auto-stop after 30 seconds of no motion
        MotionDetector::DetectionMode motion_mode = MotionDetector::DetectionMode::ADVANCED;
        bool enable_adaptive_motion = true; // Enable adaptive motion sensitivity
        
        // Cloud upload configuration
        bool enable_cloud_upload = false;   // Enable automatic cloud upload
        bool auto_upload_motion_recordings = true; // Auto-upload motion-triggered recordings
        CloudUploader::UploadPriority upload_priority = CloudUploader::UploadPriority::NORMAL;
        bool delete_after_upload = true;    // Delete local file after successful upload
        uint32_t upload_delay_sec = 30;     // Delay before uploading (seconds)
        
        // Live streaming configuration
        bool enable_live_streaming = false; // Enable live RTMP streaming
        bool auto_start_stream_on_motion = false; // Auto-start streaming when motion detected
        StreamPusher::StreamQuality stream_quality = StreamPusher::StreamQuality::HIGH;
        StreamPusher::StreamTrigger stream_trigger = StreamPusher::StreamTrigger::MANUAL;
        std::string rtmp_server_url = "";   // RTMP server URL (optional)
        std::string stream_key = "";        // Stream key (optional)
        
        // Scheduled recording
        struct ScheduleEntry {
            uint8_t start_hour = 0;
            uint8_t start_minute = 0;
            uint8_t end_hour = 23;
            uint8_t end_minute = 59;
            bool enabled = false;
            uint8_t days_of_week = 0x7F;    // Bit mask: Sun-Sat
        };
        std::vector<ScheduleEntry> schedule;
    };

    struct RecordingInfo {
        std::string filename;
        uint64_t file_size_bytes = 0;
        uint64_t duration_ms = 0;
        uint64_t start_timestamp_us = 0;
        uint64_t end_timestamp_us = 0;
        uint32_t frame_count = 0;
        uint32_t keyframe_count = 0;
        bool is_complete = false;
    };

    using StateCallback = std::function<void(RecorderState state, const std::string& message)>;
    using FileCallback = std::function<void(const RecordingInfo& info)>;
    using ErrorCallback = std::function<void(const std::string& error)>;

    VideoRecorder();
    ~VideoRecorder();

    // Recorder lifecycle
    esp_err_t initialize(const RecorderConfig& config);
    esp_err_t deinitialize();
    
    // Recording control
    esp_err_t startRecording(const std::string& filename = "");
    esp_err_t stopRecording();
    esp_err_t pauseRecording();
    esp_err_t resumeRecording();
    
    // Frame input
    esp_err_t recordFrame(camera_fb_t* frame);
    esp_err_t forceKeyFrame();
    
    // Configuration
    esp_err_t setQualityPreset(QualityPreset preset);
    esp_err_t setCustomQuality(uint32_t width, uint32_t height, uint32_t fps, uint32_t bitrate);
    esp_err_t setRecordingMode(RecordingMode mode);
    esp_err_t setOutputDirectory(const std::string& path);
    esp_err_t addScheduleEntry(const RecorderConfig::ScheduleEntry& entry);
    esp_err_t clearSchedule();
    
    // File management
    esp_err_t rotateFile();
    esp_err_t cleanupOldFiles();
    std::vector<std::string> getRecordingList() const;
    esp_err_t deleteRecording(const std::string& filename);
    uint64_t getTotalStorageUsed() const;
    uint64_t getAvailableStorage() const;
    
    // Motion detection
    esp_err_t enableMotionDetection(bool enable);
    esp_err_t setMotionThreshold(uint32_t threshold);
    esp_err_t setMotionTimeout(uint32_t timeout_ms);
    esp_err_t setMotionMode(MotionDetector::DetectionMode mode);
    bool isMotionDetected() const;
    MotionDetector::DetectionResult getMotionStatus() const;
    
    // Callbacks
    void setStateCallback(StateCallback callback);
    void setFileCallback(FileCallback callback);
    void setErrorCallback(ErrorCallback callback);
    
    // Status
    RecorderState getState() const { return current_state_; }
    bool isRecording() const { return current_state_ == RecorderState::RECORDING; }
    bool isPaused() const { return current_state_ == RecorderState::PAUSED; }
    RecordingInfo getCurrentRecordingInfo() const;
    
    // Motion-based recording status
    struct MotionRecordingStatus {
        bool motion_recording_enabled = false;
        bool motion_currently_detected = false;
        bool motion_recording_active = false;
        float current_motion_percentage = 0.0f;
        uint32_t current_motion_pixels = 0;
        uint32_t active_motion_regions = 0;
        uint64_t no_motion_duration_ms = 0;
        uint64_t motion_duration_ms = 0;
        uint64_t recording_duration_ms = 0;
        uint32_t motion_sensitivity = 50;
        uint32_t motion_timeout_ms = 30000;
        std::string motion_mode = "advanced";
        std::string current_filename = "";
    };
    MotionRecordingStatus getMotionRecordingStatus() const;
    
    // Statistics
    struct Statistics {
        uint64_t total_recordings = 0;
        uint64_t total_bytes_recorded = 0;
        uint64_t total_recording_time_ms = 0;
        uint64_t total_frames_recorded = 0;
        uint64_t dropped_frames = 0;
        uint64_t encoding_errors = 0;
        uint64_t file_errors = 0;
        double avg_fps = 0.0;
        double avg_bitrate_mbps = 0.0;
        uint32_t motion_events = 0;
        
        // Current recording stats
        uint64_t current_frames = 0;
        uint64_t current_duration_ms = 0;
        uint64_t current_file_size = 0;
        double current_fps = 0.0;
        double current_bitrate_mbps = 0.0;
    };
    
    Statistics getStatistics() const;
    void resetStatistics();
    
    // Cloud upload management
    esp_err_t enableCloudUpload(bool enable, MQTTClient* mqtt_client = nullptr);
    esp_err_t uploadFileToCloud(const std::string& file_path, CloudUploader::UploadPriority priority = CloudUploader::UploadPriority::NORMAL);
    CloudUploader::Statistics getUploadStatistics() const;
    CloudUploader::UploadProgress getCurrentUploadProgress() const;
    
    // Live streaming management
    esp_err_t enableLiveStreaming(bool enable, MQTTClient* mqtt_client = nullptr);
    esp_err_t startLiveStream(const std::string& stream_key = "");
    esp_err_t stopLiveStream();
    esp_err_t setStreamQuality(StreamPusher::StreamQuality quality);
    esp_err_t setStreamServer(const std::string& rtmp_url, const std::string& stream_key);
    StreamPusher::StreamStatus getStreamStatus() const;
    StreamPusher::StreamMetrics getStreamMetrics() const;
    bool isLiveStreaming() const;
    
    // System monitoring
    void printStatus() const;
    esp_err_t performHealthCheck();

private:
    // Internal components
    std::unique_ptr<H264Encoder> encoder_;
    std::unique_ptr<TSMuxer> muxer_;
    std::unique_ptr<MotionDetector> motion_detector_;
    std::unique_ptr<CloudUploader> cloud_uploader_;
    std::unique_ptr<StreamPusher> stream_pusher_;
    TaskManager* task_manager_;
    MQTTClient* mqtt_client_;
    
    // Configuration and state
    RecorderConfig config_;
    RecorderState current_state_;
    RecordingInfo current_recording_;
    
    // Callbacks
    StateCallback state_callback_;
    FileCallback file_callback_;
    ErrorCallback error_callback_;
    
    // Frame processing pipeline
    static void recordingTask(void* parameter);
    static void motionDetectionTask(void* parameter);
    static void scheduleTask(void* parameter);
    static void maintenanceTimerCallback(TimerHandle_t timer);
    
    esp_err_t processFrame(camera_fb_t* frame);
    esp_err_t detectMotion(camera_fb_t* frame);
    bool shouldRecord() const;
    bool isScheduledTime() const;
    
    // File management
    std::string generateFilename() const;
    esp_err_t setupQualitySettings();
    esp_err_t createOutputDirectory();
    esp_err_t validateStorageSpace();
    
    // Component callbacks
    void onEncoderStateChanged(H264Encoder::EncoderState state, const std::string& message);
    void onEncodedFrame(const H264Encoder::FrameInfo& frame);
    void onMuxerStateChanged(TSMuxer::MuxerState state, const std::string& message);
    void onFileCompleted(const std::string& filename, uint64_t size_bytes, uint64_t duration_ms);
    void onMotionDetected(const MotionDetector::DetectionResult& result);
    void onUploadProgress(const CloudUploader::UploadProgress& progress);
    void onUploadCompleted(const std::string& file_path, bool success, const std::string& cloud_url);
    void publishUploadStatus(const std::string& event, const std::string& file_path, 
                           const std::string& cloud_url, const std::string& error_message);
    
    // Task management
    esp_err_t createTasks();
    void deleteTasks();
    
    // Motion detection state
    bool motion_recording_active_;
    uint64_t last_motion_recording_start_;
    uint64_t last_motion_recording_stop_;
    
    // FreeRTOS synchronization
    QueueHandle_t frame_queue_;
    SemaphoreHandle_t state_mutex_;
    SemaphoreHandle_t stats_mutex_;
    TimerHandle_t maintenance_timer_;
    TaskHandle_t recording_task_handle_;
    TaskHandle_t motion_task_handle_;
    TaskHandle_t schedule_task_handle_;
    volatile bool tasks_running_;
    
    // Statistics
    Statistics statistics_;
    uint64_t recording_start_time_;
    uint64_t last_frame_time_;
    uint32_t frame_counter_;
    
    // Storage management
    mutable std::vector<std::string> recording_files_;
    mutable uint64_t last_file_scan_time_;
    
    // Task configuration
    static const uint32_t RECORDING_TASK_STACK_SIZE = 8192;
    static const UBaseType_t RECORDING_TASK_PRIORITY = 5;
    static const uint32_t MOTION_TASK_STACK_SIZE = 4096;
    static const UBaseType_t MOTION_TASK_PRIORITY = 3;
    static const uint32_t SCHEDULE_TASK_STACK_SIZE = 2048;
    static const UBaseType_t SCHEDULE_TASK_PRIORITY = 2;
    static const uint32_t FRAME_QUEUE_SIZE = 16;
    static const uint32_t MAINTENANCE_INTERVAL_MS = 60000;  // 1 minute
    
    // Utility methods
    void setState(RecorderState state, const std::string& message = "");
    void updateStatistics(bool frame_processed);
    uint64_t getCurrentTimestamp();
    std::string formatDuration(uint64_t duration_ms) const;
    std::string formatFileSize(uint64_t size_bytes) const;
    void logError(const std::string& error);
    
    static const char* TAG;
};