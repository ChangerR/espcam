#pragma once

#include <string>
#include <functional>
#include <memory>
#include <fstream>
#include <vector>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "h264_encoder.hpp"

// MPEG-TS (Transport Stream) muxer for H.264 video
// Generates standard MPEG-TS files compatible with media players
class TSMuxer {
public:
    enum class MuxerState {
        UNINITIALIZED,
        INITIALIZED,
        RECORDING,
        ERROR
    };

    struct TSConfig {
        std::string output_path = "/sdcard/recordings/";
        std::string filename_prefix = "video_";
        std::string filename_suffix = ".ts";
        
        // TS parameters
        uint32_t packet_size = 188;        // Standard TS packet size
        uint32_t program_number = 1;
        uint16_t video_pid = 0x100;        // Video PID
        uint16_t pmt_pid = 0x1000;         // PMT PID
        uint16_t pat_pid = 0x0000;         // PAT PID (always 0)
        uint16_t pcr_pid = 0x100;          // PCR PID (same as video)
        
        // Timing parameters
        uint32_t pcr_interval_ms = 100;    // PCR transmission interval
        uint32_t pat_interval_ms = 500;    // PAT transmission interval
        uint32_t pmt_interval_ms = 500;    // PMT transmission interval
        
        // File management
        uint64_t max_file_size_mb = 1024;  // 1GB max file size
        uint32_t max_duration_sec = 3600;  // 1 hour max duration
        bool enable_file_rotation = true;
        
        // Buffer settings
        uint32_t write_buffer_size = 64 * 1024;  // 64KB write buffer
        uint32_t packet_queue_size = 1000;       // Packet queue depth
    };

    struct TSPacket {
        uint8_t* data;
        size_t size;
        uint64_t timestamp_us;
        bool is_keyframe;
        uint32_t frame_number;
    };

    using StateCallback = std::function<void(MuxerState state, const std::string& message)>;
    using FileCallback = std::function<void(const std::string& filename, uint64_t size_bytes, uint64_t duration_ms)>;

    TSMuxer();
    ~TSMuxer();

    // Muxer lifecycle
    esp_err_t initialize(const TSConfig& config);
    esp_err_t deinitialize();
    esp_err_t startRecording(const std::string& filename = "");
    esp_err_t stopRecording();

    // Frame processing
    esp_err_t writeFrame(const H264Encoder::FrameInfo& frame);
    esp_err_t forceKeyFrame();
    
    // File management
    esp_err_t rotateFile();
    std::string getCurrentFilename() const;
    uint64_t getCurrentFileSize() const;
    uint64_t getRecordingDuration() const;
    
    // Configuration
    esp_err_t setOutputPath(const std::string& path);
    esp_err_t setMaxFileSize(uint64_t size_mb);
    esp_err_t setMaxDuration(uint32_t duration_sec);
    
    // Callbacks
    void setStateCallback(StateCallback callback);
    void setFileCallback(FileCallback callback);
    
    // Status
    MuxerState getState() const { return current_state_; }
    bool isRecording() const { return current_state_ == MuxerState::RECORDING; }
    
    // Statistics
    struct Statistics {
        uint64_t files_created = 0;
        uint64_t total_bytes_written = 0;
        uint64_t total_frames_written = 0;
        uint64_t total_recording_time_ms = 0;
        uint32_t current_file_frames = 0;
        uint64_t current_file_size = 0;
        uint64_t current_file_duration_ms = 0;
        double avg_write_speed_mbps = 0.0;
        uint32_t write_errors = 0;
    };
    
    Statistics getStatistics() const;
    void resetStatistics();

private:
    // TS packet structure
    struct TSHeader {
        uint8_t sync_byte;           // 0x47
        uint16_t transport_error_indicator : 1;
        uint16_t payload_unit_start_indicator : 1;
        uint16_t transport_priority : 1;
        uint16_t pid : 13;
        uint8_t transport_scrambling_control : 2;
        uint8_t adaptation_field_control : 2;
        uint8_t continuity_counter : 4;
    } __attribute__((packed));

    // PES header structure
    struct PESHeader {
        uint32_t packet_start_code_prefix : 24;  // 0x000001
        uint8_t stream_id;                       // 0xE0 for video
        uint16_t pes_packet_length;
        uint8_t marker_bits : 2;                 // 0b10
        uint8_t scrambling_control : 2;
        uint8_t priority : 1;
        uint8_t data_alignment_indicator : 1;
        uint8_t copyright : 1;
        uint8_t original_or_copy : 1;
        uint8_t pts_dts_flags : 2;
        uint8_t escr_flag : 1;
        uint8_t es_rate_flag : 1;
        uint8_t dsm_trick_mode_flag : 1;
        uint8_t additional_copy_info_flag : 1;
        uint8_t crc_flag : 1;
        uint8_t extension_flag : 1;
        uint8_t pes_header_data_length;
    } __attribute__((packed));

    // Internal methods
    esp_err_t createOutputFile(const std::string& filename);
    esp_err_t closeOutputFile();
    std::string generateFilename() const;
    
    // TS packet generation
    esp_err_t writePATPacket();
    esp_err_t writePMTPacket();
    esp_err_t writePCRPacket(uint64_t pcr);
    esp_err_t writeVideoPacket(const uint8_t* data, size_t size, uint64_t pts, bool is_keyframe);
    
    // Low-level TS operations
    esp_err_t writePacket(const uint8_t* packet, size_t size);
    void createTSHeader(TSHeader& header, uint16_t pid, bool payload_start, uint8_t& continuity_counter);
    void createPESHeader(PESHeader& header, size_t payload_size, uint64_t pts);
    uint64_t convertTimestampToPTS(uint64_t timestamp_us);
    uint32_t calculateCRC32(const uint8_t* data, size_t length);
    
    // File I/O management
    static void fileWriteTask(void* parameter);
    esp_err_t flushWriteBuffer();
    esp_err_t checkFileRotation();
    
    // Utilities
    void setState(MuxerState state, const std::string& message = "");
    void updateStatistics(size_t bytes_written, bool is_new_frame);
    uint64_t getCurrentTimestamp();
    bool createDirectoryIfNotExists(const std::string& path);
    
    TSConfig config_;
    MuxerState current_state_;
    StateCallback state_callback_;
    FileCallback file_callback_;
    
    // File management
    std::unique_ptr<std::ofstream> output_file_;
    std::string current_filename_;
    uint64_t file_start_timestamp_;
    uint64_t recording_start_timestamp_;
    
    // TS state
    uint8_t pat_continuity_counter_;
    uint8_t pmt_continuity_counter_;
    uint8_t video_continuity_counter_;
    uint8_t pcr_continuity_counter_;
    uint64_t last_pcr_timestamp_;
    uint64_t last_pat_timestamp_;
    uint64_t last_pmt_timestamp_;
    
    // Write buffering
    std::vector<uint8_t> write_buffer_;
    size_t write_buffer_pos_;
    
    // FreeRTOS synchronization
    TaskHandle_t write_task_handle_;
    QueueHandle_t packet_queue_;
    SemaphoreHandle_t state_mutex_;
    SemaphoreHandle_t file_mutex_;
    volatile bool writing_active_;
    
    // Statistics
    mutable SemaphoreHandle_t stats_mutex_;
    Statistics statistics_;
    
    // Task configuration
    static const uint32_t WRITE_TASK_STACK_SIZE = 8192;
    static const UBaseType_t WRITE_TASK_PRIORITY = 4;  // Medium priority
    
    static const char* TAG;
    
    // TS constants
    static const uint8_t TS_SYNC_BYTE = 0x47;
    static const uint8_t TS_PACKET_SIZE = 188;
    static const uint16_t TS_PAT_PID = 0x0000;
    static const uint8_t TS_PAT_TABLE_ID = 0x00;
    static const uint8_t TS_PMT_TABLE_ID = 0x02;
    static const uint8_t PES_VIDEO_STREAM_ID = 0xE0;
    static const uint32_t PES_START_CODE = 0x000001;
    static const uint64_t PTS_FREQUENCY = 90000;  // 90 kHz
};