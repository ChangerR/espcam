#include "ts_muxer.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>

const char* TSMuxer::TAG = "TSMuxer";

TSMuxer::TSMuxer()
    : current_state_(MuxerState::UNINITIALIZED)
    , output_file_(nullptr)
    , file_start_timestamp_(0)
    , recording_start_timestamp_(0)
    , pat_continuity_counter_(0)
    , pmt_continuity_counter_(0)
    , video_continuity_counter_(0)
    , pcr_continuity_counter_(0)
    , last_pcr_timestamp_(0)
    , last_pat_timestamp_(0)
    , last_pmt_timestamp_(0)
    , write_buffer_pos_(0)
    , write_task_handle_(nullptr)
    , packet_queue_(nullptr)
    , state_mutex_(nullptr)
    , file_mutex_(nullptr)
    , writing_active_(false)
    , stats_mutex_(nullptr)
{
    // Initialize mutexes
    state_mutex_ = xSemaphoreCreateMutex();
    file_mutex_ = xSemaphoreCreateMutex();
    stats_mutex_ = xSemaphoreCreateMutex();
    
    if (!state_mutex_ || !file_mutex_ || !stats_mutex_) {
        ESP_LOGE(TAG, "Failed to create mutexes");
    }
    
    // Initialize statistics
    statistics_ = {};
}

TSMuxer::~TSMuxer() {
    deinitialize();
    
    if (state_mutex_) {
        vSemaphoreDelete(state_mutex_);
    }
    if (file_mutex_) {
        vSemaphoreDelete(file_mutex_);
    }
    if (stats_mutex_) {
        vSemaphoreDelete(stats_mutex_);
    }
}

esp_err_t TSMuxer::initialize(const TSConfig& config) {
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_FAIL;
    }
    
    if (current_state_ != MuxerState::UNINITIALIZED) {
        ESP_LOGW(TAG, "Muxer already initialized");
        xSemaphoreGive(state_mutex_);
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing TS muxer");
    config_ = config;
    
    // Validate configuration
    if (config_.packet_size != 188) {
        ESP_LOGE(TAG, "Invalid TS packet size: %lu (must be 188)", (unsigned long)config_.packet_size);
        xSemaphoreGive(state_mutex_);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Create output directory if it doesn't exist
    if (!createDirectoryIfNotExists(config_.output_path)) {
        ESP_LOGE(TAG, "Failed to create output directory: %s", config_.output_path.c_str());
        xSemaphoreGive(state_mutex_);
        return ESP_FAIL;
    }
    
    // Initialize write buffer
    write_buffer_.resize(config_.write_buffer_size);
    write_buffer_pos_ = 0;
    
    // Create packet queue
    packet_queue_ = xQueueCreate(config_.packet_queue_size, sizeof(TSPacket));
    if (!packet_queue_) {
        ESP_LOGE(TAG, "Failed to create packet queue");
        xSemaphoreGive(state_mutex_);
        return ESP_FAIL;
    }
    
    setState(MuxerState::INITIALIZED, "TS muxer initialized");
    ESP_LOGI(TAG, "TS muxer initialized: output=%s, max_size=%lluMB", 
             config_.output_path.c_str(), config_.max_file_size_mb);
    
    xSemaphoreGive(state_mutex_);
    return ESP_OK;
}

esp_err_t TSMuxer::deinitialize() {
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_FAIL;
    }
    
    if (current_state_ == MuxerState::UNINITIALIZED) {
        xSemaphoreGive(state_mutex_);
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Deinitializing TS muxer");
    
    // Stop recording if active
    if (current_state_ == MuxerState::RECORDING) {
        stopRecording();
    }
    
    // Clean up packet queue
    if (packet_queue_) {
        vQueueDelete(packet_queue_);
        packet_queue_ = nullptr;
    }
    
    // Clear write buffer
    write_buffer_.clear();
    
    setState(MuxerState::UNINITIALIZED, "TS muxer deinitialized");
    
    xSemaphoreGive(state_mutex_);
    return ESP_OK;
}

esp_err_t TSMuxer::startRecording(const std::string& filename) {
    if (current_state_ != MuxerState::INITIALIZED) {
        ESP_LOGE(TAG, "Muxer not initialized");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Starting TS recording");
    
    // Generate filename if not provided
    std::string file_to_create = filename.empty() ? generateFilename() : filename;
    
    // Create output file
    esp_err_t ret = createOutputFile(file_to_create);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create output file");
        return ret;
    }
    
    // Reset TS state
    pat_continuity_counter_ = 0;
    pmt_continuity_counter_ = 0;
    video_continuity_counter_ = 0;
    pcr_continuity_counter_ = 0;
    last_pcr_timestamp_ = 0;
    last_pat_timestamp_ = 0;
    last_pmt_timestamp_ = 0;
    
    // Start write task
    writing_active_ = true;
    
    BaseType_t result = xTaskCreate(
        fileWriteTask,
        "ts_writer",
        WRITE_TASK_STACK_SIZE,
        this,
        WRITE_TASK_PRIORITY,
        &write_task_handle_
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create write task");
        writing_active_ = false;
        closeOutputFile();
        return ESP_FAIL;
    }
    
    // Record start time
    recording_start_timestamp_ = getCurrentTimestamp();
    file_start_timestamp_ = recording_start_timestamp_;
    
    setState(MuxerState::RECORDING, "TS recording started");
    resetStatistics();
    
    // Write initial PAT and PMT packets
    writePATPacket();
    writePMTPacket();
    
    return ESP_OK;
}

esp_err_t TSMuxer::stopRecording() {
    if (current_state_ != MuxerState::RECORDING) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Stopping TS recording");
    
    // Stop write task
    writing_active_ = false;
    
    if (write_task_handle_) {
        vTaskDelay(pdMS_TO_TICKS(100));  // Allow task to finish
        vTaskDelete(write_task_handle_);
        write_task_handle_ = nullptr;
    }
    
    // Flush any remaining data
    flushWriteBuffer();
    
    // Close output file
    closeOutputFile();
    
    setState(MuxerState::INITIALIZED, "TS recording stopped");
    
    return ESP_OK;
}

esp_err_t TSMuxer::writeFrame(const H264Encoder::FrameInfo& frame) {
    if (current_state_ != MuxerState::RECORDING) {
        ESP_LOGE(TAG, "Not in recording state");
        return ESP_FAIL;
    }
    
    if (!frame.data || frame.size == 0) {
        ESP_LOGE(TAG, "Invalid frame data");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check for file rotation
    checkFileRotation();
    
    // Convert timestamp to PTS
    uint64_t pts = convertTimestampToPTS(frame.timestamp_us);
    
    // Write PCR if needed
    uint64_t now = getCurrentTimestamp();
    if (now - last_pcr_timestamp_ >= config_.pcr_interval_ms * 1000) {
        writePCRPacket(pts);
        last_pcr_timestamp_ = now;
    }
    
    // Write PAT if needed
    if (now - last_pat_timestamp_ >= config_.pat_interval_ms * 1000) {
        writePATPacket();
        last_pat_timestamp_ = now;
    }
    
    // Write PMT if needed
    if (now - last_pmt_timestamp_ >= config_.pmt_interval_ms * 1000) {
        writePMTPacket();
        last_pmt_timestamp_ = now;
    }
    
    // Write video frame
    esp_err_t ret = writeVideoPacket(frame.data, frame.size, pts, frame.is_keyframe);
    if (ret == ESP_OK) {
        updateStatistics(frame.size, true);
    }
    
    return ret;
}

esp_err_t TSMuxer::createOutputFile(const std::string& filename) {
    if (xSemaphoreTake(file_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_FAIL;
    }
    
    // Close existing file if open
    if (output_file_) {
        output_file_->close();
    }
    
    // Create new file
    std::string full_path = config_.output_path + "/" + filename;
    output_file_ = std::make_unique<std::ofstream>(full_path, std::ios::binary | std::ios::trunc);
    
    if (!output_file_->is_open()) {
        ESP_LOGE(TAG, "Failed to create file: %s", full_path.c_str());
        output_file_.reset();
        xSemaphoreGive(file_mutex_);
        return ESP_FAIL;
    }
    
    current_filename_ = filename;
    ESP_LOGI(TAG, "Created output file: %s", full_path.c_str());
    
    // Update statistics
    if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        statistics_.files_created++;
        statistics_.current_file_frames = 0;
        statistics_.current_file_size = 0;
        statistics_.current_file_duration_ms = 0;
        xSemaphoreGive(stats_mutex_);
    }
    
    xSemaphoreGive(file_mutex_);
    return ESP_OK;
}

esp_err_t TSMuxer::closeOutputFile() {
    if (xSemaphoreTake(file_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_FAIL;
    }
    
    if (output_file_) {
        output_file_->close();
        
        ESP_LOGI(TAG, "Closed output file: %s (size: %llu bytes, duration: %llu ms)",
                 current_filename_.c_str(),
                 statistics_.current_file_size,
                 statistics_.current_file_duration_ms);
        
        // Call file completion callback
        if (file_callback_) {
            file_callback_(current_filename_, statistics_.current_file_size, statistics_.current_file_duration_ms);
        }
        
        output_file_.reset();
        current_filename_.clear();
    }
    
    xSemaphoreGive(file_mutex_);
    return ESP_OK;
}

std::string TSMuxer::generateFilename() const {
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    
    std::ostringstream oss;
    oss << config_.filename_prefix
        << std::put_time(&tm, "%Y%m%d_%H%M%S")
        << config_.filename_suffix;
    
    return oss.str();
}

esp_err_t TSMuxer::writePATPacket() {
    uint8_t packet[TS_PACKET_SIZE];
    memset(packet, 0xFF, sizeof(packet));  // Fill with padding
    
    // TS Header
    TSHeader header;
    createTSHeader(header, TS_PAT_PID, true, pat_continuity_counter_);
    memcpy(packet, &header, 4);
    
    // PAT payload
    uint8_t* payload = packet + 4;
    int pos = 0;
    
    // Pointer field (for PSI tables)
    payload[pos++] = 0x00;
    
    // PAT table
    payload[pos++] = TS_PAT_TABLE_ID;        // Table ID
    payload[pos++] = 0xB0;                   // Section syntax indicator + reserved + section length (high)
    payload[pos++] = 0x0D;                   // Section length (low) - 13 bytes
    payload[pos++] = 0x00; payload[pos++] = 0x01;  // Transport stream ID
    payload[pos++] = 0xC1;                   // Reserved + version + current_next_indicator
    payload[pos++] = 0x00;                   // Section number
    payload[pos++] = 0x00;                   // Last section number
    
    // Program loop
    payload[pos++] = (config_.program_number >> 8) & 0xFF;  // Program number (high)
    payload[pos++] = config_.program_number & 0xFF;         // Program number (low)
    payload[pos++] = 0xE0 | ((config_.pmt_pid >> 8) & 0x1F);  // Reserved + PMT PID (high)
    payload[pos++] = config_.pmt_pid & 0xFF;                   // PMT PID (low)
    
    // CRC32 (simplified - would need proper implementation)
    uint32_t crc = calculateCRC32(payload + 1, pos - 1);
    payload[pos++] = (crc >> 24) & 0xFF;
    payload[pos++] = (crc >> 16) & 0xFF;
    payload[pos++] = (crc >> 8) & 0xFF;
    payload[pos++] = crc & 0xFF;
    
    return writePacket(packet, TS_PACKET_SIZE);
}

esp_err_t TSMuxer::writePMTPacket() {
    uint8_t packet[TS_PACKET_SIZE];
    memset(packet, 0xFF, sizeof(packet));  // Fill with padding
    
    // TS Header
    TSHeader header;
    createTSHeader(header, config_.pmt_pid, true, pmt_continuity_counter_);
    memcpy(packet, &header, 4);
    
    // PMT payload
    uint8_t* payload = packet + 4;
    int pos = 0;
    
    // Pointer field
    payload[pos++] = 0x00;
    
    // PMT table
    payload[pos++] = TS_PMT_TABLE_ID;        // Table ID
    payload[pos++] = 0xB0;                   // Section syntax indicator + reserved + section length (high)
    payload[pos++] = 0x17;                   // Section length (low) - 23 bytes
    payload[pos++] = (config_.program_number >> 8) & 0xFF;  // Program number (high)
    payload[pos++] = config_.program_number & 0xFF;         // Program number (low)
    payload[pos++] = 0xC1;                   // Reserved + version + current_next_indicator
    payload[pos++] = 0x00;                   // Section number
    payload[pos++] = 0x00;                   // Last section number
    payload[pos++] = 0xE0 | ((config_.pcr_pid >> 8) & 0x1F);  // Reserved + PCR PID (high)
    payload[pos++] = config_.pcr_pid & 0xFF;                   // PCR PID (low)
    payload[pos++] = 0xF0; payload[pos++] = 0x00;             // Program info length
    
    // ES loop - Video stream
    payload[pos++] = 0x1B;                   // Stream type (H.264 video)
    payload[pos++] = 0xE0 | ((config_.video_pid >> 8) & 0x1F);  // Reserved + Elementary PID (high)
    payload[pos++] = config_.video_pid & 0xFF;                   // Elementary PID (low)
    payload[pos++] = 0xF0; payload[pos++] = 0x06;               // ES info length
    
    // H.264 descriptor
    payload[pos++] = 0x28;                   // AVC video descriptor tag
    payload[pos++] = 0x04;                   // Descriptor length
    payload[pos++] = 0x42;                   // Profile indication (Baseline)
    payload[pos++] = 0x00;                   // Constraint flags
    payload[pos++] = 0x1F;                   // Level indication
    payload[pos++] = 0xFF;                   // Reserved + length_size_minus_one
    
    // CRC32
    uint32_t crc = calculateCRC32(payload + 1, pos - 1);
    payload[pos++] = (crc >> 24) & 0xFF;
    payload[pos++] = (crc >> 16) & 0xFF;
    payload[pos++] = (crc >> 8) & 0xFF;
    payload[pos++] = crc & 0xFF;
    
    return writePacket(packet, TS_PACKET_SIZE);
}

esp_err_t TSMuxer::writePCRPacket(uint64_t pcr) {
    uint8_t packet[TS_PACKET_SIZE];
    memset(packet, 0xFF, sizeof(packet));  // Fill with padding
    
    // TS Header with adaptation field
    TSHeader header;
    header.sync_byte = TS_SYNC_BYTE;
    header.transport_error_indicator = 0;
    header.payload_unit_start_indicator = 0;
    header.transport_priority = 0;
    header.pid = config_.pcr_pid;
    header.transport_scrambling_control = 0;
    header.adaptation_field_control = 2;  // Adaptation field only
    header.continuity_counter = pcr_continuity_counter_++;
    
    memcpy(packet, &header, 4);
    
    // Adaptation field
    uint8_t* adapt = packet + 4;
    adapt[0] = 183;    // Adaptation field length (188 - 4 - 1)
    adapt[1] = 0x10;   // PCR flag set
    
    // PCR (33 bits base + 6 bits reserved + 9 bits extension)
    uint64_t pcr_base = pcr / 300;
    uint16_t pcr_ext = pcr % 300;
    
    adapt[2] = (pcr_base >> 25) & 0xFF;
    adapt[3] = (pcr_base >> 17) & 0xFF;
    adapt[4] = (pcr_base >> 9) & 0xFF;
    adapt[5] = (pcr_base >> 1) & 0xFF;
    adapt[6] = ((pcr_base & 0x01) << 7) | 0x7E | ((pcr_ext >> 8) & 0x01);
    adapt[7] = pcr_ext & 0xFF;
    
    return writePacket(packet, TS_PACKET_SIZE);
}

esp_err_t TSMuxer::writeVideoPacket(const uint8_t* data, size_t size, uint64_t pts, bool is_keyframe) {
    // Create PES header
    PESHeader pes_header;
    pes_header.packet_start_code_prefix = PES_START_CODE;
    pes_header.stream_id = PES_VIDEO_STREAM_ID;
    pes_header.pes_packet_length = 0;  // Unbounded
    pes_header.marker_bits = 2;
    pes_header.scrambling_control = 0;
    pes_header.priority = 0;
    pes_header.data_alignment_indicator = is_keyframe ? 1 : 0;
    pes_header.copyright = 0;
    pes_header.original_or_copy = 1;
    pes_header.pts_dts_flags = 2;  // PTS only
    pes_header.escr_flag = 0;
    pes_header.es_rate_flag = 0;
    pes_header.dsm_trick_mode_flag = 0;
    pes_header.additional_copy_info_flag = 0;
    pes_header.crc_flag = 0;
    pes_header.extension_flag = 0;
    pes_header.pes_header_data_length = 5;  // PTS length
    
    // Calculate bytes needed for this frame
    size_t pes_header_size = 9 + pes_header.pes_header_data_length;  // Basic PES header + PTS
    size_t total_payload_size = pes_header_size + size;
    size_t packets_needed = (total_payload_size + TS_PACKET_SIZE - 5) / (TS_PACKET_SIZE - 4);  // -4 for TS header
    
    // Create packets
    size_t data_offset = 0;
    bool first_packet = true;
    
    for (size_t i = 0; i < packets_needed; i++) {
        uint8_t packet[TS_PACKET_SIZE];
        memset(packet, 0xFF, sizeof(packet));
        
        // TS Header
        TSHeader header;
        createTSHeader(header, config_.video_pid, first_packet, video_continuity_counter_);
        memcpy(packet, &header, 4);
        
        uint8_t* payload = packet + 4;
        size_t payload_space = TS_PACKET_SIZE - 4;
        size_t payload_pos = 0;
        
        if (first_packet) {
            // Add PES header
            memcpy(payload, &pes_header, 9);
            payload_pos += 9;
            
            // Add PTS
            uint64_t pts_33 = pts & 0x1FFFFFFFFULL;
            payload[payload_pos++] = 0x21 | ((pts_33 >> 29) & 0x0E);  // '0010' + PTS[32:30] + '1'
            payload[payload_pos++] = (pts_33 >> 22) & 0xFF;           // PTS[29:22]
            payload[payload_pos++] = 0x01 | ((pts_33 >> 14) & 0xFE);  // PTS[21:15] + '1'
            payload[payload_pos++] = (pts_33 >> 7) & 0xFF;            // PTS[14:7]
            payload[payload_pos++] = 0x01 | ((pts_33 << 1) & 0xFE);   // PTS[6:0] + '1'
            
            first_packet = false;
        }
        
        // Add video data
        size_t data_to_copy = std::min(payload_space - payload_pos, size - data_offset);
        if (data_to_copy > 0) {
            memcpy(payload + payload_pos, data + data_offset, data_to_copy);
            data_offset += data_to_copy;
        }
        
        esp_err_t ret = writePacket(packet, TS_PACKET_SIZE);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    
    return ESP_OK;
}

esp_err_t TSMuxer::writePacket(const uint8_t* packet, size_t size) {
    if (xSemaphoreTake(file_mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_FAIL;
    }
    
    if (!output_file_ || !output_file_->is_open()) {
        xSemaphoreGive(file_mutex_);
        return ESP_FAIL;
    }
    
    // Add to write buffer
    if (write_buffer_pos_ + size > write_buffer_.size()) {
        // Flush buffer first
        if (write_buffer_pos_ > 0) {
            output_file_->write(reinterpret_cast<const char*>(write_buffer_.data()), write_buffer_pos_);
            updateStatistics(write_buffer_pos_, false);
            write_buffer_pos_ = 0;
        }
    }
    
    // Add packet to buffer
    memcpy(write_buffer_.data() + write_buffer_pos_, packet, size);
    write_buffer_pos_ += size;
    
    xSemaphoreGive(file_mutex_);
    return ESP_OK;
}

void TSMuxer::createTSHeader(TSHeader& header, uint16_t pid, bool payload_start, uint8_t& continuity_counter) {
    header.sync_byte = TS_SYNC_BYTE;
    header.transport_error_indicator = 0;
    header.payload_unit_start_indicator = payload_start ? 1 : 0;
    header.transport_priority = 0;
    header.pid = pid;
    header.transport_scrambling_control = 0;
    header.adaptation_field_control = 1;  // Payload only
    header.continuity_counter = continuity_counter;
    
    continuity_counter = (continuity_counter + 1) & 0x0F;
}

uint64_t TSMuxer::convertTimestampToPTS(uint64_t timestamp_us) {
    // Convert microseconds to 90kHz PTS units
    return (timestamp_us * PTS_FREQUENCY) / 1000000;
}

uint32_t TSMuxer::calculateCRC32(const uint8_t* data, size_t length) {
    // Simplified CRC32 - in a real implementation, use proper MPEG CRC32
    uint32_t crc = 0xFFFFFFFF;
    
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return ~crc;
}

void TSMuxer::fileWriteTask(void* parameter) {
    TSMuxer* self = static_cast<TSMuxer*>(parameter);
    ESP_LOGI(TAG, "File write task started");
    
    const TickType_t flush_interval = pdMS_TO_TICKS(1000);  // Flush every second
    TickType_t last_flush = xTaskGetTickCount();
    
    while (self->writing_active_) {
        TickType_t now = xTaskGetTickCount();
        
        // Periodic flush
        if (now - last_flush >= flush_interval) {
            self->flushWriteBuffer();
            last_flush = now;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Final flush
    self->flushWriteBuffer();
    
    ESP_LOGI(TAG, "File write task ended");
    vTaskDelete(nullptr);
}

esp_err_t TSMuxer::flushWriteBuffer() {
    if (xSemaphoreTake(file_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_FAIL;
    }
    
    if (output_file_ && output_file_->is_open() && write_buffer_pos_ > 0) {
        output_file_->write(reinterpret_cast<const char*>(write_buffer_.data()), write_buffer_pos_);
        output_file_->flush();
        
        updateStatistics(write_buffer_pos_, false);
        write_buffer_pos_ = 0;
    }
    
    xSemaphoreGive(file_mutex_);
    return ESP_OK;
}

esp_err_t TSMuxer::checkFileRotation() {
    if (!config_.enable_file_rotation) {
        return ESP_OK;
    }
    
    uint64_t current_size = getCurrentFileSize();
    uint64_t current_duration = getRecordingDuration();
    
    bool should_rotate = false;
    
    // Check size limit
    if (current_size >= config_.max_file_size_mb * 1024 * 1024) {
        ESP_LOGI(TAG, "File size limit reached: %llu MB", current_size / (1024 * 1024));
        should_rotate = true;
    }
    
    // Check duration limit
    if (current_duration >= config_.max_duration_sec * 1000) {
        ESP_LOGI(TAG, "File duration limit reached: %llu seconds", current_duration / 1000);
        should_rotate = true;
    }
    
    if (should_rotate) {
        return rotateFile();
    }
    
    return ESP_OK;
}

esp_err_t TSMuxer::rotateFile() {
    ESP_LOGI(TAG, "Rotating file");
    
    // Close current file
    closeOutputFile();
    
    // Create new file
    std::string new_filename = generateFilename();
    esp_err_t ret = createOutputFile(new_filename);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create new file for rotation");
        setState(MuxerState::ERROR, "File rotation failed");
        return ret;
    }
    
    // Reset file timing
    file_start_timestamp_ = getCurrentTimestamp();
    
    // Write initial packets for new file
    writePATPacket();
    writePMTPacket();
    
    return ESP_OK;
}

void TSMuxer::setState(MuxerState state, const std::string& message) {
    current_state_ = state;
    ESP_LOGI(TAG, "State changed to %d: %s", (int)state, message.c_str());
    
    if (state_callback_) {
        state_callback_(state, message);
    }
}

void TSMuxer::updateStatistics(size_t bytes_written, bool is_new_frame) {
    if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        statistics_.total_bytes_written += bytes_written;
        statistics_.current_file_size += bytes_written;
        
        if (is_new_frame) {
            statistics_.total_frames_written++;
            statistics_.current_file_frames++;
        }
        
        // Update durations
        uint64_t now = getCurrentTimestamp();
        if (recording_start_timestamp_ > 0) {
            statistics_.total_recording_time_ms = (now - recording_start_timestamp_) / 1000;
        }
        if (file_start_timestamp_ > 0) {
            statistics_.current_file_duration_ms = (now - file_start_timestamp_) / 1000;
        }
        
        // Update write speed (simplified)
        if (statistics_.total_recording_time_ms > 0) {
            statistics_.avg_write_speed_mbps = 
                (statistics_.total_bytes_written * 8.0 * 1000.0) / 
                (statistics_.total_recording_time_ms * 1024.0 * 1024.0);
        }
        
        xSemaphoreGive(stats_mutex_);
    }
}

uint64_t TSMuxer::getCurrentTimestamp() {
    return esp_timer_get_time();
}

bool TSMuxer::createDirectoryIfNotExists(const std::string& path) {
    struct stat st;
    if (stat(path.c_str(), &st) == 0) {
        return S_ISDIR(st.st_mode);
    }
    
    // Try to create directory
    if (mkdir(path.c_str(), 0755) == 0) {
        ESP_LOGI(TAG, "Created directory: %s", path.c_str());
        return true;
    }
    
    ESP_LOGE(TAG, "Failed to create directory: %s", path.c_str());
    return false;
}

std::string TSMuxer::getCurrentFilename() const {
    return current_filename_;
}

uint64_t TSMuxer::getCurrentFileSize() const {
    return statistics_.current_file_size;
}

uint64_t TSMuxer::getRecordingDuration() const {
    return statistics_.current_file_duration_ms;
}

TSMuxer::Statistics TSMuxer::getStatistics() const {
    Statistics stats = {};
    
    if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        stats = statistics_;
        xSemaphoreGive(stats_mutex_);
    }
    
    return stats;
}

void TSMuxer::resetStatistics() {
    if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        statistics_ = {};
        xSemaphoreGive(stats_mutex_);
    }
}