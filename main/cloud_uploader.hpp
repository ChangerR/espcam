#pragma once

#include <string>
#include <functional>
#include <memory>
#include <vector>
#include <queue>
#include "esp_err.h"
#include "esp_http_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "task_manager.hpp"
#include "mqtt_client.hpp"

// 云端录像上传管理器
// 通过MQTT请求云端OSS上传链接，实现录像文件云端存储
class CloudUploader {
public:
    enum class UploadState {
        IDLE,              // 空闲状态
        REQUESTING_URL,    // 请求上传URL中
        UPLOADING,         // 上传中
        UPLOAD_SUCCESS,    // 上传成功
        UPLOAD_FAILED,     // 上传失败
        UPLOAD_CANCELLED   // 上传取消
    };
    
    enum class UploadPriority {
        LOW = 0,           // 低优先级 (历史录像)
        NORMAL = 1,        // 普通优先级 (常规录像)
        HIGH = 2,          // 高优先级 (运动触发录像)
        URGENT = 3         // 紧急优先级 (告警录像)
    };
    
    struct UploadConfig {
        // MQTT配置
        std::string mqtt_request_topic = "devices/{DEVICE_ID}/upload/request";
        std::string mqtt_response_topic = "devices/{DEVICE_ID}/upload/response";
        std::string mqtt_status_topic = "devices/{DEVICE_ID}/upload/status";
        
        // 上传配置
        uint32_t max_concurrent_uploads = 2;        // 最大并发上传数
        uint32_t upload_timeout_sec = 300;          // 上传超时时间(秒)
        uint32_t request_timeout_sec = 30;          // 请求URL超时时间(秒)
        uint32_t retry_max_attempts = 3;            // 最大重试次数
        uint32_t retry_delay_sec = 60;              // 重试延迟时间(秒)
        
        // 文件配置
        bool delete_after_upload = true;            // 上传后删除本地文件
        uint32_t upload_chunk_size = 8192;          // 上传块大小(字节)
        std::string temp_suffix = ".uploading";     // 上传中临时文件后缀
        
        // 队列配置
        uint32_t upload_queue_size = 50;            // 上传队列大小
        bool auto_upload_new_recordings = true;     // 自动上传新录像
        UploadPriority default_priority = UploadPriority::NORMAL;
        
        // 网络配置
        uint32_t http_buffer_size = 4096;           // HTTP缓冲区大小
        uint32_t connect_timeout_ms = 10000;        // 连接超时(毫秒)
        uint32_t read_timeout_ms = 30000;           // 读取超时(毫秒)
    };
    
    struct UploadRequest {
        std::string local_file_path;               // 本地文件路径
        std::string cloud_file_name;               // 云端文件名
        UploadPriority priority;                   // 上传优先级
        uint64_t file_size;                        // 文件大小
        uint64_t request_timestamp;                // 请求时间戳
        std::string metadata;                      // 元数据(JSON格式)
        uint32_t retry_count;                      // 重试次数
        
        UploadRequest() : priority(UploadPriority::NORMAL), file_size(0), 
                         request_timestamp(0), retry_count(0) {}
    };
    
    struct UploadProgress {
        std::string file_path;                     // 文件路径
        uint64_t bytes_uploaded;                   // 已上传字节数
        uint64_t total_bytes;                      // 总字节数
        float progress_percentage;                 // 上传进度百分比
        uint32_t upload_speed_kbps;                // 上传速度(KB/s)
        uint64_t estimated_time_remaining_sec;     // 预计剩余时间(秒)
        UploadState state;                         // 上传状态
        std::string error_message;                 // 错误信息
    };
    
    struct CloudResponse {
        bool success;                              // 请求是否成功
        std::string upload_url;                    // OSS上传URL
        std::string upload_method;                 // HTTP方法(通常是PUT)
        std::string content_type;                  // 内容类型
        std::string cloud_file_path;               // 云端文件路径
        uint64_t url_expires_at;                   // URL过期时间戳
        std::string request_id;                    // 请求ID
        std::string error_code;                    // 错误代码
        std::string error_message;                 // 错误信息
        std::string extra_headers;                 // 额外HTTP头(JSON格式)
    };
    
    struct Statistics {
        uint64_t total_upload_requests = 0;        // 总上传请求数
        uint64_t successful_uploads = 0;           // 成功上传数
        uint64_t failed_uploads = 0;               // 失败上传数
        uint64_t total_bytes_uploaded = 0;         // 总上传字节数
        uint64_t total_upload_time_sec = 0;        // 总上传时间(秒)
        double avg_upload_speed_kbps = 0.0;        // 平均上传速度(KB/s)
        uint32_t active_uploads = 0;               // 当前活跃上传数
        uint32_t queue_size = 0;                   // 队列大小
        uint64_t last_upload_time = 0;             // 最后上传时间
        std::string last_uploaded_file;            // 最后上传的文件
    };
    
    using ProgressCallback = std::function<void(const UploadProgress& progress)>;
    using StateCallback = std::function<void(UploadState state, const std::string& message)>;
    using CompletionCallback = std::function<void(const std::string& file_path, bool success, const std::string& cloud_url)>;
    
    CloudUploader();
    ~CloudUploader();
    
    // 生命周期管理
    esp_err_t initialize(const UploadConfig& config, MQTTClient* mqtt_client);
    esp_err_t deinitialize();
    esp_err_t start();
    esp_err_t stop();
    
    // 上传管理
    esp_err_t uploadFile(const std::string& file_path, 
                        UploadPriority priority = UploadPriority::NORMAL,
                        const std::string& cloud_name = "",
                        const std::string& metadata = "");
    esp_err_t uploadFileAsync(const UploadRequest& request);
    esp_err_t cancelUpload(const std::string& file_path);
    esp_err_t cancelAllUploads();
    
    // 队列管理
    esp_err_t clearUploadQueue();
    esp_err_t pauseUploads();
    esp_err_t resumeUploads();
    std::vector<UploadRequest> getUploadQueue() const;
    
    // 状态查询
    UploadState getState() const { return current_state_; }
    bool isUploading() const { return current_state_ == UploadState::UPLOADING; }
    bool isIdle() const { return current_state_ == UploadState::IDLE; }
    UploadProgress getCurrentProgress() const;
    Statistics getStatistics() const;
    
    // 配置管理
    esp_err_t updateConfig(const UploadConfig& config);
    UploadConfig getConfig() const { return config_; }
    esp_err_t setAutoUpload(bool enable);
    esp_err_t setDefaultPriority(UploadPriority priority);
    
    // 回调设置
    void setProgressCallback(ProgressCallback callback);
    void setStateCallback(StateCallback callback);
    void setCompletionCallback(CompletionCallback callback);
    
    // 调试和监控
    void printStatus() const;
    std::vector<std::string> getUploadHistory(uint32_t count = 10) const;
    esp_err_t exportUploadLog(const std::string& log_file_path) const;
    
    // 手动触发功能
    esp_err_t requestCloudUrl(const std::string& file_path, const std::string& metadata = "");
    esp_err_t uploadWithUrl(const std::string& file_path, const CloudResponse& cloud_response);

private:
    // MQTT消息处理
    void onMqttMessage(const std::string& topic, const std::string& payload);
    esp_err_t sendUploadRequest(const UploadRequest& request);
    esp_err_t parseCloudResponse(const std::string& payload, CloudResponse& response);
    
    // HTTP上传处理
    esp_err_t performHttpUpload(const std::string& file_path, const CloudResponse& cloud_response);
    static esp_err_t httpEventHandler(esp_http_client_event_t *evt);
    esp_err_t setupHttpClient(esp_http_client_handle_t client, const CloudResponse& cloud_response);
    
    // 文件处理
    esp_err_t validateFile(const std::string& file_path, uint64_t& file_size);
    std::string generateCloudFileName(const std::string& local_path) const;
    std::string createFileMetadata(const std::string& file_path) const;
    esp_err_t moveToTempFile(const std::string& file_path, std::string& temp_path);
    esp_err_t cleanupTempFile(const std::string& temp_path);
    
    // 队列管理
    esp_err_t addToQueue(const UploadRequest& request);
    bool getNextUpload(UploadRequest& request);
    void sortUploadQueue();
    esp_err_t retryFailedUpload(const UploadRequest& request);
    
    // 任务管理
    static void uploadTask(void* parameter);
    static void monitorTask(void* parameter);
    esp_err_t createTasks();
    void deleteTasks();
    
    // 状态管理
    void setState(UploadState state, const std::string& message = "");
    void updateProgress(const std::string& file_path, uint64_t bytes_uploaded, uint64_t total_bytes);
    void updateStatistics(const UploadRequest& request, bool success, uint64_t bytes_uploaded, uint64_t upload_time);
    
    // 实用工具
    uint64_t getCurrentTimestamp();
    std::string formatFileSize(uint64_t size_bytes) const;
    std::string formatDuration(uint64_t seconds) const;
    std::string generateRequestId() const;
    void logError(const std::string& error);
    
    // 配置和状态
    UploadConfig config_;
    UploadState current_state_;
    MQTTClient* mqtt_client_;
    TaskManager* task_manager_;
    
    // 队列和同步
    std::queue<UploadRequest> upload_queue_;
    mutable SemaphoreHandle_t queue_mutex_;
    mutable SemaphoreHandle_t stats_mutex_;
    QueueHandle_t upload_queue_handle_;
    
    // 任务句柄
    TaskHandle_t upload_task_handle_;
    TaskHandle_t monitor_task_handle_;
    volatile bool tasks_running_;
    
    // 当前上传状态
    UploadProgress current_progress_;
    std::string current_upload_file_;
    CloudResponse pending_response_;
    bool paused_;
    
    // 统计信息
    Statistics statistics_;
    std::vector<std::string> upload_history_;
    
    // HTTP客户端状态
    struct HttpUploadState {
        CloudUploader* uploader;
        FILE* file_handle;
        uint64_t bytes_uploaded;
        uint64_t total_bytes;
        uint64_t start_time;
        std::string file_path;
    };
    
    // 任务配置
    static const uint32_t UPLOAD_TASK_STACK_SIZE = 8192;
    static const UBaseType_t UPLOAD_TASK_PRIORITY = 4;
    static const uint32_t MONITOR_TASK_STACK_SIZE = 4096;
    static const UBaseType_t MONITOR_TASK_PRIORITY = 2;
    static const uint32_t UPLOAD_QUEUE_ITEM_SIZE = sizeof(UploadRequest);
    
    static const char* TAG;
};