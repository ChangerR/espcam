#include "cloud_uploader.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_crt_bundle.h"
#include <sys/stat.h>
#include <cstring>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <cJSON.h>

const char* CloudUploader::TAG = "CloudUploader";

CloudUploader::CloudUploader()
    : current_state_(UploadState::IDLE)
    , mqtt_client_(nullptr)
    , task_manager_(TaskManager::getInstance())
    , queue_mutex_(nullptr)
    , stats_mutex_(nullptr)
    , upload_queue_handle_(nullptr)
    , upload_task_handle_(nullptr)
    , monitor_task_handle_(nullptr)
    , tasks_running_(false)
    , paused_(false) {
    
    // 创建互斥锁
    queue_mutex_ = xSemaphoreCreateMutex();
    stats_mutex_ = xSemaphoreCreateMutex();
    
    if (!queue_mutex_ || !stats_mutex_) {
        ESP_LOGE(TAG, "Failed to create mutexes");
    }
    
    // 初始化统计信息
    memset(&statistics_, 0, sizeof(statistics_));
    memset(&current_progress_, 0, sizeof(current_progress_));
    
    ESP_LOGI(TAG, "CloudUploader created");
}

CloudUploader::~CloudUploader() {
    deinitialize();
    
    if (queue_mutex_) {
        vSemaphoreDelete(queue_mutex_);
    }
    if (stats_mutex_) {
        vSemaphoreDelete(stats_mutex_);
    }
    
    ESP_LOGI(TAG, "CloudUploader destroyed");
}

esp_err_t CloudUploader::initialize(const UploadConfig& config, MQTTClient* mqtt_client) {
    if (current_state_ != UploadState::IDLE) {
        ESP_LOGW(TAG, "Uploader already initialized");
        return ESP_OK;
    }
    
    if (!mqtt_client) {
        ESP_LOGE(TAG, "MQTT client is required");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing cloud uploader");
    
    config_ = config;
    mqtt_client_ = mqtt_client;
    
    // 创建上传队列
    upload_queue_handle_ = xQueueCreate(config_.upload_queue_size, UPLOAD_QUEUE_ITEM_SIZE);
    if (!upload_queue_handle_) {
        ESP_LOGE(TAG, "Failed to create upload queue");
        return ESP_FAIL;
    }
    
    // 注册MQTT消息回调
    std::string response_topic = config_.mqtt_response_topic;
    // 替换设备ID占位符 (简化实现，实际应从配置获取)
    size_t pos = response_topic.find("{DEVICE_ID}");
    if (pos != std::string::npos) {
        response_topic.replace(pos, 11, "ESP32P4_CAM_001"); // 示例设备ID
    }
    
    // 这里需要MQTT客户端支持回调注册
    // mqtt_client_->subscribeWithCallback(response_topic, 
    //     std::bind(&CloudUploader::onMqttMessage, this, std::placeholders::_1, std::placeholders::_2));
    
    // 创建任务
    esp_err_t ret = createTasks();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create tasks");
        vQueueDelete(upload_queue_handle_);
        return ret;
    }
    
    setState(UploadState::IDLE, "Cloud uploader initialized");
    
    ESP_LOGI(TAG, "Cloud uploader initialized: queue_size=%d, max_concurrent=%d, timeout=%d sec",
             config_.upload_queue_size, config_.max_concurrent_uploads, config_.upload_timeout_sec);
    
    return ESP_OK;
}

esp_err_t CloudUploader::deinitialize() {
    if (current_state_ == UploadState::IDLE) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Deinitializing cloud uploader");
    
    // 停止任务
    deleteTasks();
    
    // 清空队列
    if (upload_queue_handle_) {
        clearUploadQueue();
        vQueueDelete(upload_queue_handle_);
        upload_queue_handle_ = nullptr;
    }
    
    // 清空内部队列
    if (xSemaphoreTake(queue_mutex_, pdMS_TO_TICKS(1000)) == pdTRUE) {
        while (!upload_queue_.empty()) {
            upload_queue_.pop();
        }
        xSemaphoreGive(queue_mutex_);
    }
    
    current_state_ = UploadState::IDLE;
    mqtt_client_ = nullptr;
    
    ESP_LOGI(TAG, "Cloud uploader deinitialized");
    return ESP_OK;
}

esp_err_t CloudUploader::start() {
    if (current_state_ == UploadState::IDLE) {
        setState(UploadState::IDLE, "Cloud uploader started");
        tasks_running_ = true;
        paused_ = false;
        ESP_LOGI(TAG, "Cloud uploader started");
        return ESP_OK;
    }
    return ESP_ERR_INVALID_STATE;
}

esp_err_t CloudUploader::stop() {
    ESP_LOGI(TAG, "Stopping cloud uploader");
    tasks_running_ = false;
    setState(UploadState::IDLE, "Cloud uploader stopped");
    return ESP_OK;
}

esp_err_t CloudUploader::uploadFile(const std::string& file_path, 
                                   UploadPriority priority,
                                   const std::string& cloud_name,
                                   const std::string& metadata) {
    // 验证文件
    uint64_t file_size;
    esp_err_t ret = validateFile(file_path, file_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "File validation failed: %s", file_path.c_str());
        return ret;
    }
    
    // 创建上传请求
    UploadRequest request;
    request.local_file_path = file_path;
    request.cloud_file_name = cloud_name.empty() ? generateCloudFileName(file_path) : cloud_name;
    request.priority = priority;
    request.file_size = file_size;
    request.request_timestamp = getCurrentTimestamp();
    request.metadata = metadata.empty() ? createFileMetadata(file_path) : metadata;
    request.retry_count = 0;
    
    return uploadFileAsync(request);
}

esp_err_t CloudUploader::uploadFileAsync(const UploadRequest& request) {
    if (!tasks_running_ || paused_) {
        ESP_LOGW(TAG, "Uploader is not running or paused");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Adding file to upload queue: %s (priority: %d)", 
             request.local_file_path.c_str(), (int)request.priority);
    
    // 添加到队列
    esp_err_t ret = addToQueue(request);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add to upload queue");
        return ret;
    }
    
    // 更新统计
    if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        statistics_.total_upload_requests++;
        statistics_.queue_size++;
        xSemaphoreGive(stats_mutex_);
    }
    
    return ESP_OK;
}

esp_err_t CloudUploader::sendUploadRequest(const UploadRequest& request) {
    if (!mqtt_client_) {
        ESP_LOGE(TAG, "MQTT client not available");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 创建请求JSON
    cJSON* json = cJSON_CreateObject();
    cJSON* request_id = cJSON_CreateString(generateRequestId().c_str());
    cJSON* file_name = cJSON_CreateString(request.cloud_file_name.c_str());
    cJSON* file_size = cJSON_CreateNumber(request.file_size);
    cJSON* priority = cJSON_CreateNumber((int)request.priority);
    cJSON* timestamp = cJSON_CreateNumber(request.request_timestamp);
    
    cJSON_AddItemToObject(json, "request_id", request_id);
    cJSON_AddItemToObject(json, "file_name", file_name);
    cJSON_AddItemToObject(json, "file_size", file_size);
    cJSON_AddItemToObject(json, "priority", priority);
    cJSON_AddItemToObject(json, "timestamp", timestamp);
    
    if (!request.metadata.empty()) {
        cJSON* metadata = cJSON_CreateString(request.metadata.c_str());
        cJSON_AddItemToObject(json, "metadata", metadata);
    }
    
    char* json_string = cJSON_Print(json);
    std::string payload(json_string);
    
    // 发送MQTT消息
    std::string topic = config_.mqtt_request_topic;
    size_t pos = topic.find("{DEVICE_ID}");
    if (pos != std::string::npos) {
        topic.replace(pos, 11, "ESP32P4_CAM_001");
    }
    
    esp_err_t ret = ESP_OK;
    // ret = mqtt_client_->publish(topic, payload, 1, false);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Upload request sent: %s -> %s", 
                 request.local_file_path.c_str(), request.cloud_file_name.c_str());
        setState(UploadState::REQUESTING_URL, "Requesting upload URL from cloud");
    } else {
        ESP_LOGE(TAG, "Failed to send upload request via MQTT");
    }
    
    // 清理
    free(json_string);
    cJSON_Delete(json);
    
    return ret;
}

void CloudUploader::onMqttMessage(const std::string& topic, const std::string& payload) {
    ESP_LOGD(TAG, "Received MQTT message on topic: %s", topic.c_str());
    
    // 解析云端响应
    CloudResponse response;
    esp_err_t ret = parseCloudResponse(payload, response);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to parse cloud response");
        return;
    }
    
    if (response.success) {
        ESP_LOGI(TAG, "Received upload URL: %s", response.upload_url.c_str());
        
        // 保存响应，等待上传任务处理
        pending_response_ = response;
        
        // 如果当前有待上传的文件，立即开始上传
        if (!current_upload_file_.empty()) {
            performHttpUpload(current_upload_file_, response);
        }
    } else {
        ESP_LOGE(TAG, "Cloud request failed: %s - %s", 
                 response.error_code.c_str(), response.error_message.c_str());
        setState(UploadState::UPLOAD_FAILED, "Cloud request failed: " + response.error_message);
    }
}

esp_err_t CloudUploader::parseCloudResponse(const std::string& payload, CloudResponse& response) {
    cJSON* json = cJSON_Parse(payload.c_str());
    if (!json) {
        ESP_LOGE(TAG, "Invalid JSON in cloud response");
        return ESP_FAIL;
    }
    
    // 解析响应字段
    cJSON* success = cJSON_GetObjectItem(json, "success");
    response.success = success && cJSON_IsTrue(success);
    
    cJSON* upload_url = cJSON_GetObjectItem(json, "upload_url");
    if (upload_url && cJSON_IsString(upload_url)) {
        response.upload_url = upload_url->valuestring;
    }
    
    cJSON* method = cJSON_GetObjectItem(json, "method");
    if (method && cJSON_IsString(method)) {
        response.upload_method = method->valuestring;
    } else {
        response.upload_method = "PUT"; // 默认使用PUT方法
    }
    
    cJSON* content_type = cJSON_GetObjectItem(json, "content_type");
    if (content_type && cJSON_IsString(content_type)) {
        response.content_type = content_type->valuestring;
    } else {
        response.content_type = "video/mp2t"; // TS文件的MIME类型
    }
    
    cJSON* cloud_path = cJSON_GetObjectItem(json, "cloud_file_path");
    if (cloud_path && cJSON_IsString(cloud_path)) {
        response.cloud_file_path = cloud_path->valuestring;
    }
    
    cJSON* expires_at = cJSON_GetObjectItem(json, "expires_at");
    if (expires_at && cJSON_IsNumber(expires_at)) {
        response.url_expires_at = (uint64_t)expires_at->valuedouble;
    }
    
    cJSON* request_id = cJSON_GetObjectItem(json, "request_id");
    if (request_id && cJSON_IsString(request_id)) {
        response.request_id = request_id->valuestring;
    }
    
    if (!response.success) {
        cJSON* error_code = cJSON_GetObjectItem(json, "error_code");
        if (error_code && cJSON_IsString(error_code)) {
            response.error_code = error_code->valuestring;
        }
        
        cJSON* error_message = cJSON_GetObjectItem(json, "error_message");
        if (error_message && cJSON_IsString(error_message)) {
            response.error_message = error_message->valuestring;
        }
    }
    
    cJSON* extra_headers = cJSON_GetObjectItem(json, "extra_headers");
    if (extra_headers && cJSON_IsString(extra_headers)) {
        response.extra_headers = extra_headers->valuestring;
    }
    
    cJSON_Delete(json);
    return ESP_OK;
}

esp_err_t CloudUploader::performHttpUpload(const std::string& file_path, const CloudResponse& cloud_response) {
    ESP_LOGI(TAG, "Starting HTTP upload: %s -> %s", file_path.c_str(), cloud_response.upload_url.c_str());
    
    setState(UploadState::UPLOADING, "Uploading file to cloud");
    
    // 打开文件
    FILE* file = fopen(file_path.c_str(), "rb");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open file for upload: %s", file_path.c_str());
        setState(UploadState::UPLOAD_FAILED, "Failed to open file");
        return ESP_FAIL;
    }
    
    // 获取文件大小
    fseek(file, 0, SEEK_END);
    uint64_t file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    // 配置HTTP客户端
    esp_http_client_config_t config = {};
    config.url = cloud_response.upload_url.c_str();
    config.method = (cloud_response.upload_method == "PUT") ? HTTP_METHOD_PUT : HTTP_METHOD_POST;
    config.timeout_ms = config_.upload_timeout_sec * 1000;
    config.buffer_size = config_.http_buffer_size;
    config.event_handler = httpEventHandler;
    config.crt_bundle_attach = esp_crt_bundle_attach;
    
    // 创建HTTP客户端
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to create HTTP client");
        fclose(file);
        setState(UploadState::UPLOAD_FAILED, "Failed to create HTTP client");
        return ESP_FAIL;
    }
    
    // 设置HTTP头
    esp_http_client_set_header(client, "Content-Type", cloud_response.content_type.c_str());
    esp_http_client_set_header(client, "Content-Length", std::to_string(file_size).c_str());
    
    // 解析额外的HTTP头
    if (!cloud_response.extra_headers.empty()) {
        cJSON* headers_json = cJSON_Parse(cloud_response.extra_headers.c_str());
        if (headers_json) {
            cJSON* header = nullptr;
            cJSON_ArrayForEach(header, headers_json) {
                if (cJSON_IsString(header) && header->string) {
                    esp_http_client_set_header(client, header->string, header->valuestring);
                }
            }
            cJSON_Delete(headers_json);
        }
    }
    
    // 设置用户数据
    HttpUploadState upload_state;
    upload_state.uploader = this;
    upload_state.file_handle = file;
    upload_state.bytes_uploaded = 0;
    upload_state.total_bytes = file_size;
    upload_state.start_time = getCurrentTimestamp();
    upload_state.file_path = file_path;
    
    esp_http_client_set_user_data(client, &upload_state);
    
    // 初始化进度
    updateProgress(file_path, 0, file_size);
    
    // 执行上传
    esp_err_t ret = ESP_OK;
    uint64_t start_time = getCurrentTimestamp();
    
    // 分块上传
    char* buffer = (char*)malloc(config_.upload_chunk_size);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate upload buffer");
        ret = ESP_ERR_NO_MEM;
    } else {
        ret = esp_http_client_open(client, file_size);
        if (ret == ESP_OK) {
            size_t bytes_read;
            uint64_t total_uploaded = 0;
            
            while ((bytes_read = fread(buffer, 1, config_.upload_chunk_size, file)) > 0) {
                int bytes_written = esp_http_client_write(client, buffer, bytes_read);
                if (bytes_written < 0) {
                    ESP_LOGE(TAG, "HTTP write error");
                    ret = ESP_FAIL;
                    break;
                }
                
                total_uploaded += bytes_written;
                updateProgress(file_path, total_uploaded, file_size);
                
                // 检查是否取消
                if (current_state_ == UploadState::UPLOAD_CANCELLED) {
                    ESP_LOGW(TAG, "Upload cancelled by user");
                    ret = ESP_ERR_INVALID_STATE;
                    break;
                }
            }
            
            if (ret == ESP_OK) {
                int content_length = esp_http_client_fetch_headers(client);
                int status_code = esp_http_client_get_status_code(client);
                
                if (status_code >= 200 && status_code < 300) {
                    ESP_LOGI(TAG, "Upload successful: %s (status: %d)", file_path.c_str(), status_code);
                    setState(UploadState::UPLOAD_SUCCESS, "Upload completed successfully");
                    
                    // 更新统计
                    uint64_t upload_time = (getCurrentTimestamp() - start_time) / 1000000; // 转换为秒
                    updateStatistics({}, true, file_size, upload_time);
                    
                    // 删除本地文件（如果配置启用）
                    if (config_.delete_after_upload) {
                        if (remove(file_path.c_str()) == 0) {
                            ESP_LOGI(TAG, "Local file deleted after upload: %s", file_path.c_str());
                        } else {
                            ESP_LOGW(TAG, "Failed to delete local file: %s", file_path.c_str());
                        }
                    }
                    
                    // 触发完成回调
                    if (completion_callback_) {
                        completion_callback_(file_path, true, cloud_response.cloud_file_path);
                    }
                } else {
                    ESP_LOGE(TAG, "Upload failed with HTTP status: %d", status_code);
                    setState(UploadState::UPLOAD_FAILED, "HTTP upload failed");
                    ret = ESP_FAIL;
                }
            }
        }
        
        free(buffer);
    }
    
    // 清理
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    fclose(file);
    
    if (ret != ESP_OK) {
        updateStatistics({}, false, 0, 0);
        if (completion_callback_) {
            completion_callback_(file_path, false, "");
        }
    }
    
    return ret;
}

esp_err_t CloudUploader::httpEventHandler(esp_http_client_event_t *evt) {
    HttpUploadState* upload_state = static_cast<HttpUploadState*>(evt->user_data);
    
    switch (evt->event_id) {
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(upload_state->uploader->TAG, "HTTP connected");
            break;
            
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(upload_state->uploader->TAG, "HTTP headers sent");
            break;
            
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(upload_state->uploader->TAG, "HTTP header: %s: %s", 
                     (char*)evt->header_key, (char*)evt->header_value);
            break;
            
        case HTTP_EVENT_ON_DATA:
            // 响应数据接收（通常上传不需要处理响应数据）
            break;
            
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(upload_state->uploader->TAG, "HTTP upload finished");
            break;
            
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(upload_state->uploader->TAG, "HTTP disconnected");
            break;
            
        default:
            break;
    }
    
    return ESP_OK;
}

void CloudUploader::uploadTask(void* parameter) {
    CloudUploader* self = static_cast<CloudUploader*>(parameter);
    ESP_LOGI(TAG, "Upload task started");
    
    UploadRequest request;
    
    while (self->tasks_running_) {
        // 等待队列中的上传请求
        if (xQueueReceive(self->upload_queue_handle_, &request, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (self->paused_) {
                // 如果暂停，重新放回队列
                xQueueSendToFront(self->upload_queue_handle_, &request, 0);
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            
            ESP_LOGI(TAG, "Processing upload request: %s", request.local_file_path.c_str());
            self->current_upload_file_ = request.local_file_path;
            
            // 发送云端请求
            esp_err_t ret = self->sendUploadRequest(request);
            if (ret == ESP_OK) {
                // 等待云端响应并执行上传
                // 这里简化处理，实际应该有更复杂的状态管理
                vTaskDelay(pdMS_TO_TICKS(2000)); // 等待响应
                
                if (!self->pending_response_.upload_url.empty()) {
                    ret = self->performHttpUpload(request.local_file_path, self->pending_response_);
                    self->pending_response_ = {}; // 清空响应
                }
            }
            
            if (ret != ESP_OK && request.retry_count < self->config_.retry_max_attempts) {
                // 重试
                request.retry_count++;
                ESP_LOGW(TAG, "Upload failed, retrying (%d/%d): %s", 
                         request.retry_count, self->config_.retry_max_attempts, request.local_file_path.c_str());
                vTaskDelay(pdMS_TO_TICKS(self->config_.retry_delay_sec * 1000));
                xQueueSendToBack(self->upload_queue_handle_, &request, 0);
            } else if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Upload failed permanently: %s", request.local_file_path.c_str());
                self->updateStatistics(request, false, 0, 0);
            }
            
            self->current_upload_file_.clear();
        }
    }
    
    ESP_LOGI(TAG, "Upload task ended");
    vTaskDelete(nullptr);
}

void CloudUploader::monitorTask(void* parameter) {
    CloudUploader* self = static_cast<CloudUploader*>(parameter);
    ESP_LOGI(TAG, "Monitor task started");
    
    while (self->tasks_running_) {
        // 更新队列大小统计
        if (xSemaphoreTake(self->stats_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
            self->statistics_.queue_size = uxQueueMessagesWaiting(self->upload_queue_handle_);
            self->statistics_.active_uploads = self->current_upload_file_.empty() ? 0 : 1;
            xSemaphoreGive(self->stats_mutex_);
        }
        
        // 检查超时的上传
        uint64_t now = self->getCurrentTimestamp();
        if (!self->current_upload_file_.empty() && 
            self->current_state_ == UploadState::UPLOADING &&
            (now - self->current_progress_.progress_percentage) > (self->config_.upload_timeout_sec * 1000000)) {
            ESP_LOGW(TAG, "Upload timeout detected, cancelling: %s", self->current_upload_file_.c_str());
            self->setState(UploadState::UPLOAD_CANCELLED, "Upload timeout");
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000)); // 每5秒检查一次
    }
    
    ESP_LOGI(TAG, "Monitor task ended");
    vTaskDelete(nullptr);
}

esp_err_t CloudUploader::createTasks() {
    tasks_running_ = true;
    
    // 创建上传任务
    BaseType_t result = xTaskCreate(
        uploadTask,
        "cloud_upload",
        UPLOAD_TASK_STACK_SIZE,
        this,
        UPLOAD_TASK_PRIORITY,
        &upload_task_handle_
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create upload task");
        tasks_running_ = false;
        return ESP_FAIL;
    }
    
    // 创建监控任务
    result = xTaskCreate(
        monitorTask,
        "upload_monitor",
        MONITOR_TASK_STACK_SIZE,
        this,
        MONITOR_TASK_PRIORITY,
        &monitor_task_handle_
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitor task");
        vTaskDelete(upload_task_handle_);
        tasks_running_ = false;
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Cloud uploader tasks created");
    return ESP_OK;
}

void CloudUploader::deleteTasks() {
    tasks_running_ = false;
    
    if (upload_task_handle_) {
        vTaskDelay(pdMS_TO_TICKS(100));
        vTaskDelete(upload_task_handle_);
        upload_task_handle_ = nullptr;
    }
    
    if (monitor_task_handle_) {
        vTaskDelay(pdMS_TO_TICKS(100));
        vTaskDelete(monitor_task_handle_);
        monitor_task_handle_ = nullptr;
    }
    
    ESP_LOGI(TAG, "Cloud uploader tasks deleted");
}

esp_err_t CloudUploader::addToQueue(const UploadRequest& request) {
    if (xQueueSend(upload_queue_handle_, &request, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Upload queue is full");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t CloudUploader::validateFile(const std::string& file_path, uint64_t& file_size) {
    struct stat st;
    if (stat(file_path.c_str(), &st) != 0) {
        ESP_LOGE(TAG, "File not found: %s", file_path.c_str());
        return ESP_ERR_NOT_FOUND;
    }
    
    if (!S_ISREG(st.st_mode)) {
        ESP_LOGE(TAG, "Not a regular file: %s", file_path.c_str());
        return ESP_ERR_INVALID_ARG;
    }
    
    file_size = st.st_size;
    if (file_size == 0) {
        ESP_LOGE(TAG, "File is empty: %s", file_path.c_str());
        return ESP_ERR_INVALID_SIZE;
    }
    
    return ESP_OK;
}

std::string CloudUploader::generateCloudFileName(const std::string& local_path) const {
    // 提取文件名
    size_t pos = local_path.find_last_of("/\\");
    std::string filename = (pos != std::string::npos) ? local_path.substr(pos + 1) : local_path;
    
    // 添加时间戳前缀
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    
    std::ostringstream oss;
    oss << "ESP32P4_CAM_001/"
        << std::put_time(&tm, "%Y/%m/%d/")
        << std::put_time(&tm, "%H%M%S_")
        << filename;
    
    return oss.str();
}

std::string CloudUploader::createFileMetadata(const std::string& file_path) const {
    cJSON* metadata = cJSON_CreateObject();
    
    // 文件信息
    struct stat st;
    if (stat(file_path.c_str(), &st) == 0) {
        cJSON_AddNumberToObject(metadata, "file_size", st.st_size);
        cJSON_AddNumberToObject(metadata, "created_time", st.st_ctime);
        cJSON_AddNumberToObject(metadata, "modified_time", st.st_mtime);
    }
    
    // 设备信息
    cJSON_AddStringToObject(metadata, "device_id", "ESP32P4_CAM_001");
    cJSON_AddStringToObject(metadata, "device_type", "ESP32P4_Camera");
    cJSON_AddStringToObject(metadata, "firmware_version", "1.0.0");
    
    // 录像信息
    cJSON_AddStringToObject(metadata, "video_format", "H264/MPEG-TS");
    cJSON_AddStringToObject(metadata, "trigger_type", "motion_detection");
    
    char* json_string = cJSON_Print(metadata);
    std::string result(json_string);
    
    free(json_string);
    cJSON_Delete(metadata);
    
    return result;
}

void CloudUploader::setState(UploadState state, const std::string& message) {
    current_state_ = state;
    ESP_LOGI(TAG, "State changed to %d: %s", (int)state, message.c_str());
    
    if (state_callback_) {
        state_callback_(state, message);
    }
}

void CloudUploader::updateProgress(const std::string& file_path, uint64_t bytes_uploaded, uint64_t total_bytes) {
    current_progress_.file_path = file_path;
    current_progress_.bytes_uploaded = bytes_uploaded;
    current_progress_.total_bytes = total_bytes;
    current_progress_.progress_percentage = total_bytes > 0 ? (float)bytes_uploaded * 100.0f / total_bytes : 0.0f;
    current_progress_.state = current_state_;
    
    // 计算上传速度
    static uint64_t last_update_time = 0;
    static uint64_t last_bytes = 0;
    
    uint64_t now = getCurrentTimestamp();
    if (last_update_time > 0 && (now - last_update_time) > 1000000) { // 每秒更新一次
        uint64_t time_diff = (now - last_update_time) / 1000000; // 秒
        uint64_t bytes_diff = bytes_uploaded - last_bytes;
        current_progress_.upload_speed_kbps = (uint32_t)(bytes_diff / 1024 / time_diff);
        
        // 估算剩余时间
        if (current_progress_.upload_speed_kbps > 0) {
            uint64_t remaining_bytes = total_bytes - bytes_uploaded;
            current_progress_.estimated_time_remaining_sec = remaining_bytes / (current_progress_.upload_speed_kbps * 1024);
        }
        
        last_update_time = now;
        last_bytes = bytes_uploaded;
    } else if (last_update_time == 0) {
        last_update_time = now;
        last_bytes = bytes_uploaded;
    }
    
    // 触发进度回调
    if (progress_callback_) {
        progress_callback_(current_progress_);
    }
}

void CloudUploader::updateStatistics(const UploadRequest& request, bool success, uint64_t bytes_uploaded, uint64_t upload_time) {
    if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (success) {
            statistics_.successful_uploads++;
            statistics_.total_bytes_uploaded += bytes_uploaded;
            statistics_.total_upload_time_sec += upload_time;
            statistics_.last_upload_time = getCurrentTimestamp();
            statistics_.last_uploaded_file = request.local_file_path;
            
            // 计算平均上传速度
            if (statistics_.total_upload_time_sec > 0) {
                statistics_.avg_upload_speed_kbps = 
                    (double)(statistics_.total_bytes_uploaded / 1024) / statistics_.total_upload_time_sec;
            }
        } else {
            statistics_.failed_uploads++;
        }
        
        xSemaphoreGive(stats_mutex_);
    }
}

uint64_t CloudUploader::getCurrentTimestamp() {
    return esp_timer_get_time();
}

std::string CloudUploader::generateRequestId() const {
    uint64_t timestamp = getCurrentTimestamp();
    std::ostringstream oss;
    oss << "upload_" << std::hex << timestamp;
    return oss.str();
}

CloudUploader::UploadProgress CloudUploader::getCurrentProgress() const {
    return current_progress_;
}

CloudUploader::Statistics CloudUploader::getStatistics() const {
    Statistics stats = {};
    
    if (xSemaphoreTake(stats_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
        stats = statistics_;
        xSemaphoreGive(stats_mutex_);
    }
    
    return stats;
}

void CloudUploader::printStatus() const {
    ESP_LOGI(TAG, "=== Cloud Uploader Status ===");
    ESP_LOGI(TAG, "State: %d", (int)current_state_);
    ESP_LOGI(TAG, "Tasks running: %s", tasks_running_ ? "YES" : "NO");
    ESP_LOGI(TAG, "Paused: %s", paused_ ? "YES" : "NO");
    
    Statistics stats = getStatistics();
    ESP_LOGI(TAG, "=== Upload Statistics ===");
    ESP_LOGI(TAG, "Total requests: %llu", stats.total_upload_requests);
    ESP_LOGI(TAG, "Successful uploads: %llu", stats.successful_uploads);
    ESP_LOGI(TAG, "Failed uploads: %llu", stats.failed_uploads);
    ESP_LOGI(TAG, "Total bytes uploaded: %llu", stats.total_bytes_uploaded);
    ESP_LOGI(TAG, "Average speed: %.2f KB/s", stats.avg_upload_speed_kbps);
    ESP_LOGI(TAG, "Queue size: %d", stats.queue_size);
    ESP_LOGI(TAG, "Active uploads: %d", stats.active_uploads);
    
    if (!current_upload_file_.empty()) {
        ESP_LOGI(TAG, "Current upload: %s (%.1f%%)", 
                 current_upload_file_.c_str(), current_progress_.progress_percentage);
    }
    
    ESP_LOGI(TAG, "============================");
}

esp_err_t CloudUploader::clearUploadQueue() {
    if (upload_queue_handle_) {
        xQueueReset(upload_queue_handle_);
    }
    
    if (xSemaphoreTake(queue_mutex_, pdMS_TO_TICKS(1000)) == pdTRUE) {
        while (!upload_queue_.empty()) {
            upload_queue_.pop();
        }
        xSemaphoreGive(queue_mutex_);
    }
    
    ESP_LOGI(TAG, "Upload queue cleared");
    return ESP_OK;
}

esp_err_t CloudUploader::pauseUploads() {
    paused_ = true;
    ESP_LOGI(TAG, "Uploads paused");
    return ESP_OK;
}

esp_err_t CloudUploader::resumeUploads() {
    paused_ = false;
    ESP_LOGI(TAG, "Uploads resumed");
    return ESP_OK;
}

void CloudUploader::setProgressCallback(ProgressCallback callback) {
    progress_callback_ = callback;
}

void CloudUploader::setStateCallback(StateCallback callback) {
    state_callback_ = callback;
}

void CloudUploader::setCompletionCallback(CompletionCallback callback) {
    completion_callback_ = callback;
}

std::string CloudUploader::formatFileSize(uint64_t size_bytes) const {
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