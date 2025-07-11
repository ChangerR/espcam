#include "task_manager.hpp"
#include <cstring>

const char* TaskManager::TAG = "TaskManager";
TaskManager* TaskManager::instance_ = nullptr;

TaskManager::TaskManager() {
    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) {
        ESP_LOGE(TAG, "Failed to create mutex");
    }
    ESP_LOGI(TAG, "TaskManager initialized");
}

TaskManager::~TaskManager() {
    // Clean up all tasks
    for (auto& task_pair : tasks_) {
        if (task_pair.second.handle) {
            vTaskDelete(task_pair.second.handle);
        }
    }
    
    if (mutex_) {
        vSemaphoreDelete(mutex_);
    }
}

esp_err_t TaskManager::createTask(
    const std::string& name,
    TaskFunction task_function,
    uint32_t stack_size,
    UBaseType_t priority,
    void* parameters
) {
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_FAIL;
    }
    
    // Check if task already exists
    if (tasks_.find(name) != tasks_.end()) {
        ESP_LOGW(TAG, "Task '%s' already exists", name.c_str());
        xSemaphoreGive(mutex_);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Create task parameters
    TaskParams* task_params = new TaskParams{
        .function = task_function,
        .user_params = parameters,
        .task_name = name,
        .manager = this
    };
    
    TaskHandle_t task_handle = nullptr;
    BaseType_t result = xTaskCreate(
        taskWrapper,
        name.c_str(),
        stack_size,
        task_params,
        priority,
        &task_handle
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task '%s'", name.c_str());
        delete task_params;
        xSemaphoreGive(mutex_);
        return ESP_FAIL;
    }
    
    // Store task info
    TaskInfo task_info{
        .handle = task_handle,
        .name = name,
        .stack_size = stack_size,
        .priority = priority,
        .state = TaskState::CREATED,
        .last_wake_time = xTaskGetTickCount(),
        .runtime_counter = 0
    };
    
    tasks_[name] = task_info;
    updateTaskState(name, TaskState::RUNNING);
    
    ESP_LOGI(TAG, "Task '%s' created successfully", name.c_str());
    xSemaphoreGive(mutex_);
    return ESP_OK;
}

esp_err_t TaskManager::deleteTask(const std::string& name) {
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_FAIL;
    }
    
    auto it = tasks_.find(name);
    if (it == tasks_.end()) {
        ESP_LOGW(TAG, "Task '%s' not found", name.c_str());
        xSemaphoreGive(mutex_);
        return ESP_ERR_NOT_FOUND;
    }
    
    if (it->second.handle) {
        vTaskDelete(it->second.handle);
        updateTaskState(name, TaskState::DELETED);
        ESP_LOGI(TAG, "Task '%s' deleted", name.c_str());
    }
    
    tasks_.erase(it);
    xSemaphoreGive(mutex_);
    return ESP_OK;
}

esp_err_t TaskManager::suspendTask(const std::string& name) {
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_FAIL;
    }
    
    auto it = tasks_.find(name);
    if (it == tasks_.end()) {
        ESP_LOGW(TAG, "Task '%s' not found", name.c_str());
        xSemaphoreGive(mutex_);
        return ESP_ERR_NOT_FOUND;
    }
    
    if (it->second.handle) {
        vTaskSuspend(it->second.handle);
        updateTaskState(name, TaskState::SUSPENDED);
        ESP_LOGI(TAG, "Task '%s' suspended", name.c_str());
    }
    
    xSemaphoreGive(mutex_);
    return ESP_OK;
}

esp_err_t TaskManager::resumeTask(const std::string& name) {
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_FAIL;
    }
    
    auto it = tasks_.find(name);
    if (it == tasks_.end()) {
        ESP_LOGW(TAG, "Task '%s' not found", name.c_str());
        xSemaphoreGive(mutex_);
        return ESP_ERR_NOT_FOUND;
    }
    
    if (it->second.handle) {
        vTaskResume(it->second.handle);
        updateTaskState(name, TaskState::RUNNING);
        ESP_LOGI(TAG, "Task '%s' resumed", name.c_str());
    }
    
    xSemaphoreGive(mutex_);
    return ESP_OK;
}

TaskManager::TaskInfo TaskManager::getTaskInfo(const std::string& name) const {
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return TaskInfo{};
    }
    
    auto it = tasks_.find(name);
    TaskInfo info = (it != tasks_.end()) ? it->second : TaskInfo{};
    
    xSemaphoreGive(mutex_);
    return info;
}

std::map<std::string, TaskManager::TaskInfo> TaskManager::getAllTasks() const {
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return std::map<std::string, TaskInfo>{};
    }
    
    std::map<std::string, TaskInfo> tasks_copy = tasks_;
    xSemaphoreGive(mutex_);
    return tasks_copy;
}

void TaskManager::printTaskStatus() const {
    ESP_LOGI(TAG, "=== Task Status Report ===");
    
    auto tasks = getAllTasks();
    for (const auto& task_pair : tasks) {
        const TaskInfo& info = task_pair.second;
        ESP_LOGI(TAG, "Task: %s | State: %d | Priority: %d | Stack: %d | Watermark: %d",
                 info.name.c_str(),
                 (int)info.state,
                 info.priority,
                 info.stack_size,
                 getStackWatermark(info.name));
    }
    
    ESP_LOGI(TAG, "=========================");
}

void TaskManager::printMemoryUsage() const {
    ESP_LOGI(TAG, "=== Memory Usage Report ===");
    ESP_LOGI(TAG, "Free heap: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Minimum free heap: %d bytes", esp_get_minimum_free_heap_size());
    ESP_LOGI(TAG, "===========================");
}

uint32_t TaskManager::getStackWatermark(const std::string& name) const {
    auto it = tasks_.find(name);
    if (it != tasks_.end() && it->second.handle) {
        return uxTaskGetStackHighWaterMark(it->second.handle);
    }
    return 0;
}

void TaskManager::checkStackWatermarks() const {
    ESP_LOGI(TAG, "=== Stack Watermark Check ===");
    
    for (const auto& task_pair : tasks_) {
        const TaskInfo& info = task_pair.second;
        if (info.handle) {
            uint32_t watermark = uxTaskGetStackHighWaterMark(info.handle);
            uint32_t usage_percent = ((info.stack_size - watermark) * 100) / info.stack_size;
            
            ESP_LOGI(TAG, "Task '%s': %d/%d bytes used (%d%%)",
                     info.name.c_str(),
                     info.stack_size - watermark,
                     info.stack_size,
                     usage_percent);
            
            if (usage_percent > 80) {
                ESP_LOGW(TAG, "Task '%s' stack usage is high: %d%%", info.name.c_str(), usage_percent);
            }
        }
    }
    
    ESP_LOGI(TAG, "=============================");
}

void TaskManager::setTaskCallback(TaskCallback callback) {
    task_callback_ = callback;
}

TaskManager* TaskManager::getInstance() {
    if (!instance_) {
        instance_ = new TaskManager();
    }
    return instance_;
}

void TaskManager::taskWrapper(void* parameter) {
    TaskParams* params = static_cast<TaskParams*>(parameter);
    
    ESP_LOGI(TAG, "Task '%s' started", params->task_name.c_str());
    
    // Execute the actual task function
    if (params->function) {
        params->function(params->user_params);
    }
    
    // Update task state before deletion
    params->manager->updateTaskState(params->task_name, TaskState::DELETED);
    
    ESP_LOGI(TAG, "Task '%s' ended", params->task_name.c_str());
    
    // Clean up parameters
    delete params;
    
    // Delete self
    vTaskDelete(nullptr);
}

void TaskManager::updateTaskState(const std::string& name, TaskState state) {
    auto it = tasks_.find(name);
    if (it != tasks_.end()) {
        it->second.state = state;
        it->second.last_wake_time = xTaskGetTickCount();
        
        if (task_callback_) {
            task_callback_(name, state);
        }
    }
}