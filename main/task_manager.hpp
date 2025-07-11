#pragma once

#include <map>
#include <string>
#include <functional>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"

class TaskManager {
public:
    enum class TaskState {
        CREATED,
        RUNNING,
        SUSPENDED,
        DELETED,
        ERROR
    };

    struct TaskInfo {
        TaskHandle_t handle;
        std::string name;
        uint32_t stack_size;
        UBaseType_t priority;
        TaskState state;
        TickType_t last_wake_time;
        uint32_t runtime_counter;
    };

    using TaskFunction = std::function<void(void*)>;
    using TaskCallback = std::function<void(const std::string& task_name, TaskState state)>;

    TaskManager();
    ~TaskManager();

    // Task management
    esp_err_t createTask(
        const std::string& name,
        TaskFunction task_function,
        uint32_t stack_size,
        UBaseType_t priority,
        void* parameters = nullptr
    );
    
    esp_err_t deleteTask(const std::string& name);
    esp_err_t suspendTask(const std::string& name);
    esp_err_t resumeTask(const std::string& name);
    
    // Task monitoring
    TaskInfo getTaskInfo(const std::string& name) const;
    std::map<std::string, TaskInfo> getAllTasks() const;
    
    // System monitoring
    void printTaskStatus() const;
    void printMemoryUsage() const;
    
    // Stack watermark monitoring
    uint32_t getStackWatermark(const std::string& name) const;
    void checkStackWatermarks() const;
    
    // Callback management
    void setTaskCallback(TaskCallback callback);
    
    // Singleton access
    static TaskManager* getInstance();
    
private:
    static void taskWrapper(void* parameter);
    void updateTaskState(const std::string& name, TaskState state);
    
    std::map<std::string, TaskInfo> tasks_;
    TaskCallback task_callback_;
    SemaphoreHandle_t mutex_;
    
    static TaskManager* instance_;
    static const char* TAG;
};

// Task parameter structure for wrapper function
struct TaskParams {
    TaskManager::TaskFunction function;
    void* user_params;
    std::string task_name;
    TaskManager* manager;
};