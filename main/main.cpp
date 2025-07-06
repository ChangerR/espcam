#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "wifi_provisioning.hpp"
#include "flash_storage.hpp"
#include "mqtt_client.hpp"
#include "camera_controller.hpp"
#include "network_camera.hpp"

static const char *TAG = "ESPCam";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "ESP32P4 Network Camera Starting...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    NetworkCamera camera;
    esp_err_t ret_init = camera.initialize();
    if (ret_init != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize camera: 0x%x", ret_init);
        return;
    }
    
    esp_err_t ret_start = camera.start();
    if (ret_start != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start camera: 0x%x", ret_start);
        return;
    }
    
    ESP_LOGI(TAG, "Network camera started successfully");
    
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}