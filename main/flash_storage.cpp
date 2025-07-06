#include "flash_storage.hpp"
#include "esp_log.h"
#include "nvs_flash.h"
#include <cstring>

static const char *TAG = "FlashStorage";

const char* FlashStorage::NVS_NAMESPACE = "espcam_config";
const char* FlashStorage::WIFI_SSID_KEY = "wifi_ssid";
const char* FlashStorage::WIFI_PASSWORD_KEY = "wifi_password";
const char* FlashStorage::MQTT_BROKER_KEY = "mqtt_broker";
const char* FlashStorage::MQTT_USERNAME_KEY = "mqtt_username";
const char* FlashStorage::MQTT_PASSWORD_KEY = "mqtt_password";
const char* FlashStorage::DEVICE_SERIAL_KEY = "device_serial";
const char* FlashStorage::DEVICE_NAME_KEY = "device_name";

FlashStorage::FlashStorage() : nvs_handle_(0), initialized_(false) {
}

FlashStorage::~FlashStorage() {
    if (initialized_) {
        nvs_close(nvs_handle_);
    }
}

esp_err_t FlashStorage::initialize() {
    if (initialized_) {
        return ESP_OK;
    }
    
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }
    
    initialized_ = true;
    ESP_LOGI(TAG, "Flash storage initialized");
    return ESP_OK;
}

bool FlashStorage::saveWiFiCredentials(const std::string& ssid, const std::string& password) {
    if (!initialized_ && initialize() != ESP_OK) {
        return false;
    }
    
    esp_err_t err1 = nvs_set_str(nvs_handle_, WIFI_SSID_KEY, ssid.c_str());
    esp_err_t err2 = nvs_set_str(nvs_handle_, WIFI_PASSWORD_KEY, password.c_str());
    esp_err_t err3 = nvs_commit(nvs_handle_);
    
    if (err1 != ESP_OK || err2 != ESP_OK || err3 != ESP_OK) {
        ESP_LOGE(TAG, "Error saving WiFi credentials");
        return false;
    }
    
    ESP_LOGI(TAG, "WiFi credentials saved to flash");
    return true;
}

bool FlashStorage::loadWiFiCredentials(std::string& ssid, std::string& password) {
    if (!initialized_ && initialize() != ESP_OK) {
        return false;
    }
    
    char ssid_buf[64];
    char password_buf[64];
    size_t ssid_len = sizeof(ssid_buf);
    size_t password_len = sizeof(password_buf);
    
    esp_err_t err1 = nvs_get_str(nvs_handle_, WIFI_SSID_KEY, ssid_buf, &ssid_len);
    esp_err_t err2 = nvs_get_str(nvs_handle_, WIFI_PASSWORD_KEY, password_buf, &password_len);
    
    if (err1 == ESP_OK && err2 == ESP_OK) {
        ssid = std::string(ssid_buf);
        password = std::string(password_buf);
        ESP_LOGI(TAG, "WiFi credentials loaded from flash");
        return true;
    }
    
    ESP_LOGW(TAG, "No WiFi credentials found in flash");
    return false;
}

bool FlashStorage::clearWiFiCredentials() {
    if (!initialized_ && initialize() != ESP_OK) {
        return false;
    }
    
    esp_err_t err1 = nvs_erase_key(nvs_handle_, WIFI_SSID_KEY);
    esp_err_t err2 = nvs_erase_key(nvs_handle_, WIFI_PASSWORD_KEY);
    esp_err_t err3 = nvs_commit(nvs_handle_);
    
    if (err1 == ESP_OK && err2 == ESP_OK && err3 == ESP_OK) {
        ESP_LOGI(TAG, "WiFi credentials cleared from flash");
        return true;
    }
    
    ESP_LOGE(TAG, "Error clearing WiFi credentials");
    return false;
}

bool FlashStorage::saveMQTTConfig(const std::string& broker_url, const std::string& username, const std::string& password) {
    if (!initialized_ && initialize() != ESP_OK) {
        return false;
    }
    
    esp_err_t err1 = nvs_set_str(nvs_handle_, MQTT_BROKER_KEY, broker_url.c_str());
    esp_err_t err2 = nvs_set_str(nvs_handle_, MQTT_USERNAME_KEY, username.c_str());
    esp_err_t err3 = nvs_set_str(nvs_handle_, MQTT_PASSWORD_KEY, password.c_str());
    esp_err_t err4 = nvs_commit(nvs_handle_);
    
    if (err1 != ESP_OK || err2 != ESP_OK || err3 != ESP_OK || err4 != ESP_OK) {
        ESP_LOGE(TAG, "Error saving MQTT config");
        return false;
    }
    
    ESP_LOGI(TAG, "MQTT config saved to flash");
    return true;
}

bool FlashStorage::loadMQTTConfig(std::string& broker_url, std::string& username, std::string& password) {
    if (!initialized_ && initialize() != ESP_OK) {
        return false;
    }
    
    char broker_buf[256];
    char username_buf[64];
    char password_buf[64];
    size_t broker_len = sizeof(broker_buf);
    size_t username_len = sizeof(username_buf);
    size_t password_len = sizeof(password_buf);
    
    esp_err_t err1 = nvs_get_str(nvs_handle_, MQTT_BROKER_KEY, broker_buf, &broker_len);
    esp_err_t err2 = nvs_get_str(nvs_handle_, MQTT_USERNAME_KEY, username_buf, &username_len);
    esp_err_t err3 = nvs_get_str(nvs_handle_, MQTT_PASSWORD_KEY, password_buf, &password_len);
    
    if (err1 == ESP_OK && err2 == ESP_OK && err3 == ESP_OK) {
        broker_url = std::string(broker_buf);
        username = std::string(username_buf);
        password = std::string(password_buf);
        ESP_LOGI(TAG, "MQTT config loaded from flash");
        return true;
    }
    
    ESP_LOGW(TAG, "No MQTT config found in flash");
    return false;
}

bool FlashStorage::saveDeviceConfig(const std::string& serial_number, const std::string& device_name) {
    if (!initialized_ && initialize() != ESP_OK) {
        return false;
    }
    
    esp_err_t err1 = nvs_set_str(nvs_handle_, DEVICE_SERIAL_KEY, serial_number.c_str());
    esp_err_t err2 = nvs_set_str(nvs_handle_, DEVICE_NAME_KEY, device_name.c_str());
    esp_err_t err3 = nvs_commit(nvs_handle_);
    
    if (err1 != ESP_OK || err2 != ESP_OK || err3 != ESP_OK) {
        ESP_LOGE(TAG, "Error saving device config");
        return false;
    }
    
    ESP_LOGI(TAG, "Device config saved to flash");
    return true;
}

bool FlashStorage::loadDeviceConfig(std::string& serial_number, std::string& device_name) {
    if (!initialized_ && initialize() != ESP_OK) {
        return false;
    }
    
    char serial_buf[64];
    char name_buf[64];
    size_t serial_len = sizeof(serial_buf);
    size_t name_len = sizeof(name_buf);
    
    esp_err_t err1 = nvs_get_str(nvs_handle_, DEVICE_SERIAL_KEY, serial_buf, &serial_len);
    esp_err_t err2 = nvs_get_str(nvs_handle_, DEVICE_NAME_KEY, name_buf, &name_len);
    
    if (err1 == ESP_OK && err2 == ESP_OK) {
        serial_number = std::string(serial_buf);
        device_name = std::string(name_buf);
        ESP_LOGI(TAG, "Device config loaded from flash");
        return true;
    }
    
    ESP_LOGW(TAG, "No device config found in flash");
    return false;
}

bool FlashStorage::saveString(const std::string& key, const std::string& value) {
    if (!initialized_ && initialize() != ESP_OK) {
        return false;
    }
    
    esp_err_t err1 = nvs_set_str(nvs_handle_, key.c_str(), value.c_str());
    esp_err_t err2 = nvs_commit(nvs_handle_);
    
    if (err1 != ESP_OK || err2 != ESP_OK) {
        ESP_LOGE(TAG, "Error saving string key: %s", key.c_str());
        return false;
    }
    
    return true;
}

bool FlashStorage::loadString(const std::string& key, std::string& value) {
    if (!initialized_ && initialize() != ESP_OK) {
        return false;
    }
    
    size_t required_size = 0;
    esp_err_t err = nvs_get_str(nvs_handle_, key.c_str(), nullptr, &required_size);
    
    if (err == ESP_OK && required_size > 0) {
        char* buffer = new char[required_size];
        err = nvs_get_str(nvs_handle_, key.c_str(), buffer, &required_size);
        
        if (err == ESP_OK) {
            value = std::string(buffer);
            delete[] buffer;
            return true;
        }
        delete[] buffer;
    }
    
    return false;
}

bool FlashStorage::saveInt(const std::string& key, int32_t value) {
    if (!initialized_ && initialize() != ESP_OK) {
        return false;
    }
    
    esp_err_t err1 = nvs_set_i32(nvs_handle_, key.c_str(), value);
    esp_err_t err2 = nvs_commit(nvs_handle_);
    
    return (err1 == ESP_OK && err2 == ESP_OK);
}

bool FlashStorage::loadInt(const std::string& key, int32_t& value) {
    if (!initialized_ && initialize() != ESP_OK) {
        return false;
    }
    
    esp_err_t err = nvs_get_i32(nvs_handle_, key.c_str(), &value);
    return (err == ESP_OK);
}

bool FlashStorage::eraseAll() {
    if (!initialized_ && initialize() != ESP_OK) {
        return false;
    }
    
    esp_err_t err1 = nvs_erase_all(nvs_handle_);
    esp_err_t err2 = nvs_commit(nvs_handle_);
    
    if (err1 == ESP_OK && err2 == ESP_OK) {
        ESP_LOGI(TAG, "All data erased from flash");
        return true;
    }
    
    ESP_LOGE(TAG, "Error erasing flash data");
    return false;
}