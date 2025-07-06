#pragma once

#include <string>
#include "esp_err.h"
#include "nvs.h"

class FlashStorage {
public:
    FlashStorage();
    ~FlashStorage();

    esp_err_t initialize();
    
    bool saveWiFiCredentials(const std::string& ssid, const std::string& password);
    bool loadWiFiCredentials(std::string& ssid, std::string& password);
    bool clearWiFiCredentials();
    
    bool saveMQTTConfig(const std::string& broker_url, const std::string& username, const std::string& password);
    bool loadMQTTConfig(std::string& broker_url, std::string& username, std::string& password);
    
    bool saveDeviceConfig(const std::string& serial_number, const std::string& device_name);
    bool loadDeviceConfig(std::string& serial_number, std::string& device_name);
    
    bool saveString(const std::string& key, const std::string& value);
    bool loadString(const std::string& key, std::string& value);
    
    bool saveInt(const std::string& key, int32_t value);
    bool loadInt(const std::string& key, int32_t& value);
    
    bool eraseAll();

private:
    nvs_handle_t nvs_handle_;
    bool initialized_;
    
    static const char* NVS_NAMESPACE;
    static const char* WIFI_SSID_KEY;
    static const char* WIFI_PASSWORD_KEY;
    static const char* MQTT_BROKER_KEY;
    static const char* MQTT_USERNAME_KEY;
    static const char* MQTT_PASSWORD_KEY;
    static const char* DEVICE_SERIAL_KEY;
    static const char* DEVICE_NAME_KEY;
};