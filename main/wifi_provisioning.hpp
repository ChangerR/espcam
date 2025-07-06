#pragma once

#include <string>
#include <functional>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"

class WiFiProvisioning {
public:
    enum class ProvisioningState {
        IDLE,
        AP_STARTED,
        CREDENTIALS_RECEIVED,
        CONNECTING,
        CONNECTED,
        FAILED
    };

    using ProvisioningCallback = std::function<void(ProvisioningState state, const std::string& message)>;

    WiFiProvisioning();
    ~WiFiProvisioning();

    esp_err_t initialize();
    esp_err_t startProvisioning();
    esp_err_t stopProvisioning();
    
    void setCallback(ProvisioningCallback callback);
    
    bool isProvisioned();
    bool connectToSavedWiFi();
    
    ProvisioningState getState() const { return current_state_; }

private:
    esp_err_t startSoftAP();
    esp_err_t stopSoftAP();
    esp_err_t startWebServer();
    esp_err_t stopWebServer();
    
    static esp_err_t wifiConfigHandler(httpd_req_t *req);
    static esp_err_t rootHandler(httpd_req_t *req);
    static void wifiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
    
    void setState(ProvisioningState state, const std::string& message = "");
    esp_err_t connectToWiFi(const std::string& ssid, const std::string& password);

    httpd_handle_t server_;
    ProvisioningState current_state_;
    ProvisioningCallback callback_;
    std::string ap_ssid_;
    std::string ap_password_;
    
    static WiFiProvisioning* instance_;
};