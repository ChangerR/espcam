#pragma once

#include <string>
#include <functional>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
// Note: ESP32P4 doesn't support standard wifi_provisioning APIs with hosted WiFi
// #include "wifi_provisioning/manager.h"
// #include "wifi_provisioning/scheme_softap.h"
#ifdef CONFIG_ESP_WIFI_REMOTE_LIBRARY_EPPP
#include "esp_wifi_remote.h"
#endif

class WiFiProvisioning {
public:
    enum class ProvisioningState {
        IDLE,
        STARTING,
        PROVISIONING,
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
    static void wifiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
    static void provisioningEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
    
    // HTTP server handlers
    esp_err_t startHttpServer();
    esp_err_t stopHttpServer();
    static esp_err_t httpGetHandler(httpd_req_t *req);
    static esp_err_t httpPostHandler(httpd_req_t *req);
    
    void setState(ProvisioningState state, const std::string& message = "");

    ProvisioningState current_state_;
    ProvisioningCallback callback_;
    std::string service_name_;
    std::string service_key_;
    httpd_handle_t http_server_;
    
    static WiFiProvisioning* instance_;
};