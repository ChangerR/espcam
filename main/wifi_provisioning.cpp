#include "wifi_provisioning.hpp"
#include "flash_storage.hpp"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include <cstring>

static const char *TAG = "WiFiProvisioning";

WiFiProvisioning* WiFiProvisioning::instance_ = nullptr;

WiFiProvisioning::WiFiProvisioning() 
    : server_(nullptr)
    , current_state_(ProvisioningState::IDLE)
    , ap_ssid_(CONFIG_ESPCAM_WIFI_AP_SSID)
    , ap_password_(CONFIG_ESPCAM_WIFI_AP_PASSWORD)
{
    instance_ = this;
}

WiFiProvisioning::~WiFiProvisioning() {
    stopProvisioning();
    instance_ = nullptr;
}

esp_err_t WiFiProvisioning::initialize() {
    ESP_LOGI(TAG, "Initializing WiFi provisioning");
    
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEventHandler, this));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifiEventHandler, this));
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    setState(ProvisioningState::IDLE);
    return ESP_OK;
}

esp_err_t WiFiProvisioning::startProvisioning() {
    ESP_LOGI(TAG, "Starting WiFi provisioning");
    
    ESP_ERROR_CHECK(startSoftAP());
    ESP_ERROR_CHECK(startWebServer());
    
    setState(ProvisioningState::AP_STARTED, "Access point started for provisioning");
    return ESP_OK;
}

esp_err_t WiFiProvisioning::stopProvisioning() {
    ESP_LOGI(TAG, "Stopping WiFi provisioning");
    
    stopWebServer();
    stopSoftAP();
    
    setState(ProvisioningState::IDLE);
    return ESP_OK;
}

esp_err_t WiFiProvisioning::startSoftAP() {
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    
    wifi_config_t ap_config = {};
    strcpy((char*)ap_config.ap.ssid, ap_ssid_.c_str());
    strcpy((char*)ap_config.ap.password, ap_password_.c_str());
    ap_config.ap.ssid_len = strlen((char*)ap_config.ap.ssid);
    ap_config.ap.channel = 1;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    
    if (strlen((char*)ap_config.ap.password) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    return ESP_OK;
}

esp_err_t WiFiProvisioning::stopSoftAP() {
    return esp_wifi_set_mode(WIFI_MODE_STA);
}

esp_err_t WiFiProvisioning::startWebServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    
    esp_err_t ret = httpd_start(&server_, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error starting server: %s", esp_err_to_name(ret));
        return ret;
    }
    
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = rootHandler,
        .user_ctx = this
    };
    httpd_register_uri_handler(server_, &root_uri);
    
    httpd_uri_t config_uri = {
        .uri = "/config",
        .method = HTTP_POST,
        .handler = wifiConfigHandler,
        .user_ctx = this
    };
    httpd_register_uri_handler(server_, &config_uri);
    
    ESP_LOGI(TAG, "Web server started on port %d", config.server_port);
    return ESP_OK;
}

esp_err_t WiFiProvisioning::stopWebServer() {
    if (server_) {
        esp_err_t ret = httpd_stop(server_);
        server_ = nullptr;
        return ret;
    }
    return ESP_OK;
}

esp_err_t WiFiProvisioning::rootHandler(httpd_req_t *req) {
    const char* resp_str = R"(
<!DOCTYPE html>
<html>
<head>
    <title>ESP Camera WiFi Setup</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 40px; }
        .container { max-width: 400px; margin: 0 auto; }
        input[type="text"], input[type="password"] { 
            width: 100%; padding: 12px; margin: 8px 0; 
            border: 1px solid #ddd; border-radius: 4px; 
        }
        input[type="submit"] { 
            background-color: #4CAF50; color: white; 
            padding: 14px 20px; border: none; border-radius: 4px; 
            cursor: pointer; width: 100%; 
        }
        input[type="submit"]:hover { background-color: #45a049; }
    </style>
</head>
<body>
    <div class="container">
        <h2>ESP Camera WiFi Configuration</h2>
        <form action="/config" method="post">
            <label>WiFi SSID:</label>
            <input type="text" name="ssid" required>
            <label>WiFi Password:</label>
            <input type="password" name="password" required>
            <input type="submit" value="Connect">
        </form>
    </div>
</body>
</html>
    )";
    
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t WiFiProvisioning::wifiConfigHandler(httpd_req_t *req) {
    WiFiProvisioning* self = static_cast<WiFiProvisioning*>(req->user_ctx);
    
    char content[256];
    size_t recv_size = std::min(req->content_len, sizeof(content) - 1);
    
    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (!json) {
        char ssid[64] = {0};
        char password[64] = {0};
        
        char* ssid_start = strstr(content, "ssid=");
        char* password_start = strstr(content, "password=");
        
        if (ssid_start && password_start) {
            ssid_start += 5;
            char* ssid_end = strchr(ssid_start, '&');
            if (ssid_end) {
                strncpy(ssid, ssid_start, std::min((int)(ssid_end - ssid_start), 63));
            }
            
            password_start += 9;
            strncpy(password, password_start, 63);
        }
        
        if (strlen(ssid) > 0) {
            self->setState(ProvisioningState::CREDENTIALS_RECEIVED, "WiFi credentials received");
            esp_err_t connect_result = self->connectToWiFi(ssid, password);
            
            const char* resp_str = (connect_result == ESP_OK) ? 
                "WiFi configuration saved. Device will restart." :
                "Failed to connect to WiFi. Please try again.";
            
            httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
            return connect_result;
        }
    } else {
        cJSON_Delete(json);
    }
    
    httpd_resp_send_500(req);
    return ESP_FAIL;
}

void WiFiProvisioning::wifiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    WiFiProvisioning* self = static_cast<WiFiProvisioning*>(arg);
    
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi station started");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WiFi disconnected, retrying...");
        self->setState(ProvisioningState::CONNECTING, "Attempting to connect...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        self->setState(ProvisioningState::CONNECTED, "WiFi connected successfully");
    }
}

esp_err_t WiFiProvisioning::connectToWiFi(const std::string& ssid, const std::string& password) {
    ESP_LOGI(TAG, "Connecting to WiFi: %s", ssid.c_str());
    
    FlashStorage storage;
    storage.saveWiFiCredentials(ssid, password);
    
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, ssid.c_str());
    strcpy((char*)wifi_config.sta.password, password.c_str());
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    setState(ProvisioningState::CONNECTING, "Connecting to WiFi...");
    return esp_wifi_connect();
}

bool WiFiProvisioning::isProvisioned() {
    FlashStorage storage;
    std::string ssid, password;
    return storage.loadWiFiCredentials(ssid, password);
}

bool WiFiProvisioning::connectToSavedWiFi() {
    FlashStorage storage;
    std::string ssid, password;
    
    if (storage.loadWiFiCredentials(ssid, password)) {
        ESP_LOGI(TAG, "Connecting to saved WiFi: %s", ssid.c_str());
        return connectToWiFi(ssid, password) == ESP_OK;
    }
    
    return false;
}

void WiFiProvisioning::setCallback(ProvisioningCallback callback) {
    callback_ = callback;
}

void WiFiProvisioning::setState(ProvisioningState state, const std::string& message) {
    current_state_ = state;
    ESP_LOGI(TAG, "State changed to %d: %s", (int)state, message.c_str());
    
    if (callback_) {
        callback_(state, message);
    }
}