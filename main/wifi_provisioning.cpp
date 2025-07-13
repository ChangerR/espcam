#include "wifi_provisioning.hpp"
#include "flash_storage.hpp"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include <cstring>

static const char *TAG = "WiFiProvisioning";

WiFiProvisioning* WiFiProvisioning::instance_ = nullptr;

WiFiProvisioning::WiFiProvisioning() 
    : current_state_(ProvisioningState::IDLE)
    , service_name_("PROV_" + std::string(CONFIG_ESPCAM_DEVICE_SERIAL))
    , service_key_("abcd1234")  // Default provisioning key
    , http_server_(nullptr)
{
    instance_ = this;
}

WiFiProvisioning::~WiFiProvisioning() {
    stopProvisioning();
    stopHttpServer();
    instance_ = nullptr;
}

esp_err_t WiFiProvisioning::initialize() {
    ESP_LOGI(TAG, "Initializing WiFi provisioning with ESP-IDF manager");
    
    // Initialize WiFi including netif with default config
    // Note: esp_netif_init() and esp_event_loop_create_default() should be called in main()
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    
    // Initialize WiFi - use remote API for ESP32P4+ESP32C6 configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
#ifdef CONFIG_ESP_WIFI_REMOTE_LIBRARY_EPPP
    ESP_LOGI(TAG, "Initializing ESP WiFi Remote for ESP32P4+ESP32C6");
    ESP_ERROR_CHECK(esp_wifi_remote_init(&cfg));
#else
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
#endif
    
    // Register event handlers - use appropriate events for WiFi Remote
#ifdef CONFIG_ESP_WIFI_REMOTE_LIBRARY_EPPP
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_REMOTE_EVENT, ESP_EVENT_ANY_ID, 
                                              &wifiEventHandler, this));
#else
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, 
                                              &wifiEventHandler, this));
#endif
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, 
                                              &wifiEventHandler, this));
    // Note: WIFI_PROV_EVENT not available for ESP32P4 hosted WiFi
    
    setState(ProvisioningState::IDLE);
    ESP_LOGI(TAG, "WiFi provisioning initialized successfully");
    
    return ESP_OK;
}

esp_err_t WiFiProvisioning::startProvisioning() {
    ESP_LOGI(TAG, "Starting WiFi provisioning with SoftAP scheme");
    
    if (current_state_ != ProvisioningState::IDLE) {
        ESP_LOGW(TAG, "Provisioning already in progress");
        return ESP_ERR_INVALID_STATE;
    }
    
    setState(ProvisioningState::STARTING);
    
    // Check if device is already provisioned by looking for stored credentials
    FlashStorage storage;
    storage.initialize();
    std::string ssid, password;
    bool provisioned = storage.loadWiFiCredentials(ssid, password) && !ssid.empty();
    
    if (provisioned) {
        ESP_LOGI(TAG, "Already provisioned, starting WiFi station");
        
        // Start WiFi station
#ifdef CONFIG_ESP_WIFI_REMOTE_LIBRARY_EPPP
        ESP_ERROR_CHECK(esp_wifi_remote_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_remote_start());
#else
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
#endif
        
        setState(ProvisioningState::CONNECTING);
        return ESP_OK;
    }
    
    // For ESP32P4, start AP mode for HTTP-based provisioning
    ESP_LOGI(TAG, "Starting AP mode for provisioning");
    
    wifi_config_t ap_config = {};
    strcpy((char*)ap_config.ap.ssid, service_name_.c_str());
    strcpy((char*)ap_config.ap.password, service_key_.c_str());
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap_config.ap.ssid_len = strlen(service_name_.c_str());
    ap_config.ap.max_connection = 1;
    ap_config.ap.channel = 1;
    
#ifdef CONFIG_ESP_WIFI_REMOTE_LIBRARY_EPPP
    ESP_ERROR_CHECK(esp_wifi_remote_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_remote_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_remote_start());
#else
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());
#endif
    
    setState(ProvisioningState::PROVISIONING);
    
    // Start HTTP server for web-based provisioning
    esp_err_t ret = startHttpServer();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Print connection info
    ESP_LOGI(TAG, "Provisioning AP started");
    ESP_LOGI(TAG, "Connect to WiFi: %s", service_name_.c_str());
    ESP_LOGI(TAG, "Password: %s", service_key_.c_str());
    ESP_LOGI(TAG, "Then navigate to http://192.168.4.1 to configure WiFi");
    
    return ESP_OK;
}

esp_err_t WiFiProvisioning::stopProvisioning() {
    ESP_LOGI(TAG, "Stopping WiFi provisioning");
    
    // Stop HTTP server
    stopHttpServer();
    
    // For ESP32P4, just stop WiFi
#ifdef CONFIG_ESP_WIFI_REMOTE_LIBRARY_EPPP
    esp_wifi_remote_stop();
#else
    esp_wifi_stop();
#endif
    
    setState(ProvisioningState::IDLE);
    return ESP_OK;
}

bool WiFiProvisioning::isProvisioned() {
    // Check if WiFi credentials are stored in flash
    FlashStorage storage;
    storage.initialize();
    std::string ssid, password;
    bool provisioned = storage.loadWiFiCredentials(ssid, password) && !ssid.empty();
    return provisioned;
}

bool WiFiProvisioning::connectToSavedWiFi() {
    if (!isProvisioned()) {
        ESP_LOGW(TAG, "Device not provisioned");
        return false;
    }
    
    ESP_LOGI(TAG, "Connecting to saved WiFi network");
    setState(ProvisioningState::CONNECTING);
    
    // Start WiFi in station mode
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    return true;
}

void WiFiProvisioning::wifiEventHandler(void* arg, esp_event_base_t event_base, 
                                       int32_t event_id, void* event_data) {
    WiFiProvisioning* self = static_cast<WiFiProvisioning*>(arg);
    
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "WiFi station started");
                esp_wifi_connect();
                break;
                
            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGI(TAG, "WiFi disconnected, retrying...");
                self->setState(ProvisioningState::CONNECTING, "Attempting to connect...");
                esp_wifi_connect();
                break;
                
            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP:
                {
                    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
                    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
                    self->setState(ProvisioningState::CONNECTED, "WiFi connected successfully");
                }
                break;
                
            default:
                break;
        }
    }
}

void WiFiProvisioning::provisioningEventHandler(void* arg, esp_event_base_t event_base, 
                                               int32_t event_id, void* event_data) {
    // Note: ESP32P4 with hosted WiFi doesn't use standard WiFi provisioning events
    // This function is kept for compatibility but may not be used
    (void)arg;
    (void)event_base;
    (void)event_data;
    ESP_LOGD(TAG, "Provisioning event received: %ld", event_id);
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

esp_err_t WiFiProvisioning::startHttpServer() {
    ESP_LOGI(TAG, "Starting HTTP server on port 80");
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 4;
    config.max_resp_headers = 8;
    config.stack_size = 8192;
    
    // Start the HTTP server
    esp_err_t ret = httpd_start(&http_server_, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error starting HTTP server: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register URI handlers
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = httpGetHandler,
        .user_ctx = this
    };
    httpd_register_uri_handler(http_server_, &root_uri);
    
    httpd_uri_t wifi_uri = {
        .uri = "/wifi",
        .method = HTTP_POST,
        .handler = httpPostHandler,
        .user_ctx = this
    };
    httpd_register_uri_handler(http_server_, &wifi_uri);
    
    ESP_LOGI(TAG, "HTTP server started successfully");
    return ESP_OK;
}

esp_err_t WiFiProvisioning::stopHttpServer() {
    if (http_server_) {
        ESP_LOGI(TAG, "Stopping HTTP server");
        esp_err_t ret = httpd_stop(http_server_);
        http_server_ = nullptr;
        return ret;
    }
    return ESP_OK;
}

esp_err_t WiFiProvisioning::httpGetHandler(httpd_req_t *req) {
    ESP_LOGI(TAG, "HTTP GET request received for: %s", req->uri);
    
    const char* html_response = 
        "<!DOCTYPE html>"
        "<html><head><title>ESP32P4 WiFi Setup</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<style>"
        "body { font-family: Arial; margin: 40px auto; max-width: 400px; }"
        "h1 { text-align: center; color: #333; }"
        "form { background: #f4f4f4; padding: 20px; border-radius: 8px; }"
        "input[type=text], input[type=password] { width: 100%; padding: 12px; margin: 8px 0; box-sizing: border-box; border: 1px solid #ddd; border-radius: 4px; }"
        "input[type=submit] { width: 100%; background-color: #4CAF50; color: white; padding: 14px; margin: 8px 0; border: none; border-radius: 4px; cursor: pointer; }"
        "input[type=submit]:hover { background-color: #45a049; }"
        "</style></head>"
        "<body>"
        "<h1>WiFi Configuration</h1>"
        "<form action='/wifi' method='POST'>"
        "<label for='ssid'>WiFi Network:</label><br>"
        "<input type='text' id='ssid' name='ssid' placeholder='Enter WiFi SSID' required><br>"
        "<label for='password'>Password:</label><br>"
        "<input type='password' id='password' name='password' placeholder='Enter WiFi Password'><br>"
        "<input type='submit' value='Connect'>"
        "</form>"
        "<p style='text-align: center; color: #666; font-size: 12px;'>ESP32P4 Network Camera</p>"
        "</body></html>";
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_response, HTTPD_RESP_USE_STRLEN);
    
    return ESP_OK;
}

esp_err_t WiFiProvisioning::httpPostHandler(httpd_req_t *req) {
    ESP_LOGI(TAG, "HTTP POST request received for WiFi configuration");
    
    char content[200];
    size_t recv_size = MIN(req->content_len, sizeof(content) - 1);
    
    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    ESP_LOGI(TAG, "Received data: %s", content);
    
    // Parse form data (ssid=...&password=...)
    char ssid[64] = {0};
    char password[128] = {0};
    
    // Simple form parsing
    char* ssid_start = strstr(content, "ssid=");
    char* password_start = strstr(content, "password=");
    
    if (ssid_start) {
        ssid_start += 5; // Skip "ssid="
        char* ssid_end = strstr(ssid_start, "&");
        if (!ssid_end) ssid_end = content + strlen(content);
        
        size_t ssid_len = MIN(ssid_end - ssid_start, sizeof(ssid) - 1);
        strncpy(ssid, ssid_start, ssid_len);
        ssid[ssid_len] = '\0';
        
        // URL decode (basic implementation for spaces)
        for (char* p = ssid; *p; p++) {
            if (*p == '+') *p = ' ';
        }
    }
    
    if (password_start) {
        password_start += 9; // Skip "password="
        char* password_end = strstr(password_start, "&");
        if (!password_end) password_end = content + strlen(content);
        
        size_t password_len = MIN(password_end - password_start, sizeof(password) - 1);
        strncpy(password, password_start, password_len);
        password[password_len] = '\0';
        
        // URL decode (basic implementation for spaces)
        for (char* p = password; *p; p++) {
            if (*p == '+') *p = ' ';
        }
    }
    
    ESP_LOGI(TAG, "Parsed SSID: %s", ssid);
    ESP_LOGI(TAG, "Password length: %d", strlen(password));
    
    // Save credentials to flash
    FlashStorage storage;
    storage.initialize();
    bool saved = storage.saveWiFiCredentials(std::string(ssid), std::string(password));
    
    const char* response;
    if (saved && strlen(ssid) > 0) {
        response = 
            "<!DOCTYPE html>"
            "<html><head><title>WiFi Configured</title>"
            "<meta name='viewport' content='width=device-width, initial-scale=1'>"
            "<style>body { font-family: Arial; margin: 40px auto; max-width: 400px; text-align: center; }</style></head>"
            "<body>"
            "<h1>WiFi Configured!</h1>"
            "<p>Your ESP32P4 device will now try to connect to the WiFi network.</p>"
            "<p>You can disconnect from this AP and check your device status.</p>"
            "</body></html>";
        
        // Create a timer to delay the state change to allow HTTP response to be sent
        esp_timer_handle_t delay_timer;
        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                WiFiProvisioning* self = static_cast<WiFiProvisioning*>(arg);
                self->setState(ProvisioningState::CREDENTIALS_RECEIVED, "WiFi credentials received");
            },
            .arg = req->user_ctx,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "credentials_delay",
            .skip_unhandled_events = false
        };
        
        esp_timer_create(&timer_args, &delay_timer);
        esp_timer_start_once(delay_timer, 2000000); // 2 seconds delay
        
        ESP_LOGI(TAG, "WiFi credentials saved, will attempt connection in 2 seconds");
        
    } else {
        response = 
            "<!DOCTYPE html>"
            "<html><head><title>Error</title></head>"
            "<body><h1>Error</h1><p>Failed to save WiFi credentials. Please try again.</p></body></html>";
    }
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    
    return ESP_OK;
}