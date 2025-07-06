idf_component_register(SRCS "main.cpp"
                           "camera_controller.cpp"
                           "wifi_provisioning.cpp"
                           "network_camera.cpp"
                           "mqtt_client.cpp"
                           "flash_storage.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi nvs_flash esp_http_server esp_netif mqtt esp32-camera json)