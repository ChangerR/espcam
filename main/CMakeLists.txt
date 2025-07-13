idf_component_register(SRCS "main.cpp"
                           "camera_controller.cpp"
                           "wifi_provisioning.cpp"
                           "network_camera.cpp"
                           "mqtt_client.cpp"
                           "flash_storage.cpp"
                           "task_manager.cpp"
                           "h264_encoder.cpp"
                           "ts_muxer.cpp"
                           "video_recorder.cpp"
                           "motion_detector.cpp"
                           "cloud_uploader.cpp"
                           "stream_pusher.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES espressif__esp_wifi_remote
                             espressif__esp_hosted
                             wifi_provisioning  
                             nvs_flash 
                             esp_http_server 
                             esp_netif 
                             mqtt 
                             espressif__esp32-camera 
                             espressif__esp_codec_dev 
                             espressif__esp_h264
                             json 
                             freertos 
                             esp_timer 
                             fatfs 
                             esp_http_client 
                             esp-tls 
                             protocomm 
                             esp_driver_isp 
                             esp_mm)