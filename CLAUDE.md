# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an ESP32P4 Network Camera project built with ESP-IDF framework. The project implements a complete IoT camera system that handles WiFi provisioning, MQTT communication, camera control, and flash storage.

## Build and Development Commands

### Environment Setup
```bash
# Source ESP-IDF environment (required for each session)
source $HOME/esp/esp-idf/export.sh
```

### Build Commands
```bash
# Build the project
idf.py build

# Build, flash, and monitor
idf.py -p PORT flash monitor

# Clean build
idf.py clean
```

### Configuration
```bash
# Open configuration menu
idf.py menuconfig
```

### Testing
```bash
# Run automated tests
pytest pytest_hello_world.py

# Run specific test targets
pytest -m esp32
pytest -m linux
pytest -m host_test
```

### Monitoring and Debugging
```bash
# Monitor serial output
idf.py -p PORT monitor

# Check for hardware issues
idf.py -p PORT monitor
# Then reboot the board to see output logs
```

## Architecture Overview

### Core Components
- **NetworkCamera**: Main orchestrator class that manages system state and coordinates all components
- **CameraController**: Handles ESP32P4 camera hardware with configurable pins and streaming capabilities
- **WiFiProvisioning**: Manages WiFi setup and credentials
- **MQTTClient**: Handles MQTT communication for remote control and telemetry
- **FlashStorage**: Manages persistent storage of settings and credentials

### System States
The NetworkCamera class uses a state machine with these states:
- INITIALIZING → PROVISIONING → CONNECTING_WIFI → CONNECTING_MQTT → RUNNING → ERROR

### Key Files
- `main/main.cpp`: Entry point and system initialization
- `main/network_camera.hpp/.cpp`: Main system orchestrator
- `main/camera_controller.hpp/.cpp`: Camera hardware interface
- `main/wifi_provisioning.hpp/.cpp`: WiFi setup and management
- `main/mqtt_client.hpp/.cpp`: MQTT communication
- `main/flash_storage.hpp/.cpp`: Persistent storage

### ESP32P4 Camera Pin Configuration
The camera uses specific pins configured in CameraController::CameraConfig:
- XCLK: Pin 15, SDA: Pin 4, SCL: Pin 5
- Data pins: D7-D0 on pins 16,17,18,12,10,8,9,11
- VSYNC: Pin 6, HREF: Pin 7, PCLK: Pin 13

## Development Notes

### Component Registration
The project uses ESP-IDF component system with dependencies registered in `main/CMakeLists.txt`:
- esp_wifi, nvs_flash, esp_http_server, esp_netif, mqtt5, esp_camera

### Error Handling
- Uses ESP-IDF error handling patterns with ESP_ERROR_CHECK
- C++ exception handling in main application loop
- State callbacks for component status monitoring

### Memory Management
- Uses smart pointers (std::unique_ptr) for component lifecycle management
- Proper frame buffer management for camera operations
- FreeRTOS task and timer management

### Testing Framework
Uses pytest with ESP-IDF embedded testing framework supporting:
- Hardware target testing
- QEMU simulation
- Host-based testing
- SHA256 verification for firmware integrity

## Current Build Status

✅ **Fixed Issues:**
- All source files (.cpp) are now included in CMakeLists.txt
- MQTT client converted from MQTT5 to standard MQTT API
- Camera controller configuration updated for ESP32-Camera component
- Old hello_world files removed
- Component dependencies added (esp32-camera, json)

⚠️ **Known Issues:**
- ESP32P4 requires external WiFi module for WiFi functionality
- WiFi provisioning component needs hardware-specific configuration
- Camera pins may need adjustment for specific ESP32P4 board layout

## Hardware Requirements
- ESP32P4 development board
- External WiFi module (ESP32P4 doesn't have built-in WiFi)
- Camera module compatible with ESP32-Camera library
- Proper pin connections as defined in CameraController::CameraConfig