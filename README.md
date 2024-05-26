# ESP32-CAM Barcode Reader

This project is an implementation of a barcode reader using the ESP32-CAM module. It sets up a web server to capture images and stream video using the ESP32-CAM, allowing for barcode reading and other image processing tasks.

## Requirements

- ESP32-CAM module
- Arduino IDE
- WiFi connection

## Setup Instructions

1. **Install the ESP32 Board in Arduino IDE**:
    - Open Arduino IDE.
    - Go to File > Preferences.
    - In the "Additional Board Manager URLs" field, add this URL: `https://dl.espressif.com/dl/package_esp32_index.json`.
    - Go to Tools > Board > Board Manager.
    - Search for "esp32" and install the latest version by Espressif Systems.

2. **Library Installation**:
    - Install the required libraries from the Arduino Library Manager:
        - WiFi
        - HTTPClient
        - ESP32

3. **Hardware Setup**:
    - Connect the ESP32-CAM module to your computer using an FTDI programmer. Ensure the connections are correct (GND to GND, 5V to VCC, TX to U0R, RX to U0T, and IO0 to GND for flashing).

4. **Flash the Code**:
    - Open the `esp32cam_barcode_reader.ino` file in Arduino IDE.
    - Select the correct board and port from the Tools menu (Board: "ESP32 dev Module" and the corresponding COM port).
    - Configure the board settings as follows:
        - **PSRAM**: Enable
        - **Partition Scheme**: hugo APP (3MB No OTA)
        - **CPU Frequency**: 240MHz
        - **Flash Mode**: QIO
        - **Flash Frequency**: 80MHz
        - **Flash Size**: 4MB
        - **Upload Speed**: 115200
        - **Core Debug Level**: None
    - Upload the code to the ESP32-CAM module.

## Code Explanation

### Libraries and Configuration

The code includes several libraries essential for camera operation, WiFi connectivity, and HTTP handling:

```cpp
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_http_server.h"
#include "img_converters.h"
```

### WiFi Configuration

Define the SSID and password for both WiFi station mode and access point mode:

```cpp
const char* ssid = "your_wifi_ssid";
const char* password = "your_wifi_password";
const char* apssid = "esp32cam";
const char* appassword = "12345678"; // AP password must be at least 8 characters
```

### Camera Pins Configuration

Configure the pins used by the ESP32-CAM module:

```cpp
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     21
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       19
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM        5
#define Y2_GPIO_NUM        4
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
```

### Camera Server Initialization

Initialize the camera and start the web server for streaming and capturing images:

```cpp
void startCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t status_uri = {
        .uri       = "/status",
        .method    = HTTP_GET,
        .handler   = status_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t cmd_uri = {
        .uri       = "/control",
        .method    = HTTP_GET,
        .handler   = cmd_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t capture_uri = {
        .uri       = "/capture",
        .method    = HTTP_GET,
        .handler   = capture_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
        httpd_register_uri_handler(camera_httpd, &status_uri);
        httpd_register_uri_handler(camera_httpd, &capture_uri);
    }

    config.server_port += 1;
    config.ctrl_port += 1;
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}
```

### Main Function

Setup WiFi and start the camera server:

```cpp
void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector

    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial.println();

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    if(psramFound()){
        config.frame_size = FRAMESIZE_UXGA;
        config.jpeg_quality = 10;
        config.fb_count = 2;
    } else {
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }

    // Camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }

    Serial.println("WiFi connected");
    Serial.print("Camera Stream Ready! Go to: http://");
    Serial.println(WiFi.localIP());

    startCameraServer();
}

void loop() {
    // put your main code here, to run repeatedly:
}
```

## Usage

1. Connect the ESP32-CAM module to power and ensure it's connected to the WiFi network.
2. Open a web browser and go to `http://<ESP32-CAM_IP_ADDRESS>/`.
3. You can capture images or stream video directly from the web interface.

