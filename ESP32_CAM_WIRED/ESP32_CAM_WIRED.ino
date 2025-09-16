#include "esp_camera.h"
#include <Arduino.h>

// --- ROBUST CONFIGURATION ---
// A stable FPS that balances smoothness and data load.
const int TARGET_FPS = 15;
// The baud rate for USB communication. MUST match the Python script.
const long BAUD_RATE = 2000000;

// --- Pin Definitions for XIAO ESP32S3 Sense ---
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    10
#define SIOD_GPIO_NUM    40
#define SIOC_GPIO_NUM    39
#define Y9_GPIO_NUM      48
#define Y8_GPIO_NUM      11
#define Y7_GPIO_NUM      12
#define Y6_GPIO_NUM      14
#define Y5_GPIO_NUM      16
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM      17
#define Y2_GPIO_NUM      15
#define VSYNC_GPIO_NUM   38
#define HREF_GPIO_NUM    47
#define PCLK_GPIO_NUM    13

void setup() {
    // Start serial communication. Note: All text-based Serial.println()
    // messages have been removed to ensure a clean data stream.
    Serial.begin(BAUD_RATE);

    // --- Camera Configuration ---
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
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;

    // --- Initialize Camera ---
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        // If init fails, the board will restart.
        // If you get a boot loop, check physical camera connections.
        ESP.restart();
    }

    // --- Set Mirror/Flip Settings ---
    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {
        s->set_hmirror(s, 1); // Set to 1 or 0 to get correct horizontal orientation
        s->set_vflip(s, 0);   // Set to 1 or 0 to get correct vertical orientation
    }
}

void loop() {
    static unsigned long lastFrameTime = 0;
    const unsigned long frameInterval = 1000 / TARGET_FPS;

    // Frame rate control
    if (millis() - lastFrameTime < frameInterval) {
        return;
    }
    lastFrameTime = millis();

    // Capture a frame
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        return; // Silently fail and try again on the next loop
    }

    // Communication Protocol:
    // 1. Send 4-byte little-endian integer for the frame length.
    uint32_t frame_len = fb->len;
    Serial.write((const char*)&frame_len, sizeof(frame_len));
    // 2. Send the raw JPEG data.
    Serial.write(fb->buf, fb->len);
    
    esp_camera_fb_return(fb);
}
