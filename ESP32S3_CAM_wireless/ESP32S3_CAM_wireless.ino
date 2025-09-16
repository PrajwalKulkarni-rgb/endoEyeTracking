#include "esp_camera.h"
#include "WiFi.h"
#include "esp_sntp.h"
#include <sys/time.h>

// ==========================================================
//                 CONFIGURE THESE SETTINGS
// ==========================================================
const char* WIFI_SSID = "Garp";
const char* WIFI_PASS = "952185486450";
const char* SERVER_IP = "192.168.143.41";
const int SERVER_PORT = 8000;

// *** IMPORTANT: CHANGE THIS FOR EACH OF YOUR 3 CAMERAS ***
const char* CAMERA_ID = "FORWARD"; // Use "LEFT_EYE" or "RIGHT_EYE" for the others

// --- Performance Tuning ---
const int TARGET_FPS = 10; // Target frames per second
// ==========================================================


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

// --- System State Management ---
enum SystemState {
    STATE_WIFI_CONNECT,
    STATE_NTP_SYNC,
    STATE_SERVER_CONNECT,
    STATE_STREAMING
};
SystemState currentState = STATE_WIFI_CONNECT;

// --- Global Objects ---
WiFiClient client;
const long NTP_TIMEOUT = 30000; // 30 seconds for NTP sync timeout

// --- Data Header for each frame ---
struct FrameHeader {
    char id[16];
    uint32_t timestamp_sec;
    uint32_t timestamp_usec;
    uint32_t frame_len;
};

// --- Function Prototypes ---
bool initCamera();
void connectWiFi();
void syncNTP();
void connectToServer();
void streamFrames();

// ==========================================================
//                         SETUP
// ==========================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n\nBooting up...");

    if (!initCamera()) {
        Serial.println("FATAL: Camera initialization failed. Restarting...");
        delay(5000);
        ESP.restart();
    }
}

// ==========================================================
//                       MAIN LOOP
// ==========================================================
void loop() {
    switch (currentState) {
        case STATE_WIFI_CONNECT:
            connectWiFi();
            break;
        case STATE_NTP_SYNC:
            syncNTP();
            break;
        case STATE_SERVER_CONNECT:
            connectToServer();
            break;
        case STATE_STREAMING:
            streamFrames();
            break;
    }
}

// ==========================================================
//                   HELPER FUNCTIONS
// ==========================================================

/**
 * @brief Initializes the camera sensor.
 * @return True if successful, false otherwise.
 */
bool initCamera() {
    Serial.println("Initializing camera...");
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
    config.frame_size = FRAMESIZE_VGA; // 640x480. Good balance.
    config.jpeg_quality = 12;          // Lower number = higher quality
    config.fb_count = 2;               // Use 2 frame buffers for smoother capture
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }
    Serial.println("Camera initialized successfully.");
    return true;
}

/**
 * @brief Connects to Wi-Fi if not already connected.
 */
void connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi connection verified.");
        currentState = STATE_NTP_SYNC;
        return;
    }
    
    Serial.print("Connecting to WiFi SSID: ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (millis() - startTime > 20000) { // 20-second timeout
            Serial.println("\nWiFi connection timed out. Retrying...");
            startTime = millis(); // Reset timer for next attempt
        }
    }
    Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());
    currentState = STATE_NTP_SYNC;
}

/**
 * @brief Synchronizes the system time with an NTP server.
 */
void syncNTP() {
    Serial.print("Waiting for NTP time sync... ");
    configTime(0, 0, "pool.ntp.org");
    
    struct tm timeinfo;
    unsigned long startTime = millis();
    while (!getLocalTime(&timeinfo)) {
        Serial.print(".");
        delay(500);
        if (millis() - startTime > NTP_TIMEOUT) {
            Serial.println("\nNTP sync timed out. Retrying...");
            return; // Stay in this state and retry
        }
    }
    Serial.println(" Synced!");
    currentState = STATE_SERVER_CONNECT;
}

/**
 * @brief Connects to the main Python server.
 */
void connectToServer() {
    Serial.print("Connecting to server ");
    Serial.print(SERVER_IP);
    Serial.print(":");
    Serial.println(SERVER_PORT);
    
    if (!client.connect(SERVER_IP, SERVER_PORT)) {
        Serial.println("Connection to server failed. Retrying in 2 seconds...");
        delay(2000);
        return; // Stay in this state and retry
    }
    
    Serial.println("Connected to server successfully.");
    currentState = STATE_STREAMING;
}

/**
 * @brief Captures and sends frames to the server.
 */
void streamFrames() {
    if (!client.connected()) {
        Serial.println("Server disconnected. Attempting to reconnect...");
        currentState = STATE_SERVER_CONNECT; // Change state to reconnect
        return;
    }

    static unsigned long lastFrameTime = 0;
    const unsigned long frameInterval = 1000 / TARGET_FPS;

    if (millis() - lastFrameTime < frameInterval) {
        return; // Wait until it's time for the next frame
    }
    lastFrameTime = millis();

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        return;
    }

    // Get timestamp immediately after capture
    struct timeval tv;
    gettimeofday(&tv, NULL);

    FrameHeader header;
    strncpy(header.id, CAMERA_ID, sizeof(header.id) - 1);
    header.id[sizeof(header.id) - 1] = '\0';
    header.timestamp_sec = tv.tv_sec;
    header.timestamp_usec = tv.tv_usec;
    header.frame_len = fb->len;

    // Send header and frame data
    bool success = (client.write((const uint8_t*)&header, sizeof(header)) == sizeof(header)) &&
                   (client.write(fb->buf, fb->len) == fb->len);

    esp_camera_fb_return(fb);

    if (!success) {
        Serial.println("Failed to send frame to server.");
        client.stop();
        currentState = STATE_SERVER_CONNECT; // Change state to reconnect
    }
}
