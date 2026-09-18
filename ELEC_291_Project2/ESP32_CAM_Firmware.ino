/*
 * ESP32-CAM MJPEG Stream Server (Access Point Mode)
 * 
 * This firmware runs the ESP32-CAM as a standalone Wi-Fi access point
 * and serves an MJPEG video stream that the Android app can display.
 * 
 * Wi-Fi Network:
 *   SSID:     RobotCAM
 *   Password: robot1234
 *   IP:       192.168.4.1
 * 
 * Stream URL: http://192.168.4.1/stream
 * 
 * Board Selection in Arduino IDE:
 *   Board: "AI Thinker ESP32-CAM"
 *   (Install "esp32" board package by Espressif first)
 */

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

// ========================
//  Wi-Fi AP Configuration
// ========================
const char* AP_SSID     = "RobotCAM";
const char* AP_PASSWORD = "robot1234";  // min 8 chars for WPA2

// ========================
//  AI Thinker ESP32-CAM
//  Camera Pin Definitions
// ========================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Built-in LED (active LOW on most ESP32-CAM boards)
#define LED_BUILTIN        33

// HTTP server handle
httpd_handle_t stream_httpd = NULL;

// MJPEG stream boundary
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ========================
//  MJPEG Stream Handler
// ========================
static esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    char part_buf[64];

    res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;

    // Disable caching
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera frame failed");
            res = ESP_FAIL;
            break;
        }

        // Send boundary
        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (res != ESP_OK) {
            esp_camera_fb_return(fb);
            break;
        }

        // Send part header with content length
        size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
        res = httpd_resp_send_chunk(req, part_buf, hlen);
        if (res != ESP_OK) {
            esp_camera_fb_return(fb);
            break;
        }

        // Send JPEG data
        res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        esp_camera_fb_return(fb);

        if (res != ESP_OK) break;
    }

    return res;
}

// ========================
//  Simple index page
// ========================
static esp_err_t index_handler(httpd_req_t *req) {
    const char* html = "<html><body>"
                       "<h1>RobotCAM Stream</h1>"
                       "<img src=\"/stream\" style=\"width:100%;\">"
                       "</body></html>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, strlen(html));
}

// ========================
//  Start HTTP Server
// ========================
void startStreamServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port   = 32768;

    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        // Register stream endpoint
        httpd_uri_t stream_uri = {
            .uri       = "/stream",
            .method    = HTTP_GET,
            .handler   = stream_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(stream_httpd, &stream_uri);

        // Register index page
        httpd_uri_t index_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = index_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(stream_httpd, &index_uri);

        Serial.println("Stream server started on port 80");
        Serial.println("Stream URL: http://192.168.4.1/stream");
    } else {
        Serial.println("Failed to start stream server!");
    }
}

// ========================
//  Camera Initialization
// ========================
bool initCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode    = CAMERA_GRAB_LATEST;  // always grab latest frame

    // Use VGA (640x480) - good balance of quality vs speed
    config.frame_size   = FRAMESIZE_VGA;
    config.jpeg_quality = 12;   // 0-63, lower = better quality
    config.fb_count     = 2;    // double buffer for smoother streaming
    config.fb_location  = CAMERA_FB_IN_PSRAM;

    // If no PSRAM, fall back to smaller frame
    if (!psramFound()) {
        Serial.println("No PSRAM found, using QVGA");
        config.frame_size   = FRAMESIZE_QVGA;
        config.jpeg_quality = 15;
        config.fb_count     = 1;
        config.fb_location  = CAMERA_FB_IN_DRAM;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    // Optional: tweak sensor settings
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, 1);   // -2 to 2
        s->set_contrast(s, 0);     // -2 to 2
        s->set_saturation(s, 0);   // -2 to 2
        s->set_vflip(s, 0);        // 0 or 1 — change if image is upside down
        s->set_hmirror(s, 0);      // 0 or 1 — change if image is mirrored
    }

    Serial.println("Camera initialized successfully");
    return true;
}

// ========================
//  Setup
// ========================
void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("=== RobotCAM Starting ===");

    // LED indicator
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // OFF (active low)

    // Initialize camera
    if (!initCamera()) {
        Serial.println("CAMERA INIT FAILED - restarting in 5 seconds...");
        delay(5000);
        ESP.restart();
    }

    // Start Wi-Fi Access Point
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.printf("AP started: SSID='%s' Password='%s'\n", AP_SSID, AP_PASSWORD);
    Serial.printf("AP IP address: %s\n", WiFi.softAPIP().toString().c_str());

    // Start streaming server
    startStreamServer();

    // LED on to indicate ready
    digitalWrite(LED_BUILTIN, LOW); // ON (active low)

    Serial.println("=== RobotCAM Ready ===");
    Serial.println("Connect to Wi-Fi 'RobotCAM' and open http://192.168.4.1/stream");
}

// ========================
//  Loop
// ========================
void loop() {
    // Nothing needed here — the HTTP server runs in its own task
    // Just print status periodically
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 10000) {
        Serial.printf("Clients connected: %d\n", WiFi.softAPgetStationNum());
        lastPrint = millis();
    }
    delay(100);
}
