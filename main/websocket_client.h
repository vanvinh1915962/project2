#ifndef WEBSOCKET_CLIENT_H_
#define WEBSOCKET_CLIENT_H_

#include <esp_err.h>
#include <esp_http_client.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <mbedtls/base64.h>
#include <string.h>
#include <stdlib.h>

// WebSocket configuration
#define WEBSOCKET_SERVER_URI "ws://192.168.1.93:80/ws"  // Update with your camera server IP
// You can also use the camera server's hostname if available
// #define WEBSOCKET_SERVER_URI "ws://esp32cam.local:80/ws"
// Common ESP32-CAM IP addresses:
// #define WEBSOCKET_SERVER_URI "ws://192.168.1.100:80/ws"
// #define WEBSOCKET_SERVER_URI "ws://192.168.4.1:80/ws"  // ESP32-CAM AP mode
#define WEBSOCKET_BUFFER_SIZE 32768

// Function declarations
esp_err_t init_lcd_display(void);
esp_err_t init_websocket_client(void);
esp_err_t decode_base64_image(const char* base64_data, size_t data_len, uint8_t** jpeg_data, size_t* jpeg_len);
esp_err_t display_jpeg_on_lcd(const uint8_t* jpeg_data, size_t jpeg_len);
void websocket_client_task(void *pvParameters);
esp_err_t find_camera_server_ip(char* ip_buffer, size_t buffer_size);

#endif // WEBSOCKET_CLIENT_H_ 