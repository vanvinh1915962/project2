#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "websocket_client.h"
#include "wifi_connection.h"

static const char *TAG = "WEBSOCKET_LCD_MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting WebSocket LCD client...");

    // Initialize WiFi
    esp_err_t ret = init_wifi();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi");
        return;
    }

    // Connect to WiFi
    ret = connect_wifi(WIFI_SSID, WIFI_PASSWORD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to WiFi");
        return;
    }

    ESP_LOGI(TAG, "WiFi connected successfully!");

    // Initialize GPIO for mode detection FIRST
    ret = init_mode_detection_gpio();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mode detection GPIO");
        return;
    }

    ESP_LOGI(TAG, "Mode detection GPIO initialized successfully!");

    // Initialize LCD display
    ret = init_lcd_display();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LCD display");
        return;
    }

    ESP_LOGI(TAG, "LCD display initialized successfully!");

    // Initialize image mutex (needed for LCD operations)
    ret = init_image_mutex();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize image mutex");
        return;
    }

    ESP_LOGI(TAG, "Image mutex initialized successfully!");

    // Don't initialize WebSocket client here - will be done based on mode
    ESP_LOGI(TAG, "WebSocket client will be initialized based on mode");

    // Create mode management task (GPIO already initialized)
    xTaskCreate(websocket_client_task, "mode_management_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "WebSocket LCD client started successfully");
    
    // Keep main task alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // Check every 10 seconds
        ESP_LOGI(TAG, "WebSocket LCD client is running...");
    }
} 