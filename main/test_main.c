#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "wifi_connection.h"
#include "websocket_client.h"

static const char *TAG = "TEST_MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting test...");

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

    // Initialize LCD display
    ret = init_lcd_display();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LCD display");
        return;
    }

    ESP_LOGI(TAG, "LCD initialized successfully!");

    // Keep main task alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000)); // Check every 5 seconds
        ESP_LOGI(TAG, "Test is running...");
    }
} 