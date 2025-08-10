#ifndef WIFI_CONNECTION_H_
#define WIFI_CONNECTION_H_

#include <esp_err.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

// WiFi configuration
#define WIFI_SSID "VienDanBac"
#define WIFI_PASSWORD "Thienthanh1994@"
#define WIFI_MAXIMUM_RETRY 5

// Function declarations
esp_err_t init_wifi(void);
esp_err_t connect_wifi(const char* ssid, const char* password);

#endif // WIFI_CONNECTION_H_ 