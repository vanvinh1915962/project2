#include "websocket_client.h"
#include "decode_image.h"
#include "pretty_effect.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_heap_caps.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_http_client.h"
#include "esp_websocket_client.h"
#include "jpeg_decoder.h"
#include <mbedtls/base64.h>
#include <string.h>

// #define LOG_DEBUG

static const char *TAG = "WEBSOCKET_CLIENT";

// LCD configuration - same as in lcd_tjpgd_example_main.c
#define LCD_HOST       SPI3_HOST
#define PARALLEL_LINES CONFIG_EXAMPLE_LCD_FLUSH_PARALLEL_LINES
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (20 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  0
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_DATA0          35
#define EXAMPLE_PIN_NUM_PCLK           36
#define EXAMPLE_PIN_NUM_CS             39
#define EXAMPLE_PIN_NUM_DC             38
#define EXAMPLE_PIN_NUM_RST            40
#define EXAMPLE_PIN_NUM_BK_LIGHT       4
#define EXAMPLE_LCD_H_RES              320
#define EXAMPLE_LCD_V_RES              240
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

// Expected max decoded JPEG size from ESP32-CAM server (FRAMESIZE_VGA 640x480)
#define MAX_DECODED_W 320
#define MAX_DECODED_H 240

// Global variables
static esp_websocket_client_handle_t client;
static esp_lcd_panel_handle_t panel_handle;
static SemaphoreHandle_t image_mutex;
static char *accumulated_data = NULL;
static size_t accumulated_len = 0;
static size_t accumulated_capacity = 0;
static size_t accumulated_expected_len = 0;
static TickType_t last_data_time = 0;

// Parse JPEG width/height from SOF0/SOF2 marker to size output buffer precisely
static bool parse_jpeg_size(const uint8_t *jpeg, size_t len, uint16_t *out_w, uint16_t *out_h)
{
    if (len < 4 || jpeg[0] != 0xFF || jpeg[1] != 0xD8) {
        return false;
    }
    size_t i = 2;
    while (i + 3 < len) {
        if (jpeg[i] != 0xFF) { i++; continue; }
        // skip fill bytes 0xFF
        while (i < len && jpeg[i] == 0xFF) i++;
        if (i >= len) break;
        uint8_t marker = jpeg[i++];
        // markers without length
        if (marker == 0xD9 || marker == 0xDA) {
            // EOI or SOS (SOS has length but size comes before scan data; if we hit SOS first, SOF missing)
            break;
        }
        if (i + 1 >= len) break;
        uint16_t seglen = ((uint16_t)jpeg[i] << 8) | jpeg[i+1];
        i += 2;
        if (seglen < 2 || i + (seglen - 2) > len) break;
        // SOF0 (0xC0) baseline or SOF2 (0xC2) progressive
        if (marker == 0xC0 || marker == 0xC2) {
            if (seglen >= 7) {
                // [precision][height MSB][height LSB][width MSB][width LSB]
                uint16_t h = ((uint16_t)jpeg[i+1] << 8) | jpeg[i+2];
                uint16_t w = ((uint16_t)jpeg[i+3] << 8) | jpeg[i+4];
                *out_w = w;
                *out_h = h;
                return true;
            }
            return false;
        }
        i += (seglen - 2);
    }
    return false;
}

// WebSocket event handler
static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    
    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_CONNECTED");
            // Send "capture" message to start streaming
            int ret = esp_websocket_client_send_text(client, "capture", strlen("capture"), portMAX_DELAY);
            if (ret < 0) {
                ESP_LOGE(TAG, "Failed to send capture message");
            } else {
                ESP_LOGI(TAG, "Capture message sent successfully - streaming started");
            }
            break;
            
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_DISCONNECTED");
            // Reset accumulated data
            accumulated_len = 0;
            last_data_time = 0;
            break;
            
        case WEBSOCKET_EVENT_DATA:
#if LOG_DEBUG
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_DATA");
            ESP_LOGI(TAG, "Received chunk: data_len=%ld payload_len=%ld offset=%ld", (long)data->data_len, (long)data->payload_len, (long)data->payload_offset);
#endif

            if (data->data_len > 0) {
                // Allocate big enough buffer at the start of a frame
                if (data->payload_offset == 0) {
                    accumulated_expected_len = data->payload_len;
                    accumulated_len = 0;
                    if (accumulated_capacity < accumulated_expected_len + 1) {
                        char *new_buf = realloc(accumulated_data, accumulated_expected_len + 1);
                        if (new_buf == NULL) {
                            ESP_LOGE(TAG, "Failed to allocate %ld bytes for frame", (long)(accumulated_expected_len + 1));
                            return;
                        }
                        accumulated_data = new_buf;
                        accumulated_capacity = accumulated_expected_len + 1;
                    }
                }

                // Bounds check
                size_t write_end = data->payload_offset + data->data_len;
                if (accumulated_capacity >= write_end) {
                    memcpy(accumulated_data + data->payload_offset, data->data_ptr, data->data_len);
                    accumulated_len = write_end;
                } else {
                    ESP_LOGE(TAG, "Chunk exceeds buffer: end=%zu cap=%zu", write_end, accumulated_capacity);
                    return;
                }
                last_data_time = xTaskGetTickCount();

                // Debug first/last bytes of the accumulated buffer so far
                if (accumulated_len >= 64) {
                    char debug_str[65];
                    memcpy(debug_str, accumulated_data, 64);
                    debug_str[64] = '\0';
#if LOG_DEBUG
                    ESP_LOGI(TAG, "First 64 chars: %s", debug_str);
#endif
                    char debug_end[33];
                    size_t end_len = 32;
                    memcpy(debug_end, accumulated_data + accumulated_len - end_len, end_len);
                    debug_end[end_len] = '\0';
#if LOG_DEBUG
                    ESP_LOGI(TAG, "Last %zu chars: %s", end_len, debug_end);
#endif
                }

                // If received all bytes of the frame, process
                if (accumulated_expected_len > 0 && accumulated_len == accumulated_expected_len) {
                    accumulated_data[accumulated_len] = '\0';
#if LOG_DEBUG
                    ESP_LOGI(TAG, "Complete base64 frame received, length: %zu", accumulated_len);
#endif

                    uint8_t *jpeg_data = NULL;
                    size_t jpeg_len = 0;
                    esp_err_t ret = decode_base64_image(accumulated_data, accumulated_len, &jpeg_data, &jpeg_len);
                    if (ret == ESP_OK && jpeg_data != NULL) {
                        ret = display_jpeg_on_lcd(jpeg_data, jpeg_len);
                        if (ret != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to display JPEG on LCD");
                        }
                        free(jpeg_data);
                    } else {
                        ESP_LOGE(TAG, "Failed to decode base64 image");
                    }
                    accumulated_len = 0;
                    accumulated_expected_len = 0;
                }
            }
            break;
            
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_ERROR");
            break;
            
        default:
            ESP_LOGI(TAG, "Other event id:%ld", (long)event_id);
            break;
    }
}

esp_err_t init_lcd_display(void)
{
    // Initialize GPIO for backlight
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    // Initialize SPI bus
    spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_PCLK,
        .mosi_io_num = EXAMPLE_PIN_NUM_DATA0,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = PARALLEL_LINES * EXAMPLE_LCD_H_RES * 2 + 8
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Initialize LCD panel IO
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    // Initialize LCD panel
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    // Reset and initialize LCD
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));

    // Ensure backlight is ON
    ESP_ERROR_CHECK(gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL));

    // Don't initialize pretty effect for websocket client
    // ESP_ERROR_CHECK(pretty_effect_init());

    ESP_LOGI(TAG, "LCD display initialized successfully");
    return ESP_OK;
}

esp_err_t init_websocket_client(void)
{
    // Create mutex for image processing
    image_mutex = xSemaphoreCreateMutex();
    if (image_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    // Initialize accumulated data buffer
    accumulated_data = malloc(4096); // Initial buffer size
    if (accumulated_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate accumulated data buffer");
        return ESP_FAIL;
    }
    accumulated_capacity = 4096;
    accumulated_len = 0;
    last_data_time = 0;

    // Try to find camera server IP automatically
    char camera_ip[32];
    esp_err_t ret = find_camera_server_ip(camera_ip, sizeof(camera_ip));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Found camera server at %s", camera_ip);
        // Update URI with found IP
        char uri[64];
        snprintf(uri, sizeof(uri), "ws://%s:80/ws", camera_ip);
        
        // WebSocket client configuration
        esp_websocket_client_config_t websocket_cfg = {
            .uri = uri,
            .buffer_size = WEBSOCKET_BUFFER_SIZE,
        };

        // Initialize WebSocket client
        client = esp_websocket_client_init(&websocket_cfg);
        if (client == NULL) {
            ESP_LOGE(TAG, "Failed to initialize WebSocket client");
            return ESP_FAIL;
        }

        // Register event handler
        esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, websocket_event_handler, NULL);

        // Start WebSocket client
        ret = esp_websocket_client_start(client);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start WebSocket client");
            return ret;
        }

        ESP_LOGI(TAG, "WebSocket client initialized successfully with auto-discovered IP");
        return ESP_OK;
    } else {
        ESP_LOGI(TAG, "Auto-discovery failed, using default IP");
        
        // Use default configuration
        esp_websocket_client_config_t websocket_cfg = {
            .uri = WEBSOCKET_SERVER_URI,
            .buffer_size = WEBSOCKET_BUFFER_SIZE,
        };

        // Initialize WebSocket client
        client = esp_websocket_client_init(&websocket_cfg);
        if (client == NULL) {
            ESP_LOGE(TAG, "Failed to initialize WebSocket client");
            return ESP_FAIL;
        }

        // Register event handler
        esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, websocket_event_handler, NULL);

        // Start WebSocket client
        ret = esp_websocket_client_start(client);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start WebSocket client");
            return ret;
        }

        ESP_LOGI(TAG, "WebSocket client initialized successfully with default IP");
        return ESP_OK;
    }
}

esp_err_t decode_base64_image(const char* base64_data, size_t data_len, uint8_t** jpeg_data, size_t* jpeg_len)
{
    if (xSemaphoreTake(image_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_FAIL;
    }

#if LOG_DEBUG
    ESP_LOGI(TAG, "Attempting to decode base64 data, length: %zu", data_len);
#endif
    
    // Validate base64 data (should only contain A-Z, a-z, 0-9, +, /, =)
    // Skip validation for now to see what data we're actually receiving
#if LOG_DEBUG
    ESP_LOGI(TAG, "Base64 data validation skipped for debugging");
#endif

    // Calculate decoded size using mbedTLS. Passing NULL buffer is expected to return
    // MBEDTLS_ERR_BASE64_BUFFER_TOO_SMALL and set decoded_len to the required size.
    size_t decoded_len = 0;
    int mret = mbedtls_base64_decode(NULL, 0, &decoded_len, (const unsigned char*)base64_data, data_len);
    if (mret != MBEDTLS_ERR_BASE64_BUFFER_TOO_SMALL && mret != 0) {
        ESP_LOGE(TAG, "Failed to calculate base64 decoded size, mbedtls error: %d", mret);
        xSemaphoreGive(image_mutex);
        return ESP_FAIL;
    }

    // Allocate memory for decoded data
    *jpeg_data = malloc(decoded_len);
    if (*jpeg_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for decoded data");
        xSemaphoreGive(image_mutex);
        return ESP_ERR_NO_MEM;
    }

    // Decode base64
    mret = mbedtls_base64_decode(*jpeg_data, decoded_len, &decoded_len, (const unsigned char*)base64_data, data_len);
    if (mret != 0) {
        ESP_LOGE(TAG, "Failed to decode base64 data, mbedtls error: %d", mret);
        free(*jpeg_data);
        *jpeg_data = NULL;
        xSemaphoreGive(image_mutex);
        return ESP_FAIL;
    }

    *jpeg_len = decoded_len;
#if LOG_DEBUG
    ESP_LOGI(TAG, "Base64 decoded successfully, JPEG size: %zu", decoded_len);
#endif
    xSemaphoreGive(image_mutex);
    return ESP_OK;
}

esp_err_t display_jpeg_on_lcd(const uint8_t* jpeg_data, size_t jpeg_len)
{
    if (xSemaphoreTake(image_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_FAIL;
    }

    // Determine decoded image dimensions from JPEG header to minimize required buffer
    uint16_t jpeg_w = EXAMPLE_LCD_H_RES;
    uint16_t jpeg_h = EXAMPLE_LCD_V_RES;
    if (!parse_jpeg_size(jpeg_data, jpeg_len, &jpeg_w, &jpeg_h)) {
        // Fallback if parsing fails
        jpeg_w = EXAMPLE_LCD_H_RES;
        jpeg_h = EXAMPLE_LCD_V_RES;
    }
    // Clamp to a safe maximum to avoid absurd allocations
    if (jpeg_w > MAX_DECODED_W) jpeg_w = MAX_DECODED_W;
    if (jpeg_h > MAX_DECODED_H) jpeg_h = MAX_DECODED_H;

    // Allocate memory for decoded pixels sized exactly to the image
    size_t required_bytes = (size_t)jpeg_w * (size_t)jpeg_h * sizeof(uint16_t);
#if LOG_DEBUG
    ESP_LOGI(TAG, "Required bytes: %zu", required_bytes);
#endif
    uint16_t *pixels = heap_caps_malloc(required_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (pixels == NULL) {
        pixels = heap_caps_malloc(required_bytes, MALLOC_CAP_8BIT);
    }
    if (pixels == NULL) {
        ESP_LOGE(TAG, "Failed to allocate %lu bytes for %ux%u pixels. Enable PSRAM or reduce camera frame size.", (unsigned long)required_bytes, (unsigned)jpeg_w, (unsigned)jpeg_h);
        xSemaphoreGive(image_mutex);
        return ESP_ERR_NO_MEM;
    }

    // JPEG decode config
    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = (uint8_t *)jpeg_data,
        .indata_size = jpeg_len,
        .outbuf = (uint8_t*)pixels,
        .outbuf_size = required_bytes,
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = JPEG_IMAGE_SCALE_0,
        .flags = {
            .swap_color_bytes = 1,
        }
    };

    // Decode JPEG
    esp_jpeg_image_output_t outimg;
    esp_err_t ret = esp_jpeg_decode(&jpeg_cfg, &outimg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to decode JPEG");
        free(pixels);
        xSemaphoreGive(image_mutex);
        return ret;
    }
#if LOG_DEBUG
    ESP_LOGI(TAG, "JPEG decoded successfully: %ldx%ld", (long)outimg.width, (long)outimg.height);
#endif
    // Display on LCD, crop if decoded image is larger than panel
    int draw_w = (outimg.width  > EXAMPLE_LCD_H_RES) ? EXAMPLE_LCD_H_RES : outimg.width;
    int draw_h = (outimg.height > EXAMPLE_LCD_V_RES) ? EXAMPLE_LCD_V_RES : outimg.height;
    ret = esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, draw_w, draw_h, pixels);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to draw bitmap on LCD");
    } else {
        ESP_LOGI(TAG, "Image displayed on LCD successfully");
    }

    free(pixels);
    xSemaphoreGive(image_mutex);
    return ret;
}

esp_err_t find_camera_server_ip(char* ip_buffer, size_t buffer_size)
{
    // Try common ESP32-CAM IP addresses
    const char* common_ips[] = {
        "192.168.1.93"
    };
    
    for (int i = 0; i < sizeof(common_ips) / sizeof(common_ips[0]); i++) {
        ESP_LOGI(TAG, "Trying to connect to camera server at %s", common_ips[i]);
        
        // Try to connect to WebSocket server
        char uri[64];
        snprintf(uri, sizeof(uri), "ws://%s:80/ws", common_ips[i]);
        
        esp_websocket_client_config_t test_config = {
            .uri = uri,
            .buffer_size = 1024,
        };
        
        esp_websocket_client_handle_t test_client = esp_websocket_client_init(&test_config);
        if (test_client != NULL) {
            esp_err_t ret = esp_websocket_client_start(test_client);
            if (ret == ESP_OK) {
                // Wait a bit to see if connection succeeds
                vTaskDelay(pdMS_TO_TICKS(2000));
                
                if (esp_websocket_client_is_connected(test_client)) {
                    ESP_LOGI(TAG, "Found camera server at %s", common_ips[i]);
                    strncpy(ip_buffer, common_ips[i], buffer_size - 1);
                    ip_buffer[buffer_size - 1] = '\0';
                    esp_websocket_client_stop(test_client);
                    esp_websocket_client_destroy(test_client);
                    return ESP_OK;
                }
                esp_websocket_client_stop(test_client);
            }
            esp_websocket_client_destroy(test_client);
        }
    }
    
    ESP_LOGE(TAG, "Could not find camera server IP");
    return ESP_FAIL;
}

void websocket_client_task(void *pvParameters)
{
    ESP_LOGI(TAG, "WebSocket client task started");
    
    while (1) {
        // Keep the task alive - no need to send capture repeatedly
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
} 