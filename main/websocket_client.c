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

// Simple 8x8 bitmap font for basic text display
// Each character is 8x8 pixels, stored as 8 bytes (1 byte per row)
static const uint8_t simple_font[128][8] = {
    // Space (32)
    [32] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    // A (65)
    [65] = {0x10, 0x28, 0x44, 0x44, 0x7C, 0x44, 0x44, 0x44},
    // D (68) 
    [68] = {0x78, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x78},
    // E (69)
    [69] = {0x7C, 0x40, 0x40, 0x78, 0x40, 0x40, 0x40, 0x7C},
    // L (76)
    [76] = {0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x7C},
    // M (77)
    [77] = {0x44, 0x6C, 0x54, 0x44, 0x44, 0x44, 0x44, 0x44},
    // N (78)
    [78] = {0x44, 0x64, 0x54, 0x4C, 0x44, 0x44, 0x44, 0x44},
    // O (79)
    [79] = {0x38, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x38},
    // P (80)
    [80] = {0x78, 0x44, 0x44, 0x78, 0x40, 0x40, 0x40, 0x40},
    // S (83)
    [83] = {0x38, 0x44, 0x40, 0x38, 0x04, 0x04, 0x44, 0x38},
    // V (86)
    [86] = {0x82, 0x82, 0x82, 0x44, 0x44, 0x28, 0x28, 0x10},
    // a (97)
    [97] = {0x00, 0x00, 0x38, 0x04, 0x3C, 0x44, 0x44, 0x3C},
    // g (103)
    [103] = {0x00, 0x00, 0x3C, 0x44, 0x44, 0x3C, 0x04, 0x38},
    // h (104)
    [104] = {0x80, 0x80, 0xB8, 0xC4, 0x84, 0x84, 0x84, 0x84},
    // i (105)
    [105] = {0x10, 0x00, 0x30, 0x10, 0x10, 0x10, 0x10, 0x38},
    // k (107)
    [107] = {0x40, 0x40, 0x48, 0x50, 0x60, 0x50, 0x48, 0x44},
    // l (108)
    [108] = {0x30, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x38},
    // m (109)
    [109] = {0x00, 0x00, 0xD8, 0xA4, 0xA4, 0xA4, 0xA4, 0xA4},
    // n (110)
    [110] = {0x00, 0x00, 0x78, 0x84, 0x84, 0x84, 0x84, 0x84},
    // o (111)
    [111] = {0x00, 0x00, 0x38, 0x44, 0x44, 0x44, 0x44, 0x38},
    // p (112)
    [112] = {0x00, 0x00, 0x78, 0x44, 0x44, 0x78, 0x40, 0x40},
    // t (116)
    [116] = {0x20, 0x20, 0xF8, 0x20, 0x20, 0x20, 0x20, 0x18},
    // u (117) 
    [117] = {0x00, 0x00, 0x84, 0x84, 0x84, 0x84, 0x84, 0x78},
    // Simple fallback for other chars
    [0] = {0x7E, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x7E}  // Rectangle for unknown chars
};

// Global variables
static esp_websocket_client_handle_t client;
static esp_lcd_panel_handle_t panel_handle;
static SemaphoreHandle_t image_mutex;
static char *accumulated_data = NULL;
static size_t accumulated_len = 0;
static size_t accumulated_capacity = 0;
static size_t accumulated_expected_len = 0;
static TickType_t last_data_time = 0;

// Mode detection variables
static display_mode_t current_mode = MODE_IMAGE_STREAM;
static bool websocket_streaming = false;

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
            // Send "capture" message to start streaming only if in IMAGE_STREAM mode
            if (current_mode == MODE_IMAGE_STREAM) {
                int ret = esp_websocket_client_send_text(client, "capture", strlen("capture"), portMAX_DELAY);
                if (ret < 0) {
                    ESP_LOGE(TAG, "Failed to send capture message");
                } else {
                    ESP_LOGI(TAG, "Capture message sent successfully - streaming started");
                    websocket_streaming = true;
                }
            } else {
                ESP_LOGI(TAG, "Connected but not starting streaming (mode: %d)", current_mode);
                websocket_streaming = false;
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

            // Only process image data if we're in IMAGE_STREAM mode
            if (data->data_len > 0 && current_mode == MODE_IMAGE_STREAM) {
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

esp_err_t init_image_mutex(void)
{
    // Create mutex for image processing
    image_mutex = xSemaphoreCreateMutex();
    if (image_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create image mutex");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Image mutex created successfully");
    return ESP_OK;
}

esp_err_t init_websocket_client(void)
{
    // Mutex should already be created
    if (image_mutex == NULL) {
        ESP_LOGE(TAG, "Image mutex not initialized");
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
    
    // Wait a bit to ensure all initializations are complete
    vTaskDelay(pdMS_TO_TICKS(500));
    
    display_mode_t previous_mode = get_current_mode();
    ESP_LOGI(TAG, "Initial mode detected: %d", previous_mode);
    
    // Handle initial mode
    handle_mode_change(previous_mode);
    
    while (1) {
        // Check for mode changes
        display_mode_t new_mode = get_current_mode();
        if (new_mode != previous_mode) {
            ESP_LOGI(TAG, "Mode changed from %d to %d", previous_mode, new_mode);
            handle_mode_change(new_mode);
            previous_mode = new_mode;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Check mode every 100ms
    }
}

// Initialize GPIO for mode detection
esp_err_t init_mode_detection_gpio(void)
{
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = (1ULL << MODE_DETECT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,  // Enable pull-up resistor
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    esp_err_t ret = gpio_config(&gpio_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure mode detection GPIO");
        return ret;
    }
    
    ESP_LOGI(TAG, "Mode detection GPIO %d initialized successfully", MODE_DETECT_GPIO);
    return ESP_OK;
}

// Get current mode based on GPIO state
display_mode_t get_current_mode(void)
{
    int gpio_level = gpio_get_level(MODE_DETECT_GPIO);
    ESP_LOGD(TAG, "GPIO %d level: %d", MODE_DETECT_GPIO, gpio_level);
    return (gpio_level == 0) ? MODE_IMAGE_STREAM : MODE_PASSWORD_PROMPT;
}

// Draw a single character on screen buffer
static void draw_char_on_buffer(uint16_t *buffer, int x, int y, char c, uint16_t color)
{
    if (x < 0 || y < 0 || x + 8 > EXAMPLE_LCD_H_RES || y + 8 > EXAMPLE_LCD_V_RES) {
        return; // Out of bounds
    }
    
    // Get character bitmap (use fallback for unknown chars)
    const uint8_t *char_bitmap = (c >= 0 && c < 128) ? simple_font[(int)c] : simple_font[0];
    
    // Draw 8x8 character
    for (int row = 0; row < 8; row++) {
        uint8_t bitmap_row = char_bitmap[row];
        for (int col = 0; col < 8; col++) {
            if (bitmap_row & (0x80 >> col)) { // Check if pixel should be on
                int pixel_x = x + col;
                int pixel_y = y + row;
                buffer[pixel_y * EXAMPLE_LCD_H_RES + pixel_x] = color;
            }
        }
    }
}

// Display text message on LCD
esp_err_t display_text_on_lcd(const char* text)
{
    if (xSemaphoreTake(image_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex for text display");
        return ESP_FAIL;
    }

    // Clear the screen first (fill with dark blue background)
    size_t screen_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(uint16_t);
    uint16_t *screen_buffer = malloc(screen_size);
    if (screen_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate screen buffer for text display");
        xSemaphoreGive(image_mutex);
        return ESP_ERR_NO_MEM;
    }
    
    // Fill screen with dark blue (0x001F in RGB565 - pure blue)
    uint16_t bg_color = 0x001F; // Dark blue background
    for (int i = 0; i < EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES; i++) {
        screen_buffer[i] = bg_color;
    }
    
    // Text display parameters
    uint16_t text_color = 0xFFFF; // White text
    int start_x = 40; // Start position X (centered-ish)
    int start_y = 100; // Start position Y (middle of screen)
    int char_spacing = 10; // Space between characters
    
    // Draw each character
    int x = start_x;
    int y = start_y;
    for (int i = 0; text[i] != '\0' && i < 30; i++) { // Limit to 30 chars
        char c = text[i];
        
        if (c == ' ') {
            x += char_spacing; // Space character
        } else if (c == '\n' || x + 8 >= EXAMPLE_LCD_H_RES - 10) {
            // New line or end of screen width
            x = start_x;
            y += 12; // Move to next line (8 pixel height + 4 pixel spacing)
            if (y + 8 >= EXAMPLE_LCD_V_RES) break; // Screen full
            if (c != '\n') {
                draw_char_on_buffer(screen_buffer, x, y, c, text_color);
                x += char_spacing;
            }
        } else {
            draw_char_on_buffer(screen_buffer, x, y, c, text_color);
            x += char_spacing;
        }
    }
    
    // Draw the screen buffer to LCD
    esp_err_t ret = esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES, screen_buffer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to draw text to LCD screen");
        free(screen_buffer);
        xSemaphoreGive(image_mutex);
        return ret;
    }
    
    free(screen_buffer);
    ESP_LOGI(TAG, "Text displayed on LCD: %s", text);
    
    xSemaphoreGive(image_mutex);
    return ESP_OK;
}

// Start WebSocket client dynamically
esp_err_t start_websocket_client(void)
{
    if (client != NULL) {
        ESP_LOGI(TAG, "WebSocket client already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Starting WebSocket client...");
    return init_websocket_client();
}

// Stop WebSocket client and cleanup
void stop_websocket_client(void)
{
    if (client != NULL) {
        ESP_LOGI(TAG, "Stopping WebSocket client...");
        esp_websocket_client_stop(client);
        esp_websocket_client_destroy(client);
        client = NULL;
        websocket_streaming = false;
        
        // Reset accumulated data
        accumulated_len = 0;
        accumulated_expected_len = 0;
        
        ESP_LOGI(TAG, "WebSocket client stopped and cleaned up");
    } else {
        ESP_LOGI(TAG, "WebSocket client already stopped");
    }
}

// Handle mode changes
void handle_mode_change(display_mode_t new_mode)
{
    ESP_LOGI(TAG, "Handling mode change to: %d", new_mode);
    current_mode = new_mode;
    
    switch (new_mode) {
        case MODE_IMAGE_STREAM:
            ESP_LOGI(TAG, "Switching to IMAGE STREAM mode");
            
            // Start WebSocket client if not running
            if (client == NULL) {
                esp_err_t ret = start_websocket_client();
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to start WebSocket client");
                    return;
                }
            }
            
            // Start streaming if connected
            if (client != NULL && esp_websocket_client_is_connected(client) && !websocket_streaming) {
                int ret = esp_websocket_client_send_text(client, "capture", strlen("capture"), portMAX_DELAY);
                if (ret >= 0) {
                    websocket_streaming = true;
                    ESP_LOGI(TAG, "Started websocket image streaming");
                } else {
                    ESP_LOGE(TAG, "Failed to start websocket streaming, ret=%d", ret);
                }
            }
            break;
            
        case MODE_PASSWORD_PROMPT:
            ESP_LOGI(TAG, "Switching to PASSWORD PROMPT mode");
            
            // Stop and cleanup WebSocket client completely
            stop_websocket_client();
            
            // Display password prompt (using simple text for better font support)
            esp_err_t display_ret = display_text_on_lcd("PASSWORD MODE");
            if (display_ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to display password prompt");
            }
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown mode: %d", new_mode);
            break;
    }
} 