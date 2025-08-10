set(srcs "pretty_effect.c"
    "websocket_lcd_main.c"
    "decode_image.c"
    "websocket_client.c"
    "wifi_connection.c"
    )

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "."
                    EMBED_FILES image.jpg
                    REQUIRES esp_http_client esp_lcd esp_websocket_client esp_jpeg esp_wifi nvs_flash)
