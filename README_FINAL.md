# WebSocket LCD Client - Final Version

## ✅ Đã sửa các vấn đề:

1. **NVS initialization** - Thêm nvs_flash_init() trước khi khởi tạo WiFi
2. **Memory allocation** - Bỏ pretty_effect_init() để tiết kiệm memory
3. **Auto-discovery IP** - Tự động tìm IP camera server
4. **Error handling** - Thêm error handling tốt hơn

## 🚀 Cách hoạt động:

1. **Khởi tạo NVS** → Lưu trữ cấu hình
2. **Kết nối WiFi** → Kết nối đến WiFi network
3. **Khởi tạo LCD** → Cấu hình SPI và LCD display
4. **Auto-discovery IP** → Tự động tìm IP camera server
5. **Kết nối WebSocket** → Kết nối đến camera server
6. **Gửi yêu cầu** → Gửi "capture" để yêu cầu hình ảnh
7. **Nhận và hiển thị** → Nhận base64, decode JPEG, hiển thị lên LCD

## 📋 Cấu hình:

### WiFi Credentials (wifi_connection.h):
```c
#define WIFI_SSID "VienDanBac"
#define WIFI_PASSWORD "Thienthanh1994@"
```

### Camera Server IP (websocket_client.h):
```c
#define WEBSOCKET_SERVER_URI "ws://192.168.1.100:80/ws"
```

## 🔧 Build và Flash:

```bash
cd tjpgd
idf.py build
idf.py flash monitor
```

## 📊 Expected Output:

```
I (544) WEBSOCKET_LCD_MAIN: Starting WebSocket LCD client...
I (554) WIFI_CONNECTION: WiFi initialized successfully
I (564) WIFI_CONNECTION: Connected to WiFi
I (574) WIFI_CONNECTION: got ip:192.168.1.153
I (584) WEBSOCKET_CLIENT: LCD display initialized successfully!
I (594) WEBSOCKET_CLIENT: Found camera server at 192.168.1.100
I (604) WEBSOCKET_CLIENT: WebSocket client initialized successfully with auto-discovered IP
I (614) WEBSOCKET_LCD_MAIN: WebSocket LCD client started successfully
I (624) WEBSOCKET_CLIENT: WEBSOCKET_EVENT_CONNECTED
I (634) WEBSOCKET_CLIENT: Capture message sent successfully
I (644) WEBSOCKET_CLIENT: WEBSOCKET_EVENT_DATA
I (654) WEBSOCKET_CLIENT: Received data length: 12345
I (664) WEBSOCKET_CLIENT: Base64 decoded successfully, JPEG size: 9876
I (674) WEBSOCKET_CLIENT: JPEG decoded successfully: 640x480
I (684) WEBSOCKET_CLIENT: Image displayed on LCD successfully
```

## 🛠️ Troubleshooting:

1. **WiFi không kết nối**: Kiểm tra SSID và password
2. **LCD không hiển thị**: Kiểm tra cấu hình pin LCD
3. **WebSocket không kết nối**: Kiểm tra IP camera server
4. **Memory error**: Đảm bảo ESP32 có đủ memory

## 📁 File structure:

- `websocket_client.h/c` - WebSocket client implementation
- `wifi_connection.h/c` - WiFi connection
- `websocket_lcd_main.c` - Main application
- `decode_image.c` - JPEG decoding support

## 🎯 Mục đích:

Nhận hình ảnh từ ESP32-CAM qua WebSocket và hiển thị real-time lên LCD! 