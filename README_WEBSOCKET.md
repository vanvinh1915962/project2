# WebSocket LCD Client

Project này tạo một WebSocket client để kết nối đến ESP32-CAM server và hiển thị hình ảnh lên LCD.

## Cấu hình

### 1. Cập nhật WiFi Credentials

Trong file `main/wifi_connection.h`, cập nhật thông tin WiFi:

```c
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
```

### 2. Cập nhật IP của Camera Server

Trong file `main/websocket_client.h`, cập nhật IP của camera server:

```c
#define WEBSOCKET_SERVER_URI "ws://YOUR_CAMERA_IP:80/ws"
```

Thay `YOUR_CAMERA_IP` bằng IP thực tế của ESP32-CAM.

**Lưu ý**: Code đã được cập nhật để tự động tìm IP camera server. Nếu không tìm thấy, bạn có thể thử các IP phổ biến:
- `192.168.1.100` - IP mặc định
- `192.168.4.1` - ESP32-CAM AP mode
- `192.168.1.101-103` - Các IP khác

### 3. Kết nối WiFi

Đảm bảo ESP32 với LCD đã kết nối WiFi và có thể truy cập đến camera server.

## Cách hoạt động

1. **Khởi tạo LCD**: LCD được khởi tạo với cấu hình SPI và hiển thị sẵn sàng
2. **Kết nối WebSocket**: Client kết nối đến camera server qua WebSocket
3. **Gửi yêu cầu**: Khi kết nối thành công, client gửi message "capture" để yêu cầu hình ảnh
4. **Nhận dữ liệu**: Server gửi hình ảnh dưới dạng base64
5. **Decode và hiển thị**: Client decode base64 thành JPEG, sau đó decode JPEG thành pixel data và hiển thị lên LCD

## Build và Flash

```bash
# Build project
idf.py build

# Flash to ESP32
idf.py flash monitor
```

## Cấu trúc file

- `websocket_client.h`: Header file cho WebSocket client
- `websocket_client.c`: Implementation của WebSocket client
- `websocket_lcd_main.c`: Main function khởi tạo LCD và WebSocket
- `decode_image.c`: Hỗ trợ decode JPEG (từ project gốc)

## Troubleshooting

1. **Không kết nối được WebSocket**: Kiểm tra IP camera server và kết nối WiFi
2. **LCD không hiển thị**: Kiểm tra cấu hình pin LCD
3. **Hình ảnh bị lỗi**: Kiểm tra buffer size và memory allocation

## Lưu ý

- Đảm bảo camera server đang chạy và có thể truy cập
- LCD resolution: 320x240 pixels
- WebSocket buffer size: 32KB (có thể điều chỉnh nếu cần) 