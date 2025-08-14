# GPIO Mode Detection Feature

## Tổng quan
Tính năng này cho phép ESP32 chuyển đổi giữa 2 mode hoạt động dựa trên trạng thái của GPIO:

- **GPIO LOW (0V)**: MODE_IMAGE_STREAM - Nhận và hiển thị hình ảnh từ WebSocket server
- **GPIO HIGH (3.3V)**: MODE_PASSWORD_PROMPT - Hiển thị thông báo "Vui lòng nhập mật khẩu"

## Hardware Setup

### GPIO Configuration
- **GPIO Pin**: GPIO 5 (có thể thay đổi trong `websocket_client.h`)
- **Pull-up**: Enabled (internal pull-up resistor)
- **Logic Level**: 
  - LOW (0V) = Image streaming mode
  - HIGH (3.3V) = Password prompt mode

### Wiring
```
ESP32 GPIO 5 ---- Switch ---- GND
                |
                +--- Internal Pull-up (3.3V)
```

Hoặc dùng jumper wire để test:
- Nối GPIO 5 với GND → Image mode
- Nối GPIO 5 với 3.3V (hoặc để floating) → Password mode

## Software Changes

### Files Modified
1. `websocket_client.h` - Added mode detection configuration và function declarations
2. `websocket_client.c` - Implemented mode detection và handling logic
3. `websocket_lcd_main.c` - Added GPIO initialization

### Key Functions
- `init_mode_detection_gpio()` - Initialize GPIO cho mode detection
- `get_current_mode()` - Đọc GPIO state và return current mode
- `display_text_on_lcd()` - Hiển thị text message lên LCD
- `handle_mode_change()` - Xử lý khi chuyển đổi mode

## How It Works

### Mode Detection Loop
- WebSocket task kiểm tra GPIO state mỗi 100ms
- Khi detect mode change, gọi `handle_mode_change()`

### Mode Behaviors

#### IMAGE_STREAM Mode (GPIO LOW)
- Gửi "capture" command đến WebSocket server
- Process incoming image data
- Hiển thị images lên LCD
- Set `websocket_streaming = true`

#### PASSWORD_PROMPT Mode (GPIO HIGH)
- Gửi "stop" command đến WebSocket server
- Stop processing image data
- Clear LCD screen (fill with black)
- Hiển thị message "Vui lòng nhập mật khẩu"
- Set `websocket_streaming = false`

## Configuration

### Change GPIO Pin
Trong `websocket_client.h`:
```c
#define MODE_DETECT_GPIO 5  // Change to your desired pin
```

### Modify Text Message
Trong `websocket_client.c`, function `handle_mode_change()`:
```c
display_text_on_lcd("Your custom message");
```

## Building and Running

1. Build project:
```bash
idf.py build
```

2. Flash to ESP32:
```bash
idf.py flash monitor
```

3. Test mode switching bằng cách thay đổi GPIO 5 state

## Logging
Monitor serial output để xem mode changes:
```
I (123456) WEBSOCKET_CLIENT: Mode changed from 0 to 1
I (123456) WEBSOCKET_CLIENT: Switching to PASSWORD PROMPT mode
I (123456) WEBSOCKET_CLIENT: Stopped websocket image streaming
I (123456) WEBSOCKET_CLIENT: Text display: Vui lòng nhập mật khẩu
```

## Notes

### Text Display Limitation
Hiện tại function `display_text_on_lcd()` chỉ clear screen và log text message. Để hiển thị text thực sự lên LCD, bạn cần:
1. Thêm font library (ví dụ: lvgl, u8g2)
2. Implement text rendering functions
3. Modify `display_text_on_lcd()` để vẽ text

### WebSocket Server Compatibility
Code assumes WebSocket server hỗ trợ:
- "capture" command để start streaming
- "stop" command để stop streaming

Nếu server không hỗ trợ "stop", bạn có thể remove hoặc modify command trong `handle_mode_change()`.

### Performance
- Mode detection frequency: 100ms
- GPIO read is very fast (microseconds)
- Mode switching gần như instant
