# AeroLink Drone Ground Station

Trạm điều khiển mặt đất (GCS) cho drone, viết bằng **C++17 / Qt 6**. App giao tiếp với flight controller qua cổng serial (USB, UART hoặc module LoRa ở chế độ transparent) bằng **MAVLink** (ArduPilot) hoặc **MSP** (Betaflight/INAV, chỉ đọc telemetry).

```
PC App <-> COM/USB <-> LoRa ground <-> LoRa air <-> UART/TELEM <-> Flight Controller
```

## Tính năng

- **Kết nối:** tự nhận diện MAVLink hoặc MSP trên cùng một cổng, tự reconnect khi rút/cắm lại thiết bị, nhớ cổng và baud lần trước.
- **Lệnh bay (MAVLink):** ARM, DISARM, TAKE OFF, LAND, RETURN HOME, GO TO toạ độ.
- **An toàn:**
  - Nút bay chỉ bấm được khi đã kết nối.
  - ARM, DISARM và TAKE OFF có hộp xác nhận.
  - Toạ độ và độ cao được kiểm tra trước khi gửi.
  - Cảnh báo **HEARTBEAT LOST** khi mất sóng quá 5 giây.
- **Telemetry:** pin (% và V), GPS (fix, số vệ tinh), vị trí, độ cao tương đối, tốc độ, roll/pitch/yaw, flight mode và trạng thái ARMED/DISARMED, RSSI radio.
- **Link health:** RX/TX B/s, packet/s, tuổi của heartbeat và message gần nhất.
- **Bản đồ OpenStreetMap:** zoom bằng cuộn chuột, kéo để di chuyển. Tile được cache xuống đĩa nên vùng đã xem vẫn hiện khi mất mạng.
- **Biểu đồ attitude:** roll, pitch, yaw theo thời gian thực.
- **Event log:** COMMAND_ACK, STATUSTEXT và các lệnh đã gửi.

## Yêu cầu

- Qt 6 với các module **Widgets, SerialPort, Network**
- CMake 3.16+
- Trình biên dịch C++17 tương thích với bản Qt đã cài (MinGW hoặc MSVC trên Windows, GCC hoặc Clang trên Linux)

## Build và chạy

### Windows (script có sẵn)

```bat
run_cpp.bat
```

Script sẽ configure, build, chạy test parser rồi mở app. Đường dẫn mặc định là `C:\Qt\6.11.0\mingw_64`. Nếu Qt cài ở chỗ khác, đặt biến môi trường trước khi chạy:

```bat
set QT_DIR=D:\Qt\6.8.3\mingw_64
set MINGW_DIR=D:\Qt\Tools\mingw1310_64
run_cpp.bat
```

Có thể override các biến `QT_DIR`, `CMAKE_EXE`, `NINJA_EXE`, `MINGW_DIR`.

### CMake (mọi hệ điều hành)

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=<đường-dẫn-Qt>
cmake --build build --config Release
ctest --test-dir build --output-on-failure   # test parser MAVLink/MSP
```

File chạy: `build/DroneAutomationCpp` (Windows: `build\DroneAutomationCpp.exe`, hoặc `build\Release\...` với generator multi-config). Cũng có thể mở `CMakeLists.txt` bằng Qt Creator.

## Thử không cần drone (simulator)

`test_mavlink.py` giả lập một drone gửi HEARTBEAT, ATTITUDE, GPS, vị trí và pin.

1. Tạo một cặp cổng COM ảo nối với nhau, ví dụ bằng [com0com](https://com0com.sourceforge.net/) trên Windows (COM7 ↔ COM8) hoặc `socat` trên Linux.
2. Chạy simulator trên một đầu:
   ```bash
   pip install pymavlink pyserial
   python test_mavlink.py COM7
   ```
3. Trong app, chọn đầu còn lại (COM8), baud `115200`, rồi bấm **Connect**.

Simulator chỉ gửi telemetry và không trả lời lệnh, nên các nút ARM/TAKE OFF sẽ không nhận được ACK.

## Sử dụng

1. **Refresh**, chọn cổng COM, chọn baud (thường là 57600 hoặc 115200), rồi bấm **Connect**.
2. Chờ trạng thái chuyển sang **Connected** và Link Health báo **Healthy**.
3. **TAKE OFF:** app tự chuyển sang mode GUIDED. Độ cao lấy từ ô Altitude (mặc định 5 m, giới hạn 0–120 m, tương đối so với home).
4. **GO TO TARGET:** nhập latitude, longitude, altitude rồi bấm nút. Drone được chuyển sang GUIDED và bay tới điểm đó.
5. **DISARM** tắt động cơ ngay lập tức. Nếu drone đang bay, drone sẽ **rơi**, nên chỉ dùng khi khẩn cấp hoặc đã hạ cánh.

> ⚠️ Luôn thử app với drone **đã tháo cánh** trước. Kiểm tra failsafe trên flight controller và tuân thủ quy định bay tại địa phương. Giới hạn 120 m nằm ở `MaxAltitudeM` trong `main.cpp`.

Hướng dẫn chi tiết từng màn hình nằm trong [APP_USAGE_GUIDE_VI.txt](APP_USAGE_GUIDE_VI.txt). Giải thích code và các cách kết nối (LoRa, USB, telemetry radio) nằm trong [CODE_AND_CONNECTION_GUIDE_VI.txt](CODE_AND_CONNECTION_GUIDE_VI.txt).

## MAVLink message hỗ trợ

| Nhận từ drone | Gửi lên drone |
|---|---|
| HEARTBEAT, SYS_STATUS, GPS_RAW_INT, GLOBAL_POSITION_INT, ATTITUDE, RADIO_STATUS, COMMAND_ACK, STATUSTEXT | HEARTBEAT (GCS), REQUEST_DATA_STREAM, COMMAND_LONG (ARM/DISARM, TAKEOFF, LAND, RTL), SET_MODE, SET_POSITION_TARGET_GLOBAL_INT |

Nhận được cả MAVLink v1 và v2; app gửi đi bằng v1. Tên flight mode theo bảng của **ArduCopter**.

MSP (chỉ đọc): API/FC version, STATUS, ATTITUDE, ALTITUDE, RAW_GPS, ANALOG, BATTERY_STATE.

## Cấu trúc mã nguồn

```
main.cpp                 UI Qt, MavlinkWorker (chạy trong QThread riêng), bản đồ, biểu đồ
mavlink_protocol.h       Parser/encoder MAVLink + MSP (CRC, framing, resync)
tests/test_protocol.cpp  Unit test parser, dùng frame mẫu sinh từ pymavlink
test_mavlink.py          Simulator drone giả lập
run_cpp.bat              Script build và chạy trên Windows
```

## Giới hạn hiện tại

- Lệnh bay chỉ hỗ trợ ArduPilot qua MAVLink. Link MSP chỉ đọc telemetry.
- Chưa có: tự dò baud, xem gói MAVLink thô, xuất log ra file, upload mission.
