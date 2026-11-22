Drone Automation C++/Qt

Yeu cau:
- Qt 6 co modules: Widgets, SerialPort, Network
- CMake 3.16+
- C++ compiler phu hop voi Qt da cai

Build bang terminal:
1) cmake -S . -B build
2) cmake --build build --config Release

Test parser MAVLink/MSP:
3) ctest --test-dir build --output-on-failure

Chay:
- Windows multi-config: build\Release\DroneAutomationCpp.exe
- Windows single-config: build\DroneAutomationCpp.exe

Ghi chu:
- Giao dien tieng Anh duoc dung truc tiep trong main.cpp, khong can load drone_control.ui khi chay.
- Ban C++ dung Qt SerialPort va MAVLink parser/encoder toi thieu cho cac message/lệnh
  dang co trong ban Python: HEARTBEAT, SYS_STATUS, GPS_RAW_INT, GLOBAL_POSITION_INT,
  ATTITUDE, RADIO_STATUS, COMMAND_LONG, SET_MODE, SET_POSITION_TARGET_GLOBAL_INT.
- Parser/encoder MAVLink + MSP nam trong mavlink_protocol.h (co test o tests/test_protocol.cpp).
- Neu muon build trong Qt Creator, mo CMakeLists.txt va bam Configure/Build.
