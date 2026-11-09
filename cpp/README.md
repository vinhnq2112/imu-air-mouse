# GyroPointer (C++)

Bản C++ header-only của thuật toán điều khiển con trỏ TV bằng gyro (air mouse). Toàn bộ thuật toán nằm trong một file: [`include/GyroPointer.h`](include/GyroPointer.h).

## Pipeline

Mỗi frame gọi `UpdateControlState()` một lần, trả về delta con trỏ (pixel):

1. **Đọc input:** ω (vòng/giây × 2π → rad/s) và `dt` đo bằng `std::chrono::steady_clock`, clamp 1–50 ms.
2. **Stillness lock:** hysteresis 2 ngưỡng. Tay dừng thì delta = 0. Sau khi đứng yên 0.4 s chuyển sang ngưỡng thấp (precision mode) để tinh chỉnh vào điểm nhỏ.
3. **Low-pass filter:** lọc rung tay, bắt đầu từ 0 sau khi mở khóa để con trỏ tăng tốc mượt.
4. **Gain curve:** sigmoid theo tốc độ đã lọc, ×0.6 khi nhắm chậm → ×3.0 khi quét nhanh.
5. **Tính delta:** `−ω · dt · pixelsPerRadian · gain`, dùng chung cho cả hai trục.

## Thiết lập cho người dùng

Chỉ có hai giá trị, thang 1–10, mặc định 5:

```cpp
pointer.SetSettings(sensitivity, stability);
```

| Thiết lập | Tăng lên thì |
|---|---|
| **Độ nhạy** (`sensitivity`) | Xoay ít, con trỏ đi xa hơn (mức 1: 46°, mức 5: 30°, mức 10: 10° để đi hết bề ngang màn hình) |
| **Độ ổn định** (`stability`) | Con trỏ bớt rung, bớt trôi khi dừng tay, nhưng kém phản ứng với cử động rất nhỏ |

## Tích hợp vào engine

`GyroPointer.h` không tự định nghĩa các phần sau, engine phải cung cấp **trước khi** `#include`:

- Kiểu `float2` có `x()` / `y()` trả về tham chiếu.
- Các id input `angularVelocityX`, `angularVelocityZ`.
- Hàm `getInputstate(id)` trả về vận tốc góc theo **vòng/giây** (1.0 = 360°/s). Đây cũng là đơn vị chuẩn của cột `gx, gy, gz` trong CSV của tool Python, nên log ghi từ sensor dùng chung được cho cả hai.

```cpp
#include "MyEngine.h"      // float2, getInputstate, angularVelocity*
#include "GyroPointer.h"

GyroPointer pointer;       // một instance dùng suốt vòng đời app

void OnFrame()
{
    float2 delta = pointer.UpdateControlState();
    MoveCursorBy(delta);
}
```

Nếu hệ thống cắt delta về số nguyên, đặt `kUseSubpixelAccumulator = true` trong header (xem comment trong file để biết cách test).

## Build ví dụ

Cần compiler hỗ trợ **C++17** (GCC 8+, Clang 7+, MSVC 2019+) và **CMake 3.14+**.

`example/main.cpp` tự định nghĩa `float2` / `getInputstate` giả lập và cho chạy một chuyển động mẫu ở ~60 fps (đứng yên → xoay trái → dừng → ngẩng lên → dừng), in vị trí con trỏ ra màn hình.

### CMake (Linux / macOS / Windows)

```bash
cd cpp
cmake -S . -B build
cmake --build build --config Release
./build/gyro_pointer_example          # Windows: build\Release\gyro_pointer_example.exe
```

### Không dùng CMake

```bash
cd cpp
g++ -std=c++17 -O2 -Wall -Iinclude example/main.cpp -o gyro_pointer_example
./gyro_pointer_example
```

Kết quả mong đợi: con trỏ bắt đầu ở (960, 540), đi sang trái khoảng 550 px khi xoay, **dừng hẳn** khi tay dừng (delta = 0, không trôi), sau đó đi lên khoảng 245 px.

### Dùng trong project CMake khác

```cmake
add_subdirectory(path/to/imu-air-mouse/cpp)
target_link_libraries(your_target PRIVATE gyro_pointer)
```
