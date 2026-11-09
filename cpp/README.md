# GyroPointer (C++)

Bản C++ header-only của thuật toán điều khiển con trỏ TV bằng gyro (air mouse). Đây là **nguồn thuật toán duy nhất** của repo: cùng một đoạn code chạy trên TV và chạy offline để visualize.

| File | Vai trò |
|---|---|
| [`include/GyroPointerCore.h`](include/GyroPointerCore.h) | Thuật toán. Không đọc sensor hay đồng hồ: nhận `dt` và ω (vòng/giây), trả delta **kèm trạng thái nội bộ** (khóa, precision mode, ω sau lọc, gain) |
| [`include/GyroPointer.h`](include/GyroPointer.h) | Lớp bọc cho engine trên TV: đọc `getInputstate()`, đo `dt` bằng `steady_clock`, gọi core |
| [`tools/gyro_replay.cpp`](tools/gyro_replay.cpp) | Phát lại log sensor qua core, ghi CSV cho `imu_viz.py` |
| [`example/main.cpp`](example/main.cpp) | Ví dụ dùng `GyroPointer` với sensor giả lập |

## Pipeline

Mỗi frame gọi một lần (`GyroPointer::UpdateControlState()` trên TV, `GyroPointerCore::Update()` khi phát lại), trả về delta con trỏ (pixel):

1. **Input:** ω (vòng/giây × 2π → rad/s) và `dt` (trên TV đo bằng `std::chrono::steady_clock`), clamp 1–50 ms.
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

Nếu hệ thống cắt delta về số nguyên, đặt `kUseSubpixelAccumulator = true` trong `GyroPointerCore.h` (xem comment trong file để biết cách test).

## Build

Cần compiler hỗ trợ **C++17** (GCC 8+, Clang 7+, MSVC 2019+) và **CMake 3.14+**.

Build ra hai chương trình:

- `gyro_replay`: phát lại log sensor, dùng cho `imu_viz.py`.
- `gyro_pointer_example`: tự định nghĩa `float2` / `getInputstate` giả lập và cho chạy một chuyển động mẫu ở ~60 fps (đứng yên → xoay trái → dừng → ngẩng lên → dừng), in vị trí con trỏ ra màn hình.

### CMake (Linux / macOS / Windows)

```bash
cd cpp
cmake -S . -B build
cmake --build build --config Release
./build/gyro_pointer_example          # Windows: build\Release\gyro_pointer_example.exe
./build/gyro_replay ../demo.csv out.csv
```

### Không dùng CMake

```bash
cd cpp
mkdir -p build
g++ -std=c++17 -O2 -Wall -Iinclude tools/gyro_replay.cpp -o build/gyro_replay
g++ -std=c++17 -O2 -Wall -Iinclude example/main.cpp -o build/gyro_pointer_example
./build/gyro_pointer_example
```

Kết quả mong đợi của ví dụ: con trỏ bắt đầu ở (960, 540), đi sang trái khoảng 550 px khi xoay, **dừng hẳn** khi tay dừng (delta = 0, không trôi), sau đó đi lên khoảng 245 px.

## gyro_replay

```bash
./build/gyro_replay LOG.csv [OUT.csv] [--sensitivity N] [--stability N] [--fps F]
                    [--time-unit s|ms] [--yaw-axis AXIS] [--pitch-axis AXIS] [--screen WxH]
```

- Input: CSV có cột `t, gx, gy, gz` (vòng/giây); cột `phase` được chép sang output.
- `--fps 60` (mặc định): giống TV, mỗi frame chỉ đọc **sample mới nhất**. `--fps 0`: xử lý mọi sample với `dt` riêng.
- `--yaw-axis z --pitch-axis x` (mặc định): giống `GyroPointer.h`. Thêm `-` để đảo dấu, ví dụ `-y`.
- Output (bỏ `OUT.csv` hoặc dùng `-` để in ra stdout): mỗi frame một dòng gồm `t, phase, dt`, ω chưa lọc và sau lọc (rad/s), `gain`, `locked`, `precision`, ngưỡng đang dùng (rad/s), `dx, dy` và vị trí con trỏ `x, y` (bắt đầu ở giữa màn hình).

### Dùng trong project CMake khác

```cmake
add_subdirectory(path/to/imu-air-mouse/cpp)
target_link_libraries(your_target PRIVATE gyro_pointer)
```
