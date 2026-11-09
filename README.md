# imu-air-mouse

Thuật toán điều khiển con trỏ TV bằng gyro (air mouse), kèm công cụ **offline** để xem thuật toán xử lý một bản ghi sensor như thế nào.

**Một nguồn thuật toán duy nhất: C++.** Đoạn code chạy trên TV ([`cpp/include/GyroPointerCore.h`](cpp/include/GyroPointerCore.h)) cũng là đoạn code dùng để phát lại log. Python chỉ vẽ kết quả, không chứa thuật toán, nên những gì thấy trên biểu đồ đúng là những gì xảy ra trên TV.

![Ví dụ trên dữ liệu mẫu](overview.png)

## Cấu trúc

```
log sensor (CSV, vòng/giây)
        │
        ▼
cpp/build/gyro_replay      chạy log qua GyroPointerCore (giống hệt trên TV: 1 lần mỗi frame, 60 fps)
        │                  → CSV output: delta, vị trí con trỏ, trạng thái khóa / precision mode, gain...
        ▼
imu_viz.py                 vẽ vết con trỏ, trạng thái, tốc độ góc + ngưỡng, tính số đo
```

| Thư mục / file | Vai trò |
|---|---|
| `cpp/include/GyroPointerCore.h` | Thuật toán, không phụ thuộc engine |
| `cpp/include/GyroPointer.h` | Lớp bọc cho engine trên TV (`getInputstate`, `steady_clock`) |
| `cpp/tools/gyro_replay.cpp` | Phát lại log sensor qua thuật toán |
| `imu_viz.py` | Chỉ visualize output |

## Cài đặt

**1. Build phần C++** (cần CMake 3.14+ và compiler C++17, chi tiết ở [`cpp/README.md`](cpp/README.md)):

```bash
cmake -S cpp -B cpp/build
cmake --build cpp/build --config Release
```

**2. Cài Python 3.9+ và các package:**

```bash
python -m venv .venv                 # tuỳ chọn: tạo môi trường riêng
source .venv/bin/activate            # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

## Chạy

```bash
python imu_viz.py demo.csv                         # cửa sổ tương tác
python imu_viz.py my_log.csv --save report.png     # xuất ảnh, không mở cửa sổ
python imu_viz.py my_log.csv --stability 7         # chọn sẵn thiết lập
python imu_viz.py my_log.csv --fps 0               # xử lý mọi sample thay vì 60 fps
python imu_viz.py --make-demo demo.csv             # tạo lại dữ liệu mẫu (mô phỏng)
```

Trong cửa sổ, kéo **Độ nhạy** / **Độ ổn định** (1–10) thì tool chạy lại thuật toán C++ với thiết lập mới và vẽ lại ngay. Thanh **Khung thời gian** chọn đoạn vết con trỏ hiển thị.

Có thể chạy `gyro_replay` riêng rồi đưa file output cho tool:

```bash
cpp/build/gyro_replay my_log.csv out.csv --stability 7
python imu_viz.py out.csv
```

### Đọc biểu đồ

- **Màn hình ảo:** vết con trỏ, chấm cam là chỗ con trỏ bị khóa (tay dừng).
- **Vị trí con trỏ:** X, Y theo thời gian.
- **Tốc độ góc:** |ω| chưa lọc (xám) và sau lọc (xanh), ngưỡng khóa / mở khóa đang dùng. Nền cam = đang khóa, nền xanh lá = precision mode.

### Số đo

| Số đo | Ý nghĩa | Cần nhãn |
|---|---|---|
| **Jitter (px)** | độ rung của con trỏ khi tay giữ yên | `still` |
| **Drift (px/s)** | tốc độ con trỏ trôi khi tay giữ yên | `still` |
| **Trôi sau dừng (px)** | quãng đường con trỏ còn đi trong 1 s sau khi dừng tay (trôi tiếp, quay ngược) | `still` |
| **Độ trễ (ms)** | con trỏ chậm hơn chuyển động tay bao lâu | |
| **Lệch đường thẳng (px)** | độ lệch khi kẻ thẳng | `line` |
| **Khóa khi đứng yên (%)** | tỉ lệ thời gian con trỏ bị khóa khi tay giữ yên (càng cao càng tốt) | `still` |

## Định dạng log sensor

**Chuẩn đơn vị của cả repo: vận tốc góc tính bằng vòng/giây (rev/s)**, giống đầu ra của sensor. Ghi log thẳng từ `getInputstate()` là dùng được.

| Cột | Ý nghĩa |
|---|---|
| `t` | thời gian (s; thêm `--time-unit ms` nếu là ms) |
| `gx, gy, gz` | vận tốc góc, **vòng/giây** (1.0 = 360°/s) |
| `phase` | *(tuỳ chọn)* `still` / `line` / `flick` / `circle`, dùng để tính số đo |

Các cột khác (`ax, ay, az`, ...) được bỏ qua.

Quy ước trục giống trên TV: **`gz` (yaw) → con trỏ ngang, `gx` (pitch) → con trỏ dọc**. Nếu log khác, dùng `--yaw-axis` / `--pitch-axis`, ví dụ `--yaw-axis -y`.

`demo.csv` là dữ liệu **mô phỏng** (rung tay, vẩy nhanh, vẽ tròn khi cầm nghiêng 30°), chỉ để thử tool. Muốn đánh giá thật, hãy ghi log từ remote.
