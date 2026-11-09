#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
imu_viz.py — Công cụ OFFLINE để xem thuật toán con trỏ air-mouse hoạt động thế nào.

Python KHÔNG chứa thuật toán. Toàn bộ xử lý nằm ở C++ (cpp/include/GyroPointerCore.h),
đúng đoạn code chạy trên TV. Tool này chỉ:
    1. gọi cpp/build/gyro_replay để chạy log sensor qua thuật toán C++,
    2. vẽ kết quả: vết con trỏ, trạng thái khóa / precision mode, tốc độ góc và ngưỡng,
    3. tính số đo (jitter, drift, độ trễ, lệch đường thẳng, trôi sau khi dừng).

Cách chạy (build C++ trước, xem cpp/README.md):
    python imu_viz.py demo.csv                     # cửa sổ tương tác, slider Độ nhạy / Độ ổn định
    python imu_viz.py my_log.csv --save report.png # xuất ảnh, không mở cửa sổ
    python imu_viz.py replay_out.csv               # vẽ file output có sẵn của gyro_replay
    python imu_viz.py --make-demo demo.csv         # tạo lại dữ liệu mẫu (mô phỏng)

Cần: Python 3.9+, numpy, pandas, matplotlib (pip install -r requirements.txt).

Định dạng log sensor (dòng đầu là header):
    t               thời gian (giây; thêm --time-unit ms nếu là mili giây)
    gx, gy, gz      vận tốc góc, VÒNG/GIÂY (1.0 = 360°/s) — chuẩn đơn vị của cả repo
    phase           (tuỳ chọn) nhãn đoạn test: still / line / flick / circle / ...
                    -> 'still' để tính jitter, drift, trôi sau khi dừng;
                       'line' để tính độ lệch đường thẳng.
Các cột khác (ax, ay, az, ...) được bỏ qua.

Quy ước trục giống trên TV: gz (yaw) -> con trỏ ngang, gx (pitch) -> con trỏ dọc.
Đổi bằng --yaw-axis / --pitch-axis, ví dụ: --yaw-axis -y
"""

import argparse
import io
import os
import shutil
import subprocess
import sys

import numpy as np
import pandas as pd

REPO_DIR = os.path.dirname(os.path.abspath(__file__))

PHASE_VI = {
    "still": "Đứng yên",
    "line": "Kẻ thẳng",
    "flick": "Vẩy nhanh",
    "circle": "Vẽ tròn",
}

OUTPUT_COLUMNS = {"t", "locked", "precision", "dx", "dy", "x", "y"}


# ----------------------------------------------------------------------------
# Chạy thuật toán C++
# ----------------------------------------------------------------------------
def find_replay(explicit=None):
    """Tìm file chạy gyro_replay (đã build từ cpp/)."""
    if explicit:
        if not os.path.isfile(explicit):
            sys.exit(f"Không tìm thấy gyro_replay tại: {explicit}")
        return explicit
    names = ["gyro_replay", "gyro_replay.exe"]
    dirs = [os.path.join(REPO_DIR, "cpp", "build", sub) for sub in ("", "Release", "Debug", "RelWithDebInfo")]
    for d in dirs:
        for n in names:
            p = os.path.join(d, n)
            if os.path.isfile(p):
                return p
    found = shutil.which("gyro_replay")
    if found:
        return found
    sys.exit(
        "Chưa có gyro_replay. Build phần C++ trước:\n"
        "    cmake -S cpp -B cpp/build\n"
        "    cmake --build cpp/build --config Release\n"
        "hoặc chỉ định đường dẫn bằng --replay PATH"
    )


def run_replay(exe, log_path, args, sensitivity, stability):
    """Chạy log qua thuật toán C++, trả về DataFrame output (một dòng mỗi frame)."""
    cmd = [exe, log_path, "-",
           "--sensitivity", str(int(sensitivity)), "--stability", str(int(stability)),
           "--fps", str(args.fps), "--time-unit", args.time_unit,
           "--yaw-axis", args.yaw_axis, "--pitch-axis", args.pitch_axis,
           "--screen", f"{args.screen[0]}x{args.screen[1]}"]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        sys.exit(f"gyro_replay lỗi:\n{proc.stderr.strip()}")
    return pd.read_csv(io.StringIO(proc.stdout), keep_default_na=False)


def load_output(df):
    """Chuẩn hóa DataFrame output của gyro_replay thành dict numpy."""
    phase = df["phase"].astype(str).str.strip().to_numpy() if "phase" in df else None
    if phase is not None and not np.any(phase != ""):
        phase = None
    col = lambda name: df[name].to_numpy(float)
    return dict(
        t=col("t"), phase=phase,
        yaw_raw=col("yaw_raw"), pitch_raw=col("pitch_raw"),
        yaw_filt=col("yaw_filt"), pitch_filt=col("pitch_filt"),
        gain=col("gain"), locked=col("locked") > 0.5, precision=col("precision") > 0.5,
        enter_thr=col("enter_thr"), exit_thr=col("exit_thr"),
        dx=col("dx"), dy=col("dy"), x=col("x"), y=col("y"),
    )


# ----------------------------------------------------------------------------
# Số đo (chỉ phân tích output, không xử lý tín hiệu)
# ----------------------------------------------------------------------------
def segments(mask, min_len=10):
    segs, start = [], None
    for i, m in enumerate(np.append(mask, False)):
        if m and start is None:
            start = i
        elif not m and start is not None:
            if i - start >= min_len:
                segs.append((start, i))
            start = None
    return segs


def compute_metrics(r):
    t, phase = r["t"], r["phase"]
    pos = np.stack([r["x"], r["y"]], axis=1)
    still = phase == "still" if phase is not None else np.zeros(len(t), bool)
    line = phase == "line" if phase is not None else np.zeros(len(t), bool)
    dt = np.median(np.diff(t))

    # Jitter (độ rung quanh xu hướng) & drift (tốc độ trôi) trong các đoạn đứng yên
    jit, drift = [], []
    for s, e in segments(still):
        tt = t[s:e] - t[s:e].mean()
        slopes, resid2 = [], np.zeros(e - s)
        for k in range(2):
            p = np.polyfit(tt, pos[s:e, k], 1)
            slopes.append(p[0])
            resid2 += (pos[s:e, k] - np.polyval(p, tt)) ** 2
        jit.append(resid2)
        drift.append(np.hypot(*slopes))
    jitter = float(np.sqrt(np.concatenate(jit).mean())) if jit else np.nan
    drift = float(np.mean(drift)) if drift else np.nan

    # Trôi sau khi dừng: quãng đường con trỏ còn đi trong 1 s đầu của đoạn đứng yên
    # ngay sau một đoạn di chuyển (gồm cả trôi tiếp và quay ngược).
    settle = []
    for s, e in segments(still):
        if s == 0:
            continue
        end = min(e, s + int(round(1.0 / dt)))
        settle.append(np.abs(np.diff(pos[s:end], axis=0)).sum(axis=1).sum())
    settle = float(np.mean(settle)) if settle else np.nan

    # Độ trễ: lệch thời gian giữa ω chưa lọc và vận tốc con trỏ (cross-correlation)
    moving = ~still if phase is not None else np.ones(len(t), bool)
    ref, vx = -r["yaw_raw"] * moving, r["dx"] * moving
    latency = np.nan
    if moving.sum() > 30 and np.any(vx):
        best, lag = -np.inf, 0
        for L in range(0, int(0.3 / dt)):
            a_, b_ = ref[: len(ref) - L], vx[L:]
            den = np.linalg.norm(a_) * np.linalg.norm(b_)
            c = (a_ @ b_) / den if den > 0 else 0
            if c > best:
                best, lag = c, L
        latency = lag * dt * 1000

    # Độ lệch khỏi đường thẳng khi kẻ thẳng
    dev = []
    for s, e in segments(line):
        p = pos[s:e] - pos[s:e].mean(axis=0)
        _, _, vt = np.linalg.svd(p, full_matrices=False)
        dev.append(p @ vt[1])
    straight = float(np.sqrt(np.mean(np.concatenate(dev) ** 2))) if dev else np.nan

    locked_still = float(r["locked"][still].mean() * 100) if still.any() else np.nan

    return dict(jitter=jitter, drift=drift, settle=settle, latency=latency,
                straight=straight, locked_still=locked_still)


def fmt(v, digits=1):
    return "–" if v is None or np.isnan(v) else f"{v:.{digits}f}"


METRIC_COLUMNS = [
    ("jitter", "Jitter (px)"),
    ("drift", "Drift (px/s)"),
    ("settle", "Trôi sau dừng (px)"),
    ("latency", "Độ trễ (ms)"),
    ("straight", "Lệch đường thẳng (px)"),
    ("locked_still", "Khóa khi đứng yên (%)"),
]


# ----------------------------------------------------------------------------
# Giao diện
# ----------------------------------------------------------------------------
def run_ui(source, args):
    """source: ('log', path, exe) để chạy C++ theo slider, hoặc ('output', DataFrame, name)."""
    import matplotlib
    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.widgets import RangeSlider, Slider

    W, H = args.screen
    interactive_settings = source[0] == "log"
    settings = dict(sensitivity=args.sensitivity, stability=args.stability)

    def compute():
        if interactive_settings:
            return load_output(run_replay(source[2], source[1], args,
                                          settings["sensitivity"], settings["stability"]))
        return load_output(source[1])

    r = compute()
    t = r["t"]
    name = os.path.basename(source[1] if interactive_settings else source[2])

    plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False})
    fig = plt.figure(figsize=(16, 9.4))
    if not args.save:
        fig.canvas.manager.set_window_title("IMU air-mouse — thuật toán C++")
    title = fig.suptitle("", fontsize=11)

    ax_scr = fig.add_axes([0.04, 0.34, 0.48, 0.56])
    ax_pos = fig.add_axes([0.58, 0.67, 0.40, 0.23])
    ax_w = fig.add_axes([0.58, 0.36, 0.40, 0.26], sharex=ax_pos)
    ax_tbl = fig.add_axes([0.04, 0.02, 0.94, 0.13])
    ax_tbl.axis("off")

    # Màn hình ảo
    ax_scr.set_xlim(-20, W + 20)
    ax_scr.set_ylim(H + 20, -20)
    ax_scr.set_aspect("equal")
    ax_scr.add_patch(plt.Rectangle((0, 0), W, H, fill=False, lw=1.5, ec="#444"))
    ax_scr.set_title(f"Màn hình ảo {W}×{H} — vết con trỏ (chấm = vị trí cuối khung thời gian)")
    ax_scr.set_xticks([0, W // 2, W])
    ax_scr.set_yticks([0, H // 2, H])
    trail, = ax_scr.plot([], [], color="#0072B2", lw=1.2)
    dot, = ax_scr.plot([], [], "o", color="#0072B2", ms=7, mec="white")
    stops, = ax_scr.plot([], [], "o", color="#D55E00", ms=3.5, alpha=0.8, label="điểm dừng (khóa)")
    ax_scr.legend(loc="lower right", fontsize=8, frameon=False)

    # Nhãn các đoạn test
    if r["phase"] is not None:
        ph = r["phase"]
        change = np.flatnonzero(np.r_[True, ph[1:] != ph[:-1], True])
        for j, (s, e) in enumerate(zip(change[:-1], change[1:])):
            ax_pos.text((t[s] + t[e - 1]) / 2, 1.02, PHASE_VI.get(ph[s], ph[s]),
                        transform=ax_pos.get_xaxis_transform(), ha="center", va="bottom", fontsize=8)

    ax_pos.set_ylabel("Vị trí con trỏ (px)")
    ax_pos.set_ylim(-20, max(W, H) + 20)
    l_x, = ax_pos.plot(t, r["x"], color="#0072B2", lw=1.1, label="X")
    l_y, = ax_pos.plot(t, r["y"], color="#009E73", lw=1.1, label="Y")
    ax_pos.legend(loc="upper left", fontsize=8, ncol=2, frameon=False)

    ax_w.set_ylabel("Tốc độ góc (°/s)")
    ax_w.set_xlabel("Thời gian (s)")
    ax_w.set_yscale("symlog", linthresh=1.0)
    ax_w.set_ylim(0, None)

    dyn = {}  # các artist vẽ lại khi đổi thiết lập

    def draw_dynamic():
        for a in dyn.pop("artists", []):
            a.remove()
        arts = []
        raw = np.degrees(np.hypot(r["yaw_raw"], r["pitch_raw"]))
        filt = np.degrees(np.hypot(r["yaw_filt"], r["pitch_filt"]))
        arts += ax_w.plot(t, raw, color="#999", lw=0.8, label="|ω| chưa lọc")
        arts += ax_w.plot(t, filt, color="#0072B2", lw=1.0, label="|ω| sau lọc (0 khi khóa)")
        arts += ax_w.step(t, np.degrees(r["enter_thr"]), where="post", color="#D55E00",
                          lw=0.9, ls="--", label="ngưỡng khóa")
        arts += ax_w.step(t, np.degrees(r["exit_thr"]), where="post", color="#CC79A7",
                          lw=0.9, ls=":", label="ngưỡng mở khóa")
        for ax in (ax_pos, ax_w):
            for s, e in segments(r["locked"], min_len=1):
                arts.append(ax.axvspan(t[s], t[min(e, len(t) - 1)], color="#D55E00", alpha=0.10, lw=0))
            for s, e in segments(r["precision"] & ~r["locked"], min_len=1):
                arts.append(ax.axvspan(t[s], t[min(e, len(t) - 1)], color="#009E73", alpha=0.12, lw=0))
        top = max(np.percentile(raw, 99.8) * 6, 30)  # chừa chỗ cho chú thích
        ax_w.set_ylim(0, top)
        handles = [arts[0], arts[1], arts[2], arts[3],
                   plt.Rectangle((0, 0), 1, 1, color="#D55E00", alpha=0.25),
                   plt.Rectangle((0, 0), 1, 1, color="#009E73", alpha=0.25)]
        leg = ax_w.legend(handles, ["|ω| chưa lọc", "|ω| sau lọc", "ngưỡng khóa", "ngưỡng mở khóa",
                                    "đang khóa", "precision mode"],
                          loc="upper left", fontsize=7.5, ncol=3, frameon=False)
        arts.append(leg)
        dyn["artists"] = arts
        l_x.set_ydata(r["x"])
        l_y.set_ydata(r["y"])

        m = compute_metrics(r)
        dyn["metrics"] = m
        if "tbl" in dyn:
            dyn["tbl"].remove()
        tb = ax_tbl.table(cellText=[[fmt(m[k]) for k, _ in METRIC_COLUMNS]],
                          colLabels=[lbl for _, lbl in METRIC_COLUMNS],
                          loc="upper center", cellLoc="center")
        tb.auto_set_font_size(False)
        tb.set_fontsize(9)
        tb.scale(1, 1.5)
        dyn["tbl"] = tb
        ax_tbl.set_title("Số đo — càng nhỏ càng tốt (trừ % khóa khi đứng yên). Jitter, drift, trôi sau dừng: "
                         "đoạn 'still'; độ trễ: so với ω chưa lọc; lệch: đoạn 'line'.",
                         fontsize=8.5, loc="left")
        detail = (f"   |   Độ nhạy {settings['sensitivity']} · Độ ổn định {settings['stability']}"
                  f" · {args.fps:g} fps" if interactive_settings else "   |   output có sẵn của gyro_replay")
        title.set_text(f"Thuật toán C++ (GyroPointerCore) — {name}{detail}")

    # Sliders
    sx, sw, sh = 0.13, 0.32, 0.022
    s_tw = RangeSlider(fig.add_axes([sx, 0.19, sw, sh]), "Khung thời gian (s)",
                       t[0], t[-1], valinit=(t[0], t[-1]), valfmt="%.1f")
    sliders = []
    if interactive_settings:
        s_sens = Slider(fig.add_axes([sx, 0.25, sw, sh]), "Độ nhạy", 1, 10,
                        valinit=settings["sensitivity"], valstep=1, valfmt="%d")
        s_stab = Slider(fig.add_axes([sx, 0.22, sw, sh]), "Độ ổn định", 1, 10,
                        valinit=settings["stability"], valstep=1, valfmt="%d")
        sliders = [s_sens, s_stab]
    win = [ax.axvspan(t[0], t[-1], color="#0072B2", alpha=0.06, lw=0) for ax in (ax_pos, ax_w)]

    def redraw_window(*_):
        lo, hi = s_tw.val
        idx = np.flatnonzero((t >= lo) & (t <= hi))
        if len(idx):
            trail.set_data(r["x"][idx], r["y"][idx])
            dot.set_data([r["x"][idx[-1]]], [r["y"][idx[-1]]])
            lk = r["locked"][idx]
            starts = idx[np.flatnonzero(lk & ~np.r_[False, lk[:-1]])]
            stops.set_data(r["x"][starts], r["y"][starts])
        for span in win:
            # axvspan là Rectangle (matplotlib >= 3.9) hoặc Polygon (bản cũ hơn)
            if hasattr(span, "set_width"):
                span.set_x(lo)
                span.set_width(hi - lo)
            else:
                span.set_xy([[lo, 0], [lo, 1], [hi, 1], [hi, 0], [lo, 0]])
        fig.canvas.draw_idle()

    def on_settings(_=None):
        settings.update(sensitivity=int(sliders[0].val), stability=int(sliders[1].val))
        new = compute()
        r.clear()
        r.update(new)
        draw_dynamic()
        redraw_window()

    for s in sliders:
        s.on_changed(on_settings)
    s_tw.on_changed(redraw_window)

    draw_dynamic()
    redraw_window()

    if args.save:
        if args.window:
            s_tw.set_val(tuple(args.window))
        fig.savefig(args.save, dpi=110)
        m = dyn["metrics"]
        print(f"Đã lưu {args.save}")
        print("  " + "  ".join(f"{lbl}={fmt(m[k])}" for k, lbl in METRIC_COLUMNS))
    else:
        plt.show()


# ----------------------------------------------------------------------------
# Dữ liệu mẫu (mô phỏng, chỉ để thử tool — không phải thuật toán)
# ----------------------------------------------------------------------------
def _smooth_noise(rng, n, fc, fs, scale):
    """Nhiễu ngẫu nhiên đã làm mượt, mô phỏng rung tay không đều."""
    x = rng.normal(0, scale, n)
    a = (1 / fs) / (1 / fs + 1 / (2 * np.pi * fc))
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, n):
        y[i] = y[i - 1] + a * (x[i] - y[i - 1])
    return y


def make_demo(path, seed=1):
    """Log mô phỏng theo quy ước trục của TV: gz = yaw, gx = pitch, gy = roll; đơn vị vòng/giây."""
    rng = np.random.default_rng(seed)
    fs, G = 100.0, 9.81
    plan = [  # (phase, giây)
        ("still", 4.0), ("line", 3.0), ("still", 1.5), ("flick", 3.6),
        ("still", 1.5), ("circle", 4.0), ("still", 4.0),
    ]
    T = sum(d for _, d in plan)
    n = int(T * fs)
    t = np.cumsum(np.r_[0, 1 / fs + rng.normal(0, 0.0008, n - 1)])  # dt dao động như thật
    yaw = np.zeros(n); pit = np.zeros(n); phase = np.empty(n, object)
    t0 = 0.0
    for ph, d in plan:
        m = (t >= t0) & (t < t0 + d)
        u = t[m] - t0
        phase[m] = ph
        if ph == "line":
            yaw[m] = -0.08 * np.sin(np.pi * u / d) ** 0.3          # kéo sang phải đều (rad/s)
            pit[m] = 0.01 * np.sin(2 * np.pi * u / d)
        elif ph == "flick":
            for k, (sy, sp) in enumerate([(1, 0), (-1, 0.4), (0, -0.4)]):
                c = 0.3 + k * 1.2
                w = (u > c) & (u < c + 0.25)
                pulse = np.sin(np.pi * (u[w] - c) / 0.25) ** 2 * 0.9
                yaw[np.flatnonzero(m)[w]] = sy * pulse
                pit[np.flatnonzero(m)[w]] = sp * pulse
        elif ph == "circle":
            A, Tc = 0.18, 2.0
            yaw[m] = A * np.cos(2 * np.pi * u / Tc)
            pit[m] = A * np.sin(2 * np.pi * u / Tc)
        t0 += d
    # Cầm nghiêng 30° quanh đoạn vẽ tròn (trục yaw/pitch bị trộn vào nhau)
    tc0 = sum(d for _, d in plan[:5])
    roll = np.radians(30) * np.clip((t - (tc0 - 1.0)) / 0.8, 0, 1) * np.clip((tc0 + 4.8 - t) / 0.8, 0, 1)
    # Rung tay sinh lý ~9 Hz + rung ngẫu nhiên (rad/s)
    trem = 0.012 * np.sin(2 * np.pi * 9.2 * t + rng.uniform(0, 6)) + _smooth_noise(rng, n, 6, fs, 0.03)
    trem2 = 0.010 * np.sin(2 * np.pi * 8.7 * t + 1.3) + _smooth_noise(rng, n, 6, fs, 0.03)
    yaw, pit = yaw + trem, pit + trem2
    c, s = np.cos(roll), np.sin(roll)
    g = np.stack([c * pit + s * yaw,            # gx: pitch
                  np.gradient(roll, t),         # gy: roll
                  -s * pit + c * yaw], 1)       # gz: yaw
    g = g + rng.normal(0, 0.006, (n, 3))        # chip đã bù zero drift: không có bias, chỉ còn nhiễu
    g = g / (2.0 * np.pi)                       # rad/s -> vòng/giây (chuẩn đầu vào)

    theta = np.cumsum(pit / fs)                 # góc ngẩng mũi remote
    a = np.stack([G * np.cos(theta) * s, -G * np.sin(theta), G * np.cos(theta) * c], 1)
    a = a + rng.normal(0, 0.04, (n, 3))

    df = pd.DataFrame(dict(t=np.round(t, 5), gx=g[:, 0], gy=g[:, 1], gz=g[:, 2],
                           ax=a[:, 0], ay=a[:, 1], az=a[:, 2], phase=phase))
    df.to_csv(path, index=False, float_format="%.6f")
    print(f"Đã tạo dữ liệu mẫu: {path} ({n} sample, {T:.1f} s)")


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Xem thuật toán con trỏ air-mouse (C++) chạy trên log sensor")
    ap.add_argument("csv", nargs="?", help="log sensor (t, gx, gy, gz vòng/giây) hoặc output của gyro_replay")
    ap.add_argument("--make-demo", metavar="OUT.csv", help="tạo log mẫu (mô phỏng) rồi thoát")
    ap.add_argument("--replay", metavar="PATH", help="đường dẫn gyro_replay (mặc định: cpp/build/)")
    ap.add_argument("--sensitivity", type=int, default=5, help="Độ nhạy 1..10, mặc định 5")
    ap.add_argument("--stability", type=int, default=5, help="Độ ổn định 1..10, mặc định 5")
    ap.add_argument("--fps", type=float, default=60.0,
                    help="tốc độ frame của TV, mặc định 60 (0 = xử lý mọi sample)")
    ap.add_argument("--time-unit", choices=["s", "ms"], default="s")
    ap.add_argument("--yaw-axis", default="z", help="cột gyro điều khiển con trỏ ngang, mặc định z (có thể -z)")
    ap.add_argument("--pitch-axis", default="x", help="cột gyro điều khiển con trỏ dọc, mặc định x (có thể -x)")
    ap.add_argument("--screen", type=lambda s: tuple(int(v) for v in s.lower().split("x")),
                    default=(1920, 1080), help="kích thước màn hình, mặc định 1920x1080")
    ap.add_argument("--save", metavar="OUT.png", help="xuất ảnh thay vì mở cửa sổ")
    ap.add_argument("--window", type=float, nargs=2, metavar=("T0", "T1"),
                    help="khung thời gian hiển thị khi --save")
    args = ap.parse_args()

    if args.make_demo:
        make_demo(args.make_demo)
        return
    if not args.csv:
        ap.error("cần đường dẫn CSV (hoặc --make-demo OUT.csv)")

    header = set(c.strip().lower() for c in pd.read_csv(args.csv, nrows=0).columns)
    if OUTPUT_COLUMNS <= header:
        # Đã là output của gyro_replay: chỉ vẽ, không đổi được thiết lập.
        df = pd.read_csv(args.csv, keep_default_na=False)
        df.columns = [c.strip().lower() for c in df.columns]
        run_ui(("output", df, args.csv), args)
    else:
        run_ui(("log", args.csv, find_replay(args.replay)), args)


if __name__ == "__main__":
    main()
