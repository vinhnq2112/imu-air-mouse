#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
imu_viz.py — Công cụ OFFLINE so sánh thuật toán air-mouse (IMU -> con trỏ TV).

Chạy cùng một bản ghi dữ liệu qua nhiều thuật toán, vẽ vết con trỏ chồng nhau,
cho phép kéo slider chỉnh tham số và tự tính các số đo (jitter, drift, độ trễ...).

Cách chạy:
    python imu_viz.py --make-demo demo.csv      # tạo dữ liệu mẫu để thử
    python imu_viz.py demo.csv                  # mở cửa sổ tương tác
    python imu_viz.py my_log.csv --save report.png                   # xuất ảnh

Cần: Python 3.9+, numpy, pandas, matplotlib.

Định dạng CSV (dòng đầu là header):
    t               thời gian (giây; thêm --time-unit ms nếu là mili giây)
    gx, gy, gz      vận tốc góc (vòng/giây, 1.0 = 360°/s; log cũ theo rad/s hoặc °/s
                    thì thêm --gyro-unit rad / deg). Bên trong tool đổi sang rad/s.
    ax, ay, az      gia tốc (đơn vị nào cũng được, chỉ dùng hướng trọng lực)
    qw, qx, qy, qz  (tuỳ chọn) quaternion do chip fusion xuất ra
    phase           (tuỳ chọn) nhãn đoạn test: still / line / flick / circle / ...
                    -> có nhãn 'still' thì mới tính jitter, drift;
                       có nhãn 'line' thì mới tính độ lệch đường thẳng.

Quy ước trục thiết bị: X = mũi remote, Y = bên trái, Z = hướng lên (tay phải).
Nếu cảm biến gắn khác, dùng --remap, ví dụ: --remap "x=y,y=-x,z=z"
(--remap chỉ áp cho gyro và accel, không áp cho quaternion của chip).

So sánh ma trận 2x2 (đầu vào đều là angular velocity, con trỏ = K * ω * dt):
                         Thô (chỉ nhân K)          Thuật toán
    6-axis  (ω gyro)     6-axis · thô              trừ bias + bù roll + EMA + dead zone + gain
    Fusion  (ω từ        Fusion · thô              trừ bias + EMA + dead zone + gain
             quaternion Mahony, hướng mũi remote)
Thêm (ẩn mặc định, bật bằng checkbox):
    6-axis · 1€          như "6-axis · thuật toán" nhưng dùng One Euro filter
    Quaternion chip      nếu CSV có qw..qz
"""

import argparse
import sys

import numpy as np
import pandas as pd

PHASE_VI = {
    "still": "Đứng yên",
    "line": "Kẻ thẳng",
    "flick": "Vẩy nhanh",
    "circle": "Vẽ tròn",
}



# ----------------------------------------------------------------------------
# Đọc dữ liệu
# ----------------------------------------------------------------------------
def parse_remap(s):
    idx = {"x": 0, "y": 1, "z": 2}
    out = [None, None, None]
    for part in s.split(","):
        lhs, rhs = (p.strip().lower() for p in part.split("="))
        sign = -1.0 if rhs.startswith("-") else 1.0
        out[idx[lhs]] = (idx[rhs.lstrip("+-")], sign)
    if None in out:
        sys.exit("--remap phải gán đủ x, y, z, ví dụ: x=y,y=-x,z=z")
    return out


def apply_remap(v, m):
    return np.stack([m[i][1] * v[:, m[i][0]] for i in range(3)], axis=1)


def load_csv(path, time_unit="s", gyro_unit="rev", remap=None):
    df = pd.read_csv(path)
    df.columns = [c.strip().lower() for c in df.columns]
    need = ["t", "gx", "gy", "gz", "ax", "ay", "az"]
    missing = [c for c in need if c not in df.columns]
    if missing:
        sys.exit(f"CSV thiếu cột: {missing}. Cần ít nhất: {need}")

    t = df["t"].to_numpy(float)
    if time_unit == "ms":
        t = t / 1000.0
    t = t - t[0]
    dt = np.diff(t, prepend=np.nan)
    dt[0] = np.median(dt[1:])
    dt = np.clip(dt, 1e-4, 0.1)  # chặn sample lỗi / mất gói

    # Chuẩn đầu vào: vòng/giây (rev/s). Thuật toán bên trong tính theo rad/s.
    g = df[["gx", "gy", "gz"]].to_numpy(float)
    if gyro_unit == "rev":
        g = g * (2.0 * np.pi)
    elif gyro_unit == "deg":
        g = np.radians(g)
    a = df[["ax", "ay", "az"]].to_numpy(float)
    if remap:
        m = parse_remap(remap)
        g, a = apply_remap(g, m), apply_remap(a, m)

    q = None
    if all(c in df.columns for c in ["qw", "qx", "qy", "qz"]):
        q = df[["qw", "qx", "qy", "qz"]].to_numpy(float)
        q /= np.linalg.norm(q, axis=1, keepdims=True)

    phase = df["phase"].astype(str).str.strip().to_numpy() if "phase" in df else None
    return dict(t=t, dt=dt, g=g, a=a, q=q, phase=phase, name=str(path))


# ----------------------------------------------------------------------------
# Bộ lọc
# ----------------------------------------------------------------------------
def _alpha(d, fc):
    tau = 1.0 / (2.0 * np.pi * fc)
    return d / (tau + d)


def lowpass(x, dt, fc):
    """Lọc thông thấp bậc 1 (EMA) có tính theo dt thực."""
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = y[i - 1] + _alpha(dt[i], fc) * (x[i] - y[i - 1])
    return y


def one_euro(x, dt, mincutoff, beta, dcutoff=1.0):
    """One Euro filter: tay di chậm -> lọc mạnh (bớt rung), di nhanh -> lọc nhẹ (bớt trễ)."""
    y = np.empty_like(x)
    y[0] = x[0]
    dx_hat = np.zeros_like(x[0])
    for i in range(1, len(x)):
        d = dt[i]
        dx = (x[i] - y[i - 1]) / d
        dx_hat = dx_hat + _alpha(d, dcutoff) * (dx - dx_hat)
        fc = mincutoff + beta * np.abs(dx_hat)
        y[i] = y[i - 1] + _alpha(d, fc) * (x[i] - y[i - 1])
    return y


# ----------------------------------------------------------------------------
# Sensor fusion (Mahony, 6 trục)
# ----------------------------------------------------------------------------
def mahony(g, a, dt, kp=1.0, ki=0.02):
    a0 = a[0] / np.linalg.norm(a[0])
    roll = np.arctan2(a0[1], a0[2])
    pitch = np.arctan2(-a0[0], np.hypot(a0[1], a0[2]))
    cr, sr, cp, sp = np.cos(roll / 2), np.sin(roll / 2), np.cos(pitch / 2), np.sin(pitch / 2)
    w, x, y, z = cr * cp, sr * cp, cr * sp, -sr * sp
    ix = iy = iz = 0.0
    out = np.empty((len(g), 4))
    for i in range(len(g)):
        ax_, ay_, az_ = a[i]
        n = (ax_ * ax_ + ay_ * ay_ + az_ * az_) ** 0.5
        gx, gy, gz = g[i]
        if n > 1e-9:
            ax_, ay_, az_ = ax_ / n, ay_ / n, az_ / n
            # hướng trọng lực dự đoán trong hệ thiết bị
            vx = 2 * (x * z - w * y)
            vy = 2 * (w * x + y * z)
            vz = w * w - x * x - y * y + z * z
            ex, ey, ez = ay_ * vz - az_ * vy, az_ * vx - ax_ * vz, ax_ * vy - ay_ * vx
            d = dt[i]
            ix += ki * ex * d
            iy += ki * ey * d
            iz += ki * ez * d
            gx += kp * ex + ix
            gy += kp * ey + iy
            gz += kp * ez + iz
        h = 0.5 * dt[i]
        w, x, y, z = (
            w + h * (-x * gx - y * gy - z * gz),
            x + h * (w * gx + y * gz - z * gy),
            y + h * (w * gy - x * gz + z * gx),
            z + h * (w * gz + x * gy - y * gx),
        )
        n = (w * w + x * x + y * y + z * z) ** 0.5
        w, x, y, z = w / n, x / n, y / n, z / n
        out[i] = (w, x, y, z)
    return out


def pointing_rates(q, dt):
    """Từ quaternion -> tốc độ (yaw, pitch) của vector mũi remote, rad/s.
    Quy ước pitch dương = chúc mũi xuống (giống ω_y)."""
    w, x, y, z = q.T
    fx = 1 - 2 * (y * y + z * z)
    fy = 2 * (x * y + w * z)
    fz = 2 * (x * z - w * y)
    yaw = np.arctan2(fy, fx)
    pitch = np.arcsin(np.clip(fz, -1, 1))
    dyaw = np.diff(yaw, prepend=yaw[0])
    dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi  # xử lý nhảy ±180°
    dp = np.diff(pitch, prepend=pitch[0])
    return np.stack([dyaw / dt, -dp / dt], axis=1)


# ----------------------------------------------------------------------------
# Các bước dùng chung
# ----------------------------------------------------------------------------
def level_rates(data, bias, roll_fc=2.0):
    """Trừ bias + bù roll -> (ω_yaw, ω_pitch) trong hệ 'remote thẳng đứng'."""
    g = data["g"] - bias
    a_lp = lowpass(data["a"], data["dt"], roll_fc)
    phi = np.arctan2(a_lp[:, 1], a_lp[:, 2])
    c, s = np.cos(phi), np.sin(phi)
    yaw = g[:, 1] * s + g[:, 2] * c
    pitch = g[:, 1] * c - g[:, 2] * s
    return np.stack([yaw, pitch], axis=1)


def shape(rate, P):
    """Dead zone + gain theo tốc độ -> vận tốc con trỏ (px/s), trục màn hình."""
    sp = np.hypot(rate[:, 0], rate[:, 1])
    sp2 = np.maximum(sp - P["dz"], 0.0)
    scale = sp2 / np.maximum(sp, 1e-12)
    k = P["K"] * (1.0 + P["accel"] * sp2)
    v = rate * (scale * k)[:, None]
    return np.stack([-v[:, 0], v[:, 1]], axis=1)  # yaw trái(+) -> x giảm


def integrate(v, dt, W, H):
    pos = np.empty_like(v)
    cx, cy = W / 2.0, H / 2.0
    for i in range(len(v)):
        cx = min(max(cx + v[i, 0] * dt[i], 0.0), W - 1.0)
        cy = min(max(cy + v[i, 1] * dt[i], 0.0), H - 1.0)
        pos[i] = (cx, cy)
    return pos


# ----------------------------------------------------------------------------
# Thuật toán
# ----------------------------------------------------------------------------
def build_pipelines(data, W, H, calib_s, kp, ki):
    dt = data["dt"]
    calib = data["t"] < calib_s
    bias = data["g"][calib].mean(axis=0) if calib.sum() > 5 else np.zeros(3)
    ref = level_rates(data, bias)  # tham chiếu: đã trừ bias, bù roll, chưa lọc
    # Fusion thô: Mahony chạy trên gyro thô; Fusion thuật toán: trừ bias trước khi fusion
    fusion_raw = pointing_rates(mahony(data["g"], data["a"], dt, kp, ki), dt)
    fusion_rates = pointing_rates(mahony(data["g"] - bias, data["a"], dt, kp, ki), dt)
    chip_rates = pointing_rates(data["q"], dt) if data["q"] is not None else None

    def linear(r, P):  # chỉ nhân K, không dead zone / gain / lọc
        return np.stack([-P["K"] * r[:, 0], P["K"] * r[:, 1]], axis=1)

    # --- Nguồn 6-axis: ω thô từ gyro ---
    def p6_raw(P):
        r = data["g"][:, [2, 1]]
        return r, linear(r, P)

    def p6_algo(P):  # bias + bù roll + EMA + dead zone + gain theo tốc độ
        r = lowpass(ref, dt, P["fc"])
        return r, shape(r, P)

    # --- Nguồn Sensor Fusion: ω lấy từ quaternion (hướng mũi remote) ---
    def pf_raw(P):
        return fusion_raw, linear(fusion_raw, P)

    def pf_algo(P):  # trừ bias + EMA + dead zone + gain (roll đã do fusion xử lý)
        r = lowpass(fusion_rates, dt, P["fc"])
        return r, shape(r, P)

    def p6_euro(P):
        r = one_euro(ref, dt, P["mincut"], P["beta"])
        return r, shape(r, P)

    # (tên, hàm, màu, hiện mặc định)
    pipes = [
        ("6-axis · thô", p6_raw, "#E69F00", True),
        ("6-axis · thuật toán", p6_algo, "#D55E00", True),
        ("Fusion · thô", pf_raw, "#56B4E9", True),
        ("Fusion · thuật toán", pf_algo, "#0072B2", True),
        ("6-axis · 1€ (thêm)", p6_euro, "#009E73", False),
    ]
    if chip_rates is not None:
        def p_chip(P):
            r = lowpass(chip_rates, dt, P["fc"])
            return r, shape(r, P)
        pipes.append(("Quaternion chip (thêm)", p_chip, "#CC79A7", False))

    def run_all(P):
        out = []
        for name, fn, color, on in pipes:
            rate, v = fn(P)
            out.append(dict(name=name, color=color, on=on,
                            rate=rate, v=v, pos=integrate(v, dt, W, H)))
        return out

    return run_all, ref, bias


# ----------------------------------------------------------------------------
# Số đo
# ----------------------------------------------------------------------------
def segments(mask, min_len=20):
    segs, start = [], None
    for i, m in enumerate(np.append(mask, False)):
        if m and start is None:
            start = i
        elif not m and start is not None:
            if i - start >= min_len:
                segs.append((start, i))
            start = None
    return segs


def compute_metrics(res, data, ref):
    t, phase = data["t"], data["phase"]
    still = phase == "still" if phase is not None else np.zeros(len(t), bool)
    line = phase == "line" if phase is not None else np.zeros(len(t), bool)
    moving = ~still
    med_dt = np.median(data["dt"])
    rows = []
    for r in res:
        pos = r["pos"]
        # Jitter & drift trong các đoạn đứng yên
        jit, drift = [], []
        for s, e in segments(still):
            tt = t[s:e] - t[s:e].mean()
            sl = []
            resid2 = np.zeros(e - s)
            for k in range(2):
                p = np.polyfit(tt, pos[s:e, k], 1)
                sl.append(p[0])
                resid2 += (pos[s:e, k] - np.polyval(p, tt)) ** 2
            jit.append(resid2)
            drift.append(np.hypot(*sl))
        jitter = np.sqrt(np.concatenate(jit).mean()) if jit else np.nan
        drift = float(np.mean(drift)) if drift else np.nan

        # Độ trễ: cross-correlation giữa ω tham chiếu và vận tốc con trỏ
        refx, vx = -ref[:, 0], r["v"][:, 0]
        if moving.sum() > 50:
            refx, vx = refx * moving, vx * moving
            best, lag = -np.inf, 0
            for L in range(0, int(0.3 / med_dt)):
                a_, b_ = refx[: len(refx) - L], vx[L:]
                den = np.linalg.norm(a_) * np.linalg.norm(b_)
                c = (a_ @ b_) / den if den > 0 else 0
                if c > best:
                    best, lag = c, L
            latency = lag * med_dt * 1000
        else:
            latency = np.nan

        # Độ lệch khỏi đường thẳng khi kẻ thẳng
        dev = []
        for s, e in segments(line):
            p = pos[s:e] - pos[s:e].mean(axis=0)
            _, _, vt = np.linalg.svd(p, full_matrices=False)
            dev.append(p @ vt[1])
        straight = np.sqrt(np.mean(np.concatenate(dev) ** 2)) if dev else np.nan

        rows.append(dict(name=r["name"], jitter=jitter, drift=drift,
                         latency=latency, straight=straight))
    return rows


def fmt(v, unit=""):
    return "–" if np.isnan(v) else f"{v:.1f}{unit}"


# ----------------------------------------------------------------------------
# Giao diện
# ----------------------------------------------------------------------------
def run_ui(data, args):
    import matplotlib
    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.widgets import CheckButtons, RangeSlider, Slider

    W, H = args.screen
    run_all, ref, bias = build_pipelines(data, W, H, args.calib, args.kp, args.ki)
    t = data["t"]
    P = dict(K=args.K, dz=args.deadzone, accel=args.accel, fc=args.fc,
             mincut=args.mincut, beta=args.beta)

    plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False})
    fig = plt.figure(figsize=(16, 9))
    fig.canvas.manager.set_window_title("IMU air-mouse — so sánh thuật toán") if not args.save else None
    fig.suptitle(f"So sánh thuật toán con trỏ — {data['name']}   "
                 f"(bias gyro ước lượng: {np.degrees(bias).round(2)} °/s)", fontsize=11)

    ax_scr = fig.add_axes([0.04, 0.33, 0.48, 0.58])
    ax_x = fig.add_axes([0.58, 0.66, 0.40, 0.25])
    ax_w = fig.add_axes([0.58, 0.33, 0.40, 0.27], sharex=ax_x)
    ax_tbl = fig.add_axes([0.58, 0.02, 0.40, 0.22])
    ax_tbl.axis("off")

    # Màn hình ảo
    ax_scr.set_xlim(-20, W + 20)
    ax_scr.set_ylim(H + 20, -20)
    ax_scr.set_aspect("equal")
    ax_scr.add_patch(plt.Rectangle((0, 0), W, H, fill=False, lw=1.5, ec="#444"))
    ax_scr.set_title(f"Màn hình ảo {W}×{H} — vết con trỏ (chấm = vị trí cuối khung thời gian)")
    ax_scr.set_xticks([0, W // 2, W])
    ax_scr.set_yticks([0, H // 2, H])

    # Tô nền các đoạn test
    if data["phase"] is not None:
        ph = data["phase"]
        change = np.flatnonzero(np.r_[True, ph[1:] != ph[:-1], True])
        for j, (s, e) in enumerate(zip(change[:-1], change[1:])):
            for ax in (ax_x, ax_w):
                ax.axvspan(t[s], t[e - 1], color="#000", alpha=0.04 if j % 2 else 0.0, lw=0)
            ax_x.text((t[s] + t[e - 1]) / 2, 1.02, PHASE_VI.get(ph[s], ph[s]),
                      transform=ax_x.get_xaxis_transform(), ha="center", va="bottom", fontsize=8)

    ax_x.set_ylabel("Vị trí X con trỏ (px)")
    ax_w.set_ylabel("Tốc độ yaw đã xử lý (°/s)")
    ax_w.set_xlabel("Thời gian (s)")

    res = run_all(P)
    lines = []
    for r in res:
        tr, = ax_scr.plot([], [], color=r["color"], lw=1.2, alpha=0.85)
        dot, = ax_scr.plot([], [], "o", color=r["color"], ms=7, mec="white")
        lx, = ax_x.plot(t, r["pos"][:, 0], color=r["color"], lw=1.1, label=r["name"])
        lw_, = ax_w.plot(t, np.degrees(r["rate"][:, 0]), color=r["color"], lw=0.9)
        lines.append(dict(trail=tr, dot=dot, x=lx, w=lw_))
    ax_x.legend(loc="upper left", fontsize=8, ncol=2, frameon=False)
    ax_x.set_ylim(-20, W + 20)
    win_x = ax_x.axvspan(t[0], t[-1], color="#0072B2", alpha=0.08, lw=0)
    win_w = ax_w.axvspan(t[0], t[-1], color="#0072B2", alpha=0.08, lw=0)

    # Sliders
    sx, sw, sh = 0.13, 0.32, 0.022
    def mk(y, label, lo, hi, val, fmt_="%.2f"):
        return Slider(fig.add_axes([sx, y, sw, sh]), label, lo, hi, valinit=val, valfmt=fmt_)
    s_K = mk(0.255, "K (px/rad)", 500, 6000, P["K"], "%.0f")
    s_dz = mk(0.225, "Dead zone (rad/s)", 0.0, 0.15, P["dz"], "%.3f")
    s_ac = mk(0.195, "Gain theo tốc độ", 0.0, 3.0, P["accel"])
    s_fc = mk(0.165, "EMA fc (Hz)", 0.5, 30.0, P["fc"], "%.1f")
    s_mc = mk(0.135, "1€ min fc (Hz)", 0.1, 10.0, P["mincut"], "%.2f")
    s_bt = mk(0.105, "1€ beta", 0.0, 0.5, P["beta"], "%.3f")
    s_tw = RangeSlider(fig.add_axes([sx, 0.06, sw, sh]), "Khung thời gian (s)",
                       t[0], t[-1], valinit=(t[0], t[-1]), valfmt="%.1f")

    chk = CheckButtons(fig.add_axes([0.47, 0.06, 0.10, 0.22], frameon=False),
                       [r["name"] for r in res], [r["on"] for r in res])
    try:
        for lbl, r in zip(chk.labels, res):
            lbl.set_color(r["color"])
            lbl.set_fontsize(8)
    except Exception:
        pass

    table_holder = {}

    def draw_table():
        if "tbl" in table_holder:
            table_holder["tbl"].remove()
        rows = compute_metrics(res, data, ref)
        cell = [[r["name"], fmt(r["jitter"]), fmt(r["drift"]), fmt(r["latency"]), fmt(r["straight"])]
                for r in rows]
        tb = ax_tbl.table(cellText=cell, loc="upper center", cellLoc="center",
                          colLabels=["Thuật toán", "Jitter (px)", "Drift (px/s)",
                                     "Độ trễ (ms)", "Lệch đường thẳng (px)"])
        tb.auto_set_font_size(False)
        tb.set_fontsize(8.5)
        tb.scale(1, 1.35)
        for i, r in enumerate(res):
            tb[i + 1, 0].get_text().set_color(r["color"])
        table_holder["tbl"] = tb
        ax_tbl.set_title("Số đo — càng nhỏ càng tốt  (jitter/drift: đoạn đứng yên; "
                         "độ trễ: so với gyro chưa lọc)", fontsize=8.5, loc="left")

    def redraw_window(*_):
        lo, hi = s_tw.val
        m = (t >= lo) & (t <= hi)
        idx = np.flatnonzero(m)
        for r, L, on in zip(res, lines, chk.get_status()):
            if len(idx):
                L["trail"].set_data(r["pos"][idx, 0], r["pos"][idx, 1])
                L["dot"].set_data([r["pos"][idx[-1], 0]], [r["pos"][idx[-1], 1]])
            for k in ("trail", "dot", "x", "w"):
                L[k].set_visible(on)
        for span in (win_x, win_w):
            span.set_x(lo) if hasattr(span, "set_x") else None
            try:
                span.set_width(hi - lo)
            except AttributeError:
                span.set_xy([[lo, 0], [lo, 1], [hi, 1], [hi, 0]])
        fig.canvas.draw_idle()

    def recompute(_=None):
        P.update(K=s_K.val, dz=s_dz.val, accel=s_ac.val, fc=s_fc.val,
                 mincut=s_mc.val, beta=s_bt.val)
        res[:] = run_all(P)
        for r, L in zip(res, lines):
            L["x"].set_ydata(r["pos"][:, 0])
            L["w"].set_ydata(np.degrees(r["rate"][:, 0]))
        draw_table()
        redraw_window()

    for s in (s_K, s_dz, s_ac, s_fc, s_mc, s_bt):
        s.on_changed(recompute)
    s_tw.on_changed(redraw_window)
    chk.on_clicked(redraw_window)

    draw_table()
    redraw_window()
    ymax = np.percentile(np.abs(np.degrees(data["g"][:, 2])), 99.5) * 1.15
    ax_w.set_ylim(-ymax, ymax)

    if args.save:
        if args.window:
            s_tw.set_val(tuple(args.window))
        fig.savefig(args.save, dpi=110)
        print(f"Đã lưu {args.save}")
        for r in compute_metrics(res, data, ref):
            print(f"  {r['name']:<24} jitter={fmt(r['jitter'])}px  drift={fmt(r['drift'])}px/s  "
                  f"trễ={fmt(r['latency'])}ms  lệch={fmt(r['straight'])}px")
    else:
        plt.show()


# ----------------------------------------------------------------------------
# Dữ liệu mẫu (mô phỏng)
# ----------------------------------------------------------------------------
def make_demo(path, seed=1):
    rng = np.random.default_rng(seed)
    fs, G = 100.0, 9.81
    plan = [  # (phase, giây)
        ("still", 4.0), ("line", 3.0), ("still", 1.5), ("flick", 3.6),
        ("still", 1.5), ("circle", 4.0), ("still", 4.0),
    ]
    T = sum(d for _, d in plan)
    n = int(T * fs)
    t = np.cumsum(np.r_[0, 1 / fs + rng.normal(0, 0.0008, n - 1)])  # dt dao động như thật
    yaw = np.zeros(n); pit = np.zeros(n); roll = np.zeros(n); phase = np.empty(n, object)
    t0 = 0.0
    for ph, d in plan:
        m = (t >= t0) & (t < t0 + d)
        u = t[m] - t0
        phase[m] = ph
        if ph == "line":
            yaw[m] = -0.08 * np.sin(np.pi * u / d) ** 0.3          # kéo sang phải đều
            pit[m] = 0.01 * np.sin(2 * np.pi * u / d)
        elif ph == "flick":
            for k, (sy, sp) in enumerate([(1, 0), (-1, 0.6), (0, -0.6)]):
                c = 0.3 + k * 1.2
                w = (u > c) & (u < c + 0.25)
                pulse = np.sin(np.pi * (u[w] - c) / 0.25) ** 2 * 1.2
                yaw[np.flatnonzero(m)[w]] = sy * pulse
                pit[np.flatnonzero(m)[w]] = sp * pulse
        elif ph == "circle":
            A, Tc = 0.25, 2.0
            yaw[m] = A * np.cos(2 * np.pi * u / Tc)
            pit[m] = A * np.sin(2 * np.pi * u / Tc)
        t0 += d
    # Cầm nghiêng 30° quanh đoạn vẽ tròn
    tc0 = sum(d for _, d in plan[:5])
    roll = np.radians(30) * np.clip((t - (tc0 - 1.0)) / 0.8, 0, 1) * np.clip((tc0 + 4.8 - t) / 0.8, 0, 1)
    # Run tay sinh lý ~9 Hz + run ngẫu nhiên
    trem = 0.012 * np.sin(2 * np.pi * 9.2 * t + rng.uniform(0, 6)) + lowpass_np(rng.normal(0, 0.03, n), 6, fs)
    trem2 = 0.010 * np.sin(2 * np.pi * 8.7 * t + 1.3) + lowpass_np(rng.normal(0, 0.03, n), 6, fs)
    yaw, pit = yaw + trem, pit + trem2
    rollrate = np.gradient(roll, t)

    c, s = np.cos(roll), np.sin(roll)
    gx = rollrate
    gy = c * pit + s * yaw
    gz = -s * pit + c * yaw
    bias = np.array([0.012, -0.018, 0.015])                       # bias gyro (rad/s)
    g = np.stack([gx, gy, gz], 1) + bias + rng.normal(0, 0.006, (n, 3))

    theta = np.cumsum(pit / fs)                                   # góc chúc mũi
    lin = np.abs(np.gradient(yaw, t))[:, None] * rng.normal(0, 0.05, (n, 3))  # gia tốc tịnh tiến khi vẩy
    a = np.stack([-G * np.sin(theta), G * np.cos(theta) * s, G * np.cos(theta) * c], 1)
    a = a + lin + rng.normal(0, 0.04, (n, 3))

    g = g / (2.0 * np.pi)                                         # rad/s -> rev/s (chuẩn đầu vào)
    df = pd.DataFrame(dict(t=np.round(t, 5), gx=g[:, 0], gy=g[:, 1], gz=g[:, 2],
                           ax=a[:, 0], ay=a[:, 1], az=a[:, 2], phase=phase))
    df.to_csv(path, index=False, float_format="%.6f")
    print(f"Đã tạo dữ liệu mẫu: {path} ({n} sample, {T:.1f} s)")


def lowpass_np(x, fc, fs):
    y = np.empty_like(x)
    a = _alpha(1 / fs, fc)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = y[i - 1] + a * (x[i] - y[i - 1])
    return y


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="So sánh thuật toán air-mouse offline")
    ap.add_argument("csv", nargs="?", help="file CSV dữ liệu IMU")
    ap.add_argument("--make-demo", metavar="OUT.csv", help="tạo dữ liệu mẫu rồi thoát")
    ap.add_argument("--time-unit", choices=["s", "ms"], default="s")
    ap.add_argument("--gyro-unit", choices=["rev", "rad", "deg"], default="rev",
                    help="đơn vị gx/gy/gz trong CSV: rev = vòng/giây (mặc định), rad = rad/s, deg = °/s")
    ap.add_argument("--remap", help='đổi trục cảm biến, ví dụ "x=y,y=-x,z=z"')
    ap.add_argument("--screen", type=lambda s: tuple(int(v) for v in s.lower().split("x")),
                    default=(1920, 1080), help="kích thước màn hình, mặc định 1920x1080")
    ap.add_argument("--calib", type=float, default=1.0,
                    help="số giây đầu (remote để yên) dùng để ước lượng bias, mặc định 1.0")
    ap.add_argument("--K", type=float, default=2700)
    ap.add_argument("--deadzone", type=float, default=0.02)
    ap.add_argument("--accel", type=float, default=0.5)
    ap.add_argument("--fc", type=float, default=8.0)
    ap.add_argument("--mincut", type=float, default=1.0)
    ap.add_argument("--beta", type=float, default=0.05)
    ap.add_argument("--kp", type=float, default=1.0, help="Mahony Kp")
    ap.add_argument("--ki", type=float, default=0.02, help="Mahony Ki")
    ap.add_argument("--save", metavar="OUT.png", help="xuất ảnh thay vì mở cửa sổ")
    ap.add_argument("--window", type=float, nargs=2, metavar=("T0", "T1"),
                    help="khung thời gian hiển thị khi --save")
    args = ap.parse_args()

    if args.make_demo:
        make_demo(args.make_demo)
        return
    if not args.csv:
        ap.error("cần đường dẫn CSV (hoặc --make-demo OUT.csv)")
    data = load_csv(args.csv, args.time_unit, args.gyro_unit, args.remap)
    run_ui(data, args)


if __name__ == "__main__":
    main()
