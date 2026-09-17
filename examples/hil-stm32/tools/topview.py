#!/usr/bin/env python3
"""閉環 CSV → 俯視圖錄影(mp4 / gif)+ 一張靜態軌跡圖(svg / png)。issue #6。

    python3 tools/topview.py out/run.csv [--world world.json] [--scan out/run.scan] [--out out/run]
                             [--fps 10] [--title 文字] [--meta "plant=fake;mode=lockstep;load=5.6;commit=abc123"]
                             [--no-mp4] [--gif] [--gif-fps 5] [--gif-scale 0.5]

畫的東西全部來自同一份 CSV(每 5 ms 一列):真值車體(`plant_x/y/th`)、odom 幽靈車(`odom_x/y/th`)、兩條軌跡、
碰撞步(`collided`)、旗標條(`flags` 的 8 個 bit)、命令 / 受控體輪速、CCR duty。雷射來自橋接 `--scan-log` 存的
`SCAN <step> <n> r...`(上位看到的那份;blind-scan 負對照存的就是全 range_max);沒有 scan 檔而有 world.json 時
用 plant/world.py 的解析解從真值位姿重算,並在畫面標「重算」。動畫末幀的位姿就是 CSV 末列,不另外算。

matplotlib + numpy;mp4 用 imageio-ffmpeg 帶的 ffmpeg。在 docker 裡跑(run_loop.sh 的 RECORD=1 會自動叫)。
"""
import argparse
import csv
import json
import math
import pathlib
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon, Rectangle, Circle  # noqa: E402
from matplotlib import font_manager  # noqa: E402

# 中文:容器裡沒有 CJK 字型,run_loop.sh 把主機的 /usr/share/fonts 唯讀掛進來;找得到 Noto Sans CJK 就用,找不到退回 DejaVu(中文會變方框,不影響數字)
for _f in font_manager.findSystemFonts():
    if "NotoSansCJK" in _f:
        font_manager.fontManager.addfont(_f)
plt.rcParams["font.family"] = ["Noto Sans CJK TC", "Noto Sans CJK JP", "Noto Sans CJK SC", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

FLAG_NAMES = ["ENABLED", "ESTOP", "CMD_STALE", "DRV_FAULT", "BUMPER", "STALL", "HB_LOST", "WDT_RESET"]
FLAG_COLORS = ["#8bc34a", "#d32f2f", "#ff9800", "#c62828", "#7b1fa2", "#e65100", "#f9a825", "#1565c0"]
CH_L, CH_W, WHEEL_Y, WHEEL_R = 0.30, 0.20, 0.15, 0.05   # 與 plant/isaac_plant.py 同一台車


def load_csv(path):
    rows = list(csv.DictReader(open(path, encoding="utf-8")))
    n = len(rows)
    col = lambda k, f=float: np.array([f(r[k]) for r in rows]) if k in rows[0] else None
    d = {
        "step": col("step", int), "t_renode": col("t_us") / 1e6, "wall": col("wall_ms"),
        "cmd_v": col("cmd_v"), "cmd_w": col("cmd_w"), "ccr1": col("ccr1"), "ccr2": col("ccr2"),
        "flags": col("flags", int), "px": col("plant_x") / 1000.0, "py": col("plant_y") / 1000.0, "pth": col("plant_th"),
        "pvl": col("plant_vl"), "pvr": col("plant_vr"),
        "ox": col("odom_x") / 1000.0, "oy": col("odom_y") / 1000.0, "oth": col("odom_th") / 1000.0,   # odom_th 是 mrad
        "collided": col("collided", int) if "collided" in rows[0] else np.zeros(n, dtype=int),
        "en": col("en", lambda s: int(s == "true" or s == "1")),
        "oseq": col("odom_seq", int),
    }
    # 受控體時間:lockstep = 步數 × dt;realtime = wall_ms
    if d["wall"] is not None:
        d["t"] = d["wall"] / 1000.0
        d["mode"] = "realtime"
    else:
        dt = (d["t_renode"][1] - d["t_renode"][0]) if n > 1 else 0.005
        d["t"] = d["step"] * round(dt, 4)
        d["mode"] = "lockstep"
    return d, n


def load_scans(path):
    out = {}
    for line in open(path, encoding="utf-8"):
        f = line.split()
        if len(f) < 3 or f[0] != "SCAN":
            continue
        out[int(f[1])] = np.array([float(x) for x in f[3:3 + int(f[2])]])
    return out


def load_plans(path):
    """driver 的 plan_log:每行 `PLAN <odom_seq> <n> x0 y0 x1 y1 ...`(map 座標,m);odom_seq = 收到這條路徑時 driver 手上最新的 odom 序號。
    跟 CSV 的 odom_seq 欄對齊:不必讓 ROS 的牆鐘與 Renode 時間互相換算。"""
    plans = []
    for line in open(path, encoding="utf-8"):
        f = line.split()
        if len(f) < 3 or f[0] != "PLAN":
            continue
        n = int(f[2]); xy = np.array([float(v) for v in f[3:3 + 2 * n]]).reshape(-1, 2)
        plans.append((int(f[1]), xy))
    plans.sort(key=lambda p: p[0])
    return plans


def robot_patches(ax, color, alpha, ls="-"):
    body = Polygon(np.zeros((4, 2)), closed=True, fc=color, ec=color, alpha=alpha, lw=1.2, ls=ls, zorder=5)
    wl = Polygon(np.zeros((4, 2)), closed=True, fc="#333", ec="#333", alpha=alpha, zorder=6)
    wr = Polygon(np.zeros((4, 2)), closed=True, fc="#333", ec="#333", alpha=alpha, zorder=6)
    head, = ax.plot([], [], color="black", lw=1.5, alpha=alpha, zorder=7)
    for p in (body, wl, wr):
        ax.add_patch(p)
    return body, wl, wr, head


def set_robot(parts, x, y, th):
    body, wl, wr, head = parts
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s], [s, c]])
    def rect(cx, cy, w, h):
        pts = np.array([[-w / 2, -h / 2], [w / 2, -h / 2], [w / 2, h / 2], [-w / 2, h / 2]]) + [cx, cy]
        return pts @ R.T + [x, y]
    body.set_xy(rect(0, 0, CH_L, CH_W))
    wl.set_xy(rect(0, WHEEL_Y, 2 * WHEEL_R, 0.03))
    wr.set_xy(rect(0, -WHEEL_Y, 2 * WHEEL_R, 0.03))
    head.set_data([x, x + 0.18 * c], [y, y + 0.18 * s])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--world", default=None)
    ap.add_argument("--scan", default=None, help="橋接 --scan-log 存的檔;沒給而有 --world 就從真值重算")
    ap.add_argument("--out", default=None, help="輸出前綴(預設 = CSV 去掉副檔名)")
    ap.add_argument("--fps", type=float, default=10.0, help="動畫每秒幀數(受控體時間)")
    ap.add_argument("--title", default="")
    ap.add_argument("--meta", default="", help="k=v;k=v,印在畫面左上")
    ap.add_argument("--no-mp4", action="store_true")
    ap.add_argument("--gif", action="store_true")
    ap.add_argument("--gif-fps", type=float, default=None, help="預設:≤ 30 s 的跑 5,更長的 2(gif 每幀都留在記憶體,長跑要省)")
    ap.add_argument("--gif-scale", type=float, default=None, help="預設:≤ 30 s 的跑 0.5,更長的 0.4")
    ap.add_argument("--max-frames", type=int, default=2000)
    ap.add_argument("--vline", action="append", default=[], help="x:標籤,畫一條垂直虛線(例如 --fault bumper 的 500 mm 牆)")
    ap.add_argument("--plan", default=None, help="driver 的 plan_log(Nav2 /plan);畫出當下最新的一條全域路徑")
    ap.add_argument("--no-recompute", action="store_true", help="沒有 scan 檔時不要從 world 重算雷射(blind-scan 之類的紀錄不在時用)")
    a = ap.parse_args()

    d, n = load_csv(a.csv)
    out = a.out or str(pathlib.Path(a.csv).with_suffix(""))
    world = None
    if a.world:
        sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "plant"))
        from world import World
        world = World(a.world)
    scans = load_scans(a.scan) if a.scan and pathlib.Path(a.scan).exists() else None
    plans = load_plans(a.plan) if a.plan and pathlib.Path(a.plan).exists() else None
    plan_seqs = np.array([p[0] for p in plans]) if plans else None
    scan_src = "紀錄" if scans else ("重算(world.py)" if world and not a.no_recompute else None)

    # ---- 版面:左 俯視圖,右 三張時間圖 ----
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    gs = fig.add_gridspec(3, 2, width_ratios=[1.55, 1], height_ratios=[1, 1, 0.9], left=0.04, right=0.985, top=0.93, bottom=0.17, wspace=0.16, hspace=0.42)
    axm = fig.add_subplot(gs[:, 0])
    axv = fig.add_subplot(gs[0, 1]); axd = fig.add_subplot(gs[1, 1], sharex=axv); axf = fig.add_subplot(gs[2, 1], sharex=axv)

    # 俯視圖:世界或軌跡包圍盒
    if world:
        x0, x1, y0, y1 = world.x_min - 0.25, world.x_max + 0.25, world.y_min - 0.25, world.y_max + 0.25
        axm.add_patch(Rectangle((world.x_min, world.y_min), world.x_max - world.x_min, world.y_max - world.y_min, fill=False, ec="#444", lw=2, zorder=1))
        for cx, cy, bw, bh in world.boxes:
            axm.add_patch(Rectangle((cx - bw / 2, cy - bh / 2), bw, bh, fc="#9e9e9e", ec="#424242", zorder=2))
        gx, gy, _ = world.goal
        axm.add_patch(Circle((gx, gy), 0.1, fill=False, ec="#2e7d32", ls="--", lw=1.2, zorder=3))
        axm.plot([gx], [gy], marker="*", ms=14, color="#2e7d32", zorder=3)
        axm.text(gx, gy + 0.14, "goal", ha="center", color="#2e7d32", fontsize=9)
    else:
        allx = np.concatenate([d["px"], d["ox"]]); ally = np.concatenate([d["py"], d["oy"]])
        pad = 0.45
        x0, x1, y0, y1 = allx.min() - pad, allx.max() + pad, ally.min() - pad, ally.max() + pad
        if x1 - x0 < 1.6: c = (x0 + x1) / 2; x0, x1 = c - 0.8, c + 0.8
        if y1 - y0 < 1.2: c = (y0 + y1) / 2; y0, y1 = c - 0.6, c + 0.6
    axm.set_xlim(x0, x1); axm.set_ylim(y0, y1); axm.set_aspect("equal")
    axm.grid(True, color="#e0e0e0", lw=0.6); axm.set_axisbelow(True)
    axm.set_xlabel("x (m)"); axm.set_ylabel("y (m)")
    axm.plot([d["px"][0]], [d["py"][0]], marker="o", ms=6, color="#1565c0", zorder=4)
    for vl in a.vline:
        xv, _, lab = vl.partition(":")
        axm.axvline(float(xv), color="#7b1fa2", ls="--", lw=1.2, zorder=2)
        axm.text(float(xv) + 0.02, y1 - 0.08, lab, color="#7b1fa2", fontsize=8, va="top")
    trail_t, = axm.plot([], [], color="#1565c0", lw=1.6, zorder=4, label="真值")
    trail_o, = axm.plot([], [], color="#ef6c00", lw=1.2, ls="--", zorder=4, label="odom")
    scan_pts = axm.scatter([], [], s=5, color="#ff7043", zorder=3, label="雷射")
    plan_line, = axm.plot([], [], color="#2e7d32", lw=1.4, ls="-.", zorder=3, label="Nav2 /plan" if plans else None)
    coll_pts, = axm.plot([], [], ls="", marker="x", ms=9, mew=2, color="#d32f2f", zorder=8, label="碰撞")
    bound = Circle((0, 0), 0.2, fill=False, ec="#1565c0", ls=":", lw=0.8, alpha=0.6, zorder=4)
    axm.add_patch(bound)
    truth = robot_patches(axm, "#1565c0", 0.55)
    ghost = robot_patches(axm, "#ef6c00", 0.25, ls="--")
    axm.legend(loc="upper left", fontsize=8, framealpha=0.85)
    txt = fig.text(0.04, 0.012, "", ha="left", va="bottom", fontsize=10, linespacing=1.35)   # 俯視圖下方的狀態列
    meta = " | ".join(kv.strip() for kv in a.meta.split(";") if kv.strip())
    fig.suptitle((a.title + "   " if a.title else "") + meta, fontsize=11, x=0.02, ha="left")

    # 右欄:速度、duty、旗標
    t = d["t"]
    spd = (d["pvl"] + d["pvr"]) / 2.0
    axv.plot(t, d["cmd_v"], color="#9e9e9e", lw=1, label="cmd_v")
    axv.plot(t, spd, color="#1565c0", lw=1.2, label="受控體 (vl+vr)/2")
    axv.plot(t, d["pvl"], color="#42a5f5", lw=0.7, alpha=0.7, label="vl"); axv.plot(t, d["pvr"], color="#7e57c2", lw=0.7, alpha=0.7, label="vr")
    axv.set_ylabel("mm/s"); axv.legend(fontsize=7, loc="upper right", ncol=2); axv.grid(True, color="#eee")
    axd.plot(t, d["ccr1"], color="#2e7d32", lw=1, label="CCR1 (L)"); axd.plot(t, d["ccr2"], color="#6d4c41", lw=1, label="CCR2 (R)")
    axd.set_ylabel("CCR (‰ duty)"); axd.legend(fontsize=7, loc="upper right"); axd.grid(True, color="#eee")
    for b in range(1, 8):
        on = (d["flags"] >> b) & 1
        if on.any():
            idx = np.flatnonzero(np.diff(np.concatenate([[0], on, [0]])))
            spans = [(t[s], t[min(e, n - 1)] - t[s] + 0.005) for s, e in zip(idx[::2], idx[1::2])]
            axf.broken_barh(spans, (b - 0.4, 0.8), color=FLAG_COLORS[b])
    if d["collided"].any():
        cidx = np.flatnonzero(d["collided"])
        axf.plot(t[cidx], np.full(len(cidx), 8), ls="", marker="|", color="#d32f2f", ms=8)
    axf.set_yticks(range(1, 9)); axf.set_yticklabels(FLAG_NAMES[1:] + ["collided"], fontsize=7)
    axf.set_ylim(0.3, 8.7); axf.set_xlabel("受控體時間 (s)"); axf.grid(True, axis="x", color="#eee")
    axv.set_xlim(t[0], t[-1])
    cursors = [ax.axvline(t[0], color="#d32f2f", lw=1) for ax in (axv, axd, axf)]

    # ---- 幀:每 1/fps 秒受控體時間取最近的一列 ----
    frame_t = np.arange(t[0], t[-1] + 1e-9, 1.0 / a.fps)
    if len(frame_t) > a.max_frames:
        frame_t = np.linspace(t[0], t[-1], a.max_frames)
    frame_idx = np.searchsorted(t, frame_t, side="right") - 1
    frame_idx = np.clip(frame_idx, 0, n - 1)
    frame_idx[-1] = n - 1   # 末幀 = CSV 末列
    last_scan_i = None

    def render(fi):
        nonlocal last_scan_i
        i = int(fi)
        trail_t.set_data(d["px"][:i + 1], d["py"][:i + 1]); trail_o.set_data(d["ox"][:i + 1], d["oy"][:i + 1])
        set_robot(truth, d["px"][i], d["py"][i], d["pth"][i]); set_robot(ghost, d["ox"][i], d["oy"][i], d["oth"][i])
        bound.center = (d["px"][i], d["py"][i])
        ci = np.flatnonzero(d["collided"][:i + 1])
        coll_pts.set_data(d["px"][ci], d["py"][ci])
        # 雷射:紀錄檔裡 ≤ i 最近的一筆;或從真值重算(每 100 ms 一次)
        rs = None
        if scans:
            ks = [k for k in scans if k <= i]
            if ks:
                k = max(ks)
                if k != last_scan_i:
                    rs = scans[k]; last_scan_i = k
                else:
                    rs = scans[k]
        elif world and not a.no_recompute:
            rs = np.array(world.scan(d["px"][i], d["py"][i], d["pth"][i]))
        if rs is not None and world:
            m = len(rs); ang = d["pth"][i] - math.pi + 2 * math.pi * np.arange(m) / m
            hit = rs < world.range_max - 1e-3
            pts = np.c_[d["px"][i] + rs * np.cos(ang), d["py"][i] + rs * np.sin(ang)][hit]
            scan_pts.set_offsets(pts if len(pts) else np.zeros((0, 2)))
        if plans and d["oseq"] is not None:
            j = int(np.searchsorted(plan_seqs, d["oseq"][i], side="right")) - 1
            if j >= 0:
                plan_line.set_data(plans[j][1][:, 0], plans[j][1][:, 1])
        fl = d["flags"][i]
        names = [FLAG_NAMES[b] for b in range(1, 8) if (fl >> b) & 1]
        txt.set_text(f"t = {t[i]:.2f} s(step {d['step'][i]})    真值 ({d['px'][i]*1000:.0f}, {d['py'][i]*1000:.0f}) mm θ {d['pth'][i]:.3f}    "
                     f"odom ({d['ox'][i]*1000:.0f}, {d['oy'][i]*1000:.0f}) mm θ {d['oth'][i]:.3f}\n"
                     f"cmd v {d['cmd_v'][i]:.0f} mm/s  w {d['cmd_w'][i]:.0f} mrad/s    CCR {d['ccr1'][i]:.0f} / {d['ccr2'][i]:.0f}    "
                     f"flags 0x{fl:02x} {' '.join(names) if names else '-'}{'    碰撞' if d['collided'][i] else ''}"
                     + (f"    雷射:{scan_src}" if scan_src else ""))
        for cl in cursors:
            cl.set_xdata([t[i], t[i]])

    written = []
    if not a.no_mp4:
        import imageio_ffmpeg
        plt.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
        from matplotlib.animation import FFMpegWriter
        w = FFMpegWriter(fps=a.fps, codec="libx264", extra_args=["-pix_fmt", "yuv420p", "-crf", "23"])
        with w.saving(fig, out + ".mp4", dpi=100):
            for fi in frame_idx:
                render(fi); w.grab_frame()
        written.append(out + ".mp4")
    if a.gif:
        from matplotlib.animation import PillowWriter
        long_run = (t[-1] - t[0]) > 30
        gif_fps = a.gif_fps or (2.0 if long_run else 5.0)
        gif_scale = a.gif_scale or (0.4 if long_run else 0.5)
        every = max(1, int(round(a.fps / gif_fps)))
        g = PillowWriter(fps=gif_fps)
        with g.saving(fig, out + ".gif", dpi=int(100 * gif_scale)):
            for fi in list(frame_idx[::every]) + [frame_idx[-1]]:
                render(fi); g.grab_frame()
        written.append(out + ".gif")
    # 靜態圖:末幀(整條軌跡 + 末端位姿)
    render(frame_idx[-1])
    fig.savefig(out + "_topview.svg"); fig.savefig(out + "_topview.png", dpi=100)
    written += [out + "_topview.svg", out + "_topview.png"]
    i = n - 1
    print(f"[topview] {len(frame_idx)} 幀 @ {a.fps} fps;末幀 = CSV 末列 step {d['step'][i]}:真值 ({d['px'][i]*1000:.1f}, {d['py'][i]*1000:.1f}) mm θ {d['pth'][i]:.4f};"
          f" 雷射 {scan_src or '無'};plan {len(plans) if plans else 0} 條;寫出 " + ", ".join(written))


if __name__ == "__main__":
    main()
