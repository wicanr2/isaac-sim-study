#!/usr/bin/env python3
"""Isaac 真實俯視相機的幀 ↔ 閉環 CSV 對照(issue #6 的判準):每一幀找底盤藍色像素的重心,
換算成世界座標,對 CSV 同一步的真值 plant_x/y;印最大偏差(判準 < 0.2 m)。順便把幀序列編成 mp4。

    python3 tools/topcam_check.py out/topcam_isaac_world out/isaac_world_cam2.csv --every 20 --view-w 4.8 --cx 1.5 --cy 1.0 [--mp4 out/x.mp4]
(view-w / cx / cy 是 isaac_plant.py 印的 `topview: 正交 視野寬 … 中心 (…)`)
"""
import argparse, csv, glob, math, pathlib, subprocess
import numpy as np
from PIL import Image

ap = argparse.ArgumentParser()
ap.add_argument("frames"); ap.add_argument("csv")
ap.add_argument("--every", type=int, default=20); ap.add_argument("--view-w", type=float, required=True)
ap.add_argument("--cx", type=float, required=True); ap.add_argument("--cy", type=float, required=True)
ap.add_argument("--mp4", default=None); ap.add_argument("--fps", type=float, default=10.0)
a = ap.parse_args()
rows = list(csv.DictReader(open(a.csv)))
files = sorted(glob.glob(str(pathlib.Path(a.frames) / "frame_*.png")))
worst = (0.0, -1); n_ok = 0
for k, f in enumerate(files):
    arr = np.asarray(Image.open(f).convert("RGB")).astype(int)
    H, W = arr.shape[:2]; sx = W / a.view_w
    blue = (arr[:, :, 2] - arr[:, :, 0] > 60) & (arr[:, :, 2] - arr[:, :, 1] > 20) & (arr[:, :, 2] > 120)
    ys, xs = np.nonzero(blue)
    step = min(k * a.every, len(rows) - 1)
    if not len(xs):
        print(f"幀 {k}(step {step}):沒找到底盤"); continue
    wx = (xs.mean() - W / 2) / sx + a.cx; wy = a.cy - (ys.mean() - H / 2) / sx
    tx, ty = float(rows[step]["plant_x"]) / 1000, float(rows[step]["plant_y"]) / 1000
    d = math.hypot(wx - tx, wy - ty); n_ok += 1
    if d > worst[0]: worst = (d, k)
print(f"[topcam_check] {len(files)} 幀,{n_ok} 幀找到底盤;相機重心對 CSV 真值最大偏差 {worst[0] * 1000:.0f} mm(幀 {worst[1]},判準 < 200);"
      f" 末幀 step {min((len(files) - 1) * a.every, len(rows) - 1)} 真值 ({float(rows[min((len(files) - 1) * a.every, len(rows) - 1)]['plant_x']):.1f}, {float(rows[min((len(files) - 1) * a.every, len(rows) - 1)]['plant_y']):.1f}) mm")
if a.mp4:
    import imageio_ffmpeg
    ff = imageio_ffmpeg.get_ffmpeg_exe()
    subprocess.run([ff, "-y", "-loglevel", "error", "-framerate", str(a.fps), "-i", str(pathlib.Path(a.frames) / "frame_%05d.png"),
                    "-pix_fmt", "yuv420p", "-crf", "23", a.mp4], check=True)
    print(f"[topcam_check] 寫出 {a.mp4}")
