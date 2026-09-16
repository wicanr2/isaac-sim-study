#!/usr/bin/env python3
"""從 out/run.csv 算步階響應:上升時間(10→90%)、超調、進入 ±2% 帶的時間、穩態誤差、最大加速度。

    python3 tools/step_response.py out/run.csv [--col plant_vl] [--t0 0.5] [--sp 300] [--window 3.0]

只看步階後 --window 秒(預設腳本 0.5 s 起步、3.5 s 轉向,所以是 3 s);
穩態取視窗最後 0.5 s 平均。只用標準庫;--col 可換 meas_l(韌體量到的)或 plant_vr。
"""
import argparse
import csv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--col", default="plant_vl")
    ap.add_argument("--t0", type=float, default=0.5, help="步階發生的時刻(s,腳本時間)")
    ap.add_argument("--sp", type=float, default=300.0)
    ap.add_argument("--dt", type=float, default=0.005)
    ap.add_argument("--window", type=float, default=3.0, help="步階後看幾秒")
    a = ap.parse_args()
    rows = list(csv.DictReader(open(a.csv)))
    k0 = int(round(a.t0 / a.dt))
    v = [float(r[a.col]) for r in rows]
    seg = v[k0:k0 + int(round(a.window / a.dt))]
    ss = sum(seg[-100:]) / 100.0                      # 最後 0.5 s 平均當穩態
    t10 = next((i for i, x in enumerate(seg) if x >= 0.1 * a.sp), None)
    t90 = next((i for i, x in enumerate(seg) if x >= 0.9 * a.sp), None)
    peak = max(seg)
    band = next((i for i in range(len(seg)) if all(abs(x - a.sp) <= 0.02 * a.sp for x in seg[i:i + 40])), None)
    accel = max(abs(seg[i] - seg[i - 1]) / a.dt for i in range(1, len(seg)))
    print(f"{a.col}: 步階 {a.sp:.0f} @ {a.t0}s,看 {a.window}s")
    print(f"  上升時間 10→90%  : {'—' if t10 is None or t90 is None else f'{(t90 - t10) * a.dt * 1000:.0f} ms'}(t90 = {'—' if t90 is None else f'{t90 * a.dt * 1000:.0f} ms'})")
    print(f"  超調            : {(peak - a.sp) / a.sp * 100:.1f}%(峰值 {peak:.1f})")
    print(f"  進入 ±2% 帶     : {'—' if band is None else f'{band * a.dt * 1000:.0f} ms'}")
    print(f"  穩態(最後 0.5 s): {ss:.1f},誤差 {(ss - a.sp) / a.sp * 100:+.2f}%")
    print(f"  最大 |加速度|    : {accel:.0f} mm/s²")


if __name__ == "__main__":
    main()
