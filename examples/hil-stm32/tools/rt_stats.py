#!/usr/bin/env python3
"""realtime 模式 CSV 的節拍統計:步距(wall_ms 差)的平均/最大、停頓(> 2× dt)次數、分段(驅動 / 轉向)的 Renode/牆鐘比、
受控體最大加速度與撞到馬達層上限(3000)的步。用來回答 35 篇 §5.1 第 4 點:C9 紅的是停頓還是比值。

    python3 tools/rt_stats.py out/run.csv [--dt 5]
只用標準庫。"""
import argparse
import csv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--dt", type=float, default=5.0, help="名目步距 ms")
    a = ap.parse_args()
    R = list(csv.DictReader(open(a.csv)))
    if "wall_ms" not in R[0]:
        print("不是 realtime 的 CSV(沒有 wall_ms 欄)")
        return
    w = [float(r["wall_ms"]) for r in R]
    t = [float(r["t_us"]) / 1000.0 for r in R]
    steps = [w[i] - w[i - 1] for i in range(1, len(w))]
    stalls = [(i, s) for i, s in enumerate(steps, 1) if s > 2 * a.dt]
    print(f"步數 {len(R)};步距 平均 {sum(steps)/len(steps):.2f} ms、最大 {max(steps):.1f} ms、> {2*a.dt:.0f} ms 的停頓 {len(stalls)} 次"
          + (f"(最長 {max(s for _, s in stalls):.1f} ms @ step {max(stalls, key=lambda x: x[1])[0]})" if stalls else ""))
    # 分段:cmd 變化處
    segs = []
    cur = None
    for i, r in enumerate(R):
        c = (r["cmd_v"], r["cmd_w"])
        if cur is None or c != cur[0]:
            if cur:
                segs.append((cur[0], cur[1], i - 1))
            cur = (c, i)
    segs.append((cur[0], cur[1], len(R) - 1))
    for c, i0, i1 in segs:
        if i1 - i0 < 10:
            continue
        rw = (w[i1] - w[i0]) or 1e-9
        rr = t[i1] - t[i0]
        print(f"  cmd {c}: steps {i0}-{i1}  renode/wall = {rr/rw:.3f}  牆鐘 {rw/1000:.2f} s")
    # 受控體加速度
    vl = [float(r["plant_vl"]) for r in R]
    acc = [(i, abs(vl[i] - vl[i - 1]) / ((w[i] - w[i - 1]) / 1000.0)) for i in range(1, len(vl)) if w[i] > w[i - 1]]
    top = sorted(acc, key=lambda x: -x[1])[:3]
    print("  受控體 |dv/dt| 前三(以 wall_ms 差近似;橋接 C9 用的是受控體自己的 dt,停頓步會不同):" + ", ".join(f"{v:.0f} @step {i}(步距 {steps[i-1]:.1f} ms)" for i, v in top))
    ccr = [int(r["ccr1"]) for r in R]
    jumps = [(i, ccr[i - 1], ccr[i]) for i in range(1, len(ccr)) if abs(ccr[i] - ccr[i - 1]) > 150]
    print(f"  CCR1 一步跳超過 150 的次數 {len(jumps)}" + (f",第一次 @step {jumps[0][0]}({jumps[0][1]} → {jumps[0][2]})" if jumps else ""))


if __name__ == "__main__":
    main()
