#!/usr/bin/env python3
"""從 world.json 產 Nav2 的佔據地圖(PGM P5 + yaml)。與受控體的假雷射、碰撞用同一份世界,地圖不會跟世界對不上。

    python3 tools/gen_map.py world.json out_dir [--res 0.02] [--boxes]

預設只畫牆:方塊是「地圖上沒有、雷射才看得到」的障礙物——Nav2 得靠 /scan 的 costmap 避開,
負對照 --negative blind-scan(掃描全設最大距離)才有意義。--boxes 把方塊也畫進地圖。
產物是二進位(PGM),放 /tmp 之類的地方,不進版控;run_nav.sh 每次重產。只用標準庫。"""
import argparse
import json
import pathlib


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("world")
    ap.add_argument("out_dir")
    ap.add_argument("--res", type=float, default=0.02)
    ap.add_argument("--boxes", action="store_true", help="方塊也畫進地圖(預設只有牆)")
    a = ap.parse_args()
    w = json.load(open(a.world, encoding="utf-8"))
    r = w["room"]
    res = a.res
    # 地圖比房間各多一格牆厚(牆畫成 1 格佔據)
    x0, y0 = r["x_min"] - res, r["y_min"] - res
    W = int(round((r["x_max"] - r["x_min"]) / res)) + 2
    H = int(round((r["y_max"] - r["y_min"]) / res)) + 2
    free, occ = 254, 0
    grid = [[free] * W for _ in range(H)]
    for j in range(H):
        for i in range(W):
            if i == 0 or j == 0 or i == W - 1 or j == H - 1:
                grid[j][i] = occ
    for b in (w.get("boxes", []) if a.boxes else []):
        i0 = int((b["cx"] - b["w"] / 2 - x0) / res); i1 = int((b["cx"] + b["w"] / 2 - x0) / res)
        j0 = int((b["cy"] - b["h"] / 2 - y0) / res); j1 = int((b["cy"] + b["h"] / 2 - y0) / res)
        for j in range(max(j0, 0), min(j1, H - 1) + 1):
            for i in range(max(i0, 0), min(i1, W - 1) + 1):
                grid[j][i] = occ
    out = pathlib.Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    # PGM 的第一列是地圖的上緣(y 最大),所以列要反過來寫
    data = bytearray()
    for j in range(H - 1, -1, -1):
        data += bytes(grid[j])
    (out / "room.pgm").write_bytes(b"P5\n%d %d\n255\n" % (W, H) + data)
    (out / "room.yaml").write_text(
        "image: room.pgm\nmode: trinary\nresolution: %g\norigin: [%g, %g, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
        % (res, x0, y0), encoding="utf-8")
    print(f"[gen_map] {out/'room.yaml'} {W}x{H} @ {res} m, origin ({x0:g}, {y0:g}), boxes in map: {len(w.get('boxes', [])) if a.boxes else 0}(world has {len(w.get('boxes', []))})")


if __name__ == "__main__":
    main()
