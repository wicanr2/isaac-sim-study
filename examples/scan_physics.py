#!/usr/bin/env python3
"""掃描 USD 場景裡所有與物理相關的 authored 屬性,列出「現在設了什麼」。

只列 authored(場景檔真的寫下的值),不列 schema 預設 —— 兩者要分開看,
才知道哪些是刻意設定、哪些是吃預設。

用法: scan_physics.py <scene.usd> [prim_path ...]
不給 prim_path 時掃預設的一組關鍵 prim。
"""
import sys

from pxr import Usd

DEFAULT_TARGETS = [
    "/World/AMR_MR1533/PhysicsScene",
    "/World/AMR_MR1533",
    "/World/AMR_MR1533/main",
    "/World/AMR_MR1533/main/fork_liftA1",
    "/World/AMR_MR1533/main/fork_tilt",
    "/World/AMR_MR1533/main/fork_tilt/fork_tilt_01",
    "/World/PhysicsMaterials/high_friction_fork_pallet",
    "/target_pallet",
    "/target_pallet/target_pallet",
    "/target_pallet/target_pallet/SM_RecycledWoodPallet_A04_01",
    "/target_pallet/target_pallet/Cube",
]

PHYS_PREFIX = ("physics:", "physx", "newton", "material:binding")


def scan(stage, path):
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        print("  %-58s <不存在>" % path)
        return
    prim.Load()
    rows = []
    for attr in prim.GetAttributes():
        name = attr.GetName()
        if not name.startswith(PHYS_PREFIX):
            continue
        if not attr.HasAuthoredValue():
            continue
        try:
            rows.append((name, attr.Get()))
        except Exception as e:
            rows.append((name, "<讀取失敗 %s>" % e))
    # apiSchemas 決定貼了哪些標籤
    apis = prim.GetMetadata("apiSchemas")
    api_list = list(apis.appendedItems) if apis else []

    print("\n── %s <%s>" % (path, prim.GetTypeName()))
    if api_list:
        print("   apiSchemas: %s" % ", ".join(str(a) for a in api_list))
    if not rows:
        print("   (無 authored 物理屬性)")
    for name, val in sorted(rows):
        print("   %-46s = %s" % (name, val))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    scene = sys.argv[1]
    targets = sys.argv[2:] or DEFAULT_TARGETS
    stage = Usd.Stage.Open(scene, load=Usd.Stage.LoadNone)
    print("=== %s ===" % scene)
    for t in targets:
        scan(stage, t)
    return 0


if __name__ == "__main__":
    sys.exit(main())
