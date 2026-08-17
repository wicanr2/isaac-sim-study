#!/usr/bin/env python3
"""稽核一個 USD 資產有沒有**授權**物理參數(質量、密度、碰撞近似)。

## 為什麼需要

建模擬場域時最常見的誤解是「拿了官方資產就有物理」。**不一定** ——
很多資產包只有幾何與材質,物理是使用者自己要補的
(實測案例見 docs/common/18-finding-physical-parameters §2)。

而 PhysX 對未授權的質量會用「網格體積 × 預設密度 1000 kg/m³」自動算 ——
**1000 是水的密度**,鋼構件會因此輕約 7.85 倍,而且**不會有任何警告**。

## ⚠ 不要用 grep 判斷

`.usd` 是二進位 crate。實測對某些資產 grep:`Mesh`/`points` 抓得到,
但 `UsdGeom`/`xformOp` 抓不到 —— **部分有效等於不能當判準**。
一定要用 USD API 開。

## ⚠ instanced 資產:`Traverse()` 會回 0 個 mesh

CAD 轉換器預設就開 instancing。被 instance 的內容住在 prototype 裡,
預設的 `Stage.Traverse()` 走不進去,於是「這個檔案是空的」是個常見的誤判。
本腳本會一併走訪 `GetPrototypes()`,並在輸出裡標明。
(見 docs/common/21-cad-asset-reading-and-conversion)

## 在哪跑

需要 USD Python 綁定。Isaac Sim 容器內:

    USDLIB=$(ls -d /isaac-sim/extscache/omni.usd.libs-*/ | head -1)
    export PYTHONPATH=${USDLIB}:$PYTHONPATH
    export LD_LIBRARY_PATH=${USDLIB}bin:$LD_LIBRARY_PATH
    /isaac-sim/kit/python/bin/python3 audit_asset_physics.py <usd> [<usd> ...]

## 輸出怎麼讀

    RigidBody   這個 prim 會被物理模擬
    mass/density  **空白就是吃預設 1000 kg/m³**,不是「零」
    Collision   有沒有碰撞體;approximation 是哪一種

⚠ 這支腳本只證明「檔案裡授權了什麼」。**它證明不了 runtime 會採用那個值** ——
runtime 寫入不一定被 PhysX 收下,要證明生效只有「設極端值看行為差異」。
見 docs/6.0.1/17-physics-parameter-tuning-6.0 §5。
"""
import sys

from pxr import Usd, UsdGeom, UsdPhysics


def iter_prims(stage):
    """走訪整個 stage,包含 instance prototype。

    回傳 (prim, 是否在 prototype 裡)。
    """
    for p in stage.Traverse():
        yield p, False
    for proto in stage.GetPrototypes():
        for p in Usd.PrimRange(proto):
            yield p, True


def audit(path, limit=25):
    try:
        stage = Usd.Stage.Open(path)
    except Exception as exc:
        print("  ✗ 開不起來:%s" % exc)
        return
    if stage is None:
        print("  ✗ 開不起來:Stage.Open 回 None")
        return

    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    n_total = n_proto = 0
    rb = mass = dens = coll = 0
    rows = []

    for prim, in_proto in iter_prims(stage):
        n_total += 1
        n_proto += in_proto
        has_rb = prim.HasAPI(UsdPhysics.RigidBodyAPI)
        has_mass = prim.HasAPI(UsdPhysics.MassAPI)
        has_coll = prim.HasAPI(UsdPhysics.CollisionAPI)
        if not (has_rb or has_mass or has_coll):
            continue
        rb += has_rb
        coll += has_coll

        m = d = None
        if has_mass:
            api = UsdPhysics.MassAPI(prim)
            attr = api.GetMassAttr()
            if attr and attr.HasAuthoredValue():
                m = attr.Get()
                mass += 1
            attr = api.GetDensityAttr()
            if attr and attr.HasAuthoredValue():
                d = attr.Get()
                dens += 1

        approx = ""
        if has_coll:
            attr = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr()
            if attr:
                approx = attr.Get() or ""

        rows.append((prim.GetPath().pathString, has_rb, m, d, approx, in_proto))

    print("  prim 總數 %d(其中 %d 個在 instance prototype 裡)  metersPerUnit=%s"
          % (n_total, n_proto, mpu))
    print("  RigidBody %d   授權 mass %d   授權 density %d   Collision %d"
          % (rb, mass, dens, coll))

    if not rows:
        print("  → **完全沒有物理 API** —— 這是純幾何/材質資產,物理要自己補")
        return
    if mass == 0 and dens == 0:
        print("  ⚠ **有物理體但沒有任何授權質量或密度** —— "
              "PhysX 會用 網格體積 × 1000 kg/m³(水)自動算")

    for path_str, has_rb, m, d, approx, in_proto in rows[:limit]:
        print("    %-56s rb=%-5s mass=%-8s density=%-8s %s%s"
              % (path_str[-56:], has_rb,
                 m if m is not None else "—",
                 d if d is not None else "—",
                 approx,
                 "  [prototype]" if in_proto else ""))
    if len(rows) > limit:
        print("    …(還有 %d 個)" % (len(rows) - limit))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    for target in sys.argv[1:]:
        print("\n=== %s ===" % target)
        audit(target)
