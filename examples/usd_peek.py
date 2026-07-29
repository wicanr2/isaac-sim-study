#!/usr/bin/env python3
"""唯讀檢視 USD crate 檔裡某個 prim 的物理結構,並輸出等價的 .usda 文字。

crate 是二進位,編輯器打不開;這支把「你關心的那一小塊」轉成文字給你看。
全程唯讀,不寫回原檔。

用法: usd_peek.py <scene.usd> <prim_path> [--export <out.usda>]

Isaac Sim 容器裡沒有附 usdcat,pxr 也不在預設路徑上。跑之前兩個環境變數
都要接起來 —— 只設 PYTHONPATH 會失敗在 ImportError: libusd_tf.so:

    USDLIB=$(ls -d /isaac-sim/extscache/omni.usd.libs-*/ | head -1)
    export PYTHONPATH=${USDLIB}:$PYTHONPATH
    export LD_LIBRARY_PATH=${USDLIB}bin:$LD_LIBRARY_PATH
    /isaac-sim/kit/python/bin/python3 usd_peek.py scene.usd /target_pallet

實測環境:nvcr.io/nvidia/isaac-sim:6.0.1,126 MB 的 crate 場景可正常讀取。
搭配 docs/16-model-tuning-for-6.0 使用。
"""
import sys

from pxr import Usd, UsdGeom, UsdPhysics


def describe(prim, indent=0):
    pad = "  " * indent
    apis = []
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        apis.append("RigidBody")
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        apis.append("Collision")
    if prim.HasAPI(UsdPhysics.MassAPI):
        m = UsdPhysics.MassAPI(prim).GetMassAttr()
        apis.append("Mass=%s" % (m.Get() if m else None))
    if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        a = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr()
        apis.append("approx=%s" % (a.Get() if a else None))
    tag = (" [" + ", ".join(apis) + "]") if apis else ""
    print("%s%s  <%s>%s" % (pad, prim.GetName(), prim.GetTypeName(), tag))

    # bbox:算體素邊長要用它
    if indent == 0:
        try:
            cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
            r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            size = r.GetSize()
            print("%s  bbox size = (%.3f, %.3f, %.3f) m,最長邊 %.3f m"
                  % (pad, size[0], size[1], size[2], max(size)))
        except Exception as e:
            print("%s  bbox 失敗: %s" % (pad, e))

    for child in prim.GetChildren():
        describe(child, indent + 1)


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    scene, prim_path = sys.argv[1], sys.argv[2]

    stage = Usd.Stage.Open(scene, load=Usd.Stage.LoadNone)
    print("=== 開啟 %s ===" % scene)
    print("up axis = %s, metersPerUnit = %s"
          % (UsdGeom.GetStageUpAxis(stage), UsdGeom.GetStageMetersPerUnit(stage)))

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        print("prim 不存在: %s" % prim_path)
        return 1
    # LoadNone 開的 stage 需要把這一支 payload 載進來才看得到子樹
    prim.Load()

    print("\n=== %s 的物理結構 ===" % prim_path)
    describe(prim)

    if "--export" in sys.argv:
        out = sys.argv[sys.argv.index("--export") + 1]
        # ⚠ 不可用 stage.Flatten():126 MB 的場景攤平會產生數 GB 文字。
        # 只把這一棵子樹的 spec 複製到新 layer。
        from pxr import Sdf
        dst = Sdf.Layer.CreateAnonymous(".usda")
        src_layer = prim.GetPrimStack()[0].layer if prim.GetPrimStack() else stage.GetRootLayer()
        Sdf.CreatePrimInLayer(dst, prim_path)
        Sdf.CopySpec(src_layer, prim_path, dst, prim_path)
        text = dst.ExportToString()
        with open(out, "w") as f:
            f.write(text)
        print("\n已輸出 %s (%d 字元,只含 %s 子樹)" % (out, len(text), prim_path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
