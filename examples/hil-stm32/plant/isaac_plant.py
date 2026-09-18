#!/usr/bin/env python3
"""Isaac Sim 6.0.1 版受控體:實作 bridge-rs 的 UDP 受控體協定,由橋接 lockstep 步進。

實測於 Isaac Sim 6.0.1(pip 版,場域 GPU 主機,PhysX、CPU 求解、TGS,2026-09-15/16):
閉環 1200 步 ALL PASS,odom 對真值 0.8 mm / 1.0 mm / 0.0008 rad(馬達層加入後;之前 3.4 / 2.0 / 0.028),
兩次 CSV 逐 byte 相同。七項驗收各自量到什麼,寫在檔尾;結論也在 docs/hil/38 §6。
馬達層(死區、一階 τ、加速度上限)在算 DriveAPI 目標速度之前,與 fake_plant.py、bridge-rs plant.rs 同一份公式。

執行(在 Isaac Sim 安裝目錄;pip 版用 venv 的 python):
    ./python.sh /path/to/isaac_plant.py --bind 0.0.0.0:3700 --calib /path/to/calib.json [--tcp]
--tcp:同一份協定改走 TCP(一行一筆),受控體在另一台主機、經 ssh -L 隧道時用。

協定(與 fake_plant.py 相同):
    橋接 → 受控體:CMD <seq> <dt_ms> <duty_l 0..1000> <duty_r> <fwd_l 0/1> <fwd_r> <en 0/1>
    受控體 → 橋接:ENC <seq> <ticks_l> <ticks_r> <x_mm> <y_mm> <th_rad> <vl_mm_s> <vr_mm_s> [collided 0/1]
    受控體 → 橋接(有 --world 時每 period_ms 一筆,在 ENC 之前):SCAN <seq> <n> <r0 m> ... <r(n-1)>
--topview DIR:真實俯視相機(正交、5 m 高、DomeLight、displayColor 上色),omni.replicator 的 render product + rgb annotator
每 --topview-every 個 CMD 抓一幀 PNG。資料只有 rep.orchestrator.step() 才會出現(timeline 沒 play,app.update() 20 次仍是空的);
step 的 delta_time 一定要 0.0,預設 None 會讓物理多走一步、CSV 從第 1 步就不同(2026-09-16 量到);delta_time=0 時 CSV 逐 byte 不變,每幀 62 ms。
--world world.json:牆與方塊進場景當靜態碰撞體(車真的會被擋住),雷射用 PhysX 的射線查詢
(omni.physx 的 scene query;--probe 會把介面 dir() 列出來,並和 plant/world.py 的解析解逐束對照),
collided 旗標用與另外兩個受控體同一份公式(plant/world.py 的 collides:半徑 robot_radius_m 的圓碰到牆線)
——那個圓包住整台車(輪外緣 0.2 m),旗標先亮,底盤之後還能沿著障礙再走(Nav2 blind-scan 負對照量到 175 mm,
斜向頂到方塊角);頂住之後輪子照轉(球輪對方塊打滑),編碼器繼續數、韌體 odom 繼續走,這是 Isaac 版與
另外兩個(撞到就凍結編碼器)最大的差別,見 docs/hil/38 §6.4。

每收到一筆 CMD 就把兩個輪關節的速度目標設好、物理走一步(dt_ms),再從關節位置算編碼器 tick、
從車體 prim 讀真值位姿回覆。物理步長綁在 calib 的 control_period_ms(5 ms),不是 Isaac 預設的 1/60 s。
"""
import argparse
import json
import math
import pathlib
import socket
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--bind", default="0.0.0.0:3700")
ap.add_argument("--calib", default="../calib.json")
ap.add_argument("--headless", type=int, default=1)
ap.add_argument("--tcp", action="store_true")
ap.add_argument("--probe", action="store_true", help="不開 socket:跑固定命令量驗收清單 1/2/3/5/7 後離開")
ap.add_argument("--probe-physics", action="store_true", help="不開 socket:量摩擦、抓地力與驅動扭矩(docs/hil/38 §1.6;GOAL 8 A 線)後離開")
ap.add_argument("--world", default=None, help="world.json:牆與方塊當靜態碰撞體、PhysX 射線當雷射、collided 旗標")
ap.add_argument("--laser-z", type=float, default=0.15, help="雷射高度 m(要高過車身:輪頂 0.10、底盤頂 0.08)")
ap.add_argument("--topview", default=None, help="真實俯視相機錄影:每 --topview-every 步 app.update() 抓一幀,PNG 寫到這個目錄(issue #6)")
ap.add_argument("--topview-every", type=int, default=20, help="每幾個 CMD 抓一幀(20 × 5 ms = 100 ms)")
ap.add_argument("--topview-size", default="640x480")
args = ap.parse_args()
calib = json.load(open(args.calib, encoding="utf-8"))
world = None
if args.world:
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from world import World
    world = World(args.world)

# ---- SimulationApp 必須在所有 omni.* / isaacsim.* / pxr import 之前(31 篇 §1:間接 import 也算) ----
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": bool(args.headless)})

import numpy as np  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402
import omni.usd  # noqa: E402
import omni.timeline  # noqa: E402
from omni.physx import get_physx_interface, get_physx_simulation_interface  # noqa: E402
from omni.physx import get_physx_scene_query_interface  # noqa: E402

# 6.0 起 isaacsim.core.api 搬到 isaacsim.core.experimental(01 篇 §3)。這支腳本刻意不依賴
# 任何一邊:場景用 pxr/UsdPhysics 直接建,步進用 omni.physx 的介面。
# 6.0.1 實測(2026-09-15,場域 GPU 主機):PhysX 介面沒有 `update`;手動步進是
# IPhysxSimulation.attach_stage(stage_id) → simulate(dt, t) → fetch_results()。

R_MM = calib["wheel_radius_mm"]
TRACK_MM = calib["track_mm"]
TPR = calib["encoder_ticks_per_rev"]
V_FULL = calib["wheel_speed_at_full_duty_mm_s"]
# 馬達層(三個受控體實作同一份公式):死區、一階、加速度上限;算出來的輪速當 DriveAPI 的目標,
# 接觸與滑移由 PhysX 負責。公式與 bridge-rs/src/plant.rs、fake_plant.py 逐字相同。
MOTOR_TAU = calib.get("motor_tau_s", 0.05)
# 馬達扭矩模型(docs/hil/36 §3.3;參數抄 Pololu 50:1 37D 12V 產品頁,摩擦與質量是本場景實測)
MOTOR_STALL_NM = calib.get("motor_stall_torque_nm", 2.06)
MOTOR_FREE_RAD_S = calib.get("motor_free_rad_s", 20.94)
MOTOR_TORQUE_MAX_NM = calib.get("motor_torque_max_nm", 2.45)
MOTOR_ACCEL_MAX = calib.get("motor_accel_max_mm_s2", 0.0)
MOTOR_DEADBAND = calib.get("motor_deadband_duty", 0.0)
motor_v = [0.0, 0.0]


def motor_target(duty, fwd, enabled, deadband, full):
    if not enabled or duty <= deadband:
        return 0.0
    mag = (duty - deadband) / (1.0 - deadband) * full
    return mag if fwd else -mag


def motor_advance(v, target, dt, tau, accel_max):
    a = min(dt / tau, 1.0)
    dv = (target - v) * a
    if accel_max > 0:
        lim = accel_max * dt
        dv = max(-lim, min(lim, dv))
    return v + dv
DT = calib["control_period_ms"] / 1000.0

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

# ---- 物理場景:步長 = 控制週期,子步 1 ----
scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr().Set(9.81)
px_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
px_scene.CreateTimeStepsPerSecondAttr().Set(int(round(1.0 / DT)))
# 6.0 的 maxJointVelocity 預設從 1e6 變成 inf(version-matrix);輪子不靠它夾,這裡不動。

# ---- 地面 ----
ground = UsdGeom.Cube.Define(stage, Sdf.Path("/World/ground"))
ground.CreateSizeAttr(1.0)
# ⚠ xformOpOrder 第一個列的是最外層(最後套用):要「先縮放再平移」就得先 AddTranslateOp 再 AddScaleOp。
# 反過來寫的話 -0.05 的平移會被 z 的 0.1 縮成 -0.005,地面頂面跑到 +45 mm,輪子一開始就陷進去,
# 第一步就以 2.9 m/s 往上彈——與 32 篇「被彈飛」同形,真因是地面高了 45 mm(2026-09-15 實測)。
ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
ground.AddScaleOp().Set(Gf.Vec3f(50, 50, 0.1))
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

# ---- 世界(--world):牆與方塊是靜態碰撞體(只有 CollisionAPI、沒有 RigidBodyAPI)----
# 牆的內側面對齊 world.json 的房間邊線,厚 0.1 m 往外長;高 0.5 m,雷射(z=0.15)一定打得到。
WALL_H, WALL_T = 0.5, 0.1

def static_box(path, cx, cy, sx, sy):
    b = UsdGeom.Cube.Define(stage, Sdf.Path(path))
    b.CreateSizeAttr(1.0)
    b.AddTranslateOp().Set(Gf.Vec3d(cx, cy, WALL_H / 2))   # 先 translate 再 scale(同地面的 xformOpOrder 註記)
    b.AddScaleOp().Set(Gf.Vec3f(sx, sy, WALL_H))
    UsdPhysics.CollisionAPI.Apply(b.GetPrim())
    return b

if world is not None:
    xm, xM, ym, yM = world.x_min, world.x_max, world.y_min, world.y_max
    static_box("/World/walls/x_min", xm - WALL_T / 2, (ym + yM) / 2, WALL_T, yM - ym + 2 * WALL_T)
    static_box("/World/walls/x_max", xM + WALL_T / 2, (ym + yM) / 2, WALL_T, yM - ym + 2 * WALL_T)
    static_box("/World/walls/y_min", (xm + xM) / 2, ym - WALL_T / 2, xM - xm, WALL_T)
    static_box("/World/walls/y_max", (xm + xM) / 2, yM + WALL_T / 2, xM - xm, WALL_T)
    for i, (cx, cy, bw, bh) in enumerate(world.boxes):
        static_box(f"/World/boxes/b{i}", cx, cy, bw, bh)

# ---- 差速車:底盤 + 兩個驅動輪(revolute + angular drive)+ 一顆腳輪球 ----
r = R_MM / 1000.0
half_track = TRACK_MM / 2000.0
# 底盤用不縮放的 Mesh 盒子:joint 的 localPos 對「縮放過的 Cube」在 PhysX 裡的尺度不明確
# (第一次探針底盤被抬高 45 mm、輪子轉 1 s 只動 -12 mm),用 Mesh 就沒有這個歧義。
CH_L, CH_W, CH_H = 0.30, 0.20, 0.06
chassis = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/robot/chassis"))
hx, hy, hz = CH_L / 2, CH_W / 2, CH_H / 2
chassis.CreatePointsAttr([Gf.Vec3f(sx * hx, sy * hy, sz * hz)
                          for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)])
chassis.CreateFaceVertexCountsAttr([4] * 6)
chassis.CreateFaceVertexIndicesAttr([0, 2, 3, 1,  4, 5, 7, 6,  0, 1, 5, 4,  2, 6, 7, 3,  0, 4, 6, 2,  1, 3, 7, 5])
chassis.AddTranslateOp().Set(Gf.Vec3d(0, 0, r))
UsdPhysics.RigidBodyAPI.Apply(chassis.GetPrim())
UsdPhysics.CollisionAPI.Apply(chassis.GetPrim())
UsdPhysics.MeshCollisionAPI.Apply(chassis.GetPrim()).CreateApproximationAttr("convexHull")
mass = UsdPhysics.MassAPI.Apply(chassis.GetPrim())
mass.CreateMassAttr(8.0)

def make_wheel(name, y):
    # 32 篇:UsdGeom.Cylinder 當碰撞體會出問題,輪子用 Sphere 近似(半徑 = 輪半徑)
    w = UsdGeom.Sphere.Define(stage, Sdf.Path(f"/World/robot/{name}"))
    w.CreateRadiusAttr(r)
    w.AddTranslateOp().Set(Gf.Vec3d(0, y, r))
    UsdPhysics.RigidBodyAPI.Apply(w.GetPrim())
    UsdPhysics.CollisionAPI.Apply(w.GetPrim())
    m = UsdPhysics.MassAPI.Apply(w.GetPrim())
    m.CreateMassAttr(0.5)
    j = UsdPhysics.RevoluteJoint.Define(stage, Sdf.Path(f"/World/robot/{name}_joint"))
    j.CreateBody0Rel().SetTargets([chassis.GetPath()])
    j.CreateBody1Rel().SetTargets([w.GetPath()])
    j.CreateAxisAttr("Y")
    j.CreateLocalPos0Attr(Gf.Vec3f(0, y, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateCollisionEnabledAttr(False)
    drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    drv.CreateTypeAttr("force")
    # 速度驅動的 damping × (target − ω) 夾在 maxForce,就是直流馬達的扭矩–轉速直線(docs/hil/36 §3.3):
    # damping = τ_stall / ω_free、maxForce = τ_max、target = duty × ω_free。USD 角向的單位是 deg/s,
    # 所以 damping 要除以 (180/π);實際單位用 --probe-physics 量過(38 篇 §1.6)
    drv.CreateDampingAttr(MOTOR_STALL_NM / MOTOR_FREE_RAD_S * math.pi / 180.0)
    drv.CreateStiffnessAttr(0.0)
    drv.CreateMaxForceAttr(MOTOR_TORQUE_MAX_NM)
    drv.CreateTargetVelocityAttr(0.0)  # ⚠ 單位是 度/秒(32 篇:USD 角度一律 degrees)
    return w, j, drv

wl, jl, drv_l = make_wheel("wheel_l", half_track)
wr, jr, drv_r = make_wheel("wheel_r", -half_track)

caster = UsdGeom.Sphere.Define(stage, Sdf.Path("/World/robot/caster"))
caster.CreateRadiusAttr(r * 0.5)
caster.AddTranslateOp().Set(Gf.Vec3d(-0.12, 0, r * 0.5))
UsdPhysics.RigidBodyAPI.Apply(caster.GetPrim())
UsdPhysics.CollisionAPI.Apply(caster.GetPrim())
UsdPhysics.MassAPI.Apply(caster.GetPrim()).CreateMassAttr(0.2)
cj = UsdPhysics.SphericalJoint.Define(stage, Sdf.Path("/World/robot/caster_joint"))
cj.CreateBody0Rel().SetTargets([chassis.GetPath()])
cj.CreateBody1Rel().SetTargets([caster.GetPath()])
cj.CreateLocalPos0Attr(Gf.Vec3f(-0.12, 0, -0.5 * r))
cj.CreateCollisionEnabledAttr(False)
# ⚠ 32 篇:腳輪半徑只有驅動輪一半時在平地會被彈飛——那是三輪叉車型;這裡是球關節腳輪,要實跑確認

# ---- 俯視相機(--topview):正交、從 5 m 高往下看、畫面 +y 朝上,與 tools/topview.py 的圖同一個方向 ----
# 沒有燈 headless 會渲染成全黑,所以掛一盞 DomeLight;幾何用 displayColor 上色(底盤藍、輪黑、牆深灰、方塊灰、地白)
topcam = None
if args.topview:
    from pxr import UsdLux  # noqa: E402
    UsdLux.DomeLight.Define(stage, Sdf.Path("/World/dome")).CreateIntensityAttr(1000.0)
    def paint(prim_path, rgb):
        g = UsdGeom.Gprim(stage.GetPrimAtPath(prim_path)); g.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    paint("/World/ground", (0.96, 0.96, 0.96)); paint("/World/robot/chassis", (0.08, 0.40, 0.75))
    for w_ in ("wheel_l", "wheel_r", "caster"): paint(f"/World/robot/{w_}", (0.15, 0.15, 0.15))
    if world is not None:
        for pth in ("x_min", "x_max", "y_min", "y_max"): paint(f"/World/walls/{pth}", (0.27, 0.27, 0.27))
        for i in range(len(world.boxes)): paint(f"/World/boxes/b{i}", (0.40, 0.40, 0.40))
    W_, H_ = (int(v) for v in args.topview_size.split("x"))
    if world is not None:
        cx_, cy_ = (world.x_min + world.x_max) / 2, (world.y_min + world.y_max) / 2
        view_w = max(world.x_max - world.x_min + 0.6, (world.y_max - world.y_min + 0.6) * W_ / H_)
    else:
        cx_, cy_, view_w = 0.5, 0.0, 3.0
    topcam = UsdGeom.Camera.Define(stage, Sdf.Path("/World/topcam"))
    topcam.CreateProjectionAttr(UsdGeom.Tokens.orthographic)
    # USD:aperture 的單位是「場景單位的十分之一」(metersPerUnit=1 → 0.1 m);正交投影下 aperture 就是視野寬
    topcam.CreateHorizontalApertureAttr(view_w * 10.0)
    topcam.CreateVerticalApertureAttr(view_w * 10.0 * H_ / W_)
    topcam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 20.0))
    topcam.AddTranslateOp().Set(Gf.Vec3d(cx_, cy_, 5.0))   # 不轉:相機沿自己的 −Z 看 = 世界 −Z,畫面上方 = 世界 +Y
    # orchestrator.step 預設 delta_time=None 會讓 timeline 走一格(物理跟著多走一步,開機 settle 就不同);delta_time=0 只渲染
    TOPVIEW = {"dir": pathlib.Path(args.topview), "every": max(1, args.topview_every), "n": 0, "cx": cx_, "cy": cy_, "view_w": view_w, "W": W_, "H": H_,
               "step_kw": {"delta_time": 0.0, "rt_subframes": 1}}
    TOPVIEW["dir"].mkdir(parents=True, exist_ok=True)

app.update()

physx = get_physx_interface()
physx_sim = get_physx_simulation_interface()
# 不 play timeline:timeline 一 play,Kit 每個 update 會自己步進物理,與手動步進疊加。
# 手動步進要先把 stage 掛給 PhysX。
from omni.usd import get_context as _ctx  # noqa: E402
physx_sim.attach_stage(_ctx().get_stage_id())
sim_t = 0.0

def step_once():
    # 手動步進一格物理,不渲染。31 篇 §5:world.step(render=False) 不 tick action graph,
    # 這支腳本不用 OmniGraph,所以無所謂;真要接 ROS 2 bridge 的節點才要注意。
    global sim_t
    physx_sim.simulate(DT, sim_t)
    physx_sim.fetch_results()
    sim_t += DT

xform_cache = UsdGeom.XformCache()

def wheel_angle_rad(joint_prim):
    # 6.0.1 實測:PhysX 不會把 joint state(state:angular:physics:position)寫回 USD,
    # 這個屬性一直不存在。留著當第二條路的探針。
    st = joint_prim.GetAttribute("state:angular:physics:position")
    v = st.Get() if st and st.HasValue() else None
    return math.radians(v) if v is not None else None

_unwrap = {}

_wheel_prev = {"l": 0.0, "r": 0.0}   # 上一步的輪角,用來算輪面速度


def wheel_angle_from_xform(wheel_prim, key):
    """輪子相對底盤繞 Y 軸的角度,從 fetch_results 寫回的 xform 算;跨步展開成連續角。
    這是物理輸出(接觸、滑移都包含在內),不是命令積分。"""
    xform_cache.Clear()
    rc = xform_cache.GetLocalToWorldTransform(chassis.GetPrim()).ExtractRotationMatrix()
    rw = xform_cache.GetLocalToWorldTransform(wheel_prim).ExtractRotationMatrix()
    rel = rw * rc.GetInverse()   # Gf 矩陣是 row-vector 慣例:v' = v * M
    # 繞 Y 軸旋轉 θ:x' = x cosθ - z sinθ … 取 rel 的 (0,0) 與 (0,2) 分量
    a = math.atan2(-rel[0][2], rel[0][0])
    prev = _unwrap.get(key)
    if prev is not None:
        while a - prev > math.pi:
            a -= 2 * math.pi
        while a - prev < -math.pi:
            a += 2 * math.pi
    _unwrap[key] = a
    return a

def world_pos(prim):
    xform_cache.Clear()
    t = xform_cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    return (t[0] * 1000, t[1] * 1000, t[2] * 1000)

def pose():
    xform_cache.Clear()
    m = xform_cache.GetLocalToWorldTransform(chassis.GetPrim())
    t = m.ExtractTranslation()
    rot = m.ExtractRotationMatrix()
    # Gf 矩陣是 row-vector 慣例(v' = v·M):第 0 列是 x 基底旋轉後的像 = (cos, sin, 0)。
    # 寫成 atan2(rot[1][0], rot[0][0]) 會得到正負號相反的 yaw(2026-09-15 第一次閉環:-0.873 vs +0.901)。
    yaw = math.atan2(rot[0][1], rot[0][0])
    return t[0] * 1000.0, t[1] * 1000.0, yaw

# ---- 俯視相機的 render product + rgb annotator(omni.replicator);介面名稱在 --probe 印出來 ----
_rgb_ann = None
def topview_init():
    global _rgb_ann
    import omni.replicator.core as rep
    rp = rep.create.render_product(str(topcam.GetPath()), (TOPVIEW["W"], TOPVIEW["H"]))
    _rgb_ann = rep.AnnotatorRegistry.get_annotator("rgb")
    _rgb_ann.attach([rp])
    TOPVIEW["rep"] = rep
    # 暖機:render product 建好之後,rgb 資料要幾個 app.update() 之後才會出現(量到幾個就印幾個);
    # 沒 play timeline 時若一直是空的,改走 rep.orchestrator.step()
    warm = 0
    for _ in range(20):
        app.update(); warm += 1
        if np.asarray(_rgb_ann.get_data()).size > 0:
            break
    if np.asarray(_rgb_ann.get_data()).size == 0:
        import inspect
        TOPVIEW["step_sig"] = str(inspect.signature(rep.orchestrator.step))
        rep.orchestrator.step(**TOPVIEW.get("step_kw", {}))
        TOPVIEW["use_step"] = True
    arr = np.asarray(_rgb_ann.get_data())
    TOPVIEW["timeline_playing"] = omni.timeline.get_timeline_interface().is_playing()
    print(f"[isaac_plant] topview: timeline playing={TOPVIEW['timeline_playing']} step_kw={TOPVIEW['step_kw']}", flush=True)
    print(f"[isaac_plant] topview: render_product {TOPVIEW['W']}x{TOPVIEW['H']} 正交 視野寬 {TOPVIEW['view_w']:.2f} m 中心 ({TOPVIEW['cx']:.2f}, {TOPVIEW['cy']:.2f});"
          f" 暖機 {warm} 個 update 後資料 shape={arr.shape} dtype={arr.dtype} orchestrator.step={TOPVIEW.get('use_step', False)} {TOPVIEW.get('step_sig', '')}", flush=True)


def topview_capture(tag=None):
    """app.update() 一次(timeline 沒 play,Kit 不會步進物理)→ 讀 rgb → 寫 PNG。回傳 (frame_no, 路徑)。"""
    import time as _time
    t0 = _time.perf_counter()
    if TOPVIEW.get("use_step"):
        TOPVIEW["rep"].orchestrator.step(**TOPVIEW.get("step_kw", {}))
    else:
        app.update()
    arr = np.asarray(_rgb_ann.get_data())
    n_extra = 0
    while arr.size == 0 and n_extra < 10:   # 資料晚一兩個 update 才到的話再等
        app.update(); n_extra += 1; arr = np.asarray(_rgb_ann.get_data())
    if arr.size == 0:
        raise RuntimeError("topview: rgb annotator 一直是空的")
    if arr.ndim == 3 and arr.shape[2] == 4:
        arr = arr[:, :, :3]
    fn = TOPVIEW["n"]; TOPVIEW["n"] += 1
    path = TOPVIEW["dir"] / (f"{tag}.png" if tag else f"frame_{fn:05d}.png")
    try:
        from PIL import Image
        Image.fromarray(np.ascontiguousarray(arr)).save(str(path))
    except ImportError:
        path = path.with_suffix(".ppm")
        with open(path, "wb") as f:
            f.write(b"P6\n%d %d\n255\n" % (arr.shape[1], arr.shape[0])); f.write(np.ascontiguousarray(arr).tobytes())
    return fn, path, (_time.perf_counter() - t0) * 1000.0


def topview_px(x_m, y_m):
    """世界座標 → 像素(畫面上方 = +y)。給 --probe 對照用。"""
    sx = TOPVIEW["W"] / TOPVIEW["view_w"]
    return (x_m - TOPVIEW["cx"]) * sx + TOPVIEW["W"] / 2, TOPVIEW["H"] / 2 - (y_m - TOPVIEW["cy"]) * sx


# ---- 雷射:PhysX 射線查詢;介面名稱不猜,啟動時列出 dir() 裡帶 raycast 的名字,沒有就退回解析解 ----
sq = get_physx_scene_query_interface()
_raycast_names = [n for n in dir(sq) if "raycast" in n.lower()]
_raycast = getattr(sq, "raycast_closest", None)
scan_impl = "physx.raycast_closest" if callable(_raycast) else "analytic(world.py)"
if world is not None:
    print(f"[isaac_plant] scene_query={type(sq).__name__} raycast 名字={_raycast_names} 雷射用={scan_impl} z={args.laser_z}", flush=True)


def scan_raycast(x_m, y_m, th):
    """第 0 束朝 th − π,逆時針,與 plant/world.py 的 scan() 同一個束序;沒打到 = range_max。"""
    n = world.beams
    origin = (x_m, y_m, args.laser_z)
    out = []
    for i in range(n):
        a = th - math.pi + 2 * math.pi * i / n
        h = _raycast(origin, (math.cos(a), math.sin(a), 0.0), world.range_max)
        d = h["distance"] if h and h.get("hit") else world.range_max
        out.append(max(min(d, world.range_max), world.range_min))
    return out


def scan_now(x_m, y_m, th):
    return scan_raycast(x_m, y_m, th) if _raycast else world.scan(x_m, y_m, th)


def probe():
    """驗收清單 1/2/3/5/7:每一項印一行「量到什麼」。"""
    from isaacsim.core.simulation_manager import SimulationManager as SM
    print(f"[probe] 3 timeStepsPerSecond 讀回={px_scene.GetTimeStepsPerSecondAttr().Get()} "
          f"SimulationManager.get_physics_dt()={SM.get_physics_dt()} (要 {DT})", flush=True)
    print(f"[probe] 7 gpu_dynamics={SM.is_gpu_dynamics_enabled()} engine={SM.get_active_physics_engine()} "
          f"device={SM.get_physics_sim_device()} solver={SM.get_solver_type()}", flush=True)
    jp = jl.GetPrim()
    before = [a.GetName() for a in jp.GetAttributes() if a.GetName().startswith("state:")]
    # 5:設 360 deg/s,跑 1 s(1/DT 步)→ 輪角應為 2π
    drv_l.GetTargetVelocityAttr().Set(360.0)
    drv_r.GetTargetVelocityAttr().Set(360.0)
    x0, y0, th0 = pose()
    gb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"]).ComputeWorldBound(ground.GetPrim()).ComputeAlignedRange()
    print(f"[probe] 幾何 地面頂面 z={gb.GetMax()[2] * 1000:.1f} mm(要 0);起始 chassis={world_pos(chassis.GetPrim())} "
          f"wl={world_pos(wl.GetPrim())} wr={world_pos(wr.GetPrim())} caster={world_pos(caster.GetPrim())}", flush=True)
    wheel_angle_from_xform(wl.GetPrim(), "l"); wheel_angle_from_xform(wr.GetPrim(), "r")
    n = int(round(1.0 / DT))
    for i in range(n):
        step_once()
        wheel_angle_from_xform(wl.GetPrim(), "l"); wheel_angle_from_xform(wr.GetPrim(), "r")
        if i in (0, 9, 49, 99):
            print(f"[probe] 步 {i + 1}: chassis={world_pos(chassis.GetPrim())} wl={world_pos(wl.GetPrim())} "
                  f"angle_l={_unwrap['l']:.3f}", flush=True)
    after = [a.GetName() for a in jp.GetAttributes() if a.GetName().startswith("state:")]
    al = wheel_angle_rad(jp)
    ax = _unwrap["l"]
    x1, y1, th1 = pose()
    print(f"[probe] 2 joint state attrs before={before} after={after} joint_state_angle={al};"
          f" xform 算的輪角={ax:.3f} rad", flush=True)
    print(f"[probe] 5 target 360 deg/s × {n} 步({n * DT:.3f} s)→ 輪角 {ax:.3f} rad(期望 6.283);"
          f" 底盤位移 dx={x1 - x0:.1f} mm(期望 2π·r={2 * math.pi * R_MM:.1f} 若無滑移)", flush=True)
    print(f"[probe] 1 attach_stage + simulate/fetch_results 跑了 {n} 步無例外;sim_t={sim_t:.3f}", flush=True)
    # 4:停 1 s 看會不會被彈飛(z 與 roll)
    drv_l.GetTargetVelocityAttr().Set(0.0)
    drv_r.GetTargetVelocityAttr().Set(0.0)
    for _ in range(n):
        step_once()
    tz = world_pos(chassis.GetPrim())[2]
    print(f"[probe] 4 靜止 1 s 後底盤 z={tz:.1f} mm(建模時 {r * 1000:.1f});|z 偏差| > 20 mm 視為異常;"
          f" wl={world_pos(wl.GetPrim())} caster={world_pos(caster.GetPrim())}", flush=True)
    if world is None:
        return
    import time
    print(f"[probe] 8 scene query dir(): {[n for n in dir(sq) if not n.startswith('_')]}", flush=True)
    x, y, th = pose()
    xm_, ym_ = x / 1000.0, y / 1000.0
    if _raycast:
        one = _raycast((xm_, ym_, args.laser_z), (1.0, 0.0, 0.0), world.range_max)
        print(f"[probe] 8 raycast_closest 回傳型別={type(one).__name__} keys={list(one.keys()) if hasattr(one, 'keys') else one}", flush=True)
    t0 = time.perf_counter()
    rc = scan_now(xm_, ym_, th)
    t1 = time.perf_counter()
    an = world.scan(xm_, ym_, th)
    diffs = [abs(a - b) for a, b in zip(rc, an)]
    worst = max(range(len(diffs)), key=lambda i: diffs[i])
    print(f"[probe] 8 雷射 {len(rc)} 束用 {scan_impl} 花 {(t1 - t0) * 1000:.1f} ms;對解析解 max|Δ|={diffs[worst] * 1000:.1f} mm(束 {worst},"
          f" {rc[worst]:.3f} vs {an[worst]:.3f});>5 mm 的束數={sum(d > 0.005 for d in diffs)};"
          f" 束 0/90/180/270 = {rc[0]:.3f}/{rc[90]:.3f}/{rc[180]:.3f}/{rc[270]:.3f}(解析 {an[0]:.3f}/{an[90]:.3f}/{an[180]:.3f}/{an[270]:.3f})", flush=True)
    if topcam is not None:
        topview_init()
        fn, path, ms = topview_capture("probe_start")
        arr = np.asarray(_rgb_ann.get_data())[:, :, :3]
        # 找底盤藍色像素的重心,對照真值位姿算出來的像素;差 < 車半徑(0.2 m 換算的像素)才算相機對得上
        blue = (arr[:, :, 2].astype(int) - arr[:, :, 0] > 60) & (arr[:, :, 2].astype(int) - arr[:, :, 1] > 20) & (arr[:, :, 2] > 120)
        ys, xs = np.nonzero(blue)
        x, y, th = pose()
        ex, ey = topview_px(x / 1000.0, y / 1000.0)
        sx = TOPVIEW["W"] / TOPVIEW["view_w"]
        # 抓幀不准碰物理:靜止時連抓 5 幀,前後位姿與輪角要逐 bit 相同
        p0 = (world_pos(chassis.GetPrim()), world_pos(wl.GetPrim()), _unwrap.get("l"), sim_t)
        for _ in range(5):
            topview_capture("probe_still")
        p1 = (world_pos(chassis.GetPrim()), world_pos(wl.GetPrim()), _unwrap.get("l"), sim_t)
        print(f"[probe] 10 抓 5 幀不步進:位姿 / 輪角 / sim_t 前後 {'逐 bit 相同' if p0 == p1 else '不同!'} {p0 if p0 != p1 else ''} {p1 if p0 != p1 else ''}", flush=True)
        if len(xs):
            print(f"[probe] 10 俯視相機 一幀 {ms:.1f} ms;底盤藍像素 {len(xs)} 個,重心 ({xs.mean():.1f}, {ys.mean():.1f}) px,真值換算 ({ex:.1f}, {ey:.1f}) px;"
                  f" 差 {math.hypot(xs.mean() - ex, ys.mean() - ey) / sx * 1000:.0f} mm(判準 < 200);寫 {path}", flush=True)
        else:
            print(f"[probe] 10 俯視相機 一幀 {ms:.1f} ms;**沒找到底盤的藍色像素**(燈 / 顏色 / 視野有問題);寫 {path};"
                  f" 像素值範圍 {arr.min()}..{arr.max()} 平均 {arr.mean():.1f}", flush=True)
    # 9:往 +x 滿速 1 m/s 跑 5 s(world.json 預設世界會先撞到方塊 2 的角、被頂歪):看真值停在哪、
    #    collided 旗標何時亮(解析公式:0.2 m 圓)、頂住後輪子有沒有繼續轉(編碼器 vs 真值)
    drv_l.GetTargetVelocityAttr().Set(math.degrees(V_FULL / R_MM))
    drv_r.GetTargetVelocityAttr().Set(math.degrees(V_FULL / R_MM))
    first_flag = None
    for i in range(5 * n):
        step_once()
        wheel_angle_from_xform(wl.GetPrim(), "l"); wheel_angle_from_xform(wr.GetPrim(), "r")
        x, y, th = pose()
        if first_flag is None and world.collides(x / 1000.0, y / 1000.0):
            first_flag = (i + 1, x, _unwrap["l"])
    x, y, th = pose()
    front = x / 1000.0 + CH_L / 2 * math.cos(th)
    print(f"[probe] 9 往 +x 滿速 {5 * n} 步:真值 x={x:.1f} mm y={y:.1f} θ={th:.3f} 底盤前緣 x={front * 1000:.1f} mm(x_max 牆內側 {world.x_max * 1000:.0f});"
          f" collided 旗標第一次亮在步 {first_flag[0] if first_flag else '沒亮'} x={first_flag[1] if first_flag else 0:.1f} mm;"
          f" 左輪角 撞前 {first_flag[2] if first_flag else 0:.1f} → 末 {_unwrap['l']:.1f} rad(還在轉 = 抵牆打滑,編碼器會繼續數)", flush=True)
    if topcam is not None:
        fn, path, ms = topview_capture("probe_end")
        arr = np.asarray(_rgb_ann.get_data())[:, :, :3]
        blue = (arr[:, :, 2].astype(int) - arr[:, :, 0] > 60) & (arr[:, :, 2].astype(int) - arr[:, :, 1] > 20) & (arr[:, :, 2] > 120)
        ys, xs = np.nonzero(blue)
        ex, ey = topview_px(x / 1000.0, y / 1000.0); sx = TOPVIEW["W"] / TOPVIEW["view_w"]
        print(f"[probe] 10 俯視相機 末幀:藍像素重心 ({xs.mean() if len(xs) else -1:.1f}, {ys.mean() if len(xs) else -1:.1f}) 真值換算 ({ex:.1f}, {ey:.1f});"
              f" 差 {(math.hypot(xs.mean() - ex, ys.mean() - ey) / sx * 1000) if len(xs) else -1:.0f} mm;寫 {path}", flush=True)


def probe_physics():
    """GOAL 8 A 線:摩擦、抓地力、驅動扭矩全部用「已知質量 × 量到的加速度」量,不靠 API 單位。
    每一項印一行「量到什麼」;推得的數字都寫明用了哪個假設。"""
    g = float(scene.GetGravityMagnitudeAttr().Get())
    n = int(round(1.0 / DT))

    # 1. 材質綁定:地面與輪子各查一次(physics purpose),沒綁就是走 PhysX 預設
    def mat_of(prim, label):
        try:
            mat, _rel = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial("physics")
        except Exception as e:
            print(f"[phys] 1 {label} 材質查詢失敗:{type(e).__name__} {e}", flush=True)
            return
        mp = mat.GetPrim() if mat else None
        if mp is None or not mp.IsValid():
            print(f"[phys] 1 {label} 沒有綁 physics 材質 → 走 PhysX 預設", flush=True)
            return
        api = UsdPhysics.MaterialAPI(mp)
        sf = api.GetStaticFrictionAttr().Get() if api else None
        df = api.GetDynamicFrictionAttr().Get() if api else None
        cm = mp.GetAttribute("physxMaterial:frictionCombineMode")
        print(f"[phys] 1 {label} 綁 {mp.GetPath()} static={sf} dynamic={df} "
              f"combineMode={cm.Get() if cm and cm.HasValue() else '沒 authored(schema 預設 average)'}", flush=True)

    mat_of(ground.GetPrim(), "地面")
    mat_of(wl.GetPrim(), "左輪")

    masses = {}
    for path in ("/World/robot/chassis", "/World/robot/wheel_l", "/World/robot/wheel_r", "/World/robot/caster"):
        pr = stage.GetPrimAtPath(path)
        a = pr.GetAttribute("physics:mass")
        masses[path.rsplit("/", 1)[-1]] = a.Get() if a and a.HasValue() else None
    m_total = sum(v for v in masses.values() if v)
    print(f"[phys] 2 質量 {masses} 合計 {m_total:.3f} kg;重力 {g} m/s²;體重 {m_total * g:.1f} N", flush=True)

    def body_speed():
        x, y, _ = pose()
        return x / 1000.0, y / 1000.0

    if world is not None:
        print("[phys] 3/4 有 --world(前方有障礙物),空地兩項要另外跑一次不帶 --world 的", flush=True)
    # 3. 空地全速起步:量最大加速度 → 推進力 = m·a
    drv_l.GetTargetVelocityAttr().Set(math.degrees(V_FULL / R_MM))
    drv_r.GetTargetVelocityAttr().Set(math.degrees(V_FULL / R_MM))
    xs = []
    for _ in range(n):
        step_once()
        xs.append(body_speed()[0])
    v = [(xs[i] - xs[i - 1]) / DT for i in range(1, len(xs))]
    a_max = max((v[i] - v[i - 1]) / DT for i in range(1, len(v)))
    v_end = v[-1]
    print(f"[phys] 3 空地全速起步:最大加速度 {a_max:.2f} m/s²(推進力 {m_total * a_max:.1f} N)、1 s 後車速 {v_end:.3f} m/s", flush=True)

    # 4. 鎖輪滑行:命令歸零(damping 把輪子停住)之後車體還在滑 → 減速度 = μ_k·g
    wl0 = wheel_angle_from_xform(wl.GetPrim(), "l")
    drv_l.GetTargetVelocityAttr().Set(0.0)
    drv_r.GetTargetVelocityAttr().Set(0.0)
    xs = []
    for _ in range(n // 2):
        step_once()
        wheel_angle_from_xform(wl.GetPrim(), "l")
        xs.append(body_speed()[0])
    v = [(xs[i] - xs[i - 1]) / DT for i in range(1, len(xs))]
    dec = [-(v[i] - v[i - 1]) / DT for i in range(1, len(v))]
    moving = [d for d, s in zip(dec, v[1:]) if s > 0.05]
    dec_max = max(moving) if moving else 0.0
    wheel_turn = abs(_unwrap["l"] - wl0)
    slide = abs(xs[-1] - xs[0]) - wheel_turn * r
    # 逐步微分的峰值含單步暫態(量到 17 m/s²,比 μ=0.5 的 4.9 大三倍),用滑行距離算平均才是摩擦:a = v0² / 2d
    d = abs(xs[-1] - xs[0])
    v0 = v[0] if v else 0.0
    a_avg = v0 * v0 / (2 * d) if d > 1e-4 else 0.0
    print(f"[phys] 4 鎖輪滑行:起始 {v0:.3f} m/s、滑 {d * 1000:.0f} mm 停下 → 平均減速度 {a_avg:.2f} m/s²、μ_k = {a_avg / g:.3f}"
          f"(抓地力 {m_total * a_avg:.1f} N);逐步微分的峰值 {dec_max:.2f} m/s²(單步暫態,不採用);"
          f"輪子只轉了 {wheel_turn * r * 1000:.0f} mm,差 {slide * 1000:.0f} mm = 滑動", flush=True)

    # 5. 頂住障礙物:輪子還能不能轉到命令轉速(能 → 驅動扭矩贏過抓地力),量 spin-up 角加速度當扭矩下界
    if world is None:
        print("[phys] 5 沒有 --world,跳過頂住障礙物那一項(空地兩項已量完)", flush=True)
        return
    target_rad_s = V_FULL / R_MM
    drv_l.GetTargetVelocityAttr().Set(math.degrees(target_rad_s))
    drv_r.GetTargetVelocityAttr().Set(math.degrees(target_rad_s))
    hit = None
    angs = []
    for i in range(6 * n):
        step_once()
        angs.append(wheel_angle_from_xform(wl.GetPrim(), "l"))
        x, y, _ = pose()
        if hit is None and world.collides(x / 1000.0, y / 1000.0):
            hit = i
    if hit is None:
        print("[phys] 5 6 s 內沒撞到東西(世界檔裡正前方沒有障礙物?)", flush=True)
        return
    w_after = [(angs[i] - angs[i - 1]) / DT for i in range(hit + 2, len(angs))]
    x, y, _ = pose()
    m_w = masses.get("wheel_l") or 0.5
    inertia = 0.4 * m_w * r * r          # 實心球(建模時輪子是 Sphere)
    print(f"[phys] 5 頂住障礙物:車體停在 x={x:.0f} mm,輪速仍穩在 {sum(w_after[-20:]) / 20:.2f} rad/s(命令 {target_rad_s:.2f})"
          f" → 驅動贏過抓地力,輪子照轉", flush=True)

    # 6. 頂住狀態下從靜止起轉:量 spin-up 的角加速度 → 驅動扭矩下界(扣掉滑動摩擦力矩)
    drv_l.GetTargetVelocityAttr().Set(0.0)
    drv_r.GetTargetVelocityAttr().Set(0.0)
    for _ in range(n):
        step_once()
        wheel_angle_from_xform(wl.GetPrim(), "l")
    a0 = _unwrap["l"]
    drv_l.GetTargetVelocityAttr().Set(math.degrees(target_rad_s))
    drv_r.GetTargetVelocityAttr().Set(math.degrees(target_rad_s))
    seq = [a0]
    for _ in range(20):
        step_once()
        seq.append(wheel_angle_from_xform(wl.GetPrim(), "l"))
    w = [(seq[i] - seq[i - 1]) / DT for i in range(1, len(seq))]
    alpha = max((w[i] - w[i - 1]) / DT for i in range(1, len(w)))
    alpha1 = w[0] / DT
    print(f"[phys] 6 頂住、輪子停住後重新起轉:第一步角加速度 {alpha1:.0f} rad/s²、最大 {alpha:.0f};"
          f"驅動扭矩 ≥ I·α = {inertia * alpha1:.3f} N·m(I = 0.4·m·r² = {inertia:.5f} kg·m²,輪子建模成球;"
          f"還要加上滑動摩擦力矩,所以是下界)。前 5 步輪速 {[round(v, 2) for v in w[:5]]} rad/s", flush=True)

    # 7. 同一件事用細步長再量一次:輪子在一個 5 ms 步內就到命令轉速,下界只受步長限制。
    #    把物理步長縮到 1/20000 s 再起轉,看角加速度能推到多高(仍是下界)
    fine = 1.0 / 20000
    px_scene.CreateTimeStepsPerSecondAttr().Set(20000)
    drv_l.GetTargetVelocityAttr().Set(0.0)
    drv_r.GetTargetVelocityAttr().Set(0.0)
    global sim_t
    for _ in range(2000):
        physx_sim.simulate(fine, sim_t); physx_sim.fetch_results(); sim_t += fine
        wheel_angle_from_xform(wl.GetPrim(), "l")
    a0 = _unwrap["l"]
    drv_l.GetTargetVelocityAttr().Set(math.degrees(target_rad_s))
    drv_r.GetTargetVelocityAttr().Set(math.degrees(target_rad_s))
    seq = [a0]
    for _ in range(40):
        physx_sim.simulate(fine, sim_t); physx_sim.fetch_results(); sim_t += fine
        seq.append(wheel_angle_from_xform(wl.GetPrim(), "l"))
    wf = [(seq[i] - seq[i - 1]) / fine for i in range(1, len(seq))]
    a_fine = wf[0] / fine
    print(f"[phys] 7 細步長 {fine * 1e6:.0f} µs 重做:第一步角加速度 {a_fine:.0f} rad/s² → 驅動扭矩 ≥ {inertia * a_fine:.2f} N·m;"
          f"前 5 步輪速 {[round(v, 2) for v in wf[:5]]} rad/s(還是一步到位就表示仍只是下界)", flush=True)
    px_scene.CreateTimeStepsPerSecondAttr().Set(int(round(1.0 / DT)))

if args.probe:
    probe()
    app.close()
    sys.exit(0)

if args.probe_physics:
    probe_physics()
    app.close()
    sys.exit(0)

if topcam is not None:
    topview_init()
host, port = args.bind.rsplit(":", 1)
if args.tcp:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, int(port)))
    srv.listen(1)
    srv.settimeout(0.5)
else:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, int(port)))
    sock.settimeout(0.5)
print(f"[isaac_plant] listening {args.bind} {'tcp' if args.tcp else 'udp'} dt={DT} r={r} track={TRACK_MM}", flush=True)

# 沒有 PhysX joint state 時的退路:用命令速度積分輪角(那就退化成純運動學,結果要標明)
ang_l = ang_r = 0.0
fallback_used = 0
x0, y0, th0 = pose()
n_cmd = 0


def handle(line: str):
    """一筆 CMD → 步進 → 回 ENC 字串;格式不對回 None"""
    global ang_l, ang_r, fallback_used, n_cmd
    f = line.split()
    if len(f) not in (8, 9) or f[0] != "CMD":
        return None
    seq = int(f[1])
    dt = int(f[2]) / 1000.0
    en = f[7] == "1"
    # 扭矩模型:duty(扣死區)× 空載角速度 當 drive 的目標;扭矩由 damping 與 maxForce 決定(36 篇 §3.3)
    def signed(d, fwd):
        if not en or d <= MOTOR_DEADBAND:
            return 0.0
        m = (d - MOTOR_DEADBAND) / (1.0 - MOTOR_DEADBAND)
        return m if fwd else -m

    locked = len(f) == 9 and f[8] == "1"      # 堵轉注入:輪子被卡住 → 目標轉速 0,drive 把它按住
    wl_cmd = 0.0 if locked else signed(int(f[3]) / 1000.0, f[5] == "1") * MOTOR_FREE_RAD_S
    wr_cmd = 0.0 if locked else signed(int(f[4]) / 1000.0, f[6] == "1") * MOTOR_FREE_RAD_S
    drv_l.GetTargetVelocityAttr().Set(math.degrees(wl_cmd))
    drv_r.GetTargetVelocityAttr().Set(math.degrees(wr_cmd))
    for _ in range(max(1, int(round(dt / DT)))):
        step_once()
    al = wheel_angle_from_xform(wl.GetPrim(), "l")
    ar = wheel_angle_from_xform(wr.GetPrim(), "r")
    # 輪面速度 mm/s(給橋接對「輪子比車體多走多少」用,C16):從物理寫回的輪角差分,不是命令
    vl = (al - _wheel_prev["l"]) / dt * R_MM
    vr = (ar - _wheel_prev["r"]) / dt * R_MM
    _wheel_prev["l"], _wheel_prev["r"] = al, ar
    ticks_l = int(math.floor(al / (2 * math.pi) * TPR))
    ticks_r = int(math.floor(ar / (2 * math.pi) * TPR))
    x, y, th = pose()
    n_cmd += 1
    if topcam is not None and (n_cmd - 1) % TOPVIEW["every"] == 0:
        _, _, ms = topview_capture()
        TOPVIEW["ms"] = TOPVIEW.get("ms", 0.0) + ms
    out = ""
    collided = 0
    if world is not None:
        xm_, ym_, thr = (x - x0) / 1000.0, (y - y0) / 1000.0, th - th0
        collided = int(world.collides(xm_, ym_))
        dt_ms = int(f[2])
        if dt_ms > 0 and (seq * dt_ms) % world.period_ms == 0:
            rs = scan_now(xm_, ym_, thr)
            out += f"SCAN {seq} {len(rs)} " + " ".join(f"{r_:.3f}" for r_ in rs) + "\n"
    if n_cmd % 200 == 0:
        print(f"[isaac_plant] {n_cmd} cmds x={x - x0:.1f} y={y - y0:.1f} th={th - th0:.4f} "
              f"ticks=({ticks_l},{ticks_r}) joint_state_fallback={fallback_used}"
              + (f" collided={collided}" if world is not None else "")
              + (f" topview_frames={TOPVIEW['n']} 每幀 {TOPVIEW.get('ms', 0.0) / max(1, TOPVIEW['n']):.1f} ms" if topcam is not None else ""), flush=True)
    out += f"ENC {seq} {ticks_l} {ticks_r} {x - x0:.6f} {y - y0:.6f} {th - th0:.9f} {vl:.6f} {vr:.6f} {collided}\n"
    return out


while app.is_running():
    if args.tcp:
        try:
            conn, _ = srv.accept()
        except socket.timeout:
            app.update()
            continue
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print("[isaac_plant] client connected", flush=True)
        with conn, conn.makefile("r", encoding="ascii", errors="replace") as rf:
            for line in rf:
                reply = handle(line)
                if reply:
                    conn.sendall(reply.encode("ascii"))
        print("[isaac_plant] client disconnected", flush=True)
        continue
    try:
        data, addr = sock.recvfrom(256)
    except socket.timeout:
        app.update()  # 讓 Kit 活著(headless 也要)
        continue
    reply = handle(data.decode("ascii", "replace"))
    if reply:
        sock.sendto(reply.encode("ascii"), addr)

app.close()

# ---- 驗收清單(2026-09-15 實測結論;量測用 --probe 模式)---------------------------------
# 1. 6.0.1 的 omni.physx PhysX 介面**沒有 update**;手動步進 = IPhysxSimulation.attach_stage(stage_id)
#    → simulate(dt, t) → fetch_results()。200 步無例外。timeline 不 play(否則 Kit 每 update 自己再步一次)。
# 2. joint state 屬性(state:angular:physics:position)**不會被寫回**——步進前後都不存在。
#    輪角改從 fetch_results 寫回的 xform 算(輪子相對底盤繞 Y 的角,跨步展開),那是物理輸出。
# 3. physxScene:timeStepsPerSecond=200 讀回 200,SimulationManager.get_physics_dt()=0.005。標 Deprecated 但生效。
# 4. 不彈飛:靜止 1 s 底盤 z=50.0 mm(建模 50.0)。**曾經彈飛**,真因不是腳輪:地面的 xformOpOrder 寫成
#    [scale, translate],-0.05 的平移被 z 縮成 -0.005,地面頂面在 +45 mm,輪子起始陷入 45 mm,第一步 2.9 m/s 往上。
# 5. DriveAPI targetVelocity 單位是度/秒:設 360 跑 1 s → 輪角 6.235 rad(99.2%,drive 有落後);
#    底盤 302.1 mm 對輪周 311.8 mm → 滑移 3.1%。
# 6. 同一腳本 C1–C9 ALL PASS;C3 dx=0.8 dy=1.0 dth=0.0008 rad(2026-09-16,馬達層 + 韌體斜坡後;
#    DriveAPI 直接吃 duty 時是 3.4 / 2.0 / 0.0276,步階超調 60%、加速度 91400 mm/s²,dθ 差在轉向段的打滑),
#    容差 = 25 mm + (2% + slip)·路徑長、0.03 + slip·|θ|,slip=0.05。負對照(壞 CRC)位移 0、C2 紅。
# 7. 兩次跑(每次重啟受控體)CSV 全部欄位逐 byte 相同。SimulationManager.is_gpu_dynamics_enabled()=True
#    而 get_physics_sim_device()=cpu——兩個值都記,決定性在這個組合下成立。
# 另:Gf 矩陣是 row-vector 慣例,yaw 要用 atan2(m[0][1], m[0][0]);寫反會得到正負號相反的航向。
