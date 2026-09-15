#!/usr/bin/env python3
"""Isaac Sim 6.0.1 版受控體:實作 bridge-rs 的 UDP 受控體協定,由橋接 lockstep 步進。

⚠ 未在本 repo 環境驗證(本機無 GPU)。程式碼依官方文件與本 repo 01/15/31/32 篇的結論組合,
  哪些地方要實跑才能確認,寫在檔尾「驗收清單」。假受控體(fake_plant.py)已驗過同一份協定。

執行(在 Isaac Sim 安裝目錄):
    ./python.sh /path/to/isaac_plant.py --bind 0.0.0.0:3700 --calib /path/to/calib.json

協定(與 fake_plant.py 相同):
    橋接 → 受控體:CMD <seq> <dt_ms> <duty_l 0..1000> <duty_r> <fwd_l 0/1> <fwd_r> <en 0/1>
    受控體 → 橋接:ENC <seq> <ticks_l> <ticks_r> <x_mm> <y_mm> <th_rad> <vl_mm_s> <vr_mm_s>

每收到一筆 CMD 就把兩個輪關節的速度目標設好、物理走一步(dt_ms),再從關節位置算編碼器 tick、
從車體 prim 讀真值位姿回覆。物理步長綁在 calib 的 control_period_ms(5 ms),不是 Isaac 預設的 1/60 s。
"""
import argparse
import json
import math
import socket
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--bind", default="0.0.0.0:3700")
ap.add_argument("--calib", default="../calib.json")
ap.add_argument("--headless", type=int, default=1)
args = ap.parse_args()
calib = json.load(open(args.calib, encoding="utf-8"))

# ---- SimulationApp 必須在所有 omni.* / isaacsim.* / pxr import 之前(31 篇 §1:間接 import 也算) ----
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": bool(args.headless)})

import numpy as np  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402
import omni.usd  # noqa: E402
import omni.timeline  # noqa: E402
from omni.physx import get_physx_interface, get_physx_simulation_interface  # noqa: E402

# 6.0 起 isaacsim.core.api 搬到 isaacsim.core.experimental(01 篇 §3)。這支腳本刻意不依賴
# 任何一邊:場景用 pxr/UsdPhysics 直接建,步進用 omni.physx 的介面。要實跑才知道 6.0.1 的
# omni.physx 介面名稱有沒有再變——見檔尾驗收清單第 1 條。

R_MM = calib["wheel_radius_mm"]
TRACK_MM = calib["track_mm"]
TPR = calib["encoder_ticks_per_rev"]
V_FULL = calib["wheel_speed_at_full_duty_mm_s"]
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
ground.AddScaleOp().Set(Gf.Vec3f(50, 50, 0.1))
ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

# ---- 差速車:底盤 + 兩個驅動輪(revolute + angular drive)+ 一顆腳輪球 ----
r = R_MM / 1000.0
half_track = TRACK_MM / 2000.0
chassis = UsdGeom.Cube.Define(stage, Sdf.Path("/World/robot/chassis"))
chassis.CreateSizeAttr(1.0)
chassis.AddTranslateOp().Set(Gf.Vec3d(0, 0, r))
chassis.AddScaleOp().Set(Gf.Vec3f(0.30, 0.20, 0.06))
UsdPhysics.RigidBodyAPI.Apply(chassis.GetPrim())
UsdPhysics.CollisionAPI.Apply(chassis.GetPrim())
mass = UsdPhysics.MassAPI.Apply(chassis.GetPrim())
mass.CreateMassAttr(8.0)

def make_wheel(name, y):
    # 32 篇 §:UsdGeom.Cylinder 當碰撞體會出問題,輪子用 Sphere 近似(半徑 = 輪半徑)
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
    j.CreateLocalPos0Attr(Gf.Vec3f(0, y / 0.20, 0))  # body0 是縮放過的 cube,local pos 要除回 scale
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    drv.CreateTypeAttr("force")
    drv.CreateDampingAttr(50.0)       # 速度驅動:只給 damping,不給 stiffness
    drv.CreateStiffnessAttr(0.0)
    drv.CreateMaxForceAttr(20.0)
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
cj.CreateLocalPos0Attr(Gf.Vec3f(-0.12 / 0.30, 0, -0.5 * r / 0.06))
# ⚠ 32 篇:腳輪半徑只有驅動輪一半時在平地會被彈飛——那是三輪叉車型;這裡是球關節腳輪,要實跑確認

timeline = omni.timeline.get_timeline_interface()
timeline.play()
app.update()

physx = get_physx_interface()
physx_sim = get_physx_simulation_interface()

def step_once():
    # 手動步進一格物理,不渲染。31 篇 §5:world.step(render=False) 不 tick action graph,
    # 這支腳本不用 OmniGraph,所以無所謂;真要接 ROS 2 bridge 的節點才要注意。
    physx.update(DT, DT)
    physx_sim.fetch_results()

xform_cache = UsdGeom.XformCache()

def wheel_angle_rad(joint_prim):
    # 關節角度讀法在 6.0.1 要實跑確認;這裡用 PhysX 的 joint state 屬性(若沒被寫入就讀不到)
    st = joint_prim.GetAttribute("state:angular:physics:position")
    v = st.Get() if st and st.HasValue() else None
    return math.radians(v) if v is not None else None

def pose():
    xform_cache.Clear()
    m = xform_cache.GetLocalToWorldTransform(chassis.GetPrim())
    t = m.ExtractTranslation()
    rot = m.ExtractRotationMatrix()
    yaw = math.atan2(rot[1][0], rot[0][0])
    return t[0] * 1000.0, t[1] * 1000.0, yaw

host, port = args.bind.rsplit(":", 1)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((host, int(port)))
sock.settimeout(0.5)
print(f"[isaac_plant] listening {args.bind} dt={DT} r={r} track={TRACK_MM}", flush=True)

# 沒有 PhysX joint state 時的退路:用命令速度積分輪角(那就退化成純運動學,結果要標明)
ang_l = ang_r = 0.0
last_vl = last_vr = 0.0
x0, y0, th0 = pose()
while app.is_running():
    try:
        data, addr = sock.recvfrom(256)
    except socket.timeout:
        app.update()  # 讓 Kit 活著(headless 也要)
        continue
    f = data.decode("ascii", "replace").split()
    if len(f) != 8 or f[0] != "CMD":
        continue
    seq = int(f[1])
    dt = int(f[2]) / 1000.0
    en = f[7] == "1"
    vl = (int(f[3]) / 1000.0) * V_FULL * (1 if f[5] == "1" else -1) if en else 0.0
    vr = (int(f[4]) / 1000.0) * V_FULL * (1 if f[6] == "1" else -1) if en else 0.0
    # mm/s → rad/s → deg/s(DriveAPI 的角速度單位)
    drv_l.GetTargetVelocityAttr().Set(math.degrees(vl / R_MM))
    drv_r.GetTargetVelocityAttr().Set(math.degrees(vr / R_MM))
    for _ in range(max(1, int(round(dt / DT)))):
        step_once()
    al = wheel_angle_rad(jl.GetPrim())
    ar = wheel_angle_rad(jr.GetPrim())
    if al is None or ar is None:
        ang_l += vl / R_MM * dt
        ang_r += vr / R_MM * dt
        al, ar = ang_l, ang_r
    ticks_l = int(math.floor(al / (2 * math.pi) * TPR))
    ticks_r = int(math.floor(ar / (2 * math.pi) * TPR))
    x, y, th = pose()
    reply = f"ENC {seq} {ticks_l} {ticks_r} {x - x0:.6f} {y - y0:.6f} {th - th0:.9f} {vl:.6f} {vr:.6f}\n"
    sock.sendto(reply.encode("ascii"), addr)

app.close()

# ---- 驗收清單(每一條過了才能把篇首的「未驗證」拿掉)-------------------------------------
# 1. 6.0.1 上 `from omni.physx import get_physx_interface` 與 `physx.update(dt, dt)` 是否仍是手動步進的
#    正確介面;不是的話改用 isaacsim.core.experimental 的 SimulationManager 步進。
# 2. `state:angular:physics:position` 在 6.0.1 的 PhysX 是否會回寫關節角;否則 ticks 走的是退路的
#    純運動學積分(結果仍會 ALL PASS,但那不是物理——要在 log 標明)。
# 3. 物理步長 1/DT = 200 Hz 是否生效:`timeStepsPerSecond` 讀回、一步後的 wheel 角度對 targetVelocity×dt。
# 4. 輪子用 Sphere 近似 + 球關節腳輪在 6.0.1 PhysX 110 上會不會被彈飛(32 篇的腳輪半徑問題)。
# 5. DriveAPI targetVelocity 的單位是 度/秒:設 360 → 一秒後輪角 2π。
# 6. 與假受控體同一份腳本(0.5 s 起 300 mm/s 3 s → 600 mrad/s 1.5 s)跑一次,C1–C8 全綠;
#    odom 對真值的容差可能要放寬到接觸滑移的量級(32 篇實測滑移 2~3%)。
# 7. 兩次跑 CSV 是否逐 byte 相同:PhysX GPU dynamics 不保證,CPU 模式較可能;要讀回 enableGPUDynamics。
