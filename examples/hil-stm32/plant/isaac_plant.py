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
ap.add_argument("--tcp", action="store_true")
ap.add_argument("--probe", action="store_true", help="不開 socket:跑固定命令量驗收清單 1/2/3/5/7 後離開")
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
cj.CreateLocalPos0Attr(Gf.Vec3f(-0.12, 0, -0.5 * r))
cj.CreateCollisionEnabledAttr(False)
# ⚠ 32 篇:腳輪半徑只有驅動輪一半時在平地會被彈飛——那是三輪叉車型;這裡是球關節腳輪,要實跑確認

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


if args.probe:
    probe()
    app.close()
    sys.exit(0)

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
    if len(f) != 8 or f[0] != "CMD":
        return None
    seq = int(f[1])
    dt = int(f[2]) / 1000.0
    en = f[7] == "1"
    tl = motor_target(int(f[3]) / 1000.0, f[5] == "1", en, MOTOR_DEADBAND, V_FULL)
    tr = motor_target(int(f[4]) / 1000.0, f[6] == "1", en, MOTOR_DEADBAND, V_FULL)
    motor_v[0] = motor_advance(motor_v[0], tl, dt, MOTOR_TAU, MOTOR_ACCEL_MAX)
    motor_v[1] = motor_advance(motor_v[1], tr, dt, MOTOR_TAU, MOTOR_ACCEL_MAX)
    vl, vr = motor_v
    # mm/s → rad/s → deg/s(DriveAPI 的角速度單位)
    drv_l.GetTargetVelocityAttr().Set(math.degrees(vl / R_MM))
    drv_r.GetTargetVelocityAttr().Set(math.degrees(vr / R_MM))
    for _ in range(max(1, int(round(dt / DT)))):
        step_once()
    al = wheel_angle_from_xform(wl.GetPrim(), "l")
    ar = wheel_angle_from_xform(wr.GetPrim(), "r")
    ticks_l = int(math.floor(al / (2 * math.pi) * TPR))
    ticks_r = int(math.floor(ar / (2 * math.pi) * TPR))
    x, y, th = pose()
    n_cmd += 1
    if n_cmd % 200 == 0:
        print(f"[isaac_plant] {n_cmd} cmds x={x - x0:.1f} y={y - y0:.1f} th={th - th0:.4f} "
              f"ticks=({ticks_l},{ticks_r}) joint_state_fallback={fallback_used}", flush=True)
    return f"ENC {seq} {ticks_l} {ticks_r} {x - x0:.6f} {y - y0:.6f} {th - th0:.9f} {vl:.6f} {vr:.6f}\n"


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
