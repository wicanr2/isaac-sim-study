# ScriptNode 範例:UDP 收 pose → 直接設定 prim 的 translate / orient
#
# 用法:在場景中建立 OmniGraph Script Node,把本檔內容貼入(或以檔案指定)。
# 外部程式對 UDP_PORT 送 JSON:{"x":1.0,"y":2.0,"z":0.0,"roll":0,"pitch":0,"yaw":90}
# 或文字格式:pose 1.0 2.0 0.0 0 0 90
#
# 注意:這是「非物理」控制(直改 xform 瞬移)。目標 prim 不可同時由
# PhysX articulation 驅動,否則模擬 view 會失效(見 docs/04-physics-world)。
#
# ScriptNode 執行環境會提供 `og`(omni.graph.core)名稱,故本檔未 import og。

import json
import math
import socket
import traceback

import omni.usd
from pxr import UsdGeom, Gf

# =========================
# 使用者設定
# =========================
PRIM_PATH = "/World/MyRobot"     # 要控制的 prim
UDP_HOST = "127.0.0.1"
UDP_PORT = 15001
MODEL_YAW_OFFSET_DEG = 0.0       # yaw=0 時車頭方向不對,可試 90 / -90 / 180


def _quat_from_rpy_deg(roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(float(roll_deg))
    pitch = math.radians(float(pitch_deg))
    yaw = math.radians(float(yaw_deg) + MODEL_YAW_OFFSET_DEG)
    qx = Gf.Quatf(math.cos(roll * 0.5), Gf.Vec3f(math.sin(roll * 0.5), 0.0, 0.0))
    qy = Gf.Quatf(math.cos(pitch * 0.5), Gf.Vec3f(0.0, math.sin(pitch * 0.5), 0.0))
    qz = Gf.Quatf(math.cos(yaw * 0.5), Gf.Vec3f(0.0, 0.0, math.sin(yaw * 0.5)))
    return qz * qy * qx


def _get_or_add_translate_op(xform):
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            return op
    return xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)


def _get_or_add_orient_op(xform):
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            return op
    return xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat)


def _set_pose(prim_path, x, y, z, roll_deg, pitch_deg, yaw_deg, db=None):
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        if db:
            db.log_warning("[UDP-POSE] Stage is None")
        return False
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        if db:
            db.log_warning(f"[UDP-POSE] Prim not found: {prim_path}")
        return False
    xform = UsdGeom.Xformable(prim)
    _get_or_add_translate_op(xform).Set(Gf.Vec3d(float(x), float(y), float(z)))
    _get_or_add_orient_op(xform).Set(_quat_from_rpy_deg(roll_deg, pitch_deg, yaw_deg))
    return True


def _parse_packet(data):
    text = data.decode("utf-8").strip()
    if text.startswith("{"):
        msg = json.loads(text)
        return {
            "x": float(msg["x"]),
            "y": float(msg["y"]),
            "z": float(msg.get("z", 0.0)),
            "roll": float(msg.get("roll", 0.0)),
            "pitch": float(msg.get("pitch", 0.0)),
            "yaw": float(msg.get("yaw", 0.0)),
        }
    parts = text.split()
    if len(parts) >= 7 and parts[0].lower() == "pose":
        return {
            "x": float(parts[1]), "y": float(parts[2]), "z": float(parts[3]),
            "roll": float(parts[4]), "pitch": float(parts[5]), "yaw": float(parts[6]),
        }
    raise ValueError(f"Unknown packet format: {text}")


# =========================
# Script Node 生命週期回呼
# =========================
def setup(db: og.Database):
    try:
        state = db.per_instance_state
        old_sock = getattr(state, "sock", None)
        if old_sock is not None:
            try:
                old_sock.close()
            except Exception:
                pass
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((UDP_HOST, UDP_PORT))
        state.sock = sock
        state.latest_pose = None
        print(f"[UDP-POSE] listening on {UDP_HOST}:{UDP_PORT} -> {PRIM_PATH}")
    except Exception as e:
        db.log_error(f"[UDP-POSE] setup failed: {e}")
        traceback.print_exc()


def cleanup(db: og.Database):
    try:
        state = db.per_instance_state
        sock = getattr(state, "sock", None)
        if sock is not None:
            sock.close()
        state.sock = None
        print("[UDP-POSE] socket closed")
    except Exception as e:
        db.log_warning(f"[UDP-POSE] cleanup failed: {e}")


def compute(db: og.Database):
    try:
        state = db.per_instance_state
        sock = getattr(state, "sock", None)
        if sock is None:
            db.log_warning("[UDP-POSE] socket is not initialized")
            return True

        # 非阻塞排空 UDP buffer,只保留最後一筆(即時控制語意:套用最新狀態)
        new_pose = None
        while True:
            try:
                data, _addr = sock.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                new_pose = _parse_packet(data)
                state.latest_pose = new_pose
            except Exception as e:
                db.log_warning(f"[UDP-POSE] bad packet: {e}")

        # 本次 tick 沒有新封包就什麼都不做,不重複套用舊 pose
        if new_pose is None:
            return True

        _set_pose(
            PRIM_PATH,
            new_pose["x"], new_pose["y"], new_pose.get("z", 0.0),
            new_pose.get("roll", 0.0), new_pose.get("pitch", 0.0), new_pose.get("yaw", 0.0),
            db=db,
        )
        return True
    except Exception as e:
        db.log_error(f"[UDP-POSE] compute failed: {e}")
        traceback.print_exc()
        return False
