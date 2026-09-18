#!/usr/bin/env python3
"""UDP 版假受控體:與 bridge-rs/src/plant.rs 的 `Fake` 同一個模型、讀同一份 calib.json。

用途有兩個:
1. 驗證橋接的 UDP 受控體協定(Isaac Sim 的 receiver 就是實作這個協定)
2. 與 Rust 內建的 Fake 對照:同一份輸入,兩個實作的軌跡要一致

協定(一行一筆,ASCII,空白分隔;橋接等到同 seq 的回覆才推進下一步):
  橋接 → 受控體:CMD <seq> <dt_ms> <duty_l 0..1000> <duty_r> <fwd_l 0/1> <fwd_r> <en 0/1>
  受控體 → 橋接:ENC <seq> <ticks_l> <ticks_r> <x_mm> <y_mm> <th_rad> <vl_mm_s> <vr_mm_s> [collided 0/1]
  受控體 → 橋接(有 --world 時每 period_ms 一筆,在 ENC 之前):SCAN <seq> <n> <r0 m> ... <r(n-1)>

用法:python3 fake_plant.py --bind 0.0.0.0:3700 --calib ../calib.json [--tcp]
--tcp:同一份協定改走 TCP(一行一筆),給 ssh -L 隧道用。
"""
import argparse
import pathlib
import json
import math
import socket
import sys


def motor_torque(duty: float, omega: float, stall_nm: float, free_rad_s: float, max_nm: float) -> float:
    """直流馬達的扭矩–轉速直線,PWM 當電壓比例(docs/hil/36 §3.3);duty 帶號。"""
    t = duty * stall_nm - stall_nm / free_rad_s * omega
    return max(-max_nm, min(max_nm, t))


def wheel_step(t0, b, tau_max, omega, v_wheel, dt, r, mu, n_force, i_w, v_ref):
    """一個輪子這一步(docs/hil/36 §3.3,與 bridge-rs/src/plant.rs 同一份):馬達扭矩 τ = t0 − b·ω 夾在 ±τ_max、
    摩擦 F = clamp(k·(ωr − v), ±μN);反電動勢與摩擦都用隱式解(顯式在 5 ms 步長下會震盪)。"""
    f_max = mu * n_force
    k = f_max / v_ref
    a = i_w / dt
    w1 = (a * omega + t0 + k * r * v_wheel) / (a + b + k * r * r)
    f1 = k * (w1 * r - v_wheel)
    if abs(f1) <= f_max:
        tau1 = t0 - b * w1
        if abs(tau1) <= tau_max:
            return f1, w1
        tau_c = tau_max if tau1 > 0 else -tau_max
        w2 = (a * omega + tau_c + k * r * v_wheel) / (a + k * r * r)
        f2 = k * (w2 * r - v_wheel)
        if abs(f2) <= f_max:
            return f2, w2
    f = f_max if (w1 * r - v_wheel) >= 0 else -f_max
    w3 = (a * omega + t0 - f * r) / (a + b)
    tau3 = t0 - b * w3
    if abs(tau3) > tau_max:
        w_new = omega + dt * ((tau_max if tau3 > 0 else -tau_max) - f * r) / i_w
    else:
        w_new = w3
    return f, w_new


def motor_target(duty: float, fwd: bool, enabled: bool, deadband: float, full: float) -> float:
    """死區 → 線性到滿 duty;與 Rust / Isaac 版逐字同公式"""
    if not enabled or duty <= deadband:
        return 0.0
    mag = (duty - deadband) / (1.0 - deadband) * full
    return mag if fwd else -mag


def motor_advance(v: float, target: float, dt: float, tau: float, accel_max: float) -> float:
    a = min(dt / tau, 1.0)
    dv = (target - v) * a
    if accel_max > 0:
        lim = accel_max * dt
        dv = max(-lim, min(lim, dv))
    return v + dv


class FakePlant:
    """一階馬達(時間常數 tau)+ 精確差速運動學。編碼器 tick 由各輪累計行程取整。"""

    def __init__(self, calib: dict, tau_s: float = None, world=None):
        self.world = world
        self.collided = False
        self.circ_mm = 2 * math.pi * calib["wheel_radius_mm"]
        self.track = calib["track_mm"]
        self.tpr = calib["encoder_ticks_per_rev"]
        self.v_full = calib["wheel_speed_at_full_duty_mm_s"]
        # 馬達層三個實作同一份公式(bridge-rs/src/plant.rs 的 motor_target / motor_advance)
        self.tau = tau_s if tau_s is not None else calib.get("motor_tau_s", 0.05)
        self.accel_max = calib.get("motor_accel_max_mm_s2", 0.0)
        self.deadband = calib.get("motor_deadband_duty", 0.0)
        self.vl = self.vr = 0.0
        self.sl = self.sr = 0.0
        self.x = self.y = self.th = 0.0
        # 扭矩模型(36 篇 §3.3):兩輪角速度、車體線速度與角速度
        self.r = calib["wheel_radius_mm"] / 1000.0
        self.stall = calib.get("motor_stall_torque_nm", 2.06)
        self.free_w = calib.get("motor_free_rad_s", 20.94)
        self.tau_max = calib.get("motor_torque_max_nm", 2.45)
        self.mu = calib.get("friction_mu", 0.508)
        self.mass = calib.get("robot_mass_kg", 9.2)
        self.i_w = 0.4 * calib.get("wheel_mass_kg", 0.5) * self.r ** 2
        self.j_body = calib.get("body_inertia_kg_m2", 0.11)
        self.v_ref = calib.get("friction_v_ref_m_s", 0.02)
        self.wl = self.wr = 0.0
        self.v_body = self.w_body = 0.0

    def step(self, dt: float, duty_l: float, duty_r: float, fwd_l: bool, fwd_r: bool, en: bool, locked: bool = False):
        """扭矩模型(docs/hil/36 §3.3,與 bridge-rs/src/plant.rs、isaac_plant.py 同一份公式):
        duty → 馬達扭矩 → 輪緣力(夾在抓地力 μN)→ 輪子與車體各自積分。打滑或卡住是算出來的。"""
        def signed(d, fwd):
            if not en or d <= self.deadband:
                return 0.0
            m = (d - self.deadband) / (1.0 - self.deadband)
            return m if fwd else -m

        if locked:
            # 輪子被卡住:編碼器不動、車體不動(同 bridge-rs/src/plant.rs)
            self.wl = self.wr = 0.0
            self.vl = self.vr = 0.0
            self.v_body = self.w_body = 0.0
            self.collided = False
            if self.world is not None and self.world.collides(self.x / 1000.0, self.y / 1000.0):
                self.collided = True
            return
        t0_l = signed(duty_l, fwd_l) * self.stall
        t0_r = signed(duty_r, fwd_r) * self.stall
        b_emf = self.stall / self.free_w
        self.collided = False
        half = self.track / 2000.0
        n_force = self.mass * 9.81 / 2.0
        f_l, self.wl = wheel_step(t0_l, b_emf, self.tau_max, self.wl, self.v_body - self.w_body * half, dt,
                                  self.r, self.mu, n_force, self.i_w, self.v_ref)
        f_r, self.wr = wheel_step(t0_r, b_emf, self.tau_max, self.wr, self.v_body + self.w_body * half, dt,
                                  self.r, self.mu, n_force, self.i_w, self.v_ref)
        # 碰撞判定用「這一步的力算出來的新車速」試算位置,不是更新前的車速:用舊的會鎖死
        # (停在障礙物上時 v_body = 0 → 試算位置就是現在的位置 → 永遠碰撞 → 連倒車都退不開;
        #  docs/hil/38 §6.5)。與 bridge-rs/src/plant.rs 同一份公式
        v_try = self.v_body + (f_l + f_r) / self.mass * dt
        w_try = self.w_body + (f_r - f_l) * half / self.j_body * dt
        if self.world is not None:
            ds_m = v_try * dt
            if self.world.collides(self.x / 1000.0 + ds_m * math.cos(self.th), self.y / 1000.0 + ds_m * math.sin(self.th)):
                self.collided = True
        if self.collided:
            self.v_body = 0.0
            self.w_body = 0.0
        else:
            self.v_body = v_try
            self.w_body = w_try
        self.sl += self.wl * self.r * 1000.0 * dt
        self.sr += self.wr * self.r * 1000.0 * dt
        self.vl = self.wl * self.r * 1000.0
        self.vr = self.wr * self.r * 1000.0
        ds = self.v_body * dt * 1000.0
        dth = self.w_body * dt
        th_mid = self.th + dth * 0.5
        self.x += ds * math.cos(th_mid)
        self.y += ds * math.sin(th_mid)
        self.th += dth

    def ticks(self, s_mm: float) -> int:
        t = math.floor(s_mm / self.circ_mm * self.tpr)
        # 與韌體、Rust 端一樣:i32 繞回
        return ((int(t) + 2**31) % 2**32) - 2**31


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bind", default="0.0.0.0:3700")
    ap.add_argument("--calib", default="../calib.json")
    ap.add_argument("--tau", type=float, default=None, help="覆蓋 calib 的 motor_tau_s")
    ap.add_argument("--tcp", action="store_true")
    ap.add_argument("--world", default=None, help="world.json:有給就算假雷射與碰撞")
    a = ap.parse_args()

    calib = json.load(open(a.calib, encoding="utf-8"))
    world = None
    if a.world:
        sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
        from world import World
        world = World(a.world)
    plant = FakePlant(calib, a.tau, world)
    host, port = a.bind.rsplit(":", 1)
    print(f"[fake_plant] listening {a.bind} {'tcp' if a.tcp else 'udp'} circ={plant.circ_mm:.3f}mm track={plant.track} tpr={plant.tpr} tau={a.tau}", flush=True)
    serve(plant, host, int(port), a.tcp)


def handle(plant: FakePlant, line: str, n: int) -> str | None:
    f = line.split()
    if len(f) not in (8, 9) or f[0] != "CMD":
        return None
    seq = int(f[1])
    dt = int(f[2]) / 1000.0
    plant.step(dt, int(f[3]) / 1000.0, int(f[4]) / 1000.0, f[5] == "1", f[6] == "1", f[7] == "1", len(f) == 9 and f[8] == "1")
    if n % 1000 == 0:
        print(f"[fake_plant] {n} steps x={plant.x:.1f} y={plant.y:.1f} th={plant.th:.4f}", flush=True)
    out = ""
    if plant.world is not None and int(f[2]) > 0 and (seq * int(f[2])) % plant.world.period_ms == 0:
        rs = plant.world.scan(plant.x / 1000.0, plant.y / 1000.0, plant.th)
        out += f"SCAN {seq} {len(rs)} " + " ".join(f"{r:.3f}" for r in rs) + "\n"
    out += f"ENC {seq} {plant.ticks(plant.sl)} {plant.ticks(plant.sr)} {plant.x:.6f} {plant.y:.6f} {plant.th:.9f} {plant.vl:.6f} {plant.vr:.6f} {int(plant.collided)}\n"
    return out


def serve(plant: FakePlant, host: str, port: int, tcp: bool) -> None:
    n = 0
    if not tcp:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((host, port))
        while True:
            data, addr = sock.recvfrom(256)
            n += 1
            reply = handle(plant, data.decode("ascii", "replace"), n)
            if reply:
                for ln in reply.splitlines(keepends=True):   # SCAN 與 ENC 各一個 datagram
                    sock.sendto(ln.encode("ascii"), addr)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    while True:
        conn, _ = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        with conn, conn.makefile("r", encoding="ascii", errors="replace") as rf:
            for line in rf:
                n += 1
                reply = handle(plant, line, n)
                if reply:
                    conn.sendall(reply.encode("ascii"))


if __name__ == "__main__":
    sys.exit(main())
