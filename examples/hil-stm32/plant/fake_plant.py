#!/usr/bin/env python3
"""UDP 版假受控體:與 bridge-rs/src/plant.rs 的 `Fake` 同一個模型、讀同一份 calib.json。

用途有兩個:
1. 驗證橋接的 UDP 受控體協定(Isaac Sim 的 receiver 就是實作這個協定)
2. 與 Rust 內建的 Fake 對照:同一份輸入,兩個實作的軌跡要一致

協定(一行一筆,ASCII,空白分隔;橋接等到同 seq 的回覆才推進下一步):
  橋接 → 受控體:CMD <seq> <dt_ms> <duty_l 0..1000> <duty_r> <fwd_l 0/1> <fwd_r> <en 0/1>
  受控體 → 橋接:ENC <seq> <ticks_l> <ticks_r> <x_mm> <y_mm> <th_rad> <vl_mm_s> <vr_mm_s>

用法:python3 fake_plant.py --bind 0.0.0.0:3700 --calib ../calib.json [--tcp]
--tcp:同一份協定改走 TCP(一行一筆),給 ssh -L 隧道用。
"""
import argparse
import json
import math
import socket
import sys


class FakePlant:
    """一階馬達(時間常數 tau)+ 精確差速運動學。編碼器 tick 由各輪累計行程取整。"""

    def __init__(self, calib: dict, tau_s: float = 0.050):
        self.circ_mm = 2 * math.pi * calib["wheel_radius_mm"]
        self.track = calib["track_mm"]
        self.tpr = calib["encoder_ticks_per_rev"]
        self.v_full = calib["wheel_speed_at_full_duty_mm_s"]
        self.tau = tau_s
        self.vl = self.vr = 0.0
        self.sl = self.sr = 0.0
        self.x = self.y = self.th = 0.0

    def step(self, dt: float, duty_l: float, duty_r: float, fwd_l: bool, fwd_r: bool, en: bool):
        if en:
            tl = duty_l * self.v_full * (1 if fwd_l else -1)
            tr = duty_r * self.v_full * (1 if fwd_r else -1)
        else:
            tl = tr = 0.0
        a = min(dt / self.tau, 1.0)
        self.vl += (tl - self.vl) * a
        self.vr += (tr - self.vr) * a
        dl, dr = self.vl * dt, self.vr * dt
        self.sl += dl
        self.sr += dr
        ds = (dl + dr) * 0.5
        dth = (dr - dl) / self.track
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
    ap.add_argument("--tau", type=float, default=0.050)
    ap.add_argument("--tcp", action="store_true")
    a = ap.parse_args()

    calib = json.load(open(a.calib, encoding="utf-8"))
    plant = FakePlant(calib, a.tau)
    host, port = a.bind.rsplit(":", 1)
    print(f"[fake_plant] listening {a.bind} {'tcp' if a.tcp else 'udp'} circ={plant.circ_mm:.3f}mm track={plant.track} tpr={plant.tpr} tau={a.tau}", flush=True)
    serve(plant, host, int(port), a.tcp)


def handle(plant: FakePlant, line: str, n: int) -> str | None:
    f = line.split()
    if len(f) != 8 or f[0] != "CMD":
        return None
    seq = int(f[1])
    dt = int(f[2]) / 1000.0
    plant.step(dt, int(f[3]) / 1000.0, int(f[4]) / 1000.0, f[5] == "1", f[6] == "1", f[7] == "1")
    if n % 1000 == 0:
        print(f"[fake_plant] {n} steps x={plant.x:.1f} y={plant.y:.1f} th={plant.th:.4f}", flush=True)
    return f"ENC {seq} {plant.ticks(plant.sl)} {plant.ticks(plant.sr)} {plant.x:.6f} {plant.y:.6f} {plant.th:.9f} {plant.vl:.6f} {plant.vr:.6f}\n"


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
                sock.sendto(reply.encode("ascii"), addr)
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
