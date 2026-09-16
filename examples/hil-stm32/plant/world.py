"""假雷射與碰撞用的世界(world.json):矩形房間 + 軸對齊方塊。與 bridge-rs/src/world.rs 同一份公式;
tools/gen_map.py 也用它產 Nav2 的地圖。只用標準庫。"""
import json
import math


class World:
    def __init__(self, path: str):
        w = json.load(open(path, encoding="utf-8"))
        r = w["room"]
        self.x_min, self.x_max, self.y_min, self.y_max = r["x_min"], r["x_max"], r["y_min"], r["y_max"]
        self.boxes = [(b["cx"], b["cy"], b["w"], b["h"]) for b in w.get("boxes", [])]
        self.robot_radius = w["robot_radius_m"]
        lz = w["laser"]
        self.beams, self.range_min, self.range_max, self.period_ms = lz["beams"], lz["range_min_m"], lz["range_max_m"], lz["period_ms"]
        self.goal = (w["goal"]["x"], w["goal"]["y"], w["goal"]["yaw"])
        self.segments = [
            (self.x_min, self.y_min, self.x_max, self.y_min), (self.x_max, self.y_min, self.x_max, self.y_max),
            (self.x_max, self.y_max, self.x_min, self.y_max), (self.x_min, self.y_max, self.x_min, self.y_min)]
        for cx, cy, bw, bh in self.boxes:
            x0, y0, x1, y1 = cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2
            self.segments += [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]

    def scan(self, x: float, y: float, th: float) -> list[float]:
        """第 0 束朝 th − π,逆時針(LaserScan 的 angle_min = −π);沒打到 = range_max。"""
        out = []
        for i in range(self.beams):
            a = th - math.pi + 2 * math.pi * i / self.beams
            dx, dy = math.cos(a), math.sin(a)
            best = self.range_max
            for x1, y1, x2, y2 in self.segments:
                rx, ry = x2 - x1, y2 - y1
                den = dx * ry - dy * rx
                if abs(den) < 1e-12:
                    continue
                t = ((x1 - x) * ry - (y1 - y) * rx) / den
                u = ((x1 - x) * dy - (y1 - y) * dx) / den
                if t >= 0 and 0 <= u <= 1 and t < best:
                    best = t
            out.append(max(best, self.range_min))
        return out

    def collides(self, x: float, y: float) -> bool:
        r = self.robot_radius
        if x - r < self.x_min or x + r > self.x_max or y - r < self.y_min or y + r > self.y_max:
            return True
        for cx, cy, bw, bh in self.boxes:
            ddx = max(abs(x - cx) - bw / 2, 0.0)
            ddy = max(abs(y - cy) - bh / 2, 0.0)
            if ddx * ddx + ddy * ddy < r * r:
                return True
        return False
