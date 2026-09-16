#!/usr/bin/env python3
"""方形閉環:訂 /odom、發 /cmd_vel,靠里程計把車開成一個正方形,回到起點後停。

四條邊各走 side_m,四個角各轉 +90°,判斷「走夠了/轉夠了」全用 /odom——不是用計時。
所以它不在乎 Renode 跑幾倍實時(35 篇 §5.1):時鐘慢,只是這個節點等久一點。
跑完印一行 JSON:里程計的末端位姿、用了多少 odom 訊息、牆鐘。
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SquareClient(Node):
    def __init__(self):
        super().__init__("square_client")
        self.declare_parameter("side_m", 0.6)
        self.declare_parameter("v", 0.3)        # m/s
        self.declare_parameter("w", 0.6)        # rad/s
        self.declare_parameter("timeout_s", 120.0)
        # 韌體有斜坡(calib accel_limit / alpha_limit):命令歸零後車還要 v²/2a 才停,
        # 所以提前 d_brake 停、每段之間等 odom 的速度歸零再起下一段。
        # lag_s 是「命令歸零到車真的開始減速」的延遲:馬達層 τ(calib motor_tau_s 0.05)
        # + odom 一筆(20 ms)+ cmd 一筆(20 ms)量級;這段時間車還在等速走,煞車距離要多 v·lag
        self.declare_parameter("accel", 1.5)    # m/s²,對應 calib accel_limit_mm_s2
        self.declare_parameter("alpha", 4.0)    # rad/s²,對應 calib alpha_limit_mrad_s2
        self.declare_parameter("lag_s", 0.08)
        self.side = float(self.get_parameter("side_m").value)
        self.v = float(self.get_parameter("v").value)
        self.w = float(self.get_parameter("w").value)
        self.timeout = float(self.get_parameter("timeout_s").value)
        lag = float(self.get_parameter("lag_s").value)
        self.d_brake = self.v ** 2 / (2 * float(self.get_parameter("accel").value)) + self.v * lag
        self.th_brake = self.w ** 2 / (2 * float(self.get_parameter("alpha").value)) + self.w * lag
        self.settle_count = 0
        self.vel = (0.0, 0.0)

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.create_timer(0.05, self.tick)  # 20 Hz 發命令
        self.pose = None
        self.n_odom = 0
        self.leg = 0          # 0..7:偶數直走、奇數轉
        self.leg_start = None
        self.unwrapped = 0.0
        self.prev_yaw = None
        self.t0 = time.monotonic()
        self.done_at = None
        self.first_pose = None

    def on_odom(self, m: Odometry):
        p = m.pose.pose
        yaw = yaw_of(p.orientation)
        if self.prev_yaw is not None:
            d = yaw - self.prev_yaw
            d = (d + math.pi) % (2 * math.pi) - math.pi
            self.unwrapped += d
        self.prev_yaw = yaw
        self.pose = (p.position.x, p.position.y, self.unwrapped)
        self.vel = (m.twist.twist.linear.x, m.twist.twist.angular.z)
        if self.first_pose is None:
            self.first_pose = self.pose
        self.n_odom += 1

    def tick(self):
        cmd = Twist()
        if self.pose is None:
            self.pub.publish(cmd)
            if time.monotonic() - self.t0 > self.timeout:
                self.finish("no odom")
            return
        if self.done_at is not None:
            self.pub.publish(cmd)  # 停 1 s 讓韌體與橋接把最後的 odom 送完
            if time.monotonic() - self.done_at > 1.0:
                self.finish("ok")
            return
        if self.leg_start is None:
            # 段間停定:等 odom 的 v、w 連續 5 筆都接近 0,才把這一段的起點記下來
            if abs(self.vel[0]) < 0.01 and abs(self.vel[1]) < 0.02:
                self.settle_count += 1
            else:
                self.settle_count = 0
            if self.settle_count < 5:
                self.pub.publish(cmd)
                return
            self.settle_count = 0
            self.leg_start = self.pose
        x, y, th = self.pose
        x0, y0, th0 = self.leg_start
        if self.leg % 2 == 0:
            if math.hypot(x - x0, y - y0) >= self.side - self.d_brake:
                self.next_leg()
            else:
                cmd.linear.x = self.v
        else:
            if th - th0 >= math.pi / 2 - self.th_brake:
                self.next_leg()
            else:
                cmd.angular.z = self.w
        self.pub.publish(cmd)
        if time.monotonic() - self.t0 > self.timeout:
            self.finish("timeout at leg %d" % self.leg)

    def next_leg(self):
        self.get_logger().info("leg %d done at x=%.3f y=%.3f th=%.3f (%.1f s)" % (
            self.leg, self.pose[0], self.pose[1], self.pose[2], time.monotonic() - self.t0))
        self.leg += 1
        self.leg_start = None
        if self.leg >= 8:
            self.done_at = time.monotonic()

    def finish(self, status):
        x, y, th = self.pose if self.pose else (float("nan"),) * 3
        out = {"status": status, "legs_done": self.leg, "odom_msgs": self.n_odom,
               "end_x_m": round(x, 4), "end_y_m": round(y, 4), "end_yaw_rad": round(th, 4),
               "closure_err_m": round(math.hypot(x, y), 4) if self.pose else None,
               "yaw_err_rad": round(th - 2 * math.pi, 4) if self.pose else None,
               "wall_s": round(time.monotonic() - self.t0, 1)}
        print("[square] " + json.dumps(out), flush=True)
        raise SystemExit(0 if status == "ok" else 1)


def main():
    rclpy.init()
    node = SquareClient()
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = e.code
    else:
        code = 0
    node.destroy_node()
    rclpy.try_shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
