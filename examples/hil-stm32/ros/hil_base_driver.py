#!/usr/bin/env python3
"""ROS 2 Jazzy 上位節點:/cmd_vel → UART 框包 → 橋接;橋接送回的 odom 框包 → /odom + /tf。

在 docs/hil/35 篇的拓撲裡它是「上位」:只認 cmd_vel 與 odom,對橋接講的是韌體那份序列協定
(ros/hilproto.py),跟橋接內建腳本送的 byte 一模一樣。橋接開 `--upper tcp-listen:0.0.0.0:3800`,
這個節點連過去;它也可以直接連 Renode 的 socket terminal(3456),那時橋接完全不在上位路徑上。

    ros2 run 不需要:python3 hil_base_driver.py --ros-args -p bridge:=127.0.0.1:3800
"""
import json
import math
import pathlib
import socket
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger
from action_msgs.srv import CancelGoal
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import hilproto  # noqa: E402


class HilBaseDriver(Node):
    def __init__(self):
        super().__init__("hil_base_driver")
        self.declare_parameter("bridge", "127.0.0.1:3800")
        self.declare_parameter("calib", str(pathlib.Path(__file__).resolve().parent.parent / "calib.json"))
        self.declare_parameter("cmd_rate_hz", 50.0)
        self.declare_parameter("heartbeat_hz", 10.0)   # PING;韌體 300 ms 沒收到就降速到 0
        # 假雷射:橋接 3801 每筆一行 `SCAN <seq> <n> r...`(受控體算的,不經 MCU——真車的雷射也是接上位不是接底盤板);
        # 空字串 = 不接。world.json 給射程與束數之外的參數(range_max);laser_frame 掛在 base_link 原點
        self.declare_parameter("scan", "127.0.0.1:3801")
        self.declare_parameter("world", str(pathlib.Path(__file__).resolve().parent.parent / "world.json"))
        self.declare_parameter("laser_frame", "laser")
        # 上位對安全旗標的反應(政策在上位,不在韌體):WDT_RESET 或 STALL 亮起 → 鎖住——cmd_vel 一律送 0、
        # 取消當前的 Nav2 goal(navigate_to_pose 的 cancel_goal 服務,goal_id 全 0 = 全部取消),
        # 直到操作者呼叫 /hil/fault_ack。fault_latch=false 是負對照:什麼都不做,重啟後車又走。
        self.declare_parameter("fault_latch", True)
        self.declare_parameter("cancel_service", "/navigate_to_pose/_action/cancel_goal")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        calib = json.loads(pathlib.Path(self.get_parameter("calib").value).read_text(encoding="utf-8"))
        self.track_m = calib["track_mm"] / 1000.0
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.sock = None
        self.parser = hilproto.Parser()
        self.cmd = (0, 0)  # (v mm/s, w mrad/s),最後一次 /cmd_vel
        self.n_cmd_msgs = 0
        self.n_odom = 0
        self.n_sent = 0
        self.n_ping = 0
        self.last_flags = None

        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.pub_tf = self.create_publisher(TFMessage, "/tf", 10)
        # 韌體的安全旗標原樣往上送(ENABLED/ESTOP/CMD_STALE/DRV_FAULT/BUMPER/STALL/HB_LOST/WDT_RESET),上位看得到為什麼停
        self.pub_flags = self.create_publisher(UInt8, "hil/safety_flags", 10)
        self.scan_sock = None
        self.scan_buf = b""
        self.n_scan = 0
        self.laser_frame = self.get_parameter("laser_frame").value
        self.range_max = 5.0
        try:
            self.range_max = float(json.loads(pathlib.Path(self.get_parameter("world").value).read_text(encoding="utf-8"))["laser"]["range_max_m"])
        except (OSError, KeyError, ValueError):
            pass
        self.pub_scan = self.create_publisher(LaserScan, "scan", 10)
        self.fault_latched = False
        self.n_latch = 0
        self.cancel_cli = self.create_client(CancelGoal, self.get_parameter("cancel_service").value)
        self.create_service(Trigger, "hil/fault_ack", self.on_fault_ack)
        self.pub_tf_static = self.create_publisher(TFMessage, "/tf_static", rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
        self.publish_static_tf()
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        rate = float(self.get_parameter("cmd_rate_hz").value)
        self.create_timer(1.0 / rate, self.tick_tx)
        self.create_timer(1.0 / float(self.get_parameter("heartbeat_hz").value), self.tick_ping)
        self.create_timer(0.005, self.tick_rx)
        self.create_timer(0.010, self.tick_scan)
        self.create_timer(5.0, self.report)
        self.get_logger().info("bridge=%s track=%.3f m cmd %.0f Hz" % (self.get_parameter("bridge").value, self.track_m, rate))

    # --- 連線 ---
    def ensure_connected(self):
        if self.sock is not None:
            return True
        host, port = self.get_parameter("bridge").value.rsplit(":", 1)
        try:
            s = socket.create_connection((host, int(port)), timeout=1.0)
        except OSError:
            return False
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.setblocking(False)
        self.sock = s
        self.get_logger().info("connected to bridge")
        return True

    def drop(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        self.get_logger().warning("bridge connection lost")

    # --- /cmd_vel → 框包 ---
    def on_cmd_vel(self, msg: Twist):
        self.cmd = (int(round(msg.linear.x * 1000.0)), int(round(msg.angular.z * 1000.0)))
        self.n_cmd_msgs += 1

    # --- 安全旗標的反應 ---
    def on_flags(self, flags: int):
        latch_bits = hilproto.FLAG_WDT_RESET | hilproto.FLAG_STALL
        if flags & latch_bits and not self.fault_latched and bool(self.get_parameter("fault_latch").value):
            self.fault_latched = True
            self.n_latch += 1
            self.get_logger().warn("fault latched: flags 0x%02x %s -> cmd_vel=0, cancel goal, wait /hil/fault_ack" % (
                flags, hilproto.flag_names(flags)))
            if self.cancel_cli.service_is_ready():
                self.cancel_cli.call_async(CancelGoal.Request())   # goal_id 全 0 + stamp 0 = 取消全部
            else:
                self.get_logger().warn("cancel service not ready: %s" % self.get_parameter("cancel_service").value)

    def on_fault_ack(self, req, resp):
        was = self.fault_latched
        self.fault_latched = False
        resp.success = True
        resp.message = "cleared" if was else "not latched"
        self.get_logger().info("fault ack: %s" % resp.message)
        return resp

    def tick_tx(self):
        if not self.ensure_connected():
            return
        try:
            cmd = (0, 0) if self.fault_latched else self.cmd
            self.sock.sendall(hilproto.cmd_vel(*cmd))
            self.n_sent += 1
        except OSError:
            self.drop()

    # --- 假雷射:3801 的文字行 → /scan ---
    def publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.laser_frame
        t.transform.rotation.w = 1.0
        self.pub_tf_static.publish(TFMessage(transforms=[t]))

    def tick_scan(self):
        addr = self.get_parameter("scan").value
        if not addr:
            return
        if self.scan_sock is None:
            host, port = addr.rsplit(":", 1)
            try:
                s = socket.create_connection((host, int(port)), timeout=0.5)
                s.setblocking(False)
                self.scan_sock = s
            except OSError:
                return
        try:
            data = self.scan_sock.recv(65536)
        except BlockingIOError:
            return
        except OSError:
            self.scan_sock = None
            return
        if not data:
            self.scan_sock = None
            return
        self.scan_buf += data
        while b"\n" in self.scan_buf:
            line, self.scan_buf = self.scan_buf.split(b"\n", 1)
            f = line.split()
            if len(f) < 3 or f[0] != b"SCAN":
                continue
            n = int(f[2])
            ranges = [float(x) for x in f[3:3 + n]]
            if len(ranges) != n:
                continue
            m = LaserScan()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self.laser_frame
            m.angle_min = -math.pi
            m.angle_max = math.pi - 2 * math.pi / n
            m.angle_increment = 2 * math.pi / n
            m.time_increment = 0.0
            m.scan_time = 0.1
            m.range_min = 0.05
            m.range_max = self.range_max
            m.ranges = ranges
            self.pub_scan.publish(m)
            self.n_scan += 1

    def tick_ping(self):
        if self.sock is None:
            return
        try:
            self.sock.sendall(hilproto.ping())
            self.n_ping += 1
        except OSError:
            self.drop()

    # --- 框包 → /odom + /tf ---
    def tick_rx(self):
        if self.sock is None:
            return
        try:
            data = self.sock.recv(4096)
        except BlockingIOError:
            return
        except OSError:
            self.drop()
            return
        if not data:
            self.drop()
            return
        for ty, payload in self.parser.feed(data):
            if ty == hilproto.MSG_ODOM:
                o = hilproto.parse_odom(payload)
                if o is not None:
                    self.publish_odom(o)
                    self.pub_flags.publish(UInt8(data=o["flags"]))
                    self.on_flags(o["flags"])
                    if o["flags"] != self.last_flags:
                        self.get_logger().info("flags 0x%02x %s" % (o["flags"], hilproto.flag_names(o["flags"])))
                        self.last_flags = o["flags"]

    def publish_odom(self, o):
        now = self.get_clock().now().to_msg()
        x, y, th = o["x_mm"] / 1000.0, o["y_mm"] / 1000.0, o["th_mrad"] / 1000.0
        qz, qw = math.sin(th / 2.0), math.cos(th / 2.0)
        vl, vr = o["vl_mm_s"] / 1000.0, o["vr_mm_s"] / 1000.0

        m = Odometry()
        m.header.stamp = now
        m.header.frame_id = self.odom_frame
        m.child_frame_id = self.base_frame
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = qz
        m.pose.pose.orientation.w = qw
        m.twist.twist.linear.x = (vl + vr) / 2.0
        m.twist.twist.angular.z = (vr - vl) / self.track_m
        self.pub_odom.publish(m)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.pub_tf.publish(TFMessage(transforms=[t]))
        self.n_odom += 1
        self.last = o

    def report(self):
        self.get_logger().info("cmd_vel msgs=%d frames sent=%d ping=%d odom frames=%d scans=%d bad_crc=%d connected=%s latched=%s" % (
            self.n_cmd_msgs, self.n_sent, self.n_ping, self.n_odom, self.n_scan, self.parser.bad_crc, self.sock is not None, self.fault_latched))


def main():
    rclpy.init()
    node = HilBaseDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
