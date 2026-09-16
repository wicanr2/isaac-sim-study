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
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import hilproto  # noqa: E402


class HilBaseDriver(Node):
    def __init__(self):
        super().__init__("hil_base_driver")
        self.declare_parameter("bridge", "127.0.0.1:3800")
        self.declare_parameter("calib", str(pathlib.Path(__file__).resolve().parent.parent / "calib.json"))
        self.declare_parameter("cmd_rate_hz", 50.0)
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

        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.pub_tf = self.create_publisher(TFMessage, "/tf", 10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        rate = float(self.get_parameter("cmd_rate_hz").value)
        self.create_timer(1.0 / rate, self.tick_tx)
        self.create_timer(0.005, self.tick_rx)
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

    def tick_tx(self):
        if not self.ensure_connected():
            return
        try:
            self.sock.sendall(hilproto.cmd_vel(*self.cmd))
            self.n_sent += 1
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
        self.get_logger().info("cmd_vel msgs=%d frames sent=%d odom frames=%d bad_crc=%d connected=%s" % (
            self.n_cmd_msgs, self.n_sent, self.n_odom, self.parser.bad_crc, self.sock is not None))


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
