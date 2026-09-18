#!/usr/bin/env python3
"""訂 /scan 與 /hil/safety_flags 幾秒,印統計然後結束:驗假雷射那條路(受控體 → 橋接 3801 → driver → /scan)通不通。"""
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt16


class ScanCheck(Node):
    def __init__(self):
        super().__init__("scan_check")
        self.declare_parameter("seconds", 8.0)
        self.n = 0
        self.last = None
        self.flags = set()
        self.create_subscription(LaserScan, "scan", self.on_scan, 10)
        self.create_subscription(UInt16, "hil/safety_flags", lambda m: self.flags.add(m.data), 10)
        self.create_timer(float(self.get_parameter("seconds").value), self.done)

    def on_scan(self, m):
        self.n += 1
        self.last = m

    def done(self):
        m = self.last
        if m is None:
            print('[scan_check] {"status": "no_scan"}', flush=True)
        else:
            r = list(m.ranges)
            # 第 0 束朝 −π(車尾)、第 n/2 束朝車頭
            head = r[len(r) // 2]
            print('[scan_check] {"status": "ok", "scans": %d, "beams": %d, "angle_min": %.4f, "inc": %.5f, "range_min": %.3f, "range_max": %.3f, '
                  '"min_range": %.3f, "max_range": %.3f, "ahead": %.3f, "behind": %.3f, "frame": "%s", "flags_seen": %s}' % (
                      self.n, len(r), m.angle_min, m.angle_increment, m.range_min, m.range_max, min(r), max(r), head, r[0],
                      m.header.frame_id, sorted(self.flags)), flush=True)
        raise SystemExit(0)


def main():
    rclpy.init()
    node = ScanCheck()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
