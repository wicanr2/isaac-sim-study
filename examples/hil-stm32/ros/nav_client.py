#!/usr/bin/env python3
"""Nav2 in the loop 的上位:等 Nav2 起來 → NavigateToPose 到 world.json 的 goal → 印結果一行。
不用 AMCL(map→odom 是靜態 identity),所以等的是 bt_navigator 不是 amcl。"""
import json
import math
import pathlib
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry


def main():
    rclpy.init()
    world_path = sys.argv[1] if len(sys.argv) > 1 else str(pathlib.Path(__file__).resolve().parent.parent / "world.json")
    w = json.load(open(world_path, encoding="utf-8"))
    g = w["goal"]
    nav = BasicNavigator()
    last = {"x": None}
    nav.create_subscription(Odometry, "odom", lambda m: last.__setitem__("x", m), 10)
    # localizer='robot_localization' 讓 waitUntilNav2Active 跳過等 amcl(我們沒有 amcl);它只等 bt_navigator
    nav.waitUntilNav2Active(navigator="bt_navigator", localizer="robot_localization")
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = nav.get_clock().now().to_msg()
    goal.pose.position.x = float(g["x"])
    goal.pose.position.y = float(g["y"])
    goal.pose.orientation.z = math.sin(float(g["yaw"]) / 2)
    goal.pose.orientation.w = math.cos(float(g["yaw"]) / 2)
    t0 = time.monotonic()
    # 上位「不知道底盤重啟過」的行為:goal 失敗就重送一次(retries 參數,預設 1)。
    # driver 有鎖住時重送也走不了(cmd_vel 被壓成 0);沒鎖住(--negative no-latch)就會從歸零的 odom 再開走——C12 紅
    retries = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    attempts = 0
    while True:
        attempts += 1
        nav.goToPose(goal)
        n_fb = 0
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb is not None:
                n_fb += 1
                if n_fb % 50 == 0:
                    nav.get_logger().info("distance_remaining=%.2f recoveries=%d nav_time=%.1f" % (
                        fb.distance_remaining, fb.number_of_recoveries, fb.navigation_time.sec + fb.navigation_time.nanosec / 1e9))
            time.sleep(0.1)
            if time.monotonic() - t0 > 240:
                nav.cancelTask()
                break
        res = nav.getResult()
        if res == TaskResult.FAILED and attempts <= retries:
            nav.get_logger().warn("goal failed, retry %d/%d in 2 s" % (attempts, retries))
            time.sleep(2.0)
            continue
        break
    o = last["x"]
    x = o.pose.pose.position.x if o else float("nan")
    y = o.pose.pose.position.y if o else float("nan")
    q = o.pose.pose.orientation if o else None
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) if q else float("nan")
    dist = math.hypot(x - g["x"], y - g["y"])
    print('[nav] {"result": "%s", "attempts": %d, "goal": [%g, %g], "odom_end": [%.3f, %.3f, %.3f], "dist_to_goal_m": %.3f, "wall_s": %.1f}' % (
        {TaskResult.SUCCEEDED: "succeeded", TaskResult.CANCELED: "canceled", TaskResult.FAILED: "failed"}.get(res, str(res)),
        attempts, g["x"], g["y"], x, y, yaw, dist, time.monotonic() - t0), flush=True)
    nav.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
