#!/usr/bin/env python3
"""Nav2 in the loop 的上位:等 Nav2 起來 → NavigateToPose 到 world.json 的 goal → 印結果一行。
預設不用 AMCL(map→odom 是靜態 identity),所以等的是 bt_navigator 不是 amcl。

重定位(argv[4] = 1,GOAL 6 C-2):底盤重啟(WDT_RESET)→ driver 鎖住、取消 goal 之後——
  amcl:用重啟前最後一個 AMCL 位姿發 /initialpose(σ 0.5 m / 0.5 rad),等協方差收斂(σxy ≤ 0.05 m、σyaw ≤ 0.05 rad);
        車是停的:Nav2 AMCL 只在里程計變化嚴格大於 update_min_d / update_min_a 時才更新(設 0 也一樣),
        所以等待期間 5 Hz 呼叫 request_nomotion_update 強制更新;
        地圖 180° 對稱,不做全域定位(38 篇 §6.3)
  static:不重定位,等 3 s(負對照 static-map-odom)
然後呼叫 /hil/fault_ack 解鎖、重送 goal。"""
import json
import math
import pathlib
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt8
from std_srvs.srv import Empty, Trigger

FLAG_WDT_RESET = 1 << 7
RELOC_SIGMA_XY = 0.05     # m:收斂門檻(寫死,38 篇 §6.3)
RELOC_SIGMA_YAW = 0.05    # rad
RELOC_TIMEOUT_S = 60.0


def main():
    rclpy.init()
    world_path = sys.argv[1] if len(sys.argv) > 1 else str(pathlib.Path(__file__).resolve().parent.parent / "world.json")
    w = json.load(open(world_path, encoding="utf-8"))
    g = w["goal"]
    nav = BasicNavigator()
    last = {"x": None}
    nav.create_subscription(Odometry, "odom", lambda m: last.__setitem__("x", m), 10)
    localizer = sys.argv[3] if len(sys.argv) > 3 else "static"
    reloc = len(sys.argv) > 4 and sys.argv[4] == "1"
    # 重定位要的觀測:安全旗標(第一次看到 WDT_RESET 的時刻)、AMCL 位姿的歷史
    st = {"t_wdt": None, "poses": []}
    def on_flags(m):
        if m.data & FLAG_WDT_RESET and st["t_wdt"] is None:
            st["t_wdt"] = time.monotonic()
            nav.get_logger().warn("WDT_RESET seen")
    nav.create_subscription(UInt8, "hil/safety_flags", on_flags, 10)
    if localizer == "amcl":
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        nav.create_subscription(PoseWithCovarianceStamped, "amcl_pose", lambda m: st["poses"].append((time.monotonic(), m)), qos)
    pub_init = nav.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
    ack_cli = nav.create_client(Trigger, "hil/fault_ack")
    # localizer='robot_localization' 讓 waitUntilNav2Active 跳過等 amcl;amcl 時照常等它
    nav.waitUntilNav2Active(navigator="bt_navigator", localizer="amcl" if localizer == "amcl" else "robot_localization")
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
    reloc_info = None
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
            # 重定位模式:底盤重啟了就不要讓手上的 goal 拖到 Nav2 逾時(重啟前一刻送出、driver 取消之後才到的重送也算)
            if reloc and reloc_info is None and st["t_wdt"] is not None:
                nav.get_logger().warn("WDT_RESET while navigating: cancel task, relocalize")
                nav.cancelTask()
                break
            if time.monotonic() - t0 > 240:
                nav.cancelTask()
                break
        res = nav.getResult()
        # goal 被 driver 取消時,安全旗標那筆訊息可能還在佇列裡:先把回呼處理完再判斷
        t_drain = time.monotonic() + 1.0
        while time.monotonic() < t_drain:
            rclpy.spin_once(nav, timeout_sec=0.1)
        if reloc and reloc_info is None and st["t_wdt"] is not None:
            reloc_info = relocalize(nav, localizer, st, pub_init, ack_cli)
            nav.get_logger().warn("relocalized: %s; resend goal" % reloc_info)
            continue
        if res == TaskResult.FAILED and attempts <= retries and not (reloc and st["t_wdt"] is not None):
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
    print('[nav] {"result": "%s", "attempts": %d, "goal": [%g, %g], "odom_end": [%.3f, %.3f, %.3f], "dist_to_goal_m": %.3f, "wall_s": %.1f, "reloc": %s}' % (
        {TaskResult.SUCCEEDED: "succeeded", TaskResult.CANCELED: "canceled", TaskResult.FAILED: "failed"}.get(res, str(res)),
        attempts, g["x"], g["y"], x, y, yaw, dist, time.monotonic() - t0, json.dumps(reloc_info)), flush=True)
    nav.destroy_node()
    rclpy.try_shutdown()


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def relocalize(nav, localizer, st, pub_init, ack_cli):
    info = {"localizer": localizer}
    t_start = time.monotonic()
    if localizer == "amcl":
        # 重啟前的位姿:WDT_RESET 看到之前 0.3 s 的最後一筆(重啟後的 odom 跳回原點,AMCL 可能已經被帶走)
        before = [m for (t, m) in st["poses"] if t <= st["t_wdt"] - 0.3]
        if not before:
            info["error"] = "no amcl pose before reset"
        else:
            p0 = before[-1]
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = nav.get_clock().now().to_msg()
            msg.pose.pose = p0.pose.pose
            cov = [0.0] * 36
            cov[0] = cov[7] = 0.5 ** 2
            cov[35] = 0.5 ** 2
            msg.pose.covariance = cov
            t_pub = time.monotonic()
            pub_init.publish(msg)
            info["initial"] = [round(p0.pose.pose.position.x, 3), round(p0.pose.pose.position.y, 3), round(yaw_of(p0.pose.pose.orientation), 3)]
            nomotion = nav.create_client(Empty, "request_nomotion_update")
            nomotion.wait_for_service(timeout_sec=5.0)
            t_nm = 0.0
            n_nm = 0
            converged = None
            sig_log = []
            t_log = t_pub
            while time.monotonic() - t_pub < RELOC_TIMEOUT_S:
                rclpy.spin_once(nav, timeout_sec=0.1)
                if time.monotonic() - t_nm >= 0.2:
                    nomotion.call_async(Empty.Request())
                    t_nm = time.monotonic(); n_nm += 1
                after = [m for (t, m) in st["poses"] if t > t_pub + 0.2]
                if after and time.monotonic() - t_log >= 2.0:
                    c = after[-1].pose.covariance
                    sig_log.append([round(time.monotonic() - t_pub, 1), round(c[0] ** 0.5, 3), round(c[7] ** 0.5, 3), round(c[35] ** 0.5, 3)])
                    nav.get_logger().info("reloc sigma t=%.1f s x=%.3f y=%.3f yaw=%.3f (n=%d)" % (sig_log[-1][0], sig_log[-1][1], sig_log[-1][2], sig_log[-1][3], len(after)))
                    t_log = time.monotonic()
                if after:
                    c = after[-1].pose.covariance
                    if c[0] ** 0.5 <= RELOC_SIGMA_XY and c[7] ** 0.5 <= RELOC_SIGMA_XY and c[35] ** 0.5 <= RELOC_SIGMA_YAW and len(after) >= 5:
                        converged = after[-1]
                        break
            info["sigma_log"] = sig_log[:30]
            info["nomotion_calls"] = n_nm
            if converged is None:
                info["error"] = "amcl did not converge in %.0f s" % RELOC_TIMEOUT_S
            else:
                pp = converged.pose.pose
                info["converged"] = [round(pp.position.x, 3), round(pp.position.y, 3), round(yaw_of(pp.orientation), 3)]
                info["sigma"] = [round(converged.pose.covariance[0] ** 0.5, 4), round(converged.pose.covariance[35] ** 0.5, 4)]
                info["converge_s"] = round(time.monotonic() - t_pub, 2)
    else:
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            rclpy.spin_once(nav, timeout_sec=0.1)
    info["reloc_s"] = round(time.monotonic() - t_start, 2)
    ack_cli.wait_for_service(timeout_sec=10.0)
    fut = ack_cli.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(nav, fut, timeout_sec=10.0)
    info["ack"] = fut.result().message if fut.result() else "no response"
    return info


if __name__ == "__main__":
    main()
