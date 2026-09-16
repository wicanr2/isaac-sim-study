#!/usr/bin/env bash
# 在 ros:jazzy-ros-base 容器裡跑:起 base driver(背景)→ 跑方形閉環 → 收 driver。
# 由 run_loop.sh 的 UPPER=ros 呼叫;也可以自己 docker run 進來手動用。
set -Eeo pipefail   # 不用 -u:setup.bash 內部有未定義變數
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR=/tmp/roslog ROS_HOME=/tmp/roshome
mkdir -p /tmp/roslog /tmp/roshome
python3 hil_base_driver.py --ros-args -p "bridge:=${BRIDGE:-127.0.0.1:3800}" &
DRV=$!
trap 'kill -INT $DRV 2>/dev/null; wait $DRV 2>/dev/null || true' EXIT
python3 square_client.py --ros-args -p "side_m:=${SIDE_M:-0.6}" -p "timeout_s:=${TIMEOUT_S:-120.0}"
