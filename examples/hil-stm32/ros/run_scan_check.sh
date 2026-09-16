#!/usr/bin/env bash
# 在 ros:jazzy-ros-base 容器裡跑:起 base driver(背景)→ 訂 /scan 幾秒印統計 → 收 driver。
set -Eeo pipefail
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR=/tmp/roslog ROS_HOME=/tmp/roshome
mkdir -p /tmp/roslog /tmp/roshome
python3 hil_base_driver.py --ros-args -p "bridge:=${BRIDGE:-127.0.0.1:3800}" &
DRV=$!
trap 'kill -INT $DRV 2>/dev/null; wait $DRV 2>/dev/null || true' EXIT
python3 scan_check.py --ros-args -p "seconds:=${SECONDS_CHECK:-8.0}"
