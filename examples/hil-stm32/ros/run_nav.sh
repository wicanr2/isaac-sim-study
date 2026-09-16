#!/usr/bin/env bash
# 在 hil-nav2:jazzy 容器裡跑:產地圖 → 起 base driver + Nav2 最小組合(背景)→ NavigateToPose 到 world.json 的 goal → 收掉。
# 由 run_loop.sh 的 UPPER=nav2 呼叫。沒有 AMCL:map→odom 用靜態 identity TF(里程計對真值在 mm 級,38 篇 §1)。
set -Eeo pipefail
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR=/tmp/roslog ROS_HOME=/tmp/roshome
mkdir -p /tmp/roslog /tmp/roshome
python3 ../tools/gen_map.py ../world.json /tmp/map
PIDS=()
# FAULT_LATCH=false 是負對照:driver 對 WDT_RESET / STALL 不反應
python3 hil_base_driver.py --ros-args -p "bridge:=${BRIDGE:-127.0.0.1:3800}" -p "fault_latch:=${FAULT_LATCH:-true}" & PIDS+=($!)
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom & PIDS+=($!)
for node in "nav2_map_server map_server" "nav2_planner planner_server" "nav2_controller controller_server" \
            "nav2_behaviors behavior_server" "nav2_bt_navigator bt_navigator" "nav2_lifecycle_manager lifecycle_manager"; do
  ros2 run $node --ros-args --params-file nav2_params.yaml & PIDS+=($!)
done
trap 'for p in "${PIDS[@]}"; do kill -INT $p 2>/dev/null; done; wait 2>/dev/null || true' EXIT
python3 nav_client.py ../world.json
