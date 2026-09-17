#!/usr/bin/env bash
# 在 hil-nav2:jazzy 容器裡跑:產地圖 → 起 base driver + Nav2 最小組合(背景)→ NavigateToPose 到 world.json 的 goal → 收掉。
# 由 run_loop.sh 的 UPPER=nav2 呼叫。預設沒有 AMCL:map→odom 用靜態 identity TF(里程計對真值在 mm 級,38 篇 §1);LOCALIZER=amcl 換 AMCL。
set -Eeo pipefail
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR=/tmp/roslog ROS_HOME=/tmp/roshome
mkdir -p /tmp/roslog /tmp/roshome
python3 ../tools/gen_map.py ../world.json /tmp/map
PIDS=()
# FAULT_LATCH=false 是負對照:driver 對 WDT_RESET / STALL 不反應
python3 hil_base_driver.py --ros-args -p "bridge:=${BRIDGE:-127.0.0.1:3800}" -p "fault_latch:=${FAULT_LATCH:-true}" -p "plan_log:=${PLAN_LOG:-}" & PIDS+=($!)
# LOCALIZER=static(預設):map→odom 靜態 identity;amcl:AMCL 發 map→odom(GOAL 6 C-2)
LIFECYCLE_NODES='["map_server", "planner_server", "controller_server", "behavior_server", "bt_navigator"]'
if [ "${LOCALIZER:-static}" = amcl ]; then
  ros2 run nav2_amcl amcl --ros-args --params-file nav2_params.yaml & PIDS+=($!)
  LIFECYCLE_NODES='["map_server", "amcl", "planner_server", "controller_server", "behavior_server", "bt_navigator"]'
else
  ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom & PIDS+=($!)
fi
# CONTROLLER=mppi:controller_server 多疊一份 nav2_params_mppi.yaml(FollowPath 換 MPPI;D-3)
CTRL_PARAMS=(); [ "${CONTROLLER:-dwb}" = mppi ] && CTRL_PARAMS=(--params-file nav2_params_mppi.yaml)
for node in "nav2_map_server map_server" "nav2_planner planner_server" "nav2_behaviors behavior_server" "nav2_bt_navigator bt_navigator"; do
  ros2 run $node --ros-args --params-file nav2_params.yaml & PIDS+=($!)
done
ros2 run nav2_controller controller_server --ros-args --params-file nav2_params.yaml "${CTRL_PARAMS[@]}" & PIDS+=($!)
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args --params-file nav2_params.yaml -p "node_names:=$LIFECYCLE_NODES" & PIDS+=($!)
trap 'for p in "${PIDS[@]}"; do kill -INT $p 2>/dev/null; done; wait 2>/dev/null || true' EXIT
# RELOC=1:底盤重啟、driver 鎖住、goal 被取消之後,重定位(amcl)或直接(static)→ ack 解鎖 → 重送 goal
python3 nav_client.py ../world.json 1 "${LOCALIZER:-static}" "${RELOC:-0}"
