#!/usr/bin/env bash
# 場域 GPU 主機上的 Isaac 受控體:sync(rsync 四個檔到 ~/hil-plant/)/ start(重啟,等到 listening)/ stop / log / load。
# 每一輪閉環都 start 一次:受控體的位姿與 tick 跨連線累積,重啟才是同一個起點(約 20–40 s)。
# WORLD=1 start:受控體載入 world.json(牆與方塊當靜態碰撞體、PhysX 射線當雷射);UPPER=nav2 的 run_loop 要這個。
set -Eeuo pipefail
cd "$(dirname "$0")/.."
R=tools/remote.sh
WORLD_ARG=""; [ "${WORLD:-0}" = 1 ] && WORLD_ARG="--world world.json"
# TOPVIEW=1:真實俯視相機,每 100 ms 一幀 PNG 到 ~/hil-plant/topcam/(start 前清空);fetch 把它 rsync 回 out/topcam_isaac/
[ "${TOPVIEW:-0}" = 1 ] && WORLD_ARG="$WORLD_ARG --topview topcam"
# CPUSET=6,7:受控體綁在這幾個核(主機共用時限 CPU;taskset 讓 Isaac 的所有執行緒只排在這些核上)
PIN=""; [ -n "${CPUSET:-}" ] && PIN="taskset -c $CPUSET"
case "${1:-}" in
  sync)
    for f in plant/isaac_plant.py plant/world.py calib.json world.json; do $R up "$f" '~/hil-plant/'; done
    echo "[isaac_plant] synced: isaac_plant.py world.py calib.json world.json → ~/hil-plant/" ;;
  start)
    [ "${TOPVIEW:-0}" = 1 ] && $R ssh 'rm -rf ~/hil-plant/topcam'
    $R ssh "cd ~/hil-plant && { [ -f isaac_plant.pid ] && kill \$(cat isaac_plant.pid) 2>/dev/null; sleep 1; } ;
            (nohup $PIN ~/isaac-run.sh isaac_plant.py --bind 127.0.0.1:3700 --calib calib.json --tcp $WORLD_ARG > isaac_plant.log 2>&1 & echo \$! > isaac_plant.pid)"
    for i in $(seq 1 60); do
      $R ssh 'grep -q "isaac_plant\] listening" ~/hil-plant/isaac_plant.log' 2>/dev/null && { echo "[isaac_plant] listening"; exit 0; }
      $R ssh 'grep -q "Traceback" ~/hil-plant/isaac_plant.log' 2>/dev/null && { echo "[isaac_plant] Traceback:"; $R ssh 'grep -A12 Traceback ~/hil-plant/isaac_plant.log | cut -c1-200'; exit 1; }
      sleep 3
    done
    echo "[isaac_plant] 180 s 沒起來"; exit 1 ;;
  stop) $R ssh 'cd ~/hil-plant && [ -f isaac_plant.pid ] && kill $(cat isaac_plant.pid) 2>/dev/null && rm -f isaac_plant.pid && echo "[isaac_plant] stopped" || echo "[isaac_plant] 沒在跑"' ;;
  log)  $R ssh 'grep "isaac_plant\]" ~/hil-plant/isaac_plant.log | tail -${2:-5}' ;;
  load) $R ssh 'uptime | sed "s/.*load/load/"; nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader; nvidia-smi -q | grep "License Status"' ;;
  fetch) mkdir -p "${2:-out/topcam_isaac}"; $R down '~/hil-plant/topcam/' "${2:-out/topcam_isaac}/"; ls "${2:-out/topcam_isaac}" | wc -l ;;
  probe) $R ssh "cd ~/hil-plant && ~/isaac-run.sh isaac_plant.py --calib calib.json --probe $WORLD_ARG 2>&1 | grep '\[probe\]'" ;;
  *) echo "用法: $0 sync|start|stop|log [n]|load|probe|fetch [dir]   (WORLD=1 加 --world world.json;TOPVIEW=1 加 --topview topcam)"; exit 2 ;;
esac
