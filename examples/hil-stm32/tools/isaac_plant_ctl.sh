#!/usr/bin/env bash
# 場域 GPU 主機上的 Isaac 受控體:start(重啟,等到 listening)/ stop / log。
# 每一輪閉環都 start 一次:受控體的位姿與 tick 跨連線累積,重啟才是同一個起點(約 20–40 s)。
set -Eeuo pipefail
cd "$(dirname "$0")/.."
R=tools/remote.sh
case "${1:-}" in
  start)
    $R ssh 'cd ~/hil-plant && { [ -f isaac_plant.pid ] && kill $(cat isaac_plant.pid) 2>/dev/null; sleep 1; } ;
            (nohup ~/isaac-run.sh isaac_plant.py --bind 127.0.0.1:3700 --calib calib.json --tcp > isaac_plant.log 2>&1 & echo $! > isaac_plant.pid)'
    for i in $(seq 1 60); do
      $R ssh 'grep -q "isaac_plant\] listening" ~/hil-plant/isaac_plant.log' 2>/dev/null && { echo "[isaac_plant] listening"; exit 0; }
      $R ssh 'grep -q "Traceback" ~/hil-plant/isaac_plant.log' 2>/dev/null && { echo "[isaac_plant] Traceback:"; $R ssh 'grep -A12 Traceback ~/hil-plant/isaac_plant.log | cut -c1-200'; exit 1; }
      sleep 3
    done
    echo "[isaac_plant] 180 s 沒起來"; exit 1 ;;
  stop) $R ssh 'cd ~/hil-plant && [ -f isaac_plant.pid ] && kill $(cat isaac_plant.pid) 2>/dev/null && rm -f isaac_plant.pid && echo "[isaac_plant] stopped" || echo "[isaac_plant] 沒在跑"' ;;
  log)  $R ssh 'grep "isaac_plant\]" ~/hil-plant/isaac_plant.log | tail -${2:-5}' ;;
  load) $R ssh 'uptime | sed "s/.*load/load/"; nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader; nvidia-smi -q | grep "License Status"' ;;
  *) echo "用法: $0 start|stop|log [n]|load"; exit 2 ;;
esac
