#!/usr/bin/env bash
# 增益掃描:同一個步階腳本,逐組 --cfg 覆蓋韌體 g_cfg(不重編),印一張表。
#   tools/tune_sweep.sh "64 128 256" "6 13 26" [ff_q8]   → 9 組,每組約 10 s
set -euo pipefail
cd "$(dirname "$0")/.."
KPS="${1:-64 128 256}"; KIS="${2:-6 13 26}"; FF="${3:-256}"
printf "%-6s %-6s %-6s | %-12s %-8s %-10s %-8s\n" kp ki ff rise10-90 overshoot settle2% ss_err
for kp in $KPS; do for ki in $KIS; do
  ./run_loop.sh --seconds 2.5 --script "0:0,0;0.5:300,0" --cfg "kp=$kp,ki=$ki,ff=$FF" >/dev/null 2>&1 || true
  r=$(python3 tools/step_response.py out/run.csv)
  rise=$(echo "$r" | grep "上升時間" | sed -E 's/.*: ([^(]*)\(.*/\1/' | tr -d ' ')
  os=$(echo "$r" | grep "超調" | sed -E 's/.*: ([^(]*)\(.*/\1/' | tr -d ' ')
  st=$(echo "$r" | grep "±2%" | sed -E 's/.*: (.*)$/\1/' | tr -d ' ')
  ss=$(echo "$r" | grep "穩態" | sed -E 's/.*誤差 (.*)$/\1/' | tr -d ' ')
  printf "%-6s %-6s %-6s | %-12s %-8s %-10s %-8s\n" "$kp" "$ki" "$FF" "$rise" "$os" "$st" "$ss"
done; done
