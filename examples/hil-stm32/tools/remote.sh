#!/usr/bin/env bash
# 場域 GPU 主機的連線包裝。主機、port、帳號一律從機密目錄的入口腳本推出,不寫在 repo 裡。
#   tools/remote.sh ssh  '<指令>'                       在 Isaac 帳號執行
#   tools/remote.sh up   <本機路徑> <遠端相對家目錄路徑>   rsync 上去(會先 mkdir -p)
#   tools/remote.sh down <遠端相對家目錄路徑> <本機路徑>   rsync 回來
#   tools/remote.sh tunnel <本機埠> <遠端埠>              ssh -L,前景,Ctrl-C 收
# 遠端工作區固定 ~/hil-plant;不動家目錄其他東西。那台沒有 docker / root,只用既有 Isaac venv。
set -Eeuo pipefail
ENTRY="${HIL_REMOTE_ENTRY:-$HOME/00-機密/tainan/login_rtx6000.sh}"
ISAAC_USER="${HIL_REMOTE_USER:-jinher002}"
LINE=$(grep -m1 '^ssh' "$ENTRY")
PORT=$(sed -E 's/.*-p ([0-9]+).*/\1/' <<<"$LINE")
HOST=$(awk '{print $NF}' <<<"$LINE" | sed "s/^[^@]*@/$ISAAC_USER@/")
SSH=(ssh -p "$PORT" -o BatchMode=yes -o ConnectTimeout=20)
for arg in "${3:-}" "${2:-}"; do
  case "${1:-}" in up|down) if [[ "$arg" == '~'* || "$arg" == /users/* ]] && [[ "$arg" != '~/hil-plant/'* ]]; then
    echo "拒絕:遠端路徑必須在 ~/hil-plant/ 底下(得到 $arg)" >&2; exit 4; fi;; esac
done
case "${1:-}" in
  ssh)    shift; "${SSH[@]}" "$HOST" "$@" ;;
  up)     "${SSH[@]}" "$HOST" "mkdir -p $3"; rsync -az -e "${SSH[*]}" "$2" "$HOST:$3" ;;
  down)   rsync -az -e "${SSH[*]}" "$HOST:$2" "$3" ;;
  tunnel) "${SSH[@]}" -N -L "127.0.0.1:$2:127.0.0.1:$3" "$HOST" ;;
  *) sed -n 2,7p "$0"; exit 2 ;;
esac
