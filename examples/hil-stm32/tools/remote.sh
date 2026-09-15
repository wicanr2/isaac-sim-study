#!/usr/bin/env bash
# 場域 GPU 主機的連線包裝。主機、port、帳號一律從機密目錄的入口腳本推出,不寫在 repo 裡。
#   tools/remote.sh ssh  '<指令>'                       在 Isaac 帳號執行
#   tools/remote.sh up   <本機路徑> <遠端相對家目錄路徑>   rsync 上去(會先 mkdir -p)
#   tools/remote.sh down <遠端相對家目錄路徑> <本機路徑>   rsync 回來
#   tools/remote.sh tunnel <本機位址:埠|埠> <遠端埠>      ssh -L,前景,Ctrl-C 收(位址省略 = 127.0.0.1)
# 遠端工作區固定 ~/hil-plant;不動家目錄其他東西。那台沒有 docker / root,只用既有 Isaac venv。
set -Eeuo pipefail
# 入口腳本的路徑、Isaac 環境的帳號名都放在 repo 外的 env 檔(連場域名都不進 repo):
#   HIL_REMOTE_ENTRY=<機密目錄裡的 ssh 入口腳本>
#   HIL_REMOTE_USER=<Isaac 環境所在的帳號;與入口腳本的帳號不同>
ENV_FILE="${HIL_REMOTE_ENV:-$HOME/.config/hil-stm32/remote.env}"
[ -f "$ENV_FILE" ] && . "$ENV_FILE"
: "${HIL_REMOTE_ENTRY:?請在 $ENV_FILE 設 HIL_REMOTE_ENTRY}"
: "${HIL_REMOTE_USER:?請在 $ENV_FILE 設 HIL_REMOTE_USER}"
ENTRY="$HIL_REMOTE_ENTRY"
ISAAC_USER="$HIL_REMOTE_USER"
LINE=$(grep -m1 '^ssh' "$ENTRY")
PORT=$(sed -E 's/.*-p ([0-9]+).*/\1/' <<<"$LINE")
HOST=$(awk '{print $NF}' <<<"$LINE" | sed "s/^[^@]*@/$ISAAC_USER@/")
SSH=(ssh -p "$PORT" -o BatchMode=yes -o ConnectTimeout=20)
for arg in "${3:-}" "${2:-}"; do
  case "${1:-}" in up|down) if [[ "$arg" == '~'* || "$arg" == /* ]] && [[ "$arg" != '~/hil-plant/'* ]]; then
    echo "拒絕:遠端路徑必須在 ~/hil-plant/ 底下(得到 $arg)" >&2; exit 4; fi;; esac
done
case "${1:-}" in
  ssh)    shift; "${SSH[@]}" "$HOST" "$@" ;;
  up)     "${SSH[@]}" "$HOST" "mkdir -p $3"; rsync -az -e "${SSH[*]}" "$2" "$HOST:$3" ;;
  down)   rsync -az -e "${SSH[*]}" "$HOST:$2" "$3" ;;
  tunnel) L="$2"; [[ "$L" == *:* ]] || L="127.0.0.1:$L"; exec "${SSH[@]}" -N -L "$L:127.0.0.1:$3" "$HOST" ;;
  *) sed -n 2,7p "$0"; exit 2 ;;
esac
