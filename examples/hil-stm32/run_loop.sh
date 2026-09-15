#!/usr/bin/env bash
# 一條指令跑 HIL 閉環:Renode(STM32 韌體)↔ Rust 橋接 ↔ 受控體。
#
#   ./run_loop.sh                       # 假受控體,6 s,預設腳本
#   ./run_loop.sh --seconds 3 --script "0:200,0"
#   ./run_loop.sh --negative bad-crc    # 負對照:每個命令的 CRC 都弄壞,驗收必須轉紅
#   BUILD=1 ./run_loop.sh               # 先重建韌體與橋接
#   PLANT=udp ./run_loop.sh             # 受控體改走 UDP:另起一個容器跑 plant/fake_plant.py
#
# 全部在 docker:Renode 容器 --network none;橋接容器共用它的 netns(還是不通外網)。
# 只停自己起的那一個容器(名稱帶 PID),不碰其他 docker 資源。
set -euo pipefail
cd "$(dirname "$0")"

RENODE_IMAGE="${RENODE_IMAGE:-antmicro/renode:latest}"       # 1.16.1
ARM_IMAGE="${ARM_IMAGE:-renode-golang-arm:bookworm}"          # arm-none-eabi-gcc 12.2
RUST_IMAGE="${RUST_IMAGE:-rust:1-slim-bookworm}"
PY_IMAGE="${PY_IMAGE:-ghcr.io/astral-sh/uv:python3.12-bookworm-slim}"   # 只用標準庫
PLANT="${PLANT:-fake}"
CPUS="${CPUS:-2}"
NAME="hil-renode-$$"
PNAME="hil-plant-$$"
COMMON=(--rm --cpus "$CPUS" --memory 2g --pids-limit 256
        --log-opt max-size=10m --log-opt max-file=3
        --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD":/w)

if [ "${BUILD:-0}" = "1" ]; then
  echo "[build] firmware"
  docker run "${COMMON[@]}" --network none -w /w/firmware "$ARM_IMAGE" make
  echo "[build] bridge"
  docker run "${COMMON[@]}" --network none -w /w/bridge-rs -e CARGO_HOME=/tmp/cargo "$RUST_IMAGE" \
    cargo build --release --offline
fi
test -f firmware/build/hilctl.elf || { echo "沒有 firmware/build/hilctl.elf,先 BUILD=1"; exit 2; }
test -x bridge-rs/target/release/hil-bridge || { echo "沒有橋接執行檔,先 BUILD=1"; exit 2; }

mkdir -p out renode/out
cleanup() {
  docker stop -t 2 "$NAME" >/dev/null 2>&1 || true
  docker stop -t 2 "$PNAME" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "[renode] 啟動 $NAME(Renode $RENODE_IMAGE,--network none,$CPUS 核)"
docker run -d -i --name "$NAME" --network none --cpus "$CPUS" --memory 2g --pids-limit 256 \
  --log-opt max-size=10m --log-opt max-file=3 --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$PWD":/w "$RENODE_IMAGE" \
  renode --disable-xwt --console -e "include @/w/renode/hilctl.resc" >/dev/null

# 不用 bash 的 /dev/tcp 探埠:`echo >/dev/tcp/...` 會送一個換行,External Control server
# 把它當成握手的第一個 byte,狀態機從此錯位(2026-09-15 踩到)。改由橋接自己重試連線。

PLANT_ARG=(--plant fake)
if [ "$PLANT" = "udp" ]; then
  echo "[plant] 啟動 $PNAME(plant/fake_plant.py,UDP 3700,與 Renode 同 netns)"
  docker run -d --name "$PNAME" --network "container:$NAME" --cpus 1 --memory 512m --pids-limit 64 \
    --log-opt max-size=10m --log-opt max-file=3 --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$PWD":/w -w /w/plant "$PY_IMAGE" python3 fake_plant.py --bind 0.0.0.0:3700 --calib ../calib.json >/dev/null
  PLANT_ARG=(--plant udp:127.0.0.1:3700)
fi

echo "[bridge] 開跑:${PLANT_ARG[*]} $*"
set +e
docker run --rm --network "container:$NAME" --cpus "$CPUS" --memory 1g --pids-limit 128 \
  --log-opt max-size=10m --log-opt max-file=3 --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$PWD":/w -w /w "$RUST_IMAGE" \
  ./bridge-rs/target/release/hil-bridge --log out/run.csv "${PLANT_ARG[@]}" "$@"
rc=$?
set -e
docker logs "$NAME" 2>&1 | sed 's/\x1b\[[0-9;]*m//g' > out/renode.log || true
[ "$PLANT" = "udp" ] && docker logs "$PNAME" > out/plant.log 2>&1 || true
echo "[done] rc=$rc  CSV: out/run.csv  Renode log: out/renode.log  USART2: renode/out/usart2.txt"
exit $rc
