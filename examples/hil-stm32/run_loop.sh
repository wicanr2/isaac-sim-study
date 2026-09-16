#!/usr/bin/env bash
# 一條指令跑 HIL 閉環:Renode(STM32 韌體)↔ Rust 橋接 ↔ 受控體。
#
#   ./run_loop.sh                       # 假受控體,6 s,預設腳本
#   ./run_loop.sh --seconds 3 --script "0:200,0"
#   ./run_loop.sh --negative bad-crc    # 負對照:每個命令的 CRC 都弄壞,驗收必須轉紅
#   BUILD=1 ./run_loop.sh               # 先重建韌體與橋接
#   PLANT=udp ./run_loop.sh             # 受控體改走 UDP:另起一個容器跑 plant/fake_plant.py
#   PLANT=tcp ./run_loop.sh             # 同上但走 TCP(ssh -L 隧道用的那條路)
#   FW=freertos ./run_loop.sh           # 韌體換成 FreeRTOS 版(firmware-freertos/)
#   TIMERFIX=1 ./run_loop.sh            # TIM3 換成 renode/upstream/STM32_Timer_Fixed.cs(執行期載入的修正版)
#   PLANT=remote ./run_loop.sh          # 受控體在場域 GPU 主機:自動開 ssh -L 隧道,受控體那端要先起好(埠 3700,TCP)
#   UPPER=ros ./run_loop.sh --seconds 40  # 上位換成 ROS 2 Jazzy:另起 ros:jazzy-ros-base 容器跑 ros/run_square.sh
#                                         #(base driver + 方形閉環),橋接 --upper tcp-listen:0.0.0.0:3800
#
# 全部在 docker:Renode 容器 --network none;橋接容器共用它的 netns(還是不通外網)。
# 只停自己起的那一個容器(名稱帶 PID),不碰其他 docker 資源。
set -euo pipefail
cd "$(dirname "$0")"

RENODE_IMAGE="${RENODE_IMAGE:-antmicro/renode:latest}"       # 1.16.1
ARM_IMAGE="${ARM_IMAGE:-renode-golang-arm:bookworm}"          # arm-none-eabi-gcc 12.2
RUST_IMAGE="${RUST_IMAGE:-rust:1-slim-bookworm}"
PY_IMAGE="${PY_IMAGE:-ghcr.io/astral-sh/uv:python3.12-bookworm-slim}"   # 只用標準庫
ROS_IMAGE="${ROS_IMAGE:-ros:jazzy-ros-base}"                  # rclpy 7.1.11 + tf2_msgs
UPPER="${UPPER:-script}"
PLANT="${PLANT:-fake}"
RESC=hilctl; [ "${TIMERFIX:-0}" = 1 ] && RESC=hilctl-timerfix
FW="${FW:-baremetal}"
case "$FW" in
  baremetal) ELF=/w/firmware/build/hilctl.elf; SYM=firmware/build/hilctl.sym; EXTRA=() ;;
  # FreeRTOS 版在原版 Renode 1.16.1 上第一個 SysTick 週期是 2^24 cycle(233 ms,NVIC 缺口,見 renode/upstream/),
  # 開機等待拉到 400 ms;修了 NVIC 之後可以回 100
  freertos)  ELF=/w/firmware-freertos/build/hilctl-rtos.elf; SYM=firmware-freertos/build/hilctl-rtos.sym; EXTRA=(--boot-ms 400 --dbg-extra 9) ;;
  *) echo "FW 只接受 baremetal 或 freertos"; exit 2 ;;
esac
CPUS="${CPUS:-2}"
NAME="hil-renode-$$"
PNAME="hil-plant-$$"
RNAME="hil-ros-$$"
TUNNEL_PID=""
COMMON=(--rm --cpus "$CPUS" --memory 2g --pids-limit 256
        --log-opt max-size=10m --log-opt max-file=3
        --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD":/w)

if [ "${BUILD:-0}" = "1" ]; then
  echo "[build] firmware(裸機 + FreeRTOS)"
  docker run "${COMMON[@]}" --network none -w /w/firmware "$ARM_IMAGE" make
  docker run "${COMMON[@]}" --network none -w /w/firmware-freertos "$ARM_IMAGE" make
  echo "[build] bridge"
  docker run "${COMMON[@]}" --network none -w /w/bridge-rs -e CARGO_HOME=/tmp/cargo "$RUST_IMAGE" \
    cargo build --release --offline
fi
test -f "${ELF#/w/}" || { echo "沒有 ${ELF#/w/},先 BUILD=1"; exit 2; }
test -x bridge-rs/target/release/hil-bridge || { echo "沒有橋接執行檔,先 BUILD=1"; exit 2; }

mkdir -p out renode/out
cleanup() {
  docker stop -t 2 "$NAME" >/dev/null 2>&1 || true
  docker stop -t 2 "$PNAME" >/dev/null 2>&1 || true
  docker stop -t 2 "$RNAME" >/dev/null 2>&1 || true
  # remote.sh 用 exec 起 ssh,所以 TUNNEL_PID 就是 ssh 本身
  [ -n "$TUNNEL_PID" ] && kill "$TUNNEL_PID" 2>/dev/null || true
}
trap cleanup EXIT

# 遠端受控體時 Renode 容器改掛 docker 預設 bridge 網路:容器能連到 bridge 的閘道位址,
# ssh -L 就綁在那個閘道位址上——隧道只有容器看得到,Renode 的埠也沒有 publish 到主機。
RENODE_NET=none
if [ "$PLANT" = "remote" ]; then
  RENODE_NET=bridge
  GW=$(docker network inspect bridge -f '{{(index .IPAM.Config 0).Gateway}}')
fi
echo "[renode] 啟動 $NAME(Renode $RENODE_IMAGE,--network $RENODE_NET,$CPUS 核)"
docker run -d -i --name "$NAME" --network "$RENODE_NET" --cpus "$CPUS" --memory 2g --pids-limit 256 \
  --log-opt max-size=10m --log-opt max-file=3 --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$PWD":/w "$RENODE_IMAGE" \
  renode --disable-xwt --console -e "\$bin=@$ELF" -e "\$quantum=\"${QUANTUM:-0.0001}\"" -e "include @/w/renode/${RESC:-hilctl}.resc" >/dev/null

# 不用 bash 的 /dev/tcp 探埠:`echo >/dev/tcp/...` 會送一個換行,External Control server
# 把它當成握手的第一個 byte,狀態機從此錯位(2026-09-15 踩到)。改由橋接自己重試連線。

PLANT_ARG=(--plant fake)
case "$PLANT" in
  udp|tcp)
    TCPFLAG=(); [ "$PLANT" = tcp ] && TCPFLAG=(--tcp)
    echo "[plant] 啟動 $PNAME(plant/fake_plant.py,$PLANT 3700,與 Renode 同 netns)"
    docker run -d --name "$PNAME" --network "container:$NAME" --cpus 1 --memory 512m --pids-limit 64 \
      --log-opt max-size=10m --log-opt max-file=3 --user "$(id -u):$(id -g)" -e HOME=/tmp \
      -v "$PWD":/w -w /w/plant "$PY_IMAGE" python3 fake_plant.py --bind 0.0.0.0:3700 --calib ../calib.json "${TCPFLAG[@]}" >/dev/null
    PLANT_ARG=(--plant "$PLANT:127.0.0.1:3700")
    ;;
  remote)
    echo "[tunnel] ssh -L $GW:3700 → 場域 GPU 主機 127.0.0.1:3700"
    # 隧道的 stdout/stderr 導到檔案:不然 ssh 會把這支腳本的輸出管線佔住,跑完也收不掉
    tools/remote.sh tunnel "$GW:3700" 3700 > out/tunnel.log 2>&1 &
    TUNNEL_PID=$!
    for i in $(seq 1 40); do ss -ltn 2>/dev/null | grep -q "$GW:3700" && break; sleep 0.25; done
    ss -ltn | grep -q "$GW:3700" || { echo "隧道沒起來"; exit 1; }
    # Isaac 6.0.1 實測滑移:轉向 3.1%、直行 0.5%(2026-09-15);容差用 5%
    PLANT_ARG=(--plant "tcp:$GW:3700" --slip 0.05)
    ;;
  tcp:*|udp:*)
    PLANT_ARG=(--plant "$PLANT")
    ;;
esac

UPPER_ARG=()
if [ "$UPPER" = ros ]; then
  echo "[ros] 啟動 $RNAME($ROS_IMAGE,ros/run_square.sh,與 Renode 同 netns;log → out/ros.log)"
  docker run -d --name "$RNAME" --network "container:$NAME" --cpus 1 --memory 1g --pids-limit 128 \
    --log-opt max-size=10m --log-opt max-file=3 --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -e "SIDE_M=${SIDE_M:-0.6}" -e "TIMEOUT_S=${TIMEOUT_S:-120.0}" \
    -v "$PWD":/w -w /w/ros "$ROS_IMAGE" bash ./run_square.sh >/dev/null
  UPPER_ARG=(--upper tcp-listen:0.0.0.0:3800)
elif [ "$UPPER" != script ]; then
  echo "UPPER 只接受 script 或 ros"; exit 2
fi

echo "[bridge] 開跑:${PLANT_ARG[*]} ${UPPER_ARG[*]} $*"
set +e
docker run --rm --network "container:$NAME" --cpus "$CPUS" --memory 1g --pids-limit 128 \
  --log-opt max-size=10m --log-opt max-file=3 --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$PWD":/w -w /w "$RUST_IMAGE" \
  ./bridge-rs/target/release/hil-bridge --log out/run.csv --sym "$SYM" "${EXTRA[@]}" "${PLANT_ARG[@]}" "${UPPER_ARG[@]}" "$@"
rc=$?
set -e
docker logs "$NAME" 2>&1 | sed 's/\x1b\[[0-9;]*m//g' > out/renode.log || true
if [ "$UPPER" = ros ]; then
  docker logs "$RNAME" > out/ros.log 2>&1 || true
  grep "^\[square\]" out/ros.log || echo "[square] 沒有結果行(方形沒跑完?看 out/ros.log)"
fi
[ "$PLANT" = "udp" ] && docker logs "$PNAME" > out/plant.log 2>&1 || true
echo "[done] rc=$rc  CSV: out/run.csv  Renode log: out/renode.log  USART2: renode/out/usart2.txt"
exit $rc
