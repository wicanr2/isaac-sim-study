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
#   RECORD=1 ./run_loop.sh --fault bumper  # 跑完多產一支俯視圖錄影 out/run.mp4(+ _topview.svg/png;RECORD_GIF=1 多 gif);issue #6
#   ./run_loop.sh --mode realtime       # Renode 自由跑、橋接每 5 ms 牆鐘取樣;Renode 容器自動給 4 核(CPUS= 覆蓋;2 核會被 CFS 每 100 ms 凍 50 ms)
#   RCCFIX=1 ./run_loop.sh --fault hang # RCC/IWDG 換成修正版:看門狗重置後 RCC_CSR.IWDGRSTF = 1(docs/hil/38 §1.2)
#   TIMERFIX=1 ./run_loop.sh            # TIM3 換成 renode/upstream/STM32_Timer_Fixed.cs(執行期載入的修正版)
#   PLANT=remote ./run_loop.sh          # 受控體在場域 GPU 主機:自動開 ssh -L 隧道,受控體那端要先起好(埠 3700,TCP;
#                                       # tools/isaac_plant_ctl.sh start;WORLD=1 / UPPER=nav2 時那端也要 WORLD=1 start)
#   CAN=socketcan ./run_loop.sh         # CAN 改走 Renode SocketCANBridge → 容器 netns 裡的 vcan0(37 篇 §4 的路 ②):
#                                       # 需要一個 --cap-add NET_ADMIN 且容器內 root 的 helper 建 vcan;橋接走 PF_CAN,沒有 ack
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
CAN="${CAN:-hook}"
# 編碼器:calib.json 的 encoder_source 決定韌體讀 TIM 還是 CAN;tim 時平台的 TIM2/TIM4 換成 upstream master 的
# timer(1.16.1 沒有 encoder mode),注入法由 ENC=hook|gpio|cnt 選(預設 hook)
ENC_SRC=$(python3 -c "import json;print(json.load(open('calib.json')).get('encoder_source','can'))")
ENC_PRE=()
if [ "$ENC_SRC" = tim ]; then
  [ "${TIMERFIX:-0}" = 1 ] && { echo "TIMERFIX=1 與 encoder_source=tim 目前不同時用(兩份平台描述)"; exit 2; }
  ENC_PRE=(-e "i @/w/renode/upstream/STM32_Timer_Master.cs" -e "i @/w/renode/hil_quadrature.cs" -e '$repl=@/w/renode/upstream/stm32f4-encoder.repl')
fi
# RCCFIX=1:RCC 與 IWDG 換成修正版(RCC_CSR 的重置旗標跨系統重置保留、看門狗重置設 IWDGRSTF;renode/upstream/STM32_ResetFlags.patch)
if [ "${RCCFIX:-0}" = 1 ]; then
  [ "$ENC_SRC" = tim ] || { echo "RCCFIX=1 目前只接 encoder_source=tim 的平台描述"; exit 2; }
  ENC_PRE=(-e "i @/w/renode/upstream/STM32_Timer_Master.cs" -e "i @/w/renode/hil_quadrature.cs"
           -e "i @/w/renode/upstream/STM32_IndependentWatchdog_Fixed.cs" -e "i @/w/renode/upstream/STM32F4_RCC_Fixed.cs"
           -e '$repl=@/w/renode/upstream/stm32f4-encoder-rccfix.repl')
fi
ENC_ARG=(); [ -n "${ENC:-}" ] && ENC_ARG=(--enc "$ENC")
[ "$CAN" = socketcan ] && { [ "$RESC" = hilctl ] || { echo "CAN=socketcan 與 TIMERFIX 不同時用"; exit 2; }; RESC=hilctl-socketcan; }
# CANHUBFIX=1:CANHub 換成 renode/upstream/CANHub_Fixed.cs(暫停時把主機來的訊框排隊,不丟;lockstep 才收得齊)
[ "$CAN" = socketcan ] && [ "${CANHUBFIX:-0}" = 1 ] && RESC=hilctl-socketcan-fixed
FW="${FW:-baremetal}"
case "$FW" in
  baremetal) ELF=/w/firmware/build/hilctl.elf; SYM=firmware/build/hilctl.sym; EXTRA=() ;;
  # FreeRTOS 版在原版 Renode 1.16.1 上第一個 SysTick 週期是 2^24 cycle(233 ms,NVIC 缺口,見 renode/upstream/),
  # 開機等待拉到 400 ms;修了 NVIC 之後可以回 100
  freertos)  ELF=/w/firmware-freertos/build/hilctl-rtos.elf; SYM=firmware-freertos/build/hilctl-rtos.sym; EXTRA=(--boot-ms 402 --dbg-extra 9) ;;
  *) echo "FW 只接受 baremetal 或 freertos"; exit 2 ;;
esac
# Renode 容器的 CPU 配額。lockstep 2 核夠(慢只是慢);realtime 要 4:Renode 行程在 realtime 下是模擬執行緒 + hook 執行緒
# + External Control 執行緒 + GC 一起跑,超過 2 核的配額就被 CFS 每 100 ms 凍住約 50 ms——量到的「50 ms 停頓」
# 是這個,不是主機負載(2026-09-16:同一負載下 cpus 2 → 4,停頓 70 次 × 50 ms → 9–24 次 × ≤ 28 ms)
DEFAULT_CPUS=2; case " $* " in *" realtime "*) DEFAULT_CPUS=4;; esac
CPUS="${CPUS:-$DEFAULT_CPUS}"
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
# socketcan 模式:Renode 進程等 vcan0 出現才起(SocketCANBridge 建構時就 bind);介面由下面的 helper 建
WAIT_VCAN=""; [ "$CAN" = socketcan ] && WAIT_VCAN='while [ ! -e /sys/class/net/vcan0 ]; do sleep 0.2; done; '
docker run -d -i --name "$NAME" --network "$RENODE_NET" --cpus "$CPUS" --memory 2g --pids-limit 256 \
  --log-opt max-size=10m --log-opt max-file=3 --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$PWD":/w "$RENODE_IMAGE" \
  sh -c "${WAIT_VCAN}exec renode --disable-xwt --console -e '\$bin=@$ELF' -e '\$quantum=\"${QUANTUM:-0.0001}\"' $(printf "%q " "${ENC_PRE[@]}") -e 'include @/w/renode/${RESC:-hilctl}.resc'" >/dev/null
if [ "$CAN" = socketcan ]; then
  # 建 vcan 要 CAP_NET_ADMIN 而且要是容器內的 root(非 root 行程拿不到 ambient capability)。
  # 只做這一件事:--rm、--read-only、只掛 tools/ 唯讀、netns 是 Renode 那個(--network none 的隔離 netns)。
  echo "[vcan] 在 $NAME 的 netns 建 vcan0(helper:root + NET_ADMIN,read-only)"
  docker run --rm --network "container:$NAME" --cap-add NET_ADMIN --user 0 --read-only --cpus 1 --memory 256m --pids-limit 32 \
    --log-opt max-size=10m --log-opt max-file=3 -v "$PWD/tools":/t:ro "$RENODE_IMAGE" python3 /t/vcan_up.py vcan0
  CAN_ARG=(--can socketcan:vcan0)
else
  CAN_ARG=()
fi

# 不用 bash 的 /dev/tcp 探埠:`echo >/dev/tcp/...` 會送一個換行,External Control server
# 把它當成握手的第一個 byte,狀態機從此錯位(2026-09-15 踩到)。改由橋接自己重試連線。

# WORLD=1:受控體載入 world.json(矩形房間 + 方塊):假雷射每 100 ms 一筆經橋接 3801 送上位、撞到就停;UPPER=nav2 自動開
WORLD_ARG=(); PLANT_WORLD=()
if [ "${WORLD:-0}" = 1 ] || [ "$UPPER" = nav2 ]; then
  WORLD_ARG=(--world /w/world.json --scan-listen 0.0.0.0:3801); PLANT_WORLD=(--world ../world.json)
fi
PLANT_ARG=(--plant fake)
case "$PLANT" in
  udp|tcp)
    TCPFLAG=(); [ "$PLANT" = tcp ] && TCPFLAG=(--tcp)
    echo "[plant] 啟動 $PNAME(plant/fake_plant.py,$PLANT 3700,與 Renode 同 netns)"
    docker run -d --name "$PNAME" --network "container:$NAME" --cpus 1 --memory 512m --pids-limit 64 \
      --log-opt max-size=10m --log-opt max-file=3 --user "$(id -u):$(id -g)" -e HOME=/tmp \
      -v "$PWD":/w -w /w/plant "$PY_IMAGE" python3 fake_plant.py --bind 0.0.0.0:3700 --calib ../calib.json "${TCPFLAG[@]}" "${PLANT_WORLD[@]}" >/dev/null
    PLANT_ARG=(--plant "$PLANT:127.0.0.1:3700")
    ;;
  remote)
    echo "[tunnel] ssh -L $GW:3700 → 場域 GPU 主機 127.0.0.1:3700"
    # 隧道的 stdout/stderr 導到檔案:不然 ssh 會把這支腳本的輸出管線佔住,跑完也收不掉
    tools/remote.sh tunnel "$GW:3700" 3700 > out/tunnel.log 2>&1 &
    TUNNEL_PID=$!
    for i in $(seq 1 40); do ss -ltn 2>/dev/null | grep -q "$GW:3700" && break; sleep 0.25; done
    ss -ltn | grep -q "$GW:3700" || { echo "隧道沒起來"; exit 1; }
    # Isaac 6.0.1 實測滑移:DriveAPI 直接吃 duty 時轉向 3.1%、直行 0.5%(2026-09-15 --probe 等速);
    # 馬達層 + 韌體斜坡之後 6 s 腳本量到直行 −0.1%、轉向 0.1%(2026-09-16,38 篇 §6.4);容差留 1%
    PLANT_ARG=(--plant "tcp:$GW:3700" --slip 0.01)
    ;;
  tcp:*|udp:*)
    PLANT_ARG=(--plant "$PLANT")
    ;;
esac

# --negative no-latch 是上位側的負對照(driver 對 WDT_RESET/STALL 不反應),橋接只印標籤;這裡把它翻成 driver 的參數
case " $* " in *" --negative no-latch "*) FAULT_LATCH=false ;; esac
UPPER_ARG=()
if [ "$UPPER" = ros ] || [ "$UPPER" = nav2 ]; then
  # ROS_SCRIPT:run_square.sh(預設,方形閉環)/ run_scan_check.sh(只驗 /scan);UPPER=nav2 → run_nav.sh 在 hil-nav2:jazzy(ros/Dockerfile.nav2)
  ROS_SCRIPT="${ROS_SCRIPT:-run_square.sh}"; ROS_CPUS=1
  if [ "$UPPER" = nav2 ]; then ROS_IMAGE="${NAV2_IMAGE:-hil-nav2:jazzy}"; ROS_SCRIPT=run_nav.sh; ROS_CPUS=2; fi
  echo "[ros] 啟動 $RNAME($ROS_IMAGE,ros/$ROS_SCRIPT,與 Renode 同 netns,$ROS_CPUS 核;log → out/ros.log)"
  docker run -d --name "$RNAME" --network "container:$NAME" --cpus "$ROS_CPUS" --memory 2g --pids-limit 256 \
    --log-opt max-size=10m --log-opt max-file=3 --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -e "SIDE_M=${SIDE_M:-0.6}" -e "TIMEOUT_S=${TIMEOUT_S:-120.0}" -e "SECONDS_CHECK=${SECONDS_CHECK:-8.0}" -e "FAULT_LATCH=${FAULT_LATCH:-true}" \
    -v "$PWD":/w -w /w/ros "$ROS_IMAGE" bash "./$ROS_SCRIPT" >/dev/null
  UPPER_ARG=(--upper tcp-listen:0.0.0.0:3800)
  [ "$UPPER" = nav2 ] && UPPER_ARG+=(--expect-goal 1)
elif [ "$UPPER" != script ]; then
  echo "UPPER 只接受 script、ros 或 nav2"; exit 2
fi

# 這一輪的 CSV 路徑(--log 可被 "$@" 覆蓋;後者贏);有世界時雷射也存檔(給 tools/topview.py)
LOG=out/run.csv; prev=""; for x in "$@"; do [ "$prev" = "--log" ] && LOG=$x; prev=$x; done
SCAN_ARG=(); [ ${#WORLD_ARG[@]} -gt 0 ] && SCAN_ARG=(--scan-log "${LOG%.csv}.scan")
LOAD_AT_START=$(cut -d' ' -f1 /proc/loadavg)
echo "[bridge] 開跑:${PLANT_ARG[*]} ${UPPER_ARG[*]} $*  (load $LOAD_AT_START)"
set +e
docker run --rm --network "container:$NAME" --cpus "$CPUS" --memory 1g --pids-limit 128 \
  --log-opt max-size=10m --log-opt max-file=3 --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$PWD":/w -w /w "$RUST_IMAGE" \
  ./bridge-rs/target/release/hil-bridge --log out/run.csv --sym "$SYM" "${EXTRA[@]}" "${PLANT_ARG[@]}" "${UPPER_ARG[@]}" "${CAN_ARG[@]}" "${ENC_ARG[@]}" "${WORLD_ARG[@]}" "${SCAN_ARG[@]}" "$@"
rc=$?
set -e
# RECORD=1:俯視圖錄影(issue #6):tools/topview.sh → uv 容器裡的 tools/topview.py。產出 ${LOG%.csv}.mp4 / _topview.svg / _topview.png;
# RECORD_GIF=1 多一支 gif。--fault bumper 時把 500 mm 的虛擬牆畫進去。
if [ "${RECORD:-0}" = 1 ]; then
  TV_ARGS=(--title "$(basename "${LOG%.csv}")" --meta "commit=$(git rev-parse --short HEAD 2>/dev/null);fw=$FW;plant=$PLANT;upper=$UPPER;args=$*;load=$LOAD_AT_START")
  [ ${#WORLD_ARG[@]} -gt 0 ] && TV_ARGS+=(--world world.json --scan "${LOG%.csv}.scan")
  [ "${RECORD_GIF:-0}" = 1 ] && TV_ARGS+=(--gif)
  case " $* " in *" bumper "*) TV_ARGS+=(--vline "0.5:bumper 牆");; esac
  echo "[record] tools/topview.sh $LOG"
  tools/topview.sh "$LOG" "${TV_ARGS[@]}"
fi
docker logs "$NAME" 2>&1 | sed 's/\x1b\[[0-9;]*m//g' > out/renode.log || true
if [ "$UPPER" = ros ] || [ "$UPPER" = nav2 ]; then
  docker logs "$RNAME" > out/ros.log 2>&1 || true
  grep "^\[square\]\|^\[scan_check\]\|^\[nav\]" out/ros.log || echo "[ros] 沒有結果行(看 out/ros.log)"
fi
[ "$PLANT" = "udp" ] && docker logs "$PNAME" > out/plant.log 2>&1 || true
echo "[done] rc=$rc  CSV: $LOG  Renode log: out/renode.log  USART2: renode/out/usart2.txt"
exit $rc
