#!/usr/bin/env bash
# 第 2 步驗收(不經橋接):在 Renode 容器裡跑 renode/io_check.resc,把輸出印出來。
# 用法:tools/io_check.sh [裸機 ELF 路徑,預設 firmware/build/hilctl.elf]
set -e
cd "$(dirname "$0")/.."
ELF="${1:-firmware/build/hilctl.elf}"
test -f "$ELF" || { echo "沒有 $ELF,先 BUILD=1 ./run_loop.sh"; exit 2; }
RENODE_IMAGE="${RENODE_IMAGE:-antmicro/renode:latest}"
# encoder_source=tim 的韌體要 master 版 timer(TIM2/TIM4 encoder mode),同 run_loop.sh 的 ENC_PRE
docker run --rm --network none --cpus "${CPUS:-2}" --memory 2g --pids-limit 256 \
  --log-opt max-size=10m --log-opt max-file=3 --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$PWD":/w "$RENODE_IMAGE" \
  renode --disable-xwt --console -e "\$bin=@/w/$ELF" \
    -e "i @/w/renode/upstream/STM32_Timer_Master.cs" -e "i @/w/renode/hil_quadrature.cs" \
    -e '$repl=@/w/renode/upstream/stm32f4-encoder.repl' \
    -e 'include @/w/renode/io_check.resc' 2>&1 | grep -v "^\s*$" | grep -vi "warning.*OC[12]PE\|Trying to change\|noisy"
