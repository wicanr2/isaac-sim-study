# hil-stm32:STM32 下位控制器(Renode)↔ Rust 橋接 ↔ 受控體

[docs/hil/](../../docs/hil/) 四篇的全部程式碼。一條指令跑閉環:

```bash
BUILD=1 ./run_loop.sh            # 第一次:建韌體與橋接,然後跑 6 s 預設腳本
./run_loop.sh                    # 之後直接跑
./run_loop.sh --negative bad-crc # 負對照:每個命令 CRC 弄壞,驗收必須轉紅(rc=1)
PLANT=udp ./run_loop.sh          # 受控體改走 UDP(plant/fake_plant.py,另一容器);PLANT=tcp 同理走 TCP
PLANT=remote ./run_loop.sh       # 受控體在場域 GPU 主機(先 tools/isaac_plant_ctl.sh start),自動開 ssh -L
FW=freertos ./run_loop.sh        # 韌體換 FreeRTOS 版;TIMERFIX=1 換修正版 STM32_Timer(renode/upstream/)
./run_loop.sh --mode realtime    # Renode 自由跑,橋接每 5 ms 牆鐘取樣;QUANTUM=0.001 改同步量子(預設 0.0001)
./run_loop.sh --seconds 3 --script "0:200,0;2:0,0"
```

全部在 docker,不裝東西到系統。需要三個映像:`antmicro/renode:latest`(1.16.1)、一個帶 `arm-none-eabi-gcc` 的映像(環境變數 `ARM_IMAGE`)、`rust:1-slim-bookworm`。橋接零 crate,`cargo build --offline` 就能建。Renode 容器 `--network none`,橋接容器共用它的 netns;腳本只停自己起的容器。

## 目錄

| 路徑 | 內容 | 驗證 |
|---|---|---|
| `calib.json` | 韌體、橋接、受控體共用的唯一參數來源;`tools/gen_calib.py` 產 `firmware/calib.h`。`pwm_prescaler` 0 = 10 kHz 載波(Renode 自由跑 0.46×)、9 = 1 kHz(1.0×);不影響 lockstep 結果 | — |
| `firmware/` | 裸機 STM32F4 韌體(C,無 HAL / libc):USART1 框包 + CRC16(中斷收訊)、TIM3 PWM、方向/致能 GPIO、PC13 急停、bxCAN 編碼器與狀態、5 ms PI、20 ms odom、WFI | Renode 1.16.1 實測 |
| `firmware-freertos/` | 同一台車的 FreeRTOS V11.3.1 版(三個 task + USART1 ISR;kernel 最小子集 vendor 在 `kernel/`,MIT);`FW=freertos ./run_loop.sh` | Renode 實測 ALL PASS |
| `renode/` | vendor 的 1.16.1 `stm32f4.repl`(拿掉 `ApplySVD`)、開機腳本、`hil_hook.py`(IronPython:CAN/UART ↔ TCP,每筆注入回 ack;機器在跑時走 `HandleTimeDomainEvent`)、`boot_check` / `perf_check` / `perf_freerun` / `io_check` 驗收腳本 | 實測 |
| `bridge-rs/` | Rust 橋接:External Control client、hook 對端、上位協定、Fake/UDP 受控體、lockstep 迴圈、八項驗收 | 實測 |
| `plant/fake_plant.py` | UDP 版假受控體(Python),與 Rust 內建 `Fake` 同模型 | 實測 ALL PASS |
| `plant/isaac_plant.py` | Isaac Sim 6.0.1 版受控體(UDP / TCP;`--probe` 量驗收清單) | 場域 GPU 主機實測 ALL PASS;結論在檔尾與 [38 篇 §6](../../docs/hil/38-acceptance-and-failure-modes/README.md) |
| `tools/remote.sh`、`tools/isaac_plant_ctl.sh` | 場域 GPU 主機的連線包裝(主機資訊從機密入口腳本推出)與受控體 start/stop/log/load | 實測 |
| `run_loop.sh` | 起 Renode 容器 → 橋接共用 netns → 跑 → 收 log → 停容器 | 實測 |

## 埠與訊號

| 埠 | 誰開的 | 走什麼 |
|---|---|---|
| 3500 | `emulation CreateExternalControlServer` | 時間推進、TIM3 CCR、GPIO 讀寫、`g_dbg` 讀(Renode External Control API) |
| 3600 | `hil_hook.py` | CAN 訊框雙向、UART 位元組雙向、ack、匯流排快照(13 bytes 一筆) |
| 3456 | `CreateServerSocketTerminal` | USART1 原始位元組,留作對照,橋接不用 |
| 3700 | `fake_plant.py` / `isaac_plant.py` | 受控體 UDP 文字協定 |

## 產物

- `out/run.csv`:每步一行,31 欄(設定點、量測、CCR、腳位、旗標、受控體位姿、tick、odom、CAN duty)
- `out/renode.log`、`renode/out/usart2.txt`(韌體 printer)
- 橋接 stdout:`[effect]` 生效證明四行、`[run]` 摘要(含每步四段牆鐘)、realtime 模式的 `[clocks]`、`[PASS]/[FAIL]` 八項、`[result]`

## 實測數字(2026-09-15,docker 2 核,主機另有負載)

- 6 s 預設腳本 = 1200 步:牆鐘 35.5 s,每步 29.6 ms,0.17× 實時;odom 對真值 0.9 mm / 0.9 mrad
- 兩次跑 CSV 逐 byte 相同;`--negative bad-crc` → `bad_crc=300`、位移 0、C2 紅
- 韌體 text 4732 B;WFI 讓 1 s 虛擬時間從 13.4 s 降到 1.83 s
- Isaac 6.0.1 受控體(遠端,`ssh -L`):每步 52 ms;odom 對真值 3.4 / 2.0 mm、0.028 rad;滑移 3.1%;兩次 CSV 逐 byte 相同(CPU 求解)
- `--mode realtime`(主機閒時):每步 5.0 ms 牆鐘;`pwm_prescaler` 0 → Renode 0.46× 實時、車只走 1/3(ALL PASS + `[warn]`);9 → 1.01×,末端對 lockstep 差 17–31 mm / 0.03–0.06 rad,兩次跑不同
