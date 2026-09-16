# hil-stm32:STM32 下位控制器(Renode)↔ Rust 橋接 ↔ 受控體

[docs/hil/](../../docs/hil/) 四篇的全部程式碼。一條指令跑閉環:

```bash
BUILD=1 ./run_loop.sh            # 第一次:建韌體與橋接,然後跑 6 s 預設腳本
./run_loop.sh                    # 之後直接跑
./run_loop.sh --negative bad-crc # 負對照:每個命令 CRC 弄壞,驗收必須轉紅(rc=1);enc-swap(編碼器 A/B 對調)、no-ramp(斜坡關掉,C9 紅)同理
./run_loop.sh --fault hang       # 安全 I/O 故障注入(2.0 s):hang(韌體死掉 → IWDG)、drv-fault(PC14 低)、bumper(撞 500 mm 的牆)、stall(輪子卡住)、no-ping(心跳停);C10 驗
./run_loop.sh --negative iwdg-off  # 同一個故障 + 關掉那一項防護(g_cfg.safety_mask):iwdg-off / drv-fault-off / bumper-off / stall-off / hb-off,C10 必須紅
tools/io_check.sh                # 不經橋接的韌體層驗收(renode/io_check.resc,A–J 十項)
PLANT=udp ./run_loop.sh          # 受控體改走 UDP(plant/fake_plant.py,另一容器);PLANT=tcp 同理走 TCP
PLANT=remote ./run_loop.sh       # 受控體在場域 GPU 主機(先 tools/isaac_plant_ctl.sh sync + start),自動開 ssh -L
WORLD=1 PLANT=remote ./run_loop.sh   # Isaac 版含牆與方塊(碰撞體)+ PhysX 射線雷射:那端要 WORLD=1 tools/isaac_plant_ctl.sh start;UPPER=nav2 / --fault 同樣可接
FW=freertos ./run_loop.sh        # 韌體換 FreeRTOS 版;TIMERFIX=1 換修正版 STM32_Timer(renode/upstream/)
./run_loop.sh --mode realtime    # Renode 自由跑,橋接每 5 ms 牆鐘取樣;Renode 容器自動 4 核(2 核會被 CFS 每 100 ms 凍 50 ms);QUANTUM=0.001 改同步量子;tools/rt_stats.py 算節拍統計
RECORD=1 ./run_loop.sh --fault bumper   # 跑完多產俯視圖錄影 out/run.mp4 + _topview.svg/png(RECORD_GIF=1 多 gif);既有 CSV 用 tools/topview.sh 補
./run_loop.sh --skew uniform:0.5 # lockstep 的時鐘偏斜實驗(決定性):每步 Renode 只推進 0.5×dt;stall:100:10 = 每 100 步一次 50 ms 停頓(35 篇 §5.1 第 4 點)
UPPER=ros ./run_loop.sh --seconds 45   # 上位換 ROS 2 Jazzy(ros:jazzy-ros-base 容器):base driver + 里程計閉環的方形
UPPER=nav2 ./run_loop.sh --seconds 90  # 上位換 Nav2(hil-nav2:jazzy,先 docker build -t hil-nav2:jazzy -f ros/Dockerfile.nav2 ros/):假雷射 + NavigateToPose 到 world.json 的 goal;C11
UPPER=nav2 ./run_loop.sh --seconds 60 --negative blind-scan   # 掃描全設最大距離 → 撞方塊 → C11 紅
UPPER=nav2 ./run_loop.sh --seconds 60 --fault hang --fault-at 8   # 路上底盤死掉 → IWDG 重啟 → driver 鎖住、取消 goal;C12;--negative no-latch 車又開走 → 紅
WORLD=1 UPPER=ros ROS_SCRIPT=run_scan_check.sh ./run_loop.sh --seconds 20   # 只驗 /scan 那條路
CAN=socketcan CANHUBFIX=1 ./run_loop.sh  # CAN 改走 Renode SocketCANBridge → 容器 netns 的 vcan0;CANHUBFIX=1 換修正版 CANHub(原版 lockstep 丟訊框)
./run_loop.sh --seconds 3 --script "0:200,0;2:0,0"
```

全部在 docker,不裝東西到系統。需要三個映像:`antmicro/renode:latest`(1.16.1)、一個帶 `arm-none-eabi-gcc` 的映像(環境變數 `ARM_IMAGE`)、`rust:1-slim-bookworm`。橋接零 crate,`cargo build --offline` 就能建。Renode 容器 `--network none`,橋接容器共用它的 netns;腳本只停自己起的容器。

## 目錄

| 路徑 | 內容 | 驗證 |
|---|---|---|
| `calib.json`(`encoder_source`) | `tim`:韌體從 TIM2/TIM4 encoder mode 讀 CNT(預設);`can`:第一版的 CAN 0x181 訊框。`ENC=hook\|gpio\|cnt` 選注入法 | 三種 lockstep 末端逐字相同 |
| `calib.json` | 韌體、橋接、受控體共用的唯一參數來源;`tools/gen_calib.py` 產 `firmware/calib.h`。`pwm_prescaler` 0 = 10 kHz 載波(Renode 自由跑 0.46×)、9 = 1 kHz(1.0×);不影響 lockstep 結果。`accel_limit_mm_s2` / `alpha_limit_mrad_s2`(韌體斜坡)、`ff_gain_q8`、`pi_kp_q8` / `pi_ki_q8`、`motor_tau_s` / `motor_accel_max_mm_s2` / `motor_deadband_duty`(三個受控體共用的馬達層) | 步階與 C9,[38 篇 §1.1](../../docs/hil/38-acceptance-and-failure-modes/README.md) |
| `tools/io_check.sh` | 在 Renode 容器裡跑 `renode/io_check.resc`:monitor 扮演板子(pull-up、PING、CNT、PC 腳),A–J 十項含 IWDG 重啟 | 實測 |
| `tools/topview.py`、`tools/topview.sh` | 閉環 CSV(+ `--scan-log` 的雷射 + world.json)→ 俯視圖錄影 mp4/gif + 靜態軌跡圖:真值車、odom 幽靈車、雷射、碰撞、旗標條、輪速、CCR;末幀 = CSV 末列 | 實測,[38 篇 §6.5](../../docs/hil/38-acceptance-and-failure-modes/README.md) |
| `tools/topcam_check.py` | Isaac 真實俯視相機的幀(`isaac_plant.py --topview`)對 CSV 真值:每幀底盤藍像素重心 → 世界座標 → 偏差(判準 < 0.2 m);幀序列編 mp4 | 60 幀最大 9 mm |
| `tools/rt_stats.py` | realtime CSV 的節拍統計:步距平均/最大、停頓次數、分段 Renode/牆鐘比、CCR 跳動 | 實測 |
| `tools/step_response.py`、`tools/tune_sweep.sh` | 從 CSV 算步階響應(上升、超調、±2% 帶、最大加速度);kp × ki 網格掃描,每格用橋接 `--cfg` 經 External Control 改 `g_cfg`,不重編韌體 | 實測 |
| `firmware/` | 裸機 STM32F4 韌體(C,無 HAL / libc)。`control.c`/`control.h` 是兩版共用的:USART1 框包 + CRC16、GPIO/TIM2-TIM4 encoder mode/TIM3 PWM/bxCAN/IWDG 的暫存器序列、5 ms 斜坡 + PI + 前饋、安全閘門(PC13 急停、PC14/15 驅動器故障、PC0 保險桿、堵轉、PING 心跳)、里程計、20 ms 回報、`g_dbg`/`g_cfg` 版面;`main.c` 只有 SysTick、USART1 ISR + ring buffer、主迴圈排程、餵狗 | Renode 1.16.1 實測;重構前後 lockstep CSV 逐 byte 相同 |
| `firmware-freertos/` | 同一台車的 FreeRTOS V11.3.1 版:`main-rtos.c` 只有三個 task + USART1 ISR + hooks,其餘連 `../firmware/control.c`(kernel 最小子集 vendor 在 `kernel/`,MIT);`FW=freertos ./run_loop.sh` | Renode 實測 ALL PASS;重構前後 CSV 逐 byte 相同 |
| `renode/` | vendor 的 1.16.1 `stm32f4.repl`(拿掉 `ApplySVD`)、開機腳本(`hilctl-common.resc` 共用;`hilctl-socketcan*.resc` 走 vcan)、`hil_hook.py`(IronPython:CAN/UART ↔ TCP,每筆注入回 ack;機器在跑時走 `HandleTimeDomainEvent`)、`boot_check` / `perf_check` / `perf_freerun` / `perf_timer_events` / `io_check` 驗收腳本 | 實測 |
| `renode/upstream/` | 給 Renode 上游的四項修正(`STM32_Timer` 三項、`NVIC` SysTick、`CANHub` 暫停丟訊框)的原版/patch/執行期載入版、探針、Robot、NUnit、不建 Renode 的驗證流程 | fork 三個 commit,NUnit 修正版 9/9、原版 2/9 |
| `tools/vcan_up.py` | 用 netlink 在目前 netns 建 vcan(不需要 iproute2);要 root + `NET_ADMIN` | 實測 |
| `bridge-rs/` | Rust 橋接:External Control client、hook 對端、上位協定、Fake/UDP/TCP 受控體、lockstep / realtime 迴圈、`--upper tcp-listen` 上位出口、`--cfg` 開機前寫 `g_cfg`、`--fault` 故障注入、十二項驗收 | 實測 |
| `ros/` | ROS 2 Jazzy 上位:`hil_base_driver.py`(/cmd_vel → 框包,odom → /odom + /tf,PING 10 Hz,flags → /hil/safety_flags,3801 → /scan + base_link→laser)、`square_client.py`(里程計閉環方形)、`hilproto.py`、`run_square.sh`;Nav2:`Dockerfile.nav2`(最小組合)、`nav2_params.yaml`、`run_nav.sh`、`nav_client.py`;`scan_check.py` | `ros:jazzy-ros-base` / `hil-nav2:jazzy` 實測 ALL PASS |
| `world.json`、`plant/world.py`、`tools/gen_map.py` | 假雷射與碰撞的世界(房間 + 方塊 + goal);Rust `world.rs` 同一份公式;地圖從同一份 JSON 產(預設只畫牆,方塊靠雷射) | 實測 |
| `plant/fake_plant.py` | UDP 版假受控體(Python),與 Rust 內建 `Fake` 同模型 | 實測 ALL PASS |
| `plant/isaac_plant.py` | Isaac Sim 6.0.1 版受控體(UDP / TCP;`--probe` 量驗收清單;`--world` 牆與方塊當碰撞體、`omni.physx` 射線當雷射、`collided` 旗標) | 場域 GPU 主機實測 ALL PASS(含 Nav2、五個 `--fault`);結論在檔尾與 [38 篇 §6、§6.4](../../docs/hil/38-acceptance-and-failure-modes/README.md) |
| `tools/remote.sh`、`tools/isaac_plant_ctl.sh` | 場域 GPU 主機的連線包裝(主機資訊從機密入口腳本推出)與受控體 sync/start/stop/log/load/probe/fetch(`WORLD=1` 帶世界、`TOPVIEW=1` 開真實俯視相機) | 實測 |
| `run_loop.sh` | 起 Renode 容器 → 橋接共用 netns → 跑 → 收 log → 停容器 | 實測 |

## 埠與訊號

| 埠 | 誰開的 | 走什麼 |
|---|---|---|
| 3500 | `emulation CreateExternalControlServer` | 時間推進、TIM3 CCR、GPIO 讀寫、`g_dbg` 讀(Renode External Control API) |
| 3600 | `hil_hook.py` | CAN 訊框雙向、UART 位元組雙向、ack、匯流排快照(13 bytes 一筆) |
| 3456 | `CreateServerSocketTerminal` | USART1 原始位元組,留作對照,橋接不用 |
| 3700 | `fake_plant.py` / `isaac_plant.py` | 受控體 UDP / TCP 文字協定 |
| 3800 | 橋接 `--upper tcp-listen` | 上位 UART 框包原樣進出(ROS 2 節點連這裡);注入 USART1 按 `uart_baud` 線速分批 |
| 3801 | 橋接 `--scan-listen` | 假雷射 `SCAN <seq> <n> r...` 一行一筆(受控體產生,不經 MCU;driver 發 /scan) |
| vcan0 | `tools/vcan_up.py` → Renode `SocketCANBridge` | `CAN=socketcan` 時 CAN 訊框走這裡而不是 3600;橋接用 `PF_CAN` raw socket |

## 產物

- `out/run.csv`:每步一行,32 欄(realtime 多 `wall_ms`;末欄 `collided` 只在有 world.json 時非零;設定點、量測、CCR、腳位、旗標、受控體位姿、tick、odom、CAN duty)
- `out/renode.log`、`renode/out/usart2.txt`(韌體 printer)
- 橋接 stdout:`[effect]` 生效證明六行、`[run]` 摘要(含每步四段牆鐘)、realtime 模式的 `[clocks]`、`[PASS]/[FAIL]` 十二項、`[result]`

## 實測數字(2026-09-15/16,docker 2 核,主機另有負載)

- 6 s 預設腳本 = 1200 步(現行 calib,2026-09-16,load 7–8):牆鐘 12.6 s,每步 10.5 ms,0.49× 實時;末端 900.8 / 0.4 / 0.8627,odom 對真值 0.8 mm / 0.7 mrad;裸機、FreeRTOS、hook/gpio/cnt 三種注入、vcan 路、UDP 受控體末端逐字相同
- 兩次跑 CSV 逐 byte 相同;`--negative bad-crc` → `bad_crc=300`、位移 0、C2 紅;`enc-swap` → 車跑到 5.3 m、C3/C8/C9 紅;`no-ramp` → 加速度 3000 vs 2520、C9 紅
- 安全 I/O 五項(`--fault`):韌體 1.995 s 死掉 → IWDG 2.995 s 重啟(+1000 ms,馬達以 34.5% duty 轉了 1 s);驅動器故障腳拉低 → 同一個控制步切掉;撞牆 +37 mm 停、倒車放行;堵轉 +355 ms 鎖住;PING 停 +200 ms HB_LOST、+495 ms 車停。五個 `*-off` 負對照各自紅(iwdg-off 的車跑到 1.6 m)
- 加減速(斜坡在韌體 1500 mm/s² / 4000 mrad/s²、前饋、馬達層在受控體):300 mm/s 步階假受控體超調 15.4% → 2.2%、最大加速度 9300 → 1820;Isaac 60% → 3.0%、91400 → 1900,C3 dθ 27.6 → 0.8 mrad
- 韌體 text 5092 B(含斜坡、前饋、encoder mode);WFI 讓 1 s 虛擬時間從 13.4 s 降到 1.83 s
- Isaac 6.0.1 受控體(遠端,`ssh -L`):每步 52 ms;odom 對真值 3.4 / 2.0 mm、0.028 rad;滑移 3.1%;兩次 CSV 逐 byte 相同(CPU 求解)
- 編碼器走 TIM encoder mode:注入 hook 2.9 / gpio 4.2 / cnt 2.8 ms/步;負對照 `--negative enc-swap` C3 紅;realtime 末端 = lockstep × Renode/牆鐘比(三次誤差 < 1%)
- CAN 走 vcan(`CAN=socketcan`):1.16.1 原版 `CANHub` lockstep 下 14/399 訊框到韌體;修正版 399/399、ALL PASS、末端與 hook 路逐字相同、兩次 CSV 相同、每步 10 ms
- 上位對安全旗標的反應(C12):路上 8.0 s 死機、9.0 s 重啟 → driver 看到 WDT_RESET 鎖住(cmd_vel=0、cancel goal),重啟後最遠再走 16 mm;`no-latch` 負對照 Nav2 重送 goal 從歸零的 odom 開走(1170 步在動)紅
- Nav2 in the loop(NavFn + DWB,map→odom 靜態):到達 goal 46 mm、碰撞 0、路徑 3.85 m;`blind-scan` 負對照撞方塊(第一次 @5.5 s)C11 紅;`default_server_timeout` 20 → 1000 ms 才過(load 15);上位 byte 按 115200 bps 分批注入(否則 ring buffer 溢位 114 B、4 個壞 CRC)
- ROS 2 方形閉環(0.6 m 邊、里程計判段):無斜坡版 lockstep 閉合 35 mm / 0.075 rad、realtime 70 mm / 0.141 rad;斜坡版 + 上位煞車模型含延遲 24 mm / −0.062 rad(lockstep);odom 對真值 5 mm / 1 mrad;ALL PASS
- `--mode realtime`(主機閒時,第一版韌體):每步 5.0 ms 牆鐘;`pwm_prescaler` 0 → Renode 0.46× 實時、車只走 1/3(C1–C8 PASS + `[warn]`);9 → 1.01×,末端對 lockstep 差 17–31 mm / 0.03–0.06 rad,兩次跑不同。斜坡版:C9 紅——真因不是負載或停頓,是 Renode 每 5 ms 牆鐘推進的虛擬時間抖 2–7 ms,韌體一個控制週期吃到 0 或 2 筆注入(`--skew` 拆開量,[35 篇 §5.1](../../docs/hil/35-hil-what-and-why/README.md) 第 4 點);「每 100 ms 停 50 ms」是 `--cpus 2` 的 CFS 配額,不是主機
