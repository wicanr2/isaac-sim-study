# 38 · 驗收與失敗形態:先閉環,再換 Isaac

閉環跑完印出 `ALL PASS`,問題才開始:這八個綠燈各證明了什麼?哪一個在系統壞掉時**一定會**變紅?兩次跑的結果一樣是「決定性」還是「剛好」?這一篇把判準、負對照、決定性、以及這次踩到的每一種失敗形態攤開,最後是把受控體換成 Isaac Sim 6.0.1 時七件事各量到什麼。

> **驗證狀態**:§1–§5 在本機實測(Renode 1.16.1 + Rust 橋接 + 假受控體,docker 2 核,主機另有負載 load ≈ 4/14,2026-09-15)。§6 在場域 GPU 主機實測(Isaac Sim 6.0.1 pip 版,PhysX、CPU 求解、TGS;Renode 與橋接留在本機,受控體經 `ssh -L` 隧道,同日)。

## 1. 九項判準,在開跑前寫死

[`run_loop.sh`](../../../examples/hil-stm32/run_loop.sh) 一條指令:起 Renode 容器(`--network none`)、橋接容器共用它的 netns、跑完只停自己起的那一個。橋接開頭先印生效證明:

```
[effect] encoder: calib=tim inject=hook
[effect] renode ec=127.0.0.1:3500 hook=127.0.0.1:3600 machine=hilctl boot_ms=100 t0_us=100000
[effect] g_dbg@0x2000011c magic=0x48494c31 (ok) init_err=0
[effect] mode=lockstep plant=fake dt_ms=5 steps=1200 report_every=4 script="0:0,0;0.5:300,0;3.5:0,600;5:0,0" negative=none slip=0
[effect] tim3 ARR=999 (calib pwm_arr=999) track=300 circ_um=314159 tpr=4096
[effect] g_cfg@0x20000000 kp_q8=256 ki_q8=6 accel_mm_s2=1500 ff_q8=256 alpha_mrad_s2=4000 (calib 預設)
```

每一行都是「這個變數進了系統」的證據:magic 對表示讀的是這支韌體、`ARR` 讀回值對上 calib 表示韌體吃的是同一份參數、`g_cfg` 那行是從 SRAM **讀回來**的增益與斜坡(`--cfg` 覆蓋之後也是讀回值,不是命令列的值)。

預設腳本:0.5 s 起 v = 300 mm/s 走 3 s,再 w = 600 mrad/s 轉 1.5 s,然後停;共 6 s = 1200 步。九項判準(2026-09-16,現行 calib:斜坡 1500 mm/s² / 4000 mrad/s²、kp 256、ki 6、前饋 100%、馬達層 τ 50 ms):

| # | 判準 | 抓什麼失敗 | 實測 |
|---|---|---|---|
| C1 | Renode 時間 == steps × dt | `run_for` 少跑或多跑、時鐘漂移 | 6,000,000 vs 6,000,000 |
| C2 | 腳本有命令時車有動(路徑長 > 100 mm) | 整條命令鏈斷掉 | 901.9 mm |
| C3 | 韌體 odom 對受控體真值 | 編碼器方向、換算、里程計 | dx 0.8、dy 0.4 mm、dθ 0.7 mrad(容差 25 mm + 2% 路徑長 / 30 mrad) |
| C4 | 每筆 CAN 狀態框的 duty == 同一時刻的 CCR 快照 | 兩條獨立管道不一致 | 305 筆,0 不符 |
| C5 | odom 回報數 ≥ 90% 期望 | UART 出口掉資料 | 305 / 300 |
| C6 | 韌體 `bad_crc` == 橋接送壞的數 | CRC 檢查沒在跑 | 0 vs 0 |
| C7 | 韌體收到的 cmd == 送出且未壞的數 | UART 入口掉資料 | 300 vs 300 |
| C8 | TIM 模式:CNT == 受控體 tick(mod 2^16)且韌體累計 == 前一步 tick;CAN 模式:收到的訊框 == steps − 1 | 編碼器注入掉資料;一步延遲 | CNT 10057/13431 == plant;fw == 前一步 |
| C9 | 受控體的輪加速度 ≤ (accel + alpha × 輪距/2) × 1.2 | 斜坡沒生效(韌體沒讀 `g_cfg`、算錯單位) | 2257 vs 2520 mm/s² |

任何一項 FAIL,橋接以非零碼離開。判準寫在程式裡而不是事後看 log 決定,理由同 [30 篇](../../common/30-acceptance-probes-and-preregistration/README.md)。末端位姿 x 900.8、y 0.4、θ 0.8627 是這份 calib 下的參考值,裸機與 FreeRTOS 兩版韌體、三種編碼器注入法、UDP 受控體、vcan 路都要對上它(§3、[36 篇](../36-stm32-firmware-on-renode/README.md) §3.1、[39 篇](../39-freertos-firmware-in-the-loop/README.md) §3)。

負對照有三個,每一個只差一個旗標:`--negative bad-crc`(§2)、`--negative enc-swap`(編碼器 A/B 對調:韌體量到負速度,PI 正回饋把車推到 5.3 m,C3、C8、C9 紅)、`--negative no-ramp`(§1.1)。

### 1.1 加減速:斜坡在韌體、馬達層在受控體

第一版韌體的 PI 直接追上位的階躍,受控體的馬達是「duty 乘滿速再一階趨近」——沒有死區、沒有扭矩上限。300 mm/s 的步階在假受控體上升 40 ms、超調 15%、最大加速度 9300 mm/s²;在 Isaac 上 `DriveAPI` 直接吃 duty,超調 60%、最大加速度 91400 mm/s²,在 ±2% 帶外晃了 525 ms。這兩個數字都不是「車」的行為,是「沒有動態限制的控制器 + 沒有物理上限的馬達」的行為。改了三件事,每一件放在它該在的那一層:

<p align="center"><img src="../../img/hil-ramp-and-motor-layer.svg" width="860" alt="命令到輪速的鏈:斜坡與 PI 在韌體、馬達層在受控體;兩張步階圖是假受控體與 Isaac 的前後對照"></p>

1. **斜坡在韌體。** `cmd_vel` 進來先存,每個控制步 v 與 w **各自**往命令走一步(`accel × 5 ms`、`alpha × 5 ms`),再拆成兩輪設定點。一開始寫成「兩輪各自限」,方形閉環的每一段直線偏航 +0.16 rad——從原地轉切到直走時左輪要從 −90 加到 300、右輪從 +90 加到 300,各自限速的話兩輪不同時到,車在過渡期畫弧。v/w 各限就沒有這個問題:w 的斜坡把兩輪的差同步收掉。另外設定點歸零時清積分——不清的話下一段起步時上一段留下的積分會先推一下(方形閉合 126 → 63 mm,§6.1)。
2. **前饋在韌體。** PI 之前加 `sp × 滿 duty / 滿速`;PI 只補模型誤差,kp 從 128 → 256、ki 13 → 6(`tools/tune_sweep.sh` 掃 kp × ki 網格,每一格經 External Control 寫 `g_cfg`,不重編)。
3. **馬達層在受控體,三份實作同一組公式。** 死區(|duty| < 5% 不動,之後線性到滿速)、一階趨近(τ 50 ms)、每步速度變化 ≤ 3000 mm/s² × dt(電流限制)。Rust `Fake`、`fake_plant.py`、`isaac_plant.py` 各自實作一次,Isaac 那份放在算 `DriveAPI` 目標速度之前——PhysX 的關節驅動本身沒有這些限制,它會把任何目標在一步內追到。參數在 `calib.json`(`motor_tau_s`、`motor_accel_max_mm_s2`、`motor_deadband_duty`),改公式要三處一起改。

前後(`tools/step_response.py`,`plant_vl` 對 0.5 s 的 300 mm/s 步階看 3 s):

| | 假受控體 前 → 後 | Isaac 6.0.1 前 → 後 |
|---|---|---|
| 上升時間 10→90% | 40 → 155 ms | 0 → 155 ms |
| 超調 | 15.4% → 2.2% | 60.0% → 3.0% |
| 進入 ±2% 帶 | 390 → 365 ms | 525 → 475 ms |
| 最大 \|加速度\| | 9300 → 1820 mm/s² | 91400 → 1900 mm/s² |
| 6 s 腳本 C3(dx / dy / dθ) | 0.0 / 0.9 mm / 0.9 mrad → 0.8 / 0.4 / 0.7 | 3.4 / 2.0 mm / 27.6 mrad → 0.8 / 1.0 / 0.8 |

上升 155 ms 是斜坡的值(300 / 1500 = 200 ms 到頂,10→90% 是其中 160 ms),不是 PI 的值;兩個受控體一樣,因為限制在韌體。Isaac 的 dθ 從 27.6 mrad 掉到 0.8:它的里程計誤差原本主要是轉向段的加速度撞擊——輪子在 5 ms 內從 0 到滿速,球形輪對地面打滑,而假受控體沒有這件事,所以兩者原本差 30 倍、現在同一級。

**C9 與負對照。** 判準是「受控體的輪加速度 ≤ (accel + alpha × 輪距/2) × 1.2」:v 與 w 同時起坡時一個輪子吃到兩者的和(1500 + 4000 × 0.15 = 2100),PI 追斜坡的瞬態量到 +7%,留 20%。`--negative no-ramp` 經 External Control 把 `g_cfg` 的 accel 與 alpha 寫 0(`[effect]` 那行讀回 0,韌體同一支),C9 按 calib 的 2100 驗:max 3000 vs 2520,紅——3000 是馬達層的上限,斜坡關掉時受控體撞到的是它。容差不能是 1.5:2100 × 1.5 = 3150 > 3000,負對照會綠。這是 §2 那條規則的另一個用法:負對照不只驗「測試有效」,還會**卡住容差**。

`--mode realtime` 下三項換成一致性版本:C1「0 < Renode 時間 ≤ 牆鐘 + 5%」(Renode 追牆鐘的節拍以量子為單位,量到領先最多 +1.8%)、C5 的期望值用 Renode 時間除以 20 ms(odom 是韌體按它的時間送的)、C8「≥ 90% steps」(沒有 `run_for`,一步延遲的等式不成立)。realtime 下九項全綠**不代表**車走對了——Renode 跑不到實時時,閉環速度會安靜地低到 ratio 倍,判準抓不到,見 [35 篇](../35-hil-what-and-why/README.md) §5.1。

C3 的容差寫成三項相加:韌體數值誤差(25 mm / 0.03 rad;0.9 mm 是韌體用 5 項 Taylor 的 sin/cos、以 float 積分 1200 步的誤差)+ 里程計對真值的系統性差(2% 距離)+ 受控體的接觸滑移(`--slip` × 距離、`--slip` × |θ|)。假受控體 slip = 0;Isaac 實測轉向滑移 3.1%、直行 0.5%,用 0.05——這個數字是先在 §6 量到、再寫回判準的,不是看結果調的。

## 2. 負對照:全綠證明不了測試在驗東西

`./run_loop.sh --negative bad-crc` 把每個 cmd_vel 的最後一個 byte 反相:

```
[run] fw ... cmd_frames=0 enc_frames=1199 bad_crc=300 rx_overflow=0
[run] plant  x=0.0 y=0.0 th=0.0000
[FAIL] C2 車有動(腳本有命令時) — plant 位移 0.0 mm
[PASS] C6 韌體 bad_crc == 橋接送壞的數 — 300 vs 300
[PASS] C7 韌體收到的 cmd == 送出且未壞的數 — 0 vs 0
[result] SOME FAIL
```

紅在對的地方(C2),而且 C6 說明了**為什麼**紅:300 個壞框包全部被韌體擋下,一個都沒漏。沒有 C6 的話,「車不動」與「UART 根本沒接上」在 C2 上長得一樣。

負對照要跟正對照用同一條指令、同一份腳本,只差一個旗標。它不是額外的測試,是「這套測試有效」的證據。

## 3. 決定性:逐 byte 比,而且是每個實作各自成立

同一腳本跑兩次,1201 行 CSV(每步 31 個欄位;realtime 多一欄 `wall_ms`:設定點、量測、CCR、腳位、旗標、受控體位姿、tick、odom、CAN duty)**逐 byte 相同**。靠的是 [37 篇](../37-bus-signal-bridging/README.md) §3 的 ack:每筆注入確認進了週邊才推進時間。

但換一個「同一個模型」的實作就不一樣了。`plant/fake_plant.py`(Python,走 UDP)與橋接內建的 Rust `Fake` 是同一組公式、同一份 calib:

- 末端位姿:x 900.8、y 0.4 相同,θ 0.8627 vs 0.8628
- 1200 步 × 8 個欄位裡有 **811 個不同**,第一個在第 446 步:`ticks_l` 6310 vs 6309

差在 `floor(s / 周長 × 4096)` 的邊界——兩種語言的浮點運算在最後一位偶爾不同,落在整數邊界上就差 1 tick;馬達層的死區與限幅多了幾次乘除,邊界撞到的次數也跟著多。**決定性是每個實作各自成立的性質**;要跨實作比對,用容差,不用相等。

## 4. 步邊界取樣的盲點

C4 第一版拿「這一步結束時讀到的 CCR」跟 CAN 訊框裡的 duty 比,305 筆有 2 筆不符。看那兩筆:

```
step 376: 前一步 CCR=297,這一步 CCR=298,下一步 CCR=306;CAN 框說 305
```

<p align="center"><img src="../../img/hil-step-boundary-sampling.svg" width="860" alt="一個 5 ms 視窗裡兩個邊界 tick 都算進來,橋接在邊界只看到最後的 CCR"></p>


305 這個值在任何一個步邊界都沒出現過。原因:韌體的控制步由 SysTick 的 1 ms 中斷驅動,5 ms 一次;`run_for` 的邊界剛好落在 1985 ms 這種整數 tick 上時,那個 tick 的控制步這次算進本步、下次算進下一步。於是某個 5 ms 視窗裡跑了**兩次**控制步,第一次算出 305 並發了 CAN 框,第二次算出 298 蓋掉 CCR,橋接在邊界只看得到 298。`ctrl_steps` 總數對(6100 ms 剛好 1220 次),只是分佈在視窗之間有抖動。

這是 [35 篇](../35-hil-what-and-why/README.md) §5「三個時鐘」的具體實例:韌體的 tick 與橋接的步是兩個時鐘,對齊只在平均意義上成立。解法不是放寬判準,是**在事件發生的當下取樣**——hook 的 `FrameSent` handler 在同一個模擬時刻讀 CCR 附回來,C4 改用它之後 305/305 一致。

一般化:橋接在步邊界看到的是狀態的採樣,不是狀態的歷史。要比「同一時刻」的兩個量,取樣要發生在模擬器裡面。

## 5. 這次踩到的失敗形態

每一條:症狀 → 真因 → 什麼判準抓得到。

| 症狀 | 真因 | 抓到它的 |
|---|---|---|
| 握手回 FATAL `Encountered unknown command 0x00` | 腳本用 `echo >/dev/tcp/...` 探埠,換行字元被 server 當握手第一 byte | 握手錯誤要讀出 server 的訊息,不能只看 code |
| `magic=0x00000000`、`ARR=65535` | `LoadELF` 後機器暫停,`main` 沒跑,SRAM 全零、暫存器是重置值 | 生效證明;修法是先 `run_for` 開機 100 ms |
| CAN 注入「成功」但 FIFO 空 | `FMR` 寫 `1` 把 `CAN2SB` 清成 0,bank 0 歸 CAN2,訊框被靜默濾掉 | 探針讀 `RF0R.FMP0`;log 開 Debug 看 `dropped by filter` |
| 注入呼叫丟例外 | `FrameReceived` 事件帶兩個參數,handler 只收一個 | 例外訊息本身;事件簽章先 `print dir()` 查 |
| monitor 函式型別錯 | 數字參數被轉 int、週邊名被轉物件 | 字串加引號;函式收物件 |
| C4 2/305 不符 | 一個視窗兩次控制步,中間值在邊界看不到 | 事件時刻快照(§4) |
| 1 s 虛擬時間要 13.4 s | MMIO 輪詢是 Renode 最貴的操作 | 量指令數;WFI + 中斷收訊([36 篇](../36-stm32-firmware-on-renode/README.md) §5) |
| Isaac 受控體第一步就 `AttributeError` | 6.0.1 的 PhysX 介面沒有 `update` | 列 `dir()` 找到 `simulate/fetch_results`,不猜 |
| 車在 5 ms 內以 2.9 m/s 往上飛(32 篇「被彈飛」的形狀) | 地面的 `xformOpOrder` 寫成 [scale, translate]:USD 第一個列的是最外層,-0.05 的平移被 z 的 0.1 縮成 -0.005,**地面頂面在 +45 mm**,輪子起始陷入 45 mm | 探針印地面 bbox 頂面與靜止後的 z(所有東西都停在 +45 mm 就是線索) |
| 位置對到 3 mm、航向差一個正負號 | Gf 矩陣是 row-vector 慣例,yaw 用了 column-vector 的索引 | 右輪 tick 比左輪多 → 左轉為正;韌體對、受控體錯 |
| 閉環跑完 `run_loop.sh` 收不掉 | 背景 `ssh -L` 繼承了腳本的 stdout 管線,kill 到包裝 shell 而不是 ssh | 隧道輸出導檔案、`exec` 起 ssh 讓 PID 就是它 |
| CAN 走 vcan 時 `enc_frames` 14/399,無任何 warning | `CANHub` 在 `RunFor` 之間的暫停期把主機來的訊框丟掉 | C8 紅;Debug log 數「Received from」與韌體收到的差;修在 hub([37 篇](../37-bus-signal-bridging/README.md) §4) |
| 同上,修了 hub 還是 331/399 | `i @CANHub_Fixed.cs` 執行期編譯要幾秒,External Control 已經開、橋接已經在推進,前幾十步的訊框沒人收 | 出口全部就位**之後**才開 External Control(`hilctl-common.resc` 的順序) |

| realtime 下同一腳本末端差 lockstep 8–14%,五次各不同 | 橋接守不住 5 ms 牆鐘節拍(步距 4.8–5.3 ms、停頓到 50 ms),編碼器訊框跟著步走,韌體假設每筆 = 5 ms | `wall_ms` 欄對 `plant_x`;差與步距同向。開放:訊框收到時打 SysTick 微秒戳([35 篇](../35-hil-what-and-why/README.md) §5.1) |

共同點:**每一個的第一眼症狀都指向別的地方**——握手失敗像版本不合、SRAM 全零像位址錯、FIFO 空像模型缺口、C4 不符像韌體回報錯、彈飛像腳輪或質量。每一個都是先讀原始碼或加一個更近的觀測點才看到真因;彈飛那一個,近一點的觀測點是「靜止後停在哪個高度」。

## 6. 換成 Isaac Sim 6.0.1:七件事各量到什麼

[`plant/isaac_plant.py`](../../../examples/hil-stm32/plant/isaac_plant.py) 實作同一個受控體協定(TCP 版,因為 `ssh -L` 只轉 TCP):用 `pxr` / `UsdPhysics` 直接建一台差速車(Mesh 盒底盤 + 兩個球形輪 + 球關節腳輪),`DriveAPI` 的 angular 速度目標當馬達,`omni.physx` 手動步進,物理步長綁 calib 的 5 ms。它刻意不用 OmniGraph([31 篇](../../fleet/31-omnigraph-and-ros2-bridge-truth/README.md) §5:headless 下 `world.step(render=False)` 不 tick action graph)、也不依賴 `isaacsim.core.api` 或 `isaacsim.core.experimental` 任一邊([01 篇](../../common/01-install-and-run-modes/README.md) §3 的命名空間搬家)。

拓撲:Renode 與橋接留在本機 docker,只有受控體在 GPU 主機,`run_loop.sh` 的 `PLANT=remote` 自動開隧道。每一輪都重啟受控體——它的位姿與 tick 跨連線累積,重啟才是同一個起點。

| # | 要驗的 | 量到的(2026-09-15,`--probe` 模式) |
|---|---|---|
| 1 | 手動步進介面 | 6.0.1 的 PhysX 介面**沒有 `update`**(第一次跑就 `AttributeError`)。正確路徑:`IPhysxSimulation.attach_stage(stage_id)` → `simulate(dt, t)` → `fetch_results()`;timeline 不 play,否則 Kit 每個 update 自己再步一次 |
| 2 | 關節角讀法 | `state:angular:physics:position` **不會被寫回**,步進前後都不存在。改從 `fetch_results` 寫回的 xform 算輪子相對底盤繞 Y 的角、跨步展開——那是物理輸出,滑移都在裡面 |
| 3 | 物理步長 | `timeStepsPerSecond=200` 讀回 200,`SimulationManager.get_physics_dt()` = 0.005。標 Deprecated 但生效 |
| 4 | 彈飛 | 修正後靜止 1 s 底盤 z = 50.0 mm(建模 50.0)。**曾經彈飛**,真因見 §5 |
| 5 | `targetVelocity` 單位 | 設 360 跑 1 s → 輪角 6.235 rad(99.2%,drive 有落後);底盤前進 302.1 mm 對輪周 311.8 mm → **滑移 3.1%**,與 [32 篇](../../fleet/32-differential-drive-vehicle-model/README.md)的 2~3% 同量級 |
| 6 | C1–C9 | **ALL PASS**:C3 dx 0.8 / dy 1.0 mm、dθ 0.0008 rad(容差 87.8 mm / 0.0737 rad,slip 0.05);步階超調 3.0%、最大加速度 1900 mm/s²;負對照位移 0、C2 紅。每步 52 ms 牆鐘(隧道約 +19 ms、Isaac 步進約 +3 ms)。馬達層加入前是 3.4 / 2.0 mm、0.0276 rad,超調 60%、加速度 91400——差在轉向段輪子瞬間到滿速時的打滑(§1.1) |
| 7 | 決定性 | 兩次跑 CSV **全部欄位逐 byte 相同**。`is_gpu_dynamics_enabled()` = True 而 `get_physics_sim_device()` = cpu——兩個值都記,決定性在這個組合下成立;GPU 求解沒測 |

### 6.1 上位換成 ROS 2 Jazzy:方形閉環

ROS 2 在這個拓撲裡的位置是**上位**。[`ros/hil_base_driver.py`](../../../examples/hil-stm32/ros/hil_base_driver.py)(rclpy 7.1.11,`ros:jazzy-ros-base`)訂 `/cmd_vel`、每 20 ms 送一個 cmd_vel 框包;收 odom 框包發 `/odom` 與 `/tf`(odom → base_link)。它對橋接講的是韌體那份序列協定([`ros/hilproto.py`](../../../examples/hil-stm32/ros/hilproto.py),與 Rust 側同一組已知答案),橋接開 `--upper tcp-listen:0.0.0.0:3800` 之後在上位側只當一條序列線:byte 原樣進 USART1、原樣送回。Isaac 那側不需要 ros2 bridge——受控體介面是 TCP/UDP 文字協定,不是 topic。

閉環用 [`ros/square_client.py`](../../../examples/hil-stm32/ros/square_client.py):訂 `/odom`、發 `/cmd_vel`,四條 0.6 m 的邊、四個 +90° 的角,**每一段的結束由里程計判斷,不是計時**——所以它不在乎 Renode 跑幾倍實時,慢只是等久一點。第一版韌體(無斜坡)下 `UPPER=ros ./run_loop.sh --seconds 40`:

| | lockstep | realtime |
|---|---|---|
| C1–C8 | ALL PASS | ALL PASS |
| 上位送出 / 韌體收到的 cmd 框包 | 1733 / 1733 | 1257 / 1257 |
| 回到起點的閉合誤差(odom) | 35 mm、0.075 rad | 70–86 mm、0.14–0.15 rad(兩次) |
| 受控體真值末端 (x, y, θ) | 28.2, −23.2, 6.359 | 49.1, −50.5, 6.425 / 67.0, −55.2, 6.429 |
| odom 末端 | 24, −26, 6.358 | 45, −53, 6.424 / 63, −58, 6.429 |
| 牆鐘(方形本身) | 36.5 s(Renode 20 s) | 28.1 / 21.4 s |

<p align="center"><img src="../../img/hil-square-closure.svg" width="860" alt="方形閉環的軌跡與每段結束位姿;閉合誤差拆成轉角過頭與直線段偏航"></p>


閉合誤差來自兩個地方,從每段結束時印的位姿可以拆開看(lockstep):轉角**過頭** 0.000 / 0.016 / 0.007 / 0.012 rad——里程計 20 ms 一筆、命令 20 ms 一筆,0.6 rad/s 下每一筆就是 0.012 rad,「夠了」的判斷最多晚兩筆;直線段**偏航** +0.018 / +0.003 / +0.012 rad——剛從原地轉切到直走,兩輪的 PI 從不同的速度起步,車在加速的前幾十毫秒還在轉。realtime 下同樣兩項各自變大(轉角最多 0.029、直線段最多 0.032):延遲以牆鐘計,Renode 又領先 1%。odom 對真值(4 mm、1 mrad)兩者都在 C3 容差內——閉合誤差是**控制與延遲**的事,不是里程計的事,兩個數字要分開看。

**加了斜坡之後閉合先變差再變好。** 下位有了加減速,「命令歸零」到「車停住」之間多了一段距離;上位不知道這件事就會過頭。每一步改了什麼、閉合怎麼變(lockstep,`--seconds 45`;每一列只比上一列多一件事):

| 版本 | 閉合(odom) | 每個轉角(轉 + 停,對 π/2) |
|---|---|---|
| 無斜坡(上表) | 35 mm、+0.075 rad | 過頭 0.000–0.016 |
| 斜坡,兩輪各自限 | 277 mm | 每段直線偏航 +0.16:兩輪從不同速度起步、各自限速就不同時到 |
| 斜坡改 v/w 各自限 | 228 mm | 轉→直的過渡不再畫弧;但上位仍在 odom 到 0.6 m 才停,車再滑 v²/2a |
| 上位提前 v²/2a、w²/2α 煞車,段間等 odom 速度歸零 | 126 mm | |
| 韌體設定點歸零時清積分 | 63 mm、+0.145 rad | 過頭 +0.028–0.040:煞車模型少了馬達 τ 與 odom/cmd 各一筆的延遲 |
| 上位煞車距離加 v × lag(`lag_s` 0.08 = τ 50 ms + 兩筆 20 ms 量級) | **24 mm、−0.062 rad** | 不足 0.011–0.026,散布 0.015 ≈ 一筆 odom(0.012) |

末端真值 (x, y, θ) = (−18.0, 16.4, 6.222)、odom (−13, 20, 6.221),odom 對真值 5 mm / 1 mrad,C1–C9 全綠。最後一列的殘差已經是一筆 odom 的量化,再往下要改的是上位的取樣率或用 odom 速度外推,不是下位。這一段的教訓是**上位的煞車模型要包含下位的動態**——斜坡加速度、馬達時間常數、回報週期——這些在真車上是驅動器手冊與底盤韌體的參數,上位的人拿不到就會在方形上看到 +0.03 rad/角。

C2 因為這個場景改成量**路徑長**而不是首尾位移:方形走完位移 37 mm,路徑長 2.4 m;C3 的容差也改用路徑長(里程計誤差跟著走過的距離累積)。預設腳本下兩者只差 15 mm(轉彎過渡的弧),數字不變。

## 7. 建議的分階段

不要一開始就接 Isaac。順序:

| 階段 | 迴路裡有什麼 | 判準 |
|---|---|---|
| 骨架 | 橋接 ↔ 假受控體,**沒有韌體**(橋接自己灌固定 duty) | 同一輸入跑兩次逐 byte 相同 |
| 韌體 | 加 Renode 裡的韌體 | C1–C9 全綠;負對照轉紅 |
| 受控體換 UDP | 假受控體改另一個行程 | 與內建版末端一致、途中容差內 |
| Isaac | 受控體換 `isaac_plant.py` | §6 七項;C3 容差重定 |
| ROS 2 上位 | 上位換 rclpy 節點,閉環由里程計判斷 | C1–C9 全綠;閉合誤差與 odom 誤差分開報;上位的煞車模型要含下位的斜坡與馬達延遲 |
| 實板 | Renode 換實體 STM32,橋接換實板後端 | 全部重跑;**時序與最壞延遲在這裡才算數** |

每一階段結束才往下一階段,每一階段都留下可重跑的指令與 CSV。這一區做到第五階段。

## 8. 檢查清單

- [ ] 判準寫在程式裡,開跑前定好,任一 FAIL 非零碼離開
- [ ] 生效證明每個變數一行
- [ ] 有一個負對照,跟正對照只差一個旗標,而且紅在對的地方
- [ ] 決定性用逐 byte 比驗過;跨實作用容差
- [ ] 要比「同一時刻」的兩個量,取樣在模擬器裡做
- [ ] 每個失敗形態記下「第一眼像什麼」與「真因是什麼」
- [ ] 換受控體之前,容差重新寫好
- [ ] 報告裡分清楚哪些階段做到了、哪些數字要等實板

---

延伸閱讀:[35 HIL 是什麼](../35-hil-what-and-why/README.md)、[36 STM32F4 韌體在 Renode 上開機](../36-stm32-firmware-on-renode/README.md)、[37 匯流排訊號串接](../37-bus-signal-bridging/README.md)、[30 驗收探針與實驗預先登記](../../common/30-acceptance-probes-and-preregistration/README.md)、[27 失效模式分類學](../../common/27-failure-mode-taxonomy/README.md)。
