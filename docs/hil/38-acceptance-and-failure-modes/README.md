# 38 · 驗收與失敗形態:先閉環,再換 Isaac

閉環跑完印出 `ALL PASS`,問題才開始:這八個綠燈各證明了什麼?哪一個在系統壞掉時**一定會**變紅?兩次跑的結果一樣是「決定性」還是「剛好」?這一篇把判準、負對照、決定性、以及這次踩到的每一種失敗形態攤開,最後列出把受控體換成 Isaac Sim 6.0.1 時要驗的七件事。

> **驗證狀態**:§1–§5 全部在本機實測(Renode 1.16.1 + Rust 橋接 + 假受控體,docker 2 核,主機另有負載 load ≈ 4/14,2026-09-15)。§6 的 Isaac 腳本**未在本 repo 環境驗證**。

## 1. 八項判準,在開跑前寫死

[`run_loop.sh`](../../../examples/hil-stm32/run_loop.sh) 一條指令:起 Renode 容器(`--network none`)、橋接容器共用它的 netns、跑完只停自己起的那一個。橋接開頭先印生效證明:

```
[effect] renode ec=127.0.0.1:3500 hook=127.0.0.1:3600 machine=hilctl boot_ms=100 t0_us=100000
[effect] g_dbg@0x200000f8 magic=0x48494c31 (ok) init_err=0
[effect] plant=fake dt_ms=5 steps=1200 report_every=4 script="0:0,0;0.5:300,0;3.5:0,600;5:0,0" negative=none
[effect] tim3 ARR=999 (calib pwm_arr=999) track=300 circ_um=314159 tpr=4096
```

每一行都是「這個變數進了系統」的證據:magic 對表示讀的是這支韌體、`ARR` 讀回值對上 calib 表示韌體吃的是同一份參數。

預設腳本:0.5 s 起 v = 300 mm/s 走 3 s,再 w = 600 mrad/s 轉 1.5 s,然後停;共 6 s = 1200 步。八項判準:

| # | 判準 | 抓什麼失敗 | 實測 |
|---|---|---|---|
| C1 | Renode 時間 == steps × dt | `run_for` 少跑或多跑、時鐘漂移 | 6,000,000 vs 6,000,000 |
| C2 | 腳本有命令時車有動(位移 > 100 mm) | 整條命令鏈斷掉 | 902.0 mm |
| C3 | 韌體 odom 對受控體真值 | 編碼器方向、換算、里程計 | dx 0.0、dy 0.9 mm、dθ 0.9 mrad(容差 25 mm + 2% 距離 / 30 mrad) |
| C4 | 每筆 CAN 狀態框的 duty == 同一時刻的 CCR 快照 | 兩條獨立管道不一致 | 305 筆,0 不符 |
| C5 | odom 回報數 ≥ 90% 期望 | UART 出口掉資料 | 305 / 300 |
| C6 | 韌體 `bad_crc` == 橋接送壞的數 | CRC 檢查沒在跑 | 0 vs 0 |
| C7 | 韌體收到的 cmd == 送出且未壞的數 | UART 入口掉資料 | 300 vs 300 |
| C8 | 韌體收到的編碼器訊框 == steps − 1 | CAN 入口掉資料;一步延遲 | 1199 vs 1199 |

任何一項 FAIL,橋接以非零碼離開。判準寫在程式裡而不是事後看 log 決定,理由同 [30 篇](../../common/30-acceptance-probes-and-preregistration/README.md)。

C3 的容差值得說明:0.9 mm 是韌體用 5 項 Taylor 的 sin/cos、以 float 積分 1200 步的誤差;假受控體用 f64 精確積分。換成 Isaac 之後,接觸滑移會讓這個數字變成幾十 mm([32 篇](../../fleet/32-differential-drive-vehicle-model/README.md)實測 2~3%),容差要跟著改,而且要在換之前寫好。

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

同一腳本跑兩次,1201 行 CSV(每步 31 個欄位:設定點、量測、CCR、腳位、旗標、受控體位姿、tick、odom、CAN duty)**逐 byte 相同**。靠的是 [37 篇](../37-bus-signal-bridging/README.md) §3 的 ack:每筆注入確認進了週邊才推進時間。

但換一個「同一個模型」的實作就不一樣了。`plant/fake_plant.py`(Python,走 UDP)與橋接內建的 Rust `Fake` 是同一組公式、同一份 calib:

- 末端位姿相同:x 902.0、y −0.9、θ 0.9019
- 1200 步 × 8 個欄位裡有 **462 個不同**,第一個在第 227 步:`ticks_l` 2526 vs 2525

差在 `floor(s / 周長 × 4096)` 的邊界——兩種語言的浮點運算在最後一位偶爾不同,落在整數邊界上就差 1 tick。**決定性是每個實作各自成立的性質**;要跨實作比對,用容差,不用相等。

## 4. 步邊界取樣的盲點

C4 第一版拿「這一步結束時讀到的 CCR」跟 CAN 訊框裡的 duty 比,305 筆有 2 筆不符。看那兩筆:

```
step 376: 前一步 CCR=297,這一步 CCR=298,下一步 CCR=306;CAN 框說 305
```

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

共同點:**每一個的第一眼症狀都指向別的地方**——握手失敗像版本不合、SRAM 全零像位址錯、FIFO 空像模型缺口、C4 不符像韌體回報錯。每一個都是先讀原始碼或加一個更近的觀測點才看到真因。

## 6. 換成 Isaac Sim 6.0.1:要驗的七件事

[`plant/isaac_plant.py`](../../../examples/hil-stm32/plant/isaac_plant.py) 實作同一個 UDP 協定:用 `pxr` / `UsdPhysics` 直接建一台差速車(底盤 + 兩個球形輪 + 球關節腳輪),`DriveAPI` 的 angular 速度目標當馬達,`omni.physx` 手動步進,物理步長綁 calib 的 5 ms。它刻意不用 OmniGraph([31 篇](../../fleet/31-omnigraph-and-ros2-bridge-truth/README.md) §5:headless 下 `world.step(render=False)` 不 tick action graph)、也不依賴 `isaacsim.core.api` 或 `isaacsim.core.experimental` 任一邊([01 篇](../../common/01-install-and-run-modes/README.md) §3 的命名空間搬家)。

⚠ **未在本 repo 環境驗證**。每一條過了才能把標記拿掉:

1. 6.0.1 上 `omni.physx.get_physx_interface().update(dt, dt)` 是否仍是手動步進的正確介面;不是的話改用 experimental 的 `SimulationManager`。
2. `state:angular:physics:position` 在 PhysX 110 上會不會回寫關節角;不會的話腳本退回純運動學積分——結果照樣 ALL PASS,但那不是物理,log 要標明。
3. `timeStepsPerSecond` = 200 有沒有生效:讀回,並用一步後的輪角對 targetVelocity × dt。
4. 球形輪 + 球關節腳輪在 PhysX 110 上會不會被彈飛([32 篇](../../fleet/32-differential-drive-vehicle-model/README.md):腳輪半徑只有驅動輪一半時三輪叉車型在平地會翻)。
5. `DriveAPI` 的 `targetVelocity` 單位是**度/秒**:設 360 → 一秒後輪角 2π。
6. 同一份腳本跑一次,C1–C8 全綠;C3 容差按接觸滑移放寬並寫回判準。
7. 兩次 CSV 是否逐 byte 相同:PhysX GPU dynamics 不保證,CPU 模式較可能;`enableGPUDynamics` 要讀回。

ROS 2 Jazzy 在這個拓撲裡的位置是**上位**:一個 rclpy 節點訂 `/cmd_vel`、發 `/odom`,對橋接講 UART 框包(或讓橋接直接開一個 UDP 給它)。Isaac 那側不需要 ros2 bridge——受控體介面是 UDP,不是 topic。要用 Isaac 內建的 Jazzy 也可以([14 篇](../../6.0.1/14-ros2-bridge-6.0-architecture/README.md)),但那是另一條線,不在這一區的範圍。

## 7. 建議的分階段

不要一開始就接 Isaac。順序:

| 階段 | 迴路裡有什麼 | 判準 |
|---|---|---|
| 骨架 | 橋接 ↔ 假受控體,**沒有韌體**(橋接自己灌固定 duty) | 同一輸入跑兩次逐 byte 相同 |
| 韌體 | 加 Renode 裡的韌體 | C1–C8 全綠;負對照轉紅 |
| 受控體換 UDP | 假受控體改另一個行程 | 與內建版末端一致、途中容差內 |
| Isaac | 受控體換 `isaac_plant.py` | §6 七項;C3 容差重定 |
| 實板 | Renode 換實體 STM32,橋接換實板後端 | 全部重跑;**時序與最壞延遲在這裡才算數** |

每一階段結束才往下一階段,每一階段都留下可重跑的指令與 CSV。這一區做到第三階段。

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
