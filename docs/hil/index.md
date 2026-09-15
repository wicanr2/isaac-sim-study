# HIL:把下位控制器放進迴路

這一區的 4 篇處理的是 **hardware-in-the-loop(HIL)**:讓真的底盤控制器韌體——跑在 STM32 上、講真的匯流排協定——去驅動 Isaac Sim 裡的車。Isaac Sim 這時是「受控體」(馬達、輪子、編碼器),不是全部。

做法是純軟體的:STM32F4 跑在 Renode 模擬器裡,一支 Rust 橋接程式把 Renode 的匯流排訊號(UART、GPIO、Timer PWM、CAN)接到受控體。同一份韌體之後燒到實板,橋接換一個後端,其餘不變。

## 驗證狀態

| 部分 | 狀態 |
|---|---|
| STM32F4 韌體、Renode 平台、IronPython hook、Rust 橋接、假受控體閉環 | **本機實測**(Renode 1.16.1,docker,2026-09-15);數字都是實跑值 |
| Isaac Sim 6.0.1 的受控體腳本 | **場域 GPU 主機實測**(pip 版 6.0.1,PhysX、CPU 求解,同日):閉環 ALL PASS,odom 對真值 3.4 mm / 0.028 rad,兩次 CSV 逐 byte 相同。GPU 求解沒測 |
| 實體 STM32 板 | 未做。時序與最壞延遲只有實板算數,見 [35 篇](35-hil-what-and-why/README.md) §7 |

全部程式碼在 [`examples/hil-stm32/`](../../examples/hil-stm32/),一條指令跑閉環:`./run_loop.sh`。

## 這一區的主線

> **迴路裡的每一段都要能回答「它真的在跑嗎」,而答案不能只來自它自己。**

- 模擬器說「這個週邊型別有」,不等於它對這支韌體夠用——要逐個暫存器驗
- 橋接在步邊界取樣,看不到一步之內的中間值——要在事件發生的當下取樣
- 全綠證明不了測試在驗東西——要有一個負對照讓它轉紅
- 兩個「同一個模型」的實作,末端一致而中途差 ±1 tick——決定性是每個實作各自成立

## 篇章

| # | 主題 | 一句話 |
|---|---|---|
| [35](35-hil-what-and-why/README.md) | HIL 是什麼,為什麼硬體要放在下位 | NVIDIA 課程的 HIL 是 Jetson 跑感知,這裡是 MCU 跑底盤;三個時鐘域,lockstep 與 realtime 兩種模式同一套橋接;**韌體沒有「模擬模式」、橋接不做安全、每輪要有生效證明** |
| [36](36-stm32-firmware-on-renode/README.md) | STM32F4 韌體在 Renode 上開機 | 無 HAL、無 libc 的最小韌體;「型別存在 ≠ 夠用」逐項盤點(CCR 讀得回、PWM 通道是 GPIO 線、**timer 週期是 ARR 不是 ARR+1**、CAN 交握有回應但濾波器有坑);printer 與 SRAM 兩條觀測管道;**WFI 讓 1 s 虛擬時間從 13.4 s 降到 1.83 s**,配套是收訊要改中斷 |
| [37](37-bus-signal-bridging/README.md) | 匯流排訊號串接 | External Control 協定逐 byte;GPIO 讀的是輸出腳、寫的是輸入腳;IronPython hook 對每筆注入回 ack;**CAN 送出模擬器的三條路各碰到哪一層**;lockstep 迴圈六步與每步 29.6 ms 的成本;探埠的一個換行污染了握手 |
| [38](38-acceptance-and-failure-modes/README.md) | 驗收與失敗形態 | 八項判準在開跑前寫死;負對照在 C2 轉紅而 C6 證明韌體擋了全部 300 個壞框包;兩次 CSV 逐 byte 相同;**步邊界取樣的 2/305 不符**與事件時刻快照;十種失敗形態;換成 Isaac 6.0.1 的七件事各量到什麼——**PhysX 介面沒有 `update`、joint state 不寫回、地面 xformOpOrder 反了讓車飛起來**、滑移 3.1% |

## 怎麼讀

**只想知道 HIL 跟「Isaac 裡跑純運動學」差在哪** → [35](35-hil-what-and-why/README.md)。

**要把自己的韌體放進 Renode** → [36](36-stm32-firmware-on-renode/README.md)。先做 §3 那張盤點表,再開始寫程式。

**要寫橋接、或想知道訊號怎麼從模擬器出來** → [37](37-bus-signal-bridging/README.md)。

**要決定「這個閉環算不算通了」** → [38](38-acceptance-and-failure-modes/README.md)。

## 與其他區的關係

- 受控體換成 Isaac Sim 時,ROS 2 bridge 的機制在 [05 篇](../common/05-ros2-bridge/README.md)、6.0 的架構在 [14 篇](../6.0.1/14-ros2-bridge-6.0-architecture/README.md);headless 下 OmniGraph 不 tick 的問題在 [31 篇](../fleet/31-omnigraph-and-ros2-bridge-truth/README.md) §5——這一區的 Isaac 腳本刻意不用 OmniGraph。
- 差速車的物理(輪子碰撞體、腳輪、關節速度單位)在 [32 篇](../fleet/32-differential-drive-vehicle-model/README.md);Isaac 受控體腳本的驗收清單直接引用它。
- 時鐘漂移在 [29 篇](../common/29-long-run-error-budget-and-clock-drift/README.md) §2 講的是 Isaac 模擬時鐘與牆鐘;這一區多了第三個時鐘。
- 驗收探針與預先登記:[30 篇](../common/30-acceptance-probes-and-preregistration/README.md)。這一區的八項判準就是照它的做法寫的。
