# 35 · HIL 是什麼,為什麼硬體要放在下位

把 Isaac Sim 裡的車開起來,最省事的做法是一段 Python:收 `cmd_vel`,算差速運動學,直接設兩個輪關節的速度。這在做場景、路網、多車調度時夠用。但它把一整層東西理想化掉了——真實車上,`cmd_vel` 到輪子之間有一顆 MCU:它解析協定、驗 CRC、限幅、跑 PI、發 PWM、讀編碼器、在沒命令 500 ms 後自己停車。這一層的錯誤,純運動學版本永遠不會遇到。

HIL(hardware-in-the-loop)就是把那顆 MCU 放回迴路裡:它跑真的韌體、講真的匯流排協定;Isaac Sim 只負責「馬達轉了會怎樣」。這一篇講它是什麼、跟 NVIDIA 課程裡的 HIL 差在哪、以及一旦迴路裡有三個時鐘,什麼事會變得不一樣。

> **驗證狀態**:本篇的拓撲、時鐘與規則來自 [36](../36-stm32-firmware-on-renode/README.md)–[38](../38-acceptance-and-failure-modes/README.md) 篇實測的那套系統(Renode 1.16.1 上的 STM32F4 韌體 + Rust 橋接 + 假受控體,docker,2026-09-15)。§5 的數字是實跑值。Isaac Sim 6.0.1 那一側未在本 repo 環境驗證。

## 1. 三個詞:MIL、SIL、HIL

| 縮寫 | 迴路裡的「控制器」是什麼 | 迴路裡的「受控體」是什麼 |
|---|---|---|
| MIL(model-in-the-loop) | 控制律的數學模型(方塊圖) | 受控體模型 |
| SIL(software-in-the-loop) | 編譯出來的控制程式,跑在開發機上 | 受控體模型 |
| HIL(hardware-in-the-loop) | **目標硬體上的真韌體** | 受控體模型(即時的) |

三者的差別只在「控制器那一格放什麼」。HIL 的價值是控制器那一格**跟上車的是同一個東西**:同一份二進位、同一個時序、同一組週邊暫存器。

這一區的做法是 HIL 的純軟體變體:STM32F4 跑在 Renode 模擬器裡。跟 SIL 的差別在於韌體沒有重新編譯給開發機——Renode 執行的是給 Cortex-M4 的同一份 ELF,週邊也是同一組暫存器位址。跟真 HIL 的差別在於週邊是模型不是矽,而模型的深度要自己驗([36 篇](../36-stm32-firmware-on-renode/README.md) §3);時序也不算數(§7)。

## 2. 跟 NVIDIA 課程那個 HIL 的差別

NVIDIA 的 [Leveraging ROS 2 and HIL in Isaac Sim](https://docs.nvidia.com/learning/physical-ai/getting-started-with-isaac-sim/latest/leveraging-ros-2-and-hil-in-isaac-sim/index.html)(Getting Started with Isaac Sim 系列)講的是:工作站跑 Isaac Sim(場景、車、車上相機),**Jetson 跑 Isaac ROS 的影像分割**,兩邊靠乙太網路 + ROS 2 對接。迴路裡的硬體是運算平台,沒有 MCU。

| | NVIDIA 課程 | 這一區 |
|---|---|---|
| 迴路裡的實體 | Jetson(上位運算平台) | STM32(下位控制器) |
| 硬體跑什麼 | 感知(分割網路) | 底盤韌體:協定、限幅、安全迴路、輪速控制 |
| 硬體 ↔ Isaac 的介面 | 一個:ROS 2 topic(影像進、結果出) | 兩個:上位側 ROS 2 或序列協定;**受控體側是匯流排訊號**(CAN / GPIO / PWM),要經橋接轉成關節命令 |
| 時鐘 | 全部牆鐘 | 三個時鐘域(§5) |
| 借用 | ROS 2 當膠、Isaac 當受控體與感測器來源 | 同 |
| 不借用 | — | 感知堆疊、Jetson |

兩者不衝突,是同一輛車的兩層。課程沒涵蓋 MCU 這一層,匯流排層的橋接是自己的事——這一區就是那件事。

## 3. 為什麼值得:純運動學版本看不到的東西

下位機韌體裡有四類邏輯,Python 運動學一條都沒有:

1. **協定與 CRC**:壞框包要被拒、不能半解析。[38 篇](../38-acceptance-and-failure-modes/README.md) §2 的負對照就是把每個命令的 CRC 弄壞,看韌體擋不擋。
2. **安全閘門**:沒命令 500 ms 要停、急停腳拉高要停,而且停的動作要**發生在 PWM 與致能腳上**,不是只改一個旗標。
3. **閉環控制**:PI 的積分要在停車時清空,否則下次起步會衝。
4. **時序**:5 ms 控制、20 ms 回報、1 ms tick——這些在真 MCU 上互相搶 CPU。

這四類的失敗形狀都是「上層看起來正常,車的行為不對」。放進迴路之後,它們會在 Isaac 裡以車的動作表現出來,而不是要等到實車。

## 4. 拓撲:每個行程只認一種語言

```
上位(腳本 / Nav2 / ROS 2 Jazzy 節點)
   ↓ cmd_vel        ↑ odom
橋接程式(Rust)
   ↓ UART 框包     ↑ UART 框包          ── 上位側:序列協定
   ↓ CAN 編碼器    ↑ CAN 狀態、PWM、GPIO ── 受控體側:匯流排訊號
STM32F4 韌體(Renode;之後是實板)
   ═══ 匯流排 ═══
橋接程式(同一支)
   ↓ 關節速度目標  ↑ 關節角度、位姿
受控體:假差速車(Python / Rust)| Isaac Sim 6.0.1
```

| 行程 | 認什麼 | 不認什麼 |
|---|---|---|
| 上位 | `cmd_vel`、`odom` | CAN、PWM、關節 |
| 韌體 | 序列協定;CAN / GPIO / PWM 暫存器 | **「自己在模擬裡」——它沒有任何辦法知道** |
| 橋接 | 匯流排上的 byte 與腳位;受控體的關節命令與狀態 | 上位的語意、車的動力學細節 |
| 受控體 | 關節目標、關節狀態、位姿 | CAN、電壓、任何車體協定 |

橋接是唯一同時懂兩邊的行程,所以它是這一區的主要新工件([37 篇](../37-bus-signal-bridging/README.md))。它的責任刻意窄:轉譯與紀錄。

## 5. 三個時鐘域

| 時鐘 | 誰的 | 特性 |
|---|---|---|
| 牆鐘 | 上位、橋接、作業系統 | 真實時間 |
| Renode 虛擬時間 | 韌體 | 由指令數換算(`PerformanceInMips`),主機忙時落後牆鐘;**可以被外部推進** |
| Isaac 模擬時間 | 物理 | 手動步進,每步固定 dt;real-time factor 看 GPU 與渲染 |

[29 篇](../../common/29-long-run-error-budget-and-clock-drift/README.md) §2 講的是後兩者之間的分歧;HIL 多了第一個。三個時鐘各自跑的話,「韌體 5 ms 控制一次」與「Isaac 5 ms 物理一步」之間沒有任何東西保證對齊。

兩種運作模式,同一套橋接:

| 模式 | 做法 | 用在 |
|---|---|---|
| `realtime` | 三個時鐘各自跑,橋接只做轉譯;Isaac 每步 sleep 到牆鐘對齊 | 實板 HIL、Nav2 在迴路裡 |
| `lockstep` | 橋接當節拍器:推進 Renode Δt → 交換匯流排訊號 → 受控體走 Δt → 回填感測器 → 重複 | CI:同一份輸入跑兩次要一模一樣 |

`lockstep` 這一區實測做到了,數字如下(Renode 1.16.1,`PerformanceInMips=100` + 韌體 WFI,假受控體,docker 2 核,主機另有負載 load ≈ 4/14):

- 6 s 腳本 = 1200 步 × 5 ms,牆鐘 35.5 s,**每步 29.6 ms,0.17× 實時**
- Renode 虛擬時間結束於 6,000,000 µs 整,與 1200 × 5000 分毫不差
- 同一腳本跑兩次,1201 行 CSV **逐 byte 相同**

前提是所有注入都要等到「確實進了週邊」才推進時間——沒有這個 ack,同一份輸入會因為執行緒排程而落在不同的步,決定性就沒了([37 篇](../37-bus-signal-bridging/README.md) §3)。

`lockstep` 還有一個固有性質:**一步延遲**。第 k 步結束時注入的編碼器訊框,韌體在第 k+1 步的 `run_for` 裡才讀到。這跟真實匯流排的延遲同形,但要寫進判準(38 篇 C8 的 `steps − 1`)。

## 6. 三條規則

這三條是設計決定,不是最佳實務建議。違反任何一條,HIL 的結論就不再是「韌體的行為」。

**韌體沒有「模擬模式」。** 同一個二進位在 Renode、實板、實車上跑,差別只在匯流排另一端接什麼。任何 `#ifdef SIM` 或「偵測到在模擬裡就跳過」都讓 HIL 驗的變成另一支韌體。這一區的韌體([36 篇](../36-stm32-firmware-on-renode/README.md))連鮑率與 CAN 位元時序都照真硬體寫,Renode 不看那些值也照寫。

**橋接不做安全。** 不夾限、不逾時、不幫忙停車。安全是韌體要證明的事;橋接一聰明,韌體的缺陷就被蓋住——[23 篇](../../common/23-no-shortcuts-in-physics-sim/README.md)講的捷徑,換一個位置再出現一次。橋接可以**記錄**一切,不能**干預**。

**每一輪要有生效證明。** 開跑時印一行:Renode 位址、韌體符號位址與 magic、受控體是哪一個、步長、腳本、負對照模式、`ARR` 讀回值對 calib——每個變數都要有一行證明它進了系統。這一區的橋接把它印在 `[effect]` 開頭的四行裡([38 篇](../38-acceptance-and-failure-modes/README.md) §1)。

## 7. 誠實標註:純軟體 HIL 的邊界

- **時序不算數。** Renode 的虛擬時間由指令數換算,中斷延遲、匯流排仲裁、DMA 競爭都不是實測值。最壞往返延遲與安全迴路的抖動只有實板算數。
- **週邊是模型。** 「Renode 有這個型別」與「這個型別對你的韌體夠用」是兩件事,[36 篇](../36-stm32-firmware-on-renode/README.md) §3 有一張逐項驗過的表——包括一個計數週期差 1 的 timer。
- **受控體是模型。** 假受控體是一階馬達 + 精確運動學;Isaac 的接觸力學會多出滑移(32 篇實測 2~3%),容差要跟著放。
- **Isaac 側未驗。** 腳本在 [`examples/hil-stm32/plant/isaac_plant.py`](../../../examples/hil-stm32/plant/isaac_plant.py),協定已由假受控體驗過,腳本本身要實跑;要驗的七件事列在 [38 篇](../38-acceptance-and-failure-modes/README.md) §6。

## 8. 檢查清單

- [ ] 說得出迴路裡的「硬體」是上位還是下位,以及它跑的是哪一層邏輯
- [ ] 每個行程只認一種語言;橋接是唯一同時懂兩邊的
- [ ] 三個時鐘各是誰的,以及現在用 `realtime` 還是 `lockstep`
- [ ] `lockstep` 的注入有 ack,決定性用「同一輸入跑兩次逐 byte 比」驗過
- [ ] 韌體裡沒有任何「在模擬裡」的分支
- [ ] 橋接不夾限、不逾時、不停車
- [ ] 每一輪的生效證明印得出來
- [ ] 報告裡標明哪些數字是虛擬時間、哪些要等實板

---

延伸閱讀:[36 STM32F4 韌體在 Renode 上開機](../36-stm32-firmware-on-renode/README.md)、[37 匯流排訊號串接](../37-bus-signal-bridging/README.md)、[38 驗收與失敗形態](../38-acceptance-and-failure-modes/README.md)、[29 長跑才會浮現的兩件事](../../common/29-long-run-error-budget-and-clock-drift/README.md)、[23 物理模擬不可以偷懶](../../common/23-no-shortcuts-in-physics-sim/README.md)。
