# 37 · 匯流排訊號串接:External Control、hook,與 CAN 的三條出口

韌體在 Renode 裡跑起來之後,它的輸出在模擬器裡面:PWM 是 TIM3 的 CCR 值、方向是 GPIOB 的兩個腳、狀態回報在 CAN1 的 mailbox。要把這些拿到外面給受控體,又要把編碼器塞回去給韌體,Renode 1.16.1 給了兩條路:一個叫 External Control 的 TCP 二進位 API(時間、GPIO、匯流排讀寫),和它自己的 monitor 級 IronPython(可以掛週邊事件、也能開 socket)。CAN 還有第三條路,要碰主機核心。

這一篇把每種訊號走哪條路、協定長什麼樣、以及 lockstep 迴圈怎麼靠 ack 做到決定性,寫清楚。

> **驗證狀態**:全部在本機實測(Renode 1.16.1,Rust 1.98 std-only 橋接,docker,2026-09-15)。External Control 的線路協定是從官方 C client(`tools/external_control_client/lib/renode_api.c`,943 行)逐 byte 讀出來的,官方沒有文件;server 端對照 `renode` repo v1.16.1 的 `src/Renode/Network/ExternalControl/*.cs`。§4 的 SocketCAN 路線**未實測**(要主機載 `vcan` 模組),只寫接法。程式碼在 [`examples/hil-stm32/bridge-rs/`](../../../examples/hil-stm32/bridge-rs/) 與 [`renode/hil_hook.py`](../../../examples/hil-stm32/renode/hil_hook.py)。

## 1. 每種訊號走哪裡

| 訊號 | 方向 | 出口 | 為什麼 |
|---|---|---|---|
| 時間推進、目前時間 | 橋接 → Renode | External Control `run_for` / `get_time` | lockstep 的節拍 |
| PWM duty | MCU → 受控體 | External Control `sysbus_read` TIM3 `CCR1/CCR2/ARR` | CCR 讀得回([36 篇](../36-stm32-firmware-on-renode/README.md) §3);訂閱 PWM 腳的邊緣事件每秒會有上萬筆 |
| 方向、致能 | MCU → 受控體 | External Control `gpio_get` gpioPortB 8/9/10 | 讀的是**輸出腳**(`Connections[n].IsSet`) |
| 急停 | 受控體 → MCU | External Control `gpio_set` gpioPortC 13 | 寫的是**輸入腳**(`OnGPIO(n, v)`) |
| 韌體內部狀態 | MCU → 橋接 | External Control `sysbus_read` `g_dbg`(17 個字一次讀) | 生效證明與驗收用 |
| UART(上位協定) | 雙向 | IronPython hook(`usart1.WriteChar` / `CharReceived`) | 要 ack(§3);Renode 內建的 socket terminal 也能用但沒有 ack |
| CAN | 雙向 | IronPython hook(`can1.OnFrameReceived` / `FrameSent`) | 不碰核心;三條路的取捨在 §4 |

兩個 TCP 埠:3500(External Control)、3600(hook)。3456 的 socket terminal 留著當對照。

## 2. External Control API

`emulation CreateExternalControlServer "ec" 3500` 起 server。Rust client 在 [`bridge-rs/src/ec.rs`](../../../examples/hil-stm32/bridge-rs/src/ec.rs),只用 std。線路全部 little-endian:

**握手**:client 送 `u16 count` + count 組 `(cmd u8, ver u8)`。1.16.1 的表是 `(1,0) (2,0) (3,0) (4,0) (5,1) (6,0)`——GPIO 是 v1,其餘 v0。server 回 1 byte:`5` 成功;`1` 是 FATAL,後面接 `u32 len` + 訊息。

**命令框**:`'R' 'E' cmd:u8 len:u32 data`。命令碼:1 RunFor、2 GetTime、3 GetMachine、4 ADC、5 GPIO、6 SystemBus。

**回應**:第 1 byte 是 return code,之後的形狀依 code 而定:

| code | 意思 | 後面接什麼 |
|---|---|---|
| 0 | COMMAND_FAILED | cmd u8、len u32、訊息 |
| 1 | FATAL_ERROR | len u32、訊息(**沒有 cmd**) |
| 2 | INVALID_COMMAND | cmd u8 |
| 3 | SUCCESS_WITH_DATA | cmd u8、len u32、data |
| 4 | SUCCESS_WITHOUT_DATA | cmd u8 |
| 5 | SUCCESS_HANDSHAKE | — |
| 6 | ASYNC_EVENT | cmd u8、ed u32、len u32、data |

`RunFor` 的回應可能被 ASYNC_EVENT 插隊,client 要吃掉事件再等自己的 4。GPIO 事件的 data 是 **16 bytes**:`u64 timestamp_us` + `bool` + 7 bytes 填充——C 版用 `sizeof(struct)` 帶過,Rust 端要自己算。

**實例描述子**:GPIO 與 SystemBus 命令先用 `id=-1, md, name_len, name` 拿一個 `i32 id`,之後的命令都帶那個 id。名字用 `sysbus.gpioPortB` 或 `gpioPortB` 都解得到;sysbus 本身用 `"sysbus"`。

**GPIO 語意**(server 端 `GPIOPort.cs`):`GetState(id, pin)` 回 `peripheral.Connections[pin].IsSet`——**輸出腳**;`timer3` 也是 `INumberedGPIOOutput`,所以 PWM 通道用同一個命令讀得到。`SetState(id, pin, v)` 呼叫 `OnGPIO(pin, v)`——**輸入腳**。`RegisterEvent` 掛在輸出腳的狀態變化上。

**SystemBus**:`id i32, op u8(0 讀/1 寫), width u8(1/2/4/8), addr u64, count u32, data[]`。`count` 是「幾個 width」不是幾 byte;讀 17 個 u32 一個 RPC 就好。

一個踩過的坑:**探埠會污染握手**。`run_loop.sh` 第一版用 bash 的 `echo >/dev/tcp/127.0.0.1/3500` 等埠開,那個換行字元被 server 當成握手的第一個 byte,狀態機從此錯位,client 收到 `Encountered unknown command 0x00`。解法是不用會送資料的探測,橋接自己重試連線。

## 3. hook:IronPython 一條 TCP,每筆注入回 ack

[`renode/hil_hook.py`](../../../examples/hil-stm32/renode/hil_hook.py) 由 `.resc` 的 `include @hil_hook.py` 載入,跑在 Renode 的 monitor IronPython 裡。它掛兩個事件、開一個 `TcpListener`、起一條背景執行緒收注入:

```
can1.FrameSent      += 每個 MCU 送出的 CAN 訊框 → 寫到 socket
usart1.CharReceived += 每個 MCU 送出的 UART byte → 湊 8 個寫到 socket
socket 收到 CAN 紀錄  → can1.OnFrameReceived(frame) → 回 ack
socket 收到 UART 紀錄 → usart1.WriteChar(b) ×n      → 回 ack
```

紀錄固定 13 bytes:`u32 id | u8 len | u8[8] data`。`id < 0xFFFF0000` 是 CAN;`0xFFFF0001` UART 進 MCU、`0xFFFF0002` UART 出 MCU、`0xFFFF0003` 匯流排快照、`0xFFFF00AC` ack。

**為什麼要 ack。** `WriteChar` 與 `OnFrameReceived` 都是從主機執行緒呼叫進週邊的——Renode 內建的 socket terminal 也是這樣做。橋接寫完 socket 就 `run_for` 的話,那些 byte 是在這一步之前還是之後進到週邊,由執行緒排程決定。有 ack,橋接等到「注入完成」才推進時間,同一份輸入就一定落在同一步。[38 篇](../38-acceptance-and-failure-modes/README.md) §3 的「兩次 CSV 逐 byte 相同」靠的就是這個。

**匯流排快照。** `FrameSent` handler 在送出 CAN 紀錄之後,立刻在同一個模擬時刻讀 `TIM3 CCR1/CCR2`,附一筆 `0xFFFF0003`。橋接拿它跟 CAN 訊框裡韌體自己回報的 duty 比——兩條獨立管道在**同一個時刻**比。為什麼不能拿步邊界讀到的 CCR 比,[38 篇](../38-acceptance-and-failure-modes/README.md) §4 有 2/305 筆的實例。

**機器在跑的時候,注入不能直接呼叫週邊。** `lockstep` 下注入時機器是暫停的,從背景執行緒直接 `WriteChar` 沒事。`realtime` 下機器在跑,同一行會卡死——第一個命令進得去,第二個 `WriteChar` 永遠不回來(模擬執行緒與注入執行緒搶週邊的鎖)。Renode 自己的外部裝置(socket terminal 的 `UARTBackend`、`CANHub`)走的是 `machine.HandleTimeDomainEvent(handler, arg, timestamp)`:把呼叫排進機器的時間域,在下一個同步點由模擬執行緒執行。hook 的 `_deliver` 照這條:`IsStarted` 為真就排隊,否則直呼。時間戳要用 `TimeDomainsManager.Instance.GetEffectiveVirtualTimeStamp()`——`VirtualTimeStamp` 只有註冊過的模擬執行緒能讀,從 hook 的執行緒讀會丟 `Tried to obtain a virtual time stamp of an unregistered thread`。

**關掉 Nagle。** 紀錄 13 bytes、一筆一答。`lockstep` 下 ack 是當時唯一的出站資料,立刻送;`realtime` 下 `FrameSent` 不斷從模擬執行緒寫小封包,ack 排在未確認資料後面,等對端的 delayed ACK——每步 42 ms,全部是這個。`client.NoDelay = True` 之後 hook 每步 2.4 ms。橋接那一側從一開始就 `set_nodelay(true)`,只設一邊不夠。

限制:一次一個 client;IronPython 的執行緒是 .NET 執行緒,`_send` 用 `Monitor.Enter` 保護;檔案全 ASCII。

## 4. CAN 送出模擬器的三條路,各碰到哪一層

Renode 1.16.1 把 CAN 訊框送到模擬器外面的**官方**管道只有 `CreateSocketCANBridge`,它接的是一個 Linux CAN 網路介面。沒有實體卡時那個介面是 `vcan`——名字容易誤會,它是純軟體的虛擬介面(像 `lo`),但它是核心模組(這台 `CONFIG_CAN_VCAN=m`),載入要 root。

| 路 | 碰核心嗎 | 原理 | 本篇狀態 |
|---|---|---|---|
| **① monitor 級 IronPython hook → TCP** | 不碰 | 掛 `FrameSent`、呼叫 `OnFrameReceived()`;IronPython 用 .NET socket。整條在 Renode 行程內 | **實測,主線** |
| ② 容器內 `ip link add vcan0 type vcan`(`--cap-add NET_ADMIN`) | 碰,核心自動載入 | netlink 建未知型別的 link 時核心會 `request_module("rtnl-link-vcan")`;不用手打 modprobe,但主機一樣多載一個模組 | 未實測 |
| ③ 主機 `modprobe vcan` + `SocketCANBridge` | 碰 | Renode 官方路線;橋接用 SocketCAN API 收發 | 未實測 |

②③ 是同一件事——都讓核心載入 `vcan.ko`,差別只在誰觸發。真正不碰核心的只有 ①。

①在 Renode 端遇到缺口時的處理原則:**不繞路,修 Renode 原始碼**。1.16.1 對應的 `renode-infrastructure` commit 是 `add012af003a0f620d3da52828262676f374d121`;修週邊模型可以 `i @file.cs` 執行期編譯載入,不必自建 Renode——改一行、跑一次探針是秒級迴圈。

這一區用這條路修了 `STM32_Timer` 的三個缺口([`renode/upstream/`](../../../examples/hil-stm32/renode/upstream/)):計數週期 ARR+1、OCxPE 預載、致能時就驅動 PWM 腳。每一項有一支探針(原版紅、修正版綠)與一條上游樣式的 Robot 測試(原版 3 紅、修正版 3 綠);修正版接進閉環 `TIMERFIX=1 ./run_loop.sh` 仍 ALL PASS,而且 `.resc` 印出 timer 的型別名當生效證明——沒有這一行,「修正版也綠」與「根本沒載入」看起來一樣。第四個候選 `STM32_UART` 的 TC 閂鎖在 1.16.1 上**無法重現**(CPU 寫 DR 後 TC 正常設回、TCIE 拉中斷);既有內部紀錄的條件是 DMA 傳送,這裡沒走那條路,不下結論。

第五個缺口是 FreeRTOS 版才踩到的:`NVIC` 的 SysTick 在 ENABLE 0→1 時不從 RELOAD 載入([39 篇](../39-freertos-firmware-in-the-loop/README.md) §4),NUnit 原版 1/2 紅、修正版 2/2 綠。

修正以上游樣式放在 fork(`wicanr2/renode-infrastructure`,分支 `stm32-timer-period-preload-fixes`,基於 1.16.1 的 commit),兩個 commit:`STM32_Timer.cs` + `STM32_TimerTests.cs`、`NVIC.cs` + `NVIC_SysTickTests.cs`。驗證到哪裡:上游版檔案對 1.16.1 組件編譯 0 warning、NUnit 修正版 4/4 綠、原版 0/4、Robot 3/3、閉環迴歸 ALL PASS;**沒做**完整 Renode 建置與上游全部測試。**尚未送 PR**(2026-09-15 決定:分支留在 fork,之後要送隨時可以);送了之後這一段要補連結。

## 5. lockstep 迴圈

[`bridge-rs/src/main.rs`](../../../examples/hil-stm32/bridge-rs/src/main.rs) 每一步(5 ms):

```
1. 上位腳本 → cmd_vel 框包 → hook UART 注入 → 等 ack        (每 20 ms 一次)
2. External Control run_for(5 ms)
3. 讀匯流排:CCR1/CCR2(相鄰,1 RPC)、PB8/9/10(3 RPC)、g_dbg(1 RPC)、時間(1 RPC)
4. 收 MCU 這一步吐出的 UART(odom)與 CAN(狀態 + 快照):flush → ack → drain 1 ms
5. 受控體走 5 ms → 編碼器 tick → hook CAN 注入 → 等 ack
6. 寫一行 CSV
```

**一步延遲**:第 5 步注入的訊框,韌體在下一步的 `run_for` 裡才讀。6 s 跑完 `enc_frames` = 1199 = steps − 1。

**每步成本**:主機閒時 8–13 ms(分段:`run_for` 9.3 ms、hook 2.2 ms、`ec_read` 1.0 ms),主機另有負載時 29.6 ms——橋接每步印 `[run] per-step wall: ec_read/hook/plant/run_for` 四段,看數字不用猜。要更快的話,順序是:把 drain 換成「等一個明確的 end-of-step 紀錄」(省 1 ms)、把 GPIO 三次讀合併成一次 `ODR` 匯流排讀(省 2 RPC)、最後才是把 `run_for` 拉長。

`--mode realtime` 用同一個迴圈,只換第 2 步:開跑前經 hook 送 `START`(`0xFFFF0010`,hook 呼叫 `StartAll()` 後 ack),每步不再 `run_for`,改 sleep 到下一個 5 ms 牆鐘刻度;受控體的 dt 用實際過了多久;跑完送 `PAUSE`(`0xFFFF0011`)。每步 5.0–5.2 ms(`ec_read` 1.8 + hook 2.4 + sleep 0.4),三個時鐘的分歧與後果量在 [35 篇](../35-hil-what-and-why/README.md) §5.1。

## 6. 受控體介面

橋接對受控體只暴露兩個結構:

```
MotorCmd { duty_l, duty_r: 0..1;  fwd_l, fwd_r, enabled: bool }
PlantOut { ticks_l, ticks_r: i32 (累計,繞回);  x_mm, y_mm, th_rad (真值);  vl, vr }
```

三種後端,同一個 trait:`Fake`(Rust 內建,一階馬達 + 精確差速運動學)、`Udp` 與 `Tcp`(另一個行程;TCP 是給 `ssh -L` 隧道用的,ssh 只轉 TCP)。文字協定一行一筆,受控體要以相同 `seq` 回覆,橋接等到才推進:

```
橋接 → 受控體:CMD <seq> <dt_ms> <duty_l 0..1000> <duty_r> <fwd_l> <fwd_r> <en>
受控體 → 橋接:ENC <seq> <ticks_l> <ticks_r> <x_mm> <y_mm> <th_rad> <vl> <vr>
```

[`plant/fake_plant.py`](../../../examples/hil-stm32/plant/fake_plant.py) 用 Python 實作同一個模型(UDP / TCP,實測 ALL PASS);[`plant/isaac_plant.py`](../../../examples/hil-stm32/plant/isaac_plant.py) 是 Isaac Sim 6.0.1 版,在場域 GPU 主機實測 ALL PASS,七件事的結論在 [38 篇](../38-acceptance-and-failure-modes/README.md) §6。

受控體在另一台主機時,`run_loop.sh` 的 `PLANT=remote` 讓 Renode 容器改掛 docker 的 bridge 網路,`ssh -L` 綁在 bridge 的閘道位址上——隧道只有容器看得到,Renode 的埠也沒有 publish 到主機。隧道的 stdout 要導到檔案、ssh 要用 `exec` 起,否則腳本收不掉(38 篇 §5)。實測隧道讓每步從 29.6 ms 變 48.6 ms(假受控體)/ 52 ms(Isaac)。

橋接不做安全:`MotorCmd` 直接來自匯流排讀值,不夾限、不逾時。韌體停了車,`enabled` 就是 false,受控體自己會停。

## 7. 檢查清單

- [ ] 每種訊號說得出走哪條出口、讀的是輸出腳還是輸入腳
- [ ] External Control 的握手版本表對上 server(GPIO v1)
- [ ] 沒有任何探測會往 3500 埠送資料
- [ ] 每筆注入等到 ack 才 `run_for`
- [ ] 兩條獨立管道的比對用同一時刻的快照,不用步邊界的讀值
- [ ] CAN 走哪條路、碰不碰核心,寫在 README
- [ ] Renode 缺口:修原始碼並記 commit,不繞路
- [ ] 一步延遲寫進判準

---

延伸閱讀:[36 STM32F4 韌體在 Renode 上開機](../36-stm32-firmware-on-renode/README.md)、[38 驗收與失敗形態](../38-acceptance-and-failure-modes/README.md)、[05 ROS2 橋接](../../common/05-ros2-bridge/README.md)(Isaac 那一側的 UDP 解耦做法同源)。
