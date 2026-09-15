# 36 · STM32F4 韌體在 Renode 上開機:「型別存在」不等於「夠用」

把一支韌體放進 Renode,第一個問題不是「跑不跑得起來」,是**「它跑起來之後,你怎麼知道它在跑」**。第二個問題是,平台描述裡列了 USART、TIM、CAN,這些模型對你的韌體夠不夠用——而這件事只能逐個暫存器驗,型別名稱不會告訴你。

這一篇用一支最小的差速底盤控制器韌體走一遍:它的形狀、平台描述怎麼來、每個週邊怎麼驗、兩條觀測管道、以及一個讓模擬速度差 7 倍的設計選擇。

> **驗證狀態**:全部在本機實測。Renode 1.16.1(`antmicro/renode:latest`,build `d66b0c2a-202602160923`),arm-none-eabi-gcc 12.2,docker 2 核,主機另有負載(load ≈ 4/14),2026-09-15。程式碼在 [`examples/hil-stm32/firmware/`](../../../examples/hil-stm32/firmware/) 與 [`renode/`](../../../examples/hil-stm32/renode/)。「既有內部專案」指另一個在 Renode 1.16.1 上跑量產韌體的專案,其結論匿名化引用。

## 1. 韌體的形狀

[`firmware/main.c`](../../../examples/hil-stm32/firmware/main.c) 約 400 行,做真實下位機做的事:

```
USART1 收 cmd_vel 框包(CRC16)→ 兩輪速度設定點
CAN 0x181 收編碼器累計 tick    → 輪速量測 + 里程計
每 5 ms:PI → TIM3 PWM(CCR1/CCR2)+ 方向腳 PB8/PB9 + 致能腳 PB10
每 20 ms:USART1 回 odom、CAN 0x201 回馬達狀態
安全:500 ms 沒命令 → 停;PC13 急停拉高 → 停
```

三個設計決定,每一個都與「在模擬器裡驗」有關:

**沒有 HAL、沒有 libc。** 暫存器定義自己寫([`regs.h`](../../../examples/hil-stm32/firmware/regs.h),位址與位元出自 RM0090),只連 libgcc(soft-float)。理由:每一個寫進週邊的位元都要能回答「模擬器有沒有實作它」。HAL 會在你看不到的地方碰暫存器——既有內部專案的 `HAL_CAN_Init()` 在 Renode 上凍在 `Error_Handler()`,而那支韌體的作者沒有寫過任何一行 CAN 暫存器。

**沒有「模擬模式」。** 鮑率、CAN 位元時序、GPIO 的 AF 設定全部照真硬體寫。Renode 不看鮑率,照寫。

**觀測結構固定版面。** `g_dbg` 是一個 `volatile` 結構,第一個欄位是 magic `0x48494C31`,後面是 tick、控制步數、設定點、量測、duty、收到的訊框數、壞 CRC 數、旗標、初始化錯誤碼、ring buffer 溢位數。橋接從 `nm` 的輸出拿位址,讀第一個字確認讀對了東西(§4)。

大小:text 4732 B、data 216 B、bss 316 B(`-O2`)。

## 2. 平台描述:vendor 一份,拿掉一行

Renode 內建的 `platforms/cpus/stm32f4.repl` 對這支韌體夠用——記憶體配置(flash 2 MB @ `0x08000000`、SRAM 256 KB @ `0x20000000`)、USART1/2、TIM3、GPIO A–C、CAN1、NVIC、SysTick 都在。但要**vendor 一份進 repo**([`renode/stm32f4-1.16.1.repl`](../../../examples/hil-stm32/renode/stm32f4-1.16.1.repl)),兩個理由:

1. **它會上網。** 檔尾有一行 `ApplySVD @https://dl.antmicro.com/.../STM32F40x.svd.gz`,`--network none` 下重試五次才放棄。SVD 只影響 log 裡的暫存器名稱,拿掉那行沒有功能損失。
2. **版本要鎖。** 既有內部專案比對過:master 分支的同名檔已與 1.16.1 不同(`i2c1` 的型別就不一樣)。浮動引用會讓「昨天還能跑」變成難查的問題。

[`renode/hilctl.repl`](../../../examples/hil-stm32/renode/hilctl.repl) 只有一行 `using "./stm32f4-1.16.1.repl"`;這支韌體不需要基底檔沒有的週邊。

## 3. 逐項盤點:「有」是什麼意思

判斷一個週邊「內建有沒有」,要查的是**實作深度**,不是型別名存不存在。下表每一列都是用 monitor 直接讀寫暫存器驗的,不載韌體(探針腳本的做法見 §6)。

| 週邊 | 韌體碰的暫存器 | Renode 1.16.1 實測 | 對這支韌體 |
|---|---|---|---|
| `STM32_Timer`(TIM3) | ARR、CCR1/2、CCMR1(OC1M)、CCER、CR1、CNT | 全部寫入後讀得回;**OC1PE / OC2PE 未實作**(warning);PWM 通道輸出是真的 GPIO 線:PWM1 模式下 `timer.Connections[0].IsSet` 在 CNT < CCR 時為 True | 夠用。橋接讀 CCR 算 duty |
| 同上,計數週期 | ARR | **週期是 ARR,硬體是 ARR+1**。兩組量測:ARR=999、10 MHz 跑 1 ms → CNT=10(=10000 mod 999);ARR=99 → CNT=1(=10000 mod 99) | PWM 頻率差 0.1%,無感。但這是一個可送上游的修正候選 |
| `STM32_UART`(USART1/2) | SR(RXNE/TXE)、DR、BRR、CR1 | TXE 恆為 1;RX 有佇列。既有內部專案:**TC 旗標做成「可寫清除但永不重設」**,等 TC 中斷的傳送路徑會卡死 | 韌體只輪詢 TXE、不等 TC。真硬體同樣正確 |
| `STMCAN`(CAN1) | MCR/MSR、BTR、TSR、TI0R/TDT0R/TDL0R/TDH0R、RF0R、RI0R/RDT0R/RDL0R/RDH0R、FMR/FM1R/FS1R/FFA1R/FA1R/F0R1/F0R2 | MCR.INRQ=1 → MSR 0xC01(INAK=1,SLAK 清);mailbox 寫入 TXRQ 後 `FrameSent` 立刻觸發;`OnFrameReceived()` 注入的訊框進 FIFO0(FMP0=1、RI0R 帶 STID) | 夠用,**但濾波器有一個坑**(下一段) |
| `STM32_GPIOPort` | MODER、AFRL、IDR、ODR、BSRR | 輸出腳在 `Connections[n]`;輸入腳用 `OnGPIO(n, v)`;`State` 是 protected,monitor Python 讀不到 | 夠用 |
| NVIC + SysTick | ISER、SysTick CSR/RVR | SysTick 頻率來自平台描述的 `systickFrequency: 72000000`,**不是 RCC**;USART1 IRQ 37 正常進 | 夠用;鮑率與 SysTick 的時脈來源在模擬裡是兩個不相干的數 |

CAN 濾波器的坑:`FMR` 的重置值是 `0x2A1C0E01`,其中 `CAN2SB`(bit 13:8)= 14,意思是 bank 0–13 屬於 CAN1。探針一開始寫 `FMR = 1`(只想設 FINIT),把 `CAN2SB` 清成 0——bank 0 從此屬於 CAN2,CAN1 的接收路徑對它 `Where(BelongsToMaster)` 一濾,訊框**靜默丟掉**:`OnFrameReceived()` 呼叫成功、`FrameReceived` 事件照樣觸發,FIFO 就是空的。真硬體的 HAL 用讀-改-寫所以不會踩到;自己寫暫存器的人會。韌體的 `can_init()` 因此全部用 `|=` / `&=`。

另一個與既有紀錄不一致的點要誠實寫:既有內部專案的結論是「`STMCAN` 沒實作 `HAL_CAN_Init()` 要的 MCR/MSR 交握」,本篇探針從匯流排直接寫 INRQ 卻看到 INAK 正確回應。兩者可能都對——HAL 的序列多了 `SLEEP` 清除與逾時計算,那條路徑本篇沒有走。**這裡只能說「自己寫的序列交握有回應」,不能說「既有紀錄錯了」。**

## 4. 兩條觀測管道

**printer**:USART2 接檔案後端(`usart2 CreateFileBackend @path true`;headless 下 `showAnalyzer` 沒用),韌體開機吐三行:

```
hilctl boot fw 0.1
can1 ready
main loop
```

**SRAM 變數**:`sysbus ReadDoubleWord 0x200000F8` 讀 `g_dbg.magic`,位址從 `arm-none-eabi-nm -n` 的輸出來。跑 200 ms 之後:magic `0x48494C31`、`tick_ms` 200、`ctrl_steps` 39、`flags` 4(CMD_STALE)、`init_err` 0。

兩條都要留,因為它們失效的方式不同。printer 一旦卡住(既有內部專案就是 TC 旗標讓它停在第一則),後面每一步都看不見;SRAM 變數不會卡,但要先知道位址、也看不到「順序」。既有內部專案還留了一個分辨法:PC 停在自旋迴圈與時鐘停了,現象一模一樣,分辨的方式是去讀那個被等的變數(`uwTick` 有沒有前進),不是猜。

⚠ **機器還沒跑的時候,SRAM 全零。** `LoadELF` 之後機器是暫停的,`main` 還沒執行;這時讀 `g_dbg` 得到 magic = 0、`ARR` = 65535(重置值)。橋接第一版就在這裡把「韌體不對」誤報了一次——生效證明要在**開機之後**做([38 篇](../38-acceptance-and-failure-modes/README.md) §5)。

## 5. 速度:瓶頸是 MMIO 輪詢,解法是 WFI

第一版主迴圈純輪詢(每圈讀 USART SR、CAN RF0R、GPIO IDR),量出來:

| 設定 | 1 s 虛擬時間的牆鐘 | 40 步 × 5 ms | 每步 | 1 s 執行的指令數 |
|---|---|---|---|---|
| MIPS=100,純輪詢 | 13.4 s | 2.44 s | 61 ms | 1.0 × 10⁸ |
| MIPS=10,純輪詢 | 3.2 s | 0.37 s | 9 ms | 1.0 × 10⁷ |
| **MIPS=100,迴圈尾 WFI** | **1.83 s** | **0.41 s** | **10 ms** | **5.1 × 10⁵** |

40 步與連續 1 s 的速率一樣,所以瓶頸不在 `RunFor` 的切換,是指令執行本身——每次週邊存取都要進 C# 的匯流排分派,一個只做 MMIO 輪詢的迴圈等於把最貴的操作做到滿。

降 MIPS 有效但代價是模擬的 MCU 變慢(真 F4 約 72 MIPS)。WFI 同樣有效而且保留真實速度:CPU 睡到下一個中斷,Renode 直接跳到那個時刻,1 s 只執行 51 萬條指令。

**WFI 的配套**:迴圈醒來的節奏變成 SysTick 的 1 ms。115200 bps 每毫秒進 11 個 byte,而 DR 只裝得下 1 個——1 ms 輪詢一次一定掉資料。所以 USART1 收訊改成中斷 + ring buffer。**Renode 的 UART 模型有佇列,輪詢版在模擬裡「看起來也對」**;這是模擬器比硬體寬容的地方,設計要以硬體為準。CAN 的 FIFO0 裝得下 3 筆、編碼器 5 ms 一筆,輪詢即可。

## 6. 驗收:五項,一項是負對照

[`renode/io_check.resc`](../../../examples/hil-stm32/renode/io_check.resc) 不經橋接,用 monitor 直接餵資料,把韌體這一層獨立驗到綠:

| 項 | 做什麼 | 結果 |
|---|---|---|
| A 壞 CRC | 餵一個最後一 byte 反相的 CMD_VEL | `bad_crc` 1、`cmd_frames` 0 |
| B 正確 CMD_VEL v=300 | 30 ms 後讀 | 設定點 300/300、flags ENABLED、duty 526、**CCR1 = 526**、GPIOB ODR = `0x700`(PB8/9/10 高) |
| C 編碼器 | 兩筆 CAN 0x181 各 +20 tick,間隔 5 ms | `enc_frames` 2、`meas_l` 306 mm/s(理論 20 × 314159 / 4096 / 5 = 306.8) |
| D 急停 | `gpioPortC OnGPIO 13 true` | flags ESTOP、duty 0、ODR `0x300`(PB10 低,方向腳不動) |
| E 命令逾時 | 放開急停、600 ms 不送命令 | flags CMD_STALE;`rx_overflow` 0 |

A 是負對照:它證明「B 過了」不是因為韌體什麼都收。

餵資料的 monitor 函式在 [`io_check.py`](../../../examples/hil-stm32/renode/io_check.py):`usart1.WriteChar(byte)` 是從外面塞一個收到的 byte,`can1.OnFrameReceived(frame)` 是從匯流排塞一個訊框。這兩個方法就是 [37 篇](../37-bus-signal-bridging/README.md)的 hook 在用的。

## 7. monitor 與 IronPython 的坑

這些每一個都會產生看起來像別的問題的錯誤訊息:

- **數字型參數會被轉成 int。** `inject 321 11223344` 進到 Python 函式時是 `int`,`len()` 就炸。要字串就加引號:`inject "321" "11223344"`。
- **週邊名字會被解析成物件。** `gpio_state gpioPortA 0` 收到的第一個參數是 `STM32_GPIOPort` 物件,不是字串;`"sysbus." + port` 直接型別錯。
- **`.NET 8` 下要先 `clr.AddReference("System.Net.Primitives")` 與 `"System.Net.Sockets"`**,否則 `from System.Net import IPAddress` 報 `Cannot import name`。
- **事件 handler 的簽章要對。** `can1.FrameReceived` 帶兩個參數 `(int rir, byte[] data)`;寫成一個參數的 handler,例外會從 `OnFrameReceived()` 那一側丟出來,看起來像注入失敗。
- **`gpioPortA.State` 是 protected**,monitor 級 Python 讀不到;要用 `Connections[n].IsSet`(輸出腳)或 External Control 的 GetState。
- 既有內部專案對 `Python.PythonPeripheral` 的三個坑(inline 腳本不能縮排、非 ASCII 會炸、每次暫存器存取重跑整份)是那個機制特有的;本篇用的 monitor 級 `include @file.py` 沒有前兩個問題,但檔案還是全 ASCII 寫,省得日後搬去當 peripheral 時再踩一次。

## 8. 檢查清單

- [ ] 韌體裡沒有 HAL 幫你碰的暫存器(或你知道它碰了哪些)
- [ ] 平台描述 vendor 進 repo,版本鎖死,`ApplySVD` 那行拿掉
- [ ] 每個週邊做過「寫進去讀得回來」與「行為對」兩種驗證,不是只看型別名
- [ ] CAN 的 `FMR` 用讀-改-寫
- [ ] 有 printer 之外的第二條觀測管道(SRAM 變數 + magic)
- [ ] 讀 SRAM 之前機器跑過了
- [ ] 迴圈尾有 WFI,而且收訊改成中斷
- [ ] 驗收裡至少一項是負對照
- [ ] 與既有紀錄不一致時寫「本篇量到什麼」,不寫「既有紀錄錯了」

---

延伸閱讀:[37 匯流排訊號串接](../37-bus-signal-bridging/README.md)、[38 驗收與失敗形態](../38-acceptance-and-failure-modes/README.md)、[35 HIL 是什麼](../35-hil-what-and-why/README.md)。
