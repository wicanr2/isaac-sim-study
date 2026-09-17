# renode/upstream:給 Renode 上游的修正

對象:`renode-infrastructure` @ `add012af003a0f620d3da52828262676f374d121`(= Renode 1.16.1)的 `STM32_Timer.cs`。

| 檔案 | 用途 |
|---|---|
| `STM32_Timer.1.16.1.cs` | 原版逐字副本(對照用) |
| `STM32_Timer.patch` | 對原版的 unified diff,類名不變,可直接 `git apply` 到 fork |
| `STM32_Timer_Fixed.cs` | 同一份修正、類名改成 `STM32_Timer_Fixed`,給 1.16.1 用 `i @file.cs` 執行期載入(不必自建 Renode) |
| `stm32f4-timerfix.repl` | vendor 的 stm32f4 平台,TIM3/TIM4 換成修正版 |
| `probe_stock.resc` / `probe_fixed.resc` | 三項缺口的探針,原版紅、修正版綠(輸出見 docs/hil/36 §3) |
| `STM32_Timer.robot` | 上游 `tests/peripherals/` 樣式的 Robot 測試,對原版 3 紅 |
| `STM32_Timer_fixed_local.robot` | 同一份測試對執行期載入的修正版,3 綠 |
| `probes.py` | monitor 端 helper(讀 GPIO 線、IRQ 線) |
| `STM32_TimerTests.cs` | 上游 `PeripheralsTests` 樣式的 NUnit 測試(仿 `Cadence_TTCTests`),進 fork 的 commit |
| `dotnet-verify/` | 不建整個 Renode 的驗證:上游版檔案對 1.16.1 組件編譯 0 warning;NUnit 修正版 4/4 綠、原版 4/4 紅 |

| 檔案(encoder mode) | 用途 |
|---|---|
| `STM32_Timer.master-47a4e12.cs` | 上游 master(2026-09-09)的 `STM32_Timer.cs` 逐字副本——有 encoder mode 與輸入擷取,1.16.1 沒有 |
| `STM32_Timer_Master.cs` | 同一份改名 `STM32_Timer_Master`、去掉 1.16.1 沒有的 `silent:` 參數,**再加**繞回 0↔ARR、DIR、週期 ARR+1 的修正;`i @` 執行期載入,平台只換 TIM2/TIM4 |
| `stm32f4-encoder.repl` | vendor 平台,TIM2/TIM4 → `STM32_Timer_Master` |
| `probe_encoder*.resc` | 原版 CNT 永遠 0;master 修正版 4 → 0 → 0xFFFC、DIR=1 |
| `STM32_TimerEncoderTests.cs` | 五條 NUnit(四計數/週期、反向 DIR、下繞、上繞、時脈週期);master 原版 1/5、修正版 5/5 |

fork 分支 `stm32-timer-encoder-wrap` commit `a8e98b0`(2026-09-17 rebase 到上游 `master` 0ab5d08;上游在 47a4e12 之後沒動 `STM32_Timer.cs`):encoder 繞回 + DIR + 週期 ARR+1 → [renode-infrastructure PR #252](https://github.com/renode/renode-infrastructure/pull/252)(週期項是 renode#1003 的週期那一半)。NUnit 要用改名版跑:`STM32_TimerEncoderTests.cs` 是上游形式(`new STM32_Timer(`),直接對 1.16.1 組件編譯吃到的是組件裡的原版類別,5/5 全紅;測試類與建構改成 `STM32_Timer_Master` 才是量修正版。

上游狀態(2026-09-16):`CANHub` 修正 rebase 到 `master` 的分支 `canhub-queue-while-paused` → [renode-infrastructure PR #250](https://github.com/renode/renode-infrastructure/pull/250);timer 與 SysTick 因 `master` 已重寫,開 issue [renode#1003](https://github.com/renode/renode/issues/1003)、[renode#1004](https://github.com/renode/renode/issues/1004) 附這裡的 patch。

fork:`wicanr2/renode-infrastructure` 分支 `stm32-timer-period-preload-fixes`(基於 1.16.1 的 commit),三個 commit(訊息英文):`STM32_Timer.cs` + `STM32_TimerTests.cs`;`NVIC.cs` + `NVIC_SysTickTests.cs`;`CANHub.cs` + `CANHubTests.cs`。

| 檔案(重置旗標) | 用途 |
|---|---|
| `STM32F4_RCC.1.16.1.cs` / `STM32_IndependentWatchdog.1.16.1.cs` | 原版逐字副本(1.16.1 與上游 `master` 0ab5d08 逐 byte 相同) |
| `STM32_ResetFlags.patch` | 對兩個原版檔的 diff:`RCC_CSR` 的七個重置旗標移出暫存器集合(系統重置保留、RMVF 清、上電 `0x0E000000`);看門狗要求重置前發 `ResetTriggered`,RCC 的建構參數 `independentWatchdog` 接上就設 IWDGRSTF |
| `STM32F4_RCC_Fixed.cs` / `STM32_IndependentWatchdog_Fixed.cs` | 同一份修正、類名改 `_Fixed`,`i @` 執行期載入;`stm32f4-encoder-rccfix.repl` 用它們,`RCCFIX=1 ./run_loop.sh` 接進閉環 |
| `STM32F4_RCCResetFlagsTests.cs` | 五條 NUnit(上電旗標、RMVF 清、系統重置保留、看門狗逾時設 IWDGRSTF、有餵狗不設);修正版 5/5、原版 1/5 |

fork 分支 `stm32-rcc-reset-flags`(基於上游 `master` 0ab5d08)commit `09a1622` → [renode-infrastructure PR #253](https://github.com/renode/renode-infrastructure/pull/253)。

| 檔案(IMU:I2C + 陀螺儀) | 用途 |
|---|---|
| `STM32F1_I2C.master-0ab5d08.cs` | 上游 `master` 的 `STM32F1_I2C` 逐字副本(檔頭加兩行註解)。1.16.1 只有 `STM32F4_I2C`,它在 STOP 與 repeated START 不呼叫從端的 `FinishTransmission()`,同一個從端的第二筆交易會被當成資料;`master` 換成這個模型(commit 033ee44)。F1/F2/F4 是同一代 I2C,`i @` 執行期載入,類名不變 |
| `LSM330_Gyroscope.1.16.1.cs` / `LSM330_Gyroscope.patch` / `LSM330_Gyroscope_Fixed.cs` | 原版副本(1.16.1 與 `master` 相同)/ diff / 改名執行期載入版:靈敏度照 datasheet(8.75 / 17.5 / 70 mdps/digit,原版 130 digit/dps)、輸出飽和、補 WHO_AM_I_G = 0xD4 與 CTRL_REG1..3_G |
| `LSM330_GyroscopeTests.cs` | 五條 NUnit;修正版 5/5、原版 0/5 |
| `stm32f4-encoder-imu.repl` | `stm32f4-encoder-rccfix.repl` + I2C3 換 `STM32F1_I2C` + `gyro: Sensors.LSM330_Gyroscope_Fixed @ i2c3 0x6A`;`IMU=1 ./run_loop.sh` |

fork 分支 `lsm330-gyro-datasheet-sensitivity`(基於 `master` 0ab5d08)commit `4edd246` → [renode-infrastructure PR #254](https://github.com/renode/renode-infrastructure/pull/254)。`I2CPeripheralBase` 不看子位址 MSb 的自動遞增、`Read(count)` 永遠回 1 byte——韌體每筆交易只讀一個暫存器,這一項沒修。

| 檔案(NVIC) | 用途 |
|---|---|
| `NVIC.1.16.1.cs` / `NVIC.patch` / `NVIC_Fixed.cs` | 同上三件套。改名版**只能用在 NUnit**:CPU 模型對 `nvic` 參數型別檢查,掛不進平台描述 |
| `NVIC_SysTickTests.cs` | 兩種暫存器寫入順序;原版 1/2 紅(FreeRTOS port 的順序)、修正版 2/2 綠 |
| `probe_systick.resc` | 同一件事的 monitor 探針:port 順序啟用後 10 µs CVR=0xFFFD2F,裸機順序 0x1166F |

| 檔案(CANHub) | 用途 |
|---|---|
| `CANHub.1.16.1.cs` / `CANHub.patch` / `CANHub_Fixed.cs` | 同上三件套。改名版由 `hilctl-socketcan-fixed.resc` 執行期載入,建 hub 的指令是 `CreateCANHubFixed` |
| `CANHubTests.cs` | 三條:跑中轉發、暫停期收到的訊框 Resume 時送且只送一次、不回送給發送者;原版 1/3、修正版 3/3 |
| `gen_timer_probes.py` | 不是修正:產生兩支拆 timer 事件成本的探針版 `STM32_Timer`(docs/hil/36 §5.1) |

CANHub 修正:`emulation RunFor` 是 StartAll → RunFor → PauseAll,hub 暫停時 `Transmit()` 直接丟掉訊框;`SocketCANBridge` 的讀執行緒不管暫停照樣讀 socket,所以 lockstep 下主機注入的訊框幾乎全丟(14/399)。改成暫停時排隊、`Resume()` 時送。

SysTick 修正(ARMv7-M B3.3.3):ENABLE 由 0→1 時計數器載入 RELOAD。原版只在寫 CVR 且 RELOAD 已非零時載入;FreeRTOS port 先寫 CVR 再寫 LOAD,計數器從重置值 0xFFFFFF 起跑,第一個 tick 晚 233 ms(72 MHz),韌體沒有任何錯誤。

三項 timer 修正(都對 RM0090):

1. 計數週期 = ARR+1(§18.3.1:計數 0..ARR)。原版週期 = ARR;實測 ARR=99 跑 10000 tick 讀到 CNT=1。
2. OCxPE 預載(§18.4.7):CCRx 寫入在下一個 update event 才轉到作用暫存器;讀回是預載值。原版只有 tag,寫入立即生效。
3. PWM 輸出在 CEN 與 UG 時就依 CNT vs CCR 驅動。原版只在溢位事件換電平,致能後第一次溢位前腳位不動。

第四個候選(`STM32_UART` 的 TC 永不重設)在 1.16.1 上**無法重現**:CPU 寫 DR(DoubleWord 與 Byte 兩種)後 SR 讀到 0xC0、TCIE 開了 IRQ 拉起(`probe_stock.resc` T3)。既有內部紀錄的條件是 DMA 傳送,本目錄沒走那條路,不下結論、不修。

跑法(全部 docker,`--network none`):

```bash
cd examples/hil-stm32
docker run --rm --network none -v "$PWD":/w antmicro/renode:latest renode --disable-xwt --console -e "include @/w/renode/upstream/probe_stock.resc"
docker run --rm --network none -v "$PWD":/w antmicro/renode:latest sh -c 'cd /tmp && renode-test /w/renode/upstream/STM32_Timer_fixed_local.robot'
TIMERFIX=1 ./run_loop.sh     # 修正版 timer 接進閉環(迴歸)
```
