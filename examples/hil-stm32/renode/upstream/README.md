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

三項修正(都對 RM0090):

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
