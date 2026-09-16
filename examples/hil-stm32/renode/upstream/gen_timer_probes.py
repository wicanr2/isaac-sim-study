#!/usr/bin/env python3
"""從 STM32_Timer_Fixed.cs 產生兩支「拆成本」用的探針版 timer,寫到 renode/out/probe/:

  STM32_Timer_ProbeNoGpio  比較通道的事件照排,但回呼不碰 GPIO   → 量「回呼本體」的成本
  STM32_Timer_ProbeNoCc    比較通道永遠不排事件,主計數器照跑     → 量「少兩個事件/週期」省多少

各配一份 repl 與 resc(自由跑 3 s 牆鐘讀虛擬時間)。跑法:
  python3 renode/upstream/gen_timer_probes.py
  docker run ... antmicro/renode:latest renode --disable-xwt --console -e "include @/w/renode/out/probe/ProbeNoCc.resc"
結果與解讀在 docs/hil/36 篇 §5.1。
"""
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE.parent / "out" / "probe"
OUT.mkdir(parents=True, exist_ok=True)
src = (HERE / "STM32_Timer_Fixed.cs").read_text()

a = src.replace("STM32_Timer_Fixed", "STM32_Timer_ProbeNoGpio")
i = a.index("ccTimers[j].LimitReached += delegate")
j = a.index("if(ccInterruptEnable[j])", i)
a = a[:i] + "ccTimers[j].LimitReached += delegate\n                {\n                    " + a[j:]
(OUT / "STM32_Timer_ProbeNoGpio.cs").write_text(a)

b = src.replace("STM32_Timer_Fixed", "STM32_Timer_ProbeNoCc")
old = "ccTimers[i].Enabled = Enabled && IsInterruptOrOutputEnabled(i) && Value < ccTimers[i].Limit;"
assert old in b
b = b.replace(old, "ccTimers[i].Enabled = false;")
(OUT / "STM32_Timer_ProbeNoCc.cs").write_text(b)

repl = (HERE / "stm32f4-timerfix.repl").read_text()
for name in ("ProbeNoGpio", "ProbeNoCc"):
    (OUT / f"stm32f4-{name}.repl").write_text(
        repl.replace("STM32_Timer_Fixed", "STM32_Timer_" + name).replace('using "./', 'using "../../upstream/'))
    (OUT / f"{name}.resc").write_text(f'''$bin ?= @/w/firmware/build/hilctl.elf
$quantum ?= "0.001"
using sysbus
mach create "hilctl"
emulation SetGlobalQuantum $quantum
i @/w/renode/out/probe/STM32_Timer_{name}.cs
machine LoadPlatformDescription @/w/renode/out/probe/stm32f4-{name}.repl
usart2 CreateFileBackend @/w/renode/out/usart2.txt true
sysbus LoadELF $bin
emulation RunFor "0.2"
echo "[probe] timer3 type:"
python "print type(monitor.Machine['sysbus.timer3']).__name__"
start
python "import time; time.sleep(3.0)"
pause
echo "[probe] virtual time after 3.0 s wall:"
machine ElapsedVirtualTime
quit
''')
print(f"wrote 6 files to {OUT}")
