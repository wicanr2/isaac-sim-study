# FreeRTOS-Kernel V11.3.1 最小子集(MIT)

來源:<https://github.com/FreeRTOS/FreeRTOS-Kernel/releases/tag/V11.3.1>(2026-08-21),逐字複製、未修改:

- `tasks.c`、`queue.c`、`list.c`(核心)
- `include/*.h`
- `portable/GCC/ARM_CM4F/`(Cortex-M4F port:PendSV/SVC/SysTick、FPU 上下文)
- `portable/MemMang/heap_4.c`
- `LICENSE.md`

沒帶進來的:`timers.c`、`event_groups.c`、`stream_buffer.c`、`croutine.c`、其他 port。要用再從同一個 tag 補,版本不要混。
