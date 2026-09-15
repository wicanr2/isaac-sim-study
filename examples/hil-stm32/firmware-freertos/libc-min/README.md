# libc-min:給 FreeRTOS kernel 的最小 libc 標頭

工具鏈映像沒有 newlib(`arm-none-eabi-gcc` 只有編譯器與 libgcc)。kernel 的 `tasks.c` / `queue.c` /
`heap_4.c` include `<stdlib.h>` `<string.h>`,實際只呼叫 `memcpy` `memset` `strcpy` `strlen`。
這裡只宣告這四個;實作在 `../startup-rtos.c`。`-ffreestanding` 下 `<stddef.h>` `<stdint.h>` 由 gcc 自帶。
