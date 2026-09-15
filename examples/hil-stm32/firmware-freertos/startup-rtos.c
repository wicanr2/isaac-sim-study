/* 啟動碼(FreeRTOS 版):與 ../firmware/startup.c 同,只差向量表把 SVC / PendSV / SysTick
 * 指到 FreeRTOS port 的處理函式(名字由 FreeRTOSConfig.h 的 #define 對上)。 */
#include <stdint.h>

extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;
extern int main(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void USART1_IRQHandler(void);

#include <stddef.h>
/* kernel 要的四個 libc 函式(宣告在 libc-min/string.h) */
void *memcpy(void *d, const void *s, size_t n)
{
    uint8_t *dd = d; const uint8_t *ss = s;
    while (n--) *dd++ = *ss++;
    return d;
}
void *memset(void *d, int c, size_t n)
{
    uint8_t *dd = d;
    while (n--) *dd++ = (uint8_t)c;
    return d;
}
char *strcpy(char *d, const char *s)
{
    char *r = d;
    while ((*d++ = *s++) != 0) { }
    return r;
}
size_t strlen(const char *s)
{
    size_t n = 0;
    while (s[n]) n++;
    return n;
}

static void Default_Handler(void) { for (;;) { } }

void Reset_Handler(void)
{
    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss; ) *dst++ = 0;
    main();
    for (;;) { }
}

__attribute__((section(".isr_vector")))
const void *vectors[16 + 38] = {
    &_estack,
    Reset_Handler,
    Default_Handler,  /* NMI */
    Default_Handler,  /* HardFault */
    Default_Handler,  /* MemManage */
    Default_Handler,  /* BusFault */
    Default_Handler,  /* UsageFault */
    0, 0, 0, 0,
    SVC_Handler,      /* FreeRTOS:第一個 task 從這裡起跑 */
    Default_Handler,  /* DebugMon */
    0,
    PendSV_Handler,   /* FreeRTOS:context switch */
    SysTick_Handler,  /* FreeRTOS:tick */
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler,
    USART1_IRQHandler, /* IRQ 37 */
};
