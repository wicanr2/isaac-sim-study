/* 啟動碼:向量表、.data 搬移、.bss 清零,然後進 main。
 * 沒有 libc,所以 memcpy/memset 也在這裡給編譯器一份(gcc 會自己呼叫它們)。 */
#include <stdint.h>

extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;
extern int main(void);
void SysTick_Handler(void);
void USART1_IRQHandler(void);

void *memcpy(void *d, const void *s, unsigned n)
{
    uint8_t *dd = d; const uint8_t *ss = s;
    while (n--) *dd++ = *ss++;
    return d;
}
void *memset(void *d, int c, unsigned n)
{
    uint8_t *dd = d;
    while (n--) *dd++ = (uint8_t)c;
    return d;
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

/* Cortex-M4 向量表:只填用到的,其餘指到 Default_Handler。
 * SysTick 是第 15 格;外部中斷只開 USART1(IRQ 37,第 16+37 格)。
 * 主迴圈以 WFI 收尾,所以收訊不能靠輪詢——見 main.c 的說明。 */
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
    Default_Handler,  /* SVCall */
    Default_Handler,  /* DebugMon */
    0,
    Default_Handler,  /* PendSV */
    SysTick_Handler,
    /* IRQ 0..36 */
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
