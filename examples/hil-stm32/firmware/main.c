/*
 * hilctl:差速底盤控制器的最小韌體(STM32F4,裸機,無 HAL / libc)。
 *
 * 職責與真實下位機一樣:
 *   上位(USART1)給 cmd_vel  →  兩輪速度設定點  →  每 5 ms 一次 PI  →  PWM + 方向腳
 *   編碼器(TIM2/TIM4 encoder mode,或 CAN 0x181)→  輪速量測 + 里程計  →  每 20 ms 回報 odom(USART1)
 *   安全:500 ms 沒命令 → 停;急停腳(PC13)拉高 → 停;驅動器故障腳(PC14/15)低 → 停;
 *         保險桿(PC0)斷 → 拒絕前進;堵轉(duty 高而輪不動 200 ms)→ 停到上位歸零;
 *         上位心跳(PING)300 ms 沒來 → 降速到 0;IWDG 1 s 沒餵 → 整顆重置。安全在這裡,不在橋接。
 *
 * 協定、暫存器序列、控制律、安全閘門、里程計都在 control.c(與 FreeRTOS 版共用);
 * 這個檔只有裸機才有的事:SysTick、USART1 ISR 與 ring buffer、主迴圈的排程、餵狗的位置。
 *
 * 這支韌體沒有「模擬模式」:它不知道匯流排另一端是 Renode 的模型還是真的驅動器。
 *
 * 觀測管道有兩條,教學上刻意都留:
 *   1. USART2 文字輸出(printer)
 *   2. g_dbg 結構(SRAM,橋接用 sysbus 直接讀,位址在 build/hilctl.sym)
 */
#include <stdint.h>
#include "regs.h"
#include "proto.h"
#include "calib.h"
#include "control.h"

/* 裸機版的 g_dbg 就是共同的 20 字 */
dbg_common_t g_dbg __attribute__((aligned(4)));

static volatile uint32_t s_tick_ms;

void SysTick_Handler(void)
{
    s_tick_ms++;
}

static uint32_t now_ms(void) { return s_tick_ms; }

/* USART1 收訊走中斷 + ring buffer。
 * 主迴圈以 WFI 收尾(讓模擬器快轉閒置、真硬體省電),醒來的節奏是 SysTick 的 1 ms;
 * 115200 bps 每 ms 進 11 個 byte,而 DR 只裝得下 1 個,輪詢一定掉資料。
 * Renode 的 UART 模型有佇列所以輪詢「看起來也對」——這是模擬器比硬體寬容的地方,
 * 設計要以硬體為準。 */
#define RX_RING 128
static volatile uint8_t s_ring[RX_RING];
static volatile uint32_t s_ring_w, s_ring_r;

void USART1_IRQHandler(void)
{
    while (USART_SR(USART1_BASE) & USART_SR_RXNE) {
        uint8_t b = (uint8_t)(USART_DR(USART1_BASE) & 0xFF);
        uint32_t next = (s_ring_w + 1) % RX_RING;
        if (next != s_ring_r) { s_ring[s_ring_w] = b; s_ring_w = next; }
        /* 滿了就丟,計數留給 g_dbg 看得到 */
        else g_dbg.rx_overflow++;
    }
}

static int uart1_getc(void)
{
    if (s_ring_r == s_ring_w) return -1;
    uint8_t b = s_ring[s_ring_r];
    s_ring_r = (s_ring_r + 1) % RX_RING;
    return b;
}

static rx_t s_rx;

/* ------------------------------------------------------------------------ */
int main(void)
{
    ctl_bind_dbg(&g_dbg);

    /* SysTick 1 kHz */
    SYST_RVR = SYSTICK_HZ / 1000u - 1u;
    SYST_CVR = 0;
    SYST_CSR = SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT | SYST_CSR_ENABLE;

    RCC_APB2ENR |= RCC_APB2ENR_USART1;
    RCC_APB1ENR |= RCC_APB1ENR_USART2;
    ctl_uart_init(USART1_BASE);
    ctl_uart_init(USART2_BASE);
    USART_CR1(USART1_BASE) |= USART_CR1_RXNEIE;
    NVIC_ISER(USART1_IRQN / 32) = 1u << (USART1_IRQN % 32);
    dbg_puts("hilctl boot fw ");
    dbg_put_u32(FW_VERSION_MAJOR); dbg_puts("."); dbg_put_u32(FW_VERSION_MINOR); dbg_puts("\r\n");

    ctl_gpio_init();
#if ENC_SOURCE_TIM
    ctl_encoder_init();
#endif
    ctl_pwm_init();
    if (ctl_can_init(now_ms) == 0) dbg_puts("can1 ready\r\n");
    else { dbg_puts("can1 init FAILED step "); dbg_put_u32(g_dbg.init_err); dbg_puts("\r\n"); }

    ctl_motor_apply(0, 0, 0);

    ctl_boot_detect();
    ctl_iwdg_init_if_enabled();
    dbg_puts("main loop\r\n");

    uint32_t next_ctrl = now_ms() + CONTROL_PERIOD_MS;
    uint32_t next_report = now_ms() + REPORT_PERIOD_MS;
    uint16_t seq = 0;

    for (;;) {
        /* 1. 上位協定 */
        int c;
        while ((c = uart1_getc()) >= 0) {
            if (ctl_proto_feed(&s_rx, (uint8_t)c)) ctl_handle_frame(&s_rx, now_ms());
        }

        /* 2. CAN 編碼器 */
        ctl_can_drain();

        /* 3. 5 ms 控制;4. 20 ms 回報 */
        uint32_t t = now_ms();
        g_dbg.tick_ms = t;
        if ((int32_t)(t - next_ctrl) >= 0) {
            next_ctrl += CONTROL_PERIOD_MS;
            ctl_control_step(t);
            /* 只在控制步真的跑了才餵:主迴圈活著但控制步沒排到,同樣該重置 */
            ctl_iwdg_feed();
        }
        /* 故障注入(只給 IWDG 驗收):模擬韌體死在關中斷的迴圈裡。PWM 週邊還在跑——
         * 馬達會用最後的 duty 一直轉,直到 IWDG 把整顆重置 */
        if (ctl_hang_due(t)) {
            __asm volatile("cpsid i");
            for (;;) { }
        }
        if ((int32_t)(t - next_report) >= 0) {
            next_report += REPORT_PERIOD_MS;
            seq++;
            ctl_report(seq, now_ms());
        }
        /* 睡到下一個中斷(SysTick 1 ms 或 USART1 RX)。 */
        __asm volatile("wfi");
    }
}
