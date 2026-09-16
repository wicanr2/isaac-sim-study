/*
 * hilctl-rtos:與 ../firmware/main.c 同一支差速底盤控制器,改成 FreeRTOS 三個 task。
 *
 *   rx_task     (優先權 2)  USART1 ISR 通知 → 從 ring buffer 解框包 → 更新設定點
 *   ctrl_task   (優先權 3)  每 5 ms 一次(vTaskDelayUntil):讀 CAN 編碼器、PI、PWM、安全閘門
 *   report_task (優先權 1)  每 20 ms 一次:odom(USART1)、馬達狀態(CAN);最低優先,由它餵 IWDG
 *   idle hook                WFI
 *
 * 與裸機版的差別刻意只在「誰排程」:協定、暫存器、控制律、安全閘門、里程計、g_dbg 前 20 個字的版面
 * 全部在 ../firmware/control.c(兩版共用同一個 .c),橋接不用改就能跑。
 * g_dbg 後面多了 RTOS 才有的觀測欄位(deadline miss、stack 餘量、assert 行號)。
 *
 * 沒有「模擬模式」;安全在韌體不在橋接——同 35 篇的三條規則。
 */
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "regs.h"
#include "proto.h"
#include "calib.h"
#include "control.h"

/* ---- 觀測結構:前 20 個字(dbg_common_t)與裸機版逐字相同,橋接靠這個版面;後面是 RTOS 版才有的 ---- */
typedef struct {
    dbg_common_t c;
    volatile uint32_t ctrl_missed;      /* ctrl_task 的 xTaskDelayUntil 回 pdFALSE 的次數(錯過週期) */
    volatile uint32_t report_missed;
    volatile uint32_t rx_wakeups;       /* rx_task 被 ISR 叫醒的次數 */
    volatile uint32_t stack_min_ctrl;   /* 各 task 的 stack high-water mark(word) */
    volatile uint32_t stack_min_rx;
    volatile uint32_t stack_min_report;
    volatile uint32_t assert_line;      /* configASSERT 失敗的行號;0 = 沒有 */
    volatile uint32_t stack_overflow;   /* vApplicationStackOverflowHook 被叫的次數 */
    volatile uint32_t malloc_failed;
} dbg_t;

dbg_t g_dbg __attribute__((aligned(4)));

/* USART1 收訊:ISR 進 ring buffer,然後通知 rx_task。ring 是 ISR→task 單向,
 * 不用 queue(每個 byte 一次 queue 操作太重),用 task notification 當「有資料」的旗號。 */
#define RX_RING 128
static volatile uint8_t s_ring[RX_RING];
static volatile uint32_t s_ring_w, s_ring_r;
static TaskHandle_t s_rx_task;

void USART1_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;
    while (USART_SR(USART1_BASE) & USART_SR_RXNE) {
        uint8_t b = (uint8_t)(USART_DR(USART1_BASE) & 0xFF);
        uint32_t next = (s_ring_w + 1) % RX_RING;
        if (next != s_ring_r) { s_ring[s_ring_w] = b; s_ring_w = next; }
        else g_dbg.c.rx_overflow++;
    }
    if (s_rx_task) vTaskNotifyGiveFromISR(s_rx_task, &woken);
    portYIELD_FROM_ISR(woken);
}

static int uart1_getc(void)
{
    if (s_ring_r == s_ring_w) return -1;
    uint8_t b = s_ring[s_ring_r];
    s_ring_r = (s_ring_r + 1) % RX_RING;
    return b;
}

/* ------------------------------------------------------------------------ */
/* tasks                                                                    */
/* ------------------------------------------------------------------------ */
static void rx_task(void *arg)
{
    (void)arg;
    rx_t rx = {0};
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   /* 睡到 ISR 通知 */
        g_dbg.rx_wakeups++;
        int c;
        while ((c = uart1_getc()) >= 0) {
            if (ctl_proto_feed(&rx, (uint8_t)c)) ctl_handle_frame(&rx, xTaskGetTickCount());
        }
        g_dbg.stack_min_rx = uxTaskGetStackHighWaterMark(NULL);
    }
}

static void ctrl_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        /* xTaskDelayUntil 回 pdFALSE = 這一輪已經晚了,沒有真的睡——那就是錯過週期 */
        if (xTaskDelayUntil(&last, pdMS_TO_TICKS(CONTROL_PERIOD_MS)) == pdFALSE) g_dbg.ctrl_missed++;
        g_dbg.c.tick_ms = xTaskGetTickCount();
        /* CAN 編碼器:控制步開頭把 FIFO 收乾(5 ms 一筆,FIFO 裝 3 筆,輪詢即可);裸機版在主迴圈每次醒來收 */
        ctl_can_drain();
        ctl_control_step(xTaskGetTickCount());
        g_dbg.stack_min_ctrl = uxTaskGetStackHighWaterMark(NULL);
        /* 故障注入(只給 IWDG 驗收):最高優先的 task 關中斷死迴圈,其他 task 全部餓死,沒人餵狗 */
        if (ctl_hang_due(g_dbg.c.tick_ms)) {
            __asm volatile("cpsid i");
            for (;;) { }
        }
    }
}

static void report_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    uint16_t seq = 0;
    for (;;) {
        if (xTaskDelayUntil(&last, pdMS_TO_TICKS(REPORT_PERIOD_MS)) == pdFALSE) g_dbg.report_missed++;
        ctl_iwdg_feed();   /* 最低優先的 task 才餵得到 = 整個系統還在排程 */
        seq++;
        ctl_report(seq, xTaskGetTickCount());
        g_dbg.stack_min_report = uxTaskGetStackHighWaterMark(NULL);
    }
}

/* ---- hooks ---------------------------------------------------------------- */
void vApplicationIdleHook(void) { __asm volatile("wfi"); }

void vApplicationStackOverflowHook(TaskHandle_t t, char *name)
{
    (void)t; (void)name;
    g_dbg.stack_overflow++;
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}

void vApplicationMallocFailedHook(void)
{
    g_dbg.malloc_failed++;
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}

void hil_assert_failed(const char *file, int line)
{
    (void)file;
    g_dbg.assert_line = (uint32_t)line;
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}

int main(void)
{
    ctl_bind_dbg(&g_dbg.c);

    RCC_APB2ENR |= RCC_APB2ENR_USART1;
    RCC_APB1ENR |= RCC_APB1ENR_USART2;
    ctl_uart_init(USART1_BASE);
    ctl_uart_init(USART2_BASE);
    dbg_puts("hilctl-rtos boot fw "); dbg_put_u32(FW_VERSION_MAJOR); dbg_puts("."); dbg_put_u32(FW_VERSION_MINOR);
    dbg_puts(" FreeRTOS " tskKERNEL_VERSION_NUMBER "\r\n");

    ctl_gpio_init();
#if ENC_SOURCE_TIM
    ctl_encoder_init();
#endif
    ctl_pwm_init();
    /* CAN 交握逾時用 tick 計,而 tick 要 scheduler 起來才走 → 給 NULL,control.c 用固定次數的輪詢 */
    ctl_can_init(0);
    dbg_puts(g_dbg.c.init_err ? "can1 init FAILED\r\n" : "can1 ready\r\n");

    ctl_motor_apply(0, 0, 0);

    ctl_boot_detect();
    ctl_iwdg_init_if_enabled();

    /* USART1 中斷:優先權 6(數值 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 才能用 FromISR API) */
    USART_CR1(USART1_BASE) |= USART_CR1_RXNEIE;
    NVIC_IPR(USART1_IRQN) = (uint8_t)(6u << 4);
    NVIC_ISER(USART1_IRQN / 32) = 1u << (USART1_IRQN % 32);

    xTaskCreate(rx_task,     "rx",     256, NULL, 2, &s_rx_task);
    xTaskCreate(ctrl_task,   "ctrl",   256, NULL, 3, NULL);
    xTaskCreate(report_task, "report", 256, NULL, 1, NULL);
    dbg_puts("scheduler start\r\n");
    vTaskStartScheduler();
    /* 只有記憶體不夠建 idle task 才會回來 */
    dbg_puts("scheduler returned\r\n");
    for (;;) { }
}
