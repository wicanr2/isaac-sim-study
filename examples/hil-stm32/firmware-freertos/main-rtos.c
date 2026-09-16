/*
 * hilctl-rtos:與 ../firmware/main.c 同一支差速底盤控制器,改成 FreeRTOS 三個 task。
 *
 *   rx_task     (優先權 2)  USART1 ISR 通知 → 從 ring buffer 解框包 → 更新設定點
 *   ctrl_task   (優先權 3)  每 5 ms 一次(vTaskDelayUntil):讀 CAN 編碼器、PI、PWM、安全閘門
 *   report_task (優先權 1)  每 20 ms 一次:odom(USART1)、馬達狀態(CAN)
 *   idle hook                WFI
 *
 * 與裸機版的差別刻意只在「誰排程」:協定、暫存器、控制律、g_dbg 前 17 個字的版面全部相同,
 * 橋接不用改就能跑。g_dbg 後面多了 RTOS 才有的觀測欄位(deadline miss、stack 餘量、assert 行號)。
 *
 * 沒有「模擬模式」;安全在韌體不在橋接——同 35 篇的三條規則。
 */
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "regs.h"
#include "proto.h"
#include "calib.h"
#include "crc16.h"

/* ---- 觀測結構:前 17 個字與裸機版逐字相同(橋接靠這個版面) ------------------------- */
typedef struct {
    volatile uint32_t magic;
    volatile uint32_t tick_ms;
    volatile uint32_t ctrl_steps;
    volatile int32_t  sp_l, sp_r;
    volatile int32_t  meas_l, meas_r;
    volatile int32_t  duty_l, duty_r;
    volatile int32_t  enc_l, enc_r;
    volatile uint32_t enc_frames;
    volatile uint32_t cmd_frames;
    volatile uint32_t bad_crc;
    volatile uint32_t flags;
    volatile uint32_t init_err;
    volatile uint32_t rx_overflow;
    /* ---- RTOS 版才有(第 17 字起) ---- */
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

/* 執行期可改的控制參數(同裸機版):預設從 calib 來,橋接可在開機後經 External Control 寫 SRAM 覆蓋 */
typedef struct {
    volatile uint32_t magic;        /* 0x48494C43 "HILC" */
    volatile int32_t  kp_q8, ki_q8;
    volatile int32_t  accel_mm_s2;  /* 線速度斜坡上限,0 = 不限 */
    volatile int32_t  ff_q8;        /* 前饋比例,256 = 100% */
    volatile int32_t  alpha_mrad_s2;/* 角速度斜坡上限,0 = 不限 */
} cfg_t;

cfg_t g_cfg __attribute__((aligned(4))) = { 0x48494C43u, PI_KP_Q8, PI_KI_Q8, ACCEL_LIMIT_MM_S2, FF_GAIN_Q8, ALPHA_LIMIT_MRAD_S2 };

/* ------------------------------------------------------------------------ */
/* 與裸機版相同的硬體層(複製而非共用 .c:兩支韌體要各自完整、可獨立閱讀) */
/* ------------------------------------------------------------------------ */
static void uart_init(uint32_t base)
{
    USART_BRR(base) = 0x16D;
    USART_CR1(base) = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static void uart_putc(uint32_t base, uint8_t c)
{
    uint32_t spin = 0;
    while (!(USART_SR(base) & USART_SR_TXE) && ++spin < 100000) { }
    USART_DR(base) = c;
}

static void dbg_puts(const char *s)
{
    while (*s) uart_putc(USART2_BASE, (uint8_t)*s++);
}

static void dbg_put_u32(uint32_t v)
{
    char buf[11]; int i = 10; buf[i] = 0;
    do { buf[--i] = (char)('0' + v % 10); v /= 10; } while (v);
    dbg_puts(&buf[i]);
}

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
        else g_dbg.rx_overflow++;
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

static void proto_send(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[2 + PROTO_MAX_PAYLOAD];
    buf[0] = type;
    for (uint8_t i = 0; i < len; i++) buf[1 + i] = payload[i];
    uint16_t crc = crc16_modbus(buf, (uint32_t)len + 1);
    /* 送一個框包期間不讓別的 task 插進來送:USART1 只有一條 TX 線 */
    taskENTER_CRITICAL();
    uart_putc(USART1_BASE, PROTO_SYNC0);
    uart_putc(USART1_BASE, PROTO_SYNC1);
    uart_putc(USART1_BASE, len);
    for (uint32_t i = 0; i < (uint32_t)len + 1; i++) uart_putc(USART1_BASE, buf[i]);
    uart_putc(USART1_BASE, (uint8_t)(crc & 0xFF));
    uart_putc(USART1_BASE, (uint8_t)(crc >> 8));
    taskEXIT_CRITICAL();
}

typedef struct {
    uint8_t state, len, type, idx;
    uint8_t payload[PROTO_MAX_PAYLOAD];
    uint8_t crc_lo;
} rx_t;

static int proto_feed(rx_t *r, uint8_t b)
{
    switch (r->state) {
    case 0: if (b == PROTO_SYNC0) r->state = 1; break;
    case 1: r->state = (b == PROTO_SYNC1) ? 2 : 0; break;
    case 2: if (b > PROTO_MAX_PAYLOAD) { r->state = 0; break; }
            r->len = b; r->idx = 0; r->state = 3; break;
    case 3: r->type = b; r->state = (r->len ? 4 : 5); break;
    case 4: r->payload[r->idx++] = b; if (r->idx >= r->len) r->state = 5; break;
    case 5: r->crc_lo = b; r->state = 6; break;
    case 6: {
        uint8_t tmp[1 + PROTO_MAX_PAYLOAD];
        tmp[0] = r->type;
        for (uint8_t i = 0; i < r->len; i++) tmp[1 + i] = r->payload[i];
        uint16_t want = crc16_modbus(tmp, (uint32_t)r->len + 1);
        uint16_t got  = (uint16_t)(r->crc_lo | ((uint16_t)b << 8));
        r->state = 0;
        if (want == got) return 1;
        g_dbg.bad_crc++;
        return 0;
    }
    }
    return 0;
}

static void gpio_init(void)
{
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOA | RCC_AHB1ENR_GPIOB | RCC_AHB1ENR_GPIOC;
    GPIO_MODER(GPIOA_BASE) = (GPIO_MODER(GPIOA_BASE) & ~((3u << 12) | (3u << 14))) | (2u << 12) | (2u << 14);
    GPIO_AFRL(GPIOA_BASE)  = (GPIO_AFRL(GPIOA_BASE) & ~((0xFu << 24) | (0xFu << 28))) | (2u << 24) | (2u << 28);
    GPIO_MODER(GPIOB_BASE) = (GPIO_MODER(GPIOB_BASE) & ~((3u << 16) | (3u << 18) | (3u << 20)))
                             | (1u << 16) | (1u << 18) | (1u << 20);
    GPIO_BSRR(GPIOB_BASE) = (1u << (DIR_L_PIN + 16)) | (1u << (DIR_R_PIN + 16)) | (1u << (MOTOR_EN_PIN + 16));
    GPIO_MODER(GPIOC_BASE) &= ~(3u << 26);
#if ENC_SOURCE_TIM
    /* PA0/PA1 → AF1(TIM2_CH1/CH2)左輪編碼器;PB6/PB7 → AF2(TIM4_CH1/CH2)右輪——同裸機版 */
    GPIO_MODER(GPIOA_BASE) = (GPIO_MODER(GPIOA_BASE) & ~((3u << 0) | (3u << 2))) | (2u << 0) | (2u << 2);
    GPIO_AFRL(GPIOA_BASE)  = (GPIO_AFRL(GPIOA_BASE) & ~((0xFu << 0) | (0xFu << 4))) | (1u << 0) | (1u << 4);
    GPIO_MODER(GPIOB_BASE) = (GPIO_MODER(GPIOB_BASE) & ~((3u << 12) | (3u << 14))) | (2u << 12) | (2u << 14);
    GPIO_AFRL(GPIOB_BASE)  = (GPIO_AFRL(GPIOB_BASE) & ~((0xFu << 24) | (0xFu << 28))) | (2u << 24) | (2u << 28);
#endif
}

#if ENC_SOURCE_TIM
static void encoder_tim_init(uint32_t base)
{
    TIM_CR1(base)   = 0;
    TIM_ARR(base)   = 0xFFFF;
    TIM_CCMR1(base) = TIM_CCMR1_CC1S_TI1 | TIM_CCMR1_CC2S_TI2;
    TIM_CCER(base)  = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM_SMCR(base)  = TIM_SMCR_SMS_ENCODER3;
    TIM_CNT(base)   = 0;
    TIM_CR1(base)   = TIM_CR1_CEN;
}

static void encoder_init(void)
{
    RCC_APB1ENR |= RCC_APB1ENR_TIM2 | RCC_APB1ENR_TIM4;
    encoder_tim_init(TIM2_BASE);
    encoder_tim_init(TIM4_BASE);
}
#endif

static void pwm_init(void)
{
    RCC_APB1ENR |= RCC_APB1ENR_TIM3;
    TIM3_PSC   = PWM_PSC;
    TIM3_ARR   = PWM_ARR;
    TIM3_CCR1  = 0;
    TIM3_CCR2  = 0;
    TIM3_CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
    TIM3_CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM3_EGR   = TIM_EGR_UG;
    TIM3_CR1   = TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void motor_apply(int32_t duty_l, int32_t duty_r, int enable)
{
    uint32_t set = 0, clr = 0;
    if (duty_l >= 0) set |= 1u << DIR_L_PIN; else clr |= 1u << DIR_L_PIN;
    if (duty_r >= 0) set |= 1u << DIR_R_PIN; else clr |= 1u << DIR_R_PIN;
    if (enable)      set |= 1u << MOTOR_EN_PIN; else clr |= 1u << MOTOR_EN_PIN;
    GPIO_BSRR(GPIOB_BASE) = set | (clr << 16);
    uint32_t dl = (uint32_t)(duty_l < 0 ? -duty_l : duty_l);
    uint32_t dr = (uint32_t)(duty_r < 0 ? -duty_r : duty_r);
    TIM3_CCR1 = dl * (uint32_t)(PWM_ARR + 1) / DUTY_FULL_SCALE;
    TIM3_CCR2 = dr * (uint32_t)(PWM_ARR + 1) / DUTY_FULL_SCALE;
}

static int estop_asserted(void) { return (GPIO_IDR(GPIOC_BASE) >> ESTOP_PIN) & 1u; }

static int can_send(uint32_t std_id, const uint8_t *d, uint8_t dlc)
{
    if (!(CAN_TSR & CAN_TSR_TME0)) return -1;
    CAN_TDT0R = dlc;
    CAN_TDL0R = (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
    CAN_TDH0R = (uint32_t)d[4] | ((uint32_t)d[5] << 8) | ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
    CAN_TI0R  = (std_id << 21) | CAN_TI0R_TXRQ;
    return 0;
}

static int can_recv(uint32_t *out_id, uint8_t *out_d, uint8_t *out_dlc)
{
    if ((CAN_RF0R & CAN_RF0R_FMP0_MASK) == 0) return 0;
    uint32_t ri = CAN_RI0R, lo = CAN_RDL0R, hi = CAN_RDH0R;
    *out_dlc = (uint8_t)(CAN_RDT0R & 0xF);
    *out_id  = ri >> 21;
    for (int i = 0; i < 4; i++) { out_d[i] = (uint8_t)(lo >> (8 * i)); out_d[4 + i] = (uint8_t)(hi >> (8 * i)); }
    CAN_RF0R = CAN_RF0R_RFOM0;
    return 1;
}

/* ------------------------------------------------------------------------ */
/* 控制狀態:ctrl_task 擁有;rx_task 只寫設定點與時間戳(32-bit 寫入在 Cortex-M 是原子的) */
/* ------------------------------------------------------------------------ */
static volatile int32_t s_sp_l, s_sp_r;
static volatile TickType_t s_last_cmd_tick;
static volatile int s_have_cmd;
static int32_t s_integ_l, s_integ_r;
static int32_t s_cmd_v, s_cmd_w;           /* 上位命令(mm/s、mrad/s) */
static int32_t s_v_ramp, s_w_ramp;         /* 斜坡後的 v、w */
static int32_t s_enc_l, s_enc_r;
#if !ENC_SOURCE_TIM
static int32_t s_enc_prev_l, s_enc_prev_r;
#endif
#if ENC_SOURCE_TIM
static uint16_t s_cnt_prev_l, s_cnt_prev_r;
#endif
static int32_t s_meas_l, s_meas_r, s_duty_l, s_duty_r;
static float s_x_mm, s_y_mm, s_th_rad;

static float fsin(float x)
{
    const float PI = 3.14159265f, TWO_PI = 6.28318531f;
    while (x >  PI) x -= TWO_PI;
    while (x < -PI) x += TWO_PI;
    float x2 = x * x;
    return x * (1.0f - x2 / 6.0f * (1.0f - x2 / 20.0f * (1.0f - x2 / 42.0f * (1.0f - x2 / 72.0f))));
}
static float fcos(float x) { return fsin(x + 1.57079633f); }
static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

static int32_t pi_step(int32_t sp, int32_t meas, int32_t *integ)
{
    int32_t e = sp - meas;
    *integ = clamp_i32(*integ + e, -PI_INTEGRAL_LIMIT, PI_INTEGRAL_LIMIT);
    int32_t ff = (sp * DUTY_FULL_SCALE / WHEEL_SPEED_FULL_MM_S) * g_cfg.ff_q8 >> 8;
    int32_t u  = ff + ((g_cfg.kp_q8 * e + g_cfg.ki_q8 * (*integ)) >> 8);
    return clamp_i32(u, -DUTY_FULL_SCALE, DUTY_FULL_SCALE);
}

static void control_step(void)
{
    /* CAN 編碼器:控制步開頭把 FIFO 收乾(5 ms 一筆,FIFO 裝 3 筆,輪詢即可) */
    uint32_t id; uint8_t d[8]; uint8_t dlc; uint32_t enc_new = 0;
    while (can_recv(&id, d, &dlc)) {
        if (id == CAN_ID_ENCODER && dlc == 8) {
#if !ENC_SOURCE_TIM
            s_enc_l = (int32_t)((uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24));
            s_enc_r = (int32_t)((uint32_t)d[4] | ((uint32_t)d[5] << 8) | ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24));
            g_dbg.enc_l = s_enc_l; g_dbg.enc_r = s_enc_r;
#endif
            g_dbg.enc_frames++;
#if !ENC_SOURCE_TIM
            enc_new++;
#endif
        }
    }

    /* 時間基準用「收到幾筆訊框」(協定:每 CONTROL_PERIOD_MS 一筆),不用控制週期數;
     * 0 筆就沿用上次量測——同裸機版 firmware/main.c */
#if ENC_SOURCE_TIM
    uint16_t cl = (uint16_t)TIM_CNT(TIM2_BASE), cr = (uint16_t)TIM_CNT(TIM4_BASE);
    int32_t dl = (int16_t)(cl - s_cnt_prev_l), dr = (int16_t)(cr - s_cnt_prev_r);
    s_cnt_prev_l = cl; s_cnt_prev_r = cr;
    s_enc_l += dl; s_enc_r += dr;
    g_dbg.enc_l = s_enc_l; g_dbg.enc_r = s_enc_r;
    enc_new = 1;
#else
    int32_t dl = s_enc_l - s_enc_prev_l, dr = s_enc_r - s_enc_prev_r;
    s_enc_prev_l = s_enc_l; s_enc_prev_r = s_enc_r;
#endif
    if (enc_new) {
        s_meas_l = (int32_t)((int64_t)dl * WHEEL_CIRC_UM / ENC_TICKS_PER_REV / (CONTROL_PERIOD_MS * (int32_t)enc_new));
        s_meas_r = (int32_t)((int64_t)dr * WHEEL_CIRC_UM / ENC_TICKS_PER_REV / (CONTROL_PERIOD_MS * (int32_t)enc_new));
    }

    float dl_mm = (float)dl * ((float)WHEEL_CIRC_UM / 1000.0f) / (float)ENC_TICKS_PER_REV;
    float dr_mm = (float)dr * ((float)WHEEL_CIRC_UM / 1000.0f) / (float)ENC_TICKS_PER_REV;
    float ds = (dl_mm + dr_mm) * 0.5f;
    float dth = (dr_mm - dl_mm) / (float)TRACK_MM;
    float th_mid = s_th_rad + dth * 0.5f;
    s_x_mm += ds * fcos(th_mid);
    s_y_mm += ds * fsin(th_mid);
    s_th_rad += dth;

    uint32_t flags = 0;
    int enable = 1;
    if (estop_asserted()) { flags |= ODOM_FLAG_ESTOP; enable = 0; }
    if (!s_have_cmd || (xTaskGetTickCount() - s_last_cmd_tick) > pdMS_TO_TICKS(CMD_TIMEOUT_MS)) {
        flags |= ODOM_FLAG_CMD_STALE; enable = 0;
    }
    if (enable) {
        flags |= ODOM_FLAG_ENABLED;
        /* 斜坡對 v(mm/s²)與 w(mrad/s²)各自限斜率,再換成兩輪設定點——對兩輪各自限會讓
         * 轉→直的過渡兩輪不同步(量到方形每段偏航 +0.16 rad);0 = 直接跳(第一版行為) */
        int32_t dv = g_cfg.accel_mm_s2 * CONTROL_PERIOD_MS / 1000;
        int32_t dw = g_cfg.alpha_mrad_s2 * CONTROL_PERIOD_MS / 1000;
        s_v_ramp = dv > 0 ? s_v_ramp + clamp_i32(s_cmd_v - s_v_ramp, -dv, dv) : s_cmd_v;
        s_w_ramp = dw > 0 ? s_w_ramp + clamp_i32(s_cmd_w - s_w_ramp, -dw, dw) : s_cmd_w;
        /* v_l = v - w*track/2,w 是 mrad/s → (w * TRACK_MM / 2) / 1000 mm/s */
        int32_t half = s_w_ramp * TRACK_MM / 2 / 1000;
        s_sp_l = s_v_ramp - half;
        s_sp_r = s_v_ramp + half;
        /* 設定點歸零時清積分:轉向段留下的左右不對稱積分,會讓下一段直線一起步就偏航
         * (方形每段量到 +0.07 rad);真板驅動器同樣在零命令時清 */
        if (s_sp_l == 0) s_integ_l = 0;
        if (s_sp_r == 0) s_integ_r = 0;
        s_duty_l = pi_step(s_sp_l, s_meas_l, &s_integ_l);
        s_duty_r = pi_step(s_sp_r, s_meas_r, &s_integ_r);
    } else {
        s_duty_l = s_duty_r = 0;
        s_integ_l = s_integ_r = 0;
        s_v_ramp = s_w_ramp = 0;   /* 停車後從 0 重新起坡 */
        s_sp_l = s_sp_r = 0;
    }
    motor_apply(s_duty_l, s_duty_r, enable);

    g_dbg.ctrl_steps++;
    g_dbg.sp_l = s_sp_l;   g_dbg.sp_r = s_sp_r;
    g_dbg.meas_l = s_meas_l; g_dbg.meas_r = s_meas_r;
    g_dbg.duty_l = s_duty_l; g_dbg.duty_r = s_duty_r;
    g_dbg.flags = flags;
}

static void handle_cmd_vel(const uint8_t *p)
{
    int16_t v = (int16_t)(p[0] | (p[1] << 8));
    int16_t w = (int16_t)(p[2] | (p[3] << 8));
    s_cmd_v = v; s_cmd_w = w;   /* 斜坡與換算在控制步做(對 v、w 各自限斜率,兩輪才不會在過渡時不同步) */
    s_last_cmd_tick = xTaskGetTickCount();
    s_have_cmd = 1;
    g_dbg.cmd_frames++;
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
            if (proto_feed(&rx, (uint8_t)c)) {
                if (rx.type == MSG_CMD_VEL && rx.len == 4) handle_cmd_vel(rx.payload);
                else if (rx.type == MSG_PING) {
                    uint8_t v[2] = { FW_VERSION_MAJOR, FW_VERSION_MINOR };
                    proto_send(MSG_PONG, v, 2);
                }
            }
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
        g_dbg.tick_ms = xTaskGetTickCount();
        control_step();
        g_dbg.stack_min_ctrl = uxTaskGetStackHighWaterMark(NULL);
    }
}

static void report_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    uint16_t seq = 0;
    for (;;) {
        if (xTaskDelayUntil(&last, pdMS_TO_TICKS(REPORT_PERIOD_MS)) == pdFALSE) g_dbg.report_missed++;
        seq++;
        odom_payload_t o;
        o.seq = seq;
        o.t_ms = xTaskGetTickCount();
        o.x_mm = (int32_t)s_x_mm;
        o.y_mm = (int32_t)s_y_mm;
        o.th_mrad = (int32_t)(s_th_rad * 1000.0f);
        o.vl_mm_s = (int16_t)s_meas_l;
        o.vr_mm_s = (int16_t)s_meas_r;
        o.flags = (uint8_t)g_dbg.flags;
        proto_send(MSG_ODOM, (const uint8_t *)&o, sizeof o);

        uint8_t d[8];
        d[0] = (uint8_t)(s_duty_l & 0xFF); d[1] = (uint8_t)((s_duty_l >> 8) & 0xFF);
        d[2] = (uint8_t)(s_duty_r & 0xFF); d[3] = (uint8_t)((s_duty_r >> 8) & 0xFF);
        d[4] = (uint8_t)g_dbg.flags; d[5] = (uint8_t)seq; d[6] = 0; d[7] = 0;
        can_send(CAN_ID_MOTOR_STATUS, d, 8);
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
    g_dbg.magic = 0x48494C31u;

    RCC_APB2ENR |= RCC_APB2ENR_USART1;
    RCC_APB1ENR |= RCC_APB1ENR_USART2;
    uart_init(USART1_BASE);
    uart_init(USART2_BASE);
    dbg_puts("hilctl-rtos boot fw "); dbg_put_u32(FW_VERSION_MAJOR); dbg_puts("."); dbg_put_u32(FW_VERSION_MINOR);
    dbg_puts(" FreeRTOS " tskKERNEL_VERSION_NUMBER "\r\n");

    gpio_init();
#if ENC_SOURCE_TIM
    encoder_init();
#endif
    pwm_init();
    /* CAN 初始化用 tick 計逾時,而 tick 要 scheduler 起來才走。這裡先用忙等版:
     * 初始化階段的逾時改用固定次數的輪詢 */
    RCC_APB1ENR |= RCC_APB1ENR_CAN1;
    CAN_MCR &= ~CAN_MCR_SLEEP;
    { uint32_t n = 0; while ((CAN_MSR & CAN_MSR_SLAK) && ++n < 100000) { } if (CAN_MSR & CAN_MSR_SLAK) g_dbg.init_err = 1; }
    CAN_MCR |= CAN_MCR_INRQ;
    { uint32_t n = 0; while (!(CAN_MSR & CAN_MSR_INAK) && ++n < 100000) { } if (!(CAN_MSR & CAN_MSR_INAK)) g_dbg.init_err = 2; }
    CAN_BTR = (1u << 20) | (10u << 16) | 5u;
    CAN_MCR &= ~CAN_MCR_INRQ;
    { uint32_t n = 0; while ((CAN_MSR & CAN_MSR_INAK) && ++n < 100000) { } if (CAN_MSR & CAN_MSR_INAK) g_dbg.init_err = 3; }
    CAN_FMR  |= CAN_FMR_FINIT;
    CAN_FA1R &= ~1u; CAN_FM1R &= ~1u; CAN_FS1R |= 1u; CAN_FFA1R &= ~1u;
    CAN_F0R1 = 0; CAN_F0R2 = 0;
    CAN_FA1R |= 1u;
    CAN_FMR  &= ~CAN_FMR_FINIT;
    dbg_puts(g_dbg.init_err ? "can1 init FAILED\r\n" : "can1 ready\r\n");

    motor_apply(0, 0, 0);

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
