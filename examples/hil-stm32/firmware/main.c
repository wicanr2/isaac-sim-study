/*
 * hilctl:差速底盤控制器的最小韌體(STM32F4,裸機,無 HAL / libc)。
 *
 * 職責與真實下位機一樣:
 *   上位(USART1)給 cmd_vel  →  兩輪速度設定點  →  每 5 ms 一次 PI  →  PWM + 方向腳
 *   編碼器(CAN 0x181)回累計 tick  →  輪速量測 + 里程計  →  每 20 ms 回報 odom(USART1)
 *   安全:500 ms 沒命令 → 停;急停腳(PC13)拉高 → 停;驅動器故障腳(PC14/15)低 → 停;
 *         保險桿(PC0)斷 → 拒絕前進;堵轉(duty 高而輪不動 200 ms)→ 停到上位歸零;
 *         上位心跳(PING)300 ms 沒來 → 降速到 0;IWDG 1 s 沒餵 → 整顆重置。安全在這裡,不在橋接。
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
#include "crc16.h"

/* ------------------------------------------------------------------------ */
/* 觀測用的全域狀態:volatile,固定版面,橋接以 sysbus 讀                       */
/* ------------------------------------------------------------------------ */
typedef struct {
    volatile uint32_t magic;        /* 0x48494C31 "HIL1":橋接用來確認讀對位址 */
    volatile uint32_t tick_ms;      /* SysTick 毫秒計數 */
    volatile uint32_t ctrl_steps;   /* 控制迴圈執行次數 */
    volatile int32_t  sp_l, sp_r;   /* 輪速設定點 mm/s */
    volatile int32_t  meas_l, meas_r;/* 輪速量測 mm/s */
    volatile int32_t  duty_l, duty_r;/* 帶號 duty,±DUTY_FULL_SCALE */
    volatile int32_t  enc_l, enc_r; /* 最後收到的累計 tick */
    volatile uint32_t enc_frames;   /* 收到的編碼器訊框數 */
    volatile uint32_t cmd_frames;   /* 收到的 cmd_vel 數 */
    volatile uint32_t bad_crc;      /* CRC 錯的框包數 */
    volatile uint32_t flags;        /* 同 odom flags */
    volatile uint32_t init_err;     /* 初始化哪一步逾時(0 = 沒有) */
    volatile uint32_t rx_overflow;  /* USART1 ring buffer 滿而丟掉的 byte 數 */
    volatile uint32_t resets;       /* 暖重置次數(.noinit 計數;IWDG 驗收靠它) */
    volatile uint32_t boot_csr;     /* 開機時讀到的 RCC_CSR(真板 IWDGRSTF 在 bit29;Renode 讀到 0,紀錄用) */
    volatile uint32_t ping_frames;  /* 收到的 PING 數(心跳) */
} dbg_t;

dbg_t g_dbg __attribute__((aligned(4)));

/* 跨 reset 保留的區段:startup 不清、LoadELF 不寫。magic 對就是暖重置。 */
typedef struct {
    volatile uint32_t magic;        /* 0x4E4F494E "NOIN" */
    volatile uint32_t resets;
} noinit_t;

noinit_t g_noinit __attribute__((section(".noinit"), aligned(4)));

/* 執行期可改的控制參數:預設從 calib 來,橋接可在開機後經 External Control 寫 SRAM 覆蓋
 * (--cfg kp=..,ki=..,accel=..,ff=..),讓增益掃描與「關掉斜坡」的負對照不用重編韌體。
 * 每個控制步都重新讀,所以任何時候改都生效。 */
typedef struct {
    volatile uint32_t magic;        /* 0x48494C43 "HILC" */
    volatile int32_t  kp_q8, ki_q8;
    volatile int32_t  accel_mm_s2;  /* 線速度斜坡上限,0 = 不限 */
    volatile int32_t  ff_q8;        /* 前饋比例,256 = 100% */
    volatile int32_t  alpha_mrad_s2;/* 角速度斜坡上限,0 = 不限 */
    /* ---- 安全 I/O(第 6 字起)。橋接在開機前把 --cfg 寫進 flash 裡 .data 的初始值(LMA),
     * startup 照常複製,所以 IWDG 這種「init 就定案」的參數也改得到 ---- */
    volatile int32_t  iwdg_ms;      /* IWDG 逾時;由 safety_mask 決定開不開 */
    volatile int32_t  hang_at_ms;   /* 故障注入:tick 到這個值時關中斷死迴圈(0 = 不注入)。只給 IWDG 驗收用 */
    volatile int32_t  hb_timeout_ms;/* 心跳逾時;0 = 不驗 */
    volatile int32_t  stall_duty;   /* 堵轉判定 duty 門檻(‰) */
    volatile int32_t  stall_ms;     /* 堵轉判定持續時間 */
    volatile uint32_t safety_mask;  /* SAFETY_*,全開 0x1F;負對照關一項 */
} cfg_t;

cfg_t g_cfg __attribute__((aligned(4))) = { 0x48494C43u, PI_KP_Q8, PI_KI_Q8, ACCEL_LIMIT_MM_S2, FF_GAIN_Q8, ALPHA_LIMIT_MRAD_S2,
                                            IWDG_TIMEOUT_MS, 0, HB_TIMEOUT_MS, STALL_DUTY, STALL_MS, SAFETY_MASK };

static volatile uint32_t s_tick_ms;

void SysTick_Handler(void)
{
    s_tick_ms++;
}

static uint32_t now_ms(void) { return s_tick_ms; }

/* ------------------------------------------------------------------------ */
/* USART                                                                    */
/* ------------------------------------------------------------------------ */
static void uart_init(uint32_t base)
{
    /* 鮑率不影響 Renode;真硬體才要按 APB 時脈算。這裡放 115200 @ 42 MHz 的值。 */
    USART_BRR(base) = 0x16D;
    USART_CR1(base) = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

/* 輪詢 TXE,不等 TC:既有紀錄指出 Renode 1.16.1 的 STM32_UART 把 TC 做成永不重設,
 * 等 TC 的傳送路徑會卡死。輪詢 TXE 在真硬體同樣正確,只是佔 CPU。 */
static void uart_putc(uint32_t base, uint8_t c)
{
    uint32_t spin = 0;
    while (!(USART_SR(base) & USART_SR_TXE) && ++spin < 100000) { }
    USART_DR(base) = c;
}

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

/* ------------------------------------------------------------------------ */
/* 上位協定:框包收發                                                        */
/* ------------------------------------------------------------------------ */
static void proto_send(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[2 + PROTO_MAX_PAYLOAD];
    buf[0] = type;
    for (uint8_t i = 0; i < len; i++) buf[1 + i] = payload[i];
    uint16_t crc = crc16_modbus(buf, (uint32_t)len + 1);

    uart_putc(USART1_BASE, PROTO_SYNC0);
    uart_putc(USART1_BASE, PROTO_SYNC1);
    uart_putc(USART1_BASE, len);
    for (uint32_t i = 0; i < (uint32_t)len + 1; i++) uart_putc(USART1_BASE, buf[i]);
    uart_putc(USART1_BASE, (uint8_t)(crc & 0xFF));
    uart_putc(USART1_BASE, (uint8_t)(crc >> 8));
}

/* 接收狀態機:一次餵一個 byte,收齊且 CRC 對才回 1 */
typedef struct {
    uint8_t state, len, type, idx;
    uint8_t payload[PROTO_MAX_PAYLOAD];
    uint8_t crc_lo;
} rx_t;

static rx_t s_rx;

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

/* ------------------------------------------------------------------------ */
/* GPIO / PWM                                                               */
/* ------------------------------------------------------------------------ */
static void gpio_init(void)
{
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOA | RCC_AHB1ENR_GPIOB | RCC_AHB1ENR_GPIOC;

    /* PA6/PA7 → AF2(TIM3_CH1/CH2) */
    GPIO_MODER(GPIOA_BASE) = (GPIO_MODER(GPIOA_BASE) & ~((3u << 12) | (3u << 14)))
                             | (2u << 12) | (2u << 14);
    GPIO_AFRL(GPIOA_BASE)  = (GPIO_AFRL(GPIOA_BASE) & ~((0xFu << 24) | (0xFu << 28)))
                             | (2u << 24) | (2u << 28);

    /* PB8/PB9 方向、PB10 致能 → 一般輸出 */
    GPIO_MODER(GPIOB_BASE) = (GPIO_MODER(GPIOB_BASE)
                              & ~((3u << 16) | (3u << 18) | (3u << 20)))
                             | (1u << 16) | (1u << 18) | (1u << 20);
    GPIO_BSRR(GPIOB_BASE) = (1u << (DIR_L_PIN + 16)) | (1u << (DIR_R_PIN + 16))
                          | (1u << (MOTOR_EN_PIN + 16));

    /* PC13 急停、PC14/PC15 驅動器故障、PC0 保險桿 → 輸入(MODER 00,重置值就是)。
     * 故障腳與保險桿低有效,開 pull-up:沒接東西時讀 1 = 正常。Renode 的 GPIO 不看 PUPDR,
     * 輸入腳的預設是 0,所以橋接開機前要先把這三腳拉高(等於接上了 pull-up)。 */
    GPIO_MODER(GPIOC_BASE) &= ~((3u << 26) | (3u << 28) | (3u << 30) | (3u << 0));
    GPIO_PUPDR(GPIOC_BASE) = (GPIO_PUPDR(GPIOC_BASE) & ~((3u << 28) | (3u << 30) | (3u << 0)))
                             | (1u << 28) | (1u << 30) | (1u << 0);

#if ENC_SOURCE_TIM
    /* PA0/PA1 → AF1(TIM2_CH1/CH2)左輪編碼器;PB6/PB7 → AF2(TIM4_CH1/CH2)右輪 */
    GPIO_MODER(GPIOA_BASE) = (GPIO_MODER(GPIOA_BASE) & ~((3u << 0) | (3u << 2))) | (2u << 0) | (2u << 2);
    GPIO_AFRL(GPIOA_BASE)  = (GPIO_AFRL(GPIOA_BASE) & ~((0xFu << 0) | (0xFu << 4))) | (1u << 0) | (1u << 4);
    GPIO_MODER(GPIOB_BASE) = (GPIO_MODER(GPIOB_BASE) & ~((3u << 12) | (3u << 14))) | (2u << 12) | (2u << 14);
    GPIO_AFRL(GPIOB_BASE)  = (GPIO_AFRL(GPIOB_BASE) & ~((0xFu << 24) | (0xFu << 28))) | (2u << 24) | (2u << 28);
#endif
}

#if ENC_SOURCE_TIM
/* 編碼器:TIM encoder mode(RM0090 §18.3.12)。CC1S/CC2S=01 把 TI1/TI2 當輸入,SMS=011 兩路邊緣都計數,
 * 16 位元計數器 0..0xFFFF 繞回;控制步讀 CNT 差(int16)。 */
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
    /* OC1PE/OC2PE 在 Renode 1.16.1 未實作(只警告);真硬體要它才能無毛刺換 duty */
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

static int estop_asserted(void)
{
    return (GPIO_IDR(GPIOC_BASE) >> ESTOP_PIN) & 1u;
}

static int drv_fault_asserted(void)
{
    uint32_t idr = GPIO_IDR(GPIOC_BASE);
    return !((idr >> DRV_FAULT_L_PIN) & 1u) || !((idr >> DRV_FAULT_R_PIN) & 1u);
}

static int bumper_asserted(void)
{
    return !((GPIO_IDR(GPIOC_BASE) >> BUMPER_PIN) & 1u);
}

/* ------------------------------------------------------------------------ */
/* IWDG(RM0090 §21):LSI 32 kHz、/32 → 1 ms 一格;起動後硬體上停不掉                   */
/* ------------------------------------------------------------------------ */
static void iwdg_init(uint32_t ms)
{
    if (ms == 0) ms = 1; else if (ms > 4096) ms = 4096;
    IWDG_KR  = IWDG_KEY_UNLOCK;
    IWDG_PR  = IWDG_PR_DIV32;
    IWDG_RLR = ms - 1u;
    /* 真板:PR/RLR 寫入要等 SR 的 PVU/RVU 清掉才生效(LSI 域慢);Renode 永遠讀 0,一圈就過 */
    uint32_t spin = 0;
    while ((IWDG_SR & (IWDG_SR_PVU | IWDG_SR_RVU)) && ++spin < 100000) { }
    IWDG_KR  = IWDG_KEY_START;
    IWDG_KR  = IWDG_KEY_RELOAD;   /* 起動時計數器從 0xFFF 起,先餵一次才從 RLR 算 */
}

static void iwdg_feed(void) { IWDG_KR = IWDG_KEY_RELOAD; }

/* ------------------------------------------------------------------------ */
/* bxCAN                                                                    */
/* ------------------------------------------------------------------------ */
static int wait_msr(uint32_t mask, uint32_t want, uint32_t err_code)
{
    uint32_t t0 = now_ms();
    while ((CAN_MSR & mask) != want) {
        if (now_ms() - t0 > 10) { g_dbg.init_err = err_code; return -1; }
    }
    return 0;
}

static int can_init(void)
{
    RCC_APB1ENR |= RCC_APB1ENR_CAN1;

    /* RM0090 的順序:離開 sleep → 進 init → 設 BTR → 離開 init → 設濾波器 */
    CAN_MCR &= ~CAN_MCR_SLEEP;
    if (wait_msr(CAN_MSR_SLAK, 0, 1)) return -1;
    CAN_MCR |= CAN_MCR_INRQ;
    if (wait_msr(CAN_MSR_INAK, CAN_MSR_INAK, 2)) return -1;

    /* 500 kbit/s @ APB1 42 MHz:prescaler 6、BS1 11tq、BS2 2tq。Renode 不看這個值。 */
    CAN_BTR = (1u << 20) | (10u << 16) | 5u;

    CAN_MCR &= ~CAN_MCR_INRQ;
    if (wait_msr(CAN_MSR_INAK, 0, 3)) return -1;

    /* 濾波器 bank 0:32-bit、ID-mask 模式、遮罩全 0 = 全收、指到 FIFO0。
     * FMR 用讀-改-寫,保留 CAN2SB(重置值 14):清成 0 會把 bank 0 劃給 CAN2,
     * 訊框會被靜默丟掉——第 1 步探針踩到過。 */
    CAN_FMR  |= CAN_FMR_FINIT;
    CAN_FA1R &= ~1u;
    CAN_FM1R &= ~1u;
    CAN_FS1R |= 1u;
    CAN_FFA1R &= ~1u;
    CAN_F0R1 = 0;
    CAN_F0R2 = 0;
    CAN_FA1R |= 1u;
    CAN_FMR  &= ~CAN_FMR_FINIT;
    return 0;
}

static int can_send(uint32_t std_id, const uint8_t *d, uint8_t dlc)
{
    if (!(CAN_TSR & CAN_TSR_TME0)) return -1;
    CAN_TDT0R = dlc;
    CAN_TDL0R = (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
    CAN_TDH0R = (uint32_t)d[4] | ((uint32_t)d[5] << 8) | ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
    CAN_TI0R  = (std_id << 21) | CAN_TI0R_TXRQ;
    return 0;
}

/* 有訊框就取出一筆;回 1 表示 out_* 有效 */
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
/* 控制與里程計                                                              */
/* ------------------------------------------------------------------------ */
static int32_t s_sp_l, s_sp_r;            /* 設定點 mm/s */
static int32_t s_integ_l, s_integ_r;
static int32_t s_cmd_v, s_cmd_w;           /* 上位命令(mm/s、mrad/s) */
static int32_t s_v_ramp, s_w_ramp;         /* 斜坡後的 v、w */
static int32_t s_enc_l, s_enc_r;          /* 最新累計 tick */
#if !ENC_SOURCE_TIM
static int32_t s_enc_prev_l, s_enc_prev_r;
#endif
static uint32_t s_enc_new;                /* 上次量測之後收到的編碼器訊框數(TIM 來源恆為 1) */
#if ENC_SOURCE_TIM
static uint16_t s_cnt_prev_l, s_cnt_prev_r;
#endif
static int32_t s_meas_l, s_meas_r;        /* mm/s */
static int32_t s_duty_l, s_duty_r;
static uint32_t s_last_cmd_ms;
static int s_have_cmd;
static uint32_t s_last_ping_ms;             /* 心跳:最後一筆 PING 的時刻 */
static int s_have_ping;
static int32_t s_stall_ms;                  /* 堵轉:連續「duty 高且輪不動」累計毫秒 */
static int s_stalled;                       /* 堵轉鎖住,命令歸零才解 */
static int s_warm_reset;                    /* 這次開機是暖重置 */

/* 里程計用浮點(soft-float,由 libgcc 提供);沒有 libm,所以 sin/cos 自己寫。 */
static float s_x_mm, s_y_mm, s_th_rad;

static float fsin(float x)
{
    const float PI = 3.14159265f, TWO_PI = 6.28318531f;
    while (x >  PI) x -= TWO_PI;
    while (x < -PI) x += TWO_PI;
    /* 5 項 Taylor,|x|<=pi 時誤差 < 5e-3;夠里程計用 */
    float x2 = x * x;
    return x * (1.0f - x2 / 6.0f * (1.0f - x2 / 20.0f * (1.0f - x2 / 42.0f * (1.0f - x2 / 72.0f))));
}
static float fcos(float x) { return fsin(x + 1.57079633f); }

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* 一輪 PI:回帶號 duty */
static int32_t pi_step(int32_t sp, int32_t meas, int32_t *integ)
{
    int32_t e = sp - meas;
    *integ = clamp_i32(*integ + e, -PI_INTEGRAL_LIMIT, PI_INTEGRAL_LIMIT);
    /* 前饋 sp(滿 duty = WHEEL_SPEED_FULL_MM_S)× g_cfg.ff_q8 + PI 修正,Q8 定點;增益從 g_cfg 讀 */
    int32_t ff = (sp * DUTY_FULL_SCALE / WHEEL_SPEED_FULL_MM_S) * g_cfg.ff_q8 >> 8;
    int32_t u  = ff + ((g_cfg.kp_q8 * e + g_cfg.ki_q8 * (*integ)) >> 8);
    return clamp_i32(u, -DUTY_FULL_SCALE, DUTY_FULL_SCALE);
}

static void control_step(void)
{
    /* 輪速量測:tick 差 → mm/s。編碼器訊框每 CONTROL_PERIOD_MS 一筆是協定的節拍,
     * 所以時間基準用「收到幾筆訊框」而不是「過了幾個控制週期」:匯流排抖動讓某個週期
     * 收到 0 筆或 2 筆時,量測不會變成 0 或兩倍。0 筆就沿用上一次的量測(dl=dr=0,里程計不動)。
     * mm/s = dticks * 周長(um) / TPR / (週期(ms) * 訊框數)  (um/ms == mm/s) */
#if ENC_SOURCE_TIM
    /* 讀 CNT:16 位元差,繞回由 int16 轉型處理;累計進 s_enc_* 給里程計與 g_dbg */
    uint16_t cl = (uint16_t)TIM_CNT(TIM2_BASE), cr = (uint16_t)TIM_CNT(TIM4_BASE);
    int32_t dl = (int16_t)(cl - s_cnt_prev_l), dr = (int16_t)(cr - s_cnt_prev_r);
    s_cnt_prev_l = cl; s_cnt_prev_r = cr;
    s_enc_l += dl; s_enc_r += dr;
    g_dbg.enc_l = s_enc_l; g_dbg.enc_r = s_enc_r;
    s_enc_new = 1;   /* 時間基準就是控制週期:CNT 在韌體自己的 tick 取樣 */
#else
    int32_t dl = s_enc_l - s_enc_prev_l, dr = s_enc_r - s_enc_prev_r;
    s_enc_prev_l = s_enc_l; s_enc_prev_r = s_enc_r;
#endif
    if (s_enc_new) {
        s_meas_l = (int32_t)((int64_t)dl * WHEEL_CIRC_UM / ENC_TICKS_PER_REV / (CONTROL_PERIOD_MS * (int32_t)s_enc_new));
        s_meas_r = (int32_t)((int64_t)dr * WHEEL_CIRC_UM / ENC_TICKS_PER_REV / (CONTROL_PERIOD_MS * (int32_t)s_enc_new));
        s_enc_new = 0;
    }

    /* 里程計:差速模型,中點法 */
    float dl_mm = (float)dl * ((float)WHEEL_CIRC_UM / 1000.0f) / (float)ENC_TICKS_PER_REV;
    float dr_mm = (float)dr * ((float)WHEEL_CIRC_UM / 1000.0f) / (float)ENC_TICKS_PER_REV;
    float ds = (dl_mm + dr_mm) * 0.5f;
    float dth = (dr_mm - dl_mm) / (float)TRACK_MM;
    float th_mid = s_th_rad + dth * 0.5f;
    s_x_mm += ds * fcos(th_mid);
    s_y_mm += ds * fsin(th_mid);
    s_th_rad += dth;

    /* 安全閘門。兩種處置:「切」= 致能關、duty 0、積分清(急停、命令逾時、驅動器故障、堵轉);
     * 「降」= 命令改 0 走斜坡下來,致能不關(心跳丟失、保險桿只擋前進)。 */
    uint32_t flags = s_warm_reset ? ODOM_FLAG_WDT_RESET : 0;
    uint32_t mask = g_cfg.safety_mask;
    int enable = 1;
    int32_t cmd_v = s_cmd_v, cmd_w = s_cmd_w;
    uint32_t now = now_ms();
    if (estop_asserted()) { flags |= ODOM_FLAG_ESTOP; enable = 0; }
    if (!s_have_cmd || now - s_last_cmd_ms > CMD_TIMEOUT_MS) { flags |= ODOM_FLAG_CMD_STALE; enable = 0; }
    if ((mask & SAFETY_DRV_FAULT) && drv_fault_asserted()) { flags |= ODOM_FLAG_DRV_FAULT; enable = 0; }
    if ((mask & SAFETY_BUMPER) && bumper_asserted()) { flags |= ODOM_FLAG_BUMPER; if (cmd_v > 0) cmd_v = 0; }
    if ((mask & SAFETY_HB) && g_cfg.hb_timeout_ms > 0
        && (!s_have_ping || now - s_last_ping_ms > (uint32_t)g_cfg.hb_timeout_ms)) {
        flags |= ODOM_FLAG_HB_LOST; cmd_v = 0; cmd_w = 0;
    }
    /* 堵轉:上一步的 duty 已經很高、輪子卻不動,持續 stall_ms → 鎖住;上位把命令歸零才解 */
    if (mask & SAFETY_STALL) {
        int32_t al = s_duty_l < 0 ? -s_duty_l : s_duty_l, ar = s_duty_r < 0 ? -s_duty_r : s_duty_r;
        int32_t ml = s_meas_l < 0 ? -s_meas_l : s_meas_l, mr = s_meas_r < 0 ? -s_meas_r : s_meas_r;
        int stuck = (al >= g_cfg.stall_duty && ml < STALL_SPEED_MM_S) || (ar >= g_cfg.stall_duty && mr < STALL_SPEED_MM_S);
        s_stall_ms = stuck ? s_stall_ms + CONTROL_PERIOD_MS : 0;
        if (s_stall_ms >= g_cfg.stall_ms) s_stalled = 1;
        if (s_cmd_v == 0 && s_cmd_w == 0) { s_stalled = 0; s_stall_ms = 0; }
        if (s_stalled) { flags |= ODOM_FLAG_STALL; enable = 0; }
    }

    if (enable) {
        flags |= ODOM_FLAG_ENABLED;
        /* 斜坡對 v(mm/s²)與 w(mrad/s²)各自限斜率,再換成兩輪設定點——對兩輪各自限會讓
         * 轉→直的過渡兩輪不同步(量到方形每段偏航 +0.16 rad);0 = 直接跳(第一版行為) */
        int32_t dv = g_cfg.accel_mm_s2 * CONTROL_PERIOD_MS / 1000;
        int32_t dw = g_cfg.alpha_mrad_s2 * CONTROL_PERIOD_MS / 1000;
        s_v_ramp = dv > 0 ? s_v_ramp + clamp_i32(cmd_v - s_v_ramp, -dv, dv) : cmd_v;
        s_w_ramp = dw > 0 ? s_w_ramp + clamp_i32(cmd_w - s_w_ramp, -dw, dw) : cmd_w;
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
    int16_t w = (int16_t)(p[2] | (p[3] << 8));   /* mrad/s */
    /* v_l = v - w*track/2,w 是 mrad/s → (w * TRACK_MM / 2) / 1000 mm/s */
    s_cmd_v = v; s_cmd_w = w;   /* 斜坡與換算在控制步做(對 v、w 各自限斜率,兩輪才不會在過渡時不同步) */
    s_last_cmd_ms = now_ms();
    s_have_cmd = 1;
    g_dbg.cmd_frames++;
}

static void report_odom(uint16_t seq, uint32_t flags)
{
    odom_payload_t o;
    o.seq = seq;
    o.t_ms = now_ms();
    o.x_mm = (int32_t)s_x_mm;
    o.y_mm = (int32_t)s_y_mm;
    o.th_mrad = (int32_t)(s_th_rad * 1000.0f);
    o.vl_mm_s = (int16_t)s_meas_l;
    o.vr_mm_s = (int16_t)s_meas_r;
    o.flags = (uint8_t)flags;
    proto_send(MSG_ODOM, (const uint8_t *)&o, sizeof o);
}

static void report_motor_status_can(uint8_t seq, uint32_t flags)
{
    uint8_t d[8];
    d[0] = (uint8_t)(s_duty_l & 0xFF); d[1] = (uint8_t)((s_duty_l >> 8) & 0xFF);
    d[2] = (uint8_t)(s_duty_r & 0xFF); d[3] = (uint8_t)((s_duty_r >> 8) & 0xFF);
    d[4] = (uint8_t)flags; d[5] = seq; d[6] = 0; d[7] = 0;
    can_send(CAN_ID_MOTOR_STATUS, d, 8);
}

/* ------------------------------------------------------------------------ */
int main(void)
{
    g_dbg.magic = 0x48494C31u;

    /* SysTick 1 kHz */
    SYST_RVR = SYSTICK_HZ / 1000u - 1u;
    SYST_CVR = 0;
    SYST_CSR = SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT | SYST_CSR_ENABLE;

    RCC_APB2ENR |= RCC_APB2ENR_USART1;
    RCC_APB1ENR |= RCC_APB1ENR_USART2;
    uart_init(USART1_BASE);
    uart_init(USART2_BASE);
    USART_CR1(USART1_BASE) |= USART_CR1_RXNEIE;
    NVIC_ISER(USART1_IRQN / 32) = 1u << (USART1_IRQN % 32);
    dbg_puts("hilctl boot fw ");
    dbg_put_u32(FW_VERSION_MAJOR); dbg_puts("."); dbg_put_u32(FW_VERSION_MINOR); dbg_puts("\r\n");

    gpio_init();
#if ENC_SOURCE_TIM
    encoder_init();
#endif
    pwm_init();
    if (can_init() == 0) dbg_puts("can1 ready\r\n");
    else { dbg_puts("can1 init FAILED step "); dbg_put_u32(g_dbg.init_err); dbg_puts("\r\n"); }

    motor_apply(0, 0, 0);

    /* 暖重置偵測:真板看 RCC_CSR.IWDGRSTF(讀完用 RMVF 清);Renode 的 RCC 不設它,
     * 所以另外靠 .noinit 的計數——magic 還在就是重置過,不是上電。兩個都記進 g_dbg。 */
    g_dbg.boot_csr = RCC_CSR;
    RCC_CSR |= RCC_CSR_RMVF;
    if (g_noinit.magic == 0x4E4F494Eu) { g_noinit.resets++; s_warm_reset = 1; }
    else { g_noinit.magic = 0x4E4F494Eu; g_noinit.resets = 0; }
    if (g_dbg.boot_csr & RCC_CSR_IWDGRSTF) s_warm_reset = 1;
    g_dbg.resets = g_noinit.resets;
    if (g_cfg.safety_mask & SAFETY_IWDG) iwdg_init((uint32_t)g_cfg.iwdg_ms);
    dbg_puts("main loop\r\n");

    uint32_t next_ctrl = now_ms() + CONTROL_PERIOD_MS;
    uint32_t next_report = now_ms() + REPORT_PERIOD_MS;
    uint16_t seq = 0;

    for (;;) {
        /* 1. 上位協定 */
        int c;
        while ((c = uart1_getc()) >= 0) {
            if (proto_feed(&s_rx, (uint8_t)c)) {
                if (s_rx.type == MSG_CMD_VEL && s_rx.len == 4) handle_cmd_vel(s_rx.payload);
                else if (s_rx.type == MSG_PING) {
                    uint8_t v[2] = { FW_VERSION_MAJOR, FW_VERSION_MINOR };
                    s_last_ping_ms = now_ms(); s_have_ping = 1; g_dbg.ping_frames++;
                    proto_send(MSG_PONG, v, 2);
                }
            }
        }

        /* 2. CAN 編碼器 */
        uint32_t id; uint8_t d[8]; uint8_t dlc;
        while (can_recv(&id, d, &dlc)) {
            if (id == CAN_ID_ENCODER && dlc == 8) {
                g_dbg.enc_frames++;
#if !ENC_SOURCE_TIM
                s_enc_l = (int32_t)((uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24));
                s_enc_r = (int32_t)((uint32_t)d[4] | ((uint32_t)d[5] << 8) | ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24));
                g_dbg.enc_l = s_enc_l; g_dbg.enc_r = s_enc_r;
                s_enc_new++;
#endif
            }
        }

        /* 3. 5 ms 控制;4. 20 ms 回報 */
        uint32_t t = now_ms();
        g_dbg.tick_ms = t;
        if ((int32_t)(t - next_ctrl) >= 0) {
            next_ctrl += CONTROL_PERIOD_MS;
            control_step();
            /* 只在控制步真的跑了才餵:主迴圈活著但控制步沒排到,同樣該重置 */
            if (g_cfg.safety_mask & SAFETY_IWDG) iwdg_feed();
        }
        /* 故障注入(只給 IWDG 驗收):模擬韌體死在關中斷的迴圈裡。PWM 週邊還在跑——
         * 馬達會用最後的 duty 一直轉,直到 IWDG 把整顆重置 */
        if (g_cfg.hang_at_ms > 0 && (int32_t)(t - (uint32_t)g_cfg.hang_at_ms) >= 0) {
            __asm volatile("cpsid i");
            for (;;) { }
        }
        if ((int32_t)(t - next_report) >= 0) {
            next_report += REPORT_PERIOD_MS;
            seq++;
            report_odom(seq, g_dbg.flags);
            report_motor_status_can((uint8_t)seq, g_dbg.flags);
        }
        /* 睡到下一個中斷(SysTick 1 ms 或 USART1 RX)。 */
        __asm volatile("wfi");
    }
}
