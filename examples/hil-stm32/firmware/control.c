/*
 * control.c:兩版韌體(裸機 firmware/main.c、FreeRTOS firmware-freertos/main-rtos.c)共用的部分——
 * 協定、週邊暫存器序列、控制律、安全閘門、里程計、回報、g_dbg/g_cfg 版面。
 * 兩版只剩「誰排程、誰餵狗、ISR 怎麼叫醒主迴圈」在各自的 main 裡。
 *
 * 這一層不知道時間怎麼來:每個要看時間的函式都收 now_ms(裸機給 SysTick 計數、RTOS 給 xTaskGetTickCount,
 * 兩者都是 1 kHz);也不知道 g_dbg 多大:RTOS 版在 dbg_common_t 後面接自己的欄位,開機時 ctl_bind_dbg() 告訴這裡前 23 字在哪。
 */
#include <stdint.h>
#include "regs.h"
#include "proto.h"
#include "calib.h"
#include "crc16.h"
#include "control.h"

static dbg_common_t *D;   /* g_dbg 的共同前 30 字 */

noinit_t g_noinit __attribute__((section(".noinit"), aligned(4)));

cfg_t g_cfg __attribute__((aligned(4))) = { 0x48494C43u, PI_KP_Q8, PI_KI_Q8, ACCEL_LIMIT_MM_S2, FF_GAIN_Q8, ALPHA_LIMIT_MRAD_S2,
                                            IWDG_TIMEOUT_MS, 0, HB_TIMEOUT_MS, STALL_DUTY, STALL_MS, SAFETY_MASK,
                                            SLIP_RESID_MRAD_S, SLIP_MS, GYRO_BIAS_STILL_MS,
                                            SLIP_VEL_MM_S, YAW_FUSION };

void ctl_bind_dbg(dbg_common_t *dbg) { D = dbg; D->magic = 0x48494C31u; }

/* ------------------------------------------------------------------------ */
/* USART(USART1 上位協定、USART2 printer);收訊 ISR 與 ring buffer 在各自的 main     */
/* ------------------------------------------------------------------------ */
void ctl_uart_init(uint32_t base)
{
    /* 鮑率不影響 Renode;真硬體才要按 APB 時脈算。這裡放 115200 @ 42 MHz 的值。 */
    USART_BRR(base) = 0x16D;
    USART_CR1(base) = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

/* 輪詢 TXE,不等 TC:既有紀錄指出 Renode 1.16.1 的 STM32_UART 把 TC 做成永不重設,
 * 等 TC 的傳送路徑會卡死。輪詢 TXE 在真硬體同樣正確,只是佔 CPU。 */
void ctl_uart_putc(uint32_t base, uint8_t c)
{
    uint32_t spin = 0;
    while (!(USART_SR(base) & USART_SR_TXE) && ++spin < 100000) { }
    USART_DR(base) = c;
}

void dbg_puts(const char *s)
{
    while (*s) ctl_uart_putc(USART2_BASE, (uint8_t)*s++);
}

void dbg_put_u32(uint32_t v)
{
    char buf[11]; int i = 10; buf[i] = 0;
    do { buf[--i] = (char)('0' + v % 10); v /= 10; } while (v);
    dbg_puts(&buf[i]);
}

/* ------------------------------------------------------------------------ */
/* 上位協定:框包收發                                                        */
/* ------------------------------------------------------------------------ */
void ctl_proto_send(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[2 + PROTO_MAX_PAYLOAD];
    buf[0] = type;
    for (uint8_t i = 0; i < len; i++) buf[1 + i] = payload[i];
    uint16_t crc = crc16_modbus(buf, (uint32_t)len + 1);

    ctl_uart_putc(USART1_BASE, PROTO_SYNC0);
    ctl_uart_putc(USART1_BASE, PROTO_SYNC1);
    ctl_uart_putc(USART1_BASE, len);
    for (uint32_t i = 0; i < (uint32_t)len + 1; i++) ctl_uart_putc(USART1_BASE, buf[i]);
    ctl_uart_putc(USART1_BASE, (uint8_t)(crc & 0xFF));
    ctl_uart_putc(USART1_BASE, (uint8_t)(crc >> 8));
}

/* 接收狀態機:一次餵一個 byte,收齊且 CRC 對才回 1(rx_t 在 control.h) */
int ctl_proto_feed(rx_t *r, uint8_t b)
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
        D->bad_crc++;
        return 0;
    }
    }
    return 0;
}

/* ------------------------------------------------------------------------ */
/* GPIO / 編碼器 / PWM / 安全輸入 / IWDG                                       */
/* ------------------------------------------------------------------------ */
void ctl_gpio_init(void)
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

void ctl_encoder_init(void)
{
    RCC_APB1ENR |= RCC_APB1ENR_TIM2 | RCC_APB1ENR_TIM4;
    encoder_tim_init(TIM2_BASE);
    encoder_tim_init(TIM4_BASE);
}
#endif

/* ------------------------------------------------------------------------ */
/* IMU:LSM330 陀螺儀(I2C3,7-bit 位址 0x6A = SDO_G 接地)。暫存器與靈敏度出自 LSM330 datasheet       */
/* (DocID023426 Rev 3):WHO_AM_I_G 0x0F = 0xD4、CTRL_REG1_G 0x20、OUT_Z_L/H_G 0x2C/0x2D、±250 dps 8.75 mdps/digit */
/* ------------------------------------------------------------------------ */
#define IMU_ADDR          0x6A
#define LSM330_WHO_AM_I_G 0x0F
#define LSM330_WHO_AM_I   0xD4
#define LSM330_CTRL_REG1  0x20
#define LSM330_OUT_Z_L    0x2C
#define LSM330_OUT_Z_H    0x2D
/* 加速度計(同一顆 LSM330 的另一半,DocID023426 Rev 3 Table 17)。7-bit 位址:§6.1.1 正文與 Table 15 對 SDO_A 的描述
 * 不一致,這裡取 0x1E,與平台描述一致(docs/hil/38 §1.4) */
#define ACC_ADDR            0x1E
#define LSM330_WHO_AM_I_A   0x0F
#define LSM330_WHO_AM_I_A_V 0x40
#define LSM330_CTRL_REG5_A  0x20
#define LSM330_CTRL_REG6_A  0x24
#define LSM330_OUT_X_L_A    0x28
#define LSM330_OUT_X_H_A    0x29
#define I2C_SPIN          20000u     /* 每個等待的輪詢上限;沒回應就放棄,不卡控制步 */
#define SLIP_WINDOW       10         /* 殘差滑動平均 10 個控制步 = 50 ms */
#define GYRO_BIAS_N       256        /* 零偏移動平均的長度:256 步 = 1.28 s */
#define GYRO_BIAS_MIN_N   20         /* 零偏累計不到 20 筆(100 ms)之前不做打滑判斷 */
#define ACC_RESID_LAMBDA  0.99005f   /* exp(−5 ms / 0.5 s):速度殘差的漏積分,τ 0.5 s(docs/hil/38 §1.4) */

static int s_imu_ok;
static int s_acc_ok;

/* 等 SR1 的某個位元;AF(沒有 ACK)或逾時就回負值 */
static int i2c_wait(uint32_t mask)
{
    for (uint32_t spin = 0; spin < I2C_SPIN; spin++) {
        uint32_t sr1 = I2C_SR1(I2C3_BASE);
        if (sr1 & I2C_SR1_AF) { I2C_SR1(I2C3_BASE) = sr1 & ~I2C_SR1_AF; return -1; }
        if (sr1 & mask) return 0;
    }
    return -2;
}

static int i2c_fail(int e)
{
    I2C_CR1(I2C3_BASE) |= I2C_CR1_STOP;
    return e;
}

/* 起始 + 位址(RM0090 §27.3.3 EV5/EV6);ADDR 由讀 SR1 再讀 SR2 清(接收單 byte 時呼叫端要先清 ACK) */
static int i2c_start_addr(uint8_t addr, int read)
{
    I2C_CR1(I2C3_BASE) |= I2C_CR1_START;
    if (i2c_wait(I2C_SR1_SB)) return -3;
    I2C_DR(I2C3_BASE) = (uint32_t)(addr << 1) | (read ? 1u : 0u);
    int e = i2c_wait(I2C_SR1_ADDR);
    return e ? e - 3 : 0;
}

static int i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    int e = i2c_start_addr(addr, 0);
    if (e) return i2c_fail(e);
    (void)I2C_SR1(I2C3_BASE); (void)I2C_SR2(I2C3_BASE);
    I2C_DR(I2C3_BASE) = reg;
    if (i2c_wait(I2C_SR1_TXE)) return i2c_fail(-10);
    I2C_DR(I2C3_BASE) = val;
    if (i2c_wait(I2C_SR1_BTF)) return i2c_fail(-11);
    I2C_CR1(I2C3_BASE) |= I2C_CR1_STOP;
    return 0;
}

/* 讀一個暫存器:子位址 MSb = 0(不自動遞增),單 byte 接收照 RM0090 §27.3.3「Closing the communication」第 3 點:
 * ADDR 清掉之前關 ACK,清掉之後才下 STOP */
static int i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *val)
{
    int e = i2c_start_addr(addr, 0);
    if (e) return i2c_fail(e);
    (void)I2C_SR1(I2C3_BASE); (void)I2C_SR2(I2C3_BASE);
    I2C_DR(I2C3_BASE) = reg;
    if (i2c_wait(I2C_SR1_BTF)) return i2c_fail(-12);
    e = i2c_start_addr(addr, 1);
    if (e) return i2c_fail(e - 20);
    I2C_CR1(I2C3_BASE) &= ~I2C_CR1_ACK;
    (void)I2C_SR1(I2C3_BASE); (void)I2C_SR2(I2C3_BASE);
    I2C_CR1(I2C3_BASE) |= I2C_CR1_STOP;
    if (i2c_wait(I2C_SR1_RXNE)) return i2c_fail(-13);
    *val = (uint8_t)I2C_DR(I2C3_BASE);
    return 0;
}

void ctl_imu_init(void)
{
    RCC_APB1ENR |= RCC_APB1ENR_I2C3;
    /* PA8 → AF4 開汲極;PC9 → AF4 開汲極(外部 pull-up 在板子上) */
    GPIO_MODER(GPIOA_BASE)  = (GPIO_MODER(GPIOA_BASE) & ~(3u << 16)) | (2u << 16);
    GPIO_OTYPER(GPIOA_BASE) |= 1u << 8;
    GPIO_AFRH(GPIOA_BASE)   = (GPIO_AFRH(GPIOA_BASE) & ~0xFu) | 4u;
    GPIO_MODER(GPIOC_BASE)  = (GPIO_MODER(GPIOC_BASE) & ~(3u << 18)) | (2u << 18);
    GPIO_OTYPER(GPIOC_BASE) |= 1u << 9;
    GPIO_AFRH(GPIOC_BASE)   = (GPIO_AFRH(GPIOC_BASE) & ~(0xFu << 4)) | (4u << 4);
    /* 100 kHz 標準模式 @ APB1 42 MHz(RM0090 §27.6.2/§27.6.8/§27.6.9):FREQ 42、CCR = 42 MHz / (2 × 100 kHz) = 210、TRISE = 42 + 1 */
    I2C_CR1(I2C3_BASE) = 0;
    I2C_CR2(I2C3_BASE) = 42;
    I2C_CCR(I2C3_BASE) = 210;
    I2C_TRISE(I2C3_BASE) = 43;
    I2C_CR1(I2C3_BASE) = I2C_CR1_PE;

    uint8_t who = 0;
    int e = i2c_read_reg(IMU_ADDR, LSM330_WHO_AM_I_G, &who);
    if (e) { D->imu_whoami = 0x100u | (uint32_t)(-e); s_imu_ok = 0; return; }
    D->imu_whoami = who;
    /* CTRL_REG1_G:PD = 1(normal mode)、Zen Yen Xen = 1 */
    s_imu_ok = who == LSM330_WHO_AM_I && i2c_write_reg(IMU_ADDR, LSM330_CTRL_REG1, 0x0F) == 0;

    /* 加速度計:WHO_AM_I_A = 0x40;CTRL_REG5_A = 0111 0111(400 Hz、BDU 0、XYZ);CTRL_REG6_A = 0010 0000(±16 g、BW 800 Hz)。
     * ±16 g 不是 ±2 g:受控體撞上障礙物時一步(5 ms)內把 300 mm/s 煞到 0,約 6 g;±2 g 飽和只積得出三分之一的速度變化,
     * 平移打滑漏判、凍結碰撞反而誤報(docs/hil/38 §1.4) */
    uint8_t wa = 0;
    e = i2c_read_reg(ACC_ADDR, LSM330_WHO_AM_I_A, &wa);
    if (e) { D->acc_whoami = 0x100u | (uint32_t)(-e); s_acc_ok = 0; return; }
    D->acc_whoami = wa;
    s_acc_ok = wa == LSM330_WHO_AM_I_A_V && i2c_write_reg(ACC_ADDR, LSM330_CTRL_REG5_A, 0x77) == 0
            && i2c_write_reg(ACC_ADDR, LSM330_CTRL_REG6_A, 0x20) == 0;
}

void ctl_pwm_init(void)
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

void ctl_motor_apply(int32_t duty_l, int32_t duty_r, int enable)
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

void ctl_iwdg_feed(void) { if (g_cfg.safety_mask & SAFETY_IWDG) IWDG_KR = IWDG_KEY_RELOAD; }

/* ------------------------------------------------------------------------ */
/* bxCAN                                                                    */
/* ------------------------------------------------------------------------ */
/* 交握逾時:裸機給 now_ms(SysTick 已經在走,10 ms);RTOS 版 tick 要 scheduler 起來才走,給 NULL 用迭代數(100000 圈) */
static uint32_t (*s_now_ms)(void);
static int wait_msr(uint32_t mask, uint32_t want, uint32_t err_code)
{
    if (s_now_ms) {
        uint32_t t0 = s_now_ms();
        while ((CAN_MSR & mask) != want) {
            if (s_now_ms() - t0 > 10) { D->init_err = err_code; return -1; }
        }
    } else {
        uint32_t n = 0;
        while ((CAN_MSR & mask) != want && ++n < 100000) { }
        if ((CAN_MSR & mask) != want) { D->init_err = err_code; return -1; }
    }
    return 0;
}

int ctl_can_init(uint32_t (*now_ms)(void))
{
    s_now_ms = now_ms;
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
static volatile int32_t s_sp_l, s_sp_r;   /* 設定點 mm/s(RTOS 版由另一個 task 讀) */
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
static int32_t s_yaw_hist[SLIP_WINDOW];     /* 打滑:每步的(陀螺儀 − 輪差)yaw rate,mrad/s */
static int s_yaw_idx;
static int32_t s_slip_ms;                   /* 殘差超過門檻的累計毫秒 */
static int s_slipped;                       /* 打滑鎖住,命令歸零才解 */
static int32_t s_still_ms;                  /* 陀螺儀零偏:連續靜止(命令 0、兩輪 CNT 不動)的毫秒 */
static int32_t s_bias_sum, s_bias_n;        /* 零偏移動平均:前 GYRO_BIAS_N 筆算術平均,之後 sum += gz − sum/N */
static int32_t s_abias_sum, s_abias_n;      /* 加速度計前進軸的零偏,同上 */
static float s_vel_resid;                   /* 平移打滑:漏積分的速度殘差 mm/s */
static int32_t s_v_wheel_prev;              /* 上一步的輪速(兩輪平均)mm/s */
static int32_t s_slip_acc_ms;               /* 速度殘差超過門檻的累計毫秒 */

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

void ctl_control_step(uint32_t now)
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
    D->enc_l = s_enc_l; D->enc_r = s_enc_r;
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

    /* 打滑(轉角):陀螺儀量到的 yaw rate 對輪差推出的 yaw rate。輪子頂著障礙物打滑時編碼器照數、車體不照轉,
     * 兩者分開;車體沒轉的打滑(正面頂住)這裡看不到,交給下面的加速度計(docs/hil/38 §1.3、§1.4)。殘差取
     * SLIP_WINDOW 步的平均,因為輪速的量子是 1 tick / 5 ms = 15 mm/s → 輪差 yaw rate 一步 50 mrad/s。
     * 零偏:datasheet 的典型零點 ±10 dps = 174.5 mrad/s,是打滑門檻的好幾倍(docs/hil/36 §3.2)。車靜止(命令 0、
     * 兩輪 CNT 都不動)滿 gyro_bias_still_ms 才累計;前 GYRO_BIAS_N 筆算術平均,之後移動平均。零偏累計不到
     * GYRO_BIAS_MIN_N 之前不做打滑判斷。gyro_bias_still_ms = 0 不估,零偏固定 0。加速度計用同一個靜止條件。 */
    int32_t yaw_resid = 0;
    int bias_ok = 1, acc_bias_ok = 1, have_gyro = 0;
    int32_t gz = 0, gbias = 0;
    /* 靜止 = 斜坡後的命令為 0(上一步的值)且兩輪 CNT 都不動。看斜坡後而不是收到的命令:校正期間命令被閘成 0,
     * 上位一直送非零命令也要能靜止下來估完 */
    int still = (s_v_ramp == 0 && s_w_ramp == 0 && dl == 0 && dr == 0);
    s_still_ms = still ? s_still_ms + CONTROL_PERIOD_MS : 0;
    int bias_window = g_cfg.gyro_bias_still_ms > 0 && s_still_ms >= g_cfg.gyro_bias_still_ms;
    if (g_cfg.gyro_bias_still_ms <= 0) { s_bias_sum = 0; s_bias_n = 0; s_abias_sum = 0; s_abias_n = 0; }
    if (s_imu_ok) {
        uint8_t lo = 0, hi = 0;
        if (i2c_read_reg(IMU_ADDR, LSM330_OUT_Z_L, &lo) == 0 && i2c_read_reg(IMU_ADDR, LSM330_OUT_Z_H, &hi) == 0) {
            int16_t raw = (int16_t)(lo | (hi << 8));
            gz = (int32_t)raw * 1527 / 10000;                       /* 8.75 mdps/digit = 0.1527 mrad/s */
            have_gyro = 1;
            if (bias_window) {
                if (s_bias_n < GYRO_BIAS_N) { s_bias_sum += gz; s_bias_n++; }
                else s_bias_sum += gz - s_bias_sum / GYRO_BIAS_N;
            }
            gbias = (g_cfg.gyro_bias_still_ms > 0 && s_bias_n > 0) ? s_bias_sum / s_bias_n : 0;
            int32_t ww = (s_meas_r - s_meas_l) * 1000 / TRACK_MM;   /* mrad/s */
            s_yaw_hist[s_yaw_idx] = (gz - gbias) - ww;
            s_yaw_idx = (s_yaw_idx + 1) % SLIP_WINDOW;
            D->gyro_z = gz;
        }
        if (g_cfg.gyro_bias_still_ms > 0) bias_ok = s_bias_n >= GYRO_BIAS_MIN_N;
        D->gyro_bias = bias_ok ? gbias : 0x7FFFFFFF;
        int32_t sum = 0;
        for (int i = 0; i < SLIP_WINDOW; i++) sum += s_yaw_hist[i];
        yaw_resid = (sum < 0 ? -sum : sum) / SLIP_WINDOW;
        D->yaw_resid = yaw_resid;
    }

    /* 里程計:差速模型,中點法。航向增量預設用輪差;yaw_fusion 開、零偏已估出、而且上面的殘差平均超過打滑門檻時,
     * 這一步改用「陀螺儀 − 零偏」(gyrodometry:兩者對不上的片段才換陀螺儀,正常行駛完全不吃陀螺儀的零偏漂移;
     * docs/hil/38 §1.5)。距離仍用輪子。 */
    float dl_mm = (float)dl * ((float)WHEEL_CIRC_UM / 1000.0f) / (float)ENC_TICKS_PER_REV;
    float dr_mm = (float)dr * ((float)WHEEL_CIRC_UM / 1000.0f) / (float)ENC_TICKS_PER_REV;
    float ds = (dl_mm + dr_mm) * 0.5f;
    float dth = (dr_mm - dl_mm) / (float)TRACK_MM;
    if (g_cfg.yaw_fusion && s_imu_ok && have_gyro && bias_ok && yaw_resid > g_cfg.slip_mrad_s) {
        dth = (float)(gz - gbias) * 0.001f * (float)CONTROL_PERIOD_MS * 0.001f;
        D->gyro_steps++;
    }
    float th_mid = s_th_rad + dth * 0.5f;
    s_x_mm += ds * fcos(th_mid);
    s_y_mm += ds * fsin(th_mid);
    s_th_rad += dth;

    /* 打滑(平移):加速度計前進軸對輪速。e = 漏積分(a_x − 零偏 − 輪速微分),τ = ACC_RESID_TAU_S;車體與輪子一起動時
     * e ≈ 0,輪子轉而車體不動時 e 跟著輪速長出來(撞上的那一步就跳到車速)。靜止時清零。離線分離度與門檻的出處:
     * docs/hil/38 §1.4。0.732 mg/digit(±16 g)= 7.178 mm/s²/digit。 */
    int32_t vel_resid = 0;
    if (s_acc_ok) {
        uint8_t lo = 0, hi = 0;
        int32_t v_wheel = (s_meas_l + s_meas_r) / 2;
        if (i2c_read_reg(ACC_ADDR, LSM330_OUT_X_L_A, &lo) == 0 && i2c_read_reg(ACC_ADDR, LSM330_OUT_X_H_A, &hi) == 0) {
            int16_t raw = (int16_t)(lo | (hi << 8));
            int32_t ax = (int32_t)raw * 7178 / 1000;                 /* mm/s² */
            if (bias_window) {
                if (s_abias_n < GYRO_BIAS_N) { s_abias_sum += ax; s_abias_n++; }
                else s_abias_sum += ax - s_abias_sum / GYRO_BIAS_N;
            }
            int32_t abias = (g_cfg.gyro_bias_still_ms > 0 && s_abias_n > 0) ? s_abias_sum / s_abias_n : 0;
            if (g_cfg.gyro_bias_still_ms > 0) acc_bias_ok = s_abias_n >= GYRO_BIAS_MIN_N;
            if (still || !acc_bias_ok) s_vel_resid = 0.0f;
            else s_vel_resid = s_vel_resid * ACC_RESID_LAMBDA
                             + (float)(ax - abias) * (float)CONTROL_PERIOD_MS * 0.001f - (float)(v_wheel - s_v_wheel_prev);
            D->acc_x = ax;
            D->acc_bias = acc_bias_ok ? abias : 0x7FFFFFFF;
        }
        s_v_wheel_prev = v_wheel;
        vel_resid = (int32_t)(s_vel_resid < 0 ? -s_vel_resid : s_vel_resid);
        D->vel_resid = vel_resid;
    }

    /* 安全閘門。兩種處置:「切」= 致能關、duty 0、積分清(急停、命令逾時、驅動器故障、堵轉);
     * 「降」= 命令改 0 走斜坡下來,致能不關(心跳丟失、保險桿只擋前進)。 */
    uint32_t flags = s_warm_reset ? ODOM_FLAG_WDT_RESET : 0;
    uint32_t mask = g_cfg.safety_mask;
    int enable = 1;
    int32_t cmd_v = s_cmd_v, cmd_w = s_cmd_w;
    if (estop_asserted()) { flags |= ODOM_FLAG_ESTOP; enable = 0; }
    if (!s_have_cmd || now - s_last_cmd_ms > CMD_TIMEOUT_MS) { flags |= ODOM_FLAG_CMD_STALE; enable = 0; }
    if ((mask & SAFETY_DRV_FAULT) && drv_fault_asserted()) { flags |= ODOM_FLAG_DRV_FAULT; enable = 0; }
    if ((mask & SAFETY_BUMPER) && bumper_asserted()) { flags |= ODOM_FLAG_BUMPER; if (cmd_v > 0) cmd_v = 0; }
    /* IMU 零偏還沒估出來:打滑偵測與航向融合都不能用,命令當 0 讓車靜止下來估(真板開機校正陀螺儀的做法;docs/hil/36 §3.2) */
    if ((s_imu_ok && !bias_ok) || (s_acc_ok && !acc_bias_ok)) { flags |= ODOM_FLAG_IMU_CAL; cmd_v = 0; cmd_w = 0; }
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
    /* 打滑:只回報,不切——車頂著東西時要停還是要退是上位的決定(driver 鎖住,§6.3 的 C12 同一套) */
    if ((mask & SAFETY_SLIP) && s_imu_ok && bias_ok) {
        s_slip_ms = yaw_resid > g_cfg.slip_mrad_s ? s_slip_ms + CONTROL_PERIOD_MS : 0;
        if (s_slip_ms >= g_cfg.slip_ms && !s_slipped) { s_slipped = 1; D->slip_src |= 1; }
    }
    if ((mask & SAFETY_SLIP_ACC) && s_acc_ok && acc_bias_ok) {
        s_slip_acc_ms = vel_resid > g_cfg.slip_vel_mm_s ? s_slip_acc_ms + CONTROL_PERIOD_MS : 0;
        if (s_slip_acc_ms >= g_cfg.slip_ms && !s_slipped) { s_slipped = 1; D->slip_src |= 2; }
    }
    if (s_cmd_v == 0 && s_cmd_w == 0) { s_slipped = 0; s_slip_ms = 0; s_slip_acc_ms = 0; }
    if (s_slipped) flags |= ODOM_FLAG_SLIP;

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
    ctl_motor_apply(s_duty_l, s_duty_r, enable);

    D->ctrl_steps++;
    D->sp_l = s_sp_l;   D->sp_r = s_sp_r;
    D->meas_l = s_meas_l; D->meas_r = s_meas_r;
    D->duty_l = s_duty_l; D->duty_r = s_duty_r;
    D->flags = flags;
}

static void handle_cmd_vel(const uint8_t *p, uint32_t now)
{
    int16_t v = (int16_t)(p[0] | (p[1] << 8));
    int16_t w = (int16_t)(p[2] | (p[3] << 8));   /* mrad/s */
    /* v_l = v - w*track/2,w 是 mrad/s → (w * TRACK_MM / 2) / 1000 mm/s */
    s_cmd_v = v; s_cmd_w = w;   /* 斜坡與換算在控制步做(對 v、w 各自限斜率,兩輪才不會在過渡時不同步) */
    s_last_cmd_ms = now;
    s_have_cmd = 1;
    D->cmd_frames++;
}

static void report_odom(uint16_t seq, uint32_t flags, uint32_t now)
{
    odom_payload_t o;
    o.seq = seq;
    o.t_ms = now;
    o.x_mm = (int32_t)s_x_mm;
    o.y_mm = (int32_t)s_y_mm;
    o.th_mrad = (int32_t)(s_th_rad * 1000.0f);
    o.vl_mm_s = (int16_t)s_meas_l;
    o.vr_mm_s = (int16_t)s_meas_r;
    o.flags = (uint8_t)flags;
    o.flags_hi = (uint8_t)(flags >> 8);
    ctl_proto_send(MSG_ODOM, (const uint8_t *)&o, sizeof o);
}

static void report_motor_status_can(uint8_t seq, uint32_t flags)
{
    uint8_t d[8];
    d[0] = (uint8_t)(s_duty_l & 0xFF); d[1] = (uint8_t)((s_duty_l >> 8) & 0xFF);
    d[2] = (uint8_t)(s_duty_r & 0xFF); d[3] = (uint8_t)((s_duty_r >> 8) & 0xFF);
    d[4] = (uint8_t)flags; d[5] = seq; d[6] = 0; d[7] = 0;
    can_send(CAN_ID_MOTOR_STATUS, d, 8);
}

/* CAN FIFO 收乾:編碼器訊框計數(TIM 來源只計數;CAN 來源更新累計 tick)。
 * 裸機版在主迴圈每次醒來呼叫、RTOS 版在控制步開頭呼叫——各自原本的位置 */
void ctl_can_drain(void)
{
    uint32_t id; uint8_t d[8]; uint8_t dlc;
    while (can_recv(&id, d, &dlc)) {
        if (id == CAN_ID_ENCODER && dlc == 8) {
            D->enc_frames++;
#if !ENC_SOURCE_TIM
            s_enc_l = (int32_t)((uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24));
            s_enc_r = (int32_t)((uint32_t)d[4] | ((uint32_t)d[5] << 8) | ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24));
            D->enc_l = s_enc_l; D->enc_r = s_enc_r;
            s_enc_new++;
#endif
        }
    }
}

/* 收齊的框包:CMD_VEL 進命令、PING 記時並回 PONG */
void ctl_handle_frame(const rx_t *r, uint32_t now)
{
    if (r->type == MSG_CMD_VEL && r->len == 4) handle_cmd_vel(r->payload, now);
    else if (r->type == MSG_PING) {
        uint8_t v[2] = { FW_VERSION_MAJOR, FW_VERSION_MINOR };
        s_last_ping_ms = now; s_have_ping = 1; D->ping_frames++;
        ctl_proto_send(MSG_PONG, v, 2);
    }
}

/* 20 ms 回報:USART1 odom 框包 + CAN 0x201 馬達狀態(同一個 flags byte) */
void ctl_report(uint16_t seq, uint32_t now)
{
    report_odom(seq, D->flags, now);
    report_motor_status_can((uint8_t)seq, D->flags);
}

/* 暖重置偵測:真板看 RCC_CSR.IWDGRSTF(讀完用 RMVF 清);Renode 的 RCC 不設它,
 * 所以另外靠 .noinit 的計數——magic 還在就是重置過,不是上電。兩個都記進 g_dbg。 */
void ctl_boot_detect(void)
{
    D->boot_csr = RCC_CSR;
    RCC_CSR |= RCC_CSR_RMVF;
    if (g_noinit.magic == 0x4E4F494Eu) { g_noinit.resets++; s_warm_reset = 1; }
    else { g_noinit.magic = 0x4E4F494Eu; g_noinit.resets = 0; }
    if (D->boot_csr & RCC_CSR_IWDGRSTF) s_warm_reset = 1;
    D->resets = g_noinit.resets;
}

void ctl_iwdg_init_if_enabled(void)
{
    if (g_cfg.safety_mask & SAFETY_IWDG) iwdg_init((uint32_t)g_cfg.iwdg_ms);
}

/* 故障注入(只給 IWDG 驗收):tick 到 g_cfg.hang_at_ms 就該關中斷死迴圈;呼叫端決定在哪個位置檢查 */
int ctl_hang_due(uint32_t now)
{
    return g_cfg.hang_at_ms > 0 && (int32_t)(now - (uint32_t)g_cfg.hang_at_ms) >= 0;
}
