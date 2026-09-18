/* control.h:兩版韌體共用層的介面(實作在 control.c)。誰排程、誰餵狗、ISR 怎麼叫醒主迴圈在各自的 main。 */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "proto.h"

/* 觀測用的全域狀態:volatile,固定版面,橋接以 sysbus 讀(bridge-rs dbg::WORDS = 33)。
 * RTOS 版把它當第一個成員、後面接自己的欄位。 */
typedef struct {
    volatile uint32_t magic;        /* 0x48494C31 "HIL1":橋接用來確認讀對位址 */
    volatile uint32_t tick_ms;      /* 毫秒計數(裸機 SysTick / RTOS tick) */
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
    volatile uint32_t imu_whoami;   /* 開機時讀到的 LSM330 WHO_AM_I_G(0xD4 = 有 IMU);0x100 | 錯誤碼 = I2C 沒回應 */
    volatile int32_t  gyro_z;       /* 陀螺儀 z 軸 mrad/s(每個控制步讀一次) */
    volatile int32_t  yaw_resid;    /* |(陀螺儀 − 零偏) − 輪差 yaw rate| 的滑動平均 mrad/s(打滑判斷) */
    volatile int32_t  gyro_bias;    /* 靜止時估的陀螺儀零偏 mrad/s;還沒估出來是 0x7FFFFFFF(docs/hil/36 §3.2) */
    volatile uint32_t acc_whoami;   /* 開機讀到的 WHO_AM_I_A(0x40 = 有加速度計);0x100 | 錯誤碼 = I2C 沒回應 */
    volatile int32_t  acc_x;        /* 加速度計前進軸 mm/s²(每個控制步讀一次) */
    volatile int32_t  acc_bias;     /* 靜止時估的加速度計零偏 mm/s²;還沒估出來是 0x7FFFFFFF */
    volatile int32_t  vel_resid;    /* |漏積分(a_x − 零偏 − 輪速微分)| mm/s(平移打滑判斷,docs/hil/38 §1.4) */
    volatile uint32_t slip_src;     /* 誰讓 SLIP 亮過:bit0 陀螺儀、bit1 加速度計(開機後累積) */
    volatile uint32_t gyro_steps;   /* 航向增量用了陀螺儀的步數(yaw 融合,docs/hil/38 §1.5) */
    volatile int32_t  tc_cap;       /* 牽引力控制當下的 duty 上限(‰;docs/hil/36 §3.4) */
    volatile uint32_t imu_fail;     /* I2C 讀感測器失敗、做過匯流排復原的次數(docs/hil/36 §3.5) */
    volatile int32_t  range_mm;     /* 近距離感測器最後一筆讀數 mm;沒收到過是 -1(docs/hil/38 §1.9) */
} dbg_common_t;

/* 跨 reset 保留的區段:startup 不清、LoadELF 不寫。magic 對就是暖重置。 */
typedef struct {
    volatile uint32_t magic;        /* 0x4E4F494E "NOIN" */
    volatile uint32_t resets;
} noinit_t;

/* 執行期可改的控制參數:預設從 calib 來;橋接在開機前把 --cfg 寫進 flash 裡 .data 的初始值(LMA),
 * startup 照常複製,所以 IWDG 這種「init 就定案」的參數也改得到。每個控制步都重新讀。 */
typedef struct {
    volatile uint32_t magic;        /* 0x48494C43 "HILC" */
    volatile int32_t  kp_q8, ki_q8;
    volatile int32_t  accel_mm_s2;  /* 線速度斜坡上限,0 = 不限 */
    volatile int32_t  ff_q8;        /* 前饋比例,256 = 100% */
    volatile int32_t  alpha_mrad_s2;/* 角速度斜坡上限,0 = 不限 */
    volatile int32_t  iwdg_ms;      /* IWDG 逾時;由 safety_mask 決定開不開 */
    volatile int32_t  hang_at_ms;   /* 故障注入:tick 到這個值時關中斷死迴圈(0 = 不注入)。只給 IWDG 驗收用 */
    volatile int32_t  hb_timeout_ms;/* 心跳逾時;0 = 不驗 */
    volatile int32_t  stall_duty;   /* 堵轉判定 duty 門檻(‰) */
    volatile int32_t  stall_ms;     /* 堵轉判定持續時間 */
    volatile uint32_t safety_mask;  /* SAFETY_*,全開 0x3F;負對照關一項 */
    volatile int32_t  slip_mrad_s;  /* 打滑:yaw 殘差門檻 */
    volatile int32_t  slip_ms;      /* 打滑:殘差超過門檻持續多久 */
    volatile int32_t  gyro_bias_still_ms; /* 陀螺儀零偏:靜止滿這麼久才開始估;0 = 不估(零偏固定 0) */
    volatile int32_t  slip_vel_mm_s;/* 平移打滑:速度殘差門檻 */
    volatile int32_t  yaw_fusion;   /* 1 = 殘差超過打滑門檻的步,航向增量改用陀螺儀;0 = 只用輪差 */
    volatile int32_t  traction_ctl; /* 1 = 打滑時壓 duty 上限(牽引力控制,docs/hil/36 §3.4);0 = 不壓 */
    volatile int32_t  traction_cap_step; /* 每個控制步調整的量(‰) */
    volatile int32_t  traction_cap_min;  /* duty 上限的下限(‰) */
    volatile int32_t  traction_recover_ms; /* 殘差在門檻以下連續這麼久才開始放回 */
    volatile int32_t  zone_slow_mm; /* 近距離安全區:量到比這個近就開始減速(docs/hil/38 §1.9) */
    volatile int32_t  zone_stop_mm; /* …比這個近就不准前進 */
} cfg_t;

extern cfg_t g_cfg;
extern noinit_t g_noinit;

/* 接收狀態機:一次餵一個 byte,收齊且 CRC 對才回 1 */
typedef struct {
    uint8_t state, len, type, idx;
    uint8_t payload[PROTO_MAX_PAYLOAD];
    uint8_t crc_lo;
} rx_t;

/* 開機 */
void ctl_bind_dbg(dbg_common_t *dbg);      /* 第一件事:g_dbg 前 23 字在哪;順便寫 magic */
void ctl_uart_init(uint32_t base);
void ctl_uart_putc(uint32_t base, uint8_t c);
void dbg_puts(const char *s);              /* USART2 printer */
void dbg_put_u32(uint32_t v);
void ctl_gpio_init(void);
void ctl_encoder_init(void);               /* 只在 ENC_SOURCE_TIM */
void ctl_pwm_init(void);
void ctl_imu_init(void);                   /* I2C3 + LSM330 陀螺儀;沒有 IMU(I2C 沒回應)就記在 g_dbg、打滑偵測不做 */
int  ctl_can_init(uint32_t (*now_ms)(void)); /* now_ms 給交握逾時用;NULL = 用迭代數(RTOS 版,tick 還沒走) */
void ctl_motor_apply(int32_t duty_l, int32_t duty_r, int enable);
void ctl_boot_detect(void);                /* RCC_CSR + .noinit → resets、boot_csr、暖重置旗標 */
void ctl_iwdg_init_if_enabled(void);
void ctl_iwdg_feed(void);                  /* safety_mask 沒開 IWDG 就什麼都不做 */

/* 資料路徑;now 一律是毫秒(裸機 SysTick 計數 / RTOS xTaskGetTickCount,兩者 1 kHz) */
int  ctl_proto_feed(rx_t *r, uint8_t b);
void ctl_proto_send(uint8_t type, const uint8_t *payload, uint8_t len);
void ctl_handle_frame(const rx_t *r, uint32_t now);   /* CMD_VEL / PING(回 PONG) */
void ctl_can_drain(void);                  /* 收乾 CAN FIFO */
void ctl_control_step(uint32_t now);       /* 編碼器、里程計、安全閘門、斜坡、PI、PWM、g_dbg */
void ctl_report(uint16_t seq, uint32_t now);   /* odom 框包 + CAN 0x201 */
int  ctl_hang_due(uint32_t now);           /* 故障注入到時(呼叫端決定在哪關中斷) */

#endif
