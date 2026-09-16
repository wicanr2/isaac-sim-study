/* 由 tools/gen_calib.py 從 calib.json 產生,不要手改。 */
#ifndef CALIB_H
#define CALIB_H

#define WHEEL_CIRC_UM            314159L      /* 輪周長,微米 */
#define TRACK_MM                 300
#define ENC_TICKS_PER_REV        4096
#define CONTROL_PERIOD_MS        5
#define REPORT_PERIOD_MS         20
#define CMD_TIMEOUT_MS           500
#define PWM_ARR                  999
#define PWM_PSC                  0  /* 載波 = 定時器時脈 / (PSC+1) / (ARR+1) */
#define DUTY_FULL_SCALE          1000
#define WHEEL_SPEED_FULL_MM_S    1000
#define PI_KP_Q8                 256
#define PI_KI_Q8                 6
#define PI_INTEGRAL_LIMIT        20000
#define ACCEL_LIMIT_MM_S2        1500  /* 線速度斜坡,0 = 不限 */
#define ALPHA_LIMIT_MRAD_S2      4000  /* 角速度斜坡,0 = 不限 */
#define FF_GAIN_Q8               256  /* 速度前饋比例,256 = 100% */
#define IWDG_TIMEOUT_MS          1000  /* 獨立看門狗;1..4096 */
#define HB_TIMEOUT_MS            300  /* 上位心跳(PING)逾時;0 = 不驗心跳 */
#define STALL_DUTY               600  /* 堵轉:|duty| ≥ 此值(‰)… */
#define STALL_SPEED_MM_S         20  /* …且 |輪速| < 此值(要 ≥ 一個 tick 的速度量子:76.7 µm / 5 ms = 15 mm/s)… */
#define STALL_MS                 200  /* …持續這麼久 → STALL */
#define SAFETY_MASK              0x1F  /* 五項安全功能全開;負對照才關 */
#define ENC_SOURCE_TIM           1  /* 1: TIM2/TIM4 encoder mode 讀 CNT;0: CAN 0x181 訊框 */
#define CAN_ID_ENCODER           0x181
#define CAN_ID_MOTOR_STATUS      0x201

#endif
