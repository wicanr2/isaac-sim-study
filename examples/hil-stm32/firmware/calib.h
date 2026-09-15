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
#define PI_KP_Q8                 128
#define PI_KI_Q8                 13
#define PI_INTEGRAL_LIMIT        20000
#define CAN_ID_ENCODER           0x181
#define CAN_ID_MOTOR_STATUS      0x201

#endif
