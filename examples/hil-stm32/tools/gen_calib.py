#!/usr/bin/env python3
"""從 calib.json 產生 firmware/calib.h。只用標準庫。

用法(在 examples/hil-stm32/ 底下):
    python3 tools/gen_calib.py
"""
import json
import pathlib

HERE = pathlib.Path(__file__).resolve().parent.parent
src = HERE / "calib.json"
dst = HERE / "firmware" / "calib.h"

c = json.loads(src.read_text(encoding="utf-8"))
circ_um = round(2 * 3.141592653589793 * c["wheel_radius_mm"] * 1000)

lines = [
    "/* 由 tools/gen_calib.py 從 calib.json 產生,不要手改。 */",
    "#ifndef CALIB_H",
    "#define CALIB_H",
    "",
    f"#define WHEEL_CIRC_UM            {circ_um}L      /* 輪周長,微米 */",
    f"#define TRACK_MM                 {int(round(c['track_mm']))}",
    f"#define ENC_TICKS_PER_REV        {c['encoder_ticks_per_rev']}",
    f"#define CONTROL_PERIOD_MS        {c['control_period_ms']}",
    f"#define REPORT_PERIOD_MS         {c['report_period_ms']}",
    f"#define CMD_TIMEOUT_MS           {c['cmd_timeout_ms']}",
    f"#define PWM_ARR                  {c['pwm_arr']}",
    f"#define PWM_PSC                  {c.get('pwm_prescaler', 0)}  /* 載波 = 定時器時脈 / (PSC+1) / (ARR+1) */",
    f"#define DUTY_FULL_SCALE          {c['duty_full_scale']}",
    f"#define WHEEL_SPEED_FULL_MM_S    {int(round(c['wheel_speed_at_full_duty_mm_s']))}",
    f"#define PI_KP_Q8                 {c['pi_kp_q8']}",
    f"#define PI_KI_Q8                 {c['pi_ki_q8']}",
    f"#define PI_INTEGRAL_LIMIT        {c['pi_integral_limit']}",
    f"#define ACCEL_LIMIT_MM_S2        {c.get('accel_limit_mm_s2', 0)}  /* 線速度斜坡,0 = 不限 */",
    f"#define ALPHA_LIMIT_MRAD_S2      {c.get('alpha_limit_mrad_s2', 0)}  /* 角速度斜坡,0 = 不限 */",
    f"#define FF_GAIN_Q8               {c.get('ff_gain_q8', 256)}  /* 速度前饋比例,256 = 100% */",
    f"#define ENC_SOURCE_TIM           {1 if c.get('encoder_source', 'can') == 'tim' else 0}  /* 1: TIM2/TIM4 encoder mode 讀 CNT;0: CAN 0x181 訊框 */",
    f"#define CAN_ID_ENCODER           0x{c['can_id_encoder']:X}",
    f"#define CAN_ID_MOTOR_STATUS      0x{c['can_id_motor_status']:X}",
    "",
    "#endif",
    "",
]
dst.write_text("\n".join(lines), encoding="utf-8")
print(f"wrote {dst.relative_to(HERE)}")
