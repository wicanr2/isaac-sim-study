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
    f"#define IWDG_TIMEOUT_MS          {c.get('iwdg_timeout_ms', 1000)}  /* 獨立看門狗;1..4096 */",
    f"#define HB_TIMEOUT_MS            {c.get('heartbeat_timeout_ms', 300)}  /* 上位心跳(PING)逾時;0 = 不驗心跳 */",
    f"#define STALL_DUTY               {c.get('stall_duty_permille', 600)}  /* 堵轉:|duty| ≥ 此值(‰)… */",
    f"#define STALL_SPEED_MM_S         {c.get('stall_speed_mm_s', 20)}  /* …且 |輪速| < 此值(要 ≥ 一個 tick 的速度量子:76.7 µm / 5 ms = 15 mm/s)… */",
    f"#define STALL_MS                 {c.get('stall_ms', 200)}  /* …持續這麼久 → STALL */",
    f"#define SAFETY_MASK              0x{c.get('safety_mask', 63):X}  /* 五項安全 I/O + 打滑偵測兩半 + 近距離安全區全開;負對照才關 */",
    f"#define SLIP_RESID_MRAD_S        {c.get('slip_resid_mrad_s', 45)}  /* 打滑:|陀螺儀 − 輪差| yaw rate 50 ms 平均的門檻 */",
    f"#define SLIP_MS                  {c.get('slip_ms', 50)}  /* …持續這麼久 → SLIP */",
    f"#define GYRO_BIAS_STILL_MS       {c.get('gyro_bias_still_ms', 200)}  /* 陀螺儀零偏:靜止滿這麼久才估;0 = 不估 */",
    f"#define SLIP_VEL_MM_S            {c.get('slip_vel_mm_s', 90)}  /* 平移打滑:加速度計速度殘差門檻(docs/hil/38 §1.4) */",
    f"#define YAW_FUSION               {c.get('yaw_fusion', 1)}  /* 1 = 打滑片段的航向增量改用陀螺儀(docs/hil/38 §1.5) */",
    f"#define TRACTION_CTL             {c.get('traction_ctl', 1)}  /* 1 = 打滑時壓 duty 上限(docs/hil/36 §3.4) */",
    f"#define TRACTION_CAP_STEP        {c.get('traction_cap_step', 20)}  /* 每個控制步調整 duty 上限的量(‰) */",
    f"#define TRACTION_CAP_MIN         {c.get('traction_cap_min', 100)}  /* duty 上限壓到這裡為止(‰) */",
    f"#define TRACTION_RECOVER_MS      {c.get('traction_recover_ms', 200)}  /* 殘差在門檻以下連續這麼久才放回 */",
    f"#define ZONE_SLOW_MM             {c.get('zone_slow_mm', 700)}  /* 近距離安全區:量到比這個近就開始減速(docs/hil/38 §1.9) */",
    f"#define ZONE_STOP_MM             {c.get('zone_stop_mm', 350)}  /* …比這個近就不准前進(距離是從車體中心量的) */",
    f"#define ENC_SOURCE_TIM           {1 if c.get('encoder_source', 'can') == 'tim' else 0}  /* 1: TIM2/TIM4 encoder mode 讀 CNT;0: CAN 0x181 訊框 */",
    f"#define CAN_ID_ENCODER           0x{c['can_id_encoder']:X}",
    f"#define CAN_ID_MOTOR_STATUS      0x{c['can_id_motor_status']:X}",
    f"#define CAN_ID_RANGE             0x{c.get('can_id_range', 0x301):X}  /* 近距離感測器:u16 mm(小端),20 ms 一筆 */",
    "",
    "#endif",
    "",
]
dst.write_text("\n".join(lines), encoding="utf-8")
print(f"wrote {dst.relative_to(HERE)}")
