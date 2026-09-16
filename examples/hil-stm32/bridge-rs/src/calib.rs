//! 讀 calib.json(韌體、橋接、受控體的唯一參數來源)與 nm 產生的符號表。
//! 沒有 JSON crate:這個檔案是平的 `"key": number`,用最小的掃描器就夠。

use std::fs;
use std::io;

#[derive(Debug, Clone, Copy)]
pub struct Calib {
    pub wheel_circ_um: f64,
    pub track_mm: f64,
    pub ticks_per_rev: i64,
    pub control_period_ms: u64,
    pub report_period_ms: u64,
    pub pwm_arr: u32,
    pub duty_full_scale: i64,
    pub wheel_speed_full_mm_s: f64,
    pub can_id_encoder: u32,
    pub can_id_motor_status: u32,
    /// 馬達層(三個受控體實作同一份公式):一階時間常數、加速度上限(電流限制)、死區
    pub motor_tau_s: f64,
    pub motor_accel_max_mm_s2: f64,
    pub motor_deadband_duty: f64,
    /// 韌體的編碼器來源:true = TIM2/TIM4 encoder mode(calib "encoder_source": "tim"),false = CAN 0x181
    pub encoder_tim: bool,
    /// 安全 I/O(韌體從同一份 calib 拿;橋接拿來算判準的時限)
    pub iwdg_timeout_ms: u64,
    pub heartbeat_timeout_ms: u64,
    pub heartbeat_period_ms: u64,
    pub stall_ms: u64,
    pub accel_limit_mm_s2: f64,
    /// 上位序列線的鮑率:橋接按它限制每步注入 USART1 的 byte 數(上位送得再快,線只有這麼寬)
    pub uart_baud: u64,
}

fn num(text: &str, key: &str) -> io::Result<f64> {
    let pat = format!("\"{}\"", key);
    let i = text.find(&pat).ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, format!("calib.json 缺 {key}")))?;
    let rest = &text[i + pat.len()..];
    let rest = rest.trim_start().strip_prefix(':').unwrap_or(rest).trim_start();
    let end = rest.find(|c: char| c == ',' || c == '}' || c == '\n').unwrap_or(rest.len());
    rest[..end].trim().parse::<f64>().map_err(|e| io::Error::new(io::ErrorKind::InvalidData, format!("{key}: {e}")))
}

impl Calib {
    pub fn load(path: &str) -> io::Result<Calib> {
        let t = fs::read_to_string(path)?;
        let r = num(&t, "wheel_radius_mm")?;
        let encoder_tim = t.contains("\"encoder_source\": \"tim\"");
        Ok(Calib {
            wheel_circ_um: (2.0 * std::f64::consts::PI * r * 1000.0).round(),
            track_mm: num(&t, "track_mm")?,
            ticks_per_rev: num(&t, "encoder_ticks_per_rev")? as i64,
            control_period_ms: num(&t, "control_period_ms")? as u64,
            report_period_ms: num(&t, "report_period_ms")? as u64,
            pwm_arr: num(&t, "pwm_arr")? as u32,
            duty_full_scale: num(&t, "duty_full_scale")? as i64,
            wheel_speed_full_mm_s: num(&t, "wheel_speed_at_full_duty_mm_s")?,
            can_id_encoder: num(&t, "can_id_encoder")? as u32,
            can_id_motor_status: num(&t, "can_id_motor_status")? as u32,
            encoder_tim,
            motor_tau_s: num(&t, "motor_tau_s").unwrap_or(0.05),
            motor_accel_max_mm_s2: num(&t, "motor_accel_max_mm_s2").unwrap_or(0.0),
            motor_deadband_duty: num(&t, "motor_deadband_duty").unwrap_or(0.0),
            iwdg_timeout_ms: num(&t, "iwdg_timeout_ms").unwrap_or(1000.0) as u64,
            heartbeat_timeout_ms: num(&t, "heartbeat_timeout_ms").unwrap_or(300.0) as u64,
            heartbeat_period_ms: num(&t, "heartbeat_period_ms").unwrap_or(100.0) as u64,
            stall_ms: num(&t, "stall_ms").unwrap_or(200.0) as u64,
            accel_limit_mm_s2: num(&t, "accel_limit_mm_s2").unwrap_or(0.0),
            uart_baud: num(&t, "uart_baud").unwrap_or(115200.0) as u64,
        })
    }
}

/// 從 `arm-none-eabi-nm -n` 的輸出找符號位址。
pub fn symbol_addr(sym_path: &str, name: &str) -> io::Result<u64> {
    let t = fs::read_to_string(sym_path)?;
    for line in t.lines() {
        let f: Vec<&str> = line.split_whitespace().collect();
        if f.len() == 3 && f[2] == name {
            return u64::from_str_radix(f[0], 16).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e.to_string()));
        }
    }
    Err(io::Error::new(io::ErrorKind::NotFound, format!("{sym_path} 裡沒有 {name}")))
}
