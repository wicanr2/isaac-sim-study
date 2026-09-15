//! hil-bridge:Renode 裡的 STM32 韌體 ↔ 受控體 的 lockstep 橋接。
//!
//! 每一步(預設 5 ms):
//!   1. 上位腳本 → cmd_vel 框包 → hook(UART 注入)→ 等 ack
//!   2. External Control run_for(5 ms)
//!   3. 從匯流排讀 MCU 的輸出:TIM3 CCR1/CCR2(duty)、PB8/PB9/PB10(方向、致能)、g_dbg(SRAM)
//!   4. 收 MCU 這一步吐出的 UART(odom)與 CAN(狀態)
//!   5. 受控體走 5 ms → 編碼器 tick → hook(CAN 注入)→ 等 ack
//!   6. 寫一行 CSV
//! 結束後跑驗收檢查,任何一項 FAIL 就以非零碼離開。
//!
//! 橋接不做安全:不夾限、不逾時、不幫忙停車;它只轉譯與紀錄。

mod calib;
mod ec;
mod hook;
mod plant;
mod proto;

use std::io::Write;
use std::time::{Duration, Instant};

use plant::{MotorCmd, Plant};

struct Args {
    ec: String,
    hook: String,
    machine: String,
    sym: String,
    calib: String,
    plant: String,
    seconds: f64,
    script: String,
    log: String,
    negative: String,
    boot_ms: u64,
}

fn parse_args() -> Args {
    let mut a = Args {
        ec: "127.0.0.1:3500".into(),
        hook: "127.0.0.1:3600".into(),
        machine: "hilctl".into(),
        sym: "firmware/build/hilctl.sym".into(),
        calib: "calib.json".into(),
        plant: "fake".into(),
        seconds: 6.0,
        script: "0:0,0;0.5:300,0;3.5:0,600;5:0,0".into(),
        log: "out/run.csv".into(),
        negative: "none".into(),
        boot_ms: 100,
    };
    let v: Vec<String> = std::env::args().collect();
    let mut i = 1;
    while i + 1 < v.len() {
        let val = v[i + 1].clone();
        match v[i].as_str() {
            "--ec" => a.ec = val,
            "--hook" => a.hook = val,
            "--machine" => a.machine = val,
            "--sym" => a.sym = val,
            "--calib" => a.calib = val,
            "--plant" => a.plant = val,
            "--seconds" => a.seconds = val.parse().expect("--seconds"),
            "--script" => a.script = val,
            "--log" => a.log = val,
            "--negative" => a.negative = val,
            "--boot-ms" => a.boot_ms = val.parse().expect("--boot-ms"),
            other => {
                eprintln!("未知參數 {other}");
                std::process::exit(2);
            }
        }
        i += 2;
    }
    a
}

/// 上位腳本:"t:v,w;t:v,w" → 時間 t(秒)起的 (v mm/s, w mrad/s)
fn parse_script(s: &str) -> Vec<(f64, i16, i16)> {
    let mut out: Vec<(f64, i16, i16)> = Vec::new();
    for seg in s.split(';') {
        let seg = seg.trim();
        if seg.is_empty() {
            continue;
        }
        let (t, vw) = seg.split_once(':').expect("script 段落格式 t:v,w");
        let (v, w) = vw.split_once(',').expect("script 段落格式 t:v,w");
        out.push((t.trim().parse().unwrap(), v.trim().parse().unwrap(), w.trim().parse().unwrap()));
    }
    out.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    out
}

fn cmd_at(script: &[(f64, i16, i16)], t: f64) -> (i16, i16) {
    let mut cur = (0i16, 0i16);
    for &(ts, v, w) in script {
        if t + 1e-9 >= ts {
            cur = (v, w);
        }
    }
    cur
}

/// g_dbg 的版面(firmware/main.c 的 dbg_t),以 u32 為單位的索引
mod dbg {
    pub const MAGIC: u64 = 0;
    pub const TICK_MS: u64 = 1;
    pub const CTRL_STEPS: u64 = 2;
    pub const SP_L: u64 = 3;
    pub const SP_R: u64 = 4;
    pub const MEAS_L: u64 = 5;
    pub const MEAS_R: u64 = 6;
    pub const DUTY_L: u64 = 7;
    pub const DUTY_R: u64 = 8;
    pub const ENC_FRAMES: u64 = 11;
    pub const CMD_FRAMES: u64 = 12;
    pub const BAD_CRC: u64 = 13;
    pub const FLAGS: u64 = 14;
    pub const INIT_ERR: u64 = 15;
    pub const RX_OVERFLOW: u64 = 16;
    pub const MAGIC_VALUE: u32 = 0x4849_4C31;
    pub const WORDS: u32 = 17;
}

const TIM3_BASE: u64 = 0x4000_0400;
const TIM3_CCR1: u64 = TIM3_BASE + 0x34;
const TIM3_CCR2: u64 = TIM3_BASE + 0x38;
const TIM3_ARR: u64 = TIM3_BASE + 0x2C;
const DIR_L_PIN: i32 = 8;
const DIR_R_PIN: i32 = 9;
const MOTOR_EN_PIN: i32 = 10;

struct Check {
    name: &'static str,
    pass: bool,
    detail: String,
}

fn main() {
    let a = parse_args();
    let c = calib::Calib::load(&a.calib).expect("calib.json");
    let dbg_base = calib::symbol_addr(&a.sym, "g_dbg").expect("符號 g_dbg");
    let script = parse_script(&a.script);
    let dt_s = c.control_period_ms as f64 / 1000.0;
    let steps = (a.seconds / dt_s).round() as u32;
    let report_every = (c.report_period_ms / c.control_period_ms).max(1) as u32;

    let mut ec = ec::Renode::connect_retry(a.ec.as_str(), Duration::from_secs(90)).expect("連 External Control");
    let m = ec.machine(&a.machine).expect("machine");
    let bus = ec.sysbus(m).expect("sysbus");
    let gpio_b = ec
        .gpio(m, "sysbus.gpioPortB")
        .or_else(|_| ec.gpio(m, "gpioPortB"))
        .expect("gpioPortB");
    let mut hk = hook::Hook::connect_retry(a.hook.as_str(), Duration::from_secs(90)).expect("連 hil_hook");

    let mut pl: Box<dyn Plant> = if a.plant == "fake" {
        Box::new(plant::Fake::new(c))
    } else if let Some(addr) = a.plant.strip_prefix("udp:") {
        Box::new(plant::Udp::connect(addr).expect("UDP plant"))
    } else if let Some(addr) = a.plant.strip_prefix("tcp:") {
        Box::new(plant::Tcp::connect(addr).expect("TCP plant"))
    } else {
        eprintln!("--plant 只接受 fake、udp:host:port 或 tcp:host:port");
        std::process::exit(2);
    };

    // 先讓韌體開機:LoadELF 之後機器是暫停的,main 還沒跑,g_dbg 全零
    ec.run_for_us(a.boot_ms * 1000).expect("boot run_for");

    // 生效證明:每個變數都印一行,證明它進了系統
    let magic = ec.read_u32_at(bus, dbg_base + 4 * dbg::MAGIC).expect("讀 magic");
    let arr = ec.read_u32_at(bus, TIM3_ARR).expect("讀 ARR");
    let t0 = ec.time_us().expect("time");
    println!("[effect] renode ec={} hook={} machine={} boot_ms={} t0_us={}", a.ec, a.hook, a.machine, a.boot_ms, t0);
    println!("[effect] g_dbg@0x{:08x} magic=0x{:08x} ({}) init_err={}",
        dbg_base, magic, if magic == dbg::MAGIC_VALUE { "ok" } else { "MISMATCH" },
        ec.read_u32_at(bus, dbg_base + 4 * dbg::INIT_ERR).unwrap());
    println!("[effect] plant={} dt_ms={} steps={} report_every={} script={:?} negative={}",
        a.plant, c.control_period_ms, steps, report_every, a.script, a.negative);
    println!("[effect] tim3 ARR={} (calib pwm_arr={}) track={} circ_um={} tpr={}",
        arr, c.pwm_arr, c.track_mm, c.wheel_circ_um, c.ticks_per_rev);
    if magic != dbg::MAGIC_VALUE {
        eprintln!("g_dbg magic 不對:讀到的不是這支韌體,或位址錯");
        std::process::exit(1);
    }

    if let Some(dir) = std::path::Path::new(&a.log).parent() {
        let _ = std::fs::create_dir_all(dir);
    }
    let mut log = std::fs::File::create(&a.log).expect("log");
    writeln!(log, "step,t_us,cmd_v,cmd_w,sp_l,sp_r,meas_l,meas_r,duty_l_dbg,ccr1,ccr2,dir_l,dir_r,en,flags,\
plant_x,plant_y,plant_th,plant_vl,plant_vr,ticks_l,ticks_r,odom_seq,odom_x,odom_y,odom_th,odom_vl,odom_vr,odom_flags,can_duty_l,can_duty_r").unwrap();

    let mut parser = proto::Parser::default();
    let mut last_odom = proto::Odom::default();
    let mut odom_count = 0u32;
    let mut can_status: Option<(i16, i16, u8, u8)> = None;
    let mut can_status_count = 0u32;
    let mut sent_cmds = 0u32;
    let mut corrupted = 0u32;
    let mut last_plant = plant::PlantOut::default();
    let mut can_cmp_total = 0u32;
    let mut can_cmp_mismatch = 0u32;
    let wall0 = Instant::now();

    for k in 0..steps {
        let t_s = k as f64 * dt_s;

        // 1. 上位:每個回報週期送一次 cmd_vel
        if k % report_every == 0 {
            let (v, w) = cmd_at(&script, t_s);
            let mut f = proto::cmd_vel(v, w);
            if a.negative == "bad-crc" {
                let n = f.len();
                f[n - 1] ^= 0xFF;
                corrupted += 1;
            }
            hk.uart_send(&f).expect("uart_send");
            sent_cmds += 1;
            hk.wait_acks().expect("ack");
        }

        // 2. 推進 Renode
        ec.run_for_us(c.control_period_ms * 1000).expect("run_for");

        // 3. 讀匯流排
        let ccr1 = ec.read_u32_at(bus, TIM3_CCR1).unwrap();
        let ccr2 = ec.read_u32_at(bus, TIM3_CCR2).unwrap();
        let dir_l = ec.gpio_get(gpio_b, DIR_L_PIN).unwrap();
        let dir_r = ec.gpio_get(gpio_b, DIR_R_PIN).unwrap();
        let en = ec.gpio_get(gpio_b, MOTOR_EN_PIN).unwrap();
        let d = ec.read_u32s_at(bus, dbg_base, dbg::WORDS).unwrap();
        let sp_l = d[dbg::SP_L as usize] as i32;
        let sp_r = d[dbg::SP_R as usize] as i32;
        let meas_l = d[dbg::MEAS_L as usize] as i32;
        let meas_r = d[dbg::MEAS_R as usize] as i32;
        let duty_l_dbg = d[dbg::DUTY_L as usize] as i32;
        let flags = d[dbg::FLAGS as usize];
        let t_us = ec.time_us().unwrap();

        // 4. 收 MCU 的輸出
        hk.uart_flush_request().unwrap();
        hk.wait_acks().unwrap();
        hk.drain(Duration::from_millis(1)).unwrap();
        let mut frames = Vec::new();
        let mut pending_can: Option<(i16, i16)> = None;
        for r in hk.take_inbox() {
            if r.id == hook::ID_UART_FROM_MCU {
                parser.feed(&r.data, &mut frames);
            } else if r.id == c.can_id_motor_status && r.data.len() >= 6 {
                let dl = i16::from_le_bytes([r.data[0], r.data[1]]);
                let dr = i16::from_le_bytes([r.data[2], r.data[3]]);
                can_status = Some((dl, dr, r.data[4], r.data[5]));
                can_status_count += 1;
                pending_can = Some((dl, dr));
            } else if r.id == hook::ID_BUS_SNAPSHOT && r.data.len() == 8 {
                // 兩條獨立管道在同一個模擬時刻比:CAN 狀態框裡的 duty vs hook 在 FrameSent
                // 當下讀到的 TIM3 CCR。步邊界取樣看不到步內的中間值,所以不能拿 ccr1 來比。
                if let Some((dl, dr)) = pending_can.take() {
                    let s1 = u32::from_le_bytes([r.data[0], r.data[1], r.data[2], r.data[3]]);
                    let s2 = u32::from_le_bytes([r.data[4], r.data[5], r.data[6], r.data[7]]);
                    let to_duty = |ccr: u32| ((ccr as f64) * c.duty_full_scale as f64 / (arr as f64 + 1.0)).round() as i16;
                    can_cmp_total += 1;
                    if (dl.abs() - to_duty(s1)).abs() > 1 || (dr.abs() - to_duty(s2)).abs() > 1 {
                        can_cmp_mismatch += 1;
                    }
                }
            }
        }
        for (t, p) in frames {
            if t == proto::MSG_ODOM {
                if let Some(o) = proto::parse_odom(&p) {
                    last_odom = o;
                    odom_count += 1;
                }
            }
        }

        // 5. 受控體
        let cmd = MotorCmd {
            duty_l: ccr1 as f64 / (arr as f64 + 1.0),
            duty_r: ccr2 as f64 / (arr as f64 + 1.0),
            fwd_l: dir_l,
            fwd_r: dir_r,
            enabled: en,
        };
        let out = pl.step(k, dt_s, cmd).expect("plant step");
        last_plant = out;
        let mut enc = [0u8; 8];
        enc[..4].copy_from_slice(&out.ticks_l.to_le_bytes());
        enc[4..].copy_from_slice(&out.ticks_r.to_le_bytes());
        hk.can_send(c.can_id_encoder, &enc).unwrap();
        hk.wait_acks().unwrap();

        // 6. 紀錄
        let (cv, cw) = cmd_at(&script, t_s);
        let cs = can_status.unwrap_or((0, 0, 0, 0));
        writeln!(log, "{k},{t_us},{cv},{cw},{sp_l},{sp_r},{meas_l},{meas_r},{duty_l_dbg},{ccr1},{ccr2},{},{},{},{flags},\
{:.1},{:.1},{:.4},{:.1},{:.1},{},{},{},{},{},{},{},{},{},{},{}",
            dir_l as u8, dir_r as u8, en as u8,
            out.x_mm, out.y_mm, out.th_rad, out.vl_mm_s, out.vr_mm_s, out.ticks_l, out.ticks_r,
            last_odom.seq, last_odom.x_mm, last_odom.y_mm, last_odom.th_mrad, last_odom.vl_mm_s, last_odom.vr_mm_s,
            last_odom.flags, cs.0, cs.1).unwrap();
    }

    let wall = wall0.elapsed();
    let t_end = ec.time_us().unwrap();
    let bad_crc = ec.read_u32_at(bus, dbg_base + 4 * dbg::BAD_CRC).unwrap();
    let cmd_frames = ec.read_u32_at(bus, dbg_base + 4 * dbg::CMD_FRAMES).unwrap();
    let enc_frames = ec.read_u32_at(bus, dbg_base + 4 * dbg::ENC_FRAMES).unwrap();
    let ctrl_steps = ec.read_u32_at(bus, dbg_base + 4 * dbg::CTRL_STEPS).unwrap();
    let rx_overflow = ec.read_u32_at(bus, dbg_base + 4 * dbg::RX_OVERFLOW).unwrap();
    let tick_ms = ec.read_u32_at(bus, dbg_base + 4 * dbg::TICK_MS).unwrap();

    println!("[run] steps={} renode_t_us={} wall={:.2}s ({:.1} ms/step, x{:.2} realtime)",
        steps, t_end, wall.as_secs_f64(), wall.as_secs_f64() * 1000.0 / steps as f64,
        (t_end as f64 / 1e6) / wall.as_secs_f64());
    println!("[run] fw tick_ms={} ctrl_steps={} cmd_frames={} enc_frames={} bad_crc={} rx_overflow={}",
        tick_ms, ctrl_steps, cmd_frames, enc_frames, bad_crc, rx_overflow);
    println!("[run] odom_frames={} can_status_frames={} sent_cmds={} corrupted={}",
        odom_count, can_status_count, sent_cmds, corrupted);
    println!("[run] plant  x={:.1} y={:.1} th={:.4}", last_plant.x_mm, last_plant.y_mm, last_plant.th_rad);
    println!("[run] odom   x={} y={} th={:.4} (seq {})", last_odom.x_mm, last_odom.y_mm,
        last_odom.th_mrad as f64 / 1000.0, last_odom.seq);

    // 驗收
    let expect_time = steps as u64 * c.control_period_ms * 1000;
    let dist = (last_plant.x_mm.powi(2) + last_plant.y_mm.powi(2)).sqrt();
    let tol_mm = 25.0 + 0.02 * dist;
    let dx = (last_odom.x_mm as f64 - last_plant.x_mm).abs();
    let dy = (last_odom.y_mm as f64 - last_plant.y_mm).abs();
    let dth = (last_odom.th_mrad as f64 / 1000.0 - last_plant.th_rad).abs();
    let expect_move = script.iter().any(|&(_, v, w)| v != 0 || w != 0);

    let checks = vec![
        Check { name: "C1 時間完整性 renode_t == steps*dt", pass: t_end - t0 == expect_time,
            detail: format!("{} vs {}", t_end - t0, expect_time) },
        Check { name: "C2 車有動(腳本有命令時)", pass: !expect_move || dist > 100.0,
            detail: format!("plant 位移 {:.1} mm", dist) },
        Check { name: "C3 韌體 odom 對受控體真值", pass: dx <= tol_mm && dy <= tol_mm && dth <= 0.03,
            detail: format!("dx={dx:.1} dy={dy:.1} dth={dth:.4} (tol {tol_mm:.1} mm / 0.03 rad)") },
        Check { name: "C4 兩條獨立管道一致:每筆 CAN 狀態 duty == 同一時刻的 CCR 快照", pass: can_cmp_total > 0 && can_cmp_mismatch == 0,
            detail: format!("{} 筆比對,{} 筆不符", can_cmp_total, can_cmp_mismatch) },
        Check { name: "C5 odom 回報數 ≥ 90% 期望", pass: odom_count as f64 >= 0.9 * (steps / report_every) as f64,
            detail: format!("{} / {}", odom_count, steps / report_every) },
        Check { name: "C6 韌體 bad_crc == 橋接送壞的數", pass: bad_crc == corrupted,
            detail: format!("{bad_crc} vs {corrupted}") },
        Check { name: "C7 韌體收到的 cmd == 送出且未壞的數", pass: cmd_frames == sent_cmds - corrupted,
            detail: format!("{cmd_frames} vs {}", sent_cmds - corrupted) },
        // 最後一步注入的訊框要下一個 run_for 才被讀到:lockstep 固有的一步延遲
        Check { name: "C8 韌體收到的編碼器訊框 == steps-1(一步延遲)", pass: enc_frames == steps - 1,
            detail: format!("{enc_frames} vs {}", steps - 1) },
    ];
    let mut all = true;
    for ch in &checks {
        println!("[{}] {} — {}", if ch.pass { "PASS" } else { "FAIL" }, ch.name, ch.detail);
        all &= ch.pass;
    }
    println!("[result] {}", if all { "ALL PASS" } else { "SOME FAIL" });
    std::process::exit(if all { 0 } else { 1 });
}
