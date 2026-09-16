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
mod socketcan;
mod upper;

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
    slip: f64,
    dbg_extra: u32,
    mode: String,
    upper: String,
    can: String,
    enc: String,
    cfg: String,
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
        slip: 0.0,
        dbg_extra: 0,
        mode: "lockstep".into(),
        upper: "script".into(),
        can: "hook".into(),
        enc: "auto".into(),
        cfg: String::new(),
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
            // 受控體的接觸滑移比例:假受控體 0;Isaac 6.0.1 實測轉向 3.1%、直行 0.5%,用 0.05
            "--slip" => a.slip = val.parse().expect("--slip"),
            // g_dbg 第 17 字之後的韌體專屬欄位數(FreeRTOS 版 9 個),跑完印出
            "--dbg-extra" => a.dbg_extra = val.parse().expect("--dbg-extra"),
            // lockstep(預設):橋接推進 Renode;realtime:Renode 自由跑,橋接以牆鐘 dt 取樣/注入
            "--mode" => a.mode = val,
            // 上位:script(預設,內建腳本)或 tcp-listen:ADDR(外部上位連進來講 UART 框包,例如 ROS 2 節點)
            "--upper" => a.upper = val,
            // CAN 走哪條路:hook(預設,每筆注入有 ack)或 socketcan:IFACE(Renode SocketCANBridge + vcan,沒有 ack)
            "--can" => a.can = val,
            // 編碼器注入法:auto(calib tim → hook;can → can)、hook(一筆紀錄,hook 在 Renode 裡打正交脈衝)、
            // gpio(每個邊緣一個 External Control gpio_set)、cnt(External Control 直接寫 TIM CNT)、can(0x181 訊框)
            "--enc" => a.enc = val,
            // 開機後覆蓋韌體的 g_cfg(kp=..,ki=..,accel=..,ff=..;Q8 或 mm/s²),增益掃描與負對照不用重編韌體
            "--cfg" => a.cfg = val,
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
    pub const ENC_L: u64 = 9;
    pub const ENC_R: u64 = 10;
    pub const ENC_FRAMES: u64 = 11;
    pub const CMD_FRAMES: u64 = 12;
    pub const BAD_CRC: u64 = 13;
    pub const FLAGS: u64 = 14;
    pub const INIT_ERR: u64 = 15;
    pub const RX_OVERFLOW: u64 = 16;
    pub const MAGIC_VALUE: u32 = 0x4849_4C31;
    pub const WORDS: u32 = 17;
}

const TIM2_CNT: u64 = 0x4000_0000 + 0x24;
const TIM4_CNT: u64 = 0x4000_0800 + 0x24;
const TIM3_BASE: u64 = 0x4000_0400;
const TIM3_CCR1: u64 = TIM3_BASE + 0x34;
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
    // auto:lockstep 走 hook(正交脈衝經 encoder mode,驗的是模型);realtime 走 cnt——每個邊緣在模型裡是一次
    // LimitTimer.Value 寫入(50–100 µs,同 36 篇 §5.1 的事件成本),8k 邊緣/s 會吃掉模擬執行緒
    let enc_mode = if a.enc == "auto" { if !c.encoder_tim { "can" } else if a.mode == "realtime" { "cnt" } else { "hook" } } else { a.enc.as_str() }.to_string();
    if c.encoder_tim && enc_mode == "can" { eprintln!("calib encoder_source=tim 但 --enc can:韌體不會讀 CAN 編碼器"); }
    if !c.encoder_tim && enc_mode != "can" { eprintln!("calib encoder_source=can 但 --enc {enc_mode}:韌體不會讀 TIM"); }
    let (tim_l, tim_r) = if enc_mode == "gpio" {
        (Some(ec.gpio(m, "sysbus.timer2").expect("timer2")), Some(ec.gpio(m, "sysbus.timer4").expect("timer4")))
    } else { (None, None) };
    // gpio 模式的正交相位(A, B):00 → 10 → 11 → 01,一步一個邊緣
    const QUAD: [(bool, bool); 4] = [(false, false), (true, false), (true, true), (false, true)];
    let mut quad_phase = [0usize; 2];
    let mut enc_prev_ticks = [0i32; 2];
    let mut ticks_before_last = [0i32; 2];
    println!("[effect] encoder: calib={} inject={}", if c.encoder_tim { "tim" } else { "can" }, enc_mode);

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
    println!("[effect] mode={} plant={} dt_ms={} steps={} report_every={} script={:?} negative={} slip={}",
        a.mode, a.plant, c.control_period_ms, steps, report_every, a.script, a.negative, a.slip);
    let realtime = a.mode == "realtime";
    if !realtime && a.mode != "lockstep" {
        eprintln!("--mode 只接受 lockstep 或 realtime");
        std::process::exit(2);
    }
    println!("[effect] tim3 ARR={} (calib pwm_arr={}) track={} circ_um={} tpr={}",
        arr, c.pwm_arr, c.track_mm, c.wheel_circ_um, c.ticks_per_rev);
    if magic != dbg::MAGIC_VALUE {
        eprintln!("g_dbg magic 不對:讀到的不是這支韌體,或位址錯");
        std::process::exit(1);
    }
    // 執行期控制參數 g_cfg { magic, kp_q8, ki_q8, accel_mm_s2, ff_q8 }:開機後(.data 已從 flash 複製)寫入
    let cfg_base = calib::symbol_addr(&a.sym, "g_cfg").expect("符號 g_cfg");
    let cfg_magic = ec.read_u32_at(bus, cfg_base).unwrap();
    if cfg_magic != 0x4849_4C43 {
        eprintln!("g_cfg magic 不對(0x{cfg_magic:08x})");
        std::process::exit(1);
    }
    if !a.cfg.is_empty() {
        for kv in a.cfg.split(',') {
            let (k, v) = kv.split_once('=').expect("--cfg 格式 k=v,k=v");
            let off = match k.trim() { "kp" => 1, "ki" => 2, "accel" => 3, "ff" => 4, "alpha" => 5, other => { eprintln!("--cfg 未知欄位 {other}"); std::process::exit(2); } };
            let v: i32 = v.trim().parse().expect("--cfg 值");
            ec.write_u32_at(bus, cfg_base + 4 * off, v as u32).unwrap();
        }
    }
    let calib_accel = ec.read_u32_at(bus, cfg_base + 4 * 3).unwrap() as i32;
    let calib_alpha = ec.read_u32_at(bus, cfg_base + 4 * 5).unwrap() as i32;
    if a.negative == "no-ramp" {
        // 負對照:韌體的兩個斜坡都關掉,但 C9 仍按 calib 的上限驗 → 必須紅
        ec.write_u32_at(bus, cfg_base + 4 * 3, 0).unwrap();
        ec.write_u32_at(bus, cfg_base + 4 * 5, 0).unwrap();
    }
    let cfgv = ec.read_u32s_at(bus, cfg_base, 6).unwrap();
    println!("[effect] g_cfg@0x{:08x} kp_q8={} ki_q8={} accel_mm_s2={} ff_q8={} alpha_mrad_s2={}{}",
        cfg_base, cfgv[1] as i32, cfgv[2] as i32, cfgv[3] as i32, cfgv[4] as i32, cfgv[5] as i32,
        if a.cfg.is_empty() { " (calib 預設)" } else { " (--cfg 覆蓋後讀回)" });
    let cfg_accel = if a.negative == "no-ramp" { calib_accel } else { cfgv[3] as i32 };
    let cfg_alpha = if a.negative == "no-ramp" { calib_alpha } else { cfgv[5] as i32 };
    let wheel_accel_limit = cfg_accel as f64 + cfg_alpha as f64 * c.track_mm / 2.0 / 1000.0;

    if let Some(dir) = std::path::Path::new(&a.log).parent() {
        let _ = std::fs::create_dir_all(dir);
    }
    let mut log = std::fs::File::create(&a.log).expect("log");
    // realtime 多一欄 wall_ms(每次都不同,lockstep 不放:那邊的 CSV 要能逐 byte 比)
    let wall_col = if realtime { "wall_ms," } else { "" };
    writeln!(log, "step,t_us,{wall_col}cmd_v,cmd_w,sp_l,sp_r,meas_l,meas_r,duty_l_dbg,ccr1,ccr2,dir_l,dir_r,en,flags,\
plant_x,plant_y,plant_th,plant_vl,plant_vr,ticks_l,ticks_r,odom_seq,odom_x,odom_y,odom_th,odom_vl,odom_vr,odom_flags,can_duty_l,can_duty_r").unwrap();

    let mut parser = proto::Parser::default();
    let mut last_odom = proto::Odom::default();
    let mut odom_count = 0u32;
    let mut can_status: Option<(i16, i16, u8, u8)> = None;
    let mut can_status_count = 0u32;
    let mut sent_cmds = 0u32;
    let mut corrupted = 0u32;
    // 外部上位:橋接只當序列線;用同一個 Parser 數它送進來的 cmd_vel 框包(C7)並記最後一個命令
    let mut up: Option<upper::Upper> = match a.upper.strip_prefix("tcp-listen:") {
        Some(addr) => Some(upper::Upper::listen(addr).expect("--upper tcp-listen")),
        None if a.upper == "script" => None,
        None => { eprintln!("--upper 只接受 script 或 tcp-listen:ADDR"); std::process::exit(2); }
    };
    let mut sc: Option<socketcan::SocketCan> = match a.can.strip_prefix("socketcan:") {
        Some(ifn) => {
            let s = socketcan::SocketCan::open(ifn).expect("--can socketcan");
            println!("[effect] can=socketcan({ifn}) 注入與狀態框走 PF_CAN,沒有 ack;C4 沒有事件時刻快照 → 跳過");
            Some(s)
        }
        None if a.can == "hook" => None,
        None => { eprintln!("--can 只接受 hook 或 socketcan:IFACE"); std::process::exit(2); }
    };
    let mut up_parser = proto::Parser::default();
    let mut up_last_cmd = (0i16, 0i16);
    let mut up_any_move = false;
    if let Some(u) = up.as_ref() {
        println!("[effect] upper=tcp-listen({}) script 忽略;等外部上位連線", a.upper);
        let _ = u;
    }
    let mut last_plant = plant::PlantOut::default();
    let mut path_len_mm = 0.0f64;
    let mut max_plant_accel = 0.0f64;
    let mut can_cmp_total = 0u32;
    let mut can_cmp_mismatch = 0u32;
    let wall0 = Instant::now();
    let renode_t_start = t0;
    let mut plant_t_s = 0.0f64;
    let mut max_lag_us: i64 = 0;
    // 每步各段的牆鐘累計(realtime 模式下步長由這些決定,不是由 dt)
    let mut t_ec = Duration::ZERO;
    let mut t_hook = Duration::ZERO;
    let mut t_sleep = Duration::ZERO;
    let mut t_plant = Duration::ZERO;
    if realtime {
        hk.emulation_start().expect("start");
        hk.wait_acks().expect("ack");
    }

    for k in 0..steps {
        // 腳本的時間軸:lockstep 用步數(= Renode 時間);realtime 用牆鐘。橋接落後後會不 sleep 追上,
        // 那段的步比 5 ms 密,若仍用 k·dt 決定命令時刻,命令的持續時間會被壓短(量到轉向段 1.459 s 而不是 1.5 s)
        let t_s = if realtime { wall0.elapsed().as_secs_f64() } else { k as f64 * dt_s };

        // 1. 上位:內建腳本每個回報週期送一次 cmd_vel;外部上位則把這一步之前收到的 byte 全部注入
        if let Some(u) = up.as_mut() {
            let bytes = u.poll_rx();
            if !bytes.is_empty() {
                hk.uart_send(&bytes).expect("uart_send");
                hk.wait_acks().expect("ack");
                let mut fr = Vec::new();
                up_parser.feed(&bytes, &mut fr);
                for (ty, pl) in fr {
                    if ty == proto::MSG_CMD_VEL && pl.len() == 4 {
                        sent_cmds += 1;
                        up_last_cmd = (i16::from_le_bytes([pl[0], pl[1]]), i16::from_le_bytes([pl[2], pl[3]]));
                        if up_last_cmd != (0, 0) { up_any_move = true; }
                    }
                }
            }
        } else if k % report_every == 0 {
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

        // 2. 推進 Renode(lockstep)/ 等到下一個牆鐘刻度(realtime)
        let ph = Instant::now();
        if realtime {
            let target = wall0 + Duration::from_secs_f64((k as f64 + 1.0) * dt_s);
            let now = Instant::now();
            if target > now {
                std::thread::sleep(target - now);
            }
        } else {
            ec.run_for_us(c.control_period_ms * 1000).expect("run_for");
        }
        t_sleep += ph.elapsed();

        // 3. 讀匯流排
        let ph = Instant::now();
        let ccr = ec.read_u32s_at(bus, TIM3_CCR1, 2).unwrap();
        let (ccr1, ccr2) = (ccr[0], ccr[1]);
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
        if realtime {
            let wall_us = wall0.elapsed().as_micros() as i64;
            let lag = wall_us - (t_us - renode_t_start) as i64;
            if lag > max_lag_us { max_lag_us = lag; }
        }

        t_ec += ph.elapsed();

        // 4. 收 MCU 的輸出
        let ph = Instant::now();
        hk.uart_flush_request().unwrap();
        hk.wait_acks().unwrap();
        hk.drain(Duration::from_millis(1)).unwrap();
        let mut frames = Vec::new();
        let mut pending_can: Option<(i16, i16)> = None;
        if let Some(s) = sc.as_mut() {
            for (id, d) in s.drain().unwrap() {
                if id == c.can_id_motor_status && d.len() >= 6 {
                    let dl = i16::from_le_bytes([d[0], d[1]]);
                    let dr = i16::from_le_bytes([d[2], d[3]]);
                    can_status = Some((dl, dr, d[4], d[5]));
                    can_status_count += 1;
                }
            }
        }
        for r in hk.take_inbox() {
            if r.id == hook::ID_UART_FROM_MCU {
                if let Some(u) = up.as_mut() { u.tx(&r.data); }
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

        t_hook += ph.elapsed();

        // 5. 受控體
        let ph = Instant::now();
        let cmd = MotorCmd {
            duty_l: ccr1 as f64 / (arr as f64 + 1.0),
            duty_r: ccr2 as f64 / (arr as f64 + 1.0),
            fwd_l: dir_l,
            fwd_r: dir_r,
            enabled: en,
        };
        let plant_dt = if realtime {
            // 受控體走「真的過了多久」——牆鐘;Renode 若跟不上,三個時鐘就在這裡分開
            let now_s = wall0.elapsed().as_secs_f64();
            let d = now_s - plant_t_s;
            plant_t_s = now_s;
            d
        } else {
            dt_s
        };
        let out = pl.step(k, plant_dt, cmd).expect("plant step");
        if plant_dt > 0.0 {
            let acc = ((out.vl_mm_s - last_plant.vl_mm_s) / plant_dt).abs().max(((out.vr_mm_s - last_plant.vr_mm_s) / plant_dt).abs());
            if acc > max_plant_accel { max_plant_accel = acc; }
        }
        path_len_mm += ((out.x_mm - last_plant.x_mm).powi(2) + (out.y_mm - last_plant.y_mm).powi(2)).sqrt();
        last_plant = out;
        t_plant += ph.elapsed();
        let ph = Instant::now();
        let mut dl = out.ticks_l.wrapping_sub(enc_prev_ticks[0]);
        let mut dr = out.ticks_r.wrapping_sub(enc_prev_ticks[1]);
        if a.negative == "enc-swap" {
            // 負對照:A/B 兩路對調 = 計數方向反過來;韌體會量到負速度,里程計往反方向走 → C3 必須紅
            dl = -dl;
            dr = -dr;
        }
        ticks_before_last = enc_prev_ticks;
        enc_prev_ticks = [out.ticks_l, out.ticks_r];
        match enc_mode.as_str() {
            "hook" => {
                hk.encoder_steps(dl, dr).unwrap();
                hk.wait_acks().unwrap();
            }
            "gpio" => {
                for (w, d) in [(0usize, dl), (1usize, dr)] {
                    let g = if w == 0 { tim_l.unwrap() } else { tim_r.unwrap() };
                    let step: isize = if d > 0 { 1 } else { 3 };
                    for _ in 0..d.unsigned_abs() {
                        let old = QUAD[quad_phase[w]];
                        quad_phase[w] = (quad_phase[w] + step as usize) % 4;
                        let new = QUAD[quad_phase[w]];
                        let ch = if old.0 != new.0 { 0 } else { 1 };
                        ec.gpio_set(g, ch, if ch == 0 { new.0 } else { new.1 }).unwrap();
                    }
                }
            }
            "cnt" => {
                let sgn: i32 = if a.negative == "enc-swap" { -1 } else { 1 };
                ec.write_u32_at(bus, TIM2_CNT, (out.ticks_l.wrapping_mul(sgn)) as u32 & 0xFFFF).unwrap();
                ec.write_u32_at(bus, TIM4_CNT, (out.ticks_r.wrapping_mul(sgn)) as u32 & 0xFFFF).unwrap();
            }
            _ => {
                let mut enc = [0u8; 8];
                enc[..4].copy_from_slice(&out.ticks_l.to_le_bytes());
                enc[4..].copy_from_slice(&out.ticks_r.to_le_bytes());
                if let Some(s) = sc.as_mut() {
                    s.send(c.can_id_encoder, &enc).unwrap();
                } else {
                    hk.can_send(c.can_id_encoder, &enc).unwrap();
                    hk.wait_acks().unwrap();
                }
            }
        }
        t_hook += ph.elapsed();

        // 6. 紀錄
        let (cv, cw) = if up.is_some() { up_last_cmd } else { cmd_at(&script, t_s) };
        let cs = can_status.unwrap_or((0, 0, 0, 0));
        let wall_ms = if realtime { format!("{:.1},", wall0.elapsed().as_secs_f64() * 1000.0) } else { String::new() };
        writeln!(log, "{k},{t_us},{wall_ms}{cv},{cw},{sp_l},{sp_r},{meas_l},{meas_r},{duty_l_dbg},{ccr1},{ccr2},{},{},{},{flags},\
{:.1},{:.1},{:.4},{:.1},{:.1},{},{},{},{},{},{},{},{},{},{},{}",
            dir_l as u8, dir_r as u8, en as u8,
            out.x_mm, out.y_mm, out.th_rad, out.vl_mm_s, out.vr_mm_s, out.ticks_l, out.ticks_r,
            last_odom.seq, last_odom.x_mm, last_odom.y_mm, last_odom.th_mrad, last_odom.vl_mm_s, last_odom.vr_mm_s,
            last_odom.flags, cs.0, cs.1).unwrap();
    }

    if realtime {
        hk.emulation_pause().expect("pause");
        hk.wait_acks().expect("ack");
    }
    let wall = wall0.elapsed();
    let t_end = ec.time_us().unwrap();
    let bad_crc = ec.read_u32_at(bus, dbg_base + 4 * dbg::BAD_CRC).unwrap();
    let cmd_frames = ec.read_u32_at(bus, dbg_base + 4 * dbg::CMD_FRAMES).unwrap();
    let enc_frames = ec.read_u32_at(bus, dbg_base + 4 * dbg::ENC_FRAMES).unwrap();
    let fw_enc_l = ec.read_u32_at(bus, dbg_base + 4 * dbg::ENC_L).unwrap() as i32;
    let fw_enc_r = ec.read_u32_at(bus, dbg_base + 4 * dbg::ENC_R).unwrap() as i32;
    // TIM 模式的兩個等式:CNT 暫存器 == 受控體 tick(mod 2^16,注入沒掉);韌體累計 == 前一步的受控體 tick(一步延遲)
    let (cnt_l, cnt_r) = if enc_mode != "can" {
        (ec.read_u32_at(bus, TIM2_CNT).unwrap(), ec.read_u32_at(bus, TIM4_CNT).unwrap())
    } else { (0, 0) };
    let tim_ok = enc_mode == "can" || (cnt_l == last_plant.ticks_l as u32 & 0xFFFF && cnt_r == last_plant.ticks_r as u32 & 0xFFFF
        && fw_enc_l == ticks_before_last[0] && fw_enc_r == ticks_before_last[1]);
    let ctrl_steps = ec.read_u32_at(bus, dbg_base + 4 * dbg::CTRL_STEPS).unwrap();
    let rx_overflow = ec.read_u32_at(bus, dbg_base + 4 * dbg::RX_OVERFLOW).unwrap();
    let tick_ms = ec.read_u32_at(bus, dbg_base + 4 * dbg::TICK_MS).unwrap();

    println!("[run] steps={} renode_t_us={} wall={:.2}s ({:.1} ms/step, x{:.2} realtime)",
        steps, t_end, wall.as_secs_f64(), wall.as_secs_f64() * 1000.0 / steps as f64,
        (t_end as f64 / 1e6) / wall.as_secs_f64());
    println!("[run] per-step wall: ec_read={:.1} ms hook={:.1} ms plant={:.1} ms {}={:.1} ms",
        t_ec.as_secs_f64() * 1000.0 / steps as f64, t_hook.as_secs_f64() * 1000.0 / steps as f64,
        t_plant.as_secs_f64() * 1000.0 / steps as f64,
        if realtime { "sleep" } else { "run_for" }, t_sleep.as_secs_f64() * 1000.0 / steps as f64);
    if realtime {
        // 三個時鐘:牆鐘(橋接)、Renode 虛擬時間(韌體)、受控體時間(plant_t_s)
        let renode_el = (t_end - renode_t_start) as f64 / 1e6;
        println!("[clocks] wall={:.3}s renode={:.3}s plant={:.3}s  renode/wall={:.3}  max_lag(wall-renode)={:.1} ms",
            wall.as_secs_f64(), renode_el, plant_t_s, renode_el / wall.as_secs_f64(), max_lag_us as f64 / 1000.0);
        let ratio = renode_el / wall.as_secs_f64();
        if ratio < 0.9 {
            // 八項判準驗的是一致性,抓不到這件事:Renode 跑不到實時,韌體的每個 control_period
            // 看到的是 control_period/ratio 牆鐘的編碼器增量,速度迴路會把車壓到 ratio 倍的速度。
            println!("[warn] Renode 只跑到 {:.2}x 實時:韌體每 {} ms 看到的是 {:.1} ms 牆鐘的編碼器增量,閉環速度會低到約 {:.0}%",
                ratio, c.control_period_ms, c.control_period_ms as f64 / ratio, ratio * 100.0);
        }
    }
    println!("[run] fw tick_ms={} ctrl_steps={} cmd_frames={} enc_frames={} bad_crc={} rx_overflow={}",
        tick_ms, ctrl_steps, cmd_frames, enc_frames, bad_crc, rx_overflow);
    println!("[run] odom_frames={} can_status_frames={} sent_cmds={} corrupted={}",
        odom_count, can_status_count, sent_cmds, corrupted);
    if a.dbg_extra > 0 {
        let extra = ec.read_u32s_at(bus, dbg_base + 4 * dbg::WORDS as u64, a.dbg_extra).unwrap();
        println!("[run] fw extra {:?}", extra);
    }
    println!("[run] plant  x={:.1} y={:.1} th={:.4}", last_plant.x_mm, last_plant.y_mm, last_plant.th_rad);
    println!("[run] odom   x={} y={} th={:.4} (seq {})", last_odom.x_mm, last_odom.y_mm,
        last_odom.th_mrad as f64 / 1000.0, last_odom.seq);

    // 驗收
    let expect_time = steps as u64 * c.control_period_ms * 1000;
    let renode_el_us = t_end - t0;
    let expect_odom = if realtime { renode_el_us / (c.report_period_ms * 1000) } else { (steps / report_every) as u64 };
    // 走過的路徑長,不是首尾位移:方形閉環回到原點時位移 ≈ 0,但車確實走了 2.4 m;
    // C3 的容差也該隨路徑長放大(里程計誤差跟著走過的距離累積,不是跟著離起點多遠)
    let dist = path_len_mm;
    // 容差 = 韌體數值誤差(25 mm / 0.03 rad)+ 里程計對真值的系統性差(2% 距離)+ 受控體滑移(--slip)
    let tol_mm = 25.0 + (0.02 + a.slip) * dist;
    let tol_rad = 0.03 + a.slip * last_plant.th_rad.abs();
    let dx = (last_odom.x_mm as f64 - last_plant.x_mm).abs();
    let dy = (last_odom.y_mm as f64 - last_plant.y_mm).abs();
    let dth = (last_odom.th_mrad as f64 / 1000.0 - last_plant.th_rad).abs();
    let expect_move = if up.is_some() { up_any_move } else { script.iter().any(|&(_, v, w)| v != 0 || w != 0) };
    if let Some(s) = sc.as_ref() {
        println!("[run] socketcan: frames sent={} received={}", s.sent, s.received);
    }
    if let Some(u) = up.as_ref() {
        println!("[run] upper: connected_once={} connected_at_end={} cmd_frames_from_upper={} bad_crc_from_upper={}",
            u.connected_once, u.is_connected(), sent_cmds, up_parser.bad_crc);
    }

    let checks = vec![
        // realtime:Renode 跑多快由主機決定,不是驗收項;驗收的是「三個時鐘互相一致」——
        // Renode 時間要有在走,而且不能明顯超前牆鐘。Renode 的實時節拍是以量子為單位追牆鐘,
        // 量到虛擬時間領先牆鐘最多 +1.8%(quantum 1 ms、3 s),所以留 5% + 20 ms。
        Check { name: if realtime { "C1 (realtime) 0 < Renode 時間 ≤ 牆鐘 + 5%" } else { "C1 時間完整性 renode_t == steps*dt" },
            pass: if realtime { renode_el_us > 0 && renode_el_us as f64 <= wall.as_micros() as f64 * 1.05 + 20_000.0 } else { renode_el_us == expect_time },
            detail: format!("{} vs {}", renode_el_us, if realtime { wall.as_micros() as u64 } else { expect_time }) },
        Check { name: "C2 車有動(有非零命令時,路徑長 > 100 mm)", pass: !expect_move || dist > 100.0,
            detail: format!("plant 路徑長 {:.1} mm", dist) },
        Check { name: "C3 韌體 odom 對受控體真值", pass: dx <= tol_mm && dy <= tol_mm && dth <= tol_rad,
            detail: format!("dx={dx:.1} dy={dy:.1} dth={dth:.4} (tol {tol_mm:.1} mm / {tol_rad:.4} rad)") },
        Check { name: if sc.is_some() { "C4 (socketcan) 沒有事件時刻快照,不驗" } else { "C4 兩條獨立管道一致:每筆 CAN 狀態 duty == 同一時刻的 CCR 快照" },
            pass: sc.is_some() || (can_cmp_total > 0 && can_cmp_mismatch == 0),
            detail: if sc.is_some() { format!("狀態框 {} 筆(經 vcan)", can_status_count) } else { format!("{} 筆比對,{} 筆不符", can_cmp_total, can_cmp_mismatch) } },
        // odom 是韌體按「它的」時間每 report_period 送一次,期望值用 Renode 時間算,不用牆鐘
        Check { name: "C5 odom 回報數 ≥ 90% 期望(按 Renode 時間)", pass: odom_count as f64 >= 0.9 * expect_odom as f64,
            detail: format!("{} / {}", odom_count, expect_odom) },
        // C9:斜坡生效 → 受控體的輪加速度不超過上限 × 1.2(斜坡限的是設定點,PI 追斜坡的瞬態量到 +7%;
        // 斜坡關掉時受控體撞到馬達層的 3000 上限,1.2 × 2100 = 2520 分得開);accel=0 時不驗
        // 輪加速度上限 = 線加速度 + 角加速度 × 輪距/2(v、w 同時起坡時兩者相加)
        Check { name: if cfg_accel > 0 { "C9 受控體輪加速度 ≤ (accel + alpha·track/2) × 1.2" } else { "C9 (斜坡關,accel=0) 不驗" },
            pass: cfg_accel <= 0 || max_plant_accel <= wheel_accel_limit * 1.2,
            detail: format!("max |dv/dt| = {:.0} mm/s² vs {:.0}(輪上限 {:.0} × 1.2)", max_plant_accel, wheel_accel_limit * 1.2, wheel_accel_limit) },
        Check { name: "C6 韌體 bad_crc == 橋接送壞的數", pass: bad_crc == corrupted,
            detail: format!("{bad_crc} vs {corrupted}") },
        Check { name: "C7 韌體收到的 cmd == 送出且未壞的數", pass: cmd_frames == sent_cmds - corrupted,
            detail: format!("{cmd_frames} vs {}", sent_cmds - corrupted) },
        // 最後一步注入的訊框要下一個 run_for 才被讀到:lockstep 固有的一步延遲
        Check { name: if enc_mode != "can" { if realtime { "C8 (TIM, realtime) TIM CNT == 受控體 tick mod 2^16" } else { "C8 (TIM) TIM CNT == 受控體 tick;韌體累計 == 前一步 tick(一步延遲)" } } else if realtime { "C8 (realtime) 韌體收到的編碼器訊框 ≥ 90% steps" } else { "C8 韌體收到的編碼器訊框 == steps-1(一步延遲)" },
            pass: if enc_mode != "can" { if realtime { cnt_l == last_plant.ticks_l as u32 & 0xFFFF && cnt_r == last_plant.ticks_r as u32 & 0xFFFF } else { tim_ok } } else if realtime { enc_frames as f64 >= 0.9 * steps as f64 } else { enc_frames == steps - 1 },
            detail: if enc_mode != "can" { format!("CNT {cnt_l}/{cnt_r} vs plant {}/{};fw {fw_enc_l}/{fw_enc_r} vs 前一步 {}/{}", last_plant.ticks_l as u32 & 0xFFFF, last_plant.ticks_r as u32 & 0xFFFF, ticks_before_last[0], ticks_before_last[1]) } else { format!("{enc_frames} vs {}", if realtime { steps } else { steps - 1 }) } },
    ];
    let mut all = true;
    for ch in &checks {
        println!("[{}] {} — {}", if ch.pass { "PASS" } else { "FAIL" }, ch.name, ch.detail);
        all &= ch.pass;
    }
    println!("[result] {}", if all { "ALL PASS" } else { "SOME FAIL" });
    std::process::exit(if all { 0 } else { 1 });
}
