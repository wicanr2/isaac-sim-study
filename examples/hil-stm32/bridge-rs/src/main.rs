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
mod world;

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
    /// --enc cont 的誤差攤還時間(ms):錨點更新時的追蹤誤差在這段虛擬時間內線性補上
    enc_tau_ms: f64,
    cfg: String,
    /// 故障注入:none | hang | drv-fault | bumper | stall | no-ping(在 --fault-at 秒發生)
    fault: String,
    fault_at: f64,
    /// world.json:假雷射與碰撞的世界(空 = 沒有,受控體不掃描、不碰撞)
    world: String,
    /// 假雷射的出口:TCP 監聽,每筆掃描一行 `SCAN <seq> <n> r...`,上位(ROS driver)連進來
    scan_listen: String,
    /// C11:上位(Nav2)應該把車開到 world.goal——只有這個旗標才驗
    expect_goal: bool,
    /// lockstep 專用的時鐘偏斜實驗(35 篇 §5.1 第 4 點):none | uniform:R(每步 Renode 只推進 R·dt,受控體照 dt 走——
    /// 「均勻的慢」,比值 R 但沒有停頓)| stall:N:M(每 N 步一次把 Renode 一口氣推進 M·dt、期間編碼器不更新,
    /// 受控體那一步走 M·dt——「停頓」,比值 ≈ 1)。兩個都是決定性的,不靠主機負載。
    skew: String,
    /// 假雷射存檔(給 tools/topview.py):每筆一行 `SCAN <step> <n> r...`,與送上位的內容相同(blind-scan 也照改過的存);空 = 不存
    scan_log: String,
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
        // 開機 run_for 的長度,也決定之後每個 5 ms 步邊界落在韌體時間的哪一相位。**不能是 5 的倍數**:
        // 韌體的控制步在 SysTick 的 5k ms 上跑,邊界若也落在 5k ms,橋接的暫停會切在控制步中間——
        // CNT 注入與韌體讀 CNT、CCR 寫與 g_dbg 寫,哪個在邊界前後由指令數決定,改幾行碼 CSV 就變
        // (量到:原版 +200 圈 NOP 8639 個欄位不同、C9 紅;邊界錯開 2 ms 後三個版本逐 byte 相同)
        boot_ms: 102,
        slip: 0.0,
        dbg_extra: 0,
        mode: "lockstep".into(),
        upper: "script".into(),
        can: "hook".into(),
        enc: "auto".into(),
        enc_tau_ms: 20.0,
        cfg: String::new(),
        fault: "none".into(),
        fault_at: 2.0,
        world: String::new(),
        scan_listen: String::new(),
        expect_goal: false,
        skew: "none".into(),
        scan_log: String::new(),
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
            "--fault" => a.fault = val,
            "--fault-at" => a.fault_at = val.parse().expect("--fault-at"),
            "--world" => a.world = val,
            "--scan-listen" => a.scan_listen = val,
            "--expect-goal" => a.expect_goal = val == "1",
            "--skew" => a.skew = val,
            "--scan-log" => a.scan_log = val,
            // lockstep(預設):橋接推進 Renode;realtime:Renode 自由跑,橋接以牆鐘 dt 取樣/注入
            "--mode" => a.mode = val,
            // 上位:script(預設,內建腳本)或 tcp-listen:ADDR(外部上位連進來講 UART 框包,例如 ROS 2 節點)
            "--upper" => a.upper = val,
            // CAN 走哪條路:hook(預設,每筆注入有 ack)或 socketcan:IFACE(Renode SocketCANBridge + vcan,沒有 ack)
            "--can" => a.can = val,
            // 編碼器注入法:auto(calib tim → hook;can → can)、hook(一筆紀錄,hook 在 Renode 裡打正交脈衝)、
            // gpio(每個邊緣一個 External Control gpio_set)、cnt(External Control 直接寫 TIM CNT)、can(0x181 訊框)
            "--enc" => a.enc = val,
            // cont:CNT 在韌體讀的當下由「受控體 tick + 輪速 × 經過的虛擬時間」算(renode/hil_quadrature.cs ContinuousEncoder)
            "--enc-tau-ms" => a.enc_tau_ms = val.parse().expect("--enc-tau-ms"),
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
    pub const RESETS: u64 = 17;
    pub const BOOT_CSR: u64 = 18;
    pub const PING_FRAMES: u64 = 19;
    pub const MAGIC_VALUE: u32 = 0x4849_4C31;
    pub const WORDS: u32 = 20;
}

/// odom / CAN 狀態框 / g_dbg.flags 的位元(firmware/proto.h)
mod flag {
    pub const ENABLED: u32 = 1 << 0;
    pub const ESTOP: u32 = 1 << 1;
    pub const CMD_STALE: u32 = 1 << 2;
    pub const DRV_FAULT: u32 = 1 << 3;
    pub const BUMPER: u32 = 1 << 4;
    pub const STALL: u32 = 1 << 5;
    pub const HB_LOST: u32 = 1 << 6;
    pub const WDT_RESET: u32 = 1 << 7;
}
/// g_cfg.safety_mask 的位元
mod safety {
    pub const IWDG: u32 = 1 << 0;
    pub const DRV_FAULT: u32 = 1 << 1;
    pub const BUMPER: u32 = 1 << 2;
    pub const STALL: u32 = 1 << 3;
    pub const HB: u32 = 1 << 4;
}
const DRV_FAULT_L_PIN: i32 = 14;
const DRV_FAULT_R_PIN: i32 = 15;
const BUMPER_PIN: i32 = 0;
/// bumper 故障注入:受控體 x 超過這面「牆」就把 PC0 拉低(常閉接點斷開)
const BUMPER_WALL_MM: f64 = 500.0;

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
    println!("[effect] encoder: calib={} inject={}{}", if c.encoder_tim { "tim" } else { "can" }, enc_mode,
        if enc_mode == "cont" { format!(" tau_ms={}", a.enc_tau_ms) } else { String::new() });
    if enc_mode == "cont" {
        hk.enc_cont_install((a.enc_tau_ms * 1000.0).round() as i32).expect("enc_cont_install");
        hk.wait_acks().expect("ack");
    }
    // cont 模式的追蹤誤差:每步邊界讀到的 CNT − 同一時刻的受控體 tick(步內外插準不準)
    let mut cont_cnt = [0u32; 2];
    let mut cont_max_err = 0i32;
    let mut cont_last_err = [0i32; 2];
    let mut max_step_ticks = 0i32;
    // 受控體輪速 mm/s → milli-tick/s
    let rate_milli = |v_mm_s: f64| (v_mm_s * c.ticks_per_rev as f64 * 1.0e6 / c.wheel_circ_um as f64).round() as i32;

    let world = if a.world.is_empty() { None } else { Some(world::World::load(&a.world).expect("world.json")) };
    if let Some(w) = &world {
        println!("[effect] world={} room=[{},{}]x[{},{}] boxes={} robot_r={} laser={}x{}m@{}ms goal=({},{},{})",
            a.world, w.x_min, w.x_max, w.y_min, w.y_max, w.boxes.len(), w.robot_radius, w.beams, w.range_max, w.period_ms, w.goal.0, w.goal.1, w.goal.2);
    }
    let mut scan_up = if a.scan_listen.is_empty() { None } else { Some(upper::Upper::listen(&a.scan_listen).expect("--scan-listen")) };
    let mut pl: Box<dyn Plant> = if a.plant == "fake" {
        match &world { Some(w) => Box::new(plant::Fake::new(c).with_world(w.clone())), None => Box::new(plant::Fake::new(c)) }
    } else if let Some(addr) = a.plant.strip_prefix("udp:") {
        Box::new(plant::Udp::connect(addr).expect("UDP plant"))
    } else if let Some(addr) = a.plant.strip_prefix("tcp:") {
        Box::new(plant::Tcp::connect(addr).expect("TCP plant"))
    } else {
        eprintln!("--plant 只接受 fake、udp:host:port 或 tcp:host:port");
        std::process::exit(2);
    };

    // 故障注入與負對照的組合:--negative X-off = 注入 X 的故障 + 關掉韌體對 X 的防護(g_cfg.safety_mask)
    let neg = a.negative.as_str();
    let fault: String = match neg {
        "iwdg-off" => "hang".into(), "drv-fault-off" => "drv-fault".into(), "bumper-off" => "bumper".into(),
        "stall-off" => "stall".into(), "hb-off" => "no-ping".into(), "noinit-off" => "hang".into(), _ => a.fault.clone(),
    };
    if neg == "no-latch" && !a.upper.starts_with("tcp-listen:") { eprintln!("--negative no-latch 是上位側的負對照,要配 --upper tcp-listen(UPPER=nav2)"); }
    let mask_clear: u32 = match neg {
        "iwdg-off" => safety::IWDG, "drv-fault-off" => safety::DRV_FAULT, "bumper-off" => safety::BUMPER,
        "stall-off" => safety::STALL, "hb-off" => safety::HB, _ => 0,
    };
    if !["none", "hang", "drv-fault", "bumper", "stall", "no-ping"].contains(&fault.as_str()) {
        eprintln!("--fault 只接受 none|hang|drv-fault|bumper|stall|no-ping");
        std::process::exit(2);
    }
    // bumper 場景要有「撞牆後倒車」:前進撞 500 mm 的牆 → 拒絕前進 → 倒車命令要被接受
    let script = if fault == "bumper" && a.script == "0:0,0;0.5:300,0;3.5:0,600;5:0,0" {
        parse_script("0:0,0;0.5:300,0;3.5:-200,0;5:0,0")
    } else { script };
    let fault_at_ms = (a.fault_at * 1000.0).round() as u32;
    // 負對照 noinit-off:拿掉韌體判斷暖重置的第二條證據——每步把 .noinit 的 magic 清掉,韌體開機時只剩 RCC_CSR.IWDGRSTF。
    // 不走 g_cfg:g_cfg 的改動寫在 flash,IWDG 重啟時 `macro reset` 重跑 LoadELF 會把它蓋回預設(量到的,38 篇 §1.2)
    let noinit_magic_addr = if neg == "noinit-off" { Some(calib::symbol_addr(&a.sym, "g_noinit").expect("符號 g_noinit")) } else { None };

    // 低有效的輸入腳(PC14/PC15 驅動器故障、PC0 保險桿)在 Renode 的預設是 0 = 觸發;
    // 真板有 pull-up,這裡由橋接在開機前拉高,等於接上 pull-up
    let gpio_c = ec.gpio(m, "sysbus.gpioPortC").or_else(|_| ec.gpio(m, "gpioPortC")).expect("gpioPortC");
    for pin in [DRV_FAULT_L_PIN, DRV_FAULT_R_PIN, BUMPER_PIN] { ec.gpio_set(gpio_c, pin, true).unwrap(); }

    // 執行期控制參數 g_cfg { magic, kp, ki, accel, ff, alpha, iwdg_ms, hang_at_ms, hb_timeout_ms, stall_duty, stall_ms, safety_mask }
    // 在「開機前」寫進 flash 裡 .data 的初始值(LMA = _sidata + (g_cfg − _sdata)),startup 照常複製到 SRAM——
    // 等於燒錄前改了參數區。IWDG 這種 init 就定案、之後改不了的參數也因此改得到。
    let cfg_base = calib::symbol_addr(&a.sym, "g_cfg").expect("符號 g_cfg");
    let cfg_lma = calib::symbol_addr(&a.sym, "_sidata").expect("符號 _sidata") + (cfg_base - calib::symbol_addr(&a.sym, "_sdata").expect("符號 _sdata"));
    let cfg_flash_magic = ec.read_u32_at(bus, cfg_lma).unwrap();
    if cfg_flash_magic != 0x4849_4C43 {
        eprintln!("flash 裡的 g_cfg 初始值 magic 不對(0x{cfg_flash_magic:08x} @0x{cfg_lma:08x})");
        std::process::exit(1);
    }
    let calib_accel = ec.read_u32_at(bus, cfg_lma + 4 * 3).unwrap() as i32;
    let calib_alpha = ec.read_u32_at(bus, cfg_lma + 4 * 5).unwrap() as i32;
    let calib_mask = ec.read_u32_at(bus, cfg_lma + 4 * 11).unwrap();
    let mut patches: Vec<(u64, u32)> = Vec::new();
    if !a.cfg.is_empty() {
        for kv in a.cfg.split(',') {
            let (k, v) = kv.split_once('=').expect("--cfg 格式 k=v,k=v");
            let off = match k.trim() { "kp" => 1, "ki" => 2, "accel" => 3, "ff" => 4, "alpha" => 5, "iwdg" => 6, "hang" => 7, "hb" => 8,
                "stall_duty" => 9, "stall_ms" => 10, "mask" => 11, other => { eprintln!("--cfg 未知欄位 {other}"); std::process::exit(2); } };
            let v: i32 = v.trim().parse().expect("--cfg 值");
            patches.push((off, v as u32));
        }
    }
    if neg == "no-ramp" {
        // 負對照:韌體的兩個斜坡都關掉,但 C9 仍按 calib 的上限驗 → 必須紅
        patches.push((3, 0)); patches.push((5, 0));
    }
    // hang 的時刻寫的是韌體的 tick:韌體在 Renode t=0 開機,橋接的 t=0 是開機 boot_ms 之後
    if fault == "hang" { patches.push((7, fault_at_ms + a.boot_ms as u32)); }
    if mask_clear != 0 { patches.push((11, calib_mask & !mask_clear)); }
    for (off, v) in &patches { ec.write_u32_at(bus, cfg_lma + 4 * off, *v).unwrap(); }

    // 讓韌體開機:LoadELF 之後機器是暫停的,main 還沒跑,g_dbg 全零
    ec.run_for_us(a.boot_ms * 1000).expect("boot run_for");

    // 生效證明:每個變數都印一行,證明它進了系統
    let magic = ec.read_u32_at(bus, dbg_base + 4 * dbg::MAGIC).expect("讀 magic");
    let arr = ec.read_u32_at(bus, TIM3_ARR).expect("讀 ARR");
    let t0 = ec.time_us().expect("time");
    println!("[effect] renode ec={} hook={} machine={} boot_ms={} t0_us={}", a.ec, a.hook, a.machine, a.boot_ms, t0);
    println!("[effect] g_dbg@0x{:08x} magic=0x{:08x} ({}) init_err={} resets={} boot_csr=0x{:08x}",
        dbg_base, magic, if magic == dbg::MAGIC_VALUE { "ok" } else { "MISMATCH" },
        ec.read_u32_at(bus, dbg_base + 4 * dbg::INIT_ERR).unwrap(),
        ec.read_u32_at(bus, dbg_base + 4 * dbg::RESETS).unwrap(), ec.read_u32_at(bus, dbg_base + 4 * dbg::BOOT_CSR).unwrap());
    println!("[effect] mode={} plant={} dt_ms={} steps={} report_every={} script={:?} negative={} fault={}@{:.1}s slip={}",
        a.mode, a.plant, c.control_period_ms, steps, report_every, a.script, a.negative, fault, a.fault_at, a.slip);
    let realtime = a.mode == "realtime";
    if !realtime && a.mode != "lockstep" {
        eprintln!("--mode 只接受 lockstep 或 realtime");
        std::process::exit(2);
    }
    // 時鐘偏斜實驗(lockstep 專用):uniform:R → 每步 Renode 推進 R·dt;stall:N:M → 每 N 步一次推進 M·dt
    let (skew_scale, skew_every, skew_mult): (f64, u32, u32) = match a.skew.as_str() {
        "none" => (1.0, 0, 1),
        s if s.starts_with("uniform:") => (s[8..].parse().expect("--skew uniform:R"), 0, 1),
        s if s.starts_with("stall:") => { let v: Vec<&str> = s[6..].split(':').collect(); (1.0, v[0].parse().expect("--skew stall:N:M"), v[1].parse().expect("--skew stall:N:M")) }
        _ => { eprintln!("--skew 只接受 none | uniform:R | stall:N:M"); std::process::exit(2); }
    };
    if a.skew != "none" {
        if realtime { eprintln!("--skew 只在 lockstep 有意義"); std::process::exit(2); }
        println!("[effect] skew={}:{}", a.skew, if skew_every > 0 { format!(" 每 {} 步一次 Renode 一口氣推進 {}×dt、編碼器凍結、受控體那步走 {}×dt(停頓,比值 ≈ 1)", skew_every, skew_mult, skew_mult) } else { format!(" 每步 Renode 只推進 {}×dt、受控體走 dt(均勻的慢,比值 {})", skew_scale, skew_scale) });
    }
    println!("[effect] tim3 ARR={} (calib pwm_arr={}) track={} circ_um={} tpr={}",
        arr, c.pwm_arr, c.track_mm, c.wheel_circ_um, c.ticks_per_rev);
    if magic != dbg::MAGIC_VALUE {
        eprintln!("g_dbg magic 不對:讀到的不是這支韌體,或位址錯");
        std::process::exit(1);
    }
    // 開機後從 SRAM 讀回:證明 startup 複製的是改過的那份
    let cfgv = ec.read_u32s_at(bus, cfg_base, 12).unwrap();
    if cfgv[0] != 0x4849_4C43 {
        eprintln!("g_cfg magic 不對(0x{:08x})", cfgv[0]);
        std::process::exit(1);
    }
    println!("[effect] g_cfg@0x{:08x} (flash LMA 0x{:08x}) kp_q8={} ki_q8={} accel_mm_s2={} ff_q8={} alpha_mrad_s2={}{}",
        cfg_base, cfg_lma, cfgv[1] as i32, cfgv[2] as i32, cfgv[3] as i32, cfgv[4] as i32, cfgv[5] as i32,
        if patches.is_empty() { " (calib 預設)" } else { " (開機前改 flash,開機後讀回)" });
    println!("[effect] g_cfg safety: iwdg_ms={} hang_at_ms={} hb_timeout_ms={} stall_duty={} stall_ms={} mask=0x{:02x}{}",
        cfgv[6] as i32, cfgv[7] as i32, cfgv[8] as i32, cfgv[9] as i32, cfgv[10] as i32, cfgv[11],
        if mask_clear != 0 { format!(" (負對照關掉 0x{:02x})", mask_clear) } else { String::new() });
    let cfg_accel = if neg == "no-ramp" { calib_accel } else { cfgv[3] as i32 };
    let cfg_alpha = if neg == "no-ramp" { calib_alpha } else { cfgv[5] as i32 };
    let wheel_accel_limit = cfg_accel as f64 + cfg_alpha as f64 * c.track_mm / 2.0 / 1000.0;

    if let Some(dir) = std::path::Path::new(&a.log).parent() {
        let _ = std::fs::create_dir_all(dir);
    }
    let mut log = std::fs::File::create(&a.log).expect("log");
    let mut scan_log = if a.scan_log.is_empty() { None } else { Some(std::fs::File::create(&a.scan_log).expect("--scan-log")) };
    // realtime 多一欄 wall_ms(每次都不同,lockstep 不放:那邊的 CSV 要能逐 byte 比)
    let wall_col = if realtime { "wall_ms," } else { "" };
    writeln!(log, "step,t_us,{wall_col}cmd_v,cmd_w,sp_l,sp_r,meas_l,meas_r,duty_l_dbg,ccr1,ccr2,dir_l,dir_r,en,flags,\
plant_x,plant_y,plant_th,plant_vl,plant_vr,ticks_l,ticks_r,odom_seq,odom_x,odom_y,odom_th,odom_vl,odom_vr,odom_flags,can_duty_l,can_duty_r,collided").unwrap();

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
    let mut up_pending: Vec<u8> = Vec::new();
    let uart_budget = (c.uart_baud as f64 / 10.0 * dt_s).floor().max(1.0) as usize;   // 8N1:每 byte 10 bit
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
    let mut renode_advanced_us: u64 = 0;   // lockstep:實際 run_for 的總和(--skew 時 ≠ steps·dt)
    let mut skew_stalls = 0u32;
    let mut plant_dt_mult = 1.0f64;
    let mut plant_t_s = 0.0f64;
    let mut max_lag_us: i64 = 0;
    // 每步各段的牆鐘累計(realtime 模式下步長由這些決定,不是由 dt)
    let mut t_ec = Duration::ZERO;
    let mut t_hook = Duration::ZERO;
    let mut t_upper = Duration::ZERO;           // 第 1 段:上位框包注入(realtime 下的停頓常在這裡,所以另計)
    let (mut m_upper, mut m_ec, mut m_hook, mut m_plant) = (Duration::ZERO, Duration::ZERO, Duration::ZERO, Duration::ZERO);   // 單步最大
    let mut t_sleep = Duration::ZERO;
    let mut t_plant = Duration::ZERO;
    // 安全 I/O 驗收(C10)用的觀測:第一次看到各旗標的步、韌體重啟(tick 倒退)的步、故障期間的 CCR/EN
    let ping_every = (c.heartbeat_period_ms / c.control_period_ms).max(1) as u32;
    let mut first_flag: [Option<u32>; 8] = [None; 8];
    let mut prev_tick: u32 = 0;
    let mut reset_step: Option<u32> = None;
    let mut ccr_after_reset: Option<u32> = None;
    let mut enabled_after_reset: Option<u32> = None;
    let mut hang_step: Option<u32> = None;     // ctrl_steps 停止增加的第一步(韌體真的死了的時刻)
    let mut prev_ctrl_steps: u32 = 0;
    let mut ccr_while_hung_max: u32 = 0;
    let mut drv_low = false;
    let mut bumper_low = false;
    let mut drv_violations = 0u32;     // 故障腳拉低期間 CCR ≠ 0 或 EN = 1 的步數
    let mut drv_recovered = false;     // 放開後 EN 回到 1
    let mut stall_violations = 0u32;   // STALL 旗標出現後、命令歸零前 CCR ≠ 0 的步數
    let mut stop_step: Option<u32> = None;  // 故障注入後受控體兩輪 |v| < 5 的第一步
    let mut x_max = f64::MIN;
    let fault_step = (a.fault_at / dt_s).round() as u32;
    let fault_end_step = ((a.fault_at + 1.5) / dt_s).round() as u32;   // drv-fault / stall 的注入持續 1.5 s
    let mut last_flags: u32 = 0;
    let mut collisions = 0u32;
    let mut moved_after_reset = 0u32;      // C12:重啟 0.5 s 後受控體還在動的步數
    let mut dist_after_reset = 0.0f64;
    let mut pos_at_reset: Option<(f64, f64)> = None;
    let mut first_collision: Option<u32> = None;
    let mut scans_sent = 0u32;
    if realtime {
        hk.emulation_start().expect("start");
        hk.wait_acks().expect("ack");
    }

    for k in 0..steps {
        // 腳本的時間軸:lockstep 用步數(= Renode 時間);realtime 用牆鐘。橋接落後後會不 sleep 追上,
        // 那段的步比 5 ms 密,若仍用 k·dt 決定命令時刻,命令的持續時間會被壓短(量到轉向段 1.459 s 而不是 1.5 s)
        let t_s = if realtime { wall0.elapsed().as_secs_f64() } else { k as f64 * dt_s };
        let in_fault_window = t_s >= a.fault_at && t_s < a.fault_at + 1.5;

        if let Some(addr) = noinit_magic_addr { ec.write_u32_at(bus, addr, 0).unwrap(); }
        // 0. 故障注入(腳位):驅動器故障腳在視窗內拉低;保險桿在受控體撞到牆時斷開(低)
        if fault == "drv-fault" && in_fault_window != drv_low {
            drv_low = in_fault_window;
            ec.gpio_set(gpio_c, DRV_FAULT_L_PIN, !drv_low).unwrap();
            println!("[fault] t={:.3}s PC14 驅動器故障腳 → {}", t_s, if drv_low { "低(故障)" } else { "高(解除)" });
        }
        if fault == "bumper" {
            let hit = last_plant.x_mm >= BUMPER_WALL_MM;
            if hit != bumper_low {
                bumper_low = hit;
                ec.gpio_set(gpio_c, BUMPER_PIN, !hit).unwrap();
                println!("[fault] t={:.3}s x={:.1} mm {} 牆 {:.0} mm → PC0 {}", t_s, last_plant.x_mm, if hit { "撞到" } else { "離開" }, BUMPER_WALL_MM, if hit { "低(斷開)" } else { "高" });
            }
        }

        // 1. 上位:內建腳本每個回報週期送一次 cmd_vel、每 heartbeat_period 送一次 PING;外部上位則把這一步之前收到的 byte 全部注入
        let ph = Instant::now();
        if up.is_none() && k % ping_every == 0 && !(fault == "no-ping" && t_s >= a.fault_at) {
            hk.uart_send(&proto::ping()).expect("uart_send");
            hk.wait_acks().expect("ack");
        }
        if let Some(u) = up.as_mut() {
            // 序列線有線速:115200 bps = 每 5 ms 最多 57.6 byte。上位在牆鐘上送、橋接在 Renode 時間上注入,
            // 橋接一停頓上位的 byte 就堆起來;一次全灌進 USART1 的話 Renode 的 UART 不分 baud 節拍、ISR 一個接一個,
            // 主迴圈搶不到 ring buffer(128 B)就溢位(量到 rx_overflow=114、4 個壞 CRC)。真線路不會這樣,所以按線速分批
            up_pending.extend_from_slice(&u.poll_rx());
            let n = up_pending.len().min(uart_budget);
            if n > 0 {
                let bytes: Vec<u8> = up_pending.drain(..n).collect();
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
        { let d = ph.elapsed(); t_upper += d; if d > m_upper { m_upper = d; } }

        // 2. 推進 Renode(lockstep)/ 等到下一個牆鐘刻度(realtime)
        let ph = Instant::now();
        if realtime {
            let target = wall0 + Duration::from_secs_f64((k as f64 + 1.0) * dt_s);
            let now = Instant::now();
            if target > now {
                std::thread::sleep(target - now);
            }
        } else {
            let is_stall = skew_every > 0 && k > 0 && k % skew_every == 0;
            let mult = if is_stall { skew_mult as f64 } else { skew_scale };
            let us = (c.control_period_ms as f64 * 1000.0 * mult).round() as u64;
            ec.run_for_us(us).expect("run_for");
            renode_advanced_us += us;
            if is_stall { skew_stalls += 1; }
            plant_dt_mult = if is_stall { skew_mult as f64 } else { 1.0 };
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
        let tick = d[dbg::TICK_MS as usize];
        // 只記故障注入之後第一次出現的旗標(開機那一步 CMD_STALE/HB_LOST 本來就亮;重啟後輸入腳回 0 也會亮一步)
        if k >= fault_step { for b in 0..8 { if flags & (1 << b) != 0 && first_flag[b].is_none() { first_flag[b] = Some(k); } } }
        if (flags ^ last_flags) & !flag::ENABLED != 0 {
            println!("[flags] t={:.3}s 0x{:02x} → 0x{:02x}{}{}{}{}{}{}{}", t_s, last_flags, flags,
                if flags & flag::ESTOP != 0 { " ESTOP" } else { "" }, if flags & flag::CMD_STALE != 0 { " CMD_STALE" } else { "" },
                if flags & flag::DRV_FAULT != 0 { " DRV_FAULT" } else { "" }, if flags & flag::BUMPER != 0 { " BUMPER" } else { "" },
                if flags & flag::STALL != 0 { " STALL" } else { "" }, if flags & flag::HB_LOST != 0 { " HB_LOST" } else { "" },
                if flags & flag::WDT_RESET != 0 { " WDT_RESET" } else { "" });
        }
        last_flags = flags;
        if k > 0 && tick < prev_tick && reset_step.is_none() {
            reset_step = Some(k);
            println!("[fault] t={:.3}s 韌體 tick {} → {}:重啟了(IWDG)", t_s, prev_tick, tick);
            // 機器重置把 GPIO 埠也重置了,輸入腳回到 0 = 低有效的三腳全部「觸發」;真板的 pull-up 在板子上,
            // 這裡橋接就是板子——重新拉高
            for pin in [DRV_FAULT_L_PIN, DRV_FAULT_R_PIN, BUMPER_PIN] { ec.gpio_set(gpio_c, pin, true).unwrap(); }
        }
        prev_tick = tick;
        let ctrl_now = d[dbg::CTRL_STEPS as usize];
        if fault == "hang" && k >= fault_step && hang_step.is_none() && reset_step.is_none() && ctrl_now == prev_ctrl_steps && ctrl_now > 0 {
            hang_step = Some(k - 1);
            println!("[fault] t={:.3}s 韌體 ctrl_steps 停在 {}:死了", (k - 1) as f64 * dt_s, ctrl_now);
        }
        prev_ctrl_steps = ctrl_now;
        if fault == "hang" && t_s >= a.fault_at {
            // 重啟那一步讀到的 CCR:週邊被重置、pwm_init 寫 0;下一步上位的 cmd_vel 又進來,車就又走了
            if reset_step.is_none() { ccr_while_hung_max = ccr_while_hung_max.max(ccr1); }
            else if ccr_after_reset.is_none() { ccr_after_reset = Some(ccr1); }
            if reset_step.is_some() && enabled_after_reset.is_none() && flags & flag::ENABLED != 0 { enabled_after_reset = Some(k); }
        }
        if fault == "drv-fault" {
            if drv_low && k > fault_step && (ccr1 != 0 || ccr2 != 0 || en) { drv_violations += 1; }
            if !drv_low && k > fault_end_step + 20 && en { drv_recovered = true; }
        }
        if fault == "stall" {
            if let Some(f0) = first_flag[5] { if k > f0 && t_s < 5.0 && (ccr1 != 0 || ccr2 != 0) { stall_violations += 1; } }
        }
        if enc_mode == "cont" {
            cont_cnt = [ec.read_u32_at(bus, TIM2_CNT).unwrap(), ec.read_u32_at(bus, TIM4_CNT).unwrap()];
        }
        let t_us = ec.time_us().unwrap();
        if realtime {
            let wall_us = wall0.elapsed().as_micros() as i64;
            let lag = wall_us - (t_us - renode_t_start) as i64;
            if lag > max_lag_us { max_lag_us = lag; }
        }

        { let d = ph.elapsed(); t_ec += d; if d > m_ec { m_ec = d; } }

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

        { let d = ph.elapsed(); t_hook += d; if d > m_hook { m_hook = d; } }

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
            dt_s * plant_dt_mult
        };
        // 堵轉注入:輪子被卡住——受控體不動、編碼器不動,不管韌體給多少 duty
        let cmd = if fault == "stall" && in_fault_window { MotorCmd { duty_l: 0.0, duty_r: 0.0, fwd_l: true, fwd_r: true, enabled: false } } else { cmd };
        let out = pl.step(k, plant_dt, cmd).expect("plant step");
        if fault != "none" && t_s >= a.fault_at && stop_step.is_none() && out.vl_mm_s.abs() < 5.0 && out.vr_mm_s.abs() < 5.0 { stop_step = Some(k); }
        if out.x_mm > x_max { x_max = out.x_mm; }
        if let Some(rs) = reset_step {
            if pos_at_reset.is_none() { pos_at_reset = Some((out.x_mm, out.y_mm)); }
            if k >= rs + (0.5 / dt_s) as u32 {
                if out.vl_mm_s.abs() >= 5.0 || out.vr_mm_s.abs() >= 5.0 { moved_after_reset += 1; }
                let (x0, y0) = pos_at_reset.unwrap();
                dist_after_reset = dist_after_reset.max(((out.x_mm - x0).powi(2) + (out.y_mm - y0).powi(2)).sqrt());
            }
        }
        if out.collided {
            collisions += 1;
            if first_collision.is_none() { first_collision = Some(k); println!("[world] t={:.3}s 受控體撞到牆或方塊 @({:.0}, {:.0}) mm", t_s, out.x_mm, out.y_mm); }
        }
        // 假雷射:受控體給一筆就原樣轉給上位(語意在上位解;橋接只加行首與 seq)。負對照 blind-scan:全部改成 range_max
        if let Some(mut sc) = pl.take_scan() {
            if neg == "blind-scan" { if let Some(w) = &world { for r in sc.iter_mut() { *r = w.range_max as f32; } } }
            let mut line = format!("SCAN {} {}", k, sc.len());
            for r in &sc { line.push_str(&format!(" {:.3}", r)); }
            line.push('\n');
            if let Some(f) = scan_log.as_mut() { use std::io::Write; f.write_all(line.as_bytes()).unwrap(); }
            if let Some(su) = scan_up.as_mut() {
                su.poll_rx();
                su.tx(line.as_bytes());
                scans_sent += 1;
            }
        }
        // 撞牆那一步受控體的速度直接歸零,那不是韌體斜坡的事,C9 不算它
        if plant_dt > 0.0 && !out.collided && !last_plant.collided {
            let acc = ((out.vl_mm_s - last_plant.vl_mm_s) / plant_dt).abs().max(((out.vr_mm_s - last_plant.vr_mm_s) / plant_dt).abs());
            if acc > max_plant_accel { max_plant_accel = acc; }
        }
        path_len_mm += ((out.x_mm - last_plant.x_mm).powi(2) + (out.y_mm - last_plant.y_mm).powi(2)).sqrt();
        last_plant = out;
        { let d = ph.elapsed(); t_plant += d; if d > m_plant { m_plant = d; } }
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
            "cont" => {
                let sgn: i32 = if a.negative == "enc-swap" { -1 } else { 1 };
                let p = [out.ticks_l.wrapping_mul(sgn), out.ticks_r.wrapping_mul(sgn)];
                // 這一步邊界讀到的 CNT 是上一筆錨點外插到現在;受控體剛走完同一段 → 同一時刻,可以比
                for w in 0..2 {
                    let e = (cont_cnt[w] as u16).wrapping_sub(p[w] as u16) as i16 as i32;
                    cont_last_err[w] = e;
                    if k > 0 && e.abs() > cont_max_err { cont_max_err = e.abs(); }
                }
                max_step_ticks = max_step_ticks.max(dl.abs()).max(dr.abs());
                hk.enc_cont_update(p, [rate_milli(out.vl_mm_s) * sgn, rate_milli(out.vr_mm_s) * sgn]).unwrap();
                hk.wait_acks().unwrap();
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
        { let d = ph.elapsed(); t_hook += d; if d > m_hook { m_hook = d; } }

        // 6. 紀錄
        let (cv, cw) = if up.is_some() { up_last_cmd } else { cmd_at(&script, t_s) };
        let cs = can_status.unwrap_or((0, 0, 0, 0));
        let wall_ms = if realtime { format!("{:.1},", wall0.elapsed().as_secs_f64() * 1000.0) } else { String::new() };
        writeln!(log, "{k},{t_us},{wall_ms}{cv},{cw},{sp_l},{sp_r},{meas_l},{meas_r},{duty_l_dbg},{ccr1},{ccr2},{},{},{},{flags},\
{:.1},{:.1},{:.4},{:.1},{:.1},{},{},{},{},{},{},{},{},{},{},{},{}",
            dir_l as u8, dir_r as u8, en as u8,
            out.x_mm, out.y_mm, out.th_rad, out.vl_mm_s, out.vr_mm_s, out.ticks_l, out.ticks_r,
            last_odom.seq, last_odom.x_mm, last_odom.y_mm, last_odom.th_mrad, last_odom.vl_mm_s, last_odom.vr_mm_s,
            last_odom.flags, cs.0, cs.1, out.collided as u8).unwrap();
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
    // cont:沒有「一步延遲」這件事(CNT 外插到當下);判準是步邊界追蹤誤差 ≤ 一步的 tick 數 + 1,
    // 且車停下後(腳本最後 1 s 靜止)誤差歸零——CNT 與受控體 tick 逐字相等
    // --skew 刻意讓 Renode 與受控體的時鐘不同步,步內外插必然落後(均勻 R 時穩態落後 ≈ v·tau·(1−R)/R),上界不驗
    let cont_ok = (cont_max_err <= max_step_ticks + 1 || a.skew != "none") && cont_last_err == [0, 0]
        && cnt_l == last_plant.ticks_l as u32 & 0xFFFF && cnt_r == last_plant.ticks_r as u32 & 0xFFFF;
    let tim_ok = enc_mode == "can" || (cnt_l == last_plant.ticks_l as u32 & 0xFFFF && cnt_r == last_plant.ticks_r as u32 & 0xFFFF
        && fw_enc_l == ticks_before_last[0] && fw_enc_r == ticks_before_last[1]);
    let ctrl_steps = ec.read_u32_at(bus, dbg_base + 4 * dbg::CTRL_STEPS).unwrap();
    let rx_overflow = ec.read_u32_at(bus, dbg_base + 4 * dbg::RX_OVERFLOW).unwrap();
    let tick_ms = ec.read_u32_at(bus, dbg_base + 4 * dbg::TICK_MS).unwrap();

    println!("[run] steps={} renode_t_us={} wall={:.2}s ({:.1} ms/step, x{:.2} realtime)",
        steps, t_end, wall.as_secs_f64(), wall.as_secs_f64() * 1000.0 / steps as f64,
        (t_end as f64 / 1e6) / wall.as_secs_f64());
    println!("[run] per-step wall: upper={:.1} ms ec_read={:.1} ms hook={:.1} ms plant={:.1} ms {}={:.1} ms;單步最大 upper={:.1} ec={:.1} hook={:.1} plant={:.1} ms",
        t_upper.as_secs_f64() * 1000.0 / steps as f64,
        t_ec.as_secs_f64() * 1000.0 / steps as f64, t_hook.as_secs_f64() * 1000.0 / steps as f64,
        t_plant.as_secs_f64() * 1000.0 / steps as f64,
        if realtime { "sleep" } else { "run_for" }, t_sleep.as_secs_f64() * 1000.0 / steps as f64,
        m_upper.as_secs_f64() * 1000.0, m_ec.as_secs_f64() * 1000.0, m_hook.as_secs_f64() * 1000.0, m_plant.as_secs_f64() * 1000.0);
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
    if world.is_some() {
        println!("[run] world: collisions={} steps{} scans_sent={} end=({:.0}, {:.0}) mm goal=({:.0}, {:.0}) mm dist={:.0} mm",
            collisions, first_collision.map(|k| format!(" (第一次 @{:.3}s)", k as f64 * dt_s)).unwrap_or_default(), scans_sent,
            last_plant.x_mm, last_plant.y_mm, world.as_ref().unwrap().goal.0 * 1000.0, world.as_ref().unwrap().goal.1 * 1000.0,
            ((last_plant.x_mm - world.as_ref().unwrap().goal.0 * 1000.0).powi(2) + (last_plant.y_mm - world.as_ref().unwrap().goal.1 * 1000.0).powi(2)).sqrt());
    }
    if a.dbg_extra > 0 {
        let extra = ec.read_u32s_at(bus, dbg_base + 4 * dbg::WORDS as u64, a.dbg_extra).unwrap();
        println!("[run] fw extra {:?}", extra);
    }
    println!("[run] plant  x={:.1} y={:.1} th={:.4}", last_plant.x_mm, last_plant.y_mm, last_plant.th_rad);
    println!("[run] odom   x={} y={} th={:.4} (seq {})", last_odom.x_mm, last_odom.y_mm,
        last_odom.th_mrad as f64 / 1000.0, last_odom.seq);

    // 驗收
    let expect_time = if a.skew == "none" { steps as u64 * c.control_period_ms * 1000 } else { renode_advanced_us };
    if a.skew != "none" {
        println!("[skew] renode 推進 {:.3}s / 受控體 {:.3}s = {:.3};停頓 {} 次;max |dv/dt| = {:.0} mm/s²(輪上限 {:.0} × 1.2 = {:.0})",
            renode_advanced_us as f64 / 1e6, steps as f64 * dt_s + skew_stalls as f64 * (skew_mult as f64 - 1.0) * dt_s,
            renode_advanced_us as f64 / 1e6 / (steps as f64 * dt_s + skew_stalls as f64 * (skew_mult as f64 - 1.0) * dt_s),
            skew_stalls, max_plant_accel, wheel_accel_limit, wheel_accel_limit * 1.2);
    }
    let renode_el_us = t_end - t0;
    let expect_odom = if realtime || a.skew != "none" { renode_el_us / (c.report_period_ms * 1000) } else { (steps / report_every) as u64 };
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

    // 安全 I/O(C10):只在有故障注入時驗;每一種故障一個判準,負對照 --negative X-off 用同一條判準必須紅
    let resets_end = ec.read_u32_at(bus, dbg_base + 4 * dbg::RESETS).unwrap();
    let ping_frames = ec.read_u32_at(bus, dbg_base + 4 * dbg::PING_FRAMES).unwrap();
    let step_ms = |k: Option<u32>| k.map(|k| k as f64 * dt_s * 1000.0);
    let fault_ms = a.fault_at * 1000.0;
    let hang_counts = fault == "hang";   // 重啟後 g_dbg 的計數歸零,cmd/odom/enc 的等式不成立
    let (c10_name, c10_pass, c10_detail): (String, bool, String) = match fault.as_str() {
        "hang" => {
            let t_reset = step_ms(reset_step);
            // 死掉的時刻以 ctrl_steps 停止增加為準(RTOS 版的 tick 與 Renode 時間差一個開機與 SysTick 首週期)
            let t_hang = step_ms(hang_step).unwrap_or(fault_ms);
            let within = t_reset.map(|t| t - t_hang <= c.iwdg_timeout_ms as f64 * 1.2 + 100.0).unwrap_or(false);
            let stopped = ccr_after_reset == Some(0);
            // 韌體要自己知道這次開機是暖重置(WDT_RESET 亮):上位的鎖(C12)靠它;noinit-off 時只剩 RCC_CSR 這條證據
            let wdt_flag = reset_step.is_some() && first_flag[7].map(|k| k >= reset_step.unwrap()).unwrap_or(false);
            let boot_csr_end = ec.read_u32_at(bus, dbg_base + 4 * dbg::BOOT_CSR).unwrap();
            // 開機前寫進 flash 的 g_cfg 在重啟後還在不在(`macro reset` 會重跑 LoadELF)
            let cfg_after = ec.read_u32s_at(bus, cfg_base, 12).unwrap();
            println!("[fault] 跑完讀回 g_cfg(SRAM):hang_at_ms={} safety_mask=0x{:02x}(開機前寫進 flash 的是 hang_at_ms={})", cfg_after[7] as i32, cfg_after[11], fault_at_ms + a.boot_ms as u32);
            (format!("C10 IWDG:韌體死掉後 {} ms 內重啟、馬達停、韌體亮 WDT_RESET", c.iwdg_timeout_ms),
             // noinit-off 時 .noinit 計數被清,resets 恆 0;重啟改由 tick 倒退(reset_step)判定
             within && (resets_end >= 1 || (neg == "noinit-off" && reset_step.is_some())) && stopped && wdt_flag,
             format!("死於 {:.0} ms → 重啟 {}(resets={},WDT_RESET {},重啟後 boot_csr=0x{:08x} IWDGRSTF={});死掉期間 CCR 停在 {}(馬達照轉),重啟那步 CCR {};之後上位命令又進來,ENABLED 於 {}",
                t_hang, t_reset.map(|t| format!("@{:.0} ms(+{:.0})", t, t - t_hang)).unwrap_or("沒發生".into()), resets_end,
                if wdt_flag { "亮" } else { "沒亮" }, boot_csr_end, (boot_csr_end >> 29) & 1,
                ccr_while_hung_max, ccr_after_reset.map(|v| v.to_string()).unwrap_or("—".into()),
                step_ms(enabled_after_reset).map(|t| format!("{:.0} ms", t)).unwrap_or("—".into())))
        }
        "drv-fault" => {
            let react = step_ms(first_flag[3]).map(|t| t - fault_ms);
            (format!("C10 驅動器故障腳:拉低後 10 ms 內 DRV_FAULT、EN 低、CCR 0,放開後恢復"),
             react.map(|r| r <= 10.0).unwrap_or(false) && drv_violations == 0 && drv_recovered,
             format!("旗標 {} ;故障期間 CCR≠0 或 EN=1 的步數 {};放開後 EN 恢復 {}",
                react.map(|r| format!("+{:.0} ms", r)).unwrap_or("沒出現".into()), drv_violations, drv_recovered))
        }
        "bumper" => {
            let x_end = last_plant.x_mm;
            (format!("C10 保險桿:撞牆後 {:.0} mm 內停、拒絕前進、接受倒車", 60.0),
             first_flag[4].is_some() && x_max <= BUMPER_WALL_MM + 60.0 && x_end <= x_max - 100.0,
             format!("牆 {:.0} mm,x 最遠 {:.1}(超出 {:.1}),旗標 {},倒車後 x 末端 {:.1}",
                BUMPER_WALL_MM, x_max, x_max - BUMPER_WALL_MM, if first_flag[4].is_some() { "有" } else { "沒出現" }, x_end))
        }
        "stall" => {
            let react = step_ms(first_flag[5]).map(|t| t - fault_ms);
            let limit = c.stall_ms as f64 + 300.0;
            (format!("C10 堵轉:輪子卡住後 {:.0} ms 內 STALL 且 CCR 0,命令歸零才解", limit),
             react.map(|r| r <= limit).unwrap_or(false) && stall_violations == 0 && last_flags & flag::STALL == 0,
             format!("旗標 {};STALL 期間 CCR≠0 的步數 {};結束時 STALL {}",
                react.map(|r| format!("+{:.0} ms", r)).unwrap_or("沒出現".into()), stall_violations, if last_flags & flag::STALL != 0 { "仍鎖住" } else { "已解" }))
        }
        "no-ping" => {
            let react = step_ms(first_flag[6]).map(|t| t - fault_ms);
            let stop = step_ms(stop_step).map(|t| t - fault_ms);
            let brake_ms = if c.accel_limit_mm_s2 > 0.0 { 300.0 / c.accel_limit_mm_s2 * 1000.0 } else { 0.0 };
            let limit = c.heartbeat_timeout_ms as f64 + brake_ms + 200.0;
            (format!("C10 心跳:PING 停後 {} ms 內 HB_LOST、{:.0} ms 內車停", c.heartbeat_timeout_ms + 20, limit),
             react.map(|r| r <= c.heartbeat_timeout_ms as f64 + 20.0).unwrap_or(false) && stop.map(|t| t <= limit).unwrap_or(false),
             format!("PING 停於 {:.0} ms(韌體共收 {} 筆);旗標 {};車停 {}",
                fault_ms, ping_frames, react.map(|r| format!("+{:.0} ms", r)).unwrap_or("沒出現".into()), stop.map(|t| format!("+{:.0} ms", t)).unwrap_or("沒停".into())))
        }
        _ => ("C10 安全 I/O(沒有故障注入,不驗)".into(), true, format!("PING {} 筆,旗標軌跡見 [flags]", ping_frames)),
    };

    let checks = vec![
        // realtime:Renode 跑多快由主機決定,不是驗收項;驗收的是「三個時鐘互相一致」——
        // Renode 時間要有在走,而且不能明顯超前牆鐘。Renode 的實時節拍是以量子為單位追牆鐘,
        // 量到虛擬時間領先牆鐘最多 +1.8%(quantum 1 ms、3 s),所以留 5% + 20 ms。
        Check { name: if realtime { "C1 (realtime) 0 < Renode 時間 ≤ 牆鐘 + 5%" } else { "C1 時間完整性 renode_t == steps*dt" },
            pass: if realtime { renode_el_us > 0 && renode_el_us as f64 <= wall.as_micros() as f64 * 1.05 + 20_000.0 } else { renode_el_us == expect_time },
            detail: format!("{} vs {}", renode_el_us, if realtime { wall.as_micros() as u64 } else { expect_time }) },
        Check { name: "C2 車有動(有非零命令時,路徑長 > 100 mm)", pass: !expect_move || dist > 100.0 || hang_counts,
            detail: format!("plant 路徑長 {:.1} mm", dist) },
        Check { name: if hang_counts { "C3 (hang 重啟,不驗)" } else { "C3 韌體 odom 對受控體真值" }, pass: hang_counts || (dx <= tol_mm && dy <= tol_mm && dth <= tol_rad),
            detail: format!("dx={dx:.1} dy={dy:.1} dth={dth:.4} (tol {tol_mm:.1} mm / {tol_rad:.4} rad)") },
        Check { name: if sc.is_some() { "C4 (socketcan) 沒有事件時刻快照,不驗" } else { "C4 兩條獨立管道一致:每筆 CAN 狀態 duty == 同一時刻的 CCR 快照" },
            pass: sc.is_some() || (can_cmp_total > 0 && can_cmp_mismatch == 0),
            detail: if sc.is_some() { format!("狀態框 {} 筆(經 vcan)", can_status_count) } else { format!("{} 筆比對,{} 筆不符", can_cmp_total, can_cmp_mismatch) } },
        // odom 是韌體按「它的」時間每 report_period 送一次,期望值用 Renode 時間算,不用牆鐘
        Check { name: if hang_counts { "C5 (hang 重啟,不驗)" } else { "C5 odom 回報數 ≥ 90% 期望(按 Renode 時間)" }, pass: hang_counts || odom_count as f64 >= 0.9 * expect_odom as f64,
            detail: format!("{} / {}", odom_count, expect_odom) },
        // C9:斜坡生效 → 受控體的輪加速度不超過上限 × 1.2(斜坡限的是設定點,PI 追斜坡的瞬態量到 +7%;
        // 斜坡關掉時受控體撞到馬達層的 3000 上限,1.2 × 2100 = 2520 分得開);accel=0 時不驗
        // 輪加速度上限 = 線加速度 + 角加速度 × 輪距/2(v、w 同時起坡時兩者相加)
        Check { name: if fault != "none" { "C9 (故障注入會硬切,不驗)" } else if cfg_accel > 0 { "C9 受控體輪加速度 ≤ (accel + alpha·track/2) × 1.2" } else { "C9 (斜坡關,accel=0) 不驗" },
            pass: cfg_accel <= 0 || fault != "none" || max_plant_accel <= wheel_accel_limit * 1.2,
            detail: format!("max |dv/dt| = {:.0} mm/s² vs {:.0}(輪上限 {:.0} × 1.2)", max_plant_accel, wheel_accel_limit * 1.2, wheel_accel_limit) },
        Check { name: "C6 韌體 bad_crc == 橋接送壞的數", pass: bad_crc == corrupted,
            detail: format!("{bad_crc} vs {corrupted}") },
        Check { name: if hang_counts { "C7 (hang 重啟,不驗)" } else { "C7 韌體收到的 cmd == 送出且未壞的數" }, pass: hang_counts || cmd_frames == sent_cmds - corrupted,
            detail: format!("{cmd_frames} vs {}", sent_cmds - corrupted) },
        // 最後一步注入的訊框要下一個 run_for 才被讀到:lockstep 固有的一步延遲
        Check { name: if enc_mode == "cont" { if a.skew != "none" { "C8 (TIM, cont, skew) 車停下後 CNT == 受控體 tick(時鐘刻意偏斜,追蹤上界不驗)" } else if realtime { "C8 (TIM, cont, realtime) 車停下後 CNT == 受控體 tick;步邊界追蹤誤差 ≤ 一步 tick + 1" } else { "C8 (TIM, cont) 步邊界 CNT 對受控體 tick 的追蹤誤差 ≤ 一步 tick + 1;車停下後逐字相等" } } else if enc_mode != "can" { if realtime { "C8 (TIM, realtime) TIM CNT == 受控體 tick mod 2^16" } else { "C8 (TIM) TIM CNT == 受控體 tick;韌體累計 == 前一步 tick(一步延遲)" } } else if realtime { "C8 (realtime) 韌體收到的編碼器訊框 ≥ 90% steps" } else { "C8 韌體收到的編碼器訊框 == steps-1(一步延遲)" },
            pass: hang_counts || if enc_mode == "cont" { cont_ok } else if enc_mode != "can" { if realtime { cnt_l == last_plant.ticks_l as u32 & 0xFFFF && cnt_r == last_plant.ticks_r as u32 & 0xFFFF } else { tim_ok } } else if realtime { enc_frames as f64 >= 0.9 * steps as f64 } else { enc_frames == steps - 1 },
            detail: if enc_mode == "cont" { format!("max |CNT − tick| = {cont_max_err}(一步最多 {max_step_ticks} tick);末步誤差 {:?};末端 CNT {cnt_l}/{cnt_r} vs plant {}/{}", cont_last_err, last_plant.ticks_l as u32 & 0xFFFF, last_plant.ticks_r as u32 & 0xFFFF) } else if enc_mode != "can" { format!("CNT {cnt_l}/{cnt_r} vs plant {}/{};fw {fw_enc_l}/{fw_enc_r} vs 前一步 {}/{}", last_plant.ticks_l as u32 & 0xFFFF, last_plant.ticks_r as u32 & 0xFFFF, ticks_before_last[0], ticks_before_last[1]) } else { format!("{enc_frames} vs {}", if realtime { steps } else { steps - 1 }) } },
        Check { name: c10_name.leak(), pass: c10_pass, detail: c10_detail },
        // C12:上位對 WDT_RESET 的反應——韌體重啟後 0.5 s 起到跑完,受控體不得再動(上位該把命令歸零、取消 goal);
        // 只在 hang + 外部上位驗。負對照 --negative no-latch 是上位那側的參數(driver 不反應),橋接只印標籤
        Check { name: if fault == "hang" && up.is_some() { "C12 上位對 WDT_RESET 的反應:重啟 0.5 s 後車不再動" } else { "C12 (沒有 hang + 外部上位,不驗)" },
            pass: !(fault == "hang" && up.is_some()) || (reset_step.is_some() && moved_after_reset == 0),
            detail: if fault == "hang" && up.is_some() { format!("重啟 {};重啟 0.5 s 後 |v| ≥ 5 mm/s 的步數 {},最遠再走 {:.0} mm",
                step_ms(reset_step).map(|t| format!("@{:.0} ms", t)).unwrap_or("沒發生".into()), moved_after_reset, dist_after_reset) } else { String::new() } },
        // C11:有世界且上位是外部的(Nav2)→ 到達 goal 0.1 m 內、途中沒撞;沒世界不驗
        // 有故障注入時不驗到達:那個場景在驗 C10/C12(車該停),不是該到
        Check { name: if world.is_some() && a.expect_goal && fault == "none" { "C11 Nav2:受控體到達 world.goal 0.1 m 內,途中沒撞牆或方塊" } else if fault != "none" { "C11 (故障注入場景,不驗到達)" } else { "C11 (沒有 --expect-goal,不驗)" },
            pass: match (&world, a.expect_goal && fault == "none") {
                (Some(w), true) => collisions == 0 && ((last_plant.x_mm - w.goal.0 * 1000.0).powi(2) + (last_plant.y_mm - w.goal.1 * 1000.0).powi(2)).sqrt() <= 100.0,
                _ => true },
            detail: match &world {
                Some(w) => format!("末端 ({:.0}, {:.0}) 對 goal ({:.0}, {:.0}) 差 {:.0} mm;碰撞 {} 步{}", last_plant.x_mm, last_plant.y_mm, w.goal.0 * 1000.0, w.goal.1 * 1000.0,
                    ((last_plant.x_mm - w.goal.0 * 1000.0).powi(2) + (last_plant.y_mm - w.goal.1 * 1000.0).powi(2)).sqrt(), collisions,
                    first_collision.map(|k| format!("(第一次 @{:.3}s)", k as f64 * dt_s)).unwrap_or_default()),
                None => String::new() } },
    ];
    let mut all = true;
    for ch in &checks {
        println!("[{}] {} — {}", if ch.pass { "PASS" } else { "FAIL" }, ch.name, ch.detail);
        all &= ch.pass;
    }
    println!("[result] {}", if all { "ALL PASS" } else { "SOME FAIL" });
    std::process::exit(if all { 0 } else { 1 });
}
