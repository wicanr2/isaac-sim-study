//! 受控體(plant):吃 MCU 的馬達命令,回編碼器 tick 與真值位姿。
//!
//! 兩種後端同一個介面:
//! - `Fake`:純軟體差速車(一階馬達 + 精確差速運動學),決定性,拿來把橋接驗到綠
//! - `Udp`:把同一組量丟給另一個行程(plant/fake_plant.py 或 Isaac Sim 的 receiver),
//!   文字協定一行一筆,見 plant/README.md
//!
//! 橋接不做安全:不夾限、不逾時、不幫忙停車。這裡只轉譯與紀錄。

use std::io::{self, BufRead, BufReader, Write};
use std::net::{TcpStream, UdpSocket};
use std::time::Duration;

use crate::calib::Calib;
use crate::world::World;

/// MCU 這一步送出的馬達命令(已從匯流排讀回)
#[derive(Debug, Clone, Copy, Default)]
pub struct MotorCmd {
    pub duty_l: f64, // 0..1
    pub duty_r: f64,
    pub fwd_l: bool,
    pub fwd_r: bool,
    pub enabled: bool,
    /// 堵轉注入:輪子被卡住(ω 強制 0,車體也不動)。扭矩模型下「不給 duty」只會讓輪子靠反電動勢慢慢停,
    /// 那不是卡住,所以卡住要由受控體直接表示(38 篇 §1.2 的 stall)
    pub locked: bool,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct PlantOut {
    pub ticks_l: i32, // 累計,會繞回
    pub ticks_r: i32,
    pub x_mm: f64, // 真值位姿(橋接只拿來對照,不回給 MCU)
    pub y_mm: f64,
    pub th_rad: f64,
    pub vl_mm_s: f64,
    pub vr_mm_s: f64,
    /// 這一步受控體與牆或方塊相交(world.json 的世界;沒有世界的受控體永遠 false)
    pub collided: bool,
}

pub trait Plant {
    fn step(&mut self, seq: u32, dt_s: f64, cmd: MotorCmd) -> io::Result<PlantOut>;
    /// 這一步有雷射掃描就拿走(假雷射每 period_ms 一筆;Isaac 版沒有)。橋接只轉給上位,不解語意。
    fn take_scan(&mut self) -> Option<Vec<f32>> { None }
}

/// 馬達扭矩(直流馬達的扭矩–轉速直線,PWM 當電壓比例;36 篇 §3.3):
/// `τ = duty × τ_stall − (τ_stall / ω_free) × ω`,夾在 ±τ_max。duty 帶號(倒退為負)。
pub fn motor_torque(duty: f64, omega: f64, stall_nm: f64, free_rad_s: f64, max_nm: f64) -> f64 {
    let t = duty * stall_nm - stall_nm / free_rad_s * omega;
    t.clamp(-max_nm, max_nm)
}

/// 一個輪子這一步:正規化庫侖摩擦 + 半隱式積分(36 篇 §3.3)。
/// 馬達扭矩 `τ = t0 − b·ω`(t0 = duty × τ_stall、b = τ_stall / ω_free),夾在 ±τ_max;
/// 摩擦 `F = clamp(k·(ωr − v), ±μN)`,k = μN / v_ref。反電動勢與摩擦都用隱式解——兩者的
/// 顯式增益在 5 ms 步長下都接近 1,顯式積分會讓輪速每步在兩個值之間跳(量到 442 ↔ 18 mm/s)。
/// 回傳 (施在車體上的力 N, 新的輪角速度 rad/s)。
pub fn wheel_step(t0: f64, b: f64, tau_max: f64, omega: f64, v_wheel: f64, dt: f64,
                  r: f64, mu: f64, n_force: f64, i_w: f64, v_ref: f64) -> (f64, f64) {
    let f_max = mu * n_force;
    let k = f_max / v_ref;
    let a = i_w / dt;
    // 1. 黏著(摩擦未飽和)+ 扭矩未飽和
    let w1 = (a * omega + t0 + k * r * v_wheel) / (a + b + k * r * r);
    let f1 = k * (w1 * r - v_wheel);
    if f1.abs() <= f_max {
        let tau1 = t0 - b * w1;
        if tau1.abs() <= tau_max {
            return (f1, w1);
        }
        // 2. 扭矩飽和、摩擦仍黏著
        let tau_c = tau1.signum() * tau_max;
        let w2 = (a * omega + tau_c + k * r * v_wheel) / (a + k * r * r);
        let f2 = k * (w2 * r - v_wheel);
        if f2.abs() <= f_max {
            return (f2, w2);
        }
    }
    // 3. 滑動:摩擦飽和在 μN,方向與滑移相反;扭矩仍可能飽和
    let dir = if w1 * r - v_wheel >= 0.0 { 1.0 } else { -1.0 };
    let f = dir * f_max;
    let w3 = (a * omega + t0 - f * r) / (a + b);
    let tau3 = t0 - b * w3;
    let w_new = if tau3.abs() > tau_max { omega + dt * (tau3.signum() * tau_max - f * r) / i_w } else { w3 };
    (f, w_new)
}

/// 舊的速度源馬達層(GOAL 8 之前;`--motor speed` 才用)。三個受控體實作同一份公式:
/// 1. 死區:|duty| < deadband 不動;之後線性到滿 duty = full 速度
/// 2. 一階趨近(時間常數 tau),但每步的速度變化量不超過 accel_max × dt(電流限制 = 扭矩上限)
/// 改公式要三處一起改,末端位姿要互相在容差內(38 篇 §3)。
pub fn motor_target(duty: f64, fwd: bool, enabled: bool, deadband: f64, full: f64) -> f64 {
    if !enabled || duty <= deadband {
        return 0.0;
    }
    let mag = (duty - deadband) / (1.0 - deadband) * full;
    if fwd { mag } else { -mag }
}

pub fn motor_advance(v: f64, target: f64, dt: f64, tau: f64, accel_max: f64) -> f64 {
    let a = (dt / tau).min(1.0);
    let mut dv = (target - v) * a;
    if accel_max > 0.0 {
        let lim = accel_max * dt;
        dv = dv.clamp(-lim, lim);
    }
    v + dv
}

/// 純軟體差速車。馬達:上面的 motor_target / motor_advance;運動學:精確弧線積分。
pub struct Fake {
    c: Calib,
    /// 扭矩模型的狀態:兩輪角速度(rad/s)、車體線速度(m/s)與角速度(rad/s)
    wl_rad_s: f64,
    wr_rad_s: f64,
    v_body: f64,
    w_body: f64,
    world: Option<World>,
    scan: Option<Vec<f32>>,
    tau_s: f64,
    vl: f64,
    vr: f64,
    sl_mm: f64, // 各輪累計行程
    sr_mm: f64,
    x: f64,
    y: f64,
    th: f64,
}

impl Fake {
    pub fn new(c: Calib) -> Fake {
        Fake { c, world: None, scan: None, tau_s: c.motor_tau_s, vl: 0.0, vr: 0.0, sl_mm: 0.0, sr_mm: 0.0, x: 0.0, y: 0.0, th: 0.0,
               wl_rad_s: 0.0, wr_rad_s: 0.0, v_body: 0.0, w_body: 0.0 }
    }

    pub fn with_world(mut self, w: World) -> Fake {
        self.world = Some(w);
        self
    }

    fn ticks(&self, s_mm: f64) -> i32 {
        let rev = s_mm / (self.c.wheel_circ_um / 1000.0);
        (rev * self.c.ticks_per_rev as f64).floor() as i64 as i32
    }
}

impl Plant for Fake {
    fn take_scan(&mut self) -> Option<Vec<f32>> { self.scan.take() }

    fn step(&mut self, seq: u32, dt: f64, cmd: MotorCmd) -> io::Result<PlantOut> {
        // 扭矩模型(36 篇 §3.3):duty → 馬達扭矩 → 輪緣力(夾在抓地力 μN)→ 輪子與車體各自積分。
        // 打滑或卡住是算出來的,沒有 CONTACT 旗標
        let r = self.c.wheel_radius_mm / 1000.0;
        let half = self.c.track_mm / 2000.0;
        let db = self.c.motor_deadband_duty;
        let signed_duty = |d: f64, fwd: bool| -> f64 {
            if !cmd.enabled || d <= db { 0.0 } else { let m = (d - db) / (1.0 - db); if fwd { m } else { -m } }
        };
        let dl_duty = signed_duty(cmd.duty_l, cmd.fwd_l);
        let dr_duty = signed_duty(cmd.duty_r, cmd.fwd_r);
        let t0_l = dl_duty * self.c.motor_stall_torque_nm;
        let t0_r = dr_duty * self.c.motor_stall_torque_nm;
        let b_emf = self.c.motor_stall_torque_nm / self.c.motor_free_rad_s;

        // 這一步的掃描時機(碰撞判定移到算完輪子的力之後,見下面 collided 的註解)
        let (mut collided, mut do_scan) = (false, false);
        if let Some(w) = &self.world {
            let step_ms = (dt * 1000.0).round() as u64;
            do_scan = step_ms > 0 && (seq as u64 * step_ms) % w.period_ms == 0;
        }

        let m = self.c.robot_mass_kg;
        let n_force = m * 9.81 / 2.0;                       // 每輪的正向力(忽略腳輪分擔與載重轉移)
        let i_w = 0.4 * self.c.wheel_mass_kg * r * r;       // 輪子建模成球
        let v_l = self.v_body - self.w_body * half;
        let v_r = self.v_body + self.w_body * half;
        if cmd.locked {
            // 輪子被卡住:編碼器不動、車體不動(法向力由卡住的機構承擔)
            self.wl_rad_s = 0.0; self.wr_rad_s = 0.0; self.vl = 0.0; self.vr = 0.0;
            self.v_body = 0.0; self.w_body = 0.0;
            if do_scan { if let Some(w) = &self.world { self.scan = Some(w.scan(self.x / 1000.0, self.y / 1000.0, self.th)); } }
            return Ok(PlantOut { ticks_l: self.ticks(self.sl_mm), ticks_r: self.ticks(self.sr_mm), x_mm: self.x, y_mm: self.y,
                                 th_rad: self.th, vl_mm_s: 0.0, vr_mm_s: 0.0, collided });
        }
        let v_ref = self.c.friction_v_ref_m_s;
        let tm = self.c.motor_torque_max_nm;
        let (f_l, wl_new) = wheel_step(t0_l, b_emf, tm, self.wl_rad_s, v_l, dt, r, self.c.friction_mu, n_force, i_w, v_ref);
        let (f_r, wr_new) = wheel_step(t0_r, b_emf, tm, self.wr_rad_s, v_r, dt, r, self.c.friction_mu, n_force, i_w, v_ref);
        self.wl_rad_s = wl_new;
        self.wr_rad_s = wr_new;
        // 碰撞判定用「這一步的力算出來的新車速」去試算位置,不是用更新前的車速:
        // 用舊車速試算會鎖死——停在障礙物上時 v_body = 0,試算位置就是現在的位置、永遠判定碰撞,
        // 於是 v_body 永遠回不到非零,連倒車都退不開。GOAL 8 C 線量到的症狀是「真值 20 s 完全不動,
        // 而 odom 以為已經退了 1.25 m」(docs/hil/38 §6.5)
        let v_try = self.v_body + (f_l + f_r) / m * dt;
        let w_try = self.w_body + (f_r - f_l) * half / self.c.body_inertia_kg_m2 * dt;
        if let Some(w) = &self.world {
            let ds = v_try * dt;
            collided = w.collides(self.x / 1000.0 + ds * self.th.cos(), self.y / 1000.0 + ds * self.th.sin());
        }
        if collided {
            // 障礙物承擔法向力:車體停住(位置與朝向都不動),輪子照上面的滑動解繼續轉。
            // 離開障礙物的方向不會被擋——試算位置不碰撞就照常走
            self.v_body = 0.0;
            self.w_body = 0.0;
        } else {
            self.v_body = v_try;
            self.w_body = w_try;
        }
        // 編碼器數的是輪子轉了多少(打滑時比車體多)
        self.sl_mm += self.wl_rad_s * r * 1000.0 * dt;
        self.sr_mm += self.wr_rad_s * r * 1000.0 * dt;
        self.vl = self.wl_rad_s * r * 1000.0;
        self.vr = self.wr_rad_s * r * 1000.0;
        let ds = self.v_body * dt * 1000.0;
        let dth = self.w_body * dt;
        let th_mid = self.th + dth * 0.5;
        self.x += ds * th_mid.cos();
        self.y += ds * th_mid.sin();
        self.th += dth;
        if do_scan { if let Some(w) = &self.world { self.scan = Some(w.scan(self.x / 1000.0, self.y / 1000.0, self.th)); } }
        Ok(PlantOut {
            ticks_l: self.ticks(self.sl_mm),
            ticks_r: self.ticks(self.sr_mm),
            x_mm: self.x,
            y_mm: self.y,
            th_rad: self.th,
            vl_mm_s: self.vl,
            vr_mm_s: self.vr,
            collided,
        })
    }
}

/// 文字協定(一行一筆,ASCII,空白分隔;UDP 與 TCP 同一份):
///   橋接 → 受控體:`CMD <seq> <dt_ms> <duty_l 0..1000> <duty_r> <fwd_l 0/1> <fwd_r> <en 0/1>\n`
///   受控體 → 橋接:`ENC <seq> <ticks_l> <ticks_r> <x_mm> <y_mm> <th_rad> <vl_mm_s> <vr_mm_s> [collided 0/1]\n`
///   受控體 → 橋接(可選,ENC 之前):`SCAN <seq> <n> <r0 m> ... <r(n-1)>\n`(假雷射,每 period_ms 一筆)
/// 受控體必須以相同 seq 回覆;橋接等到 ENC 才推進下一步(lockstep)。
pub struct Udp {
    sock: UdpSocket,
    scan: Option<Vec<f32>>,
}

impl Udp {
    pub fn connect(addr: &str) -> io::Result<Udp> {
        let sock = UdpSocket::bind("0.0.0.0:0")?;
        sock.connect(addr)?;
        sock.set_read_timeout(Some(Duration::from_secs(10)))?;
        Ok(Udp { sock, scan: None })
    }
}

impl Plant for Udp {
    fn take_scan(&mut self) -> Option<Vec<f32>> { self.scan.take() }

    fn step(&mut self, seq: u32, dt: f64, cmd: MotorCmd) -> io::Result<PlantOut> {
        self.sock.send(cmd_line(seq, dt, cmd).as_bytes())?;
        let mut buf = [0u8; 4096];
        loop {
            let n = self.sock.recv(&mut buf)?;
            let line = String::from_utf8_lossy(&buf[..n]);
            if let Some(sc) = parse_scan(&line, seq) { self.scan = Some(sc); continue; }
            if let Some(out) = parse_enc(&line, seq) {
                return Ok(out);
            }
            // 舊的或格式不對的回覆:丟掉,繼續等對的 seq
        }
    }
}

/// TCP 版:同一份文字協定,一行一筆。用在受控體在另一台主機、走 `ssh -L` 隧道時
/// (ssh 的 -L 只轉 TCP)。
pub struct Tcp {
    w: TcpStream,
    r: BufReader<TcpStream>,
    scan: Option<Vec<f32>>,
}

impl Tcp {
    pub fn connect(addr: &str) -> io::Result<Tcp> {
        let s = TcpStream::connect(addr)?;
        s.set_nodelay(true)?;
        s.set_read_timeout(Some(Duration::from_secs(30)))?;
        let r = BufReader::new(s.try_clone()?);
        Ok(Tcp { w: s, r, scan: None })
    }
}

impl Plant for Tcp {
    fn take_scan(&mut self) -> Option<Vec<f32>> { self.scan.take() }

    fn step(&mut self, seq: u32, dt: f64, cmd: MotorCmd) -> io::Result<PlantOut> {
        self.w.write_all(cmd_line(seq, dt, cmd).as_bytes())?;
        let mut line = String::new();
        loop {
            line.clear();
            if self.r.read_line(&mut line)? == 0 {
                return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "受控體關閉連線"));
            }
            if let Some(sc) = parse_scan(&line, seq) { self.scan = Some(sc); continue; }
            if let Some(out) = parse_enc(&line, seq) {
                return Ok(out);
            }
        }
    }
}

fn cmd_line(seq: u32, dt: f64, cmd: MotorCmd) -> String {
    format!(
        "CMD {} {} {} {} {} {} {} {}\n",
        seq,
        (dt * 1000.0).round() as i64,
        (cmd.duty_l * 1000.0).round() as i64,
        (cmd.duty_r * 1000.0).round() as i64,
        cmd.fwd_l as u8,
        cmd.fwd_r as u8,
        cmd.enabled as u8,
        cmd.locked as u8
    )
}

fn parse_enc(s: &str, seq: u32) -> Option<PlantOut> {
    let f: Vec<&str> = s.split_whitespace().collect();
    if f.len() >= 9 && f[0] == "ENC" && f[1].parse::<u32>().ok() == Some(seq) {
        let p = |i: usize| f[i].parse::<f64>().unwrap_or(0.0);
        return Some(PlantOut {
            ticks_l: f[2].parse().unwrap_or(0),
            ticks_r: f[3].parse().unwrap_or(0),
            x_mm: p(4),
            y_mm: p(5),
            th_rad: p(6),
            vl_mm_s: p(7),
            vr_mm_s: p(8),
            collided: f.len() >= 10 && f[9] == "1",
        });
    }
    None
}

fn parse_scan(s: &str, seq: u32) -> Option<Vec<f32>> {
    let mut it = s.split_whitespace();
    if it.next()? != "SCAN" || it.next()?.parse::<u32>().ok()? != seq { return None; }
    let n: usize = it.next()?.parse().ok()?;
    let v: Vec<f32> = it.take(n).filter_map(|x| x.parse().ok()).collect();
    if v.len() == n { Some(v) } else { None }
}
