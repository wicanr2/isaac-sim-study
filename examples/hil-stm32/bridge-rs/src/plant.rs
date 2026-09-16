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

/// 馬達層,三個受控體實作(這裡、plant/fake_plant.py、plant/isaac_plant.py)同一份公式:
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
        Fake { c, world: None, scan: None, tau_s: c.motor_tau_s, vl: 0.0, vr: 0.0, sl_mm: 0.0, sr_mm: 0.0, x: 0.0, y: 0.0, th: 0.0 }
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
        let full = self.c.wheel_speed_full_mm_s;
        let db = self.c.motor_deadband_duty;
        let tl = motor_target(cmd.duty_l, cmd.fwd_l, cmd.enabled, db, full);
        let tr = motor_target(cmd.duty_r, cmd.fwd_r, cmd.enabled, db, full);
        self.vl = motor_advance(self.vl, tl, dt, self.tau_s, self.c.motor_accel_max_mm_s2);
        self.vr = motor_advance(self.vr, tr, dt, self.tau_s, self.c.motor_accel_max_mm_s2);
        let dl = self.vl * dt;
        let dr = self.vr * dt;
        // 碰撞與假雷射(有世界才有):碰到就停在原地(牆與方塊不讓車穿過),雷射每 period_ms 一筆
        let (mut collided, mut do_scan) = (false, false);
        if let Some(w) = &self.world {
            let step_ms = (dt * 1000.0).round() as u64;
            do_scan = step_ms > 0 && (seq as u64 * step_ms) % w.period_ms == 0;
            let ds = (dl + dr) * 0.5 / 1000.0;
            let nx = self.x / 1000.0 + ds * self.th.cos();
            let ny = self.y / 1000.0 + ds * self.th.sin();
            collided = w.collides(nx, ny);
        }
        if collided {
            self.vl = 0.0; self.vr = 0.0;
            if do_scan { let w = self.world.as_ref().unwrap(); self.scan = Some(w.scan(self.x / 1000.0, self.y / 1000.0, self.th)); }
            return Ok(PlantOut { ticks_l: self.ticks(self.sl_mm), ticks_r: self.ticks(self.sr_mm), x_mm: self.x, y_mm: self.y, th_rad: self.th, vl_mm_s: 0.0, vr_mm_s: 0.0, collided: true });
        }
        self.sl_mm += dl;
        self.sr_mm += dr;
        let ds = (dl + dr) * 0.5;
        let dth = (dr - dl) / self.c.track_mm;
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
        "CMD {} {} {} {} {} {} {}\n",
        seq,
        (dt * 1000.0).round() as i64,
        (cmd.duty_l * 1000.0).round() as i64,
        (cmd.duty_r * 1000.0).round() as i64,
        cmd.fwd_l as u8,
        cmd.fwd_r as u8,
        cmd.enabled as u8
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
