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
}

pub trait Plant {
    fn step(&mut self, seq: u32, dt_s: f64, cmd: MotorCmd) -> io::Result<PlantOut>;
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
        Fake { c, tau_s: c.motor_tau_s, vl: 0.0, vr: 0.0, sl_mm: 0.0, sr_mm: 0.0, x: 0.0, y: 0.0, th: 0.0 }
    }

    fn ticks(&self, s_mm: f64) -> i32 {
        let rev = s_mm / (self.c.wheel_circ_um / 1000.0);
        (rev * self.c.ticks_per_rev as f64).floor() as i64 as i32
    }
}

impl Plant for Fake {
    fn step(&mut self, _seq: u32, dt: f64, cmd: MotorCmd) -> io::Result<PlantOut> {
        let full = self.c.wheel_speed_full_mm_s;
        let db = self.c.motor_deadband_duty;
        let tl = motor_target(cmd.duty_l, cmd.fwd_l, cmd.enabled, db, full);
        let tr = motor_target(cmd.duty_r, cmd.fwd_r, cmd.enabled, db, full);
        self.vl = motor_advance(self.vl, tl, dt, self.tau_s, self.c.motor_accel_max_mm_s2);
        self.vr = motor_advance(self.vr, tr, dt, self.tau_s, self.c.motor_accel_max_mm_s2);
        let dl = self.vl * dt;
        let dr = self.vr * dt;
        self.sl_mm += dl;
        self.sr_mm += dr;
        let ds = (dl + dr) * 0.5;
        let dth = (dr - dl) / self.c.track_mm;
        let th_mid = self.th + dth * 0.5;
        self.x += ds * th_mid.cos();
        self.y += ds * th_mid.sin();
        self.th += dth;
        Ok(PlantOut {
            ticks_l: self.ticks(self.sl_mm),
            ticks_r: self.ticks(self.sr_mm),
            x_mm: self.x,
            y_mm: self.y,
            th_rad: self.th,
            vl_mm_s: self.vl,
            vr_mm_s: self.vr,
        })
    }
}

/// 文字協定(一行一筆,ASCII,空白分隔;UDP 與 TCP 同一份):
///   橋接 → 受控體:`CMD <seq> <dt_ms> <duty_l 0..1000> <duty_r> <fwd_l 0/1> <fwd_r> <en 0/1>\n`
///   受控體 → 橋接:`ENC <seq> <ticks_l> <ticks_r> <x_mm> <y_mm> <th_rad> <vl_mm_s> <vr_mm_s>\n`
/// 受控體必須以相同 seq 回覆;橋接等到回覆才推進下一步(lockstep)。
pub struct Udp {
    sock: UdpSocket,
}

impl Udp {
    pub fn connect(addr: &str) -> io::Result<Udp> {
        let sock = UdpSocket::bind("0.0.0.0:0")?;
        sock.connect(addr)?;
        sock.set_read_timeout(Some(Duration::from_secs(10)))?;
        Ok(Udp { sock })
    }
}

impl Plant for Udp {
    fn step(&mut self, seq: u32, dt: f64, cmd: MotorCmd) -> io::Result<PlantOut> {
        self.sock.send(cmd_line(seq, dt, cmd).as_bytes())?;
        let mut buf = [0u8; 256];
        loop {
            let n = self.sock.recv(&mut buf)?;
            if let Some(out) = parse_enc(&String::from_utf8_lossy(&buf[..n]), seq) {
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
}

impl Tcp {
    pub fn connect(addr: &str) -> io::Result<Tcp> {
        let s = TcpStream::connect(addr)?;
        s.set_nodelay(true)?;
        s.set_read_timeout(Some(Duration::from_secs(30)))?;
        let r = BufReader::new(s.try_clone()?);
        Ok(Tcp { w: s, r })
    }
}

impl Plant for Tcp {
    fn step(&mut self, seq: u32, dt: f64, cmd: MotorCmd) -> io::Result<PlantOut> {
        self.w.write_all(cmd_line(seq, dt, cmd).as_bytes())?;
        let mut line = String::new();
        loop {
            line.clear();
            if self.r.read_line(&mut line)? == 0 {
                return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "受控體關閉連線"));
            }
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
        });
    }
    None
}
