//! renode/hil_hook.py 的對端:一條 TCP,13 bytes 一筆 `u32 id | u8 len | u8[8] data`。
//! 注入(CAN 或 UART)後等 ack 才算完成——這是 lockstep 決定性的前提。

use std::io::{self, Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

pub const ID_UART_TO_MCU: u32 = 0xFFFF_0001;
pub const ID_UART_FROM_MCU: u32 = 0xFFFF_0002;
pub const ID_BUS_SNAPSHOT: u32 = 0xFFFF_0003;
pub const ID_START: u32 = 0xFFFF_0010;
pub const ID_PAUSE: u32 = 0xFFFF_0011;
pub const ID_ENCODER_STEPS: u32 = 0xFFFF_0020;
pub const ID_ENC_CONT_CFG: u32 = 0xFFFF_0021;
pub const ID_ENC_CONT_L: u32 = 0xFFFF_0022;
pub const ID_ENC_CONT_R: u32 = 0xFFFF_0023;
pub const ID_ACK: u32 = 0xFFFF_00AC;

#[derive(Debug, Clone)]
pub struct Rec {
    pub id: u32,
    pub data: Vec<u8>,
}

pub struct Hook {
    s: TcpStream,
    pending_acks: u32,
    /// 收到但尚未被上層取走的紀錄(CAN 訊框、UART 位元組)
    inbox: Vec<Rec>,
}

impl Hook {
    pub fn connect_retry<A: ToSocketAddrs + Clone>(addr: A, retry: Duration) -> io::Result<Hook> {
        let t0 = std::time::Instant::now();
        loop {
            match Hook::connect(addr.clone()) {
                Ok(h) => return Ok(h),
                Err(e) if t0.elapsed() < retry && e.kind() == io::ErrorKind::ConnectionRefused => {
                    std::thread::sleep(Duration::from_millis(250));
                }
                Err(e) => return Err(e),
            }
        }
    }

    pub fn connect<A: ToSocketAddrs>(addr: A) -> io::Result<Hook> {
        let s = TcpStream::connect(addr)?;
        s.set_nodelay(true)?;
        s.set_read_timeout(Some(Duration::from_millis(2000)))?;
        Ok(Hook { s, pending_acks: 0, inbox: Vec::new() })
    }

    fn write_rec(&mut self, id: u32, data: &[u8]) -> io::Result<()> {
        assert!(data.len() <= 8);
        let mut b = [0u8; 13];
        b[..4].copy_from_slice(&id.to_le_bytes());
        b[4] = data.len() as u8;
        b[5..5 + data.len()].copy_from_slice(data);
        self.s.write_all(&b)
    }

    /// 注入一個 CAN 訊框(受控體 → MCU)。
    pub fn can_send(&mut self, id: u32, data: &[u8]) -> io::Result<()> {
        self.write_rec(id, data)?;
        self.pending_acks += 1;
        Ok(())
    }

    /// 編碼器計數差(左、右)→ hook 在 Renode 裡對 TIM2/TIM4 的輸入腳打正交脈衝,一筆一 ack。
    pub fn encoder_steps(&mut self, dl: i32, dr: i32) -> io::Result<()> {
        let mut d = [0u8; 8];
        d[..4].copy_from_slice(&dl.to_le_bytes());
        d[4..].copy_from_slice(&dr.to_le_bytes());
        self.write_rec(ID_ENCODER_STEPS, &d)?;
        self.pending_acks += 1;
        Ok(())
    }

    /// 連續編碼器(37 篇 §3):在 TIM2/TIM4 的 CNT 裝讀取 hook,CNT 在韌體讀的當下由錨點算出;tau = 誤差攤還時間。
    pub fn enc_cont_install(&mut self, tau_us: i32) -> io::Result<()> {
        self.write_rec(ID_ENC_CONT_CFG, &tau_us.to_le_bytes())?;
        self.pending_acks += 1;
        Ok(())
    }

    /// 每步一次:受控體累計 tick 與輪速(milli-tick/s),左右各一筆。
    pub fn enc_cont_update(&mut self, ticks: [i32; 2], rate_milli: [i32; 2]) -> io::Result<()> {
        for (w, id) in [(0usize, ID_ENC_CONT_L), (1usize, ID_ENC_CONT_R)] {
            let mut d = [0u8; 8];
            d[..4].copy_from_slice(&ticks[w].to_le_bytes());
            d[4..].copy_from_slice(&rate_milli[w].to_le_bytes());
            self.write_rec(id, &d)?;
            self.pending_acks += 1;
        }
        Ok(())
    }

    /// 送 UART 位元組給 MCU(上位 → MCU),每 8 bytes 一筆。
    pub fn uart_send(&mut self, bytes: &[u8]) -> io::Result<()> {
        for chunk in bytes.chunks(8) {
            self.write_rec(ID_UART_TO_MCU, chunk)?;
            self.pending_acks += 1;
        }
        Ok(())
    }

    /// realtime 模式:讓 Renode 自由跑 / 暫停(經 hook 的 StartAll / PauseAll)。
    pub fn emulation_start(&mut self) -> io::Result<()> {
        self.write_rec(ID_START, &[])?;
        self.pending_acks += 1;
        Ok(())
    }

    pub fn emulation_pause(&mut self) -> io::Result<()> {
        self.write_rec(ID_PAUSE, &[])?;
        self.pending_acks += 1;
        Ok(())
    }

    /// 要求 hook 把它手上不足 8 bytes 的 UART 輸出先吐過來。
    pub fn uart_flush_request(&mut self) -> io::Result<()> {
        self.write_rec(ID_UART_TO_MCU, &[])?;
        self.pending_acks += 1;
        Ok(())
    }

    fn read_one(&mut self) -> io::Result<Rec> {
        let mut b = [0u8; 13];
        self.s.read_exact(&mut b)?;
        let id = u32::from_le_bytes([b[0], b[1], b[2], b[3]]);
        let n = (b[4] as usize).min(8);
        Ok(Rec { id, data: b[5..5 + n].to_vec() })
    }

    /// 等所有未完成的注入被 ack;途中收到的紀錄放進 inbox。
    pub fn wait_acks(&mut self) -> io::Result<()> {
        while self.pending_acks > 0 {
            let r = self.read_one()?;
            if r.id == ID_ACK {
                self.pending_acks -= 1;
            } else {
                self.inbox.push(r);
            }
        }
        Ok(())
    }

    /// 非阻塞地把目前 socket 裡已到的紀錄收進 inbox(用短 timeout 探一次)。
    pub fn drain(&mut self, timeout: Duration) -> io::Result<()> {
        self.s.set_read_timeout(Some(timeout))?;
        loop {
            match self.read_one() {
                Ok(r) => {
                    if r.id == ID_ACK {
                        self.pending_acks = self.pending_acks.saturating_sub(1);
                    } else {
                        self.inbox.push(r);
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock || e.kind() == io::ErrorKind::TimedOut => break,
                Err(e) => {
                    self.s.set_read_timeout(Some(Duration::from_millis(2000)))?;
                    return Err(e);
                }
            }
        }
        self.s.set_read_timeout(Some(Duration::from_millis(2000)))?;
        Ok(())
    }

    pub fn take_inbox(&mut self) -> Vec<Rec> {
        std::mem::take(&mut self.inbox)
    }
}
