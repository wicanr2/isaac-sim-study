//! 上位側的 TCP 出口:外部上位(ROS 2 節點)連進來,講的是與韌體相同的 UART 框包。
//! 橋接在這一側只當一條「序列線」:收到的 byte 原樣注入 USART1,MCU 吐出的 byte 原樣送回。
//! 語意(cmd_vel 的內容)橋接不管;它只數框包給 C7 用,並記最後一個命令進 CSV。

use std::io::{self, Read, Write};
use std::net::{TcpListener, TcpStream};

pub struct Upper {
    listener: TcpListener,
    conn: Option<TcpStream>,
    pub connected_once: bool,
}

impl Upper {
    pub fn listen(addr: &str) -> io::Result<Self> {
        let listener = TcpListener::bind(addr)?;
        listener.set_nonblocking(true)?;
        Ok(Self { listener, conn: None, connected_once: false })
    }

    fn accept_if_none(&mut self) {
        if self.conn.is_some() {
            return;
        }
        if let Ok((s, _)) = self.listener.accept() {
            let _ = s.set_nodelay(true);
            let _ = s.set_nonblocking(true);
            self.conn = Some(s);
            self.connected_once = true;
        }
    }

    /// 這一步之前上位送來的所有 byte(非阻塞,沒有就是空)。
    pub fn poll_rx(&mut self) -> Vec<u8> {
        self.accept_if_none();
        let mut out = Vec::new();
        let mut drop = false;
        if let Some(s) = self.conn.as_mut() {
            let mut buf = [0u8; 256];
            loop {
                match s.read(&mut buf) {
                    Ok(0) => { drop = true; break; }
                    Ok(n) => out.extend_from_slice(&buf[..n]),
                    Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                    Err(_) => { drop = true; break; }
                }
            }
        }
        if drop {
            self.conn = None;
        }
        out
    }

    /// MCU 吐出的 byte 原樣送給上位;沒連線就丟掉(上位不在,odom 沒人看)。
    pub fn tx(&mut self, bytes: &[u8]) {
        let mut drop = false;
        if let Some(s) = self.conn.as_mut() {
            if let Err(e) = s.write_all(bytes) {
                if e.kind() != io::ErrorKind::WouldBlock {
                    drop = true;
                }
            }
        }
        if drop {
            self.conn = None;
        }
    }

    pub fn is_connected(&self) -> bool {
        self.conn.is_some()
    }
}
