//! Renode External Control API 的 client(Renode 1.16.1)。
//!
//! 線路協定是從官方 C client(`tools/external_control_client/lib/renode_api.c`)
//! 逐位元組讀出來的,官方沒有文件。全部 little-endian。
//!
//! 握手:client 送 `u16 count` + count 組 `(cmd u8, ver u8)`;server 回 1 byte `5`。
//! 命令:`'R' 'E' cmd:u8 len:u32 data`。
//! 回應:第 1 byte 是 return code:
//!   0 COMMAND_FAILED      → cmd u8, len u32, 訊息
//!   1 FATAL_ERROR         → len u32, 訊息
//!   2 INVALID_COMMAND     → cmd u8
//!   3 SUCCESS_WITH_DATA   → cmd u8, len u32, data
//!   4 SUCCESS_WITHOUT_DATA→ cmd u8
//!   5 SUCCESS_HANDSHAKE
//!   6 ASYNC_EVENT         → cmd u8, ed u32, len u32, data(GPIO 事件:u64 ts_us + bool + 7 填充)

use std::io::{self, Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

const CMD_RUN_FOR: u8 = 1;
const CMD_GET_TIME: u8 = 2;
const CMD_GET_MACHINE: u8 = 3;
const CMD_GPIO: u8 = 5;
const CMD_SYSBUS: u8 = 6;

/// server 端定義的版本表:(命令, 版本)。GPIO 是 v1,其餘 v0。
const COMMAND_VERSIONS: [(u8, u8); 6] = [(1, 0), (2, 0), (3, 0), (4, 0), (5, 1), (6, 0)];

pub struct Renode {
    s: TcpStream,
}

#[derive(Clone, Copy)]
pub struct Machine(i32);

#[derive(Clone, Copy)]
pub struct Gpio(i32);

#[derive(Clone, Copy)]
pub struct BusCtx(i32);

fn err(msg: String) -> io::Error {
    io::Error::new(io::ErrorKind::Other, msg)
}

impl Renode {
    /// 連線;server 還沒起來就重試,最多 `retry` 秒。
    /// 不要用會送資料的探埠(bash 的 `echo >/dev/tcp/...`):server 會把那個 byte 當握手。
    pub fn connect_retry<A: ToSocketAddrs + Clone>(addr: A, retry: Duration) -> io::Result<Renode> {
        let t0 = std::time::Instant::now();
        loop {
            match Renode::connect(addr.clone()) {
                Ok(r) => return Ok(r),
                Err(e) if t0.elapsed() < retry && e.kind() == io::ErrorKind::ConnectionRefused => {
                    std::thread::sleep(Duration::from_millis(250));
                }
                Err(e) => return Err(e),
            }
        }
    }

    pub fn connect<A: ToSocketAddrs>(addr: A) -> io::Result<Renode> {
        let s = TcpStream::connect(addr)?;
        s.set_nodelay(true)?;
        s.set_read_timeout(Some(Duration::from_secs(30)))?;
        let mut r = Renode { s };
        r.handshake()?;
        Ok(r)
    }

    fn handshake(&mut self) -> io::Result<()> {
        let mut b = Vec::with_capacity(2 + COMMAND_VERSIONS.len() * 2);
        b.extend_from_slice(&(COMMAND_VERSIONS.len() as u16).to_le_bytes());
        for (c, v) in COMMAND_VERSIONS {
            b.push(c);
            b.push(v);
        }
        self.s.write_all(&b)?;
        let code = self.read_u8()?;
        if code == 1 {
            let n = self.read_u32()? as usize;
            let m = self.read_bytes(n)?;
            return Err(err(format!("handshake FATAL: {}", String::from_utf8_lossy(&m))));
        }
        if code != 5 {
            return Err(err(format!("handshake: 回應碼 {} 不是 5", code)));
        }
        Ok(())
    }

    fn send(&mut self, cmd: u8, data: &[u8]) -> io::Result<()> {
        let mut b = Vec::with_capacity(7 + data.len());
        b.extend_from_slice(b"RE");
        b.push(cmd);
        b.extend_from_slice(&(data.len() as u32).to_le_bytes());
        b.extend_from_slice(data);
        self.s.write_all(&b)
    }

    fn read_u8(&mut self) -> io::Result<u8> {
        let mut b = [0u8; 1];
        self.s.read_exact(&mut b)?;
        Ok(b[0])
    }

    fn read_u32(&mut self) -> io::Result<u32> {
        let mut b = [0u8; 4];
        self.s.read_exact(&mut b)?;
        Ok(u32::from_le_bytes(b))
    }

    fn read_bytes(&mut self, n: usize) -> io::Result<Vec<u8>> {
        let mut v = vec![0u8; n];
        self.s.read_exact(&mut v)?;
        Ok(v)
    }

    /// 讀一個回應;非同步事件(code 6)在這裡直接吃掉(本橋接不訂閱事件)。
    fn recv(&mut self, expect_cmd: u8) -> io::Result<Vec<u8>> {
        loop {
            let code = self.read_u8()?;
            match code {
                6 => {
                    let _cmd = self.read_u8()?;
                    let _ed = self.read_u32()?;
                    let n = self.read_u32()? as usize;
                    let _ = self.read_bytes(n)?;
                    continue;
                }
                1 => {
                    let n = self.read_u32()? as usize;
                    let m = self.read_bytes(n)?;
                    return Err(err(format!("FATAL: {}", String::from_utf8_lossy(&m))));
                }
                _ => {}
            }
            let cmd = self.read_u8()?;
            match code {
                0 => {
                    let n = self.read_u32()? as usize;
                    let m = self.read_bytes(n)?;
                    return Err(err(format!("命令 {} 失敗: {}", cmd, String::from_utf8_lossy(&m))));
                }
                2 => return Err(err(format!("命令 {} 無效", cmd))),
                3 => {
                    let n = self.read_u32()? as usize;
                    let d = self.read_bytes(n)?;
                    if cmd != expect_cmd {
                        return Err(err(format!("回應命令 {} 與送出 {} 不符", cmd, expect_cmd)));
                    }
                    return Ok(d);
                }
                4 => {
                    if cmd != expect_cmd {
                        return Err(err(format!("回應命令 {} 與送出 {} 不符", cmd, expect_cmd)));
                    }
                    return Ok(Vec::new());
                }
                _ => return Err(err(format!("未知回應碼 {}", code))),
            }
        }
    }

    fn call(&mut self, cmd: u8, data: &[u8]) -> io::Result<Vec<u8>> {
        self.send(cmd, data)?;
        self.recv(cmd)
    }

    /// 推進模擬 `us` 微秒;回來時模擬已暫停。
    pub fn run_for_us(&mut self, us: u64) -> io::Result<()> {
        self.call(CMD_RUN_FOR, &us.to_le_bytes())?;
        Ok(())
    }

    /// 目前虛擬時間(微秒)。
    pub fn time_us(&mut self) -> io::Result<u64> {
        let d = self.call(CMD_GET_TIME, &[0u8; 8])?;
        Ok(u64::from_le_bytes(d[..8].try_into().unwrap()))
    }

    pub fn machine(&mut self, name: &str) -> io::Result<Machine> {
        let mut b = Vec::new();
        b.extend_from_slice(&(name.len() as i32).to_le_bytes());
        b.extend_from_slice(name.as_bytes());
        let d = self.call(CMD_GET_MACHINE, &b)?;
        let md = i32::from_le_bytes(d[..4].try_into().unwrap());
        if md < 0 {
            return Err(err(format!("machine {name}: 無效描述子")));
        }
        Ok(Machine(md))
    }

    fn instance(&mut self, cmd: u8, m: Machine, name: &str) -> io::Result<i32> {
        let mut b = Vec::new();
        b.extend_from_slice(&(-1i32).to_le_bytes());
        b.extend_from_slice(&m.0.to_le_bytes());
        b.extend_from_slice(&(name.len() as i32).to_le_bytes());
        b.extend_from_slice(name.as_bytes());
        let d = self.call(cmd, &b)?;
        let id = i32::from_le_bytes(d[..4].try_into().unwrap());
        if id < 0 {
            return Err(err(format!("{name}: 無效實例描述子")));
        }
        Ok(id)
    }

    pub fn gpio(&mut self, m: Machine, name: &str) -> io::Result<Gpio> {
        Ok(Gpio(self.instance(CMD_GPIO, m, name)?))
    }

    /// 讀輸出腳(週邊的 Connections[pin].IsSet)。
    pub fn gpio_get(&mut self, g: Gpio, pin: i32) -> io::Result<bool> {
        let mut b = Vec::new();
        b.extend_from_slice(&g.0.to_le_bytes());
        b.push(0);
        b.extend_from_slice(&pin.to_le_bytes());
        let d = self.call(CMD_GPIO, &b)?;
        Ok(d.first().copied().unwrap_or(0) != 0)
    }

    /// 寫輸入腳(週邊的 OnGPIO(pin, state))。
    pub fn gpio_set(&mut self, g: Gpio, pin: i32, state: bool) -> io::Result<()> {
        let mut b = Vec::new();
        b.extend_from_slice(&g.0.to_le_bytes());
        b.push(1);
        b.extend_from_slice(&pin.to_le_bytes());
        b.push(state as u8);
        self.call(CMD_GPIO, &b)?;
        Ok(())
    }

    pub fn sysbus(&mut self, m: Machine) -> io::Result<BusCtx> {
        Ok(BusCtx(self.instance(CMD_SYSBUS, m, "sysbus")?))
    }

    pub fn read_u32_at(&mut self, ctx: BusCtx, addr: u64) -> io::Result<u32> {
        let mut b = Vec::new();
        b.extend_from_slice(&ctx.0.to_le_bytes());
        b.push(0); // read
        b.push(4); // double word
        b.extend_from_slice(&addr.to_le_bytes());
        b.extend_from_slice(&1u32.to_le_bytes());
        let d = self.call(CMD_SYSBUS, &b)?;
        Ok(u32::from_le_bytes(d[..4].try_into().unwrap()))
    }

    /// 一次讀 n 個連續的 u32(一個 RPC)。
    pub fn read_u32s_at(&mut self, ctx: BusCtx, addr: u64, n: u32) -> io::Result<Vec<u32>> {
        let mut b = Vec::new();
        b.extend_from_slice(&ctx.0.to_le_bytes());
        b.push(0);
        b.push(4);
        b.extend_from_slice(&addr.to_le_bytes());
        b.extend_from_slice(&n.to_le_bytes());
        let d = self.call(CMD_SYSBUS, &b)?;
        Ok(d.chunks_exact(4).map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]])).collect())
    }

    pub fn write_u32_at(&mut self, ctx: BusCtx, addr: u64, v: u32) -> io::Result<()> {
        let mut b = Vec::new();
        b.extend_from_slice(&ctx.0.to_le_bytes());
        b.push(1); // write
        b.push(4);
        b.extend_from_slice(&addr.to_le_bytes());
        b.extend_from_slice(&1u32.to_le_bytes());
        b.extend_from_slice(&v.to_le_bytes());
        self.call(CMD_SYSBUS, &b)?;
        Ok(())
    }
}
