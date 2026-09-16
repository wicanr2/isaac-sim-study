//! CAN 的第三條路(docs/hil/37 篇 §4 的 ②③):Renode `CreateSocketCANBridge` 接一個 Linux CAN 介面,
//! 橋接用 SocketCAN(PF_CAN / CAN_RAW)收發。std 沒有 AF_CAN,這裡直接宣告需要的幾個 libc 符號
//! (std 本來就連 libc),零 crate。版面照 <linux/can.h>,只支援 x86_64 / aarch64 Linux。
//!
//! 與 hook 那條路的差別:**沒有 ack**。寫進 socket 之後,Renode 的 SocketCANBridge 在自己的
//! 執行緒 read 到、再經 CANHub 排進機器的時間域——橋接不知道它落在哪一步。

use std::io;

const PF_CAN: i32 = 29;
const SOCK_RAW: i32 = 3;
const CAN_RAW: i32 = 1;
const SIOCGIFINDEX: u64 = 0x8933;
const F_GETFL: i32 = 3;
const F_SETFL: i32 = 4;
const O_NONBLOCK: i32 = 0o4000;
const EAGAIN: i32 = 11;

extern "C" {
    fn socket(domain: i32, ty: i32, protocol: i32) -> i32;
    fn ioctl(fd: i32, req: u64, ...) -> i32;
    fn bind(fd: i32, addr: *const u8, len: u32) -> i32;
    fn read(fd: i32, buf: *mut u8, n: usize) -> isize;
    fn write(fd: i32, buf: *const u8, n: usize) -> isize;
    fn close(fd: i32) -> i32;
    fn fcntl(fd: i32, cmd: i32, ...) -> i32;
}

/// struct can_frame:u32 can_id | u8 len | u8 pad | u8 res0 | u8 len8_dlc | u8 data[8] = 16 bytes
const FRAME_LEN: usize = 16;

pub struct SocketCan {
    fd: i32,
    pub sent: u32,
    pub received: u32,
}

impl SocketCan {
    pub fn open(ifname: &str) -> io::Result<Self> {
        let fd = unsafe { socket(PF_CAN, SOCK_RAW, CAN_RAW) };
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        // struct ifreq:char ifr_name[16] + union(16 bytes);SIOCGIFINDEX 把 ifr_ifindex(i32)寫在 offset 16
        let mut ifr = [0u8; 40];
        let name = ifname.as_bytes();
        if name.len() >= 16 {
            return Err(io::Error::new(io::ErrorKind::InvalidInput, "介面名太長"));
        }
        ifr[..name.len()].copy_from_slice(name);
        if unsafe { ioctl(fd, SIOCGIFINDEX, ifr.as_mut_ptr()) } < 0 {
            let e = io::Error::last_os_error();
            unsafe { close(fd) };
            return Err(io::Error::new(e.kind(), format!("SIOCGIFINDEX {ifname}: {e}")));
        }
        let ifindex = i32::from_ne_bytes([ifr[16], ifr[17], ifr[18], ifr[19]]);
        // struct sockaddr_can:u16 can_family | pad | i32 can_ifindex | union 16 bytes = 24 bytes
        let mut addr = [0u8; 24];
        addr[..2].copy_from_slice(&(PF_CAN as u16).to_ne_bytes());
        addr[4..8].copy_from_slice(&ifindex.to_ne_bytes());
        if unsafe { bind(fd, addr.as_ptr(), addr.len() as u32) } < 0 {
            let e = io::Error::last_os_error();
            unsafe { close(fd) };
            return Err(io::Error::new(e.kind(), format!("bind {ifname}: {e}")));
        }
        let fl = unsafe { fcntl(fd, F_GETFL) };
        if fl < 0 || unsafe { fcntl(fd, F_SETFL, fl | O_NONBLOCK) } < 0 {
            let e = io::Error::last_os_error();
            unsafe { close(fd) };
            return Err(e);
        }
        Ok(Self { fd, sent: 0, received: 0 })
    }

    pub fn send(&mut self, id: u32, data: &[u8]) -> io::Result<()> {
        let mut f = [0u8; FRAME_LEN];
        f[..4].copy_from_slice(&id.to_ne_bytes());
        f[4] = data.len().min(8) as u8;
        f[8..8 + data.len().min(8)].copy_from_slice(&data[..data.len().min(8)]);
        let n = unsafe { write(self.fd, f.as_ptr(), FRAME_LEN) };
        if n != FRAME_LEN as isize {
            return Err(io::Error::last_os_error());
        }
        self.sent += 1;
        Ok(())
    }

    /// 把目前收得到的訊框全部拿出來(非阻塞)。自己送出去的訊框預設不會回送(CAN_RAW_RECV_OWN_MSGS 關)。
    pub fn drain(&mut self) -> io::Result<Vec<(u32, Vec<u8>)>> {
        let mut out = Vec::new();
        loop {
            let mut f = [0u8; FRAME_LEN];
            let n = unsafe { read(self.fd, f.as_mut_ptr(), FRAME_LEN) };
            if n < 0 {
                let e = io::Error::last_os_error();
                if e.raw_os_error() == Some(EAGAIN) {
                    return Ok(out);
                }
                return Err(e);
            }
            if n as usize != FRAME_LEN {
                return Err(io::Error::new(io::ErrorKind::UnexpectedEof, format!("read 回 {n} bytes")));
            }
            let id = u32::from_ne_bytes([f[0], f[1], f[2], f[3]]) & 0x1FFF_FFFF;
            let len = (f[4] as usize).min(8);
            out.push((id, f[8..8 + len].to_vec()));
            self.received += 1;
        }
    }
}

impl Drop for SocketCan {
    fn drop(&mut self) {
        unsafe { close(self.fd) };
    }
}
