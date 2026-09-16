//! 上位協定(與 firmware/proto.h 同一份版面)+ CRC-16/MODBUS。
//!
//! 框包:`A5 5A len type payload crc_lo crc_hi`,CRC 算 type+payload。

pub const SYNC0: u8 = 0xA5;
pub const SYNC1: u8 = 0x5A;
pub const MSG_CMD_VEL: u8 = 0x01;
pub const MSG_ODOM: u8 = 0x02;
pub const MSG_PING: u8 = 0x03;
pub const MSG_PONG: u8 = 0x83;

pub fn crc16_modbus(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= b as u16;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xA001 } else { crc >> 1 };
        }
    }
    crc
}

pub fn encode(msg_type: u8, payload: &[u8]) -> Vec<u8> {
    let mut body = Vec::with_capacity(1 + payload.len());
    body.push(msg_type);
    body.extend_from_slice(payload);
    let crc = crc16_modbus(&body);
    let mut f = Vec::with_capacity(5 + payload.len());
    f.push(SYNC0);
    f.push(SYNC1);
    f.push(payload.len() as u8);
    f.extend_from_slice(&body);
    f.push((crc & 0xFF) as u8);
    f.push((crc >> 8) as u8);
    f
}

pub fn ping() -> Vec<u8> {
    encode(MSG_PING, &[])
}

pub fn cmd_vel(v_mm_s: i16, w_mrad_s: i16) -> Vec<u8> {
    let mut p = Vec::with_capacity(4);
    p.extend_from_slice(&v_mm_s.to_le_bytes());
    p.extend_from_slice(&w_mrad_s.to_le_bytes());
    encode(MSG_CMD_VEL, &p)
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Odom {
    pub seq: u16,
    pub t_ms: u32,
    pub x_mm: i32,
    pub y_mm: i32,
    pub th_mrad: i32,
    pub vl_mm_s: i16,
    pub vr_mm_s: i16,
    pub flags: u8,
}

/// odom_payload_t(packed):seq 2 + t 4 + x 4 + y 4 + th 4 + vl 2 + vr 2 + flags 1
pub const ODOM_LEN: usize = 23;

pub fn parse_odom(p: &[u8]) -> Option<Odom> {
    if p.len() != ODOM_LEN {
        return None;
    }
    Some(Odom {
        seq: u16::from_le_bytes([p[0], p[1]]),
        t_ms: u32::from_le_bytes([p[2], p[3], p[4], p[5]]),
        x_mm: i32::from_le_bytes([p[6], p[7], p[8], p[9]]),
        y_mm: i32::from_le_bytes([p[10], p[11], p[12], p[13]]),
        th_mrad: i32::from_le_bytes([p[14], p[15], p[16], p[17]]),
        vl_mm_s: i16::from_le_bytes([p[18], p[19]]),
        vr_mm_s: i16::from_le_bytes([p[20], p[21]]),
        flags: p[22],
    })
}

/// 接收狀態機:一次餵一段 bytes,回收齊且 CRC 正確的 (type, payload)。
#[derive(Default)]
pub struct Parser {
    state: u8,
    len: u8,
    msg_type: u8,
    payload: Vec<u8>,
    crc_lo: u8,
    pub bad_crc: u32,
}

impl Parser {
    pub fn feed(&mut self, bytes: &[u8], out: &mut Vec<(u8, Vec<u8>)>) {
        for &b in bytes {
            match self.state {
                0 => if b == SYNC0 { self.state = 1 },
                1 => self.state = if b == SYNC1 { 2 } else { 0 },
                2 => { self.len = b; self.payload.clear(); self.state = 3; }
                3 => { self.msg_type = b; self.state = if self.len > 0 { 4 } else { 5 }; }
                4 => { self.payload.push(b); if self.payload.len() >= self.len as usize { self.state = 5; } }
                5 => { self.crc_lo = b; self.state = 6; }
                _ => {
                    let mut body = vec![self.msg_type];
                    body.extend_from_slice(&self.payload);
                    let want = crc16_modbus(&body);
                    let got = self.crc_lo as u16 | ((b as u16) << 8);
                    self.state = 0;
                    if want == got {
                        out.push((self.msg_type, self.payload.clone()));
                    } else {
                        self.bad_crc += 1;
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_known_answer() {
        // CRC-16/MODBUS 的標準已知答案
        assert_eq!(crc16_modbus(b"123456789"), 0x4B37);
    }

    #[test]
    fn cmd_vel_bytes_match_firmware_side() {
        // 與第 2 步 io_check.resc 餵給韌體、韌體接受的那一串完全相同
        assert_eq!(cmd_vel(300, 0), vec![0xA5, 0x5A, 0x04, 0x01, 0x2C, 0x01, 0x00, 0x00, 0x40, 0x90]);
    }

    #[test]
    fn parser_roundtrip_and_bad_crc() {
        let mut p = Parser::default();
        let mut out = Vec::new();
        let mut f = cmd_vel(-100, 250);
        p.feed(&f, &mut out);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].0, MSG_CMD_VEL);
        let n = f.len();
        f[n - 1] ^= 0xFF;
        p.feed(&f, &mut out);
        assert_eq!(out.len(), 1);
        assert_eq!(p.bad_crc, 1);
    }
}
