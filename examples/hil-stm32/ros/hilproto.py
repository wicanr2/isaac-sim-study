"""上位協定(firmware/proto.h 的同一份版面)的 Python 實作:框包、CRC-16/MODBUS、odom 解析。

只用標準庫;ROS 2 節點與任何想直接對橋接(或 Renode 的 3456 socket terminal)講話的程式共用。
"""
import struct

SYNC0, SYNC1 = 0xA5, 0x5A
MSG_CMD_VEL = 0x01
MSG_ODOM = 0x02
MSG_PING = 0x03
MSG_PONG = 0x83

_ODOM = struct.Struct("<HIiiihhB")  # seq t_ms x_mm y_mm th_mrad vl vr flags = 23 bytes
ODOM_LEN = _ODOM.size


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def encode(msg_type: int, payload: bytes = b"") -> bytes:
    body = bytes([msg_type]) + payload
    crc = crc16_modbus(body)
    return bytes([SYNC0, SYNC1, len(payload)]) + body + bytes([crc & 0xFF, crc >> 8])


def cmd_vel(v_mm_s: int, w_mrad_s: int) -> bytes:
    return encode(MSG_CMD_VEL, struct.pack("<hh", int(v_mm_s), int(w_mrad_s)))


def ping() -> bytes:
    """心跳:韌體 heartbeat_timeout_ms 沒收到就把命令降到 0(flags HB_LOST)。"""
    return encode(MSG_PING)


# odom / CAN 0x201 的 flags 位元(firmware/proto.h)
FLAG_NAMES = {1 << 0: "ENABLED", 1 << 1: "ESTOP", 1 << 2: "CMD_STALE", 1 << 3: "DRV_FAULT",
              1 << 4: "BUMPER", 1 << 5: "STALL", 1 << 6: "HB_LOST", 1 << 7: "WDT_RESET"}


def flag_names(flags: int) -> str:
    return "|".join(n for b, n in FLAG_NAMES.items() if flags & b) or "-"


def parse_odom(payload: bytes):
    """回 dict 或 None。"""
    if len(payload) != ODOM_LEN:
        return None
    seq, t_ms, x, y, th, vl, vr, flags = _ODOM.unpack(payload)
    return {"seq": seq, "t_ms": t_ms, "x_mm": x, "y_mm": y, "th_mrad": th,
            "vl_mm_s": vl, "vr_mm_s": vr, "flags": flags}


class Parser:
    """接收狀態機,與 bridge-rs/src/proto.rs 的 Parser 同一個狀態圖。"""

    def __init__(self):
        self.state = 0
        self.length = 0
        self.msg_type = 0
        self.payload = bytearray()
        self.crc_lo = 0
        self.bad_crc = 0

    def feed(self, data: bytes):
        out = []
        for b in data:
            s = self.state
            if s == 0:
                if b == SYNC0:
                    self.state = 1
            elif s == 1:
                self.state = 2 if b == SYNC1 else 0
            elif s == 2:
                self.length = b
                self.payload = bytearray()
                self.state = 3
            elif s == 3:
                self.msg_type = b
                self.state = 4 if self.length else 5
            elif s == 4:
                self.payload.append(b)
                if len(self.payload) >= self.length:
                    self.state = 5
            elif s == 5:
                self.crc_lo = b
                self.state = 6
            else:
                want = crc16_modbus(bytes([self.msg_type]) + bytes(self.payload))
                got = self.crc_lo | (b << 8)
                self.state = 0
                if want == got:
                    out.append((self.msg_type, bytes(self.payload)))
                else:
                    self.bad_crc += 1
        return out


if __name__ == "__main__":
    # 已知答案:與 Rust 側的單元測試相同
    assert crc16_modbus(b"123456789") == 0x4B37
    assert cmd_vel(300, 0) == bytes([0xA5, 0x5A, 0x04, 0x01, 0x2C, 0x01, 0x00, 0x00, 0x40, 0x90])
    p = Parser()
    f = bytearray(cmd_vel(-100, 250))
    assert len(p.feed(bytes(f))) == 1
    f[-1] ^= 0xFF
    assert p.feed(bytes(f)) == [] and p.bad_crc == 1
    print("hilproto: 3 checks ok")
