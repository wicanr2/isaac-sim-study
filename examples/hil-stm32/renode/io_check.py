# step 2 check helpers (monitor-level IronPython, ASCII only).
# Addresses come from the symbol file, not hard-coded: g_dbg / g_cfg move whenever the firmware changes.
import System
from Antmicro.Renode.Core.CAN import CANMessageFrame

SYM = "/w/firmware/build/hilctl.sym"
DBG_FIELDS = ["magic", "tick_ms", "ctrl_steps", "sp_l", "sp_r", "meas_l", "meas_r", "duty_l", "duty_r",
              "enc_l", "enc_r", "enc_frames", "cmd_frames", "bad_crc", "flags", "init_err", "rx_overflow",
              "resets", "boot_csr", "ping_frames"]
CFG_FIELDS = ["magic", "kp_q8", "ki_q8", "accel_mm_s2", "ff_q8", "alpha_mrad_s2",
              "iwdg_ms", "hang_at_ms", "hb_timeout_ms", "stall_duty", "stall_ms", "safety_mask"]
FLAG_NAMES = ["ENABLED", "ESTOP", "CMD_STALE", "DRV_FAULT", "BUMPER", "STALL", "HB_LOST", "WDT_RESET"]

def _sym(name):
    for line in open(SYM):
        f = line.split()
        if len(f) == 3 and f[2] == name:
            return int(f[0], 16)
    raise Exception("symbol %s not in %s" % (name, SYM))

def _rd(addr):
    return int(self.Machine["sysbus"].ReadDoubleWord(System.UInt64(addr)))

def _s32(v):
    return v - (1 << 32) if v & 0x80000000 else v

def mc_dbg(field):
    v = _rd(_sym("g_dbg") + 4 * DBG_FIELDS.index(field))
    if field == "flags":
        names = [n for i, n in enumerate(FLAG_NAMES) if v & (1 << i)]
        print "    g_dbg.flags = 0x%02x %s" % (v, "|".join(names) or "-")
    elif field in ("magic", "boot_csr"):
        print "    g_dbg.%s = 0x%08x" % (field, v)
    else:
        print "    g_dbg.%s = %d" % (field, _s32(v))

def mc_cfg_write(field, value):
    addr = _sym("g_cfg") + 4 * CFG_FIELDS.index(field)
    self.Machine["sysbus"].WriteDoubleWord(System.UInt64(addr), System.UInt32(int(value) & 0xFFFFFFFF))
    print "    g_cfg.%s <- %s" % (field, value)

def mc_uart1_feed(hexstr):
    u = self.Machine["sysbus.usart1"]
    for i in range(0, len(hexstr), 2):
        u.WriteChar(System.Byte(int(hexstr[i:i+2], 16)))
    print "uart1_feed: %d bytes" % (len(hexstr) / 2)

def mc_ping():
    # MSG_PING = 0x03, empty payload: A5 5A 00 03 crc16(03) = 0x41FF -> FF 41 (LSB first)
    mc_uart1_feed("A55A0003FF41")

def mc_can_feed(idhex, hexdata):
    can1 = self.Machine["sysbus.can1"]
    raw = [int(hexdata[i:i+2], 16) for i in range(0, len(hexdata), 2)]
    frame = CANMessageFrame(System.UInt32(int(idhex, 16)), System.Array[System.Byte](raw), False, False, False, False)
    can1.OnFrameReceived(frame)
    print "can_feed: id=0x%s dlc=%d" % (idhex, len(raw))

def mc_enc_add(dl, dr):
    # TIM2 / TIM4 CNT: what the encoder-mode counter would show after dl / dr more edges
    for base, d in ((0x40000000, int(dl)), (0x40000800, int(dr))):
        cnt = _rd(base + 0x24)
        self.Machine["sysbus"].WriteDoubleWord(System.UInt64(base + 0x24), System.UInt32((cnt + d) & 0xFFFF))

def mc_pin(pin, level):
    # PC<pin> input; the bridge/board drives it (pull-ups are on the board, Renode inputs default to 0)
    self.Machine["sysbus.gpioPortC"].OnGPIO(int(pin), bool(int(level)))
    print "    PC%s <- %s" % (pin, level)
