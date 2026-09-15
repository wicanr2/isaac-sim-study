# step 2 check helpers (monitor-level IronPython, ASCII only)
import System
from Antmicro.Renode.Core.CAN import CANMessageFrame

def mc_uart1_feed(hexstr):
    u = self.Machine["sysbus.usart1"]
    for i in range(0, len(hexstr), 2):
        u.WriteChar(System.Byte(int(hexstr[i:i+2], 16)))
    print "uart1_feed: %d bytes" % (len(hexstr) / 2)

def mc_can_feed(idhex, hexdata):
    can1 = self.Machine["sysbus.can1"]
    raw = [int(hexdata[i:i+2], 16) for i in range(0, len(hexdata), 2)]
    frame = CANMessageFrame(System.UInt32(int(idhex, 16)), System.Array[System.Byte](raw), False, False, False, False)
    can1.OnFrameReceived(frame)
    print "can_feed: id=0x%s dlc=%d" % (idhex, len(raw))
