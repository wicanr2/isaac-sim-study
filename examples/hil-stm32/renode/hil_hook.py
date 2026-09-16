# hil_hook.py -- one TCP port for everything the bridge exchanges with the MCU's
# buses, served by Renode's own monitor-level IronPython. No C#, no kernel
# module, no extra process. ASCII only in this file (IronPython inline rule;
# kept here for consistency).
#
# Wire format on TCP <port> (default 3600), both directions, little-endian,
# fixed 13 bytes per record:
#
#     u32 id | u8 len | u8[8] data
#
#   id < 0xFFFF0000        CAN frame on can1 (len = DLC).
#                          bridge -> Renode : injected via can1.OnFrameReceived()
#                          Renode -> bridge : every frame the MCU sends (FrameSent)
#   id == 0xFFFF0001       UART bytes for the MCU (usart1.WriteChar), len = 1..8
#   id == 0xFFFF0002       UART bytes from the MCU (usart1.CharReceived), len = 1..8
#   id == 0xFFFF0003       BUS SNAPSHOT, Renode -> bridge, sent right after each CAN
#                          frame record: data = u32 TIM3_CCR1 | u32 TIM3_CCR2 read on the
#                          emulation thread at the instant the frame was sent. The bridge
#                          samples the bus only at step boundaries; a value that lives and
#                          dies inside one step is invisible there. This is how the bridge
#                          compares two independent channels at the same instant.
#   id == 0xFFFF0010       START, bridge -> Renode: let the emulation run freely (realtime mode).
#   id == 0xFFFF0011       PAUSE, bridge -> Renode: pause it again. Both are acked.
#   id == 0xFFFF00AC       ACK, Renode -> bridge, data[0] = running counter.
#                          Sent after each injected record (CAN or UART) has been
#                          handed to the peripheral. The bridge waits for it before
#                          the next run_for, which is what makes lockstep
#                          deterministic: the injection is complete, not "in flight".
#
# Why an ACK at all: Renode's built-in socket terminal and this hook both hand
# bytes to the peripheral from a host thread. Without an ACK the bridge cannot
# know whether the bytes landed before or after the emulation advanced.

import clr
import System
clr.AddReference("System.Net.Primitives")
clr.AddReference("System.Net.Sockets")
from System.Net import IPAddress
from System.Net.Sockets import TcpListener
from System.Threading import Thread, ThreadStart
from Antmicro.Renode.Core.CAN import CANMessageFrame
from Antmicro.Renode.Time import TimeDomainsManager

ID_UART_TO_MCU   = 0xFFFF0001
ID_UART_FROM_MCU = 0xFFFF0002
ID_BUS_SNAPSHOT  = 0xFFFF0003
ID_START         = 0xFFFF0010
ID_PAUSE         = 0xFFFF0011
ID_ACK           = 0xFFFF00AC
TIM3_CCR1        = 0x40000434
TIM3_CCR2        = 0x40000438

_st = {"listener": None, "stream": None, "thread": None, "running": False,
       "can_sent": 0, "can_injected": 0, "uart_out": 0, "uart_in": 0, "acks": 0,
       "lock": System.Object(), "trace": False}

def mc_hil_hook_trace(on=1):
    _st["trace"] = bool(int(on))

def _record(rid, data):
    b = System.Array.CreateInstance(System.Byte, 13)
    for i in range(4):
        b[i] = (rid >> (8 * i)) & 0xFF
    n = len(data)
    b[4] = n
    for i in range(8):
        b[5 + i] = int(data[i]) & 0xFF if i < n else 0
    return b

def _send(rec):
    s = _st["stream"]
    if s is None:
        return
    lock = _st["lock"]
    System.Threading.Monitor.Enter(lock)
    try:
        s.Write(rec, 0, 13)
        s.Flush()
    except Exception:
        _st["stream"] = None
    finally:
        System.Threading.Monitor.Exit(lock)

def _u32_bytes(v):
    return [(v >> (8 * i)) & 0xFF for i in range(4)]

def _on_can_sent(frame):
    _st["can_sent"] += 1
    _send(_record(int(frame.Id), [frame.Data[i] for i in range(len(frame.Data))]))
    sb = self.Machine["sysbus"]
    ccr1 = int(sb.ReadDoubleWord(TIM3_CCR1))
    ccr2 = int(sb.ReadDoubleWord(TIM3_CCR2))
    _send(_record(ID_BUS_SNAPSHOT, _u32_bytes(ccr1) + _u32_bytes(ccr2)))

_uart_buf = []
def _on_uart_byte(b):
    # coalesce up to 8 bytes per record; flush when full. A partial record is
    # flushed by mc_hil_flush (the bridge calls it via a zero-length UART_TO_MCU).
    _st["uart_out"] += 1
    _uart_buf.append(int(b))
    if len(_uart_buf) >= 8:
        _flush_uart()

def _flush_uart():
    if not _uart_buf:
        return
    chunk = _uart_buf[:]
    del _uart_buf[:]
    _send(_record(ID_UART_FROM_MCU, chunk))

def _ack():
    _st["acks"] = (_st["acks"] + 1) & 0xFF
    _send(_record(ID_ACK, [_st["acks"]]))

def _deliver(machine, handler, arg_type, arg):
    # When the emulation is running, a peripheral must not be poked from a foreign thread:
    # WriteChar / OnFrameReceived raise interrupts and touch the CPU. Renode's own externals
    # (socket terminals, CAN hubs) queue the call on the machine's time domain instead.
    # When paused (lockstep) the direct call is what makes the injection land before the
    # next run_for, so keep it.
    if emulationManager.CurrentEmulation.IsStarted:
        machine.HandleTimeDomainEvent[arg_type](System.Action[arg_type](handler), arg,
                                                TimeDomainsManager.Instance.GetEffectiveVirtualTimeStamp())
    else:
        handler(arg)

def _rx_loop():
    machine = self.Machine
    can1 = self.Machine["sysbus.can1"]
    usart1 = self.Machine["sysbus.usart1"]
    lst = _st["listener"]
    while _st["running"]:
        try:
            client = lst.AcceptTcpClient()
        except Exception:
            break
        # 13-byte records answered one at a time: with Nagle on, an ack queued behind
        # unacknowledged FrameSent records waits for the peer's delayed ACK (~40 ms).
        client.NoDelay = True
        stream = client.GetStream()
        _st["stream"] = stream
        buf = System.Array.CreateInstance(System.Byte, 13)
        while _st["running"]:
            got = 0
            while got < 13:
                try:
                    n = stream.Read(buf, got, 13 - got)
                except Exception:
                    n = 0
                if n <= 0:
                    break
                got += n
            if got < 13:
                break
            rid = int(buf[0]) | (int(buf[1]) << 8) | (int(buf[2]) << 16) | (int(buf[3]) << 24)
            n = int(buf[4])
            data = [int(buf[5 + i]) for i in range(n)]
            if rid == ID_START:
                emulationManager.CurrentEmulation.StartAll()
                print "hil_hook: START"
                _ack()
            elif rid == ID_PAUSE:
                emulationManager.CurrentEmulation.PauseAll()
                _ack()
            elif rid == ID_UART_TO_MCU:
                if n == 0:
                    _flush_uart()
                else:
                    if _st["trace"]:
                        print "hil_hook: uart_in %d bytes" % n
                    for x in data:
                        _deliver(machine, usart1.WriteChar, System.Byte, System.Byte(x))
                    _st["uart_in"] += n
                    if _st["trace"]:
                        print "hil_hook: uart_in done"
                _ack()
            elif rid < 0xFFFF0000:
                frame = CANMessageFrame(System.UInt32(rid), System.Array[System.Byte](data), False, False, False, False)
                _deliver(machine, can1.OnFrameReceived, CANMessageFrame, frame)
                _st["can_injected"] += 1
                _ack()
        _st["stream"] = None
        try:
            client.Close()
        except Exception:
            pass

def mc_hil_hook_start(port=3600, hook_can="True"):
    # hook_can False: CAN goes through CreateSocketCANBridge instead (docs/hil/37 sec. 4 path 2/3);
    # the hook then carries UART only. Monitor passes the flag as a string.
    can1 = self.Machine["sysbus.can1"]
    usart1 = self.Machine["sysbus.usart1"]
    _st["hook_can"] = str(hook_can).lower() == "true"
    if _st["hook_can"]:
        can1.FrameSent += _on_can_sent
    usart1.CharReceived += _on_uart_byte
    lst = TcpListener(IPAddress.Any, int(port))
    lst.Start()
    _st["listener"] = lst
    _st["running"] = True
    t = Thread(ThreadStart(_rx_loop))
    t.IsBackground = True
    t.Start()
    _st["thread"] = t
    print "hil_hook: listening on %d; %susart1.CharReceived hooked" % (int(port), "can1.FrameSent + " if _st["hook_can"] else "")

def mc_hil_hook_stats():
    print "hil_hook: can_sent=%d can_injected=%d uart_out=%d uart_in=%d acks=%d client=%s" % (
        _st["can_sent"], _st["can_injected"], _st["uart_out"], _st["uart_in"], _st["acks"],
        _st["stream"] is not None)
