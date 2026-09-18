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
ID_ENCODER_STEPS = 0xFFFF0020   # i32 dl, i32 dr: quadrature counts to feed TIM2 (left) / TIM4 (right)
ID_ENC_CONT_CFG  = 0xFFFF0021   # i32 tau_us: install the continuous encoders on TIM2/TIM4 (hil_quadrature.cs)
ID_ENC_CONT_L    = 0xFFFF0022   # i32 plant ticks, i32 rate (milli-ticks/s): update the left continuous encoder
ID_ENC_CONT_R    = 0xFFFF0023   # same, right
ID_ENC_CONT_TIME = 0xFFFF0025   # u64 virtual microseconds at which the plant was sampled (anchor time of the next updates)
ID_GYRO_Z        = 0xFFFF0024   # i32 milli-dps: angular rate Z of the IMU gyroscope (sysbus.i2c3.gyro)
ID_ACCEL_X       = 0xFFFF0026   # i32 micro-g: forward acceleration of the IMU accelerometer (sysbus.i2c3.accel)
ID_I2C_RESET     = 0xFFFF0027   # no payload: reset the I2C3 controller (fault injection: bus dies mid-run)
ID_ACK           = 0xFFFF00AC
TIM3_CCR1        = 0x40000434
TIM3_CCR2        = 0x40000438

_st = {"listener": None, "stream": None, "thread": None, "running": False,
       "can_sent": 0, "can_injected": 0, "uart_out": 0, "uart_in": 0, "acks": 0,
       "enc_edges": 0, "lock": System.Object(), "trace": False}

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

# Encoder feed: the quadrature edges are generated by a .NET helper (hil_quadrature.cs, loaded with
# `i @` before this hook) so that, when the machine is running, the work queued into the time domain
# runs on the emulation thread without entering IronPython there. A Python lambda in that position
# stalled the emulation as long as this hook's thread was inside Python (2026-09-16).
_feeders = {}

def _feeder(key, timer):
    if key not in _feeders:
        asms = [a for a in System.AppDomain.CurrentDomain.GetAssemblies() if a.GetType("Antmicro.Renode.Hil.QuadratureFeeder") is not None]
        if not asms:
            raise Exception("hil_quadrature.cs not loaded (i @/w/renode/hil_quadrature.cs before hil_hook.py)")
        T = asms[0].GetType("Antmicro.Renode.Hil.QuadratureFeeder")
        _feeders[key] = System.Activator.CreateInstance(T, System.Array[System.Object]([timer]))
    return _feeders[key]

def _enc_feed(machine, timer, key, delta):
    if delta == 0:
        return
    fd = _feeder(key, timer)
    if emulationManager.CurrentEmulation.IsStarted:
        # one time-domain event per (wheel, step); all edges of the step land at the same virtual instant
        machine.HandleTimeDomainEvent[System.Int32](System.Action[System.Int32](fd.Feed), System.Int32(delta),
                                                    TimeDomainsManager.Instance.GetEffectiveVirtualTimeStamp())
    else:
        fd.Feed(int(delta))
    _st["enc_edges"] += abs(int(delta))

# Continuous encoder (docs/hil/37 sec. 3): CNT computed at read time from an anchor updated once per step.
_conts = {}

def _cont_install(machine, tim_left, tim_right, tau_us):
    asms = [a for a in System.AppDomain.CurrentDomain.GetAssemblies() if a.GetType("Antmicro.Renode.Hil.ContinuousEncoder") is not None]
    if not asms:
        raise Exception("hil_quadrature.cs not loaded (i @/w/renode/hil_quadrature.cs before hil_hook.py)")
    T = asms[0].GetType("Antmicro.Renode.Hil.ContinuousEncoder")
    for key, tim in (("left", tim_left), ("right", tim_right)):
        if key not in _conts:
            _conts[key] = System.Activator.CreateInstance(T, System.Array[System.Object]([machine, tim, System.Double(tau_us / 1e6)]))
    print "hil_hook: continuous encoders on timer2/timer4, tau %d us" % tau_us

def _cont_update(machine, key, ticks, rate_milli):
    enc = _conts[key]
    packed = System.Int64((int(ticks) << 32) | (int(rate_milli) & 0xFFFFFFFF))
    if emulationManager.CurrentEmulation.IsStarted:
        machine.HandleTimeDomainEvent[System.Int64](System.Action[System.Int64](enc.Update), packed,
                                                    TimeDomainsManager.Instance.GetEffectiveVirtualTimeStamp())
    else:
        enc.Update(packed)

def _cont_time(machine, micros):
    for key in ("left", "right"):
        enc = _conts[key]
        if emulationManager.CurrentEmulation.IsStarted:
            machine.HandleTimeDomainEvent[System.Int64](System.Action[System.Int64](enc.SetSampleTime), System.Int64(micros),
                                                        TimeDomainsManager.Instance.GetEffectiveVirtualTimeStamp())
        else:
            enc.SetSampleTime(System.Int64(micros))

def mc_hil_enc_stats():
    for key in ("left", "right"):
        if key in _conts:
            e = _conts[key]
            print "hil_hook: cont %s updates=%d reads=%d max_abs_err=%.2f last_err=%.2f" % (key, e.Updates, e.Reads, e.MaxAbsError, e.LastError)

# IMU gyroscope: the property is set by a .NET helper for the same reason as the encoder feeder
_gyro = {}

_accel = {}

def _accel_set(machine, micro_g):
    if "feeder" not in _accel:
        sensor = self.Machine["sysbus.i2c3.accel"]
        asms = [a for a in System.AppDomain.CurrentDomain.GetAssemblies() if a.GetType("Antmicro.Renode.Hil.AccelerationFeeder") is not None]
        T = asms[0].GetType("Antmicro.Renode.Hil.AccelerationFeeder")
        _accel["feeder"] = System.Activator.CreateInstance(T, System.Array[System.Object]([sensor]))
    fd = _accel["feeder"]
    if emulationManager.CurrentEmulation.IsStarted:
        machine.HandleTimeDomainEvent[System.Int32](System.Action[System.Int32](fd.Set), System.Int32(micro_g),
                                                    TimeDomainsManager.Instance.GetEffectiveVirtualTimeStamp())
    else:
        fd.Set(int(micro_g))

def _i2c_reset(machine):
    dev = self.Machine["sysbus.i2c3"]
    if emulationManager.CurrentEmulation.IsStarted:
        machine.HandleTimeDomainEvent[System.Int32](System.Action[System.Int32](lambda _: dev.Reset()), System.Int32(0),
                                                    TimeDomainsManager.Instance.GetEffectiveVirtualTimeStamp())
    else:
        dev.Reset()

def _gyro_set(machine, milli_dps):
    if "feeder" not in _gyro:
        sensor = self.Machine["sysbus.i2c3.gyro"]
        asms = [a for a in System.AppDomain.CurrentDomain.GetAssemblies() if a.GetType("Antmicro.Renode.Hil.AngularRateFeeder") is not None]
        T = asms[0].GetType("Antmicro.Renode.Hil.AngularRateFeeder")
        _gyro["feeder"] = System.Activator.CreateInstance(T, System.Array[System.Object]([sensor]))
    fd = _gyro["feeder"]
    if emulationManager.CurrentEmulation.IsStarted:
        machine.HandleTimeDomainEvent[System.Int32](System.Action[System.Int32](fd.Set), System.Int32(milli_dps),
                                                    TimeDomainsManager.Instance.GetEffectiveVirtualTimeStamp())
    else:
        fd.Set(int(milli_dps))

def _i32(b):
    v = int(b[0]) | (int(b[1]) << 8) | (int(b[2]) << 16) | (int(b[3]) << 24)
    return v - (1 << 32) if v & 0x80000000 else v

def _rx_loop():
    machine = self.Machine
    can1 = self.Machine["sysbus.can1"]
    usart1 = self.Machine["sysbus.usart1"]
    tim_left = self.Machine["sysbus.timer2"]
    tim_right = self.Machine["sysbus.timer4"]
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
            elif rid == ID_ENC_CONT_TIME:
                lo = _i32(data[0:4]) & 0xFFFFFFFF; hi = _i32(data[4:8]) & 0xFFFFFFFF
                _cont_time(machine, (hi << 32) | lo)
                _ack()
            elif rid == ID_GYRO_Z:
                _gyro_set(machine, _i32(data[0:4]))
                _ack()
            elif rid == ID_ACCEL_X:
                _accel_set(machine, _i32(data[0:4]))
                _ack()
            elif rid == ID_I2C_RESET:
                _i2c_reset(machine)
                print "hil_hook: i2c3 reset"
                _ack()
            elif rid == ID_ENC_CONT_CFG:
                _cont_install(machine, tim_left, tim_right, _i32(data[0:4]))
                _ack()
            elif rid == ID_ENC_CONT_L or rid == ID_ENC_CONT_R:
                _cont_update(machine, "left" if rid == ID_ENC_CONT_L else "right", _i32(data[0:4]), _i32(data[4:8]))
                _ack()
            elif rid == ID_ENCODER_STEPS:
                dl = _i32(data[0:4]); dr = _i32(data[4:8])
                _enc_feed(machine, tim_left, "left", dl)
                _enc_feed(machine, tim_right, "right", dr)
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
    print "hil_hook: can_sent=%d can_injected=%d uart_out=%d uart_in=%d enc_edges=%d acks=%d client=%s" % (
        _st["can_sent"], _st["can_injected"], _st["uart_out"], _st["uart_in"], _st["enc_edges"], _st["acks"],
        _st["stream"] is not None)
