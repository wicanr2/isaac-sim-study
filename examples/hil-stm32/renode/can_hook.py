# CAN <-> TCP: monitor-level IronPython, no C#, no kernel module.
# ASCII only in this file (IronPython inline rule; kept here for consistency).
#
# Wire format on TCP 3600, both directions, little-endian:
#   u32 id | u8 dlc | u8[8] data     (13 bytes per frame)
# Renode -> client: every frame the firmware puts on can1 (FrameSent event)
# client -> Renode: frames injected into can1 via OnFrameReceived()
#
# Serve one client at a time. Accept and reads are polled from the monitor
# thread by mc_can_poll (called by the bridge through the External Control
# run_for cadence is NOT possible, so we poll with a background thread).
import clr
import System
clr.AddReference("System.Net.Primitives")
clr.AddReference("System.Net.Sockets")
from System.Net import IPAddress
from System.Net.Sockets import TcpListener
from System.Threading import Thread, ThreadStart
from Antmicro.Renode.Core.CAN import CANMessageFrame

_can_state = {"listener": None, "client": None, "stream": None, "thread": None,
              "sent": 0, "injected": 0, "running": False}

def _pack(frame):
    b = System.Array.CreateInstance(System.Byte, 13)
    fid = int(frame.Id)
    for i in range(4):
        b[i] = (fid >> (8 * i)) & 0xFF
    data = frame.Data
    b[4] = len(data)
    for i in range(8):
        b[5 + i] = data[i] if i < len(data) else 0
    return b

def _on_frame_sent(frame):
    _can_state["sent"] += 1
    s = _can_state["stream"]
    if s is None:
        return
    try:
        b = _pack(frame)
        s.Write(b, 0, 13)
        s.Flush()
    except Exception as e:
        _can_state["stream"] = None

def _rx_loop():
    # background thread: accept one client, then read 13-byte frames forever
    can1 = self.Machine["sysbus.can1"]
    lst = _can_state["listener"]
    while _can_state["running"]:
        try:
            client = lst.AcceptTcpClient()
        except Exception:
            break
        stream = client.GetStream()
        _can_state["client"] = client
        _can_state["stream"] = stream
        buf = System.Array.CreateInstance(System.Byte, 13)
        while _can_state["running"]:
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
            fid = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24)
            dlc = buf[4]
            raw = [int(buf[5 + i]) for i in range(dlc)]
            data = System.Array[System.Byte](raw)
            frame = CANMessageFrame(System.UInt32(fid), data, False, False, False, False)
            can1.OnFrameReceived(frame)
            _can_state["injected"] += 1
        _can_state["stream"] = None
        try:
            client.Close()
        except Exception:
            pass

def mc_can_hook_start(port=3600):
    can1 = self.Machine["sysbus.can1"]
    can1.FrameSent += _on_frame_sent
    lst = TcpListener(IPAddress.Any, int(port))
    lst.Start()
    _can_state["listener"] = lst
    _can_state["running"] = True
    t = Thread(ThreadStart(_rx_loop))
    t.IsBackground = True
    t.Start()
    _can_state["thread"] = t
    print "can_hook: listening on %d, can1.FrameSent hooked" % int(port)

def mc_can_hook_stats():
    print "can_hook: sent=%d injected=%d client=%s" % (
        _can_state["sent"], _can_state["injected"], _can_state["stream"] is not None)
