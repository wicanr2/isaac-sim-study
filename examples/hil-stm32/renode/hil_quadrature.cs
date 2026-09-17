//
// Quadrature feeder for the HIL bridge (docs/hil/37): turns a signed tick delta into A/B edges on an
// STM32 timer running in encoder mode. It is a .NET method so it can be queued into the machine's time
// domain (HandleTimeDomainEvent) and run on the emulation thread without entering IronPython there;
// a Python lambda in that position stalls the emulation while the hook's own thread is inside Python.
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Hil
{
    public class QuadratureFeeder
    {
        public QuadratureFeeder(IGPIOReceiver timer)
        {
            this.timer = timer;
        }

        // (A, B) walks 00 -> 10 -> 11 -> 01 for +1 counts (A leads B), the other way for -1.
        // Every step is one edge on one channel; encoder mode 3 counts each of them.
        public void Feed(int delta)
        {
            var step = delta > 0 ? 1 : 3;
            for(var i = 0; i < Math.Abs(delta); i++)
            {
                var old = Quad[phase];
                phase = (phase + step) % 4;
                var now = Quad[phase];
                var ch = old.Item1 != now.Item1 ? 0 : 1;
                timer.OnGPIO(ch, ch == 0 ? now.Item1 : now.Item2);
                Edges++;
            }
        }

        public long Edges { get; private set; }

        private int phase;
        private readonly IGPIOReceiver timer;

        private static readonly Tuple<bool, bool>[] Quad =
        {
            Tuple.Create(false, false), Tuple.Create(true, false), Tuple.Create(true, true), Tuple.Create(false, true),
        };
    }

    // Continuous encoder for realtime (docs/hil/37 sec. 3): instead of edges or a CNT write at the step
    // boundary, the CNT register of an encoder-mode timer is computed at the instant the CPU reads it,
    // from an anchor that the bridge updates once per step:
    //
    //     pos(t) = anchor + rate * (t - tA) + err * min(1, (t - tA) / tau)
    //
    // rate is the plant's wheel speed (ticks per second); err = plantTicks - pos(tA) is the tracking
    // error at the update, paid back over tau so the position stays continuous (no jump at the update,
    // no overshoot if the next update comes late). t is the virtual time of the read (cpu.SyncTime()
    // first, so it is the current instruction, not the last quantum boundary). A firmware write to CNT
    // re-bases an offset, so CNT = 0 at init (also after an IWDG reset) works as on a real board.
    // Edges are not generated: at 8k edges/s each edge is a timer event that realtime cannot afford.
    public class ContinuousEncoder
    {
        public ContinuousEncoder(Machine machine, IBusPeripheral timer, double tauSeconds)
        {
            this.machine = machine;
            this.tau = tauSeconds;
            var cnt = new Antmicro.Renode.Core.Range(CntOffset, 4);
            machine.SystemBus.SetHookAfterPeripheralRead<uint>(timer, (value, offset) => Read(), cnt);
            machine.SystemBus.SetHookBeforePeripheralWrite<uint>(timer, (value, offset) => { Rebase(value); return value; }, cnt);
        }

        // Virtual time (microseconds) at which the plant was sampled; the next Update() anchors there instead of at
        // the instant the update is handled. In lockstep both are the same; in realtime the update lands a few ms
        // later and that latency jitters (2-7 ms per 5 ms wall step), which the error term would turn into speed.
        public void SetSampleTime(long virtualMicros)
        {
            lock(sync)
            {
                sampleTime = virtualMicros / 1e6;
            }
        }

        // packed = (int32 plantTicks << 32) | uint32(rate in milli-ticks per second)
        public void Update(long packed)
        {
            var ticks = (int)(packed >> 32);
            var rateMilli = unchecked((int)(packed & 0xFFFFFFFF));
            lock(sync)
            {
                var t = sampleTime ?? Now();
                sampleTime = null;
                var c = Position(t);
                var e = ticks - c;
                if(Math.Abs(e) > MaxAbsError) { MaxAbsError = Math.Abs(e); }
                LastError = e;
                anchor = c;
                tA = t;
                err = e;
                rate = rateMilli / 1000.0;
                Updates++;
            }
        }

        public double MaxAbsError { get; private set; }
        public double LastError { get; private set; }
        public long Updates { get; private set; }
        public long Reads { get; private set; }

        private uint Read()
        {
            lock(sync)
            {
                Reads++;
                var v = (long)Math.Floor(Position(Now()) + offset + 0.5);
                return (uint)(v & 0xFFFF);
            }
        }

        private void Rebase(uint value)
        {
            lock(sync)
            {
                offset = (value & 0xFFFF) - Position(Now());
            }
        }

        private double Position(double t)
        {
            var dt = Math.Max(0.0, t - tA);
            var k = tau > 0 ? Math.Min(1.0, dt / tau) : 1.0;
            return anchor + rate * dt + err * k;
        }

        private double Now()
        {
            if(machine.SystemBus.TryGetCurrentCPU(out var cpu) && cpu.OnPossessedThread)
            {
                cpu.SyncTime();
            }
            return machine.ElapsedVirtualTime.TimeElapsed.TotalSeconds;
        }

        private const ulong CntOffset = 0x24;
        private readonly Machine machine;
        private readonly double tau;
        private readonly object sync = new object();
        private double anchor, rate, err, tA, offset;
        private double? sampleTime;
    }

    // IMU gyroscope feeder (docs/hil/38 sec. 1.2, slip detection): sets AngularRateZ (dps) of an I2C gyroscope model
    // from the bridge's milli-dps. Reflection, because the sensor type may itself be compiled at runtime.
    public class AngularRateFeeder
    {
        public AngularRateFeeder(object sensor)
        {
            this.sensor = sensor;
            property = sensor.GetType().GetProperty("AngularRateZ");
            if(property == null)
            {
                throw new ArgumentException("sensor has no AngularRateZ property");
            }
        }

        public void Set(int milliDps)
        {
            property.SetValue(sensor, (decimal)milliDps / 1000m);
            Updates++;
        }

        public long Updates { get; private set; }

        private readonly object sensor;
        private readonly System.Reflection.PropertyInfo property;
    }
}
