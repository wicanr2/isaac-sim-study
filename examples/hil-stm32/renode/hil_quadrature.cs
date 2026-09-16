//
// Quadrature feeder for the HIL bridge (docs/hil/37): turns a signed tick delta into A/B edges on an
// STM32 timer running in encoder mode. It is a .NET method so it can be queued into the machine's time
// domain (HandleTimeDomainEvent) and run on the emulation thread without entering IronPython there;
// a Python lambda in that position stalls the emulation while the hook's own thread is inside Python.
//
using System;
using Antmicro.Renode.Core;

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
}
