//
// Copyright (c) 2010-2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//

using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.IRQControllers;
using Antmicro.Renode.Time;

using NUnit.Framework;

namespace Antmicro.Renode.UnitTests
{
    [TestFixture]
    public class NVIC_SysTickTests
    {
        [OneTimeSetUp]
        public void CreatePeripheral()
        {
            machine = new Machine();
            nvic = new NVIC(machine, SysTickFrequency);
        }

        [SetUp]
        public void PreparePeripheral()
        {
            nvic.Reset();
        }

        [Test]
        public void ShouldLoadReloadValueOnEnable()
        {
            // ARMv7-M B3.3.3: when ENABLE goes 0 -> 1 the counter loads RELOAD. This is the register
            // write order used by the FreeRTOS Cortex-M ports: CVR before RVR, then CSR.
            nvic.WriteDoubleWord(SysTickControl, 0);
            nvic.WriteDoubleWord(SysTickValue, 0);
            nvic.WriteDoubleWord(SysTickReload, Reload);
            nvic.WriteDoubleWord(SysTickControl, ControlClockSource | ControlEnable);
            AdvanceByTicks(720);
            var value = nvic.ReadDoubleWord(SysTickValue);
            Assert.That(value, Is.InRange(Reload - 720 - 2, Reload - 720 + 2), "first period must start from RELOAD, not from 0xFFFFFF");
        }

        [Test]
        public void ShouldKeepWorkingWithReloadWrittenBeforeValue()
        {
            nvic.WriteDoubleWord(SysTickControl, 0);
            nvic.WriteDoubleWord(SysTickReload, Reload);
            nvic.WriteDoubleWord(SysTickValue, 0);
            nvic.WriteDoubleWord(SysTickControl, ControlClockSource | ControlEnable);
            AdvanceByTicks(720);
            var value = nvic.ReadDoubleWord(SysTickValue);
            Assert.That(value, Is.InRange(Reload - 720 - 2, Reload - 720 + 2));
        }

        private void AdvanceByTicks(ulong ticks)
        {
            ((BaseClockSource)machine.ClockSource).Advance(TimeInterval.FromSeconds((double)ticks / SysTickFrequency));
        }

        private IMachine machine;
        private NVIC nvic;

        private const ulong SysTickFrequency = 72000000;
        private const uint Reload = 71999;
        private const long SysTickControl = 0x10;
        private const long SysTickReload = 0x14;
        private const long SysTickValue = 0x18;
        private const uint ControlEnable = 1u << 0;
        private const uint ControlClockSource = 1u << 2;
    }
}
