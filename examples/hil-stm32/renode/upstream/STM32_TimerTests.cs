//
// Copyright (c) 2010-2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//

using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

using NUnit.Framework;

namespace Antmicro.Renode.UnitTests
{
    [TestFixture]
    public class STM32_TimerTests
    {
        [OneTimeSetUp]
        public void CreatePeripheral()
        {
            machine = new Machine();
            timer = new STM32_Timer(machine, TimerFrequency, initialLimit: 0xFFFF);
        }

        [SetUp]
        public void PreparePeripheral()
        {
            timer.Reset();
        }

        [Test]
        public void ShouldCountArrPlusOneTicksPerPeriod()
        {
            // RM0090 18.3.1: the counter counts from 0 to ARR inclusive, so one period is ARR + 1 ticks.
            // With ARR = 99, exactly 10000 ticks must land back on 0; a period of ARR ticks would read 1.
            timer.WriteDoubleWord((long)Registers.AutoReload, 99);
            timer.WriteDoubleWord((long)Registers.Control1, ControlCounterEnable);
            AdvanceByTicks(10000);
            Assert.AreEqual(0, timer.ReadDoubleWord((long)Registers.Counter));
        }

        [Test]
        public void ShouldDrivePwmOutputOnEnable()
        {
            // PWM mode 1 with CNT < CCR1 right after CEN: the output is active immediately,
            // it does not wait for the first overflow.
            SetupPwmChannel1(arr: 999, ccr: 500, preload: false);
            AdvanceByTicks(200);
            Assert.True(timer.Connections[0].IsSet);
        }

        [Test]
        public void ShouldApplyPreloadedCompareValueOnUpdateEvent()
        {
            // RM0090 18.4.7: with OC1PE set, a CCR1 write goes to the preload register and is
            // transferred to the active register at the next update event. Readback returns the
            // preloaded value.
            SetupPwmChannel1(arr: 999, ccr: 500, preload: true);
            AdvanceByTicks(200);
            Assert.True(timer.Connections[0].IsSet);

            timer.WriteDoubleWord((long)Registers.CaptureOrCompare1, 100);
            Assert.True(timer.Connections[0].IsSet, "output must not change before the update event");
            Assert.AreEqual(100, timer.ReadDoubleWord((long)Registers.CaptureOrCompare1));

            AdvanceByTicks(1000);
            // CNT is now 200 again, above the new CCR1 of 100
            Assert.AreEqual(200, timer.ReadDoubleWord((long)Registers.Counter));
            Assert.False(timer.Connections[0].IsSet);
        }

        [Test]
        public void ShouldApplyCompareValueImmediatelyWithoutPreload()
        {
            SetupPwmChannel1(arr: 999, ccr: 500, preload: false);
            AdvanceByTicks(200);
            Assert.True(timer.Connections[0].IsSet);

            timer.WriteDoubleWord((long)Registers.CaptureOrCompare1, 100);
            AdvanceByTicks(1);
            Assert.False(timer.Connections[0].IsSet);
        }

        private void SetupPwmChannel1(uint arr, uint ccr, bool preload)
        {
            timer.WriteDoubleWord((long)Registers.AutoReload, arr);
            timer.WriteDoubleWord((long)Registers.CaptureOrCompare1, ccr);
            timer.WriteDoubleWord((long)Registers.CaptureOrCompareMode1, CaptureCompareMode1PwmMode1 | (preload ? CaptureCompareMode1Preload : 0u));
            timer.WriteDoubleWord((long)Registers.CaptureOrCompareEnable, CaptureCompareEnableChannel1);
            timer.WriteDoubleWord((long)Registers.Control1, ControlCounterEnable);
        }

        private void AdvanceByTicks(ulong ticks)
        {
            ((BaseClockSource)machine.ClockSource).Advance(TimeInterval.FromSeconds((double)ticks / TimerFrequency));
        }

        private IMachine machine;
        private STM32_Timer timer;

        private const ulong TimerFrequency = 10000000;
        private const uint ControlCounterEnable = 1u << 0;
        private const uint CaptureCompareMode1PwmMode1 = 6u << 4;
        private const uint CaptureCompareMode1Preload = 1u << 3;
        private const uint CaptureCompareEnableChannel1 = 1u << 0;

        private enum Registers
        {
            Control1 = 0x00,
            CaptureOrCompareMode1 = 0x18,
            CaptureOrCompareEnable = 0x20,
            Counter = 0x24,
            AutoReload = 0x2C,
            CaptureOrCompare1 = 0x34,
        }
    }
}
