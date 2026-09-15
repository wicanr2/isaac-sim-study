*** Settings ***
Suite Setup                   Setup
Suite Teardown                Teardown
Test Setup                    Reset Emulation
Test Teardown                 Test Teardown
Resource                      ${RENODEKEYWORDS}

*** Variables ***
${REPL}                       @/w/renode/upstream/stm32f4-timerfix.repl
${TIM4}                       0x40000800
${TIM4_CR1}                   ${TIM4}
${TIM4_EGR}                   0x40000814
${TIM4_CCMR1}                 0x40000818
${TIM4_CCER}                  0x40000820
${TIM4_CNT}                   0x40000824
${TIM4_ARR}                   0x4000082C
${TIM4_CCR1}                  0x40000834

*** Keywords ***
Create Machine
    Execute Command           i @/w/renode/upstream/STM32_Timer_Fixed.cs
    Execute Command           mach create
    Execute Command           machine LoadPlatformDescription ${REPL}

Write Register
    [Arguments]  ${address}  ${value}
    Execute Command           sysbus WriteDoubleWord ${address} ${value}

Read Register
    [Arguments]  ${address}
    ${value}=  Execute Command  sysbus ReadDoubleWord ${address}
    RETURN  ${value.strip()}

Channel 1 Should Be
    [Arguments]  ${expected}
    ${state}=  Execute Command  python "print self.Machine['sysbus.timer4'].Connections[0].IsSet"
    Should Be Equal As Strings  ${state.strip()}  ${expected}

*** Test Cases ***
Counter Period Should Be ARR Plus One
    # timer4 runs at 10 MHz in stm32f4.repl. With ARR=99 the counter runs 0..99, so after
    # exactly 10000 ticks (1 ms) it must read 0 again; a period of ARR ticks would read 1.
    Create Machine
    Write Register            ${TIM4_ARR}  99
    Write Register            ${TIM4_CR1}  0x1
    Execute Command           emulation RunFor "0.001"
    ${cnt}=  Read Register    ${TIM4_CNT}
    Should Be Equal As Strings  ${cnt}  0x00000000

PWM Output Should Be Driven On Enable
    # PWM mode 1, CCR1=500, ARR=999. Right after CEN with CNT=0 < CCR1 the output is active;
    # it must not wait for the first overflow.
    Create Machine
    Write Register            ${TIM4_ARR}  999
    Write Register            ${TIM4_CCR1}  500
    Write Register            ${TIM4_CCMR1}  0x60
    Write Register            ${TIM4_CCER}  0x1
    Write Register            ${TIM4_CR1}  0x1
    Execute Command           emulation RunFor "0.00002"
    Channel 1 Should Be       True

CCR Write With Preload Should Apply On Update Event
    # OC1PE=1: writing CCR1=100 while CNT=200 must not change the output until the next
    # update event; the readback returns the preloaded value.
    Create Machine
    Write Register            ${TIM4_ARR}  999
    Write Register            ${TIM4_CCR1}  500
    Write Register            ${TIM4_CCMR1}  0x68
    Write Register            ${TIM4_CCER}  0x1
    Write Register            ${TIM4_CR1}  0x1
    Execute Command           emulation RunFor "0.00012"
    Channel 1 Should Be       True
    Write Register            ${TIM4_CCR1}  100
    Channel 1 Should Be       True
    ${ccr}=  Read Register    ${TIM4_CCR1}
    Should Be Equal As Strings  ${ccr}  0x00000064
    Execute Command           emulation RunFor "0.0001"
    Channel 1 Should Be       False
