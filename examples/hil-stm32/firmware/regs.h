/*
 * STM32F4 暫存器定義:只列這支韌體真的碰到的,位址與位元出自 RM0090。
 * 不用 CMSIS / HAL,理由見 docs/hil/36 篇:每一個寫進去的位元都要能回答
 * 「模擬器有沒有實作它」,標頭檔裡看不到的東西不會被隱含要求。
 */
#ifndef REGS_H
#define REGS_H

#include <stdint.h>

#define REG(addr) (*(volatile uint32_t *)(addr))

/* ---- RCC @0x40023800 ---------------------------------------------------- */
#define RCC_BASE      0x40023800u
#define RCC_AHB1ENR   REG(RCC_BASE + 0x30)
#define RCC_APB1ENR   REG(RCC_BASE + 0x40)
#define RCC_APB2ENR   REG(RCC_BASE + 0x44)
#define RCC_AHB1ENR_GPIOA  (1u << 0)
#define RCC_AHB1ENR_GPIOB  (1u << 1)
#define RCC_AHB1ENR_GPIOC  (1u << 2)
#define RCC_APB1ENR_TIM2   (1u << 0)
#define RCC_APB1ENR_TIM3   (1u << 1)
#define RCC_APB1ENR_TIM4   (1u << 2)
#define RCC_APB1ENR_USART2 (1u << 17)
#define RCC_APB1ENR_CAN1   (1u << 25)
#define RCC_APB2ENR_USART1 (1u << 4)

/* ---- GPIO --------------------------------------------------------------- */
#define GPIOA_BASE    0x40020000u
#define GPIOB_BASE    0x40020400u
#define GPIOC_BASE    0x40020800u
#define GPIO_MODER(b)  REG((b) + 0x00)
#define GPIO_IDR(b)    REG((b) + 0x10)
#define GPIO_ODR(b)    REG((b) + 0x14)
#define GPIO_BSRR(b)   REG((b) + 0x18)
#define GPIO_AFRL(b)   REG((b) + 0x20)
#define GPIO_AFRH(b)   REG((b) + 0x24)

/* 腳位配置(與 renode/hilctl.repl、bridge 的設定要一致) */
#define PWM_L_PIN      6   /* PA6 = TIM3_CH1 (AF2) 左輪 PWM */
#define PWM_R_PIN      7   /* PA7 = TIM3_CH2 (AF2) 右輪 PWM */
#define DIR_L_PIN      8   /* PB8 左輪方向,1 = 前進 */
#define DIR_R_PIN      9   /* PB9 右輪方向,1 = 前進 */
#define MOTOR_EN_PIN   10  /* PB10 馬達致能,1 = 致能 */
#define ESTOP_PIN      13  /* PC13 急停輸入,1 = 急停觸發 */

/* ---- TIM3 @0x40000400(APB1)------------------------------------------- */
#define TIM3_BASE     0x40000400u
#define TIM3_CR1      REG(TIM3_BASE + 0x00)
#define TIM3_EGR      REG(TIM3_BASE + 0x14)
#define TIM3_CCMR1    REG(TIM3_BASE + 0x18)
#define TIM3_CCER     REG(TIM3_BASE + 0x20)
#define TIM3_CNT      REG(TIM3_BASE + 0x24)
#define TIM3_PSC      REG(TIM3_BASE + 0x28)
#define TIM3_ARR      REG(TIM3_BASE + 0x2C)
#define TIM3_CCR1     REG(TIM3_BASE + 0x34)
#define TIM3_CCR2     REG(TIM3_BASE + 0x38)

/* TIM2(PA0/PA1 AF1)與 TIM4(PB6/PB7 AF2)當編碼器介面:RM0090 §18.3.12 encoder mode */
#define TIM2_BASE     0x40000000u
#define TIM4_BASE     0x40000800u
#define TIM_CR1(b)    REG((b) + 0x00)
#define TIM_SMCR(b)   REG((b) + 0x08)
#define TIM_CCMR1(b)  REG((b) + 0x18)
#define TIM_CCER(b)   REG((b) + 0x20)
#define TIM_CNT(b)    REG((b) + 0x24)
#define TIM_ARR(b)    REG((b) + 0x2C)
#define TIM_SMCR_SMS_ENCODER3  3u          /* TI1 與 TI2 的邊緣都計數 */
#define TIM_CCMR1_CC1S_TI1     (1u << 0)
#define TIM_CCMR1_CC2S_TI2     (1u << 8)
#define TIM_CR1_CEN        (1u << 0)
#define TIM_CR1_ARPE       (1u << 7)
#define TIM_EGR_UG         (1u << 0)
#define TIM_CCMR1_OC1M_PWM1 (6u << 4)
#define TIM_CCMR1_OC1PE    (1u << 3)
#define TIM_CCMR1_OC2M_PWM1 (6u << 12)
#define TIM_CCMR1_OC2PE    (1u << 11)
#define TIM_CCER_CC1E      (1u << 0)
#define TIM_CCER_CC2E      (1u << 4)

/* ---- USART ---------------------------------------------------------------
 * USART1 = 上位協定(二進位框包),USART2 = 人看的除錯輸出。 */
#define USART1_BASE   0x40011000u
#define USART2_BASE   0x40004400u
#define USART_SR(b)    REG((b) + 0x00)
#define USART_DR(b)    REG((b) + 0x04)
#define USART_BRR(b)   REG((b) + 0x08)
#define USART_CR1(b)   REG((b) + 0x0C)
#define USART_SR_RXNE  (1u << 5)
#define USART_SR_TXE   (1u << 7)
#define USART_CR1_RE   (1u << 2)
#define USART_CR1_TE   (1u << 3)
#define USART_CR1_RXNEIE (1u << 5)
#define USART_CR1_UE   (1u << 13)
#define USART1_IRQN    37  /* RM0090 向量表 */

/* ---- NVIC ---------------------------------------------------------------- */
#define NVIC_ISER(n)  REG(0xE000E100u + 4u * (n))
#define NVIC_IPR(irq) (*(volatile uint8_t *)(0xE000E400u + (irq)))

/* ---- bxCAN CAN1 @0x40006400 -------------------------------------------- */
#define CAN1_BASE     0x40006400u
#define CAN_MCR       REG(CAN1_BASE + 0x000)
#define CAN_MSR       REG(CAN1_BASE + 0x004)
#define CAN_TSR       REG(CAN1_BASE + 0x008)
#define CAN_RF0R      REG(CAN1_BASE + 0x00C)
#define CAN_BTR       REG(CAN1_BASE + 0x01C)
#define CAN_TI0R      REG(CAN1_BASE + 0x180)
#define CAN_TDT0R     REG(CAN1_BASE + 0x184)
#define CAN_TDL0R     REG(CAN1_BASE + 0x188)
#define CAN_TDH0R     REG(CAN1_BASE + 0x18C)
#define CAN_RI0R      REG(CAN1_BASE + 0x1B0)
#define CAN_RDT0R     REG(CAN1_BASE + 0x1B4)
#define CAN_RDL0R     REG(CAN1_BASE + 0x1B8)
#define CAN_RDH0R     REG(CAN1_BASE + 0x1BC)
#define CAN_FMR       REG(CAN1_BASE + 0x200)
#define CAN_FM1R      REG(CAN1_BASE + 0x204)
#define CAN_FS1R      REG(CAN1_BASE + 0x20C)
#define CAN_FFA1R     REG(CAN1_BASE + 0x214)
#define CAN_FA1R      REG(CAN1_BASE + 0x21C)
#define CAN_F0R1      REG(CAN1_BASE + 0x240)
#define CAN_F0R2      REG(CAN1_BASE + 0x244)
#define CAN_MCR_INRQ   (1u << 0)
#define CAN_MCR_SLEEP  (1u << 1)
#define CAN_MSR_INAK   (1u << 0)
#define CAN_MSR_SLAK   (1u << 1)
#define CAN_TSR_TME0   (1u << 26)
#define CAN_RF0R_FMP0_MASK 0x3u
#define CAN_RF0R_RFOM0 (1u << 5)
#define CAN_TI0R_TXRQ  (1u << 0)
#define CAN_FMR_FINIT  (1u << 0)
#define CAN_FMR_CAN2SB_MASK (0x3Fu << 8)

/* ---- SysTick @0xE000E010 ------------------------------------------------ */
#define SYST_CSR      REG(0xE000E010u)
#define SYST_RVR      REG(0xE000E014u)
#define SYST_CVR      REG(0xE000E018u)
#define SYST_CSR_ENABLE    (1u << 0)
#define SYST_CSR_TICKINT   (1u << 1)
#define SYST_CSR_CLKSOURCE (1u << 2)

/* SysTick 的時脈。真硬體是 HCLK;Renode 的 NVIC 用平台描述裡的
 * systickFrequency(內建 stm32f4.repl 為 72 MHz),見 renode/hilctl.repl。 */
#define SYSTICK_HZ    72000000u

#endif
