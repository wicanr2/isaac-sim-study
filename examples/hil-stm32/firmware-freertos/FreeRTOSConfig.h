/*
 * FreeRTOS 設定:hilctl-rtos(STM32F4,Renode 1.16.1 的 stm32f4 平台)。
 * 每一個值都與 Renode 的平台描述或韌體的其他設定綁在一起,改之前看註解。
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* SysTick 的時脈。Renode 用平台描述的 nvic.systickFrequency(72 MHz),不看 RCC;
 * regs.h 的 SYSTICK_HZ 也是這個值。真硬體要與實際 HCLK 一致。 */
#define configCPU_CLOCK_HZ                      72000000UL
#define configTICK_RATE_HZ                      1000          /* 1 ms tick,與裸機版的 SysTick 相同 */
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  0
#define configUSE_TICKLESS_IDLE                 0             /* idle hook 自己 WFI,見 main-rtos.c */
#define configMAX_PRIORITIES                    5
#define configMINIMAL_STACK_SIZE                128           /* 單位是 word */
#define configTICK_TYPE_WIDTH_IN_BITS           TICK_TYPE_WIDTH_32_BITS
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   1
#define configUSE_MUTEXES                       0
#define configUSE_COUNTING_SEMAPHORES           0
#define configUSE_QUEUE_SETS                    0
#define configQUEUE_REGISTRY_SIZE               0
#define configUSE_TIMERS                        0
#define configUSE_CO_ROUTINES                   0
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   ( 12 * 1024 ) /* 三個 task + idle 的 TCB 與 stack */
#define configUSE_IDLE_HOOK                     1
#define configUSE_TICK_HOOK                     0
#define configUSE_MALLOC_FAILED_HOOK            1
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0
#define configENABLE_FPU                        1
#define configENABLE_MPU                        0
#define configENABLE_TRUSTZONE                  0
#define configRUN_FREERTOS_SECURE_ONLY          1

/* Cortex-M4 的 NVIC 有 4 個優先權位元(Renode 的 nvic priorityMask: 0xF0 同義)。
 * 核心(PendSV/SysTick)用最低優先權 15;能呼叫 FromISR API 的中斷,數值要 >= 5(即較不緊急)。
 * USART1 IRQ 設 6,見 main-rtos.c。 */
#define configPRIO_BITS                         4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5
#define configKERNEL_INTERRUPT_PRIORITY         ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

/* 出事時停在這裡:PC 會停在 while(1),g_dbg.assert_line 記下行號(橋接讀得到) */
extern void hil_assert_failed(const char *file, int line);
#define configASSERT( x )                       if( ( x ) == 0 ) hil_assert_failed( __FILE__, __LINE__ )

#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskDelayUntil                 1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_vTaskPrioritySet                0
#define INCLUDE_uxTaskPriorityGet               0
#define INCLUDE_vTaskDelete                     0

/* Cortex-M 的三個核心例外對到 port 的處理函式(startup-rtos.c 的向量表用這些名字) */
#define vPortSVCHandler                         SVC_Handler
#define xPortPendSVHandler                      PendSV_Handler
#define xPortSysTickHandler                     SysTick_Handler

#endif
