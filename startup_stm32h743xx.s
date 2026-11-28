/*
 * Startup file for STM32H743 (Cortex-M7)
 * Vector table, reset handler, and default exception handlers
 */

.syntax unified
.cpu cortex-m7
.fpu fpv5-d16
.thumb

/* Symbol imports */
.extern _sidata
.extern _sdata
.extern _edata
.extern _sbss
.extern _ebss
.extern _estack
.extern main

/* Vector table */
.section .isr_vector, "a", %progbits
.type g_pfnVectors, %object
.global g_pfnVectors

g_pfnVectors:
    .word _estack              /* 0x00: Initial Stack Pointer */
    .word Reset_Handler        /* 0x04: Reset Handler */
    .word NMI_Handler          /* 0x08: NMI Handler */
    .word HardFault_Handler    /* 0x0C: Hard Fault Handler */
    .word MemManage_Handler    /* 0x10: MPU Fault Handler */
    .word BusFault_Handler     /* 0x14: Bus Fault Handler */
    .word UsageFault_Handler   /* 0x18: Usage Fault Handler */
    .word 0                    /* 0x1C: Reserved */
    .word 0                    /* 0x20: Reserved */
    .word 0                    /* 0x24: Reserved */
    .word 0                    /* 0x28: Reserved */
    .word SVC_Handler          /* 0x2C: SVCall Handler */
    .word DebugMon_Handler     /* 0x30: Debug Monitor Handler */
    .word 0                    /* 0x34: Reserved */
    .word PendSV_Handler       /* 0x38: PendSV Handler */
    .word SysTick_Handler      /* 0x3C: SysTick Handler */

    /* External Interrupts (truncated for brevity - all weak aliases to Default_Handler) */
    .word Default_Handler      /* 0x40: WWDG */
    .word Default_Handler      /* 0x44: PVD */
    .word Default_Handler      /* 0x48: TAMP_STAMP */
    .word Default_Handler      /* 0x4C: RTC_WKUP */
    .word Default_Handler      /* 0x50: FLASH */
    .word Default_Handler      /* 0x54: RCC */
    .word Default_Handler      /* 0x58: EXTI0 */
    .word Default_Handler      /* 0x5C: EXTI1 */
    .word Default_Handler      /* 0x60: EXTI2 */
    .word Default_Handler      /* 0x64: EXTI3 */
    .word Default_Handler      /* 0x68: EXTI4 */
    .word Default_Handler      /* 0x6C: DMA1_STR0 */
    .word Default_Handler      /* 0x70: DMA1_STR1 */
    .word Default_Handler      /* 0x74: DMA1_STR2 */
    .word Default_Handler      /* 0x78: DMA1_STR3 */
    .word Default_Handler      /* 0x7C: DMA1_STR4 */
    .word Default_Handler      /* 0x80: DMA1_STR5 */
    .word Default_Handler      /* 0x84: DMA1_STR6 */
    .word Default_Handler      /* 0x88: ADC */
    .word Default_Handler      /* 0x8C: FDCAN1_IT0 */
    .word Default_Handler      /* 0x90: FDCAN1_IT1 */
    .word Default_Handler      /* 0x94: EXTI9_5 */
    .word Default_Handler      /* 0x98: TIM1_BRK */
    .word Default_Handler      /* 0x9C: TIM1_UP */
    .word Default_Handler      /* 0xA0: TIM1_TRG_COM */
    .word Default_Handler      /* 0xA4: TIM1_CC */
    .word Default_Handler      /* 0xA8: TIM2 */
    .word Default_Handler      /* 0xAC: TIM3 */
    .word Default_Handler      /* 0xB0: TIM4 */
    .word Default_Handler      /* 0xB4: I2C1_EV */
    .word Default_Handler      /* 0xB8: I2C1_ER */
    .word Default_Handler      /* 0xBC: I2C2_EV */
    .word Default_Handler      /* 0xC0: I2C2_ER */
    .word Default_Handler      /* 0xC4: SPI1 */
    .word Default_Handler      /* 0xC8: SPI2 */
    .word Default_Handler      /* 0xCC: USART1 */
    .word Default_Handler      /* 0xD0: USART2 */
    .word Default_Handler      /* 0xD4: USART3 */
    .word Default_Handler      /* 0xD8: EXTI15_10 */
    .word Default_Handler      /* 0xDC: RTC_ALARM */
    .word Default_Handler      /* 0xE0: USB_FS */
    .word Default_Handler      /* 0xE4: DMA1_STR7 */
    .word Default_Handler      /* 0xE8: DMA2_STR0 */
    .word Default_Handler      /* 0xEC: DMA2_STR1 */
    .word Default_Handler      /* 0xF0: DMA2_STR2 */
    .word Default_Handler      /* 0xF4: DMA2_STR3 */
    .word Default_Handler      /* 0xF8: DMA2_STR4 */
    .word Default_Handler      /* 0xFC: ETH */
    .word Default_Handler      /* 0x100: ETH_WKUP */
    .word Default_Handler      /* 0x104: FDCAN_CAL */

    /* Pad to 0x200 bytes (512 bytes) for 128 interrupt vectors */
    .fill 128 - 42, 4, 0

.size g_pfnVectors, . - g_pfnVectors

/* Default exception handler */
.section .text.Default_Handler, "ax", %progbits
Default_Handler:
    b Default_Handler

/* Weak aliases for all handlers */
.weak NMI_Handler
.weak HardFault_Handler
.weak MemManage_Handler
.weak BusFault_Handler
.weak UsageFault_Handler
.weak SVC_Handler
.weak DebugMon_Handler
.weak PendSV_Handler
.weak SysTick_Handler

/* Aliases to Default_Handler */
NMI_Handler:          b Default_Handler
HardFault_Handler:    b Default_Handler
MemManage_Handler:    b Default_Handler
BusFault_Handler:     b Default_Handler
UsageFault_Handler:   b Default_Handler
SVC_Handler:          b Default_Handler
DebugMon_Handler:     b Default_Handler
PendSV_Handler:       b Default_Handler
SysTick_Handler:      b Default_Handler

/* External interrupt handlers */
.weak WWDG_IRQHandler
.weak PVD_IRQHandler
.weak TAMP_STAMP_IRQHandler
.weak RTC_WKUP_IRQHandler
.weak FLASH_IRQHandler
.weak RCC_IRQHandler
.weak EXTI0_IRQHandler
.weak EXTI1_IRQHandler
.weak EXTI2_IRQHandler
.weak EXTI3_IRQHandler
.weak EXTI4_IRQHandler
.weak DMA1_STR0_IRQHandler
.weak DMA1_STR1_IRQHandler
.weak DMA1_STR2_IRQHandler
.weak DMA1_STR3_IRQHandler
.weak DMA1_STR4_IRQHandler
.weak DMA1_STR5_IRQHandler
.weak DMA1_STR6_IRQHandler
.weak ADC_IRQHandler
.weak FDCAN1_IT0_IRQHandler
.weak FDCAN1_IT1_IRQHandler
.weak EXTI9_5_IRQHandler
.weak TIM1_BRK_IRQHandler
.weak TIM1_UP_IRQHandler
.weak TIM1_TRG_COM_IRQHandler
.weak TIM1_CC_IRQHandler
.weak TIM2_IRQHandler
.weak TIM3_IRQHandler
.weak TIM4_IRQHandler
.weak I2C1_EV_IRQHandler
.weak I2C1_ER_IRQHandler
.weak I2C2_EV_IRQHandler
.weak I2C2_ER_IRQHandler
.weak SPI1_IRQHandler
.weak SPI2_IRQHandler
.weak USART1_IRQHandler
.weak USART2_IRQHandler
.weak USART3_IRQHandler
.weak EXTI15_10_IRQHandler
.weak RTC_ALARM_IRQHandler
.weak USB_FS_IRQHandler
.weak DMA1_STR7_IRQHandler
.weak DMA2_STR0_IRQHandler
.weak DMA2_STR1_IRQHandler
.weak DMA2_STR2_IRQHandler
.weak DMA2_STR3_IRQHandler
.weak DMA2_STR4_IRQHandler
.weak ETH_IRQHandler
.weak ETH_WKUP_IRQHandler
.weak FDCAN_CAL_IRQHandler

WWDG_IRQHandler:      b Default_Handler
PVD_IRQHandler:       b Default_Handler
TAMP_STAMP_IRQHandler: b Default_Handler
RTC_WKUP_IRQHandler:  b Default_Handler
FLASH_IRQHandler:     b Default_Handler
RCC_IRQHandler:       b Default_Handler
EXTI0_IRQHandler:     b Default_Handler
EXTI1_IRQHandler:     b Default_Handler
EXTI2_IRQHandler:     b Default_Handler
EXTI3_IRQHandler:     b Default_Handler
EXTI4_IRQHandler:     b Default_Handler
DMA1_STR0_IRQHandler: b Default_Handler
DMA1_STR1_IRQHandler: b Default_Handler
DMA1_STR2_IRQHandler: b Default_Handler
DMA1_STR3_IRQHandler: b Default_Handler
DMA1_STR4_IRQHandler: b Default_Handler
DMA1_STR5_IRQHandler: b Default_Handler
DMA1_STR6_IRQHandler: b Default_Handler
ADC_IRQHandler:       b Default_Handler
FDCAN1_IT0_IRQHandler: b Default_Handler
FDCAN1_IT1_IRQHandler: b Default_Handler
EXTI9_5_IRQHandler:   b Default_Handler
TIM1_BRK_IRQHandler:  b Default_Handler
TIM1_UP_IRQHandler:   b Default_Handler
TIM1_TRG_COM_IRQHandler: b Default_Handler
TIM1_CC_IRQHandler:   b Default_Handler
TIM2_IRQHandler:      b Default_Handler
TIM3_IRQHandler:      b Default_Handler
TIM4_IRQHandler:      b Default_Handler
I2C1_EV_IRQHandler:   b Default_Handler
I2C1_ER_IRQHandler:   b Default_Handler
I2C2_EV_IRQHandler:   b Default_Handler
I2C2_ER_IRQHandler:   b Default_Handler
SPI1_IRQHandler:      b Default_Handler
SPI2_IRQHandler:      b Default_Handler
USART1_IRQHandler:    b Default_Handler
USART2_IRQHandler:    b Default_Handler
USART3_IRQHandler:    b Default_Handler
EXTI15_10_IRQHandler: b Default_Handler
RTC_ALARM_IRQHandler: b Default_Handler
USB_FS_IRQHandler:    b Default_Handler
DMA1_STR7_IRQHandler: b Default_Handler
DMA2_STR0_IRQHandler: b Default_Handler
DMA2_STR1_IRQHandler: b Default_Handler
DMA2_STR2_IRQHandler: b Default_Handler
DMA2_STR3_IRQHandler: b Default_Handler
DMA2_STR4_IRQHandler: b Default_Handler
ETH_IRQHandler:       b Default_Handler
ETH_WKUP_IRQHandler:  b Default_Handler
FDCAN_CAL_IRQHandler: b Default_Handler

/* Reset Handler - CRITICAL: Enable FPU BEFORE any FP operations */
.section .text.Reset_Handler, "ax", %progbits
.global Reset_Handler
.type Reset_Handler, %function

Reset_Handler:
    /* 1. Enable FPU (Cortex-M7 with FPv5) - MUST BE FIRST */
    ldr r0, =0xE000ED88      /* CPACR */
    ldr r1, [r0]
    orr r1, r1, #(0xF << 20) /* Enable CP10 and CP11 (full access) */
    str r1, [r0]
    dsb
    isb

    /* 2. Enable DWT cycle counter for profiling */
    ldr r0, =0xE000EDFC      /* DEMCR */
    ldr r1, [r0]
    orr r1, r1, #(1 << 24)   /* TRCENA */
    str r1, [r0]
    ldr r0, =0xE0001000      /* DWT_CTRL */
    mov r1, #1               /* CYCCNTENA */
    str r1, [r0]

    /* 3. Copy .data from Flash to RAM_D1 (AXI SRAM) */
    ldr r0, =_sidata
    ldr r1, =_sdata
    ldr r2, =_edata
    cmp r0, r1
    beq bss_init_loop
data_copy_loop:
    ldr r3, [r0], #4
    str r3, [r1], #4
    cmp r1, r2
    bne data_copy_loop

    /* 4. Zero .bss in RAM_D1 */
bss_init_loop:
    ldr r1, =_sbss
    ldr r2, =_ebss
    cmp r1, r2
    beq call_main
bss_zero_loop:
    mov r3, #0
    str r3, [r1], #4
    cmp r1, r2
    bne bss_zero_loop

    /* 5. Branch to main (C++ entry point) */
call_main:
    bl main

    /* Should never return */
    b .

.size Reset_Handler, . - Reset_Handler

.end