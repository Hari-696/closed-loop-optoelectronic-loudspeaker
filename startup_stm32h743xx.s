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
.extern _stack_top
.extern main

/* Vector table */
.section .isr_vector, "a", %progbits
.type g_pfnVectors, %object
.global g_pfnVectors

g_pfnVectors:
    .word _stack_top              /* 0x00: Initial Stack Pointer */
    .word Reset_Handler           /* 0x04: Reset Handler */
    .word NMI_Handler             /* 0x08: NMI Handler */
    .word HardFault_Handler       /* 0x0C: Hard Fault Handler */
    .word MemManage_Handler       /* 0x10: MPU Fault Handler */
    .word BusFault_Handler        /* 0x14: Bus Fault Handler */
    .word UsageFault_Handler      /* 0x18: Usage Fault Handler */
    .word 0                       /* 0x1C: Reserved */
    .word 0                       /* 0x20: Reserved */
    .word 0                       /* 0x24: Reserved */
    .word 0                       /* 0x28: Reserved */
    .word SVC_Handler             /* 0x2C: SVCall Handler */
    .word DebugMon_Handler        /* 0x30: Debug Monitor Handler */
    .word 0                       /* 0x34: Reserved */
    .word PendSV_Handler          /* 0x38: PendSV Handler */
    .word SysTick_Handler         /* 0x3C: SysTick Handler */

    /* External Interrupts */
    .word WWDG_IRQHandler         /* 0x40: Window Watchdog */
    .word PVD_IRQHandler          /* 0x44: PVD */
    .word TAMP_STAMP_IRQHandler   /* 0x48: Tamper/TimeStamp */
    .word RTC_WKUP_IRQHandler     /* 0x4C: RTC Wakeup */
    .word FLASH_IRQHandler        /* 0x50: Flash */
    .word RCC_IRQHandler          /* 0x54: RCC */
    .word EXTI0_IRQHandler        /* 0x58: EXTI Line 0 */
    .word EXTI1_IRQHandler        /* 0x5C: EXTI Line 1 */
    .word EXTI2_IRQHandler        /* 0x60: EXTI Line 2 */
    .word EXTI3_IRQHandler        /* 0x64: EXTI Line 3 */
    .word EXTI4_IRQHandler        /* 0x68: EXTI Line 4 */
    .word DMA1_STR0_IRQHandler    /* 0x6C: DMA1 Stream 0 */
    .word DMA1_STR1_IRQHandler    /* 0x70: DMA1 Stream 1 */
    .word DMA1_STR2_IRQHandler    /* 0x74: DMA1 Stream 2 */
    .word DMA1_STR3_IRQHandler    /* 0x78: DMA1 Stream 3 */
    .word DMA1_STR4_IRQHandler    /* 0x7C: DMA1 Stream 4 */
    .word DMA1_STR5_IRQHandler    /* 0x80: DMA1 Stream 5 */
    .word DMA1_STR6_IRQHandler    /* 0x84: DMA1 Stream 6 */
    .word ADC_IRQHandler          /* 0x88: ADC */
    .word FDCAN1_IT0_IRQHandler   /* 0x8C: FDCAN1 IT0 */
    .word FDCAN1_IT1_IRQHandler   /* 0x90: FDCAN1 IT1 */
    .word EXTI9_5_IRQHandler      /* 0x94: EXTI Line 9..5 */
    .word TIM1_BRK_IRQHandler     /* 0x98: TIM1 Break */
    .word TIM1_UP_IRQHandler      /* 0x9C: TIM1 Update */
    .word TIM1_TRG_COM_IRQHandler /* 0xA0: TIM1 Trigger/Commutation */
    .word TIM1_CC_IRQHandler      /* 0xA4: TIM1 Capture Compare */
    .word TIM2_IRQHandler         /* 0xA8: TIM2 */
    .word TIM3_IRQHandler         /* 0xAC: TIM3 */
    .word TIM4_IRQHandler         /* 0xB0: TIM4 */
    .word I2C1_EV_IRQHandler      /* 0xB4: I2C1 Event */
    .word I2C1_ER_IRQHandler      /* 0xB8: I2C1 Error */
    .word I2C2_EV_IRQHandler      /* 0xBC: I2C2 Event */
    .word I2C2_ER_IRQHandler      /* 0xC0: I2C2 Error */
    .word SPI1_IRQHandler         /* 0xC4: SPI1 */
    .word SPI2_IRQHandler         /* 0xC8: SPI2 */
    .word USART1_IRQHandler       /* 0xCC: USART1 */
    .word USART2_IRQHandler       /* 0xD0: USART2 */
    .word USART3_IRQHandler       /* 0xD4: USART3 */
    .word EXTI15_10_IRQHandler    /* 0xD8: EXTI Line 15..10 */
    .word RTC_ALARM_IRQHandler    /* 0xDC: RTC Alarm */
    .word USB_FS_IRQHandler       /* 0xE0: USB FS */
    .word DMA1_STR7_IRQHandler    /* 0xE4: DMA1 Stream 7 */
    .word DMA2_STR0_IRQHandler    /* 0xE8: DMA2 Stream 0 */
    .word DMA2_STR1_IRQHandler    /* 0xEC: DMA2 Stream 1 */
    .word DMA2_STR2_IRQHandler    /* 0xF0: DMA2 Stream 2 */
    .word DMA2_STR3_IRQHandler    /* 0xF4: DMA2 Stream 3 */
    .word DMA2_STR4_IRQHandler    /* 0xF8: DMA2 Stream 4 */
    .word ETH_IRQHandler          /* 0xFC: Ethernet */
    .word ETH_WKUP_IRQHandler     /* 0x100: Ethernet Wakeup */
    .word FDCAN_CAL_IRQHandler    /* 0x104: FDCAN Calibration */
    .word 0                       /* 0x108: Reserved */
    .word 0                       /* 0x10C: Reserved */
    .word 0                       /* 0x110: Reserved */
    .word 0                       /* 0x114: Reserved */
    .word 0                       /* 0x118: Reserved */
    .word 0                       /* 0x11C: Reserved */
    .word 0                       /* 0x120: Reserved */
    .word 0                       /* 0x124: Reserved */
    .word 0                       /* 0x128: Reserved */
    .word 0                       /* 0x12C: Reserved */
    .word 0                       /* 0x130: Reserved */
    .word 0                       /* 0x134: Reserved */
    .word 0                       /* 0x138: Reserved */
    .word 0                       /* 0x13C: Reserved */
    .word 0                       /* 0x140: Reserved */
    .word 0                       /* 0x144: Reserved */
    .word 0                       /* 0x148: Reserved */
    .word 0                       /* 0x14C: Reserved */
    .word 0                       /* 0x150: Reserved */
    .word 0                       /* 0x154: Reserved */
    .word 0                       /* 0x158: Reserved */
    .word 0                       /* 0x15C: Reserved */
    .word 0                       /* 0x160: Reserved */
    .word 0                       /* 0x164: Reserved */
    .word 0                       /* 0x168: Reserved */
    .word 0                       /* 0x16C: Reserved */
    .word 0                       /* 0x170: Reserved */
    .word 0                       /* 0x174: Reserved */
    .word 0                       /* 0x178: Reserved */
    .word 0                       /* 0x17C: Reserved */
    .word 0                       /* 0x180: Reserved */
    .word 0                       /* 0x184: Reserved */
    .word 0                       /* 0x188: Reserved */
    .word 0                       /* 0x18C: Reserved */
    .word 0                       /* 0x190: Reserved */
    .word 0                       /* 0x194: Reserved */
    .word 0                       /* 0x198: Reserved */
    .word 0                       /* 0x19C: Reserved */
    .word 0                       /* 0x1A0: Reserved */
    .word 0                       /* 0x1A4: Reserved */
    .word 0                       /* 0x1A8: Reserved */
    .word 0                       /* 0x1AC: Reserved */
    .word 0                       /* 0x1B0: Reserved */
    .word 0                       /* 0x1B4: Reserved */
    .word 0                       /* 0x1B8: Reserved */
    .word 0                       /* 0x1BC: Reserved */
    .word 0                       /* 0x1C0: Reserved */
    .word 0                       /* 0x1C4: Reserved */
    .word 0                       /* 0x1C8: Reserved */
    .word 0                       /* 0x1CC: Reserved */
    .word 0                       /* 0x1D0: Reserved */
    .word 0                       /* 0x1D4: Reserved */
    .word 0                       /* 0x1D8: Reserved */
    .word 0                       /* 0x1DC: Reserved */
    .word 0                       /* 0x1E0: Reserved */
    .word 0                       /* 0x1E4: Reserved */
    .word 0                       /* 0x1E8: Reserved */
    .word 0                       /* 0x1EC: Reserved */
    .word 0                       /* 0x1F0: Reserved */
    .word 0                       /* 0x1F4: Reserved */
    .word 0                       /* 0x1F8: Reserved */
    .word 0                       /* 0x1FC: Reserved */

.size g_pfnVectors, . - g_pfnVectors

/* Default exception handlers */
.section .text.Default_Handler, "ax", %progbits
.weak NMI_Handler
.weak HardFault_Handler
.weak MemManage_Handler
.weak BusFault_Handler
.weak UsageFault_Handler
.weak SVC_Handler
.weak DebugMon_Handler
.weak PendSV_Handler
.weak SysTick_Handler
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

Default_Handler:
    b Default_Handler

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

/* Reset Handler */
.section .text.Reset_Handler, "ax", %progbits
.global Reset_Handler
.type Reset_Handler, %function

Reset_Handler:
    /* Copy .data from Flash to RAM */
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

    /* Zero .bss */
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

    /* Enable FPU (Cortex-M7 with FPv5) */
    ldr r0, =0xE000ED88      /* CPACR */
    ldr r1, [r0]
    orr r1, r1, #(0xF << 20) /* Enable CP10 and CP11 (full access) */
    str r1, [r0]
    dsb
    isb

    /* Enable DWT cycle counter */
    ldr r0, =0xE000EDFC      /* DEMCR */
    ldr r1, [r0]
    orr r1, r1, #(1 << 24)   /* TRCENA */
    str r1, [r0]
    ldr r0, =0xE0001000      /* DWT_CTRL */
    mov r1, #1               /* CYCCNTENA */
    str r1, [r0]

    /* Branch to main */
call_main:
    bl main

    /* Should never return */
    b .

.size Reset_Handler, . - Reset_Handler

.end