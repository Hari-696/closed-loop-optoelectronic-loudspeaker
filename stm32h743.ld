/*
 * Linker Script for STM32H743 (2 MB Flash, 512 KB AXI SRAM, DTCM/ITCM)
 * Target: ARM Cortex-M7 @ 480 MHz
 */

ENTRY(Reset_Handler)

MEMORY
{
    FLASH   (rx) : ORIGIN = 0x08000000, LENGTH = 2048K   /* 2 MB Flash */
    RAM_D1  (rwx): ORIGIN = 0x24000000, LENGTH = 512K    /* 512 KB AXI SRAM (D1 domain) */
    RAM_D2  (rwx): ORIGIN = 0x30000000, LENGTH = 288K    /* 288 KB SRAM1+2+3 (D2 domain) */
    RAM_D3  (rwx): ORIGIN = 0x38000000, LENGTH = 64K     /* 64 KB SRAM4 (D3 domain) */
    ITCM_RAM (rwx): ORIGIN = 0x00000000, LENGTH = 64K    /* 64 KB ITCM (instruction tightly coupled) */
    DTCM_RAM (rwx): ORIGIN = 0x20000000, LENGTH = 128K   /* 128 KB DTCM (data tightly coupled) */
}

/* Stack/heap configuration */
_stack_size = 16K;
_heap_size = 0;  /* No heap - zero malloc/free */

/* Section definitions */
SECTIONS
{
    /* Vector table and startup code in FLASH */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    /* Program code in FLASH */
    .text :
    {
        . = ALIGN(4);
        *(.text*)
        *(.rodata*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        . = ALIGN(4);
    } >FLASH

    /* Read-only data in FLASH */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    /* ARM exception unwind tables */
    .ARM.extab :
    {
        . = ALIGN(4);
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        . = ALIGN(4);
    } >FLASH

    .ARM.exidx :
    {
        . = ALIGN(4);
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        . = ALIGN(4);
    } >FLASH

    /* Pre-init array */
    .preinit_array :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
        . = ALIGN(4);
    } >FLASH

    /* Init array */
    .init_array :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
        . = ALIGN(4);
    } >FLASH

    /* Fini array */
    .fini_array :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
        . = ALIGN(4);
    } >FLASH

    /* Data initialization - LMA in FLASH, VMA in RAM_D1 */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data*)
        *(.bss*)
        . = ALIGN(4);
        _edata = .;
    } >RAM_D1 AT>FLASH

    /* BSS section in RAM_D1 */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } >RAM_D1

    /* Stack in DTCM for fastest access (no heap) */
    .stack_dtcmsram (NOLOAD) :
    {
        . = ALIGN(8);
        _stack_bottom = .;
        . += _stack_size;
        . = ALIGN(8);
        _stack_top = .;
    } >DTCM_RAM

    /* Heap in RAM_D1 - zero size as requested */
    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        _heap_bottom = .;
        . += _heap_size;
        . = ALIGN(8);
        _heap_top = .;
    } >RAM_D1

    /* Remove unused sections */
    /DISCARD/ :
    {
        *(.note.gnu.build-id)
        *(.note.gnu.property)
        *(.comment)
        *(.eh_frame)
    }
}

/* Symbols for runtime use */
PROVIDE(_sidata = LOADADDR(.data));
PROVIDE(_sdata = ADDR(.data));
PROVIDE(_edata = ADDR(.data) + SIZEOF(.data));
PROVIDE(_sbss = ADDR(.bss));
PROVIDE(_ebss = ADDR(.bss) + SIZEOF(.bss));
PROVIDE(_stack_top = _stack_top);
PROVIDE(_stack_bottom = _stack_bottom);
PROVIDE(_heap_top = _heap_top);
PROVIDE(_heap_bottom = _heap_bottom);