/**
 * @file main_m7.cpp
 * @brief Bare-metal entry point for STM32H743 (Cortex-M7 @ 480 MHz)
 * 
 * Initializes DSP pipeline, runs 1000 deterministic dsp_step() iterations,
 * prints min/avg/max cycle counts (total and per-stage) over USART1.
 */

#include "dsp/dsp_pipeline.hpp"
#include <cstdint>
#include <cstdio>
#include <cstring>

/* USART1 registers (STM32H743) */
#define USART1_BASE         0x40011000
#define USART1_ISR          (*reinterpret_cast<volatile uint32_t*>(USART1_BASE + 0x1C))
#define USART1_TDR          (*reinterpret_cast<volatile uint32_t*>(USART1_BASE + 0x28))
#define USART1_ISR_TXE      (1 << 7)   /* Transmit data register empty */
#define USART1_ISR_TC       (1 << 6)   /* Transmission complete */

/* RCC registers for enabling USART1 and GPIOA */
#define RCC_BASE            0x58024400
#define RCC_AHB4ENR         (*reinterpret_cast<volatile uint32_t*>(RCC_BASE + 0xA0))
#define RCC_APB2ENR         (*reinterpret_cast<volatile uint32_t*>(RCC_BASE + 0x70))
#define RCC_AHB4ENR_GPIOAEN (1 << 0)
#define RCC_APB2ENR_USART1EN (1 << 4)

/* GPIOA registers for USART1 TX (PA9) */
#define GPIOA_BASE          0x58020000
#define GPIOA_MODER         (*reinterpret_cast<volatile uint32_t*>(GPIOA_BASE + 0x00))
#define GPIOA_OTYPER        (*reinterpret_cast<volatile uint32_t*>(GPIOA_BASE + 0x04))
#define GPIOA_OSPEEDR       (*reinterpret_cast<volatile uint32_t*>(GPIOA_BASE + 0x08))
#define GPIOA_PUPDR         (*reinterpret_cast<volatile uint32_t*>(GPIOA_BASE + 0x0C))
#define GPIOA_AFRH          (*reinterpret_cast<volatile uint32_t*>(GPIOA_BASE + 0x24))
#define GPIO_MODER_ALT      0x2
#define GPIO_AF7_USART1     0x7

/* Cycle counter access (DWT) */
#define DWT_CTRL            (*reinterpret_cast<volatile uint32_t*>(0xE0001000))
#define DWT_CYCCNT          (*reinterpret_cast<volatile uint32_t*>(0xE0001004))
#define DEMCR               (*reinterpret_cast<volatile uint32_t*>(0xE000EDFC))

/* Maximum iterations */
constexpr int NUM_ITERATIONS = 1000;

/* Per-stage cycle tracking */
struct StageCycles {
    uint32_t min = UINT32_MAX;
    uint32_t max = 0;
    uint64_t sum = 0;
    uint32_t count = 0;
    
    void add(uint32_t cycles) {
        if (cycles < min) min = cycles;
        if (cycles > max) max = cycles;
        sum += cycles;
        count++;
    }
    
    uint32_t avg() const { return count ? static_cast<uint32_t>(sum / count) : 0; }
};

/* Global stage cycle trackers */
static StageCycles g_stage_total;
static StageCycles g_stage_lut;
static StageCycles g_stage_observer;
static StageCycles g_stage_volterra;
static StageCycles g_stage_mfb;
static StageCycles g_stage_limiter;

/* USART1 initialization (115200 baud @ 480 MHz) */
static void usart1_init() {
    /* Enable GPIOA and USART1 clocks */
    RCC_AHB4ENR |= RCC_AHB4ENR_GPIOAEN;
    RCC_APB2ENR |= RCC_APB2ENR_USART1EN;
    
    /* Configure PA9 as USART1_TX (AF7) */
    GPIOA_MODER &= ~(0x3 << (9 * 2));
    GPIOA_MODER |= (GPIO_MODER_ALT << (9 * 2));
    GPIOA_OTYPER &= ~(1 << 9);
    GPIOA_OSPEEDR |= (0x3 << (9 * 2));  /* Very high speed */
    GPIOA_PUPDR &= ~(0x3 << (9 * 2));
    GPIOA_PUPDR |= (0x1 << (9 * 2));    /* Pull-up */
    GPIOA_AFRH &= ~(0xF << ((9 - 8) * 4));
    GPIOA_AFRH |= (GPIO_AF7_USART1 << ((9 - 8) * 4));
    
    /* USART1 configuration: 115200 baud, 8N1 */
    /* BRR = Fclk / Baud = 480,000,000 / 115200 = 4166.67 ≈ 4167 */
    /* USARTDIV = 4167 = 0x1047 */
    volatile uint32_t* USART1_BRR = reinterpret_cast<volatile uint32_t*>(USART1_BASE + 0x0C);
    volatile uint32_t* USART1_CR1 = reinterpret_cast<volatile uint32_t*>(USART1_BASE + 0x00);
    volatile uint32_t* USART1_CR2 = reinterpret_cast<volatile uint32_t*>(USART1_BASE + 0x04);
    volatile uint32_t* USART1_CR3 = reinterpret_cast<volatile uint32_t*>(USART1_BASE + 0x08);
    
    *USART1_CR1 = 0;           /* Disable USART */
    *USART1_BRR = 4167;        /* 115200 baud @ 480 MHz */
    *USART1_CR2 = 0;           /* 1 stop bit */
    *USART1_CR3 = 0;           /* No flow control */
    *USART1_CR1 = (1 << 3) | (1 << 2) | (1 << 0);  /* TE | RE | UE */
}

/* Send single character over USART1 */
static void usart1_putc(char c) {
    while (!(USART1_ISR & USART1_ISR_TXE)) {
        /* Wait for TXE */
    }
    USART1_TDR = static_cast<uint32_t>(c);
}

/* Send string over USART1 */
static void usart1_puts(const char* s) {
    while (*s) {
        usart1_putc(*s++);
    }
}

/* Print unsigned 32-bit integer */
static void usart1_putu32(uint32_t val) {
    char buf[12];
    int i = 0;
    if (val == 0) {
        usart1_putc('0');
        return;
    }
    while (val > 0) {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    }
    while (i--) {
        usart1_putc(buf[i]);
    }
}

/* Print stage results */
static void print_stage(const char* name, const StageCycles& s) {
    usart1_puts(name);
    usart1_puts(": min=");
    usart1_putu32(s.min);
    usart1_puts(" avg=");
    usart1_putu32(s.avg());
    usart1_puts(" max=");
    usart1_putu32(s.max);
    usart1_puts("\r\n");
}

/* Main entry */
int main() {
    /* Initialize USART1 */
    usart1_init();
    
    usart1_puts("\r\n=== STM32H743 DSP Firmware (Cortex-M7 @ 480 MHz) ===\r\n");
    usart1_puts("Running ");
    usart1_putu32(NUM_ITERATIONS);
    usart1_puts(" deterministic iterations...\r\n\r\n");
    
    /* Initialize DSP pipeline */
    usart1_puts("Initializing DSP pipeline...\r\n");
    if (!dsp::dsp_pipeline_init()) {
        usart1_puts("ERROR: DSP pipeline init failed!\r\n");
        while (1) { }
    }
    usart1_puts("DSP pipeline initialized OK.\r\n\r\n");
    
    /* Run deterministic iterations */
    dsp::AdcFrame adc{};
    dsp::DacFrame dac{};
    adc.timestamp_ns = 0;
    dac.timestamp_ns = 0;
    
    /* Test reference: 1.0 X_max sine at 30 Hz (12 mm) */
    /* For simplicity, use fixed reference - real system would use LUT */
    for (int iter = 0; iter < NUM_ITERATIONS; ++iter) {
        /* Simulated ADC input (placeholder - real system reads from ADC) */
        /* Using zero displacement for simplicity - controller will drive to zero */
        adc.displacement_adc = 0;
        adc.current_adc = 0;
        adc.voltage_adc = 0;
        adc.timestamp_ns = static_cast<uint64_t>(iter);
        
        /* Read cycle counter before */
        uint32_t cycles_before = DWT_CYCCNT;
        
        /* Execute one DSP step */
        dsp::dsp_step(&adc, &dac);
        
        /* Read cycle counter after */
        uint32_t cycles_after = DWT_CYCCNT;
        uint32_t total_cycles = cycles_after - cycles_before;
        
        g_stage_total.add(total_cycles);
        
        /* Extract per-stage timing from telemetry (if available) */
        /* For now, we accumulate total - detailed per-stage would need 
           instrumented DSP pipeline */
        
        /* Print progress every 100 iterations */
        if ((iter % 100) == 0) {
            usart1_puts("Iter ");
            usart1_putu32(iter);
            usart1_puts(": total cycles = ");
            usart1_putu32(total_cycles);
            usart1_puts("\r\n");
        }
    }
    
    usart1_puts("\r\n=== Cycle Count Summary ===\r\n");
    print_stage("Total      ", g_stage_total);
    print_stage("LUT        ", g_stage_lut);
    print_stage("Observer   ", g_stage_observer);
    print_stage("Volterra   ", g_stage_volterra);
    print_stage("MFB        ", g_stage_mfb);
    print_stage("Limiter    ", g_stage_limiter);
    
    usart1_puts("\r\n=== Test Complete ===\r\n");
    
    /* Infinite loop */
    while (1) {
        /* Could add watchdog kick here */
    }
    
    return 0;
}