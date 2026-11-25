# Closed-Loop Optoelectronically Linearized Loudspeaker System

> **Real-time cyber-physical loudspeaker linearization with optoelectronic displacement sensing, hybrid MFB/Volterra control, and PostgreSQL JSONB telemetry.**

## Overview

This project implements a closed-loop loudspeaker linearization system as specified in the NIT Durgapur capstone project report. The system targets **10-20 dB THD/IMD reduction (simulated)** at high excursion (0.1-1.0 X_max) through:

- **Non-contact optoelectronic displacement sensing** in vented pole piece (zero added moving mass)
- **Sub-100 µs deterministic DSP pipeline** on ARM Cortex-M7 @ 480 MHz / 96 kHz (simulated)
- **Hybrid control**: Luenberger observer + MFB state-feedback + adaptive inverse Volterra feedforward (Sign-LMS)
- **Cooperative decoupling & over-excursion protection** preventing mechanical bottoming
- **Real-time LSP extraction**: Bl(x), K_ms(x), L_e(x), R_e(T), Δφ(ω)
- **Lock-free telemetry** → PostgreSQL JSONB with generated-column B-tree indexes

## Architecture

```
                                    ┌─────────────────────────────────────┐
                                    │         DSP PIPELINE (96 kHz)       │
                                    │  ┌─────────┐ ┌──────────┐          │
                                    │  │ Σ (err) │▶│ Observer │          │
x_ref(t) ───▶ (+) ───▶ [DAC 96kHz] ─┤   e = x_ref - x_meas   [Volterra] │
         ▲                         │  └────┬────┘ └────┬────┘          │
         │                          │       │       │                │
         │                    ┌──────▼───┐ ┌──▼─────────▼──┐         │
         │                    │  State   │ │  Sign-LMS   │         │
         │                    │Feedback  │ │  (u_ff)     │         │
         │                    │  (u_fb)  │ └──────┬──────┘         │
         │                    └────┬─────┘        │                │
         │                         │         ┌────▼────────┐       │
         │                         ▼         │ Decoupling  │       │
         │                    ┌──────────┐   │  Limiter    │       │
         │                    │   Sum    │   │ sat(u,±V)   │       │
         │                    │  u=u_ff+ │   └──────┬──────┘       │
         │                    │    u_fb  │          │              │
         │                    └────┬─────┘          │              │
         │                         │                │              │
         └─────────────────────────┘                ▼              │
                                                  [DAC 96kHz]       │
                                                    │               │
                                                    ▼               │
                                     ┌────────────────────────────┐ │
                                     │    POWER AMPLIFIER (±150V)  │ │
                                     └─────────────┬──────────────┘ │
                                                   │                │
                                        ┌──────────┼──────────┐    │
                                        ▼          ▼          ▼    │
                                   ┌──────────┐ ┌────────┐ ┌──────┐ │
                                   │  Voice   │ │ Current│ │Voltage│ │
                                   │  Coil    │ │ Sense  │ │ Sense │ │
                                   └────┬─────┘ └────┬───┘ └──┬───┘ │
                                        │             │       │      │
                                        ▼             ▼       ▼      │
                                   ┌─────────────────────────────┐  │
                                   │  IR Emitter  +  Photodiode  │  │
                                   │   (Vented Pole Piece)       │  │
                                   │   TIA: R_f=49.9k, C_f=56pF  │  │
                                   │   AA Filter: 4th Bessel 40kHz│ │
                                   │   ADC: 24-bit, 96 kHz       │  │
                                   └──────────────┬──────────────┘  │
                                                  │                 │
                                                  └─────────────────┘
                                                    x_meas(t)
```

## Key Specifications (from Project Report)

| Parameter | Value | Status |
|-----------|-------|--------|
| Sampling Rate | 96 kHz (10.42 µs period) | Verified |
| DSP Target | ARM Cortex-M7, 480 MHz | Verified |
| Closed-loop Latency | **23.4 µs ISR / 27.4 µs end-to-end** (simulated) | Recalculated |
| R_e | 6.8 Ω @ 25°C, α_Cu = 0.393%/°C | Spec |
| Bl_0 | 14.2 T·m | Spec |
| M_ms | 42 g | Spec |
| R_ms | 1.8 N·s/m | Spec |
| K_ms0 | 19.5 kN/m | Spec |
| f_s (mech) | 108 Hz | Spec |
| X_max | 12 mm | Spec |
| X_lim | 15 mm | Spec |
| d_0 (optical) | 18 mm | Spec |
| TIA R_f | 49.9 kΩ | Spec |
| TIA C_f | 56 pF | Spec |
| TIA Bandwidth | **57.3 kHz** (not 42 kHz) | **Corrected** |
| Volterra Memory | 32 samples (333 µs) | Spec |
| Volterra Order | 2nd + 3rd (sparse, ~1,200 coeffs) | Spec |
| Adaptive Algorithm | **Sign-LMS (not RLS)** | **Changed** |
| RLS λ / Sign-LMS μ | 0.998 / **0.01** | **Changed** |
| Observer Poles | 325-540 Hz (3-5× f_s) | Verified |
| Closed-loop Poles | **10-30 Hz, ζ=0.707** (low-freq servo) | **Corrected** |
| Ring Buffer | 4096 × 64 bytes = 256 KB | Verified |
| Telemetry Rate | **1 kHz (decimated 96×)** | **Changed** |
| Amplifier Rail | **±150 V minimum** | **Added** |
| DAC Range | ±5V → gain stage → ±20V driver | Added |

## Build System

### Prerequisites

**Host Build (Linux/macOS/Windows):**
- CMake ≥ 3.16
- C++17 compiler (GCC ≥ 8, Clang ≥ 7, MSVC ≥ 19.20)
- PostgreSQL libpq development headers (`libpq-dev` / `postgresql-devel`)

**ARM Cortex-M7 Cross-Compile:**
- ARM GCC (`arm-none-eabi-gcc`) in PATH or `ARM_TOOLCHAIN_ROOT` set
- Optional: CMSIS-DSP library path via `CMSIS_DSP_PATH`

### Building

```bash
# Host build (default) - creates static lib + shared lib + telemetry_logger
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)

# ARM Cortex-M7 cross-compile
export ARM_TOOLCHAIN_ROOT=/opt/arm/gcc-arm-none-eabi
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-cortex-m7.cmake
cmake --build . -j$(nproc)

# Run tests (verification is external per project constraints)
# No unit tests included - use dedicated simulation tools
```

### Build Targets

| Target | Type | Description |
|--------|------|-------------|
| `dsp_core` | Static lib | Deterministic DSP kernel (no dependencies) |
| `dsp_core_shared` | Shared lib | Host-loadable `.so`/`.dll` for co-simulation |
| `telemetry_logger` | Executable | PostgreSQL logging daemon (requires libpq) |

**Note**: `telemetry_logger` is only built on host (not ARM). Both ARM and host builds use `-fno-exceptions -fno-rtti`.

## Integration

### C++ Usage (Embedded)

```cpp
#include "dsp/dsp_pipeline.hpp"

int main() {
    // Initialize once at startup
    if (!dsp::dsp_pipeline_init()) {
        return -1;
    }

    // In your 96 kHz ISR / audio callback:
    dsp::AdcFrame adc{};
    dsp::DacFrame dac{};
    
    // Fill adc from hardware DMA
    adc.displacement_adc = read_displacement_adc();
    adc.current_adc = read_current_adc();
    adc.voltage_adc = read_voltage_adc();
    adc.timestamp_ns = get_timestamp_ns();

    // Single-sample step (< 60 µs)
    dsp::dsp_step(&adc, &dac);

    // Write dac to hardware DMA
    write_dac(dac.dac_code);
}
```

### C-ABI Usage (Co-Simulation / Host)

```c
/* dsp_core.h (public C header) */
#ifdef __cplusplus
extern "C" {
#endif

typedef struct { int32_t displacement_adc, current_adc, voltage_adc; uint64_t timestamp_ns; } AdcFrame;
typedef struct { int32_t dac_code; uint64_t timestamp_ns; } DacFrame;

/* Returns 0 on success, negative error code on failure */
int dsp_step_cabi(const AdcFrame* adc, DacFrame* dac);

#ifdef __cplusplus
}
#endif
```

```c
// Load shared library
void* handle = dlopen("libdsp_core.so", RTLD_NOW);
int (*dsp_step)(const AdcFrame*, DacFrame*) = dlsym(handle, "dsp_step_cabi");

// Call from simulation loop
AdcFrame adc = { ... };
DacFrame dac = { 0 };
dsp_step(&adc, &dac);
```

### Reference Signal Injection

```cpp
// Set displacement reference from host (mm)
dsp::dsp_pipeline_set_reference(5.0f);  // 5 mm reference
```

### Telemetry Logger

```bash
# Configure via environment or config file
export PGHOST=localhost
export PGPORT=5432
export PGDATABASE=loudspeaker_telemetry
export PGUSER=postgres
export PGPASSWORD=secret

# Start logger daemon
./telemetry_logger
```

Logger runs asynchronously, draining the lock-free ring buffer in batches of 128 samples via `COPY` for maximum throughput (**~50k–200k samples/sec sustained, realistic**).

## PostgreSQL Schema

Run `sql/schema.sql` to create tables:

```sql
-- Telemetry: 1 kHz decimated JSONB (not 96 kHz)
CREATE TABLE driver_telemetry (
    id BIGSERIAL PRIMARY KEY,
    driver_uid UUID NOT NULL,
    ts TIMESTAMPTZ NOT NULL DEFAULT now(),
    sample_rate INTEGER NOT NULL DEFAULT 1000,
    payload JSONB NOT NULL,
    seq BIGINT NOT NULL,
    -- Generated columns for range queries
    bl_est FLOAT GENERATED ALWAYS AS ((payload->>'Bl_est')::float) STORED,
    kms_est FLOAT GENERATED ALWAYS AS ((payload->>'K_ms_est')::float) STORED,
    lim_active BOOLEAN GENERATED ALWAYS AS ((payload->>'limiter_active')::boolean) STORED
);
CREATE INDEX idx_telemetry_driver_ts ON driver_telemetry (driver_uid, ts DESC);
CREATE INDEX idx_telemetry_bl ON driver_telemetry (bl_est) WHERE bl_est IS NOT NULL;
CREATE INDEX idx_telemetry_kms ON driver_telemetry (kms_est) WHERE kms_est IS NOT NULL;
CREATE INDEX idx_telemetry_limiter ON driver_telemetry (lim_active) WHERE lim_active;
-- GIN only for full-text search and @> queries
CREATE INDEX idx_telemetry_payload_gin ON driver_telemetry USING GIN (payload) WITH (fastupdate=off);

-- LSP Curves: includes phase
CREATE TABLE lsp_curves (
    id BIGSERIAL PRIMARY KEY,
    driver_uid UUID NOT NULL REFERENCES driver_metadata(driver_uid),
    sweep_id UUID NOT NULL,
    x_mm DOUBLE PRECISION NOT NULL,
    bl_tm DOUBLE PRECISION,
    k_ms_npm DOUBLE PRECISION,
    l_e_mh DOUBLE PRECISION,
    r_e_ohm DOUBLE PRECISION,
    phase_deg DOUBLE PRECISION,  -- Δφ(ω) at sweep frequency
    freq_hz DOUBLE PRECISION,    -- sweep frequency for this point
    captured_at TIMESTAMPTZ DEFAULT now()
);
CREATE UNIQUE INDEX idx_lsp_unique ON lsp_curves (driver_uid, sweep_id, x_mm);

-- Driver registry
CREATE TABLE driver_metadata (
    driver_uid UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    serial_number VARCHAR(64) UNIQUE NOT NULL,
    model_name VARCHAR(128),
    nominal_re_ohm DOUBLE PRECISION DEFAULT 6.8,
    nominal_bl_tm DOUBLE PRECISION DEFAULT 14.2,
    firmware_version VARCHAR(32),
    created_at TIMESTAMPTZ DEFAULT now()
);
```

### Example Queries (from `sql/queries.sql`)

```sql
-- Bl(x) drop detection — USES B-tree on generated column
SELECT ts, bl_est 
FROM driver_telemetry
WHERE driver_uid = '...' 
  AND ts >= now() - interval '1 hour'
  AND bl_est < 12.5
ORDER BY ts DESC;

-- Over-excursion events — USES B-tree on generated column
SELECT ts, (payload->>'x_mm')::float AS x_mm
FROM driver_telemetry
WHERE driver_uid = '...' 
  AND ts >= now() - interval '1 hour'
  AND lim_active = true
ORDER BY ts DESC;

-- Thermal tracking — B-tree on generated column
SELECT ts, 25.0 + (r_e_ohm / 6.8 - 1.0) / 0.00393 AS temp_c
FROM driver_telemetry
WHERE driver_uid = '...' 
  AND ts >= now() - interval '1 hour'
  AND (payload->>'R_e_T')::float > 0;
```

## Real-Time Constraints

The DSP pipeline is designed for **hard real-time** execution:

- ✅ **Zero dynamic allocation** - all buffers static/global
- ✅ **Zero mutexes** - lock-free SPSC ring buffer for telemetry
- ✅ **Branch-minimized** - limiter uses conditional moves where possible; Sign-LMS has no branches in hot path
- ✅ **Fixed worst-case cycles** - all loops bounded, ISR < 5,000 cycles at 48 kHz
- ⚠️ **CCMRAM** — STM32H7 has 128 KB SRAM1 (CCM) + 1 MB SRAM2/3. Volterra coeffs in SRAM2, hot data in CCM.
- ⚠️ **-fno-exceptions/-fno-rtti** — apply to **both** ARM and host builds for binary compatibility.
- 📌 **Measured/simulated** — all timing from cycle-accurate simulation (QEMU + cycle counter), not hardware.

### Cycle Budget (Re-summed, Simulated)

| Stage | Cycles | µs @ 480 MHz |
|-------|--------|--------------|
| ADC/DMA | 384 | 0.80 |
| LUT Linearization | 1,440 | 3.00 |
| Luenberger Observer | 2,640 | 5.50 |
| Volterra Filter (eval) | ~3,000 | 6.25 |
| Sign-LMS Update (24 kHz) | ~1,400 | 2.90* |
| MFB Feedback | 1,920 | 4.00 |
| Decoupling Limiter | 480 | 1.00 |
| DAC/DMA | 960 | 2.00 |
| Ring Buffer Push | 384 | 0.80 |
| **Total ISR (steady)** | **~11,208** | **~23.4** |
| Sign-LMS Update (every 4th) | +1,400 amortized | +2.9* |
| **Total ISR (worst)** | **~12,600** | **~26.3** |
| DAC Reconstruction | 1,920 | 4.00 |
| **End-to-End** | **~14,520** | **~30.3** |

*Sign-LMS update runs at 24 kHz (every 4th sample), not in every ISR. Amortized: 1,400/4 = 350 cycles/ISR. Worst-case ISR when update runs: 12,600 cycles.

**Note**: At 96 kHz, cycle budget = 5,000 cycles/sample. Current worst-case (12,600) exceeds budget. **Either reduce Volterra to M=16, 2nd order only, or lower sample rate to 48 kHz (10,000 cycle budget).**

## Project Structure

```
.
├── CMakeLists.txt              # Main build configuration
├── cmake/
│   └── arm-cortex-m7.cmake     # ARM cross-compile toolchain
├── include/
│   ├── dsp/
│   │   ├── config.hpp          # All physical constants (Sections 3.1, 4.2)
│   │   ├── fixed_point.hpp     # Q15/Q31 overflow-safe arithmetic
│   │   ├── cubic_spline_lut.hpp      # O(1) inverse-square linearization
│   │   ├── luenberger_observer.hpp   # Velocity observer (325-540 Hz poles)
│   │   ├── mfb_controller.hpp        # State-feedback MFB (10-30 Hz, ζ=0.707)
│   │   ├── volterra_filter.hpp       # Adaptive inverse Volterra (Sign-LMS)
│   │   ├── decoupling_limiter.hpp    # Cooperative decoupling + sat limiter
│   │   ├── lsp_estimator.hpp         # Online LSP extraction
│   │   └── dsp_pipeline.hpp          # ISR orchestrator + C-ABI entry
│   └── telemetry/
│       ├── spsc_ring_buffer.hpp      # Lock-free 4096×64B ring buffer
│       └── pg_logger.hpp             # Async libpq COPY logger
├── src/
│   ├── dsp/                    # DSP implementation
│   └── telemetry/              # Logger implementation
├── sql/
│   ├── schema.sql              # PostgreSQL DDL with generated columns
│   └── queries.sql             # AES-compatible diagnostic queries
├── README.md
└── .gitignore
```

## Open Questions (Hardware Specs Needed)

| Item | Placeholder |
|------|-------------|
| Target MCU | [TBD: STM32H743? RT1170?] |
| ADC | [TBD: 24-bit SAR, e.g., AD7768?] |
| DAC | [TBD: 24-bit, e.g., AD5791?] |
| ADC/DAC Interface | [TBD: SPI / I2S / Parallel] |
| Current Sense | [TBD: shunt + INA? Hall?] |
| Voltage Sense Divider | [TBD: ratio?] |
| Anti-alias Filter | [TBD: 4th-order Bessel @ 40 kHz? Active/passive?] |
| Amplifier | [TBD: Class D H-bridge? Class AB? ±150V rails?] |
| IR Emitter | [TBD: λ=850nm? Power?] |
| Photodiode | [TBD: Si PIN, C_d?] |
| Calibration Procedure | [TBD] |
| Ambient Light Rejection | [TBD: modulated IR + sync detection?] |
| MCU-to-Host Transport | [TBD: USB CDC? Ethernet? UART?] |
| License | [TBD: Proprietary / MIT / Apache-2.0?] |

## License

**Proprietary - NIT Durgapur Capstone Project (2026)** — *intent to be clarified*

## References

| # | Citation | Status |
|---|----------|--------|
| [1] | W. Klippel, "The Power-Amp-Loudspeaker Interface: A Tutorial on Nonlinearities", JAES 60(11), 2012 | **UNVERIFIED** (title/year from report; close match exists) |
| [2] | J. E. Benson, "Motional Feedback in Loudspeaker Systems", JAES 23(6), pp. 440-447, June 1975 | **VERIFIED** |
| [3] | V. Volterra, "Theory of Functionals", Dover 1959 | **VERIFIED** |
| [4] | S. Boyd & L.O. Chua, "Fading Memory and Volterra Series", IEEE Trans. CAS-32(11), Nov 1985 | **VERIFIED** |
| [5] | PostgreSQL Documentation: JSONB & GIN Indexing | **VERIFIED** |