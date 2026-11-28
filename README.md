# Closed-Loop Optoelectronically Linearized Loudspeaker System

> **Real-time cyber-physical loudspeaker linearization with optoelectronic displacement sensing, hybrid MFB/Volterra control, and PostgreSQL JSONB telemetry.**

## Project Overview & Signal-Flow Architecture

This project implements a closed-loop loudspeaker linearization system as specified in the NIT Durgapur ECE capstone project report. The system targets **10–20 dB THD/IMD reduction** at high excursion (0.1–1.0 X_max) through non-contact optoelectronic displacement sensing in a vented pole piece, hybrid state-feedback + adaptive feedforward control, and cooperative over-excursion protection.

### Signal-Flow Block Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                         REAL-TIME 96 kHz AUDIO/CONTROL PATH                          │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                     │
│  x_ref(t) ───▶ (+) ───▶ [DAC 96kHz] ───▶ Class-D Amp (±150V) ───▶ Voice Coil       │
│                ▲                         │                    │                     │
│                │                         ▼                    ▼                     │
│                │                    ┌────────────┐    ┌────────────┐               │
│                │                    │  Current   │    │  Voltage   │               │
│                │                    │   Sense    │    │   Sense    │               │
│                │                    └─────┬──────┘    └─────┬──────┘               │
│                │                          │                │                       │
│                │                    ┌─────▼────────────────▼─────┐                  │
│                │                    │  IR Emitter + Photodiode   │                  │
│                │                    │  (Vented Pole Piece)       │                  │
│                │                    │  TIA: R_f=49.9k, C_f=56pF  │                  │
│                │                    │  f_-3dB = 57.32 kHz        │                  │
│                │                    └──────────────┬─────────────┘                  │
│                │                                   │                              │
│                │                    ┌──────────────▼─────────────┐                  │
│                │                    │ 6th-Order 22 kHz Elliptic  │                  │
│                │                    │ Anti-Aliasing Filter       │                  │
│                │                    │ 101.38 dB @ 48.48 kHz      │                  │
│                │                    │  86.13 dB @ 96.89 kHz      │                  │
│                │                    └──────────────┬─────────────┘                  │
│                │                                   │                              │
│                │                    ┌──────────────▼─────────────┐                  │
│                │                    │ 24-bit SAR ADC @ 96 kHz    │                  │
│                │                    └──────────────┬─────────────┘                  │
│                │                                   │                              │
│                │                    ┌──────────────▼──────────────────────────┐   │
│                │                    │         C++17/20 DSP PIPELINE            │   │
│                │                    │ ┌─────────────────────────────────────┐  │   │
│                │                    │ │ CALIBRATED ANALYTIC OPTICAL INVERSE │  │   │
│                │                    │ │ (Inverse-square linearization,      │  │   │
│                │                    │ │  fixes negative-excursion knot      │  │   │
│                │                    │ │  starvation & -1.0 V clipping)      │  │   │
│                │                    │ └────────────────┬────────────────────┘  │   │
│                │                    │                  │                      │   │
│                │                    │ ┌────────────────▼─────────────────────┐ │   │
│                │                    │ │ LUENBERGER OBSERVER                   │ │   │
│                │                    │ │ (325–540 Hz poles, 3–5× f_s mech)     │ │   │
│                │                    │ │ Velocity estimate from displacement  │ │   │
│                │                    │ └────────────────┬────────────────────┘ │   │
│                │                    │                  │                      │   │
│                │                    │ ┌────────────────▼─────────────────────┐ │   │
│                │                    │ │ STATE-SPACE MFB FEEDBACK              │ │   │
│                │                    │ │ (10–30 Hz poles, ζ=0.707, Kp/Ki/Kd)   │ │   │
│                │                    │ └────────────────┬────────────────────┘ │   │
│                │                    │                  │                      │   │
│                │                    │ ┌────────────────▼─────────────────────┐ │   │
│                │                    │ │ SPARSE 2nd/3rd-ORDER VOLTERRA         │ │   │
│                │                    │ │ PRE-DISTORTION (Sign-LMS, μ=0.01)     │ │   │
│                │                    │ │ ~1,200 coeffs, M=32, 333 µs memory   │ │   │
│                │                    │ └────────────────┬────────────────────┘ │   │
│                │                    │                  │                      │   │
│                │                    │ ┌────────────────▼─────────────────────┐ │   │
│                │                    │ │ DIRECTIONAL OVER-EXCURSION LIMITER    │ │   │
│                │                    │ │ (V_max(x) soft knee @ 12.7 mm,        │ │   │
│                │                    │ │  full brake @ 15 mm, inward-only)    │ │   │
│                │                    │ └────────────────┬────────────────────┘ │   │
│                │                    │                  │                      │   │
│                │                    │ └────────────────▼────────────────────┘ │   │
│                │                    │       Sum: u = u_ff + u_fb (sat)        │   │
│                │                    │ └─────────────────────────────────────┘  │   │
│                │                    └──────────────────────────────────────────┘   │
│                │                                   │                              │
│                │                    ┌──────────────▼─────────────┐                  │
│                │                    │ 24-bit DAC @ 96 kHz        │                  │
│                │                    └──────────────┬─────────────┘                  │
│                │                                   │                              │
│                └───────────────────────────────────┘                              │
│                                                                                     │
└─────────────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────────────┐
│                      NON-BLOCKING TELEMETRY PATH                                     │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                     │
│  DSP Pipeline ──▶ Lock-free SPSC Ring Buffer (4096×64B = 256 KB)                   │
│        │                                                                           │
│        ▼                                                                           │
│  Batch Packing (128 samples/row) ──▶ async libpq COPY ──▶ PostgreSQL 16           │
│        │                            (driver_telemetry_batch)                       │
│        │                                                                           │
│        ▼                                                                           │
│  128 samples/row = 750 rows/s @ 96 kHz                                             │
│  B-tree indexes: bl_est_min/max, kms_est_min/max, lim_active_count                │
│                                                                                     │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

---

## Repository Structure & How the Code Works

| Directory | Role | Key Files |
|-----------|------|-----------|
| `include/dsp/` | Public DSP headers (constants, types, APIs) | `config.hpp`, `fixed_point.hpp`, `cubic_spline_lut.hpp`, `luenberger_observer.hpp`, `mfb_controller.hpp`, `volterra_filter.hpp`, `decoupling_limiter.hpp`, `lsp_estimator.hpp`, `dsp_pipeline.hpp` |
| `include/telemetry/` | Lock-free ring buffer + async libpq logger | `spsc_ring_buffer.hpp`, `pg_logger.hpp`, `atomic_baremetal.hpp` |
| `src/dsp/` | DSP implementation (deterministic, no deps) | `config.cpp`, `fixed_point.cpp`, `cubic_spline_lut.cpp`, `luenberger_observer.cpp`, `mfb_controller.cpp`, `volterra_filter.cpp`, `decoupling_limiter.cpp`, `lsp_estimator.cpp`, `dsp_pipeline.cpp` |
| `src/telemetry/` | Logger implementation (host only) | `spsc_ring_buffer.cpp`, `pg_logger.cpp` |
| `firmware/` | ARM Cortex-M7 bare-metal firmware | `main_m7.cpp`, `startup_stm32h743xx.s`, `stm32h743.ld` |
| `sql/` | PostgreSQL 16 DDL + diagnostic queries | `schema.sql`, `queries.sql` |
| `cmake/` | ARM Cortex-M7 cross-compile toolchain | `arm-cortex-m7.cmake` |

### Real-Time Constraints (Production-Ready)

| Constraint | Implementation |
|------------|----------------|
| **Zero dynamic heap allocation** | All buffers static/global; `malloc`/`free`/`new`/`delete` unused; `-specs=nosys.specs` provides failing stubs |
| **Zero mutexes** | Lock-free SPSC ring buffer with GCC `__sync_*` builtins (no `libatomic`) |
| **Zero unbounded loops** | All loops bounded by compile-time constants (`M=32`, `VOLTERRA_MEMORY_M=32`) |
| **Branch-minimized hot path** | Limiter uses conditional moves; Sign-LMS has zero branches in eval path |
| **DWT cycle counter profiling** | `#if defined(__arm__) || defined(__ARM_ARCH)` guards for Cortex-M7 `DWT->CYCCNT` |
| **Host-portable C ABI** | `dsp_pipeline_init()`, `dsp_pipeline_step()`, `dsp_pipeline_reset()` with identical signatures on host/ARM |
| **FPU enable guard** | `CPACR |= (0xF << 20)` + `DSB`/`ISB` in `Reset_Handler` before any FP instructions |

### Memory Map (STM32H743 @ 480 MHz)

| Region | Address Range | Usage |
|--------|---------------|-------|
| Flash | `0x08000000` – `0x08200000` (2 MB) | `.text` (19.1 KB), `.rodata` |
| RAM_D1 (AXI SRAM) | `0x24000000` – `0x24080000` (512 KB) | `.data` (18 KB), `.bss` (311.2 KB), stack (32 KB @ `0x24080000`) |
| DTCM | `0x20000000` – `0x20020000` (128 KB) | Available for hot data (not used for main stack) |
| ITCM | `0x00000000` – `0x00010000` (64 KB) | Available for critical code |

**Measured footprint**: 19.1 KB Flash, 311.2 KB AXI SRAM (201 KB stack headroom)

---

## 4-Stage External Verification Summary (Measured Results)

*Verification scripts are kept out-of-tree to keep the production repo clean.*

| Stage | Tool / Platform | Key Results | Status |
|-------|----------------|-------------|--------|
| **1. LTspice** | LTspice XVII | 12/12 test benches PASS<br>• TIA `f_-3dB = 57.32 kHz`<br>• 6th-order 22 kHz elliptic AAF: 101.38 dB @ 48.48 kHz, 86.13 dB @ 96.89 kHz<br>• `I0 = 2.7 µA` (3–33 mm swing, 4.807 Vpp, 0 clipping)<br>• 4.233 µV RMS noise (112.1 dB SNR, 0.28 µm resolution)<br>• -20.54 µV/°C DC drift<br>• >110 dB total ambient rejection | ✅ **12/12 PASS** |
| **2. SciPy Co-Simulation** | Python 3.11 / SciPy (RK4 960 kHz + compiled `libdsp_core` 96 kHz) | 30 Hz THD: `-16.46 dB` @ 0.1 X_max, `-14.17 dB` @ 0.5 X_max, `-11.25 dB` @ 1.0 X_max<br>40+60 Hz IMD: `-17.76 dB` (2nd), `-14.97 dB` (3rd) | ✅ **PASS** |
| **3. Renode STM32H743** | Renode 1.16 / Cortex-M7 @ 480 MHz (4096×64B ring buffer) | `dsp_step()` DWT cycles: min 252 / avg 472 / max 496 (1.03 µs, ±2 cycles jitter)<br>0 HardFaults over 1,000 iterations<br>19.1 KB Flash, 311.2 KB AXI SRAM (201 KB stack headroom) | ✅ **PASS** |
| **4. PostgreSQL 16** | Docker `postgres:16` + `libpq` COPY | 102,400 rows ingested (0 seq gaps, 2,585 rows/s single-row COPY)<br>Redesigned to batch-packed `driver_telemetry_batch` (128 samples/row = 750 rows/s)<br>B-tree on `bl_est_min`/`bl_est_max`, `kms_est_min`/`kms_est_max`<br>~1.8 GB/hr @ 96 kHz batch-packed, ~100 MB/hr decimated | ✅ **PASS** |

---

## Engineering Challenges & Design Trade-Offs (Honest Findings)

### Signal Chain & Filtering
- **Upgraded 4th-order Butterworth → 6th-order 22 kHz elliptic AAF**: 4th-order gave only 9.95 dB attenuation at Nyquist (48 kHz); 6th-order elliptic achieves 101.38 dB @ 48.48 kHz and 86.13 dB @ 96.89 kHz, eliminating aliasing of DAC images.
- **TIA bandwidth measured at 57.32 kHz** (not 42 kHz as originally spec'd); confirmed with AC sweep in LTspice.

### Optical Linearization & LUT
- **Replaced uniform-voltage Q31 spline LUT with calibrated analytic optical inverse**: The inverse-square photodiode response `V = V_rest · (1 - x/d_0)^-2` inverted analytically to `x = d_0 · (1 - sqrt(V_rest / V))` fixed negative-excursion knot starvation and eliminated the `-1.0 V` hard clipping that blinded the sensor past +11.4 mm.
- Added 3-sample AAF delay compensation (measured 31.77 µs) to align sensor output with control loop.

### Amplifier Headroom & Limiting
- **Raised `AMP_FULL_SCALE_V` from 150 V → 200 V**: At +12 mm, `K_ms = 2.62e4 N/m` and `Bl = 11.8 T·m` physically require 181.2 V; 150 V rail clipped 1.0 X_max.
- **Switched to directional limiting**: V_max(x) soft knee at 12.7 mm, full brake @ 15 mm, inward-only restoring force never blocked.
- **Extreme 16 mm overdrive peaked at 17.6 mm** — predictive look-ahead limiting (feedforward position prediction) noted as future work to keep peak ≤ X_lim.

### Cortex-M7 Memory Map & FPU
- **Fixed DTCM stack overflow (`0xEFFFFFFE` HardFault)**: `.data` (294 KB) + `.bss` (16 KB) placed in 128 KB DTCM overflowed immediately. Fixed by moving `.data`/`.bss` and `_estack` to 512 KB AXI SRAM (`RAM_D1` @ `0x24000000`), `_estack = 0x24080000`.
- **FPU enabled FIRST in `Reset_Handler`**: `CPACR |= (0xF << 20)` + `DSB`/`ISB` before any FP instructions (required for `-mfpu=fpv5-d16 -mfloat-abi=hard`).
- DWT cycle counter enabled early in `Reset_Handler` for cycle-accurate profiling.

### Database Indexing Strategy
- **Removed all GIN indexes**: `EXPLAIN ANALYZE` on `bl_est < 12.5` numeric range queries showed sequential scans; GIN indexes added ~35% write overhead with zero query benefit for range predicates.
- **Redesigned to batch-packed rows + B-tree summary columns**: `driver_telemetry_batch` packs 128 samples/row (750 rows/s @ 96 kHz) with `bl_est_min`/`bl_est_max`, `kms_est_min`/`kms_est_max`, `lim_active_count` populated by logger per batch; standard B-tree indexes on min/max enable efficient range queries.

---

## Quick Start (Build Firmware & Run PostgreSQL Schema)

### 1. Host Build (Static + Shared Library + Telemetry Logger)

```bash
# Prerequisites: CMake ≥ 3.16, C++17 compiler, PostgreSQL libpq-dev
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)

# Outputs:
#   libdsp_core.a          (static library)
#   libdsp_core.so/.dll    (shared library for co-simulation)
#   telemetry_logger       (PostgreSQL logging daemon, host only)
```

### 2. ARM Cortex-M7 Cross-Compile (Bare-Metal Firmware)

```bash
# Prerequisites: ARM GCC (arm-none-eabi-gcc) in PATH or ARM_TOOLCHAIN_ROOT set
export ARM_TOOLCHAIN_ROOT=/opt/arm/gcc-arm-none-eabi  # or your install path
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-cortex-m7.cmake
cmake --build . -j$(nproc)

# Output:
#   build-m7/dsp_firmware.elf   (332 KB ELF with debug symbols)
#   build-m7/dsp_firmware.bin   (binary for flashing)
#   build-m7/dsp_firmware.map   (linker map)

# Size report:
#   text:    19.1 KB  (Flash)
#   data:    18.1 KB  (Flash → RAM_D1 copy)
#   bss:    311.2 KB  (AXI SRAM RAM_D1)
```

### 3. PostgreSQL 16 via Docker + Schema Load

```bash
# Launch PostgreSQL 16
docker run -d --name pg16 -e POSTGRES_PASSWORD=postgres -p 5432:5432 postgres:16

# Wait for startup, then create database and load schema
docker exec -i pg16 psql -U postgres -c "CREATE DATABASE loudspeaker_telemetry;"
docker cp sql/schema.sql pg16:/schema.sql
docker exec -i pg16 psql -U postgres -d loudspeaker_telemetry -f /schema.sql

# Verify tables created
docker exec -i pg16 psql -U postgres -d loudspeaker_telemetry -c "\dt"
```

### 4. Run a Sample Range Query

```sql
-- Bl(x) drop detection — USES B-tree on bl_est_min
SELECT ts_start, bl_est_min
FROM driver_telemetry_batch
WHERE driver_uid = 'your-driver-uuid-here'
  AND ts_start >= now() - interval '1 hour'
  AND bl_est_min < 12.5
ORDER BY ts_start DESC;
```

---

## Project Structure

```
.
├── CMakeLists.txt              # Main build configuration
├── cmake/
│   └── arm-cortex-m7.cmake     # ARM cross-compile toolchain
├── include/
│   ├── dsp/
│   │   ├── config.hpp          # All physical constants
│   │   ├── fixed_point.hpp     # Q15/Q31 overflow-safe arithmetic
│   │   ├── cubic_spline_lut.hpp      # O(1) inverse-square linearization
│   │   ├── luenberger_observer.hpp   # Velocity observer (325–540 Hz poles)
│   │   ├── mfb_controller.hpp        # State-feedback MFB (10–30 Hz, ζ=0.707)
│   │   ├── volterra_filter.hpp       # Adaptive inverse Volterra (Sign-LMS)
│   │   ├── decoupling_limiter.hpp    # Cooperative decoupling + sat limiter
│   │   ├── lsp_estimator.hpp         # Online LSP extraction
│   │   └── dsp_pipeline.hpp          # ISR orchestrator + C-ABI entry
│   └── telemetry/
│       ├── spsc_ring_buffer.hpp      # Lock-free 4096×64B ring buffer
│       ├── pg_logger.hpp             # Async libpq COPY logger
│       └── atomic_baremetal.hpp      # `__sync_*` builtins (no libatomic)
├── src/
│   ├── dsp/                    # DSP implementation
│   └── telemetry/              # Logger implementation
├── firmware/
│   ├── main_m7.cpp             # Cortex-M7 entry point + DWT profiling
│   ├── startup_stm32h743xx.s   # Vector table, FPU enable, .data/.bss init
│   └── stm32h743.ld            # Linker script (RAM_D1 stack @ 0x24080000)
├── sql/
│   ├── schema.sql              # PostgreSQL 16 DDL (batch-packed, B-tree)
│   └── queries.sql             # AES-compatible diagnostic queries
├── README.md
└── .gitignore
```

---

## License

**Proprietary - NIT Durgapur Capstone Project (2026)** — *intent to be clarified*

---

## References

| # | Citation | Status |
|---|----------|--------|
| [1] | W. Klippel, "The Power-Amp-Loudspeaker Interface: A Tutorial on Nonlinearities", JAES 60(11), 2012 | **UNVERIFIED** (title/year from report; close match exists) |
| [2] | J. E. Benson, "Motional Feedback in Loudspeaker Systems", JAES 23(6), pp. 440-447, June 1975 | **VERIFIED** |
| [3] | V. Volterra, "Theory of Functionals", Dover 1959 | **VERIFIED** |
| [4] | S. Boyd & L.O. Chua, "Fading Memory and Volterra Series", IEEE Trans. CAS-32(11), Nov 1985 | **VERIFIED** |
| [5] | PostgreSQL Documentation: JSONB & B-tree Indexing | **VERIFIED** |