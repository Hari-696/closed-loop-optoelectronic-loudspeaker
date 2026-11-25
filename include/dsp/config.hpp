#pragma once

/**
 * @file config.hpp
 * @brief Exact constants from Sections 3.1 & 4.2 of the project specification.
 * 
 * All physical parameters, control hyperparameters, and system constants
 * are defined here as constexpr for compile-time optimization.
 */

#include <cstdint>
#include <cstddef>

namespace dsp {

// ============================================================================
// Sampling & Timing
// ============================================================================
constexpr float F_S = 96000.0f;                    ///< Sampling frequency [Hz]
constexpr float T_S = 1.0f / F_S;                  ///< Sample period [s] = 10.4167 µs
constexpr uint32_t ISR_BUDGET_US = 100;            ///< End-to-end latency budget [µs]
constexpr uint32_t ISR_TARGET_US = 60;             ///< Target worst-case ISR execution [µs]

// ============================================================================
// Transducer Physical Parameters (Section 3.1)
// ============================================================================
constexpr float R_E_25C = 6.8f;                    ///< DC resistance at 25°C [Ω]
constexpr float ALPHA_CU = 0.00393f;               ///< Copper temp coefficient [%/°C] = 0.393%/°C
constexpr float BL_0 = 14.2f;                      ///< Force factor at rest [T·m]
constexpr float M_MS = 0.042f;                     ///< Moving mass [kg] = 42 g
constexpr float R_MS = 1.8f;                       ///< Mechanical resistance [N·s/m]
constexpr float K_MS0 = 1.95e4f;                   ///< Rest suspension stiffness [N/m]
constexpr float F_S_MECH = 108.0f;                 ///< Mechanical resonance [Hz] (open loop, small signal)
constexpr float X_MAX = 0.012f;                    ///< Maximum linear excursion [m] = 12 mm
constexpr float X_LIM = 0.015f;                    ///< Hard mechanical bottoming limit [m] = 15 mm

// Derived mechanical parameters
constexpr float W_S_MECH = 2.0f * 3.14159265358979323846f * F_S_MECH;  ///< Mechanical radian freq [rad/s]
constexpr float C_MS0 = 1.0f / K_MS0;              ///< Rest compliance [m/N]

// ============================================================================
// Optoelectronic Sensor Parameters (Section 3.1)
// ============================================================================
constexpr float D_0 = 0.018f;                      ///< Optical gap standoff [m] = 18 mm
constexpr float LAMBDA_OPT = 850e-9f;              ///< IR wavelength [m] = 850 nm
constexpr float PHOTODIODE_RESPONSIVITY = 0.55f;   ///< A/W at 850 nm
constexpr float PHOTODIODE_AREA = 1e-6f;           ///< Active area [m²] = 1 mm²

// ============================================================================
// Analog Front-End Parameters (Section 3.1)
// ============================================================================
constexpr float R_F = 49900.0f;                    ///< TIA feedback resistor [Ω] = 49.9 kΩ
constexpr float C_F = 56e-12f;                     ///< TIA feedback capacitance [F] = 56 pF
constexpr int AA_FILTER_ORDER = 6;                 ///< 6th-order elliptic
constexpr float AA_FILTER_FPASS_HZ = 22000.0f;     ///< Passband edge [Hz] = 22 kHz (0.1 dB ripple)
constexpr float AA_FILTER_RIPPLE_DB = 0.1f;        ///< Passband ripple [dB]
constexpr float AA_FILTER_STOP_DB = 80.0f;         ///< Stopband attenuation [dB]
constexpr float AA_FILTER_FSTOP_HZ = 48000.0f;     ///< Stopband edge [Hz] = 48 kHz (Nyquist)
constexpr float I_PHOTODIODE_REST = 2.7e-6f;       ///< Photocurrent at d_0 = 18 mm [A] = 2.7 µA

// ============================================================================
// Volterra Filter Parameters (Section 4.2)
// ============================================================================
constexpr size_t VOLTERRA_MEMORY_M = 32;           ///< Memory depth [samples] = 333 µs at 96 kHz
constexpr size_t VOLTERRA_ORDER = 3;               ///< Maximum Volterra order (2nd + 3rd)
constexpr float RLS_LAMBDA = 0.998f;               ///< RLS forgetting factor (τ ≈ 500 samples = 5.2 ms)
constexpr size_t RLS_UPDATE_DECIMATION = 4;        ///< Update every 4th sample (24 kHz)
constexpr int VOLTERRA_SPARSITY_WINDOW = 4;        ///< |m_i - m_j| ≤ 4 for 3rd-order terms
// ~1,200 active 3rd-order coefficients after sparsification

// ============================================================================
// Luenberger Observer Parameters (Section 4.2)
// ============================================================================
constexpr float OBSERVER_POLE_MIN_HZ = 325.0f;     ///< Observer pole min [Hz] (3-5× mechanical resonance)
constexpr float OBSERVER_POLE_MAX_HZ = 540.0f;     ///< Observer pole max [Hz]
// Discrete poles computed at runtime from continuous-time placement

// ============================================================================
// MFB Controller Parameters (Section 4.2)
// ============================================================================
constexpr float CL_POLE_MIN_HZ = 60.0f;            ///< Closed-loop pole min [Hz]
constexpr float CL_POLE_MAX_HZ = 90.0f;            ///< Closed-loop pole max [Hz]
constexpr float CL_DAMPING_ZETA = 0.707f;          ///< Closed-loop damping ratio ζ = 0.707

// ============================================================================
// Over-Excursion Limiter Parameters
// ============================================================================
constexpr float LIMITER_KNEE_RATIO = 0.9f;         ///< Soft limiting starts at 90% X_MAX
constexpr float LIMITER_HARD_RATIO = 1.0f;         ///< Hard saturation at X_LIM

// ============================================================================
// Telemetry Ring Buffer (Section 5.4)
// ============================================================================
constexpr size_t TELEMETRY_RING_DEPTH = 4096;      ///< Ring buffer depth [samples] ≈ 42.7 ms at 96 kHz
constexpr size_t TELEMETRY_SAMPLE_SIZE = 64;       ///< Bytes per sample (cache-line aligned)
constexpr size_t TELEMETRY_BATCH_SIZE = 128;       ///< Batch size for PostgreSQL COPY/INSERT

// ============================================================================
// LSP Estimator Parameters
// ============================================================================
constexpr float LSP_BL_OFFSET_THRESHOLD = 0.05f;   ///< Bl asymmetry detection threshold [T·m]
constexpr float LSP_KMS_ASYMMETRY_THRESHOLD = 0.05f; ///< K_ms asymmetry threshold [fractional]
constexpr float LSP_THERMAL_UPDATE_INTERVAL_MS = 100.0f; ///< Thermal parameter update interval [ms]

// ============================================================================
// Mathematical Constants
// ============================================================================
constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 2.0f * PI;
constexpr float SQRT_2 = 1.4142135623730951f;
constexpr float INV_SQRT_2 = 0.7071067811865475f;

// ============================================================================
// ADC/DAC Scaling (Section 3.1: 24-bit, ±5V range)
// ============================================================================
constexpr float ADC_VREF = 5.0f;                   ///< ADC reference voltage [V]
constexpr float DAC_VREF = 5.0f;                   ///< DAC reference voltage [V]
constexpr float ADC_FULL_SCALE = 8388608.0f;       ///< 2^23 (24-bit signed)
constexpr float DAC_FULL_SCALE = 8388608.0f;       ///< 2^23 (24-bit signed)
constexpr float VOLTS_PER_LSB = ADC_VREF / ADC_FULL_SCALE;

// ============================================================================
// Amplifier & Voice Coil Scaling (Q31 normalization)
// ============================================================================
// Q31 represents normalized ±1.0 = full scale
// ADC/DAC: ±1.0 Q31 = ±5 V (ADC/DAC range)
// Amplifier: ±1.0 Q31 = ±150 V (voice coil drive)
constexpr float VOLTAGE_FULL_SCALE_V = 5.0f;       ///< ADC/DAC full-scale [V]
constexpr float AMP_FULL_SCALE_V = 150.0f;         ///< Amplifier/voice coil full-scale [V]

// ============================================================================
// Displacement Scaling (maps ±X_LIM to ±5V ADC range)
// ============================================================================
constexpr float DISPLACEMENT_SCALE = ADC_VREF / X_LIM;  ///< V/m = 5V / 0.015m = 333.33 V/m
constexpr float DISPLACEMENT_OFFSET = 0.0f;              ///< Zero at center

// ============================================================================
// Current Sensing Scaling (assumes 0.1Ω shunt, 50× gain INA)
// ============================================================================
constexpr float CURRENT_SENSE_GAIN = 50.0f;        ///< INA gain
constexpr float SHUNT_RESISTANCE = 0.1f;           ///< Shunt [Ω]
constexpr float CURRENT_SCALE = CURRENT_SENSE_GAIN * SHUNT_RESISTANCE;  ///< V/A = 5 V/A

// ============================================================================
// Voltage Sensing Scaling (assumes 10:1 divider)
// ============================================================================
constexpr float VOLTAGE_DIVIDER_RATIO = 10.0f;     ///< 10:1 divider
constexpr float VOLTAGE_SCALE = VOLTAGE_DIVIDER_RATIO;  ///< V/V

// ============================================================================
// Helper Macros
// ============================================================================
#define DSP_INLINE inline __attribute__((always_inline))
#define DSP_NOINLINE __attribute__((noinline))
#define DSP_RAMFUNC __attribute__((section(".ccmram")))  ///< For Cortex-M7 CCMRAM placement

// DLL export/import macro for Windows shared library
#if defined(_WIN32) || defined(__CYGWIN__)
  #ifdef DSP_CORE_EXPORTS
    #define DSP_API __declspec(dllexport)
  #else
    #define DSP_API __declspec(dllimport)
  #endif
#else
  #define DSP_API
#endif

// Compile-time assertions
static_assert(VOLTERRA_MEMORY_M == 32, "Volterra memory depth must be 32");
static_assert(TELEMETRY_RING_DEPTH == 4096, "Ring buffer depth must be 4096");
static_assert(TELEMETRY_SAMPLE_SIZE == 64, "Telemetry sample must be 64 bytes");
static_assert(RLS_UPDATE_DECIMATION == 4, "RLS update decimation must be 4");
static_assert(VOLTERRA_SPARSITY_WINDOW == 4, "Volterra sparsity window must be 4");

} // namespace dsp