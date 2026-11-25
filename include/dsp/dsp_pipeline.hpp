#pragma once

/**
 * @file dsp_pipeline.hpp
 * @brief Single-sample ISR orchestrator chaining all DSP stages.
 * 
 * Pipeline: ADC → LUT → Observer → Volterra → MFB → Limiter → DAC → Ring Buffer
 * Worst-case execution < 60 µs at 96 kHz (10.42 µs sample period).
 * C-ABI entry point: dsp_step(const AdcFrame*, DacFrame*) for external co-simulation.
 */

#include <cstdint>
#include "config.hpp"
#include "fixed_point.hpp"

namespace dsp {

// ============================================================================
// Data Structures for C-ABI Interface
// ============================================================================

// ADC input frame (from hardware or simulator)
struct AdcFrame {
    int32_t displacement_adc;   ///< Raw ADC code from displacement sensor (±5V → 24-bit)
    int32_t current_adc;        ///< Raw ADC code from current sense (±5V → 24-bit)
    int32_t voltage_adc;        ///< Raw ADC code from voltage sense (±5V → 24-bit)
    uint64_t timestamp_ns;      ///< Sample timestamp [ns]
};

// DAC output frame (to hardware or simulator)
struct DacFrame {
    int32_t dac_code;           ///< DAC output code (±5V → 24-bit)
    uint64_t timestamp_ns;      ///< Output timestamp [ns]
};

// Internal processing frame (fixed-point Q31)
struct DspFrame {
    fixed::q31_t x_meas;        ///< Measured displacement [m] in Q31 (±X_LIM)
    fixed::q31_t x_ref;         ///< Reference displacement [m] in Q31 (±X_MAX)
    fixed::q31_t i_meas;        ///< Measured current [A] in Q31 (±10A)
    fixed::q31_t v_meas;        ///< Measured voltage [V] in Q31 (±50V)
    fixed::q31_t x_hat;         ///< Observer displacement estimate [m] in Q31
    fixed::q31_t v_hat;         ///< Observer velocity estimate [m/s] in Q31
    fixed::q31_t u_ff;          ///< Feedforward voltage [V] in Q31
    fixed::q31_t u_fb;          ///< Feedback voltage [V] in Q31
    fixed::q31_t u_lim;         ///< Limited voltage [V] in Q31
    fixed::q31_t e;             ///< Excursion error [m] in Q31
    uint32_t sample_count;      ///< Sample counter
    bool limiter_active;        ///< Limiter engaged flag
};

// Pipeline statistics
struct PipelineStats {
    uint32_t total_samples;
    uint32_t max_cycles;
    uint32_t min_cycles;
    uint32_t avg_cycles;
    uint32_t overruns;
    float cpu_load_pct;
};

// ============================================================================
// Global Pipeline State
// ============================================================================

extern DspFrame g_dsp_frame;
extern PipelineStats g_pipeline_stats;

// ============================================================================
// Public API
// ============================================================================

/**
 * @brief Initialize complete DSP pipeline.
 * 
 * Initializes all submodules: LUT, Observer, Volterra, MFB, Limiter, LSP.
 * Must be called before first dsp_step().
 * 
 * @return true on success
 */
bool dsp_pipeline_init() noexcept;

/**
 * @brief Reset pipeline state (between runs).
 */
void dsp_pipeline_reset() noexcept;

/**
 * @brief Single-sample DSP step - MAIN ISR ENTRY POINT.
 * 
 * Processes one sample through the complete pipeline:
 * 1. Convert ADC codes to physical units (Q31)
 * 2. Cubic-spline LUT: x_meas = S⁻¹(V_out)
 * 3. Luenberger observer: estimate x_hat, v_hat
 * 4. Volterra feedforward: u_ff = V(x_ref_history)
 * 5. MFB feedback: u_fb = -K*x_hat + K_r*x_ref
 * 6. Decoupling limiter: u_lim = sat(u_ff + u_fb, ±V_max(x_meas))
 * 7. Update LSP estimator (decimated)
 * 8. Push telemetry to ring buffer
 * 9. Convert to DAC code
 * 
 * @param adc Input ADC frame (from hardware DMA or simulator)
 * @param dac Output DAC frame (to hardware DMA or simulator)
 * 
 * Deterministic execution: no heap allocation, no mutexes, no exceptions.
 * Worst-case < 60 µs on Cortex-M7 @ 480 MHz.
 */
void dsp_step(const AdcFrame* adc, DacFrame* dac) noexcept;

/**
 * @brief Get current pipeline frame (for telemetry/monitoring).
 */
DSP_INLINE const DspFrame& dsp_pipeline_get_frame() noexcept {
    return g_dsp_frame;
}

/**
 * @brief Get pipeline performance statistics.
 */
DSP_INLINE PipelineStats dsp_pipeline_get_stats() noexcept {
    return g_pipeline_stats;
}

/**
 * @brief Set reference displacement (from host/command interface).
 * 
 * @param x_ref_mm Reference displacement [mm]
 */
DSP_INLINE void dsp_pipeline_set_reference(float x_ref_mm) noexcept {
    g_dsp_frame.x_ref = fixed::q31_from_float(x_ref_mm * 0.001f);  // mm → m
}

/**
 * @brief Get current reference displacement.
 */
DSP_INLINE float dsp_pipeline_get_reference_mm() noexcept {
    return fixed::q31_to_float(g_dsp_frame.x_ref) * 1000.0f;
}

/**
 * @brief Get reference displacement in X_MAX units (for MFB controller).
 * 
 * MFB controller expects reference in X_MAX units (1.0 Q31 = X_MAX = 12 mm).
 */
DSP_INLINE fixed::q31_t dsp_pipeline_get_reference_xmax() noexcept {
    // x_ref is in X_LIM units, convert to X_MAX: x_max = x_lim * (X_LIM / X_MAX)
    return fixed::q31_sat_mul(g_dsp_frame.x_ref, fixed::q31_from_float(X_LIM / X_MAX));
}

} // namespace dsp

// ============================================================================
// C-ABI Entry Points (for shared library loading by external co-simulators)
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the DSP pipeline.
 * 
 * Must be called before any other C-ABI function.
 * 
 * @return 0 on success, negative error code on failure
 */
DSP_API int dsp_init_cabi(void) noexcept;

/**
 * @brief Reset the DSP pipeline state.
 * 
 * Resets all internal state (filters, observers, limiters, statistics).
 * Does not re-initialize submodules - use dsp_init_cabi() for full re-initialization.
 */
DSP_API void dsp_reset_cabi(void) noexcept;

/**
 * @brief Set the displacement reference.
 * 
 * @param x_ref_mm Reference displacement in millimeters
 */
DSP_API void dsp_set_reference_cabi(float x_ref_mm) noexcept;

/**
 * @brief Get MFB controller gains for debugging.
 * 
 * @param k1_out Pointer to store k1 (position gain)
 * @param k2_out Pointer to store k2 (velocity gain)
 * @param k_r_out Pointer to store k_r (reference gain)
 * @return 0 on success, negative error code on failure
 */
DSP_API int dsp_get_mfb_gains_cabi(int32_t* k1_out, int32_t* k2_out, int32_t* k_r_out) noexcept;

/**
 * @brief Get MFB controller output for given inputs (debug).
 * 
 * @param x_ref_mm Reference displacement [mm]
 * @param x_hat_mm Estimated displacement [mm]
 * @param v_hat_mps Estimated velocity [m/s]
 * @param u_fb_out Pointer to store u_fb in normalized Q31 (±1.0 = ±150V)
 * @param fb_out Pointer to store fb component
 * @param ff_out Pointer to store ff component
 * @return 0 on success, negative error code on failure
 */
DSP_API int dsp_debug_mfb_step_cabi(float x_ref_mm, float x_hat_mm, float v_hat_mps, int32_t* u_fb_out, int32_t* fb_out, int32_t* ff_out) noexcept;

/**
 * @brief Debug full pipeline step (returns all internal signals).
 * 
 * @param x_ref_mm Reference displacement [mm]
 * @param adc_disp ADC code for displacement
 * @param adc_curr ADC code for current
 * @param adc_volt ADC code for voltage
 * @param x_meas_out Pointer to store x_meas [mm]
 * @param x_hat_out Pointer to store x_hat [mm]
 * @param v_hat_out Pointer to store v_hat [m/s]
 * @param u_ff_out Pointer to store u_ff [normalized Q31]
 * @param u_fb_out Pointer to store u_fb [normalized Q31]
 * @param u_lim_out Pointer to store u_lim [normalized Q31]
 * @param dac_code_out Pointer to store DAC code
 * @return 0 on success, negative error code on failure
 */
DSP_API int dsp_debug_pipeline_step_cabi(float x_ref_mm, int32_t adc_disp, int32_t adc_curr, int32_t adc_volt,
                                          float* x_meas_out, float* x_hat_out, float* v_hat_out,
                                          int32_t* u_ff_out, int32_t* u_fb_out, int32_t* u_lim_out,
                                          int32_t* dac_code_out) noexcept;

/**
 * @brief C-ABI DSP step function.
 * 
 * Exported symbol for dynamic loading by host simulators.
 * 
 * @param adc Pointer to AdcFrame input
 * @param dac Pointer to DacFrame output
 * @return 0 on success, negative error code on failure
 */
DSP_API int dsp_step_cabi(const void* adc, void* dac) noexcept;

#ifdef __cplusplus
}
#endif