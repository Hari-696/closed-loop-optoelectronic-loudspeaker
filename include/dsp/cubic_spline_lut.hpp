#pragma once

/**
 * @file cubic_spline_lut.hpp
 * @brief O(1) piecewise-polynomial cubic-spline LUT for inverse-square optical linearization.
 * 
 * Implements x_meas = S⁻¹(V_out) where S: x → V_out is the inverse-square sensor response
 * P ∝ 1/(d₀ − x)². The LUT stores cubic polynomial coefficients for each segment,
 * evaluated via Horner's method in fixed-point for deterministic O(1) execution.
 */

#include <cstdint>
#include <cstddef>
#include <array>
#include "config.hpp"
#include "fixed_point.hpp"

namespace dsp {

// ============================================================================
// Cubic Spline LUT Configuration
// ============================================================================

constexpr size_t SPLINE_NUM_SEGMENTS = 256;        ///< Number of LUT segments (8-bit index)
constexpr size_t SPLINE_COEFFS_PER_SEGMENT = 4;    ///< a0, a1, a2, a3 per segment
// Actual optical voltage range for operating range x ∈ [-12mm, 12mm]:
// V_out = -I_ph*R_f, I_ph = 2.7µA at d=18mm (x=0) to ~30µA at d=6mm (x=12mm)
// V_out ranges from -0.135V (x=0) to -1.22V (x=±12mm)
// Use range [-2.0V, 0.0V] to cover operating range with margin
constexpr float SPLINE_INPUT_MIN_V = -2.0f;        ///< Minimum input voltage [V]
constexpr float SPLINE_INPUT_MAX_V = 0.0f;         ///< Maximum input voltage [V]
// LUT output is normalized: 1.0 Q31 = X_LIM (15mm). Clamp values in normalized units.
constexpr float SPLINE_OUTPUT_MIN_X = -1.0f;       ///< Minimum output displacement [normalized to X_LIM]
constexpr float SPLINE_OUTPUT_MAX_X = 1.0f;        ///< Maximum output displacement [normalized to X_LIM]

// Derived constants
constexpr float SPLINE_INPUT_RANGE = SPLINE_INPUT_MAX_V - SPLINE_INPUT_MIN_V;
constexpr float SPLINE_SEGMENT_WIDTH = SPLINE_INPUT_RANGE / static_cast<float>(SPLINE_NUM_SEGMENTS);
constexpr float SPLINE_INV_SEGMENT_WIDTH = 1.0f / SPLINE_SEGMENT_WIDTH;

// ============================================================================
// Spline Coefficient Storage (Q31 fixed-point)
// ============================================================================

struct SplineSegmentQ31 {
    fixed::q31_t a0, a1, a2, a3;  ///< Coefficients for: a0 + a1*dx + a2*dx² + a3*dx³
};

struct SplineLUT {
    std::array<SplineSegmentQ31, SPLINE_NUM_SEGMENTS> segments;
    fixed::q31_t input_min_q31;
    fixed::q31_t input_max_q31;
    fixed::q31_t inv_segment_width_q31;
    bool initialized = false;
};

// Global LUT instance (zero-initialized in .bss)
extern SplineLUT g_spline_lut;

// ============================================================================
// Public API
// ============================================================================

/**
 * @brief Initialize the cubic spline LUT from calibration data.
 * 
 * @param voltage_samples Array of N+1 voltage values [V] (monotonic increasing)
 * @param displacement_samples Array of N+1 displacement values [m] corresponding to voltages
 * @param num_points Number of calibration points (must be ≥ 4)
 * @return true on success, false on invalid input
 * 
 * This computes natural cubic spline coefficients offline. The resulting
 * piecewise polynomials are stored in Q31 fixed-point for ISR evaluation.
 */
bool cubic_spline_lut_init(const float* voltage_samples, const float* displacement_samples, size_t num_points) noexcept;

/**
 * @brief Evaluate the inverse sensor mapping: x_meas = S⁻¹(V_out)
 * 
 * @param v_out Input voltage from ADC [V] (range -5V to +5V)
 * @return Estimated displacement [m] (range -X_LIM to +X_LIM)
 * 
 * Deterministic O(1) execution: ~8 multiply-adds, no branches in hot path.
 * Safe for real-time ISR use.
 */
DSP_INLINE float cubic_spline_lut_eval(float v_out) noexcept {
    // Clamp input to valid range
    if (v_out <= SPLINE_INPUT_MIN_V) return SPLINE_OUTPUT_MIN_X;
    if (v_out >= SPLINE_INPUT_MAX_V) return SPLINE_OUTPUT_MAX_X;
    
    // Compute segment index (0 to SPLINE_NUM_SEGMENTS-1)
    float normalized = (v_out - SPLINE_INPUT_MIN_V) * SPLINE_INV_SEGMENT_WIDTH;
    size_t idx = static_cast<size_t>(normalized);
    if (idx >= SPLINE_NUM_SEGMENTS) idx = SPLINE_NUM_SEGMENTS - 1;
    
    // Compute dx within segment [0, segment_width)
    float dx = v_out - (SPLINE_INPUT_MIN_V + static_cast<float>(idx) * SPLINE_SEGMENT_WIDTH);
    
    // Evaluate cubic polynomial using Horner's method
    const auto& seg = g_spline_lut.segments[idx];
    float result = seg.a3;
    result = result * dx + seg.a2;
    result = result * dx + seg.a1;
    result = result * dx + seg.a0;
    
    return result;
}

/**
 * @brief Fixed-point evaluation for ISR (Q31 input/output)
 * 
 * @param v_out_q31 Input voltage in Q31 (scaled to ±5V range)
 * @return Displacement in Q31 (scaled to ±X_LIM range)
 */
DSP_INLINE fixed::q31_t cubic_spline_lut_eval_q31(fixed::q31_t v_out_q31) noexcept {
    using namespace fixed;
    
    // Clamp
    if (v_out_q31 <= g_spline_lut.input_min_q31) return fixed::q31_from_float(SPLINE_OUTPUT_MIN_X);
    if (v_out_q31 >= g_spline_lut.input_max_q31) return fixed::q31_from_float(SPLINE_OUTPUT_MAX_X);
    
    // Compute segment index: idx = (v_out - min) * inv_width
    q31_t diff = q31_sat_sub(v_out_q31, g_spline_lut.input_min_q31);
    // Multiply by inv_segment_width (pre-scaled to Q31)
    // idx = (diff * inv_width) >> 31
    int64_t idx_raw = (static_cast<int64_t>(diff) * static_cast<int64_t>(g_spline_lut.inv_segment_width_q31)) >> 31;
    size_t idx = static_cast<size_t>(idx_raw);
    if (idx >= SPLINE_NUM_SEGMENTS) idx = SPLINE_NUM_SEGMENTS - 1;
    
    // Compute dx = v_out - (min + idx * segment_width)
    // segment_width_q31 = (max - min) / NUM_SEGMENTS
    constexpr q31_t segment_width_q31 = fixed::q31_from_float(SPLINE_SEGMENT_WIDTH);
    q31_t idx_q31 = fixed::q31_from_float(static_cast<float>(idx));
    q31_t segment_start = q31_sat_add(g_spline_lut.input_min_q31, q31_sat_mul(segment_width_q31, idx_q31));
    q31_t dx = q31_sat_sub(v_out_q31, segment_start);
    
    // Evaluate cubic: ((a3 * dx + a2) * dx + a1) * dx + a0
    const auto& seg = g_spline_lut.segments[idx];
    q31_t result = seg.a3;
    result = q31_sat_add(q31_sat_mul(result, dx), seg.a2);
    result = q31_sat_add(q31_sat_mul(result, dx), seg.a1);
    result = q31_sat_add(q31_sat_mul(result, dx), seg.a0);
    
    return result;
}

/**
 * @brief Check if LUT is initialized and ready for use.
 */
DSP_INLINE bool cubic_spline_lut_ready() noexcept {
    return g_spline_lut.initialized;
}

/**
 * @brief Get LUT metadata for diagnostics.
 */
struct SplineLUTInfo {
    size_t num_segments;
    float input_min_v, input_max_v;
    float output_min_x, output_max_x;
    bool initialized;
};

DSP_INLINE SplineLUTInfo cubic_spline_lut_get_info() noexcept {
    return {
        SPLINE_NUM_SEGMENTS,
        SPLINE_INPUT_MIN_V, SPLINE_INPUT_MAX_V,
        SPLINE_OUTPUT_MIN_X, SPLINE_OUTPUT_MAX_X,
        g_spline_lut.initialized
    };
}

/**
 * @brief Alternative initialization from inverse-square model directly.
 * 
 * Generates calibration points from the physical model:
 * P_rx ∝ 1/(d₀ - x)², I_ph = ℜ·P_rx, V_out = -I_ph·R_f
 * Inverts to get x(V_out) for the LUT.
 */
bool cubic_spline_lut_init_from_model(float d0, float r_f, float responsivity, float p_tx_optical) noexcept;

} // namespace dsp