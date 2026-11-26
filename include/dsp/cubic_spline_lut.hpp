#pragma once

/**
 * @file cubic_spline_lut.hpp
 * @brief Exact inverse optical formula LUT for inverse-square optical linearization.
 * 
 * Uses the exact inverse optical formula instead of cubic spline interpolation:
 * V_out = -I_ph * R_f, I_ph = ℜ * P_rx, P_rx ∝ 1/(d₀ - x)²
 * V_out = V_rest * (d₀/(d₀ - x))²  where V_rest = -I_ph_rest * R_f
 * 
 * Inverse: x = d₀ * (1 - √(V_rest / V_out))  for V_out < 0
 * 
 * Normalized output: x / X_LIM (so ±1.0 Q31 = ±X_LIM)
 */

#include <cstdint>
#include <cstddef>
#include <array>
#include <cmath>
#include "config.hpp"
#include "fixed_point.hpp"

namespace dsp {

// ============================================================================
// LUT Configuration (Exact Inverse Optical Formula)
// ============================================================================

// LUT range: V_out from -5.0 V to -0.035 V (strictly increasing / less negative)
constexpr float LUT_V_MIN = -5.0f;
constexpr float LUT_V_MAX = -0.035f;

// LUT output normalization: 1.0 Q31 = X_LIM (15 mm)
constexpr float LUT_OUTPUT_MIN_X = -1.0f;
constexpr float LUT_OUTPUT_MAX_X = 1.0f;

// ============================================================================
// Precomputed Constants
// ============================================================================

constexpr float V_REST = -I_PHOTODIODE_REST * R_F;  // -0.13473 V for I_0 = 2.7 µA, R_f = 49.9 kΩ
constexpr float X_LIM_F = X_LIM;  // 0.015 m
constexpr float D_0_F = D_0;      // 0.018 m

// LUT output normalization: 1.0 Q31 = X_LIM (15 mm)
constexpr float X_NORM = 1.0f / X_LIM_F;

// ============================================================================
// LUT Instance (simplified - no spline coefficients needed)
// ============================================================================

struct SplineLUT {
    bool initialized = false;
};

extern SplineLUT g_spline_lut;

// ============================================================================
// Public API - Exact Formula Implementation (inline for ISR use)
// ============================================================================

/**
 * @brief Exact inverse optical formula: x = d₀ * (1 - √(V_rest / V_out))
 * 
 * @param v_out_q31 Input voltage in Q31 (scaled to ±5V range)
 * @return Normalized displacement in Q31 (where ±1.0 = ±X_LIM = ±15 mm)
 * 
 * Deterministic O(1) execution: exact formula, no branches in hot path.
 * Safe for real-time ISR use.
 */
DSP_INLINE fixed::q31_t cubic_spline_lut_eval_q31(fixed::q31_t v_out_q31) noexcept {
    using namespace fixed;
    
    // Clamp input to valid range
    if (v_out_q31 >= q31_from_float(LUT_V_MAX)) {
        return q31_from_float(-1.0f);  // x = -X_LIM
    }
    if (v_out_q31 <= q31_from_float(LUT_V_MIN)) {
        return q31_from_float(1.0f);   // x = +X_LIM
    }
    
    // Convert Q31 to float voltage
    float v_out = q31_to_float(v_out_q31);
    
    // Exact inverse formula: x = d₀ * (1 - √(V_rest / V_out))
    // V_rest and V_out are both negative, so ratio is positive
    float ratio = V_REST / v_out;  // positive
    float sqrt_ratio = std::sqrt(ratio);
    float x_m = D_0_F * (1.0f - sqrt_ratio);  // meters
    
    // Normalize to X_LIM: 1.0 = X_LIM
    float x_norm = x_m * X_NORM;
    
    // Clamp to [-1, 1]
    if (x_norm > 1.0f) x_norm = 1.0f;
    if (x_norm < -1.0f) x_norm = -1.0f;
    
    return q31_from_float(x_norm);
}

/**
 * @brief Float version for debug/host use
 */
DSP_INLINE float cubic_spline_lut_eval(float v_out) noexcept {
    if (v_out >= LUT_V_MAX) return -X_LIM_F;
    if (v_out <= LUT_V_MIN) return X_LIM_F;
    
    float ratio = V_REST / v_out;
    float sqrt_ratio = std::sqrt(ratio);
    float x_m = D_0_F * (1.0f - sqrt_ratio);
    
    if (x_m > X_LIM_F) x_m = X_LIM_F;
    if (x_m < -X_LIM_F) x_m = -X_LIM_F;
    
    return x_m;
}

/**
 * @brief Check if LUT is initialized and ready for use.
 */
DSP_INLINE bool cubic_spline_lut_ready() noexcept {
    return g_spline_lut.initialized;
}

/**
 * @brief Initialize LUT (no-op for exact formula implementation)
 */
bool cubic_spline_lut_init(const float*, const float*, size_t) noexcept;

/**
 * @brief Initialize LUT from exact inverse-square model.
 * 
 * @param d0 Optical gap standoff [m]
 * @param r_f TIA feedback resistor [Ω]
 * @param responsivity Photodiode responsivity [A/W]
 * @param p_tx_optical Optical power [W]
 * @return true on success
 */
bool cubic_spline_lut_init_from_model(float d0, float r_f, float responsivity, float p_tx_optical) noexcept;

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
        0,  // No segments (exact formula)
        LUT_V_MIN, LUT_V_MAX,
        -1.0f, 1.0f,
        g_spline_lut.initialized
    };
}

} // namespace dsp