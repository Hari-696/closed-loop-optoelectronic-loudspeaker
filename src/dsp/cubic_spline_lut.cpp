/**
 * @file cubic_spline_lut.cpp
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

#include "dsp/cubic_spline_lut.hpp"
#include "dsp/config.hpp"
#include "dsp/fixed_point.hpp"
#include <cmath>

namespace dsp {

// Global LUT instance
SplineLUT g_spline_lut{};

// ============================================================================
// Non-inline functions
// ============================================================================

bool cubic_spline_lut_init(const float*, const float*, size_t) noexcept {
    // No calibration data needed - we use exact formula
    g_spline_lut.initialized = true;
    return true;
}

bool cubic_spline_lut_init_from_model(float d0, float r_f, float responsivity, float p_tx_optical) noexcept {
    (void)d0; (void)r_f; (void)responsivity; (void)p_tx_optical;
    // No initialization needed - exact formula used directly
    g_spline_lut.initialized = true;
    return true;
}

} // namespace dsp