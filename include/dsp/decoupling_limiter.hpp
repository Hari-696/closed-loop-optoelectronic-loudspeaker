#pragma once

/**
 * @file decoupling_limiter.hpp
 * @brief Cooperative decoupling network and over-excursion protection limiter.
 * 
 * Decoupling: MFB sees only residual error after feedforward; Volterra RLS freezes when limiter engages.
 * Limiter: u_lim[n] = sat(u[n], ±V_max(x_meas)) preventing bottoming at X_lim = 15 mm.
 * V_max decreases as |x_meas| approaches X_max, providing soft then hard limiting.
 */

#include <cstdint>
#include "config.hpp"
#include "fixed_point.hpp"

namespace dsp {

// ============================================================================
// Limiter Configuration
// ============================================================================

// Voltage limit profile: V_max(x) = V_nominal * (1 - (|x|/X_lim)^k) for |x| > X_knee
// Soft knee at LIMITER_KNEE_RATIO * X_MAX, hard limit at X_LIM
constexpr float LIMITER_KNEE_X = X_MAX * LIMITER_KNEE_RATIO;  // 10.8 mm
constexpr float LIMITER_HARD_X = X_LIM;                        // 15 mm
constexpr float LIMITER_V_NOMINAL = 5.0f;                      // Nominal DAC range [V]
constexpr float LIMITER_EXPONENT = 2.0f;                       // Quadratic roll-off

// Limiter state
struct LimiterState {
    fixed::q31_t v_max_q31;           // Current voltage limit [V] in Q31
    bool limiter_active = false;      // True when |x| > knee
    bool hard_limit_active = false;   // True when |x| >= X_LIM
    fixed::q31_t last_input_q31;      // For rate limiting detection
    bool rls_freeze = false;          // Signal to freeze Volterra RLS
};

extern LimiterState g_limiter_state;

// ============================================================================
// Public API
// ============================================================================

/**
 * @brief Initialize decoupling limiter.
 * @return true on success
 */
DSP_INLINE bool decoupling_limiter_init() noexcept {
    g_limiter_state = LimiterState{};
    g_limiter_state.v_max_q31 = fixed::q31_from_float(LIMITER_V_NOMINAL);
    return true;
}

/**
 * @brief Compute voltage limit V_max based on measured displacement.
 * 
 * V_max(x) = V_nominal                    for |x| ≤ X_knee
 * V_max(x) = V_nominal * (1 - ((|x|-X_knee)/(X_lim-X_knee))^k)  for X_knee < |x| < X_lim
 * V_max(x) = 0                            for |x| ≥ X_lim
 * 
 * @param x_meas_q31 Measured displacement [m] in Q31 (±X_LIM)
 * @return Voltage limit [V] in Q31 (±5V)
 */
DSP_INLINE fixed::q31_t decoupling_limiter_compute_vmax(fixed::q31_t x_meas_q31) noexcept {
    using namespace fixed;
    
    // Absolute displacement
    fixed::q31_t x_abs = q31_abs(x_meas_q31);
    fixed::q31_t knee_q31 = q31_from_float(LIMITER_KNEE_X / X_LIM);  // Normalized to X_LIM
    fixed::q31_t hard_q31 = fixed::q31_from_float(1.0f);  // X_LIM normalized = 1.0
    
    if (x_abs <= knee_q31) {
        // Below knee - full voltage available
        return q31_from_float(LIMITER_V_NOMINAL);
    }
    
    if (x_abs >= hard_q31) {
        // At or beyond hard limit - zero voltage
        return 0;
    }
    
    // In knee region - quadratic roll-off
    // normalized position in knee region: (|x| - knee) / (hard - knee)
    fixed::q31_t pos = q31_sat_sub(x_abs, knee_q31);
    
    // pos / range in Q31
    fixed::q31_t norm = q31_sat_mul(pos, q31_from_float(1.0f / (1.0f - LIMITER_KNEE_RATIO)));
    
    // norm^2
    fixed::q31_t norm2 = q31_sat_mul(norm, norm);
    
    // V_max = V_nominal * (1 - norm^2)
    fixed::q31_t one = q31_from_float(1.0f);
    fixed::q31_t factor = q31_sat_sub(one, norm2);
    fixed::q31_t vmax = q31_sat_mul(q31_from_float(LIMITER_V_NOMINAL), factor);
    
    return vmax;
}

/**
 * @brief Apply decoupling and limiting to combined control signal.
 * 
 * Decoupling logic:
 * - u_fb (MFB) sees only residual error: u_fb = -K*(x_hat - x_ref) 
 *   (already implemented in MFB controller as -K*x_hat + K_r*x_ref)
 * - u_ff (Volterra) provides feedforward pre-distortion
 * - Sum: u = u_ff + u_fb
 * - Limit: u_lim = sat(u, ±V_max(x_meas))
 * - RLS freeze: when |x_meas| > X_knee, freeze Volterra adaptation
 * 
 * @param u_ff_q31 Feedforward voltage [V] in Q31
 * @param u_fb_q31 Feedback voltage [V] in Q31
 * @param x_meas_q31 Measured displacement [m] in Q31 (±X_LIM)
 * @return Limited voltage [V] in Q31 (±5V)
 * 
 * Deterministic: ~10 operations, no branches in saturation path.
 */
DSP_INLINE fixed::q31_t decoupling_limiter_apply(fixed::q31_t u_ff_q31, fixed::q31_t u_fb_q31, 
                                                  fixed::q31_t x_meas_q31) noexcept {
    using namespace fixed;
    auto& S = g_limiter_state;
    
    // Compute voltage limit based on displacement
    fixed::q31_t v_max = decoupling_limiter_compute_vmax(x_meas_q31);
    S.v_max_q31 = v_max;
    
    // Check limiter engagement
    fixed::q31_t x_abs = q31_abs(x_meas_q31);
    fixed::q31_t knee_q31 = q31_from_float(LIMITER_KNEE_X / X_LIM);
    fixed::q31_t hard_q31 = q31_from_float(1.0f);
    
    S.limiter_active = (x_abs > knee_q31);
    S.hard_limit_active = (x_abs >= hard_q31);
    S.rls_freeze = S.limiter_active;  // Freeze RLS when limiter engages
    
    // Sum feedforward and feedback
    fixed::q31_t u_sum = q31_sat_add(u_ff_q31, u_fb_q31);
    
    // Symmetric saturation: sat(u, ±v_max)
    fixed::q31_t u_limited;
    if (u_sum > v_max) {
        u_limited = v_max;
    } else if (u_sum < q31_neg(v_max)) {
        u_limited = q31_neg(v_max);
    } else {
        u_limited = u_sum;
    }
    
    S.last_input_q31 = u_limited;
    return u_limited;
}

/**
 * @brief Check if Volterra RLS should be frozen (limiter engaged).
 */
DSP_INLINE bool decoupling_limiter_rls_freeze() noexcept {
    return g_limiter_state.rls_freeze;
}

/**
 * @brief Get limiter status for telemetry.
 */
struct LimiterTelemetry {
    bool active;
    bool hard_limit;
    float v_max_v;
    float x_meas_mm;
};

DSP_INLINE LimiterTelemetry decoupling_limiter_get_telemetry(fixed::q31_t x_meas_q31) noexcept {
    auto& S = g_limiter_state;
    return {
        S.limiter_active,
        S.hard_limit_active,
        fixed::q31_to_float(S.v_max_q31),
        fixed::q31_to_float(x_meas_q31) * 1000.0f  // Convert to mm
    };
}

/**
 * @brief Reset limiter state.
 */
DSP_INLINE void decoupling_limiter_reset() noexcept {
    decoupling_limiter_init();
}

} // namespace dsp