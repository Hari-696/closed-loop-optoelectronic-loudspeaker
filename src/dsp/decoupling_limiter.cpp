/**
 * @file decoupling_limiter.cpp
 * @brief Decoupling limiter implementation.
 */

#include "dsp/decoupling_limiter.hpp"

namespace dsp {

// Global state instance
LimiterState g_limiter_state{};

// ============================================================================
// Implementation
// ============================================================================

bool decoupling_limiter_init() noexcept {
    g_limiter_state = LimiterState{};
    g_limiter_state.v_max_q31 = fixed::q31_from_float(LIMITER_V_NOMINAL);
    return true;
}

fixed::q31_t decoupling_limiter_compute_vmax(fixed::q31_t x_meas_q31) noexcept {
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

fixed::q31_t decoupling_limiter_apply_amp(fixed::q31_t u_ff_q31, fixed::q31_t u_fb_q31, 
                                                      fixed::q31_t x_meas_q31) noexcept {
    using namespace fixed;
    auto& S = g_limiter_state;
    
    // Compute voltage limit based on displacement (in AMP units)
    fixed::q31_t v_max = decoupling_limiter_compute_vmax(x_meas_q31);
    S.v_max_q31 = v_max;
    
    // Check limiter engagement
    fixed::q31_t x_abs = q31_abs(x_meas_q31);
    fixed::q31_t knee_q31 = q31_from_float(LIMITER_KNEE_X / X_LIM);
    fixed::q31_t hard_q31 = q31_from_float(1.0f);
    
    S.limiter_active = (x_abs > knee_q31);
    S.hard_limit_active = (x_abs >= hard_q31);
    S.rls_freeze = S.limiter_active;  // Freeze RLS when limiter engages
    
    // Sum feedforward and feedback (both in AMP units)
    fixed::q31_t u_sum = q31_sat_add(u_ff_q31, u_fb_q31);
    
    // Symmetric saturation: sat(u, ±v_max) in AMP units
    fixed::q31_t u_limited_amp;
    if (u_sum > v_max) {
        u_limited_amp = v_max;
    } else if (u_sum < q31_neg(v_max)) {
        u_limited_amp = q31_neg(v_max);
    } else {
        u_limited_amp = u_sum;
    }
    
    S.last_input_q31 = u_limited_amp;
    return u_limited_amp;
}

fixed::q31_t decoupling_limiter_apply(fixed::q31_t u_ff_q31, fixed::q31_t u_fb_q31, 
                                                  fixed::q31_t x_meas_q31) noexcept {
    fixed::q31_t u_limited_amp = decoupling_limiter_apply_amp(u_ff_q31, u_fb_q31, x_meas_q31);
    // Scale from AMP units (±150V) to ADC units (±5V): multiply by 5/150 = 1/30
    return fixed::q31_sat_mul(u_limited_amp, fixed::q31_from_float(AMP_TO_ADC_SCALE));
}

bool decoupling_limiter_rls_freeze() noexcept {
    return g_limiter_state.rls_freeze;
}

LimiterTelemetry decoupling_limiter_get_telemetry(fixed::q31_t x_meas_q31) noexcept {
    auto& S = g_limiter_state;
    return {
        S.limiter_active,
        S.hard_limit_active,
        fixed::q31_to_float(S.v_max_q31) * AMP_FULL_SCALE_V,  // Convert AMP Q31 to volts
        fixed::q31_to_float(x_meas_q31) * 1000.0f  // Convert to mm
    };
}

void decoupling_limiter_reset() noexcept {
    decoupling_limiter_init();
}

} // namespace dsp