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
    float x_abs_m = q31_to_float(q31_abs(x_meas_q31)) * X_LIM;
    constexpr float KNEE_M = 0.0127f;  // 12.7 mm: clean at 1.0 X_max
    if (x_abs_m <= KNEE_M) return q31_from_float(LIMITER_V_NOMINAL);
    constexpr float STOP_M = 0.0130f;
    if (x_abs_m >= STOP_M) return 0;
    float norm = (x_abs_m - KNEE_M) / (STOP_M - KNEE_M);
    float factor = 1.0f - norm * norm;
    if (factor < 0.0f) factor = 0.0f;
    return q31_from_float(LIMITER_V_NOMINAL * factor);
}

fixed::q31_t decoupling_limiter_apply_amp(fixed::q31_t u_ff_q31, fixed::q31_t u_fb_q31,
                                                       fixed::q31_t x_meas_q31) noexcept {
    using namespace fixed;
    auto& S = g_limiter_state;
    fixed::q31_t v_max = decoupling_limiter_compute_vmax(x_meas_q31);
    S.v_max_q31 = v_max;

    float x_m = q31_to_float(x_meas_q31) * X_LIM;
    constexpr float KNEE_M = 0.0127f;  // 12.7 mm: clean at 1.0 X_max
    S.limiter_active = (x_m > KNEE_M || x_m < -KNEE_M);
    S.hard_limit_active = (x_m >= X_LIM || x_m <= -X_LIM);
    S.rls_freeze = S.limiter_active;

    fixed::q31_t u_sum = q31_sat_add(u_ff_q31, u_fb_q31);
    fixed::q31_t u_limited_amp = u_sum;

    // Directional limiting: only cap outward drive, never block restoring force
    if (x_m > KNEE_M) {
        // Beyond KNEE_M: progressively limit outward drive using v_max
        if (u_sum > v_max) u_limited_amp = v_max;
        // Also apply active braking: immediately limit to full inward force at KNEE_M
        // This ensures we start braking as soon as we exceed 1.0 X_max
        fixed::q31_t brake = q31_from_float(-1.0f);  // Full inward brake immediately
        if (u_limited_amp > brake) u_limited_amp = brake;
    } else if (x_m < -KNEE_M) {
        if (u_sum < q31_neg(v_max)) u_limited_amp = q31_neg(v_max);
        fixed::q31_t brake = q31_from_float(1.0f);  // Full inward brake immediately
        if (u_limited_amp < brake) u_limited_amp = brake;
    }

    // Hard stop at X_LIM (15 mm): completely block outward drive
    if (x_m >= X_LIM && u_limited_amp > 0) u_limited_amp = 0;
    else if (x_m <= -X_LIM && u_limited_amp < 0) u_limited_amp = 0;

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