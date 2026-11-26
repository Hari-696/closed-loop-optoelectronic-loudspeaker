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
// Nominal full-scale in AMP units: ±1.0 Q31 = AMP_FULL_SCALE_V (150V)
constexpr float LIMITER_V_NOMINAL = 1.0f;                      
constexpr float LIMITER_EXPONENT = 2.0f;                       // Quadratic roll-off

// AMP to ADC scaling: 150V → 5V = 1/30
constexpr float AMP_TO_ADC_SCALE = VOLTAGE_FULL_SCALE_V / AMP_FULL_SCALE_V;  // 5/150 = 1/30

// Limiter state
struct LimiterState {
    fixed::q31_t v_max_q31;           // Current voltage limit [V] in Q31 (AMP units)
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
bool decoupling_limiter_init() noexcept;

/**
 * @brief Compute voltage limit V_max based on measured displacement.
 * 
 * V_max(x) = V_nominal                    for |x| ≤ X_knee
 * V_max(x) = V_nominal * (1 - ((|x|-X_knee)/(X_lim-X_knee))^k)  for X_knee < |x| < X_lim
 * V_max(x) = 0                            for |x| ≥ X_lim
 * 
 * @param x_meas_q31 Measured displacement [m] in Q31 (±X_LIM)
 * @return Voltage limit [V] in Q31 (AMP units: ±1.0 = ±150V)
 */
fixed::q31_t decoupling_limiter_compute_vmax(fixed::q31_t x_meas_q31) noexcept;

/**
 * @brief Apply decoupling and limiting to combined control signal (returns AMP units).
 * 
 * Decoupling logic:
 * - u_fb (MFB) sees only residual error: u_fb = -K*(x_hat - x_ref) 
 *   (already implemented in MFB controller as -K*x_hat + K_r*x_ref)
 * - u_ff (Volterra) provides feedforward pre-distortion
 * - Sum: u = u_ff + u_fb (in AMP units: ±1.0 Q31 = ±150V)
 * - Limit: u_lim = sat(u, ±V_max(x_meas)) in AMP units
 * - RLS freeze: when |x_meas| > X_knee, freeze Volterra adaptation
 * 
 * @param u_ff_q31 Feedforward voltage [V] in Q31 (AMP units: ±1.0 = ±150V)
 * @param u_fb_q31 Feedback voltage [V] in Q31 (AMP units: ±1.0 = ±150V)
 * @param x_meas_q31 Measured displacement [m] in Q31 (±X_LIM)
 * @return Limited voltage [V] in Q31 (AMP units: ±1.0 = ±150V)
 * 
 * Deterministic: ~10 operations, no branches in saturation path.
 */
fixed::q31_t decoupling_limiter_apply_amp(fixed::q31_t u_ff_q31, fixed::q31_t u_fb_q31, 
                                                      fixed::q31_t x_meas_q31) noexcept;

/**
 * @brief Apply decoupling and limiting, returning ADC units for DAC.
 * 
 * Same as apply_amp but scales output to ADC units (±1.0 Q31 = ±5V).
 * 
 * @return Limited voltage [V] in Q31 (ADC units: ±1.0 = ±5V)
 */
fixed::q31_t decoupling_limiter_apply(fixed::q31_t u_ff_q31, fixed::q31_t u_fb_q31, 
                                                  fixed::q31_t x_meas_q31) noexcept;

/**
 * @brief Check if Volterra RLS should be frozen (limiter engaged).
 */
bool decoupling_limiter_rls_freeze() noexcept;

/**
 * @brief Get limiter status for telemetry.
 */
struct LimiterTelemetry {
    bool active;
    bool hard_limit;
    float v_max_v;      // In volts (AMP scale)
    float x_meas_mm;
};

LimiterTelemetry decoupling_limiter_get_telemetry(fixed::q31_t x_meas_q31) noexcept;

/**
 * @brief Reset limiter state.
 */
void decoupling_limiter_reset() noexcept;

} // namespace dsp