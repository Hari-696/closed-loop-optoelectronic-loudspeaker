#pragma once

/**
 * @file mfb_controller.hpp
 * @brief State-feedback MFB controller with pole placement.
 * 
 * Control law in physical units:
 *   u_ff_linear = 9338.0 * x_ref        (V)   // inverse DC gain
 *   e = x_ref - x_meas                  (m)
 *   integ += e / 96000 (clamped)
 *   u_fb = Kp*e + Ki*integ - Kd*v_hat   (V)
 *   u_total = u_ff_linear + u_fb + u_ff_nl
 *   u_norm = clamp(u_total / AMP_FULL_SCALE_V, -1.0, 1.0)
 * Fixed-point q31 implementation.
 */

#include <cstdint>
#include "config.hpp"
#include "fixed_point.hpp"
#include "luenberger_observer.hpp"

namespace dsp {

// ============================================================================
// MFB Controller Configuration
// ============================================================================

// State-feedback gain K = [k1, k2] for u = -K * x_hat
// Reference gain K_r for tracking
// Physical gains stored for use in step function
struct MFBGainsQ31 {
    fixed::q31_t k1;           ///< Position feedback gain (Kp scaled)
    fixed::q31_t k2;           ///< Velocity feedback gain (Kd scaled)
    fixed::q31_t k_r;          ///< Reference tracking gain (Kr scaled)
    
    // Physical gains for step function
    float kp_phys = 60000.0f;   // V/m
    float ki_phys = 1.5e7f;     // V/(m*s)
    float kd_phys = 60.0f;      // V/(m/s)
    float kr_phys = 9338.0f;    // V/m (inverse DC gain)
};

extern MFBGainsQ31 g_mfb_gains;

// ============================================================================
// Public API
// ============================================================================

/**
 * @brief Initialize MFB controller with pole placement.
 * 
 * Places closed-loop poles at 200-220 Hz with ζ = 0.707.
 * Computes K via Ackermann's formula on discrete plant (A_d, B_d).
 * Computes K_r for unity DC gain: K_r = 1 / (C_d * (I - (A_d - B_d*K))⁻¹ * B_d)
 * 
 * @param cl_pole_hz Desired closed-loop pole frequency [Hz] (150-220)
 * @param damping Damping ratio ζ (default 0.707)
 * @return true on success
 */
DSP_API bool mfb_controller_init(float cl_pole_hz = 200.0f, float damping = CL_DAMPING_ZETA) noexcept;

/**
 * @brief Compute feedback control action.
 * 
 * u_fb = -K * x_hat + K_r * x_ref
 * 
 * @param x_ref_q31 Reference displacement [m] in Q31 (±X_MAX)
 * @param x_hat_q31 Estimated displacement [m] in Q31 (±X_LIM)
 * @param v_hat_q31 Estimated velocity [m/s] in Q31 (±10 m/s)
 * @return Control voltage [V] in Q31 (±AMP_FULL_SCALE_V)
 * 
 * Deterministic: 2 MACs + 1 MAC for reference, no branches.
 * Not inlined to avoid register pressure issues with Q31Accumulator in pipeline context.
 */
/**
 * @brief Compute feedback control action with physical units.
 * 
 * @param x_ref_q31 Reference displacement [m] in Q31 (±X_MAX)
 * @param x_hat_q31 Estimated displacement [m] in Q31 (±X_LIM)
 * @param v_hat_q31 Estimated velocity [m/s] in Q31 (±10 m/s)
 * @param x_meas_q31 Measured displacement [m] in Q31 (±X_LIM)
 * @param v_hat_q31_vel Estimated velocity [m/s] in Q31 (±10 m/s)
 * @param x_ref_q31_ref Reference displacement [m] in Q31 (±X_MAX)
 * @return Control voltage [V] in Q31 (±AMP_FULL_SCALE_V)
 * 
 * Deterministic: 5 MACs, no branches in hot path.
 * Uses physical units internally for clarity.
 */
/**
 * @brief Compute feedback control action with physical units.
 * 
 * @param x_ref_q31 Current reference displacement [m] in Q31 (±X_MAX)
 * @param x_hat_q31 Estimated displacement [m] in Q31 (±X_LIM) - unused
 * @param v_hat_q31 Estimated velocity [m/s] in Q31 (±10 m/s) - unused
 * @param x_meas_q31 Measured displacement [m] in Q31 (±X_LIM)
 * @param v_hat_vel_q31 Estimated velocity [m/s] in Q31 (±10 m/s)
 * @param x_ref_q31_ref Reference displacement [m] in Q31 (±X_MAX)
 * @return Control voltage [V] in Q31 (±AMP_FULL_SCALE_V)
 * 
 * Deterministic: 5 MACs, no branches in hot path.
 * Uses physical units internally for clarity.
 */
DSP_API fixed::q31_t mfb_controller_step(
    fixed::q31_t x_ref_q31,
    fixed::q31_t x_hat_q31,
    fixed::q31_t v_hat_q31,
    fixed::q31_t x_meas_q31,
    fixed::q31_t v_hat_vel_q31,
    fixed::q31_t x_ref_q31_ref) noexcept;

/**
 * @brief Get current controller gains (for diagnostics).
 */
DSP_INLINE MFBGainsQ31 mfb_controller_get_gains() noexcept {
    return g_mfb_gains;
}

/**
 * @brief Reset controller integral state.
 */
DSP_API void mfb_controller_reset() noexcept;

} // namespace dsp