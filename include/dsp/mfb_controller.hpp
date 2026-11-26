#pragma once

/**
 * @file mfb_controller.hpp
 * @brief State-feedback MFB controller with pole placement.
 * 
 * Control law: u_fb[k] = -K * x_hat[k] + K_r * x_ref[k]
 * Closed-loop poles: 110-120 Hz at ζ = 0.707 (above 108 Hz mechanical resonance)
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
struct MFBGainsQ31 {
    fixed::q31_t k1;       ///< Position feedback gain
    fixed::q31_t k2;       ///< Velocity feedback gain
    fixed::q31_t k_r;      ///< Reference tracking gain
};

extern MFBGainsQ31 g_mfb_gains;

// ============================================================================
// Public API
// ============================================================================

/**
 * @brief Initialize MFB controller with pole placement.
 * 
 * Places closed-loop poles at 110-120 Hz with ζ = 0.707.
 * Computes K via Ackermann's formula on discrete plant (A_d, B_d).
 * Computes K_r for unity DC gain: K_r = 1 / (C_d * (I - (A_d - B_d*K))⁻¹ * B_d)
 * 
 * @param cl_pole_hz Desired closed-loop pole frequency [Hz] (110-120)
 * @param damping Damping ratio ζ (default 0.707)
 * @return true on success
 */
bool mfb_controller_init(float cl_pole_hz = 115.0f, float damping = CL_DAMPING_ZETA) noexcept;

/**
 * @brief Compute feedback control action.
 * 
 * u_fb = -K * x_hat + K_r * x_ref
 * 
 * @param x_ref_q31 Reference displacement [m] in Q31 (±X_MAX)
 * @param x_hat_q31 Estimated displacement [m] in Q31 (±X_LIM)
 * @param v_hat_q31 Estimated velocity [m/s] in Q31 (±10 m/s)
 * @return Control voltage [V] in Q31 (±5V)
 * 
 * Deterministic: 2 MACs + 1 MAC for reference, no branches.
 * Not inlined to avoid register pressure issues with Q31Accumulator in pipeline context.
 */
fixed::q31_t mfb_controller_step(fixed::q31_t x_ref_q31, 
                                              fixed::q31_t x_hat_q31, 
                                              fixed::q31_t v_hat_q31) noexcept;

/**
 * @brief Get current controller gains (for diagnostics).
 */
DSP_INLINE MFBGainsQ31 mfb_controller_get_gains() noexcept {
    return g_mfb_gains;
}

/**
 * @brief Reset controller (no internal state to reset for static feedback).
 */
DSP_INLINE void mfb_controller_reset() noexcept {
    // Static state-feedback has no internal state
}

} // namespace dsp