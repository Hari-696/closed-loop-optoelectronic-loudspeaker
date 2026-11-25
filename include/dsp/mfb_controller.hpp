#pragma once

/**
 * @file mfb_controller.hpp
 * @brief State-feedback MFB controller with pole placement.
 * 
 * Control law: u_fb[k] = -K * x_hat[k] + K_r * x_ref[k]
 * Closed-loop poles: 60-90 Hz at ζ = 0.707
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
 * Places closed-loop poles at 60-90 Hz with ζ = 0.707.
 * Computes K via Ackermann's formula on discrete plant (A_d, B_d).
 * Computes K_r for unity DC gain: K_r = 1 / (C_d * (I - (A_d - B_d*K))⁻¹ * B_d)
 * 
 * @param cl_pole_hz Desired closed-loop pole frequency [Hz] (60-90)
 * @param damping Damping ratio ζ (default 0.707)
 * @return true on success
 */
bool mfb_controller_init(float cl_pole_hz = 75.0f, float damping = CL_DAMPING_ZETA) noexcept;

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
 */
DSP_INLINE fixed::q31_t mfb_controller_step(fixed::q31_t x_ref_q31, 
                                             fixed::q31_t x_hat_q31, 
                                             fixed::q31_t v_hat_q31) noexcept {
    using namespace fixed;
    const auto& G = g_mfb_gains;
    
    Q31Accumulator acc;
    
    // -K * x_hat = -(k1*x_hat + k2*v_hat)
    acc.mac(G.k1, x_hat_q31);
    acc.mac(G.k2, v_hat_q31);
    fixed::q31_t fb = q31_neg(acc.to_q31());
    
    // K_r * x_ref
    acc.reset();
    acc.mac(G.k_r, x_ref_q31);
    fixed::q31_t ff = acc.to_q31();
    
    // u_fb = fb + ff
    return q31_sat_add(fb, ff);
}

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