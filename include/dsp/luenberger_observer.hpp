#pragma once

/**
 * @file luenberger_observer.hpp
 * @brief Discrete ZOH Luenberger observer for velocity estimation from displacement.
 * 
 * State: x_hat[k] = [x_hat[k], v_hat[k]]^T
 * Observer poles: 325-540 Hz (3-5× mechanical resonance at 108 Hz)
 * Fixed-point q31 implementation for deterministic execution.
 */

#include <cstdint>
#include <array>
#include "config.hpp"
#include "fixed_point.hpp"

namespace dsp {

// ============================================================================
// Observer Configuration
// ============================================================================

// Continuous-time plant matrices (from Section 3.2)
// x_dot = A_c * x + B_c * u
// y = C_c * x
// State: [x, v]^T
// A_c = [0, 1; -K_ms0/M_ms, -R_ms/M_ms]
// B_c = [0; Bl_0/M_ms]
// C_c = [1, 0]

constexpr float A_C_00 = 0.0f;
constexpr float A_C_01 = 1.0f;
constexpr float A_C_10 = -K_MS0 / M_MS;
constexpr float A_C_11 = -R_MS / M_MS;
constexpr float B_C_0 = 0.0f;
constexpr float B_C_1 = BL_0 / M_MS;
constexpr float C_C_0 = 1.0f;
constexpr float C_C_1 = 0.0f;

// ============================================================================
// Observer State Structure (Q31)
// ============================================================================

struct ObserverStateQ31 {
    fixed::q31_t x_hat;      ///< Estimated displacement [m] in Q31 (±X_LIM)
    fixed::q31_t v_hat;      ///< Estimated velocity [m/s] in Q31 (±10 m/s scaled)
    fixed::q31_t x_meas_prev; ///< Previous measurement for numerical differentiation fallback
    bool initialized = false;
};

struct ObserverMatricesQ31 {
    // Discrete A_d, B_d (2x2 and 2x1)
    fixed::q31_t a00, a01, a10, a11;
    fixed::q31_t b0, b1;
    // Observer gain L (2x1)
    fixed::q31_t l0, l1;
    // C_d = [1, 0]
    // Precomputed (A_d - L*C_d) for prediction+correction combined
    fixed::q31_t a_l00, a_l01, a_l10, a_l11;
};

extern ObserverMatricesQ31 g_observer_matrices;
extern ObserverStateQ31 g_observer_state;

// ============================================================================
// Public API
// ============================================================================

/**
 * @brief Initialize observer with discrete matrices from ZOH discretization.
 * 
 * Computes A_d, B_d from continuous matrices at F_S = 96 kHz.
 * Places observer poles at 325-540 Hz via Ackermann's formula.
 * All matrices precomputed and stored in Q31.
 * 
 * @param observer_pole_hz Desired observer pole frequency [Hz] (325-540)
 * @return true on success
 */
bool luenberger_observer_init(float observer_pole_hz = 432.0f) noexcept;  // Default: 4× f_s_mech

/**
 * @brief Reset observer state to zero.
 */
DSP_INLINE void luenberger_observer_reset() noexcept {
    g_observer_state = ObserverStateQ31{};
}

/**
 * @brief Single-step observer update (prediction + correction).
 * 
 * Implements: x_hat[k+1] = A_d*x_hat[k] + B_d*u[k] + L*(y[k] - C_d*x_hat[k])
 * Combined as: x_hat[k+1] = (A_d - L*C_d)*x_hat[k] + B_d*u[k] + L*y[k]
 * 
 * @param u_q31 Drive voltage input [V] in Q31 (±5V)
 * @param y_q31 Measured displacement [m] in Q31 (±X_LIM)
 * @return Updated state (x_hat, v_hat)
 * 
 * Deterministic: ~12 MACs, no branches, no loops.
 * Safe for real-time ISR use.
 */
DSP_INLINE ObserverStateQ31 luenberger_observer_step(fixed::q31_t u_q31, fixed::q31_t y_q31) noexcept {
    using namespace fixed;
    const auto& M = g_observer_matrices;
    auto& S = g_observer_state;
    
    if (!S.initialized) {
        // First sample: initialize from measurement
        S.x_hat = y_q31;
        S.v_hat = 0;
        S.x_meas_prev = y_q31;
        S.initialized = true;
        return S;
    }
    
    // Prediction + correction combined:
    // x_hat_new = (A_d - L*C_d) * x_hat + B_d * u + L * y
    // where C_d = [1, 0], so L*C_d = [l0, 0; l1, 0]
    // (A_d - L*C_d) = [a00-l0, a01; a10-l1, a11]
    // These are precomputed as a_l00, a_l01, a_l10, a_l11
    
    Q31Accumulator acc_x, acc_v;
    
    // (A_d - L*C_d) * x_hat
    acc_x.mac(M.a_l00, S.x_hat);
    acc_x.mac(M.a_l01, S.v_hat);
    acc_v.mac(M.a_l10, S.x_hat);
    acc_v.mac(M.a_l11, S.v_hat);
    
    // B_d * u
    acc_x.mac(M.b0, u_q31);
    acc_v.mac(M.b1, u_q31);
    
    // L * y
    acc_x.mac(M.l0, y_q31);
    acc_v.mac(M.l1, y_q31);
    
    S.x_hat = acc_x.to_q31();
    S.v_hat = acc_v.to_q31();
    S.x_meas_prev = y_q31;
    
    return S;
}

/**
 * @brief Get current estimated state (displacement, velocity).
 */
DSP_INLINE ObserverStateQ31 luenberger_observer_get_state() noexcept {
    return g_observer_state;
}

/**
 * @brief Get current estimated velocity in float [m/s].
 */
DSP_INLINE float luenberger_observer_get_velocity_f32() noexcept {
    return fixed::q31_to_float(g_observer_state.v_hat);
}

/**
 * @brief Get current estimated displacement in float [m].
 */
DSP_INLINE float luenberger_observer_get_displacement_f32() noexcept {
    return fixed::q31_to_float(g_observer_state.x_hat);
}

/**
 * @brief Check if observer is initialized.
 */
DSP_INLINE bool luenberger_observer_ready() noexcept {
    return g_observer_state.initialized;
}

} // namespace dsp