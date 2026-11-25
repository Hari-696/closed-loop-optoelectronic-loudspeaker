#pragma once

/**
 * @file volterra_filter.hpp
 * @brief Adaptive feedforward inverse Volterra filter (2nd + 3rd order, sparse).
 * 
 * u_ff[n] = Σ g_1[m]·x_ref[n-m] + Σ Σ g_2[m1,m2]·x_ref[n-m1]·x_ref[n-m2] + Σ Σ Σ g_3[m1,m2,m3]·x_ref[n-m1]·x_ref[n-m2]·x_ref[n-m3]
 * 
 * Memory depth M = 32 samples. Sparsification: |m_i - m_j| ≤ 4 for 3rd order (~1200 coeffs).
 * Online RLS update with λ = 0.998, executed every 4th sample (24 kHz).
 * Fixed-point q31 implementation.
 */

#include <cstdint>
#include <array>
#include "config.hpp"
#include "fixed_point.hpp"

namespace dsp {

// ============================================================================
// Volterra Filter Configuration (use constants from config.hpp)
// ============================================================================

// Use constants from config.hpp:
// VOLTERRA_MEMORY_M, VOLTERRA_SPARSITY_WINDOW, RLS_LAMBDA, RLS_UPDATE_DECIMATION

constexpr size_t VOLTERRA_1ST_COEFFS = VOLTERRA_MEMORY_M;
constexpr size_t VOLTERRA_2ND_COEFFS_MAX = VOLTERRA_MEMORY_M * (2 * VOLTERRA_SPARSITY_WINDOW + 1);  // 288
constexpr size_t VOLTERRA_3RD_COEFFS_MAX = 1200;  // As specified in report

// RLS configuration (use from config.hpp)
constexpr fixed::q31_t RLS_LAMBDA_Q31 = fixed::q31_from_float(RLS_LAMBDA);
constexpr fixed::q31_t RLS_ONE_MINUS_LAMBDA_Q31 = fixed::q31_from_float(1.0f - RLS_LAMBDA);

// ============================================================================
// Sparse Volterra Coefficient Storage
// ============================================================================

// 1st-order kernel: g1[m] for m = 0..M-1
struct Volterra1stOrder {
    std::array<fixed::q31_t, VOLTERRA_1ST_COEFFS> g1;
    fixed::q31_t lambda_inv[VOLTERRA_1ST_COEFFS];  // 1/λ for RLS (precomputed)
};

// 2nd-order kernel: g2[m1][m2] where |m1-m2| ≤ W, stored as packed triangular
// Index mapping: idx = m1*(2W+1) + (m2 - m1 + W) for m1 ≤ m2, symmetric
struct Volterra2ndOrder {
    std::array<fixed::q31_t, VOLTERRA_2ND_COEFFS_MAX> g2;
    std::array<fixed::q31_t, VOLTERRA_2ND_COEFFS_MAX> p_inv;  // Inverse correlation diagonal (RLS)
    uint16_t valid_mask[VOLTERRA_MEMORY_M];  // Bitmask of valid m2 for each m1
};

// 3rd-order kernel: g3[m1][m2][m3] where max(|mi-mj|) ≤ W
// Stored as flat array with index mapping for near-diagonal terms only
struct Volterra3rdOrder {
    std::array<fixed::q31_t, VOLTERRA_3RD_COEFFS_MAX> g3;
    std::array<fixed::q31_t, VOLTERRA_3RD_COEFFS_MAX> p_inv;  // RLS inverse correlation
    uint16_t num_active;  // Number of active coefficients
    
    // Index mapping tables (precomputed at init)
    struct TripleIndex { uint8_t m1, m2, m3; };
    TripleIndex index_map[VOLTERRA_3RD_COEFFS_MAX];
};

struct VolterraFilter {
    Volterra1stOrder order1;
    Volterra2ndOrder order2;
    Volterra3rdOrder order3;
    
    // Delay line for x_ref history (circular buffer)
    std::array<fixed::q31_t, VOLTERRA_MEMORY_M> delay_line;
    size_t delay_head = 0;
    
    // RLS state
    fixed::q31_t rls_lambda_q31;
    uint32_t sample_counter = 0;
    bool initialized = false;
    
    // Input scaling factors
    fixed::q31_t input_scale_q31;   // x_ref scaling to Q31
    fixed::q31_t output_scale_q31;  // u_ff scaling from Q31
};

extern VolterraFilter g_volterra_filter;

// ============================================================================
// Public API
// ============================================================================

/**
 * @brief Initialize Volterra filter coefficients and RLS state.
 * 
 * @param input_scale Scale factor for x_ref input (maps physical to Q31)
 * @param output_scale Scale factor for u_ff output (maps Q31 to physical)
 * @return true on success
 */
bool volterra_filter_init(float input_scale = 1.0f / X_MAX, float output_scale = 5.0f) noexcept;

/**
 * @brief Reset filter state (delay line, RLS counters).
 */
DSP_INLINE void volterra_filter_reset() noexcept {
    auto& F = g_volterra_filter;
    F.delay_line.fill(0);
    F.delay_head = 0;
    F.sample_counter = 0;
    // Don't reset coefficients - they adapt online
}

/**
 * @brief Single-sample feedforward evaluation (every sample at 96 kHz).
 * 
 * Computes u_ff[n] = Σ g1·x + Σ g2·x·x + Σ g3·x·x·x
 * Uses sparse indexing for 2nd/3rd order terms.
 * 
 * @param x_ref_q31 Reference displacement [m] in Q31 (scaled)
 * @return Feedforward voltage [V] in Q31 (scaled)
 * 
 * Deterministic: ~1760 MACs worst-case, no branches in hot path.
 */
fixed::q31_t volterra_filter_eval(fixed::q31_t x_ref_q31) noexcept;

/**
 * @brief RLS coefficient update (every 4th sample at 24 kHz).
 * 
 * Updates g1, g2, g3 using error e = x_meas - x_pred where
 * x_pred is the linear model prediction. Uses sparsified RLS.
 * 
 * @param x_ref_q31 Current reference input
 * @param x_meas_q31 Measured displacement
 * @param x_pred_q31 Linear model prediction (from MFB path)
 * @param u_fb_q31 Feedback control voltage
 */
void volterra_filter_rls_update(fixed::q31_t x_ref_q31, fixed::q31_t x_meas_q31, 
                                 fixed::q31_t x_pred_q31, fixed::q31_t u_fb_q31) noexcept;

/**
 * @brief Get filter diagnostics.
 */
struct VolterraDiag {
    uint32_t sample_count;
    uint32_t rls_update_count;
    fixed::q31_t g1_norm;    // L2 norm of 1st order kernel
    fixed::q31_t g2_norm;    // L2 norm of 2nd order kernel
    fixed::q31_t g3_norm;    // L2 norm of 3rd order kernel
    bool initialized;
};

DSP_INLINE VolterraDiag volterra_filter_get_diag() noexcept {
    auto& F = g_volterra_filter;
    VolterraDiag d;
    d.sample_count = F.sample_counter;
    d.rls_update_count = F.sample_counter / RLS_UPDATE_DECIMATION;
    d.initialized = F.initialized;
    
    // Compute norms (simplified - could be cached)
    fixed::Q31Accumulator acc1, acc2, acc3;
    for (size_t i = 0; i < VOLTERRA_1ST_COEFFS; ++i) acc1.mac(F.order1.g1[i], F.order1.g1[i]);
    for (size_t i = 0; i < VOLTERRA_2ND_COEFFS_MAX; ++i) acc2.mac(F.order2.g2[i], F.order2.g2[i]);
    for (size_t i = 0; i < F.order3.num_active; ++i) acc3.mac(F.order3.g3[i], F.order3.g3[i]);
    d.g1_norm = acc1.to_q31();
    d.g2_norm = acc2.to_q31();
    d.g3_norm = acc3.to_q31();
    return d;
}

} // namespace dsp