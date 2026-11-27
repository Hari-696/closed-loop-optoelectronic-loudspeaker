/**
 * @file volterra_filter.cpp
 * @brief Adaptive inverse Volterra filter implementation with sparse RLS.
 * 
 * Implements:
 * - 1st order: g1[m] for m=0..31 (32 coeffs)
 * - 2nd order: g2[m1,m2] for |m1-m2|≤4 (symmetric, ~144 unique)
 * - 3rd order: g3[m1,m2,m3] for max|mi-mj|≤4 (~1200 coeffs)
 * - RLS update every 4 samples with λ=0.998
 */

#include "dsp/volterra_filter.hpp"
#include <cmath>
#include <algorithm>

namespace dsp {

VolterraFilter g_volterra_filter{};

// ============================================================================
// Forward Declarations
// ============================================================================

static void generate_3rd_order_index_map(Volterra3rdOrder& order3) noexcept;

// ============================================================================
// Index Mapping for Sparse Volterra Kernels
// ============================================================================

// 2nd-order index: map (m1,m2) with m1≤m2 and m2-m1≤W to flat index
DSP_INLINE size_t volterra_2nd_index(uint8_t m1, uint8_t m2) noexcept {
    constexpr int W = VOLTERRA_SPARSITY_WINDOW;
    if (m1 > m2) std::swap(m1, m2);
    if (m2 - m1 > W) return SIZE_MAX;
    return static_cast<size_t>(m1) * (2 * W + 1) + static_cast<size_t>(m2 - m1 + W);
}

// 3rd-order: generate all valid triples (m1,m2,m3) with max|mi-mj|≤W
// Precomputed at init into index_map

static void generate_3rd_order_index_map(Volterra3rdOrder& order3) noexcept {
    constexpr int W = VOLTERRA_SPARSITY_WINDOW;
    constexpr size_t M = VOLTERRA_MEMORY_M;
    
    size_t idx = 0;
    for (uint8_t m1 = 0; m1 < M; ++m1) {
        for (uint8_t m2 = 0; m2 < M; ++m2) {
            if (std::abs(int(m2) - int(m1)) > W) continue;
            for (uint8_t m3 = 0; m3 < M; ++m3) {
                if (std::abs(int(m3) - int(m1)) > W) continue;
                if (std::abs(int(m3) - int(m2)) > W) continue;
                if (idx < VOLTERRA_3RD_COEFFS_MAX) {
                    order3.index_map[idx] = {m1, m2, m3};
                    ++idx;
                }
            }
        }
    }
    order3.num_active = idx;
}

// ============================================================================
// Initialization
// ============================================================================

bool volterra_filter_init(float input_scale, float output_scale) noexcept {
    using namespace fixed;
    auto& F = g_volterra_filter;
    
    // Clear all coefficients
    F.order1.g1.fill(0);
    F.order2.g2.fill(0);
    F.order3.g3.fill(0);
    
    // Initialize RLS inverse correlation (P = δ⁻¹I, δ small)
    constexpr fixed::q31_t delta_inv = fixed::q31_from_float(100.0f);  // 1/δ
    
    for (size_t i = 0; i < VOLTERRA_1ST_COEFFS; ++i) {
        F.order1.lambda_inv[i] = fixed::q31_from_float(1.0f / RLS_LAMBDA);
    }
    for (size_t i = 0; i < VOLTERRA_2ND_COEFFS_MAX; ++i) {
        F.order2.p_inv[i] = delta_inv;
    }
    for (size_t i = 0; i < VOLTERRA_3RD_COEFFS_MAX; ++i) {
        F.order3.p_inv[i] = delta_inv;
    }
    
    // Generate 2nd-order valid masks
    constexpr int W = VOLTERRA_SPARSITY_WINDOW;
    for (size_t m1 = 0; m1 < VOLTERRA_MEMORY_M; ++m1) {
        uint16_t mask = 0;
        for (int dm = -W; dm <= W; ++dm) {
            int m2 = static_cast<int>(m1) + dm;
            if (m2 >= 0 && m2 < static_cast<int>(VOLTERRA_MEMORY_M)) {
                mask |= (1u << (dm + W));
            }
        }
        F.order2.valid_mask[m1] = mask;
    }
    
    // Generate 3rd-order index map
    generate_3rd_order_index_map(F.order3);
    
    // Initialize delay line
    F.delay_line.fill(0);
    F.delay_head = 0;
    F.sample_counter = 0;
    
    // Scaling factors
    F.input_scale_q31 = fixed::q31_from_float(input_scale);
    F.output_scale_q31 = fixed::q31_from_float(output_scale);
    F.rls_lambda_q31 = RLS_LAMBDA_Q31;
    
    F.initialized = true;
    return true;
}

// ============================================================================
// Feedforward Evaluation (every sample)
// ============================================================================

fixed::q31_t volterra_filter_eval(fixed::q31_t x_ref_q31) noexcept {
    using namespace fixed;
    auto& F = g_volterra_filter;
    
    if (!F.initialized) return 0;
    
    // Update delay line (circular buffer)
    F.delay_line[F.delay_head] = x_ref_q31;
    F.delay_head = (F.delay_head + 1) % VOLTERRA_MEMORY_M;
    
    Q31Accumulator acc;
    
    // ---- 1st Order: Σ g1[m] * x[n-m] ----
    // Most recent sample (m=0) is at (delay_head - 1), m samples ago at (delay_head - 1 - m)
    for (size_t m = 0; m < VOLTERRA_1ST_COEFFS; ++m) {
        size_t idx = (F.delay_head + VOLTERRA_MEMORY_M - 1 - m) % VOLTERRA_MEMORY_M;
        acc.mac(F.order1.g1[m], F.delay_line[idx]);
    }
    
    // ---- 2nd Order: Σ Σ g2[m1,m2] * x[n-m1] * x[n-m2] ----
    // Only compute for m1 ≤ m2, |m1-m2| ≤ W, exploit symmetry
    constexpr int W = VOLTERRA_SPARSITY_WINDOW;
    for (size_t m1 = 0; m1 < VOLTERRA_MEMORY_M; ++m1) {
        size_t idx1 = (F.delay_head + VOLTERRA_MEMORY_M - 1 - m1) % VOLTERRA_MEMORY_M;
        fixed::q31_t x1 = F.delay_line[idx1];
        
        uint16_t mask = F.order2.valid_mask[m1];
        for (int dm = -W; dm <= W; ++dm) {
            if (!(mask & (1u << (dm + W)))) continue;
            int m2 = static_cast<int>(m1) + dm;
            if (m2 < 0 || m2 >= static_cast<int>(VOLTERRA_MEMORY_M)) continue;
            
            size_t idx2 = (F.delay_head + VOLTERRA_MEMORY_M - 1 - m2) % VOLTERRA_MEMORY_M;
            fixed::q31_t x2 = F.delay_line[idx2];
            
            size_t coeff_idx = volterra_2nd_index(static_cast<uint8_t>(m1), static_cast<uint8_t>(m2));
            if (coeff_idx == SIZE_MAX) continue;
            
            fixed::q31_t g2 = F.order2.g2[coeff_idx];
            
            // g2 * x1 * x2 (Q31 * Q31 * Q31 = Q31 with double shift)
            // Use 64-bit intermediate
            int64_t prod = static_cast<int64_t>(g2) * static_cast<int64_t>(x1);
            prod = (prod * static_cast<int64_t>(x2)) >> 31;  // Q31 * Q31 * Q31 → Q31
            prod = (prod + (1LL << 30)) >> 31;  // Extra shift for Q31*Q31
            
            fixed::q31_t term = static_cast<fixed::q31_t>(
                prod > 2147483647LL ? 2147483647LL : (prod < -2147483648LL ? -2147483648LL : prod)
            );
            
            // Account for symmetry: if m1 != m2, term appears twice
            if (m1 != static_cast<size_t>(m2)) {
                term = q31_sat_add(term, term);  // 2 * term
            }
            
            // Accumulate directly to avoid MAC overhead for this pattern
            acc.mac(term, fixed::q31_from_float(1.0f));  // Just add term
        }
    }
    
    // ---- 3rd Order: Σ Σ Σ g3[m1,m2,m3] * x[n-m1] * x[n-m2] * x[n-m3] ----
    for (size_t i = 0; i < F.order3.num_active; ++i) {
        auto idxs = F.order3.index_map[i];
        size_t idx1 = (F.delay_head + VOLTERRA_MEMORY_M - 1 - idxs.m1) % VOLTERRA_MEMORY_M;
        size_t idx2 = (F.delay_head + VOLTERRA_MEMORY_M - 1 - idxs.m2) % VOLTERRA_MEMORY_M;
        size_t idx3 = (F.delay_head + VOLTERRA_MEMORY_M - 1 - idxs.m3) % VOLTERRA_MEMORY_M;
        
        fixed::q31_t x1 = F.delay_line[idx1];
        fixed::q31_t x2 = F.delay_line[idx2];
        fixed::q31_t x3 = F.delay_line[idx3];
        fixed::q31_t g3 = F.order3.g3[i];
        
        // g3 * x1 * x2 * x3 (triple product in Q31)
        int64_t prod = static_cast<int64_t>(g3) * static_cast<int64_t>(x1);
        prod = (prod * static_cast<int64_t>(x2)) >> 31;
        prod = (prod * static_cast<int64_t>(x3)) >> 31;
        prod = (prod + (1LL << 30)) >> 31;  // Final Q31 rounding
        
        fixed::q31_t term = static_cast<fixed::q31_t>(
            prod > 2147483647LL ? 2147483647LL : (prod < -2147483648LL ? -2147483648LL : prod)
        );
        
        acc.mac(term, fixed::q31_from_float(1.0f));
    }
    
    fixed::q31_t result = acc.to_q31();
    
    // Apply output scaling
    result = q31_sat_mul(result, F.output_scale_q31);
    
    return result;
}

// ============================================================================
// RLS Update (every 4th sample)
// ============================================================================

// RLS update for a single coefficient: 
// k = P * φ / (λ + φ^T * P * φ)
// θ_new = θ_old + k * e
// P_new = (I - k * φ^T) * P / λ
// 
// For diagonal P approximation (scalar per coefficient):
// k = p * φ / (λ + p * φ²)
// p_new = (p - k * φ * p) / λ = p * (1 - k*φ) / λ

void volterra_filter_rls_update(fixed::q31_t /*x_ref_q31*/, fixed::q31_t x_meas_q31,
                                 fixed::q31_t x_pred_q31, fixed::q31_t /*u_fb_q31*/) noexcept {
    using namespace fixed;
    auto& F = g_volterra_filter;
    
    if (!F.initialized) return;
    
    F.sample_counter++;
    if (F.sample_counter % RLS_UPDATE_DECIMATION != 0) return;
    
    // Error signal: e = x_meas - (x_pred + volterra_contribution)
    // But we want to update Volterra to cancel the non-linear distortion
    // The error for Volterra adaptation is the residual after linear MFB
    // e_rls = x_meas - x_pred  (what the linear model couldn't predict)
    // We want Volterra to produce u_ff that cancels this
    
    // Error for RLS: we want Volterra to cancel the nonlinear distortion
    // x_pred is what the linear model predicts, x_meas is actual
    // If x_meas > x_pred, plant moved MORE than predicted → need LESS feedforward
    // So error for adaptation should be: e = x_pred - x_meas (opposite of tracking error)
    // This way: if x_meas > x_pred (positive tracking error), e is negative,
    // update reduces coefficients, reducing u_ff to compensate
    fixed::q31_t error = q31_sat_sub(x_pred_q31, x_meas_q31);
    
    // Scale error to voltage domain for coefficient update
    // The gradient of u_ff w.r.t. coefficients is the regressor φ
    
    // ---- 1st Order RLS ----
    for (size_t m = 0; m < VOLTERRA_1ST_COEFFS; ++m) {
        size_t idx = (F.delay_head + VOLTERRA_MEMORY_M - m) % VOLTERRA_MEMORY_M;
        fixed::q31_t phi = F.delay_line[idx];  // Regressor
        
        fixed::q31_t p = F.order1.lambda_inv[m];  // Actually storing P (inverse correlation)
        
        // k = p * phi / (λ + p * phi²)
        fixed::q31_t phi2 = q31_sat_mul(phi, phi);
        fixed::q31_t p_phi2 = q31_sat_mul(p, phi2);
        fixed::q31_t denom = q31_sat_add(F.rls_lambda_q31, p_phi2);
        
        // Avoid division - use approximation for small phi or precomputed inverse
        // For fixed-point, use: k ≈ p*phi/λ when p*phi² << λ
        fixed::q31_t k;
        if (q31_abs(p_phi2) < q31_abs(F.rls_lambda_q31) >> 2) {
            k = q31_sat_mul(p, phi);
            k = q31_sat_mul(k, fixed::q31_from_float(1.0f / RLS_LAMBDA));
        } else {
            // Use reciprocal approximation
            float denom_f = fixed::q31_to_float(denom);
            float k_f = fixed::q31_to_float(q31_sat_mul(p, phi)) / denom_f;
            k = fixed::q31_from_float(k_f);
        }
        
        // θ_new = θ_old + k * e
        fixed::q31_t update = q31_sat_mul(k, error);
        F.order1.g1[m] = q31_sat_add(F.order1.g1[m], update);
        
        // P_new = (P - k * phi * P) / λ
        fixed::q31_t k_phi = q31_sat_mul(k, phi);
        fixed::q31_t p_new = q31_sat_sub(p, q31_sat_mul(k_phi, p));
        p_new = q31_sat_mul(p_new, F.order1.lambda_inv[m]);  // /λ
        F.order1.lambda_inv[m] = p_new;
    }
    
    // ---- 2nd Order RLS ----
    constexpr int W = VOLTERRA_SPARSITY_WINDOW;
    for (size_t m1 = 0; m1 < VOLTERRA_MEMORY_M; ++m1) {
        size_t idx1 = (F.delay_head + VOLTERRA_MEMORY_M - m1) % VOLTERRA_MEMORY_M;
        fixed::q31_t x1 = F.delay_line[idx1];
        
        uint16_t mask = F.order2.valid_mask[m1];
        for (int dm = -W; dm <= W; ++dm) {
            if (!(mask & (1u << (dm + W)))) continue;
            int m2 = static_cast<int>(m1) + dm;
            if (m2 < 0 || m2 >= static_cast<int>(VOLTERRA_MEMORY_M)) continue;
            
            size_t idx2 = (F.delay_head + VOLTERRA_MEMORY_M - 1 - m2) % VOLTERRA_MEMORY_M;
            fixed::q31_t x2 = F.delay_line[idx2];
            
            size_t coeff_idx = volterra_2nd_index(static_cast<uint8_t>(m1), static_cast<uint8_t>(m2));
            if (coeff_idx == SIZE_MAX) continue;
            
            // Regressor: φ = x1 * x2
            fixed::q31_t phi = q31_sat_mul(x1, x2);
            if (m1 != static_cast<size_t>(m2)) {
                // Symmetric term appears twice in sum
                phi = q31_sat_add(phi, phi);
            }
            
            fixed::q31_t p = F.order2.p_inv[coeff_idx];
            fixed::q31_t phi2 = q31_sat_mul(phi, phi);
            fixed::q31_t p_phi2 = q31_sat_mul(p, phi2);
            fixed::q31_t denom = q31_sat_add(F.rls_lambda_q31, p_phi2);
            
            fixed::q31_t k;
            if (q31_abs(p_phi2) < q31_abs(F.rls_lambda_q31) >> 2) {
                k = q31_sat_mul(p, phi);
                k = q31_sat_mul(k, fixed::q31_from_float(1.0f / RLS_LAMBDA));
            } else {
                float denom_f = fixed::q31_to_float(denom);
                float k_f = fixed::q31_to_float(q31_sat_mul(p, phi)) / denom_f;
                k = fixed::q31_from_float(k_f);
            }
            
            fixed::q31_t update = q31_sat_mul(k, error);
            F.order2.g2[coeff_idx] = q31_sat_add(F.order2.g2[coeff_idx], update);
            
            fixed::q31_t k_phi = q31_sat_mul(k, phi);
            fixed::q31_t p_new = q31_sat_sub(p, q31_sat_mul(k_phi, p));
            p_new = q31_sat_mul(p_new, fixed::q31_from_float(1.0f / RLS_LAMBDA));
            F.order2.p_inv[coeff_idx] = p_new;
        }
    }
    
    // ---- 3rd Order RLS ----
    for (size_t i = 0; i < F.order3.num_active; ++i) {
        auto idxs = F.order3.index_map[i];
        size_t idx1 = (F.delay_head + VOLTERRA_MEMORY_M - idxs.m1) % VOLTERRA_MEMORY_M;
        size_t idx2 = (F.delay_head + VOLTERRA_MEMORY_M - idxs.m2) % VOLTERRA_MEMORY_M;
        size_t idx3 = (F.delay_head + VOLTERRA_MEMORY_M - idxs.m3) % VOLTERRA_MEMORY_M;
        
        fixed::q31_t x1 = F.delay_line[idx1];
        fixed::q31_t x2 = F.delay_line[idx2];
        fixed::q31_t x3 = F.delay_line[idx3];
        
        // Regressor: φ = x1 * x2 * x3
        fixed::q31_t phi = q31_sat_mul(x1, x2);
        phi = q31_sat_mul(phi, x3);
        
        fixed::q31_t p = F.order3.p_inv[i];
        fixed::q31_t phi2 = q31_sat_mul(phi, phi);
        fixed::q31_t p_phi2 = q31_sat_mul(p, phi2);
        fixed::q31_t denom = q31_sat_add(F.rls_lambda_q31, p_phi2);
        
        fixed::q31_t k;
        if (q31_abs(p_phi2) < q31_abs(F.rls_lambda_q31) >> 2) {
            k = q31_sat_mul(p, phi);
            k = q31_sat_mul(k, fixed::q31_from_float(1.0f / RLS_LAMBDA));
        } else {
            float denom_f = fixed::q31_to_float(denom);
            float k_f = fixed::q31_to_float(q31_sat_mul(p, phi)) / denom_f;
            k = fixed::q31_from_float(k_f);
        }
        
        fixed::q31_t update = q31_sat_mul(k, error);
        F.order3.g3[i] = q31_sat_add(F.order3.g3[i], update);
        
        fixed::q31_t k_phi = q31_sat_mul(k, phi);
        fixed::q31_t p_new = q31_sat_sub(p, q31_sat_mul(k_phi, p));
        p_new = q31_sat_mul(p_new, fixed::q31_from_float(1.0f / RLS_LAMBDA));
        F.order3.p_inv[i] = p_new;
    }
}

} // namespace dsp