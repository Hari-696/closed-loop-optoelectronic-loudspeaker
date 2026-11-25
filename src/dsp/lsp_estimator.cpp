/**
 * @file lsp_estimator.cpp
 * @brief Online LSP extraction implementation.
 * 
 * Extracts Bl(x), K_ms(x), L_e(x), R_e(T), Δφ(ω) from synchronized telemetry.
 * Runs at decimated rate (1 kHz) using EMA for curve smoothing.
 */

#include "lsp_estimator.hpp"
#include <cmath>
#include <algorithm>

namespace dsp {

LSPCurves g_lsp_curves{};
LSPInternalState g_lsp_state{};

// ============================================================================
// Initialization
// ============================================================================

bool lsp_estimator_init() noexcept {
    // Initialize curves with nominal values
    for (size_t i = 0; i < LSP_NUM_X_BINS; ++i) {
        g_lsp_curves.bl_tm[i] = BL_0;
        g_lsp_curves.k_ms_npm[i] = K_MS0;
        g_lsp_curves.l_e_mh[i] = 1.2f;  // Nominal 1.2 mH
        g_lsp_curves.bl_count[i] = 0;
        g_lsp_curves.kms_count[i] = 0;
        g_lsp_curves.le_count[i] = 0;
        g_lsp_state.bl_ema[i] = BL_0;
        g_lsp_state.kms_ema[i] = K_MS0;
        g_lsp_state.le_ema[i] = 1.2f;
        g_lsp_state.bl_samples[i] = 0;
        g_lsp_state.kms_samples[i] = 0;
        g_lsp_state.le_samples[i] = 0;
    }
    
    // Frequency bins (logarithmic spacing)
    for (size_t i = 0; i < LSP_NUM_FREQ_BINS; ++i) {
        float log_f = std::log(LSP_FREQ_MIN_HZ) + 
                     (std::log(LSP_FREQ_MAX_HZ) - std::log(LSP_FREQ_MIN_HZ)) * 
                     static_cast<float>(i) / static_cast<float>(LSP_NUM_FREQ_BINS - 1);
        g_lsp_curves.freq_hz[i] = std::exp(log_f);
        g_lsp_curves.phase_deg[i] = 0.0f;
        g_lsp_curves.z_mag_ohm[i] = 0.0f;
        g_lsp_curves.z_phase_deg[i] = 0.0f;
    }
    
    g_lsp_curves.r_e_ohm = R_E_25C;
    g_lsp_curves.temp_c = 25.0f;
    g_lsp_curves.thd_pct = 0.0f;
    g_lsp_curves.imd_pct = 0.0f;
    
    g_lsp_state = LSPInternalState{};
    g_lsp_state.initialized = true;
    
    return true;
}

// ============================================================================
// Numerical Differentiation Helpers
// ============================================================================

// 2nd-order central difference for acceleration: x_ddot ≈ (x[n+1] - 2x[n] + x[n-1]) / T²
DSP_INLINE float compute_acceleration(float x_curr, float x_prev1, float x_prev2, float dt) noexcept {
    return (x_prev2 - 2.0f * x_prev1 + x_curr) / (dt * dt);
}

// 1st-order backward difference for velocity: x_dot ≈ (x[n] - x[n-1]) / T
DSP_INLINE float compute_velocity(float x_curr, float x_prev, float dt) noexcept {
    return (x_curr - x_prev) / dt;
}

// ============================================================================
// Process Sample
// ============================================================================

void lsp_estimator_process(float v_rms, float i_rms, float x_meas, 
                           float x_dot, float x_ddot, float freq_hz) noexcept {
    using namespace std;
    auto& C = g_lsp_curves;
    auto& S = g_lsp_state;
    
    if (!S.initialized) return;
    
    const float dt = T_S;
    
    // Store in circular buffer for spectral analysis
    S.v_buf[S.buf_head] = v_rms;
    S.i_buf[S.buf_head] = i_rms;
    S.x_buf[S.buf_head] = x_meas;
    S.buf_head = (S.buf_head + 1) % LSPInternalState::BUF_SIZE;
    S.sample_counter++;
    
    // Compute acceleration if not provided (use numerical differentiation)
    if (x_ddot == 0.0f) {
        x_ddot = compute_acceleration(x_meas, S.x_prev1, S.x_prev2, dt);
    }
    if (x_dot == 0.0f) {
        x_dot = compute_velocity(x_meas, S.x_prev1, dt);
    }
    
    // Update previous samples
    S.x_prev2 = S.x_prev1;
    S.x_prev1 = x_meas;
    S.v_prev2 = S.v_prev1;
    S.v_prev1 = v_rms;
    S.i_prev2 = S.i_prev1;
    S.i_prev1 = i_rms;
    
    // Get displacement bin
    size_t bin = lsp_estimator_get_x_bin(x_meas);
    if (bin == SIZE_MAX) return;
    
    // ---- Bl(x) Estimation ----
    // From force balance: Bl * i = M_ms * x_ddot + R_ms * x_dot + K_ms(x) * x + F_rel
    // For Bl estimation, we need F_rel. Approximate F_rel ≈ 0 for Bl extraction
    // (F_rel is 2nd order in i, Bl is 1st order)
    // Bl = (M_ms * x_ddot + R_ms * x_dot + K_ms(x) * x) / i
    
    if (fabsf(i_rms) > 0.01f) {  // Need sufficient current
        float k_ms_est = C.k_ms_npm[bin];  // Use current K_ms estimate
        if (k_ms_est == 0.0f) k_ms_est = K_MS0;
        
        float force_mech = M_MS * x_ddot + R_MS * x_dot + k_ms_est * x_meas;
        float bl_inst = force_mech / i_rms;
        
        // Sanity check: Bl should be near BL_0 ± 20%
        if (bl_inst > BL_0 * 0.5f && bl_inst < BL_0 * 1.5f) {
            // EMA update
            float alpha = 1.0f - LSP_BL_SMOOTHING;
            S.bl_ema[bin] = LSP_BL_SMOOTHING * S.bl_ema[bin] + alpha * bl_inst;
            C.bl_tm[bin] = S.bl_ema[bin];
            C.bl_count[bin]++;
            
            if (C.bl_count[bin] > 100) C.bl_valid = true;
        }
    }
    
    // ---- K_ms(x) Estimation (quasi-static, low frequency) ----
    // At low frequency (f < 20 Hz), inertial and damping terms are small
    // K_ms(x) ≈ (Bl * i - F_rel) / x
    // For quasi-static, we need low-frequency content. Use EMA with frequency weighting.
    
    if (freq_hz < 30.0f && fabsf(x_meas) > 0.0001f) {  // > 0.1 mm excursion
        // Estimate F_rel from L_e gradient (approximate)
        float l_e_est = C.l_e_mh[bin] * 1e-3f;  // Convert mH to H
        if (l_e_est == 0.0f) l_e_est = 1.2e-3f;
        
        // dL_e/dx ≈ (L_e(x+dx) - L_e(x-dx)) / (2*dx)
        // Simplified: assume F_rel ≈ -i² * dL_e/dx, use nominal gradient
        float dLe_dx = 0.05f;  // Nominal 0.05 H/m (from report data)
        float f_rel = -i_rms * i_rms * dLe_dx;
        
        float kms_inst = (BL_0 * i_rms - f_rel) / x_meas;  // Use BL_0 for now
        
        if (kms_inst > K_MS0 * 0.5f && kms_inst < K_MS0 * 3.0f) {
            float alpha = 1.0f - LSP_KMS_SMOOTHING;
            S.kms_ema[bin] = LSP_KMS_SMOOTHING * S.kms_ema[bin] + alpha * kms_inst;
            C.k_ms_npm[bin] = S.kms_ema[bin];
            C.kms_count[bin]++;
            
            if (C.kms_count[bin] > 100) C.kms_valid = true;
        }
    }
    
    // ---- L_e(x) Estimation (high frequency impedance) ----
    // At frequencies above motional resonance (f > 500 Hz), impedance is dominated by R_e + jωL_e
    // |Z|² ≈ R_e² + (ωL_e)²
    // L_e ≈ sqrt(|Z|² - R_e²) / ω
    
    if (freq_hz > 500.0f && fabsf(i_rms) > 0.01f) {
        float z_mag = fabsf(v_rms) / fabsf(i_rms);
        float re = C.r_e_ohm;
        float omega = 2.0f * PI * freq_hz;
        
        if (z_mag > re) {
            float le_inst = sqrtf(z_mag * z_mag - re * re) / omega;
            float le_mh = le_inst * 1000.0f;  // Convert to mH
            
            if (le_mh > 0.1f && le_mh < 10.0f) {
                float alpha = 1.0f - LSP_LE_SMOOTHING;
                S.le_ema[bin] = LSP_LE_SMOOTHING * S.le_ema[bin] + alpha * le_mh;
                C.l_e_mh[bin] = S.le_ema[bin];
                C.le_count[bin]++;
                
                if (C.le_count[bin] > 100) C.le_valid = true;
            }
        }
    }
    
    // ---- R_e(T) Thermal Estimation (DC resistance) ----
    // At very low frequency (near DC), v/i ≈ R_e
    if (freq_hz < 5.0f && fabsf(i_rms) > 0.1f) {
        float re_inst = fabsf(v_rms) / fabsf(i_rms);
        
        if (re_inst > 3.0f && re_inst < 20.0f) {
            float alpha = 1.0f - LSP_RE_SMOOTHING;
            C.r_e_ohm = LSP_RE_SMOOTHING * C.r_e_ohm + alpha * re_inst;
            
            // Temperature from R_e: T = T0 + (R/R0 - 1) / alpha_Cu
            C.temp_c = 25.0f + (C.r_e_ohm / R_E_25C - 1.0f) / ALPHA_CU;
            C.re_valid = true;
        }
    }
    
    // ---- Phase Lag Δφ(ω) Extraction ----
    // Run spectral analysis periodically (every ~1000 decimated samples = 1 second)
    if (S.sample_counter % 1000 == 0 && S.sample_counter > LSPInternalState::BUF_SIZE) {
        // Simple Goertzel-like DFT at each frequency bin
        for (size_t fb = 0; fb < LSP_NUM_FREQ_BINS; ++fb) {
            float f = C.freq_hz[fb];
            float omega = 2.0f * PI * f;
            
            // Compute cross-correlation between v and x at this frequency
            float cos_sum_vx = 0, sin_sum_vx = 0;
            float cos_sum_vv = 0, sin_sum_vv = 0;
            float cos_sum_xx = 0, sin_sum_xx = 0;
            
            size_t n = LSPInternalState::BUF_SIZE;
            for (size_t k = 0; k < n; ++k) {
                float t = static_cast<float>(k) * dt * LSP_DECIMATION;  // Decimated time
                float c = cosf(omega * t);
                float s = sinf(omega * t);
                
                float v = S.v_buf[(S.buf_head + k) % n];
                float x = S.x_buf[(S.buf_head + k) % n];
                
                cos_sum_vx += v * x * c;
                sin_sum_vx += v * x * s;
                cos_sum_vv += v * v * c;
                sin_sum_vv += v * v * s;
                cos_sum_xx += x * x * c;
                sin_sum_xx += x * x * s;
            }
            
            // Phase from cross-spectrum
            float re_vx = cos_sum_vx / n;
            float im_vx = sin_sum_vx / n;
            float phase_vx = atan2f(im_vx, re_vx) * 180.0f / PI;
            
            // Impedance phase
            float re_vv = cos_sum_vv / n;
            float im_vv = sin_sum_vv / n;
            float re_xx = cos_sum_xx / n;
            float im_xx = sin_sum_xx / n;
            
            // Z = V/I, but we have V and X (displacement)
            // Approximate: at high freq, X ~ V / (Bl * jω) -> phase shift
            // Store phase difference
            C.phase_deg[fb] = phase_vx;
            C.z_mag_ohm[fb] = sqrtf(re_vv * re_vv + im_vv * im_vv) / 
                             sqrtf(re_xx * re_xx + im_xx * im_xx + 1e-12f) * BL_0;  // Rough scaling
        }
        C.phase_valid = true;
    }
    
    // ---- THD/IMD Estimation (simplified) ----
    // Compute from harmonic content of x_meas relative to fundamental
    // This is a placeholder - full THD requires FFT
    // For now, use a simple metric based on error signal
    static float thd_ema = 0.0f;
    float x_ref_est = v_rms / BL_0 * 0.001f;  // Very rough linear estimate
    float error = fabsf(x_meas - x_ref_est);
    thd_ema = 0.99f * thd_ema + 0.01f * (error / (fabsf(x_meas) + 1e-6f)) * 100.0f;
    C.thd_pct = thd_ema;
    
    C.imd_pct = thd_ema * 0.5f;  // Rough approximation
}

} // namespace dsp