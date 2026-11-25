#pragma once

/**
 * @file lsp_estimator.hpp
 * @brief Online Large-Signal Parameter (LSP) extraction.
 * 
 * Extracts from synchronized v(t), i(t), x(t) telemetry:
 * - Bl(x) from force balance: Bl = (M_ms*x_ddot + R_ms*x_dot + K_ms(x)*x + F_rel) / i
 * - K_ms(x) from quasi-static restoring force at low frequency
 * - L_e(x) from impedance magnitude above motional region
 * - R_e(T) from DC resistance tracks (thermal)
 * - Δφ(ω) from transfer function between drive voltage and displacement
 * 
 * Runs at decimated rate (1 kHz) to reduce computation.
 */

#include <cstdint>
#include <array>
#include <cmath>
#include "config.hpp"
#include "fixed_point.hpp"

namespace dsp {

// ============================================================================
// LSP Estimator Configuration
// ============================================================================

constexpr size_t LSP_NUM_X_BINS = 64;           ///< Displacement bins for LSP curves (-X_LIM to +X_LIM)
constexpr size_t LSP_DECIMATION = 96;           ///< Update every 96 samples (1 kHz at 96 kHz)
constexpr float LSP_BL_SMOOTHING = 0.99f;       ///< Bl estimate smoothing (EMA)
constexpr float LSP_KMS_SMOOTHING = 0.99f;      ///< K_ms estimate smoothing
constexpr float LSP_LE_SMOOTHING = 0.99f;       ///< L_e estimate smoothing
constexpr float LSP_RE_SMOOTHING = 0.999f;      ///< R_e estimate smoothing (slow thermal)

// Frequency bins for phase/impedance analysis
constexpr size_t LSP_NUM_FREQ_BINS = 16;
constexpr float LSP_FREQ_MIN_HZ = 20.0f;
constexpr float LSP_FREQ_MAX_HZ = 5000.0f;

// ============================================================================
// LSP State Structures
// ============================================================================

struct LSPCurves {
    // Displacement-dependent curves (indexed by x bin)
    std::array<float, LSP_NUM_X_BINS> bl_tm;        ///< Bl(x) [T·m]
    std::array<float, LSP_NUM_X_BINS> k_ms_npm;     ///< K_ms(x) [N/m]
    std::array<float, LSP_NUM_X_BINS> l_e_mh;       ///< L_e(x) [mH]
    std::array<uint32_t, LSP_NUM_X_BINS> bl_count;  ///< Sample counts for averaging
    std::array<uint32_t, LSP_NUM_X_BINS> kms_count;
    std::array<uint32_t, LSP_NUM_X_BINS> le_count;
    
    // Thermal
    float r_e_ohm = R_E_25C;                        ///< R_e(T) [Ω]
    float temp_c = 25.0f;                           ///< Voice-coil temperature [°C]
    
    // Phase/impedance vs frequency
    std::array<float, LSP_NUM_FREQ_BINS> freq_hz;
    std::array<float, LSP_NUM_FREQ_BINS> phase_deg; ///< Δφ(ω) [deg]
    std::array<float, LSP_NUM_FREQ_BINS> z_mag_ohm; ///< |Z(ω)| [Ω]
    std::array<float, LSP_NUM_FREQ_BINS> z_phase_deg; ///< ∠Z(ω) [deg]
    
    // Distortion estimates
    float thd_pct = 0.0f;
    float imd_pct = 0.0f;
    
    // Validation flags
    bool bl_valid = false;
    bool kms_valid = false;
    bool le_valid = false;
    bool re_valid = false;
    bool phase_valid = false;
};

struct LSPInternalState {
    // Running sums for EMA
    float bl_ema[LSP_NUM_X_BINS];
    float kms_ema[LSP_NUM_X_BINS];
    float le_ema[LSP_NUM_X_BINS];
    uint32_t bl_samples[LSP_NUM_X_BINS];
    uint32_t kms_samples[LSP_NUM_X_BINS];
    uint32_t le_samples[LSP_NUM_X_BINS];
    
    // Signal buffers for spectral analysis (decimated)
    static constexpr size_t BUF_SIZE = 1024;
    float v_buf[BUF_SIZE];
    float i_buf[BUF_SIZE];
    float x_buf[BUF_SIZE];
    size_t buf_head = 0;
    uint32_t sample_counter = 0;
    
    // Previous samples for numerical differentiation
    float x_prev1 = 0, x_prev2 = 0;
    float v_prev1 = 0, v_prev2 = 0;
    float i_prev1 = 0, i_prev2 = 0;
    
    // Initialization
    bool initialized = false;
};

extern LSPCurves g_lsp_curves;
extern LSPInternalState g_lsp_state;

// ============================================================================
// Public API
// ============================================================================

/**
 * @brief Initialize LSP estimator.
 */
bool lsp_estimator_init() noexcept;

/**
 * @brief Reset LSP estimator state.
 */
DSP_INLINE void lsp_estimator_reset() noexcept {
    g_lsp_curves = LSPCurves{};
    g_lsp_state = LSPInternalState{};
    // Re-init frequency bins
    for (size_t i = 0; i < LSP_NUM_FREQ_BINS; ++i) {
        float log_f = std::log(LSP_FREQ_MIN_HZ) + 
                     (std::log(LSP_FREQ_MAX_HZ) - std::log(LSP_FREQ_MIN_HZ)) * 
                     static_cast<float>(i) / static_cast<float>(LSP_NUM_FREQ_BINS - 1);
        g_lsp_curves.freq_hz[i] = std::exp(log_f);
    }
}

/**
 * @brief Get displacement bin index from measured displacement.
 * 
 * @param x_meas Measured displacement [m]
 * @return Bin index [0, LSP_NUM_X_BINS-1], or SIZE_MAX if out of range
 */
DSP_INLINE size_t lsp_estimator_get_x_bin(float x_meas) noexcept {
    if (x_meas < -X_LIM || x_meas > X_LIM) return SIZE_MAX;
    float normalized = (x_meas + X_LIM) / (2.0f * X_LIM);  // 0 to 1
    size_t bin = static_cast<size_t>(normalized * static_cast<float>(LSP_NUM_X_BINS));
    if (bin >= LSP_NUM_X_BINS) bin = LSP_NUM_X_BINS - 1;
    return bin;
}

/**
 * @brief Process one sample for LSP extraction (called at decimated rate).
 * 
 * Computes instantaneous estimates and updates EMA curves.
 * 
 * @param v_rms Drive voltage [V]
 * @param i_rms Voice-coil current [A]
 * @param x_meas Measured displacement [m]
 * @param x_dot Estimated velocity [m/s]
 * @param x_ddot Estimated acceleration [m/s²] (can be 0 if not available)
 * @param freq_hz Current excitation frequency [Hz] (for phase extraction)
 */
void lsp_estimator_process(float v_rms, float i_rms, float x_meas, 
                           float x_dot, float x_ddot, float freq_hz) noexcept;

/**
 * @brief Get current LSP curves (for telemetry/logging).
 */
DSP_INLINE const LSPCurves& lsp_estimator_get_curves() noexcept {
    return g_lsp_curves;
}

/**
 * @brief Check if LSP estimator has valid data.
 */
DSP_INLINE bool lsp_estimator_ready() noexcept {
    return g_lsp_state.initialized && g_lsp_curves.bl_valid;
}

} // namespace dsp