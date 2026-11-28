#include <cmath>
/**
 * @file dsp_pipeline.cpp
 * @brief Complete DSP pipeline ISR implementation.
 * 
 * Chains: ADC → LUT → Observer → Volterra → MFB → Limiter → DAC → Ring Buffer
 * Worst-case < 60 µs at 96 kHz on Cortex-M7 @ 480 MHz.
 */

#include "dsp/dsp_pipeline.hpp"
#include "dsp/cubic_spline_lut.hpp"
#include "dsp/luenberger_observer.hpp"
#include "dsp/mfb_controller.hpp"
#include "dsp/volterra_filter.hpp"
#include "dsp/decoupling_limiter.hpp"
#include "dsp/lsp_estimator.hpp"
#ifndef ARM_MATH_CM7
#include "telemetry/spsc_ring_buffer.hpp"  // For telemetry push
#endif
#include <cstdint>

namespace dsp {

// Global state
DspFrame g_dsp_frame{};
PipelineStats g_pipeline_stats{};

// ============================================================================
// Reference Access (non-inline for consistent register allocation)
// ============================================================================

fixed::q31_t dsp_pipeline_get_reference_xmax() noexcept {
    return g_dsp_frame.x_ref;
}

// Cycle counter for profiling (Cortex-M7 DWT) - only on ARM
#if defined(__arm__) || defined(__ARM_ARCH)
static volatile uint32_t* const DWT_CYCCNT = reinterpret_cast<volatile uint32_t*>(0xE0001004);
static volatile uint32_t* const DWT_CTRL = reinterpret_cast<volatile uint32_t*>(0xE0001000);
static volatile uint32_t* const DEMCR = reinterpret_cast<volatile uint32_t*>(0xE000EDFC);
#else
// Host build: no cycle counter hardware
static uint32_t dummy_cycle_counter = 0;
static inline uint32_t get_cycle_count() noexcept { return 0; }
#endif

// ============================================================================
// Initialization
// ============================================================================

bool dsp_pipeline_init() noexcept {
#if defined(__arm__) || defined(__ARM_ARCH)
    // Enable cycle counter for profiling (Cortex-M7)
    *DEMCR |= (1 << 24);  // Enable DWT
    *DWT_CTRL |= (1 << 0);  // Enable cycle counter
#else
    // Host build: no cycle counter hardware
    (void)dummy_cycle_counter;
#endif
    
    // Initialize all submodules
    bool ok = true;
    ok &= cubic_spline_lut_init_from_model(D_0, R_F, PHOTODIODE_RESPONSIVITY, 1.0f);
    ok &= luenberger_observer_init(432.0f);  // 4× mechanical resonance
    ok &= mfb_controller_init(200.0f, CL_DAMPING_ZETA);
    ok &= volterra_filter_init(1.0f / X_MAX, 1.0f);
    ok &= decoupling_limiter_init();
    ok &= lsp_estimator_init();
#ifndef ARM_MATH_CM7
    ok &= telemetry::spsc_ring_buffer_init();
#endif
    
    // Reset pipeline state
    dsp_pipeline_reset();
    
    // Reset stats
    g_pipeline_stats = PipelineStats{};
    g_pipeline_stats.min_cycles = UINT32_MAX;
    
    return ok;
}

void dsp_pipeline_reset() noexcept {
    g_dsp_frame = DspFrame{};
    cubic_spline_lut_init_from_model(D_0, R_F, PHOTODIODE_RESPONSIVITY, 1.0f);  // Re-init LUT
    luenberger_observer_reset();
    mfb_controller_reset();
    volterra_filter_reset();
    // Also clear Volterra coefficients (they persist across reset)
    g_volterra_filter.order1.g1.fill(0);
    g_volterra_filter.order2.g2.fill(0);
    g_volterra_filter.order3.g3.fill(0);
    decoupling_limiter_reset();
    lsp_estimator_reset();
#ifndef ARM_MATH_CM7
    telemetry::spsc_ring_buffer_reset();
#endif
}

// ============================================================================
// ADC/DAC Conversion Helpers
// ============================================================================
// Q31 normalization: ±1.0 Q31 = ±5 V (ADC/DAC range)

DSP_INLINE fixed::q31_t adc_to_q31_voltage(int32_t adc_code) noexcept {
    // 24-bit signed ADC, ±5V range → normalized ±1.0 Q31
    // adc_code * (5.0f / 8388608.0f) gives voltage in volts
    constexpr float scale = 5.0f / 8388608.0f;
    return fixed::voltage_to_q31_adc(static_cast<float>(adc_code) * scale);
}

DSP_INLINE fixed::q31_t adc_to_q31_current(int32_t adc_code) noexcept {
    // Current sense: 0.1Ω shunt, 50× INA, ±5V ADC = ±10A range
    // ADC voltage in volts, then scaled by CURRENT_SCALE (5 V/A)
    constexpr float scale = 5.0f / 8388608.0f;
    fixed::q31_t v_q31 = fixed::voltage_to_q31_adc(static_cast<float>(adc_code) * scale);
    return fixed::q31_sat_mul(v_q31, fixed::q31_from_float(1.0f / CURRENT_SCALE));
}

DSP_INLINE int32_t q31_voltage_to_dac(fixed::q31_t v_q31) noexcept {
    // ±1.0 Q31 (full scale = ±150 V, amplifier range) → 24-bit signed DAC code
    // ±1.0 Q31 = full scale = ±8388607 codes = ±150 V at amplifier output
    float v = fixed::q31_to_float(v_q31);  // ±1.0 = full scale (±150 V)
    int32_t code = static_cast<int32_t>(v * 8388608.0f);
    if (code > 8388607) code = 8388607;
    if (code < -8388608) code = -8388608;
    return code;
}

// ============================================================================
// Telemetry Sample Construction (disabled on bare-metal)
// ============================================================================

#ifndef ARM_MATH_CM7
DSP_INLINE void push_telemetry_sample() noexcept {
    using namespace fixed;
    auto& F = g_dsp_frame;
    
    telemetry::TelemetrySample sample{};
    sample.x = q31_to_float(F.x_meas) * 1000.0f;           // m → mm
    sample.x_dot = q31_to_float(F.v_hat);                   // m/s
    sample.i = q31_to_float(F.i_meas);                      // A
    sample.v = q31_to_float(F.v_meas);                      // V
    sample.e = q31_to_float(F.e) * 1000.0f;                 // m → mm
    sample.u = q31_to_float(F.u_lim);                       // V
    
    // LSP estimates (from global curves)
    size_t bin = lsp_estimator_get_x_bin(q31_to_float(F.x_meas));
    if (bin != SIZE_MAX) {
        sample.Bl = g_lsp_curves.bl_tm[bin];
        sample.K_ms = g_lsp_curves.k_ms_npm[bin];
        sample.L_e = g_lsp_curves.l_e_mh[bin];
    } else {
        sample.Bl = BL_0;
        sample.K_ms = K_MS0;
        sample.L_e = 1.2f;
    }
    sample.R_e_T = g_lsp_curves.r_e_ohm;
    sample.thd = g_lsp_curves.thd_pct;
    sample.imd = g_lsp_curves.imd_pct;
    sample.ts = F.sample_count;  // Use sample counter as timestamp
    
    telemetry::spsc_ring_buffer_push(sample);
}
#else
DSP_INLINE void push_telemetry_sample() noexcept {
    // No-op on bare-metal
}
#endif

// ============================================================================
// Main ISR Step Function
// ============================================================================

void dsp_step(const AdcFrame* adc, DacFrame* dac) noexcept {
    using namespace fixed;
    
    // Start cycle counting
#if defined(__arm__) || defined(__ARM_ARCH)
    uint32_t start_cycles = *DWT_CYCCNT;
#else
    uint32_t start_cycles = 0;
    (void)start_cycles;  // Silence unused warning on host builds
#endif
    
    // ================================================================
    // Stage 1: ADC → Physical Units (Q31)
    // ================================================================
    fixed::q31_t v_adc_q31 = adc_to_q31_voltage(adc->displacement_adc);  // ±1.0 = ±5V
    fixed::q31_t i_meas_q31 = adc_to_q31_current(adc->current_adc);
    fixed::q31_t v_meas_q31 = adc_to_q31_voltage(adc->voltage_adc);
    
    // ================================================================
    // Stage 2: Cubic-Spline LUT Linearization
    // x_meas = S⁻¹(V_out)
    // LUT expects voltage in Q31 where ±1.0 = ±1V. ADC gives ±1.0 = ±5V.
    // Convert: v_lut = v_adc * 5.0 (scale from 5V FS to 1V FS)
    // Use float intermediate to avoid Q31 saturation
    // ================================================================
    float v_adc_volts = fixed::q31_to_float(v_adc_q31) * VOLTAGE_FULL_SCALE_V;
    constexpr float V_REST_V = -I_PHOTODIODE_REST * R_F;
    float v_safe = (v_adc_volts < -0.02f) ? v_adc_volts : -0.02f;
    float x_meas_m = D_0 * (1.0f - std::sqrt(V_REST_V / v_safe));
    float x_meas_norm = x_meas_m / X_LIM;
    if (x_meas_norm >  1.0f) x_meas_norm =  1.0f;
    if (x_meas_norm < -1.0f) x_meas_norm = -1.0f;
    fixed::q31_t x_meas_q31 = fixed::q31_from_float(x_meas_norm);
    
    // ================================================================
    // Stage 3: Luenberger Observer
    // x_hat, v_hat = observer_step(u_lim_prev, x_meas)
    // ================================================================
    // Use previous limited output as observer input
    fixed::q31_t u_observer_in = (g_dsp_frame.sample_count == 0) ? 0 : g_dsp_frame.u_lim;
    ObserverStateQ31 obs_state = luenberger_observer_step(u_observer_in, x_meas_q31);
    fixed::q31_t x_hat_q31 = obs_state.x_hat;
    fixed::q31_t v_hat_q31 = obs_state.v_hat;
    
    // ================================================================
    // Stage 4: Volterra Feedforward
    // u_ff = V(x_ref_history)
    // ================================================================
    fixed::q31_t u_ff_q31 = volterra_filter_eval(g_dsp_frame.x_ref);
    
    // ================================================================
    // Stage 5: MFB Feedback
    // u_fb = -K*x_hat + K_r*x_ref
    // MFB controller expects x_ref in X_MAX units (1.0 = 12mm)
    // ================================================================
    fixed::q31_t u_fb_q31 = mfb_controller_step(
    dsp_pipeline_get_reference_xmax(),
    x_hat_q31,
    v_hat_q31,
    x_meas_q31,
    v_hat_q31,
    dsp_pipeline_get_reference_xmax()
);
    
    // ================================================================
    // Stage 6: Decoupling Limiter (manual implementation)
    // u_lim = sat(u_ff + u_fb, ±V_max(x_meas))
    // ================================================================
    fixed::q31_t u_lim_amp_q31 = decoupling_limiter_apply_amp(0, u_fb_q31, x_meas_q31);
    bool rls_freeze = decoupling_limiter_rls_freeze();

    // u_lim_amp_q31 is already in AMP units (±1.0 = ±150V), pass directly to DAC
    // DAC converter now expects AMP units
    
    // ================================================================
    // Stage 7: Error Signal (for monitoring)
    // e = x_ref - x_meas
    // ================================================================
    fixed::q31_t e_q31 = q31_sat_sub(g_dsp_frame.x_ref, x_meas_q31);
    
    // ================================================================
    // Stage 8: LSP Estimator Update (decimated)
    // ================================================================
    if (g_dsp_frame.sample_count % LSP_DECIMATION == 0) {
        float x_meas_f = q31_to_float(x_meas_q31);
        float v_hat_f = q31_to_float(v_hat_q31);
        float v_meas_f = q31_to_float(v_meas_q31);
        float i_meas_f = q31_to_float(i_meas_q31);
        
        // Estimate acceleration from observer velocity (finite difference)
        static float v_hat_prev = 0.0f;
        float x_ddot_f = (v_hat_f - v_hat_prev) / T_S;
        v_hat_prev = v_hat_f;
        
        lsp_estimator_process(v_meas_f, i_meas_f, x_meas_f, v_hat_f, x_ddot_f, 100.0f);
    }
    
    // ================================================================
    // Stage 9: Volterra RLS Update (every 4th sample, if not frozen)
    // ================================================================
    if (!rls_freeze && (g_dsp_frame.sample_count % RLS_UPDATE_DECIMATION == 0)) {
        // Linear prediction from MFB path (simplified)
        fixed::q31_t x_pred_q31 = x_hat_q31;  // Observer prediction
        volterra_filter_rls_update(g_dsp_frame.x_ref, x_meas_q31, x_pred_q31, u_fb_q31);
    }
    
    // ================================================================
    // Stage 10: DAC Output (AMP units: ±1.0 Q31 = ±150 V)
    // ================================================================
    // q31_voltage_to_dac expects AMP units (±1.0 = ±150 V), pass directly
    int32_t dac_code = q31_voltage_to_dac(u_lim_amp_q31);
    
    // ================================================================
    // Stage 11: Telemetry Push
    // ================================================================
    push_telemetry_sample();
    
    // ================================================================
    // Update Frame State
    // ================================================================
    g_dsp_frame.x_meas = x_meas_q31;
    g_dsp_frame.i_meas = i_meas_q31;
    g_dsp_frame.v_meas = v_meas_q31;
    g_dsp_frame.x_hat = x_hat_q31;
    g_dsp_frame.v_hat = v_hat_q31;
    g_dsp_frame.u_ff = u_ff_q31;
    g_dsp_frame.u_fb = u_fb_q31;
    g_dsp_frame.u_lim = u_lim_amp_q31;
    g_dsp_frame.e = e_q31;
    g_dsp_frame.sample_count++;
    g_dsp_frame.limiter_active = g_limiter_state.limiter_active;
    
    // Output
    dac->dac_code = dac_code;
    dac->timestamp_ns = adc->timestamp_ns + 10417;  // +1 sample period in ns
    
    // ================================================================
    // Statistics
    // ================================================================
#if defined(__arm__) || defined(__ARM_ARCH)
    uint32_t elapsed = *DWT_CYCCNT - start_cycles;
#else
    uint32_t elapsed = 0;
#endif
    g_pipeline_stats.total_samples++;
    g_pipeline_stats.max_cycles = (elapsed > g_pipeline_stats.max_cycles) ? elapsed : g_pipeline_stats.max_cycles;
    g_pipeline_stats.min_cycles = (elapsed < g_pipeline_stats.min_cycles) ? elapsed : g_pipeline_stats.min_cycles;
    // Running average (EMA)
    g_pipeline_stats.avg_cycles = (g_pipeline_stats.avg_cycles * 99 + elapsed) / 100;
    g_pipeline_stats.cpu_load_pct = (g_pipeline_stats.avg_cycles * 100.0f) / (480000000.0f / 96000.0f);
}

// ============================================================================
// C-ABI Wrappers
// ============================================================================

extern "C" {

DSP_API int dsp_init_cabi(void) noexcept {
    return dsp::dsp_pipeline_init() ? 0 : -1;
}

DSP_API void dsp_reset_cabi(void) noexcept {
    dsp::dsp_pipeline_reset();
}

DSP_API void dsp_set_reference_cabi(float x_ref_mm) noexcept {
    dsp::dsp_pipeline_set_reference(x_ref_mm);
}

DSP_API int dsp_get_mfb_gains_cabi(int32_t* k1_out, int32_t* k2_out, int32_t* k_r_out) noexcept {
    if (!k1_out || !k2_out || !k_r_out) return -1;
    *k1_out = dsp::g_mfb_gains.k1;
    *k2_out = dsp::g_mfb_gains.k2;
    *k_r_out = dsp::g_mfb_gains.k_r;
    return 0;
}

DSP_API int dsp_debug_get_ref_cabi(int32_t* x_ref_out, int32_t* x_ref_mm_out) noexcept {
    if (!x_ref_out || !x_ref_mm_out) return -1;
    *x_ref_out = dsp::g_dsp_frame.x_ref;
    // Convert X_MAX Q31 to mm * 1000
    *x_ref_mm_out = static_cast<int32_t>(dsp::fixed::q31_to_float(dsp::g_dsp_frame.x_ref) * X_MAX * 1000000.0f);
    return 0;
}

DSP_API int dsp_debug_mfb_direct_cabi(int32_t* u_fb_out) noexcept {
    if (!u_fb_out) return -1;
    using namespace dsp;
    using namespace fixed;
    fixed::q31_t x_ref_q31 = dsp_pipeline_get_reference_xmax();
    fixed::q31_t x_hat_q31 = g_dsp_frame.x_hat;
    fixed::q31_t v_hat_q31 = g_dsp_frame.v_hat;
    fixed::q31_t x_meas_q31 = g_dsp_frame.x_meas;
    fixed::q31_t v_hat_vel_q31 = g_dsp_frame.v_hat;
    fixed::q31_t x_ref_q31_ref = dsp_pipeline_get_reference_xmax();
    (void)v_hat_q31; (void)x_ref_q31_ref;  // suppress unused warnings
    
    fixed::q31_t u_fb_q31 = mfb_controller_step(
        x_ref_q31, x_hat_q31, v_hat_q31,
        x_meas_q31, v_hat_vel_q31, x_ref_q31_ref
    );
    *u_fb_out = u_fb_q31;
    return 0;
}

DSP_API int dsp_debug_get_state_cabi(int32_t* x_ref_out, int32_t* x_meas_out, int32_t* x_hat_out, int32_t* v_hat_out, int32_t* u_ff_out, int32_t* u_fb_out, int32_t* u_lim_out) noexcept {
    if (!x_ref_out || !x_meas_out || !x_hat_out || !v_hat_out || !u_ff_out || !u_fb_out || !u_lim_out) return -1;
    using namespace dsp;
    using namespace fixed;
    *x_ref_out = g_dsp_frame.x_ref;
    *x_meas_out = g_dsp_frame.x_meas;
    *x_hat_out = g_dsp_frame.x_hat;
    *v_hat_out = g_dsp_frame.v_hat;
    *u_ff_out = g_dsp_frame.u_ff;
    *u_fb_out = g_dsp_frame.u_fb;
    *u_lim_out = g_dsp_frame.u_lim;
    return 0;
}

DSP_API int dsp_debug_get_limiter_state_cabi(int32_t* u_ff_out, int32_t* u_fb_out, int32_t* u_sum_out, int32_t* v_max_out, int32_t* u_lim_out) noexcept {
    if (!u_ff_out || !u_fb_out || !u_sum_out || !v_max_out || !u_lim_out) return -1;
    using namespace dsp;
    using namespace fixed;
    // Debug: directly use the integer constant for 1.0 Q31
    fixed::q31_t v_max = 2147483647;  // 1.0 Q31 = 2^31 - 1
    fixed::q31_t u_sum = fixed::q31_sat_add(g_dsp_frame.u_ff, g_dsp_frame.u_fb);
    fixed::q31_t u_lim;
    if (u_sum >= v_max) {
        u_lim = v_max;
    } else if (u_sum <= fixed::q31_neg(v_max)) {
        u_lim = fixed::q31_neg(v_max);
    } else {
        u_lim = u_sum;
    }
    *u_ff_out = g_dsp_frame.u_ff;
    *u_fb_out = g_dsp_frame.u_fb;
    *u_sum_out = u_sum;
    *v_max_out = v_max;
    *u_lim_out = u_lim;
    return 0;
}

DSP_API int dsp_debug_get_observer_matrices_cabi(
    int32_t* a_l00, int32_t* a_l01, int32_t* a_l10, int32_t* a_l11,
    int32_t* l0, int32_t* l1, int32_t* b0, int32_t* b1,
    int32_t* initialized
) noexcept {
    if (!a_l00 || !a_l01 || !a_l10 || !a_l11 || !l0 || !l1 || !b0 || !b1 || !initialized) return -1;
    using namespace dsp;
    using namespace fixed;
    const auto& M = g_observer_matrices;
    *a_l00 = M.a_l00;
    *a_l01 = M.a_l01;
    *a_l10 = M.a_l10;
    *a_l11 = M.a_l11;
    *l0 = M.l0;
    *l1 = M.l1;
    *b0 = M.b0;
    *b1 = M.b1;
    *initialized = g_observer_state.initialized ? 1 : 0;
    return 0;
}

DSP_API int dsp_debug_get_mfb_ref_cabi(int32_t* x_ref_out) noexcept {
    if (!x_ref_out) return -1;
    *x_ref_out = dsp::dsp_pipeline_get_reference_xmax();
    return 0;
}

DSP_API int dsp_debug_observer_step_cabi(int32_t u_observer_in, int32_t x_meas_q31, int32_t* x_hat_out, int32_t* v_hat_out) noexcept {
    if (!x_hat_out || !v_hat_out) return -1;
    using namespace dsp;
    using namespace fixed;
    fixed::q31_t u_q31 = static_cast<fixed::q31_t>(u_observer_in);
    fixed::q31_t x_m_q31 = static_cast<fixed::q31_t>(x_meas_q31);
    ObserverStateQ31 state = luenberger_observer_step(u_q31, x_m_q31);
    *x_hat_out = state.x_hat;
    *v_hat_out = state.v_hat;
    return 0;
}

DSP_API int dsp_debug_full_step_cabi(int32_t adc_disp, int32_t /*adc_curr*/, int32_t /*adc_volt*/,
                                      int32_t* x_meas_out, int32_t* x_hat_out, int32_t* v_hat_out,
                                      int32_t* u_ff_out, int32_t* u_fb_out, int32_t* u_lim_out,
                                      int32_t* dac_code_out) noexcept {
    if (!x_meas_out || !x_hat_out || !v_hat_out || !u_ff_out || !u_fb_out || !u_lim_out || !dac_code_out) return -1;
    using namespace dsp;
    using namespace fixed;
    
    // Stage 1: ADC → Voltage
    fixed::q31_t v_adc_q31 = adc_to_q31_voltage(adc_disp);
    
    // Stage 2: LUT
    // ADC Q31 (5V FS) → volts → LUT Q31 (1V FS)
    float v_adc_volts = fixed::q31_to_float(v_adc_q31) * 5.0f;
    fixed::q31_t v_lut_q31 = fixed::q31_from_float(v_adc_volts);
    fixed::q31_t x_meas_q31 = cubic_spline_lut_eval_q31(v_lut_q31);
    
    // Stage 3: Observer
    fixed::q31_t u_observer_in = (g_dsp_frame.sample_count == 0) ? 0 : g_dsp_frame.u_lim;
    ObserverStateQ31 obs_state = luenberger_observer_step(u_observer_in, x_meas_q31);
    fixed::q31_t x_hat_q31 = obs_state.x_hat;
    fixed::q31_t v_hat_q31 = obs_state.v_hat;
    
    // Stage 4: Volterra
    fixed::q31_t u_ff_q31 = volterra_filter_eval(g_dsp_frame.x_ref);
    
    // Stage 5: MFB - compute manually to avoid Q31Accumulator register pressure issues
    fixed::q31_t x_ref_q31 = dsp_pipeline_get_reference_xmax();
    const auto& G = g_mfb_gains;
    
    // fb = -(k1*x_hat + k2*v_hat)
    fixed::q31_t fb = fixed::q31_neg(
        fixed::q31_sat_add(
            fixed::q31_sat_mul(G.k1, x_hat_q31),
            fixed::q31_sat_mul(G.k2, v_hat_q31)
        )
    );
    
    // ff = k_r * x_ref
    fixed::q31_t ff = fixed::q31_sat_mul(G.k_r, x_ref_q31);
    
    // u_fb = fb + ff
    fixed::q31_t u_fb_q31 = fixed::q31_sat_add(fb, ff);
    
    // Stage 6: Limiter - manual implementation to avoid register pressure issues
    // Compute V_max based on displacement
    fixed::q31_t x_abs = fixed::q31_abs(x_meas_q31);
    fixed::q31_t knee_q31 = fixed::q31_from_float(LIMITER_KNEE_X / X_LIM);
    fixed::q31_t v_max;
    if (x_abs <= knee_q31) {
        v_max = fixed::q31_from_float(LIMITER_V_NOMINAL);
    } else if (x_abs >= fixed::q31_from_float(1.0f)) {
        v_max = 0;
    } else {
        fixed::q31_t pos = fixed::q31_sat_sub(x_abs, fixed::q31_from_float(LIMITER_KNEE_X / X_LIM));
        fixed::q31_t norm = fixed::q31_sat_mul(pos, fixed::q31_from_float(1.0f / (1.0f - LIMITER_KNEE_RATIO)));
        fixed::q31_t norm2 = fixed::q31_sat_mul(norm, norm);
        fixed::q31_t one = fixed::q31_from_float(1.0f);
        fixed::q31_t factor = fixed::q31_sat_sub(one, norm2);
        v_max = fixed::q31_sat_mul(fixed::q31_from_float(LIMITER_V_NOMINAL), factor);
    }
    
    // Sum feedforward and feedback (AMP units)
    fixed::q31_t u_sum = fixed::q31_sat_add(u_ff_q31, u_fb_q31);
    
    // Symmetric saturation in AMP units (use >= / <= to clamp at boundary)
    fixed::q31_t u_lim_amp_q31;
    if (u_sum >= v_max) {
        u_lim_amp_q31 = v_max;
    } else if (u_sum <= fixed::q31_neg(v_max)) {
        u_lim_amp_q31 = fixed::q31_neg(v_max);
    } else {
        u_lim_amp_q31 = u_sum;
    }
    
    // q31_voltage_to_dac expects AMP units (±1.0 = ±150 V), pass directly
    int32_t dac_code = q31_voltage_to_dac(u_lim_amp_q31);
    
    // Return values (including x_ref for debug)
    *x_meas_out = x_meas_q31;
    *x_hat_out = x_hat_q31;
    *v_hat_out = v_hat_q31;
    *u_ff_out = u_ff_q31;
    *u_fb_out = u_fb_q31;
    *u_lim_out = u_lim_amp_q31;
    *dac_code_out = dac_code;
    
    return 0;
}

DSP_API int dsp_debug_mfb_step_cabi(float x_ref_mm, float x_hat_mm, float v_hat_mps, int32_t* u_fb_out, int32_t* fb_out, int32_t* ff_out) noexcept {
    if (!u_fb_out || !fb_out || !ff_out) return -1;
    using namespace dsp;
    using namespace fixed;
    
    fixed::q31_t x_ref_q31 = fixed::q31_from_float(x_ref_mm / 1000.0f / X_MAX);
    fixed::q31_t x_hat_q31 = fixed::q31_from_float(x_hat_mm / 1000.0f / X_LIM);
    fixed::q31_t v_hat_q31 = fixed::q31_from_float(v_hat_mps / 10.0f);
    fixed::q31_t x_ref_q31_ref = x_ref_q31;
    
    // Use x_hat as x_meas proxy (debug function doesn't have real ADC data)
    // Use v_hat as velocity estimate
    fixed::q31_t u_fb_q31 = mfb_controller_step(
        x_ref_q31, 0, 0,
        x_hat_q31, v_hat_q31, x_ref_q31_ref
    );
    
    // Compute fb and ff separately for debug output using observer states
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
    
    *u_fb_out = u_fb_q31;
    *fb_out = fb;
    *ff_out = ff;
    return 0;
}

DSP_API int dsp_debug_pipeline_step_cabi(float x_ref_mm, int32_t adc_disp, int32_t adc_curr, int32_t adc_volt,
                                          float* x_meas_out, float* x_hat_out, float* v_hat_out,
                                          int32_t* u_ff_out, int32_t* u_fb_out, int32_t* u_lim_out,
                                          int32_t* dac_code_out) noexcept {
    if (!x_meas_out || !x_hat_out || !v_hat_out || !u_ff_out || !u_fb_out || !u_lim_out || !dac_code_out) return -1;
    
    // Create ADC frame
    dsp::AdcFrame adc;
    adc.displacement_adc = adc_disp;
    adc.current_adc = adc_curr;
    adc.voltage_adc = adc_volt;
    adc.timestamp_ns = 0;
    
    dsp::DacFrame dac;
    dac.dac_code = 0;
    dac.timestamp_ns = 0;
    
    // Set reference
    dsp::dsp_pipeline_set_reference(x_ref_mm);
    
    // Execute step
    dsp::dsp_step(&adc, &dac);
    
    // Capture results (convert from normalized Q31 to physical units)
    constexpr float X_LIM_MM = X_LIM * 1000.0f;  // 15mm
    constexpr float VEL_SCALE_MMPS = 10.0f * 1000.0f;  // 10 m/s = 10000 mm/s
    *x_meas_out = fixed::q31_to_float(dsp::g_dsp_frame.x_meas) * X_LIM_MM;
    *x_hat_out = fixed::q31_to_float(dsp::g_dsp_frame.x_hat) * X_LIM_MM;
    *v_hat_out = fixed::q31_to_float(dsp::g_dsp_frame.v_hat) * VEL_SCALE_MMPS;
    *u_ff_out = dsp::g_dsp_frame.u_ff;
    *u_fb_out = dsp::g_dsp_frame.u_fb;
    *u_lim_out = dsp::g_dsp_frame.u_lim;
    *dac_code_out = dac.dac_code;
    
    return 0;
}

DSP_API int dsp_step_cabi(const void* adc, void* dac) noexcept {
    if (!adc || !dac) return -1;
    dsp::dsp_step(static_cast<const dsp::AdcFrame*>(adc), static_cast<dsp::DacFrame*>(dac));
    return 0;
}

} // extern "C"
} // namespace dsp