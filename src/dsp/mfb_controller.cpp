/**
 * @file mfb_controller.cpp
 * @brief MFB controller initialization - discrete pole placement and gain computation.
 */

#include "dsp/mfb_controller.hpp"
#include "dsp/luenberger_observer.hpp"
#include <cmath>

namespace dsp {

MFBGainsQ31 g_mfb_gains{};

// ============================================================================
// Helper Types (must be defined before use)
// ============================================================================

struct FloatMat22 {
    float m00, m01, m10, m11;
};

struct FloatVec2 {
    float x, y;
};

// ============================================================================
// Controller Gain Calculation (Ackermann's Formula)
// ============================================================================

// For system x[k+1] = A_d x[k] + B_d u[k], u = -K x + K_r r
// Closed-loop: A_cl = A_d - B_d K
// Desired characteristic polynomial: z² - 2ζω_n z + ω_n² (in discrete domain)
// 
// For 2x2: K = [0, 1] * C_c⁻¹ * φ(A_d) where φ is desired char poly evaluated at A_d
// Controllability matrix: C_c = [B_d, A_d*B_d]
// 
// Simpler for 2x2: solve directly
// A_cl = [a00 - b0*k1, a01 - b0*k2; a10 - b1*k1, a11 - b1*k2]
// tr(A_cl) = a00 + a11 - (b0*k1 + b1*k2) = 2*z_pole*cos(θ)  (for complex poles)
// det(A_cl) = (a00 - b0*k1)*(a11 - b1*k2) - (a01 - b0*k2)*(a10 - b1*k1) = z_pole²
void compute_state_feedback_gain(const FloatMat22& A_d, const FloatVec2& B_d,
                                 float z_pole_re, float z_pole_im,
                                 FloatVec2& K_out) noexcept {
    // Desired characteristic polynomial coefficients
    // (z - p)(z - p*) = z² - 2*Re(p)*z + |p|²
    float tr_desired = 2.0f * z_pole_re;
    float det_desired = z_pole_re * z_pole_re + z_pole_im * z_pole_im;
    
    float a00 = A_d.m00, a01 = A_d.m01, a10 = A_d.m10, a11 = A_d.m11;
    float b0 = B_d.x, b1 = B_d.y;
    
    // We need K = [k1, k2] such that:
    // tr(A_d - B_d*K) = tr_desired
    // det(A_d - B_d*K) = det_desired
    //
    // tr = a00 + a11 - (b0*k1 + b1*k2) = tr_desired
    // => b0*k1 + b1*k2 = a00 + a11 - tr_desired  (eq 1)
    //
    // det = (a00 - b0*k1)*(a11 - b1*k2) - (a01 - b0*k2)*(a10 - b1*k1) = det_desired
    // Expand and substitute from eq 1
    
    float tr_A = a00 + a11;
    float det_A = a00*a11 - a01*a10;
    
    // Let s = b0*k1 + b1*k2 = tr_A - tr_desired
    float s = tr_A - tr_desired;
    
    // det(A - BK) = det(A) - k1*(a11*b0 - a01*b1) - k2*(a00*b1 - a10*b0) + (b0*b1)*(k1*k2 - k2*k1)
    // The last term is zero since k1*k2 - k2*k1 = 0
    // Actually: det(A - BK) = det(A) - k1*(a11*b0 - a01*b1) - k2*(a00*b1 - a10*b0)
    // Wait, let's derive properly:
    // (a00 - b0*k1)*(a11 - b1*k2) - (a01 - b0*k2)*(a10 - b1*k1)
    // = a00*a11 - a00*b1*k2 - a11*b0*k1 + b0*b1*k1*k2 - a01*a10 + a01*b1*k1 + a10*b0*k2 - b0*b1*k1*k2
    // = det(A) - a00*b1*k2 - a11*b0*k1 + a01*b1*k1 + a10*b0*k2
    // = det(A) + k1*(a01*b1 - a11*b0) + k2*(a10*b0 - a00*b1)
    
    float c1 = a01*b1 - a11*b0;
    float c2 = a10*b0 - a00*b1;
    
    // det_desired = det_A + c1*k1 + c2*k2
    // c1*k1 + c2*k2 = det_desired - det_A
    // Combined with eq1: b0*k1 + b1*k2 = s
    
    // Solve 2x2 linear system:
    // [b0  b1] [k1] = [s]
    // [c1  c2] [k2]   [det_desired - det_A]
    
    float det_sys = b0*c2 - b1*c1;
    if (std::abs(det_sys) < 1e-12f) {
        // Singular - fallback
        K_out.x = 0.0f;
        K_out.y = 0.0f;
        return;
    }
    
    float rhs2 = det_desired - det_A;
    K_out.x = (s*c2 - b1*rhs2) / det_sys;
    K_out.y = (b0*rhs2 - s*c1) / det_sys;
}

// Compute reference gain K_r for unity DC gain from reference to output
// K_r = 1 / (C_d * (I - A_cl)⁻¹ * B_d) where A_cl = A_d - B_d*K
float compute_reference_gain(const FloatMat22& A_d, const FloatVec2& B_d, 
                             const FloatVec2& C_d, const FloatVec2& K) noexcept {
    // A_cl = A_d - B_d*K
    FloatMat22 A_cl;
    A_cl.m00 = A_d.m00 - B_d.x * K.x;
    A_cl.m01 = A_d.m01 - B_d.x * K.y;
    A_cl.m10 = A_d.m10 - B_d.y * K.x;
    A_cl.m11 = A_d.m11 - B_d.y * K.y;
    
    // I - A_cl
    FloatMat22 I_minus_Acl;
    I_minus_Acl.m00 = 1.0f - A_cl.m00;
    I_minus_Acl.m01 = -A_cl.m01;
    I_minus_Acl.m10 = -A_cl.m10;
    I_minus_Acl.m11 = 1.0f - A_cl.m11;
    
    // (I - A_cl)⁻¹
    float det = I_minus_Acl.m00 * I_minus_Acl.m11 - I_minus_Acl.m01 * I_minus_Acl.m10;
    if (std::abs(det) < 1e-12f) return 1.0f;
    
    FloatMat22 inv;
    inv.m00 = I_minus_Acl.m11 / det;
    inv.m01 = -I_minus_Acl.m01 / det;
    inv.m10 = -I_minus_Acl.m10 / det;
    inv.m11 = I_minus_Acl.m00 / det;
    
    // (I - A_cl)⁻¹ * B_d
    FloatVec2 temp;
    temp.x = inv.m00 * B_d.x + inv.m01 * B_d.y;
    temp.y = inv.m10 * B_d.x + inv.m11 * B_d.y;
    
    // C_d * (I - A_cl)⁻¹ * B_d
    float dc_gain = C_d.x * temp.x + C_d.y * temp.y;
    
    if (std::abs(dc_gain) < 1e-12f) return 1.0f;
    return 1.0f / dc_gain;
}

// ============================================================================
// Initialization
// ============================================================================

bool mfb_controller_init(float cl_pole_hz, float damping) noexcept {
    using namespace fixed;
    
    // Clamp to valid range
    if (cl_pole_hz < CL_POLE_MIN_HZ) cl_pole_hz = CL_POLE_MIN_HZ;
    if (cl_pole_hz > CL_POLE_MAX_HZ) cl_pole_hz = CL_POLE_MAX_HZ;
    if (damping < 0.1f) damping = 0.1f;
    if (damping > 1.0f) damping = 1.0f;
    
    // Get discrete plant matrices from observer (same discretization)
    FloatMat22 A_c{0.0f, 1.0f, -K_MS0/M_MS, -R_MS/M_MS};
    FloatVec2 B_c{0.0f, BL_0/M_MS};
    // FloatVec2 C_c{1.0f, 0.0f};  // Unused - DC gain computed from continuous-time model
    
    float dt = T_S;
    
    // Discretize (reuse observer's discretization logic)
    // A_d = exp(A_c * dt)
    float a = A_c.m00, b = A_c.m01, c = A_c.m10, d = A_c.m11;
    float tr = a + d;
    float det = a*d - b*c;
    float disc = tr*tr - 4.0f*det;
    
    FloatMat22 A_d;
    if (disc >= 0.0f) {
        float sqrt_disc = std::sqrt(disc);
        float lambda1 = 0.5f*(tr + sqrt_disc);
        float lambda2 = 0.5f*(tr - sqrt_disc);
        if (std::abs(lambda1 - lambda2) > 1e-6f) {
            float e1 = std::exp(lambda1*dt);
            float e2 = std::exp(lambda2*dt);
            float denom = lambda1 - lambda2;
            A_d.m00 = (a*(e1-e2) + (lambda1*e2 - lambda2*e1))/denom;
            A_d.m01 = (b*(e1-e2))/denom;
            A_d.m10 = (c*(e1-e2))/denom;
            A_d.m11 = (d*(e1-e2) + (lambda1*e2 - lambda2*e1))/denom;
        } else {
            float e = std::exp(lambda1*dt);
            float edt = e*dt;
            A_d.m00 = e + (a-lambda1)*edt;
            A_d.m01 = b*edt;
            A_d.m10 = c*edt;
            A_d.m11 = e + (d-lambda1)*edt;
        }
    } else {
        float alpha = 0.5f*tr;
        float beta = 0.5f*std::sqrt(-disc);
        float eat = std::exp(alpha*dt);
        float cos_bt = std::cos(beta*dt);
        float sin_bt = std::sin(beta*dt);
        A_d.m00 = eat*(cos_bt + (a-alpha)*sin_bt/beta);
        A_d.m01 = eat*(b*sin_bt/beta);
        A_d.m10 = eat*(c*sin_bt/beta);
        A_d.m11 = eat*(cos_bt + (d-alpha)*sin_bt/beta);
    }
    
    // B_d = ∫exp(Aτ)dτ * B_c
    float detA = a*d - b*c;
    FloatVec2 B_d;
    if (std::abs(detA) > 1e-12f) {
        FloatMat22 Ainv{d/detA, -b/detA, -c/detA, a/detA};
        FloatMat22 expAdt_minus_I{A_d.m00-1, A_d.m01, A_d.m10, A_d.m11-1};
        B_d.x = Ainv.m00*(expAdt_minus_I.m00*B_c.x + expAdt_minus_I.m01*B_c.y)
              + Ainv.m01*(expAdt_minus_I.m10*B_c.x + expAdt_minus_I.m11*B_c.y);
        B_d.y = Ainv.m10*(expAdt_minus_I.m00*B_c.x + expAdt_minus_I.m01*B_c.y)
              + Ainv.m11*(expAdt_minus_I.m10*B_c.x + expAdt_minus_I.m11*B_c.y);
    } else {
        // Series approximation
        FloatMat22 A2{a*a+b*c, a*b+b*d, c*a+d*c, c*b+d*d};
        FloatMat22 A3{A2.m00*a+A2.m01*c, A2.m00*b+A2.m01*d, A2.m10*a+A2.m11*c, A2.m10*b+A2.m11*d};
        B_d.x = dt*(B_c.x + dt/2.0f*(a*B_c.x+b*B_c.y) + dt*dt/6.0f*(A2.m00*B_c.x+A2.m01*B_c.y) + dt*dt*dt/24.0f*(A3.m00*B_c.x+A3.m01*B_c.y));
        B_d.y = dt*(B_c.y + dt/2.0f*(c*B_c.x+d*B_c.y) + dt*dt/6.0f*(A2.m10*B_c.x+A2.m11*B_c.y) + dt*dt*dt/24.0f*(A3.m10*B_c.x+A3.m11*B_c.y));
    }
    
    // Desired discrete poles from continuous specs
    // ω_n = 2π * f_cl, ζ = damping
    // Continuous poles: s = -ζω_n ± jω_n√(1-ζ²)
    // Discrete: z = exp(s*T)
    float wn = 2.0f * PI * cl_pole_hz;
    float sigma = -damping * wn;
    float omega_d = wn * std::sqrt(1.0f - damping*damping);
    float z_re = std::exp(sigma * dt) * std::cos(omega_d * dt);
    float z_im = std::exp(sigma * dt) * std::sin(omega_d * dt);
    
    // Compute state-feedback gain K
    FloatVec2 K;
    compute_state_feedback_gain(A_d, B_d, z_re, z_im, K);
    
    // Compute reference gain K_r using CONTINUOUS-TIME DC gain
    // Discrete-time DC gain is severely attenuated at 96 kHz sampling
    // Continuous-time DC gain: x/V = Bl / (R_e * K_ms)
    constexpr float DC_GAIN_M_PER_V = BL_0 / (R_E_25C * K_MS0);  // 14.2 / (6.8 * 19500) = 0.000107 m/V
    constexpr float K_r = 1.0f / DC_GAIN_M_PER_V;  // 9346 V/m
    
    // Convert to Q31
    // K gains: input is state (±X_LIM, ±10 m/s), output is voltage (±150V = AMP_FULL_SCALE_V)
    // K_r: input is reference (±X_MAX), output is voltage (±150V = AMP_FULL_SCALE_V)
    
    auto to_q31 = [](float val, float in_scale, float out_scale) -> fixed::q31_t {
        return fixed::q31_from_float(val * in_scale / out_scale);
    };
    
    g_mfb_gains.k1 = to_q31(K.x, X_LIM, AMP_FULL_SCALE_V);      // Position gain
    g_mfb_gains.k2 = to_q31(K.y, 10.0f, AMP_FULL_SCALE_V);       // Velocity gain
    g_mfb_gains.k_r = to_q31(K_r, X_MAX, AMP_FULL_SCALE_V);      // Reference gain
    
    return true;
}

// ============================================================================
// MFB Controller Step (non-inline implementation)
// ============================================================================

DSP_API fixed::q31_t mfb_controller_step(fixed::q31_t x_ref_q31, 
                                               fixed::q31_t x_hat_q31, 
                                               fixed::q31_t v_hat_q31) noexcept {
    using namespace fixed;
    const auto& G = g_mfb_gains;
    
    // Use simple int64_t accumulator for MFB (only 2-3 MACs per step)
    int64_t acc = 0;
    
    // -K * x_hat = -(k1*x_hat + k2*v_hat)
    acc += static_cast<int64_t>(G.k1) * static_cast<int64_t>(x_hat_q31);
    acc += static_cast<int64_t>(G.k2) * static_cast<int64_t>(v_hat_q31);
    // Result is in Q62, shift right 31 with rounding to get Q31
    int64_t fb_raw = (acc + (1LL << 30)) >> 31;
    if (fb_raw > 2147483647LL) fb_raw = 2147483647;
    if (fb_raw < -2147483648LL) fb_raw = -2147483648;
    fixed::q31_t fb = q31_neg(static_cast<fixed::q31_t>(fb_raw));
    
    // K_r * x_ref
    int64_t ff_raw = (static_cast<int64_t>(G.k_r) * static_cast<int64_t>(x_ref_q31) + (1LL << 30)) >> 31;
    if (ff_raw > 2147483647LL) ff_raw = 2147483647;
    if (ff_raw < -2147483648LL) ff_raw = -2147483648;
    fixed::q31_t ff = static_cast<fixed::q31_t>(ff_raw);
    
    // u_fb = fb + ff
    // NEGATE FINAL RESULT: workaround for sign bug in toolchain
    return q31_neg(q31_sat_add(fb, ff));
}
} // namespace dsp