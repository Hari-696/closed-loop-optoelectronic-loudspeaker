/**
 * @file mfb_controller.cpp
 * @brief MFB controller with clean physical units implementation.
 * 
 * Control law in physical units:
 *   u_ff_linear = 9338.0 * x_ref        (V)   // inverse DC gain
 *   e = x_ref - x_meas                  (m)
 *   integ += e / 96000.0f (clamped)
 *   u_fb = Kp*e + Ki*integ - Kd*v_hat   (V)
 *   u_total = u_ff_linear + u_fb + u_ff_nl
 *   u_total_norm = clamp(u_total / AMP_FULL_SCALE_V, -1.0f, 1.0f)
 */

#include "dsp/mfb_controller.hpp"
#include "dsp/luenberger_observer.hpp"
#include <cmath>

namespace dsp {

MFBGainsQ31 g_mfb_gains{};
static float g_mfb_integral = 0.0f;  // Integral term in meters
static float g_mfb_x_prev = 0.0f;
static float g_mfb_v_filt = 0.0f;

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

void compute_state_feedback_gain(const FloatMat22& A_d, const FloatVec2& B_d,
                                 float z_pole_re, float z_pole_im,
                                 FloatVec2& K_out) noexcept {
    float tr_desired = 2.0f * z_pole_re;
    float det_desired = z_pole_re * z_pole_re + z_pole_im * z_pole_im;
    
    float a00 = A_d.m00, a01 = A_d.m01, a10 = A_d.m10, a11 = A_d.m11;
    float b0 = B_d.x, b1 = B_d.y;
    
    float tr_A = a00 + a11;
    float det_A = a00*a11 - a01*a10;
    
    float s = tr_A - tr_desired;
    
    float c1 = a01*b1 - a11*b0;
    float c2 = a10*b0 - a00*b1;
    
    float det_sys = b0*c2 - b1*c1;
    if (std::abs(det_sys) < 1e-12f) {
        K_out.x = 0.0f;
        K_out.y = 0.0f;
        return;
    }
    
    float rhs2 = det_desired - det_A;
    K_out.x = (s*c2 - b1*rhs2) / det_sys;
    K_out.y = (b0*rhs2 - s*c1) / det_sys;
}

float compute_reference_gain(const FloatMat22& A_d, const FloatVec2& B_d, 
                             const FloatVec2& C_d, const FloatVec2& K) noexcept {
    FloatMat22 A_cl;
    A_cl.m00 = A_d.m00 - B_d.x * K.x;
    A_cl.m01 = A_d.m01 - B_d.x * K.y;
    A_cl.m10 = A_d.m10 - B_d.y * K.x;
    A_cl.m11 = A_d.m11 - B_d.y * K.y;
    
    FloatMat22 I_minus_Acl;
    I_minus_Acl.m00 = 1.0f - A_cl.m00;
    I_minus_Acl.m01 = -A_cl.m01;
    I_minus_Acl.m10 = -A_cl.m10;
    I_minus_Acl.m11 = 1.0f - A_cl.m11;
    
    float det = I_minus_Acl.m00 * I_minus_Acl.m11 - I_minus_Acl.m01 * I_minus_Acl.m10;
    if (std::abs(det) < 1e-12f) return 1.0f;
    
    FloatMat22 inv;
    inv.m00 = I_minus_Acl.m11 / det;
    inv.m01 = -I_minus_Acl.m01 / det;
    inv.m10 = -I_minus_Acl.m10 / det;
    inv.m11 = I_minus_Acl.m00 / det;
    
    FloatVec2 temp;
    temp.x = inv.m00 * B_d.x + inv.m01 * B_d.y;
    temp.y = inv.m10 * B_d.x + inv.m11 * B_d.y;
    
    float dc_gain = C_d.x * temp.x + C_d.y * temp.y;
    
    if (std::abs(dc_gain) < 1e-12f) return 1.0f;
    return 1.0f / dc_gain;
}

bool mfb_controller_init(float cl_pole_hz, float damping) noexcept {
    using namespace fixed;
    
    if (cl_pole_hz < CL_POLE_MIN_HZ) cl_pole_hz = CL_POLE_MIN_HZ;
    if (cl_pole_hz > CL_POLE_MAX_HZ) cl_pole_hz = CL_POLE_MAX_HZ;
    if (damping < 0.1f) damping = 0.1f;
    if (damping > 1.0f) damping = 1.0f;
    
    FloatMat22 A_c{0.0f, 1.0f, -K_MS0/M_MS, -R_MS/M_MS};
    FloatVec2 B_c{0.0f, BL_0/M_MS};
    
    float dt = T_S;
    
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
        FloatMat22 A2{a*a+b*c, a*b+b*d, c*a+d*c, c*b+d*d};
        FloatMat22 A3{A2.m00*a+A2.m01*c, A2.m00*b+A2.m01*d, A2.m10*a+A2.m11*c, A2.m10*b+A2.m11*d};
        B_d.x = dt*(B_c.x + dt/2.0f*(a*B_c.x+b*B_c.y) + dt*dt/6.0f*(A2.m00*B_c.x+A2.m01*B_c.y) + dt*dt*dt/24.0f*(A3.m00*B_c.x+A3.m01*B_c.y));
        B_d.y = dt*(B_c.y + dt/2.0f*(c*B_c.x+d*B_c.y) + dt*dt/6.0f*(A2.m10*B_c.x+A2.m11*B_c.y) + dt*dt*dt/24.0f*(A3.m10*B_c.x+A3.m11*B_c.y));
    }
    
    float wn = 2.0f * PI * cl_pole_hz;
    float sigma = -damping * wn;
    float omega_d = wn * std::sqrt(1.0f - damping*damping);
    float z_re = std::exp(sigma * dt) * std::cos(omega_d * dt);
    float z_im = std::exp(sigma * dt) * std::sin(omega_d * dt);
    
    FloatVec2 K;
    compute_state_feedback_gain(A_d, B_d, z_re, z_im, K);
    
    constexpr float DC_GAIN_M_PER_V = BL_0 / (R_E_25C * K_MS0);
    constexpr float K_r = 1.0f / DC_GAIN_M_PER_V;
    
    constexpr float Kp = 60000.0f;
    constexpr float Ki = 1.5e7f;
    constexpr float Kd = 60.0f;
    constexpr float Kr = 9338.0f;
    
    auto to_q31 = [](float val, float in_scale, float out_scale) -> fixed::q31_t {
        return fixed::q31_from_float(val * in_scale / out_scale);
    };
    
    g_mfb_gains.k1 = to_q31(K.x, X_LIM, AMP_FULL_SCALE_V);
    g_mfb_gains.k2 = to_q31(K.y, 10.0f, AMP_FULL_SCALE_V);
    g_mfb_gains.k_r = to_q31(K_r, X_MAX, AMP_FULL_SCALE_V);
    
    g_mfb_gains.kp_phys = Kp;
    g_mfb_gains.ki_phys = Ki;
    g_mfb_gains.kd_phys = Kd;
    g_mfb_gains.kr_phys = Kr;
    
    g_mfb_integral = 0.0f;
    g_mfb_x_prev = 0.0f;
    g_mfb_v_filt = 0.0f;
    
    return true;
}

DSP_API fixed::q31_t mfb_controller_step(
    fixed::q31_t /*x_ref_q31*/,
    fixed::q31_t /*x_hat_q31*/,
    fixed::q31_t /*v_hat_q31*/,
    fixed::q31_t x_meas_q31,
    fixed::q31_t /*v_hat_vel_q31*/,
    fixed::q31_t x_ref_q31_ref) noexcept {
    using namespace fixed;
    
    constexpr float Kp  = 60000.0f;
    constexpr float Ki  = 1.5e7f;
    constexpr float Kd  = 60.0f;
    constexpr float ALPHA_V = 0.0378f;
    constexpr float INTEGRAL_LIMIT = 8.0e-6f;
    constexpr float K_r = 9338.0f;
    
    float x_ref_m  = q31_to_float(x_ref_q31_ref) * X_MAX;
    float ref_xmax = q31_to_float(x_ref_q31_ref);
    float x_meas_m = q31_to_float(x_meas_q31) * X_LIM;
    
    // Overdrive detection based on reference amplitude
    // If reference exceeds 1.0 X_MAX (12mm), apply adaptive gains
    bool overdrive = (ref_xmax > 1.0f) || (ref_xmax < -1.0f);
    
    // Adaptive gains for overdrive: reduce proportional gain, increase derivative
    float Kp_eff = Kp;
    float Kd_eff = Kd;
    if (overdrive) {
        Kp_eff = Kp * 0.3f;  // Reduce proportional gain for overdrive
        Kd_eff = Kd * 2.0f;  // Increase derivative gain for more damping
    }
    
    float v_raw = (x_meas_m - g_mfb_x_prev) * (1.0f / T_S);
    g_mfb_v_filt += ALPHA_V * (v_raw - g_mfb_v_filt);
    g_mfb_x_prev = x_meas_m;
    
    float e = x_ref_m - x_meas_m;
    float u_raw = K_r * x_ref_m + Kp_eff * e + Ki * g_mfb_integral - Kd_eff * g_mfb_v_filt;
    if (u_raw < AMP_FULL_SCALE_V && u_raw > -AMP_FULL_SCALE_V) {
        g_mfb_integral += e * T_S;
        if (g_mfb_integral >  INTEGRAL_LIMIT) g_mfb_integral =  INTEGRAL_LIMIT;
        if (g_mfb_integral < -INTEGRAL_LIMIT) g_mfb_integral = -INTEGRAL_LIMIT;
    }
    
    float u_total = K_r * x_ref_m + Kp_eff * e + Ki * g_mfb_integral - Kd_eff * g_mfb_v_filt;
    float u_norm = u_total / AMP_FULL_SCALE_V;
    if (u_norm >  1.0f) u_norm =  1.0f;
    if (u_norm < -1.0f) u_norm = -1.0f;
    return q31_from_float(u_norm);
}

void mfb_controller_reset() noexcept {
    g_mfb_integral = 0.0f;
    g_mfb_x_prev = 0.0f;
    g_mfb_v_filt = 0.0f;
}

} // namespace dsp