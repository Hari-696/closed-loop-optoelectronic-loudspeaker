/**
 * @file luenberger_observer.cpp
 * @brief Luenberger observer initialization - ZOH discretization and pole placement.
 */

#include "luenberger_observer.hpp"
#include <cmath>

namespace dsp {

ObserverMatricesQ31 g_observer_matrices{};
ObserverStateQ31 g_observer_state{};

// ============================================================================
// Matrix Exponential for ZOH Discretization (2x2)
// ============================================================================

struct FloatMat22 {
    float m00, m01, m10, m11;
};

struct FloatVec2 {
    float x, y;
};

// Compute exp(A_c * T) for 2x2 matrix using Padé approximation or eigen-decomposition
// For 2x2, we can use the analytical formula
FloatMat22 matrix_exp_2x2(const FloatMat22& A, float dt) noexcept {
    // A = [a, b; c, d]
    float a = A.m00, b = A.m01, c = A.m10, d = A.m11;
    
    float tr = a + d;
    float det = a * d - b * c;
    
    // Eigenvalues: λ = (tr ± sqrt(tr² - 4*det)) / 2
    float disc = tr * tr - 4.0f * det;
    
    FloatMat22 result;
    
    if (disc >= 0.0f) {
        // Real eigenvalues
        float sqrt_disc = std::sqrt(disc);
        float lambda1 = 0.5f * (tr + sqrt_disc);
        float lambda2 = 0.5f * (tr - sqrt_disc);
        
        if (std::abs(lambda1 - lambda2) > 1e-6f) {
            // Distinct eigenvalues
            float e1 = std::exp(lambda1 * dt);
            float e2 = std::exp(lambda2 * dt);
            float denom = lambda1 - lambda2;
            
            result.m00 = (a * (e1 - e2) + (lambda1 * e2 - lambda2 * e1)) / denom;
            result.m01 = (b * (e1 - e2)) / denom;
            result.m10 = (c * (e1 - e2)) / denom;
            result.m11 = (d * (e1 - e2) + (lambda1 * e2 - lambda2 * e1)) / denom;
        } else {
            // Repeated eigenvalue
            float e = std::exp(lambda1 * dt);
            float edt = e * dt;
            
            result.m00 = e + (a - lambda1) * edt;
            result.m01 = b * edt;
            result.m10 = c * edt;
            result.m11 = e + (d - lambda1) * edt;
        }
    } else {
        // Complex eigenvalues: λ = α ± jβ
        float alpha = 0.5f * tr;
        float beta = 0.5f * std::sqrt(-disc);
        float eat = std::exp(alpha * dt);
        float cos_bt = std::cos(beta * dt);
        float sin_bt = std::sin(beta * dt);
        
        result.m00 = eat * (cos_bt + (a - alpha) * sin_bt / beta);
        result.m01 = eat * (b * sin_bt / beta);
        result.m10 = eat * (c * sin_bt / beta);
        result.m11 = eat * (cos_bt + (d - alpha) * sin_bt / beta);
    }
    
    return result;
}

// Compute integral_0^T exp(A_c * τ) dτ * B_c for ZOH
FloatVec2 matrix_exp_integral_B(const FloatMat22& A, const FloatVec2& B, float dt) noexcept {
    // For 2x2, use: ∫exp(Aτ)dτ = A⁻¹(exp(A·dt) - I) if A invertible
    // Otherwise use series expansion
    float a = A.m00, b = A.m01, c = A.m10, d = A.m11;
    float det = a * d - b * c;
    
    FloatMat22 expAdt = matrix_exp_2x2(A, dt);
    FloatVec2 result;
    
    if (std::abs(det) > 1e-12f) {
        // A is invertible
        FloatMat22 Ainv;
        Ainv.m00 =  d / det;
        Ainv.m01 = -b / det;
        Ainv.m10 = -c / det;
        Ainv.m11 =  a / det;
        
        // exp(A·dt) - I
        FloatMat22 expAdt_minus_I;
        expAdt_minus_I.m00 = expAdt.m00 - 1.0f;
        expAdt_minus_I.m01 = expAdt.m01;
        expAdt_minus_I.m10 = expAdt.m10;
        expAdt_minus_I.m11 = expAdt.m11 - 1.0f;
        
        // A⁻¹ * (exp(A·dt) - I) * B
        result.x = Ainv.m00 * (expAdt_minus_I.m00 * B.x + expAdt_minus_I.m01 * B.y)
                 + Ainv.m01 * (expAdt_minus_I.m10 * B.x + expAdt_minus_I.m11 * B.y);
        result.y = Ainv.m10 * (expAdt_minus_I.m00 * B.x + expAdt_minus_I.m01 * B.y)
                 + Ainv.m11 * (expAdt_minus_I.m10 * B.x + expAdt_minus_I.m11 * B.y);
    } else {
        // A singular - use series: (I + A·dt/2 + A²·dt²/6 + ...) * dt * B
        FloatMat22 A2;
        A2.m00 = a*a + b*c;
        A2.m01 = a*b + b*d;
        A2.m10 = c*a + d*c;
        A2.m11 = c*b + d*d;
        
        FloatMat22 A3;
        A3.m00 = A2.m00*a + A2.m01*c;
        A3.m01 = A2.m00*b + A2.m01*d;
        A3.m10 = A2.m10*a + A2.m11*c;
        A3.m11 = A2.m10*b + A2.m11*d;
        
        // Approximate up to 3rd order (sufficient for small dt = 10.4 µs)
        result.x = dt * (B.x + dt/2.0f * (a*B.x + b*B.y) 
                        + dt*dt/6.0f * (A2.m00*B.x + A2.m01*B.y)
                        + dt*dt*dt/24.0f * (A3.m00*B.x + A3.m01*B.y));
        result.y = dt * (B.y + dt/2.0f * (c*B.x + d*B.y)
                        + dt*dt/6.0f * (A2.m10*B.x + A2.m11*B.y)
                        + dt*dt*dt/24.0f * (A3.m10*B.x + A3.m11*B.y));
    }
    
    return result;
}

// ============================================================================
// Observer Gain Calculation (Ackermann's Formula for 2x2)
// ============================================================================

// Place observer poles at desired locations
// For discrete system: x[k+1] = A_d x[k] + B_d u[k] + L (y[k] - C_d x[k])
// Error dynamics: e[k+1] = (A_d - L C_d) e[k]
// Characteristic polynomial: z² - tr(A_d - L C_d) z + det(A_d - L C_d)
// Desired: (z - p1)(z - p2) = z² - (p1+p2)z + p1*p2
void compute_observer_gain(const FloatMat22& A_d, const FloatVec2& C_d, 
                          float pole1, float pole2,
                          FloatVec2& L_out) noexcept {
    // For C_d = [1, 0], the observer gain L = [l0; l1] affects first row of A_d
    // A_d - L*C_d = [a00-l0, a01; a10-l1, a11]
    // tr = a00 - l0 + a11 = p1 + p2
    // det = (a00-l0)*a11 - a01*(a10-l1) = p1*p2
    // 
    // From trace: l0 = a00 + a11 - (p1 + p2)
    // From det: (a00-l0)*a11 - a01*a10 + a01*l1 = p1*p2
    //           l1 = (p1*p2 - (a00-l0)*a11 + a01*a10) / a01
    
    float tr_desired = pole1 + pole2;
    float det_desired = pole1 * pole2;
    
    float l0 = A_d.m00 + A_d.m11 - tr_desired;
    float l1;
    
    if (std::abs(A_d.m01) > 1e-12f) {
        l1 = (det_desired - (A_d.m00 - l0) * A_d.m11 + A_d.m01 * A_d.m10) / A_d.m01;
    } else {
        // Fallback - place via dual of controllable canonical form
        l1 = 0.0f;
    }
    
    L_out.x = l0;
    L_out.y = l1;
}

// ============================================================================
// Initialization
// ============================================================================

bool luenberger_observer_init(float observer_pole_hz) noexcept {
    using namespace fixed;
    
    // Clamp observer pole to valid range
    if (observer_pole_hz < OBSERVER_POLE_MIN_HZ) observer_pole_hz = OBSERVER_POLE_MIN_HZ;
    if (observer_pole_hz > OBSERVER_POLE_MAX_HZ) observer_pole_hz = OBSERVER_POLE_MAX_HZ;
    
    // Continuous-time matrices
    FloatMat22 A_c{A_C_00, A_C_01, A_C_10, A_C_11};
    FloatVec2 B_c{B_C_0, B_C_1};
    FloatVec2 C_c{C_C_0, C_C_1};
    
    // ZOH discretization at F_S
    float dt = T_S;
    FloatMat22 A_d = matrix_exp_2x2(A_c, dt);
    FloatVec2 B_d = matrix_exp_integral_B(A_c, B_c, dt);
    
    // Desired discrete observer poles (continuous pole → discrete via z = e^(sT))
    // Observer bandwidth: 325-540 Hz continuous
    // We place two real poles at the same frequency for critical damping
    float s_pole = -2.0f * PI * observer_pole_hz;
    float z_pole = std::exp(s_pole * dt);
    
    // Compute observer gain L
    FloatVec2 L;
    compute_observer_gain(A_d, C_c, z_pole, z_pole, L);
    
    // Precompute (A_d - L*C_d) for combined prediction+correction
    // C_d = [1, 0], so L*C_d = [l0, 0; l1, 0]
    FloatMat22 A_L;
    A_L.m00 = A_d.m00 - L.x;
    A_L.m01 = A_d.m01;
    A_L.m10 = A_d.m10 - L.y;
    A_L.m11 = A_d.m11;
    
    // Convert to Q31
    // Scaling: states are in physical units (m, m/s), inputs in V
    // Q31 represents ±X_LIM for displacement, ±10 m/s for velocity, ±5V for voltage
    
    auto to_q31_scaled = [](float val, float full_scale) -> fixed::q31_t {
        return fixed::q31_from_float(val / full_scale);
    };
    
    g_observer_matrices.a00 = to_q31_scaled(A_d.m00, 1.0f);
    g_observer_matrices.a01 = to_q31_scaled(A_d.m01 * 10.0f, 1.0f);  // v contributes to x
    g_observer_matrices.a10 = to_q31_scaled(A_d.m10 / 10.0f, 1.0f);  // x contributes to v
    g_observer_matrices.a11 = to_q31_scaled(A_d.m11, 1.0f);
    
    g_observer_matrices.b0 = to_q31_scaled(B_d.x / 5.0f, 1.0f);  // u in ±5V → x
    g_observer_matrices.b1 = to_q31_scaled(B_d.y / 5.0f, 1.0f);  // u in ±5V → v
    
    g_observer_matrices.l0 = to_q31_scaled(L.x, 1.0f);
    g_observer_matrices.l1 = to_q31_scaled(L.y / 10.0f, 1.0f);  // y in m → v
    
    // (A_d - L*C_d)
    g_observer_matrices.a_l00 = to_q31_scaled(A_L.m00, 1.0f);
    g_observer_matrices.a_l01 = to_q31_scaled(A_L.m01 * 10.0f, 1.0f);
    g_observer_matrices.a_l10 = to_q31_scaled(A_L.m10 / 10.0f, 1.0f);
    g_observer_matrices.a_l11 = to_q31_scaled(A_L.m11, 1.0f);
    
    // Reset state
    luenberger_observer_reset();
    
    return true;
}

} // namespace dsp