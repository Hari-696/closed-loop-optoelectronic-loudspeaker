/**
 * @file cubic_spline_lut.cpp
 * @brief Cubic spline LUT initialization - natural cubic spline computation.
 * 
 * Computes piecewise cubic polynomial coefficients for the inverse mapping
 * x_meas = S⁻¹(V_out) from calibration data points.
 * Uses natural spline boundary conditions (second derivative = 0 at endpoints).
 */

#include "cubic_spline_lut.hpp"
#include <algorithm>
#include <cmath>

namespace dsp {

// Global LUT instance
SplineLUT g_spline_lut{};

// ============================================================================
// Natural Cubic Spline Solver (tridiagonal system)
// ============================================================================

struct SplineWorkspace {
    std::vector<float> h;       // Interval widths
    std::vector<float> alpha;   // RHS of tridiagonal system
    std::vector<float> c;       // Second derivatives (spline coefficients)
    std::vector<float> l, u, z; // Tridiagonal solver workspace
    std::vector<float> b, d;    // Remaining polynomial coefficients
    
    SplineWorkspace(size_t n) : h(n), alpha(n), c(n), l(n), u(n), z(n), b(n), d(n) {}
};

bool cubic_spline_lut_init(const float* voltage_samples, const float* displacement_samples, size_t num_points) noexcept {
    if (!voltage_samples || !displacement_samples || num_points < 4) {
        return false;
    }
    
    // Verify monotonic increasing voltage
    for (size_t i = 1; i < num_points; ++i) {
        if (voltage_samples[i] <= voltage_samples[i-1]) {
            return false;
        }
    }
    
    const size_t n = num_points - 1;  // Number of intervals
    SplineWorkspace w(n + 1);
    
    // Step 1: Compute interval widths h[i] = x[i+1] - x[i]
    for (size_t i = 0; i < n; ++i) {
        w.h[i] = voltage_samples[i+1] - voltage_samples[i];
        if (w.h[i] <= 0.0f) return false;
    }
    
    // Step 2: Compute alpha[i] for natural spline (3 * ((y[i+1]-y[i])/h[i] - (y[i]-y[i-1])/h[i-1]))
    w.alpha[0] = 0.0f;  // Natural boundary: c[0] = 0
    w.alpha[n] = 0.0f;  // Natural boundary: c[n] = 0
    for (size_t i = 1; i < n; ++i) {
        float dy1 = (displacement_samples[i+1] - displacement_samples[i]) / w.h[i];
        float dy0 = (displacement_samples[i] - displacement_samples[i-1]) / w.h[i-1];
        w.alpha[i] = 3.0f * (dy1 - dy0);
    }
    
    // Step 3: Solve tridiagonal system for c (second derivatives)
    // Matrix is:
    // [ 1                        0     ... ] [c0]   [0]
    // [ h0    2(h0+h1)    h1   0     ... ] [c1]   [alpha1]
    // [ 0      h1    2(h1+h2)  h2   ... ] [c2] = [alpha2]
    // [ ...                      ...     1] [cn]   [0]
    
    w.l[0] = 1.0f;
    w.u[0] = 0.0f;
    w.z[0] = 0.0f;
    
    for (size_t i = 1; i < n; ++i) {
        w.l[i] = 2.0f * (w.h[i-1] + w.h[i]) - w.h[i-1] * w.u[i-1];
        if (std::abs(w.l[i]) < 1e-12f) return false;
        w.u[i] = w.h[i] / w.l[i];
        w.z[i] = (w.alpha[i] - w.h[i-1] * w.z[i-1]) / w.l[i];
    }
    
    w.l[n] = 1.0f;
    w.z[n] = 0.0f;
    w.c[n] = 0.0f;
    
    // Back substitution
    for (int i = static_cast<int>(n) - 1; i >= 0; --i) {
        w.c[i] = w.z[i] - w.u[i] * w.c[i+1];
    }
    
    // Step 4: Compute b and d coefficients for each interval
    // Polynomial on [x_i, x_{i+1}]: a_i + b_i*(x-x_i) + c_i*(x-x_i)^2 + d_i*(x-x_i)^3
    // where:
    // a_i = y_i
    // b_i = (y_{i+1}-y_i)/h_i - h_i*(2*c_i + c_{i+1})/3
    // c_i = c_i (already computed)
    // d_i = (c_{i+1} - c_i) / (3*h_i)
    for (size_t i = 0; i < n; ++i) {
        w.d[i] = (w.c[i+1] - w.c[i]) / (3.0f * w.h[i]);
        float dy = (displacement_samples[i+1] - displacement_samples[i]) / w.h[i];
        w.b[i] = dy - w.h[i] * (2.0f * w.c[i] + w.c[i+1]) / 3.0f;
    }
    
    // Step 5: Resample spline onto uniform grid for LUT
    // We need SPLINE_NUM_SEGMENTS uniform segments covering [V_min, V_max]
    // Each segment stores coefficients for cubic in dx (offset within segment)
    
    const float v_min = SPLINE_INPUT_MIN_V;
    const float v_max = SPLINE_INPUT_MAX_V;
    const float seg_width = SPLINE_SEGMENT_WIDTH;
    
    // For each LUT segment, find which calibration interval it falls in
    // and evaluate the spline polynomial at segment boundaries to get
    // the cubic coefficients in the local dx coordinate
    
    size_t cal_idx = 0;  // Current calibration interval
    
    for (size_t seg = 0; seg < SPLINE_NUM_SEGMENTS; ++seg) {
        float seg_start = v_min + static_cast<float>(seg) * seg_width;
        float seg_end = seg_start + seg_width;
        float seg_mid = seg_start + 0.5f * seg_width;
        
        // Find calibration interval containing segment midpoint
        while (cal_idx < n && voltage_samples[cal_idx + 1] < seg_mid) {
            ++cal_idx;
        }
        if (cal_idx >= n) cal_idx = n - 1;
        
        // We'll fit a cubic to the spline over this segment by sampling
        // at 4 points and solving for coefficients. This avoids complex
        // coordinate transformation of the spline polynomials.
        
        // Sample at dx = 0, 1/3, 2/3, 1 of segment width
        float dx_samples[4] = {0.0f, seg_width/3.0f, 2.0f*seg_width/3.0f, seg_width};
        float y_samples[4];
        
        for (int k = 0; k < 4; ++k) {
            float v = seg_start + dx_samples[k];
            
            // Find correct calibration interval for this voltage
            size_t idx = cal_idx;
            while (idx < n && voltage_samples[idx + 1] < v) ++idx;
            while (idx > 0 && voltage_samples[idx] > v) --idx;
            if (idx >= n) idx = n - 1;
            
            // Evaluate spline polynomial in this interval
            float dx = v - voltage_samples[idx];
            float a = displacement_samples[idx];
            float b = w.b[idx];
            float c = w.c[idx];
            float d = w.d[idx];
            
            // Horner: ((d * dx + c) * dx + b) * dx + a
            y_samples[k] = ((d * dx + c) * dx + b) * dx + a;
        }
        
        // Fit cubic to 4 points: y = A0 + A1*dx + A2*dx^2 + A3*dx^3
        // Using Lagrange interpolation or direct solve
        // Points at dx = 0, h/3, 2h/3, h where h = seg_width
        float h = seg_width;
        float h2 = h * h;
        float h3 = h2 * h;
        
        float y0 = y_samples[0];
        float y1 = y_samples[1];
        float y2 = y_samples[2];
        float y3 = y_samples[3];
        
        // Coefficients for cubic through (0,y0), (h/3,y1), (2h/3,y2), (h,y3)
        // Derived from Vandermonde inverse
        g_spline_lut.segments[seg].a0 = fixed::q31_from_float(y0);
        g_spline_lut.segments[seg].a1 = fixed::q31_from_float(
            (-11.0f*y0 + 18.0f*y1 - 9.0f*y2 + 2.0f*y3) / h
        );
        g_spline_lut.segments[seg].a2 = fixed::q31_from_float(
            (18.0f*y0 - 45.0f*y1 + 36.0f*y2 - 9.0f*y3) / h2
        );
        g_spline_lut.segments[seg].a3 = fixed::q31_from_float(
            (-9.0f*y0 + 27.0f*y1 - 27.0f*y2 + 9.0f*y3) / h3
        );
    }
    
    // Store metadata in Q31
    g_spline_lut.input_min_q31 = fixed::q31_from_float(SPLINE_INPUT_MIN_V);
    g_spline_lut.input_max_q31 = fixed::q31_from_float(SPLINE_INPUT_MAX_V);
    g_spline_lut.inv_segment_width_q31 = fixed::q31_from_float(SPLINE_INV_SEGMENT_WIDTH);
    g_spline_lut.initialized = true;
    
    return true;
}

/**
 * @brief Alternative initialization from inverse-square model directly.
 * 
 * Generates calibration points from the physical model:
 * P_rx ∝ 1/(d₀ - x)², I_ph = ℜ·P_rx, V_out = -I_ph·R_f
 * Inverts to get x(V_out) for the LUT.
 */
bool cubic_spline_lut_init_from_model(float d0, float r_f, float responsivity, float p_tx_optical) noexcept {
    constexpr size_t NUM_CAL_POINTS = 513;  // Dense sampling for accurate spline
    static float voltages[NUM_CAL_POINTS];
    static float displacements[NUM_CAL_POINTS];
    
    // Generate points spanning -X_LIM to +X_LIM
    for (size_t i = 0; i < NUM_CAL_POINTS; ++i) {
        float x = -X_LIM + (2.0f * X_LIM * static_cast<float>(i)) / static_cast<float>(NUM_CAL_POINTS - 1);
        float d = d0 - x;
        float p_rx = p_tx_optical * responsivity * (1.0f / (4.0f * PI * d * d));  // Simplified
        float i_ph = responsivity * p_rx;
        float v_out = -i_ph * r_f;
        
        voltages[i] = v_out;
        displacements[i] = x;
    }
    
    // Sort by voltage (should already be monotonic for inverse-square)
    // But ensure it's strictly increasing
    for (size_t i = 1; i < NUM_CAL_POINTS; ++i) {
        if (voltages[i] <= voltages[i-1]) {
            // Non-monotonic - model parameters may be wrong
            return false;
        }
    }
    
    return cubic_spline_lut_init(voltages, displacements, NUM_CAL_POINTS);
}

} // namespace dsp