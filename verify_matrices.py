import numpy as np
import math

# Constants from config.hpp
F_S = 96000.0
T_S = 1.0 / F_S
PI = math.pi

# Physical parameters
R_E = 6.8
BL = 14.2
M_MS = 0.042
R_MS = 1.8
K_MS0 = 1.95e4
X_MAX = 0.012
X_LIM = 0.015
AMP_FULL_SCALE_V = 150.0

# Continuous-time matrices
A_c = np.array([[0.0, 1.0],
                [-K_MS0/M_MS, -R_MS/M_MS]])
B_c = np.array([0.0, BL/M_MS])
C_c = np.array([1.0, 0.0])

print("=== Continuous-time matrices ===")
print(f"A_c = \n{A_c}")
print(f"B_c = {B_c}")
print(f"C_c = {C_c}")

# ZOH discretization
dt = T_S
# Matrix exponential using eigen-decomposition
eigvals_A, eigvecs_A = np.linalg.eig(A_c)
exp_eigvals = np.exp(eigvals_A * dt)
A_d = eigvecs_A @ np.diag(exp_eigvals) @ np.linalg.inv(eigvecs_A)

# B_d = ∫₀ᵀ exp(Aτ) dτ B_c = A⁻¹ (exp(A·T) - I) B_c
if abs(np.linalg.det(A_c)) > 1e-12:
    A_inv = np.linalg.inv(A_c)
    B_d = A_inv @ (A_d - np.eye(2)) @ B_c
else:
    # Series approximation
    B_d = dt * (np.eye(2) + A_c*dt/2 + A_c@A_c*dt*dt/6) @ B_c

print(f"\n=== Discrete-time matrices (ZOH) ===")
print(f"A_d = \n{A_d}")
print(f"B_d = {B_d}")
print(f"A_d eigenvalues: {np.linalg.eigvals(A_d)}")

# Observer poles
OBSERVER_POLE_HZ = 432.0  # 4x mechanical resonance (108 Hz)
s_pole = -2.0 * PI * OBSERVER_POLE_HZ
z_pole = math.exp(s_pole * dt)
print(f"\n=== Observer ===")
print(f"Continuous observer pole: {s_pole:.4f} rad/s ({OBSERVER_POLE_HZ} Hz)")
print(f"Discrete observer pole: z = {z_pole:.6f}")

# Observer gain L using Ackermann's formula (dual)
# For C = [1, 0], L = [l0; l1] 
# A_d - L*C = [a00-l0, a01; a10-l1, a11]
# tr = a00 - l0 + a11 = 2*z_pole
# det = (a00-l0)*a11 - a01*(a10-l1) = z_pole^2

a00, a01 = A_d[0,0], A_d[0,1]
a10, a11 = A_d[1,0], A_d[1,1]

tr_desired = 2.0 * z_pole
det_desired = z_pole * z_pole

l0 = a00 + a11 - tr_desired
l1 = (det_desired - (a00 - l0) * a11 + a01 * a10) / a01

L = np.array([l0, l1])
print(f"L = {L}")

# Error dynamics matrix
A_L = A_d - np.outer(L, C_c)
print(f"A_d - L*C_d = \n{A_L}")
eig_A_L = np.linalg.eigvals(A_L)
print(f"Eigenvalues of A_d - L*C_d: {eig_A_L}")
print(f"Magnitudes: {np.abs(eig_A_L)}")

# MFB Controller
CL_POLE_HZ = 75.0
CL_DAMPING = 0.707

wn = 2.0 * PI * CL_POLE_HZ
sigma = -CL_DAMPING * wn
omega_d = wn * math.sqrt(1.0 - CL_DAMPING*CL_DAMPING)
z_re = math.exp(sigma * dt) * math.cos(omega_d * dt)
z_im = math.exp(sigma * dt) * math.sin(omega_d * dt)
print(f"\n=== MFB Controller ===")
print(f"Closed-loop pole freq: {CL_POLE_HZ} Hz, damping: {CL_DAMPING}")
print(f"Discrete poles: z = {z_re:.6f} ± j{z_im:.6f}")
print(f"Pole magnitude: {math.sqrt(z_re*z_re + z_im*z_im):.6f}")

# Compute K using the same formula as in mfb_controller.cpp
tr_desired_cl = 2.0 * z_re
det_desired_cl = z_re*z_re + z_im*z_im

tr_A = a00 + a11
det_A = a00*a11 - a01*a10
b0, b1 = B_d[0], B_d[1]

s = tr_A - tr_desired_cl
c1 = a01*b1 - a11*b0
c2 = a10*b0 - a00*b1
rhs2 = det_desired_cl - det_A

det_sys = b0*c2 - b1*c1
K1 = (s*c2 - b1*rhs2) / det_sys
K2 = (b0*rhs2 - s*c1) / det_sys
K = np.array([K1, K2])
print(f"K = {K}")

# Closed-loop matrix
A_cl = A_d - np.outer(B_d, K)
print(f"A_d - B_d*K = \n{A_cl}")
eig_A_cl = np.linalg.eigvals(A_cl)
print(f"Eigenvalues: {eig_A_cl}")
print(f"Magnitudes: {np.abs(eig_A_cl)}")

# DC gain and K_r
DC_GAIN_M_PER_V = BL / (R_E * K_MS0)
K_r = 1.0 / DC_GAIN_M_PER_V
print(f"\nDC gain (continuous): {DC_GAIN_M_PER_V:.6f} m/V")
print(f"K_r: {K_r:.2f} V/m")

# Now check Q31 scaling consistency
print(f"\n=== Q31 Scaling Verification ===")
print(f"x_q31: 1.0 = X_LIM = {X_LIM} m")
print(f"v_q31: 1.0 = 10 m/s")
print(f"u_q31: 1.0 = AMP = {AMP_FULL_SCALE_V} V")
print(f"x_ref_q31: 1.0 = X_MAX = {X_MAX} m")

# Check observer scaling
X_LIM_OVER_VEL = X_LIM / 10.0
VEL_OVER_X_LIM = 10.0 / X_LIM
AMP_OVER_X_LIM = AMP_FULL_SCALE_V / X_LIM
AMP_OVER_VEL = AMP_FULL_SCALE_V / 10.0

print(f"\nObserver scaling factors:")
print(f"X_LIM/10 = {X_LIM_OVER_VEL:.6f}")
print(f"10/X_LIM = {VEL_OVER_X_LIM:.6f}")
print(f"AMP/X_LIM = {AMP_OVER_X_LIM:.2f}")
print(f"AMP/10 = {AMP_OVER_VEL:.2f}")

# Check MFB scaling
print(f"\nMFB scaling:")
print(f"K1 scaled by X_LIM/AMP = {K1 * X_LIM / AMP_FULL_SCALE_V:.6f}")
print(f"K2 scaled by 10/AMP = {K2 * 10.0 / AMP_FULL_SCALE_V:.6f}")
print(f"K_r scaled by X_MAX/AMP = {K_r * X_MAX / AMP_FULL_SCALE_V:.6f}")

# Simulate 20 samples to check for instability
print(f"\n=== 20-sample simulation (x_ref = 1.2mm) ===")
x_ref_mm = 1.2
x_ref = x_ref_mm / 1000.0  # meters
x_ref_q31 = x_ref / X_MAX

# Initialize state
x_hat = 0.0
v_hat = 0.0
u_lim_prev = 0.0
x_meas = 0.0

# Observer uses scaled matrices
# State in physical units for simulation
x_phys = 0.0
v_phys = 0.0

for i in range(20):
    # Reference step
    u_ff = 0.0  # Volterra starts at zero
    u_fb = -K1 * (x_hat / X_LIM) - K2 * (v_hat / 10.0) + K_r * (x_ref / X_MAX)
    u_total = u_ff + u_fb
    
    # Limiter (simplified - just check if it would saturate)
    if abs(u_total) > AMP_FULL_SCALE_V:
        u_lim = AMP_FULL_SCALE_V if u_total > 0 else -AMP_FULL_SCALE_V
    else:
        u_lim = u_total
    
    # Plant: x[k+1] = A_d x[k] + B_d u[k]
    x_phys_new = a00 * x_phys + a01 * v_phys + b0 * u_lim
    v_phys_new = a10 * x_phys + a11 * v_phys + b1 * u_lim
    
    # Measurement (assume perfect for now)
    x_meas = x_phys_new
    
    # Observer update (physical units)
    x_hat_new = a00 * x_hat + a01 * v_hat + b0 * u_lim_prev + l0 * (x_meas - x_hat)
    v_hat_new = a10 * x_hat + a11 * v_hat + b1 * u_lim_prev + l1 * (x_meas - x_hat)
    
    print(f"Sample {i:2d}: x_ref={x_ref_mm:.1f}mm, x_meas={x_phys_new*1000:7.3f}mm, "
          f"x_hat={x_hat_new*1000:7.3f}mm, v_hat={v_hat_new:7.3f}m/s, "
          f"u_fb={u_fb:7.2f}V, u_lim={u_lim:7.2f}V")
    
    x_phys, v_phys = x_phys_new, v_phys_new
    x_hat, v_hat = x_hat_new, v_hat_new
    u_lim_prev = u_lim