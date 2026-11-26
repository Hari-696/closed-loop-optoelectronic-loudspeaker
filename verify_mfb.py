import numpy as np
import math

# Constants
F_S = 96000.0
T_S = 1.0 / F_S
PI = math.pi

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

# Discrete matrices
dt = T_S
eigvals_A, eigvecs_A = np.linalg.eig(A_c)
exp_eigvals = np.exp(eigvals_A * dt)
A_d = eigvecs_A @ np.diag(exp_eigvals) @ np.linalg.inv(eigvecs_A)
A_inv = np.linalg.inv(A_c)
B_d = A_inv @ (A_d - np.eye(2)) @ B_c

a00, a01 = A_d[0,0].real, A_d[0,1].real
a10, a11 = A_d[1,0].real, A_d[1,1].real
b0, b1 = B_d[0].real, B_d[1].real

print(f"a00={a00:.10f}, a01={a01:.10f}")
print(f"a10={a10:.10f}, a11={a11:.10f}")
print(f"b0={b0:.10e}, b1={b1:.10e}")

# MFB desired poles
CL_POLE_HZ = 75.0
CL_DAMPING = 0.707
wn = 2.0 * PI * CL_POLE_HZ
sigma = -CL_DAMPING * wn
omega_d = wn * math.sqrt(1.0 - CL_DAMPING*CL_DAMPING)
z_re = math.exp(sigma * dt) * math.cos(omega_d * dt)
z_im = math.exp(sigma * dt) * math.sin(omega_d * dt)

print(f"\nDesired discrete poles: {z_re:.8f} ± j{z_im:.8f}")
print(f"Pole magnitude: {math.sqrt(z_re*z_re + z_im*z_im):.8f}")

tr_desired = 2.0 * z_re
det_desired = z_re*z_re + z_im*z_im
tr_A = a00 + a11
det_A = a00*a11 - a01*a10

print(f"\ntr_A={tr_A:.8f}, det_A={det_A:.8f}")
print(f"tr_desired={tr_desired:.8f}, det_desired={det_desired:.8f}")

s = tr_A - tr_desired
c1 = a01*b1 - a11*b0
c2 = a10*b0 - a00*b1
rhs2 = det_desired - det_A
det_sys = b0*c2 - b1*c1

print(f"\ns={s:.8e}")
print(f"c1={c1:.8e}, c2={c2:.8e}")
print(f"rhs2={rhs2:.8e}")
print(f"det_sys={det_sys:.8e}")

K1 = (s*c2 - b1*rhs2) / det_sys
K2 = (b0*rhs2 - s*c1) / det_sys

print(f"\nK1={K1:.8f}, K2={K2:.8f}")

# Check closed-loop eigenvalues
A_cl = A_d - np.outer(B_d, [K1, K2])
eig_cl = np.linalg.eigvals(A_cl)
print(f"\nClosed-loop eigenvalues: {eig_cl}")
print(f"Magnitudes: {np.abs(eig_cl)}")

# Check the bound: -K_ms0 * R_e / Bl_0
bound = -K_MS0 * R_E / BL
print(f"\nBound -K_ms0*R_e/Bl_0 = {bound:.2f}")
print(f"K1 = {K1:.2f} (should be > {bound:.2f})")

# Now try with 115 Hz poles (above mechanical resonance)
print(f"\n{'='*60}")
print(f"TRYING 115 Hz CLOSED-LOOP POLES")
print(f"{'='*60}")

CL_POLE_HZ = 115.0
wn = 2.0 * PI * CL_POLE_HZ
sigma = -CL_DAMPING * wn
omega_d = wn * math.sqrt(1.0 - CL_DAMPING*CL_DAMPING)
z_re = math.exp(sigma * dt) * math.cos(omega_d * dt)
z_im = math.exp(sigma * dt) * math.sin(omega_d * dt)

print(f"Desired discrete poles: {z_re:.8f} ± j{z_im:.8f}")
print(f"Pole magnitude: {math.sqrt(z_re*z_re + z_im*z_im):.8f}")

tr_desired = 2.0 * z_re
det_desired = z_re*z_re + z_im*z_im

s = tr_A - tr_desired
rhs2 = det_desired - det_A

K1 = (s*c2 - b1*rhs2) / det_sys
K2 = (b0*rhs2 - s*c1) / det_sys

print(f"\nK1={K1:.8f}, K2={K2:.8f}")

A_cl = A_d - np.outer(B_d, [K1, K2])
eig_cl = np.linalg.eigvals(A_cl)
print(f"Closed-loop eigenvalues: {eig_cl}")
print(f"Magnitudes: {np.abs(eig_cl)}")
print(f"K1 > bound? {K1 > bound}")