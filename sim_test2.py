#!/usr/bin/env python3
"""
Closed-loop simulation test with internal signal monitoring.
"""
import ctypes
import numpy as np
import math

dll_path = r"C:\Users\ASUS\Downloads\Opencode\build\libdsp_core.dll"
dsp = ctypes.CDLL(dll_path)

dsp.dsp_init_cabi.restype = ctypes.c_int
dsp.dsp_reset_cabi.restype = None
dsp.dsp_set_reference_cabi.argtypes = [ctypes.c_float]
dsp.dsp_set_reference_cabi.restype = None
dsp.dsp_step_cabi.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
dsp.dsp_step_cabi.restype = ctypes.c_int
dsp.dsp_debug_get_ref_cabi.argtypes = [ctypes.POINTER(ctypes.c_int32), ctypes.POINTER(ctypes.c_int32)]
dsp.dsp_debug_get_ref_cabi.restype = ctypes.c_int

class AdcFrame(ctypes.Structure):
    _fields_ = [
        ("displacement_adc", ctypes.c_int32),
        ("current_adc", ctypes.c_int32),
        ("voltage_adc", ctypes.c_int32),
        ("timestamp_ns", ctypes.c_int64),
    ]

class DacFrame(ctypes.Structure):
    _fields_ = [
        ("dac_code", ctypes.c_int32),
        ("timestamp_ns", ctypes.c_int64),
    ]

F_S = 96000.0
T_S = 1.0 / F_S
X_LIM = 0.015
X_MAX = 0.012
AMP_FULL_SCALE_V = 150.0
ADC_VREF = 5.0
ADC_FULL_SCALE = 8388608.0

M_MS = 0.042
R_MS = 1.8
K_MS0 = 1.95e4
BL = 14.2
R_E = 6.8

A_c = np.array([[0.0, 1.0], [-K_MS0/M_MS, -R_MS/M_MS]])
B_c = np.array([0.0, BL/M_MS])

dt = T_S
eigvals_A, eigvecs_A = np.linalg.eig(A_c)
exp_eigvals = np.exp(eigvals_A * dt)
A_d = eigvecs_A @ np.diag(exp_eigvals) @ np.linalg.inv(eigvecs_A)
A_inv = np.linalg.inv(A_c)
B_d = A_inv @ (A_d - np.eye(2)) @ B_c
A_d = A_d.real
B_d = B_d.real

D_0 = 0.018
I_PHOTODIODE_REST = 2.7e-6
R_F = 49900.0
V_REST = -I_PHOTODIODE_REST * R_F

def displacement_to_voltage(x):
    return V_REST * (D_0 / (D_0 - x))**2

def voltage_to_adc_code(v):
    code = int(v / ADC_VREF * ADC_FULL_SCALE)
    return max(-8388608, min(8388607, code))

def dac_code_to_voltage(code):
    return code / ADC_FULL_SCALE * ADC_VREF

def q31_to_float(q31):
    return q31 / 2147483648.0

print("Initializing DSP...")
dsp.dsp_init_cabi()

# Set reference
x_ref_mm = 1.2
dsp.dsp_set_reference_cabi(x_ref_mm)

# Check reference
x_ref_q31 = ctypes.c_int32()
x_ref_mm_q = ctypes.c_int32()
dsp.dsp_debug_get_ref_cabi(ctypes.byref(x_ref_q31), ctypes.byref(x_ref_mm_q))
print(f"Reference: x_ref_q31={x_ref_q31.value} ({q31_to_float(x_ref_q31.value):.6f}), x_ref_mm={x_ref_mm_q.value/1e6:.6f}mm")

# Simulation state
x_phys = 0.0
v_phys = 0.0
dac_code = 0

print(f"\n{'Sample':>6} {'x_ref':>7} {'x_phys':>8} {'v_phys':>8} {'DAC_V':>7} {'v_amp':>7}")

for i in range(30):
    # ADC from plant
    v_optical = displacement_to_voltage(x_phys)
    adc_disp = voltage_to_adc_code(v_optical)
    
    v_amp = dac_code_to_voltage(dac_code)
    i_coil = (v_amp - BL * v_phys) / R_E
    v_current = i_coil * 5.0
    adc_curr = voltage_to_adc_code(v_current)
    
    v_voltage = v_amp / 10.0
    adc_volt = voltage_to_adc_code(v_voltage)
    
    adc = AdcFrame()
    adc.displacement_adc = adc_disp
    adc.current_adc = adc_curr
    adc.voltage_adc = adc_volt
    adc.timestamp_ns = int(i * T_S * 1e9)
    
    dac = DacFrame()
    dsp.dsp_step_cabi(ctypes.byref(adc), ctypes.byref(dac))
    
    dac_code = dac.dac_code
    v_amp = dac_code_to_voltage(dac_code)
    
    # Plant update
    x_new = A_d[0,0] * x_phys + A_d[0,1] * v_phys + B_d[0] * v_amp
    v_new = A_d[1,0] * x_phys + A_d[1,1] * v_phys + B_d[1] * v_amp
    x_phys, v_phys = x_new, v_new
    
    if i < 10 or i >= 25:
        print(f"{i:6d} {x_ref_mm:7.2f} {x_phys*1000:8.3f} {v_phys:8.3f} {dac_code_to_voltage(dac_code):7.3f} {v_amp:7.3f}")

print(f"\nFinal: x={x_phys*1000:.3f}mm, error={(x_ref_mm - x_phys*1000):.3f}mm")