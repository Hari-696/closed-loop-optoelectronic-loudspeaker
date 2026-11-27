import ctypes
import numpy as np

dll = ctypes.CDLL(r"C:\Users\ASUS\Downloads\Opencode\build\libdsp_core.dll")

dll.dsp_init_cabi.restype = ctypes.c_int
dll.dsp_reset_cabi.restype = None
dll.dsp_set_reference_cabi.argtypes = [ctypes.c_float]
dll.dsp_set_reference_cabi.restype = None

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

dll.dsp_step_cabi.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
dll.dsp_step_cabi.restype = ctypes.c_int

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
    code = int(v / 5.0 * 8388608)
    return max(-8388608, min(8388607, code))

def dac_code_to_voltage(code):
    return code / 8388608.0 * 5.0

dll.dsp_init_cabi()
dll.dsp_set_reference_cabi(1.2)  # 1.2mm reference

x_phys = 0.0
v_phys = 0.0
dac_code = 0

print("Sample | x_ref | x_phys | v_phys | DAC_V")
for i in range(5000):
    v_optical = V_REST * (D_0 / (D_0 - x_phys))**2
    adc_disp = int(v_optical / 5.0 * 8388608)
    adc_disp = max(-8388608, min(8388607, adc_disp))
    
    v_amp = dac_code / 8388608.0 * 5.0
    i_coil = (v_amp - BL * v_phys) / R_E
    v_current = i_coil * 5.0
    adc_curr = int(v_current / 5.0 * 8388608)
    adc_curr = max(-8388608, min(8388607, adc_curr))
    
    v_voltage = v_amp / 10.0
    adc_volt = int(v_voltage / 5.0 * 8388608)
    adc_volt = max(-8388608, min(8388607, adc_volt))
    
    class AdcFrame(ctypes.Structure):
        _fields_ = [("displacement_adc", ctypes.c_int32),
                    ("current_adc", ctypes.c_int32),
                    ("voltage_adc", ctypes.c_int32),
                    ("timestamp_ns", ctypes.c_int64)]
    
    class DacFrame(ctypes.Structure):
        _fields_ = [("dac_code", ctypes.c_int32), ("timestamp_ns", ctypes.c_int64)]
    
    adc = type('AdcFrame', (), {})()
    adc.displacement_adc = int(v_optical / 5.0 * 8388608)
    adc.current_adc = 0
    adc.voltage_adc = 0
    adc.timestamp_ns = 0
    
    dac = type('DacFrame', (), {})()
    dac.dac_code = 0
    dac.timestamp_ns = 0
    
    # Use ctypes structures properly
    class AdcFrameC(ctypes.Structure):
        _fields_ = [("displacement_adc", ctypes.c_int32),
                    ("current_adc", ctypes.c_int32),
                    ("voltage_adc", ctypes.c_int32),
                    ("timestamp_ns", ctypes.c_int64)]
    class DacFrameC(ctypes.Structure):
        _fields_ = [("dac_code", ctypes.c_int32), ("timestamp_ns", ctypes.c_int64)]
    
    adc = ctypes.c_int32(int(v_optical / 5.0 * 8388608))
    adc = max(-8388608, min(8388607, int(v_optical / 5.0 * 8388608)))
    
    adc_frame = type('AdcFrame', (ctypes.Structure,), {'_fields_': [('displacement_adc', ctypes.c_int32), ('current_adc', ctypes.c_int32), ('voltage_adc', ctypes.c_int32), ('timestamp_ns', ctypes.c_int64)]})()
    adc_frame.displacement_adc = int(v_optical / 5.0 * 8388608)
    adc_frame.current_adc = 0
    adc_frame.voltage_adc = 0
    adc_frame.timestamp_ns = 0
    
    dac_frame = type('DacFrame', (ctypes.Structure,), {'_fields_': [('dac_code', ctypes.c_int32), ('timestamp_ns', ctypes.c_int64)]})()
    
    # Simpler approach
    adc_disp = int(v_optical / 5.0 * 8388608)
    adc_disp = max(-8388608, min(8388607, adc_disp))
    
    class AdcFrame(ctypes.Structure):
        _fields_ = [("displacement_adc", ctypes.c_int32), ("current_adc", ctypes.c_int32), ("voltage_adc", ctypes.c_int32), ("timestamp_ns", ctypes.c_int64)]
    class DacFrame(ctypes.Structure):
        _fields_ = [("dac_code", ctypes.c_int32), ("timestamp_ns", ctypes.c_int64)]
    
    adc = AdcFrame()
    adc.displacement_adc = adc_disp
    adc.current_adc = 0
    adc.voltage_adc = 0
    adc.timestamp_ns = 0
    
    dac = DacFrame()
    dac.dac_code = 0
    dac.timestamp_ns = 0
    
    dll.dsp_step_cabi(ctypes.byref(adc), ctypes.byref(dac))
    
    dac_code = dac.dac_code
    v_amp = dac_code_to_voltage(dac_code)
    
    x_new = A_d[0,0] * x_phys + A_d[0,1] * v_phys + B_d[0] * v_amp
    v_new = A_d[1,0] * x_phys + A_d[1,1] * v_phys + B_d[1] * v_amp
    x_phys, v_phys = x_new, v_new
    
    if i % 1000 == 0:
        print(f"Sample {i}: x_ref=1.2mm, x={x_phys*1000:.3f}mm, v={v_phys:.3f}m/s, DAC={dac_code_to_voltage(dac_code):.3f}V")

print(f"\nFinal: x={x_phys*1000:.3f}mm, error={(1.2 - x_phys*1000):.3f}mm")