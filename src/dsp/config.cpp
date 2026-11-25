/**
 * @file config.cpp
 * @brief Configuration constants - ensures symbols are emitted for linking.
 */

#include "config.hpp"

namespace dsp {

// Force instantiation of constexpr variables for linking
const float F_S;
const float T_S;
const uint32_t ISR_BUDGET_US;
const uint32_t ISR_TARGET_US;

const float R_E_25C;
const float ALPHA_CU;
const float BL_0;
const float M_MS;
const float R_MS;
const float K_MS0;
const float F_S_MECH;
const float X_MAX;
const float X_LIM;

const float D_0;
const float LAMBDA_OPT;
const float PHOTODIODE_RESPONSIVITY;
const float PHOTODIODE_AREA;

const float R_F;
const float C_F;
const float AA_FILTER_FC;
const int AA_FILTER_ORDER;

const size_t VOLTERRA_MEMORY_M;
const size_t VOLTERRA_ORDER;
const float RLS_LAMBDA;
const size_t RLS_UPDATE_DECIMATION;
const int VOLTERRA_SPARSITY_WINDOW;

const float OBSERVER_POLE_MIN_HZ;
const float OBSERVER_POLE_MAX_HZ;

const float CL_POLE_MIN_HZ;
const float CL_POLE_MAX_HZ;
const float CL_DAMPING_ZETA;

const float LIMITER_KNEE_RATIO;
const float LIMITER_HARD_RATIO;

const int Q15_SHIFT;
const int Q31_SHIFT;
const float Q15_SCALE;
const float Q31_SCALE;

const size_t TELEMETRY_RING_DEPTH;
const size_t TELEMETRY_SAMPLE_SIZE;
const size_t TELEMETRY_BATCH_SIZE;

const float LSP_BL_OFFSET_THRESHOLD;
const float LSP_KMS_ASYMMETRY_THRESHOLD;
const float LSP_THERMAL_UPDATE_INTERVAL_MS;

const float PI;
const float TWO_PI;
const float SQRT_2;
const float INV_SQRT_2;

const float ADC_VREF;
const float DAC_VREF;
const float ADC_FULL_SCALE;
const float DAC_FULL_SCALE;
const float VOLTS_PER_LSB;

const float DISPLACEMENT_SCALE;
const float DISPLACEMENT_OFFSET;

const float CURRENT_SENSE_GAIN;
const float SHUNT_RESISTANCE;
const float CURRENT_SCALE;

const float VOLTAGE_DIVIDER_RATIO;
const float VOLTAGE_SCALE;

} // namespace dsp