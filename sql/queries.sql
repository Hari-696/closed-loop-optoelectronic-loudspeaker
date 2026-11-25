-- Closed-Loop Optoelectronic Loudspeaker System
-- AES-Compatible Diagnostic Queries (Section 5.4)
-- Run against the driver_telemetry and lsp_curves tables

-- ============================================================================
-- 1. Bl(x) Drop Detection: Find all samples where Bl fell below threshold
-- ============================================================================

-- All samples where estimated Bl < 12.5 T·m (coil exiting gap) for specific driver
SELECT 
    ts,
    (payload->>'x_mm')::float AS x_mm,
    (payload->>'Bl_est')::float AS bl_tm,
    (payload->>'i_a')::float AS current_a,
    (payload->>'v_rms')::float AS voltage_v
FROM driver_telemetry
WHERE driver_uid = 'a1b2c3d4-e5f6-7890-abcd-ef1234567890'  -- Replace with actual UUID
  AND ts >= now() - interval '1 hour'
  AND (payload->>'Bl_est')::float < 12.5
ORDER BY ts DESC
LIMIT 1000;

-- Count of Bl drop events per hour (last 24 hours)
SELECT 
    date_trunc('hour', ts) AS hour,
    COUNT(*) AS bl_drop_count,
    MIN((payload->>'Bl_est')::float) AS min_bl_tm
FROM driver_telemetry
WHERE driver_uid = 'a1b2c3d4-e5f6-7890-abcd-ef1234567890'
  AND ts >= now() - interval '24 hours'
  AND (payload->>'Bl_est')::float < 12.5
GROUP BY date_trunc('hour', ts)
ORDER BY hour DESC;

-- ============================================================================
-- 2. Over-Excursion Protection Events
-- ============================================================================

-- All limiter engagement events (active excursion limiting)
SELECT 
    ts,
    (payload->>'x_mm')::float AS x_mm,
    (payload->>'e_mm')::float AS error_mm,
    (payload->>'u_dac')::float AS output_v,
    (payload->>'limiter_active')::boolean AS limiter_on
FROM driver_telemetry
WHERE driver_uid = 'a1b2c3d4-e5f6-7890-abcd-ef1234567890'
  AND ts >= now() - interval '1 hour'
  AND (payload->>'limiter_active')::boolean = true
ORDER BY ts DESC;

-- Limiter engagement statistics
SELECT 
    date_trunc('minute', ts) AS minute,
    COUNT(*) AS limiter_events,
    MAX((payload->>'x_mm')::float) AS max_excursion_mm,
    AVG((payload->>'e_mm')::float) AS avg_error_mm
FROM driver_telemetry
WHERE driver_uid = 'a1b2c3d4-e5f6-7890-abcd-ef1234567890'
  AND ts >= now() - interval '1 hour'
  AND (payload->>'limiter_active')::boolean = true
GROUP BY date_trunc('minute', ts)
ORDER BY minute DESC;

-- Hard limit events (|x| >= X_lim = 15mm)
SELECT 
    ts,
    (payload->>'x_mm')::float AS x_mm
FROM driver_telemetry
WHERE driver_uid = 'a1b2c3d4-e5f6-7890-abcd-ef1234567890'
  AND ts >= now() - interval '1 hour'
  AND ABS((payload->>'x_mm')::float) >= 15.0
ORDER BY ts DESC;

-- ============================================================================
-- 3. Thermal Tracking: Voice-Coil Temperature from R_e
-- ============================================================================

-- Temperature evolution over time (from R_e thermal tracking)
SELECT 
    ts,
    (payload->>'R_e_T')::float AS r_e_ohm,
    25.0 + ((payload->>'R_e_T')::float / 6.8 - 1.0) / 0.00393 AS temp_c
FROM driver_telemetry
WHERE driver_uid = 'a1b2c3d4-e5f6-7890-abcd-ef1234567890'
  AND ts >= now() - interval '1 hour'
  AND (payload->>'R_e_T')::float > 0
ORDER BY ts DESC;

-- Thermal compression detection: THD rise correlated with temperature
SELECT 
    date_trunc('minute', ts) AS minute,
    AVG((payload->>'R_e_T')::float) AS avg_r_e,
    AVG(25.0 + ((payload->>'R_e_T')::float / 6.8 - 1.0) / 0.00393) AS avg_temp_c,
    AVG((payload->>'thd_pct')::float) AS avg_thd_pct,
    MAX((payload->>'thd_pct')::float) AS max_thd_pct
FROM driver_telemetry
WHERE driver_uid = 'a1b2c3d4-e5f6-7890-abcd-ef1234567890'
  AND ts >= now() - interval '1 hour'
  AND (payload->>'R_e_T')::float > 0
GROUP BY date_trunc('minute', ts)
ORDER BY minute DESC;

-- ============================================================================
-- 4. Large-Signal Parameter Curve Comparison (LSP Curves)
-- ============================================================================

-- Compare Bl(x) curves from two different sweeps
SELECT 
    c1.x_mm,
    c1.bl_tm AS bl_sweep1,
    c2.bl_tm AS bl_sweep2,
    c1.bl_tm - c2.bl_tm AS bl_delta
FROM lsp_curves c1
JOIN lsp_curves c2 
  ON c1.driver_uid = c2.driver_uid 
  AND c1.x_mm = c2.x_mm
  AND c1.sweep_id = '11111111-2222-3333-4444-555555555555'  -- Sweep 1
  AND c2.sweep_id = 'aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee'  -- Sweep 2
WHERE c1.driver_uid = 'a1b2c3d4-e5f6-7890-abcd-ef1234567890'
ORDER BY c1.x_mm;

-- K_ms(x) asymmetry: compare +x and -x stiffness
SELECT 
    ABS(x_mm) AS abs_x_mm,
    AVG(CASE WHEN x_mm > 0 THEN k_ms_npm END) AS kms_positive,
    AVG(CASE WHEN x_mm < 0 THEN k_ms_npm END) AS kms_negative,
    AVG(CASE WHEN x_mm > 0 THEN k_ms_npm END) - AVG(CASE WHEN x_mm < 0 THEN k_ms_npm END) AS asymmetry_npm
FROM lsp_curves
WHERE driver_uid = 'a1b2c3d4-e5f6-7890-abcd-ef1234567890'
  AND sweep_id = '11111111-2222-3333-4444-555555555555'
  AND k_ms_npm IS NOT NULL
GROUP BY ABS(x_mm)
ORDER BY abs_x_mm;

-- L_e(x) compression profile
SELECT 
    x_mm,
    l_e_mh
FROM lsp_curves
WHERE driver_uid = 'a1b2c3d4-e5f6-7890-abcd-ef1234567890'
  AND sweep_id = '11111111-2222-3333-4444-555555555555'
  AND l_e_mh IS NOT NULL
ORDER BY x_mm;

-- ============================================================================
-- 5. Distortion Analysis (THD/IMD vs Excursion)
-- ============================================================================

-- THD vs excursion level (binned)
SELECT 
    round(ABS((payload->>'x_mm')::float) / 1.0) * 1.0 AS x_bin_mm,
    AVG((payload->>'thd_pct')::float) AS avg_thd_pct,
    MAX((payload->>'thd_pct')::float) AS max_thd_pct,
    AVG((payload->>'imd_pct')::float) AS avg_imd_pct,
    COUNT(*) AS sample_count
FROM driver_telemetry
WHERE driver_uid = 'a1b2c3d4-e5f6-7890-abcd-ef1234567890'
  AND ts >= now() - interval '1 hour'
GROUP BY round(ABS((payload->>'x_mm')::float) / 1.0) * 1.0
ORDER BY x_bin_mm;

-- THD reduction: compare open-loop baseline (if logged) vs closed-loop
-- Requires baseline data in payload or separate table
SELECT 
    'closed_loop' AS mode,
    round(ABS((payload->>'x_mm')::float) / 1.0) * 1.0 AS x_bin_mm,
    AVG((payload->>'thd_pct')::float) AS avg_thd_pct
FROM driver_telemetry
WHERE driver_uid = 'a1b2c3d4-e5f6-7890-abcd-ef1234567890'
  AND ts >= now() - interval '1 hour'
GROUP BY round(ABS((payload->>'x_mm')::float) / 1.0) * 1.0
ORDER BY x_bin_mm;

-- ============================================================================
-- 6. Phase Margin and Group Delay Analysis
-- ============================================================================

-- Phase lag from telemetry (requires phase data in payload or lsp_curves)
SELECT 
    freq_hz,
    phase_deg
FROM lsp_curves
CROSS JOIN LATERAL (
    SELECT unnest(freq_hz_array) AS freq_hz,
           unnest(phase_deg_array) AS phase_deg
    -- Note: requires array columns or separate phase table
) p
WHERE driver_uid = 'a1b2c3d4-e5f6-7890-abcd-ef1234567890'
  AND sweep_id = '11111111-2222-3333-4444-555555555555';

-- Group delay approximation from phase slope
-- τ_g = -dφ/dω ≈ -(φ2 - φ1) / (ω2 - ω1)
WITH phase_data AS (
    SELECT 
        freq_hz,
        phase_deg,
        LAG(phase_deg) OVER (ORDER BY freq_hz) AS prev_phase,
        LAG(freq_hz) OVER (ORDER BY freq_hz) AS prev_freq
    FROM lsp_curves
    WHERE driver_uid = 'a1b2c3d4-e5f6-7890-abcd-ef1234567890'
      AND sweep_id = '11111111-2222-3333-4444-555555555555'
)
SELECT 
    freq_hz,
    phase_deg,
    CASE WHEN prev_freq IS NOT NULL AND prev_freq != freq_hz
         THEN -(phase_deg - prev_phase) / (2.0 * PI() * (freq_hz - prev_freq)) * 1000.0
    END AS group_delay_ms
FROM phase_data
WHERE prev_freq IS NOT NULL
ORDER BY freq_hz;

-- ============================================================================
-- 7. System Health and Aging Diagnostics
-- ============================================================================

-- Daily health summary
SELECT 
    date(ts) AS day,
    COUNT(*) AS total_samples,
    AVG((payload->>'thd_pct')::float) AS avg_thd,
    MAX((payload->>'thd_pct')::float) AS max_thd,
    AVG((payload->>'R_e_T')::float) AS avg_r_e,
    COUNT(*) FILTER (WHERE (payload->>'limiter_active')::boolean) AS limiter_events,
    COUNT(*) FILTER (WHERE ABS((payload->>'x_mm')::float) >= 15.0) AS hard_limit_events
FROM driver_telemetry
WHERE driver_uid = 'a1b2c3d4-e5f6-7890-abcd-ef1234567890'
  AND ts >= now() - interval '30 days'
GROUP BY date(ts)
ORDER BY day DESC;

-- Aging trend: Bl_0 drift over sweeps
SELECT 
    s.started_at,
    c.sweep_id,
    AVG(c.bl_tm) FILTER (WHERE c.x_mm BETWEEN -1.0 AND 1.0) AS bl_rest_tm,
    MAX(c.bl_tm) - MIN(c.bl_tm) AS bl_range_tm
FROM lsp_curves c
JOIN sweep_sessions s ON c.sweep_id = s.sweep_id
WHERE c.driver_uid = 'a1b2c3d4-e5f6-7890-abcd-ef1234567890'
  AND c.bl_tm IS NOT NULL
GROUP BY s.started_at, c.sweep_id
ORDER BY s.started_at;

-- Spider creep detection: K_ms0 drift
SELECT 
    s.started_at,
    c.sweep_id,
    AVG(c.k_ms_npm) FILTER (WHERE c.x_mm BETWEEN -1.0 AND 1.0) AS kms_rest_npm
FROM lsp_curves c
JOIN sweep_sessions s ON c.sweep_id = s.sweep_id
WHERE c.driver_uid = 'a1b2c3d4-e5f6-7890-abcd-ef1234567890'
  AND c.k_ms_npm IS NOT NULL
GROUP BY s.started_at, c.sweep_id
ORDER BY s.started_at;

-- ============================================================================
-- 8. Cross-Driver Comparison (Fleet Analysis)
-- ============================================================================

-- Average THD by driver model
SELECT 
    dm.model_name,
    COUNT(DISTINCT dt.driver_uid) AS driver_count,
    AVG((dt.payload->>'thd_pct')::float) AS fleet_avg_thd,
    PERCENTILE_CONT(0.5) WITHIN GROUP (ORDER BY (dt.payload->>'thd_pct')::float) AS fleet_median_thd
FROM driver_telemetry dt
JOIN driver_metadata dm ON dt.driver_uid = dm.driver_uid
WHERE dt.ts >= now() - interval '24 hours'
GROUP BY dm.model_name
ORDER BY fleet_avg_thd;

-- Limiter engagement rate by driver
SELECT 
    dt.driver_uid,
    dm.serial_number,
    COUNT(*) AS total_samples,
    COUNT(*) FILTER (WHERE (dt.payload->>'limiter_active')::boolean) AS limiter_events,
    ROUND(100.0 * COUNT(*) FILTER (WHERE (dt.payload->>'limiter_active')::boolean) / COUNT(*), 2) AS limiter_pct
FROM driver_telemetry dt
JOIN driver_metadata dm ON dt.driver_uid = dm.driver_uid
WHERE dt.ts >= now() - interval '1 hour'
GROUP BY dt.driver_uid, dm.serial_number
HAVING COUNT(*) > 1000
ORDER BY limiter_pct DESC;

-- ============================================================================
-- 9. Real-Time Monitoring Queries (for dashboards)
-- ============================================================================

-- Current status (last 10 seconds)
SELECT 
    driver_uid,
    MAX(ts) AS last_sample,
    (MAX(payload)->>'x_mm')::float AS last_x_mm,
    (MAX(payload)->>'thd_pct')::float AS last_thd_pct,
    (MAX(payload)->>'R_e_T')::float AS last_r_e,
    (MAX(payload)->>'limiter_active')::boolean AS limiter_active
FROM driver_telemetry
WHERE ts >= now() - interval '10 seconds'
GROUP BY driver_uid;

-- Alert: any driver with THD > 5% in last minute
SELECT 
    driver_uid,
    MAX(ts) AS ts,
    MAX((payload->>'thd_pct')::float) AS max_thd_pct,
    MAX((payload->>'x_mm')::float) AS max_x_mm
FROM driver_telemetry
WHERE ts >= now() - interval '1 minute'
GROUP BY driver_uid
HAVING MAX((payload->>'thd_pct')::float) > 5.0;

-- Alert: voice-coil temperature > 100°C
SELECT 
    driver_uid,
    MAX(ts) AS ts,
    MAX(25.0 + ((payload->>'R_e_T')::float / 6.8 - 1.0) / 0.00393) AS max_temp_c
FROM driver_telemetry
WHERE ts >= now() - interval '1 minute'
  AND (payload->>'R_e_T')::float > 0
GROUP BY driver_uid
HAVING MAX(25.0 + ((payload->>'R_e_T')::float / 6.8 - 1.0) / 0.00393) > 100.0;

-- ============================================================================
-- 10. Data Export for External Analysis (CSV/JSON)
-- ============================================================================

-- Export telemetry for specific sweep (JSONL format)
SELECT 
    jsonb_build_object(
        'ts', ts,
        'x_mm', (payload->>'x_mm')::float,
        'x_dot_mps', (payload->>'x_dot_mps')::float,
        'i_a', (payload->>'i_a')::float,
        'v_rms', (payload->>'v_rms')::float,
        'thd_pct', (payload->>'thd_pct')::float,
        'imd_pct', (payload->>'imd_pct')::float,
        'Bl_est', (payload->>'Bl_est')::float,
        'K_ms_est', (payload->>'K_ms_est')::float,
        'L_e_est', (payload->>'L_e_est')::float,
        'R_e_T', (payload->>'R_e_T')::float
    ) AS sample_json
FROM driver_telemetry
WHERE driver_uid = 'a1b2c3d4-e5f6-7890-abcd-ef1234567890'
  AND ts BETWEEN '2026-01-15 10:00:00' AND '2026-01-15 10:05:00'
ORDER BY ts;

-- Export LSP curves as CSV
COPY (
    SELECT 
        driver_uid,
        sweep_id,
        x_mm,
        bl_tm,
        k_ms_npm,
        l_e_mh,
        r_e_ohm,
        captured_at
    FROM lsp_curves
    WHERE driver_uid = 'a1b2c3d4-e5f6-7890-abcd-ef1234567890'
    ORDER BY sweep_id, x_mm
) TO STDOUT WITH CSV HEADER;

-- ============================================================================
-- Notes on AES Compatibility
-- ============================================================================
-- These queries follow AES-style telemetry conventions:
-- - TIMESTAMPTZ for absolute time (AES67/AES70)
-- - UUID for driver/sweep identification
-- - JSONB for flexible payload (AES70 OCA)
-- - GIN indexes for real-time analytical queries
-- - Partitioning-ready schema for high-volume deployments