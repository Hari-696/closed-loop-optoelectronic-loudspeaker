-- Closed-Loop Optoelectronic Loudspeaker System
-- PostgreSQL Schema for Telemetry and LSP Storage
-- Compatible with AES-style telemetry and GIN-indexed JSONB

-- ============================================================================
-- Extensions
-- ============================================================================

CREATE EXTENSION IF NOT EXISTS "uuid-ossp";
CREATE EXTENSION IF NOT EXISTS "btree_gin";

-- ============================================================================
-- driver_telemetry: Real-time per-sample telemetry (96 kHz)
-- ============================================================================

CREATE TABLE driver_telemetry (
    id BIGSERIAL PRIMARY KEY,
    driver_uid UUID NOT NULL,
    ts TIMESTAMPTZ NOT NULL DEFAULT now(),
    sample_rate INTEGER NOT NULL,
    payload JSONB NOT NULL,
    seq BIGINT NOT NULL
);

-- Primary index for time-series queries
CREATE INDEX idx_telemetry_driver_ts ON driver_telemetry (driver_uid, ts DESC);

-- GIN index on full JSONB payload for flexible querying
CREATE INDEX idx_telemetry_payload_gin ON driver_telemetry USING GIN (payload);

-- Partial GIN index on Bl_est for fast force-factor queries
CREATE INDEX idx_telemetry_payload_bl ON driver_telemetry USING GIN ((payload -> 'Bl_est'))
WHERE payload ? 'Bl_est';

-- Partial GIN index on K_ms_est for stiffness queries
CREATE INDEX idx_telemetry_payload_kms ON driver_telemetry USING GIN ((payload -> 'K_ms_est'))
WHERE payload ? 'K_ms_est';

-- Partial GIN index on limiter_active for protection events
CREATE INDEX idx_telemetry_limiter ON driver_telemetry USING GIN ((payload -> 'limiter_active'))
WHERE (payload ->> 'limiter_active')::boolean = true;

-- Optional: Partition by time for high-volume deployments
-- CREATE TABLE driver_telemetry_y2026m01 PARTITION OF driver_telemetry
-- FOR VALUES FROM ('2026-01-01') TO ('2026-02-01');

-- ============================================================================
-- lsp_curves: Large-Signal Parameter curves (quasi-static sweeps)
-- ============================================================================

CREATE TABLE lsp_curves (
    id BIGSERIAL PRIMARY KEY,
    driver_uid UUID NOT NULL,
    sweep_id UUID NOT NULL,
    x_mm DOUBLE PRECISION NOT NULL,
    bl_tm DOUBLE PRECISION,
    k_ms_npm DOUBLE PRECISION,
    l_e_mh DOUBLE PRECISION,
    r_e_ohm DOUBLE PRECISION,
    captured_at TIMESTAMPTZ DEFAULT now()
);

-- Unique constraint: one curve point per driver/sweep/displacement
CREATE UNIQUE INDEX idx_lsp_unique ON lsp_curves (driver_uid, sweep_id, x_mm);

-- Index for driver-level queries
CREATE INDEX idx_lsp_driver ON lsp_curves (driver_uid, captured_at DESC);

-- Index for sweep-level queries
CREATE INDEX idx_lsp_sweep ON lsp_curves (sweep_id, x_mm);

-- ============================================================================
-- driver_metadata: Driver identity and calibration info
-- ============================================================================

CREATE TABLE driver_metadata (
    driver_uid UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    serial_number VARCHAR(64) UNIQUE NOT NULL,
    model_name VARCHAR(128),
    manufacture_date DATE,
    nominal_re_ohm DOUBLE PRECISION DEFAULT 6.8,
    nominal_bl_tm DOUBLE PRECISION DEFAULT 14.2,
    nominal_m_ms_kg DOUBLE PRECISION DEFAULT 0.042,
    nominal_r_ms_nsm DOUBLE PRECISION DEFAULT 1.8,
    nominal_k_ms0_npm DOUBLE PRECISION DEFAULT 19500.0,
    nominal_f_s_hz DOUBLE PRECISION DEFAULT 108.0,
    x_max_mm DOUBLE PRECISION DEFAULT 12.0,
    x_lim_mm DOUBLE PRECISION DEFAULT 15.0,
    d0_mm DOUBLE PRECISION DEFAULT 18.0,
    r_f_ohm DOUBLE PRECISION DEFAULT 49900.0,
    c_f_pf DOUBLE PRECISION DEFAULT 56.0,
    firmware_version VARCHAR(32),
    calibration_data JSONB,
    created_at TIMESTAMPTZ DEFAULT now(),
    updated_at TIMESTAMPTZ DEFAULT now()
);

-- ============================================================================
-- sweep_sessions: Log of LSP extraction sweeps
-- ============================================================================

CREATE TABLE sweep_sessions (
    sweep_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    driver_uid UUID NOT NULL REFERENCES driver_metadata(driver_uid),
    sweep_type VARCHAR(32) NOT NULL,  -- 'quasi-static', 'sine-sweep', 'multitone'
    start_freq_hz DOUBLE PRECISION,
    end_freq_hz DOUBLE PRECISION,
    amplitude_v DOUBLE PRECISION,
    temperature_c DOUBLE PRECISION,
    status VARCHAR(16) DEFAULT 'running',  -- 'running', 'completed', 'aborted'
    notes TEXT,
    started_at TIMESTAMPTZ DEFAULT now(),
    completed_at TIMESTAMPTZ
);

CREATE INDEX idx_sweep_driver ON sweep_sessions (driver_uid, started_at DESC);

-- ============================================================================
-- Views for Common Queries
-- ============================================================================

-- Latest telemetry per driver
CREATE VIEW latest_telemetry AS
SELECT DISTINCT ON (driver_uid) driver_uid, ts, payload, seq
FROM driver_telemetry
ORDER BY driver_uid, ts DESC;

-- Average THD/IMD per driver over last hour
CREATE VIEW hourly_distortion AS
SELECT 
    driver_uid,
    date_trunc('hour', ts) AS hour,
    AVG((payload->>'thd_pct')::float) AS avg_thd_pct,
    AVG((payload->>'imd_pct')::float) AS avg_imd_pct,
    MAX((payload->>'thd_pct')::float) AS max_thd_pct,
    COUNT(*) AS sample_count
FROM driver_telemetry
WHERE ts >= now() - interval '24 hours'
GROUP BY driver_uid, date_trunc('hour', ts);

-- Bl(x) curve reconstruction from telemetry (median per bin)
CREATE VIEW bl_curve_telemetry AS
SELECT 
    driver_uid,
    round((payload->>'x_mm')::float / 0.5) * 0.5 AS x_bin_mm,
    percentile_cont(0.5) WITHIN GROUP (ORDER BY (payload->>'Bl_est')::float) AS bl_median_tm,
    COUNT(*) AS sample_count
FROM driver_telemetry
WHERE payload ? 'Bl_est'
  AND ts >= now() - interval '1 hour'
GROUP BY driver_uid, round((payload->>'x_mm')::float / 0.5) * 0.5
ORDER BY driver_uid, x_bin_mm;

-- ============================================================================
-- Helper Functions
-- ============================================================================

-- Convert nanosecond timestamp to TIMESTAMPTZ
CREATE OR REPLACE FUNCTION ns_to_timestamptz(ns BIGINT)
RETURNS TIMESTAMPTZ LANGUAGE sql IMMUTABLE AS $$
    SELECT to_timestamp(ns / 1e9);
$$;

-- Extract float from JSONB with default
CREATE OR REPLACE FUNCTION jsonb_get_float(j JSONB, key TEXT, default_val FLOAT DEFAULT 0.0)
RETURNS FLOAT LANGUAGE sql IMMUTABLE AS $$
    SELECT COALESCE((j ->> key)::float, default_val);
$$;

-- Extract boolean from JSONB with default
CREATE OR REPLACE FUNCTION jsonb_get_bool(j JSONB, key TEXT, default_val BOOLEAN DEFAULT false)
RETURNS BOOLEAN LANGUAGE sql IMMUTABLE AS $$
    SELECT COALESCE((j ->> key)::boolean, default_val);
$$;

-- ============================================================================
-- Grants (adjust for your deployment)
-- ============================================================================

-- GRANT SELECT, INSERT ON driver_telemetry TO telemetry_writer;
-- GRANT SELECT ON driver_telemetry TO telemetry_reader;
-- GRANT SELECT, INSERT ON lsp_curves TO telemetry_writer;
-- GRANT SELECT ON lsp_curves TO telemetry_reader;
-- GRANT SELECT ON driver_metadata TO telemetry_reader;
-- GRANT SELECT ON sweep_sessions TO telemetry_reader;
-- GRANT USAGE ON ALL SEQUENCES IN SCHEMA public TO telemetry_writer;