-- Closed-Loop Optoelectronic Loudspeaker System
-- PostgreSQL 16 Schema for Batched JSONB Telemetry and LSP Storage
-- Compatible with standard PostgreSQL 16 (no TimescaleDB extension required)
-- Batch-packed: 128 samples per row = 750 rows/s @ 96 kHz

-- ============================================================================
-- Extensions
-- ============================================================================

CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- ============================================================================
-- driver_telemetry_batch: Batched telemetry (128 samples per row = 750 rows/s @ 96 kHz)
-- ============================================================================

CREATE TABLE driver_telemetry_batch (
    id              BIGSERIAL,
    driver_uid      UUID        NOT NULL,
    ts_start        TIMESTAMPTZ NOT NULL DEFAULT now(),
    sample_period_ns INTEGER    NOT NULL DEFAULT 10417,  -- 96 kHz = 10.417 µs = 10417 ns
    sample_count    SMALLINT    NOT NULL DEFAULT 128,
    payload         JSONB       NOT NULL,           -- array of 128 sample objects
    bl_est_min      DOUBLE PRECISION,               -- min Bl_est in batch (populated by logger)
    bl_est_max      DOUBLE PRECISION,               -- max Bl_est in batch (populated by logger)
    kms_est_min     DOUBLE PRECISION,               -- min K_ms_est in batch
    kms_est_max     DOUBLE PRECISION,               -- max K_ms_est in batch
    lim_active_count SMALLINT,                      -- count of limiter_active=true in batch
    seq_start       BIGINT        NOT NULL          -- sequence number of first sample in batch
);

-- Primary key for time-series partitioning
ALTER TABLE driver_telemetry_batch ADD PRIMARY KEY (driver_uid, ts_start);

-- Composite index for driver + time-range queries
CREATE INDEX idx_telemetry_driver_ts ON driver_telemetry_batch (driver_uid, ts_start DESC);

-- B-tree indexes on per-batch min/max for range queries
CREATE INDEX idx_telemetry_bl_min ON driver_telemetry_batch (bl_est_min) WHERE bl_est_min IS NOT NULL;
CREATE INDEX idx_telemetry_bl_max ON driver_telemetry_batch (bl_est_max) WHERE bl_est_max IS NOT NULL;
CREATE INDEX idx_telemetry_kms_min ON driver_telemetry_batch (kms_est_min) WHERE kms_est_min IS NOT NULL;
CREATE INDEX idx_telemetry_kms_max ON driver_telemetry_batch (kms_est_max) WHERE kms_est_max IS NOT NULL;
CREATE INDEX idx_telemetry_limiter_count ON driver_telemetry_batch (lim_active_count) WHERE lim_active_count > 0;

-- Optional: Partition by time for high-volume deployments (native PostgreSQL partitioning)
-- CREATE TABLE driver_telemetry_batch_y2026m01 PARTITION OF driver_telemetry_batch
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
    phase_deg DOUBLE PRECISION,
    freq_hz DOUBLE PRECISION,
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

-- Latest telemetry batch per driver
CREATE VIEW latest_telemetry_batch AS
SELECT DISTINCT ON (driver_uid) driver_uid, ts_start, payload, seq_start
FROM driver_telemetry_batch
ORDER BY driver_uid, ts_start DESC;

-- Bl_est range detection per batch (uses bl_est_min/bl_est_max indexes)
CREATE VIEW batch_bl_range AS
SELECT 
    driver_uid,
    ts_start,
    bl_est_min,
    bl_est_max,
    (bl_est_max - bl_est_min) AS bl_range,
    sample_count
FROM driver_telemetry_batch
WHERE bl_est_min IS NOT NULL AND bl_est_max IS NOT NULL
ORDER BY driver_uid, ts_start DESC;

-- Limiter events (batches with any limiter activation)
CREATE VIEW limiter_events AS
SELECT driver_uid, ts_start, lim_active_count, sample_count
FROM driver_telemetry_batch
WHERE lim_active_count > 0
ORDER BY ts_start DESC;

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

-- GRANT SELECT, INSERT ON driver_telemetry_batch TO telemetry_writer;
-- GRANT SELECT ON driver_telemetry_batch TO telemetry_reader;
-- GRANT SELECT, INSERT ON lsp_curves TO telemetry_writer;
-- GRANT SELECT ON lsp_curves TO telemetry_reader;
-- GRANT SELECT ON driver_metadata TO telemetry_reader;
-- GRANT SELECT ON sweep_sessions TO telemetry_reader;
-- GRANT USAGE ON ALL SEQUENCES IN SCHEMA public TO telemetry_writer;