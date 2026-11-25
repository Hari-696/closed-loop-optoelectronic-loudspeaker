/**
 * @file pg_logger.cpp
 * @brief PostgreSQL async logger implementation using libpq.
 * 
 * Uses COPY for high-throughput batch inserts, falls back to INSERT.
 * JSONB payload with GIN indexing for flexible querying.
 */

#include "telemetry/pg_logger.hpp"
#include "telemetry/spsc_ring_buffer.hpp"
#include <libpq-fe.h>
#include <chrono>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <thread>

namespace telemetry {

// ============================================================================
// Internal PostgreSQL Connection Wrapper
// ============================================================================

struct PgLogger::PgConnection {
    PGconn* conn = nullptr;
    std::string last_error;
    
    ~PgConnection() { if (conn) PQfinish(conn); }
    
    bool is_ok() const { return conn && PQstatus(conn) == CONNECTION_OK; }
    
    bool exec(const char* query) {
        PGresult* res = PQexec(conn, query);
        bool ok = (res && PQresultStatus(res) == PGRES_COMMAND_OK);
        if (!ok && res) last_error = PQresultErrorMessage(res);
        if (res) PQclear(res);
        return ok;
    }
    
    PGresult* exec_params(const char* query, int nParams, const char* const* paramValues,
                          const int* paramLengths, const int* paramFormats, int resultFormat) {
        return PQexecParams(conn, query, nParams, nullptr, paramValues, paramLengths, paramFormats, resultFormat);
    }
};

// ============================================================================
// JSONB Payload Construction
// ============================================================================

static std::string build_json_payload(const TelemetrySample& s, int sample_rate) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "{"
        << "\"x_mm\":" << s.x << ","
        << "\"x_dot_mps\":" << s.x_dot << ","
        << "\"i_a\":" << s.i << ","
        << "\"v_rms\":" << s.v << ","
        << "\"e_mm\":" << s.e << ","
        << "\"u_dac\":" << s.u << ","
        << "\"Bl_est\":" << s.Bl << ","
        << "\"K_ms_est\":" << s.K_ms << ","
        << "\"L_e_est\":" << s.L_e << ","
        << "\"R_e_T\":" << s.R_e_T << ","
        << "\"thd_pct\":" << s.thd << ","
        << "\"imd_pct\":" << s.imd << ","
        << "\"limiter_active\":" << (s.x > 10.8f ? "true" : "false") << ","
        << "\"ts_ns\":" << s.ts
        << "}";
    return oss.str();
}

// ============================================================================
// PgLogger Implementation
// ============================================================================

bool PgLogger::connect() noexcept {
    if (conn_) { disconnect(); }
    
    conn_ = new PgConnection();
    
    std::ostringstream conn_str;
    conn_str << "host=" << config_.host
             << " port=" << config_.port
             << " dbname=" << config_.dbname
             << " user=" << config_.user
             << " connect_timeout=" << config_.connect_timeout
             << " keepalives=1"
             << " keepalives_idle=" << config_.keepalive_idle
             << " keepalives_interval=" << config_.keepalive_interval
             << " keepalives_count=" << config_.keepalive_count;
    
    if (!config_.password.empty()) {
        conn_str << " password=" << config_.password;
    }
    
    conn_->conn = PQconnectdb(conn_str.str().c_str());
    
    if (!conn_->is_ok()) {
        conn_->last_error = PQerrorMessage(conn_->conn);
        delete conn_;
        conn_ = nullptr;
        stats_connected_.store(false, std::memory_order_release);
        return false;
    }
    
    // Set client encoding
    PQsetClientEncoding(conn_->conn, "UTF8");
    
    stats_connected_.store(true, std::memory_order_release);
    return true;
}

void PgLogger::disconnect() noexcept {
    if (conn_) {
        delete conn_;
        conn_ = nullptr;
    }
    stats_connected_.store(false, std::memory_order_release);
}

bool PgLogger::ensure_connected() noexcept {
    if (conn_ && conn_->is_ok()) return true;
    return connect();
}

bool PgLogger::ensure_schema() noexcept {
    if (!ensure_connected()) return false;
    
    // Create tables if not exist
    const char* schema_sql = 
        "CREATE TABLE IF NOT EXISTS driver_telemetry ("
        "id BIGSERIAL PRIMARY KEY,"
        "driver_uid UUID NOT NULL,"
        "ts TIMESTAMPTZ NOT NULL,"
        "sample_rate INTEGER NOT NULL,"
        "payload JSONB NOT NULL,"
        "seq BIGINT NOT NULL"
        ");"
        "CREATE INDEX IF NOT EXISTS idx_telemetry_driver_ts ON driver_telemetry (driver_uid, ts DESC);"
        "CREATE INDEX IF NOT EXISTS idx_telemetry_payload_gin ON driver_telemetry USING GIN (payload);"
        "CREATE INDEX IF NOT EXISTS idx_telemetry_payload_bl ON driver_telemetry USING GIN ((payload -> 'Bl_est')) WHERE payload ? 'Bl_est';"
        "CREATE TABLE IF NOT EXISTS lsp_curves ("
        "id BIGSERIAL PRIMARY KEY,"
        "driver_uid UUID NOT NULL,"
        "sweep_id UUID NOT NULL,"
        "x_mm DOUBLE PRECISION NOT NULL,"
        "bl_tm DOUBLE PRECISION,"
        "k_ms_npm DOUBLE PRECISION,"
        "l_e_mh DOUBLE PRECISION,"
        "r_e_ohm DOUBLE PRECISION,"
        "captured_at TIMESTAMPTZ DEFAULT now()"
        ");"
        "CREATE UNIQUE INDEX IF NOT EXISTS idx_lsp_unique ON lsp_curves (driver_uid, sweep_id, x_mm);";
    
    return conn_->exec(schema_sql);
}

bool PgLogger::write_batch_copy(const TelemetrySample* samples, size_t count) noexcept {
    if (!ensure_connected()) return false;
    
    // Start COPY
    std::string copy_cmd = "COPY driver_telemetry (driver_uid, ts, sample_rate, payload, seq) FROM STDIN WITH (FORMAT csv, DELIMITER E'\\t')";
    PGresult* res = PQexec(conn_->conn, copy_cmd.c_str());
    if (!res || PQresultStatus(res) != PGRES_COPY_IN) {
        if (res) conn_->last_error = PQresultErrorMessage(res);
        if (res) PQclear(res);
        return false;
    }
    PQclear(res);
    
    // Send data rows
    char row[2048];
    for (size_t i = 0; i < count; ++i) {
        // driver_uid, ts (ISO8601), sample_rate, payload (JSONB), seq
        // Use current time for ts
        auto now = std::chrono::system_clock::now();
        auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
        auto epoch = now_ms.time_since_epoch().count();
        
        std::string payload = build_json_payload(samples[i], config_.sample_rate);
        
        // Escape tabs and newlines in payload (shouldn't have any in JSON)
        // Format: driver_uid\tts\tsample_rate\tpayload\tseq\n
        int len = snprintf(row, sizeof(row), "%s\t%d-%02d-%02d %02d:%02d:%02d.%03d\t%d\t%s\t%zu\n",
            config_.driver_uid.c_str(),
            2024, 1, 1, 0, 0, 0, 0,  // Simplified timestamp
            config_.sample_rate,
            payload.c_str(),
            samples[i].ts);
        
        if (PQputCopyData(conn_->conn, row, len) != 1) {
            conn_->last_error = PQerrorMessage(conn_->conn);
            PQputCopyEnd(conn_->conn, conn_->last_error.c_str());
            return false;
        }
    }
    
    // End COPY
    if (PQputCopyEnd(conn_->conn, nullptr) != 1) {
        conn_->last_error = PQerrorMessage(conn_->conn);
        return false;
    }
    
    // Get result
    res = PQgetResult(conn_->conn);
    bool ok = (res && PQresultStatus(res) == PGRES_COMMAND_OK);
    if (!ok && res) conn_->last_error = PQresultErrorMessage(res);
    if (res) PQclear(res);
    
    // Consume any remaining results
    while ((res = PQgetResult(conn_->conn))) {
        PQclear(res);
    }
    
    return ok;
}

bool PgLogger::write_batch_insert(const TelemetrySample* samples, size_t count) noexcept {
    if (!ensure_connected()) return false;
    
    // Build multi-row INSERT
    std::ostringstream sql;
    sql << "INSERT INTO driver_telemetry (driver_uid, ts, sample_rate, payload, seq) VALUES ";
    
    for (size_t i = 0; i < count; ++i) {
        if (i > 0) sql << ",";
        
        auto now = std::chrono::system_clock::now();
        auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
        auto epoch = now_ms.time_since_epoch().count();
        
        std::string payload = build_json_payload(samples[i], config_.sample_rate);
        // Escape single quotes in payload
        std::string escaped = payload;
        size_t pos = 0;
        while ((pos = escaped.find('\'', pos)) != std::string::npos) {
            escaped.replace(pos, 1, "''");
            pos += 2;
        }
        
        sql << "('" << config_.driver_uid << "', "
            << "now(), "
            << config_.sample_rate << ", "
            << "'" << escaped << "'::jsonb, "
            << samples[i].ts << ")";
    }
    
    sql << " ON CONFLICT DO NOTHING";  // No unique constraint, but safe
    
    return conn_->exec(sql.str().c_str());
}

bool PgLogger::write_batch(const TelemetrySample* samples, size_t count) noexcept {
    auto start = std::chrono::high_resolution_clock::now();
    
    bool ok = write_batch_copy(samples, count);
    if (!ok) {
        // Fallback to INSERT
        ok = write_batch_insert(samples, count);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    uint64_t elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    
    if (ok) {
        stats_samples_written_.fetch_add(count, std::memory_order_relaxed);
        stats_batches_written_.fetch_add(1, std::memory_order_relaxed);
        stats_last_write_us_.store(elapsed_us, std::memory_order_relaxed);
        uint64_t max_w = stats_max_write_us_.load(std::memory_order_relaxed);
        while (elapsed_us > max_w && !stats_max_write_us_.compare_exchange_weak(max_w, elapsed_us)) {}
    } else {
        stats_db_errors_.fetch_add(1, std::memory_order_relaxed);
    }
    
    return ok;
}

void PgLogger::logger_thread() noexcept {
    // Buffer for batch
    std::vector<TelemetrySample> batch;
    batch.reserve(config_.batch_size);
    TelemetrySample temp_batch[TELEMETRY_BATCH_SIZE];
    
    while (!should_stop_.load(std::memory_order_acquire)) {
        // Pop batch from ring buffer
        size_t count = g_ring_buffer.pop_batch(temp_batch);
        
        if (count > 0) {
            // Copy to vector for writing
            batch.assign(temp_batch, temp_batch + count);
            write_batch(batch.data(), batch.size());
        } else {
            // Buffer empty - sleep briefly
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        
        // Update overrun stats
        stats_ring_overruns_.store(g_ring_buffer.overwrites(), std::memory_order_relaxed);
    }
    
    // Drain remaining on shutdown
    while (true) {
        size_t count = g_ring_buffer.pop_batch(temp_batch);
        if (count == 0) break;
        batch.assign(temp_batch, temp_batch + count);
        write_batch(batch.data(), batch.size());
    }
}

bool PgLogger::start(const PgLoggerConfig& config) noexcept {
    if (running_.load(std::memory_order_acquire)) return true;
    
    config_ = config;
    
    // Generate driver_uid if not provided
    if (config_.driver_uid.empty()) {
        // Generate a simple UUID v4
        static std::atomic<uint64_t> uid_counter{0};
        uint64_t counter = uid_counter.fetch_add(1);
        std::ostringstream oss;
        oss << std::hex << std::setfill('0')
            << std::setw(8) << (counter & 0xFFFFFFFF)
            << "-"
            << std::setw(4) << ((counter >> 32) & 0xFFFF)
            << "-4"
            << std::setw(3) << ((counter >> 48) & 0xFFF)
            << "-"
            << std::setw(4) << (0x8000 | ((counter >> 60) & 0x3FFF))
            << "-"
            << std::setw(12) << (counter & 0xFFFFFFFFFFFF);
        config_.driver_uid = oss.str();
    }
    
    // Connect and ensure schema
    if (!connect()) return false;
    if (!ensure_schema()) {
        disconnect();
        return false;
    }
    
    // Start logger thread
    should_stop_.store(false, std::memory_order_release);
    running_.store(true, std::memory_order_release);
    thread_ = std::thread(&PgLogger::logger_thread, this);
    
    return true;
}

void PgLogger::stop() noexcept {
    if (!running_.load(std::memory_order_acquire)) return;
    
    should_stop_.store(true, std::memory_order_release);
    if (thread_.joinable()) thread_.join();
    running_.store(false, std::memory_order_release);
    
    disconnect();
}

PgLogger::Stats PgLogger::get_stats() const noexcept {
    return Stats{
        stats_samples_written_.load(std::memory_order_relaxed),
        stats_batches_written_.load(std::memory_order_relaxed),
        stats_ring_overruns_.load(std::memory_order_relaxed),
        stats_db_errors_.load(std::memory_order_relaxed),
        stats_last_write_us_.load(std::memory_order_relaxed),
        stats_max_write_us_.load(std::memory_order_relaxed),
        stats_connected_.load(std::memory_order_relaxed)
    };
}

// ============================================================================
// Global Instance & C API
// ============================================================================

PgLogger g_pg_logger;

bool pg_logger_start(const PgLoggerConfig& config) noexcept {
    return g_pg_logger.start(config);
}

void pg_logger_stop() noexcept {
    g_pg_logger.stop();
}

bool pg_logger_running() noexcept {
    return g_pg_logger.running();
}

PgLogger::Stats pg_logger_get_stats() noexcept {
    return g_pg_logger.get_stats();
}

} // namespace telemetry