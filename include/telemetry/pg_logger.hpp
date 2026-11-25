#pragma once

/**
 * @file pg_logger.hpp
 * @brief Asynchronous PostgreSQL telemetry logger using libpq.
 * 
 * Dedicated logger thread drains SPSC ring buffer in batches of 128 samples
 * via libpq async COPY/INSERT into JSONB tables.
 * Zero audio-thread blocking - logger runs independently.
 */

#include <cstdint>
#include <thread>
#include <atomic>
#include <string>
#include <vector>
#include "spsc_ring_buffer.hpp"

namespace telemetry {

// ============================================================================
// Logger Configuration
// ============================================================================

struct PgLoggerConfig {
    std::string host = "localhost";
    uint16_t port = 5432;
    std::string dbname = "loudspeaker_telemetry";
    std::string user = "postgres";
    std::string password = "";
    std::string driver_uid;  // UUID for this driver
    int sample_rate = 96000;
    size_t batch_size = TELEMETRY_BATCH_SIZE;  // 128
    int connect_timeout = 10;  // seconds
    int keepalive_idle = 30;
    int keepalive_interval = 10;
    int keepalive_count = 3;
};

// ============================================================================
// Logger State
// ============================================================================

class PgLogger {
public:
    PgLogger() noexcept = default;
    ~PgLogger() noexcept { stop(); }
    
    PgLogger(const PgLogger&) = delete;
    PgLogger& operator=(const PgLogger&) = delete;
    
    /**
     * @brief Start logger thread and connect to PostgreSQL.
     * 
     * @param config Connection and runtime configuration
     * @return true on success
     */
    bool start(const PgLoggerConfig& config) noexcept;
    
    /**
     * @brief Stop logger thread and disconnect.
     */
    void stop() noexcept;
    
    /**
     * @brief Check if logger is running.
     */
    bool running() const noexcept { return running_.load(std::memory_order_acquire); }
    
    /**
     * @brief Get logger statistics.
     */
    struct Stats {
        uint64_t samples_written = 0;
        uint64_t batches_written = 0;
        uint64_t ring_overruns = 0;
        uint64_t db_errors = 0;
        uint64_t last_write_us = 0;
        uint64_t max_write_us = 0;
        bool connected = false;
    };
    
    Stats get_stats() const noexcept;

private:
    // Logger thread function
    void logger_thread() noexcept;
    
    // PostgreSQL connection management
    bool connect() noexcept;
    void disconnect() noexcept;
    bool ensure_connected() noexcept;
    
    // Batch writing
    bool write_batch(const TelemetrySample* samples, size_t count) noexcept;
    bool write_batch_copy(const TelemetrySample* samples, size_t count) noexcept;
    bool write_batch_insert(const TelemetrySample* samples, size_t count) noexcept;
    
    // Schema setup
    bool ensure_schema() noexcept;
    
    // Configuration
    PgLoggerConfig config_;
    
    // Runtime state
    std::atomic<bool> running_{false};
    std::atomic<bool> should_stop_{false};
    std::thread thread_;
    
    // PostgreSQL connection (opaque - defined in pg_logger.cpp)
    struct PgConnection;
    PgConnection* conn_ = nullptr;
    
    // Statistics
    std::atomic<uint64_t> stats_samples_written_{0};
    std::atomic<uint64_t> stats_batches_written_{0};
    std::atomic<uint64_t> stats_ring_overruns_{0};
    std::atomic<uint64_t> stats_db_errors_{0};
    std::atomic<uint64_t> stats_last_write_us_{0};
    std::atomic<uint64_t> stats_max_write_us_{0};
    std::atomic<bool> stats_connected_{false};
};

// Global logger instance
extern PgLogger g_pg_logger;

// ============================================================================
// C-style API
// ============================================================================

bool pg_logger_start(const PgLoggerConfig& config) noexcept;
void pg_logger_stop() noexcept;
bool pg_logger_running() noexcept;
PgLogger::Stats pg_logger_get_stats() noexcept;

} // namespace telemetry