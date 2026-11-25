#pragma once

/**
 * @file spsc_ring_buffer.hpp
 * @brief Lock-free, wait-free SPSC ring buffer for audio-thread to logger-thread telemetry.
 * 
 * 4096 entries × 64 bytes = 256 KB SRAM, cache-line aligned.
 * Producer (audio ISR) pushes with acquire/release semantics.
 * Consumer (logger thread) drains in batches of 128 for PostgreSQL COPY.
 */

#include <cstdint>
#include <atomic>
#include <array>
#include "config.hpp"

namespace telemetry {

// ============================================================================
// Telemetry Sample Structure (64 bytes, cache-line aligned)
// ============================================================================

struct alignas(64) TelemetrySample {
    float x;            ///< Displacement [mm]
    float x_dot;        ///< Velocity [m/s]
    float i;            ///< Current [A]
    float v;            ///< Voltage [V]
    float e;            ///< Excursion error [mm]
    float u;            ///< Output voltage [V]
    float Bl;           ///< Force factor estimate [T·m]
    float K_ms;         ///< Suspension stiffness estimate [N/m]
    float L_e;          ///< Inductance estimate [mH]
    float R_e_T;        ///< Thermal resistance [Ω]
    float thd;          ///< THD [%]
    float imd;          ///< IMD [%]
    uint64_t ts;        ///< Timestamp [sample count or ns]
    
    // Padding to exactly 64 bytes (13 floats = 52 bytes + 8 bytes = 60, pad 4)
    uint32_t _pad;
};

static_assert(sizeof(TelemetrySample) == 64, "TelemetrySample must be 64 bytes");

// ============================================================================
// Ring Buffer
// ============================================================================

class SpscRingBuffer {
public:
    static constexpr size_t CAPACITY = TELEMETRY_RING_DEPTH;  // 4096
    static constexpr size_t BATCH_SIZE = TELEMETRY_BATCH_SIZE; // 128
    
    SpscRingBuffer() noexcept = default;
    
    // Non-copyable, non-movable
    SpscRingBuffer(const SpscRingBuffer&) = delete;
    SpscRingBuffer& operator=(const SpscRingBuffer&) = delete;
    
    /**
     * @brief Initialize buffer (zero-initialize).
     */
    void init() noexcept {
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
        overwrites_.store(0, std::memory_order_relaxed);
    }
    
    /**
     * @brief Push sample (producer side - audio ISR).
     * 
     * Wait-free: if buffer full, overwrites oldest sample (overrun counted).
     * Uses release semantics for publication.
     * 
     * @param sample Telemetry sample to push
     * @return true if pushed without overwrite, false if overrun occurred
     */
    bool push(const TelemetrySample& sample) noexcept {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t next_head = (head + 1) % CAPACITY;
        size_t tail = tail_.load(std::memory_order_acquire);  // Acquire: see consumer's progress
        
        if (next_head == tail) {
            // Buffer full - overwrite oldest (tail advances)
            overwrites_.fetch_add(1, std::memory_order_relaxed);
            buffer_[head] = sample;
            tail_.store((tail + 1) % CAPACITY, std::memory_order_release);
            head_.store(next_head, std::memory_order_release);
            return false;  // Overrun
        }
        
        buffer_[head] = sample;
        head_.store(next_head, std::memory_order_release);  // Release: publish to consumer
        return true;
    }
    
    /**
     * @brief Pop batch of samples (consumer side - logger thread).
     * 
     * Drains up to BATCH_SIZE samples. Returns number of samples popped.
     * Uses acquire semantics to see producer's writes.
     * 
     * @param out Output array (must have capacity ≥ BATCH_SIZE)
     * @return Number of samples copied
     */
    size_t pop_batch(TelemetrySample* out) noexcept {
        size_t tail = tail_.load(std::memory_order_relaxed);
        size_t head = head_.load(std::memory_order_acquire);  // Acquire: see producer's writes
        
        size_t available = (head >= tail) ? (head - tail) : (CAPACITY - tail + head);
        size_t to_copy = (available < BATCH_SIZE) ? available : BATCH_SIZE;
        
        if (to_copy == 0) return 0;
        
        // Copy in one or two contiguous chunks
        size_t first_chunk = (to_copy <= CAPACITY - tail) ? to_copy : (CAPACITY - tail);
        
        for (size_t i = 0; i < first_chunk; ++i) {
            out[i] = buffer_[(tail + i) % CAPACITY];
        }
        
        if (to_copy > first_chunk) {
            for (size_t i = 0; i < to_copy - first_chunk; ++i) {
                out[first_chunk + i] = buffer_[i];
            }
        }
        
        tail_.store((tail + to_copy) % CAPACITY, std::memory_order_release);  // Release: consume
        return to_copy;
    }
    
    /**
     * @brief Get number of available samples (consumer view).
     */
    size_t available() const noexcept {
        size_t head = head_.load(std::memory_order_acquire);
        size_t tail = tail_.load(std::memory_order_acquire);
        return (head >= tail) ? (head - tail) : (CAPACITY - tail + head);
    }
    
    /**
     * @brief Get number of free slots (producer view).
     */
    size_t free() const noexcept {
        return CAPACITY - 1 - available();
    }
    
    /**
     * @brief Get overrun count.
     */
    uint64_t overwrites() const noexcept {
        return overwrites_.load(std::memory_order_relaxed);
    }
    
    /**
     * @brief Check if buffer is empty.
     */
    bool empty() const noexcept {
        size_t head = head_.load(std::memory_order_acquire);
        size_t tail = tail_.load(std::memory_order_acquire);
        return head == tail;
    }
    
    /**
     * @brief Check if buffer is full.
     */
    bool full() const noexcept {
        size_t head = head_.load(std::memory_order_acquire);
        size_t next_head = (head + 1) % CAPACITY;
        size_t tail = tail_.load(std::memory_order_acquire);
        return next_head == tail;
    }
    
    /**
     * @brief Reset buffer (call only when both threads quiesced).
     */
    void reset() noexcept {
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
        overwrites_.store(0, std::memory_order_relaxed);
    }

private:
    std::array<TelemetrySample, CAPACITY> buffer_;
    alignas(64) std::atomic<size_t> head_{0};  // Producer index
    alignas(64) std::atomic<size_t> tail_{0};  // Consumer index
    alignas(64) std::atomic<uint64_t> overwrites_{0};
};

// Global instance
extern SpscRingBuffer g_ring_buffer;

// ============================================================================
// C-style API (for dsp_pipeline.cpp)
// ============================================================================

DSP_INLINE void spsc_ring_buffer_init() noexcept {
    g_ring_buffer.init();
}

DSP_INLINE void spsc_ring_buffer_reset() noexcept {
    g_ring_buffer.reset();
}

DSP_INLINE bool spsc_ring_buffer_push(const TelemetrySample& sample) noexcept {
    return g_ring_buffer.push(sample);
}

DSP_INLINE size_t spsc_ring_buffer_pop_batch(TelemetrySample* out) noexcept {
    return g_ring_buffer.pop_batch(out);
}

DSP_INLINE size_t spsc_ring_buffer_available() noexcept {
    return g_ring_buffer.available();
}

DSP_INLINE uint64_t spsc_ring_buffer_overwrites() noexcept {
    return g_ring_buffer.overwrites();
}

} // namespace telemetry