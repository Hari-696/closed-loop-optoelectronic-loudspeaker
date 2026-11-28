/**
 * @file atomic_baremetal.hpp
 * @brief Bare-metal compatible atomic operations using GCC builtins
 * 
 * Replaces std::atomic for bare-metal Cortex-M7 builds where libatomic is not available.
 * Uses GCC __sync_* builtins which are available without libatomic.
 */

#pragma once

#include <cstdint>
#include <cstddef>

namespace dsp {

// Bare-metal atomic bool
class atomic_bool {
    uint8_t value_;
public:
    constexpr atomic_bool(bool v = false) noexcept : value_(v) {}
    
    bool load() const noexcept { 
        // Use volatile cast for GCC builtins
        return __sync_fetch_and_add(const_cast<uint8_t*>(&value_), 0) != 0; 
    }
    void store(bool v) noexcept { __sync_lock_release(&value_); value_ = v; }
    bool exchange(bool v) noexcept { return __sync_lock_test_and_set(&value_, v) != 0; }
    bool compare_exchange_weak(bool& expected, bool desired) noexcept {
        bool old = __sync_val_compare_and_swap(&value_, expected, desired);
        if (old == expected) return true;
        expected = old;
        return false;
    }
    bool compare_exchange_strong(bool& expected, bool desired) noexcept {
        return compare_exchange_weak(expected, desired);
    }
};

// Bare-metal atomic for integral types
template<typename T>
class atomic_integral {
    static_assert(sizeof(T) <= 8, "Atomic integral size must be <= 8 bytes");
    T value_;
public:
    constexpr atomic_integral(T v = 0) noexcept : value_(v) {}
    
    T load() const noexcept {
        if constexpr (sizeof(T) <= 4) {
            return __sync_fetch_and_add(const_cast<T*>(&value_), 0);
        } else {
            // For 64-bit on 32-bit ARM, use __sync_fetch_and_add_8
            return __sync_fetch_and_add_8(const_cast<T*>(&value_), 0);
        }
    }
    
    void store(T v) noexcept {
        if constexpr (sizeof(T) <= 4) {
            __sync_lock_release(&value_);
            value_ = v;
        } else {
            // For 64-bit, use __sync_lock_test_and_set with a loop
            // Actually, GCC doesn't have __sync_lock_release_8, so we use atomic exchange
            while (true) {
                T old = load();
                if (compare_exchange_strong(old, v)) break;
            }
        }
    }
    
    T exchange(T v) noexcept {
        if constexpr (sizeof(T) <= 4) {
            return __sync_lock_test_and_set(&value_, v);
        } else {
            T old = load();
            while (!compare_exchange_strong(old, v)) {
                old = load();
            }
            return old;
        }
    }
    
    bool compare_exchange_weak(T& expected, T desired) noexcept {
        if constexpr (sizeof(T) <= 4) {
            return __sync_bool_compare_and_swap(&value_, expected, desired);
        } else {
            return __sync_bool_compare_and_swap_8(&value_, expected, desired);
        }
    }
    
    bool compare_exchange_strong(T& expected, T desired) noexcept {
        return compare_exchange_weak(expected, desired);
    }
    
    T fetch_add(T v) noexcept {
        if constexpr (sizeof(T) <= 4) {
            return __sync_fetch_and_add(&value_, v);
        } else {
            return __sync_fetch_and_add_8(&value_, v);
        }
    }
    
    T fetch_sub(T v) noexcept {
        if constexpr (sizeof(T) <= 4) {
            return __sync_fetch_and_sub(&value_, v);
        } else {
            return __sync_fetch_and_sub_8(&value_, v);
        }
    }
};

// Type aliases for common sizes
using atomic_size_t = atomic_integral<size_t>;
using atomic_uint32_t = atomic_integral<uint32_t>;
using atomic_uint64_t = atomic_integral<uint64_t>;

} // namespace dsp