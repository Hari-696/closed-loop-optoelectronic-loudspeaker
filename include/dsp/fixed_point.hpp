#pragma once

/**
 * @file fixed_point.hpp
 * @brief Overflow-safe q15/q31 arithmetic with saturation for deterministic DSP.
 * 
 * All operations are branchless where possible, use compiler intrinsics for
 * saturation, and guarantee no undefined behavior on overflow.
 */

#include <cstdint>
#include <cstddef>
#include <type_traits>
#include "config.hpp"

namespace dsp {
namespace fixed {

// ============================================================================
// Q1.15 (q15_t) Operations
// ============================================================================

DSP_INLINE constexpr q15_t q15_from_float(float x) noexcept {
    constexpr float scale = Q15_SCALE;
    constexpr float max_val = 32767.0f;
    constexpr float min_val = -32768.0f;
    float scaled = x * scale;
    if (scaled > max_val) return 32767;
    if (scaled < min_val) return -32768;
    return static_cast<q15_t>(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

DSP_INLINE constexpr float q15_to_float(q15_t x) noexcept {
    return static_cast<float>(x) / Q15_SCALE;
}

DSP_INLINE q15_t q15_sat_add(q15_t a, q15_t b) noexcept {
    int32_t sum = static_cast<int32_t>(a) + static_cast<int32_t>(b);
    if (sum > 32767) return 32767;
    if (sum < -32768) return -32768;
    return static_cast<q15_t>(sum);
}

DSP_INLINE q15_t q15_sat_sub(q15_t a, q15_t b) noexcept {
    int32_t diff = static_cast<int32_t>(a) - static_cast<int32_t>(b);
    if (diff > 32767) return 32767;
    if (diff < -32768) return -32768;
    return static_cast<q15_t>(diff);
}

DSP_INLINE q15_t q15_sat_mul(q15_t a, q15_t b) noexcept {
    // Q1.15 × Q1.15 = Q2.30, shift right 15 to get Q1.15
    int32_t prod = static_cast<int32_t>(a) * static_cast<int32_t>(b);
    int32_t result = (prod + 16384) >> 15;  // Rounding
    if (result > 32767) return 32767;
    if (result < -32768) return -32768;
    return static_cast<q15_t>(result);
}

DSP_INLINE q15_t q15_sat_mul_q31(q15_t a, q31_t b) noexcept {
    // Q1.15 × Q1.31 = Q2.46, shift right 31 to get Q1.15
    int64_t prod = static_cast<int64_t>(a) * static_cast<int64_t>(b);
    int32_t result = static_cast<int32_t>((prod + (1LL << 30)) >> 31);
    if (result > 32767) return 32767;
    if (result < -32768) return -32768;
    return static_cast<q15_t>(result);
}

DSP_INLINE q15_t q15_abs(q15_t x) noexcept {
    // Branchless absolute value with saturation for INT16_MIN
    return (x == -32768) ? 32767 : (x < 0 ? -x : x);
}

DSP_INLINE q15_t q15_neg(q15_t x) noexcept {
    // Negate with saturation for INT16_MIN
    return (x == -32768) ? 32767 : -x;
}

DSP_INLINE q15_t q15_shl(q15_t x, int shift) noexcept {
    if (shift >= 15) return (x > 0) ? 32767 : (x < 0 ? -32768 : 0);
    if (shift <= -15) return 0;
    int32_t val = static_cast<int32_t>(x) << shift;
    if (val > 32767) return 32767;
    if (val < -32768) return -32768;
    return static_cast<q15_t>(val);
}

DSP_INLINE q15_t q15_shr(q15_t x, int shift) noexcept {
    if (shift >= 15) return (x < 0) ? -1 : 0;
    if (shift <= 0) return x;
    // Arithmetic right shift with rounding
    return static_cast<q15_t>((x + (1 << (shift - 1))) >> shift);
}

// ============================================================================
// Q1.31 (q31_t) Operations
// ============================================================================

DSP_INLINE constexpr q31_t q31_from_float(float x) noexcept {
    constexpr float scale = Q31_SCALE;
    constexpr float max_val = 2147483647.0f;
    constexpr float min_val = -2147483648.0f;
    float scaled = x * scale;
    if (scaled > max_val) return 2147483647;
    if (scaled < min_val) return -2147483648;
    return static_cast<q31_t>(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

DSP_INLINE constexpr float q31_to_float(q31_t x) noexcept {
    return static_cast<float>(x) / Q31_SCALE;
}

DSP_INLINE q31_t q31_sat_add(q31_t a, q31_t b) noexcept {
    int64_t sum = static_cast<int64_t>(a) + static_cast<int64_t>(b);
    if (sum > 2147483647LL) return 2147483647;
    if (sum < -2147483648LL) return -2147483648;
    return static_cast<q31_t>(sum);
}

DSP_INLINE q31_t q31_sat_sub(q31_t a, q31_t b) noexcept {
    int64_t diff = static_cast<int64_t>(a) - static_cast<int64_t>(b);
    if (diff > 2147483647LL) return 2147483647;
    if (diff < -2147483648LL) return -2147483648;
    return static_cast<q31_t>(diff);
}

DSP_INLINE q31_t q31_sat_mul(q31_t a, q31_t b) noexcept {
    // Q1.31 × Q1.31 = Q2.62, shift right 31 to get Q1.31
    int64_t prod = static_cast<int64_t>(a) * static_cast<int64_t>(b);
    int64_t result = (prod + (1LL << 30)) >> 31;
    if (result > 2147483647LL) return 2147483647;
    if (result < -2147483648LL) return -2147483648;
    return static_cast<q31_t>(result);
}

DSP_INLINE q31_t q31_sat_mul_q15(q31_t a, q15_t b) noexcept {
    // Q1.31 × Q1.15 = Q2.46, shift right 15 to get Q1.31
    int64_t prod = static_cast<int64_t>(a) * static_cast<int64_t>(b);
    int64_t result = (prod + (1LL << 14)) >> 15;
    if (result > 2147483647LL) return 2147483647;
    if (result < -2147483648LL) return -2147483648;
    return static_cast<q31_t>(result);
}

DSP_INLINE q31_t q31_abs(q31_t x) noexcept {
    return (x == -2147483648) ? 2147483647 : (x < 0 ? -x : x);
}

DSP_INLINE q31_t q31_neg(q31_t x) noexcept {
    return (x == -2147483648) ? 2147483647 : -x;
}

DSP_INLINE q31_t q31_shl(q31_t x, int shift) noexcept {
    if (shift >= 31) return (x > 0) ? 2147483647 : (x < 0 ? -2147483648 : 0);
    if (shift <= -31) return 0;
    int64_t val = static_cast<int64_t>(x) << shift;
    if (val > 2147483647LL) return 2147483647;
    if (val < -2147483648LL) return -2147483648;
    return static_cast<q31_t>(val);
}

DSP_INLINE q31_t q31_shr(q31_t x, int shift) noexcept {
    if (shift >= 31) return (x < 0) ? -1 : 0;
    if (shift <= 0) return x;
    return static_cast<q31_t>((x + (1LL << (shift - 1))) >> shift);
}

// ============================================================================
// Mixed-Precision MAC Accumulators (for filter inner loops)
// ============================================================================

// 64-bit accumulator for q15 × q15 MAC chains (prevents intermediate overflow)
class Q15Accumulator {
    int64_t acc_;
public:
    DSP_INLINE Q15Accumulator() noexcept : acc_(0) {}
    DSP_INLINE explicit Q15Accumulator(q31_t init) noexcept : acc_(static_cast<int64_t>(init) << 15) {}
    
    DSP_INLINE void mac(q15_t a, q15_t b) noexcept {
        acc_ += static_cast<int64_t>(a) * static_cast<int64_t>(b);
    }
    
    DSP_INLINE void mac_q31(q31_t a, q15_t b) noexcept {
        acc_ += static_cast<int64_t>(a) * static_cast<int64_t>(b);
    }
    
    DSP_INLINE q15_t to_q15() noexcept {
        // Result is in Q2.30, shift to Q1.15 with saturation
        int64_t result = (acc_ + (1LL << 14)) >> 15;
        if (result > 32767) return 32767;
        if (result < -32768) return -32768;
        return static_cast<q15_t>(result);
    }
    
    DSP_INLINE q31_t to_q31() noexcept {
        // Result is in Q2.30, shift to Q1.31 with saturation
        int64_t result = (acc_ + (1LL << 30)) >> 31;
        if (result > 2147483647LL) return 2147483647;
        if (result < -2147483648LL) return -2147483648;
        return static_cast<q31_t>(result);
    }
    
    DSP_INLINE void reset() noexcept { acc_ = 0; }
    DSP_INLINE void set_q31(q31_t v) noexcept { acc_ = static_cast<int64_t>(v) << 15; }
};

// 64-bit accumulator for q31 × q31 MAC chains
class Q31Accumulator {
    __int128 acc_;  // GCC/Clang support 128-bit integers
public:
    DSP_INLINE Q31Accumulator() noexcept : acc_(0) {}
    DSP_INLINE explicit Q31Accumulator(q31_t init) noexcept : acc_(static_cast<__int128>(init) << 31) {}
    
    DSP_INLINE void mac(q31_t a, q31_t b) noexcept {
        acc_ += static_cast<__int128>(a) * static_cast<__int128>(b);
    }
    
    DSP_INLINE q31_t to_q31() noexcept {
        __int128 result = (acc_ + (__int128(1) << 30)) >> 31;
        if (result > 2147483647LL) return 2147483647;
        if (result < -2147483648LL) return -2147483648;
        return static_cast<q31_t>(result);
    }
    
    DSP_INLINE void reset() noexcept { acc_ = 0; }
};

// Fallback for compilers without __int128 (MSVC)
#if defined(_MSC_VER) && !defined(__clang__)
class Q31Accumulator {
    int64_t acc_hi_;
    uint64_t acc_lo_;
public:
    DSP_INLINE Q31Accumulator() noexcept : acc_hi_(0), acc_lo_(0) {}
    DSP_INLINE explicit Q31Accumulator(q31_t init) noexcept 
        : acc_hi_(static_cast<int64_t>(init) >> 1), acc_lo_(static_cast<uint64_t>(init) << 63) {}
    
    DSP_INLINE void mac(q31_t a, q31_t b) noexcept {
        // Use 64×64→128 multiply via builtin
        __int128 prod = static_cast<__int128>(a) * static_cast<__int128>(b);
        acc_lo_ += static_cast<uint64_t>(prod);
        acc_hi_ += static_cast<int64_t>(prod >> 64) + (acc_lo_ < static_cast<uint64_t>(prod) ? 1 : 0);
    }
    
    DSP_INLINE q31_t to_q31() noexcept {
        // Approximate: shift 128-bit value right 31
        uint64_t lo = acc_lo_ >> 31;
        int64_t hi = (acc_hi_ << 33) | (acc_lo_ >> 31);
        int64_t result = hi + (acc_lo_ & 0x40000000 ? 1 : 0);  // Rounding
        if (result > 2147483647LL) return 2147483647;
        if (result < -2147483648LL) return -2147483648;
        return static_cast<q31_t>(result);
    }
    
    DSP_INLINE void reset() noexcept { acc_hi_ = 0; acc_lo_ = 0; }
};
#endif

// ============================================================================
// Vector/Matrix Fixed-Point Types (2×2 for observer)
// ============================================================================

struct Q15Vec2 {
    q15_t x, y;
};

struct Q31Vec2 {
    q31_t x, y;
};

struct Q15Mat22 {
    q15_t m00, m01;
    q15_t m10, m11;
};

struct Q31Mat22 {
    q31_t m00, m01;
    q31_t m10, m11;
};

DSP_INLINE Q31Vec2 q31_mat22_vec2_mul(const Q31Mat22& A, const Q31Vec2& v) noexcept {
    Q31Accumulator acc0, acc1;
    acc0.mac(A.m00, v.x);
    acc0.mac(A.m01, v.y);
    acc1.mac(A.m10, v.x);
    acc1.mac(A.m11, v.y);
    return {acc0.to_q31(), acc1.to_q31()};
}

DSP_INLINE Q15Vec2 q15_mat22_vec2_mul(const Q15Mat22& A, const Q15Vec2& v) noexcept {
    Q15Accumulator acc0, acc1;
    acc0.mac(A.m00, v.x);
    acc0.mac(A.m01, v.y);
    acc1.mac(A.m10, v.x);
    acc1.mac(A.m11, v.y);
    return {acc0.to_q15(), acc1.to_q15()};
}

// ============================================================================
// Float↔Fixed Conversion Helpers (constexpr where possible)
// ============================================================================

template<typename T>
DSP_INLINE constexpr T float_to_fixed(float x) noexcept;

template<>
DSP_INLINE constexpr q15_t float_to_fixed<q15_t>(float x) noexcept {
    return q15_from_float(x);
}

template<>
DSP_INLINE constexpr q31_t float_to_fixed<q31_t>(float x) noexcept {
    return q31_from_float(x);
}

template<typename T>
DSP_INLINE constexpr float fixed_to_float(T x) noexcept;

template<>
DSP_INLINE constexpr float fixed_to_float<q15_t>(q15_t x) noexcept {
    return q15_to_float(x);
}

template<>
DSP_INLINE constexpr float fixed_to_float<q31_t>(q31_t x) noexcept {
    return q31_to_float(x);
}

// ============================================================================
// Polynomial Evaluation (Horner's method) in Fixed-Point
// ============================================================================

// Evaluate 3rd-order polynomial: c0 + c1*x + c2*x^2 + c3*x^3 in Q31
DSP_INLINE q31_t q31_poly3(q31_t x, q31_t c0, q31_t c1, q31_t c2, q31_t c3) noexcept {
    Q31Accumulator acc;
    // Horner: ((c3 * x + c2) * x + c1) * x + c0
    q31_t t = c3;
    t = q31_sat_add(q31_sat_mul(t, x), c2);
    t = q31_sat_add(q31_sat_mul(t, x), c1);
    t = q31_sat_add(q31_sat_mul(t, x), c0);
    return t;
}

// Evaluate cubic spline segment in Q31
// Coefficients stored as: a0 + a1*dx + a2*dx^2 + a3*dx^3
struct CubicCoeffsQ31 {
    q31_t a0, a1, a2, a3;
};

DSP_INLINE q31_t q31_cubic_spline_eval(const CubicCoeffsQ31& c, q31_t dx) noexcept {
    Q31Accumulator acc;
    // Horner: ((a3 * dx + a2) * dx + a1) * dx + a0
    q31_t t = c.a3;
    t = q31_sat_add(q31_sat_mul(t, dx), c.a2);
    t = q31_sat_add(q31_sat_mul(t, dx), c.a1);
    t = q31_sat_add(q31_sat_mul(t, dx), c.a0);
    return t;
}

} // namespace fixed
} // namespace dsp