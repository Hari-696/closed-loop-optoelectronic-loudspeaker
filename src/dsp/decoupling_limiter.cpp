/**
 * @file decoupling_limiter.cpp
 * @brief Decoupling limiter - all implementation in header.
 * 
 * This file exists only to satisfy the build system.
 * All implementation is in decoupling_limiter.hpp as inline functions.
 */

#include "dsp/decoupling_limiter.hpp"

namespace dsp {

// Global state instance
LimiterState g_limiter_state{};

// Prevent empty translation unit warning
static constexpr int dummy = 0;

} // namespace dsp