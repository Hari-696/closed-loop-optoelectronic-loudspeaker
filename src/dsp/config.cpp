/**
 * @file config.cpp
 * @brief Configuration constants - ensures symbols are emitted for linking.
 * 
 * All constexpr variables are defined in config.hpp. This file exists
 * only to satisfy the build system and provide a translation unit for
 * any non-constexpr symbols that might be needed.
 */

#include "dsp/config.hpp"

namespace dsp {

// Force instantiation of constexpr variables for linking (if needed)
// Note: constexpr variables don't need separate definition in C++17+
// This file is kept for build system completeness.

static constexpr int dummy = 0;

} // namespace dsp