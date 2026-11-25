cmake_minimum_required(VERSION 3.16)
project(ClosedLoopOptoelectronicLoudspeaker
    VERSION 1.0.0
    LANGUAGES C CXX ASM
    DESCRIPTION "Closed-Loop Optoelectronically Linearized Loudspeaker System"
)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Warning flags
if(MSVC)
    add_compile_options(/W4 /WX /permissive-)
else()
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
        -Werror
        -Wshadow
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wcast-align
        -Wunused
        -Woverloaded-virtual
        -Wdouble-promotion
        -Wformat=2
        -Wnull-dereference
        # Disable these for now due to implicit conversions in DSP math
        -Wno-conversion
        -Wno-sign-conversion
    )
endif()

# Optimization for release
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -DNDEBUG")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O2 -DNDEBUG")

# ARM Cortex-M7 toolchain (optional cross-compilation)
if(DEFINED ENV{ARM_TOOLCHAIN})
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/arm-cortex-m7.cmake)
    message(STATUS "Using ARM Cortex-M7 toolchain")
else()
    message(STATUS "Building for host (x86_64) - shared library target")
endif()

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

# Find PostgreSQL libpq for telemetry_logger
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(LIBPQ libpq)
endif()
if(NOT LIBPQ_FOUND)
    find_library(LIBPQ_LIBRARY pq)
    find_path(LIBPQ_INCLUDE_DIR libpq-fe.h)
    if(LIBPQ_LIBRARY AND LIBPQ_INCLUDE_DIR)
        set(LIBPQ_FOUND TRUE)
        set(LIBPQ_LIBRARIES ${LIBPQ_LIBRARY})
        set(LIBPQ_INCLUDE_DIRS ${LIBPQ_INCLUDE_DIR})
    endif()
endif()

# --- dsp_core library ---
add_library(dsp_core STATIC
    src/dsp/config.cpp
    src/dsp/fixed_point.cpp
    src/dsp/cubic_spline_lut.cpp
    src/dsp/luenberger_observer.cpp
    src/dsp/mfb_controller.cpp
    src/dsp/volterra_filter.cpp
    src/dsp/decoupling_limiter.cpp
    src/dsp/lsp_estimator.cpp
    src/dsp/dsp_pipeline.cpp
    src/telemetry/spsc_ring_buffer.cpp
)

target_include_directories(dsp_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(dsp_core PRIVATE
    DSP_CORE_EXPORTS
    DSP_FIXED_POINT=1
)

# ARM Cortex-M7 specific flags
if(DEFINED ENV{ARM_TOOLCHAIN})
    target_compile_options(dsp_core PRIVATE
        -mcpu=cortex-m7
        -mfpu=fpv5-d16
        -mfloat-abi=hard
        -mthumb
        -ffunction-sections
        -fdata-sections
        -fno-exceptions
        -fno-rtti
        -fno-threadsafe-statics
        -fno-use-cxa-atexit
    )
    target_link_options(dsp_core PRIVATE
        -Wl,--gc-sections
        -specs=nano.specs
        -specs=nosys.specs
    )
else()
    # Host build - export C ABI for shared library loading
    target_compile_options(dsp_core PRIVATE -fvisibility=hidden)
endif()

# --- dsp_core shared library (host) ---
if(NOT DEFINED ENV{ARM_TOOLCHAIN})
    add_library(dsp_core_shared SHARED
        src/dsp/config.cpp
        src/dsp/fixed_point.cpp
        src/dsp/cubic_spline_lut.cpp
        src/dsp/luenberger_observer.cpp
        src/dsp/mfb_controller.cpp
        src/dsp/volterra_filter.cpp
        src/dsp/decoupling_limiter.cpp
        src/dsp/lsp_estimator.cpp
        src/dsp/dsp_pipeline.cpp
        src/telemetry/spsc_ring_buffer.cpp
    )
    target_include_directories(dsp_core_shared PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
    target_compile_definitions(dsp_core_shared PRIVATE
        DSP_CORE_EXPORTS
        DSP_FIXED_POINT=1
    )
    target_compile_options(dsp_core_shared PRIVATE -fvisibility=hidden)
    set_target_properties(dsp_core_shared PROPERTIES
        OUTPUT_NAME "dsp_core"
        SOVERSION 1
        VERSION 1.0.0
    )
endif()

# --- telemetry_logger executable ---
if(LIBPQ_FOUND)
    add_executable(telemetry_logger
        src/telemetry/pg_logger.cpp
        src/telemetry/spsc_ring_buffer.cpp
    )
    target_include_directories(telemetry_logger PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${LIBPQ_INCLUDE_DIRS}
    )
    target_link_libraries(telemetry_logger
        dsp_core
        ${LIBPQ_LIBRARIES}
        pthread
    )
    target_compile_definitions(telemetry_logger PRIVATE
        TELEMETRY_LOGGER_EXPORTS
    )
else()
    message(WARNING "libpq not found - telemetry_logger will not be built")
endif()

# --- Installation ---
install(TARGETS dsp_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
if(NOT DEFINED ENV{ARM_TOOLCHAIN})
    install(TARGETS dsp_core_shared
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )
endif()
if(LIBPQ_FOUND)
    install(TARGETS telemetry_logger
        RUNTIME DESTINATION bin
    )
endif()
install(DIRECTORY include/ DESTINATION include)
install(FILES sql/schema.sql sql/queries.sql DESTINATION share/closed_loop_loudspeaker/sql)

# --- Testing (external only - no test targets per constraints) ---
# Verification is performed externally in dedicated simulation tools

# --- Packaging ---
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)