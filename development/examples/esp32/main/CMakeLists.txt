# =============================================================================
# TLE92466ED Driver Examples - Main Component CMakeLists.txt
# =============================================================================
# This CMakeLists.txt is designed to work with the build_app.sh script
# which handles app type validation and source file discovery.
# =============================================================================

# =============================================================================
# Variable Validation and Defaults
# =============================================================================
if(NOT DEFINED APP_TYPE)
    if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
        set(APP_TYPE "${DEFAULT_APP_TYPE}")
        message(STATUS "APP_TYPE not defined during requirements phase, using default: ${APP_TYPE}")
    else()
        message(FATAL_ERROR 
            "APP_TYPE not defined. Please use build_app.sh to build this project.\n"
            "Example: ./scripts/build_app.sh ${DEFAULT_APP_TYPE} Release\n"
            "Use './scripts/build_app.sh list' to see available app types."
        )
    endif()
endif()

if(NOT DEFINED APP_SOURCE_FILE)
    if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
        set(APP_SOURCE_FILE "${DEFAULT_SOURCE_FILE}")
        message(STATUS "APP_SOURCE_FILE not defined during requirements phase, using default: ${APP_SOURCE_FILE}")
    else()
        message(FATAL_ERROR 
            "APP_SOURCE_FILE not defined. Please use build_app.sh to build this project.\n"
            "Example: ./scripts/build_app.sh ${DEFAULT_APP_TYPE} ${DEFAULT_BUILD_TYPE}\n"
            "Use './scripts/build_app.sh list' to see available app types."
        )
    endif()
endif()

# =============================================================================
# Build Configuration and Status Messages
# =============================================================================
message(STATUS "Building app type: ${APP_TYPE}")
message(STATUS "Using source file: ${APP_SOURCE_FILE}")

# =============================================================================
# Include Directories Configuration
# =============================================================================
set(INC_BASE_PATH "../../../inc")

set(MAIN_INCLUDE_DIRS
    "."
    "${INC_BASE_PATH}"
)

# =============================================================================
# Component Dependencies
# =============================================================================
set(MAIN_REQUIRES
    driver
    esp_timer
    freertos
    spi_flash
    nvs_flash
    hf_tle92466ed
)

# Add ADC component for solenoid control test
if(APP_TYPE STREQUAL "solenoid_control_test")
    list(APPEND MAIN_REQUIRES esp_adc)
endif()

# =============================================================================
# Component Registration
# =============================================================================
idf_component_register(
    SRCS "${APP_SOURCE_FILE}"
    INCLUDE_DIRS ${MAIN_INCLUDE_DIRS}
    REQUIRES ${MAIN_REQUIRES}
)

# =============================================================================
# Compiler Configuration
# =============================================================================
# TLE92466ED requires C++20 (uses tle::expected polyfill for C++23 std::expected)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_20)

# =============================================================================
# Build Type Specific Compiler Flags
# =============================================================================
if(BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${COMPONENT_LIB} PRIVATE
        -Wall -Wextra -Wpedantic -O0 -g3
        -DDEBUG -DTLE92466ED_DEBUG
    )
else()
    target_compile_options(${COMPONENT_LIB} PRIVATE
        -Wall -Wextra -Wpedantic -O2 -g
        -DNDEBUG -DTLE92466ED_RELEASE
    )
endif()

# =============================================================================
# Example Type Compile Definitions
# =============================================================================
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    "EXAMPLE_TYPE_${APP_TYPE}=1"
    "TLE92466ED_ESP32_S3=1"
)
